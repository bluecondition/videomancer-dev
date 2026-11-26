#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
GYRE v0.1.0 - build hook.  Writes gyre_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.onevolt_work/rb_gyre/imodel.py) imports the same tables.

Package contents:
  C_GY_ARR   32 x 32    Color Chords layer colours {Y, U, V, 00}: arrangement a (0 .. 8) x layer l (0 = /4,
                        1 = /2, 2 = /1) at index 3 a + l; the eight arrangements are the six orders of one triple of
                        colours and three of a second triple
  C_GY_MROM  256 x 32   the per-mode constants of the ramp / gradient DDAs, the camera mean and the camera's
                        vertical slew: word 16 t + j, t = the timing ID (registers_in(8)), j = MROM_KEYS
  C_GY_PAL   1024 x 32  the palettes {Y, U, V, 00}: mode (Staircase / Doorway) x 512 + key level x 128 + n (see palettes)
  C_GY_K     the black (the Color Chords background and the compositor's B input); C_GY_WH the overdriven core
Colours are in the hardware convention (U register = Cr, V register = Cb).  Luma is pre-scaled 16/13 for the
shell's 13/16 output scale (capped at 940).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

K0 = (6, 4, 12)
WHITE = (255, 245, 230)
TRI_A = [(255, 20, 140), (255, 190, 0), (0, 90, 255)]       # magenta, amber, blue
TRI_B = [(0, 230, 120), (150, 0, 255), (255, 70, 0)]         # green, violet, orange
ORD_A = [(0, 1, 2), (1, 2, 0), (2, 0, 1), (0, 2, 1), (2, 1, 0), (1, 0, 2)]
ORD_B = [(0, 1, 2), (2, 0, 1), (1, 2, 0)]


def rgb_to_hw10f(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, int(round(u)), int(round(v)))


def word(yuv):
    y, u, v = yuv
    return (y << 22) | (u << 12) | (v << 2)


def wrgb(rgb):
    return word(prescale(rgb_to_hw10f(rgb)))


def arrangements():
    out = []
    for o in ORD_A:
        out.append([TRI_A[i] for i in o])
    for o in ORD_B:
        out.append([TRI_B[i] for i in o])
    return out


def arr_rom():
    """27 entries (9 arrangements x 3 layers), padded to 32 with black"""
    out = [wrgb(c) for a in arrangements() for c in a]
    return out + [wrgb(K0)] * (32 - len(out))


def trig_rom():
    out = []
    for k in range(512):
        a = (k + 0.5) * 2 * math.pi / 4096
        s = int(round(1023 * math.sin(a)))
        c = int(round(1023 * math.cos(a)))
        out.append((s << 16) | c)
    return out


def _tri(x):
    return 1.0 - abs(2.0 * (x - math.floor(x)) - 1.0)


def _clamp(x):
    return min(1.0, max(0.0, x))


def weights(mode):
    """per-entry slot weights (floats) of the Color Chords priority mix"""
    out = []
    for n in range(256):
        if mode == 0:
            w = (n + 0.5) / 64.0                      # Staircase phase, 4 bands per cycle
            k4, k2, k1 = [_clamp((_tri(w / d) - 0.5) * 1.6 + 0.5) for d in (4.0, 2.0, 1.0)]
            out.append((k4 * (1 - k2) * (1 - k1), k2 * (1 - k1), k1))
        else:
            ol = n / 255.0                            # Doorway outline level
            ka, kb, kw = _clamp(3 * ol), _clamp(3 * ol - 0.75), 0.625 * _clamp(3 * ol - 2)
            out.append((ka * (1 - kb) * (1 - kw), kb * (1 - kw), kw))
    return out


def wrom():
    out = []
    for m in (0, 1):
        for w in weights(m):
            q = [min(255, int(round(x * 256))) for x in w]
            out.append((q[0] << 24) | (q[1] << 16) | (q[2] << 8))
    return out


def mean_weights(mode):
    fa = 0.3 if mode == 0 else 0.15
    w = weights(mode)
    return [int(round(fa * 256 * sum(x[l] for x in w) / 256.0)) for l in range(3)]


KC = 1.6467602581210656            # the CORDIC gain (12 iterations), folded into the ramp constants


def mode_class(tid):
    """(name, xs, ys, sd, lines per field) for a timing ID, as the shell's f_vstep_x / f_vstep_y"""
    if tid == 0:
        return '480i', 4, 1138, True, 240
    if tid == 4:
        return '480p', 4, 576, True, 480
    if tid == 8:
        return '576i', 4, 960, True, 288
    if tid == 12:
        return '576p', 4, 480, True, 576
    if tid in (5, 6, 14):
        return '720p', 3, 384, False, 720
    if tid in (1, 2, 10):
        return '1080i', 2, 512, False, 540
    return '1080p', 2, 256, False, 1080


def mode_consts(tid):
    """per-mode constants.  Virtual coordinates: hx = 2 (x - 960) half px, yq = 256 (y - 540) (Q8 lines)."""
    nm, xs, ys, sd, lines = mode_class(tid)
    hx0 = -1440 if sd else -1920
    width = 720 if sd else (1280 if nm == '720p' else 1920)
    yq0 = -138240
    return dict(
        XS=xs, YS=ys, SD=1 if sd else 0,
        HX2=hx0 * hx0,                                   # pixel DDA start (x Aq)
        DX0C=-(2 * hx0 * xs + xs * xs),                  # its first difference, magnitude (the sign is -)
        DDC=2 * xs * xs,                                 # its second difference
        LD0C=(-(2 * yq0 * ys + ys * ys)) >> 8,           # line DDA first difference / 256, magnitude (sign -)
        LDDC=2 * ys * ys,                                # line DDA second difference
        RC=int(round(ys * (1 << 24) / 276480)),          # Ramp: line step of T yv / 1080, / 256
        DXC=int(round(34953 * xs / KC)),                 # ramp-gain x step (Q12 of 8.5333 xs), / the CORDIC gain
        DYC=int(round(273.0667 * ys / KC)),              # ramp-gain y step (Q12 of ys / 15), / the CORDIC gain
        X0C=int(round((12288 if sd else 16384) / KC)),   # the H ramp's start (x Nw), / the CORDIC gain
        Y0C=int(round(9216 / KC)),                       # the V ramp's start (x Nh), / the CORDIC gain
        R=int(round((1 << 24) / ((width // 8) * lines))),   # camera mean: 2^24 / cells per field
        VSH={'480i': 4, '576i': 4, '480p': 5, '576p': 5, '1080i': 5}.get(nm, 6),
    )


YQ2S = (138240 * 138240) >> 16      # line DDA start / 2^16 (x Aq)
MROM_KEYS = ["HX2", "DX0C", "DDC", "LD0C", "LDDC", "RC", "DXC", "DYC", "R", "VSH", "X0C", "Y0C"]


def mrom():
    """256 x 32: word 16 t + j = constant MROM_KEYS[j] of timing ID t (j >= 10: 0)"""
    out = []
    for t in range(16):
        mc = mode_consts(t)
        for j in range(16):
            out.append(mc[MROM_KEYS[j]] if j < len(MROM_KEYS) else 0)
    return out


def _w10(word):
    return [(word >> 22) & 1023, (word >> 12) & 1023, (word >> 2) & 1023]


def palettes():
    """the palette ROM (1024 words): index = mode (0 Staircase, 1 Doorway) x 512 + key level L x 128 + n: the Color
    Chords priority mix of the magenta / amber / blue layers (Doorway: magenta edge, amber core, white) at Staircase
    phase / outline level n (128 steps), darkened toward the black by L / 3 (the Visual Cortex key, baked in)"""
    K = _w10(wrgb(K0))
    cols = arrangements()[0]
    D0 = [[a - k for a, k in zip(_w10(wrgb(c)), K)] for c in cols]
    lo, hi = (D0[0], D0[1]) if D0[0][0] <= D0[1][0] else (D0[1], D0[0])
    D1 = [lo, hi, [a - k for a, k in zip(_w10(wrgb(WHITE)), K)]]
    pal = []
    for m, D in ((0, D0), (1, D1)):
        base = []
        for q in wrom()[256 * m: 256 * m + 256: 2]:
            w = [(q >> 24) & 255, (q >> 16) & 255, (q >> 8) & 255]
            base.append([K[ch] + sum((w[l] * D[l][ch]) >> 8 for l in range(3)) for ch in range(3)])
        for L in range(4):
            for c in base:
                pal.append(word([K[ch] + ((c[ch] - K[ch]) * (3 - L)) // 3 for ch in range(3)]))
    return pal


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by gyre.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package gyre_pkg is",
             vhd_table("C_GY_ARR", "std_logic_vector", 32, 32, arr_rom()),
             vhd_table("C_GY_MROM", "std_logic_vector", 256, 32, mrom()),
             vhd_table("C_GY_PAL", "std_logic_vector", 1024, 32, palettes()),
             '    constant C_GY_K : std_logic_vector(31 downto 0) := x"%08X";' % wrgb(K0),
             '    constant C_GY_WH : std_logic_vector(31 downto 0) := x"%08X";' % wrgb(WHITE),
             "end package gyre_pkg;", ""]
    with open(os.path.join(HERE, "gyre_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
