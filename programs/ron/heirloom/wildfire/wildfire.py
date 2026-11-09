#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
WILDFIRE v0.1.1 - build hook.  Writes wildfire_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  imodel.py (.heirloom_work/rb_wildfire/) imports the same tables.

The patch, as read from clip WIN_20230509_23_58_21 (0.5 s; see .heirloom_work/rb_wildfire/notes.md):
  phi_v(x, y) = G * (c2(x) Y^2 + c1(x) Y + c0(x)) - drift     V oscillator: its frequency and phase set per column
                                                              by the H ramp (knot table, 64 px), Y = the soft-clipped
                                                              V ramp (slope 1 to line 450, then easing to 0.12)
  phi_h(x, y) = 1.5 + Ge * (h(x) - 1.5 + s(x) y) - drift      H oscillator: a slewed staircase (the egg column, the
                                                              eye gaps, the long fins, the fin boundary tilting in)
  phi_w(x, y) = (y cos20 - x sin20) / 22 - drift              fine oscillator, VCA'd in from line ~520 (the lace)
  F = cos phi_h + cos phi_v + aw(y) cos phi_w - lake(y)
  R = ROM_R[F - thr - d],  G = ROM_G[F - thr + 0.6 d]          high-gain colouriser: black (20,41,12) -> yellow
                                                              (240,248,5); d = R/G threshold split (green fringes
                                                              on the right fan, orange around the eggs, red in the
                                                              lake); a thin olive notch high up (the "eyes")
  B = 12 + 110 sigmoid(key(x, y) + 1.8 aw cos phi_w)          the blue key under a wavy line (the lake)
  x < 112 virtual px: a per-line strip (the colouriser fed by a slow vertical oscillator during the H clamp)

Tables:
  C_WF_COS   512 x 8 signed     round(127 cos(2 pi i / 512))
  C_WF_RCOL  512 x 24           R contributions / 4 {Y 8 (pre-scaled 16/13), Cr 8 signed, Cb 8 signed}, index = u/2 + 256
  C_WF_GCOL  512 x 32           G contributions, same layout
  C_WF_BCOL  32 x 32            B contributions by key level {Y 10, Cr 11 signed, Cb 11 signed}
  C_WF_KNOT  256 x 16 signed    knot constants: word 8k + {0 A, 1 B, 2 E, 3 S, 4 HB, 5 DX, 6 BK, 7 TZ}
  C_WF_EXP2  256 x 16           round(32768 * 2^(i/256))
Colours in the hardware convention (U register = Cr, V register = Cb).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model
KX = [64 * k for k in range(31)]
# V oscillator per knot: (c2 rad/line^2, c1 rad/line, c0 rad) at the photo, interpolated over these points
V_PTS = {0: (0, 0.0044, 1.94), 128: (0, 0.0044, 1.94), 192: (0, 0.0046, 1.94), 256: (0, 0.0065, 1.94),
         320: (0, 0.0062, 2.0), 384: (0, 0.0170, 3.308),
         512: (0, 0.0230, 4.844), 640: (0, 0.0340, 6.380), 768: (0, 0.0580, 7.916), 896: (0, 0.0860, 9.452),
         960: (-5.8e-5, 0.0917, 10.25), 1024: (-3.8e-5, 0.0899, 9.98), 1216: (-0.8e-5, 0.0821, 11.11),
         1280: (1.1e-5, 0.0764, 12.18), 1408: (-2.8e-5, 0.1047, 12.50), 1472: (-3.5e-5, 0.1146, 13.94),
         1536: (-4.5e-5, 0.1230, 16.80), 1600: (-4.5e-5, 0.131, 20.07), 1664: (-4.5e-5, 0.139, 23.6),
         1728: (-4.5e-5, 0.147, 27.07), 1920: (-4.5e-5, 0.17, 37.6)}
# H oscillator per knot: (tilt rad/line, phase in units of pi)
H_PTS = {0: (0, 1.0), 330: (0, 1.0), 450: (0, 1.25), 600: (0, 1.0), 790: (0, 1.0), 870: (0, 2.0), 1000: (0, 3.0),
         1100: (0, 4.0), 1230: (0, 5.0), 1500: (0, 5.0), 1600: (-0.0015, 5.0), 1700: (-0.0025, 4.7),
         1800: (-0.002, 4.2), 1920: (0, 4.0)}
D_PTS = {0: -0.15, 1000: -0.15, 1250: 0.30, 1920: 0.35}           # R/G split (F units) above line 450
BK_PTS = {0: 540, 128: 540, 256: 580, 384: 680, 512: 720, 640: 650, 704: 570, 1100: 570, 1216: 700,
          1300: 780, 1600: 820, 1920: 870}   # blue key line (virtual lines)
# navy patches: per-knot darkness (key units above the key's saturation), applied in a line window 520 .. 690
TZ_PTS = {0: 0, 576: 0, 768: 127, 1024: 127, 1216: 0, 1920: 0}
# V ramp: Y = y to line 450, then the slope eases 1 -> 0.45 (560) -> 0.18 (700) -> 0.12 (1080)
Y_SEG = [(450, 450.0, 1.0, 0.0025), (560, 529.75, 0.45, 0.27 / 280.0), (700, 573.85, 0.18, 0.06 / 760.0)]


def interp(x, pts):
    ks = sorted(pts)
    if x <= ks[0]:
        return pts[ks[0]]
    for a, b in zip(ks, ks[1:]):
        if x <= b:
            f = (x - a) / float(b - a)
            pa, pb = pts[a], pts[b]
            if isinstance(pa, tuple):
                return tuple(pa[i] + f * (pb[i] - pa[i]) for i in range(len(pa)))
            return pa + f * (pb - pa)
    return pts[ks[-1]]


def knot_words():
    """per knot k: A = c2 * 64 / 2pi * 2^24 (x Y2 >> 12 -> Q12), B = c1 / (4 * 2pi) * 2^20 (x Y_Q2 >> 8),
    E = c0 / 2pi * 4096, S = tilt / 2pi * 2^20 (x y >> 8), HB = (h - 3) / 2 * 4096 (cycles about 1.5),
    DX = d * 64, BK = key line, TZ = navy-patch depth"""
    out = []
    for k in range(32):
        x = min(64 * k, 1920)
        c2, c1, c0 = interp(x, V_PTS)
        s, h = interp(x, H_PTS)
        w = [round(c2 * 64 / (2 * math.pi) * 2 ** 24), round(c1 / (8 * math.pi) * 2 ** 20),
             round(c0 / (2 * math.pi) * 4096), round(s / (2 * math.pi) * 2 ** 20), round((h - 3) / 2.0 * 4096),
             round(interp(x, D_PTS) * 64), round(interp(x, BK_PTS)), round(interp(x, TZ_PTS))]
        for v in w:
            assert -32768 <= v < 32768, (k, w)
        out += w
    return out + [0] * (256 - len(out))


def sig(z):
    return 1.0 / (1.0 + math.exp(-z))


def contrib(c, which):
    """(Y pre-scaled 16/13, Cr, Cb) contribution of one channel level c (0..255) in 10-bit units"""
    ky = {'r': 0.299, 'g': 0.587, 'b': 0.114}[which]
    kr = {'r': 0.5, 'g': -0.418688, 'b': -0.081312}[which]
    kb = {'r': -0.168736, 'g': -0.331264, 'b': 0.5}[which]
    y = ky * c / 255.0 * 876 * 16.0 / 13.0
    return int(round(y)), int(round(kr * c / 255.0 * 896)), int(round(kb * c / 255.0 * 896))


def pack(t):
    y, cr, cb = t
    return ((y & 0x3FF) << 22) | ((cr & 0x7FF) << 11) | (cb & 0x7FF)


EYE_U, EYE_W = 2.62, 0.05
def r_level(u):
    e = math.exp(-((u - EYE_U) / EYE_W) ** 2)
    return 20 + 220 * sig(6 * u) * (1 - 0.28 * e)


def g_level(u):
    e = math.exp(-((u - EYE_U) / EYE_W) ** 2)
    return 41 + 207 * sig(6 * u) * (1 - 0.18 * e)


def pack24(t):
    """(Y, Cr, Cb) contribution / 4, rounded: {Y 8 unsigned, Cr 8 signed, Cb 8 signed}"""
    y, cr, cb = [int(round(v / 4.0)) for v in t]
    return ((y & 0xFF) << 16) | ((cr & 0xFF) << 8) | (cb & 0xFF)


def col_rom(fn, which):
    # index i: u = (i - 256) * 2 / 127 F units
    return [pack24(contrib(fn((i - 256) * 2 / 127.0), which)) for i in range(512)]


def b_rom():
    # index i: key value kv = (i - 16) / 4 sigmoid units
    return [pack(contrib(12 + 110 * sig((i - 16) / 4.0), 'b')) for i in range(32)]


def cos_rom():
    return [int(round(127 * math.cos(2 * math.pi * i / 512))) for i in range(512)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def black():
    """palette black: R ROM entry 0 + G ROM entry 0 + B ROM entry 0 (key off), as the output path sums it"""
    def unp24(w):
        y, cr, cb = (w >> 16) & 0xFF, (w >> 8) & 0xFF, w & 0xFF
        return 4 * y, 4 * (cr - 256 if cr >= 128 else cr), 4 * (cb - 256 if cb >= 128 else cb)
    def unp(w):
        y, cr, cb = (w >> 22) & 0x3FF, (w >> 11) & 0x7FF, w & 0x7FF
        return y, cr - 2048 if cr >= 1024 else cr, cb - 2048 if cb >= 1024 else cb
    r, g, b = unp24(col_rom(r_level, 'r')[0]), unp24(col_rom(g_level, 'g')[0]), unp(b_rom()[0])
    return 64 + r[0] + g[0] + b[0], 512 + r[1] + g[1] + b[1], 512 + r[2] + g[2] + b[2]


def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        else:
            items.append('x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width))
    per = 8 if signed else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by wildfire.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package wildfire_pkg is",
             vhd_table("C_WF_COS", "signed", 512, 8, cos_rom(), signed=True),
             vhd_table("C_WF_RCOL", "std_logic_vector", 512, 24, col_rom(r_level, 'r')),
             vhd_table("C_WF_GCOL", "std_logic_vector", 512, 24, col_rom(g_level, 'g')),
             vhd_table("C_WF_BCOL", "std_logic_vector", 32, 32, b_rom()),
             vhd_table("C_WF_KNOT", "signed", 256, 16, knot_words(), signed=True),
             vhd_table("C_WF_EXP2", "unsigned", 256, 16, exp2_rom()),
             "    constant C_WF_KY : integer := %d;" % black()[0],
             "    constant C_WF_KU : integer := %d;" % black()[1],
             "    constant C_WF_KV : integer := %d;" % black()[2],
             "end package wildfire_pkg;", ""]
    with open(os.path.join(HERE, "wildfire_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
