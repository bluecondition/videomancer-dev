#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FLYBACK v0.1.0 - build hook.  Writes flyback_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.onevolt_work/rb_flyback/imodel.py) imports the same tables and constants.

Package contents:
  C_FB_EXP   256 x 16   round(4096 * 2^(i / 256)): the shared exp2 mantissa (knob laws in the zone, the
                        oscillator frequency per pixel under the Frequency CV)
  C_FB_PAL   1024 x 32  {Y, U, V, 00} colouriser tables (the zone's palette writer crossfades two of them through
                        the Pedestal + Gain stage into a 256-entry live palette indexed by the faded triangle): index = s10 * 512 + setting * 64 + level
                        s10 = 0: Color Chords, 8 colourings (three layers: triangle, parabolic, exponential, each
                                 with an RGB colour and an opacity);
                        s10 = 1: a second Staircase fed by the triangle, /1 /2 /4 into R G B, 8 Frequency settings
                        level = the shaped Prismatic Ray output y (0 .. 1023) >> 4
  C_FB_K     black (outside the active picture)
  per-mode constants (functions of the timing ID): the pixel / line DDA coefficients of the ramp, the diamond and
  the ellipse in Q30 fold periods.
Colours are in the hardware convention (U register = Cr, V register = Cb).  Luma is pre-scaled 16/13 for the
shell's 13/16 output scale (capped at 940).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Color Chords colourings: (C1 triangle layer, C2 parabolic layer, C3 exponential layer, opacities o1 o2 o3)
COL = [
    ((1.00, 0.00, 0.65), (0.10, 0.15, 0.95), (1.00, 0.55, 0.05), (0.9, 0.0, 0.9)),   # prism: blue, magenta, orange
    ((1.00, 0.12, 0.00), (0.45, 0.00, 0.75), (1.00, 0.85, 0.10), (0.9, 0.0, 0.9)),   # sunset: violet, red, yellow
    ((0.00, 0.80, 1.00), (0.00, 0.10, 0.55), (0.90, 0.95, 1.00), (0.9, 0.0, 0.9)),   # ice: navy, cyan, white
    ((1.00, 0.00, 0.70), (0.00, 0.70, 0.10), (1.00, 0.95, 0.10), (0.9, 0.0, 0.9)),   # acid: green, magenta, yellow
    ((1.00, 0.00, 0.00), (0.00, 0.85, 0.00), (0.00, 0.25, 1.00), (0.0, 0.0, 0.0)),   # rgb sum
    ((1.00, 0.45, 0.00), (0.50, 0.00, 0.02), (1.00, 0.95, 0.60), (0.9, 0.0, 0.9)),   # ember
    ((0.55, 1.00, 0.00), (0.00, 0.45, 0.50), (0.85, 1.00, 0.95), (0.9, 0.0, 0.9)),   # lagoon: teal, lime, white
    ((0.00, 1.00, 0.55), (0.50, 0.00, 0.90), (1.00, 0.10, 0.55), (0.9, 0.0, 0.9)),   # neon: violet, green, pink
]
K0 = (0.0, 0.0, 0.0)


def softclip(v):
    k = 0.72
    return v if v < k else k + (1 - k) * math.tanh((v - k) / (1 - k))


def fold(v, k):
    p = v * k / 2.0
    return 1 - abs(1 - 2 * (p - math.floor(p)))


def chords(y, k, s10):
    """colour (r, g, b 0..1) of the Prismatic Ray level y (0..1) in colouriser setting k (0..7)"""
    if s10:
        F = 1.0 + 1.5 * k / 7.0
        return tuple(softclip(c * 0.95) for c in (fold(y, 4 * F), fold(y, 2 * F), fold(y, F)))
    C = COL[k]
    L = (y, 1 - (1 - y) ** 2, y ** 3)
    acc = [0.0, 0.0, 0.0]
    for li in (1, 0, 2):            # parabolic underneath, triangle, exponential on top
        for ch in range(3):
            acc[ch] = acc[ch] * (1 - C[3][li] * L[li]) + C[li][ch] * L[li]
    return tuple(softclip(a) for a in acc)


def rgb_to_hw10f(rgb):
    r, g, b = [min(1.0, max(0.0, c)) for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    return (int(round(min(940, max(64, y2)))), int(round(min(1023, max(0, u)))), int(round(min(1023, max(0, v)))))


def word(yuv):
    y, u, v = yuv
    return (y << 22) | (u << 12) | (v << 2)


def pal_rom():
    out = []
    for s10 in (0, 1):
        for k in range(8):
            for lv in range(64):
                y = (lv * 16 + 8) / 1023.0
                out.append(word(prescale(rgb_to_hw10f(chords(y, k, s10)))))
    return out


def exp_rom():
    return [int(round(4096 * 2.0 ** (i / 256.0))) for i in range(256)]


# ---- per-mode constants --------------------------------------------------------------------------------------
# timing ID -> (xs: h units per pixel, ys: Q8 virtual lines per field line, x0: virtual px of pixel 0)
def tid_mode(t):
    ys = {0: 1138, 4: 576, 8: 960, 12: 480, 5: 384, 6: 384, 14: 384, 1: 512, 2: 512, 10: 512}.get(t, 256)
    xs = 4 if t in (0, 4, 8, 12) else (3 if t in (5, 6, 14) else 2)
    return xs, ys, (240 if xs == 4 else 0)


def xconst(xs, x0):
    v = xs / 2.0
    d = 960 - x0
    return dict(
        RA=int(round(v * 2 ** 30 / 1920)),                        # ramp, per pixel
        RQ0=int(round(x0 * 2 ** 30 / 1920)),                      # ramp at pixel 0
        BD=int(round(v * 2 ** 30 / 1782)),                        # diamond, per pixel
        DQ0=int(round(d * 2 ** 30 / 1782)),                       # diamond at pixel 0
        EQ0=int(round(d * d * 2 ** 30 / 1312200)),                # ellipse at pixel 0
        ED0=int(round((2 * d * v - v * v) * 2 ** 30 / 1312200)),  # - ellipse first difference at pixel 0
        EDD=int(round(2 * v * v * 2 ** 30 / 1312200)),            # ellipse second difference
        IC=int(round(d / v)),                                     # pixel index of the centre column
        XADJ={2: 0, 3: 150, 4: 256}[xs],                          # log2(v) in Q8
    )


def yconst(ys):
    return dict(
        DSD=int(round(ys * 2 ** 22 / 1188)),                      # diamond, per field line (Q30)
        EDDY=int(round(ys * ys * 2 ** 15 / 583200)),              # ellipse second difference per field line
        YS2=ys * ys,
    )


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def vhd_func(name, width, key, fn):
    """function name(t : 4-bit timing ID) return unsigned(width) with one case arm per distinct value"""
    groups = {}
    for t in range(16):
        groups.setdefault(fn(t), []).append(t)
    body = ["    function %s(t : std_logic_vector(3 downto 0)) return unsigned is" % name,
            "    begin", "        case to_integer(unsigned(t)) is"]
    items = sorted(groups.items(), key=lambda kv: -len(kv[1]))
    for val, ts in items[1:]:
        body.append("            when %s => return to_unsigned(%d, %d);" % (" | ".join(str(t) for t in ts), val, width))
    body.append("            when others => return to_unsigned(%d, %d);" % (items[0][0], width))
    body += ["        end case;", "    end function;"]
    return "\n".join(body)


def main():
    xw = dict(RA=21, RQ0=28, BD=21, DQ0=30, EQ0=30, ED0=24, EDD=13, IC=11, XADJ=9)
    yw = dict(DSD=22, EDDY=17, YS2=21)
    funcs = []
    for k, w in xw.items():
        f = lambda t, k=k: xconst(tid_mode(t)[0], tid_mode(t)[2])[k]
        assert max(f(t) for t in range(16)) < 2 ** w and min(f(t) for t in range(16)) >= 0, k
        funcs.append(vhd_func("f_x" + k.lower().rstrip('_'), w, k, f))
    for k, w in yw.items():
        f = lambda t, k=k: yconst(tid_mode(t)[1])[k]
        assert max(f(t) for t in range(16)) < 2 ** w, k
        funcs.append(vhd_func("f_y" + k.lower(), w, k, f))
    parts = ["-- Generated by flyback.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package flyback_pkg is",
             vhd_table("C_FB_EXP", "unsigned", 256, 16, exp_rom()),
             vhd_table("C_FB_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             '    constant C_FB_K : std_logic_vector(31 downto 0) := x"%08X";' % word(prescale(rgb_to_hw10f(K0)))]
    parts += [f.split("\n")[0].replace(" is", ";") for f in funcs]
    parts += ["end package flyback_pkg;", "", "package body flyback_pkg is"]
    parts += funcs
    parts += ["end package body flyback_pkg;", ""]
    with open(os.path.join(HERE, "flyback_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
