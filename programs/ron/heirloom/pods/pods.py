#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PODS v0.1.1 - build hook.  Writes pods_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Built from the clip WIN_20230328_00_11_14 at 0.5 s (see .heirloom_work/rb_pods/notes.md).  One patch:

  r^2   = (X + 50)^2 + y^2,  X = 1004.5 - |x - 1004.5|     the H and V ramps from just off the top-left
                                                          corner, folded at the mirror seam, squared, summed
  F     = V(y) + Ax cos(2 pi (kL r^2 + pL)) + Eplat - Dv val(kE r^2) + etch * luma
          V(y) = cos(2 pi (y - y0) / 187.5)               the rows (cyan / maroon), scrolling up 141.6 px/s
          cos(kL r^2)                                     the zone-plate rings: cells where they cross the rows
          val(t)                                          the slow custom waveform of the same r^2 ramp that
                                                          opens the mint gaps (corner, two arcs, the bottom)
  colour = PAL[(F + 2.75) * 128], 512 entries per palette (F in 1/256 units: index = (F + 704) >> 1)

Package contents:
  C_PD_PAL   1024 x 32   {Y, U, V, 00}: entries 0..511 = Pods (the still), 512..1023 = Chevron (the clip at
                         00_17_52: grey / black with rainbow fringes).  Hardware convention (U register = Cr,
                         V = Cb), luma pre-scaled 16/13 for the shell's 13/16 output scale, capped at 940
  C_PD_COS   1024 x 10   round(511 cos(2 pi i / 1024)), two's complement
  C_PD_VAL   1024 x 8    valley depth 0..255 over 16 turns of the envelope phase (64 entries per turn)
  C_PD_VCOS   256 x 12   quarter-wave cosine, round(2047 cos(pi/2 * i / 256))
  C_PD_EXP2   256 x 16   round(32768 * 2^(i/256))
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- the colouriser (Pods): class colours measured on the still (median RGB per class), upper bounds in F
TH = [-2.48, -1.165, -0.82, -0.336, 0.254, 0.743, 1.175]
COL = [(176, 224, 176),   # mint ground
       (48, 30, 12),      # dark brown (gap rims, maroon cores)
       (89, 33, 9),       # maroon
       (153, 26, 8),      # red
       (56, 95, 5),       # olive
       (138, 247, 0),     # yellow-green
       (23, 69, 41),      # dark teal
       (0, 255, 180)]     # cyan
RAMP = 6                  # entries of linear blend either side of a class boundary (1 entry = 1/128 F)

# ---- Chevron (the 00_17_52 clip): grey-white, black, rainbow fringes
GREY = (180, 189, 181)
BLACK = (28, 29, 12)
RAINBOW_LO = [(32, 184, 180), (174, 48, 184), (48, 37, 191)]                    # grey -> black: cyan, magenta, blue
RAINBOW_HI = [(166, 36, 13), (172, 186, 7), (45, 186, 9)]                      # black -> grey: red, yellow, green

# ---- the envelope waveform (turns of the lattice phase at the photo's settings; period 16 turns)
VP = dict(c0=0.5, cw=0.9, cd=0.916, d0=10.39, d1=16.0, dw=1.05, a1=4.982, a1w=0.768, a1d=0.672,
          a2=8.968, a2w=0.718, a2d=0.657)


def f_of_index(i):
    """F (rows = 1) at palette index i (0..511)"""
    return (2 * i - 704) / 256.0


def lerp(a, b, t):
    return tuple(a[k] + (b[k] - a[k]) * t for k in range(3))


def pods_rgb(i):
    f = f_of_index(i)
    c = 0
    while c < len(TH) and f >= TH[c]:
        c += 1
    col = COL[c]
    # blend across the nearest boundary (RAMP entries each side)
    for b, t in enumerate(TH):
        ib = (t * 256 + 704) / 2.0          # boundary position in entries
        d = i + 0.5 - ib
        if abs(d) < RAMP:
            w = 0.5 + d / (2.0 * RAMP)
            col = lerp(COL[b], COL[b + 1], w)
    return col


def chevron_rgb(i):
    f = f_of_index(i)
    # grey below the mint threshold; a 3-hue rainbow fringe into black; black through the low half; a 3-hue
    # fringe (red, yellow, green) up to grey; every hue >= 0.32 F wide (>= 2 rows at the finest rows, HD)
    lo0, lo1 = TH[0], TH[1] - 0.15
    hi0, hi1 = TH[3] + 0.2, TH[5] + 0.1
    if f < lo0:
        return GREY
    if f < lo1:
        k = int((f - lo0) / (lo1 - lo0) * len(RAINBOW_LO))
        return RAINBOW_LO[min(k, len(RAINBOW_LO) - 1)]
    if f < hi0:
        return BLACK
    if f < hi1:
        k = int((f - hi0) / (hi1 - hi0) * len(RAINBOW_HI))
        return RAINBOW_HI[min(k, len(RAINBOW_HI) - 1)]
    return GREY


def valley(t, q=VP):
    t = t % 16.0

    def bump(c, hw):
        d = abs(t - c)
        return 0.5 + 0.5 * math.cos(math.pi * d / hw) if d < hw else 0.0

    def plateau(a, b, w):
        u = (t - a) % 16.0
        L = (b - a) % 16.0
        if u > L:
            return 0.0
        up = 0.5 - 0.5 * math.cos(math.pi * u / w) if u < w else 1.0
        dn = 0.5 - 0.5 * math.cos(math.pi * min(L - u, w) / w) if u > L - w else 1.0
        return min(up, dn)

    v = plateau(q['d0'], q['d1'], q['dw'])
    v = max(v, q['cd'] * bump(q['c0'], q['cw']))
    v = max(v, q['a1d'] * bump(q['a1'], q['a1w']))
    v = max(v, q['a2d'] * bump(q['a2'], q['a2w']))
    return v


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + int(round((y - 64) * 16 / 13.0))
    y2 = min(940, max(64, y2))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def word(yuv):
    y, u, v = prescale(yuv)
    return (y << 22) | (u << 12) | (v << 2)


def pal_rom():
    rows = [word(rgb_to_hw10(pods_rgb(i))) for i in range(512)]
    rows += [word(rgb_to_hw10(chevron_rgb(i))) for i in range(512)]
    return rows


def cos_rom():
    return [int(round(511 * math.cos(2 * math.pi * i / 1024.0))) & 1023 for i in range(1024)]


def val_rom():
    return [int(round(255 * valley(i / 64.0))) for i in range(1024)]


def vcos_rom():
    return [int(round(2047 * math.cos(math.pi / 2 * i / 256.0))) for i in range(256)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width) for v in vals]
    per = 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by pods.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package pods_pkg is",
             vhd_table("C_PD_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             vhd_table("C_PD_COS", "unsigned", 1024, 10, cos_rom()),
             vhd_table("C_PD_VAL", "unsigned", 1024, 8, val_rom()),
             vhd_table("C_PD_VCOS", "unsigned", 256, 12, vcos_rom()),
             vhd_table("C_PD_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package pods_pkg;", ""]
    with open(os.path.join(HERE, "pods_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
