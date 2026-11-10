#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BUTTERFLY v0.1.0 - build hook.  Writes butterfly_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).

Fitted to the still WIN_20231008_20_41_21 t0.5 (see .heirloom_work/rb_butterfly/notes.md).  Cell coordinates:
u = distance to the nearest X column (0 .. 470 virtual px, H period 940), v = distance to the nearest lens row
(0 .. 262.25 lines, V period 524.5).

  F1 = 5.75 ln(v / (t0 sin(pi u / 940)))        the lens field: a pencil of curves through each X point
                                                (t0 = 174: lens edge t = 134, blue band 174, glow edge 224)
  F2 = table(u, v)                              the heart / diamond / arc ring field, measured from the still on a
                                                33 x 17 node grid (bilinear), 1/16 palette units
  |g| = min(|F1'|, |F2|)                        whichever field is nearer its blue line wins
  F1' = F1 blended near each X point toward a constant-width dark arm (the camera's blur of the pencil)
  colour = palette(|g|): 0 blue, 1 black, 2.45+ magenta (measured on the lens columns)

Package contents (all also read by imodel.py), G units = 26.12 per palette unit:
  C_BF_LP    1024 x 12   round(-256 ln sin(pi u / 940)), u = (i + 0.5) * 470 / 1024, capped at 4095
  C_BF_LV     512 x 12   round(256 ln v), v = (i + 0.5) * 262.25 / 512, signed
  C_BF_TAB   1024 x 8    F2 node (j, i) at index 33 j + i (j 0..16, i 0..32), 1/16 palette units, signed
  C_BF_PAL    256 x 32   {Y, U, V, 00}: index = pal * 128 + |g| in G units (0..127; >= 64 saturates)
  C_BF_EXP2   256 x 16   round(32768 * 2^(i / 256))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
GU = 26.12          # G units per palette unit (2.45 -> 64)

# palette |g| -> RGB, measured on the lens columns of the still (palfit1.py)
PAL_G = [0.0, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05, 1.15, 1.25, 1.35, 1.45, 1.55, 1.65, 1.75,
         1.85, 1.95, 2.05, 2.15, 2.25, 2.35, 2.45, 9.0]
PAL_MAG = [(0, 41, 171), (0, 36, 145), (0.5, 30, 124), (1, 24, 105), (2, 18.5, 90), (2.4, 13.4, 78.6), (2.7, 9.5, 67.7),
           (5, 8.8, 61.6), (13.7, 8.8, 57.6), (28.6, 8.6, 56.2), (47, 7.6, 57.6), (63, 6.7, 60.6), (82.7, 5.1, 67.3),
           (102, 3.7, 77), (119.6, 3.1, 88.7), (137, 1.8, 103.3), (155.6, 1, 122.7), (181, 0.15, 153.5), (194, 0, 171),
           (207, 0, 190.6), (220, 0, 212.5), (230, 0, 230.7), (240.7, 0, 246), (247.4, 0, 253.4), (255, 0, 255),
           (255, 0, 255)]
# alternative palette "Ember": teal band, black, amber
PAL_EMB = [(0, 200, 190), (0, 170, 165), (0, 140, 140), (0, 110, 110), (2, 85, 85), (4, 62, 64), (6, 44, 46),
           (10, 30, 30), (22, 22, 20), (40, 20, 12), (62, 22, 6), (88, 28, 4), (115, 36, 2), (140, 46, 0), (165, 58, 0),
           (188, 72, 0), (205, 88, 0), (220, 104, 0), (232, 118, 0), (240, 130, 0), (247, 141, 0), (251, 150, 0),
           (253, 156, 0), (255, 160, 0), (255, 162, 0), (255, 162, 0)]

# F2 nodes (1/16 palette units), rows j = 0..16 (v = 16.39 j), columns i = 0..32 (u = 14.69 i)
TAB = [
    [-40, -36, -30, -42, -52, -60, -67, -73, -78, -82, -85, -89, -93, -98, -105, -112, -121, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127],
    [-33, -29, -24, -24, -31, -39, -48, -55, -61, -66, -70, -74, -78, -83, -89, -96, -105, -115, -126, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127],
    [-47, -40, -33, -26, -19, -20, -28, -36, -43, -49, -54, -59, -63, -67, -73, -80, -89, -99, -110, -122, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127],
    [-56, -50, -43, -36, -32, -23, -14, -18, -26, -32, -39, -44, -48, -53, -57, -64, -72, -83, -94, -105, -117, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127],
    [-61, -56, -50, -44, -38, -35, -33, -21, -10, -16, -24, -30, -35, -39, -43, -48, -57, -67, -78, -88, -99, -109, -119, -127, -127, -127, -127, -127, -127, -127, -127, -127, -127],
    [-62, -58, -54, -50, -45, -39, -34, -34, -32, -18, -7, -12, -21, -27, -31, -34, -42, -51, -62, -72, -81, -90, -99, -107, -115, -123, -127, -127, -127, -127, -127, -127, -127],
    [-58, -57, -55, -53, -50, -45, -39, -32, -28, -30, -24, -11, -3, -8, -17, -23, -28, -36, -45, -54, -62, -70, -78, -85, -92, -99, -106, -113, -119, -126, -127, -127, -127],
    [-50, -52, -53, -54, -52, -50, -45, -38, -30, -25, -21, -17, -10, 3, 4, -4, -12, -19, -26, -34, -42, -49, -56, -63, -69, -75, -81, -86, -92, -98, -103, -109, -114],
    [-39, -43, -48, -51, -52, -52, -49, -44, -35, -28, -23, -19, -14, -7, -1, 4, 6, 3, -4, -13, -20, -27, -33, -40, -46, -51, -56, -60, -65, -70, -74, -78, -83],
    [-26, -31, -38, -45, -49, -51, -50, -47, -40, -31, -26, -22, -17, -12, -8, -2, 2, 9, 12, 11, 5, -2, -9, -16, -21, -26, -30, -34, -38, -42, -45, -48, -51],
    [-16, -20, -27, -34, -41, -46, -48, -47, -42, -33, -27, -23, -19, -14, -11, -7, -2, 2, 7, 11, 15, 16, 14, 9, 5, 0, -5, -8, -12, -15, -17, -18, -19],
    [-8, -12, -17, -23, -30, -37, -43, -45, -41, -33, -27, -24, -20, -15, -11, -8, -4, -1, 2, 7, 10, 14, 19, 21, 21, 20, 18, 16, 14, 12, 12, 12, 13],
    [0, -4, -10, -15, -21, -27, -33, -38, -38, -31, -27, -23, -19, -15, -11, -8, -4, -1, 1, 5, 9, 11, 16, 21, 26, 28, 29, 30, 31, 32, 34, 37, 41],
    [7, 3, -3, -9, -14, -19, -24, -29, -31, -29, -25, -21, -17, -13, -10, -6, -2, 0, 2, 6, 9, 12, 14, 18, 23, 28, 32, 35, 40, 45, 52, 59, 66],
    [12, 9, 3, -3, -9, -13, -18, -23, -26, -26, -22, -18, -14, -11, -8, -4, -1, 1, 5, 8, 11, 14, 16, 19, 21, 25, 30, 35, 44, 55, 66, 78, 90],
    [19, 14, 8, 2, -4, -8, -12, -17, -21, -22, -19, -15, -11, -8, -4, -1, 2, 5, 8, 11, 14, 17, 20, 22, 24, 26, 28, 34, 46, 62, 78, 96, 113],
    [25, 19, 13, 7, 1, -4, -8, -12, -16, -17, -14, -10, -7, -4, 0, 2, 6, 9, 12, 15, 19, 22, 24, 27, 28, 30, 31, 37, 50, 68, 90, 113, 127],
]

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))

def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    return (min(940, int(round(y2))), u, v)

def word(yuv):
    y, u, v = prescale(yuv)
    return (y << 22) | (u << 12) | (v << 2)

def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for k in range(1, len(xs)):
        if x <= xs[k]:
            a = (x - xs[k - 1]) / (xs[k] - xs[k - 1])
            return tuple(ys[k - 1][c] + a * (ys[k][c] - ys[k - 1][c]) for c in range(3))
    return ys[-1]

def pal_rgb(p, k):
    return interp(k / GU, PAL_G, PAL_MAG if p == 0 else PAL_EMB)

def pal_rom():
    return [word(rgb_to_hw10(pal_rgb(p, k))) for p in (0, 1) for k in range(128)]

def lp_rom():
    out = []
    for i in range(1024):
        u = (i + 0.5) * 470.0 / 1024
        out.append(min(4095, int(round(-256 * math.log(math.sin(math.pi * u / 940.0))))))
    return out

def lv_rom():
    return [int(round(256 * math.log((i + 0.5) * 262.25 / 512))) for i in range(512)]

def tab_rom():
    out = [0] * 1024
    for j in range(17):
        for i in range(33):
            out[33 * j + i] = TAB[j][i]
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        elif width % 4 == 0:
            items.append('x"%0*X"' % (width // 4, v))
        else:
            items.append('to_unsigned(%d, %d)' % (v, width))
    per = 8 if signed else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    parts = ["-- Generated by butterfly.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package butterfly_pkg is",
             vhd_table("C_BF_LP", "unsigned", 1024, 12, lp_rom()),
             vhd_table("C_BF_LV", "signed", 512, 12, lv_rom(), signed=True),
             vhd_table("C_BF_TAB", "signed", 1024, 8, tab_rom(), signed=True),
             vhd_table("C_BF_PAL", "std_logic_vector", 256, 32, pal_rom()),
             vhd_table("C_BF_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package butterfly_pkg;", ""]
    with open(os.path.join(HERE, "butterfly_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
