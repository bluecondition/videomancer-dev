#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEON v0.1.0 - build hook.  Writes neon_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the clip WIN_20230919_00_32_29 at 3.0 s (see .heirloom_work/rb_neon/notes.md):

  bands    U = (x + 184) * F0 * (1 + A cos(2 pi (y - yp) / 2240))       an H-synced oscillator, FM'd by a slow
           F0 = 1 / 647.7 band per virtual px, A = 0.146, yp scrolling down 316 lines/s   vertical sine
           colour = PROF(frac(U)): teal, a red glow, a white tube, a red gap, a lavender tube, [interior],
           a red-magenta rim, a white tube, a red glow (measured, row-aligned median of 3 frames)
  interior (frame-locked) ax = distance of P = |x - 992| * 1024/770 to the nearest multiple of 1024 (q units)
           phi = (y - 515) / 2080;  D = 44 (1 - |cos 4 pi phi|),  W = 42 |cos 2 pi phi|
           S = min(|ax - 127 - D| - W,  |ax + 827 sqrt(dphi) - 105| - 35)   (eyes at the cusp rows)
           colour = IPAL(S): magenta core, white line at S = 0, violet, dimming;  dark cores where
           ||ax - 127 - D| - 135| is small near the mid rows

Package contents (all tables also read by imodel.py):
  C_NE_PROF  512 x 32  {Y10, Cr9, Cb9, glow1, 0, alpha2 (0 / 1/2 / 1)}: index = 256 + 512 (u - 0.7225), u = band phase fraction
  C_NE_IPAL  512 x 32  {Y10, Cr9, Cb9, 0000}: index = S + 96 (S in q units, 1 q = 770/1024 virtual px)
  C_NE_COSQ  1024 x 16 quarter-wave cosine, round(32767 cos(2 pi (i + 0.5) / 4096))
  C_NE_SQRT  256 x 16  eye term: min(511, round(827 sqrt((i + 0.5) / 2048)))   (i = dphi in 1/2048 cycles)
  C_NE_EXP2  256 x 16  round(32768 * 2^(i/256))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

UC = 0.7225                     # interior centre (band phase fraction)
UL0, UL1 = 0.595, 0.610         # interior fade-in (left wall)
UR0, UR1 = 0.836, 0.848         # interior fade-out (right wall)
GLOW_L, GLOW_R = 0.522, 0.915   # outside these the profile is glow / teal ("Bare" replaces it with teal)
TEAL = (0, 66, 71)
DARK = (25, 13, 42)              # the dark cores near the mid rows

# row-aligned median band profile (u, R, G, B); u > 1 is the right glow wrapping into the next period
PROF_KNOTS = [
    (0.250, 0, 66, 70), (0.255, 0, 66, 70), (0.260, 0, 66, 71), (0.265, 0, 66, 70), (0.270, 0, 66, 70), (0.275, 0, 66, 70),
    (0.280, 0, 66, 69), (0.285, 1, 65, 69), (0.290, 1, 65, 69), (0.295, 1, 65, 69), (0.300, 1, 65, 70), (0.305, 2, 65, 70),
    (0.310, 2, 65, 70), (0.315, 3, 65, 70), (0.320, 3, 65, 70), (0.325, 4, 65, 70), (0.330, 4, 65, 70), (0.335, 5, 65, 70),
    (0.340, 6, 65, 70), (0.345, 7, 65, 70), (0.350, 9, 64, 70), (0.355, 10, 64, 71), (0.360, 12, 64, 71), (0.365, 14, 64, 71),
    (0.370, 15, 64, 71), (0.375, 17, 64, 71), (0.380, 19, 64, 71), (0.385, 22, 63, 71), (0.390, 24, 63, 70), (0.395, 27, 63, 70),
    (0.400, 30, 63, 69), (0.405, 33, 63, 69), (0.410, 36, 62, 69), (0.415, 40, 62, 69), (0.420, 43, 62, 69), (0.425, 46, 62, 69),
    (0.430, 50, 62, 68), (0.435, 54, 62, 68), (0.440, 59, 61, 68), (0.445, 62, 61, 68), (0.450, 66, 62, 68), (0.455, 70, 62, 68),
    (0.460, 74, 61, 68), (0.465, 79, 61, 68), (0.470, 84, 60, 69), (0.475, 89, 60, 69), (0.480, 94, 60, 70), (0.485, 99, 60, 69),
    (0.490, 104, 59, 69), (0.495, 109, 59, 68), (0.500, 114, 59, 68), (0.505, 119, 59, 68), (0.510, 121, 60, 67), (0.515, 117, 57, 64),
    (0.520, 124, 64, 77), (0.525, 171, 118, 135), (0.530, 198, 161, 175), (0.535, 216, 193, 204), (0.540, 228, 217, 223), (0.545, 233, 230, 232),
    (0.550, 229, 228, 229), (0.555, 221, 207, 213), (0.560, 217, 183, 196), (0.565, 211, 156, 173), (0.570, 203, 130, 147), (0.575, 190, 102, 120),
    (0.580, 179, 88, 112), (0.585, 200, 133, 168), (0.590, 217, 168, 215), (0.595, 220, 163, 222), (0.600, 213, 118, 227), (0.605, 208, 83, 237),
    (0.610, 204, 63, 248), (0.615, 202, 55, 253), (0.620, 202, 47, 254), (0.625, 199, 41, 255), (0.630, 197, 45, 255), (0.635, 195, 48, 255),
    (0.640, 197, 54, 255), (0.645, 199, 58, 255), (0.650, 203, 58, 255), (0.655, 207, 51, 255), (0.660, 208, 38, 255), (0.665, 209, 31, 255),
    (0.670, 211, 25, 255), (0.675, 212, 18, 255), (0.680, 209, 14, 255), (0.685, 205, 12, 255), (0.690, 202, 14, 255), (0.695, 200, 20, 255),
    (0.700, 204, 32, 255), (0.705, 207, 43, 255), (0.710, 210, 59, 255), (0.715, 210, 70, 255), (0.720, 206, 78, 255), (0.725, 204, 72, 255),
    (0.730, 198, 61, 255), (0.735, 195, 55, 255), (0.740, 192, 47, 255), (0.745, 189, 40, 255), (0.750, 180, 35, 255), (0.755, 165, 29, 254),
    (0.760, 154, 24, 253), (0.765, 148, 16, 253), (0.770, 143, 13, 252), (0.775, 137, 12, 252), (0.780, 132, 12, 250), (0.785, 131, 12, 250),
    (0.790, 126, 12, 251), (0.795, 121, 11, 250), (0.800, 117, 11, 248), (0.805, 116, 10, 247), (0.810, 116, 10, 245), (0.815, 114, 9, 245),
    (0.820, 114, 9, 246), (0.825, 110, 9, 248), (0.830, 102, 9, 245), (0.835, 96, 7, 234), (0.840, 106, 6, 218), (0.845, 132, 14, 206),
    (0.850, 151, 25, 193), (0.855, 155, 35, 176), (0.860, 157, 50, 167), (0.865, 194, 125, 194), (0.870, 209, 167, 206), (0.875, 212, 185, 208),
    (0.880, 210, 186, 203), (0.885, 212, 186, 203), (0.890, 221, 203, 216), (0.895, 230, 223, 231), (0.900, 231, 229, 233), (0.905, 217, 215, 219),
    (0.910, 199, 190, 199), (0.915, 178, 159, 172), (0.920, 157, 130, 144), (0.925, 137, 106, 120), (0.930, 120, 88, 100), (0.935, 108, 77, 88),
    (0.940, 100, 72, 82), (0.945, 92, 69, 78), (0.950, 86, 67, 76), (0.955, 80, 67, 74), (0.960, 75, 66, 73), (0.965, 71, 66, 72),
    (0.970, 67, 65, 72), (0.975, 63, 65, 71), (0.980, 58, 65, 71), (0.985, 54, 64, 71), (0.990, 50, 64, 71), (0.995, 46, 64, 71),
    (1.000, 43, 64, 71), (1.005, 39, 64, 71), (1.010, 36, 64, 71), (1.015, 33, 64, 71), (1.020, 29, 64, 71), (1.025, 26, 64, 71),
    (1.030, 24, 65, 72), (1.035, 21, 65, 72), (1.040, 19, 65, 72), (1.045, 17, 65, 72), (1.050, 15, 65, 72), (1.055, 13, 65, 72),
    (1.060, 12, 65, 72), (1.065, 10, 65, 71), (1.070, 8, 65, 71), (1.075, 7, 66, 71), (1.080, 6, 66, 71), (1.085, 5, 66, 71),
    (1.090, 5, 66, 71), (1.095, 4, 65, 71), (1.100, 3, 65, 70), (1.105, 3, 65, 70), (1.110, 2, 65, 70), (1.115, 2, 66, 70),
    (1.120, 2, 66, 71), (1.125, 1, 66, 71), (1.130, 1, 66, 71), (1.135, 1, 66, 71), (1.140, 0, 66, 71), (1.145, 0, 66, 71),
    (1.150, 0, 66, 71), (1.155, 0, 66, 71), (1.160, 0, 66, 71), (1.165, 0, 66, 71), (1.170, 0, 66, 71)
]

# interior colouriser knots over S in the fit's units (1 unit = 1.1 virtual px; the table index is in q units)
IPAL_KNOTS = [(-60, 232, 6, 255), (-16, 232, 8, 255), (-11, 233, 70, 254), (-6, 232, 190, 242), (0, 234, 230, 236),
              (6, 228, 190, 242), (11, 205, 80, 250), (16, 150, 6, 255), (24, 128, 3, 252), (36, 108, 5, 236),
              (50, 92, 8, 208), (70, 96, 11, 200), (100, 104, 8, 222), (140, 112, 8, 245), (400, 112, 8, 245)]
Q_PER_FIT = 1.1 * 1024 / 770     # q units per fit unit


def interp(knots, x):
    if x <= knots[0][0]:
        return knots[0][1:]
    for a, b in zip(knots, knots[1:]):
        if x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            return tuple(a[i] + (b[i] - a[i]) * t for i in (1, 2, 3))
    return knots[-1][1:]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def pack(rgb, low4):
    y, u, v = rgb_to_hw10(rgb)
    y = 64 + (y - 64) * 16 / 13
    yi = max(64, min(940, int(round(y))))
    ui = max(0, min(511, int(round(u / 2))))
    vi = max(0, min(511, int(round(v / 2))))
    return (yi << 22) | (ui << 13) | (vi << 4) | (low4 & 15)


def prof_rgb(u):
    if u < PROF_KNOTS[0][0]:
        u += 1.0
    if u > PROF_KNOTS[-1][0]:
        return TEAL
    return interp(PROF_KNOTS, u)


def tables():
    prof = []
    for i in range(512):
        v = (i - 256) / 512.0
        u = UC + v
        a = min(1.0, max(0.0, (u - UL0) / (UL1 - UL0))) * min(1.0, max(0.0, (UR1 - u) / (UR1 - UR0)))
        al = 0 if a < 0.25 else (1 if a < 0.75 else 2)    # interior weight 0 / 1/2 / 1
        glow = 1 if (u < GLOW_L or u > GLOW_R) else 0
        uu = min(max(u, UL0), UR1) if a > 0 else u        # inside the window: hold the wall colours (unused)
        prof.append(pack(prof_rgb(uu % 1.0 if uu >= 0 else uu + 1.0), (glow << 3) | al))
    ipal = []
    for i in range(512):
        s = (i - 96) / Q_PER_FIT
        ipal.append(pack(interp(IPAL_KNOTS, s), 0))
    cosq = [int(round(32767 * math.cos(2 * math.pi * (i + 0.5) / 4096))) for i in range(1024)]
    sqrt_t = [min(511, int(round(827 * math.sqrt((i + 0.5) / 2048.0)))) for i in range(256)]
    exp2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    return dict(prof=prof, ipal=ipal, cosq=cosq, sqrt=sqrt_t, exp2=exp2)


def vhdl_array(name, typ, vals, width, hexw):
    s = "    constant %s : %s := (\n" % (name, typ)
    items = ['x"%0*X"' % (hexw, v) if width % 4 == 0 else '"%s"' % format(v, '0%db' % width) for v in vals]
    for k in range(0, len(items), 8):
        s += "        " + ", ".join(items[k:k + 8]) + ("," if k + 8 < len(items) else "") + "\n"
    s += "    );\n"
    return s


def main():
    t = tables()
    s = "-- Generated by neon.py: do not edit.\nlibrary ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n"
    s += "package neon_pkg is\n"
    s += "    type t_ne_w32 is array (0 to 511) of std_logic_vector(31 downto 0);\n"
    s += "    type t_ne_cos is array (0 to 1023) of unsigned(15 downto 0);\n"
    s += "    type t_ne_256 is array (0 to 255) of unsigned(15 downto 0);\n"
    s += vhdl_array("C_NE_PROF", "t_ne_w32", t['prof'], 32, 8)
    s += vhdl_array("C_NE_IPAL", "t_ne_w32", t['ipal'], 32, 8)
    s += vhdl_array("C_NE_COSQ", "t_ne_cos", t['cosq'], 16, 4)
    s += vhdl_array("C_NE_SQRT", "t_ne_256", t['sqrt'], 16, 4)
    s += vhdl_array("C_NE_EXP2", "t_ne_256", t['exp2'], 16, 4)
    dk = pack(DARK, 0)
    s += "    constant C_NE_DKY : integer := %d;\n" % ((dk >> 22) & 1023)
    s += "    constant C_NE_DKU : integer := %d;\n" % (((dk >> 13) & 511) * 2)
    s += "    constant C_NE_DKV : integer := %d;\n" % (((dk >> 4) & 511) * 2)
    s += "end package neon_pkg;\n"
    with open(os.path.join(HERE, "neon_pkg.vhd"), "w") as f:
        f.write(s)


if __name__ == "__main__":
    main()
