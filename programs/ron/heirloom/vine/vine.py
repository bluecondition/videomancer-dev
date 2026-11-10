#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
VINE v0.1.0 - build hook.  Writes vine_pkg.vhd next to this file (pure python: the builder runs it with the system
python3).  imodel.py (.heirloom_work/rb_vine) imports the same tables.

Fitted to the still WIN_20231026_23_13_39 at 0.5 s (see .heirloom_work/rb_vine/notes.md):

  vines   an FM'd horizontal oscillator: phi = (x + 160) F(y), F = F0 (1 + m sin(2 pi y / 1125 + theta)),
          F0 = 1 / 556.6 px, m = 0.130, vine centres at phi = k + 0.688
  ground  pure green, G = 144.5 + 125.9 cos(2 pi (y - 257) / 524) - 21.3 cos(4 pi (y - 257) / 524), minus a
          horizontal parabola 116 (x / 1900)^2
  colours the vines' blue -> violet -> magenta ramp in 64 quantile steps of the still's vine cores (VCOL), indexed
          through the integer model's CDF of the colour noise (VCDF), tongue / leaf / rim ramps

Package contents:
  C_VN_PAL   1024 x 32  {Y, U, V, 00}: index = set * 512 + region * 128 + level
                        set 0 Vine (the clip), 1 Ember (orange vines, yellow blossoms on a deep blue ground)
                        region 0 ground (level = G / 2), 1 vine (level = colour noise / 2), 2 tongue (0..63) and
                        leaf (64..127), 3 rim (0..63) and dark spike (64..75)
  C_VN_SIN   256 x 16   quarter sine, round(32767 sin(2 pi (i + 0.5) / 1024))
  C_VN_EXP2  256 x 16   round(32768 * 2^(i / 256))
  C_VN_GRN   1024 x 8   the ground's vertical profile, index = (y - 257) * 512 / 524 mod 512 (+512: Night)
  C_VN_DX    256 x 8    the ground's horizontal darkening, index = virtual x (half px) / 16
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# 64 quantile steps of the still's vine cores (|u| < 0.08 cycle), ranked by R + 0.7 G
VCOL = [
    (0, 0, 228), (0, 0, 227), (0, 0, 234), (0, 0, 235), (0, 0, 235), (0, 0, 233), (0, 0, 225), (0, 0, 228),
    (0, 0, 233), (0, 0, 234), (0, 0, 236), (0, 0, 235), (0, 0, 240), (0, 0, 239), (0, 0, 235), (0, 0, 234),
    (0, 0, 230), (0, 0, 229), (1, 0, 239), (2, 0, 241), (3, 0, 243), (4, 0, 243), (5, 0, 244), (7, 0, 243),
    (8, 0, 245), (10, 0, 244), (11, 0, 247), (13, 0, 247), (15, 0, 248), (16, 0, 248), (18, 0, 249), (20, 0, 250),
    (22, 0, 251), (25, 0, 251), (27, 0, 251), (29, 1, 251), (32, 0, 252), (35, 0, 252), (37, 0, 252), (40, 0, 252),
    (43, 0, 253), (46, 0, 253), (49, 0, 253), (53, 0, 253), (57, 0, 253), (61, 0, 253), (65, 0, 254), (69, 0, 254),
    (74, 0, 254), (78, 0, 254), (84, 1, 254), (89, 1, 254), (95, 1, 254), (102, 1, 254), (108, 1, 254), (116, 1, 254),
    (124, 1, 254), (133, 1, 255), (144, 1, 255), (156, 1, 255), (171, 2, 255), (191, 3, 255), (217, 6, 255), (245, 19, 254),
]
# CDF of the colour index (0..127) over vine pixels at the defaults, x 64 (imodel.py vcdf()); placeholder = linear
VCDF = [
    0.03, 0.06, 0.08, 0.11, 0.15, 0.19, 0.24, 0.30, 0.37, 0.45, 0.53, 0.63, 0.74, 0.86, 1.00, 1.15,
    1.33, 1.52, 1.73, 1.96, 2.20, 2.47, 2.74, 3.04, 3.35, 3.68, 4.02, 4.38, 4.75, 5.15, 5.57, 6.00,
    6.45, 6.92, 7.41, 7.90, 8.40, 8.92, 9.44, 9.98, 10.54, 11.11, 11.69, 12.28, 12.89, 13.51, 14.16, 14.81,
    15.48, 16.17, 16.88, 17.60, 18.34, 19.10, 19.87, 20.66, 21.47, 22.29, 23.12, 23.96, 24.79, 25.64, 26.48, 27.33,
    28.18, 29.05, 29.92, 30.79, 31.67, 32.54, 33.40, 34.25, 35.10, 35.94, 36.78, 37.62, 38.46, 39.30, 40.14, 40.98,
    41.82, 42.65, 43.47, 44.28, 45.09, 45.89, 46.69, 47.47, 48.24, 49.01, 49.76, 50.49, 51.21, 51.91, 52.59, 53.25,
    53.89, 54.51, 55.11, 55.70, 56.27, 56.81, 57.34, 57.84, 58.33, 58.80, 59.25, 59.67, 60.07, 60.44, 60.80, 61.13,
    61.45, 61.74, 62.01, 62.25, 62.48, 62.69, 62.87, 63.04, 63.18, 63.31, 63.43, 63.53, 63.62, 63.70, 63.77, 63.90,
]

TONGUE = [(0.00, (60, 10, 245)), (0.25, (140, 90, 245)), (0.50, (175, 180, 240)), (0.70, (225, 225, 240)),
          (0.85, (120, 215, 200)), (1.00, (165, 225, 120))]
LEAF = [(0.00, (90, 215, 60)), (0.25, (170, 230, 110)), (0.45, (200, 240, 170)), (0.60, (85, 222, 196)),
        (0.78, (150, 200, 232)), (0.90, (222, 139, 205)), (1.00, (238, 225, 245))]
RIM = (10, 4, 140)
DARK = (2, 4, 10)

EMBER_VINE = [(0.0, (150, 20, 0)), (0.45, (215, 45, 0)), (0.62, (240, 110, 0)), (0.80, (255, 190, 20)),
              (1.0, (255, 245, 150))]
EMBER_GROUND = (20, 40, 235)
EMBER_TONGUE = [(0.00, (230, 70, 0)), (0.3, (250, 160, 60)), (0.55, (255, 230, 190)), (0.75, (120, 230, 255)),
                (1.00, (60, 190, 255))]
EMBER_LEAF = [(0.0, (0, 220, 255)), (0.4, (120, 240, 255)), (0.7, (255, 240, 120)), (1.0, (255, 255, 255))]
EMBER_RIM = (140, 10, 0)


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def word(rgb):
    rgb = tuple(int(round(min(255, max(0, v)))) for v in rgb)
    y, u, v = rgb_to_hw10(rgb)
    y2 = int(round(min(940, max(64, 64 + (y - 64) * 16.0 / 13.0))))
    u = min(1000, max(24, int(u))); v = min(1000, max(24, int(v)))
    return (y2 << 22) | (u << 12) | (v << 2)


def ramp(keys, t):
    t = min(1.0, max(0.0, t))
    for i in range(len(keys) - 1):
        a, ca = keys[i]; b, cb = keys[i + 1]
        if t <= b:
            w = 0.0 if b == a else (t - a) / (b - a)
            return tuple(ca[c] + (cb[c] - ca[c]) * w for c in range(3))
    return keys[-1][1]


def vine_rgb(lvl, ember):
    cdf = VCDF[lvl] if VCDF is not None else lvl / 2.0
    q = min(63, int(cdf))
    if not ember:
        return VCOL[q]
    return ramp(EMBER_VINE, q / 63.0)


def pal_rgb():
    out = []
    for ember in (0, 1):
        for lvl in range(128):                                   # region 0: ground
            g = min(255, 2 * lvl + 1)
            if ember:
                out.append(tuple(c * g / 255.0 for c in EMBER_GROUND))
            else:
                out.append((0, g, 0))
        for lvl in range(128):                                   # region 1: vine
            out.append(vine_rgb(lvl, ember))
        for lvl in range(128):                                   # region 2: tongue / leaf
            if lvl < 64:
                out.append(ramp(EMBER_TONGUE if ember else TONGUE, lvl / 63.0))
            else:
                out.append(ramp(EMBER_LEAF if ember else LEAF, (lvl - 64) / 63.0))
        rim = EMBER_RIM if ember else RIM
        for lvl in range(128):                                   # region 3: rim / dark spike
            if lvl < 64:
                w = (lvl / 63.0) ** 1.2
            else:
                w = max(0.0, 1.0 - (lvl - 64) / 12.0)
            out.append(tuple(DARK[c] + w * rim[c] for c in range(3)))
    return out


def pal_rom():
    return [word(c) for c in pal_rgb()]


def sin_rom():
    return [int(round(32767 * math.sin(2 * math.pi * (i + 0.5) / 1024))) for i in range(256)]


def exp2_rom():
    return [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]


def grn_rom():
    """the ground's vertical profile (0..511), then the Night profile (512..1023): clip(1.625 (255 - G) - 200)"""
    out = []
    for i in range(512):
        yy = i * 524.0 / 512.0
        g = 144.5 + 125.9 * math.cos(2 * math.pi * yy / 524) - 21.3 * math.cos(4 * math.pi * yy / 524)
        out.append(int(round(min(255, max(0, g)))))
    night = []
    for g in out:
        t = 255 - g
        night.append(min(255, max(0, t + (t >> 1) + (t >> 3) - 200)))
    return out + night


def dx_rom():
    """the ground's horizontal darkening by virtual x / 16 (h = half virtual px): 116 (x / 1900)^2 levels"""
    return [min(127, int(116.0 * ((16 * i + 8) / 3800.0) ** 2)) for i in range(256)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by vine.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package vine_pkg is",
             vhd_table("C_VN_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             vhd_table("C_VN_SIN", "unsigned", 256, 16, sin_rom()),
             vhd_table("C_VN_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_VN_GRN", "unsigned", 1024, 8, grn_rom()),
             vhd_table("C_VN_DX", "unsigned", 256, 8, dx_rom()),
             "end package vine_pkg;", ""]
    with open(os.path.join(HERE, "vine_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
