#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LANTERN v0.1.0 - build hook.  Writes lantern_pkg.vhd next to this file (pure python: the builder runs it
with the system python3; proto / imodel in .heirloom_work/rb_lantern import the same constants).

Fitted to the still WIN_20231002_01_20_50 at 0.5 s (see .heirloom_work/rb_lantern/notes.md):

  u   = |x - XC| (virtual px; the left half reads u * (1 + 1/128) and adds DL: the fold's two halves are a
        little apart, which gives the photo's step at the lantern's apex)
  A   = KA u + KB u^2                                 an |x| fold plus an x^2 parabola (periods)
  Vg  = G0 + GV (1 - cos(2 pi (y - Y) / PV)) / 2       a vertical sine oscillator: the bowl's valley at Y
  Lh  = LAM (y - YL)                                  a static vertical ramp
  f1  = A + Vg + Lh,  f2 = A + Vg - Lh               two crossing families of contours
  e_i = FLOOR - f_i below the floor level (2), else |f_i - round(f_i)|
  s   = min(e1, e2)                                    -> a 1-D palette (yellow line, brown core, purple, pink)
  the left 55 virtual px hold the line-start value (the analog line-start hold)

Package contents:
  C_LT_COS  1024 x 16   round(65535 * (1 - cos(2 pi i / 1024)) / 2)       (the RTL interpolates between entries)
  C_LT_PY   1024 x 8    palette luma >> 2 (pre-scaled 16/13, capped 940): index = gel * 512 + ink * 256 + s_q
  C_LT_PU   1024 x 8    palette U (= Cr) >> 2
  C_LT_PV   1024 x 8    palette V (= Cb) >> 2
  s_q = round(s * 512), 0..255 (s in periods, saturating at 0.498)
Colours are in the hardware convention (U register = Cr, V register = Cb).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (virtual px / lines, periods)
XC = 987.33          # mirror axis (virtual px); the RTL puts it on 987 (HD / 720p) or 988 (SD)
AL = 1.0 + 1.0 / 128 # left-half scale (fit 1.0071)
DL = -0.032          # left-half offset (periods)
KA = 2.19095e-3      # periods per virtual px (|x| term)
KB = 1.11515e-6      # periods per virtual px^2 (x^2 term)
G0 = 0.5623          # bowl minimum (periods)
GV = 2.5640          # bowl peak-to-valley (periods)
YV = 720.55          # bowl valley (virtual line) in the still
PV = 2500.47         # vertical oscillator period (virtual lines)
LAM = 0.96234e-3     # tilt (periods per virtual line)
YL = 517.67          # tilt zero line
FLOOR = 2.0          # first drawn level
XH = 55.0            # line-start hold (virtual px)
VY = 181.0           # bowl drift in the clip (virtual lines per second, downward)

# ------------------------------------------------------------------ palettes: (s, (R, G, B)) control points
PAL_MAG_NEON = [  # measured medians of the still per s bin
    (0.0, (199, 210, 75)), (0.0025, (199, 210, 75)), (0.0075, (193, 202, 71)), (0.0125, (184, 190, 63)),
    (0.0175, (169, 171, 54)), (0.0225, (147, 145, 43)), (0.0275, (122, 117, 33)), (0.0325, (96, 88, 25)),
    (0.0375, (76, 61, 17)), (0.045, (61, 36, 9)), (0.055, (52, 23, 2)), (0.065, (44, 21, 1)), (0.075, (43, 20, 4)),
    (0.085, (46, 20, 12)), (0.095, (50, 19, 24)), (0.11, (56, 18, 46)), (0.13, (67, 16, 74)), (0.15, (77, 15, 101)),
    (0.17, (88, 13, 126)), (0.19, (99, 12, 141)), (0.21, (111, 11, 144)), (0.235, (125, 10, 144)),
    (0.265, (143, 9, 144)), (0.295, (159, 8, 143)), (0.33, (179, 7, 142)), (0.375, (201, 6, 141)),
    (0.425, (225, 5, 139)), (0.475, (247, 4, 138)), (0.6, (255, 3, 136))]
PAL_MAG_NIGHT = [  # Ink = Night: the lines glow pink-gold on a black ground
    (0.0, (255, 236, 150)), (0.012, (250, 206, 90)), (0.03, (255, 60, 170)), (0.06, (235, 12, 150)),
    (0.11, (150, 8, 140)), (0.18, (70, 10, 105)), (0.26, (28, 6, 46)), (0.34, (12, 6, 16)), (0.6, (10, 6, 12))]
PAL_CYAN_NEON = [  # Gel = Cyan: cyan lantern, deep blue ramp, orange lines
    (0.0, (255, 150, 50)), (0.01, (250, 140, 45)), (0.02, (220, 110, 35)), (0.03, (150, 70, 25)),
    (0.04, (70, 40, 20)), (0.06, (12, 22, 30)), (0.09, (10, 24, 40)), (0.14, (15, 40, 110)), (0.2, (20, 50, 160)),
    (0.3, (10, 120, 200)), (0.4, (0, 180, 225)), (0.48, (0, 215, 235)), (0.6, (0, 215, 235))]
PAL_CYAN_NIGHT = [
    (0.0, (255, 250, 200)), (0.012, (255, 200, 90)), (0.03, (40, 230, 240)), (0.06, (0, 190, 230)),
    (0.11, (0, 110, 190)), (0.18, (10, 40, 120)), (0.26, (6, 16, 46)), (0.34, (5, 8, 16)), (0.6, (5, 6, 12))]
PALS = [PAL_MAG_NEON, PAL_MAG_NIGHT, PAL_CYAN_NEON, PAL_CYAN_NIGHT]   # index = gel * 2 + ink


def pal_rgb(pal, s):
    if s <= pal[0][0]:
        return pal[0][1]
    for (s0, c0), (s1, c1) in zip(pal, pal[1:]):
        if s <= s1:
            w = (s - s0) / (s1 - s0)
            return tuple(c0[i] + w * (c1[i] - c0[i]) for i in range(3))
    return pal[-1][1]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def pal_rom():
    """three 1024 x 8 tables (Y, U, V >> 2), index = gel * 512 + ink * 256 + s_q"""
    ys, us, vs = [], [], []
    for p in range(4):
        for q in range(256):
            s = q / 512.0
            y, u, v = prescale(rgb_to_hw10(pal_rgb(PALS[p], s)))
            ys.append(min(255, (y + 2) >> 2)); us.append(min(255, (u + 2) >> 2)); vs.append(min(255, (v + 2) >> 2))
    return ys, us, vs


def cos_rom():
    return [min(65535, int(round(65535 * (1 - math.cos(2 * math.pi * i / 1024)) / 2))) for i in range(1024)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    ys, us, vs = pal_rom()
    parts = ["-- Generated by lantern.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package lantern_pkg is",
             vhd_table("C_LT_COS", "unsigned", 1024, 16, cos_rom()),
             vhd_table("C_LT_PY", "unsigned", 1024, 8, ys),
             vhd_table("C_LT_PU", "unsigned", 1024, 8, us),
             vhd_table("C_LT_PV", "unsigned", 1024, 8, vs),
             "end package lantern_pkg;", ""]
    with open(os.path.join(HERE, "lantern_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
