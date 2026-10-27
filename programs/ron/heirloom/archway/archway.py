#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ARCHWAY v0.5.0 - build hook.  Writes archway_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).

Fitted to the photo WIN_20230523_01_26_43 (see .heirloom_work/rb_archway/notes.md):

  F    = |x' - 1013| - s(y) + etch          one field for every element (virtual px; x' held near x = 100)
  s(y) = 3.8736e-4 (y + 16.5)^2 - 2.6       the vertical ramp through a squarer (s(1080) = 463)
  Fs   = F sampled on a 52 px grid in (x, s): column edges 1023 + 52k, rows every 52 units of s
         (the right-hand columns past x = 1439 step a quarter row later)
  n    = floor((Fs - 20) / 52)              the stepped colouriser: one colour per step (STEP below)
  arcs = F in {282, 380, 486, 600} +- 7.5    black arcs (unquantised)
  lines= tri((F - 36.7)/37.05) + 0.636 tri(F/263.3 + ..) over three thresholds per half:
         cyan / blue rim / teal / see-through (left), fat cyan with navy gaps (right)
  chevron P = F in [-218, -60] AND Fs in [-188, -32) (below s = 95); pale W = the S&H band alone

Package contents:
  C_AW_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_AW_PAL   512 x 32   {Y, U, V, 00}: index = palette * 64 + entry (entries listed in ENTRIES)
  C_AW_FFL / C_AW_FRL / C_AW_FSL  2048 x 8 / 4 / 4   region flags of F, F - d, Fs - d (flag_roms)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# photo palette (sRGB medians of clean regions, palmed.py)
RGB = dict(K=(21, 16, 13), Y=(247, 202, 0), o=(56, 50, 8), H=(255, 0, 164), m=(99, 10, 45), B=(21, 10, 182),
           n=(21, 14, 48), P=(246, 114, 233), W=(229, 201, 196), C=(1, 205, 252), R=(25, 130, 211),
           G=(0, 232, 31), t=(26, 91, 118), Vd=(85, 22, 85), M=(170, 15, 130), V=(125, 8, 185),
           Hp=(253, 1, 222), Mg=(160, 20, 120))
# colouriser steps n = -2 .. 12 (entries 0..14)
STEP = ['Vd', 'Y', 'o', 'm', 'Y', 'Y', 'o', 'n', 'Y', 'o', 'H', 'm', 'B', 'n', 'n']
# S&H settling ring after each row step, 30 entries (photo s units into the row)
RING = ['o'] * 4 + ['M'] * 2 + ['Hp'] * 2 + ['V'] * 2 + ['B'] * 2 + ['M'] * 3 + ['V', 'B', 'B'] + \
       ['Hp'] * 5 + ['Mg'] * 4 + ['Hp'] * 3
# palette entries: 0..14 steps, 15 K, 16 right step 6, 17 P, 18 W, 19 C, 20 R, 21 t, 22 G, 23 n, 24..53 ring
ENTRIES = STEP + ['K', 'o', 'P', 'W', 'C', 'R', 't', 'G', 'n'] + RING + ['K'] * 10
assert len(ENTRIES) == 64
PERM = [(0, 1, 2), (1, 2, 0), (2, 0, 1), (0, 2, 1), (2, 1, 0), (1, 0, 2)]
PAL_NAMES = ['Archway', 'Lagoon', 'Orchard', 'Jade', 'Ultra', 'Citrus', 'Ember', 'Frost']


def palette(i):
    if i < 6:
        a, b, c = PERM[i]
        return {k: (v[a], v[b], v[c]) for k, v in RGB.items()}
    out = {}
    for k, v in RGB.items():
        l = (0.299 * v[0] + 0.587 * v[1] + 0.114 * v[2]) / 255.0
        if i == 6:   # Ember: luma onto black-red-orange-cream
            out[k] = (min(255, 40 + 330 * l), min(255, 10 + 230 * l * l), min(255, 10 + 140 * l ** 3))
        else:        # Frost: luma onto navy-cyan-white
            out[k] = (min(255, 10 + 200 * l * l), min(255, 20 + 260 * l), min(255, 60 + 260 * l))
    return out


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


def word(yuv):
    y, u, v = prescale(yuv)
    return (y << 22) | (u << 12) | (v << 2)


def pal_rom():
    out = []
    for p in range(8):
        pal = palette(p)
        for e in ENTRIES:
            out.append(word(rgb_to_hw10(pal[e])))
    return out


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def flag_roms():
    """region flags of the field, indexed by (F + 4096) >> 2 (2048 bins of 4 h; tests at the bin centre)
       FFL (on F):       0 arc, 1 line region (-20, 564), 2 F >= 564, 3 F < 754, 4 F < 1192, 5 F < 1376
       FRL (on F - d):   0 chevron band [-436, -120], 1 pale limit <= -4, 2 nested lines < -516
       FSL (on Fs - d):  0 S&H band [-376, -64), 1 inside < -376, 2 ring band >= -168, 3 opaque band >= -584"""
    ffl, frl, fsl = [], [], []
    for i in range(2048):
        f = 4 * i - 4096 + 2
        arc = any(abs(f - b) < 15 for b in (564, 760, 972, 1200))
        ffl.append(int(arc) | (int(-20 < f < 564) << 1) | (int(f >= 564) << 2) | (int(f < 754) << 3) |
                   (int(f < 1192) << 4) | (int(f < 1376) << 5))
        frl.append(int(-436 <= f <= -120) | (int(f <= -4) << 1) | (int(f < -516) << 2))
        fsl.append(int(-376 <= f < -64) | (int(f < -376) << 1) | (int(f >= -168) << 2) | (int(f >= -584) << 3))
    return ffl, frl, fsl


def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by archway.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package archway_pkg is",
             vhd_table("C_AW_EXP2", 256, 16, exp2_rom()),
             vhd_table("C_AW_PAL", 512, 32, pal_rom()),
             vhd_table("C_AW_FFL", 2048, 8, flag_roms()[0]),
             vhd_table("C_AW_FRL", 2048, 4, flag_roms()[1]),
             vhd_table("C_AW_FSL", 2048, 4, flag_roms()[2]),
             "end package archway_pkg;", ""]
    with open(os.path.join(HERE, "archway_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
