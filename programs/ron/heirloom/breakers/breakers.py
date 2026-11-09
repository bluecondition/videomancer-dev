#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BREAKERS v0.1.1 - build hook.  Writes breakers_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  imodel.py (.heirloom_work/rb_breakers) imports the same tables.

Fitted to the still WIN_20230214_23_42_08 at 0.5 s (see .heirloom_work/rb_breakers/notes.md):

  field  f = (x - X0(y)) / S(y)      band units; band 0 (the first blue ribbon) at f = 0, band 1 at f = 1
         X0 = M(y) + A(y) sin(Theta(y)),  Theta = vf (y / 420 + 0.0005 y^2 / 1080) cycles - tau + phase
  clean  one band period of the still (BAND, measured along f from band 0 to band 1, pre-sharpened against the
         output blur), cyan past f = 1.5 with a faint contour at f = 1.80
  surf   indexed by d = shore - f: broad dark-teal stripes on the chirped surf phase 3.2 d + 0.45 d^2 with random
         fine stripes between, coloured by the still's surf palette shares

Package contents:
  C_BR_PAL   2048 x 32  {Y, U, V, 00}: index = set * 1024 + half * 512 + i
                        set 0 Sea (the still), 1 Crab (the sibling clip WIN_20230214_23_38_38)
                        half 0 clean: f = i / 128 - 1 (f in -1 .. 3); half 1 surf: d = i / 128 - 0.5
  C_BR_SIN   256 x 16   quarter sine, round(32767 sin(2 pi (i + 0.5) / 1024))
  C_BR_EXP2  256 x 16   round(32768 * 2^(i / 256))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))

# one band period of the still, f = 0 (band 0 core) .. 1 (band 1 core), 128 steps, RGB
BAND = [
    (0, 59, 245), (0, 61, 241), (0, 65, 242), (0, 70, 243), (0, 76, 242), (0, 82, 241), (0, 89, 241), (0, 96, 241),
    (0, 103, 241), (0, 109, 241), (1, 116, 242), (3, 122, 242), (8, 127, 241), (19, 133, 242), (37, 137, 242), (63, 142, 242),
    (92, 147, 242), (123, 151, 242), (152, 156, 240), (178, 163, 239), (200, 170, 238), (213, 177, 238), (214, 185, 237), (202, 194, 236),
    (178, 202, 235), (147, 211, 235), (111, 219, 237), (75, 226, 238), (43, 233, 239), (19, 238, 240), (5, 243, 241), (1, 245, 242),
    (4, 248, 243), (11, 250, 243), (21, 252, 244), (34, 254, 244), (49, 255, 243), (64, 255, 243), (80, 255, 242), (97, 255, 242),
    (114, 255, 241), (132, 255, 241), (150, 255, 240), (169, 255, 241), (186, 255, 241), (203, 255, 241), (217, 255, 241), (226, 255, 240),
    (228, 255, 240), (223, 255, 240), (212, 255, 240), (196, 255, 241), (176, 255, 242), (156, 255, 242), (135, 255, 242), (114, 255, 243),
    (94, 255, 244), (74, 255, 245), (54, 255, 246), (37, 255, 247), (24, 255, 248), (15, 255, 249), (10, 255, 250), (7, 255, 251),
    (6, 255, 251), (6, 255, 252), (6, 255, 251), (8, 255, 252), (16, 255, 252), (31, 255, 252), (52, 255, 252), (76, 255, 252),
    (99, 255, 251), (122, 255, 251), (144, 255, 250), (165, 255, 250), (184, 255, 250), (202, 255, 250), (217, 255, 250), (225, 255, 249),
    (226, 255, 248), (218, 255, 248), (205, 255, 248), (187, 255, 249), (166, 255, 249), (144, 255, 249), (121, 255, 250), (100, 255, 251),
    (78, 255, 252), (57, 255, 253), (37, 255, 254), (21, 255, 254), (10, 254, 255), (3, 251, 255), (0, 248, 255), (5, 243, 255),
    (20, 238, 255), (44, 233, 255), (73, 227, 255), (104, 221, 255), (135, 216, 255), (166, 210, 255), (192, 205, 255), (209, 202, 255),
    (214, 200, 254), (207, 197, 254), (189, 193, 254), (164, 189, 254), (133, 184, 255), (98, 179, 255), (65, 172, 255), (37, 165, 255),
    (18, 157, 255), (8, 149, 255), (2, 141, 255), (0, 133, 255), (0, 125, 255), (0, 117, 255), (0, 110, 255), (0, 103, 255),
    (0, 95, 255), (0, 88, 255), (0, 81, 255), (0, 75, 255), (0, 69, 255), (0, 63, 255), (0, 60, 255), (0, 59, 253),
]

SEA_CYAN = (6, 254, 255)
SEA_LINE = (20, 205, 255)        # the faint contour in the open water
SURF_COLS = [(78, 247, 139), (33, 244, 94), (15, 171, 131), (19, 237, 152), (19, 171, 78), (53, 251, 193),
             (88, 224, 86), (127, 250, 180), (146, 240, 110), (190, 245, 120), (205, 200, 130)]
SURF_SHARE = [15.7, 11.9, 8.0, 10.6, 7.0, 10.2, 8.3, 9.1, 8.7, 6.0, 3.0]
SURF_DARK = (6, 100, 72)
SURF_EDGE = (40, 240, 200)       # just on the clean side of the shore (reached only through the noise)

# Crab (the sibling clip): band phase a = |f - round f| in 0 .. 0.5 -> colour; open water black-green
CRAB_KEYS = [(0.00, (16, 20, 240)), (0.07, (40, 22, 205)), (0.11, (110, 42, 215)), (0.15, (125, 30, 100)),
             (0.19, (26, 111, 119)), (0.25, (11, 125, 12)), (0.31, (160, 104, 9)), (0.36, (70, 96, 12)),
             (0.41, (115, 28, 23)), (0.46, (14, 21, 17)), (0.50, (10, 14, 12))]
CRAB_OPEN = (12, 18, 14)
CRAB_LINE = (20, 22, 110)
CRAB_SURF = [(14, 21, 17), (11, 115, 10), (105, 28, 23), (17, 18, 81), (151, 100, 9), (70, 96, 12), (16, 12, 153),
             (97, 28, 111)]
CRAB_SHARE = [24, 18, 9, 9, 10, 12, 8, 6]
CRAB_DARK = (4, 6, 5)
CRAB_EDGE = (20, 60, 120)


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def word(rgb):
    y, u, v = rgb_to_hw10(rgb)
    y2 = int(round(min(940, max(64, 64 + (y - 64) * 16.0 / 13.0))))
    u = min(1000, max(24, int(u))); v = min(1000, max(24, int(v)))
    return (y2 << 22) | (u << 12) | (v << 2)


def lerp(a, b, w):
    return tuple(a[i] + (b[i] - a[i]) * w for i in range(3))


def band_sea(f):
    i = int(round((f % 1.0) * 128)) % 128
    r, g, b = BAND[i]
    if i < 64:
        b = max(b, 248)
    return (r, g, b)


def band_crab(f):
    a = abs(f - round(f))
    for (a0, c0), (a1, c1) in zip(CRAB_KEYS, CRAB_KEYS[1:]):
        if a <= a1:
            return lerp(c0, c1, (a - a0) / (a1 - a0))
    return CRAB_KEYS[-1][1]


def clean_half(band, openc, linec):
    out = []
    for i in range(512):
        f = i / 128.0 - 1.0
        if f < 1.5:
            c = band(f)
        else:
            c = openc
            d = abs(f - 1.80)
            if d < 0.05:
                c = lerp(c, linec, 0.6 * (1 - d / 0.05))
        out.append(c)
    return out


def surf_psi(d):
    return 3.2 * d + 0.45 * d * d if d > 0 else 3.2 * d


def surf_half(cols, share, dark, edge, seed):
    rng = random.Random(seed)
    tot = float(sum(share))
    cum = []
    acc = 0.0
    for s in share:
        acc += s / tot
        cum.append(acc)
    seq = [None] * 512
    i = 0
    prev = -1
    while i < 512:
        w = rng.randint(3, 9)
        while True:
            u = rng.random()
            j = next(k for k, c in enumerate(cum) if u <= c)
            if j != prev:
                break
        for k in range(i, min(512, i + w)):
            seq[k] = cols[j]
        prev = j
        i += w
    out = []
    for i in range(512):
        d = i / 128.0 - 0.5
        if d < 0:
            out.append(lerp(edge, seq[i], max(0.0, 1 + d / 0.5) * 0.5))
            continue
        ps = surf_psi(d) % 1.0
        w = min(1.0, max(0.0, 1.6 - abs(ps - 0.5) / 0.10))
        out.append(lerp(seq[i], dark, w))
    # soft joins [1 2 1]
    sm = []
    for i in range(512):
        a = out[max(0, i - 1)]; b = out[i]; c = out[min(511, i + 1)]
        sm.append(tuple((a[k] + 2 * b[k] + c[k]) / 4.0 for k in range(3)))
    return sm


def pal_rgb():
    """2048 RGB tuples in the C_BR_PAL order"""
    sea = clean_half(band_sea, SEA_CYAN, SEA_LINE) + surf_half(SURF_COLS, SURF_SHARE, SURF_DARK, SURF_EDGE, 5)
    crab = clean_half(band_crab, CRAB_OPEN, CRAB_LINE) + surf_half(CRAB_SURF, CRAB_SHARE, CRAB_DARK, CRAB_EDGE, 7)
    return sea + crab


def pal_rom():
    return [word(tuple(int(round(min(255, max(0, v)))) for v in c)) for c in pal_rgb()]


def sin_rom():
    return [int(round(32767 * math.sin(2 * math.pi * (i + 0.5) / 1024))) for i in range(256)]


def exp2_rom():
    return [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by breakers.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package breakers_pkg is",
             vhd_table("C_BR_PAL", "std_logic_vector", 2048, 32, pal_rom()),
             vhd_table("C_BR_SIN", "unsigned", 256, 16, sin_rom()),
             vhd_table("C_BR_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package breakers_pkg;", ""]
    with open(os.path.join(HERE, "breakers_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
