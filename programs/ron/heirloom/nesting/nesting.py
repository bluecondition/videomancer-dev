#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NESTING v0.1.0 - build hook.  Writes nesting_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Built from the clip WIN_20230627_23_37_56 at 7.0 s (see .heirloom_work/rb_nesting/notes.md).  The patch:

  layer A (the contour field): H ramp mirrored at x 1018 and folded again at the eyes (+-471 px), V ramp folded at
      y 510; both through triangle-wave folders at a swept gain g (the breathing LFO), combined in a max mixer with a
      little of the min:  F = max(Tu, Tv) + k min(Tu, Tv)  (k 0.15: bevelled squares), into a colouriser.
  layer B (the magenta key): the same two-eye geometry (x 1038, y 516) at a fixed gain, keyed in rings 97 px apart.

Package contents:
  C_NS_PAL   1024 x 32   {Y, U, V, 00}: entries 0..511 = Photo (the still), 512..1023 = Ember.  Entry i holds the
                         colouriser at F * 471 = i * 471 / 320 px-units (the pixel path indexes F_q10 * 320 >> 10).
                         Hardware convention (U register = Cr, V = Cb), luma pre-scaled 16/13 for the shell's 13/16
                         output scale, capped at 940
  C_NS_BCOL   256 x 32   the magenta key's colours (entries 0..31; the rest pad the block RAM): index = q * 4 + ring2 * 2
                         + edge (q = 0 purple .. 4 magenta)
  C_NS_COS    256 x 10   round(511 cos(pi i / 256)), two's complement (half wave)
  C_NS_EXP2   256 x 16   round(32768 * 2^(i/256))
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

C = dict(blk=(37, 18, 23), nvy=(28, 50, 155), dnv=(22, 29, 91), BLU=(19, 66, 234), CYN=(37, 128, 252),
         lav=(138, 93, 235), drd=(113, 12, 10), RED=(200, 12, 2), gry=(108, 66, 89), org=(220, 70, 10))
# the colouriser along the axis from the eye centre, px at gain 1 (measured on the still, both sides and up/down)
PA_BP = [(0, 'nvy'), (12, 'dnv'), (22, 'blk'), (38, 'blk'), (46, 'dnv'), (54, 'BLU'), (98, 'BLU'), (106, 'nvy'),
         (114, 'dnv'), (128, 'dnv'), (134, 'nvy'), (142, 'BLU'), (156, 'CYN'), (174, 'CYN'), (182, 'lav'),
         (194, 'lav'), (204, 'CYN'), (214, 'BLU'), (226, 'BLU'), (232, 'nvy'), (236, 'dnv'), (240, 'blk'),
         (258, 'blk'), (264, 'dnv'), (270, 'nvy'), (284, 'BLU'), (296, 'nvy'), (304, 'dnv'), (310, 'blk'),
         (328, 'blk'), (334, 'dnv'), (340, 'nvy'), (348, 'BLU'), (362, 'BLU'), (368, 'lav'), (382, 'lav'),
         (392, 'BLU'), (400, 'nvy'), (408, 'dnv'), (414, 'blk'), (424, 'drd'), (436, 'RED'), (446, 'drd'),
         (454, 'blk'), (466, 'blk'), (474, 'drd'), (486, 'RED'), (506, 'RED'), (516, 'drd'), (524, 'blk'),
         (532, 'dnv'), (540, 'nvy'), (552, 'BLU'), (560, 'nvy'), (568, 'gry'), (586, 'gry'), (594, 'drd'),
         (602, 'RED'), (616, 'org'), (626, 'RED'), (634, 'drd'), (642, 'blk'), (650, 'dnv'), (660, 'nvy'),
         (672, 'gry'), (690, 'gry'), (700, 'nvy'), (712, 'BLU'), (722, 'nvy'), (732, 'blk'), (744, 'drd'),
         (760, 'RED')]
PX_PER_ENTRY = 471.0 / 320.0

# the magenta key (median RGB of the still's key pixels by distance from the centre column and ring)
MAG_IN = (198, 0, 184)     # rings 0, 1
MAG_OUT = (158, 1, 134)    # ring 2 and beyond
PURPLE = (110, 0, 218)     # at the centre column
EDGE = 0.85                # ring edges darkened


def interp(d):
    d = max(0.0, min(760.0, d))
    for (d0, c0), (d1, c1) in zip(PA_BP[:-1], PA_BP[1:]):
        if d0 <= d <= d1:
            t = 0.0 if d1 == d0 else (d - d0) / (d1 - d0)
            a, b = C[c0], C[c1]
            return tuple(a[k] + (b[k] - a[k]) * t for k in range(3))
    return C[PA_BP[-1][1]]


def photo_rgb(i):
    return interp(i * PX_PER_ENTRY)


def ember_rgb(i):
    r, g, b = photo_rgb(i)
    return (min(255.0, max(b, 0.9 * r)), min(255.0, 0.75 * b + 0.35 * g + 0.1 * r), min(255.0, 0.25 * g + 0.45 * r))


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
    return [word(rgb_to_hw10(photo_rgb(i))) for i in range(512)] + \
           [word(rgb_to_hw10(ember_rgb(i))) for i in range(512)]


def bcol_rgb(q, ring2, edge):
    w = [1.0, 0.75, 0.5, 0.25, 0.0][min(q, 4)]
    outer = MAG_OUT if ring2 else MAG_IN
    c = tuple(w * PURPLE[k] + (1 - w) * outer[k] for k in range(3))
    if edge:
        c = tuple(EDGE * v for v in c)
    return c


def bcol_rom():
    rows = []
    for i in range(32):
        q, ring2, edge = i >> 2, (i >> 1) & 1, i & 1
        rows.append(word(rgb_to_hw10(bcol_rgb(q, ring2, edge))))
    return rows


def cos_rom():
    return [int(round(511 * math.cos(math.pi * i / 256.0))) & 1023 for i in range(256)]


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
    parts = ["-- Generated by nesting.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package nesting_pkg is",
             vhd_table("C_NS_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             vhd_table("C_NS_BCOL", "std_logic_vector", 256, 32, bcol_rom() + [0] * 224),
             vhd_table("C_NS_COS", "unsigned", 256, 10, cos_rom()),
             vhd_table("C_NS_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package nesting_pkg;", ""]
    with open(os.path.join(HERE, "nesting_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
