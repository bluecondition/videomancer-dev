#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PLUMAGE v0.1.0 - build hook.  Writes plumage_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).

Fitted to the still WIN_20230710_00_15_14 @ 1.5 s (see .heirloom_work/rb_plumage/notes.md):

  feathers  lanes u = lane0 - G ln(Z), Z = PI - y - K ln(x + 140): parallel log curves (K = 930 = the bend)
            re-spaced by a second log so they crowd toward the lower right (PI = 1448 + 8.32 K);
            a blade = the first w of each lane (crisp dark edge on its upper-left side), shaded across by
            the palette at t = s / (w tap), tapered at both ends of a 15000 px segment that streams inward.
  teeth     rows of a flatter family phi_B = y + 0.375 K ln(x + 140), 170.7 lines apart, one tooth every
            520 px along a row (rounded nose, pointed tip), only in the lowest `rows` rows.
  ground    warm (yellow -> orange -> crimson) / cool (green -> teal -> navy) by x, in a 1600-line band
            that scrolls down; the band's top boundary is lumpy (TOP); a green -> black root strip at x < 190.

Package contents (also read by .heirloom_work/rb_plumage/imodel.py):
  C_PL_KLN   512 x 16   round(16 * 930 ln(1 + m / 512)): the bend K = 930 times the log mantissa (Q4 lines)
  C_PL_KE    13         round(16 * 930 p ln 2), its exponent part
  C_PL_FSRC  512 x 16   fill sources: round(1024 ln(1 + m / 256)) (0..255), the taper C_PL_QA (256..383)
  C_PL_GLN0, C_PL_GE0, C_PL_QAP0   the per-field RAM tables at the default knobs (power-up contents)
  C_PL_QA    128 x 16   feather taper: round(64 / (tap / 127)^0.7), tap 0 -> 4095
  C_PL_QB    256 x 16   tooth: round(64 / (0.42 tooth(f))), capped 4095
  C_PL_PAL   256 x 32   {Y, U, V, 0, solid}: bank * 128 + t * 64 (Plumage = the still, Rainbow = the 00_03_01 look)
  C_PL_BG    256 x 32   {Y, U, V, 00}: mode * 64 + i: cool / warm at x = 32 i + 16, root at x = 8 i + 4
  C_PL_E     19         round(1024 p ln 2), the exponent part of the logs
  C_PL_OFFA  16         feather segment offsets per lane (h units, mod 32768): tips at x 2300..5800 at t = 0
  C_PL_JW    16         feather width code per lane, w (1 + j / 8)
  C_PL_OFFB  16         tooth phase offset per row (1/256 tooth)
  C_PL_TOP   256 x 16   band top boundary (band units = 1.28 lines) by x / 8
  C_PL_EXP2  256 x 16   round(32768 * 2^(i/256))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale with a soft knee, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PAL_PLUME = [(0.00, (55, 0, 60)), (0.07, (62, 0, 95)), (0.17, (78, 0, 172)), (0.31, (95, 0, 182)),
             (0.41, (118, 5, 140)), (0.50, (118, 14, 95)), (0.58, (92, 54, 64)), (0.66, (108, 48, 80)),
             (0.75, (150, 24, 150)), (0.85, (190, 2, 200)), (0.94, (214, 10, 186)), (1.00, (240, 60, 160))]
PAL_RAINBOW = [(0.00, (20, 8, 70)), (0.10, (13, 35, 203)), (0.24, (14, 140, 180)), (0.38, (60, 200, 60)),
               (0.50, (150, 210, 10)), (0.60, (200, 130, 30)), (0.72, (238, 20, 21)), (0.86, (212, 11, 120)),
               (1.00, (150, 25, 160))]
WARM = [(0, (245, 215, 50)), (500, (247, 185, 40)), (1000, (250, 125, 55)), (1400, (253, 48, 55)),
        (1920, (250, 5, 70))]
COOL = [(0, (10, 150, 20)), (500, (50, 190, 85)), (900, (80, 135, 80)), (1200, (30, 100, 105)),
        (1500, (10, 55, 110)), (1920, (5, 15, 118))]
ROOT = [(0, (0, 0, 0)), (12, (5, 160, 50)), (25, (5, 204, 67)), (60, (3, 170, 50)), (100, (2, 60, 18)),
        (125, (0, 8, 4)), (190, (0, 0, 0)), (1920, (0, 0, 0))]
TOPX = [(0, 40), (380, 40), (470, 230), (740, 200), (800, 20), (1130, 20), (1180, 150), (1400, 180),
        (1480, 260), (1920, 260)]
BAND = 1.28          # band units per virtual line (1600-line period -> 2048)
TOOTH_W = 0.42


def interp(x, pts):
    if x <= pts[0][0]:
        return pts[0][1]
    for (x0, c0), (x1, c1) in zip(pts, pts[1:]):
        if x <= x1:
            f = (x - x0) / float(x1 - x0)
            return tuple(c0[i] + (c1[i] - c0[i]) * f for i in range(len(c0))) if isinstance(c0, tuple) \
                else c0 + (c1 - c0) * f
    return pts[-1][1]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    if y2 > 760:
        y2 = 760 + (y2 - 760) * 0.7
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def word(rgb, a=0):
    y, u, v = prescale(rgb_to_hw10(rgb))
    return (y << 22) | (u << 12) | (v << 2) | a


def tooth(f, fn=0.10, g=1.0):
    if f < fn:
        return math.sqrt(f / fn)
    return max(0.0, (1 - f) / (1 - fn)) ** g


K0 = 930


def ln_rom():
    return [int(round(1024 * math.log(1 + m / 512.0))) for m in range(512)]


def kln_rom():
    return [int(round(16 * K0 * math.log(1 + m / 512.0))) for m in range(512)]


def ke_tab():
    return [int(round(16 * K0 * p * math.log(2))) for p in range(13)]


def ln8_rom():
    return [int(round(1024 * math.log(1 + m / 256.0))) for m in range(256)]


G0, RW0 = 914, 123


def gln_tab(g=G0):
    return [(g * v) >> 8 for v in ln8_rom()]


def ge_tab(g=G0):
    out = [((g * v) >> 8) & 16383 for v in e_tab()]
    return out + [0] * (32 - len(out))


def fsrc_rom():
    return ln8_rom() + qa_rom() + [0] * 128


def qap_tab(rw=RW0):
    return [min(4095, (rw * v) >> 6) for v in qa_rom()]


def qa_rom():
    return [4095 if i == 0 else min(4095, int(round(64 / (i / 127.0) ** 0.7))) for i in range(128)]


def qb_rom():
    out = []
    for i in range(256):
        w = TOOTH_W * tooth((i + 0.5) / 256.0)
        out.append(4095 if w <= 64 / 4095.0 else min(4095, int(round(64 / w))))
    return out


def pal_rom():
    out = []
    for tab in (PAL_PLUME, PAL_RAINBOW):
        for i in range(128):
            t = (i + 0.5) / 64.0
            # bit 0: 1 = solid, 0 = the soft trailing fringe (blended half into the ground)
            if t < 1.0:
                out.append(word(interp(t, tab), 1))
            else:
                out.append(word(tab[-1][1], 0))
    return out


def bg_rom():
    out = []
    for mode, tab in enumerate((COOL, WARM, ROOT, ROOT)):
        for i in range(64):
            x = i * 8 + 4 if mode >= 2 else i * 32 + 16
            out.append(word(interp(x, tab)))
    return out


def e_tab():
    return [int(round(1024 * p * math.log(2))) for p in range(19)]


def offa():
    # tip x (virtual px) per lane at t = 0, all right of the frame; segment = a in [0, 30000) h
    tips = [3100, 4700, 2500, 5300, 3700, 2900, 4300, 5700, 2300, 3400, 5000, 2700, 4000, 4900, 3200, 2600]
    return [(30000 - 2 * t) % 32768 for t in tips]


JW = [0, 1, -1, 2, 0, -2, 1, 0, -1, 2, 0, 1, -2, 0, 1, -1]
OFFB = [0, 151, 37, 203, 98, 12, 177, 64, 229, 120, 45, 190, 83, 140, 25, 212]


def int_array(name, vals, rng):
    return ("    type t_%s is array (0 to %d) of integer range %d to %d;" % (name.lower(), len(vals) - 1, rng[0], rng[1])
            + chr(10)
            + "    constant %s : t_%s := (%s);" % (name, name.lower(), ", ".join(str(v) for v in vals)))


def top_rom():
    return [int(round(BAND * interp(i * 8 + 4, TOPX))) for i in range(256)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by plumage.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package plumage_pkg is",
             vhd_table("C_PL_KLN", "unsigned", 512, 16, kln_rom()),
             vhd_table("C_PL_FSRC", "unsigned", 512, 16, fsrc_rom()),
             vhd_table("C_PL_GLN0", "unsigned", 256, 16, gln_tab()),
             vhd_table("C_PL_GE0", "unsigned", 32, 16, ge_tab()),
             vhd_table("C_PL_QAP0", "unsigned", 128, 16, qap_tab()),
             int_array("C_PL_KE", ke_tab(), (0, 131071)),
             vhd_table("C_PL_QB", "unsigned", 256, 16, qb_rom()),
             vhd_table("C_PL_PAL", "std_logic_vector", 256, 32, pal_rom()),
             vhd_table("C_PL_BG", "std_logic_vector", 256, 32, bg_rom()),
             int_array("C_PL_E", e_tab(), (0, 16383)),
             int_array("C_PL_OFFA", offa(), (0, 32767)),
             int_array("C_PL_JW", JW, (-2, 2)),
             int_array("C_PL_OFFB", OFFB, (0, 255)),
             vhd_table("C_PL_TOP", "unsigned", 256, 16, top_rom()),
             vhd_table("C_PL_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package plumage_pkg;", ""]
    with open(os.path.join(HERE, "plumage_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
