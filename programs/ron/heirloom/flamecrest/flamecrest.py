#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FLAMECREST v0.1.1 - build hook.  Writes flamecrest_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  Heirloom phase 3, clip WIN_20240213_23_38_52 at 3.0 s (see .heirloom_work/rb_flamecrest/).

The patch, as modelled (virtual 1920 x 1080 px; "C" = colouriser units, 8192 per palette cycle, 5851 = 1.0):

  d     = |x - 1010|                          (the still's mirror axis; Twin: 2 | d - 505 |)
  t     = mirror-fold of d * kD on a 2048-unit lattice (1024 units = 940 px: centre .. notch (512) .. side)
  R     = vertical ramp (y + 45) / 1035, with a retrace at y = 990 + ripple(x) (45 lines back to 0)
  s     = P0(t) - su(t) R            (R < Rd = ramp at the diamond row y 505)
        = Q0(t) - sd(t) (R - Rd)     (below)          a VCA: the horizontal shape deepens down the frame
        capped at 0.88, + NP(rd)     (the notch hole and its island, rd = 0.922 |t - 512| + 1.094 |y - 505|)
  r     = T(t) + g(|y - 505|) + A2 S(d)            (Q2 px: asymmetric triangle, two-slope vertical, tongues)
  ring  = 0.5 - frac(r / 100 px + etch * luma)
  pre   = s - 0.05 + 0.24 ring + etch * luma ; Blaze: (pre - 0.12) G + 0.12 + lift ; floor -0.3 (in the black window)
  index = (pre + offset) mod 1.4  ->  palette (4 zone rows from the centre palette to the side palette)
  strokes: hard red where frac(r / 100 px) is within K5 of 0.62 (jittered, where S > -10 px), olive shadow 8 px right

Package contents (all tables also read by imodel.py):
  C_FC_KT    256 x 53   {P0 13u, Q0 14s, su 14u, sd 12u} (C units; su/sd per full ramp) by t >> 2
  C_FC_NP    256 x 14s  notch profile (C units) by rd (px, saturated)
  C_FC_S     256 x 9s   A2 * tongue waveform (Q2 px) by tongue phase
  C_FC_RIP   256 x 8s   bottom ripple (Q2 lines) by x phase (800 px period)
  C_FC_PAL   1024 x 32  {Y, U, V, 00}: zone * 256 + index (hardware convention, U = Cr; luma pre-scaled 16/13, cap 940)
  C_FC_EXP2  256 x 16   round(32768 * 2^(i/256))
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
C1 = 5851.0                     # C units per proto unit (8192 per 1.4-unit cycle)

# knot table over d (px): (d, P0, su, sd)
KN = [(0, 0.78, 0.80, 0.95), (200, 0.79, 0.82, 0.95), (300, 0.82, 1.00, 0.85), (420, 0.96, 1.8, 0.6),
      (470, 1.04, 2.1, 0.6), (540, 1.01, 1.95, 0.55), (650, 0.98, 1.85, 0.5), (1010, 0.98, 1.85, 0.5)]
RD = (505 + 45) / 1035.0        # ramp at the diamond row
HOLE, HD, BUMP, BRI = 0.25, 160.0, 0.85, 70.0
A2, H2, H2P = 50.0, 0.2, 0.8

# palettes over p in [0, 1.4): black window at 0..0.10 and 1.0..1.4 (proto15 stops)
PAL_C = [(0.00, (1, 7, 0)), (0.10, (1, 7, 0)), (0.17, (110, 12, 0)), (0.24, (245, 20, 4)), (0.33, (250, 45, 5)),
         (0.42, (245, 125, 15)), (0.51, (240, 212, 12)), (0.60, (245, 238, 110)), (0.70, (250, 205, 215)),
         (0.80, (245, 110, 200)), (0.88, (248, 165, 232)), (0.96, (150, 60, 130)), (1.03, (40, 10, 30)), (1.08, (1, 7, 0))]
PAL_S = [(0.00, (1, 7, 0)), (0.10, (1, 7, 0)), (0.15, (60, 30, 0)), (0.21, (190, 45, 4)), (0.28, (190, 200, 10)),
         (0.39, (150, 245, 40)), (0.47, (40, 248, 140)), (0.56, (20, 235, 235)), (0.65, (160, 235, 238)),
         (0.75, (10, 128, 252)), (0.84, (200, 100, 205)), (0.89, (240, 190, 240)), (0.96, (150, 60, 130)), (1.03, (40, 10, 30)), (1.08, (1, 7, 0))]
ZONE_BLEND = [0.0, 1 / 3.0, 2 / 3.0, 1.0]
RED = (250, 22, 5)
OLIVE = (45, 50, 5)


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def word(yuv, prescale=True):
    y, u, v = yuv
    if prescale:
        y = 64 + (y - 64) * 16.0 / 13.0
    y = max(64, min(940, int(round(y))))
    u = max(4, min(1019, int(round(u))))
    v = max(4, min(1019, int(round(v))))
    return (y << 22) | (u << 12) | (v << 2)


def kt_rom():
    out = []
    xs = [k[0] for k in KN]
    for i in range(256):
        t = i * 4 + 2                       # t units (0.918 px)
        dpx = t * 940.0 / 1024.0
        p0 = interp(dpx, xs, [k[1] for k in KN])
        su = interp(dpx, xs, [k[2] for k in KN])
        sd = interp(dpx, xs, [k[3] for k in KN])
        P0 = int(round(p0 * C1)); SU = int(round(su * C1)); SD = int(round(sd * C1))
        Q0 = int(round((p0 - su * RD) * C1))
        P0 = min(P0, 8191); SU = min(SU, 16383); SD = min(SD, 4095)
        out.append((P0 << 40) | ((Q0 & 0x3FFF) << 26) | (SU << 12) | SD)
    return out


def kt_fields(w):
    P0 = (w >> 40) & 0x1FFF
    Q0 = (w >> 26) & 0x3FFF
    if Q0 >= 0x2000:
        Q0 -= 0x4000
    SU = (w >> 12) & 0x3FFF
    SD = w & 0xFFF
    return P0, Q0, SU, SD


def np_rom():
    out = []
    for rd in range(256):
        v = BUMP * max(0.0, 1 - rd / BRI) - HOLE * max(0.0, 1 - rd / HD)
        out.append(int(round(v * C1)))
    return out


def s_rom():
    out = []
    for i in range(256):
        ph = (i + 0.5) / 256.0
        v = math.cos(2 * math.pi * ph) + H2 * math.cos(4 * math.pi * ph + H2P)
        out.append(max(-255, min(255, int(round(A2 * v * 4)))))
    return out


def rip_rom():
    out = []
    for i in range(256):
        x = (i + 0.5) / 256.0 * 800.0       # px over one 800 px period (400 and 266.7 px sines)
        v = 14 * math.sin(2 * math.pi * x / 400.0) + 9 * math.sin(2 * math.pi * x / 266.667 + 1.3)
        out.append(max(-127, min(127, int(round(v * 4)))))
    return out


def pal_rgb(stops, p):
    if p > 1.08:
        return (1, 7, 0)
    xs = [s[0] for s in stops]
    return tuple(interp(p, xs, [s[1][k] for s in stops]) for k in range(3))


def pal_rom():
    out = []
    for z in range(4):
        b = ZONE_BLEND[z]
        for i in range(256):
            p = (i + 0.5) * 1.4 / 256.0
            c0 = pal_rgb(PAL_C, p); c1 = pal_rgb(PAL_S, p)
            rgb = tuple(c0[k] * (1 - b) + c1[k] * b for k in range(3))
            out.append(word(rgb_to_hw10(rgb)))
    return out


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def consts():
    """compose-stage constants (hardware convention, luma pre-scaled)"""
    red = word(rgb_to_hw10(RED)); olv = rgb_to_hw10(OLIVE)
    oy = 64 + (olv[0] - 64) * 16.0 / 13.0
    return dict(RED=((red >> 22) & 1023, (red >> 12) & 1023, (red >> 2) & 1023),
                OLV=(int(round(0.75 * oy)), int(round(0.75 * olv[1])), int(round(0.75 * olv[2]))))


def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        elif typ == "std_logic_vector":
            bits = format(v & ((1 << width) - 1), '0%db' % width)
            items.append('"%s"' % bits if width % 4 else 'x"%0*X"' % (width // 4, v))
        else:
            items.append('to_unsigned(%d, %d)' % (v, width))
    per = 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    c = consts()
    parts = ["-- Generated by flamecrest.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package flamecrest_pkg is",
             vhd_table("C_FC_KT", "std_logic_vector", 256, 53, kt_rom()),
             vhd_table("C_FC_NP", "signed", 256, 14, np_rom(), signed=True),
             vhd_table("C_FC_S", "signed", 256, 9, s_rom(), signed=True),
             vhd_table("C_FC_RIP", "signed", 256, 8, rip_rom(), signed=True),
             vhd_table("C_FC_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             vhd_table("C_FC_EXP2", "unsigned", 256, 16, exp2_rom()),
             "    constant C_RED_Y : integer := %d;" % c['RED'][0],
             "    constant C_RED_U : integer := %d;" % c['RED'][1],
             "    constant C_RED_V : integer := %d;" % c['RED'][2],
             "    constant C_OLV_Y : integer := %d;" % c['OLV'][0],
             "    constant C_OLV_U : integer := %d;" % c['OLV'][1],
             "    constant C_OLV_V : integer := %d;" % c['OLV'][2],
             "end package flamecrest_pkg;", ""]
    with open(os.path.join(HERE, "flamecrest_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
