#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SALTIRE v0.1.0 - build hook.  Writes saltire_pkg.vhd next to this file (pure python: the builder runs it
with the system python3; imodel.py imports the same tables).

Fitted to the clip still WIN_20230620_00_50_54 at 0.5 s (see .heirloom_work/rb_saltire/notes.md).  The picture
is mirrored about virtual line 517; v = |y - 517|, x in virtual 1920 px.

  band   = one skewed-V band per half: inner edge max(PIN_L(x), 0.99 (x - 1485)), outer edge
           max(POUT_L(x), 1.164 (x - 1393)) -> the two halves nearly touch at the vertex (x 1530): the saltire
  teeth  = the stripe oscillator's 3rd harmonic through a rectifier (scallops), added to the inner edge;
           amplitude grows toward the vertex (NEAR), where the outer edge gets them too (flames)
  stripes= ramp oscillator reset at the line start, 252 px per cycle, FM'd by a vertical sine (224 lines);
           the per-stripe profile (bright edge spike, plateau / ramp, green or teal) is STRIPE[k][q]
  blobs  = cos(x) + cos(v) + 0.9 cos(x + 1.2 v) + top-left corner ramp: soft holes in the blue field and
           red patches in the magenta band (quarter steps)

Package contents:
  C_SL_XROM  512 x 32  by x / 4 (x in virtual px, 0..2047): [9:0] PIN = max(PIN_L, R-arm inner edge), [19:10]
             POUT = max(POUT_L (+ the outer edge's slow lumps), R-arm outer edge), [22:20] NEAR (0..7, the flame
             zone), [29:23] LUMP (yellow drips under the inner edge)
  C_SL_STR   512 x 8   [k(4)][q(5)]: [5:0] stripe level (0..63), [6] teal
  C_SL_TOOTH 512 x 8   [saw][q(8)]: scallop 127 (1 - 2|sin(pi q)|) / saw 254 (q - 1/2), signed
  C_SL_COS   256 x 12  round(2047 cos(2 pi i / 256)), signed
  C_SL_EXP2  256 x 16  round(32768 2^(i / 256))
  C_SL_PAL   512 x 32  [bank][index]: bank 1 = the guns rotated (Guns switch); [29:20] Y, [19:10] U (= Cr), [9:0] V (= Cb), hardware convention, luma pre-scaled 16/13
             (capped 940) for the shell's 13/16 output scale.  Indices:
               0..63 green ground level, 64..127 teal ground level, 128 black, 129 yellow, 130 red,
               131..135 magenta -> red (blob hole quarters), 136..140 blue -> black (hole quarters),
               141..144 magenta -> blue (the band's soft outer edge, quarters),
               145..156 magenta -> white (the neon fringe, 1..12 px before a dark pixel)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# main band, L part (teeth removed), measured on the still
XI = [0, 120, 300, 450, 1050, 1250, 1450, 1530]
VI = [266, 262, 225, 172, 165, 105, 70, 55]
XO = [0, 120, 300, 450, 700, 1050, 1250, 1450, 1530]
VO = [370, 362, 335, 285, 280, 262, 220, 205, 200]
XN = [0, 1100, 1430, 1540, 1600, 4000]
VN = [0, 0, 1, 1, 0, 0]

# stripes per index k: body level, ramp over the stripe, teal, edge spike level
STRIPES = [
    # body  ramp  teal  spike
    (1.00, 0.00, 0, 0.00),
    (0.53, 0.00, 0, 0.91),
    (0.45, 0.00, 1, 0.80),
    (0.24, 0.55, 1, 0.57),
    (0.40, 0.30, 1, 0.60),
    (0.21, 0.00, 0, 0.74),
    (0.20, 0.00, 0, 0.74),
    (0.20, 0.00, 0, 0.74),
    (0.20, 0.00, 0, 0.74),
]
P0 = 252.0
DUTY = 0.41

C_BLACK = (15, 14, 5)
C_YEL = (230, 236, 0)
C_RED = (240, 9, 6)
C_MAG = (247, 4, 250)
C_BLU = (12, 2, 253)
C_WHT = (211, 189, 222)


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / float(xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def xrom():
    out = []
    for i in range(512):
        x = 4 * i
        near = interp(x, XN, VN)
        if x <= 1530:
            pin = interp(x, XI, VI)
            wob = 30 * math.sin(2 * math.pi * x / 460.0 + 2.2) * (1 - near)
            pout = interp(x, XO, VO) + wob
        else:
            pin, pout = 0, 0
        lump = 0.0
        if 200 < x < 760:
            lump = 75 * min(1.0, max(0.0, 1.6 * math.cos(2 * math.pi * (x - 320) / 330.0) - 0.6))
        # the R arm (a straight band climbing from the vertex), exactly as Q4 shift-adds at x = 4 i
        d = 64 * i - 23760
        pin_r = (d - (d >> 6)) >> 4
        e = 64 * i - 22288
        pout_r = (e + (e >> 3) + (e >> 5) + (e >> 7)) >> 4
        pin_i = max(0, min(1023, max(int(round(pin)), pin_r)))
        pout_i = max(0, min(1023, max(int(round(pout)), pout_r)))
        near_i = max(0, min(7, int(round(7 * near))))
        lump_i = max(0, min(127, int(round(lump))))
        out.append(pin_i | (pout_i << 10) | (near_i << 20) | (lump_i << 23))
    return out


def stripe_rom():
    out = []
    for k in range(16):
        body, ramp, teal, spk = STRIPES[min(k, 8)]
        for qi in range(32):
            q = qi / 64.0
            if k == 0:
                lvl = 1.0
            else:
                lvl = body + ramp * (q / DUTY) + max(spk - body, 0.0) * math.exp(-q * P0 / 12.0)
            l6 = max(1, min(63, int(round(lvl * 63))))
            out.append(l6 | (teal << 6))
    return out


def tooth_rom():
    out = []
    for q in range(256):
        out.append(int(round(127 * (1 - 2 * abs(math.sin(math.pi * q / 256.0))))))
    for q in range(256):
        out.append(int(round(254 * (q / 256.0 - 0.5))))
    return [max(-127, min(127, v)) for v in out]


def cos_rom():
    return [int(round(2047 * math.cos(2 * math.pi * i / 256.0))) for i in range(256)]


def exp2_rom():
    return [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    y10 = 64 + y * 876
    y10 = min(940, int(round(64 + (y10 - 64) * 16 / 13.0)))
    u10 = max(24, min(1000, int(round(512 + cr * 896))))
    v10 = max(24, min(1000, int(round(512 + cb * 896))))
    return y10, u10, v10


def mix(a, b, t):
    return tuple(a[i] * (1 - t) + b[i] * t for i in range(3))


def palette_rgb():
    pal = [C_BLACK] * 256
    for l in range(64):
        f = l / 63.0
        pal[l] = (max(C_BLACK[0], 8 * f), max(C_BLACK[1], 255 * f), C_BLACK[2])
        pal[64 + l] = (C_BLACK[0] * (1 - f), max(C_BLACK[1], 255 * f), max(C_BLACK[2], 255 * f * 0.475))
    pal[128] = C_BLACK
    pal[129] = C_YEL
    pal[130] = C_RED
    for h in range(5):
        pal[131 + h] = mix(C_MAG, C_RED, h / 4.0)
        pal[136 + h] = mix(C_BLU, C_BLACK, h / 4.0)
    for e in range(4):
        pal[141 + e] = mix(C_MAG, C_BLU, (e + 1) / 4.0)
    for lev in range(1, 13):
        pal[144 + lev] = mix(C_MAG, C_WHT, lev / 12.0)
    return pal


def palette():
    """512 entries: bank 0 = the photo's guns, bank 1 = the guns rotated (R, G, B) -> (B, R, G): a yellow
    band with green patches, cyan arms, a red field, blue stripes"""
    out = []
    for bank in range(2):
        for rgb in palette_rgb():
            if bank:
                rgb = (rgb[2], rgb[0], rgb[1])
            y, u, v = rgb_to_hw10(rgb)
            out.append((y << 20) | (u << 10) | v)
    return out


def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        elif typ == "std_logic_vector":
            items.append('x"%0*X"' % ((width + 3) // 4, v))
        else:
            items.append('to_unsigned(%d, %d)' % (v, width))
    per = 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by saltire.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package saltire_pkg is",
             vhd_table("C_SL_XROM", "std_logic_vector", 512, 32, xrom()),
             vhd_table("C_SL_STR", "std_logic_vector", 512, 8, stripe_rom()),
             vhd_table("C_SL_TOOTH", "signed", 512, 8, tooth_rom(), signed=True),
             vhd_table("C_SL_COS", "signed", 256, 12, cos_rom(), signed=True),
             vhd_table("C_SL_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_SL_PAL", "std_logic_vector", 512, 32, palette()),
             "end package saltire_pkg;", ""]
    with open(os.path.join(HERE, "saltire_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
