#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LOZENGE v0.5.0 - build hook.  Writes lozenge_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).

Fitted to the photo WIN_20230410_01_10_37 (see .heirloom_work/rb_lozenge/notes.md):

  t      = aL|x - 993| (left, aL = 1.0102) or |x - 993| (right) + k|y - 518.6|   (k 1.800 above, 1.790 below)
           the diamond field (H triangle + V triangle), held at its x = 66 value at the line start
  u      = frac((t - flow) / 915.2)                  a sawtooth colouriser, one cycle per 915.2 px of field
  band   = black  if |u - 0.2611| < 0.0962 or |u - 0.9834| < 0.0962 (wrapping)
           ring   (navy / grey) between the two black bands
           B      (olive / pink) if |u - 0.6218| < 0.1847, else A (pink / olive)
           eye    (grey / navy) where t < 72 (a small diamond at the centre)
  stripe = parity of the stripe phase psi(y): a vertical square wave whose frequency glides from
           1/142.6 to 1/53.5 cycles per line, f = (1/87.37) 2^(1.415 (sigmoid((y - 496.35) / 74.2) - 1/2));
           pink-type stripe when frac(psi) < 1/2 (psi(0) = -0.4024).  It swaps the pairs above.
  lines  = a one-shot at every black band's leading edge (scan order), drawn over the black: yellow after
           olive or navy, light pink after pink, a double white line after the grey ring (LP_PHOTO, the
           photo's line profiles deconvolved from its horizontal blur; the RTL's two 9-tap boxes add it back)

Package contents (all tables also read by imodel.py):
  C_LZ_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_LZ_SIG   256 x 12   (sigmoid((8 i + 4 - 496.35) / 74.2) - 1/2) * 4096, signed; i = virtual line / 8
  C_LZ_PROF  1024 x 32  {Y, U, V, 00} line profiles: index = pal * 512 + glow * 256 + type * 64 + offset
                        (road: virtual px 0..63; glow: 4 virtual px per entry, 0..255)
  C_LZ_BASE  32 x 32    {Y, U, V, 00} band colours: index = pal * 16 + band * 2 + s (band 0 K, 1 eye, 2 ring, 3 A, 4 B)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (virtual px / lines)
XC, CY, AL, KT, KB = 993.0, 518.6, 1.0102, 1.800, 1.790
XH = 66.0
PER = 915.2
U_K1, U_K2, U_HK, U_B, U_HB = 0.2611, 0.9834, 0.0962, 0.6218, 0.1847
EYE = 72.0
FM, CH, YM, YW, PH0 = 1 / 87.37, 1.415, 496.35, 74.2, -0.4024

# band colours (photo RGB, palette 0) and the alternative palette 1 ("Cyan Rust")
PAL_RGB = [
    dict(K=(17, 18, 10), eye=((25, 26, 18), (22, 26, 54)), ring=((22, 26, 54), (25, 26, 18)),
         A=((254, 10, 187), (58, 61, 9)), B=((58, 61, 9), (254, 10, 187))),
    dict(K=(14, 12, 20), eye=((46, 18, 52), (10, 40, 44)), ring=((10, 40, 44), (46, 18, 52)),
         A=((20, 205, 230), (150, 52, 12)), B=((150, 52, 12), (20, 205, 230))),
]
# palette-1 line hues per type (Y line after ring/olive-type, WW after grey ring, W after pink, Y after olive)
LINE1_RGB = [(205, 250, 255), (255, 214, 150), (200, 255, 255), (255, 170, 70)]

# Line profiles of the photo (hardware 10-bit Y, U = Cr, V = Cb, NOT pre-scaled), offset 0..63 virtual px
# after the black band's leading edge.  Types: 0 = after navy ring / olive-type stripe (yellow),
# 1 = after grey ring / pink-type (double white), 2 = after pink A band (light pink), 3 = after olive A (yellow).
LP_PHOTO = [
    # ringO_Y
    [(126, 616, 547), (153, 613, 535), (205, 615, 513), (276, 624, 483), (358, 639, 446), (443, 657, 402), (526, 675, 355), (599, 689, 306), 
     (660, 695, 259), (703, 692, 220), (723, 680, 193), (716, 659, 182), (680, 634, 189), (618, 607, 212), (537, 581, 248), (449, 558, 292), 
     (364, 540, 338), (292, 526, 379), (236, 518, 412), (200, 514, 433), (181, 513, 446), (174, 513, 453), (172, 514, 457), (168, 515, 460), 
     (161, 515, 464), (145, 515, 475), (131, 514, 484), (123, 513, 492), (121, 513, 496), (123, 512, 498), (125, 512, 498), (125, 512, 498), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     ],
    # ringP_WW
    [(99, 609, 493), (139, 605, 494), (208, 603, 495), (297, 603, 496), (399, 606, 497), (505, 610, 497), (608, 613, 497), (701, 614, 496), 
     (775, 610, 495), (822, 600, 495), (834, 585, 494), (805, 565, 495), (737, 545, 496), (638, 526, 497), (522, 510, 497), (403, 500, 498), 
     (296, 495, 498), (211, 493, 498), (153, 496, 498), (121, 500, 498), (110, 506, 498), (111, 514, 498), (119, 522, 498), (127, 531, 498), 
     (136, 542, 498), (150, 554, 498), (175, 568, 498), (215, 582, 498), (276, 595, 498), (357, 606, 498), (454, 613, 498), (559, 616, 498), 
     (660, 614, 498), (745, 606, 498), (802, 594, 498), (823, 579, 498), (803, 562, 498), (741, 545, 498), (646, 530, 498), (529, 517, 498), 
     (407, 508, 498), (298, 502, 498), (216, 501, 499), (168, 502, 499), (153, 504, 499), (159, 508, 498), (172, 511, 498), (177, 513, 498), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     ],
    # bandO_P_W
    [(432, 849, 634), (450, 841, 636), (487, 825, 636), (538, 802, 632), (595, 774, 623), (649, 742, 611), (690, 708, 597), (710, 674, 581), 
     (704, 641, 567), (669, 611, 554), (610, 585, 544), (532, 564, 536), (446, 547, 529), (362, 534, 524), (287, 526, 519), (227, 519, 514), 
     (186, 515, 510), (162, 513, 506), (153, 511, 504), (152, 510, 502), (154, 510, 501), (154, 510, 500), (151, 510, 500), (144, 512, 499), 
     (134, 513, 499), (124, 514, 498), (119, 515, 498), (118, 514, 498), (121, 514, 498), (125, 513, 498), (127, 512, 498), (126, 512, 498), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     ],
    # bandP_O_Y
    [(248, 660, 413), (271, 648, 402), (314, 636, 383), (371, 633, 359), (437, 640, 332), (506, 653, 301), (574, 670, 269), (634, 684, 237), 
     (682, 691, 209), (711, 688, 187), (718, 674, 175), (702, 651, 176), (662, 624, 190), (602, 596, 218), (527, 571, 254), (445, 549, 296), 
     (364, 533, 339), (292, 522, 378), (234, 515, 410), (193, 512, 434), (168, 511, 450), (156, 512, 460), (150, 512, 465), (148, 513, 469), 
     (145, 513, 474), (138, 513, 481), (131, 513, 487), (126, 512, 493), (124, 512, 496), (124, 512, 497), (125, 512, 498), (124, 512, 498), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), (122, 513, 499), 
     ],
]

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

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def sig_rom():
    out = []
    for i in range(256):
        yv = 8 * i + 4
        s = 1.0 / (1.0 + math.exp(-(yv - YM) / YW)) - 0.5
        out.append(max(-2047, min(2047, int(round(s * 4096)))))
    return out

def base_rom():
    out = [0] * 32
    for p, pal in enumerate(PAL_RGB):
        for s in (0, 1):
            out[p * 16 + 0 * 2 + s] = word(rgb_to_hw10(pal['K']))
            for b, key in ((1, 'eye'), (2, 'ring'), (3, 'A'), (4, 'B')):
                out[p * 16 + b * 2 + s] = word(rgb_to_hw10(pal[key][s]))
        for i in range(10, 16):
            out[p * 16 + i] = word(rgb_to_hw10(pal['K']))
    return out

def glow_profile(prof, kyuv):
    """Glow: a long luminous decay behind the edge in the colour of the line's peak.  Entry c covers
    virtual px 4c .. 4c + 3 (the RTL indexes the glow table at a quarter of the road resolution)."""
    pk = max(prof[:48], key=lambda e: e[0])
    out = []
    for c in range(64):
        px = 4 * c + 1.5
        g = min(1.0, (px + 1) / 6.0) * math.exp(-max(0.0, px - 6) / 55.0)
        if c >= 56:
            g *= (63 - c) / 8.0
        out.append(tuple(kyuv[i] + g * (pk[i] - kyuv[i]) for i in range(3)))
    return out

def recolour(prof, kyuv, tgt):
    ty, tu, tv = rgb_to_hw10(tgt)
    out = []
    for (y, u, v) in prof:
        w = (y - kyuv[0]) / float(max(1, ty - kyuv[0]))
        w = min(1.2, max(0.0, w))
        out.append((y, kyuv[1] + w * (tu - kyuv[1]), kyuv[2] + w * (tv - kyuv[2])))
    return out

def prof_rom():
    out = [0] * 1024
    k0 = rgb_to_hw10(PAL_RGB[0]['K'])
    k1 = rgb_to_hw10(PAL_RGB[1]['K'])
    for p in (0, 1):
        for glow in (0, 1):
            for t in range(4):
                prof = LP_PHOTO[t]
                kk = k0
                if p == 1:
                    # shift the photo profile's black floor to palette 1's black, then recolour
                    prof = [(y - k0[0] + k1[0], u - k0[1] + k1[1], v - k0[2] + k1[2]) for (y, u, v) in prof]
                    prof = recolour(prof, k1, LINE1_RGB[t])
                    kk = k1
                if glow:
                    prof = glow_profile(prof, kk)
                for c in range(64):
                    y, u, v = prof[c]
                    out[p * 512 + glow * 256 + t * 64 + c] = word((int(round(y)), int(round(u)), int(round(v))))
    return out

def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        else:
            items.append('x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width))
    per = 8 if signed else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    parts = ["-- Generated by lozenge.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package lozenge_pkg is",
             vhd_table("C_LZ_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_LZ_SIG", "signed", 256, 12, sig_rom(), signed=True),
             vhd_table("C_LZ_PROF", "std_logic_vector", 1024, 32, prof_rom()),
             vhd_table("C_LZ_BASE", "std_logic_vector", 32, 32, base_rom()),
             "end package lozenge_pkg;", ""]
    with open(os.path.join(HERE, "lozenge_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
