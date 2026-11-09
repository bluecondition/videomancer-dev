#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BRAZIER v0.1.0 - build hook.  Writes brazier_pkg.vhd next to this file (pure python: the builder runs it with
the system python3; imodel.py in .heirloom_work/rb_brazier imports the same tables).

Fitted to the still WIN_20230930_01_37_53 @ 0.5 s (see .heirloom_work/rb_brazier/notes.md):

  fins     phi = F(y) (x + 134) + 0.7786 cycles,  F(y) = (1/226.66) (1 - 0.1408 cos(2 pi (y - 338 - 570 L) / 570))
           (x, y virtual px; L = the LFO phase, one cycle per 5.5 s in the clip: the FM wave scrolls down)
           lit where frac(phi) > T(y) (+ 0.25 on every other fin: the sub-octave makes big / small fins);
           colour by 1 - frac(phi) (the rim is the reset); T(y) = the three bands (crescents / fins / flames)
  diamond  D = kx |x - cx(y)| + V(y),  V = 0.953 (cy - y) above, 522 (1 - exp(-(y - cy) / 400)) below
           g = m (0.45 + 7.8594e-3 D - 1.8971e-5 D^2 + 2.2634e-8 D^3): resets at D = 86, 398, 575
           colour index c = frac(g), reversed (1 - frac) right of the centre; keyed D < 575; a dark outline
           to 589; ring palette per vertical zone (8 zones: darker / greener at the top, magenta at the bottom)
  ground   a vertical ramp: dark brown at the top, near black at y 240, bright red at the bottom

Package contents:
  C_BZ_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_BZ_SV    512 x 16   0..255: round(2047 sin(2 pi i / 256)) (signed); 256..511: V below the centre,
                        round(522 (1 - exp(-(4 j + 2) / 400))), j = i - 256 (t = 4 j virtual lines)
  C_BZ_BG    256 x 32   {Y, U, V, band}: ground colour and fin band for virtual line y = (64 i + 32) / 15
  C_BZ_TT    512 x 8    fin threshold T * 256 (255 = no fins) for y = (32 i + 16) / 15
  C_BZ_BRE   256 x 32   breath, one LFO cycle: {kx - 1 (Q8, 10b), (cy - 515) / 2 (8b), wobble px (8b), jitter Q3 (6b)}
  C_BZ_RPAL  512 x 32   {Y, U, V, 00} ring colours: zone * 64 + c (c = colour index, 6 bits)
  C_BZ_FPAL  256 x 32   {Y, U, V, 00} fin colours: band * 64 + q (q = 1 - frac(phi), 6 bits)
Colours in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's 13/16
output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ colour helpers (hwcolor.py, inline)
def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))

def prescale(yuv):
    y, u, v = yuv
    y = 64 + (y - 64) * 16.0 / 13.0
    y = min(940, max(64, int(round(y))))
    return (y, min(1023, max(0, u)), min(1023, max(0, v)))

def word(yuv, low=0):
    y, u, v = yuv
    return (y << 22) | (u << 12) | (v << 2) | (low & 3)

def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / float(xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]

def ramp(stops, n):
    pos = [s[0] for s in stops]
    out = []
    for i in range(n):
        t = (i + 0.5) / n
        out.append(tuple(interp(t, pos, [s[1][k] for s in stops]) for k in range(3)))
    return out

# ------------------------------------------------------------------ ground (measured column profile)
BG_Y = [15, 45, 75, 105, 135, 165, 195, 225, 255, 285, 315, 345, 375, 405, 435, 465, 495, 525, 555, 585, 615, 645,
        675, 705, 735, 765, 795, 825, 855, 885, 915, 945, 975, 1005, 1035, 1079]
BG_R = [35, 31, 27, 22.5, 17, 14.5, 12.5, 9.5, 8, 10.5, 13, 16, 21, 26.5, 32, 39, 45, 50, 56, 62, 69, 77, 83, 91.5, 98,
        107, 115.5, 124.5, 133, 143, 151.5, 159.5, 171, 180, 186, 190]
BG_G = [16, 16, 16, 16, 15.5, 16, 15.5, 14, 14, 14.5, 14, 13.5, 13, 10.5, 10, 13, 12.5, 11.5, 9.5, 9, 8, 8, 8, 7.5,
        7.5, 7.5, 8.5, 7.5, 7.5, 7, 6, 5.5, 4, 6, 4, 3]

def band_of(y):
    return 0 if y < 250 else (1 if y < 760 else 2)

def bg_rom():
    out = []
    for i in range(256):
        y = (64 * i + 32) / 15.0
        rgb = (interp(y, BG_Y, BG_R), interp(y, BG_Y, BG_G), 0.0)
        out.append(word(prescale(rgb_to_hw10(rgb)), band_of(y)))
    return out

# ------------------------------------------------------------------ fin thresholds (measured)
T_Y = [-60, 4, 20, 40, 60, 80, 100, 120, 145, 380, 420, 450, 480, 510, 540, 570, 600, 620, 654, 680, 830, 856, 880,
       904, 928, 952, 976, 1000, 1024, 1036, 1079]
T_V = [0.13, 0.407, 0.475, 0.565, 0.648, 0.735, 0.825, 0.92, 1.05, 1.05, 0.835, 0.709, 0.585, 0.46, 0.53, 0.604, 0.70,
       0.804, 0.936, 1.05, 1.08, 0.963, 0.85, 0.73, 0.60, 0.465, 0.37, 0.275, 0.161, 0.0, -0.2]

def tt_rom():
    out = []
    for i in range(512):
        y = (32 * i + 16) / 15.0
        t = interp(y, T_Y, T_V)
        out.append(255 if t >= 0.995 else max(0, min(254, int(round(t * 256)))))
    return out

# ------------------------------------------------------------------ breath (one LFO cycle, measured on the clip)
BK = [(0, 1.0, 515, 8, 1.0), (0.09, 1.03, 515, 10, 1.0), (0.18, 1.12, 515, 14, 1.2), (0.27, 1.4, 520, 30, 1.6),
      (0.36, 1.9, 530, 50, 2.2), (0.45, 2.8, 560, 60, 3.0), (0.5, 3.2, 800, 55, 3.0), (0.545, 2.6, 950, 45, 2.6),
      (0.64, 1.7, 1000, 30, 2.0), (0.73, 1.3, 980, 20, 1.5), (0.82, 1.15, 885, 14, 1.2), (0.91, 1.05, 760, 10, 1.0),
      (1.0, 1.0, 515, 8, 1.0)]

def breath(phi):
    xs = [k[0] for k in BK]
    return [interp(phi % 1.0, xs, [k[i] for k in BK]) for i in (1, 2, 3, 4)]

def bre_rom():
    out = []
    for i in range(256):
        kx, cy, wb, jb = breath(i / 256.0)
        kxm = min(1023, int(round((kx - 1) * 256)))
        cyo = min(255, int(round((cy - 515) / 2.0)))
        out.append((kxm << 22) | (cyo << 14) | (min(255, int(round(wb))) << 6) | min(63, int(round(jb * 8))))
    return out

# ------------------------------------------------------------------ ring palettes (8 vertical zones)
RING_BODY = [(0.0, (245, 25, 253)), (0.05, (215, 9, 253)), (0.13, (160, 8, 252)), (0.26, (86, 6, 243)),
             (0.40, (28, 8, 210)), (0.55, (14, 10, 120)), (0.70, (9, 12, 30))]
ZG = [1.3, 1.15, 1.0, 0.9, 0.72, 0.58, 0.52, 0.52]
ZGW = [0.13, 0.11, 0.09, 0.07, 0.07, 0.07, 0.07, 0.07]
ZDEEP = [(10, 22, 6), (9, 16, 6), (8, 13, 6), (8, 13, 6), (8, 13, 6), (8, 13, 6), (8, 13, 6), (8, 13, 6)]
ZRIM = [(170, 135, 195), (205, 128, 225), (230, 120, 242), (235, 118, 245), (238, 112, 248), (240, 100, 250),
        (245, 90, 250), (245, 90, 250)]
FRINGE_G = (38, 64, 16)
FRINGE_R = (105, 8, 3)

def ring_rgb(z):
    body = ramp(RING_BODY + [(0.82, ZDEEP[z]), (1.0, ZDEEP[z])], 256)
    out = []
    for i in range(64):
        c = (i + 0.5) / 64
        if c < 0.035:
            out.append(ZRIM[z])
        elif c >= 0.935:
            out.append(FRINGE_R)
        elif c >= 0.935 - ZGW[z]:
            out.append(FRINGE_G)
        else:
            b = min((c - 0.035) / (0.9 - ZGW[z]) * ZG[z], 0.999)
            out.append(body[int(b * 256)])
    return out

def rpal_rom():
    out = []
    for z in range(8):
        for rgb in ring_rgb(z):
            out.append(word(prescale(rgb_to_hw10(rgb))))
    return out

# ------------------------------------------------------------------ fin palettes (fitted: median colour by band and q)
FIN_RGB = [
  [(153,149,170), (153,149,170), (153,149,170), (154,154,169), (148,145,164), (132,124,147), (112,104,125), (96,92,109), (88,86,102), (87,85,100), (88,86,101), (90,86,103), (91,86,104), (92,86,104), (92,87,104), (91,87,102), (89,87,99), (87,88,98), (86,89,96), (86,90,94), (87,92,95), (89,95,98), (92,98,100), (94,101,103), (96,104,105), (98,106,106), (99,108,107), (101,110,108), (102,112,109), (104,112,108), (103,107,101), (96,94,86), (81,74,63), (65,57,42), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34), (60,50,34)],
  [(215,80,250), (215,80,250), (176,35,251), (180,33,254), (183,32,255), (186,30,255), (188,24,255), (188,18,255), (187,13,255), (187,10,255), (186,10,255), (185,11,255), (184,12,255), (184,12,255), (183,11,255), (183,9,255), (182,8,255), (183,8,255), (183,8,255), (183,8,255), (184,8,255), (186,9,255), (188,10,255), (190,12,255), (192,13,255), (194,14,254), (196,17,254), (198,20,254), (199,23,253), (200,27,251), (202,30,249), (202,34,244), (195,36,220), (183,36,182), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164), (177,36,164)],
  [(250,70,255), (250,70,255), (246,6,255), (244,2,255), (242,0,255), (241,0,255), (241,0,255), (242,0,255), (242,0,255), (242,0,255), (243,0,255), (245,0,255), (247,0,255), (248,0,255), (250,0,255), (252,0,255), (253,0,255), (254,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,0,255), (255,1,254), (255,2,252), (255,3,250), (255,3,246), (254,3,238), (253,4,229), (251,4,218), (248,4,208), (245,4,202), (243,4,196), (242,5,193), (241,6,194), (240,7,202), (240,8,221), (243,10,242), (250,13,253), (254,16,255), (255,16,255), (255,13,255), (255,8,255), (255,3,255), (255,0,255)],
]

def fin_rgb():
    p = [list(b) for b in FIN_RGB]
    # the fitted tails beyond the fins' widest q are leading-edge / ground mixes: hold the last fin colour
    for k in range(31, 64):
        p[0][k] = p[0][30]
    for k in range(32, 64):
        p[1][k] = p[1][31]
    return p

def fpal_rom():
    p = fin_rgb()
    out = []
    for b in range(4):
        for k in range(64):
            out.append(word(prescale(rgb_to_hw10(p[min(b, 2)][k]))))
    return out

# ------------------------------------------------------------------ small tables
def exp2_rom():
    return [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]

def sv_rom():
    out = []
    for i in range(256):
        out.append(int(round(2047 * math.sin(2 * math.pi * i / 256.0))) & 0xFFFF)
    for j in range(256):
        out.append(int(round(522 * (1 - math.exp(-(4 * j + 2) / 400.0)))))
    return out

# ring law (per table entry i, D = 2 i px), Q32 forward differences for m = 1 (scaled by m8 / 256 per field)
A0, A1, A2, A3 = 0.45, 0.007859423824617828, -1.8970687636764454e-05, 2.2634417165118747e-08

def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    if width % 4 == 0:
        items = ['x"%0*X"' % (width // 4, v) for v in vals]
    else:
        items = ['"' + format(v, '0%db' % width) + '"' for v in vals]
    per = 16 if width <= 16 else 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    parts = ["-- Generated by brazier.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package brazier_pkg is",
             vhd_table("C_BZ_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_BZ_SV", "std_logic_vector", 512, 16, sv_rom()),
             vhd_table("C_BZ_BG", "std_logic_vector", 256, 32, bg_rom()),
             vhd_table("C_BZ_TT", "unsigned", 512, 8, tt_rom()),
             vhd_table("C_BZ_BRE", "std_logic_vector", 256, 32, bre_rom()),
             vhd_table("C_BZ_RPAL", "std_logic_vector", 512, 32, rpal_rom()),
             vhd_table("C_BZ_FPAL", "std_logic_vector", 256, 32, fpal_rom()),
             "end package brazier_pkg;", ""]
    with open(os.path.join(HERE, "brazier_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
