#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
HORNET v0.1.1 - build hook.  Writes hornet_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the still WIN_20240207_23_34_28 at 0.5 s (see .heirloom_work/rb_hornet/notes.md):

  theta = (x - 535) / 950                                   the eye oscillator (virtual px)
  g     = V(yr) + E1(yr) cos theta + E2(yr) cos(2 theta + lean (yr - 470) / 475) + E3(yr) cos 3 theta
          + D(yr) sq(x) + KC(yr) saw65(y) - pupil(theta, yr)              (palette cycles)
          yr = content row (0 .. 1150, rolls), y = screen row; sq = the 574-px square with 30-px slews, rising at
          x = 265; saw65 = frac((y - 53) / 65.2) - 1/2 (screen-locked cuts)
  key   = frac(g(x - 28 px) + noise) > 1 - KW on the rows flagged KEY  (a window key, its path delayed)
  d     = -g (signed);  fill = d > DPK ? CINV + 2 DPK - d : CINV + d   (orange edge, yellow body, orange core)
  f     = pupil ? PGREY : key ? fill : g ;   colour = PAL[f mod 1]
  pupil = a flat grey disc (~40 x 25 px) at every eye-oscillator peak on content row 300

The envelopes V, E1, E2, E3, D, KC were fitted per row (wrapped IRLS on the palette position of every pixel, then a
descent on the RGB error) on a 20-line grid; KNOTS below holds them.

Package contents (all tables also read by imodel.py):
  C_HN_COS  256 x 12   quarter-wave cosine magnitude, round(2047 cos(2 pi (i + 0.5) / 1024))
  C_HN_TA   256 x 32   {V - VB (22 bits signed, Q16 cycles), KC (8 bits, Q9 cycles), KEY, 0} at content row i * 1150 / 256
  C_HN_TB   256 x 32   {E1, E2, E3 (10 bits signed each, Q10 cycles), 00}
  C_HN_TC   256 x 16   {D (10 bits signed, Q10 cycles), 000000}
  C_HN_PAL  512 x 32   {Y, U, V, 00}: Fire (0..255, the still's 48-colour cyclic palette) and Wasp (256..511, the
                       same with R = G -> yellow / black / white / cyan, clip 23_33_13)
  C_HN_PUY  16 x 5     pupil profile by |content row - 300|: round(18 (1 - (d / 12.5)^2))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the 13/16 output
scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ the still's palette (SOM order) and positions
SOM = [(67, 30, 2), (87, 12, 1), (111, 6, 1), (128, 20, 2), (131, 58, 6), (163, 72, 18), (175, 38, 7), (167, 9, 1),
       (186, 5, 1), (210, 5, 1), (232, 3, 1), (245, 3, 0), (249, 16, 0), (243, 43, 2), (244, 69, 2), (248, 92, 1),
       (249, 115, 2), (251, 137, 6), (254, 159, 4), (254, 181, 2), (253, 199, 9), (250, 198, 44), (246, 200, 88),
       (241, 211, 129), (221, 197, 163), (233, 222, 202), (234, 238, 228), (206, 238, 234), (170, 235, 235),
       (142, 238, 238), (116, 238, 239), (89, 239, 242), (61, 238, 244), (24, 237, 246), (5, 226, 248),
       (23, 200, 241), (71, 173, 220), (116, 157, 184), (137, 126, 133), (90, 108, 118), (42, 99, 69), (28, 93, 8),
       (12, 63, 1), (4, 36, 1), (1, 11, 0), (1, 2, 0), (12, 3, 1), (45, 15, 1)]
CEN = [0.0121, 0.0364, 0.0607, 0.0850, 0.1093, 0.1336, 0.1579, 0.1850, 0.2150, 0.2450, 0.2750, 0.3050, 0.3242, 0.3325, 0.3408, 0.3492, 0.3575, 0.3658, 0.3760, 0.3880, 0.4000, 0.4120, 0.4240, 0.4400, 0.4600, 0.4800, 0.5000, 0.5200, 0.5388, 0.5563, 0.5738, 0.5913, 0.6150, 0.6450, 0.6750, 0.7050, 0.7263, 0.7388, 0.7513, 0.7638, 0.7783, 0.7950, 0.8117, 0.8300, 0.8500, 0.8833, 0.9300, 0.9767]

# ------------------------------------------------------------------ fitted model constants
PC = 1150.0          # content period (lines)
YB = 470.0           # band reference row (V is stored relative to V(YB))
CINV = 0.33          # key fill at the key edge (orange)
DPK = 0.095          # key fill fold: yellow peak at this depth, back to orange at the lemon core
DKEY = 28            # key path delay (virtual px)
KW = 0.30            # key window width
PUPR = 18            # pupil radius in 2047 (1 - cos theta) units (~20 px)
PUPY = 12.5          # pupil half-height (lines)
PGREY_RGB = (158, 141, 142)   # the still's pupil grey
KCSH = 0.31          # Shred: cut depth on every row (cycles)
P12DEF = 160         # the still's lean on P12 (lean = 1.6 px per line at 1023)
LEANK = 55           # lean per P12 step, Q24 cycles of 2 theta per line (1.6 / 475 * 2^24 / 1023)
EGK = 86             # etch gain constant (2.5 cycles at full luma at K6 = 1023)
SAW0 = 12263         # frac((0 - 53) / 65.2) * 65536

#@@KNOTS-BEGIN (written by .heirloom_work/rb_hornet/mkhook.py from the fit)
KNOTS = {
    'V': [(0, 8.4800), (20, 8.4711), (40, 8.4825), (60, 7.5479), (80, 6.5256), (100, 5.4951), (120, 4.1968), (140, 3.4400), (160, 2.5368), (180, 2.1957), (200, 1.6464), (220, 1.3157), (240, 0.9466), (260, 0.5429), (280, 0.3904), (300, 0.3681), (320, 0.3607), (340, 0.3428), (360, 0.2309), (380, 0.2686), (400, 0.1865), (420, 0.1494), (440, 0.1507), (460, 0.1396), (480, 0.0900), (500, 0.1272), (520, 0.1079), (540, 0.0674), (560, 0.1869), (580, 0.2758), (600, 0.2814), (620, 0.3751), (640, 0.3477), (660, 0.3676), (680, 0.1207), (700, 0.0700), (720, 0.1706), (740, -0.2314), (760, -0.6693), (780, -1.1401), (800, -1.2726), (820, -1.1106), (840, -1.0242), (860, -1.1194), (880, -1.2600), (900, -1.4077), (920, -1.5248), (940, -0.8682), (960, -0.8400), (980, -1.0729), (1000, -1.0471), (1020, -1.2551), (1040, -1.8400), (1060, -2.1279), (1080, -2.0339), (1100, -1.9900), (1120, -2.0100), (1150, 8.9600)],
    'E1': [(0, 0.0394), (20, 0.0096), (40, -0.0054), (60, 0.0032), (80, -0.0003), (100, -0.0052), (120, 0.0096), (140, -0.0016), (160, 0.0352), (180, -0.1270), (200, -0.0047), (220, -0.3640), (240, 0.0401), (260, -0.1580), (280, -0.1400), (300, -0.1247), (320, -0.1077), (340, -0.0841), (360, 0.0249), (380, 0.0789), (400, 0.1170), (420, 0.1121), (440, 0.0670), (460, 0.0593), (480, 0.0053), (500, 0.0174), (520, -0.0396), (540, -0.0674), (560, -0.0307), (580, -0.0583), (600, -0.0798), (620, 0.1178), (640, 0.0910), (660, 0.2064), (680, 0.0609), (700, 0.1044), (720, 0.2647), (740, 0.1632), (760, -0.0257), (780, 0.0058), (800, 0.0185), (820, 0.0417), (840, 0.0196), (860, 0.0349), (880, -0.0052), (900, 0.0052), (920, -0.0182), (940, 0.0109), (960, 0.0154), (980, -0.0096), (1000, 0.0394), (1020, -0.0044), (1040, 0.0109), (1060, -0.0061), (1080, 0.0361), (1100, 0.0000), (1120, 0.0000), (1150, 0.0000)],
    'E2': [(0, -0.0522), (20, -0.0276), (40, 0.0224), (60, 0.0176), (80, -0.0029), (100, 0.0076), (120, 0.0002), (140, 0.0000), (160, 0.0019), (180, -0.0113), (200, -0.0499), (220, 0.0417), (240, -0.0506), (260, -0.0783), (280, -0.0399), (300, -0.0425), (320, -0.0729), (340, -0.0614), (360, 0.0320), (380, 0.1008), (400, 0.1960), (420, 0.2375), (440, 0.2701), (460, 0.2628), (480, 0.2681), (500, 0.2725), (520, 0.2502), (540, 0.2439), (560, 0.2703), (580, 0.0873), (600, 0.1235), (620, -0.0840), (640, -0.0898), (660, -0.1107), (680, -0.0390), (700, 0.0570), (720, -0.0928), (740, -0.0111), (760, -0.0102), (780, 0.0128), (800, -0.0152), (820, -0.0178), (840, -0.0214), (860, 0.0179), (880, 0.0052), (900, -0.0037), (920, 0.0496), (940, -0.0170), (960, 0.0074), (980, -0.0067), (1000, 0.0534), (1020, -0.0044), (1040, -0.0069), (1060, 0.0064), (1080, 0.0201), (1100, 0.0000), (1120, 0.0000), (1150, 0.0000)],
    'E3': [(0, -0.0012), (20, -0.0002), (40, -0.0001), (60, 0.0000), (80, 0.0019), (100, -0.0015), (120, -0.0022), (140, 0.0006), (160, -0.0090), (180, 0.0312), (200, -0.0586), (220, 0.1488), (240, -0.0723), (260, -0.0086), (280, 0.0301), (300, 0.0595), (320, -0.0588), (340, -0.0892), (360, -0.1764), (380, -0.1059), (400, -0.1304), (420, -0.0999), (440, -0.0589), (460, -0.0554), (480, -0.0189), (500, -0.0455), (520, -0.0365), (540, -0.0245), (560, 0.0344), (580, 0.0223), (600, 0.1781), (620, 0.0250), (640, 0.0459), (660, -0.0363), (680, -0.0517), (700, 0.0275), (720, 0.1034), (740, -0.0640), (760, 0.0148), (780, -0.0156), (800, 0.0127), (820, -0.0271), (840, 0.0138), (860, 0.0092), (880, 0.0006), (900, -0.0019), (920, 0.0016), (940, -0.0188), (960, -0.0195), (980, 0.0004), (1000, -0.0096), (1020, 0.0052), (1040, -0.0026), (1060, 0.0080), (1080, 0.0080), (1100, 0.0000), (1120, 0.0000), (1150, 0.0000)],
    'D': [(0, -0.5144), (20, -0.4321), (40, -0.3429), (60, -0.2046), (80, -0.1507), (100, -0.0714), (120, -0.1344), (140, 0.0648), (160, -0.0060), (180, -0.0008), (200, -0.0074), (220, -0.0176), (240, -0.0077), (260, 0.0028), (280, -0.0096), (300, -0.0035), (320, 0.0003), (340, -0.0021), (360, -0.0019), (380, -0.0190), (400, 0.0235), (420, 0.0003), (440, -0.0093), (460, 0.0000), (480, -0.0125), (500, -0.0011), (520, -0.0039), (540, -0.0035), (560, 0.0000), (580, 0.0221), (600, 0.0102), (620, -0.0136), (640, 0.0009), (660, -0.0092), (680, 0.0015), (700, 0.0026), (720, -0.0250), (740, -0.0233), (760, 0.0065), (780, -0.0022), (800, -0.0837), (820, 0.0596), (840, 0.2358), (860, 0.2836), (880, 0.2836), (900, 0.3097), (920, 0.7244), (940, 0.2412), (960, 0.2930), (980, -0.0626), (1000, 0.0778), (1020, -0.1328), (1040, 0.0129), (1060, 0.0058), (1080, 0.0133), (1100, -0.3333), (1120, -0.4000), (1150, -0.5000)],
    'KC': [(0, 0.2960), (20, 0.0976), (40, -0.2576), (60, -0.2086), (80, -0.3680), (100, 0.3185), (120, 0.4900), (140, 0.3080), (160, 0.6100), (180, 0.7140), (200, 0.5300), (220, 0.2660), (240, 0.5460), (260, 0.4191), (280, 0.1029), (300, -0.0006), (320, 0.0240), (340, 0.0272), (360, 0.1856), (380, -0.0428), (400, 0.0394), (420, -0.0640), (440, -0.0384), (460, 0.1519), (480, 0.0134), (500, -0.0818), (520, 0.0912), (540, -0.0432), (560, -0.1302), (580, -0.0073), (600, 0.1909), (620, -0.0096), (640, -0.2482), (660, 0.0141), (680, 0.0827), (700, 0.2505), (720, 0.2632), (740, -0.0825), (760, 0.7718), (780, 0.7300), (800, -0.1900), (820, -0.0188), (840, 0.1256), (860, 0.3256), (880, 0.3041), (900, 0.3715), (920, 0.1380), (940, 0.4260), (960, 0.2084), (980, 0.2404), (1000, -0.2300), (1020, 0.4842), (1040, 0.8500), (1060, 0.0400), (1080, -0.2272), (1100, 0.0000), (1120, 0.0000), (1150, 0.0000)],
    'KEY': [(0, 0.0000), (330, 0.0000), (340, 1.0000), (600, 1.0000), (610, 0.0000), (1150, 0.0000)],
}
#@@KNOTS-END

def interp(kn, y):
    y = y % PC
    for i in range(len(kn) - 1):
        y0, v0 = kn[i]; y1, v1 = kn[i + 1]
        if y0 <= y <= y1:
            return v0 if y1 == y0 else v0 + (v1 - v0) * (y - y0) / (y1 - y0)
    return kn[-1][1]

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

def fire_rgb(n=256):
    out = []
    P = [c - 1 for c in CEN] + CEN + [c + 1 for c in CEN]
    C = SOM * 3
    for i in range(n):
        t = (i + 0.5) / n
        j = max(k for k in range(len(P)) if P[k] <= t)
        w = (t - P[j]) / (P[j + 1] - P[j])
        out.append(tuple(C[j][k] * (1 - w) + C[j + 1][k] * w for k in range(3)))
    return out

def wasp_rgb(fire):
    return [(r, max(r, g), b) for (r, g, b) in fire]

def pal_rom():
    fire = fire_rgb(); wasp = wasp_rgb(fire)
    return [word(rgb_to_hw10(c)) for c in fire] + [word(rgb_to_hw10(c)) for c in wasp]

def black_index():
    fire = fire_rgb()
    return min(range(256), key=lambda i: sum(fire[i]))

def cos_rom():
    return [int(round(2047 * math.cos(2 * math.pi * (i + 0.5) / 1024))) for i in range(256)]

def sat(v, lo, hi):
    return max(lo, min(hi, v))

def tables():
    vb = interp(KNOTS['V'], YB)
    ta, tb, tc = [], [], []
    for i in range(256):
        r = i * PC / 256.0
        v = sat(int(round((interp(KNOTS['V'], r) - vb) * 65536)), -(1 << 21), (1 << 21) - 1)
        kc = sat(int(round(interp(KNOTS['KC'], r) * 512)), 0, 255)
        key = 1 if interp(KNOTS['KEY'], r) > 0.5 else 0
        e1 = sat(int(round(interp(KNOTS['E1'], r) * 1024)), -511, 511)
        e2 = sat(int(round(interp(KNOTS['E2'], r) * 1024)), -511, 511)
        e3 = sat(int(round(interp(KNOTS['E3'], r) * 1024)), -511, 511)
        d = sat(int(round(interp(KNOTS['D'], r) * 1024)), -511, 511)
        ta.append(((v & 0x3FFFFF) << 10) | (kc << 2) | (key << 1))
        tb.append(((e1 & 0x3FF) << 22) | ((e2 & 0x3FF) << 12) | ((e3 & 0x3FF) << 2))
        tc.append((d & 0x3FF) << 6)
    return ta, tb, tc, vb

def puy_rom():
    return [max(0, int(round(PUPR * (1 - (d / PUPY) ** 2)))) for d in range(16)]

def grey_index():
    fire = fire_rgb()
    return min(range(256), key=lambda i: sum((fire[i][k] - PGREY_RGB[k]) ** 2 for k in range(3)))

def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        items.append('x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width))
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    ta, tb, tc, vb = tables()
    consts = [
        "    constant C_VB     : integer := %d;   -- V at the band row (Q16 cycles, mod 1)" % (int(round(vb * 65536)) & 0xFFFF),
        "    constant C_CINV   : integer := %d;" % int(round(CINV * 65536)),
        "    constant C_KTHR   : integer := %d;" % int(round((1 - KW) * 65536)),
        "    constant C_DPK    : integer := %d;   -- key fill fold depth (Q16)" % int(round(DPK * 65536)),
        "    constant C_CINV2  : integer := %d;   -- CINV + 2 DPK" % int(round((CINV + 2 * DPK) * 65536)),
        "    constant C_PGREY  : integer := %d;   -- the pupil grey's palette entry" % grey_index(),
        "    constant C_PUY    : integer := 300;",
        "    constant C_KCSH   : integer := %d;" % int(round(KCSH * 512)),
        "    constant C_P12DEF : integer := %d;" % P12DEF,
        "    constant C_LEANK  : integer := %d;" % LEANK,
        "    constant C_EGK    : integer := %d;" % EGK,
        "    constant C_SAW0   : integer := %d;" % SAW0,
        "    constant C_ROLL0  : integer := 0;",
        "    constant C_PBLK   : integer := %d;   -- the palette's black entry" % black_index(),
    ]
    parts = ["-- Generated by hornet.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package hornet_pkg is"] + consts + [
             vhd_table("C_HN_COS", "unsigned", 256, 12, cos_rom()),
             vhd_table("C_HN_TA", "std_logic_vector", 256, 32, ta),
             vhd_table("C_HN_TB", "std_logic_vector", 256, 32, tb),
             vhd_table("C_HN_TC", "std_logic_vector", 256, 16, tc),
             vhd_table("C_HN_PAL", "std_logic_vector", 512, 32, pal_rom()),
             vhd_table("C_HN_PUY", "unsigned", 16, 5, puy_rom()),
             "end package hornet_pkg;", ""]
    with open(os.path.join(HERE, "hornet_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
