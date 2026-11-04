#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ZEBRA v0.1.0 - build hook.  Writes zebra_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  imodel.py (.heirloom_work/rb_zebra) imports the same tables and constants.

Fitted to the clip still WIN_20230404_23_59_18 t9.5 s (see .heirloom_work/rb_zebra/notes.md):

  seams  S0 = a1 u + a2 v + a4 u v + a5 v^2 + c3 u v^2 - phi      u = (x - 960)/1000, v = (y - 540)/1000 (virtual px)
         (an H ramp whose gain a V parabola bends, plus a V parabola: 5 terms, phase coherence 0.869 on the rims)
         S  = 3.5 + sc (S0 - 3.5)       sc = the Swirl (P12) seam density, 1 = the photo; pivot just outside the outer seam
         the outer ring is S >= 3 (no drift / etch); elsewhere ribbon n = floor(S + drift + etch), sigma = frac(...)
  base   W  = f ky (|y - 520| m(x) + crease) + delta n + w0 + hook(sigma) + etch luma + flow
         crease = k |x - 1000| clipped at x = 332 / 1645 (right slope 0.919 k); m(x) ramps 1 -> 1.8 over 220 px beyond
         the clips; the outer ring has W = 1.8 f ky |y - 520| + 0.5 (dense horizontal stripes, no crease)
  colour F  = G tri(W) - T ; black for F < 0 ; else palette(fold(F)) whose dark end is black (the thin zebra lines)
  rims   at every seam: a dark line, a brown line, a lavender glow (Curl)

Package contents:
  C_ZB_PAL   512 x 32  {Y, U, V, 00}: index = pal * 256 + fold position (0 = dark .. 255 = blue); entries < 26 black
  C_ZB_HOOK  256 x 16  round(4096 * 1.2 * exp(-(i / 1024) / 0.06)): the stripes' curl into the seam, Q12 cycles
  C_ZB_EXP2  256 x 16  round(32768 * 2^(i / 256))
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale and capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted seam field (rb_zebra/fitq3.py)
A1, A2, A4, A5, C3 = -4.094259287095266, -5.074398883137135, -4.514454797430695, -1.4262831389803339, -1.485778395125988
PHI = 0.17734724688893197
PIVOT = 3.5
SADDLE_K = -16.0      # S9 Seams = Saddle: S0 = -16 u v, hyperbolic seams, two calm outer-ring corners
# base field (rb_zebra/fito2.py: stripe orientation; fitrib*.py / freq traces: densities)
Y0, XC, XL, XR = 520.0, 1000.0, 332.0, 1645.0
KY, F_BASE = 1.0 / 473.0, 0.55
KR_RATIO = 2.17 / 2.36
M_FLAT, WT = 1.8, 220.0
W0, O_TOP = 0.9, 0.5
HOOK_AMP, HOOK_TAU = 1.2, 0.06
G_PHOTO = 3.55

# ------------------------------------------------------------------ fixed-point constants used by the RTL
def r(x):
    return int(math.floor(x + 0.5))

# per-frame seam coefficients, multiplied by sc (Q12) and shifted right 12 in the sequencer
SEAM = dict(
    A1c=r(A1 * 2**20), A4c=r(A4 * 2**35 / 32000), C3c=r(C3 * 2**50 / 32000**2),
    A2c=r(A2 * 2**35 / 32000), A5c=r(A5 * 2**50 / 32000**2), P35c=r((PIVOT + PHI) * 2**20))
ZSEAM = dict(A1c=0, A4c=r(SADDLE_K * 2**35 / 32000), C3c=0, A2c=0, A5c=0, P35c=r(PIVOT * 2**20))
K35 = r(PIVOT * 2**20)
THREE22 = 3 << 22

# timing classes: (x step in half-px, beam x of the first pixel in half-px)  0: 1080 (xs 2), 1: 720p (xs 3), 2: SD (xs 4)
XCLASS = [(2, 16), (3, 24), (4, 512)]       # x0 includes the 8-pixel lead that cancels the output boxes
def xclass_consts(xs, x0h):
    u0 = (x0h - 1920) / 2000.0
    du = xs / 2000.0
    # side-zone ramp DDA (per line: Rmax = (AY5 * CRM) >> 18, step RS = (AY5 * CRS) >> 18, Q20)
    crs = F_BASE * KY * (M_FLAT - 1) / 32 * 2**38 * xs / (2 * WT)
    r0steps = max(0, min(int(2 * WT) // xs, (r(2 * XL) - x0h) // xs))   # steps from x0 to the left clip (SD: 38)
    return dict(U0=r(abs(u0) * 2**14), DU=r(du * 2**23), CRS=r(crs), R0S=r0steps, R0FULL=int(x0h <= r(2 * XL) - r(2 * WT)))
XC_CONSTS = [xclass_consts(*c) for c in XCLASS]

# per-line constants (Q20 W cycles from |y - 520| in 1/32 lines)
CWY = r(F_BASE * KY / 32 * 2**30)                 # Wy  = (AY5 * CWY) >> 10
CRM = r(F_BASE * KY * (M_FLAT - 1) / 32 * 2**38)  # Rmax = (AY5 * CRM) >> 18 (Q20): the ramp's full height
CW0 = r(F_BASE * KY * M_FLAT / 32 * 2**29)        # Wr0 = (AY5 * CW0) >> 9
# crease: kk (Q17 W per half-px) = (k2 - 512) * KKc >> 13, k = 6 (k2 - 512) / 511, 2.36 at 713;
# H is a DDA: H0 = kk * HLc (Q20, held left of the clip), step -kk * 8 xs (to the crease), +kkR * 8 xs (to the right clip)
KKc = r(F_BASE * KY * 6.0 / 511 / 2 * 2**20 * 1024)
XL_H, XC_H, XR_H = r(2 * XL), r(2 * XC), r(2 * XR)
HLc = (XC_H - XL_H) * 8
RL_H, RR_H = XL_H - r(2 * WT), XR_H + r(2 * WT)
# rim widths: (T*c x sc12) >> 20 = Q14 seam fraction
TDc, TBc, TGc = r(0.01575 * 2**22), r(0.035 * 2**22), r(0.015 * 2**22)
W0_Q20, OTOP_Q20 = r(W0 * 2**20), r(O_TOP * 2**20)
G_PHOTO_Q8 = r(G_PHOTO * 256)

# ------------------------------------------------------------------ palettes (photo RGB)
NEON = [(12, 12, 10), (38, 5, 12), (46, 13, 20), (71, 13, 15), (79, 21, 23), (103, 27, 11), (113, 37, 21), (143, 41, 21),
        (154, 51, 31), (182, 55, 24), (194, 67, 36), (223, 72, 37), (232, 82, 46), (252, 92, 42), (255, 102, 52),
        (255, 110, 56), (255, 117, 63), (255, 125, 58), (255, 127, 60), (241, 143, 72), (242, 144, 73), (205, 163, 81),
        (205, 163, 81), (165, 181, 90), (165, 181, 90), (123, 198, 109), (123, 198, 109), (83, 213, 119), (83, 213, 119),
        (39, 230, 130), (38, 229, 129), (1, 245, 141), (1, 245, 141), (0, 247, 154), (0, 248, 155), (28, 232, 164),
        (29, 234, 165), (46, 225, 169), (47, 226, 170), (83, 209, 176), (86, 213, 180), (128, 192, 185), (129, 193, 186),
        (169, 175, 198), (171, 177, 200), (207, 158, 209), (206, 157, 208), (237, 138, 215), (237, 138, 215),
        (255, 118, 229), (255, 116, 227), (255, 104, 237), (255, 97, 230), (246, 88, 253), (235, 77, 242), (222, 74, 255),
        (217, 70, 254), (188, 60, 255), (180, 51, 255), (156, 44, 255), (147, 35, 255), (120, 28, 255), (109, 18, 255),
        (78, 14, 255), (70, 7, 255), (35, 12, 255), (28, 5, 255), (12, 7, 255), (13, 8, 255), (27, 4, 255)]
# S10 Fire: the red / yellow / pale colouriser of the same patch in clip WIN_20230405_00_01_57
FIRE = [(8, 4, 2), (120, 10, 0), (230, 40, 0), (255, 90, 0), (255, 150, 0), (255, 205, 20), (255, 240, 120),
        (255, 225, 210), (250, 200, 230), (255, 120, 60), (235, 30, 10), (200, 0, 0)]
BLACK_RGB = (5, 7, 5)
BLACKCUT = 26                      # fold positions below 0.10: black (the thin zebra lines at the dark folds)
LAV_RGB, BROWN_RGB, DARK_RGB = (150, 120, 225), (150, 100, 75), (10, 9, 10)

def smooth_ramp(src, n=256):
    # pair-average (the camera's 4:2:2 chroma repeats samples), light 3-tap smoothing, resample to n
    m = len(src)
    sm = []
    for i in range(m):
        acc = [0.0, 0.0, 0.0]; w = 0.0
        for d, k in ((-2, 1), (-1, 2), (0, 3), (1, 2), (2, 1)):
            j = min(m - 1, max(0, i + d))
            for c in range(3):
                acc[c] += k * src[j][c]
            w += k
        sm.append([a / w for a in acc])
    out = []
    for i in range(n):
        t = i / (n - 1) * (m - 1)
        j = int(math.floor(t)); j1 = min(m - 1, j + 1); a = t - j
        out.append([sm[j][c] * (1 - a) + sm[j1][c] * a for c in range(3)])
    return out

def lerp_ramp(src, n=256):
    m = len(src); out = []
    for i in range(n):
        t = i / (n - 1) * (m - 1)
        j = int(math.floor(t)); j1 = min(m - 1, j + 1); a = t - j
        out.append([src[j][c] * (1 - a) + src[j1][c] * a for c in range(3)])
    return out

def rgb_to_hw10(rgb):
    rr, g, b = [v / 255.0 for v in rgb]
    y = 0.299 * rr + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (rr - y) / 1.402
    yy = 64 + y * 876
    yy = 64 + (yy - 64) * 16 / 13          # pre-scale for the output's 13/16
    yy = min(940, max(64, r(yy)))
    u = min(1023, max(0, r(512 + cr * 896)))   # U register = Cr
    v = min(1023, max(0, r(512 + cb * 896)))   # V register = Cb
    return yy, u, v

def pal_table():
    tab = []
    for ramp in (smooth_ramp(NEON), lerp_ramp(FIRE)):
        for i, c in enumerate(ramp):
            tab.append(rgb_to_hw10(BLACK_RGB if i < BLACKCUT else c))
    return tab

def hook_table():
    return [min(65535, r(4096 * HOOK_AMP * math.exp(-(i / 1024.0) / HOOK_TAU))) for i in range(256)]

def exp2_table():
    return [min(65535, r(32768 * 2 ** (i / 256.0))) for i in range(256)]

PAL = pal_table()
HOOK = hook_table()
EXP2 = exp2_table()
COL_BLACK, COL_LAV, COL_BROWN, COL_DARK = (rgb_to_hw10(c) for c in (BLACK_RGB, LAV_RGB, BROWN_RGB, DARK_RGB))

def slv(v, w):
    return '"' + format(v & ((1 << w) - 1), '0%db' % w) + '"'

def word(y, u, v):
    return (y << 22) | (u << 12) | (v << 2)

def write_pkg(path):
    L = []
    L.append('-- Generated by zebra.py - do not edit.')
    L.append('library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n')
    L.append('package zebra_pkg is')
    L.append('    type t_zb_pal is array (0 to 511) of std_logic_vector(31 downto 0);')
    L.append('    type t_zb_u16 is array (0 to 255) of unsigned(15 downto 0);')
    L.append('    constant C_ZB_PAL : t_zb_pal := (')
    L.append(',\n'.join('        x"%08x"' % word(*c) for c in PAL) + ');')
    L.append('    constant C_ZB_HOOK : t_zb_u16 := (')
    L.append(',\n'.join('        to_unsigned(%d, 16)' % v for v in HOOK) + ');')
    L.append('    constant C_ZB_EXP2 : t_zb_u16 := (')
    L.append(',\n'.join('        to_unsigned(%d, 16)' % v for v in EXP2) + ');')
    for nm, c in (('BLACK', COL_BLACK), ('LAV', COL_LAV), ('BROWN', COL_BROWN), ('DARK', COL_DARK)):
        L.append('    constant C_ZB_%s_Y : integer := %d; constant C_ZB_%s_U : integer := %d; constant C_ZB_%s_V : integer := %d;'
                 % (nm, c[0], nm, c[1], nm, c[2]))
    for k, v in SEAM.items():
        L.append('    constant C_ZB_%s : integer := %d;' % (k.upper(), v))
    for k, v in ZSEAM.items():
        L.append('    constant C_ZB_Z%s : integer := %d;' % (k.upper(), v))
    for i, c in enumerate(XC_CONSTS):
        for k, v in c.items():
            L.append('    constant C_ZB_%s_%d : integer := %d;' % (k.upper(), i, v))
    for nm in ('K35', 'CWY', 'CRM', 'CW0', 'KKc', 'XL_H', 'XC_H', 'XR_H', 'HLc', 'RL_H', 'RR_H', 'TDc', 'TBc', 'TGc',
               'W0_Q20', 'OTOP_Q20', 'G_PHOTO_Q8'):
        L.append('    constant C_ZB_%s : integer := %d;' % (nm.upper(), globals()[nm]))
    L.append('end package zebra_pkg;')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')

if __name__ == '__main__':
    write_pkg(os.path.join(HERE, 'zebra_pkg.vhd'))
    print('zebra.py: wrote zebra_pkg.vhd')
