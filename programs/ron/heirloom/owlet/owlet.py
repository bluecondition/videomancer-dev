#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
OWLET v0.1.0 - build hook.  Writes owlet_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; imodel.py in .heirloom_work/rb_owlet imports the same tables).

Fitted to the still WIN_20230818_01_17_40 @ 3.0 s (see .heirloom_work/rb_owlet/notes.md).  The field (virtual
1080-line coordinates):
    F = V(Y, d) + g(Y, d) S(hn) + a(d) Z(hn) + etch * luma,   d = Y - 610 + s (the rolling eye oscillator)
    V = Vs(d) + P(Y),  g = gs(d) Gf(Y)  (Vs, gs: the eye profile, symmetric, scrolling; P, Gf: the still's
    screen-fixed residuals, so s = 0 is the still exactly)
    hn = distance to the nearest eye axis / half the eye period (0 .. 1);  S = 470 hn^1.27;  Z = the fin sawtooth
    colour = palette(F); the red band: a red monochrome of the palette brightness

Package contents:
  C_OW_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_OW_SC    256 x 32   scroll table, u = 4 i lines past the eye (mod 1024, u >= 512 = above it):
                        {Vs * 4 (13b signed), gs * 1024 (11b), wn (8b: 0 at the eye .. 255 at the maximum)}
  C_OW_SF    256 x 32   screen table, Y = 1024 j / 243: {P * 4 (14b signed), Gf * 1024 (12b), 000000}
  C_OW_S     1024 x 10  2 * 470 hn^p, hn = (k + 0.5) / 512; k < 512: p = 1.27 (the still), else p = 0.62 (Diamond)
  C_OW_Z     512 x 8    round(256 saw((470 hn - 36) / 134) hn) (signed): the fins
  C_OW_PAL   1024 x 32  {Y8, U8, V8, R8} for F = -640 + 2 i + 1: palette colour /4 (luma pre-scaled 16/13, capped
                        at 940) and the red level of the red band
  C_OW_RED   256 x 32   {00, Y10, U10, V10}: the red level r -> Y = 64 + 1.453 r, Cr = 512 + 1.641 r, Cb = 512 - 0.625 r
Colours in the hardware convention (U register = Cr, V register = Cb).
"""
import math, os

PX, PXD = 1.27, 0.62
RL, RPH, RRISE = 134.0, 36.0, 0.7
KN = [[0, 700, 0.40], [98, 645, 0.42], [130, 631, 0.43], [166, 577, 0.44], [206, 531, 0.45], [246, 441, 0.46],
      [286, 381, 0.48], [326, 352, 0.50], [366, 300, 0.53], [406, 240, 0.56], [446, 213, 0.62], [486, 93, 0.81],
      [526, -4, 1.03], [566, -47, 1.21], [606, -67, 1.34], [646, -35, 1.40], [686, -13, 1.45], [726, 44, 1.47],
      [766, 105, 1.40], [806, 176, 1.31], [846, 241, 1.26], [886, 288, 1.24], [926, 353, 1.25], [966, 380, 1.23],
      [1006, 525, 1.16], [1046, 562, 1.13], [1080, 610, 1.10]]
KEYS = [
    (-560, (10, 14, 40)),      # trough of the fold below the pupil
    (-540, (8, 36, 131)),
    (-520, (6, 55, 239)),
    (-500, (3, 107, 252)),
    (-480, (11, 200, 250)),
    (-462, (166, 161, 246)),
    (-445, (230, 172, 242)),   # P-1
    (-428, (166, 161, 246)),
    (-410, (11, 200, 250)),
    (-392, (3, 107, 252)),
    (-372, (8, 36, 131)),
    (-358, (14, 12, 40)),      # trough
    (-342, (8, 36, 131)),
    (-322, (6, 70, 245)),
    (-300, (7, 161, 253)),
    (-280, (89, 182, 248)),
    (-262, (230, 172, 242)),   # P0 (pink core of a deep pupil: the clip at +1.5 s)
    (-238, (230, 172, 242)),
    (-218, (89, 182, 248)),
    (-196, (7, 161, 253)),
    (-172, (3, 107, 252)),
    (-150, (6, 55, 239)),
    (-118, (52, 10, 250)),
    (-71, (74, 5, 227)),       # pupil violet (deepest in the still)
    (-10, (60, 8, 248)),
    (30, (45, 16, 252)),
    (55, (20, 42, 252)),
    (80, (6, 70, 252)),
    (100, (3, 107, 252)),
    (120, (7, 150, 253)),
    (140, (60, 165, 250)),
    (158, (166, 161, 246)),
    (172, (230, 172, 242)),    # P1 (wide pink)
    (225, (234, 176, 240)),
    (240, (166, 166, 245)),
    (254, (40, 175, 252)),
    (268, (11, 200, 250)),
    (284, (7, 161, 253)),
    (298, (3, 107, 252)),
    (309, (6, 55, 239)),
    (316, (8, 36, 131)),
    (322, (14, 14, 22)),       # K: the black outline
    (338, (14, 14, 22)),
    (344, (8, 36, 131)),
    (351, (6, 60, 240)),
    (360, (3, 107, 252)),
    (370, (11, 200, 250)),
    (378, (166, 170, 245)),
    (385, (232, 190, 240)),    # P2
    (408, (220, 196, 240)),
    (416, (166, 170, 245)),
    (424, (11, 200, 250)),
    (434, (7, 161, 253)),
    (446, (3, 107, 252)),
    (456, (52, 10, 250)),
    (467, (112, 4, 254)),      # T2 shoulder: violet
    (480, (17, 12, 147)),
    (490, (12, 10, 36)),       # T2 core (the saddle X)
    (500, (17, 12, 147)),
    (512, (52, 10, 250)),
    (524, (6, 60, 245)),
    (540, (3, 107, 252)),
    (556, (7, 161, 253)),
    (568, (11, 217, 249)),
    (578, (169, 221, 236)),
    (588, (214, 222, 234)),    # P3 (white-pink)
    (612, (225, 200, 238)),
    (622, (169, 221, 236)),
    (632, (11, 217, 249)),
    (646, (7, 161, 253)),
    (660, (3, 107, 252)),
    (672, (6, 55, 239)),
    (680, (8, 36, 131)),
    (686, (14, 12, 20)),       # T3 core (near black; where the red key cuts in at the top)
    (698, (14, 12, 20)),
    (704, (8, 36, 131)),
    (714, (6, 60, 240)),
    (728, (3, 107, 252)),
    (744, (7, 161, 253)),
    (758, (11, 217, 249)),
    (768, (166, 161, 246)),
    (776, (230, 172, 242)),    # P4
    (800, (230, 172, 242)),
    (810, (166, 161, 246)),
    (820, (11, 210, 250)),
    (834, (7, 161, 253)),
    (848, (3, 107, 252)),
    (862, (6, 55, 239)),
    (872, (6, 70, 235)),
    (881, (10, 60, 200)),      # T4 (shallow: the bright bottom)
    (890, (6, 80, 240)),
    (900, (6, 60, 240)),
    (915, (3, 107, 252)),
    (932, (7, 161, 253)),
    (948, (11, 217, 249)),
    (960, (169, 221, 236)),
    (970, (214, 222, 234)),    # P5
    (995, (214, 222, 234)),
    (1005, (169, 221, 236)),
    (1015, (11, 217, 249)),
    (1030, (7, 161, 253)),
    (1045, (3, 107, 252)),
    (1058, (6, 55, 239)),
    (1067, (6, 70, 230)),
    (1076, (10, 50, 180)),     # T5 (period 195 from here on)
]


def interp(x, xs, ys):
    if x <= xs[0]: return ys[0]
    if x >= xs[-1]: return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        m = (lo + hi) // 2
        if xs[m] <= x: lo = m
        else: hi = m
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + t * (ys[hi] - ys[lo])

def smooth(a, sig, circ):
    n = int(3 * sig)
    k = [math.exp(-0.5 * (i / sig) ** 2) for i in range(-n, n + 1)]
    s = sum(k); k = [v / s for v in k]
    N = len(a); out = []
    for i in range(N):
        acc = 0.0
        for j in range(-n, n + 1):
            p = i + j
            if circ: p %= N
            else: p = min(max(p, 0), N - 1)
            acc += k[j + n] * a[p]
        out.append(acc)
    return out

def tables():
    """screen tables V0, G0 (Y 0..1079); scroll tables Vs, gs (index d + 512, d = -512..511); P, Gf; wn."""
    xs = [r[0] for r in KN]
    V0 = smooth([interp(y, xs, [r[1] for r in KN]) for y in range(1080)], 16.0, False)
    G0 = smooth([interp(y, xs, [r[2] for r in KN]) for y in range(1080)], 16.0, False)
    Vs, gs = [], []
    for d in range(-512, 512):
        a = abs(d)
        up = interp(610 - a, list(range(1080)), V0); gu = interp(610 - a, list(range(1080)), G0)
        if 610 + a <= 1079:
            Vs.append((up + V0[610 + a]) / 2); gs.append((gu + G0[610 + a]) / 2)
        else:
            Vs.append(up); gs.append(gu)
    Vs = smooth(Vs, 10.0, True); gs = smooth(gs, 10.0, True)
    P = [V0[y] - Vs[min(max(y - 610, -512), 511) + 512] for y in range(1080)]
    Gf = [G0[y] / gs[min(max(y - 610, -512), 511) + 512] for y in range(1080)]
    vmin, vmax = Vs[512], max(Vs)
    wn = [min(max((v - vmin) / (vmax - vmin), 0.0), 1.0) for v in Vs]
    # the fins live above the eye only (d < 0), faded in over the first 48 lines past the oscillator's maximum
    wn = [w * (min(1.0, (d + 512) / 48.0) if d < 0 else 0.0) for w, d in zip(wn, range(-512, 512))]
    return V0, G0, Vs, gs, P, Gf, wn

def pal_rgb(F):
    xs = [k[0] for k in KEYS]; lo, hi = xs[0], xs[-1]; per = 195.0
    if F >= hi: F = hi - per + math.fmod(F - hi, per)
    if F < lo: F = lo + (F - lo) % per
    return tuple(interp(F, xs, [k[1][c] for k in KEYS]) for c in range(3))

def red_level(F):
    r, g, b = pal_rgb(F)
    y = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    return min(max(20 + 280 * y ** 1.35, 0), 255)

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772; cr = (r - y) / 1.402
    return round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896)

def saw(h):
    ph = ((h - RPH) / RL) % 1.0
    return (ph / RRISE if ph < RRISE else (1.0 - ph) / (1 - RRISE)) - 0.5

def build():
    V0, G0, Vs, gs, P, Gf, wn = tables()
    exp2 = [round(32768 * 2 ** (i / 256.0)) for i in range(256)]
    sc = []
    for i in range(256):
        u = 4 * i
        d = u if u < 512 else u - 1024
        v = round(4 * Vs[d + 512]) & 0x1FFF
        g = min(round(1024 * gs[d + 512]), 2047)
        w = round(255 * wn[d + 512])
        sc.append((v << 19) | (g << 8) | w)
    sf = []
    for j in range(256):
        y = 1024.0 * j / 243.0
        yi = min(int(y), 1078); t = y - yi
        p = P[yi] + t * (P[yi + 1] - P[yi]) if y < 1079 else P[1079]
        gf = Gf[yi] + t * (Gf[yi + 1] - Gf[yi]) if y < 1079 else Gf[1079]
        sf.append(((round(4 * p) & 0x3FFF) << 18) | (min(round(1024 * gf), 4095) << 6))
    srom = []
    for k in range(1024):
        p = PX if k < 512 else PXD
        hn = ((k & 511) + 0.5) / 512.0
        srom.append(min(round(940 * hn ** p), 1023))
    zrom = []
    for k in range(512):
        hn = (k + 0.5) / 512.0
        zrom.append(max(-128, min(127, round(256 * saw(470 * hn) * hn))) & 0xFF)
    pal = []
    for i in range(1024):
        F = -640 + 2 * i + 1
        y, u, v = rgb_to_hw10(pal_rgb(F))
        y = min(64 + (y - 64) * 16.0 / 13.0, 940)
        y8, u8, v8 = round(y / 4), min(round(u / 4), 255), min(round(v / 4), 255)
        pal.append((y8 << 24) | (u8 << 16) | (v8 << 8) | round(red_level(F)))
    red = []
    for r in range(256):
        ry = 64 + r + (r >> 1) - (r >> 5) - (r >> 6)
        ru = min(1023, 512 + r + (r >> 1) + (r >> 3) + (r >> 6))
        rv = 512 - (r >> 1) - (r >> 3)
        red.append((ry << 20) | (ru << 10) | rv)
    return dict(EXP2=exp2, SC=sc, SF=sf, S=srom, Z=zrom, PAL=pal, RED=red)

def vhd_array(name, typ, vals, width):
    digits = (width + 3) // 4
    lines = []
    for i in range(0, len(vals), 8):
        lines.append('        ' + ', '.join('x"%0*X"' % (digits, v) if width % 4 == 0 else '"%s"' % format(v, '0%db' % width) for v in vals[i:i + 8]))
    return '    constant %s : %s := (\n%s\n    );\n' % (name, typ, ',\n'.join(lines))

def write_pkg(path):
    t = build()
    s = ['-- Generated by owlet.py -- do not edit.  Copyright (C) 2026 czartroniks, GPL-3.0-only\n',
         'library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\npackage owlet_pkg is\n',
         '    type t_ow_u16x256 is array (0 to 255) of unsigned(15 downto 0);\n',
         '    type t_ow_w32x256 is array (0 to 255) of std_logic_vector(31 downto 0);\n',
         '    type t_ow_u10x1024 is array (0 to 1023) of unsigned(9 downto 0);\n',
         '    type t_ow_w8x512 is array (0 to 511) of std_logic_vector(7 downto 0);\n',
         '    type t_ow_w32x1024 is array (0 to 1023) of std_logic_vector(31 downto 0);\n']
    s.append(vhd_array('C_OW_EXP2', 't_ow_u16x256', t['EXP2'], 16))
    s.append(vhd_array('C_OW_SC', 't_ow_w32x256', t['SC'], 32))
    s.append(vhd_array('C_OW_SF', 't_ow_w32x256', t['SF'], 32))
    s.append(vhd_array('C_OW_S', 't_ow_u10x1024', t['S'], 10).replace('"', '"'))
    s.append(vhd_array('C_OW_Z', 't_ow_w8x512', t['Z'], 8))
    s.append(vhd_array('C_OW_PAL', 't_ow_w32x1024', t['PAL'], 32))
    s.append(vhd_array('C_OW_RED', 't_ow_w32x256', t['RED'], 32))
    s.append('end package owlet_pkg;\n')
    open(path, 'w').write(''.join(s))

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    write_pkg(os.path.join(here, 'owlet_pkg.vhd'))
