#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TATTER v0.1.1 - build hook.  Writes tatter_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; .heirloom_work/rb_tatter/imodel.py imports the same tables).

Fitted to the still WIN_20230424_23_47_33 at 8.0 s (see .heirloom_work/rb_tatter/notes.md).  The patch:

  u      = Fx(x) + Fy(y)          a sum lattice: Fx an H waveform (two regimes split by the tear's crack), Fy a V cosine
                                  (period 280 lines); 1 field unit = 256 LSB = one tube-palette cycle (1.56 fitted units)
  u >= 0 -> the tube palette      first cycle: dark checker edge -> white plateau -> dark; later cycles start with the
                                  orange ring (the eye rings, the wavy band's ring line)
  u <  0 -> the checker           rows (FM'd by the V oscillator) alternate two clipped-sawtooth H oscillators
                                  (periods 100.5 / 176.5 px), through an RGB colouriser with a zone that rides the V
                                  oscillator (yellow / red with dark-violet zones; Q set: orange / violet, green / blue)

Package contents:
  C_TT_FX    2048 x 10  Fx in LSB (signed): index = regime * 1024 + x / 2 (regime 0 = left of the crack, 1 = right)
  C_TT_PAL   512 x 30   tube palette {Y, Cr, Cb} (hardware convention, u = Cr): index = first * 256 + t
  C_TT_CHK   512 x 30   checker colours {Y, Cr, Cb}: index = set * 256 + z * 32 + s
  C_TT_V     512 x 16   {z (3 bits) at 12..10, Fy (signed 10 bits, +-164)}: index = V phase (512 per period)
  C_TT_EXP2  256 x 16   round(32768 * 2^(i / 256))
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PER = 1.56                 # tube-palette period in fitted field units = 256 LSB
LSB = PER / 256.0


def pchip(xk, yk, x):
    n = len(xk)
    h = [xk[i + 1] - xk[i] for i in range(n - 1)]
    d = [(yk[i + 1] - yk[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for i in range(1, n - 1):
        if d[i - 1] * d[i] > 0:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    m[0] = d[0]
    m[-1] = d[-1]
    x = min(max(x, xk[0]), xk[-1])
    i = 0
    while i < n - 2 and x > xk[i + 1]:
        i += 1
    t = (x - xk[i]) / h[i]
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * yk[i] + (t ** 3 - 2 * t ** 2 + t) * h[i] * m[i]
            + (-2 * t ** 3 + 3 * t ** 2) * yk[i + 1] + (t ** 3 - t ** 2) * h[i] * m[i + 1])


# ------------------------------------------------------------------ H waveform (fitted per column, fx fits)
# right of the crack: the eye column at x 990, the big checker diamond (min ~1480), the wavy band rising to the edge
KR = [(700, -0.9), (760, -0.75), (800, -0.43), (840, -0.08), (870, 0.06), (900, 0.24), (930, 0.51), (960, 0.76),
      (990, 0.9), (1020, 0.79), (1050, 0.59), (1080, 0.38), (1110, 0.19), (1140, 0.05), (1170, -0.05), (1200, -0.18),
      (1230, -0.41), (1260, -0.62), (1290, -0.83), (1320, -0.98), (1400, -1.3), (1480, -1.4), (1555, -1.15),
      (1590, -0.82), (1620, -0.54), (1650, -0.28), (1680, -0.06), (1710, 0.09), (1740, 0.31), (1770, 0.58),
      (1800, 0.79), (1830, 0.98), (1860, 1.30), (1890, 1.52), (1920, 1.66), (1960, 1.75), (1990, 1.76)]
# left of the crack: the left-edge rings, the X column at x 510, then period ~480 (X columns at 990, 1470, ...)
KL = [(0, 0.6), (120, 0.62), (200, 1.3), (240, 1.24), (270, 0.97), (300, 0.78), (330, 0.69), (360, 0.5),
      (390, 0.29), (420, 0.09), (450, -0.03), (480, -0.1), (510, -0.25), (540, -0.1), (570, 0.0), (600, 0.18),
      (630, 0.46), (660, 0.77), (690, 0.95), (750, 1.15), (870, 0.45), (990, -0.25), (1110, 0.45), (1230, 1.15),
      (1350, 0.45), (1470, -0.25), (1590, 0.45), (1710, 1.15), (1830, 0.45), (1950, -0.25), (2048, 0.3)]


def fx_r(x):
    a = pchip([k[0] for k in KR], [k[1] for k in KR], x)
    if x >= 990:
        return a
    b = pchip([k[0] for k in KR], [k[1] for k in KR], 1980 - x)      # mirrored about the eye column
    w = min(1.0, max(0.0, (x - 700) / 200.0))
    return w * a + (1 - w) * b


def fx_l(x):
    a = pchip([k[0] for k in KL], [k[1] for k in KL], x)
    if x > 1990:                                                      # ease into the wrap (x = 0)
        w = (x - 1990) / 58.0
        a = (1 - w) * a + w * 0.6
    return a


def lsb(v):
    return max(-511, min(511, int(round(v / LSB))))


FX_ROM = [lsb(fx_l(2 * i + 1)) for i in range(1024)] + [lsb(fx_r(2 * i + 1)) for i in range(1024)]


# ------------------------------------------------------------------ colour helpers (hardware convention)
def rgb_hw(r, g, b):
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def lerp3(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def plin(knots, t):
    if t <= knots[0][0]:
        return knots[0][1:]
    for i in range(len(knots) - 1):
        if t <= knots[i + 1][0]:
            f = (t - knots[i][0]) / (knots[i + 1][0] - knots[i][0])
            return lerp3(knots[i][1:], knots[i + 1][1:], f)
    return knots[-1][1:]


# tube palette (median colour per u bin of the still): K1 = the first cycle from the checker edge, K2 = later cycles
K2 = [(0.000, 190, 108, 12), (0.110, 190, 108, 12), (0.140, 92, 94, 52), (0.17, 89, 97, 62),
      (0.24, 101, 113, 91), (0.32, 136, 145, 125), (0.44, 178, 186, 170), (0.56, 213, 220, 206), (0.68, 249, 253, 240),
      (0.72, 254, 255, 245), (0.96, 252, 255, 244), (1.00, 234, 242, 230), (1.08, 201, 212, 198), (1.16, 169, 182, 168),
      (1.24, 139, 155, 139), (1.32, 110, 127, 110), (1.40, 82, 100, 82), (1.48, 61, 76, 56), (1.56, 46, 60, 38)]
K1 = [(0.000, 88, 92, 64), (0.03, 92, 96, 68), (0.08, 100, 104, 78), (0.16, 114, 122, 102), (0.24, 140, 149, 131),
      (0.32, 168, 177, 160), (0.40, 199, 205, 190), (0.48, 229, 233, 219), (0.56, 252, 254, 242), (0.62, 254, 255, 245),
      (0.96, 252, 255, 244), (1.00, 234, 242, 230), (1.08, 201, 212, 198), (1.16, 169, 182, 168), (1.24, 139, 155, 139),
      (1.32, 110, 127, 110), (1.40, 82, 100, 82), (1.48, 61, 76, 56), (1.56, 46, 60, 38)]
PAL_RGB = []
for first in range(2):
    for t in range(256):
        u = (t + 0.5) / 256.0 * PER
        PAL_RGB.append(plin(K2 if first == 0 else K1, u))
PAL_ROM = [rgb_hw(*c) for c in PAL_RGB]

# checker colouriser: s = G-high fraction (0..31), z = zone (0..7)
LHI0, LLO0 = (255, 184, 8), (255, 52, 50)      # yellow / red
LHI1, LLO1 = (255, 180, 9), (66, 45, 69)       # zone: yellow / dark violet-grey
QHI0, QLO0 = (255, 108, 9), (196, 14, 182)     # Q (sync lost): orange / violet
QHI1, QLO1 = (34, 100, 6), (2, 8, 208)         # Q zone: green / blue


def clip01(v):
    return min(1.0, max(0.0, v))


CHK_RGB = []
for st in range(2):
    for z in range(8):
        for s in range(32):
            zf = z / 7.0
            sf = min(1.0, s / 31.0 * (1.2 if st == 0 else 1.6))     # G high dominates (the still's shares)
            if st == 0:
                zl = clip01((zf - 0.30) / 0.40)
                c = lerp3(lerp3(LLO0, LLO1, zl), lerp3(LHI0, LHI1, zl), sf)
            else:
                zlo = clip01((zf - 0.05) / 0.35)            # violet -> blue early in the zone
                zhi = clip01((zf - 0.40) / 0.50)            # orange -> olive -> green only at its depth
                c = lerp3(lerp3(QLO0, QLO1, zlo), lerp3(QHI0, QHI1, zhi), sf)
            CHK_RGB.append(c)
CHK_ROM = [rgb_hw(*c) for c in CHK_RGB]
FRINGE_RGB = (230, 88, 58)        # light enough that the thin fringe never reads as a dark line
SCAR_RGB = (200, 205, 195)
SCARD_RGB = (40, 50, 40)
FRINGE = rgb_hw(*FRINGE_RGB)
SCAR = rgb_hw(*SCAR_RGB)
SCARD = rgb_hw(*SCARD_RGB)

# ------------------------------------------------------------------ V oscillator: Fy (period 512 entries) and the zone
FY_AMP = 1.0 / LSB          # 164.1 LSB
V_ROM = []
for j in range(512):
    a = 2 * math.pi * (j + 0.5) / 512
    fy = int(round(FY_AMP * math.cos(a)))
    z = int(round(7 * clip01(math.cos(a + 2 * math.pi * 56 / 280.0))))
    V_ROM.append((z, fy))

EXP2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


# ------------------------------------------------------------------ package
def tc(v, w):
    return v & ((1 << w) - 1)


def write_pkg(path):
    L = []
    L.append('-- Generated by tatter.py -- do not edit.')
    L.append('library ieee;')
    L.append('use ieee.std_logic_1164.all;')
    L.append('use ieee.numeric_std.all;')
    L.append('package tatter_pkg is')
    L.append('    type t_tt_fx is array (0 to 2047) of std_logic_vector(9 downto 0);')
    L.append('    type t_tt_c is array (0 to 511) of std_logic_vector(29 downto 0);')
    L.append('    type t_tt_v is array (0 to 511) of std_logic_vector(15 downto 0);')
    L.append('    type t_tt_e is array (0 to 255) of unsigned(15 downto 0);')
    L.append('    constant C_TT_FRINGE : std_logic_vector(29 downto 0) := "%s";'
             % format((FRINGE[0] << 20) | (FRINGE[1] << 10) | FRINGE[2], '030b'))
    L.append('    constant C_TT_SCAR : std_logic_vector(29 downto 0) := "%s";'
             % format((SCAR[0] << 20) | (SCAR[1] << 10) | SCAR[2], '030b'))
    L.append('    constant C_TT_SCARD : std_logic_vector(29 downto 0) := "%s";'
             % format((SCARD[0] << 20) | (SCARD[1] << 10) | SCARD[2], '030b'))

    def arr(name, typ, vals, fmt):
        L.append('    constant %s : %s := (' % (name, typ))
        row = []
        for k, v in enumerate(vals):
            row.append(fmt(v))
            if len(row) == 8:
                L.append('        ' + ', '.join(row) + (',' if k < len(vals) - 1 else ''))
                row = []
        if row:
            L.append('        ' + ', '.join(row))
        L.append('    );')

    arr('C_TT_FX', 't_tt_fx', FX_ROM, lambda v: '"%s"' % format(tc(v, 10), '010b'))
    yuv = lambda c: '"%s"' % format((c[0] << 20) | (c[1] << 10) | c[2], '030b')
    arr('C_TT_PAL', 't_tt_c', PAL_ROM, yuv)
    arr('C_TT_CHK', 't_tt_c', CHK_ROM, yuv)
    arr('C_TT_V', 't_tt_v', V_ROM, lambda v: 'x"%04x"' % ((v[0] << 10) | tc(v[1], 10)))
    arr('C_TT_EXP2', 't_tt_e', EXP2, lambda v: 'to_unsigned(%d, 16)' % v)
    L.append('end package tatter_pkg;')
    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    write_pkg(os.path.join(HERE, 'tatter_pkg.vhd'))
