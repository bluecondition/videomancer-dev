#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
YOLK v0.1.0 - build hook.  Writes yolk_pkg.vhd next to this file (pure python: the builder runs it with the system
python3; .heirloom_work/rb_yolk/imodel.py imports the same tables).

Fitted to the still WIN_20240111_01_01_24 at 0.5 s (see .heirloom_work/rb_yolk/notes.md).  The patch is an RGB
colouriser fed by two mixes of the same two oscillators:

  T(x)   = mirrored H triangle about x = 993.5 (period 425 px, the eggs at |dx| = 259 + 425 k), 0 at the egg
           columns, 1 between them; the centre is a half-depth notch; held at its x = 90 value at the line start
  VG(y)  = the G mix's vertical wave (period 1055 lines, sharp-ish max at y = 950, rounded min at y = 345)
  VR(y)  = the R mix's vertical wave: a shallower bowl with a long flat bottom (the R core is a vertical slot)
  F_G    = T + VG,  F_R = (T + VR - c0) * (1 + 0.5 (x - 960) / 960) + c0     (the R gain leans across the frame)
  key    = static window (y < 264 or y >= 770) and F_G > 0.55: the white layer
  colour = R(F_R), G(F_G), B(F_G) through the palette ROMs; in the white layer R, G, B all read F_G

Package contents (F in 1/64 units of T: index i covers F = i / 64 - 3, i = 0 .. 511):
  C_YK_V     512 x 24   {VG, VR} (Q10 two's complement, 12 bits each) by the vertical phase psi (index = psi >> 15)
  C_YK_R     2048 x 8   R:      index = pal * 1024 + key * 512 + i
  C_YK_GB    2048 x 16  {G, B}: index = pal * 1024 + key * 512 + i
  C_YK_EXP2  256 x 16   round(32768 * 2^(i / 256))
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PV = 1055.0          # vertical period (virtual lines)
YMAX = 950.0         # VG / VR maximum in the still
NV = 512             # V table entries per period


# ------------------------------------------------------------------ vertical waves (deficits below the max, T units)
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
    i = 0
    while i < n - 2 and x > xk[i + 1]:
        i += 1
    t = (x - xk[i]) / h[i]
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1
    h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2
    h11 = t ** 3 - t ** 2
    return h00 * yk[i] + h10 * h[i] * m[i] + h01 * yk[i + 1] + h11 * h[i] * m[i + 1]


# lo: distance d = 950 - y going up from the bottom max; up: d = y + 105 going down from the top max (the same max
# one period earlier).  Measured on the still: ring sizes, the G-diamond widths, the key edge, the U-flame slopes.
VG_LO = [(0, 0.0), (74, 0.17), (142, 0.625), (180, 0.95), (250, 1.26), (350, 1.61), (450, 1.81), (510, 2.0), (605, 2.15)]
VG_UP = [(0, 0.0), (74, 0.17), (130, 0.50), (230, 0.70), (260, 1.05), (300, 1.60), (335, 1.83), (365, 1.93),
         (395, 2.07), (450, 2.15)]
VR_LO = [(0, 0.0), (120, 0.40), (180, 0.93), (250, 1.38), (329, 1.594), (405, 1.781), (450, 1.91), (500, 1.94),
         (605, 1.956)]
VR_UP = [(0, 0.0), (74, 0.17), (130, 0.48), (230, 0.68), (260, 1.00), (296, 1.594), (346, 1.781), (385, 1.93),
         (450, 1.956)]
VG_MAX, VR_MAX = 1.60, 1.83


def side_table(vmax, up, lo, n=NV):
    """V over the proto phase phi = (YMAX - y) / PV (0 at the bottom max, rising up the screen), n samples."""
    dlo = lo[-1][0]
    out = []
    for k in range(n):
        q = (k + 0.5) / n * PV
        if q <= dlo:
            v = vmax - pchip([p[0] for p in lo], [p[1] for p in lo], q)
        else:
            v = vmax - pchip([p[0] for p in up], [p[1] for p in up], PV - q)
        out.append(v)
    return out


VG_T = side_table(VG_MAX, VG_UP, VG_LO)
VR_T = side_table(VR_MAX, VR_UP, VR_LO)
# RTL phase psi = (y - YMAX) / PV + scroll = -phi: ROM[j] = V_T[511 - j]
V_ROM = [(int(round(VG_T[NV - 1 - j] * 1024)), int(round(VR_T[NV - 1 - j] * 1024))) for j in range(NV)]


# ------------------------------------------------------------------ palettes (F = i / 64 - 3)
NF = 512
FLO = -3.0


def ramp(f, a, b):
    return min(1.0, max(0.0, (f - a) / (b - a)))


def bump(f, c, w):
    return min(1.0, max(0.0, 1 - abs(f - c) / w))


def wrapq(f, l0, per):
    return ((f - l0) / per + 0.5) % 1.0 - 0.5


def pal_entry(pal, key, f):
    """(R, G, B) 0..1 for palette pal, layer key (0 = heat layer, 1 = white layer), field value f (T units)."""
    if pal == 0:      # Yolk: black core -> red -> yellow -> green fold; white layer with magenta lines
        if key == 0:
            r = ramp(f, 0.06, 0.50) * (1 - 0.85 * bump(f, 1.27, 0.22))
            g = ramp(f, 0.20, 0.62) ** 1.2
            g *= (1 - 0.8 * bump(f, 1.55, 0.06)) * (1 - 0.8 * bump(f, 1.95, 0.06)) * (1 - 0.8 * bump(f, 2.35, 0.06))
            g *= (1 - 0.8 * bump(f, 2.75, 0.06)) * (1 - 0.8 * bump(f, 3.15, 0.06))
            b = 0.55 * ramp(f, 1.65, 2.1) * (1 - 0.6 * ramp(f, 2.9, 3.3))
            return r, g, b
        q = wrapq(f, 0.03, 0.40)
        mag = max(0.0, 1 - abs(q) / 0.09) ** 0.35
        g = 1 - 0.97 * mag
        yel = max(0.0, 1 - abs(q + 0.25) / 0.10) ** 0.5
        b = 1 - 0.90 * yel
        red = max(0.0, 1 - abs(q + 0.105) / 0.035)
        b -= 0.9 * red
        g -= 0.85 * red
        g -= 0.30 * max(0.0, 1 - abs(q - 0.15) / 0.06)
        return 1.0, g, b
    # Lagoon (the sibling clip WIN_20240111_00_56_40): red ground, green / cyan / blue rings, white-yellow highs
    if key == 0:
        r = 1 - 0.95 * ramp(f, 0.05, 0.30)
        r += 0.9 * bump(f, 1.35, 0.22) + 0.85 * ramp(f, 2.0, 2.4)
        g = ramp(f, 0.12, 0.40) * (1 - 0.75 * bump(f, 0.95, 0.10))
        g *= (1 - 0.8 * bump(f, 1.60, 0.06)) * (1 - 0.8 * bump(f, 2.40, 0.06))
        b = ramp(f, 0.40, 0.75) * (1 - 0.6 * ramp(f, 1.1, 1.5)) + 0.8 * ramp(f, 1.8, 2.2)
        r = min(1.0, r)
        return r, g, b
    q = wrapq(f, 0.03, 0.40)
    line = max(0.0, 1 - abs(q) / 0.09) ** 0.35
    r = 1 - 0.85 * line
    g = 1 - 0.25 * line
    b = 0.25 + 0.75 * max(0.0, 1 - abs(q - 0.22) / 0.22)
    return r, g, b


def q8(v):
    return int(round(min(1.0, max(0.0, v)) * 255))


R_ROM = []
GB_ROM = []
for pal in range(2):
    for key in range(2):
        for i in range(NF):
            f = FLO + i / 64.0
            r, g, b = pal_entry(pal, key, f)
            R_ROM.append(q8(r))
            GB_ROM.append((q8(g) << 8) | q8(b))

EXP2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


# ------------------------------------------------------------------ package
def tc(v, w):
    return v & ((1 << w) - 1)


def write_pkg(path):
    L = []
    L.append('-- Generated by yolk.py -- do not edit.')
    L.append('library ieee;')
    L.append('use ieee.std_logic_1164.all;')
    L.append('use ieee.numeric_std.all;')
    L.append('package yolk_pkg is')
    L.append('    type t_yk_v is array (0 to 511) of std_logic_vector(23 downto 0);')
    L.append('    type t_yk_r is array (0 to 2047) of std_logic_vector(7 downto 0);')
    L.append('    type t_yk_gb is array (0 to 2047) of std_logic_vector(15 downto 0);')
    L.append('    type t_yk_e is array (0 to 255) of unsigned(15 downto 0);')

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

    arr('C_YK_V', 't_yk_v', V_ROM, lambda v: 'x"%06x"' % ((tc(v[0], 12) << 12) | tc(v[1], 12)))
    arr('C_YK_R', 't_yk_r', R_ROM, lambda v: 'x"%02x"' % v)
    arr('C_YK_GB', 't_yk_gb', GB_ROM, lambda v: 'x"%04x"' % v)
    arr('C_YK_EXP2', 't_yk_e', EXP2, lambda v: 'to_unsigned(%d, 16)' % v)
    L.append('end package yolk_pkg;')
    with open(path, 'w') as fh:
        fh.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    write_pkg(os.path.join(HERE, 'yolk_pkg.vhd'))
