#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
MARMALADE - build hook (pure Python; the builder runs it with the system python3).

Generates marmalade_pkg.vhd from tables measured on the photo (WIN_20230307_23_36_05):

  C_ZT   1024 x 4   the Osc source: the photo's sample-and-hold pattern as colourizer
                    zones, 16 rows (one cycle of the vertical master) x 64 cells
                    (address row * 64 + cell).  Columns 0..49 are measured (cell -3 .. 46
                    of the 40.24 px grid, outer rows re-aligned by the 90 px shift),
                    50..63 mirror 49..36.
  C_ANC  128 x 30   palette anchors {mode(2), tint(2), stage(3)} -> Y10 & U10 & V10.
                    modes: 0 = magenta stage palette, 1 = dark stage palette,
                    2 = warm flat keys, 3 = cool flat keys (S8).  tint 0..2 are the
                    photo's three orange-mix levels, tint 3 = full orange.  Luma is
                    pre-compensated for the shell's 13/16 output scale (capped at 940;
                    saturated capped colours get their chroma re-solved).  U = Cr,
                    V = Cb (hardware convention).
  C_SLW  1024 x 8   slew decay {s(4), d(6)} -> round(255 (1 - a)^d), a = 2^(-0.4 s).
  C_SIN  256 x 9    signed sine, +-255.

Zone order (the colourizer's level order, fitted to the edge slivers):
  0 M0 magenta  1 M1 pink  2 M2 red  3 Of orange key  4 D2 dark orange
  5 D1 brown    6 D0 dark  7 Af amber key  8 Yf yellow key
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

ZT = [
    "8333377700333333311000666666111778844442770033355555533300772444",
    "3333666000333333121000066663333778825557778821555555551288777555",
    "2553555100333333555100000668888778866666777330015555100337776666",
    "8665444200333333555100007776660770077776660228815555188220666777",
    "8335577700333333331100007777411776664444200773355555533770024444",
    "3335560000333333331100007777333776665555100001155555511000015555",
    "2555555100333333555100007778888770066666660110015555100110666666",
    "8665542200333333331100008888600771667777700338855555588330077777",
    "8335777700333333311100008888211770064444600883325555233880064444",
    "3335560000333333351100008888333771665555600001155555511000065555",
    "2555555100333333321000068888888770066666001100515555150011006666",
    "8666642200333333311000664444600771177777117788815555188877117777",
    "1666777700333333521000024445551771144445212233555555553322125444",
    "8666660000333333311000664444333771155551111115555555555111111555",
    "1666551100333333121000064448888771166661210000615555160000121666",
    "8666642200333333311000664444600771177777117788815555188877117777",
]

# measured family colours (RGB), stages k0..k4
FAM = {
    'M0': [(64, 65, 47), (130, 123, 117), (244, 224, 234), (11, 233, 244), (255, 22, 253)],
    'M1': [(168, 75, 49), (174, 107, 78), (254, 160, 140), (132, 163, 145), (255, 45, 147)],
    'M2': [(237, 97, 18), (201, 100, 64), (255, 144, 69), (222, 145, 72), (255, 83, 76)],
    'D0': [(244, 225, 234), (27, 220, 234), (255, 240, 0), (34, 39, 19), (78, 63, 47)],
    'D1': [(244, 175, 161), (93, 182, 165), (255, 179, 0), (108, 53, 15), (132, 70, 34)],
    'D2': [(255, 143, 69), (222, 146, 70), (255, 154, 2), (229, 87, 8), (237, 95, 15)],
    'Yf': [(255, 240, 0)] * 5,
    'Af': [(255, 178, 0)] * 5,
    'Of': [(255, 153, 18), (255, 153, 18), (255, 136, 3), (255, 136, 3), (255, 136, 3)],
}
ORANGE = (255, 104, 12)                       # tint 3: the full orange the keyer mixes in
COOL = [[(30, 200, 235)] * 5, [(60, 120, 230)] * 5, [(70, 60, 200)] * 5, [(90, 40, 170)] * 5]

def anchors_rgb():
    """A[mode][tint][k] -> RGB"""
    A = [[None] * 4 for _ in range(4)]
    A[0] = [FAM['M0'], FAM['M1'], FAM['M2'], [ORANGE] * 5]
    A[1] = [FAM['D0'], FAM['D1'], FAM['D2'], [ORANGE] * 5]
    A[2] = [FAM['Yf'], FAM['Af'], FAM['Of'], [ORANGE] * 5]
    A[3] = COOL
    return A

def rgb_to_ycc(r, g, b):
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, (b - y) / 1.772, (r - y) / 1.402

def precomp(rgb):
    """10-bit (Y, U=Cr, V=Cb) with the luma pre-compensated for the 13/16 output scale."""
    y, cb, cr = rgb_to_ycc(*rgb)
    ys = y * 16 / 13
    if ys > 1.0:
        if math.hypot(cb, cr) > 0.15:
            # re-solve chroma at the displayed (capped) luma: least squares in RGB
            yc = 13 / 16
            t = [c / 255.0 - yc for c in rgb]
            # RGB - yc = [1.402 Cr, -.344136 Cb - .714136 Cr, 1.772 Cb]
            a11 = 0.344136 ** 2 + 1.772 ** 2; a12 = 0.344136 * 0.714136; a22 = 1.402 ** 2 + 0.714136 ** 2
            b1 = -0.344136 * t[1] + 1.772 * t[2]; b2 = 1.402 * t[0] - 0.714136 * t[1]
            det = a11 * a22 - a12 * a12
            cb = (b1 * a22 - a12 * b2) / det
            cr = (a11 * b2 - a12 * b1) / det
        ys = 1.0
    cb = max(-0.5, min(0.5, cb)); cr = max(-0.5, min(0.5, cr))
    Y = int(round(64 + ys * 876)); U = int(round(512 + cr * 896)); V = int(round(512 + cb * 896))
    return max(64, min(940, Y)), max(64, min(960, U)), max(64, min(960, V))

def anc_table():
    A = anchors_rgb()
    out = []
    for m in range(4):
        for t in range(4):
            for k in range(8):
                out.append(precomp(A[m][t][min(k, 4)]))
    return out

def slw_table():
    out = []
    for s in range(16):
        a = 2.0 ** (-0.4 * s)
        for d in range(64):
            out.append(int(round(255 * (1 - a) ** d)) if a < 1 else (255 if d == 0 else 0))
    return out

def sin_table():
    return [int(round(255 * math.sin(2 * math.pi * i / 256))) for i in range(256)]

def zt_table():
    return [int(ZT[r][c]) for r in range(16) for c in range(64)]

def slv(v, w):
    return '"' + format(v & ((1 << w) - 1), '0%db' % w) + '"'

def emit(name, typ, vals, w, per=8):
    lines = ['    constant %s : %s := (' % (name, typ)]
    for i in range(0, len(vals), per):
        chunk = ', '.join(slv(v, w) for v in vals[i:i + per])
        lines.append('        ' + chunk + (',' if i + per < len(vals) else ''))
    lines.append('    );')
    return '\n'.join(lines)

def main():
    anc = [(y << 20) | (u << 10) | v for (y, u, v) in anc_table()]
    txt = [
        '-- Generated by marmalade.py -- do not edit.',
        'library ieee;', 'use ieee.std_logic_1164.all;', '',
        'package marmalade_pkg is',
        '    type t_zt is array (0 to 1023) of std_logic_vector(3 downto 0);',
        '    type t_anc is array (0 to 127) of std_logic_vector(29 downto 0);',
        '    type t_slw is array (0 to 1023) of std_logic_vector(7 downto 0);',
        '    type t_sin is array (0 to 255) of std_logic_vector(8 downto 0);',
        emit('C_ZT', 't_zt', zt_table(), 4, 16),
        emit('C_ANC', 't_anc', anc, 30, 4),
        emit('C_SLW', 't_slw', slw_table(), 8, 16),
        emit('C_SIN', 't_sin', sin_table(), 9, 16),
        'end package marmalade_pkg;', '',
    ]
    with open(os.path.join(HERE, 'marmalade_pkg.vhd'), 'w') as f:
        f.write('\n'.join(txt))

if __name__ == '__main__':
    main()
