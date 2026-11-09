#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
QUILL v0.1.0 - build hook.  Writes quill_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the still WIN_20230910_00_11_50 @ 7.0 s (see .heirloom_work/rb_quill/notes.md):

  phi    = (x + 130) k(y)                  a VCO hard-synced 130 px before the picture (the HD back porch)
  k(y)   = k0 2^(g E(a)),  a = |y - spine|  exponential FM by a parabola in the distance from the spine,
           E = 2.29e-6 (a - 220)^2 + 0.0011 max(0, 190 - a)   plus a cusp at the spine
  B      = body key: a * gb  <  HB[frac(phi)]     (three plumes per period of different reach)
  G, R   = barbs: windows round six line phases, dotted by a second oscillator psi = 0.042 d (n + line phase);
           the spine band and the far field add a texture key on frac(d / 44 + 0.45 WV(phi) + 0.45 noise)

Package contents (also read by .heirloom_work/rb_quill/imodel.py):
  C_QL_ROM  256 x 32 by macro phase (frac(phi) * 256):
            [31:22] HB  body reach, lines from the spine
            [21:14] e   signed distance to the nearest barb line, 1/1024 period (+-127 = far)
            [13:11] li  that line's index (0..5)
            [10:4]  wv  band wiggle 0.45 * (1 - cos 6 pi f) / 2, Q8 of a cycle
  C_QL_PAL  16 x 32 {Y, U, V, 00}: palette * 8 + code (code = R G B key bits)
  C_QL_EXP2 256 x 16 round(32768 * 2^(i / 256))
  C_QL_SIN  1024 x 16 round(32767 * sin(2 pi i / 1024)), two's complement
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
# body reach by macro phase (256 bins): the row where the still's blue ends, lines below the spine (y 45),
# plume peaks stretched x1.9 above 480 (the thin blue continuations)
HB = [260, 246, 232, 228, 225, 223, 221, 220, 220, 220, 220, 221, 222, 224, 227, 229, 232, 237, 242, 247, 256, 266,
      283, 299, 317, 339, 360, 376, 391, 404, 412, 418, 424, 429, 434, 439, 443, 446, 450, 453, 458, 473, 514, 567,
      621, 670, 689, 685, 657, 625, 567, 488, 439, 400, 356, 324, 308, 292, 276, 264, 254, 246, 240, 236, 232, 229,
      226, 225, 224, 224, 225, 225, 225, 225, 225, 223, 222, 221, 219, 218, 217, 216, 216, 215, 215, 216, 218, 221,
      224, 226, 229, 231, 233, 235, 238, 240, 242, 244, 245, 246, 246, 247, 248, 248, 247, 247, 246, 245, 244, 244,
      246, 247, 249, 251, 254, 256, 259, 263, 268, 277, 287, 298, 309, 320, 330, 339, 353, 368, 382, 394, 407, 414,
      419, 425, 429, 433, 437, 441, 445, 449, 454, 459, 486, 530, 577, 653, 732, 766, 784, 794, 759, 707, 659, 624,
      561, 505, 469, 431, 388, 363, 334, 304, 289, 278, 266, 257, 251, 246, 242, 241, 241, 242, 245, 247, 250, 252,
      255, 257, 259, 261, 263, 269, 276, 286, 296, 307, 315, 322, 329, 338, 346, 357, 370, 382, 393, 403, 411, 417,
      423, 428, 433, 437, 441, 444, 448, 453, 467, 496, 541, 582, 623, 684, 718, 735, 751, 762, 735, 721, 717, 702,
      688, 675, 662, 650, 648, 646, 645, 646, 648, 661, 675, 688, 702, 642, 569, 495, 448, 407, 412, 417, 424, 467,
      538, 607, 677, 744, 716, 686, 656, 556, 464, 420, 372, 322, 299, 279]
GL = [40, 139, 148, 206, 220, 234]      # barb line phases (1/256 period)
# the still's core colours (RGB) by key code R G B; Ember = the alternative palette
ICE = [(0, 0, 1), (0, 0, 246), (0, 250, 0), (0, 238, 249), (231, 0, 0), (243, 0, 255), (225, 223, 0), (225, 221, 254)]
EMB = [(0, 0, 1), (235, 50, 0), (0, 200, 200), (250, 220, 0), (40, 40, 240), (255, 120, 200), (150, 60, 255),
       (240, 240, 240)]


def rom_words():
    out = []
    for i in range(256):
        best = None
        for j, g in enumerate(GL):
            e = (i - g + 128) % 256 - 128          # 1/256 period
            if best is None or abs(e) < abs(best[0]):
                best = (e, j)
        e4 = max(-127, min(127, best[0] * 4))      # 1/1024 period
        wv = int(round(0.45 * (0.5 - 0.5 * math.cos(2 * math.pi * 3 * i / 256.0)) * 256))
        out.append((min(1023, HB[i]) << 22) | ((e4 & 0xFF) << 14) | (best[1] << 11) | (wv << 4))
    return out


def ycc(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, (b - y) / 1.772, (r - y) / 1.402


def hw10(rgb):
    """rgb -> hardware (Y pre-scaled 16/13, capped 940; U = Cr, V = Cb)"""
    y, cb, cr = ycc(rgb)
    yy = 64 + y * 876
    yy = 64 + (yy - 64) * 16.0 / 13.0
    yy = max(64, min(940, int(round(yy))))
    u = max(4, min(1020, int(round(512 + cr * 896))))
    v = max(4, min(1020, int(round(512 + cb * 896))))
    return yy, u, v


def pal_words():
    out = []
    for pal in (ICE, EMB):
        for c in pal:
            y, u, v = hw10(c)
            out.append((y << 22) | (u << 12) | (v << 2))
    return out


def tables():
    rom = rom_words()
    pal = pal_words()
    exp2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    sin = [int(round(32767 * math.sin(2 * math.pi * i / 1024.0))) & 0xFFFF for i in range(1024)]
    return rom, pal, exp2, sin


def vhd_array(name, typ, vals, width):
    digits = (width + 3) // 4
    body = ',\n'.join('        ' + ', '.join('x"%0*X"' % (digits, v) for v in vals[i:i + 8]) for i in range(0, len(vals), 8))
    return '    constant %s : %s := (\n%s);\n' % (name, typ, body)


def main():
    rom, pal, exp2, sin = tables()
    s = ['-- Generated by quill.py (Quill v0.1.0): do not edit; edit quill.py and rebuild.',
         'library ieee;', 'use ieee.std_logic_1164.all;', 'use ieee.numeric_std.all;', '',
         'package quill_pkg is',
         '    type t_ql_w32_256 is array (0 to 255) of std_logic_vector(31 downto 0);',
         '    type t_ql_w32_16 is array (0 to 15) of std_logic_vector(31 downto 0);',
         '    type t_ql_u16_256 is array (0 to 255) of unsigned(15 downto 0);',
         '    type t_ql_s16_1k is array (0 to 1023) of signed(15 downto 0);']
    s.append(vhd_array('C_QL_ROM', 't_ql_w32_256', rom, 32))
    s.append(vhd_array('C_QL_PAL', 't_ql_w32_16', pal, 32))
    s.append(vhd_array('C_QL_EXP2', 't_ql_u16_256', exp2, 16))
    s.append(vhd_array('C_QL_SIN', 't_ql_s16_1k', sin, 16))
    s.append('end package quill_pkg;')
    with open(os.path.join(HERE, 'quill_pkg.vhd'), 'w') as f:
        f.write('\n'.join(s) + '\n')


if __name__ == '__main__':
    main()
