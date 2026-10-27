#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SHIELD v0.5.0 - build hook.  Writes shield_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).

Fitted to the photo WIN_20230529_22_56_41 (see .heirloom_work/rb_shield/notes.md and gfit.json):

  u   = distance to the nearest shield axis, an H triangle (period 941.7 px, axes 526.2 + 941.7 m),
        held at its x = 66 value at the line start; V tips rounded: u' = sqrt(u^2 + 6.6^2)
  F2  = u' + depth * (5.9517e-4 (y + 2.24)^2 + 4.19)       the H triangle plus a V parabola: nested U shields
  F1  = 1741.4 - 2.892 u' - 0.2496 (y - 700) - 1.406e-4 (y - 700)^2    a steep inverted triangle + V ramp
  F   = min(F2, F1) + etch * luma      (non-additive mix: F1 wins only near the seams in the lower half,
                                         giving the blue pillars, the M body's sides and the seam stripes)
  colouriser on F mod 1024, with a horizontal-stripe square wave (half period 73.545 lines, phase 56.39)
  selecting the band pair:
    F < 427.7 (ramp frame):  bands n = floor(F_k / 93.3) (keyer frame): orange rows Ye Or Ye Or R1,
                             red rows OR Rd R1 R2 OR (n = 0..4 in the photo; tables continue mod 8)
    427.7 .. 641.7:          dark-red ramp then navy -> blue ramp (linear in F, ramp frame)
    641.7 .. 830.5 (keyer):  the red M body
    >= 830.5 (keyer):        bright blue (the bottom triangles)
  The keyer path (band edges, M, triangles, white line) runs 10.5 px behind the linear ramp path
  (the photo's comparator delay), so the ramps sit 10 px left of the bands.
  White line: a one-shot where F_k falls through 530 after having exceeded 552 (scan order); lines on the
  steep F1 side fade with the line's crease peak (560 .. 665).  Dark line: a one-shot where F (ramp frame)
  rises through 427.7 (the right arms' thin dark edge).

Package contents:
  C_SH_PAL  512 x 32  {Y, U, V, d, w}: index = pal * 256 + idx;  idx 0..127 the ramp at 2 px steps
                      (F = 427.7 + 2 i + 1), 128 + s * 8 + n the band colours (s = 0 orange row, 1 red row),
                      144 M body, 145 bright blue, 146 white line, 147 black.  d = the dark line may draw on
                      this entry (the ramp), w = the white line may (ramp and bands): real data in the two low
                      lanes, so the ROM maps as a full 32-bit word
  C_SH_TIP  64 x 5    round(4 * (sqrt((i + 0.5)^2 + 6.6^2) - (i + 0.5)))   tip rounding, Q2 px, u < 64 px
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.  Palette 1 ("Azure") is palette 0 with the chroma inverted (U, V -> 1024 - U, V).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (virtual px / lines)
TZ, TM, TT = 427.72, 641.72, 830.53
LW, LH = 530.0, 552.0

YE = (255, 151, 1); OR_ = (255, 105, 1); OR3 = (255, 103, 1); R1 = (201, 12, 7)
ORR = (254, 55, 0); RD = (230, 55, 1); R2 = (160, 13, 10)
MRED = (159, 14, 12); BBLUE = (13, 9, 250); WLINE = (235, 225, 230); BLACK = (16, 12, 12)
TAB_O = [YE, OR_, YE, OR3, R1, OR_, YE, OR3]
TAB_R = [ORR, RD, R1, R2, ORR, RD, R1, R2]


def ramp_rgb(f):
    """The outline ramps (linear path), photo RGB, F in px (ramp frame)."""
    r = min(max(157 + 1.5 * (440 - f), 9), 255)
    t = min(max((f - 535) / 95.0, 0.0), 1.0)
    if f < 535:
        b = min(max(2 + (f - 440) * 0.25, 2), 30)
    else:
        b = 45 + 184 * t
    g = min(max(9 + (f - 440) * 0.05, 9), 14) + 15 * t
    if f > 535:
        r = 17 - 8 * t
    return (r, g, b)


def rgb_to_hw10(rgb):
    """photo RGB 0..255 -> (Y, U = Cr, V = Cb) 10-bit, BT.601 (Y = 64 + 876 Y')."""
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + 876 * y)), int(round(512 + 896 * cr)), int(round(512 + 896 * cb)))


def pal_entry(rgb, pal):
    y, u, v = rgb_to_hw10(rgb)
    y = min(940, int(round(64 + (y - 64) * 16 / 13.0)))
    if pal:
        u, v = 1024 - u, 1024 - v
    u = min(max(u, 1), 1023)
    v = min(max(v, 1), 1023)
    return y, u, v


def palette():
    """512 entries of (Y, U, V)."""
    out = []
    for pal in (0, 1):
        ent = []
        for i in range(128):
            ent.append(pal_entry(ramp_rgb(TZ + 2 * i + 1), pal))
        for s in (0, 1):
            for n in range(8):
                ent.append(pal_entry((TAB_O if s == 0 else TAB_R)[n], pal))
        ent.append(pal_entry(MRED, pal))
        ent.append(pal_entry(BBLUE, pal))
        ent.append(pal_entry(WLINE, pal))
        ent.append(pal_entry(BLACK, pal))
        while len(ent) < 256:
            ent.append(pal_entry(BLACK, pal))
        out += ent
    return out


def flags():
    """(d, w) per entry: dark line allowed on the ramp, white line on the ramp and the bands."""
    f = []
    for pal in (0, 1):
        for i in range(256):
            f.append((1 if i < 128 else 0, 1 if i < 144 else 0))
    return f


def tip_table():
    return [int(round(4 * (math.sqrt((i + 0.5) ** 2 + 6.59 ** 2) - (i + 0.5)))) for i in range(64)]


def main():
    pal = palette()
    flg = flags()
    tip = tip_table()
    L = []
    L.append('-- Generated by shield.py - do not edit.')
    L.append('library ieee;')
    L.append('use ieee.std_logic_1164.all;')
    L.append('use ieee.numeric_std.all;')
    L.append('')
    L.append('package shield_pkg is')
    L.append('    type t_sh_pal is array (0 to 511) of std_logic_vector(31 downto 0);')
    L.append('    type t_sh_tip is array (0 to 63) of unsigned(4 downto 0);')
    L.append('    constant C_SH_PAL : t_sh_pal := (')
    rows = []
    for i, (y, u, v) in enumerate(pal):
        w = (y << 22) | (u << 12) | (v << 2) | (flg[i][0] << 1) | flg[i][1]
        rows.append('        %d => x"%08X"' % (i, w))
    L.append(',\n'.join(rows))
    L.append('    );')
    L.append('    constant C_SH_TIP : t_sh_tip := (')
    L.append(',\n'.join('        %d => to_unsigned(%d, 5)' % (i, t) for i, t in enumerate(tip)))
    L.append('    );')
    L.append('end package shield_pkg;')
    with open(os.path.join(HERE, 'shield_pkg.vhd'), 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
