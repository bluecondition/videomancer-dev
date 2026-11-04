#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TONGUES v0.1.0 - build hook.  Writes tongues_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).

Fitted to the clip still WIN_20230127_01_48_55 at 25.5 s (see .heirloom_work/rb_tongues/notes.md):

  cos    = cos(2 pi (vy - yc_k - psi_k xl) / P)        a vertical cosine, P = 530 virtual lines (2.04 cycles per
                                                        frame), its phase stepped per column (yc_k) and tilted
                                                        within a column (psi_k); vy in virtual 1080 lines
  column = 8 columns across the width (k = 0..7), xl = the position inside the column (0..1)
  window = [C - w, C + w] on the cosine; C = C0_k + s_k xl (the level rises along each column: the 8x saw),
           w = w_k + wr_k xl + wv (|vy - 530| - 260) / 530 (the key opens from left to right)
  G      = min(8 (cos - (C - w)), khi ((C + w) - cos)) clipped 0..1   a window key, sharp low edge, soft high edge
  R      = clip(a_k + b_k cos(2 pi (vy - 530) / P), floor_k, 255)      the red flame hump, per column
  colour = (R, G, 0) in RGB ("Fire"); "Ice" draws R as indigo and G as cyan

Package contents (also read by imodel.py):
  C_TG_COS   1024 x 12  round(2047 cos(2 pi i / 1024)), signed
  C_TG_EXP2  256 x 16   round(32768 * 2^(i/256))
  per-column tables (8 entries): C_TG_YC (virtual lines, the still's tilt removed), C_TG_YCF (= YC + PSI, the
  column END phase, loaded under Flip), C_TG_SGN (tilt sign), C_TG_PSI (lines per column), C_TG_C0 / C_TG_S /
  C_TG_W / C_TG_WR (Q11), C_TG_WE (= W + WR, the END width), C_TG_A / C_TG_B / C_TG_FL (8-bit R)
  The RTL computes the red per line for columns 0, 1, 2 and 4 only: column 3 shares column 2's red and column 7
  column 6's, and columns 5..7 have no cosine term (asserted below).
Colours are composed per pixel from (R, G, B) by a fixed RGB -> YUV matrix in the hardware convention (U = Cr,
V = Cb), luma pre-scaled 16/13 for the shell's 13/16 output scale and capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

P_LINES = 530.0            # vertical period at the still (virtual lines); cycles per frame = 1080 / 530 = 2.04
TILT0 = 25.0               # the still sits at +1 of the tilt LFO: left columns -25 lines, right +25
# per column: yc, psi, C0, s, w, wr, khi shift (13/32 or 13/64), gmax(1 or 13/16), a, b, floor
COLS = [
    # yc   psi    C0     s     w    wr   khi   gm   a    b   fl
    (503,   0, -0.17, 0.825, 0.09, 0.05, 5, 1,  64,   8, 64),
    (502,   0,  0.18, 0.525, 0.10, 0.05, 5, 1, 104,  16,  0),
    (541, -28, -1.00, 1.200, 0.11, 0.14, 5, 0, 308, 222, 64),
    (492,  46,  1.00, 0.000, 0.25, 0.14, 6, 0, 308, 222, 64),
    (537,   0,  0.24, 0.100, 0.51, 0.14, 5, 1, 112,  24,  0),
    (533,   0,  0.04, 0.100, 0.67, 0.05, 5, 1,  62,   0,  0),
    (529,   0, -0.09, 0.050, 0.74, 0.05, 5, 1,  16,   0,  0),
    (528,   0, -0.05, 0.000, 0.79, 0.05, 5, 1,  16,   0,  0),
]
SGN = [-1, -1, -1, 1, 1, 1, 1, 1]
WV = 0.08                  # vertical width term (outer rows wider), faded per column by (8 - k) / 8
YC_R = 530.0

def q11(v):
    return int(round(v * 2048))

def cos_rom():
    return [int(round(2047 * math.cos(2 * math.pi * i / 1024.0))) for i in range(1024)]

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def tables():
    t = {}
    t['YC'] = [int(round(c[0] - TILT0 * s)) for c, s in zip(COLS, SGN)]
    t['SGN'] = SGN
    t['PSI'] = [c[1] for c in COLS]
    t['C0'] = [q11(c[2]) for c in COLS]
    t['S'] = [q11(c[3]) for c in COLS]
    t['W'] = [q11(c[4]) for c in COLS]
    t['WR'] = [q11(c[5]) for c in COLS]
    t['YCF'] = [y + c[1] for y, c in zip(t['YC'], COLS)]
    t['WE'] = [q11(c[4] + c[5]) for c in COLS]
    t['KHI'] = [c[6] for c in COLS]
    t['GM'] = [c[7] for c in COLS]
    t['A'] = [c[8] for c in COLS]
    t['B'] = [c[9] for c in COLS]
    t['FL'] = [c[10] for c in COLS]
    return t

def check_red_groups():
    red = [(c[8], c[9], c[10]) for c in COLS]
    assert red[2] == red[3], "RTL shares column 2's red with column 3"
    assert red[6] == red[7], "RTL shares column 6's red with column 7"
    assert all(r[1] == 0 and r[2] == 0 for r in red[5:]), "columns 5..7: no cosine term, no floor (per-field red)"
    assert COLS[0][1] == 0 and COLS[7][1] == 0, "the beam's first column (0, or 7 under Flip) must have no tilt"
    assert [c[1] for c in COLS][4:] == [0, 0, 0, 0] and COLS[0][1] == COLS[1][1] == 0, "only columns 2 and 3 tilt"

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

def int_table(name, vals):
    return "    constant %s : t_tg_i8 := (%s);" % (name, ", ".join(str(int(v)) for v in vals))

def main():
    check_red_groups()
    t = tables()
    parts = ["-- Generated by tongues.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tongues_pkg is",
             "    type t_tg_i8 is array (0 to 7) of integer;",
             vhd_table("C_TG_COS", "signed", 1024, 12, cos_rom(), signed=True),
             vhd_table("C_TG_EXP2", "unsigned", 256, 16, exp2_rom())]
    for k in ('YC', 'YCF', 'SGN', 'PSI', 'C0', 'S', 'W', 'WR', 'WE', 'KHI', 'GM', 'A', 'B', 'FL'):
        parts.append(int_table("C_TG_" + k, t[k]))
    parts += ["end package tongues_pkg;", ""]
    with open(os.path.join(HERE, "tongues_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
