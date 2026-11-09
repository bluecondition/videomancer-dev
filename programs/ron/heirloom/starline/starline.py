#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
STARLINE v0.1.0 - build hook.  Writes starline_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  imodel.py imports the same tables.

Fitted to the clip WIN_20231218_21_31_11 at 3.0 s (see .heirloom_work/rb_starline/notes.md):
  T        = |x - 1071| (virtual px, rounded within 64 px of the centre)
  star     lines where log2(B(y)) - log2(T - c(y)) = i * 0.24, i = 0 .. N-1
           B = b(y) = 395 + q (y - 605)^2 above the kink row 775 (q 0.0075 above the apex, 0.0118 below),
           B = b_k, c = -5 (y - 775) below it (the V, parallel lines)
  band     lines where log2(a(y)) - log2(T) = j * 0.45, a = 8.7 (374 - y): rays from the dip
  colour   LUT(u, k): u = hue ramp (x - x_v) / 1920, k = intensity (wash level + lines / glow)

Package contents:
  C_SL_LOG2  256 x 8   log2(1 + m/256) * 256, m = 8-bit mantissa below the leading one
  C_SL_EXP2  256 x 16  2^(f/256) * 32768 (15-bit mantissa, used by the pixel path and by the sequencer)
  C_SL_SOFT  256 x 8   (dx * dx) >> 9: the rounded centre of |x - 1071|
  C_SL_BUMP  256 x 16  {tube profile, glow profile} by half the px distance (h)
  C_SL_CY / C_SL_CU / C_SL_CV  2048 x 8  the colouriser (index u7 & k4), 10-bit value >> 2,
           hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 (shell output
           13/16), capped at 940
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------------
# colouriser (RGB 0..255), knots written as still x positions (reset line at x = 312, period 1920 virtual px)
PH = 2200.0
XV = 312.0


def U(x):
    return ((x - XV) / PH) % 1.0


def K(lst):
    return sorted([(U(x),) + tuple(c) for (x, *c) in lst])


# a full-intensity tube, as seen over the wash
LINE = K([(312, 240, 112, 236), (545, 234, 160, 234), (780, 232, 200, 230), (895, 226, 198, 160), (990, 200, 196, 80),
          (1080, 168, 192, 22), (1180, 110, 212, 0), (1260, 72, 219, 0), (1340, 150, 236, 0), (1440, 180, 240, 10),
          (1560, 160, 222, 0), (1660, 205, 220, 60), (1760, 226, 202, 140), (1830, 205, 70, 200), (1900, 205, 50, 215),
          (140, 236, 80, 242), (240, 238, 96, 240)])
# wash, middle level: magenta zone, violet before the reset, black over the green part of the ramp
WASH_MID = K([(312, 236, 0, 254), (560, 240, 8, 250), (740, 225, 20, 235), (840, 150, 25, 120), (920, 60, 15, 20),
              (1000, 10, 6, 0), (1100, 0, 3, 0), (1900, 0, 3, 0), (30, 60, 0, 120), (120, 130, 0, 205), (220, 190, 0, 245)])
# wash, top level: magenta-pink, a green glow over the green zone, black after it
WASH_TOP = K([(312, 247, 22, 249), (480, 240, 60, 242), (620, 232, 95, 232), (700, 200, 110, 160), (780, 110, 90, 30),
              (880, 40, 100, 0), (1050, 5, 100, 0), (1200, 0, 70, 0), (1320, 0, 12, 0), (1880, 0, 4, 0), (1910, 40, 0, 100),
              (100, 150, 10, 200), (190, 230, 15, 245)])
# wash, bottom: dull magenta / maroon / olive
WASH_BOT = K([(312, 155, 52, 105), (480, 125, 58, 50), (600, 60, 40, 5), (700, 20, 20, 0), (840, 0, 4, 0), (1880, 0, 3, 0),
              (20, 40, 22, 0), (80, 72, 38, 0), (180, 115, 38, 40), (270, 150, 45, 100)])
KW = (1 / 15.0, 3 / 15.0, 5 / 15.0)   # wash levels: bottom, middle, top (k rows 1, 3, 5 of 0..15)
NU, NK = 128, 16


def interp_periodic(knots, u):
    u = u % 1.0
    ext = [(k[0] - 1,) + k[1:] for k in knots] + list(knots) + [(k[0] + 1,) + k[1:] for k in knots]
    for i in range(len(ext) - 1):
        a, b = ext[i], ext[i + 1]
        if a[0] <= u <= b[0]:
            t = 0.0 if b[0] == a[0] else (u - a[0]) / (b[0] - a[0])
            return tuple(a[c] + (b[c] - a[c]) * t for c in (1, 2, 3))
    return knots[0][1:]


def design():
    """lut[iu][ik] = (r, g, b) floats 0..255"""
    lut = []
    for iu in range(NU):
        u = (iu + 0.5) / NU
        Lc = [v / 255 for v in interp_periodic(LINE, u)]
        Wb = [v / 255 for v in interp_periodic(WASH_BOT, u)]
        Wm = [v / 255 for v in interp_periodic(WASH_MID, u)]
        Wt = [v / 255 for v in interp_periodic(WASH_TOP, u)]
        row = []
        for j in range(NK):
            k = j / (NK - 1)
            if k <= KW[0]:
                w = [c * (k / KW[0]) for c in Wb]
            elif k <= KW[1]:
                a = (k - KW[0]) / (KW[1] - KW[0]); w = [(1 - a) * p + a * q for p, q in zip(Wb, Wm)]
            elif k <= KW[2]:
                a = (k - KW[1]) / (KW[2] - KW[1]); w = [(1 - a) * p + a * q for p, q in zip(Wm, Wt)]
            else:
                w = Wt
            li = min(1.0, max(0.0, (k - KW[2]) / (1 - KW[2])))
            row.append(tuple(min(1.0, max(0.0, wc * (1 - li) + lc * li ** 0.8)) * 255 for wc, lc in zip(w, Lc)))
        lut.append(row)
    return lut


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def colour_tables():
    """three lists of 2048 bytes (index iu * 16 + ik): Y, U (= Cr), V (= Cb), each 10-bit >> 2"""
    lut = design()
    ty, tu, tv = [], [], []
    for iu in range(NU):
        for ik in range(NK):
            y, u, v = rgb_to_hw10(lut[iu][ik])
            y = min(940, 64 + int(round((y - 64) * 16 / 13)))
            ty.append(max(16, min(255, y >> 2)))
            tu.append(max(1, min(255, (u + 2) >> 2)))
            tv.append(max(1, min(255, (v + 2) >> 2)))
    return ty, tu, tv


def log2_table():
    return [min(255, int(round(math.log2(1 + m / 256.0) * 256))) for m in range(256)]


def exp2_table():
    return [min(65535, int(round(2 ** (f / 256.0) * 32768))) for f in range(256)]


def soft_table():
    """rounded |x - centre| near the centre: (dx * dx) >> 9 for dx < 256 (T = 128 + this)"""
    return [(d * d) >> 9 for d in range(256)]


def bump_table():
    """{tube (8), glow (8)} by j = min(255, distance_h >> 1): tube exp(-((2j+1)/17)^2), glow exp(-((4(j>>1)+2)/52)^2)"""
    out = []
    for j in range(256):
        jl = min(j, 63); jg = min(j >> 1, 63)
        bl = int(round(255 * math.exp(-((2 * jl + 1) / 17.0) ** 2)))
        bg = int(round(255 * math.exp(-((4 * jg + 2) / 52.0) ** 2)))
        out.append((bl << 8) | bg)
    return out


def vhdl_array(name, typ, width, vals, per=16):
    hexw = (width + 3) // 4
    lines = []
    for i in range(0, len(vals), per):
        lines.append('        ' + ', '.join('x"%0*x"' % (hexw, v) for v in vals[i:i + per]))
    return '    constant %s : %s := (\n%s\n    );\n' % (name, typ, ',\n'.join(lines))


def main():
    ty, tu, tv = colour_tables()
    out = []
    out.append('-- Generated by starline.py - do not edit.\n')
    out.append('library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n')
    out.append('package starline_pkg is\n')
    out.append('    type t_sl_8x256 is array (0 to 255) of std_logic_vector(7 downto 0);\n')
    out.append('    type t_sl_16x256 is array (0 to 255) of std_logic_vector(15 downto 0);\n')
    out.append('    type t_sl_8x2048 is array (0 to 2047) of std_logic_vector(7 downto 0);\n')
    out.append(vhdl_array('C_SL_LOG2', 't_sl_8x256', 8, log2_table()))
    out.append(vhdl_array('C_SL_EXP2', 't_sl_16x256', 16, exp2_table()))
    out.append(vhdl_array('C_SL_SOFT', 't_sl_8x256', 8, soft_table()))
    out.append(vhdl_array('C_SL_BUMP', 't_sl_16x256', 16, bump_table()))
    out.append(vhdl_array('C_SL_CY', 't_sl_8x2048', 8, ty))
    out.append(vhdl_array('C_SL_CU', 't_sl_8x2048', 8, tu))
    out.append(vhdl_array('C_SL_CV', 't_sl_8x2048', 8, tv))
    out.append('end package starline_pkg;\n')
    with open(os.path.join(HERE, 'starline_pkg.vhd'), 'w') as f:
        f.write(''.join(out))


if __name__ == '__main__':
    main()
