#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BATWING v0.1.0 - build hook.  Writes batwing_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; imodel.py imports the same tables).

Fitted to the clip WIN_20230927_23_58_07 at 2.5 s (see .heirloom_work/rb_batwing/notes.md):

  xp  = distance from the tile seam, 0 .. 511 units (1 unit = 0.9297 virtual px at the photo's 952 px tile)
  sA  = TA(xp + waist(y) * max(0, 1 - xp / 184)) - 64             the wing sides (a sqrt-like ramp off the seam)
        TA(w) = 256 (0.0282 (0.9297 w)^0.6 - 0.165) + 64
  sB  = (795.6 - Dv(phi) (1 + 0.00154 xpb) - 0.226 xp) / 419.6     the bowtie's top / bottom (rolling vertical wave)
        Dv = the vertical oscillator's waveform (measured on the photo's top path), xpb = xp + RND(511 - xp)
        (the tile-centre fold rounded, radius 129 units)
  s   = min(sA, sB) -> palette; plateau key: xp > 202 and (Dv - 155) < 0.372 (511 - xp)

Package contents:
  C_BW_TA     512 x 9   TA(w) + 64 bias (s in Q8: 256 = one palette span, magenta edge at 0)
  C_BW_DV    1024 x 10  Dv(phi): phase 0.5 = the oscillator's peak row; virtual lines at the photo's 1117-line period
  C_BW_RND    512 x 8   u + r - sqrt(u^2 + r^2), r = 129 units
  C_BW_PAL   1024 x 24  {Y8, U8 (bit 0 = shade flag), V8}: index = colours * 512 + palette index
                        palette index: s < 1.0 -> s * 256 + 128 (clamped at 0); s >= 1.0 -> 384 + ((s - 1) * 256 mod 128)
  C_BW_EXP2   256 x 16  round(32768 * 2^(i/256))
Colours in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's 13/16
output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model
UNIT = 476.0 / 512.0                 # virtual px per xp unit
KA, PA, A0 = 0.0282, 0.6, 0.165
PER = 1117.0
KN_PHI = [-558.5, -520, -480, -397, -365, -359, -327, -309, -281, -261, -249, -235, -225, -215, -204, -193, -185, -177,
          -170, -160, -100, 0, 100, 300, 470, 520, 558.5]
KN_DV = [558.5, 548, 525, 397, 370, 366, 349, 345, 332, 305, 276, 246, 231, 218, 204.5, 192.5, 182.7, 175.4,
         170.5, 155, 98, 0, 100, 300, 470, 535, 558.5]
RF = 129.0                           # fold rounding radius (xp units)

def interp(x, xs, ys):
    if x <= xs[0]: return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]

def ta_rom():
    out = []
    for w in range(512):
        v = 256.0 * (KA * (w * UNIT) ** PA - A0) + 64
        out.append(max(0, min(511, int(round(v)))))
    return out

def dv_rom():
    return [max(0, min(1023, int(round(interp((i / 1024.0 - 0.5) * PER, KN_PHI, KN_DV))))) for i in range(1024)]

def rnd_rom():
    return [max(0, min(255, int(round(u + RF - math.sqrt(u * u + RF * RF))))) for u in range(512)]

# ------------------------------------------------------------------ colours
def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876.0, 512 + cr * 896.0, 512 + cb * 896.0)

def prescale8(yuv):
    """10-bit hardware YUV -> 8-bit words (luma pre-scaled 16/13, capped 940)"""
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = min(940, max(64, y2))
    return (int(round(y2 / 4.0)) & 255, max(0, min(255, int(round(u / 4.0)))), max(0, min(255, int(round(v / 4.0)))))

# the photo's colours (k-means of the still) and the alternative "Meadow" set (the green / yellow alt clip)
MAG = (252, 0, 255); VIO = (99, 8, 243); YEL = (215, 223, 27); GRN = (17, 54, 7); NAV = (59, 25, 109)
DRK = (44, 31, 59); LIL = (178, 70, 249)
RAMP_END = (90, 8, 245)
# (every run >= 0.03 of the span: >= 3 rows at the steepest default gradient, so no band thins to a single row)
RUNS = [(0.20, 'GRN'), (0.27, 'VIO'), (0.30, 'YEL'), (0.335, 'GRN'), (0.38, 'YEL'),
        (0.50, 'VIO'), (0.565, 'YEL'), (0.60, 'GRN'), (0.63, 'YEL'), (0.685, 'VIO'), (0.74, 'GRN'), (0.795, 'VIO'),
        (0.845, 'GRN'), (0.88, 'VIO'), (0.91, 'GRN'), (0.94, 'YEL'), (0.97, 'VIO'), (1.0, 'DRK')]
SETS = [
    dict(MAG=MAG, RAMP_END=RAMP_END, VIO=VIO, YEL=YEL, GRN=GRN, NAV=NAV, DRK=DRK, LIL=LIL, SHD=(62, 16, 120)),
    dict(MAG=(232, 240, 4), RAMP_END=(150, 200, 40), VIO=(5, 108, 0), YEL=(240, 246, 70), GRN=(8, 57, 3),
         NAV=(81, 58, 141), DRK=(53, 43, 85), LIL=(124, 98, 176), SHD=(3, 62, 0)),
]

def step_colour(s, C):
    """(rgb, shade flag) of the palette at s (the photo's band sequence, from the bottom path at the tile centre)"""
    if s < 0: return C['MAG'], 0
    if s < 0.11:
        t = s / 0.11
        return tuple(C['MAG'][j] + t * (C['RAMP_END'][j] - C['MAG'][j]) for j in range(3)), 0
    prev = 0.11
    for end, name in RUNS:
        if s < end: return C[name], 1 if name == 'VIO' else 0
        prev = end
    return C['DRK'], 0

def pal_rom():
    out = [0] * 1024
    for p, C in enumerate(SETS):
        for i in range(512):
            if i < 384: s = (i - 128) / 256.0
            else: s = 0.5 + 0.5 * (i - 384) / 128.0          # s >= 1: the upper bands repeat (inside an inverted key)
            acc = [0.0, 0.0, 0.0]; fl = 0
            for d in (-0.012, -0.006, 0.0, 0.006, 0.012):     # soft band edges (~2 px at the photo's gradient)
                c, f = step_colour(s + d, C)
                for j in range(3): acc[j] += c[j] / 5.0
                if d == 0.0: fl = f
            y, u, v = prescale8(rgb_to_hw10(acc))
            u = (u & 0xFE) | fl
            out[p * 512 + i] = (y << 16) | (u << 8) | v
    return out

def lilac_words():
    return [prescale8(rgb_to_hw10(C['LIL'])) for C in SETS]

def shade_words():
    """the violet bands where the wing-side term wins (the photo's navy / dark-purple sides)"""
    return [prescale8(rgb_to_hw10(C['SHD'])) for C in SETS]

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        items.append('x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width))
    per = 16 if width % 4 == 0 else 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    lw = lilac_words(); sw = shade_words()
    parts = ["-- Generated by batwing.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package batwing_pkg is",
             vhd_table("C_BW_TA", "unsigned", 512, 9, ta_rom()),
             vhd_table("C_BW_DV", "unsigned", 1024, 10, dv_rom()),
             vhd_table("C_BW_RND", "unsigned", 512, 8, rnd_rom()),
             vhd_table("C_BW_PAL", "std_logic_vector", 1024, 24, pal_rom()),
             vhd_table("C_BW_EXP2", "unsigned", 256, 16, exp2_rom()),
             "    constant C_BW_LIL0 : std_logic_vector(23 downto 0) := x\"%02X%02X%02X\";" % lw[0],
             "    constant C_BW_LIL1 : std_logic_vector(23 downto 0) := x\"%02X%02X%02X\";" % lw[1],
             "    constant C_BW_SHD0 : std_logic_vector(23 downto 0) := x\"%02X%02X%02X\";" % sw[0],
             "    constant C_BW_SHD1 : std_logic_vector(23 downto 0) := x\"%02X%02X%02X\";" % sw[1],
             "end package batwing_pkg;", ""]
    with open(os.path.join(HERE, "batwing_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
