#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SERPENT v0.1.0 - build hook.  Writes serpent_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  imodel.py (.heirloom_work/rb_serpent) imports the same constants and tables.

Fitted to the clip still WIN_20230529_01_27_21 at 0.5 s (see .heirloom_work/rb_serpent/notes.md):

  u(x, y)  = phi(y) + m(y) (x - 1000) / 1000           the S colour field: one ramp per line (a horizontal ramp
                                                       whose offset and gain are vertical CVs), fitted per row
                                                       with coherence 0.93-0.98
  phi(y)   = 0.8662 + 0.2715 cos(th - 0.1041) + 5.480 E      E = max(0, y - 630) / 1080: the bottom fan
  m(y)     = 0.9097 + 6.360 E                                (u per 1000 px)
  th(y)    = 2 pi (y - 110) / 790 - scroll              the vertical oscillator (scrolls down 160 lines/s)
  colour   = R 100 X + 90 Y - 20 frac(u), G 10 + 213 frac(u), B 297 - 136 X - 55 Y - 25 frac(u)
             (X, Y = px / 1000; G is a sawtooth of u, R / B a diagonal ramp: rms 5 / 31 / 17 levels)
  body     = between xL(y) = 726.5 + 74.4 cos(th - 2.726) - 1477 E and xR(y) = 1396.6 + 194.0 cos(th - 3.080)
             (blended to the kink line x = 1390 in the fan); a soft window T, keyed against a sawtooth column
             oscillator (22.48 per 1920 px, fast 1/8 fall): fins, teeth and the body's vertical right edges
  holes    = lens slots in alternate columns where |cos(th + 0.119 + 6.8 E)| > 0.62
  tails    = fins and red bands along the integer contours of u right of the body (the tip, and in the fan
             a second, gentler field beyond x = 1390: 0.594 u(1390) + 0.41 + 0.45 (x - 1390) / 1000)
  halo     = a red layer (246, 9, 1) around the body and the tails, with black stripes, showing through holes

Package contents:
  C_SP_COS   1024 x 16  round(32767 cos(2 pi i / 1024)), signed
  C_SP_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_SP_GR, C_SP_GG, C_SP_GB   256 x 32   per-gun YCrCb contributions of the level (R, G, B guns):
                        {Y' * 876 * 16/13 (10 bits), Cr * 896 (11 bits signed), Cb * 896 (11 bits signed)}
                        (luma pre-scaled 16/13 for the shell's 13/16 output scale; the RTL caps it at 940)
Colours are summed in the hardware convention (U register = Cr, V register = Cb).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model, fixed point
Q24 = 1 << 24
PHI0_Q24 = 14531920          # 0.86617 u
CPHI_Q24 = 4554836           # 0.27149 u (cos amplitude of phi)
PSI_PHI = 277969             # phase of phi's cosine, Q24 periods
FPHI_Q28 = 85136             # 5.48044 u per E, per Q4 line: 5.48044 / 17280 * 2^28
MH0_Q28 = 122098             # m0 / 2000 u per h (0.90970 u per 1000 px)
MH1_Q36 = 12647              # Fm / 1080 / 2000 / 16 * 2^36 (u per h per Q4 line)
XL0_Q8 = 371979              # 726.52 px = 1453.04 h, Q8
CL_Q8 = 38107                # 74.43 px (cos amplitude of xL), h Q8
PSI_L = 7279932
FL_Q16 = -11201              # -1476.76 px per E: h Q16 per Q4 line
XR0_Q8 = 715059              # 1396.60 px
CR_Q8 = 99315                # 193.97 px
PSI_R = 8223516
XK_H = 2780                  # the kink line, 1390 px
YF4 = 10080                  # fan start, line 630 (Q4)
TQ0 = 21237                  # 1080 / 790 turns per 1080 lines: Q24 periods per line
Y0_LINES = 110               # the oscillator's peak (the tongue) at line 110
DH_Q24 = 318553              # hole rows: + 15 / 790 period
HW_Q24 = 1051                # hole rows: + 6.8 rad per E, per Q4 line (Q24 periods)
BL_Q24 = 16182               # xR blend: f E / 0.06, per Q4 line
HV0 = 20316                  # 0.62 * 32767
HA_Q12 = 3277                # hole amplitude 0.8
TC_Q24 = 6878658             # tail field: + 0.41
TST_Q28 = 60398              # tail field slope 0.45 u per 1000 px, per h
CPH_Q24 = 98225              # 22.482 columns per 1920 px: Q24 column phase per h
COLX0_H = 37                 # column phase origin 18.6 px


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def cos_rom():
    return [int(round(32767 * math.cos(2 * math.pi * i / 1024.0))) for i in range(1024)]


def gun_rom(g):
    """per-gun contributions: Y' (pre-scaled 16/13) and Cr / Cb, 10-bit units"""
    wy = (0.299, 0.587, 0.114)
    wcr = (0.5, -0.587 / 1.402, -0.114 / 1.402)
    wcb = (-0.299 / 1.772, -0.587 / 1.772, 0.5)
    out = []
    if True:
        for v in range(256):
            y = int(round(wy[g] * v / 255.0 * 876 * 16 / 13))
            cr = int(round(wcr[g] * v / 255.0 * 896)) & 0x7FF
            cb = int(round(wcb[g] * v / 255.0 * 896)) & 0x7FF
            out.append((y << 22) | (cr << 11) | cb)
    return out


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


def main():
    parts = ["-- Generated by serpent.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package serpent_pkg is",
             vhd_table("C_SP_COS", "signed", 1024, 16, cos_rom(), signed=True),
             vhd_table("C_SP_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_SP_GR", "std_logic_vector", 256, 32, gun_rom(0)),
             vhd_table("C_SP_GG", "std_logic_vector", 256, 32, gun_rom(1)),
             vhd_table("C_SP_GB", "std_logic_vector", 256, 32, gun_rom(2)),
             "end package serpent_pkg;", ""]
    with open(os.path.join(HERE, "serpent_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
