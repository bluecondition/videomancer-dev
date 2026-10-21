#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
RICOCHET v0.5.0 - build hook.  Writes ricochet_pkg.vhd next to this file.

The look is fitted to the photo (WIN_20230515_23_08_59): one scalar field

    phiA = knee(F0 + F1 x - F2 x^2) + Y (gA0 - gA1 x)     (horizontal ramp + vertical triangle)
    phiB = gB0 - gB1 x + c (AMP - Y)                      (the seam field: the triangle inverted, 4x steeper)
    phi  = min(phiA, phiB)

coloured by a 1-D colouriser along the photo's colour path (red -> orange -> salmon -> pink -> lavender
-> periwinkle -> cyan).  All constants below are the fitted values in photo units ("u", 1 u = 1 photo
line of the vertical triangle); the hardware works in hw units, 4096 hw = the triangle amplitude
(261.5 u), so 1 u = 15.66348 hw.

Package contents:
  C_DDA_*     per timing class (0 = 1080, 1 = 720p, 2 = SD 720) and mode (0 Right, 1 Left, 2 Twin):
              F ramp start (Q12) / first difference (Q20) / second difference (Q20), the Y-gain start
              (Q12) and slope (Q20 per pixel), the seam-field start (hw) and slope (Q12 per pixel).
  C_RIC_CORR  512 x 12 signed: band chirp kap (w^2 - w) in Q14 cycles per band, w = (i + .5) / 512.
  C_RIC_COS   1024 x 9 signed: full-wave cosine, round(a0 * 1024 * cos(2 pi (i + .5) / 1024)).
  C_RIC_KSQ   128 x 8: the ramp knee's square, (q * q) >> 6 (LUT logic).
  C_PAL       256 x 32: {Y, U, V, 00} along the colour path, s = i / 255.  Hardware convention
              (U register = Cr, V register = Cb); Y pre-scaled by 16/13 so the shell's output luma
              scale (13/16) lands on the photo's levels.  The Azure palette is the same entries with
              the chroma mirrored about 512 (done in the RTL).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (photo units)
AMP = 261.5                       # triangle amplitude, u (half the 523-line period of the photo)
HU = 4096.0 / AMP                 # hw units per u
F0, F1, F2 = -56.302, 0.41135, 4.32162e-05
FF = -35.069                      # knee floor (soft max), u
gA0, gA1 = 1.34713, 0.000373324
gB0, gB1 = 332.152, 0.102307
C_DEF = 3.97745
P0_DEF, D_DEF = 102.886, 397.339
NB_DEF, KAP = 8.33719, 0.279687
B0, B1, A0 = 0.31849, 0.599033, 0.19754
S0, KC = 0.168145, 0.260977
SF, KCUT, CS, F1C = 0.204818, 0.0287787, 2.60033, 0.00046378
KR, DR, DFL = 0.00501225, 27.0075, 11.9462
PH0 = 0.5 - 256.0 / 523.0         # vertical phase at the top line
YFB, MFB = 1032.0, 1.2            # bottom flyback: from this virtual line the triangle keeps going, 1.2x

PATH = [(224, 14, 3), (252, 14, 2), (255, 53, 1), (255, 75, 1), (254, 91, 5), (252, 101, 29), (250, 111, 72),
        (244, 120, 109), (232, 130, 145), (209, 138, 176), (182, 147, 204), (150, 157, 224), (111, 168, 231),
        (70, 181, 234), (24, 196, 237)]

XS = [1.0, 1.5, 1920.0 / 720.0]   # virtual x per real pixel: 1080, 720p, SD (720 wide)


def path_l():
    seg = [math.dist(PATH[i], PATH[i + 1]) for i in range(len(PATH) - 1)]
    tot = sum(seg); acc = [0.0]
    for s in seg:
        acc.append(acc[-1] + s / tot)
    return acc


def path_rgb(s):
    L = path_l()
    s = min(max(s, 0.0), 1.0)
    for i in range(len(L) - 1):
        if s <= L[i + 1] or i == len(L) - 2:
            t = (s - L[i]) / (L[i + 1] - L[i])
            return tuple(PATH[i][k] + t * (PATH[i + 1][k] - PATH[i][k]) for k in range(3))


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772; cr = (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def dda_table():
    """[class][mode] -> dict of integer DDA constants."""
    out = []
    for xs in XS:
        row = []
        for mode in range(3):
            st = 2.0 * xs if mode == 2 else xs
            if mode == 1:          # Left: x' = 1920 - n xs
                x0, sg = 1920.0, -1.0
            else:
                x0, sg = 0.0, 1.0
            Fx = lambda x: F0 + F1 * x - F2 * x * x
            fst = Fx(x0)
            fd0 = Fx(x0 + sg * st) - Fx(x0)
            dd = -2.0 * F2 * st * st
            ga_st = gA0 - gA1 * x0
            ga_in = -gA1 * sg * st
            gb_st = gB0 - gB1 * x0
            gb_in = -gB1 * sg * st
            row.append(dict(
                fst=round(fst * HU * 4096), fd0=round(fd0 * HU * 2 ** 20), dd=round(dd * HU * 2 ** 20),
                gast=round(ga_st * 4096), gain=round(ga_in * 2 ** 20),
                gbst=round(gb_st * HU), gbin=round(gb_in * HU * 4096)))
        out.append(row)
    return out


def corr_rom():
    return [round(KAP * (w * w - w) * 2 ** 14) for w in [(i + 0.5) / 512 for i in range(512)]]


def cos_rom():
    return [round(A0 * 1024 * math.cos(2 * math.pi * (i + 0.5) / 1024)) for i in range(1024)]


def pal_rom():
    out = []
    for i in range(256):
        y, u, v = rgb_to_hw10(path_rgb(i / 255.0))
        y = round(64 + (y - 64) * 16 / 13)
        out.append((min(max(y, 64), 768), min(max(u, 64), 960), min(max(v, 64), 960)))
    return out


# integer constants shared by the RTL and the integer model
K = dict(
    HU=HU,
    FF=round(FF * HU), KW=256,
    PH_TOP=round(PH0 * 2 ** 20),
    YFB_Q8=round(YFB * 256),
    GB0=round(gB0 * HU),
    WALL0=round(S0 * 1024), KC=KC * 1024 / HU,                # wall = 172 + d * 17.06
    SF=round(SF * 1024), KCUT=KCUT * 1024 / HU, F1C=F1C * 1024 / HU, SFMIN=143,
    FL0=round(0.06 * 1024), KR=KR * 1024 / HU, DR=round(DR * HU), DFL=round(DFL * HU),
    BASE0=round(B0 * 1024), KB=B1 * 1024 / 4096,             # base = 326 + wq * 0.1498
    WQK=4096 / (D_DEF * HU),                                   # wq = d * 0.6581 (d / Ddef in Q12)
    CS=round(CS * HU), SMIN=61,
)


def write_pkg():
    dd = dda_table()
    L = []
    L.append("-- Generated by ricochet.py -- DO NOT EDIT.")
    L.append("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n")
    L.append("package ricochet_pkg is")
    L.append("    type t_i9 is array (0 to 8) of integer;")
    for key in ('fst', 'fd0', 'dd', 'gast', 'gain', 'gbst', 'gbin'):
        vals = [dd[c][m][key] for c in range(3) for m in range(3)]
        L.append("    constant C_DDA_%s : t_i9 := (%s);" % (key.upper(), ", ".join(str(v) for v in vals)))
    L.append("    type t_corr is array (0 to 511) of signed(11 downto 0);")
    L.append("    constant C_RIC_CORR : t_corr := (")
    cr = corr_rom()
    L.append(",\n".join("        " + ", ".join("to_signed(%d, 12)" % v for v in cr[i:i + 8]) for i in range(0, 512, 8)) + ");")
    L.append("    type t_cos is array (0 to 1023) of signed(8 downto 0);")
    L.append("    constant C_RIC_COS : t_cos := (")
    co = cos_rom()
    L.append(",\n".join("        " + ", ".join("to_signed(%d, 9)" % v for v in co[i:i + 8]) for i in range(0, 1024, 8)) + ");")
    L.append("    type t_ksq is array (0 to 127) of unsigned(7 downto 0);")
    L.append("    constant C_RIC_KSQ : t_ksq := (")
    L.append(",\n".join("        " + ", ".join("to_unsigned(%d, 8)" % ((q * q) >> 6) for q in range(i, i + 16)) for i in range(0, 128, 16)) + ");")
    L.append("    type t_pal is array (0 to 255) of std_logic_vector(31 downto 0);")
    L.append("    constant C_RIC_PAL : t_pal := (")
    pr = pal_rom()
    L.append(",\n".join("        " + ", ".join('x"%08X"' % ((y << 22) | (u << 12) | (v << 2)) for (y, u, v) in pr[i:i + 4])
                        for i in range(0, 256, 4)) + ");")
    L.append("end package ricochet_pkg;")
    with open(os.path.join(HERE, "ricochet_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    write_pkg()
    print("ricochet_pkg.vhd written")
