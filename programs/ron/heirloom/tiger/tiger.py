#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TIGER v0.1.1 - build hook.  Writes tiger_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  imodel.py (.heirloom_work/rb_tiger/) imports the same tables.

Fitted to the clip WIN_20231208_23_03_08 at 0.5 s (see .heirloom_work/rb_tiger/notes.md).  The patch:

  theta  = (x + xb) * f(y) + c(y)          a line-locked H triangle oscillator, its phase reset at the line
                                           start (xb = 185 virtual px left of the picture: the fan pivot)
  f(y)   = (1 + aF (1 - cos(V - d)) / 2) / P0    FM from a slow V sine: 1/650 per px on the fold row,
                                           ~2.2x faster at the top (the fan)
  V      = 2 pi (y0 - y) / Pv              the V oscillator (Pv ~ 3000 lines, rolling up in the clip)
  w      = A sin V                         the split: ADDED to the red colouriser, SUBTRACTED from the blue
  T      = triangle(theta)                 symmetric (the fit's rise fraction 0.479 ~ 1/2)
  uR     = frac(T + w),  uB = frac(T - w)  two wrapping colourisers (sum / difference mixer)
  R, B   = camera transfer of uR, uB; G = base + a little of R and B; red crosstalk spikes at the blue wraps

On the fold row (w = 0) R = B = T: pink peaks, dark troughs.  Above it the red channel wraps around the
triangle peaks and the blue around the troughs (blue wedges); below, the reverse (orange wedges).

Package contents:
  C_TG_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_TG_SIN   256 x 16   round(32767 * sin(2 pi i / 256)), signed (the line engine interpolates)
  C_TG_FAN   256 x 16   round(65535 * |sin(pi i / 256)|^2.371): the fan's shape over one V cycle
  C_TG_CR    512 x 32   red-colouriser term {Y 10, U 11 signed, V 11 signed} by pal * 256 + uR (8 bits)
  C_TG_CB    512 x 32   blue-colouriser term, same packing, by pal * 256 + uB
  C_TG_BASE  2 x 32     {Y, U, V} base (the green floor) per palette, 10-bit each in bits 31..22 / 21..12 / 11..2
Colours are in the hardware convention (U register = Cr, V register = Cb).  The ROM terms are linear pieces of
the BT.601 matrix, so base + CR + CB is exactly the colour of (R, G, B); luma is pre-scaled 16/13 for the shell's
13/16 output scale and the sum is capped at 940 in the RTL.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted geometry (virtual px / lines)
# fit5.py / fit6.py (MSE on R and B over the still): P0 638.8 px, xb 166 px, aF 1.90, Pv 2754 lines, fold row
# y0 880.6, fan centre 30.6 lines below the fold, fan shape |sin(V/2)|^2.371; with the triangle held symmetric
# (fit6): c -0.1841 cycles on the fold row, lean s1 -0.114 cycles per 1080 lines, split A 0.2742, B offset 0.0233.
FANQ = 2.371
K = dict(
    KB0=13132,      # H oscillator step per h (half virtual px), Q24 cycles: 2^24 / (2 * 638.8)
    XBH=332,        # fan pivot: 166 virtual px left of the picture, in h
    KV0=6092,       # V oscillator phase per virtual line, Q24: 2^24 / 2754
    VS0=5364517,    # V phase at line 0 (the fold row at 880.6), Q24
    DPH=186124,     # fan centre offset (30.6 lines), Q24 of the V phase
    AFL=973,        # fan depth aF = k2 * AFL >> 6 (Q12; 7784 = 1.90 at 512, 3.8 at 1023)
    APK=5400,       # split A = p12^2 * APK >> 20 (Q12; 1123 = 0.2742 at the default 467)
    P12D=467, K4D=690, K5D=512, K6D=128,
    HDK=83700,      # H drift per field = v^2 * HDK >> 16 (Q24 cycles; 0.0022 at the default roll)
    EGK=5857,       # etch gain = k6^2 * EGK >> 16 (Q24 per luma step; 0.021 cycles at full luma, K6 128); Osc 24x
    CC=13688531,    # H phase on the fold row: frac(-0.1841), Q24
    Y0L=881,        # the lean pivots about the fold row (virtual line 881)
    LEAN0=1771,     # lean on the clip: +0.114 cycles per 1080 lines -> Q24 per virtual line
    LEANK=7767,     # K5 Lean: (k5 - 512) * 7767 >> 7 = 60.7 Q24 per virtual line per step (+-2 cycles / frame)
    DB=95,          # blue colouriser offset, Q12 (0.0233)
    KW=31069,       # Wave: a 540-line V sine into the H phase (Q24 per virtual line)
    WD=1229,        # Wave depth 0.30 cycles, Q12
    WS0=0,
    WROLL=16730612, # Wave phase per field: -1.5 lines (mod 2^24)
)

# ------------------------------------------------------------------ colourisers (camera transfer, measured)
# Rd = 255 * clip((uR - loR) / (hiR - loR)) ^ gaR ; Bd likewise ; sp = red spike after each blue wrap ;
# G = g0 + gr Rd + gb Bd - dips at both wraps.  Palette 1 ("Jungle", the session's other look, clip
# WIN_20231208_22_55_57): a green floor instead of the dark gaps and yellow lines instead of red.
PALS = [
    dict(loR=0.186, hiR=0.925, gaR=1.405, loB=0.294, hiB=0.879, gaB=1.254, spw=0.254, spa=0.582, spg=0.0,
         g0=57.2, gr=0.117, gb=0.068, dw=0.03, da=32.0),
    dict(loR=0.24, hiR=0.90, gaR=1.3, loB=0.34, hiB=0.86, gaB=1.6, spw=0.16, spa=0.60, spg=0.75,
         g0=92.0, gr=0.10, gb=0.02, dw=0.03, da=55.0),
]

def clip01(v):
    return 0.0 if v < 0 else (1.0 if v > 1 else v)

def tf(u, lo, hi, g):
    return clip01((u - lo) / (hi - lo)) ** g

def red_term(u, p):
    """(dR, dG, dB) contributed by the red colouriser at phase u (0..1)."""
    rd = 255.0 * tf(u, p['loR'], p['hiR'], p['gaR'])
    dip = clip01(1 - min(u, 1 - u) / p['dw'])
    return (rd, p['gr'] * rd - p['da'] * dip, 0.0)

def blue_term(u, p):
    bd = 255.0 * tf(u, p['loB'], p['hiB'], p['gaB'])
    sp = 255.0 * clip01(1 - u / p['spw']) * p['spa']
    dip = clip01(1 - min(u, 1 - u) / p['dw'])
    return (sp, p['gb'] * bd - p['da'] * dip + sp * p['spg'], bd)

def rgb_lin(d):
    """linear part of the BT.601 hardware conversion, luma pre-scaled 16/13: (dY, dCr, dCb) in 10-bit codes"""
    r, g, b = [c / 255.0 for c in d]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    return (y * 876.0 * 16.0 / 13.0, cr * 896.0, cb * 896.0)

def base_yuv(p):
    """the base colour (G floor only), absolute 10-bit hardware codes, luma pre-scaled"""
    dy, dcr, dcb = rgb_lin((0.0, p['g0'], 0.0))
    return (64.0 + dy, 512.0 + dcr, 512.0 + dcb)

def pack(y, u, v):
    y = max(0, min(1023, int(round(y))))
    u = max(-1024, min(1023, int(round(u)))) & 0x7FF
    v = max(-1024, min(1023, int(round(v)))) & 0x7FF
    return (y << 22) | (u << 11) | v

def term_rom_signed_y(fn):
    """Y terms may be slightly negative (the G dips): store Y as a 10-bit two's complement too"""
    out = []
    for p in PALS:
        for i in range(256):
            dy, dcr, dcb = rgb_lin(fn((i + 0.5) / 256.0, p))
            y = max(-512, min(511, int(round(dy)))) & 0x3FF
            u = max(-1024, min(1023, int(round(dcr)))) & 0x7FF
            v = max(-1024, min(1023, int(round(dcb)))) & 0x7FF
            out.append((y << 22) | (u << 11) | v)
    return out

def base_rom():
    out = []
    for p in PALS:
        y, u, v = base_yuv(p)
        out.append((int(round(y)) << 22) | (int(round(u)) << 12) | (int(round(v)) << 2))
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def sin_rom():
    return [int(round(32767 * math.sin(2 * math.pi * i / 256.0))) for i in range(256)]

def fan_rom():
    return [int(round(65535 * abs(math.sin(math.pi * i / 256.0)) ** FANQ)) for i in range(256)]

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
    parts = ["-- Generated by tiger.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tiger_pkg is",
             "\n".join("    constant C_%s : integer := %d;" % (k, v) for k, v in K.items()),
             vhd_table("C_TG_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_TG_SIN", "signed", 256, 16, sin_rom(), signed=True),
             vhd_table("C_TG_FAN", "unsigned", 256, 16, fan_rom()),
             vhd_table("C_TG_CR", "std_logic_vector", 512, 32, term_rom_signed_y(red_term)),
             vhd_table("C_TG_CB", "std_logic_vector", 512, 32, term_rom_signed_y(blue_term)),
             vhd_table("C_TG_BASE", "std_logic_vector", 2, 32, base_rom()),
             "end package tiger_pkg;", ""]
    with open(os.path.join(HERE, "tiger_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
