#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PYRAMID v0.1.0 - build hook.  Writes pyramid_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the still WIN_20231124_00_45_44 @ 5.0 s (see .heirloom_work/rb_pyramid/notes.md):

  key    = |dx| < w(d): a curved-sided pyramid, w = 421 px * (d / 1000)^0.755 at d lines below the apex (728, 28)
  inside = three oscillators into R, G, B:
           G = a screen-fixed wavefolded stripe, period 322.75 px (C_PY_G: the measured waveform, 512 entries)
           B = 135 + 82 cos(2 pi (x - xb) / 700)                 (C_PY_B)
           R = clip(40 + 135 tri(0.396 (0.77 cos(2 pi x / 1333) + cos(2 pi y / 896)) + 0.204) - apex)
               (C_PY_RX = 98 cos, C_PY_RY = 127 cos)
  ground = L = 172 - min(0.14 A, 0.219 y - 0.043 |dx| + 84), A = |dx| - w; colour = a ramp of L (C_PY_BG*)

Package contents (all tables also read by imodel.py):
  C_PY_G    512 x 8    G stripe waveform over one period
  C_PY_B    512 x 8    135 + 82 cos(2 pi i / 512)
  C_PY_RX   512 x 8    round(98 cos(2 pi i / 512)), signed
  C_PY_RY   512 x 8    round(127 cos(2 pi i / 512)), signed
  C_PY_LOG  256 x 8    round(256 log2(1 + i / 256))
  C_PY_EXP  256 x 16   round(32768 * 2^(i / 256))
  C_PY_BGR/G/B 512 x 8 ground ramp: index = ground * 256 + L (ground 0 teal, 1 grey)
  C_PY_MR/MG/MB 256 x 32  RGB -> YUV contributions, {Y (11 bits), |Cr| (10), |Cb| (10), 0}, all x2:
           Y = 64 + (MR.y + MG.y + MB.y) / 2           (luma pre-scaled 16/13 for the shell's 13/16 output scale)
           U (Cr) = 512 + (MR.u - MG.u - MB.u) / 2
           V (Cb) = 512 + (MB.v - MR.v - MG.v) / 2
Colours are in the hardware convention (U register = Cr, V register = Cb).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

G512 = [111,114,116,119,121,123,126,128,131,133,136,139,142,144,146,149,152,154,157,160,163,165,167,170,172,175,177,180,183,185,187,189,191,193,195,197,199,201,202,204,205,207,208,209,210,211,211,212,212,212,212,212,212,211,211,210,209,208,207,206,205,203,202,200,198,196,194,192,190,188,185,183,180,178,175,173,171,168,166,163,161,158,156,155,154,152,151,150,149,148,148,148,148,148,148,148,148,149,149,149,149,148,148,147,146,145,144,144,143,142,140,139,138,137,137,136,136,135,134,134,134,134,135,135,135,136,136,137,137,138,139,140,140,140,141,141,141,142,143,143,144,144,145,145,146,145,145,145,144,143,143,142,142,140,139,137,135,133,130,129,127,125,122,120,118,116,113,112,111,109,107,105,103,102,101,100,99,99,98,97,97,97,97,97,97,98,98,99,99,100,101,103,104,106,108,110,112,113,115,118,120,123,126,128,131,133,135,138,141,144,147,150,153,155,157,160,163,165,168,170,173,174,176,178,180,182,184,186,188,189,189,190,192,193,194,195,195,196,197,197,198,198,198,198,198,197,197,196,195,195,194,193,192,190,189,187,185,183,182,180,179,176,174,171,168,165,163,161,159,156,153,149,146,143,140,137,134,131,128,125,122,119,116,112,108,104,100,96,93,89,86,82,78,73,69,65,62,60,57,55,52,48,45,43,40,39,37,36,35,32,30,28,27,26,25,24,23,21,19,18,16,15,14,13,12,11,10,9,9,8,8,7,6,5,5,5,5,6,6,6,6,6,6,7,7,8,9,9,9,10,12,14,16,20,24,27,30,34,38,41,45,49,54,57,60,65,69,75,80,86,93,97,102,106,111,114,117,120,122,125,128,131,134,137,139,142,145,147,149,150,152,154,155,157,159,160,161,163,164,165,166,168,169,171,172,173,174,175,176,178,180,182,183,185,186,187,188,190,192,193,194,196,197,199,200,201,202,203,204,205,205,206,207,208,208,209,209,209,209,209,209,209,209,209,209,208,207,206,205,204,203,202,201,199,198,196,194,192,190,188,186,184,182,179,176,174,171,168,165,162,159,157,154,151,149,146,143,139,136,133,130,128,125,123,120,117,114,112,109,108,106,104,103,101,99,98,97,96,96,96,96,96,97,97,97,98,100,101,103,105,107,109]

TEAL = [(0, 22, 24, 12), (30, 42, 44, 26), (52, 59, 60, 41), (67, 69, 79, 57), (82, 77, 98, 75), (97, 84, 114, 96),
        (112, 93, 130, 114), (127, 101, 144, 136), (142, 112, 163, 151), (157, 123, 181, 167), (172, 130, 192, 179),
        (255, 172, 240, 228)]
GREY = [(0, 20, 20, 8), (37, 46, 46, 20), (52, 59, 60, 39), (67, 71, 72, 59), (82, 85, 84, 78), (97, 101, 92, 99),
        (112, 110, 111, 115), (127, 121, 129, 133), (142, 132, 141, 154), (157, 144, 155, 170), (172, 152, 166, 183),
        (255, 196, 214, 236)]

def interp(tab, L, i):
    for a, b in zip(tab, tab[1:]):
        if a[0] <= L <= b[0]:
            f = (L - a[0]) / float(b[0] - a[0])
            return a[i] + f * (b[i] - a[i])
    return tab[-1][i]

def bg_rom(ch):
    out = []
    for tab in (TEAL, GREY):
        out += [max(0, min(255, int(round(interp(tab, L, ch))))) for L in range(256)]
    return out

def matrix_rom(ch):
    ky, ku, kv = [(1.2642, 2.4819, 0.4820)[ch], (1.7569, 1.4711, 0.2857)[ch], (0.5929, 1.1640, 1.7569)[ch]]
    out = []
    for v in range(256):
        y = int(round(2 * ky * v)); u = int(round(2 * ku * v)); w = int(round(2 * kv * v))
        out.append((y << 21) | (u << 11) | (w << 1))
    return out

def cos_rom(amp, off=0):
    return [int(round(off + amp * math.cos(2 * math.pi * i / 512))) for i in range(512)]

def log_rom():
    return [min(255, int(round(256 * math.log2(1 + i / 256.0)))) for i in range(256)]

def exp_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

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

def tables():
    return dict(G=G512, B=cos_rom(82, 135), RX=cos_rom(98), RY=cos_rom(127), LOG=log_rom(), EXP=exp_rom(),
                BGR=bg_rom(1), BGG=bg_rom(2), BGB=bg_rom(3), MR=matrix_rom(0), MG=matrix_rom(1), MB=matrix_rom(2))

def main():
    t = tables()
    parts = ["-- Generated by pyramid.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package pyramid_pkg is",
             vhd_table("C_PY_G", "unsigned", 512, 8, t['G']),
             vhd_table("C_PY_B", "unsigned", 512, 8, t['B']),
             vhd_table("C_PY_RX", "signed", 512, 8, t['RX'], signed=True),
             vhd_table("C_PY_RY", "signed", 512, 8, t['RY'], signed=True),
             vhd_table("C_PY_LOG", "unsigned", 256, 8, t['LOG']),
             vhd_table("C_PY_EXP", "unsigned", 256, 16, t['EXP']),
             vhd_table("C_PY_BGR", "unsigned", 512, 8, t['BGR']),
             vhd_table("C_PY_BGG", "unsigned", 512, 8, t['BGG']),
             vhd_table("C_PY_BGB", "unsigned", 512, 8, t['BGB']),
             vhd_table("C_PY_MR", "std_logic_vector", 256, 32, t['MR']),
             vhd_table("C_PY_MG", "std_logic_vector", 256, 32, t['MG']),
             vhd_table("C_PY_MB", "std_logic_vector", 256, 32, t['MB']),
             "end package pyramid_pkg;", ""]
    with open(os.path.join(HERE, "pyramid_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
