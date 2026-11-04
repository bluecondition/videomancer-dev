#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LENTIL v0.1.0 - build hook.  Writes lentil_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the still WIN_20230501_01_34_42 @ 0.5 s (see .heirloom_work/rb_lentil/notes.md):

  lens    = blue where |e| < W(row): e = signed position in the lens period (940 px, centres at 1008 + k 940),
            W = the lens half-width at this row, from the INVERSE lens profile at v = |y - 518| / sigma - c.
            The profile A(d) (lens half-height vs distance d from the pinch) was measured on all four
            half-lenses of the still; top and bottom halves differ (A_TOP / A_BOT below).  "Round" = a sine lens.
  stripes = a sawtooth VCO colourised purple -> pink -> yellow (PAL_EMP: the still's colour vs saw phase).
            Frequency f(y) = fs (1 + 1.14e-3 hyp(u)), u = | |y - 527| - 469 | (the ")" chevrons), hyp = sqrt(u^2 + 35^2).
            In the lens band the phase is re-seeded at every lens exit to 0.05 + kW 2 W and runs with a lag tail
            Gt e^(-d / 32 px) on top of f(y): stripes leave each lens parallel to its edge, then fan out flat.
  fringe  = a 26 px dark-purple ramp outside each lens.

Package contents (also read by .heirloom_work/rb_lentil/imodel.py):
  C_LT_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_LT_PROF  1024 x 16  lens half-width W16 (32768 = half a period): index = round * 512 + bottom * 256 + v/2
  C_LT_CHEV  1024 x 16  m(u) = round(4096 * 1.14e-3 * sqrt(u^2 + 35^2)), u = 0 .. 1023 virtual lines
  C_LT_PAL   512 x 32   {Y, U, V, 00}: bank * 256 + saw phase (Flamingo = the still, Lagoon)
  C_LT_FILL / C_LT_RIM   lens fill and fringe colours per bank
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

A_TOP = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 4, 4, 5, 6, 8, 8, 8, 10, 10, 12, 12, 12, 12, 14, 14, 15, 17, 20, 20, 20, 21, 22, 24, 25, 26, 26, 26, 28, 29, 31, 31, 32, 32, 34, 35, 38, 38, 40, 43, 44, 47, 48, 49, 50, 54, 56, 58, 60, 64, 64, 66, 67, 69, 71, 74, 77, 80, 82, 84, 86, 86, 87, 90, 90, 92, 93, 94, 97, 98, 98, 102, 103, 103, 104, 106, 106, 108, 108, 109, 110, 111, 112, 113, 114, 114, 116, 118, 118, 120, 121, 122, 122, 123, 123, 124, 125, 126, 126, 126, 127, 128, 129, 130, 131, 132, 132, 132, 134, 134, 135, 136, 136, 136, 137, 138, 138, 138, 139, 140, 140, 142, 142, 142, 142, 144, 144, 145, 146, 146, 146, 148, 148, 148, 149, 149, 150, 150, 150, 150, 151, 152, 152, 152, 152, 154, 154, 155, 156, 156, 157, 157, 158, 158, 159, 160, 160, 160, 161, 161, 162, 162, 163, 163, 163, 164, 164, 165, 165, 166, 166, 167, 167, 167, 168, 168, 168, 169, 170, 170, 171, 172, 173, 173, 173, 174, 175, 175, 175, 176, 177, 178, 178, 179, 180, 180, 181, 181, 182, 183, 183, 184, 184, 184, 185, 185, 185, 186, 187, 188, 189, 189, 190, 190, 192, 193, 193, 194, 194, 195, 196, 196, 197, 198, 198, 198, 200, 200, 200, 201, 202, 203, 204, 205, 206, 207, 208, 208, 208, 210, 210, 212, 212, 213, 213, 214, 215, 216, 217, 219, 220, 220, 221, 222, 223, 223, 224, 224, 226, 228, 229, 230, 231, 232, 233, 236, 237, 238, 239, 240, 241, 244, 244, 246, 248, 249, 251, 252, 253, 254, 254, 255, 256, 258, 260, 260, 262, 263, 264, 265, 266, 266, 268, 268, 269, 269, 270, 272, 273, 274, 274, 276, 277, 278, 279, 281, 282, 284, 284, 284, 286, 286, 286, 287, 288, 290, 290, 291, 292, 294, 294, 295, 296, 298, 298, 299, 300, 300, 300, 302, 302, 303, 304, 305, 306, 307, 308, 308, 308, 310, 310, 311, 311, 312, 313, 314, 314, 316, 316, 316, 316, 317, 318, 319, 318, 319, 320, 322, 322, 322, 322, 322, 324, 324, 325, 326, 326, 326, 328, 328, 328, 329, 329, 330, 330, 330, 330, 331, 332, 332, 333, 334, 334, 334, 334, 334, 334, 336, 336, 336, 336, 336, 338, 338, 338, 338, 339, 340, 340, 340, 341, 342, 342, 343, 344, 349, 350, 350, 350, 350, 352, 352, 353, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354, 354]
A_BOT = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 8, 9, 11, 12, 15, 16, 20, 22, 24, 24, 25, 28, 32, 32, 33, 38, 40, 45, 48, 50, 52, 56, 56, 56, 58, 59, 60, 61, 62, 64, 66, 66, 68, 68, 68, 70, 70, 71, 72, 72, 73, 74, 74, 76, 77, 78, 80, 80, 80, 81, 82, 83, 83, 84, 86, 86, 86, 86, 87, 88, 88, 90, 90, 90, 92, 92, 93, 94, 95, 96, 97, 97, 97, 98, 98, 98, 99, 99, 100, 100, 101, 102, 102, 103, 104, 104, 106, 106, 107, 107, 108, 108, 109, 110, 110, 111, 112, 112, 112, 113, 114, 115, 116, 116, 116, 118, 118, 118, 119, 119, 120, 121, 121, 122, 122, 122, 122, 123, 124, 124, 125, 126, 126, 126, 128, 128, 128, 128, 130, 130, 130, 130, 131, 132, 133, 133, 133, 134, 135, 136, 137, 137, 138, 138, 139, 140, 140, 140, 141, 141, 141, 142, 143, 144, 144, 144, 145, 146, 147, 147, 148, 149, 150, 150, 150, 151, 152, 152, 152, 154, 155, 156, 156, 156, 157, 158, 158, 158, 161, 161, 161, 162, 163, 164, 165, 165, 166, 168, 168, 169, 170, 171, 172, 172, 174, 174, 176, 178, 178, 179, 179, 180, 182, 182, 184, 185, 186, 186, 187, 189, 190, 192, 193, 194, 195, 195, 196, 198, 199, 202, 204, 204, 206, 207, 208, 210, 210, 212, 214, 218, 219, 220, 220, 222, 224, 226, 228, 230, 231, 232, 232, 236, 237, 238, 239, 240, 240, 241, 242, 243, 244, 246, 247, 250, 250, 251, 252, 252, 252, 254, 254, 255, 256, 256, 257, 258, 258, 260, 260, 261, 262, 262, 263, 263, 264, 266, 266, 267, 268, 268, 268, 268, 270, 270, 270, 271, 272, 273, 274, 274, 274, 274, 276, 276, 276, 276, 278, 278, 278, 278, 279, 279, 280, 281, 281, 281, 282, 282, 282, 284, 284, 285, 286, 286, 286, 287, 287, 288, 288, 289, 289, 290, 291, 291, 292, 292, 293, 293, 293, 294, 295, 295, 296, 296, 297, 298, 298, 298, 299, 300, 300, 301, 301, 301, 302, 302, 303, 304, 304, 305, 305, 306, 306, 306, 307, 307, 308, 309, 310, 310, 310, 310, 310, 311, 312, 312, 312, 314, 314, 314, 316, 316, 316, 316, 316, 317, 318, 318, 318, 318, 320, 320, 320, 322, 322, 322, 323, 324, 324, 324, 324, 326, 326, 326, 327, 328, 328, 328, 328, 330, 330, 330, 330, 330, 332, 337, 339, 339, 339, 340, 341, 341, 341, 341, 343, 343, 343, 343, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None]
PAL_EMP = [(242, 95, 149), (232, 81, 151), (221, 71, 153), (212, 64, 155), (206, 60, 156), (203, 57, 157), (202, 56, 157), (202, 57, 158), (203, 57, 158), (204, 58, 157), (207, 60, 157), (210, 62, 157), (214, 65, 157), (219, 68, 156), (224, 72, 156), (228, 75, 155), (233, 79, 155), (237, 83, 154), (240, 87, 154), (243, 91, 153), (246, 95, 153), (248, 99, 152), (250, 103, 152), (252, 107, 151), (253, 111, 151), (254, 115, 151), (255, 120, 150), (255, 124, 150), (255, 128, 149), (255, 132, 149), (255, 136, 148), (255, 141, 148), (254, 145, 147), (254, 149, 147), (254, 153, 146), (254, 158, 146), (254, 162, 145), (253, 166, 145), (253, 170, 144), (253, 174, 144), (252, 178, 143), (252, 182, 143), (252, 186, 142), (251, 190, 142), (251, 194, 141), (250, 198, 141), (250, 202, 140), (250, 205, 140), (249, 209, 139), (249, 213, 139), (249, 216, 138), (248, 219, 138), (248, 222, 138), (248, 225, 137), (247, 228, 137), (247, 230, 137), (247, 232, 136), (247, 233, 136), (248, 231, 138), (250, 220, 140), (252, 199, 143), (254, 170, 145), (254, 141, 146), (250, 115, 148)]

KV, VR = 1.14e-3, 35.0
FILL_RGB = [(3, 0, 177), (150, 0, 40)]       # lens fill: Flamingo blue, Lagoon crimson
RIM_RGB = [(96, 8, 168), (60, 4, 40)]        # fringe: dark purple, dark wine


def prof(a, atop):
    """measured half-height (None past the band edge) -> monotone profile reaching atop past the measurement"""
    last = max(i for i, v in enumerate(a) if v is not None)
    out, m = [], 0.0
    for d in range(len(a)):
        v = a[d] if (d <= last and a[d] is not None) else atop
        m = max(m, float(v))
        out.append(m)
    return out


def round_prof():
    return [0.0 if d < 30 else 372.0 * math.sin(math.pi / 2 * min(1.0, (d - 30) / 440.0)) for d in range(471)]


def inverse(a, v):
    """smallest d (interpolated) with a(d) >= v, starting at the tip; 470 past the top"""
    d0 = max(i for i in range(len(a)) if a[i] <= 0.0)
    if v <= 0:
        return float(d0)
    for d in range(d0 + 1, len(a)):
        if a[d] >= v:
            lo = a[d - 1]
            return d - 1 + (v - lo) / max(1e-9, a[d] - lo)
    return 470.0


def prof_rom():
    tabs = [prof(A_TOP, 362.0), prof(A_BOT, 352.0), round_prof(), round_prof()]
    out = []
    for t in tabs:
        for i in range(256):
            d = inverse(t, 2 * i + 1)
            out.append(max(0, min(32768, int(round(32768 * (470.0 - d) / 470.0)))))
    return out


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def prescale(yuv):
    """luma x 16/13 for the shell's 13/16 output scale, with a soft knee above 760 (so the still's yellow
    ramp keeps its gradient instead of clipping flat), capped at 940"""
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    if y2 > 760:
        y2 = 760 + (y2 - 760) * 0.7
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def word(rgb):
    y, u, v = prescale(rgb_to_hw10(rgb))
    return (y << 22) | (u << 12) | (v << 2)


def lagoon():
    lo, mid, hi = (12, 70, 96), (40, 196, 205), (226, 252, 150)
    out = []
    for p in PAL_EMP:
        t = min(1.0, max(0.0, (p[1] - 56) / (233.0 - 56.0)))
        if t < 0.5:
            c = [lo[i] + (mid[i] - lo[i]) * t / 0.5 for i in range(3)]
        else:
            c = [mid[i] + (hi[i] - mid[i]) * (t - 0.5) / 0.5 for i in range(3)]
        out.append(tuple(c))
    return out


def pal_rom():
    out = []
    for tab in (PAL_EMP, lagoon()):
        for j in range(256):
            idx = j / 4.0
            i0 = int(math.floor(idx)) % 64
            fr = idx - math.floor(idx)
            c = [tab[i0][k] * (1 - fr) + tab[(i0 + 1) % 64][k] * fr for k in range(3)]
            out.append(word(c))
    return out


def chev_rom():
    return [min(65535, int(round(4096 * KV * math.sqrt(u * u + VR * VR)))) for u in range(1024)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by lentil.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package lentil_pkg is",
             vhd_table("C_LT_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_LT_PROF", "unsigned", 1024, 16, prof_rom()),
             vhd_table("C_LT_CHEV", "unsigned", 1024, 16, chev_rom()),
             vhd_table("C_LT_PAL", "std_logic_vector", 512, 32, pal_rom()),
             "    type t_lt_col is array (0 to 1) of std_logic_vector(31 downto 0);",
             "    constant C_LT_FILL : t_lt_col := (x\"%08X\", x\"%08X\");" % (word(FILL_RGB[0]), word(FILL_RGB[1])),
             "    constant C_LT_RIM  : t_lt_col := (x\"%08X\", x\"%08X\");" % (word(RIM_RGB[0]), word(RIM_RGB[1])),
             "end package lentil_pkg;", ""]
    with open(os.path.join(HERE, "lentil_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
