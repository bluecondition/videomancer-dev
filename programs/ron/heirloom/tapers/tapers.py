#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TAPERS v0.1.0 - build hook.  Writes tapers_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).

Fitted to the clip WIN_20230203_00_19_56 at 6.0 s (see .heirloom_work/rb_tapers/notes.md):
  key     cos(2pi (x - 48) / 289.5) - (0.0017 |x - 1000| - 0.5)  >  T(y)          (virtual 1080-line px)
  T(y)    0.75 sin(2pi (y - ys) / 1380) + 0.35 (1/2 - frac((y - 185) / 154.3)),  ys drifting up 440 lines/s
  inside  a vertical sine of period 206 lines through the pink / lavender / sky / teal / green LUT, sampled per
          22-line mosaic row, scrolling up 128 lines/s
  outside the camera (44 x 22 mosaic, 8 levels): black, navy, dashed cycle, block cycle, maroon, blue, pink,
          yellow; the cycle = two square waves of period 174 lines (blue, maroon, black, navy), up 160 lines/s

Package contents (also read by imodel.py):
  C_TP_SIN   1024 x 12  signed round(1024 sin(2pi i / 1024))
  C_TP_EXP2  256 x 16   round(32768 2^(i/256))
  C_TP_PIN   256 x 32   {Y, U, V, 00} inside colours: index = pal * 32 + hue (32 steps, 0 = pink peak)
  C_TP_POUT  256 x 32   {Y, U, V, 00} outside colours: index = pal * 32 + level * 4 + A * 2 + B
                        (level 2 / 3 use A, B; the others ignore them)
Colours in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# interior hue samples of the photo (y, RGB) at x 880-960; phase = (y + 5.5 - 220) / 206, 0 = the pink peak
HUE_SAMPLES = [(33, (201, 93, 255)), (44, (161, 136, 255)), (55, (133, 164, 240)), (66, (96, 199, 204)),
               (77, (73, 218, 179)), (88, (71, 220, 177)), (99, (33, 252, 135)), (110, (0, 255, 78)),
               (121, (0, 255, 78)), (132, (25, 253, 124)), (143, (50, 238, 154)), (154, (75, 218, 180)),
               (165, (73, 218, 180)), (176, (120, 179, 232)), (187, (161, 136, 255)), (198, (202, 93, 255)),
               (209, (228, 63, 255)), (220, (238, 52, 255)), (231, (233, 58, 255)), (242, (199, 96, 255))]
L_HUE, Y_HUE0 = 206.0, 220.0

# outside colours per palette: black, navy, blue, maroon, pink, yellow
PAL_OUT = [
    dict(black=(22, 32, 9), navy=(22, 31, 48), blue=(19, 27, 198), maroon=(105, 25, 50), pink=(232, 17, 165), yellow=(244, 212, 55)),
    dict(black=(10, 10, 24), navy=(40, 20, 60), blue=(40, 160, 220), maroon=(170, 50, 20), pink=(255, 230, 60), yellow=(230, 250, 255)),
]


def interp(ph, xs, ys):
    # linear interpolation on a cyclic axis (xs sorted in 0..1)
    xs2 = [x - 1 for x in xs] + list(xs) + [x + 1 for x in xs]
    ys2 = list(ys) * 3
    for i in range(len(xs2) - 1):
        if xs2[i] <= ph <= xs2[i + 1]:
            w = (ph - xs2[i]) / max(1e-9, xs2[i + 1] - xs2[i])
            return ys2[i] * (1 - w) + ys2[i + 1] * w
    return ys2[0]


def hue_lut():
    pts = sorted((((y + 5.5 - Y_HUE0) / L_HUE) % 1.0, c) for y, c in HUE_SAMPLES)
    xs = [p for p, _ in pts]
    out = []
    for i in range(32):
        p = (i + 0.5) / 32
        out.append(tuple(interp(p, xs, [c[k] for _, c in pts]) for k in range(3)))
    return out


def hue_lut2():
    out = []
    for i in range(32):
        p = (i + 0.5) / 32
        c = 0.5 - 0.5 * math.cos(2 * math.pi * p)
        c = min(1.0, max(0.0, (c - 0.1) / 0.8))
        out.append(tuple((1 - c) * a + c * b for a, b in zip((255, 170, 20), (0, 225, 255))))
    return out


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def word(rgb):
    y, u, v = prescale(rgb_to_hw10(rgb))
    return (y << 22) | (u << 12) | (v << 2)


def sin_rom():
    return [max(-2048, min(2047, int(round(1024 * math.sin(2 * math.pi * i / 1024))))) for i in range(1024)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def pin_rom():
    out = [0] * 256
    for p, lut in enumerate((hue_lut(), hue_lut2())):
        for i in range(32):
            out[p * 32 + i] = word(lut[i])
    for i in range(64, 256):
        out[i] = out[i % 64]
    return out


def pout_rom():
    out = [0] * 256
    for p, pal in enumerate(PAL_OUT):
        cyc = [pal['blue'], pal['maroon'], pal['black'], pal['navy']]   # (A,B) = (1,0), (1,1), (0,1), (0,0)
        def cyc_col(a, b):
            if a and not b: return cyc[0]
            if a and b: return cyc[1]
            if (not a) and b: return cyc[2]
            return cyc[3]
        for lv in range(8):
            for a in (0, 1):
                for b in (0, 1):
                    if lv == 0: c = pal['black']
                    elif lv == 1: c = pal['navy']
                    elif lv in (2, 3): c = cyc_col(a, b)
                    elif lv == 4: c = pal['maroon']
                    elif lv == 5: c = pal['blue']
                    elif lv == 6: c = pal['pink']
                    else: c = pal['yellow']
                    out[p * 32 + lv * 4 + a * 2 + b] = word(c)
    for i in range(64, 256):
        out[i] = out[i % 64]
    return out


def vhd_table(name, typ, depth, width, vals, signed=False):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        elif width % 4 == 0:
            items.append('x"%0*X"' % (width // 4, v))
        else:
            items.append('to_unsigned(%d, %d)' % (v, width))
    per = 8 if signed else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by tapers.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tapers_pkg is",
             vhd_table("C_TP_SIN", "signed", 1024, 12, sin_rom(), signed=True),
             vhd_table("C_TP_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_TP_PIN", "std_logic_vector", 256, 32, pin_rom()),
             vhd_table("C_TP_POUT", "std_logic_vector", 256, 32, pout_rom()),
             "end package tapers_pkg;", ""]
    with open(os.path.join(HERE, "tapers_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
