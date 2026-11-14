#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CHLADNI v0.1.0 - build hook.  Writes chladni_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The same tables are read by the prototype and the integer model
(.patchbay_work/rb_chladni/proto.py, imodel.py).

Package contents:
  C_CH_SIN10  512 x 16   round(1023 sin((i + 1/2) pi / 1024))   quarter wave for the two H oscillators
  C_CH_SIN12  512 x 16   round(4095 sin((i + 1/2) pi / 1024))   quarter wave for the V oscillators / the mixer
  C_CH_PAL    1024 x 32  {Y, U, V, 00} colouriser: index = look * 512 + colours * 256 + sign * 128 + phase
                         (phase = 7 bits of one colouriser period of |F|, 0 at the node); the Sand look has no
                         sign, and its sign = 1 half holds the sand cross-sections: 640 + colours * 256 + look * 32 + t,
                         t = 32 (1 - |F| / window): 1 at the window's edge .. 31 at the nodal line
  C_CH_RIM    4 x 32     plate rim colour by look * 2 + colours;  C_CH_K the black outside the plate
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the
shell's 13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

K0 = (10, 8, 14)
# Colouriser settings: (start phase 0..1, rgb) stops over one period of |F|; each colour holds to the next stop.
# "Ember": indigo at the node, cyan fringe, magenta, yellow fringe, orange-red, dark fringe (positive lobes);
#          the negative lobes run pink fringe, violet, aqua fringe, blue.
PAL0P = [(0.00, (26, 20, 110)), (0.20, (40, 190, 235)), (0.26, (232, 20, 150)), (0.56, (255, 225, 40)),
         (0.62, (240, 70, 12)), (0.92, (20, 10, 30))]
PAL0N = [(0.00, (26, 20, 110)), (0.20, (255, 160, 200)), (0.26, (96, 26, 190)), (0.56, (60, 220, 235)),
         (0.62, (18, 96, 200)), (0.92, (20, 10, 30))]
# "Verdigris": deep green at the node, lime fringe, jade, cream fringe, amber / orange fringe, red, peach, plum
PAL1P = [(0.00, (6, 44, 36)), (0.20, (200, 245, 40)), (0.26, (0, 168, 120)), (0.56, (255, 250, 200)),
         (0.62, (236, 150, 0)), (0.92, (10, 14, 10))]
PAL1N = [(0.00, (6, 44, 36)), (0.20, (255, 170, 60)), (0.26, (200, 40, 30)), (0.56, (255, 215, 170)),
         (0.62, (110, 16, 70)), (0.92, (10, 14, 10))]
# Sand look: black, with a thin dust ring at the end of every colouriser period
DUST0 = [(0.00, (6, 6, 8)), (0.86, (96, 84, 64)), (0.94, (190, 170, 130)), (0.99, (6, 6, 8))]
DUST1 = [(0.00, (4, 8, 8)), (0.86, (50, 90, 84)), (0.94, (120, 200, 186)), (0.99, (4, 8, 8))]
# sand cross-sections (fringe at the window's edge, body, core) by look * 2 + colours
SANDC = [((60, 30, 10), (236, 214, 160), (255, 246, 214)),
         ((20, 40, 30), (232, 226, 190), (255, 252, 230)),
         ((90, 70, 40), (238, 220, 170), (255, 248, 220)),
         ((40, 80, 76), (200, 236, 226), (240, 255, 250))]
RIM = [(196, 150, 60), (150, 170, 160), (120, 104, 80), (70, 110, 104)]


def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def band_palette(stops, n=128, soft=2.0):
    out = []
    for i in range(n):
        ph = (i + 0.5) / n
        acc = [0.0, 0.0, 0.0]
        wsum = 0.0
        for k in range(-8, 9):
            p2 = (ph + k * soft / (8.0 * n)) % 1.0
            c = stops[-1][1]
            for s, col in stops:
                if p2 >= s:
                    c = col
            w = 1.0 - abs(k) / 9.0
            for j in range(3):
                acc[j] += w * c[j]
            wsum += w
        out.append(tuple(a / wsum for a in acc))
    return out


def sand_profile(fringe, body, core, n=32):
    """index t = 32 (1 - |F| / window), 1 .. 31 (0 = outside the window: the bands show): a thin dark fringe at
    the window's edge, a ramp to the body colour, then brightening to the core at the nodal line."""
    out = []
    for t in range(n):
        if t < 5:
            c = fringe
        elif t < 12:
            c = lerp(fringe, body, (t - 4) / 8.0)
        else:
            c = lerp(body, core, (t - 11) / 20.0)
        out.append(c)
    return out


def pal_rgb():
    """[look][colours][sign] -> 128 rgb"""
    d0 = band_palette(DUST0, soft=1.5)
    d1 = band_palette(DUST1, soft=1.5)
    return [[[band_palette(PAL0P), band_palette(PAL0N)], [band_palette(PAL1P), band_palette(PAL1N)]],
            [[d0, d0], [d1, d1]]]


def sand_rgb():
    """[look * 2 + colours] -> 32 rgb"""
    return [sand_profile(*c) for c in SANDC]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))


def yuv_of(rgb):
    return prescale(rgb_to_hw10(rgb))


def word(rgb):
    y, u, v = yuv_of(rgb)
    return (y << 22) | (u << 12) | (v << 2)


def sin10_rom():
    return [int(round(1023 * math.sin((i + 0.5) * math.pi / 1024))) for i in range(512)]


def sin12_rom():
    return [int(round(4095 * math.sin((i + 0.5) * math.pi / 1024))) for i in range(512)]


def pal_rom():
    """1024 words.  Bands: index = look * 512 + colours * 256 + sign * 128 + phase (the Sand look has no sign: its
    sign = 1 half holds the sand cross-sections: 640 + colours * 256 + look * 32 + t)."""
    p = pal_rgb()
    sd = sand_rgb()
    out = []
    for look in (0, 1):
        for col in (0, 1):
            for sg in (0, 1):
                if look == 1 and sg == 1:
                    blk = [word(c) for c in sd[col]] + [word(c) for c in sd[2 + col]] + [word(K0)] * 64
                else:
                    blk = [word(c) for c in p[look][col][sg]]
                out += blk
    return out


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by chladni.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package chladni_pkg is",
             vhd_table("C_CH_SIN10", "unsigned", 512, 16, sin10_rom()),
             vhd_table("C_CH_SIN12", "unsigned", 512, 16, sin12_rom()),
             vhd_table("C_CH_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             vhd_table("C_CH_RIM", "std_logic_vector", 4, 32, [word(c) for c in RIM]),
             '    constant C_CH_K : std_logic_vector(31 downto 0) := x"%08X";' % word(K0),
             "end package chladni_pkg;", ""]
    with open(os.path.join(HERE, "chladni_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
