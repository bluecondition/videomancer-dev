#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
MONOCHORD v0.1.0 - build hook (two strings per pixel).  Writes monochord_pkg.vhd next to this file (pure python: the builder runs it with
the system python3; imodel.py imports the same tables).

Package contents:
  C_MC_SIN  1024 x 10  quarter sine: round(1023 sin(pi/2 * 4 i / 4095)), indexed by the node distance >> 2
  C_MC_PAL  512 x 24   colourizer, index = pal * 256 + hue * 16 + level: {dY/4 (unsigned), dU/4, dV/4 (signed)}, the
                       colour ADDED by one string at that level (hardware convention U = Cr, V = Cb; dY pre-scaled
                       16/13 for the shell's 13/16 output scale)
  C_MC_SQ   256 x 16   l * l (square-law camera)
  C_MC_WD   20 x 7     bead half-width (half lines) at |v| = k lines: round(2 sqrt(20^2 - k^2))
  C_MC_WD8  8 x 7      the same for a radius of 8 (480i / 576i, where a line unit is 3 virtual lines)
  C_MC_W    32 x 11    row amplitude weights, Q10: 0..7 ladder (a slight unevenness), 8..15 Fan's fundamental,
                       16..23 Fan's top harmonic (x 0.5)
  C_MC_DN   8 x 8      LFO detune per row (Q8 harmonics, signed)
  C_MC_RT0  256 x 16   row table power-up contents (word 7 of each row = the LFO start phase)

Levels (per string): 0 none, 1-2 fringe (a hue 3 steps on), 3 edge AA, 4 edge peak, 5-9 the dwell ramp, 10 body,
11 line halo, 12 bead / fret white, 13 white-hot core, 14 / 15 Fan's dim body / ramp.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

HUES = [(255, 30, 40), (255, 90, 10), (255, 160, 0), (240, 225, 0), (140, 240, 20), (30, 230, 60),
        (0, 230, 160), (0, 210, 255), (30, 120, 255), (60, 50, 255), (130, 30, 255), (200, 20, 255),
        (255, 20, 220), (255, 30, 150), (255, 40, 100), (255, 35, 60)]
PHOS = [(255, 150, 20), (255, 190, 40), (200, 255, 40), (60, 255, 80), (40, 255, 190), (80, 220, 255),
        (150, 180, 255), (220, 210, 255), (255, 230, 200), (255, 120, 60), (255, 80, 110), (255, 120, 220),
        (200, 120, 255), (120, 160, 255), (60, 255, 255), (255, 210, 120)]
BODY = 0.46
GN = [1.00, 0.95, 1.04, 0.97, 1.03, 0.94, 1.05, 0.98]
FANW = [1.0, 0.5]
DN = [0.00, 0.07, -0.05, 0.11, -0.08, 0.04, 0.13, -0.10]
PH0 = [0.135, 0.325, 0.097, 0.334, 0.125, 0.341, 0.087, 0.285]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def clip3(c):
    return [min(255.0, max(0.0, x)) for x in c]


def level_rgb(pal, hue, L):
    base = HUES[hue % 16] if pal == 0 else PHOS[hue % 16]
    if pal == 0:
        fr = HUES[(hue + 3) % 16]
    else:
        fr = [PHOS[hue % 16][0] * 1.0, PHOS[hue % 16][1] * 0.8, PHOS[hue % 16][2] * 0.6]
    mul = lambda c, k, a=0.0: [x * k + a for x in c]
    if L == 0: return None
    if L == 1: c = mul(fr, 0.30)
    elif L == 2: c = mul(fr, 0.62)
    elif L == 3: c = mul(base, 0.60)
    elif L == 4: c = [min(255.0, x + 40) for x in base]
    elif 5 <= L <= 9:
        f = 1.0 - (L - 4) / 6.0
        c = mul(base, BODY + (1 - BODY) * f ** 1.4)
    elif L == 10: c = mul(base, BODY)
    elif L == 11: c = [min(255.0, x * 0.85 + 70) for x in base]
    elif L == 12: c = mul(base, 0.08, 222)
    elif L == 13: c = mul(base, 0.16, 200)
    elif L == 14: c = mul(base, 0.24)
    else: c = mul(base, 0.40)
    return clip3(c)


def pal_word(pal, hue, L):
    c = level_rgb(pal, hue, L)
    if c is None: return 0
    y, u, v = rgb_to_hw10(c)
    dy = (y - 64) * 16.0 / 13.0
    y8 = min(255, max(0, int(round(dy / 4.0))))
    u8 = min(127, max(-128, int(round((u - 512) / 4.0))))
    v8 = min(127, max(-128, int(round((v - 512) / 4.0))))
    return (y8 << 16) | ((u8 & 255) << 8) | (v8 & 255)


def pal_rom():
    return [pal_word(p, h, L) for p in (0, 1) for h in range(16) for L in range(16)]


def sin1k_rom():
    return [min(1023, int(round(math.sin(math.pi / 2 * 4 * i / 4095.0) * 1023))) for i in range(1024)]


def sin_rom():
    out = []
    for i in range(256):
        a = math.sin(math.pi / 2 * 16 * i / 4095.0) * 1023
        b = math.sin(math.pi / 2 * 16 * (i + 1) / 4095.0) * 1023
        s0 = min(1023, int(round(a)))
        d8 = min(63, max(0, int(round(8 * (b - a)))))
        out.append((d8 << 10) | s0)
    return out


def sq_rom():
    return [i * i for i in range(256)]


def wd_rom():
    return [int(round(2 * math.sqrt(400 - k * k))) for k in range(20)]


def wd8_rom():
    return [int(round(2 * math.sqrt(64 - k * k))) for k in range(8)]


def w_rom():
    out = [int(round(g * 1024)) for g in GN]
    for r in range(2):
        out += [int(round(GN[n] * FANW[r] * 1024)) for n in range(8)]
    return out + [1024] * 8


def dn_rom():
    return [int(round(d * 256)) for d in DN]


def rt0():
    out = [0] * 256
    for n in range(8):
        out[n * 8 + 7] = int(round(PH0[n] * 65536)) & 0xFFFF
    return out


def vhd_table(name, typ, depth, width, vals, signed=False):
    t = "t_" + name.lower()
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % (t, depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, t)]
    items = []
    for v in vals:
        if signed:
            items.append('to_signed(%d, %d)' % (v, width))
        elif typ == 'unsigned':
            items.append('to_unsigned(%d, %d)' % (v, width))
        else:
            items.append('x"%0*X"' % ((width + 3) // 4, v))
    per = 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by monochord.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package monochord_pkg is",
             vhd_table("C_MC_SIN", "unsigned", 1024, 10, sin1k_rom()),
             vhd_table("C_MC_PAL", "std_logic_vector", 512, 24, pal_rom()),
             vhd_table("C_MC_SQ", "unsigned", 256, 16, sq_rom()),
             vhd_table("C_MC_WD", "unsigned", 20, 7, wd_rom()),
             vhd_table("C_MC_WD8", "unsigned", 8, 7, wd8_rom()),
             vhd_table("C_MC_W", "unsigned", 32, 11, w_rom()),
             vhd_table("C_MC_DN", "signed", 8, 8, dn_rom(), signed=True),
             vhd_table("C_MC_RT0", "unsigned", 256, 16, rt0()),
             "end package monochord_pkg;", ""]
    with open(os.path.join(HERE, "monochord_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
