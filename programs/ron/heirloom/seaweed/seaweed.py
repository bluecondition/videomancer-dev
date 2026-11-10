#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SEAWEED v0.1.0 - build hook.  Writes seaweed_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).

Fitted to the still WIN_20230402_23_03_06 at 0.5 s (see .heirloom_work/rb_seaweed/notes.md):

  VCO P (the curtain bands) is reset by hsync (phase 0 at x0 = -155 virtual px) and FM'd per line by one
  travelling vertical LFO (562 lines, depth 9.6 %, down at 253 lines/s):
      period(y) = 206.0 px (1 + 0.096 sin(2 pi y / 562 - 0.642)),  u = (x - x0) / period(y)
  A sub-octave colouriser paints each band from its phase fu = frac(u) and its parity (two cycles per
  palette): even bands carry a violet line with a blue edge, odd bands a brown core.  A half-rate VCO Q
  (q = (k + 0.33) (1/2 + eps m) + psi(y), psi a vertical ramp) plus a quarter-rate term give each band a key
  level z: odd bands grow brown cores where z is high, even bands turn magenta and then cut hard black
  pebble blobs where z plus a vertical ramp is high (the top of the frame).

Package contents (also read by imodel.py):
  C_SW_SIN   1024 x 12  round(2047 sin(2 pi i / 1024)), signed (the line engine's LFO and Q sines)
  C_SW_EXP2  256 x 16   round(32768 * 2^(i/256))
  C_SW_ZC    1024 x 16  the key oscillator Q: parabolic cosine of (q + 1/4), {z Q10 (11 bits), dz (5 bits)},
                        interpolated over the 6 phase bits below the index
  C_SW_SQRT  512 x 8    round(255 sqrt(r (2 - r))), r = i / 511 (the round ends of cores and blobs)
  C_SW_GND   512 x 32   {Y, U, V, 00} ground palette: page (Colours) | parity | fu (128 per band)
  C_SW_FEAT  256 x 32   feature colours: page | 0..63 core by distance from its right edge, 64..95 edge
                        (blue line, lavender, pink), 96..103 core lead-in, 104..111 magenta levels,
                        112 cyan tip, 113 blob, 114 blob rim, 115 glow blob, 116 glow rim,
                        117..125 the even bands' edge (blue line, violet-magenta fringe)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the
shell's 13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ground palettes, keypoints (fu, RGB) measured from the still's unwrapped band profiles
GE = [(0.00, (95, 234, 181)), (0.06, (135, 218, 192)), (0.12, (170, 201, 199)), (0.20, (222, 174, 214)),
      (0.245, (150, 212, 170)), (0.275, (210, 160, 60)), (0.30, (205, 120, 45)), (0.34, (150, 70, 100)),
      (0.53, (137, 72, 110)), (0.55, (250, 140, 86)), (0.62, (240, 160, 75)), (0.70, (215, 180, 80)),
      (0.76, (170, 205, 95)), (0.84, (100, 236, 115)), (0.92, (20, 255, 145))]
GO = [(0.00, (30, 255, 158)), (0.05, (80, 240, 150)), (0.11, (120, 226, 125)), (0.17, (110, 230, 115)),
      (0.22, (160, 205, 90)), (0.27, (225, 160, 60)), (0.31, (232, 127, 41)), (0.36, (220, 150, 60)),
      (0.42, (150, 214, 105)), (0.50, (110, 230, 115)), (0.60, (80, 240, 125)), (0.70, (40, 252, 135)),
      (0.80, (14, 255, 150)), (0.88, (40, 250, 165)), (0.95, (80, 240, 178))]

CORE_KEYS = [(0.00, (130, 95, 120)), (0.02, (150, 88, 70)), (0.05, (167, 82, 31)), (0.10, (135, 67, 28)),
             (0.15, (100, 50, 22)), (0.20, (65, 36, 17)), (0.25, (36, 25, 15)), (0.30, (24, 23, 14)),
             (0.50, (26, 22, 14))]
EDGE_KEYS = [(0.000, (100, 90, 200)), (0.008, (84, 81, 248)), (0.030, (84, 81, 248)), (0.036, (150, 120, 220)),
             (0.060, (150, 120, 220)), (0.066, (221, 135, 150)), (0.090, (200, 160, 140))]
LEAD = [(215, 140, 45), (190, 115, 40), (160, 95, 35), (125, 72, 30), (95, 55, 26), (70, 42, 22), (50, 33, 18),
        (36, 27, 15)]
MAG = [(137, 72, 110), (160, 66, 128), (185, 60, 146), (205, 55, 162), (222, 50, 176), (236, 46, 186),
       (247, 42, 194), (255, 40, 200)]
SPECIAL = [(60, 230, 240), (5, 7, 5), (230, 70, 90), (255, 248, 120), (255, 120, 40)]
EVEN_EDGE = [(60, 40, 245), (60, 40, 245), (60, 40, 245), (60, 40, 245), (110, 50, 228), (166, 58, 210),
             (166, 58, 210), (166, 58, 210), (166, 58, 210)]

CORAL_DEG = 160.0   # Colours = Coral: chroma rotated (green -> coral pink, orange -> teal, blue -> amber)


def lerp_keys(keys, x, wrap):
    f = [k[0] for k in keys]
    c = [k[1] for k in keys]
    if wrap:
        f = f + [keys[0][0] + 1.0]
        c = c + [keys[0][1]]
    if x <= f[0]:
        return c[0]
    for i in range(len(f) - 1):
        if f[i] <= x <= f[i + 1]:
            t = (x - f[i]) / (f[i + 1] - f[i]) if f[i + 1] > f[i] else 0.0
            return tuple(c[i][j] + (c[i + 1][j] - c[i][j]) * t for j in range(3))
    return c[-1]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def coral(yuv):
    y, u, v = yuv
    a = math.radians(CORAL_DEG)
    cr, cb = u - 512, v - 512
    cr2 = cr * math.cos(a) - cb * math.sin(a)
    cb2 = cr * math.sin(a) + cb * math.cos(a)
    return (y, 512 + cr2, 512 + cb2)


def word(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    u = int(round(min(1000, max(24, u))))
    v = int(round(min(1000, max(24, v))))
    return (y2 << 22) | (u << 12) | (v << 2)


def ground_rgb():
    """[parity][128] RGB of the ground palettes"""
    return [[lerp_keys(GE if par == 0 else GO, i / 128.0, True) for i in range(128)] for par in (0, 1)]


def feat_rgb():
    t = []
    for i in range(64):
        t.append(lerp_keys(CORE_KEYS, i / 128.0, False))
    for j in range(32):
        t.append(lerp_keys(EDGE_KEYS, j / 256.0, False))
    t += LEAD + MAG + SPECIAL + EVEN_EDGE
    while len(t) < 128:
        t.append((0, 0, 0))
    return t


def tables():
    gnd, feat = [], []
    g = ground_rgb()
    f = feat_rgb()
    for page in (0, 1):
        for par in (0, 1):
            for i in range(128):
                yuv = rgb_to_hw10(g[par][i])
                gnd.append(word(coral(yuv) if page else yuv))
    for page in (0, 1):
        for i in range(128):
            yuv = rgb_to_hw10(f[i])
            if page and i not in (113, 115):   # blobs keep their colour in Coral
                yuv = coral(yuv)
            feat.append(word(yuv))
    return gnd, feat


def sin_rom():
    return [int(round(2047 * math.sin(2 * math.pi * i / 1024.0))) for i in range(1024)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def zc_tab():
    """the key oscillator's waveform: a parabolic cosine of (q + 1/4) in 1024 steps, Q10, clipped to +-1023"""
    out = []
    for i in range(1024):
        a = i & 511
        sv = min(1023, (a * (512 - a)) >> 6)
        out.append(-sv if i >= 512 else sv)
    return out


def zc_rom():
    """{z (11 bits, signed), dz = z[i+1] - z[i] (5 bits, signed)} per entry, for linear interpolation"""
    z = zc_tab()
    return [((z[i] & 2047) << 5) | ((z[(i + 1) & 1023] - z[i]) & 31) for i in range(1024)]


def sqrt_rom():
    out = []
    for i in range(512):
        r = i / 511.0
        out.append(int(round(255 * math.sqrt(max(0.0, r * (2 - r))))))
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
    gnd, feat = tables()
    parts = ["-- Generated by seaweed.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package seaweed_pkg is",
             vhd_table("C_SW_SIN", "signed", 1024, 12, sin_rom(), signed=True),
             vhd_table("C_SW_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_SW_ZC", "std_logic_vector", 1024, 16, zc_rom()),
             vhd_table("C_SW_SQRT", "unsigned", 512, 8, sqrt_rom()),
             vhd_table("C_SW_GND", "std_logic_vector", 512, 32, gnd),
             vhd_table("C_SW_FEAT", "std_logic_vector", 256, 32, feat),
             "end package seaweed_pkg;", ""]
    with open(os.path.join(HERE, "seaweed_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
