#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CRUCIBLE v0.1.1 - build hook.  Writes crucible_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  Heirloom from the video clips; still WIN_20230518_23_23_28 t0.5.

The patch, as fitted (.heirloom_work/rb_crucible/notes.md):
  W(y)   = cos(2 pi ((y - 102) / P - rp))         a vertical oscillator rolling down (P 356 lines, 172 lines/s)
  F      = W + 1.4 u + heat (+ camera)             u = |x - 992| / 960: the H triangle mixed in
  zone   = |x - 992| < hw(y)                       the black figure: a per-line half-width table, morphing
                                                   head (the still) <-> diamond band, locked to the roll
  inside the zone:  yellow/orange where F - m max(0,|d|-110)/100 > 0.80 (cream over a lens), the lens where
                    the flame level is mid (or at a deep trough: the eyes), black elsewhere
  outside:          red/pink stripes where F + n (1 - |d|/500) > 1.68 (and a red border along the cross),
                    a pale rim on the figure's edge, mauve fading to violet away from it
  lenses = a column of stacked blue lenses at x = 875: -cos(2 pi phi(|y - 520|)) + 1.65 lx^2 < th, the row
           phase phi chirped (rows at 0, 326, 426, 512 lines), breathing 9 % with the roll

Fixed-point units used by the RTL and imodel.py:
  x in h (half virtual px, 3840 per line); lines in virtual 1080-line units; F in Q8 (1.0 = 256)
  zone ROM: 1024 x 12, [0..255] head-phase upper arm, [256..511] lower arm, [512..767] diamond phase, by
            (scaled distance from the centre line) >> 1; value = half-width in h (4095 = whole line)
  lens ROM: 512 x 8 signed, cos(2 pi phi(2 i)) * 127
  cos ROM : 256 x 10 signed, round(255 cos(2 pi i / 256))
  exp2 ROM: 256 x 16, round(32768 * 2^(i/256))
  palette : 3 x 1024 x 8 (Y >> 2, Cr >> 2, Cb >> 2; hardware convention, luma pre-scaled 16/13, cap 940):
            0..255 violet [band 3][mauve 5], 256..319 red/pink, 320..383 yellow/orange, 384..447 lens depth,
            448 black, 449 cream, 450 cyan, 451 rim
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# zone half-width (virtual px) vs distance (lines) from the centre line
D_UP = [0, 60, 90, 100, 110, 125, 140, 160, 180, 200, 220, 240, 260, 300, 335, 360, 380, 395, 405, 415, 421, 425]
H_UP = [2000, 1500, 1100, 900, 560, 400, 330, 270, 210, 160, 120, 100, 90, 87, 86, 82, 74, 64, 54, 40, 26, 0]
D_LO = [0, 60, 90, 100, 130, 160, 200, 240, 300, 360, 380, 400, 412, 425]
H_LO = [2000, 1500, 1100, 900, 560, 420, 360, 320, 270, 220, 175, 110, 50, 0]
D_DI = [0, 100, 140, 170, 200, 230, 260, 290, 310, 330, 345]
H_DI = [2000, 2000, 1400, 900, 600, 470, 380, 300, 220, 110, 0]
# lens row phase vs distance from the lens mirror line
LD = [0, 326, 426, 512, 600, 700, 1100]
LP = [0, 1, 2, 3, 4, 5, 9.6]

# palette (RGB)
BLACK = (15, 6, 6)
CREAM = (206, 218, 149)
CYAN = (0, 235, 235)
MAUVE = (108, 42, 92)
RIMC = (205, 180, 175)
RED_R = 252


def interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def sstep(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a)))
    return t * t * (3 - 2 * t)


def rgb_to_hw10(r, g, b):
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    yy = 64 + y * 876 * 16 / 13          # pre-scaled for the shell's 13/16 output luma
    yy = min(940, round(yy))
    return int(yy), int(round(512 + cr * 896)), int(round(512 + cb * 896))


def violet(dv):
    rv = min(205, max(110, 144 + 0.28 * (dv - 270)))
    bv = min(250, max(150, 250 - 1.2 * max(0, 230 - dv)))
    return (rv, 0.0, bv)


def palette():
    """1024 RGB entries."""
    pal = [BLACK] * 1024
    for band in range(8):
        dv = 100 + 64 * band + 32
        v = violet(dv)
        for m in range(32):
            mk = 0.95 * max(0.0, 1 - m / 31.0) ** 1.5
            pal[band * 32 + m] = tuple(v[c] + (MAUVE[c] - v[c]) * mk for c in range(3))
    for p in range(64):
        pal[256 + p] = (RED_R, 0, p * 140 / 63.0)
    for i in range(64):
        f = i * 8 / 256.0
        gy = min(210, max(0, 205 - 95 * sstep(0.15, 0.9, f) + 60 * sstep(1.1, 1.6, f)))
        pal[320 + i] = (250, gy, 0)
    for q in range(64):
        dep = min(1.0, q / 42.5)
        pal[384 + q] = (8, 135 - 95 * dep, 250)
    pal[448] = BLACK
    pal[449] = CREAM
    pal[450] = tuple(0.9 * CYAN[c] + 0.1 * (8, 135, 250)[c] for c in range(3))
    pal[451] = tuple(0.75 * RIMC[c] + 0.25 * MAUVE[c] for c in range(3))
    return pal


def tables():
    zone = [0] * 1024
    for i in range(256):
        d = 2 * i
        zone[i] = min(4095, int(round(2 * interp(D_UP, H_UP, d))))
        zone[256 + i] = min(4095, int(round(2 * interp(D_LO, H_LO, d))))
        zone[512 + i] = min(4095, int(round(2 * interp(D_DI, H_DI, d))))
    lens = [int(round(127 * math.cos(2 * math.pi * interp(LD, LP, 2 * i)))) for i in range(512)]
    cosr = [int(round(255 * math.cos(2 * math.pi * i / 256))) for i in range(256)]
    exp2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    return zone, lens, cosr, exp2


def main():
    zone, lens, cosr, exp2 = tables()
    pal = palette()
    yuv = [rgb_to_hw10(*c) for c in pal]
    L = ["-- Generated by crucible.py: do not edit.  Zone / lens / cos / exp2 tables and the palette.",
         "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
         "package crucible_pkg is"]

    def arr(name, typ, n, w, vals, signed=False):
        L.append(f"    type {typ} is array (0 to {n - 1}) of std_logic_vector({w - 1} downto 0);")
        L.append(f"    constant {name} : {typ} := (")
        body = []
        for v in vals:
            if signed and v < 0:
                v += 1 << w
            body.append(f'"{v:0{w}b}"')
        for i in range(0, len(body), 8):
            L.append("        " + ", ".join(body[i:i + 8]) + ("," if i + 8 < len(body) else ""))
        L.append("    );")

    arr("C_CR_ZONE", "t_cr_zone", 1024, 12, zone)
    arr("C_CR_LENS", "t_cr_lens", 512, 8, lens, True)
    arr("C_CR_COS", "t_cr_cos", 256, 10, cosr, True)
    arr("C_CR_EXP2", "t_cr_exp2", 256, 16, exp2)
    arr("C_CR_PY", "t_cr_pal", 1024, 8, [min(255, y >> 2) for y, u, v in yuv])
    L.append("    constant C_CR_PU : t_cr_pal := (")
    body = [f'"{min(255, max(0, u >> 2)):08b}"' for y, u, v in yuv]
    for i in range(0, 1024, 8):
        L.append("        " + ", ".join(body[i:i + 8]) + ("," if i + 8 < 1024 else ""))
    L.append("    );")
    L.append("    constant C_CR_PV : t_cr_pal := (")
    body = [f'"{min(255, max(0, v >> 2)):08b}"' for y, u, v in yuv]
    for i in range(0, 1024, 8):
        L.append("        " + ", ".join(body[i:i + 8]) + ("," if i + 8 < 1024 else ""))
    L.append("    );")
    L.append("end package crucible_pkg;")
    with open(os.path.join(HERE, "crucible_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
