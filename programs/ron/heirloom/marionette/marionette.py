#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
MARIONETTE v0.1.0 - build hook.  Writes marionette_pkg.vhd next to this file (pure python: the builder runs it with
the system python3; .heirloom_work/rb_marionette/imodel.py imports the same tables).

Fitted to the clip WIN_20231003_00_48_14 at 1.5 s (see .heirloom_work/rb_marionette/notes.md).  Levels: one scalar
field per pixel, 4096 level units (lu) over the palette range -1.30 .. +1.10 (1 level = 1706.67 lu); the palette has
256 entries (lu >> 4) in two slope variants (rising: red fringes, falling: cyan / white fringes) and two families
(Day = the clip, Night = the sibling clip WIN_20231003_00_50_39).

Package contents:
  C_MN_PAL   1024 x 32  {Y10, U10 (Cr), V10 (Cb), 00}: index = night & variant (0 rise, 1 fall) & level >> 4
  C_MN_E1    256 x 16   {NECK8, DIP8}: profile entry i (signed tile phase t = (i + 0.5) / 256 - 0.5)
  C_MN_E2    256 x 16   {CROWN8 (16 lu units), CWT4}: the crown figure profile, heads-scaled index
  C_MN_LN    512 x 8    rim drop by distance (0..127 lines, 16 lu units) | smoothstep (64) | dA, dK, Dt by nu / 2 (64 each, 4 lu units)
  C_MN_EXP2  256 x 16   round(32768 * 2^(i / 256))
  C_MN_STRIP 16 x 16    the strips' power-up offsets {off8 (lines, signed), hs8 (signed, 0.16 tile / 127)}
Colours in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's 13/16
output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PMIN, PMAX, NL = -1.30, 1.10, 256
LU = 4096.0 / (PMAX - PMIN)                # lu per level

DAY_BASE = [
    (-1.30, 10, 245, 0), (-0.95, 0, 205, 0), (-0.76, 0, 183, 0), (-0.64, 0, 172, 0),
    (-0.51, 5, 140, 3), (-0.43, 3, 66, 1), (-0.40, 2, 58, 0), (-0.36, 1, 50, 0), (-0.33, 3, 38, 1), (-0.30, 4, 24, 4),
    (-0.267, 6, 8, 9), (-0.24, 9, 7, 16), (-0.21, 17, 6, 59), (-0.18, 30, 5, 120), (-0.15, 47, 5, 180),
    (-0.12, 69, 8, 231), (-0.09, 97, 0, 245), (-0.06, 125, 0, 252),
    (0.02, 200, 0, 248), (0.15, 214, 0, 255), (0.35, 240, 0, 255), (0.55, 255, 0, 255), (0.62, 255, 0, 255),
    (0.74, 246, 96, 246), (0.86, 238, 170, 242), (0.98, 236, 226, 236), (1.10, 236, 236, 236),
]
DAY_RISE = [(0.62, 255, 0, 255), (0.665, 240, 0, 110), (0.70, 236, 30, 120), (0.74, 246, 96, 246),
            (-0.06, 125, 0, 252), (-0.045, 215, 0, 110), (-0.032, 250, 0, 10), (-0.016, 245, 0, 75), (0.00, 225, 0, 160),
            (0.02, 200, 0, 248),
            (-0.615, 0, 170, 0), (-0.602, 130, 80, 10), (-0.59, 160, 30, 0), (-0.578, 120, 70, 5), (-0.565, 2, 158, 1)]
DAY_FALL = [(0.62, 255, 0, 255), (0.665, 200, 110, 252), (0.70, 220, 150, 248), (0.74, 246, 96, 246),
            (-0.06, 125, 0, 252), (-0.045, 120, 120, 255), (-0.032, 150, 215, 250), (-0.016, 232, 226, 236),
            (0.00, 190, 110, 250), (0.02, 200, 0, 248),
            (-0.615, 0, 170, 0), (-0.602, 120, 210, 140), (-0.59, 178, 232, 205), (-0.578, 110, 205, 125), (-0.565, 2, 158, 1)]
NIGHT_BASE = [
    (-1.30, 0, 0, 6), (-1.00, 0, 14, 4), (-0.80, 0, 70, 8), (-0.62, 20, 200, 20), (-0.51, 40, 250, 40),
    (-0.43, 10, 130, 10), (-0.36, 0, 40, 6), (-0.30, 0, 6, 10), (-0.267, 0, 0, 14), (-0.21, 8, 10, 100),
    (-0.15, 30, 40, 210), (-0.09, 60, 70, 255), (-0.06, 90, 60, 255),
    (0.02, 150, 40, 255), (0.15, 200, 30, 250), (0.35, 240, 40, 250), (0.62, 255, 90, 250),
    (0.74, 250, 150, 250), (0.86, 240, 200, 250), (0.98, 220, 230, 255), (1.10, 230, 236, 255),
]
NIGHT_RISE = [(-0.06, 90, 60, 255), (-0.045, 230, 0, 120), (-0.032, 250, 10, 40), (-0.016, 240, 40, 120), (0.02, 150, 40, 255),
              (-0.615, 30, 230, 30), (-0.59, 230, 40, 40), (-0.565, 40, 250, 40)]
NIGHT_FALL = [(-0.06, 90, 60, 255), (-0.045, 60, 160, 255), (-0.032, 80, 230, 255), (-0.016, 150, 220, 255), (0.02, 150, 40, 255),
              (-0.615, 30, 230, 30), (-0.59, 120, 240, 255), (-0.565, 40, 250, 40)]
RANGES = [(-0.06, 0.02), (-0.615, -0.565), (0.62, 0.74)]

# profile shapes (per entry of the signed tile phase), vertical tables
RIM_E = [0, 30, 42, 92, 400]
RIM_L = [0, 0.62, 0.86, 1.85, 1.85]
NU_TAB = [0.0, 0.04, 0.15, 0.29, 0.43, 0.56, 0.70, 0.84, 0.98, 1.0]
A_TAB = [0.57, 0.57, 0.588, 0.562, 0.475, 0.36, 0.272, 0.237, 0.19, 0.19]
K_TAB = [0.967, 0.967, 1.03, 1.077, 1.03, 0.91, 0.817, 0.80, 0.75, 0.75]
D_NU = [0.0, 0.30, 0.41, 0.56, 0.70, 0.84, 0.98, 1.0]
D_TAB = [0, 0, 0.04, 0.10, 0.18, 0.25, 0.30, 0.30]
STRIP_OFF = [-26, 31, -12, 0, 22, -35, 9, 38, -18, 27, -5, -30, 14, 33, -22, 6]
STRIP_HS = [-61, 92, 25, -118, 70, -14, 109, -83, 41, -100, 7, 120, -47, 66, -9, -127]


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for k in range(1, len(xs)):
        if x <= xs[k]:
            a = (x - xs[k - 1]) / (xs[k] - xs[k - 1])
            return ys[k - 1] + a * (ys[k] - ys[k - 1])
    return ys[-1]


def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def pal_rgb(base, var):
    pts = {}
    for p in [q for q in base if not any(lo < q[0] < hi for lo, hi in RANGES)] + var:
        pts[round(p[0], 4)] = p[1:]
    ks = sorted(pts)
    out = []
    for j in range(NL):
        lv = PMIN + (j + 0.5) * (PMAX - PMIN) / NL
        out.append(tuple(interp(lv, ks, [pts[k][c] for k in ks]) for c in range(3)))
    return out


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def pack(rgb):
    y, u, v = rgb_to_hw10(rgb)
    y = 64 + (y - 64) * 16 / 13
    yi = max(64, min(940, int(round(y))))
    ui = max(4, min(1019, int(round(u))))
    vi = max(4, min(1019, int(round(v))))
    return (yi << 22) | (ui << 12) | (vi << 2)


def palette():
    """1024 words: night & variant & level"""
    out = []
    for base, rise, fall in ((DAY_BASE, DAY_RISE, DAY_FALL), (NIGHT_BASE, NIGHT_RISE, NIGHT_FALL)):
        for var in (rise, fall):
            out += [pack(c) for c in pal_rgb(base, var)]
    return out


def ti(i):
    return (i + 0.5) / 256 - 0.5


def neck8():
    return [int(round(255 * (1 - smooth((ti(i) + 0.23) / 0.35)))) for i in range(256)]


def dip8():
    out = []
    for i in range(256):
        fi = abs(2 * i - 255)
        g = 256 - fi
        out.append(int(round(255 * (1 - (g / 76.8) ** 2))) if g < 76.8 else 0)
    return out


def crown_tabs():
    cw, tc = 0.42, -0.24
    cr, wt = [], []
    for i in range(256):
        t = ti(i)
        if t < tc:
            sh = min(1.0, max(0.0, (t + cw) / (tc + cw)))
        else:
            sh = min(1.0, max(0.0, (cw - t) / (cw - tc)))
        lev = -1.30 + (0.84 + 1.30) * sh ** 0.6
        cr.append(int(round((lev + 1.30) * LU / 16)))
        wt.append(int(round(10 * (min(1.0, sh * 3) * 0.6 + 0.4))))
    return cr, wt


def line_rom():
    rim = [int(round(interp(e, RIM_E, RIM_L) * LU / 16)) for e in range(128)]
    sm = [int(round(255 * smooth(j / 63.0))) for j in range(64)]
    da, dk, dt = [], [], []
    for n in range(64):
        nu = (2 * n + 0.5) / 127.0
        da.append(int(round((0.588 - interp(nu, NU_TAB, A_TAB)) * LU / 4)))
        dk.append(int(round((1.03 - interp(nu, NU_TAB, K_TAB)) * LU / 4)))
        dt.append(int(round(interp(nu, D_NU, D_TAB) * LU / 4)))
    return [min(255, max(0, v)) for v in rim + sm + da + dk + dt] + [0] * 128


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def strip_init():
    return [((o & 255) << 8) | (h & 255) for o, h in zip(STRIP_OFF, STRIP_HS)]


def tables():
    cr, wt = crown_tabs()
    return dict(pal=palette(), e1=[(n << 8) | d for n, d in zip(neck8(), dip8())],
                e2=[(c << 4) | w for c, w in zip(cr, wt)], ln=line_rom(), exp2=exp2_rom(), strip=strip_init())


def vhdl_array(name, typ, vals, hexw):
    s = "    constant %s : %s := (\n" % (name, typ)
    items = ['x"%0*X"' % (hexw, v) for v in vals]
    for k in range(0, len(items), 8):
        s += "        " + ", ".join(items[k:k + 8]) + ("," if k + 8 < len(items) else "") + "\n"
    s += "    );\n"
    return s


def main():
    t = tables()
    s = "-- Generated by marionette.py: do not edit.\nlibrary ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n"
    s += "package marionette_pkg is\n"
    s += "    type t_mn_pal   is array (0 to 1023) of std_logic_vector(31 downto 0);\n"
    s += "    type t_mn_e16   is array (0 to 255) of std_logic_vector(15 downto 0);\n"
    s += "    type t_mn_ln    is array (0 to 511) of std_logic_vector(7 downto 0);\n"
    s += "    type t_mn_strip is array (0 to 15) of std_logic_vector(15 downto 0);\n"
    s += vhdl_array("C_MN_PAL", "t_mn_pal", t['pal'], 8)
    s += vhdl_array("C_MN_E1", "t_mn_e16", t['e1'], 4)
    s += vhdl_array("C_MN_E2", "t_mn_e16", t['e2'], 4)
    s += vhdl_array("C_MN_LN", "t_mn_ln", t['ln'], 2)
    s += vhdl_array("C_MN_EXP2", "t_mn_e16", t['exp2'], 4)
    s += vhdl_array("C_MN_STRIP", "t_mn_strip", t['strip'], 4)
    s += "end package marionette_pkg;\n"
    with open(os.path.join(HERE, "marionette_pkg.vhd"), "w") as f:
        f.write(s)


if __name__ == "__main__":
    main()
