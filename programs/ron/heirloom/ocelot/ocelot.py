#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
OCELOT v0.1.0 - build hook.  Writes ocelot_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the clip WIN_20231227_00_20_47 at 6.0 s (see .heirloom_work/rb_ocelot/notes.md):

  zebra    phi = Bands * PHIP(xs) + DPHI(xs) * Chevron * T(y): one chirped H oscillator (13 bands across, period 210 px
           at the left, 122 px at x 1150, 170 px at the right), dipped into "<" chevrons on the rows 252 and 780
           (T = (1 - d / 160)^1.35, d = lines from the cusp row); xs = x (1 - eps(y)) is the beam stretched by the
           body's LFO (eps = 0.235 max(0, Rv - 0.32))
           colour = ZEBRA(w, phi): orange band with a dark left ramp and a yellow right edge over salmon / grey
           horizontal stripes (195 lines, w = the stripe's pink weight)
  body     a rolling key: Rv(y) (triangle LFO, 1165 lines, rolling up 2.07 lines / field) + 0.31 W(theta) > 0.5 + kc,
           W = |sin|^0.6 scallops 455 px across; the zebra band pushes the key back into red flames
  dashes   inside the body, three glowing tan bars per pink half of the stripe oscillator, on the band phase
           (BLACK(d, phi)); blue crescents where phi is in 0.56 .. 0.94 and the W phase + 1.1 Rv falls in a window
           (BLUE(d, phi): the dash brightness colourised deep blue -> cyan)

Package contents (all tables also read by imodel.py):
  C_OC_PHI   1024 x 20 {PHIP Q8 (12 bits), DPHI Q8 (8 bits)}: index i <-> xs = 2 i + 1 virtual px
  C_OC_W     256 x 8   the scallop shape W(theta) = 255 |sin(pi theta)|^0.6
  C_OC_T     129 x 8   the chevron dip T(a), a = |cusp phase - 1/2| in 1/256 of 528 lines
  C_OC_PAL   1024 x 24 {Y8, Cr8, Cb8}: 0 w3 phi6 = zebra, 10 s2 b6 = dash glow (b = brightness, s = salmon core
             step), 11 xx b6 = blue crescents (the dash brightness colourised deep blue -> cyan)
  C_OC_GV    64 x 6    the bar glow down a bar, 0.38 lines per step (raised cosine, half-width 19 lines)
  C_OC_GH    64 x 6    the dash window over the band phase (0.29 .. 0.88 cycle)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# the zebra's grey-stripe row profile over one band cycle (64 bins, phase 0 = the orange band's left edge),
# measured on row 1000 of the still (four bands aligned on their left edges)
PG = [(214, 106, 0), (229, 113, 0), (236, 117, 0), (243, 119, 0), (247, 120, 0), (249, 121, 0), (250, 121, 0),
      (250, 121, 0), (250, 121, 0), (251, 120, 0), (252, 120, 0), (252, 119, 0), (253, 120, 0), (252, 120, 0),
      (253, 120, 0), (252, 121, 0), (252, 123, 0), (252, 126, 0), (251, 128, 0), (250, 132, 0), (245, 136, 0),
      (239, 142, 1), (230, 147, 6), (217, 151, 9), (200, 150, 12), (175, 144, 27), (149, 133, 34), (121, 119, 44),
      (100, 107, 51), (84, 97, 57), (74, 92, 61), (68, 88, 63), (65, 86, 65), (64, 85, 68), (65, 85, 70),
      (65, 86, 71), (65, 87, 72), (65, 88, 73), (64, 88, 74), (65, 88, 74), (64, 88, 75), (64, 88, 75),
      (64, 87, 75), (64, 87, 75), (64, 88, 75), (63, 87, 75), (62, 88, 75), (63, 88, 75), (63, 87, 75),
      (63, 87, 75), (63, 87, 75), (63, 87, 76), (62, 87, 76), (62, 87, 75), (63, 87, 75), (64, 87, 75),
      (66, 85, 71), (70, 83, 67), (75, 80, 55), (85, 76, 37), (103, 74, 20), (132, 79, 8), (166, 87, 2),
      (193, 98, 0)]
GREY = (62.0, 87.0, 74.0)
PINK = (255.0, 80.0, 60.0)
# plateau band left edges (virtual px, floor of the body's stretch) and the cusp-row phase shift (cycles)
XK = [-16.8, 170.9, 377.4, 555.3, 725.2, 867.5, 996.9, 1122.4, 1244.9, 1367.4, 1504.7, 1665.8, 1832.7]
DX = [0.0, 103.7, 259.2, 410.8, 582.2, 753.7, 902.3, 1040.9, 1176.5, 1312.1, 1450.6, 1613.1, 1785.6, 1914.2]
DY = [0.0, 0.37, 0.61, 0.84, 0.89, 0.81, 0.77, 0.74, 0.67, 0.59, 0.50, 0.33, 0.33, 0.30]
# dash glow: brightness -> colour (heat ramp), the salmon core, the blue crescent ramp, the flame red
HEAT = [(0.0, 7, 7, 0), (0.12, 40, 6, 0), (0.25, 80, 22, 0), (0.42, 125, 68, 4), (0.6, 170, 116, 52),
        (0.8, 205, 145, 88), (1.0, 228, 152, 100)]
SALM = (240.0, 128.0, 84.0)
BLUE = [(0.0, 2, 14, 205), (0.35, 1, 60, 245), (0.65, 6, 140, 245), (1.0, 22, 200, 236)]
RED = (226, 14, 0)
BLACK = (7, 7, 0)
BAR_HW = 19.0                # bar glow half-width (virtual lines)


def pchip(xk, yk, x):
    n = len(xk)
    h = [xk[i + 1] - xk[i] for i in range(n - 1)]
    d = [(yk[i + 1] - yk[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    for i in range(1, n - 1):
        if d[i - 1] * d[i] > 0:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])
    m[0] = d[0]
    m[-1] = d[-1]
    if x < xk[0]:
        return yk[0] + (x - xk[0]) * m[0]
    if x > xk[-1]:
        return yk[-1] + (x - xk[-1]) * m[-1]
    i = 0
    while i < n - 2 and x > xk[i + 1]:
        i += 1
    t = (x - xk[i]) / h[i]
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1
    h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2
    h11 = t ** 3 - t ** 2
    return h00 * yk[i] + h10 * h[i] * m[i] + h01 * yk[i + 1] + h11 * h[i] * m[i + 1]


def interp(tab, b):
    for k in range(len(tab) - 1):
        if b <= tab[k + 1][0]:
            t = (b - tab[k][0]) / (tab[k + 1][0] - tab[k][0])
            return tuple(tab[k][i] + t * (tab[k + 1][i] - tab[k][i]) for i in (1, 2, 3))
    return tuple(tab[-1][1:])


def band_presence(i):
    return min(1.0, max(0.0, (PG[i][0] - 64.0) / 189.0))


def zebra(w3, p6):
    g = PG[p6]
    bz = band_presence(p6)
    pk = [min(255.0, g[c] + (PINK[c] - GREY[c]) * (1 - bz)) for c in range(3)]
    pk[0] = max(pk[0], g[0])
    w = [0.0, 0.1875, 0.3125, 0.4375, 0.5625, 0.6875, 0.8125, 1.0][w3]
    return tuple(g[c] + w * (pk[c] - g[c]) for c in range(3))


def gv(d):
    return 0.5 + 0.5 * math.cos(math.pi * d / BAR_HW) if d < BAR_HW else 0.0


def gh(ph, a=0.29, b=0.42, c=0.80, e=0.88):
    up = min(1.0, max(0.0, (ph - a) / (b - a)))
    dn = min(1.0, max(0.0, (e - ph) / (e - c)))
    s = min(up, dn)
    return s * s * (3 - 2 * s)


def dash_col(sal, b):
    """glow colour at brightness b (0..1); sal = 0..3: how far right in the dash (the salmon core)"""
    b = min(1.0, b)
    col = interp(HEAT, b)
    s = sal / 3.0 * min(1.0, max(0.0, (b - 0.6) / 0.3))
    return tuple(col[c] + s * (SALM[c] - col[c]) for c in range(3))


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)


def pack24(rgb):
    y, u, v = rgb_to_hw10(rgb)
    y = 64 + (y - 64) * 16 / 13
    yi = max(16, min(235, int(round(y / 4))))
    ui = max(1, min(254, int(round(u / 4))))
    vi = max(1, min(254, int(round(v / 4))))
    return (yi << 16) | (ui << 8) | vi


def tables():
    phi = []
    off = -pchip(XK, list(range(len(XK))), 1.0) + 0.05       # keep PHIP >= 0 (only its fraction matters)
    for i in range(1024):
        xs = 2 * i + 1.0
        p = pchip(XK, list(range(len(XK))), xs) + off
        q = int(round(p * 256)) & 4095
        dp = pchip(DX, DY, min(xs, 1920.0))
        dq = max(0, min(255, int(round(dp * 256))))
        phi.append((q << 8) | dq)
    wt = [max(0, min(255, int(round(255 * abs(math.sin(math.pi * (i + 0.5) / 256)) ** 0.6)))) for i in range(256)]
    tt = []
    for a in range(129):
        d = a * 528.0 / 256.0
        tt.append(int(round(255 * max(0.0, 1 - d / 160.0) ** 1.35)))
    gvt = [int(round(63 * gv(i * 2 * 195.0 / 1024.0))) for i in range(64)]
    ght = [int(round(63 * gh((j + 0.5) / 64.0))) for j in range(64)]
    pal = []
    for a in range(1024):
        b = (a & 63) / 62.0
        if a < 512:
            pal.append(pack24(zebra(a >> 6, a & 63)))
        elif a < 768:
            pal.append(pack24(dash_col((a >> 6) & 3, b)))
        else:
            pal.append(pack24(interp(BLUE, min(1.0, b))))
    return dict(phi=phi, w=wt, t=tt, gv=gvt, gh=ght, pal=pal, red=pack24(RED), black=pack24(BLACK))


def vhdl_array(name, typ, vals, hexw):
    s = "    constant %s : %s := (\n" % (name, typ)
    items = ['x"%0*X"' % (hexw, v) for v in vals]
    for k in range(0, len(items), 8):
        s += "        " + ", ".join(items[k:k + 8]) + ("," if k + 8 < len(items) else "") + "\n"
    s += "    );\n"
    return s


def vhdl_bits(name, typ, vals, width):
    s = "    constant %s : %s := (\n" % (name, typ)
    items = ['"%s"' % format(v, '0%db' % width) for v in vals]
    for k in range(0, len(items), 8):
        s += "        " + ", ".join(items[k:k + 8]) + ("," if k + 8 < len(items) else "") + "\n"
    s += "    );\n"
    return s


def main():
    t = tables()
    s = "-- Generated by ocelot.py: do not edit.\nlibrary ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n"
    s += "package ocelot_pkg is\n"
    s += "    type t_oc_phi is array (0 to 1023) of std_logic_vector(19 downto 0);\n"
    s += "    type t_oc_w   is array (0 to 255) of std_logic_vector(7 downto 0);\n"
    s += "    type t_oc_t   is array (0 to 128) of std_logic_vector(7 downto 0);\n"
    s += "    type t_oc_pal is array (0 to 1023) of std_logic_vector(23 downto 0);\n"
    s += "    type t_oc_g   is array (0 to 63) of std_logic_vector(5 downto 0);\n"
    s += vhdl_array("C_OC_PHI", "t_oc_phi", t['phi'], 5)
    s += vhdl_array("C_OC_W", "t_oc_w", t['w'], 2)
    s += vhdl_array("C_OC_T", "t_oc_t", t['t'], 2)
    s += vhdl_array("C_OC_PAL", "t_oc_pal", t['pal'], 6)
    s += vhdl_bits("C_OC_GV", "t_oc_g", t['gv'], 6)
    s += vhdl_bits("C_OC_GH", "t_oc_g", t['gh'], 6)
    s += "    constant C_OC_RED   : std_logic_vector(23 downto 0) := x\"%06X\";\n" % t['red']
    s += "    constant C_OC_BLACK : std_logic_vector(23 downto 0) := x\"%06X\";\n" % t['black']
    s += "end package ocelot_pkg;\n"
    with open(os.path.join(HERE, "ocelot_pkg.vhd"), "w") as f:
        f.write(s)


if __name__ == "__main__":
    main()
