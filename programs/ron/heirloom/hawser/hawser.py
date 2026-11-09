#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
HAWSER v0.1.0 - build hook.  Writes hawser_pkg.vhd next to this file (pure python, system python3).
Heirloom from the video clips; still WIN_20231116_23_41_37 t3.5 (clip frame at 3.5 s).

The patch, as fitted (.heirloom_work/rb_hawser/notes.md), in virtual 1080-line coordinates:
  tubes : p1 = (x + 290) k1 (1 - a1 w1(y)),  k1 = 1/420 px,  w1 = (1 - cos 2 pi (y F1 + phi1)) / 2,  1/F1 = 1120 lines
  cords : p2 = (x + 146) k2 (1 + a2 w2(y)),  k2 = 1/304 px,  w2 the same at 2 F1 (its own phase)
          a1 = 0.153 (the photo), a2 = 0.366 a1: the tubes swing right, the cords counter-swing a little
  colour: the tube profile T(frac p1) keyed over the cord profile C(frac p2) over black; red glints on top
  glints: one cell per cord bundle and row (rows at 310 / 620 lines), a ragged diamond D = |dx| + 0.585 |dy|
          + zigzags - size, coloured in rings (red, salmon, pink, a white line, red, tinted red ...)

ROMs written (Y / U / V in the HARDWARE register convention: U = Cr, V = Cb; luma pre-scaled 16/13, cap 940):
  C_HW_TUBE  512 x 32  page (Amber / Lemon) x 256 tube-phase steps: Y8 U8 V8, key, core (Wire), 6 spare
  C_HW_CORD  1024 x 24 page (cords Amber / Lemon, Braid tubes Amber / Lemon) x 256 cord-phase steps: Y8 U8 V8
  C_HW_SEQ   512 x 16  0..255 exp2: 32768 2^(i/256); 256..511 quarter sine 32767 sin(pi/2 (i + 0.5)/256)
  C_HW_SBOX  256 x 16  glint cells: size (px, 8 ..118) << 8 | slant (2 bits) << 6 | 6 random bits
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# tube / cord colour profiles (RGB) in 100 phase bins, measured on the still (phase-aligned medians;
# .heirloom_work/rb_hawser/profx2.py; the tube's rim sharpened)
TUBE = [(246,178,37), (245,177,37), (245,177,41), (245,181,49), (244,188,62), (242,196,79), (241,204,95), (241,209,107), (242,204,109), (242,191,101), (229,175,86), (202,155,67), (148,120,50), (107,89,40), (80,66,40), (115,99,84), (150,132,128), (169,151,147), (159,143,141), (120,110,110), (80,76,74), (40,42,38), (22,30,21), (5,19,4), (5,19,4), (5,19,4), (5,19,4), (5,19,4), (5,19,4), (5,19,4), (6,19,5), (5,19,4), (5,19,3), (4,19,4), (5,19,4), (5,19,4), (22,27,5), (24,26,11), (21,26,15), (22,25,15), (20,24,15), (16,23,12), (12,22,11), (9,21,7), (6,20,6), (6,19,5), (5,19,4), (4,19,4), (4,19,3), (4,19,3), (4,19,3), (4,19,2), (4,19,2), (4,18,2), (4,18,2), (3,18,1), (3,18,2), (3,18,2), (3,18,2), (3,18,2), (3,18,2), (3,18,2), (3,18,3), (3,18,3), (3,18,3), (3,17,3), (3,18,3), (3,18,3), (4,18,4), (5,18,5), (5,19,5), (4,19,5), (5,19,5), (5,19,5), (4,19,4), (4,19,3), (4,19,3), (4,19,3), (4,19,4), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (8,17,4), (43,14,5), (121,49,8), (146,81,0), (137,80,0), (122,60,0), (133,58,0), (167,80,0), (197,109,0), (221,138,1), (234,167,24), (237,188,47), (240,199,61), (243,198,61), (246,191,52), (246,182,42)]
CORD = [(42,33,32), (34,31,30), (25,32,30), (18,28,25), (13,24,20), (9,22,14), (7,20,10), (6,19,7), (5,19,5), (4,19,4), (4,19,4), (4,19,4), (4,18,3), (4,18,3), (4,18,3), (4,18,4), (4,18,4), (4,18,4), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (4,18,3), (5,17,3), (7,17,4), (17,18,5), (32,18,7), (44,20,12), (43,27,21), (47,27,22), (48,28,21), (48,28,20), (50,32,21), (53,36,23), (60,39,23), (69,42,24), (80,50,28), (95,60,35), (115,73,46), (136,88,58), (159,104,72), (181,123,87), (203,144,104), (221,163,121), (232,175,129), (235,176,127), (233,169,116), (222,155,95), (203,136,68), (182,113,38), (156,89,11), (132,66,0), (116,47,0), (118,40,0), (132,42,0), (149,51,0), (167,63,0), (182,76,0), (195,92,0), (206,104,11), (213,110,22), (217,113,29), (220,116,38), (224,125,46), (219,133,52), (218,140,53), (219,143,62), (212,144,64), (196,140,61), (174,127,54), (152,110,39), (126,91,25), (103,74,12), (81,59,3), (66,46,0), (64,44,0), (74,49,9), (93,65,28), (118,87,55), (147,112,89), (173,138,123), (195,161,153), (203,177,174), (203,182,183), (194,176,181), (180,165,171), (160,149,154), (138,127,133), (115,109,116), (94,89,96), (82,70,75), (74,55,58), (71,42,41), (66,33,30), (59,26,22), (48,25,20)]
BLACK = (5, 19, 4)
LBLACK = (6, 1, 0)
KEY0, KEY1 = 0.85, 0.22          # tube opaque from 0.85 round to 0.22 of its period
CORE0, CORE1 = 0.95, 0.10        # the yellow core (Wire hollows it)
# lemon page (clip 23_43_35): luma ramp black / maroon / red / gold / lemon, neutral entries -> cool white
LEM_RAMP = [(0.00, (6,1,0)), (0.10, (57,6,2)), (0.20, (110,22,4)), (0.30, (150,40,2)), (0.42, (163,121,3)),
            (0.55, (202,173,3)), (0.68, (234,233,5)), (1.0, (250,250,60))]
LEM_WHITE = (198, 212, 206)


def luma(c):
    return 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]


def interp_periodic(tab, n=256):
    nb = len(tab)
    out = []
    for i in range(n):
        t = (i + 0.5) / n * nb - 0.5          # position in bin-centre units
        j = math.floor(t); f = t - j
        a = tab[j % nb]; b = tab[(j + 1) % nb]
        out.append(tuple(a[k] * (1 - f) + b[k] * f for k in range(3)))
    return out


def lerp_stops(stops, x):
    if x <= stops[0][0]: return stops[0][1]
    for (x0, c0), (x1, c1) in zip(stops, stops[1:]):
        if x <= x1:
            f = (x - x0) / (x1 - x0)
            return tuple(c0[k] * (1 - f) + c1[k] * f for k in range(3))
    return stops[-1][1]


def lemonize(tab):
    out = []
    for c in tab:
        L = luma(c) / 255.0
        sat = (max(c) - min(c)) / 255.0
        if L < 0.09:
            out.append(LBLACK)
        elif sat < 0.22 and L > 0.25:
            s = min(L / 0.70, 1.15)
            out.append(tuple(min(255.0, LEM_WHITE[k] * s) for k in range(3)))
        else:
            out.append(lerp_stops(LEM_RAMP, L * 1.12))
    return out


def profiles():
    tube = interp_periodic(TUBE)
    tube = [BLACK if luma(c) < 22 else c for c in tube]
    cord = interp_periodic(CORD)
    cord = [tuple(min(255.0, v * 1.08) for v in c) for c in cord]
    cord = [BLACK if luma(c) < 24 else c for c in cord]
    return tube, cord


def keybits(i):
    t = (i + 0.5) / 256.0
    key = 1 if (t >= KEY0 or t < KEY1) else 0
    core = 1 if (t >= CORE0 or t < CORE1) else 0
    return key, core


def rgb_to_hw8(c):
    """RGB 0..255 -> hardware 8-bit (Y pre-scaled 16/13, cap 940 in 10 bits; U = Cr, V = Cb)."""
    r, g, b = [v / 255.0 for v in c]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772; cr = (r - y) / 1.402
    y10 = 64 + y * 876
    y10 = 64 + (y10 - 64) * 16.0 / 13.0
    y10 = min(940.0, y10)
    u10 = 512 + cr * 896; v10 = 512 + cb * 896
    cl = lambda v: max(0, min(255, int(round(v / 4.0))))
    return cl(y10), cl(u10), cl(v10)


def ycc_to_hw8(y, cb, cr):
    y10 = 64 + y * 876
    y10 = min(940.0, 64 + (y10 - 64) * 16.0 / 13.0)
    cl = lambda v: max(0, min(255, int(round(v / 4.0))))
    return cl(y10), cl(512 + cr * 896), cl(512 + cb * 896)


def tables():
    tube, cord = profiles()
    tubeL = lemonize(tube); cordL = lemonize(cord)
    # tube ROM: page 0 amber, 1 lemon
    trom = []
    for page in (tube, tubeL):
        for i in range(256):
            y, u, v = rgb_to_hw8(page[i])
            key, core = keybits(i)
            trom.append((y << 24) | (u << 16) | (v << 8) | (key << 7) | (core << 6))
    # cord ROM: 0 cords amber, 1 cords lemon, 2 braid tubes amber, 3 braid tubes lemon (black off the key)
    crom = []
    for page, blk in ((cord, None), (cordL, None), (tube, BLACK), (tubeL, LBLACK)):
        for i in range(256):
            c = page[i]
            if blk is not None and not keybits(i)[0]: c = blk
            y, u, v = rgb_to_hw8(c)
            crom.append((y << 16) | (u << 8) | v)
    seq = [int(round(32768 * 2 ** (i / 256.0))) & 0xFFFF for i in range(256)]
    seq += [int(round(32767 * math.sin(math.pi / 2 * (i + 0.5) / 256.0))) for i in range(256)]
    # glint cells: a fixed pseudo-random table (LCG), size 8 + h 110 / 255 px, slant 2 bits
    sbox = []
    s = 0x2F6B
    for i in range(256):
        s = (s * 1103515245 + 12345) & 0x7FFFFFFF
        h = (s >> 16) & 255
        s = (s * 1103515245 + 12345) & 0x7FFFFFFF
        sl = (s >> 16) & 3
        r6 = (s >> 20) & 63
        size = 8 + (h * 110) // 255
        sbox.append((size << 8) | (sl << 6) | r6)
    return trom, crom, seq, sbox


# glint ring colours: (G lo, G hi, Y or None = the strand's luma x 13/16 clamped, Cb, Cr) for Amber / Lemon
RINGS = [(0, 14, 0.30, -0.14, 0.36), (14, 20, 0.50, -0.11, 0.30), (20, 22, 0.70, -0.06, 0.19), (22, 24, 0.85, -0.01, 0.08),
         (24, 26, 0.70, -0.06, 0.19), (26, 48, 0.34, -0.14, 0.36), (48, 60, None, -0.12, 0.30), (60, 72, 0.32, -0.14, 0.36),
         (72, 4096, None, -0.12, 0.30)]
RINGS_L = [(0, 14, 0.32, -0.18, 0.30), (14, 20, 0.52, -0.15, 0.26), (20, 22, 0.75, -0.08, 0.10), (22, 24, 0.88, -0.02, 0.02),
           (24, 26, 0.75, -0.08, 0.10), (26, 48, 0.36, -0.18, 0.30), (48, 60, None, -0.16, 0.26), (60, 72, 0.34, -0.18, 0.30),
           (72, 4096, None, -0.16, 0.26)]


def ring_consts():
    out = []
    for tab in (RINGS, RINGS_L):
        row = []
        for lo, hi, y, cb, cr in tab:
            yy, u, v = ycc_to_hw8(0.0 if y is None else y, cb, cr)
            row.append((lo, hi, None if y is None else yy, u, v))
        out.append(row)
    return out


def rom(lines, name, typ, n, w, vals, hexw):
    lines.append("    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % (typ, n - 1, w - 1))
    lines.append("    constant %s : %s := (" % (name, typ))
    for i in range(0, n, 8):
        row = ", ".join('x"%0*x"' % (hexw, vals[j]) for j in range(i, i + 8))
        lines.append("        " + row + ("," if i + 8 < n else ""))
    lines.append("    );")


def main():
    trom, crom, seq, sbox = tables()
    L = ["-- Generated by hawser.py: do not edit.  Tube / cord colour profiles, exp2 + quarter sine, glint cells,",
         "-- glint ring colours.",
         "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
         "package hawser_pkg is"]
    rom(L, "C_HW_TUBE", "t_hw_tube", 512, 32, trom, 8)
    rom(L, "C_HW_CORD", "t_hw_cord", 1024, 24, crom, 6)
    rom(L, "C_HW_SEQ", "t_hw_seq", 512, 16, seq, 4)
    rom(L, "C_HW_SBOX", "t_hw_sbox", 256, 16, sbox, 4)
    rc = ring_consts()
    for pi, nm in ((0, "A"), (1, "L")):
        for k, (lo, hi, y, u, v) in enumerate(rc[pi]):
            L.append("    constant C_R%s%d_Y : integer := %d;  constant C_R%s%d_U : integer := %d;  constant C_R%s%d_V : integer := %d;"
                     % (nm, k, 0 if y is None else y, nm, k, u, nm, k, v))
    by, bu, bv = rgb_to_hw8(BLACK)
    L.append("    constant C_BLK_Y : integer := %d;  constant C_BLK_U : integer := %d;  constant C_BLK_V : integer := %d;" % (by, bu, bv))
    L.append("end package hawser_pkg;")
    with open(os.path.join(HERE, "hawser_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("hawser_pkg.vhd written (TUBE 512x32, CORD 1024x24, SEQ 512x16, SBOX 256x16, ring colours)")


if __name__ == "__main__":
    main()
