#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
HARLEQUIN v0.1.0 - build hook.  Writes harlequin_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  imodel.py (.heirloom_work/rb_harlequin) imports the same tables.

Fitted to the still WIN_20231218_00_10_10 t0.5 (see .heirloom_work/rb_harlequin/notes.md).  Coordinates: one
diamond period = 942 virtual px = 65536 phase units; the vertical cycle (main diamond row + seam + squashed band +
seam) = 1150 virtual lines, v = 0 at the main lemon's top tip.

Package contents:
  C_HQ_VT   512 x 32   vertical table.  0..287 (v = 4 n): hb [8:0], hg [17:9], hgl [26:18] (half-widths of the orange
                       core, the lemon and the glass in 1/1024 period), band [27] (keys centred half a period over),
                       seam strip [30:28].  288..359 (v = 16 k): F [11:0] stripe cycles per period * 256, DF [20:12]
                       the stripes' far displacement (1/1024 period).
  C_HQ_EX   512 x 8    E(i) = 255 (1 - exp(-8 i / 739)): the stripe kick after each core (tau = 170 px)
  C_HQ_EXP2 256 x 16   round(32768 * 2^(i/256))
  C_HQ_PAL  1024 x 32  {Y [29:20], U [19:10], V [9:0]}, index = bank [9], region [8:7], low [6:0]:
                       region 0 stripes  low = s[6:4] g[3:2] kg[1] ol[0]
                       region 1 core     low = r[6:2] kg[1] ol[0]
                       region 2 core with stripes (Fill = Stripes)  low = s[6:4] r2[3:2] kg[1] ol[0]
                       bank 1 = the RGB patch rotated (R <- G <- B <- R).
Colours in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 (cap 940) for the
shell's 13/16 output scale.
"""
import math, os

HERE = os.path.dirname(os.path.abspath(__file__))
PV = 1150
PX2U = 1024.0 / 942.0          # px -> 1/1024 period

BK_MAIN = [(-50,0),(-48,12),(-20,21),(0,37),(20,46),(40,51),(60,53),(80,56),(100,62),(120,71),(140,86),(160,99),(180,123),
           (200,144),(220,164),(240,190),(260,213),(280,232),(300,247),(320,252),(340,254),(360,253),(380,250),(400,246),
           (420,241),(440,233),(460,218),(480,195),(500,167),(520,136),(540,105),(560,79),(580,60),(600,45),(620,33),
           (640,24),(660,18),(680,18),(700,15),(720,10),(735,8),(740,0)]
GK_MAIN = [(0,0),(10,14),(20,49),(40,90),(60,115),(80,143),(100,166),(120,184),(140,197),(160,212),(180,221),(200,228),
           (220,230),(240,231),(260,227),(280,220),(300,213),(320,208),(340,203),(360,199),(380,197),(420,197),(460,199),
           (480,201),(500,200),(520,199),(540,194),(560,185),(580,174),(600,158),(620,139),(640,119),(660,94),(680,66),
           (700,40),(720,6),(730,0)]
GLASS = [(-130,0),(-20,163),(20,215),(60,255),(100,297),(140,326),(180,351),(220,371),(260,383),(300,393),(340,395),
         (380,390),(420,384),(460,372),(500,353),(540,330),(580,300),(620,268),(660,231),(700,191),(740,150),(780,104),
         (820,58),(840,33),(855,0)]
BK_BAND = [(791,0),(792,20),(825,30),(850,50),(860,70),(880,105),(900,146),(920,163),(940,174),(960,141),(980,97),
           (1000,74),(1015,40),(1030,0)]
GK_BAND = [(825,0),(850,25),(860,41),(880,73),(900,110),(920,146),(940,189),(960,175),(980,158),(1000,133),(1020,102),
           (1030,89),(1075,0)]
BAND_V = (792, 1072)           # rows whose keys sit half a period over
PER = [(-40,86),(100,78),(250,76),(330,66),(380,64),(430,72),(480,80),(560,98),(640,103),(720,106),(790,100),(850,95),
       (950,90),(1050,88),(1110,88)]
DFAR = [(-40,0),(100,10),(200,40),(280,90),(350,130),(420,120),(480,90),(560,40),(640,10),(720,0),(820,0),(900,60),
        (940,80),(1000,40),(1060,0)]
SEAMS = [(728, 760), (760, 792), (1040, 1072), (1072, 1100)]   # strip codes 1..4

def interp(anch, v):
    best = 0.0
    for sh in (-PV, 0, PV):
        xs = [a[0] + sh for a in anch]
        if xs[0] <= v <= xs[-1]:
            for i in range(len(anch) - 1):
                if xs[i] <= v <= xs[i + 1]:
                    t = 0.0 if xs[i + 1] == xs[i] else (v - xs[i]) / float(xs[i + 1] - xs[i])
                    best = max(best, anch[i][1] + t * (anch[i + 1][1] - anch[i][1]))
                    break
    return best

def interp_wrap(anch, v):
    xs = [a[0] - PV for a in anch] + [a[0] for a in anch] + [a[0] + PV for a in anch]
    ys = [a[1] for a in anch] * 3
    for i in range(len(xs) - 1):
        if xs[i] <= v <= xs[i + 1]:
            t = 0.0 if xs[i + 1] == xs[i] else (v - xs[i]) / float(xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[0]

def smooth(arr, sig):
    n = len(arr); r = 3 * sig
    k = [math.exp(-0.5 * (j / float(sig)) ** 2) for j in range(-r, r + 1)]
    s = sum(k)
    return [sum(k[j + r] * arr[(i + j) % n] for j in range(-r, r + 1)) / s for i in range(n)]

def profiles():
    """1-line resolution tables over the cycle (px), as the float proto uses them."""
    bkm = smooth([interp(BK_MAIN, v) for v in range(PV)], 4)
    bkb = smooth([interp(BK_BAND, v) for v in range(PV)], 4)
    gkm = smooth([interp(GK_MAIN, v) for v in range(PV)], 4)
    gkb = smooth([interp(GK_BAND, v) for v in range(PV)], 4)
    gl = smooth([interp(GLASS, v) for v in range(PV)], 6)
    per = smooth([interp_wrap(PER, v) for v in range(PV)], 30)
    df = smooth([interp_wrap(DFAR, v) for v in range(PV)], 30)
    band = [1 if BAND_V[0] <= v < BAND_V[1] else 0 for v in range(PV)]
    seam = [0] * PV
    for k, (a, b) in enumerate(SEAMS):
        for v in range(a, b):
            seam[v] = k + 1
    hb = [bkb[v] if band[v] else bkm[v] for v in range(PV)]
    hg = [gkb[v] if band[v] else gkm[v] for v in range(PV)]
    return dict(hb=hb, hg=hg, gl=gl, per=per, df=df, band=band, seam=seam)

def vt_rom():
    p = profiles()
    out = [0] * 512
    for n in range(288):
        v = 4 * n
        if v >= PV:
            v = PV - 1
        hb = min(511, int(round(p['hb'][v] * PX2U)))
        hg = min(511, int(round(p['hg'][v] * PX2U)))
        gl = min(511, int(round(p['gl'][v] * PX2U)))
        out[n] = hb | (hg << 9) | (gl << 18) | (p['band'][v] << 27) | (p['seam'][v] << 28)
    for k in range(72):
        v = min(PV - 1, 16 * k)
        f = int(round(942.0 / p['per'][v] * 256))
        df = min(511, int(round(p['df'][v] * PX2U)))
        out[288 + k] = f | (df << 12)
    return out

def ex_rom():
    return [int(round(255 * (1 - math.exp(-8.0 * i / 739.0)))) for i in range(512)]

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772; cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def entry_rgb(region, low):
    kg = (low >> 1) & 1; ol = low & 1
    if region == 1:
        r = (low >> 2) & 31
        R = min(252.0, 142 + r * 110.0 / 31); G = 140.0 if kg else 92.0; B = 0.0
    else:
        s = (low >> 4) & 7; sf = s / 7.0
        if region == 2:
            r2 = (low >> 2) & 3
            R = min(252.0, 160 + r2 * 30.0); G = 140.0 if kg else 92.0; B = 205.0 * sf
            R = R - 70 * sf
        else:
            g = (low >> 2) & 3; gf = g / 3.0
            R = 90.0 if kg else 127.0 - 30 * gf
            G = 140.0 if kg else 9.0 + 82 * gf
            B = 238.0 * sf * (0.95 if kg else 1.0 - 0.03 * gf)
            R += 7.0 * (1 - abs(2 * sf - 1))          # the purple fringe between violet and red
    if ol:
        G = G - 50 if kg else G + 85                  # the lemon's outline: dark inside, bright outside
    return (clamp(R, 0, 255), clamp(G, 0, 255), clamp(B, 0, 255))

def pal_rom():
    out = [0] * 1024
    for bank in (0, 1):
        for region in range(4):
            for low in range(128):
                R, G, B = entry_rgb(region if region < 3 else 0, low)
                if bank:
                    R, G, B = G, B, R
                y, u, v = rgb_to_hw10((R, G, B))
                y = min(940, int(round(64 + (y - 64) * 16.0 / 13.0)))
                u = clamp(u, 4, 1019); v = clamp(v, 4, 1019)
                out[bank * 512 + region * 128 + low] = (y << 20) | (u << 10) | v
    return out

def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) if width % 4 == 0 else 'to_unsigned(%d, %d)' % (v, width) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    parts = ["-- Generated by harlequin.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package harlequin_pkg is",
             vhd_table("C_HQ_VT", "std_logic_vector", 512, 32, vt_rom()),
             vhd_table("C_HQ_EX", "unsigned", 512, 8, ex_rom()),
             vhd_table("C_HQ_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_HQ_PAL", "std_logic_vector", 1024, 32, pal_rom()),
             "end package harlequin_pkg;", ""]
    with open(os.path.join(HERE, "harlequin_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
