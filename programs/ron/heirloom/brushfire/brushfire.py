#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BRUSHFIRE v0.1.0 - build hook.  Writes brushfire_pkg.vhd next to this file (pure python, system python3).
Heirloom from the video clips; still WIN_20230820_01_33_09 t0.5 (clip frame at 0.5 s).

The patch, as fitted (.heirloom_work/rb_brushfire/notes.md), in virtual 1080-line coordinates:
  tiles  : a = |x - axis| folded per tile (period 945 px = 2.03 tiles, axes at 540 / 1485), a_n = a / 472.5
  y'     : y S + O        (the breathing vertical ramp: S, O from P12 Blaze and the 5.8 s breath LFO)
  flame  : F = V(y') - B(y') H(a) - G by(y') bx(a - c(y'))
           V = y', B = 1 above the rim (y' < 300); below it V = 300 + 350 E, B = 1 + 0.65 E, E = e^((y'-300)/350) - 1
           H(a): the arch ramp (rises from the axis to the lobe column, falls toward the tile boundary)
           by, bx: the lobe bump (gaussians, sd 150 lines / 42 px), c = 268 + 0.22 (y' - 300) (tilted lobes)
  key    : flame where F < 300, a < wall(y') and above the floor (y' 470, tongues over 110 lines)
  colour : fire palette by (heat, fine phase F / 110 px); chevrons (F > 300, period 220 px) in the warm zone
           a < Wz(y'); bands (cool / warm palette by the band phase) elsewhere, pink diagonal stripes.

ROMs written (Y / U / V are the HARDWARE register convention: U = Cr, V = Cb; luma pre-scaled 16/13, cap 940):
  C_BF_H     512 x 16   i = a_n / 2: H (px, 0..1023) << 6 | heat_a (Q3, 0..13)
  C_BF_BX    512 x 8    j = |a_n - c| (a_n units of 472.5/1023 px): 255 exp(-(j 0.4619 / 42)^2)
  C_BF_FIRE  512 x 24   pages 0..6: heat 0, 0.5 .. 3 x 64 fine-phase steps; 448..503 chevron (q8 / 2);
                        504..511 the rim ramp (g / 8); Y8 U8 V8
  C_BF_BAND  256 x 24   warm bit x 128 band-phase steps; Y8 U8 V8
  C_BF_SEQ   512 x 16   0..255 exp2: 32768 2^(i/256); 256..511 quarter sine 32767 sin(pi/2 (i + 0.5)/256)
  C_BF_LN    1024 x 16  0..511 as C_BF_SEQ (the line engine's copy); 512..767: Wz (a_n/4) << 8 | wall (a_n/4)
                        over y' = 8 (i - 512) - 512; 768..1023: by = 255 exp(-((2 j) / 150)^2), j = |y' - 300| / 2
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

AN_PX = 472.5 / 1023.0          # px per a_n unit (at the photo's 2.03 tiles)
TH0 = 2000                      # breath phase at power-up, 2^16 per cycle

# ---------------------------------------------------------------- colours (RGB 0..255)
RED = (246, 45, 5); ORG = (240, 132, 11); YOR = (239, 183, 98); YEL = (242, 209, 6); WHT = (240, 238, 228)
PKW = (237, 207, 195); CYN = (150, 225, 238); GRY = (109, 126, 123); SKY = (70, 165, 232)

CYC = [
    [(0.0, WHT), (0.30, WHT), (0.42, (246, 230, 140)), (0.56, YEL), (0.72, (244, 160, 40)), (0.84, YOR), (0.90, CYN), (0.94, WHT)],
    [(0.0, RED), (0.08, ORG), (0.30, (244, 170, 30)), (0.46, YEL), (0.60, WHT), (0.70, PKW), (0.75, CYN), (0.79, PKW), (0.90, ORG)],
    [(0.0, RED), (0.14, (250, 80, 8)), (0.24, ORG), (0.50, YOR), (0.70, PKW), (0.86, ORG)],
    [(0.0, RED), (0.5, (250, 62, 6)), (0.9, RED)],
]
CHEV = [(0.0, RED), (0.10, (250, 80, 8)), (0.17, ORG), (0.24, SKY), (0.30, GRY), (0.37, RED)]
COOL = [(0.0, (28, 34, 28)), (0.045, (40, 50, 40)), (0.07, (222, 226, 234)), (0.11, (222, 226, 234)),
        (0.13, (92, 100, 14)), (0.24, (150, 160, 6)), (0.35, (210, 206, 6)), (0.40, (92, 108, 242)),
        (0.58, (110, 124, 242)), (0.78, (212, 216, 232)), (0.97, (226, 229, 236))]
WARM = [(0.0, (70, 28, 22)), (0.045, (95, 40, 30)), (0.07, (214, 165, 165)), (0.11, (214, 165, 165)),
        (0.13, (135, 55, 22)), (0.24, (182, 92, 8)), (0.35, (200, 112, 20)), (0.40, (150, 72, 178)),
        (0.58, (162, 82, 184)), (0.78, (206, 160, 172)), (0.97, (216, 170, 170))]
PINK = (236, 112, 168)


def lerp(c0, c1, t):
    t = min(1.0, max(0.0, t))
    return tuple(c0[k] * (1 - t) + c1[k] * t for k in range(3))


def ramp(t, keys):
    """cyclic piecewise-linear colour ramp over t in [0, 1)"""
    ts = [k[0] for k in keys] + [keys[0][0] + 1.0]
    cs = [k[1] for k in keys] + [keys[0][1]]
    t = t % 1.0
    for i in range(len(keys)):
        if ts[i] <= t < ts[i + 1]:
            return lerp(cs[i], cs[i + 1], (t - ts[i]) / (ts[i + 1] - ts[i]))
    return cs[0]


def fire_rgb(ph, heat):
    h0 = min(2, int(math.floor(heat)))
    hf = heat - h0
    return lerp(ramp(ph, CYC[h0]), ramp(ph, CYC[h0 + 1]), hf)


def rgb_to_hw8(c):
    """RGB -> hardware 8-bit (Y8, U8 = Cr, V8 = Cb); luma pre-scaled 16/13 and capped 940 (10-bit)."""
    r, g, b = [v / 255.0 for v in c]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    y10 = 64 + y * 876
    y10 = 64 + (y10 - 64) * 16.0 / 13.0
    y10 = min(940.0, y10)
    u10 = 512 + cr * 896
    v10 = 512 + cb * 896
    y8 = int(round(y10 / 4.0)); u8 = int(round(u10 / 4.0)); v8 = int(round(v10 / 4.0))
    return max(0, min(255, y8)), max(0, min(255, u8)), max(0, min(255, v8))


# ---------------------------------------------------------------- shapes
HK = [0, 60, 130, 200, 260, 300, 340, 380, 430, 472.5]
HV = [0, 135, 290, 440, 560, 630, 680, 700, 500, 100]


def smooth_interp(xq, xs, ys):
    """Catmull-Rom style cubic Hermite through the knots (gradient by central differences)"""
    n = len(xs)
    m = []
    for i in range(n):
        if i == 0:
            m.append((ys[1] - ys[0]) / (xs[1] - xs[0]))
        elif i == n - 1:
            m.append((ys[-1] - ys[-2]) / (xs[-1] - xs[-2]))
        else:
            hl = xs[i] - xs[i - 1]; hr = xs[i + 1] - xs[i]
            dl = (ys[i] - ys[i - 1]) / hl; dr = (ys[i + 1] - ys[i]) / hr
            m.append((hr * dl + hl * dr) / (hl + hr))   # numpy.gradient's second-order interior formula
    i = 0
    while i < n - 2 and xq > xs[i + 1]:
        i += 1
    x0, x1 = xs[i], xs[i + 1]
    t = min(1.0, max(0.0, (xq - x0) / (x1 - x0)))
    h = x1 - x0
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1; h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2; h11 = t ** 3 - t ** 2
    return h00 * ys[i] + h10 * h * m[i] + h01 * ys[i + 1] + h11 * h * m[i + 1]


def interp(xq, xs, ys):
    if xq <= xs[0]:
        return ys[0]
    for i in range(len(xs) - 1):
        if xq <= xs[i + 1]:
            return ys[i] + (ys[i + 1] - ys[i]) * (xq - xs[i]) / (xs[i + 1] - xs[i])
    return ys[-1]


WZ_Y = [0, 300, 600, 800, 1080, 1500]; WZ_V = [0, 220, 260, 240, 170, 0]       # warm-zone half-width, px
AW_Y = [-200, 0, 150, 300, 480]; AW_V = [472.5, 472.5, 380, 345, 335]           # flame wall half-width, px


def tables():
    h = []
    for i in range(512):
        a = (2 * i + 1) * AN_PX
        hv = int(round(smooth_interp(a, HK, HV)))
        hv = max(0, min(1023, hv))
        heat_a = int(round(min(1.0, max(0.0, (a - 30) / 150.0)) * 1.6 * 8))
        h.append((hv << 6) | heat_a)
    bx = [int(round(255 * math.exp(-((j * AN_PX) / 42.0) ** 2))) for j in range(512)]
    fire = []
    for p in range(7):
        heat = p * 0.5
        for k in range(64):
            ph = (k + 0.5) / 64.0
            fire.append(rgb_to_hw8(fire_rgb(ph, min(2.999, heat)) if heat < 3 else fire_rgb(ph, 2.999)))
    for k in range(56):            # chevron: q8 = 2k .. 2k+1 (opaque for q8 < 108)
        q = (2 * k + 1) / 256.0
        fire.append(rgb_to_hw8(ramp(min(q, 0.4199) / 0.42, [(c[0] / 0.42, c[1]) for c in CHEV])))
    for k in range(8):             # rim: g = 8k .. 8k + 7
        g = 8 * k + 4
        fire.append(rgb_to_hw8(lerp(RED, ORG, (g - 14) / 26.0)))
    band = []
    for w in range(2):
        for k in range(128):
            band.append(rgb_to_hw8(ramp((k + 0.5) / 128.0, WARM if w else COOL)))
    seq = [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]
    seq += [int(round(32767 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]
    ln = list(seq)
    for i in range(256):
        yp = 8 * i - 512 + 4
        wz = interp(yp, WZ_Y, WZ_V) if yp > 0 else 0.0
        aw = interp(yp, AW_Y, AW_V)
        wzn = min(255, int(round(wz / AN_PX / 4)))
        awn = min(255, int(round(aw / AN_PX / 4)))
        ln.append((wzn << 8) | awn)
    for j in range(256):
        ln.append(int(round(255 * math.exp(-((2 * j) / 150.0) ** 2))))
    return h, bx, fire, band, seq, ln


def rom(lines, name, typ, n, width, vals, hexw):
    lines.append("    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % (typ, n - 1, width - 1))
    lines.append("    constant %s : %s := (" % (name, typ))
    for i in range(0, n, 8):
        row = ", ".join('x"%0*x"' % (hexw, vals[j]) for j in range(i, i + 8))
        lines.append("        " + row + ("," if i + 8 < n else ""))
    lines.append("    );")


def main():
    h, bx, fire, band, seq, ln = tables()
    fire24 = [(y << 16) | (u << 8) | v for (y, u, v) in fire]
    band24 = [(y << 16) | (u << 8) | v for (y, u, v) in band]
    L = ["-- Generated by brushfire.py: do not edit.  Arch ramp + heat, lobe bump, fire / band palettes,",
         "-- exp2 + quarter sine, line tables (warm zone, flame wall, lobe gaussian).",
         "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
         "package brushfire_pkg is"]
    rom(L, "C_BF_H", "t_bf_h", 512, 16, h, 4)
    rom(L, "C_BF_BX", "t_bf_bx", 512, 8, bx, 2)
    rom(L, "C_BF_FIRE", "t_bf_fire", 512, 24, fire24, 6)
    rom(L, "C_BF_BAND", "t_bf_band", 256, 24, band24, 6)
    rom(L, "C_BF_SEQ", "t_bf_seq", 512, 16, seq, 4)
    rom(L, "C_BF_LN", "t_bf_ln", 1024, 16, ln, 4)
    th0 = TH0
    vm = (th0 + 16384) & 0xFFFF
    idx = (vm >> 6) & 255
    if vm & 0x4000: idx = 255 - idx
    cos0 = seq[256 + idx] * (-1 if vm & 0x8000 else 1)
    L.append("    constant C_TH0 : integer := %d;   -- breath phase at power-up (the still: 0.03 of the 5.8 s cycle)" % th0)
    L.append("    constant C_COS0 : integer := %d;  -- cos of it (quarter-sine ROM), Q15" % cos0)
    py, pu, pv = rgb_to_hw8(PINK)
    L.append("    constant PINK_Y8 : integer := %d;" % py)
    L.append("    constant PINK_U8 : integer := %d;" % pu)
    L.append("    constant PINK_V8 : integer := %d;" % pv)
    L.append("end package brushfire_pkg;")
    with open(os.path.join(HERE, "brushfire_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("brushfire_pkg.vhd written (H 512x16, BX 512x8, FIRE 512x24, BAND 256x24, SEQ 512x16, LN 1024x16)")


if __name__ == "__main__":
    main()
