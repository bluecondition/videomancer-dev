#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
IRONWORK v0.1.1 - build hook.  Writes ironwork_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  Heirloom phase 3, from the still WIN_20230717_23_51_09 at 7.5 s
(see .heirloom_work/rb_ironwork/notes.md and proto7.py / imodel.py).

The patch, as fitted:
  u, v   = H triangle (tile 944 px, A axes at x 524 + 944 k) and V triangle (row 515 lines, troughs at y 497)
  r      = sqrt((u + d0)^2 + (1.8 v)^2)      an offset-circle field: lenses on the A axes, X crossings on the B
                                              axes, diamond eyes at the B lattice corners, gothic arches below
           (d0 = 236 px), read in rings of 205 px: phi = frac(r / 205 + 0.045 + lace)
  lace   = a fast H oscillator (stripes, 37.5 px, frequency x (1 - 0.075 cos(2 pi (y - yw) / 1024))) added to phi
           with depth 0.35 (1 - V), and used as a KEY: black where the stripe is below the zone's threshold
  zones  = phi in [0.10, 0.66): type X, else type Y; each type has its own colouriser offset per line (C_IW_VT,
           the photo's vertical colour score), ci = h(y) + slope(phi) + cycle;  colour = PAL[ci + 0.2 (S - 1/2)],
           threshold = THR[ci]

Package contents:
  C_IW_LROM  512 x 16   0..255: round(32768 * 2^(i/256)); 256 + i: low byte round(127 cos(2 pi i / 256)) (signed),
                        high byte the colour score hX (256 + i) / hY (320 + i) at virtual line 16.875 i
  C_IW_SQRT  1024 x 13  round(256 sqrt(m))
  C_IW_PAL   512 x 32   {Y, U, V, 00}: index = palette * 256 + ci  (hardware convention, U = Cr; luma pre-scaled 16/13)
  C_IW_THR   512 x 8    stripe-key threshold (0 = solid, 255 = black)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# zone palette along the colour index (one cycle), [pos, rgb, threshold]  -- fitted to the photo
ZS_IRON = [[0.00, (12, 18, 10), 1.0], [0.04, (40, 20, 16), 0.85], [0.08, (120, 24, 34), 0.4],
           [0.12, (186, 23, 37), 0.18], [0.16, (225, 70, 78), 0.13], [0.20, (240, 117, 128), 0.12],
           [0.32, (240, 117, 128), 0.12], [0.36, (180, 23, 138), 0.10], [0.42, (185, 52, 227), 0.12],
           [0.49, (38, 26, 157), 0.1], [0.55, (11, 181, 211), 0.04], [0.60, (4, 241, 14), 0.0],
           [0.67, (4, 241, 14), 0.0], [0.72, (140, 235, 50), 0.30], [0.79, (175, 240, 110), 0.50],
           [0.86, (110, 190, 60), 0.65], [0.92, (12, 18, 10), 1.0], [1.0, (12, 18, 10), 1.0]]
# "Verdigris": aged copper, gold and teal on black (same thresholds)
VERD = [(10, 14, 14), (60, 30, 16), (120, 50, 22), (170, 80, 30), (215, 120, 50), (235, 190, 90), (235, 190, 90),
        (150, 110, 40), (110, 120, 50), (20, 80, 80), (60, 190, 170), (40, 220, 170), (40, 220, 170),
        (170, 235, 210), (200, 240, 230), (30, 80, 70), (10, 14, 14), (10, 14, 14)]
ZS_VERD = [[z[0], VERD[i], z[2]] for i, z in enumerate(ZS_IRON)]

# vertical colour score [virtual line, hX, hY] (cycles)
VT = [[0, 0.84, 0.60], [60, 0.84, 0.60], [120, 0.82, 0.61], [180, 0.80, 0.62], [240, 0.82, 0.64],
      [300, 0.88, 0.70], [360, 1.16, 0.78], [420, 1.22, 0.80], [480, 1.24, 0.80], [540, 1.22, 0.80],
      [600, 1.28, 0.78], [660, 1.30, 0.66], [720, 1.27, 0.60], [780, 1.32, 0.58], [840, 1.22, 0.62],
      [900, 1.21, 0.82], [960, 1.22, 0.88], [1020, 1.21, 0.90], [1080, 1.21, 0.90]]

def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(len(xs) - 1):
        if x <= xs[i + 1]:
            t = (x - xs[i]) / float(xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)

def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, int(round(min(1000, max(24, u)))), int(round(min(1000, max(24, v)))))

def word(yuv):
    y, u, v = prescale(yuv)
    return (y << 22) | (u << 12) | (v << 2)

def zone_tables(zs):
    pos = [z[0] for z in zs]
    pal, thr = [], []
    for i in range(256):
        t = (i + 0.5) / 256.0
        rgb = tuple(interp(t, pos, [z[1][c] for z in zs]) for c in range(3))
        pal.append(word(rgb_to_hw10(rgb)))
        thr.append(int(round(min(255, max(0, 255 * interp(t, pos, [z[2] for z in zs]))))))
    return pal, thr

def pal_rom():
    a, _ = zone_tables(ZS_IRON)
    b, _ = zone_tables(ZS_VERD)
    return a + b

def thr_rom():
    _, a = zone_tables(ZS_IRON)
    return a + a

def black_word(p):
    return word(rgb_to_hw10(ZS_IRON[0][1] if p == 0 else VERD[0]))

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def sqrt_rom():
    return [int(round(256 * math.sqrt(m))) for m in range(1024)]

def cos_rom():
    return [int(round(127 * math.cos(2 * math.pi * i / 256.0))) for i in range(256)]

def vt_rom():
    out = []
    xs = [p[0] for p in VT]
    for i in range(64):
        y = i * 1080.0 / 64
        hx = int(round(interp(y, xs, [p[1] for p in VT]) * 256)) & 255
        hy = int(round(interp(y, xs, [p[2] for p in VT]) * 256)) & 255
        out.append((hx << 8) | hy)
    return out

def lrom():
    """512 x 16: 0..255 exp2; 256..511 low byte = cos (two's complement), high byte = colour score:
    256 + i = hX[i], 320 + i = hY[i] (i = 0..63), else 0"""
    ex = exp2_rom(); cs = cos_rom(); vt = vt_rom()
    out = list(ex)
    for i in range(256):
        hi = 0
        if i < 64:
            hi = vt[i] >> 8
        elif i < 128:
            hi = vt[i - 64] & 255
        out.append((hi << 8) | (cs[i] & 255))
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
    per = 8 if (signed or width % 4) else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    k0, k1 = black_word(0), black_word(1)
    parts = ["-- Generated by ironwork.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package ironwork_pkg is",
             '    constant C_IW_K0 : std_logic_vector(31 downto 0) := x"%08X";' % k0,
             '    constant C_IW_K1 : std_logic_vector(31 downto 0) := x"%08X";' % k1,
             vhd_table("C_IW_LROM", "unsigned", 512, 16, lrom()),
             vhd_table("C_IW_SQRT", "unsigned", 1024, 13, sqrt_rom()),
             vhd_table("C_IW_PAL", "std_logic_vector", 512, 32, pal_rom()),
             vhd_table("C_IW_THR", "unsigned", 512, 8, thr_rom()),
             "end package ironwork_pkg;", ""]
    with open(os.path.join(HERE, "ironwork_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
