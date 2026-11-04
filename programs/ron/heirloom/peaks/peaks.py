#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PEAKS v0.1.0 - build hook.  Writes peaks_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).

Fitted to the clip WIN_20230210_01_00_21 at 3.0 s (see .heirloom_work/rb_peaks/notes.md): a sunset of two
yellow peaks over a sky of diagonal peach/magenta bands and a green/violet weave, reflected in rippled water.

  band   = a skewed saw (rise 7/8 of the cycle), 2.03 cycles per width, leaning -0.7 px/line; its palette index
           drops 1.4/frame going down and its key threshold rises 1.05/frame (peach gone by line ~210, the
           band itself by ~470)
  weave  = 8.4 cycles per width, leaning +0.6 px/line with a +-20 px wobble; index 0.24 - 0.24 cos, rising
           0.76/frame down the sky (green/violet at the top, violet/magenta at the horizon)
  sun    = a symmetric triangle VCO at 1.96 cycles per width (static): key where 0.8 (y - 395) > dist, the
           apex rounded (r = 60 px), a 25 px soft edge, a 130 px yellow core
  water  = from line 636 the sky's per-line values hold (S8 Hold) or run back (Mirror); each line is shifted
           by a sawtooth ripple (-6 .. 172 px, ~36 lines, jittered) and the reflected sun's core breathes
  slips  = common H-sync slips at lines 85 (+60, glided), 316 (-20), 408 (-20), 468 (-30), 576 (+47) with
           smeared rows (the oscillators pause for a stretch of the line)

Package contents (all tables also read by imodel.py):
  C_PK_BAND  512 x 24  {Y8, U8, V8} band palette, index = pal * 256 + i (i = Q8 band index: 1.0 = 256)
  C_PK_WEAVE 512 x 24  weave palette, index = pal * 256 + i (i = Q8 weave index)
  C_PK_SUN   512 x 24  sun colour by core depth, index = pal * 256 + clamp(key px, 0, 255)
  C_PK_WSIN  256 x 8   round(256 * 0.24 * (1 - cos(2 pi p/256)))  (0 .. 123)
  C_PK_SUND  256 x 10  round(sqrt((d * 978/512)^2 + 60^2) - 60), d = folded sun phase (0 at the trough)
  C_PK_EXP2  256 x 16  round(32768 * 2^(i/256)) for the exponential knobs (Bands, Curtains)
Colours are in the hardware convention (U register = Cr, V register = Cb), 8 bits each (10-bit >> 2); luma
pre-scaled 16/13 for the shell's 13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PAL = [
 dict(band=[(0.46,(150,90,125)),(0.52,(255,25,120)),(0.64,(255,30,120)),(0.70,(255,90,120)),(0.76,(255,135,115)),(0.90,(255,160,105)),(1.05,(255,178,105))],
      weave=[(0.0,(83,171,125)),(0.28,(85,150,123)),(0.40,(100,112,122)),(0.50,(95,75,119)),(0.60,(93,30,118)),(0.68,(90,12,135)),(0.76,(150,70,125)),(0.86,(235,50,120)),(0.95,(245,95,120)),(1.05,(255,140,110))],
      sun_edge=(255,180,105), sun_core=(255,212,102)),
 dict(band=[(0.46,(60,40,110)),(0.52,(40,60,190)),(0.64,(60,110,230)),(0.70,(120,170,240)),(0.76,(170,210,245)),(0.90,(215,235,250)),(1.05,(235,245,255))],
      weave=[(0.0,(40,140,130)),(0.28,(30,110,120)),(0.40,(30,60,100)),(0.50,(25,30,90)),(0.60,(40,20,110)),(0.68,(70,20,140)),(0.76,(60,50,170)),(0.86,(40,100,210)),(0.95,(90,150,230)),(1.05,(150,200,240))],
      sun_edge=(190,215,235), sun_core=(245,245,240)),
]

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)

def prescale8(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = min(940, max(64, y2))
    return (int(round(y2 / 4)), int(round(min(1000, max(24, u)) / 4)), int(round(min(1000, max(24, v)) / 4)))

def word8(yuv):
    y, u, v = prescale8(yuv)
    return (y << 16) | (u << 8) | v

def grad(stops, v):
    ps = [p for p, _ in stops]; cs = [c for _, c in stops]
    if v <= ps[0]: return cs[0]
    if v >= ps[-1]: return cs[-1]
    for i in range(len(ps) - 1):
        if ps[i] <= v < ps[i + 1]:
            t = (v - ps[i]) / (ps[i + 1] - ps[i])
            return tuple(cs[i][k] * (1 - t) + cs[i + 1][k] * t for k in range(3))
    return cs[-1]

def smoothstep(t):
    t = min(1.0, max(0.0, t)); return t * t * (3 - 2 * t)

def band_rom():
    out = []
    for p in PAL:
        for i in range(256): out.append(word8(rgb_to_hw10(grad(p['band'], i / 256.0))))
    return out

def weave_rom():
    out = []
    for p in PAL:
        for i in range(256): out.append(word8(rgb_to_hw10(grad(p['weave'], i / 256.0))))
    return out

def sun_rom():
    out = []
    for p in PAL:
        for i in range(256):
            t = smoothstep(i / 130.0)
            c = tuple(p['sun_edge'][k] * (1 - t) + p['sun_core'][k] * t for k in range(3))
            out.append(word8(rgb_to_hw10(c)))
    return out

def wsin_rom():
    return [int(round(256 * 0.24 * (1 - math.cos(2 * math.pi * i / 256.0)))) for i in range(256)]

def sund_rom():
    out = []
    for d in range(256):
        px = d * 978.0 / 512.0
        out.append(int(round(math.sqrt(px * px + 60.0 ** 2) - 60.0)))
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

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
    parts = ["-- Generated by peaks.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package peaks_pkg is",
             vhd_table("C_PK_BAND", "std_logic_vector", 512, 24, band_rom()),
             vhd_table("C_PK_WEAVE", "std_logic_vector", 512, 24, weave_rom()),
             vhd_table("C_PK_SUN", "std_logic_vector", 512, 24, sun_rom()),
             vhd_table("C_PK_WSIN", "unsigned", 256, 8, wsin_rom()),
             vhd_table("C_PK_SUND", "unsigned", 256, 10, sund_rom()),
             vhd_table("C_PK_EXP2", "unsigned", 256, 16, exp2_rom()),
             "end package peaks_pkg;", ""]
    with open(os.path.join(HERE, "peaks_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
