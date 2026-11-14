#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
AGATE v0.1.0 - build hook.  Writes agate_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.patchbay_work/rb_agate/imodel.py) imports the same tables from here.

Package contents:
  C_AG_EXP2   256 x 16   round(32768 * 2^(i/256))                     exponent maps of Scale and Bands
  C_AG_SINI   256 x 16   round(65535 * sin(2 pi i / 1024))            quarter sine, the source of the stage tables
                         (entry 256, the peak, is the constant 65535)
  C_AG_SHARP  512 x 32   {Y10, U9, V9, vein3, 0}: index = pal * 256 + entry.  The colouriser palette, 256 entries
                         per cycle, band edges 3 entries soft (chroma stored / 2).  vein (S9): bit 2 = dark (else
                         pale), bits 1..0 = coverage 0, 1/4, 1/2, 1 (an 8-entry trapezoid across every band edge).
  C_AG_SOFT   256 x 32   {Y10, U10, V10, 00}: index = pal * 128 + 64 * m + i.  m = 0: the palette blurred over
                         ~16 entries, i = entry / 4.  m = 1: the mean of the half-cycle window centred on entry 4 i.
  C_AG_VEIN   constants  the pale and the dark vein colour
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# bands: (width, RGB at the band's start, RGB at its end); ten bands per palette cycle, the fold shows five at a time
STONE = [
    (30, (12, 16, 70), (20, 40, 150)),       # deep blue
    (14, (30, 150, 200), (120, 220, 235)),   # pale teal fringe
    (34, (250, 236, 190), (240, 200, 120)),  # cream
    (30, (245, 150, 10), (225, 95, 0)),      # amber
    (10, (255, 215, 60), (255, 215, 60)),    # yellow fringe
    (36, (190, 40, 16), (120, 10, 24)),      # rust red
    (22, (60, 8, 50), (20, 6, 30)),          # dark maroon
    (12, (240, 240, 250), (200, 215, 250)),  # white line
    (38, (20, 110, 120), (10, 60, 90)),      # teal
    (30, (8, 10, 24), (10, 12, 50)),         # near black
]
NEON = [
    (30, (8, 4, 20), (40, 0, 90)),           # black violet
    (14, (130, 40, 255), (190, 110, 255)),   # violet fringe
    (34, (255, 20, 170), (255, 90, 200)),    # hot pink
    (30, (255, 240, 40), (230, 255, 60)),    # yellow
    (10, (255, 255, 255), (255, 255, 255)),  # white fringe
    (36, (20, 235, 60), (0, 170, 80)),       # green
    (22, (0, 60, 40), (0, 20, 30)),          # dark green
    (12, (60, 250, 255), (60, 250, 255)),    # cyan line
    (38, (20, 80, 255), (10, 30, 190)),      # blue
    (30, (6, 4, 30), (8, 4, 20)),            # near black
]
VEINW = 8                                    # vein width, palette entries
VEIN_PALE = (255, 250, 235)
VEIN_DARK = (14, 8, 12)

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)

def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(round(u)))), min(1000, max(24, int(round(v)))))

def yuv_of(rgb):
    return prescale(rgb_to_hw10(rgb))

def word(rgb):
    y, u, v = yuv_of(rgb)
    return (y << 22) | (u << 12) | (v << 2)

def sword(rgb, vein):
    y, u, v = yuv_of(rgb)
    return (y << 22) | ((u >> 1) << 13) | ((v >> 1) << 4) | (vein << 1)

VEINCOV = [1, 2, 3, 3, 3, 3, 2, 1]           # coverage code across a vein: 1/4, 1/2, 1

def luma_of(c):
    return 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]

def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))

def build_palette(bands):
    """256 RGB entries and the vein code per entry (a vein at every band edge: pale where the two neighbours
    are dark on average, else dark)."""
    rgb = [(0.0, 0.0, 0.0)] * 256
    vein = [0] * 256
    tot = sum(b[0] for b in bands)
    pos = 0.0
    edges = []
    for n, (w, c0, c1) in enumerate(bands):
        e0 = int(round(pos * 256 / tot)); pos += w; e1 = int(round(pos * 256 / tot))
        for i in range(e0, e1):
            rgb[i] = lerp(c0, c1, (i - e0) / float(max(1, e1 - e0 - 1)))
        nxt = bands[(n + 1) % len(bands)]
        edges.append((e1 % 256, 1 if (luma_of(c1) + luma_of(nxt[1])) / 2 < 105 else 2))
    for (e, v) in edges:
        for j in range(VEINW):
            vein[(e - VEINW // 2 + j) % 256] = (4 if v == 2 else 0) | VEINCOV[j]
    return rgb, vein

def cblur(rgb, n):
    """circular box blur over n entries, centred"""
    out = []
    for i in range(256):
        acc = [0.0, 0.0, 0.0]
        for j in range(n):
            c = rgb[(i - n // 2 + j) % 256]
            for q in range(3):
                acc[q] += c[q]
        out.append(tuple(a / n for a in acc))
    return out

def palettes():
    """per palette: (sharp 256 RGB, vein 256, soft 64 RGB, mean 64 RGB)"""
    out = []
    for bands in (STONE, NEON):
        raw, vein = build_palette(bands)
        sharp = cblur(raw, 3)
        soft = cblur(cblur(raw, 16), 8)[2::4]
        mean = cblur(raw, 128)[2::4]
        out.append((sharp, vein, soft, mean))
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def sini_rom():
    return [int(round(65535 * math.sin(2 * math.pi * i / 1024))) for i in range(256)]

def sharp_rom():
    out = []
    for (sharp, vein, soft, mean) in palettes():
        out += [sword(sharp[i], vein[i]) for i in range(256)]
    return out

def soft_rom():
    out = []
    for (sharp, vein, soft, mean) in palettes():
        out += [word(c) for c in soft] + [word(c) for c in mean]
    return out

def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def main():
    vp, vd = yuv_of(VEIN_PALE), yuv_of(VEIN_DARK)
    parts = ["-- Generated by agate.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package agate_pkg is",
             vhd_table("C_AG_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_AG_SINI", "unsigned", 256, 16, sini_rom()),
             vhd_table("C_AG_SHARP", "std_logic_vector", 512, 32, sharp_rom()),
             vhd_table("C_AG_SOFT", "std_logic_vector", 256, 32, soft_rom()),
             "    constant C_AG_VPY : integer := %d;" % vp[0],
             "    constant C_AG_VPU : integer := %d;" % vp[1],
             "    constant C_AG_VPV : integer := %d;" % vp[2],
             "    constant C_AG_VDY : integer := %d;" % vd[0],
             "    constant C_AG_VDU : integer := %d;" % vd[1],
             "    constant C_AG_VDV : integer := %d;" % vd[2],
             "end package agate_pkg;", ""]
    with open(os.path.join(HERE, "agate_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
