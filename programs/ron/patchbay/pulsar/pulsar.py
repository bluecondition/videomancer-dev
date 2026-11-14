#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PULSAR v0.1.0 - build hook.  Writes pulsar_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  The same tables are read by the integer model (.patchbay_work/rb_pulsar/imodel.py).

Package contents:
  C_PU_EXP2  256 x 16  round(32768 * 2^(i/256))                  the exponential unit (all gains are octaves)
  C_PU_LOG   256 x 16  round(-1024 log2(i/256)), entry 0 = FFFF   attenuation of an 8-bit level
  C_PU_LENV  256 x 16  round(-1024 log2(env)), env = (10 + 245 (cos bump)^1.5) / 256: the burst envelope
  C_PU_SIN   256 x 8   round(127 sin(2 pi i / 256)), signed       the two VCOs (and the burst sway)
  C_PU_SHP   256 x 8   255 clip(1.6 ((1 + m) / 2)^3), m = (i-128)/127  waveshaper after the VCO mixer
  C_PU_PAL   256 x 32  {dY (11, signed), dU / 4 (8), dV / 4 (8), 00000}: colouriser colour minus the ground;
                       index = paper * 128 + saturation (0..3) * 32 + level (baseline .. peak)
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the
shell's 13/16 output scale, capped at 940 (drawn whites ~777).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# colouriser by height: baseline .. peak
PAL_KEYS = [(0.00, (40, 150, 255)), (0.12, (110, 90, 255)), (0.30, (255, 40, 210)), (0.52, (255, 60, 40)),
            (0.75, (255, 205, 30)), (1.00, (255, 255, 235))]
WHITE = (255, 255, 255)
PAPER = (236, 228, 206)
INK = (20, 20, 30)

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))

def prescale(yuv):
    y, u, v = yuv
    y2 = 64 + (y - 64) * 16.0 / 13.0
    y2 = int(round(min(940, max(64, y2))))
    return (y2, min(1000, max(24, int(u))), min(1000, max(24, int(v))))

def pal_rgb(n=32):
    out = []
    for i in range(n):
        t = i / (n - 1.0)
        for (t0, c0), (t1, c1) in zip(PAL_KEYS[:-1], PAL_KEYS[1:]):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0)
                out.append(tuple(c0[j] + f * (c1[j] - c0[j]) for j in range(3)))
                break
    return out

def consts():
    """grounds (Y, U, V), pre-scaled."""
    return dict(black=prescale(rgb_to_hw10((0, 0, 0))), paper=prescale(rgb_to_hw10(PAPER)))

def pal_cd():
    """256 entries (dY, dU / 4, dV / 4): colour - ground.  On black the unsaturated line is white; on paper it is dark
    ink and the colours are darkened to inks."""
    c = consts(); out = []
    for paper in (0, 1):
        bg = c['paper'] if paper else c['black']
        base = INK if paper else WHITE
        for s in range(4):
            f = s / 3.0
            for col in pal_rgb():
                if paper: col = tuple(0.42 * v for v in col)
                rgb = tuple(base[j] + f * (col[j] - base[j]) for j in range(3))
                y, u, v = prescale(rgb_to_hw10(rgb))
                out.append((y - bg[0], int(round((u - bg[1]) / 4.0)), int(round((v - bg[2]) / 4.0))))
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def log_rom():
    return [65535] + [int(round(-1024 * math.log2(i / 256.0))) for i in range(1, 256)]

def env_curve():
    out = []
    for i in range(256):
        x = 2 * (i + 0.5) / 256.0 - 1
        b = (0.5 * (1 + math.cos(math.pi * x))) ** 1.5
        out.append(10 + 245 * b)
    out[0] = 10.0; out[255] = 10.0
    return out

def lenv_rom():
    return [int(round(-1024 * math.log2(e / 256.0))) for e in env_curve()]

def sin_rom():
    return [int(round(127 * math.sin(2 * math.pi * i / 256.0))) for i in range(256)]

def shp_rom():
    out = []
    for i in range(256):
        m = max(-1.0, min(1.0, (i - 128) / 127.0))
        out.append(int(round(255 * min(1.0, 1.6 * ((1 + m) / 2) ** 3))))
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
    pal = []
    for (dy, du, dv) in pal_cd():
        assert -1024 <= dy < 1024 and -128 <= du < 128 and -128 <= dv < 128
        pal.append(((dy & 0x7FF) << 21) | ((du & 0xFF) << 13) | ((dv & 0xFF) << 5))
    c = consts()
    cl = []
    for nm in ('black', 'paper'):
        y, u, v = c[nm]
        cl.append("    constant C_PU_%s : std_logic_vector(29 downto 0) := \"%s\";" %
                  (nm.upper(), format((y << 20) | (u << 10) | v, '030b')))
    parts = ["-- Generated by pulsar.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package pulsar_pkg is",
             vhd_table("C_PU_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_PU_LOG", "unsigned", 256, 16, log_rom()),
             vhd_table("C_PU_LENV", "unsigned", 256, 16, lenv_rom()),
             vhd_table("C_PU_SIN", "signed", 256, 8, sin_rom(), signed=True),
             vhd_table("C_PU_SHP", "unsigned", 256, 8, shp_rom()),
             vhd_table("C_PU_PAL", "std_logic_vector", 256, 32, pal),
             "\n".join(cl),
             "end package pulsar_pkg;", ""]
    with open(os.path.join(HERE, "pulsar_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
