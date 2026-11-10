#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
GOTHIC v0.1.1 - build hook.  Writes gothic_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; imodel.py imports the same tables and constants).

Fitted to the still WIN_20230817_00_06_49 at 0.5 s (see .heirloom_work/rb_gothic/notes.md):

  F    = (-PH tri(u) + CX x + warp(x) + g(y) - flow) / P + Etch * luma        the arch field, in colouriser cycles
         u = (x - XP) / PH (arch phase), tri = 0 at the apex .. 1/2 at the trough
         g(y) = (S / w) sin(w (y - YZ)) + GOFF, a slow vertical sine near its peak (the gothic bow)
  f    = frac(F) -> frame bands: gap (the stripes show through), cyan line, ribbon body
  Q    = 0.658 * 2 tri + 0.321 u + 0.2405 cos(2 pi (Y - 0.381 + rise) / 2) - 0.068     red / blue key
  body = red waves where frac(Q) is in [0.70, 0.12) or [0.36, 0.44), else a blue leaf (core) with dark green flanks
  red  = sin^3 profile of  x N / 324 + wob(f) sin(2 pi (y / 117 + x / 1800 + t))
  bg   = green stripes: clip(0.596 cos(2 pi (y - 152 - roll) / 173) + 0.556)

Package contents:
  C_GT_EXP2  256 x 16    round(32768 * 2^(i/256))
  C_GT_SIN   1024 x 16   round(32767 sin(2 pi i / 1024)), signed
  C_GT_WS    256 x 8     round(127 sin(2 pi i / 256)), signed (red-wave wobble)
  C_GT_WARP  512 x 16    warp(x) / P0 in Q16 colouriser cycles, x = 4 i + 2 virtual px, signed
  C_GT_FR    256 x 32    frame word by f8: mode(2) env(6) leaf(6) dg(6) rbody(6) wob(6)
  C_GT_PAL   512 x 32    {Y, U, V, 00} by zone(3) & level(6): 0 green stripe, 1 cyan stripe (S7), 2 cyan line,
                         3 red wave, 4 blue leaf, 5 dark green
  C_GT_VT    256 x 14    P12 vault factor v (Q12) by p12 >> 2
  C_GT_SP    256 x 11    speed factor (Q8) by (|k4 - 512| - 8) >> 1
  C_GT_PRO   64 x 6      red line profile sin^3
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (virtual px / lines)
PH, XP, CX = 965.7, 1020.75, 0.2096
WARP = [(6.316, -21.100), (23.837, 8.425), (-34.352, 12.830)]
YZ, PV, SV, GOFF = -2150.0, 11950.0, 1.7158, -3129.0
P0 = 333.8
SP_Y, SP_PER, SP_A, SP_B = 152.0, 173.0, 0.596, 0.556
QA, QY0, QC = 0.2405, 0.381, -0.068

# ------------------------------------------------------------------ integer constants shared with the RTL
K_DU1 = 8687          # arch phase per h at x1 (Q24): 2^24 / (2 PH)
K_XP2 = 4083          # 2 * XP in h (2 * 2041.5)
K_RX = 6868           # CX * 8 * 4096: R (Q4 F-px, Q12 fraction) per h
K_PHT = 15451         # PH * 16: Kt = K_PHT * invP = the triangle's full swing in Q24 cycles (x 2)
K_INVP = 3141         # 2^20 / P0: cycles (Q24) per Q4 F-px
K_AG = 40042          # 4096 * SV * PV / (2 pi) / P0: Ag (Q12 cycles) at the still
K_OM = 359410         # 2^32 / PV: vertical phase per virtual line (Q32)
K_YZ = 2150           # -YZ
K_GOFF = -50064       # 16 * GOFF (Q4 F-px)
K_DWH = 233016        # 9 * 2^24 / 648: red-wave phase per h at N = 9 (Q24)
K_WSH = 4660          # 2^24 / 3600: wobble phase per h (Q24)
K_PS = 379            # 2^24 / 173 / 256: stripe phase per Q8 line (Q24)
K_PS0 = 2036541       # stripe phase at line 0: frac(-152 / 173) (Q24)
K_QP = 30             # 2^24 / 2160 / 256: key phase per Q8 line (Q24) (30.34)
K_QP0 = 13581156      # key phase at line 0: frac(-0.381 / 2) (Q24)
K_QA = 985            # QA (Q12)
K_QC = -279           # QC (Q12)
K_TP = 14             # 2^16 / 18 / 256: tooth phase per Q8 line (Q16) (14.2)
K_WP = 560            # 2^24 / 117 / 256: wobble phase per Q8 line (Q24)
K_TEAR_Y = 260        # slipped band: +260 virtual lines, +90 virtual px (180 h), 340 lines tall
K_TEAR_X = 180
K_TEAR_H = 340


def smooth(e0, e1, x):
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3 - 2 * t)


def band(f, lo, hi, soft):
    d = (f - lo) % 1.0
    w = (hi - lo) % 1.0
    if d >= w:
        return 0.0
    return smooth(0, soft, d) * smooth(0, soft, w - d)


def warp_px(x):
    return sum(a * math.cos(2 * math.pi * (i + 1) * x / 1920) + b * math.sin(2 * math.pi * (i + 1) * x / 1920)
               for i, (a, b) in enumerate(WARP))


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(32767 * math.sin(2 * math.pi * i / 1024))) for i in range(1024)]


def ws_rom():
    return [int(round(127 * math.sin(2 * math.pi * i / 256))) for i in range(256)]


def warp_rom():
    out = []
    for i in range(512):
        x = min(i, 479) * 4 + 2
        out.append(int(round(65536 * warp_px(x) / P0)))
    return out


def frame_fields(f):
    """(mode, env, leaf, dg, rbody, wob) for f in [0, 1)."""
    gap = band(f, 0.42, 0.77, 0.02)
    dc = ((f - 0.79 + 0.5) % 1.0) - 0.5
    cyan = math.exp(-(dc / 0.024) ** 2)
    body = band(f, 0.79, 0.42, 0.02)
    rbody = band(f, 0.81, 0.40, 0.03)
    fl = min(max(((f - 0.85) % 1.0) / 0.38, 0.0), 1.0)
    if fl < 0.22:
        leaf = smooth(0, 0.22, fl)
    else:
        leaf = smooth(1.0, 0.45, fl) * 0.8 + (0.2 if fl < 1 else 0.0)
    dg = 0.8 * math.sin(math.pi * min(((f - 0.81) % 1.0) / 0.06, 1.0)) \
        + 0.6 * math.sin(math.pi * min(((f - 0.24) % 1.0) / 0.14, 1.0))
    wob = 0.85 * (0.45 + 1.1 * math.sin(math.pi * min(((f - 0.79) % 1.0) / 0.63, 1.0)))
    if cyan > 0.3:
        mode, env = 1, cyan
    elif 0.40 <= f < 0.78:
        mode, env = 0, gap
    else:
        mode, env = 2, body
    q = lambda v: max(0, min(63, int(round(63 * v))))
    return mode, q(env), q(leaf), q(dg), q(rbody), max(0, min(63, int(round(wob / 1.95 * 63))))


def frame_rom():
    out = []
    for i in range(256):
        mode, env, leaf, dg, rb, wob = frame_fields((i + 0.5) / 256)
        out.append((mode << 30) | (env << 24) | (leaf << 18) | (dg << 12) | (rb << 6) | wob)
    return out


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    y10 = 64 + y * 876
    y10 = 64 + (y10 - 64) * 16 / 13          # pre-scale for the 13/16 output stage
    return (int(round(min(y10, 940))), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def pal_rgb(zone, lv):
    t = lv / 63.0
    blk = (6, 8, 6)
    if zone == 0:                      # green stripe
        return lerp(blk, (0, 240, 4), t)
    if zone == 1:                      # cyan-cored stripe (S7)
        g = lerp(blk, (0, 240, 4), t)
        sc = min(max((t - 0.55) / 0.45, 0.0), 1.0)
        return lerp(g, (90, 240, 235), sc)
    if zone == 2:                      # cyan line
        return lerp(blk, (70, 215, 210), t)
    if zone == 3:                      # red wave
        return lerp((10, 8, 5), (196, 24, 8), t)
    if zone == 4:                      # blue leaf
        return lerp(blk, (10, 24, 215), t)
    if zone == 5:                      # dark green
        return lerp(blk, (5, 70, 13), t)
    return blk


def pal_rom():
    out = []
    for zone in range(8):
        for lv in range(64):
            y, u, v = rgb_to_hw10(pal_rgb(zone, lv))
            out.append((y << 22) | (u << 12) | (v << 2))
    return out


def vt_rom():
    out = []
    for i in range(256):
        p = 4 * i + 2
        v = 0.08 + 0.92 * (p / 400.0) ** 1.5 if p <= 400 else 1 + 2 * ((p - 400) / 623.0) ** 1.3
        out.append(int(round(4096 * v)))
    return out


def sp_rom():
    return [min(2047, int(round(256 * ((2 * i + 1) / 180.0) ** 1.5))) for i in range(256)]


def pro_rom():
    return [int(round(63 * math.sin(math.pi * (i + 0.5) / 64) ** 3)) for i in range(64)]


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
    per = 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by gothic.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package gothic_pkg is",
             vhd_table("C_GT_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_GT_SIN", "signed", 1024, 16, sin_rom(), signed=True),
             vhd_table("C_GT_WS", "signed", 256, 8, ws_rom(), signed=True),
             vhd_table("C_GT_WARP", "signed", 512, 16, warp_rom(), signed=True),
             vhd_table("C_GT_FR", "std_logic_vector", 256, 32, frame_rom()),
             vhd_table("C_GT_PAL", "std_logic_vector", 512, 32, pal_rom()),
             vhd_table("C_GT_VT", "unsigned", 256, 14, vt_rom()),
             vhd_table("C_GT_SP", "unsigned", 256, 11, sp_rom()),
             vhd_table("C_GT_PRO", "unsigned", 64, 6, pro_rom()),
             "end package gothic_pkg;", ""]
    with open(os.path.join(HERE, "gothic_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
