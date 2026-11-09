#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ARCADE v0.1.0 - build hook.  Writes arcade_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  Fitted to the clip still WIN_20230825_00_00_20 at 6.5 s (see .heirloom_work/rb_arcade/notes.md).

The patch, as fitted (virtual 1920 x 1080 px; h = half virtual px):
  row A    circles on one H fold, period P = 948 px, centres x = 996 + k P, y = yA (64 in the still)
           dA = distance to the nearest centre, eA = to the far neighbour (sqrt via normalize + ROM)
           disc (dA < 368 px) = texture "globe", ring 368..462 (yellow -> red at 418 -> yellow), white rim,
           cyan halo, then blue arches to 791 px: dark rings every 56 px, cyan / blue weave from eA's rings
  row B    chevrons (H triangle + V ramp): hB = (y - yB) - 1.2 |x - midline|; a double orange band
           0..156 px, a navy line, then texture whose bands follow hB (nested chevrons)
  texture  a 32 x 32 tile (stripe phase x band phase, + band parity) of 8 colour classes: white ground,
           tan pointed lenses, cyan wavy bands, green / dark / navy stripe dashes, grey outlines

Package contents (all tables also read by .heirloom_work/rb_arcade/imodel.py):
  C_AR_SQA   1024 x 11  round(64 sqrt(m))                 (dA mantissa sqrt)
  C_AR_TILE  2048 x 3   texture class: index = parity * 1024 + sigma5 * 32 + beta5
  C_AR_EXP2   256 x 16  round(32768 * 2^(i/256))
  C_AR_SZ     256 x 12  P12 globe radius rd (h) by the glided P12 >> 2 (S = 0.35 .. 1 .. 2.0)
  C_AR_PAL    512 x 24  {Y8, U8, V8}: 0..127 ring / rim / halo (1 px per entry from rd), 128..210 chevron band
                        + navy, 224..231 texture classes, 232 tear line, 256..511 arches {psi3, phi5}
Colours in the hardware convention (U register = Cr, V register = Cb), luma pre-scaled 16/13 for the shell's
13/16 output scale and capped at 940 (stored >> 2).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)

def word8(rgb):
    y, u, v = rgb_to_hw10(rgb)
    y = min(940.0, 64 + (y - 64) * 16.0 / 13.0)
    yy = max(16, min(235, int(round(y / 4.0))))
    uu = max(1, min(255, int(round(u / 4.0))))
    vv = max(1, min(255, int(round(v / 4.0))))
    return (yy << 16) | (uu << 8) | vv

def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))

def interp(stops, p):
    if p <= stops[0][0]: return stops[0][1]
    for i in range(1, len(stops)):
        if p <= stops[i][0]:
            t = (p - stops[i - 1][0]) / float(stops[i][0] - stops[i - 1][0])
            return lerp(stops[i - 1][1], stops[i][1], t)
    return stops[-1][1]

# ---------------------------------------------------------------- measured colours (still, RGB)
# ring + rim + halo vs virtual px from the circle centre (column x = 997 below the middle circle)
RING_STOPS = [(368, (240, 220, 160)), (374, (241, 199, 126)), (380, (247, 187, 36)), (392, (251, 152, 0)),
              (404, (255, 110, 0)), (410, (255, 83, 0)), (418, (255, 49, 0)), (426, (255, 60, 0)),
              (434, (255, 87, 0)), (446, (252, 123, 0)), (452, (249, 142, 0)), (458, (244, 170, 20)),
              (462, (238, 214, 190)), (468, (237, 239, 239)), (476, (202, 242, 243)), (482, (110, 228, 245)),
              (490, (30, 170, 255)), (502, (0, 120, 252))]
# chevron band vs px below its outer edge (midline column x = 526)
CHEV_STOPS = [(0, (238, 210, 175)), (6, (250, 197, 90)), (12, (247, 178, 20)), (18, (249, 162, 1)),
              (24, (249, 147, 0)), (30, (251, 132, 0)), (36, (254, 115, 0)), (42, (255, 83, 0)),
              (48, (255, 84, 0)), (54, (253, 94, 0)), (60, (252, 111, 0)), (66, (251, 128, 0)),
              (72, (247, 150, 0)), (78, (245, 167, 0)), (84, (243, 183, 40)), (90, (246, 185, 43)),
              (96, (246, 172, 0)), (102, (251, 151, 0)), (108, (254, 121, 0)), (114, (255, 98, 0)),
              (120, (255, 76, 0)), (126, (255, 79, 0)), (132, (255, 96, 0)), (138, (252, 118, 0)),
              (144, (250, 143, 0)), (150, (248, 187, 22)), (156, (232, 201, 120)), (158, (90, 90, 215)),
              (166, (60, 60, 190))]
TEXPAL = [(232, 238, 236), (150, 228, 238), (40, 222, 245), (238, 192, 112),
          (72, 112, 10), (26, 38, 40), (52, 48, 160), (160, 170, 182)]
TEAR = (150, 236, 255)
BLUE, BCYAN, BDARK, BGREY = (0, 85, 250), (0, 205, 252), (14, 40, 60), (130, 158, 190)

C_WHITE, C_LCYAN, C_CYAN, C_TAN, C_GREEN, C_DARK, C_NAVY, C_GREY = range(8)

def tile():
    n = 32
    T = [0] * 2048
    for par in range(2):
        for i in range(n):
            s = (i + 0.5) / n - 0.5
            a = abs(s)
            for j in range(n):
                b = (j + 0.5) / n
                c = C_WHITE
                dc = abs(b - 0.74 - 0.05 * math.sin(2 * math.pi * (s + 0.25) * 2))
                cw = 0.085 * max(0.0, (a - 0.18) / 0.32)
                if par == 1: cw *= 0.5
                if dc < cw: c = C_CYAN
                elif dc < cw + 0.03 and a > 0.2: c = C_LCYAN
                lh = 0.19 * max(0.0, (a - 0.10) / 0.40) ** 0.55
                if abs(b - 0.28) < lh: c = C_TAN
                elif abs(b - 0.28) < lh + 0.025 and a > 0.13: c = C_GREY
                if a < 0.15 and (0.04 < b < 0.46 or (par == 0 and 0.60 < b < 0.90)):
                    c = C_GREEN if (par == 0) == (b < 0.5) else (C_DARK if par == 0 else C_NAVY)
                T[par * 1024 + i * 32 + j] = c
    return T

def sqa_rom():
    return [min(2047, int(round(64 * math.sqrt(m)))) for m in range(1024)]

def sqe_rom():
    return [min(255, int(round(16 * math.sqrt(m)))) for m in range(256)]

def warp_rom():
    out = []
    for i in range(512):
        r = 1.0 - i * (268.0 / 128.0) / 736.0          # dA / rd
        if r < 0: r = 0.0
        g = 1.0 / math.sqrt(max(0.08, min(1.0, 1 - (r / 1.08) ** 2))) - 1.0
        out.append(max(0, min(63, int(round(16 * g)))))
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def size_of(idx):
    """P12 glided >> 2 (0..255) -> globe scale S (0.35 .. 1 at 128 .. 2.0)"""
    if idx >= 128: return 1.0 + (idx - 128) / 127.0
    return 1.0 - (128 - idx) / 128.0 * 0.65

def sz_rom():
    return [int(round(736 * size_of(i))) for i in range(256)]

def pal_rom():
    out = [word8((0, 0, 0))] * 512
    for i in range(128):                                   # ring / rim / halo: 1 px per entry
        px = 368 + i
        out[i] = word8(interp(RING_STOPS, px))
    for i in range(83):                                    # chevron band: 2 px per entry (h >> 2)
        out[128 + i] = word8(interp(CHEV_STOPS, 2 * i + 1))
    for c in range(8):
        out[224 + c] = word8(TEXPAL[c])
    out[232] = word8(TEAR)
    for psi in range(8):
        cs = 0.5 + 0.5 * math.cos(2 * math.pi * (psi + 0.5) / 8)
        mix = max(0.0, min(1.0, (cs - 0.5) / 0.35))
        base = lerp(BLUE, BCYAN, mix); line = lerp(BDARK, BGREY, mix)
        for ph in range(32):
            f = min(ph, 32 - ph) / 32.0 + 1 / 64.0     # |phase| from the line centre, cycles
            lw = max(0.0, min(1.0, (f - 0.10) / 0.08))
            out[256 + psi * 32 + ph] = word8(lerp(line, base, lw))
    return out

def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = []
    for v in vals:
        items.append('"' + format(v, '0%db' % width) + '"' if width % 4 else 'x"%0*X"' % (width // 4, v))
    per = 8 if width % 4 else 16
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

TABLES = dict(SQA=sqa_rom, SQE=sqe_rom, TILE=tile, WARP=warp_rom, EXP2=exp2_rom, SZ=sz_rom, PAL=pal_rom)

def main():
    parts = ["-- Generated by arcade.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package arcade_pkg is",
             vhd_table("C_AR_SQA", 1024, 11, sqa_rom()),
             vhd_table("C_AR_TILE", 2048, 3, tile()),
             vhd_table("C_AR_EXP2", 256, 16, exp2_rom()),
             vhd_table("C_AR_SZ", 256, 12, sz_rom()),
             vhd_table("C_AR_PAL", 512, 24, pal_rom()),
             "end package arcade_pkg;", ""]
    with open(os.path.join(HERE, "arcade_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
