#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
APERTURE v0.1.0 - build hook.  Writes aperture_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  The integer model (.kinetic_work/rb_aperture/imodel.py) imports the same table functions.

Package contents:
  C_AP_EXP2  256 x 16   round(32768 * 2^(i/256))                      (pitch, rate)
  C_AP_SIN   256 x 16   round(1023 * sin(pi/2 * (i + 1/2) / 256))     the wave, one quarter cycle
  C_AP_TH0   256 x 16   the cell's base phase Spread * index (Q14 cycles), Spread 0.1 (Linear) / 0.2 (Radial)
                        cycle per cell; staggered rows count i + 1/2.  Address: Radial 1 & |2 i + odd| (4) & |j| (3)
                        -> max + min / 2 of (S/2 |2 i + odd|, S |j|); Linear 00 & (i + j) (5, two's complement) & odd
                        -> S/2 (2 (i + j) + odd)
  C_AP_PAL   512 x 32   per palette 32 words, hardware convention (U register = Cr), CENTRED:
                        {Y - 64 (10 bits), U / 4 - 128 (8 bits), V / 4 - 128 (8 bits)}:
                        0 .. 8   T2 rim: accent + (ground - accent) co / 8 (co = 0 .. 8; word 8 = the ground)
                        9        T2 glass: B;  10 .. 22 zero
                        23 .. 30 the eight blade tones (A shaded toward black and tinted toward the accent by the
                                 blade's angle to the light); 31 the seam colour
  pal_table() keeps the plain list per palette: ground, A, B, accent, seam, tones 0 .. 7 (raw 10-bit Y, U, V)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

LIGHT = 225.0           # light direction in the cell frame (p right, q DOWN): 225 = from the upper left
SH0, SH1 = 0.42, 0.78   # blade shade = SH0 + SH1 * f, f = 0.5 + 0.5 cos(blade angle - light)
TINT = 0.6              # chroma tint toward the accent on the lit side
YCAP = 900

# The 16 shared Kinetic palettes (.kinetic_work/tools/kpalette.py): label, ground, A, B, accent
PALETTES = [
    ("Ink",        "#0E0E10", "#F2EFE6", "#8C8C8C", "#E23B2E"),
    ("Paper",      "#F2EFE6", "#141414", "#9A9A94", "#1F4E9E"),
    ("Bauhaus",    "#EDE4CF", "#D1362F", "#1F4E9E", "#F0B81C"),
    ("Vega",       "#0B1E5B", "#1BB5E8", "#D81E8C", "#F5D90A"),
    ("Outrun",     "#12002B", "#FF2E97", "#00D9F5", "#FFB000"),
    ("Ultra",      "#0D0221", "#7B2CBF", "#E0AAFF", "#3CF2C0"),
    ("Ocean",      "#03045E", "#0096C7", "#90E0EF", "#F4F1DE"),
    ("Ice",        "#06141B", "#9BD1E5", "#F1F7FA", "#4A7C9B"),
    ("Moss",       "#0F2A1D", "#7FB069", "#E6F0A3", "#F18F01"),
    ("Citrus",     "#14213D", "#FCA311", "#E5E5E5", "#E63946"),
    ("Terra",      "#2B1A12", "#C8553D", "#F2D0A4", "#588B8B"),
    ("Ember",      "#1A0A00", "#FF4500", "#FFB347", "#8C1C13"),
    ("Gold",       "#0A0A0A", "#D4A017", "#F6E7B0", "#7A5C00"),
    ("Candy",      "#FFF1E6", "#FF6B9D", "#56CBF9", "#FFD23F"),
    ("Mint",       "#F7F3E9", "#2EC4B6", "#FF6B6B", "#1D3557"),
    ("Plum",       "#2D1E2F", "#F7B2AD", "#9ABCA7", "#F2E94E"),
]


def hw10(h):
    """sRGB hex -> (y, u, v) 10-bit, hardware convention (u = Cr, v = Cb), BT.601, Y = 64 + 876 y."""
    h = h.lstrip("#")
    r, g, b = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def shade(c, s):
    return (64 + (c[0] - 64) * s, 512 + (c[1] - 512) * s, 512 + (c[2] - 512) * s)


def clampc(c):
    y, u, v = [int(round(x)) for x in c]
    return (max(64, min(YCAP, y)), max(4, min(1019, u)), max(4, min(1019, v)))


def pal_entries(p):
    """16 (y, u, v) words of one palette."""
    G, A, B, K = [hw10(c) for c in p[1:]]
    seam = shade(A, 0.14) if A[0] > 330 else shade(G, 0.6)
    tones = []
    for k in range(8):
        phi = k * 45.0 - 67.5          # blade k's sector centre at aperture 0 (p right, q down)
        f = 0.5 + 0.5 * math.cos(math.radians(phi - LIGHT))
        c = list(shade(A, SH0 + SH1 * f))
        tk = max(0.0, (f - 0.55) / 0.45) * TINT
        c[1] += (K[1] - c[1]) * tk
        c[2] += (K[2] - c[2]) * tk
        tones.append(clampc(c))
    words = [clampc(G), clampc(A), clampc(B), clampc(K), clampc(seam)] + tones + [(64, 512, 512)] * 3
    return words


def pal_table():
    return [pal_entries(p) for p in PALETTES]


S_LIN, S_RAD = 1638, 3276


def th0_rom():
    out = [0] * 256
    for a in range(16):
        for b in range(8):
            ask, asu = (S_RAD >> 1) * a, S_RAD * b
            mx, mn = (ask, asu) if ask >= asu else (asu, ask)
            out[128 + (a << 3) + b] = ((mx & 0x3FFF) + ((mn & 0x7FFF) >> 1)) & 0x3FFF
    for n in range(-16, 16):
        for odd in range(2):
            out[((n & 31) << 1) | odd] = ((S_LIN >> 1) * (2 * n + odd)) & 0x3FFF
    return out


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(1023 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]


def cen(c):
    y, u, v = c
    return ((y - 64) << 16) | ((((u >> 2) - 128) & 255) << 8) | (((v >> 2) - 128) & 255)


def centred(c):
    return (c[0] - 64, (c[1] >> 2) - 128, (c[2] >> 2) - 128)


def pack(c):
    return (c[0] << 16) | ((c[1] & 255) << 8) | (c[2] & 255)


def t2_words(p):
    """T2 per palette: 9 rim entries, the glass, then zero padding to 23 words (centred tuples)."""
    G, B, K = centred(p[0]), centred(p[2]), centred(p[3])
    rim = [tuple(K[i] + (((G[i] - K[i]) * co) >> 3) for i in range(3)) for co in range(9)]
    return rim + [B] + [(0, 0, 0)] * 13


def pal_rom():
    out = []
    for p in pal_table():
        words = [pack(c) for c in t2_words(p)]
        words += [pack(centred(t)) for t in p[5:13]] + [pack(centred(p[4]))]
        out += words
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
    parts = ["-- Generated by aperture.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package aperture_pkg is",
             vhd_table("C_AP_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_AP_SIN", "unsigned", 256, 16, sin_rom()),
             vhd_table("C_AP_TH0", "unsigned", 256, 16, th0_rom()),
             vhd_table("C_AP_PAL", "std_logic_vector", 512, 32, pal_rom()),
             "end package aperture_pkg;", ""]
    with open(os.path.join(HERE, "aperture_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
