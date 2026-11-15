#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SNAKES v0.1.0 - build hook.  Writes snakes_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.kinetic_work/rb_snakes/imodel.py) imports the same table functions.

Package contents:
  C_SN_EXP2  256 x 16   round(32768 * 2^(i/256))                      (1 / pitch, rate)
  C_SN_SIN   1024 x 16  round(1023 * sin(pi/2 * (i + 1/2) / 1024))    the wave, one quarter cycle
  C_SN_RCP   1024 x 16  min(4095, round(2^22 / (1024 + i)))           reciprocal of the normalised larger |coordinate|
  C_SN_ATN   1024 x 16  round(atan((i + 1/2) / 1024) * 8192 / 2 pi)   angle within the octant (8192 per turn)
  C_SN_SEC   512 x 16   round((sqrt(1 + ((i + 1/2) / 512)^2) - 1) * 4096)   radius = larger * (1 + this / 4096)
  C_SN_PAL   256 x 32   {Y, U, V, 00}: per palette 8 words = page ground, then the four ring tones black, dark,
                        white, light (the luminance order of the illusion), hardware convention (U = Cr, V = Cb)

The four tones are the palette's own four colours sorted by luma and SHADED to fixed luma bands so the order
black < dark < light < white is exact on every palette: the darkest is scaled to luma <= 0.05, the lightest is
mixed toward white to luma >= 0.93, the two middle colours are brought into 0.22 .. 0.34 and 0.62 .. 0.74.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

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


def rgb01(h):
    h = h.lstrip("#")
    return [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]


def luma(c):
    return 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]


def to_band(c, lo, hi):
    """Shade c into the luma band [lo, hi]: scale toward black if too light, mix toward white if too dark."""
    l = luma(c)
    if l > hi:
        return [x * hi / l for x in c]
    if l < lo:
        t = (lo - l) / (1.0 - l)
        return [x + t * (1.0 - x) for x in c]
    return list(c)


def tones(p):
    """palette row -> [ground, black, dark, white, light] as rgb 0..1."""
    cols = sorted([rgb01(h) for h in p[1:]], key=luma)
    return [rgb01(p[1]), to_band(cols[0], 0.0, 0.05), to_band(cols[1], 0.22, 0.34),
            to_band(cols[3], 0.93, 1.0), to_band(cols[2], 0.62, 0.74)]


def hw10(c):
    """rgb 0..1 -> (y, u, v) 10-bit, hardware convention (u = Cr, v = Cb), BT.601, Y = 64 + 876 y."""
    r, g, b = c
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def pal_table():
    """16 x 5 list of (y, u, v): ground, black, dark, white, light."""
    return [[hw10(c) for c in tones(p)] for p in PALETTES]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(1023 * math.sin(math.pi / 2 * (i + 0.5) / 1024))) for i in range(1024)]


def rcp_rom():
    return [min(4095, int(round(2 ** 22 / (1024.0 + i)))) for i in range(1024)]


def atn_rom():
    return [int(round(math.atan((i + 0.5) / 1024.0) * 8192 / (2 * math.pi))) for i in range(1024)]


def sec_rom():
    return [int(round((math.sqrt(1 + ((i + 0.5) / 512.0) ** 2) - 1) * 4096)) for i in range(512)]


def pal_rom():
    out = []
    for p in pal_table():
        row = [(y << 22) | (u << 12) | (v << 2) for (y, u, v) in p]
        out += row + [0] * (8 - len(row))
    return out + [0] * (256 - len(out))      # padded to 256 words so it maps to block RAM


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by snakes.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package snakes_pkg is",
             vhd_table("C_SN_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_SN_SIN", "unsigned", 1024, 16, sin_rom()),
             vhd_table("C_SN_RCP", "unsigned", 1024, 16, rcp_rom()),
             vhd_table("C_SN_ATN", "unsigned", 1024, 16, atn_rom()),
             vhd_table("C_SN_SEC", "unsigned", 512, 16, sec_rom()),
             vhd_table("C_SN_PAL", "std_logic_vector", 256, 32, pal_rom()),
             "end package snakes_pkg;", ""]
    with open(os.path.join(HERE, "snakes_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
