#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BRAID v0.1.0 (Kinetic) - build hook.  Writes braid_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.kinetic_work/rb_braid/imodel.py) imports the same tables.

Package contents:
  C_BR_TAB   1024 x 16   0 .. 255:   round(32768 * 2^(i/256))                        (pitch, rate: exponential)
                         256 + 32 p + w: palette p (the 16 shared Kinetic palettes, hardware convention, U = Cr):
                                     w 0 .. 2: the ground Y, U, V (U and V quantised to steps of 4);
                                     w 3 + 9 s + 3 ch + (0 lo, 1 s1, 2 s2): strand colour s (A, B, accent),
                                     channel ch (Y, U, V): the shade ramp's dark end lo and its two steps (two's
                                     complement): k 0 .. 63 goes lo -> colour in steps of s1 / 64, k 64 .. 127
                                     colour -> highlight in steps of s2 / 64
                         768 + i:    round(65536 * 1.16 / (1 + sqrt(1 + 0.93 A^2))),  A = (i + 0.5) / 32
                                     (the largest strand radius / swing with no overlap where two strands swap
                                     depth order, 3 strands)
                         896 + i:    the tube profile by edge distance: e / rh = (i + 1/2) / 128 -> shade index
                                     0 .. 127 (read by the pixel path outside the vblank zone)
  C_BR_SQ     256 x 10   round(256 sqrt(1 + (a/64)^2)), capped 1023                   (tilted tube width factor g)
  C_BR_SIN   1024 x 12   round(4095 sin((i + 1/2) pi / 2048))                          (a quarter wave)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# The 16 shared Kinetic palettes (copied from .kinetic_work/tools/kpalette.py so this folder is self-contained)
PALETTES = [
    # label        ground     A          B          accent
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


def pal_table():
    """16 x 4 list of (y, u, v)."""
    return [[hw10(c) for c in p[1:]] for p in PALETTES]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sq_rom():
    return [min(1023, int(round(256 * math.sqrt(1 + (a / 64.0) ** 2)))) for a in range(256)]


def sin_rom():
    return [int(round(4095 * math.sin((i + 0.5) * math.pi / 2048.0))) for i in range(1024)]


def prof_rom():
    """q -> shade index.  Lambert across the tube (0.22 + 0.78 n^0.9, n = sqrt(1 - q^2)) mapped onto the dark ->
    full ramp (0.15 .. 1.0 over 0 .. 64), plus a highlight n^14 on the full -> highlight ramp."""
    out = []
    for i in range(128):
        q = (i + 0.5) / 128.0
        n = math.sqrt(max(0.0, 1 - q * q))
        diff = 0.22 + 0.78 * n ** 0.9
        k = (diff - 0.15) / 0.85 * 64 + 63 * n ** 14
        out.append(max(0, min(127, int(round(k)))))
    return out


def ramp_words(col, ch):
    """shade ramp of one colour channel: lo (dark end), s1 = colour - lo, s2 = highlight - colour"""
    nv, wv = (64, 940) if ch == 0 else (512, 512)
    dd = col - nv
    lo = nv + (dd >> 3) + (dd >> 6) + (dd >> 7)
    dw = wv - col
    hi = col + (dw >> 1) + (dw >> 4) - (dw >> 6)
    return lo, col - lo, hi - col


def palette_words():
    out = []
    for p in pal_table():
        g = p[0]
        w = [g[0], min(255, (g[1] + 2) >> 2) * 4, min(255, (g[2] + 2) >> 2) * 4]
        for s in range(3):
            for ch in range(3):
                lo, s1, s2 = ramp_words(p[1 + s][ch], ch)
                # the ramps stay inside 0 .. 1023 with U / V quantised to 8 bits (no clamps in the fill)
                for k in range(128):
                    v = (lo * 64 + s1 * k) >> 6 if k < 64 else ((lo + s1) * 64 + s2 * (k - 64)) >> 6
                    assert 0 <= v <= 1021, (p, s, ch, k, v)
                w += [lo, s1 & 0xFFFF, s2 & 0xFFFF]
        w += [0, 0]
        assert len(w) == 32
        out += w
    return out


def bcap_rom():
    out = []
    for i in range(128):
        A = (i + 0.5) / 32.0
        out.append(min(65535, int(round(65536 * 1.16 / (1 + math.sqrt(1 + 0.93 * A * A))))))
    return out


def prof_e_rom():
    p = prof_rom()
    return [p[127 - i] for i in range(128)]


def tab_rom():
    out = exp2_rom()
    out += palette_words()
    out += bcap_rom()
    out += prof_e_rom()
    assert len(out) == 1024
    return out


def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of unsigned(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['to_unsigned(%d, %d)' % (v, width) for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by braid.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package braid_pkg is",
             vhd_table("C_BR_TAB", 1024, 16, tab_rom()),
             vhd_table("C_BR_SQ", 256, 10, sq_rom()),
             vhd_table("C_BR_SIN", 1024, 12, sin_rom()),
             "end package braid_pkg;", ""]
    with open(os.path.join(HERE, "braid_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
