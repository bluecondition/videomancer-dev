#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TUFTED v0.1.0 - build hook.  Writes tufted_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.kinetic_work/rb_tufted/imodel.py) imports the same table functions.

Package contents:
  C_TF_EXP2  256 x 16   round(32768 * 2^(i/256))                         (pitch, shape gain, rate)
  C_TF_SIN   256 x 16   round(1023 * sin(pi/2 * (i + 1/2) / 256))         the wave, one quarter cycle
  C_TF_PAL   256 x 32   words 0..127: 16 palettes x 8 key colours {Y 10, U 8, V 8, 000000}, hardware convention
                        (U register = Cr): 0 ground, 1 A, 2 highlight, 3 B, 4 B dark, 5 B light, 6 white, 7 black
                        words 128..191: the pillow tone curve per tone step t, {matte 10 bits, gloss 10 bits}
                        words 192..223: the button tone curve per step; a curve entry = {segment 2, weight 8}
  C_TF_PLT   2048 x 4   pleat weight in sixteenths (0..15) by {|d| level 5 bits, distance from the seam 6 bits}
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

# pillow tone curve: tone step t (0..63) <-> light term tl = (t + 1/2 - 32) / 32; ramp position
# r = 0.5 + 0.5 tl^LIT_G on the lit side (broad A, compact highlight), 0.5 - 0.5 |tl|^SHD_G in shadow
LIT_G, SHD_G = 1.3, 0.85
GLOSS_TH, GLOSS_W = 0.36, 0.10     # Gloss: the highlight is reached at tl = TH, then white over GW (a blown spot)
# button tone curve: 32 steps, dark B -> B (first BSPLIT of the range) -> light B
BSPLIT = 0.55
# pleat profile: width pw = PW0 + PW1 |d| (cell half-width units), weight = (1 - e / pw)^GAMMA
PW0, PW1, PGAMMA = 0.035, 0.20, 1.5


def hexrgb(h):
    h = h.lstrip("#")
    return [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]


def hw10(h):
    """sRGB hex -> (y, u, v) 10-bit, hardware convention (u = Cr, v = Cb), BT.601, Y = 64 + 876 y."""
    r, g, b = hexrgb(h)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def lum(h):
    r, g, b = hexrgb(h)
    return 0.299 * r + 0.587 * g + 0.114 * b


def key8(c):
    """(y10, u10, v10) -> (y10, u8, v8): chroma kept to 8 bits (steps of 4) around 128."""
    return (c[0], min(255, (c[1] + 2) >> 2), min(255, (c[2] + 2) >> 2))


def mix8(a, b, f):
    return tuple(int(round(a[i] + (b[i] - a[i]) * f)) for i in range(3))


WHITE = (940, 128, 128)
BLACK = (64, 128, 128)


def keys():
    """16 x 8 key colours (y10, u8, v8): ground, A, highlight, B, B dark, B light, white, black.
    highlight = the brightest of ground / B / accent, or white when none is clearly brighter than A, so the lit side
    of a pillow is always the brighter one."""
    out = []
    for name, g, a, b, c in PALETTES:
        cand = max((g, b, c), key=lum)
        hi = key8(hw10(cand)) if lum(cand) > lum(a) + 0.08 else WHITE
        G, A, B = key8(hw10(g)), key8(hw10(a)), key8(hw10(b))
        out.append([G, A, hi, B, mix8(BLACK, B, 0.45), mix8(B, WHITE, 0.6), WHITE, BLACK])
    return out


def _seg(r):
    """ramp position r in [0, 1] (two segments) -> (segment, weight 0..255)"""
    if r < 0.5:
        return 0, min(255, int(round(r * 2 * 256)))
    return 1, min(255, int(round((r - 0.5) * 2 * 256)))


def tone_curves():
    """64 x (matte, gloss) curve entries, each (segment, weight).  Segments: 0 black->A, 1 A->highlight,
    2 highlight->white."""
    out = []
    def rr(tl):
        return 0.5 + 0.5 * tl ** LIT_G if tl >= 0 else 0.5 - 0.5 * (-tl) ** SHD_G
    rth = rr(GLOSS_TH)
    for t in range(64):
        tl = (t + 0.5 - 32) / 32.0
        r = rr(tl)
        m = _seg(r)
        if tl >= GLOSS_TH:
            g = (2, min(255, int(round(256 * (tl - GLOSS_TH) / GLOSS_W))))
        elif r < 0.5:
            g = _seg(r)
        else:
            g = (1, min(255, int(round(256 * (r - 0.5) / (rth - 0.5)))))     # reaches the highlight at TH
        out.append((m, g))
    return out


def button_curve():
    """32 entries (segment, weight): 0 B dark -> B, 1 B -> B light."""
    out = []
    for t in range(32):
        r = t / 31.0
        if r < BSPLIT:
            out.append((0, min(255, int(round(256 * r / BSPLIT)))))
        else:
            out.append((1, min(255, int(round(256 * (r - BSPLIT) / (1 - BSPLIT))))))
    return out


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(1023 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]


def pleat_rom():
    """address = dq * 64 + e6:  |d| = (dq + 1/2) / 16, e = (e6 + 1/2) / 128 (distance past the solid core, cell
    half-width units); value = round(16 (1 - e / pw)^GAMMA), at most 15."""
    out = []
    for dq in range(32):
        pw = PW0 + PW1 * (dq + 0.5) / 16.0
        for e6 in range(64):
            e = (e6 + 0.5) / 128.0
            w = max(0.0, 1.0 - e / pw) ** PGAMMA
            out.append(min(15, int(round(16 * w))))
    return out


def pal_rom():
    out = []
    for p in keys():
        for (y, u, v) in p:
            out.append((y << 22) | (u << 14) | (v << 6))
    for (m, g) in tone_curves():
        out.append((((m[0] << 8) | m[1]) << 10) | ((g[0] << 8) | g[1]))
    for (s, w) in button_curve():
        out.append(((s << 8) | w) << 10)
    return out + [0] * (256 - len(out))


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % ((width + 3) // 4, v) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by tufted.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tufted_pkg is",
             vhd_table("C_TF_EXP2", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_TF_SIN", "unsigned", 256, 16, sin_rom()),
             vhd_table("C_TF_PAL", "std_logic_vector", 256, 32, pal_rom()),
             vhd_table("C_TF_PLT", "unsigned", 2048, 4, pleat_rom()),
             "end package tufted_pkg;", ""]
    with open(os.path.join(HERE, "tufted_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
