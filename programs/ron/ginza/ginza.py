#!/usr/bin/env python3
# ginza.py - Build hook for the Ginza Videomancer program.
# Copyright (C) 2026  czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# Generates ginza_glyphs_pkg.vhd: 32 bold 16x16 1-bit glyphs in the spirit of
# 1920s-30s Japanese commercial-art lettering (hand-cut, geometric, 3-px
# strokes).  Glyphs 0..15 are kanji-like characters (the stacked column of the
# poster); 16..31 are geometric Latin capitals and digits ("ABCDEFGHI", "20").
# Each glyph is a list of filled rectangles (x0, y0, x1, y1), inclusive, on a
# 16x16 grid, so every stroke is uniformly bold and the ROM is deterministic
# with no font files or third-party Python dependencies.
#
# Output rows are 16-bit std_logic_vector with the MSB = leftmost pixel.

import os
import sys

W = 16
H = 16


def H_(y, x0=1, x1=14, t=3):
    """Horizontal bar: rows y..y+t-1, columns x0..x1."""
    return (x0, y, x1, y + t - 1)


def V_(x, y0=0, y1=15, t=3):
    """Vertical bar: columns x..x+t-1, rows y0..y1."""
    return (x, y0, x + t - 1, y1)


# ---------------------------------------------------------------- kanji-like
KANJI = [
    # 0 月
    [V_(2, 1, 14), V_(11, 1, 14), H_(1, 2, 13), H_(6, 4, 11, 2), H_(10, 4, 11, 2),
     (0, 12, 3, 14)],
    # 1 日
    [H_(1, 1, 14), H_(13, 1, 14), V_(1, 1, 14), V_(12, 1, 14), H_(7, 1, 14, 2)],
    # 2 出
    [V_(7, 0, 15, 2), V_(2, 1, 7), V_(11, 1, 7), H_(6, 2, 13), V_(1, 8, 15),
     V_(12, 8, 15), H_(13, 1, 14)],
    # 3 大
    [H_(4, 1, 14), V_(7, 0, 8, 2), (5, 9, 7, 10), (3, 11, 5, 12), (1, 13, 3, 15),
     (8, 9, 10, 10), (10, 11, 12, 12), (12, 13, 14, 15)],
    # 4 店
    [(7, 0, 8, 1), H_(2, 1, 14), V_(1, 2, 15, 2), H_(6, 6, 12, 2), V_(8, 5, 9, 2),
     H_(9, 5, 13, 2), H_(13, 5, 13, 2), V_(5, 9, 14, 2), V_(12, 9, 14, 2)],
    # 5 未
    [H_(2, 4, 11, 2), H_(6, 1, 14), V_(7, 0, 15, 2), (4, 10, 6, 11), (2, 12, 4, 14),
     (9, 10, 11, 11), (11, 12, 13, 14)],
    # 6 書
    [H_(0, 3, 12, 2), H_(3, 1, 14, 2), H_(6, 3, 12, 2), V_(7, 0, 7, 2),
     H_(9, 2, 13, 2), H_(14, 2, 13, 2), V_(2, 9, 15, 2), V_(12, 9, 15, 2),
     H_(12, 2, 13, 1)],
    # 7 夜
    [(7, 0, 8, 1), H_(2, 1, 14, 2), V_(3, 4, 15), (8, 4, 10, 9), (11, 10, 13, 15),
     (7, 10, 9, 13), (10, 4, 13, 6)],
    # 8 入
    [(7, 0, 9, 3), (6, 1, 8, 4), (4, 5, 6, 8), (2, 9, 4, 12), (0, 13, 2, 15),
     (8, 3, 10, 6), (10, 7, 12, 10), (12, 11, 14, 15)],
    # 9 服
    [V_(0, 1, 14, 2), V_(5, 1, 14, 2), H_(1, 0, 6, 2), H_(6, 0, 6, 2), H_(10, 0, 6, 2),
     H_(1, 8, 15, 2), V_(8, 1, 9, 2), V_(13, 1, 9, 2), (9, 10, 11, 12), (12, 13, 14, 15),
     (8, 13, 10, 15)],
    # 10 田
    [H_(1, 1, 14, 2), H_(13, 1, 14, 2), V_(1, 1, 14, 2), V_(13, 1, 14, 2),
     H_(7, 1, 14, 2), V_(7, 1, 14, 2)],
    # 11 中
    [H_(3, 2, 13, 2), H_(8, 2, 13, 2), V_(2, 3, 9, 2), V_(12, 3, 9, 2), V_(7, 0, 15, 2)],
    # 12 山
    [V_(7, 0, 15, 2), V_(1, 5, 14, 2), V_(13, 5, 14, 2), H_(12, 1, 14)],
    # 13 川
    [V_(1, 1, 12), V_(7, 3, 15, 2), V_(12, 0, 15)],
    # 14 工
    [H_(1, 2, 13), H_(12, 1, 14), V_(7, 1, 14, 2)],
    # 15 王
    [H_(1, 2, 13, 2), H_(7, 3, 12, 2), H_(13, 1, 14), V_(7, 1, 14, 2)],
]

# ------------------------------------------------------------------- latin
LATIN = {
    'A': [H_(0, 4, 11), V_(1, 3, 15), V_(12, 3, 15), H_(8, 1, 14)],
    'B': [V_(1, 0, 15), H_(0, 1, 11), H_(7, 1, 11, 2), H_(13, 1, 11), V_(10, 2, 7),
          V_(11, 8, 13)],
    'C': [H_(0, 3, 13), H_(13, 3, 13), V_(1, 2, 13), (12, 3, 13, 4), (12, 11, 13, 12)],
    'D': [V_(1, 0, 15), H_(0, 1, 10), H_(13, 1, 10), V_(11, 3, 12)],
    'E': [V_(1, 0, 15), H_(0, 1, 14), H_(7, 1, 11, 2), H_(13, 1, 14)],
    'F': [V_(1, 0, 15), H_(0, 1, 14), H_(7, 1, 11, 2)],
    'G': [H_(0, 3, 13), H_(13, 3, 13), V_(1, 2, 13), (12, 3, 13, 4), H_(7, 8, 14, 2),
          V_(11, 8, 13)],
    'H': [V_(1, 0, 15), V_(12, 0, 15), H_(7, 1, 14, 2)],
    'I': [V_(6, 0, 15), H_(0, 3, 11, 2), H_(14, 3, 11, 2)],
    '2': [H_(0, 2, 13), V_(11, 2, 7), H_(6, 2, 13), V_(1, 8, 13), H_(13, 1, 14)],
    '0': [H_(0, 3, 12), H_(13, 3, 12), V_(1, 2, 13), V_(12, 2, 13)],
}
LATIN_ORDER = "ABCDEFGHI2020202"


def render(rects):
    rows = [0] * H
    for (x0, y0, x1, y1) in rects:
        for y in range(max(0, y0), min(H - 1, y1) + 1):
            for x in range(max(0, x0), min(W - 1, x1) + 1):
                rows[y] |= 1 << (W - 1 - x)
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "ginza_glyphs_pkg.vhd")

    glyphs = [("kanji %d" % i, render(r)) for i, r in enumerate(KANJI)]
    glyphs += [("latin %s" % ch, render(LATIN[ch])) for ch in LATIN_ORDER]
    n = len(glyphs)
    assert n == 32

    if "--show" in sys.argv:
        for name, rows in glyphs:
            print(name)
            for r in rows:
                print("".join("#" if (r >> (W - 1 - x)) & 1 else "." for x in range(W)))
            print()

    lines = []
    lines.append("-- Copyright (C) 2026 czartroniks")
    lines.append("-- SPDX-License-Identifier: GPL-3.0-only")
    lines.append("--")
    lines.append("-- This file is part of Videomancer Community Programs.")
    lines.append("-- See LICENSE file in the repository root for full license text.")
    lines.append("--")
    lines.append("-- AUTO-GENERATED by ginza.py - do not edit by hand.")
    lines.append(f"-- {n} glyphs x {H} rows x {W} bits.  MSB = leftmost pixel.")
    lines.append("-- 0..15 kanji-like, 16..31 Latin capitals / digits.")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("")
    lines.append("package ginza_glyphs_pkg is")
    lines.append("")
    lines.append(f"    constant C_NGLYPH  : integer := {n};")
    lines.append("")
    lines.append("    type t_glyph_rom is array (0 to C_NGLYPH * 16 - 1)")
    lines.append("        of std_logic_vector(15 downto 0);")
    lines.append("")
    lines.append("    constant C_GLYPHS : t_glyph_rom := (")
    for idx, (name, rows) in enumerate(glyphs):
        row_strs = ", ".join(f'x"{r:04X}"' for r in rows)
        sep = "," if idx < n - 1 else " "
        lines.append(f"        -- {idx}: {name}")
        lines.append(f"        {row_strs}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("end package ginza_glyphs_pkg;")
    lines.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(lines))
    print(f"Wrote {out_path} ({n} glyphs, {W}x{H} 1bpp)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
