#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TRIPLICATE - build hook (pure Python: the builder runs it with the system python3).

Generates triplicate_pkg.vhd:

  C_OUTLINE  256 x 16: the cut-out outline of the collage's bear (skull dome between two ears, a neck,
             a bell-shaped body wider on the right, flat base), measured from the three tinted copies
             of selfie.jpg (they agree to 1 % of their height: one cut-out, photocopied at three sizes).
             128 rows (row r = height fraction r/128); each row holds two intervals, [left ear + dome]
             and [right ear] (identical below the ears), as four 8-bit edges in half store columns
             (store column u = (x/h + 0.25) * 204.8, x about the ear centre in copy heights h).
             Word address = row * 2 + w, word w = {edge 2w+1, edge 2w}.
  C_PALH / C_PALL  256 x 16: the photocopy inks.  Address = palette(3) & level(5); entry =
             paper + (ink - paper) * level/31 in 10-bit Y/U/V (hardware convention: U = Cr, V = Cb),
             packed {Y(9:0), U(9:4)} / {U(3:0), V(9:0), "00"}.  Palettes: 0 red 1 green 2 blue
             (measured on the collage), 3 cyan 4 magenta 5 yellow 6 black, 7 the front copy's
             luma ramp (Y = 64 + 876 * level/31, chroma neutral; the wall uses it too).
  f_osc_col / f_osc_row  the Osc figure (a skull-faced bear holding a TI computer, laid out like the
             collage's tinted copies), as two case tables over store column / store row (128 each,
             LUT logic).  Column flags: cross V/H, three lines of writing, teeth, monitor, screen,
             logo, console, keys.  Row: jaw / eye / nose half-widths and the matching row flags.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

OUTLINE = [
    (41, 41, 100, 100, 162, 162),
    (41, 41, 85, 118, 162, 162),
    (41, 41, 78, 126, 162, 162),
    (41, 41, 72, 131, 162, 162),
    (41, 41, 66, 134, 156, 167),
    (39, 44, 62, 137, 151, 171),
    (35, 56, 59, 140, 146, 172),
    (34, 171, 34, 171, 34, 171),
    (33, 171, 33, 171, 33, 171),
    (33, 171, 33, 171, 33, 171),
    (33, 171, 33, 171, 33, 171),
    (34, 170, 34, 170, 34, 170),
    (36, 168, 36, 168, 36, 168),
    (37, 167, 37, 167, 37, 167),
    (39, 166, 39, 166, 39, 166),
    (41, 164, 41, 164, 41, 164),
    (43, 163, 43, 163, 43, 163),
    (44, 162, 44, 162, 44, 162),
    (45, 161, 45, 161, 45, 161),
    (45, 161, 45, 161, 45, 161),
    (45, 161, 45, 161, 45, 161),
    (44, 162, 44, 162, 44, 162),
    (43, 162, 43, 162, 43, 162),
    (42, 162, 42, 162, 42, 162),
    (41, 163, 41, 163, 41, 163),
    (40, 164, 40, 164, 40, 164),
    (39, 165, 39, 165, 39, 165),
    (38, 165, 38, 165, 38, 165),
    (37, 166, 37, 166, 37, 166),
    (36, 167, 36, 167, 36, 167),
    (35, 168, 35, 168, 35, 168),
    (35, 169, 35, 169, 35, 169),
    (34, 170, 34, 170, 34, 170),
    (33, 171, 33, 171, 33, 171),
    (33, 172, 33, 172, 33, 172),
    (32, 172, 32, 172, 32, 172),
    (32, 173, 32, 173, 32, 173),
    (32, 174, 32, 174, 32, 174),
    (32, 175, 32, 175, 32, 175),
    (32, 176, 32, 176, 32, 176),
    (32, 177, 32, 177, 32, 177),
    (32, 177, 32, 177, 32, 177),
    (32, 178, 32, 178, 32, 178),
    (32, 179, 32, 179, 32, 179),
    (32, 180, 32, 180, 32, 180),
    (32, 181, 32, 181, 32, 181),
    (31, 183, 31, 183, 31, 183),
    (31, 185, 31, 185, 31, 185),
    (31, 188, 31, 188, 31, 188),
    (31, 190, 31, 190, 31, 190),
    (31, 193, 31, 193, 31, 193),
    (31, 197, 31, 197, 31, 197),
    (31, 200, 31, 200, 31, 200),
    (30, 203, 30, 203, 30, 203),
    (30, 205, 30, 205, 30, 205),
    (29, 208, 29, 208, 29, 208),
    (28, 210, 28, 210, 28, 210),
    (27, 212, 27, 212, 27, 212),
    (26, 214, 26, 214, 26, 214),
    (24, 215, 24, 215, 24, 215),
    (23, 217, 23, 217, 23, 217),
    (22, 219, 22, 219, 22, 219),
    (20, 220, 20, 220, 20, 220),
    (19, 222, 19, 222, 19, 222),
    (18, 223, 18, 223, 18, 223),
    (17, 225, 17, 225, 17, 225),
    (16, 226, 16, 226, 16, 226),
    (15, 228, 15, 228, 15, 228),
    (14, 229, 14, 229, 14, 229),
    (13, 231, 13, 231, 13, 231),
    (12, 233, 12, 233, 12, 233),
    (11, 234, 11, 234, 11, 234),
    (10, 236, 10, 236, 10, 236),
    (10, 238, 10, 238, 10, 238),
    (9, 239, 9, 239, 9, 239),
    (8, 241, 8, 241, 8, 241),
    (8, 242, 8, 242, 8, 242),
    (7, 244, 7, 244, 7, 244),
    (7, 245, 7, 245, 7, 245),
    (6, 246, 6, 246, 6, 246),
    (6, 247, 6, 247, 6, 247),
    (5, 248, 5, 248, 5, 248),
    (5, 249, 5, 249, 5, 249),
    (4, 249, 4, 249, 4, 249),
    (4, 249, 4, 249, 4, 249),
    (4, 250, 4, 250, 4, 250),
    (3, 250, 3, 250, 3, 250),
    (3, 250, 3, 250, 3, 250),
    (3, 250, 3, 250, 3, 250),
    (3, 251, 3, 251, 3, 251),
    (3, 251, 3, 251, 3, 251),
    (3, 252, 3, 252, 3, 252),
    (3, 252, 3, 252, 3, 252),
    (3, 252, 3, 252, 3, 252),
    (3, 251, 3, 251, 3, 251),
    (4, 251, 4, 251, 4, 251),
    (4, 251, 4, 251, 4, 251),
    (5, 250, 5, 250, 5, 250),
    (5, 250, 5, 250, 5, 250),
    (6, 249, 6, 249, 6, 249),
    (7, 248, 7, 248, 7, 248),
    (7, 247, 7, 247, 7, 247),
    (8, 247, 8, 247, 8, 247),
    (9, 246, 9, 246, 9, 246),
    (10, 245, 10, 245, 10, 245),
    (11, 244, 11, 244, 11, 244),
    (12, 243, 12, 243, 12, 243),
    (13, 242, 13, 242, 13, 242),
    (14, 241, 14, 241, 14, 241),
    (15, 240, 15, 240, 15, 240),
    (16, 238, 16, 238, 16, 238),
    (18, 237, 18, 237, 18, 237),
    (20, 235, 20, 235, 20, 235),
    (21, 233, 21, 233, 21, 233),
    (23, 231, 23, 231, 23, 231),
    (25, 229, 25, 229, 25, 229),
    (27, 228, 27, 228, 27, 228),
    (28, 227, 28, 227, 28, 227),
    (28, 227, 28, 227, 28, 227),
    (27, 227, 27, 227, 27, 227),
    (28, 227, 28, 227, 28, 227),
    (30, 225, 30, 225, 30, 225),
    (33, 222, 33, 222, 33, 222),
    (36, 219, 36, 219, 36, 219),
    (37, 218, 37, 218, 37, 218),
    (37, 218, 37, 218, 37, 218),
    (36, 219, 36, 219, 36, 219),
    (36, 219, 36, 219, 36, 219)
]


# (ink, paper) in hardware 10-bit Y, U(Cr), V(Cb): measured inks/papers of selfie.jpg (rgb_to_hw10)
INKS = [((365, 814, 422), (823, 585, 474)),   # red
        ((441, 377, 433), (873, 478, 474)),   # green
        ((351, 332, 697), (834, 449, 563)),   # blue
        ((443, 235, 682), (863, 461, 543)),   # cyan
        ((386, 793, 580), (838, 574, 521)),   # magenta
        ((711, 631, 186), (900, 534, 438)),   # yellow
        ((178, 514, 518), (861, 522, 492))]   # black


def palette():
    out = []
    for p in range(8):
        for lv in range(32):
            if p < 7:
                ink, paper = INKS[p]
                y, u, v = (int(round(paper[k] + (ink[k] - paper[k]) * lv / 31)) for k in range(3))
            else:
                y, u, v = int(round(64 + 876 * lv / 31)), 512, 512
            out.append((y, u, v))
    return out


import math
PAT = [0b10110111011011101101101110110101, 0b11011011101101110111011011011101, 0b01101110110111011011011101101110]


def osc_col(u):
    """11 column flags of the Osc figure at store column u (0..127)."""
    f = 0
    def bit(k, c):
        nonlocal f
        if c: f |= 1 << k
    bit(0, 40 <= u < 43)                                          # cross, vertical bar
    bit(1, 33 <= u < 50)                                          # cross, horizontal bar
    bit(2, 30 <= u < 68 and (PAT[0] >> ((u >> 1) & 31)) & 1)     # writing, line 1
    bit(3, 34 <= u < 62 and (PAT[1] >> ((u >> 1) & 31)) & 1)     # writing, line 2
    bit(4, 38 <= u < 56 and (PAT[2] >> ((u >> 1) & 31)) & 1)     # writing, line 3
    bit(5, 41 <= u < 59 and u % 2 == 0)                           # teeth
    bit(6, 13 <= u < 61)                                          # monitor
    bit(7, 18 <= u < 56)                                          # screen
    bit(8, 26 <= u < 48 and u % 4 != 3)                           # logo lettering
    bit(9, 6 <= u < 71)                                           # console
    bit(10, 14 <= u < 49 and u % 3 != 0)                          # keys
    return f


def hw(rx, ry, cr, r):
    d = abs(r + 0.5 - cr)
    return int(round(rx * math.sqrt(1 - (d / ry) ** 2))) if d < ry else 0


def osc_row(r):
    """(hwJ 5b, hwE 4b, hwN 3b, flags 10b) at store row r: jaw / eye / nose half-widths and
    row flags 0 crossV 1 crossH 2..4 writing lines 5 teeth 6 monitor 7 screen 8 console 9 keys."""
    hj = min(31, hw(18.8, 10.9, 32.0, r)); he = min(15, hw(8.0, 4.4, 23.5, r)); hn = min(7, hw(3.7, 3.6, 31.5, r))
    f = 0
    for k, c in enumerate((2 <= r < 10, 5 <= r < 7, 11 <= r < 13, 14 <= r < 16, 17 <= r < 19, 37 <= r < 40,
                           64 <= r < 95, 67 <= r < 90, 97 <= r < 119, 102 <= r < 116 and r % 3 != 0)):
        if c: f |= 1 << k
    return hj, he, hn, f


def vhdl_tables():
    col = ["        case to_integer(u) is"]
    for u in range(128):
        col.append(f'            when {u} => return "{osc_col(u):011b}";')
    col.append('            when others => return "00000000000";\n        end case;')
    row = ["        case to_integer(r) is"]
    for r in range(128):
        hj, he, hn, f = osc_row(r)
        row.append(f'            when {r} => return "{hj:05b}{he:04b}{hn:03b}{f:010b}";')
    row.append('            when others => return "0000000000000000000000";\n        end case;')
    return (
        "    function f_osc_col(u : unsigned(6 downto 0)) return std_logic_vector;\n"
        "    function f_osc_row(r : unsigned(6 downto 0)) return std_logic_vector;\n",
        "    function f_osc_col(u : unsigned(6 downto 0)) return std_logic_vector is\n    begin\n"
        + "\n".join(col) + "\n    end function;\n"
        "    function f_osc_row(r : unsigned(6 downto 0)) return std_logic_vector is\n    begin\n"
        + "\n".join(row) + "\n    end function;\n")


def outline2(r):
    """Measured 3-interval row -> 2 intervals: [left ear + dome] (the left notch, 2 rows deep, is
    filled), [right ear]."""
    l0, l1, d0, d1, r0, r1 = r
    if l1 > l0:                       # left ear present: join it to the dome
        a = (min(l0, d0), max(l1, d1))
    else:
        a = (d0, d1)
    return (a[0], a[1], r0, r1)


def emit():
    words = []
    for r in OUTLINE:
        e = outline2(r)
        words += [(e[1] << 8) | e[0], (e[3] << 8) | e[2]]
    body = []
    for i in range(0, len(words), 16):
        body.append("        " + ", ".join(f'x"{v:04X}"' for v in words[i:i + 16]))
    decl, fbody = vhdl_tables()
    pal = palette()
    ph = [(y << 6) | (u >> 4) for (y, u, v) in pal]
    pl = [((u & 15) << 12) | (v << 2) for (y, u, v) in pal]
    def arr(name, vals):
        rows = []
        for i in range(0, len(vals), 16):
            rows.append("        " + ", ".join(f'x"{v:04X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : t_pal256 := (\n" + ",\n".join(rows) + "\n    );\n"
    txt = (
        "-- triplicate_pkg.vhd -- GENERATED by triplicate.py; edit the generator, not this file.\n"
        "--\n"
        "-- C_OUTLINE: 256 x 16, the bear cut-out of selfie.jpg; address = row * 2 + w,\n"
        "-- word w = {edge 2w+1, edge 2w}, edges in half store columns (0..255).\n\n"
        "library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n"
        "package triplicate_pkg is\n"
        "    type t_outline is array(0 to 255) of std_logic_vector(15 downto 0);\n"
        "    constant C_OUTLINE : t_outline := (\n" + ",\n".join(body) + "\n    );\n"
        "    type t_pal256 is array(0 to 255) of std_logic_vector(15 downto 0);\n"
        + arr("C_PALH", ph) + arr("C_PALL", pl) + decl +
        "end package triplicate_pkg;\n\n"
        "package body triplicate_pkg is\n" + fbody + "end package body triplicate_pkg;\n")
    with open(os.path.join(HERE, "triplicate_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
