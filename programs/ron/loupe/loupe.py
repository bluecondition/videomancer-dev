#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
LOUPE - build hook.

Generates loupe_pkg.vhd:

  C_SQRT  2048 x 11: the mantissa square root.  Address = {L(0), m(9:0)} where
          s = m . 2^(L-10) with m in [1024, 2048) (bit 10 implicit) and L the
          leading-one position of s.  Value v = round(sqrt(m . 2^(L&1)) . 64)
          - 2048 (0..2047); d = ((2048 + v) << 1) >> (12 - L/2).
  C_SIN   256 x 8 signed: the sequencer's sine (lens drift).
  C_PY / C_PU / C_PV   2048 x 8: eight palettes of 256 entries,
          address = palette(3) & entry(8).  Entries 0..223 are the field's
          contour cycle (posterized bands with thin fringes, entry 0 = the
          scallops' cores), entries 224..255 the chevron stripes of the lens
          facet (one stripe period).

Y is legal-range 10-bit BT.601 / 4; U/V are 10-bit / 4 and stored U/V-SWAPPED
for this hardware (the U register carries Cr).

"Loupe" is fitted to the reference capture: white cores, cyan, grey with a
yellow/black lightning band, lavender with magenta fringes, dark green, cyan
fringes; magenta / white / cyan chevrons.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
NF = 224         # field entries per palette
NS = 32          # stripe entries per palette

# ---------------------------------------------------------------- palettes
# Each palette: field bands [(end_entry, rgb)] covering 0..224 and stripe
# bands [(end_entry, rgb)] covering 0..32.  Hard steps (analog colourizer
# posterization); one- and two-entry bands are the fringe lines.

WHITE = (244, 249, 251); LTC = (100, 225, 235); CYAN = (10, 253, 253)
GREY = (133, 152, 151); BLK = (10, 11, 11); YEL = (216, 222, 51)
DARK = (17, 44, 27); LAV = (193, 205, 225); MAG = (229, 81, 246)
DKG = (52, 77, 65); GLAV = (163, 178, 188)

PALETTES = [
    ("Loupe",
     [(50, WHITE), (52, LTC), (85, CYAN), (87, LTC), (97, GREY), (98, BLK),
      (100, YEL), (101, DARK), (104, GREY), (105, YEL), (111, DARK), (128, GREY),
      (132, GLAV), (135, LAV), (138, MAG), (144, DKG), (145, MAG), (159, LAV),
      (166, GREY), (180, DARK), (182, CYAN), (190, GREY), (204, DARK), (205, LAV),
      (206, WHITE), (214, CYAN), (215, MAG), (224, DKG)],
     [(10, MAG), (13, (240, 200, 250)), (22, (40, 240, 250)), (27, (200, 245, 250)),
      (32, (190, 170, 240))]),
    ("Ember",
     [(38, (255, 250, 225)), (40, (255, 230, 140)), (68, (255, 190, 40)),
      (70, (255, 140, 20)), (82, (205, 70, 20)), (83, (0, 0, 0)), (85, (60, 240, 255)),
      (87, (20, 10, 10)), (110, (150, 40, 25)), (113, (120, 30, 40)), (124, (90, 20, 60)),
      (126, (255, 60, 170)), (131, (40, 10, 20)), (132, (255, 60, 170)),
      (153, (110, 35, 30)), (156, (160, 80, 30)), (174, (30, 12, 10)),
      (176, (255, 170, 40)), (186, (120, 40, 20)), (205, (25, 10, 8)),
      (207, (200, 90, 30)), (208, (255, 230, 150)), (213, (255, 150, 30)),
      (224, (60, 20, 15))],
     [(10, (255, 120, 0)), (13, (255, 240, 180)), (22, (255, 210, 40)),
      (27, (255, 250, 230)), (32, (220, 40, 30))]),
    ("Abyss",
     [(38, (225, 250, 255)), (40, (160, 230, 255)), (68, (60, 170, 255)),
      (70, (30, 120, 220)), (82, (20, 60, 130)), (83, (0, 0, 0)), (85, (240, 250, 255)),
      (87, (5, 10, 30)), (110, (15, 45, 95)), (113, (20, 80, 110)), (124, (10, 120, 130)),
      (126, (180, 90, 255)), (131, (5, 25, 45)), (132, (180, 90, 255)),
      (153, (12, 95, 115)), (156, (20, 60, 100)), (174, (4, 10, 28)),
      (176, (40, 230, 255)), (186, (18, 50, 100)), (205, (6, 12, 34)),
      (207, (60, 140, 200)), (208, (220, 245, 255)), (213, (40, 180, 240)),
      (224, (10, 30, 60))],
     [(10, (40, 220, 255)), (13, (230, 250, 255)), (22, (30, 80, 230)),
      (27, (170, 230, 255)), (32, (140, 90, 255))]),
    ("Toxic",
     [(38, (240, 255, 180)), (40, (210, 255, 90)), (68, (170, 255, 40)),
      (70, (120, 220, 20)), (82, (60, 110, 30)), (83, (0, 0, 0)), (85, (255, 40, 200)),
      (87, (10, 20, 8)), (110, (45, 90, 30)), (113, (70, 100, 35)), (124, (95, 115, 40)),
      (126, (255, 240, 40)), (131, (20, 30, 12)), (132, (255, 240, 40)),
      (153, (90, 110, 40)), (156, (60, 95, 30)), (174, (12, 22, 8)),
      (176, (190, 255, 60)), (186, (50, 90, 30)), (205, (14, 24, 10)),
      (207, (120, 160, 40)), (208, (240, 255, 170)), (213, (170, 255, 40)),
      (224, (30, 45, 15))],
     [(10, (170, 255, 30)), (13, (250, 255, 200)), (22, (20, 30, 10)),
      (27, (255, 240, 60)), (32, (90, 140, 30))]),
    ("Candy",
     [(38, (255, 245, 250)), (40, (255, 210, 235)), (68, (255, 170, 215)),
      (70, (240, 140, 200)), (82, (170, 215, 255)), (83, (90, 60, 200)),
      (85, (255, 255, 255)), (87, (120, 90, 210)), (110, (170, 215, 255)),
      (113, (185, 195, 255)), (124, (200, 170, 255)), (126, (255, 120, 190)),
      (131, (150, 120, 220)), (132, (255, 120, 190)), (153, (200, 170, 255)),
      (156, (180, 225, 230)), (174, (160, 240, 210)), (176, (255, 255, 255)),
      (186, (255, 215, 160)), (205, (160, 240, 210)), (207, (255, 200, 230)),
      (208, (255, 255, 255)), (213, (170, 215, 255)), (224, (200, 170, 255))],
     [(10, (255, 110, 190)), (13, (255, 245, 250)), (22, (140, 210, 255)),
      (27, (250, 250, 255)), (32, (200, 170, 255))]),
    ("Mono",
     [(38, (245, 245, 245)), (40, (200, 200, 200)), (68, (175, 175, 175)),
      (70, (150, 150, 150)), (82, (120, 120, 120)), (83, (0, 0, 0)),
      (85, (255, 255, 255)), (87, (20, 20, 20)), (110, (110, 110, 110)),
      (113, (150, 150, 150)), (124, (200, 200, 200)), (126, (255, 255, 255)),
      (131, (60, 60, 60)), (132, (255, 255, 255)), (153, (190, 190, 190)),
      (156, (120, 120, 120)), (174, (25, 25, 25)), (176, (230, 230, 230)),
      (186, (110, 110, 110)), (205, (20, 20, 20)), (207, (180, 180, 180)),
      (208, (255, 255, 255)), (213, (170, 170, 170)), (224, (50, 50, 50))],
     [(10, (10, 10, 10)), (13, (128, 128, 128)), (22, (245, 245, 245)),
      (27, (200, 200, 200)), (32, (70, 70, 70))]),
    ("Neon",
     [(38, (20, 18, 40)), (40, (120, 0, 255)), (68, (0, 255, 240)),
      (70, (0, 160, 255)), (82, (40, 0, 80)), (83, (0, 0, 0)), (85, (255, 255, 0)),
      (87, (0, 0, 0)), (110, (50, 0, 100)), (113, (90, 0, 140)), (124, (20, 0, 40)),
      (126, (255, 0, 170)), (131, (0, 0, 0)), (132, (255, 0, 170)),
      (153, (30, 0, 70)), (156, (60, 255, 60)), (174, (0, 0, 0)),
      (176, (0, 255, 240)), (186, (45, 0, 90)), (205, (0, 0, 0)),
      (207, (255, 0, 170)), (208, (255, 255, 255)), (213, (0, 200, 255)),
      (224, (25, 0, 50))],
     [(10, (255, 0, 190)), (13, (0, 0, 0)), (22, (0, 255, 240)),
      (27, (10, 10, 30)), (32, (255, 255, 40))]),
    ("Rust",
     [(38, (250, 240, 220)), (40, (180, 210, 200)), (68, (90, 170, 170)),
      (70, (60, 130, 130)), (82, (150, 70, 30)), (83, (20, 10, 5)),
      (85, (255, 200, 60)), (87, (30, 15, 8)), (110, (160, 80, 35)),
      (113, (175, 100, 55)), (124, (190, 120, 70)), (126, (110, 200, 180)),
      (131, (60, 30, 20)), (132, (110, 200, 180)), (153, (185, 115, 65)),
      (156, (140, 70, 35)), (174, (55, 28, 18)), (176, (110, 140, 130)),
      (186, (150, 75, 35)), (205, (45, 22, 14)), (207, (190, 130, 80)),
      (208, (250, 235, 210)), (213, (100, 160, 150)), (224, (80, 40, 25))],
     [(10, (200, 110, 50)), (13, (250, 240, 220)), (22, (70, 160, 160)),
      (27, (230, 210, 180)), (32, (120, 50, 25))]),
]


# The other palettes were drawn on a first-fit band layout; their band ends
# are mapped onto Loupe's layout so the lightning band and the fringes land
# at the same field levels in every palette.
REMAP = [(0, 0), (38, 50), (40, 52), (68, 85), (70, 87), (82, 97), (83, 98),
         (85, 100), (87, 102), (110, 128), (113, 132), (124, 135), (126, 138),
         (131, 144), (132, 145), (153, 159), (156, 166), (174, 180), (176, 182),
         (186, 190), (205, 204), (207, 205), (208, 206), (213, 214), (224, 224)]


def remap(spec):
    out = []
    for end, rgb in spec:
        for (a0, b0), (a1, b1) in zip(REMAP, REMAP[1:]):
            if a0 <= end <= a1:
                out.append((int(round(b0 + (b1 - b0) * (end - a0) / (a1 - a0))), rgb))
                break
    return out


PALETTES = [PALETTES[0]] + [(n, remap(f), s) for n, f, s in PALETTES[1:]]


def bands(spec, n):
    out = []
    j = 0
    for i in range(n):
        while i >= spec[j][0]:
            j += 1
        out.append(spec[j][1])
    return out


def rgb_table():
    out = []
    for _name, field, stripe in PALETTES:
        assert field[-1][0] == NF and stripe[-1][0] == NS
        out += bands(field, NF) + bands(stripe, NS)
    return out


def yuv8(rgb):
    """sRGB (0..255) -> (Y10/4, Cr10/4, Cb10/4): legal-range BT.601, U/V swapped."""
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    Y = 64 + 876 * y
    Cb = 512 + 896 * cb
    Cr = 512 + 896 * cr
    q = lambda v, lo, hi: max(lo, min(hi, int(round(v / 4.0))))
    # hardware swaps U/V: U register carries Cr
    return q(Y, 16, 235), q(Cr, 16, 240), q(Cb, 16, 240)


def sqrt_table():
    out = []
    for odd in (0, 1):
        for lo in range(1024):
            m = (1024 + lo) << odd
            v = int(round(math.sqrt(m) * 64)) - 2048
            out.append(max(0, min(2047, v)))
    return out


def sin_table():
    return [int(round(127 * math.sin(2 * math.pi * i / 256))) for i in range(256)]


def emit():
    words = [yuv8(c) for c in rgb_table()]
    assert len(words) == 2048

    def arr(name, typ, vals, digits):
        body = []
        for i in range(0, len(vals), 16):
            if digits == 2:
                body.append("        " + ", ".join(f'x"{v & 0xFF:02X}"' for v in vals[i:i + 16]))
            else:
                body.append("        " + ", ".join(f'"{v & 0x7FF:011b}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    consts = [
        arr("C_SQRT", "t_sqrt", sqrt_table(), 11),
        arr("C_SIN", "t_rom256", sin_table(), 2),
        arr("C_PY", "t_rom2k", [w[0] for w in words], 2),
        arr("C_PU", "t_rom2k", [w[1] for w in words], 2),
        arr("C_PV", "t_rom2k", [w[2] for w in words], 2),
    ]
    names = ", ".join(p[0] for p in PALETTES)
    txt = f"""-- loupe_pkg.vhd -- GENERATED by loupe.py; edit the generator, not this file.
--
-- C_SQRT: 2048 x 11 mantissa square root, address {{L(0), m(9:0)}}.
-- C_SIN:  256 x 8 signed sine for the sequencer (lens drift).
-- C_PY/C_PU/C_PV: eight palettes ({names}), 256 entries each,
-- address = palette & entry; entries 0..223 = the field's contour cycle,
-- 224..255 = the lens facet's chevron stripe.  Y and chroma are 10-bit
-- values / 4; U/V-SWAPPED for this hardware (U carries Cr).

library ieee;
use ieee.std_logic_1164.all;

package loupe_pkg is
    type t_sqrt is array(0 to 2047) of std_logic_vector(10 downto 0);
    type t_rom256 is array(0 to 255) of std_logic_vector(7 downto 0);
    type t_rom2k is array(0 to 2047) of std_logic_vector(7 downto 0);
{chr(10).join(consts)}
end package loupe_pkg;
"""
    with open(os.path.join(HERE, "loupe_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
