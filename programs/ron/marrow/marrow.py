#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
MARROW - build hook.

Generates marrow_pkg.vhd:

  C_WAVE  512 x 8 signed: entries 0..255 = cos(2 pi i / 256) . 127 (Round),
          256..511 = the cosine-phased triangle (Diamond).  The pixel path
          reads it four times per pixel (four EBR copies).
  C_SIN   256 x 8 signed: the sequencer's own sine for the per-line LFOs.
  C_PY / C_PU / C_PV   2048 x 8: eight 256-entry colourizer palettes,
          address = palette(3) & index(8).  Index 0 is the FOLD (the outline
          drawn where the field crosses zero), 255 the CLIP (the background
          the envelope floods everything with).

Y is legal-range 10-bit BT.601 divided by 4; U/V are 10-bit / 4 and stored
U/V-SWAPPED for this hardware (the U register carries Cr).

"Marrow" is fitted to the reference capture: dark-red outlines, then slate,
azure, sky blue, grey-pink, peach and orange, into flat signal yellow.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
N = 256          # entries per palette


def interp(t, knots):
    """piecewise-linear through (t, (r, g, b)) knots, rgb 0..255"""
    if t <= knots[0][0]:
        return knots[0][1]
    for (t0, c0), (t1, c1) in zip(knots, knots[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return tuple(a + (b - a) * f for a, b in zip(c0, c1))
    return knots[-1][1]


# index 0 = the fold (outline), 1 = the clip (background)
PALETTES = [
    ("Marrow", [      # the reference capture
        (0.00, (175, 16, 0)), (0.07, (150, 17, 0)), (0.13, (112, 20, 2)),
        (0.17, (82, 44, 30)), (0.21, (40, 70, 110)), (0.25, (8, 112, 210)),
        (0.30, (5, 138, 240)), (0.35, (45, 152, 220)), (0.40, (115, 162, 190)),
        (0.45, (185, 168, 160)), (0.52, (230, 176, 128)), (0.62, (243, 168, 70)),
        (0.74, (244, 150, 45)), (0.84, (243, 185, 50)), (0.92, (242, 225, 30)),
        (0.97, (240, 244, 0)), (1.00, (240, 244, 0))]),
    ("Abyss", [       # glowing cyan rings on midnight
        (0.00, (250, 245, 220)), (0.05, (120, 230, 240)), (0.12, (20, 160, 200)),
        (0.22, (10, 90, 150)), (0.35, (25, 40, 110)), (0.50, (45, 20, 85)),
        (0.65, (55, 14, 68)), (0.80, (20, 8, 38)), (0.95, (6, 6, 20)),
        (1.00, (6, 6, 20))]),
    ("Thermal", [     # white-hot outlines through iron-bow into black
        (0.00, (255, 255, 235)), (0.08, (255, 232, 110)), (0.18, (255, 160, 30)),
        (0.30, (230, 60, 20)), (0.45, (170, 20, 90)), (0.60, (90, 20, 130)),
        (0.75, (30, 20, 110)), (0.90, (10, 10, 40)), (0.97, (2, 2, 12)),
        (1.00, (2, 2, 12))]),
    ("Acid", [        # magenta / violet / teal / lime on dark olive
        (0.00, (255, 40, 200)), (0.10, (130, 25, 220)), (0.22, (25, 45, 170)),
        (0.33, (0, 175, 125)), (0.46, (120, 250, 40)), (0.60, (225, 250, 70)),
        (0.76, (60, 80, 25)), (0.92, (14, 20, 10)), (1.00, (14, 20, 10))]),
    ("Lagoon", [      # coral rims, cream, aqua into deep sea
        (0.00, (255, 110, 85)), (0.08, (255, 228, 196)), (0.20, (120, 228, 210)),
        (0.35, (20, 170, 180)), (0.50, (10, 112, 132)), (0.70, (5, 70, 82)),
        (0.92, (0, 40, 52)), (1.00, (0, 40, 52))]),
    ("Ink", [         # sepia ink rings on paper
        (0.00, (18, 10, 6)), (0.08, (60, 35, 20)), (0.20, (128, 84, 50)),
        (0.36, (190, 150, 100)), (0.55, (224, 200, 160)), (0.80, (238, 226, 198)),
        (0.95, (244, 236, 214)), (1.00, (244, 236, 214))]),
    ("Candy", [       # hot pink rims through lilac, sky and mint to butter
        (0.00, (255, 55, 140)), (0.10, (255, 150, 200)), (0.22, (200, 165, 255)),
        (0.36, (140, 220, 255)), (0.52, (160, 250, 200)), (0.72, (255, 245, 180)),
        (0.92, (255, 215, 225)), (1.00, (255, 215, 225))]),
    ("Venom", [       # black rims, crimson, orange, gold, lime on jade
        (0.00, (0, 0, 0)), (0.06, (45, 0, 60)), (0.16, (165, 0, 80)),
        (0.30, (255, 85, 0)), (0.45, (255, 205, 0)), (0.60, (175, 255, 60)),
        (0.80, (0, 200, 120)), (0.95, (0, 92, 72)), (1.00, (0, 92, 72))]),
]


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


def rgb_table():
    out = []
    for _name, knots in PALETTES:
        for i in range(N):
            out.append(interp(i / (N - 1), knots))
    return out


def wave_table():
    cosv = [int(round(127 * math.cos(2 * math.pi * i / 256))) for i in range(256)]
    tri = [int(round(127 - 254 * min(i, 256 - i) / 128)) for i in range(256)]
    return cosv + tri


def sin_table():
    return [int(round(127 * math.sin(2 * math.pi * i / 256))) for i in range(256)]


def emit():
    words = [yuv8(c) for c in rgb_table()]
    assert len(words) == 2048

    def arr(name, typ, vals):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v & 0xFF:02X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    consts = [
        arr("C_WAVE", "t_rom512", wave_table()),
        arr("C_SIN", "t_rom256", sin_table()),
        arr("C_PY", "t_rom2k", [w[0] for w in words]),
        arr("C_PU", "t_rom2k", [w[1] for w in words]),
        arr("C_PV", "t_rom2k", [w[2] for w in words]),
    ]
    names = ", ".join(n for n, _ in PALETTES)
    txt = f"""-- marrow_pkg.vhd -- GENERATED by marrow.py; edit the generator, not this file.
--
-- C_WAVE: 512 x 8 signed, 0..255 cosine (Round), 256..511 triangle (Diamond).
-- C_SIN:  256 x 8 signed sine for the sequencer's per-line LFOs.
-- C_PY/C_PU/C_PV: eight palettes ({names}), 256 entries each,
-- address = palette & index; index 0 = the fold (outline), 255 = the clip
-- (background).  Y and chroma are 10-bit values / 4; U/V-SWAPPED for this
-- hardware (U carries Cr).

library ieee;
use ieee.std_logic_1164.all;

package marrow_pkg is
    type t_rom512 is array(0 to 511) of std_logic_vector(7 downto 0);
    type t_rom256 is array(0 to 255) of std_logic_vector(7 downto 0);
    type t_rom2k is array(0 to 2047) of std_logic_vector(7 downto 0);
{chr(10).join(consts)}
end package marrow_pkg;
"""
    with open(os.path.join(HERE, "marrow_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
