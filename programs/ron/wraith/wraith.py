#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
WRAITH - build hook.

Generates wraith_pkg.vhd:

  C_COS   1024 x 8 signed: cos(2 pi i / 1024) . 127, the pixel path's
          slave-oscillator wave.
  C_SIN   1024 x 16 signed: the sequencer's own sine for the per-line LFOs
          (fine enough that the slow vertical waves step < 0.2 % a line:
          the lobe phase u . v amplifies any step on the far side of the
          spine).
  C_SY / C_SU / C_SV   2048 x 8: eight 256-entry CYCLIC flame palettes,
          address = palette(3) & index(8).  Index 0 is the rim (where the
          flame field crosses zero); the palette wraps, so the per-line hue
          offset and Rings can run round it.
  C_BY / C_BU / C_BV   1024 x 8: eight 128-entry background ramps,
          address = palette(3) & T(7); T = 0 at the trough between tiles,
          127 at the spine.

Y is legal-range 10-bit BT.601 divided by 4; U/V are 10-bit / 4 and stored
U/V-SWAPPED for this hardware (the U register carries Cr).

"Silk" is fitted to the reference capture: an azure ramp peaking at the
spine with a dim brown fringe in the trough; violet rims, magenta, sky blue,
cyan cores, pale white-lavender, pink, peach, yellow-green and signal green.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
NS = 256         # flame palette entries
NB = 128         # background ramp entries


def interp(t, knots):
    """piecewise-linear through (t, (r, g, b)) knots, rgb 0..255"""
    if t <= knots[0][0]:
        return knots[0][1]
    for (t0, c0), (t1, c1) in zip(knots, knots[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return tuple(a + (b - a) * f for a, b in zip(c0, c1))
    return knots[-1][1]


def g(v):
    return (v, v, v)


# (name, flame knots: cyclic, 0.0 == 1.0 ; background knots: trough -> spine)
PALETTES = [
    ("Silk",
     [(0.00, (70, 22, 235)), (0.05, (140, 24, 248)), (0.10, (215, 45, 255)),
      (0.16, (150, 105, 254)), (0.23, (60, 140, 255)), (0.31, (10, 190, 255)),
      (0.39, (0, 235, 252)), (0.46, (110, 245, 245)), (0.53, (215, 232, 240)),
      (0.60, (235, 175, 225)), (0.67, (232, 140, 175)), (0.74, (215, 200, 140)),
      (0.81, (160, 245, 85)), (0.87, (0, 228, 30)), (0.93, (20, 120, 200)),
      (1.00, (70, 22, 235))],
     [(0.00, (28, 15, 8)), (0.05, (14, 15, 10)), (0.15, (11, 22, 60)),
      (0.40, (6, 44, 150)), (0.65, (3, 62, 225)), (0.80, (2, 72, 250)),
      (1.00, (0, 92, 255))]),
    ("Ember",
     [(0.00, (120, 10, 30)), (0.08, (220, 30, 20)), (0.18, (255, 110, 20)),
      (0.30, (255, 190, 60)), (0.42, (255, 240, 170)), (0.52, (255, 250, 235)),
      (0.62, (255, 200, 150)), (0.72, (240, 120, 110)), (0.82, (200, 40, 90)),
      (0.92, (150, 20, 60)), (1.00, (120, 10, 30))],
     [(0.00, (22, 8, 4)), (0.08, (10, 4, 4)), (0.40, (70, 10, 8)),
      (0.75, (150, 25, 10)), (1.00, (190, 40, 12))]),
    ("Venom",
     [(0.00, (30, 0, 60)), (0.10, (120, 0, 160)), (0.20, (230, 40, 200)),
      (0.30, (255, 140, 160)), (0.40, (255, 240, 120)), (0.50, (200, 255, 60)),
      (0.60, (80, 255, 40)), (0.70, (0, 220, 120)), (0.80, (0, 160, 160)),
      (0.90, (20, 60, 120)), (1.00, (30, 0, 60))],
     [(0.00, (12, 10, 6)), (0.08, (5, 12, 8)), (0.40, (0, 60, 40)),
      (1.00, (0, 125, 85))]),
    ("Frost",
     [(0.00, (20, 40, 140)), (0.12, (60, 120, 220)), (0.25, (140, 200, 255)),
      (0.38, (220, 240, 255)), (0.50, (255, 255, 255)), (0.62, (210, 220, 250)),
      (0.74, (170, 170, 240)), (0.86, (100, 140, 230)), (1.00, (20, 40, 140))],
     [(0.00, (12, 10, 16)), (0.08, (8, 8, 14)), (0.40, (20, 40, 80)),
      (1.00, (60, 100, 150))]),
    ("Orchid",
     [(0.00, (90, 0, 140)), (0.12, (200, 30, 200)), (0.25, (255, 90, 170)),
      (0.38, (255, 170, 160)), (0.50, (255, 235, 220)), (0.62, (240, 200, 255)),
      (0.75, (190, 140, 255)), (0.88, (140, 60, 230)), (1.00, (90, 0, 140))],
     [(0.00, (20, 8, 10)), (0.08, (12, 4, 12)), (0.40, (60, 10, 60)),
      (1.00, (120, 20, 110))]),
    ("Aurora",
     [(0.00, (0, 90, 60)), (0.12, (0, 200, 90)), (0.25, (80, 255, 140)),
      (0.38, (150, 255, 230)), (0.50, (100, 200, 255)), (0.62, (120, 110, 255)),
      (0.75, (190, 60, 240)), (0.88, (120, 20, 150)), (1.00, (0, 90, 60))],
     [(0.00, (10, 10, 6)), (0.08, (2, 6, 10)), (0.40, (10, 20, 60)),
      (1.00, (30, 30, 120))]),
    ("Spectrum",
     [(0.000, (255, 30, 40)), (0.125, (255, 140, 0)), (0.250, (250, 240, 20)),
      (0.375, (40, 240, 40)), (0.500, (0, 230, 230)), (0.625, (30, 90, 255)),
      (0.750, (140, 40, 255)), (0.875, (240, 40, 200)), (1.000, (255, 30, 40))],
     [(0.00, (14, 6, 8)), (0.08, (6, 0, 12)), (0.40, (40, 0, 70)),
      (1.00, (70, 10, 110))]),
    ("Mono",
     [(0.00, g(50)), (0.25, g(150)), (0.50, g(255)), (0.75, g(150)), (1.00, g(50))],
     [(0.00, g(16)), (0.08, g(12)), (0.40, g(40)), (1.00, g(90))]),
]


def yuv8(rgb):
    """sRGB (0..255) -> (Y10/4, Cr10/4, Cb10/4): legal-range BT.601, U/V swapped."""
    r, g_, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g_ + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g_ + 0.5 * b
    cr = 0.5 * r - 0.418688 * g_ - 0.081312 * b
    Y = 64 + 876 * y
    Cb = 512 + 896 * cb
    Cr = 512 + 896 * cr
    q = lambda v, lo, hi: max(lo, min(hi, int(round(v / 4.0))))
    # hardware swaps U/V: U register carries Cr
    return q(Y, 16, 235), q(Cr, 16, 240), q(Cb, 16, 240)


def flame_rgb():
    out = []
    for _name, knots, _bg in PALETTES:
        for i in range(NS):
            out.append(interp(i / NS, knots))          # cyclic: 1.0 == entry 0
    return out


def bg_rgb():
    out = []
    for _name, _knots, bg in PALETTES:
        for i in range(NB):
            out.append(interp(i / (NB - 1), bg))
    return out


def cos_table():
    return [int(round(127 * math.cos(2 * math.pi * i / 1024))) for i in range(1024)]


def sin_table():
    return [int(round(32767 * math.sin(2 * math.pi * i / 1024))) for i in range(1024)]


def emit():
    fw = [yuv8(c) for c in flame_rgb()]
    bw = [yuv8(c) for c in bg_rgb()]
    assert len(fw) == 2048 and len(bw) == 1024

    def arr(name, typ, vals, nib=2):
        body = []
        msk = (1 << (4 * nib)) - 1
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v & msk:0{nib}X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    consts = [
        arr("C_COS", "t_rom1k", cos_table()),
        arr("C_SIN", "t_sin1k", sin_table(), 4),
        arr("C_SY", "t_rom2k", [w[0] for w in fw]),
        arr("C_SU", "t_rom2k", [w[1] for w in fw]),
        arr("C_SV", "t_rom2k", [w[2] for w in fw]),
        arr("C_BY", "t_rom1k", [w[0] for w in bw]),
        arr("C_BU", "t_rom1k", [w[1] for w in bw]),
        arr("C_BV", "t_rom1k", [w[2] for w in bw]),
    ]
    names = ", ".join(p[0] for p in PALETTES)
    txt = f"""-- wraith_pkg.vhd -- GENERATED by wraith.py; edit the generator, not this file.
--
-- C_COS: 1024 x 8 signed cosine (the slave oscillator's wave).
-- C_SIN: 1024 x 16 signed sine for the sequencer's per-line LFOs.
-- C_SY/C_SU/C_SV: eight cyclic flame palettes ({names}), 256 entries
-- each, address = palette & index; index 0 = the rim.
-- C_BY/C_BU/C_BV: eight 128-entry background ramps, address = palette & T
-- (0 = the trough between tiles, 127 = the spine).
-- Y and chroma are 10-bit values / 4; U/V-SWAPPED for this hardware.

library ieee;
use ieee.std_logic_1164.all;

package wraith_pkg is
    type t_sin1k is array(0 to 1023) of std_logic_vector(15 downto 0);
    type t_rom1k is array(0 to 1023) of std_logic_vector(7 downto 0);
    type t_rom2k is array(0 to 2047) of std_logic_vector(7 downto 0);
{chr(10).join(consts)}
end package wraith_pkg;
"""
    with open(os.path.join(HERE, "wraith_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
