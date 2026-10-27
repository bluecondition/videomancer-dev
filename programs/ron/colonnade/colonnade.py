#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
COLONNADE - build hook.

Generates colonnade_pkg.vhd:

  * four palettes of 512 entries, stored as even/odd banks so the pixel path
    reads entries n and n+1 on one clock and interpolates between them:

      index = palette(2) & group(1) & n(8)
      group 0 : SHEET  - the cyclic contour ramp (the field's rings)
      group 1 : FLUTE  - the lit-tube ramp across one pillar (cyclic too)

    C_EY/C_EU/C_EV : entries 0, 2, 4 ... (bank address = index/2), 1024 x 8
    C_OY/C_OU/C_OV : entries 1, 3, 5 ...                             1024 x 8

  * C_SINQ : a quarter-wave sine, 256 x 12 bits (amplitude 2047), sampled at
    (i + 0.5)/256 of a quarter turn so the mirrored quadrants meet cleanly.

Y is legal-range 10-bit BT.601 / 4; U/V are 10-bit / 4 and stored U/V-SWAPPED
for this hardware (the U register carries Cr).

Palettes are authored as cyclic sRGB knot lists (t in [0, 1), wrapping).
"Chrome" is fitted to the reference capture: navy -> blue -> white -> yellow
-> orange -> red plateau -> purple plateau -> magenta plateau -> violet ->
cyan plateau -> blue -> black; its flutes are lit from the left: a crimson
edge, a cream highlight, pink-purple, blue, and a black shadow.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
N = 256


def cyc(knots):
    """cyclic piecewise-linear palette from (t, (r, g, b)) knots, 0..255 sRGB"""
    ks = sorted(knots)
    ts = [k[0] for k in ks] + [ks[0][0] + 1.0]
    cs = [k[1] for k in ks] + [ks[0][1]]
    out = []
    for i in range(N):
        t = (i + 0.5) / N
        if t < ts[0]:
            t += 1.0
        for j in range(len(ts) - 1):
            if ts[j] <= t <= ts[j + 1]:
                f = (t - ts[j]) / (ts[j + 1] - ts[j])
                out.append(tuple((cs[j][c] + (cs[j + 1][c] - cs[j][c]) * f) / 255.0
                                 for c in range(3)))
                break
    assert len(out) == N
    return out


# --- Chrome: fitted to the reference -------------------------------------
CHROME_SHEET = cyc([
    (0.00, (5, 7, 5)),
    (0.06, (6, 38, 50)),
    (0.13, (3, 46, 96)),
    (0.19, (2, 62, 150)),
    (0.24, (30, 100, 200)),
    (0.285, (195, 212, 208)),   # white highlight
    (0.32, (245, 236, 90)),     # yellow
    (0.36, (240, 140, 40)),     # orange
    (0.395, (254, 22, 0)),      # red plateau
    (0.445, (250, 8, 0)),
    (0.48, (140, 10, 24)),      # dark red
    (0.51, (118, 0, 88)),       # purple plateau
    (0.56, (118, 0, 92)),
    (0.60, (255, 0, 254)),      # magenta plateau
    (0.665, (255, 0, 250)),
    (0.70, (70, 40, 250)),      # violet
    (0.73, (5, 200, 255)),      # cyan plateau
    (0.78, (3, 215, 255)),
    (0.82, (15, 50, 245)),      # blue
    (0.87, (10, 20, 110)),      # indigo
    (0.93, (30, 8, 36)),
])
CHROME_FLUTE = cyc([
    (0.00, (8, 5, 22)),
    (0.035, (200, 0, 80)),      # crimson lit edge
    (0.09, (215, 95, 62)),
    (0.15, (232, 196, 76)),     # cream highlight
    (0.20, (226, 212, 135)),
    (0.26, (214, 120, 92)),
    (0.32, (190, 64, 112)),     # pink
    (0.40, (128, 52, 165)),     # purple
    (0.48, (40, 14, 150)),
    (0.57, (8, 6, 165)),        # blue
    (0.70, (5, 4, 118)),
    (0.84, (4, 3, 48)),
    (0.93, (6, 5, 10)),         # black shadow
])

# --- Oil: thin-film iridescence ---------------------------------------------
OIL_SHEET = cyc([
    (0.00, (4, 6, 8)),
    (0.08, (8, 40, 48)),        # deep teal
    (0.16, (20, 110, 100)),
    (0.23, (120, 170, 60)),     # green-gold
    (0.29, (230, 200, 70)),     # gold
    (0.34, (240, 150, 60)),
    (0.40, (200, 50, 120)),     # magenta
    (0.47, (120, 40, 190)),     # violet
    (0.54, (40, 60, 200)),      # blue
    (0.61, (30, 170, 200)),     # turquoise
    (0.67, (150, 230, 190)),    # mint flash
    (0.72, (230, 220, 120)),
    (0.78, (200, 110, 40)),     # bronze
    (0.85, (90, 30, 60)),
    (0.92, (20, 10, 30)),
])
OIL_FLUTE = cyc([
    (0.00, (6, 4, 10)),
    (0.04, (150, 90, 20)),      # bronze edge
    (0.11, (240, 200, 90)),     # gold highlight
    (0.18, (200, 230, 160)),
    (0.26, (90, 170, 140)),     # verdigris
    (0.36, (40, 90, 110)),
    (0.48, (60, 30, 110)),      # violet shadow
    (0.62, (30, 14, 70)),
    (0.80, (12, 6, 30)),
    (0.93, (5, 3, 8)),
])

# --- Candy: pastel sugar glass ------------------------------------------------
CANDY_SHEET = cyc([
    (0.00, (250, 245, 250)),    # white
    (0.08, (255, 180, 215)),    # pink
    (0.17, (240, 100, 170)),    # rose
    (0.25, (190, 140, 240)),    # lilac
    (0.33, (130, 190, 255)),    # sky
    (0.41, (120, 235, 200)),    # mint
    (0.49, (200, 250, 150)),
    (0.56, (255, 240, 120)),    # lemon
    (0.64, (255, 190, 130)),    # peach
    (0.72, (255, 120, 120)),    # coral
    (0.80, (200, 90, 200)),     # orchid
    (0.88, (150, 120, 230)),
    (0.95, (220, 200, 250)),
])
CANDY_FLUTE = cyc([
    (0.00, (120, 60, 130)),
    (0.04, (255, 120, 180)),    # pink edge
    (0.11, (255, 240, 245)),    # sugar highlight
    (0.19, (190, 245, 220)),    # mint
    (0.30, (120, 210, 220)),
    (0.44, (110, 140, 230)),
    (0.60, (130, 90, 200)),     # lilac shadow
    (0.78, (100, 50, 150)),
    (0.92, (70, 30, 100)),
])

# --- Thermal: heat camera ------------------------------------------------------
THERMAL_SHEET = cyc([
    (0.00, (0, 0, 0)),
    (0.08, (30, 0, 90)),        # indigo
    (0.17, (110, 0, 150)),      # purple
    (0.26, (200, 0, 110)),      # magenta
    (0.35, (240, 40, 20)),      # red
    (0.44, (255, 130, 0)),      # orange
    (0.52, (255, 220, 30)),     # yellow
    (0.59, (255, 255, 220)),    # white hot
    (0.65, (255, 220, 30)),
    (0.72, (240, 90, 0)),
    (0.80, (160, 0, 60)),
    (0.88, (60, 0, 90)),
    (0.95, (10, 0, 30)),
])
THERMAL_FLUTE = cyc([
    (0.00, (0, 4, 20)),
    (0.04, (40, 120, 220)),     # ice edge
    (0.11, (230, 250, 255)),    # frost highlight
    (0.18, (120, 230, 255)),    # cyan
    (0.30, (30, 140, 230)),
    (0.44, (10, 60, 170)),      # blue
    (0.60, (5, 25, 100)),
    (0.78, (2, 8, 45)),
    (0.92, (0, 2, 12)),
])

PALETTES = [
    ("Chrome", CHROME_SHEET, CHROME_FLUTE),
    ("Oil", OIL_SHEET, OIL_FLUTE),
    ("Candy", CANDY_SHEET, CANDY_FLUTE),
    ("Thermal", THERMAL_SHEET, THERMAL_FLUTE),
]


def rgb_table():
    """4 palettes x (sheet, flute) x 256 sRGB triples (0..1)"""
    out = []
    for _name, sheet, flute in PALETTES:
        out.extend(sheet)
        out.extend(flute)
    return out


def yuv8(rgb):
    """sRGB (0..1) -> (Y10/4, Cr10/4, Cb10/4): legal-range BT.601, U/V swapped."""
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    Y = 64 + 876 * y
    Cb = 512 + 896 * cb
    Cr = 512 + 896 * cr
    q = lambda v, lo, hi: max(lo, min(hi, int(round(v / 4.0))))
    # hardware swaps U/V: U register carries Cr
    return q(Y, 16, 235), q(Cr, 16, 240), q(Cb, 16, 240)


def sine_quarter():
    return [int(round(2047 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]


def emit():
    words = [yuv8(c) for c in rgb_table()]
    assert len(words) == 2048
    banks = {}
    for bank, par in (("E", 0), ("O", 1)):
        sel = words[par::2]
        assert len(sel) == 1024
        for ch, k in (("Y", 0), ("U", 1), ("V", 2)):
            banks[f"C_{bank}{ch}"] = [w[k] for w in sel]

    def arr(name, vals, typ, fmt):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(fmt(v) for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    consts = "\n".join(arr(n, v, "t_pal8", lambda v: f'x"{v:02X}"') for n, v in banks.items())
    sq = arr("C_SINQ", sine_quarter(), "t_sinq", lambda v: f'x"{v:03X}"')
    names = ", ".join(n for n, _, _ in PALETTES)
    txt = f"""-- colonnade_pkg.vhd -- GENERATED by colonnade.py; edit the generator, not this file.
--
-- Four palettes ({names}), 512 entries each: 0..255 the
-- SHEET contour ramp, 256..511 the FLUTE lit-tube ramp (both cyclic), split
-- into even (E) and odd (O) banks of 1024 x 8 so entries n and n+1 read on
-- one clock.  Y and the chroma are 10-bit values / 4; U/V-SWAPPED for this
-- hardware (U carries Cr).  C_SINQ: quarter-wave sine, 256 x 12, amplitude
-- 2047, sampled at (i + 0.5) / 256 of a quarter turn.

library ieee;
use ieee.std_logic_1164.all;

package colonnade_pkg is
    type t_pal8 is array(0 to 1023) of std_logic_vector(7 downto 0);
    type t_sinq is array(0 to 255) of std_logic_vector(11 downto 0);
{consts}
{sq}
end package colonnade_pkg;
"""
    with open(os.path.join(HERE, "colonnade_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
