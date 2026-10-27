#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
TOTEM - build hook.

Generates totem_pkg.vhd: four colourizer palettes of 4 groups x 16 rows x
8 ramp knots (512 entries each), stored as even/odd banks so the pixel path
reads knots k and k+1 on one clock and interpolates between them.

  entry = palette(2) & group(2) & row(4) & knot(3)
  group 0 CURTAIN : the backdrop bands (row = band index)
  group 1 SHEET   : the same bands seen through the diagonal sheet
  group 2 LENS    : the totem's even slats
  group 3 FLAME   : the totem's odd slats

Each row is one band of a sawtooth: knot 0 is the band's dark, saturated
start; knot 7 its bright, paler end.  The pixel path only interpolates
between knots 0..7 of ONE row (ramp position r*7/256), so a band edge is
always a hard step to the next row.

  C_EY/C_EU/C_EV : entries 0, 2, 4 ... (bank address = entry/2), 1024 x 8
  C_OY/C_OU/C_OV : entries 1, 3, 5 ...                           1024 x 8
  C_SIN          : quarter-wave sine, 256 x 8 (0..255)

Y is legal-range 10-bit BT.601 divided by 4; U/V are 10-bit /4 and stored
U/V-SWAPPED for this hardware (the U register carries Cr).

"Signal" is fitted to the reference capture (an analog colourizer: cool
curtain bands that brighten and pale toward green across each band, a warm
hue-shifted diagonal sheet, red-ribbon lens slats with pastel swirls, flat
red/orange flame slats with dark seams).
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
KNOTS = 8
ROWS = 16


def clip01(v):
    return max(0.0, min(1.0, v))


def lerp3(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def ramp(c0, c1, c2, ease=0.75):
    """8 knots: a lit tube.  Knot 0 = the dark, saturated start; rising
    through c1 to the peak c2 at knot 5 (71 % across the band); then a
    slight fall into the next band's edge (softens the hard step)"""
    out = []
    for k in range(KNOTS):
        if k <= 5:
            t = (k / 5) ** ease
            c = lerp3(c0, c1, t * 2) if t < 0.5 else lerp3(c1, c2, t * 2 - 1)
        else:
            c = tuple(v * (1.0 - 0.09 * (k - 5)) for v in c2)
        out.append(tuple(clip01(v / 255.0) for v in c))
    return out


def flat(c, dark=0.38, edge=None):
    """flame row: a dark seam on knot 0, then (nearly) flat colour"""
    e = edge if edge is not None else c
    out = [tuple(clip01(v * dark / 255.0) for v in c)]
    for k in range(1, KNOTS):
        t = (k - 1) / (KNOTS - 2)
        out.append(tuple(clip01(v / 255.0) for v in lerp3(c, e, t)))
    return out


def to_yuv(rgb):
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b
    return y, cb, cr


def from_yuv(y, cb, cr):
    r = y + 1.402 * cr
    g = y - 0.344136 * cb - 0.714136 * cr
    b = y + 1.772 * cb
    return clip01(r), clip01(g), clip01(b)


def hue_turn(rgb, turns, sat=1.0, lift=0.0):
    """rotate the chroma of an sRGB colour by `turns` of a circle"""
    y, cb, cr = to_yuv(rgb)
    a = 2 * math.pi * turns
    cb2 = (cb * math.cos(a) - cr * math.sin(a)) * sat
    cr2 = (cb * math.sin(a) + cr * math.cos(a)) * sat
    return from_yuv(clip01(y + lift), cb2, cr2)


def hsv(h, s, v):
    h = (h % 1.0) * 6.0
    i = int(h)
    f = h - i
    p, q, r = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, r, p), (q, v, p), (p, v, r), (p, q, v), (r, p, v), (v, p, q)][i % 6]


# ---------------------------------------------------------------------------
# Signal: fitted to the reference
# ---------------------------------------------------------------------------
SIG_CURTAIN = [
    ramp((0, 46, 0), (0, 150, 0), (0, 240, 2)),            # green
    ramp((52, 34, 98), (66, 78, 100), (76, 120, 100)),      # violet -> teal
    ramp((78, 12, 158), (84, 62, 184), (88, 104, 190)),     # violet -> periwinkle
    ramp((128, 8, 190), (172, 40, 226), (214, 82, 242)),    # magenta
    ramp((48, 96, 250), (70, 160, 210), (108, 214, 142)),   # blue -> green
    ramp((14, 36, 8), (28, 110, 36), (46, 178, 66)),        # moss
    ramp((64, 24, 154), (92, 82, 126), (104, 154, 162)),    # violet -> grey-cyan
    ramp((50, 42, 150), (46, 100, 128), (56, 126, 136)),    # blue -> teal
    ramp((4, 64, 4), (10, 168, 8), (16, 244, 14)),          # bright green
    ramp((104, 6, 196), (116, 30, 222), (128, 54, 238)),    # violet -> lavender
    ramp((38, 38, 236), (60, 72, 210), (100, 118, 170)),    # deep blue
    ramp((72, 24, 108), (96, 100, 150), (126, 164, 180)),   # violet -> pale blue
    ramp((146, 6, 210), (166, 26, 230), (188, 48, 242)),    # magenta-violet
    ramp((36, 86, 244), (66, 108, 190), (106, 116, 114)),   # blue -> grey
    ramp((42, 38, 80), (60, 110, 110), (80, 166, 130)),     # dusk -> teal
    ramp((92, 12, 150), (100, 40, 154), (110, 62, 156)),    # periwinkle
]

SIG_LENS = [
    ramp((96, 12, 10), (196, 10, 8), (244, 78, 6)),         # red ribbon
    ramp((222, 180, 162), (140, 192, 162), (54, 232, 194)), # pastel -> cyan
    ramp((84, 14, 10), (176, 14, 9), (240, 44, 6)),         # red ribbon
    ramp((132, 34, 160), (190, 60, 92), (212, 78, 70)),     # violet -> salmon
    ramp((98, 13, 8), (166, 12, 8), (212, 56, 48)),         # red ribbon
    ramp((202, 102, 64), (180, 152, 72), (164, 202, 78)),   # orange -> lime
    ramp((90, 14, 11), (210, 8, 6), (245, 76, 5)),          # red ribbon
    ramp((143, 136, 151), (103, 156, 165), (68, 206, 182)), # grey -> teal
    ramp((72, 14, 28), (120, 14, 52), (136, 42, 62)),       # dark magenta
    ramp((196, 158, 61), (208, 88, 62), (166, 56, 108)),    # gold -> rose
    ramp((88, 12, 10), (190, 12, 8), (242, 60, 6)),         # red ribbon
    ramp((53, 232, 194), (130, 200, 170), (224, 190, 150)), # cyan -> cream
    ramp((100, 14, 8), (200, 10, 6), (246, 96, 4)),         # red ribbon
    ramp((233, 142, 11), (243, 120, 0), (246, 58, 8)),      # orange swirl
    ramp((92, 13, 9), (180, 12, 8), (236, 40, 10)),         # red ribbon
    ramp((40, 168, 40), (130, 190, 70), (200, 96, 160)),    # green -> pink
]

SIG_FLAME = [
    flat((240, 10, 0)),
    flat((222, 64, 0)),
    flat((210, 100, 90), edge=(226, 82, 60)),
    flat((236, 10, 2)),
    flat((232, 118, 0), edge=(244, 166, 0)),
    flat((184, 10, 6)),
    flat((243, 88, 2)),
    flat((238, 6, 0)),
    flat((214, 40, 30)),
    flat((243, 160, 0), edge=(200, 170, 0)),
    flat((240, 12, 0)),
    flat((220, 70, 40)),
    flat((236, 8, 0)),
    flat((246, 120, 0)),
    flat((200, 12, 4)),
    flat((232, 96, 72)),
]


def sheet_of(curtain, turns=0.5, sat=1.0, lift=0.0):
    return [[hue_turn(c, turns, sat, lift) for c in row] for row in curtain]


def signal():
    return [SIG_CURTAIN, sheet_of(SIG_CURTAIN, -0.15, 1.35, 0.05), SIG_LENS, SIG_FLAME]


def ember():
    """Signal with every hue turned half a circle: warm curtains, icy totem"""
    return [sheet_of(g, 0.5, 1.2) for g in signal()]


def neon():
    """full-saturation bands stepping round the wheel; a white-hot totem"""
    cur = []
    for i in range(ROWS):
        h = (i * 0.381966) % 1.0
        cur.append([hsv(h + 0.04 * k / 7, 1.0 - 0.35 * k / 7, 0.18 + 0.80 * (k / 7) ** 0.8)
                    for k in range(KNOTS)])
    lens = []
    for i in range(ROWS):
        h = 0.83 if i % 2 == 0 else 0.50
        lens.append([hsv(h, 0.95 - 0.5 * k / 7, 0.25 + 0.72 * k / 7) for k in range(KNOTS)])
    flame = []
    for i in range(ROWS):
        c = [(1.0, 1.0, 0.96), (1.0, 0.92, 0.45), (1.0, 0.78, 0.25), (0.95, 0.95, 1.0)][i % 4]
        flame.append(flat(tuple(v * 255 for v in c), dark=0.25))
    return [cur, sheet_of(cur, 0.5), lens, flame]


def dusk():
    """pastel pinks / lavender / peach curtains, an indigo sheet, a gold totem"""
    cur = []
    hues = [0.92, 0.78, 0.05, 0.70, 0.97, 0.60, 0.85, 0.08]
    for i in range(ROWS):
        h = hues[i % 8] + 0.01 * (i // 8)
        cur.append([hsv(h, 0.75 - 0.45 * k / 7, 0.30 + 0.62 * k / 7) for k in range(KNOTS)])
    sheet = [[hsv(0.68 + 0.03 * (i % 3), 0.85 - 0.3 * k / 7, 0.16 + 0.55 * k / 7)
              for k in range(KNOTS)] for i in range(ROWS)]
    lens = []
    for i in range(ROWS):
        if i % 2 == 0:
            lens.append([hsv(0.12, 0.9 - 0.4 * k / 7, 0.30 + 0.68 * k / 7) for k in range(KNOTS)])
        else:
            lens.append([hsv(0.50, 0.55 - 0.3 * k / 7, 0.35 + 0.55 * k / 7) for k in range(KNOTS)])
    flame = [flat(tuple(v * 255 for v in hsv(0.10 + 0.015 * (i % 4), 0.85, 0.95)), dark=0.3)
             for i in range(ROWS)]
    return [cur, sheet, lens, flame]


PALETTES = [("Signal", signal), ("Ember", ember), ("Neon", neon), ("Dusk", dusk)]


def rgb_table():
    """4 x 512 sRGB triples (0..1), entry = pal & group & row & knot"""
    out = []
    for _name, fn in PALETTES:
        groups = fn()
        assert len(groups) == 4
        for g in groups:
            assert len(g) == ROWS
            for row in g:
                assert len(row) == KNOTS
                out.extend(tuple(clip01(c) for c in rgb) for rgb in row)
    assert len(out) == 2048
    return out


def yuv8(rgb):
    """sRGB (0..1) -> (Y10/4, Cr10/4, Cb10/4): legal-range BT.601, U/V swapped.
    Drawn luma is capped at ~768 (Y8 192) so no colour clips RGB."""
    y, cb, cr = to_yuv(rgb)
    Y = 64 + 876 * y
    Cb = 512 + 896 * cb
    Cr = 512 + 896 * cr
    q = lambda v, lo, hi: max(lo, min(hi, int(round(v / 4.0))))
    return q(Y, 16, 235), q(Cr, 16, 240), q(Cb, 16, 240)


def sine_table():
    """quarter-wave: sin(pi/2 * (i + 0.5) / 256) * 255"""
    return [int(round(255 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]


def emit():
    words = [yuv8(c) for c in rgb_table()]
    banks = {}
    for bank, par in (("E", 0), ("O", 1)):
        sel = words[par::2]
        assert len(sel) == 1024
        for ch, k in (("Y", 0), ("U", 1), ("V", 2)):
            banks[f"C_{bank}{ch}"] = [w[k] for w in sel]

    def arr(name, vals, typ):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v:02X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    names = ", ".join(n for n, _ in PALETTES)
    consts = "\n".join(arr(n, v, "t_pal8") for n, v in banks.items())
    consts += "\n" + arr("C_SIN", sine_table(), "t_sin8")
    txt = f"""-- totem_pkg.vhd -- GENERATED by totem.py; edit the generator, not this file.
--
-- Four colourizer palettes ({names}): 4 groups (curtain, sheet, lens,
-- flame) x 16 rows x 8 ramp knots = 512 entries each, split into even (E)
-- and odd (O) banks of 1024 x 8 so knots k and k+1 read on one clock.  Y and
-- the chroma are 10-bit values / 4; U/V-SWAPPED for this hardware (U carries
-- Cr).  C_SIN: quarter-wave sine, 256 x 8.

library ieee;
use ieee.std_logic_1164.all;

package totem_pkg is
    type t_pal8 is array(0 to 1023) of std_logic_vector(7 downto 0);
    type t_sin8 is array(0 to 255) of std_logic_vector(7 downto 0);
{consts}
end package totem_pkg;
"""
    with open(os.path.join(HERE, "totem_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
