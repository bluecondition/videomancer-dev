#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
TALLOW - build hook.

Generates tallow_pkg.vhd: four 512-entry colourizer palettes, stored as
even/odd banks so the pixel path can read entries n and n+1 on the same
clock and interpolate between them (smooth bands at any slope).

  index  = palette(2) & n(9)        n = 0..511
  n 0..255   : the RAINBOW ramp  (key low:  t = n/256 down the frame)
  n 256..511 : the STRIPE ramp   (key high: t = (n-256)/256)

  C_EY/C_EU/C_EV : entries 0, 2, 4 ... (bank address = n/2), 1024 x 8
  C_OY/C_OU/C_OV : entries 1, 3, 5 ...                       1024 x 8
  C_NE/C_NO      : width noise, even / odd vertical cells x 64 columns

Y is legal-range 10-bit BT.601 divided by 4; U/V are 10-bit /4 and stored
U/V-SWAPPED for this hardware (the U register carries Cr).

Palettes are authored as sRGB functions of t in [0, 1).  "Signal" is fitted
to the reference photo (an analog colourizer: red falling, blue rising and
saturating, green a chirped triangle fold; the stripe ramp has red off, a
fast green fold and blue falling to black).
"""

import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
N = 256          # entries per ramp


def interp(t, knots):
    """piecewise-linear through (t, v) knots"""
    if t <= knots[0][0]:
        return knots[0][1]
    for (t0, v0), (t1, v1) in zip(knots, knots[1:]):
        if t <= t1:
            return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    return knots[-1][1]


def tri(phi):
    """1 at integer phi, 0 at half-integers, linear between"""
    f = phi - math.floor(phi)
    return 1.0 - 2.0 * min(f, 1.0 - f)


def clip01(v):
    return max(0.0, min(1.0, v))


def hsv(h, s, v):
    h = (h % 1.0) * 6.0
    i = int(h)
    f = h - i
    p, q, r = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, r, p), (q, v, p), (p, v, r), (p, q, v), (r, p, v), (v, p, q)][i % 6]


# ---------------------------------------------------------------------------
# palettes: each is (rainbow(t) -> rgb, stripe(t) -> rgb), rgb in 0..1
# ---------------------------------------------------------------------------
def signal_rain(t):
    r = interp(t, [(0.0, 1.0), (0.10, 0.92), (0.25, 0.88), (0.39, 0.71),
                   (0.61, 0.57), (0.81, 0.36), (1.0, 0.29)])
    b = interp(t, [(0.0, 0.03), (0.08, 0.12), (0.14, 0.24), (0.19, 0.40),
                   (0.25, 0.55), (0.33, 0.66), (0.42, 0.78), (0.47, 0.87),
                   (0.53, 1.0), (1.0, 1.0)])
    phi = 3.30 * math.log((t + 0.766) / 0.876)
    g = 0.02 + 0.94 * tri(phi)
    return r, g, b


def signal_stripe(t):
    r = clip01(0.20 - 0.72 * t)
    phi = (t - 0.042) / 0.105 + 0.5          # troughs at 0.042 + k*0.105
    tr = tri(phi)
    g = 0.06 + 0.90 * tr
    if t > 0.672:                            # the last crest falls to black
        g = 0.03
    b = interp(t, [(0.0, 1.0), (0.30, 1.0), (0.32, 0.93), (0.40, 0.76),
                   (0.50, 0.60), (0.58, 0.52), (0.66, 0.36), (0.80, 0.20),
                   (0.92, 0.07), (1.0, 0.05)])
    # mid section (Ron, v0.3): brighter, boosted greens -- wider crests at
    # full green, blue pulled out of the crests (navy only in the troughs),
    # a touch of red at the peak for a vivid lime-leaning green
    w = interp(t, [(0.26, 0.0), (0.33, 1.0), (0.62, 1.0), (0.672, 0.0)])
    if w > 0.0 and t <= 0.672:
        sh = tr ** 0.55
        g = g + w * ((0.04 + 0.96 * sh) - g)
        b = b + w * (b * (1.0 - 0.72 * sh) - b)
        r = r + w * 0.12 * sh ** 2
    return r, g, b


def ember_rain(t):
    # molten: deep red -> orange -> gold -> cream, with a slow fold in green
    r = interp(t, [(0.0, 0.55), (0.15, 0.95), (1.0, 1.0)])
    g = clip01(0.08 + 0.80 * tri(1.6 * t + 0.5) * interp(t, [(0, 0.5), (0.5, 1.0), (1, 1.0)]))
    b = interp(t, [(0.0, 0.05), (0.6, 0.10), (0.85, 0.45), (1.0, 0.75)])
    return r, g, b


def ember_stripe(t):
    # smoke: violet / magenta stripes sinking into black
    env = interp(t, [(0.0, 1.0), (0.55, 0.85), (0.9, 0.15), (1.0, 0.05)])
    f = tri((t - 0.02) / 0.09)
    r = clip01(env * (0.18 + 0.62 * f))
    g = clip01(env * 0.05)
    b = clip01(env * (0.42 + 0.30 * (1 - f)))
    return r, g, b


def glacier_rain(t):
    # ice: white -> pale cyan -> teal -> deep blue, folded in red
    r = clip01(0.10 + 0.75 * tri(1.4 * t) * interp(t, [(0, 1.0), (1, 0.35)]))
    g = interp(t, [(0.0, 0.98), (0.45, 0.80), (0.75, 0.55), (1.0, 0.30)])
    b = interp(t, [(0.0, 1.0), (0.7, 0.95), (1.0, 0.60)])
    return r, g, b


def glacier_stripe(t):
    # coral / peach stripes over plum, fading to black
    env = interp(t, [(0.0, 1.0), (0.5, 0.9), (0.85, 0.25), (1.0, 0.06)])
    f = tri((t - 0.03) / 0.1)
    r = clip01(env * (0.35 + 0.62 * f))
    g = clip01(env * (0.08 + 0.42 * f))
    b = clip01(env * (0.45 - 0.20 * f))
    return r, g, b


def acid_rain(t):
    # the whole hue wheel down the frame, full saturation
    return hsv(0.98 - 0.92 * t, 0.95, 1.0)


def acid_stripe(t):
    # black-and-white zebra, getting finer down the frame (no chroma)
    phi = 9.0 * t + 4.0 * t * t
    v = clip01(0.05 + 0.92 * tri(phi))
    return v, v, v


PALETTES = [
    ("Signal",  signal_rain,  signal_stripe),
    ("Ember",   ember_rain,   ember_stripe),
    ("Glacier", glacier_rain, glacier_stripe),
    ("Acid",    acid_rain,    acid_stripe),
]


def rgb_table():
    """4 x 512 sRGB triples (0..1)"""
    out = []
    for _name, rain, stripe in PALETTES:
        for half in (rain, stripe):
            for i in range(N):
                out.append(tuple(clip01(c) for c in half((i + 0.5) / N)))
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


def noise_table():
    """width noise: 32 vertical cells x 64 columns, uniform 0..255 (seeded)"""
    rng = random.Random(0x7A110)
    return [[rng.randrange(256) for _k in range(64)] for _j in range(32)]


def emit():
    words = [yuv8(c) for c in rgb_table()]
    assert len(words) == 2048
    banks = {}
    for bank, par in (("E", 0), ("O", 1)):
        sel = words[par::2]
        assert len(sel) == 1024
        for ch, k in (("Y", 0), ("U", 1), ("V", 2)):
            banks[f"C_{bank}{ch}"] = [w[k] for w in sel]

    def arr(name, vals):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v:02X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : t_pal8 := (\n" + ",\n".join(body) + "\n    );"

    nz = noise_table()
    banks["C_NE"] = [nz[j][k] for j in range(0, 32, 2) for k in range(64)]
    banks["C_NO"] = [nz[j][k] for j in range(1, 32, 2) for k in range(64)]

    names = ", ".join(n for n, _, _ in PALETTES)
    consts = "\n".join(arr(n, v) for n, v in banks.items())
    txt = f"""-- tallow_pkg.vhd -- GENERATED by tallow.py; edit the generator, not this file.
--
-- Four colourizer palettes ({names}), 512 entries each
-- (0..255 rainbow ramp, 256..511 stripe ramp), split into even (E) and odd
-- (O) banks of 1024 x 8 so entries n and n+1 read on one clock.  Y and the
-- chroma are 10-bit values / 4; U/V-SWAPPED for this hardware (U carries Cr).
-- C_NE/C_NO: width noise, 32 cells down x 64 columns, even / odd cell rows
-- (address = cell/2 & column), uniform 0..255.

library ieee;
use ieee.std_logic_1164.all;

package tallow_pkg is
    type t_pal8 is array(0 to 1023) of std_logic_vector(7 downto 0);
{consts}
end package tallow_pkg;
"""
    with open(os.path.join(HERE, "tallow_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
