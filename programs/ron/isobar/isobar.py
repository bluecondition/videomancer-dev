#!/usr/bin/env python3
# Copyright (C) 2026 bluecondition
# SPDX-License-Identifier: GPL-3.0-only
"""
ISOBAR - build hook.

Generates isobar_pkg.vhd:

  C_PY / C_PU / C_PV : the contour palettes, 2048 entries each
      entry = palette(2) & row(2) & pool(1) & pos(6)
      row  : the macro cycle (ring) index mod 4, counted outward
      pool : 1 = the smooth pool at the start of the cycle, 0 = the stripes
      pos  : 64 steps across the pool, or across ONE stripe
      C_PY : luma offset, signed 4 bits (x16 codes in the pixel path)
      C_PU / C_PV : chroma offsets from neutral, signed 8 bits (x4 codes),
                    stored U/V-SWAPPED for this hardware (C_PU carries Cr)
  C_SQ : sqrt shaper, 2048 x 12.  D = sqrt(F) in 2^-9 units:
      entries 0..1023    : F = i / 1024            (the pools' fine range)
      entries 1024..2047 : F = (16 j + 8) / 1024   (F up to 16)
  C_SIN: quarter-wave sine, 256 x 8 (0..255)

Colours are written as measured-style BT.601 offsets on a 0..255 scale
(U = 0.564 (B - Y), V = 0.713 (R - Y)), then converted.  "Psyche" is fitted
to the reference capture: the stripes are a chroma-only loop along the
magenta <-> yellow-green axis (passing near neutral on the way, a narrow
ellipse), the pools are saturated gradients (blue heart with a red rim,
orange glow, sky blue, deep red).
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
POS = 64


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(len(a)))


def smooth(t):
    return t * t * (3 - 2 * t)


def stripe_loop(centre, swing, ell, ripple):
    """one stripe, pos 0..63: a soft-square cosine along +swing (pos 0 =
    the +swing pole), with a small quadrature term so the way out and the
    way back differ (the reference's narrow ellipse); luma ripple +-ripple
    (positive on the -swing pole)"""
    out = []
    for k in range(POS):
        ph = 2 * math.pi * (k + 0.5) / POS
        c = clamp(1.4 * math.cos(ph), -1.0, 1.0)
        s = math.sin(ph)
        u = centre[0] + swing[0] * c + ell[0] * s
        v = centre[1] + swing[1] * c + ell[1] * s
        out.append((-ripple * c, u, v))
    return out


def pool_ramp(e_in, knots, e_out, ydip):
    """pool, pos 0..63: blends from the incoming stripe colour e_in into the
    knot gradient (evenly spread over the middle), then out to e_out (the
    first stripe's colour).  ydip darkens the middle of the pool."""
    out = []
    n = len(knots)
    for k in range(POS):
        t = (k + 0.5) / POS
        # gradient through the knots
        g = t * (n - 1)
        i = min(int(g), n - 2)
        c = lerp(knots[i], knots[i + 1], smooth(g - i))
        # edge blends (first and last 12 %)
        if t < 0.12:
            c = lerp(e_in, c, smooth(t / 0.12))
        elif t > 0.88:
            c = lerp(c, e_out, smooth((t - 0.88) / 0.12))
        y = -ydip * math.sin(math.pi * t)
        out.append((y, c[0], c[1]))
    return out


def palette(rows):
    """rows: 4 dicts with pool knots, stripe centre/swing/ell, ripple, ydip.
    Returns 4 x (pool 64, stripes 64) as (yoff, U, V) tuples, in the ROM
    order row & pool & pos (stripes first: pool bit 0 = stripes)."""
    loops = [stripe_loop(r["centre"], r["swing"], r["ell"], r.get("ripple", 1.0)) for r in rows]
    out = []
    for i, r in enumerate(rows):
        e_in = loops[i - 1][-1][1:]           # the previous ring's last stripe
        e_out = loops[i][0][1:]
        pool = pool_ramp(e_in, r["pool"], e_out, r.get("ydip", 0.0))
        out.append(loops[i])                  # pool = 0
        out.append(pool)                      # pool = 1
    return out


# ---------------------------------------------------------------------------
# Palettes.  (U, V) on the measured 0..255 scale, unswapped BT.601.
# ---------------------------------------------------------------------------
SW = (46, 25)      # reference stripe half-swing (magenta pole at +)
EL = (0, -10)      # reference ellipse: lower V on the way to green


def psyche():
    return palette([
        dict(pool=[(-20, 52), (24, 18), (58, -26), (24, 18), (-34, 60)],   # heart: red rim, blue core
             centre=(-4, 23), swing=SW, ell=EL, ripple=1.0, ydip=1.5),
        dict(pool=[(-60, 66), (-82, 72), (-70, 40), (-82, 72), (-50, 62)],  # orange / yellow glow
             centre=(-35, 44), swing=SW, ell=EL, ripple=1.0),
        dict(pool=[(10, 10), (34, -34), (44, -40), (24, -18), (8, 18)],     # sky blue
             centre=(-4, 23), swing=SW, ell=EL, ripple=1.0),
        dict(pool=[(-14, 62), (-6, 72), (18, 56), (-6, 72), (-12, 60)],     # deep red / raspberry
             centre=(2, 32), swing=SW, ell=EL, ripple=1.0, ydip=1.0),
    ])


def ember():
    sw = (-34, 36)     # yellow <-> crimson
    return palette([
        dict(pool=[(-30, 62), (-60, 50), (-80, 30), (-60, 50), (-30, 66)],
             centre=(-30, 40), swing=sw, ell=(8, 6), ripple=1.5, ydip=1.0),
        dict(pool=[(-50, 70), (-20, 80), (0, 70), (-20, 80), (-46, 66)],
             centre=(-24, 46), swing=sw, ell=(8, 6), ripple=1.5),
        dict(pool=[(-70, 20), (-88, 30), (-70, 50), (-88, 30), (-60, 30)],
             centre=(-36, 34), swing=sw, ell=(8, 6), ripple=1.5),
        dict(pool=[(10, 60), (20, 50), (36, 40), (20, 50), (6, 60)],
             centre=(-20, 44), swing=sw, ell=(8, 6), ripple=1.5, ydip=1.5),
    ])


def ice():
    sw = (40, 30)      # magenta-violet <-> teal
    return palette([
        dict(pool=[(50, -40), (64, -10), (50, 20), (64, -10), (46, -36)],
             centre=(20, -10), swing=sw, ell=(-10, 8), ripple=1.0, ydip=1.5),
        dict(pool=[(10, -50), (30, -60), (50, -50), (30, -60), (12, -46)],
             centre=(16, -16), swing=sw, ell=(-10, 8), ripple=1.0),
        dict(pool=[(60, 10), (40, 40), (20, 50), (40, 40), (58, 14)],
             centre=(24, -4), swing=sw, ell=(-10, 8), ripple=1.0),
        dict(pool=[(-10, -40), (-30, -50), (0, -60), (-30, -50), (-8, -36)],
             centre=(12, -20), swing=sw, ell=(-10, 8), ripple=1.0, ydip=1.0),
    ])


def acid():
    # every ring turns the stripe axis by 45 degrees
    rows = []
    pools = [
        [(-60, -40), (-70, -10), (-40, -60), (-70, -10), (-56, -36)],     # lime
        [(50, 50), (70, 30), (40, 66), (70, 30), (46, 46)],               # hot pink
        [(-60, 60), (-80, 40), (-70, 70), (-80, 40), (-56, 56)],          # orange
        [(60, -30), (70, 0), (50, -50), (70, 0), (56, -28)],              # electric blue
    ]
    for i, p in enumerate(pools):
        a = math.pi / 4 * i + 0.5
        sw = (52 * math.cos(a), 52 * math.sin(a))
        rows.append(dict(pool=p, centre=(0, 0), swing=sw, ell=(-12 * math.sin(a), 12 * math.cos(a)),
                         ripple=2.0, ydip=1.0))
    return palette(rows)


PALETTES = [("Psyche", psyche), ("Ember", ember), ("Ice", ice), ("Acid", acid)]


def words():
    """2048 (y4, cr8, cb8) words, entry = pal & row & pool & pos"""
    out = []
    for _n, fn in PALETTES:
        segs = fn()
        assert len(segs) == 8
        for seg in segs:
            assert len(seg) == POS
            for (y, u, v) in seg:
                # measured 0..255 offset -> 10-bit offset (x 896/255) -> /4
                cb = clamp(int(round(u * 896 / 255 / 4)), -127, 127)
                cr = clamp(int(round(v * 896 / 255 / 4)), -127, 127)
                yo = clamp(int(round(y)), -8, 7)
                out.append((yo & 0xF, cr & 0xFF, cb & 0xFF))     # U/V swapped
    assert len(out) == 2048
    return out


def sqrt_table():
    out = []
    for i in range(1024):
        out.append(int(round(math.sqrt(i / 1024.0) * 512)))
    for j in range(1024):
        out.append(min(4095, int(round(math.sqrt((16 * j + 8) / 1024.0) * 512))))
    return out


def beat_table():
    """heartbeat envelope, 32 x 8: a 'lub' (full) then a 'dub' (60 %)"""
    out = []
    for i in range(32):
        t = (i + 0.5) / 32
        v = math.exp(-((t - 0.07) / 0.055) ** 2) + 0.6 * math.exp(-((t - 0.29) / 0.06) ** 2)
        out.append(clamp(int(round(255 * v)), 0, 255))
    return out


def sine_table():
    """quarter-wave: sin(pi/2 * (i + 0.5) / 256) * 255"""
    return [int(round(255 * math.sin(math.pi / 2 * (i + 0.5) / 256))) for i in range(256)]


def emit():
    w = words()

    def arr(name, vals, typ, digits):
        body = []
        for i in range(0, len(vals), 16):
            body.append("        " + ", ".join(f'x"{v:0{digits}X}"' for v in vals[i:i + 16]))
        return f"    constant {name} : {typ} := (\n" + ",\n".join(body) + "\n    );"

    names = ", ".join(n for n, _ in PALETTES)
    consts = "\n".join([
        arr("C_PY", [x[0] for x in w], "t_pal4", 1),
        arr("C_PU", [x[1] for x in w], "t_pal8", 2),
        arr("C_PV", [x[2] for x in w], "t_pal8", 2),
        arr("C_SQ", sqrt_table(), "t_sq12", 3),
        arr("C_SIN", sine_table(), "t_sin8", 2),
        arr("C_BEAT", beat_table(), "t_beat8", 2),
    ])
    txt = f"""-- isobar_pkg.vhd -- GENERATED by isobar.py; edit the generator, not this file.
--
-- Contour palettes ({names}): entry = palette(2) & row(2) & pool(1) & pos(6).
-- C_PY = signed 4-bit luma offset (x16), C_PU/C_PV = signed 8-bit chroma
-- offsets (x4), U/V-SWAPPED for this hardware (C_PU carries Cr).
-- C_SQ: sqrt shaper (2^-9 units; 0..1023 fine range F = i/1024, 1024..2047
-- coarse range F = (16j+8)/1024).  C_SIN: quarter-wave sine, 256 x 8.
-- C_BEAT: heartbeat envelope, 32 x 8.

library ieee;
use ieee.std_logic_1164.all;

package isobar_pkg is
    type t_pal4 is array(0 to 2047) of std_logic_vector(3 downto 0);
    type t_pal8 is array(0 to 2047) of std_logic_vector(7 downto 0);
    type t_sq12 is array(0 to 2047) of std_logic_vector(11 downto 0);
    type t_sin8 is array(0 to 255) of std_logic_vector(7 downto 0);
    type t_beat8 is array(0 to 31) of std_logic_vector(7 downto 0);
{consts}
end package isobar_pkg;
"""
    with open(os.path.join(HERE, "isobar_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
