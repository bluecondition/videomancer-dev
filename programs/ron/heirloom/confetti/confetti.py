#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CONFETTI v0.1.0 - build hook.  Writes confetti_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).

Fitted to the still WIN_20231123_23_27_50 @ 5.5 s (see .heirloom_work/rb_confetti/notes.md):

  columns  u = f1 (0.5 + ub (x - 315) + uc kc (x^2 - 315^2)), cells h = frac(u)     a linearly FM'd H VCO
  cards    Eh(h) + Ev(dy) < T  and  V2(dy) + A (1 - tri(S)) < 1,  S = (y - m h - s0) / P
           Eh = |(h - 0.4822) / 0.4862|^1.378, Ev = |dy / b|^1.378 (b 511 below / 433 above the centre),
           V2 = |dy2 / b2|^1.521 (b2 629 / 456), dy = the V oscillator (period 1116.5 lines) folded
  ground   phi = 4.938 P + 1.863 P^2, P = (x + 152.5)(y - y0) / 1e6  (orange / teal stripes, a hyperbolic fan)
  zone     red / navy where an oblique blob + a corner lobe + horizontal stripes (40.5 lines) pass two levels
  colour   ground / zone colour by (zone, stripe phase, brightness); a card shows (1.3 R + 150, 0, 254) of the
           colour under it (magenta over orange / red, violet over teal / navy)

Package contents (also read by .heirloom_work/rb_confetti/imodel.py):
  C_CF_PY / PU / PV  1024 x 8  ground colour Y, U, V >> 2, index {pal, zone(2), sidx(3), lv(4)}
  C_CF_CARD          256 x 24  card colour {Y, U, V} >> 2, index {pal, zone(2), sidx(3), lv(3:2)}
  C_CF_EH            256 x 12  Eh(h), Q10, h = (i + 0.5) / 256
  C_CF_POW           512 x 12  t^1.378 (0..255) and t^1.521 (256..511), Q10, t = (i mod 256) / 128
  C_CF_XR            1024 x 16 by virtual x / 2: [15:8] lobe x-term ((x + 190) / 500)^2 Q7, [7:0] corner-petal
                     x-term ((x - 2600) / 1250)^2 Q8 (both clamped at 255)
  C_CF_LV            1024 x 4  ground brightness level 0..15 on a 64 x 64 virtual-px grid, index {y / 64, x / 64}
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the shell's
13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

H0, HA, PE, QE = 0.4822, 0.4862, 1.378, 1.521

ORANGE = (150.0, 62.0, 8.0)
TEAL = (30.0, 96.0, 150.0)
RED = (155.0, 4.0, 0.0)
GOLD = (235.0, 100.0, 12.0)
NAVY = (6.0, 38.0, 58.0)


def ycc(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, (b - y) / 1.772, (r - y) / 1.402


def rgb_of(y, cb, cr):
    r = y + 1.402 * cr
    b = y + 1.772 * cb
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return tuple(max(0.0, min(255.0, c * 255.0)) for c in (r, g, b))


def ground_rgb(sidx, lv, zone, pal):
    """sidx: stripe phase bin 0..7 (bin centres (i + 0.5) / 8 of a cycle: 0 / 7 orange, 3 / 4 teal),
    lv: brightness 0..15, zone 0 ground, 1 navy, 2 red (3 = ground)."""
    gs = math.cos(2 * math.pi * (sidx + 0.5) / 8.0)
    lg = 0.55 + lv * (0.85 / 15)
    mixg = max(-1.0, min(1.0, gs * 1.6)) * 0.5 + 0.5
    g = tuple((o * mixg + t * (1 - mixg)) * lg for o, t in zip(ORANGE, TEAL))
    gd = 0.55 if sidx in (3, 4) else 1.0
    br = max(0.0, min(1.0, (lg - 1.0) / 0.35))
    rc = tuple((r * (1 - br) + o * br) * gd for r, o in zip(RED, GOLD))
    nc = tuple(n * gd for n in NAVY) if lg < 1.0 else g
    out = rc if zone == 2 else (nc if zone == 1 else g)
    out = tuple(max(0.0, min(255.0, c)) for c in out)
    if pal:                                   # Citrus: teal -> violet, red zone -> deep green-blue
        y, cb, cr = ycc(out)
        out = rgb_of(y, -cr * 0.9 + 0.05, cb * 0.9)
    return out


def card_rgb(under, pal):
    r = max(0.0, min(255.0, under[0] * 1.3 + 150))
    if pal:
        return (250.0, r * 0.9, 0.0)          # Citrus: yellow / lime cards
    return (r, 0.0, 254.0)


def hw10(rgb):
    """rgb -> hardware (Y pre-scaled 16/13, capped 940; U = Cr, V = Cb)"""
    y, cb, cr = ycc(rgb)
    yy = 64 + y * 876
    yy = 64 + (yy - 64) * 16.0 / 13.0
    yy = max(64, min(940, int(round(yy))))
    u = max(4, min(1020, int(round(512 + cr * 896))))
    v = max(4, min(1020, int(round(512 + cb * 896))))
    return yy, u, v


def pal_tables():
    py, pu, pv = [], [], []
    for pal in (0, 1):
        for zone in range(4):
            for sidx in range(8):
                for lv in range(16):
                    y, u, v = hw10(ground_rgb(sidx, lv, zone if zone < 3 else 0, pal))
                    py.append(min(255, (y + 2) >> 2)); pu.append(min(255, (u + 2) >> 2)); pv.append(min(255, (v + 2) >> 2))
    card = []
    for pal in (0, 1):
        for zone in range(4):
            for sidx in range(8):
                for lh in range(4):
                    under = ground_rgb(sidx, lh * 4 + 2, zone if zone < 3 else 0, pal)
                    y, u, v = hw10(card_rgb(under, pal))
                    card.append((min(255, (y + 2) >> 2) << 16) | (min(255, (u + 2) >> 2) << 8) | min(255, (v + 2) >> 2))
    return py, pu, pv, card


def eh_table():
    out = []
    for i in range(256):
        h = (i + 0.5) / 256.0
        out.append(min(4095, int(round(abs((h - H0) / HA) ** PE * 1024))))
    return out


def pow_table():
    out = []
    for e in (PE, QE):
        for i in range(256):
            out.append(min(4095, int(round((i / 128.0) ** e * 1024))))
    return out


def xr_table():
    out = []
    for i in range(1024):
        x = 2 * i + 1.0
        lx = min(255, int(round(((x + 190) / 500.0) ** 2 * 128)))
        ex = min(255, int(round(((x - 2600) / 1250.0) ** 2 * 256)))
        out.append((lx << 8) | ex)
    return out


def lv_table():
    out = []
    for yi in range(32):
        for xi in range(32):
            x = xi * 64 + 32.0
            y = yi * 64 + 32.0
            r1 = max(0.0, min(1.0, 1 - x / 800.0)); q1 = max(0.0, min(1.0, 1.15 - y / 900.0))
            r2 = max(0.0, min(1.0, 1 - abs(x - 1000) / 900.0)); q2 = max(0.0, min(1.0, y / 1080.0))
            lg = 0.62 + 0.75 * r1 * q1 + 0.25 * q2 * r2
            out.append(max(0, min(15, int(round((lg - 0.55) / (0.85 / 15))))))
    return out


def vhd_array(name, typ, vals, width):
    digits = (width + 3) // 4
    body = ',\n'.join('        ' + ', '.join('x"%0*X"' % (digits, v) for v in vals[i:i + 8]) for i in range(0, len(vals), 8))
    return '    constant %s : %s := (\n%s);\n' % (name, typ, body)


def main():
    py, pu, pv, card = pal_tables()
    s = ['-- Generated by confetti.py (Confetti v0.1.0): do not edit; edit confetti.py and rebuild.',
         'library ieee;', 'use ieee.std_logic_1164.all;', 'use ieee.numeric_std.all;', '',
         'package confetti_pkg is',
         '    type t_cf_u8_1k is array (0 to 1023) of unsigned(7 downto 0);',
         '    type t_cf_w24_256 is array (0 to 255) of std_logic_vector(23 downto 0);',
         '    type t_cf_u12_256 is array (0 to 255) of unsigned(11 downto 0);',
         '    type t_cf_u12_512 is array (0 to 511) of unsigned(11 downto 0);',
         '    type t_cf_u16_1k is array (0 to 1023) of unsigned(15 downto 0);',
         '    type t_cf_u4_1k is array (0 to 1023) of unsigned(3 downto 0);']
    s.append(vhd_array('C_CF_PY', 't_cf_u8_1k', py, 8))
    s.append(vhd_array('C_CF_PU', 't_cf_u8_1k', pu, 8))
    s.append(vhd_array('C_CF_PV', 't_cf_u8_1k', pv, 8))
    s.append(vhd_array('C_CF_CARD', 't_cf_w24_256', card, 24))
    s.append(vhd_array('C_CF_EH', 't_cf_u12_256', eh_table(), 12))
    s.append(vhd_array('C_CF_POW', 't_cf_u12_512', pow_table(), 12))
    s.append(vhd_array('C_CF_XR', 't_cf_u16_1k', xr_table(), 16))
    s.append(vhd_array('C_CF_LV', 't_cf_u4_1k', lv_table(), 4))
    s.append('end package confetti_pkg;')
    with open(os.path.join(HERE, 'confetti_pkg.vhd'), 'w') as f:
        f.write('\n'.join(s) + '\n')


if __name__ == '__main__':
    main()
