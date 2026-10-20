#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE ROM tables (pixel path + microcode engine).

Every table the RTL holds is generated here; the model reads the same lists,
so model and RTL agree bit for bit.  Logs are Q18 octaves, angles 2^20 per
turn.  Pixel-path tables are 256 x 32-bit words (two EBR each):

  LOG  [i] = log2(1 + i/256)            Q18 (18 b) << 11 | slope (11 b)
  UG   [k] = 0.5 log2(1 + 4^-u)         Q18 (18 b) << 14 | -slope (14 b)   u = k/16 oct
  UA   [k] = atan(2^-u)  2^20/turn      (18 b)     << 12 | -slope (12 b)
  LCOS [i] = -log2 cos(x)               Q17 (18 b) << 13 | slope (13 b)    x = i/256 quarter turn,
                                        capped at 2 octaves (x <= 75.5 deg)
  EXP  [i] = 2^(i/256) - 1              Q18 (18 b) << 11 | slope (11 b)
Engine:
  SIN  [i] = sin(i/256 quarter turn)    Q15 (the 257th point is 32767)
"""
import math

LCOS_CAP = 2 * 2 ** 17 - 1


def _pack(vals, slopes, vbits, sbits):
    out = []
    for v, s in zip(vals, slopes):
        assert 0 <= v < 2 ** vbits and 0 <= s < 2 ** sbits, (v, s)
        out.append((v << sbits) | s)
    return out


def tab_log():
    v = [round(2 ** 18 * math.log2(1 + i / 256)) for i in range(257)]
    return _pack(v[:256], [v[i + 1] - v[i] for i in range(256)], 18, 11)


def tab_ug():
    g = [round(2 ** 18 * 0.5 * math.log2(1 + 4 ** (-k / 16))) for k in range(257)]
    return _pack(g[:256], [g[k] - g[k + 1] for k in range(256)], 18, 14)


def tab_ua():
    a = [round(math.atan(2 ** (-k / 16)) / (2 * math.pi) * 2 ** 20) for k in range(257)]
    return _pack(a[:256], [a[k] - a[k + 1] for k in range(256)], 18, 12)


def tab_lcos():
    def f(i):
        c = math.cos(i / 256 * math.pi / 2)
        if c <= 0.25:
            return LCOS_CAP
        return min(LCOS_CAP, round(-2 ** 17 * math.log2(c)))
    v = [f(i) for i in range(257)]
    s = [0 if v[i] >= LCOS_CAP else min(v[i + 1], LCOS_CAP) - v[i] for i in range(256)]
    return _pack(v[:256], s, 18, 13)


def tab_exp():
    e = [round(2 ** 18 * (2 ** (i / 256) - 1)) for i in range(257)]
    return _pack(e[:256], [e[i + 1] - e[i] for i in range(256)], 18, 11)


def tab_sin():
    return [min(32767, round(32767 * math.sin(i / 256 * math.pi / 2))) for i in range(256)]


LOG, UG, UA, LCOS, EXP, SIN = tab_log(), tab_ug(), tab_ua(), tab_lcos(), tab_exp(), tab_sin()


def vhdl_rom(name, vals, width, depth=256):
    hexw = (width + 3) // 4
    vals = list(vals) + [0] * (depth - len(vals))
    items = [f'x"{v:0{hexw}X}"' for v in vals]
    rows = [', '.join(items[i:i + 8]) for i in range(0, depth, 8)]
    return (f'    constant {name} : t_rom{width}_{depth} := (\n        '
            + ',\n        '.join(rows) + ');\n')
