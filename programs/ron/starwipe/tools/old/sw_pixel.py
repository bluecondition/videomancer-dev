#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE pixel pipeline, integer-exact (the RTL mirrors this stage by stage).

Units: coordinates and distances in KEY UNITS: the engine pre-scales the
raster DDA by the frame's filter gain G = 256 / w (w = edge width, at least
one pixel), so a distance of one edge width is 256 and the key is simply
128 - (D - T).  Angles 2^20 per turn; logs Q18 octaves; keys 0..256.

Field selection (port PSEL):
  0 MIX     no field: both keys = TK
  1 LIN     D = u                                (bars)
  2 BOX     D = max(|u| - BA, |v| - BB)
  3 ABSU    D = |u|                              (walk metric)
  4 ABSV    D = |v|                              (walk metric)
  8 CIRCLE  D = r
  9 STAR    D = r cos(fold_n(theta) - alpha)     (stars / polygons: exact straight edges)
"""
import numpy as np
from sw_tables import LOG, UG, UA, LCOS, EXP, LCOS_CAP

LOG_A = np.array(LOG, dtype=np.int64)
UG_A = np.array(UG, dtype=np.int64)
UA_A = np.array(UA, dtype=np.int64)
LCOS_A = np.array(LCOS, dtype=np.int64)
EXP_A = np.array(EXP, dtype=np.int64)

TURN = 1 << 20
CMAX = (1 << 20) - 1          # coordinate magnitudes saturate to 20 bits
D_MAX = (1 << 20) - 1         # D saturates to 21-bit signed
SD_MAX = (1 << 17) - 1        # sd saturates to 18-bit signed (the key range)

PSEL = {'MIX': 0, 'LIN': 1, 'BAR': 1, 'BOX': 2, 'ABSU': 3, 'ABSV': 4, 'CIRCLE': 8, 'STAR': 9}


def sat(x, m):
    return np.clip(x, -m - 1, m)


def msb(x):
    """index of the highest set bit (x >= 1, 20-bit), vectorised"""
    m = np.zeros_like(x)
    for i in range(1, 20):
        m = np.where(x >= (1 << i), i, m)
    return m


def log2_unit(x):
    """x in 1..2^20-1 -> log2 x, Q18 (msb << 18 + PWL of 16 mantissa bits)"""
    m = msb(x)
    n = np.where(m <= 16, x << np.maximum(16 - m, 0), x >> np.maximum(m - 16, 0)) & 0xFFFF
    i, f = n >> 8, n & 255
    w = LOG_A[i]
    return (m << 18) + (w >> 11) + (((w & 2047) * f + 128) >> 8)


def polar(u, v):
    """(u, v) -> (log2 r Q18, theta 2^20/turn; v grows downward, so clockwise)"""
    au = np.minimum(np.abs(u), CMAX)
    av = np.minimum(np.abs(v), CMAX)
    sw = av > au
    hi = np.maximum(np.where(sw, av, au), 1)
    lo = np.where(sw, au, av)
    lz = lo == 0
    lh = log2_unit(hi)
    ll = log2_unit(np.maximum(lo, 1))
    du = np.where(lz, 16 * 2 ** 18 - 1, np.minimum(lh - ll, 16 * 2 ** 18 - 1))
    ia, fa = du >> 14, (du >> 2) & 4095
    wg, wa = UG_A[ia], UA_A[ia]
    g = (wg >> 14) - (((wg & 16383) * fa + 2048) >> 12)
    a = (wa >> 12) - (((wa & 4095) * fa + 2048) >> 12)
    lr = lh + g
    q = TURN >> 2
    phi = np.where(sw, q - a, a)                      # angle from the +u axis
    su, sv = u < 0, v < 0
    th = np.where(~su & ~sv, phi,
         np.where(su & ~sv, 2 * q - phi,
         np.where(su & sv, 2 * q + phi, 4 * q - phi)))
    return lr, th & (TURN - 1)


def star_logcos(th, n, S, alpha):
    """-log2 cos(fold_n(theta) - alpha), Q18; S = round(2^20 / n), alpha 2^20/turn"""
    k = (th * n) >> 20
    ph = np.clip(th - k * S, 0, S)
    ph = np.minimum(ph, S - ph)                       # mirror about the sector bisector
    x = np.minimum(np.abs(ph - alpha), (TURN >> 2) - 1)
    ix, fx = x >> 10, x & 1023
    w = LCOS_A[ix]
    lc = (w >> 13) + (((w & 8191) * fx + 512) >> 10)  # Q17
    return np.minimum(lc, LCOS_CAP) << 1


def exp2_unit(L):
    """L (Q18 octaves of key units) -> 2^L, rounded, saturated"""
    L = np.maximum(L, 0)
    ie = L >> 18
    i, f = (L >> 10) & 255, L & 1023
    w = EXP_A[i]
    m = (1 << 18) + (w >> 11) + (((w & 2047) * f + 512) >> 10)      # 19-bit mantissa
    sh = np.maximum(18 - ie, 0)
    d = np.where(ie <= 18, (m + ((1 << sh) >> 1)) >> sh, np.where(ie == 19, m << 1, D_MAX))
    return np.minimum(d, D_MAX)


def field(u, v, P):
    """per-pixel D (u-units, signed) for the frame's PSEL"""
    psel = P['PSEL']
    if psel == 1:
        return sat(u, D_MAX)
    if psel == 2:
        return sat(np.maximum(np.abs(u) - P['BA'], np.abs(v) - P['BB']), D_MAX)
    if psel == 3:
        return sat(np.abs(u), D_MAX)
    if psel == 4:
        return sat(np.abs(v), D_MAX)
    if psel in (8, 9):
        lr, th = polar(u, v)
        if psel == 9:
            lr = lr - star_logcos(th, P['SN'], P['SS'], P['SAL'])
        return exp2_unit(lr)
    return np.zeros_like(u)


def keys(D, P):
    """D (key units) -> (k_inner, k_outer), 0..256"""
    if P['PSEL'] == 0:
        k = np.full(D.shape, P['TK'], dtype=np.int64)
        return k, k
    sd = (P['T'] - D) if P['REV'] else (D - P['T'])
    kr = 128 - sat(sd, SD_MAX)
    return np.clip(kr, 0, 256), np.clip(kr + P['DK'], 0, 256)


def lerp(x, y, k):
    return x + (((y - x) * k + 128) >> 8)


def compose(A, ki, ko, P):
    """A: (lines, W, 3) int YUV in hardware order; B/border colours per frame.
    Luma per pixel; chroma CO-SITED: both samples of a 4:2:2 pair (U taken
    on the even pixel, V on the odd one by the core's packer) use the EVEN
    pixel's endpoints and weight, so coloured edges never fringe."""
    B = np.array(P['BC'], dtype=np.int64)
    R = np.array(P['RC'], dtype=np.int64)
    outer = ko < 256
    k = np.where(outer, ko, ki)
    ce = outer.copy(); ce[:, 1::2] = outer[:, 0::2][:, :outer[:, 1::2].shape[1]]
    kc = k.copy(); kc[:, 1::2] = k[:, 0::2][:, :k[:, 1::2].shape[1]]
    out = np.empty_like(A)
    X = np.where(outer, A[..., 0], R[0]); Y = np.where(outer, R[0], B[0])
    out[..., 0] = lerp(X, Y, k)
    for c in (1, 2):
        X = np.where(ce, A[..., c], R[c]); Y = np.where(ce, R[c], B[c])
        out[..., c] = lerp(X, Y, kc)
    return out


def pack422(yuv):
    """the core's 4:4:4 -> 4:2:2 packer as seen on a monitor: U from the even
    pixel, V from the odd one, held across the pair"""
    o = yuv.copy()
    W2 = o.shape[1] // 2 * 2
    o[:, 1:W2:2, 1] = o[:, 0:W2:2, 1]
    o[:, 0:W2:2, 2] = o[:, 1:W2:2, 2]
    return o
