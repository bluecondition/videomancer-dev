#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE scanline engine model (integer-exact; the RTL mirrors it).

A pattern is a FRAME TABLE of rays from an anchor.  Each ray k is a straight
boundary; the piece to its right (until the next ray) is a field that is
linear or circular (q = g (|p - c|^2 - R^2)): along any scanline that piece
is a quadratic D(x) = D0 + x s0 + q x (x - 1) / 2 with D0, s0 the value and
step AT x = 0 of this line, and D0 is itself a polynomial (degree <= 2) in
the line number while s0 is constant (q per accumulator).  So per line the
hardware LINE BUILDER steps each record by its differences (adds only),
computes the entry value at the crossing with one short multiply, and
writes a LINE TABLE of (x, D, s) breakpoints; the PIXEL PATH runs two
quadratic accumulators (A, B) reloaded at each breakpoint, keys them and
composes A -> border -> B.

Units: key units, 256 per edge width (the engine folds the EDGE gain and
the threshold into the table), so key = 128 - D.  Line-table entry:
  x   12-bit pixel (entries at x <= 0 load at pixel 0)
  D   Q.8 signed 32   value at pixel x
  s   Q.8 signed 20   per-pixel step at x (forward difference)
Ray records (7 x 32-bit words in the engine's DATA memory): x, dx Q.16 (px,
the crossing); D0 Q.8, dD0 Q.16, ddD0 Q.20 (value at x = 0); s0 Q.16 (step
at x = 0); range = active lines [lstart, lend] and the side the crossing
is on outside that range (before / after).  Nothing here depends on the
ray's slope except x, so steep rays are safe; a ray outside its range is
skipped (crossing far right) or loaded at pixel 0 (far left).  The first
record of a list is the START entry (x fixed at 0); the rest are in
increasing-x order.
"""
import numpy as np

KEYU = 256                    # key units per edge width
XF, DF, DF1, DDF, SF, QF = 16, 8, 16, 20, 16, 16
COT_MAX = 512.0
X_ACTIVE = 24000.0            # px: the crossing is tracked while |x| <= this
MIN_GAP = 5                   # px between kept line-table entries (fetcher load latency)


class Ray:
    __slots__ = ('x', 'dx', 'D', 'dD', 'ddD', 's', 'lstart', 'lend', 'left_before', 'left_after', 'start')

    def __init__(self, x, dx, D, dD, ddD, s, rng=0, start=False):
        self.x, self.dx, self.D, self.dD, self.ddD, self.s = int(x), int(dx), int(D), int(dD), int(ddD), int(s)
        self.lstart = rng & 2047
        self.lend = (rng >> 11) & 2047
        self.left_before = (rng >> 22) & 1
        self.left_after = (rng >> 23) & 1
        self.start = start


def pack_range(lstart, lend, left_before, left_after):
    return (lstart & 2047) | (lend & 2047) << 11 | (left_before & 1) << 22 | (left_after & 1) << 23


def ray_from_float(x_px, dx_px, D0, dD0, ddD0, s0, rng, start=False):
    """float quantities (pixels, key units) -> integer record"""
    return Ray(round(x_px * 2 ** XF), round(dx_px * 2 ** XF), round(D0 * 2 ** DF), round(dD0 * 2 ** DF1),
               round(ddD0 * 2 ** DDF), round(s0 * 2 ** SF), rng, start)


class Band:
    """one anchor row: an up-list for lines above ay (field line < ay) and a
    down-list for lines at/below; y_end = last field line of the band"""
    def __init__(self, ay_line, up, down, y_end):
        self.ay, self.up, self.down, self.y_end = ay_line, up, down, y_end


def step_ray(r):
    """the line builder's per-line update (in place); x wraps harmlessly
    outside the record's active range"""
    r.x = ((r.x + r.dx + 2 ** 31) % 2 ** 32) - 2 ** 31
    r.D += r.dD >> (DF1 - DF)
    r.dD += r.ddD >> (DDF - DF1)


def build_line(rays, W, line, q):
    """line builder: records (already stepped to this line) -> entries
    (xi, D, s, q): D and s evaluated at pixel xi from the x = 0 values.

    Entry spacing (the hardware fetcher needs ~MIN_GAP clocks per load):
    an entry's x is raised to (last COMMITTED x + MIN_GAP) and its D/s are
    evaluated there; an entry closer than MIN_GAP to the PENDING one replaces
    it (the pending one is dropped), else the pending one is committed.  So
    no entry with a long span is ever lost, nothing loads late, and the only
    error is the committed piece extending over a bunch of narrow ones."""
    out = []
    pend = None            # (xi, D, s, q) written in the current slot
    wx = None              # x of the last committed entry
    for r in rays:
        if r.start:
            xi = 0
        elif line < r.lstart or line > r.lend:
            left = r.left_before if line < r.lstart else r.left_after
            if not left:
                continue
            xi = 0
        else:
            xi = -((-r.x) >> XF)                       # ceil
            if xi >= W:
                continue
            if xi < 0:
                xi = 0
        if wx is not None and xi < wx + MIN_GAP:
            xi = wx + MIN_GAP
        elif wx is None and pend is not None and 0 < xi < MIN_GAP:
            xi = MIN_GAP                                 # keep the x = 0 entry in slot 0
        qx1 = q * (xi - 1)                              # Q.16
        w = r.s + (qx1 >> 1)                            # s0 + q (xi - 1) / 2, Q.16
        D = r.D + ((xi * w) >> (SF - DF))               # Q.8
        s = r.s + qx1 + q                               # s0 + q xi, Q.16
        D = max(-2 ** 31, min(2 ** 31 - 1, D))
        s = max(-2 ** 27, min(2 ** 27 - 1, s))
        e = (xi, D, s >> (SF - 8), q)
        if pend is not None and xi >= pend[0] + MIN_GAP:
            out.append(pend)
            wx = pend[0]
        pend = e
    if pend is not None:
        out.append(pend)
    return out


def run_line(entries, W):
    """pixel path: entries (xi ascending) -> D per pixel (Q.8, int64)"""
    D = np.zeros(W, dtype=np.int64)
    x = np.arange(W, dtype=np.int64)
    if not entries:
        return D
    xs = [e[0] for e in entries] + [W]
    for i, (xi, D0, s, q) in enumerate(entries):
        x1 = min(max(xs[i + 1], xi), W)
        if x1 <= xi:
            continue
        n = x - xi
        seg = slice(xi, x1)
        nn = n[seg]
        # D_n = D0 + n s + q n(n-1)/2  (q in Q.16 -> Q.8)
        D[seg] = D0 + nn * s + ((q * nn * (nn - 1)) >> 9)
    return D


def key_of(D):
    """Q.8 field -> key 0..256 (128 at the edge)"""
    k = 128 - (D >> 8)                          # the rounding half is in the field constant
    return np.clip(k, 0, 256)


class Frame:
    """a frame's tables: per accumulator (A, B) a list of bands and q; op"""
    def __init__(self, bands_a, bands_b=None, op='A', qa=0, qb=0):
        self.a, self.b, self.op, self.qa, self.qb = bands_a, bands_b or [], op, qa, qb


def render_fields(frame, W, nlines):
    """returns DA, DB arrays (nlines x W, Q.8) by stepping every record line
    by line exactly as the hardware does"""
    out = []
    for bands, q in ((frame.a, frame.qa), (frame.b, frame.qb)):
        D = np.zeros((nlines, W), dtype=np.int64)
        if not bands:
            out.append(D)
            continue
        bi = 0
        for y in range(nlines):
            while bi + 1 < len(bands) and y > bands[bi].y_end:
                bi += 1
            band = bands[bi]
            lst = band.up if y < band.ay else band.down
            first = 0 if y < band.ay else band.ay           # the list's initial line
            if y > first:
                for r in lst:
                    step_ray(r)
            D[y] = run_line(build_line(lst, W, y, q), W)
        out.append(D)
    return out


def combine(DA, DB, op):
    if op == 'A':
        return DA
    if op == 'min':
        return np.minimum(DA, DB)
    if op == 'max':
        return np.maximum(DA, DB)
    raise ValueError(op)


def compose(A, D, DK, BC, RC, rev=False):
    """A: (lines, W, 3) int YUV hardware order; D Q.8 field; DK border in key
    units; chroma co-sited per 4:2:2 pair; rev = REVERSE (key inverted)"""
    kx = D >> 8                                 # (the rounding half is in the field constant)
    kr = 128 + kx if rev else 128 - kx          # unclipped raw key
    ki = np.clip(kr, 0, 256)
    ko = np.clip(kr + DK, 0, 256)
    B = np.array(BC, dtype=np.int64)
    R = np.array(RC, dtype=np.int64)
    outer = ko < 256
    k = np.where(outer, ko, ki)
    ce = outer.copy(); ce[:, 1::2] = outer[:, 0::2][:, :ce[:, 1::2].shape[1]]
    kc = k.copy(); kc[:, 1::2] = k[:, 0::2][:, :kc[:, 1::2].shape[1]]
    out = np.empty_like(A)
    X = np.where(outer, A[..., 0], R[0]); Y = np.where(outer, R[0], B[0])
    out[..., 0] = X + (((Y - X) * k + 128) >> 8)
    for c in (1, 2):
        X = np.where(ce, A[..., c], R[c]); Y = np.where(ce, R[c], B[c])
        out[..., c] = X + (((Y - X) * kc + 128) >> 8)
    return out


def pack422(yuv):
    """the core's 4:4:4 -> 4:2:2 packer as seen on a monitor"""
    o = yuv.copy()
    W2 = o.shape[1] // 2 * 2
    o[:, 1:W2:2, 1] = o[:, 0:W2:2, 1]
    o[:, 0:W2:2, 2] = o[:, 1:W2:2, 2]
    return o
