#!/usr/bin/env python3
"""Integer (RTL-exact) prototype of the per-circle cubic-Hermite cascade field.

Fixed-point conventions (candidate RTL widths):
  f accumulator : Q16 value + 16 frac guard = 32-bit signed
  df            : 26-bit signed (Q16+16 per-pixel step)
  ddf, dddf     : 22-bit signed
  distances     : Q6 sub-pixel (from isqrt remainder * recip(2r))
  reciprocals   : C_MANT-style ROM -> 12-bit mantissa, value = m >> (sh)
  engine mults  : <= 15x15 signed through one shared multiplier
Boundaries: true marks (f exactly 0/1), centre-column splits, pseudo-marks
every 256 px, and x=0 for clipped runs.
"""
import numpy as np
from PIL import Image

W, H = 480, 270
NR = 12
S = 14
cx0, cy0 = W // 2, H // 2

FQ = 16          # f value bits (1.0 == 1<<FQ)
GQ = 12          # extra guard fraction on the accumulator (HW-locked)
DQ = 6           # sub-pixel bits on distances
KNOT = 16        # PIECEWISE-LINEAR knot grid (HW-locked; sparser = banding)
LADDER = (2, 4, 8)   # kink-ladder offsets around close-tangent centre columns
RECIP_BITS = 9   # reciprocal ROM normalize width (HW-locked: 256x10 ROM)
# NOTE (2026-07-30): cubic Hermite superseded by piecewise-LINEAR at 16px
# knots — measured visually equal, engine needs no slope math at all and the
# pixel path is a single add.  MAXRUN/dd/ddd kept in the replay for
# compatibility but the linear engine emits dd=ddd=0.

def isqrt_rem(v):
    v = int(v)
    r = int(np.sqrt(max(v, 0)))
    while (r + 1) * (r + 1) <= v: r += 1
    while r * r > v: r -= 1
    return r, v - r * r

def recip_rom(v, bits=None):
    if bits is None: bits = RECIP_BITS
    """C_MANT-style reciprocal: 1/v as (mant, shift) with `bits`-bit mantissa.
    Returns integer R and shift s such that 1/v ~= R / 2^s, R < 2^bits."""
    if v <= 0: v = 1
    nb = v.bit_length()
    # normalize v to [2^(bits-1), 2^bits): idx = top bits
    if nb >= bits:
        top = v >> (nb - bits)
    else:
        top = v << (bits - nb)
    mant = ((1 << (2 * bits - 1)) + top // 2) // top      # in [2^(bits-1), 2^bits]
    s = (2 * bits - 1) + (nb - bits)
    return mant, s

def rmul(a, m, s, outq=0):
    """a * (m>>s) << outq, rounding."""
    p = a * m
    sh = s - outq
    if sh >= 0:
        return (p + (1 << (sh - 1))) >> sh if sh > 0 else p
    return p << (-sh)

def make_nest(dragmax):
    dr = [int(dragmax * (1 - k / NR) ** 1.5) for k in range(NR)]
    return [(cx0 + dr[k], cy0) for k in range(NR)], [(k + 1) * S for k in range(NR)]

def dist_q6(x, y, cx, cy):
    """distance to centre, Q6 sub-pixel, via isqrt + remainder*recip(2r+1)."""
    d2 = (x - cx) ** 2 + (y - cy) ** 2
    r, rem = isqrt_rem(d2)
    if r == 0:
        return 0
    m, s = recip_rom(2 * r + 1)
    return (r << DQ) + rmul(rem << DQ, m, s)

def engine_line(y, centers, radii):
    """Produce boundary list: (x, f_Q16, df, ddf, dddf) per sub-run start."""
    marks = []
    for k in range(NR - 1, -1, -1):
        dy = y - centers[k][1]
        if dy * dy < radii[k] ** 2:
            w, _ = isqrt_rem(radii[k] ** 2 - dy * dy)
            marks.append((centers[k][0] - w, 'L', k))
    for k in range(NR):
        dy = y - centers[k][1]
        if dy * dy < radii[k] ** 2:
            w, _ = isqrt_rem(radii[k] ** 2 - dy * dy)
            marks.append((centers[k][0] + w, 'R', k))
    subruns = []
    for i in range(len(marks) - 1):
        xa, sa, ka = marks[i]
        xb, sb, kb = marks[i + 1]
        if xb <= xa: continue
        own = ka if sa == 'L' else ka + 1
        if own >= NR: continue
        inner = own - 1
        icx, icy = centers[max(inner, 0)]
        iR = radii[inner] if inner >= 0 else 0
        ocx, ocy = centers[own]
        oR = radii[own]

        def f_at(x):
            ri = dist_q6(x, y, icx, icy)          # Q6
            ro = dist_q6(x, y, ocx, ocy)
            di = max(ri - (iR << DQ), 0)
            do = max((oR << DQ) - ro, 0)
            g = max(di + do, 1 << DQ)
            mg, sg = recip_rom(g)
            return min(max(rmul(di, mg, sg, FQ), 0), 1 << FQ)

        # knots: marks, centre columns (+ kink ladder when the scanline
        # passes close to that circle), 16px grid
        pts = {int(xa), int(xb)}
        for (cc, ccy, cR) in ((icx, icy, iR), (ocx, ocy, oR)):
            if xa + 1 < cc < xb - 1:
                pts.add(int(cc))
                approach = abs(abs(y - ccy) - cR)
                if approach < KNOT:
                    for off in LADDER:
                        for xo in (cc - off, cc + off):
                            if xa + 1 < xo < xb - 1:
                                pts.add(int(xo))
        x0 = int(xa) + KNOT
        while x0 < int(xb) - 1:
            pts.add(x0)
            x0 += KNOT
        pts = sorted(pts)
        for a, b in zip(pts[:-1], pts[1:]):
            if b <= a: continue
            fa, fb = f_at(a), f_at(b)
            mL, sL = recip_rom(b - a)
            d0 = rmul((fb - fa) << GQ, mL, sL)
            subruns.append((a, b, fa << GQ, d0, 0, 0))
    return subruns

def render_int(centers, radii):
    img = np.zeros((H, W), dtype=np.uint8)
    for y in range(H):
        for (a, b, f, d, dd, ddd) in engine_line(y, centers, radii):
            for x in range(max(a, 0), min(b, W)):
                if x > a:
                    f += d; d += dd; dd += ddd
                fv = min(max(f >> GQ, 0), 1 << FQ)
                tri = fv if fv < (1 << (FQ - 1)) else (1 << FQ) - fv
                img[y][x] = min(255, tri >> (FQ - 9))
    return img

def render_ref(centers, radii):
    xs, ys = np.meshgrid(np.arange(W, dtype=float), np.arange(H, dtype=float))
    r = np.stack([np.sqrt((xs - c[0]) ** 2 + (ys - c[1]) ** 2) for c in centers])
    R = np.array(radii, dtype=float)[:, None, None]
    inside = r < R
    j = np.where(inside.any(0), inside.argmax(0), NR)
    f = np.zeros((H, W))
    m = j == 0
    f[m] = (r[0] / radii[0])[m]
    for k in range(1, NR):
        m = j == k
        if m.any():
            di = r[k - 1][m] - radii[k - 1]
            do = radii[k] - r[k][m]
            f[m] = di / np.maximum(di + do, 1e-6)
    lum = 1 - np.abs(2 * np.clip(f, 0, 1) - 1)
    lum[j == NR] = 0
    return (lum * 255).astype(np.uint8), j

if __name__ == '__main__':
    panels = []
    for dragmax in (0, 95):
        c, R = make_nest(dragmax)
        ii = render_int(c, R)
        rr, j = render_ref(c, R)
        m = j < NR
        err = np.abs(ii.astype(int) - rr.astype(int))[m]
        print(f'drag={dragmax}: mean {err.mean():.2f} p99 {np.percentile(err,99):.0f} max {err.max()}')
        panels += [ii, rr, (np.abs(ii.astype(int) - rr.astype(int)) * 4).clip(0, 255).astype(np.uint8)]
    Image.fromarray(np.concatenate(panels, axis=0)).save(
        '/home/ron/videomancer-dev/programs/ron/cascade/percircle_int2.png')
    print('sheet per drag: INT / REF / errx4')
