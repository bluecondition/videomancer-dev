#!/usr/bin/env python3
"""RTL-faithful integer prototype of the PER-CIRCLE cascade gradient.

Field: f = d_in / (d_in + d_out) between each disc and the disc nested in it
(model D), streamed as model G: per scanline run between true edge marks,
  q(x)   = 1/gap lerped between the run's endpoint gaps   (engine, Q15)
  f_acc  += / -= q at every integer-radius crossing of the INNER circle
           (pixel path, adds only; sign = crossing direction)
  seeds  : f_acc = 2^15 at an own-circle mark, 0 at an inner-circle mark
Inner-circle integer-radius crossings come from an incremental r^2 tracker
re-seeded at every mark (engine provides r^2, slope, r, at the mark x).

ENGINE work per mark: 2 isqrt (distance to each bounding circle) + 1
reciprocal + 1 mult for the q slope. PIXEL work: 3 adds + compares.

Innermost disc: inner circle degenerates to the centre point (R=0).
Outer field: unchanged concentric ring phase (not modelled here).
"""
import numpy as np
from PIL import Image

W, H = 480, 270
NR = 12
S = 14
cx0, cy0 = W // 2, H // 2
FQ = 15                      # f fixed-point bits
GQ = 3                       # sub-pixel bits carried on gap values

def isqrt_rem(v):
    r = int(np.floor(np.sqrt(max(v, 0))))
    while (r + 1) * (r + 1) <= v: r += 1
    while r * r > v: r -= 1
    return r, v - r * r

def make_nest(dragmax):
    dr = [int(dragmax * (1 - k / NR) ** 1.5) for k in range(NR)]
    return [(cx0 + dr[k], cy0) for k in range(NR)], [(k + 1) * S for k in range(NR)]

def render_int(centers, radii):
    img = np.zeros((H, W), dtype=np.uint8)
    for y in range(H):
        # ---------- ENGINE (per line, integer) ----------
        marks = []   # (x, k) left edges desc-k then right edges asc-k
        for k in range(NR - 1, -1, -1):
            dy = y - centers[k][1]
            if dy * dy < radii[k] ** 2:
                w, _ = isqrt_rem(radii[k] ** 2 - dy * dy)
                marks.append(('L', centers[k][0] - w, k))
        for k in range(NR):
            dy = y - centers[k][1]
            if dy * dy < radii[k] ** 2:
                w, _ = isqrt_rem(radii[k] ** 2 - dy * dy)
                marks.append(('R', centers[k][0] + w, k))
        # runs between consecutive marks; run i spans [marks[i].x, marks[i+1].x)
        # own disc / inner disc of each run + seed values
        runs = []
        for i in range(len(marks) - 1):
            side_a, xa, ka = marks[i]
            side_b, xb, kb = marks[i + 1]
            if xb <= xa:  # degenerate
                continue
            # which annulus is this run in?
            if side_a == 'L':
                own = ka           # just entered disc ka
            else:
                own = ka + 1       # just left disc ka
            if own >= NR:
                continue           # outer field (not modelled)
            inner = own - 1        # -1 = centre point of disc 0
            # inner-circle geometry (centre point for own==0)
            icx, icy = centers[max(inner, 0)] if inner >= 0 else centers[0]
            iR = radii[inner] if inner >= 0 else 0
            dyi = y - icy
            # ENGINE sqrt #1: r_inner at run start
            ra, rema = isqrt_rem((xa - icx) ** 2 + dyi * dyi)
            # ENGINE sqrt #2: r_inner at run end
            rb, _ = isqrt_rem((xb - icx) ** 2 + dyi * dyi)
            # gaps: total annulus gap at each end (one bound sits ON a circle)
            # own-circle end -> gap = d_in there; inner-circle end -> d_out
            ocx, ocy = centers[own]
            dyo = y - ocy
            roa, _ = isqrt_rem((xa - ocx) ** 2 + dyo * dyo)
            rob, _ = isqrt_rem((xb - ocx) ** 2 + dyo * dyo)
            d_in_a, d_in_b = ra - iR, rb - iR
            d_out_a, d_out_b = radii[own] - roa, radii[own] - rob
            gA = max((d_in_a + d_out_a) << GQ, 1 << GQ)
            gB = max((d_in_b + d_out_b) << GQ, 1 << GQ)
            qA = ((1 << (FQ + GQ)) + gA // 2) // gA          # Q15 reciprocal
            qB = ((1 << (FQ + GQ)) + gB // 2) // gB
            qs_num = qB - qA
            run_len = xb - xa
            qslope = (qs_num << 8) // run_len                # Q15.8 per px
            f_seed = (d_in_a * qA)                            # Q15
            runs.append(dict(xa=xa, xb=xb, own=own,
                             r2=(xa - icx) ** 2 + dyi * dyi, r=ra,
                             sl=2 * (xa - icx) + 1,
                             fs=f_seed, qA=qA << 8, qsl=qslope))
        # ---------- PIXEL PATH (adds only) ----------
        line = np.zeros(W, dtype=np.uint8)
        for run in runs:
            r2, r, sl = run['r2'], run['r'], run['sl']
            f, q = run['fs'], run['qA']
            up2 = (r + 1) * (r + 1)
            dn2 = r * r
            for x in range(max(run['xa'], 0), min(run['xb'], W)):
                if x > run['xa']:
                    r2 += sl; sl += 2
                    q += run['qsl']
                    # integer-radius crossing detect (both directions)
                    while r2 >= up2:
                        r += 1; f += q >> 8
                        dn2 = up2; up2 = (r + 1) * (r + 1)
                    while r2 < dn2:
                        r -= 1; f -= q >> 8
                        up2 = dn2; dn2 = r * r
                fc = min(max(f, 0), (1 << FQ))
                tri = fc if fc < (1 << (FQ - 1)) else (1 << FQ) - fc  # triangle
                line[x] = min(255, (tri >> (FQ - 9)))                 # 0..255
            img[y][max(run['xa'], 0):min(run['xb'], W)] = \
                line[max(run['xa'], 0):min(run['xb'], W)]
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
        int_img = render_int(c, R)
        ref_img, j = render_ref(c, R)
        m = j < NR
        err = np.abs(int_img.astype(int) - ref_img.astype(int))[m]
        print(f'drag={dragmax}: mean err {err.mean():.2f}/255  p99 {np.percentile(err,99):.0f}  max {err.max()}')
        panels += [int_img, ref_img, (np.abs(int_img.astype(int) - ref_img.astype(int)) * 4).clip(0, 255).astype(np.uint8)]
    Image.fromarray(np.concatenate(panels, axis=0)).save(
        '/home/ron/videomancer-dev/programs/ron/cascade/percircle_int.png')
    print('sheet per drag: INT / REF / |err|x4')
