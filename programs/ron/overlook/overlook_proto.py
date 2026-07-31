#!/usr/bin/env python3
"""Overlook — Shining-carpet pattern prototype (v22: the user's exact
vector spec).

The orange figure is a set of identical horizontal SERPENTINE polylines
stacked vertically (same phase).  Hexagon "cells" are negative space;
solid red hexagons float in the dark field, touching nothing.

User tile: 100 x 142 px, stroke 12.5, MITER joins.  Scaled x10.24 so one
period = 1024 texture units (stroke = 128 exactly).  Half-tile polyline
(x-fold symmetric about 0 and 512):

    A(0,0) B(374,210) C(374,650) D(143,773) E(143,1213) F(512,1423)

Rows every 1454.  Red hexagons: regular pointy-top, circumradius 169,
at (0, 430) and (512, 993).

Stroke model (exact miter): per segment, the perpendicular slab of
half-width 64, clipped at each interior join by the line through the
vertex and the miter point (direction = sum of the two outer normals).
The x-fold handles the apex/tip mirror joins automatically.
Run with --consts to print the quantized plane constants for the VHDL.
"""
import numpy as np
from PIL import Image
import sys

# ---- geometry (texture units) ----
PERX = 1024.0
PERY = 1454.0
HS   = 64.0          # half stroke
VERTS = [(0.0, 0.0), (374.0, 210.0), (374.0, 650.0),
         (143.0, 773.0), (143.0, 1213.0), (512.0, 1423.0)]
HEXR = 169.0         # red hexagon circumradius
HEX1 = (0.0, 430.0)
HEX2 = (512.0, 993.0)

BROWN  = (52, 26, 35)    # #341A23
ORANGE = (240, 86, 50)   # #F05632
RED    = (180, 30, 55)   # #B41E37

W_PX, H_PX = 800, 710
TEX_PER_PX = 10.24       # 1 tile = 100 px, as the user specified


def _norm(v):
    n = np.hypot(v[0], v[1])
    return (v[0] / n, v[1] / n)


def _segment_planes():
    """Per segment: list of (a, b, c) with keep-side a*x + b*y <= c."""
    segs = []
    n = len(VERTS)
    for i in range(n - 1):
        p0, p1 = VERTS[i], VERTS[i + 1]
        d = _norm((p1[0] - p0[0], p1[1] - p0[1]))
        nx, ny = -d[1], d[0]                      # left normal
        planes = []
        # slab: |n.(p-p0)| <= HS
        c0 = nx * p0[0] + ny * p0[1]
        planes.append((nx, ny, c0 + HS))
        planes.append((-nx, -ny, -(c0 - HS)))
        # miter cut lines at interior joins
        for (j, other, sign) in ((i, i - 1, -1.0), (i + 1, i + 1, 1.0)):
            if other < 0 or other >= n - 1:
                continue                          # fold joins: handled by |x|
            q0, q1 = VERTS[j], VERTS[j + (1 if sign > 0 else 0)]
            # outer normals of the two segments at joint VERTS[j]
            pa, pb = VERTS[j - 1], VERTS[j]
            da = _norm((pb[0] - pa[0], pb[1] - pa[1]))
            pc, pd = VERTS[j], VERTS[j + 1]
            db = _norm((pd[0] - pc[0], pd[1] - pc[1]))
            # turn direction decides outer side
            crossz = da[0] * db[1] - da[1] * db[0]
            s = 1.0 if crossz < 0 else -1.0       # outer = left if right turn
            na = (s * -da[1], s * da[0])
            nb = (s * -db[1], s * db[0])
            m = _norm((na[0] + nb[0], na[1] + nb[1]))   # vertex->miter dir
            # cut line through VERTS[j] with direction m: keep own side
            ax, ay = -m[1], m[0]
            cc = ax * VERTS[j][0] + ay * VERTS[j][1]
            mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
            if ax * mid[0] + ay * mid[1] <= cc:
                planes.append((ax, ay, cc))
            else:
                planes.append((-ax, -ay, -cc))
        segs.append(planes)
    return segs


SEG_PLANES = _segment_planes()


def stroke_mask(xf, y):
    """xf: folded |x| in [0,512]; y: unwrapped within-tile y (any float)."""
    m = np.zeros(xf.shape, dtype=bool)
    for planes in SEG_PLANES:
        sm = np.ones(xf.shape, dtype=bool)
        for (a, b, c) in planes:
            sm &= (a * xf + b * y) <= c + 1e-9
        m |= sm
    return m


def hex_mask(dx, dy):
    ax, ay = np.abs(dx), np.abs(dy)
    return (ax <= HEXR * np.sqrt(3) / 2) & (ax / np.sqrt(3) + ay <= HEXR)


def render(w, h, tex_per_px, u0=0.0, v0=0.0):
    px = np.arange(w) - w / 2
    py = np.arange(h) - h / 2
    u = (u0 + px * tex_per_px) % PERX
    v = (v0 + py * tex_per_px) % PERY
    uu, vv = np.meshgrid(u, v)
    xf = np.abs(((uu + PERX / 2) % PERX) - PERX / 2)   # fold to [0, 512]

    org = stroke_mask(xf, vv)                 # this row
    org |= stroke_mask(xf, vv + PERY)         # row above (its V-tip dips in)
    org |= stroke_mask(xf, vv - PERY)         # row below (its apex pokes up)

    red = hex_mask(xf - HEX1[0], vv - HEX1[1])
    red |= hex_mask(xf - HEX2[0], vv - HEX2[1])

    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = BROWN
    img[org] = ORANGE
    img[red] = RED
    return img


if __name__ == "__main__":
    if "--consts" in sys.argv:
        for i, planes in enumerate(SEG_PLANES):
            print(f"segment {i+1}:")
            for (a, b, c) in planes:
                print(f"  {a:+.6f}*x {b:+.6f}*y <= {c:+.3f}")
        sys.exit(0)
    out = sys.argv[1] if len(sys.argv) > 1 else "proto.png"
    img = render(W_PX, H_PX, TEX_PER_PX)
    Image.fromarray(img).save(out)
    print("wrote", out)
