#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE pattern previews: a quick FLOAT model of every catalogue pattern at
~50 % with a gold border over the test image.  A visual menu, not the bit-
accurate model (sw_pixel / sw_frame).

Coordinates: u right, v down, in frame heights, anchor at the frame centre;
theta clockwise from +u.  Each pattern returns a signed distance (B inside,
negative) at progress t, or a finished RGB image for the bespoke ones.

  python3 sw_preview.py OUTDIR          one PNG per pattern + catalogue JSON
"""
import sys, os, json, math
import numpy as np
from PIL import Image, ImageDraw

W, H = 960, 540
ASP = W / H
BORDER = 0.012                      # frame heights
PX = 1 / H

# ------------------------------------------------------------ catalogue
# (name, modifier meaning, how it maps to the hardware design)
#   seg   = per-line segments / arcs (planned architecture)
#   cell  = cell-based, segments per cell (HD comfortably; SD caps counts)
#   mod   = modulator field (sine / hash unit)
#   polar = needs per-pixel angle or radius sweep; not planned
CLASSIC = [
    ('MIX', 'additive <- linear -> non-additive', 'seg'),
    ('BAR >', 'edge tilt +-45', 'seg'),
    ('BAR v', 'edge tilt +-45', 'seg'),
    ('DIAGONAL', 'edge angle', 'seg'),
    ('SLIDE', 'direction (4 steps)', 'seg'),
    ('BARN DOOR <>', 'tilt', 'seg'),
    ('BARN DOOR ^v', 'tilt', 'seg'),
    ('BARN DOOR diag', 'angle', 'seg'),
    ('X DOORS', 'X angle', 'seg'),
    ('BOX', 'aspect', 'seg'),
    ('FOUR BOX', 'aspect', 'seg'),
    ('DIAMOND', 'aspect', 'seg'),
    ('DOUBLE DIAMOND', 'aspect', 'seg'),
    ('CIRCLE', 'ellipse squash', 'seg'),
    ('ROUND RECT', 'corner radius', 'seg'),
    ('EYE', 'orientation / width', 'seg'),
    ('CROSS', 'arm thickness', 'seg'),
    ('BOW TIE', 'orientation / angle', 'seg'),
    ('VEE', 'vee sharpness', 'seg'),
    ('BARN VEE', 'vee sharpness', 'seg'),
    ('ZIGZAG', 'tooth count', 'seg'),
    ('BARN ZIGZAG', 'tooth count', 'seg'),
    ('TRIANGLE', 'notch -> arrowhead', 'seg'),
    ('POLYGON', 'sides 3-12', 'seg'),
    ('CLOCK', 'start angle', 'seg'),
    ('PINWHEEL', 'blades 2-8', 'seg'),
    ('DOUBLE FAN', 'spread / orientation', 'seg'),
    ('DOUBLE SWEEP', 'orientation', 'seg'),
    ('SALOON DOOR', 'hinge orientation', 'seg'),
    ('WINDSHIELD', 'wiper style', 'seg'),
    ('BLINDS =', 'slat count', 'seg'),
    ('BLINDS ||', 'slat count', 'seg'),
    ('CHECKERBOARD', 'cell size', 'cell'),
    ('SNAKE', 'cell size', 'cell'),
    ('SPIRAL BOXES', 'cell size', 'cell'),
    ('WATERFALL', 'column count', 'cell'),
    ('MOSAIC', 'block size', 'cell'),
    ('ROULETTE', '(random classic)', 'seg'),
]
CHEESE = [
    ('STAR WIPE', 'points 3-12', 'seg'),
    ('STARBURST', 'spikes / jaggedness', 'seg'),
    ('HEART', 'plumpness / throb', 'seg'),
    ('KEYHOLE', 'slot length', 'seg'),
    ('FLOWER', 'petals 3-12', 'seg'),
    ('GEAR', 'tooth count', 'seg'),
    ('CLOUD', 'bump count', 'seg'),
    ('SPLAT', 'splatter amount', 'seg'),
    ('SPIRAL', 'arms / twist', 'polar'),
    ('VORTEX', 'twist', 'polar'),
    ('SUNBURST', 'ray count', 'seg'),
    ('SHOCKWAVE', 'ring count', 'polar'),
    ('APERTURE', 'blades 5-9', 'seg'),
    ('RADAR', 'afterglow length', 'seg'),
    ('SPOTLIGHT', 'wander size', 'seg'),
    ('TV OFF', 'glow intensity', 'seg'),
    ('THEATER CURTAIN', 'fold count', 'mod'),
    ('PAGE PEEL', 'curl radius', 'seg'),
    ('WAVY CURTAIN', 'wave frequency', 'mod'),
    ('LIGHTNING', 'jaggedness', 'mod'),
    ('MELT', 'drip count', 'seg'),
    ('PLASMA', 'blob scale', 'mod'),
    ('HALFTONE', 'dot pitch', 'cell'),
    ('SWISS CHEESE', 'hole density', 'cell'),
    ('STARS', 'density', 'cell'),
    ('HONEYCOMB', 'cell size', 'cell'),
    ('TRIANGLES', 'tile size', 'cell'),
    ('HARLEQUIN', 'tile size', 'cell'),
    ('PIXEL DISSOLVE', 'grain', 'mod'),
    ('ROULETTE', '(random cheese)', 'seg'),
]

# ------------------------------------------------------------ helpers
def grid():
    y, x = np.mgrid[0:H, 0:W].astype(np.float64)
    u = (x + 0.5) / H - ASP / 2
    v = (y + 0.5) / H - 0.5
    return u, v


def polar(u, v):
    return np.hypot(u, v), np.mod(np.arctan2(v, u), 2 * np.pi)


def fold(th, n, offset=0.0):
    """angle folded into [-pi/n, pi/n] about the nearest sector centre"""
    s = 2 * np.pi / n
    return np.mod(th - offset + s / 2, s) - s / 2


def rot(u, v, a):
    c, s = math.cos(a), math.sin(a)
    return u * c + v * s, -u * s + v * c


def box(u, v, a, b):
    return np.maximum(np.abs(u) - a, np.abs(v) - b)


def circle(u, v, cx, cy, r):
    return np.hypot(u - cx, v - cy) - r


def seg_dist(u, v, ax, ay, bx, by):
    pu, pv = u - ax, v - ay
    bu, bv = bx - ax, by - ay
    h = np.clip((pu * bu + pv * bv) / (bu * bu + bv * bv), 0, 1)
    return np.hypot(pu - bu * h, pv - bv * h)


def polygon_sdf(u, v, pts):
    """signed distance to a polygon (list of (x, y)), negative inside"""
    d = np.full(u.shape, np.inf)
    inside = np.zeros(u.shape, dtype=bool)
    n = len(pts)
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        d = np.minimum(d, seg_dist(u, v, ax, ay, bx, by))
        cond = ((ay > v) != (by > v))
        xint = ax + (v - ay) * (bx - ax) / (by - ay + 1e-12)
        inside ^= cond & (u < xint)
    return np.where(inside, -d, d)


def sector(th, start, span):
    """signed 'distance' of a wedge from angle start spanning span (clockwise):
    inside negative; uses sin of the angular gap (x r later)"""
    a = np.mod(th - start, 2 * np.pi)
    inside = a < span
    gap = np.where(inside, np.minimum(a, span - a), np.minimum(a - span, 2 * np.pi - a))
    return np.where(inside, -np.sin(np.minimum(gap, np.pi / 2)), np.sin(np.minimum(gap, np.pi / 2)))


def star_pts(n, rho, R, cx=0, cy=0, phase=-math.pi / 2):
    pts = []
    for k in range(2 * n):
        r = R if k % 2 == 0 else rho * R
        a = phase + k * math.pi / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def regular(n, R, cx=0, cy=0, phase=-math.pi / 2):
    return [(cx + R * math.cos(phase + 2 * math.pi * k / n), cy + R * math.sin(phase + 2 * math.pi * k / n))
            for k in range(n)]


def cells(u, v, size, x0=-ASP / 2, y0=-0.5):
    """cell index and local coords (0..1) for a square grid of pitch size"""
    cu, cv = (u - x0) / size, (v - y0) / size
    return np.floor(cu).astype(int), np.floor(cv).astype(int), cu - np.floor(cu), cv - np.floor(cv)


def mini_bar(fu, lt):
    """local bar wipe inside a cell: progress lt in 0..1 (cell units)"""
    return fu - lt


def mini_box(fu, fv, lt):
    return np.maximum(np.abs(fu - 0.5), np.abs(fv - 0.5)) - 0.5 * lt


def cascade(order, t, spread=0.5):
    """local progress of a cell with normalised order (0..1) under a cascade"""
    return np.clip((t - order * spread) / (1 - spread), 0, 1)


RNG = np.random.RandomState(7)

# ------------------------------------------------------------ patterns
# each returns D (frame heights, B negative) or dict(rgb=...) when bespoke
def p_mix(u, v, t):
    return dict(mix=t)


def p_bar_r(u, v, t, tilt=0.15):
    uu, _ = rot(u, v, tilt)
    return uu - (-ASP / 2 - 0.3 + (ASP + 0.6) * t)


def p_bar_d(u, v, t, tilt=0.15):
    _, vv = rot(u, v, tilt)
    return vv - (-0.8 + 1.6 * t)


def p_diag(u, v, t):
    uu, _ = rot(u, v, math.pi / 4)
    return uu - (-1.1 + 2.2 * t)


def p_slide(u, v, t):
    D = p_bar_r(u, v, t, 0)
    return dict(D=D, slide=(-ASP / 2 - 0.3 + (ASP + 0.6) * t) - ASP / 2)


def p_barn_h(u, v, t, tilt=0.12):
    uu, _ = rot(u, v, tilt)
    return np.abs(uu) - 1.05 * t


def p_barn_v(u, v, t, tilt=0.12):
    _, vv = rot(u, v, tilt)
    return np.abs(vv) - 0.75 * t


def p_barn_d(u, v, t):
    uu, _ = rot(u, v, math.pi / 4)
    return np.abs(uu) - 1.1 * t


def p_xdoors(u, v, t):
    a, _ = rot(u, v, math.pi / 4)
    b, _ = rot(u, v, -math.pi / 4)
    return np.minimum(np.abs(a), np.abs(b)) - 0.9 * t


def p_box(u, v, t):
    return box(u, v, ASP / 2 * t, 0.5 * t)


def p_fourbox(u, v, t):
    return np.maximum((ASP / 2 - np.abs(u)) - ASP / 2 * t, (0.5 - np.abs(v)) - 0.5 * t)


def p_diamond(u, v, t):
    return (np.abs(u) / ASP * 1.3 + np.abs(v)) / 1.414 - 0.78 * t


def p_ddiamond(u, v, t):
    d1 = (np.abs(u) + np.abs(v)) / 1.414 - 1.0 * t
    d2 = ((ASP / 2 + 0.5) - (np.abs(u) + np.abs(v))) / 1.414 - 1.0 * t
    return np.minimum(d1, d2)


def p_circle(u, v, t):
    return np.hypot(u, v) - 1.03 * t


def p_roundrect(u, v, t, k=0.5):
    s = t
    hx, hy = ASP / 2 * s * 1.05, 0.5 * s * 1.05
    r = k * min(hx, hy)
    qx, qy = np.abs(u) - (hx - r), np.abs(v) - (hy - r)
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - r


def p_eye(u, v, t, w=0.55):
    R = 1.4 * t
    d = w * R
    return np.maximum(np.hypot(u, v - d), np.hypot(u, v + d)) - R


def p_cross(u, v, t, k=0.35):
    s = 1.1 * t
    return np.minimum(box(u, v, s, k * s), box(u, v, k * s, s))


def p_bowtie(u, v, t):
    a = math.radians(35)
    wedge = np.abs(v) * math.cos(a) - np.abs(u) * math.sin(a)
    return np.maximum(wedge, np.abs(u) - 1.0 * t)


def p_vee(u, v, t, k=0.6):
    return (v - k * np.abs(u)) - (-0.6 + 1.9 * t) + 0.3


def p_barnvee(u, v, t, k=0.6):
    return np.abs(v) - k * np.abs(u) - 1.0 * t + 0.4


def tri(x):
    return 2 * np.abs(np.mod(x, 1) - 0.5)


def p_zigzag(u, v, t, n=7, amp=0.07):
    return (u - amp * tri(v * n)) - (-ASP / 2 - 0.1 + (ASP + 0.2) * t)


def p_barnzigzag(u, v, t, n=7, amp=0.07):
    return np.abs(u - amp * tri(v * n)) - 1.0 * t


def p_triangle(u, v, t, notch=0.3):
    s = 1.3 * t
    pts = [(0, -s * 0.9), (s * 1.0, s * 0.6), (0, s * 0.6 - notch * s), (-s * 1.0, s * 0.6)]
    return polygon_sdf(u, v, pts)


def p_polygon(u, v, t, n=6):
    r, th = polar(u, v)
    return r * np.cos(fold(th, n, -math.pi / 2)) - 1.0 * t


def p_clock(u, v, t, start=-math.pi / 2):
    r, th = polar(u, v)
    return r * sector(th, start, 2 * math.pi * t)


def p_pinwheel(u, v, t, n=4):
    r, th = polar(u, v)
    s = 2 * math.pi / n
    return r * sector(np.mod(th, s), 0, s * t) if False else r * sector(np.mod(th, s) * n, 0, 2 * math.pi * t) / n


def p_dfan(u, v, t):
    r, th = polar(u, v)
    span = math.pi * t
    d1 = sector(th, -math.pi / 2 - span / 2, span)
    d2 = sector(th, math.pi / 2 - span / 2, span)
    return r * np.minimum(d1, d2)


def p_dsweep(u, v, t):
    return p_pinwheel(u, v, t, 2)


def p_saloon(u, v, t):
    a = math.pi * t
    out = np.full(u.shape, 1.0)
    for sgn in (1, -1):
        hx = -sgn * ASP / 2
        du = (u - hx) * sgn
        # original door: 0..ASP/2 from the hinge; rotated door
        orig = box(du - ASP / 4, v, ASP / 4, 0.5)
        ru, rv = du * math.cos(a) + v * math.sin(a), -du * math.sin(a) + v * math.cos(a)
        rotd = box(ru - ASP / 4, rv, ASP / 4, 0.5)
        out = np.minimum(out, np.maximum(orig, -rotd))
    return out


def p_windshield(u, v, t):
    r1, th1 = polar(u + 0.45, v - 0.55)
    r2, th2 = polar(u - 0.45, v - 0.55)
    span = math.pi * t
    d1 = np.maximum(r1 * sector(th1, math.pi, span), r1 - 1.15)
    d2 = np.maximum(r2 * sector(th2, math.pi, span), r2 - 1.15)
    return np.minimum(d1, d2)


def p_blinds_h(u, v, t, n=8):
    f = np.mod((v + 0.5) * n, 1)
    return (f - t) / n


def p_blinds_v(u, v, t, n=12):
    f = np.mod((u + ASP / 2) * n / ASP, 1)
    return (f - t) * ASP / n


def p_checker(u, v, t, size=0.2):
    ci, cj, fu, fv = cells(u, v, size)
    par = (ci + cj) & 1
    lt = np.where(par == 0, np.clip(2 * t, 0, 1), np.clip(2 * t - 1, 0, 1))
    return mini_bar(fu, lt) * size


def order_snake(nc, nr):
    o = np.zeros((nr, nc), int)
    k = 0
    for j in range(nr):
        rng = range(nc) if j % 2 == 0 else range(nc - 1, -1, -1)
        for i in rng:
            o[j, i] = k; k += 1
    return o


def order_spiral(nc, nr):
    o = np.full((nr, nc), -1)
    k = 0
    top, bot, left, right = 0, nr - 1, 0, nc - 1
    while top <= bot and left <= right:
        for i in range(left, right + 1): o[top, i] = k; k += 1
        for j in range(top + 1, bot + 1): o[j, right] = k; k += 1
        if top < bot:
            for i in range(right - 1, left - 1, -1): o[bot, i] = k; k += 1
        if left < right:
            for j in range(bot - 1, top, -1): o[j, left] = k; k += 1
        top += 1; bot -= 1; left += 1; right -= 1
    return o


def p_cellorder(u, v, t, order_fn, size=0.2):
    nc, nr = int(math.ceil(ASP / size)), int(math.ceil(1 / size))
    o = order_fn(nc, nr)
    ci, cj, fu, fv = cells(u, v, size)
    ci = np.clip(ci, 0, nc - 1); cj = np.clip(cj, 0, nr - 1)
    k = o[cj, ci] / (nc * nr)
    lt = np.clip((t - k * 0.9) / 0.1, 0, 1)
    return mini_box(fu, fv, lt) * size


def p_snake(u, v, t):
    return p_cellorder(u, v, t, order_snake)


def p_spiralboxes(u, v, t):
    return p_cellorder(u, v, t, order_spiral)


def p_waterfall(u, v, t, n=12):
    c = np.clip(np.floor((u + ASP / 2) / ASP * n), 0, n - 1)
    lt = cascade(c / n, t, 0.5)
    return (v + 0.5) - lt * 1.0


def p_mosaic(u, v, t, size=0.16):
    nc, nr = int(math.ceil(ASP / size)) + 1, int(math.ceil(1 / size)) + 1
    o = RNG.permutation(nc * nr).reshape(nr, nc) / (nc * nr)
    ci, cj, fu, fv = cells(u, v, size)
    ci = np.clip(ci, 0, nc - 1); cj = np.clip(cj, 0, nr - 1)
    lt = np.clip((t - o[cj, ci] * 0.85) / 0.15, 0, 1)
    return mini_box(fu, fv, lt) * size


def p_star(u, v, t, n=5):
    rho = {3: .36, 4: .4, 5: .382, 6: .5, 7: .47, 8: .5}.get(n, .5)
    return polygon_sdf(u, v, star_pts(n, rho, 1.15 * t))


def p_starburst(u, v, t, n=14):
    rs = RNG.RandomState(3) if hasattr(RNG, 'RandomState') else np.random.RandomState(3)
    R = 1.1 * t
    pts = []
    for k in range(2 * n):
        r = R * (1.0 if k % 2 == 0 else 0.62) * (1 + 0.25 * (rs.rand() - 0.5))
        a = -math.pi / 2 + k * math.pi / n + 0.3 * (rs.rand() - 0.5) * math.pi / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    return polygon_sdf(u, v, pts)


def p_heart(u, v, t, plump=1.0):
    s = 1.25 * t
    c1 = circle(u, v, -0.5 * s, -0.3 * s, 0.52 * s * plump)
    c2 = circle(u, v, 0.5 * s, -0.3 * s, 0.52 * s * plump)
    trip = polygon_sdf(u, v, [(-0.98 * s, -0.22 * s), (0.98 * s, -0.22 * s), (0, 0.95 * s)])
    return np.minimum(np.minimum(c1, c2), trip)


def p_keyhole(u, v, t, slot=1.0):
    s = 1.2 * t
    return np.minimum(circle(u, v, 0, -0.3 * s, 0.5 * s), box(u, v - 0.35 * s, 0.2 * s, 0.6 * s * slot))


def p_flower(u, v, t, n=6):
    s = 1.15 * t
    d = circle(u, v, 0, 0, 0.45 * s)
    for k in range(n):
        a = -math.pi / 2 + 2 * math.pi * k / n
        d = np.minimum(d, circle(u, v, 0.55 * s * math.cos(a), 0.55 * s * math.sin(a), 0.42 * s))
    return d


def p_gear(u, v, t, n=10):
    s = 1.15 * t
    r, th = polar(u, v)
    tl = fold(th, n)
    teeth = np.maximum(r - s, r * np.abs(np.sin(tl)) - 0.28 * s * math.pi / n * 1.0)
    return np.minimum(r - 0.78 * s, np.maximum(teeth, 0.78 * s * 0 - r + 0))


def p_cloud(u, v, t, n=7):
    s = 1.3 * t
    d = np.hypot(u / 1.0, v / 0.55) * 0.55 - 0.55 * s * 0.9
    for k in range(n):
        a = 2 * math.pi * k / n + 0.3
        rr = 0.33 * s * (1 + 0.25 * ((k * 7) % 3 - 1) / 2)
        d = np.minimum(d, circle(u, v, 0.78 * s * math.cos(a), 0.45 * s * math.sin(a), rr))
    return d


def p_splat(u, v, t, amount=1.0):
    rs = np.random.RandomState(11)
    s = 1.2 * t
    d = circle(u, v, 0, 0, 0.5 * s)
    for k in range(6):
        a = rs.rand() * 2 * math.pi
        d = np.minimum(d, circle(u, v, 0.35 * s * math.cos(a), 0.35 * s * math.sin(a), (0.25 + 0.2 * rs.rand()) * s))
    for k in range(10):
        a = rs.rand() * 2 * math.pi
        dist = (0.8 + 0.6 * rs.rand()) * s * amount
        d = np.minimum(d, circle(u, v, dist * math.cos(a), dist * math.sin(a), (0.04 + 0.08 * rs.rand()) * s))
    return d


def p_spiral(u, v, t, arms=3):
    r, th = polar(u, v)
    p = 0.22
    ph = np.mod(r / p - arms * th / (2 * math.pi), 1)
    return (ph - t) * p


def p_vortex(u, v, t, twist=4.0):
    r, th = polar(u, v)
    return r - 1.05 * t * (1 + 0.12 * np.sin(7 * th + twist * r * 6))


def p_sunburst(u, v, t, n=16):
    r, th = polar(u, v)
    tl = fold(th, n)
    return r * np.abs(np.sin(tl)) - r * math.sin(math.pi / n) * t * 1.05 - 0.004


def p_shockwave(u, v, t, n=6):
    r, th = polar(u, v)
    p = 1.1 / n
    return (np.abs(np.mod(r / p, 1) - 0.5) - 0.5 * t * 1.05) * p


def p_aperture(u, v, t, n=7):
    s = 1.15 * t
    a0 = 0.6 * t
    r, th = polar(u, v)
    D = r * np.cos(fold(th, n, a0)) - s
    # seams: from each vertex outward, skewed
    seams = np.full(u.shape, 1.0)
    Rv = s / math.cos(math.pi / n)
    for k in range(n):
        a = a0 + math.pi / n + 2 * math.pi * k / n
        vx, vy = Rv * math.cos(a), Rv * math.sin(a)
        b = a + 0.9
        seams = np.minimum(seams, seg_dist(u, v, vx, vy, vx + 2 * math.cos(b), vy + 2 * math.sin(b)))
    return dict(D=D, lines=seams)


def p_radar(u, v, t, glow=0.9):
    r, th = polar(u, v)
    start = -math.pi / 2
    span = 2 * math.pi * t
    a = np.mod(th - start, 2 * math.pi)
    inside = a < span
    k = np.where(inside, np.clip((span - a) / glow, 0, 1), 0.0)
    hand = r * np.abs(np.sin(np.clip(a - span, -1, 1)))
    return dict(key=k, lines=np.where(np.abs(a - span) < 0.5, hand, 1.0))


def p_spotlight(u, v, t, wander=0.35):
    a = 2 * math.pi * t
    cx, cy = wander * math.sin(a), wander * 0.5 * math.sin(2 * a)
    return dict(D=np.hypot(u - cx, v - cy) - 1.15 * t, soft=0.08)


def p_tvoff(u, v, t, glow=1.0):
    if t < 0.6:
        h = 0.5 * (1 - t / 0.6) + 0.004
        D = h - np.abs(v)
        g = np.clip(1 - (np.abs(np.abs(v) - h)) / (0.04 * glow), 0, 1) ** 2
    else:
        w = ASP / 2 * (1 - (t - 0.6) / 0.3) + 0.004
        D = np.where(np.abs(v) < 0.004, w - np.abs(u), -1.0)
        g = np.clip(1 - np.hypot(np.maximum(np.abs(u) - w, 0), v) / (0.04 * glow), 0, 1) ** 2
    return dict(D=D, glow=g)


def p_curtain(u, v, t, folds=9):
    s = ASP / 2 * t * 1.05
    hem = 0.03 * np.abs(np.sin(v * 24))
    D = (ASP / 2 - np.abs(u) - hem) - s
    shade = 0.72 + 0.28 * np.sin(u * folds * 2 * math.pi / ASP * 1.5)
    return dict(D=D, bmod=shade)


def p_pagepeel(u, v, t, curl=0.12):
    uu, vv = rot(u + ASP / 2, v + 0.5, math.pi / 4)
    D = (2.0 * t) - uu           # sheet from the top-left corner: B where uu < 2t
    D = -D
    flap = np.clip(-D / curl, 0, 1)      # 0..1 across the flap, on the A side? no: ahead of the edge
    return dict(D=D + 0.0, flap=np.where((D > 0) & (D < curl), D / curl, -1.0))


def p_wavy(u, v, t, k=5):
    return (u - 0.09 * np.sin(k * 2 * math.pi * v + 1.0)) - (-ASP / 2 - 0.15 + (ASP + 0.3) * t)


def p_lightning(u, v, t, jag=1.0):
    rs = np.random.RandomState(5)
    knots = np.cumsum(rs.randn(40) * 0.05 * jag)
    knots -= knots.mean()
    j = np.interp((v + 0.5) * 39, np.arange(40), knots)
    return (u - j) - (-ASP / 2 - 0.15 + (ASP + 0.3) * t)


def p_melt(u, v, t, n=9):
    rs = np.random.RandomState(9)
    wid = 0.6 + 0.8 * rs.rand(n)
    edges = np.concatenate([[0], np.cumsum(wid)]) / wid.sum() * (ASP + 0.2) - ASP / 2 - 0.1
    d = np.full(u.shape, 1.0)
    for i in range(n):
        x0, x1 = edges[i], edges[i + 1]
        hw = (x1 - x0) / 2
        cx = (x0 + x1) / 2
        h = -0.5 + (1.2 + 0.6 * rs.rand()) * t * 1.0
        d = np.minimum(d, seg_dist(u, v, cx, -1.0, cx, h - hw) - hw)
    return d


def p_plasma(u, v, t, scale=1.0):
    f = (np.sin(7.1 * u / scale + 0.5) + np.sin(6.3 * v / scale + 1.3) + np.sin(5.2 * (u + v) / scale)
         + np.sin(np.hypot(u - 0.3, v + 0.2) * 9 / scale))
    return (f - (4.2 - 8.4 * t)) * 0.08


def p_halftone(u, v, t, pitch=0.11):
    ci, cj, fu, fv = cells(u, v, pitch)
    cx = (ci + 0.5) * pitch - ASP / 2
    cy = (cj + 0.5) * pitch - 0.5
    dist = np.hypot(cx, cy) / 1.1
    rr = (t * 1.6 - 0.6 * dist) * pitch * 0.75
    return np.hypot(fu - 0.5, fv - 0.5) * pitch - rr


def p_cheese(u, v, t, dens=1.0):
    rs = np.random.RandomState(13)
    d = np.full(u.shape, 1.0)
    for k in range(int(60 * dens)):
        cx, cy = (rs.rand() - 0.5) * (ASP + 0.3), (rs.rand() - 0.5) * 1.3
        t0 = rs.rand() * 0.7
        R = max(0.0, (t - t0) / (1 - t0)) * (0.12 + 0.2 * rs.rand())
        d = np.minimum(d, circle(u, v, cx, cy, R))
    return d


def p_stars(u, v, t, size=0.16):
    rs = np.random.RandomState(17)
    ci, cj, fu, fv = cells(u, v, size)
    nc, nr = int(math.ceil(ASP / size)) + 1, int(math.ceil(1 / size)) + 1
    jx, jy, t0 = rs.rand(nr, nc) * 0.5 - 0.25, rs.rand(nr, nc) * 0.5 - 0.25, rs.rand(nr, nc) * 0.5
    ci = np.clip(ci, 0, nc - 1); cj = np.clip(cj, 0, nr - 1)
    lu, lv = fu - 0.5 - jx[cj, ci], fv - 0.5 - jy[cj, ci]
    R = np.clip((t - t0[cj, ci]) / 0.6, 0, 1) * 1.2
    r, th = polar(lu, lv)
    # 5-point star via sector edge: r cos(fold(th) - alpha) with rho 0.382
    al = math.atan2(1 - 0.382 * math.cos(math.pi / 5), 0.382 * math.sin(math.pi / 5))
    ph = np.abs(fold(th, 5, -math.pi / 2))
    D = r * np.cos(ph - al) / math.cos(al) - R * 0.5
    return D * size


def p_honeycomb(u, v, t, size=0.14):
    # hex lattice (pointy-top): find nearest centre by two offset square grids
    h = size * math.sqrt(3) / 2
    best = np.full(u.shape, 10.0)
    for ox, oy in ((0, 0), (h, 1.5 * size)):        # pointy-top lattice
        gx, gy = 2 * h, 3 * size
        ci = np.floor((u - ox) / gx + 0.5); cj = np.floor((v - oy) / gy + 0.5)
        cx, cy = ci * gx + ox, cj * gy + oy
        lu, lv = u - cx, v - cy
        order = np.hypot(cx, cy) / 1.05
        lt = np.clip((t - 0.6 * order) / 0.4, 0, 1)
        hexd = np.maximum.reduce([np.abs(lu), np.abs(lu * 0.5 + lv * 0.866), np.abs(lu * 0.5 - lv * 0.866)]) - size * 0.86 * lt
        best = np.minimum(best, hexd)
    return best


def p_triangles(u, v, t, size=0.2):
    h = size * math.sqrt(3) / 2
    cj = np.floor((v + 0.5) / h)
    fy = (v + 0.5) / h - cj
    xs = (u + ASP / 2) / size + (cj % 2) * 0.5
    ci = np.floor(xs)
    fx = xs - ci
    up = fy >= np.abs(2 * fx - 1)
    tri_up = [(0, 1), (1, 1), (0.5, 0)]
    tri_l = [(-0.5, 0), (0.5, 0), (0, 1)]
    tri_r = [(0.5, 0), (1.5, 0), (1, 1)]
    # which triangle, its centroid, and the cell order for the cascade
    cxs = np.where(up, 0.5, np.where(fx < 0.5, 0.0, 1.0))
    cys = np.where(up, 2 / 3, 1 / 3)
    gx, gy = (ci + cxs) * size - ASP / 2, (cj + cys) * h - 0.5
    order = np.hypot(gx, gy) / 1.1
    lt = np.clip((t - 0.6 * order) / 0.4, 0, 1)
    f = np.maximum(lt, 1e-3)
    px_, py_ = (fx - cxs) / f + cxs, (fy - cys) / f + cys
    d = np.where(up, polygon_sdf(px_, py_, tri_up),
                 np.where(fx < 0.5, polygon_sdf(px_, py_, tri_l), polygon_sdf(px_, py_, tri_r)))
    return d * f * size


def p_harlequin(u, v, t, size=0.2):
    uu, vv = rot(u, v, math.pi / 4)
    ci, cj, fu, fv = cells(uu, vv, size, -2, -2)
    cx, cy = (ci + 0.5) * size - 2, (cj + 0.5) * size - 2
    order = np.hypot(cx, cy) / 1.3
    lt = np.clip((t - 0.6 * order) / 0.4, 0, 1)
    return mini_box(fu, fv, lt) * size


def p_pixel(u, v, t, grain=6):
    ci, cj, _, _ = cells(u, v, grain / H)
    hsh = np.mod((ci * 374761393 + cj * 668265263) ^ (ci * cj * 2654435761), 2 ** 32) / 2 ** 32
    return dict(key=(hsh < t).astype(float))


PATTERNS = {
    'MIX': (p_mix, 0.5), 'BAR >': (p_bar_r, 0.5), 'BAR v': (p_bar_d, 0.5), 'DIAGONAL': (p_diag, 0.5),
    'SLIDE': (p_slide, 0.5), 'BARN DOOR <>': (p_barn_h, 0.45), 'BARN DOOR ^v': (p_barn_v, 0.45),
    'BARN DOOR diag': (p_barn_d, 0.45), 'X DOORS': (p_xdoors, 0.35), 'BOX': (p_box, 0.55),
    'FOUR BOX': (p_fourbox, 0.45), 'DIAMOND': (p_diamond, 0.6), 'DOUBLE DIAMOND': (p_ddiamond, 0.45),
    'CIRCLE': (p_circle, 0.5), 'ROUND RECT': (p_roundrect, 0.6), 'EYE': (p_eye, 0.45),
    'CROSS': (p_cross, 0.6), 'BOW TIE': (p_bowtie, 0.6), 'VEE': (p_vee, 0.5), 'BARN VEE': (p_barnvee, 0.45),
    'ZIGZAG': (p_zigzag, 0.5), 'BARN ZIGZAG': (p_barnzigzag, 0.45), 'TRIANGLE': (p_triangle, 0.6),
    'POLYGON': (p_polygon, 0.55), 'CLOCK': (p_clock, 0.4), 'PINWHEEL': (p_pinwheel, 0.45),
    'DOUBLE FAN': (p_dfan, 0.45), 'DOUBLE SWEEP': (p_dsweep, 0.45), 'SALOON DOOR': (p_saloon, 0.35),
    'WINDSHIELD': (p_windshield, 0.45), 'BLINDS =': (p_blinds_h, 0.5), 'BLINDS ||': (p_blinds_v, 0.5),
    'CHECKERBOARD': (p_checker, 0.35), 'SNAKE': (p_snake, 0.5), 'SPIRAL BOXES': (p_spiralboxes, 0.5),
    'WATERFALL': (p_waterfall, 0.55), 'MOSAIC': (p_mosaic, 0.5), 'ROULETTE': (p_box, 0.5),
    'STAR WIPE': (p_star, 0.55), 'STARBURST': (p_starburst, 0.6), 'HEART': (p_heart, 0.55),
    'KEYHOLE': (p_keyhole, 0.6), 'FLOWER': (p_flower, 0.55), 'GEAR': (p_gear, 0.55), 'CLOUD': (p_cloud, 0.5),
    'SPLAT': (p_splat, 0.5), 'SPIRAL': (p_spiral, 0.5), 'VORTEX': (p_vortex, 0.5), 'SUNBURST': (p_sunburst, 0.4),
    'SHOCKWAVE': (p_shockwave, 0.5), 'APERTURE': (p_aperture, 0.5), 'RADAR': (p_radar, 0.6),
    'SPOTLIGHT': (p_spotlight, 0.5), 'TV OFF': (p_tvoff, 0.45), 'THEATER CURTAIN': (p_curtain, 0.5),
    'PAGE PEEL': (p_pagepeel, 0.5), 'WAVY CURTAIN': (p_wavy, 0.5), 'LIGHTNING': (p_lightning, 0.5),
    'MELT': (p_melt, 0.5), 'PLASMA': (p_plasma, 0.5), 'HALFTONE': (p_halftone, 0.5),
    'SWISS CHEESE': (p_cheese, 0.6), 'STARS': (p_stars, 0.55), 'HONEYCOMB': (p_honeycomb, 0.55),
    'TRIANGLES': (p_triangles, 0.55), 'HARLEQUIN': (p_harlequin, 0.55), 'PIXEL DISSOLVE': (p_pixel, 0.5),
}

# ------------------------------------------------------------ compose
GOLD = np.array([0.85, 0.66, 0.22])
B_RED = np.array([0.55, 0.05, 0.08])
B_BLACK = np.array([0.0, 0.0, 0.0])


def checker_b(u, v, shift=0.0):
    c = ((np.floor((u - shift) / 0.12) + np.floor(v / 0.12)) % 2)[..., None]
    return np.where(c > 0, 0.9, 0.1) * np.ones(3)


def compose(A, res, soft=0.0, bcol=None):
    """A: HxWx3 float; res: D array or dict -> RGB"""
    u, v = grid()
    B = np.broadcast_to(B_BLACK if bcol is None else bcol, A.shape)
    if isinstance(res, dict) and 'mix' in res:
        return A * (1 - res['mix']) + B * res['mix']
    if isinstance(res, dict) and 'key' in res and 'D' not in res:
        k = res['key'][..., None]
        out = A * (1 - k) + B * k
        if 'lines' in res:
            l = np.clip(1 - (res['lines'] - 0.004) / PX, 0, 1)[..., None]
            out = out * (1 - l) + GOLD * l
        return out
    if isinstance(res, dict):
        D = res['D']
    else:
        D = res
    w = max(PX, (res.get('soft', 0.0) if isinstance(res, dict) else soft))
    ki = np.clip(-D / w + 0.5, 0, 1)
    ko = np.clip(-(D - BORDER) / w + 0.5, 0, 1)
    if isinstance(res, dict) and 'slide' in res:
        B = checker_b(u, v, res['slide'])
    if isinstance(res, dict) and 'bmod' in res:
        B = np.broadcast_to(B_RED, A.shape) * res['bmod'][..., None]
    out = A * (1 - ko[..., None]) + GOLD * ko[..., None]
    out = out * (1 - ki[..., None]) + B * ki[..., None]
    if isinstance(res, dict) and 'lines' in res:
        l = np.clip(1 - (res['lines'] - 0.004) / PX, 0, 1)[..., None] * (ki[..., None] < 0.5)
        out = out * (1 - l) + GOLD * l
    if isinstance(res, dict) and 'glow' in res:
        g = res['glow'][..., None]
        out = out * (1 - g) + np.array([1.0, 0.95, 0.85]) * g
    if isinstance(res, dict) and 'flap' in res:
        f = res['flap']
        m = (f >= 0)[..., None]
        shade = (0.55 + 0.6 * np.sin(np.clip(f, 0, 1) * math.pi)) ** 1.0
        col = np.array([0.75, 0.72, 0.68]) * shade[..., None]
        out = np.where(m, col, out)
    return out


def test_image():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sw_sheet import test_rgb
    im = Image.fromarray((test_rgb() * 255 + 0.5).astype(np.uint8)).resize((W, H), Image.LANCZOS)
    return np.asarray(im) / 255.0


def render_all(outdir):
    os.makedirs(outdir, exist_ok=True)
    A = test_image()
    u, v = grid()
    cat = []
    for bank, lst in (('CLASSIC', CLASSIC), ('CHEESE', CHEESE)):
        for i, (name, modif, tag) in enumerate(lst):
            fn, t = PATTERNS[name]
            res = fn(u, v, t)
            bcol = B_RED if name in ('THEATER CURTAIN',) else None
            rgb = compose(A, res, bcol=bcol)
            im = Image.fromarray((np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)).resize((480, 270), Image.LANCZOS)
            fname = f'{bank.lower()}_{i + 1:02d}.jpg'
            im.save(os.path.join(outdir, fname), quality=82)
            cat.append(dict(bank=bank, slot=i + 1, name=name, modifier=modif, tag=tag, file=fname, t=t,
                            knob=f'{(i + 0.5) / len(lst) * 300 - 150:+.0f} deg'))
            print(bank, i + 1, name)
    json.dump(cat, open(os.path.join(outdir, 'catalogue.json'), 'w'), indent=1)


if __name__ == '__main__':
    render_all(sys.argv[1])
