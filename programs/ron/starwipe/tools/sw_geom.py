#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE frame-side geometry: pattern -> ray table (the spec the microcode
frame program follows).  Float in, integer ray records out (sw_scan.Ray).

A SECTOR SHAPE is described in pattern space (u right, v down, pixels, anchor
at the origin, already rotated) by rays at angles psi_k (radians, clockwise
from +u, sorted) and, for the sector from ray k to ray k+1, a PIECE:
  ('lin', nx, ny, c)        D = nx u + ny v + c
  ('circ', cx, cy, g, c, ex, ey)   D = g (ex (u-cx)^2 + ey (v-cy)^2) + c
Everything is in KEY UNITS: the caller has already multiplied by the edge
gain G (256 / edge width in px) and folded in -T.  Screen pixels may be non
square: a line is ly px tall (NTSC 1.1), field lines step 2 ly interlaced.
"""
import math
from sw_scan import Ray, ray_from_float, Band, pack_range

TWO_PI = 2 * math.pi


def piece_coeffs_line(piece, x_of_y, y):
    """unused helper placeholder"""
    raise NotImplementedError


def shape_to_bands(rays, pieces, ax, ay, W, nlines, ly, pitch, y0):
    """rays: sorted angles; pieces[k] = piece between rays[k] and rays[k+1]
    (pieces[-1] wraps); ax, ay: anchor in frame pixels; ly: line height in
    px; pitch: frame lines per field line; y0: frame line of this field's
    first line.  Returns (Band, q) for sw_scan: every record carries the
    piece's value at x = 0 (polynomial in the line) and its step there."""
    from sw_scan import COT_MAX, X_ACTIVE
    n = len(rays)
    assert n == len(pieces)
    dv = pitch * ly
    ay_line = max(0, math.ceil((ay / ly - y0) / pitch))
    qs = [2 * p[3] * p[5] for p in pieces if p[0] == 'circ']
    q = qs[0] if qs else 0.0

    def mk(piece, cot, v0, start):
        xr = -ax                                   # x = 0 relative to the anchor
        if piece[0] == 'lin':
            _, nx, ny, c = piece
            D0 = nx * xr + ny * v0 + c
            dD0 = ny * dv
            ddD0 = 0.0
            s0 = nx
        else:
            _, cx, cy, g, c, ex, ey = piece
            gx, gy = g * ex, g * ey
            def Dv(v):
                return gx * (xr - cx) ** 2 + gy * (v - cy) ** 2 + c
            D0, D1, D2 = Dv(v0), Dv(v0 + dv), Dv(v0 + 2 * dv)
            dD0, ddD0 = D1 - D0, D2 - 2 * D1 + D0
            s0 = 2 * gx * (xr - cx) + gx
        if start:
            return ray_from_float(0.0, 0.0, D0, dD0, ddD0, s0, pack_range(0, 2047, 1, 1), True)
        # crossing x(v) = ax + v cot ; active while |x| <= X_ACTIVE
        x0 = ax + v0 * cot
        dxl = cot * dv
        if abs(dxl) < 1e-9:
            lo, hi = 0, 2047
        else:
            ta = (-X_ACTIVE - x0) / dxl
            tb = (X_ACTIVE - x0) / dxl
            lo, hi = sorted((ta, tb))
            lo, hi = max(0, math.ceil(lo)), min(2047, math.floor(hi))
        if hi < lo:
            lo, hi = 2047, 2047                    # never active
        left_before = 1 if x0 < 0 else 0
        left_after = 1 if dxl < 0 else 0
        return ray_from_float(x0, dxl, D0, dD0, ddD0, s0, pack_range(lo, hi, left_before, left_after), False)

    def half(sign):
        """sign +1: lines below the anchor (v > 0); -1: above.  Below, walking
        right means DEcreasing angle (crossing ray k enters sector k-1);
        above it means increasing angle (ray k enters sector k)."""
        v0 = (y0 + (ay_line if sign > 0 else 0) * pitch) * ly - ay
        idx = [k for k in range(n) if sign * math.sin(rays[k]) > 1e-9]
        items = []
        for k in idx:
            cot = math.cos(rays[k]) / math.sin(rays[k])
            if abs(cot) > COT_MAX:
                continue                           # treated as horizontal: never crosses
            items.append((k, cot))
        items.sort(key=lambda t: t[1] * sign)          # increasing crossing x
        lst = []
        if items:
            k_first = items[0][0]
            left_piece = pieces[k_first] if sign > 0 else pieces[(k_first - 1) % n]
        else:
            ang = math.pi / 2 if sign > 0 else 3 * math.pi / 2
            below = [k for k in range(n) if rays[k] <= ang + 1e-9]
            left_piece = pieces[max(below)] if below else pieces[n - 1]
        lst.append(mk(left_piece, 0.0, v0, True))
        for k, cot in items:
            piece = pieces[(k - 1) % n] if sign > 0 else pieces[k]
            lst.append(mk(piece, cot, v0, False))
        return lst

    return Band(ay_line, half(-1), half(+1), nlines - 1), q


# ---------------------------------------------------------------- shapes
# Every generator returns (rays, pieces) in pattern space at size T (px) with
# gain G, B inside (negative).  Pieces are sorted by ray angle by sort_rp.
def sort_rp(rays, pieces):
    order = sorted(range(len(rays)), key=lambda i: rays[i] % TWO_PI)
    return [rays[i] % TWO_PI for i in order], [pieces[i] for i in order]


def lin(nx, ny, c):
    return ('lin', nx, ny, c)


def circ(cx, cy, g, c, ex=1.0, ey=1.0):
    return ('circ', cx, cy, g, c, ex, ey)


def star(n, rho, T, G):
    """n-point star, tip straight up, tip radius T"""
    al = math.atan2(1 - rho * math.cos(math.pi / n), rho * math.sin(math.pi / n))
    rays, pieces = [], []
    for k in range(n):
        tip = -math.pi / 2 + k * TWO_PI / n
        for sgn in (+1, -1):
            rays.append(tip if sgn > 0 else tip - math.pi / n)
            nang = tip + sgn * al
            pieces.append(lin(math.cos(nang) * G, math.sin(nang) * G, -T * math.cos(al) * G))
    return sort_rp(rays, pieces)


def polygon(n, T, G, phase=-math.pi / 2):
    """regular n-gon, apothem T (phase = first face normal)"""
    rays, pieces = [], []
    for k in range(n):
        fa = phase + k * TWO_PI / n
        rays.append(fa - math.pi / n)
        pieces.append(lin(math.cos(fa) * G, math.sin(fa) * G, -T * G))
    return sort_rp(rays, pieces)


def box(a, b, G):
    """box half sizes a, b: L-inf with diagonal rays"""
    rays = [math.atan2(b, a), math.atan2(b, -a), math.atan2(-b, -a), math.atan2(-b, a)]
    pieces = [lin(0, G, -b * G), lin(-G, 0, -a * G), lin(0, -G, -b * G), lin(G, 0, -a * G)]
    return sort_rp(rays, pieces)


def diamond(a, b, G):
    """diamond with half axes a (u) and b (v); edges are true distances"""
    rays, pieces = [], []
    for (sx, sy) in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        # edge from (sx a, 0) to (0, sy b): outward normal (sx b, sy a)/L
        L = math.hypot(a, b)
        nx, ny = sx * b / L, sy * a / L
        rays.append(math.atan2(sy * b, 0) if False else None)
        pieces.append(lin(nx * G, ny * G, -(a * b / L) * G))
    rays = [math.atan2(0, 1), math.atan2(1, 0), math.atan2(0, -1), math.atan2(-1, 0)]   # vertex rays: right, down, left, up
    # sector right->down holds edge (+,+) etc.
    pieces = [pieces[0], pieces[1], pieces[2], pieces[3]]
    return sort_rp(rays, pieces)


def halfplane(nx, ny, c, G):
    """one linear field everywhere (bars): D = (nx u + ny v + c) G"""
    return [0.0], [lin(nx * G, ny * G, c * G)]


def absplane(nx, ny, c, G):
    """|nx u + ny v| + c (barn doors): two half-planes split at the line"""
    ang = math.atan2(ny, nx)
    rays = [ang + math.pi / 2, ang - math.pi / 2]
    pieces = [lin(nx * G, ny * G, c * G), lin(-nx * G, -ny * G, c * G)]
    return sort_rp(rays, pieces)


def circle(T, G, w, cx=0.0, cy=0.0, ex=1.0, ey=1.0, extra=0.0):
    """circle radius T in q form, edge gain G at the radius; w = edge width
    (px) keeps the gain finite as T -> 0; extra lifts the field (clean start)"""
    g = G / (2 * T + w)
    return [0.0], [circ(cx, cy, g, -g * T * T + extra, ex, ey)]


def wedge(start, span, G):
    """clock sector from angle start spanning span clockwise; B inside.  The
    field is the signed distance to the nearer edge line: sectors split at
    the bisector and its opposite (2 rays)."""
    span = max(1e-4, min(span, TWO_PI - 1e-4))
    a0, a1 = start, start + span
    bis, opp = start + span / 2, start + span / 2 + math.pi
    return sort_rp([bis, opp], [wedge_edge(a1, +1, G), wedge_edge(a0, -1, G)])


def pinwheel(n, frac, G, phase=0.0):
    """n blades, each a wedge of (2 pi / n) * frac starting at its sector"""
    rays, pieces = [], []
    S = TWO_PI / n
    span = max(1e-4, min(S * frac, S - 1e-4))
    for k in range(n):
        a0 = phase + k * S
        a1 = a0 + span
        mid = (a1 + a0 + S) / 2                       # gap bisector to the next blade
        rays += [a0 + span / 2, mid]
        pieces += [wedge_edge(a1, +1, G), wedge_edge(a0 + S, -1, G)]
    return sort_rp(rays, pieces)


def wedge_edge(ang, sgn, G):
    nx, ny = -math.sin(ang), math.cos(ang)
    return lin(sgn * nx * G, sgn * ny * G, 0.0)


def fan(axis, half_span, G):
    """symmetric wedge about axis with half-span (a fan opening)"""
    return wedge(axis - half_span, 2 * half_span, G)


def union_rays(shapes):
    """several (rays, pieces) around the SAME centre whose B regions are
    disjoint in angle: merge by concatenation (rays sorted)"""
    rays, pieces = [], []
    for r, p in shapes:
        rays += r; pieces += p
    return sort_rp(rays, pieces)


def cross(T, k, G):
    """plus sign: arms half-length T, half-thickness k T (k < 1)"""
    a, b = T, k * T
    rays, pieces = [], []
    # 12 sectors around the plus: for each of 4 arms, the arm end (box face)
    # and the two concave corners
    for arm in range(4):
        ang = arm * math.pi / 2                       # arm direction
        c, s_ = math.cos(ang), math.sin(ang)
        # arm end face: normal = arm direction, at distance a
        rays.append(ang - math.atan2(b, a))
        pieces.append(lin(c * G, s_ * G, -a * G))
        # side faces of this arm (between the end and the concave corner):
        # normal perpendicular to the arm, at distance b
        for sgn in (+1, -1):
            nx, ny = -s_ * sgn, c * sgn
            r0 = ang + sgn * math.atan2(b, a)
            r1 = ang + sgn * math.pi / 4
            lo, hi = (r0, r1) if sgn > 0 else (r1, r0)
            rays.append(lo)
            pieces.append(lin(nx * G, ny * G, -b * G))
    return sort_rp(rays, pieces)


def triangle(T, G, notch=0.0):
    """isoceles triangle pointing up: apex (0,-0.9T), base at v = 0.6T,
    half-width T; notch deepens the base into an arrowhead"""
    apex = (0.0, -0.9 * T); bl = (-T, 0.6 * T); br = (T, 0.6 * T)
    nb = (0.0, 0.6 * T - notch * T)
    pts = [apex, br, nb, bl] if notch > 1e-6 else [apex, br, bl]
    return convex_polygon(pts, G, star_shaped_centre=(0.0, 0.0))


def convex_polygon(pts, G, star_shaped_centre=(0.0, 0.0)):
    """polygon given clockwise on screen (v down) as rays from a centre
    through its vertices; each edge is a linear piece"""
    cx, cy = star_shaped_centre
    n = len(pts)
    rays, pieces = [], []
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        rays.append(math.atan2(y0 - cy, x0 - cx))
        ex, ey = x1 - x0, y1 - y0
        L = math.hypot(ex, ey)
        nx, ny = -ey / L, ex / L               # outward for clockwise-on-screen order
        # orientation: make sure the centre is inside (D < 0 at centre)
        d = nx * (cx - x0) + ny * (cy - y0)
        if d > 0:
            nx, ny = -nx, -ny
        pieces.append(lin(nx * G, ny * G, -(nx * x0 + ny * y0) * G))
    return sort_rp(rays, pieces)


def bowtie(T, half_angle, G):
    """two wedges meeting at the centre (left and right), closed by |u| < T"""
    pts_r = [(0, 0), (T, -T * math.tan(half_angle)), (T, T * math.tan(half_angle))]
    pts_l = [(0, 0), (-T, T * math.tan(half_angle)), (-T, -T * math.tan(half_angle))]
    # as sectors from the centre: right wedge [-ha, +ha] has the face u = T;
    # outside pieces are the wedge edges
    ha = half_angle
    rays = [-ha, ha, math.pi - ha, math.pi + ha]
    pieces = [lin(G, 0, -T * G), wedge_edge(ha, +1, G), lin(-G, 0, -T * G), wedge_edge(math.pi + ha, +1, G)]
    # between ha and pi-ha (below) the nearest edges: right wedge's lower edge
    # then the left wedge's lower edge -- split at pi/2
    rays = [-ha, ha, math.pi / 2, math.pi - ha, math.pi + ha, 3 * math.pi / 2]
    pieces = [lin(G, 0, -T * G), wedge_edge(ha, +1, G), wedge_edge(math.pi - ha, -1, G),
              lin(-G, 0, -T * G), wedge_edge(math.pi + ha, +1, G), wedge_edge(-ha, -1, G)]
    return sort_rp(rays, pieces)


def vee(k, G, c=0.0, sign=1):
    """vee-shaped edge: D = sign (v - k |u|) + c; sectors split at the axis"""
    L = math.hypot(1, k)
    rays = [math.pi / 2, 3 * math.pi / 2]
    pieces = [lin(-k / L * sign * G, 1 / L * sign * G, c / L * G), lin(k / L * sign * G, 1 / L * sign * G, c / L * G)]
    return sort_rp(rays, pieces)


def sunburst(n, T, G, phase=0.0):
    """n rays with half-width T (B), merging at T = sin(pi/n) r"""
    rays, pieces = [], []
    S = TWO_PI / n
    for k in range(n):
        a = phase + k * S
        # inside the ray's band: distance to the band edge = |perp| - T
        rays += [a - S / 2, a, a + S / 2 - 1e-9]
        pieces += [wedge_edge(a, +1, G), wedge_edge(a, -1, G), wedge_edge(a + S, +1, G)]
        pieces[-3] = ('lin', pieces[-3][1], pieces[-3][2], -T * G)
        pieces[-2] = ('lin', pieces[-2][1], pieces[-2][2], -T * G)
        pieces[-1] = ('lin', pieces[-1][1], pieces[-1][2], -T * G)
    # pieces: [a-S/2, a): nearest band edge is ray a's edge on its ccw side: D = (-leftnormal.p) - T ... keep as built, 2 per ray
    rays2, pieces2 = [], []
    for k in range(n):
        a = phase + k * S
        rays2 += [a - S / 2, a]
        pieces2 += [('lin', *[c for c in (math.sin(a) * G, -math.cos(a) * G)], -T * G),
                    ('lin', -math.sin(a) * G, math.cos(a) * G, -T * G)]
    return sort_rp(rays2, pieces2)
