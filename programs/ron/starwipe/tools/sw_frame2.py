#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE frame driver for the scanline engine (float frame side, integer
pixel side).  Controls -> pattern geometry at this frame's T -> ray bands ->
render.  This is the behavioural spec the microcode frame program follows;
the T-bar bound here is numeric (the microcode's is analytic).
"""
import math
import numpy as np
import sw_scan as sc
import sw_geom as gm

TWO_PI = 2 * math.pi
MODES = {
    '1080p': dict(W=1920, HF=1080, ilace=False, ly=1.0, tid=13),
    '720p':  dict(W=1280, HF=720,  ilace=False, ly=1.0, tid=14),
    '480i':  dict(W=720,  HF=486,  ilace=True,  ly=1.1, tid=0),
    '576i':  dict(W=720,  HF=576,  ilace=True,  ly=11 / 12, tid=8),
}

CLASSIC = ['MIX', 'BAR >', 'BAR v', 'DIAGONAL', 'BARN DOOR <>', 'BARN DOOR ^v', 'BARN DOOR diag', 'X DOORS',
           'BOX', 'DIAMOND', 'CIRCLE', 'ROUND RECT', 'EYE', 'CROSS', 'BOW TIE', 'VEE', 'BARN VEE', 'TRIANGLE',
           'POLYGON', 'CLOCK', 'PINWHEEL', 'DOUBLE FAN', 'DOUBLE SWEEP']
CHEESE = ['STAR WIPE', 'STARBURST', 'HEART', 'KEYHOLE', 'FLOWER', 'SUNBURST', 'APERTURE', 'SPOTLIGHT']
EDGE_LAUNCHED = {'BAR >', 'BAR v', 'DIAGONAL', 'BARN DOOR <>', 'BARN DOOR ^v', 'BARN DOOR diag', 'X DOORS',
                 'VEE', 'BARN VEE'}
STAR_RHO = {3: .36, 4: .4, 5: .382, 6: .5, 7: .47, 8: .5, 9: .5, 10: .52, 11: .53, 12: .55}

# colours (hardware order Y, Cr, Cb)
def ycc(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return (64 + round(876 * y), 512 + round(896 * (r - y) / 1.402), 512 + round(896 * (b - y) / 1.772))
WHITE, BLACK, GOLD = (765, 512, 512), (64, 512, 512), ycc(0.80, 0.62, 0.20)


def hue_yuv(h):
    th = h * TWO_PI + math.radians(108.7)
    y = 393 + round(166 * math.cos(th + math.radians(153.6)))
    return (y, 512 + round(242 * math.sin(th)), 512 + round(242 * math.cos(th)))


def b_source(k6):
    if k6 < 48:
        return BLACK
    if k6 < 304:
        return hue_yuv((k6 - 48) / 256)
    return WHITE


def border_colour(k5):
    if k5 < 64:
        return WHITE
    if k5 < 704:
        return hue_yuv((k5 - 64) / 640)
    if k5 < 768:
        return BLACK
    return GOLD


def origin(k3, W, FH):
    pts = [(0.5, 0.5), (0, 0), (0.5, 0), (1, 0), (1, 0.5), (1, 1), (0.5, 1), (0, 1), (0, 0.5), (0, 0)]
    seg = k3 * 9 / 1024
    i = min(8, int(seg))
    f = min(1.0, max(0.0, (seg - i - 0.2) / 0.6))
    (x0, y0), (x1, y1) = pts[i], pts[i + 1]
    return (x0 + (x1 - x0) * f) * W, (y0 + (y1 - y0) * f) * FH


def count_mod(k2, lo, hi, noon):
    """count-type modifier: noon = default, ends at lo / hi"""
    if k2 < 512:
        return lo + round((noon - lo) * k2 / 512)
    return noon + round((hi - noon) * (k2 - 512) / 511)


class FrameSetup:
    """everything the pixel side needs for one frame"""
    pass


def geometry(name, T, G, w, k2, FH, W, h0=0.0):
    """pattern at size/position T -> dict A=(rays, pieces, (ox, oy)),
    B=... or None, op in A / min / max; pattern space (anchor at the origin,
    unrotated), offsets are each accumulator's own centre.  T is in pixels
    along the pattern's natural measure (radius, position, apothem...)."""
    g1 = geometry1(name, T, G, w, k2, FH, W, h0)
    if isinstance(g1, dict):
        return g1
    return dict(A=(g1[0], g1[1], (0.0, 0.0)), B=None, op='A')


def geometry1(name, T, G, w, k2, FH, W, h0=0.0):
    m = (k2 - 512) / 512.0            # -1..1 continuous modifier
    if name == 'STAR WIPE':
        n = count_mod(k2, 3, 12, 5)
        return gm.star(n, STAR_RHO[n], T, G)
    if name == 'POLYGON':
        return gm.polygon(count_mod(k2, 3, 12, 6), T, G)
    if name == 'APERTURE':
        n = count_mod(k2, 5, 9, 7)
        return gm.polygon(n, T, G, phase=-math.pi / 2)
    if name == 'STARBURST':
        rs = np.random.RandomState(3)
        n = count_mod(k2, 8, 20, 14)
        pts = []
        for k in range(2 * n):
            r = T * (1.0 if k % 2 == 0 else 0.62) * (1 + 0.25 * (rs.rand() - 0.5))
            a = -math.pi / 2 + k * math.pi / n + 0.3 * (rs.rand() - 0.5) * math.pi / n
            pts.append((r * math.cos(a), r * math.sin(a)))
        return gm.convex_polygon(pts, G)
    if name == 'BOX':
        rho = (W / FH) * 2 ** m
        a, b = (1.0, 1.0 / rho) if rho >= 1 else (rho, 1.0)
        return gm.box(a * T, b * T, G)
    if name == 'DIAMOND':
        rho = 1.3 * 2 ** m
        a, b = (1.0, 1.0 / rho) if rho >= 1 else (rho, 1.0)
        return gm.diamond(a * T, b * T, G)
    if name == 'CIRCLE':
        e = 2 ** (m * 0.8)
        return gm.circle(T, G, w, ex=1 / e, ey=e, extra=h0 * max(0.0, 1 - T / (4 * w)))
    if name == 'SPOTLIGHT':
        return gm.circle(T, G, w, extra=h0 * max(0.0, 1 - T / (4 * w)))
    if name == 'EYE':
        wd = 0.55 + 0.35 * m
        d = wd * T
        Rr = T * (1 + wd)
        g = G / (2 * Rr + w)
        rays = [0.0, math.pi]
        pieces = [gm.circ(0.0, -d, g, -g * Rr * Rr), gm.circ(0.0, d, g, -g * Rr * Rr)]
        return gm.sort_rp(rays, pieces)
    if name == 'ROUND RECT':
        k = 0.5 + 0.5 * m
        rho = W / FH
        hx, hy = rho * T, T
        r = max(1.0, k * min(hx, hy))
        g = G / (2 * r + w)
        # A: union of box(hx, hy-r) and box(hx-r, hy) = a 12-vertex cross
        ix, iy = hx - r, hy - r
        pts = [(ix, -hy), (ix, -iy), (hx, -iy), (hx, iy), (ix, iy), (ix, hy),
               (-ix, hy), (-ix, iy), (-hx, iy), (-hx, -iy), (-ix, -iy), (-ix, -hy)]
        A = gm.convex_polygon(pts, G)
        # B: the four corner discs in the diagonal sectors
        rays, pieces = [], []
        for (sx, sy) in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            rays.append(math.atan2(sy * hy, sx * hx) - (sy * sx) * 0 - math.pi / 4 * 0)
            pieces.append(gm.circ(sx * ix, sy * iy, g, -g * r * r))
        rays = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
        pieces = [gm.circ(ix, iy, g, -g * r * r), gm.circ(-ix, iy, g, -g * r * r),
                  gm.circ(-ix, -iy, g, -g * r * r), gm.circ(ix, -iy, g, -g * r * r)]
        return dict(A=(A[0], A[1], (0.0, 0.0)), B=(rays, pieces, (0.0, 0.0)), op='min')
    if name == 'HEART':
        plump = 1.0 + 0.2 * max(0.0, -m)
        s_ = T
        R = 0.52 * s_ * plump
        g = G / (2 * R + w)
        # A: the two lobes split at the vertical; B: a kite from the cleft to the point
        A = gm.sort_rp([math.pi / 2, 3 * math.pi / 2],
                       [gm.circ(-0.5 * s_, -0.3 * s_, g, -g * R * R), gm.circ(0.5 * s_, -0.3 * s_, g, -g * R * R)])
        kite = gm.convex_polygon([(0, -0.44 * s_), (0.98 * s_, -0.22 * s_), (0, 0.95 * s_), (-0.98 * s_, -0.22 * s_)], G,
                                 star_shaped_centre=(0.0, 0.1 * s_))
        return dict(A=(A[0], A[1], (0.0, 0.0)), B=(kite[0], kite[1], (0.0, 0.1 * s_)), op='min')
    if name == 'KEYHOLE':
        s_ = T
        slot = 0.6 * s_ * (1 + 0.5 * m)
        R = 0.5 * s_
        g = G / (2 * R + w)
        cy = 0.35 * s_ + slot / 2                 # slot centre
        box_ = gm.box(0.2 * s_, slot / 2 + 0.35 * s_ - cy + (cy - 0.35 * s_), G)
        box_ = gm.box(0.2 * s_, (0.35 * s_ + slot - (-0.3 * s_)) / 2, G)   # from the circle centre row down to the slot bottom
        cyb = (-0.3 * s_ + 0.35 * s_ + slot) / 2
        box_ = gm.box(0.2 * s_, (0.35 * s_ + slot + 0.3 * s_) / 2, G)
        return dict(A=(box_[0], box_[1], (0.0, cyb)),
                    B=([0.0], [gm.circ(0.0, 0.0, g, -g * R * R)], (0.0, -0.3 * s_)), op='min')
    if name == 'FLOWER':
        n = count_mod(k2, 3, 12, 6)
        s_ = T
        pr = (0.42 if n >= 5 else 0.5) * s_
        g = G / (2 * pr + w)
        rays, pieces = [], []
        for k in range(n):
            a = -math.pi / 2 + TWO_PI * k / n
            rays.append(a - math.pi / n)
            pieces.append(gm.circ(0.55 * s_ * math.cos(a), 0.55 * s_ * math.sin(a), g, -g * pr * pr))
        A = gm.sort_rp(rays, pieces)
        dr = 0.5 * s_
        gd = G / (2 * dr + w)
        return dict(A=(A[0], A[1], (0.0, 0.0)), B=([0.0], [gm.circ(0.0, 0.0, gd, -gd * dr * dr)], (0.0, 0.0)), op='min')
    if name == 'CROSS':
        return gm.cross(T, 0.35 + 0.3 * m, G)
    if name == 'BOW TIE':
        return gm.bowtie(T, math.radians(35 + 20 * m), G)
    if name == 'TRIANGLE':
        return gm.triangle(T, G, notch=max(0.0, m) * 0.8)
    if name == 'CLOCK':
        return gm.wedge(-math.pi / 2 + m * math.pi, T, G)          # T = swept angle (rad)
    if name == 'PINWHEEL':
        return gm.pinwheel(count_mod(k2, 2, 8, 4), T, G)             # T = blade fraction
    if name == 'DOUBLE SWEEP':
        return gm.pinwheel(2, T, G, phase=m * math.pi / 2)
    if name == 'DOUBLE FAN':
        return gm.union_rays([gm.wedge(-math.pi / 2 - T / 2, T, G), gm.wedge(math.pi / 2 - T / 2, T, G)])
    if name == 'SUNBURST':
        return gm.sunburst(count_mod(k2, 6, 24, 16), T, G)
    # edge-launched: T = position along the normal
    if name == 'BAR >':
        a = m * math.pi / 4
        return gm.halfplane(math.cos(a), math.sin(a), -T, G)
    if name == 'BAR v':
        a = m * math.pi / 4
        return gm.halfplane(-math.sin(a), math.cos(a), -T, G)
    if name == 'DIAGONAL':
        a = math.pi / 4 + m * math.pi / 4
        return gm.halfplane(math.cos(a), math.sin(a), -T, G)
    if name == 'BARN DOOR <>':
        a = m * math.pi / 4
        return gm.absplane(math.cos(a), math.sin(a), -T, G)
    if name == 'BARN DOOR ^v':
        a = m * math.pi / 4
        return gm.absplane(-math.sin(a), math.cos(a), -T, G)
    if name == 'BARN DOOR diag':
        a = math.pi / 4 + m * math.pi / 4
        return gm.absplane(math.cos(a), math.sin(a), -T, G)
    if name == 'X DOORS':
        a = math.pi / 4 + m * math.pi / 8
        r1, p1 = gm.absplane(math.cos(a), math.sin(a), -T, G)
        r2, p2 = gm.absplane(math.cos(-a), math.sin(-a), -T, G)
        # min of the two: sectors split at the bisectors (0, 90, 180, 270 deg)
        rays = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
        # nearest line in each quadrant-bisector sector: around angle 0 the lines
        # cross at the centre; the region |angle| < 45 is nearest to ... use the
        # general rule: piece = the line whose left/right normal points toward the sector
        pieces = []
        for ang in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
            best = None
            for (nx, ny) in ((math.cos(a), math.sin(a)), (-math.cos(a), -math.sin(a)),
                             (math.cos(-a), math.sin(-a)), (-math.cos(-a), -math.sin(-a))):
                d = nx * math.cos(ang) + ny * math.sin(ang)
                if best is None or d > best[0]:
                    best = (d, nx, ny)
            pieces.append(gm.lin(best[1] * G, best[2] * G, -T * G))
        return gm.sort_rp(rays, pieces)
    if name == 'VEE':
        return gm.vee(0.3 + 0.9 * (m + 1) / 2, G, c=-T, sign=1)
    if name == 'BARN VEE':
        k = 0.3 + 0.9 * (m + 1) / 2
        r, p = gm.vee(k, G, c=-T, sign=1)
        r2, p2 = gm.vee(k, G, c=-T, sign=-1)
        # |v| - k|u| - T: upper half uses sign -1 (v -> -v)
        rays = [0.0, math.pi]
        return gm.sort_rp(rays, [p[0], p2[0]]) if False else gm.sort_rp([0.0, math.pi / 2, math.pi, 3 * math.pi / 2],
                                                                        [p[0], p[1], p2[1], p2[0]])
    raise KeyError(name)


def rotate_shape(sh, psi):
    """rotate a dict shape: rays, pieces and each accumulator's offset"""
    c, s = math.cos(psi), math.sin(psi)
    out = dict(op=sh['op'])
    for key in ('A', 'B'):
        if sh[key] is None:
            out[key] = None
            continue
        rays, pieces, (ox, oy) = sh[key]
        r2, p2 = rotate_geom(rays, pieces, psi)
        out[key] = (r2, p2, (ox * c - oy * s, ox * s + oy * c))
    return out


def rotate_geom(rays, pieces, psi):
    c, s = math.cos(psi), math.sin(psi)
    rays2 = [r + psi for r in rays]
    out = []
    for p in pieces:
        if p[0] == 'lin':
            _, nx, ny, cc = p
            out.append(gm.lin(nx * c - ny * s, nx * s + ny * c, cc))
        else:
            _, cx, cy, g, cc, ex, ey = p
            out.append(gm.circ(cx * c - cy * s, cx * s + cy * c, g, cc, ex, ey))
    return rays2, out


def fields_of(sh, ax, ay, W, nl, ly, pitch, y0):
    """dict shape -> combined D (nl x W, Q.8) through the table machinery"""
    outs = []
    for key in ('A', 'B'):
        if sh[key] is None:
            continue
        rays, pieces, (ox, oy) = sh[key]
        band, q = gm.shape_to_bands(rays, pieces, ax + ox, ay + oy, W, nl, ly, pitch, y0)
        DA, _ = sc.render_fields(sc.Frame([band], qa=q), W, nl)
        outs.append(DA)
    if len(outs) == 1:
        return outs[0]
    return sc.combine(outs[0], outs[1], sh['op'])


def field_bound(name, G, w, k2, FH, W, ax, ay, psi, T, ly):
    """numeric min / max of the field over the frame at size T (every 8th
    line and pixel, through the exact table machinery)"""
    sh = rotate_shape(geometry(name, T, G, w, k2, FH, W), psi)
    step = 8
    nl = FH // step
    D = fields_of(sh, ax, ay, W, nl, ly * step, 1, 0)[:, ::step] / 256.0
    return D.min(), D.max()


def frame(mode, ctl):
    """controls -> FrameSetup for both fields"""
    M = MODES[mode]
    W, FH, ly = M['W'], M['HF'], M['ly']
    k1, k2, k3, k4, k5, k6 = (ctl[f'k{i}'] for i in range(1, 7))
    bank, rev, spin = ctl['s7'], ctl['s8'], ctl['s9']
    lst = CHEESE if bank else CLASSIC
    name = lst[min(len(lst) - 1, k1 * len(lst) // 1024)]
    t = min(1.0, max(0.0, (ctl['p12'] / 1023 - 0.02) / 0.96))
    # EDGE
    w, b = 1.0, 0.0
    if k4 < 496:
        w = 2 ** (math.log2(0.3 * FH) * (496 - k4) / 496)
    elif k4 > 528:
        b = 2 ** (math.log2(0.08 * FH) * (k4 - 528) / 495)
    G = 256.0 / w
    DK = b * G
    margin = 8.0 * G                          # key units of clearance beyond the ramp
    h0 = 128 + DK + margin
    h1 = 128 + margin
    F = FrameSetup()
    F.name, F.W, F.FH, F.ly, F.ilace = name, W, FH, ly, M['ilace']
    F.BC = b_source(k6)
    F.RC = border_colour(k5) if b > 0 else F.BC
    F.DK = int(round(DK))
    F.mix = name == 'MIX'
    if F.mix:
        F.TK = round(t * 256)
        return F
    if name in EDGE_LAUNCHED:
        ax, ay = W / 2, FH / 2
    else:
        ax, ay = origin(k3, W, FH)
    psi = 0.0
    if spin:
        psi = (1.25 * TWO_PI * t) * (-1 if rev else 1)
    # natural measure range for T
    if name in ('CLOCK',):
        Tn = lambda u: u * TWO_PI
    elif name in ('PINWHEEL', 'DOUBLE SWEEP'):
        Tn = lambda u: u
    elif name == 'DOUBLE FAN':
        Tn = lambda u: u * math.pi
    else:
        Tn = None
    if Tn is not None:
        # angular patterns: the sweep itself is the travel; clean ends come from
        # the edge clearance folded into the angular span
        u = 1 - t if rev else t
        sh = geometry(name, Tn(min(1.0, max(0.0, u))), G, w, k2, FH, W)
        F.ends = (t <= 0.0, t >= 1.0)
    else:
        # size sweep: find T0 (all A) and T1 (all B) from the field's bound at
        # unit size (linear pieces) or by the circle formula
        if name in ('CIRCLE', 'SPOTLIGHT', 'EYE', 'HEART', 'KEYHOLE', 'FLOWER', 'ROUND RECT'):
            # q-form shapes: scan T upward until the max is below -h1 (coarse), start at 0
            T0 = 0.0
            lo, hi = 1.0, 4.0 * max(W, FH)
            for _ in range(24):
                mid = (lo + hi) / 2
                _, dmax = field_bound(name, G, w, k2, FH, W, ax, ay, psi, mid, ly)
                if dmax * 256 <= -h1:
                    hi = mid
                else:
                    lo = mid
            T1 = hi
        else:
            # linear pieces: D(T) is linear in T; sample at T = 8 and 16
            # (T = 0 degenerates polygons) and extrapolate per extreme
            Dmin8, Dmax8 = field_bound(name, G, w, k2, FH, W, ax, ay, psi, 8.0, ly)
            Dmin16, Dmax16 = field_bound(name, G, w, k2, FH, W, ax, ay, psi, 16.0, ly)
            k = (Dmin8 - Dmin16) * 256 / 8
            kx = (Dmax8 - Dmax16) * 256 / 8
            Dmin0, Dmax0 = Dmin8 + 8 * k / 256, Dmax8 + 8 * kx / 256
            T0 = (Dmin0 * 256 - h0) / k if k > 0 else 0.0
            T1 = (Dmax0 * 256 + h1) / kx if kx > 0 else 1.0
            if T0 > 0:
                T0 = 0.0
        if rev:
            T = T1 + (T0 - T1) * t
        else:
            T = T0 + (T1 - T0) * t
        sh = geometry(name, T, G, w, k2, FH, W, h0=h0)
        F.ends = (False, False)
    F.shape = rotate_shape(sh, psi)
    F.ax, F.ay, F.rev = ax, ay, rev
    return F


def render(mode, ctl, A):
    M = MODES[mode]
    F = frame(mode, ctl)
    W, FH, ly = F.W, F.FH, F.ly
    out = np.zeros_like(A)
    if F.mix:
        k = F.TK
        B = np.array(F.BC)
        out[:] = A + (((B - A) * k + 128) >> 8)
        return sc.pack422(out) if hasattr(sc, 'pack422') else out
    fields = (0, 1) if M['ilace'] else (0,)
    pitch = 2 if M['ilace'] else 1
    for f in fields:
        nl = FH // 2 if M['ilace'] else FH
        DA = fields_of(F.shape, F.ax, F.ay, W, nl, ly, pitch, f)
        if F.rev and F.name in ('CLOCK', 'PINWHEEL', 'DOUBLE SWEEP', 'DOUBLE FAN'):
            DA = -DA
        rows = slice(f, None, 2) if M['ilace'] else slice(None)
        out[rows] = sc.compose(A[rows], DA, F.DK, F.BC, F.RC)
    return pack422(out)


def pack422(yuv):
    o = yuv.copy()
    W2 = o.shape[1] // 2 * 2
    o[:, 1:W2:2, 1] = o[:, 0:W2:2, 1]
    o[:, 0:W2:2, 2] = o[:, 1:W2:2, 2]
    return o
