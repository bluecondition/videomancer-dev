#!/usr/bin/env python3
"""CUBIST -- look-development model.

Mirrors the planned hardware algorithm:
  per-frame : chunk geometry (static block + turning slab), rotate, project,
              backface cull, painter order, per-face lighting setup, auto-zoom
  per-line  : quad edge intersections -> span ends with perspective-correct UV,
              linear UV across the span
  per-pixel : (face,u,v) -> cubie / sticker / bevel / gap shading, analytic AA

Float math throughout; the fixed-point HW port is diffed against this later.
"""

import math
import os

import numpy as np
from PIL import Image

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

W, H = 1920, 1080
CAM_DIST = 13.0          # camera distance in cubie units (cube side = 3)
TARGET_FRAC = 0.84       # auto-zoom: projected cube height as fraction of H
CAM = np.array([0.0, 0.0, CAM_DIST])

# ---------------- geometry / look constants (cubie units) ----------------
RB = 0.080       # cube outer edge/corner rounding radius (8-10% of cubie)
BW = RB          # face-boundary bevel zone width
SIL_INSET = 0.024  # silhouette sits slightly inside the sharp box outline
AA_PX = 1.4      # AA feather in pixels
G = 0.026        # gap half-width between cubies
CBW = 0.055      # cubie edge rounding width at interior gaps
HS = 0.428       # sticker half-extent
RS = 0.085       # sticker corner radius
SEW = 0.030      # sticker edge bevel width
PHI_MAX = math.radians(80)
PHI_PROF = 1.7   # bevel tilt profile exponent (flat stays flat, curl at edge)
SEG_PX = 64      # perspective re-correction interval along a span (HW divides)

# ---------------- lighting rig ----------------
def _n(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)

L_KEY = _n([-0.42, 0.60, 0.72])      # soft key: upper-left-front
KEY_POS = L_KEY * 7.5                # finite-distance key -> localized highlight
KEY_COL = np.array([1.00, 0.985, 0.955])
L_FILL = _n([0.55, -0.55, 0.63])     # dim cool fill: lower-right
FILL_COL = np.array([0.80, 0.86, 1.0]) * 0.32
AMB = 0.245
KD = 0.80
# specular (broad sheen / tight hot spot), sticker vs plastic
PB, PH = 12.0, 130.0
KSB_ST, KSH_ST = 0.235, 0.70
KSB_PL, KSH_PL = 0.085, 0.24

# ---------------- palette ----------------
def hex2lin(h):
    def l(c):
        c /= 255.0
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    return np.array([l(int(h[0:2], 16)), l(int(h[2:4], 16)), l(int(h[4:6], 16))])

PALETTE = {
    'white':  hex2lin('F8F8F8'),
    'yellow': hex2lin('FFD500'),
    'red':    hex2lin('C41E3A'),
    'orange': hex2lin('FF5800'),
    'green':  hex2lin('009E60'),
    'blue':   hex2lin('0051BA'),
}
PLASTIC = np.array([0.020, 0.020, 0.023])

# ---------------- cube faces / state ----------------
E = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]

# face key -> (normal, Uvec, Vvec) with Uvec x Vvec = normal
FACES = {
    'U': (E[1], E[2], E[0]),
    'D': (-E[1], E[0], E[2]),
    'R': (E[0], E[1], E[2]),
    'L': (-E[0], E[2], E[1]),
    'F': (E[2], E[0], E[1]),
    'B': (-E[2], E[1], E[0]),
}
COLOR_OF = {'U': 'white', 'D': 'yellow', 'R': 'red',
            'L': 'orange', 'F': 'green', 'B': 'blue'}

def face_of_dir(n):
    for k, (fn, _, _) in FACES.items():
        if np.allclose(fn, n):
            return k
    raise ValueError(n)

def solved_state():
    return {f: np.full((3, 3), COLOR_OF[f], dtype=object) for f in FACES}

def rot_axis_int(axis, k):
    """Integer rotation matrix, k quarter turns about +axis."""
    a = math.pi / 2 * k
    c, s = round(math.cos(a)), round(math.sin(a))
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    R = np.eye(3, dtype=int)
    R[i, i] = c; R[i, j] = -s
    R[j, i] = s; R[j, j] = c
    return R

def apply_move(state, axis, layer, k):
    """Quarter-turn(s) of one layer. axis 0..2, layer -1/0/1, k signed turns."""
    R = rot_axis_int(axis, k)
    new = {f: state[f].copy() for f in state}
    for f, (n, Uv, Vv) in FACES.items():
        for i in range(3):
            for j in range(3):
                p = 1.5 * n + (i - 1) * Uv + (j - 1) * Vv
                cubie = p - 0.5 * n
                if round(cubie[axis]) != layer:
                    continue
                p2 = R @ p
                n2 = R @ n
                f2 = face_of_dir(n2)
                _, U2, V2 = FACES[f2]
                i2 = int(round(p2 @ U2)) + 1
                j2 = int(round(p2 @ V2)) + 1
                new[f2][i2, j2] = state[f][i, j]
    return new

# ---------------- scene geometry ----------------
def euler(yaw, pitch, roll):
    """R = Ry(yaw) @ Rx(pitch) @ Rz(roll), degrees."""
    y, p, r = map(math.radians, (yaw, pitch, roll))
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Ry @ Rx @ Rz

def rot_axis_f(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    R = np.eye(3)
    R[i, i] = c; R[i, j] = -s
    R[j, i] = s; R[j, j] = c
    return R

class Face:
    pass

def make_chunks(turn):
    """-> list of (bounds[(lo,hi)x3], turn_rot_or_None)."""
    full = [(-1.5, 1.5)] * 3
    if turn is None:
        return [(full, None)]
    axis, layer, theta = turn
    slab = list(full)
    slab[axis] = (layer - 0.5, layer + 0.5)
    Q = rot_axis_f(axis, theta)
    chunks = [(slab, Q)]
    if layer == 1:
        rest = list(full); rest[axis] = (-1.5, 0.5)
        chunks.append((rest, None))
    elif layer == -1:
        rest = list(full); rest[axis] = (-0.5, 1.5)
        chunks.append((rest, None))
    else:
        lo = list(full); lo[axis] = (-1.5, -0.5)
        hi = list(full); hi[axis] = (0.5, 1.5)
        chunks += [(lo, None), (hi, None)]
    return chunks

def scene_quads(Rmat, turn, state):
    """Build all visible quads with projection + adjacency, painter-ordered."""
    chunk_faces = []
    for bounds, Q in make_chunks(turn):
        faces = []
        M = Rmat @ (Q if Q is not None else np.eye(3))
        for axis in range(3):
            for sgn in (1, -1):
                fc = Face()
                n_l = E[axis] * sgn
                lo, hi = bounds[axis]
                offset = hi if sgn > 0 else lo
                outer = abs(abs(offset) - 1.5) < 1e-9
                if outer:
                    fkey = face_of_dir(n_l)
                    _, Uv, Vv = FACES[fkey]
                else:
                    others = [a for a in range(3) if a != axis]
                    Uv, Vv = E[others[0]], E[others[1]]
                    if not np.allclose(np.cross(Uv, Vv), n_l):
                        Uv, Vv = Vv, Uv
                axU = int(np.argmax(np.abs(Uv)))
                axV = int(np.argmax(np.abs(Vv)))
                nu = int(round(bounds[axU][1] - bounds[axU][0]))
                nv = int(round(bounds[axV][1] - bounds[axV][0]))
                if outer:
                    u0 = bounds[axU][0] + 1.5
                    v0 = bounds[axV][0] + 1.5
                    st = np.empty((nu, nv, 3))
                    for ci in range(nu):
                        for cj in range(nv):
                            st[ci, cj] = PALETTE[state[fkey][int(u0) + ci,
                                                            int(v0) + cj]]
                    origin_l = (1.5 * n_l - 1.5 * Uv - 1.5 * Vv) \
                        + u0 * Uv + v0 * Vv
                    fc.stickers = st
                else:
                    origin_l = offset * n_l \
                        + bounds[axU][0] * Uv + bounds[axV][0] * Vv
                    fc.stickers = None
                fc.n_local = n_l
                fc.Uv_l, fc.Vv_l = Uv, Vv
                fc.nu, fc.nv = nu, nv
                fc.Ow = M @ origin_l
                fc.Uw = M @ Uv
                fc.Vw = M @ Vv
                fc.nw = M @ n_l
                fc.corners_w = np.array([
                    fc.Ow,
                    fc.Ow + nu * fc.Uw,
                    fc.Ow + nu * fc.Uw + nv * fc.Vw,
                    fc.Ow + nv * fc.Vw,
                ])
                center = fc.Ow + (nu / 2) * fc.Uw + (nv / 2) * fc.Vw
                fc.visible = float(np.dot(fc.nw, CAM - center)) > 1e-9
                faces.append(fc)
        by_dir = {tuple(np.round(f.n_local).astype(int)): f for f in faces}
        for fc in faces:
            adj = [
                by_dir[tuple(np.round(-fc.Uv_l).astype(int))],
                by_dir[tuple(np.round(fc.Uv_l).astype(int))],
                by_dir[tuple(np.round(-fc.Vv_l).astype(int))],
                by_dir[tuple(np.round(fc.Vv_l).astype(int))],
            ]
            fc.sil = [not a.visible for a in adj]
        chunk_faces.append(faces)

    # auto-zoom from every chunk corner
    ratios_y, ratios_x = [], []
    for faces in chunk_faces:
        for fc in faces:
            for c in fc.corners_w:
                zv = CAM_DIST - c[2]
                ratios_y.append(abs(c[1] / zv))
                ratios_x.append(abs(c[0] / zv))
    f_y = (TARGET_FRAC * H / 2) / max(ratios_y)
    f_x = (0.94 * W / 2) / max(ratios_x)
    foc = min(f_y, f_x)

    def project(p):
        zv = CAM_DIST - p[2]
        return (W / 2 + foc * p[0] / zv,
                H / 2 - foc * p[1] / zv,
                zv)

    all_faces = []
    for faces in chunk_faces:
        depth = np.mean([project(c)[2] for fc in faces for c in fc.corners_w])
        vis = [fc for fc in faces if fc.visible]
        for fc in vis:
            pr = np.array([project(c) for c in fc.corners_w])
            fc.pxy = pr[:, :2]
            fc.pz = pr[:, 2]
            area = 0.5 * abs(np.cross(fc.pxy[2] - fc.pxy[0],
                                      fc.pxy[3] - fc.pxy[1]))
            fc.px_scale = math.sqrt(max(area, 1e-6) / (fc.nu * fc.nv))
        all_faces.append((depth, vis))
    all_faces.sort(key=lambda t: -t[0])   # farthest chunk first
    quads = [fc for _, vis in all_faces for fc in vis]
    return quads, foc

# ---------------- shading ----------------
def shade_span(fc, u, v):
    """u, v: arrays of face-local UV (cubie units). -> (rgb (N,3), alpha (N,))"""
    N = len(u)
    nu, nv = fc.nu, fc.nv
    Pw = fc.Ow[None, :] + u[:, None] * fc.Uw[None, :] + v[:, None] * fc.Vw[None, :]
    Vv = CAM[None, :] - Pw
    Vv /= np.linalg.norm(Vv, axis=1, keepdims=True)

    # ---- face boundary bevel (rounded box edges/corners) ----
    pu = u - nu / 2.0
    pv = v - nv / 2.0
    qu = np.abs(pu) - (nu / 2.0 - RB)
    qv = np.abs(pv) - (nv / 2.0 - RB)
    qup = np.maximum(qu, 0.0)
    qvp = np.maximum(qv, 0.0)
    outn = np.sqrt(qup ** 2 + qvp ** 2)
    d_bnd = RB - np.where((qu > 0) & (qv > 0), outn, np.maximum(qu, qv))
    t_f = np.clip(1.0 - d_bnd / BW, 0.0, 1.0)
    fdir_u = np.sign(pu) * qup
    fdir_v = np.sign(pv) * qvp

    # ---- silhouette alpha ----
    # Straight sections: distance to each silhouette-flagged edge.
    # Corners: where a silhouette edge meets ANY other boundary edge, cut a
    # rounded arc using both edge distances -- at a two-face outline vertex the
    # two faces' arcs meet at the shared seam and join into one continuous
    # rounded corner (each face cuts its half of the corner patch).
    d_edges = [u, nu - u, v, nv - v]
    m = [np.maximum(RB - de, 0.0) for de in d_edges]
    d_sil = np.full(N, RB)
    for de, flag in zip(d_edges, fc.sil):
        if flag:
            d_sil = np.minimum(d_sil, de)
    for i, j in ((0, 2), (0, 3), (1, 2), (1, 3)):
        if fc.sil[i] or fc.sil[j]:
            active = (m[i] > 0.0) & (m[j] > 0.0)
            c = RB - np.sqrt(m[i] ** 2 + m[j] ** 2)
            d_sil = np.where(active, np.minimum(d_sil, c), d_sil)
    alpha = np.clip(((d_sil - SIL_INSET) * fc.px_scale) / AA_PX + 0.5, 0.0, 1.0)

    # ---- interior gap lines + cubie edge rounding ----
    if nu >= 2:
        ku = np.clip(np.round(u), 1, nu - 1)
        d_iu = np.abs(u - ku)
        sgn_u = np.sign(ku - u)
    else:
        d_iu = np.full(N, 9.9); sgn_u = np.zeros(N)
    if nv >= 2:
        kv = np.clip(np.round(v), 1, nv - 1)
        d_iv = np.abs(v - kv)
        sgn_v = np.sign(kv - v)
    else:
        d_iv = np.full(N, 9.9); sgn_v = np.zeros(N)
    mgu = np.maximum(G + CBW - d_iu, 0.0)
    mgv = np.maximum(G + CBW - d_iv, 0.0)
    mg = np.sqrt(mgu ** 2 + mgv ** 2)
    t_c = np.clip(mg / CBW, 0.0, 1.0)
    gap_depth = np.clip((mg - CBW) / G, 0.0, 1.0)
    cdir_u = sgn_u * mgu
    cdir_v = sgn_v * mgv

    # ---- effective normal (bevel tilt) ----
    gx = fdir_u + cdir_u
    gy = fdir_v + cdir_v
    gn = np.sqrt(gx ** 2 + gy ** 2)
    gn = np.where(gn < 1e-9, 1.0, gn)
    gx /= gn; gy /= gn
    t = np.clip(t_f + t_c, 0.0, 1.0)
    phi = PHI_MAX * t ** PHI_PROF
    dirw = gx[:, None] * fc.Uw[None, :] + gy[:, None] * fc.Vw[None, :]
    n_eff = np.cos(phi)[:, None] * fc.nw[None, :] + np.sin(phi)[:, None] * dirw

    # ---- sticker ----
    ci = np.clip(np.floor(u).astype(int), 0, nu - 1)
    cj = np.clip(np.floor(v).astype(int), 0, nv - 1)
    fu = u - ci - 0.5
    fv = v - cj - 0.5
    if fc.stickers is not None:
        qsu = np.abs(fu) - (HS - RS)
        qsv = np.abs(fv) - (HS - RS)
        qsup = np.maximum(qsu, 0.0)
        qsvp = np.maximum(qsv, 0.0)
        souter = np.sqrt(qsup ** 2 + qsvp ** 2)
        ds = RS - np.where((qsu > 0) & (qsv > 0), souter, np.maximum(qsu, qsv))
        st_alpha = np.clip(ds * fc.px_scale / 0.9 + 0.5, 0.0, 1.0)
        edge_f = 0.68 + 0.32 * np.clip(ds / SEW, 0.0, 1.0)
        st_rgb = fc.stickers[ci, cj] * edge_f[:, None]
    else:
        st_alpha = np.zeros(N)
        st_rgb = np.zeros((N, 3))

    albedo = PLASTIC[None, :] * (1 - st_alpha[:, None]) + st_rgb * st_alpha[:, None]

    # ---- lighting ----
    Lp = KEY_POS[None, :] - Pw
    Lp /= np.linalg.norm(Lp, axis=1, keepdims=True)
    ndl = np.maximum(np.sum(n_eff * Lp, axis=1), 0.0)
    ndf = np.maximum(n_eff @ L_FILL, 0.0)
    Hh = Lp + Vv
    Hh /= np.linalg.norm(Hh, axis=1, keepdims=True)
    ndh = np.maximum(np.sum(n_eff * Hh, axis=1), 0.0)
    spec_b = ndh ** PB
    spec_h = ndh ** PH
    ksb = KSB_PL + (KSB_ST - KSB_PL) * st_alpha
    ksh = KSH_PL + (KSH_ST - KSH_PL) * st_alpha

    color = albedo * (AMB + KD * ndl[:, None] * KEY_COL[None, :]
                      + ndf[:, None] * FILL_COL[None, :]) \
        + (ksb * spec_b + ksh * spec_h)[:, None] * KEY_COL[None, :]

    # crevice AO
    color *= (1.0 - 0.75 * gap_depth[:, None])
    color *= (1.0 - 0.18 * t_c[:, None] * (1 - gap_depth[:, None]))
    return color, alpha

# ---------------- rasterizer ----------------
def rasterize(fc, img):
    P = fc.pxy
    Z = fc.pz
    UVc = np.array([[0, 0], [fc.nu, 0], [fc.nu, fc.nv], [0, fc.nv]], dtype=float)
    ymin = max(int(math.floor(P[:, 1].min())), 0)
    ymax = min(int(math.ceil(P[:, 1].max())), H - 1)
    for y in range(ymin, ymax + 1):
        yc = y + 0.5
        hits = []
        for i in range(4):
            j = (i + 1) % 4
            y0, y1 = P[i, 1], P[j, 1]
            if y0 == y1:
                continue
            if (y0 <= yc < y1) or (y1 <= yc < y0):
                t = (yc - y0) / (y1 - y0)
                x = P[i, 0] + t * (P[j, 0] - P[i, 0])
                w0, w1 = 1.0 / Z[i], 1.0 / Z[j]
                w = w0 + t * (w1 - w0)
                uw = UVc[i, 0] * w0 + t * (UVc[j, 0] * w1 - UVc[i, 0] * w0)
                vw = UVc[i, 1] * w0 + t * (UVc[j, 1] * w1 - UVc[i, 1] * w0)
                hits.append((x, uw / w, vw / w, w))
        if len(hits) < 2:
            continue
        hits.sort(key=lambda h: h[0])
        xl, ul, vl, wl = hits[0]
        xr, ur, vr, wr = hits[-1]
        if xr - xl < 1e-6:
            continue
        x0 = max(int(math.ceil(xl - 0.5)), 0)
        x1 = min(int(math.floor(xr - 0.5)), W - 1)
        if x1 < x0:
            continue
        xs = np.arange(x0, x1 + 1) + 0.5
        # perspective-exact UV at segment nodes (HW: shared divider),
        # linear in between.  uw, vw, w are all linear in screen x.
        nseg = max(1, int(math.ceil((xr - xl) / SEG_PX)))
        tn = np.linspace(0.0, 1.0, nseg + 1)
        wn = wl + tn * (wr - wl)
        un = (ul * wl + tn * (ur * wr - ul * wl)) / wn
        vn = (vl * wl + tn * (vr * wr - vl * wl)) / wn
        xn = xl + tn * (xr - xl)
        u = np.interp(xs, xn, un)
        v = np.interp(xs, xn, vn)
        rgb, a = shade_span(fc, u, v)
        row = img[y, x0:x1 + 1]
        img[y, x0:x1 + 1] = row * (1 - a[:, None]) + rgb * a[:, None]

# ---------------- background ----------------
def background(shadow_cx=None, shadow_cy=None, shadow_w=None):
    ys = np.linspace(0.0, 1.0, H)[:, None]
    top = np.array([0.360, 0.345, 0.325])    # light warm gray
    bot = np.array([0.130, 0.138, 0.155])    # deeper cool gray
    g = top[None, None, :] + (bot - top)[None, None, :] * (ys ** 1.15)[:, :, None]
    img = np.repeat(g, W, axis=1)
    # vignette pool of light behind the cube
    xx, yy = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    r2 = (((xx - W * 0.5) / (0.72 * W)) ** 2
          + ((yy - H * 0.44) / (0.80 * H)) ** 2)
    img *= (1.0 - 0.26 * np.clip(r2, 0, 1))[:, :, None]
    if shadow_cx is not None:
        sx = (xx - shadow_cx) / (shadow_w * 0.72)
        sy = (yy - shadow_cy) / (shadow_w * 0.11)
        img *= (1.0 - 0.26 * np.exp(-(sx ** 2 + sy ** 2)))[:, :, None]
    return img

# ---------------- top level ----------------
def render(yaw, pitch, roll, turn=None, state=None, shadow=True):
    state = state if state is not None else solved_state()
    Rmat = euler(yaw, pitch, roll)
    quads, foc = scene_quads(Rmat, turn, state)
    if shadow:
        ally = [p[1] for fc in quads for p in fc.pxy]
        allx = [p[0] for fc in quads for p in fc.pxy]
        cx = 0.5 * (min(allx) + max(allx))
        wpx = max(allx) - min(allx)
        img = background(cx, max(ally) + 0.045 * wpx, wpx)
    else:
        img = background()
    for fc in quads:
        rasterize(fc, img)
    return img

def save(img, name):
    out = np.clip(img, 0.0, 1.0) ** (1 / 2.2)
    Image.fromarray((out * 255 + 0.5).astype(np.uint8)).save(
        os.path.join(OUT_DIR, name))
    print('wrote', name)

if __name__ == '__main__':
    st = solved_state()
    save(render(-32, 24, 0), 'prev_a_hero.png')

    scr = st
    for mv in [(1, 1, 1), (0, 1, -1), (2, -1, 1), (0, -1, 2), (1, -1, -1)]:
        scr = apply_move(scr, *mv)
    save(render(-32, 24, 0, turn=(1, 1, 40.0), state=scr), 'prev_b_turn.png')

    save(render(-45, 33, 0), 'prev_c_corner.png')
    save(render(-9, 6, 0), 'prev_d_facish.png')
