#!/usr/bin/env python3
"""model.py -- integer model of the APPLIQUE fill (pattern generator, relief, colour), mirroring
applique.vhd operation for operation, plus an approximate model of the matte pipeline (shared
with VENEER) so the fills can be previewed on any image at true 1080p scale without a simulation.

  python3 model.py IMAGE OUT.png [--set N] [--scale 0..1023] [--depth 0..15] [--colour 0..3]
                   [--hue 0..15] [--levels 2..8] [--flat] [--bias] [--invert] [--noseams] [--fixed]
  python3 model.py --sheet OUT.png [--scale K]      all 16 patterns x 4 variants
  python3 model.py --sets OUT.png [--scale K]       the 8 sets, slot by slot

Needs numpy + Pillow (the SDK image tester's venv has both)."""
import sys, pathlib
import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tables

# ----------------------------------------------------------------------------------------
# coordinates: virtual 1080-line units, Scale (K5) folded into the accumulator steps
# ----------------------------------------------------------------------------------------
M21 = (1 << 21) - 1

def vstep(scale_k):
    """pattern positions (1/64 cell, Q.10) per virtual pixel; scale_k = settled K5 (0..1023)"""
    kf = scale_k << 4
    o = kf >> 12; f = kf & 4095
    return (8192 - f) >> (1 + o)

def steps(scale_k, timing=13):
    """(x step per pixel, y step per field line) for a timing ID"""
    v = vstep(scale_k)
    if timing in (14, 6, 5):   xs = v + (v >> 1)                 # 720p
    elif timing in (0, 8, 4, 12): xs = v << 1                    # SD
    else: xs = v
    ys = {0: (v << 2) + (v >> 1), 8: (v << 2) - (v >> 2), 4: (v << 1) + (v >> 2), 12: (v << 1) - (v >> 3),
          14: v + (v >> 1), 6: v + (v >> 1), 5: v + (v >> 1), 10: v << 1, 2: v << 1, 1: v << 1}.get(timing, v)
    return xs, ys

def coords(W, H, scale_k=384, timing=13, xoff=0, yoff=0):
    """the four 11-bit coordinates x, y, x+y, x-y of every pixel (T0)"""
    xs, ys = steps(scale_k, timing)
    xa = (xoff + np.arange(W, dtype=np.int64) * xs) & M21
    ya = (yoff + np.arange(H, dtype=np.int64) * ys) & M21
    gx, gy = np.meshgrid(xa >> 8, ya >> 8)
    gu = (gx + gy) & 8191; gv = (gx - gy) & 8191
    return gx >> 2, gy >> 2, gu >> 2, gv >> 2

# ----------------------------------------------------------------------------------------
# pattern generator (T1..T4): recipe word -> t4 (0..15, 0 = tone A, 15 = tone B)
# ----------------------------------------------------------------------------------------
BAYER = np.array([0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5])

def pattern(word, cx, cy, cu, cv, bias_sw=False, invert_sw=False):
    typ = word & 15; v = (word >> 4) & 3; size = (word >> 6) & 3
    bias = ((word >> 8) & 1) ^ (1 if bias_sw else 0); turn = (word >> 9) & 1
    stg = (word >> 10) & 1; inv = ((word >> 11) & 1) ^ (1 if invert_sw else 0); hst = (word >> 12) & 1
    # T1: axes
    if bias: a, b = (cv, cu) if turn else (cu, cv)
    else:    a, b = (cy, cx) if turn else (cx, cy)
    # T2: size, stagger, folds
    fa = (a >> size) & 255; fb = (b >> size) & 255
    if stg: fa = fa ^ (((fb >> 6) & 1) << 5)
    if hst: fa = fa ^ (((fb >> 5) & 1) << 5)
    pa = fa & 63; ia = fa >> 6; pb = fb & 63; ib = fb >> 6
    da = np.where(pa & 32, pa & 31, ~pa & 31); db = np.where(pb & 32, pb & 31, ~pb & 31)
    # T3: features
    gt = pa > pb; sm = pa + pb; df = (pa - pb) & 63
    hg = da > db; li = np.maximum(da, db); l1 = da + db
    sqa = da * da; r2 = sqa + db * db
    ch = (pb + da) & 63; sc = ((~pb & 31) - (sqa >> 5)) & 31
    gs = (pb >> 1) > da
    ia0 = ia & 1; ib0 = ib & 1
    b5 = lambda q: (q >> 5) & 1
    b4 = lambda q: (q >> 4) & 1
    b3 = lambda q: (q >> 3) & 1
    W = np.where
    # T4: pattern
    if typ == 0:      # plain
        t = [8 + 0 * pa, W(((pa ^ pb) >> 2) & 1, 9, 7), W(b3(pb), 9, 7), W(b3(sm), 9, 7)][v]
    elif typ == 1:    # stripe
        t = [W(b5(pa), 15, 0), W(b4(pa), 15, 0), W((pa >> 3) == 0, 2, 12),
             W(b5(pa) == 0, W(b4(pa), 15, 0), W(((pa >> 2) & 7) == 4, 15, 6))][v]
    elif typ == 2:    # ridge
        t = [(pa >> 1) & 15, da >> 1, pa >> 2, W(b4(pa), pa & 15, ~pa & 15)][v]
    elif typ == 3:    # check
        t = [W(b5(pa ^ pb), 15, 0), np.array([1, 8, 15])[b5(pa) + b5(pb)],
             W(((pa >> 2) == 0) | ((pb >> 2) == 0), 2, 12), W(b4(pa ^ pb), 15, 0)][v]
    elif typ == 4:    # plaid
        la = (pa >> 1) & 15; lb = (pb >> 1) & 15
        if v == 0:
            t = np.array([3, 8, 13])[b5(pa) + b5(pb)]
            t = W((la == 4) | (lb == 4), 15, W((la == 12) | (lb == 12), 0, t))
        elif v == 1:
            t = np.array([3, 8, 13])[b5(pa) + b5(pb)]
            t = W((la == 4) | (la == 6) | (lb == 4) | (lb == 6), 15, t)
        elif v == 2:
            A = W(ia0 == 1, b5(pa), b5(pa) & b4(pa)); B = W(ib0 == 1, b5(pb), b5(pb) & b4(pb))
            t = np.array([3, 8, 13])[A + B]
            t = W((la == 2) | (lb == 2), 0, t)
        else:
            t = 1 + 2 * ((pa >> 4) + (pb >> 4))
    elif typ == 5:    # houndstooth
        if v == 0:   t = W(W(b4(sm) == 0, b5(pa), b5(pb)) == 1, 15, 0)
        elif v == 1: t = W(W(b3(sm) == 0, b5(pa), b5(pb)) == 1, 15, 0)
        elif v == 2: t = W((ia0 ^ ib0) == 1, W(W(b4(sm) == 0, b5(pa), b5(pb)) == 1, 15, 0), W(b3(pa ^ pb), 11, 4))
        else:        t = W(b5(pa) == b5(pb), W(b5(pa), 15, 0), W(b4(sm), 11, 4))
    elif typ == 6:    # dot
        t = [W(r2 < 256, 15, 1), W(r2 < 128, 15, 1), W(r2 < 512, 15, 1), W((r2 < 512) & (r2 >= 128), 15, 1)][v]
    elif typ == 7:    # target
        t = [W(b3(li), 15, 0), W((li >> 2) & 1, 15, 0), W((r2 >> 8) & 1, 15, 0), W(b3(l1), 15, 0)][v]
    elif typ == 8:    # diamond
        if v == 0:   t = W(l1 < 32, 15, 0)
        elif v == 1:
            line = ((df >> 2) == 0) | (((sm >> 2) & 15) == 15)
            t = W(l1 < 32, W(line, 0, 12), W(line, 15, 3))
        elif v == 2: t = W(l1 < 16, 15, 1)
        else:        t = W((l1 >> 3) <= 4, 15, 0)
    elif typ == 9:    # stud
        t = [15 - (li >> 1), li >> 1, W(r2 >= 1024, 0, 15 - ((r2 >> 6) & 15)), 15 - 2 * (l1 >> 3)][v]
    elif typ == 10:   # triangles
        if v == 0:   t = W(gt, 15, 0)
        elif v == 1:
            hi = (sm >> 6) & 1
            q = ia0 * 2 + ib0
            t = W(W(q == 0, gt, W(q == 2, hi == 1, W(q == 3, ~gt, hi == 0))), 15, 0)
        elif v == 2: t = W(hg, 15, 0)
        else:        t = W(gs, 15, 0)
    elif typ == 11:   # basket
        if v == 2: d = ia0 ^ ib0
        else:      d = b5(pa ^ pb)
        x = W(d == 1, pa, pb)
        t = [x & 15, W(((x >> 1) & 7) == 0, 1, 12), (x >> 1) & 15, W(b3(x), 12, 4)][v]
    elif typ == 12:   # zigzag
        hb = W(b5(pa) == 1, sm, df)
        t = [W(b4(ch), 15, 0), W(b3(hb), 15, 0), (ch >> 1) & 15, W(b3(ch), 15, 0)][v]
    elif typ == 13:   # brick
        ja = ((pa >> 2) & 7) == 0; jb = ((pb >> 2) & 7) == 0
        m0 = jb | (ja & (b5(pa) == b5(pb)))
        t = [W(m0, 2, 12), W(ja | jb, 2, 12), W(ja | ((pb >> 2) == 0), 2, 12), W(m0, 2, 6 + ((pb >> 2) & 7))][v]
    elif typ == 14:   # scale
        t = [W((sc >> 2) == 0, 2, 12), sc >> 1, W(b4(sc), 15, 0), W((sc >> 2) & 1, 13, 3)][v]
    else:             # cross
        plus = (((da >> 3) == 0) & ((db >> 3) != 3)) | (((db >> 3) == 0) & ((da >> 3) != 3))
        lat = ((df >> 2) == 0) | (((sm >> 2) & 15) == 15)
        star = (l1 < 12) | ((da >> 2) == 0) | ((db >> 2) == 0)
        t = [W(plus, 15, 1), W(lat, 2, 12), BAYER[(pb >> 4) * 4 + (pa >> 4)], W(star, 15, 1)][v]
    t = np.asarray(t, np.int64) & 15
    return 15 - t if inv else t

# ----------------------------------------------------------------------------------------
# colour (shared with VENEER)
# ----------------------------------------------------------------------------------------
_M = {0: (0, 0, 0, 0, 0), 1: (40, 37, 28, 15, 0), 2: (80, 74, 57, 31, 0), 3: (60, 55, 42, 23, 0)}
def hue_uv(h, s):
    """8-bit chroma (128 centred) of hue16 h at saturation code s, as f_hue_uv"""
    h = np.asarray(h) & 15; s = np.asarray(s)
    U = np.zeros(h.shape, np.int64); V = U.copy()
    for sc in range(4):
        m = np.array(_M[sc])
        r = h & 3; q = h >> 2
        cu = np.where(q == 0, m[r], np.where(q == 1, -m[4 - r], np.where(q == 2, -m[r], m[4 - r])))
        cv = np.where(q == 0, m[4 - r], np.where(q == 1, m[r], np.where(q == 2, -m[4 - r], -m[r])))
        U = np.where(s == sc, cu, U); V = np.where(s == sc, cv, V)
    return 128 + U, 128 + V

def relief_of(t4, il=0):
    """f5_rel: the top two bits of t against the sample two pixels left and two lines up (one
    field line when interlaced), held at half horizontal resolution in the relief line buffer"""
    t2 = t4 >> 2
    H, W = t2.shape
    dy = 1 if il else 2
    up = np.vstack([t2[:1].repeat(dy, axis=0), t2[:-dy]])
    xs = np.maximum((np.arange(W) - 2) & ~1, 0)
    rel = 21 * (t2 - up[:, xs])
    rel[:, :12] = 0                                   # the line's first pixels have no sample yet
    return rel

def fill(lab, cen, set_i=0, depth=10, colour=2, relief=True, hue=0, bias=False, invert=False, seams=True,
         scale_k=384, timing=13, xoff=0, yoff=0):
    """lab: dict lum/hue/sat (H x W, full res).  Returns 10-bit Y, U, V."""
    lum, hu, sat = lab['lum'], lab['hue'], lab['sat']
    H, W = lum.shape
    cx, cy, cu, cv = coords(W, H, scale_k, timing, xoff, yoff)
    X = np.meshgrid(np.arange(W), np.arange(H))[0]
    slot = lum if colour != 3 else (lum + hu + ((hu & 3) << 1) + sat) & 7
    srom = tables.srom_words()
    t4 = np.zeros((H, W), np.int64)
    hA = t4.copy(); sA = t4.copy(); hB = t4.copy(); sB = t4.copy()
    h1 = hu * 2 + 1
    for s_ in range(8):
        mk = slot == s_
        if not mk.any(): continue
        w0 = srom[(set_i * 8 + s_) * 2]; w1 = srom[(set_i * 8 + s_) * 2 + 1]
        t4[mk] = pattern(w0, cx, cy, cu, cv, bias, invert)[mk]
        pa = (w1 >> 12) & 15; psa = (w1 >> 10) & 3; pb = (w1 >> 6) & 15; psb = (w1 >> 4) & 3
        dh = (w1 >> 1) & 7; dh = dh - 8 if dh >= 4 else dh; ds = w1 & 1
        if colour == 0:
            hA[mk] = h1[mk]; sA[mk] = sat[mk]
            hB[mk] = np.where(sat[mk] == 0, h1[mk], (h1[mk] + dh) & 15)
            sB[mk] = np.where(sat[mk] == 0, 0, 2 if ds else sat[mk])
        elif colour == 1:
            hA[mk] = (h1[mk] + pa) & 15; sA[mk] = np.maximum(1, sat[mk])
            hB[mk] = (h1[mk] + pa + dh) & 15; sB[mk] = 2
        else:
            hA[mk] = pa; sA[mk] = psa; hB[mk] = pb; sB[mk] = psb
    UA, VA = hue_uv(hA + hue, sA); UB, VB = hue_uv(hB + hue, sB)
    dU = UB - UA; dV = VB - VA
    t = t4 * 17
    rel = relief_of(t4, 1 if timing in (0, 8, 1, 2, 10) else 0)
    ts = np.empty_like(t); acc = t[:, 0] * 4
    for x in range(W):
        acc = acc + t[:, x] - (acc >> 2); ts[:, x] = acc >> 2
    e = t - 128 + (2 * rel if relief else 0)
    px0 = X & 1
    m = (ts + np.where(px0 == 1, 48, 16)) >> 6
    g1 = np.where(m == 4, dU, np.where((m & 2) != 0, dU >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dU >> 2, 0)
    U8 = UA + g1 + g2
    g1 = np.where(m == 4, dV, np.where((m & 2) != 0, dV >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dV >> 2, 0)
    V8 = VA + g1 + g2
    Yo = np.clip(cen[lum] * 4 + ((e * depth) >> 3), 64, 940)
    U10 = U8 * 4; V10 = V8 * 4
    if seams:
        lb = lum * 32 + hu * 4 + sat
        bd = (lb != np.hstack([lb[:, :2], lb[:, :-2]])) | (lb != np.vstack([lb[:1], lb[:-1]]))
        Yo = np.where(bd, 64, Yo); U10 = np.where(bd, 512, U10); V10 = np.where(bd, 512, V10)
    return Yo, U10, V10

# ----------------------------------------------------------------------------------------
# approximate matte pipeline (labels) and output conversion (as VENEER's model.py)
# ----------------------------------------------------------------------------------------
FDEL = {2: 110, 3: 73, 4: 55, 5: 44, 6: 37, 7: 31, 8: 27}

def _mode5(a, axis):
    pads = [(0, 0), (0, 0)]; pads[axis] = (2, 2)
    p = np.pad(a, pads, mode='edge'); n = a.shape[axis]
    taps = [np.take(p, range(k, k + n), axis=axis) for k in range(5)]
    cnt = [sum((taps[i] == taps[j]).astype(np.int32) for j in range(5)) for i in range(5)]
    best = taps[2].copy(); bc = cnt[2].copy()
    for i in (1, 3, 0, 4):
        mm = cnt[i] > bc
        best[mm] = taps[i][mm]; bc[mm] = cnt[i][mm]
    return best

def mattes(img, nlev=4, adaptive=True):
    a = np.asarray(img.convert('RGB')).astype(np.float64) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    Yf = np.clip((0.299 * r + 0.587 * g + 0.114 * b) * 219 + 16, 16, 235).astype(np.int32)
    Uf = np.clip((-0.168736 * r - 0.331264 * g + 0.5 * b) * 224 + 128, 16, 240).astype(np.int32)
    Vf = np.clip((0.5 * r - 0.418688 * g - 0.081312 * b) * 224 + 128, 16, 240).astype(np.int32)
    H, W = Yf.shape; W2 = W // 2
    pa = lambda q: (q[:, 0:2 * W2:2] + q[:, 1:2 * W2:2] + 1) >> 1
    y, u, v = pa(Yf), pa(Uf), pa(Vf)
    p = np.pad(y, 1, mode='edge')
    c3 = p[:-2, :] + 2 * p[1:-1, :] + p[2:, :]
    ys = (c3[:, :-2] + 2 * c3[:, 1:-1] + c3[:, 2:]) >> 4
    if adaptive:
        hist = np.bincount((ys >> 2).ravel(), minlength=64)[:64]; cum = np.cumsum(hist); tot = cum[-1]
        thr = [min(235, int(np.searchsorted(cum, tot * k / nlev)) * 4 + 4) for k in range(1, nlev)]
        cen = [min(235, int(np.searchsorted(cum, tot * (k + 0.5) / nlev)) * 4 + 2) for k in range(nlev)]
    else:
        d = FDEL[nlev]; thr = [16 + d * k for k in range(1, nlev)]; cen = [16 + d * k + d // 2 for k in range(nlev)]
    lum = np.zeros_like(ys)
    for t in thr: lum += (ys > t).astype(np.int32)
    W4 = W2 // 2
    uq = (u[:, 0:2 * W4:2] + u[:, 1:2 * W4:2] + 1) >> 1; vq = (v[:, 0:2 * W4:2] + v[:, 1:2 * W4:2] + 1) >> 1
    def box(q):
        q2 = q + np.vstack([q[:1], q[:-1]])
        return (q2 + np.hstack([q2[:, :1], q2[:, :-1]])) >> 2
    du = box(uq) - 128; dv = box(vq) - 128; au, av = np.abs(du), np.abs(dv)
    sn = np.maximum(au, av) + (np.minimum(au, av) >> 1); gt = au > av
    hue = np.where((dv >= 0) & (du >= 0), np.where(gt, 0, 1), np.where((dv >= 0) & (du < 0), np.where(gt, 3, 2),
          np.where((dv < 0) & (du < 0), np.where(gt, 4, 5), np.where(gt, 7, 6))))
    sat = np.where(sn < 10, 0, np.where(sn < 36, 1, 2))
    hue = np.repeat(hue, 2, axis=1)[:, :W2]; sat = np.repeat(sat, 2, axis=1)[:, :W2]
    chroma = hue * 4 + sat
    lum = _mode5(_mode5(lum, 1), 0); chroma = _mode5(_mode5(chroma, 1), 0)
    up = lambda q: np.repeat(q, 2, axis=1)
    return dict(lum=up(lum), hue=up(chroma >> 2), sat=up(chroma & 3)), np.array(cen + [235] * (8 - len(cen)))

def to_rgb(Y, U, V):
    f = lambda q: (np.hstack([q[:, :1], q[:, :-1]]) + 2 * q + np.hstack([q[:, 1:], q[:, -1:]])) >> 2
    Y, U, V = f(Y), f(U), f(V)
    d = np.maximum(Y - 64, 0); Y = Y - (d >> 3) - (d >> 4)
    y = (np.clip(Y, 64, 940) - 64) / 876.0; cb = (U - 512) / 896.0; cr = (V - 512) / 896.0
    rgb = np.stack([y + 1.402 * cr, y - 0.344136 * cb - 0.714136 * cr, y + 1.772 * cb], -1)
    return (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)

# ----------------------------------------------------------------------------------------
# contact sheets
# ----------------------------------------------------------------------------------------
def _swatch(word, colw, S, scale_k, depth=10, hue=0, cen=140):
    lab = dict(lum=np.zeros((S, S), np.int64), hue=np.zeros((S, S), np.int64), sat=np.zeros((S, S), np.int64))
    cx, cy, cu, cv = coords(S, S, scale_k)
    t4 = pattern(word, cx, cy, cu, cv)
    t = t4 * 17
    rel = relief_of(t4)
    ts = np.empty_like(t); acc = t[:, 0] * 4
    for x in range(S):
        acc = acc + t[:, x] - (acc >> 2); ts[:, x] = acc >> 2
    X = np.meshgrid(np.arange(S), np.arange(S))[0]
    m = (ts + np.where((X & 1) == 1, 48, 16)) >> 6
    pa = (colw >> 12) & 15; psa = (colw >> 10) & 3; pb = (colw >> 6) & 15; psb = (colw >> 4) & 3
    UA, VA = hue_uv(np.full((S, S), pa + hue), np.full((S, S), psa)); UB, VB = hue_uv(np.full((S, S), pb + hue), np.full((S, S), psb))
    dU = UB - UA; dV = VB - VA
    g1 = np.where(m == 4, dU, np.where((m & 2) != 0, dU >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dU >> 2, 0)
    U8 = UA + g1 + g2
    g1 = np.where(m == 4, dV, np.where((m & 2) != 0, dV >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dV >> 2, 0)
    V8 = VA + g1 + g2
    e = t - 128 + 2 * rel
    Yo = np.clip(cen * 4 + ((e * depth) >> 3), 64, 940)
    return to_rgb(Yo, U8 * 4, V8 * 4)

def sheet(out, scale_k=384, S=200):
    img = Image.new('RGB', (4 * (S + 8) + 8, 16 * (S + 8) + 8), (20, 20, 20))
    d = ImageDraw.Draw(img)
    for ty in range(16):
        for v in range(4):
            w = ty | (v << 4) | (1 << 6) | ((1 << 12) if ty == 14 else 0)
            sw = _swatch(w, tables.colour_word(tables.C(0, 0, 0, 0)), S, scale_k)
            img.paste(Image.fromarray(sw), (8 + v * (S + 8), 8 + ty * (S + 8)))
            d.text((12 + v * (S + 8), 10 + ty * (S + 8)), f'{tables.PATTERNS[ty]} {v}', fill=(255, 60, 60))
    img.save(out)

def sets_sheet(out, scale_k=384, S=200):
    img = Image.new('RGB', (8 * (S + 8) + 8, 8 * (S + 8) + 8), (20, 20, 20))
    d = ImageDraw.Draw(img)
    srom = tables.srom_words()
    for si in range(8):
        for sl in range(8):
            sw = _swatch(srom[(si * 8 + sl) * 2], srom[(si * 8 + sl) * 2 + 1], S, scale_k, cen=60 + 22 * sl)
            img.paste(Image.fromarray(sw), (8 + sl * (S + 8), 8 + si * (S + 8)))
        d.text((12, 10 + si * (S + 8)), tables.SET_NAMES[si], fill=(255, 255, 255))
    img.save(out)

if __name__ == '__main__':
    a = sys.argv[1:]
    opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    if a[0] == '--sheet':
        sheet(a[1], opt('--scale', 384)); print('wrote', a[1]); sys.exit(0)
    if a[0] == '--sets':
        sets_sheet(a[1], opt('--scale', 384)); print('wrote', a[1]); sys.exit(0)
    img = Image.open(a[0]).convert('RGB').resize((1920, 1080), Image.LANCZOS)
    lab, cen = mattes(img, opt('--levels', 5), adaptive='--fixed' not in a)
    Y, U, V = fill(lab, cen, opt('--set', 0), opt('--depth', 10), opt('--colour', 2), '--flat' not in a,
                   opt('--hue', 0), '--bias' in a, '--invert' in a, '--noseams' not in a, opt('--scale', 384))
    Image.fromarray(to_rgb(Y, U, V)).save(a[1])
    print('wrote', a[1])
