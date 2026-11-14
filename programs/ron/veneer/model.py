#!/usr/bin/env python3
"""model.py -- integer model of the VENEER fill (noise fields, shaping, colour), mirroring
veneer.vhd operation for operation, plus an approximate model of the matte pipeline so the
fills can be previewed on any image at true 1080p scale without a simulation.

  python3 model.py IMAGE OUT.png [--set N] [--mode ink|tex] [--depth 0..15] [--colour 0..3]
                   [--relief 0..3] [--hue 0..15] [--levels 2..8] [--dense] [--fixed]

Needs numpy + Pillow (the SDK image tester's venv has both)."""
import sys, math, pathlib
import numpy as np
from PIL import Image
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tables

# ----------------------------------------------------------------------------------------
# noise fields
# ----------------------------------------------------------------------------------------
_R = np.array(tables.NROM, np.int64)
def line_offsets(y):
    """this line's table offsets (octave 1, octave 2): two ROM words combined"""
    w0 = _R[y & 255]; w1 = _R[(y >> 3) & 255]
    hi = ((w0 >> 8) + (w1 & 255)) & 255; lo = ((w0 & 255) + (w1 >> 8)) & 255
    return lo, hi

def rnd16(c, y, o):
    """the 16 random bits node c filters on line y: R[(c + offset(line, octave)) mod 256]"""
    lo, hi = line_offsets(y)
    return _R[(c + (hi if o else lo)) & 255]

def node_gain(s2, k):
    """node value (centred, +-511) from the 8.8 state; the gain follows the pole shift
    (k 3: /16, k 4: x1.5 / 16, k 5: / 8)"""
    c = s2 - 0x8000
    v = c >> (3 if k == 5 else 4)
    if k == 4: v = v + (v >> 1)
    return np.clip(v, -511, 511)

KB = [5, 4]          # vertical pole shift of octave 1 / 2

class Octave:
    """o: 0 = octave 1 (nodes 64 px apart, pole shift 5), 1 = octave 2 (16 px, shift 4)"""
    def __init__(self, o):
        self.o = o
        self.n = 64 if o == 0 else 128
        self.s1 = np.full(self.n, 0x8000, np.int64); self.s2 = self.s1.copy()

    def line(self, W, y, xo, il=0):
        """advance the vertical filters of the nodes of line y and return the W values"""
        o = self.o
        lD = 6 if o == 0 else 4; D = 1 << lD
        k = KB[o] - il
        a = 3 if o == 0 else 2
        xo &= 0xFFF
        c0 = xo >> lD; k0 = xo & (D - 1)
        nn = (W - 1 + k0) // D + 3                      # nodes fetched: 3 at the line start, 1 per boundary
        nodes = (c0 + np.arange(nn)) & 0xFF
        idx = nodes % self.n
        v = rnd16(nodes, y, o)
        u16 = ((v >> 8) << 8) | (v & 0xFF & ((1 << k) - 1))
        mk = (1 << k) - 1
        s1d = (self.s1[idx] & ~mk) | ((v >> 2) & mk)
        d1 = u16 - self.s1[idx]; d2 = s1d - self.s2[idx]
        self.s1[idx] += d1 >> k; self.s2[idx] += d2 >> k
        val = node_gain(self.s2[idx], k)
        kk = np.arange(W) + k0; m = kk >> lD; f = kk & (D - 1)
        r = val[m] + (((val[m + 1] - val[m]) * f) >> lD)
        acc = r[0] << a; out = np.empty(W, np.int64)
        for x in range(W):
            if x: acc = acc + r[x] - (acc >> a)
            out[x] = acc >> a
        return out

def fields(W, H, frame=2, drift=0, il=0, warm=2):
    """n1, n2 (centred, about +-511) for one frame; the vertical filters run on across frames
    (warm = frames run before the one returned).  drift = K5 pixels per frame (octave 2)."""
    o1, o2 = Octave(0), Octave(1)
    n1 = np.zeros((H, W), np.int64); n2 = n1.copy()
    for fr in range(frame - warm, frame + 1):
        x1, x2 = 0, -drift * fr
        for y in range(H):
            a = o1.line(W, y, x1, il); b = o2.line(W, y, x2, il)
            if fr == frame: n1[y] = a; n2[y] = b
            if il or (y & 1) == 0: x1 += 1
            if il or (y & 1) == 1: x2 -= 1
    return n1, n2

# ----------------------------------------------------------------------------------------
# shaping (T1..T4)
# ----------------------------------------------------------------------------------------
def tex_value(word, n1, n2, X, Y, dense=False):
    g1 = (word >> 13) & 7; g2 = (word >> 10) & 7; sx = (word >> 7) & 7; sy = (word >> 4) & 7
    yneg = (word >> 3) & 1; shape = word & 7
    d = 1 if dense else 0
    ta = (n1 << (g1 + d)) if g1 != 7 else 0 * n1
    tb = (n2 << (g2 + d)) if g2 != 7 else 0 * n2
    def stripe(coord, s):
        if s == 0: return 0 * coord
        return (coord << (s + 1)) & 1023
    tc = stripe(X, sx); td = stripe(Y, sy)
    adj = 512 if ((g1 == 0 and not dense) != (g2 == 0 and not dense)) else 0
    ab = ta + tb
    cd = ((tc - td) if yneg else (tc + td)) + adj
    p = (ab + cd) & 1023
    c = np.clip(ab, -512, 511)
    v8 = (c >> 2) + 128
    tr8 = np.where(p < 512, p >> 1, (1023 - p) >> 1)
    a9 = np.where(c < 0, -c - 1, c)                     # ones' complement magnitude
    if shape == 0: t = v8
    elif shape == 1: t = tr8
    elif shape == 2: t = np.where(tr8 < 64, tr8 << 2, 255)
    elif shape == 3: t = np.where((v8 >> 5) <= 2, 0, np.where((v8 >> 5) >= 5, 255, 128))
    elif shape == 4: t = np.where(a9 < 128, a9 << 1, 255)
    elif shape == 5: t = np.where((c >= -64) & (c < 64), (c + 64) << 1, np.where(c < 0, 0, 255))
    elif shape == 6: t = np.where((tr8 >> 5) <= 2, 0, np.where((tr8 >> 5) >= 5, 255, (tr8 - 96) << 1))
    else:            t = np.where((tr8 >> 5) <= 2, 0, np.where((tr8 >> 5) >= 5, 255, 128))
    return t.astype(np.int64)

# ----------------------------------------------------------------------------------------
# colour
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

def fill(lab, cen, set_i=0, mode='tex', depth=12, colour=0, relief=2, hue=0, dense=False, light=False,
         invert=False, ground=False, n=None, frame=2, drift=0):
    """lab: dict lum/hue/sat (H x W, full res).  mode 'tex' / 'ink'.  relief 0..3 (K6), hue 0..15
    (K2).  Returns 10-bit Y, U, V before the ink lines."""
    lum, hu, sat = lab['lum'], lab['hue'], lab['sat']
    H, W = lum.shape
    n1, n2 = n if n is not None else fields(W, H, frame, drift)
    X, Y = np.meshgrid(np.arange(W), np.arange(H))
    slot = lum if colour != 3 else (lum + hu + ((hu & 3) << 1) + sat) & 7
    srom = tables.srom_words()
    t = np.zeros((H, W), np.int64)
    hA = t.copy(); sA = t.copy(); hB = t.copy(); sB = t.copy()
    h1 = hu * 2 + 1
    for s_ in range(8):
        mk = slot == s_
        if not mk.any(): continue
        w0 = srom[(set_i * 8 + s_) * 2]; w1 = srom[(set_i * 8 + s_) * 2 + 1]
        t[mk] = tex_value(w0, n1, n2, X, Y, dense)[mk]
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
    tex = mode == 'tex'
    if invert: t = 255 - t
    # T5: relief, smoothed t for the colour ramp
    t3 = np.hstack([t[:, :1].repeat(3, axis=1), t[:, :-3]])
    rel = np.clip(t - t3, -64, 63)
    rel = [0 * rel, rel >> 1, rel, rel << 1][relief]
    ts = np.empty_like(t); acc = t[:, 0] * 4
    for x in range(W):
        acc = acc + t[:, x] - (acc >> 2); ts[:, x] = acc >> 2
    e = t - 128
    if tex: e = e - rel if light else e + rel
    px0 = (X + 1) & 1
    m = (ts + np.where(px0 == 1, 48, 16)) >> 6 if tex else 0 * t
    g1 = np.where(m == 4, dU, np.where((m & 2) != 0, dU >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dU >> 2, 0)
    U8 = UA + g1 + g2
    g1 = np.where(m == 4, dV, np.where((m & 2) != 0, dV >> 1, 0)); g2 = np.where(((m & 4) == 0) & ((m & 1) != 0), dV >> 2, 0)
    V8 = VA + g1 + g2
    ey = e * depth
    Yo = cen[lum] * 4 + ((ey >> 4) if tex else (ey >> 6))
    Yo = np.clip(Yo, 64, 940)
    U10 = U8 * 4; V10 = V8 * 4
    if not tex and not ground:
        bg = lum == 0
        Yo = np.where(bg, 64, Yo); U10 = np.where(bg, 512, U10); V10 = np.where(bg, 512, V10)
    return Yo, U10, V10

# ----------------------------------------------------------------------------------------
# approximate matte pipeline (labels) and output conversion
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

if __name__ == '__main__':
    a = sys.argv[1:]
    opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    img = Image.open(a[0]).convert('RGB').resize((1920, 1080), Image.LANCZOS)
    lab, cen = mattes(img, opt('--levels', 5), adaptive='--fixed' not in a)
    Y, U, V = fill(lab, cen, opt('--set', 1), opt('--mode', 'tex'), opt('--depth', 12), opt('--colour', 2),
                   opt('--relief', 2), opt('--hue', 0), '--dense' in a)
    Image.fromarray(to_rgb(Y, U, V)).save(a[1])
    print('wrote', a[1])
