"""Straightedge concept sketches: float doodles of the ten ideas (NOT RTL models).
Run: videomancer-sdk/tools/vhdl-image-tester/.venv/bin/python programs/ron/straightedge/concepts/sketch.py
Each figure is rendered at 2x and box-downsampled (2x2) to 640x360. Coordinates: y in [-0.5, 0.5] (screen
height = 1), x in [-0.889, 0.889]. Tone 0 = black, 1 = white. Every edge in every figure is a straight line."""
import os
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
W, H, SS = 640, 360, 2
TAU = 2 * np.pi
R3 = np.sqrt(3.0)


def grid():
    w, h = W * SS, H * SS
    y, x = np.mgrid[0:h, 0:w].astype(np.float64)
    x = (x + 0.5 - w / 2) / h
    y = (h / 2 - (y + 0.5)) / h
    return x, y


def out(t):
    t = np.clip(t, 0, 1)
    t = t.reshape(H, SS, W, SS).mean(axis=(1, 3))
    return Image.fromarray((t * 255 + 0.5).astype(np.uint8), "L")


def steps(phase, n=4):
    return (np.floor(phase) % n) / (n - 1)


def band(phase, duty=0.5):
    return (np.mod(phase, 1.0) < duty).astype(float)


def hash2(i, j, seed=0):
    h = (i.astype(np.int64) * 374761393 + j.astype(np.int64) * 668265263 + seed * 144665) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFF) / 65536.0


# 1 SLIVER: long translucent triangles (films of light) crossing the frame, each turning at its own rate ------
def sliver(n=5, spread=0.30, t=0.0, ink=False):
    x, y = grid()
    acc = np.zeros_like(x)
    for k in range(n):
        a = TAU * k / n + 0.35 + t * (1 if k % 2 == 0 else -0.75)
        ox, oy = spread * np.cos(2.4 * k + 0.9) * 1.5, spread * np.sin(1.7 * k + 0.4)
        d = a + np.pi + (0.5 * np.sin(3.1 * k) if spread > 0 else 0)   # axis direction, pointing back across the frame
        half = np.tan(np.radians(7 + 3 * (k % 3)))
        px, py = x - ox, y - oy
        along = px * np.cos(d) + py * np.sin(d)
        perp = -px * np.sin(d) + py * np.cos(d)
        acc += ((along > 0) & (np.abs(perp) < along * half)).astype(float)
    return np.mod(acc, 2) if ink else acc / 3.0


# 2 CREASE: a striped sheet folded along straight creases (each fold a reflection) ------------------------------
def crease(folds=((0.35, -0.10), (1.75, 0.16), (2.7, 0.05)), ang=0.5, period=0.085, grey=True):
    x, y = grid()
    px, py = x.copy(), y.copy()
    for (fa, fc) in folds:
        nx, ny = np.cos(fa), np.sin(fa)
        m = np.maximum(0.0, px * nx + py * ny - fc)
        px, py = px - 2 * nx * m, py - 2 * ny * m
    ph = (px * np.cos(ang) + py * np.sin(ang)) / period
    return steps(ph) if grey else band(ph)


# 3 KOCH: the snowflake's fold distance: nested polygonal bands, endlessly self-similar at x3 ------------------
def koch(levels=4, period=0.030, size=0.86, grey=True):
    x, y = grid()
    px, py = np.abs(x / size), y / size + 0.14
    py = py - R3 / 6
    n0, n1 = 0.5, -R3 / 2
    m = np.maximum(0.0, (px - 0.5) * n0 + py * n1)
    px, py = px - 2 * n0 * m, py - 2 * n1 * m
    n0, n1 = R3 / 2, -0.5
    sc = 1.0
    px = px + 0.5
    for _ in range(levels):
        px, py, sc = px * 3, py * 3, sc * 3
        px = np.abs(px - 1.5) - 0.5
        m = np.minimum(0.0, px * n0 + py * n1)
        px, py = px - 2 * n0 * m, py - 2 * n1 * m
    d = py / sc * size
    if grey:
        return np.where(d < 0, 0.33 + 0.67 * steps(-d / period + 0.999, 3) * 1.0, 0.0) * (d < 0) + \
            (d >= 0) * 0.33 * band(d / period)
    return np.maximum(band(np.abs(d) / period, 0.22), 0.0) * 1.0


# 4 MEANDER: Greek-key friezes, alternate rows running opposite ways -----------------------------------------------
KEY = ["XXXXXXX.",
       "X.....X.",
       "X.XXX.X.",
       "X.X...X.",
       "X.XXXXX.",
       "X.......",
       "XXXXXXXX"]


def meander(rows=4, shift=0.0, grey=True):
    x, y = grid()
    key = np.array([[c == "X" for c in r] for r in KEY])
    rh = 1.0 / rows
    cell = rh * 0.78 / 7
    r = np.floor((y + 0.5) / rh).astype(int)
    v = (y + 0.5) - r * rh - rh * 0.11
    kj = np.floor(v / cell).astype(int)
    odd = (r % 2 == 1)
    u = np.where(odd, -x, x) / cell + shift * np.where(odd, 0.75, 1.0) + r * 2.5
    ki = np.floor(u).astype(int) % 8
    inside = (kj >= 0) & (kj < 7)
    ink = np.zeros_like(x)
    ink[inside] = key[6 - kj[inside], ki[inside]]
    if grey:
        return np.where(odd, 0.20 + 0.47 * ink, ink * 1.0)
    return ink


# 5 TINES: rows of long needle triangles from the top and the bottom, interleaving like backgammon points ----------
def tines(reach=0.80, pitch=0.16, ink=False, pitch2=None):
    x, y = grid()
    p2 = pitch if pitch2 is None else pitch2

    def tri(u):
        return 1 - np.abs(2 * np.mod(u, 1.0) - 1)

    top = ((0.5 - y) < reach * tri(x / pitch)).astype(float)
    bot = ((y + 0.5) < reach * tri(x / p2 + 0.5)).astype(float)
    if ink:
        return np.mod(top + bot, 2)
    return 0.40 * top + 0.60 * bot


# 6 PENTAGRID: N families of crisp parallel bands; the stepped sum makes quasi-crystal stars ----------------------
def pentagrid(N=5, period=0.115, grey=True, fine=False, off=0.21):
    x, y = grid()
    acc = np.zeros_like(x)
    for k in range(N):
        th = np.pi * k / N + 0.05
        ph = (x * np.cos(th) + y * np.sin(th)) / period + off + 0.37 * k
        acc += band(ph, 0.12) if fine else band(ph)
    if fine:
        return np.clip(acc, 0, 1)
    return acc / N if grey else np.mod(acc, 2)


# 7 SPLINTER: the Conway-Radin pinwheel tiling: a 1:2 right triangle splits into five, turning every time ----------
def splinter(levels=4, grey=True):
    x, y = grid()
    s = 0.95                                # rectangle 2s x s, two triangles
    px, py = x + s, y + s / 2
    low = (py * 2 < px)                      # below the diagonal from (0,0) to (2s, s)
    A = [np.where(low, 2 * s, 0.0), np.where(low, 0.0, s)]
    B = [np.where(low, 0.0, 2 * s), np.where(low, 0.0, s)]
    C = [np.where(low, 2 * s, 0.0), np.where(low, s, 0.0)]
    P = [px, py]
    tone = np.zeros_like(x)
    edge = np.full_like(x, 9.0)
    for lv in range(levels):
        D = [C[i] + (B[i] - C[i]) * 0.2 for i in range(2)]
        E = [(A[i] + B[i]) / 2 for i in range(2)]
        F = [(D[i] + B[i]) / 2 for i in range(2)]
        G = [(A[i] + D[i]) / 2 for i in range(2)]
        kids = [(D, A, C), (G, E, A), (F, B, E), (E, G, F), (D, F, G)]   # (right angle, long-leg end, short-leg end)

        def inside(T):
            a, b, c = T
            d = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            u = ((P[0] - a[0]) * (c[1] - a[1]) - (P[1] - a[1]) * (c[0] - a[0])) / d
            v = ((b[0] - a[0]) * (P[1] - a[1]) - (b[1] - a[1]) * (P[0] - a[0])) / d
            return u, v

        best = np.full_like(x, -9.0)
        idx = np.zeros(x.shape, int)
        for k, T in enumerate(kids):
            u, v = inside(T)
            m = np.minimum(np.minimum(u, v), 1 - u - v)
            better = m > best
            best = np.where(better, m, best)
            idx = np.where(better, k, idx)
        nA, nB, nC = [np.zeros_like(x) for _ in range(2)], [np.zeros_like(x) for _ in range(2)], \
            [np.zeros_like(x) for _ in range(2)]
        for k, T in enumerate(kids):
            for i in range(2):
                nA[i] = np.where(idx == k, T[0][i], nA[i])
                nB[i] = np.where(idx == k, T[1][i], nB[i])
                nC[i] = np.where(idx == k, T[2][i], nC[i])
        A, B, C = nA, nB, nC
        tone = idx
        # distance to the nearest edge of the current triangle (in screen units)
        def dist(p0, p1):
            ex, ey = p1[0] - p0[0], p1[1] - p0[1]
            return np.abs((P[0] - p0[0]) * ey - (P[1] - p0[1]) * ex) / np.hypot(ex, ey)
        edge = np.minimum(np.minimum(dist(A, B), dist(A, C)), dist(B, C))
    line = (edge < 0.0022).astype(float)
    if grey:
        g = np.array([0.18, 0.42, 0.66, 0.90, 0.30])[tone]
        return g * (1 - line)
    return line


# 8 LOWPOLY: a faceted sheet: a mesh of triangles with jittered vertices, every facet one flat lit grey ------------
def lowpoly(cw=0.21, amp=0.42, shear=0.25, lines=False, t=0.0):
    x, y = grid()
    xs = x + shear * y
    i = np.floor(xs / cw).astype(int)
    f = xs / cw - i
    rh = cw * 0.9

    def vy(ii, jj):
        return jj * rh + amp * rh * (hash2(ii, jj, 1) - 0.5 + 0.25 * np.sin(TAU * (hash2(ii, jj, 5) + t)))

    def vz(ii, jj):
        return hash2(ii, jj, 9)

    j0 = np.floor(y / rh).astype(int)
    jj = j0.copy()
    for dj in (-1, 0, 1, 2):          # which band between polylines contains the pixel
        lo = vy(i, j0 + dj) * (1 - f) + vy(i + 1, j0 + dj) * f
        jj = np.where(y >= lo, j0 + dj, jj)
    par = ((i + jj) % 2 == 0)
    # quad corners: bottom polyline jj, top polyline jj+1
    bl, br, tl, tr = vy(i, jj), vy(i + 1, jj), vy(i, jj + 1), vy(i + 1, jj + 1)
    diag = np.where(par, bl + (tr - bl) * f, tl + (br - tl) * f)
    up = y >= diag
    zbl, zbr, ztl, ztr = vz(i, jj), vz(i + 1, jj), vz(i, jj + 1), vz(i + 1, jj + 1)
    # facet gradient -> flat shade
    gx = np.where(par, np.where(up, ztr - ztl, zbr - zbl), np.where(up, ztr - ztl, zbr - zbl))
    gy = np.where(par, np.where(up, ztl - zbl, ztr - zbr), np.where(up, ztr - zbr, ztl - zbl))
    shade = np.clip(0.5 + 0.55 * gx + 0.45 * gy, 0.04, 1.0)
    if lines:
        lo = bl * (1 - f) + br * f
        hi = tl * (1 - f) + tr * f
        w = 0.004
        e = (np.abs(y - lo) < w) | (np.abs(y - hi) < w) | (np.abs(y - diag) < w) | (f * cw < w * 1.2)
        return e.astype(float)
    return np.round(shade * 5) / 5


# 9 HINGE: rigid squares hinged at their corners; alternate squares turn opposite ways and the holes open ----------
def hinge(theta=20.0, s=0.20, grey=True, fine=False):
    x, y = grid()
    th = np.radians(theta)
    a = s * (np.cos(th) + np.sin(th))
    i, j = np.round(x / a), np.round(y / a)
    lx, ly = x - i * a, y - j * a
    par = ((i + j) % 2 == 0)
    sg = np.where(par, 1.0, -1.0)
    u = lx * np.cos(th) + sg * ly * np.sin(th)
    v = -sg * lx * np.sin(th) + ly * np.cos(th)
    d = np.maximum(np.abs(u), np.abs(v))
    inside = d < s / 2
    if fine:
        return (inside & (d > s / 2 - 0.012)).astype(float)
    if grey:
        return inside * np.where(par, 1.0, 0.55)
    return inside.astype(float)


# 10 WINDROSE: a compass rose: tiers of pointed stars, every point split light / dark, tiers turning apart ---------
def windrose(point=0.16, t=0.0, ink=False):
    x, y = grid()
    tone = np.zeros_like(x)
    tiers = [(0.30, 22.5, 0.55, 0.30), (0.30, 67.5, 0.55, 0.30), (0.40, 45.0, 0.72, 0.36), (0.49, 0.0, 1.0, 0.42)]
    acc = np.zeros_like(x)
    for (R, deg, hi, lo) in tiers:
        a = np.radians(deg) + t * (1 if deg % 45 == 0 else -1) * (0.49 / R)
        u = x * np.cos(a) + y * np.sin(a)
        v = -x * np.sin(a) + y * np.cos(a)
        au, av = np.abs(u), np.abs(v)
        mx, mn = np.maximum(au, av), np.minimum(au, av)
        w = point * R
        inside = (mx + (R - w) / w * mn) < R
        light = ((u > 0) ^ (v > 0) ^ (au > av))
        tone = np.where(inside, np.where(light, hi, lo * 0.55), tone)
        acc += inside
    return np.mod(acc, 2) if ink else tone


PANELS = [
    ("1 sliver: Grey (films), spread", lambda: sliver()),
    ("1 sliver: Ink, tips together (P12 0 %)", lambda: sliver(spread=0.0, ink=True)),
    ("2 crease: Grey, three folds", lambda: crease()),
    ("2 crease: Ink, one fold", lambda: crease(folds=((0.2, 0.0),), ang=1.1, period=0.07, grey=False)),
    ("3 koch: Grey bands", lambda: koch()),
    ("3 koch: Ink, Fine", lambda: koch(grey=False)),
    ("4 meander: Grey", lambda: meander()),
    ("4 meander: Ink, 3 rows", lambda: meander(rows=3, grey=False)),
    ("5 tines: Grey, deep reach", lambda: tines()),
    ("5 tines: Ink, two pitches", lambda: tines(reach=0.95, pitch=0.16, pitch2=0.128, ink=True)),
    ("6 pentagrid: Grey sum, N = 5", lambda: pentagrid(period=0.21)),
    ("6 pentagrid: Fine lines, N = 5", lambda: pentagrid(fine=True)),
    ("7 splinter: Grey, 3 levels", lambda: splinter(levels=3)),
    ("7 splinter: Ink lines, 4 levels", lambda: splinter(levels=4, grey=False)),
    ("8 lowpoly: Grey facets", lambda: lowpoly()),
    ("8 lowpoly: Fine (the mesh)", lambda: lowpoly(lines=True)),
    ("9 hinge: Grey, 20 degrees", lambda: hinge()),
    ("9 hinge: Ink, 38 degrees", lambda: hinge(theta=38.0, grey=False)),
    ("10 windrose: Grey", lambda: windrose()),
    ("10 windrose: Ink, sharper, turned", lambda: windrose(point=0.10, t=0.25, ink=True)),
]


def main():
    tiles = []
    for name, fn in PANELS:
        im = out(fn()).convert("RGB")
        tag = name.split(":")[0].split(" ")[1]
        im.save(os.path.join(HERE, "%s_%d.png" % (tag, len([t for t in tiles if t[0].split(':')[0] == name.split(':')[0]]))))
        tiles.append((name, im))
    cols, pad, lab = 2, 6, 16
    sheet = Image.new("RGB", (cols * (W + pad) + pad, (len(tiles) // cols) * (H + lab + pad) + pad), (48, 48, 48))
    d = ImageDraw.Draw(sheet)
    for i, (name, im) in enumerate(tiles):
        cx, cy = pad + (i % cols) * (W + pad), pad + (i // cols) * (H + lab + pad)
        sheet.paste(im, (cx, cy + lab))
        d.text((cx + 2, cy + 2), name, fill=(230, 230, 230))
    sheet.save(os.path.join(HERE, "sheet.png"))
    print("wrote", os.path.join(HERE, "sheet.png"))


if __name__ == "__main__":
    main()
