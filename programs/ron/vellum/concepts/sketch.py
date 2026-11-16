"""Vellum concept sketches: float doodles of the ten ideas (NOT RTL models).
Run: videomancer-sdk/tools/vhdl-image-tester/.venv/bin/python programs/ron/vellum/concepts/sketch.py
Each figure is rendered at 2x and box-downsampled (2x2) to 640x360. Coordinates: y in [-0.5, 0.5] (screen
height = 1), x in [-0.889, 0.889]. Tone 0 = black, 1 = white."""
import os
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
W, H, SS = 640, 360, 2
TAU = 2 * np.pi


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
    """stepped grey by band index: black, dark, light, white, ..."""
    return (np.floor(phase) % n) / (n - 1)


def band(phase, duty=0.5):
    return (np.mod(phase, 1.0) < duty).astype(float)


# 1 ECLIPSE: large translucent discs (films of light) gliding past each other ------------------------------
def eclipse(rings=False, n=3, spread=0.27, t=0.0):
    x, y = grid()
    acc = np.zeros_like(x)
    for k in range(n):
        a = TAU * k / n + 0.6 + t * (1 + 0.37 * k) * (1 if k % 2 == 0 else -1)
        cx, cy = spread * np.cos(a) * 1.25, spread * np.sin(a) * 0.9
        R = 0.40 - 0.05 * k
        r = np.hypot(x - cx, y - cy)
        inside = (r < R).astype(float)
        if rings:
            inside *= band(r / (0.028 + 0.006 * k))
        acc += inside
    return acc / n if not rings else np.mod(acc, 2)


# 2 CARTOUCHE: exterior distance of a box -> sharp rectangles inside, rounded rectangles outward, circles far out
def cartouche(core=(0.20, 0.07), period=0.036, grey=True, pair=False):
    x, y = grid()

    def box_sdf(x, y, bx, by):
        qx, qy = np.abs(x) - bx, np.abs(y) - by
        return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)

    d = box_sdf(x, y, *core)
    ph = d / period + 0.25
    t = steps(ph) if grey else band(ph)
    if pair:
        d2 = box_sdf(y, x, *core)  # the same cartouche turned 90 degrees
        t = 0.5 * t + 0.5 * (steps(d2 / period + 0.25) if grey else band(d2 / period + 0.25))
    return t


# 3 TRINE: a triangular lattice, every cell a nest of triangles; or the Sierpinski gasket ----------------------
def trine(side=0.30, depth=5, gasket=False):
    x, y = grid()
    if gasket:
        # big triangle: base at y=-0.48, apex at y=+0.48
        hgt = 0.96
        s = hgt / (np.sqrt(3) / 2)
        b = (y + 0.48) / hgt
        a = (x + s / 2) / s - b / 2
        inside = (a >= 0) & (b >= 0) & (a + b <= 1)
        L = 9
        i = np.floor(a * 2 ** L).astype(np.int64)
        j = np.floor(b * 2 ** L).astype(np.int64)
        g = ((i & j) == 0) & inside
        return g.astype(float)
    b = y / (side * np.sqrt(3) / 2)
    a = x / side - b / 2
    fa, fb = a - np.floor(a), b - np.floor(b)
    up = fa + fb < 1
    b0 = np.where(up, fa, 1 - fa)
    b1 = np.where(up, fb, 1 - fb)
    b2 = np.where(up, 1 - fa - fb, fa + fb - 1)
    m = 3 * np.minimum(np.minimum(b0, b1), b2)  # 0 at the cell edge, 1 at its centre
    ph = m * depth
    return steps(ph + np.where(up, 0, 2))


# 4 WHORL: nested polygons whose shape changes going out (triangle -> square -> hexagon -> circle), whirling --
def whorl(N=3, twist=0.9, period=0.15, arms=0, a0=1.6, a1=-0.9, grey=True):
    x, y = grid()
    r = np.hypot(x, y) + 1e-9; th = np.arctan2(y, x); lr = np.log2(r)
    s = TAU / N
    th2 = th + twist * lr
    P = np.log2(np.cos(np.pi / N) / np.cos(np.mod(th2, s) - s / 2))   # <= 0, 0 at the corners
    f = np.clip((lr + 5.0) / 4.6, 0, 1)
    alpha = a0 + (a1 - a0) * f       # sharp/pinched polygon at the centre -> circle -> soft star outside
    ph = (lr - alpha * P) / period + arms * th / TAU
    return steps(ph) if grey else band(ph)



# 5 ROSETTE: halftone dot screens at slightly different angles -> a slow rosette moire -----------------------
def rosette(angles=(0, 5.0), pitch=0.032, rad=0.36, films=True):
    x, y = grid()
    acc = np.zeros_like(x)
    for k, a in enumerate(angles):
        c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
        u = (c * x + s * y) / pitch
        v = (-s * x + c * y) / pitch
        fu, fv = u - np.round(u), v - np.round(v)
        acc += (np.hypot(fu, fv) < rad).astype(float)
    return acc / len(angles) if films else np.mod(acc, 2)


# 6 DECADE: log-log graph paper; scrolling in log space is an endless zoom -----------------------------------
def decade(scale=60.0, curves=0.0):
    x, y = grid()
    X = (x + 0.93) * scale; Y = (y + 0.53) * scale
    px = 1.0 / (H * SS)

    def lines(Z):
        e = np.floor(np.log10(Z)); m = Z / 10 ** e
        n = np.clip(np.round(m), 1, 10)
        dist = np.abs(m - n) * 10 ** e / (scale * px)
        wid = np.where((n == 1) | (n == 10), 3.0, np.where(n == 5, 1.8, 1.1)) * SS
        tone = np.where((n == 1) | (n == 10), 1.0, np.where(n == 5, 0.7, 0.42))
        return np.clip(wid / 2 - dist + 0.5, 0, 1) * tone
    t = np.maximum(lines(X), lines(Y))
    if curves:
        L = np.log10(Y) - curves * np.log10(X); ph = L * 2
        g = np.hypot(1 / (Y * np.log(10)), curves / (X * np.log(10))) * 2 * scale * px  # |grad ph| per pixel
        t = np.maximum(t, np.clip(1.4 * SS - np.abs(ph - np.round(ph)) / g, 0, 1))
    return t




# 7 CASSINI: Cassini ovals around two points (two ovals -> figure eight -> peanut -> oval -> circle) ---------
def cassini(a=0.26, period=0.33, family="cassini"):
    x, y = grid()
    r1 = np.hypot(x - a, y) + 1e-9
    r2 = np.hypot(x + a, y) + 1e-9
    if family == "cassini":
        ph = (np.log2(r1 * r2) - 2 * np.log2(a)) / period
        return steps(ph + 0.5)
    e = (r1 + r2) / (0.06)
    hyp = (r1 - r2) / (0.06)
    return np.mod(np.floor(e) + np.floor(hyp), 2)


# 8 PHYLLO: dots on a log-polar lattice: sunflower parastichies, dots growing outward ------------------------
def phyllo(alpha=0.381966, eps=0.01027, rad=0.42, both=False):
    x, y = grid()
    r = np.hypot(x, y) + 1e-9
    rho, phi = np.log(r), np.arctan2(y, x)
    k0 = np.round(rho / eps)
    best = np.full(x.shape, 9.0)
    for dk in range(-40, 41):
        k = k0 + dk
        dr = rho - k * eps
        dp = np.mod(phi - TAU * k * alpha + np.pi, TAU) - np.pi
        best = np.minimum(best, np.hypot(dr, dp))
    nn = 0.25  # approx. nearest-neighbour spacing in (rho, phi) units for these parameters
    return (best < rad * nn).astype(float)


# 9 NAUTILUS: whirling golden rectangles with quarter arcs, squares stepped in grey -------------------------
def nautilus(levels=14, grey=True):
    x, y = grid()
    phi = (1 + 5 ** 0.5) / 2
    s = 0.98 / 1.0
    # rectangle [x0, x0+w] x [y0, y0+h], landscape phi:1 filling the height
    x0 = np.full(x.shape, -phi * s / 2)
    y0 = np.full(x.shape, -s / 2)
    w = np.full(x.shape, phi * s)
    h = np.full(x.shape, s)
    lvl = np.full(x.shape, -1.0)
    edge = np.zeros(x.shape)
    arc = np.zeros(x.shape)
    done = np.zeros(x.shape, bool)
    px = 1.0 / (H * SS)
    for L in range(levels):
        side = np.minimum(w, h)
        d = L % 4  # cut the square from: 0 left, 1 top, 2 right, 3 bottom
        if d == 0:
            sx0, sy0 = x0, y0
            cx, cy = x0 + side, y0  # arc centre: the square's lower-right corner
        elif d == 1:
            sx0, sy0 = x0, y0 + h - side
            cx, cy = x0, y0 + h - side
        elif d == 2:
            sx0, sy0 = x0 + w - side, y0
            cx, cy = x0 + w - side, y0 + side
        else:
            sx0, sy0 = x0, y0
            cx, cy = x0 + side, y0 + side
        ins = (~done) & (x >= sx0) & (x < sx0 + side) & (y >= sy0) & (y < sy0 + side)
        lvl[ins] = L
        de = np.minimum.reduce([x - sx0, sx0 + side - x, y - sy0, sy0 + side - y])
        edge[ins] = np.clip(1.6 - de[ins] / px, 0, 1)
        da = np.abs(np.hypot(x - cx, y - cy) - side)
        arc[ins] = np.clip(2.2 - da[ins] / px, 0, 1)
        done |= ins
        if d == 0:
            x0 = x0 + side; w = w - side
        elif d == 1:
            h = h - side
        elif d == 2:
            w = w - side
        else:
            y0 = y0 + side; h = h - side
    inside_any = lvl >= 0
    t = np.where(inside_any, (3 - (lvl % 4)) / 3 * 0.75 if grey else 0, 0)
    t = np.maximum(t, edge * 0.55)
    t = np.maximum(t, arc)
    return t


# 10 AQUEDUCT: rows of round arches at three depths, translucent grey films sliding at parallax speeds -------
def aqueduct(t=0.0):
    x, y = grid()
    greys = [0.62, 0.36, 0.0]     # far .. near (atmospheric: far light, near black)
    tone = np.full(x.shape, 0.86)  # sky
    for k in range(3):
        s = 0.30 * 1.8 ** k
        base = -0.5 + 0.30 - 0.15 * k
        xl = np.mod(x + t * s * 1.0 + 0.21 * k, s) - s / 2
        r = 0.30 * s; spring = base + 0.42 * s
        opening = (np.abs(xl) < r) & (y < spring) | (np.hypot(xl, y - spring) < r)
        top = spring + r + 0.16 * s
        wall = (y < top) & ~opening
        ring = (np.abs(np.hypot(xl, y - spring) - (r + 0.06 * s)) < 0.012 * s) & (y > spring) & wall
        tone = np.where(wall, greys[k], tone)
        tone = np.where(ring, 1.0 if k == 2 else greys[k] + 0.25, tone)
    return tone



PANELS = [
    ("1 eclipse: translucent discs", lambda: eclipse()),
    ("1 eclipse: Rings, Ink", lambda: eclipse(rings=True)),
    ("2 cartouche: Grey", lambda: cartouche()),
    ("2 cartouche: Ink, pair", lambda: cartouche(grey=False, pair=True)),
    ("3 trine: nested cells", lambda: trine()),
    ("3 trine: gasket", lambda: trine(gasket=True)),
    ("4 whorl: triangle -> circle -> star, Grey", lambda: whorl()),
    ("4 whorl: square, 4 arms, Ink", lambda: whorl(N=4, arms=4, twist=0.3, grey=False)),
    ("5 rosette: two screens 5 deg", lambda: rosette()),
    ("5 rosette: three screens", lambda: rosette(angles=(15, 45, 75), pitch=0.045)),
    ("6 decade: log-log paper", lambda: decade()),
    ("6 decade: with plotted curves", lambda: decade(curves=1.4)),
    ("7 cassini: Cassini ovals", lambda: cassini()),
    ("7 cassini: confocal", lambda: cassini(family="confocal")),
    ("8 phyllo: golden angle", lambda: phyllo()),
    ("8 phyllo: off golden", lambda: phyllo(alpha=0.3905)),
    ("9 nautilus: Grey", lambda: nautilus()),
    ("9 nautilus: Ink", lambda: nautilus(grey=False)),
    ("10 aqueduct: three depths", lambda: aqueduct()),
    ("10 aqueduct: slid", lambda: aqueduct(t=0.4)),
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
