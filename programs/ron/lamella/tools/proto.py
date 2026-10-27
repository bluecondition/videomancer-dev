#!/usr/bin/env python3
"""Lamella look-proto (float, NOT bit-accurate): calibrates the tile profile,
corner rounding, gloss and palette against the reference, and emits the ROM
tables used by lamella.vhd (--emit).

usage: proto.py [--img source.png] [--out out.png] [--emit]
"""
import argparse
import numpy as np
from PIL import Image

W, H = 960, 540


def env_synth():
    """Diagonal landscape: lime sky, pale steel, copper horizon, dark ground."""
    yy, xx = np.mgrid[0:H, 0:W].astype(float)
    t = (yy / H) - 0.45 * (xx / W - 0.5)          # bands rise to the right
    keys = [(-0.3, (230, 240, 170)), (0.25, (190, 210, 200)), (0.42, (150, 160, 165)),
            (0.52, (220, 120, 55)), (0.60, (60, 55, 50)), (0.9, (25, 20, 18)), (1.4, (40, 38, 36))]
    img = np.zeros((H, W, 3))
    for i in range(len(keys) - 1):
        t0, c0 = keys[i]
        t1, c1 = keys[i + 1]
        m = (t >= t0) & (t < t1)
        f = ((t - t0) / (t1 - t0))[m][:, None]
        img[m] = np.array(c0) * (1 - f) + np.array(c1) * f
    return img


def rgb2yuv(img):
    r, g, b = img[..., 0] / 255, img[..., 1] / 255, img[..., 2] / 255
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, (b - y) * 0.564, (r - y) * 0.713


def yuv2rgb(y, u, v):
    r = y + 1.403 * v
    b = y + 1.773 * u
    g = (y - 0.299 * r - 0.114 * b) / 0.587
    return np.clip(np.stack([r, g, b], -1) * 255, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- tables ---
def profile_rom():
    """256 entries over d in [0, 0.5): (delta Q8 tiles, highlight signed)."""
    d = (np.arange(256) + 0.5) / 512.0
    x = 2 * d                                       # 0..1
    # read distance from the tile centre (tiles): gentle magnification in the
    # middle, a steep run at the rim that crushes ~1.4 tiles into a few px
    p = 0.8 * d + 1.2 * x ** 7
    delta = np.clip(np.round((p - d) * 256), -512, 511).astype(int)   # signed Q8 tiles
    # highlight: thin bright line just inside the rim, dark groove past it
    hl = 40 * np.exp(-((x - 0.90) / 0.035) ** 2) - 48 * np.clip((x - 0.955) / 0.045, 0, 1) ** 0.7
    hl = np.clip(np.round(hl), -64, 63).astype(int)
    return delta, hl


def palette(sel):
    """256-entry luma -> (U, V) in BT.601 (swap happens in the emitter)."""
    if sel == 0:   # Copper / lime / steel (the reference)
        keys = [(0.00, (22, 16, 12)), (0.18, (58, 46, 38)), (0.33, (120, 70, 40)),
                (0.42, (215, 118, 55)), (0.52, (150, 160, 168)), (0.70, (196, 210, 206)),
                (0.85, (208, 232, 150)), (1.00, (238, 246, 200))]
    elif sel == 1: # Ice: ink / teal / violet / frost
        keys = [(0.00, (10, 12, 22)), (0.20, (14, 46, 66)), (0.38, (70, 70, 140)),
                (0.55, (120, 170, 205)), (0.75, (175, 220, 235)), (1.00, (240, 246, 252))]
    elif sel == 2: # Brass
        keys = [(0.00, (20, 14, 6)), (0.30, (110, 70, 20)), (0.55, (200, 150, 50)),
                (0.80, (245, 220, 130)), (1.00, (255, 250, 220))]
    else:          # Chrome: near-mono, faint blue
        keys = [(0.00, (8, 10, 14)), (0.40, (70, 78, 90)), (0.60, (160, 170, 185)),
                (0.80, (225, 232, 240)), (1.00, (255, 255, 255))]
    ys = np.arange(256) / 255
    out = np.zeros((256, 3))
    for i in range(len(keys) - 1):
        t0, c0 = keys[i]
        t1, c1 = keys[i + 1]
        m = (ys >= t0) & (ys <= t1)
        f = ((ys[m] - t0) / (t1 - t0))[:, None]
        out[m] = np.array(c0) * (1 - f) + np.array(c1) * f
    _, u, v = rgb2yuv(out[None])
    return u[0], v[0]


# ---------------------------------------------------------------- render ---
def render(env, ncol=7.0, ntile=4.0, lean=-0.12, stair=0.18, s=1.0, gloss=1.0,
           metal=1.0, pal=0, brushed=True, concave=False, shuffle=False):
    ey, eu, ev = rgb2yuv(env)
    # coarse store 128x64 (box), bilinear readback
    def store(c, gw, gh):
        return c.reshape(gh, H // gh, gw, W // gw).mean(axis=(1, 3))
    sy = np.round(store(ey, 60, 60) * 255) / 255         # 8-bit luma, 64x64
    su = store(eu, 30, 30)
    sv = store(ev, 30, 30)

    def bil(g, u, v):
        gh, gw = g.shape
        u = np.clip(u * gw - 0.5, 0, gw - 1.001)
        v = np.clip(v * gh - 0.5, 0, gh - 1.001)
        iu, iv = u.astype(int), v.astype(int)
        fu, fv = u - iu, v - iv
        a = g[iv, iu] * (1 - fu) + g[iv, iu + 1] * fu
        b = g[iv + 1, iu] * (1 - fu) + g[iv + 1, iu + 1] * fu
        return a * (1 - fv) + b * fv

    delta, hl = profile_rom()
    yy, xx = np.mgrid[0:H, 0:W].astype(float)
    A = (xx - W / 2 + lean * (yy - H / 2)) * ncol / W
    c = np.floor(A)
    lx = A - c
    rng = np.random.default_rng(3)
    stag = rng.random(256)
    B = yy * ntile / H + stag[(c.astype(int)) & 255]
    t = np.floor(B)
    ly = B - t
    d = np.abs(ly - 0.5)
    e = np.abs(lx - 0.5)
    e8 = np.minimum((e * 512).astype(int), 255)
    eex = np.maximum(e8 - 224, 0)
    d8 = np.minimum((d * 512).astype(int) + (eex * eex >> 2), 255)
    sgn = np.where(ly < 0.5, -1, 1)
    if concave:
        sgn = -sgn
    dv_t = sgn * delta[d8] / 256 * s + stair * c
    if shuffle:
        dv_t += (rng.random((64, 256))[(t.astype(int)) & 63, c.astype(int) & 255] - 0.5) * 0.8
    v = yy / H + dv_t / ntile
    hlens = (np.sign(lx - 0.5) * (0.3 * e + 0.8 * (eex / 32.0) ** 2)) * s
    u = xx / W + hlens / ncol
    y = bil(sy, u, v)
    disp = 0.06 * gloss
    uu = bil(su, u, v + disp * dv_t / ntile)
    vv = bil(sv, u, v + disp * dv_t / ntile)
    # metal: contrast + palette chroma
    pu, pv = palette(pal)
    yi = np.clip((y * 255).astype(int), 0, 255)
    yc = np.clip(0.5 + (y - 0.5) * (1 + 0.6 * metal), 0, 1)
    y = y * (1 - metal) + yc * metal
    uu = uu * (1 - metal) + pu[yi] * metal
    vv = vv * (1 - metal) + pv[yi] * metal
    # gloss: rim highlight (top rims brighter)
    h = hl[d8] / 64.0
    h = np.where((h > 0) & (sgn > 0), h * 0.5, h)
    y = y + 0.45 * gloss * h
    if brushed:
        key = (t.astype(int) * 977 + (ly * 300).astype(int) * 131 + c.astype(int) * 59) & 0xFFFF
        n = ((key * 2654435761) >> 13 & 255) / 255 - 0.5
        y = y + n * 0.16 * np.clip(1 - y * 1.6, 0, 1)
    return yuv2rgb(np.clip(y, 0, 1), uu, vv)


def emit():
    delta, hl = profile_rom()
    words = [((int(dl) & 0x3FF) << 6) | (int(h) & 0x3F) for dl, h in zip(delta, hl)]
    print("    -- profile ROM: delta(15..6) signed Q8 tiles, highlight(5..0) signed")
    for i in range(0, 256, 8):
        print("        " + ", ".join("x\"%04X\"" % w for w in words[i:i + 8]) + ",")
    print("    -- palette ROM: index pal & y(7 downto 1); U(15..8) V(7..0) signed, HW-swapped")
    for sel in (0, 1, 2, 3):
        pu, pv = palette(sel)
        ws = []
        for i in range(0, 256, 2):
            # hardware swaps U/V: BT.601 V goes into the U field
            uq = int(np.clip(np.round(pv[i] * 896 / 4), -112, 112)) & 0xFF
            vq = int(np.clip(np.round(pu[i] * 896 / 4), -112, 112)) & 0xFF
            ws.append((uq << 8) | vq)
        for i in range(0, 128, 8):
            print("        " + ", ".join("x\"%04X\"" % w for w in ws[i:i + 8]) + ",")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--img")
    ap.add_argument("--out", default="proto.png")
    ap.add_argument("--emit", action="store_true")
    a = ap.parse_args()
    if a.emit:
        emit()
    else:
        env = env_synth() if not a.img else np.asarray(
            Image.open(a.img).convert("RGB").resize((W, H)), float)
        rows = [render(env), render(env, metal=0.0, s=0.4, pal=1, concave=True),
                render(env, ncol=14, ntile=8, s=1.0, shuffle=True, pal=1)]
        Image.fromarray(np.concatenate(rows, 0)).save(a.out)
