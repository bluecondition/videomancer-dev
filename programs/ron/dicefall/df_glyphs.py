#!/usr/bin/env python3
"""NAT20 numeral glyphs: 4-bit signed distance fields for the sprite sampler.

12 glyphs, 16 x 20 texels each: the digits 0..9, then 6 and 9 with the
underline the d10 and d20 use to tell them apart.  All digits share one
scale and baseline (Lato Bold's lining figures, cap height = 80 % of the
cell), each centred across its cell.

Code k <-> distance (k - ZERO) * STEP texels, negative inside the ink:
16 levels from -1 to +0.875 texel, which covers a 1-px anti-aliasing ramp
from ~0.5 px per texel (the smallest dice) up.

Storage, as the RTL reads it: four banks, 2 x 2 interleaved, so a bilinear
sample's four texels come from four different banks in one clock.  Bank
(bx, by) holds the texels with x & 1 = bx, y & 1 = by, at address
    glyph * 80 + (y >> 1) * 8 + (x >> 1)          (10 bits, 1024 x 4)

    python3 nat20_glyphs.py        # writes glyph_preview.png
"""
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
FONT = os.environ.get("NAT20_FONT", os.path.join(HERE, "fonts", "Lato-Bold.ttf"))
if not os.path.exists(FONT):
    FONT = "/mnt/c/Windows/Fonts/Lato-Bold.ttf"
TW, TH = 16, 20
NG = 12
FILL = 0.75                      # cap height / cell height
STEP, ZERO = 0.125, 8
UL_Y, UL_HALF, UL_W = 0.64, 0.065, 0.62  # underline: centre below the middle, half
                                         # thickness, width (all x cap height); the
                                         # digit + bar pair is centred in the cell
RENDER = 640                     # high-res render height (px)


def _render(ch):
    f = ImageFont.truetype(FONT, RENDER)
    img = Image.new("L", (RENDER * 2, RENDER * 2), 0)
    ImageDraw.Draw(img).text((RENDER * 0.4, RENDER * 0.2), ch, font=f, fill=255)
    return np.array(img) > 127


def _cap_box():
    """the digits' common vertical extent (lining figures share it)"""
    a = _render("0")
    ys = np.nonzero(a.any(1))[0]
    return ys.min(), ys.max() + 1


def glyph_codes(i):
    """quantised SDF (TH x TW) of glyph i (0..9 digits, 10 = 6_, 11 = 9_)"""
    ch = str(i if i < 10 else (6 if i == 10 else 9))
    a = _render(ch)
    y0, y1 = _cap_box()
    cap = y1 - y0
    xs = np.nonzero(a.any(0))[0]
    cx = (xs.min() + xs.max() + 1) / 2
    cy = (y0 + y1) / 2
    if i >= 10:                                     # underline bar
        uy = cy + UL_Y * cap
        ys0, ys1 = int(uy - UL_HALF * cap), int(uy + UL_HALF * cap)
        xh = UL_W * cap / 2
        a[ys0:ys1, int(cx - xh):int(cx + xh)] = True
        cy += (UL_Y + UL_HALF - 0.5) * cap / 2      # centre the pair
    cell_h = cap / FILL
    tex = cell_h / TH                               # high-res px per texel
    ty = cy + (np.arange(TH) + 0.5 - TH / 2) * tex
    tx = cx + (np.arange(TW) + 0.5 - TW / 2) * tex
    pad = np.pad(a, 1, constant_values=False)
    edge = a & ~(pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:])
    by, bx = np.nonzero(edge)
    by = by + 0.5
    bx = bx + 0.5
    TY, TX = np.meshgrid(ty, tx, indexing="ij")
    pts = np.stack([TY.ravel(), TX.ravel()], 1)
    best = np.full(len(pts), 1e9)
    for k in range(0, len(by), 4000):
        d = np.hypot(pts[:, :1] - by[None, k:k + 4000], pts[:, 1:] - bx[None, k:k + 4000])
        best = np.minimum(best, d.min(1))
    best = best.reshape(TY.shape)
    iy = np.clip(np.round(TY).astype(int), 0, a.shape[0] - 1)
    ix = np.clip(np.round(TX).astype(int), 0, a.shape[1] - 1)
    sd = np.where(a[iy, ix], -best, best) / tex     # texel units
    return np.clip(np.round(sd / STEP + ZERO), 0, 15).astype(int)


_CACHE = os.path.join(HERE, ".glyphs_16x20_v2.npy")


def glyph_set():
    """(NG, TH, TW) int codes, cached"""
    if os.path.exists(_CACHE):
        return np.load(_CACHE)
    g = np.stack([glyph_codes(i) for i in range(NG)])
    np.save(_CACHE, g)
    return g


def banks():
    """the four 1024 x 4 bank images, index 2 * by + bx"""
    g = glyph_set()
    out = [[15] * 1024 for _ in range(4)]
    for gi in range(NG):
        for y in range(TH):
            for x in range(TW):
                b = 2 * (y & 1) + (x & 1)
                out[b][gi * 80 + (y >> 1) * 8 + (x >> 1)] = int(g[gi, y, x])
    return out


def preview(path, mag=12):
    """each glyph drawn back from its stored field at mag px per texel"""
    g = glyph_set()
    tiles = []
    for gi in range(NG):
        q = g[gi]
        H, W = TH * mag, TW * mag
        yy, xx = np.mgrid[0:H, 0:W] + 0.5
        u, v = xx / mag - 0.5, yy / mag - 0.5
        x0, y0 = np.floor(u).astype(int), np.floor(v).astype(int)
        fx, fy = u - x0, v - y0

        def t(x, y):
            ok = (x >= 0) & (x < TW) & (y >= 0) & (y < TH)
            return np.where(ok, q[np.clip(y, 0, TH - 1), np.clip(x, 0, TW - 1)], 15)
        top = t(x0, y0) + (t(x0 + 1, y0) - t(x0, y0)) * fx
        bot = t(x0, y0 + 1) + (t(x0 + 1, y0 + 1) - t(x0, y0 + 1)) * fx
        d = (top + (bot - top) * fy - ZERO) * STEP * mag
        tiles.append((255 * np.clip(0.5 + d, 0, 1)).astype(np.uint8))
    Image.fromarray(np.concatenate(tiles, 1)).save(path)


if __name__ == "__main__":
    out = os.path.join(HERE, "glyph_preview.png")
    preview(out)
    print("font", FONT)
    print("wrote", out)
