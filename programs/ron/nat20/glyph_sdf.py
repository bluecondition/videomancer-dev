#!/usr/bin/env python3
"""Numeral signed-distance fields for NAT20.

Renders each glyph large with PIL, computes an exact Euclidean signed
distance (negative INSIDE the ink), samples it on a coarse texel grid and
quantises it the way the FPGA would store it.  sample() reproduces the
hardware read: four texels (one per 2x2-interleaved bank), bilinear blend,
plus the free gradient from the same four taps.

Fonts must be redistributable (the program is GPL): Lato Bold (OFL) is the
Classic candidate, DejaVu Serif Bold (Bitstream Vera licence) a stand-in for
Ornate.
"""
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONTS = {
    "classic": "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "ornate": "/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf",
}
GLYPHS = "0123456789"


def glyph_sdf(ch, font="classic", tex_w=16, tex_h=24, render_h=480,
              cell_aspect=0.62, spread=2.0, stretch_x=1.0, embolden=0.0):
    """Return (texels float in texel units, quantised int, meta).

    The glyph is fitted so its ink height fills `fill` of the cell height; the
    cell is tex_w x tex_h texels.  Distances are in TEXEL units, clamped to
    +-spread before quantising to 4 bits (16 levels).
    """
    fill = 0.80
    scale = render_h * 1.0
    f = ImageFont.truetype(FONTS[font], int(scale))
    # render on a big canvas, crop to ink bbox
    img = Image.new("L", (int(scale * 2), int(scale * 2)), 0)
    d = ImageDraw.Draw(img)
    d.text((scale * 0.3, scale * 0.1), ch, font=f, fill=255)
    a = np.array(img) > 127
    ys, xs = np.nonzero(a)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    ink_h = y1 - y0
    # high-res cell: ink height = fill * cell height, ink centred
    cell_h_px = ink_h / fill
    cell_w_px = cell_h_px * tex_w / tex_h
    cy, cx = (y0 + y1) / 2, (x0 + x1) / 2
    # texel centres in the high-res image
    ty = cy + (np.arange(tex_h) + 0.5 - tex_h / 2) * cell_h_px / tex_h
    tx = cx + (np.arange(tex_w) + 0.5 - tex_w / 2) * cell_w_px / tex_w / stretch_x
    # signed distance at texel centres, brute force against the ink boundary
    pad = np.pad(a, 1, constant_values=False)
    inner = a & ~(pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:])
    by, bx = np.nonzero(inner)
    by = by.astype(np.float64) + 0.5
    bx = bx.astype(np.float64) + 0.5
    TY, TX = np.meshgrid(ty, tx, indexing="ij")
    best = np.full(TY.shape, 1e9)
    pts = np.stack([TY.ravel(), TX.ravel()], 1)
    for i in range(0, len(by), 4000):
        dy = pts[:, :1] - by[None, i:i + 4000]
        dx = (pts[:, 1:] - bx[None, i:i + 4000]) * stretch_x
        best = np.minimum(best, np.sqrt(dy * dy + dx * dx).min(1).reshape(TY.shape))
    iy = np.clip(np.round(TY).astype(int), 0, a.shape[0] - 1)
    ix = np.clip(np.round(TX).astype(int), 0, a.shape[1] - 1)
    inside = a[iy, ix]
    sd = np.where(inside, -best, best)              # high-res pixels
    sd_tex = sd / (cell_h_px / tex_h) - embolden    # texel units
    q = quantise(sd_tex, spread)
    return sd_tex, q


# 4-bit code k <-> distance (k - ZERO) * STEP texels: 16 levels from -1.5 to
# +0.5 texel.  The inside gets three quarters of the range because the cut
# wall and the floor edge live there; outside only needs the AA ramp.
STEP = 0.125
ZERO = 11.5


def quantise(sd_tex, spread=None, bits=4):
    n = 1 << bits
    return np.clip(np.round(sd_tex / STEP + ZERO - 0.5) , 0, n - 1).astype(int)


def dequant(k, spread=None, bits=4):
    return (k - ZERO + 0.5) * STEP


def sample(qtex, u, v, spread=2.0):
    """Bilinear sample of a quantised SDF texture at texel coords (u, v)
    (u across, v down, texel centres at k + 0.5).  Returns (d, gx, gy) in
    texel units: value and the gradient from the same four taps."""
    th, tw = qtex.shape
    x = u - 0.5
    y = v - 0.5
    x0 = np.floor(x).astype(int)
    y0 = np.floor(y).astype(int)
    fx = x - x0
    fy = y - y0
    out = (x0 < 0) | (y0 < 0) | (x0 >= tw - 1) | (y0 >= th - 1)
    x0c = np.clip(x0, 0, tw - 2)
    y0c = np.clip(y0, 0, th - 2)
    t00 = dequant(qtex[y0c, x0c], spread)
    t10 = dequant(qtex[y0c, x0c + 1], spread)
    t01 = dequant(qtex[y0c + 1, x0c], spread)
    t11 = dequant(qtex[y0c + 1, x0c + 1], spread)
    a = t00 + (t10 - t00) * fx
    b = t01 + (t11 - t01) * fx
    d = a + (b - a) * fy
    gx = (t10 - t00) + ((t11 - t01) - (t10 - t00)) * fy
    gy = b - a
    d = np.where(out, dequant(15), d)
    gx = np.where(out, 0, gx)
    gy = np.where(out, 0, gy)
    return d, gx, gy


def build_font(font="classic", tex_w=16, tex_h=24, spread=2.0, stretch_x=1.0, embolden=0.0):
    return {ch: glyph_sdf(ch, font, tex_w, tex_h, spread=spread, stretch_x=stretch_x,
                          embolden=embolden)[1] for ch in GLYPHS}


if __name__ == "__main__":
    import sys
    # quick look: draw each glyph at 12x magnification from its SDF
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glyph_test.png")
    rows = []
    for (tw, th) in [(12, 18), (16, 24), (20, 30), (24, 36)]:
        F = build_font("classic", tw, th)
        mag = 300 / th
        H = 300
        W = int(tw * mag)
        tiles = []
        for ch in GLYPHS:
            yy, xx = np.mgrid[0:H, 0:W] + 0.5
            d, gx, gy = sample(F[ch], xx / mag, yy / mag)
            cov = np.clip(0.5 - d * mag, 0, 1)
            tiles.append((255 * (1 - cov)).astype(np.uint8))
        rows.append(np.concatenate(tiles, 1))
    wmax = max(r.shape[1] for r in rows)
    img = np.concatenate([np.pad(r, ((0, 10), (0, wmax - r.shape[1])), constant_values=255) for r in rows], 0)
    Image.fromarray(img).save(out)
    print("wrote", out)
