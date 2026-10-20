#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE model renders: test image, contact sheets, 4x edge crops, end checks.

  python3 sw_sheet.py sheet  OUTDIR        contact sheet + crops per mode
  python3 sw_sheet.py ends                 A/B dead-zone checks over the grid
"""
import sys, os, itertools
import numpy as np
from PIL import Image, ImageDraw
import sw_frame as fr

# ---------------------------------------------------------------- test image
def test_rgb(W=1920, H=1080):
    y, x = np.mgrid[0:H, 0:W].astype(np.float64)
    yn, xn = y / H, x / W
    img = np.zeros((H, W, 3))
    sky = np.stack([0.10 + 0.85 * yn, 0.12 + 0.45 * yn, 0.45 - 0.25 * yn], -1)
    img[:] = sky
    # sun
    d = np.hypot(x - 0.70 * W, y - 0.42 * H) / H
    img += np.clip(1 - d / 0.11, 0, 1)[..., None] * np.array([1.0, 0.85, 0.4])
    img += (0.25 * np.exp(-d / 0.08))[..., None] * np.array([1.0, 0.6, 0.2])
    # ridges
    for k, (amp, base, col) in enumerate([(0.06, 0.55, (0.35, 0.18, 0.45)),
                                          (0.05, 0.62, (0.22, 0.10, 0.30)),
                                          (0.04, 0.70, (0.10, 0.25, 0.12))]):
        ridge = base + amp * np.sin(xn * (7 + 3 * k) * np.pi + k) + 0.02 * np.sin(xn * 41 + k)
        m = yn > ridge
        img[m] = np.array(col) * (0.8 + 0.4 * (1 - yn[m]))[:, None]
    # ground grid
    g = yn > 0.70
    gx = np.abs(((xn - 0.5) / np.maximum(yn - 0.69, 0.01)) * 0.6 % 0.1 - 0.05) < 0.004
    gy = np.abs((1 / np.maximum(yn - 0.69, 0.01)) % 6 - 3) < 0.25
    img[g & (gx | gy)] = (0.9, 0.3, 0.8)
    # colour patches
    for i, col in enumerate([(1, 0.2, 0.2), (0.2, 1, 0.3), (0.2, 0.4, 1), (1, 1, 0.2),
                             (1, 0.3, 1), (0.2, 1, 1)]):
        cx, cy = 0.08 + 0.05 * i, 0.12
        m = (np.abs(xn - cx) < 0.018) & (np.abs(yn - cy) < 0.04)
        img[m] = col
    # fine stripes (detail)
    m = (yn > 0.20) & (yn < 0.30) & (xn > 0.05) & (xn < 0.35)
    img[m] *= (0.6 + 0.4 * (np.sin(x[m] * 0.9) > 0))[:, None]
    return np.clip(img, 0, 1)


def to_yuv(rgb):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    Y = 64 + np.round(876 * y)
    Cb = 512 + np.round(896 * (b - y) / 1.772)
    Cr = 512 + np.round(896 * (r - y) / 1.402)
    return np.stack([Y, Cr, Cb], -1).astype(np.int64)        # hardware order


def to_rgb8(yuv):
    Y = (yuv[..., 0] - 64) / 876
    Cr = (yuv[..., 1] - 512) / 896
    Cb = (yuv[..., 2] - 512) / 896
    r = Y + 1.402 * Cr
    b = Y + 1.772 * Cb
    g = (Y - 0.299 * r - 0.114 * b) / 0.587
    return (np.clip(np.stack([r, g, b], -1), 0, 1) * 255 + 0.5).astype(np.uint8)


_MASTER = None


def source(mode):
    """the test image at a mode's native raster (SD: 4:3 centre crop)"""
    global _MASTER
    if _MASTER is None:
        _MASTER = Image.fromarray((test_rgb() * 255 + 0.5).astype(np.uint8))
    M = fr.MODES[mode]
    im = _MASTER
    if M['W'] == 720:
        im = im.crop((240, 0, 1680, 1080))
    im = im.resize((M['W'], M['HF']), Image.LANCZOS)
    return to_yuv(np.asarray(im) / 255.0)


def display(mode, yuv):
    """native frame -> square-pixel display image"""
    M = fr.MODES[mode]
    im = Image.fromarray(to_rgb8(yuv))
    if M['W'] == 720:
        dw = round(M['W'] * 4096 / M['sy8'])          # line height is sy8/4096 h-px
        im = im.resize((dw, M['HF']), Image.LANCZOS)
    return im


BASE = dict(k1=0, k2=512, k3=0, k4=512, k5=900, k6=0, s7=0, s8=0, s9=0, s10=0, s11=0, p12=512)


def pat_ctl(name, **kw):
    if name in fr.CLASSIC:
        bank, idx, n = 0, fr.CLASSIC.index(name), len(fr.CLASSIC)
    else:
        bank, idx, n = 1, fr.CHEESE.index(name), len(fr.CHEESE)
    c = dict(BASE, s7=bank, k1=(idx * 1024 + 512) // n)
    c.update(kw)
    return c


SHEET = [
    ('MIX', dict(k4=512)),
    ('BAR', dict(k4=700, k2=600)),
    ('BOX', dict(k4=700)),
    ('CIRCLE', dict(k4=700)),
    ('STAR', dict(k4=700)),
    ('STAR', dict(k4=700, k3=114)),           # from the top-left corner
    ('STAR', dict(k4=700, s9=1)),             # SPIN
    ('STAR', dict(k4=200, k6=150)),           # soft edge, matte B
    ('CIRCLE', dict(k4=512, s8=1, k3=560)),   # reverse, hard, bottom-right
]


def label(im, text):
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 8 + 7 * len(text), 16), fill=(0, 0, 0))
    d.text((4, 2), text, fill=(255, 255, 255))
    return im


def sheet(outdir):
    os.makedirs(outdir, exist_ok=True)
    for mode in ('480i', '720p', '1080p'):
        A = source(mode)
        tiles = []
        for name, kw in SHEET:
            ctl = pat_ctl(name, **kw)
            out, P, clk = fr.render_emu(mode, ctl, A)
            im = display(mode, out)
            tag = name + ''.join(f' {k}={v}' for k, v in kw.items() if k != 'k4')
            tiles.append(label(im.resize((640, 360) if im.width > 700 else (480, 360), Image.LANCZOS), tag))
        tw, th = tiles[0].size
        S = Image.new('RGB', (3 * tw, 3 * th))
        for i, t in enumerate(tiles):
            S.paste(t, ((i % 3) * tw, (i // 3) * th))
        S.save(os.path.join(outdir, f'sheet_{mode}.png'))
        crops(mode, None, outdir)
        print('wrote', mode)


def crops(mode, out, outdir):
    """4x nearest-neighbour crops at native pixels: the small star's top tip
    and an inner vertex (gold border, then hard edge) and a 45-degree circle
    edge, labelled"""
    import math
    M = fr.MODES[mode]
    ly = M['sy8'] / 4096                       # line height in h-px
    tiles = []
    cw, ch = (40, 30) if M['W'] > 720 else (30, 22)
    for name, p12, k4, tag in (('STAR', 110, 700, 'border'), ('STAR', 110, 512, 'hard'),
                                ('CIRCLE', 300, 700, 'border'), ('CIRCLE', 300, 512, 'hard')):
        ctl = pat_ctl(name, p12=p12, k4=k4)
        img, P, _ = fr.render_emu(mode, ctl, source(mode))
        rgb = to_rgb8(img)
        G = math.hypot(P['DUX'], P['DVX']) / 1024     # key units per h-px
        cx, cy = M['W'] / 2, M['HF'] / 2 * ly
        if name == 'STAR':
            n = P['SN']; rho = fr.STAR_RHO[n]
            al = P['SAL'] / 2 ** 20 * 2 * math.pi
            R = P['T'] / G / math.cos(al)            # tip radius, h-px
            pts = [(cx, cy - R, 'tip'),
                   (cx + rho * R * math.sin(math.pi / n), cy - rho * R * math.cos(math.pi / n), 'vertex')]
        else:
            R = P['T'] / G
            pts = [(cx + R * 0.7071, cy - R * 0.7071, '45 deg')]
        for (hx, hy, what) in pts:
            x0 = min(max(0, int(hx - cw / 2)), M['W'] - cw); y0 = min(max(0, int(hy / ly - ch / 2)), M['HF'] - ch)
            c = rgb[y0:y0 + ch, x0:x0 + cw]
            t = Image.fromarray(np.ascontiguousarray(c)).resize((cw * 4, ch * 4), Image.NEAREST)
            tiles.append(label(t, f'{name.lower()} {what} {tag}'))
    S = Image.new('RGB', (sum(t.width for t in tiles) + 6 * (len(tiles) - 1), tiles[0].height), (40, 40, 40))
    x = 0
    for t in tiles:
        S.paste(t, (x, 0)); x += t.width + 6
    S.save(os.path.join(outdir, f'crop4x_{mode}.png'))


# ---------------------------------------------------------------- end checks
def ends():
    """bottom dead zone == A exactly, top == B exactly, no border survivors"""
    bad = 0
    n = 0
    for mode in ('480i', '720p', '1080p'):
        A = source(mode)
        for name in fr.CLASSIC + fr.CHEESE:
            for k3, k2, s9, k4, s8 in itertools.product(
                    [0, 57, 114, 228, 341, 455, 569, 683, 797, 910],    # centre + 8 anchors + end
                    [0, 512, 1023], [0, 1], [0, 512, 1023], [0, 1]):
                for p12, want in ((0, 'A'), (10, 'A'), (1013, 'B'), (1023, 'B')):
                    ctl = pat_ctl(name, k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12, k6=500)
                    out = fr.render(mode, ctl, A)
                    P = fr.frame_params(mode, ctl)
                    ref = A if want == 'A' else np.broadcast_to(np.array(P['BC']), A.shape)
                    n += 1
                    if not np.array_equal(out, ref):
                        bad += 1
                        if bad < 20:
                            print('FAIL', mode, name, dict(k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12),
                                  'pixels', int((out != ref).any(-1).sum()))
        print(mode, 'done')
    print(f'{n} end checks, {bad} failures')


if __name__ == '__main__':
    if sys.argv[1] == 'sheet':
        sheet(sys.argv[2])
    elif sys.argv[1] == 'ends':
        ends()
