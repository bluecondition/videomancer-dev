#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE render helpers: test image, YUV conversions, per-mode source / display, labels."""
import sys, os, itertools
import numpy as np
from PIL import Image, ImageDraw
import sw_frame2 as fr

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




def label(im, text):
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 8 + 7 * len(text), 16), fill=(0, 0, 0))
    d.text((4, 2), text, fill=(255, 255, 255))
    return im
