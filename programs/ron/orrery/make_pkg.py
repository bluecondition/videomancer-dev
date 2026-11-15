#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ORRERY - table generator.  Writes orrery_pkg.vhd next to this file.

NOT a build hook.  Earth's map is NASA's public-domain Blue Marble (SVS
a002915 bluemarble-2048.png, kept in .moon_work/src_earth/); every other
surface is PROCEDURAL (seeded).  Run by hand:

    python3 make_pkg.py [dir holding bm_svs.png]

Package contents (see orrery.vhd):

  C_OR_SIN   1024 x 15   16383 * sin(2 pi i / 1024), two's complement.
  C_OR_TEX   16384 x 4   eight surface pictures (the planets; the Sun is plain),
        64 columns x 32 rows each, a 4-bit index into that body's own palette.
        Address = (body - 1) * 2048 + row * 64 + column.  Columns go round the planet (the visible face is
        32 columns wide, so the picture scrolls through twice its width per
        turn); rows are top to bottom of the disc.
        Bodies: 1 Mercury, 2 Venus, 3 Earth, 4 Mars, 5 Jupiter, 6 Saturn,
        7 Uranus, 8 Neptune.
  C_OR_PAL   256 x 22    palettes, address = body * 16 + index (body 0 = the Sun,
        index 0; body 9 = Saturn's ring, index = position across the ring).  {Y (8, 0 .. 175) & Cr / 8 (7) &
        Cb / 8 (7)}, hardware convention (the u register carries Cr).
"""
import math
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.expanduser("~/videomancer-dev/.moon_work/src_earth")
RK = [0, 4, 5, 5, 4, 6, 6, 5, 5]                         # planet radius = 2^RK half units (Small)
R_SUN = 96                                               # the Sun's radius, half units (Small)
TW, TH = 64, 32


def fbm(lat, lon, seed, octaves=4, f0=2.0):
    rng = np.random.default_rng(seed)
    p = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=-1)
    out = np.zeros(lat.shape)
    for o in range(octaves):
        f, n = f0 * 2 ** o, 20
        k = rng.normal(size=(n, 3))
        k = k / np.linalg.norm(k, axis=1, keepdims=True) * f * rng.uniform(0.7, 1.3, size=(n, 1))
        out += np.cos(p @ k.T + rng.uniform(0, 2 * np.pi, n)).sum(axis=-1) / np.sqrt(n) * 0.6 ** o
    return out


def grid(ss=4):
    sy = 1 - (np.arange(TH * ss) + 0.5) / (TH * ss) * 2
    lon = (np.arange(TW * ss) + 0.5) / (TW * ss) * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    return np.arcsin(sy), lon


def box(a, ss=4):
    return a.reshape(TH, ss, TW, ss, 3).mean(axis=(1, 3))


def col(c):
    return np.array(c)[None, None, :]


def spot(lat, lon, la0, lo0, wla, wlo):
    dl = (lon - lo0 + np.pi) % (2 * np.pi) - np.pi
    return np.exp(-((lat - la0) / wla) ** 2 - (dl / wlo) ** 2)[..., None]


def earth(src):
    path = f"{src}/bm_svs.png"
    lat, lon = grid()
    if not os.path.exists(path):
        land = (fbm(lat, lon, 31, 4, 1.6) > 0.35)[..., None]
        return box(np.where(land, col((0.25, 0.42, 0.18)), col((0.05, 0.14, 0.42))))
    bm = np.asarray(Image.open(path).convert("RGB")).astype(np.float64) / 255.0
    bm = np.clip(bm ** 0.85 * 1.10, 0, 1)
    ocean = (bm.max(axis=2) < 0.33) & (bm[..., 2] > bm[..., 0])
    bm[ocean] = bm[ocean] * 0.6 + np.array([0.02, 0.10, 0.30]) * 0.9
    h, w = bm.shape[:2]
    yy = np.clip(((0.5 - lat / np.pi) * h).astype(int), 0, h - 1)
    xx = ((lon / (2 * np.pi) + 0.5) * w).astype(int) % w
    return box(bm[yy, xx])


def surfaces(src):
    lat, lon = grid()
    s = np.sin(lat)[..., None]
    t = {}
    g = fbm(lat, lon, 1, 4, 10.0)[..., None]
    sun = col((1.0, 0.93, 0.62)) * (0.90 + 0.06 * g)
    for la0, lo0 in ((0.3, -1.2), (-0.25, 0.6), (0.2, 2.3), (-0.35, -2.6)):
        sun = sun * (1 - 0.75 * spot(lat, lon, la0, lo0, 0.07, 0.10))
    t[0] = sun
    t[1] = col((0.60, 0.58, 0.56)) * (1 + 0.16 * fbm(lat, lon, 2, 4, 6.0)[..., None])
    t[2] = col((0.93, 0.84, 0.58)) * (1 + 0.07 * fbm(lat * 2.5, lon - 2.0 * np.abs(lat), 3, 3, 2.0)[..., None])
    t[3] = None
    dark = np.clip(fbm(lat, lon, 4, 4, 2.2)[..., None] * 0.9 + 0.15, 0, 1)
    mars = col((0.82, 0.40, 0.22)) * (1 - 0.45 * dark)
    t[4] = np.where(np.abs(s) > 0.88, col((0.95, 0.93, 0.90)), mars)
    wob = 0.05 * fbm(lat, lon, 5, 3, 5.0)
    belts = (0.5 + 0.5 * np.sin((lat + wob) * 11.0))[..., None]
    jup = col((0.93, 0.87, 0.74)) * belts + col((0.70, 0.50, 0.36)) * (1 - belts)
    jup = jup * (1 - 0.25 * (np.abs(s) > 0.8)) + 0.03 * fbm(lat, lon, 6, 3, 12.0)[..., None]
    rs = np.clip(spot(lat, lon, -0.38, 0.0, 0.16, 0.42) * 1.6, 0, 1)
    t[5] = jup * (1 - rs) + col((0.80, 0.24, 0.14)) * rs
    bands = (0.5 + 0.5 * np.sin(lat * 9.0))[..., None]
    t[6] = col((0.93, 0.85, 0.62)) * bands + col((0.82, 0.72, 0.50)) * (1 - bands)
    t[7] = col((0.62, 0.88, 0.92)) * (1 + 0.04 * np.sin(lat * 6.0)[..., None])
    nep = col((0.25, 0.40, 0.95)) * (1 + 0.08 * np.sin(lat * 7.0)[..., None])
    nep = nep * (1 - 0.5 * spot(lat, lon, -0.35, 0.8, 0.12, 0.22))
    t[8] = nep + 0.5 * spot(lat, lon, -0.15, 1.3, 0.03, 0.25) + 0.4 * spot(lat, lon, 0.45, -1.5, 0.03, 0.3)
    out = {i: np.clip(box(np.broadcast_to(v, lat.shape + (3,)) + 0 * s), 0, 1) for i, v in t.items() if v is not None}
    out[3] = earth(src)
    return out


def kmeans(px, k, iters=30, seed=3):
    rng = np.random.default_rng(seed)
    c = px[rng.choice(len(px), k, replace=False)].copy()
    for _ in range(iters):
        lab = ((px[:, None, :] - c[None, :, :]) ** 2).sum(axis=2).argmin(axis=1)
        for j in range(k):
            if (lab == j).any():
                c[j] = px[lab == j].mean(axis=0)
    return c, lab


def pal_word(c):
    y = 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]
    cr = int(np.clip(round(0.713 * (c[0] - y) * 896 / 8), -64, 63))
    cb = int(np.clip(round(0.564 * (c[2] - y) * 896 / 8), -64, 63))
    return int(round(min(y, 1.0) * 175)), cr, cb


def build_tables(src=SRC):
    """Returns (tex[9][32][64] indices, pal[160] of (Y, Cr, Cb), palette RGB[160], tn[9][3])."""
    surf = surfaces(src)
    tex = np.zeros((9, TH, TW), dtype=int)
    rgb = np.zeros((160, 3))
    for i in range(9):
        c, lab = kmeans(surf[i].reshape(-1, 3), 16)
        tex[i] = lab.reshape(TH, TW)
        rgb[i * 16:i * 16 + 16] = c
    ring = [0.55, 0.75, 0.85, 0.90, 0.80, 0.95, 0.90, 0.85, 0.12, 0.20, 0.80, 0.75, 0.70, 0.60, 0.45, 0.30]
    for j, b in enumerate(ring):
        rgb[144 + j] = np.array([0.85, 0.76, 0.58]) * b
    rgb[0:16] = np.array([1.0, 0.93, 0.62])                    # the Sun: one plain colour
    pal = [pal_word(c) for c in rgb]
    return tex, pal, rgb


def main():
    tex, pal, rgb = build_tables(sys.argv[1] if len(sys.argv) > 1 else SRC)
    sin = [int(round(16383 * math.sin(2 * math.pi * i / 1024))) for i in range(1024)]
    with open(os.path.join(HERE, "orrery_pkg.vhd"), "w") as f:
        f.write("-- Generated by make_pkg.py -- DO NOT EDIT.\n")
        f.write("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\npackage orrery_pkg is\n")
        f.write("    type t_or_sin is array (0 to 1023) of std_logic_vector(14 downto 0);\n")
        f.write("    type t_or_tex is array (0 to 16383) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_or_pal is array (0 to 255) of std_logic_vector(21 downto 0);\n")
        f.write("    constant C_OR_SIN : t_or_sin := (\n")
        for i in range(0, 1024, 16):
            f.write("        " + ", ".join('"{:015b}"'.format(v & 0x7FFF) for v in sin[i:i + 16]) + ("" if i + 16 >= 1024 else ",") + "\n")
        f.write("    );\n    constant C_OR_TEX : t_or_tex := (\n")
        flat = tex[1:].reshape(-1)
        for i in range(0, len(flat), 64):
            f.write("        " + ", ".join(f'x"{v:X}"' for v in flat[i:i + 64]) + ("" if i + 64 >= len(flat) else ",") + "\n")
        f.write("    );\n    constant C_OR_PAL : t_or_pal := (\n")
        words = ['"{:08b}{:07b}{:07b}"'.format(y, cr & 127, cb & 127) for y, cr, cb in pal] + ['"' + "0" * 22 + '"'] * 96
        for i in range(0, 256, 8):
            f.write("        " + ", ".join(words[i:i + 8]) + ("" if i + 8 >= 256 else ",") + "\n")
        f.write("    );\nend package orrery_pkg;\n")
    print("wrote orrery_pkg.vhd")
    sheet = Image.new("RGB", (TW * 4 * 3, TH * 4 * 3))
    for i in range(9):
        im = Image.fromarray((rgb[i * 16 + tex[i]] * 255).astype(np.uint8)).resize((TW * 4, TH * 4), Image.NEAREST)
        sheet.paste(im, ((i % 3) * TW * 4, (i // 3) * TH * 4))
    sheet.save(os.path.expanduser("~/videomancer-dev/.moon_work/model_out/_orrery_textures.png"))


if __name__ == "__main__":
    main()
