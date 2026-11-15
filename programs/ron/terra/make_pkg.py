#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TERRA - table generator.  Writes terra_pkg.vhd next to this file.

NOT a build hook (the builder only runs <program>.py): the surface comes from
NASA's public-domain Blue Marble (SVS a002915 bluemarble-2048.png), kept in
.moon_work/src_earth/.  The CLOUDS ARE PROCEDURAL (NASA's cloud map host was
unreachable on 2026-11-14); swap cloud_field() for a real map when one is to
hand.  Run by hand:

    python3 make_pkg.py [dir holding bm_svs.png]

Package contents (see terra.vhd for how they are read):

  C_TE_A00/A01/A10/A11   4 x (5120 x 4)   surface, 256 columns x 80 rows, each
        texel a 4-bit index into a 16-colour palette.  Columns are longitude
        (wrapping), rows are sin(latitude), north at the top; four sub-banks by
        (row parity, column parity): bank[rp][cp](r * 128 + c) = texel(2r + rp, 2c + cp).
  f_te_pal(i)   the palette, as logic: {Y (7) & U (6) & V (6)} per colour.
        Y is 0 .. 88 (x 8 = the drawn level above black, cap 704).  U / V are
        two's complement in units of 8 codes, in the HARDWARE convention
        (the u register carries Cr, v carries Cb).
  C_TE_C00/C01/C10/C11   4 x (1024 x 4)   cloud cover, 128 columns x 32 rows,
        0 .. 15.  Column c is centred on surface column coordinate 2c.
  C_TE_Z     1024 x 8    sqrt: z(i) = 255 * sqrt(1 - (i + 0.5) / 1024).
  C_TE_ASIN  512 x 11    asin((i + 0.5) / 512) in surface columns x 32 (Q6.5).
"""
import os
import sys

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.expanduser("~/videomancer-dev/.moon_work/src_earth")

AW, AH = 256, 80     # surface columns x rows
SW, SH = 128, 32     # cloud columns x rows


def load_rgb(path):
    return np.asarray(Image.open(path).convert("RGB")).astype(np.float64) / 255.0


def sample_equirect(m, lat, lon):
    h, w = m.shape
    fx = (lon / (2 * np.pi) + 0.5) * w - 0.5
    fy = (0.5 - lat / np.pi) * h - 0.5
    x0 = np.floor(fx).astype(int)
    y0 = np.floor(fy).astype(int)
    ax, ay = fx - x0, fy - y0
    x1 = (x0 + 1) % w
    x0 %= w
    y1 = np.clip(y0 + 1, 0, h - 1)
    y0 = np.clip(y0, 0, h - 1)
    return ((m[y0, x0] * (1 - ax) + m[y0, x1] * ax) * (1 - ay)
            + (m[y1, x0] * (1 - ax) + m[y1, x1] * ax) * ay)


def sample_rgb(m, lat, lon):
    return np.stack([sample_equirect(m[..., c], lat, lon) for c in range(3)], axis=-1)


def fbm_sphere(lat, lon, seed, octaves=5, f0=1.6):
    """Seamless noise on the sphere: a sum of random plane waves in 3-D."""
    rng = np.random.default_rng(seed)
    p = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=-1)
    out = np.zeros(lat.shape)
    for o in range(octaves):
        f, n = f0 * 2 ** o, 24
        k = rng.normal(size=(n, 3))
        k = k / np.linalg.norm(k, axis=1, keepdims=True) * f * rng.uniform(0.7, 1.3, size=(n, 1))
        ph = rng.uniform(0, 2 * np.pi, n)
        out += np.cos(p @ k.T + ph).sum(axis=-1) / np.sqrt(n) * 0.6 ** o
    return out


def cloud_field(lat, lon):
    """Procedural cloud cover 0 .. 1: weather-scale noise, an equatorial band
    and two mid-latitude storm belts, clear subtropics."""
    n = fbm_sphere(lat, lon, 7, 5, 2.6) * 0.50 + fbm_sphere(lat, lon, 11, 3, 9.0) * 0.22
    band = (0.30 * np.exp(-(lat / 0.12) ** 2)
            + 0.42 * np.exp(-((np.abs(lat) - 0.92) / 0.28) ** 2)
            - 0.10 * np.exp(-((np.abs(lat) - 0.42) / 0.16) ** 2))
    return np.clip((n * 0.60 + band - 0.20) * 2.1, 0, 1)


def kmeans(px, k, iters=30, seed=3):
    rng = np.random.default_rng(seed)
    c = px[rng.choice(len(px), k, replace=False)]
    for _ in range(iters):
        lab = ((px[:, None, :] - c[None, :, :]) ** 2).sum(axis=2).argmin(axis=1)
        for j in range(k):
            if (lab == j).any():
                c[j] = px[lab == j].mean(axis=0)
    return c, lab


def build_tables(src=SRC):
    """Returns (index map 80x256, palette [(Y8, U6, V6)], palette RGB, cloud 32x128, z, asin)."""
    bm = load_rgb(f"{src}/bm_svs.png")
    bm = np.clip(bm ** 0.85 * 1.10, 0, 1)                    # lift the dark map a little
    ocean = (bm.max(axis=2) < 0.33) & (bm[..., 2] > bm[..., 0])
    bm[ocean] = bm[ocean] * 0.6 + np.array([0.02, 0.10, 0.30]) * 0.9   # bluer seas

    ss = 8
    sy = 1 - (np.arange(AH * ss) + 0.5) / (AH * ss) * 2
    lon = (np.arange(AW * ss) + 0.5) / (AW * ss) * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    fine = sample_rgb(bm, np.arcsin(sy), lon).reshape(AH, ss, AW, ss, 3).mean(axis=(1, 3))
    rgb, lab = kmeans(fine.reshape(-1, 3), 16)
    order = np.argsort(rgb @ np.array([0.299, 0.587, 0.114]))
    rank = np.empty(16, dtype=int)
    rank[order] = np.arange(16)
    rgb, idx = rgb[order], rank[lab].reshape(AH, AW)
    luma = rgb @ np.array([0.299, 0.587, 0.114])
    pal = []
    for c, yv in zip(rgb, luma):                             # hardware convention: U = Cr, V = Cb
        cr, cb = 0.713 * (c[0] - yv) * 896 / 8, 0.564 * (c[2] - yv) * 896 / 8
        pal.append((int(round(yv * 88)), int(np.clip(round(cr), -32, 31)), int(np.clip(round(cb), -32, 31))))

    sy = 1 - (np.arange(SH * ss) + 0.5) / (SH * ss) * 2
    k = (np.arange(SW * ss) + 0.5) / ss - 0.5                # cloud-column coordinate
    lon = (2 * k + 0.5) / AW * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    cl = cloud_field(np.arcsin(sy), lon).reshape(SH, ss, SW, ss).mean(axis=(1, 3))
    ccode = np.round(cl * 15).astype(int)

    z = np.round(255 * np.sqrt(1 - (np.arange(1024) + 0.5) / 1024)).astype(int)
    asin = np.round(np.arcsin((np.arange(512) + 0.5) / 512) * (AW / (2 * np.pi)) * 32).astype(int)
    return idx, pal, rgb, ccode, z, asin


def banks(tex):
    return {(rp, cp): tex[rp::2, cp::2].reshape(-1) for rp in (0, 1) for cp in (0, 1)}


def emit(f, name, tname, vals, bits):
    digits = (bits + 3) // 4
    mask = (1 << bits) - 1
    f.write(f"    constant {name} : {tname} := (\n")
    vals = [int(v) & mask for v in vals]
    for i in range(0, len(vals), 32):
        if bits % 4:
            row = ", ".join(f'"{v:0{bits}b}"' for v in vals[i:i + 32])
        else:
            row = ", ".join(f'x"{v:0{digits}X}"' for v in vals[i:i + 32])
        f.write("        " + row + ("" if i + 32 >= len(vals) else ",") + "\n")
    f.write("    );\n")


def main():
    idx, pal, rgb, ccode, z, asin = build_tables(sys.argv[1] if len(sys.argv) > 1 else SRC)
    ab, cb = banks(idx), banks(ccode)
    path = os.path.join(HERE, "terra_pkg.vhd")
    with open(path, "w") as f:
        f.write("-- Generated by make_pkg.py -- DO NOT EDIT.\n")
        f.write("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n")
        f.write("package terra_pkg is\n")
        f.write("    type t_te_map  is array (0 to 5119) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_te_cld  is array (0 to 1023) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_te_z    is array (0 to 1023) of std_logic_vector(7 downto 0);\n")
        f.write("    type t_te_asin is array (0 to 511) of std_logic_vector(10 downto 0);\n")
        f.write("    function f_te_pal(i : unsigned(3 downto 0)) return std_logic_vector;\n")
        for (rp, cp), v in ab.items():
            emit(f, f"C_TE_A{rp}{cp}", "t_te_map", v, 4)
        for (rp, cp), v in cb.items():
            emit(f, f"C_TE_C{rp}{cp}", "t_te_cld", v, 4)
        emit(f, "C_TE_Z", "t_te_z", z, 8)
        emit(f, "C_TE_ASIN", "t_te_asin", asin, 11)
        f.write("end package terra_pkg;\n\n")
        f.write("package body terra_pkg is\n")
        f.write("    -- {Y (7) & U = Cr / 8 (6) & V = Cb / 8 (6)}\n")
        f.write("    function f_te_pal(i : unsigned(3 downto 0)) return std_logic_vector is\n    begin\n")
        f.write("        case to_integer(i) is\n")
        for n, (yv, u, v) in enumerate(pal):
            word = f"{yv & 127:07b}{u & 63:06b}{v & 63:06b}"
            sel = f"when {n}     " if n < 15 else "when others"
            c = rgb[n]
            f.write(f'            {sel} => return "{word}";   -- rgb {c[0]:.2f} {c[1]:.2f} {c[2]:.2f}\n')
        f.write("        end case;\n    end function;\nend package body terra_pkg;\n")
    print(f"wrote {path}: surface {idx.shape}, clouds {ccode.shape} codes {ccode.min()}..{ccode.max()}")
    for p_, c in zip(pal, rgb):
        print("  pal", p_, np.round(c, 2))


if __name__ == "__main__":
    main()
