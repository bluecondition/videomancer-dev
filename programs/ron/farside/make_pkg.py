#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FARSIDE - table generator.  Writes farside_pkg.vhd next to this file.

NOT a build hook (the builder only runs <program>.py): the tables come from
NASA's public-domain LRO maps (SVS "CGI Moon Kit": lroc_color_poles_4k.tif and
ldem_4.tif), which are too big to keep in the program folder.  Run by hand:

    python3 make_pkg.py [dir holding albedo.tif and ldem_4.tif]

Package contents (see farside.vhd for how they are read):

  C_FS_A00/A01/A10/A11   4 x (5120 x 4)   whole-Moon albedo, 256 columns x 80
        rows at 4 bits.  Columns are longitude (column j centre at
        (j + 0.5) / 256 of a turn, -180 deg at j = 0, wrapping); rows are
        sin(latitude), north at the top (row i centre at sin = 1 - (i + 0.5) / 40),
        which is the screen line for an upright globe.  Four sub-banks by (row
        parity, column parity): bank[rp][cp](r * 128 + c) = texel(2r + rp, 2c + cp).
        Displayed level = (208 + 32 * code) / 688 of the brightest highlands.
  C_FS_S00/S01/S10/S11   4 x (1024 x 4)   east-west surface slope, 128 columns x
        32 rows, two's complement -7 .. 7 = -0.10 .. +0.10 rise per run
        (positive = rising to the east), faded to zero at the poles.  Column c
        is centred on albedo column coordinate 2c (so the slope column
        coordinate is exactly half the albedo one); row r centre at
        sin = 1 - (r + 0.5) / 16.  bank[rp][cp](r * 64 + c) = slope(2r + rp, 2c + cp).
  C_FS_Z     1024 x 8    sqrt: z(i) = 255 * sqrt(1 - (i + 0.5) / 1024).
  C_FS_ASIN  512 x 11    asin((i + 0.5) / 512) in albedo columns x 32 (Q6.5):
        how far round the globe a point sits from the centre meridian.
"""
import os
import sys

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.expanduser("~/videomancer-dev/.moon_work/src")

RM = 1737.4          # lunar radius, km
SMAX = 0.10          # slope at code 7
SHARPEN = 0.6        # pre-emphasis against the bilinear blur
AW, AH = 256, 80     # albedo columns x rows
SW, SH = 128, 32     # slope columns x rows


def load_grey(path):
    a = np.asarray(Image.open(path)).astype(np.float64)
    if a.ndim == 3:
        a = a[..., :3] @ np.array([0.2126, 0.7152, 0.0722])
    return a


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


def blur3_wrap(t):
    p = np.pad(np.pad(t, ((1, 1), (0, 0)), mode="edge"), ((0, 0), (1, 1)), mode="wrap")
    return (p[:-2, 1:-1] + p[2:, 1:-1] + p[1:-1, :-2] + p[1:-1, 2:] + 4 * t) / 8


def build_tables(src=SRC):
    """Returns (albedo codes 80x256 0..15, slope codes 32x128 -7..7, z, asin)."""
    alb = load_grey(f"{src}/albedo.tif")
    hgt = load_grey(f"{src}/ldem_4.tif")
    band = alb[alb.shape[0] // 6: -alb.shape[0] // 6]
    alb = np.clip(alb / np.percentile(band, 99.7), 0, 1)

    # ---- albedo: rows uniform in sin(latitude), 8x oversampled box filter
    ss = 8
    sy = 1 - (np.arange(AH * ss) + 0.5) / (AH * ss) * 2
    lon = (np.arange(AW * ss) + 0.5) / (AW * ss) * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    t = sample_equirect(alb, np.arcsin(sy), lon).reshape(AH, ss, AW, ss).mean(axis=(1, 3))
    t = np.clip(t + SHARPEN * (t - blur3_wrap(t)), 0, 1)
    acode = np.clip(np.round((t * 688 - 208) / 32), 0, 15).astype(int)

    # ---- slope: column c centred on albedo column coordinate 2c
    sy = 1 - (np.arange(SH * ss) + 0.5) / (SH * ss) * 2
    k = (np.arange(SW * ss) + 0.5) / ss - 0.5            # slope-column coordinate
    lon = (2 * k + 0.5) / AW * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    lat = np.arcsin(sy)
    d = 13.0 / (RM * np.maximum(np.cos(lat), 0.15))
    sl = (sample_equirect(hgt, lat, lon + d) - sample_equirect(hgt, lat, lon - d)) / 26.0
    sl = sl * np.clip((np.cos(lat) - 0.3) / 0.3, 0, 1)   # fade out at the poles
    sl = sl.reshape(SH, ss, SW, ss).mean(axis=(1, 3))
    scode = np.round(np.clip(sl / SMAX, -1, 1) * 7).astype(int)

    z = np.round(255 * np.sqrt(1 - (np.arange(1024) + 0.5) / 1024)).astype(int)
    asin = np.round(np.arcsin((np.arange(512) + 0.5) / 512) * (AW / (2 * np.pi)) * 32).astype(int)
    return acode, scode, z, asin


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
    acode, scode, z, asin = build_tables(sys.argv[1] if len(sys.argv) > 1 else SRC)
    ab, sb = banks(acode), banks(scode)
    path = os.path.join(HERE, "farside_pkg.vhd")
    with open(path, "w") as f:
        f.write("-- Generated by make_pkg.py -- DO NOT EDIT.\n")
        f.write("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n")
        f.write("package farside_pkg is\n")
        f.write("    type t_fs_alb  is array (0 to 5119) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_fs_slp  is array (0 to 1023) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_fs_z    is array (0 to 1023) of std_logic_vector(7 downto 0);\n")
        f.write("    type t_fs_asin is array (0 to 511) of std_logic_vector(10 downto 0);\n")
        for (rp, cp), v in ab.items():
            emit(f, f"C_FS_A{rp}{cp}", "t_fs_alb", v, 4)
        for (rp, cp), v in sb.items():
            emit(f, f"C_FS_S{rp}{cp}", "t_fs_slp", v, 4)
        emit(f, "C_FS_Z", "t_fs_z", z, 8)
        emit(f, "C_FS_ASIN", "t_fs_asin", asin, 11)
        f.write("end package farside_pkg;\n")
    print(f"wrote {path}: albedo {acode.shape} codes {acode.min()}..{acode.max()}, "
          f"slope {scode.shape} codes {scode.min()}..{scode.max()}, asin max {asin.max()}")


if __name__ == "__main__":
    main()
