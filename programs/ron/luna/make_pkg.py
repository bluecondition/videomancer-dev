#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LUNA - table generator.  Writes luna_pkg.vhd next to this file.

NOT a build hook (the builder only runs <program>.py): the tables come from
NASA's public-domain LRO maps (SVS "CGI Moon Kit": lroc_color_poles_4k.tif and
ldem_4.tif), which are too big to keep in the program folder.  Run by hand:

    python3 make_pkg.py [dir holding albedo.tif and ldem_4.tif]

Package contents (see luna.vhd for how they are read):

  C_LU_A00/A01/A10/A11   4 x (4096 x 4)   near-side albedo, 128 x 128 texels at 4
        bits, orthographic (what you see from Earth, north up).  Texel (row i,
        col j) has its centre at xn = (j + 0.5) / 64 - 1, yn = (i + 0.5) / 64 - 1
        (yn down).  Split into four sub-banks by (row parity, col parity) so one
        read per bank gives the four bilinear taps:
        bank[rp][cp](r * 64 + c) = texel(2r + rp, 2c + cp).
        Displayed level = (208 + 32 * code) / 688 of the brightest highlands.
  C_LU_S00/S01/S10/S11   4 x (2048 x 4)   east-west surface slope, 128 columns x
        64 rows, two's complement -7 .. 7 = -0.10 .. +0.10 rise per run (positive
        = rising to the east).  Columns sit on the albedo columns; row j is
        centred at yn = (2j + 0.5) / 64 - 1, so the slope row coordinate is
        exactly half the albedo row coordinate.  Zero near the limb.
        bank[rp][cp](r * 64 + c) = slope(2r + rp, 2c + cp).
  C_LU_Z   1024 x 8   sqrt: z(i) = 255 * sqrt(1 - (i + 0.5) / 1024).
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


def near_side(m, x, y):
    """Sample map m at unit-disc position (x east, y north); the limb is held outside."""
    r = np.sqrt(x * x + y * y)
    k = np.where(r > 0.9995, 0.9995 / np.maximum(r, 1e-9), 1.0)
    x, y = x * k, y * k
    z = np.sqrt(np.clip(1 - x * x - y * y, 0, 1))
    return sample_equirect(m, np.arcsin(y), np.arctan2(x, z))


def blur3(t):
    p = np.pad(t, 1, mode="edge")
    return (p[:-2, 1:-1] + p[2:, 1:-1] + p[1:-1, :-2] + p[1:-1, 2:] + 4 * t) / 8


def build_tables(src=SRC):
    """Returns (albedo codes 128x128 0..15, slope codes 64x128 -7..7, z table)."""
    alb = load_grey(f"{src}/albedo.tif")
    hgt = load_grey(f"{src}/ldem_4.tif")

    # ---- albedo: 128 x 128, box-filtered from 8x oversampling
    n, ss = 128, 8
    c = (np.arange(n * ss) + 0.5) / (n * ss) * 2 - 1
    x, y = np.meshgrid(c, -c)
    fine = near_side(alb, x, y)
    hi = np.percentile(fine[x * x + y * y < 0.98], 99.7)
    t = np.clip(fine / hi, 0, 1).reshape(n, ss, n, ss).mean(axis=(1, 3))
    t = np.clip(t + SHARPEN * (t - blur3(t)), 0, 1)
    acode = np.clip(np.round((t * 688 - 208) / 32), 0, 15).astype(int)

    # ---- slope: fine 1024 x 1024 field, then 128 columns x 64 offset rows
    nf = 1024
    c = (np.arange(nf) + 0.5) / nf * 2 - 1
    x, y = np.meshgrid(c, -c)
    h = near_side(hgt, x, y)
    px_km = 2 * RM / nf
    b = max(1, int(round(13.0 / px_km)))
    sl = (np.roll(h, -b, axis=1) - np.roll(h, b, axis=1)) / (2 * b * px_km)
    sl[x * x + y * y > 0.94] = 0.0
    cols = sl.reshape(nf, 128, 8).mean(axis=2)             # 1024 rows x 128 cols
    scode = np.zeros((64, 128), dtype=int)
    for j in range(64):
        r0, r1 = max(0, 16 * j - 4), min(nf, 16 * j + 12)
        v = cols[r0:r1].mean(axis=0)
        scode[j] = np.round(np.clip(v / SMAX, -1, 1) * 7).astype(int)

    z = np.round(255 * np.sqrt(1 - (np.arange(1024) + 0.5) / 1024)).astype(int)
    return acode, scode, z


def banks(tex):
    """Split rows/cols by parity: [rp][cp] -> flat list, address r * 64 + c."""
    out = {}
    for rp in (0, 1):
        for cp in (0, 1):
            out[(rp, cp)] = tex[rp::2, cp::2].reshape(-1)
    return out


def emit(f, name, tname, vals, bits):
    digits = (bits + 3) // 4
    mask = (1 << bits) - 1
    f.write(f"    constant {name} : {tname} := (\n")
    vals = [int(v) & mask for v in vals]
    for i in range(0, len(vals), 32):
        row = ", ".join(f'x"{v:0{digits}X}"' for v in vals[i:i + 32])
        f.write("        " + row + ("" if i + 32 >= len(vals) else ",") + "\n")
    f.write("    );\n")


def main():
    acode, scode, z = build_tables(sys.argv[1] if len(sys.argv) > 1 else SRC)
    ab, sb = banks(acode), banks(scode)
    path = os.path.join(HERE, "luna_pkg.vhd")
    with open(path, "w") as f:
        f.write("-- Generated by make_pkg.py -- DO NOT EDIT.\n")
        f.write("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n")
        f.write("package luna_pkg is\n")
        f.write("    type t_lu_alb is array (0 to 4095) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_lu_slp is array (0 to 2047) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_lu_z   is array (0 to 1023) of std_logic_vector(7 downto 0);\n")
        for (rp, cp), v in ab.items():
            emit(f, f"C_LU_A{rp}{cp}", "t_lu_alb", v, 4)
        for (rp, cp), v in sb.items():
            emit(f, f"C_LU_S{rp}{cp}", "t_lu_slp", v, 4)
        emit(f, "C_LU_Z", "t_lu_z", z, 8)
        f.write("end package luna_pkg;\n")
    print(f"wrote {path}: albedo {acode.shape} codes {acode.min()}..{acode.max()}, "
          f"slope {scode.shape} codes {scode.min()}..{scode.max()}")


if __name__ == "__main__":
    main()
