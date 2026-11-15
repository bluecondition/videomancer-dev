#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
HELIOS - table generator.  Writes helios_pkg.vhd next to this file.

Everything here is PROCEDURAL (seeded, so the tables are reproducible): the
Sun has no fixed surface, and no real solar map was to hand.  A real SDO
synoptic map could replace surface_map().  Run by hand:

    python3 make_pkg.py

Package contents (see helios.vhd for how they are read):

  C_HE_A00/A01/A10/A11   4 x (4096 x 4)   surface brightness, 256 columns x 64
        rows at 4 bits: bright network, eight active regions (plage) with
        sunspots.  Columns are longitude (wrapping), rows are sin(latitude);
        four sub-banks by (row parity, column parity):
        bank[rp][cp](r * 128 + c) = texel(2r + rp, 2c + cp).
  C_HE_N00/N01/N10/N11   4 x (1024 x 4)   one tiling noise map, 128 columns x
        32 rows at 4 bits.  The program reads it TWICE (two copies in block
        RAM), the second copy mirrored, each drifting its own way, and
        multiplies the two: that product is the boil.
  C_HE_Z     1024 x 8    sqrt: z(i) = 255 * sqrt(1 - (i + 0.5) / 1024).
  C_HE_ASIN  512 x 11    asin((i + 0.5) / 512) in surface columns x 32 (Q6.5).
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
AW, AH = 256, 64
NW, NH = 128, 32


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


def surface_map(ss=4, seed=5):
    rng = np.random.default_rng(seed)
    sy = 1 - (np.arange(AH * ss) + 0.5) / (AH * ss) * 2
    lon = (np.arange(AW * ss) + 0.5) / (AW * ss) * 2 * np.pi - np.pi
    lon, sy = np.meshgrid(lon, sy)
    lat = np.arcsin(sy)
    net = 1 - np.abs(fbm_sphere(lat, lon, 21, 4, 9.0)) / 2.2            # bright network
    base = 0.50 + 0.22 * net + 0.07 * fbm_sphere(lat, lon, 22, 3, 3.0)
    p = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], axis=-1)
    for _ in range(8):                                                  # active regions
        la = np.radians(rng.uniform(8, 28)) * rng.choice([-1, 1])
        lo = rng.uniform(-np.pi, np.pi)
        c = np.array([np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)])
        d = np.arccos(np.clip(p @ c, -1, 1))
        base += 0.42 * np.exp(-(d / 0.11) ** 2)
        for _ in range(rng.integers(1, 4)):                             # sunspots
            la2, lo2 = la + rng.normal(0, 0.035), lo + rng.normal(0, 0.07)
            c2 = np.array([np.cos(la2) * np.cos(lo2), np.cos(la2) * np.sin(lo2), np.sin(la2)])
            d2 = np.arccos(np.clip(p @ c2, -1, 1))
            sz = rng.uniform(0.018, 0.034)
            base *= 1 - 0.55 * np.exp(-(d2 / (sz * 1.9)) ** 2)
            base *= 1 - 0.85 * np.exp(-(d2 / sz) ** 2)
    base = np.clip(base.reshape(AH, ss, AW, ss).mean(axis=(1, 3)), 0, 1)
    return np.round(base * 15).astype(int)


def noise_map(seed=9, beta=1.6):
    """Tiling noise, made square (128 x 128) and box-filtered to 128 x 32."""
    rng = np.random.default_rng(seed)
    n = NW
    f = np.fft.fftfreq(n)
    k = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)
    k[0, 0] = 1
    spec = rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n))
    a = np.real(np.fft.ifft2(spec / k ** beta * (k > 3.0 / n)))
    a = a.reshape(NH, n // NH, n).mean(axis=1)
    lo, hi = np.percentile(a, 1), np.percentile(a, 99)
    return np.round(np.clip((a - lo) / (hi - lo), 0, 1) * 15).astype(int)


def build_tables():
    z = np.round(255 * np.sqrt(1 - (np.arange(1024) + 0.5) / 1024)).astype(int)
    asin = np.round(np.arcsin((np.arange(512) + 0.5) / 512) * (AW / (2 * np.pi)) * 32).astype(int)
    return surface_map(), noise_map(), z, asin


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
    smap, nmap, z, asin = build_tables()
    path = os.path.join(HERE, "helios_pkg.vhd")
    with open(path, "w") as f:
        f.write("-- Generated by make_pkg.py -- DO NOT EDIT.\n")
        f.write("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\n")
        f.write("package helios_pkg is\n")
        f.write("    type t_he_map  is array (0 to 4095) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_he_nse  is array (0 to 1023) of std_logic_vector(3 downto 0);\n")
        f.write("    type t_he_z    is array (0 to 1023) of std_logic_vector(7 downto 0);\n")
        f.write("    type t_he_asin is array (0 to 511) of std_logic_vector(10 downto 0);\n")
        for (rp, cp), v in banks(smap).items():
            emit(f, f"C_HE_A{rp}{cp}", "t_he_map", v, 4)
        for (rp, cp), v in banks(nmap).items():
            emit(f, f"C_HE_N{rp}{cp}", "t_he_nse", v, 4)
        emit(f, "C_HE_Z", "t_he_z", z, 8)
        emit(f, "C_HE_ASIN", "t_he_asin", asin, 11)
        f.write("end package helios_pkg;\n")
    print(f"wrote {path}: surface {smap.shape} codes {smap.min()}..{smap.max()} mean {smap.mean():.1f}, "
          f"noise {nmap.shape} mean {nmap.mean():.1f}")


if __name__ == "__main__":
    main()
