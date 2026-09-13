#!/usr/bin/env python3
"""Generate warpfield_rom_pkg.vhd: glow-kernel ROMs, shimmer palette,
nebula texture and the zoom exp2 table.

Kernel ROM layout (per layer, 1024 x 8-bit):
    address = {glow_level(2b), shape(2b), |dy|(3b), |dx|(3b)}

Shapes (same family as starfield):
    0 = gaussian round glow
    1 = soft-edged square (superellipse)
    2 = diffraction spike (gauss core + axis arms)
    3 = point + faint halo

Unlike starfield, all 8 layer ROMs are IDENTICAL: in warpfield a layer's
depth is its zoom phase, which rotates over time, so no per-layer fade can
be baked. Instead a brightness ramp is baked per glow LEVEL (far stars are
small levels = dimmer, near stars big levels = full), which is what a
depth fade means when depth drives the level. Eight separately-named
constants are still required: GHDL synthesis crashes merging 8 parallel
reads of one array-of-arrays constant.

EXP2 table: 256 x 13-bit, exp2n[p] = round(4096 * 2^(-p/256)), i.e. the
per-pixel coordinate step k in 4.12 fixed point for zoom phase p. k spans
(2048, 4096]: screen cell size 16..32 px across one octave.

Usage: python3 gen_kernels.py   (writes warpfield_rom_pkg.vhd next to itself)
"""

import math
import os

NUM_LAYERS = 6
LEVEL_GAIN = [150, 190, 225, 255]          # /255 per glow level (depth ramp)

GAUSS_SIGMA = [0.70, 1.15, 1.80, 2.80]     # shape 0, per glow level
SQUARE_R = [0.90, 1.40, 2.10, 2.90]        # shape 1 superellipse radius
SPIKE_CORE = [0.70, 0.90, 1.15, 1.45]      # shape 2 core sigma
SPIKE_LAMBDA = [1.00, 1.50, 2.30, 3.20]    # shape 2 arm decay
SPIKE_AMP = 0.75                           # arm peak vs core peak
SPIKE_WIDTH = 0.40                         # arm perpendicular sigma
HALO_SIGMA = [0.55, 0.75, 0.95, 1.20]      # shape 3 halo sigma
HALO_AMP = 0.30

WINDOW_START = 4.5   # radial window: 1.0 inside, cos-roll to 0
WINDOW_END = 7.2

# Shimmer palette ROM: {shim_phase(4b), palette(2b), entry(2b)} -> U,V
# (hardware U/V convention, same values as starfield).
PALETTES = [
    # Mixed
    [(512, 512), (300, 760), (622, 347), (597, 647)],
    # Cool
    [(512, 512), (280, 800), (620, 720), (480, 560)],
    # Warm
    [(512, 512), (720, 340), (640, 400), (560, 460)],
    # Rainbow
    [(512, 512), (800, 350), (180, 660), (780, 760)],
]
SHIM_STEPS = 16
SHIM_AMP_DEG = [12.0, 12.0, 12.0, 25.0]

# Nebula texture: 64x64 tileable 3-octave value noise (same as starfield).
NEB_SIZE = 64
NEB_SEED = 0x5EED
NEB_MAX = 200


def window(r):
    if r <= WINDOW_START:
        return 1.0
    if r >= WINDOW_END:
        return 0.0
    t = (r - WINDOW_START) / (WINDOW_END - WINDOW_START)
    return 0.5 * (1.0 + math.cos(math.pi * t))


def shape_value(shape, level, dx, dy):
    r2 = dx * dx + dy * dy
    r = math.sqrt(r2)
    if shape == 0:  # gaussian
        s = GAUSS_SIGMA[level]
        w = math.exp(-r2 / (2.0 * s * s))
    elif shape == 1:  # soft superellipse square
        r4 = (dx ** 4 + dy ** 4) ** 0.25
        R = SQUARE_R[level]
        w = math.exp(-((r4 / R) ** 4))
    elif shape == 2:  # diffraction spike
        s = SPIKE_CORE[level]
        core = math.exp(-r2 / (2.0 * s * s))
        lam = SPIKE_LAMBDA[level]
        pw2 = 2.0 * SPIKE_WIDTH * SPIKE_WIDTH
        arm_x = SPIKE_AMP * math.exp(-dx / lam) * math.exp(-(dy * dy) / pw2)
        arm_y = SPIKE_AMP * math.exp(-dy / lam) * math.exp(-(dx * dx) / pw2)
        w = max(core, arm_x, arm_y)
    else:  # point + faint halo
        s = HALO_SIGMA[level]
        halo = HALO_AMP * math.exp(-r2 / (2.0 * s * s))
        w = 1.0 if (dx == 0 and dy == 0) else halo
    return w * window(r)


def build_kernel():
    rom = []
    for level in range(4):
        gain = LEVEL_GAIN[level]
        for shape in range(4):
            for ady in range(8):
                for adx in range(8):
                    w = shape_value(shape, level, adx, ady)
                    q = int(round(w * gain))
                    q = max(0, min(255, q))
                    if q < 2:
                        q = 0
                    rom.append(q)
    return rom


def build_shimpal():
    rom = []
    for step in range(SHIM_STEPS):
        for pal in range(4):
            amp = math.radians(SHIM_AMP_DEG[pal])
            theta = amp * math.sin(2.0 * math.pi * step / SHIM_STEPS)
            c, s = math.cos(theta), math.sin(theta)
            for entry in range(4):
                u0, v0 = PALETTES[pal][entry]
                du, dv = u0 - 512, v0 - 512
                u = 512 + du * c - dv * s
                v = 512 + du * s + dv * c
                u = max(64, min(960, int(round(u))))
                v = max(64, min(960, int(round(v))))
                rom.append((u, v))
    return rom


def _lattice(period, seed):
    state = seed & 0xFFFFFFFF
    vals = []
    for _ in range(period * period):
        state = (state * 1103515245 + 12345) & 0xFFFFFFFF
        vals.append((state >> 16) / 65535.0)
    return vals


def _value_noise(x, y, period, lat):
    def smooth(t):
        return t * t * (3.0 - 2.0 * t)
    x = x * period / NEB_SIZE
    y = y * period / NEB_SIZE
    x0, y0 = int(x) % period, int(y) % period
    x1, y1 = (x0 + 1) % period, (y0 + 1) % period
    fx, fy = smooth(x - int(x)), smooth(y - int(y))
    v00 = lat[y0 * period + x0]
    v10 = lat[y0 * period + x1]
    v01 = lat[y1 * period + x0]
    v11 = lat[y1 * period + x1]
    top = v00 + (v10 - v00) * fx
    bot = v01 + (v11 - v01) * fx
    return top + (bot - top) * fy


def build_nebula():
    octaves = [(2, 1.0), (4, 0.55), (8, 0.30), (16, 0.12)]
    lats = [(_lattice(p, NEB_SEED + i * 977), p) for i, (p, _) in enumerate(octaves)]
    raw = []
    for y in range(NEB_SIZE):
        for x in range(NEB_SIZE):
            v = 0.0
            for (lat, p), (_, amp) in zip(lats, octaves):
                v += amp * _value_noise(x, y, p, lat)
            raw.append(v / sum(a for _, a in octaves))
    shaped = [max(0.0, (v - 0.44) / 0.56) ** 1.6 for v in raw]
    out = []
    for y in range(NEB_SIZE):
        for x in range(NEB_SIZE):
            acc = 0.0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    w = 4 if (dx == 0 and dy == 0) else (2 if dx == 0 or dy == 0 else 1)
                    acc += w * shaped[((y + dy) % NEB_SIZE) * NEB_SIZE
                                      + ((x + dx) % NEB_SIZE)]
            out.append(min(255, int(round(acc / 16.0 * NEB_MAX))))
    return out


def build_exp2():
    return [int(round(4096.0 * 2.0 ** (-p / 256.0))) for p in range(256)]


def emit_vhdl(path):
    lines = []
    lines.append("-- Auto-generated by gen_kernels.py -- DO NOT EDIT BY HAND")
    lines.append("-- Glow-kernel ROMs {level(2b),shape(2b),|dy|(3b),|dx|(3b)} -> weight,")
    lines.append("-- shimmer palette, nebula texture, and the zoom exp2 table.")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("")
    lines.append("package warpfield_rom_pkg is")
    lines.append("")
    lines.append("    type t_kern_rom is array (0 to 1023) of std_logic_vector(7 downto 0);")
    lines.append("")
    rom = build_kernel()
    body = []
    for base in range(0, 1024, 16):
        chunk = ", ".join(f'x"{v:02X}"' for v in rom[base:base + 16])
        sep = "," if base + 16 < 1024 else ""
        body.append(f"        {chunk}{sep}")
    # 8 identical, separately-named constants (GHDL EBR-merge crash workaround)
    for layer in range(NUM_LAYERS):
        lines.append(f"    constant C_KERNEL_L{layer} : t_kern_rom := (")
        lines.extend(body)
        lines.append("    );")
        lines.append("")
    lines.append("    -- Shimmer palette: {shim_phase(4b), palette(2b), entry(2b)}")
    lines.append("    -- -> U(19:10) & V(9:0), hardware U/V convention.")
    lines.append("    type t_shim_rom is array (0 to 255) of std_logic_vector(19 downto 0);")
    lines.append("    constant C_SHIMPAL : t_shim_rom := (")
    shim = build_shimpal()
    for base in range(0, 256, 4):
        chunk = ", ".join(f'x"{(u << 10) | v:05X}"' for u, v in shim[base:base + 4])
        sep = "," if base + 4 < 256 else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("    -- Nebula texture: 64x64 tileable FBM wisps, address {y(5:0), x(5:0)}")
    lines.append("    type t_neb_rom is array (0 to 4095) of std_logic_vector(7 downto 0);")
    lines.append("    constant C_NEBULA : t_neb_rom := (")
    neb = build_nebula()
    for base in range(0, 4096, 16):
        chunk = ", ".join(f'x"{v:02X}"' for v in neb[base:base + 16])
        sep = "," if base + 16 < 4096 else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("    -- Zoom step table: exp2n[p] = round(4096 * 2^(-p/256)), 4.12 fixed")
    lines.append("    type t_exp_rom is array (0 to 255) of std_logic_vector(12 downto 0);")
    lines.append("    constant C_EXP2 : t_exp_rom := (")
    exp2 = build_exp2()
    for base in range(0, 256, 8):
        chunk = ", ".join(f'"{v:013b}"' for v in exp2[base:base + 8])
        sep = "," if base + 8 < 256 else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("end package warpfield_rom_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    emit_vhdl(os.path.join(here, "warpfield_rom_pkg.vhd"))
