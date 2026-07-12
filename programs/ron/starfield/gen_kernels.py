#!/usr/bin/env python3
"""Generate starfield_rom_pkg.vhd: per-layer glow-kernel ROMs.

Kernel ROM layout (per layer, 1024 x 8-bit):
    address = {glow_level(2b), shape(2b), |dy|(3b), |dx|(3b)}

Shapes:
    0 = gaussian round glow
    1 = soft-edged square (superellipse)
    2 = diffraction spike (gauss core + axis arms)
    3 = point + faint halo

Glow level 0..3 selects kernel size (sigma). Depth fade is baked into
each layer's ROM (layer 0 dimmest ... layer 7 full brightness).

A radial window forces every kernel toward zero beyond r ~= 5 so that
single-cell sampling (|d| <= 7) never shows a hard clip at cell edges.

Usage: python3 gen_kernels.py   (writes starfield_rom_pkg.vhd next to itself)
"""

import math
import os

NUM_LAYERS = 8
DEPTH_FADE = [64, 96, 128, 160, 192, 224, 240, 255]  # /255 per layer

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
# (hardware U/V convention, values copied from the v2 case ROM).
# Entry 0 is neutral white; hue rotation of a zero chroma vector is a
# no-op, so white stays white at every shimmer step for free.
PALETTES = [
    # Mixed (entries 2,3 get a saturation boost -- they read as grey)
    [(512, 512), (300, 760), (622, 347), (597, 647)],
    # Cool
    [(512, 512), (280, 800), (620, 720), (480, 560)],
    # Warm
    [(512, 512), (720, 340), (640, 400), (560, 460)],
    # Rainbow
    [(512, 512), (800, 350), (180, 660), (780, 760)],
]
SHIM_STEPS = 16
SHIM_AMP_DEG = [12.0, 12.0, 12.0, 25.0]   # per palette; Rainbow swings wide

# Nebula texture: 64x64 tileable 3-octave value noise, wisp-shaped and
# pre-blurred (it is nearest-sampled at 4x stretch on hardware, so the
# blur is what keeps texel blocks invisible).
NEB_SIZE = 64
NEB_SEED = 0x5EED
NEB_MAX = 200          # peak texture value (weight domain, pre-scale)


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


def build_kernel(layer):
    fade = DEPTH_FADE[layer]
    rom = []
    for level in range(4):
        for shape in range(4):
            for ady in range(8):
                for adx in range(8):
                    w = shape_value(shape, level, adx, ady)
                    q = int(round(w * fade))
                    q = max(0, min(255, q))
                    if q < 2:
                        q = 0
                    rom.append(q)
    return rom


def ascii_preview(level, shape):
    ramp = " .:-=+*#%@"
    print(f"shape {shape} level {level} (layer 7):")
    for dy in range(7, -1, -1):
        row = ""
        for dx in range(-7, 8):
            v = shape_value(shape, level, abs(dx), dy) * 255
            row += ramp[min(9, int(v) * 10 // 256)]
        print("   " + row)


def build_shimpal():
    """256-entry shimmer palette: sinusoidal hue orbit around each base."""
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
    """Random values on a period x period lattice (deterministic LCG)."""
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
    """64x64 tileable FBM wisps, gamma-shaped, 3x3 blurred, 0..NEB_MAX."""
    octaves = [(2, 1.0), (4, 0.55), (8, 0.30), (16, 0.12)]
    lats = [(_lattice(p, NEB_SEED + i * 977), p) for i, (p, _) in enumerate(octaves)]
    raw = []
    for y in range(NEB_SIZE):
        for x in range(NEB_SIZE):
            v = 0.0
            for (lat, p), (_, amp) in zip(lats, octaves):
                v += amp * _value_noise(x, y, p, lat)
            raw.append(v / sum(a for _, a in octaves))
    # wisp shaping: push midtones down so cloud cores stand out
    shaped = [max(0.0, (v - 0.44) / 0.56) ** 1.6 for v in raw]
    # 3x3 blur (wrapping)
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


def emit_vhdl(path):
    lines = []
    lines.append("-- Auto-generated by gen_kernels.py -- DO NOT EDIT BY HAND")
    lines.append("-- Per-layer glow-kernel ROMs: {level(2b),shape(2b),|dy|(3b),|dx|(3b)} -> weight")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("")
    lines.append("package starfield_rom_pkg is")
    lines.append("")
    lines.append("    type t_kern_rom is array (0 to 1023) of std_logic_vector(7 downto 0);")
    lines.append("")
    # One separately-named constant per layer: GHDL synthesis crashes
    # (netlists-memories.adb internal error) if the 8 parallel reads come
    # from a single array-of-arrays constant -- it tries to merge them
    # into one 8192-deep ROM. Distinct constants infer 8 independent EBR
    # ROMs cleanly (same pattern as inferno's C_PAL).
    for layer in range(NUM_LAYERS):
        rom = build_kernel(layer)
        lines.append(f"    -- layer {layer} (depth fade {DEPTH_FADE[layer]}/255)")
        lines.append(f"    constant C_KERNEL_L{layer} : t_kern_rom := (")
        for base in range(0, 1024, 16):
            chunk = ", ".join(f'x"{v:02X}"' for v in rom[base:base + 16])
            sep = "," if base + 16 < 1024 else ""
            lines.append(f"        {chunk}{sep}")
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
    lines.append("end package starfield_rom_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for shp in range(4):
        ascii_preview(3, shp)
    emit_vhdl(os.path.join(here, "starfield_rom_pkg.vhd"))
