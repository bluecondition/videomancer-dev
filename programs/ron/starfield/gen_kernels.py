#!/usr/bin/env python3
"""Generate starfield_rom_pkg.vhd: per-layer glow-kernel ROMs, shimmer
palette, palette luma dims, nebula texture and the P12 speed curve.

Kernel ROM layout:
    far layers 0..5  (16 px cells): 256 x 8-bit, one EBR
        address = {glow_level(2b), shape(2b), |dy|(2b), |dx|(2b)}
    near layers 6..7 (32 px cells): 1024 x 8-bit, two EBR
        address = {glow_level(2b), shape(2b), |dy|(3b), |dx|(3b)}

Shapes:
    0 = gaussian round glow
    1 = soft-edged square (superellipse)
    2 = diffraction spike (gauss core + axis arms)
    3 = point + faint halo

Glow level 0..3 selects kernel size (sigma). Depth fade is baked into
each layer's ROM (layer 0 dimmest ... layer 7 full brightness).

Cell-safe kernels (warpfield's HW-confirmed fix): a star's kernel must
be zero before it reaches its cell edge, because the neighbouring cell
draws only its own star (v3.0's 7.2-unit window with offsets 2..13 in a
16-unit cell sliced big stars flat). Two size classes:
  SMALL (far layers, 16-unit cells, offsets 4..11): zero from r = 4, so
      |d| <= 3 holds every non-zero weight -> 256-entry ROM.
  BIG (near layers, 32-unit cells, offsets 8..23): v3.0's full-size
      kernels (window to 7.2 < 8) -> 1024-entry ROM.
The near pair is where big stars belong anyway: nearer = bigger, sparser,
faster, brighter.

SPEED table: 256 x 16-bit, the P12 slider's speed curve in 1/64 px/frame
(front layer): a slow-drift floor at 0, quadratic up to ~40 px/frame.

Usage: python3 gen_kernels.py   (writes starfield_rom_pkg.vhd next to itself)
"""

import math
import os

NUM_LAYERS = 8
FIRST_BIG = 6                                        # layers 6, 7 = near pair
DEPTH_FADE = [64, 96, 128, 160, 192, 224, 240, 255]  # /255 per layer

SPIKE_AMP = 0.75                           # arm peak vs core peak
SPIKE_WIDTH = 0.40                         # arm perpendicular sigma
HALO_AMP = 0.30

# Per size class, per glow level 0..3. window = radial cos-roll start/end;
# reach = |d| values stored per axis.
SMALL = dict(                              # invisible beyond 4 units
    gauss=[0.60, 0.90, 1.30, 1.80],        # shape 0 sigma
    square=[0.80, 1.20, 1.65, 2.20],       # shape 1 superellipse radius
    spike_core=[0.60, 0.75, 0.95, 1.15],   # shape 2 core sigma
    spike_lambda=[0.80, 1.10, 1.50, 2.00], # shape 2 arm decay
    halo=[0.50, 0.65, 0.80, 1.00],         # shape 3 halo sigma
    window=(2.5, 4.0), reach=4)
BIG = dict(                                # v3.0's sizes, zero beyond 7.2
    gauss=[0.70, 1.15, 1.80, 2.80],
    square=[0.90, 1.40, 2.10, 2.90],
    spike_core=[0.70, 0.90, 1.15, 1.45],
    spike_lambda=[1.00, 1.50, 2.30, 3.20],
    halo=[0.55, 0.75, 0.95, 1.20],
    window=(4.5, 7.2), reach=8)


def kset(layer):
    return BIG if layer >= FIRST_BIG else SMALL

# Shimmer palette ROM: {shim_phase(4b), palette(3b), entry(2b)} -> U,V.
# Warpfield's HW-approved set (2026-10-07). Entries are authored as sRGB
# hues and converted with BT.601 (U = 512 + 896*Cr, V = 512 + 896*Cb --
# the hardware's U/V-swapped convention, verified on a scope: red =
# U 960 / V 360), then scaled to a chroma magnitude. Palette 7 is B&W
# (decoded in VHDL; entries here are white).
def _hue(rgb, mag):
    r, g, b = (c / 255.0 for c in rgb)
    cr = 0.5 * r - 0.4187 * g - 0.0813 * b
    cb = -0.1687 * r - 0.3313 * g + 0.5 * b
    n = math.hypot(cr, cb)
    if n == 0:
        return (512, 512)
    cr, cb = cr / n * mag, cb / n * mag
    return (int(round(512 + cr)), int(round(512 + cb)))

# Each entry: (sRGB hue, chroma magnitude, luma dim) -- dim 0 = x1,
# 1 = x7/8, 2 = x3/4, 3 = x1/2, applied to the brightness operand for star
# pixels only (entry 0 must stay dim 0: it also rides under the nebula).
W = ((255, 255, 255), 0, 0)
PALETTES = [
    # Silver -- white, silver, blue, ice blue
    [W, ((200, 210, 255), 45, 2), ((40, 80, 255), 330, 0), ((130, 180, 255), 180, 1)],
    # Candle -- white with some yellow / orange
    [W, ((255, 240, 190), 60, 0), ((255, 220, 40), 280, 0), ((255, 140, 20), 320, 0)],
    # Twilight -- blue / purple / pink (lavender-white sparkle)
    [((220, 210, 255), 90, 0), ((40, 70, 255), 330, 0), ((140, 40, 255), 320, 0), ((255, 70, 180), 300, 0)],
    # Neon -- green and magenta only
    [((40, 255, 80), 340, 0), ((255, 0, 255), 360, 0), ((120, 255, 140), 160, 1), ((255, 90, 230), 250, 0)],
    # Ember -- red / pink / orange (peach-white sparkle)
    [((255, 225, 205), 90, 0), ((255, 20, 20), 340, 0), ((255, 70, 150), 300, 0), ((255, 130, 0), 320, 0)],
    # Honey -- all yellow hues: yellow, gold, pale yellow, deep amber
    [((255, 250, 220), 110, 0), ((255, 230, 0), 300, 0), ((255, 180, 0), 290, 1), ((230, 150, 0), 320, 2)],
    # Rainbow -- white, red, green, blue (wide shimmer swing)
    [W, ((255, 0, 0), 340, 0), ((0, 255, 0), 340, 0), ((0, 0, 255), 340, 0)],
    # B&W (VHDL forces neutral chroma)
    [W, W, W, W],
]
SHIM_STEPS = 16
SHIM_AMP_DEG = [8.0, 10.0, 20.0, 15.0, 15.0, 10.0, 45.0, 0.0]

# P12 speed curve (front layer, 1/64 px/frame): floor at slider 0 so the
# field never stops (Direction/Depth stay live), quadratic to the top.
SPEED_FLOOR = 16       # 0.25 px/frame
SPEED_MAX = 2560       # 40 px/frame

# Nebula texture: 64x64 tileable 3-octave value noise, wisp-shaped and
# pre-blurred (it is nearest-sampled at 4x stretch on hardware, so the
# blur is what keeps texel blocks invisible).
NEB_SIZE = 64
NEB_SEED = 0x5EED
NEB_MAX = 200          # peak texture value (weight domain, pre-scale)


def window(r, k):
    w0, w1 = k["window"]
    if r <= w0:
        return 1.0
    if r >= w1:
        return 0.0
    t = (r - w0) / (w1 - w0)
    return 0.5 * (1.0 + math.cos(math.pi * t))


def shape_value(shape, level, dx, dy, k):
    r2 = dx * dx + dy * dy
    r = math.sqrt(r2)
    if shape == 0:  # gaussian
        s = k["gauss"][level]
        w = math.exp(-r2 / (2.0 * s * s))
    elif shape == 1:  # soft superellipse square
        r4 = (dx ** 4 + dy ** 4) ** 0.25
        R = k["square"][level]
        w = math.exp(-((r4 / R) ** 4))
    elif shape == 2:  # diffraction spike
        s = k["spike_core"][level]
        core = math.exp(-r2 / (2.0 * s * s))
        lam = k["spike_lambda"][level]
        pw2 = 2.0 * SPIKE_WIDTH * SPIKE_WIDTH
        arm_x = SPIKE_AMP * math.exp(-dx / lam) * math.exp(-(dy * dy) / pw2)
        arm_y = SPIKE_AMP * math.exp(-dy / lam) * math.exp(-(dx * dx) / pw2)
        w = max(core, arm_x, arm_y)
    else:  # point + faint halo
        s = k["halo"][level]
        halo = HALO_AMP * math.exp(-r2 / (2.0 * s * s))
        w = 1.0 if (dx == 0 and dy == 0) else halo
    return w * window(r, k)


def build_kernel(layer):
    fade = DEPTH_FADE[layer]
    k = kset(layer)
    reach = k["reach"]
    rom = []
    for level in range(4):
        for shape in range(4):
            for ady in range(9):
                for adx in range(9):
                    w = shape_value(shape, level, adx, ady, k)
                    q = int(round(w * fade))
                    q = max(0, min(255, q))
                    if q < 2:
                        q = 0
                    if adx >= reach or ady >= reach:
                        # cell-safe: nothing may survive outside the reach
                        assert q == 0, (layer, level, shape, adx, ady, q)
                        continue
                    rom.append(q)
    assert len(rom) == 16 * reach * reach
    return rom


def ascii_preview(level, shape, k):
    ramp = " .:-=+*#%@"
    reach = k["reach"]
    print(f"shape {shape} level {level} (reach {reach}):")
    for dy in range(reach - 1, -1, -1):
        row = ""
        for dx in range(-(reach - 1), reach):
            v = shape_value(shape, level, abs(dx), dy, k) * 255
            row += ramp[min(9, int(v) * 10 // 256)]
        print("   " + row)


def build_shimpal():
    """512-entry shimmer palette: sinusoidal hue orbit around each base."""
    rom = []
    for step in range(SHIM_STEPS):
        for pal in range(8):
            amp = math.radians(SHIM_AMP_DEG[pal])
            theta = amp * math.sin(2.0 * math.pi * step / SHIM_STEPS)
            c, s = math.cos(theta), math.sin(theta)
            for entry in range(4):
                rgb, mag, _dim = PALETTES[pal][entry]
                u0, v0 = _hue(rgb, mag)
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


def build_speed():
    return [int(round(SPEED_FLOOR + (SPEED_MAX - SPEED_FLOOR) * (k / 255.0) ** 2))
            for k in range(256)]


def emit_vhdl(path):
    lines = []
    lines.append("-- Auto-generated by gen_kernels.py -- DO NOT EDIT BY HAND")
    lines.append("-- Per-layer glow-kernel ROMs: {level(2b),shape(2b),|dy|,|dx|} -> weight")
    lines.append("-- (2-bit |d| far layers 0..5, 3-bit |d| near layers 6..7),")
    lines.append("-- shimmer palette, palette luma dims, nebula texture, P12 speed curve.")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("")
    lines.append("package starfield_rom_pkg is")
    lines.append("")
    lines.append("    type t_kern_rom is array (0 to 255) of std_logic_vector(7 downto 0);")
    lines.append("    type t_kern_big_rom is array (0 to 1023) of std_logic_vector(7 downto 0);")
    lines.append("")
    # One separately-named constant per layer: GHDL synthesis crashes
    # (netlists-memories.adb internal error) if the 8 parallel reads come
    # from a single array-of-arrays constant -- it tries to merge them
    # into one 8192-deep ROM. Distinct constants infer 8 independent EBR
    # ROMs cleanly (same pattern as inferno's C_PAL).
    for layer in range(NUM_LAYERS):
        rom = build_kernel(layer)
        big = layer >= FIRST_BIG
        lines.append(f"    -- layer {layer} (depth fade {DEPTH_FADE[layer]}/255, "
                     f"{'32 px cells, v3.0 sizes' if big else '16 px cells'})")
        lines.append(f"    constant C_KERNEL_L{layer} : "
                     f"{'t_kern_big_rom' if big else 't_kern_rom'} := (")
        n = len(rom)
        for base in range(0, n, 16):
            chunk = ", ".join(f'x"{v:02X}"' for v in rom[base:base + 16])
            sep = "," if base + 16 < n else ""
            lines.append(f"        {chunk}{sep}")
        lines.append("    );")
        lines.append("")
    lines.append("    -- Shimmer palette: {shim_phase(4b), palette(3b), entry(2b)}")
    lines.append("    -- -> U(19:10) & V(9:0), hardware U/V convention.")
    lines.append("    type t_shim_rom is array (0 to 511) of std_logic_vector(19 downto 0);")
    lines.append("    constant C_SHIMPAL : t_shim_rom := (")
    shim = build_shimpal()
    for base in range(0, 512, 4):
        chunk = ", ".join(f'x"{(u << 10) | v:05X}"' for u, v in shim[base:base + 4])
        sep = "," if base + 4 < 512 else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("    -- Per-entry luma dim {palette(3b), entry(2b)} -> 0 = x1, 1 = x7/8,")
    lines.append("    -- 2 = x3/4, 3 = x1/2 (star pixels only)")
    lines.append("    type t_dim_rom is array (0 to 31) of std_logic_vector(1 downto 0);")
    lines.append("    constant C_PALDIM : t_dim_rom := (")
    dims = [f'"{PALETTES[pal][e][2]:02b}"' for pal in range(8) for e in range(4)]
    for base in range(0, 32, 8):
        sep = "," if base + 8 < 32 else ""
        lines.append("        " + ", ".join(dims[base:base + 8]) + sep)
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
    lines.append("    -- P12 speed curve: front-layer speed in 1/64 px/frame, floor")
    lines.append(f"    -- {SPEED_FLOOR} at slider 0, quadratic to {SPEED_MAX} at the top")
    lines.append("    type t_speed_rom is array (0 to 255) of std_logic_vector(15 downto 0);")
    lines.append("    constant C_SPEED : t_speed_rom := (")
    spd = build_speed()
    for base in range(0, 256, 8):
        chunk = ", ".join(f'x"{v:04X}"' for v in spd[base:base + 8])
        sep = "," if base + 8 < 256 else ""
        lines.append(f"        {chunk}{sep}")
    lines.append("    );")
    lines.append("")
    lines.append("end package starfield_rom_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for k in (SMALL, BIG):
        for shp in (0, 2):
            ascii_preview(3, shp, k)
    emit_vhdl(os.path.join(here, "starfield_rom_pkg.vhd"))
