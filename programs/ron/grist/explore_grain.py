#!/usr/bin/env python3
"""Grain-engine extremes explorer for GRIST (forked from morsel's dough grain).

Isolates morsel's surface-noise engine -- fine per-pixel hash grain, block-scale
mottle ("browning"), and sparse threshold pores -- and pushes each axis to an
extreme.  All variant math is integer shift/add/compare, so every look shown
here is directly implementable on the HX4K (no DSP, no BRAM needed).

Renders ext_*.png previews at 1920x1080 over a synthetic luma test base
(ramp + soft disc) so shadows, mids and highlights all show the texture.
"""

import numpy as np
from PIL import Image

W, H = 1920, 1080
M16 = 0xFFFF

# ---------------------------------------------------------------------------
# hash -- bit-exact with f_pack / f_mix1 / f_mix2 in morsel.vhd
# ---------------------------------------------------------------------------

def rotl16(v, n):
    return ((v << n) | (v >> (16 - n))) & M16


def f_pack(x, y, seed):
    x = np.asarray(x, dtype=np.int64)
    y = np.asarray(y, dtype=np.int64)
    a = (((x & 0x3FF) << 6) | (y & 0x3F)) & M16
    b = (((y & 0x3FF) << 6) | (x & 0x3F)) & M16
    return (a ^ rotl16(b, 5)) ^ seed


def f_mix1(h):
    h = h ^ rotl16(h, 7)
    return (h + rotl16(h, 3)) & M16


def f_mix2(h):
    h = h ^ rotl16(h, 11)
    return (h + rotl16(h, 5)) & M16


def f_hash(x, y, seed):
    return f_mix2(f_mix1(f_pack(x, y, seed)))


# ---------------------------------------------------------------------------
# coordinate grids and luma test base (10-bit, like the pipeline)
# ---------------------------------------------------------------------------
Y, X = np.mgrid[0:H, 0:W].astype(np.int64)

# horizontal ramp 96..928 plus a soft bright disc, so every tonal zone exists
base = 96 + (X * (928 - 96) // (W - 1))
r2 = (X - W // 3) ** 2 + (Y - H // 2) ** 2
disc = np.clip((360 ** 2 - r2) >> 9, 0, 300)
BASE = np.clip(base + disc, 0, 1023).astype(np.int64)


def nz_fine(seed=0x1234):
    """per-pixel signed nibble, -8..+7 (morsel's fine grain source)"""
    return (f_hash(X, Y, seed) & 15) - 8


def nz_block(ms, seed=0x77A1, bits=4):
    """block-scale signed noise: coords >> ms into the hash"""
    h = f_hash(X >> ms, Y >> ms, seed)
    if bits == 4:
        return ((h >> 4) & 15) - 8
    return (h & 0xFF) - 128        # 8-bit variant for big amplitude looks


def save(name, y10):
    y10 = np.clip(y10, 0, 1023)
    Image.fromarray((y10 >> 2).astype(np.uint8), "L").save(name)
    print(name)


# ---------------------------------------------------------------------------
# 1. baseline -- exactly what morsel ships (g=4, ms=3, pores at -60)
# ---------------------------------------------------------------------------
g = 4
fine = (nz_fine() * g) >> 1
mott = (nz_block(3) * g) >> 2
pore = np.where(((f_hash(X, Y, 0x1234) >> 12) & 15) < g, -60, 0)
save("ext_a_morsel_baseline.png", BASE + fine + mott + pore)

# ---------------------------------------------------------------------------
# 2. amplitude extreme -- grain IS the image (g -> 15, wider product)
# ---------------------------------------------------------------------------
g = 15
save("ext_b_amp_blizzard.png", BASE + ((nz_fine() * g) >> 0))

# ---------------------------------------------------------------------------
# 3. block extreme -- ms=6 (64 px cells): plaster / camouflage mottle
# ---------------------------------------------------------------------------
save("ext_c_blocks_plaster.png",
     BASE + ((nz_block(6, bits=8) * 3) >> 1) + ((nz_fine() * 3) >> 1))

# ---------------------------------------------------------------------------
# 4. pore extremes -- dense pitting (corroded metal) and bright sparkle
# ---------------------------------------------------------------------------
hp = (f_hash(X, Y, 0x1234) >> 12) & 15
save("ext_d_pores_corroded.png",
     BASE + np.where(hp < 10, -140, 0) + ((nz_fine() * 4) >> 1))
save("ext_e_pores_sparkle.png",
     (BASE >> 1) + np.where(hp >= 15, 500, 0) + ((nz_fine() * 3) >> 1))

# ---------------------------------------------------------------------------
# 5. octave stack -- 4 block scales summed: FBM-ish concrete / handmade paper
# ---------------------------------------------------------------------------
oct_sum = (nz_fine() * 2
           + (nz_block(1, seed=0x3B2F) << 2)
           + (nz_block(3, seed=0x77A1) << 3)
           + (nz_block(5, seed=0xC65D, bits=8) >> 1))
save("ext_f_octaves_concrete.png", BASE + oct_sum)

# ---------------------------------------------------------------------------
# 6. binary extreme -- grain vs luma comparator: mezzotint / photocopy
#    (pixel is white iff luma > dithered threshold; pure compare, no multiply)
# ---------------------------------------------------------------------------
thr = 512 + (nz_fine() << 5) + (nz_block(2, seed=0x9E37) << 4)
save("ext_g_binary_mezzotint.png", np.where(BASE > thr, 940, 64))

# ---------------------------------------------------------------------------
# 7. tonal-response extreme -- amplitude rides the mid-tones like film stock
#    weight = min(luma, 1023-luma) >> 5  (triangle peaked at mid grey)
# ---------------------------------------------------------------------------
wt = np.minimum(BASE, 1023 - BASE) >> 5      # 0..16
save("ext_h_film_response.png", BASE + ((nz_fine() * wt) >> 2))

# ---------------------------------------------------------------------------
# 8. carve extreme -- grain added BEFORE a 3-bit posterize, so contour edges
#    boil with grain while flat areas stay clean (etched / woodcut zones)
# ---------------------------------------------------------------------------
post = ((BASE + (nz_fine() << 3) + (nz_block(4, seed=0x51ED) << 4))
        >> 7) << 7
save("ext_i_carve_posterize.png", post + 64)
