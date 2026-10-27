#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""MOOD lookup tables: the engine ROM (1024 x 16) and shared constants.

ROM layout (word addresses):
     0..511   quarter-wave sine, Q15 (the SIN unit folds the angle)
   512..      raw tables, 16-word banks: SIN imm=raw(bank) reads
              512 + ((A + 16 * bank) mod 512)
     bank  0  128  contour coefficient am = 65536 * (1 - exp(-1/tau)), index
                   K>>3, tau 0.3 ms .. 100 ms, log taper
     bank  8   64  Table I: zero-crossing count (6 bits) -> colour tap n
     bank 12   64  popcount of a 6-bit value
     bank 16   64  legal chroma radius by hue (64 hues), offset units (x896)
     bank 20   16  bass note phase increments (minor pentatonic, ~41-247 Hz)
     bank 21   16  mid note phase increments (minor pentatonic, ~294-1976 Hz)
     bank 22   64  pattern flags, index arrangement*16 + step
     bank 26    8  format class: Wpx/32, Q8.8
     bank 27    8  format class: field lines/32, Q8.8
     bank 28    4  1/N for the H array (Q15), N = 1, 2, 3, 5
     bank 29    4  1/M for the V array (Q15), M = 1, 2, 4, 8
     bank 30   16  phantom voice levels, index arrangement*4 + voice
                   (kick, bass, mid, hats)
"""
import math

FS = 15734.264          # virtual sample rate = NTSC line rate (4.5 MHz / 286)
VFRAME = 262            # virtual lines per virtual frame
VIS_CLK = 208           # virtual clocks across the active width (of 228)
VIS_LINES = 200         # virtual lines down the active height (31..230)
Y_OBJ = 458             # object luma (y = 0.45)
Y_FRAC = (Y_OBJ - 64) / 876.0
CHROMA_CAP = 0.40       # flatten the palette: no hue louder than this radius

# format classes: (name, active width, active lines per FIELD)
CLASSES = [
    ('1080p', 1920, 1080),
    ('1080i', 1920, 540),
    ('720p', 1280, 720),
    ('480i', 720, 243),
    ('480p', 720, 480),
    ('576i', 720, 288),
    ('576p', 720, 576),
]

# pattern flag bits
F_KICK, F_BASSL, F_BASSR, F_HATL, F_HATR, F_MID, F_ACC, F_SNR = (1 << i for i in range(8))
ARR_NAMES = ['full', 'breakdown', 'build', 'drop']
# voice peak levels per arrangement: kick, bass, mid, hats
LEVELS = [
    (10000, 5000, 6000, 6000),     # full
    (0, 3000, 9000, 3000),         # breakdown
    (8000, 4500, 7000, 7500),      # build
    (12000, 6000, 5000, 9000),     # drop
]


def contour_tau(i):
    """tau in samples for table index i (0..127): 0.3 ms .. 100 ms, log"""
    t0, t1 = 0.0003 * FS, 0.100 * FS
    return t0 * (t1 / t0) ** (i / 127.0)


def contour_table():
    return [round(65536 * (1 - math.exp(-1.0 / contour_tau(i)))) for i in range(128)]


def table_i(c):
    """Atari patent Table I: X1 discarded, zones widen with the count"""
    c &= 63
    if c < 16:
        return 16 - (c >> 1)
    if c < 32:
        return 8 - ((c - 16) >> 2)
    return 4 - ((c - 32) >> 3)


def legal_radius(h):
    """largest chroma radius (normalised, pb/pr units) keeping RGB in [0,1]
    at the object luma, for hue angle h (atan2(pr, pb))"""
    y = Y_FRAC
    c, s = math.cos(h), math.sin(h)
    best = CHROMA_CAP
    for a, b in ((0, 1.402), (-0.344136, -0.714136), (1.772, 0)):
        g = a * c + b * s
        if g > 1e-9:
            best = min(best, (1 - y) / g)
        if g < -1e-9:
            best = min(best, y / -g)
    return best


def inc(f):
    return round(f * 65536 / FS)


def pent(f0, n):
    """minor pentatonic ascending from f0, n notes"""
    steps = [0, 3, 5, 7, 10]
    out = []
    k = 0
    while len(out) < n:
        out.append(f0 * 2 ** ((12 * (k // 5) + steps[k % 5]) / 12.0))
        k += 1
    return out


BASS_HZ = pent(41.2, 14)        # E1 .. ~247 Hz
MID_HZ = pent(293.66, 14)       # D4 .. ~2 kHz


def patterns():
    """16-step patterns per arrangement (full / breakdown / build / drop)"""
    P = [[0] * 16 for _ in range(4)]
    for s in range(16):
        f = 0
        if s % 4 == 0: f |= F_KICK
        if s % 4 == 2: f |= F_BASSL | F_HATL | F_ACC
        if s % 4 == 3: f |= F_BASSR
        if s % 2 == 0 and s % 4 != 2: f |= F_HATR
        if s in (4, 12): f |= F_SNR
        if s in (0, 8): f |= F_MID
        P[0][s] = f
        # breakdown: no kick, sparse bass, slow mid swells, soft hats
        f = 0
        if s in (0, 8): f |= F_BASSL | F_MID
        if s in (4, 12): f |= F_BASSR
        if s % 4 == 2: f |= F_HATL if s % 8 == 2 else F_HATR
        P[1][s] = f
        # build: kick on beats, 16th hats, 8th bass
        f = F_HATL | F_HATR
        if s % 4 == 0: f |= F_KICK
        if s % 2 == 0: f |= F_BASSL
        if s % 2 == 1: f |= F_BASSR
        if s % 8 == 0: f |= F_MID
        if s in (4, 12): f |= F_SNR
        P[2][s] = f
        # drop: full with offbeat kick, accented 16th hats
        f = F_HATL | F_HATR
        if s % 4 == 0 or s == 14: f |= F_KICK
        if s % 2 == 0: f |= F_BASSL | F_BASSR
        if s % 4 == 2: f |= F_ACC
        if s in (4, 12): f |= F_SNR
        if s in (0, 8): f |= F_MID
        P[3][s] = f
    return [f for arr in P for f in arr]


BANK = dict(contour=0, tablei=8, popcount=12, radius=16, bass=20, mid=21,
            pattern=22, wpx=26, lines=27, recipn=28, recipm=29, levels=30)


def rom():
    sin = [min(32767, round(32767 * math.sin(math.pi / 2 * i / 512))) for i in range(512)]
    raw = [0] * 512

    def put(bank, vals):
        b = 16 * BANK[bank]
        raw[b:b + len(vals)] = vals

    put('contour', contour_table())
    put('tablei', [table_i(c) for c in range(64)])
    put('popcount', [bin(c).count('1') for c in range(64)])
    put('radius', [round(896 * legal_radius(2 * math.pi * (i + 0.5) / 64)) for i in range(64)])
    put('bass', [inc(f) for f in BASS_HZ])
    put('mid', [inc(f) for f in MID_HZ])
    put('pattern', patterns())
    put('wpx', [round(w / 32 * 256) for _, w, _ in CLASSES])
    put('lines', [round(lf / 32 * 256) for _, _, lf in CLASSES])
    put('recipn', [32767, 16384, 10923, 6554])
    put('recipm', [32767, 16384, 8192, 4096])
    put('levels', [l for arr in LEVELS for l in arr])
    words = sin + raw
    assert len(words) == 1024 and all(-32768 <= v < 32768 for v in words)
    return words


if __name__ == '__main__':
    r = rom()
    for k, b in BANK.items():
        print(f'{k:9s}', r[512 + 16 * b:512 + 16 * b + 16])
