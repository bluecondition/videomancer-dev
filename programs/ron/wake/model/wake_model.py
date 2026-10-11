#!/usr/bin/env python3
"""WAKE / motion_key -- fixed-point reference model.

Golden model for the VHDL. Integer math only, with the same widths, rounding,
LUTs and update rules the RTL will use, so values tuned here transfer
directly and the per-update NPZ dumps can be diffed bit-exactly against a
GHDL testbench.

Phase 1 scope: Motion mode; effects Matte, Heat, Mono, Quarter Turn,
Negative; controls WAKE, FLOOR, CHROMA, SOFTNESS, STIR, KEY.

Engine summary (one "update" = one frame, or one field when interlaced)
  Geometry   C cols (32 HD / 24 SD) x R=18 rows. Cell edges come from a
             Bresenham counter over the MEASURED active W x H:
             col(x) = floor(x*C/W), row(y) = floor(y*R/H). A half-cell
             counter (2C, 2R) gives the interpolation segments.
  Accumulate 8-bit samples (10-bit >> 2). Per line, per cell: partial sum
             P; column accumulator A += P >> sh (sh keeps A < 2^16).
  Normalize  cur = min(255, (A*recip + 2^15) >> 16),
             recip = round(2^(16+sh) * C*R / (W*H)).
  Difference d = |dY| + (((|dU|+|dV|) * CHROMA) >> 9)      (CHROMA 512 = 1x)
  Floor/gain dp = min(255, max(0, d - (FLOOR>>5)) << 2)    (M integer units)
  Release    e16 = clamp((((WAKE>>3)*3)>>2) - 95 + RATE_OFF16[tid] (-16 if one-field), 0, 255)
             dec = max(1, ((M >> e16[7:4]) * MANT_DN[e16[3:0]]) >> 5) if M>0
             M   = max(dp << 4, M - dec)          (M is 12-bit 8.4 fixed point)
  Reference  ref = cur (Motion mode). First update after load or a geometry
             change seeds ref and holds M = 0 (no start-up flash).
  Interlace  default: update on field 0 only (same-parity compare, half rate);
             --both-fields compares every field with the previous one.
Render (position-locked; frame N uses M after update N-1)
  m10 = M >> 2 per cell; bilinear between cell centres:
    vertical lerp per line (cell rate), horizontal value DDA per pixel;
  SOFTNESS < 128: nearest cell (blocky).
  key = clamp((m - thr) * gain), thr from STIR (exponential), gain from
  SOFTNESS (half-octave steps of shift + 1.5x), KEY Invert = 1023 - key.
Mix  out = a + (((b - a) * key + 512) >> 10)  (SDK interpolator_u, clamped)

Usage
  wake_model.py --synthetic --format 1080p30 --out clips/synth_1080p30
  wake_model.py --input clip.mp4 --format 720p60 --out clips/mine --effect Mono --key 1
  wake_model.py --synthetic --format ntsc --preset "First Look" --dump g.npz
Control flags take raw register values (knobs/fader 0..1023, switches 0/1).
"""
import argparse
import math
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Formats: name -> (timing_id, width, frame_height, interlaced, update_hz,
#                   cols, matrix). Rows are always 18.
# ---------------------------------------------------------------------------
FORMATS = {
    "ntsc":      (0,  720,  486,  True,  59.94,  24, "601"),
    "pal":       (8,  720,  576,  True,  50.0,   24, "601"),
    "480p":      (4,  720,  480,  False, 59.94,  24, "601"),
    "576p":      (12, 720,  576,  False, 50.0,   24, "601"),
    "720p60":    (14, 1280, 720,  False, 60.0,   32, "709"),
    "720p5994":  (6,  1280, 720,  False, 59.94,  32, "709"),
    "720p50":    (5,  1280, 720,  False, 50.0,   32, "709"),
    "1080i60":   (10, 1920, 1080, True,  60.0,   32, "709"),
    "1080i5994": (2,  1920, 1080, True,  59.94,  32, "709"),
    "1080i50":   (1,  1920, 1080, True,  50.0,   32, "709"),
    "1080p30":   (7,  1920, 1080, False, 30.0,   32, "709"),
    "1080p2997": (13, 1920, 1080, False, 29.97,  32, "709"),
    "1080p25":   (11, 1920, 1080, False, 25.0,   32, "709"),
    "1080p24":   (3,  1920, 1080, False, 24.0,   32, "709"),
    "1080p2398": (9,  1920, 1080, False, 23.976, 32, "709"),
}
ROWS = 18

# log2(update rate) in 1/16 octave, indexed by timing ID (VHDL: 16-entry constant).
RATE_OFF16 = [94] * 16                      # unknown IDs: 59.94
for _tid, _w, _h, _il, _hz, _c, _m in FORMATS.values():
    RATE_OFF16[_tid] = round(16 * math.log2(_hz))

MANT_DN = [round(32 * 2 ** (-i / 16)) for i in range(16)]  # release mantissa
MANT_UP = [round(32 * 2 ** (i / 16)) for i in range(16)]   # STIR threshold mantissa

# WAKE knob -> release time constant (exponential taper, 6 octaves):
#   e16_sec = (((WAKE>>3) * 3) >> 2) - 95   ->  tau = 2^(e16_sec/16) s = 1/61 s .. 1 s
# tau = 1 s keeps a stopped object keyed ~4 s at the default STIR/SOFTNESS.
WAKE_BIAS = 95


def wake_e16(wake, rate16):
    """Release exponent in 1/16 octaves of updates (0 = release in one update)."""
    return min(255, max(0, (((wake >> 3) * 3) >> 2) - WAKE_BIAS + rate16))

EFFECTS = ["Matte", "Heat", "Mono", "Quarter Turn", "Negative",
           "Solarize", "Crush", "Smear", "Cloak"]
IMPLEMENTED = {"Matte", "Heat", "Mono", "Quarter Turn", "Negative", "Solarize", "Crush", "Smear"}


def effect_index(v):
    """Register value -> label index (the firmware loads a labelled default as the
    raw index, so values < N are taken as indices)."""
    n = len(EFFECTS)
    return v if v < n else (v * n) >> 10


# ---------------------------------------------------------------------------
# Colour conversion (float, I/O only -- the engine never sees floats)
# ---------------------------------------------------------------------------
KRB = {"601": (0.299, 0.114), "709": (0.2126, 0.0722)}


def rgb_to_yuv10(rgb, matrix):
    kr, kb = KRB[matrix]
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = kr * r + (1 - kr - kb) * g + kb * b
    cb = (b - y) / (2 * (1 - kb))
    cr = (r - y) / (2 * (1 - kr))
    out = np.stack([64 + 876 * y, 512 + 896 * cb, 512 + 896 * cr], -1)
    return np.clip(np.rint(out), 0, 1023).astype(np.int32)


def yuv10_to_rgb8(yuv, matrix):
    kr, kb = KRB[matrix]
    y = (yuv[..., 0] - 64) / 876.0
    cb = (yuv[..., 1] - 512) / 896.0
    cr = (yuv[..., 2] - 512) / 896.0
    r = y + 2 * (1 - kr) * cr
    b = y + 2 * (1 - kb) * cb
    g = (y - kr * r - kb * b) / (1 - kr - kb)
    return np.clip(np.rint(np.stack([r, g, b], -1) * 255), 0, 255).astype(np.uint8)


def heat_palette():
    """64 entries: black, blue, magenta, red, yellow, white (BT.601, 10-bit)."""
    keys = [(0, (0, 0, 0)), (16, (0, 0, 1)), (28, (1, 0, 1)),
            (40, (1, 0, 0)), (52, (1, 1, 0)), (63, (1, 1, 1))]
    rgb = np.zeros((64, 3))
    for (i0, c0), (i1, c1) in zip(keys, keys[1:]):
        for i in range(i0, i1 + 1):
            f = (i - i0) / (i1 - i0)
            rgb[i] = np.array(c0) * (1 - f) + np.array(c1) * f
    return rgb_to_yuv10(rgb, "601")


HEAT = heat_palette()


# ---------------------------------------------------------------------------
# Controls
# ---------------------------------------------------------------------------
@dataclass
class Controls:
    wake: int = 409       # K1  (40%)
    floor: int = 256      # K2  (25%; v3 FLOOR >> 4)
    chroma: int = 512     # K3  (centre = luma and chroma equal)
    softness: int = 512   # K4  (50%)
    detail: int = 0       # K5  (Phase 3)
    effect: int = 1       # K6  label index (Heat)
    mode: int = 0         # S7  Motion / Presence (Phase 4)
    plate: int = 0        # S8  Live / Hold (Phase 4)
    steady: int = 0       # S9  (Phase 3)
    track: int = 0        # S10 (Phase 4)
    key: int = 0          # S11 Normal / Invert
    stir: int = 614       # P12 (60%)


TOML_KEYS = {
    "rotary_potentiometer_1": "wake", "rotary_potentiometer_2": "floor",
    "rotary_potentiometer_3": "chroma", "rotary_potentiometer_4": "softness",
    "rotary_potentiometer_5": "detail", "rotary_potentiometer_6": "effect",
    "toggle_switch_7": "mode", "toggle_switch_8": "plate",
    "toggle_switch_9": "steady", "toggle_switch_10": "track",
    "toggle_switch_11": "key", "linear_potentiometer_12": "stir",
}


def load_preset(toml_path, name):
    data = tomllib.loads(Path(toml_path).read_text())
    for p in data.get("preset", []):
        if p.get("name", "").lower() == name.lower():
            ctl = Controls()
            for k, field in TOML_KEYS.items():
                if k in p:
                    setattr(ctl, field, int(p[k]))
            return ctl
    sys.exit(f"preset {name!r} not found in {toml_path}")


# ---------------------------------------------------------------------------
# Geometry (all integer; mirrors the RTL's measured-geometry registers)
# ---------------------------------------------------------------------------
class Geometry:
    def __init__(self, W, H, C, R, sh=None):
        self.W, self.H, self.C, self.R = W, H, C, R
        x = np.arange(W)
        y = np.arange(H)
        hc = (x * 2 * C) // W                 # half-column Bresenham
        hr = (y * 2 * R) // H
        self.col_x = hc >> 1
        self.row_y = hr >> 1
        self.col_starts = np.flatnonzero(np.diff(self.col_x, prepend=-1))
        self.row_starts = np.flatnonzero(np.diff(self.row_y, prepend=-1))
        cw_max = int(np.diff(np.append(self.col_starts, W)).max())
        ch_max = int(np.diff(np.append(self.row_starts, H)).max())
        n_max = cw_max * ch_max
        self.sh = max(0, n_max.bit_length() - 8) if sh is None else sh   # half-cells share the cell sh
        num = (C * R) << (16 + self.sh)
        self.recip = (num + (W * H) // 2) // (W * H)
        self.recip_c = ((C << 16) + W // 2) // W   # 2^16 * C / W
        self.recip_r = ((R << 16) + H // 2) // H   # 2^16 * R / H

        # horizontal interpolation segments: segment s runs from the centre of
        # col s to the centre of col s+1 (half-columns 2s+1, 2s+2). Edge
        # half-cells clamp: left -> (seg 0, j 0), right -> (seg C-1, j 0).
        seg = (hc - 1) >> 1
        seg_start = {}
        for xi in range(W):
            if 0 < hc[xi] < 2 * C - 1 and seg[xi] not in seg_start:
                seg_start[seg[xi]] = xi
        j = np.array([xi - seg_start[seg[xi]] if 0 < hc[xi] < 2 * C - 1 else 0
                      for xi in range(W)])
        seg = np.where(hc == 0, 0, np.where(hc == 2 * C - 1, C - 1, seg))
        self.seg_x, self.j_x = seg, j.astype(np.int32)

        # vertical: per line, rows r0/r1 and 8-bit fraction fy
        sr = (hr - 1) >> 1
        r_start = {}
        for yi in range(H):
            if 0 < hr[yi] < 2 * R - 1 and sr[yi] not in r_start:
                r_start[sr[yi]] = yi
        jy = np.array([yi - r_start[sr[yi]] if 0 < hr[yi] < 2 * R - 1 else 0
                       for yi in range(H)])
        top, bot = hr == 0, hr == 2 * R - 1
        self.r0 = np.where(top, 0, np.where(bot, R - 1, sr))
        self.r1 = np.where(top, 0, np.where(bot, R - 1, sr + 1))
        self.fy = np.where(top | bot, 0, np.minimum(255, (jy * self.recip_r) >> 8)).astype(np.int32)

    def describe(self):
        return (f"{self.W}x{self.H} grid {self.C}x{self.R} sh={self.sh} "
                f"recip={self.recip} recip_c={self.recip_c} recip_r={self.recip_r}")


def accumulate(ch8, geo):
    """Exact RTL accumulation: per-line cell partial sums, >> sh, summed down."""
    p = np.add.reduceat(ch8, geo.col_starts, axis=1) >> geo.sh   # (H, C)
    a = np.add.reduceat(p, geo.row_starts, axis=0)               # (R, C)
    assert a.max() < 65536, "accumulator overflow"
    return a


# ---------------------------------------------------------------------------
# motion_key engine (analysis side)
# ---------------------------------------------------------------------------
class MotionKey:
    def __init__(self, timing_id, cols, one_field=False):
        # one_field: under interlace, update only on field 0 (same-parity compare,
        # half update rate) -- removes field-alternation false motion for free.
        self.tid, self.cols, self.one_field = timing_id, cols, one_field
        self.geo = None
        self.ref = None
        self.M = None

    def update(self, yuv10, ctl, field=0):
        H, W = yuv10.shape[:2]
        if self.one_field and field and self.M is not None:
            return {"acc": np.zeros((ROWS, self.cols, 3), np.int64), "cur": self.ref, "d": np.zeros_like(self.M), "M": self.M.copy(),
                    "d_sum": 0, "seed": False, "skipped": True}
        seed = self.geo is None or (self.geo.W, self.geo.H) != (W, H)
        if seed:
            self.geo = Geometry(W, H, self.cols, ROWS)
        a = np.stack([accumulate((yuv10[..., c] >> 2).astype(np.int64), self.geo) for c in range(3)], -1)
        return self.finalize(a, ctl, seed)

    def finalize(self, a, ctl, seed=False):
        """Cell-rate part of an update, from the accumulator sums (control-dependent)."""
        g = self.geo
        cur = np.minimum(255, (a * g.recip + 32768) >> 16)
        if seed:
            d = np.zeros((ROWS, self.cols), np.int64)
            self.M = np.zeros_like(d)
        else:
            diff = np.abs(cur - self.ref)
            d = diff[..., 0] + (((diff[..., 1] + diff[..., 2]) * ctl.chroma) >> 9)
            dp = np.minimum(255, np.maximum(0, d - (ctl.floor >> 5)) << 2) << 4
            rate16 = RATE_OFF16[self.tid] - (16 if self.one_field else 0)
            e16 = wake_e16(ctl.wake, rate16)
            dec = ((self.M >> (e16 >> 4)) * MANT_DN[e16 & 15]) >> 5
            dec = np.where(self.M > 0, np.maximum(dec, 1), 0)
            self.M = np.maximum(dp, self.M - dec)
        self.ref = cur
        return {"acc": a, "cur": cur, "d": d, "M": self.M.copy(),
                "d_sum": int(d.sum()), "seed": seed, "skipped": False}


# ---------------------------------------------------------------------------
# v3 engine (proposed): 2x2 luma sub-cells, 1/4-LSB means, short-memory reference.
# Bench + footage tuning (2026-10-10): v2's integer cell means compared frame to
# frame keyed 0-30% of real human motion; v3 keys 50-95% at the same false rate.
# ---------------------------------------------------------------------------
GAIN3 = 4            # dp = min(255, ((d4 - f4) * GAIN3) >> 2)   (d4 in 1/4 LSB)
CUT_THR = 224        # STEADY cut guard: running mean d (1/4 LSB, rows >= 2) above this = cut (footage: cuts >= 236, action p99 180)
DETAIL_GAIN = 3      # DETAIL: edge energy = min(1023, |Y[x]-Y[x-2]| << DETAIL_GAIN)
DETAIL_DECAY = 64    # DETAIL: peak-hold spread, energy lost per pixel (rightward)
STIR_V2 = False      # True = the v0.2.x STIR curve
ALPHA_SH = 2         # reference memory: ref += (cur - ref) >> ALPHA_SH  (1/4)


def accumulate3(yuv10, geo):
    """Luma per half-cell (2C x 2R) + U/V per cell (C x R). Same per-line >> sh rule."""
    g2 = geo.half
    y = accumulate((yuv10[..., 0] >> 2).astype(np.int64), g2)          # (2R, 2C)
    u = accumulate((yuv10[..., 1] >> 2).astype(np.int64), geo)          # (R, C)
    v = accumulate((yuv10[..., 2] >> 2).astype(np.int64), geo)
    return y, u, v


class MotionKey3:
    """v3: cur in 1/4 LSB (10-bit), ref in 1/16 LSB (12-bit), d in 1/4 LSB."""

    def __init__(self, timing_id, cols, one_field=False):
        self.tid, self.cols, self.one_field = timing_id, cols, one_field
        self.geo = None
        self.ref_y = self.ref_u = self.ref_v = None
        self.M = None
        self.g_prev = 0          # mean raw d of the previous update (STEADY)
        self.seed_next = False   # cut -> re-seed on the next update
        self.mute = False        # cut -> key muted until the re-seed completes

    def set_geometry(self, W, H):
        self.geo = Geometry(W, H, self.cols, ROWS)
        self.geo.half = Geometry(W, H, 2 * self.cols, 2 * ROWS, sh=self.geo.sh)

    def update(self, yuv10, ctl, field=0):
        H, W = yuv10.shape[:2]
        if self.one_field and field and self.M is not None:
            return {"d": np.zeros_like(self.M), "M": self.M.copy(), "seed": False, "skipped": True}
        seed = self.geo is None or (self.geo.W, self.geo.H) != (W, H)
        if seed:
            self.set_geometry(W, H)
        return self.finalize(accumulate3(yuv10, self.geo), ctl, seed)

    def finalize(self, acc3, ctl, seed=False):
        g = self.geo
        seed = seed or self.seed_next
        self.seed_next = False
        ay, au, av = acc3
        cy = np.minimum(1023, (ay * g.recip + 2048) >> 12)     # sub-cell luma: area/4
        cu = np.minimum(1023, (au * g.recip + 8192) >> 14)
        cv = np.minimum(1023, (av * g.recip + 8192) >> 14)
        R, C = cu.shape
        cut = False
        if seed:
            self.ref_y, self.ref_u, self.ref_v = cy << 2, cu << 2, cv << 2
            self.M = np.zeros((R, C), np.int64)
            d = np.zeros((R, C), np.int64)
            self.mute = False
        else:
            dys = np.abs(cy - ((self.ref_y + 2) >> 2))
            dy = dys.reshape(R, 2, C, 2).sum(axis=(1, 3)) >> 1
            du = np.abs(cu - ((self.ref_u + 2) >> 2))
            dv = np.abs(cv - ((self.ref_v + 2) >> 2))
            d = dy + (((du + dv) * ctl.chroma) >> 9)
            dc = d
            seedrows = np.zeros(R, bool)
            if ctl.steady:
                # cut guard: running mean of raw d as rows complete (checked from row 2);
                # rows after the cut row re-seed at once (as the RTL does, row by row)
                run_sum = np.cumsum(d.sum(axis=1))
                thr_acc = CUT_THR * C * np.arange(1, R + 1)
                hits = [r for r in range(2, R) if run_sum[r] > thr_acc[r]]
                if hits:
                    cut = True
                    seedrows[hits[0] + 1:] = True
                # pan / fade compensation: subtract last update's mean difference
                dc = np.maximum(0, d - self.g_prev)
            d = np.where(seedrows[:, None], 0, d)
            dp = np.minimum(255, (np.maximum(0, dc - (ctl.floor >> 4)) * GAIN3) >> 2) << 4   # f4 = FLOOR >> 4
            rate16 = RATE_OFF16[self.tid] - (16 if self.one_field else 0)
            e16 = wake_e16(ctl.wake, rate16)
            dec = ((self.M >> (e16 >> 4)) * MANT_DN[e16 & 15]) >> 5
            dec = np.where(self.M > 0, np.maximum(dec, 1), 0)
            self.M = np.maximum(dp, self.M - dec)
            self.ref_y = self.ref_y + (((cy << 2) - self.ref_y) >> ALPHA_SH)
            self.ref_u = self.ref_u + (((cu << 2) - self.ref_u) >> ALPHA_SH)
            self.ref_v = self.ref_v + (((cv << 2) - self.ref_v) >> ALPHA_SH)
            if seedrows.any():
                sr2 = np.repeat(seedrows, 2)
                self.M[seedrows] = 0
                self.ref_y[sr2] = cy[sr2] << 2
                self.ref_u[seedrows] = cu[seedrows] << 2
                self.ref_v[seedrows] = cv[seedrows] << 2
        recip_n = 455 if C == 32 else 607          # round(2^18 / cells): RTL constants
        self.g_prev = ((int(d.sum()) >> 4) * recip_n) >> 14
        if cut:
            self.seed_next = True
            self.mute = True
        return {"d": d, "M": self.M.copy(), "d_sum": int(d.sum()), "seed": seed, "skipped": False,
                "cut": cut, "mute": self.mute, "g": self.g_prev}


# ---------------------------------------------------------------------------
# Renderer + key shaping
# ---------------------------------------------------------------------------
def stir_threshold(stir):
    """v3 curve (footage-tuned): exponential over 8 octaves, STIR 1023 -> 4,
    60% -> 41, 25% -> ~320, bottom ~6% of the throw >= 1024 (nothing keyed).
    v2 (RTL v0.2.x): e16 = ((1023-stir)*11)>>6; thr = ((MANT_UP << i) >> 5) - 1."""
    if STIR_V2:
        e16 = ((1023 - stir) * 11) >> 6
        return min(1024, ((MANT_UP[e16 & 15] << (e16 >> 4)) >> 5) - 1)
    e16 = ((1023 - stir) * 17) >> 7
    return min(1024, (MANT_UP[e16 & 15] << (e16 >> 4)) >> 3)


def softness_params(soft):
    """-> (nearest, k, half): gain = 2^k * (1.5 if half else 1)."""
    if soft < 128:
        return True, 8, 0
    kq = 16 - ((soft - 128) >> 6)
    return False, kq >> 1, kq & 1


def render_matte(M, geo, nearest):
    m10 = (M >> 2).astype(np.int32)
    if nearest:
        return m10[geo.row_y][:, geo.col_x]
    a, b = m10[geo.r0], m10[geo.r1]                              # (H, C)
    v = a + (((b - a) * geo.fy[:, None] + 128) >> 8)             # cell-rate lerp
    step = ((v[:, 1:] - v[:, :-1]) * geo.recip_c) >> 8           # 1/256 px units
    step = np.concatenate([step, np.zeros((v.shape[0], 1), np.int32)], 1)
    acc = (v[:, geo.seg_x] << 8) + 128 + geo.j_x[None, :] * step[:, geo.seg_x]
    return np.clip(acc >> 8, 0, 1023)


def shape_key(m, ctl, gate=None):
    """-> (key after KEY Invert, raw key). gate = DETAIL edge gate, applied to the
    shaped key (min), so DETAIL fades the effect out of flat areas linearly."""
    nearest, k, half = softness_params(ctl.softness)
    x = np.maximum(0, m - stir_threshold(ctl.stir)) << k
    if half:
        x = x + (x >> 1)
    key = np.minimum(1023, x)
    if gate is not None:
        key = np.minimum(key, gate)
    return (1023 - key if ctl.key else key), key


def interp(a, b, t):
    return np.clip(a + (((b - a) * t + 512) >> 10), 0, 1023)


def mix7(a, b, t):
    """v3 mixer (replaces 3 SDK interpolators): t7 = (t + 8) >> 4 in 0..64,
    a + ((b - a) * t7 + 32) >> 6 -- an 11x7 multiply, exactly b at t = 1023."""
    t7 = (t + 8) >> 4
    return np.clip(a + (((b - a) * t7 + 32) >> 6), 0, 1023)


def detail_gate(y10, detail, px_scale=1):
    """px_scale: HW pixels per model pixel (2 when the model renders at 960 wide)."""
    """Edge gate, multiply-free: horizontal gradient |Y[x]-Y[x-2]| << DETAIL_GAIN,
    spread rightward by a peak hold losing DETAIL_DECAY per pixel; gate =
    clamp(energy + (1023 - DETAIL)). DETAIL 0 -> 1023 everywhere (no gating)."""
    g = np.zeros_like(y10)
    g[:, 2:] = np.abs(y10[:, 2:] - y10[:, :-2])
    e = np.minimum(1023, g << DETAIL_GAIN)
    out = e.copy()
    k = 1
    while k * DETAIL_DECAY * px_scale < 1023:
        sh = np.zeros_like(e)
        sh[:, k:] = e[:, :-k] - k * DETAIL_DECAY * px_scale
        out = np.maximum(out, sh)
        k += 1
    return np.clip(out + (1023 - detail), 0, 1023)


def wet_signal(name, yuv):
    y, u, v = yuv[..., 0], yuv[..., 1], yuv[..., 2]
    mid = np.full_like(y, 512)
    if name == "Mono":
        return np.stack([y, mid, mid], -1)
    if name == "Quarter Turn":
        return np.stack([y, np.minimum(1023, 1024 - v), u], -1)
    if name == "Negative":
        return np.stack([np.clip(1004 - y, 0, 1023),
                         np.minimum(1023, 1024 - u), np.minimum(1023, 1024 - v)], -1)
    if name == "Solarize":               # luma folded at mid-grey: black->black, mid->white, white->black
        f = np.minimum(np.maximum(y, 64), np.maximum(0, 1004 - y))
        return np.stack([np.clip(64 + ((f - 64) << 1), 64, 940), u, v], -1)
    if name == "Crush":                  # ~3 bits of luma, 2 bits of chroma (neutral kept)
        yc = np.minimum(940, ((np.maximum(y, 64) - 64) & ~0x7F) + 128)
        return np.stack([yc, np.minimum(1023, (u + 128) & 0x700), np.minimum(1023, (v + 128) & 0x700)], -1)
    if name == "Smear":                  # horizontal sample-and-hold, blocks of W >> 7 px
        n = max(1, yuv.shape[1] >> 7)
        idx = (np.arange(yuv.shape[1]) // n) * n
        return yuv[:, idx]
    raise ValueError(name)


def compose(name, yuv, key, m, raw_key=None):
    if name not in IMPLEMENTED:
        name = "Matte"
    if name == "Matte":                  # mix(black, white, key), as the RTL does
        y = mix7(np.full_like(key, 64), np.full_like(key, 940), key)
        mid = np.full_like(y, 512)
        return np.stack([y, mid, mid], -1)
    if name == "Heat":
        dim = np.stack([64 + ((yuv[..., 0] - 64) >> 1),
                        np.full_like(key, 512), np.full_like(key, 512)], -1)
        # motion diagnostic: palette by matte strength, revealed by the
        # un-inverted key (KEY Invert would paint every still area black)
        if raw_key is not None:
            key = raw_key
        t = np.minimum(1023, key << 2)[..., None]
        return mix7(dim, HEAT[m >> 4], t)
    return mix7(yuv, wet_signal(name, yuv), key[..., None])


# ---------------------------------------------------------------------------
# Synthetic test clip
# ---------------------------------------------------------------------------
T_PAN0, T_PAN1 = 3.0, 5.0
T_FADE0, T_FADE1 = 6.0, 8.0
T_CUT = 8.5
T_BALL_STOP = 10.0
T_END = float(__import__("os").environ.get("WAKE_T_END", 12.0))
SEGMENTS = [("static", 0.5, T_PAN0), ("pan", T_PAN0, T_PAN1),
            ("static2", T_PAN1 + 0.3, T_FADE0), ("fade", T_FADE0, T_FADE1),
            ("pre-cut", T_FADE1 + 0.2, T_CUT), ("cut+0.5s", T_CUT, T_CUT + 0.5),
            ("post-cut", T_CUT + 0.5, T_BALL_STOP), ("ball stopped", T_BALL_STOP, T_END)]


def _box_blur(a, r):
    for ax in (0, 1):
        c = np.cumsum(np.pad(a, [(r + 1, r) if i == ax else (0, 0) for i in range(a.ndim)],
                             mode="edge"), axis=ax)
        a = (np.take(c, range(2 * r + 1, c.shape[ax]), axis=ax)
             - np.take(c, range(0, c.shape[ax] - 2 * r - 1), axis=ax)) / (2 * r + 1)
    return a


def make_background(H, W, seed, hue):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    img = np.empty((H, W, 3), np.float32)
    for c in range(3):
        img[..., c] = (0.38 + 0.14 * np.sin(2 * np.pi * (xx / W * (1.5 + c) + yy / H * (0.7 + hue))
                                             + c * 2.1 + hue * 5))
    s = H / 1080
    for _ in range(70):
        h, w = int(rng.uniform(20, 220) * s), int(rng.uniform(20, 320) * s)
        y0, x0 = rng.integers(0, H - h), rng.integers(0, W - w)
        col = np.clip(rng.uniform(0.05, 0.95, 3) * (0.6 + 0.4 * np.array([hue, 1 - hue, 0.5])), 0, 1)
        img[y0:y0 + h, x0:x0 + w] = 0.3 * img[y0:y0 + h, x0:x0 + w] + 0.7 * col
    tex = _box_blur(rng.standard_normal((H, W)).astype(np.float32), max(1, int(3 * s)))
    img += (0.35 * tex + 0.03 * rng.standard_normal((H, W)))[..., None]
    sy0, sy1 = int(0.06 * H), int(0.22 * H)                 # fine horizontal stripes
    rows = (np.arange(sy0, sy1) // max(1, int(2 * s))) % 2
    img[sy0:sy1, int(0.6 * W):int(0.85 * W)] = (0.2 + 0.6 * rows)[:, None, None]
    img[int(0.70 * H):int(0.82 * H)] = 0.5                 # flat grey band (chroma test)
    return np.clip(img, 0, 1)


def _tri(t, period):
    p = (t / period) % 1.0
    return 2 * p if p < 0.5 else 2 - 2 * p


def same_luma_magenta(matrix, y=0.5):
    kr, kb = KRB[matrix]
    r = b = 0.9
    return np.array([r, (y - kr * r - kb * b) / (1 - kr - kb), b])


class Synth:
    """Static texture + fast ball + slow drift + same-luma chroma patch,
    pan, lighting fade, hard cut, ball stops."""

    def __init__(self, W, H, matrix, seed=1):
        self.W, self.H, self.matrix = W, H, matrix
        self.bgA = make_background(H, int(W * 1.6), seed, 0.2)
        self.bgB = make_background(H, int(W * 1.6), seed + 7, 0.8)
        self.rng = np.random.default_rng(seed + 100)
        self.magenta = same_luma_magenta(matrix)
        yy, xx = np.mgrid[0:H, 0:W]
        self.yy, self.xx = yy.astype(np.float32), xx.astype(np.float32)

    def objects(self, t):
        W, H = self.W, self.H
        tb = min(t, T_BALL_STOP)
        ball = (W * (0.08 + 0.84 * _tri(tb, 3.2)), H * (0.12 + 0.48 * _tri(tb, 2.3)), 0.035 * H)
        drift = (W * (0.22 + 0.025 * t), H * 0.40, 0.085 * W, 0.15 * H)
        patch = (W * (0.05 + 0.75 * _tri(t, 9.0)), 0.12 * W)
        return ball, drift, patch

    def _bbox(self, cx, cy, rx, ry):
        return (slice(max(0, int(cy - ry - 2)), min(self.H, int(cy + ry + 3))),
                slice(max(0, int(cx - rx - 2)), min(self.W, int(cx + rx + 3))))

    def frame(self, t):
        W, H = self.W, self.H
        pan = 0.25 * W * (min(max(t, T_PAN0), T_PAN1) - T_PAN0) if t < T_CUT else 0
        bg = self.bgA if t < T_CUT else self.bgB
        img = bg[:, int(pan):int(pan) + W].copy()
        ball, drift, patch = self.objects(t)
        mask = np.zeros((H, W), bool)
        # slow drift: dark soft ellipse
        cx, cy, rx, ry = drift
        bb = self._bbox(cx, cy, rx, ry)
        e = np.sqrt(((self.xx[bb] - cx) / rx) ** 2 + ((self.yy[bb] - cy) / ry) ** 2)
        al = np.clip((1 - e) * rx / 3, 0, 1)[..., None]
        img[bb] = img[bb] * (1 - al) + np.array([0.12, 0.13, 0.2]) * al
        mask[bb] |= al[..., 0] > 0.5
        # same-luma chroma patch inside the grey band
        x0, w = int(patch[0]), int(patch[1])
        img[int(0.71 * H):int(0.81 * H), x0:x0 + w] = self.magenta
        mask[int(0.71 * H):int(0.81 * H), x0:x0 + w] = True
        patchmask = np.zeros((H, W), bool)
        patchmask[int(0.71 * H):int(0.81 * H), x0:x0 + w] = True
        # fast ball
        bx, by, r = ball
        bb = self._bbox(bx, by, r, r)
        dist = np.sqrt((self.xx[bb] - bx) ** 2 + (self.yy[bb] - by) ** 2)
        al = np.clip(r - dist + 0.5, 0, 1)[..., None]
        img[bb] = img[bb] * (1 - al) + np.array([1.0, 0.85, 0.3]) * al
        ballmask = np.zeros((H, W), bool)
        ballmask[bb] = al[..., 0] > 0.5
        mask |= ballmask
        # lighting fade, flicker, sensor noise
        gain = 1.0
        if T_FADE0 <= t < T_FADE1:
            gain = 1 - 0.5 * _tri(t - T_FADE0, T_FADE1 - T_FADE0)
        img = img * gain + self.rng.normal(0, 0.002)
        img += self.rng.standard_normal(img.shape, dtype=np.float32) * 0.012
        return np.clip(img, 0, 1), mask, {"ball": ballmask, "patch": patchmask}


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------
def box_resize(img, h, w):
    """Area-average resize (float), arbitrary ratio."""
    if img.shape[0] % h == 0 and img.shape[1] % w == 0:
        fy, fx = img.shape[0] // h, img.shape[1] // w
        acc = np.zeros((h, w, img.shape[2]), np.float32)
        for i in range(fy):
            for j in range(fx):
                acc += img[i::fy, j::fx]
        return acc / (fy * fx)
    ys = (np.arange(h) * img.shape[0]) // h
    xs = (np.arange(w) * img.shape[1]) // w
    a = np.add.reduceat(img.astype(np.float64), ys, axis=0)
    a /= np.diff(np.append(ys, img.shape[0]))[:, None, None]
    a = np.add.reduceat(a, xs, axis=1)
    a /= np.diff(np.append(xs, img.shape[1]))[None, :, None]
    return a


def iter_input(args, W, H, matrix):
    """Yields (t, rgb float H x W x 3, object mask or None, ball mask or None)."""
    if args.synthetic:
        syn = Synth(W, H, matrix)
        n = int(round(T_END * args.rate))
        for i in range(n):
            t = i / args.rate
            yield (t,) + syn.frame(t)
        return
    src = Path(args.input)
    if src.is_dir():
        from PIL import Image
        files = sorted(p for p in src.iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp"))
        frames = (np.asarray(Image.open(p).convert("RGB")) for p in files)
    else:
        import imageio.v2 as iio
        frames = iio.get_reader(str(src), "ffmpeg")
    for i, f in enumerate(frames):
        f = np.asarray(f, np.float32) / 255.0
        if f.shape[:2] != (H, W):
            f = box_resize(f, H, W)
        yield i / args.rate, f, None, None


def label(img, text):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, 8 + 7 * len(text), 16], fill=(0, 0, 0))
    d.text((4, 2), text, fill=(255, 255, 255))
    return np.asarray(im)


def event_name(t):
    for name, t0, t1 in SEGMENTS:
        if t0 <= t < t1:
            return name
    return ""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--synthetic", action="store_true", help="generate the synthetic test clip")
    src.add_argument("--input", help="video file or folder of frames")
    ap.add_argument("--format", default="1080p30", choices=FORMATS)
    ap.add_argument("--out", help="output MP4 path (no extension): input | Heat | effect")
    ap.add_argument("--dump", help="per-update NPZ dump (grid, reference, stats)")
    ap.add_argument("--sheet", help="comma-separated times (s) to save as PNG stills")
    ap.add_argument("--preset", help="preset name from ../wake.toml")
    ap.add_argument("--toml", default=str(Path(__file__).resolve().parent.parent / "wake.toml"))
    ap.add_argument("--panel-h", type=int, default=360)
    ap.add_argument("--both-fields", action="store_true",
                    help="interlace: compare every field with the previous (opposite-parity) one")
    for f in Controls.__dataclass_fields__:
        ap.add_argument(f"--{f}", default=None)
    args = ap.parse_args()

    tid, W, H, interlaced, hz, cols, matrix = FORMATS[args.format]
    ctl = load_preset(args.toml, args.preset) if args.preset else Controls()
    for f in Controls.__dataclass_fields__:
        v = getattr(args, f)
        if v is not None:
            if f == "effect" and not v.isdigit():
                v = EFFECTS.index(v)
            setattr(ctl, f, int(v))
    eff = EFFECTS[effect_index(ctl.effect)]
    args.rate = hz                       # updates per second (fields when interlaced)
    print(f"format {args.format} (id {tid}) {'fields' if interlaced else 'frames'} @ {hz} Hz; "
          f"effect {eff}; {ctl}")

    engine = MotionKey(tid, cols, one_field=interlaced and not args.both_fields)
    pw = int(round(args.panel_h * (16 / 9 if W > 720 else 4 / 3)))
    writer = None
    if args.out:
        import imageio_ffmpeg
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        writer = imageio_ffmpeg.write_frames(args.out + ".mp4", (3 * pw, args.panel_h), fps=hz,
                                             codec="libx264", pix_fmt_out="yuv420p", quality=8,
                                             macro_block_size=4, ffmpeg_log_level="error")
        writer.send(None)
    sheet_t = sorted(float(s) for s in args.sheet.split(",")) if args.sheet else []
    dumps = {k: [] for k in ("acc", "cur", "d", "M", "d_sum", "seed", "skipped")}
    seg_stats = {name: [] for name, _, _ in SEGMENTS}
    prev_M = None
    ball_keyed_until = None

    for i, (t, rgb, mask, ballmask) in enumerate(iter_input(args, W, H, matrix)):
        if interlaced:                    # emulate interlace: alternate field lines
            sl = slice(i & 1, None, 2)
            rgb = rgb[sl]
            mask = mask[sl] if mask is not None else None
            ballmask = {k: v[sl] for k, v in ballmask.items()} if ballmask is not None else None
        yuv = rgb_to_yuv10(rgb, matrix)
        # render with the grid from the previous update (position-locked, 1-update lag)
        if prev_M is None:
            m = np.zeros(yuv.shape[:2], np.int32)
            key = np.zeros(yuv.shape[:2], np.int32)
            raw_key = key.copy()
            if ctl.key:
                key[:] = 1023
        else:
            nearest = softness_params(ctl.softness)[0]
            m = render_matte(prev_M, engine.geo, nearest)
            key, raw_key = shape_key(m, ctl)
        out = engine.update(yuv, ctl, field=i & 1)
        prev_M = out["M"]
        for k in dumps:
            dumps[k].append(out[k])

        if mask is not None:              # tuning metrics: key on objects vs far background
            geo = engine.geo
            cellmask = np.add.reduceat(np.add.reduceat(mask.astype(np.int64), geo.col_starts, 1),
                                       geo.row_starts, 0) > 0
            near = cellmask.copy()        # dilate by one cell
            near[1:] |= cellmask[:-1]; near[:-1] |= cellmask[1:]
            near[:, 1:] |= near[:, :-1].copy(); near[:, :-1] |= near[:, 1:].copy()
            far = ~near[geo.row_y][:, geo.col_x]
            kf = key / 1023.0
            spill = kf[far].mean() if far.any() else 0
            for name, t0, t1 in SEGMENTS:
                if t0 <= t < t1:
                    seg_stats[name].append((kf[ballmask["ball"]].mean(),
                                            kf[ballmask["patch"]].mean(), spill))
            if t >= T_BALL_STOP and kf[ballmask["ball"]].mean() > 0.1:
                ball_keyed_until = t

        if writer or sheet_t:
            ph = args.panel_h
            heat = compose("Heat", yuv, key, m, raw_key)
            res = compose(eff, yuv, key, m, raw_key)
            panels = []
            for img, name in ((yuv, f"input  t={t:5.2f}s {event_name(t)}"), (heat, "Heat"),
                              (res, eff + (" (inverted key)" if ctl.key else ""))):
                if interlaced:
                    img = np.repeat(img, 2, axis=0)
                small = np.rint(box_resize(img, ph, pw)).astype(np.int64)
                panels.append(label(yuv10_to_rgb8(small, matrix), name))
            strip = np.concatenate(panels, 1)
            if writer:
                writer.send(np.ascontiguousarray(strip))
            while sheet_t and t >= sheet_t[0]:
                from PIL import Image
                p = Path((args.out or "sheet") + f"_t{sheet_t.pop(0):05.2f}.png")
                Image.fromarray(strip).save(p)
        if i % 60 == 0:
            m = out["M"] >> 4
            print(f"  t={t:6.2f}s  d_sum={out['d_sum']:6d}  M_int max={m.max():3d} "
                  f"cells>0={int((m > 0).sum()):3d}", flush=True)

    if writer:
        writer.close()
        print(f"wrote {args.out}.mp4")
    if args.dump:
        np.savez_compressed(args.dump, **{k: np.array(v) for k, v in dumps.items()},
                            geometry=np.array([engine.geo.W, engine.geo.H, engine.geo.C, engine.geo.R,
                                               engine.geo.sh, engine.geo.recip, engine.geo.recip_c,
                                               engine.geo.recip_r]))
        print(f"wrote {args.dump}")
    print("geometry:", engine.geo.describe())
    print(f"STIR thr={stir_threshold(ctl.stir)}  softness={softness_params(ctl.softness)}  "
          f"floor={ctl.floor >> 5}  e16={wake_e16(ctl.wake, RATE_OFF16[tid] - (16 if engine.one_field else 0))}")
    if args.synthetic:
        print(f"{'segment':14s} {'ball':>6s} {'patch':>6s} {'spill':>6s}   (mean key 0..1; spill = cells"
              " >1 cell from any object, includes wake trails)")
        for name, _, _ in SEGMENTS:
            s = np.array(seg_stats[name])
            if len(s):
                print(f"{name:14s} {s[:, 0].mean():6.3f} {s[:, 1].mean():6.3f} {s[:, 2].mean():6.3f}")
        if ball_keyed_until is not None:
            print(f"ball keyed (>10%) until {ball_keyed_until - T_BALL_STOP:.2f} s after it stopped")
        else:
            print("ball not keyed after it stopped")


if __name__ == "__main__":
    main()
