#!/usr/bin/env python3
"""
PULP look-development simulator.

Renders incoming video as a 1965 four-color newspaper press: clustered-dot
halftone screens at the classic rotated angles, dot gain, ink-over-paper
compositing, misregistration, plus the comic layer (posterize flats, Sobel
ink linework, panels, speech bubbles) and the AGED bad-print-run defects.

The math is written to mirror the planned VHDL:
  * Screen coordinates come from a DDA (per-pixel constant increments of
    f*cos/f*sin) -- here vectorized, but identical values.
  * Spot function: dc = metric(cell center), dk = metric(nearest corner),
    threshold T = 0.5 + (dc - dk)/2.  Symmetric: black dots grow from cell
    centers in highlights, checkerboard at exactly 50%, white dots on solid
    ink in shadows.  The metric is a morphable p-norm family:
    line -> diamond (L1) -> round (L2) -> elliptical -> square (Linf).
  * Dot gain: t' = t + gain * min(t, 1-t)  (triangle bump -- shift/add).
  * Ink over paper: multiplicative transmittance per ink layer.

Controls are taken in raw 0..1023 panel units, same mapping the VHDL will use.

Usage:  python3 pulp_sim.py [--input img.png] [--outdir .] [--suite]
"""

import argparse
import os
import numpy as np
from PIL import Image

# ----------------------------------------------------------------------------
# constants: the press
# ----------------------------------------------------------------------------

# classic process angles (degrees)
ANGLES = {"c": 15.0, "m": 75.0, "y": 0.0, "k": 45.0}

# ink transmittance triplets (what fraction of paper R,G,B survives the ink).
# deliberately impure -- real newsprint inks are muddy.
INK_TRANS = {
    "c": np.array([0.13, 0.55, 0.85], dtype=np.float32),
    "m": np.array([0.87, 0.12, 0.45], dtype=np.float32),
    "y": np.array([0.95, 0.86, 0.14], dtype=np.float32),
    "k": np.array([0.16, 0.15, 0.145], dtype=np.float32),  # warm dark gray
}

# paper stocks along the PAPER knob: bright white -> warm cream -> aged pulp
PAPER_WHITE = np.array([0.97, 0.965, 0.945], dtype=np.float32)
PAPER_CREAM = np.array([0.945, 0.905, 0.815], dtype=np.float32)
PAPER_AGED  = np.array([0.870, 0.780, 0.560], dtype=np.float32)

# misregistration unit directions per plate (K is the reference, stays put)
MISREG_DIR = {
    "c": (-0.8, +0.45),
    "m": (+0.9, -0.35),
    "y": (+0.35, +0.9),
    "k": (0.0, 0.0),
}


def _hash2(x, y, seed):
    """Integer lattice hash -> [0,1).  Stand-in for the VHDL LFSR/hash."""
    h = (x.astype(np.uint32) * np.uint32(0x9E3779B1)
         ^ y.astype(np.uint32) * np.uint32(0x85EBCA77)
         ^ np.uint32(seed))
    h ^= h >> np.uint32(13)
    h = h * np.uint32(0xC2B2AE35)
    h ^= h >> np.uint32(16)
    return (h & np.uint32(0xFFFF)).astype(np.float32) / 65535.0


# ----------------------------------------------------------------------------
# spot function
# ----------------------------------------------------------------------------

def _metric(ax, ay, shape):
    """Morphable dot metric, normalized so cell corner ~= 1.0.

    shape in [0,1]:  0.00 line | 0.25 diamond | 0.50 round |
                     0.75 elliptical | 1.00 square
    Built from the three cheap norms (L1, alpha-max-beta-min L2, Linf)
    exactly as the VHDL will.
    """
    def l2(px, py):
        mx = np.maximum(px, py)
        mn = np.minimum(px, py)
        # max-of-two-planes euclid approx, +-1.7% ripple, shift-add exact
        return np.maximum(mx + 0.125 * mn,
                          0.8125 * mx + 0.59375 * mn) * 1.4142

    m_line = 2.0 * ay
    m_diam = ax + ay
    m_l2 = l2(ax, ay)
    m_sq = 2.0 * np.maximum(ax, ay)
    # elliptical: L2 with the u axis squashed (dot stretches along v)
    m_ell = l2(np.minimum(ax * 1.5, 0.75), ay) * 0.943

    s = np.clip(shape, 0.0, 1.0) * 4.0
    seg = int(np.floor(min(s, 3.999)))
    f = s - seg
    lo = [m_line, m_diam, m_l2, m_ell][seg]
    hi = [m_diam, m_l2, m_ell, m_sq][seg]
    return lo * (1.0 - f) + hi * f


def spot_threshold(fu, fv, shape):
    """Screen threshold T in [0,1] from in-cell fractional coords."""
    du = np.abs(fu - 0.5)
    dv = np.abs(fv - 0.5)
    if shape < 0.25:
        # the line-screen end ignores u for the corner distance too
        pass
    dc = _metric(du, dv, shape)                    # distance-ish from center
    dk = _metric(0.5 - du, 0.5 - dv, shape)        # from nearest corner
    return np.clip(0.5 + 0.5 * (dc - dk), 0.0, 1.0)


def screen_coords(w, h, angle_deg, cell_px, phase=(0.0, 0.0), interlace_y=None):
    """Rotated screen-space cell coordinates via the same increments the
    DDA accumulators will use."""
    a = np.deg2rad(angle_deg)
    f = 1.0 / max(cell_px, 1.0)
    ca, sa = np.cos(a) * f, np.sin(a) * f
    xs = np.arange(w, dtype=np.float32)
    ys = np.arange(h, dtype=np.float32) if interlace_y is None else interlace_y
    u = xs[None, :] * ca + ys[:, None] * sa + phase[0]
    v = -xs[None, :] * sa + ys[:, None] * ca + phase[1]
    return u, v


# ----------------------------------------------------------------------------
# tone pipeline
# ----------------------------------------------------------------------------

def dot_gain(t, gain):
    """Midtone spread: triangle bump, matches the VHDL shift-add."""
    return np.clip(t + gain * np.minimum(t, 1.0 - t), 0.0, 1.0)


def posterize_tone(t, amount, levels=4):
    """Blend toward flat comic levels; amount in [0,1]."""
    if amount <= 0.0:
        return t
    q = np.round(t * (levels - 1)) / (levels - 1)
    return t * (1.0 - amount) + q * amount


def rgb_to_inks(rgb, gcr=0.75):
    """RGB -> CMY(K) with gray-component replacement."""
    cmy = 1.0 - rgb
    k = np.min(cmy, axis=-1) * gcr
    c = np.clip(cmy[..., 0] - k, 0.0, 1.0)
    m = np.clip(cmy[..., 1] - k, 0.0, 1.0)
    y = np.clip(cmy[..., 2] - k, 0.0, 1.0)
    return {"c": c, "m": m, "y": y, "k": k}


# ----------------------------------------------------------------------------
# controls
# ----------------------------------------------------------------------------

class Controls:
    """Raw panel values (0..1023 knobs/slider, 0/1 switches)."""

    def __init__(self, **kw):
        self.k1_screen = kw.get("k1_screen", 512)
        self.k2_shape = kw.get("k2_shape", 512)
        self.k3_ink = kw.get("k3_ink", 512)
        self.k4_paper = kw.get("k4_paper", 512)
        self.k5_linework = kw.get("k5_linework", 0)
        self.k6_register = kw.get("k6_register", 128)
        self.s7_color = kw.get("s7_color", 1)          # 0 B&W, 1 CMYK
        self.s8_posterize = kw.get("s8_posterize", 0)
        self.s9_panels = kw.get("s9_panels", 0)
        self.s10_bubbles = kw.get("s10_bubbles", 0)
        self.s11_aged = kw.get("s11_aged", 0)
        self.p12_press = kw.get("p12_press", 512)


def resolve(c, height=1080):
    """Panel units -> effect parameters, including the PRESS macro curve.

    Cell sizes are normalized to output height so the look matches at 1080p.
    """
    r = {}
    unit = height / 1080.0
    press = c.p12_press / 1023.0

    # K1: base cell size, exponential 8px .. 96px (at 1080p)
    k1 = c.k1_screen / 1023.0
    base_cell = 8.0 * (12.0 ** k1)

    # PRESS overrides/scales SCREEN: bottom third sweeps from invisible-fine
    # up to the knob setting; top third coarsens on toward Lichtenstein.
    if press < 0.35:
        g = press / 0.35
        cell = 2.6 * (base_cell / 2.6) ** g
    elif press < 0.70:
        cell = base_cell
    else:
        g = (press - 0.70) / 0.30
        cell = base_cell * (4.5 ** g)
    r["cell"] = float(np.clip(cell * unit, 2.4, 420.0))

    r["shape"] = c.k2_shape / 1023.0

    # K3 + PRESS: ink density -> threshold bias and dot gain.
    # Past press ~0.7 gain RELAXES: pop-art wants bold flats, not plugged
    # shadows (heavy gain at giant cells turns the page to mud).
    ink = c.k3_ink / 1023.0
    ink_eff = np.clip(ink + 0.12 * max(0.0, press - 0.60), 0.0, 1.05)
    r["bias"] = (ink_eff - 0.5) * 0.38
    gain = np.clip(0.08 + 0.45 * ink_eff, 0.0, 0.62)
    if press > 0.7:
        gain *= 1.0 - 0.6 * (press - 0.7) / 0.3
    r["gain"] = gain

    # K4: paper stock + grain
    p = c.k4_paper / 1023.0
    if p < 0.5:
        paper = PAPER_WHITE * (1 - 2 * p) + PAPER_CREAM * (2 * p)
    else:
        paper = PAPER_CREAM * (2 - 2 * p) + PAPER_AGED * (2 * p - 1)
    r["paper"] = paper.astype(np.float32)
    r["grain"] = 0.015 + 0.075 * p

    # K5: linework weight. 0 = off; then threshold eases + dilate 0..3
    lw = c.k5_linework / 1023.0
    r["line_on"] = lw > 0.04
    r["line_thresh"] = 0.42 - 0.30 * lw
    r["line_dilate"] = int(np.clip(np.floor(lw * 3.6), 0, 3))

    # K6: misregistration in pixels (drunk pressman ~ 9 px at 1080p)
    r["misreg"] = (c.k6_register / 1023.0) ** 1.5 * 9.0 * unit

    # posterize depth: switch forces flats; PRESS fades them in anyway
    pz = 1.0 if c.s8_posterize else np.clip((press - 0.45) / 0.35, 0.0, 1.0)
    r["poster"] = pz
    r["poster_levels"] = 4 if c.s7_color else 3

    # ragged dot edges: always some; more when aged.  Amplitude shrinks as
    # cells grow so perimeter raggedness stays ~1-2 px at any dot size.
    r["edge_noise"] = (0.05 + (0.06 if c.s11_aged else 0.0)) \
        * float(np.clip(14.0 / r["cell"], 0.12, 1.0))

    # at the very bottom of the slider, crossfade toward the dry video
    r["dry_mix"] = np.clip(1.0 - press / 0.16, 0.0, 1.0)

    r["color"] = bool(c.s7_color)
    r["panels"] = bool(c.s9_panels)
    r["bubbles"] = bool(c.s10_bubbles)
    r["aged"] = bool(c.s11_aged)
    r["press"] = press
    return r


# ----------------------------------------------------------------------------
# layout department: panels
# ----------------------------------------------------------------------------

PANEL_TEMPLATES = [
    # each: list of (x0,y0,x1,y1) in unit coords -- matches the 8 VHDL
    # layouts (rolled from the LFSR each time PANELS flips on).  One
    # column split per row, which is all comic grammar needs.
    [(x/2, y/3, (x+1)/2, (y+1)/3) for y in range(3) for x in range(2)],  # 6 = 3 tiers of 2
    [(0.0, 0.0, 3/8, 1.0), (3/8, 0.0, 1.0, 1/3),
     (3/8, 1/3, 1.0, 2/3), (3/8, 2/3, 1.0, 1.0)],            # tall left + 3
    [(x/2, y/2, (x+1)/2, (y+1)/2) for y in range(2) for x in range(2)],  # 2x2
    [(0.0, 0.0, 1.0, 1/3), (0.0, 1/3, 1.0, 2/3), (0.0, 2/3, 1.0, 1.0)],  # strips
    [(0.0, 0.0, 1.0, 0.5), (0.0, 0.5, 0.5, 1.0), (0.5, 0.5, 1.0, 1.0)],  # splash+2
    [(0.0, 0.0, 0.5, 0.5), (0.5, 0.0, 1.0, 0.5), (0.0, 0.5, 1.0, 1.0)],  # 2+splash
    [(0.0, 0.0, 3/8, 0.5), (3/8, 0.0, 1.0, 0.5),
     (0.0, 0.5, 5/8, 1.0), (5/8, 0.5, 1.0, 1.0)],            # staggered 2x2
    [(0.0, 0.0, 1.0, 1/3), (0.0, 1/3, 0.5, 2/3), (0.5, 1/3, 1.0, 2/3),
     (0.0, 2/3, 0.5, 1.0), (0.5, 2/3, 1.0, 1.0)],            # strip + 2x2
]


def panel_maps(w, h, template_idx, border_px=6, gutter_px=16):
    """Returns (panel_index_map int, border_mask, gutter_mask)."""
    idx = np.full((h, w), -1, dtype=np.int32)
    border = np.zeros((h, w), dtype=bool)
    xs = np.arange(w)[None, :]
    ys = np.arange(h)[:, None]
    g = gutter_px // 2
    for i, (x0, y0, x1, y1) in enumerate(PANEL_TEMPLATES[template_idx]):
        px0 = int(x0 * w) + (g if x0 > 0 else 0)
        px1 = int(x1 * w) - (g if x1 < 1 else 0)
        py0 = int(y0 * h) + (g if y0 > 0 else 0)
        py1 = int(y1 * h) - (g if y1 < 1 else 0)
        inside = (xs >= px0) & (xs < px1) & (ys >= py0) & (ys < py1)
        core = (xs >= px0 + border_px) & (xs < px1 - border_px) & \
               (ys >= py0 + border_px) & (ys < py1 - border_px)
        border |= inside & ~core
        idx[core] = i
    gutter = (idx < 0) & ~border
    return idx, border, gutter


# ----------------------------------------------------------------------------
# layout department: bubbles
# ----------------------------------------------------------------------------

# tiny grawlix glyph ROM, 8x8, matches the eventual VHDL ROM
GLYPHS = {
    "!": ["00111000", "00111000", "00111000", "00110000", "00110000",
          "00000000", "00110000", "00110000"],
    "?": ["01111100", "11000110", "00000110", "00011100", "00110000",
          "00000000", "00110000", "00110000"],
    "#": ["00100100", "11111111", "01001000", "01001000", "10010010",
          "11111111", "00100100", "00100100"],
    "*": ["00010000", "10010010", "01010100", "00111000", "01010100",
          "10010010", "00010000", "00000000"],
    "$": ["00010000", "01111100", "10010000", "01111000", "00010100",
          "11111000", "00010000", "00000000"],
    "@": ["01111100", "10000010", "10111010", "10101010", "10111100",
          "10000000", "01111100", "00000000"],
}


def _glyph_mask(ch):
    return np.array([[c == "1" for c in row] for row in GLYPHS[ch]], dtype=bool)


def draw_bubble(w, h, kind, cx, cy, bw, bh, tail_dir, text, seed=0):
    """Returns (fill_mask, outline_mask, text_mask) for one bubble."""
    xs = np.arange(w, dtype=np.float32)[None, :]
    ys = np.arange(h, dtype=np.float32)[:, None]
    dx = (xs - cx) / bw
    dy = (ys - cy) / bh
    out = np.zeros((h, w), dtype=bool)
    fill = np.zeros((h, w), dtype=bool)

    if kind == "speech":
        # superellipse p=4: rounded rectangle
        m = dx ** 4 + dy ** 4
        fill = m <= 1.0
        out = (m <= 1.28) & ~fill
        # tail: triangle
        tx, ty = tail_dir
        t0 = np.array([cx + tx * bw * 0.35, cy + ty * bh * 0.8])
        t1 = np.array([cx + tx * bw * 0.72, cy + ty * bh * 0.75])
        tp = np.array([cx + tx * bw * 1.15, cy + ty * bh * 2.1])
        tri = _tri_mask(xs, ys, t0, t1, tp)
        tri_o = _tri_mask(xs, ys, t0, t1, tp, grow=3.5)
        out |= tri_o & ~tri & ~fill
        fill |= tri
        out &= ~fill
    elif kind == "thought":
        m = np.minimum.reduce([
            ((xs - (cx - bw * 0.55)) / (bw * 0.55)) ** 2 + ((ys - cy) / (bh * 0.8)) ** 2,
            ((xs - (cx + bw * 0.45)) / (bw * 0.6)) ** 2 + ((ys - (cy - bh * 0.15)) / (bh * 0.85)) ** 2,
            ((xs - cx) / (bw * 0.7)) ** 2 + ((ys - (cy + bh * 0.2)) / (bh * 0.75)) ** 2,
        ])
        fill = m <= 1.0
        out = (m <= 1.35) & ~fill
        tx, ty = tail_dir
        for i, r in [(1.6, 0.13), (2.15, 0.08)]:
            bx, by = cx + tx * bw * i, cy + ty * bh * i
            d = ((xs - bx) / (bw * r)) ** 2 + ((ys - by) / (bh * r * 1.2)) ** 2
            fill |= d <= 1.0
            out |= (d <= 1.6) & (d > 1.0)
        out &= ~fill
    else:  # starburst
        ang = np.arctan2(ys - cy, xs - cx)
        rr = np.sqrt(((xs - cx) / bw) ** 2 + ((ys - cy) / bh) ** 2)
        spikes = 11
        saw = np.abs(((ang * spikes / (2 * np.pi)) % 1.0) - 0.5) * 2.0
        rlim = 0.55 + 0.45 * saw
        fill = rr <= rlim
        out = (rr <= rlim + 0.09) & ~fill

    # text: grawlix glyphs, scaled x4, centered
    tmask = np.zeros((h, w), dtype=bool)
    scale = max(2, int(bh / 14))
    tw = len(text) * 9 * scale
    ox = int(cx - tw / 2)
    oy = int(cy - 4 * scale)
    for ci, ch in enumerate(text):
        if ch not in GLYPHS:
            continue
        g = _glyph_mask(ch)
        g = np.kron(g, np.ones((scale, scale), dtype=bool))
        gy, gx = g.shape
        x0, y0 = ox + ci * 9 * scale, oy
        if 0 <= y0 and y0 + gy < h and 0 <= x0 and x0 + gx < w:
            tmask[y0:y0 + gy, x0:x0 + gx] |= g
    tmask &= fill
    return fill, out, tmask


def _tri_mask(xs, ys, a, b, p, grow=0.0):
    def half(p0, p1):
        ex, ey = p1[0] - p0[0], p1[1] - p0[1]
        n = max(np.hypot(ex, ey), 1e-6)
        return ((xs - p0[0]) * ey - (ys - p0[1]) * ex) / n
    d1, d2, d3 = half(a, b), half(b, p), half(p, a)
    s = np.sign(half(a, b)[int(p[1]), int(np.clip(p[0], 0, xs.shape[1] - 1))]
                if False else 1.0)
    lo = -grow
    inside = ((d1 >= lo) & (d2 >= lo) & (d3 >= lo)) | \
             ((d1 <= -lo) & (d2 <= -lo) & (d3 <= -lo))
    return inside


# ----------------------------------------------------------------------------
# the press
# ----------------------------------------------------------------------------

def render(rgb_in, c, seed=7, drift_t=0.0):
    """Full PULP render.  rgb_in float32 HxWx3 in [0,1]."""
    h, w, _ = rgb_in.shape
    r = resolve(c, height=h)
    rng = np.random.default_rng(seed)

    xs32 = np.arange(w, dtype=np.uint32)[None, :]
    ys32 = np.arange(h, dtype=np.uint32)[:, None]

    # ---- panels ------------------------------------------------------
    if r["panels"]:
        tpl = seed % len(PANEL_TEMPLATES)
        pidx, pborder, pgutter = panel_maps(
            w, h, tpl, border_px=max(4, int(6 * h / 1080)),
            gutter_px=max(10, int(16 * h / 1080)))
    else:
        pidx = np.zeros((h, w), dtype=np.int32)
        pborder = np.zeros((h, w), dtype=bool)
        pgutter = np.zeros((h, w), dtype=bool)

    # ---- tone separation ----------------------------------------------
    luma = (0.299 * rgb_in[..., 0] + 0.587 * rgb_in[..., 1]
            + 0.114 * rgb_in[..., 2]).astype(np.float32)

    if r["color"]:
        inks = rgb_to_inks(rgb_in)
        plates = ["y", "c", "m", "k"]        # print order: Y first, K last
    else:
        inks = {"k": 1.0 - luma}
        plates = ["k"]

    # ---- aged press artifacts (tone-side) -------------------------------
    streaks = None
    if r["aged"]:
        col = _hash2(xs32 // np.uint32(96), np.zeros_like(xs32), seed + 40)
        col2 = _hash2(xs32 // np.uint32(23), np.zeros_like(xs32), seed + 41)
        streaks = np.clip((col - 0.62) * 1.6, 0, 1) * (0.25 + 0.3 * col2)
        band = 0.06 * np.sin(ys32.astype(np.float32) * 0.055
                             + 40.0 * _hash2(np.uint32(seed) * np.ones((1, 1), np.uint32),
                                             np.zeros((1, 1), np.uint32), 3))
        streaks = (streaks + np.maximum(band, 0)).astype(np.float32)

    # ---- paper ----------------------------------------------------------
    grain = (_hash2(xs32 // np.uint32(3), ys32 // np.uint32(3), seed + 9) - 0.5) \
        + 0.5 * (_hash2(xs32, ys32, seed + 10) - 0.5)
    paper = r["paper"][None, None, :] * (1.0 + r["grain"] * grain)[..., None]
    out = paper.copy()

    # ---- screen + composite each plate ---------------------------------
    edge_n = (_hash2(xs32, ys32, seed + 21) - 0.5) * 2.0 * r["edge_noise"]
    per_panel_phase = [(_hash2(np.uint32(i + 3) * np.ones((1, 1), np.uint32),
                               np.uint32(seed) * np.ones((1, 1), np.uint32), 5)[0, 0],
                        _hash2(np.uint32(i + 9) * np.ones((1, 1), np.uint32),
                               np.uint32(seed) * np.ones((1, 1), np.uint32), 6)[0, 0])
                       for i in range(4)]

    for p in plates:
        t = inks[p] if r["color"] else inks["k"]
        t = posterize_tone(t, r["poster"], r["poster_levels"])
        t = np.clip(t + r["bias"], 0.0, 1.0)
        t = dot_gain(t, r["gain"])
        if streaks is not None:
            t = np.clip(t - streaks, 0.0, 1.0)

        mis = r["misreg"]
        dxp = MISREG_DIR[p][0] * mis + 0.8 * np.sin(drift_t * 2.1 + hash(p) % 7)
        dyp = MISREG_DIR[p][1] * mis
        if mis < 0.3:
            dxp = dyp = 0.0

        # channel sampling offset (the cheap horizontal part in HW)
        if abs(dxp) >= 1.0:
            t = np.roll(t, int(round(dxp)), axis=1)

        ang = ANGLES[p] if r["color"] else 45.0
        uu, vv = screen_coords(w, h, ang, r["cell"],
                               phase=(dxp / r["cell"], dyp / r["cell"]))
        if r["panels"]:
            # per-panel screen phase re-seed (reads as separately printed)
            pu = np.take(np.array([q[0] * 7.37 for q in per_panel_phase],
                                  np.float32), np.clip(pidx, 0, 3))
            pv = np.take(np.array([q[1] * 5.91 for q in per_panel_phase],
                                  np.float32), np.clip(pidx, 0, 3))
            uu = uu + pu
            vv = vv + pv

        iu = np.floor(uu)
        iv = np.floor(vv)
        T = spot_threshold(uu - iu, vv - iv, r["shape"])
        ink_on = t > (T + edge_n)

        if r["aged"]:
            # random dot dropouts on the rotated screen-cell lattice
            drop = _hash2((iu.astype(np.int64) & 0xFFFF).astype(np.uint32),
                          (iv.astype(np.int64) & 0xFFFF).astype(np.uint32),
                          seed + 60 + ord(p)) > 0.90
            ink_on &= ~drop

        trans = INK_TRANS[p][None, None, :]
        if r["aged"]:
            trans = trans + (1.0 - trans) * 0.18   # faded ink
        out = np.where(ink_on[..., None], out * trans, out)

    # ---- comic linework -------------------------------------------------
    if r["line_on"]:
        gx = np.zeros_like(luma)
        gy = np.zeros_like(luma)
        gx[:, 1:-1] = luma[:, 2:] - luma[:, :-2]
        gy[1:-1, :] = luma[2:, :] - luma[:-2, :]
        edge = np.abs(gx) + np.abs(gy)
        emask = edge > r["line_thresh"]
        for _ in range(r["line_dilate"]):
            emask[1:, :] |= emask[:-1, :]
            emask[:, 1:] |= emask[:, :-1]
        ink_rgb = (r["paper"] * INK_TRANS["k"] * INK_TRANS["k"])[None, None, :]
        out = np.where(emask[..., None], ink_rgb, out)

    # ---- panels overlay --------------------------------------------------
    if r["panels"]:
        out = np.where(pgutter[..., None], paper, out)
        out = np.where(pborder[..., None],
                       (r["paper"] * INK_TRANS["k"] * INK_TRANS["k"])[None, None, :],
                       out)

    # ---- bubbles ----------------------------------------------------------
    if r["bubbles"]:
        kinds = ["speech", "thought", "speech", "burst"]
        kind = kinds[seed % 4]
        texts = ["!", "?", "#$@*", "!?", "*!"]
        bw_ = w * 0.11
        bh_ = h * 0.10
        cx = w * (0.28 + 0.44 * _hash2(np.uint32(seed) * np.ones((1, 1), np.uint32),
                                       np.ones((1, 1), np.uint32), 71)[0, 0])
        cy = h * (0.22 + 0.35 * _hash2(np.uint32(seed) * np.ones((1, 1), np.uint32),
                                       np.ones((1, 1), np.uint32), 72)[0, 0])
        tail = [(-0.6, 1.0), (0.7, 1.0)][seed % 2]
        fill, outl, tmask = draw_bubble(w, h, kind, cx, cy, bw_, bh_,
                                        tail, texts[seed % 5])
        ink_rgb = (r["paper"] * INK_TRANS["k"] * INK_TRANS["k"])[None, None, :]
        out = np.where(fill[..., None], paper * 1.03, out)
        out = np.where((outl | tmask)[..., None], ink_rgb, out)

    # ---- dry crossfade at slider bottom ---------------------------------
    if r["dry_mix"] > 0.0:
        out = out * (1.0 - r["dry_mix"]) + rgb_in * r["dry_mix"]

    return np.clip(out, 0.0, 1.0)


# ----------------------------------------------------------------------------
# suite
# ----------------------------------------------------------------------------

def _load(path):
    img = Image.open(path).convert("RGB")
    return np.asarray(img, dtype=np.float32) / 255.0


def testcard(w=1920, h=1080):
    """Photographic-ish test card: shaded sphere (face proxy), skin/hair
    tones, smooth ramps -- for judging tonal reproduction, not shapes."""
    xs = np.linspace(0, 1, w, dtype=np.float32)[None, :]
    ys = np.linspace(0, 1, h, dtype=np.float32)[:, None]
    img = np.zeros((h, w, 3), dtype=np.float32)
    # background: soft diagonal studio gradient
    bg = 0.25 + 0.5 * (0.6 * xs + 0.4 * ys)
    img[:] = bg[..., None] * np.array([0.72, 0.75, 0.82], np.float32)
    # horizontal gray ramp strip (bottom)
    strip = ys > 0.86
    img = np.where(strip[..., None], (xs * np.ones_like(ys))[..., None], img)
    # big shaded sphere, skin-toned (the "face")
    cx, cy, rad = 0.40, 0.44, 0.30
    dx = (xs - cx) * (w / h)
    dy = ys - cy
    rr = np.sqrt(dx * dx + dy * dy) / rad
    inside = rr < 1.0
    nz = np.sqrt(np.clip(1 - rr * rr, 0, 1))
    lite = np.clip(0.15 + 0.9 * (0.55 * -dx / rad + 0.35 * -dy / rad + 0.75 * nz), 0, 1.15)
    skin = np.array([0.86, 0.62, 0.50], np.float32)
    img = np.where(inside[..., None], lite[..., None] * skin[None, None, :], img)
    # dark "hair" crescent
    hair = (rr < 1.05) & (rr > 0.72) & (dy < -0.05)
    img = np.where(hair[..., None], (0.12 * lite)[..., None] *
                   np.array([1.0, 0.85, 0.7], np.float32), img)
    # color patches (right side)
    patches = [(0.86, 0.10, (0.75, 0.15, 0.12)), (0.86, 0.26, (0.15, 0.55, 0.2)),
               (0.86, 0.42, (0.15, 0.25, 0.7)), (0.86, 0.58, (0.9, 0.75, 0.15))]
    for px, py, col in patches:
        m = (np.abs(xs - px) < 0.055) & (np.abs(ys - py) < 0.07)
        img = np.where(m[..., None], np.array(col, np.float32), img)
    return np.clip(img, 0, 1)


def _save_crop(arr, path, zoom=2):
    h, w, _ = arr.shape
    c = arr[h // 2 - 160:h // 2 + 110, w // 2 - 240:w // 2 + 240]
    c8 = (np.clip(c, 0, 1) * 255).astype(np.uint8)
    im = Image.fromarray(c8).resize((c.shape[1] * zoom, c.shape[0] * zoom),
                                    Image.NEAREST)
    im.save(path)
    print("wrote", path)


def _save(arr, path):
    Image.fromarray((np.clip(arr, 0, 1) * 255).astype(np.uint8)).save(path)
    print("wrote", path)


SUITE = [
    ("001_bw_coarse", Controls(s7_color=0, k1_screen=650, k2_shape=512,
                               k3_ink=512, k4_paper=600, p12_press=560)),
    ("002_cmyk_fine", Controls(s7_color=1, k1_screen=120, k2_shape=512,
                               k3_ink=480, k4_paper=520, k6_register=200,
                               p12_press=560)),
    ("003_popart", Controls(s7_color=1, k1_screen=700, k2_shape=512,
                            k3_ink=560, k4_paper=380, k6_register=280,
                            p12_press=1023)),
    ("004_comic", Controls(s7_color=1, k1_screen=300, k2_shape=512,
                           k3_ink=500, k4_paper=560, k5_linework=520,
                           k6_register=180, s8_posterize=1, s9_panels=1,
                           s10_bubbles=1, p12_press=560)),
    ("005_aged_sunday", Controls(s7_color=1, k1_screen=420, k2_shape=512,
                                 k3_ink=560, k4_paper=880, k6_register=420,
                                 s11_aged=1, p12_press=560)),
    ("006_line_screen", Controls(s7_color=0, k1_screen=380, k2_shape=0,
                                 k3_ink=512, k4_paper=640, p12_press=560)),
    ("007_slider_low", Controls(s7_color=1, k1_screen=512, p12_press=120)),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", default=os.path.join(
        os.path.dirname(__file__), "..", "turpentine", "sim_000_input.png"))
    ap.add_argument("--outdir", default=os.path.dirname(__file__) or ".")
    ap.add_argument("--only", default=None, help="render one suite entry")
    ap.add_argument("--testcard", action="store_true",
                    help="use synthetic portrait test card as input")
    args = ap.parse_args()

    rgb = testcard() if args.testcard else _load(args.input)
    tag = "tc_" if args.testcard else ""
    for name, ctl in SUITE:
        if args.only and args.only not in name:
            continue
        out = render(rgb, ctl, seed=7)
        _save(out, os.path.join(args.outdir, f"sim_{tag}{name}.png"))
        if name.startswith(("002", "003", "005")):
            _save_crop(out, os.path.join(args.outdir, f"crop_{tag}{name}.png"))


if __name__ == "__main__":
    main()
