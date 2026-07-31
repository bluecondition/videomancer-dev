#!/usr/bin/env python3
"""
MORSEL -- bit-exact preview of the hardware pixel pipeline.

Everything here is the integer maths the VHDL performs, vectorised with numpy
so a whole 1920x1080 frame renders in a second.  The VHDL is a transcription
of this file; if the preview looks right the hardware looks right.

Usage:  python3 preview_morsel.py [source.png]
"""

import math
import os
import sys

import numpy as np
from PIL import Image

import genchips

HERE = os.path.dirname(os.path.abspath(__file__))
LIGHT = 80                      # light from upper-left: angle index 225 deg


# ---------------------------------------------------------------------------
# tables
# ---------------------------------------------------------------------------
SHAPE_ROWS, _ = genchips.build_shape_rom()
SHAPE = np.array([int(v) for row in SHAPE_ROWS for v in row], dtype=np.int32)
RTAB = np.array(genchips.build_rtab(), dtype=np.int32)
COSN = np.array([(c - 256 if c > 127 else c) for c in genchips.build_cos()],
                dtype=np.int32)


# ---------------------------------------------------------------------------
# hash -- bit-exact with f_pack / f_mix1 / f_mix2 in morsel.vhd
# ---------------------------------------------------------------------------
M16 = 0xFFFF


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
# frame constants from the panel
# ---------------------------------------------------------------------------
class Ctl:
    def __init__(self, k1=512, k2=512, k3=430, k4=620, k5=560, k6=520,
                 p12=760, s7=1, s8=0, s9=0, s10=1, s11=0):
        # K1 SCALE -> cell size 8/16/32/64/128 px
        self.scl = 0 if k1 < 205 else 1 if k1 < 410 else 2 if k1 < 640 else \
                   3 if k1 < 900 else 4
        self.cell = 8 << self.scl
        self.sh = 3 + self.scl
        self.aa = [32, 16, 8, 4, 2][self.scl]

        # K6 CHIP SIZE.  64 = the chip exactly as modelled; capped at 76 so
        # the largest silhouette (radius 101 quarter-units) plus its size
        # jitter still fits inside the half-cell (32 units = 128 quarter).
        self.szb = 20 + (k6 * 56 >> 10)                 # 20..76
        sh = lambda n: self.szb >> n
        self.sztab = [self.szb - sh(3), self.szb - sh(4) - sh(5),
                      self.szb - sh(4), self.szb - sh(5), self.szb,
                      self.szb + sh(5), self.szb + sh(4),
                      self.szb + sh(4) + sh(5)]

        # Position jitter is scaled to the room the biggest chip in the cell
        # actually leaves, so chips scatter as far as they can without ever
        # being clipped by their cell.  C_JMAX in the VHDL: this is
        # clamp((126 - 1.73*szb)/4, 0, 15) as a table, because working it out
        # arithmetically was a vblank critical path.
        C_JMAX = [15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 13, 11, 10, 8, 6, 4,
                  3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.jmax = C_JMAX[self.szb >> 2]
        jm = self.jmax
        j34 = (jm >> 1) + (jm >> 2)          # 3/4, as one shift-add
        self.jtab = [-jm, -j34, -(jm >> 1), -(jm >> 2), -(jm >> 3),
                     jm >> 3, jm >> 2, jm >> 1, j34, jm,
                     -j34, jm >> 1, -(jm >> 2), j34, -(jm >> 1), jm >> 2]

        # K3 BAKE -> dough tone floor + span
        self.blo = 480 - (k3 >> 2)                       # 480..224
        self.bspan = 420 + (k3 >> 3)                     # 420..547

        # K4 RELIEF -> shading gain + specular
        self.rel = 2 if k4 >= 683 else (1 if k4 >= 341 else 0)
        self.spec = (k4 * 320) >> 10

        # K5 GRAIN
        self.grain = k5 * 8 >> 10                        # 0..7

        # K2 SHAPE -> which of the 8 modelled orientations the hash may pick.
        # 0 = only the side-on flat-bottomed teardrops, 1023 = only the
        # rounder top-down chips, middle = the full spread.
        lo = (k2 * 5) >> 10                              # 0..4 window start
        self.vartab = [min(7, lo + ((j * 5) // 8)) for j in range(8)]

        self.dens = p12 >> 2                             # 0..255
        self.s7, self.s8, self.s9 = s7, s8, s9
        self.s10, self.s11 = s10, s11


# ---------------------------------------------------------------------------
# render
# ---------------------------------------------------------------------------
def render(rgb, c):
    H, W, _ = rgb.shape
    R = rgb[:, :, 0].astype(np.int32)
    G = rgb[:, :, 1].astype(np.int32)
    B = rgb[:, :, 2].astype(np.int32)
    # 10-bit BT.601 in the program's internal (standard) convention
    vy = np.clip((299 * R + 587 * G + 114 * B) // 1000, 0, 255) * 4
    vu = np.clip(512 + (-169 * R - 331 * G + 500 * B) * 4 // 1000, 0, 1023)
    vv = np.clip(512 + (500 * R - 419 * G - 81 * B) * 4 // 1000, 0, 1023)
    y8 = vy >> 2

    X = np.arange(W, dtype=np.int32)[None, :] * np.ones((H, 1), dtype=np.int32)
    Y = np.arange(H, dtype=np.int32)[:, None] * np.ones((1, W), dtype=np.int32)

    CELL, SH = c.cell, c.sh
    cellx = X >> SH
    # Every cell COLUMN is slid vertically by its own pseudo-random phase.
    # The cell boundaries move with the chip, so nothing is ever clipped, and
    # the horizontal ROWS that a plain lattice produces disappear entirely --
    # jitter alone can never manage this, because a chip may only wander as
    # far as the slack left inside its own cell.
    hcol = f_hash(cellx, cellx + 733, 0x13D7)
    if SH >= 6:
        voff = ((hcol >> 3) & 0x3F).astype(np.int32) << (SH - 6)
    else:
        voff = ((hcol >> 3) & 0x3F).astype(np.int32) >> (6 - SH)
    yo = Y + voff
    celly = yo >> SH

    # in-cell position normalised to 0..63 whatever the cell size
    if SH <= 6:
        nx = (X & (CELL - 1)) << (6 - SH)
        ny = (yo & (CELL - 1)) << (6 - SH)
    else:
        nx = (X & (CELL - 1)) >> (SH - 6)
        ny = (yo & (CELL - 1)) >> (SH - 6)

    # ---- per-cell luma (ping-pong cell buffer: sampled at the top line of
    # the PREVIOUS cell row, held for every line of this one, so a chip never
    # changes its mind part-way down)
    ncy = (H >> SH) + 3
    ncx = (W >> SH) + 2
    ci = np.arange(ncx)
    if SH >= 6:
        cvo = ((f_hash(ci, ci + 733, 0x13D7) >> 3) & 0x3F) << (SH - 6)
    else:
        cvo = ((f_hash(ci, ci + 733, 0x13D7) >> 3) & 0x3F) >> (6 - SH)
    cvo = np.asarray(cvo, dtype=np.int32)
    store = np.zeros((ncy, ncx), dtype=np.int32) + 128
    xs = np.clip(ci * CELL + (CELL >> 1), 0, W - 1)
    for j in range(ncy):
        yl = np.clip(j * CELL - cvo, 0, H - 1)           # this column's top line
        store[j] = y8[yl, xs]
    used = np.vstack([store[:1], store[:-1]])            # one cell row of lag
    cluma = used[np.clip(celly, 0, ncy - 1), np.clip(cellx, 0, ncx - 1)]

    # ---- per-cell random parameters
    h1 = f_hash(cellx, celly, 0x5A3C)
    h2 = f_hash(cellx, celly, 0x9E37)
    pres = (h1 & 0xFF).astype(np.int32)
    rot = ((h1 >> 8) & 0x7F).astype(np.int32)
    var3 = (h2 & 0x7).astype(np.int32)
    sz3 = ((h2 >> 3) & 0x7).astype(np.int32)
    jt = np.array(c.jtab, dtype=np.int32)
    jx = jt[((h2 >> 6) & 0xF).astype(np.int32)]
    jy = jt[((h2 >> 10) & 0xF).astype(np.int32)]
    milk = ((h2 >> 14) & 1).astype(np.int32)
    white = ((h2 >> 15) & 1).astype(np.int32)

    variant = np.array(c.vartab, dtype=np.int32)[var3]

    # ---- luma drives the chips (S7): dark cells get more, and bigger, chips
    lm = (255 - cluma) if not c.s9 else cluma
    if c.s7:
        thr = np.clip(c.dens + (lm - 128), 0, 255)
        lsz = lm >> 4                                     # 0..15 -> +0..23%
    else:
        thr = np.full_like(pres, c.dens)
        lsz = np.zeros_like(pres)
    present = pres < thr

    # ---- polar coordinates about the (jittered) chip centre
    cx = nx - 32 - jx
    cy = ny - 32 - jy
    axv = np.abs(cx)
    ayv = np.abs(cy)
    sw = ayv > axv
    mx = np.where(sw, ayv, axv)
    mn = np.where(sw, axv, ayv)
    hi = mx >= 32
    mxs = np.where(hi, mx >> 1, mx)
    mns = np.where(hi, mn >> 1, mn)

    packed = RTAB[(mns << 5) + mxs]
    r4 = np.where(hi, (packed >> 4) << 1, packed >> 4)
    sub = packed & 0xF

    # screen angle, 128 steps, counted from +x toward +y (screen down)
    px = cx >= 0
    py = cy >= 0
    a7 = np.where(px & py, np.where(sw, 32 - sub, sub),
         np.where(~px & py, np.where(sw, 32 + sub, 64 - sub),
         np.where(~px & ~py, np.where(sw, 96 - sub, 64 + sub),
                             np.where(sw, 96 + sub, 128 - sub)))) & 127

    ar7 = (a7 - rot) & 127
    r4rom = SHAPE[(variant << 7) + ar7]

    szc = np.array(c.sztab, dtype=np.int32)[sz3] + lsz
    R4 = np.minimum((r4rom * szc) >> 6, 255)

    d = R4 - r4
    aa = c.aa
    ah = aa >> 1
    alpha = np.where(d >= aa, 4, np.where(d >= ah, 3, np.where(d >= 0, 2,
            np.where(d >= -ah, 1, 0))))
    alpha = np.where(present, alpha, 0)

    # ---- dough ------------------------------------------------------------
    dy = c.blo + ((y8 * c.bspan) >> 8)

    g = c.grain
    ms = min(3, 1 + c.scl)
    hp = f_hash(X, Y, 0x1234)
    hm = f_hash(X >> ms, Y >> ms, 0x77A1)
    dy = dy + (((hp & 15) - 8) * g >> 1)                  # fine grain
    dy = dy + ((((hm >> 4) & 15) - 8) * g >> 2)           # uneven browning
    dy = dy - np.where((hp >> 12) < g, 60, 0)             # crumb pores

    if c.s10:                                             # relief from video
        gx = y8 - np.roll(y8, 1, axis=1)
        gy = y8 - np.roll(y8, 1, axis=0)
        emb = np.clip(gx + gy, -80, 80)
        dy = dy + (emb << c.rel)

    du = np.where(dy < 256, 396, np.where(dy < 512, 392,
         np.where(dy < 768, 388, 396)))
    dv = np.where(dy < 256, 628, np.where(dy < 512, 618,
         np.where(dy < 768, 606, 590)))

    if c.s8:                                              # tint from video
        du = du + ((vu - 512) >> 1)
        dv = dv + ((vv - 512) >> 1)

    # ---- chip -------------------------------------------------------------
    lit = COSN[(a7 - LIGHT) & 127]
    band = np.where(d < (R4 >> 3), 3, np.where(d < (R4 >> 2), 2, 1))
    sh0 = np.where(band == 3, lit >> 1, np.where(band == 2, lit >> 2, lit >> 3))
    shade = sh0 << c.rel

    hot = (d < (R4 >> 4)) & (lit > 104)
    shade = shade + np.where(hot, c.spec, 0)
    rim = (d < (R4 >> 4)) & (lit < -80)              # dark contact edge
    shade = shade - np.where(rim, 60, 0)

    ctex = (((hp >> 6) & 7) - 4) * 2 + (((hm >> 9) & 7) - 4)

    isw = (white == 1) & (c.s11 == 1)
    cy0 = np.where(isw, 860, np.where(milk == 1, 196, 148))
    cu0 = np.where(isw, 424, 470)
    cv0 = np.where(isw, 556, 566)
    chip_y = cy0 + shade + ctex
    chip_u = cu0
    chip_v = cv0

    # contact shadow the chip throws onto the dough on its unlit side
    shadow = present & (d < 0) & (d > -(4 * aa)) & (lit < -50)
    dy = dy - np.where(shadow, 30 << c.rel, 0)

    dy = np.clip(dy, 0, 1023)

    # ---- composite --------------------------------------------------------
    oy = dy + (((chip_y - dy) * alpha) >> 2)
    ou = du + (((chip_u - du) * alpha) >> 2)
    ov = dv + (((chip_v - dv) * alpha) >> 2)
    oy = np.clip(oy, 0, 1023)
    ou = np.clip(ou, 0, 1023)
    ov = np.clip(ov, 0, 1023)

    return yuv_to_rgb(oy, ou, ov)


def yuv_to_rgb(y, u, v):
    yf = y / 4.0
    uf = (u - 512) / 4.0
    vf = (v - 512) / 4.0
    r = yf + 1.402 * vf
    g = yf - 0.344136 * uf - 0.714136 * vf
    b = yf + 1.772 * uf
    out = np.stack([r, g, b], axis=-1)
    return np.clip(out, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(HERE, "..", "sugarcoat", "test_face.png")
    img = np.array(Image.open(src).convert("RGB"))

    shots = [
        ("a_default", Ctl(k1=700, k6=780, p12=620)),
        ("b_small",   Ctl(k1=300, k6=800, p12=700)),
        ("c_big",     Ctl(k1=950, k6=820, p12=620)),
        ("d_sparse",  Ctl(k1=700, k6=780, p12=300)),
        ("e_packed",  Ctl(k1=700, k6=900, p12=1023)),
        ("f_round",   Ctl(k1=700, k6=780, k2=1023)),
        ("g_sideon",  Ctl(k1=700, k6=780, k2=0)),
        ("h_tint",    Ctl(k1=700, k6=780, s8=1, s11=1)),
    ]
    for name, ctl in shots:
        out = render(img, ctl)
        Image.fromarray(out).save(os.path.join(HERE, "prev_%s.png" % name))
        print("prev_%s.png  cell=%d szb=%d jmax=%d dens=%d"
              % (name, ctl.cell, ctl.szb, ctl.jmax, ctl.dens))
