#!/usr/bin/env python3
"""SPILLWAY physics prototype: per-column dam-overflow luma cascade.

Faithful to the planned RTL: state at half horizontal resolution (one state
column per 2 px), updated once per line top-to-bottom in scan order, all math
integer/shift-based. Lines are vectorized across columns with numpy, which is
semantically identical to scan order because columns are independent.

Fixed point (matches SUMMARY.md):
  video 10-bit; w/v/foam 8-bit saturating; v is Q4.4; envelopes Q8.8.
  Per-frame "multiplies" happen in vblank() only; the per-line path uses the
  2-term shift pair (fs0, fs1) exactly as the RTL will.
"""
import numpy as np
from PIL import Image
import os, sys

OUT = os.path.dirname(os.path.abspath(__file__))
W, H = 960, 540
NC = W // 2                    # state columns (x >> 1)
NG = 8                         # flood gates
GATE_COLS = NC // NG

# ---------------------------------------------------------------- colorspace
def rgb_to_yuv(img):
    r, g, b = [img[..., i].astype(np.int32) for i in range(3)]
    y = (299 * r + 587 * g + 114 * b) // 1000            # 0..255
    u = 512 + ((b - y) * 2258) // 1000                   # ~0.564*4
    v = 512 + ((r - y) * 2852) // 1000                   # ~0.713*4
    return (y << 2).clip(0, 1023), u.clip(0, 1023), v.clip(0, 1023)

def yuv_to_rgb(y, u, v):
    yf = (y >> 2).astype(np.int32)
    cb = (u.astype(np.int32) - 512)
    cr = (v.astype(np.int32) - 512)
    r = yf + (cr * 350) // 1000
    b = yf + (cb * 443) // 1000
    g = yf - (cr * 179) // 1000 - (cb * 86) // 1000
    return np.stack([r, g, b], -1).clip(0, 255).astype(np.uint8)

# ---------------------------------------------------------------- noise
def hash8(c, yy, ph):
    x = (c * 0x9E3779B1) ^ ((yy & 0xFFFF) * 0x85EBCA77) ^ ((ph & 0xFFFF) * 0xC2B2AE3D)
    x = (x * 2654435761) & 0xFFFFFFFF
    return (x >> 24) & 0xFF

COLS = np.arange(NC, dtype=np.int64)

def vhash8(a, b, c):
    """RTL-faithful 16-bit add-shift mixer (3 rounds), returns high byte.
    Matches spillway.vhd f_mxa/f_mxb/f_mxc exactly in structure."""
    x = (np.asarray(a, np.int64) ^ ((np.asarray(b, np.int64) << 5) & 0xFFFF)
         ^ (np.asarray(c, np.int64) * 0x101)) & 0xFFFF
    x = (x + (x << 5)) & 0xFFFF
    x = x ^ (x >> 7)
    x = (x + (x << 3)) & 0xFFFF
    x = x ^ (x >> 5)
    x = (x + (x << 7)) & 0xFFFF
    x = x ^ (x >> 9)
    return ((x >> 8) & 0xFF).astype(np.int32)

# 2-term shift-pair approximation of gain/1024 (RTL-faithful)
def shift_pair(gain):
    """gain 0..1023 -> (fs0, fs1) with 2^-fs0 + 2^-fs1 ~= gain/1024; 15 = off."""
    if gain < 1:
        return 15, 15
    best = (15, 15); berr = 1e9
    for a in range(1, 11):
        for b in range(a, 12):
            g = 1024 // (1 << a) + 1024 // (1 << b)
            e = abs(g - gain)
            if e < berr:
                berr = e; best = (a, b)
    return best

# ---------------------------------------------------------------- controls
def default_controls():
    return dict(K1=560, K2=512, K3=512, K4=512, K5=512, K6=0,
                S7=0, S8=0, S9=0, S10=0, S11=0, P12=0)

# DEPTHS palette anchors (sim = plain BT.601): (Y, U, V, carried_weight_num/4)
PALETTE = [
    (0,   512, 512, 4),   # clear: pure carried colour
    (940, 588, 452, 2),   # glacial cyan-white
    (560, 560, 420, 1),   # deep teal-green
    (420, 470, 600, 1),   # murky flood-brown
]

class Spillway:
    def __init__(self):
        # per-column state (the four 16-bit BRAMs)
        self.w    = np.zeros(NC, np.int32)
        self.v    = np.zeros(NC, np.int32)
        self.cu   = np.full(NC, 128, np.int32)   # carried chroma, 8-bit
        self.cv   = np.full(NC, 128, np.int32)
        self.foam = np.zeros(NC, np.int32)
        self.im_s = np.zeros(NC, np.int32)       # impact strength (prev frame)
        self.im_y = np.full(NC, 255, np.int32)   # impact row >>3 (prev frame)
        self.dk   = np.zeros(NC, np.int32)       # undertow dark accumulator
        self.disch_prev = np.zeros(NC, bool)     # ledge detect
        self.ut_h = np.zeros(NC // 2, np.int32)  # undertow tendril heights (x>>2)
        # slow state (vblank)
        self.fg_pos = 0; self.fg_vel = 0         # floodgate spring, Q(10).6
        self.surge_ph = 0
        self.fe = 0                              # freeze env 0..255
        self.level = 0                           # basin level, lines Q8
        self.vol = 0                             # spill volume accumulator
        self.gate_env = np.full(NG, 255, np.int32)
        self.gate_tgt = np.full(NG, 255, np.int32)
        self.gate_tmr = np.array([60 + hash8(g, 7, 3) for g in range(NG)], np.int32)
        self.fall_ph = 0                         # foam scroll accumulator Q4
        self.frame = 0
        self.frozen_seed = 0
        self.ring = np.zeros((4, NC), np.int32)  # basin reflection ring (out luma>>2)

    # ------------------------------------------------------------ vblank
    def vblank(self, c):
        fr = self.frame
        # --- FREEZE envelope (state w/ hysteresis) ---
        if c['S11']:
            self.fe = min(255, self.fe + 2)              # ~2 s at 60 fps
        else:
            self.fe = max(0, self.fe - 4)                # ~1 s thaw
        fe = self.fe
        if fe < 255:
            self.frozen_seed = fr                        # LFSR runs until solid
        # slider position is captured in the ice: latch while solid, and the
        # thaw crossing back below 128 snaps to the live position = the lurch
        if fe <= 128:
            self.fg_latch = None

        # --- FLOODGATE spring (asymmetric, under-damped release) ---
        # asymmetric stiffness, SHARED soft damping (branching the damping by
        # err sign kills the ring the moment an overshoot flips the sign).
        # Characterized at 60 fps: open t63 ~15 f; on release the discharge
        # dies in ~0.6 s, one ~17% backwash lobe at ~1 s, settled ~2 s.
        tgt = c['P12'] << 6                              # Q16
        err = tgt - self.fg_pos
        self.fg_vel += err >> (2 if err >= 0 else 5)
        self.fg_vel -= (self.fg_vel >> 5) + (self.fg_vel >> 6)
        self.fg_pos += self.fg_vel >> 3
        # anti-windup: hitting a clamp zeroes the velocity, or a fast pull
        # pins the gate at the stop for a second
        if self.fg_pos > (1023 << 6):
            self.fg_pos = 1023 << 6
            self.fg_vel = 0
        elif self.fg_pos < -131072:
            self.fg_pos = -131072
            self.fg_vel = 0
        pos = max(0, self.fg_pos >> 6)                   # 0..1023 effective
        if fe > 128:
            if getattr(self, 'fg_latch', None) is None:
                self.fg_latch = pos
            pos = self.fg_latch                          # captured in the ice

        # --- SURGE (~0.15 Hz triangle, rides on pos) ---
        surge = 0
        if c['S10']:
            self.surge_ph = (self.surge_ph + 700) & 0xFFFFF   # ~0.16 Hz
            t = self.surge_ph >> 12                            # 0..255
            surge = t if t < 128 else 255 - t                  # 0..127 tri
        # --- gate curve: crest drop + flow gain ---
        p2 = (pos * pos) >> 10                           # 0..1023 quadratic
        self.crest_drop = (p2 * 3) >> 2                  # up to ~768
        flow_k = c['K2'] >> 2                            # 0..255 character
        open_k = 0 if pos < 40 else min(255, ((pos - 40) * 3) >> 2)
        gain = (flow_k * open_k) >> 7                    # 0..~510
        gain = min(1023, gain + (surge * open_k >> 7))   # surge adds discharge
        self.fs0, self.fs1 = shift_pair(gain)
        self.armed = pos < 40                            # closed: shimmer only

        # --- per-frame constants from knobs ---
        self.crest = 96 + ((c['K1'] * 7) >> 3)           # 96..991
        # NOTE freeze does NOT touch g/dsh/leak: with a static source the
        # column state re-derives identically each frame, so stopping the
        # scroll phase + noise seed + wobble is what crystallizes the render.
        k3 = c['K3'] >> 2
        self.g   = max(1, k3 >> 3)                       # Q4 gravity per line
        # water that goes over the crest falls all the way down: decay is
        # SLOW (it models sheet->spray thinning, not evaporation).  Low
        # PLUNGE = syrupy short-fall look = faster decay. w>>8 == 0 -> off.
        i3 = k3 >> 5
        self.dsh = (4, 5, 6, 7, 8, 8, 8, 8)[i3] + (0 if H < 864 else 1)
        self.leak = 0 if i3 >= 2 else 1
        k4 = min(255, (c['K4'] >> 2) + (surge >> 1))     # surge is whiter
        self.k4 = k4
        self.cs   = max(3, 8 - (k4 >> 6))                # aeration rate shift
        self.fdec = 4
        self.mist_on = c['K5'] > 8
        self.mist_sh = max(0, 3 - (c['K5'] >> 8))        # bigger K5 = brighter
        # DEPTHS palette interp (vblank mult is fine)
        k6 = c['K6']
        seg = min(2, k6 // 342); f = (k6 - seg * 342) * 3 // 4  # 0..255 in seg
        a, b = PALETTE[seg], PALETTE[seg + 1]
        self.tintY = a[0] + ((b[0] - a[0]) * f >> 8)
        self.tintU = a[1] + ((b[1] - a[1]) * f >> 8)
        self.tintV = a[2] + ((b[2] - a[2]) * f >> 8)
        self.cw    = a[3] if f < 128 else b[3]           # carried weight /4
        self.roar = max(0, pos - 960) << 1               # white-out very top

        # --- BASIN level dynamics ---
        if c['S9']:
            self.level += self.vol >> 9
            self.level -= self.level >> 6
            self.level = min(self.level, (H // 3) << 8)
        else:
            self.level = max(0, self.level - (self.level >> 4) - 64)
        self.surface = H - (self.level >> 8)
        self.crown = min(255, self.vol >> 6)
        self.vol = 0

        # --- gate envelopes (damped ~1 s travel) ---
        if c['S7']:
            if fe < 200:                                 # gates freeze too
                self.gate_tmr -= 1
                for g in range(NG):
                    if self.gate_tmr[g] <= 0:
                        r = hash8(g * 31 + 7, fr, 0x5EED)
                        self.gate_tgt[g] = 255 if (r & 3) else 0  # mostly open
                        self.gate_tmr[g] = 90 + (r >> 1)          # 1.5..3.6 s
                d = self.gate_tgt - self.gate_env
                self.gate_env += np.sign(d) * np.minimum(np.abs(d), 4)
        else:
            self.gate_env[:] = 255
        self.gate_lift = ((255 - self.gate_env) * 4).astype(np.int32)

        # --- foam fall-phase scroll (freeze stops it) ---
        vavg = int(self.v.mean())
        self.fall_ph += ((24 + vavg) * (255 - fe)) >> 12   # ~7-30 px/frame

        # --- frame-start state reset: no water above line 0 ---
        self.w[:] = 0
        self.v[:] = 16
        self.foam[:] = 0

        # --- column sweep: undertow heights chase, impacts decay ---
        self.im_s -= self.im_s >> 3
        # (3-tap smoothing dropped: the mist band dither masks columniness
        # and the LCs were needed back on hardware)
        dk4 = self.dk.reshape(-1, 2).sum(1) >> 3
        tgt4 = np.minimum(dk4, H // 3)
        self.ut_h += (tgt4 - self.ut_h) >> 3
        self.dk[:] = 0

        self.noise_ph = self.frozen_seed
        self.frame += 1

    # ------------------------------------------------------------ frame
    def render(self, ysrc, usrc, vsrc, c):
        """ysrc/usrc/vsrc: HxW 10-bit arrays. Returns 10-bit YUV out."""
        self.vblank(c)
        oy = np.empty((H, W), np.int32)
        ou = np.empty((H, W), np.int32)
        ov = np.empty((H, W), np.int32)
        crest_eff = np.maximum(0, self.crest - self.crest_drop
                               + np.repeat(self.gate_lift, GATE_COLS))
        fs0, fs1 = self.fs0, self.fs1
        surface = self.surface if c['S9'] else H
        im_s_new = np.zeros(NC, np.int32)
        im_y_new = np.full(NC, 255, np.int32)
        nph = self.noise_ph
        ring_frozen = False
        # per-frame constant: which mirror wobble per basin line
        for y in range(H):
            Ys = ysrc[y, ::2]; Us = usrc[y, ::2]; Vs = vsrc[y, ::2]
            # ---------------- column state update (even-pixel cadence) ------
            h = np.maximum(0, Ys - crest_eff)
            dq = np.zeros(NC, np.int32)
            if not self.armed:
                if fs0 < 15: dq = dq + (h >> fs0)
                if fs1 < 15: dq = dq + (h >> fs1)
            wet = self.w > 8
            self.v = np.where(wet, np.minimum(255, self.v + self.g), 16)
            self.w = np.clip(self.w - (self.w >> self.dsh) - self.leak + dq, 0, 255)
            self.foam = np.clip(self.foam - (self.foam >> self.fdec)
                                + np.where(wet, self.v >> self.cs, 0), 0, 255)
            disch = dq > 2
            self.cu = np.where(disch, self.cu + ((Us >> 2) - self.cu >> 2), self.cu)
            self.cv = np.where(disch, self.cv + ((Vs >> 2) - self.cv >> 2), self.cv)
            self.vol += int(dq.sum()) >> 6
            # rock impact: streak crosses a bright->dark LEDGE lip (one-shot:
            # only on the first dark line after discharge, else every dark
            # region eats the whole fall)
            rock = self.disch_prev & (h == 0) & (Ys < 220) & (self.w > 96)
            self.disch_prev = disch
            if y < surface:
                st = np.where(rock, self.w, 0)
                rec = st > im_s_new
                im_s_new = np.where(rec, st, im_s_new)
                im_y_new = np.where(rec, y >> 3, im_y_new)
                self.w = np.where(rock, self.w - (self.w >> 2), self.w)
            elif not ring_frozen:
                # first basin line: every column lands -> impact records
                rec = self.w > im_s_new
                im_s_new = np.where(rec, self.w, im_s_new)
                im_y_new = np.where(rec, y >> 3, im_y_new)
            # undertow dark accumulation below the low waterline
            if c['S8']:
                self.dk += (Ys < 200) & (y > H // 2)

            # ---------------- render ------------------------------------
            # filament cells: 1 col wide, height stretches with velocity
            # (4 lines near the crest -> 32-line filaments at speed); the
            # texture scrolls DOWN faster where the water moves faster.
            csh = 2 + np.minimum(3, self.v >> 6)
            yy = y + ((self.fall_ph * (2 + (self.v >> 5))) >> 2)
            cell = yy >> csh
            # brightness modulation: scrolls with the fall, stable per frame
            fil = vhash8(COLS, cell, 0x77)
            # presence: LONG cells (4x filament cell) -> continuous streaks
            pf = vhash8(COLS, yy >> (csh + 2), 0xA1)
            # fine foam noise re-rolls every frame (boiling); cluster scrolls
            n = vhash8(COLS * 3 + 1, yy >> 2, (nph & 0xFF) ^ 0x5A)
            fil2 = vhash8(COLS + 13, yy >> 3, 0x33)
            # continuity: coverage ~ flux/velocity -- the sheet necks down into
            # fewer, brighter filaments as it accelerates.
            # GRADED, not binary: opacity grows with the MARGIN past the
            # threshold, so filaments have soft edges instead of the on/off
            # barcode that read as a glitch.
            cthr = np.maximum(0, np.minimum(240, self.w) - (self.v >> 1))
            margin = cthr - pf
            opc = np.clip(margin << 2, 0, 255)          # soft 64-unit edge
            cap = np.clip((self.w - 8) << 1, 0, 255)    # thin water is faint
            op = np.minimum(opc, cap)
            # glassy laminar sheet near the crest: see through it
            op = np.where((self.v < 64) & ~ (fil2 < self.foam),
                          np.minimum(op, 160), op)
            # water body luma: flux + speed bright, filament-modulated
            wl = np.clip(220 + (self.w << 2) + (self.v >> 1)
                         - ((fil & 63) << 1), 0, 1023)
            # foam: whitecaps, whiteness graded by cluster margin (soft
            # clumps, not hard blobs), gated on water presence
            fmargin = np.clip((self.foam - fil2) << 1, 0, 255)
            foamy = (fmargin > 0) & (n < 176) & (self.w > 8) & (op > 32)
            fop = np.where(foamy, np.minimum(fmargin, cap), 0)
            wl = np.where(foamy, 800 + ((n & 47) << 2), wl)
            op = np.maximum(op, fop)
            # crest roll: sparkling specular lip where water bends over the
            # weir (dithered edge, not a solid bar)
            roll = (h > 0) & (h < 8 + (n & 15)) & (dq > 0)
            wl = np.where(roll, 1000, wl)
            op = np.where(roll, 255, op)
            # water colour: carried vs DEPTHS tint (weights /4)
            cw = self.cw
            wcU = ((self.cu << 2) * cw + self.tintU * (4 - cw)) >> 2
            wcV = ((self.cv << 2) * cw + self.tintV * (4 - cw)) >> 2
            wcU = np.where(foamy, 512 + ((wcU - 512) >> 2), wcU)
            wcV = np.where(foamy, 512 + ((wcV - 512) >> 2), wcV)

            Y = ysrc[y].copy(); U = usrc[y].copy(); V = vsrc[y].copy()
            # sweep energy out of over-crest source (FLOW carries it away)
            hfull = np.maximum(0, ysrc[y] - np.repeat(crest_eff, 2))
            if not self.armed:
                dqf = np.zeros(W, np.int32)
                if fs0 < 15: dqf = dqf + (hfull >> fs0)
                if fs1 < 15: dqf = dqf + (hfull >> fs1)
                # drain toward crest at 3/4 strength: keeps structure alive
                # in spilling regions instead of posterized flats
                Y = Y - np.minimum(hfull, dqf - (dqf >> 2))
            # crest shimmer when armed (pressure waiting)
            if self.armed:
                nearcrest = np.abs(ysrc[y] - self.crest) < 24
                Y = Y + np.where(nearcrest, ((np.repeat(n, 2) & 7) << 2) - 16, 0)

            # water crossfade composite: quarter-step alpha + PIXEL-RATE
            # noise dither of the fractional part. At 60 fps this reads as a
            # smooth 8-bit blend; the residual grain reads as aeration.
            # (RTL: base = op>>6, +1 when op&63 beats 6 LFSR bits.)
            op2 = np.repeat(op, 2)
            pdith = vhash8(np.arange(W), y * 3 + 1,
                           (self.frame * 5 + y) & 0xFFFF) & 63
            a2 = (op2 >> 6) + ((op2 & 63) > pdith)
            Y = (Y * (4 - a2) + np.repeat(wl, 2) * a2) >> 2
            U = (U * (4 - a2) + np.repeat(wcU, 2) * a2) >> 2
            V = (V * (4 - a2) + np.repeat(wcV, 2) * a2) >> 2
            # freeze glint: rare stable vertical threads of icicle sheen
            if self.fe > 128:
                gl = np.repeat((margin > 0) & (self.w > 24)
                               & ((fil & 31) == 0) & (n < 128), 2)
                Y = np.where(gl, 1023, Y)
                U = np.where(gl, 512 + ((U - 512) >> 2), U)
                V = np.where(gl, 512 + ((V - 512) >> 2), V)
            # mist above last frame's impacts
            if self.mist_on:
                d = self.im_y - (y >> 3)
                mband = (d >= 0) & (d < 12)
                # band-edge dither: +0/+1 on d before shaping kills the
                # visible 16-line halo terraces
                dd = np.abs(d) + (n & 1)
                fog = np.where(mband, self.im_s >> np.minimum(7, 1 + (dd >> 1)
                                                              + self.mist_sh), 0)
                fog2 = np.repeat(fog, 2)
                Y = np.minimum(1023, Y + np.minimum(384, fog2 << 3))
                sat = fog2 > 16
                U = np.where(sat, U + ((512 - U) >> 1), U)
                V = np.where(sat, V + ((512 - V) >> 1), V)
                spark = np.repeat((n ^ (COLS & 0xFF)) & 0xFF, 2)
                Y = np.where((fog2 > 8) & (spark < np.minimum(24, fog2 >> 1)),
                             1023, Y)
            # undertow tendrils
            if c['S8']:
                yb = H - y
                ut = np.repeat(self.ut_h, 4)[:NC]
                rag = (n >> 3).astype(np.int32)
                mask = np.repeat(yb < (ut + rag), 2)
                Y = np.where(mask, Y >> 2, Y)
                U = np.where(mask, 512 + ((U - 512) >> 2), U)
                V = np.where(mask, 512 + ((V - 512) >> 2), V)
            # basin
            if c['S9'] and y >= surface:
                depth = y - surface
                wob = ((hash8(y, self.frame >> 1, 77) * (255 - self.fe)) >> 8)
                mi = (depth + (wob >> 5)) & 3
                hoff = ((wob >> 3) & 7) - 4 if self.fe < 200 else 0
                refl = np.roll(self.ring[mi], hoff)
                dsh = min(4, 1 + (depth >> 5))
                ry = np.repeat(refl, 2)
                # water is never colourless: clear DEPTHS still gets a cool hue
                bu = (self.tintU * (4 - cw) + 528 * cw) >> 2
                bv = (self.tintV * (4 - cw) + 476 * cw) >> 2
                Y = np.minimum(1023, (ry - (ry >> dsh)) + (self.tintY >> 2))
                # fade to deep water with depth: the 4-line ring repetition
                # disappears into the body of the pool
                q = min(3, depth >> 4)
                deep = 170 + (self.tintY >> 2)
                Y = (Y * (4 - q) + deep * q) >> 2
                # waterline: one bright specular line at the surface
                if depth == 0:
                    Y = np.maximum(Y, 760)
                U = np.full(W, bu); V = np.full(W, bv)
                crown_h = 2 + (self.crown >> 6)
                if depth < crown_h:   # foam crown where the falls land
                    cr = np.repeat((n < self.crown), 2)
                    Y = np.where(cr, 1023, Y)
                    U = np.where(cr, 512 + ((U - 512) >> 2), U)
                    V = np.where(cr, 512 + ((V - 512) >> 2), V)
            # roar white-out
            if self.roar:
                Y = np.minimum(1023, Y + self.roar)
            Y = Y.clip(0, 1023); U = U.clip(0, 1023); V = V.clip(0, 1023)
            oy[y] = Y; ou[y] = U; ov[y] = V
            # basin ring: freeze once the beam enters the basin
            if y < surface:
                self.ring[y & 3] = Y[::2] >> 2 << 2  # 8-bit fidelity
            else:
                ring_frozen = True
        self.im_s = np.maximum(self.im_s, im_s_new)
        upd = im_s_new >= self.im_s
        self.im_y = np.where(upd, im_y_new, self.im_y)
        return oy, ou, ov


# ---------------------------------------------------------------- harness
def load_input():
    p = os.path.join(OUT, '..', 'turpentine', 'sim_000_input.png')
    img = Image.open(p).convert('RGB').resize((W, H), Image.LANCZOS)
    return rgb_to_yuv(np.asarray(img))

def save(y, u, v, name):
    Image.fromarray(yuv_to_rgb(y, u, v)).save(os.path.join(OUT, name))

def run(name, frames, ctl_fn, dump_every=None, dump_at=None, engine=None):
    ys, us, vs = load_input()
    sp = engine or Spillway()
    for f in range(frames):
        c = ctl_fn(f)
        oy, ou, ov = sp.render(ys, us, vs, c)
        if (dump_every and f % dump_every == dump_every - 1) or \
           (dump_at and f in dump_at):
            save(oy, ou, ov, f'{name}_{f:03d}.png')
    return sp

if __name__ == '__main__':
    which = sys.argv[1] if len(sys.argv) > 1 else 'all'

    def base(f, **kw):
        c = default_controls(); c.update(kw); return c

    if which in ('all', 'core'):
        # core engine: crest+flow+plunge at half-open floodgate
        run('core', 60, lambda f: base(f, P12=600), dump_at={20, 59})
        # long screaming streaks
        run('plunge', 60, lambda f: base(f, P12=700, K3=1023), dump_at={59})
        # syrupy low-head
        run('syrup', 60, lambda f: base(f, P12=700, K3=100), dump_at={59})
    if which in ('all', 'water'):
        run('churn', 60, lambda f: base(f, P12=700, K4=1023), dump_at={59})
        run('mist', 90, lambda f: base(f, P12=700, K5=1023, S9=1), dump_at={89})
        for i, k6 in enumerate((100, 450, 800, 1023)):
            run(f'depths{i}', 45, lambda f, k=k6: base(f, P12=700, K6=k), dump_at={44})
    if which in ('all', 'struct'):
        run('gates', 240, lambda f: base(f, P12=700, S7=1), dump_at={80, 160, 239})
        run('basin', 300, lambda f: base(f, P12=800, S9=1, K4=700), dump_at={60, 150, 299})
        run('undertow', 180, lambda f: base(f, P12=650, S8=1), dump_at={90, 179})
    if which in ('all', 'dyn'):
        run('surge', 400, lambda f: base(f, P12=600, S10=1), dump_at={100, 200, 300, 399})
        sp = run('freeze', 200, lambda f: base(f, P12=700, S11=1 if f > 30 else 0),
                 dump_at={30, 100, 199})
    if which in ('all', 'perf'):
        # THE deliverable: scripted floodgate gesture closed -> breach -> release
        def gesture(f):
            if f < 40:    p = 0                       # closed, shimmer
            elif f < 130: p = min(1023, (f - 40) * 12)  # haul open ~1.5 s
            elif f < 240: p = 1023                    # full breach
            else:         p = 0                       # let go -> slosh
            return base(f, P12=p, S9=1, K4=650, K5=650)
        run('gesture', 360, gesture, dump_every=12)
    print('done')
