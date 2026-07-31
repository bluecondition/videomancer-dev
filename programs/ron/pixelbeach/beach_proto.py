#!/usr/bin/env python3
"""Pixel Beach v3 physics prototype: integer 1D shallow water, side profile.

Grid: NC cell-columns x NR cell-rows (16px cells at 720p -> 80x45).
All math integer/shift-based so it ports straight to VHDL.

Fixed point: heights in 1/64 cell-row units (Q6).
  eta[i]  surface elevation rel. sea level (signed)
  u[i]    velocity (signed, cells/frame in Q8-ish)
  b[i]    bottom elevation rel. sea level (static, from x)
  foam[i] 0..255
  wet[i]  0..255 sand wetness
"""
import numpy as np
from PIL import Image
import os

NC, NR = 80, 45
Q = 64                      # Q6 height units per cell-row
SL_ROW = 26                 # sea level screen row (0 = top)

# ---- bottom profile: dry beach on left, shoaling slope, deep flat right ----
def bottom(i):
    # elevation in Q6 units, relative to sea level
    if i < 6:
        return (5 - i) * Q // 2 + Q * 2      # dry sand, above sea level
    if i < 40:
        # slope from +2 rows down to -10 rows across the surf zone
        return int(Q * (2 - (i - 6) * 12 / 34))
    return -10 * Q                            # deep water

B = np.array([bottom(i) for i in range(NC)], dtype=np.int32)

# ---- state ----
eta = np.maximum(B, 0) * 0    # start flat at sea level (eta=0); on dry sand depth clamps to 0
eta = np.zeros(NC, np.int32)
u = np.zeros(NC, np.int32)    # velocity at interface i+1/2 (between i and i+1)
foam = np.zeros(NC, np.int32)
wet = np.zeros(NC, np.int32)

GDT_SH = 3        # u += (eta[i]-eta[i+1]) >> GDT_SH   (gravity accel)
FLUX_SH = 5       # eta += (flux diff) >> FLUX_SH ; flux = u * depth_q
DEPTH_CLAMP = 12 * Q
FRICT_SH = 6      # u -= u >> FRICT_SH in shallow water
SWELL_AMP = 3 * Q # boundary swell amplitude
SWELL_PER = 140   # frames per swell

def hash8(x):
    x = (x * 2654435761) & 0xFFFFFFFF
    return (x >> 24) & 0xFF

frame = [0]

def step():
    global eta, u, foam, wet
    t = frame[0]
    # depth per column (clamped)
    d = np.clip(eta - B, 0, DEPTH_CLAMP)

    # -- velocity update: gravity on surface slope, only where water on both sides
    for i in range(NC - 1):
        if d[i] > 0 or d[i + 1] > 0:
            de = eta[i] - eta[i + 1]
            u[i] += de >> GDT_SH if de >= 0 else -((-de) >> GDT_SH)
            # shallow friction: stronger when depth small
            dep = max(d[i], d[i + 1])
            fr = FRICT_SH if dep > 2 * Q else FRICT_SH - 2
            u[i] -= u[i] >> fr if u[i] >= 0 else -((-u[i]) >> fr)
        else:
            u[i] = 0
        u[i] = max(-255, min(255, u[i]))

    # -- flux + height update (upwind depth)
    flux = np.zeros(NC, np.int32)
    for i in range(NC - 1):
        dep = d[i + 1] if u[i] < 0 else d[i]   # upwind: u<0 flows leftward(from i+1)
        # wait: u[i] positive means flow from i to i+1 (rightward)
        f = u[i] * (dep >> 4)                  # depth in Q2 rows (0..47)
        flux[i] = f
    # conservative: quantize each interface flux ONCE, add left/subtract right
    qf = np.zeros(NC, np.int32)
    for i in range(NC - 1):
        f = flux[i]
        qf[i] = f >> FLUX_SH if f >= 0 else -((-f) >> FLUX_SH)
    for i in range(NC):
        fl = qf[i - 1] if i > 0 else 0
        fr = qf[i]     if i < NC - 1 else 0
        eta[i] += fl - fr
    # pairwise diffusion (conservative) to damp odd-even noise
    for i in range(NC - 1):
        df = eta[i] - eta[i + 1]
        df = df >> 5 if df >= 0 else -((-df) >> 5)
        eta[i] -= df; eta[i + 1] += df

    # -- boundary swell at right edge: triangle-ish sine
    ph = (t % SWELL_PER) / SWELL_PER
    sw = int(SWELL_AMP * np.sin(2 * np.pi * ph))
    eta[NC - 1] = sw

    # -- breaking: steep leftward-moving front in shallow water -> foam
    for i in range(1, NC - 1):
        slope = eta[i + 1] - eta[i]            # front face of leftward wave
        dep = d[i]
        if dep > 0 and dep < 6 * Q and slope > Q // 3 and u[i] < -16:
            foam[i] = min(255, foam[i] + 90)
            foam[i - 1] = min(255, foam[i - 1] + 60)
    # foam advect + decay
    nf = foam.copy()
    for i in range(NC):
        uu = u[i - 1] if i > 0 else 0
        if uu < -32 and i > 0:
            m = foam[i] >> 2
            nf[i] -= m; nf[i - 1] = min(255, nf[i - 1] + m)
        elif uu > 32 and i < NC - 1:
            m = foam[i] >> 2
            nf[i] -= m; nf[i + 1] = min(255, nf[i + 1] + m)
    foam = nf - (nf >> 4) - (nf > 0).astype(np.int32)
    foam = np.clip(foam, 0, 255)

    # -- wet sand
    for i in range(NC):
        if d[i] > 0:
            wet[i] = 255
        else:
            wet[i] = max(0, wet[i] - 2)
    frame[0] += 1

# ---- palette (RGB for proto; VHDL will use YUV) ----
SKY = (150, 210, 235); SKY2 = (185, 228, 240)
SUN = (255, 230, 120)
SAND = (232, 200, 140); SANDW = (176, 148, 104)
DEEP = (20, 60, 150); MID = (24, 96, 190); AQUA = (40, 170, 190); SURF = (90, 210, 200)
FOAM = (245, 250, 250)

def render():
    img = np.zeros((NR, NC, 3), np.uint8)
    d = np.clip(eta - B, 0, DEPTH_CLAMP)
    for i in range(NC):
        surf_row = SL_ROW - (eta[i] + Q // 2) // Q      # screen row of surface
        bot_row = SL_ROW - (B[i] + Q // 2) // Q
        for r in range(NR):
            if r >= bot_row:
                img[r, i] = SANDW if wet[i] > 128 else SAND
            elif r >= surf_row and d[i] > 4:
                depth_rows = r - surf_row
                if foam[i] > 140 and depth_rows <= 1:
                    img[r, i] = FOAM
                elif foam[i] > 60 and depth_rows == 0:
                    img[r, i] = FOAM
                elif depth_rows == 0:
                    img[r, i] = SURF
                elif depth_rows < 3:
                    img[r, i] = AQUA
                elif depth_rows < 7:
                    img[r, i] = MID
                else:
                    img[r, i] = DEEP
            else:
                # sky with sun block
                img[r, i] = SKY2 if r > SL_ROW - 14 else SKY
                if 4 <= r <= 8 and 58 <= i <= 63:
                    img[r, i] = SUN
    return img

OUT = os.path.dirname(os.path.abspath(__file__))
frames_to_save = {160, 180, 200, 215, 230, 245, 260, 280}
strip = []
for f in range(300):
    step()
    if f in frames_to_save:
        strip.append(render())
if strip:
    sheet = np.concatenate(strip, axis=0)
    Image.fromarray(np.kron(sheet, np.ones((4, 4, 1), np.uint8))).save(
        os.path.join(OUT, "proto_strip.png"))
wetmass = np.sum(np.clip(eta - B, 0, None))
print("max |u|:", np.max(np.abs(u)), "eta range:", eta.min(), eta.max(),
      "foam max:", foam.max(), "wet mass:", wetmass)
