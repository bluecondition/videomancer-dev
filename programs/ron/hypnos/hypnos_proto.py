#!/usr/bin/env python3
"""
HYPNOS prototype — numeric model of the concentric-ring engine.

Goal: prove the LOOK and the fixed-point plan before writing VHDL.
The per-pixel functional form here is exactly what the VHDL will implement:

    dx,dy   = x-cx, y-cy              (center freely off-screen, +/-2 screens)
    a,b     = |dx|,|dy|
    hi,lo   = max,min(a,b)
    r       = hi + k*lo               METRIC:  k=1 diamond, k~0.41 circle, k=0 square
    r'      = Rref * (r/Rref)^p       WARP:    p<1 tunnel (dense center),
                                               p=1 flat, p>1 dome (dense edge)
              implemented r' = exp2( p*(log2 r - LREF) + LREF )
    r''     = r' + input_luma_displace
    phi     = r'' * f + anim          f from PERIOD*ZOOM (exponential)
    band    = frac(phi) < duty        hard 1-bit key
    soft    = adaptive from |d phi/dx| (grays out near the alias limit) + baseline
    out     = black (band A) / white (band B), or soft gradient

Renders a contact sheet of the signature states so we can eyeball it.
"""
import numpy as np
from PIL import Image
import os

W, H = 640, 480                      # proto raster (SD-ish, fast)
LREF_LOG = np.log2(1024.0)           # log2(Rref), Rref = 1024

# ---- exp2 mantissa table the VHDL will use (64 entries, Q10) --------------
UFRAC = 6
MANT = [round(1024 * 2 ** (f / 64.0)) for f in range(64)]   # [1024,2048)

def exp2_fixed(e_q):
    """exp2 of a value in log2 domain with UFRAC frac bits, matching HW LUT+barrel."""
    e_q = np.asarray(e_q)
    ie = np.floor(e_q / (1 << UFRAC)).astype(np.int64)
    fe = (e_q - ie * (1 << UFRAC)).astype(np.int64)
    mant = np.array(MANT, dtype=np.int64)[np.clip(fe, 0, 63)]
    sh = ie - 10
    val = np.where(sh >= 0, mant << np.clip(sh, 0, 40),
                            mant >> np.clip(-sh, 0, 40))
    return np.clip(val, 0, 1 << 20).astype(np.float64)

def log2_fixed(r):
    """log2(r) with UFRAC frac bits, matching a HW priority-encoder + mantissa."""
    r = np.maximum(r, 1).astype(np.int64)
    msb = np.floor(np.log2(r)).astype(np.int64)          # HW: priority encoder
    # mantissa fraction = (r - 2^msb)/2^msb in [0,1); log2(1+m) via table.
    frac = (r - (1 << msb)).astype(np.float64) / (1 << msb).astype(np.float64)
    # accurate mantissa correction (HW: 32-entry LUT on top mantissa bits)
    lm = np.log2(1.0 + frac)                             # in [0,1)
    return ((msb.astype(np.float64) + lm) * (1 << UFRAC))

def render(cx, cy, period, warp, metric, duty, zoom, animate, out_mode,
           anim_phase=0.0, luma=None, ex=0, ey=0, dyn=0, inv=0):
    """One frame. period/warp/metric/duty/zoom/... in 0..1023 knob units.
    (cx,cy) is the CURRENT glided centre; (ex,ey) is the glide LAG (target-cx,
    target-cy) in px; dyn=1 enables the catch-up centre shift; inv=1 inverts."""
    xs = np.arange(W); ys = np.arange(H)
    X, Y = np.meshgrid(xs, ys)
    dx = (X - cx).astype(np.int64); dy = (Y - cy).astype(np.int64)

    # CATCH-UP (S8): nudge each pixel's (dx,dy) toward the target by a radius-
    # weighted slice of the lag BEFORE the metric, so the CORDIC measures from a
    # per-ring shifted centre -- inner rings lead (weight ~1), outer lag (~0).
    if dyn:
        adx = np.abs(dx); ady = np.abs(dy)
        rp = np.maximum(adx, ady) + (np.minimum(adx, ady) >> 1)   # ~circular proxy
        vw = np.clip(128 - (rp >> 3), 0, 128)            # Q7 weight (C_KW=3)
        dx = dx - ((vw * int(ex)) >> 7)
        dy = dy - ((vw * int(ey)) >> 7)

    a = np.abs(dx); b = np.abs(dy)
    hi = np.maximum(a, b); lo = np.minimum(a, b)

    # METRIC: TRUE Euclidean circle, computed in HW by a packed CORDIC magnitude
    # pipeline (G=5 guard bits, 10 iterations) -- shift-adds only, no multiply or
    # sqrt.  Model it exactly so the sheet matches the hardware.
    a = hi.astype(np.int64); b = lo.astype(np.int64)
    x = a << 5; y = b << 5
    for s in range(5):
        for i in (2 * s, 2 * s + 1):
            sx = x >> i; sy = y >> i; pos = y >= 0
            x, y = np.where(pos, x + sy, x - sy), np.where(pos, y - sx, y + sx)
    r = ((x << 8) + (x << 5) + (x << 4) + (x << 3)) >> 14        # * 312 >> 14
    r = np.maximum(r, 1)

    # frequency f from PERIOD and ZOOM, both exponential (log-linear in knob).
    # base: ~0.5 ring across the frame (f_lo) up to the alias limit (f_hi).
    Lf = (-11.0) + (period / 1023.0) * 9.0 + (zoom / 1023.0) * 4.0   # log2(f)
    f = 2.0 ** Lf

    # WARP (final HW map): p = K4/512, LINEAR -- 0 extreme tunnel .. 1 flat
    # (detent) .. 2 dome.  Deadband snaps flat (exact bypass).
    if abs(warp - 512) < 14:
        rp = r.astype(np.float64)                 # flat path (exact)
    else:
        p = warp / 512.0                          # 0 .. ~2
        lu = log2_fixed(r) - LREF_LOG * (1 << UFRAC)
        pe = (p * lu)
        rp = exp2_fixed(pe + LREF_LOG * (1 << UFRAC))
    rp = np.clip(rp, 0, 1 << 15)

    # SPIRAL (K5, re-uses the `metric` arg): twist the rings by the polar angle
    # (Vertigo).  0 = pure circles; the CORDIC already yields the angle in HW.
    ang = np.arctan2(dy.astype(np.float64), dx.astype(np.float64))   # -pi..pi
    spiral_k = (metric - 512) / 512.0                                # bipolar
    spiral = spiral_k * 8.0 * ang / (2 * np.pi)                      # up to +/-8 turns

    # banding duty, computed early so we can bias the centre into mid-band-A.
    D = np.clip(duty / 1023.0, 0.05, 0.95)

    # + D/2 puts phi=0 (the exact centre) in the MIDDLE of band A -> the inner
    # ring is a solid filled disc, not a band edge (which would grey one pixel).
    phi = rp * f + anim_phase + spiral + 0.5 * D
    frac = np.mod(phi, 1.0)

    # local frequency = |d phi / dx| for adaptive softening
    lfreq = np.abs(np.diff(phi, axis=1, prepend=phi[:, :1]))
    # signed distance to nearest band edge, peaks mid-band
    mA = np.minimum(frac, D - frac)                 # >0 inside band A
    mB = np.minimum(frac - D, 1.0 - frac)           # >0 inside band B
    m = np.where(frac < D, mA, -mB)                 # signed, 0 at edges

    base_gain = 6.0 if out_mode == 0 else 0.9       # Key vs Gradient
    adapt = np.clip(lfreq * 2.2, 0.0, 1.0)          # 0 sharp .. 1 near alias
    gain = base_gain * (1.0 - 0.85 * adapt) + 0.15
    key = np.clip(0.5 + m * gain, 0.0, 1.0)

    if inv:
        key = 1.0 - key                             # INVERT (S9): figure/ground swap

    yout = (key * 255).astype(np.uint8)
    return np.stack([yout, yout, yout], axis=-1)

def label(img, txt):
    from PIL import ImageDraw
    im = Image.fromarray(img); d = ImageDraw.Draw(im)
    d.rectangle([0, 0, len(txt) * 7 + 6, 14], fill=(0, 0, 0))
    d.text((3, 2), txt, fill=(255, 255, 0))
    return np.asarray(im)

if __name__ == "__main__":
    outdir = os.path.dirname(os.path.abspath(__file__))
    # default reference: center up & right, flat, 50% duty, hard edges, static
    cx, cy = W // 2, H // 2
    tiles = []
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 0), "centered (NO centre dot)"))
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 0, inv=1), "INVERT (S9)"))
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 0, ex=190, ey=130, dyn=0),
                       "uniform (S8 off)"))
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 0, ex=190, ey=130, dyn=1),
                       "CATCH-UP (S8 on)"))
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 0, ex=320, ey=0, dyn=1),
                       "catch-up X sweep"))
    tiles.append(label(render(cx, cy, 430, 512, 720, 512, 0, 0, 0, ex=220, ey=120, dyn=1),
                       "catch-up + spiral"))
    tiles.append(label(render(cx, cy, 430, 180, 512, 512, 200, 0, 0), "TUNNEL warp+zoom"))
    tiles.append(label(render(cx, cy, 430, 850, 512, 512, 0, 0, 0), "DOME warp"))
    tiles.append(label(render(cx, cy, 430, 512, 720, 512, 0, 0, 0), "SPIRAL (Vertigo)"))
    tiles.append(label(render(int(W*2.4), cy, 250, 512, 512, 512, 0, 0, 0), "center OFF-SCREEN (arc)"))
    tiles.append(label(render(int(W*3.2), int(H*2.0), 250, 512, 512, 512, 0, 0, 0), "far off (stripes)"))
    tiles.append(label(render(cx, cy, 430, 512, 512, 512, 0, 0, 1), "GRADIENT out"))
    # contact sheet 4 x 3
    cols = 4
    rows = (len(tiles) + cols - 1) // cols
    sheet = np.zeros((rows * H, cols * W, 3), np.uint8)
    for i, t in enumerate(tiles):
        rr, cc = divmod(i, cols)
        sheet[rr*H:(rr+1)*H, cc*W:(cc+1)*W] = t
    Image.fromarray(sheet).save(os.path.join(outdir, "proto_sheet.png"))
    print("wrote proto_sheet.png", sheet.shape)
    print("MANT table (64 x Q10):")
    print(", ".join(str(v) for v in MANT))
