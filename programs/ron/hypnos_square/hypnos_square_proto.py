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
           anim_phase=0.0, luma=None):
    """One frame. period/warp/metric/duty/zoom/... in 0..1023 knob units."""
    xs = np.arange(W); ys = np.arange(H)
    X, Y = np.meshgrid(xs, ys)
    dx = X - cx; dy = Y - cy
    a = np.abs(dx); b = np.abs(dy)
    hi = np.maximum(a, b); lo = np.minimum(a, b)

    # METRIC (final HW map): k = (1023-metric)/4 -> 255 diamond .. 0 square,
    # detent snapped to 128 (circle).  One subtract + shift.
    if abs(metric - 512) < 14:
        k = 128
    else:
        k = (1023 - metric) >> 2
    k = int(np.clip(k, 0, 255))
    r = hi + (k * lo) // 256
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

    # INPUT: luma displaces the radius (feed black -> uniform, invisible).
    if luma is not None:
        rp = rp + (luma.astype(np.float64) - 128.0) * 0.75

    # center dot: hold the innermost few pixels stable (tunnel singularity).
    phi = rp * f + anim_phase
    frac = np.mod(phi, 1.0)

    # local frequency = |d phi / dx| for adaptive softening
    lfreq = np.abs(np.diff(phi, axis=1, prepend=phi[:, :1]))

    # banding
    D = np.clip(duty / 1023.0, 0.05, 0.95)
    # signed distance to nearest band edge, peaks mid-band
    mA = np.minimum(frac, D - frac)                 # >0 inside band A
    mB = np.minimum(frac - D, 1.0 - frac)           # >0 inside band B
    m = np.where(frac < D, mA, -mB)                 # signed, 0 at edges

    base_gain = 6.0 if out_mode == 0 else 0.9       # Key vs Gradient
    adapt = np.clip(lfreq * 2.2, 0.0, 1.0)          # 0 sharp .. 1 near alias
    gain = base_gain * (1.0 - 0.85 * adapt) + 0.15
    key = np.clip(0.5 + m * gain, 0.0, 1.0)

    r2 = a * a + b * b
    key = np.where(r2 < 9, 0.0, key)                # stable center dot (band A)

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
    cx_r, cy_r = int(W * 0.72), int(H * 0.30)
    tiles = []
    tiles.append(label(render(cx_r, cy_r, 430, 512, 512, 512, 0, 0, 0), "REFERENCE target"))
    tiles.append(label(render(W//2, H//2, 430, 512, 512, 512, 0, 0, 0), "centered flat"))
    tiles.append(label(render(W//2, H//2, 430, 180, 512, 512, 200, 0, 0), "TUNNEL warp+zoom"))
    tiles.append(label(render(W//2, H//2, 430, 850, 512, 512, 0, 0, 0), "DOME warp"))
    tiles.append(label(render(W//2, H//2, 520, 512, 0,   512, 0, 0, 0), "DIAMOND metric"))
    tiles.append(label(render(W//2, H//2, 520, 512, 1023,512, 0, 0, 0), "SQUARE metric"))
    tiles.append(label(render(int(W*2.4), cy_r, 250, 512, 512, 512, 0, 0, 0), "center OFF-SCREEN (arc)"))
    tiles.append(label(render(int(W*3.2), int(H*2.0), 250, 512, 512, 512, 0, 0, 0), "far off (stripes)"))
    tiles.append(label(render(W//2, H//2, 430, 512, 512, 180, 0, 0, 0), "thin DUTY"))
    tiles.append(label(render(W//2, H//2, 430, 512, 512, 850, 0, 0, 0), "fat DUTY"))
    tiles.append(label(render(W//2, H//2, 430, 512, 512, 512, 0, 0, 1), "GRADIENT out"))
    tiles.append(label(render(W//2, H//2, 620, 180, 512, 512, 500, 0, 0), "deep tunnel (alias->gray)"))
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
