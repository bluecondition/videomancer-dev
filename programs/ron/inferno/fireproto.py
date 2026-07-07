#!/usr/bin/env python3
"""Bit-exact prototype of the new Inferno fire pipeline.

All ops integer, matching planned VHDL stages:
  world coords (1/16 px) -> warp noise -> domain-warped 3-octave value noise
  -> vertical gradient shaping -> heat -> blackbody palette.
"""
import numpy as np
from PIL import Image
import sys

W, H = 1280, 720

# ---------------------------------------------------------------- hash
def rol12(h, n):
    return ((h << n) | (h >> (12 - n))) & 0xFFF

def hash8(cx, cy, seed):
    """cx, cy: integer arrays (cell coords). Returns 0..255. 12-bit state
    (folded from the 16-bit coord pair), 2 adds for carry nonlinearity."""
    x = (cx & 0xFF).astype(np.uint32)
    y = (cy & 0xFF).astype(np.uint32)
    h = (((x << 8) | y) ^ seed) & 0xFFFF
    h = (h ^ (h >> 4)) & 0xFFF
    h = (h + rol12(h, 5)) & 0xFFF
    h ^= rol12(h, 9)
    h = (h + rol12(h, 3)) & 0xFFF
    h ^= (h >> 5)
    return (h >> 2) & 0xFF

# ---------------------------------------------------------------- smoothstep LUT (4-bit frac -> 0..16)
SS = np.array([round(16 * (3 * ((f + 0.5) / 16) ** 2 - 2 * ((f + 0.5) / 16) ** 3))
               for f in range(16)], dtype=np.int32)

def vnoise(wx, wy, sx, sy, seed):
    """Value noise. wx, wy in 1/16 px. Cell size 2^sx by 2^sy pixels."""
    cx = (wx >> (sx + 4)).astype(np.int64)
    cy = (wy >> (sy + 4)).astype(np.int64)
    fx = SS[(wx >> sx) & 15]
    fy = SS[(wy >> sy) & 15]
    h00 = hash8(cx,     cy,     seed).astype(np.int32)
    h10 = hash8(cx + 1, cy,     seed).astype(np.int32)
    h01 = hash8(cx,     cy + 1, seed).astype(np.int32)
    h11 = hash8(cx + 1, cy + 1, seed).astype(np.int32)
    top = h00 + (((h10 - h00) * fx) >> 4)
    bot = h01 + (((h11 - h01) * fx) >> 4)
    return top + (((bot - top) * fy) >> 4)     # 0..255

# ---------------------------------------------------------------- palette
def make_palette():
    t = np.arange(256) / 255.0
    # v2: black -> ember -> yellow -> ORANGE (dominant) -> pale blue -> white
    tp = [0.00, 0.12, 0.30, 0.50, 0.75, 0.92, 1.00]
    rp = [0.00, 0.25, 0.55, 0.95, 1.00, 1.00, 1.00]
    gp = [0.00, 0.01, 0.03, 0.12, 0.55, 0.85, 0.93]
    bp = [0.00, 0.00, 0.00, 0.01, 0.02, 0.06, 0.12]
    r = np.interp(t, tp, rp)
    g = np.interp(t, tp, gp)
    b = np.interp(t, tp, bp)
    return r, g, b

def rgb_to_yuv10(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = 0.564 * (b - y)
    cr = 0.713 * (r - y)
    Y = np.clip(np.round(y * 1023), 0, 1023).astype(np.int32)
    U = np.clip(np.round(512 + cb * 1023), 0, 1023).astype(np.int32)
    V = np.clip(np.round(512 + cr * 1023), 0, 1023).astype(np.int32)
    return Y, U, V

# ---------------------------------------------------------------- fire frame
def render(frame, fuel=512, scale=512, smoke_k=512, turb=512, rise=512,
           seed0=0x9E37, o1sy=7, dither=3):
    py, px = np.mgrid[0:H, 0:W]
    t = frame * (rise >> 3)          # yoff units (1/16 px) per frame: rise=512 -> 4 px/frame
    wx = (px << 4)
    wy = (py << 4) + t               # world rises: sample coord increases

    # --- domain warp: two low-freq noises offset x and y
    wyw = (py << 4) + ((t * 3) >> 2)  # warp field rises slightly slower
    nwx = vnoise(wx, wyw, 6, 6, 0x517C)
    nwy = vnoise(wx + 12345, wyw + 7777, 6, 6, 0xA342)
    turb4 = turb >> 6                # 0..15
    dx = ((nwx - 128) * turb4) >> 1  # +-960 units = +-60 px at max
    dy = ((nwy - 128) * turb4) >> 2

    ax = wx + dx
    ay = wy + dy

    # --- 3 octaves, vertically stretched 2:1
    n1 = vnoise(ax, ay, 5, o1sy, 0x1234)
    n2 = vnoise(ax, ay, 4, 5, 0x5678)
    n3 = vnoise(ax, ay + (t >> 1), 3, 4, 0x9ABC)  # fine octave rises 1.5x faster
    ridge3 = 255 - (np.abs(n3 - 128) << 1)
    fbm = (n1 * 9 + n2 * 5 + ridge3 * 2) >> 4     # 0..255

    # --- vertical gradient: g at bottom = gbot, 1 unit per line (servo in HW)
    gbot = 260 + (fuel >> 1)                       # 260..772
    g = gbot - (H - 1 - py)

    # --- heat: gradient minus noise deficit; raggedness coupled to fuel
    heatk = 10 + (fuel >> 6)                       # 10..25
    cut = ((255 - fbm) * heatk) >> 3
    raw = g - cut
    if dither:
        rng = np.random.RandomState(42)  # stands in for per-pixel LFSR bits
        raw = raw + rng.randint(-dither, dither + 1, raw.shape)
    # soft knee above 208 (compress by 8) -> white only at extreme peaks
    heat = np.clip(np.where(raw > 208, 208 + ((raw - 208) >> 3), raw), 0, 255)

    # --- smoke: originates at the tips, rises off-screen, dissipates with
    # altitude: weak wisps die low, strong wisps survive high
    S = smoke_k >> 2
    d = -raw
    ramp_in = np.clip((d + 16) << 2, 0, 255)
    wisp = np.clip((fbm - 110) << 1, 0, 240)
    base = (255 - S) >> 1                  # knob 0 -> kills all smoke
    att = np.clip((d >> 3) + base, 0, 255)
    smoke = np.clip(np.minimum(np.minimum(ramp_in, wisp), 120) - att, 0, 255)
    return heat.astype(np.int32), smoke.astype(np.int32)

def to_rgb(heat, smoke=None):
    r, g, b = make_palette()
    rr = np.clip(np.round(r * 255), 0, 255).astype(np.int32)
    gg = np.clip(np.round(g * 255), 0, 255).astype(np.int32)
    bb = np.clip(np.round(b * 255), 0, 255).astype(np.int32)
    img = np.stack([rr[heat], gg[heat], bb[heat]], -1)
    if smoke is not None:
        luma = (img[..., 0]*30 + img[..., 1]*59 + img[..., 2]*11) // 100
        m = smoke > luma
        img[m] = np.stack([smoke, smoke, smoke], -1)[m]
    return img.astype(np.uint8)

if __name__ == '__main__':
    frames = [int(a) for a in sys.argv[1:]] or [0, 8, 16]
    for f in frames:
        heat, smoke = render(f)
        Image.fromarray(to_rgb(heat, smoke)).save(f'fire_f{f:03d}.png')
        print(f'frame {f} done, heat range {heat.min()}..{heat.max()}')
