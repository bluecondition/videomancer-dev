#!/usr/bin/env python3
"""FRINGE -- bit-accurate model of the moire engine.

Mirrors the VHDL integer pipeline exactly -- verified PIXEL-EXACT against
GHDL renders of the RTL across all ten textures, both invert modes, both
backgrounds, both draw orders and a glided phase frame.  Use it to eyeball
texture maths and moire behaviour without a bitstream build, and as the
reference when chasing an RTL difference.

  python3 fringe_model.py            # contact sheet of all 10 textures
  python3 fringe_model.py boot       # the power-up default frame
  python3 fringe_model.py moire      # a few performance snapshots

Phase units: 1024 units = one pattern period.  Accumulators carry 8
fractional bits (Q8), exactly like the hardware DDAs.
"""

import sys
import numpy as np

W, H = 1280, 720                      # 720p model frame
PH = 1024                             # phase units per period
Q = 8                                 # fractional bits in the accumulators

TEX_NAMES = ["Circles", "Squares", "Spokes", "Wavy H", "Wavy V",
             "Diagonal", "Grid", "Hex", "Checker", "Brick"]
N_TEX = 10

# ---------------------------------------------------------------- frame terms


def scale_step(k):
    """Glided scale knob (0..1023) -> phase units per pixel, Q8.

    mantissa/exponent split: e = k>>7, m = k(6:0); step = (256 + 2m) << e.
    Period in pixels = 1024*256/step, i.e. 1024 px (k=0) .. 4.0 px (k=1023),
    continuous and monotonic across the whole knob.
    """
    e = k >> 7
    m = k & 127
    return (256 + 2 * m) << e


def period_px(step):
    return PH * (1 << Q) / step


def spoke_count(k):
    """Scale knob -> spoke count for the radial-spokes texture."""
    return 4 + ((k * 60) >> 10)          # 4 .. 63


# CORDIC gain undo (K = 1.6468) and the 45-degree projection, as the
# per-frame constant fiddles the hardware does in the vblank sequencer.
def step_radial(step):
    return (step * 39) >> 6              # * 0.6094 ~ 1/1.6468


def step_diag(step):
    return (step * 181) >> 8             # * 0.7070 ~ 1/sqrt(2)


# ------------------------------------------------------------------- CORDIC

C_NS = 4                                 # pipeline stages, 2 iterations each
C_CG = 3                                 # guard bits -- must match the VHDL
C_ATAN = [512, 302, 160, 81, 41, 20, 10, 5, 3, 1]    # atan(2^-i), 4096 = turn


def cordic(dx, dy):
    """Vectoring CORDIC: returns (r_scaled_by_K, angle 0..4095).

    Bit-identical to the VHDL: octant fold, 2*C_NS iterations, guard bits.
    """
    a, b = np.abs(dx), np.abs(dy)
    hi = np.maximum(a, b).astype(np.int64)
    lo = np.minimum(a, b).astype(np.int64)
    swap = (b > a)
    x = hi << C_CG
    y = lo << C_CG
    z = np.zeros_like(x)
    for i in range(2 * C_NS):
        pos = y >= 0
        xs = np.where(pos, x + (y >> i), x - (y >> i))
        ys = np.where(pos, y - (x >> i), y + (x >> i))
        z = np.where(pos, z + C_ATAN[i], z - C_ATAN[i])
        x, y = xs, ys
    r = x >> C_CG                       # K * hypot, gain undone in step_radial
    mag = np.where(swap, 1024 - z, z)   # fold [0,45) -> [0,90)
    ang = np.where((dx >= 0) & (dy >= 0), mag,
          np.where((dx < 0) & (dy >= 0), 2048 - mag,
          np.where((dx < 0) & (dy < 0), 2048 + mag, -mag)))
    return r, ang & 4095


# --------------------------------------------------------------- sine table

# Quarter-wave table with a half-sample offset, folded exactly as f_sin()
# does in the VHDL: index bit 6 mirrors, bit 7 negates.
_SINQ = [int(round(127 * np.sin(2 * np.pi * (i + 0.5) / 256))) for i in range(64)]
SIN8 = np.array([(-1 if (a >> 7) & 1 else 1) *
                 _SINQ[(a & 63) if not ((a >> 6) & 1) else 63 - (a & 63)]
                 for a in range(256)], dtype=np.int64)


# ------------------------------------------------------------------ textures

DUTY = 512          # grating ink duty, 50%
W_GRID = 192        # square-grid line half-width, phase units (18.75%)
W_HEXV = 84         # honeycomb vertical-edge half-width
W_HEXD = 190        # honeycomb slanted-edge half-width (gradient sqrt(5))
W_MORT = 128        # brick mortar half-width
A_WAVE = 1          # sine displacement shift: +/-127<<1 = +/-254 units


def layer_mask(tex, xs, ys, cx, cy, step, offx, offy, rot):
    """One layer's ink mask.  xs/ys are pixel coordinate grids."""
    stepc = step_diag(step) if tex == 5 else step

    # --- cartesian DDA accumulators (exact signed scaled coordinates, Q8)
    px = (xs - cx - offx) * stepc
    py = (ys - cy - offy) * stepc
    ph_x = (px >> Q) & (PH - 1)          # 10-bit stripe phase
    ph_y = (py >> Q) & (PH - 1)
    full_x = (px >> Q) & (4 * PH - 1)    # 12-bit window: cell indices
    full_y = (py >> Q) & (4 * PH - 1)

    if tex == 0:                          # concentric circles
        dx, dy = xs - cx - offx, ys - cy - offy
        r, _ = cordic(dx, dy)
        p = ((r * step_radial(step)) >> Q) & (PH - 1)
        return p < DUTY

    if tex == 1:                          # concentric squares
        p = ((np.maximum(np.abs(px), np.abs(py))) >> Q) & (PH - 1)
        return p < DUTY

    if tex == 2:                          # radial spokes -- centred, phase rotates
        dx, dy = xs - cx, ys - cy
        _, ang = cordic(dx, dy)
        n64 = spoke_count_of_step(step) * 64
        p = ((((ang + rot) & 4095) * n64) >> Q) & (PH - 1)
        return p < DUTY

    if tex == 3:                          # horizontal lines, sine-displaced in y
        s = SIN8[(px >> (Q + 4)) & 255]
        p = (ph_y + (s << A_WAVE)) & (PH - 1)
        return p < DUTY

    if tex == 4:                          # vertical stripes, sine-displaced in x
        s = SIN8[(py >> (Q + 4)) & 255]
        p = (ph_x + (s << A_WAVE)) & (PH - 1)
        return p < DUTY

    if tex == 5:                          # 45-degree parallel lines
        p = ((px + py) >> Q) & (PH - 1)
        return p < DUTY

    if tex == 6:                          # square grid
        return (ph_x < W_GRID) | (ph_y < W_GRID)

    if tex == 7:                          # honeycomb (hex Voronoi of a brick lattice)
        row = (full_y >> 10) & 1
        u = ((full_x + (row << 9)) & (PH - 1)) - 512     # -512..511 in cell
        v = ph_y - 512
        a, b = np.abs(u), np.abs(v)
        d = a + 2 * b - 1280                             # slanted-edge bisector
        vert = (np.abs(a - 512) < W_HEXV) & (d < 0)
        slant = np.abs(d) < W_HEXD
        return vert | slant

    if tex == 8:                          # checkerboard
        return ((ph_x >> 9) ^ (ph_y >> 9)) == 1

    # tex == 9                            # running-bond brick, mortar line-work
    row = (full_y >> 10) & 1
    u = (full_x + (row << 10)) & (2 * PH - 1)            # 2:1 bricks
    return (ph_y < W_MORT) | (u < W_MORT)


_SPOKE_LUT = {}


def spoke_count_of_step(step):
    return _SPOKE_LUT.get(step, 16)


# ------------------------------------------------------------------ composite

def render(texA, kscA, kphA, texB, kscB, kphB, kslider,
           invA=0, invB=0, swap=0, bg_white=1, colour_mode=0):
    """Full frame.  Returns a HxW uint8 luma image (0 or 255)."""
    ys, xs = np.meshgrid(np.arange(H, dtype=np.int64),
                         np.arange(W, dtype=np.int64), indexing="ij")
    cx, cy = W // 2, H // 2

    stepA, stepB = scale_step(kscA), scale_step(kscB)
    _SPOKE_LUT[stepA] = spoke_count(kscA)
    _SPOKE_LUT[stepB] = spoke_count(kscB)

    offxA = (kphA - 512) * 2                 # +/-1024 px of translation
    offxB = (kphB - 512) * 2
    offy_top = (kslider - 512) * 2
    rotA = ((kphA - 512) * 4) & 4095         # spokes: X phase == rotation
    rotB = ((kphB - 512) * 4) & 4095

    top_is_B = (swap == 0)                   # default draw order: B on top
    offyA = offy_top if not top_is_B else 0
    offyB = offy_top if top_is_B else 0
    if texA == 2:
        rotA = (rotA - offyA) & 4095         # spokes: Y phase counter-rotates
    if texB == 2:
        rotB = (rotB - offyB) & 4095

    mA = layer_mask(texA, xs, ys, cx, cy, stepA, offxA, offyA, rotA)
    mB = layer_mask(texB, xs, ys, cx, cy, stepB, offxB, offyB, rotB)

    if colour_mode == 0:                     # mask invert
        mA, mB = mA ^ bool(invA), mB ^ bool(invB)
        cA = cB = 0
    else:                                    # colour flip
        cA, cB = (255 if invA else 0), (255 if invB else 0)

    bg = 255 if bg_white else 0
    if top_is_B:
        (mt, ct), (mb_, cb) = (mB, cB), (mA, cA)
    else:
        (mt, ct), (mb_, cb) = (mA, cA), (mB, cB)

    out = np.full((H, W), bg, dtype=np.uint8)
    out = np.where(mb_, cb, out)
    out = np.where(mt, ct, out)
    return out.astype(np.uint8)


# ---------------------------------------------------------------------- main

def save(img, path):
    try:
        from PIL import Image
        Image.fromarray(img).save(path)
    except ImportError:
        with open(path.replace(".png", ".pgm"), "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())
        path = path.replace(".png", ".pgm")
    print("wrote", path)


def contact_sheet():
    """All 10 textures at a mid scale, single layer, on white."""
    tiles = []
    for t in range(N_TEX):
        img = render(t, 640, 512, t, 640, 512, 512, bg_white=1,
                     swap=1)             # A on top, B identical underneath
        tiles.append(img[::4, ::4])      # 320x180 thumbnails
    rows = [np.hstack(tiles[i:i + 5]) for i in (0, 5)]
    sheet = np.vstack(rows)
    save(sheet, "model_textures.png")
    for t in range(N_TEX):
        print("%d %-9s period %.1f px  spokes %d" %
              (t, TEX_NAMES[t], period_px(scale_step(640)), spoke_count(640)))


def boot_frame():
    img = render(0, 512, 512, 0, 560, 512, 512)
    save(img, "model_boot.png")
    print("A period %.1f px, B period %.1f px" %
          (period_px(scale_step(512)), period_px(scale_step(560))))


def moire_set():
    shots = [
        ("rings",   dict(texA=0, kscA=512, kphA=512, texB=0, kscB=560, kphB=512, kslider=512)),
        ("offset",  dict(texA=0, kscA=512, kphA=512, texB=0, kscB=560, kphB=560, kslider=640)),
        ("grating", dict(texA=4, kscA=800, kphA=512, texB=5, kscB=810, kphB=512, kslider=512)),
        ("spokes",  dict(texA=2, kscA=700, kphA=512, texB=2, kscB=712, kphB=560, kslider=512)),
        ("hexgrid", dict(texA=7, kscA=760, kphA=512, texB=6, kscB=770, kphB=520, kslider=560)),
        ("brickck", dict(texA=9, kscA=740, kphA=512, texB=8, kscB=752, kphB=512, kslider=530)),
    ]
    for name, kw in shots:
        save(render(**kw), "model_moire_%s.png" % name)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "sheet"
    if mode == "boot":
        boot_frame()
    elif mode == "moire":
        moire_set()
    else:
        contact_sheet()
