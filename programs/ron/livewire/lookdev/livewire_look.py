#!/usr/bin/env python3
"""LIVEWIRE look-dev model.

A numpy model of the LIVEWIRE pipeline, written to mirror the hardware
architecture rather than to be a free shader: one Sobel over a blended luma
pyramid, a 16-bin orientation, a decimated frame-recursive field, and eight
modes built from a shared primitive bank.  Everything here has a known cheap
HX4K implementation; nothing uses a frame buffer or a per-pixel divide that
isn't a small ROM.

Colour is authored in standard BT.601 10-bit YUV (Y 64..940, chroma centred
512) exactly as the VHDL will be, and converted to RGB only for the preview.
"""
import argparse
import numpy as np
from PIL import Image

# ----------------------------------------------------------------- constants
W_CELLS, H_CELLS = 128, 64          # field grid = 8192 words (power-of-2 addr)
FIELD_LEVELS = 31                   # 5-bit field word
Y_MIN, Y_MAX, Y_CAP = 64.0, 940.0, 768.0   # CLAUDE: cap drawn luma at ~768
CH = 512.0                          # chroma centre


# ------------------------------------------------------------------- helpers
def ramp(x, a, b):
    return np.clip((x - a) / max(b - a, 1e-6), 0.0, 1.0)


def blur121(a):
    """separable [1 2 1]/4 in both axes -- the window's L1 level."""
    p = np.pad(a, 1, mode="edge")
    h = (p[:, :-2] + 2.0 * p[:, 1:-1] + p[:, 2:]) * 0.25
    v = (h[:-2, :] + 2.0 * h[1:-1, :] + h[2:, :]) * 0.25
    return v


def sobel(a):
    p = np.pad(a, 1, mode="edge")
    gx = (p[:-2, 2:] + 2.0 * p[1:-1, 2:] + p[2:, 2:]
          - p[:-2, :-2] - 2.0 * p[1:-1, :-2] - p[2:, :-2])
    gy = (p[2:, :-2] + 2.0 * p[2:, 1:-1] + p[2:, 2:]
          - p[:-2, :-2] - 2.0 * p[:-2, 1:-1] - p[:-2, 2:])
    return gx, gy


def octagonal(gx, gy):
    """max(|a|,|b|) + min(|a|,|b|)/2 -- isotropic, adds and shifts only."""
    a, b = np.abs(gx), np.abs(gy)
    return np.maximum(a, b) + 0.5 * np.minimum(a, b)


def coarse_causal(a, hw, vh):
    """The coarse level as the RASTER can actually build it.

    Horizontally symmetric (the whole line is in hand), vertically CAUSAL --
    a symmetric vertical blur of radius R needs R lines of lookahead, and
    delaying the picture by R lines would cost R full-res line buffers the
    part does not have.  So the vertical box covers rows y-vh+1..y and its
    centroid sits ~vh/2 lines ABOVE the pixel: coarse features are offset
    upward by that much.  This function exists to show what that costs.
    """
    c = np.cumsum(np.pad(a, ((0, 0), (hw, hw)), mode="edge"), axis=1)
    h = (c[:, 2 * hw:] - c[:, :-2 * hw]) / (2 * hw)
    c2 = np.cumsum(np.pad(h, ((vh, 0), (0, 0)), mode="edge"), axis=0)
    return (c2[vh:, :] - c2[:-vh, :]) / vh


def cell_dims(H, W):
    return -(-H // H_CELLS), -(-W // W_CELLS)      # ceil


def cell_reduce(a, how="max"):
    H, W = a.shape
    ch, cw = cell_dims(H, W)
    pad = np.pad(a, ((0, H_CELLS * ch - H), (0, W_CELLS * cw - W)), mode="edge")
    r = pad.reshape(H_CELLS, ch, W_CELLS, cw)
    return r.max(axis=(1, 3)) if how == "max" else r.mean(axis=(1, 3))


def upsample(field, H, W):
    """bilinear from the cell grid -- lerp weights are just the low bits of x,y."""
    ch, cw = cell_dims(H, W)
    ux = np.arange(W) / cw - 0.5
    uy = np.arange(H) / ch - 0.5
    i0 = np.clip(np.floor(ux).astype(int), 0, W_CELLS - 1)
    j0 = np.clip(np.floor(uy).astype(int), 0, H_CELLS - 1)
    i1 = np.clip(i0 + 1, 0, W_CELLS - 1)
    j1 = np.clip(j0 + 1, 0, H_CELLS - 1)
    tx = np.clip(ux - np.floor(ux), 0, 1)[None, :]
    ty = np.clip(uy - np.floor(uy), 0, 1)[:, None]
    a = field[np.ix_(j0, i0)]
    b = field[np.ix_(j0, i1)]
    c = field[np.ix_(j1, i0)]
    d = field[np.ix_(j1, i1)]
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty


def shift(f, dy, dx):
    out = np.zeros_like(f)
    ys = slice(max(dy, 0), H_CELLS + min(dy, 0))
    xs = slice(max(dx, 0), W_CELLS + min(dx, 0))
    yd = slice(max(-dy, 0), H_CELLS + min(-dy, 0))
    xd = slice(max(-dx, 0), W_CELLS + min(-dx, 0))
    out[ys, xs] = f[yd, xd]
    return out


def field_step(F, inject, spread, persist):
    """exactly one frame of the recursion -- this is the hardware's per-frame work."""
    n = np.maximum.reduce([shift(F, 1, 0), shift(F, -1, 0),
                           shift(F, 0, 1), shift(F, 0, -1)])
    F = np.maximum(inject, np.maximum(spread * n, persist * F))
    return np.round(F * FIELD_LEVELS) / FIELD_LEVELS


def field_settle(inject, spread, persist, iters=40, aniso=None):
    """F = max(inject, spread*max(neighbours), persist*F), quantised to 5 bits.

    On hardware this is one frame per iteration; on a still it settles.  aniso,
    if given, is (wx, wy) per-cell tap weights biasing the bleed along the local
    stroke direction.
    """
    F = np.zeros_like(inject)
    for _ in range(iters):
        n = np.maximum.reduce([shift(F, 1, 0), shift(F, -1, 0),
                               shift(F, 0, 1), shift(F, 0, -1)])
        if aniso is not None:
            wx, wy = aniso
            n = np.maximum.reduce([
                shift(F, 1, 0) * wy, shift(F, -1, 0) * wy,
                shift(F, 0, 1) * wx, shift(F, 0, -1) * wx])
        F = np.maximum(inject, np.maximum(spread * n, persist * F))
        F = np.round(F * FIELD_LEVELS) / FIELD_LEVELS
    return F


def hash2(xi, yi, salt):
    h = (xi.astype(np.int64) * 374761393 + yi.astype(np.int64) * 668265263
         + salt * 2654435761)
    h = (h ^ (h >> 13)) * 1274126177
    h = h ^ (h >> 16)
    return (h & 0xFFFF) / 65535.0


# ------------------------------------------------------------------- colour
def hue_uv(h, sat):
    """h in turns, sat 0..1 -> (U, V) about 512, standard BT.601 axes."""
    a = 2.0 * np.pi * h
    return CH + 448.0 * sat * np.cos(a), CH + 448.0 * sat * np.sin(a)


def uv_of(rgb, sat=1.0):
    """(U,V) for a named colour, at a fraction of its natural saturation."""
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return CH + 896.0 * (b - y) / 1.772 * sat, CH + 896.0 * (r - y) / 1.402 * sat


PAL = dict(
    cyan=(0.35, 1.0, 1.0), violet=(0.62, 0.30, 1.0), magenta=(1.0, 0.25, 0.85),
    white=(1.0, 1.0, 1.0), amber=(1.0, 0.62, 0.10), red=(1.0, 0.10, 0.10),
    green=(0.25, 1.0, 0.35), blue=(0.20, 0.45, 1.0), pink=(1.0, 0.42, 0.72),
    sepia=(0.42, 0.26, 0.12), indigo=(0.16, 0.18, 0.52), black=(0.1, 0.1, 0.1),
    copper=(1.0, 0.55, 0.32), brass=(1.0, 0.80, 0.36), steel=(0.86, 0.90, 1.0),
    gold=(1.0, 0.78, 0.22), paper=(1.0, 0.985, 0.94),
)


def pal_lerp(names, t, sat=1.0):
    """continuous K6 walk along a list of named colours."""
    t = float(np.clip(t, 0, 0.999)) * (len(names) - 1)
    i = int(t)
    a = np.array(PAL[names[i]])
    b = np.array(PAL[names[min(i + 1, len(names) - 1)]])
    return uv_of(tuple(a + (b - a) * (t - i)), sat)


def yuv_to_rgb(Y, U, V):
    y = (Y - 64.0) / 876.0
    cb = (U - 512.0) / 896.0
    cr = (V - 512.0) / 896.0
    r = y + 1.402 * cr
    g = y - 0.344136 * cb - 0.714136 * cr
    b = y + 1.772 * cb
    return np.clip(np.stack([r, g, b], -1), 0, 1)


def rgb_to_yuv10(rgb):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return 64.0 + 876.0 * y, 512.0 + 896.0 * cb, 512.0 + 896.0 * cr


class Layer:
    """alpha-over on chroma, additive-with-clamp on luma -- the compose stage."""

    def __init__(self, Y, U, V):
        self.Y, self.U, self.V = Y, U, V

    def over(self, a, Y, U, V, additive=False):
        a = np.clip(a, 0, 1)
        if additive:
            self.Y = np.minimum(self.Y + a * (Y - Y_MIN), Y_CAP)
        else:
            self.Y = self.Y * (1 - a) + Y * a
        self.U = self.U * (1 - a) + U * a
        self.V = self.V * (1 - a) + V * a
        return self

    def rgb(self):
        return yuv_to_rgb(np.clip(self.Y, Y_MIN, Y_MAX), self.U, self.V)


# --------------------------------------------------------------- parameters
class P:
    """One control snapshot.  Knob/switch names are the panel names."""

    def __init__(self, mode=0, scale=0.5, thresh=0.5, material=0.5, light=0.5,
                 colour=0.5, emerge=0.6, invert=False, interior_video=False,
                 single_pol=False, spectral_overlay=False, persist=0.0):
        self.mode, self.scale, self.thresh = mode, scale, thresh
        self.material, self.light, self.colour = material, light, colour
        self.emerge = emerge
        self.invert, self.interior_video = invert, interior_video
        self.single_pol, self.spectral_overlay = single_pol, spectral_overlay
        self.persist = persist
        self.scale_bias = MODE_SCALE[mode] if mode < len(MODE_SCALE) else 0.5

    # --- EMERGE -> the derived per-frame constants every mode reads
    @property
    def d(self):
        e = self.emerge
        return dict(
            # threshold: silhouettes only at the bottom, everything by 60%
            # the first quarter of the sweep drops the threshold fast enough that
            # the strongest silhouettes are already drawing at ~12%
            thr=(0.105 + 0.450 * (1 - self.thresh))
                * (1 - 0.55 * ramp(e, 0.03, 0.25) - 0.36 * ramp(e, 0.25, 0.70)) + 0.014,
            # coarse bias: EMERGE forces coarse scale early, releases to K2
            scale=np.clip(self.scale_bias + 0.9 * (self.scale - 0.5)
                          + (1 - ramp(e, 0.05, 0.45)), 0, 1),
            # weight and spread, with overdrive past 75%
            weight=(0.25 + 0.75 * self.material) * (0.45 + 0.55 * ramp(e, 0.25, 0.78))
                   * (1.0 + 2.2 * ramp(e, 0.75, 1.0)),
            spread=np.clip((0.34 + 0.30 * self.material) + 0.22 * ramp(e, 0.30, 1.0), 0, 0.90),
            interior=ramp(e, 0.22, 0.55),
            over=ramp(e, 0.75, 1.0),
            # EMERGE 0 must be bit-exact dry video: one final lerp, dry at t=0
            wet=ramp(e, 0.0, 0.05),
        )


# ------------------------------------------------------------- core features
def features(img_rgb, p):
    Y10, U10, V10 = rgb_to_yuv10(img_rgb)
    y8 = np.clip((Y10 - 64.0) * (255.0 / 876.0), 0, 255)
    H, W = y8.shape
    d = p.d

    # --- SCALE: a luma pyramid blended BEFORE one Sobel
    L0 = y8
    L1 = blur121(y8)
    L2 = coarse_causal(L1, 8, 16) if COARSE_CAUSAL else \
        upsample(cell_reduce(blur121(L1), "mean"), H, W)
    s = d["scale"] * 2.0
    if s <= 1.0:
        Ls = L0 * (1 - s) + L1 * s
    else:
        Ls = L1 * (2 - s) + L2 * (s - 1)

    gx, gy = sobel(Ls)
    # blurring the source shrinks its gradient, so each scale carries a
    # normalising gain (a per-frame shift on hardware) -- SCALE then changes
    # WHICH structures are seen, not how strong the edges are
    sgain = 1.0 + 3.4 * d["scale"] ** 1.3
    mag = octagonal(gx, gy) / 1020.0 * sgain
    ang = np.arctan2(gy, gx) / (2 * np.pi) % 1.0        # 0..1 turns, signed = polarity
    bin16 = np.floor(ang * 16 + 0.5).astype(int) % 16
    angq = bin16 / 16.0

    # fine and coarse magnitudes for corner/detail decisions
    fgx, fgy = sobel(L0)
    # the coarse gradient is taken BETWEEN CELLS and then interpolated -- taking
    # it from the upsampled luma instead facets every coarse edge into stair steps
    if COARSE_CAUSAL:
        cgx, cgy = sobel(L2)
        cgx, cgy = cgx * 7.0, cgy * 7.0      # per-pixel slope of a wide blur
    else:
        cellL = cell_reduce(blur121(L1), "mean")
        cgx = upsample(sobel(cellL)[0], H, W)
        cgy = upsample(sobel(cellL)[1], H, W)
    mag_f = octagonal(fgx, fgy) / 1020.0 * 0.85
    mag_c = octagonal(cgx, cgy) / 1020.0 * 0.95

    # --- polarity: S3 keeps one half of the wheel
    pol_gate = np.ones_like(mag)
    if p.single_pol:
        pol_gate = np.clip(0.15 + 1.6 * np.maximum(np.cos(2 * np.pi * (ang - p.light)), 0), 0, 1)

    # --- the soft alpha ramp: continuous magnitude IS the anti-aliasing
    thr = d["thr"]
    # WEIGHT thickens lines by moving the base threshold down (lagoon), and
    # every band of the material is expressed as a MULTIPLE of that threshold:
    # k=0.6 outer glow, k=1 body, k=3 core.  aa is the soft AA shoulder.
    thr_w = np.maximum(thr * (1.30 - 0.80 * np.clip(d["weight"], 0, 1.3)), 0.010)

    def edge_alpha(k=1.0, aa=0.55, src=None):
        m = mag if src is None else src
        t = thr_w * k
        return np.clip((m - t) / (aa * t), 0, 1) * pol_gate

    # --- the field: injected per cell, diffused a cell per frame, decayed
    fthr, fspr = MODE_FIELD[p.mode]
    inj = cell_reduce(np.clip((mag_c - thr * fthr) / max(thr * 1.6, 1e-3), 0, 1))
    # PERSIST does not ADD to the halo -- it changes what the field IS.  With
    # persistence on, neighbour coupling is cut back and the cell's own history
    # takes over, so the glow tightens and a comet wake appears behind motion.
    # (Both terms feed the same max, so a big halo would simply swallow a wake.)
    persist = 0.93 * p.persist
    spr = float(np.clip(d["spread"] + fspr, 0.2, 0.94)) * (1.0 - 0.45 * p.persist)
    st = getattr(p, "field_state", None)
    if st is None:                       # a still: settle to the steady state
        F = field_settle(inj, spr, persist)
    else:                                # a moving sequence: one frame per call
        F = field_step(st, inj, spr, persist)
        p.field_state = F
    fu = np.clip(upsample(F, H, W), 0, 1)

    inj_up = np.clip(upsample(inj, H, W), 0, 1)
    ch_, cw_ = cell_dims(H, W)
    cgx_c = cell_reduce(gx, "mean")
    cgy_c = cell_reduce(gy, "mean")
    cmag = np.repeat(np.repeat(octagonal(cgx_c, cgy_c) / 1020.0 * 4.4,
                               ch_, axis=0), cw_, axis=1)[:H, :W]
    cang = np.repeat(np.repeat(
        (np.arctan2(cgy_c, cgx_c) / (2 * np.pi)) % 1.0,
        cell_dims(H, W)[0], axis=0), cell_dims(H, W)[1], axis=1)[:H, :W]
    sang = (np.arctan2(cgy, cgx) / (2 * np.pi)) % 1.0        # smooth form angle
    cu = np.clip(upsample(cell_reduce(U10, "mean"), H, W), 0, 1023)
    cv = np.clip(upsample(cell_reduce(V10, "mean"), H, W), 0, 1023)
    gpx = octagonal(*sobel(L1)) / 1020.0
    return dict(S=W / 1920.0, inj_up=inj_up, cmag=cmag, sang=sang, gpx=gpx, cang=cang, cu=cu, cv=cv,
                H=H, W=W, y8=y8, L1=L1, L2=L2, Y10=Y10, U10=U10, V10=V10,
                gx=gx, gy=gy, mag=mag, mag_f=mag_f, mag_c=mag_c, ang=ang,
                angq=angq, bin16=bin16, edge_alpha=edge_alpha, thr=thr, thr_w=thr_w,
                F=F, fu=fu, d=d, pol_gate=pol_gate)


def coords(f):
    yy, xx = np.mgrid[0:f["H"], 0:f["W"]]
    return xx.astype(np.float64), yy.astype(np.float64)


COARSE_CAUSAL = True          # model the raster's real vertical asymmetry
MODE_SCALE = [0.54, 0.62, 0.90, 0.46, 0.86, 0.34, 0.58, 0.80]   # native scale per mode
# per mode: (inject threshold multiplier, spread bonus) -- a bloom needs room,
# so the modes with big halos inject only from the strong, coarse boundaries
MODE_FIELD = [(3.2, 0.14), (2.0, -0.12), (3.6, 0.22), (2.2, -0.06),
              (2.4, -0.12), (3.0, 0.08), (2.8, 0.00), (2.4, -0.06)]


def ground(f, p, Yg, Ug, Vg):
    """interior: video (S2 / low EMERGE) crossfading to the mode's native fill."""
    m = f["d"]["interior"]
    if p.interior_video:
        # video shows through the material, tinted toward the mode's ground
        Yv = f["Y10"] * (0.45 + 0.55 * (1 - m))
        Uv = f["U10"] * 0.65 + Ug * 0.35
        Vv = f["V10"] * 0.65 + Vg * 0.35
        return Layer(Yv, Uv, Vv)
    Y = f["Y10"] * (1 - m) + Yg * m
    U = f["U10"] * (1 - m) + Ug * m
    V = f["V10"] * (1 - m) + Vg * m
    return Layer(Y, U, V)


# -------------------------------------------------------------------- modes
def m_neon(f, p):
    d, S = f["d"], f["S"]
    tubes = ["cyan", "blue", "violet", "magenta", "pink", "red", "amber", "green"]
    Ub, Vb = pal_lerp(tubes, p.colour, 1.0)          # tube body
    Uh, Vh = pal_lerp(tubes, (p.colour + 0.14) % 1.0, 0.85)   # halo, separable
    Uc, Vc = uv_of(PAL["white"], 1.0)                # core, white hot
    L = ground(f, p, Y_MIN + 6, CH, CH)

    xx, yy = coords(f)
    cellh = hash2((xx / (15 * S)).astype(int), (yy / (17 * S)).astype(int), 7 + p.frame)
    fine_ratio = np.clip(f["mag_f"] / (f["mag_c"] + 0.03), 0, 1)
    flick = 1.0 - (0.10 + 0.45 * p.light) * fine_ratio * cellh   # bad transformer

    body = f["edge_alpha"](1.0, 0.55) * flick                # saturated gas body
    core = f["edge_alpha"](3.0 * (1 + 3.0 * d["over"]), 1.30) * flick   # filament
    glow = f["edge_alpha"](0.55, 1.60) * 0.40                # tight near-glow
    halo = np.clip(f["fu"] - 0.55 * f["inj_up"], 0, 1)       # far halo, outside
    halo = np.clip(halo ** (2.2 - 1.0 * p.material) * (1.5 + 1.5 * d["over"]), 0, 1)

    L.over(np.clip(f["fu"] ** 3.0, 0, 1) * 0.30, Y_MIN + 210, Uh, Vh, additive=True)
    L.over(halo * 0.70, Y_MIN + 330, Uh, Vh, additive=True)
    L.over(glow, Y_MIN + 330, Uh, Vh, additive=True)
    L.over(body, Y_MIN + 470, Ub, Vb, additive=True)
    L.over(core * (1 - 0.35 * d["over"]), Y_MIN + 250, Uc, Vc, additive=True)
    return L


def m_burin(f, p):
    d, S = f["d"], f["S"]
    xx, yy = coords(f)
    tone = np.clip(1.0 - f["L2"] / 255.0, 0, 1)          # tone from the FORM, not the fur
    plate = p.light * np.pi
    # strokes follow the form: tangent to the gradient, 8 families over 180 deg,
    # flat regions falling back to the plate angle (intaglio's hold)
    tang = f["sang"] * 2 * np.pi + np.pi / 2
    # the cell gradient is already smooth -- follow it almost everywhere, and
    # fall back to the plate angle only where there is genuinely no form
    strong = f["mag_c"] > 0.085
    a = np.where(strong, np.round(tang / (np.pi / 8)) * (np.pi / 8), plate)
    pitch = (30.0 - 16.0 * p.material) * S
    ph = ((xx * np.cos(a) + yy * np.sin(a)) % pitch) / pitch
    perp = ((xx * np.cos(a + np.pi / 2) + yy * np.sin(a + np.pi / 2)) % pitch) / pitch
    diag = ((xx * np.cos(a + np.pi / 4) + yy * np.sin(a + np.pi / 4)) % pitch) / pitch

    e = p.emerge
    t1, t2, t3 = 0.06 + 0.26 * (1 - e), 0.40 + 0.32 * (1 - e), 0.70 + 0.28 * (1 - e)
    wg = 0.55 + 0.75 * d["weight"]

    def stroke(phase, t):
        w = np.clip((tone - t) * wg, 0, 0.46)
        return np.clip((w - np.abs(phase - 0.5)) * (pitch * 0.9), 0, 1)

    ink = np.maximum.reduce([stroke(ph, t1), stroke(perp, t2), stroke(diag, t3)])
    ink = np.maximum(ink, f["edge_alpha"](1.6, 0.7) * (0.45 + 0.55 * d["over"]))
    Ui, Vi = pal_lerp(["black", "sepia", "red", "indigo"], p.colour, 0.55)
    Up, Vp = uv_of(PAL["paper"], 0.5)
    L = ground(f, p, 912.0, Up, Vp)
    L.over(ink, Y_MIN + 40, Ui, Vi)
    return L


def m_kirlian(f, p):
    d, S = f["d"], f["S"]
    xx, yy = coords(f)
    # filaments run outward along the normal: stripes in the tangent coordinate,
    # each stripe given its own length by a hash (branching, not glow)
    a = f["sang"] * 2 * np.pi
    q = (-xx * np.sin(a) + yy * np.cos(a))
    period = (6.0 + 14.0 * (1 - p.light)) * S
    sidx = np.floor(q / period).astype(int)
    ramp_q = np.abs((q / period) % 1.0 - 0.5) * 2.0
    branch = hash2(sidx, np.zeros_like(sidx), 3)
    reach = hash2(sidx, np.ones_like(sidx), 9)                 # per-filament length
    fil = np.clip((0.92 - ramp_q) * 3.4, 0, 1) * (0.10 + 1.35 * branch)
    # a filament dies at its own distance from the source: the field value is a
    # proxy for distance, so gate each stripe on its own threshold
    fil = fil * np.clip((f["fu"] - 0.18 * reach) * 4.0, 0, 1)

    corner = np.clip(f["mag_f"] * f["mag_c"] * 26.0, 0, 1)
    out = np.clip(f["fu"] - 0.45 * f["inj_up"], 0, 1)
    disc = np.clip(out ** (1.25 - 0.45 * p.material), 0, 1) * fil
    disc = np.clip(disc * (1.5 + 1.4 * corner + 1.6 * d["over"]), 0, 1)
    src = np.maximum(f["edge_alpha"](1.0, 0.7, src=f["mag_c"]),
                     f["edge_alpha"](2.6, 0.9) * 0.55)

    Uv_, Vv_ = pal_lerp(["violet", "magenta", "blue"], p.colour, 0.95)
    Uc_, Vc_ = pal_lerp(["cyan", "blue", "green"], p.colour, 0.85)
    Uw_, Vw_ = uv_of(PAL["white"], 1.0)
    L = ground(f, p, Y_MIN + 4, CH, CH)
    if not p.interior_video:
        L.Y = np.minimum(L.Y, Y_MIN + 12 + 120 * (1 - d["interior"]))
    L.over(disc * 1.0, Y_MIN + 300, Uv_, Vv_, additive=True)
    L.over(np.clip(disc - 0.30, 0, 1) * 1.6, Y_MIN + 420, Uc_, Vc_, additive=True)
    L.over(np.clip(disc - 0.72, 0, 1) * 3.0, Y_MIN + 560, Uw_, Vw_, additive=True)
    L.over(src * 0.85, Y_MIN + 640, Uc_ * 0.35 + CH * 0.65, Vc_ * 0.35 + CH * 0.65,
           additive=True)
    return L


METALS = ["copper", "brass", "steel", "gold"]


def m_relief(f, p):
    d = f["d"]
    aL = 2 * np.pi * p.light
    emb = (f["gx"] * np.cos(aL) + f["gy"] * np.sin(aL)) / 1020.0
    if p.single_pol:
        emb = np.maximum(emb, 0)
    hi = np.clip(emb * (5.0 + 14.0 * d["weight"]), -1, 1)
    Um, Vm = pal_lerp(METALS, p.colour, 0.55)
    base = 470.0
    Y = Y_MIN + base + 300.0 * hi - 110.0 * np.clip(f["fu"], 0, 1) * (0.4 + 0.6 * d["over"])
    spec = np.clip((hi - 0.66) * 4.4, 0, 1)
    L = ground(f, p, Y_MIN + base, Um, Vm)
    L.over(0.20 + 0.80 * d["interior"], np.clip(Y, Y_MIN, Y_CAP), Um, Vm)
    L.over(spec, Y_CAP, CH, CH)
    return L


HYPSO = [((0.08, 0.20, 0.55), 210), ((0.13, 0.42, 0.62), 330),
         ((0.30, 0.55, 0.35), 450), ((0.62, 0.66, 0.32), 560),
         ((0.76, 0.60, 0.36), 650), ((0.80, 0.72, 0.62), 760),
         ((1.00, 1.00, 1.00), 890)]


def m_contour(f, p):
    d = f["d"]
    N = int(3 + 21 * p.light)                                   # K5 = level count
    lum = f["L2"] / 255.0                                       # contours follow FORM
    lv = lum * N
    level = np.floor(lv).astype(int)
    frac = lv - level
    gpx = np.maximum(f["gpx"], 1.5e-3)                          # luma change per pixel
    dist = (np.minimum(frac, 1 - frac) / N) / gpx               # distance in pixels
    wpx = (0.35 + 1.5 * d["weight"]) * f["S"] * np.where(level % 5 == 0, 2.4, 1.0)
    line = np.clip((wpx - dist) * 1.8, 0, 1)

    Yg = np.zeros_like(lum) + 900.0
    Ug = np.zeros_like(lum) + CH
    Vg = np.zeros_like(lum) + CH
    band = np.clip((level * len(HYPSO)) // max(N, 1), 0, len(HYPSO) - 1)
    for i, (rgb, yy_) in enumerate(HYPSO):
        m = band == i
        u, v = uv_of(rgb, 0.9 * p.colour)
        Yg = np.where(m, Y_MIN + yy_ * (0.62 + 0.38 * p.colour), Yg)
        Ug = np.where(m, u, Ug)
        Vg = np.where(m, v, Vg)
    L = ground(f, p, Yg, Ug, Vg)
    Ui, Vi = uv_of(PAL["sepia"], 0.45)
    L.over(line, Y_MIN + 70, Ui, Vi)
    return L


def m_cel(f, p):
    d = f["d"]
    N = int(2 + 6 * p.light)
    lum = f["L2"] / 255.0                                # flats from the coarse level
    q = np.floor(lum * N + 0.5) / N
    Yq = Y_MIN + 876.0 * np.clip(q * (0.85 + 0.3 * p.colour), 0, 1)
    du, dv = f["cu"] - CH, f["cv"] - CH                  # cell-mean chroma = flat regions
    hu = (np.arctan2(dv, du) / (2 * np.pi)) % 1.0
    sat = np.clip(np.hypot(du, dv) / 448.0, 0, 1)
    sq = np.clip(np.floor(sat * 3 + 0.5) / 3 * (1.5 + 1.4 * p.colour), 0, 1.6)
    hq = np.floor(hu * 12 + 0.5) / 12 * 2 * np.pi
    Uq = CH + 448.0 * sq * np.cos(hq)
    Vq = CH + 448.0 * sq * np.sin(hq)

    outline = np.clip((f["mag_c"] - f["thr"] * 1.0) / max(f["thr"] * 0.45, 1e-3), 0, 1)
    outline = np.clip(outline * (0.8 + 1.4 * d["weight"]), 0, 1)
    detail = np.clip((f["mag_f"] - f["thr"] * 4.0) / max(f["thr"] * 1.5, 1e-3), 0, 1) * 0.8
    ink = np.maximum(outline, detail) * f["pol_gate"]
    L = ground(f, p, Yq, Uq, Vq)
    L.over(ink, Y_MIN + 26, CH, CH)
    return L


def m_spectral(f, p):
    d = f["d"]
    hu = (f["angq"] + p.colour) % 1.0
    U, V = hue_uv(hu, 0.98)
    hb = (f["cang"] + p.colour) % 1.0
    Ub_, Vb_ = hue_uv(hb, 0.90)
    a = f["edge_alpha"](1.0, 0.6)
    bloom = np.clip(np.clip(f["fu"] - 0.8 * f["inj_up"], 0, 1) ** 1.6, 0, 1) \
        * (0.6 + 0.8 * d["over"])
    L = ground(f, p, Y_MIN + 8, CH, CH)
    L.over(bloom * 0.6, Y_MIN + 280, Ub_, Vb_, additive=True)
    L.over(a, Y_MIN + 640, U, V, additive=True)
    return L


def m_ink(f, p):
    d, S = f["d"], f["S"]
    xx, yy = coords(f)
    fibre = hash2((xx / (2.6 * S)).astype(int), (yy / (0.9 * S)).astype(int), 11)
    grain = hash2((xx / (7 * S)).astype(int), (yy / (3 * S)).astype(int), 5)
    ragged = f["edge_alpha"](0.8 + 0.9 * fibre, 0.55)
    # bleed: the field outside the line, fibrous and biased along the stroke
    a = f["sang"] * 2 * np.pi
    q = (xx * np.cos(a) + yy * np.sin(a)) / (3.0 * S)
    feather = np.clip(0.45 + 0.75 * ((q % 1.0) * 0.6 + fibre * 0.9), 0, 1)
    out = np.clip(f["fu"] - 0.35 * f["inj_up"], 0, 1)
    bleed = np.clip(out ** (1.5 - 0.7 * p.material) * feather * (1.6 + 1.5 * d["over"]), 0, 1)

    Ui, Vi = pal_lerp(["black", "sepia", "indigo", "red"], p.colour, 0.6)
    Up, Vp = uv_of(PAL["paper"], 0.55)
    L = ground(f, p, 922.0 - 30.0 * grain, Up, Vp)
    L.over(np.clip(bleed * 0.95, 0, 1), Y_MIN + 330, Ui, Vi)
    L.over(np.clip(bleed - 0.45, 0, 1) * 1.6, Y_MIN + 170, Ui, Vi)
    L.over(ragged, Y_MIN + 45, Ui, Vi)
    return L


MODES = [("NEON", m_neon), ("BURIN", m_burin), ("KIRLIAN", m_kirlian),
         ("RELIEF", m_relief), ("CONTOUR", m_contour), ("SPECTRAL", m_spectral),
         ("INK", m_ink), ("CEL", m_cel)]


# ------------------------------------------------------------------- render
def render(img_rgb, p, frame=0):
    p.frame = frame
    f = features(img_rgb, p)
    L = MODES[p.mode][1](f, p)
    if p.spectral_overlay:
        hu = (f["angq"] + p.colour) % 1.0
        U, V = hue_uv(hu, 0.95)
        L.over(f["edge_alpha"](1.0, 0.6) * 0.75, np.minimum(L.Y + 120, Y_CAP), U, V)
    w = f["d"]["wet"]
    if w < 1.0:                       # exact dry at EMERGE 0 (turpentine lerp)
        L.Y = f["Y10"] * (1 - w) + L.Y * w
        L.U = f["U10"] * (1 - w) + L.U * w
        L.V = f["V10"] * (1 - w) + L.V * w
    if p.invert:
        L.Y = Y_MIN + Y_MAX - np.clip(L.Y, Y_MIN, Y_MAX)
        L.U = 2 * CH - L.U
        L.V = 2 * CH - L.V
    return (L.rgb() * 255).astype(np.uint8)


def load(path, scale=1.0):
    im = Image.open(path).convert("RGB")
    if scale != 1.0:
        im = im.resize((int(im.width * scale), int(im.height * scale)), Image.LANCZOS)
    return np.asarray(im).astype(np.float64) / 255.0


def sheet(tiles, cols, out, tw=640):
    ims = []
    for label, arr in tiles:
        im = Image.fromarray(arr)
        im = im.resize((tw, int(tw * im.height / im.width)), Image.LANCZOS)
        ims.append((label, im))
    w, h = ims[0][1].size
    rows = -(-len(ims) // cols)
    canvas = Image.new("RGB", (cols * w, rows * (h + 22)), (16, 16, 18))
    from PIL import ImageDraw
    dr = ImageDraw.Draw(canvas)
    for i, (label, im) in enumerate(ims):
        x, y = (i % cols) * w, (i // cols) * (h + 22)
        canvas.paste(im, (x, y + 22))
        dr.text((x + 6, y + 6), label, fill=(230, 230, 235))
    canvas.save(out)
    print("wrote", out, canvas.size)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="/home/ron/showcase_src.png")
    ap.add_argument("--out", default="/home/ron/livewire_work")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--what", default="modes")
    a = ap.parse_args()
    import os
    os.makedirs(a.out, exist_ok=True)
    img = load(a.image, a.scale)

    if a.what == "modes":
        tiles = []
        for i, (name, _) in enumerate(MODES):
            p = P(mode=i, emerge=0.62, scale=0.35, thresh=0.55, material=0.5,
                  light=0.5, colour=0.5)
            tiles.append((name, render(img, p)))
        sheet(tiles, 4, f"{a.out}/modes.png")
    elif a.what == "emerge":
        for i, (name, _) in enumerate(MODES):
            tiles = []
            for e in (0.0, 0.18, 0.38, 0.58, 0.80, 1.0):
                p = P(mode=i, emerge=e, scale=0.35, thresh=0.55)
                tiles.append((f"{name} E={int(e*100)}", render(img, p)))
            sheet(tiles, 3, f"{a.out}/emerge_{name.lower()}.png")
