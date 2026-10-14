#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""
GAMUT reference renderer (Phase 2: library + DIVE/LIFT).

Mirrors the integer geometry the VHDL uses so stills can be judged before and
beside hardware: per-frame layout -> per-line row constants -> per-pixel
rounded-rect coverage from (2*cx-1)^2 + (2*cy-1)^2 against (2r)^2, shift-based
AA ramps, square-falloff shadows, dock ring, radial tinted backdrop, LIFT as
a lerp of the hero rectangle toward the screen while the list recedes.

Usage: python3 gamut_ref.py            -> contact/ref/*.png (+ lift.gif)
"""
import json, math, os, sys
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "contact", "ref")
MAN = json.load(open(os.path.join(HERE, "contact", "manifest.json")))
SHELVES = MAN["shelves"]

# ----------------------------------------------------------------------------- frame-level layout (vblank math)

DENSITY = [  # (card_h, card_w, gap) as k/256 of H / W / H ; cols
    dict(h=64, w=128, gap=10, cols=1),   # LARGE   H/4,  W/2
    dict(h=37, w=115, gap=6,  cols=1),   # MEDIUM  H/7,  0.45W
    dict(h=32, w=85,  gap=6,  cols=2),   # GRID 2  H/8,  W/3
    dict(h=26, w=64,  gap=5,  cols=3),   # GRID 3  H/10, W/4
]

def smoothstep10(p):          # p 0..1023 -> s 0..1023 (3p^2 - 2p^3)
    p2 = (p * p) >> 10
    p3 = (p2 * p) >> 10
    return max(0, min(1023, 3 * p2 - 2 * p3))

def ilog2(v):
    return max(0, v.bit_length() - 1)

def layout(W, H, density, k6, shelf, cursor, scroll_pos, dive, theme, shift_x=0):
    """scroll_pos in rows*256 (eased), dive 0..1023 (eased), k6 0..1023."""
    d = DENSITY[density]
    cols = d["cols"]
    card_h = (H * d["h"]) >> 8
    card_w = (W * d["w"]) >> 8
    gap = (H * d["gap"]) >> 8
    pitch = card_h + gap
    focal = (H >> 1) - (card_h >> 1)              # top of the focal card
    span = cols * card_w + (cols - 1) * gap       # grid width
    x_left = ((W - span) >> 1) + shift_x
    col_x0 = [x_left + c * (card_w + gap) for c in range(cols)]
    r = min(255, (k6 * card_h) >> 11)             # 0 .. card_h/2 (pill)
    n_pal = len(SHELVES[shelf]["palettes"])
    cur_row = cursor // cols
    cur_col = cursor - cur_row * cols
    s = smoothstep10(dive)
    lift = card_h >> 5                            # hero scale-up (3%)
    # hero rectangle in the list, then lerped toward the screen
    hy0 = focal + (((cur_row << 8) - scroll_pos) * pitch >> 8) - lift
    hx0 = col_x0[cur_col] - lift
    hx1 = col_x0[cur_col] + card_w + lift
    hy1 = hy0 + card_h + 2 * lift
    def lerp(a, b):
        return a + (((b - a) * s) >> 10)
    hero = dict(x0=lerp(hx0, 0), x1=lerp(hx1, W), y0=lerp(hy0, 0), y1=lerp(hy1, H))
    hero["r"] = r - ((r * s) >> 10)
    S = (card_h >> 2) + ((card_h * s) >> 11)
    hero["sh_spread"] = max(0, ilog2(4 * S * (2 * hero["r"] + S)) - 6)   # log2(Rs^2-R^2) - 6 -> 64 levels
    hero["sh_oy"] = (card_h >> 5) + (card_h >> 4)
    hero["shadow"] = 8 if theme == 0 else 4                  # strength /16 (VHDL: shift 1 / 2)
    gap_ring = max(2, H // 180)
    thick = max(2, H // 360)
    hero["ring_r0"] = hero["r"] + gap_ring
    hero["ring_r1"] = hero["r"] + gap_ring + thick
    hero["ring_alpha"] = max(0, 16 - (s >> 4))
    inset_lift = (card_h * s) >> 13                          # list recedes up to 12%
    fade = s >> 6                                            # list dims to 25%/16
    return dict(W=W, H=H, cols=cols, card_h=card_h, card_w=card_w, gap=gap, pitch=pitch, focal=focal,
                col_x0=col_x0, r=r, n_pal=n_pal, cur_row=cur_row, cur_col=cur_col, s=s, hero=hero,
                inset_lift=inset_lift, fade=fade, shelf=shelf, cursor=cursor, scroll_pos=scroll_pos, theme=theme,
                list_shadow=8 if theme == 0 else 4, list_sh_spread=max(0, ilog2(4 * (card_h >> 3) * (2 * r + (card_h >> 3))) - 6),
                list_sh_oy=card_h >> 5)

# ----------------------------------------------------------------------------- per-pixel evaluator (one rounded rect)

def sat_shift(v, sh, lo=-8, hi=8):
    t = np.right_shift(v, sh)
    return np.clip(t, lo, hi)

def rrect(X, Y, x0, x1, y0, y1, r, x0f=0, x1f=0, y0f=0, y1f=0):
    """coverage 0..16 of a rounded rect [x0,x1]x[y0,y1] with radius r (ints) and 1/16 edge fractions."""
    ex = np.maximum(np.maximum(x0 + r - X, X - (x1 - 1 - r)), 0)
    ey = np.maximum(np.maximum(y0 + r - Y, Y - (y1 - 1 - r)), 0)
    cx = np.minimum(ex, 255); cy = np.minimum(ey, 255)
    EX = np.maximum(2 * cx - 1, 0); EY = np.maximum(2 * cy - 1, 0)
    D2 = EX * EX + EY * EY
    R2 = 4 * r * r
    if r >= 2:
        cov_c = 8 + sat_shift(R2 - D2, ilog2(r) - 1)
    else:
        cov_c = np.full_like(D2, 16)
    cov_x = np.where(X == x0, 16 - x0f, np.where(X == x1, x1f, 16))
    cov_y = np.where(Y == y0, 16 - y0f, np.where(Y == y1, y1f, 16))
    inside = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    cov = np.minimum(np.minimum(cov_c, cov_x), cov_y)
    return np.where(inside, np.clip(cov, 0, 16), 0), D2

def ring(D2, r0, r1):
    a = 8 + sat_shift(D2 - 4 * r0 * r0, max(0, ilog2(r0) - 1))
    b = 8 + sat_shift(4 * r1 * r1 - D2, max(0, ilog2(r1) - 1))
    return np.clip(np.minimum(a, b), 0, 16)

def shadow(X, Y, x0, x1, y0, y1, r, oy, spread_sh):
    _, D2 = rrect(X, Y, x0, x1, y0 + oy, y1 + oy, r)
    R2 = 4 * r * r
    return np.clip(np.right_shift(R2 + (1 << (spread_sh + 6)) - D2, spread_sh), 0, 64)

def bars(X, x0, x1, cols_rgb):
    """bar index by DDA: acc += n per pixel from x0, wrap at w."""
    n = len(cols_rgb); w = x1 - x0
    idx = np.clip(((X - x0) * n) // max(w, 1), 0, n - 1)
    pal = np.array(cols_rgb, dtype=np.int32)
    return pal[idx]

# ----------------------------------------------------------------------------- frame render

def hex_rgb(h):
    return [int(h[i:i + 2], 16) for i in (1, 3, 5)]

def render(L, tint_rgb=None):
    W, H = L["W"], L["H"]
    Y, X = np.mgrid[0:H, 0:W].astype(np.int32)
    dark = L["theme"] == 0
    # backdrop: radial lift + tint
    cx, cy = W >> 1, H >> 1
    rad = ((X - cx) ** 2 + (Y - cy) ** 2) >> (12 if W >= 1536 else 11 if W >= 1024 else 10)
    liftv = np.clip(160 - rad, 0, 160)                       # 0..160 in 10-bit luma codes
    h = ((X * 0x9E37 + Y * 0x7C15) >> 5) & 3                 # 2-bit dither of the 10-bit value
    lift8 = (liftv * 4 + h) / 16.0                           # -> 8-bit with dither
    if dark:
        base = np.array([30, 30, 34])
        bg = base[None, None, :] + lift8[..., None] * np.array([0.45, 0.45, 0.5])[None, None, :]
        if tint_rgb is not None:
            t = (np.array(tint_rgb) - 128) * 0.10
            bg = bg + lift8[..., None] / 40.0 * t[None, None, :]
    else:
        base = np.array([246, 243, 237])
        bg = base[None, None, :] - (40 - lift8)[..., None] * np.array([0.5, 0.5, 0.4])[None, None, :]
    bg = np.clip(bg, 0, 255)

    cols, card_h, card_w, pitch = L["cols"], L["card_h"], L["card_w"], L["pitch"]
    pals = SHELVES[L["shelf"]]["palettes"]
    hero = L["hero"]
    sh_total = np.zeros((H, W), np.int32)
    layer_list = np.zeros((H, W, 3)); cov_list = np.zeros((H, W), np.int32)
    # list rows
    r_first = max(0, (L["scroll_pos"] >> 8) - 6)
    for row in range(r_first, r_first + 14):
        top = L["focal"] + (((row << 8) - L["scroll_pos"]) * pitch >> 8)
        if top > H or top + card_h < 0:
            continue
        dist = min(abs((row << 8) - L["scroll_pos"]), 512)
        inset = ((dist * (card_h >> 4)) >> 9) + L["inset_lift"]
        dim = max(0, 64 - (dist >> 4) - 2 * L["fade"])          # 0..64, scales the list coverage
        y0, y1 = top + inset, top + card_h - inset
        for c in range(cols):
            i = row * cols + c
            if i >= L["n_pal"] or (row == L["cur_row"] and c == L["cur_col"]):
                continue
            x0, x1 = L["col_x0"][c] + inset, L["col_x0"][c] + card_w - inset
            ys, ye = max(0, y0 - 64), min(H, y1 + 96); xs, xe = max(0, x0 - 64), min(W, x1 + 64)
            Xs, Ys = X[ys:ye, xs:xe], Y[ys:ye, xs:xe]
            cov, _ = rrect(Xs, Ys, x0, x1, y0, y1, L["r"])
            sh = shadow(Xs, Ys, x0, x1, y0, y1, L["r"], L["list_sh_oy"], L["list_sh_spread"])
            sh_total[ys:ye, xs:xe] += (sh * L["list_shadow"]) >> 4      # 0..64 * k/16
            col = bars(Xs, x0, x1, [hex_rgb(k["hex"]) for k in pals[i]["colours"]])
            cov = (cov * dim) >> 6
            layer_list[ys:ye, xs:xe] = np.where(cov[..., None] > 0, col, layer_list[ys:ye, xs:xe])
            cov_list[ys:ye, xs:xe] = np.maximum(cov_list[ys:ye, xs:xe], cov)
    # hero
    hx0, hx1, hy0, hy1, hr = hero["x0"], hero["x1"], hero["y0"], hero["y1"], hero["r"]
    hcov, D2 = rrect(X, Y, hx0, hx1, hy0, hy1, hr)
    hsh = shadow(X, Y, hx0, hx1, hy0, hy1, hr, hero["sh_oy"], hero["sh_spread"])
    hsh = (hsh * hero["shadow"]) >> 4                        # 0..64
    rcov = ring(D2, hero["ring_r0"], hero["ring_r1"]) * hero["ring_alpha"] >> 4
    rcov = np.where(hcov > 0, 0, rcov)
    hcol = bars(X, hx0, hx1, [hex_rgb(k["hex"]) for k in pals[L["cursor"]]["colours"]])
    # composite
    sh_all = np.clip(sh_total + hsh, 0, 64)
    bg2 = bg * (1 - sh_all[..., None] / 64.0)
    # one blend: (over, cov) = hero | ring | list ; under = list where fully covered else backdrop
    ring_rgb = np.array([235, 235, 240] if dark else [40, 40, 48])
    top = hcov > 0
    over = np.where(top[..., None], hcol, np.where((rcov > 0)[..., None], ring_rgb[None, None, :], layer_list))
    cov = np.where(top, hcov, np.where(rcov > 0, rcov, cov_list))
    listfull = ((hcov > 0) | (rcov > 0)) & (cov_list == 16)
    under = np.where(listfull[..., None], layer_list, bg2)
    out = under + (over - under) * cov[..., None] / 16.0
    top_cov = np.maximum(hcov, rcov)
    # scroll track + thumb (UI layer, drawn only when not covered)
    tx1 = W - (W >> 6); tx0 = tx1 - max(4, H // 180); ty0, ty1 = H // 10, H - H // 10
    tcov, _ = rrect(X, Y, tx0, tx1, ty0, ty1, (tx1 - tx0) // 2)
    n_rows = (L["n_pal"] + cols - 1) // cols
    th = max(24, (ty1 - ty0) * 3 // max(n_rows, 3))
    tpos = ty0 + ((ty1 - ty0 - th) * L["scroll_pos"]) // max(1, (n_rows - 1) << 8)
    thcov, _ = rrect(X, Y, tx0 - 2, tx1 + 2, tpos, tpos + th, (tx1 - tx0) // 2 + 2)
    ui_col = np.array([90, 90, 100] if dark else [200, 196, 188]); th_col = np.array([200, 200, 210] if dark else [90, 90, 100])
    ui = np.where(thcov[..., None] > 0, th_col, ui_col)
    uicov = np.maximum(tcov, thcov) * (16 - top_cov) >> 4
    out = out + (ui - out) * uicov[..., None] / 16.0
    # pagination dots
    dp = max(10, H // 60); drad = max(2, H // 360); n_sh = len(SHELVES)
    dx0 = (W >> 1) - (n_sh * dp) // 2; dy = H - H // 24
    for k in range(n_sh):
        if k == L["shelf"]:
            c, _ = rrect(X, Y, dx0 + k * dp - drad, dx0 + k * dp + dp - drad, dy - drad, dy + drad, drad)
            colr = th_col
        else:
            c, _ = rrect(X, Y, dx0 + k * dp, dx0 + k * dp + 2 * drad, dy - drad, dy + drad, drad)
            colr = ui_col
        c = c * (16 - top_cov) >> 4
        out = out + (colr - out) * c[..., None] / 16.0
    return np.clip(out, 0, 255).astype(np.uint8)

# ----------------------------------------------------------------------------- spring (per frame)

def spring(pos, vel, target, ks, kc):
    vel += ((target - pos) >> ks) - (vel >> kc)
    pos += vel
    return pos, vel

def dominant(shelf, cursor):
    p = SHELVES[shelf]["palettes"][cursor]
    for c, r in zip(p["colours"], p["roles"]):
        if r == "D":
            return hex_rgb(c["hex"])
    return hex_rgb(p["colours"][0]["hex"])

def main():
    os.makedirs(OUT, exist_ok=True)
    W, H = 1920, 1080
    k6 = 340
    frames = []
    def save(name, L, tint=None, caption=""):
        img = Image.fromarray(render(L, tint))
        if caption:
            ImageDraw.Draw(img).text((24, 20), caption, fill=(255, 255, 255) if L["theme"] == 0 else (0, 0, 0))
        img.save(os.path.join(OUT, name + ".png"))
        frames.append((name, img))
        print("wrote", name)
    # library at the four densities, dark, NATURE shelf cursor 5
    shelf, cur = 9, 5
    for d in range(4):
        cursor = cur if d < 2 else 7
        L = layout(W, H, d, k6, shelf, cursor, (cursor // DENSITY[d]["cols"]) << 8, 0, 0)
        save("lib_density%d_dark" % d, L, dominant(shelf, cursor), "LIBRARY density %d dark" % d)
    L = layout(W, H, 1, k6, 2, 4, 4 << 8, 0, 1)
    save("lib_density1_light", L, None, "LIBRARY density 1 light (MOVEMENTS)")
    # shape sweep
    for k in (0, 1023):
        L = layout(W, H, 1, k, 12, 3, 3 << 8, 0, 0)
        save("lib_shape%d" % k, L, dominant(12, 3), "SHAPE K6=%d" % k)
    # scroll mid-glide: cursor jumped 5 -> 8, spring at frame 6
    pos, vel = 5 << 8, 0
    for f in range(6):
        pos, vel = spring(pos, vel, 8 << 8, 4, 2)
    L = layout(W, H, 1, k6, shelf, 8, pos, 0, 0)
    save("scroll_midglide", L, dominant(shelf, 8), "SCROLL glide frame 6 (5 -> 8), pos=%.2f rows" % (pos / 256))
    # LIFT at progress values
    lift_frames = []
    for p in (0, 150, 300, 450, 600, 750, 900, 1023):
        L = layout(W, H, 1, k6, shelf, cur, cur << 8, p, 0)
        save("lift_p%04d" % p, L, dominant(shelf, cur), "LIFT P12=%d/1023  s=%d" % (p, L["s"]))
    # animated lift with the dive spring (underdamped), 56 frames at half size (--gif)
    if "--gif" not in sys.argv:
        frames_sheet(frames); return
    pos, vel = 0, 0
    gif = []
    for f in range(56):
        pos, vel = spring(pos, vel, 1023 if f < 32 else 0, 4, 3)
        pos = max(0, min(1023, pos))
        L = layout(W, H, 1, k6, shelf, cur, cur << 8, pos, 0)
        gif.append(Image.fromarray(render(L, dominant(shelf, cur))).resize((960, 540)))
    gif[0].save(os.path.join(OUT, "lift.gif"), save_all=True, append_images=gif[1:], duration=33, loop=0)
    print("wrote lift.gif")
    frames_sheet(frames)

def frames_sheet(frames):
    thumbs = [im.resize((640, 360)) for _, im in frames]
    cols = 4; rows = (len(thumbs) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * 650 + 10, rows * 370 + 10), (12, 12, 14))
    for i, t in enumerate(thumbs):
        sheet.paste(t, (10 + (i % cols) * 650, 10 + (i // cols) * 370))
    sheet.save(os.path.join(OUT, "phase2_sheet.png"))
    print("wrote phase2_sheet.png")

if __name__ == "__main__":
    main()
