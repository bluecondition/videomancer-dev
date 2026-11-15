#!/usr/bin/env python3
"""CLAW reference renderer: same integer rules the VHDL uses, rendered per line.

  python3 claw_ref.py [--mode 1080p|720p|sd] [--theme N] [--x K1] [--cable P12]
                      [--open 0|1] [--depth 0..2] [--out file.png]

Knob values are 0..1023 like the hardware registers.
"""
import argparse
import os
import sys
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import claw_layout as L  # noqa: E402

BAYER = np.array([[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]])

MODES = {
    "1080p": (1920, 1080, 2, 256),
    "720p": (1280, 720, 3, 384),
    "sd": (720, 486, 4, 1138 // 2),   # 480 lines shown line-doubled: frame step 1080/486
}


def claw_state(vw, k1, p12, opened, k2):
    span = vw - L.CLAW_X_MARGIN - L.CLAW_X_MIN
    X = L.CLAW_X_MIN + ((k1 * span) >> 10)
    Yh = L.CLAW_Y_MIN + ((p12 * L.CLAW_Y_RANGE) >> 10)
    o = L.OPEN_MAX if opened else L.OPEN_MIN
    depth = 0 if k2 < 341 else (1 if k2 < 682 else 2)
    return X, Yh, o, depth


def anchor_x(a, off, vw, X, J=0):
    return {"L": 0, "R": vw, "C": vw // 2, "X": X, "J": J}[a] + off


def anchor_y(a, off, Yh, J=0):
    return {"A": 0, "H": Yh, "J": J}[a] + off


def joy_geom(vw, jo, pose):
    ty, r, rc = L.JOY_POSE[pose]
    jx = vw // 2 + L.JOY_X + (((L.JOY_BASE_Y - ty) * jo) >> 5)
    return jx, ty, r, rc


def line_slots(vy, vw, X, Yh, o, inset, depth=0, jo=0, pose="up"):
    """Per-line slot registers: dict slot -> (elem, Lo, Ro, Li, Ri, Hx, hl, sh)."""
    slots = {}
    jx, jy, jr, jrc = joy_geom(vw, jo, pose)
    for ei, (name, slot, xa, xb, ya, yb, rc, kind, lean) in enumerate(L.scaled_elements(depth) + [L.HOLE]):
        x0 = anchor_x(xa[0], xa[1], vw, X, jx)
        x1 = anchor_x(xb[0], xb[1], vw, X, jx)
        y0 = anchor_y(ya[0], ya[1], Yh, jy)
        y1 = anchor_y(yb[0], yb[1], Yh, jy)
        if kind == L.K_JBALL:
            x0, x1, y0, y1, rc = jx - jr, jx + jr, jy - jr, jy + jr, jrc
        if not (y0 <= vy < y1):
            continue
        ktop = vy - y0
        kbot = y1 - 1 - vy
        k = min(ktop, kbot, 31)
        io, ii = inset[rc * 32 + k]
        if lean == 2:
            d = (kbot * jo) >> L.LEAN_SHIFT
            x0 += d
            x1 += d
        elif lean:
            d = (ktop * o) >> L.LEAN_SHIFT
            x0 += lean * d
            x1 += lean * d
        r, t = L.RCLASS[rc]
        Lo, Ro = x0 + io, x1 - io
        if ii == 63:
            Li = Ri = 0
            vi = False
        else:
            Li, Ri = x0 + ii, x1 - ii
            vi = True
        hl = (ktop - t) < L.HL_ROWS
        sh = (kbot - t) < L.SH_ROWS
        slots[slot] = (kind, ei, Lo, Ro, vi, Li, Ri, Li + L.HL_COLS, hl, sh)
        if kind == L.K_CHUTE:
            slots["flap"] = ((ktop - t) < L.FLAP_ROWS, (ktop - t) >= L.FLAP_ROWS - L.FLAP_EDGE)
    # cable: direct logic in slot 8
    hw, bot = L.CABLE_HW[depth], L.CABLE_BOT[depth]
    if L.CABLE_TOP <= vy < Yh + bot:
        lo = X - hw
        slots[8] = (L.K_CABLE, -1, lo, lo + 2 * hw, True, lo + L.CABLE_T, lo + 2 * hw - L.CABLE_T,
                    lo + L.CABLE_T + 3, False, False)
    return slots


def build_lb(vy, vw, slots, nib2m, sprites):
    """Line buffer for one line: pal address per 2-unit entry (0 = empty)."""
    n_ent = vw // 2 + 4
    lb = np.zeros(n_ent, np.int32)
    for s in slots:
        dy = vy - s["y"]
        if not (0 <= dy < L.SPRITE_PX):
            continue
        v = dy >> 2
        x0 = s["x"]
        if s["tilt"] == 1:
            x0 += (dy - 96) >> 3
        elif s["tilt"] == 2:
            x0 -= (dy - 96) >> 3
        elif s["tilt"] == 3:
            v = 47 - v
        e0 = x0 >> 1
        spr = sprites[s["type"]]
        if s.get("tr", 0):
            spr = spr.T
        row = spr[v]
        cols = np.where((spr != 0).any(0))[0]
        f, l = int(cols[0]), int(cols[-1])
        if s["mirror"]:
            f, l = 47 - l, 47 - f
        for m in range(f, l + 1):
            c = row[47 - m] if s["mirror"] else row[m]
            if c == 0:
                continue
            pa = nib2m[s["type"]][c] + 1000 * s["depth"]   # depth rides with the colour
            for j in (0, 1):
                e = e0 + 2 * m + j
                if 0 <= e < n_ent:
                    lb[e] = pa
    return lb


def render(mode="1080p", theme=0, k1=420, p12=300, opened=False, k2=0, video=None,
           jo=0, pose="up"):
    W, H, xs, ys = MODES[mode]
    vw = 1440 if mode == "sd" else 1920
    A = np.load(os.path.join(HERE, "claw_assets.npz"))
    sprites = A["sprites"]
    font = A["font"]
    nib2m = A["nib2m"]
    # sprites are stored as master indices; convert to nibbles through nib2m
    spr_nib = []
    for t in range(len(sprites)):
        m2n = {int(m): n for n, m in enumerate(nib2m[t]) if (m or n == 0)}
        spr_nib.append(np.vectorize(lambda v: m2n[int(v)])(sprites[t]))
    pile = L.pile_slots(vw) + [L.door_slot(vw), L.hole_slot(vw)]
    pal = L.palette_yuv([("m", tuple(int(v) for v in c)) for c in A["master"]])
    inset = L.inset_table()
    X, Yh, o, cdepth = claw_state(vw, k1, p12, opened, k2)
    th = L.TH_BASE_ADDR + 16 * theme

    img = np.zeros((H, W, 3), np.int32)
    vxs = (np.arange(W) * xs) >> 1
    for j in range(H):
        vy = (j * ys) >> 8
        slots = line_slots(vy, vw, X, Yh, o, inset, cdepth, jo, pose)
        lb = build_lb(vy, vw, pile, nib2m, spr_nib)
        for i in range(W):
            vx = int(vxs[i])
            pa, lift, wall = pixel(vx, vy, i, j, vw, slots, lb, font, th, X, Yh, cdepth)
            Y, U, V = L.haze_yuv(pal[pa % 1000], pa // 1000)
            if wall and video is not None:
                vy_, vu_, vv_ = video[j % video.shape[0], i % video.shape[1]]
                Y, U, V = 64 + ((vy_ - 64) >> 1), 512 + ((vu_ - 512) >> 1), 512 + ((vv_ - 512) >> 1)
            Y = min(1000, Y + 16 * lift if not (wall and video is not None) else Y + 8 * lift)
            img[j, i] = (Y, U, V)
    return img


def in_slot(sl, vx):
    kind, ei, Lo, Ro, vi, Li, Ri, Hx, hl, sh = sl
    o = Lo <= vx < Ro
    i = vi and (Li <= vx < Ri)
    h = i and (vx < Hx or hl)
    s = i and sh and not h
    return o, i, h, s


def ramp(o, i, h, s, sh, base, hl):
    if not i:
        return L.M["outline"]
    if h:
        return hl
    if s:
        return sh
    return base


def text_pixel(vx, vy, vw, font, th):
    for sid, txt, sc, xa, y0 in L.TEXTS:
        x0 = anchor_x(xa[0], xa[1], vw, 0)
        if vw == 1440 and sid in L.TEXT_X_SD:
            x0 = L.TEXT_X_SD[sid]
        size = 16 << sc
        if not (y0 <= vy < y0 + size):
            continue
        dx = vx - x0
        if not (0 <= dx < 8 * size):
            continue
        ci = dx // size
        if ci >= len(txt):
            continue
        g = L.GLYPH_CODES[txt[ci]]
        col = (dx >> sc) & 15
        row = ((vy - y0) >> sc) & 15
        v = int(font[g][row][col])
        if v == 0:
            return None
        if v == 1:
            return L.M["outline"]
        if sid == 0:
            return th + (L.TH_TITLE if v == 2 else L.TH_TITLE_HL)
        if sid == 5:
            return L.M["white"]
        return L.M["bulb_on"] if v == 2 else L.M["white"]
    return None


def bulb_pixel(vx, vy, vw, phase=0):
    for yc in L.BULB_Y:
        dy = vy - yc
        if abs(dy) > 7:
            continue
        if not (L.BULB_X_MIN <= vx <= vw - L.BULB_X_MIN):
            return None
        dx = (vx & 31) - 16
        d2 = dx * dx + dy * dy
        if d2 >= L.BULB_R2_RING:
            return None
        if d2 >= L.BULB_R2_CORE:
            return L.M["chrome_sh"]
        if -4 <= dx <= -2 and -4 <= dy <= -2:
            return L.M["white"]
        lit = (((vx >> 5) + phase) >> 1) & 1 == 0
        return L.M["bulb_on"] if lit else L.M["bulb_off"]
    return None


def pixel(vx, vy, px, py, vw, slots, lb, font, th, X, Yh, cdepth):
    flap = slots.get("flap", (False, False))
    g = {k: in_slot(v, vx) for k, v in slots.items() if k != "flap"}
    kind = {k: v[0] for k, v in slots.items() if k != "flap"}
    F = (False, False, False, False)
    s0, s1, s2, s3, s4 = (g.get(k, F) for k in range(5))
    lbv = int(lb[vx >> 1]) if (vx >> 1) < len(lb) else 0
    chrome = (L.M["chrome_sh"], L.M["chrome"], L.M["chrome_hl"])
    lift = 0
    # the joystick stands on the prize-door panel, in front of everything
    if s2[0] and kind.get(2) == L.K_JBALL:
        return ramp(*s2, L.M["red_sh"], L.M["red"], L.M["red_hl"]), 0, False
    g5 = g.get(5, F)
    if g5[0] and kind.get(5) == L.K_JSTICK:
        return ramp(*g5, *chrome), 0, False
    if s1[1] and kind[1] == L.K_WINDOW:
        sv = vx + vy - vw // 2
        for lo, hi, st in L.STREAKS:
            if lo <= sv < hi:
                lift = st
        if s3[1] and kind[3] == L.K_CHUTE and flap[0]:
            if flap[1]:
                return L.M["outline"], lift, False
            return (L.M["chrome_hl"] if slots[3][8] else L.M["chrome"]), lift, False
        if s3[0] and kind[3] == L.K_CHUTE and not s3[1]:
            return L.M["outline"], lift, False
        if s3[1] and kind[3] == L.K_CHUTE:
            lift += 5 if s3[2] else 2
        lbd = lbv // 1000
        claw_ok = lbv == 0 or lbd >= cdepth
        if claw_ok:
            for k in (7, 2, 5, 6, 8):
                gk = g.get(k, F)
                if gk[0] and (k != 2 or kind[2] == L.K_CHROME and slots[2][1] == 7):
                    return ramp(*gk, *chrome), lift, False
        if lbv:
            return lbv, lift, False
        if s3[0] and kind[3] == L.K_ACCENT:
            return ramp(*s3, th + L.TH_ACC_SH, th + L.TH_ACC, th + L.TH_ACC_HL), lift, False
        if s2[0] and kind[2] == L.K_CHROME:
            return ramp(*s2, *chrome), lift, False
        b = BAYER[py & 3][px & 3]
        if vy >= L.PILE_DARK_Y:
            f = (vy - L.PILE_DARK_Y) >> 2
            if f >= 16 or b < f:
                return th + L.TH_DOOR, lift, False
            return th + L.TH_WALL3, lift, True
        v16 = max(0, vy - 164) >> 4
        shade = min(2, v16 >> 4)
        frac = v16 & 15 if v16 < 48 else 15
        if b < frac:
            shade += 1
        return th + L.TH_WALL0 + shade, lift, True
    if s1[1] and kind[1] == L.K_PANEL:
        t = text_pixel(vx, vy, vw, font, th)
        return (t if t is not None else th + L.TH_DEEP), 0, False
    if s1[1] and kind[1] == L.K_DOOR:
        return (lbv if lbv else th + L.TH_DOOR), 0, False
    if s1[0]:
        return L.M["outline"], 0, False
    if s0[1]:
        if kind[0] == L.K_FRAME:
            b = bulb_pixel(vx, vy, vw)
            if b is not None:
                return b, 0, False
            return ramp(*s0, th + L.TH_BASE_SH, th + L.TH_BASE, th + L.TH_BASE_HL), 0, False
        if kind[0] == L.K_BEZEL:
            return ramp(*s0, *chrome), 0, False
        if s3[0] and kind.get(3) == L.K_DOOR:            # the prize hole
            if not s3[1]:
                return L.M["outline"], 0, False
            return (lbv if lbv else th + L.TH_DOOR), 0, False
        if s2[0] and kind[2] == L.K_DECAL:
            if s2[1]:
                t = text_pixel(vx, vy, vw, font, th)
                if t is not None:
                    return t, 0, False
            return ramp(*s2, th + L.TH_ACC_SH, th + L.TH_ACC, th + L.TH_ACC_HL), 0, False
        return ramp(*s0, th + L.TH_BASE_SH, th + L.TH_BASE, th + L.TH_BASE_HL), 0, False
    if s0[0]:
        return L.M["outline"], 0, False
    return th + L.TH_BG, 0, False


def to_rgb(img):
    Y = (img[..., 0] - 64) / 876.0
    U = (img[..., 1] - 512) / 896.0
    V = (img[..., 2] - 512) / 896.0
    r = Y + 1.402 * V
    g = Y - 0.344136 * U - 0.714136 * V
    b = Y + 1.772 * U
    return (np.clip(np.stack([r, g, b], -1), 0, 1) * 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="1080p")
    ap.add_argument("--theme", type=int, default=0)
    ap.add_argument("--x", type=int, default=420)
    ap.add_argument("--cable", type=int, default=300)
    ap.add_argument("--open", type=int, default=0)
    ap.add_argument("--depth", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--jo", type=int, default=0)
    ap.add_argument("--pose", default="up")
    a = ap.parse_args()
    img = render(a.mode, a.theme, a.x, a.cable, bool(a.open), a.depth * 400, jo=a.jo, pose=a.pose)
    out = a.out or os.path.join(HERE, "..", "preview", "ref_%s_t%d.png" % (a.mode, a.theme))
    Image.fromarray(to_rgb(img)).save(out)
    print(out)


if __name__ == "__main__":
    main()
