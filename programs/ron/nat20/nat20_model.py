#!/usr/bin/env python3
"""NAT20 look model (float, numpy) -- the design target for the RTL.

Renders one die the way the proposed hardware would: orthographic camera,
per-face affine face coordinates, ownership by inside tests, edge-distance
bevels (rounded edges as a normal tilt), engraved numerals from 4-bit
bilinear distance fields (lit/shadowed cut walls, matte ink floor, lip
shadow), a finite-distance key light so highlights glide across flat faces,
and a gradient + vignette + contact-shadow backdrop.

    python3 nat20_model.py                 # hero sheet of all five dice
    python3 nat20_model.py --die d20 --yaw 30 --pitch -20 --light 0.33 \
        --color crimson --ink white --finish gloss --out x.png

Not bit-accurate: this fixes the LOOK and the per-pixel quantities the
hardware has to produce.  Fixed-point modelling comes with the RTL.
"""
import argparse
import json
import math
import os

import numpy as np
from PIL import Image

import glyph_sdf as G

HERE = os.path.dirname(os.path.abspath(__file__))
G.FONTS["lato"] = os.environ.get("NAT20_FONT", "/mnt/c/Windows/Fonts/Lato-Bold.ttf")

# ---------------------------------------------------------------- palettes
def srgb(c):
    c = np.asarray(c, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def to_srgb8(lin):
    lin = np.clip(lin, 0, 1)
    s = np.where(lin <= 0.0031308, lin * 12.92, 1.055 * lin ** (1 / 2.4) - 0.055)
    return (np.clip(s, 0, 1) * 255 + 0.5).astype(np.uint8)


# body, secondary (marble veins / pearl sheen / glitter), metal tint
COLORS = {
    "crimson":   ((150, 16, 30), (235, 205, 190), (215, 160, 70)),
    "midnight":  ((22, 32, 92), (150, 175, 230), (150, 160, 185)),
    "obsidian":  ((16, 16, 20), (205, 205, 215), (90, 92, 100)),
    "bone":      ((228, 218, 192), (150, 118, 84), (225, 200, 150)),
    "emerald":   ((8, 118, 62), (190, 235, 170), (175, 190, 160)),
    "amethyst":  ((92, 38, 140), (225, 175, 245), (190, 170, 210)),
    "frost":     ((172, 214, 236), (255, 255, 255), (205, 220, 235)),
    "ember":     ((232, 92, 18), (255, 222, 120), (225, 150, 80)),
    "lagoon":    ((0, 118, 124), (150, 240, 215), (160, 200, 200)),
    "bubblegum": ((240, 118, 170), (255, 240, 248), (235, 185, 190)),
    "smoke":     ((68, 70, 76), (190, 190, 198), (120, 122, 130)),
    "sunflower": ((240, 182, 18), (200, 110, 10), (235, 195, 90)),
}
INKS = {
    "white": (238, 236, 228), "gold": (205, 160, 52), "silver": (188, 190, 196),
    "black": (14, 14, 16), "red": (186, 18, 24), "tonal": None, "unpainted": None,
}
FINISHES = ["gloss", "matte", "pearl", "marble", "gem", "glitter", "wood", "metal"]

# ---------------------------------------------------------------- dice data
with open(os.path.join(HERE, "dice.json")) as fh:
    DICE = json.load(fh)


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot(yaw, pitch, roll):
    """Die -> view rotation: roll about z, then pitch about x, then yaw about y."""
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Ry @ Rx @ Rz


# numeral layout per die, in die units (circumradius 1).  ink = cap height.
LAYOUT = {
    "d4":  {"ink": 0.22, "corner_pos": 0.50, "condense": 1.0},   # corner_pos: fraction centroid->vertex
    "d6":  {"ink": 0.50, "offset": 0.0, "condense": 1.0},
    "d8":  {"ink": 0.42, "offset": -0.03, "condense": 1.0},
    "d10": {"ink": 0.30, "axis_pos": 0.62, "condense": 0.92},    # axis_pos: fraction apex->foot
    "d20": {"ink": 0.26, "offset": -0.015, "condense": 0.80, "gap": 0.07},
}
UNDERLINE = {"d10", "d20"}


class Die:
    """Face frames + numeral instances for one die."""

    def __init__(self, name):
        d = DICE[name]
        self.name = name
        self.V = np.array(d["verts"], float)
        self.faces = []
        lay = LAYOUT[name]
        Fv = [f["verts"] for f in d["faces"]]
        # adjacency: face across each edge
        emap = {}
        for fi, f in enumerate(Fv):
            for k in range(len(f)):
                emap.setdefault(tuple(sorted((f[k], f[(k + 1) % len(f)]))), []).append(fi)
        for fi, fd in enumerate(d["faces"]):
            f = fd["verts"]
            P = self.V[f]
            n = unit(np.cross(P[1] - P[0], P[2] - P[0]))
            r_in = float(n @ P[0])
            cen = P.mean(0)
            # numeral up direction in the face plane
            if "up_vertex" in fd:
                upv = self.V[fd["up_vertex"]]
                up = unit((upv - cen) - ((upv - cen) @ n) * n)
            elif "up_dir" in fd:
                u = np.array(fd["up_dir"], float)
                up = unit(u - (u @ n) * n)
            else:
                up = unit(P[0] - cen)
            right = np.cross(up, n)
            edges = []
            for k in range(len(f)):
                a, b = self.V[f[k]], self.V[f[(k + 1) % len(f)]]
                m = unit(np.cross(b - a, n))          # outward in-plane normal
                g = [x for x in emap[tuple(sorted((f[k], f[(k + 1) % len(f)])))] if x != fi][0]
                ng = unit(np.cross(self.V[Fv[g][1]] - self.V[Fv[g][0]], self.V[Fv[g][2]] - self.V[Fv[g][0]]))
                theta = math.acos(np.clip(n @ ng, -1, 1))   # exterior angle at the crease
                edges.append({"a": a, "m": m, "nbr": g, "theta": theta})
            inst = []   # numeral instances: (centre, up, right, text)
            if name == "d4":
                for k, vi in enumerate(f):
                    vpos = self.V[vi]
                    u = unit(vpos - cen)
                    c = cen + (vpos - cen) * lay["corner_pos"]
                    inst.append((c, u, np.cross(u, n), str(fd["corner_numbers"][k])))
            elif name == "d10":
                apex = self.V[fd["up_vertex"]]
                foot = P[2]   # verts rotated so apex first: T, side, foot, side
                c = apex + (foot - apex) * lay["axis_pos"]
                u = unit(apex - foot)
                inst.append((c, u, np.cross(u, n), str(fd["number"])))
            else:
                c = cen + up * lay.get("offset", 0.0)
                inst.append((c, up, right, str(fd["number"])))
            self.faces.append({"verts": f, "n": n, "r_in": r_in, "cen": cen, "up": up,
                               "right": right, "edges": edges, "inst": inst,
                               "number": fd.get("number"), "P": P})


# ---------------------------------------------------------------- font cache
_FONT = {}


def font_tex(font, tw, th):
    key = (font, tw, th)
    if key not in _FONT:
        cache = os.path.join(HERE, f".sdf2_{font}_{tw}x{th}.npz")
        if os.path.exists(cache):
            z = np.load(cache)
            _FONT[key] = {ch: z[ch] for ch in G.GLYPHS}
        else:
            _FONT[key] = G.build_font(font, tw, th, spread=1.0)
            np.savez(cache, **_FONT[key])
    return _FONT[key]


# ---------------------------------------------------------------- render
def light_dir(s):
    """P12 arc: s=0 low raking from the left, ~1/3 the classic upper-left key,
    s=1 low raking from the right."""
    a = math.pi * s
    return unit([-math.cos(a), 0.80 * math.sin(a), 0.22 + 0.55 * math.sin(a)])


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def hash3(ix, iy, iz):
    h = (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)
    h = (h ^ (h >> 13)) * 1274126177
    h = h ^ (h >> 16)
    return (h & 0xFFFFFF) / float(1 << 24)


def render(die="d20", R=None, W=960, H=540, light=0.33, color="crimson", ink="white",
           finish="gloss", edges="tumbled", font="lato", pips=False, background="gradient",
           video=None, tex=(20, 36), circ=0.475):
    D = Die(die)
    if R is None:
        R = rot(25, -25, 0)
    S = circ * H                       # px per die unit (circumradius 1)
    yy, xx = np.mgrid[0:H, 0:W].astype(float)
    X = (xx + 0.5 - W / 2) / S
    Y = -(yy + 0.5 - H / 2) / S
    Lk = light_dir(light)
    Lfill = unit([0.65, 0.15, 0.75])
    V = np.array([0, 0, 1.0])
    Lpos = Lk * 3.2                    # finite-distance key -> gliding highlights
    body, second, metal = (srgb(c) for c in COLORS[color])
    rho = 0.085 if edges == "tumbled" else 0.010     # fillet radius, die units
    px = 1.0 / S                                      # one pixel in die units

    # -------- per front face: plane hit, face-local coords, inside score
    best = np.full((H, W), -1e9)
    own = np.full((H, W), -1, int)
    Rf = []
    for fi, F in enumerate(D.faces):
        n = R @ F["n"]
        Rf.append(n)
        if n[2] <= 1e-6:
            continue
        z = (F["r_in"] - n[0] * X - n[1] * Y) / n[2]
        Pv = np.stack([X, Y, z], -1)
        Pd = Pv @ R                     # back to die space (R orthonormal)
        # min over edges of the in-plane distance converted to screen px
        sc = None
        for e in F["edges"]:
            dist = (e["a"] - Pd) @ e["m"]            # >0 inside, die units
            me = R @ e["m"]
            ed = R @ np.cross(F["n"], e["m"])
            ed2 = unit(ed[:2]) if np.linalg.norm(ed[:2]) > 1e-9 else np.array([1.0, 0])
            perp = np.array([-ed2[1], ed2[0]])
            k = abs(me[:2] @ perp) * S + 1e-9        # px per die unit across this edge
            dpx = dist * k
            sc = dpx if sc is None else np.minimum(sc, dpx)
        upd = sc > best
        best = np.where(upd, sc, best)
        own = np.where(upd, fi, own)

    # -------- shade the owner of every pixel
    col = np.zeros((H, W, 3))
    alpha = np.zeros((H, W))
    fonttex = font_tex(font, *tex)
    tw, th = tex
    own = np.where(best < -1.5, -1, own)
    for fi, F in enumerate(D.faces):
        m = own == fi
        if not m.any():
            continue
        n_v = Rf[fi]
        Xm, Ym = X[m], Y[m]
        z = (F["r_in"] - n_v[0] * Xm - n_v[1] * Ym) / n_v[2]
        Pv = np.stack([Xm, Ym, z], -1)
        Pd = Pv @ R
        npx = Xm.shape[0]
        # edge distances (die units), silhouette coverage
        dists, cover = [], np.ones(npx)
        tilt = np.zeros((npx, 3))
        nrm_d = np.tile(F["n"], (npx, 1))
        for e in F["edges"]:
            dist = (e["a"] - Pd) @ e["m"]
            dists.append(dist)
            nb_front = (R @ D.faces[e["nbr"]]["n"])[2] > 1e-6
            th_e = e["theta"]
            w = rho * math.tan(th_e / 2)                      # fillet extent on this face
            # tilt angle grows from 0 at d = w to theta/2 at the crease
            s = np.clip((w - dist) / rho, 0, math.sin(th_e / 2))
            phi = np.arcsin(s)
            tilt += np.tan(phi)[:, None] * e["m"][None, :]
            if not nb_front:
                # silhouette edge: rounded outline sits inside the sharp one
                inset = rho * (1 / math.cos(th_e / 2) - 1)
                me = R @ e["m"]
                ed = R @ np.cross(F["n"], e["m"])
                ed2 = unit(ed[:2]) if np.linalg.norm(ed[:2]) > 1e-9 else np.array([1.0, 0])
                perp = np.array([-ed2[1], ed2[0]])
                k = abs(me[:2] @ perp) * S + 1e-9
                cover = np.minimum(cover, np.clip(0.5 + (dist - inset) * k, 0, 1))
        nd = nrm_d + tilt
        nd /= np.linalg.norm(nd, axis=1, keepdims=True)
        dmin = np.min(np.stack(dists), 0)

        # -------- numerals (one glyph sample per pixel)
        lay = LAYOUT[die]
        inkh = lay["ink"]
        cellh = inkh / 0.80
        cellw = cellh * tw / th
        sd = np.full(npx, 9.0)          # die units, >0 outside the ink
        gdir = np.zeros((npx, 3))       # in-plane outward gradient (die space)
        insts = F["inst"]
        if die == "d4":
            # corner region = nearest numeral centre
            dd = np.stack([np.sum((Pd - c) ** 2, 1) for (c, u, r, t) in insts])
            pick = np.argmin(dd, 0)
        else:
            pick = np.zeros(npx, int)
        use_pips = pips and die == "d6"
        for ii, (c, up, right, text) in enumerate(insts):
            sel = pick == ii
            if not sel.any():
                continue
            q = Pd[sel] - c
            gx = q @ right
            gy = q @ up
            if use_pips:
                # pips: analytic discs on a 3x3 grid of the face
                side = np.linalg.norm(F["P"][1] - F["P"][0])
                rp = 0.095 * side
                pos = {1: [(0, 0)], 2: [(-1, 1), (1, -1)], 3: [(-1, 1), (0, 0), (1, -1)],
                       4: [(-1, -1), (-1, 1), (1, -1), (1, 1)],
                       5: [(-1, -1), (-1, 1), (0, 0), (1, -1), (1, 1)],
                       6: [(-1, -1), (-1, 0), (-1, 1), (1, -1), (1, 0), (1, 1)]}[F["number"]]
                best_d = np.full(gx.shape, 9.0)
                bvx = np.zeros(gx.shape)
                bvy = np.zeros(gx.shape)
                for (px_, py_) in pos:
                    vx = gx - px_ * side * 0.27
                    vy = gy - py_ * side * 0.27
                    r = np.hypot(vx, vy)
                    dd_ = r - rp
                    b = dd_ < best_d
                    best_d = np.where(b, dd_, best_d)
                    bvx = np.where(b, vx / (r + 1e-9), bvx)
                    bvy = np.where(b, vy / (r + 1e-9), bvy)
                sd[sel] = best_d
                gdir[sel] = bvx[:, None] * right[None, :] + bvy[:, None] * up[None, :]
                continue
            digits = text
            cond = lay["condense"]
            if len(digits) == 2:
                adv = cellw * cond * 0.5 + lay.get("gap", 0.05) * inkh
                left = gx < 0
                lx = np.where(left, gx + adv, gx - adv)
                chs = np.where(left, digits[0], digits[1])
            else:
                lx = gx
                chs = np.full(gx.shape, digits)
            # glyph texel coords (u right, v down), condensed horizontally
            tu = lx / (cellw * cond) * tw + tw / 2
            tv = -gy / cellh * th + th / 2
            dval = np.full(gx.shape, 1.0)
            ggx = np.zeros(gx.shape)
            ggy = np.zeros(gx.shape)
            for ch in set(chs.tolist()):
                k = chs == ch
                d_, gx_, gy_ = G.sample(fonttex[ch], tu[k], tv[k], spread=1.0)
                dval[k], ggx[k], ggy[k] = d_, gx_, gy_
            texel = cellh / th                                # die units per texel (v)
            sdv = dval * texel
            # gradient back to die space (u right / cond, v down)
            gr = ggx / cond
            gu = -ggy
            gn = np.hypot(gr, gu) + 1e-9
            gd = (gr / gn)[:, None] * right[None, :] + (gu / gn)[:, None] * up[None, :]
            # underline under 6 and 9 on d10/d20
            if die in UNDERLINE and text in ("6", "9"):
                ux = np.abs(gx) - 0.27 * inkh
                uy = np.abs(gy + 0.60 * inkh) - 0.055 * inkh
                ud = np.maximum(ux, uy)
                u_in = ud < sdv
                sdv = np.where(u_in, ud, sdv)
                vert = uy > ux
                udir = np.where(vert[:, None], np.sign(gy + 0.60 * inkh)[:, None] * up[None, :],
                                np.sign(gx)[:, None] * right[None, :])
                gd = np.where(u_in[:, None], udir, gd)
            sd[sel] = sdv
            gdir[sel] = gd

        # engraving: lip at sd=0, wall to sd=-ww, floor beyond.
        texel_d = (LAYOUT[die]["ink"] / 0.80) / th
        ww = 0.65 * texel_d                              # wall width: a fixed SDF code band
        depth = 1.1 * ww
        psi = math.atan2(depth, ww)
        walln = math.cos(psi) * nd - math.sin(psi) * gdir   # wall faces into the cut
        walln /= np.linalg.norm(walln, axis=1, keepdims=True) + 1e-12
        a_lip = np.clip(0.5 - sd / px, 0, 1)                  # 1 inside the cut
        a_floor = np.clip(0.5 - (sd + ww) / px, 0, 1)        # 1 on the floor

        # -------- lighting
        def shade(nn, kd, ks, shin, spec_col, alb):
            nv = nn @ R.T                                     # die -> view
            Lp = Lpos[None, :] - Pv
            Lp /= np.linalg.norm(Lp, axis=1, keepdims=True)
            Hh = Lp + V[None, :]
            Hh /= np.linalg.norm(Hh, axis=1, keepdims=True)
            ndl = np.clip(np.sum(nv * Lp, 1), 0, None)
            ndf = np.clip(nv @ Lfill, 0, None)
            sky = 0.5 + 0.5 * nv[:, 1]
            dif = 0.92 * ndl + 0.30 * ndf + 0.16 * sky + 0.07
            ndh = np.clip(np.sum(nv * Hh, 1), 0, 1)
            # sharp lobe + broad softbox lobe (the glide on flat faces)
            spec = (ks * ndh ** shin + 0.22 * ks * ndh ** (shin * 0.12)) * (ndl > 0)
            return alb * (kd * dif)[:, None] + spec[:, None] * spec_col[None, :], nv

        alb = np.tile(body, (npx, 1))
        white = np.ones(3)
        kd, ks, shin, scol = 1.0, 0.85, 110.0, white
        if finish == "matte":
            ks, shin = 0.10, 10.0
        elif finish == "pearl":
            ks, shin = 0.35, 40.0
        elif finish == "metal":
            kd, ks, shin = 0.10, 1.4, 90.0
            alb = np.tile(srgb(COLORS[color][2]), (npx, 1))
            scol = alb[0] * 0.7 + 0.3
        elif finish == "gem":
            ks, shin = 1.2, 260.0
        elif finish == "marble":
            p = Pd * 2.6
            t = p @ unit([1, 0.4, 0.2]) * 3.0 + 1.6 * np.sin(p @ unit([0.2, 1, 0.5]) * 2.3
                                                              + 1.3 * np.sin(p @ unit([0.6, -0.3, 1]) * 3.1))
            vein = np.abs(np.sin(t)) ** 0.35
            alb = body[None, :] * vein[:, None] + second[None, :] * (1 - vein)[:, None]
            ks, shin = 0.7, 120.0
        elif finish == "wood":
            ax = unit([0.25, 0.15, 1.0])
            q = Pd + np.array([0.37, -0.21, 0.0])
            r = np.sqrt(np.clip(np.sum(q * q, 1) - (q @ ax) ** 2, 0, None))
            ring = (r * 11.0 + 0.35 * np.sin(q @ ax * 7.0) + 0.12 * np.sin(q @ unit([1, 2, 0.3]) * 23)) % 1.0
            gr = smoothstep(0.0, 0.25, ring) * (1 - smoothstep(0.55, 1.0, ring))
            woodl, woodd = srgb((196, 140, 86)), srgb((120, 70, 36))
            base = woodd[None, :] + (woodl - woodd)[None, :] * gr[:, None]
            stain = 0.55 if color not in ("bone", "sunflower") else 0.25
            alb = base * (1 - stain) + base * body[None, :] * 2.2 * stain
            ks, shin = 0.25, 30.0
        c_face, nv = shade(nd, kd, ks, shin, scol, alb)
        if finish == "metal":
            # strong reflection gradient: environment by reflected y, nudged by position
            Rv = 2 * nv[:, 2:3] * nv - V[None, :]
            ry = Rv[:, 1] + 0.55 * Pv[:, 1] + 0.25 * Pv[:, 0]
            # studio environment: dark floor, bright horizon band, soft sky
            env = (0.04 + 0.55 * smoothstep(-0.1, 0.5, ry) + 0.9 * np.exp(-((ry - 0.05) / 0.07) ** 2)
                   - 0.35 * smoothstep(0.55, 0.95, ry))
            c_face = c_face + alb * np.clip(env, 0, None)[:, None] * 1.1
        if finish == "pearl":
            fres = (1 - np.clip(nv[:, 2], 0, 1)) ** 1.5
            sheen = 0.25 + 0.75 * fres
            c_face = c_face * (1 - 0.45 * sheen[:, None]) + second[None, :] * (0.55 * sheen)[:, None] * \
                (0.35 + 0.65 * np.clip(nv @ Lk, 0, 1))[:, None]
        if finish == "gem":
            inr = np.min([np.linalg.norm(F["cen"] - (e["a"] + ((F["cen"] - e["a"]) @ np.cross(F["n"], e["m"])) * np.cross(F["n"], e["m"])))
                          for e in F["edges"]])
            t = np.clip(dmin / (inr + 1e-9), 0, 1)
            deep = body * 0.35
            glow = body * 1.6 + 0.10
            gem = glow[None, :] * (1 - t[:, None]) ** 2 + deep[None, :] * (1 - (1 - t[:, None]) ** 2)
            if video is not None:
                vid = video[yy[m].astype(int), xx[m].astype(int)]
                gem = gem * 0.55 + vid * body[None, :] * 1.4 * (0.35 + 0.65 * t[:, None])
            spec_only = shade(nd, 0.0, ks, shin, scol, alb)[0]
            c_face = gem * (0.65 + 0.55 * np.clip(nv @ Lk, 0, 1))[:, None] + np.clip(spec_only, 0, None)
        if finish == "glitter":
            p = Pd * 34.0
            ip = np.floor(p).astype(np.int64)
            fr = p - ip - 0.5
            h = hash3(ip[:, 0], ip[:, 1], ip[:, 2])
            on = h < 0.20
            ox = (hash3(ip[:, 1], ip[:, 2], ip[:, 0]) - 0.5) * 0.6
            oy = (hash3(ip[:, 2], ip[:, 0], ip[:, 1]) - 0.5) * 0.6
            oz = (hash3(ip[:, 0] + 3, ip[:, 2], ip[:, 1]) - 0.5) * 0.6
            rr = np.sqrt((fr[:, 0] - ox) ** 2 + (fr[:, 1] - oy) ** 2 + (fr[:, 2] - oz) ** 2)
            dot = np.clip(1.0 - (rr - 0.09) / (px * 34.0 * 1.0), 0, 1) * on
            facet = unit(np.array([1.0, 0.3, 0.6]))
            tw_ = (hash3(ip[:, 0] + 7, ip[:, 1], ip[:, 2]) * 6.283)
            fn = np.stack([np.cos(tw_), np.sin(tw_), np.full_like(tw_, 0.8)], 1)
            fn /= np.linalg.norm(fn, axis=1, keepdims=True)
            fnv = (fn @ R.T)
            Hc = unit(Lk + V)
            twinkle = np.clip(fnv @ Hc, 0, 1) ** 24
            c_face = c_face + (second * 0.25 + 1.2 * twinkle[:, None]) * dot[:, None]

        # cut walls (lit / shadowed) and ink floor
        c_wall, _ = shade(walln, kd if finish != "metal" else 0.5, ks * 0.6, max(shin * 0.5, 8), scol, alb)
        if ink == "unpainted":
            ink_alb = alb * 0.78
        elif ink == "tonal":
            ink_alb = alb * 0.42
        else:
            ink_alb = np.tile(srgb(INKS[ink]), (npx, 1))
        nvf = nd @ R.T
        ndl_f = np.clip(nvf @ Lk, 0, None)
        # lip shadow on the floor: a wall between the floor point and the light
        Lt = Lk - (nvf @ Lk)[:, None] * nvf
        g_v = gdir @ R.T
        toward = np.sum(Lt * g_v, 1)                           # >0: light comes over this edge
        shadow_len = depth * np.clip(toward, 0, None) / (np.clip(nvf @ Lk, 0.05, None))
        inside_depth = -(sd + ww)                              # distance onto the floor
        lit = smoothstep(-px, px, inside_depth - shadow_len)
        dif_f = 0.85 * ndl_f * lit + 0.22 * np.clip(nvf @ Lfill, 0, None) + 0.12
        c_floor = ink_alb * dif_f[:, None] * 0.92
        c = c_face * (1 - a_lip)[:, None] + (c_wall * (1 - a_floor)[:, None] + c_floor * a_floor[:, None]) * a_lip[:, None]
        col[m] = c
        sc = best[m]
        alpha[m] = np.where(sc < 0, np.minimum(cover, np.clip(0.5 + sc, 0, 1)), cover)

    # -------- backdrop
    if background == "video" and video is not None:
        bg = video
    else:
        t = yy / H
        g0 = srgb((172, 174, 178)) * (1 - t)[..., None] + srgb((104, 106, 110)) * t[..., None]
        r2 = ((xx - W / 2) / (0.75 * W)) ** 2 + ((yy - H / 2) / (0.75 * H)) ** 2
        bg = g0 * (1 - 0.35 * np.clip(r2, 0, 1))[..., None]
        # soft contact shadow under the die
        sy = H / 2 + S * 1.04
        e = ((xx - W / 2) / (0.80 * S)) ** 2 + ((yy - sy) / (0.11 * S)) ** 2
        bg = bg * (1 - 0.38 * np.exp(-1.6 * e))[..., None]
    out = col * alpha[..., None] + bg * (1 - alpha[..., None])
    return to_srgb8(out)


HERO_TILT = {"d4": (6, -8), "d6": (22, -24), "d8": (16, -18), "d10": (10, -16), "d20": (16, -18)}


def present(die, face, tilt=None, roll=0.0):
    """Rotation that faces `face` toward the camera, numeral upright, then a
    small hero tilt so neighbouring faces show."""
    D = Die(die)
    F = D.faces[face]
    n, up = F["n"], F["up"]
    if die == "d4":
        # d4 presents a vertex: the corner whose number is the 'result'
        vi = F["verts"][0]
        n = unit(D.V[vi])
        others = [D.V[x] for x in range(4) if x != vi]
        up = unit(others[0] - (others[0] @ n) * n)
    right = np.cross(up, n)
    M = np.stack([right, up, n])          # rows: die->view basis
    t = tilt if tilt is not None else HERO_TILT[die]
    return rot(t[0], t[1], roll) @ M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--die", default=None)
    ap.add_argument("--face", type=int, default=None)
    ap.add_argument("--yaw", type=float, default=None)
    ap.add_argument("--pitch", type=float, default=-24)
    ap.add_argument("--roll", type=float, default=0)
    ap.add_argument("--light", type=float, default=0.33)
    ap.add_argument("--color", default="crimson")
    ap.add_argument("--ink", default="white")
    ap.add_argument("--finish", default="gloss")
    ap.add_argument("--edges", default="tumbled")
    ap.add_argument("--pips", action="store_true")
    ap.add_argument("--W", type=int, default=960)
    ap.add_argument("--H", type=int, default=540)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.die:
        if a.face is not None:
            R = present(a.die, a.face)
        else:
            R = rot(a.yaw if a.yaw is not None else 25, a.pitch, a.roll)
        img = render(a.die, R, a.W, a.H, a.light, a.color, a.ink, a.finish, a.edges, pips=a.pips)
        out = a.out or os.path.join(HERE, f"prev_{a.die}.png")
        Image.fromarray(img).save(out)
        print("wrote", out)


if __name__ == "__main__":
    main()
