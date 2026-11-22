#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEON BEACH - build hook.  Writes neonbeach_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  `python3 neonbeach.py --preview DIR` also dumps the sprite art and the palettes as PNGs
(needs PIL; the build never uses it).

Package contents
  C_NB_PAL   256 x 48   colour table (the initial contents of a RAM: vblank rewrites the per-field entries).
                        Address = theme (1) & time of day (2) & entry (5).
                        Word = Y0 U0 V0 dY dU dV, 8 bits each (10-bit value >> 2, hardware convention U = Cr,
                        V = Cb; d = signed, half the step to the next key).  colour = c0 + (d * t) >> 5, t 0..63.
                        Time of day 0 day, 1 low sun, 2 night:
                          entry 2 r + s   fill ramp r (0, 1, 5, 6, 7), segment s (key s -> key s + 1)
                          entries 4 .. 7  the water, four segments (K0 deep .. K4 top)          [per field]
                          entry 16 + w    wire colour of object w, lerped toward the palette's hot colour
                                          (21 water and 23 wash: K4 -> white                   [per field])
                          entry 16 spray, 19 the sky (zenith -> horizon) [per field], 27 star, 28 / 29 foam
                        Time of day 3 = the keyframes: entries 0 .. 7 the zenith, 8 .. 15 the horizon (keyframe i
                        lerping to i + 1, the sky follows the sun's elevation smoothly); the water's keys
                        16 + 4 k + i (K0 .. K3) and 19 + 4 i (K4), keyframe i -> i + 1 for i = 0 .. 2 (night, dusk,
                        golden, day: the water follows the sun gradually); 31 white.
  C_NB_LROM  1024 x 16  per-line tables: the circle's half-chord, then the span-coded sprite rows
                        (4 words a row: b1 b0 / b3 b2 / b5 b4 / zones z0 .. z6, 2 bits each, z0 in the LSBs,
                        bit 14 = the row's crease parity; zone = z[number of boundaries <= x])
  C_NB_BED   128 x 10   sea bed / beach row at each knot (16 virtual px apart from the canvas's left edge)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nbengine  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------
# Palettes.  Ramps (3 keys each, t = 0 .. 127 runs key0 -> key1 -> key2):
#   0 sky (lit by the camera with the fill off)   4 (unused: the wash takes the water's top colour)
#   1 sun / moon  bottom .. top                    5 sand      wet / deep .. dry surface
#   2 (unused: the water is per field)             6 flora     clouds, gulls: dark .. bright
#   3 foam        turning white .. white           7 crab      dark .. body .. highlight
# Wires (by object): 1 sun, 2 cloud / gull, 5 water, 6 foam, 7 wash, 8 sand, 9 crab, 11 star, 12 spray.
# water: the static water keys (K0 deep .. K4 top) only seed the RAM before the first field.
# wkf (per theme): the water's keyframes, night / dusk / golden / day, each K0 .. K4 (Pixel Beach's DEEP, MID,
# AQUA, SURF bands, with a neon top), lerped by the sun's elevation every field.
# ---------------------------------------------------------------------------------------------------------
def T(ramps, wires, hot="#FFFFFF"):
    return dict(ramps=ramps, wires=wires, hot=hot)

THEMES = [
    dict(name="Neon",
         tod=[
             T([("#0A1A4A", "#1E5AB4", "#7FD4FF"), ("#FF6A2A", "#FFC23D", "#FFF6B0"),
                None, ("#9AE8E8", "#E0FCFF", "#FFFFFF"),
                None, ("#704820", "#E8C080", "#FFF4D8"),
                ("#2A2A3A", "#9AA8C8", "#FFFFFF"), ("#3A0008", "#E8203C", "#FF9A6A")],
               {1: "#FFE14D", 2: "#FFFFFF", 5: "#00E5FF", 6: "#A8F0F8", 7: "#7FFFF4", 8: "#FFB000",
                9: "#FF3B6B", 11: "#FFFFFF", 12: "#E0FFFF"}),
             T([("#1A0A3A", "#8A1F8A", "#FF7A3D"), ("#C8102E", "#FF4A1A", "#FFB04D"),
                None, ("#E8B8E0", "#FFE8F8", "#FFFFFF"),
                None, ("#4A2018", "#C87850", "#FFC890"),
                ("#1A0A1A", "#6A3A5A", "#FFD0B0"), ("#2A0004", "#C81028", "#FF8040")],
               {1: "#FFB040", 2: "#FFC8E0", 5: "#FF4FD8", 6: "#FFC8E8", 7: "#FF9AE0", 8: "#FF7A00",
                9: "#FF2A4A", 11: "#FFFFFF", 12: "#FFE0F0"}),
             T([("#000008", "#0A0A30", "#2A3A7A"), ("#5A6A8A", "#B8C8E0", "#F0F8FF"),
                None, ("#7AB0C8", "#C8E8F8", "#F0F8FF"),
                None, ("#0C0C14", "#505068", "#9898B0"),
                ("#08080F", "#3A4058", "#A8B0D0"), ("#10000A", "#7A1030", "#C84060")],
               {1: "#E0F0FF", 2: "#A8B8E0", 5: "#2A9AFF", 6: "#90C8E8", 7: "#5AC8FF", 8: "#8A8AC8",
                9: "#C82A6A", 11: "#FFFFFF", 12: "#A8D8FF"}, "#E8F0FF"),
         ],
         # (neighbouring keyframes are picked so their lerp never passes through grey: the top goes aqua -> mint
         # -> pale gold -> salmon -> pink -> lavender -> moonlit blue)
         wkf=[["#020818", "#082048", "#184070", "#306888", "#5090C0"],      # night
              ["#0C0C30", "#202868", "#404890", "#9060B0", "#F080B0"],      # dusk (sun just under the horizon)
              ["#0A1C48", "#183878", "#3068A8", "#58A8A0", "#E8E0A0"],      # golden (sun low)
              ["#06185A", "#104098", "#2070CC", "#38B4C4", "#80F0E0"]],     # day
         zenith=["#000006", "#0A0628", "#2A0A4A", "#4A1060", "#2A2A88", "#1E4AB0", "#2E78D8", "#3A9AF0"],
         horizon=["#06102A", "#3A1050", "#C8287A", "#FF6A3A", "#FFB060", "#FFE0A0", "#BFF0FF", "#9AE4FF"]),
    dict(name="Ice",
         tod=[
             T([("#020B1F", "#12508C", "#C8F4FF"), ("#7FD8FF", "#D4F4FF", "#FFFFFF"),
                None, ("#A8E8F8", "#E8FCFF", "#FFFFFF"),
                None, ("#1A1630", "#9090B8", "#ECECFF"),
                ("#02202E", "#468FAF", "#FFFFFF"), ("#1A0A2A", "#B02A8A", "#FF8AD0")],
               {1: "#E8FBFF", 2: "#FFFFFF", 5: "#00B4D8", 6: "#B0F0FF", 7: "#90E0EF", 8: "#B8A8FF",
                9: "#FF6B9D", 11: "#FFFFFF", 12: "#E0FFFF"}),
             T([("#0A0628", "#4A2A8A", "#FF9AC8"), ("#FF5A8A", "#FFA8C8", "#FFF0F8"),
                None, ("#C8B0F0", "#F0E8FF", "#FFFFFF"),
                None, ("#1A0A2A", "#8A5A98", "#F8C8E8"),
                ("#0A0420", "#5A4A8A", "#FFE0F0"), ("#20001A", "#C81060", "#FF9AB0")],
               {1: "#FFA8D0", 2: "#FFE0F0", 5: "#B87AFF", 6: "#E0C8FF", 7: "#E0A8FF", 8: "#FF8AC8",
                9: "#FF3A80", 11: "#FFFFFF", 12: "#F0E0FF"}),
             T([("#000006", "#020A28", "#0A2A5A"), ("#6A7A9A", "#C8D8F0", "#FFFFFF"),
                None, ("#6A9AC0", "#C0DCF0", "#F0F8FF"),
                None, ("#06060F", "#3A3A5A", "#8080A8"),
                ("#04040F", "#2A3A5A", "#90A8C8"), ("#0A0010", "#5A1040", "#A84078")],
               {1: "#F0F8FF", 2: "#90A8D0", 5: "#1A8AE8", 6: "#90C0E0", 7: "#4AB0F0", 8: "#7A7AC0",
                9: "#B02A70", 11: "#FFFFFF", 12: "#A8D8FF"}, "#E8F0FF"),
         ],
         wkf=[["#000210", "#021432", "#0A3A6A", "#1A5A8A", "#3A8AB8"],
              ["#08041E", "#201450", "#3A1A7A", "#7A50C8", "#C890FF"],
              ["#060A28", "#18306A", "#3A5AA0", "#7A9AD8", "#C8E0FF"],
              ["#010C20", "#06306A", "#0B5FA0", "#3AB8E0", "#A0F8FF"]],
         zenith=["#000004", "#020420", "#0A0A3A", "#1A1060", "#0A2A70", "#0A4A90", "#1A6AB8", "#2A88D8"],
         horizon=["#020A20", "#0A1A4A", "#5A2A8A", "#C85AA8", "#FF9AC8", "#E8D0FF", "#C0F4FF", "#A8ECFF"]),
]
TOD_WKF = [3, 1, 0]          # the keyframe that seeds each time of day's water before the first field
TOD_SKY = [6, 3, 0]          # and its sky entry


def hw8(h):
    """sRGB hex -> (y, u, v) 8-bit (10-bit >> 2), hardware convention (u = Cr, v = Cb), BT.601."""
    h = h.lstrip("#")
    r, g, b = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    f = lambda v: max(0, min(255, int(round(v / 4.0))))
    return (f(64 + y * 876), f(512 + cr * 896), f(512 + cb * 896))


def pal_entry(c0, c1):
    a, b = hw8(c0), hw8(c1)
    d = [int(round((b[i] - a[i]) / 2.0)) for i in range(3)]
    w = 0
    for v in (a[0], a[1], a[2], d[0] & 255, d[1] & 255, d[2] & 255):
        w = (w << 8) | v
    return w


E_WATER, E_SPRAY, E_SKY, E_FOAM, E_HOT = 4, 16, 19, 28, 31


def wkey_entry(k, i):
    """The keyframe slot's entry for water key k (0 .. 4) and keyframe interval i (0 .. 2)."""
    return 19 + 4 * i if k == 4 else 16 + 4 * k + i


def pal_rom():
    out = []
    for th in THEMES:
        for tod, p in enumerate(th["tod"]):
            ent = [0] * 32
            for r, keys in enumerate(p["ramps"]):
                if keys is None or r == 3:
                    continue
                k0, k1, k2 = keys
                ent[2 * r] = pal_entry(k0, k1)
                ent[2 * r + 1] = pal_entry(k1, k2)
            for w in range(16):
                c = p["wires"].get(w, "#FFFFFF")
                ent[16 + w] = pal_entry(c, p["hot"])
            # the per-field entries' seeds
            wk = th["wkf"][TOD_WKF[tod]]
            for s in range(4):
                ent[E_WATER + s] = pal_entry(wk[s], wk[s + 1])
            ent[21] = ent[23] = pal_entry(wk[4], "#FFFFFF")
            ent[E_SKY] = pal_entry(th["zenith"][TOD_SKY[tod]], th["horizon"][TOD_SKY[tod]])
            # spray (moved from 28), the foam's fill (two segments)
            ent[E_SPRAY] = pal_entry(p["wires"][12], p["hot"])
            f0, f1, f2 = p["ramps"][3]
            ent[E_FOAM] = pal_entry(f0, f1)
            ent[E_FOAM + 1] = pal_entry(f1, f2)
            for e in (8, 9, 20, 26, 30, 31):
                ent[e] = 0
            out += ent
        ent = [0] * 32
        for base, keys in ((0, th["zenith"]), (8, th["horizon"])):
            for i in range(8):
                ent[base + i] = pal_entry(keys[i], keys[min(7, i + 1)])
        for k in range(5):
            for i in range(3):
                ent[wkey_entry(k, i)] = pal_entry(th["wkf"][i][k], th["wkf"][i + 1][k])
        ent[E_HOT] = pal_entry("#FFFFFF", "#FFFFFF")
        out += ent
    return out


# ---------------------------------------------------------------------------------------------------------
# Sprite art: zone canvases (list of rows of small ints), drawn with a few pure-python primitives
# ---------------------------------------------------------------------------------------------------------
def canvas(w, h):
    return [[0] * w for _ in range(h)]


def poly(cv, pts, z):
    h, w = len(cv), len(cv[0])
    n = len(pts)
    for y in range(h):
        yc = y + 0.5
        xs = []
        for i in range(n):
            (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
            if (y0 <= yc < y1) or (y1 <= yc < y0):
                xs.append(x0 + (yc - y0) * (x1 - x0) / (y1 - y0))
        xs.sort()
        for i in range(0, len(xs) - 1, 2):
            for x in range(max(0, int(math.ceil(xs[i] - 0.5))), min(w, int(math.floor(xs[i + 1] - 0.5)) + 1)):
                cv[y][x] = z


def disc(cv, cx, cy, rx, ry, z):
    h, w = len(cv), len(cv[0])
    for y in range(max(0, int(cy - ry - 1)), min(h, int(cy + ry + 2))):
        for x in range(max(0, int(cx - rx - 1)), min(w, int(cx + rx + 2))):
            if ((x + 0.5 - cx) / rx) ** 2 + ((y + 0.5 - cy) / ry) ** 2 <= 1.0:
                cv[y][x] = z


def thick(cv, p0, p1, r, z):
    n = int(max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))) + 1
    for i in range(n + 1):
        s = i / float(n)
        disc(cv, p0[0] + (p1[0] - p0[0]) * s, p0[1] + (p1[1] - p0[1]) * s, r, r, z)


def hbits(lines, rows, step):
    """Per-ROM-row parity that flips at each virtual row in `lines`: a wire across the sprite's body there."""
    return [sum(1 for l in lines if l <= r * step) & 1 for r in range(rows)]


def art_cloud():
    """256 x 48 units (2 virtual px a unit both ways): zone 1 top, zone 2 the lower shelf."""
    cv = canvas(256, 48)
    for (cx, cy, rx, ry) in [(60, 34, 34, 12), (96, 24, 30, 18), (136, 18, 34, 17), (176, 27, 30, 15),
                             (206, 35, 26, 10), (118, 34, 80, 10)]:
        disc(cv, cx, cy, rx, ry, 1)
    for y in range(48):
        for x in range(256):
            if y >= 41:
                cv[y][x] = 0
            elif y >= 33 and cv[y][x]:
                cv[y][x] = 2
    return cv, [0] * 48


def bar(cv, p0, p1, w, z):
    """A straight bar of constant width w with square caps (a quad running w / 2 past each end, so two bars
    meeting at a joint overlap instead of leaving a notch)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n * w / 2.0, dy / n * w / 2.0
    ox, oy = -uy, ux
    a = (p0[0] - ux, p0[1] - uy)
    b = (p1[0] + ux, p1[1] + uy)
    poly(cv, [(a[0] + ox, a[1] + oy), (b[0] + ox, b[1] + oy), (b[0] - ox, b[1] - oy), (a[0] - ox, a[1] - oy)], z)


def art_birds(frame):
    """256 x 32 units (2 virtual px a unit both ways, so 512 x 64 on screen): three gulls, each two long flat
    planks meeting at the centre (no body), wings up (frame 0, a shallow V) / down (frame 1).  Zone 1 the left
    wing, zone 2 the right (two planes, two shades).  The gulls never share a row (six boundaries a row)."""
    cv = canvas(256, 32)
    L, TH = 34, 3
    for (cx, base) in [(50, 0), (128, 11), (210, 22)]:
        rise = 6 if frame == 0 else -3            # the wing tips above (+) or below (-) the centre
        vy = base + (6 if frame == 0 else 0)      # the centre's top row
        for i in range(-L, L + 1):
            if i == 0:
                continue
            yc = vy - rise * abs(i) / float(L)
            for y in range(int(round(yc)), int(round(yc)) + TH):
                if 0 <= y < 32:
                    cv[y][cx + i] = 1 if i < 0 else 2
        for y in range(vy, vy + TH):
            cv[y][cx] = 2
    return cv, [0] * 32


def art_crab(frame):
    """Half a crab, the centre line at x = 0 (the engine folds |x|), drawn in straight lines: a hexagonal shell
    cut into facet bands, bars for legs (thigh up, shin down, sharp knees), angular pincers, diamond eyes on
    stalks.  Canvas units are half a virtual px both ways (the pixel path doubles x into the ROM, one ROM row a
    virtual row): 256 x 96 units = 128 x 48 virtual px half-width, so the crab is about 122 x 48 on screen (half
    v0.3's size).  Zone 1 the shell (creases = row parity: the facet bands), 2 eyes / mouth, 3 claws and legs.
    Frame 1 lifts the other legs."""
    cv = canvas(256, 96)
    # legs nested (the outer one rooted highest), each a thigh out and a little down, then a shin straight down:
    # one crossing per leg per row, so a row never needs more than six boundaries
    # (a step lifts a foot and swings it out; the knee stays below the root, so the thigh never rises)
    lift = [0, 1, 0] if frame == 0 else [1, 0, 1]
    legs = [((36, 66), (74, 69), (80, 93)), ((42, 57), (88, 61), (95, 93)), ((48, 49), (102, 52), (110, 93))]
    for i, (root, knee, foot) in enumerate(legs):
        l = lift[i]
        knee = (knee[0] + 2 * l, knee[1] - 2 * l)
        foot = (foot[0] + 5 * l, foot[1] - 5 * l)
        bar(cv, root, knee, 6, 3)
        bar(cv, knee, foot, 6, 3)
    bar(cv, (50, 40), (72, 24), 6, 3)                                                             # arm
    poly(cv, [(68, 26), (84, 4), (110, 2), (94, 13), (114, 18), (92, 30)], 3)                    # pincer
    bar(cv, (8, 32), (12, 16), 4, 1)                                                              # eye stalk
    poly(cv, [(12, 6), (19, 12), (12, 18), (5, 12)], 2)                                          # eye
    poly(cv, [(0, 30), (34, 30), (58, 42), (62, 54), (54, 70), (0, 70)], 1)                      # shell
    poly(cv, [(34, 30), (58, 42), (40, 42)], 2)                                                  # shoulder facet
    rows = [cv[y] for y in range(0, 96, 2)]
    return rows, hbits([42, 54, 62], 48, 2)


def row_words(row, hb=0):
    """A 256-wide row of zones (+ the row's crease parity) -> 4 words."""
    segs = []
    for x, z in enumerate(row):
        if not segs or segs[-1][0] != z:
            segs.append([z, x])
    while len(segs) > 7:
        best = None
        for i in range(1, len(segs)):
            end = segs[i + 1][1] if i + 1 < len(segs) else 256
            ln = end - segs[i][1]
            if best is None or ln < best[0]:
                best = (ln, i)
        del segs[best[1]]
        m = [segs[0]]
        for s in segs[1:]:
            if s[0] != m[-1][0]:
                m.append(s)
        segs = m
    zones = [s[0] for s in segs]
    bounds = [s[1] for s in segs[1:]]
    while len(bounds) < 6:
        bounds.append(255)
        zones.append(zones[-1])
    zw = 0
    for i, z in enumerate(zones):
        zw |= (z & 3) << (2 * i)
    return [bounds[0] | (bounds[1] << 8), bounds[2] | (bounds[3] << 8), bounds[4] | (bounds[5] << 8), zw | (hb << 14)]


def decode_row(words):
    b = [words[0] & 255, words[0] >> 8, words[1] & 255, words[1] >> 8, words[2] & 255, words[2] >> 8]
    out = []
    for x in range(256):
        k = sum(1 for v in b if x >= v)
        out.append((words[3] >> (2 * k)) & 3)
    return out


# line ROM layout (words)
LR_CIRCLE = 0                       # 256 half-chords, two a word (even index in the low byte)
LR_CLOUD, CLOUD_ROWS = 128, 48
LR_BIRD, BIRD_ROWS = 320, 32        # frame 1 at + 128
LR_CRAB, CRAB_ROWS = 576, 48        # frame 1 at + 192

SPRITES = [("cloud", LR_CLOUD, art_cloud), ("bird0", LR_BIRD, lambda: art_birds(0)),
           ("bird1", LR_BIRD + 128, lambda: art_birds(1)), ("crab0", LR_CRAB, lambda: art_crab(0)),
           ("crab1", LR_CRAB + 192, lambda: art_crab(1))]


def line_rom():
    rom = [0] * 1024
    ch = [min(255, int(round(math.sqrt(max(0.0, 256.0 * 256.0 - (i + 0.5) ** 2))))) for i in range(256)]
    for i in range(128):
        rom[i] = ch[2 * i] | (ch[2 * i + 1] << 8)
    for _, base, fn in SPRITES:
        rows, hb = fn()
        for r, row in enumerate(rows):
            rom[base + 4 * r: base + 4 * r + 4] = row_words(row, hb[r])
    return rom


# ---------------------------------------------------------------------------------------------------------
# Sea bed and beach: row (from the top of the 1080-row canvas) at x = 16 k virtual px
# ---------------------------------------------------------------------------------------------------------
BED_PTS = [(0, 512), (300, 516), (520, 548), (860, 626), (1180, 730), (1440, 800), (1640, 828), (2048, 832)]


def bed_row(x):
    x = max(0, min(2048, x))
    for (x0, y0), (x1, y1) in zip(BED_PTS, BED_PTS[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / float(x1 - x0)
    return BED_PTS[-1][1]


def bed_tab():
    # piecewise linear, then a 7-knot box twice: no corners and no flat steps
    v = [bed_row(16 * k) for k in range(-8, 136)]
    for _ in range(2):
        v = [sum(v[max(0, i - 3):i + 4]) / len(v[max(0, i - 3):i + 4]) for i in range(len(v))]
    return [int(round(v[k + 8])) for k in range(128)]


# ---------------------------------------------------------------------------------------------------------
# Waves (the filler).  Depth index i = min(15, still depth >> 4) (rows).
#   C_NB_SOFF  128 x 16  each knot's phase offset (4096 = a wave), shortening over the rising bed
#   C_NB_DT2   16 x 8    shoal (5: amplitude gain, eighths) << 3 | class (wave shape, 0 round .. 3 breaking)
#   C_NB_SIZE  16 x 4    each wave's size in eighths (4 .. 11), by its number: waves come in sets
#   C_NB_SHAPE 512 x 8   wave profile by class (2) & phase (7): signed, 64 = the amplitude.  Phase 0 is the
#                        crest; the front (the shoreward, left side) is the end of the phase.
#   C_NB_UCODE 512 x 24  the engine's program (nbengine.py)
#   C_NB_SCINIT 512 x 16 the engine's scratch RAM at start: 128 .. 255 the phase offsets, 256 .. 383 the bed
# ---------------------------------------------------------------------------------------------------------
WAVE_DEEP_KNOTS = 32.0                        # wavelength in deep water (16 px knots)


def dtab():
    out = []
    for i in range(16):
        d = 16 * i + 8.0
        lam = WAVE_DEEP_KNOTS * math.sqrt(min(1.0, max(d, 8.0) / 240.0))
        ks = int(round(65536.0 / lam))
        sh = int(round(8.0 * (240.0 / max(d, 12.0)) ** 0.25))
        sh = max(8, min(16, sh))
        cl = max(0, min(7, int(round(7 - i / 2.0))))
        out.append((ks, sh, cl))
    return out


SIZES = [6, 7, 9, 11, 8, 6, 5, 7, 10, 8, 6, 7, 9, 6, 5, 8]


def shape_tab():
    """4 classes x 128 phases: 0 round (deep) .. 3 a steep breaking front."""
    out = []
    for c in range(4):
        F = 0.5 - 0.42 * c / 3.0                 # the front's share of the wave
        q = 1.0 + 1.3 * c / 3.0                  # peaked crests, flat troughs
        ys = []
        for p in range(128):
            ph = (p + 0.5) / 128.0
            if ph < 1.0 - F:
                y = math.cos(math.pi * ph / (1.0 - F))
            else:
                y = -math.cos(math.pi * (ph - (1.0 - F)) / F)
            y = 2.0 * ((y + 1.0) / 2.0) ** q - 1.0
            ys.append(y)
        m = sum(ys) / len(ys)
        out += [max(-127, min(127, int(round(64.0 * (y - m))))) for y in ys]
    return out


def dt2_tab():
    """By depth index: shoal << 3 | class (0 .. 3)."""
    return [(sh << 3) | (cl >> 1) for ks, sh, cl in dtab()]


def soff_tab():
    """Each knot's phase offset (4096 = one wave), accumulated from the right edge at the mean sea level, so the
    waves shorten over the rising bed."""
    bed = bed_tab()
    acc, out = 0, [0] * 128
    for k in range(127, -1, -1):
        d = bed[k] - 626
        i = min(15, d >> 4) if d > 0 else 0
        acc += dtab()[i][0]
        out[k] = int(round(acc / 16.0)) & 0xFFFF
    return out


def scinit_tab():
    v = [0] * 512
    v[nbengine.S_CX], v[nbengine.S_CY], v[nbengine.S_CT] = 540, 700, 40      # the crab's first place
    for k, (o, b) in enumerate(zip(soff_tab(), bed_tab())):
        v[128 + k] = o
        v[256 + k] = b
    return v


def vhd_table(name, depth, width, vals):
    lines = ["    type t_%s is array (0 to %d) of unsigned(%d downto 0);" % (name.lower(), depth - 1, width - 1),
             "    constant %s : t_%s := (" % (name, name.lower())]
    if width % 4 == 0:
        items = ['x"%0*X"' % (width // 4, v) for v in vals]
    else:
        items = ['"' + format(v, "0%db" % width) + '"' for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def write_pkg():
    parts = ["-- Generated by neonbeach.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package neonbeach_pkg is",
             vhd_table("C_NB_PAL", 256, 48, pal_rom()),
             vhd_table("C_NB_LROM", 1024, 16, line_rom()),
             vhd_table("C_NB_BED", 128, 10, bed_tab()),
             vhd_table("C_NB_SOFF", 128, 16, soff_tab()),
             vhd_table("C_NB_DT2", 16, 8, dt2_tab()),
             vhd_table("C_NB_SIZE", 16, 4, SIZES),
             vhd_table("C_NB_SHAPE", 512, 8, [v & 255 for v in shape_tab()]),
             vhd_table("C_NB_UCODE", 512, 24, (nbengine.ucode() + [0] * 512)[:512]),
             vhd_table("C_NB_SCINIT", 512, 16, scinit_tab()),
             "end package neonbeach_pkg;", ""]
    with open(os.path.join(HERE, "neonbeach_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


def preview(outdir):
    from PIL import Image
    os.makedirs(outdir, exist_ok=True)
    rom = line_rom()
    cols = [(10, 10, 20), (60, 220, 200), (255, 90, 170), (255, 230, 90)]
    tiles = []
    for name, base, fn in SPRITES:
        rows = len(fn()[0])
        z = [decode_row(rom[base + 4 * r: base + 4 * r + 4]) for r in range(rows)]
        hb = [(rom[base + 4 * r + 3] >> 14) & 1 for r in range(rows)]
        ident = [[(z[r][x], hb[r] if z[r][x] == 1 else 0) for x in range(256)] for r in range(rows)]
        im = Image.new("RGB", (256, rows))
        for r in range(rows):
            for x in range(256):
                edge = (x > 0 and ident[r][x] != ident[r][x - 1]) or (r > 0 and ident[r][x] != ident[r - 1][x])
                im.putpixel((x, r), (255, 255, 255) if edge else tuple(c // 3 for c in cols[z[r][x]]))
        tiles.append(im.resize((512, rows * 4), Image.NEAREST))
    W = 2 * 512 + 30
    H = sum(t.size[1] for t in tiles[0::2]) + 60
    sheet = Image.new("RGB", (W, max(H, 600)), (40, 40, 40))
    y0 = [10, 10]
    for i, t in enumerate(tiles):
        c = i % 2
        sheet.paste(t, (10 + c * 522, y0[c]))
        y0[c] += t.size[1] + 10
    sheet.save(os.path.join(outdir, "sprites.png"))
    # palettes: per theme, per time of day, ramps and wires; then the sky keyframes
    pal = pal_rom()
    im = Image.new("RGB", (32 * 24, 8 * 24), (0, 0, 0))
    def rgb(w):
        y, u, v = (w >> 40) & 255, (w >> 32) & 255, (w >> 24) & 255
        Y = (y * 4 - 64) / 876.0; Cr = (u * 4 - 512) / 896.0; Cb = (v * 4 - 512) / 896.0
        r = Y + 1.402 * Cr; b = Y + 1.772 * Cb; g = (Y - 0.299 * r - 0.114 * b) / 0.587
        return tuple(max(0, min(255, int(c * 255))) for c in (r, g, b))
    for p in range(8):
        for e in range(32):
            c = rgb(pal[32 * p + e])
            for yy in range(22):
                for xx in range(22):
                    im.putpixel((e * 24 + xx, p * 24 + yy), c)
    im.save(os.path.join(outdir, "palettes.png"))


if __name__ == "__main__":
    write_pkg()
    if len(sys.argv) > 2 and sys.argv[1] == "--preview":
        preview(sys.argv[2])
