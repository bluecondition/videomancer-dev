#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEONSCAPE v0.3.0 - build hook.  Writes neonscape_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  `python3 neonscape.py --preview DIR` also dumps the sprite art and the
palettes as PNGs (needs PIL; the build never uses it).

Package contents
  C_NS_PAL   256 x 48   colour table, 8 palettes x 32 entries.  Word = Y0 U0 V0 dY dU dV, 8 bits each
                        (Y0.. = 10-bit value >> 2, hardware convention U = Cr, V = Cb; d = signed, half the
                        step to the next key).  colour = c0 + (d * t) >> 5, t = 0 .. 63.
                          entry 2 r + s   fill ramp r (0 .. 7), segment s (key s -> key s + 1)
                          entry 16 + w    wire colour of object w, lerped toward the palette's hot colour
                          entry 26        the grid's wire colour, lerped toward the floor's glow (ramp 5, key 2)
  C_NS_LROM  2048 x 16  per-line tables: the circle's half-chord, then the span-coded sprite rows
                        (4 words a row: b1 b0 / b3 b2 / b5 b4 / zones z0 .. z6, 2 bits each, z0 in the LSBs,
                        bit 14 = the row's crease parity; zone = z[number of boundaries <= x])
  C_NS_NEAR  64 x 6     near range knots: A[c + 1] & A[c]  (3 bits each, 64 rows a unit, cells 128 wide)
  C_NS_FAR   64 x 6     far range knots:  A[c + 1] & A[c]  (3 bits each, 32 rows a unit, cells 64 wide)
  C_NS_CITY  64 x 16    skyline: mastB mastA styleB styleA wide hB hA  (cells 128 wide; heights in 32-row floors)
  C_NS_RAIN  32 x 5     rain columns: present & phase(4)
  C_NS_WIN   64 x 1     lit-window pattern
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------
# Palettes.  Ramps (3 keys each, t = 0 .. 127 runs key0 -> key1 -> key2):
#   0 sky      top .. horizon              4 near range  foot .. peak (+ lit slopes)
#   1 sun      bottom .. top               5 floor       deep .. horizon glow
#   2 far range foot .. peak               6 flora       palm / birds dark .. cloud bright
#   3 city     foot .. roof .. lit window  7 car         glass .. body .. lights
# Wires (by object): 1 sun, 2 cloud / bird, 3 far range, 4 city, 5 near range, 6 palm, 7 grid, 8 road,
#   9 car, 11 star, 12 rain;  hot = what a wire flashes toward when the sheen passes.
# ---------------------------------------------------------------------------------------------------------
def P(name, ramps, wires, hot):
    return dict(name=name, ramps=ramps, wires=wires, hot=hot)

PALETTES = [
    P("Sunset",
      [("#160A3A", "#B5179E", "#FF7A3D"), ("#FF2E97", "#FF8A3D", "#FFE14D"), ("#2A0F5E", "#5A189A", "#B66DFF"),
       ("#0B0221", "#3C096C", "#FFD166"), ("#1B0038", "#7B2CBF", "#FF5CA8"), ("#0D0221", "#5A0F8C", "#FF4FD8"),
       ("#05101C", "#14746F", "#FFD6E8"), ("#0C0C14", "#C81D4A", "#FFF275")],
      {1: "#FFE14D", 2: "#FFB3DE", 3: "#B388FF", 4: "#00E5FF", 5: "#FF4FD8", 6: "#3CF2C0", 7: "#00D9F5",
       8: "#FFB000", 9: "#FF3B6B", 11: "#FFFFFF", 12: "#7FDBFF"}, "#FFFFFF"),
    P("Miami",
      [("#03045E", "#0096C7", "#FFAFCC"), ("#FF5D8F", "#FF87AB", "#FFF0C8"), ("#023E8A", "#0077B6", "#72EFDD"),
       ("#03071E", "#1B3A6B", "#FDFFB6"), ("#240046", "#C9184A", "#FFA3B5"), ("#001233", "#0353A4", "#64DFDF"),
       ("#012A2A", "#2A9D8F", "#FFFFFF"), ("#0B132B", "#F72585", "#FFE66D")],
      {1: "#FFF0A0", 2: "#FFFFFF", 3: "#48CAE4", 4: "#FF70A6", 5: "#FF9770", 6: "#70E000", 7: "#FF5D8F",
       8: "#FFD670", 9: "#00F5D4", 11: "#FFFFFF", 12: "#A0E7FF"}, "#FFFFFF"),
    P("Laser",
      [("#000000", "#0A0A3C", "#4C1DFF"), ("#B00020", "#FF2A00", "#FFC400"), ("#020230", "#1B1BD6", "#4CC9F0"),
       ("#000000", "#101040", "#FF2A6D"), ("#0A0014", "#3A0CA3", "#F72585"), ("#000000", "#14006B", "#FF0055"),
       ("#000814", "#00509D", "#C8E7FF"), ("#000000", "#2B2BFF", "#FFFFFF")],
      {1: "#FF3C00", 2: "#4CC9F0", 3: "#4361EE", 4: "#FF2A6D", 5: "#F72585", 6: "#05D9E8", 7: "#FF0055",
       8: "#FFFFFF", 9: "#05D9E8", 11: "#D1F7FF", 12: "#4361EE"}, "#FFFFFF"),
    P("Arcade",
      [("#000000", "#10002B", "#3C096C"), ("#FF0054", "#FF5400", "#FFBD00"), ("#001219", "#005F73", "#94D2BD"),
       ("#03071E", "#370617", "#FFBA08"), ("#10002B", "#9D0208", "#FAA307"), ("#000000", "#0B3D2E", "#39FF14"),
       ("#081C15", "#1B4332", "#F1FAEE"), ("#0D1B2A", "#FFD60A", "#FF006E")],
      {1: "#FFBD00", 2: "#F1FAEE", 3: "#0AFF99", 4: "#FF6D00", 5: "#FF0054", 6: "#39FF14", 7: "#39FF14",
       8: "#FFD60A", 9: "#00BBF9", 11: "#FFFFFF", 12: "#9B5DE5"}, "#FFFFFF"),
    P("Chrome",
      [("#0A1128", "#3D5A80", "#FFE8D6"), ("#FF8C42", "#FFD08A", "#FFFFFF"), ("#1C2541", "#5C7AA8", "#E0FBFC"),
       ("#0B132B", "#2F4A73", "#FFF3B0"), ("#101826", "#6E85A8", "#FFFFFF"), ("#05070F", "#273C5E", "#FFC27A"),
       ("#0D1B2A", "#415A77", "#FFFFFF"), ("#0A0A0A", "#B8C4D6", "#FF4D4D")],
      {1: "#FFD08A", 2: "#FFFFFF", 3: "#98C1D9", 4: "#EE6C4D", 5: "#E0FBFC", 6: "#98C1D9", 7: "#FFC27A",
       8: "#FFFFFF", 9: "#EE6C4D", 11: "#FFFFFF", 12: "#C0D8F0"}, "#FFFFFF"),
    P("Toxic",
      [("#0B001A", "#3D0066", "#9BFF00"), ("#00C853", "#B2FF59", "#FFFF8D"), ("#14002E", "#5E17A8", "#D17BFF"),
       ("#060012", "#2A0A4A", "#C6FF00"), ("#100020", "#6A00F4", "#F0A6FF"), ("#040010", "#1E5C00", "#AEFF00"),
       ("#001A0A", "#007F5F", "#E9FFDB"), ("#080808", "#8900F2", "#C6FF00")],
      {1: "#C6FF00", 2: "#E9FFDB", 3: "#BC00DD", 4: "#AEFF00", 5: "#E500A4", 6: "#00F5A0", 7: "#9BFF00",
       8: "#FFFF8D", 9: "#F20089", 11: "#FFFFFF", 12: "#B5FFE1"}, "#FFFFFF"),
    P("Ice",
      [("#020B1F", "#12508C", "#C8F4FF"), ("#7FD8FF", "#D4F4FF", "#FFFFFF"), ("#06263F", "#2A6F97", "#A9E5F7"),
       ("#03101F", "#0E3A5C", "#FFFFFF"), ("#041525", "#3A86B8", "#F2FCFF"), ("#010814", "#0B4F8A", "#7DF9FF"),
       ("#02202E", "#468FAF", "#FFFFFF"), ("#050B14", "#89C2D9", "#FF6B9D")],
      {1: "#E8FBFF", 2: "#FFFFFF", 3: "#61A5C2", 4: "#7DF9FF", 5: "#CAF0F8", 6: "#48CAE4", 7: "#00B4D8",
       8: "#FFFFFF", 9: "#FF6B9D", 11: "#FFFFFF", 12: "#90E0EF"}, "#FFFFFF"),
    P("Ember",
      [("#0A0000", "#6A040F", "#FFBA08"), ("#D00000", "#F48C06", "#FFE8A3"), ("#1A0303", "#7A0C12", "#F48C06"),
       ("#050000", "#370617", "#FFD166"), ("#120202", "#9D0208", "#FFB347"), ("#030000", "#5C0A0A", "#FF5400"),
       ("#0A0503", "#6F1D1B", "#FFE5B4"), ("#070707", "#E85D04", "#FFFFFF")],
      {1: "#FFBA08", 2: "#FFE5B4", 3: "#DC2F02", 4: "#FFBA08", 5: "#FF5400", 6: "#BB3E03", 7: "#FF2200",
       8: "#FFD166", 9: "#FFFFFF", 11: "#FFE8A3", 12: "#FF8C61"}, "#FFF4D6"),
]


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


def pal_rom():
    out = []
    for p in PALETTES:
        ent = [0] * 32
        for r, (k0, k1, k2) in enumerate(p["ramps"]):
            ent[2 * r] = pal_entry(k0, k1)
            ent[2 * r + 1] = pal_entry(k1, k2)
        for w in range(16):
            c = p["wires"].get(w, "#FFFFFF")
            ent[16 + w] = pal_entry(c, p["hot"])
        ent[26] = pal_entry(p["wires"][7], p["ramps"][5][2])       # the grid's wires running into the floor's glow
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


def bez(p0, p1, p2, s):
    a = (1 - s) * (1 - s); b = 2 * s * (1 - s); c = s * s
    return (a * p0[0] + b * p1[0] + c * p2[0], a * p0[1] + b * p1[1] + c * p2[1])


def hbits(lines, rows, step):
    """Per-ROM-row parity that flips at each virtual row in `lines`: a wire across the sprite's body there."""
    return [sum(1 for l in lines if l <= r * step) & 1 for r in range(rows)]


def art_palm():
    """256 x 384 virtual px; trunk = zone 2 (ringed), fronds = zone 1."""
    cv = canvas(256, 384)
    crown = (134.0, 120.0)
    fronds = [  # control, tip
        ((70, 70), (14, 150)), ((84, 40), (30, 52)), ((122, 44), (112, 8)),
        ((176, 36), (222, 46)), ((200, 66), (246, 148)),
    ]
    for ctrl, tip in fronds:
        for i in range(260):
            s = i / 259.0
            x, y = bez(crown, ctrl, tip, s)
            r = 2.0 + 11.0 * (math.sin(math.pi * min(1.0, s * 1.15)) ** 0.8) * (1.0 - 0.45 * s)
            disc(cv, x, y, r, r, 1)
    for y in range(118, 384):
        f = (383 - y) / 265.0
        xc = 116.0 + 18.0 * (f ** 1.6)
        hw = 9.0 - 3.5 * f
        for x in range(int(xc - hw), int(xc + hw) + 1):
            cv[y][x] = 2
    disc(cv, 134, 122, 12, 10, 1)
    return [cv[y] for y in range(0, 384, 2)], hbits(range(150, 384, 22), 192, 2)     # 192 rows


def art_car_rear():
    """Half a rear view, centre at x = 0: 256 x 192 virtual px.  A wedge supercar as a wire structure:
    1 body, 2 glass / tyres / plate / panel seams (thin slivers), 3 lights; creases from the row parity."""
    cv = canvas(256, 192)
    poly(cv, [(0, 14), (96, 14), (122, 24), (178, 74), (214, 84), (232, 104), (234, 150), (226, 160), (0, 160)], 1)
    # rear glass with louvres
    poly(cv, [(0, 24), (98, 24), (116, 32), (158, 72), (0, 72)], 2)
    for y in range(24, 72):
        if ((y - 24) // 8) % 2 == 1:
            for x in range(256):
                if cv[y][x] == 2:
                    cv[y][x] = 1
    # seams: the centre line, the deck's diagonal, the fender
    poly(cv, [(0, 74), (2, 74), (2, 98), (0, 98)], 2)
    poly(cv, [(146, 76), (154, 76), (214, 98), (206, 98)], 2)
    poly(cv, [(214, 104), (217, 104), (217, 150), (214, 150)], 2)
    # light bar: grille, frame, two lamps
    poly(cv, [(0, 98), (228, 98), (232, 124), (0, 124)], 1)
    poly(cv, [(0, 102), (44, 102), (44, 120), (0, 120)], 2)
    poly(cv, [(52, 102), (124, 102), (124, 120), (52, 120)], 3)
    poly(cv, [(132, 102), (206, 102), (210, 120), (132, 120)], 3)
    # bumper: centre seam, plate
    poly(cv, [(0, 124), (2, 124), (2, 160), (0, 160)], 2)
    poly(cv, [(0, 132), (40, 132), (40, 150), (0, 150)], 2)
    poly(cv, [(118, 126), (121, 126), (121, 160), (118, 160)], 2)                                           # bumper rib
    # the wing: a blade above the roof, an end plate, a strut down to the deck
    poly(cv, [(0, 3), (214, 3), (214, 11), (0, 11)], 1)
    poly(cv, [(204, 0), (216, 0), (216, 30), (208, 36), (204, 30)], 1)
    poly(cv, [(166, 11), (173, 11), (176, 80), (169, 80)], 1)
    # valance, exhaust, tyre
    poly(cv, [(0, 160), (148, 160), (154, 176), (0, 176)], 1)
    poly(cv, [(64, 164), (100, 164), (100, 173), (64, 173)], 2)
    poly(cv, [(148, 160), (228, 160), (228, 190), (154, 190)], 2)
    return [cv[y] for y in range(0, 192, 2)], hbits([82, 128, 146], 96, 2)            # 96 rows


def art_car_side():
    """Side view facing right: 512 x 160 virtual px, stored 256 x 80 (2 px a unit both ways)."""
    cv = canvas(512, 160)
    poly(cv, [(10, 126), (6, 98), (16, 74), (140, 62), (196, 26), (300, 22), (378, 64), (498, 92), (506, 112),
              (502, 126)], 1)                                                                               # shell
    for y in range(30, 62):                                                                                 # glass: the whole
        for x in range(512):                                                                                # cabin under the roof
            if cv[y][x] == 1:
                cv[y][x] = 2
    poly(cv, [(246, 30), (254, 30), (254, 62), (246, 62)], 1)                                               # B pillar
    poly(cv, [(2, 26), (72, 33), (74, 41), (24, 41), (20, 48), (4, 48)], 1)                                 # wing and end plate
    poly(cv, [(36, 41), (44, 41), (49, 72), (41, 72)], 1)                                                   # strut
    poly(cv, [(226, 64), (230, 64), (227, 90), (223, 90)], 2)                                               # door seams
    poly(cv, [(356, 64), (360, 64), (357, 90), (353, 90)], 2)
    poly(cv, [(6, 92), (20, 92), (20, 106), (6, 106)], 3)                                                   # tail light
    poly(cv, [(484, 94), (504, 100), (505, 108), (482, 104)], 3)                                            # head light
    disc(cv, 112, 124, 34, 34, 2)
    disc(cv, 398, 124, 34, 34, 2)
    return [cv[y][0:512:2] for y in range(0, 160, 2)], hbits([76, 98, 114], 80, 2)                             # 80 rows


def art_cloud():
    """256 x 48 units (2 virtual px a unit both ways): zone 1 top, zone 2 the lower shelf."""
    return _cloud(), [0] * 48


def _cloud():
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
    return cv


def art_birds(frame):
    return _birds(frame), [0] * 32


def _birds(frame):
    """256 x 32 virtual px: three gulls on a diagonal, wings up (0) or down (1)."""
    cv = canvas(256, 32)
    for (cx, cy) in [(60, 6), (132, 16), (204, 26)]:
        for i in range(-24, 25):
            a = abs(i)
            if frame == 0:
                yy = cy + 3 - a * 0.36 + (a * a) * 0.004
            else:
                yy = cy - 3 + a * 0.2
            for t in range(-2, 3):
                y = int(round(yy + t * 0.8))
                if 0 <= y < 32:
                    cv[y][cx + i] = 1
    return cv


def row_words(row, hb=0):
    """A 256-wide row of zones (+ the row's crease parity) -> 4 words."""
    segs = []                                       # [zone, start]
    for x, z in enumerate(row):
        if not segs or segs[-1][0] != z:
            segs.append([z, x])
    while len(segs) > 7:                            # too many pieces: drop the shortest inner one
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
LR_CIRCLE = 0                      # 256 half-chords, two a word (even index in the low byte)
LR_PALM, PALM_ROWS = 128, 192
LR_CAR, CAR_ROWS = 896, 96
LR_SIDE, SIDE_ROWS = 1280, 80
LR_CLOUD, CLOUD_ROWS = 1600, 48
LR_BIRD, BIRD_ROWS = 1792, 32      # frame 1 at + 128

SPRITES = [("palm", LR_PALM, art_palm), ("car", LR_CAR, art_car_rear), ("side", LR_SIDE, art_car_side),
           ("cloud", LR_CLOUD, art_cloud), ("bird0", LR_BIRD, lambda: art_birds(0)),
           ("bird1", LR_BIRD + 128, lambda: art_birds(1))]


def line_rom():
    rom = [0] * 2048
    ch = [min(255, int(round(math.sqrt(max(0.0, 256.0 * 256.0 - (i + 0.5) ** 2))))) for i in range(256)]
    for i in range(128):
        rom[i] = ch[2 * i] | (ch[2 * i + 1] << 8)
    for _, base, fn in SPRITES:
        rows, hb = fn()
        for r, row in enumerate(rows):
            rom[base + 4 * r: base + 4 * r + 4] = row_words(row, hb[r])
    return rom


# ---------------------------------------------------------------------------------------------------------
# Layer tables
# ---------------------------------------------------------------------------------------------------------
def knots(seed, lo, hi, valley):
    rnd = random.Random(seed)
    a = []
    prev = 3
    for i in range(64):
        v = rnd.randint(lo, hi)
        if abs(v - prev) < 2:
            v = min(hi, v + 2) if rnd.random() < 0.5 else max(lo, v - 2)
        a.append(v); prev = v
    for i, v in valley.items():
        a[i % 64] = v
    return a


def near_tab():
    a = knots(11, 1, 7, {-3: 6, -2: 4, -1: 1, 0: 0, 1: 1, 2: 3, 3: 6, 4: 4})
    return [a[c] | (a[(c + 1) % 64] << 3) for c in range(64)]


def far_tab():
    a = knots(23, 1, 7, {-1: 3, 0: 2, 1: 4})
    return [a[c] | (a[(c + 1) % 64] << 3) for c in range(64)]


def city_tab():
    rnd = random.Random(5)
    out = []
    for c in range(64):
        wide = 1 if rnd.random() < 0.35 else 0
        ha = rnd.randint(2, 12)
        hb = rnd.randint(2, 12)
        if c in (63, 0):
            ha = min(ha, 5); hb = min(hb, 4)
        sa = rnd.choice([3, 3, 3, 2, 2, 1, 1, 0])          # wire pattern: bit 1 floors, bit 0 bays
        sb = rnd.choice([3, 3, 3, 2, 2, 1, 1, 0])
        anta = 1 if (ha >= 8 and rnd.random() < 0.6) else 0
        antb = 1 if (hb >= 8 and rnd.random() < 0.6) else 0
        out.append(ha | (hb << 4) | (wide << 8) | (sa << 9) | (sb << 11) | (anta << 13) | (antb << 14))
    return out


def rain_tab():
    rnd = random.Random(9)
    return [(1 if rnd.random() < 0.42 else 0) << 4 | rnd.randint(0, 15) for _ in range(32)]


def win_tab():
    rnd = random.Random(3)
    return [1 if rnd.random() < 0.38 else 0 for _ in range(64)]


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
    parts = ["-- Generated by neonscape.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package neonscape_pkg is",
             vhd_table("C_NS_PAL", 256, 48, pal_rom()),
             vhd_table("C_NS_LROM", 2048, 16, line_rom()),
             vhd_table("C_NS_NEAR", 64, 6, near_tab()),
             vhd_table("C_NS_FAR", 64, 6, far_tab()),
             vhd_table("C_NS_CITY", 64, 16, city_tab()),
             vhd_table("C_NS_RAIN", 32, 5, rain_tab()),
             '    constant C_NS_WIN : std_logic_vector(0 to 63) := "%s";' % "".join(str(b) for b in win_tab()),
             "end package neonscape_pkg;", ""]
    with open(os.path.join(HERE, "neonscape_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


def preview(outdir):
    from PIL import Image
    os.makedirs(outdir, exist_ok=True)
    rom = line_rom()
    cols = [(10, 10, 20), (60, 220, 200), (255, 90, 170), (255, 230, 90)]
    tiles = []
    for name, base, fn in SPRITES:
        rows = len(fn()[0])
        gate = 2 if name == "palm" else 1
        z = [decode_row(rom[base + 4 * r: base + 4 * r + 4]) for r in range(rows)]
        hb = [(rom[base + 4 * r + 3] >> 14) & 1 for r in range(rows)]
        ident = [[(z[r][x], hb[r] if z[r][x] == gate else 0) for x in range(256)] for r in range(rows)]
        im = Image.new("RGB", (256, rows))
        for r in range(rows):
            for x in range(256):
                edge = (x > 0 and ident[r][x] != ident[r][x - 1]) or (r > 0 and ident[r][x] != ident[r - 1][x])
                im.putpixel((x, r), (255, 255, 255) if edge else tuple(c // 3 for c in cols[z[r][x]]))
        xsc = 4 if name == "side" else 2
        ysc = 2 if name.startswith("bird") else 4
        if name in ("side", "cloud"):
            xsc, ysc = 4, 4
        elif name.startswith("bird"):
            xsc, ysc = 2, 2
        else:
            xsc, ysc = 2, 4
        tiles.append(im.resize((256 * xsc, rows * ysc), Image.NEAREST))
    W = sum(t.size[0] for t in tiles[:3]) + 40
    sheet = Image.new("RGB", (W, 800 + 420), (40, 40, 40))
    x = 10
    for t in tiles[:3]:
        sheet.paste(t, (x, 10)); x += t.size[0] + 10
    x = 10
    for t in tiles[3:]:
        sheet.paste(t, (x, 800)); x += t.size[0] + 10
    sheet.save(os.path.join(outdir, "sprites.png"))


if __name__ == "__main__":
    write_pkg()
    if len(sys.argv) > 2 and sys.argv[1] == "--preview":
        preview(sys.argv[2])
