#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEONSCAPE v0.1.0 - build hook.  Writes neonscape_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  `python3 neonscape.py --preview DIR` also dumps the sprite art and the
palettes as PNGs (needs PIL; the build never uses it).

Package contents
  C_NS_PAL   256 x 48   colour table, 8 palettes x 32 entries.  Word = Y0 U0 V0 dY dU dV, 8 bits each
                        (Y0.. = 10-bit value >> 2, hardware convention U = Cr, V = Cb; d = signed, half the
                        step to the next key).  colour = c0 + (d * t) >> 5, t = 0 .. 63.
                          entry 2 r + s   fill ramp r (0 .. 7), segment s (key s -> key s + 1)
                          entry 16 + w    wire colour of object w, lerped toward the palette's hot colour
  C_NS_LROM  2048 x 16  per-line tables: the circle's half-chord, then the span-coded sprite rows
                        (4 words a row: b1 b0 / b3 b2 / b5 b4 / zones z0 .. z6, 2 bits each, z0 in the LSBs;
                        zone = z[number of boundaries <= x])
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


def art_palm():
    """256 x 384 virtual px; trunk = zone 2, fronds = zone 1."""
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
    return [cv[y] for y in range(0, 384, 2)]          # 192 rows


def art_car_rear():
    """Half a rear view, centre at x = 0: 256 x 192 virtual px.  1 body, 2 glass / tyres / plate, 3 lights."""
    cv = canvas(256, 192)
    poly(cv, [(0, 16), (104, 16), (128, 24), (186, 78), (224, 92), (232, 112), (232, 160), (0, 160)], 1)   # shell
    poly(cv, [(0, 26), (100, 26), (118, 34), (160, 74), (0, 74)], 2)                                       # rear glass
    poly(cv, [(0, 100), (212, 100), (216, 124), (0, 124)], 3)                                               # light bar
    poly(cv, [(0, 134), (44, 134), (44, 152), (0, 152)], 2)                                                 # plate
    poly(cv, [(140, 160), (226, 160), (226, 190), (146, 190)], 2)                                           # tyre
    poly(cv, [(0, 160), (140, 160), (146, 176), (0, 176)], 1)                                               # valance
    poly(cv, [(70, 164), (104, 164), (104, 174), (70, 174)], 3)                                             # exhaust
    return [cv[y] for y in range(0, 192, 2)]          # 96 rows


def art_car_side():
    """Side view facing right, 256 x 96 at 2 virtual px a unit across and 2 a row (512 x 96 on screen... rows / 2)."""
    cv = canvas(256, 96)
    poly(cv, [(6, 58), (10, 42), (58, 36), (92, 14), (150, 12), (188, 34), (236, 42), (250, 54), (250, 70),
              (6, 70)], 1)                                                                                  # shell
    poly(cv, [(70, 36), (98, 19), (124, 18), (124, 36)], 2)                                                 # rear window
    poly(cv, [(130, 18), (148, 17), (178, 36), (130, 36)], 2)                                               # front window
    poly(cv, [(6, 46), (22, 44), (22, 54), (6, 56)], 3)                                                     # tail light
    poly(cv, [(238, 46), (250, 54), (250, 60), (236, 58)], 3)                                               # head light
    disc(cv, 58, 70, 21, 21, 2)
    disc(cv, 196, 70, 21, 21, 2)
    return [cv[y] for y in range(0, 96, 2)]           # 48 rows


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
    return cv


def art_birds(frame):
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


def row_words(row):
    """A 256-wide row of zones -> 4 words."""
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
    return [bounds[0] | (bounds[1] << 8), bounds[2] | (bounds[3] << 8), bounds[4] | (bounds[5] << 8), zw]


def decode_row(words):
    b = [words[0] & 255, words[0] >> 8, words[1] & 255, words[1] >> 8, words[2] & 255, words[2] >> 8]
    out = []
    for x in range(256):
        k = sum(1 for v in b if x >= v)
        out.append((words[3] >> (2 * k)) & 3)
    return out


# line ROM layout
LR_CIRCLE = 0
LR_PALM, PALM_ROWS = 256, 192
LR_CAR, CAR_ROWS = 1024, 96
LR_SIDE, SIDE_ROWS = 1408, 48
LR_CLOUD, CLOUD_ROWS = 1600, 48
LR_BIRD, BIRD_ROWS = 1792, 32      # frame 1 at + 128

SPRITES = [("palm", LR_PALM, art_palm), ("car", LR_CAR, art_car_rear), ("side", LR_SIDE, art_car_side),
           ("cloud", LR_CLOUD, art_cloud), ("bird0", LR_BIRD, lambda: art_birds(0)),
           ("bird1", LR_BIRD + 128, lambda: art_birds(1))]


def line_rom():
    rom = [0] * 2048
    for i in range(256):
        rom[i] = min(255, int(round(math.sqrt(max(0.0, 256.0 * 256.0 - (i + 0.5) ** 2)))))
    for _, base, fn in SPRITES:
        for r, row in enumerate(fn()):
            rom[base + 4 * r: base + 4 * r + 4] = row_words(row)
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
        rows = len(fn())
        im = Image.new("RGB", (256, rows))
        for r in range(rows):
            z = decode_row(rom[base + 4 * r: base + 4 * r + 4])
            for x in range(256):
                im.putpixel((x, r), cols[z[x]])
        ysc = 1 if name.startswith("bird") else 2
        tiles.append(im.resize((256 * 2, rows * ysc * 2), Image.NEAREST))
    W = sum(t.size[0] for t in tiles) + 10 * (len(tiles) + 1)
    H = max(t.size[1] for t in tiles) + 20
    sheet = Image.new("RGB", (W, H), (40, 40, 40))
    x = 10
    for t in tiles:
        sheet.paste(t, (x, 10)); x += t.size[0] + 10
    sheet.save(os.path.join(outdir, "sprites.png"))


if __name__ == "__main__":
    write_pkg()
    if len(sys.argv) > 2 and sys.argv[1] == "--preview":
        preview(sys.argv[2])
