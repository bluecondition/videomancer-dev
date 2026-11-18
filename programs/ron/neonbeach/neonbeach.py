#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEON BEACH - build hook.  Writes neonbeach_pkg.vhd next to this file (pure python: the builder runs it with
the system python3).  `python3 neonbeach.py --preview DIR` also dumps the sprite art and the palettes as PNGs
(needs PIL; the build never uses it).

Package contents
  C_NB_PAL   256 x 48   colour table.  Address = theme (1) & time of day (2) & entry (5).
                        Word = Y0 U0 V0 dY dU dV, 8 bits each (10-bit value >> 2, hardware convention U = Cr,
                        V = Cb; d = signed, half the step to the next key).  colour = c0 + (d * t) >> 5, t 0..63.
                        Time of day 0 day, 1 low sun, 2 night:
                          entry 2 r + s   fill ramp r (0 .. 7), segment s (key s -> key s + 1)
                          entry 16 + w    wire colour of object w, lerped toward the palette's hot colour
                          entry 27        star, entry 28 spray
                        Time of day 3 = the sky keyframes: entries 0 .. 7 the zenith, 8 .. 15 the horizon,
                        keyframe i lerping to i + 1 (the sky colour follows the sun's elevation smoothly).
  C_NB_LROM  1024 x 16  per-line tables: the circle's half-chord, then the span-coded sprite rows
                        (4 words a row: b1 b0 / b3 b2 / b5 b4 / zones z0 .. z6, 2 bits each, z0 in the LSBs,
                        bit 14 = the row's crease parity; zone = z[number of boundaries <= x])
  C_NB_BED   128 x 10   sea bed / beach row at each knot (16 virtual px apart from the canvas's left edge)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------
# Palettes.  Ramps (3 keys each, t = 0 .. 127 runs key0 -> key1 -> key2):
#   0 sky (lit by the camera with the fill off)   4 wash      dark .. bright
#   1 sun / moon  bottom .. top                    5 sand      deep .. surface
#   2 water       deep .. surface                  6 flora     clouds, gulls: dark .. bright
#   3 foam        dark .. bright                   7 crab      dark .. body .. highlight
# Wires (by object): 1 sun, 2 cloud / gull, 5 water, 6 foam, 7 wash, 8 sand, 9 crab, 11 star, 12 spray.
# ---------------------------------------------------------------------------------------------------------
def T(ramps, wires, hot="#FFFFFF"):
    return dict(ramps=ramps, wires=wires, hot=hot)

THEMES = [
    dict(name="Neon",
         tod=[
             T([("#0A1A4A", "#1E5AB4", "#7FD4FF"), ("#FF6A2A", "#FFC23D", "#FFF6B0"),
                ("#04123A", "#0B5FA8", "#2EE6E0"), ("#4FB8E8", "#C8F8FF", "#FFFFFF"),
                ("#0E6F9A", "#38C8E8", "#B8FFFF"), ("#5A3A1A", "#C8964A", "#FFE0A0"),
                ("#2A2A3A", "#9AA8C8", "#FFFFFF"), ("#3A0008", "#E8203C", "#FF9A6A")],
               {1: "#FFE14D", 2: "#FFFFFF", 5: "#00E5FF", 6: "#FFFFFF", 7: "#7FFFF4", 8: "#FFB000",
                9: "#FF3B6B", 11: "#FFFFFF", 12: "#E0FFFF"}),
             T([("#1A0A3A", "#8A1F8A", "#FF7A3D"), ("#C8102E", "#FF4A1A", "#FFB04D"),
                ("#12052E", "#5A1A7A", "#FF5CA8"), ("#C85AA0", "#FFD0E8", "#FFFFFF"),
                ("#5A1A7A", "#C83CA8", "#FFC0E0"), ("#3A1A1A", "#A8583A", "#FFB070"),
                ("#1A0A1A", "#6A3A5A", "#FFD0B0"), ("#2A0004", "#C81028", "#FF8040")],
               {1: "#FFB040", 2: "#FFC8E0", 5: "#FF4FD8", 6: "#FFFFFF", 7: "#FF9AE0", 8: "#FF7A00",
                9: "#FF2A4A", 11: "#FFFFFF", 12: "#FFE0F0"}),
             T([("#000008", "#0A0A30", "#2A3A7A"), ("#5A6A8A", "#B8C8E0", "#F0F8FF"),
                ("#000418", "#06204A", "#1A6AA8"), ("#2A5A88", "#A8D0F0", "#E8F8FF"),
                ("#062A4A", "#1A6A9A", "#7AC8F0"), ("#10101A", "#3A3A50", "#7A7A98"),
                ("#08080F", "#3A4058", "#A8B0D0"), ("#10000A", "#7A1030", "#C84060")],
               {1: "#E0F0FF", 2: "#A8B8E0", 5: "#2A9AFF", 6: "#C8E8FF", 7: "#5AC8FF", 8: "#8A8AC8",
                9: "#C82A6A", 11: "#FFFFFF", 12: "#A8D8FF"}, "#E8F0FF"),
         ],
         zenith=["#000006", "#0A0628", "#2A0A4A", "#4A1060", "#2A2A88", "#1E4AB0", "#2E78D8", "#3A9AF0"],
         horizon=["#06102A", "#3A1050", "#C8287A", "#FF6A3A", "#FFB060", "#FFE0A0", "#BFF0FF", "#9AE4FF"]),
    dict(name="Ice",
         tod=[
             T([("#020B1F", "#12508C", "#C8F4FF"), ("#7FD8FF", "#D4F4FF", "#FFFFFF"),
                ("#010814", "#0B4F8A", "#7DF9FF"), ("#48CAE4", "#CAF0F8", "#FFFFFF"),
                ("#06263F", "#2A6F97", "#A9E5F7"), ("#1A1630", "#6A6A9A", "#D8D8F0"),
                ("#02202E", "#468FAF", "#FFFFFF"), ("#1A0A2A", "#B02A8A", "#FF8AD0")],
               {1: "#E8FBFF", 2: "#FFFFFF", 5: "#00B4D8", 6: "#FFFFFF", 7: "#90E0EF", 8: "#B8A8FF",
                9: "#FF6B9D", 11: "#FFFFFF", 12: "#E0FFFF"}),
             T([("#0A0628", "#4A2A8A", "#FF9AC8"), ("#FF5A8A", "#FFA8C8", "#FFF0F8"),
                ("#0A0420", "#3A1A7A", "#B87AFF"), ("#9A7AE8", "#E0D0FF", "#FFFFFF"),
                ("#1A0A4A", "#6A3AB8", "#E0C0FF"), ("#1A0A2A", "#6A3A7A", "#E8A8D8"),
                ("#0A0420", "#5A4A8A", "#FFE0F0"), ("#20001A", "#C81060", "#FF9AB0")],
               {1: "#FFA8D0", 2: "#FFE0F0", 5: "#B87AFF", 6: "#FFFFFF", 7: "#E0A8FF", 8: "#FF8AC8",
                9: "#FF3A80", 11: "#FFFFFF", 12: "#F0E0FF"}),
             T([("#000006", "#020A28", "#0A2A5A"), ("#6A7A9A", "#C8D8F0", "#FFFFFF"),
                ("#000210", "#02183A", "#0A5A8A"), ("#1A4A7A", "#90C0E8", "#E8F8FF"),
                ("#021A3A", "#0A4A7A", "#5AB0E0"), ("#08081A", "#2A2A4A", "#6A6A98"),
                ("#04040F", "#2A3A5A", "#90A8C8"), ("#0A0010", "#5A1040", "#A84078")],
               {1: "#F0F8FF", 2: "#90A8D0", 5: "#1A8AE8", 6: "#C8E8FF", 7: "#4AB0F0", 8: "#7A7AC0",
                9: "#B02A70", 11: "#FFFFFF", 12: "#A8D8FF"}, "#E8F0FF"),
         ],
         zenith=["#000004", "#020420", "#0A0A3A", "#1A1060", "#0A2A70", "#0A4A90", "#1A6AB8", "#2A88D8"],
         horizon=["#020A20", "#0A1A4A", "#5A2A8A", "#C85AA8", "#FF9AC8", "#E8D0FF", "#C0F4FF", "#A8ECFF"]),
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
    for th in THEMES:
        for p in th["tod"]:
            ent = [0] * 32
            for r, (k0, k1, k2) in enumerate(p["ramps"]):
                ent[2 * r] = pal_entry(k0, k1)
                ent[2 * r + 1] = pal_entry(k1, k2)
            for w in range(16):
                c = p["wires"].get(w, "#FFFFFF")
                ent[16 + w] = pal_entry(c, p["hot"])
            out += ent
        ent = [0] * 32
        for base, keys in ((0, th["zenith"]), (8, th["horizon"])):
            for i in range(8):
                ent[base + i] = pal_entry(keys[i], keys[min(7, i + 1)])
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


def art_birds(frame):
    """256 x 32 units (2 virtual px a unit both ways, so 512 x 64 on screen): three gulls, wings up / down.
    Zone 1 the wings, zone 2 the body."""
    cv = canvas(256, 32)
    for (cx, cy) in [(60, 8), (140, 18), (206, 12)]:
        for i in range(-26, 27):
            a = abs(i)
            if frame == 0:
                yy = cy + 3 - a * 0.40 + (a * a) * 0.0045
            else:
                yy = cy - 2 + a * 0.22
            for t in range(-2, 3):
                y = int(round(yy + t * 0.7))
                if 0 <= y < 32:
                    cv[y][cx + i] = 1
        disc(cv, cx, cy + 3, 5, 3, 2)
    return cv, [0] * 32


def art_crab(frame):
    """Half a crab, the centre line at x = 0 (the engine folds |x|): 128 x 96 virtual px, so the crab is
    256 x 96 on screen, stored 48 rows (2 rows a unit).  Zone 1 the shell, 2 eyes / joints / mouth, 3 claws
    and legs; the creases come from the row parity.  Frame 1 lifts the other legs."""
    cv = canvas(256, 96)
    lift = [0, 7, 0] if frame == 0 else [7, 0, 7]
    for i in range(3):
        l = lift[i]
        root = (44 + 8 * i, 66)
        knee = (78 + 13 * i, 58 - l)
        foot = (90 + 13 * i, 93 - l)
        thick(cv, root, knee, 3.0, 3)
        thick(cv, knee, foot, 2.4, 3)
    thick(cv, (54, 48), (82, 34), 3.6, 3)                                                        # arm
    poly(cv, [(78, 34), (94, 10), (114, 7), (122, 20), (104, 25), (110, 36), (90, 44)], 3)      # claw
    disc(cv, 82, 34, 5, 5, 2)                                                                    # elbow
    thick(cv, (12, 46), (20, 22), 2.4, 1)                                                        # eye stalk
    disc(cv, 20, 19, 6, 6, 2)                                                                    # eye
    disc(cv, 0, 58, 66, 26, 1)                                                                   # shell
    poly(cv, [(0, 58), (64, 58), (52, 76), (0, 76)], 1)
    poly(cv, [(0, 70), (14, 70), (11, 77), (0, 77)], 2)                                          # mouth
    rows = [cv[y] for y in range(0, 96, 2)]
    return rows, hbits([44, 58, 70], 48, 2)


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
#   C_NB_DTAB  16 x 22   kstep (14: phase per knot, 65536 = one wave) & shoal (5: amplitude gain, eighths)
#                        & class (3: wave shape, 0 = round .. 7 = a steep breaking front)
#   C_NB_SIZE  16 x 4    each wave's size in eighths (4 .. 11), by its number: waves come in sets
#   C_NB_SHAPE 1024 x 8  wave profile by class (3) & phase (7): signed, 64 = the amplitude.  Phase 0 is the
#                        crest; the front (the shoreward, left side) is the end of the phase.
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
    out = []
    for c in range(8):
        F = 0.5 - 0.42 * c / 7.0                 # the front's share of the wave
        q = 1.0 + 1.3 * c / 7.0                  # peaked crests, flat troughs
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
             vhd_table("C_NB_DTAB", 16, 22, [(ks << 8) | (sh << 3) | cl for ks, sh, cl in dtab()]),
             vhd_table("C_NB_SIZE", 16, 4, SIZES),
             vhd_table("C_NB_SHAPE", 1024, 8, [v & 255 for v in shape_tab()]),
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
