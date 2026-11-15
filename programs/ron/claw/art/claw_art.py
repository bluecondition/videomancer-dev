#!/usr/bin/env python3
"""CLAW art pipeline: prize sprites, font, palette -> claw_pkg.vhd + sheets.

Prizes are authored as vector parts on an 8x canvas (384 x 384), each part
auto-shaded (top-left highlight band, bottom-right shadow band, part outline),
then box-downsampled to 48 x 48 by majority vote and given a closed 1-px
silhouette outline.  Each prize maps its colours onto 4-bit nibbles
(0 = transparent, 1 = outline) through a per-type palette of master colours.

Supplied art (art/supplied/<name>.png) replaces the fallback of the same
prize: it is box-fitted to 48x48, quantised to the master palette (nearest
colour, alpha < 128 = transparent) and outlined the same way.

Usage: python3 claw_art.py            (writes ../claw_pkg.vhd, ../preview/*.png)
"""
import os
import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import claw_layout as L  # noqa: E402

S = 8                      # authoring supersample
N = L.SPRITE_W             # 48
HI = N * S
M = L.M


# --------------------------------------------------------------------------
# Mask helpers (coordinates in sprite pixels, floats ok)
# --------------------------------------------------------------------------
def _img():
    return Image.new("L", (HI, HI), 0)


def _mask(img):
    return np.array(img) > 127


def rr(x0, y0, x1, y1, r):
    im = _img()
    ImageDraw.Draw(im).rounded_rectangle(
        [x0 * S, y0 * S, x1 * S - 1, y1 * S - 1], radius=max(0, r * S), fill=255)
    return _mask(im)


def ell(cx, cy, rx, ry=None):
    ry = rx if ry is None else ry
    im = _img()
    ImageDraw.Draw(im).ellipse(
        [(cx - rx) * S, (cy - ry) * S, (cx + rx) * S - 1, (cy + ry) * S - 1], fill=255)
    return _mask(im)


def poly(pts):
    im = _img()
    ImageDraw.Draw(im).polygon([(x * S, y * S) for x, y in pts], fill=255)
    return _mask(im)


def line(pts, w):
    im = _img()
    d = ImageDraw.Draw(im)
    d.line([(x * S, y * S) for x, y in pts], fill=255, width=int(w * S))
    for x, y in (pts[0], pts[-1]):
        d.ellipse([x * S - w * S / 2, y * S - w * S / 2, x * S + w * S / 2, y * S + w * S / 2], fill=255)
    return _mask(im)


def shift(m, dx, dy):
    out = np.zeros_like(m)
    h, w = m.shape
    ys = slice(max(0, dy), h + min(0, dy))
    yd = slice(max(0, -dy), h + min(0, -dy))
    xs = slice(max(0, dx), w + min(0, dx))
    xd = slice(max(0, -dx), w + min(0, -dx))
    out[ys, xs] = m[yd, xd]
    return out


def erode(m, r):
    out = m.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out &= shift(m, dx, dy)
    return out


class Canvas:
    def __init__(self):
        self.idx = np.zeros((HI, HI), dtype=np.int32)

    def part(self, mask, ramp, band=2.0, outline=True, ow=1.0):
        """ramp = (shadow, base, highlight) master names; band in sprite px."""
        sh, base, hl = (M[c] for c in ramp)
        b = max(1, int(band * S))
        self.idx[mask] = base
        self.idx[mask & ~shift(mask, b, b)] = hl
        self.idx[mask & ~shift(mask, -b, -b) & (self.idx != hl)] = sh
        if outline:
            edge = mask & ~erode(mask, max(1, int(ow * S)))
            self.idx[edge] = M["outline"]

    def flat(self, mask, colour):
        self.idx[mask] = M[colour]

    def hole(self, mask):
        self.idx[mask] = 0

    def downsample(self):
        blk = self.idx.reshape(N, S, N, S).transpose(0, 2, 1, 3).reshape(N, N, S * S)
        out = np.zeros((N, N), dtype=np.int32)
        ol = M["outline"]
        for y in range(N):
            for x in range(N):
                v = blk[y, x]
                cnt = np.bincount(v, minlength=48)
                if cnt[ol] >= 20:
                    out[y, x] = ol
                else:
                    out[y, x] = int(np.argmax(cnt))
        return close_outline(out)


def close_outline(a):
    """Every transparent pixel 4-adjacent to a coloured pixel becomes outline."""
    ol = M["outline"]
    op = (a != 0) & (a != ol)
    n = shift(op, 1, 0) | shift(op, -1, 0) | shift(op, 0, 1) | shift(op, 0, -1)
    a = a.copy()
    a[(a == 0) & n] = ol
    return a


# --------------------------------------------------------------------------
# Prize designs (fallback art).  Order = sprite type index.
# --------------------------------------------------------------------------
def prize_tv():
    c = Canvas()
    c.part(line([(24, 13), (13, 3)], 1.6), ("chrome_sh", "chrome", "chrome_hl"), band=0.6)
    c.part(line([(24, 13), (36, 4)], 1.6), ("chrome_sh", "chrome", "chrome_hl"), band=0.6)
    c.part(ell(13, 3, 2.2), ("red_sh", "red", "red_hl"), band=0.8)
    c.part(ell(36, 4, 2.2), ("red_sh", "red", "red_hl"), band=0.8)
    c.part(rr(9, 39, 15, 46, 1.5), ("black", "black", "black_hl"), band=0.8)
    c.part(rr(33, 39, 39, 46, 1.5), ("black", "black", "black_hl"), band=0.8)
    c.part(rr(4, 12, 44, 42, 6), ("red_sh", "red", "red_hl"), band=2.0)
    c.part(rr(7, 15, 33, 39, 4.5), ("black", "black", "black_hl"), band=1.2)
    scr = rr(9.5, 17.5, 30.5, 36.5, 3)
    bars = ["white", "yellow", "cyan", "green", "magenta", "red", "blue"]
    for i, col in enumerate(bars):
        x0 = 9.5 + i * 3
        c.flat(scr & rr(x0, 0, x0 + 3.05, 31, 0), col)
    c.flat(scr & rr(0, 31, 48, 40, 0), "blue")
    c.flat(scr & rr(16, 31, 23, 40, 0), "black")
    c.flat(scr & poly([(11, 19), (16, 19), (11, 25)]), "white")
    for ky in (21, 28):
        c.part(ell(38.5, ky, 2.6), ("chrome_sh", "chrome", "chrome_hl"), band=0.9)
    for gy in (34, 36.5):
        c.flat(rr(35, gy, 42, gy + 1.2, 0.5), "outline")
    return c


def prize_vhs():
    """A VHS cassette seen from the top: wide black shell, white label with
    handwriting, a big smoky window showing a full and a nearly empty reel
    (tape packs cut off by the window, white spoked hubs), ribbed grips at
    the ends, and the dust door along the front with a sliver of tape."""
    c = Canvas()
    body = rr(1, 11, 47, 37.5, 2.5)
    c.part(body, ("black", "black", "black_hl"), band=1.2)
    # dust door along the front edge, tape showing through its gap
    door = rr(3, 33.6, 45, 37.5, 1.2) & body
    c.part(door, ("outline", "black", "black_hl"), band=0.6, outline=False)
    c.flat(rr(8, 34.6, 40, 35.5, 0), "brown_sh")
    # label: cream sticker, a thin red rule, two lines of grey "handwriting"
    c.part(rr(6, 12.8, 42, 20.6, 1.2), ("cream", "cream", "white"), band=0.7)
    c.flat(rr(7, 13.9, 41, 14.7, 0), "red")
    c.flat(line([(9, 16.6), (12, 16.0), (15, 16.8), (18, 16.1), (21, 16.7), (24, 16.0), (27, 16.6)], 0.9), "grey")
    c.flat(line([(9, 18.8), (12, 18.2), (15, 18.9), (18, 18.3)], 0.9), "grey")
    # ribbed grips at both ends
    for gx in (3.3, 5.3, 42.7, 44.7):
        c.flat(rr(gx - 0.45, 23, gx + 0.45, 32, 0.4), "black_hl")
    # the window: smoky glass with the two reels behind it
    win = rr(9.5, 22, 38.5, 33, 2)
    c.part(win, ("black_hl", "grey", "grey"), band=0.8)
    full = ell(17.2, 27.5, 7.4) & win          # full reel: tape pack cut off
    empty = ell(31.2, 27.5, 4.7) & win         # nearly empty reel
    for pack in (full, empty):
        c.part(pack, ("brown_sh", "brown_sh", "brown"), band=1.0, outline=False)
    for hx, a0 in ((17.2, 90), (31.2, 40)):
        # hub: a chrome wheel with three dark spokes, the two reels turned
        # differently (matching white hubs with dark centres read as eyes)
        c.flat(ell(hx, 27.5, 3.4), "chrome")
        c.flat(ell(hx - 0.9, 26.6, 1.3) & ell(hx, 27.5, 3.4), "chrome_hl")
        import math
        for k in range(3):
            ang = math.radians(a0 + 120 * k)
            tx = hx + 3.4 * math.cos(ang)
            ty = 27.5 - 3.4 * math.sin(ang)
            c.flat(line([(hx, 27.5), (tx, ty)], 0.7), "black")
    # screws
    for sx in (3.2, 44.8):
        c.flat(ell(sx, 13.2, 0.8), "grey")
    return c


def prize_dvd():
    c = Canvas()
    disc = ell(24, 24, 21.5)
    c.part(disc, ("chrome_sh", "chrome", "chrome_hl"), band=2.5)
    yy, xx = np.mgrid[0:HI, 0:HI]
    ang = (np.degrees(np.arctan2(-(yy - 24 * S), xx - 24 * S)) + 360) % 360
    rad = np.hypot(yy - 24 * S, xx - 24 * S) / S
    ring = disc & (rad > 9.5) & (rad < 19.5)
    for lo, hi, col in [(20, 38, "cyan_hl"), (38, 52, "purple_hl"), (52, 64, "pink_hl"),
                        (64, 74, "yellow_hl"), (200, 216, "yellow_hl"),
                        (216, 232, "green_hl"), (232, 248, "cyan_hl"), (248, 260, "purple_hl")]:
        c.flat(ring & (ang >= lo) & (ang < hi), col)
    c.flat(disc & (rad > 19.6) & (rad < 20.6) & (ang > 100) & (ang < 170), "white")
    c.part(ell(24, 24, 8), ("chrome", "chrome_hl", "white"), band=1.0, ow=0.7)
    c.part(ell(24, 24, 5.2), ("chrome_sh", "chrome_sh", "chrome"), band=0.8, ow=0.6)
    c.hole(ell(24, 24, 3.0))
    return c


def prize_spark():
    c = Canvas()
    bolt = poly([(27, 1.5), (7, 27.5), (21, 27.5), (13.5, 47), (41, 17.5), (27.5, 17.5), (37, 1.5)])
    c.part(bolt, ("yellow_sh", "yellow", "yellow_hl"), band=2.2)
    for ex in (20.5, 29):
        c.flat(ell(ex, 15, 2.1, 2.6) & bolt, "outline")
        c.flat(ell(ex - 0.7, 14, 0.8) & bolt, "white")
    c.flat(ell(16.8, 19.5, 2.0, 1.2) & bolt, "blush")
    c.flat(ell(31.5, 19.5, 2.0, 1.2) & bolt, "blush")
    smile = ell(24.7, 18.0, 3.0, 2.6) & ~ell(24.7, 16.6, 3.2, 2.4)
    c.flat(smile & bolt, "outline")
    c.flat(bolt & poly([(31, 3), (35.5, 3), (26, 15), (23, 15)]), "white")
    return c


def prize_bars():
    c = Canvas()
    pillow = rr(3, 9, 45, 41, 8.5)
    c.part(pillow, ("black", "white", "white"), band=1.5)
    cols = ["white", "yellow", "cyan", "green", "magenta", "red", "blue"]
    hl = ["white", "yellow_hl", "cyan_hl", "green_hl", "pink_hl", "red_hl", "blue_hl"]
    top = pillow & ~shift(pillow, 0, int(2.6 * S))
    inner = pillow & erode(pillow, S)
    for i, col in enumerate(cols):
        x0 = 3 + i * 6
        band = inner & rr(x0, 0, x0 + 6.02, 32, 0)
        c.flat(band, col)
        c.flat(band & top, hl[i])
    strip = inner & rr(0, 32, 48, 48, 0)
    for x0, x1, col in [(3, 12, "blue"), (12, 21, "black"), (21, 30, "white"),
                        (30, 39, "black"), (39, 45, "blue")]:
        c.flat(strip & rr(x0, 0, x1, 48, 0), col)
    for ex in (17, 26):
        c.flat(ell(ex, 21, 1.6, 2.1), "outline")
        c.flat(ell(ex - 0.5, 20.3, 0.6), "white")
    smile = ell(21.5, 24.2, 2.6, 2.0) & ~ell(21.5, 23.0, 2.8, 1.9)
    c.flat(smile, "outline")
    return c


def prize_synth():
    c = Canvas()
    c.part(rr(3, 10, 45, 41, 3.5), ("black", "black", "black_hl"), band=1.6)
    c.part(rr(3, 10, 45, 14.5, 3.5) & rr(3, 10, 45, 41, 3.5), ("gold_sh", "gold", "gold_hl"), band=0.7, outline=False)
    c.part(rr(6, 16, 22, 28, 2), ("black_hl", "black_hl", "grey"), band=0.6)
    scr = rr(7.5, 17.5, 20.5, 26.5, 1.3)
    c.flat(scr, "screen")
    yy, xx = np.mgrid[0:HI, 0:HI]
    t = np.linspace(0, 2 * np.pi, 400)
    lx = 14 + 5.2 * np.sin(3 * t + 0.6)
    ly = 22 + 3.6 * np.sin(2 * t)
    liss = np.zeros((HI, HI), bool)
    for x, y in zip(lx, ly):
        liss |= (np.abs(xx - x * S) < 5) & (np.abs(yy - y * S) < 5)
    c.flat(liss & scr, "cyan_hl")
    caps = [("red_sh", "red", "red_hl"), ("gold_sh", "gold", "gold_hl"),
            ("cyan_sh", "cyan", "cyan_hl"), ("gold_sh", "gold", "gold_hl"),
            ("red_sh", "red", "red_hl"), ("cyan_sh", "cyan", "cyan_hl")]
    k = 0
    for ky in (18.5, 25.5):
        for kx in (27.5, 34, 40.5):
            c.part(ell(kx, ky, 2.7), caps[k], band=0.9, ow=0.7)
            k += 1
    c.flat(rr(6.5, 32, 41.5, 34, 1), "outline")
    c.part(rr(25, 29, 31, 37, 1.5), ("chrome_sh", "chrome", "chrome_hl"), band=0.8, ow=0.7)
    for jx in (8, 12.5, 17):
        c.flat(ell(jx, 37.8, 1.3), "outline")
        c.flat(ell(jx, 37.8, 0.5), "grey")
    return c


PRIZES = [
    # name (for banners), supplied key, fallback art
    ("CRT TV", "tv", prize_tv),
    ("VHS TAPE", "vhs", prize_vhs),
    ("DVD", "dvd", prize_dvd),
    ("LZX TOKEN", "lzx_token", None),
    ("EYEBALL", "eyeball", None),
    ("SYNTH", "synth", prize_synth),
    ("RED HAT", "redhat", None),
    ("WIZARD", "wizard", None),
]
# unused fallbacks kept for reference: prize_spark, prize_bars

# Master palette as built: the named colours plus supplied-art colours
# allocated into the spare slots (exact colours, so the art isn't repainted).
MASTER_BUILT = [list(e) for e in L.MASTER]
# colours the VHDL names directly (PA_*) stay put
RESERVED = {"outline", "chrome_sh", "chrome", "chrome_hl", "white", "black", "black_hl",
            "red_sh", "red", "red_hl", "bulb_on", "bulb_off"}
FREE = []              # filled by build_sprites: slots no remaining sprite uses
_next_spare = [0]      # count of slots handed out


def alloc_colour(rgb, tol=14.0):
    """Index of a master colour within tol of rgb, else a new spare slot."""
    best, bd = 0, 1e9
    for i in range(1, L.NMASTER):
        if i in FREE:
            continue
        d = sum((a - b) ** 2 for a, b in zip(MASTER_BUILT[i][1], rgb)) ** 0.5
        if d < bd:
            best, bd = i, d
    if bd <= tol:
        return best
    if not FREE:
        print("  master palette full; %s -> nearest (d=%.0f)" % (rgb, bd))
        return best
    i = FREE.pop(0)
    MASTER_BUILT[i] = ["sup_%02x%02x%02x" % tuple(rgb), tuple(int(c) for c in rgb)]
    _next_spare[0] += 1
    return i


def supplied_sprite(path):
    """48x48 RGBA pixel art used as-is: darkest colours become the outline,
    the rest keep their exact colour (merged down to 14 if needed)."""
    im = Image.open(path).convert("RGBA")
    if im.size != (N, N):
        k = max(1, max(im.size) // N)
        im = im.resize((im.width // k, im.height // k), Image.NEAREST)
        canvas = Image.new("RGBA", (N, N), (0, 0, 0, 0))
        canvas.paste(im, ((N - im.width) // 2, (N - im.height) // 2))
        im = canvas
    a = np.array(im)
    op = a[..., 3] >= 128
    cols, cnt = np.unique(a[op][:, :3], axis=0, return_counts=True)
    cols = [tuple(int(v) for v in c) for c in cols]
    luma = lambda c: 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]
    remap = {c: c for c in cols}
    body = {c: n for c, n in zip(cols, cnt) if luma(c) >= 40}
    while len(body) > 14:                      # merge the least-used colour
        c = min(body, key=body.get)
        others = [o for o in body if o != c]
        o = min(others, key=lambda o: sum((p - q) ** 2 for p, q in zip(o, c)))
        body[o] += body.pop(c)
        for k2, v in remap.items():
            if v == c:
                remap[k2] = o
    out = np.zeros((N, N), dtype=np.int32)
    idx = {}
    for y in range(N):
        for x in range(N):
            if not op[y, x]:
                continue
            c = remap[tuple(int(v) for v in a[y, x, :3])]
            if luma(c) < 40:
                out[y, x] = M["outline"]
            else:
                if c not in idx:
                    idx[c] = alloc_colour(c)
                out[y, x] = idx[c]
    return close_outline(out)


def build_sprites():
    # fallbacks first: whatever master colours they (and the VHDL) don't use
    # are free for the supplied art
    out = {}
    used = {0}
    for i, (name, key, fn) in enumerate(PRIZES):
        if not os.path.exists(os.path.join(HERE, "supplied", key + ".png")):
            out[i] = (name, fn().downsample(), "fallback")
            used |= set(int(v) for v in np.unique(out[i][1]))
    used |= {M[n] for n in RESERVED}
    FREE[:] = [i for i in range(1, L.NMASTER) if i not in used]
    for i, (name, key, fn) in enumerate(PRIZES):
        if i not in out:
            out[i] = (name, supplied_sprite(os.path.join(HERE, "supplied", key + ".png")), "supplied")
    return [out[i] for i in range(len(PRIZES))]


def type_palette(spr):
    """Map master indices in a sprite to nibbles. 0 transparent, 1 outline."""
    used = sorted(set(int(v) for v in np.unique(spr)) - {0, M["outline"]})
    if len(used) > 14:
        raise SystemExit("sprite uses %d colours (max 14)" % len(used))
    m2n = {0: 0, M["outline"]: 1}
    nib2m = [0, M["outline"]] + used + [0] * (14 - len(used))
    for i, mi in enumerate(used):
        m2n[mi] = 2 + i
    return m2n, nib2m


# --------------------------------------------------------------------------
# Font: 6x7 (7x7 for W, M) chunky glyphs -> 16x16 cells, 2bpp
#   0 transparent, 1 outline, 2 fill, 3 highlight (top edge of each stroke)
# --------------------------------------------------------------------------
GLYPHS = {
    " ": [""] * 7,
    "!": ["..##..", "..##..", "..##..", "..##..", "..##..", "......", "..##.."],
    "0": [".####.", "##..##", "##..##", "##..##", "##..##", "##..##", ".####."],
    "1": ["..##..", ".###..", "..##..", "..##..", "..##..", "..##..", ".####."],
    "2": [".####.", "##..##", "....##", "..###.", ".##...", "##....", "######"],
    "3": [".####.", "##..##", "....##", "..###.", "....##", "##..##", ".####."],
    "4": ["...##.", "..###.", ".####.", "##.##.", "######", "...##.", "...##."],
    "5": ["######", "##....", "#####.", "....##", "....##", "##..##", ".####."],
    "6": [".####.", "##....", "##....", "#####.", "##..##", "##..##", ".####."],
    "7": ["######", "....##", "...##.", "..##..", "..##..", "..##..", "..##.."],
    "8": [".####.", "##..##", "##..##", ".####.", "##..##", "##..##", ".####."],
    "9": [".####.", "##..##", "##..##", ".#####", "....##", "....##", ".####."],
    "A": [".####.", "##..##", "##..##", "######", "##..##", "##..##", "##..##"],
    "B": ["#####.", "##..##", "##..##", "#####.", "##..##", "##..##", "#####."],
    "C": [".####.", "##..##", "##....", "##....", "##....", "##..##", ".####."],
    "D": ["#####.", "##..##", "##..##", "##..##", "##..##", "##..##", "#####."],
    "E": ["######", "##....", "##....", "#####.", "##....", "##....", "######"],
    "F": ["######", "##....", "##....", "#####.", "##....", "##....", "##...."],
    "H": ["##..##", "##..##", "##..##", "######", "##..##", "##..##", "##..##"],
    "I": ["######", "..##..", "..##..", "..##..", "..##..", "..##..", "######"],
    "J": ["....##", "....##", "....##", "....##", "##..##", "##..##", ".####."],
    "K": ["##..##", "##.##.", "####..", "###...", "####..", "##.##.", "##..##"],
    "L": ["##....", "##....", "##....", "##....", "##....", "##....", "######"],
    "M": ["##...##", "###.###", "#######", "##.#.##", "##...##", "##...##", "##...##"],
    "N": ["##..##", "###.##", "######", "##.###", "##..##", "##..##", "##..##"],
    "P": ["#####.", "##..##", "##..##", "#####.", "##....", "##....", "##...."],
    "R": ["#####.", "##..##", "##..##", "#####.", "####..", "##.##.", "##..##"],
    "S": [".####.", "##..##", "##....", ".####.", "....##", "##..##", ".####."],
    "T": ["######", "..##..", "..##..", "..##..", "..##..", "..##..", "..##.."],
    "U": ["##..##", "##..##", "##..##", "##..##", "##..##", "##..##", ".####."],
    "W": ["##...##", "##...##", "##.#.##", "##.#.##", "#######", "###.###", "##...##"],
    "Y": ["##..##", "##..##", "##..##", ".####.", "..##..", "..##..", "..##.."],
}


def build_font():
    order = [None] * 32
    for ch, code in L.GLYPH_CODES.items():
        if ch == "O":
            continue
        order[code] = ch
    cells = []
    for ch in order:
        rows = GLYPHS[ch]
        w = max([len(r) for r in rows] + [6])
        g = np.zeros((7, 7), bool)
        for y, r in enumerate(rows):
            for x, cc in enumerate(r):
                g[y, x] = cc == "#"
        cell = np.zeros((16, 16), np.int32)
        fill = np.zeros((16, 16), bool)
        ox = 1 if w == 7 else 2
        for y in range(7):
            for x in range(7):
                if g[y, x]:
                    fill[1 + 2 * y:3 + 2 * y, ox + 2 * x:ox + 2 * x + 2] = True
        dil = fill.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                dil |= shift(fill, dx, dy)
        cell[dil] = 1
        cell[fill] = 2
        top = fill & ~shift(fill, 0, 1)
        cell[top] = 3
        cells.append(cell)
    return cells


# --------------------------------------------------------------------------
# VHDL emission
# --------------------------------------------------------------------------
def vhdl_slv(v, w):
    return '"' + format(v, "0%db" % w) + '"'


def emit_pkg(path, sprites, pals, font, slots_hd, slots_sd):
    lines = []
    w = lines.append
    w("-- Copyright (C) 2026 czartroniks")
    w("-- SPDX-License-Identifier: GPL-3.0-only")
    w("--")
    w("-- claw_pkg.vhd -- GENERATED by art/claw_art.py; do not edit by hand.")
    w("-- Sprite ROM (6 prizes, 48x48 4bpp), per-type palettes, master palette,")
    w("-- 2bpp 16x16 font, corner-inset table, pile slot table, strings, and the")
    w("-- cabinet element table.")
    w("")
    w("library ieee;")
    w("use ieee.std_logic_1164.all;")
    w("use ieee.numeric_std.all;")
    w("")
    w("package claw_pkg is")
    w("")
    # sprites
    flat = []
    for _, spr, _ in sprites:
        m2n = pals[len(flat) // (N * N)][0]
        for y in range(N):
            for x in range(N):
                flat.append(m2n[int(spr[y, x])])
    total = len(flat)
    w("    constant C_NTYPES : natural := %d;" % len(sprites))
    w("    constant C_NPILE_HD : natural := %d;" % (len(slots_hd) - 3))
    w("    constant C_NPILE_SD : natural := %d;" % (len(slots_sd) - 3))
    w("    -- after the pile: the static control-panel synth, the prize hole, the held prize")
    w("    -- px address = type*2304 + row*48 + col; split 8192 + 8192 + 2048")
    for name, lo, size in (("A", 0, 8192), ("B", 8192, 8192), ("C", 16384, 2048)):
        w("    type t_spr_%s is array (0 to %d) of std_logic_vector(3 downto 0);" % (name.lower(), size - 1))
        w("    constant C_SPR_%s : t_spr_%s := (" % (name, name.lower()))
        items = []
        for i in range(size):
            v = flat[lo + i] if lo + i < total else 0
            items.append('x"%X"' % v)
        for i in range(0, size, 16):
            w("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < size else ""))
        w("    );")
    # type palettes
    w("    type t_typepal is array (0 to 127) of unsigned(5 downto 0);")
    w("    constant C_TYPEPAL : t_typepal := (")
    items = []
    for t in range(8):
        nib2m = pals[t][1] if t < len(pals) else [0] * 16
        for n in range(16):
            items.append("to_unsigned(%d, 6)" % nib2m[n])
    for i in range(0, 128, 8):
        w("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < 128 else ""))
    w("    );")
    # palette
    pal = L.palette_yuv([tuple(e) for e in MASTER_BUILT])
    w("    -- palette word = Y(29:20) U(19:10) V(9:0), BT.601")
    w("    type t_pal is array (0 to 255) of std_logic_vector(31 downto 0);")
    w("    constant C_CLAW_PAL : t_pal := (")
    items = []
    for (Y, U, V) in pal:
        items.append('x"%08X"' % ((Y << 20) | (U << 10) | V))
    for i in range(0, 256, 8):
        w("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < 256 else ""))
    w("    );")
    # font: word = {glyph(5), row(4), half(1)}; 8 px x 2 bits, px 0 = bits 15:14
    w("    type t_font is array (0 to 1023) of std_logic_vector(15 downto 0);")
    w("    constant C_FONT : t_font := (")
    items = []
    for cell in font:
        for y in range(16):
            for half in range(2):
                v = 0
                for x in range(8):
                    v = (v << 2) | int(cell[y, half * 8 + x])
                items.append('x"%04X"' % v)
    for i in range(0, 1024, 16):
        w("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < 1024 else ""))
    w("    );")
    # inset table
    w("    -- inset word: outer inset (5:0), inner inset (13:8), 63 = no inner row")
    w("    type t_inset is array (0 to 255) of std_logic_vector(15 downto 0);")
    w("    constant C_INSET : t_inset := (")
    items = ['x"%04X"' % ((ii << 8) | io) for io, ii in L.inset_table()]
    for i in range(0, 256, 16):
        w("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < 256 else ""))
    w("    );")
    # slot table: records of 2 words; HD list at word 0, SD list at word 64
    def rec(s, last):
        w0 = ((s["x"] & 0x7FF) << 5) | (s["type"] << 2) | (s["mirror"] << 1) | int(last)
        w1 = ((s["y"] & 0x7FF) << 5) | (s["depth"] << 3) | (s["tilt"] << 1) | s.get("tr", 0)
        return [w0, w1]
    words = [0] * 256
    for base, lst in ((0, slots_hd), (64, slots_sd)):
        for i, s in enumerate(lst):
            words[base + 2 * i:base + 2 * i + 2] = rec(s, i == len(lst) - 1)
    w("    -- slot record: w0 = x(15:5) type(4:2) mirror(1) last(0);")
    w("    --              w1 = y(15:5) depth(4:3) tilt(2:1) transpose(0).  HD at 0, SD at 64.")
    w("    -- tilt 1/2 = lean right/left, 3 = vertical flip (quarter turns, see ORIENT)")
    w("    type t_slots is array (0 to 255) of std_logic_vector(15 downto 0);")
    w("    constant C_SLOTS : t_slots := (")
    items = ['x"%04X"' % v for v in words]
    for i in range(0, 256, 16):
        w("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < 256 else ""))
    w("    );")
    # strings
    w("    type t_str is array (0 to 63) of unsigned(4 downto 0);")
    w("    constant C_STR : t_str := (")
    items = []
    for sid in range(8):
        txt = ""
        for t in L.TEXTS:
            if t[0] == sid:
                txt = t[1]
        txt = (txt + " " * 8)[:8]
        for ch in txt:
            items.append("to_unsigned(%d, 5)" % L.GLYPH_CODES[ch])
    for i in range(0, 64, 8):
        w("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < 64 else ""))
    w("    );")
    # per-type opaque column extent (stamper trims to it)
    w("    type t_xext is array (0 to 7) of unsigned(11 downto 0);  -- first(11:6) last(5:0)")
    ext = []
    for t in range(8):
        if t < len(sprites):
            cols = np.where((sprites[t][1] != 0).any(0))[0]
            ext.append("to_unsigned(%d, 12)" % ((int(cols[0]) << 6) | int(cols[-1])))
        else:
            ext.append("to_unsigned(0, 12)")
    w("    constant C_XEXT : t_xext := (" + ", ".join(ext) + ");")
    ext = []
    for t in range(8):
        if t < len(sprites):
            rows = np.where((sprites[t][1] != 0).any(1))[0]
            ext.append("to_unsigned(%d, 12)" % ((int(rows[0]) << 6) | int(rows[-1])))
        else:
            ext.append("to_unsigned(0, 12)")
    w("    constant C_YEXT : t_xext := (" + ", ".join(ext) + ");  -- opaque rows (for transposed prizes)")
    w("    type t_tbase is array (0 to 7) of unsigned(14 downto 0);")
    w("    constant C_TBASE : t_tbase := (" + ", ".join("to_unsigned(%d, 15)" % (t * 2304) for t in range(8)) + ");")
    # cabinet element table
    xa = {"L": 0, "R": 1, "C": 2, "X": 3, "J": 4}
    ya = {"A": 0, "H": 1, "J": 2}
    E = L.ELEMENTS
    def arr(name, typ, vals):
        w("    constant %s : %s := (%s);" % (name, typ, ", ".join(vals)))
    def arr2(name, typ, vals):
        assert len(vals) <= 16, "element tables are 16 entries"
        arr(name, typ, vals + ["0"] * (16 - len(vals)))
    w("    constant C_NEL : natural := %d;" % len(E))
    w("    type t_el_i is array (0 to 15) of integer;")
    def pad(v):
        return v + ["0"] * (32 - len(v))
    arr2("C_EL_SLOT", "t_el_i", [str(e[1]) for e in E])
    arr2("C_EL_XA", "t_el_i", [str(xa[e[2][0]]) for e in E])
    arr2("C_EL_X0", "t_el_i", [str(e[2][1]) for e in E])
    arr2("C_EL_XB", "t_el_i", [str(xa[e[3][0]]) for e in E])
    arr2("C_EL_X1", "t_el_i", [str(e[3][1]) for e in E])
    arr2("C_EL_YA", "t_el_i", [str(ya[e[4][0]]) for e in E])
    arr2("C_EL_Y0", "t_el_i", [str(e[4][1]) for e in E])
    arr2("C_EL_YB", "t_el_i", [str(ya[e[5][0]]) for e in E])
    arr2("C_EL_Y1", "t_el_i", [str(e[5][1]) for e in E])
    arr2("C_EL_RC", "t_el_i", [str(e[6]) for e in E])
    arr2("C_EL_KIND", "t_el_i", [str(e[7]) for e in E])
    arr2("C_EL_LEAN", "t_el_i", [str(e[8]) for e in E])
    for d in (1, 2):
        Ed = L.scaled_elements(d)
        arr2("C_EL_X0_%d" % d, "t_el_i", [str(e[2][1]) for e in Ed])
        arr2("C_EL_X1_%d" % d, "t_el_i", [str(e[3][1]) for e in Ed])
        arr2("C_EL_Y0_%d" % d, "t_el_i", [str(e[4][1]) for e in Ed])
        arr2("C_EL_Y1_%d" % d, "t_el_i", [str(e[5][1]) for e in Ed])
    w("    type t_d3_i is array (0 to 3) of integer;")
    arr("C_CLAW_REACH", "t_d3_i", [str(v) for v in L.CLAW_REACH] + ["0"])
    arr("C_CLAW_HOLD", "t_d3_i", [str(v) for v in L.CLAW_HOLD] + ["0"])
    arr("C_ROW_LAND_Y", "t_d3_i", [str(v) for v in L.ROW_LAND_Y] + ["0"])
    arr("C_CABLE_HW", "t_d3_i", [str(v) for v in L.CABLE_HW] + ["0"])
    arr("C_CABLE_BOT", "t_d3_i", [str(v) for v in L.CABLE_BOT] + ["0"])
    for nm in ("JOY_X JOY_BASE_Y CHUTE_WIN_X CHUTE_LAND_Y HOLE_Y HOLE_PX "
               "CABLE_TOP CABLE_T FLAP_ROWS FLAP_EDGE TH_BASE_ADDR").split():
        w("    constant C_%s : integer := %d;" % (nm, getattr(L, nm)))
    for k, (ty, r, rc) in L.JOY_POSE.items():
        w("    constant C_JOY_%s_Y : integer := %d;" % (k.upper(), ty))
        w("    constant C_JOY_%s_R : integer := %d;" % (k.upper(), r))
        w("    constant C_JOY_%s_RC : integer := %d;" % (k.upper(), rc))
    w("    type t_rc_i is array (0 to 7) of integer;")
    arr("C_RC_T", "t_rc_i", [str(t) for r, t in L.RCLASS])
    for i, e in enumerate(E):
        w("    constant E_%s : natural := %d;" % (e[0].upper(), i))
    for nm in ("K_FRAME K_BEZEL K_PANEL K_WINDOW K_DOOR K_CHROME K_DECAL K_ACCENT "
               "K_CHUTE K_PRONG K_CABLE K_BASE K_JSTICK K_JBALL").split():
        w("    constant %s : natural := %d;" % (nm, getattr(L, nm)))
    for nm in ("HL_ROWS SH_ROWS HL_COLS CLAW_Y_MIN CLAW_Y_RANGE CLAW_X_MIN CLAW_X_MARGIN "
               "LEAN_SHIFT OPEN_MAX OPEN_MIN PILE_DARK_Y BULB_X_MIN BULB_R2_RING BULB_R2_CORE").split():
        w("    constant C_%s : integer := %d;" % (nm, getattr(L, nm)))
    w("    constant C_BULB_Y0 : integer := %d;" % L.BULB_Y[0])
    w("    constant C_BULB_Y1 : integer := %d;" % L.BULB_Y[1])
    for i, (lo, hi, st) in enumerate(L.STREAKS):
        w("    constant C_STREAK%d_LO : integer := %d;" % (i, lo))
        w("    constant C_STREAK%d_HI : integer := %d;" % (i, hi))
        w("    constant C_STREAK%d_ST : integer := %d;" % (i, st))
    for nm in ("TH_BASE_SH TH_BASE TH_BASE_HL TH_ACC_SH TH_ACC TH_ACC_HL TH_WALL0 TH_WALL3 "
               "TH_DEEP TH_DOOR TH_TITLE TH_TITLE_HL TH_BG").split():
        w("    constant C_%s : natural := %d;" % (nm, getattr(L, nm)))
    # palette address constants
    w("")
    for name, idx in L.M.items():
        if name.startswith("spare") or name == "empty":
            continue
        w("    constant PA_%s : unsigned(7 downto 0) := to_unsigned(%d, 8);" % (name.upper(), idx))
    w("")
    w("end package claw_pkg;")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# --------------------------------------------------------------------------
# Sheets
# --------------------------------------------------------------------------
def sprite_rgb(spr, depth=0, scale=4):
    pal = L.palette_yuv([tuple(e) for e in MASTER_BUILT])
    h, w = spr.shape
    img = np.zeros((h, w, 4), np.uint8)
    for y in range(h):
        for x in range(w):
            v = int(spr[y, x])
            if v:
                img[y, x, :3] = L.yuv10_to_rgb(*L.haze_yuv(pal[v], depth))
                img[y, x, 3] = 255
    im = Image.fromarray(img, "RGBA")
    return im.resize((w * scale, h * scale), Image.NEAREST)


def contact_sheet(sprites, path):
    cw, ch = 300, 330
    W = cw * len(sprites)
    H = ch + 150
    sheet = Image.new("RGB", (W, H), (58, 72, 140))
    d = ImageDraw.Draw(sheet)
    try:
        fnt = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
        fsm = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    except OSError:
        fnt = fsm = ImageFont.load_default()
    for i, (name, spr, src) in enumerate(sprites):
        x0 = i * cw
        big = sprite_rgb(spr, 0, 4)
        sheet.paste(big, (x0 + (cw - 192) // 2, 40), big)
        d.text((x0 + 12, 10), "%d  %s" % (i, name), fill=(255, 255, 255), font=fnt)
        ncol = len(set(np.unique(spr)) - {0})
        d.text((x0 + 12, 240), "%s, %d colours + transparent" % (src, ncol), fill=(220, 220, 240), font=fsm)
        for dd in (1, 2):
            sm = sprite_rgb(spr, dd, 2)
            sheet.paste(sm, (x0 + 30 + (dd - 1) * 120, 270), sm)
        d.text((x0 + 30, 372), "mid row      back row", fill=(220, 220, 240), font=fsm)
    d.text((12, H - 40), "CLAW prize sprites: 48x48 4bpp, drawn 4x (192 px at 1080p). "
           "Top: front row. Bottom: depth-row palettes at 2x.", fill=(255, 255, 255), font=fsm)
    sheet.save(path)


def font_sheet(font, path):
    pal = L.palette_rgb()
    cols = {0: (30, 10, 40), 1: pal[1], 2: (255, 206, 40), 3: (255, 248, 200)}
    sc = 4
    img = Image.new("RGB", (16 * 16 * sc, 2 * 16 * sc + 8), (30, 10, 40))
    px = img.load()
    for gi, cell in enumerate(font):
        gx, gy = (gi % 16) * 16 * sc, (gi // 16) * (16 * sc + 8)
        for y in range(16):
            for x in range(16):
                for yy in range(sc):
                    for xx in range(sc):
                        px[gx + x * sc + xx, gy + y * sc + yy] = cols[int(cell[y, x])]
    img.save(path)


def main():
    out_dir = os.path.join(HERE, "..", "preview")
    os.makedirs(out_dir, exist_ok=True)
    sprites = build_sprites()
    pals = [type_palette(spr) for _, spr, _ in sprites]
    font = build_font()
    held = dict(x=0, y=2047, type=0, mirror=0, depth=0, tilt=0, tr=0)   # y 2047 = hidden
    hd = L.pile_slots(1920) + [L.door_slot(1920), L.hole_slot(1920), held]
    sd = L.pile_slots(1440) + [L.door_slot(1440), L.hole_slot(1440), held]
    emit_pkg(os.path.join(HERE, "..", "claw_pkg.vhd"), sprites, pals, font, hd, sd)
    contact_sheet(sprites, os.path.join(out_dir, "prize_contact_sheet.png"))
    font_sheet(font, os.path.join(out_dir, "font_sheet.png"))
    np.savez(os.path.join(HERE, "claw_assets.npz"),
             sprites=np.array([s for _, s, _ in sprites]),
             font=np.array(font),
             nib2m=np.array([p[1] for p in pals]),
             master=np.array([e[1] for e in MASTER_BUILT]))
    print("supplied colours added: %d; master slots still free: %d" % (_next_spare[0], len(FREE)))
    for (name, spr, src), (m2n, nib2m) in zip(sprites, pals):
        print("%-12s %-8s colours=%d" % (name, src, len(set(np.unique(spr)) - {0})))
    print("slots HD=%d SD=%d" % (len(hd), len(sd)))


if __name__ == "__main__":
    main()
