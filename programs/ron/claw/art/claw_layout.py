"""CLAW shared layout: palette, themes, cabinet elements, pile slots, text.

Single source of truth for claw_art.py (which emits claw_pkg.vhd) and
claw_ref.py (the Python reference renderer).  All geometry is in virtual
1080-line units: VW = 1920 at 1080p/720p, 1440 at SD (a 4:3 window).
"""
import math
import random

# ---------------------------------------------------------------------------
# Palette.  256 entries, 10-bit BT.601 YUV (no U/V swap: CLAW's generated
# avid keeps the encoders' Cb/Cr pairing straight; HW-confirmed v0.2.0 showed
# red as blue with the swap on).
#   0..63    master colours (0 = empty / transparent, never drawn); the depth
#            haze is arithmetic at the output (row 1 x13/16, row 2 x5/8)
#   64..159  6 cabinet themes x 16 colours: addr = 64 + 16*theme + c
# Supplied prize art adds its exact colours into the spare master slots.
# ---------------------------------------------------------------------------
MASTER = [
    ("empty",        (0, 0, 0)),
    ("outline",      (26, 20, 40)),
    ("chrome_sh",    (104, 112, 132)),
    ("chrome",       (168, 176, 194)),
    ("chrome_hl",    (226, 232, 242)),
    ("white",        (250, 250, 250)),
    ("black",        (54, 50, 66)),
    ("black_hl",     (96, 92, 112)),
    ("red_sh",       (150, 22, 42)),
    ("red",          (228, 46, 62)),
    ("red_hl",       (255, 128, 128)),
    ("orange",       (250, 140, 30)),
    ("yellow_sh",    (205, 150, 0)),
    ("yellow",       (255, 214, 40)),
    ("yellow_hl",    (255, 246, 165)),
    ("green_sh",     (22, 130, 62)),
    ("green",        (52, 200, 92)),
    ("green_hl",     (155, 240, 165)),
    ("cyan_sh",      (0, 140, 172)),
    ("cyan",         (40, 210, 232)),
    ("cyan_hl",      (172, 246, 252)),
    ("blue_sh",      (30, 50, 152)),
    ("blue",         (62, 102, 232)),
    ("blue_hl",      (145, 172, 255)),
    ("purple_sh",    (88, 30, 140)),
    ("purple",       (150, 72, 212)),
    ("purple_hl",    (204, 152, 252)),
    ("pink_sh",      (172, 30, 120)),
    ("pink",         (242, 82, 172)),
    ("pink_hl",      (255, 172, 222)),
    ("brown_sh",     (92, 50, 26)),
    ("brown",        (152, 92, 46)),
    ("brown_hl",     (208, 148, 92)),
    ("cream",        (246, 232, 192)),
    ("grey",         (132, 132, 148)),
    ("screen",       (18, 28, 52)),
    ("gold_sh",      (160, 108, 10)),
    ("gold",         (238, 182, 40)),
    ("gold_hl",      (255, 236, 142)),
    ("blush",        (255, 142, 152)),
    ("bulb_on",      (255, 244, 186)),
    ("bulb_off",     (128, 78, 36)),
    ("magenta",      (226, 60, 210)),
]
NMASTER = 64
MASTER += [("spare%d" % i, (0, 0, 0)) for i in range(len(MASTER), NMASTER)]
assert len(MASTER) == NMASTER
TH_BASE_ADDR = 64
M = {name: i for i, (name, _) in enumerate(MASTER)}

# Cabinet theme colour slots (c = 0..15).
TH_BASE_SH, TH_BASE, TH_BASE_HL = 0, 1, 2
TH_ACC_SH, TH_ACC, TH_ACC_HL = 3, 4, 5
TH_WALL0, TH_WALL1, TH_WALL2, TH_WALL3 = 6, 7, 8, 9
TH_DEEP, TH_DOOR, TH_TITLE, TH_TITLE_HL, TH_SPARE, TH_BG = 10, 11, 12, 13, 14, 15

THEMES = [
    # name, base (sh, mid, hl), accent (sh, mid, hl), wall 0..3 (top->bottom),
    # marquee deep, door interior, title fill, title highlight, spare, bg
    ("Classic",
     [(128, 14, 26), (204, 32, 44), (248, 98, 96)],
     [(192, 140, 0), (255, 206, 40), (255, 242, 152)],
     [(22, 24, 64), (32, 38, 88), (44, 54, 114), (58, 72, 140)],
     (30, 10, 40), (14, 12, 26), (255, 206, 40), (255, 248, 200),
     (0, 0, 0), (60, 8, 16)),
    ("Neon",
     [(168, 10, 100), (255, 42, 160), (255, 146, 214)],
     [(0, 138, 178), (0, 228, 255), (172, 250, 255)],
     [(16, 8, 36), (26, 14, 58), (38, 22, 84), (52, 32, 112)],
     (14, 4, 30), (8, 4, 18), (0, 236, 255), (220, 255, 255),
     (0, 0, 0), (40, 4, 40)),
    ("Jackpot",
     [(140, 90, 0), (222, 162, 22), (255, 226, 112)],
     [(130, 10, 10), (212, 32, 32), (255, 112, 100)],
     [(44, 16, 8), (66, 26, 12), (92, 40, 18), (120, 56, 24)],
     (40, 8, 4), (20, 8, 4), (255, 232, 100), (255, 252, 220),
     (0, 0, 0), (70, 40, 0)),
    ("Vaporwave",
     [(70, 26, 122), (132, 62, 202), (202, 142, 255)],
     [(170, 50, 132), (255, 122, 202), (255, 202, 236)],
     [(34, 14, 70), (34, 40, 100), (30, 76, 124), (32, 112, 146)],
     (24, 8, 48), (12, 6, 26), (255, 132, 210), (255, 230, 248),
     (0, 0, 0), (40, 14, 70)),
    ("Mint",
     [(40, 150, 116), (112, 222, 182), (192, 252, 228)],
     [(110, 120, 140), (182, 192, 206), (236, 241, 248)],
     [(16, 52, 52), (24, 70, 68), (36, 92, 88), (50, 116, 108)],
     (10, 40, 38), (8, 22, 22), (192, 252, 228), (255, 255, 255),
     (0, 0, 0), (20, 80, 64)),
    ("Studio",
     [(20, 20, 24), (44, 44, 50), (104, 104, 114)],
     [(150, 150, 160), (232, 232, 236), (255, 255, 255)],
     [(26, 26, 30), (36, 36, 42), (50, 50, 58), (66, 66, 76)],
     (14, 14, 16), (8, 8, 10), (240, 240, 244), (255, 255, 255),
     (0, 0, 0), (16, 16, 18)),
]


def theme_colours(t):
    name, base, acc, wall, deep, door, title, title_hl, spare, bg = THEMES[t]
    return base + acc + wall + [deep, door, title, title_hl, spare, bg]


HAZE_ON = False   # v0.4.0: removed (Ron's pick, to make room for Auto)


def haze_yuv(yuv, depth):
    """Depth-row haze, exactly as the VHDL: scale toward black / neutral."""
    if not HAZE_ON:
        return yuv
    def k(d):
        if depth == 1:
            return d - (d >> 3)                     # x7/8
        if depth == 2:
            return d - (d >> 2)                     # x3/4
        return d
    Y, U, V = yuv
    return 64 + k(Y - 64), 512 + k(U - 512), 512 + k(V - 512)


def rgb_to_yuv10(rgb):
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    pb = 0.564 * (b - y)
    pr = 0.713 * (r - y)
    Y = int(round(64 + 876 * y))
    U = int(round(512 + 896 * pb))
    V = int(round(512 + 896 * pr))
    clamp = lambda v: max(4, min(1019, v))
    return clamp(Y), clamp(U), clamp(V)


def yuv10_to_rgb(Y, U, V):
    y = (Y - 64) / 876.0
    pb = (U - 512) / 896.0
    pr = (V - 512) / 896.0
    r = y + 1.402 * pr
    g = y - 0.344136 * pb - 0.714136 * pr
    b = y + 1.772 * pb
    f = lambda c: max(0, min(255, int(round(c * 255))))
    return f(r), f(g), f(b)


def palette_rgb(master=None):
    master = master or MASTER
    pal = [(0, 0, 0)] * 256
    for i, (_, rgb) in enumerate(master):
        pal[i] = rgb
    for t in range(len(THEMES)):
        for c, rgb in enumerate(theme_colours(t)):
            pal[TH_BASE_ADDR + 16 * t + c] = rgb
    return pal


def palette_yuv(master=None):
    return [rgb_to_yuv10(c) for c in palette_rgb(master)]


# ---------------------------------------------------------------------------
# Corner-radius classes and the inset table (1 EBR, 256 x 16).
# Row k (0 = the element's top or bottom row, whichever is nearer):
#   outer inset io(k) = r - round(sqrt(r^2 - (r-k-0.5)^2))  (k < r), else 0
#   inner inset ii(k) = t + inset of radius (r-t) at row k-t, or 63 = none
# ---------------------------------------------------------------------------
RCLASS = [(30, 5), (14, 5), (13, 5), (16, 5), (26, 5), (21, 5), (18, 5), (5, 2)]


def corner_inset(r, k):
    if k >= r:
        return 0
    d = r - k - 0.5
    return r - int(round(math.sqrt(max(0.0, r * r - d * d))))


def inset_table():
    tab = []
    for rc in range(8):
        r, t = RCLASS[rc]
        for k in range(32):
            io = corner_inset(r, k)
            if k < t:
                ii = 63
            else:
                ii = t + corner_inset(r - t, k - t)
            tab.append((io, ii))
    return tab


# ---------------------------------------------------------------------------
# Cabinet elements (procedural, evaluated through 9 per-line slots).
# x anchors: L = 0, R = VW, C = VW/2, X = claw x, J = joystick ball x.
# y anchors: A = 0, H = hub y, J = joystick top y.  lean: -1/+1 = prong (pivot
# at the top, x -/+ (row * open) >> 5); 2 = joystick (pivot at the bottom).
# The joystick ball's box is J +- ball radius (special-cased in the VHDL).
# ---------------------------------------------------------------------------
K_FRAME, K_BEZEL, K_PANEL, K_WINDOW, K_DOOR, K_CHROME, K_DECAL, K_ACCENT, \
    K_CHUTE, K_PRONG, K_CABLE, K_BASE, K_JSTICK, K_JBALL = range(14)

# name, slot, (xa, x0), (xb, x1), (ya, y0), (yb, y1), rclass, kind, lean
ELEMENTS = [
    ("marq_out",   0, ("L", 16),  ("R", -16),  ("A", 10),  ("A", 132),  0, K_FRAME,  0),
    ("bezel",      0, ("L", 20),  ("R", -20),  ("A", 142), ("A", 950),  0, K_BEZEL,  0),
    ("base",       0, ("L", 20),  ("R", -20),  ("A", 958), ("A", 1074), 4, K_BASE,   0),
    ("marq_panel", 1, ("L", 40),  ("R", -40),  ("A", 34),  ("A", 108),  1, K_PANEL,  0),
    ("window",     1, ("L", 42),  ("R", -42),  ("A", 164), ("A", 928),  1, K_WINDOW, 0),
    ("door",       1, ("C", -150), ("C", 150), ("A", 972), ("A", 1060), 1, K_DOOR,   0),
    ("rail",       2, ("L", 56),  ("R", -56),  ("A", 176), ("A", 202),  2, K_CHROME, 0),
    ("hub",        2, ("X", -70), ("X", 70),   ("H", 0),   ("H", 62),   3, K_CHROME, 0),
    ("decal",      2, ("C", 184), ("C", 336),  ("A", 990), ("A", 1042), 2, K_DECAL,  0),
    ("carriage",   3, ("X", -84), ("X", 84),   ("A", 164), ("A", 218),  1, K_ACCENT, 0),
    ("chute",      3, ("R", -340), ("R", -60), ("A", 600), ("A", 950),  3, K_CHUTE,  0),
    ("prong_l",    5, ("X", -64), ("X", -34),  ("H", 40),  ("H", 190),  1, K_PRONG, -1),
    ("prong_r",    6, ("X", 34),  ("X", 64),   ("H", 40),  ("H", 190),  1, K_PRONG,  1),
    ("prong_m",    7, ("X", -15), ("X", 15),   ("H", 40),  ("H", 178),  1, K_PRONG,  0),
    ("joy_stick",  5, ("C", -7),  ("C", 7),    ("J", 0),   ("A", 1016), 7, K_JSTICK, 2),
    ("joy_ball",   2, ("J", -18), ("J", 18),   ("J", -18), ("J", 18),   6, K_JBALL,  0),
]
# Joystick (the v0.2.0 joystick, ball +15 %): drawn on top of the prize-door
# window, the stick's base at the centre of the window's top band, in front of
# everything.  Stick = element joy_stick (lean 2: pivot at the bottom row),
# ball = element joy_ball (box = J +- radius, special-cased in the VHDL).
JOY_X = 0               # stick base x, relative to VW/2
JOY_BASE_Y = 1016       # stick bottom
# pose: (top y, ball radius, ball rclass) for back-tilt, upright, front-tilt
JOY_POSE = {"back": (962, 14, 1), "up": (952, 18, 6), "front": (966, 21, 5)}

# Claw scale per depth row (front, middle, back): hub, prongs and cable shrink
# toward the hub top; reach and hold point scale with them.
CLAW_SCALE = [1.0, 0.82, 0.66]
CLAW_ELEMS = ("hub", "prong_l", "prong_r", "prong_m")
# Cable: direct logic (slot 8), x = claw x +- half width, y = CABLE_TOP .. hub y + bottom
CABLE_TOP = 212
CABLE_HW = [6, 5, 4]          # half width per depth row (outline 2)
CABLE_BOT = [8, 7, 5]
CABLE_T = 2
# Chute flap: the chute element's top band (rows inside the outline)
FLAP_ROWS = 22
FLAP_EDGE = 3                 # outline rows at the band's bottom


def scaled_elements(depth):
    k = CLAW_SCALE[depth]
    out = []
    for e in ELEMENTS:
        name, slot, xa, xb, ya, yb = e[:6]
        if name in CLAW_ELEMS:
            sc = lambda a: (a[0], int(round(a[1] * k)) if a[0] in ("X", "H") else a[1])
            e = (name, slot, sc(xa), sc(xb), sc(ya), sc(yb)) + e[6:]
        out.append(e)
    return out


CLAW_REACH = [int(round(150 * k)) for k in CLAW_SCALE]   # prong tips reach y + this
CLAW_HOLD = [int(round(40 * k)) for k in CLAW_SCALE]     # held prize top = hub y + this
# The prize hole under the chute: a hardwired 17th element (the table is 16
# entries), same size as the door window; won prizes land in it.
HOLE = ("hole", 3, ("R", -350), ("R", -50), ("A", 972), ("A", 1060), 1, K_DOOR, 0)
HOLE_Y = 968                     # a won prize's top in the hole
HOLE_PX = -296                   # its x, relative to VW (centre VW - 200, minus 96)
NSLOT = 9
HL_ROWS = 5      # top highlight rows inside the outline
SH_ROWS = 5      # bottom shadow rows inside the outline
HL_COLS = 7      # left highlight columns inside the outline

# Claw geometry limits
CLAW_Y_MIN = 226         # hub top at full retract (under the carriage)
CLAW_Y_RANGE = 520       # P12 travel
LEAN_SHIFT = 5           # prong lean = (row * open) >> 5
OPEN_MAX, OPEN_MIN = 8, -6
CLAW_X_MIN = 130
CLAW_X_MARGIN = 200      # VW - margin = right-most claw x (over the chute centre)
CHUTE_WIN_X = 340        # claw x >= VW - this = over the chute opening
CHUTE_LAND_Y = 968       # a won prize falls through the chute into the hole
ROW_LAND_Y = [708, 646, 586]   # where a dropped prize lands, per depth row

# ---------------------------------------------------------------------------
# Text: 2bpp 16x16 font, 32 glyphs.  Code 0 = space, 1 = '!', 2..11 = 0..9,
# 12.. = letters below.  'O' renders as '0'.
# ---------------------------------------------------------------------------
LETTERS = "ABCDEFHIJKLMNPRSTUWY"
GLYPH_CODES = {" ": 0, "!": 1}
for i in range(10):
    GLYPH_CODES[str(i)] = 2 + i
for i, ch in enumerate(LETTERS):
    GLYPH_CODES[ch] = 12 + i
GLYPH_CODES["O"] = GLYPH_CODES["0"]

# id, string (8 chars max), scale shift (1 = 2x, 2 = 4x), (xa, x0), y0
TEXTS = [
    (0, "CLAW",     2, ("C", -128), 39),
    # (v0.5.0: ids 1-4, the CREDITS / TRIES / WINS scoreboard, cut for room)
    (5, "DROP",     1, ("C", 192),  1000),
]

# Text zones start on 32-unit cells (the title on 64) so the VHDL takes char
# and column straight from x bits; at SD the title and decal shift to the cell
# grid: x overrides for VW = 1440.
TEXT_X_SD = {0: 576, 5: 928}

# Bulbs on the marquee frame
BULB_Y = (24, 118)
BULB_R2_RING = 49     # dx^2 + dy^2 < 49  -> bulb (socket ring)
BULB_R2_CORE = 26     # < 26 -> lamp
BULB_X_MIN = 56       # and <= VW - 56

# Glass streaks: s = vx + vy - VW/2, lighten inside the window
STREAKS = [(-60, 0, 2), (24, 44, 1), (700, 784, 2)]   # [lo, hi), strength

# ---------------------------------------------------------------------------
# Pile slot layout.  Each record: x (left, virtual), y (top), type, mirror,
# depth (0 front, 1 mid, 2 back), tilt (0 none, 1 lean right, 2 lean left).
# Records are in draw order: back row first.
# ---------------------------------------------------------------------------
SPRITE_W = 48
SPRITE_SCALE = 4
SPRITE_PX = SPRITE_W * SPRITE_SCALE     # 192 virtual units
ROW_Y = [(2, 586), (1, 646), (0, 708), (0, 772)]   # rows back->front: (depth, top)
PILE_DARK_Y = 712     # wall below this dithers into the dark heap interior


TYPE_WEIGHTS = [4, 4, 4, 1, 3, 2, 3, 1]   # rare prizes are rare (token, wizard)
# quarter turns as (transpose, mirror, tilt): tilt 3 = vertical flip, no lean
#   1 = 90 deg CW, 2 = 180, 3 = 270
ORIENT = {0: (0, 0, 0), 1: (1, 1, 0), 2: (0, 1, 3), 3: (1, 0, 3)}


def pile_slots(vw, seed=7, ntypes=8):
    rng = random.Random(seed + vw)
    per_row = 6 if vw >= 1920 else 5
    x_lo, x_hi = 40, vw - 340 - 110
    pitch = (x_hi - x_lo) / (per_row - 1)
    slots = []
    stag = [0.0, 0.5, 0.15, 0.65]
    for r, (depth, ytop) in enumerate(ROW_Y):
        for i in range(per_row):
            x = x_lo + i * pitch + (stag[r] - 0.3) * pitch + rng.uniform(-26, 26)
            x = int(round(x / 2.0)) * 2
            x = max(24, min(vw - 340 - 40, x))
            y = ytop + rng.randint(-12, 12)
            d = dict(x=x, y=y, type=rng.choices(range(ntypes), TYPE_WEIGHTS[:ntypes])[0],
                     mirror=rng.randint(0, 1), depth=depth,
                     tilt=rng.choice([0, 0, 1, 2]), tr=0)
            o = rng.choices([0, 1, 2, 3], [80, 8, 4, 8])[0]
            if o:
                d["tr"], d["mirror"], d["tilt"] = ORIENT[o]
            slots.append(d)
    # every prize type appears at least once (the rare ones too); the rare
    # token and wizard go to the front rows so they're seen
    front = [i for i, d in enumerate(slots) if d["depth"] == 0]
    mid = [i for i, d in enumerate(slots) if d["depth"] == 1]
    rng.shuffle(front)
    rng.shuffle(mid)
    picks = {3: front[0], 7: front[1]}
    rest = [i for i in front[2:] + mid if i not in picks.values()]
    for t in range(ntypes):
        if t in picks:
            slots[picks[t]]["type"] = t
        elif not any(d["type"] == t for d in slots):
            slots[rest.pop(0)]["type"] = t
    return slots


def worst_line(slots, h=192):
    best = 0
    for y in range(1080):
        n = sum(1 for s in slots if s["y"] <= y < s["y"] + h)
        best = max(best, n)
    return best


DOOR_SLOT = dict(x=None, y=968, type=5, mirror=0, depth=0, tilt=0, tr=0)  # x = VW/2 - 96


def hole_slot(vw):
    return dict(x=vw + HOLE_PX, y=2047, type=0, mirror=0, depth=0, tilt=0, tr=0)


def door_slot(vw):
    d = dict(DOOR_SLOT)
    d["x"] = vw // 2 - 96
    return d
