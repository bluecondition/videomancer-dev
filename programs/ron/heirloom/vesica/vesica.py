#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
VESICA v0.5.0 - build hook.  Writes vesica_pkg.vhd next to this file.

Fitted to the photo WIN_20230307_01_04_17 (see SUMMARY.md):
  * pastel lattice  F = A(x) + 1.03 B(y) - 1.05,  A = |1 - 2 frac((x - 971.96) / 966.03)| with a soft
    knee at the top (w = 0.044) + 0.126 u^2 - 0.0096 u (u = (x - 980) / 960),
    B = |1 - 2 frac((y - 519.04) / 516.30 + 0.5)|, coloured by |F| through the photo's band palette
    (lavender / pink / aqua / sky / yellow / green) sampled from the photo in 64 steps of 0.01;
  * orange layer: two circle families on one horizontal fold (period 944.83) and the compressive
    vertical waveform Y = d + a - 0.639e-3 a^2 (a = |y - 521.90|):
    V (vesica)  cx 1013.32, d 665.30, R 804.94;   C (crescents)  cx 997.41, d 977.79, R 1233.16.
All lengths are virtual 1080-line pixels.

Package contents:
  C_VES_PY/PU/PV  512 x 8: {Y8, U8, V8} (hardware convention U = Cr, V = Cb), address
              {layer, |F| index (0.01 steps)}.  Layers: 0 lattice, 1 plate
              (translucent brown over the lattice), 2 dash dark, 3 dash orange, 4..7 vesica stripe
              levels 0 .. 3 (translucent orange over the lattice).  Y pre-scaled by 1.086 so the
              shell's 13/16 output luma lands the brightest band at ~790.
  C_VES_PQ    512 x 10: the lattice's parabola every 4 virtual px.
  C_VES_KNEE  256 x 12: the A-ramp top knee: corrected A (Q12) over its top 9 %.
  C_VES_GLT   1024 x 8: lattice glitch notches over two vertical periods (depth x 256).
  C_VES_ROWS  1024 x 12: line-row phase n(a) + 64, Q4 (a = |y - cy| in virtual lines).
  C_VES_CRES  128 x 24: crescent colour by {bottom, a / 8}.
  C_VES_KH    256 x 16: K1 lattice phase step; C_VES_IL 256 x 16: K3 1 / stripe pitch;
  C_VES_K26   512 x 8: K2 palette gain {shift, mantissa} | K6 video gain;
  C_VES_EYE0..4  5 x 256 x 16: P12 lens geometry (R^2, tip, half-height, plate, fold period).
  C_VES_KQ    the crescent shoulder (56 - t)^2 / 224; C_VES_TV_* / TC_* edge and zone thresholds on
              (rho^2 - R^2) at the photo's radii; C_VES_RC2 the crescent circle's R^2; C_VES_<colour>
              the rim / arc / tail / plate-halo / axis / crescent-shade colours.
  C_VES_*     fixed colours (Y8 U8 V8).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PAL_PHOTO = [
    (168, 180, 254), (169, 180, 254), (169, 180, 255), (169, 180, 255), (171, 180, 255), (173, 180, 255), (183, 180, 255), (205, 180, 255),
    (229, 179, 255), (239, 178, 255), (241, 178, 255), (241, 179, 255), (240, 180, 255), (239, 182, 255), (233, 192, 253), (214, 214, 247),
    (191, 241, 237), (171, 253, 229), (166, 254, 227), (165, 255, 227), (165, 255, 227), (165, 254, 229), (165, 254, 232), (165, 253, 242),
    (165, 250, 250), (165, 246, 252), (166, 245, 253), (166, 244, 253), (167, 244, 253), (169, 245, 252), (176, 246, 247), (200, 252, 226),
    (232, 254, 194), (247, 255, 180), (250, 255, 178), (247, 255, 178), (240, 255, 177), (233, 255, 177), (227, 255, 177), (215, 255, 177),
    (207, 255, 177), (203, 255, 176), (202, 255, 176), (202, 255, 176), (202, 255, 176), (202, 255, 175), (202, 255, 175), (202, 255, 175),
    (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175),
    (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175), (202, 255, 175),
]

# ------------------------------------------------------------------ fitted model (virtual px)
LAT_XA, LAT_PA, LAT_Q, LAT_L, LAT_W = 971.958, 966.030, 0.126, -0.0096, 0.0441
LAT_YC, LAT_PB, LAT_KB, LAT_FS = 519.042, 516.302, 1.03, 1.05
CY, PER, CC = 521.90, 944.83, 0.639e-3
CXV, DV, RV0 = 1013.32, 665.30, 804.94
CXC, DC, RC0 = 997.41, 977.79, 1233.16
H0 = RV0 - DV                       # 139.64, the photo's eye opening
LINE_A = [518, 509, 495, 484, 469, 463, 456, 452, 446, 438, 430, 422, 412, 401, 389, 372, 338]
GLITCH = {0: [(-7.4, 1.5, 0.42), (-2.4, 2.0, 0.42), (2.6, 1.0, 0.6), (6.6, 1.0, 0.6), (10.6, 2.2, 0.73),
              (14.1, 1.0, 0.4), (28.6, 0.8, 0.5), (32.1, 0.6, 0.35)],
          1: [(-31.2, 0.6, 0.35), (-20.2, 10.5, 0.68), (-3.2, 0.6, 0.4), (0.3, 0.8, 0.6), (2.8, 0.8, 0.45),
              (7.3, 2.2, 0.72)]}
CRES_A = [240, 280, 320, 360, 395]
CRES_TOP = [(241, 65, 12), (233, 90, 38), (234, 120, 49), (227, 149, 73), (227, 164, 82)]
CRES_BOT = [(226, 75, 23), (238, 67, 12), (245, 66, 9), (248, 75, 16), (241, 88, 26)]
TSTR = [0.0, 0.3, 0.6, 0.9]          # vesica stripe levels (layers 4..7)
COLS = dict(RIM=(200, 60, 18), RIMR=(200, 60, 30), INNER=(255, 135, 35), ARC=(200, 70, 30),
            TAIL=(200, 45, 25), TAN=(222, 175, 110), AXIS=(240, 190, 100), CDARK=(180, 48, 22))
YPRE = 1.086


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return 64 + y * 876, 512 + cr * 896, 512 + cb * 896


def yuv8(rgb):
    """RGB -> (Y8, U8, V8): hardware convention, Y pre-scaled, 8 bits each (10-bit >> 2)."""
    y, u, v = rgb_to_hw10(rgb)
    y = min(1023.0, 64 + (y - 64) * YPRE)
    cl = lambda t: max(0, min(255, int(round(t / 4.0))))
    return cl(y), cl(u), cl(v)


def clip8(t):
    return max(0.0, min(255.0, t))


def layer_rgb(lat, layer):
    r, g, b = lat
    if layer == 0:
        return lat
    if layer == 1:
        return (clip8(0.457 * r + 101.7), clip8(0.365 * g + 26.8), clip8(0.783 * b - 67.5))
    if layer == 2:
        return (clip8((0.033 * r + 188.8) * 0.85), clip8((0.592 * g - 4.7) * 0.85), clip8((0.193 * b + 53.5) * 0.85))
    if layer == 3:
        return (228.0, clip8(0.45 * g), clip8(0.15 * b + 20))
    t = TSTR[layer - 4]
    base = (222.6, 0.173 * g + 55.0, 0.214 * b - 6.5)
    peak = (255.0, 0.316 * g + 60.0, 0.204 * b - 5.5)
    return tuple(clip8(base[i] + (peak[i] - base[i]) * t) for i in range(3))


def pal_rom():
    rom = []
    for pal in (PAL_PHOTO,):
        for layer in range(8):
            for i in range(64):
                rom.append(yuv8(layer_rgb(pal[i], layer)))
    return rom


def knee_rom():
    return [min(4095, int(round(((i + 0.5) * 32) ** 2 / (4 * LAT_W * 65536)))) for i in range(256)]


def pq_rom():
    """the lattice oscillator's slow bend q u^2 + l u (Q12) every 8 half-px (4 virtual px)."""
    out = []
    for i in range(512):
        u = ((i * 8 + 4) / 2.0 - 980) / 960.0
        out.append(max(0, min(1023, int(round((LAT_Q * u * u + LAT_L * u) * 4096)))))
    return out


def knee12_rom():
    """the lattice's top knee: the corrected A in Q12 for tA = 59755 + 32 i + 16 (tA >> 4 below the knee)."""
    k = knee_rom()
    return [(59755 + 32 * i + 16 - k[i]) >> 4 for i in range(256)]


def glitch_rom():
    out = []
    for i in range(1024):
        ph = (i + 0.5) / 512.0
        k = int(math.floor(ph + 0.5)); rel = (ph - k) * LAT_PB; par = k % 2
        d = 0.0
        for off, hw, dep in GLITCH[par]:
            if abs(rel - off) <= hw + 0.5:
                d = max(d, dep)
        out.append(int(round(d * 256)) if d < 1 else 255)
    return out


def n_of_a(a):
    if a <= LINE_A[-1]:
        return len(LINE_A) - 1 + (LINE_A[-1] - a) / 9.5
    if a >= LINE_A[0]:
        return -(a - LINE_A[0]) / 9.5
    for j in range(len(LINE_A) - 1):
        a0, a1 = LINE_A[j], LINE_A[j + 1]
        if a1 <= a <= a0:
            return j + (a0 - a) / (a0 - a1)
    return 0.0


def rows_rom():
    return [int(round((n_of_a(a) + 64) * 16)) for a in range(1024)]


def lerp_tab(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for j in range(len(xs) - 1):
        if xs[j] <= x <= xs[j + 1]:
            t = (x - xs[j]) / (xs[j + 1] - xs[j])
            return tuple(ys[j][c] + (ys[j + 1][c] - ys[j][c]) * t for c in range(3))


def cres_rom():
    out = []
    for bot in (0, 1):
        for i in range(64):
            out.append(yuv8(lerp_tab(CRES_A, CRES_BOT if bot else CRES_TOP, i * 8 + 4)))
    return out


def exp_rom():
    return [int(round(32768 * 2 ** (-i / 256.0))) for i in range(256)]


def hexv(t, w):
    return '"' + format(t, '0%db' % w) + '"'


# ------------------------------------------------------------------ RTL constants
CLASSES = [(2, 0), (3, 0), (4, 480)]          # (xs half-px per pixel, hx0 first-pixel half-px): 1080, 720p, SD


def dda_class(xs, hx0):
    """lattice parabola q u^2 + l u (Q12 F units) as a 2nd-order DDA in Q28, per pixel."""
    x0 = hx0 / 2.0; u0 = (x0 - 980) / 960.0; du = (xs / 2.0) / 960.0
    k = 4096 * 65536
    P0 = int(round((LAT_Q * u0 * u0 + LAT_L * u0) * k))
    D0 = int(round((2 * LAT_Q * u0 * du + LAT_L * du + LAT_Q * du * du) * k))
    DD = int(round(2 * LAT_Q * du * du * k))
    return P0, D0, DD


def tv(k):
    """V-lens threshold for a radial offset k/16 px, in dVs units ((rho^2 - R^2) >> 7) at the photo's R."""
    d = k / 16.0
    return int(round((2 * RV0 * d + d * abs(d)) / 128.0))


def tc(k):
    """C-family threshold for k/16 px in dCs units ((rho^2 - R^2) >> 9) at the photo's R."""
    d = k / 16.0
    return int(round((2 * RC0 * d + d * abs(d)) / 512.0))


def kq_rom():
    """crescent outer-edge knee: (56 - t)^2 / 224 for t < 56 (dCs units)."""
    return [((56 - t) ** 2) // 224 if t < 56 else 0 for t in range(64)]


# ------------------------------------------------------------------ knob ROMs (glided knob >> 2)
def e2v(x):
    e = exp_rom()[x & 255]
    n = x >> 8
    return e >> 1 if n == 1 else (e << 1 if n == -1 else (e << 2 if n == -2 else e))


def s10(v):
    v &= 1023
    return v - 1024 if v >= 512 else v


def kh_rom():
    """K1 Lattice: the lattice phase step per half-pixel, 2^24 / (966 px x scale) / 2."""
    return [(e2v(s10((4 * i - 512) >> 1)) * 8684 >> 15) & 0x7FFF for i in range(256)]


def invl_rom():
    """K3 Stripes: 65536 / stripe pitch (Q16 cycles per virtual px)."""
    out = []
    for i in range(256):
        u3 = 4 * i - 512
        x3 = (u3 * 150) >> 8 if u3 >= 0 else (u3 * 200) >> 8
        out.append((e2v(s10(x3)) * 923 >> 15) & 0xFFF)
    return out


def k26_rom():
    """K2 Bands: palette gain as {shift (2), mantissa 16..31 (5)}; K6 Etch: video gain as {exp (3), mantissa (3)}."""
    out = []
    for i in range(256):
        g2 = 4 * i
        gq = (e2v(s10((((g2 - 512) << 7) + ((g2 - 512) << 4) + ((g2 - 512) << 1)) >> 8)) * 1600 >> 15) & 0xFFF
        if gq >= 2048: m, e = gq >> 7, 2
        elif gq >= 1024: m, e = gq >> 6, 1
        else: m, e = gq >> 5, 0
        out.append((e << 5) | (m & 31))
    for i in range(256):
        g6 = 4 * i
        ge = (g6 * g6 * 230 >> 20) & 0xFF
        e = 0
        while ge >> e > 7: e += 1
        out.append((e << 3) | (ge >> e))
    return out


def eye(gp):
    """P12 Eye: the lens geometry for the glided slider value gp (0..1023)."""
    h16 = 384 + (gp * 3192 >> 10) + ((gp * gp * 1064 >> 20) & 0xFFF)
    Rv16 = 10645 + h16
    Rv2 = (Rv16 * Rv16 >> 8) & 0xFFFFF
    tipX = math.isqrt(max(Rv2 - 442624, 0))
    m = max(79, ((tipX + 25) * 10923 >> 16) & 0x1FF)
    ch2 = (((h16 * h16) >> 2) * 670 >> 22) & 0x3FF
    wh16 = (h16 + ch2 + (((ch2 * h16) * 1340 >> 24) & 0x1FF)) & 0x1FFF
    bar16 = (wh16 * 113 >> 8) & 0xFFF
    D016 = ((tipX * 2864 >> 8) + (wh16 * 392 >> 8)) & 0x7FFF
    AX2 = (tipX * 338 >> 8) & 0x7FF
    return dict(h16=h16, Rv2=Rv2, tipX=tipX, m=m, wh16=wh16, bar16=bar16, D016=D016, AX2=AX2)


def eye_roms():
    w = [[], [], [], [], []]
    for i in range(256):
        e = eye(4 * i); mq = e['m'] - 79
        w[0].append(e['Rv2'] & 0xFFFF)
        w[1].append(((e['Rv2'] >> 16) << 12) | (e['AX2'] << 1) | (mq & 1))
        w[2].append((e['wh16'] << 3) | ((mq >> 1) & 7))
        w[3].append((e['bar16'] << 4) | (((mq >> 4) & 3) << 2))
        w[4].append(e['D016'] << 1)
    return w


RC16 = 19731
RC2 = (RC16 * RC16) >> 8


def write_pkg():
    L = ["-- Generated by vesica.py -- DO NOT EDIT.", "library ieee;", "use ieee.std_logic_1164.all;",
         "use ieee.numeric_std.all;", "", "package vesica_pkg is"]
    def arr(name, data, w, n):
        L.append("    type t_%s is array (0 to %d) of std_logic_vector(%d downto 0);" % (name.lower(), n - 1, w - 1))
        L.append("    constant %s : t_%s := (" % (name, name.lower()))
        for i in range(0, n, 8):
            L.append("        " + ", ".join(hexv(v, w) for v in data[i:i + 8]) + ("," if i + 8 < n else ""))
        L.append("    );")
    pal = pal_rom()
    arr("C_VES_PY", [p[0] for p in pal], 8, 512)
    arr("C_VES_PU", [p[1] for p in pal], 8, 512)
    arr("C_VES_PV", [p[2] for p in pal], 8, 512)
    arr("C_VES_PQ", pq_rom(), 10, 512)
    arr("C_VES_KNEE", knee12_rom(), 12, 256)
    arr("C_VES_GLT", glitch_rom(), 8, 1024)
    arr("C_VES_ROWS", rows_rom(), 12, 1024)
    arr("C_VES_CRES", [(y << 16) | (u << 8) | v for (y, u, v) in cres_rom()], 24, 128)
    arr("C_VES_KH", kh_rom(), 16, 256)
    arr("C_VES_IL", invl_rom(), 16, 256)
    arr("C_VES_K26", k26_rom(), 8, 512)
    for k, wv in enumerate(eye_roms()):
        arr("C_VES_EYE%d" % k, wv, 16, 256)
    L.append("    constant C_VES_RC2 : integer := %d;" % RC2)
    L.append("    type t_kq is array (0 to 63) of integer range 0 to 15;")
    L.append("    constant C_VES_KQ : t_kq := (" + ", ".join(str(v) for v in kq_rom()) + ");")
    for nm, k in (("TV_RIM", 29), ("TV_INN", 80), ("TV_HALO", 112), ("TV_T1", 128), ("TV_T2", 352)):
        L.append("    constant C_VES_%s : integer := %d;" % (nm, tv(k)))
    for nm, k in (("TC_D0", 640), ("TC_D1", 1200), ("TC_D2", 2112), ("TC_A1", 16), ("TC_A2", 29),
                  ("TC_T1", 56), ("TC_T2", 72), ("TC_E0", 114), ("TC_E1", 158)):
        L.append("    constant C_VES_%s : integer := %d;" % (nm, tc(k)))
    for k, c in COLS.items():
        y, u, v = yuv8(c)
        L.append("    constant C_VES_%s : std_logic_vector(23 downto 0) := %s;" % (k, hexv((y << 16) | (u << 8) | v, 24)))
    L.append("end package vesica_pkg;")
    with open(os.path.join(HERE, "vesica_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    write_pkg()
