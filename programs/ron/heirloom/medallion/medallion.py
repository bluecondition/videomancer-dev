#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
MEDALLION v0.5.0 - build hook.  Writes medallion_pkg.vhd next to this file (pure python: the
builder runs it with the system python3).

Fitted to the photo WIN_20230305_23_51_38 (see .heirloom_work/rb_medallion/notes.md):

  w      = T(x) + V(y)                     the diamond field: a horizontal triangle (period 939 px,
                                           peak on the central fold x = 996.6, rounded peak, filleted
                                           minimum, held at its x = 57 value at the line start) plus a
                                           curved vertical ramp V = 0.9744 d + 0.000792 d^2, d = |y - 522.4|
  level  = min(RING(p), E(w))              RING: the ring waveform over two ring periods (measured code
                                           steps; plateaus clipped at level 7), p = (w - 83.5) / 261.15;
                                           E: the diamonds' outer envelope, one level per 17.63 w
  code   = floor(level)                    a 3-bit colourizer (8 flat levels: dark red .. yellow)
  colour = PAL8[code XOR 4*stripe]         the horizontal stripes flip the code's MSB (half a cycle)
  stripe = parity of the stripe phase psi(y) (measured boundaries, symmetric about y = 553)
  medallion: ellipse (980.34, 533.19, rx 459.11, ry 430.9) keying in a CMY/grey plaid tile
           (304.15 x 207.5 px, 8 equal row bands, measured column edges)

Hardware units: x / T / V / w in half virtual pixels ("h", 1920 virtual px = 3840 h), virtual lines Q8.

Package contents (all tables also read by imodel.py):
  C_MD_RING  1024 x 8   level * 32 (0..255) over two ring periods, index = p * 512
  C_MD_PSI   512 x 12   stripe phase psi(4 j) in Q7 half-cycles (signed), j = virtual line / 4
  C_MD_PAL   512 x 32   {Y, U, V, 00}: bank 0 = the 8 posterised levels (index >> 5), bank 1 = smooth
  C_MD_TILE  2048 x 4   plaid class (0..7) at row * 256 + col
  C_MD_TCOL  8 x 32     plaid class colours {Y, U, V, 00}
  C_MD_RQM   64 x 6     z^2 / 120 (rounded peak, z < 60 h);  C_MD_RQN 48 x 5: d^2 / 96 (filleted minimum)
Colours are in the hardware convention (U register = Cr, V register = Cb), luma pre-scaled 16/13 for the
shell's 13/16 output scale and capped so drawn whites come out at ~777.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (virtual px / lines)
XF, XH = 996.57, 57.23            # central fold (triangle peak), line-start hold point
PX_DEF = 939.34                   # triangle period at the default (2.044 diamonds per 1920)
RHO_MAX, RHO_MIN = 30.0, 24.0     # peak rounding, minimum fillet
YC, B1, C1 = 522.4, 0.9744, 0.000792
W_P0, W_P2 = 83.5, 605.8          # ring phase: plateau centres of ring 1 / ring 3 at p = 0 / 2
EW0, ESTEP = 645.1, 17.63         # envelope: level 7 at w = EW0, one level per ESTEP
# 3-bit code steps along w in the photo's A stripes (w, level reached), plateaus (8.2 = clipped) and
# trough bottoms; the ring LUT interpolates these
RING_ANCH = [(0.0, 6.6), (24.0, 6.85), (32.3, 7), (60, 8.2), (108, 8.2), (134.9, 7), (155.0, 6), (177.8, 5),
             (198.0, 4), (222.2, 3), (230, 2.55), (237.6, 3), (260.4, 4), (281.1, 5), (300.0, 6), (320.1, 7),
             (330, 8.2), (395, 8.2), (407.5, 7), (425.5, 6), (445.0, 5), (464.6, 4), (472, 3.55), (503, 3.55),
             (510.9, 4), (529.5, 5), (547.5, 6), (566.5, 7), (580, 8.2), (630, 8.2), (645.1, 7)]
# the 8 colourizer levels (photo RGB)
PAL8 = [(143, 34, 8), (186, 33, 10), (223, 63, 4), (254, 63, 1), (254, 97, 1), (254, 138, 1), (254, 178, 1),
        (254, 236, 0)]
# stripe boundaries (first line of the next stripe is the boundary + 1; phase k + 0.5 at boundary + 0.5)
BOUND = [108, 178, 250, 304, 350, 391, 439, 487, 533, 577, 623, 667, 707, 749, 800, 864, 928, 998]
SYC, SA, SR = 553.383, 9.789, 424.383      # sine model of the stripe phase (used outside the boundaries)
# medallion
CX, CY, RX, RY = 980.34, 533.19, 459.11, 430.9
# plaid tile
PH, PV, PX0, PY0 = 304.15, 207.5, 690.0, 134.5
TC = {'W': (253, 252, 251), 'C': (5, 254, 254), 'M': (254, 23, 255), 'Y': (254, 254, 3), 'K': (16, 41, 20),
      'd': (50, 74, 58), 'g': (128, 143, 140), 'L': (201, 205, 214)}
TCLS = 'WCMYKdgL'
TROWS = [
    [(0, 'd'), (91.3, 'K'), (156, 'g'), (168.4, 'L'), (203.7, 'g')],
    [(0, 'K'), (156, 'g')],
    [(0, 'C'), (91.3, 'W'), (151.0, 'C'), (154.5, 'W'), (158.4, 'M'), (170.7, 'Y'), (202.7, 'M')],
    [(0, 'C'), (91.3, 'W'), (151.0, 'C'), (154.5, 'W'), (158.4, 'M'), (170.7, 'Y'), (202.7, 'M')],
    [(0, 'C'), (91.3, 'W'), (151.0, 'C'), (154.5, 'W'), (158.4, 'M'), (170.7, 'Y'), (202.7, 'M')],
    [(0, 'W'), (158.4, 'M')],
    [(0, 'C'), (91.3, 'W'), (151.0, 'C'), (154.0, 'W'), (168.2, 'C'), (200.1, 'W')],
    [(0, 'W')],
]

# ------------------------------------------------------------------ hardware integer constants
XF2, XH2 = round(2 * XF), 114                 # 1993, 114 (h)
RQM_N, RQN_N = 60, 48                         # 2 * RHO_MAX, 2 * RHO_MIN (h)
YC8 = round(YC * 256)                         # 133734
V_K1, V_K2 = round(0.5 * B1 * 1024), round(2 * C1 / 16 * 2 ** 20)   # V2 = d_q2*K1>>10 + d_q2^2*K2>>20
WP0_2 = round(2 * W_P0)                       # 167
KG_BASE = 256 * 512 * 2 / (2 * (W_P2 - W_P0)) # 250.98: ring index (1/512 period) per h, Q8
CX2 = round(2 * CX)                           # 1961
CY8 = round(CY * 256)                         # 136497
HW_K = round(RX / RY * 1024)                  # 1091
TK = round(2 ** 16 * 256 / (2 * PH))          # 27581: plaid column (Q16) per h
PX0_2 = round(2 * PX0)                        # 1380
PY08 = round(PY0 * 256)                       # 34432
RK = round(8 / PV * 2 ** 16)                  # 2527: plaid row (Q24) per virtual line Q8


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(len(xs) - 1):
        if x <= xs[i + 1]:
            t = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + t * (ys[i + 1] - ys[i])
    return ys[-1]


def pw(w):
    return (w - W_P0) * (2.0 / (W_P2 - W_P0))


def ring_levels(n=1024):
    """level over two ring periods (float), the hardware LUT before quantisation"""
    pts = [(pw(w), l) for w, l in RING_ANCH if 0 <= pw(w) < 2]
    pts = [(p - 2, l) for p, l in pts] + pts + [(p + 2, l) for p, l in pts]
    pts.sort()
    xs = [p for p, _ in pts]; ys = [l for _, l in pts]
    return [interp((i + 0.5) / n * 2, xs, ys) for i in range(n)]


def ring_rom():
    return [max(0, min(255, int(math.floor(l * 32)))) for l in ring_levels()]


def psi(y):
    """stripe phase (half-cycles) at virtual line y (line centre)"""
    by = [b + 0.5 for b in BOUND]
    ph = [k - 8.5 for k in range(9)] + [k + 0.5 for k in range(9)]
    if y < by[0]:
        return ph[0] + SA * (math.sin((y - SYC) / SR) - math.sin((by[0] - SYC) / SR))
    if y > by[-1]:
        return ph[-1] + SA * (math.sin((y - SYC) / SR) - math.sin((by[-1] - SYC) / SR))
    return interp(y, by, ph)


def psi_rom():
    return [max(-2047, min(2047, round(psi(4 * j) * 128))) for j in range(512)]


def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (round(64 + y * 876), round(512 + cr * 896), round(512 + cb * 896))


def hw_word(rgb):
    """{Y, U(=Cr), V(=Cb), 00} with luma pre-scaled 16/13 and capped at 940 (-> ~777 after the 13/16 scale)"""
    y, u, v = rgb_to_hw10(rgb)
    ys = min(940, 64 + round((y - 64) * 16 / 13))
    u = max(1, min(1023, u)); v = max(1, min(1023, v))
    return (ys << 22) | (u << 12) | (v << 2)


def pal_rom():
    words = []
    for i in range(256):
        words.append(hw_word(PAL8[i >> 5]))
    for i in range(256):
        k, f = i >> 5, (i & 31) / 32.0
        if k == 7:
            c = PAL8[7]
        else:
            c = tuple(PAL8[k][j] * (1 - f) + PAL8[k + 1][j] * f for j in range(3))
        words.append(hw_word(c))
    return words


def tile_rom():
    out = []
    for r in range(8):
        row = TROWS[r]
        for c in range(256):
            x = (c + 0.5) * PH / 256
            k = [e[1] for e in row if e[0] <= x][-1]
            out.append(TCLS.index(k))
    return out


def write_pkg():
    ring, psit, pal, tile = ring_rom(), psi_rom(), pal_rom(), tile_rom()
    tcol = [hw_word(TC[c]) for c in TCLS]
    rqm = [min(63, round(z * z / 120.0)) for z in range(64)]
    rqn = [min(31, round(d * d / 96.0)) for d in range(64)]
    L = ["-- Generated by medallion.py -- DO NOT EDIT.", "library ieee;", "use ieee.std_logic_1164.all;",
         "use ieee.numeric_std.all;", "", "package medallion_pkg is"]

    def arr(name, typ, vals, fmt, per=8):
        L.append(f"    constant {name} : {typ} := (")
        rows = [", ".join(fmt(v) for v in vals[i:i + per]) for i in range(0, len(vals), per)]
        L.append(",\n".join("        " + r for r in rows))
        L.append("    );")
    L.append("    type t_md_ring is array (0 to 1023) of unsigned(7 downto 0);")
    arr("C_MD_RING", "t_md_ring", ring, lambda v: f'x"{v:02X}"', 16)
    L.append("    type t_md_psi is array (0 to 511) of signed(11 downto 0);")
    arr("C_MD_PSI", "t_md_psi", psit, lambda v: f'to_signed({v}, 12)', 8)
    L.append("    type t_md_pal is array (0 to 511) of std_logic_vector(31 downto 0);")
    arr("C_MD_PAL", "t_md_pal", pal, lambda v: f'x"{v:08X}"', 8)
    L.append("    type t_md_tile is array (0 to 2047) of unsigned(3 downto 0);")
    arr("C_MD_TILE", "t_md_tile", tile, lambda v: f'x"{v:X}"', 32)
    L.append("    type t_md_tcol is array (0 to 7) of std_logic_vector(31 downto 0);")
    arr("C_MD_TCOL", "t_md_tcol", tcol, lambda v: f'x"{v:08X}"', 8)
    L.append("    type t_md_rqm is array (0 to 63) of unsigned(5 downto 0);")
    arr("C_MD_RQM", "t_md_rqm", rqm, lambda v: f'to_unsigned({v}, 6)', 16)
    L.append("    type t_md_rqn is array (0 to 63) of unsigned(4 downto 0);")
    arr("C_MD_RQN", "t_md_rqn", rqn, lambda v: f'to_unsigned({v}, 5)', 16)
    L.append("end package medallion_pkg;")
    with open(os.path.join(HERE, "medallion_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    write_pkg()
