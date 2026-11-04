#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CHALICE v0.1.0 - build hook.  Writes chalice_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  Heirloom from the video clips; still WIN_20230305_00_05_43 t0.5 (clip static).

The patch, as fitted (.heirloom_work/rb_chalice/notes.md): one field phi (ring units, 1 = one B-channel
period; higher phi = lower field) drives three waveshapers:
  phi_raw = ramp(y) + S(y) (1 - |u|/U0)^2  -  K(y) h(|u|/w(y))  +  Kd(y) dimple(|u|/w(y))  + camera luma
  phi     = -3 + |phi_raw + 3|                                   (a wavefolder at -3: the rim ring)
  R = clamp(255 - 58 |phi - 1.525|)           a slow triangle (peak = the orange band)
  G = clamp(128 - 42 (phi - 1.2), 0, 128)     a clamp, delayed 40 virtual px (the analog channel delay)
  B = clamp(2 tri(phi_B) - 1/2)               a fast triangle, peaks at integer phi, 50 % dead zones;
                                              phi_B has the ramp clipped at -0.5 and the total at 5.5
  ramp(y): the vertical ramp, -0.62 (olive plateau) to y = 220, then 0.0084 / 0.0114 / 0.0214 per line (knees at
  455 and 685), 6 = black (y = 780).
  K, w, S, Kd: per-line piecewise-linear tables (below) in virtual 1080-line coordinates; u in virtual px
  from the axis (the screen centre); U0 = 1000 virtual px.

Fixed-point units used by the RTL and imodel.py:
  phi, ramp, S   : 1/256 ring (signed 14 bits);  K : 1/64 ring;  Kd : 1/128 ring;  w : 1/16 virtual px
  table slopes   : units per virtual line, Q8 (value = start + (dy_q8 * slope) >> 16, dy_q8 = 256 * lines)
  profile ROM    : 1024 entries by v = |u|/w in 1/256 (v 0..4): bits 19..8 = h * 4095, bits 7..0 = dimple * 255
  exp2 ROM       : 256 x 16, round(32768 * 2^(i/256)) (knob curves)
Colours are made in RGB (8 bits) and converted to the hardware convention (U register = Cr, V = Cb) by
the matrix in the RTL (luma pre-scaled 16/13 for the shell's 13/16 output scale, capped at 940).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ fitted model (rings, virtual px / lines)
AXIS_VPX = 960.0          # the program centres the chalice (the still has it at 983)
U0_VPX = 1000.0
FOLD = -3.0
RAMP_PLATEAU, RAMP_BLACK, RAMP_BCLIP, BMAX = -0.62, 6.0, -0.5, 5.5
R_C, R_K, G_C, G_K = 1.525, 58.0, 1.2, 42.0
GSHIFT_VPX = 40

# piecewise-linear tables: (y_start, value_at_start, slope per line); the last segment runs to 1080
RAMP_SEG = [(0, -0.62, 0.0), (220, -0.62, 0.00838), (455, 1.35, 0.01139), (685, 3.97, 0.02137)]
K_SEG = [(0, 10.4, -0.00667), (480, 7.2, -0.02412), (650, 3.1, 0.025), (790, 6.6, -0.00931)]
W_SEG = [(0, 345.0, -0.28125), (480, 210.0, -0.62727), (700, 72.0, 0.8267), (1000, 320.0, 0.3125)]
S_SEG = [(0, 0.3, 0.0), (100, 0.3, 0.009), (300, 2.1, 0.00111), (480, 2.3, 0.0005)]
KD_SEG = [(0, 2.0, -0.005267), (300, 0.42, -0.0014), (600, 0.0, 0.0)]

# profile h(v): control points (v, h), smoothed; dimple(v) = exp(-(v / 0.265)^2)
H_PTS = [(0, 1.0), (0.15, 0.99), (0.3, 0.95), (0.47, 0.88), (0.6, 0.82), (0.73, 0.72), (0.87, 0.61), (1.0, 0.48),
         (1.07, 0.36), (1.13, 0.22), (1.2, 0.10), (1.27, 0.04), (1.33, 0.0), (8.0, 0.0)]
H_SMOOTH = 0.08
DIM_W = 0.265

# RGB -> hardware YUV matrix, /256 (luma 16/13 pre-scaled; Y = 64 + ..., Cr/Cb = 512 + ...)
MAT_Y = (324, 635, 123)
MAT_CR = (450, -377, -73)
MAT_CB = (-152, -298, 450)
Y_CAP = 940


def interp(pts, x):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def profile_tables():
    """h (0..4095) and dimple (0..255) by v = i / 256, i = 0..1023."""
    step = 0.001
    n = int(8.0 / step)
    raw = [interp(H_PTS, i * step) for i in range(n)]
    hw = int(H_SMOOTH / step)
    kern = [math.exp(-0.5 * ((k - hw) * step / (H_SMOOTH / 2)) ** 2) for k in range(2 * hw + 1)]
    ks = sum(kern)
    sm = []
    for i in range(n):
        acc = 0.0
        for k in range(2 * hw + 1):
            j = min(max(i + k - hw, 0), n - 1)
            acc += raw[j] * kern[k]
        sm.append(acc / ks)
    h = []
    d = []
    for i in range(1024):
        v = i / 256.0
        j = min(int(round(v / step)), n - 1)
        h.append(max(0, min(4095, int(round(sm[j] * 4095)))))
        d.append(max(0, min(255, int(round(math.exp(-(v / DIM_W) ** 2) * 255)))))
    return h, d


def seg_rtl(segs, unit, slope_scale):
    """(y_start, start in RTL units, |slope| Q8-per-line in RTL units, sign)."""
    out = []
    for y0, v0, sl in segs:
        s = sl * unit * 256
        out.append((y0, int(round(v0 * unit)), int(round(abs(s))), -1 if s < 0 else 1))
    return out


RAMP_RTL = seg_rtl(RAMP_SEG, 256, 1)
K_RTL = seg_rtl(K_SEG, 64, 1)
W_RTL = seg_rtl(W_SEG, 16, 1)
S_RTL = seg_rtl(S_SEG, 256, 1)
KD_RTL = seg_rtl(KD_SEG, 128, 1)


def main():
    h, d = profile_tables()
    exp2 = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    lines = []
    lines.append("-- Generated by chalice.py: do not edit.  Profile ROM (h / dimple by v = |u| / w) and exp2.")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("use ieee.numeric_std.all;")
    lines.append("")
    lines.append("package chalice_pkg is")
    lines.append("    type t_ch_prof is array (0 to 1023) of std_logic_vector(19 downto 0);")
    lines.append("    constant C_CH_PROF : t_ch_prof := (")
    for i in range(0, 1024, 8):
        row = ", ".join('x"%05x"' % ((h[j] << 8) | d[j]) for j in range(i, i + 8))
        lines.append("        " + row + ("," if i + 8 < 1024 else ""))
    lines.append("    );")
    lines.append("    type t_ch_exp2 is array (0 to 255) of unsigned(15 downto 0);")
    lines.append("    constant C_CH_EXP2 : t_ch_exp2 := (")
    for i in range(0, 256, 8):
        row = ", ".join('x"%04x"' % min(exp2[j], 65535) for j in range(i, i + 8))
        lines.append("        " + row + ("," if i + 8 < 256 else ""))
    lines.append("    );")
    lines.append("end package chalice_pkg;")
    with open(os.path.join(HERE, "chalice_pkg.vhd"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("chalice_pkg.vhd written (profile 1024 x 20, exp2 256 x 16)")
    for name, t in (("RAMP", RAMP_RTL), ("K", K_RTL), ("W", W_RTL), ("S", S_RTL), ("KD", KD_RTL)):
        print(name, t)


if __name__ == "__main__":
    main()
