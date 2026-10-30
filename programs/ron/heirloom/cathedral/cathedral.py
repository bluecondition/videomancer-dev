#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CATHEDRAL v0.5 - build hook.  Generates cathedral_pkg.vhd (pure Python, no numpy).

All shapes and colours were fitted to the photo (WIN_20230602_01_30_58_Pro.jpg) in
.heirloom_work/rb_cathedral/ (fitF.py, fitR4.py, colprof.py, bandprof.py).

  C_G   512 x 16: g(x'), the horizontal sine-like shaper of the egg-crate field
        F = h(|Y|) - g(x'), x' = distance to the nearest saddle column (0 .. 470 of a
        940 px period), Q.11 F units.  Index = x' * 1024 / 940.
  C_H   256 x 16: h(|Y| * ky), the vertical shaper, Q.11.  Index = |Y| ky / 2.2.
  C_BC  256 x 16: teal-band colour (G | B, 8-bit RGB) vs |Y| / 4.
  C_GR  512 x 16: ground ramp G (warm / pink) | the pink layer's B base, vs Y / 4.
  C_CR  256 x 16: rising-edge curves aG | aB (1/16) vs dr + 16 (signed distance to a
        band's rising edge, -16 .. 47 px).
  C_CE  256 x 16: trailing / post curves vs e + 32 (signed distance to the band's
        true end, -32 .. 31): shG | shB inside (e <= 0), postG | postR after (e > 0).
  C_KT 1024 x 16: knob curves, 4 x 256 (index = knob / 4): lens-phase increment per
        Q.1 unit (K1), h-index scale KJ Q.15 (K2, x0.5 .. x4), spill Q.8 (K5),
        band-phase gain M Q.3 (P12).
  C_MR/C_MG/C_MB 256 x 32: RGB -> YUV contributions per 8-bit channel value:
        Y (Q.1, pre-boosted 16/13 for the output conditioning) | Cr | Cb (signed
        10-bit), hardware convention (U register = Cr, V register = Cb).
"""
import math, os

HERE = os.path.dirname(os.path.abspath(__file__))

# fitted shapers (rb_cathedral/fitF.py nonparametric fit, plateaus from the corner contours)
XK = [i * 10.0 for i in range(48)] + [530.0]
GV = [0.0, 0.0471, 0.0833, 0.1114, 0.135, 0.1618, 0.2007, 0.2447, 0.2851, 0.3171, 0.345, 0.3704,
      0.394, 0.4188, 0.4474, 0.4794, 0.513, 0.5428, 0.5657, 0.5835, 0.5998, 0.6156, 0.6301, 0.65,
      0.6775, 0.7056, 0.7314, 0.753, 0.7711, 0.7871, 0.8024, 0.8172, 0.8297, 0.84, 0.849, 0.8573,
      0.8643, 0.87, 0.875, 0.88, 0.885, 0.8895, 0.894, 0.8975, 0.901, 0.9013, 0.9017, 0.902, 0.902]
YK = [i * 10.0 for i in range(53)] + [565.0]
HV = [0.0, 0.0314, 0.0609, 0.0875, 0.113, 0.1412, 0.1738, 0.2104, 0.2468, 0.2791, 0.3045, 0.3257,
      0.3445, 0.3646, 0.3888, 0.4181, 0.4486, 0.478, 0.5041, 0.5272, 0.5489, 0.5693, 0.5887,
      0.6072, 0.6249, 0.642, 0.6587, 0.675, 0.6911, 0.7073, 0.7235, 0.74, 0.757, 0.7703, 0.7799,
      0.7885, 0.7965, 0.8039, 0.8108, 0.8173, 0.823, 0.8278, 0.8327, 0.8367, 0.84, 0.845, 0.85,
      0.8535, 0.857, 0.8615, 0.866, 0.8695, 0.873, 0.88]
# teal band colour vs |Y| (RGB G, B)
BY = [0, 40, 80, 120, 160, 200, 240, 280, 320, 360, 400, 440, 480, 520, 600]
BG = [82, 90, 98, 107, 112, 133, 147, 139, 109, 98, 93, 89, 82, 80, 80]
BB = [150, 141, 130, 124, 112, 92, 78, 88, 115, 134, 141, 146, 155, 158, 158]
GRAMP = [7.59556129e-11, 6.11078009e-08, -2.95285542e-04, -8.61855519e-02, 2.51082950e+02]


def interp(x, xs, ys):
    if x <= xs[0]: return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + (ys[i] - ys[i - 1]) * f
    return ys[-1]


def clip(v, a, b):
    return a if v < a else b if v > b else v


def smooth(t):
    t = clip(t, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def gramp(y):
    v = 0.0
    for c in GRAMP: v = v * y + c
    return clip(v, 0, 255)


def rom_g():
    return [int(round(interp((i + 0.5) * 940.0 / 1024.0, XK, GV) * 2048)) for i in range(512)]


def rom_h():
    return [int(round(interp((j + 0.5) * 2.2, YK, HV) * 2048)) for j in range(256)]


def rom_bc():
    out = []
    for k in range(256):
        ay = 4 * k + 2
        g = int(round(interp(ay, BY, BG))); b = int(round(interp(ay, BY, BB)))
        out.append((g << 8) | b)
    return out


def rom_gr():
    out = []
    for m in range(512):
        y = min(1079, 4 * m + 2)
        g = int(round(gramp(y))); bb = int(round(166 + 24 * y / 1080.0))
        out.append((g << 8) | bb)
    return out


def aG(dr):
    return interp(dr, [-9, -6, -3, 0, 3, 6, 9, 12], [0, .13, .38, .66, .86, .94, .98, 1])


def aB(dr):
    return interp(dr, [-14, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 21, 24, 30, 40],
                  [0, .05, .12, .19, .26, .39, .53, .64, .73, .79, .84, .88, .91, .93, .95, 1])


def rom_cr():
    out = []
    for i in range(256):
        dr = i - 16
        if i >= 64: dr = 47
        out.append((int(round(16 * aG(dr))) << 5) | int(round(16 * aB(dr))))
    return out


def rom_ce():
    out = []
    for i in range(256):
        e = i - 32 if i < 64 else 31
        if e <= 0:
            a = 0.48 + 0.52 * clip(-e / 30.0, 0, 1)
            b = clip((-e - 4) / 20.0, 0, 1) ** 0.9
        else:
            rp2 = smooth((e - 9) / 18.0)
            a = interp(e, [0, 2, 5, 9], [0.35, 0.47, 0.57, 0.57]) * (1 - rp2) + rp2
            b = smooth((e - 1) / 17.0)
        out.append((int(round(16 * a)) << 5) | int(round(16 * b)))
    return out


def rom_kt():
    t0, t1, t2, t3 = [], [], [], []
    for k in range(256):
        lens = 2.0 ** (2.6 * (k - 128) / 255.0)
        t0.append(int(round(2 ** 24 / 1880.0 * lens)))                   # phase inc per Q.1 unit
        ky = 2.0 ** (3.0 * k / 255.0 - 1.0)                              # x0.5 .. x4, x1 at k 85
        t1.append(min(65535, int(ky * 32768 / 2.2)))                      # floor: x1 = v0.5 bit-exact
        t2.append(int(round(256 * 2.0 ** (3.0 * (k - 128) / 255.0))))    # spill Q.8
        t3.append(int(round(256 / 0.4735 * 2.0 ** (4.0 * (k - 128) / 255.0))))  # M Q.3 = 8*32/P
    return t0 + t1 + t2 + t3


def rom_mat(ch):
    """RGB channel ch (0 R, 1 G, 2 B) -> (Y Q.1 boosted 16/13) << 21 | Cr10s << 11 | Cb10s << 1"""
    ky = (0.299, 0.587, 0.114)[ch] * 876 / 255.0
    cr = ((0.5, -0.418688, -0.081312)[ch]) * 896 / 255.0
    cb = ((-0.168736, -0.331264, 0.5)[ch]) * 896 / 255.0
    out = []
    for v in range(256):
        y = int(round(v * ky * 16 / 13 * 2)); r = int(round(v * cr)); b = int(round(v * cb))
        out.append((y << 21) | ((r & 1023) << 11) | ((b & 1023) << 1))
    return out


def vhdl_rom(name, typ, depth, width, vals):
    lines = [f"    type {typ} is array (0 to {depth - 1}) of std_logic_vector({width - 1} downto 0);",
             f"    constant {name} : {typ} := ("]
    body = []
    for i in range(depth):
        body.append(f'x"{vals[i]:0{width // 4}X}"')
    for i in range(0, depth, 8):
        lines.append("        " + ", ".join(body[i:i + 8]) + ("," if i + 8 < depth else ");"))
    return "\n".join(lines)


def main():
    parts = ["-- generated by cathedral.py -- do not edit", "library ieee;",
             "use ieee.std_logic_1164.all;", "", "package cathedral_pkg is"]
    parts.append(vhdl_rom("C_G", "t_g", 512, 16, rom_g()))
    parts.append(vhdl_rom("C_H", "t_h", 256, 16, rom_h()))
    parts.append(vhdl_rom("C_BC", "t_bc", 256, 16, rom_bc()))
    parts.append(vhdl_rom("C_GR", "t_gr", 512, 16, rom_gr()))
    parts.append(vhdl_rom("C_CR", "t_cr", 256, 16, rom_cr()))
    parts.append(vhdl_rom("C_CE", "t_ce", 256, 16, rom_ce()))
    parts.append(vhdl_rom("C_KT", "t_kt", 1024, 16, rom_kt()))
    parts.append(vhdl_rom("C_MR", "t_mr", 256, 32, rom_mat(0)))
    parts.append(vhdl_rom("C_MG", "t_mg", 256, 32, rom_mat(1)))
    parts.append(vhdl_rom("C_MB", "t_mb", 256, 32, rom_mat(2)))
    parts.append("end package cathedral_pkg;")
    with open(os.path.join(HERE, "cathedral_pkg.vhd"), "w") as f:
        f.write("\n".join(parts) + "\n")


if __name__ == "__main__":
    main()
