#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PLATTER v0.1.0 - build hook.  Writes platter_pkg.vhd next to this file (pure python, system python3).
Heirloom from the video clips; still WIN_20231204_21_01_30 t0.5 (.heirloom_work/rb_platter/notes.md).

ROMs written (luma pre-scaled 16/13 for the output stage's 13/16, capped at 768 after it):
  C_PL_PROF  512 x 32  tube colour profile by s (px from the tube's inner rim, outward; 256 entries over the 1300-px
                       period, entry i = s (i + 0.5) * 1300/256 - 400): Rose 0..255, Neon 256..511.
                       Y10 << 20 | U10 << 10 | V10 (hardware convention, U = Cr)
  C_PL_WT    512 x 32  the measured per-line tube warps (sign-magnitude, entry i = virtual line 32 i / 15, rows 0 .. 505):
                       D0 (left zone, base 707 px: sign, 6-bit |D0|) << 25 | D1 (centre zone, base 707 px: sign, 8-bit
                       |D1|) << 16 | D2 / 2 (right zone, base 1600 px: sign, 9-bit |D2| / 2) << 6 | G2 << 5, where G2
                       marks the right zone's steep rows (the pipe-end step and the rows before it where |D2| moves more
                       than 6 px a row): the RTL grounds the right zone there (no single-row sweep line)
  C_PL_SEQ  1024 x 16  0..255: sine Q15 (one turn), 256..511: round(32768 * 2^(i/256)),
                       512..1023: round(32767 sqrt(1 - (i - 512 + 0.5) / 512))  (the disc chord by v = (Y/b)^2)
  C_PL_COS   256 x 16  the disc's colour oscillator: Cb = round(500 - 85 cos(2 pi i/256))
  C_PL_LUM   256 x 16  the disc's luma curve: noise 0..255 -> Y (the still's texture percentiles)
  C_PL_BALL  256 x 16  per ball row |dy| (bases 0 / 64 / 96 / 128 / 160 for B1 / B2 / B3 / B4 / hole):
                       half-width in virtual px << 8 | luma drop
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

WT_HEX = ["01301301301401401401401401401401401501501501501601601601701701801801901901a01a01b01b01c01c01d01e01e01f02002002102202202302302402502502602602702702802902902a02a02b02b02c02c02c02d02d02e02e02f02f02f03003003003103103103203203203303303303303403403403403503503503503503603603603603603603603603603703703703703703703703703703703703703703703703703703703703603603603603603603603603603603503503503503503403403403403403303303303303203203203203103103103003003002f02f02f02e02e02e02d02d02d02c02c02b02b02b02a02a02902902802802702702702602602502502402402302302202202102002001f01f01e01e01d01d01c01b01b01a01a01901901801701701601601501401401301201201101001000f00e00e00d00c00c00b00a009009008007006005005004003002002001000ffffffffeffdffcffcffbffaff9ff9ff8ff7ff7ff6ff5ff5ff4ff3ff3ff2ff2ff1ff1ff0ff0feffeffeefeefedfedfedfecfecfebfebfeafeafeafe9fe9fe8fe8fe7fe7fe6fe6fe5fe5fe4fe4fe3fe3fe2fe2fe1fe1fe0fe0fdffdffdefdefddfddfdcfdcfdcfdbfdbfdafdafdafd9fd9fd8fd8fd8fd7fd7fd7fd6fd6fd6fd5fd5fd5fd4fd4fd4fd3fd3fd3fd2fd2fd2fd1fd1fd1fd0fd0fd0fcffcffcffcefcefcefcefcdfcdfcdfcdfccfccfccfccfccfcbfcbfcbfcbfcbfcbfcafcafcafcafcafcafcafcafcafc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fc9fcafcafcafcafcafcafcafcafcafcbfcbfcbfcbfcbfcbfccfccfccfccfcdfcdfcdfcefcefcefcffcffd0fd0fd1fd1fd2fd2fd3fd4fd4fd5fd6fd6fd7fd8fd8fd9fdafdbfdbfdcfddfdefdffdffe0fe1fe2fe3fe3fe4fe5fe6fe6fe7fe8fe8fe9fe9feafeafebfebfecfecfedfedfedfeefeefeefeefeefeffeffeffeffeffeffefff0ff0ff0ff0ff0ff0ff0ff1ff1ff1ff1ff1ff2ff2ff2ff2ff3ff3ff3ff3ff4ff4ff4ff5ff5ff5ff6ff6ff6ff6ff7ff7ff7ff7ff7ff7ff7ff7ff7", "0de0dc0da0d80d50d20cf0cc0c90c50c20be0ba0b70b30b00ac0a80a50a109d09a09609308f08b08808408007d07907607206e06b06706306005c05905505104e04a04604303f03c03803403102d02a02602201f01b01801401100e00a007004001ffeffbff8ff6ff3ff0feefecfeafe8fe6fe4fe2fe1fe0fdefddfdcfdcfdbfdbfdafdafdafdafdafdbfdbfdcfdcfddfdefe0fe1fe2fe4fe5fe7fe9febfedfefff1ff3ff5ff8ffaffdfff00200400700900c00f01101401601801b01d01f02102302502702802a02b02c02e02e02f03003003103103103103103103103003003002f02f02f02e02e02e02d02d02c02c02c02b02b02a02a02902902802802802702702602602502502402402402302302202102102002001f01e01e01d01c01c01b01b01a01a01901801801801701701601501501401401301201201101001000f00e00d00c00c00b00a009009008007006005005004003002002001000ffffffffeffdffcffcffbffaff9ff8ff8ff7ff6ff5ff5ff4ff3ff2ff2ff1ff0ff0feffeffeefeefeefedfedfedfedfecfecfecfecfebfebfebfeafeafeafe9fe9fe8fe7fe7fe6fe6fe5fe4fe4fe3fe3fe2fe1fe1fe0fe0fdffdffdefdefddfddfddfdcfdcfdbfdbfdbfdafdafd9fd9fd9fd8fd8fd8fd7fd7fd7fd6fd6fd6fd5fd5fd5fd4fd4fd4fd3fd3fd2fd2fd2fd1fd1fd1fd0fd0fd0fcffcffcffcefcefcefcefcdfcdfcdfcdfccfccfccfccfccfcbfcbfcbfcbfcbfcafcafcafcafcafc9fc9fc9fc8fc8fc8fc7fc6fc5fc4fc3fc2fc0fbefbcfbafb8fb5fb3fb0fadfaafa6fa3fa0f9cf98f95f91f8df89f85f81f7df79f75f71f6df69f64f60f5df59f55f51f4df49f46f42f3ff3bf38f35f31f2ef2cf29f26f23f21f1ff1df1bf19f17f15f14f12f11f10f0ff0ef0df0df0df0cf0cf0cf0cf0df0df0ef0ff10f11f13f14f16f17f19f1bf1df1ff21f24f26f28f2af2df2ff31f34f36f39f3bf3df40f42f44f47f49f4bf4ef50f52f55f57f59f5cf5ef60f63f65f67f6af6cf6ef71f73f76f78f7af7df7ff81f84f86f88f8af8cf8ff90f92f94f95f97f98f98f98f98f98f98", "0ba0ba0ba0ba0ba0ba0ba0ba0ba0b90b90b90b80b70b60b50b30b00ae0aa0a70a209e09909408e08808307d07707206d06806406005c05905705405205004e04d04b04904704504304103f03c03a03703503202f02c02a02702402101f01c01901601301000c009006003000ffdffaff7ff5ff2ff0feefecfeafe9fe7fe6fe5fe4fe3fe2fe1fe0fe0fdffdefdefddfdcfdcfdbfdbfdafdafd9fd9fd9fd9fd8fd8fd8fd8fd8fd8fd8fd8fd9fd9fd9fd9fdafdafdafdbfdbfdcfdcfddfdefdefdffe0fe1fe2fe2fe3fe4fe5fe6fe8fe9feafebfecfedfefff0ff1ff3ff4ff6ff7ff9ffbffcffe00000200400600800a00c00e01001301501701901b01e02002202502702a02c02e03103403603903c03e04104404704a04d05005305605905c05f06206606a06d07207607b08008508a08f09409a0a10aa0b50c30d30e70fe1171341521721931b5195e21e2ee3ce4ce5de6ee81e94ea7ebaeccedeef0f00f0ff1df29f34f3ef47f4ef54f59f5df60f64f67f6af6df70f73f76f79f7cf7ff83f86f89f8df90f94f97f9bf9efa2fa6faafaefb1fb5fb9fbdfc1fc5fc8fccfcffd3fd6fd9fdcfdffe2fe4fe7fe9fecfeeff1ff3ff6ff8ffbffd00000200500700a00c00e01001301501701901b01d01e02002202402502702902a02c02d02f03003203303503603703903a03b03c03e03f04004104204304404504604704704804904a04a04b04c04d04d04e04e04f04f05005005105105205205205305305305405405405405405505505505505505505605605605605605605605605605605505505505505505405405405305305205205205105105005004f04f04e04e04d04d04c04b04b04a04a04904804804704604504404404304204104003f03e03d03c03b03a03903703603503403203103002e02d02c02a02902702602402302102001e01c01b01901801601401301100f00d00b009007005003001fffffdffbff9ff7ff5ff2ff0feefecfeafe7fe5fe3fe0fdefdbfd9fd6fd3fd1fcefcbfc8fc5fc2fbffbcfb9fb6fb3fb0fadfabfa8fa6fa4fa2fa1f9ff9ef9df9cf9cf9cf9cf9cf9c"]

# tube colour profile by s (measured on the left tube, rim sharpened; the pink "ghost" tube 450..570 px out)
PS = [-400, -60, -36, -24, -12, -6, -2, 3, 8, 14, 24, 36, 48, 60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192,
      204, 216, 240, 270, 300, 420, 450, 480, 510, 540, 570, 600, 900]
PY = [73, 78, 84, 95, 130, 260, 440, 440, 230, 200, 330, 437, 515, 581, 634, 665, 657, 610, 552, 492, 427, 365, 315,
      244, 169, 133, 113, 89, 76, 73, 76, 200, 420, 480, 380, 200, 80, 73]
PU = [520, 526, 540, 555, 575, 590, 600, 600, 605, 605, 611, 611, 608, 603, 599, 596, 596, 598, 599, 598, 593, 586,
      578, 575, 568, 557, 547, 537, 524, 520, 525, 640, 690, 700, 690, 640, 530, 520]
PV = [513, 511, 508, 503, 490, 470, 462, 462, 467, 465, 460, 455, 448, 438, 428, 416, 404, 390, 372, 355, 350, 359,
      379, 413, 453, 475, 487, 502, 511, 513, 513, 520, 530, 540, 530, 520, 513, 513]
# disc luma curve: the noise quantiles -> the still's texture percentiles (dark end lowered for the output blur)
NQ = [0.0, 0.071, 0.173, 0.353, 0.498, 0.643, 0.824, 0.925, 1.0]
YQ = [72, 84, 135, 192, 285, 420, 575, 645, 700]
# balls: (rx, ry, luma drop at the top/bottom row)
BALLS = [(77, 58, 85), (31, 22, 60), (35, 24, 80), (26, 18, 13), (32, 22, 0)]
BASES = [0, 64, 96, 128, 160]


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for k in range(1, len(xs)):
        if x <= xs[k]:
            t = (x - xs[k - 1]) / (xs[k] - xs[k - 1])
            return ys[k - 1] + (ys[k] - ys[k - 1]) * t
    return ys[-1]


def ysc(y):
    """luma pre-scaled 16/13 (the output takes 13/16), capped at 768 after the scale"""
    v = 64 + (y - 64) * 16.0 / 13.0
    return int(round(min(max(v, 64), 64 + (768 - 64) * 16.0 / 13.0)))


def neon(u, v):
    return 1024 - u, v + (u - 512) * 0.6


def clamp10(v):
    return max(0, min(1023, int(round(v))))


def tables():
    prof = []
    for bank in range(2):
        for i in range(256):
            s = (i + 0.5) * 1300.0 / 256.0 - 400.0
            y, u, v = interp(s, PS, PY), interp(s, PS, PU), interp(s, PS, PV)
            if bank:
                u, v = neon(u, v)
            prof.append((ysc(y) << 20) | (clamp10(u) << 10) | clamp10(v))
    wt = []
    def sx(v):
        return v - 4096 if v & 0x800 else v
    cols = [[sx(int(h[3 * i:3 * i + 3], 16)) for i in range(len(h) // 3)] for h in WT_HEX]
    # the right zone's steep rows: the pipe-end step row and the rows before it where |D2| moves more than 6 px a row
    dd = [abs(cols[2][i] - cols[2][i - 1]) for i in range(1, 512)]
    stp = 1 + max(range(len(dd)), key=lambda k: dd[k])
    G2ROWS = set(i for i in range(stp - 30, stp + 1) if abs(cols[2][i] - cols[2][i - 1]) > 6) | set(range(stp, stp + 13))
    for i in range(512):
        d0, d1, d2 = cols[0][i], cols[1][i], int(round(cols[2][i] / 2.0))
        def sm(v, bits):
            return ((1 << (bits - 1)) if v < 0 else 0) | min(abs(v), (1 << (bits - 1)) - 1)
        g2 = 1 if i in G2ROWS else 0
        wt.append((sm(d0, 7) << 25) | (sm(d1, 9) << 16) | (sm(d2, 10) << 6) | (g2 << 5))
    seq = [int(round(32767 * math.sin(2 * math.pi * i / 256))) & 0xFFFF for i in range(256)]
    seq += [min(65535, int(round(32768 * 2 ** (i / 256.0)))) for i in range(256)]
    seq += [int(round(32767 * math.sqrt(1 - (i + 0.5) / 512.0))) for i in range(512)]
    cos = [int(round(500 - 85 * math.cos(2 * math.pi * i / 256))) for i in range(256)]
    lum = [ysc(interp((i + 0.5) / 256.0, NQ, YQ)) for i in range(256)]
    ball = [0] * 256
    for (rx, ry, drop), b in zip(BALLS, BASES):
        for d in range(ry):
            hw = int(round(rx * math.sqrt(max(0.0, 1 - ((d + 0.5) / ry) ** 2))))
            dl = int(round(drop * ((d + 0.5) / ry) ** 2 * 16 / 13))
            ball[b + d] = (hw << 8) | dl
    return prof, wt, seq, cos, lum, ball


def rom(lines, name, typ, n, width, vals):
    hexw = (width + 3) // 4
    lines.append("    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % (typ, n - 1, width - 1))
    lines.append("    constant %s : %s := (" % (name, typ))
    for i in range(0, n, 8):
        row = ", ".join('x"%0*x"' % (hexw, vals[j]) for j in range(i, i + 8))
        lines.append("        " + row + ("," if i + 8 < n else ""))
    lines.append("    );")


def main():
    prof, wt, seq, cos, lum, ball = tables()
    L = ["-- Generated by platter.py: do not edit.  Tube profile, warp tables, sine/exp2, disc colour and luma curves,",
         "-- ball rows.", "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
         "package platter_pkg is"]
    rom(L, "C_PL_PROF", "t_pl_prof", 512, 32, prof)
    rom(L, "C_PL_WT", "t_pl_wt", 512, 32, wt)
    rom(L, "C_PL_SEQ", "t_pl_seq", 1024, 16, seq)
    rom(L, "C_PL_COS", "t_pl_cos", 256, 16, cos)
    rom(L, "C_PL_LUM", "t_pl_lum", 256, 16, lum)
    rom(L, "C_PL_BALL", "t_pl_ball", 256, 16, ball)
    L.append("end package platter_pkg;")
    with open(os.path.join(HERE, "platter_pkg.vhd"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("platter_pkg.vhd written (PROF 512x32, WT 512x32, SEQ 1024x16, COS/LUM/BALL 256x16)")


if __name__ == "__main__":
    main()
