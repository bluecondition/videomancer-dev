#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CREASE v0.1.1 (Straightedge) - build hook.  Writes crease_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.straightedge_work/rb_crease/imodel.py) reads the same tables.

The sheet is folded along three creases (normal angle A[k], distance C[k] from the pivot), fold 1 first.  With the
fold decisions (s1, s2, s3) = the region code, the folded point is M q + t, so the stripe phase of a region is an
affine function of the screen point and every crease test is a half-plane test with a constant unit normal.

Package contents:
  C_CR_EXP    256 x 16   round(32768 * 2^(i/256))                 (exponential stripe period, Drift speed)
  C_CR_SIN   1024 x 16   round(16384 * sin(i * pi / 2048))        (a quarter wave; the sequencer interpolates)
  C_CR_K       64 x 16   the fold constants, four words per row:
       rows 0 .. 7   region R (code = s3 s2 s1): the stripe direction is  cR + sg * theta  (65536 per turn)
                     word 0 = sg (bit 0 set: minus theta), word 1 = cR, words 2 / 3 = the fold offset t (x, y) in
                     virtual px, Q3, sign-magnitude (bit 15 = minus)
       rows 8 .. 14  crease test j (0: fold 1; 1, 2: fold 2 for s1 = 0, 1; 3 .. 6: fold 3 for s2 s1 = 00, 01, 10, 11):
                     word 0 = cos, word 1 = sin of the normal (Q12, signed), word 2 = its distance from the pivot
                     (virtual px, Q3, signed)
       row 15        the Pleat sheet's second region (reflection in the pleat crease through the pivot)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

TH0_DEG = -47.5                  # stripe gradient direction at Angle = 0 (y down); the slider adds a half turn
A_DEG = [-25.0, 50.0, 110.0]     # crease normals
C_PX = [-80.0, 120.0, 200.0]     # crease distances from the pivot, virtual px
AP_DEG = 90.0                    # Pleat: crease normal (horizontal creases)
PLEAT_N = [3, 2, 1]              # Pleat: stripe cycles per strip, Folds = 1, 2, 3
PLEAT_E0 = 3.0 / 16              # Pleat: the pivot sits 3/16 of a strip pair past a crease


def refl(n):
    """reflection matrix I - 2 n n^T"""
    return [[1 - 2 * n[0] * n[0], -2 * n[0] * n[1]], [-2 * n[0] * n[1], 1 - 2 * n[1] * n[1]]]


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(2)) for j in range(2)] for i in range(2)]


def mvec(a, v):
    return [a[0][0] * v[0] + a[0][1] * v[1], a[1][0] * v[0] + a[1][1] * v[1]]


def region(code, a_deg=A_DEG, c_px=C_PX):
    """folded point = M q + t for the fold decisions in code (bit k = fold k + 1 taken)"""
    M = [[1.0, 0.0], [0.0, 1.0]]
    t = [0.0, 0.0]
    for k in range(3):
        if (code >> k) & 1:
            n = [math.cos(math.radians(a_deg[k])), math.sin(math.radians(a_deg[k]))]
            H = refl(n)
            M = mmul(H, M)
            t = mvec(H, t)
            t = [t[0] + 2 * c_px[k] * n[0], t[1] + 2 * c_px[k] * n[1]]
    return M, t


def ang16(deg):
    return int(round(deg * 65536 / 360.0)) & 0xFFFF


def smag(v):
    """sign-magnitude, Q3"""
    m = int(round(abs(v) * 8))
    assert m < 32768
    return (0x8000 if v < 0 and m else 0) | m


def region_row(M, t):
    # direction of the phase gradient: M^T d(theta) = (cos(cR + sg theta), sin(..))
    e0 = [M[0][0], M[0][1]]                       # M^T (1, 0)
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    cr = math.degrees(math.atan2(e0[1], e0[0]))
    return [1 if det < 0 else 0, ang16(cr), smag(t[0]), smag(t[1])]


def test_row(M, t, k):
    n = [math.cos(math.radians(A_DEG[k])), math.sin(math.radians(A_DEG[k]))]
    npr = [M[0][0] * n[0] + M[1][0] * n[1], M[0][1] * n[0] + M[1][1] * n[1]]     # M^T n
    d = C_PX[k] - (n[0] * t[0] + n[1] * t[1])
    return [int(round(npr[0] * 4096)) & 0xFFFF, int(round(npr[1] * 4096)) & 0xFFFF, int(round(d * 8)) & 0xFFFF, 0]


def k_rom():
    rows = [region_row(*region(code)) for code in range(8)]
    rows.append(test_row(*region(0), 0))
    rows.append(test_row(*region(0), 1))
    rows.append(test_row(*region(1), 1))
    for code in range(4):
        rows.append(test_row(*region(code), 2))
    rows.append(region_row(*region(1, [AP_DEG, 0, 0], [0.0, 0, 0])))
    return [w for r in rows for w in r]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(16384 * math.sin(i * math.pi / 2048.0))) for i in range(1024)]


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v & ((1 << width) - 1)) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    parts = ["-- Generated by crease.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package crease_pkg is",
             "    constant C_CR_TH0 : unsigned(15 downto 0) := to_unsigned(%d, 16);" % ang16(TH0_DEG),
             vhd_table("C_CR_EXP", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_CR_SIN", "unsigned", 1024, 16, sin_rom()),
             vhd_table("C_CR_K", "unsigned", 64, 16, k_rom()),
             "end package crease_pkg;", ""]
    with open(os.path.join(HERE, "crease_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
