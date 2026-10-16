#!/usr/bin/env python3
"""Tonecurve curve ROM: 8 curves x 128 point samples at luma codes 8*i.

The builder in tonecurve.vhd interpolates linearly between samples i and
i+1, so sample i is the curve's value AT code 8*i (v0.1.0 stored bin centres,
8*i+4, which lifted gamma 0.5's black to 123).  Order is the K1 sweep:
bright -> dark -> flat -> contrasty -> crushed.

  python3 gen_curves.py            print the VHDL constant
  python3 gen_curves.py --check    compare against the v0.1.0 table
"""
import math, sys

BLACK, SPAN = 64, 876          # 64 .. 940

def g05(t): return t ** 0.5
def g07(t): return t ** 0.7
def g15(t): return t ** 1.5
def g22(t): return t ** 2.2
def inv_s(t): return math.acos(1 - 2 * t) / math.pi
def soft_s(t): return (1 - math.cos(math.pi * t)) / 2
def hard_s(t): return 1 / (1 + math.exp(-10 * (t - 0.5)))   # 70 .. 934, as v0.1
def crush(t): return min(1.0, max(0.0, 2 * t - 0.5))        # 2x about mid grey

CURVES = [("gamma 0.5", g05), ("gamma 0.7", g07), ("gamma 1.5", g15), ("gamma 2.2", g22),
          ("inverse S", inv_s), ("soft S", soft_s), ("hard S", hard_s), ("crush", crush)]

def value(f, code):
    t = min(1.0, max(0.0, (code - BLACK) / SPAN))
    return int(round(BLACK + SPAN * f(t)))

def table(offset=0):
    return [[value(f, 8 * i + offset) for i in range(128)] for _, f in CURVES]

if __name__ == "__main__":
    if "--check" in sys.argv:
        import re
        src = open(sys.argv[2]).read()
        body = src[src.index("C_CURVE : t_curve"):]
        body = body[:body.index(");")]
        old = [int(v) for v in re.findall(r"-?\d+", body.split(":= (", 1)[1])]
        old = [old[k * 128:(k + 1) * 128] for k in range(8)]
        v01_order = [0, 1, 2, 3, 5, 6, 4, 7]   # v0.1: g.5 g.7 g1.5 g2.2 softS hardS invS crush
        new = table(offset=4)
        for k, (name, _) in enumerate(CURVES):
            o = old[v01_order.index(k)]
            err = max(abs(a - b) for a, b in zip(o, new[k]))
            print(f"{name:10s} max |v0.1 - formula| at bin centres = {err}")
        sys.exit(0)
    rows = []
    for (name, _), vals in zip(CURVES, table()):
        rows.append(f"        -- {name}")
        for j in range(0, 128, 16):
            rows.append("        " + ", ".join(f"{v:3d}" for v in vals[j:j + 16]) + ",")
    rows[-1] = rows[-1].rstrip(",")
    print("    constant C_CURVE : t_curve := (")
    print("\n".join(rows))
    print("    );")
