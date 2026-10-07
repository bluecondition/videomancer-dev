#!/usr/bin/env python3
"""Validate the fixed-centred-iris formulation.

Screen normalised so the iris is the UNIT circle at the origin.  The pupil is a
circle of radius P whose centre is at distance D along +x (the frame is rotated so
the pupil offset is on the axis; the rotation is folded into per-frame constants).

Because the iris is fixed and centred, the two limiting points of the coaxal pencil
are inverse in the unit circle: they are a and 1/a.  So the Moebius map is the disc
automorphism w = (z - a)/(1 - a z), and

    (z-a)(1-a conj(z)) = Xd + i Yn ,  Xd = x(1+a^2) - a(1 + x^2 + y^2) ,  Yn = y(1-a^2)

Xd is a quadratic in x and Yn is constant along a scanline, so BOTH come from plain
accumulators.  One CORDIC vectoring on (Xd, Yn) yields arg w and |Xd+iYn| = r1*r2.
With num = |z-a|^2 = r1^2 from a second accumulator,

    log2|w| = log2(r1/r2) = log2(num) - log2(r1*r2)

so the whole per-pixel geometry is one CORDIC plus accumulators -- no second CORDIC
lane, and no probe of the pixel engine at all.
"""
import math, cmath

def solve_a(D, P):
    """limiting point inside the pupil, unit-disc normalisation"""
    Q = 1.0 + D * D - P * P
    return (Q - math.sqrt(Q * Q - 4 * D * D)) / (2 * D)

def check(D, P, n=9):
    a = solve_a(D, P)
    W = lambda z: (z - a) / (1 - a * z)
    # the pupil must map to a circle centred on the origin
    rad = [abs(W(complex(D + P * math.cos(t), P * math.sin(t)))) for t in
           [2 * math.pi * i / 64 for i in range(64)]]
    # the iris (unit circle) must map to the unit circle
    rim = [abs(W(cmath.exp(1j * 2 * math.pi * i / 64))) for i in range(64)]
    # and the identity used by the engine must hold everywhere
    worst_id, worst_ang, worst_mag = 0.0, 0.0, 0.0
    for i in range(n):
        for j in range(n):
            z = complex(-1.3 + 2.6 * i / (n - 1), -1.3 + 2.6 * j / (n - 1))
            if abs(z - a) < 1e-6 or abs(1 - a * z) < 1e-6: continue
            x, y = z.real, z.imag
            Xd = x * (1 + a * a) - a * (1 + x * x + y * y)
            Yn = y * (1 - a * a)
            prod = (z - a) * (1 - a * z.conjugate())
            worst_id = max(worst_id, abs(complex(Xd, Yn) - prod))
            w = W(z)
            worst_ang = max(worst_ang, abs((cmath.phase(complex(Xd, Yn)) - cmath.phase(w) + math.pi) % (2 * math.pi) - math.pi))
            num = abs(z - a) ** 2
            mag = abs(complex(Xd, Yn))
            worst_mag = max(worst_mag, abs((math.log2(num) - math.log2(mag)) - math.log2(abs(w))))
    return a, min(rad), max(rad), min(rim), max(rim), worst_id, worst_ang, worst_mag

print(f"{'D':>6} {'P':>6} {'a':>8} {'pupil image r (min..max)':>30} {'rim image r':>22} {'identity':>10} {'arg err':>9} {'log err':>9}")
for D, P in [(0.30, 0.06), (0.001, 0.06), (0.60, 0.06), (0.30, 0.0015), (0.30, 0.30), (0.75, 0.20), (0.90, 0.05)]:
    a, r0, r1, m0, m1, wi, wa, wm = check(D, P)
    print(f"{D:6.3f} {P:6.4f} {a:8.5f}   {r0:.9f}..{r1:.9f}   {m0:.9f}..{m1:.9f} {wi:10.2e} {wa:9.2e} {wm:9.2e}")

# ---- fixed-point check: num = |z-a|^2 as a second-order integer DDA along a line
def dda_error(D, P, F, W=1920, R=0.94 * 540, cy_frac=0.31):
    """F = fractional bits of the num accumulator; returns the worst log2(num) error
    in octaves over one scanline, and the worst ring-coordinate error in rings."""
    a = solve_a(D, P)
    # screen -> normalised, no rotation (worst case is the line through the pupil)
    sc = 1.0 / R
    yline = (cy_frac - 0.5) * 1080 * sc
    f = lambda i: ((i - W / 2) * sc - a) ** 2 + yline * yline
    d2 = f(2) - 2 * f(1) + f(0)                      # constant second difference
    q = lambda v: int(round(v * (1 << F)))
    v, d, dd = q(f(0)), q(f(1) - f(0)), q(d2)
    worst_l, worst_u = 0.0, 0.0
    N, Lg = 30, None
    for i in range(W):
        exact = f(i)
        got = v / (1 << F)
        if exact > 1e-12 and got > 0:
            e = abs(math.log2(got) - math.log2(exact))
            worst_l = max(worst_l, e)
            # rings: u = N*(log2|w|-log2 eps)/(0-log2 eps); log2|w| = (log2 num)/2 - log2|r1r2|
            eps = abs((D - P - a) / (1 - a * (D - P)))
            worst_u = max(worst_u, e / 2 * N / (-math.log2(eps)))
        v += d; d += dd
    return worst_l, worst_u

print()
print(f"{'case':>22} {'bits':>5} {'worst log2 err':>15} {'worst ring err':>15}")
for name, (D, P) in [('reference pupil 6%', (0.30, 0.06)), ('pinpoint pupil', (0.30, 0.0015)),
                     ('pupil at rim', (0.88, 0.06)), ('wide pupil 30%', (0.30, 0.30))]:
    for F in (24, 28, 32, 36):
        wl, wu = dda_error(D, P, F)
        print(f"{name:>22} {F:5d} {wl:15.2e} {wu:15.2e}")
