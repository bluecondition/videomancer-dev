#!/usr/bin/env python3
"""Generate pipedream's two EBR ROM constants (paste between the GEN markers
in pipedream.vhd).

C_KROM   : 256 x 16  K = round(2^14 / r_px4)  -- per-pixel radius normaliser.
           r_px4 is the (luma-scaled) tube radius in quarter pixels; the
           cross-section d (px) maps to unit-radius units by dn = (d*K) >> 6,
           so |dn| = 64 at the tube wall for ANY radius.
C_SATOFF : 256 x 16  highlight offset (1/64 radius) vs light projection p
           (two's-complement 8-bit index), see satoff().
C_SHADE  : 256 x 16  {finish, |et|(6..0)} -> Y8 (15..8) & cf (7..0).
           |et| = |dn - highlight offset|, 64 = one radius.  Y8*4 = Y10.
           cf = chroma factor (255 = full hue saturation, 0 = white/grey).
           Profiles are EVEN in et so arcs and straights (whose cross-section
           axes can point opposite ways at a joint) always shade identically.
"""
import math

def krom():
    out = []
    for a in range(256):
        out.append(0 if a == 0 else min(32767, round(16384 / a)))
    return out

def smooth(e0, e1, x):
    t = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)

def y10(l):
    return min(800, 64 + 876 * max(0.0, l))

def gloss(t):
    om = 0.35
    d = max(0.0, 1 - (t / (1 + om)) ** 2)          # offset-light diffuse
    s = max(0.0, 1 - (t / 0.30) ** 2) ** 3          # tight specular
    lum = 0.07 + 0.50 * d + 0.45 * s
    cf = 255 * (1 - 0.92 * s)
    return y10(lum), cf

def chrome(t):
    # symmetric bands about the light point: white streak, bright sky,
    # a crisp dark horizon, then a dim ground that lifts toward the rim.
    streak = 1 - smooth(0.08, 0.16, t)
    sky = 0.72 - 0.30 * smooth(0.10, 0.46, t)
    horizon = smooth(0.44, 0.50, t) * (1 - smooth(0.56, 0.62, t))
    ground = 0.12 + 0.30 * smooth(0.60, 1.05, t) - 0.20 * smooth(1.05, 1.40, t)
    base = sky if t < 0.53 else ground
    lum = base * (1 - 0.92 * horizon) + 0.20 * streak
    cf = 70 * (1 - streak) + 30 * (t > 0.53)
    return y10(lum), cf

def shade():
    out = []
    for fin, f in ((0, gloss), (1, chrome)):
        for i in range(128):
            y, cf = f(i / 64.0)
            y8 = min(255, int(round(y / 4)))
            out.append((y8 << 8) | max(0, min(255, int(round(cf)))))
    return out

def satoff(O=22, c=55.0, pr=106.0):
    # smooth saturating highlight offset vs the (light . normal) projection p:
    # O * p/sqrt(p^2+c^2), normalised so a straight lit at 45 deg (p ~ pr)
    # sits at ~O.  A soft knee: concentric round most of a bend, with a gentle
    # S where the light crosses the bend's normal (the hard clamp kinked).
    out = []
    for i in range(256):
        p = i if i < 128 else i - 256
        v = O * p / math.sqrt(p * p + c * c) * math.sqrt(pr * pr + c * c) / pr
        out.append(int(round(v)) & 0xFFFF)
    return out

def vhdl(name, vals, width=16):
    lines = [f"    constant {name} : t_rom256 := ("]
    for i in range(0, 256, 8):
        chunk = ", ".join(f'x"{v:04X}"' for v in vals[i:i + 8])
        lines.append("        " + chunk + ("," if i + 8 < 256 else ");"))
    return "\n".join(lines)

if __name__ == "__main__":
    print("    -- GEN-BEGIN (gen_tables.py)")
    print(vhdl("C_KROM", krom()))
    print(vhdl("C_SHADE", shade()))
    print(vhdl("C_SATOFF", satoff()))
    print("    -- GEN-END")
