#!/usr/bin/env python3
"""Generate oracle_tables_pkg.vhd (Oracle v2 constant ROMs).

Tables
  C_PS_Y/U/V   256-step spectrum, one full ROYGBIV cycle (plain)
  C_PA_Y/U/V   same spectrum with soft dark absorption lines (top zone)
  C_QSIN       quarter-wave sine, 256 x 12 (sequencer: ripple, warps, wander)
  C_CSIN       full-wave sine, 512 x 8 signed (static cloud, per pixel)
  C_PVES       vesica (mandorla) half-width profile, 256 x 12
  C_PCIR       circle half-width profile, 256 x 12

The spectrum is built from the seven scope-verified phosphor colours
(memory: phosphor_rainbow_palette), stored in the HARDWARE convention
(the u register carries Cr), so the sim shows mirrored hues and the screen
shows ROYGBIV.  Between anchors it interpolates HSV hue/value, so every
step stays fully saturated and in gamut (straight YUV greys out the
green->blue leg).

Run:  python3 gen/gen_tables.py   (writes ../oracle_tables_pkg.vhd)
"""
import math
import os

N = 256

# (Y, U=Cr, V=Cb) 10-bit, HW convention.  Order = Red..Violet, evenly spaced.
ANCHORS = [
    (328, 960, 360),   # red
    (584, 772, 212),   # orange
    (840, 584, 64),    # yellow
    (580, 136, 216),   # green
    (164, 440, 960),   # blue
    (192, 608, 696),   # indigo
    (349, 756, 852),   # violet
]


def yuv_to_rgbp(y, u, v):
    yp = (y - 64) / 876.0
    cr = (u - 512) / 896.0
    cb = (v - 512) / 896.0
    r = yp + 1.402 * cr
    g = yp - 0.344136 * cb - 0.714136 * cr
    b = yp + 1.772 * cb
    return r, g, b


def rgbp_to_yuv(r, g, b):
    yp = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - yp) / 1.772
    cr = (r - yp) / 1.402
    y = 64 + yp * 876
    u = 512 + cr * 896
    v = 512 + cb * 896
    return y, u, v


def dec(c):   # gamma-encoded -> linear (BT.1886-ish)
    c = min(max(c, 0.0), 1.0)
    return c ** 2.4


def enc(c):
    c = min(max(c, 0.0), 1.0)
    return c ** (1 / 2.4)


def lin_to_oklab(r, g, b):
    l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b
    l, m, s = (math.copysign(abs(x) ** (1 / 3), x) for x in (l, m, s))
    return (0.2104542553 * l + 0.7936177850 * m - 0.0040720468 * s,
            1.9779984951 * l - 2.4285922050 * m + 0.4505937099 * s,
            0.0259040371 * l + 0.7827717662 * m - 0.8086757660 * s)


def oklab_to_lin(L, a, b):
    l = L + 0.3963377774 * a + 0.2158037573 * b
    m = L - 0.1055613458 * a - 0.0638541728 * b
    s = L - 0.0894841775 * a - 1.2914855480 * b
    l, m, s = l ** 3, m ** 3, s ** 3
    return (4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
            -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
            -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s)


def to_lch(yuv):
    r, g, b = (dec(c) for c in yuv_to_rgbp(*yuv))
    L, a, bb = lin_to_oklab(r, g, b)
    return L, math.hypot(a, bb), math.atan2(bb, a)


def in_gamut(rgb, eps=1e-4):
    return all(-eps <= c <= 1 + eps for c in rgb)


def lch_to_yuv(L, C, h):
    # reduce chroma until the linear RGB point is inside the cube
    lo, hi = 0.0, C
    rgb = oklab_to_lin(L, C * math.cos(h), C * math.sin(h))
    if not in_gamut(rgb):
        for _ in range(30):
            mid = (lo + hi) / 2
            if in_gamut(oklab_to_lin(L, mid * math.cos(h), mid * math.sin(h))):
                lo = mid
            else:
                hi = mid
        rgb = oklab_to_lin(L, lo * math.cos(h), lo * math.sin(h))
    rp = [enc(c) for c in rgb]
    return rgbp_to_yuv(*rp)


def rgb_to_hsv(r, g, b):
    r, g, b = (min(max(c, 0.0), 1.0) for c in (r, g, b))
    mx, mn = max(r, g, b), min(r, g, b)
    d = mx - mn
    if d == 0:
        h = 0.0
    elif mx == r:
        h = ((g - b) / d) % 6
    elif mx == g:
        h = (b - r) / d + 2
    else:
        h = (r - g) / d + 4
    return h * 60.0, (d / mx if mx else 0.0), mx


def hsv_to_rgb(h, s, v):
    h = (h % 360.0) / 60.0
    i = int(h) % 6
    f = h - int(h)
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][i]


def spectrum():
    # HSV-hue interpolation between the anchors: every intermediate colour
    # stays on the RGB cube surface (fully saturated, always in gamut),
    # unlike straight YUV (greys out green->blue) or a linear-light space
    # (near-zero channels blow up under the gamma and wash the blues out).
    hsv = [rgb_to_hsv(*yuv_to_rgbp(*a)) for a in ANCHORS]
    out = []
    k = len(ANCHORS)
    for i in range(N):
        t = i * k / N
        j = int(t)
        f = t - j
        h0, s0, v0 = hsv[j]
        h1, s1, v1 = hsv[(j + 1) % k]
        if h1 < h0:
            h1 += 360.0
        # half smoothstep: each anchor is a gentle plateau of its pure
        # colour rather than a kink
        fs = f * f * (3 - 2 * f)
        fb = 0.5 * f + 0.5 * fs
        rgb = hsv_to_rgb(h0 + (h1 - h0) * fb, s0 + (s1 - s0) * fb, v0 + (v1 - v0) * fb)
        out.append(rgbp_to_yuv(*rgb))
    return out


def q10(x):
    return max(0, min(1023, int(round(x))))


def absorbed(spec):
    # soft dark lines (Fraunhofer-style) at fixed hues: depth, centre, sigma
    lines = [(0.62, 0.075, 2.6), (0.70, 0.275, 2.4), (0.55, 0.50, 2.8),
             (0.66, 0.705, 2.6)]
    out = []
    for i, (y, u, v) in enumerate(spec):
        g = 1.0
        for depth, c, sig in lines:
            d = i - c * N
            d -= N * round(d / N)
            g *= 1 - depth * math.exp(-0.5 * (d / sig) ** 2)
        out.append((64 + (y - 64) * g, 512 + (u - 512) * g, 512 + (v - 512) * g))
    return out


def vhdl_u(name, vals, width, per=8):
    lines = [f"    constant {name} : t_rom{len(vals)}x{width} := ("]
    body = []
    for i in range(0, len(vals), per):
        chunk = ", ".join(f"to_unsigned({v:4d}, {width})" for v in vals[i:i + per])
        body.append("        " + chunk)
    lines.append(",\n".join(body) + ");")
    return "\n".join(lines)


def vhdl_s(name, vals, width, per=8):
    lines = [f"    constant {name} : t_srom{len(vals)}x{width} := ("]
    body = []
    for i in range(0, len(vals), per):
        chunk = ", ".join(f"to_signed({v:4d}, {width})" for v in vals[i:i + per])
        body.append("        " + chunk)
    lines.append(",\n".join(body) + ");")
    return "\n".join(lines)


def main():
    spec = spectrum()
    absr = absorbed(spec)
    ps = [[q10(c[k]) for c in spec] for k in range(3)]
    pa = [[q10(c[k]) for c in absr] for k in range(3)]

    qsin = [int(round(4095 * math.sin((i + 0.5) * math.pi / 512))) for i in range(256)]
    csin = [int(round(127 * math.sin(2 * math.pi * (i + 0.5) / 512))) for i in range(512)]

    # vesica: two circles of radius 1 centred at +-D; profile normalised to
    # 1 at the waist and 0 at the tips; v = i/256 of the half-height
    D = 0.6
    hv = math.sqrt(1 - D * D)
    pves, pcir = [], []
    for i in range(256):
        v = i / 256.0
        pv = (math.sqrt(max(0.0, 1 - (v * hv) ** 2)) - D) / (1 - D)
        pc = math.sqrt(max(0.0, 1 - v * v))
        pves.append(max(0, min(4095, int(round(4095 * pv)))))
        pcir.append(max(0, min(4095, int(round(4095 * pc)))))

    hdr = '''-- oracle_tables_pkg.vhd -- GENERATED by gen/gen_tables.py, do not edit.
--
-- Spectrum palettes are in the HARDWARE convention (u carries Cr): built
-- from the seven scope-verified phosphor colours, interpolated in HSV.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package oracle_tables_pkg is
    type t_rom256x10 is array(0 to 255) of unsigned(9 downto 0);
    type t_rom256x12 is array(0 to 255) of unsigned(11 downto 0);
    type t_srom512x8 is array(0 to 511) of signed(7 downto 0);
'''
    parts = [hdr]
    parts.append(vhdl_u("C_PS_Y", ps[0], 10) + "\n")
    parts.append(vhdl_u("C_PS_U", ps[1], 10) + "\n")
    parts.append(vhdl_u("C_PS_V", ps[2], 10) + "\n")
    parts.append(vhdl_u("C_PA_Y", pa[0], 10) + "\n")
    parts.append(vhdl_u("C_PA_U", pa[1], 10) + "\n")
    parts.append(vhdl_u("C_PA_V", pa[2], 10) + "\n")
    parts.append(vhdl_u("C_QSIN", qsin, 12) + "\n")
    parts.append(vhdl_s("C_CSIN", csin, 8) + "\n")
    parts.append(vhdl_u("C_PVES", pves, 12) + "\n")
    parts.append(vhdl_u("C_PCIR", pcir, 12) + "\n")
    parts.append("end package;\n")

    here = os.path.dirname(os.path.abspath(__file__))
    dst = os.path.join(here, "..", "oracle_tables_pkg.vhd")
    with open(dst, "w") as f:
        f.write("\n".join(parts))

    # report: anchor positions and a few samples
    for k in range(7):
        i = int(round(k * N / 7))
        print(f"anchor {k} at {i}: Y{ps[0][i]} U{ps[1][i]} V{ps[2][i]}  (src {ANCHORS[k]})")
    print("Y range", min(ps[0]), max(ps[0]))
    print("pves[0,128,200,255]", pves[0], pves[128], pves[200], pves[255])
    print("pcir[0,128,200,255]", pcir[0], pcir[128], pcir[200], pcir[255])
    print("wrote", os.path.normpath(dst))


if __name__ == "__main__":
    main()
