"""Emit the VHDL ROM constants for Spiroscope v6 (paste between the markers).

  C_COSQ   256 x 9   quarter-wave cosine, half-step offset: T[i] = 511*cos((i+.5)*pi/512)
  C_DIRUV  256 x 16  segment direction: idx = |dx|<<4 | |dy| -> {u8, v8},
                     u = 2|dy|/L, v = 2|dx|/L in Q6 (L = true length)
  C_PALC   128 x 16  palette chroma {Cr/4 signed 8, Cb/4 signed 8} (HW U/V swap:
                     the U field carries Cr), idx = pal<<4 | age
  C_PALY   128 x 16  palette luma {Ylight/4, Yink/4}
  C_HREC   16 x 17   harmonograph increment per gear unit: round(18*4096/m)
"""
import colorsys
import math


def yuv10(rgb):
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = 0.564 * (b - y)
    cr = 0.713 * (r - y)
    return 64 + 876 * y, 512 + 896 * cb, 512 + 896 * cr


def lerp_stops(stops, t):
    if len(stops) == 1:
        return stops[0]
    x = t * (len(stops) - 1)
    k = min(int(x), len(stops) - 2)
    f = x - k
    return tuple(stops[k][i] * (1 - f) + stops[k + 1][i] * f for i in range(3))


PALS = [
    ('Neon',     [(1.0, 0.0, 0.85), (0.55, 0.1, 1.0), (0.1, 0.45, 1.0), (0.0, 0.95, 1.0)]),
    ('Ember',    [(0.7, 0.0, 0.0), (1.0, 0.25, 0.0), (1.0, 0.6, 0.05), (1.0, 0.92, 0.45)]),
    ('Ice',      [(0.05, 0.1, 0.9), (0.1, 0.5, 1.0), (0.4, 0.9, 1.0), (0.92, 0.98, 1.0)]),
    ('Rainbow',  None),
    ('Gold',     [(0.6, 0.3, 0.0), (0.95, 0.6, 0.1), (1.0, 0.82, 0.3), (1.0, 0.95, 0.7)]),
    ('Aurora',   [(0.0, 0.9, 0.3), (0.0, 0.85, 0.75), (0.3, 0.45, 1.0), (0.75, 0.2, 1.0)]),
    ('Phosphor', [(0.25, 1.0, 0.35)]),
    ('White',    [(1.0, 1.0, 1.0)]),
]
YMAX = 0.78          # light luma cap (Y' -> ~747 in 10-bit)


def light(rgb):
    y = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
    s = min(1.0, YMAX / y) if y > 0 else 1.0
    return tuple(c * s for c in rgb)


def ink(name, rgb):
    if name == 'White':
        return (0.05, 0.05, 0.06)
    if name == 'Phosphor':
        return (0.0, 0.3, 0.08)
    return tuple(c * 0.42 for c in rgb)


def s8(v):
    v = max(-128, min(127, int(round(v))))
    return v & 0xFF


def main():
    out = []
    cos = [round(511 * math.cos((i + 0.5) * math.pi / 512)) for i in range(256)]
    out.append('    constant C_COSQ : t_rom9 := (')
    out.append(fmt_list(cos, 9))
    uv = []
    for adx in range(16):
        for ady in range(16):
            L = math.hypot(adx, ady)
            if L == 0:
                u = v = 0
            else:
                u = round(2 * ady / L * 64)
                v = round(2 * adx / L * 64)
            uv.append((u << 8) | v)
    out.append('    constant C_DIRUV : t_rom16 := (')
    out.append(fmt_list(uv, 16))
    pc, py = [], []
    for name, stops in PALS:
        for h in range(16):
            t = h / 15.0
            if stops is None:
                rgb = colorsys.hsv_to_rgb(t * 300 / 360.0, 1.0, 1.0)
            else:
                rgb = lerp_stops(stops, t)
            lt = light(rgb)
            yl, cbl, crl = yuv10(lt)
            yi, _, _ = yuv10(ink(name, rgb))
            # HW U/V swap: U field = Cr, V field = Cb
            pc.append((s8((crl - 512) / 4) << 8) | s8((cbl - 512) / 4))
            py.append((min(255, int(round(yl / 4))) << 8) | min(255, int(round(yi / 4))))
    out.append('    constant C_PALC : t_pal16 := (')
    out.append(fmt_list(pc, 16))
    out.append('    constant C_PALY : t_pal16 := (')
    out.append(fmt_list(py, 16))
    hr = [round(18 * 4096 / m) for m in range(1, 17)]
    out.append('    constant C_HREC : t_hrec := (')
    out.append(fmt_list(hr, 17))
    paper = yuv10(tuple(c * 0.82 for c in (0.93, 0.89, 0.78)))
    out.append('    -- paper ground Y/Cb/Cr: %d %d %d' % tuple(round(v) for v in paper))
    print('\n'.join(out))


def fmt_list(vals, w):
    lines = []
    per = 8
    for k in range(0, len(vals), per):
        chunk = vals[k:k + per]
        s = ', '.join('to_unsigned(%d, %d)' % (v, w) for v in chunk)
        lines.append('        ' + s + (',' if k + per < len(vals) else ');'))
    return '\n'.join(lines)


if __name__ == '__main__':
    main()
