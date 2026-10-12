#!/usr/bin/env python3
"""Infrared palette + zoom tables -> splices them into infrared.vhd.

8 palettes x 256 entries.  Words: C_PAL_YC = Y8 & Cr8, C_PAL_B = Cb8, i.e.
stored U/V-SWAPPED (the program sends the Cr slot out on U), the convention
every generated-colour program here uses so hues land right on hardware.
8-bit codes: Y8 = 16 + 219 y, C8 = 128 + 224 c (10-bit = code x 4).

The four v0.1 palettes (Thermal, Ice, Plasma, X-Ray: HW-approved 2026-09-11)
are kept bit-exact at their old 64 sample points and linearly interpolated
in between; the four new ones are RGB stop ramps.

    python3 tools/gen_palettes.py            # rewrite the generated block
    python3 tools/gen_palettes.py --png out.png   # also write a preview strip
"""
import math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
VHD = os.path.join(HERE, '..', 'infrared.vhd')
BEGIN = '    -- BEGIN GENERATED TABLES (tools/gen_palettes.py)\n'
END = '    -- END GENERATED TABLES\n'

# v0.1 tables: (Y8, Cr8, Cb8) x 64, read once from the v0.1.0 source
# (infrared.vhd at 3fa438d) and frozen here.
V01_YC = """
1080 1180 1280 1381 1581 1681 1781 1882 1982 1B82 1C83 1E85 2188 248C 2790 2A94
2D97 309B 339F 36A3 39A6 3CAA 3DAF 3EB2 3FB6 40BA 41BE 42C3 43C6 44CB 45CF 46D2
49D5 4FD4 54D3 59D2 5FD2 64D2 69D1 6ED0 74D0 79CF 7ECE 85CA 8BC5 91C1 98BC 9EB8
A4B3 AAAF B0AA B6A6 BDA1 C29D C69A CA97 CE94 D291 D68F DB8B DE89 E286 E782 EB80
1080 117E 137E 147C 157B 177A 1979 1A78 1C77 1D76 1F75 2074 2272 2471 286E 2B6C
2F69 3266 3663 3A61 3E5E 415B 4559 4956 4C54 5051 554F 5C4E 614C 684B 6E49 7448
7946 8045 8644 8C42 9241 983F 9E3F A242 A746 AC4A B04D B551 BA54 BE58 C35C C85F
CC63 D166 D66A D86C DA6E DB6F DD71 DF72 E074 E176 E377 E579 E67B E77C E97E EB80
2379 257C 277F 2982 2B85 2D88 2F8B 318F 3392 3595 3698 399B 3B9E 3DA1 3EA4 40A8
43AA 46AB 49AC 4CAC 4FAD 53AE 56AF 59B0 5CB0 5FB1 62B2 66B3 69B4 6CB5 6FB5 72B6
75B6 78B6 7BB6 7EB6 80B5 83B5 86B4 89B4 8CB4 8FB4 92B3 94B3 97B2 9AB2 9DB2 A0B1
A3AF A6AD A9AB ACA8 AEA6 B1A3 B4A1 B79E BA9C BD9A C097 C395 C693 C890 CB8E CE8B
DD7A D87A D279 CD79 C778 C278 BD77 B877 B276 AD76 A775 A275 9D74 9774 9273 8D73
8873 8373 7E73 7973 7573 7074 6B74 6674 6275 5D75 5876 5376 4F76 4A77 4577 4077
3D77 3A78 3878 3678 3478 3279 2F7A 2D7A 2B7B 297B 277B 257C 227C 207C 1E7D 1C7D
1B7E 1A7E 197E 197E 187E 177E 167E 157F 147F 147F 137F 137F 127F 117F 117F 1080
"""
V01_B = """
80 83 86 89 8C 8F 92 95 98 9B 9E A0 A2 A2 A4 A5 A6 A7 A8 A9 AA AB A6 A1 9B 96 91 8B 86 81 7B 76
71 6C 67 62 5D 58 53 4E 4A 45 40 3C 38 35 31 2E 2A 26 23 1F 1C 1F 28 32 3C 45 4F 59 62 6C 76 80
80 82 85 87 8B 8D 90 92 95 98 9A 9D A0 A2 A4 A7 A9 AB AD AF B2 B4 B6 B8 BB BD BC BB BA B8 B7 B5
B5 B3 B1 B0 AF AE AC A9 A7 A4 A1 9E 9C 99 97 94 91 8F 8C 8A 89 88 87 87 86 85 84 83 82 82 81 80
B7 B7 B7 B7 B7 B6 B7 B7 B7 B6 B6 B6 B6 B6 B6 B6 B5 B2 AE AB A8 A4 A1 9E 9A 97 94 90 8D 8A 86 83
80 7D 79 76 72 6F 6B 68 64 61 5E 5A 57 53 50 4D 4A 47 45 42 3F 3D 3A 37 34 32 2F 2D 2A 28 25 22
88 89 8A 8C 8D 8F 90 91 93 94 95 97 98 9A 9B 9C 9D 9D 9D 9D 9D 9C 9C 9C 9C 9C 9C 9C 9C 9C 9C 9B
9B 9A 99 98 97 96 95 95 94 93 92 91 90 8F 8E 8D 8C 8B 8B 89 89 88 87 86 86 84 84 83 82 81 81 80
"""

def v01(slot):
    yc = [int(h, 16) for h in V01_YC.split()]
    b = [int(h, 16) for h in V01_B.split()]
    t = [(w >> 8, w & 255, c) for w, c in zip(yc, b)]
    return t[slot * 64:(slot + 1) * 64]

def upsample64(tab):
    """256 entries: entry j covers index 4j..4j+3 of the 1024 ramp; the old
    entry k covered 16k..16k+15, so sample the old table at (4j-6)/16."""
    out = []
    for j in range(256):
        p = min(max((4 * j - 6) / 16.0, 0.0), 63.0)
        k = min(int(p), 62)
        f = p - k
        out.append(tuple(round(a + (b - a) * f) for a, b in zip(tab[k], tab[k + 1])))
    return out

def rgb_ramp(stops):
    out = []
    for j in range(256):
        t = j / 255.0
        for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
            if t0 <= t <= t1:
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                r, g, b = (a + (c - a) * f for a, c in zip(c0, c1))
                break
        y = 0.299 * r + 0.587 * g + 0.114 * b
        cb = 0.564 * (b - y)
        cr = 0.713 * (r - y)
        clip = lambda v: max(0, min(255, v))
        out.append((clip(round(16 + 219 * y)), clip(round(128 + 224 * cr)),
                    clip(round(128 + 224 * cb))))
    return out

IRONBOW = [(0.00, (0.00, 0.00, 0.05)), (0.10, (0.05, 0.02, 0.40)),
           (0.25, (0.35, 0.00, 0.65)), (0.42, (0.75, 0.05, 0.55)),
           (0.58, (0.95, 0.25, 0.20)), (0.72, (1.00, 0.55, 0.00)),
           (0.86, (1.00, 0.85, 0.15)), (1.00, (1.00, 1.00, 0.95))]
RAINBOW = [(0.00, (0.00, 0.00, 0.00)), (0.10, (0.15, 0.00, 0.40)),
           (0.22, (0.00, 0.05, 0.90)), (0.36, (0.00, 0.70, 0.95)),
           (0.50, (0.00, 0.85, 0.25)), (0.64, (0.85, 0.95, 0.00)),
           (0.78, (1.00, 0.45, 0.00)), (0.90, (1.00, 0.05, 0.05)),
           (1.00, (1.00, 0.95, 0.95))]
NIGHT = [(0.00, (0.00, 0.00, 0.00)), (0.40, (0.02, 0.30, 0.05)),
         (0.75, (0.20, 0.85, 0.25)), (1.00, (0.85, 1.00, 0.80))]
WHITE = [(0.00, (0.0, 0.0, 0.0)), (1.00, (1.0, 1.0, 1.0))]

# K4 order (zone = knob(9:7)); names must match the TOML value_labels
PALETTES = [
    ('Thermal', upsample64(v01(0))),
    ('Ironbow', rgb_ramp(IRONBOW)),
    ('Plasma', upsample64(v01(2))),
    ('Rainbow', rgb_ramp(RAINBOW)),
    ('Ice', upsample64(v01(1))),
    ('Night Vision', rgb_ramp(NIGHT)),
    ('White Hot', rgb_ramp(WHITE)),
    ('X-Ray', upsample64(v01(3))),
]

def zoom_rom():
    return [round(1024 * 2 ** (i / 64.0)) for i in range(64)]

def emit():
    ent = [e for _, p in PALETTES for e in p]
    assert len(ent) == 2048
    L = []
    L.append(BEGIN)
    L.append('    -- K4 zones: ' + ', '.join(n for n, _ in PALETTES) + '\n')
    L.append('    type t_pal16 is array (0 to 2047) of std_logic_vector(15 downto 0);\n')
    L.append('    type t_pal8  is array (0 to 2047) of std_logic_vector(7 downto 0);\n')
    L.append('    constant C_PAL_YC : t_pal16 := (\n')
    rows = []
    for i in range(0, 2048, 16):
        rows.append('        ' + ', '.join('x"%02X%02X"' % (y, cr) for y, cr, _ in ent[i:i + 16]))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append('    constant C_PAL_B : t_pal8 := (\n')
    rows = []
    for i in range(0, 2048, 16):
        rows.append('        ' + ', '.join('x"%02X"' % cb for _, _, cb in ent[i:i + 16]))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append('    -- 1024 * 2^(i/64): one octave of zoom steps\n')
    L.append('    type t_zoom_rom is array (0 to 63) of unsigned(10 downto 0);\n')
    L.append('    constant C_ZOOM_ROM : t_zoom_rom := (\n')
    z = zoom_rom()
    rows = []
    for i in range(0, 64, 8):
        rows.append('        ' + ', '.join('to_unsigned(%d, 11)' % v for v in z[i:i + 8]))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append(END)
    return ''.join(L)

def preview(path):
    import numpy as np
    from PIL import Image
    img = np.zeros((8 * 40, 256 * 3, 3), np.uint8)
    for p, (_, tab) in enumerate(PALETTES):
        for j, (y8, cr8, cb8) in enumerate(tab):
            y = (y8 - 16) / 219.0
            cr = (cr8 - 128) / 224.0
            cb = (cb8 - 128) / 224.0
            r = y + 1.402 * cr
            g = y - 0.344136 * cb - 0.714136 * cr
            b = y + 1.772 * cb
            img[p * 40:p * 40 + 36, j * 3:j * 3 + 3] = [max(0, min(255, round(v * 255))) for v in (r, g, b)]
    Image.fromarray(img).save(path)

def main():
    src = open(VHD).read()
    a, b = src.index(BEGIN), src.index(END) + len(END)
    new = src[:a] + emit() + src[b:]
    if new != src:
        open(VHD, 'w').write(new)
        print('infrared.vhd: tables rewritten')
    else:
        print('infrared.vhd: tables unchanged')
    if '--png' in sys.argv:
        preview(sys.argv[sys.argv.index('--png') + 1])

if __name__ == '__main__':
    main()
