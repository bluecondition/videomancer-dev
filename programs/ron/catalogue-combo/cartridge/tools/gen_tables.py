#!/usr/bin/env python3
"""Cartridge palette + posterize tables -> splices them into cartridge.vhd.

8 palettes x 16 entries, from canonical RGB, BT.601 studio range:
    Y8 = 16 + 219 y,  C8 = 128 + 224 c   (10-bit = code x 4)
stored U/V-SWAPPED (Cr in the U slot, Cb in the V slot) -- the convention of
every generated-colour program here (v0.1's HW-approved Gameboy/C64/EGA rows
are exactly this formula; its CGA row was full range and is fixed here).

Two views of each palette:
  C_MATCH        what the LUT builder measures distance against
  C_PAL_YC/_B    what is drawn
They differ only for the monochrome handhelds (Gameboy, Virtual Boy): those
are matched on luma alone with the shades spread evenly over black..white, so
a bright input lands on the brightest shade (the Virtual Boy's brightest red
is darker than mid grey and would otherwise lose to black).

C_POST: luma band posterize, 8 settings x 64 input steps -> Y8.
    setting 0 = Off (identity; the VHDL bypasses it), 1..7 = 10/8/6/5/4/3/2 bands
    spread evenly over black (16) .. white (235).

    python3 tools/gen_tables.py      # rewrite the generated block
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
VHD = os.path.join(HERE, '..', 'cartridge.vhd')
BEGIN = '    -- BEGIN GENERATED TABLES (tools/gen_tables.py)\n'
END = '    -- END GENERATED TABLES\n'


def hexrgb(s):
    s = s.lstrip('#')
    return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4))


def ycc(rgb):
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (round(16 + 219 * y), round(128 + 224 * cr), round(128 + 224 * cb))


PALETTES = [
    # name, mono (match on spread luma), colours
    ('Gameboy', True, ['0F380F', '306230', '8BAC0F', '9BBC0F']),
    ('Virtual Boy', True, ['000000', '550000', 'AA0000', 'FF0000']),
    ('CGA', False, ['000000', '55FFFF', 'FF55FF', 'FFFFFF']),
    ('ZX Spectrum', False, ['000000', '0000D7', 'D70000', 'D700D7',
                            '00D700', '00D7D7', 'D7D700', 'D7D7D7',
                            '000000', '0000FF', 'FF0000', 'FF00FF',
                            '00FF00', '00FFFF', 'FFFF00', 'FFFFFF']),
    ('Apple II', False, ['000000', 'E31E60', '604EBD', 'FF44FD',
                         '00A360', '9C9C9C', '14CFFD', 'D0C3FF',
                         '607203', 'FF6A3C', '9C9C9C', 'FFA0D0',
                         '14F53C', 'D0DD8D', '72FFD0', 'FFFFFF']),
    # Colodore (v0.1's table, also the c64 program's)
    ('C64', False, ['000000', 'FFFFFF', '813338', '75CEC8',
                    '8E3C97', '56AC4D', '2E2C9B', 'EDF171',
                    '8E5029', '553800', 'C46C71', '4A4A4A',
                    '7B7B7B', 'A9FF9F', '706DEB', 'B2B2B2']),
    ('EGA', False, ['000000', '0000AA', '00AA00', '00AAAA',
                    'AA0000', 'AA00AA', 'AA5500', 'AAAAAA',
                    '555555', '5555FF', '55FF55', '55FFFF',
                    'FF5555', 'FF55FF', 'FFFF55', 'FFFFFF']),
    ('Pico-8', False, ['000000', '1D2B53', '7E2553', '008751',
                       'AB5236', '5F574F', 'C2C3C7', 'FFF1E8',
                       'FF004D', 'FFA300', 'FFEC27', '00E436',
                       '29ADFF', '83769C', 'FF77A8', 'FFCCAA']),
]

BANDS = [None, 10, 8, 6, 5, 4, 3, 2]


def build():
    match, draw, dark = [], [], []
    for name, mono, cols in PALETTES:
        n = len(cols)
        ent = [ycc(hexrgb(c)) for c in cols]
        if mono:
            # shades sorted dark -> light get evenly spread match lumas
            order = sorted(range(n), key=lambda i: ent[i][0])
            mlum = {}
            for rank, i in enumerate(order):
                mlum[i] = round(16 + 219 * rank / (n - 1))
            mt = [(mlum[i], 128, 128) for i in range(n)]
        else:
            mt = ent
        reps = 16 // n
        draw += ent * reps
        match += mt * reps
        dark.append(min(e[0] for e in ent))
    post = []
    for nb in BANDS:
        for k in range(64):
            y = k * 4 + 2
            if nb is None:
                post.append(y)
                continue
            t = min(max((y - 16) / 219.0, 0.0), 1.0)
            band = round(t * (nb - 1))
            post.append(round(16 + 219 * band / (nb - 1)))
    return match, draw, dark, post


def emit(match, draw, dark, post):
    L = [BEGIN]
    L.append('    -- palettes: ' + ', '.join(p[0] for p in PALETTES) + '\n')
    L.append('    type t_match is array (0 to 127) of std_logic_vector(23 downto 0);\n')
    L.append('    constant C_MATCH : t_match := (\n')
    rows = []
    for z in range(8):
        rows.append('        ' + ', '.join('x"%02X%02X%02X"' % match[z * 16 + i] for i in range(16)))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append('    type t_pal16 is array (0 to 127) of std_logic_vector(15 downto 0);\n')
    L.append('    constant C_PAL_YC : t_pal16 := (\n')
    rows = []
    for z in range(8):
        rows.append('        ' + ', '.join('x"%02X%02X"' % (draw[z * 16 + i][0], draw[z * 16 + i][1]) for i in range(16)))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append('    constant C_PAL_B : t_pal16 := (\n')
    rows = []
    for z in range(8):
        rows.append('        ' + ', '.join('x"00%02X"' % draw[z * 16 + i][2] for i in range(16)))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append('    type t_dark is array (0 to 7) of unsigned(7 downto 0);\n')
    L.append('    constant C_DARK : t_dark := (' +
             ', '.join('x"%02X"' % d for d in dark) + ');\n')
    L.append('    type t_post is array (0 to 511) of std_logic_vector(7 downto 0);\n')
    L.append('    constant C_POST : t_post := (\n')
    rows = []
    for s in range(8):
        for h in range(4):
            rows.append('        ' + ', '.join('x"%02X"' % post[s * 64 + h * 16 + i] for i in range(16)))
    L.append(',\n'.join(rows) + '\n    );\n')
    L.append(END)
    return ''.join(L)


def main():
    block = emit(*build())
    src = open(VHD).read()
    a = src.index(BEGIN)
    b = src.index(END) + len(END)
    open(VHD, 'w').write(src[:a] + block + src[b:])
    print('wrote tables into', os.path.normpath(VHD))


if __name__ == '__main__':
    main()
