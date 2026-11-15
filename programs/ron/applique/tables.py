#!/usr/bin/env python3
"""tables.py -- the APPLIQUE set ROM, in one place.

  SROM (256 x 16, 128 used): two words per (set 0..7, slot 0..7):
      word 0 recipe : [3:0] pattern  [5:4] variant  [7:6] size  [8] bias  [9] turn
                      [10] stagger  [11] invert  [12] half stagger
      word 1 colour : [15:12] hA  [11:10] sA  [9:6] hB  [5:4] sB  [3:1] dH (signed)  [0] dS

Recipe: the pattern is drawn in cell coordinates (a, b): 64 positions per cell, the cell is
16 << size virtual pixels at Scale 0% (x16 at 100%).  bias = the 45 degree axes (x + y, x - y),
turn = a and b swapped (90 degrees), stagger = every other row of cells shifted half a cell
(half stagger: every other HALF row, for scales and running bonds), invert = the two tones swapped.  Patterns and their four variants: see PATTERNS below and
p_fill in applique.vhd (model.py mirrors it).

Colours are (hue16, sat): hue16 = 22.5 degree steps counter-clockwise from +Cb (0 blue-violet,
2 magenta, 4 red, 6 orange, 7 ochre, 8 yellow, 10 green, 12 teal, 14 sky blue), sat 0 grey,
1 soft, 2 full, 3 medium.  A = the dark tone's colour, B = the light tone's.
In Source / Tint the accent is the region's own hue + dH steps (dS: go one step richer).

`python3 tables.py` rewrites the constants between the TABLES markers of applique.vhd."""
import pathlib

PATTERNS = ['plain', 'stripe', 'ridge', 'check', 'plaid', 'hound', 'dot', 'target',
            'diamond', 'stud', 'tri', 'basket', 'zigzag', 'brick', 'scale', 'cross']

def R(pattern, variant=0, size=0, bias=0, turn=0, stagger=0, invert=0, half=0):
    return (PATTERNS.index(pattern) | (variant << 4) | (size << 6) | (bias << 8) | (turn << 9)
            | (stagger << 10) | (invert << 11) | (half << 12))

def C(ha, sa, hb, sb, dh=-1, ds=1):
    return (ha, sa, hb, sb, dh, ds)

# 8 slots per set, dark bin -> light bin: (recipe, colour)
SETS = [
    ('Quilt',  [(R('tri', 1, 1),             C(0, 2, 4, 2)),      # pinwheel, navy / red
                (R('target', 0, 1),          C(15, 2, 14, 1)),    # log-cabin squares, navy / sky
                (R('tri', 2, 1),             C(10, 2, 7, 2)),     # hourglass, green / gold
                (R('check', 0, 0),           C(4, 3, 0, 0)),      # checkerboard, red / white
                (R('diamond', 0, 1),         C(12, 2, 3, 1)),     # harlequin, teal / pink
                (R('tri', 3, 1),             C(6, 2, 8, 1)),      # flying geese, orange / cream
                (R('basket', 0, 1),          C(14, 1, 0, 0)),     # basketweave, pale blue
                (R('stripe', 1, 0, bias=1),  C(14, 1, 0, 0))]),   # bias stripe
    ('Weave',  [(R('ridge', 0, 0, bias=1),   C(15, 2, 14, 1)),    # twill, indigo
                (R('zigzag', 1, 1),          C(0, 3, 14, 1)),     # herringbone
                (R('basket', 0, 0),          C(5, 2, 7, 1)),      # basket, rust
                (R('zigzag', 0, 0),          C(6, 3, 8, 1)),      # chevron
                (R('basket', 2, 1),          C(9, 1, 8, 1)),      # big basket, olive
                (R('ridge', 1, 0, turn=1),   C(5, 1, 7, 1)),      # rounded ribs, tan
                (R('plain', 1, 0),           C(7, 1, 0, 0)),      # canvas
                (R('ridge', 3, 0),           C(8, 1, 0, 0))]),    # fine ribs
    ('Stripe', [(R('stripe', 0, 1),          C(0, 2, 0, 0)),      # bold, navy / white
                (R('stripe', 2, 0),          C(15, 3, 0, 0)),     # pinstripe
                (R('stripe', 0, 1, bias=1),  C(4, 2, 0, 0)),      # bias, red / white
                (R('stripe', 3, 1, turn=1),  C(0, 2, 4, 2)),      # regimental
                (R('stripe', 1, 0, turn=1),  C(14, 2, 0, 0)),     # fine horizontal
                (R('stripe', 0, 1, bias=1, turn=1), C(6, 2, 0, 0)),   # bias the other way, orange / white
                (R('stripe', 2, 1, turn=1),  C(14, 1, 0, 0)),     # wide-set pin, horizontal
                (R('stripe', 1, 0),          C(4, 1, 0, 0))]),
    ('Check',  [(R('plaid', 0, 1),           C(0, 2, 10, 2)),     # tartan, navy / green
                (R('hound', 0, 0),           C(0, 0, 0, 0)),      # houndstooth, black / white
                (R('check', 1, 0),           C(4, 2, 0, 0)),      # gingham, red
                (R('plaid', 2, 1),           C(4, 2, 8, 2)),      # tartan, red / yellow
                (R('check', 0, 1, bias=1),   C(14, 2, 0, 0)),     # diamond check
                (R('check', 2, 1),           C(12, 1, 0, 0)),     # windowpane
                (R('check', 1, 0),           C(14, 2, 0, 0)),     # gingham, blue
                (R('check', 3, 0),           C(3, 1, 0, 0))]),
    ('Dots',   [(R('dot', 2, 1, stagger=1),  C(1, 2, 6, 2)),      # big dots, purple / orange
                (R('dot', 3, 1),             C(0, 2, 13, 2)),     # rings
                (R('dot', 0, 0, stagger=1),  C(4, 2, 0, 0)),      # polka, red / white
                (R('target', 2, 1),          C(12, 2, 8, 2)),     # bullseyes
                (R('dot', 1, 0, stagger=1),  C(14, 2, 3, 1)),     # pin dots
                (R('stud', 2, 0, stagger=1), C(6, 2, 8, 2)),      # domes
                (R('dot', 0, 1, invert=1),   C(3, 1, 0, 0)),      # dark dots
                (R('dot', 1, 0, invert=1),   C(12, 2, 0, 0))]),
    ('Tile',   [(R('brick', 0, 1),           C(0, 0, 5, 2)),      # brick, terracotta
                (R('scale', 1, 1, half=1),   C(0, 2, 12, 2)),     # fish scales, cobalt / teal
                (R('diamond', 3, 1),         C(12, 2, 0, 0)),     # snowball octagons
                (R('cross', 1, 1),           C(5, 2, 8, 1)),      # lattice
                (R('brick', 1, 0),           C(0, 0, 13, 2)),     # square tiles
                (R('target', 0, 1, bias=1),  C(6, 2, 8, 1)),      # nested diamonds
                (R('scale', 0, 1, half=1),   C(12, 1, 8, 1)),     # scallop outlines
                (R('brick', 3, 0),           C(0, 0, 8, 1))]),
    ('Deco',   [(R('zigzag', 0, 1),          C(0, 0, 7, 2)),      # chevron, black / gold
                (R('target', 3, 1),          C(11, 2, 8, 2)),     # nested diamonds, emerald
                (R('scale', 3, 1, half=1),   C(12, 2, 8, 2)),     # fans
                (R('diamond', 1, 1),         C(0, 0, 7, 2)),      # argyle
                (R('stud', 0, 0),            C(7, 3, 8, 2)),      # pyramids
                (R('zigzag', 2, 1),          C(11, 2, 8, 1)),     # shaded chevron
                (R('tri', 0, 1, bias=1),     C(7, 2, 0, 0)),      # triangles
                (R('target', 1, 1),          C(8, 1, 0, 0))]),
    ('Hatch',  [(R('cross', 1, 0, invert=1), C(5, 1, 7, 1)),      # dense lattice, light on dark (sepia)
                (R('check', 2, 0, bias=1),   C(5, 1, 7, 1)),      # crosshatch
                (R('stripe', 1, 0, bias=1),  C(5, 1, 7, 1)),      # bias hatch
                (R('stripe', 2, 0, bias=1, turn=1), C(6, 1, 7, 1)),
                (R('cross', 2, 0),           C(6, 1, 7, 1)),      # halftone screen
                (R('dot', 1, 0, stagger=1, invert=1), C(6, 1, 7, 1)),
                (R('stripe', 2, 1, turn=1),  C(6, 1, 7, 1)),      # sparse horizontal rule
                (R('plain', 1, 0),           C(7, 1, 0, 0))]),
]
SET_NAMES = [n for n, _ in SETS]

def colour_word(c):
    ha, sa, hb, sb, dh, ds = c
    return (ha << 12) | (sa << 10) | (hb << 6) | (sb << 4) | ((dh & 7) << 1) | ds

def srom_words():
    w = []
    for _, slots in SETS:
        assert len(slots) == 8
        for rec, col in slots:
            w += [rec, colour_word(col)]
    return w + [0] * (256 - len(w))                # declared 256 deep (power of two)

def _vhdl_array(name, words):
    rows = []
    for i in range(0, len(words), 8):
        rows.append('        ' + ', '.join(f'x"{v:04X}"' for v in words[i:i + 8]))
    return f'    constant {name} : t_rom16 := (\n' + ',\n'.join(rows) + '\n    );\n'

def vhdl_consts():
    return ('    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);\n'
            '    -- set ROM: 2 words per (set, slot): recipe, colours (see tables.py)\n' + _vhdl_array('C_SROM', srom_words()))

def stamp(path):
    s = path.read_text()
    a = s.index('    -- TABLES BEGIN'); b = s.index('    -- TABLES END')
    a = s.index('\n', a) + 1
    path.write_text(s[:a] + vhdl_consts() + s[b:])

if __name__ == '__main__':
    stamp(pathlib.Path(__file__).resolve().parent / 'applique.vhd')
    print('tables stamped')
