#!/usr/bin/env python3
"""tables.py -- the VENEER ROM contents, in one place.

  NROM (256 x 16): random words: the bytes the noise nodes filter (model.py: rnd16)

  SROM (256 x 16, 128 used): two words per (set 0..7, slot 0..7):
      word 0 recipe : [15:13] g1  [12:10] g2  [9:7] sx  [6:4] sy  [3] yneg  [2:0] shape
      word 1 colour : [15:12] hA  [11:10] sA  [9:6] hB  [5:4] sB  [3:1] dH (signed)  [0] dS

Recipe: phase = (n1 << g1) + (n2 << g2) + (X << (sx + 1)) +- (Y << (sy + 1))   (g = 7: unused,
s = 0: unused; X/Y shifts drop one per Scale step), then a shape:
  0 direct  1 bands  2 thin dark line  3 camouflage  4 veins  5 blobs  6 zebra  7 terraces
Colours are (hue16, sat): hue16 = 22.5 degree steps counter-clockwise from +Cb (0 blue-violet,
2 magenta, 4 red, 6 orange, 7 ochre, 8 yellow, 10 green, 12 teal, 14 sky blue), sat 0 grey,
1 soft, 2 full, 3 medium.  A = base colour, B = the colour the texture swings toward.
In Source / Tint the accent is the region's own hue + dH steps (dS: go one step richer).

`python3 tables.py` rewrites the constants between the TABLES markers of veneer.vhd."""
import pathlib

import random
_rng = random.Random(20261113)
NROM = [_rng.getrandbits(16) for _ in range(256)]      # noise ROM: 256 random words

OFF = 7
#            g1   g2   sx sy yneg shape
RECIPES = {
    'wash':    (0,   OFF, 0, 0, 0, 0),
    'cloud':   (0,   0,   0, 0, 0, 0),
    'mottle':  (OFF, 1,   0, 0, 0, 0),
    'agate':   (2,   1,   0, 0, 0, 1),
    'burl':    (3,   0,   0, 0, 0, 1),
    'topo':    (3,   OFF, 0, 0, 0, 2),
    'crackle': (OFF, 2,   0, 0, 0, 2),
    'camo':    (1,   0,   0, 0, 0, 3),
    'vein':    (1,   OFF, 0, 0, 0, 4),
    'vein2':   (0,   0,   0, 0, 0, 4),
    'spots':   (OFF, 1,   0, 0, 0, 5),
    'islands': (0,   1,   0, 0, 0, 5),
    'marble':  (2,   1,   3, 0, 0, 1),
    'wood':    (2,   0,   0, 4, 0, 1),
    'grain':   (1,   0,   0, 5, 0, 1),
    'tiger':   (2,   0,   3, 2, 1, 6),
    'zebra':   (2,   1,   0, 3, 0, 6),
    'ripple':  (OFF, 2,   3, 3, 0, 1),
    'fur':     (1,   1,   4, 5, 1, 1),
    'strata':  (1,   1,   0, 3, 0, 7),
    'dune':    (1,   OFF, 2, 4, 1, 1),
    'lava':    (2,   2,   0, 0, 0, 6),
}

def C(ha, sa, hb, sb, dh=-1, ds=1):
    return (ha, sa, hb, sb, dh, ds)

# 8 slots per set, dark bin -> light bin: (recipe, colour)
SETS = [
    ('Stone',  [('vein',    C(1, 0, 13, 1)), ('marble', C(6, 1, 7, 1)),  ('agate',   C(7, 1, 6, 2)),  ('camo',    C(13, 1, 0, 0)),
                ('vein2',   C(7, 1, 0, 0)),  ('strata', C(6, 1, 8, 1)),  ('cloud',   C(8, 1, 0, 0)),  ('crackle', C(0, 0, 7, 1))]),
    ('Timber', [('wood',    C(5, 1, 6, 2)),  ('burl',   C(6, 2, 5, 1)),  ('grain',   C(7, 2, 6, 1)),  ('wood',    C(7, 1, 6, 2)),
                ('burl',    C(8, 1, 6, 2)),  ('dune',   C(7, 1, 8, 1)),  ('grain',   C(8, 1, 7, 1)),  ('wash',    C(8, 1, 7, 1))]),
    ('Tide',   [('ripple',  C(15, 2, 13, 1)), ('agate', C(14, 2, 12, 2)), ('cloud',  C(13, 2, 15, 1)), ('ripple', C(12, 2, 14, 1)),
                ('islands', C(13, 1, 11, 2)), ('topo',  C(12, 1, 14, 2)), ('mottle', C(13, 1, 0, 0)),  ('wash',   C(12, 1, 0, 0))]),
    ('Moss',   [('spots',   C(11, 2, 9, 1)), ('camo',   C(10, 2, 8, 2)), ('islands', C(9, 2, 11, 2)), ('vein',    C(10, 1, 8, 2)),
                ('camo',    C(9, 1, 11, 2)), ('mottle', C(8, 2, 10, 1)), ('crackle', C(9, 1, 7, 1)),  ('cloud',   C(8, 1, 10, 1))]),
    ('Hide',   [('spots',   C(6, 2, 0, 0)),  ('tiger',  C(6, 2, 5, 1)),  ('zebra',   C(0, 0, 7, 1)),  ('spots',   C(7, 2, 5, 2)),
                ('fur',     C(7, 2, 6, 1)),  ('tiger',  C(7, 1, 6, 2)),  ('fur',     C(8, 1, 7, 1)),  ('spots',   C(8, 1, 6, 2))]),
    ('Ember',  [('lava',    C(4, 2, 6, 2)),  ('vein',   C(5, 2, 7, 2)),  ('agate',   C(5, 2, 7, 2)),  ('lava',    C(6, 2, 4, 2)),
                ('burl',    C(6, 2, 8, 2)),  ('islands', C(7, 2, 5, 2)), ('vein2',   C(7, 2, 5, 1)),  ('cloud',   C(8, 2, 6, 1))]),
    ('Orchid', [('agate',   C(2, 2, 12, 2)), ('ripple', C(3, 2, 0, 2)),  ('burl',    C(1, 2, 10, 2)), ('zebra',   C(4, 2, 13, 2)),
                ('islands', C(14, 2, 3, 2)), ('agate',  C(2, 2, 8, 2)),  ('topo',    C(12, 2, 3, 2)), ('camo',    C(3, 1, 12, 1))]),
    ('Paper',  [('mottle',  C(7, 1, 0, 0)),  ('cloud',  C(7, 1, 0, 0)),  ('grain',   C(7, 1, 0, 0)),  ('crackle', C(0, 0, 7, 1)),
                ('wash',    C(7, 1, 0, 0)),  ('mottle', C(8, 1, 0, 0)),  ('topo',    C(7, 1, 0, 0)),  ('cloud',   C(0, 0, 7, 1))]),
]
SET_NAMES = [n for n, _ in SETS]

def recipe_word(name):
    g1, g2, sx, sy, yneg, shape = RECIPES[name]
    return (g1 << 13) | (g2 << 10) | (sx << 7) | (sy << 4) | (yneg << 3) | shape

def colour_word(c):
    ha, sa, hb, sb, dh, ds = c
    return (ha << 12) | (sa << 10) | (hb << 6) | (sb << 4) | ((dh & 7) << 1) | ds

def srom_words():
    w = []
    for _, slots in SETS:
        for rec, col in slots:
            w += [recipe_word(rec), colour_word(col)]
    return w + [0] * (256 - len(w))                # declared 256 deep (power of two)

def _vhdl_array(name, words):
    rows = []
    for i in range(0, len(words), 8):
        rows.append('        ' + ', '.join(f'x"{v:04X}"' for v in words[i:i + 8]))
    return f'    constant {name} : t_rom16 := (\n' + ',\n'.join(rows) + '\n    );\n'

def vhdl_consts():
    return ('    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);\n'
            '    -- noise ROM: 256 random words\n' + _vhdl_array('C_NROM', NROM) +
            '    -- set ROM: 2 words per (set, slot): recipe, colours (see tables.py)\n' + _vhdl_array('C_SROM', srom_words()))

def stamp(path):
    s = path.read_text()
    a = s.index('    -- TABLES BEGIN'); b = s.index('    -- TABLES END')
    a = s.index('\n', a) + 1
    path.write_text(s[:a] + vhdl_consts() + s[b:])

if __name__ == '__main__':
    stamp(pathlib.Path(__file__).resolve().parent / 'veneer.vhd')
    print('tables stamped')
