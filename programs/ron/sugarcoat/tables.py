#!/usr/bin/env python3
"""tables.py -- the SUGARCOAT v2 ROMs, in one place: palette, candies, themes.

  PAL  (256 x 16, 64 used)  palette: [15:10] Y / 16   [9:5] U / 32   [4:0] V / 32  (BT.601, 10 bit)
  GROM (256 x 16)           2 words per candy id (0..127): the geometry recipe
        word 0: [4:0] shape  [6:5] variant  [8:7] size  [9] bias  [10] turn  [11] stagger
                [12] half stagger  [13] wide  [15:14] jitter
        word 1: [1:0] shading  [3:2] grain  [4] grain on the piece  [5] random quarter turn
                [6] 1-D hash (colour per column)  [7] always sprinkled
  CROM (256 x 16)           2 words per candy id: the colours (palette indices)
        word 0: [5:0] A (ground)  [11:6] B (piece)  [13:12] assortment (0 one colour, 1 pair,
                2 four, 3 eight: the low bits of B come from the cell hash / band index)
        word 1: [5:0] C (detail)
  TROM (256 x 8)            theme map: [bank][theme 2:0][class 3:0] -> candy id.  Classes 0..7
                            are the luma slots (dark -> light), 8..15 the hue sectors.

Shapes, variants and what the fields mean: see p_fill in sugarcoat.vhd; model.py mirrors it
operation for operation.  `python3 tables.py` rewrites the constants between the TABLES
markers of sugarcoat.vhd and prints the candy list."""
import pathlib

# ----------------------------------------------------------------------------------------
# palette (sRGB 0..255); blocks of 8 so an assortment can pick 2 / 4 / 8 neighbours
# ----------------------------------------------------------------------------------------
PALETTE = [
    # 0..7 brights (sprinkles, jelly beans, gumballs)
    ('red', (226, 35, 45)), ('orange', (250, 130, 30)), ('yellow', (252, 212, 40)), ('green', (70, 190, 70)),
    ('blue', (50, 140, 235)), ('purple', (140, 80, 200)), ('hot pink', (245, 90, 160)), ('white', (250, 250, 250)),
    # 8..15 pastels: warm four, cool four
    ('p pink', (250, 180, 200)), ('p orange', (250, 190, 130)), ('p lemon', (250, 235, 150)), ('p white', (252, 250, 245)),
    ('p mint', (170, 230, 190)), ('p blue', (170, 205, 245)), ('p lavender', (205, 180, 235)), ('p aqua', (150, 225, 225)),
    # 16..23 fruit jellies: citrus four, berry four
    ('j orange', (240, 120, 20)), ('j lemon', (235, 200, 30)), ('j lime', (120, 190, 40)), ('j grapefruit', (240, 110, 110)),
    ('j cherry', (190, 20, 50)), ('j grape', (110, 50, 150)), ('j blue', (30, 110, 210)), ('j apple', (90, 170, 50)),
    # 24..31 bakery
    ('dark choc', (62, 36, 24)), ('milk choc', (118, 70, 42)), ('caramel', (200, 135, 50)), ('cookie', (212, 170, 110)),
    ('gingerbread', (150, 90, 45)), ('biscuit', (225, 185, 120)), ('cream', (245, 230, 195)), ('toasted', (95, 55, 30)),
    # 32..39
    ('liquorice', (28, 26, 30)), ('cane red', (205, 25, 40)), ('mint green', (60, 170, 110)), ('pink icing', (245, 150, 185)),
    ('white icing', (250, 248, 242)), ('ivory', (245, 238, 220)), ('butter', (250, 225, 140)), ('jam', (165, 20, 45)),
    # 40..47
    ('cocoa', (40, 24, 18)), ('silver', (190, 195, 205)), ('coconut pink', (245, 130, 170)), ('teal', (40, 170, 170)),
    ('deep blue', (40, 70, 160)), ('peanut', (225, 160, 70)), ('honey', (235, 175, 40)), ('charcoal', (70, 66, 72)),
    # 48..55
    ('pistachio', (165, 200, 110)), ('strawberry', (235, 90, 110)), ('lilac', (190, 150, 220)), ('sky', (120, 190, 240)),
    ('plum', (95, 40, 80)), ('rust', (175, 80, 35)), ('olive', (120, 125, 50)), ('grey', (128, 128, 128)),
]
PALETTE += [('spare', (128, 128, 128))] * (63 - len(PALETTE)) + [('WHITE', (255, 255, 255))]
PIDX = {n: i for i, (n, _) in enumerate(PALETTE)}
WHITE = 63

def yuv10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return 64 + 876 * y, 512 + 896 * (-0.168736 * r - 0.331264 * g + 0.5 * b), 512 + 896 * (0.5 * r - 0.418688 * g - 0.081312 * b)

def pal_word(rgb):
    y, u, v = yuv10(rgb)
    y6 = max(4, min(58, int(round(y / 16.0))))
    u5 = max(0, min(31, int(round(u / 32.0)))); v5 = max(0, min(31, int(round(v / 32.0))))
    return (y6 << 10) | (u5 << 5) | v5

def pal_words():
    return [pal_word(rgb) for _, rgb in PALETTE] + [0x2210] * (256 - len(PALETTE))

# ----------------------------------------------------------------------------------------
# candies
# ----------------------------------------------------------------------------------------
SHAPES = ['full', 'disc', 'ring', 'bean', 'chip', 'square', 'wedge', 'target', 'heart', 'corn', 'stripe',
          'tube', 'check', 'pips', 'wafer', 'ridge', 'zigzag', 'scallop', 'basket', 'quilt', 'rosette', 'holes']
MATTE, SOFT, GLOSSY, JELLY = 0, 1, 2, 3          # shading
G_NONE, G_CRUMB, G_SUGAR, G_NONP = 0, 1, 2, 3     # grain

def K(name, shape, var=0, size=1, A='cream', B='red', C='WHITE', asst=0, shd=SOFT, grn=G_NONE, gp=0,
      jit=0, rot=0, stg=0, hst=0, wide=0, bias=0, turn=0, h1d=0, sprk=0):
    g0 = (SHAPES.index(shape) | (var << 5) | (size << 7) | (bias << 9) | (turn << 10) | (stg << 11)
          | (hst << 12) | (wide << 13) | (jit << 14))
    g1 = shd | (grn << 2) | (gp << 4) | (rot << 5) | (h1d << 6) | (sprk << 7)
    pi = lambda c: c if isinstance(c, int) else PIDX[c]
    c0 = pi(A) | (pi(B) << 6) | (asst << 12)
    c1 = pi(C)
    return dict(name=name, g0=g0, g1=g1, c0=c0, c1=c1)

CANDIES = [
    # ---- chocolate ----
    K('dark chocolate bar', 'pips', 0, 1, B='dark choc', shd=SOFT),
    K('milk chocolate bar', 'pips', 2, 1, B='milk choc', wide=1, stg=1, shd=SOFT),
    K('white chocolate bar', 'pips', 0, 1, B='cream', shd=SOFT),
    K('cookies and cream bar', 'pips', 2, 1, B='p white', C='cocoa', grn=G_CRUMB, gp=1, shd=SOFT),
    K('chocolate truffles', 'disc', 0, 1, A='cocoa', B='milk choc', C='toasted', grn=G_CRUMB, gp=1, shd=SOFT, jit=1, stg=1),
    K('nonpareil chocolates', 'disc', 0, 1, A='cream', B='dark choc', grn=G_NONP, gp=1, shd=SOFT, jit=1, stg=1),
    K('honeycomb toffee', 'holes', 2, 0, B='caramel', C='rust', jit=2, shd=SOFT, wide=1, stg=1),
    K('chocolate fingers', 'tube', 0, 0, A='cocoa', B='milk choc', shd=SOFT, turn=1),
    K('peanut butter cups', 'target', 2, 1, A='peanut', B='milk choc', shd=SOFT, stg=1, jit=1),
    K('caramel drizzle', 'zigzag', 1, 1, B='caramel', C='dark choc', shd=SOFT),
    K('crinkle cookie', 'square', 0, 1, A='cocoa', B='p white', shd=MATTE, grn=G_SUGAR, gp=1, stg=1),
    K('rocky road', 'chip', 3, 0, A='dark choc', B='p white', asst=1, jit=2, rot=1, shd=SOFT),
    # ---- cookies and bakery ----
    K('chocolate chip cookie', 'chip', 3, 0, A='cookie', B='dark choc', C='gingerbread', jit=2, rot=1, grn=G_CRUMB, shd=SOFT),
    K('double chocolate cookie', 'chip', 1, 0, A='toasted', B='cream', C='cocoa', jit=3, rot=1, grn=G_CRUMB, shd=SOFT),
    K('sandwich cookies', 'target', 3, 1, A='white icing', B='cocoa', shd=SOFT, stg=1, jit=1),
    K('iced sugar cookies', 'ring', 0, 1, A='ivory', B='biscuit', C='pink icing', shd=SOFT, stg=1, jit=1, sprk=1),
    K('jam thumbprints', 'ring', 2, 1, A='cream', B='cookie', C='jam', shd=GLOSSY, stg=1, jit=2),
    K('gingerbread icing', 'scallop', 0, 1, B='gingerbread', C='white icing', hst=1, shd=SOFT),
    K('waffle cone', 'wafer', 0, 0, B='biscuit', bias=1, shd=SOFT),
    K('wafer layers', 'stripe', 2, 1, A='biscuit', B='pink icing', C='milk choc', turn=1, shd=MATTE),
    K('shortbread', 'pips', 1, 1, B='butter', C='cookie', shd=SOFT),
    K('battenberg', 'check', 0, 1, A='butter', B='pink icing', C='p lemon', shd=MATTE),
    K('lattice pie', 'basket', 3, 1, B='biscuit', C='jam', bias=1, shd=SOFT),
    K('stroopwafel', 'wafer', 3, 1, B='peanut', C='caramel', bias=1, shd=SOFT),
    # ---- frosting ----
    K('piped buttercream', 'ridge', 0, 0, B='pink icing', shd=SOFT, turn=1),
    K('rosettes', 'rosette', 0, 1, A='p pink', B='p pink', shd=SOFT, stg=1),
    K('shell border', 'scallop', 1, 1, B='butter', hst=1, shd=SOFT),
    K('basketweave piping', 'basket', 0, 1, B='ivory', shd=SOFT),
    K('quilted fondant', 'quilt', 1, 1, B='p white', shd=GLOSSY),
    K('funfetti frosting', 'full', 1, 1, B='white icing', shd=SOFT, grn=G_NONP, gp=1),
    K('chocolate frosting', 'full', 1, 1, B='milk choc', shd=GLOSSY),
    K('doughnut glaze', 'full', 3, 2, B='pink icing', shd=SOFT, sprk=1),
    K('meringue kisses', 'rosette', 3, 0, A='ivory', B='white icing', shd=SOFT, stg=1),
    K('mint frosting', 'zigzag', 0, 0, B='p mint', shd=SOFT),
    # ---- hard candy ----
    K('candy canes', 'tube', 1, 0, A='ivory', B='cane red', C='mint green', shd=GLOSSY),
    K('starlight mints', 'wedge', 0, 1, A='p pink', B='cane red', shd=GLOSSY, stg=1, jit=1),
    K('spearmint starlights', 'wedge', 0, 1, A='p mint', B='mint green', shd=GLOSSY, stg=1, jit=1),
    K('lollipops', 'target', 0, 1, A='p white', B='red', asst=3, shd=GLOSSY, stg=1),
    K('gumballs', 'disc', 0, 1, A='p white', B='red', asst=3, shd=GLOSSY, stg=1, jit=1),
    K('candy sticks', 'tube', 0, 0, A='p white', B='p pink', asst=3, shd=GLOSSY, h1d=1),
    K('butterscotch discs', 'disc', 1, 0, A='cream', B='honey', shd=JELLY, stg=1, jit=1),
    K('cinnamon hearts', 'heart', 0, 0, A='p pink', B='cane red', shd=GLOSSY, stg=1, jit=1),
    K('conversation hearts', 'heart', 1, 1, A='ivory', B='p pink', asst=3, shd=SOFT, stg=1, jit=1),
    K('candy buttons', 'disc', 2, 0, A='white icing', B='p pink', asst=3, shd=SOFT),
    K('candy corn', 'corn', 0, 0, A='toasted', B='yellow', C='orange', shd=SOFT, stg=1),
    K('sugar pearls', 'quilt', 3, 0, B='ivory', shd=GLOSSY),
    K('rock candy', 'heart', 2, 0, A='p white', B='p pink', asst=3, shd=SOFT, stg=1, jit=1),
    K('peppermint bark', 'chip', 0, 0, A='p white', B='cane red', jit=2, rot=1, shd=SOFT),
    # ---- chewy and jelly ----
    K('jelly beans', 'bean', 0, 1, A='p white', B='red', asst=3, shd=GLOSSY, rot=1, stg=1, jit=1),
    K('gummy rings', 'ring', 1, 1, A='ivory', B='j orange', asst=3, shd=JELLY, stg=1, jit=1),
    K('gumdrops', 'disc', 1, 0, A='p white', B='j orange', asst=3, shd=SOFT, grn=G_SUGAR, gp=1, stg=1, jit=1),
    K('fruit slices', 'wedge', 1, 1, A='p white', B='j orange', asst=2, shd=SOFT, stg=1, jit=1),
    K('sour belts', 'stripe', 3, 1, B='red', asst=3, shd=MATTE, grn=G_SUGAR, gp=1, bias=1),
    K('liquorice allsorts', 'square', 3, 0, A='p white', B='liquorice', C='coconut pink', jit=1, rot=1, shd=SOFT),
    K('liquorice twists', 'tube', 2, 0, A='liquorice', B='charcoal', shd=GLOSSY),
    K('red twists', 'tube', 2, 0, A='plum', B='cane red', shd=GLOSSY, turn=1),
    K('saltwater taffy', 'scallop', 3, 1, A='p white', B='p pink', C='p mint', hst=1, shd=SOFT),
    K('marshmallows', 'square', 1, 0, A='p pink', B='p white', asst=1, jit=2, shd=SOFT),
    K('caramels', 'square', 1, 0, A='cream', B='caramel', jit=2, shd=GLOSSY),
    K('turkish delight', 'square', 2, 0, A='p white', B='strawberry', jit=3, rot=1, shd=SOFT, grn=G_SUGAR, gp=1),
    K('speckled eggs', 'bean', 1, 0, A='ivory', B='p mint', C='toasted', asst=2, rot=1, stg=1, jit=1, shd=SOFT, grn=G_CRUMB, gp=1),
    K('chocolate lentils', 'disc', 1, 0, A='milk choc', B='red', asst=3, shd=GLOSSY, stg=1, jit=1),
    K('pink and white capsules', 'bean', 2, 0, A='liquorice', B='hot pink', asst=1, rot=1, jit=3, shd=GLOSSY),
    K('marble fudge', 'scallop', 2, 1, B='milk choc', C='cream', hst=1, shd=SOFT),
    K('neapolitan', 'stripe', 2, 1, A='p white', B='strawberry', C='milk choc', shd=MATTE, bias=1),
    K('nougat', 'chip', 1, 0, A='p white', B='pistachio', asst=1, jit=3, rot=1, shd=MATTE),
    K('bullseyes', 'target', 0, 1, A='p aqua', B='hot pink', asst=1, shd=GLOSSY, stg=1, jit=1),
    # ---- single-colour kin (the hue maps need a candy for every hue) ----
    K('grape gumdrops', 'disc', 1, 0, A='p lavender', B='j grape', shd=SOFT, grn=G_SUGAR, gp=1, stg=1, jit=1),
    K('lavender rosettes', 'rosette', 1, 1, A='lilac', B='p lavender', shd=SOFT, stg=1),
    K('violet rock candy', 'heart', 2, 0, A='p lavender', B='purple', shd=GLOSSY, stg=1, jit=1),
    K('bubblegum', 'disc', 0, 1, A='p pink', B='hot pink', shd=GLOSSY, stg=1, jit=1),
    K('raspberry macarons', 'ring', 2, 1, A='p pink', B='strawberry', C='p white', shd=SOFT, stg=1, jit=1),
    K('cherry drops', 'disc', 1, 0, A='p pink', B='j cherry', shd=JELLY, stg=1, jit=1),
    K('strawberry buttercream', 'ridge', 2, 0, B='strawberry', shd=SOFT),
    K('orange slices', 'wedge', 1, 1, A='p orange', B='j orange', shd=SOFT, stg=1, jit=1),
    K('lemon drops', 'disc', 1, 0, A='p lemon', B='j lemon', shd=JELLY, stg=1, jit=1),
    K('lime slices', 'wedge', 1, 1, A='p lemon', B='j lime', shd=SOFT, stg=1, jit=1),
    K('pistachio macarons', 'ring', 2, 1, A='cream', B='pistachio', C='p white', shd=SOFT, stg=1, jit=1),
    K('green apple rings', 'ring', 1, 1, A='p mint', B='j apple', shd=JELLY, stg=1, jit=1),
    K('mint chip', 'chip', 3, 0, A='p mint', B='dark choc', C='mint green', jit=2, rot=1, grn=G_CRUMB, shd=SOFT),
    K('aqua rock candy', 'heart', 3, 0, A='p aqua', B='teal', shd=GLOSSY, stg=1, jit=1),
    K('teal jelly beans', 'bean', 0, 1, A='p aqua', B='teal', shd=GLOSSY, rot=1, stg=1, jit=1),
    K('aqua fondant', 'quilt', 2, 1, B='p aqua', shd=GLOSSY),
    K('blue raspberry rings', 'ring', 1, 1, A='p blue', B='j blue', shd=JELLY, stg=1, jit=1),
    K('blueberry gumdrops', 'disc', 1, 0, A='p blue', B='deep blue', shd=SOFT, grn=G_SUGAR, gp=1, stg=1, jit=1),
    K('blue buttercream', 'ridge', 0, 0, B='sky', shd=SOFT, turn=1),
    K('cotton candy', 'scallop', 2, 1, B='p pink', C='p blue', hst=1, shd=MATTE, grn=G_SUGAR, gp=1),
    K('chocolate rosettes', 'rosette', 0, 1, A='cocoa', B='milk choc', shd=SOFT, stg=1),
    K('pistachio frosting', 'zigzag', 0, 0, B='pistachio', shd=SOFT, turn=1),
    K('grape jelly beans', 'bean', 3, 0, A='p lavender', B='purple', shd=GLOSSY, rot=1, stg=1, jit=1),
    K('blue lollies', 'wedge', 3, 1, A='p white', B='blue', shd=GLOSSY, stg=1, jit=1),
    K('toblerone', 'corn', 1, 0, A='cocoa', B='milk choc', shd=SOFT, stg=1),
    K('watermelon slices', 'corn', 3, 0, A='p white', B='strawberry', C='j apple', shd=SOFT, stg=1, grn=G_CRUMB, gp=1),
    K('candy gingham', 'check', 3, 0, A='p white', B='cane red', C='p pink', shd=MATTE),
    K('ribbon candy', 'zigzag', 2, 0, B='cane red', C='p white', shd=GLOSSY),
    K('fondant stripes', 'stripe', 1, 1, A='p white', B='p blue', C='p pink', shd=MATTE, bias=1),
]
CIDX = {c['name']: i for i, c in enumerate(CANDIES)}
assert len(CANDIES) <= 128 and len(CIDX) == len(CANDIES)

# ----------------------------------------------------------------------------------------
# themes: 8 luma slots (dark -> light) + 8 hue sectors.  Hue sectors (counter-clockwise
# from +Cb, 45 degrees each): 0 blue-violet, 1 magenta-pink, 2 red, 3 orange-yellow,
# 4 yellow-green, 5 green, 6 teal-cyan, 7 sky blue.
# ----------------------------------------------------------------------------------------
THEMES = []        # filled in after the candies are tuned (see bottom)

def T(name, luma, hue):
    assert len(luma) == 8 and len(hue) == 8
    THEMES.append((name, [CIDX[n] for n in luma], [CIDX[n] for n in hue]))

def trom_words():
    assert len(THEMES) == 16, len(THEMES)
    w = []
    for _, luma, hue in THEMES:
        w += luma + hue
    return w + [0] * (256 - len(w))

def grom_words():
    w = []
    for c in CANDIES: w += [c['g0'], c['g1']]
    return w + [0] * (256 - len(w))

def crom_words():
    w = []
    for c in CANDIES: w += [c['c0'], c['c1']]
    return w + [0] * (256 - len(w))

# ----------------------------------------------------------------------------------------
# VHDL stamping
# ----------------------------------------------------------------------------------------
def _arr(name, typ, words, digits):
    rows = []
    for i in range(0, len(words), 8):
        rows.append('        ' + ', '.join(f'x"{v:0{digits}X}"' for v in words[i:i + 8]))
    return f'    constant {name} : {typ} := (\n' + ',\n'.join(rows) + '\n    );\n'

def vhdl_consts():
    return ('    type t_rom16 is array (0 to 255) of std_logic_vector(15 downto 0);\n'
            '    type t_rom8  is array (0 to 255) of std_logic_vector(7 downto 0);\n'
            '    -- palette: Y / 16, U / 32, V / 32\n' + _arr('C_PAL', 't_rom16', pal_words(), 4)
            + '    -- candy geometry, 2 words per candy\n' + _arr('C_GROM', 't_rom16', grom_words(), 4)
            + '    -- candy colours, 2 words per candy\n' + _arr('C_CROM', 't_rom16', crom_words(), 4)
            + '    -- theme map: bank, theme, class -> candy\n' + _arr('C_TROM', 't_rom8', trom_words(), 2))

def stamp(path):
    s = path.read_text()
    a = s.index('    -- TABLES BEGIN'); b = s.index('    -- TABLES END')
    a = s.index('\n', a) + 1
    path.write_text(s[:a] + vhdl_consts() + s[b:])

# ----------------------------------------------------------------------------------------
# the 16 themes (K1 x S9).  Luma lists are re-sorted by the measured mean luma of each candy
# (LUMA_Y, from `model.py --stats`) so a theme always runs dark -> light.
# hue order: violet, pink, red, orange / yellow, yellow-green, green, teal, blue
# ----------------------------------------------------------------------------------------
import json as _json
_yf = pathlib.Path(__file__).resolve().parent / 'candy_luma.json'
LUMA_Y = _json.loads(_yf.read_text()) if _yf.exists() else {}

def TS(name, luma, hue):
    T(name, sorted(luma, key=lambda n: LUMA_Y.get(n, 0)), hue)

# bank A
TS('Candy Shop', ['dark chocolate bar', 'liquorice twists', 'chocolate lentils', 'candy corn', 'liquorice allsorts', 'candy canes', 'gumballs', 'jelly beans'],
   ['grape gumdrops', 'bubblegum', 'cinnamon hearts', 'butterscotch discs', 'lime slices', 'spearmint starlights', 'aqua rock candy', 'blue raspberry rings'])
TS('Bakery', ['double chocolate cookie', 'sandwich cookies', 'gingerbread icing', 'chocolate chip cookie', 'stroopwafel', 'waffle cone', 'iced sugar cookies', 'meringue kisses'],
   ['lavender rosettes', 'raspberry macarons', 'jam thumbprints', 'shortbread', 'pistachio macarons', 'mint chip', 'mint frosting', 'blue buttercream'])
TS('Chocolate', ['dark chocolate bar', 'chocolate truffles', 'milk chocolate bar', 'chocolate frosting', 'peanut butter cups', 'nonpareil chocolates', 'caramel drizzle', 'white chocolate bar'],
   ['chocolate lentils', 'rocky road', 'red twists', 'honeycomb toffee', 'marble fudge', 'mint chip', 'cookies and cream bar', 'chocolate fingers'])
TS('Frosting', ['chocolate rosettes', 'chocolate frosting', 'doughnut glaze', 'piped buttercream', 'shell border', 'funfetti frosting', 'quilted fondant', 'meringue kisses'],
   ['lavender rosettes', 'rosettes', 'strawberry buttercream', 'basketweave piping', 'pistachio frosting', 'mint frosting', 'aqua fondant', 'blue buttercream'])
TS('Liquorice', ['pink and white capsules', 'liquorice twists', 'red twists', 'sandwich cookies', 'liquorice allsorts', 'neapolitan', 'crinkle cookie', 'cookies and cream bar'],
   ['violet rock candy', 'candy gingham', 'red twists', 'candy corn', 'lemon drops', 'green apple rings', 'teal jelly beans', 'blue lollies'])
TS('Pastel', ['doughnut glaze', 'gummy rings', 'marshmallows', 'speckled eggs', 'conversation hearts', 'saltwater taffy', 'rock candy', 'candy buttons'],
   ['lavender rosettes', 'cotton candy', 'raspberry macarons', 'shell border', 'pistachio macarons', 'mint frosting', 'aqua fondant', 'fondant stripes'])
TS('Gummy', ['red twists', 'sour belts', 'gummy rings', 'gumdrops', 'fruit slices', 'jelly beans', 'turkish delight', 'rock candy'],
   ['grape gumdrops', 'bubblegum', 'cherry drops', 'orange slices', 'lime slices', 'green apple rings', 'teal jelly beans', 'blue raspberry rings'])
TS('Peppermint', ['dark chocolate bar', 'red twists', 'gingerbread icing', 'candy canes', 'starlight mints', 'spearmint starlights', 'peppermint bark', 'quilted fondant'],
   ['grape jelly beans', 'ribbon candy', 'cinnamon hearts', 'caramels', 'pistachio frosting', 'spearmint starlights', 'mint frosting', 'blue lollies'])
# bank B: a second helping of each
TS('Candy Shop B', ['milk chocolate bar', 'red twists', 'peanut butter cups', 'sour belts', 'starlight mints', 'lollipops', 'bullseyes', 'candy buttons'],
   ['violet rock candy', 'doughnut glaze', 'ribbon candy', 'candy corn', 'lemon drops', 'green apple rings', 'candy sticks', 'blue lollies'])
TS('Bakery B', ['rocky road', 'toblerone', 'marble fudge', 'lattice pie', 'wafer layers', 'shortbread', 'battenberg', 'crinkle cookie'],
   ['grape gumdrops', 'iced sugar cookies', 'jam thumbprints', 'waffle cone', 'pistachio macarons', 'mint frosting', 'aqua fondant', 'blue buttercream'])
TS('Chocolate B', ['dark chocolate bar', 'chocolate fingers', 'rocky road', 'toblerone', 'double chocolate cookie', 'chocolate lentils', 'marble fudge', 'cookies and cream bar'],
   ['nonpareil chocolates', 'neapolitan', 'chocolate truffles', 'peanut butter cups', 'caramel drizzle', 'mint chip', 'white chocolate bar', 'sandwich cookies'])
TS('Frosting B', ['chocolate rosettes', 'caramel drizzle', 'strawberry buttercream', 'rosettes', 'mint frosting', 'basketweave piping', 'sugar pearls', 'funfetti frosting'],
   ['violet rock candy', 'doughnut glaze', 'piped buttercream', 'shell border', 'pistachio frosting', 'spearmint starlights', 'aqua fondant', 'cotton candy'])
TS('Liquorice B', ['dark chocolate bar', 'pink and white capsules', 'chocolate truffles', 'liquorice twists', 'rocky road', 'double chocolate cookie', 'nonpareil chocolates', 'liquorice allsorts'],
   ['grape jelly beans', 'pink and white capsules', 'red twists', 'honeycomb toffee', 'candy corn', 'mint chip', 'teal jelly beans', 'chocolate lentils'])
TS('Pastel B', ['bullseyes', 'candy sticks', 'marshmallows', 'turkish delight', 'nougat', 'meringue kisses', 'fondant stripes', 'cotton candy'],
   ['grape gumdrops', 'conversation hearts', 'candy gingham', 'saltwater taffy', 'speckled eggs', 'pistachio macarons', 'aqua rock candy', 'blueberry gumdrops'])
TS('Gummy B', ['chocolate lentils', 'watermelon slices', 'butterscotch discs', 'gumballs', 'lollipops', 'cherry drops', 'lemon drops', 'gumdrops'],
   ['grape jelly beans', 'bubblegum', 'watermelon slices', 'orange slices', 'lemon drops', 'lime slices', 'aqua rock candy', 'blueberry gumdrops'])
TS('Peppermint B', ['liquorice twists', 'toblerone', 'gingerbread icing', 'cinnamon hearts', 'candy canes', 'ribbon candy', 'peppermint bark', 'sugar pearls'],
   ['violet rock candy', 'starlight mints', 'red twists', 'candy corn', 'pistachio frosting', 'spearmint starlights', 'aqua rock candy', 'blue lollies'])

if __name__ == '__main__':
    p = pathlib.Path(__file__).resolve().parent / 'sugarcoat.vhd'
    if p.exists() and '-- TABLES BEGIN' in p.read_text():
        stamp(p); print('tables stamped')
    for i, c in enumerate(CANDIES):
        print(f'{i:3d} {c["name"]}')
