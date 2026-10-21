#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FACET v0.5 - build hook.  Generates facet_pkg.vhd (pure Python, no numpy):

  C_CO 512 x 32, C_CI 1024 x 32, C_TG 512 x 32: colour ramps over the Lambda
        gradient D = Y - 1.05|X - axis|, entry = (D + 1024) / 8, selected by the
        address's top bits: CO = background / purple (wedge, floor V); CI = dark
        green / light green (the halo beside the red) / red (inside an eye) /
        the light-dark half-way colour of the halo's soft edge;
        TG = the white edge line / the x-order shadow (both derived from the
        background).  Word = Y(10) & U(10) & V(10) & "00",
        hardware convention (U = Cr, V = Cb), luma pre-boosted by 16/13 so the
        output conditioning's 13/16 luma scale lands on the photo's levels.
  C_LS  512 x 16: the eye shape s(t) = sin(pi/2 * (0.4545 - t) / 0.4979) (signed
        10 bits, x511; negative past the tip) & the perpendicular factor c(t)
        (6 bits, x63) for t = folded lens phase in [0, 0.5).
  C_INC 256 x 16: lens-phase increment per Q.1 virtual unit (24-bit phase),
        n = 0.75 * 8^(i/255) eyes per 1920 virtual px.
  f_*   small profile functions (x-order edge shadow / white line, rims,
        outlines, the lens-edge blend), weights in 1/8.

All colours were fitted to the photo (WIN_20230602_01_26_17_Pro.jpg) by
.heirloom_work/rb_facet/fitpal.py (medians of each region binned by D).
"""
import math, os

HERE = os.path.dirname(os.path.abspath(__file__))
WHITE_U, WHITE_V = 478, 572

# display-space (photo) ramps: (D, Y, U=Cr, V=Cb), 10-bit
PAL = {
 'BG': [(-1100, 238, 390, 760), (-925, 253, 374, 760), (-525, 284, 353, 702), (-25, 319, 326, 613),
        (475, 357, 298, 529), (975, 394, 273, 456), (1200, 400, 266, 430)],
 'PU': [(-1100, 160, 600, 585), (-275, 142, 589, 568), (25, 123, 579, 550), (425, 101, 558, 526),
        (825, 85, 532, 512), (1100, 70, 512, 512)],
 'RD': [(-1100, 200, 730, 437), (-275, 181, 710, 445), (225, 161, 677, 456), (725, 138, 638, 470),
        (1100, 125, 615, 480)],
 'LT': [(-1100, 92, 501, 497), (-475, 105, 485, 491), (-375, 116, 477, 482), (-275, 131, 463, 474),
        (-175, 177, 432, 448), (-75, 249, 410, 427), (175, 373, 391, 407), (375, 483, 389, 411),
        (575, 613, 403, 431), (775, 720, 425, 460), (1100, 760, 440, 470)],
 'DK': [(-1100, 88, 503, 499), (-775, 92, 501, 497), (-475, 106, 481, 488), (-175, 127, 466, 476),
        (1100, 131, 463, 474)],
}

def interp(x, pts):
    if x <= pts[0][0]: return pts[0][1:]
    for a, b in zip(pts, pts[1:]):
        if x <= b[0]:
            f = (x - a[0]) / (b[0] - a[0])
            return tuple(a[i] + (b[i] - a[i]) * f for i in (1, 2, 3))
    return pts[-1][1:]

def boost(y):
    return min(1015, max(64, int(round(64 + (y - 64) * 16 / 13))))

def pal_entry(name, i):
    D = i * 8 - 1024 + 4
    y, u, v = interp(D, PAL[name])
    return boost(y), int(round(u)), int(round(v))

def pal_table(name):
    return [pal_entry(name, i) for i in range(256)]

def sh_entry(i):
    """x-order shadow colour: the background darkened (luma x0.34, Cr x0.3, Cb + 25)"""
    y, u, v = pal_entry('BG', i)
    d = y - 64; e = u - 512
    return 64 + (d >> 2) + (d >> 4) + (d >> 5), 512 + (e >> 2) + (e >> 5) + (e >> 6), min(v + 25, 700)

def wh_entry(i):
    """white edge line: half the background luma lifted (photo peaks Y 515-655)"""
    y, u, v = pal_entry('BG', i)
    return 534 + (y >> 1), WHITE_U, WHITE_V

def rom_co():   # address = AB & idx: background / purple
    return pal_table('BG') + pal_table('PU')
def rom_ci():   # address = cls(2) & idx: 0 dark green, 1 light, 2 red, 3 light/dark half (halo edge)
    lt, dk = pal_table('LT'), pal_table('DK')
    mid = [tuple((a + b + 1) // 2 for a, b in zip(l, d)) for l, d in zip(lt, dk)]
    return dk + lt + pal_table('RD') + mid
def rom_tg():   # address = sel & idx: 0 white line, 1 shadow
    return [wh_entry(i) for i in range(256)] + [sh_entry(i) for i in range(256)]

# eye shape
UT, LT = 0.4545, 0.4979
def ls_entry(i):
    t = (i + 0.5) / 1024.0
    a = max(-1.0, min(1.0, (UT - t) / LT))
    s = int(round(511 * math.sin(math.pi / 2 * a)))
    # slope dh/du at the photo's Hl = 308, P = 944
    dh = 308 * math.cos(math.pi / 2 * a) * (math.pi / 2) / (LT * 944) if a < 1 else 0.0
    c = int(round(63 / math.sqrt(1 + dh * dh)))
    return s, c

def inc_entry(i):
    n = 0.75 * 8 ** (i / 255.0)
    return int(round(n * 2 ** 24 / 3840.0))

# profiles: (x, w) breakpoints, weight 0..1 -> 1/16
def prof(x, pts):
    if x <= pts[0][0]: return pts[0][1]
    for a, b in zip(pts, pts[1:]):
        if x <= b[0]:
            return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])
    return pts[-1][1]

EDGE_W = [(-26, 0), (-20, 0.17), (-16, 0.45), (-12, 0.75), (-8, 0.95), (-5, 0.95), (-3.5, 0.6), (-3, 0.55), (-2, 0.78),
          (0, 0.85), (2, 0.8), (4, 0.62), (6, 0.36), (8, 0.17), (10, 0.07), (13, 0)]
TAIL = [(2, 0), (6, 1.0), (10, 0.75), (16, 0.35), (24, 0)]           # x 55 on V
RED_OL = [(-10, 0), (-5, 0.5), (-1, 0.7), (2, 0.7), (6, 0.4), (10, 0)]
RIM_L = [(7, 0), (11, 0.5), (14, 0.85), (17, 0.8), (22, 0.5), (30, 0.25), (40, 0.04), (45, 0)]
RIM_R = [(-1, 0), (1, 0.35), (4, 0.78), (7, 0.42), (10, 0.04), (12, 0)]
DIP_L = [(-3, 0), (0, 0.45), (6, 0.5), (9, 0.0)]
DD = [(2, 0), (5, 0.3), (9, 0.35), (14, 0.15), (22, 0)]
ALPHA = [(-7, 0.0), (-3, 0.2), (0, 0.5), (3, 0.82), (6, 1.0)]
ALPHA_V = [(-12, 0.0), (-8, 0.2), (-5, 0.5), (-2, 0.82), (1, 1.0)]
HB = [(-14, 0), (-8, 0.25), (-3, 0.6), (0, 0.85), (3, 1.0)]

def lut(pts, lo, hi, scale=8):
    return [int(round(scale * prof(x, pts))) for x in range(lo, hi + 1)]

# index ranges (clamped in the RTL)
LUTS = {
    'EDGE': (EDGE_W, -32, 31, 8), 'TAIL': (TAIL, -32, 31, 55), 'REDOL': (RED_OL, -32, 31, 8),
    'RIML': (RIM_L, -32, 63, 8), 'RIMR': (RIM_R, -32, 63, 8), 'DIPL': (DIP_L, -32, 63, 8),
    'DD': (DD, -32, 63, 8), 'ALPHA': (ALPHA, -32, 63, 8), 'ALPHAV': (ALPHA_V, -32, 63, 8),
}
def luts():
    return {k: (lo, lut(p, lo, hi, sc)) for k, (p, lo, hi, sc) in LUTS.items()}

# colour targets (boosted)


RIMW = (boost(820), 462, 496)
DKC = (boost(70), 500, 500)

def word(y, u, v):
    return '"%s"' % (format(y, '010b') + format(u, '010b') + format(v, '010b') + '00')

def gen():
    L = []
    L.append('-- Generated by facet.py -- do not edit.')
    L.append('library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n')
    L.append('package facet_pkg is')
    for nm, tab in (('CO', rom_co()), ('CI', rom_ci()), ('TG', rom_tg())):
        n = len(tab)
        L.append('    type t_%s is array (0 to %d) of std_logic_vector(31 downto 0);' % (nm.lower(), n - 1))
        L.append('    constant C_%s : t_%s := (' % (nm, nm.lower()))
        L.append(',\n'.join('        ' + ', '.join(word(*tab[i]) for i in range(r, r + 4)) for r in range(0, n, 4)) + ');')
    L.append('    type t_ls is array (0 to 511) of std_logic_vector(15 downto 0);')
    L.append('    constant C_LS : t_ls := (')
    ent = []
    for i in range(512):
        s, c = ls_entry(i)
        ent.append('"%s%s"' % (format(s & 0x3FF, '010b'), format(c, '06b')))
    L.append(',\n'.join('        ' + ', '.join(ent[r:r + 8]) for r in range(0, 512, 8)) + ');')
    L.append('    type t_inc is array (0 to 255) of std_logic_vector(15 downto 0);')
    L.append('    constant C_INC : t_inc := (')
    ent = ['"%s"' % format(inc_entry(i), '016b') for i in range(256)]
    L.append(',\n'.join('        ' + ', '.join(ent[r:r + 8]) for r in range(0, 256, 8)) + ');')
    for k, (lo, vals) in luts().items():
        w = max(1, max(vals).bit_length())
        L.append('    function f_%s(x : signed) return unsigned;' % k.lower())
    L.append('    constant C_RIMW_Y : integer := %d;' % RIMW[0])
    L.append('    constant C_DKC_Y : integer := %d;' % DKC[0])
    L.append('end package facet_pkg;\n')
    L.append('package body facet_pkg is')
    for k, (lo, vals) in luts().items():
        w = max(1, max(vals).bit_length())
        L.append('    -- %s: index %d .. %d (clamped), width %d' % (k, lo, lo + len(vals) - 1, w))
        L.append('    function f_%s(x : signed) return unsigned is' % k.lower())
        L.append('    begin')
        L.append('        case to_integer(x) is')
        groups = {}
        for i, v in enumerate(vals):
            groups.setdefault(v, []).append(lo + i)
        default = max(groups, key=lambda v: len(groups[v]))
        for v, xs in sorted(groups.items()):
            if v == default: continue
            L.append('            when %s => return to_unsigned(%d, %d);' % (' | '.join(str(x) for x in xs), v, w))
        L.append('            when others => return to_unsigned(%d, %d);' % (default, w))
        L.append('        end case;')
        L.append('    end function;')
    L.append('end package body facet_pkg;')
    return '\n'.join(L) + '\n'

if __name__ == '__main__':
    with open(os.path.join(HERE, 'facet_pkg.vhd'), 'w') as f:
        f.write(gen())
    print('facet.py: wrote facet_pkg.vhd')
