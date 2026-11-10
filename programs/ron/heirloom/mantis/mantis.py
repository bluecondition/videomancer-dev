#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
MANTIS v0.1.1 - build hook.  Writes mantis_pkg.vhd next to this file (pure python: the builder runs it with the
system python3; imodel.py in .heirloom_work/rb_mantis imports the same tables).

Fitted to the clip WIN_20230906_22_43_27 (still at 2.5 s; see .heirloom_work/rb_mantis/notes.md).  The patch:
three colour keyers, each fed a vertical skewed-triangle oscillator (2.9 cycles per frame) at its own gain plus a
static kaleidoscope shape signal, all scrolled by one slow phase (one colour cycle per 8.93 s):
    channel_c = tone_c( cos(2 pi (psi_c(y) - tau)) + L_c(X, Y) ),   c = R, G, B
    psi_c     = a_c + m_c y + A_c w(frac((y - 150 - off_c) / P)),  w = skewed triangle (rise 81 %), off_G = P / 2
    L_c       = row_c(Y) + col_c(X) + sum_k W_ck alpha_k / 3 + rip_c       (X = |x - 1075|, Y = |y - 518|)
alpha_k = 0..3: the 2-bit soft keys of the 13 shape elements (wings, arms, pillars, lobes ...), see mantis.vhd.

Package contents:
  C_MT_EXP2   256 x 16  round(32768 * 2^(i/256))
  C_MT_COS    512 x 8   round(64 cos(2 pi i / 512)) (signed): the vertical wave, Q6
  C_MT_SIN    256 x 8   round(127 sin(2 pi (i + 0.5) / 256)) (signed): the wood grain
  C_MT_ROW    128 x 32  {rowR, rowG, rowB (Q6 signed), pillar half-width (1/1024 period)} for Y = 8 i .. 8 i + 7
  C_MT_COL    128 x 32  {colR, colG, colB (Q6 signed), 0} for X = 16 i .. 16 i + 15
  C_MT_PIL    512 x 16  {s (9b: distance to the nearest pillar, 1/1024 period), sag (7b px)} for q = (i + 0.5) / 512
  C_MT_TONE_R/G/B 256 x 32 {Y10, Cr10, Cb10} contributions for z = i - 128 (Q6): luma pre-scaled 16/13
  C_MT_P0..P4  16 x 27  pair weights {R9, G9, B9} (Q6 signed) = W_a alpha_a / 3 + W_b alpha_b / 3 for the element
                        pairs (oleaf, ileaf), (above, armedge), (cap, eye), (lobe, pil), (body, node); index 4 alpha_a + alpha_b
Colours in the hardware convention (U register = Cr, V register = Cb).
"""
import math, os

# vertical oscillator (fitted on the clip, rms 0.011 cycles)
VP, VR, VY0 = 371.5, 0.81, 150.0
AMA = [(-0.444528, -0.000461157, 0.290220),     # R: a, m (cycles / line), A
       (-0.461546, -0.000454274, 0.317099),     # G (half a period late)
       (-0.007141, +0.002230122, -0.201024)]    # B
GAIN = [0.7, 0.7, 1.0]
HI = [250.0, 228.0, 250.0]
T_CYCLE = 8.93

NAMES = ['oleaf', 'stripe', 'ileaf', 'dark', 'above', 'armedge', 'cap', 'lobe', 'eye', 'pil', 'spine', 'node', 'body']
W = [[96, 29, 80, 2, 85, -107, 23, 48, 43, 30, 6, 29, 24],
       [31, 4, 57, -1, 52, -34, -34, 96, -38, -55, -39, -11, -3],
       [54, 22, 48, 2, 38, -39, 25, 24, 93, 70, 25, 29, 23]]
ROW = [[-81, -79, -76, -72, -68, -64, -59, -55, -50, -46, -42, -38, -36, -34, -33, -33, -33, -32, -31, -31, -31, -32,
        -33, -33, -34, -36, -38, -40, -42, -42, -41, -41, -42, -44, -47, -50, -53, -59, -67, -77, -85, -90, -92, -92,
        -92, -90, -85, -78, -72, -68, -66, -65, -66, -67, -69, -68, -67, -66, -64, -63, -63, -63, -63, -62, -57, -48,
        -44, -43, -51, -67],
       [-31, -31, -32, -32, -32, -33, -33, -32, -32, -30, -28, -28, -28, -27, -28, -32, -36, -39, -43, -47, -49, -51,
        -53, -52, -49, -48, -49, -49, -47, -44, -41, -39, -38, -35, -32, -30, -30, -29, -28, -27, -26, -26, -26, -26,
        -27, -28, -29, -30, -30, -31, -32, -34, -35, -33, -27, -23, -22, -25, -30, -37, -44, -51, -60, -67, -74, -79,
        -79, -74, -69, -69],
       [-57, -55, -51, -48, -45, -43, -40, -38, -37, -36, -34, -32, -29, -27, -26, -25, -23, -20, -20, -21, -24, -25,
        -26, -26, -27, -27, -28, -29, -30, -31, -31, -32, -32, -34, -36, -40, -43, -45, -47, -49, -51, -55, -57, -60,
        -62, -65, -66, -66, -62, -58, -54, -50, -47, -46, -47, -47, -46, -45, -45, -46, -47, -47, -46, -45, -45, -43,
        -41, -38, -46, -68]]
COL = [[-17, -18, -20, -21, -19, -19, -19, -17, -15, -13, -7, -5, -9, -12, -12, -7, 1, 8, 13, 17, 16, 12, 11, 10, 9, 6,
        4, 2, 2, 5, 6, 5, 3, 2, 2, 2, 3, 4, 5, 6, 6, 4, 2, 1, 2, 4, 3, 0, -1, 2, 7, 9, 10, 10, 9, 8, 8, 4, -1, 0, 7,
        15, 19, 18, 16, 14, 2, -17, -22, -21, -26, -31],
       [-3, -3, -1, 3, 6, 6, 7, 9, 8, 5, 0, -1, 3, 11, 18, 19, 14, 10, 10, 11, 12, 18, 24, 25, 22, 16, 10, 7, 5, 2, -1,
        1, 3, 4, 3, 3, 3, 3, 2, 1, -1, -4, -5, -4, -3, -1, 0, -1, -4, -7, -6, -3, 0, 0, 0, -2, -6, -11, -18, -23, -22,
        -16, -10, -4, 0, 1, -12, -32, -38, -37, -41, -45],
       [-15, -16, -16, -16, -17, -18, -17, -17, -17, -14, -8, -5, -8, -13, -16, -13, -7, 1, 11, 17, 12, 5, 2, 1, 2, 2,
        1, 0, 1, 4, 5, 5, 3, 1, 1, 1, 2, 3, 4, 5, 5, 6, 7, 7, 7, 7, 7, 6, 5, 6, 9, 10, 11, 10, 9, 9, 10, 9, 8, 12, 15,
        16, 16, 12, 9, 7, 2, -7, -15, -22, -29, -34]]
# pillars: pairs 160 px apart, period 490 px, the first at x = 120; lobe sag 0.00405 u (g - u) px
PP, PD, P0, LOBA = 490.0, 160.0, 120.0, 0.00405


def pwn(y):
    """pillar half-width in 1/1024 of the period for rows Y = y .. y + 7: 16 + (Y - 300)+ 26 / 256 px, capped at 46"""
    pw = min(16 + ((max(y + 4 - 300, 0) * 26) >> 8), 46)
    return (pw * 535) >> 8


def tone(z, ch):
    """channel level 0..HI for z (float, 1.0 = full cos swing): a soft key"""
    v = min(max(0.5 + GAIN[ch] * z, 0.0), 1.0)
    v = v * v * (3 - 2 * v) * 0.6 + v * 0.4
    return HI[ch] * v


YC = [0.299, 0.587, 0.114]
CRC = [0.5, -0.418688, -0.081312]
CBC = [-0.168736, -0.331264, 0.5]


def tone_word(i, ch):
    v = tone((i - 128) / 64.0, ch) / 255.0
    y = round(YC[ch] * v * 876 * 16.0 / 13.0)
    cr = round(CRC[ch] * v * 896)
    cb = round(CBC[ch] * v * 896)
    return (y << 20) | ((cr & 0x3FF) << 10) | (cb & 0x3FF)


def build():
    exp2 = [round(32768 * 2 ** (i / 256.0)) for i in range(256)]
    cos = [round(64 * math.cos(2 * math.pi * i / 512)) & 0xFF for i in range(512)]
    sin = [round(127 * math.sin(2 * math.pi * (i + 0.5) / 256)) & 0xFF for i in range(256)]
    row = []
    for i in range(128):
        j = min(i, 69)
        row.append(((ROW[0][j] & 0xFF) << 24) | ((ROW[1][j] & 0xFF) << 16) | ((ROW[2][j] & 0xFF) << 8) | pwn(8 * j))
    col = []
    for i in range(128):
        j = min(i, 71)
        col.append(((COL[0][j] & 0xFF) << 24) | ((COL[1][j] & 0xFF) << 16) | ((COL[2][j] & 0xFF) << 8))
    pil = []
    g1 = PD / PP
    for k in range(512):
        q = (k + 0.5) / 512
        if q < g1:
            u, g = q, g1
        else:
            u, g = q - g1, 1 - g1
        s = min(round(min(u, g - u) * 1024), 511)
        sag = min(round(LOBA * u * (g - u) * PP * PP), 127)
        pil.append((s << 7) | sag)
    tr = [tone_word(i, 0) for i in range(256)]
    tg = [tone_word(i, 1) for i in range(256)]
    tb = [tone_word(i, 2) for i in range(256)]
    return dict(EXP2=exp2, COS=cos, SIN=sin, ROW=row, COL=col, PIL=pil, TR=tr, TG=tg, TB=tb)


def wterm(ch, el, a):
    if NAMES[el] == 'node':      # the node element was cut for the LC budget
        return 0
    return int(math.floor(W[ch][el] * a / 3.0 + 0.5))


def vhd_array(name, typ, vals, width):
    digits = (width + 3) // 4
    lines = []
    for i in range(0, len(vals), 8):
        lines.append('        ' + ', '.join('x"%0*X"' % (digits, v) for v in vals[i:i + 8]))
    return '    constant %s : %s := (\n%s\n    );\n' % (name, typ, ',\n'.join(lines))


PAIRS = [('oleaf', 'ileaf'), ('above', 'armedge'), ('cap', 'eye'), ('lobe', 'pil'), ('body', 'node')]


def pair_rom(p):
    a, b = [NAMES.index(n) for n in PAIRS[p]]
    out = []
    for i in range(16):
        aa, ab = i >> 2, i & 3
        w = [wterm(ch, a, aa) + wterm(ch, b, ab) for ch in range(3)]
        out.append(((w[0] & 0x1FF) << 18) | ((w[1] & 0x1FF) << 9) | (w[2] & 0x1FF))
    return out


def vhd_wfunc(name, ch):
    s = ['    function %s(el : integer; a : unsigned(1 downto 0)) return signed is\n' % name,
         '    begin\n', '        case el is\n']
    for el in range(len(NAMES)):
        t = [wterm(ch, el, a) for a in range(4)]
        s.append('            when %d => -- %s\n' % (el, NAMES[el]))
        s.append('                case to_integer(a) is\n')
        for a in (1, 2, 3):
            s.append('                    when %d => return to_signed(%d, 8);\n' % (a, t[a]))
        s.append('                    when others => return to_signed(0, 8);\n')
        s.append('                end case;\n')
    s.append('            when others => return to_signed(0, 8);\n')
    s.append('        end case;\n    end function;\n')
    return ''.join(s)


def write_pkg(path):
    t = build()
    s = ['-- Generated by mantis.py -- do not edit.  Copyright (C) 2026 czartroniks, GPL-3.0-only\n',
         'library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n\npackage mantis_pkg is\n',
         '    type t_mt_u16x256 is array (0 to 255) of unsigned(15 downto 0);\n',
         '    type t_mt_w8x512 is array (0 to 511) of std_logic_vector(7 downto 0);\n',
         '    type t_mt_w8x256 is array (0 to 255) of std_logic_vector(7 downto 0);\n',
         '    type t_mt_w32x128 is array (0 to 127) of std_logic_vector(31 downto 0);\n',
         '    type t_mt_w16x512 is array (0 to 511) of std_logic_vector(15 downto 0);\n',
         '    type t_mt_w32x256 is array (0 to 255) of std_logic_vector(31 downto 0);\n',
         '    type t_mt_w27x16 is array (0 to 15) of std_logic_vector(26 downto 0);\n']
    s.append(vhd_array('C_MT_EXP2', 't_mt_u16x256', t['EXP2'], 16))
    s.append(vhd_array('C_MT_COS', 't_mt_w8x512', t['COS'], 8))
    s.append(vhd_array('C_MT_SIN', 't_mt_w8x256', t['SIN'], 8))
    s.append(vhd_array('C_MT_ROW', 't_mt_w32x128', t['ROW'], 32))
    s.append(vhd_array('C_MT_COL', 't_mt_w32x128', t['COL'], 32))
    s.append(vhd_array('C_MT_PIL', 't_mt_w16x512', t['PIL'], 16))
    s.append(vhd_array('C_MT_TONE_R', 't_mt_w32x256', t['TR'], 32))
    s.append(vhd_array('C_MT_TONE_G', 't_mt_w32x256', t['TG'], 32))
    s.append(vhd_array('C_MT_TONE_B', 't_mt_w32x256', t['TB'], 32))
    for p in range(len(PAIRS)):
        vals = pair_rom(p)
        lines = ',\n'.join('        "%s"' % format(v, '027b') for v in vals)
        s.append('    constant C_MT_P%d : t_mt_w27x16 := (  -- %s, %s\n%s\n    );\n' % (p, PAIRS[p][0], PAIRS[p][1], lines))
    s.append('end package mantis_pkg;\n')
    open(path, 'w').write(''.join(s))


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    write_pkg(os.path.join(here, 'mantis_pkg.vhd'))
