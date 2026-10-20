#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE microcode v2: the per-field frame program for the scanline engine.

Runs from the field anchor (vsync edge after active video):
  1. inputs, video mode, measured raster; T-bar deadband + glide + dead zones
  2. pattern slot (hysteresis) -> recipe; EDGE -> G (key units per px), border
     DK, clearances h0/h1; colours; anchor (ORIGIN path) and rotation (SPIN)
  3. the pattern HANDLER writes a SECTOR SPEC per accumulator (A, and B for
     unions) at the reference size S_ref = 1024 px: per sector the ray
     direction (any scale) and the piece (linear: unit normal + c, or circle:
     centre + radius)
  4. BOUND: the scale f1 at which the field is <= -h1 over the whole frame
     (linear pieces: exact corner projection; circles: far corner inside by
     h1/G), combined per the union op; the T-bar maps f = f1 t (REVERSE: 1-t)
     with an extra lift of h0 over the first 6 % so the ends are clean
  5. COMPILE: for each accumulator, the up list (lines above its anchor) and
     the down list (at / below), each a START record plus one record per
     crossing ray, in increasing-x order, initialised at the list's first
     field line; records go to the DATA memory for the line builder
  6. ports: DK, colours, q per accumulator, list descriptors, END

Units: px in Q.8 (ax8, W8, ...); key units in Q.8 for c (key = 256 per edge
width, G8 = key/px Q.8); angles 65536 per turn; f16 = scale Q.16.
Records: x, dx Q.16; D Q.8; dD Q.16; ddD Q.20; s Q.16; ds Q.20 (sw_scan).
The records are DOUBLE-BUFFERED: the engine computes the NEXT field's set
(CTL 'set' says which) during the current field while the line builder
reads the other; the hardware swaps sets and shadow ports at the anchor.
DATA map (1024 words): spec A 0..143, spec B 144..191, header 192..202,
record set 0 256..549, set 1 576..869, index scratch 896.., ray exits 960..
"""
import math
from sw_uasm import Asm, PBASE

SPEC_A, SPEC_B, HDR = 0, 144, 192          # spec A 24 x 6, spec B 8 x 6, header 11
REC0, REC_STRIDE = 256, 320                 # record set 0 at 256, set 1 at 576 (42 x 7 each)
SCR, EXT = 204, 928                         # scratch: corner sectors (4), corners (8), index list (+16..); ray exits + entries (24 x 4)
NA_MAX, NB_MAX = 24, 8
S_REF = 1024                         # px
PI16 = 32768

#            name            handler    bound   ctr  args...
# handler: REG (n, rho, phase), BAR (angle, abs), CIRC, WEDGE (kind), VERT (table), ...
RECIPES = {
    'MIX':           ('MIX', 0, 1),
    'BAR >':         ('BAR', 1, 1, 0, 0),
    'BAR v':         ('BAR', 1, 1, 16384, 0),
    'DIAGONAL':      ('BAR', 1, 1, 8192, 0),
    'BARN DOOR <>':  ('BAR', 1, 1, 0, 1),
    'BARN DOOR ^v':  ('BAR', 1, 1, 16384, 1),
    'BARN DOOR diag':('BAR', 1, 1, 8192, 1),
    'BOX':           ('BOX', 1, 0),
    'DIAMOND':       ('DIA', 1, 0),
    'CIRCLE':        ('CIRC', 2, 0),
    'POLYGON':       ('REG', 1, 0, 'poly'),
    'STAR WIPE':     ('REG', 1, 0, 'star'),
    'CLOCK':         ('WEDGE', 3, 0, 'clock'),
    'PINWHEEL':      ('WEDGE', 3, 0, 'pin'),
    'DOUBLE SWEEP':  ('WEDGE', 3, 0, 'dsweep'),
    'DOUBLE FAN':    ('WEDGE', 3, 0, 'dfan'),
    'SUNBURST':      ('SUNB', 1, 0),
    'SPOTLIGHT':     ('CIRC', 2, 0),
    'X DOORS':       ('XDOOR', 1, 1),
    'EYE':           ('EYE', 2, 0),
    'CROSS':         ('VPOLY', 1, 0, 'vt_cross'),
    'VEE':           ('VEE', 1, 1, 0),
    'BARN VEE':      ('VEE', 1, 1, 1),
    'TRIANGLE':      ('VPOLY', 1, 0, 'vt_tri'),
    'APERTURE':      ('REG', 1, 0, 'aper'),
}
CLASSIC = ['MIX', 'BAR >', 'BAR v', 'DIAGONAL', 'BARN DOOR <>', 'BARN DOOR ^v', 'BARN DOOR diag', 'X DOORS',
           'BOX', 'DIAMOND', 'CIRCLE', 'EYE', 'CROSS', 'VEE', 'BARN VEE', 'TRIANGLE',
           'POLYGON', 'CLOCK', 'PINWHEEL', 'DOUBLE FAN', 'DOUBLE SWEEP']
CHEESE = ['STAR WIPE', 'SUNBURST', 'APERTURE', 'SPOTLIGHT']

STAR_RHO = {3: .36, 4: .4, 5: .382, 6: .5, 7: .47, 8: .5, 9: .5, 10: .52, 11: .53, 12: .55}

MODE_TAB = {0: (1, 2, 282), 8: (1, 2, 235), 4: (0, 2, 282), 12: (0, 2, 235),
            5: (0, 1, 256), 6: (0, 1, 256), 14: (0, 1, 256),
            1: (1, 0, 256), 2: (1, 0, 256), 10: (1, 0, 256)}      # ilace, class, line height Q.8 px
CLASS_FH = {0: 1080, 1: 720, 2: 534}
WHITE, BLACK = (765, 512, 512), (64, 512, 512)


def ycc(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return (64 + round(876 * y), 512 + round(896 * (r - y) / 1.402), 512 + round(896 * (b - y) / 1.772))


GOLD = ycc(0.80, 0.62, 0.20)
HUE_RED = round(108.7 / 360 * 65536)
HUE_YPH = round(153.6 / 360 * 65536)
MARGIN_PX = 6.0                      # clearance beyond the ramp, px


def build():
    A = Asm()
    e = A.emit
    # window banks: 0 = control phase, 1 = handlers, 2 = bound + list building, 3 = mkrec
    A.wreg(0, 'tq', 'clo', 'chi', 'cmid', 'NS', 'rb', 'seg', 'oi', 'of', 'pk', 'q0', 'q1',
           'th', 'hs', 'hc', 'hy', 'hlink', 'hh', 'cy', 'cuv', 'rec', 'cls', 'HF', 'LwK', 'LbK', 'mw', 'DK', 'bw8',
           'auto', 'aph')
    A.wreg(1, 'cnt', 'elink', 'n', 'S', 'Sh', 'rho', 'al', 'cref', 'ptyp', 'pw3', 'pw4', 'pw5', 'k', 'tip',
           'clink', 'cmlo', 'cmhi', 'nn', 'nlo', 'nhi', 'nf', 'kx', 'absf', 'ang', 'nw', 'wst', 'span', 'a0', 'a1', 'ra', 'spp')
    A.alias(ha='absf', hb='ang', hx8='nw', hy8='wst', mhx='span', mhy='a0', L8='a1', nxu='rho', nyu='al', cd='cref',
            sxn='cmlo', syn='cmhi')
    # vertex-polygon handler (CROSS, TRIANGLE): table, count, index, vertex pair, edge, normal, scales
    A.alias(vtab='span', vcnt='n', vi='k', vx0='a0', vy0='a1', vxo='cmlo', vyo='cmhi', vex='nlo', vey='nhi',
            vL='nf', nxv='kx', nyv='absf', AX='rho', AY='nw', BX='al', BY='wst', vlink='tip')
    A.wreg(2, 'fb', 'blink', 'lup', 'ldn', 'ayl', 'v0up', 'v0dn', 'cnt2', 'j', 'lbase',
           'k0', 'i', 'kk', 'rx1', 'bk', 'bptyp', 'bw3', 'bw4', 'bw5', 'fA', 'fB', 'f1', 'acc',
           'fb0', 'pmax', 'pmin', 'rsec', 'csec', 'cr1', 'cr2', 'tt', 'tin')
    A.alias(fk='lup', cxL='ayl', cxR='v0up', cyT='v0dn', cyB='cnt2', rfar='j', X='lbase', Y='k0', o8='kk',
            dc='rx1', lo='ldn', hi='kk', lsg='fb', passes='fb0')
    A.wreg(3, 'mlink', 'spp2', 'cot', 'x18', 'dx18', 'rp', 'snx', 'sny', 'c12', 'rng', 'D0', 'dD', 'D1', 'D2',
           'ddD', 'xr', 'cx8', 'cy8', 'R8', 'g16', 'gx', 'gy', 'vv', 'uu', 'ua', 's20', 'vlo', 'vhi', 'lo', 'hi',
           'mptyp', 'mw3')
    A.alias(mw4='D1', mw5='D2')

    def ldc(dst, v):
        v = int(v)
        if -(1 << 19) <= v < (1 << 19):
            e('LDI', dst, imm=v)
        else:
            e('LDI', dst, imm=v >> 12)
            e('SHL', dst, dst, imm=12)
            if v & 4095:
                e('ADDI', dst, dst, imm=v & 4095)

    def mulr(dst, a, b, sh):
        e('MUL', dst, a, b, imm=sh)

    def call(sub):
        e('CALL', imm=sub)

    def jlt(a, b, l): e('JLT', a=a, b=b, imm=l)
    def jge(a, b, l): e('JGE', a=a, b=b, imm=l)

    def clamp(dst, x, lo, hi):
        e('MAX', dst, x, lo)
        e('MIN', dst, dst, hi)

    def absr(dst, x):
        e('SUB', 't', 'zero', x)
        e('MAX', dst, x, 't')

    def bit(dst, src, n):
        e('SHR', dst, src, imm=n)
        e('ANDI', dst, dst, imm=1)

    def hset(i, reg):
        """handler output i (nA nB oxA oyA oxB oyB op exA eyA exB eyB) <- reg"""
        e('STD', a='chdr', b=reg, imm=i)

    # ================= constants
    for n, v in (('zero', 0), ('one', 1), ('c4', 4), ('chdr', HDR), ('c16', 16), ('c63', 63), ('c64', 64),
                 ('c256', 256), ('c4096', 4096),
                 ('c16384', 16384), ('c32768', 32768), ('c65536', 65536)):
        e('LDI', n, imm=v)
    A.win(0)

    # ================= inputs
    for k in ('k2', 'sw', 'W', 'H', 'fld'):
        e('CTL', k, imm=k)
    e('CTL', 't', imm='set')
    ldc('recb', REC0)
    e('LDI', 'pbase', imm=PBASE)
    lset = A.lbl()
    e('JZ', a='t', imm=lset)
    ldc('recb', REC0 + REC_STRIDE)
    e('LDI', 'pbase', imm=PBASE + 16)
    A.label(lset)
    e('MAX', 'W', 'W', 'c16')
    e('MAX', 'H', 'H', 'c16')

    # video mode from the timing id: interlaced {0,1,2,8,10}; SD class when
    # (tid & 3) = 0 (NTSC 480 / PAL 576 lines: 282 / 235 Q.8 px tall); 720p
    # class {5,6,14}; else 1080
    e('CTL', 'x', imm='tid')
    ldc('t', (1 << 0) | (1 << 1) | (1 << 2) | (1 << 8) | (1 << 10))
    e('SHRV', 'ilace', 't', 'x'); e('ANDI', 'ilace', 'ilace', imm=1)
    ldc('t', (1 << 5) | (1 << 6) | (1 << 14))
    e('SHRV', 'cls', 't', 'x'); e('ANDI', 'cls', 'cls', imm=1)
    e('LDI', 'ly8', imm=256)
    lsd = A.lbl()
    e('ANDI', 't', 'x', imm=3); e('JNZ', a='t', imm=lsd)
    e('LDI', 'cls', imm=2)
    e('LDI', 'ly8', imm=282)
    e('ANDI', 't', 'x', imm=8); e('JZ', a='t', imm=lsd)
    e('LDI', 'ly8', imm=235)
    A.label(lsd)
    e('ADD', 't', 'cls', 'cls')
    e('ADDI', 't', 't', imm='cltab')
    e('LDM', 'LwK', 't')
    e('ADDI', 't', 't', imm=1)
    e('LDM', 'LbK', 't')

    # raster: W8, FH8 (frame height px Q.8), dv8 (field line pitch px Q.8), y0
    e('SHL', 'W8', 'W', imm=8)
    e('SHLV', 'HF', 'H', 'ilace')
    mulr('FH8', 'HF', 'ly8', 0)
    e('SHLV', 'dv8', 'ly8', 'ilace')
    e('AND', 'y0', 'ilace', 'fld')               # bottom field (fld=1 under interlace) starts a frame line lower
    e('MUL', 'y08', 'y0', 'ly8', imm=0)          # y0 in px Q.8

    e('ANDI', 'bank', 'sw', imm=1)
    bit('rev', 'sw', 1)
    bit('spin', 'sw', 2)

    # ================= T-bar: deadband (2 counts) + glide, dead zones -> T (Q.16)
    e('CTL', 't', imm='p12')
    e('SHL', 'tq', 't', imm=6)
    e('SUB', 'x', 'tq', 'Th')
    absr('a', 'x')
    e('LDI', 'b', imm=128)
    lk = A.lbl()
    jlt('a', 'b', lk)
    e('MOV', 'Th', 'tq')
    A.label(lk)
    e('SUB', 'x', 'Th', 'Ts')
    absr('a', 'x')
    e('SHR', 'y', 'x', imm=2)
    e('LDI', 'b', imm=4096)
    lg = A.lbl()
    jlt('a', 'b', lg)
    e('SHR', 'y', 'x', imm=1)
    A.label(lg)
    e('ADD', 'Ts', 'Ts', 'y')
    e('LDI', 'b', imm=4)
    l3 = A.lbl()
    jge('a', 'b', l3)
    e('MOV', 'Ts', 'Th')
    A.label(l3)
    e('ADDI', 'x', 'Ts', imm=-1280)
    ldc('b', 68198)
    mulr('x', 'x', 'b', 16)
    clamp('T', 'x', 'zero', 'c65536')
    # AUTO (switch 11): T sweeps there and back on its own; the T-bar sets the
    # rate, ~34 s a cycle at 0 % to ~0.5 s at 100 % (2^(5.3 p12) steps per
    # field); the triangle overshoots both ends by 4096 so each end dwells
    # bit-exact for ~6 % of the cycle
    bit('auto', 'sw', 4)
    lna, lnw, lnt = A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='auto', imm=lna)
    e('LDI', 't', imm=21730); mulr('e2', 'tq', 't', 16)
    call('pow2')
    e('LDI', 't', imm=110); mulr('t', 'pw', 't', 8)
    e('ADD', 'aph', 'aph', 't')
    e('LDI', 'x', imm=147456)
    e('SUB', 'y', 'aph', 'x'); jlt('y', 'zero', lnw); e('MOV', 'aph', 'y')
    A.label(lnw)
    e('LDI', 'x', imm=73728)
    e('MOV', 'y', 'aph'); jlt('aph', 'x', lnt)
    e('LDI', 'x', imm=147456); e('SUB', 'y', 'x', 'aph')
    A.label(lnt)
    e('ADDI', 'y', 'y', imm=-4096)
    clamp('T', 'y', 'zero', 'c65536')
    A.label(lna)
    # u = T or 1 - T under REVERSE (the sweep variable)
    e('MOV', 'u', 'T')
    lnr = A.lbl()
    e('JZ', a='rev', imm=lnr)
    e('SUB', 'u', 'c65536', 'T')
    A.label(lnr)

    # ================= pattern slot (hysteresis +-2 counts; bank change resets)
    e('LDI', 'NS', imm=len(CLASSIC))
    e('LDI', 'rb', imm='rtab')
    l4 = A.lbl()
    e('JZ', a='bank', imm=l4)
    e('LDI', 'NS', imm=len(CHEESE))
    e('LDI', 'rb', imm='rtab2')
    A.label(l4)
    e('CTL', 'x', imm='k1')
    e('ADDI', 'a', 'x', imm=-2)
    e('ADDI', 'b', 'x', imm=2)
    e('LDI', 'y', imm=1023)
    clamp('a', 'a', 'zero', 'y')
    clamp('b', 'b', 'zero', 'y')
    mulr('clo', 'a', 'NS', 10)
    mulr('chi', 'b', 'NS', 10)
    mulr('cmid', 'x', 'NS', 10)
    lnew, lkeep = A.lbl(), A.lbl()
    e('SUB', 't', 'bank', 'pbank')
    e('JNZ', a='t', imm=lnew)
    jlt('pslot', 'clo', lnew)
    jge('chi', 'pslot', lkeep)
    A.label(lnew)
    e('MOV', 'pslot', 'cmid')
    A.label(lkeep)
    e('MOV', 'pbank', 'bank')
    e('ADD', 't', 'rb', 'pslot')
    e('LDM', 'rec', 't')                          # recipe word: [7:0] handler, [9:8] bound, [10] centred, [31:16] arg
    e('ANDI', 'hnd', 'rec', imm=255)
    e('SHR', 'btyp', 'rec', imm=8); e('ANDI', 'btyp', 'btyp', imm=3)
    bit('ctr', 'rec', 10)
    e('SHR', 'harg', 'rec', imm=16)
    e('ADDI', 'm', 'k2', imm=-512)                # modifier -512..511

    # ================= EDGE -> w8 (px Q.8), G8 (key/px Q.8), bw8, DK (key units), h0, h1 (key Q.8)
    e('LDI', 'w8', imm=256)
    e('LDI', 'bw8', imm=0)
    lsoft, lbord, ledge = A.lbl(), A.lbl(), A.lbl()
    e('CTL', 'y', imm='k4')
    e('LDI', 't', imm=496)
    jlt('y', 't', lsoft)
    e('LDI', 't', imm=529)
    jge('y', 't', lbord)
    e('JMP', imm=ledge)
    A.label(lsoft)
    e('SUB', 'a', 't', 'y')
    mulr('e2', 'a', 'LwK', 16)
    call('pow2')
    e('MOV', 'w8', 'pw')
    e('JMP', imm=ledge)
    A.label(lbord)
    e('SUB', 'a', 'y', 't')
    mulr('e2', 'a', 'LbK', 16)
    call('pow2')
    e('MOV', 'bw8', 'pw')
    A.label(ledge)
    e('DIV', 'G8', 'c256', 'w8')                  # 256 * 2^16 / w8 = (256 / w) * 2^8
    mulr('DK', 'bw8', 'G8', 16)                   # border in key units
    ldc('t', round(MARGIN_PX * 256))
    mulr('h1', 't', 'G8', 8)                      # Q.8 key units of margin
    e('ADD', 'h1', 'h1', 'c32768')                # + 128 key units (half a ramp)
    e('SHL', 't', 'DK', imm=8)
    e('ADD', 'h0', 'h1', 't')
    # REVERSE inverts the key, so the border clearance (h0) belongs to the END
    # of the sweep and the plain clearance (h1) to its start: swap them
    lhs = A.lbl()
    e('JZ', a='rev', imm=lhs)
    e('MOV', 't', 'h0'); e('MOV', 'h0', 'h1'); e('MOV', 'h1', 't')
    A.label(lhs)
    e('PRT', a='DK', imm='DK')

    # ================= colours
    lbk, lbm, lbd = A.lbl(), A.lbl(), A.lbl()
    e('CTL', 'y', imm='k6')
    e('LDI', 't', imm=48)
    jlt('y', 't', lbk)
    e('LDI', 'b', imm=304)
    jlt('y', 'b', lbm)
    call('white'); e('JMP', imm=lbd)
    A.label(lbk)
    call('black'); e('JMP', imm=lbd)
    A.label(lbm)
    e('SUB', 'hh', 'y', 't'); e('SHL', 'hh', 'hh', imm=8)
    call('hue')
    A.label(lbd)
    e('PRT', a='cy', imm='BCY'); e('PRT', a='cuv', imm='BCUV')
    lrw, lrh, lrb, lrd = A.lbl(), A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='bw8', imm=lrd)
    e('CTL', 'y', imm='k5')
    e('LDI', 't', imm=64)
    jlt('y', 't', lrw)
    e('LDI', 'b', imm=704)
    jlt('y', 'b', lrh)
    e('LDI', 'b', imm=768)
    jlt('y', 'b', lrb)
    e('LDI', 'cy', imm=GOLD[0]); ldc('cuv', (GOLD[1] << 10) | GOLD[2])
    e('JMP', imm=lrd)
    A.label(lrw); call('white'); e('JMP', imm=lrd)
    A.label(lrb); call('black'); e('JMP', imm=lrd)
    A.label(lrh)
    e('SUB', 'hh', 'y', 't')
    e('LDI', 't', imm=6554); mulr('hh', 'hh', 't', 6)
    call('hue')
    A.label(lrd)
    e('PRT', a='cy', imm='RCY'); e('PRT', a='cuv', imm='RCUV')

    # ================= MIX: no fields
    lnmix = A.lbl()
    e('JNZ', a='hnd', imm=lnmix)
    e('ADDI', 't', 'T', imm=128); e('SHR', 't', 't', imm=8)
    e('PRT', a='t', imm='TK')
    e('LDI', 't', imm=4)                          # MODE 4 = mix
    e('PRT', a='t', imm='MODE')
    e('END')
    A.label(lnmix)

    # ================= anchor (px Q.8) and rotation
    lc, la = A.lbl(), A.lbl()
    e('JZ', a='ctr', imm=lc)
    e('SHR', 'ax8', 'W8', imm=1)
    e('SHR', 'ay8', 'FH8', imm=1)
    e('JMP', imm=la)
    A.label(lc)
    e('CTL', 'y', imm='k3')
    e('SHL', 'seg', 'y', imm=3); e('ADD', 'seg', 'seg', 'y')
    e('SHR', 'oi', 'seg', imm=10)
    e('ANDI', 'of', 'seg', imm=1023)
    e('ADDI', 'of', 'of', imm=-205)
    e('LDI', 't', imm=1708); mulr('of', 'of', 't', 10)
    e('LDI', 't', imm=1024); clamp('of', 'of', 'zero', 't')
    e('ADD', 'oi', 'oi', 'oi')
    pts = [(1, 1), (0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0)]
    px_ = sum(p[0] << 2 * i for i, p in enumerate(pts))
    py_ = sum(p[1] << 2 * i for i, p in enumerate(pts))
    for (packed, ext, out) in ((px_, 'W8', 'ax8'), (py_, 'FH8', 'ay8')):
        ldc('pk', packed)
        e('SHRV', 'q0', 'pk', 'oi')
        e('SHR', 'q1', 'q0', imm=2)
        e('ANDI', 'q0', 'q0', imm=3)
        e('ANDI', 'q1', 'q1', imm=3)
        e('SUB', 'x', 'q1', 'q0')
        mulr('x', 'x', 'of', 0)
        e('SHL', 'y', 'q0', imm=10)
        e('ADD', 'x', 'x', 'y')
        mulr(out, 'x', ext, 11)
    A.label(la)
    e('LDI', 'psi', imm=0)
    l8, l9 = A.lbl(), A.lbl()
    e('JZ', a='spin', imm=l8)
    e('SHR', 't', 'T', imm=2)
    e('ADD', 'x', 't', 'T')                       # 1.25 turns over the travel
    e('JZ', a='rev', imm=l9)
    e('SUB', 'x', 'zero', 'x')
    A.label(l9)
    e('MOV', 'psi', 'x')
    A.label(l8)
    e('ADD', 'sa', 'psi', 'c16384'); call('sinf'); e('MOV', 'pcs', 'sv')
    e('MOV', 'sa', 'psi'); call('sinf'); e('MOV', 'psn', 'sv')

    # ================= handler: fills SPEC_A (and B), sets nA, nB, oxA.., op, exA/eyA
    A.win(1)
    e('LDI', 't', imm=HDR)
    for i in range(12):
        e('STD', a='t', b=('zero' if (i < 7 or i == 11) else 'c65536'), imm=i)
    e('LDI', 'spp', imm=SPEC_A)
    e('ADDI', 't', 'hnd', imm='htab')
    e('LDM', 't', 't')
    e('JR', a='t')
    A.label('hret')
    A.win(2)
    e('MOV', 'fsc', 'c65536')

    # ================= bound -> f0, f1 (Q.16 scales where the field is >= h0
    # everywhere / <= -h1 everywhere); f = f0 + (f1 - f0) u; q-form shapes
    # get an extra lift of h0 over the first 6 % instead of an f0
    lbnd, lbnd2 = A.lbl(), A.lbl()
    e('LDI', 'f0', imm=0); e('LDI', 'f1', imm=0)
    ldc('gmax', 1 << 30)
    e('ADDI', 't', 'btyp', imm=-3)
    e('JZ', a='t', imm=lbnd)                      # angular: f = 1, lifts at both ends
    e('LDI', 'acc', imm=0); call('bound_acc'); e('MOV', 'fA', 'fb'); e('MOV', 'f0', 'fb0')
    e('MOV', 'f1', 'fA')
    e('LDD', 'x', 'chdr', imm=1)
    e('JZ', a='x', imm=lbnd2)
    e('LDI', 'acc', imm=1); call('bound_acc'); e('MOV', 'fB', 'fb')
    e('MIN', 'f0', 'f0', 'fb0')
    e('LDD', 'x', 'chdr', imm=6)
    e('ADDI', 't', 'x', imm=-2)
    lmx = A.lbl()
    e('JZ', a='t', imm=lmx)
    e('MIN', 'f1', 'fA', 'fB'); e('JMP', imm=lbnd2)
    A.label(lmx)
    e('MAX', 'f1', 'fA', 'fB')
    A.label(lbnd2)
    e('SUB', 'x', 'f1', 'f0')
    mulr('f', 'x', 'u', 16); e('ADD', 'f', 'f', 'f0')
    e('JMP', imm='fdone')
    A.label(lbnd)
    e('MOV', 'f', 'c65536')
    A.label('fdone')
    # extra lift: q-form shapes (header flag) and angular patterns: h0 over the
    # first 6 % of the sweep; angular patterns also -h1 over the last 6 %
    e('LDI', 'extra', imm=0)
    lx1, lx2 = A.lbl(), A.lbl()
    e('LDD', 'x', 'chdr', imm=11)
    e('ADDI', 'a', 'btyp', imm=-3)
    e('ADD', 'x', 'x', 'a'); e('ADDI', 'x', 'x', imm=3)     # x = qlift + btyp: nonzero unless (no lift, btyp 0)
    e('JZ', a='x', imm=lx2)
    e('LDI', 't', imm=3932)
    jge('u', 't', lx1)
    e('SUB', 'x', 't', 'u'); e('DIV', 'x', 'x', 't'); mulr('extra', 'x', 'h0', 16)
    A.label(lx1)
    e('JNZ', a='a', imm=lx2)                      # the end lift only for angular patterns
    e('LDI', 'x', imm=65536 - 3932)
    jlt('u', 'x', lx2)
    e('SUB', 'x', 'u', 'x'); e('DIV', 'x', 'x', 't'); mulr('x', 'x', 'h1', 16)
    e('SUB', 'extra', 'extra', 'x')
    A.label(lx2)
    e('ADDI', 'extra', 'extra', imm=128)          # the key's rounding half (key = 128 - D >> 8)

    # ================= compile both accumulators
    e('LDI', 'ridx', imm=0)                       # next free record index
    e('LDI', 'acc', imm=0); call('compile')
    e('PRT', a='lup', imm='LA_UP'); e('PRT', a='ldn', imm='LA_DN'); e('PRT', a='ayl', imm='LA_AY'); e('PRT', a='qq', imm='QA')
    lnb = A.lbl()
    e('LDD', 'x', 'chdr', imm=1)
    e('JZ', a='x', imm=lnb)
    e('LDI', 'acc', imm=1); call('compile')
    e('PRT', a='lup', imm='LB_UP'); e('PRT', a='ldn', imm='LB_DN'); e('PRT', a='ayl', imm='LB_AY'); e('PRT', a='qq', imm='QB')
    A.label(lnb)
    # MODE: [1:0] op, [2] mix, [3] reverse (the pixel path inverts the key)
    e('LDD', 'x', 'chdr', imm=6)
    e('SHL', 't', 'rev', imm=3); e('ADD', 'x', 'x', 't')
    # dead zones: T is clamped to exactly 0 / 65536 there, so force MIX with
    # TK 0 / 256 -- the ends are bit-exact A / B by construction, whatever
    # the line tables hold (bunched entries near the anchor are approximated)
    lnz, lmd = A.lbl(), A.lbl()
    e('JNZ', a='T', imm=lnz)
    e('LDI', 't', imm=0); e('PRT', a='t', imm='TK'); e('ADDI', 'x', 'x', imm=4)
    e('JMP', imm=lmd)
    A.label(lnz)
    e('ADDI', 't', 'T', imm=4); e('SUB', 't', 't', 'c65536'); jlt('t', 'zero', lmd)   # T >= 65532 (p12 >= 1004: T = 65533 there)
    e('LDI', 't', imm=256); e('PRT', a='t', imm='TK'); e('ADDI', 'x', 'x', imm=4)
    A.label(lmd)
    e('PRT', a='x', imm='MODE')
    e('END')

    # ======================================================================
    # subroutines
    # ======================================================================
    A.inbank(0)
    # pow2: e2 (Q.12 octaves >= 0) -> pw = 256 * 2^e2 (px Q.8)   (globals only)
    A.label('pow2')
    e('SHR', 'en', 'e2', imm=12)
    e('ANDI', 'ef', 'e2', imm=4095)
    e('LDI', 't', imm=1407); mulr('x', 'ef', 't', 12)
    e('ADDI', 'x', 'x', imm=2689); mulr('x', 'ef', 'x', 12)
    e('ADD', 'x', 'x', 'c4096')                   # 2^f, Q.12
    e('SHLV', 'b', 'c256', 'en')
    mulr('pw', 'b', 'x', 12)
    e('JR', a='link')

    # sinf: sv = sin(sa) Q.15, interpolated on the 64-unit grid (leaf)
    A.label('sinf')
    e('SIN', 'z0', 'sa')
    e('ADD', 'sb', 'sa', 'c64'); e('SIN', 'z1', 'sb')
    e('SUB', 'z1', 'z1', 'z0')
    e('AND', 'sb', 'sa', 'c63')
    mulr('z1', 'z1', 'sb', 6)
    e('ADD', 'sv', 'z0', 'z1')
    e('JR', a='link')

    A.label('white')
    e('LDI', 'cy', imm=WHITE[0]); ldc('cuv', (512 << 10) | 512); e('JR', a='link')
    A.label('black')
    e('LDI', 'cy', imm=BLACK[0]); ldc('cuv', (512 << 10) | 512); e('JR', a='link')

    A.label('hue')                                # hh (16-bit turn) -> cy, cuv (U = Cr)
    e('MOV', 'hlink', 'link')
    ldc('t', HUE_RED); e('ADD', 'th', 'hh', 't')
    e('MOV', 'sa', 'th'); call('sinf'); e('MOV', 'hs', 'sv')
    e('ADD', 'sa', 'th', 'c16384'); call('sinf'); e('MOV', 'hc', 'sv')
    ldc('t', HUE_YPH + 16384); e('ADD', 'sa', 'th', 't'); call('sinf'); e('MOV', 'hy', 'sv')
    e('LDI', 't', imm=166); mulr('cy', 'hy', 't', 15); e('ADDI', 'cy', 'cy', imm=393)
    e('LDI', 't', imm=242); mulr('a', 'hs', 't', 15); mulr('b', 'hc', 't', 15)
    e('ADDI', 'a', 'a', imm=512); e('ADDI', 'b', 'b', imm=512)
    e('SHL', 'cuv', 'a', imm=10); e('ADD', 'cuv', 'cuv', 'b')
    e('JR', a='hlink')

    A.inbank(1)
    # ---------------- spec emit: (rdx, rdy, type, w3, w4, w5) from registers -> DATA[spp..], spp += 6, count++
    A.label('emit')
    for (x, y) in (('rdx', 'rdy'), ('pw3', 'pw4')):
        mulr('a', x, 'pcs', 15); mulr('b', y, 'psn', 15); e('SUB', 'ra', 'a', 'b')
        mulr('a', x, 'psn', 15); mulr('b', y, 'pcs', 15); e('ADD', y, 'a', 'b')
        e('MOV', x, 'ra')
    e('STD', a='spp', b='rdx', imm=0)
    e('STD', a='spp', b='rdy', imm=1)
    e('STD', a='spp', b='ptyp', imm=2)
    e('STD', a='spp', b='pw3', imm=3)
    e('STD', a='spp', b='pw4', imm=4)
    e('STD', a='spp', b='pw5', imm=5)
    e('ADDI', 'spp', 'spp', imm=6)
    e('ADDI', 'cnt', 'cnt', imm=1)
    e('JR', a='link')

    # ---------------- handlers (each ends with JMP hret) -------------------
    # REG: regular star / polygon.  n from the modifier (3..12, noon 5 star /
    # 6 polygon), rho from a table (star) or cos(pi/n) (polygon = star whose
    # inner vertex lies on the edge).  2n sectors: per point the mirrored
    # edge then the clockwise edge.  Tip straight up (angle 3/4 turn).
    A.label('h_reg')
    e('LDI', 'cnt', imm=0)
    e('MOV', 'elink', 'link')
    lstar, lnap = A.lbl(), A.lbl()
    e('LDI', 'nn', imm=6)                         # polygon noon
    e('LDI', 'nlo', imm=3); e('LDI', 'nhi', imm=12)
    e('ADDI', 't', 'harg', imm=-2); e('JNZ', a='t', imm=lnap)
    e('LDI', 'nn', imm=7); e('LDI', 'nlo', imm=5); e('LDI', 'nhi', imm=9)   # aperture
    e('JMP', imm=lstar)
    A.label(lnap)
    e('JZ', a='harg', imm=lstar)
    e('LDI', 'nn', imm=5)
    A.label(lstar)
    call('count_mod')                             # -> nf in nlo..nhi around nn
    e('MOV', 'n', 'nf')
    e('DIV', 'S', 'one', 'n')                     # 65536 / n = sector (turn/n)
    e('SHR', 'Sh', 'S', imm=1)                    # half sector
    # rho16 (Q.16) and the edge angle alpha16 from ROM tables indexed by n
    lpoly = A.lbl()
    e('MOV', 'al', 'Sh')                          # polygon: a star whose inner vertex lies on the edge: alpha = pi/n
    e('ADDI', 't', 'harg', imm=-1); e('JNZ', a='t', imm=lpoly)
    e('ADDI', 't', 'n', imm='altab'); e('ADDI', 't', 't', imm=-3); e('LDM', 'al', 't')
    A.label(lpoly)
    # c_ref (key Q.8) = -(S_ref * cos(alpha)) * G : cos(alpha) Q.15
    e('ADD', 'sa', 'al', 'c16384'); call('sinf')
    e('SHL', 't', 'sv', imm=10)                   # S_ref=1024 px: cos*1024 Q.15 -> px Q.15
    mulr('cref', 't', 'G8', 15)                   # px * key/px Q.8 -> key Q.8
    e('SUB', 'cref', 'zero', 'cref')
    e('LDI', 'ptyp', imm=0)
    e('MOV', 'pw5', 'zero')
    e('LDI', 'k', imm=0)
    e('LDI', 'tip', imm=49152)                    # up (SPIN is applied once, in emit)
    lapt = A.lbl()
    e('ADDI', 't', 'harg', imm=-2); e('JNZ', a='t', imm=lapt)
    e('ADD', 'tip', 'tip', 'Sh')                  # aperture: a FACE up
    A.label(lapt)
    lreg = A.lbl()
    A.label(lreg)
    # sector 1: ray at tip - S/2, normal at tip - alpha (mirrored edge)
    e('SUB', 'sa', 'tip', 'Sh'); call('dirvec')
    e('SUB', 'sa', 'tip', 'al'); call('normvec')
    e('MOV', 'pw5', 'cref')
    call('emit')
    # sector 2: ray at tip, normal at tip + alpha
    e('MOV', 'sa', 'tip'); call('dirvec')
    e('ADD', 'sa', 'tip', 'al'); call('normvec')
    e('MOV', 'pw5', 'cref')
    call('emit')
    e('ADD', 'tip', 'tip', 'S')
    e('ADDI', 'k', 'k', imm=1)
    jlt('k', 'n', lreg)
    hset(0, 'cnt')
    e('JMP', imm='hret')

    # dirvec: sa (angle) -> rdx, rdy (Q.15 unit)
    A.label('dirvec')
    e('MOV', 'dlink', 'link')
    e('ADD', 'sa', 'sa', 'c16384'); call('sinf'); e('MOV', 'rdx', 'sv')
    e('SUB', 'sa', 'sa', 'c16384'); call('sinf'); e('MOV', 'rdy', 'sv')
    e('JR', a='dlink')
    # normvec: sa (angle) -> pw3, pw4 (Q.16 unit normal)
    A.label('normvec')
    e('MOV', 'dlink', 'link')
    e('ADD', 'sa', 'sa', 'c16384'); call('sinf'); e('SHL', 'pw3', 'sv', imm=1)
    e('SUB', 'sa', 'sa', 'c16384'); call('sinf'); e('SHL', 'pw4', 'sv', imm=1)
    e('JR', a='dlink')

    # count_mod: nf in nlo..nhi, noon = nn, hysteresis +-2 on k2 (state npts)
    A.label('count_mod')
    e('MOV', 'clink', 'link')
    e('ADDI', 'kx', 'k2', imm=-2); e('MAX', 'kx', 'kx', 'zero'); call('cm1'); e('MOV', 'cmlo', 'nf')
    e('ADDI', 'kx', 'k2', imm=2); e('LDI', 't', imm=1023); e('MIN', 'kx', 'kx', 't'); call('cm1'); e('MOV', 'cmhi', 'nf')
    e('MOV', 'kx', 'k2'); call('cm1')
    lk2, ln2 = A.lbl(), A.lbl()
    jlt('npts', 'cmlo', ln2)
    jge('cmhi', 'npts', lk2)
    A.label(ln2)
    e('MOV', 'npts', 'nf')
    A.label(lk2)
    e('MOV', 'nf', 'npts')
    e('JR', a='clink')
    A.label('cm1')                                # kx -> nf = nlo + (nn-nlo) kx/512 | nn + (nhi-nn)(kx-512)/511
    l5 = A.lbl()
    e('LDI', 't', imm=512)
    jge('kx', 't', l5)
    e('SUB', 'a', 'nn', 'nlo'); e('ADDI', 'a', 'a', imm=0)
    e('SHL', 'a', 'a', imm=1)                     # 2(nn-nlo); + 1 for rounding half-steps
    e('ADDI', 'a', 'a', imm=1)
    mulr('nf', 'kx', 'a', 10)                     # kx*(2(nn-nlo)+1)/1024
    e('ADD', 'nf', 'nf', 'nlo')
    e('JR', a='link')
    A.label(l5)
    e('SUB', 'a', 'nhi', 'nn'); e('SHL', 'a', 'a', imm=1); e('ADDI', 'a', 'a', imm=1)
    e('ADDI', 'b', 'kx', imm=-512)
    mulr('nf', 'b', 'a', 10)
    e('ADD', 'nf', 'nf', 'nn')
    e('MIN', 'nf', 'nf', 'nhi')
    e('JR', a='link')

    # BAR: one half-plane (abs=0) or two (abs=1) at angle harg + tilt
    A.label('h_bar')
    e('LDI', 'cnt', imm=0)
    e('ANDI', 'absf', 'harg', imm=1)
    e('ANDI', 'ang', 'harg', imm=-2)              # angle (harg holds angle | abs in bit 0)
    e('SHL', 't', 'm', imm=4)                     # tilt +-45 deg = +-8192 over +-512
    e('ADD', 'ang', 'ang', 't')
    e('SHL', 'cref', 'G8', imm=10)                # c_ref = -S_ref * G
    e('SUB', 'cref', 'zero', 'cref')
    e('LDI', 'ptyp', imm=0)
    # ray perpendicular to the normal (either side), piece normal = ang
    e('ADD', 'sa', 'ang', 'c16384'); call('dirvec')
    e('MOV', 'sa', 'ang'); call('normvec')
    e('MOV', 'pw5', 'cref')
    call('emit')
    lbar1 = A.lbl()
    e('JZ', a='absf', imm=lbar1)
    e('SUB', 'sa', 'ang', 'c16384'); call('dirvec')
    e('ADD', 'sa', 'ang', 'c32768'); call('normvec')
    e('MOV', 'pw5', 'cref')
    call('emit')
    A.label(lbar1)
    hset(0, 'cnt')
    e('JMP', imm='hret')

    # CIRC: circle (ellipse by exA/eyA from the modifier) / spotlight
    A.label('h_circ')
    e('LDI', 'cnt', imm=0)
    e('LDI', 'rdx', imm=32767); e('LDI', 'rdy', imm=0)
    e('LDI', 'ptyp', imm=1)
    e('LDI', 'pw3', imm=0); e('LDI', 'pw4', imm=0)
    e('LDI', 'pw5', imm=S_REF * 256)              # R_ref px Q.8
    call('emit')
    hset(0, 'cnt'); hset(11, 'one')
    # ellipse: ex = 2^(-0.8 m), ey = 2^(0.8 m) (Q.16) via pow2 on |m|
    lcirc = A.lbl()
    e('JNZ', a='harg', imm=lcirc)                 # spotlight: no squash (harg=1)
    absr('a', 'm')
    ldc('t', round(0.8 * 4096 * 65536 / 512)); mulr('e2', 'a', 't', 16)
    call('pow2')                                  # pw = 256 * 2^e2 (Q.8) -> Q.16 = pw << 8
    e('SHL', 'b', 'pw', imm=8)
    e('DIV', 'a', 'c256', 'pw')                   # 256 * 2^16 / pw = 2^-e2 Q.16
    lcp = A.lbl()
    jlt('m', 'zero', lcp)
    hset(7, 'a'); hset(8, 'b'); e('JMP', imm=lcirc)
    A.label(lcp)
    hset(7, 'b'); hset(8, 'a')
    A.label(lcirc)
    e('JMP', imm='hret')

    # WEDGE family: clock (1 wedge, span = u turn), pinwheel (n blades, span
    # u of a sector), double sweep (2 blades), double fan (2 symmetric wedges)
    A.label('h_wedge')
    e('LDI', 'cnt', imm=0)
    e('LDI', 'ptyp', imm=0); e('LDI', 'pw5', imm=0)
    # blades: clock 1, pinwheel n (2..8, noon 4), double sweep / fan 2
    e('LDI', 'nw', imm=1)
    lwp, lw2_, lwn = A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='harg', imm=lwn)
    e('ADDI', 't', 'harg', imm=-1); e('JNZ', a='t', imm=lw2_)
    e('LDI', 'nn', imm=4); e('LDI', 'nlo', imm=2); e('LDI', 'nhi', imm=8)
    call('count_mod'); e('MOV', 'nw', 'nf'); e('JMP', imm=lwn)
    A.label(lw2_)
    e('LDI', 'nw', imm=2)
    A.label(lwn)
    e('DIV', 'S', 'one', 'nw')                    # sector = turn / blades
    mulr('span', 'S', 'u', 16)                    # the swept part of each sector
    e('ADDI', 't', 'S', imm=-16)
    clamp('span', 'span', 'c16', 't')
    # start angle: clock up + m half turn ; pinwheel 0 ; double sweep m quarter turn ;
    # double fan up - span / 2
    e('LDI', 'wst', imm=0)
    lws, lwd = A.lbl(), A.lbl()
    e('ADDI', 't', 'harg', imm=-1); e('JZ', a='t', imm=lwd)              # pinwheel: 0
    e('ADDI', 't', 'harg', imm=-2); e('JNZ', a='t', imm=lws)
    e('SHL', 'wst', 'm', imm=5); e('JMP', imm=lwd)                        # double sweep
    A.label(lws)
    ldc('wst', 49152)
    e('ADDI', 't', 'harg', imm=-3); e('JNZ', a='t', imm=lws + 'c')
    e('SHR', 't', 'span', imm=1); e('SUB', 'wst', 'wst', 't'); e('JMP', imm=lwd)   # double fan
    A.label(lws + 'c')
    e('SHL', 't', 'm', imm=6); e('ADD', 'wst', 'wst', 't')                # clock
    A.label(lwd)
    e('LDI', 'k', imm=0)
    lwl = A.lbl()
    A.label(lwl)
    mulr('a0', 'k', 'S', 0); e('ADD', 'a0', 'a0', 'wst')
    e('ADD', 'a1', 'a0', 'span')
    e('SHR', 't', 'span', imm=1); e('ADD', 'sa', 'a0', 't'); call('dirvec')
    e('ADD', 'sa', 'a1', 'c16384'); call('normvec')      # clockwise normal of edge a1
    call('emit')
    e('ADD', 't', 'a0', 'S'); e('ADD', 'sa', 'a1', 't'); e('SHR', 'sa', 'sa', imm=1)   # gap bisector
    call('dirvec')
    e('ADD', 'sa', 'a0', 'S'); e('SUB', 'sa', 'sa', 'c16384'); call('normvec')   # -(clockwise normal of next a0)
    call('emit')
    e('ADDI', 'k', 'k', imm=1)
    jlt('k', 'nw', lwl)
    hset(0, 'cnt')
    e('JMP', imm='hret')

    # SUNBURST: n rays (6..24, noon 16) of half-width S_ref * u... as a size
    # sweep: c = -S_ref*G scaled by f; bound type linear
    A.label('h_sunb')
    e('LDI', 'cnt', imm=0)
    e('LDI', 'nn', imm=8); e('LDI', 'nlo', imm=4); e('LDI', 'nhi', imm=12)
    call('count_mod'); e('MOV', 'n', 'nf')
    e('DIV', 'S', 'one', 'n'); e('SHR', 'Sh', 'S', imm=1)
    e('LDI', 'ptyp', imm=0)
    e('SHL', 'cref', 'G8', imm=10); e('SUB', 'cref', 'zero', 'cref')
    e('LDI', 'ang', imm=0)
    e('LDI', 'k', imm=0)
    lsb = A.lbl()
    A.label(lsb)
    e('SUB', 'sa', 'ang', 'Sh'); call('dirvec')
    e('SUB', 'sa', 'ang', 'c16384'); call('normvec'); e('MOV', 'pw5', 'cref'); call('emit')
    e('MOV', 'sa', 'ang'); call('dirvec')
    e('ADD', 'sa', 'ang', 'c16384'); call('normvec'); e('MOV', 'pw5', 'cref'); call('emit')
    e('ADD', 'ang', 'ang', 'S')
    e('ADDI', 'k', 'k', imm=1)
    jlt('k', 'n', lsb)
    hset(0, 'cnt')
    e('JMP', imm='hret')

    # BOX / DIAMOND: 4-vertex polygons with aspect from the modifier
    A.label('h_box')
    e('LDI', 'cnt', imm=0)
    # rho = (W/FH) * 2^m (box) or 1.3 * 2^m (diamond), Q.16; a = min(1, rho) b = min(1, 1/rho)
    lbx = A.lbl()
    e('DIV', 'rho', 'W8', 'FH8')
    e('JZ', a='harg', imm=lbx)
    ldc('rho', round(1.3 * 65536))
    A.label(lbx)
    absr('a', 'm'); e('SHL', 'e2', 'a', imm=3); call('pow2')       # 256 * 2^|m| (Q.8), |m| in Q.12 octaves = m*8
    e('SHL', 'b', 'pw', imm=8)
    lbm2 = A.lbl()
    jge('m', 'zero', lbm2)
    e('DIV', 'b', 'c256', 'pw')
    A.label(lbm2)
    mulr('rho', 'rho', 'b', 16)
    ldc('t', 16384); ldc('b', 262144); clamp('rho', 'rho', 't', 'b')
    e('MOV', 'ha', 'c65536'); e('MOV', 'hb', 'c65536')
    lba = A.lbl()
    jge('rho', 'c65536', lba)
    e('MOV', 'ha', 'rho'); e('JMP', imm='h_box2')
    A.label(lba)
    e('DIV', 'hb', 'c65536', 'rho')               # 2^32 / rho -> too big? hb = 65536*65536/rho ... use c256 trick
    A.label('h_box2')
    # half sizes at S_ref: hx8 = 1024*256*ha>>16, hy8 likewise
    e('SHL', 'hx8', 'ha', imm=2); e('SHL', 'hy8', 'hb', imm=2)      # 1024*256/65536 = 4
    e('LDI', 'ptyp', imm=0)
    e('SUB', 'mhx', 'zero', 'hx8'); e('SUB', 'mhy', 'zero', 'hy8')
    # diamond edge normal (nxu, nyu) = (hy, hx) / L and its distance cd
    mulr('a', 'hx8', 'hx8', 16); mulr('b', 'hy8', 'hy8', 16); e('ADD', 'a', 'a', 'b')
    e('SQRT', 'L8', 'a')
    e('DIV', 'nxu', 'hy8', 'L8'); e('DIV', 'nyu', 'hx8', 'L8')
    mulr('cd', 'hx8', 'hy8', 8); e('DIV', 'cd', 'cd', 'L8'); e('SHR', 'cd', 'cd', imm=8)
    mulr('cd', 'cd', 'G8', 8); e('SUB', 'cd', 'zero', 'cd')
    # vertices k = 0..3 clockwise: box (+,+) (-,+) (-,-) (+,-); diamond (hx,0)
    # (0,hy) (-hx,0) (0,-hy).  sx = - for k in {1,2}, sy = - for k >= 2.
    e('LDI', 'k', imm=0)
    lbx = A.lbl()
    A.label(lbx)
    e('ANDI', 'a', 'k', imm=1); e('SHR', 'b', 'k', imm=1); e('SUB', 'sxn', 'a', 'b')   # != 0 <=> k in {1, 2}
    e('MOV', 'syn', 'b')                                                             # 1 <=> k >= 2
    e('MOV', 'rdx', 'hx8'); e('MOV', 'rdy', 'hy8')
    lsx, lsy = A.lbl(), A.lbl()
    e('JZ', a='sxn', imm=lsx); e('MOV', 'rdx', 'mhx'); A.label(lsx)
    e('JZ', a='syn', imm=lsy); e('MOV', 'rdy', 'mhy'); A.label(lsy)
    ldia = A.lbl()
    e('JNZ', a='harg', imm=ldia)
    # box face after vertex k: 0 bottom (0,+,hy) 1 left (-,0,hx) 2 top (0,-,hy) 3 right (+,0,hx)
    e('LDI', 'pw3', imm=0); e('LDI', 'pw4', imm=0)
    mulr('pw5', 'hy8', 'G8', 8)
    lb1, lb2, lb3, lbe = A.lbl(), A.lbl(), A.lbl(), A.lbl()
    e('JNZ', a='k', imm=lb1); e('MOV', 'pw4', 'c65536'); e('JMP', imm=lbe)
    A.label(lb1); e('ADDI', 'a', 'k', imm=-1); e('JNZ', a='a', imm=lb2); e('SUB', 'pw3', 'zero', 'c65536'); mulr('pw5', 'hx8', 'G8', 8); e('JMP', imm=lbe)
    A.label(lb2); e('ADDI', 'a', 'k', imm=-2); e('JNZ', a='a', imm=lb3); e('SUB', 'pw4', 'zero', 'c65536'); e('JMP', imm=lbe)
    A.label(lb3); e('MOV', 'pw3', 'c65536'); mulr('pw5', 'hx8', 'G8', 8)
    A.label(lbe)
    e('SUB', 'pw5', 'zero', 'pw5')
    e('JMP', imm='h_bxe')
    A.label(ldia)
    lda, ldb = A.lbl(), A.lbl()
    e('ANDI', 'a', 'k', imm=1)
    e('JNZ', a='a', imm=lda); e('LDI', 'rdy', imm=0); e('JMP', imm=ldb)
    A.label(lda); e('LDI', 'rdx', imm=0)
    A.label(ldb)
    e('MOV', 'pw3', 'nxu'); e('MOV', 'pw4', 'nyu'); e('MOV', 'pw5', 'cd')
    ldn1, ldn2 = A.lbl(), A.lbl()
    e('JZ', a='sxn', imm=ldn1); e('SUB', 'pw3', 'zero', 'pw3'); A.label(ldn1)
    e('JZ', a='syn', imm=ldn2); e('SUB', 'pw4', 'zero', 'pw4'); A.label(ldn2)
    A.label('h_bxe')
    call('emit')
    e('ADDI', 'k', 'k', imm=1)
    jlt('k', 'c4', lbx)
    hset(0, 'cnt'); e('JMP', imm='hret')

    # X DOORS: the union of two bars (|n.p| < T) with normals at +-a,
    # a = 45 + 22.5 m deg.  Eight 45-degree sectors; in each the piece is the
    # bar whose normal is most nearly perpendicular to the sector's bisector,
    # signed so n.p > 0 there: base angles 8192 x {7, 3, 1, 5, 3, 7, 5, 1}
    # with -delta for sectors 0, 1, 4, 5 and +delta for 2, 3, 6, 7
    # (delta = a - 45 deg = 8 m units).
    A.label('h_xdoor')
    e('LDI', 'cnt', imm=0); e('LDI', 'ptyp', imm=0)
    e('SHL', 'cref', 'G8', imm=10); e('SUB', 'cref', 'zero', 'cref')
    e('SHL', 'ang', 'm', imm=3)                   # delta
    ldc('wst', sum(b << 3 * i for i, b in enumerate((7, 3, 1, 5, 3, 7, 5, 1))))
    e('LDI', 'k', imm=0)
    lxd, lxo, lxn = A.lbl(), A.lbl(), A.lbl()
    A.label(lxd)
    e('SHL', 'sa', 'k', imm=13); call('dirvec')
    e('ADD', 't', 'k', 'k'); e('ADD', 't', 't', 'k'); e('SHRV', 'a', 'wst', 't'); e('ANDI', 'a', 'a', imm=7)
    e('SHL', 'sa', 'a', imm=13)
    e('ANDI', 't', 'k', imm=2); e('JNZ', a='t', imm=lxo)
    e('SUB', 'sa', 'sa', 'ang'); e('JMP', imm=lxn)
    A.label(lxo); e('ADD', 'sa', 'sa', 'ang')
    A.label(lxn)
    call('normvec'); e('MOV', 'pw5', 'cref'); call('emit')
    e('ADDI', 'k', 'k', imm=1); e('LDI', 't', imm=8); jlt('k', 't', lxd)
    hset(0, 'cnt'); e('JMP', imm='hret')

    # VEE (harg 0): v + k|u| = T, a chevron front; BARN VEE (harg 1): |v| - k|u|
    # = T.  The modifier sets the arm angle theta = atan k, 17..50 deg.  Piece
    # normals: 90 +- theta (lower pieces), 270 -+ theta (upper, barn only).
    A.label('h_vee')
    e('LDI', 'cnt', imm=0); e('LDI', 'ptyp', imm=0)
    e('ADDI', 't', 'm', imm=512); e('LDI', 'a', imm=6); mulr('al', 't', 'a', 0); e('ADDI', 'al', 'al', imm=3040)
    e('ADD', 'sa', 'al', 'c16384'); call('sinf')
    e('SHL', 't', 'sv', imm=10); mulr('cref', 't', 'G8', 15); e('SUB', 'cref', 'zero', 'cref')   # -S_ref cos(theta) G
    e('LDI', 'nw', imm=2); e('LDI', 'wst', imm=32768); e('LDI', 'a0', imm=16384)
    lvb, lvl, lvo, lvn = A.lbl(), A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='harg', imm=lvb)
    e('LDI', 'nw', imm=4); e('LDI', 'wst', imm=16384); e('LDI', 'a0', imm=0)
    A.label(lvb)
    e('LDI', 'k', imm=0)
    A.label(lvl)
    e('MOV', 'sa', 'a0'); call('dirvec')
    e('SHR', 'b', 'k', imm=1); e('SHL', 'sa', 'b', imm=15); e('ADD', 'sa', 'sa', 'c16384')
    e('ADD', 't', 'k', 'b'); e('ANDI', 't', 't', imm=1); e('JNZ', a='t', imm=lvo)
    e('ADD', 'sa', 'sa', 'al'); e('JMP', imm=lvn)
    A.label(lvo); e('SUB', 'sa', 'sa', 'al')
    A.label(lvn)
    call('normvec'); e('MOV', 'pw5', 'cref'); call('emit')
    e('ADD', 'a0', 'a0', 'wst')
    e('ADDI', 'k', 'k', imm=1); jlt('k', 'nw', lvl)
    hset(0, 'cnt'); e('JMP', imm='hret')

    # EYE: the lens between two circles of radius R = (1 + wd) S_ref centred
    # wd S_ref above / below; wd = 0.55 + 0.35 m.  Rays 0 / 180: the lower
    # half is bounded by the upper circle and vice versa.
    A.label('h_eye')
    e('LDI', 'cnt', imm=0); e('LDI', 'ptyp', imm=1)
    e('LDI', 'a', imm=45); mulr('al', 'm', 'a', 0); e('LDI', 't', imm=36045); e('ADD', 'al', 'al', 't')   # wd Q.16
    e('SHL', 'a0', 'al', imm=2)                   # d (px Q.8 at S_ref = 2^18)
    e('LDI', 'pw5', imm=S_REF * 256); e('ADD', 'pw5', 'pw5', 'a0')
    e('LDI', 'sa', imm=0); call('dirvec'); e('LDI', 'pw3', imm=0); e('SUB', 'pw4', 'zero', 'a0'); call('emit')
    ldc('sa', 32768); call('dirvec'); e('LDI', 'pw3', imm=0); e('MOV', 'pw4', 'a0'); call('emit')
    hset(0, 'cnt'); hset(11, 'one')
    e('JMP', imm='hret')

    # VPOLY: star-shaped polygon from a ROM vertex table at harg: word 0 =
    # count | kind << 8, then one word per vertex, four signed Q.4 bytes
    # (ax, bx, ay, by): vertex = (ax AX + bx BX, ay AY + by BY) / 16 with the
    # scales from the kind: 0 CROSS (AX = AY = S_ref, BX = BY = k S_ref,
    # k = 0.35 + 0.3 m), 1 TRIANGLE (BY = notch, 0.8 max(0, m) S_ref).
    # Vertices in increasing angle; the piece after vertex i is its edge to
    # vertex i + 1, oriented so the centre is inside.
    A.label('h_vpoly')
    e('LDI', 'cnt', imm=0); e('LDI', 'ptyp', imm=0)
    e('MOV', 'vtab', 'harg')
    e('LDM', 't', 'vtab'); e('ANDI', 'vcnt', 't', imm=255); e('SHR', 'a', 't', imm=8)
    e('LDI', 'AX', imm=S_REF * 256); e('MOV', 'AY', 'AX'); e('LDI', 'BX', imm=0); e('LDI', 'BY', imm=0)
    lvk1, lvks, lvpl, lvnw, lvok = A.lbl(), A.lbl(), A.lbl(), A.lbl(), A.lbl()
    e('JNZ', a='a', imm=lvk1)
    e('LDI', 'b', imm=38); mulr('BX', 'm', 'b', 0); e('LDI', 't', imm=22938); e('ADD', 'BX', 'BX', 't'); e('SHL', 'BX', 'BX', imm=2)
    e('MOV', 'BY', 'BX'); e('JMP', imm=lvks)
    A.label(lvk1)
    e('LDI', 'b', imm=410); e('MAX', 't', 'm', 'zero'); mulr('BY', 't', 'b', 0)
    A.label(lvks)
    e('LDI', 'vi', imm=0)
    A.label(lvpl)
    e('ADDI', 't', 'vtab', imm=1); e('ADD', 't', 't', 'vi'); e('LDM', 't', 't'); call('vtx')
    e('MOV', 'vx0', 'vxo'); e('MOV', 'vy0', 'vyo')
    e('ADDI', 'b', 'vi', imm=1); jlt('b', 'vcnt', lvnw); e('LDI', 'b', imm=0)
    A.label(lvnw)
    e('ADDI', 't', 'vtab', imm=1); e('ADD', 't', 't', 'b'); e('LDM', 't', 't'); call('vtx')
    e('SUB', 'vex', 'vxo', 'vx0'); e('SUB', 'vey', 'vyo', 'vy0')
    mulr('a', 'vex', 'vex', 16); mulr('b', 'vey', 'vey', 16); e('ADD', 'a', 'a', 'b'); e('SQRT', 'vL', 'a')
    # unit normal (-vey, vex) / L, Q.16 (unsigned DIV + sign; 'ra' is a window name here)
    lvs1, lvs2 = A.lbl(), A.lbl()
    e('SUB', 't', 'zero', 'vey'); e('MAX', 'a', 'vey', 't'); e('DIV', 'a', 'a', 'vL')
    jlt('vey', 'one', lvs1); e('SUB', 'a', 'zero', 'a')
    A.label(lvs1); e('MOV', 'nxv', 'a')
    e('SUB', 't', 'zero', 'vex'); e('MAX', 'b', 'vex', 't'); e('DIV', 'b', 'b', 'vL')
    jge('vex', 'zero', lvs2); e('SUB', 'b', 'zero', 'b')
    A.label(lvs2); e('MOV', 'nyv', 'b')
    mulr('a', 'nxv', 'vx0', 16); mulr('b', 'nyv', 'vy0', 16); e('ADD', 'a', 'a', 'b'); e('SUB', 'pw5', 'zero', 'a')
    jlt('pw5', 'one', lvok)
    e('SUB', 'nxv', 'zero', 'nxv'); e('SUB', 'nyv', 'zero', 'nyv'); e('SUB', 'pw5', 'zero', 'pw5')
    A.label(lvok)
    e('MOV', 'pw3', 'nxv'); e('MOV', 'pw4', 'nyv'); mulr('pw5', 'pw5', 'G8', 8)
    mulr('rdx', 'vx0', 'one', 4); mulr('rdy', 'vy0', 'one', 4)
    call('emit')
    e('ADDI', 'vi', 'vi', imm=1); jlt('vi', 'vcnt', lvpl)
    hset(0, 'cnt'); e('JMP', imm='hret')
    # vtx: packed vertex word in t -> vxo, vyo (px Q.8)
    A.label('vtx')
    e('MOV', 'vlink', 'link')
    e('SHR', 'a', 't', imm=24); mulr('vxo', 'a', 'AX', 4)
    e('SHL', 'b', 't', imm=8); e('SHR', 'b', 'b', imm=24); mulr('b', 'b', 'BX', 4); e('ADD', 'vxo', 'vxo', 'b')
    e('SHL', 'b', 't', imm=16); e('SHR', 'b', 'b', imm=24); mulr('vyo', 'b', 'AY', 4)
    e('SHL', 'b', 't', imm=24); e('SHR', 'b', 'b', imm=24); mulr('b', 'b', 'BY', 4); e('ADD', 'vyo', 'vyo', 'b')
    e('JR', a='vlink')

    A.inbank(2)
    # ---------------- bound_acc: acc -> fb (Q.16 scale at which the field is
    # <= -h1 over the whole frame), fb0 (scale at which it is >= h0).  Per
    # piece the extremes over its own sector are taken at the sector's two
    # ray exits (where the rays leave the frame) and the frame corners that
    # lie inside the sector.  Also sets gmax from the far corner.
    A.label('bound_acc')
    e('MOV', 'blink', 'link')
    call('acc_sel')
    # the four corners relative to the anchor at SCR + 8 .. 15 (x, y pairs)
    e('SUB', 'cxL', 'zero', 'aax8'); e('SUB', 'cxR', 'W8', 'aax8')
    e('SUB', 'cyT', 'zero', 'aay8'); e('SUB', 'cyB', 'FH8', 'aay8')
    e('LDI', 't', imm=SCR + 8)
    e('STD', a='t', b='cxL', imm=0); e('STD', a='t', b='cyT', imm=1)
    e('STD', a='t', b='cxR', imm=2); e('STD', a='t', b='cyT', imm=3)
    e('STD', a='t', b='cxL', imm=4); e('STD', a='t', b='cyB', imm=5)
    e('STD', a='t', b='cxR', imm=6); e('STD', a='t', b='cyB', imm=7)
    e('LDI', 'rfar', imm=0)
    e('LDI', 'i', imm=0)
    lrf = A.lbl()
    A.label(lrf)
    e('SHL', 't', 'i', imm=1); e('ADDI', 't', 't', imm=SCR + 8)
    e('LDD', 'X', 't', imm=0); e('LDD', 'Y', 't', imm=1)
    mulr('a', 'X', 'X', 16); mulr('b', 'Y', 'Y', 16); e('ADD', 'a', 'a', 'b'); e('SQRT', 'a', 'a')
    e('MAX', 'rfar', 'rfar', 'a')
    e('ADDI', 'i', 'i', imm=1); jlt('i', 'c4', lrf)
    # gmax = 2^20 * 2^16 / rfar^2 (px^2): g rfar^2 <= 2^20 key units
    mulr('a', 'rfar', 'rfar', 16); e('MAX', 'a', 'a', 'c256')
    ldc('b', 1 << 20); e('DIV', 'a', 'b', 'a'); e('MIN', 'gmax', 'gmax', 'a')
    # ray exits / entries: per ray, per axis (x then y): t_out = min of the
    # facing-edge distance / |d|, t_in = max of the edge-behind distance / |d|
    e('LDI', 'bk', imm=0)
    lex = A.lbl()
    A.label(lex)
    e('MOV', 'ks', 'bk'); call('spec_at')
    ldc('tt', 0x7FFFFFF); e('LDI', 'tin', imm=0)
    e('LDI', 'i', imm=0)
    lax, laxn = A.lbl(), A.lbl()
    A.label(lax)
    e('MOV', 'dc', 'rdx'); e('MOV', 'lo', 'cxL'); e('MOV', 'hi', 'cxR')
    e('JZ', a='i', imm=laxn)
    e('MOV', 'dc', 'rdy'); e('MOV', 'lo', 'cyT'); e('MOV', 'hi', 'cyB')
    A.label(laxn)
    lz, lneg, ld = A.lbl(), A.lbl(), A.lbl()
    absr('x', 'dc'); e('LDI', 't', imm=64)
    jlt('x', 't', ld)                             # (nearly) parallel to this edge pair: skip
    jlt('dc', 'zero', lneg)
    e('MOV', 'a', 'hi'); e('MOV', 'b', 'dc'); e('MOV', 'x', 'lo'); e('JMP', imm=lz)
    A.label(lneg)
    e('SUB', 'a', 'zero', 'lo'); e('SUB', 'b', 'zero', 'dc'); e('SUB', 'x', 'zero', 'hi')
    A.label(lz)
    e('MAX', 'a', 'a', 'zero'); e('DIV', 'a', 'a', 'b'); e('MIN', 'tt', 'tt', 'a')
    e('MAX', 'x', 'x', 'zero'); e('DIV', 'x', 'x', 'b'); e('MAX', 'tin', 'tin', 'x')
    A.label(ld)
    e('ADDI', 'i', 'i', imm=1); e('LDI', 't', imm=2); jlt('i', 't', lax)
    e('SHL', 't', 'bk', imm=2); e('ADDI', 't', 't', imm=EXT)
    mulr('a', 'rdx', 'tt', 16); mulr('b', 'rdy', 'tt', 16)
    e('STD', a='t', b='a', imm=0); e('STD', a='t', b='b', imm=1)
    mulr('a', 'rdx', 'tin', 16); mulr('b', 'rdy', 'tin', 16)
    e('STD', a='t', b='a', imm=2); e('STD', a='t', b='b', imm=3)
    e('ADDI', 'bk', 'bk', imm=1)
    jlt('bk', 'n', lex)
    # corner sectors: SCR[c] = index of the sector holding corner c
    e('LDI', 'i', imm=0)
    lco = A.lbl()
    A.label(lco)
    e('SHL', 't', 'i', imm=1); e('ADDI', 't', 't', imm=SCR + 8)
    e('LDD', 'X', 't', imm=0); e('LDD', 'Y', 't', imm=1)
    e('LDI', 'csec', imm=0)
    e('LDI', 'bk', imm=0)
    lcs, lcn, lcd = A.lbl(), A.lbl(), A.lbl()
    A.label(lcs)
    e('MOV', 'ks', 'bk'); call('spec_at')
    mulr('a', 'rdx', 'Y', 16); mulr('b', 'rdy', 'X', 16); e('SUB', 'cr1', 'a', 'b')
    e('ADDI', 'ks', 'bk', imm=1)
    lw = A.lbl(); jlt('ks', 'n', lw); e('LDI', 'ks', imm=0); A.label(lw)
    call('spec_at')
    mulr('a', 'rdx', 'Y', 16); mulr('b', 'rdy', 'X', 16); e('SUB', 'cr2', 'a', 'b')
    jlt('cr1', 'zero', lcn)
    jge('cr2', 'zero', lcn)
    e('MOV', 'csec', 'bk'); e('JMP', imm=lcd)
    A.label(lcn)
    e('ADDI', 'bk', 'bk', imm=1)
    jlt('bk', 'n', lcs)
    A.label(lcd)
    e('ADDI', 't', 'i', imm=SCR); e('STD', a='t', b='csec', imm=0)
    e('ADDI', 'i', 'i', imm=1); jlt('i', 'c4', lco)
    # per piece: candidates = exits / entries of rays k, k+1 and the corners in sector k
    e('LDI', 'fb', imm=0); ldc('fb0', 1 << 30)
    e('LDI', 'bk', imm=0)
    lbl_ = A.lbl()
    A.label(lbl_)
    e('SHL', 't', 'bk', imm=1); e('ADD', 't', 't', 'bk'); e('SHL', 't', 't', imm=1)
    e('ADD', 'sp2', 'sp', 't')
    e('LDD', 'bptyp', 'sp2', imm=2); e('LDD', 'bw3', 'sp2', imm=3); e('LDD', 'bw4', 'sp2', imm=4); e('LDD', 'bw5', 'sp2', imm=5)
    ldc('pmax', -(1 << 30)); ldc('pmin', 1 << 30); e('LDI', 'rsec', imm=0)
    e('MOV', 'ks', 'bk')
    e('LDI', 'i', imm=0)
    lcan = A.lbl()
    A.label(lcan)                                 # i = 0..3: exit k, entry k, exit k+1, entry k+1
    e('SHL', 't', 'ks', imm=2); e('ADDI', 't', 't', imm=EXT)
    e('ANDI', 'a', 'i', imm=1); e('ADD', 'a', 'a', 'a'); e('ADD', 't', 't', 'a')
    e('LDD', 'X', 't', imm=0); e('LDD', 'Y', 't', imm=1)
    call('bcand')
    e('ADDI', 'i', 'i', imm=1)
    lk1 = A.lbl()
    e('ADDI', 'a', 'i', imm=-2); e('JNZ', a='a', imm=lk1)
    e('ADDI', 'ks', 'bk', imm=1)
    lw2 = A.lbl(); jlt('ks', 'n', lw2); e('LDI', 'ks', imm=0); A.label(lw2)
    A.label(lk1)
    jlt('i', 'c4', lcan)
    e('LDI', 'i', imm=0)
    lcc = A.lbl()
    A.label(lcc)
    lsk = A.lbl()
    e('ADDI', 't', 'i', imm=SCR); e('LDD', 't', 't', imm=0)
    e('SUB', 't', 't', 'bk')
    e('JNZ', a='t', imm=lsk)
    e('SHL', 't', 'i', imm=1); e('ADDI', 't', 't', imm=SCR + 8)
    e('LDD', 'X', 't', imm=0); e('LDD', 'Y', 't', imm=1)
    call('bcand')
    A.label(lsk)
    e('ADDI', 'i', 'i', imm=1); jlt('i', 'c4', lcc)
    lbc, lbn = A.lbl(), A.lbl()
    e('JNZ', a='bptyp', imm=lbc)
    e('SUB', 'b', 'zero', 'bw5'); e('MAX', 'b', 'b', 'one')
    mulr('a', 'pmax', 'G8', 8); e('ADD', 'a', 'a', 'h1')
    call('sdiv_ab'); e('MAX', 'fb', 'fb', 'ra')
    mulr('a', 'pmin', 'G8', 8); e('SUB', 'a', 'a', 'h0')
    e('SUB', 'b', 'zero', 'bw5'); e('MAX', 'b', 'b', 'one')
    call('sdiv_ab'); e('MIN', 'fb0', 'fb0', 'ra')
    e('JMP', imm=lbn)
    A.label(lbc)
    # circle piece: the centre offset and the candidates in the ELLIPSE metric
    mulr('a', 'bw3', 'bw3', 16); mulr('a', 'a', 'exx', 16); mulr('b', 'bw4', 'bw4', 16); mulr('b', 'b', 'eyy', 16)
    e('ADD', 'a', 'a', 'b'); e('SQRT', 'o8', 'a')
    e('SUB', 'b', 'bw5', 'o8'); e('MAX', 'b', 'b', 'c256')
    e('DIV', 'a', 'h1', 'G8'); e('SHR', 'a', 'a', imm=8)
    e('ADD', 'a', 'a', 'rsec')
    e('DIV', 'fk', 'a', 'b')
    ldc('t', round(1.1 * 65536)); mulr('fk', 'fk', 't', 16)
    e('MAX', 'fb', 'fb', 'fk')
    e('LDI', 'fb0', imm=0)
    A.label(lbn)
    e('ADDI', 'bk', 'bk', imm=1)
    jlt('bk', 'n', lbl_)
    e('JR', a='blink')

    # bcand: candidate point (X, Y) px Q.8 rel. anchor -> pmax/pmin of n.p (linear) or rsec (circle)
    A.label('bcand')
    lcc2, lcd2 = A.lbl(), A.lbl()
    e('JNZ', a='bptyp', imm=lcc2)
    mulr('a', 'bw3', 'X', 16); mulr('b', 'bw4', 'Y', 16); e('ADD', 'a', 'a', 'b')
    e('MAX', 'pmax', 'pmax', 'a'); e('MIN', 'pmin', 'pmin', 'a')
    e('JR', a='link')
    A.label(lcc2)
    mulr('a', 'X', 'X', 16); mulr('a', 'a', 'exx', 16); mulr('b', 'Y', 'Y', 16); mulr('b', 'b', 'eyy', 16)
    e('ADD', 'a', 'a', 'b'); e('SQRT', 'a', 'a')
    e('MAX', 'rsec', 'rsec', 'a')
    e('JR', a='link')

    A.label('acc_sel')                            # acc -> sp, n, ox, oy, exx, eyy, aax8 / aay8
    e('LDI', 'sp', imm=SPEC_A)
    lsa = A.lbl()
    e('JZ', a='acc', imm=lsa); e('LDI', 'sp', imm=SPEC_B); A.label(lsa)
    e('ADDI', 't', 'acc', imm=HDR); e('LDD', 'n', 't', imm=0)
    e('ADD', 't', 't', 'acc'); e('LDD', 'ox', 't', imm=2); e('LDD', 'oy', 't', imm=3)
    e('LDD', 'exx', 't', imm=7); e('LDD', 'eyy', 't', imm=8)
    mulr('a', 'ox', 'pcs', 15); mulr('b', 'oy', 'psn', 15); e('SUB', 'a', 'a', 'b')
    mulr('x', 'a', 'fsc', 16); e('ADD', 'aax8', 'ax8', 'x')
    mulr('a', 'ox', 'psn', 15); mulr('b', 'oy', 'pcs', 15); e('ADD', 'a', 'a', 'b')
    mulr('x', 'a', 'fsc', 16); e('ADD', 'aay8', 'ay8', 'x')
    e('JR', a='link')

    # ---------------- compile: acc -> records + descriptors (lup, ldn, ayl, qq)
    A.label('compile')
    e('MOV', 'blink', 'link')
    e('MOV', 'fsc', 'f')
    call('acc_sel')
    e('LDI', 'qq', imm=0)
    e('LDI', 'passes', imm=0)
    # ay_line = max(0, ceil((aay8/ly8 - y0) / pitch))  (field lines)
    e('MAX', 'a', 'aay8', 'zero')
    e('DIV', 'a', 'a', 'ly8'); e('SHR', 'a', 'a', imm=8)                # frame lines Q.8
    e('SHL', 'b', 'y0', imm=8); e('SUB', 'a', 'a', 'b')
    e('MAX', 'a', 'a', 'zero')
    e('SHRV', 'a', 'a', 'ilace')
    e('ADDI', 'a', 'a', imm=255); e('SHR', 'ayl', 'a', imm=8)
    # v0 for the up list (field line 0) and the down list (field line ayl)
    e('SUB', 'v0up', 'y08', 'aay8')
    mulr('a', 'ayl', 'dv8', 0); e('ADD', 'v0dn', 'v0up', 'a')
    # per-sector trig: ray direction -> cot (Q.16, clamped +-64) and dy sign; cached in DATA spec word? no space: recompute per use
    # --- the two lists.  lsg = +1: the DOWN list (rays with dy > 0, emitted in
    # decreasing angle = reverse cyclic order, crossing ray k enters sector
    # k-1, START piece = the last collected ray's sector); lsg = -1: the UP
    # list (dy < 0, forward order, ray k enters sector k, START piece = the
    # sector before the first collected ray).
    e('LDI', 'lsg', imm=1)
    llist = A.lbl()
    A.label(llist)
    e('LDI', 'cnt2', imm=0)
    e('LDI', 'k0', imm=0)
    lf, lf2 = A.lbl(), A.lbl()
    A.label(lf)                                   # k0 = an index whose ray is NOT in this list
    e('MOV', 'ks', 'k0'); call('spec_at')
    mulr('t', 'rdy', 'lsg', 0)
    jlt('t', 'one', lf2)
    e('ADDI', 'k0', 'k0', imm=1)
    jlt('k0', 'n', lf)
    e('LDI', 'k0', imm=0)
    A.label(lf2)
    e('LDI', 'i', imm=0)                          # collect, in cyclic order from k0
    lc1, lc2 = A.lbl(), A.lbl()
    A.label(lc1)
    e('ADD', 'bk', 'k0', 'i')
    lkw = A.lbl(); jlt('bk', 'n', lkw); e('SUB', 'bk', 'bk', 'n'); A.label(lkw)
    e('MOV', 'ks', 'bk'); call('spec_at')
    mulr('t', 'rdy', 'lsg', 0)
    jlt('t', 'one', lc2)
    e('ADDI', 't', 'cnt2', imm=SCR + 16); e('STD', a='t', b='bk', imm=0)
    e('ADDI', 'cnt2', 'cnt2', imm=1)
    A.label(lc2)
    e('ADDI', 'i', 'i', imm=1)
    jlt('i', 'n', lc1)
    e('MOV', 'lbase', 'ridx')
    e('MOV', 'v0', 'v0dn')
    lv0 = A.lbl()
    jge('lsg', 'zero', lv0); e('MOV', 'v0', 'v0up'); A.label(lv0)
    # START piece
    lns, lhs = A.lbl(), A.lbl()
    e('JZ', a='cnt2', imm=lns)
    e('ADDI', 'j', 'cnt2', imm=-1)                # down: the last collected ; up: the first
    lj0 = A.lbl(); jge('lsg', 'zero', lj0); e('LDI', 'j', imm=0); A.label(lj0)
    call('idx_get')
    e('MOV', 'kp', 'kk')
    lsp = A.lbl()
    jge('lsg', 'zero', lsp)
    e('ADDI', 'kp', 'kk', imm=-1)
    lw_ = A.lbl(); jge('kp', 'zero', lw_); e('ADD', 'kp', 'kp', 'n'); A.label(lw_)
    A.label(lsp)
    e('JMP', imm=lhs)
    A.label(lns)                                  # no crossing: the sector containing straight down / up
    e('LDI', 'kp', imm=0)
    e('LDI', 'i', imm=0)
    lq1, lq2 = A.lbl(), A.lbl()
    A.label(lq1)
    e('MOV', 'ks', 'i'); call('spec_at'); mulr('rx1', 'rdx', 'lsg', 0)
    e('ADDI', 'ks', 'i', imm=1)
    lkw2 = A.lbl(); jlt('ks', 'n', lkw2); e('LDI', 'ks', imm=0); A.label(lkw2)
    call('spec_at'); mulr('t', 'rdx', 'lsg', 0)
    jlt('rx1', 'zero', lq2); jge('t', 'zero', lq2)
    e('MOV', 'kp', 'i'); e('JMP', imm=lhs)
    A.label(lq2)
    e('ADDI', 'i', 'i', imm=1)
    jlt('i', 'n', lq1)
    A.label(lhs)
    e('LDI', 'isstart', imm=1)
    call('mkrec')
    # rays: down from the last collected to the first, up forward
    e('LDI', 'isstart', imm=0)
    e('ADDI', 'j', 'cnt2', imm=-1)
    lj1 = A.lbl(); jge('lsg', 'zero', lj1); e('LDI', 'j', imm=0); A.label(lj1)
    lr1, lr2 = A.lbl(), A.lbl()
    A.label(lr1)
    jlt('j', 'zero', lr2); jge('j', 'cnt2', lr2)
    call('idx_get')
    e('MOV', 'kray', 'kk')
    e('MOV', 'kp', 'kk')
    lrp = A.lbl()
    jlt('lsg', 'zero', lrp)
    e('ADDI', 'kp', 'kk', imm=-1)
    lw2 = A.lbl(); jge('kp', 'zero', lw2); e('ADD', 'kp', 'kp', 'n'); A.label(lw2)
    A.label(lrp)
    call('mkrec')
    e('SUB', 'j', 'j', 'lsg')
    e('JMP', imm=lr1)
    A.label(lr2)
    e('SUB', 'a', 'ridx', 'lbase')
    # descriptor = record ADDRESS << 8 | count (the builder adds nothing)
    e('SHL', 'b', 'lbase', imm=3); e('SUB', 'b', 'b', 'lbase'); e('ADD', 'b', 'b', 'recb')
    e('SHL', 'b', 'b', imm=8); e('ADD', 'b', 'b', 'a')
    lld, lld2 = A.lbl(), A.lbl()
    jlt('lsg', 'zero', lld); e('MOV', 'ldn', 'b'); e('JMP', imm=lld2)
    A.label(lld); e('MOV', 'lup', 'b')
    A.label(lld2)
    e('LDI', 'lsg', imm=-1)
    e('ADDI', 'passes', 'passes', imm=1)
    e('ADDI', 't', 'passes', imm=-2)
    e('JNZ', a='t', imm=llist)
    e('JR', a='blink')

    # spec_at: k0 -> sp2 = sp + 6 k0; rdx, rdy rotated by psi (Q.15 scale irrelevant)
    A.label('spec_at')                            # ks -> sp2 = sp + 6 ks; rdx, rdy (globals only)
    e('SHL', 't', 'ks', imm=1); e('ADD', 't', 't', 'ks'); e('SHL', 't', 't', imm=1)
    e('ADD', 'sp2', 'sp', 't')
    e('LDD', 'rdx', 'sp2', imm=0); e('LDD', 'rdy', 'sp2', imm=1)
    e('JR', a='link')

    A.label('idx_get')                            # j -> kk from the index scratch
    e('ADDI', 't', 'j', imm=SCR + 16); e('LDD', 'kk', 't', imm=0)
    e('JR', a='link')

    # ---------------- mkrec: one record from piece kp (spec), ray kray (if not
    # isstart), v0 -> DATA[recb + 7 ridx]; ridx++.  The record carries the
    # piece's value at x = 0 (D0, dD0, ddD0), its step there (s0), and the
    # crossing (x, dx) with its active line range (sw_scan).
    A.label('mkrec')
    A.win(3)
    e('MOV', 'mlink', 'link')
    e('SHL', 't', 'kp', imm=1); e('ADD', 't', 't', 'kp'); e('SHL', 't', 't', imm=1)
    e('ADD', 'spp2', 'sp', 't')
    e('LDD', 'mptyp', 'spp2', imm=2); e('LDD', 'mw3', 'spp2', imm=3); e('LDD', 'mw4', 'spp2', imm=4); e('LDD', 'mw5', 'spp2', imm=5)
    e('SHL', 't', 'ridx', imm=3); e('SUB', 't', 't', 'ridx')
    e('ADD', 'rp', 't', 'recb')
    # ----- crossing: cot (Q.16, |cot| <= 512), x, dx, range
    lms = A.lbl()
    e('LDI', 'x18', imm=0); e('LDI', 'dx18', imm=0)
    e('LDI', 't', imm='rngtab'); e('LDM', 'rng', 't')
    e('JNZ', a='isstart', imm=lms)
    e('MOV', 'ks', 'kray'); call('spec_at')      # rdx, rdy
    absr('da', 'rdx'); absr('db', 'rdy')
    e('SHL', 't', 'db', imm=9)
    lcl = A.lbl()
    jlt('da', 't', lcl)
    e('MOV', 'da', 't')
    A.label(lcl)
    e('MAX', 'db', 'db', 'one')
    e('DIV', 'cot', 'da', 'db')
    e('SHR', 'a', 'rdx', imm=31); e('SHR', 'b', 'rdy', imm=31); e('SUB', 'a', 'a', 'b')
    lcs = A.lbl()
    e('JZ', a='a', imm=lcs); e('SUB', 'cot', 'zero', 'cot')
    A.label(lcs)
    e('SHL', 'x18', 'aax8', imm=8)
    mulr('a', 'v0', 'cot', 8); e('ADD', 'x18', 'x18', 'a')          # modular: only valid inside the range
    mulr('dx18', 'dv8', 'cot', 8)
    # range: |cot| < 1 px/line -> always active; else the lines where |x| <= 24000 px
    ldc('rng', 2047 << 11)
    absr('a', 'cot')
    lrf = A.lbl()
    jlt('a', 'c65536', lrf)
    for (lim, dst) in ((-24000 * 256, 'vlo'), (24000 * 256, 'vhi')):
        ldc('a', lim); e('SUB', 'a', 'a', 'aax8')
        e('MOV', 'b', 'cot'); call('sdiv_ab')                        # v8 = a * 2^16 / cot16 (signed)
        e('MOV', dst, 'ra')
    e('MIN', 'a', 'vlo', 'vhi'); e('MAX', 'vhi', 'vlo', 'vhi'); e('MOV', 'vlo', 'a')
    e('SUB', 'a', 'vlo', 'v0'); e('ADD', 'a', 'a', 'dv8'); e('ADDI', 'a', 'a', imm=-1)
    e('MOV', 'b', 'dv8'); call('sdiv_ab'); e('SHR', 'lo', 'ra', imm=16)       # ceil((vlo - v0) / dv)
    e('SUB', 'a', 'vhi', 'v0'); e('MOV', 'b', 'dv8'); call('sdiv_ab'); e('SHR', 'hi', 'ra', imm=16)
    e('LDI', 't', imm=2047)
    clamp('lo', 'lo', 'zero', 't'); clamp('hi', 'hi', 'zero', 't')
    e('SHL', 'rng', 'hi', imm=11); e('ADD', 'rng', 'rng', 'lo')
    # sides: before = sign(v0 * cot) < 0 -> left ; after = dx < 0 -> left
    e('SHR', 'a', 'v0', imm=31); e('SHR', 'b', 'cot', imm=31); e('SUB', 'a', 'a', 'b')
    lsb_ = A.lbl()
    e('JZ', a='a', imm=lsb_); ldc('t', 1 << 22); e('ADD', 'rng', 'rng', 't')
    A.label(lsb_)
    lsa_ = A.lbl()
    jge('dx18', 'zero', lsa_); ldc('t', 1 << 23); e('ADD', 'rng', 'rng', 't')
    A.label(lsa_)
    A.label(lrf)
    A.label(lms)
    e('STD', a='rp', b='x18', imm=0); e('STD', a='rp', b='dx18', imm=1); e('STD', a='rp', b='rng', imm=6)
    # ----- the piece at x = 0 (xr = -aax8)
    e('SUB', 'xr', 'zero', 'aax8')
    e('JNZ', a='mptyp', imm='mk_circ')
    mulr('snx', 'mw3', 'G8', 8); mulr('sny', 'mw4', 'G8', 8)           # Q.16 key/px
    mulr('c12', 'mw5', 'f', 16); e('ADD', 'c12', 'c12', 'extra')
    mulr('a', 'snx', 'xr', 16); mulr('b', 'sny', 'v0', 16); e('ADD', 'D0', 'a', 'b'); e('ADD', 'D0', 'D0', 'c12')
    mulr('dD', 'sny', 'dv8', 8)
    e('STD', a='rp', b='D0', imm=2); e('STD', a='rp', b='dD', imm=3); e('STD', a='rp', b='zero', imm=4)
    e('STD', a='rp', b='snx', imm=5)
    e('JMP', imm='mk_done')
    A.label('mk_circ')
    mulr('cx8', 'mw3', 'f', 16); mulr('cy8', 'mw4', 'f', 16); mulr('R8', 'mw5', 'f', 16)
    e('LDI', 't', imm=1024); e('MAX', 'R8', 'R8', 't')
    e('SHL', 'a', 'R8', imm=1); e('ADD', 'a', 'a', 'w8')
    e('DIV', 'g16', 'G8', 'a')
    e('MIN', 'g16', 'g16', 'gmax')
    mulr('gx', 'g16', 'exx', 16); mulr('gy', 'g16', 'eyy', 16)
    e('SHL', 'qq', 'gx', imm=1)
    mulr('a', 'R8', 'R8', 16); mulr('c12', 'a', 'g16', 8); e('SUB', 'c12', 'zero', 'c12')
    e('ADD', 'c12', 'c12', 'extra')
    e('SUB', 'uu', 'xr', 'cx8')
    mulr('a', 'uu', 'uu', 8); mulr('ua', 'a', 'gx', 16)             # gx uu^2 (constant along v), px^2 kept Q.8
    e('SUB', 'vv', 'v0', 'cy8')
    mulr('b', 'vv', 'vv', 8); mulr('b', 'b', 'gy', 16)
    e('ADD', 'D0', 'ua', 'b'); e('ADD', 'D0', 'D0', 'c12')
    # exact line differences (finite differences of truncated values drift
    # when the line step is not a whole pixel): dD = gy (2 v d + d^2) Q.16,
    # ddD = 2 gy d^2 Q.20
    e('SHL', 'a', 'vv', imm=1); e('ADD', 'a', 'a', 'dv8'); mulr('a', 'a', 'dv8', 0)
    mulr('dD', 'a', 'gy', 16)
    mulr('a', 'dv8', 'dv8', 0); e('SHL', 'a', 'a', imm=1); mulr('ddD', 'a', 'gy', 12)
    e('SHL', 'a', 'uu', imm=1); e('ADD', 'a', 'a', 'c256'); mulr('s20', 'gx', 'a', 8)
    e('STD', a='rp', b='D0', imm=2); e('STD', a='rp', b='dD', imm=3); e('STD', a='rp', b='ddD', imm=4)
    e('STD', a='rp', b='s20', imm=5)
    A.label('mk_done')
    e('ADDI', 'ridx', 'ridx', imm=1)
    e('MOV', 'ra', 'mlink')
    A.win(2)
    e('JR', a='ra')

    # sdiv_ab: fk = a * 2^16 / b, signed a and b (globals only; any bank)
    A.label('sdiv_ab')
    e('SHR', 'da', 'a', imm=31); e('SHR', 'db', 'b', imm=31); e('SUB', 'da', 'da', 'db')     # sign differs?
    absr('a', 'a'); absr('b', 'b'); e('MAX', 'b', 'b', 'one')
    e('DIV', 'ra', 'a', 'b')
    lsd2 = A.lbl()
    e('JZ', a='da', imm=lsd2); e('SUB', 'ra', 'zero', 'ra')
    A.label(lsd2)
    e('JR', a='link')

    # ======================================================================
    # data tables
    # ======================================================================
    A.label('cltab')
    for c in (0, 1, 2):
        A.data(round(math.log2(0.3 * CLASS_FH[c]) * 4096 * 65536 / 496))
        A.data(round(math.log2(0.08 * CLASS_FH[c]) * 4096 * 65536 / 495))
    def vword(ax, bx, ay, by):
        return (ax & 255) << 24 | (bx & 255) << 16 | (ay & 255) << 8 | (by & 255)
    A.label('vt_cross')                           # plus sign, arms (AX, AY), half-thickness (BX, BY); from 20 deg
    VT_CROSS = [(16, 0, 0, 16), (0, 16, 0, 16), (0, 16, 16, 0), (0, -16, 16, 0), (0, -16, 0, 16), (-16, 0, 0, 16),
                (-16, 0, 0, -16), (0, -16, 0, -16), (0, -16, -16, 0), (0, 16, -16, 0), (0, 16, 0, -16), (16, 0, 0, -16)]
    A.data(len(VT_CROSS) | 0 << 8)
    for v in VT_CROSS:
        A.data(vword(*v))
    A.label('vt_tri')                             # isoceles, apex up (0, -0.875), base 0.625 with the notch on BY
    VT_TRI = [(16, 0, 10, 0), (0, 0, 10, -16), (-16, 0, 10, 0), (0, 0, -14, 0)]
    A.data(len(VT_TRI) | 1 << 8)
    for v in VT_TRI:
        A.data(vword(*v))
    HND = {'MIX': 0, 'BAR': 1, 'CIRC': 2, 'REG': 3, 'WEDGE': 4, 'SUNB': 5, 'BOX': 6, 'DIA': 7,
           'XDOOR': 8, 'VEE': 9, 'EYE': 10, 'VPOLY': 11}
    HLBL = {1: 'h_bar', 2: 'h_circ', 3: 'h_reg', 4: 'h_wedge', 5: 'h_sunb', 6: 'h_box', 7: 'h_box',
            8: 'h_xdoor', 9: 'h_vee', 10: 'h_eye', 11: 'h_vpoly'}
    def rword(p):
        r = RECIPES[p]
        hnd, btyp, ctr = HND[r[0]], r[1], r[2]
        arg = 0
        if r[0] == 'BAR':
            arg = (r[3] >> 1) << 1 | r[4]
        elif r[0] == 'REG':
            arg = {'poly': 0, 'star': 1, 'aper': 2}[r[3]]
        elif r[0] == 'VEE':
            arg = r[3]
        elif r[0] == 'VPOLY':
            arg = A.labels[r[3]]
        elif r[0] == 'WEDGE':
            arg = {'clock': 0, 'pin': 1, 'dsweep': 2, 'dfan': 3}[r[3]]
        elif r[0] == 'CIRC':
            arg = 1 if p == 'SPOTLIGHT' else 0
        elif r[0] == 'DIA':
            arg = 1
        return hnd | btyp << 8 | ctr << 10 | arg << 16
    A.label('rtab')
    for p in CLASSIC:
        A.data(rword(p))
    A.label('rtab2')
    for p in CHEESE:
        A.data(rword(p))
    A.label('htab')
    A.data(0)
    for h in range(1, 12):
        A.data(A.labels[HLBL[h]])
    A.label('rngtab')
    A.data((2047 << 11) | (1 << 22) | (1 << 23))  # START: always active, left on both sides
    A.data(2047 << 11)                            # a ray that is always active
    A.label('altab')                              # star edge-normal angle alpha (16-bit) by n - 3 (n = 3..12)
    for n in range(3, 13):
        rho = STAR_RHO[n]
        al = math.atan2(1 - rho * math.cos(math.pi / n), rho * math.sin(math.pi / n))
        A.data(round(al / (2 * math.pi) * 65536))
    return A


if __name__ == '__main__':
    A = build()
    w = A.words()
    print(len(w), 'words,', len(A.regs), 'registers')
