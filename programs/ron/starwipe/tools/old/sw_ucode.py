#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE microcode: the per-frame sequencer (runs from the frame anchor).

  1. inputs, video mode (timing id), measured raster
  2. T-bar: deadband + glide (16-bit), 2 % dead zones -> T (Q16)
  3. pattern slot (hysteresis) -> its RECIPE word (data in the ROM)
  4. anchor (ORIGIN path or frame centre), rotation, SPIN, raster DDA
     increments and both fields' seeds
  5. EDGE -> filter gain G (scales the raster DDA into key units), border
     width (DK), clearances h0/h1
  6. colours (B source, border)
  7. coverage: walk the frame perimeter through the pixel pipeline and map
     T to the threshold so both ends of travel are exactly A / exactly B
Units: u-units (1/16 h-pixel); 16-bit angles here (65536 per turn), 2^20 per
turn in the pixel path.  Persistent registers: Ts, Th, pslot, pbank, npts
(the register file powers up at zero).

Recipe word: [3:0] field select  [5:4] bound (0 mix, 1 linear, 2 gauge,
3 box)  [6] centre-anchored (edge-launched)  [8:7] modifier (0 none, 1 tilt
+-45 deg, 2 star points, 3 aspect)  [23:16] base rotation (signed, 1/256 turn)
"""
import math
from sw_uasm import Asm

PSEL = {'MIX': 0, 'LIN': 1, 'BOX': 2, 'ABSU': 3, 'ABSV': 4, 'CIRCLE': 8, 'STAR': 9}

#            name      psel  bound ctr mod rot(1/256 turn)
RECIPES = {
    'MIX':    (0,  0, 1, 0, 0),
    'BAR':    (1,  1, 1, 1, 0),
    'BOX':    (2,  3, 0, 3, 0),
    'CIRCLE': (8,  2, 0, 0, 0),
    'STAR':   (9,  2, 0, 2, -64),
}
CLASSIC = ['MIX', 'BAR', 'BOX', 'CIRCLE']
CHEESE = ['STAR']

STAR_RHO = {3: 0.36, 4: 0.40, 5: 0.382, 6: 0.50, 7: 0.47, 8: 0.50, 9: 0.50,
            10: 0.52, 11: 0.53, 12: 0.55}


def star_alpha(n):
    rho = STAR_RHO[n]
    a = math.atan2(1 - rho * math.cos(math.pi / n), rho * math.sin(math.pi / n))
    return round(a / (2 * math.pi) * 2 ** 20)


def ycc(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return (64 + round(876 * y), 512 + round(896 * (r - y) / 1.402),
            512 + round(896 * (b - y) / 1.772))           # Y, Cr (-> U), Cb (-> V)


WHITE = (765, 512, 512)
BLACK = (64, 512, 512)
GOLD = ycc(0.80, 0.62, 0.20)

# hue wheel: chroma phase (from +Cb toward +Cr), amplitude 242 (in gamut for
# every hue), luma = 393 + 166 cos(theta + 153.6 deg) (yellow bright, blue
# dark); 0 = red, rising through yellow, green, cyan, blue, magenta
HUE_RED = round(108.7 / 360 * 65536)
HUE_YPH = round(153.6 / 360 * 65536)

# soft max 30 % and border max 8 % of frame height, log2 px per knob count
CLASS_FH = {0: 1080, 1: 720, 2: 534}
ILACE_MASK = (1 << 0) | (1 << 8) | (1 << 1) | (1 << 2) | (1 << 10)
SD_MASK = 0x1111
M720_MASK = (1 << 5) | (1 << 6) | (1 << 14)

MARGIN = 96          # u-units: 4 px walk-sampling slack + 2 px


def recipe_word(p):
    psel, bound, ctr, mod, rot = RECIPES[p]
    return psel | bound << 4 | ctr << 6 | mod << 7 | (rot & 255) << 16


def build():
    A = Asm()
    e = A.emit
    # pooled temporaries (lifetimes never overlap across sections)
    A.alias(tq='u0', clo='u0', chi='u1', cmid='u2', NS='u3', rb='u4',
            kx='u0', nlo='u1', nhi='u2', nf='u3',
            seg='u0', oi='u1', of='u2', m3='u3', pk='u4', q0='u5', q1='u6',
            dx0='u0', dy0='u1',
            e2='u0', pw='u1', ef='u2', en='u3',
            asp='u4', rho='u5', ka='u6', kb='u7', umax='u8', vmax='u9', hx='u10', sm='u11',
            th='v0', hs='v1', hc='v2', hy='v3', hlink='v4',
            sb='w0', z0='w1', z1='w2')

    def ldc(dst, v):
        if -(1 << 19) <= v < (1 << 19):
            e('LDI', dst, imm=v)
        else:
            assert dst != 'kt'
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
        e('AND', dst, dst, 'one')

    # ================= constants
    for n, v in (('zero', 0), ('one', 1), ('c8', 8), ('c16', 16), ('c63', 63), ('c64', 64),
                 ('c512', 512), ('c1023', 1023), ('c4095', 4095), ('c4096', 4096),
                 ('c16384', 16384), ('c65536', 65536)):
        e('LDI', n, imm=v)

    # ================= inputs
    for k in ('k1', 'k2', 'k3', 'k4', 'k5', 'k6', 'sw', 'p12', 'tid', 'W', 'H', 'fld'):
        e('CTL', k, imm=k)
    e('MAX', 'W', 'W', 'c16')                 # power-up floor
    e('MAX', 'H', 'H', 'c16')

    # video mode from the timing id
    for (nm, mask) in (('ilace', ILACE_MASK), ('sd', SD_MASK), ('m720', M720_MASK)):
        e('LDI', 't', imm=mask)
        e('SHRV', nm, 't', 'tid')
        e('AND', nm, nm, 'one')
    e('MOV', 'cls', 'm720')
    l1 = A.lbl()
    e('LDI', 'sy8', imm=4096)
    e('JZ', a='sd', imm=l1)
    e('LDI', 'cls', imm=2)
    e('LDI', 'sy8', imm=4506)                 # NTSC lines: 11/10 h-px
    bit('t', 'tid', 3)
    e('JZ', a='t', imm=l1)
    e('LDI', 'sy8', imm=3755)                 # PAL lines: 11/12 h-px
    A.label(l1)
    e('ADD', 't', 'cls', 'cls')
    e('ADDI', 't', 't', imm='cltab')
    e('LDM', 'LwK', 't')
    e('ADD', 't', 't', 'one')
    e('LDM', 'LbK', 't')

    # raster: frame height / width in u-units
    e('SHLV', 'HF', 'H', 'ilace')
    mulr('FHu', 'HF', 'sy8', 8)
    e('SHL', 'Wu', 'W', imm=4)
    e('SHLV', 'syf8', 'sy8', 'ilace')

    # switches
    e('AND', 'bank', 'sw', 'one')
    bit('rev', 'sw', 1)
    bit('spin', 'sw', 2)

    # ================= T-bar: deadband (2 counts) + glide, then dead zones
    e('SHL', 'tq', 'p12', imm=6)
    e('SUB', 'x', 'tq', 'Th')
    absr('a', 'x')
    e('LDI', 'b', imm=128)
    lk = A.lbl()
    jlt('a', 'b', lk)
    e('MOV', 'Th', 'tq')                      # the hold follows real moves only
    A.label(lk)
    e('SUB', 'x', 'Th', 'Ts')
    absr('a', 'x')
    e('SHR', 'y', 'x', imm=2)
    e('LDI', 'b', imm=4096)
    lg = A.lbl()
    jlt('a', 'b', lg)
    e('SHR', 'y', 'x', imm=1)                 # big moves: half the gap per frame
    A.label(lg)
    e('ADD', 'Ts', 'Ts', 'y')
    e('LDI', 'b', imm=4)
    l3 = A.lbl()
    jge('a', 'b', l3)
    e('MOV', 'Ts', 'Th')                      # land exactly
    A.label(l3)
    e('ADDI', 'x', 'Ts', imm=-1280)
    ldc('b', 68198)
    mulr('x', 'x', 'b', 16)
    clamp('T', 'x', 'zero', 'c65536')

    # ================= pattern slot (hysteresis +-8 counts; bank change resets)
    e('LDI', 'NS', imm=len(CLASSIC))
    e('LDI', 'rb', imm='rtab')
    l4 = A.lbl()
    e('JZ', a='bank', imm=l4)
    e('LDI', 'NS', imm=len(CHEESE))
    e('LDI', 'rb', imm='rtab2')
    A.label(l4)
    e('SUB', 'a', 'k1', 'c8')
    e('ADD', 'b', 'k1', 'c8')
    clamp('a', 'a', 'zero', 'c1023')
    clamp('b', 'b', 'zero', 'c1023')
    mulr('clo', 'a', 'NS', 10)
    mulr('chi', 'b', 'NS', 10)
    mulr('cmid', 'k1', 'NS', 10)
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
    e('LDM', 'rec', 't')
    e('ANDI', 'psel', 'rec', imm=15)
    e('SHR', 'typ', 'rec', imm=4); e('ANDI', 'typ', 'typ', imm=3)
    bit('ctr', 'rec', 6)
    e('SHR', 'mod', 'rec', imm=7); e('ANDI', 'mod', 'mod', imm=3)
    e('SHR', 'psi', 'rec', imm=16)            # signed 8-bit (1/256 turn) -> 16-bit angle
    e('SHL', 'psi', 'psi', imm=16)
    e('SHR', 'psi', 'psi', imm=8)

    # ================= modifiers
    lm2, lm3, lmd = A.lbl(), A.lbl(), A.lbl()
    e('ADDI', 't', 'mod', imm=-2)
    e('JZ', a='t', imm=lm2)
    e('ADDI', 't', 'mod', imm=-1)
    e('JNZ', a='t', imm=lmd)
    e('SUB', 't', 'k2', 'c512')               # tilt +-45 deg
    e('SHL', 't', 't', imm=4)
    e('ADD', 'psi', 'psi', 't')
    e('JMP', imm=lmd)
    A.label(lm2)
    # star points 3..12 (noon = 5), hysteresis +-8
    e('SUB', 'kx', 'k2', 'c8'); e('MAX', 'kx', 'kx', 'zero')
    call('npf'); e('MOV', 'nlo', 'nf')
    e('ADD', 'kx', 'k2', 'c8'); e('MIN', 'kx', 'kx', 'c1023')
    call('npf'); e('MOV', 'nhi', 'nf')
    e('MOV', 'kx', 'k2')
    call('npf')
    lk2, ln2 = A.lbl(), A.lbl()
    jlt('npts', 'nlo', ln2)
    jge('nhi', 'npts', lk2)
    A.label(ln2)
    e('MOV', 'npts', 'nf')
    A.label(lk2)
    e('DIV', 'x', 'c16', 'npts')              # 2^20 / n
    e('PRT', a='npts', imm='SN')
    e('PRT', a='x', imm='SS')
    e('ADDI', 't', 'npts', imm='atab')
    e('LDM', 'x', 't')
    e('PRT', a='x', imm='SAL')
    A.label(lmd)

    # ================= EDGE: soft (CCW of noon) / hard / border (CW), widths
    # in u-units; the filter gain G = 256 / w (Q12) scales the whole raster
    # DDA so the pixel path sees KEY UNITS (one edge width = 256)
    e('MOV', 'w', 'c16')
    e('LDI', 'bw', imm=0)
    lsoft, lbord, ledge = A.lbl(), A.lbl(), A.lbl()
    e('LDI', 't', imm=496)
    jlt('k4', 't', lsoft)
    e('LDI', 't', imm=529)
    jge('k4', 't', lbord)
    e('JMP', imm=ledge)
    A.label(lsoft)
    e('SUB', 'a', 't', 'k4')
    mulr('e2', 'a', 'LwK', 16)
    call('pow2')
    e('MOV', 'w', 'pw')
    e('JMP', imm=ledge)
    A.label(lbord)
    e('SUB', 'a', 'k4', 't')
    mulr('e2', 'a', 'LbK', 16)
    call('pow2')
    e('MOV', 'bw', 'pw')
    A.label(ledge)
    e('DIV', 'G', 'c16', 'w')                 # 2^20 / w = 256 / w, Q12
    mulr('DK', 'bw', 'G', 12)                 # border width in key units
    e('LDI', 't', imm=MARGIN)
    mulr('h1', 't', 'G', 12)
    e('ADDI', 'h1', 'h1', imm=128)            # half an edge + margin
    e('ADD', 'h0', 'h1', 'DK')
    e('PRT', a='DK', imm='DK')

    # ================= anchor
    lc, la = A.lbl(), A.lbl()
    e('JZ', a='ctr', imm=lc)
    e('SHR', 'ax', 'Wu', imm=1)
    e('SHR', 'ay', 'FHu', imm=1)
    e('JMP', imm=la)
    A.label(lc)
    # ORIGIN: centre dwell -> TL -> clockwise; 9 segments, 20 % dwell each end.
    # Path points (halves of the frame) packed 2 bits each, x and y.
    e('SHL', 'seg', 'k3', imm=3)
    e('ADD', 'seg', 'seg', 'k3')
    e('SHR', 'oi', 'seg', imm=10)
    e('AND', 'of', 'seg', 'c1023')
    e('ADDI', 'of', 'of', imm=-205)
    e('LDI', 't', imm=1708)
    mulr('of', 'of', 't', 10)
    e('LDI', 't', imm=1024)
    clamp('of', 'of', 'zero', 't')
    e('ADD', 'oi', 'oi', 'oi')
    pts = [(1, 1), (0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0)]
    px_ = sum(p[0] << 2 * i for i, p in enumerate(pts))
    py_ = sum(p[1] << 2 * i for i, p in enumerate(pts))
    e('LDI', 'm3', imm=3)
    for (packed, ext, out) in ((px_, 'Wu', 'ax'), (py_, 'FHu', 'ay')):
        ldc('pk', packed)
        e('SHRV', 'q0', 'pk', 'oi')
        e('SHR', 'q1', 'q0', imm=2)
        e('AND', 'q0', 'q0', 'm3')
        e('AND', 'q1', 'q1', 'm3')
        e('SUB', 'x', 'q1', 'q0')
        mulr('x', 'x', 'of', 0)
        e('SHL', 'y', 'q0', imm=10)
        e('ADD', 'x', 'x', 'y')
        mulr(out, 'x', ext, 11)
    A.label(la)

    # ================= rotation: SPIN = 1.25 turns over the travel (REVERSE flips)
    l8, l9 = A.lbl(), A.lbl()
    e('JZ', a='spin', imm=l8)
    e('SHR', 't', 'T', imm=2)
    e('ADD', 'x', 't', 'T')
    e('JZ', a='rev', imm=l9)
    e('SUB', 'x', 'zero', 'x')
    A.label(l9)
    e('ADD', 'psi', 'psi', 'x')
    A.label(l8)
    e('ADD', 'sa', 'psi', 'c16384')
    call('sinf'); e('MOV', 'cs', 'sv')
    e('MOV', 'sa', 'psi')
    call('sinf'); e('MOV', 'sn', 'sv')

    # raster DDA in u-units << 14, then x G >> 16: key units << 10
    e('SHL', 'dux', 'cs', imm=3)
    e('SHL', 't', 'sn', imm=3)
    e('SUB', 'dvx', 'zero', 't')
    mulr('duy', 'sn', 'syf8', 9)
    mulr('dvy', 'cs', 'syf8', 9)
    for r_ in ('dux', 'dvx', 'duy', 'dvy'):
        mulr(r_, r_, 'G', 16)
    e('PRT', a='duy', imm='DUY')
    e('PRT', a='dvy', imm='DVY')
    # first-line seeds of both fields (top = frame line 0, bottom = line 1).
    # The walk always runs from the TOP seed so both fields get the same
    # threshold (no 30 Hz edge twitter); the upcoming field's own seed is
    # loaded into the line accumulators after the walks.  Interlaced, the
    # next field follows the last one's parity (field_n '1' = top).
    e('SUB', 'dx0', 'c8', 'ax')
    for l0, (ru, rv) in ((0, ('suT', 'svT')), (1, ('suB', 'svB'))):
        e('LDI', 'x', imm=2 * l0 + 1)
        mulr('x', 'x', 'sy8', 9)
        e('SUB', 'dy0', 'x', 'ay')
        mulr('a', 'dx0', 'cs', 9)
        mulr('b', 'dy0', 'sn', 9)
        e('ADD', 'a', 'a', 'b')
        mulr(ru, 'a', 'G', 16)
        mulr('a', 'dy0', 'cs', 9)
        mulr('b', 'dx0', 'sn', 9)
        e('SUB', 'a', 'a', 'b')
        mulr(rv, 'a', 'G', 16)
    e('PRT', a='suT', imm='SU')
    e('PRT', a='svT', imm='SV')

    # ================= colours
    # B source: BLACK | MATTE hue wheel | WHITE (the rest of the list: later)
    lbk, lbm, lbd = A.lbl(), A.lbl(), A.lbl()
    e('LDI', 't', imm=48)
    jlt('k6', 't', lbk)
    e('LDI', 'b', imm=304)
    jlt('k6', 'b', lbm)
    call('white')
    e('JMP', imm=lbd)
    A.label(lbk)
    call('black')
    e('JMP', imm=lbd)
    A.label(lbm)
    e('SUB', 'hh', 'k6', 't')
    e('SHL', 'hh', 'hh', imm=8)               # 256 counts = one turn
    call('hue')
    A.label(lbd)
    e('PRT', a='cy', imm='BCY')
    e('PRT', a='cuv', imm='BCUV')
    # border: WHITE | hue wheel | BLACK | specials (gold for now); = B if no border
    lrw, lrh, lrb, lrd = A.lbl(), A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='bw', imm=lrd)
    e('LDI', 't', imm=64)
    jlt('k5', 't', lrw)
    e('LDI', 'b', imm=704)
    jlt('k5', 'b', lrh)
    e('LDI', 'b', imm=768)
    jlt('k5', 'b', lrb)
    e('LDI', 'cy', imm=GOLD[0]); ldc('cuv', (GOLD[1] << 10) | GOLD[2])
    e('JMP', imm=lrd)
    A.label(lrw)
    call('white')
    e('JMP', imm=lrd)
    A.label(lrb)
    call('black')
    e('JMP', imm=lrd)
    A.label(lrh)
    e('SUB', 'hh', 'k5', 't')
    e('LDI', 't', imm=6554)                   # 640 counts = one turn: x 65536/640 >> 6
    mulr('hh', 'hh', 't', 6)
    call('hue')
    A.label(lrd)
    e('PRT', a='cy', imm='RCY')
    e('PRT', a='cuv', imm='RCUV')

    # ================= coverage + threshold
    e('SHL', 't', 'rev', imm=4)
    e('ADD', 'flags', 'psel', 't')
    for s_, d_ in (('dux', 'dux8'), ('dvx', 'dvx8'), ('duy', 'duy8'), ('dvy', 'dvy8')):
        e('SHL', d_, s_, imm=3)
    e('ADDI', 'nx', 'W', imm=6); e('SHR', 'nx', 'nx', imm=3)
    e('ADDI', 'ny', 'H', imm=6); e('SHR', 'ny', 'ny', imm=3)
    lmix, lbox, lfin = A.lbl(), A.lbl(), A.lbl()
    e('JZ', a='typ', imm=lmix)
    e('ADDI', 't', 'typ', imm=-3)
    e('JZ', a='t', imm=lbox)
    # linear / gauge: walk the pattern itself
    e('MOV', 'wm', 'psel')
    call('walk')
    e('ADDI', 't', 'typ', imm=-2)
    lng = A.lbl()
    e('JNZ', a='t', imm=lng)
    e('LDI', 'dmin', imm=0)                   # gauges: the anchor lies in the frame
    A.label(lng)
    lrv, lmp = A.lbl(), A.lbl()
    e('JNZ', a='rev', imm=lrv)
    e('SUB', 's0', 'dmin', 'h0')
    e('ADD', 's1', 'dmax', 'h1')
    e('JMP', imm=lmp)
    A.label(lrv)
    e('ADD', 's0', 'dmax', 'h0')
    e('SUB', 's1', 'dmin', 'h1')
    A.label(lmp)
    call('tmap')
    e('PRT', a='sv', imm='T')
    e('JMP', imm=lfin)

    A.label(lmix)
    e('ADDI', 't', 'T', imm=128)
    e('SHR', 't', 't', imm=8)
    e('PRT', a='t', imm='TK')
    e('JMP', imm=lfin)

    A.label(lbox)
    # aspect rho (Q12) = (W/FH) * 2^((K2 - 512) / 256), clamped to 1/4..4
    e('DIV', 'asp', 'Wu', 'FHu')
    e('SHR', 'asp', 'asp', imm=4)
    e('SUB', 'e2', 'k2', 'c512')
    e('SHL', 'e2', 'e2', imm=4)
    call('pow2s')
    mulr('rho', 'asp', 'pw', 12)
    e('LDI', 'a', imm=1024)
    e('LDI', 'b', imm=16384)
    clamp('rho', 'rho', 'a', 'b')
    lw1, lw2 = A.lbl(), A.lbl()
    jlt('rho', 'c4096', lw1)
    e('MOV', 'ka', 'c4096')
    e('DIV', 'kb', 'c4096', 'rho')
    e('SHR', 'kb', 'kb', imm=4)
    e('JMP', imm=lw2)
    A.label(lw1)
    e('MOV', 'ka', 'rho')
    e('MOV', 'kb', 'c4096')
    A.label(lw2)
    e('LDI', 'wm', imm=PSEL['ABSU'])
    call('walk')
    e('MOV', 'umax', 'dmax')
    e('LDI', 'wm', imm=PSEL['ABSV'])
    call('walk')
    e('MOV', 'vmax', 'dmax')
    lbr, lbm2 = A.lbl(), A.lbl()
    e('JNZ', a='rev', imm=lbr)
    e('SUB', 's0', 'zero', 'h0')
    e('MOV', 'hx', 'h1'); call('smax'); e('MOV', 's1', 'sm')
    e('JMP', imm=lbm2)
    A.label(lbr)
    e('MOV', 'hx', 'h0'); call('smax'); e('MOV', 's0', 'sm')
    e('SUB', 's1', 'zero', 'h1')
    A.label(lbm2)
    call('tmap')
    mulr('t', 'sv', 'ka', 12)
    e('PRT', a='t', imm='BA')
    mulr('t', 'sv', 'kb', 12)
    e('PRT', a='t', imm='BB')
    e('PRT', a='zero', imm='T')

    A.label(lfin)
    lseed = A.lbl()
    e('AND', 'x', 'ilace', 'fld')
    e('JZ', a='x', imm=lseed)
    e('PRT', a='suB', imm='SU')
    e('PRT', a='svB', imm='SV')
    A.label(lseed)
    e('PRT', a='flags', imm='FLAGS')
    e('PRT', a='dux', imm='DUX')
    e('PRT', a='dvx', imm='DVX')
    e('END')

    # ================= subroutines (return through 'link'; none nests)
    A.label('tmap')                           # sv = s0 + (s1 - s0) * T >> 16
    e('SUB', 'x', 's1', 's0')
    mulr('x', 'x', 'T', 16)
    e('ADD', 'sv', 's0', 'x')
    e('JR', a='link')

    A.label('smax')                           # sm = max((umax+hx)/ka, (vmax+hx)/kb) Q12
    # dividends pre-shifted by 4 so key-unit extents stay inside the
    # divider's contract ((A << 16) / B < 2^24) at 1/4 aspect
    e('ADD', 'a', 'umax', 'hx')
    e('SHR', 'a', 'a', imm=4)
    e('DIV', 'a', 'a', 'ka')
    e('ADD', 'b', 'vmax', 'hx')
    e('SHR', 'b', 'b', imm=4)
    e('DIV', 'b', 'b', 'kb')
    e('MAX', 'sm', 'a', 'b')
    e('JR', a='link')

    A.label('npf')                            # kx -> nf: star points of a knob position
    l5 = A.lbl()
    e('LDI', 't', imm=614)
    jge('kx', 't', l5)
    e('LDI', 'b', imm=5124)
    mulr('nf', 'kx', 'b', 20)
    e('ADDI', 'nf', 'nf', imm=3)
    e('JR', a='link')
    A.label(l5)
    e('SUB', 'a', 'kx', 't')
    e('LDI', 'b', imm=1119)
    mulr('nf', 'a', 'b', 16)
    e('ADDI', 'nf', 'nf', imm=6)
    e('LDI', 't', imm=12)
    e('MIN', 'nf', 'nf', 't')
    e('JR', a='link')

    # pow2: e2 (Q12 octaves, >= 0) -> en, ef; pw = (16 << en) * 2^ef >> 12
    A.label('pow2')
    e('SHR', 'en', 'e2', imm=12)
    e('AND', 'ef', 'e2', 'c4095')
    e('LDI', 't', imm=1407)
    mulr('x', 'ef', 't', 12)
    e('LDI', 'b', imm=2689)
    e('ADD', 'x', 'x', 'b')
    mulr('x', 'ef', 'x', 12)
    e('ADD', 'x', 'x', 'c4096')               # 2^f, Q12
    e('SHLV', 'b', 'c16', 'en')
    mulr('pw', 'b', 'x', 12)
    e('JR', a='link')

    # pow2s: pw = 2^(e2 / 4096), Q12, e2 in -2..+2 octaves
    A.label('pow2s')
    e('SHR', 'en', 'e2', imm=12)
    e('AND', 'ef', 'e2', 'c4095')
    e('LDI', 't', imm=1407)
    mulr('x', 'ef', 't', 12)
    e('LDI', 'b', imm=2689)
    e('ADD', 'x', 'x', 'b')
    mulr('x', 'ef', 'x', 12)
    e('ADD', 'pw', 'x', 'c4096')
    lneg, ldn = A.lbl(), A.lbl()
    jlt('en', 'zero', lneg)
    e('SHLV', 'pw', 'pw', 'en')
    e('JMP', imm=ldn)
    A.label(lneg)
    e('SUB', 't', 'zero', 'en')
    e('SHRV', 'pw', 'pw', 't')
    A.label(ldn)
    e('JR', a='link')

    # sinf: sv = sin(sa) Q15, interpolated between the ROM's 256 quarter points
    # (leaf: callers that are subroutines save their link first)
    A.label('sinf')
    e('SIN', 'z0', 'sa')
    e('ADD', 'sb', 'sa', 'c64')
    e('SIN', 'z1', 'sb')
    e('SUB', 'z1', 'z1', 'z0')
    e('AND', 'sb', 'sa', 'c63')
    mulr('z1', 'z1', 'sb', 6)
    e('ADD', 'sv', 'z0', 'z1')
    e('JR', a='link')

    A.label('white')
    e('LDI', 'cy', imm=WHITE[0])
    ldc('cuv', (512 << 10) | 512)
    e('JR', a='link')
    A.label('black')
    e('LDI', 'cy', imm=BLACK[0])
    ldc('cuv', (512 << 10) | 512)
    e('JR', a='link')

    # hue: hh (16-bit turn, 0 = red) -> cy, cuv (U = Cr, V = Cb)
    A.label('hue')
    e('MOV', 'hlink', 'link')
    ldc('t', HUE_RED)
    e('ADD', 'sa', 'hh', 't')
    e('MOV', 'th', 'sa')
    def sinc(dst):
        e('CALL', imm='sinf')
        e('MOV', dst, 'sv')
    sinc('hs')                                # sin -> Cr
    e('ADD', 'sa', 'th', 'c16384')
    sinc('hc')                                # cos -> Cb
    ldc('t', HUE_YPH + 16384)
    e('ADD', 'sa', 'th', 't')
    sinc('hy')                                # cos(theta + 153.6 deg) -> luma
    e('LDI', 't', imm=166)
    mulr('cy', 'hy', 't', 15)
    e('ADDI', 'cy', 'cy', imm=393)
    e('LDI', 't', imm=242)
    mulr('a', 'hs', 't', 15)
    mulr('b', 'hc', 't', 15)
    e('ADD', 'a', 'a', 'c512')
    e('ADD', 'b', 'b', 'c512')
    e('SHL', 'cuv', 'a', imm=10)
    e('ADD', 'cuv', 'cuv', 'b')
    e('JR', a='hlink')

    # walk: the top field's perimeter through the pixel pipeline, metric wm.
    # A PRT to WALK holds the engine until that edge has stepped (so the next
    # DUX/DVX write is safe); reading DMIN/DMAX holds until the pipe is flushed.
    A.label('walk')
    e('PRT', a='wm', imm='FLAGS')
    for i, (du, dv, cnt) in enumerate((('dux8', 'dvx8', 'nx'), ('duy8', 'dvy8', 'ny'),
                                       ('dux8', 'dvx8', 'nx'), ('duy8', 'dvy8', 'ny'))):
        if i >= 2:
            e('SUB', 'a', 'zero', du); e('PRT', a='a', imm='DUX')
            e('SUB', 'a', 'zero', dv); e('PRT', a='a', imm='DVX')
        else:
            e('PRT', a=du, imm='DUX'); e('PRT', a=dv, imm='DVX')
        if i == 0:
            ldc('t', 0x800000)
            e('ADD', 't', 't', cnt)
            e('PRT', a='t', imm='WALK')
        else:
            e('PRT', a=cnt, imm='WALK')
    e('CTL', 'dmin', imm='DMIN')
    e('CTL', 'dmax', imm='DMAX')
    e('JR', a='link')

    # ================= data
    A.label('rtab')
    for p in CLASSIC:
        A.data(recipe_word(p))
    A.label('rtab2')
    for p in CHEESE:
        A.data(recipe_word(p))
    A.label('cltab')
    for c in (0, 1, 2):
        A.data(round(math.log2(0.3 * CLASS_FH[c]) * 4096 * 65536 / 496))
        A.data(round(math.log2(0.08 * CLASS_FH[c]) * 4096 * 65536 / 495))
    A.label('atab')                           # indexed by n (0..12)
    for n in range(13):
        A.data(star_alpha(n) if n >= 3 else 0)
    return A


if __name__ == '__main__':
    A = build()
    w = A.words()
    print(len(w), 'words,', len(A.regs), 'registers')
