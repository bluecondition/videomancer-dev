#!/usr/bin/env python3
"""ORACLE v2 microcode: every per-field and per-line computation.

    python3 gen/oracle_ucode.py          # assemble, self-test, write VHDL
    python3 gen/oracle_ucode.py --dump   # also print per-line outputs

Writes ../oracle_ucode_pkg.vhd (C_UCODE + C_RFINIT).  The emulator test
runs a few fields at 1080p / 1080i / NTSC and checks ranges, cycle budgets
and the eye profile.

Program layout
  0      WAIT field tick
         knobs (deadband + glide, one subroutine), raster, knob laws, motion
         phases, eye size/width/aura normaliser, gaze, bar chirp, band
         frequency + perspective numerator, tear bands, line accumulators
         LINE(r0); XFER; LINE(r0 + step)
  LOOP:  WAIT line trigger (a pending field tick restarts at 0); XFER; LINE
LINE computes the line-register block for row rc into rf[1 .. NXF], which
the next XFER copies into the pixel path, then steps every accumulator.
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from oasm import Asm, PORTS, CTLR, XFER_FIELDS, NXF, emit_vhdl, emit_rf_init  # noqa
from oemu import Emu, s32  # noqa

A = Asm()
REG = {}
INIT = {}
CONST = {}


def r(name):
    if name not in REG:
        REG[name] = len(REG)
        assert REG[name] < 256, 'register file full'
    return REG[name]


def k(v):
    """register holding constant v"""
    if v not in CONST:
        CONST[v] = r(f'#{v}')
        INIT[CONST[v]] = v
    return CONST[v]


# register 0 = 0, then the line block (contiguous, XFER order)
r('zero')
A.zero = 0
for f in XFER_FIELDS:
    r('c_' + f)


def op(o, d=None, a=None, b=None, imm=0):
    A.emit(o, dst=(r(d) if isinstance(d, str) else (d or 0)),
           a=(r(a) if isinstance(a, str) else (a or 0)),
           b=(r(b) if isinstance(b, str) else (b or 0)), imm=imm)


def L(name):
    A.label(name)


def call(sub, ret):
    op('LDI', 'ra', imm=ret)
    op('JMP', imm=sub)
    L(ret)


def out(port, a):
    op('OUT', a=a, imm=PORTS[port])


# initial knob state (512) so the first fields glide from centre
for n in ('1', '2', '3', '5', '6', '12'):
    INIT[r('H' + n)] = 512
    INIT[r('E' + n)] = 512 * 16

# ======================================================================
# FIELD
# ======================================================================
L('FIELD')
op('WAIT', imm=0)

# ---- knobs: K1 K2 K3 K5 K6 P12 (registers_in 0 1 2 4 5 7) ----
for n, c in (('1', 'k0'), ('2', 'k1'), ('3', 'k2'), ('5', 'k4'), ('6', 'k5'), ('12', 'k7')):
    op('CTL', 't0', imm=CTLR[c])
    op('MOV', 'kh', 'H' + n)
    op('MOV', 'ke', 'E' + n)
    call('SMOOTH', 'ks_ret' + n)
    op('MOV', 'H' + n, 'kh')
    op('MOV', 'E' + n, 'ke')
    op('SHR', 'K' + n, 'E' + n, imm=4)
op('CTL', 'K4', imm=CTLR['k3'])
op('CTL', 'sw', imm=CTLR['sw'])

# ---- raster ----
op('CTL', 'fh', imm=CTLR['fh'])
op('CTL', 'mh', imm=CTLR['mh'])
op('CTL', 'ilace', imm=CTLR['ilace'])
op('CTL', 'botf', imm=CTLR['botf'])
op('MOV', 'r0', 'botf')
op('JNZ', a='ilace', imm='IL1')
op('LDI', 'r0', imm=0)
L('IL1')
op('ADD', 'step', 'ilace', k(1))
# mode: SD / 720p / 1080
op('LDI', 'sd', imm=1)
op('LDI', 'W', imm=720)
op('MOV', 'kw', k(23302))
op('MOV', 'kwsq', k(132564))
op('JLT', a='mh', b=k(1000), imm='MD')
op('LDI', 'sd', imm=0)
op('LDI', 'W', imm=1280)
op('MOV', 'kw', k(13107))
op('MOV', 'kwsq', k(41942))
op('JLT', a='mh', b=k(1600), imm='MD')
op('LDI', 'W', imm=1920)
op('MOV', 'kw', k(8738))
op('MOV', 'kwsq', k(18640))
L('MD')
# zones: A = top 3/16, C = bottom 3/16, window inset H/64
op('SHR', 't1', 'fh', imm=3)
op('SHR', 't2', 'fh', imm=4)
op('ADD', 'yA', 't1', 't2')
op('SUB', 'yC', 'fh', 'yA')
op('SHR', 't3', 'fh', imm=6)
op('ADD', 'wy0', 'yA', 't3')
op('SUB', 'wy1', 'yC', 't3')
op('SHR', 'cy0', 'fh', imm=1)
op('SHR', 't4', 'yA', imm=3)
op('SUB', 'yh', 'yC', 't4')
out('yA', 'yA'); out('yC', 'yC'); out('wy0', 'wy0'); out('wy1', 'wy1')
op('SHR', 't1', 'W', imm=6)
op('SHR', 't2', 'W', imm=8)
op('ADD', 'wx0', 't1', 't2')
op('SUB', 't3', 'W', 'wx0')
op('SUB', 't3', 't3', k(1))
out('wx0', 'wx0'); out('wx1m', 't3')
op('SHR', 'cx0', 'W', imm=1)
op('SUB', 't1', 'cx0', k(1))
out('xhm1', 't1')
# per-pixel steps from kw = 2^24/W: cloud 5 / 3.75 cycles per width, sway
# 2.5 / 1.56
op('SHR', 't1', 'kw', imm=6)
op('SHR', 't2', 'kw', imm=8)
op('ADD', 'kS1', 't1', 't2')
out('kS1', 'kS1')
op('SHR', 't3', 'kS1', imm=2)
op('SUB', 'kS2', 'kS1', 't3')
out('kS2', 'kS2')
op('SHR', 't1', 'kS1', imm=1)
out('kW1x', 't1')
op('SHR', 't4', 'kS1', imm=4)
op('ADD', 't3', 't3', 't4')
out('kW2x', 't3')

# ---- knob laws ----
op('MAX', 'o', 'K12', k(6))
op('SHR', 't1', 'K6', imm=4)
op('ADD', 'nbc', 't1', k(8))
op('ADD', 'nCq', 't1', k(24))
op('SHR', 't1', 'K12', imm=4)
op('ADD', 'Ab', 't1', k(12))
out('boost', 't1')
op('SHR', 't2', 'K12', imm=5)
op('SUB', 't3', 'K12', 't2')
op('ADD', 'ampR', 't3', k(245))
op('ADD', 't3', 't2', k(4))
out('spk', 't3')
op('SHR', 't1', 'K3', imm=5)
op('ADD', 't1', 't1', k(6))
out('kR', 't1')
op('SUB', 't1', k(1023), 'K3')                  # corona slope: (1+sm/4)*2^se
op('SHR', 't1', 't1', imm=5)
op('SHR', 't2', 't1', imm=2)
out('se', 't2')
op('AND', 't2', 't1', imm=3)
out('sm', 't2')
op('SHR', 't1', 'K12', imm=8)
op('ADD', 't2', 't1', k(3))
out('g3', 't2')
op('SUB', 't2', k(3), 't1')
out('rsh', 't2')
op('SUB', 't1', k(1023), 'K12')
op('SHR', 't1', 't1', imm=3)
op('ADD', 't1', 't1', k(60))
out('pup2', 't1')
op('SHR', 't1', 'K4', imm=6)
out('as4', 't1')
op('SHR', 't1', 'K4', imm=4)
out('lenm', 't1')
# flow law: square (fine control at the slow end)
op('MUL', a='K5', b='K5')
op('MRD', 'fl2', imm=0)
op('SHR', 'fl2', 'fl2', imm=10)
# motion phases (Q16 in the low bits)
op('SHL', 't1', 'fl2', imm=1)
op('ADD', 'tA', 'tA', 't1')
op('ADD', 'tRg', 'tRg', 't1')
op('ADD', 'tR', 'tR', 'fl2')
op('ADD', 'tC', 'tC', 'fl2')
op('SHR', 't2', 'fl2', imm=1)
op('ADD', 'tW', 'tW', 't2')
op('ADD', 'tG', 'tG', 't2')
op('SHR', 't3', 'fl2', imm=2)
op('ADD', 'tK', 'tK', 't3')
op('ADD', 'tM', 'tM', 't3')
op('ADD', 'tgx', 'tgx', k(47))
op('ADD', 'tgy', 'tgy', k(29))
op('ADD', 'fc', 'fc', k(1))
out('tM', 'tM'); out('tG', 'tG'); out('tRg', 'tRg')
op('MOV', 'c_tw1', 'tW')
op('SHR', 't1', 'tW', imm=1)
op('ADD', 't1', 't1', 'tW')
op('NEG', 'c_tw2', 't1')
# bar base phase; the ripple product is biased by +8192*ampR
op('SHL', 't1', 'tA', imm=8)
op('SHL', 'tw13', 'ampR', imm=13)               # phase term: wraps mod 2^24
op('SUB', 'phA0', 't1', 'tw13')

# ---- eye: half-height, width at full open, aura normaliser ----
op('MUL', a='fh', b='K1')
op('MRD', 't1', imm=0)
op('SHR', 't1', 't1', imm=11)
op('SHR', 't2', 'fh', imm=5)
op('ADD', 'hh', 't1', 't2')
op('MAX', 'hh', 'hh', k(32))
op('DIV', a=k(1 << 20), b='hh')
op('DRD', 'ihh')
op('MOV', 'wmax', 'hh')
op('JNZ', a='sd', imm='WSD')
op('JMP', imm='WSE')
L('WSD')
op('SHR', 't1', 'hh', imm=3)                    # SD pixel aspect
op('ADD', 'wmax', 'wmax', 't1')
L('WSE')
op('MUL', a='wmax', b='o')
op('MRD', 't1', imm=0)
op('SHR', 'We4', 't1', imm=6)                   # open half-width, Q4 px
op('SHR', 'We', 'We4', imm=4)
op('SHR', 't1', 'hh', imm=2)
op('MAX', 'Wa', 'We', 't1')
op('MAX', 'Wa', 'Wa', k(16))
# ---- gaze: slow Lissajous wander, x ~ W/4, y ~ H/16 at full ----
op('MUL', a='W', b='K2')
op('MRD', 'ampX', imm=12)
op('MUL', a='fh', b='K2')
op('MRD', 'ampY', imm=12)
op('SHR', 'ampY', 'ampY', imm=2)
L('AX3')
op('SHR', 't1', 'tgx', imm=6)
op('SIN', 'sx', 't1')
op('ABS', 't2', 'sx')
op('MUL', a='t2', b='ampX')
op('MRD', 't3', imm=12)
op('ADD', 'cx', 'cx0', 't3')
op('JGE', a='sx', b='zero', imm='CX1')
op('SUB', 'cx', 'cx0', 't3')
L('CX1')
out('cx', 'cx')
op('SHR', 't1', 'tgy', imm=6)
op('SIN', 'sy', 't1')
op('ABS', 't2', 'sy')
op('MUL', a='t2', b='ampY')
op('MRD', 't3', imm=12)
op('ADD', 'cy', 'cy0', 't3')
op('JGE', a='sy', b='zero', imm='CY1')
op('SUB', 'cy', 'cy0', 't3')
L('CY1')

# ---- rho^2 DDA: u^2 = s*(x-cx)^2 in Q24 with s = 2^24/Wa^2, started n
# columns left of the eye (n = min(cx, 4 Wa): beyond that rho^2 >= 16)
op('DIV', a=k((1 << 24) - 1), b='Wa')
op('DRD', 't1')
op('DIV', a='t1', b='Wa')
op('DRD', 'hs')
op('SHL', 't2', 'Wa', imm=2)
op('MIN', 'hn', 'cx', 't2')
op('SUB', 't3', 'cx', 'hn')
op('SUB', 't3', 't3', k(1))
out('hxm1', 't3')                               # start column - 1
op('MUL', a='hs', b='hn')
op('MRD', 'hp', imm=0)                          # s*n
op('SHR', 't3', 'hp', imm=8)
op('MUL', a='t3', b='hn')
op('MRD', 't4', imm=0)
op('MOV', 'c_hq0', 't4')                        # Q(xs) = s*n^2 >> 8 (XFER shifts)
op('SHL', 't5', 'hp', imm=1)
op('SUB', 'c_hd0', 'hs', 't5')                  # D(xs) = s*(1 - 2n)
op('SHL', 't5', 'hs', imm=1)
out('hs2', 't5')                                # D step = 2s

# ---- bars: chirp from 2.5x at the edges to 1x at the centre ----
op('MUL', a='kw', b='nbc')
op('MRD', 'fAc', imm=0)
op('SHL', 't1', 'fAc', imm=1)
op('SHR', 't2', 'fAc', imm=1)
op('ADD', 'c_frA', 't1', 't2')
op('SHR', 't1', 'kwsq', imm=2)
op('SHL', 't2', 'nbc', imm=1)
op('ADD', 't2', 't2', 'nbc')
op('MUL', a='t1', b='t2')
op('MRD', 't3', imm=0)
op('SHR', 'cA', 't3', imm=10)
out('cA', 'cA')

# ---- bands: flat step per row, perspective numerator, phase at row 0 ----
op('DIV', a=k(1 << 20), b='yA')
op('DRD', 'ihc')
op('MUL', a='ihc', b='nCq')
op('MRD', 't1', imm=0)
op('SHR', 'kC', 't1', imm=8)
op('MUL', a='yA', b='nCq')
op('MRD', 't1', imm=0)
op('SHL', 'tkz', 't1', imm=9)                  # Kz < 2^24, read unsigned by DIV
op('SHL', 't3', 't1', imm=6)
op('ADD', 'Kz', 'tkz', 't3')
op('MUL', a='kC', b='yC')
op('MRD', 't1', imm=0)
op('SUB', 'aCf', 'tC', 't1')
op('JNZ', a='r0', imm='AC1')
op('JMP', imm='AC2')
L('AC1')
op('ADD', 'aCf', 'aCf', 'kC')
L('AC2')
# per-row accumulator steps
op('SHR', 't1', 'ihc', imm=3)
op('SHR', 't2', 'ihc', imm=4)
op('ADD', 't3', 't1', 't2')
op('SHR', 't4', 'ihc', imm=6)
op('ADD', 'kR1', 't3', 't4')
op('SHR', 't1', 'ihc', imm=2)
op('ADD', 't3', 't1', 't2')
op('SHR', 't4', 'ihc', imm=5)
op('ADD', 'kR2', 't3', 't4')
op('SHR', 'kW1', 'ihc', imm=5)
op('SHR', 't1', 'ihc', imm=6)
op('ADD', 'kW2', 'kW1', 't1')
op('SHR', 'kD1', 'kS1', imm=1)
op('SHR', 'kD2', 'kS2', imm=1)
op('SHR', 't1', 'sw', imm=3)
op('AND', 'hor', 't1', imm=1)

# ---- tear: re-roll two bands every 4 fields (a burst in 3 of 8) ----
op('CTL', 'rnd', imm=CTLR['rand'])
op('AND', 't1', 'fc', imm=7)
op('JNZ', a='t1', imm='TS1')
out('tseed', 'rnd')
L('TS1')
op('AND', 'tear', 'sw', imm=1)
op('JNZ', a='tear', imm='TON')
op('LDI', 'ta0', imm=0)
op('LDI', 'ta1', imm=0)
op('JMP', imm='TEND')
L('TON')
op('AND', 't1', 'fc', imm=3)
op('JNZ', a='t1', imm='TEND')
op('LDI', 'ta0', imm=0)
op('LDI', 'ta1', imm=0)
op('AND', 't1', 'rnd', imm=7)
op('JGE', a='t1', b=k(3), imm='TPK')
op('LDI', 'ta0', imm=1)
op('SHR', 't1', 'rnd', imm=9)
op('AND', 'ta1', 't1', imm=1)
L('TPK')
for i in (0, 1):
    op('CTL', 't5', imm=CTLR['rand'])
    op('AND', f'ts{i}', 't5', imm=2047)        # start row (>= fh: off screen)
    op('SHR', 't1', 't5', imm=11)
    op('AND', 't1', 't1', imm=(127 if i == 0 else 63))
    op('ADD', f'th{i}', 't1', k(2))            # height 2..129 rows
    op('CTL', 't6', imm=CTLR['rand'])
    op('AND', 't3', 't6', imm=255)
    op('SUB', f'to{i}', 't3', k(40))           # shift -40..+215 px
L('TEND')

# ---- line accumulators at row r0, then the first two lines ----
op('MOV', 'rc', 'r0')
op('MOV', 'aR1', 'tR')
op('SHR', 't1', 'tR', imm=1)
op('SUB', 'aR2', k(-1), 't1')
op('MOV', 'aW1', 'tK')
op('SUB', 'aW2', k(-1), 'tK')
op('SHR', 'aD1', 'tK', imm=1)
op('SUB', 'aD2', k(-1), 'tK')
op('LDI', 'w4p', imm=0)
call('LINE', 'FR1')
op('XFER', a=REG['c_rc'])
call('LINE', 'FR2')
L('LOOP')
op('WAIT', imm=1)
op('XFER', a=REG['c_rc'])
call('LINE', 'LR1')
op('JMP', imm='LOOP')

# ======================================================================
# SMOOTH: t0 raw, kh held, ke glide (Q10.4) -> kh, ke
# ======================================================================
L('SMOOTH')
# the first three fields after load snap straight to the knob (no glide in
# from the power-up value); fc goes negative only after 2^23 fields
op('JLT', a='fc', b='zero', imm='SM0')
op('JGE', a='fc', b=k(3), imm='SM0')
op('MOV', 'kh', 't0')
op('SHL', 'ke', 't0', imm=4)
op('JR', a='ra')
L('SM0')
op('SUB', 't1', 't0', 'kh')
op('ABS', 't2', 't1')
op('JLT', a='t2', b=k(5), imm='SM1')
op('MOV', 'kh', 't0')                           # moved 5+ counts: follow
L('SM1')
op('SHL', 't3', 'kh', imm=4)
op('SUB', 't4', 't3', 'ke')
op('ABS', 't5', 't4')
op('SHR', 't4', 't4', imm=3)
op('ADD', 'ke', 'ke', 't4')                     # glide 1/8 of the gap
op('JGE', a='t5', b=k(8), imm='SM2')
op('MOV', 'ke', 't3')                           # snap inside half a count
L('SM2')
op('JR', a='ra')

# ======================================================================
# LINE: row rc -> line block; then step the accumulators
# ======================================================================
L('LINE')
op('MOV', 'c_rc', 'rc')
op('CTL', 'lj', imm=CTLR['rand'])
# ripple: two interpolation-free sines x amplitude (+ twinkle)
op('SHR', 't1', 'aR1', imm=6)
op('SIN', 't1', 't1')
op('SHR', 't2', 'aR2', imm=6)
op('SIN', 't2', 't2')
op('ADD', 't1', 't1', 't2')
op('ADD', 't1', 't1', k(8192))
op('MUL', a='t1', b='ampR')
op('MRD', 'tph', imm=0)                         # a phase: wraps mod 2^24
op('AND', 't2', 'lj', imm=31)
op('SHL', 't2', 't2', imm=13)
op('ADD', 'tph', 'tph', 't2')
op('ADD', 'c_phA', 'tph', 'phA0')
# cloud warps
op('SHR', 't1', 'aW1', imm=6)
op('SIN', 't1', 't1')
op('SHL', 't1', 't1', imm=2)
op('ADD', 'c_w1', 'aD1', 't1')
op('SHR', 't1', 'aW2', imm=6)
op('SIN', 't1', 't1')
op('SHL', 't1', 't1', imm=2)
op('ADD', 'c_w2', 'aD2', 't1')
# eye: v = |rc - cy| / hh (Q10), v^2, lens base
op('SUB', 'dys', 'rc', 'cy')
op('ABS', 'dy', 'dys')
op('MUL', a='ihh', b='dy')
op('MRD', 'v', imm=12)
op('SHL', 'v', 'v', imm=2)
op('MIN', 'v', 'v', k(4095))
op('MUL', a='v', b='v')
op('MRD', 't2', imm=12)
op('SHL', 't2', 't2', imm=2)
op('MIN', 'c_v2', 't2', k(16383))
op('SHL', 't2', 'v', imm=7)
op('SHL', 't3', 'v', imm=6)
op('ADD', 't2', 't2', 't3')
op('ADD', 'c_lb', 'tC', 't2')
op('JGE', a='dys', b='zero', imm='LB1')
op('SUB', 'c_lb', 'tC', 't2')
L('LB1')
op('LDI', 'c_eye', imm=0)
op('LDI', 'w4', imm=0)
op('JGE', a='v', b=k(1024), imm='NOEYE')
op('LDI', 'c_eye', imm=1)
op('SHR', 'idx', 'v', imm=2)
op('PRV', 'pv', 'idx')
op('PRC', 'pc', 'idx')
op('AND', 't2', 'v', imm=2)
op('JNZ', a='t2', imm='HALF')
op('JMP', imm='NOINT')
L('HALF')                                       # half-step interpolation
op('ADD', 'idx', 'idx', k(1))
op('LDI', 't2', imm=0)
op('LDI', 't3', imm=0)
op('JGE', a='idx', b=k(256), imm='HZ')
op('PRV', 't2', 'idx')
op('PRC', 't3', 'idx')
L('HZ')
op('ADD', 'pv', 'pv', 't2')
op('SHR', 'pv', 'pv', imm=1)
op('ADD', 'pc', 'pc', 't3')
op('SHR', 'pc', 'pc', imm=1)
L('NOINT')
op('SUB', 't2', 'pc', 'pv')                     # vesica -> circle by opening
op('MAX', 't2', 't2', 'zero')
op('MUL', a='t2', b='o')
op('MRD', 't2', imm=0)
op('SHR', 't2', 't2', imm=10)
op('ADD', 'pm', 'pv', 't2')
op('MUL', a='We4', b='pm')
op('MRD', 'w4', imm=12)
L('NOEYE')
op('MOV', 'c_w4', 'w4')
# AA ramp: 2^k px ~ the boundary's slope against the previous line
op('SUB', 't2', 'w4', 'w4p')
op('ABS', 't1', 't2')
op('MOV', 'w4p', 'w4')
op('SHR', 't1', 't1', imm=4)
op('MIN', 't1', 't1', k(63))
op('LDI', 'c_k', imm=0)
L('KL')
op('JNZ', a='t1', imm='KI')
op('JMP', imm='KD')
L('KI')
op('SHR', 't1', 't1', imm=1)
op('ADD', 'c_k', 'c_k', k(1))
op('JMP', imm='KL')
L('KD')
# bands: flat unless Deep and in zone C
op('MOV', 'c_phC', 'aCf')
op('MOV', 'c_swA', 'Ab')
op('LDI', 'c_ah', imm=0)
op('LDI', 'c_aref', imm=0)
op('LDI', 'c_ref', imm=0)
op('JLT', a='rc', b='yC', imm='BDONE')
op('JNZ', a='hor', imm='DEEP')
op('JMP', imm='BDONE')
L('DEEP')
op('SUB', 'dz', 'rc', 'yh')
op('DIV', a='Kz', b='dz')
op('DRD', 't1')
op('SUB', 'c_phC', 't1', 'tC')                  # bands recede into the haze
op('MUL', a='ihc', b='dz')
op('MRD', 'dzn', imm=0)
op('SHR', 'dzn', 'dzn', imm=12)
op('MIN', 'dzn', 'dzn', k(511))
op('SUB', 't1', 'dzn', k(32))                   # haze: 15 at the horizon
op('MAX', 't1', 't1', 'zero')
op('SHR', 't1', 't1', imm=2)
op('SUB', 't1', k(15), 't1')
op('MAX', 'c_ah', 't1', 'zero')
op('SHL', 't1', 'dzn', imm=1)                   # reflection fades with depth
op('ADD', 't1', 't1', 'dzn')
op('SHR', 't1', 't1', imm=6)
op('SUB', 't1', k(13), 't1')
op('MAX', 'c_aref', 't1', k(1))
op('SHR', 't1', 'dzn', imm=2)                   # deep sway grows toward the viewer
op('SHR', 't2', 'Ab', imm=1)
op('ADD', 't1', 't1', 't2')
op('MIN', 'c_swA', 't1', k(127))
op('SHR', 't1', 'lj', imm=12)                   # reflection column, glinting
op('AND', 't1', 't1', imm=15)
op('ADD', 't2', 'cx', 't1')
op('SUB', 't2', 't2', k(8))
op('SHR', 't3', 'We4', imm=5)
op('SHR', 't4', 'dz', imm=2)
op('ADD', 't3', 't3', 't4')
op('ADD', 't3', 't3', k(2))
op('SUB', 'c_rx0', 't2', 't3')
op('MAX', 'c_rx0', 'c_rx0', 'zero')
op('ADD', 'c_rx1', 't2', 't3')
op('SHR', 't1', 'lj', imm=8)
op('AND', 't1', 't1', imm=3)
op('JNZ', a='t1', imm='RF1')
op('JMP', imm='BDONE')
L('RF1')
op('LDI', 'c_ref', imm=1)
L('BDONE')
# tear: rows inside a band read the output buffer with their own delay
op('LDI', 'c_d', imm=40)
op('LDI', 'c_dc', imm=40)
for i in (0, 1):
    op('JNZ', a=f'ta{i}', imm=f'TC{i}')
    op('JMP', imm=f'TN{i}')
    L(f'TC{i}')
    op('SUB', 't1', 'rc', f'ts{i}')
    op('JLT', a='t1', b='zero', imm=f'TN{i}')
    op('JGE', a='t1', b=f'th{i}', imm=f'TN{i}')
    op('MOV', 't4', f'to{i}')
    op('JMP', imm='TIN')
    L(f'TN{i}')
op('JMP', imm='TDN')
L('TIN')
op('AND', 't1', 'lj', imm=15)
op('ADD', 't1', 't1', 't4')
op('ADD', 't1', 't1', k(32))                    # D0 - 8
op('MAX', 't1', 't1', k(2))
op('MIN', 'c_d', 't1', k(240))
op('SHR', 't2', 'lj', imm=5)
op('AND', 't2', 't2', imm=3)
op('ADD', 't2', 't2', k(1))
op('ADD', 'c_dc', 'c_d', 't2')
L('TDN')
# accumulators: once per frame row (twice per field line on interlace)
L('ACC')
op('ADD', 'aR1', 'aR1', 'kR1')
op('ADD', 'aR2', 'aR2', 'kR2')
op('ADD', 'aW1', 'aW1', 'kW1')
op('ADD', 'aW2', 'aW2', 'kW2')
op('ADD', 'aD1', 'aD1', 'kD1')
op('SUB', 'aD2', 'aD2', 'kD2')
op('ADD', 'aCf', 'aCf', 'kC')
op('ADD', 'rc', 'rc', k(1))
op('SUB', 't1', 'rc', 'c_rc')
op('JLT', a='t1', b='step', imm='ACC')
op('JR', a='ra')

WORDS = A.words()


# ======================================================================
# tables (shared with oracle_tables_pkg.vhd)
# ======================================================================
def read_table(name):
    t = open(os.path.join(HERE, '..', 'oracle_tables_pkg.vhd')).read()
    m = re.search(name + r' : \S+ := \((.*?)\);', t, re.S)
    return [int(x) for x in re.findall(r'\(\s*(-?\d+),', m.group(1))]


def emulate(fh, mh, ilace, knobs, fields=3, dump=False):
    qsin, pves, pcir = read_table('C_QSIN'), read_table('C_PVES'), read_table('C_PCIR')
    state = {'lfsr': 0xACE1, 'botf': 0}

    def rand():
        v = state['lfsr']
        for _ in range(7):
            v = (v >> 1) ^ (0xB400 if v & 1 else 0)
        state['lfsr'] = v
        return v
    ctl = {CTLR['k0']: knobs[0], CTLR['k1']: knobs[1], CTLR['k2']: knobs[2],
           CTLR['k3']: knobs[3], CTLR['k4']: knobs[4], CTLR['k5']: knobs[5],
           CTLR['sw']: knobs[6], CTLR['k7']: knobs[7],
           CTLR['fh']: fh // (2 if ilace else 1) * (2 if ilace else 1),
           CTLR['mh']: mh, CTLR['ilace']: ilace,
           CTLR['botf']: lambda: state['botf'], CTLR['rand']: rand}
    wrap = [REG[n] for n in ('tA', 'tRg', 'tR', 'tC', 'tW', 'tG', 'tK', 'tM', 'tgx', 'tgy',
                             'fc', 'aR1', 'aR2', 'aW1', 'aW2', 'aD1', 'aD2', 'aCf', 'phA0',
                             'c_phA', 'c_lb', 'c_phC', 'c_w1', 'c_w2', 'c_tw1', 'c_tw2',
                             'Kz', 'tph', 'tw13', 'tkz')]
    e = Emu(A.resolve(), INIT, qsin, pves, pcir, ctl, wrap_ok=wrap,
            names={v: k for k, v in REG.items()})
    rows_per_field = fh // (2 if ilace else 1)
    worst = 0
    for f in range(fields):
        state['botf'] = f & 1 if ilace else 0
        e.xfers = []
        fc = e.run('field')
        lines = []
        for ln in range(rows_per_field):
            c = e.run('line')
            worst = max(worst, c)
            lines.append(c)
        if dump and f == fields - 1:
            print(f'field cycles {fc}, line cycles max {max(lines)} mean {sum(lines)//len(lines)}')
    return e, worst, fc


def main():
    print(f'program: {len(WORDS)} words, {len(REG)} registers ({len(CONST)} constants)')
    assert len(WORDS) <= 512
    knobs = [512, 300, 512, 400, 380, 512, 0b01001, 560]
    budget = {'1080p': 2200, '1080i': 2200, 'ntsc': 858, '720p': 1650}
    for name, fh, mh, il in (('1080p', 1080, 1920, 0), ('1080i', 1080, 1920, 1),
                             ('720p', 720, 1280, 0), ('ntsc', 486, 720, 1)):
        for kn in (knobs, [1023] * 6 + [0b11111] + [1023], [0] * 6 + [0b01000] + [0]):
            e, worst, fc = emulate(fh, mh, il, kn)
            assert not e.overflows, f'24-bit overflow: {e.overflows}'
            ok = 'OK' if worst + 64 < budget[name] else 'OVER'
            print(f'{name:6s} knobs {kn[0]:4d}..: field {fc:5d} clk, worst line {worst:4d} clk '
                  f'(line period {budget[name]}) {ok}')
    if '--dump' in sys.argv:
        e, worst, fc = emulate(1080, 1920, 0, knobs, dump=True)
        P = {v: k for k, v in PORTS.items()}
        print({P[p]: s32(v) for p, v in sorted(e.ports.items())})
        for i, x in enumerate(e.xfers):
            if i % 60 == 0 or 540 < x[0] < 548:
                print(dict(zip(XFER_FIELDS, x)))
    with open(os.path.join(HERE, '..', 'oracle_ucode_pkg.vhd'), 'w') as f:
        f.write('-- oracle_ucode_pkg.vhd -- GENERATED by gen/oracle_ucode.py, do not edit.\n\n'
                'library ieee;\nuse ieee.std_logic_1164.all;\n\n'
                'package oracle_ucode_pkg is\n'
                + emit_vhdl(WORDS) + '\n' + emit_rf_init(INIT) + '\n'
                f'    constant C_XFER_BASE : integer := {REG["c_rc"]};\n'
                'end package;\n')


if __name__ == '__main__':
    main()
