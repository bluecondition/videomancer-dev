#!/usr/bin/env python3
"""CUBIST per-frame microcode, v0.3: a turning cube.

Runs once per field in vertical blanking on the engine in cubist.vhd
(assembled by uasm.py, emulated by uemu.py).  Each field it:

  1. reads the controls: knobs 1-3 (orientation, deadbanded and glided),
     switches S7 run (off = solved) / S8-S10 auto yaw, pitch, roll, slider =
     layer-turn speed;
  2. steps the layer turn: a random face (never the same one twice running)
     turns a quarter turn in a random direction with an eased angle, then the
     sticker state is permuted and the next turn starts;
  3. builds the cube basis from yaw/pitch/roll, and from it two boxes -- the
     turning SLICE (3x3x1, rotated by theta about the turn axis) and the
     BLOCK (3x3x2) -- and their visible faces, near box first;
  4. for each visible face (<= 6 slots) writes the pixel path's descriptors:
     the u/v DDAs (gradient, field seed, line wrap), the face descriptor
     (sticker base, shape, gap edge, silhouette edges), the half-vector
     projections, the flat light and the lit sticker chroma per colour.

Screen geometry is in Q6 pixels throughout (no whole-pixel rounding, so
faces neither skew nor snap as the cube moves); UV gradients come from the
projected box axes, not from rounded corner deltas.
"""
import sys
sys.path.insert(0, '.')
from uasm import Asm, emit_vhdl, SLOTW, CTLR
import turn_model as tm

a = Asm()

# ------------------------------------------------------------ register map
ZERO, ONE = 0, 1
T = list(range(2, 14))                      # T0..T11 scratch
T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11 = T
RET, RET2 = 14, 15
KH = [16, 17, 18]                           # deadbanded knob values
GA = [19, 20, 21]                           # glided knob angles (16-bit)
AP = [22, 23, 24]                           # auto-rotate phases
SW, SLD, TT, TF, TD, TACT, TPREV = 25, 26, 27, 28, 29, 30, 31
TRG = 32                                    # 32..39: sin/cos yaw,pitch,roll,theta
B = 44                                      # 44..52 basis B[3i + comp], Q12
Z, CX6, CY6, YST, Y0, K16M, TH = 53, 54, 55, 56, 57, 58, 59
BOX0, BOX1 = 60, 86                         # 26 fields per box
PX, PY, NZ, HD, KD, FD, HPX, HPY, CBX, CBY = 0, 3, 6, 9, 12, 15, 18, 20 + 1, 24, 25
VIS = 112                                   # 112..123: VIS[6*box + face]
FIRST, SLOT, AAX, ASIDE = 124, 125, 126, 127

a.zero = ZERO

# face-loop names (reuse 32..43, free once the boxes are built)
BOX, FI, BB, NCD, UCD, VCD = 32, 33, 34, 35, 36, 37
SUX, SUY, SVX, SVY, OX, OY = 38, 39, 40, 41, 42, 43
DS, DA = RET2, TH                           # sign and magnitude of D

C_HX, C_HY, C_HZ = -57, 81, 236             # half-vector, Q8
KEY = (-107, 154, 184)                      # key light direction, Q8-ish
FILL = (141, -141, 161)
RATES = (72, 45, 28)                        # auto yaw/pitch/roll, per field
C_AMB = 62

_n = [0]


def L(stem):
    _n[0] += 1
    return f'{stem}_{_n[0]}'


def call(sub):
    r = L('ret')
    a.emit('LDI', dst=RET, imm=r)
    a.emit('JMP', imm=sub)
    a.label(r)


def negif(dst, src, flag):
    """dst = -src if flag /= 0 else src"""
    l1, l2 = L('ng'), L('ng')
    a.emit('JNZ', a=flag, imm=l1)
    a.emit('MOV', dst=dst, a=src)
    a.emit('JMP', imm=l2)
    a.label(l1)
    a.emit('NEG', dst=dst, a=src)
    a.label(l2)


def ldi(dst, v, scratch):
    """any 32-bit constant (scratch is clobbered)"""
    assert scratch != dst
    if -8192 <= v < 8192:
        a.emit('LDI', dst=dst, imm=v)
        return
    hi, lo = v >> 12, v & 0xFFF
    ldi(dst, hi, scratch)
    a.emit('SHL', dst=dst, a=dst, imm=12)
    if lo:
        a.emit('LDI', dst=scratch, imm=lo)
        a.emit('ADD', dst=dst, a=dst, b=scratch)


a.emit('JMP', imm='main')

# ======================================================= subroutines
# TRIG: T1 = sin(T0) in Q12, T0 a 16-bit angle (65536 = one turn).  Folded
# quarter table (65 entries), linear interpolation on the low 8 bits.
a.label('TRIG')
a.emit('SHR', dst=T3, a=T0, imm=8)
a.emit('AND', dst=T3, a=T3, imm=63)
a.emit('BIT', dst=T4, a=T0, imm=14)
a.emit('JNZ', a=T4, imm='tr_mir')
a.emit('SIN', dst=T5, a=T3)
a.emit('ADD', dst=T3, a=T3, b=ONE)
a.emit('SIN', dst=T3, a=T3)
a.emit('JMP', imm='tr_int')
a.label('tr_mir')
a.emit('LDI', dst=T4, imm=64)
a.emit('SUB', dst=T3, a=T4, b=T3)
a.emit('SIN', dst=T5, a=T3)
a.emit('SUB', dst=T3, a=T3, b=ONE)
a.emit('SIN', dst=T3, a=T3)
a.label('tr_int')
a.emit('SUB', dst=T3, a=T3, b=T5)
a.emit('AND', dst=T2, a=T0, imm=255)
a.emit('MUL', a=T3, b=T2)
a.emit('MRD', dst=T3, imm=0)
a.emit('SHR', dst=T3, a=T3, imm=8)
a.emit('ADD', dst=T1, a=T5, b=T3)
a.emit('BIT', dst=T4, a=T0, imm=15)
a.emit('JNZ', a=T4, imm='tr_neg')
a.emit('JR', a=RET)
a.label('tr_neg')
a.emit('NEG', dst=T1, a=T1)
a.emit('JR', a=RET)

# DIVS: T1 = +/- min((|T0| << 16) / DA, 2^20-1), negative iff T0 < 0 xor DS.
# The divider takes magnitudes only; the signs are handled here.
a.label('DIVS')
a.emit('JLT', a=T0, b=ZERO, imm='dv_neg')
a.emit('DIV', a=T0, b=DA, imm=16)
a.emit('DRD', dst=T1)
a.emit('JNZ', a=DS, imm='dv_flip')
a.emit('JR', a=RET)
a.label('dv_neg')
a.emit('NEG', dst=T0, a=T0)
a.emit('DIV', a=T0, b=DA, imm=16)
a.emit('DRD', dst=T1)
a.emit('JNZ', a=DS, imm='dv_out')
a.label('dv_flip')
a.emit('NEG', dst=T1, a=T1)
a.label('dv_out')
a.emit('JR', a=RET)

# FCODE: axis codes of face T0 (U D R L F B): T1 = normal, T2 = U, T3 = V.
# normal = face ^ 2 for faces 0..3; U = 4 - 2p, V = (6 - 2p) mod 6 for the
# face pair p = face >> 1, swapped on odd faces.
a.label('FCODE')
a.emit('MOV', dst=T1, a=T0)
a.emit('LDI', dst=T4, imm=4)
a.emit('JGE', a=T0, b=T4, imm='fc_n')
a.emit('LDI', dst=T4, imm=2)
a.emit('ADD', dst=T1, a=T0, b=T4)
a.emit('JLT', a=T0, b=T4, imm='fc_n')
a.emit('SUB', dst=T1, a=T0, b=T4)
a.label('fc_n')
a.emit('SHR', dst=T4, a=T0, imm=1)
a.emit('ADD', dst=T4, a=T4, b=T4)                 # 2p
a.emit('LDI', dst=T2, imm=4)
a.emit('SUB', dst=T2, a=T2, b=T4)
a.emit('LDI', dst=T3, imm=6)
a.emit('SUB', dst=T3, a=T3, b=T4)
a.emit('LDI', dst=T4, imm=6)
a.emit('JLT', a=T3, b=T4, imm='fc_v')
a.emit('LDI', dst=T3, imm=0)
a.label('fc_v')
a.emit('AND', dst=T4, a=T0, imm=1)
a.emit('JNZ', a=T4, imm='fc_sw')
a.emit('JR', a=RET)
a.label('fc_sw')
a.emit('MOV', dst=T4, a=T2)
a.emit('MOV', dst=T2, a=T3)
a.emit('MOV', dst=T3, a=T4)
a.emit('JR', a=RET)

# ======================================================= main
a.label('main')
a.emit('LDI', dst=ZERO, imm=0)
a.emit('LDI', dst=ONE, imm=1)

# ---------------------------------------------------- 1. controls
a.emit('CTL', dst=SW, imm=CTLR['sw'])
a.emit('CTL', dst=SLD, imm=CTLR['slider'])
a.emit('CTL', dst=T6, imm=CTLR['zfirst'])
RAW, RATE = TRG, TRG + 3                    # scratch until the trig runs
for k in range(3):
    a.emit('CTL', dst=RAW + k, imm=CTLR['k%d' % (k + 1)])
    a.emit('LDI', dst=RATE + k, imm=RATES[k])
a.emit('SHR', dst=T11, a=SW, imm=1)               # S8.. walk down to bit 0
a.emit('LDI', dst=T7, imm=0)                      # k
a.label('kn')
# deadband: the pot's ADC noise must never reach the rotation
a.emit('LDX', dst=T0, a=T7, imm=RAW)
a.emit('LDX', dst=T8, a=T7, imm=KH[0])
a.emit('SUB', dst=T1, a=T0, b=T8)
a.emit('ABS', dst=T1, a=T1)
a.emit('LDI', dst=T2, imm=3)
a.emit('JLT', a=T1, b=T2, imm='kn_keep')
a.emit('MOV', dst=T8, a=T0)
a.label('kn_keep')
a.emit('JNZ', a=T6, imm='kn_snap')
a.emit('JMP', imm='kn_glide')
a.label('kn_snap')
a.emit('MOV', dst=T8, a=T0)
a.label('kn_glide')
a.emit('STX', a=T7, b=T8, imm=KH[0])
# glide toward the knob the short way round (16-bit wrap), 1/8 per field
a.emit('SHL', dst=T0, a=T8, imm=6)
a.emit('LDX', dst=T9, a=T7, imm=GA[0])
a.emit('SUB', dst=T1, a=T0, b=T9)
a.emit('SHL', dst=T1, a=T1, imm=8)
a.emit('SHL', dst=T1, a=T1, imm=8)
a.emit('SHR', dst=T1, a=T1, imm=8)
a.emit('SHR', dst=T1, a=T1, imm=11)
a.emit('ADD', dst=T9, a=T9, b=T1)
a.emit('JNZ', a=T6, imm='kn_gs')
a.emit('JMP', imm='kn_gd')
a.label('kn_gs')
a.emit('MOV', dst=T9, a=T0)
a.label('kn_gd')
a.emit('STX', a=T7, b=T9, imm=GA[0])
# auto-rotate: S8 yaw, S9 pitch, S10 roll
a.emit('AND', dst=T1, a=T11, imm=1)
a.emit('JNZ', a=T1, imm='kn_auto')
a.emit('JMP', imm='kn_next')
a.label('kn_auto')
a.emit('LDX', dst=T2, a=T7, imm=AP[0])
a.emit('LDX', dst=T3, a=T7, imm=RATE)
a.emit('ADD', dst=T2, a=T2, b=T3)
a.emit('STX', a=T7, b=T2, imm=AP[0])
a.label('kn_next')
a.emit('SHR', dst=T11, a=T11, imm=1)
a.emit('ADD', dst=T7, a=T7, b=ONE)
a.emit('LDI', dst=T0, imm=3)
a.emit('JLT', a=T7, b=T0, imm='kn')

# ---------------------------------------------------- 2. the layer turn
# S7 run: off = solved -- re-written every field while off (idempotent, so a
# bouncing switch can't matter), which also snaps a turn in progress home
a.emit('BIT', dst=T0, a=SW, imm=0)
a.emit('JNZ', a=T0, imm='t_run')
a.emit('JMP', imm='t_reset')
a.label('t_run')
a.emit('JNZ', a=TACT, imm='t_adv')

a.label('t_start')                                # random face, not the last
a.emit('CTL', dst=T0, imm=CTLR['rand'])
a.emit('LDI', dst=T5, imm=5)
a.label('t_pick')
a.emit('AND', dst=T1, a=T0, imm=7)
a.emit('LDI', dst=T2, imm=6)
a.emit('JGE', a=T1, b=T2, imm='t_pnx')
a.emit('SUB', dst=T3, a=T1, b=TPREV)
a.emit('JNZ', a=T3, imm='t_got')
a.label('t_pnx')
a.emit('SHR', dst=T0, a=T0, imm=3)
a.emit('SUB', dst=T5, a=T5, b=ONE)
a.emit('JNZ', a=T5, imm='t_pick')
a.emit('ADD', dst=T1, a=TPREV, b=ONE)             # fallback: the next face
a.emit('LDI', dst=T2, imm=6)
a.emit('JLT', a=T1, b=T2, imm='t_got')
a.emit('LDI', dst=T1, imm=0)
a.label('t_got')
a.emit('MOV', dst=TF, a=T1)
a.emit('MOV', dst=TPREV, a=T1)
a.emit('CTL', dst=T0, imm=CTLR['rand'])
a.emit('BIT', dst=TD, a=T0, imm=15)
a.emit('LDI', dst=TT, imm=0)
a.emit('LDI', dst=TACT, imm=1)
a.emit('JMP', imm='t_theta')

a.label('t_adv')                                  # slider = turn speed
a.emit('SHL', dst=T0, a=SLD, imm=2)
a.emit('ADD', dst=T0, a=T0, b=SLD)
a.emit('LDI', dst=T1, imm=410)
a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('ADD', dst=TT, a=TT, b=T0)
a.emit('BIT', dst=T0, a=TT, imm=16)
a.emit('JNZ', a=T0, imm='t_done')
a.emit('JMP', imm='t_theta')

a.label('t_done')                                 # permute the stickers
a.emit('LDI', dst=T0, imm=20)
a.emit('MUL', a=TF, b=T0)
a.emit('MRD', dst=T10, imm=0)
a.emit('LDI', dst=T0, imm=tm.TABLE_CYC)
a.emit('ADD', dst=T10, a=T10, b=T0)
a.emit('LDI', dst=T11, imm=5)
a.label('t_cyc')
for m in range(4):
    a.emit('SRD', dst=T0 + m, a=T10, imm=m)       # T0..T3 = members
for m in range(4):
    a.emit('SRD', dst=T4 + m, a=T0 + m, imm=0)    # T4..T7 = colours
a.emit('JNZ', a=TD, imm='t_rev')
a.emit('SWR', a=T1, b=T4)
a.emit('SWR', a=T2, b=T5)
a.emit('SWR', a=T3, b=T6)
a.emit('SWR', a=T0, b=T7)
a.emit('JMP', imm='t_cnx')
a.label('t_rev')
a.emit('SWR', a=T0, b=T5)
a.emit('SWR', a=T1, b=T6)
a.emit('SWR', a=T2, b=T7)
a.emit('SWR', a=T3, b=T4)
a.label('t_cnx')
a.emit('LDI', dst=T0, imm=4)
a.emit('ADD', dst=T10, a=T10, b=T0)
a.emit('SUB', dst=T11, a=T11, b=ONE)
a.emit('JNZ', a=T11, imm='t_cyc')
a.emit('LDI', dst=TACT, imm=0)
a.emit('LDI', dst=TT, imm=0)
a.emit('JMP', imm='t_idle')

a.label('t_reset')                                # write the solved colours
a.emit('LDI', dst=T0, imm=tm.TABLE_STATE)
a.emit('LDI', dst=T1, imm=0)
a.emit('LDI', dst=T2, imm=9)
a.label('t_rl')
a.emit('SWR', a=T0, b=T1)
a.emit('ADD', dst=T0, a=T0, b=ONE)
a.emit('SUB', dst=T2, a=T2, b=ONE)
a.emit('JNZ', a=T2, imm='t_rl')
a.emit('LDI', dst=T2, imm=9)
a.emit('ADD', dst=T1, a=T1, b=ONE)
a.emit('LDI', dst=T3, imm=6)
a.emit('JLT', a=T1, b=T3, imm='t_rl')
a.emit('LDI', dst=TACT, imm=0)
a.emit('LDI', dst=TT, imm=0)

a.label('t_idle')
a.emit('LDI', dst=TH, imm=0)
a.emit('JMP', imm='t_trig')

a.label('t_theta')                                # eased: 45 deg (1 - cos pi t)
a.emit('SHR', dst=T0, a=TT, imm=1)
call('TRIG_COS')
a.emit('LDI', dst=T2, imm=4096)
a.emit('ADD', dst=T2, a=T2, b=T2)                 # 8192
a.emit('ADD', dst=T1, a=T1, b=T1)
a.emit('SUB', dst=TH, a=T2, b=T1)
a.emit('JNZ', a=TD, imm='t_thn')
a.emit('JMP', imm='t_trig')
a.label('t_thn')
a.emit('NEG', dst=TH, a=TH)

# ---------------------------------------------------- 3. trig
ANG = TRG + 8                                     # 40..43
a.label('t_trig')
for k in range(3):
    a.emit('ADD', dst=ANG + k, a=GA[k], b=AP[k])
a.emit('MOV', dst=ANG + 3, a=TH)
a.emit('LDI', dst=T6, imm=0)                      # k
a.emit('LDI', dst=T7, imm=0)                      # 2k
a.label('tg')
a.emit('LDX', dst=T0, a=T6, imm=ANG)
call('TRIG')
a.emit('STX', a=T7, b=T1, imm=TRG)
a.emit('LDX', dst=T0, a=T6, imm=ANG)
call('TRIG_COS')
a.emit('STX', a=T7, b=T1, imm=TRG + 1)
a.emit('ADD', dst=T6, a=T6, b=ONE)
a.emit('ADD', dst=T7, a=T7, b=ONE)
a.emit('ADD', dst=T7, a=T7, b=ONE)
a.emit('LDI', dst=T0, imm=4)
a.emit('JLT', a=T6, b=T0, imm='tg')
a.emit('JMP', imm='basis')

# cos(T0) = sin(T0 + 16384); 16384 does not fit a 14-bit immediate
a.label('TRIG_COS')
a.emit('LDI', dst=T2, imm=4096)
a.emit('SHL', dst=T2, a=T2, imm=2)
a.emit('ADD', dst=T0, a=T0, b=T2)
a.emit('JMP', imm='TRIG')

# ---------------------------------------------------- 4. basis
a.label('basis')
for c in range(9):
    a.emit('LDI', dst=B + c, imm=4096 if c in (0, 4, 8) else 0)
# (a, b) := (a c - b s, a s + b c) per basis vector: roll, pitch, yaw
S_YAW, C_YAW, S_PIT, C_PIT, S_ROL, C_ROL, S_TH, C_TH = range(TRG, TRG + 8)
for n, (p0, p1, rc, rs) in enumerate(((0, 1, C_ROL, S_ROL), (1, 2, C_PIT, S_PIT),
                                      (2, 0, C_YAW, S_YAW))):
    a.emit('LDI', dst=T7, imm=0)                  # 3i
    lb = L('bas')
    a.label(lb)
    a.emit('LDX', dst=T4, a=T7, imm=B + p0)
    a.emit('LDX', dst=T5, a=T7, imm=B + p1)
    a.emit('MUL', a=T4, b=rc); a.emit('MRD', dst=T0, imm=12)
    a.emit('MUL', a=T5, b=rs); a.emit('MRD', dst=T1, imm=12)
    a.emit('MUL', a=T4, b=rs); a.emit('MRD', dst=T2, imm=12)
    a.emit('MUL', a=T5, b=rc); a.emit('MRD', dst=T3, imm=12)
    a.emit('SUB', dst=T0, a=T0, b=T1)
    a.emit('ADD', dst=T2, a=T2, b=T3)
    a.emit('STX', a=T7, b=T0, imm=B + p0)
    a.emit('STX', a=T7, b=T2, imm=B + p1)
    a.emit('LDI', dst=T0, imm=3)
    a.emit('ADD', dst=T7, a=T7, b=T0)
    a.emit('LDI', dst=T0, imm=9)
    a.emit('JLT', a=T7, b=T0, imm=lb)

# ---------------------------------------------------- 5. framing
# Zoom from K6: the cube's body diagonal (5.196 cubies) is 50..90% of the
# frame height, so it fits at every orientation.  Z = px per cubie in Q4
# = hf * F / 4096 with F = 6308 + k6*20209/4096 (3.08*hf at 100%).
# The glided zoom persists in table-RAM word 255 (every register is reused
# within a field; the pixel path reads sticker words only): it moves 1/8 per
# field toward the knob and ignores moves under 12 (about a Q4 pixel), so pot
# noise never makes the cube breathe.
a.emit('CTL', dst=T0, imm=CTLR['hf'])
a.emit('CTL', dst=T1, imm=CTLR['k6'])
a.emit('LDI', dst=T2, imm=5052); a.emit('SHL', dst=T2, a=T2, imm=2)
a.emit('ADD', dst=T2, a=T2, b=ONE)               # 20209 (14-bit immediates)
a.emit('MUL', a=T1, b=T2)
a.emit('MRD', dst=T3, imm=12)
a.emit('LDI', dst=T2, imm=6308)
a.emit('ADD', dst=T3, a=T3, b=T2)
a.emit('MUL', a=T0, b=T3)
a.emit('MRD', dst=T4, imm=12)
a.emit('LDI', dst=T6, imm=255)
a.emit('SRD', dst=Z, a=T6, imm=0)
a.emit('CTL', dst=T5, imm=CTLR['zfirst'])
a.emit('JNZ', a=T5, imm='z_snap')
a.emit('SUB', dst=T1, a=T4, b=Z)
a.emit('ABS', dst=T2, a=T1)
a.emit('LDI', dst=T3, imm=12)
a.emit('JLT', a=T2, b=T3, imm='z_store')
a.emit('SHR', dst=T1, a=T1, imm=3)
a.emit('ADD', dst=Z, a=Z, b=T1)
a.emit('JMP', imm='z_store')
a.label('z_snap')
a.emit('MOV', dst=Z, a=T4)
a.label('z_store')
a.emit('SWR', a=T6, b=Z)
a.emit('CTL', dst=CX6, imm=CTLR['cx']); a.emit('SHL', dst=CX6, a=CX6, imm=6)
a.emit('CTL', dst=CY6, imm=CTLR['cy']); a.emit('SHL', dst=CY6, a=CY6, imm=6)
# AA slope for the pixel path as mantissa (s_pxm) and exponent (s_pxe):
# halve in a loop, counting -- the count is the exponent.
a.emit('LDI', dst=T1, imm=183)
a.emit('MUL', a=Z, b=T1)
a.emit('MRD', dst=T0, imm=12)
a.emit('AND', dst=T0, a=T0, imm=255)
a.emit('LDI', dst=T1, imm=0)
a.emit('MOV', dst=T3, a=T0)
a.emit('LDI', dst=T4, imm=8)
a.label('pxl')
a.emit('JLT', a=T3, b=T4, imm='pxd')
a.emit('SHR', dst=T3, a=T3, imm=1)
a.emit('ADD', dst=T1, a=T1, b=ONE)
a.emit('JMP', imm='pxl')
a.label('pxd')
# the pixel path uses mantissa 4 only: round a mantissa of 6+ up an octave
a.emit('LDI', dst=T4, imm=6)
a.emit('JLT', a=T3, b=T4, imm='px_rnd')
a.emit('ADD', dst=T1, a=T1, b=ONE)
a.label('px_rnd')
a.emit('LDI', dst=T4, imm=5)
a.emit('MIN', dst=T1, a=T1, b=T4)
a.emit('SUB', dst=T4, a=T4, b=T1)
a.emit('SLW', imm=SLOTW['pxe'], a=T4, b=ZERO)
# DDA line stepping: interlace walks frame lines two at a time, and the field
# with field_n = '0' starts on frame line 1
a.emit('CTL', dst=T0, imm=CTLR['ilace'])
a.emit('CTL', dst=T1, imm=CTLR['field'])
a.emit('ADD', dst=YST, a=ONE, b=T0)
a.emit('SUB', dst=T2, a=ONE, b=T1)
a.emit('MIN', dst=Y0, a=T0, b=T2)

# ---------------------------------------------------- 6. the two boxes
# BLOCK (box 1) straight off the basis: projected axes (Q6 px per cubie,
# screen y down), axis z (visibility), and H / key / fill dotted with each
# axis, so every face's value is just +/- one of these.
a.emit('LDI', dst=T7, imm=0)                      # 3i
a.emit('LDI', dst=T6, imm=BOX1)                   # BOX1 + i
a.label('bx1')
a.emit('LDX', dst=T8, a=T7, imm=B)
a.emit('LDX', dst=T9, a=T7, imm=B + 1)
a.emit('LDX', dst=T10, a=T7, imm=B + 2)
a.emit('MUL', a=T8, b=Z); a.emit('MRD', dst=T0, imm=0)
a.emit('SHR', dst=T0, a=T0, imm=10)
a.emit('STX', a=T6, b=T0, imm=PX)
a.emit('MUL', a=T9, b=Z); a.emit('MRD', dst=T0, imm=0)
a.emit('SHR', dst=T0, a=T0, imm=10)
a.emit('NEG', dst=T0, a=T0)
a.emit('STX', a=T6, b=T0, imm=PY)
a.emit('STX', a=T6, b=T10, imm=NZ)
for fld, coef in ((HD, (C_HX, C_HY, C_HZ)), (KD, KEY), (FD, FILL)):
    a.emit('LDI', dst=T2, imm=0)
    for comp in range(3):
        a.emit('LDI', dst=T1, imm=coef[comp])
        a.emit('MUL', a=T8 + comp, b=T1)
        a.emit('MRD', dst=T0, imm=12)
        a.emit('ADD', dst=T2, a=T2, b=T0)
    a.emit('STX', a=T6, b=T2, imm=fld)
a.emit('ADD', dst=T6, a=T6, b=ONE)
a.emit('LDI', dst=T0, imm=3)
a.emit('ADD', dst=T7, a=T7, b=T0)
a.emit('LDI', dst=T0, imm=9)
a.emit('JLT', a=T7, b=T0, imm='bx1')

# turn axis a and side from the turn face (face <-> normal code is x ^ 2
# for 0..3, identity for 4, 5)
a.emit('LDI', dst=T0, imm=4)
a.emit('MOV', dst=T1, a=TF)
a.emit('JGE', a=TF, b=T0, imm='ax_ok')
a.emit('LDI', dst=T0, imm=2)
a.emit('JLT', a=TF, b=T0, imm='ax_up')
a.emit('SUB', dst=T1, a=TF, b=T0)
a.emit('JMP', imm='ax_ok')
a.label('ax_up')
a.emit('ADD', dst=T1, a=TF, b=T0)
a.label('ax_ok')
a.emit('SHR', dst=AAX, a=T1, imm=1)
a.emit('AND', dst=ASIDE, a=T1, imm=1)             # 1 = negative side

# SLICE (box 0): the block's values, axes b and c turned by theta about a
a.emit('ADD', dst=T8, a=AAX, b=ONE)               # T8 = b, T9 = c
a.emit('LDI', dst=T0, imm=3)
a.emit('JLT', a=T8, b=T0, imm='bc1')
a.emit('LDI', dst=T8, imm=0)
a.label('bc1')
a.emit('ADD', dst=T9, a=T8, b=ONE)
a.emit('JLT', a=T9, b=T0, imm='bc2')
a.emit('LDI', dst=T9, imm=0)
a.label('bc2')
a.emit('LDI', dst=T10, imm=0)                     # field offset
a.label('sl_fld')
a.emit('ADD', dst=T4, a=T10, b=AAX)               # copy axis a
a.emit('LDX', dst=T0, a=T4, imm=BOX1)
a.emit('STX', a=T4, b=T0, imm=BOX0)
a.emit('ADD', dst=T5, a=T10, b=T8)                # xb, xc
a.emit('LDX', dst=T6, a=T5, imm=BOX1)
a.emit('ADD', dst=T7, a=T10, b=T9)
a.emit('LDX', dst=T11, a=T7, imm=BOX1)
a.emit('MUL', a=T6, b=C_TH); a.emit('MRD', dst=T0, imm=12)
a.emit('MUL', a=T11, b=S_TH); a.emit('MRD', dst=T1, imm=12)
a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('STX', a=T5, b=T0, imm=BOX0)
a.emit('MUL', a=T11, b=C_TH); a.emit('MRD', dst=T0, imm=12)
a.emit('MUL', a=T6, b=S_TH); a.emit('MRD', dst=T1, imm=12)
a.emit('SUB', dst=T0, a=T0, b=T1)
a.emit('STX', a=T7, b=T0, imm=BOX0)
a.emit('LDI', dst=T0, imm=3)
a.emit('ADD', dst=T10, a=T10, b=T0)
a.emit('LDI', dst=T0, imm=18)
a.emit('JLT', a=T10, b=T0, imm='sl_fld')

# half extents x projected axis (1.5 cubies, except along a: slice 0.5,
# block 1.0), and the box centres (slice at +/-1 along a, block at -/+0.5)
for bx in (BOX0, BOX1):
    for i in range(3):
        for src, dst in ((PX, HPX), (PY, HPY)):
            a.emit('SHR', dst=T0, a=bx + src + i, imm=1)
            a.emit('ADD', dst=bx + dst + i, a=bx + src + i, b=T0)
for comp, (src, dst, cen) in enumerate(((PX, HPX, CX6), (PY, HPY, CY6))):
    a.emit('LDX', dst=T0, a=AAX, imm=BOX1 + src)      # P along a
    a.emit('SHR', dst=T1, a=T0, imm=1)                # 0.5 P
    a.emit('LDI', dst=T3, imm=BOX0 + dst)
    a.emit('ADD', dst=T3, a=T3, b=AAX)
    a.emit('STX', a=T3, b=T1, imm=0)                  # slice: 0.5 P
    a.emit('LDI', dst=T3, imm=BOX1 + dst)
    a.emit('ADD', dst=T3, a=T3, b=AAX)
    a.emit('STX', a=T3, b=T0, imm=0)                  # block: 1.0 P
    negif(T4, T0, ASIDE)                              # +/- P
    a.emit('ADD', dst=BOX0 + (CBX if comp == 0 else CBY), a=cen, b=T4)
    negif(T4, T1, ASIDE)
    a.emit('SUB', dst=BOX1 + (CBX if comp == 0 else CBY), a=cen, b=T4)

# visibility: a face shows iff its normal points at the viewer
a.emit('LDI', dst=T7, imm=0)                      # face
a.emit('LDI', dst=T9, imm=VIS)                    # &VIS[face]
a.label('vis')
a.emit('MOV', dst=T0, a=T7)
call('FCODE')                                     # T1 = normal code
a.emit('SHR', dst=T2, a=T1, imm=1)                # axis
a.emit('AND', dst=T3, a=T1, imm=1)                # negative?
for bi, bx in enumerate((BOX0, BOX1)):
    a.emit('LDX', dst=T4, a=T2, imm=bx + NZ)
    negif(T4, T4, T3)
    # cull only faces under a pixel across: a culled face that still shows
    # would leave a hole between the two boxes
    a.emit('LDI', dst=T5, imm=8)
    a.emit('LDI', dst=T0, imm=0)
    lv = L('vs')
    a.emit('JGE', a=T5, b=T4, imm=lv)
    a.emit('LDI', dst=T0, imm=1)
    a.label(lv)
    a.emit('STX', a=T9, b=T0, imm=6 * bi)
a.emit('ADD', dst=T9, a=T9, b=ONE)
a.emit('ADD', dst=T7, a=T7, b=ONE)
a.emit('LDI', dst=T0, imm=6)
a.emit('JLT', a=T7, b=T0, imm='vis')
# the box on the viewer's side of the cut plane is drawn first
a.emit('LDX', dst=T0, a=AAX, imm=BOX1 + NZ)
negif(T1, T0, ASIDE)
a.emit('LDI', dst=FIRST, imm=0)
a.emit('JLT', a=ZERO, b=T1, imm='near_ok')
a.emit('LDI', dst=FIRST, imm=1)
a.label('near_ok')

# ---------------------------------------------------- 7. the face loop
IN, IU, IV = 35, 36, 37                           # box data index per axis
VB = K16M
NS, US, VS = Z, CX6, CY6                          # axis signs (1 = negative)
a.emit('LDI', dst=SLOT, imm=0)
# face-loop constants, in the basis registers (free by now)
a.emit('LDI', dst=44, imm=2)
a.emit('LDI', dst=45, imm=3)
a.emit('LDI', dst=46, imm=4)
a.emit('LDI', dst=47, imm=6)
a.emit('LDI', dst=48, imm=63)
a.emit('LDI', dst=49, imm=128)
a.emit('LDI', dst=50, imm=255)
a.emit('LDI', dst=51, imm=176)
a.emit('LDI', dst=52, imm=12)
a.emit('MOV', dst=BOX, a=FIRST)
a.label('pass')
a.emit('LDI', dst=BB, imm=BOX0)
a.emit('JNZ', a=BOX, imm='bb1')
a.emit('JMP', imm='bb_ok')
a.label('bb1')
a.emit('LDI', dst=BB, imm=BOX1)
a.label('bb_ok')
# VB = &VIS[6 box] (kept in K16M, which the face loop does not use)
a.emit('LDI', dst=VB, imm=VIS)
a.emit('JNZ', a=BOX, imm='vb1')
a.emit('JMP', imm='vb_ok')
a.label('vb1')
a.emit('LDI', dst=VB, imm=VIS + 6)
a.label('vb_ok')
a.emit('LDI', dst=FI, imm=0)
a.label('face')
a.emit('ADD', dst=T0, a=VB, b=FI)
a.emit('LDX', dst=T1, a=T0, imm=0)
a.emit('JNZ', a=T1, imm='f_vis')
a.emit('JMP', imm='f_next')
a.label('f_vis')
a.emit('MOV', dst=T0, a=FI)
call('FCODE')
for code, idx, sgn in ((T1, IN, NS), (T2, IU, US), (T3, IV, VS)):
    a.emit('SHR', dst=idx, a=code, imm=1)
    a.emit('ADD', dst=idx, a=idx, b=BB)
    a.emit('AND', dst=sgn, a=code, imm=1)


def ld(dst, idx, sgn, fld):
    """dst = +/- box[fld + axis]"""
    a.emit('LDX', dst=T1, a=idx, imm=fld)
    negif(dst, T1, sgn)


# screen axis vectors of the face (per cubie) and its origin corner
ld(SUX, IU, US, PX); ld(SUY, IU, US, PY)
ld(SVX, IV, VS, PX); ld(SVY, IV, VS, PY)
a.emit('LDX', dst=OX, a=BB, imm=CBX)
a.emit('LDX', dst=OY, a=BB, imm=CBY)
for idx, sgn, op in ((IN, NS, 'ADD'), (IU, US, 'SUB'), (IV, VS, 'SUB')):
    ld(T3, idx, sgn, HPX); a.emit(op, dst=OX, a=OX, b=T3)
    ld(T3, idx, sgn, HPY); a.emit(op, dst=OY, a=OY, b=T3)

# D = cross(SU, SV) in Q12; gradients (units Q6 per px) = 2^24 * P / D
a.emit('MUL', a=SUX, b=SVY); a.emit('MRD', dst=T0, imm=0)
a.emit('MUL', a=SUY, b=SVX); a.emit('MRD', dst=T1, imm=0)
a.emit('SUB', dst=T0, a=T0, b=T1)
a.emit('LDI', dst=DS, imm=0)
a.emit('JLT', a=ZERO, b=T0, imm='d_pos')
a.emit('LDI', dst=DS, imm=1)
a.emit('NEG', dst=T0, a=T0)
a.label('d_pos')
a.emit('SHR', dst=DA, a=T0, imm=8)
GUX, GUY, GVX, GVY = T8, T9, T10, T11
for src, neg, dst in ((SVY, False, GUX), (SVX, True, GUY),
                      (SUY, True, GVX), (SUX, False, GVY)):
    if neg: a.emit('NEG', dst=T0, a=src)
    else:   a.emit('MOV', dst=T0, a=src)
    call('DIVS')
    a.emit('MOV', dst=dst, a=T1)

# the face's two DDAs.  u(x, y) = gx*(x - ox) + gy*(y - oy), Q6 units, with
# the origin in Q4 (so the seed products fit 32 bits).  A gradient past 14
# bits (a face under ~48 px across) is block-floated by 6: the DDA then
# counts whole units with a mantissa of 8+ bits, and 27 bits cover the
# screen either way (DRD clamps gradients at 2^20: faces under ~0.75 px).
a.emit('SHR', dst=OX, a=OX, imm=2)
a.emit('SHR', dst=OY, a=OY, imm=2)
ldi(T7, 16383, T6)
for c, (gx, gy) in enumerate(((GUX, GUY), (GVX, GVY))):
    pe, pg, pw, ps = (('eu', 'gu', 'wu', 'su'), ('ev', 'gv', 'wv', 'sv'))[c]
    a.emit('ABS', dst=T0, a=gx)
    a.emit('ABS', dst=T1, a=gy)
    a.emit('MAX', dst=T0, a=T0, b=T1)
    lbf, lbd = L('bf'), L('bfd')
    a.emit('JLT', a=T7, b=T0, imm=lbf)
    a.emit('MOV', dst=T2, a=gx)
    a.emit('MOV', dst=T3, a=gy)
    a.emit('SLW', imm=SLOTW[pe], a=ZERO, b=SLOT)
    a.emit('JMP', imm=lbd)
    a.label(lbf)
    a.emit('LDI', dst=T4, imm=32)
    a.emit('ADD', dst=T2, a=gx, b=T4); a.emit('SHR', dst=T2, a=T2, imm=6)
    a.emit('ADD', dst=T3, a=gy, b=T4); a.emit('SHR', dst=T3, a=T3, imm=6)
    a.emit('SLW', imm=SLOTW[pe], a=ONE, b=SLOT)
    a.label(lbd)
    a.emit('SLW', imm=SLOTW[pg], a=T2, b=SLOT)
    # seed = (gx*(0 - ox4) + gy*(16 y0 - oy4)) >> 4
    a.emit('NEG', dst=T4, a=OX)
    a.emit('MUL', a=T2, b=T4); a.emit('MRD', dst=T5, imm=0)
    a.emit('SHL', dst=T4, a=Y0, imm=4)
    a.emit('SUB', dst=T4, a=T4, b=OY)
    a.emit('MUL', a=T3, b=T4); a.emit('MRD', dst=T6, imm=0)
    a.emit('ADD', dst=T5, a=T5, b=T6)
    a.emit('SHR', dst=T5, a=T5, imm=4)
    a.emit('SLW', imm=SLOTW[ps], a=T5, b=SLOT)
    # wrap = gy*ystep - gx*W
    a.emit('MUL', a=T3, b=YST); a.emit('MRD', dst=T6, imm=0)
    a.emit('CTL', dst=T4, imm=CTLR['W'])
    a.emit('MUL', a=T2, b=T4); a.emit('MRD', dst=T5, imm=0)
    a.emit('SUB', dst=T6, a=T6, b=T5)
    a.emit('SLW', imm=SLOTW[pw], a=T6, b=SLOT)

# ---- face descriptor: sticker base, shape, gap edge, silhouette edges
# T6 = TABLE_BASE + 12 turnface + 6 box: this box's six base entries
a.emit('MUL', a=TF, b=52)
a.emit('MRD', dst=T6, imm=0)
a.emit('MUL', a=BOX, b=47)
a.emit('MRD', dst=T0, imm=0)
a.emit('ADD', dst=T6, a=T6, b=T0)
a.emit('ADD', dst=T6, a=T6, b=51)                 # + TABLE_BASE
a.emit('ADD', dst=T0, a=T6, b=FI)
a.emit('SRD', dst=T8, a=T0, imm=0)                # T8 = base (63 = interior)
a.emit('LDI', dst=T9, imm=0)                      # T9 = silhouette bits
a.emit('LDI', dst=T10, imm=0)                     # T10 = gap code
a.emit('LDI', dst=T0, imm=tm.TABLE_ADJ)
a.emit('ADD', dst=T0, a=T0, b=FI)
a.emit('SRD', dst=T11, a=T0, imm=0)               # T11 = the four neighbours
for e in range(4):
    # neighbour face across edge e (-U +U -V +V)
    if e: a.emit('SHR', dst=T0, a=T11, imm=3 * e)
    else: a.emit('MOV', dst=T0, a=T11)
    a.emit('AND', dst=T0, a=T0, imm=7)
    # an exterior face next to an interior one: a cut edge, drawn as a gap
    a.emit('ADD', dst=T2, a=T6, b=T0)
    a.emit('SRD', dst=T3, a=T2, imm=0)            # neighbour base
    lsil, lgap, lnx = L('es'), L('eg'), L('en')
    a.emit('SUB', dst=T5, a=T3, b=48)             # (48 holds 63)
    a.emit('JNZ', a=T5, imm=lsil)
    a.emit('SUB', dst=T5, a=T8, b=48)
    a.emit('JNZ', a=T5, imm=lgap)
    a.label(lsil)                                 # else silhouette iff hidden
    a.emit('ADD', dst=T2, a=VB, b=T0)
    a.emit('LDX', dst=T3, a=T2, imm=0)
    a.emit('JNZ', a=T3, imm=lnx)
    a.emit('LDI', dst=T3, imm=1 << e)
    a.emit('ADD', dst=T9, a=T9, b=T3)
    a.emit('JMP', imm=lnx)
    a.label(lgap)
    a.emit('LDI', dst=T10, imm=e + 1)
    a.label(lnx)


def ext(dst, idx):
    """dst = box extent along the axis of idx: 3 cubies, except along the
    turn axis (slice 1, block 2)."""
    l2 = L('ex')
    a.emit('SUB', dst=T0, a=idx, b=BB)
    a.emit('LDI', dst=dst, imm=3)
    a.emit('SUB', dst=T0, a=T0, b=AAX)
    a.emit('JNZ', a=T0, imm=l2)
    a.emit('ADD', dst=dst, a=ONE, b=BOX)
    a.label(l2)


# shape code: (nu, nv) = (3,3) 0, (3,1) 1, (1,3) 2, (3,2) 3, (2,3) 4
ext(T4, IU)
ext(T5, IV)
a.emit('LDI', dst=T0, imm=0)
a.emit('ADD', dst=T1, a=T4, b=T5)
a.emit('JGE', a=T1, b=47, imm='sh_done')
for code, reg, val in ((1, T5, 1), (2, T4, 1), (3, T5, 2)):
    a.emit('LDI', dst=T0, imm=code)
    a.emit('LDI', dst=T2, imm=val)
    a.emit('SUB', dst=T1, a=reg, b=T2)
    lnx = L('sh')
    a.emit('JNZ', a=T1, imm=lnx)
    a.emit('JMP', imm='sh_done')
    a.label(lnx)
a.emit('LDI', dst=T0, imm=4)
a.label('sh_done')
# fd = base << 10 | shape << 7 | gap << 4 | sil
a.emit('SHL', dst=T1, a=T8, imm=10)
a.emit('SHL', dst=T0, a=T0, imm=7)
a.emit('ADD', dst=T1, a=T1, b=T0)
a.emit('SHL', dst=T0, a=T10, imm=4)
a.emit('ADD', dst=T1, a=T1, b=T0)
a.emit('ADD', dst=T1, a=T1, b=T9)
a.emit('SLW', imm=SLOTW['fd'], a=T1, b=SLOT)

# ---- half-vector projections: h1 = hu << 6 | hn[5:0], h2 = hv << 6 | hn[9:6]
ld(T9, IN, NS, HD)
ld(T10, IU, US, HD)
ld(T11, IV, VS, HD)
a.emit('AND', dst=T0, a=T10, imm=0x3FF); a.emit('SHL', dst=T0, a=T0, imm=6)
a.emit('AND', dst=T1, a=T9, imm=0x3F); a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('SLW', imm=SLOTW['h1'], a=T0, b=SLOT)
a.emit('AND', dst=T0, a=T11, imm=0x3FF); a.emit('SHL', dst=T0, a=T0, imm=6)
a.emit('SHR', dst=T1, a=T9, imm=6); a.emit('AND', dst=T1, a=T1, imm=0xF)
a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('SLW', imm=SLOTW['h2'], a=T0, b=SLOT)

# ---- flat light: ambient + 3/4 key + 1/4 fill against the face normal
ld(T9, IN, NS, KD)
ld(T10, IN, NS, FD)
a.emit('LDI', dst=T3, imm=C_AMB)
a.emit('JGE', a=ZERO, b=T9, imm='lt_k')
a.emit('SHR', dst=T1, a=T9, imm=2)
a.emit('SUB', dst=T2, a=T9, b=T1)
a.emit('ADD', dst=T3, a=T3, b=T2)
a.label('lt_k')
a.emit('JGE', a=ZERO, b=T10, imm='lt_f')
a.emit('SHR', dst=T1, a=T10, imm=2)
a.emit('ADD', dst=T3, a=T3, b=T1)
a.label('lt_f')
a.emit('MIN', dst=T3, a=T3, b=50)
# lf word = lf (sticker chroma is full-saturation since v0.4, so the old
# gamma-level byte, and with it the engine's GAM op, is gone)
a.emit('SLW', imm=SLOTW['lf'], a=T3, b=SLOT)

a.emit('ADD', dst=SLOT, a=SLOT, b=ONE)
a.label('f_next')
a.emit('ADD', dst=FI, a=FI, b=ONE)
a.emit('JLT', a=FI, b=47, imm='face')
a.emit('SUB', dst=BOX, a=ONE, b=BOX)              # the other box ...
a.emit('SUB', dst=T0, a=BOX, b=FIRST)             # ... unless back at FIRST
a.emit('JNZ', a=T0, imm='pass')

# ---------------------------------------------------- 8. publish
a.emit('SLW', imm=SLOTW['nslot'], a=SLOT, b=ZERO)
a.emit('SLW', imm=SLOTW['kick'], a=ZERO, b=ZERO)
a.emit('END')

if __name__ == '__main__':
    w = a.words()
    print(f'-- {len(w)} instructions', file=sys.stderr)
    assert len(w) <= 1024
    print(emit_vhdl(w, depth=1024))
