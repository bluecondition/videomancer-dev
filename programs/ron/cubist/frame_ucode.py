#!/usr/bin/env python3
"""CUBIST frame-setup microcode.

Translation of the hand-written frame FSM (~110 states) onto the engine in
ueng.vhd.  The arithmetic is identical; what disappears is ~110 private
datapaths and the 18-bit operand mux feeding the shared multiplier.

Register map (rf, 128 x signed 16):
   0.. 8  basis B(axis,component), Q12
   9..11  H projected on each cube axis, Q8
  12..17  half-basis sxh/syh, screen Q2
  18..25  corner x        26..33  corner y
  34..39  cs_yaw_s, cs_yaw_c, cs_pit_s, cs_pit_c, cs_rol_s, cs_rol_c
  40..42  glide-smoothed yaw / pitch / roll, 12-bit angle
  43      loop counter j        44  loop counter i        45  loop counter c
  46..59  scratch
"""
import sys
sys.path.insert(0, '.')
from uasm import Asm, emit_vhdl, SLOTW

R_BAS, R_H, R_HB, R_CX, R_CY = 0, 9, 12, 18, 26
R_TRIG, R_ANG = 34, 40
J, I, C = 43, 44, 45
T0, T1, T2, T3, T4, T5 = 46, 47, 48, 49, 50, 51
ANG, SK, SNEG, S0, S1, KK = 52, 53, 54, 55, 56, 57
ZERO, ONE = 58, 59
EX, EY, FY, ZOOM, ZSM, FUSE, CXR, CYR = 60, 61, 62, 63, 64, 65, 66, 67
R_VIS = 68          # 6 visibility flags
R_TBL = 74          # packed face tables (corner/adj/axis), LDX by face
R_SLOT, R_FACE = 92, 93

a = Asm()

# ---------------------------------------------------------------- constants
a.emit('LDI', dst=ZERO, imm=0)
a.emit('LDI', dst=ONE,  imm=1)

# -------------------------------------------------- phase 1: angle glide
# an += clamp_to_at_least_one_step((knob<<2) - an) >> 4, so the cube eases
# toward the knob instead of snapping.  s_zfirst forces a snap on the first
# valid frame (see the autozoom note: raster measurements are only good from
# frame 3, so everything that latches them must survive a mid-flight change).
for k, (ctl, reg) in enumerate(((0, R_ANG), (1, R_ANG + 1), (2, R_ANG + 2))):
    a.emit('CTL', dst=T0, imm=ctl)          # knob, 10 bits
    a.emit('SHL', dst=T0, a=T0, imm=2)      # -> 12-bit angle
    a.emit('SUB', dst=T1, a=T0, b=reg)      # delta
    a.emit('SHR', dst=T2, a=T1, imm=4)      # one sixteenth of the way
    # if the step rounded to zero but there is still a gap, move one tick
    a.emit('JNZ', a=T2, imm=f'glide{k}')
    a.emit('JLT', a=T1, b=ZERO, imm=f'gneg{k}')
    a.emit('JLT', a=ZERO, b=T1, imm=f'gpos{k}')
    a.emit('JMP', imm=f'glide{k}')
    a.label(f'gpos{k}'); a.emit('MOV', dst=T2, a=ONE); a.emit('JMP', imm=f'glide{k}')
    a.label(f'gneg{k}'); a.emit('NEG', dst=T2, a=ONE)
    a.label(f'glide{k}')
    a.emit('ADD', dst=reg, a=reg, b=T2)
    a.emit('AND', dst=reg, a=reg, imm=0xFFF)
    # first valid frame: snap
    a.emit('CTL', dst=T3, imm=10)           # zfirst
    a.emit('JNZ', a=T3, imm=f'snap{k}')
    a.emit('JMP', imm=f'done{k}')
    a.label(f'snap{k}'); a.emit('MOV', dst=reg, a=T0)
    a.label(f'done{k}')

# ------------------------------------------- phase 2: six sine lookups
# One ROM port, folded quarter table, linear interpolation between entries.
# j = 0..5 selects (yaw,pitch,roll) x (sin,cos); cos is sin(a + 90 deg).
a.emit('LDI', dst=J, imm=0)
a.label('trig')
a.emit('SHR', dst=T0, a=J, imm=1)                 # which angle
a.emit('LDI', dst=ANG, imm=0)
a.emit('JNZ', a=T0, imm='tr_np')
a.emit('MOV', dst=ANG, a=R_ANG); a.emit('JMP', imm='tr_have')
a.label('tr_np')
a.emit('SUB', dst=T1, a=T0, b=ONE)
a.emit('JNZ', a=T1, imm='tr_r')
a.emit('MOV', dst=ANG, a=R_ANG + 1); a.emit('JMP', imm='tr_have')
a.label('tr_r'); a.emit('MOV', dst=ANG, a=R_ANG + 2)
a.label('tr_have')
a.emit('BIT', dst=T2, a=J, imm=0)                 # odd j -> cosine
a.emit('JNZ', a=T2, imm='tr_cos')
a.emit('JMP', imm='tr_fold')
a.label('tr_cos')
a.emit('LDI', dst=T3, imm=1024)
a.emit('ADD', dst=ANG, a=ANG, b=T3)
a.emit('AND', dst=ANG, a=ANG, imm=0xFFF)
a.label('tr_fold')
# index = ang(9:4), mirrored in the upper half of each half-cycle
a.emit('SHR', dst=T0, a=ANG, imm=4)
a.emit('AND', dst=T0, a=T0, imm=0x3F)
a.emit('BIT', dst=T1, a=ANG, imm=10)
a.emit('BIT', dst=SNEG, a=ANG, imm=11)
a.emit('JNZ', a=T1, imm='tr_mir')
a.emit('MOV', dst=SK, a=T0)
a.emit('ADD', dst=KK, a=T0, b=ONE)
a.emit('JMP', imm='tr_rd')
a.label('tr_mir')
a.emit('LDI', dst=T2, imm=64)
a.emit('SUB', dst=SK, a=T2, b=T0)
a.emit('SUB', dst=KK, a=SK, b=ONE)
a.label('tr_rd')
a.emit('SIN', dst=S0, a=SK)
a.emit('SIN', dst=S1, a=KK)
a.emit('JNZ', a=SNEG, imm='tr_neg')
a.emit('JMP', imm='tr_int')
a.label('tr_neg')
a.emit('NEG', dst=S0, a=S0)
a.emit('NEG', dst=S1, a=S1)
a.label('tr_int')
# interpolate the low 4 angle bits: s0 + (s1-s0)*frac/16
a.emit('SUB', dst=T0, a=S1, b=S0)
a.emit('AND', dst=T1, a=ANG, imm=0xF)
a.emit('MUL', a=T0, b=T1)
a.emit('MRD', dst=T2, imm=4)
a.emit('ADD', dst=T3, a=S0, b=T2)
# store to cs_* by j
for n in range(6):
    a.emit('LDI', dst=T4, imm=n)
    a.emit('SUB', dst=T5, a=J, b=T4)
    a.emit('JNZ', a=T5, imm=f'tr_s{n}')
    a.emit('MOV', dst=R_TRIG + n, a=T3)
    a.label(f'tr_s{n}')
a.emit('ADD', dst=J, a=J, b=ONE)
a.emit('LDI', dst=T4, imm=6)
a.emit('JLT', a=J, b=T4, imm='trig')


# ------------------------------------------ phase 3: basis = identity
for c in range(9):
    a.emit('LDI', dst=R_BAS + c, imm=4096 if c in (0, 4, 8) else 0)

# ------------------------------- phase 4: basis rotation, UNROLLED
# (a,b) := (a*c - b*s, a*s + b*c) for each basis vector about each axis.
# Unrolled rather than looped: the loop body needs B[3i+p] -- a computed
# register address -- and indexed addressing would cost an extra pipeline
# state in the engine.  Microcode ROM is block RAM, which is the resource
# this design has spare.
AXES = ((0, 1, R_TRIG + 5, R_TRIG + 4),    # roll  : c = cs_rol_c, s = cs_rol_s
        (1, 2, R_TRIG + 3, R_TRIG + 2),    # pitch
        (2, 0, R_TRIG + 1, R_TRIG + 0))    # yaw
for (p0, p1, rc, rs) in AXES:
    for i in range(3):
        ra, rb = R_BAS + 3 * i + p0, R_BAS + 3 * i + p1
        a.emit('MUL', a=ra, b=rc); a.emit('MRD', dst=T0, imm=12)   # a*c
        a.emit('MUL', a=rb, b=rs); a.emit('MRD', dst=T1, imm=12)   # b*s
        a.emit('MUL', a=ra, b=rs); a.emit('MRD', dst=T2, imm=12)   # a*s
        a.emit('MUL', a=rb, b=rc); a.emit('MRD', dst=T3, imm=12)   # b*c
        a.emit('SUB', dst=ra, a=T0, b=T1)
        a.emit('ADD', dst=rb, a=T2, b=T3)


# --------------------------- phase 5: orthographic extent + autozoom
# The projected half-extent on each screen axis is just the sum of the three
# basis vectors' magnitudes on that axis -- no corner projection needed.
a.emit('ABS', dst=T0, a=R_BAS + 0)
a.emit('ABS', dst=T1, a=R_BAS + 3); a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('ABS', dst=T1, a=R_BAS + 6); a.emit('ADD', dst=T0, a=T0, b=T1)
a.emit('ABS', dst=T2, a=R_BAS + 1)
a.emit('ABS', dst=T1, a=R_BAS + 4); a.emit('ADD', dst=T2, a=T2, b=T1)
a.emit('ABS', dst=T1, a=R_BAS + 7); a.emit('ADD', dst=T2, a=T2, b=T1)
a.emit('SHR', dst=T1, a=T0, imm=1); a.emit('ADD', dst=EX, a=T0, b=T1)  # x1.5
a.emit('SHR', dst=T1, a=T2, imm=1); a.emit('ADD', dst=EY, a=T2, b=T1)

# f (px per cubie unit, Q4) = 0.42*H*65536/ext_y, 0.42*65536 taken as 27<<10
a.emit('CTL', dst=T0, imm=5)                     # s_hf
a.emit('SHL', dst=T1, a=T0, imm=5)
a.emit('SHL', dst=T2, a=T0, imm=2)
a.emit('SUB', dst=T1, a=T1, b=T2)
a.emit('SUB', dst=T1, a=T1, b=T0)                # 27*hf
a.emit('DIV', a=T1, b=EY, imm=10)
a.emit('DRD', dst=T3)
a.emit('LDI', dst=T4, imm=4095)
a.emit('CLP', dst=FY, a=T3, b=T4)
a.emit('CTL', dst=T0, imm=3)                     # s_W
a.emit('SHL', dst=T1, a=T0, imm=5)
a.emit('SHL', dst=T2, a=T0, imm=1)
a.emit('SUB', dst=T1, a=T1, b=T2)                # 30*W
a.emit('DIV', a=T1, b=EX, imm=10)
a.emit('DRD', dst=T3)
a.emit('CLP', dst=T3, a=T3, b=T4)
a.emit('MIN', dst=ZOOM, a=T3, b=FY)

# smoothing: snap when the target moves a long way (startup, or a resolution
# change -- the raster measurement is not valid for the first frames and a
# 1/16 glide would take ~30 frames to walk off a bad initial value)
a.emit('SHL', dst=T0, a=ZOOM, imm=4)
a.emit('SUB', dst=T1, a=T0, b=ZSM)
a.emit('ABS', dst=T2, a=T1)
a.emit('SHR', dst=T3, a=ZSM, imm=3)
a.emit('JLT', a=T3, b=T2, imm='az_snap')
a.emit('SHR', dst=T1, a=T1, imm=4)
a.emit('ADD', dst=ZSM, a=ZSM, b=T1)
a.emit('JMP', imm='az_done')
a.label('az_snap'); a.emit('MOV', dst=ZSM, a=T0)
a.label('az_done')
a.emit('SHR', dst=FUSE, a=ZSM, imm=4)

# ------------------------ phase 6: scaled half-basis (screen Q2)
# sxh(i) = 1.5 * f * B(3i).x, syh(i) likewise on y
for i in range(3):
    for (comp, out) in ((0, R_HB + i), (1, R_HB + 3 + i)):
        a.emit('MUL', a=R_BAS + 3 * i + comp, b=FUSE)
        a.emit('MRD', dst=T0, imm=0)
        a.emit('SHR', dst=T1, a=T0, imm=1)
        a.emit('ADD', dst=T0, a=T0, b=T1)
        a.emit('SHR', dst=out, a=T0, imm=14)

# ------------------- phase 7: the 8 screen corners are sign sums
# Each corner is a sign combination of the three half-basis vectors: adds
# only, no per-corner multiply.
a.emit('CTL', dst=CXR, imm=6); a.emit('SHL', dst=CXR, a=CXR, imm=2)
a.emit('CTL', dst=CYR, imm=7); a.emit('SHL', dst=CYR, a=CYR, imm=2)
for k in range(8):
    for (base, out, sub) in ((R_HB, R_CX + k, False), (R_HB + 3, R_CY + k, True)):
        first = True
        for bit in range(3):
            op = 'ADD' if (k >> bit) & 1 else 'SUB'
            if first:
                if (k >> bit) & 1: a.emit('MOV', dst=T0, a=base + bit)
                else:              a.emit('NEG', dst=T0, a=base + bit)
                first = False
            else:
                a.emit(op, dst=T0, a=T0, b=base + bit)
        if sub: a.emit('SUB', dst=out, a=CYR, b=T0)
        else:   a.emit('ADD', dst=out, a=CXR, b=T0)


# ---------------------------- phase 8: visibility + face table in the RF
# Orthographic, so a face shows iff its normal's z is positive: one compare
# per face, no screen geometry.  Face normals are +/- basis rows 2,5,8.
a.emit('LDI', dst=T4, imm=64)
a.emit('NEG', dst=T5, a=T4)
# bz0/bz1/bz2 are the Z components of the three rotated axes: basis words
# 2, 5 and 8.  Faces 0/1 test axis 1, faces 2/3 axis 0, faces 4/5 axis 2.
for f, (reg, pos) in enumerate(((R_BAS + 5, True), (R_BAS + 5, False),
                                (R_BAS + 2, True), (R_BAS + 2, False),
                                (R_BAS + 8, True), (R_BAS + 8, False))):
    a.emit('LDI', dst=R_VIS + f, imm=0)
    if pos: a.emit('JGE', a=T4, b=reg, imm=f'nv{f}')     # skip if 64 >= bz
    else:   a.emit('JGE', a=reg, b=T5, imm=f'nv{f}')     # skip if bz >= -64
    a.emit('LDI', dst=R_VIS + f, imm=1)
    a.label(f'nv{f}')

# The per-face loop reads its constants from the register file rather than
# from six unrolled copies.  Each table entry is 3 bits, so all four corner
# indices for a face pack into one word: 18 registers instead of 66, which
# is what keeps the file inside 128 words.  Within the loop body the corner
# slot j is a compile-time constant, so unpacking is SHR by an immediate.
C_FCORN = [[2,6,7,3],[0,1,5,4],[1,3,7,5],[0,4,6,2],[4,5,7,6],[1,0,2,3]]
C_FADJ  = [[5,4,3,2],[3,2,5,4],[1,0,5,4],[5,4,1,0],[3,2,1,0],[1,0,3,2]]
C_FN, C_FU, C_FV = [2,3,0,1,4,5], [4,0,2,4,0,2], [0,4,4,2,2,0]
for f in range(6):
    a.emit('LDI', dst=R_TBL + f,      imm=sum(C_FCORN[f][j] << (3*j) for j in range(4)))
    a.emit('LDI', dst=R_TBL + 6 + f,  imm=sum(C_FADJ[f][j]  << (3*j) for j in range(4)))
    a.emit('LDI', dst=R_TBL + 12 + f, imm=C_FN[f] | (C_FU[f] << 3) | (C_FV[f] << 6))


# ============================ phase 9: the per-visible-face loop ===========
# States 58-97 of the old FSM.  The face index is a runtime value, so every
# per-face constant comes out of the register file by LDX; the four-corner
# and four-edge inner loops are unrolled instead, because their shift amounts
# would otherwise have to be runtime values and SHR takes an immediate.
F_PX0, F_PY0, F_DX1, F_DX2, F_DY1, F_DY2, F_DET = 94, 95, 96, 97, 98, 99, 100
F_CORN, F_ADJ, F_AX = 101, 102, 103
F_YMIN, F_YMAX, F_LACC, F_AC1, F_LF, F_GBASE = 104, 105, 106, 107, 108, 109
BS = 110                                   # 110..112: signed face-normal basis
T6, T7, T8, T9 = 113, 114, 115, 116
R_COL = 117                                # 117..122: packed sticker colour
K32000, K65535 = 123, 124

C_STY = [995, 807, 332, 513, 416, 276]
C_STU = [512,  91, 460, 244, 496, 757]
C_STV = [512, 654, 811, 848, 238, 330]

# Constants too wide for a 14-bit immediate, built once.
a.label('p9')
a.emit('LDI', dst=K32000, imm=4000); a.emit('SHL', dst=K32000, a=K32000, imm=3)
# (the shift field is four bits, so 65536 is two steps)
a.emit('LDI', dst=T0, imm=1); a.emit('SHL', dst=T0, a=T0, imm=15)
a.emit('ADD', dst=T0, a=T0, b=T0)
a.emit('SUB', dst=K65535, a=T0, b=ONE)

# sticker colour packed three 10-bit fields to a word: 6 registers, not 18
for f in range(6):
    # built ten bits at a time: a 14-bit immediate is sign-extended, so every
    # chunk has to stay under 8192, and the shift field is only four bits.
    a.emit('LDI', dst=T0, imm=C_STV[f])
    a.emit('SHL', dst=T0, a=T0, imm=10)
    a.emit('LDI', dst=T1, imm=C_STU[f]); a.emit('ADD', dst=T0, a=T0, b=T1)
    a.emit('SHL', dst=T0, a=T0, imm=10)
    a.emit('LDI', dst=T1, imm=C_STY[f]); a.emit('ADD', dst=R_COL + f, a=T0, b=T1)

a.emit('LDI', dst=R_SLOT, imm=0)
a.emit('LDI', dst=I, imm=0)
a.label('face_top')
a.emit('LDX', dst=T0, a=I, imm=R_VIS)
a.emit('JNZ', a=T0, imm='f_vis')
a.emit('JMP', imm='f_next')
a.label('f_vis')
a.emit('LDI', dst=T1, imm=3)
a.emit('JGE', a=R_SLOT, b=T1, imm='f_end')       # the slot table holds three

a.emit('LDX', dst=F_CORN, a=I, imm=R_TBL)
a.emit('LDX', dst=F_ADJ,  a=I, imm=R_TBL + 6)
a.emit('LDX', dst=F_AX,   a=I, imm=R_TBL + 12)


def corner_idx(dst, slot):
    """dst = the packed corner table's 3-bit field for this face vertex."""
    if slot == 0:
        a.emit('AND', dst=dst, a=F_CORN, imm=7)
    else:
        a.emit('SHR', dst=dst, a=F_CORN, imm=3 * slot)
        a.emit('AND', dst=dst, a=dst, imm=7)


# --- screen edge vectors from P0, P1, P3 (the two spanning edges)
corner_idx(T0, 0)
a.emit('LDX', dst=F_PX0, a=T0, imm=R_CX)
a.emit('LDX', dst=F_PY0, a=T0, imm=R_CY)
for slot, (dx, dy) in ((1, (F_DX1, F_DY1)), (3, (F_DX2, F_DY2))):
    corner_idx(T1, slot)
    a.emit('LDX', dst=T2, a=T1, imm=R_CX)
    a.emit('SUB', dst=T2, a=T2, b=F_PX0); a.emit('SHR', dst=dx, a=T2, imm=2)
    a.emit('LDX', dst=T2, a=T1, imm=R_CY)
    a.emit('SUB', dst=T2, a=T2, b=F_PY0); a.emit('SHR', dst=dy, a=T2, imm=2)

# --- D = dx1*dy2 - dy1*dx2, the signed screen area
a.emit('MUL', a=F_DX1, b=F_DY2); a.emit('MRD', dst=F_DET, imm=0)
a.emit('MUL', a=F_DY1, b=F_DX2); a.emit('MRD', dst=T0, imm=0)
a.emit('SUB', dst=F_DET, a=F_DET, b=T0)

# --- the four UV gradients.  The FSM negates the numerator when D < 0 and
# divides by |D|; the divider already takes its sign from both operands, so
# passing the signed numerator and signed D gives the same result in one step.
for src, neg, port in ((F_DY2, False, 'gux'), (F_DX2, True, 'guy'),
                       (F_DY1, True, 'gvx'), (F_DX1, False, 'gvy')):
    if neg: a.emit('NEG', dst=T0, a=src)
    else:   a.emit('MOV', dst=T0, a=src)
    a.emit('ADD', dst=T1, a=T0, b=T0)
    a.emit('ADD', dst=T1, a=T1, b=T0)            # 3*gn, then << 20 in the DIV
    a.emit('DIV', a=T1, b=F_DET, imm=20)
    a.emit('DRD', dst=T4)
    a.emit('MIN', dst=T4, a=T4, b=K65535)
    a.emit('NEG', dst=T5, a=K65535)
    a.emit('MAX', dst=T4, a=T4, b=T5)
    a.emit('SLW', imm=SLOTW[port], a=T4, b=R_SLOT)
a.emit('SHR', dst=T0, a=F_PX0, imm=2); a.emit('SLW', imm=SLOTW['px0'], a=T0, b=R_SLOT)
a.emit('SHR', dst=T0, a=F_PY0, imm=2); a.emit('SLW', imm=SLOTW['py0'], a=T0, b=R_SLOT)

# --- ymin / ymax over the four corners, seeded from corner 0
corner_idx(T0, 0)
a.emit('LDX', dst=F_YMIN, a=T0, imm=R_CY)
a.emit('MOV', dst=F_YMAX, a=F_YMIN)
for slot in (1, 2, 3):
    corner_idx(T0, slot)
    a.emit('LDX', dst=T1, a=T0, imm=R_CY)
    a.emit('MIN', dst=F_YMIN, a=F_YMIN, b=T1)
    a.emit('MAX', dst=F_YMAX, a=F_YMAX, b=T1)
a.emit('SHL', dst=F_GBASE, a=R_SLOT, imm=5)
a.emit('SHR', dst=T0, a=F_YMIN, imm=2)
a.emit('MAX', dst=T0, a=T0, b=ZERO)
a.emit('GWR', imm=0, a=F_GBASE, b=T0)
a.emit('SHR', dst=T0, a=F_YMAX, imm=2)
a.emit('ADD', dst=T0, a=T0, b=ONE)
a.emit('MAX', dst=T0, a=T0, b=ZERO)
a.emit('GWR', imm=1, a=F_GBASE, b=T0)

# --- the four screen edges: y0, x0 and dx/dy, written straight to the
# geometry RAM the line engine reads.
for c in range(4):
    n = (c + 1) % 4
    corner_idx(T0, c)
    a.emit('LDX', dst=T2, a=T0, imm=R_CX)        # sxa
    a.emit('LDX', dst=T3, a=T0, imm=R_CY)        # sya
    corner_idx(T1, n)
    a.emit('LDX', dst=T4, a=T1, imm=R_CX)
    a.emit('LDX', dst=T5, a=T1, imm=R_CY)
    a.emit('SUB', dst=T6, a=T4, b=T2)            # dx along the edge
    a.emit('SUB', dst=T7, a=T5, b=T3)            # dy along the edge
    a.emit('GWR', imm=2 + 3 * c, a=F_GBASE, b=T3)
    a.emit('GWR', imm=3 + 3 * c, a=F_GBASE, b=T2)
    # a near-horizontal edge would divide by ~0; floor the denominator at 2
    # but keep its sign, because the slope's sign comes from both operands.
    a.emit('ABS', dst=T8, a=T7)
    a.emit('LDI', dst=T9, imm=2)
    a.emit('JGE', a=T8, b=T9, imm=f'ed{c}')
    a.emit('JLT', a=T7, b=ZERO, imm=f'edn{c}')
    a.emit('MOV', dst=T7, a=T9); a.emit('JMP', imm=f'ed{c}')
    a.label(f'edn{c}'); a.emit('NEG', dst=T7, a=T9)
    a.label(f'ed{c}')
    a.emit('DIV', a=T6, b=T7, imm=6)
    a.emit('DRD', dst=T8)
    a.emit('MIN', dst=T8, a=T8, b=K32000)
    a.emit('NEG', dst=T9, a=K32000)
    a.emit('MAX', dst=T8, a=T8, b=T9)
    a.emit('GWR', imm=4 + 3 * c, a=F_GBASE, b=T8)

# --- flat face light: key and fill dotted against the face normal, which is
# +/- one basis row, so it is three multiplies each and no normalisation.
a.emit('AND', dst=T0, a=F_AX, imm=7)             # C_FN for this face
a.emit('SHR', dst=T1, a=T0, imm=1)
a.emit('ADD', dst=T2, a=T1, b=T1); a.emit('ADD', dst=T2, a=T2, b=T1)
a.emit('BIT', dst=T3, a=T0, imm=0)               # odd -> negated axis
for p in range(3):
    a.emit('LDI', dst=T4, imm=p)
    a.emit('ADD', dst=T4, a=T2, b=T4)
    a.emit('LDX', dst=T5, a=T4, imm=R_BAS)
    a.emit('JNZ', a=T3, imm=f'bn{p}')
    a.emit('JMP', imm=f'bs{p}')
    a.label(f'bn{p}'); a.emit('NEG', dst=T5, a=T5)
    a.label(f'bs{p}'); a.emit('MOV', dst=BS + p, a=T5)
a.emit('LDI', dst=F_LACC, imm=0)
a.emit('LDI', dst=F_AC1, imm=0)
for c, coef in enumerate((-107, 154, 184, 141, -141, 161)):
    a.emit('LDI', dst=T5, imm=coef)
    a.emit('MUL', a=BS + (c % 3), b=T5)
    a.emit('MRD', dst=T0, imm=12)
    acc = F_LACC if c < 3 else F_AC1
    a.emit('ADD', dst=acc, a=acc, b=T0)
a.emit('LDI', dst=T0, imm=62)                    # ambient floor
a.emit('JGE', a=ZERO, b=F_LACC, imm='nokey')
a.emit('SHR', dst=T1, a=F_LACC, imm=2)
a.emit('SUB', dst=T2, a=F_LACC, b=T1)
a.emit('ADD', dst=T0, a=T0, b=T2)
a.label('nokey')
a.emit('JGE', a=ZERO, b=F_AC1, imm='nofill')
a.emit('SHR', dst=T1, a=F_AC1, imm=2)
a.emit('ADD', dst=T0, a=T0, b=T1)
a.label('nofill')
a.emit('LDI', dst=T1, imm=255)
a.emit('MIN', dst=F_LF, a=T0, b=T1)

# --- the per-slot descriptor the line and pixel engines read
a.emit('SLW', imm=SLOTW['face'], a=I, b=R_SLOT)
a.emit('LDI', dst=T0, imm=0)
for j in range(4):
    if j == 0: a.emit('AND', dst=T1, a=F_ADJ, imm=7)
    else:
        a.emit('SHR', dst=T1, a=F_ADJ, imm=3 * j)
        a.emit('AND', dst=T1, a=T1, imm=7)
    a.emit('LDX', dst=T2, a=T1, imm=R_VIS)
    a.emit('JNZ', a=T2, imm=f'sil{j}')           # neighbour visible -> no rim
    a.emit('LDI', dst=T3, imm=1 << j)
    a.emit('ADD', dst=T0, a=T0, b=T3)
    a.label(f'sil{j}')
a.emit('SLW', imm=SLOTW['sil'], a=T0, b=R_SLOT)
# the three half-vector projections, picked and signed by the face's axes
for shift, port in ((0, 'hn'), (3, 'hu'), (6, 'hv')):
    if shift == 0: a.emit('AND', dst=T0, a=F_AX, imm=7)
    else:
        a.emit('SHR', dst=T0, a=F_AX, imm=shift)
        a.emit('AND', dst=T0, a=T0, imm=7)
    a.emit('SHR', dst=T1, a=T0, imm=1)
    a.emit('LDX', dst=T2, a=T1, imm=R_H)
    a.emit('BIT', dst=T3, a=T0, imm=0)
    a.emit('JNZ', a=T3, imm=f'hn{shift}')
    a.emit('JMP', imm=f'hs{shift}')
    a.label(f'hn{shift}'); a.emit('NEG', dst=T2, a=T2)
    a.label(f'hs{shift}')
    a.emit('SLW', imm=SLOTW[port], a=T2, b=R_SLOT)

# --- unlit albedo, the flat light, and the chroma scaled by that light so a
# shadowed face desaturates by exactly as much as it dims.
a.emit('LDX', dst=T9, a=I, imm=R_COL)
a.emit('AND', dst=T0, a=T9, imm=0x3FF)
a.emit('SLW', imm=SLOTW['ly'], a=T0, b=R_SLOT)
a.emit('SLW', imm=SLOTW['lf'], a=F_LF, b=R_SLOT)
a.emit('GAM', dst=T4, a=F_LF)
for steps, port in ((1, 'cu'), (2, 'cv')):
    a.emit('SHR', dst=T0, a=T9, imm=10)
    if steps == 2: a.emit('SHR', dst=T0, a=T0, imm=10)
    a.emit('AND', dst=T0, a=T0, imm=0x3FF)
    a.emit('LDI', dst=T1, imm=512)
    a.emit('SUB', dst=T0, a=T0, b=T1)
    a.emit('MUL', a=T0, b=T4)
    a.emit('MRD', dst=T2, imm=8)
    a.emit('ADD', dst=T2, a=T2, b=T1)
    a.emit('LDI', dst=T3, imm=1023)
    a.emit('CLP', dst=T2, a=T2, b=T3)
    a.emit('SLW', imm=SLOTW[port], a=T2, b=R_SLOT)

a.emit('ADD', dst=R_SLOT, a=R_SLOT, b=ONE)
a.label('f_next')
a.emit('ADD', dst=I, a=I, b=ONE)
a.emit('LDI', dst=T0, imm=6)
a.emit('JLT', a=I, b=T0, imm='face_top')
a.label('f_end')

# --- vignette seed cx^2 (low 22 bits, as the pixel path takes it) and the
# slot count the line engine loops over
a.emit('CTL', dst=T0, imm=6)
a.emit('MUL', a=T0, b=T0)
a.emit('MRD', dst=T1, imm=0)
a.emit('AND', dst=T2, a=T1, imm=0x3FFF)
a.emit('SHR', dst=T3, a=T1, imm=14)
a.emit('AND', dst=T3, a=T3, imm=0xFF)
a.emit('SHL', dst=T3, a=T3, imm=14)
a.emit('ADD', dst=T2, a=T2, b=T3)
a.emit('SLW', imm=SLOTW['qx0'], a=T2, b=ZERO)
a.emit('GWR', imm=127, a=ZERO, b=R_SLOT)

a.emit('END')

if __name__ == '__main__':
    w = a.words()
    print(f'-- {len(w)} instructions', file=sys.stderr)
    print(emit_vhdl(w, depth=1024))
