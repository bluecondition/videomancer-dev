#!/usr/bin/env python3
"""NAT20 per-field microcode (v0.2.0: the miniature, a scanline vector die).

Runs once per field on the engine in nat20.vhd (assembled by nat20_uasm.py).
Each field it:

  1. reads the controls: K1 die (5 zones, hysteresis), K2 yaw / K3 pitch
     (centre = the die's hero pose, deadbanded and glided), K4 colour
     (12 zones, hysteresis), K5 size and P12 key light (glided);
  2. builds the view rotation R = Rx(pitch) Ry(yaw) and projects the die's
     vertices (Q4 px, sized to the frame height, SD pixel aspect corrected)
     into the line-buffer WINDOW: x from the window's left edge, y from the
     top of the field;
  3. FRONT pass: every triangle facing the viewer gets its flat colour (key
     + fill + sky + ambient through the gamma curve), packed Y | U8 << 10 |
     V8 << 18 | (light g >> 3) << 26; a triangle facing away gets 0;
  4. EDGE pass: every edge with a front triangle on either side becomes one
     CROSSING RECORD for the renderer: the sub-lines it crosses [y0, y1)
     (16 a field line, half-open, so the edges meeting at a vertex share its
     sub-line exactly once), its x at sub-line y0 and its slope per sub-line
     (Q16 px), and the colour step across it, left to right: right face
     minus left face.  A silhouette stores its face's colour (+ entering on
     the left, - leaving on the right) and the renderer subtracts the
     backdrop of the line it draws.  The two halves of a split d6 square or
     d10 kite share a colour, so their diagonal has no record.

Records (32-bit words, the DDA table's 4 words an edge):
  w0  y0 | y1 << 15                      sub-line indices, field-relative
  w1  x at sub-line y0, Q16 px from the window's left edge
  w2  x step per sub-line, Q16 px
  w3  dY (11) | dU8 (9) << 11 | dV8 (9) << 20 | sil << 29 | side << 30
      (two's complement fields; side = 1: the face is on the left)

Formats: angles 65536 a turn; R, light Q12; die vertices and normals Q14
(circumradius 1); screen positions Q4 px.
"""
import math
import sys

from nat20_uasm import Asm, PORT, CTLR, ROM_SIN, ROM_SHW, ROM_NUM, ROM_INK, ROM_INKM, ROM_GAM
import nat20_tables as T

a = Asm()
e = a.emit
r = a.r

# A triangle is FRONT when its area2 (Q8 px^2) is under -4 R (R the die's
# projected radius, Q4).  Vertices are rounded to 1/16 px, so a face seen
# edge-on can measure on either side of 0, and so can its parallel opposite
# face.  The rounding error grows with the die (~R); four times that drops
# only faces narrower than ~0.25 px, which are always on the outline.
YTOP, YBOT = 660, 430            # backdrop luma, top (C_BGT in the VHDL) and bottom
MAX_EDGES = 32                   # crossing records a field (the table's depth)
SIZE_LO, SIZE_SPAN = 1638, 5571  # K5: radius 2.5 % .. 11 % of the height (Q16)

# ------------------------------------------------------------ registers
# Persistent (live across the passes; the glides across fields -- the
# register file powers up at zero and the first field snaps them).
for nm in ('zero', 'one', 'die', 'col', 'gy', 'gp', 'gl', 'gs', 'hy', 'hp', 'hl', 'hs'):
    r(nm)
T0, T1, T2, T3, T4, T5, T6 = (r(f't{i}') for i in range(7))
RET = r('ret')
SX = a.block('SX', 12)
SY = a.block('SY', 12)
FC = a.block('FC', 20)           # tri -> packed flat colour; 0 = facing away
R = [[r(f'r{i}{j}') for j in range(3)] for i in range(3)]
YST, YOFF, NEO = r('yst'), r('yoff'), r('neo')
TB, NT, EB, NE, NB, SPL = (r(n) for n in ('tbase', 'nt', 'ebase', 'ne', 'nbase', 'split'))
ZY = r('zy')


def L(stem):
    return a.lbl(stem)


def call(sub):
    ret = L('ret')
    e('LDI', RET, imm=ret)
    e('JMP', imm=sub)
    a.label(ret)


def mulsh(dst, x, y, sh=12):
    e('MULSH', dst, x, y, imm=sh)


e('JMP', imm='main')

# ======================================================= subroutines
# TRIG: t1 = sin(t0) Q12, t0 an angle (65536 a turn; only the low 16 bits
# count).  Quarter table (65 entries) with linear interpolation.
a.label('TRIG')
e('SHR', T3, T0, imm=8)
e('AND', T3, T3, imm=63)
e('SHR', T4, T0, imm=14)
e('AND', T4, T4, imm=1)
e('JNZ', a=T4, imm='tr_mir')
e('ROM', T5, T3)
e('ADD', T3, T3, 'one')
e('ROM', T3, T3)
e('JMP', imm='tr_int')
a.label('tr_mir')
e('LDI', T4, imm=64)
e('SUB', T3, T4, T3)
e('ROM', T5, T3)
e('SUB', T3, T3, 'one')
e('ROM', T3, T3)
a.label('tr_int')
e('SUB', T3, T3, T5)
e('AND', T2, T0, imm=255)
mulsh(T3, T3, T2, 0)
e('SHR', T3, T3, imm=8)
e('ADD', T1, T5, T3)
e('SHR', T4, T0, imm=15)
e('AND', T4, T4, imm=1)
e('JNZ', a=T4, imm='tr_neg')
e('JR', a=RET)
a.label('tr_neg')
e('NEG', T1, T1)
e('JR', a=RET)

a.label('TRIG_COS')                     # cos(t0) = sin(t0 + 16384)
e('LDI', T2, imm=4096)
e('SHL', T2, T2, imm=2)
e('ADD', T0, T0, T2)
e('JMP', imm='TRIG')

# DIV: t2 = t0 / t1 (t0 in 0 .. 2^26 - 1, t1 > 0), restoring, 26 quotient
# bits.  t0 is pre-shifted so its bit 25 sits in the sign bit.
a.label('DIV')
e('SHL', T0, T0, imm=6)
e('LDI', T2, imm=0)
e('LDI', T3, imm=0)                     # remainder
e('LDI', T4, imm=26)
a.label('dv_lp')
e('SHL', T3, T3, imm=1)
e('JGE', a=T0, b='zero', imm='dv_nb')
e('ADD', T3, T3, 'one')
a.label('dv_nb')
e('SHL', T0, T0, imm=1)
e('SHL', T2, T2, imm=1)
e('JLT', a=T3, b=T1, imm='dv_ns')
e('SUB', T3, T3, T1)
e('ADD', T2, T2, 'one')
a.label('dv_ns')
e('SUB', T4, T4, 'one')
e('JNZ', a=T4, imm='dv_lp')
e('JR', a=RET)

# PACK: t2 = t0 << 16 | (t1 & 0xFFFF)  (two signed 16-bit fields in a word)
a.label('PACK')
e('JGE', a=T1, b='zero', imm='pk_p')
e('LDI', T2, imm=1)
e('SHLL', T2, T2, imm=16)
e('ADD', T1, T1, T2)
a.label('pk_p')
e('SHLL', T2, T0, imm=16)
e('ADD', T2, T2, T1)
e('JR', a=RET)

# ISQRT: t2 = floor(sqrt(t0)), t0 < 2^30 (bit by bit; clobbers t0, t3, t4)
a.label('ISQRT')
e('LDI', T2, imm=0)
e('LDI', T3, imm=1)
e('SHLL', T3, T3, imm=28)
a.label('sq_lp')
e('ADD', T4, T2, T3)
e('JLT', a=T0, b=T4, imm='sq_no')
e('SUB', T0, T0, T4)
e('SHR', T2, T2, imm=1)
e('ADD', T2, T2, T3)
e('JMP', imm='sq_nx')
a.label('sq_no')
e('SHR', T2, T2, imm=1)
a.label('sq_nx')
e('SHR', T3, T3, imm=2)
e('JNZ', a=T3, imm='sq_lp')
e('JR', a=RET)

# ======================================================= main
a.label('main')
e('LDI', 'zero', imm=0)
e('LDI', 'one', imm=1)

# ---------------------------------------------------- 1. controls
with a.scope():
    ZF = r('zf')
    e('CTL', ZF, imm=CTLR['zfirst'])

    def zones(dst, ctl, n, hyst):
        """dst = zone 0..n-1 of a labelled knob, with a +-hyst count
        hysteresis band.  At program load the firmware writes a labelled
        knob's default as the label INDEX (0..n-1), not a position, so a
        value under n is taken as the index itself."""
        lo, done = L('zl'), L('zd')
        e('CTL', T0, imm=ctl)
        e('LDI', T5, imm=n)
        mulsh(T1, T0, T5, 0)
        e('SHR', T1, T1, imm=10)            # raw zone
        e('JGE', a=T0, b=T5, imm=(lpos := L('zp')))
        e('MOV', T1, T0)                    # an index, not a position
        e('JMP', imm=lo)
        a.label(lpos)
        e('JNZ', a=ZF, imm=lo)              # first field: snap
        e('LDI', T2, imm=hyst)
        e('SUB', T3, T0, T2)
        e('MAX', T3, T3, 'zero')
        mulsh(T3, T3, T5, 0)
        e('SHR', T3, T3, imm=10)            # zone of knob - hyst
        e('ADD', T4, T0, T2)
        e('LDI', T2, imm=1023)
        e('MIN', T4, T4, T2)
        mulsh(T4, T4, T5, 0)
        e('SHR', T4, T4, imm=10)            # zone of knob + hyst
        e('MAX', dst, dst, T3)
        e('MIN', dst, dst, T4)
        e('JMP', imm=done)
        a.label(lo)
        e('MOV', dst, T1)
        a.label(done)

    zones('die', CTLR['k1'], 5, 24)
    zones('col', CTLR['k4'], 12, 16)

    # GLIDE: deadband then glide.  t0 = knob, t5 = h, t6 = g, t3 = 1 for an
    # angle.  h follows the knob when it moves 3+ counts and g eases 1/8 a
    # field toward the target: (h - 512) << 6 for an angle (knob centre = 0,
    # 65536 a turn, the short way round), else h << 4 (Q4).  The first field
    # snaps.
    e('JMP', imm='gl_over')
    a.label('GLIDE')
    e('JNZ', a=ZF, imm='gl_snap')
    e('SUB', T1, T0, T5)
    e('ABS', T1, T1)
    e('LDI', T2, imm=3)
    e('JLT', a=T1, b=T2, imm='gl_keep')
    a.label('gl_snap')
    e('MOV', T5, T0)
    a.label('gl_keep')
    e('JNZ', a=T3, imm='gl_ang')
    e('SHL', T0, T5, imm=4)
    e('JMP', imm='gl_tg')
    a.label('gl_ang')
    e('LDI', T2, imm=512)
    e('SUB', T0, T5, T2)
    e('SHL', T0, T0, imm=6)
    a.label('gl_tg')
    e('JNZ', a=ZF, imm='gl_set')
    e('SUB', T1, T0, T6)
    e('JNZ', a=T3, imm='gl_sx')
    e('JMP', imm='gl_nsx')
    a.label('gl_sx')                    # sign-extend the low 16 bits
    e('SHL', T1, T1, imm=15)
    e('SHL', T1, T1, imm=1)
    e('SHR', T1, T1, imm=15)
    e('SHR', T1, T1, imm=1)
    a.label('gl_nsx')
    e('SHR', T1, T1, imm=3)
    e('ADD', T6, T6, T1)
    e('JR', a=RET)
    a.label('gl_set')
    e('MOV', T6, T0)
    e('JR', a=RET)
    a.label('gl_over')

    for g, h, ctl, angle in (('gy', 'hy', CTLR['k2'], 1), ('gp', 'hp', CTLR['k3'], 1),
                             ('gs', 'hs', CTLR['k5'], 0), ('gl', 'hl', CTLR['slider'], 0)):
        e('CTL', T0, imm=ctl)
        e('MOV', T5, h)
        e('MOV', T6, g)
        e('LDI', T3, imm=angle)
        call('GLIDE')
        e('MOV', h, T5)
        e('MOV', g, T6)

# ---------------------------------------------------- 2. rotation
with a.scope():
    SYW, CYW, SPT, CPT = r('syw'), r('cyw'), r('spt'), r('cpt')
    for ang, s, c in (('gy', SYW, CYW), ('gp', SPT, CPT)):
        e('MOV', T0, ang); call('TRIG'); e('MOV', s, T1)
        e('MOV', T0, ang); call('TRIG_COS'); e('MOV', c, T1)
    # R = Rx(p) Ry(y): rows (cy, 0, sy), (sp sy, cp, -sp cy), (-cp sy, sp, cp cy)
    e('MOV', R[0][0], CYW); e('LDI', R[0][1], imm=0); e('MOV', R[0][2], SYW)
    mulsh(R[1][0], SPT, SYW); e('MOV', R[1][1], CPT)
    mulsh(T0, SPT, CYW); e('NEG', R[1][2], T0)
    mulsh(T0, CPT, SYW); e('NEG', R[2][0], T0)
    e('MOV', R[2][1], SPT); mulsh(R[2][2], CPT, CYW)

# ---------------------------------------------------- 3. framing, vertices
with a.scope():
    W, HF, BW, ZX, CX4, CY4 = (r(n) for n in ('W', 'hf', 'bw', 'zx', 'cx4', 'cy4'))
    e('CTL', W, imm=CTLR['W'])
    e('CTL', HF, imm=CTLR['hf'])
    e('CTL', BW, imm=CTLR['bw'])
    # radius (Q4 px) = hf * 16 * (2.5 % .. 11 %) from the K5 glide (Q4)
    e('LDI', T0, imm=SIZE_SPAN)
    mulsh(T0, 'gs', T0, 0)
    e('SHR', T0, T0, imm=14)
    e('LDI', T1, imm=SIZE_LO)
    e('ADD', T0, T0, T1)
    mulsh(ZY, HF, T0)
    e('CTL', T1, imm=CTLR['xa'])        # SD pixel aspect, Q12 offset
    mulsh(T1, ZY, T1)
    e('ADD', ZX, ZY, T1)
    e('SHL', CX4, BW, imm=3)            # the window's centre
    e('SHL', CY4, HF, imm=3)
    e('SUB', T0, W, BW)                 # window left edge, published
    e('SHR', T0, T0, imm=1)
    e('SLW', a=T0, b='zero', imm=PORT['wx0'])
    e('CTL', T0, imm=CTLR['ilace'])
    e('CTL', T1, imm=CTLR['field'])
    e('ADD', YST, 'one', T0)
    e('SUB', T2, 'one', T1)
    e('MIN', YOFF, T0, T2)
    e('SHL', YOFF, YOFF, imm=4)         # the field's first line, Q4

    VB, NV = r('vbase'), r('nv')
    e('SHL', T0, 'die', imm=3)
    for i, reg in enumerate((VB, NV, TB, NT, EB, NE, NB, SPL)):
        e('SRD', reg, T0, imm=T.HDR + i)

    # projection rows, Q4 px per die unit (Q12 R)
    M = [[r(f'm{i}{j}') for j in range(3)] for i in range(2)]
    for j in range(3):
        mulsh(M[0][j], ZX, R[0][j])
        mulsh(T0, ZY, R[1][j])
        e('NEG', M[1][j], T0)

    VC = [r(f'vc{c}') for c in range(3)]
    e('LDI', T5, imm=0)                 # i
    e('MOV', T4, VB)                    # table pointer
    a.label('vx')
    for c in range(3):
        e('SRDS', VC[c], T4, imm=c)
    for row, base, cen in ((0, SX, CX4), (1, SY, CY4)):
        mulsh(T0, M[row][0], VC[0])
        mulsh(T1, M[row][1], VC[1])
        e('ADD', T0, T0, T1)
        mulsh(T1, M[row][2], VC[2])
        e('ADD', T0, T0, T1)
        e('SHR', T0, T0, imm=2)
        e('ADD', T0, T0, cen)
        e('STX', a=T5, b=T0, imm=base)
    e('LDI', T0, imm=3)
    e('ADD', T4, T4, T0)
    e('ADD', T5, T5, 'one')
    e('JLT', a=T5, b=NV, imm='vx')

# ---------------------------------------------------- 4. light, front pass
with a.scope():
    LD = [r(f'ld{j}') for j in range(3)]
    FD = [r(f'fd{j}') for j in range(3)]
    YB, UB, VB = r('yb'), r('ub'), r('vbd')
    with a.scope():
        LSA, LCA, NRM = r('lsa'), r('lca'), r('lnrm')
        e('SHL', T0, 'gl', imm=1)       # P12 (Q4) 0..1023 -> 0..pi
        e('MOV', NRM, T0)
        call('TRIG'); e('MOV', LSA, T1)
        e('MOV', T0, NRM); call('TRIG_COS'); e('MOV', LCA, T1)
        # L = (-cos, 0.80 sin, 0.22 + 0.55 sin) * ~1/|L|
        e('LDI', T0, imm=311)
        mulsh(T0, T0, LSA)
        e('LDI', NRM, imm=4000)
        e('SUB', NRM, NRM, T0)
        LV = [r('lx'), r('ly'), r('lz')]
        mulsh(T0, LCA, NRM); e('NEG', LV[0], T0)
        e('LDI', T0, imm=3277); mulsh(T0, T0, LSA); mulsh(LV[1], T0, NRM)
        e('LDI', T0, imm=2253); mulsh(T0, T0, LSA)
        e('LDI', T1, imm=901); e('ADD', T0, T0, T1); mulsh(LV[2], T0, NRM)
        FV = [r('fvx'), r('fvy'), r('fvz')]
        for reg, v in zip(FV, (2652, 612, 3060)):    # fill (0.65, 0.15, 0.75)
            e('LDI', reg, imm=v)
        for vec, dst in ((LV, LD), (FV, FD)):        # into die space: R^T v
            for j in range(3):
                mulsh(dst[j], R[0][j], vec[0])
                mulsh(T0, R[1][j], vec[1]); e('ADD', dst[j], dst[j], T0)
                mulsh(T0, R[2][j], vec[2]); e('ADD', dst[j], dst[j], T0)
    e('SHL', T0, 'col', imm=1)
    e('ADD', T0, T0, 'col')
    e('SRD', YB, T0, imm=T.COLB)
    e('SRD', UB, T0, imm=T.COLB + 1)
    e('SRD', VB, T0, imm=T.COLB + 2)

    VI = [r(f'fv{k}') for k in range(3)]
    FX = [r(f'fx{k}') for k in range(3)]
    FY = [r(f'fy{k}') for k in range(3)]
    FTH, TI = r('fth'), r('ti')
    NV3 = [r('nx'), r('ny'), r('nz')]
    NL, NF, NU, LIN = r('nl'), r('nf'), r('nu'), r('lin')
    e('SHL', FTH, ZY, imm=2)
    e('NEG', FTH, FTH)
    e('LDI', TI, imm=0)                 # t
    a.label('fr')
    e('ADD', T0, TB, TI)
    e('SRD', T4, T0, imm=0)             # v0 | v1 << 4 | v2 << 8
    e('AND', VI[0], T4, imm=15)
    e('SHR', T4, T4, imm=4)
    e('AND', VI[1], T4, imm=15)
    e('SHR', T4, T4, imm=4)
    e('AND', VI[2], T4, imm=15)
    for k in range(3):
        e('LDX', FX[k], VI[k], imm=SX)
        e('LDX', FY[k], VI[k], imm=SY)
    e('SUB', T0, FX[1], FX[0])
    e('SUB', T1, FY[2], FY[0])
    mulsh(T2, T0, T1, 0)
    e('SUB', T0, FY[1], FY[0])
    e('SUB', T1, FX[2], FX[0])
    mulsh(T3, T0, T1, 0)
    e('SUB', T2, T2, T3)                # area2, Q8 px^2 (negative = front)
    e('STX', a=TI, b='zero', imm=FC)
    e('JGE', a=T2, b=FTH, imm='fr_next')  # facing away (or edge-on)
    # flat colour of the triangle's face
    e('MOV', T4, TI)
    e('JNZ', a=SPL, imm='fr_split')
    e('JMP', imm='fr_f')
    a.label('fr_split')
    e('SHR', T4, TI, imm=1)
    a.label('fr_f')
    e('SHL', T0, T4, imm=1)
    e('ADD', T0, T0, T4)
    e('ADD', T4, T0, NB)
    for c in range(3):
        e('SRDS', NV3[c], T4, imm=c)

    def dot(dst, vec):
        mulsh(dst, NV3[0], vec[0], 12)
        mulsh(T0, NV3[1], vec[1], 12); e('ADD', dst, dst, T0)
        mulsh(T0, NV3[2], vec[2], 12); e('ADD', dst, dst, T0)
        e('SHR', dst, dst, imm=2)       # Q14 x Q12 >> 14 = Q12

    dot(NL, LD)
    dot(NF, FD)
    dot(NU, R[1])                       # view up in die space = row 1 of R
    e('MAX', NL, NL, 'zero')
    e('MAX', NF, NF, 'zero')
    # lin = 0.92 nl + 0.30 nf + 0.08 nu + 0.15 (Q12): key, fill, sky, ambient
    e('LDI', T0, imm=3768); mulsh(LIN, NL, T0)
    e('LDI', T0, imm=1229); mulsh(T1, NF, T0); e('ADD', LIN, LIN, T1)
    e('LDI', T0, imm=328); mulsh(T1, NU, T0); e('ADD', LIN, LIN, T1)
    e('LDI', T0, imm=614); e('ADD', LIN, LIN, T0)
    # gamma index lin * 96 / 4096, 0..127
    e('LDI', T0, imm=96); mulsh(T1, LIN, T0)
    e('LDI', T0, imm=127)
    e('CLP', T1, T1, T0)
    e('LDI', T0, imm=ROM_GAM)
    e('ADD', T1, T1, T0)
    e('ROM', T5, T1)                    # g, 256 = 1.0
    # Y = 64 + (yb - 64) g / 256 (capped 768), U/V = 512 + (c - 512) g / 256
    e('LDI', T0, imm=64)
    e('SUB', T1, YB, T0)
    mulsh(T1, T1, T5, 0)
    e('SHR', T1, T1, imm=8)
    e('ADD', T1, T1, T0)
    e('LDI', T0, imm=768)
    e('MIN', T6, T1, T0)                # packed colour, Y in bits 0..9
    for src, sh in ((UB, 10), (VB, 18)):
        e('LDI', T0, imm=512)
        e('SUB', T1, src, T0)
        mulsh(T1, T1, T5, 0)
        e('SHR', T1, T1, imm=8)
        e('ADD', T1, T1, T0)
        e('LDI', T0, imm=1023)
        e('CLP', T1, T1, T0)
        e('SHR', T1, T1, imm=2)         # 8-bit chroma
        e('SHLL', T1, T1, imm=sh)
        e('ADD', T6, T6, T1)
    e('SHR', T1, T5, imm=3)             # the face's light g / 8, for its ink
    e('SHLL', T1, T1, imm=26)
    e('ADD', T6, T6, T1)
    e('STX', a=TI, b=T6, imm=FC)
    a.label('fr_next')
    e('ADD', TI, TI, 'one')
    e('JLT', a=TI, b=NT, imm='fr')

# ---------------------------------------------------- 5. edge pass
with a.scope():
    TA, KA, TBi, FK = r('tA'), r('kA'), r('tB'), r('fk')
    VA, VQ, CR, CL = r('va'), r('vq'), r('cr'), r('cl')
    PX, PY, QX, QY, DX, DY = (r(n) for n in ('px', 'py', 'qx', 'qy', 'dx', 'dy'))
    Y0E, Y1E, SL, XF, W3, EI = (r(n) for n in ('y0e', 'y1e', 'sl', 'xf', 'w3', 'ei'))
    e('LDI', NEO, imm=0)
    e('LDI', EI, imm=0)
    a.label('ed')
    e('ADD', T0, EB, EI)
    e('SRD', T4, T0, imm=0)             # tA | kA<<5 | tB<<7 | kB<<12 | fake<<14
    e('AND', TA, T4, imm=31)
    e('SHR', T4, T4, imm=5)
    e('AND', KA, T4, imm=3)
    e('SHR', T4, T4, imm=2)
    e('AND', TBi, T4, imm=31)
    e('SHR', FK, T4, imm=7)
    e('LDX', CR, TA, imm=FC)
    e('LDX', CL, TBi, imm=FC)
    e('JNZ', a=CR, imm='ed_a')
    e('JNZ', a=CL, imm='ed_go')
    e('JMP', imm='ed_next')             # both facing away
    a.label('ed_a')
    e('JNZ', a=CL, imm='ed_two')
    e('JMP', imm='ed_go')
    a.label('ed_two')
    e('JNZ', a=FK, imm='ed_next')       # a split face's own diagonal
    a.label('ed_go')
    # the edge's vertices in tri tA's direction: edge kA runs v_kA -> v_(kA+1)
    e('ADD', T0, TB, TA)
    e('SRD', T4, T0, imm=0)
    e('AND', T1, T4, imm=15)            # v0
    e('SHR', T4, T4, imm=4)
    e('AND', T2, T4, imm=15)            # v1
    e('SHR', T4, T4, imm=4)
    e('AND', T3, T4, imm=15)            # v2
    e('JNZ', a=KA, imm='ek12')
    e('MOV', VA, T1); e('MOV', VQ, T2); e('JMP', imm='ek_ok')
    a.label('ek12')
    e('SUB', T0, KA, 'one')
    e('JNZ', a=T0, imm='ek2')
    e('MOV', VA, T2); e('MOV', VQ, T3); e('JMP', imm='ek_ok')
    a.label('ek2')
    e('MOV', VA, T3); e('MOV', VQ, T1)
    a.label('ek_ok')
    e('LDX', PX, VA, imm=SX)
    e('LDX', PY, VA, imm=SY)
    e('LDX', QX, VQ, imm=SX)
    e('LDX', QY, VQ, imm=SY)
    e('SUB', DY, QY, PY)
    e('JNZ', a=DY, imm='ed_ny')
    e('JMP', imm='ed_next')             # horizontal: crosses no sub-line
    a.label('ed_ny')
    # A front triangle winds counter-clockwise on the y-down screen, so the
    # one whose own direction along the edge runs DOWN lies on its right.
    # tA runs P -> Q: going down, tA is the right face; else tB is.
    e('JLT', a='zero', b=DY, imm='ed_down')
    e('MOV', T0, CR); e('MOV', CR, CL); e('MOV', CL, T0)     # right = tB
    e('MOV', T0, PX); e('MOV', PX, QX); e('MOV', QX, T0)     # P the top end
    e('MOV', T0, PY); e('MOV', PY, QY); e('MOV', QY, T0)
    e('NEG', DY, DY)
    a.label('ed_down')
    e('SUB', DX, QX, PX)
    # sub-lines crossed, [ceil((Py - yoff) / yst), ceil((Qy - yoff) / yst))
    for src, dst in ((PY, Y0E), (QY, Y1E)):
        e('SUB', dst, src, YOFF)
        e('SUB', T0, YST, 'one')
        e('JNZ', a=T0, imm=(lh := L('yh')))
        e('JMP', imm=(ld := L('yd')))
        a.label(lh)
        e('ADD', dst, dst, 'one')
        e('SHR', dst, dst, imm=1)
        a.label(ld)
    e('JGE', a=Y0E, b=Y1E, imm='ed_next')
    # slope per sub-line, Q16 px: yst * dx * 4096 / dy
    e('ABS', T0, DX)
    e('SHL', T0, T0, imm=12)
    e('SUB', T1, YST, 'one')
    e('JNZ', a=T1, imm='sl_2')
    e('JMP', imm='sl_1')
    a.label('sl_2')
    e('SHL', T0, T0, imm=1)
    a.label('sl_1')
    e('MOV', T1, DY)
    e('SHR', T2, DY, imm=1)             # rounded: the slope's error
    e('ADD', T0, T0, T2)                # accumulates down the edge
    call('DIV')
    e('MOV', SL, T2)
    e('JGE', a=DX, b='zero', imm='sl_pos')
    e('NEG', SL, SL)
    a.label('sl_pos')
    # x at sub-line y0: Px, plus half a step when that sub-line sits one Q4
    # row below P (interlace)
    e('SHL', XF, PX, imm=12)
    e('LDI', T0, imm=2048)              # + 1/32 px: the renderer's 1/16 px
    e('ADD', XF, XF, T0)                # fraction then rounds, not truncates
    e('MOV', T0, Y0E)
    e('SUB', T1, YST, 'one')
    e('JNZ', a=T1, imm='xf_2')
    e('JMP', imm='xf_1')
    a.label('xf_2')
    e('ADD', T0, T0, T0)
    a.label('xf_1')
    e('ADD', T0, T0, YOFF)
    e('SUB', T0, T0, PY)
    e('JNZ', a=T0, imm='xf_h')
    e('JMP', imm='xf_ok')
    a.label('xf_h')
    e('SHR', T0, SL, imm=1)
    e('ADD', XF, XF, T0)
    a.label('xf_ok')
    # colour step across the edge, left to right: right face - left face
    # (a face facing away counts 0, so a silhouette carries +-its face)
    e('AND', T0, CR, imm=1023)
    e('AND', T1, CL, imm=1023)
    e('SUB', W3, T0, T1)
    e('AND', W3, W3, imm=2047)
    for sh, out in ((10, 11), (18, 20)):
        e('SHRL', T0, CR, imm=sh)
        e('AND', T0, T0, imm=255)
        e('SHRL', T1, CL, imm=sh)
        e('AND', T1, T1, imm=255)
        e('SUB', T0, T0, T1)
        e('AND', T0, T0, imm=511)
        e('SHLL', T0, T0, imm=out)
        e('ADD', W3, W3, T0)
    e('JNZ', a=CR, imm='w3_r')
    a.label('w3_side')                  # right face away: the face is on the left
    e('LDI', T0, imm=3)
    e('SHLL', T0, T0, imm=29)           # sil | side
    e('ADD', W3, W3, T0)
    e('JMP', imm='w3_ok')
    a.label('w3_r')
    e('JNZ', a=CL, imm='w3_ok')
    e('LDI', T0, imm=1)
    e('SHLL', T0, T0, imm=29)           # sil
    e('ADD', W3, W3, T0)
    a.label('w3_ok')
    e('SHL', T0, Y1E, imm=15)
    e('ADD', T0, T0, Y0E)
    e('SLW', a=T0, b=NEO, imm=PORT['e0'])
    e('SLW', a=XF, b=NEO, imm=PORT['e1'])
    e('SLW', a=SL, b=NEO, imm=PORT['e2'])
    e('SLW', a=W3, b=NEO, imm=PORT['e3'])
    e('ADD', NEO, NEO, 'one')
    e('LDI', T0, imm=MAX_EDGES)
    e('JGE', a=NEO, b=T0, imm='ed_done')
    a.label('ed_next')
    e('ADD', EI, EI, 'one')
    e('JLT', a=EI, b=NE, imm='ed')
    a.label('ed_done')

# ---------------------------------------------------- 6. numerals
# One SPRITE per numeral on a front face: its glyph box mapped onto the face.
# The frame comes from the projected corners (projection is linear, so a
# fixed mix of a face's corners is the projection of the same mix in die
# space): a centre P, an up vector UPV and a rightward RT, then
#     c = P + UPV * TC,  U = RT * SU,  V = -UPV * SV     (per-die constants)
# U, V are the screen px a glyph texel steps (Q11).  Inverted, they give the
# texel steps per pixel (Q12) the renderer walks; the box's screen bounds
# give the lines and the span it walks them over.
with a.scope():
    TCr, SUr, SVr, IKB = (r(n) for n in ('tc', 'su', 'sv', 'ikb'))
    NSO, FI, NF, CF, W0, W1 = (r(n) for n in ('nso', 'fi', 'nf', 'cf', 'w0', 'w1'))
    KK, NK, NUM, GA, GB, NU = (r(n) for n in ('kk', 'nk', 'num', 'ga', 'gb', 'nu'))
    # P, UPV, RT become O, V, U in place
    PXr, PYr, UXr, UYr, RXr, RYr = (r(n) for n in ('ppx', 'ppy', 'upx', 'upy', 'rtx', 'rty'))
    OX, OY, VX, VY, UX, UY = PXr, PYr, UXr, UYr, RXr, RYr
    DET, RR, DUX, DVX, DUY, DVY = (r(n) for n in ('det', 'rr', 'dux', 'dvx', 'duy', 'dvy'))
    SX0, SW, LY0, LY1 = (r(n) for n in ('sx0', 'sw', 'ly0', 'ly1'))
    QV = a.block('QV', 7)               # a face's corners, then the first ones again
    assert QV + 6 < 128

    e('LDI', NSO, imm=0)
    e('SHL', T0, 'die', imm=1)
    e('ADD', T0, T0, 'die')
    e('LDI', T1, imm=ROM_NUM)
    e('ADD', T0, T0, T1)
    e('ROMS', TCr, T0)
    e('ADD', T0, T0, 'one')
    e('ROM', SUr, T0)
    e('ADD', T0, T0, 'one')
    e('ROM', SVr, T0)
    # ink: black on the light colours, else white (a mask bit per colour)
    e('LDI', T0, imm=ROM_INKM)
    e('ROM', T1, T0)
    e('MOV', T2, 'col')
    a.label('ik_sh')
    e('JNZ', a=T2, imm='ik_go')
    e('JMP', imm='ik_dn')
    a.label('ik_go')
    e('SHR', T1, T1, imm=1)
    e('SUB', T2, T2, 'one')
    e('JMP', imm='ik_sh')
    a.label('ik_dn')
    e('AND', T1, T1, imm=1)
    e('SHL', T0, T1, imm=1)
    e('ADD', T0, T0, T1)                # 3 * ink
    e('LDI', T1, imm=ROM_INK)
    e('ADD', IKB, T0, T1)

    e('MOV', NF, NT)
    e('JNZ', a=SPL, imm='nm_spl')
    e('JMP', imm='nm_nf')
    a.label('nm_spl')
    e('SHR', NF, NT, imm=1)
    a.label('nm_nf')
    e('LDI', FI, imm=0)
    a.label('nm_face')
    e('MOV', T5, FI)                    # the face's first triangle
    e('JNZ', a=SPL, imm='nm_t2')
    e('JMP', imm='nm_t1')
    a.label('nm_t2')
    e('ADD', T5, FI, FI)
    a.label('nm_t1')
    e('LDX', CF, T5, imm=FC)
    e('JNZ', a=CF, imm='nm_front')
    e('JMP', imm='nm_fnext')
    a.label('nm_front')
    e('ADD', T0, TB, T5)
    e('SRD', W0, T0, imm=0)
    e('SRD', W1, T0, imm=1)
    e('LDI', NK, imm=1)
    e('JNZ', a='die', imm='nm_nk')
    e('LDI', NK, imm=3)                 # d4: a numeral at each corner
    a.label('nm_nk')
    e('LDI', KK, imm=0)
    a.label('nm_inst')
    # ---- the frame of this numeral (P, UPV, RT in Q4 px) and its number.
    # The face's corners go to QV[0..3] (a square's 4th is tri 2's last, a
    # kite's foot tri 2's middle) and again from QV[4]; a triangle repeats
    # with period 3 instead.  So the four corners from k are QV[k..k+3]; each
    # of P, UPV, RT is a weighted sum of them, the weights per face shape.
    e('LDI', T6, imm=QV)
    for i in range(3):
        e('SHR', T0, W0, imm=4 * i)
        e('AND', T0, T0, imm=15)
        e('STX', a=T6, b=T0, imm=i)
        e('STX', a=T6, b=T0, imm=4 + i)
    e('AND', T0, W0, imm=15)
    e('LDI', T2, imm=0)                 # shape: triangle
    e('JNZ', a=SPL, imm='nm_spl4')
    e('SHR', T1, W0, imm=4)             # a triangle repeats with period 3:
    e('AND', T1, T1, imm=15)            # QV[3], QV[4] = its corners 0, 1
    e('STX', a=T6, b=T1, imm=4)
    e('JMP', imm='nm_4th')
    a.label('nm_spl4')
    e('SHR', T0, W1, imm=4)             # kite: the foot
    e('LDI', T2, imm=24)
    e('LDI', T1, imm=3)
    e('JGE', a='die', b=T1, imm='nm_kf')
    e('SHR', T0, W1, imm=8)             # square: the 4th corner
    e('LDI', T2, imm=12)
    a.label('nm_kf')
    e('AND', T0, T0, imm=15)
    a.label('nm_4th')
    e('STX', a=T6, b=T0, imm=3)
    e('LDI', T1, imm=ROM_SHW)
    e('ADD', T6, T2, T1)                # this shape's weights
    e('SHR', T3, W0, imm=12)            # k: the up corner / edge ...
    e('AND', T3, T3, imm=3)
    e('JNZ', a='die', imm='nm_kd')
    e('MOV', T3, KK)                    # ... or the d4 instance
    a.label('nm_kd')
    e('LDX', T0, T3, imm=QV)
    e('ADD', NUM, T0, 'one')            # d4: the corner's vertex number
    e('JNZ', a='die', imm='nm_num')
    e('JMP', imm='nm_numd')
    a.label('nm_num')
    e('ADD', NUM, FI, 'one')
    e('LDI', T0, imm=3)
    e('SUB', T0, 'die', T0)
    e('JNZ', a=T0, imm='nm_numd')
    e('MOV', NUM, FI)                   # d10: 0..9
    a.label('nm_numd')
    for reg in (PXr, PYr, UXr, UYr, RXr, RYr):
        e('LDI', reg, imm=0)
    J = W1                              # W1 is spent once the corners are in
    e('LDI', J, imm=0)
    a.label('nm_wl')
    e('ADD', T0, T3, J)
    e('LDX', T4, T0, imm=QV)            # corner k + j
    e('LDX', T5, T4, imm=SX)
    e('LDX', T4, T4, imm=SY)
    for ax, ay, wi in ((PXr, PYr, 0), (UXr, UYr, 4), (RXr, RYr, 8)):
        e('ADD', T0, T6, J)
        if wi:
            e('LDI', T1, imm=wi)
            e('ADD', T0, T0, T1)
        e('ROMS', T1, T0)               # weight, Q12
        mulsh(T2, T5, T1)
        e('ADD', ax, ax, T2)
        mulsh(T2, T4, T1)
        e('ADD', ay, ay, T2)
    e('ADD', J, J, 'one')
    e('LDI', T0, imm=4)
    e('JLT', a=J, b=T0, imm='nm_wl')
    a.label('nm_frame')
    # ---- glyphs: one digit, or two for 10..20; 6 and 9 underlined on d10/d20
    e('LDI', GB, imm=15)
    e('LDI', NU, imm=16)
    e('MOV', GA, NUM)
    e('LDI', T0, imm=10)
    e('JLT', a=NUM, b=T0, imm='gl_one')
    e('LDI', NU, imm=32)
    e('LDI', GA, imm=1)
    e('SUB', GB, NUM, T0)
    e('JLT', a=GB, b=T0, imm='gl_dn')
    e('LDI', GA, imm=2)
    e('SUB', GB, GB, T0)
    e('JMP', imm='gl_dn')
    a.label('gl_one')
    e('LDI', T0, imm=3)
    e('JLT', a='die', b=T0, imm='gl_dn')    # d4 d6 d8: no underline
    e('LDI', T0, imm=6)
    e('SUB', T0, NUM, T0)
    e('JNZ', a=T0, imm='gl_n9')
    e('LDI', GA, imm=10)
    a.label('gl_n9')
    e('LDI', T0, imm=9)
    e('SUB', T0, NUM, T0)
    e('JNZ', a=T0, imm='gl_dn')
    e('LDI', GA, imm=11)
    a.label('gl_dn')
    # ---- screen frame (Q11): c = P + UPV TC, V = -UPV SV, U = RT SU, then
    # the box origin O = c - U NU/2 - 10 V (P, UPV, RT -> O, V, U in place)
    for P_, U_, R_ in ((PXr, UXr, RXr), (PYr, UYr, RYr)):
        mulsh(T0, U_, TCr)
        e('ADD', T0, T0, P_)
        e('SHL', P_, T0, imm=7)         # c, Q11
        mulsh(T0, U_, SVr)
        e('NEG', U_, T0)                # V
        mulsh(R_, R_, SUr)              # U
        mulsh(T0, R_, NU, 0)
        e('SHR', T0, T0, imm=1)
        e('SUB', P_, P_, T0)
        e('SHL', T0, U_, imm=3)
        e('SHL', T1, U_, imm=1)
        e('ADD', T0, T0, T1)
        e('SUB', P_, P_, T0)            # O
    # ---- inverse: texel steps per pixel, Q12 (det Q22; too squashed: skip)
    mulsh(DET, UX, VY, 0)
    mulsh(T0, VX, UY, 0)
    e('SUB', DET, DET, T0)
    e('SHR', T1, DET, imm=7)
    e('LDI', T0, imm=1040)
    e('JLT', a=T1, b=T0, imm='nm_inext')
    e('LDI', T0, imm=1)
    e('SHLL', T0, T0, imm=25)
    call('DIV')
    e('MOV', RR, T2)                    # 2^25 / (det >> 7) < 2^15

    def big(reg):
        """skip the numeral when |reg| >= 2^14 (a step past 4 texels a px:
        too small to read; the interlace doubling then stays in 16 bits)"""
        e('ABS', T0, reg)
        e('SHR', T0, T0, imm=14)
        e('JNZ', a=T0, imm='nm_inext')

    for dst, src, neg in ((DUX, VY, False), (DVX, UY, True), (DUY, VX, True), (DVY, UX, False)):
        mulsh(dst, src, RR, 0)
        e('SHR', dst, dst, imm=9)
        if neg:
            e('NEG', dst, dst)
        big(dst)
    # ---- the box's screen bounds: x span (window px), lines (field lines)
    for Uo, Vo, Oo, lo, hi in ((UX, VX, OX, SX0, SW), (UY, VY, OY, LY0, LY1)):
        mulsh(T0, Uo, NU, 0)            # the box's two edge vectors
        e('SHL', T1, Vo, imm=2)
        e('ADD', T2, T1, Vo)
        e('SHL', T1, T2, imm=2)         # 20 V
        e('MOV', lo, Oo)
        e('MOV', hi, Oo)
        for vec in (T0, T1):
            l1, l2 = L('bb'), L('bb')
            e('JLT', a=vec, b='zero', imm=l1)
            e('ADD', hi, hi, vec)
            e('JMP', imm=l2)
            a.label(l1)
            e('ADD', lo, lo, vec)
            a.label(l2)
    e('SHR', SX0, SX0, imm=11)
    e('SUB', SX0, SX0, 'one')
    e('MAX', SX0, SX0, 'zero')
    e('SHR', SW, SW, imm=11)
    e('LDI', T0, imm=2)
    e('ADD', SW, SW, T0)
    e('CTL', T0, imm=CTLR['bw'])
    e('SUB', T0, T0, 'one')
    e('MIN', SW, SW, T0)
    e('SUB', SW, SW, SX0)               # span: [sx0, sx0 + sw], the last pixel blank
    for reg in (LY0, LY1):
        e('SHR', reg, reg, imm=7)       # frame Q4
        e('SUB', reg, reg, YOFF)
        e('SHR', reg, reg, imm=4)
        e('SUB', T0, YST, 'one')
        l1, l2 = L('ly'), L('ly')
        e('JNZ', a=T0, imm=l2)
        e('JMP', imm=l1)
        a.label(l2)
        e('SHR', reg, reg, imm=1)
        a.label(l1)
    e('LDI', T0, imm=2)
    e('ADD', LY1, LY1, T0)
    # ---- texel coordinates at the span's first pixel centre on line ly0
    # (sample point relative to O, Q7), less half a texel (texel centres)
    e('SHL', T0, SX0, imm=11)
    e('LDI', T1, imm=1024)
    e('ADD', T0, T0, T1)
    e('SUB', T5, T0, OX)
    e('SHR', T5, T5, imm=4)             # dx, Q7
    e('SHL', T0, LY0, imm=4)
    e('LDI', T1, imm=8)
    e('ADD', T0, T0, T1)
    mulsh(T0, T0, YST, 0)
    e('ADD', T0, T0, YOFF)
    e('SHL', T0, T0, imm=7)
    e('SUB', T6, T0, OY)
    e('SHR', T6, T6, imm=4)             # dy, Q7
    U0, V0 = W1, RR                     # both dead here
    for dst, dx_, dy_ in ((U0, DUX, DUY), (V0, DVX, DVY)):
        mulsh(dst, dx_, T5, 0)
        mulsh(T0, dy_, T6, 0)
        e('ADD', dst, dst, T0)
        e('SHR', dst, dst, imm=7)
        e('LDI', T0, imm=2048)
        e('SUB', dst, dst, T0)
        e('SHR', dst, dst, imm=4)       # Q8 for the record
    # steps per FIELD line
    e('SUB', T0, YST, 'one')
    e('JNZ', a=T0, imm='nm_il')
    e('JMP', imm='nm_pr')
    a.label('nm_il')
    e('ADD', DUY, DUY, DUY)
    e('ADD', DVY, DVY, DVY)
    a.label('nm_pr')
    # ---- AA scale: px per texel, Q4 = sqrt(det) >> 7
    e('MOV', T0, DET)
    call('ISQRT')
    e('SHR', T2, T2, imm=7)
    e('LDI', T0, imm=16)                # at least 1 px a texel: the field only
    e('MAX', T2, T2, T0)                # reaches +0.875 texel outside the ink
    e('LDI', T0, imm=255)
    e('MIN', DET, T2, T0)
    # ---- the record (index = sprite + 4)
    e('LDI', T0, imm=4)
    e('ADD', T5, NSO, T0)
    e('SHL', T0, LY1, imm=11)
    e('ADD', T0, T0, LY0)
    e('SHLL', T1, GA, imm=22)
    e('ADD', T0, T0, T1)
    e('SHLL', T1, GB, imm=26)
    e('ADD', T0, T0, T1)
    e('SLW', a=T0, b=T5, imm=PORT['p0'])
    e('SHL', T0, SW, imm=9)
    e('ADD', T0, T0, SX0)
    e('SHLL', T1, DET, imm=17)
    e('ADD', T0, T0, T1)
    e('SLW', a=T0, b=T5, imm=PORT['p1'])
    for hi, lo, port in ((DVX, DUX, 'p2'), (DVY, DUY, 'p3'), (V0, U0, 'p5')):
        e('MOV', T0, hi)
        e('MOV', T1, lo)
        call('PACK')
        e('SLW', a=T2, b=T5, imm=PORT[port])
    # ---- the ink's colour step over this face: the ink lit like the face
    W4 = GA                             # dead now
    e('SHRL', T6, CF, imm=26)
    e('AND', T6, T6, imm=63)
    e('SHL', T6, T6, imm=3)             # g, 256 = 1.0
    e('ROM', T1, IKB)
    e('LDI', T0, imm=64)
    e('SUB', T1, T1, T0)
    mulsh(T1, T1, T6, 0)
    e('SHR', T1, T1, imm=8)
    e('ADD', T1, T1, T0)
    e('LDI', T0, imm=768)
    e('MIN', T1, T1, T0)
    e('AND', T0, CF, imm=1023)
    e('SUB', W4, T1, T0)
    e('AND', W4, W4, imm=2047)
    for i, sh, out in ((1, 10, 11), (2, 18, 20)):
        e('ADD', T1, IKB, 'one' if i == 1 else 'one')
        if i == 2:
            e('ADD', T1, T1, 'one')
        e('ROM', T1, T1)
        e('LDI', T0, imm=512)
        e('SUB', T1, T1, T0)
        mulsh(T1, T1, T6, 0)
        e('SHR', T1, T1, imm=8)
        e('ADD', T1, T1, T0)
        e('LDI', T0, imm=1023)
        e('CLP', T1, T1, T0)
        e('SHR', T1, T1, imm=2)
        e('SHRL', T0, CF, imm=sh)
        e('AND', T0, T0, imm=255)
        e('SUB', T1, T1, T0)
        e('AND', T1, T1, imm=511)
        e('SHLL', T1, T1, imm=out)
        e('ADD', W4, W4, T1)
    e('SLW', a=W4, b=T5, imm=PORT['p4'])
    e('ADD', NSO, NSO, 'one')
    e('LDI', T0, imm=12)
    e('JGE', a=NSO, b=T0, imm='nm_done')
    a.label('nm_inext')
    e('ADD', KK, KK, 'one')
    e('JLT', a=KK, b=NK, imm='nm_inst')
    a.label('nm_fnext')
    e('ADD', FI, FI, 'one')
    e('JLT', a=FI, b=NF, imm='nm_face')
    a.label('nm_done')
    e('SLW', a=NSO, b='zero', imm=PORT['ns'])

# ---------------------------------------------------- 7. backdrop, publish
# per field line luma step, Q16: (YBOT - YTOP) * 65536 / hf * ystep
e('CTL', T0, imm=CTLR['rhf'])
e('LDI', T1, imm=YBOT - YTOP)
mulsh(T0, T0, T1, 0)                    # Q20 / hf
e('SHR', T0, T0, imm=4)
mulsh(T0, T0, YST, 0)
e('SLW', a=T0, b='zero', imm=PORT['bgs'])
e('SLW', a=NEO, b='zero', imm=PORT['ne'])
e('SLW', a='zero', b='zero', imm=PORT['kick'])
e('END')


# ======================================================= engine ROM
INK_WHITE, INK_BLACK = (238, 236, 228), (14, 14, 16)     # the look model's inks
BLACK_INK = ("bone", "frost", "sunflower")               # light bodies get black ink
# numeral layout (the look model's): cap height (die units), and where the
# numeral sits -- TC: along UPV from P; d8/d20 from a small offset (die units)
LAYOUT = {"d4": dict(ink=0.22, tc=0.50, cond=1.0), "d6": dict(ink=0.50, tc=0.0, cond=1.0),
          "d8": dict(ink=0.42, off=-0.03, cond=1.0), "d10": dict(ink=0.30, tc=0.38, cond=0.92),
          "d20": dict(ink=0.26, off=-0.015, cond=0.80)}


def numeral_consts():
    """per die (TC Q12, SU, SV): c = P + UPV TC; a texel steps RT * SU / 2^19
    across and UPV * SV / 2^19 down (P, UPV, RT as the microcode builds them)"""
    import numpy as np
    from nat20_glyphs import FILL, TW, TH
    out = []
    for die in T.DICE:
        g = T.build_die(die)
        V, f = g['V'], g['faces'][0]['verts']
        if die in ('d4', 'd8', 'd20'):
            k = 0 if die == 'd4' else g['ups'][0]
            A, B, C = V[f[k]], V[f[(k + 1) % 3]], V[f[(k + 2) % 3]]
            upv, rt = np.linalg.norm(A - (A + B + C) / 3), np.linalg.norm(C - B)
        elif die == 'd6':
            k = g['ups'][0]
            P = [V[f[(k + i) % 4]] for i in range(4)]
            upv, rt = np.linalg.norm((P[0] + P[1] - P[2] - P[3]) / 4), np.linalg.norm(P[0] - P[1])
        else:
            t, sa, ff, sb = f
            upv, rt = np.linalg.norm(V[t] - V[ff]), np.linalg.norm(V[sb] - V[sa])
        lay = LAYOUT[die]
        cellh = lay['ink'] / FILL
        tw_d = cellh * TW / TH * lay['cond'] / TW         # die units a texel, across
        th_d = cellh / TH                                  # and down
        tc = lay.get('tc', lay.get('off', 0.0) / upv)
        su, sv = round(tw_d / rt * 2 ** 19), round(th_d / upv * 2 ** 19)
        assert su < 32768 and sv < 32768, (die, su, sv)
        out.append((round(tc * 4096), su, sv))
    return out


def engine_rom():
    rom = [0] * 256
    for i in range(65):                                    # quarter sine, Q12
        rom[ROM_SIN + i] = int(round(4096 * math.sin(math.pi / 2 * i / 64)))
    third = (1365, 1365, 1366, 0)
    shapes = (third + (2731, -1365, -1366, 0) + (0, -4096, 4096, 0),     # tri A B C
              (1024,) * 4 + (1024, 1024, -1024, -1024) + (4096, -4096, 0, 0),   # square
              (0, 0, 0, 4096) + (4096, 0, 0, -4096) + (0, -4096, 4096, 0))   # kite T Sa Sb F
    for i, w in enumerate(sum(shapes, ())):                  # corner weights
        rom[ROM_SHW + i] = w & 0xFFFF
    for d, (tc, su, sv) in enumerate(numeral_consts()):     # numeral frames
        rom[ROM_NUM + 3 * d: ROM_NUM + 3 * d + 3] = [tc & 0xFFFF, su, sv]
    for i, rgb in enumerate((INK_WHITE, INK_BLACK)):
        rom[ROM_INK + 3 * i: ROM_INK + 3 * i + 3] = list(T.yuv10(rgb))
    rom[ROM_INKM] = sum(1 << T.COLOR_ORDER.index(c) for c in BLACK_INK)
    for i in range(128):                                   # face gamma, 256 = 1.0
        lin = i / 96.0
        rom[ROM_GAM + i] = int(round(256 * lin ** (1 / 2.2)))
    return rom


def build():
    return a.words(), engine_rom()


if __name__ == '__main__':
    w, rom = build()
    print(f'-- {len(w)} instructions, peak {a.peak} registers', file=sys.stderr)
    assert len(w) <= 1024
