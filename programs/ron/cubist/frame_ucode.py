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
from uasm import Asm, emit_vhdl

R_BAS, R_H, R_HB, R_CX, R_CY = 0, 9, 12, 18, 26
R_TRIG, R_ANG = 34, 40
J, I, C = 43, 44, 45
T0, T1, T2, T3, T4, T5 = 46, 47, 48, 49, 50, 51
ANG, SK, SNEG, S0, S1, KK = 52, 53, 54, 55, 56, 57
ZERO, ONE = 58, 59

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

a.emit('END')

if __name__ == '__main__':
    w = a.words()
    print(f'-- {len(w)} instructions', file=sys.stderr)
    print(emit_vhdl(w, depth=512))
