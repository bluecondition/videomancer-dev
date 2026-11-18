#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NEON BEACH engine: assembler, emulator and the per-field program.

One small datapath runs once a field (started in vertical blanking, free to run on through the field: the knot
table is double-buffered): a 16-bit ALU, sixteen 16-bit registers (r0 reads 0) kept in the top of the scratch
block RAM (496 .. 511, read one operand at a time: no register-file multiplexers), a serial 16 x 8 multiplier,
the program in block RAM.  Ten clocks an instruction (plus the multiplier's wait in MRD); every enable and
select is decoded into a register two or more clocks before it is used.

Instruction, 24 bits:  [23:19] op  [18:15] d  [14:11] a  [10:7] b  [10:0] imm11 (sign-extended where it is a
value)  LDI: [14:0] imm15 (signed)  jumps: [8:0] target

  NOP                      LDI d,imm   d = imm             ADD d,a,b   d = a + b       SUB d,a,b   d = a - b
  ADDI d,a,imm  d = a + imm            SHR d,a,n   d = a >> n (logical, n 1..4)        SHL d,a,n   d = a << n (n 1..4)
                (the assembler splits a shift of 5 .. 8 in two)
  AND d,a,imm   d = a & imm            MUL a,b     start a * b[7:0] (unsigned, 8 clocks)
  MRD d,s       d = product >> {0,6,7,3}[s] (waits for the multiplier)
  JMP t         JN a,t  if a < 0      JZ a,t  if a == 0       JP a,t  if a > 0
  IN d,p        d = input port p       OUT a,p     output port p = a
  MIN d,a,b  MAX d,a,b  (signed)        ABS d,a
  SHP d,a       d = wave shape[a & 511] (signed)
  TAB d,a,f     f 0: d = depth table[a & 15] (shoal << 3 | class)   f 1: d = wave size[a & 15]
  FRD d,a       d = scratch[a & 511]   FWR a,b   scratch[a & 511] = b
                (scratch: 0 .. 127 foam per knot, 128 .. 255 phase offset per knot, 256 .. 383 bed per knot,
                 384 .. variables; the offsets and the bed are its initial contents)
  KST a,f       knot word field f = a  (0 surface, 1 its step, 2 bed, 3 its step, 4 foam, 5 water flag)
  KWR a         knot table (the bank not on screen) [a & 127] = the staged word
  TRI d,a       d = triangle of a[15:8] (0 .. 127)
  END           done for this field
"""
import math

OPS = {'NOP': 0, 'LDI': 1, 'ADD': 2, 'SUB': 3, 'ADDI': 4, 'SHR': 5, 'SHL': 6, 'AND': 7, 'MUL': 8, 'MRD': 9,
       'JMP': 10, 'JN': 11, 'JZ': 12, 'IN': 13, 'RDB': 14, 'SHP': 15, 'TAB': 16, 'FRD': 17, 'FWR': 18,
       'KST': 19, 'KWR': 20, 'OUT': 21, 'END': 22, 'TRI': 23, 'JP': 24, 'MIN': 25, 'MAX': 26, 'ABS': 27}

# input ports
P_K1, P_K3, P_K4, P_K5, P_SL, P_T, P_RND, P_COFF = 0, 1, 2, 3, 4, 5, 6, 7
# output ports: swash envelope, shoreline knot; the crab: centre x (virtual px from the canvas's left edge),
# top row, sink (0 .. 96 rows into its hole) | frame << 7, its hole's wetness cell (row << 6 | column)
O_E, O_KS, O_CX, O_CY, O_CS, O_CH = 0, 1, 2, 3, 4, 5
# knot word fields
K_SRF, K_DS, K_BED, K_DB, K_FOAM, K_W = 0, 1, 2, 3, 4, 5
MRD_SHIFT = [0, 6, 7, 3]


class Asm:
    def __init__(self):
        self.code, self.labels, self.fix, self.n = [], {}, [], 0

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def tmp(self):
        self.n += 1
        return "_L%d" % self.n

    def emit(self, op, d=0, a=0, b=0, imm=0, target=None):
        if target is not None:
            self.fix.append((len(self.code), target))
        assert 0 <= d < 16 and 0 <= a < 16 and 0 <= b < 16, (op, d, a, b)
        self.code.append([OPS[op], d, a, b, imm])

    # instructions
    def LDI(s, d, v): assert -16384 <= v < 16384; s.emit('LDI', d=d, imm=v)
    def ADD(s, d, a, b): s.emit('ADD', d, a, b)
    def SUB(s, d, a, b): s.emit('SUB', d, a, b)
    def ADDI(s, d, a, v): assert -1024 <= v < 1024; s.emit('ADDI', d, a, imm=v)
    def SHR(s, d, a, n):
        assert 1 <= n <= 8
        if n > 4:
            s.emit('SHR', d, a, imm=3); a, n = d, n - 4
        s.emit('SHR', d, a, imm=n - 1)

    def SHL(s, d, a, n):
        assert 1 <= n <= 8
        if n > 4:
            s.emit('SHL', d, a, imm=3); a, n = d, n - 4
        s.emit('SHL', d, a, imm=n - 1)
    def AND(s, d, a, v): assert 0 <= v < 2048; s.emit('AND', d, a, imm=v)
    def MUL(s, a, b): s.emit('MUL', a=a, b=b)
    def MRD(s, d, sh): s.emit('MRD', d=d, imm=MRD_SHIFT.index(sh))
    def JMP(s, t): s.emit('JMP', target=t)
    def JN(s, a, t): s.emit('JN', a=a, target=t)
    def JZ(s, a, t): s.emit('JZ', a=a, target=t)
    def JP(s, a, t): s.emit('JP', a=a, target=t)
    def IN(s, d, p): s.emit('IN', d=d, imm=p)
    def OUT(s, a, p): s.emit('OUT', a=a, imm=p)
    def SHP(s, d, a): s.emit('SHP', d, a)
    def TAB(s, d, a, f): s.emit('TAB', d, a, imm=f)
    def FRD(s, d, a): s.emit('FRD', d, a)
    def FWR(s, a, b): s.emit('FWR', a=a, b=b)
    def KST(s, a, f): s.emit('KST', a=a, imm=f)
    def KWR(s, a): s.emit('KWR', a=a)
    def TRI(s, d, a): s.emit('TRI', d, a)
    def END(s): s.emit('END')

    # macros (r0 reads 0)
    def MOV(s, d, a): s.ADD(d, a, 0)
    def NEG(s, d, a): s.SUB(d, 0, a)

    def ABS(s, d, a): s.emit('ABS', d, a)
    def MIN(s, d, a, b, t=None): s.emit('MIN', d, a, b)
    def MAX(s, d, a, b, t=None): s.emit('MAX', d, a, b)

    def assemble(s):
        for i, t in s.fix:
            s.code[i][4] = s.labels[t]
        words = []
        for op, d, a, b, imm in s.code:
            if op == OPS['LDI']:
                w = (op << 19) | (d << 15) | (imm & 0x7FFF)
            elif op in (OPS['JMP'], OPS['JN'], OPS['JZ'], OPS['JP']):
                w = (op << 19) | (a << 11) | (imm & 0x1FF)
            elif op in (OPS['ADD'], OPS['SUB'], OPS['MUL'], OPS['FWR'], OPS['MIN'], OPS['MAX']):
                w = (op << 19) | (d << 15) | (a << 11) | (b << 7)
            else:
                w = (op << 19) | (d << 15) | (a << 11) | (imm & 0x7FF)
            words.append(w)
        assert len(words) <= 512, len(words)
        return words


# ---------------------------------------------------------------------------------------------------------
# The program
# ---------------------------------------------------------------------------------------------------------
# persistent registers
R_P, R_PF, R_E, R_RT = 1, 2, 3, 4          # wave phase (4096 = a wave), its fraction, swash envelope, rise|target
# per-field
R_SL, R_K, R_KS = 5, 6, 7                  # sea level (with the surge), the knot, the shoreline knot (128 = none)
R_SN, R_BN = 8, 9                          # the previous (right-hand) knot's surface and bed
T0, T1, T2, T3, T4, T5 = 10, 11, 12, 13, 14, 15
# scratch RAM
S_FOAM, S_SOFF, S_BEDT = 0, 128, 256    # per knot: last field's foam, the phase offset, the bed
S_NSH, S_SZSH, S_NPREV, S_FIRST, S_D, S_CAP, S_EXC, S_TS, S_SRF, S_BED, S_PH = range(384, 395)
S_CX, S_CY, S_CS, S_CT, S_CD, S_CF, S_CW = range(400, 407)      # crab: x, y, sink, timer, step, frame clock, walking


def program():
    a = Asm()
    a.label('field')
    # ---- the phase: dP = 96 + Rhythm / 2, in 16ths of a phase unit ----
    a.IN(T0, P_K1); a.SHR(T0, T0, 1); a.ADDI(T0, T0, 96)
    a.ADD(T1, R_PF, T0); a.SHR(T2, T1, 4); a.AND(R_PF, T1, 15); a.ADD(R_P, R_P, T2)
    # ---- the surge: sg = (Swell >> 5) x (tri(2P) - 64) >> 6, signed ----
    a.SHL(T0, R_P, 1); a.TRI(T0, T0); a.ADDI(T0, T0, -64)
    a.ABS(T1, T0)
    a.IN(T2, P_K5); a.SHR(T2, T2, 5)
    a.MUL(T1, T2); a.MRD(T1, 6)
    a.IN(R_SL, P_SL)
    a.JN(T0, 'sg_up')
    a.SUB(R_SL, R_SL, T1); a.JMP('sg_done')
    a.label('sg_up')
    a.ADD(R_SL, R_SL, T1)
    a.label('sg_done')
    # ---- the knot loop, from the right edge ----
    a.LDI(R_K, 127); a.LDI(R_KS, 128)
    a.LDI(T0, 1); a.LDI(T1, S_FIRST); a.FWR(T1, T0)
    a.label('knot')
    a.ADDI(T0, R_K, S_BEDT); a.FRD(T0, T0)             # bed
    a.LDI(T1, S_BED); a.FWR(T1, T0)
    a.SUB(T1, T0, R_SL)                                # d = bed - sea level
    a.LDI(T2, S_D); a.FWR(T2, T1)
    a.ADDI(T2, R_K, S_SOFF); a.FRD(T2, T2)             # phase offset
    a.SUB(T2, R_P, T2)                                 # Pk = P - offset
    a.SHR(T3, T2, 5); a.AND(T3, T3, 127)               # ph = Pk[11:5]
    a.LDI(T4, S_PH); a.FWR(T4, T3)
    a.SHR(T2, T2, 8); a.SHR(T2, T2, 4)                 # n = Pk[15:12]
    a.TAB(T3, T2, 1)                                   # size
    a.JP(T1, 'water')
    # -- the beach: the first beach knot is the shoreline --
    a.LDI(T4, 128); a.SUB(T4, R_KS, T4); a.JN(T4, 'beach_sheet')
    a.MOV(R_KS, R_K)
    a.LDI(T4, S_NSH); a.FWR(T4, T2)
    a.LDI(T4, S_SZSH); a.FWR(T4, T3)
    a.label('beach_sheet')
    # th = clamp((e - 8 (ks - k)) >> 1, 0, 14); surface = bed - th
    a.SUB(T4, R_KS, R_K); a.SHL(T4, T4, 3)
    a.AND(T5, R_E, 255); a.SUB(T4, T5, T4)
    a.LDI(T5, 0); a.JN(T4, 'th_done')
    a.SHR(T5, T4, 1); a.LDI(T4, 14); a.MIN(T5, T5, T4, T2)
    a.label('th_done')
    a.SUB(T0, T0, T5)                                  # surface
    a.LDI(T4, 0); a.LDI(T5, S_EXC); a.FWR(T5, T4)
    a.JMP('foam')
    # -- water --
    a.label('water')
    # cap = 3 d / 8; depth index i = min(15, d >> 4); shoal, class
    a.SHR(T4, T1, 2); a.SHR(T5, T1, 3); a.ADD(T4, T4, T5)
    a.LDI(T5, S_CAP); a.FWR(T5, T4)
    a.SHR(T4, T1, 4); a.LDI(T5, 15); a.MIN(T4, T4, T5, T0)
    a.TAB(T4, T4, 0)                                   # shoal << 3 | class
    a.SHR(T5, T4, 3)                                   # shoal (eighths)
    a.AND(T4, T4, 7)                                   # class
    # A = A0 x size x shoal / 64,  A0 = 8 + Swell/16 + Swell/64
    a.MUL(T3, T5); a.MRD(T3, 0)                        # size x shoal
    a.IN(T5, P_K5); a.SHR(T0, T5, 4); a.SHR(T5, T5, 6); a.ADD(T5, T5, T0); a.ADDI(T5, T5, 8)
    a.MUL(T5, T3); a.MRD(T3, 6)                        # A
    # the depth caps it: the excess breaks into foam, the shape turns into a breaker (class 3)
    a.LDI(T5, S_CAP); a.FRD(T5, T5)
    a.SUB(T0, T3, T5)
    a.LDI(T1, 0)
    a.JN(T0, 'nocap')
    a.MOV(T1, T0); a.MOV(T3, T5); a.LDI(T4, 3)
    a.label('nocap')
    a.LDI(T0, S_EXC); a.FWR(T0, T1)
    # the profile: dev = A x shape[class, ph] / 64 (signed)
    a.SHL(T4, T4, 7); a.LDI(T0, S_PH); a.FRD(T0, T0); a.ADD(T4, T4, T0)
    a.SHP(T4, T4)
    a.ABS(T0, T4)
    a.MUL(T3, T0); a.MRD(T3, 6)                        # |dev|
    a.JN(T4, 'dev_neg')
    a.SUB(T3, R_SL, T3); a.JMP('dev_done')             # crest side: up = a smaller row
    a.label('dev_neg')
    a.ADD(T3, R_SL, T3)
    a.label('dev_done')
    # chop: two short waves (8 and 4 knots), gain Chop / 32, clamped to d / 4
    a.SHL(T0, R_P, 5); a.SHL(T1, R_K, 8); a.SHL(T1, T1, 5); a.SUB(T0, T0, T1); a.TRI(T0, T0)
    a.SHL(T1, R_P, 1); a.ADD(T1, T1, R_P); a.SHL(T1, T1, 5)
    a.SHL(T2, R_K, 8); a.SHL(T2, T2, 6); a.SUB(T1, T1, T2); a.TRI(T1, T1)
    a.ADD(T0, T0, T1); a.ADDI(T0, T0, -128)            # -128 .. 126
    a.ABS(T1, T0)
    a.IN(T2, P_K4); a.SHR(T2, T2, 5)
    a.MUL(T1, T2); a.MRD(T1, 7)
    a.LDI(T2, S_D); a.FRD(T2, T2); a.SHR(T2, T2, 2); a.MIN(T1, T1, T2, T4)
    a.JN(T0, 'chop_neg')
    a.SUB(T3, T3, T1); a.JMP('chop_done')
    a.label('chop_neg')
    a.ADD(T3, T3, T1)
    a.label('chop_done')
    # never above the top, never below the bed
    a.LDI(T0, S_BED); a.FRD(T0, T0)
    a.MIN(T3, T3, T0, T1)
    a.MAX(T0, T3, 0, T1)
    # ---- foam: breaking, whitecaps on steep fronts, the shoreline; 5 a field of decay ----
    a.label('foam')                                    # T0 = this knot's surface
    a.LDI(T1, S_SRF); a.FWR(T1, T0)
    a.IN(T5, P_K3); a.SHR(T5, T5, 6)                   # Foam gain 0 .. 15
    a.LDI(T1, S_EXC); a.FRD(T1, T1)
    a.MUL(T1, T5); a.MRD(T1, 0); a.SHR(T1, T1, 1)
    a.LDI(T2, 255); a.MIN(T1, T1, T2, T3)              # breaking
    a.LDI(T2, S_FIRST); a.FRD(T2, T2); a.JP(T2, 'no_wc')
    a.SUB(T2, T0, R_SN); a.ADDI(T2, T2, -10)           # whitecap: (srf - srf[k+1] - 10) x gain
    a.JN(T2, 'no_wc'); a.JZ(T2, 'no_wc')
    a.LDI(T3, 255); a.MIN(T2, T2, T3, T4)
    a.MUL(T2, T5); a.MRD(T2, 0)
    a.LDI(T3, 255); a.MIN(T2, T2, T3, T4)
    a.MAX(T1, T1, T2, T3)
    a.label('no_wc')
    a.SUB(T2, R_KS, R_K); a.ADDI(T2, T2, -2)           # the shoreline: ks - k <= 1
    a.JP(T2, 'no_shore'); a.JZ(T2, 'no_shore')
    a.LDI(T2, 128); a.SUB(T2, R_KS, T2); a.JP(T2, 'no_shore'); a.JZ(T2, 'no_shore')
    a.SHL(T2, T5, 3); a.MAX(T1, T1, T2, T3)
    a.label('no_shore')
    a.FRD(T2, R_K); a.ADDI(T2, T2, -5); a.MAX(T2, T2, 0, T3)
    a.MAX(T1, T1, T2, T3)                              # the foam
    a.FWR(R_K, T1)
    # ---- write the knot ----
    a.LDI(T2, S_BED); a.FRD(T2, T2)
    a.KST(T0, K_SRF); a.KST(T2, K_BED); a.KST(T1, K_FOAM)
    a.LDI(T3, S_FIRST); a.FRD(T3, T3)
    a.LDI(T4, 0); a.LDI(T5, 0)
    a.JP(T3, 'first')
    a.SUB(T4, R_SN, T0); a.SUB(T5, R_BN, T2)
    a.label('first')
    a.KST(T4, K_DS); a.KST(T5, K_DB)
    a.LDI(T3, S_D); a.FRD(T3, T3); a.LDI(T4, 0); a.JN(T3, 'dry'); a.JZ(T3, 'dry'); a.LDI(T4, 1)
    a.label('dry')
    a.KST(T4, K_W)
    a.KWR(R_K)
    a.MOV(R_SN, T0); a.MOV(R_BN, T2)
    a.LDI(T3, S_FIRST); a.FWR(T3, 0)
    a.JZ(R_K, 'pass_done')
    a.ADDI(R_K, R_K, -1)
    a.JMP('knot')
    # ---- the end of the pass: a crest at the shoreline starts a run-up; the envelope ----
    a.label('pass_done')
    a.LDI(T0, 128); a.SUB(T0, R_KS, T0); a.JP(T0, 'env'); a.JZ(T0, 'env')       # no shoreline: no swash
    a.LDI(T1, S_NSH); a.FRD(T1, T1); a.LDI(T2, S_NPREV); a.FRD(T3, T2); a.FWR(T2, T1)
    a.SUB(T3, T1, T3); a.JZ(T3, 'env')
    # target = min(255, 48 + Swell / 8 + size x 8); rise flag = bit 8 of R_RT
    a.IN(T0, P_K5); a.SHR(T0, T0, 3); a.ADDI(T0, T0, 48)
    a.LDI(T1, S_SZSH); a.FRD(T1, T1); a.SHL(T1, T1, 3); a.ADD(T0, T0, T1)
    a.LDI(T1, 255); a.MIN(T0, T0, T1, T2)
    a.ADDI(R_RT, T0, 256)
    a.label('env')
    a.SHR(T0, R_RT, 8); a.JZ(T0, 'falling')
    a.ADDI(R_E, R_E, 6); a.LDI(T1, 255); a.MIN(R_E, R_E, T1, T2)                # rising: e += 6 (to 255)
    a.AND(T1, R_RT, 255); a.SUB(T1, R_E, T1); a.JN(T1, 'out')                    # until e >= target
    a.AND(R_RT, R_RT, 255); a.JMP('out')
    a.label('falling')
    a.ADDI(R_E, R_E, -2); a.MAX(R_E, R_E, 0, T1)
    a.label('out')
    a.OUT(R_E, O_E); a.OUT(R_KS, O_KS)
    # ---- the crab ----
    # it lives on the sand between x 280 and 800 (its centre), its top row from 640 (it runs down to 960); it walks and pauses
    # at random, runs down the sand when the wash reaches its column and depth, sinks into a hole when switched
    # off and comes up somewhere new when switched on
    a.LDI(T5, S_CS); a.FRD(R_SL, T5)                   # sink (R_SL is free now)
    a.IN(T0, P_COFF); a.JZ(T0, 'crab_on')
    a.ADDI(R_SL, R_SL, 3); a.LDI(T1, 96); a.MIN(R_SL, R_SL, T1)   # switched off: sink (to 96)
    a.JMP('crab_out')
    a.label('crab_on')
    a.JZ(R_SL, 'crab_live')
    a.LDI(T1, 96); a.SUB(T1, R_SL, T1); a.JN(T1, 'crab_rise')
    # fully hidden and switched on: a new place
    a.IN(T0, P_RND); a.AND(T1, T0, 511); a.ADDI(T1, T1, 290)
    a.LDI(T2, 790); a.MIN(T1, T1, T2); a.LDI(T2, S_CX); a.FWR(T2, T1)
    a.SHR(T1, T0, 8); a.SHR(T1, T1, 1); a.AND(T1, T1, 127); a.LDI(T2, 640); a.ADD(T1, T1, T2)
    a.LDI(T2, S_CY); a.FWR(T2, T1)
    a.label('crab_rise')
    a.ADDI(R_SL, R_SL, -3); a.MAX(R_SL, R_SL, 0)
    a.JMP('crab_out')
    a.label('crab_live')
    a.LDI(T5, S_CX); a.FRD(R_K, T5)                    # x (R_K is free now)
    a.LDI(T5, S_CY); a.FRD(R_SN, T5)                   # y
    # the wash at its column?  front column = (ks - e / 8) / 2, its column = x / 32, the wash to e rows deep
    a.SHR(T0, R_K, 5)
    a.SHR(T1, R_E, 3); a.SUB(T1, R_KS, T1); a.SHR(T1, T1, 1)
    a.SUB(T1, T0, T1); a.JN(T1, 'no_flee')             # left of the front
    a.SHL(T1, T0, 1); a.SUB(T1, T1, R_KS); a.JP(T1, 'no_flee'); a.JZ(T1, 'no_flee')     # under the water side
    a.SHR(T2, R_K, 4); a.ADDI(T2, T2, S_BEDT); a.FRD(T2, T2)       # the bed at its knot
    a.SUB(T2, R_SN, T2)                                # its depth under the sand line
    a.AND(T3, R_E, 255); a.ADDI(T3, T3, 48); a.SUB(T3, T2, T3); a.JP(T3, 'no_flee')
    a.ADDI(R_SN, R_SN, 6); a.LDI(T3, 960); a.MIN(R_SN, R_SN, T3)   # flee: down the sand, legs going fast
    a.LDI(T5, S_CF); a.FRD(T3, T5); a.ADDI(T3, T3, 3); a.FWR(T5, T3)
    a.JMP('crab_keep')
    a.label('no_flee')
    # the timer: a new choice when it runs out (walk left, walk right, pause), 0.5 .. 1.6 s
    a.LDI(T5, S_CT); a.FRD(T0, T5); a.ADDI(T0, T0, -1); a.FWR(T5, T0)
    a.JP(T0, 'crab_move')
    a.IN(T1, P_RND); a.AND(T2, T1, 63); a.ADDI(T2, T2, 30); a.FWR(T5, T2)
    a.SHR(T2, T1, 6); a.AND(T2, T2, 3)                 # 0, 1 left, 2 right, 3 pause
    a.LDI(T3, 0); a.JZ(T2, 'crab_dir')
    a.ADDI(T4, T2, -3); a.JZ(T4, 'crab_dir')
    a.LDI(T3, -2); a.ADDI(T4, T2, -1); a.JZ(T4, 'crab_dir')
    a.LDI(T3, 2)
    a.label('crab_dir')
    a.LDI(T5, S_CD); a.FWR(T5, T3)
    a.label('crab_move')
    a.LDI(T5, S_CD); a.FRD(T3, T5)
    a.ADD(R_K, R_K, T3)
    a.LDI(T4, 280); a.SUB(T4, R_K, T4); a.JP(T4, 'cx_lo'); a.LDI(T3, 2); a.FWR(T5, T3); a.LDI(R_K, 280)
    a.label('cx_lo')
    a.LDI(T4, 800); a.SUB(T4, R_K, T4); a.JN(T4, 'cx_hi'); a.LDI(T3, -2); a.FWR(T5, T3); a.LDI(R_K, 800)
    a.label('cx_hi')
    a.JZ(T3, 'crab_keep')                              # paused: the legs rest
    a.LDI(T5, S_CF); a.FRD(T3, T5); a.ADDI(T3, T3, 1); a.FWR(T5, T3)
    a.label('crab_keep')
    a.LDI(T5, S_CX); a.FWR(T5, R_K)
    a.LDI(T5, S_CY); a.FWR(T5, R_SN)
    a.label('crab_out')
    a.LDI(T5, S_CS); a.FWR(T5, R_SL)
    a.LDI(T5, S_CX); a.FRD(T0, T5); a.OUT(T0, O_CX)
    a.SHR(T1, T0, 5); a.AND(T1, T1, 63)                # its hole's cell: column x / 32,
    a.LDI(T5, S_CY); a.FRD(T2, T5); a.OUT(T2, O_CY)
    a.SHR(T3, T0, 4); a.ADDI(T3, T3, S_BEDT); a.FRD(T3, T3)
    a.ADDI(T2, T2, 90); a.SUB(T2, T2, T3); a.MAX(T2, T2, 0); a.SHR(T2, T2, 5); a.LDI(T3, 31); a.MIN(T2, T2, T3)
    a.SHL(T2, T2, 6); a.ADD(T1, T1, T2); a.OUT(T1, O_CH)            # row (its foot's depth) / 32
    a.LDI(T5, S_CF); a.FRD(T0, T5); a.SHR(T0, T0, 3); a.AND(T0, T0, 1); a.SHL(T0, T0, 7)
    a.ADD(T0, T0, R_SL); a.OUT(T0, O_CS)
    a.END()
    return a


def ucode():
    return program().assemble()


# ---------------------------------------------------------------------------------------------------------
# Emulator (16-bit registers, the hardware's semantics)
# ---------------------------------------------------------------------------------------------------------
def s16(v):
    v &= 0xFFFF
    return v - 65536 if v & 0x8000 else v


def tri7(v8):
    v8 &= 255
    return (v8 ^ 127) & 127 if v8 & 128 else v8 & 127


class Engine:
    def __init__(self, words, tables):
        self.w = words
        self.t = tables                      # dict: bed, soff, dtab, size, shape (signed)
        self.r = [0] * 16
        self.scr = [0] * 512
        self.scr[S_CX], self.scr[S_CY], self.scr[S_CT] = 540, 700, 40
        for k in range(128):
            self.scr[S_SOFF + k] = tables['soff'][k]
            self.scr[S_BEDT + k] = tables['bed'][k]
        self.kst = [0] * 6
        self.knots = {}
        self.out = [0] * 16
        self.prod = 0
        self.clocks = 0

    def run(self, ports):
        pc = 0
        steps = 0
        while True:
            w = self.w[pc]
            op, d, a, b = (w >> 19) & 31, (w >> 15) & 15, (w >> 11) & 15, (w >> 7) & 15
            i11 = w & 0x7FF
            s11 = i11 - 2048 if i11 & 0x400 else i11
            i15 = w & 0x7FFF
            s15 = i15 - 32768 if i15 & 0x4000 else i15
            tgt = w & 0x1FF
            A, B = self.r[a] if a else 0, self.r[b] if b else 0
            npc = pc + 1
            res = None
            self.clocks += 10
            if op == OPS['NOP']: pass
            elif op == OPS['LDI']: res = s15
            elif op == OPS['ADD']: res = A + B
            elif op == OPS['SUB']: res = A - B
            elif op == OPS['ADDI']: res = A + s11
            elif op == OPS['SHR']: res = (A & 0xFFFF) >> ((i11 & 3) + 1)
            elif op == OPS['SHL']: res = A << ((i11 & 3) + 1)
            elif op == OPS['AND']: res = A & i11
            elif op == OPS['MUL']: self.prod = (A & 0xFFFF) * (B & 0xFF); self.clocks += 8
            elif op == OPS['MRD']: res = self.prod >> MRD_SHIFT[i11 & 3]
            elif op == OPS['JMP']: npc = tgt
            elif op == OPS['JN']: npc = tgt if s16(A) < 0 else npc
            elif op == OPS['JZ']: npc = tgt if (A & 0xFFFF) == 0 else npc
            elif op == OPS['JP']: npc = tgt if s16(A) > 0 else npc
            elif op == OPS['IN']: res = ports[i11 & 15]
            elif op == OPS['OUT']: self.out[i11 & 15] = A & 0xFFFF
            elif op == OPS['SHP']: res = self.t['shape'][A & 511]
            elif op == OPS['TAB']: res = (self.t['dtab'] if (i11 & 1) == 0 else self.t['size'])[A & 15]
            elif op == OPS['FRD']: res = self.scr[A & 511]
            elif op == OPS['FWR']: self.scr[A & 511] = B & 0xFFFF
            elif op == OPS['KST']: self.kst[i11 & 7] = A & 0xFFFF
            elif op == OPS['KWR']: self.knots[A & 127] = list(self.kst)
            elif op == OPS['TRI']: res = tri7((A & 0xFFFF) >> 8)
            elif op == OPS['MIN']: res = A if s16(A) <= s16(B) else B
            elif op == OPS['MAX']: res = A if s16(A) >= s16(B) else B
            elif op == OPS['ABS']: res = -s16(A) if s16(A) < 0 else A
            elif op == OPS['END']: return self.knots
            else: raise ValueError("op %d at %d" % (op, pc))
            if res is not None and d:
                self.r[d] = res & 0xFFFF
            pc = npc
            steps += 1
            assert steps < 100000, "runaway"
