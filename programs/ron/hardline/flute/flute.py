#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FLUTE v0.1.0 (Hardline) - build hook.  Writes flute_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.hardline_work/rb_flute/imodel.py) imports this file and
runs the same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs unless noted):
  C_FL_LOGV  256 x 16   round(16384 * log2(1 + i/256))                 pixel path: log2 mantissa, Q14
  C_FL_LOGS  256 x 16   its first difference (7 bits)
  C_FL_UG    256 x 16   round(16384 * 0.5 * log2(1 + 4^-(i/16)))       pixel path: log2 r - log2 hi, Q14
  C_FL_UGS   256 x 16   its first difference (negative, two's complement)
  C_FL_EXM   256 x 16   round(16384 * 2^(i/256))                       pixel path: exp2 mantissa, Q14
  C_FL_EXS   256 x 16   its first difference
  C_FL_HT    64 x 8     round(64 * 0.5 * log2(1 + 2^(-2 * 4k / 64)))   fabric: gradient combine
  C_FL_FT    512 x 16   the per-field engine's tables: 0 .. 256 a quarter sine (32768 sin(i pi / 512)),
                        257 .. 385 exp (16384 * 2^(j/128)), 386 .. 391 2^3 .. 2^8, 392 .. 407 log2 mantissa
  C_FL_UC    1024 x 16  the per-field microprogram (see PROGRAM below)
  C_FL_RFI   256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults)

The per-field engine: a 37-bit accumulator, a 64 x 32-bit register file, one adder, a serial multiplier
(18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode + 12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

_LV = [int(round(16384 * math.log2(1 + i / 256.0))) for i in range(257)]
LOGV = _LV[:256]
LOGS = [_LV[i + 1] - _LV[i] for i in range(256)]
_G = [int(round(16384 * 0.5 * math.log2(1 + 4.0 ** (-k / 16.0)))) for k in range(257)]
UG = _G[:256]
UGS = [_G[k + 1] - _G[k] for k in range(256)]            # <= 0
_EM = [int(round(16384 * 2 ** (i / 256.0))) for i in range(257)]
EXM = _EM[:256]
EXS = [_EM[i + 1] - _EM[i] for i in range(256)]
HT = [int(round(64 * 0.5 * math.log2(1 + 2.0 ** (-2 * (4 * k) / 64.0)))) for k in range(64)]
LG = [int(round(64 * math.log2(1 + (i + 0.5) / 16.0))) for i in range(16)]

# the engine's table ROM
SINB, EXPB, P2B, LGB = 0, 257, 386, 392
FT = [int(round(32768 * math.sin(i * math.pi / 512.0))) for i in range(257)]
FT += [int(round(16384 * 2 ** (j / 128.0))) for j in range(129)]
FT += [1 << k for k in range(3, 9)]
FT += LG
FT += [0] * (512 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=784, k4=661, k5=512, k6=1023, p12=802)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, WL=8, H=9, YSTP=10, ROM=11, CNT=12, LS14=13, LS12=14, LS6=15)
CONDS = dict(LT=0, GLIDE=1, STRIPES=2, SRC=3, GE=8, NGLIDE=9, NSTRIPES=10, NSRC=11)
OUTS = dict(ADR=0, EOFF=1, KC=2, LFP=3, LP=4, AS=5, BS=6, WQ4=7, T3=8, R7=9, NSH=10, UINC=11, UINCW=12,
            QINC=13, QINCW=14, VINC=15, SY=16, QLINE=17, VLINE=18, QJUMP=19, LU=20, LQ=21, LV=22, G=23)


class Asm:
    def __init__(self):
        self.code = []          # (op, arg or label name)
        self.labels = {}
        self.regs = {}
        self.init = {}
        self.nl = 0

    def reg(self, name, init=0):
        if name not in self.regs:
            self.regs[name] = len(self.regs)
            assert len(self.regs) <= 64
            self.init[name] = init
        return self.regs[name]

    def new(self):
        self.nl += 1
        return "_L%d" % self.nl

    def label(self, name):
        assert name not in self.labels
        self.labels[name] = len(self.code)

    def emit(self, op, arg=0):
        self.code.append((op, arg))

    def __getattr__(self, op):
        if op in ("LD", "ADD", "SUB", "MUL", "ST"):
            return lambda r: self.emit(op, self.reg(r))
        if op in ("LDI", "ADDI", "SUBI", "MULI"):
            def f(i):
                assert -2048 <= i <= 2047, i
                self.emit(op, i & 0xFFF)
            return f
        if op == "IN":
            return lambda n: self.emit(op, INS[n])
        if op == "OUT":
            return lambda n: self.emit(op, OUTS[n])
        if op == "SHR":
            def f(n):
                assert 1 <= n <= 31
                self.emit(op, n)
            return f
        if op == "SKIP":
            return lambda c: self.emit(op, CONDS[c])
        if op in ("JMP", "CALL"):
            return lambda l: self.emit(op, l)
        if op == "RET":
            return lambda: self.emit("SPC", 0)
        if op == "HALT":
            return lambda: self.emit("SPC", 1)
        if op == "NORM":
            return lambda: self.emit("SPC", 2)
        raise AttributeError(op)

    def words(self):
        out = []
        for op, arg in self.code:
            if isinstance(arg, str):
                arg = self.labels[arg]
            out.append((OPC[op] << 12) | (arg & 0xFFF))
        return out

    # macros ------------------------------------------------------------------------------------------
    def dz(self, n):
        """bipolar deadband: v >= n -> v - n; v <= -n -> v + n; else 0"""
        d = self.new()
        self.SUBI(n); self.SKIP('LT'); self.JMP(d); self.ADDI(2 * n); self.SKIP('LT'); self.LDI(0)
        self.label(d)

    def abs_(self, tmp):
        d = self.new()
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def floor512(self):
        self.ADDI(512); self.SKIP('GE'); self.LDI(0); self.SUBI(512)


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults)
    for n, v in (('h0', D['k1']), ('h1', D['k2']), ('h2', D['k3']), ('h3', D['k4']), ('h4', D['k5']),
                 ('h5', D['k6']), ('h6', D['p12'])):
        a.reg(n, v)
    for n, v in (('a3', D['k3']), ('a4', D['k4']), ('a5', D['k5']), ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('cxs'); a.reg('cys')

    a.JMP('main')

    # ---- INTERP: ACC = T[ix] + ((T[ix + 1] - T[ix]) * fx >> 8)        (fx: 8 fraction bits)
    a.label('INTERP')
    a.LD('ix'); a.OUT('ADR'); a.IN('ROM'); a.ST('v0'); a.LD('ix'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM')
    a.SUB('v0'); a.ST('dv'); a.LD('fx'); a.MUL('dv'); a.SHR(8); a.ADD('v0'); a.RET()

    # ---- LOG: ACC = log2(ACC) in Q6 (msb position * 64 + 4 mantissa bits through the table); 0 -> -2000
    a.label('LOG')
    a.ST('t0'); a.LDI(16); a.MUL('t0'); a.NORM(); a.SUBI(16); a.SKIP('GE'); a.JMP('lzero')
    a.ADDI(LGB); a.OUT('ADR'); a.IN('CNT'); a.MULI(64); a.ST('t1'); a.IN('ROM'); a.ADD('t1'); a.RET()
    a.label('lzero')
    a.LDI(-2000); a.RET()

    # ---- CGL: the scene centre along one axis.  ck = knob, cdim = screen size, cs = centre (Q8 px, glided)
    #      target = size / 2 + (knob - 512, +-8 deadband) * size / 256; ease 1/16 of the gap, never slower
    #      than 1 px per field, land exactly; Glide off = take the target.  Returns the centre in Q3.
    a.label('CGL')
    a.LD('ck'); a.SUBI(512); a.dz(8); a.MUL('cdim'); a.ST('t0'); a.LD('cdim'); a.SHR(1); a.MULI(256)
    a.ADD('t0'); a.SUB('cs'); a.ST('t1')
    a.SKIP('GLIDE'); a.JMP('csnap')
    a.abs_('t2'); a.ST('t2'); a.SUBI(257); a.SKIP('GE'); a.JMP('csnap')
    a.LD('t2'); a.SHR(4); a.SUBI(256); a.SKIP('LT'); a.JMP('cease')
    a.LDI(256); a.ST('t3'); a.LD('t1'); a.SKIP('LT'); a.JMP('cadd'); a.LDI(-256); a.ST('t3')
    a.label('cadd')
    a.LD('cs'); a.ADD('t3'); a.JMP('cst')
    a.label('cease')
    a.LD('t1'); a.SHR(4); a.ADD('cs'); a.JMP('cst')
    a.label('csnap')
    a.LD('t1'); a.ADD('cs')
    a.label('cst')
    a.ST('cs'); a.SHR(5); a.RET()

    a.label('main')
    a.IN('WL'); a.ST('W'); a.IN('H'); a.ST('H'); a.IN('YSTP'); a.ST('Y')

    # ---- knob holds (a knob has to move more than one count) and glides (Q6; S7 Off = no easing)
    def hold(inp, h):
        u, k = a.new(), a.new()
        a.IN(inp); a.SUB(h); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
        a.label(u); a.IN(inp); a.ST(h)
        a.label(k)

    def glide(h, acc):
        s = a.new()
        a.LD(h); a.MULI(64); a.SKIP('GLIDE'); a.JMP(s); a.SUB(acc); a.SHR(4); a.ADD(acc)
        a.label(s); a.ST(acc)

    hold('K1', 'h0'); hold('K2', 'h1')
    for inp, h, acc in (('K3', 'h2', 'a3'), ('K4', 'h3', 'a4'), ('K5', 'h4', 'a5'), ('K6', 'h5', 'a6'),
                        ('P12', 'h6', 'ap')):
        hold(inp, h); glide(h, acc)

    # ---- X / Y
    a.LD('h0'); a.ST('ck'); a.LD('W'); a.ST('cdim'); a.LD('cxs'); a.ST('cs'); a.CALL('CGL'); a.ST('cx3')
    a.LD('cs'); a.ST('cxs')
    a.LD('h1'); a.ST('ck'); a.LD('H'); a.ST('cdim'); a.LD('cys'); a.ST('cs'); a.CALL('CGL'); a.ST('cy3')
    a.LD('cs'); a.ST('cys')

    # ---- PERIOD: log2(cycles per virtual px) = -10 + K3 * 5 / 1024, Q12
    a.LD('a3'); a.MULI(5); a.SHR(4); a.ST('t0'); a.LDI(-640); a.MULI(64); a.ADD('t0'); a.ST('lf')
    a.SHR(12); a.ST('t1'); a.ADDI(4); a.OUT('KC')
    a.LD('t1'); a.MULI(64); a.MULI(64); a.ST('t2'); a.LD('lf'); a.SUB('t2'); a.MULI(4); a.ST('t2')
    # exponent offset: the x step, the coordinates' 3 fraction bits, the stripes' 1 / sqrt 2, the period's
    # fractional octaves
    a.LDI(-48); a.SKIP('NSTRIPES'); a.LDI(-56); a.MULI(1024); a.ADD('t2'); a.ST('t2'); a.IN('LS14')
    a.ADD('t2'); a.OUT('EOFF')
    a.LD('lf'); a.SHR(6); a.ST('t0'); a.IN('LS6'); a.ADD('t0'); a.ST('lfp'); a.OUT('LFP')

    # ---- FLUTES: log2(width in x pixels) = 4 + K4 * 19 / 4096 - log2(x step), Q12 -> width Q4
    a.LDI(16); a.MULI(1024); a.ST('t1'); a.LD('a4'); a.MULI(19); a.SHR(6); a.ADD('t1'); a.ST('t1')
    a.IN('LS12'); a.ST('t2'); a.LD('t1'); a.SUB('t2'); a.ST('lw')
    a.SHR(12); a.ST('wi'); a.MULI(64); a.MULI(64); a.ST('t0'); a.LD('lw'); a.SUB('t0'); a.ST('fr')
    a.SHR(5); a.ST('j'); a.MULI(32); a.ST('t0'); a.LD('fr'); a.SUB('t0'); a.MULI(8); a.ST('fx')
    a.LD('j'); a.ADDI(EXPB); a.ST('ix'); a.CALL('INTERP'); a.ST('em')
    a.LD('wi'); a.ADDI(P2B - 3); a.OUT('ADR'); a.IN('ROM'); a.ST('t0'); a.LD('em'); a.MUL('t0'); a.SHR(10)
    a.ST('wq4'); a.OUT('WQ4')
    a.SHR(3); a.ST('t0'); a.LD('wq4'); a.SHR(2); a.ADD('t0'); a.OUT('T3')
    a.LD('wq4'); a.MULI(16); a.ST('wq8'); a.LD('wq4'); a.MULI(1024); a.ST('w14')
    a.LDI(128 + EXPB); a.SUB('j'); a.OUT('ADR'); a.IN('ROM'); a.SHR(9); a.OUT('R7')
    a.LDI(8); a.SUB('wi'); a.OUT('NSH')

    # ---- ANGLE: +-90 degrees, +-6 counts of detent at vertical; cos >= 0, sin signed, Q15
    a.LDI(-512); a.MULI(64); a.ADD('a5'); a.dz(384); a.ST('d5')
    a.abs_('t0'); a.MULI(33); a.ST('af'); a.LDI(1024); a.MULI(1024); a.ST('q20'); a.SUB('af')
    a.SKIP('LT'); a.JMP('aok'); a.LD('q20'); a.ST('af')
    a.label('aok')

    def sine(ang):
        a.LD(ang); a.SHR(12); a.ST('ix'); a.MULI(64); a.MULI(64); a.ST('t0'); a.LD(ang); a.SUB('t0')
        a.SHR(4); a.ST('fx'); a.CALL('INTERP')

    sine('af'); a.ST('sm')
    a.LD('q20'); a.SUB('af'); a.ST('t2'); sine('t2'); a.ST('ca')
    a.LD('d5'); a.SKIP('LT'); a.JMP('spos'); a.LDI(0); a.SUB('sm'); a.JMP('sst')
    a.label('spos')
    a.LD('sm')
    a.label('sst')
    a.ST('sa'); a.ADD('ca'); a.ST('cps'); a.LD('ca'); a.SUB('sa'); a.ST('cms')

    # ---- POWER - 1 (Q10): bipolar, +-8 counts of detent at flat glass; up to +3.93, down to -4.92
    a.LDI(-512); a.MULI(64); a.ADD('ap'); a.dz(512); a.ST('d12'); a.SHR(3); a.ST('pm1'); a.LD('d12')
    a.SKIP('LT'); a.JMP('pdone'); a.SHR(5); a.ADD('pm1'); a.ST('pm1')
    a.label('pdone')
    # gradient terms: log2 |Power|, and for the stripes log2 |cos -+ sin| / sqrt 2
    a.LD('pm1'); a.ADDI(1024); a.abs_('t2'); a.CALL('LOG'); a.SUBI(640); a.floor512(); a.ST('lp'); a.OUT('LP')
    a.LD('cms'); a.abs_('t2'); a.CALL('LOG'); a.SUBI(992); a.floor512(); a.ADD('lfp'); a.OUT('AS')
    a.LD('cps'); a.abs_('t2'); a.CALL('LOG'); a.SUBI(992); a.floor512(); a.ADD('lfp'); a.ADD('lp'); a.OUT('BS')

    # ---- CAMERA: phase per luma step = K6 * 150 / 256 (2 periods at white), Osc twice that
    a.LD('a6'); a.SHR(6); a.MULI(150); a.SHR(8); a.SKIP('NSRC'); a.MULI(2); a.OUT('G')

    # ---- the flute offset at the top-left pixel: s = -(W/2 cos + H/2 sin) (Q8), then
    #      u = ((s + Wf/2) mod Wf) - Wf by 9 trial subtractions; t = u + Wf/2
    a.LD('W'); a.SHR(1); a.MUL('ca'); a.ST('t0'); a.LD('H'); a.SHR(1); a.MUL('sa'); a.ADD('t0'); a.ST('t0')
    a.LDI(0); a.SUB('t0'); a.SHR(7); a.ST('sab')
    a.LD('wq8'); a.MULI(256); a.ST('ws'); a.LD('wq8'); a.SHR(1); a.ADD('sab'); a.ADD('ws'); a.ST('x')
    a.LDI(8); a.ST('c')
    a.label('rloop')
    a.LD('x'); a.SUB('ws'); a.SKIP('LT'); a.ST('x'); a.LD('ws'); a.SHR(1); a.ST('ws')
    a.LD('c'); a.SUBI(1); a.ST('c'); a.SKIP('LT'); a.JMP('rloop')
    a.LD('x'); a.SUB('wq8'); a.ST('u8'); a.MULI(64); a.OUT('LU'); a.LD('wq8'); a.SHR(1); a.ADD('u8'); a.ST('t8')

    # ---- steps per line along the flute normal and along the flute (Q14)
    a.LD('Y'); a.MUL('sa'); a.SHR(9); a.ST('sy'); a.OUT('SY')
    a.LD('Y'); a.MUL('ca'); a.SHR(9); a.OUT('VLINE')

    # ---- lens gain kappa (Q10): rings Power - 1; stripes (Power - 1)(cos + sin)
    a.LD('pm1'); a.SKIP('STRIPES'); a.JMP('kdone'); a.MUL('cps'); a.SHR(15)
    a.label('kdone')
    a.ST('kap')

    # ---- q (the scene coordinate along the flute normal; stripes: the stripe coordinate * sqrt 2)
    a.SKIP('STRIPES'); a.JMP('qrings')
    a.LD('kap'); a.MUL('t8'); a.SHR(4); a.ST('t0'); a.LD('cx3'); a.ADD('cy3'); a.MULI(1024); a.ST('t1')
    a.ADD('t1'); a.ST('t1'); a.LD('t0'); a.SUB('t1'); a.JMP('qinit')
    a.label('qrings')
    a.LD('cx3'); a.MUL('ca'); a.ST('t0'); a.LD('cy3'); a.MUL('sa'); a.ADD('t0'); a.ST('t0')
    a.LD('kap'); a.MUL('t8'); a.SUB('t0'); a.SHR(4)
    a.label('qinit')
    a.OUT('LQ')
    a.SKIP('STRIPES'); a.JMP('irings')
    a.LD('kap'); a.MUL('ca'); a.ST('t0'); a.LDI(32); a.MULI(1024); a.MULI(1024); a.ADD('t0'); a.SHR(11)
    a.JMP('iset')
    a.label('irings')
    a.LD('kap'); a.ADDI(1024); a.MUL('ca'); a.SHR(11)
    a.label('iset')
    a.ST('qinc'); a.OUT('QINC')
    a.SKIP('STRIPES'); a.JMP('lrings')
    a.LD('kap'); a.MUL('sy'); a.ST('t0'); a.LD('Y'); a.MULI(64); a.MULI(1024); a.ADD('t0'); a.SHR(10)
    a.JMP('lset')
    a.label('lrings')
    a.LD('kap'); a.ADDI(1024); a.MUL('sy'); a.SHR(10)
    a.label('lset')
    a.OUT('QLINE')
    a.LD('kap'); a.MUL('wq4'); a.ST('qj'); a.OUT('QJUMP')
    a.LD('qinc'); a.SUB('qj'); a.OUT('QINCW')

    # ---- v (the scene coordinate along the flute)
    a.LD('cy3'); a.MUL('ca'); a.ST('t0'); a.LD('cx3'); a.MUL('sa'); a.SUB('t0'); a.SHR(4); a.OUT('LV')
    a.LD('sm'); a.SHR(1); a.ST('t0'); a.LD('d5'); a.SKIP('GE'); a.JMP('vneg'); a.LDI(0); a.SUB('t0')
    a.JMP('vout')
    a.label('vneg')
    a.LD('t0')
    a.label('vout')
    a.OUT('VINC')
    a.LD('ca'); a.SHR(1); a.OUT('UINC'); a.SUB('w14'); a.OUT('UINCW')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; flute.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=100000):
    """inp: dict over INS names (K1.. raw knobs, WL, H, YSTP, LS14, LS12, LS6); flags: GLIDE, STRIPES, SRC.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (outs dict by OUTS name, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; cnt = 0; adr = 0
    outs = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        if op in (0, 1, 2, 3):
            b = _sx(rf[arg & 63], 32)
        elif op in (4, 5, 6, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, 37)
        elif op in (2, 6):
            acc = _sx(acc - b, 37)
        elif op in (3, 7):
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 17
        elif op == 8:
            rf[arg & 63] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            elif n == INS['CNT']: acc = cnt
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 511
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'STRIPES', 3: 'SRC'}[c]])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 1023
        elif op == 14:
            lr = pc + 1; npc = arg & 1023
        else:
            if arg == 0: npc = lr
            elif arg == 1: break
            else:
                cnt = 0
                while (acc >> 5) != 0:
                    acc >>= 1; cnt += 1
                clocks += cnt
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    inv_o = {v: k for k, v in OUTS.items()}
    return {inv_o[k]: v for k, v in outs.items()}, rf, clocks


def vhd_table(name, typ, depth, width, vals):
    hexw = (width + 3) // 4
    mask = (1 << (hexw * 4)) - 1
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, hexw * 4 - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (hexw, v & mask) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert max(LOGS) < 128 and min(UGS) >= -1024 and max(EXS) < 256 and max(FT) <= 32768
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by flute.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package flute_pkg is",
             vhd_table("C_FL_LOGV", "std_logic_vector", 256, 16, LOGV),
             vhd_table("C_FL_LOGS", "std_logic_vector", 256, 16, LOGS),
             vhd_table("C_FL_UG", "std_logic_vector", 256, 16, UG),
             vhd_table("C_FL_UGS", "std_logic_vector", 256, 16, UGS),
             vhd_table("C_FL_EXM", "std_logic_vector", 256, 16, EXM),
             vhd_table("C_FL_EXS", "std_logic_vector", 256, 16, EXS),
             vhd_table("C_FL_HT", "unsigned", 64, 8, HT),
             vhd_table("C_FL_FT", "std_logic_vector", 512, 16, FT),
             vhd_table("C_FL_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_FL_RFI", "std_logic_vector", 256, 32, RF_INIT),
             "end package flute_pkg;", ""]
    with open(os.path.join(HERE, "flute_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
