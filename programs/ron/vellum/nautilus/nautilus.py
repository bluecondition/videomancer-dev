#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
NAUTILUS v0.1.0 (Vellum) - build hook.  Writes nautilus_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.vellum_work/rb_nautilus/imodel.py) imports this file and
runs the same microprogram through emulate(), so the per-field arithmetic has one source.

C_NT_FT  512 x 16   the engine's table ROM:
                      0 .. 256  EXP: round(32768 * 2^(i/256)) - 32768
                    257 .. 272  2^j, j = 0 .. 15
                    273 .. 437  five P12 columns, 33 points each (knob = 32 i), address 273 + 33 c + i:
                                c 0 l' = log2(rho) - 1/2 (Q16); c 1 lam_ref (Q12 levels, two's complement);
                                c 2 / 3 the eye of the centred rectangle, x / y (virtual px Q2, y down);
                                c 4 cn = log2 min(rho^4 - 1, rho + rho^3) (Q12)
                    438 .. 493  per video mode (8 words): log2 p (Q12), 1/p (Q15), x0 / y0 (AA units, signed),
                                x / y step (Q14 AA units per pixel / line)
C_NT_UC  1024 x 16  the per-field microprogram (PROGRAM below)
C_NT_RFI 256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults)

The per-field engine (after Hardline flute's): a 37-bit accumulator, a 64 x 32-bit register file, one adder,
a serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode + 12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PHI = (1 + 5 ** 0.5) / 2
TW = 1080.0 / (PHI + PHI ** 3) * PHI ** 5          # window top: T(entry 0) in [TW / rho, TW) virtual px
LOG2TW3 = int(round((math.log2(TW) + 3) * 65536))   # log2 of TW in Q3 units at p = 1
K8 = 1.0
for _i in range(8):
    K8 *= math.sqrt(1 + 2.0 ** (-2 * _i))
KH = int(round(K8 / 2 * 65536))                     # CORDIC gain / 2, Q16
# 8 x the line widths in virtual px: (ink, fine) -> (edge, arc, diagonal, chord)
W8 = {(0, 0): (20, 64, 20, 24), (0, 1): (16, 20, 16, 16), (1, 0): (24, 64, 24, 24), (1, 1): (16, 20, 16, 16)}
N_ENT = 20

EXPB, P2B, PCB, MTB = 0, 257, 273, 438
# video modes: (log2 p Q12, 1/p Q15, x0, y0, xs Q14, ys Q14); index from the timing ID (mode_of)
MODES = [(0, 32768, -960, -540, 16384, 16384),          # 0 1080p
         (2396, 21845, -640, -360, 16384, 16384),       # 1 720p      (p = 1.5)
         (2396, 21845, -640, -360, 10923, 21845),       # 2 1080i     (p = 1.5)
         (4096, 16384, -360, -270, 16384, 18432),       # 3 480p      (p = 2)
         (4096, 16384, -360, -270, 16384, 15360),       # 4 576p
         (6492, 10923, -240, -180, 10923, 24277),       # 5 480i      (p = 3)
         (6492, 10923, -240, -180, 10923, 20480)]       # 6 576i


def mode_of(tid):
    return {5: 1, 6: 1, 14: 1, 1: 2, 2: 2, 10: 2, 4: 3, 12: 4, 0: 5, 8: 6}.get(tid, 0)


def log2rho(knob):
    lp = math.log2(PHI)
    if knob <= 512:
        return 0.5 + (lp - 0.5) * knob / 512.0
    return lp + (1 - lp) * (knob - 512) / 512.0


def h0(rho):
    return min(1080.0, 1900.0 / rho)


def lam_ref(rho):
    return 5 + math.log(h0(rho) / ((rho + rho ** 3) * TW)) / math.log(rho)


def eye_def(rho):
    H0 = h0(rho)
    return H0 * (rho ** 3 / (rho ** 2 + 1) - rho / 2), H0 * (0.5 - 1 / (rho ** 2 + 1))


def columns():
    cols = [[], [], [], [], []]
    for i in range(33):
        l = log2rho(32 * i)
        rho = 2.0 ** l
        ex, ey = eye_def(rho)
        cols[0].append(int(round((l - 0.5) * 65536)))
        cols[1].append(int(round(lam_ref(rho) * 4096)))
        cols[2].append(int(round(ex * 4)))
        cols[3].append(int(round(ey * 4)))
        cols[4].append(int(round(math.log2(min(rho ** 4 - 1, rho + rho ** 3)) * 4096)))
    return cols


def ft_rom():
    r = [0] * 512
    for i in range(257):
        r[EXPB + i] = int(round(32768 * 2 ** (i / 256.0))) - 32768
    for j in range(16):
        r[P2B + j] = 1 << j
    for c, col in enumerate(columns()):
        for i, v in enumerate(col):
            r[PCB + 33 * c + i] = v & 0xFFFF
    for m, words in enumerate(MODES):
        for w, v in enumerate(words):
            r[MTB + 8 * m + w] = v & 0xFFFF
    return r


FT = ft_rom()

DEFAULTS = dict(k1=512, k2=512, k3=512, k5=680, k6=1023, p12=512)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, MODE=8, ROM=9, XE=10, XI=11, XR=12)
CONDS = dict(LT=0, GLIDE=1, HAND=2, INK=3, FINE=4, GE=8, NGLIDE=9, NHAND=10, NINK=11, NFINE=12)
OUTS = dict(ADR=0, RADR=1, R0=2, R1=3, R2=4, R3=5, R4=6, R5=7, R7=8, XB0=9, XBS0=10, YB0=11, YBS0=12,
            XS=13, XSS=14, YS=15, YSS=16, WE=17, WA=18, WC=19, WH=20, CW=21, GST=22, ROT=23, DONE=24, XL=25)


class Asm:
    def __init__(self):
        self.code = []
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
        raise AttributeError(op)

    def words(self):
        out = []
        for op, arg in self.code:
            if isinstance(arg, str):
                arg = self.labels[arg]
            out.append((OPC[op] << 12) | (arg & 0xFFF))
        return out

    # macros ------------------------------------------------------------------------------------------
    def abs_(self, tmp):
        d = self.new()
        self.ST(tmp); self.SKIP('GE'); self.JMP(d + 'n'); self.JMP(d)
        self.label(d + 'n'); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def sat(self, cname):
        """ACC = min(ACC, reg cname)"""
        self.SUB(cname); self.SKIP('LT'); self.LDI(0); self.ADD(cname)

    def sx16(self):
        """ACC (0 .. 65535) as a signed 16-bit value"""
        self.SUB('c32768'); self.SKIP('GE'); self.ADD('c65536'); self.SUB('c32768')


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults); constants
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h6', D['k6']), ('hp', D['p12']), ('h5', D['k5'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('vgs', (D['k5'] - 512) * 64)
    a.reg('lamd', 0)
    for n, v in (('c32768', 32768), ('c65536', 65536), ('c65535', 65535), ('c2p20', 1 << 20), ('c2p19', 1 << 19),
                 ('c14', 14 * 65536), ('cTW', LOG2TW3), ('cKH', KH), ('c3760', 3760), ('c2080', 2080),
                 ('c16384', 16384), ('cFV', 196608 + 169408)):
        a.reg(n, v)

    a.JMP('main')

    # ---- EXP: ACC = 2^(ACC / 65536) = M 2^(e - 15), M the interpolated Q15 mantissa; e < 2 counts as 2
    a.label('EXP')
    a.OUT('XL'); a.IN('XR'); a.ST('xr'); a.IN('XI'); a.OUT('ADR'); a.IN('ROM'); a.ST('xv')
    a.IN('XI'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM'); a.SUB('xv'); a.MUL('xr'); a.SHR(8); a.ADD('xv')
    a.ADD('c32768'); a.ST('xm')
    a.IN('XE'); a.SUBI(2); a.SKIP('GE'); a.JMP('ezero')
    a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('xm'); a.SHR(13); a.RET()
    a.label('ezero'); a.LDI(0); a.RET()

    # ---- INTERP column c (operand pi / pf): ACC = C[pi] + ((C[pi + 1] - C[pi]) pf >> 8)
    def interp(c, signed):
        a.LD('pi'); a.ADDI(PCB + 33 * c); a.OUT('ADR'); a.IN('ROM')
        if signed: a.sx16()
        a.ST('xv'); a.LD('pi'); a.ADDI(PCB + 33 * c + 1); a.OUT('ADR'); a.IN('ROM')
        if signed: a.sx16()
        a.SUB('xv'); a.MUL('pf'); a.SHR(8); a.ADD('xv')

    a.label('main')
    # ---- knob holds (a knob has to move more than one count) and glides (Q6): 1/16 of the gap per field,
    #      never under half a count, landing exactly; S7 Off = no easing
    def hold(inp, h):
        u, k = a.new(), a.new()
        a.IN(inp); a.SUB(h); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
        a.label(u); a.IN(inp); a.ST(h)
        a.label(k)

    def glide(h, acc):
        snap, ease, add, done = a.new(), a.new(), a.new(), a.new()
        a.LD(h); a.MULI(64); a.SUB(acc); a.ST('g')
        a.SKIP('GLIDE'); a.JMP(snap)
        a.abs_('t1'); a.SUBI(33); a.SKIP('GE'); a.JMP(snap)
        a.ADDI(33); a.SHR(4); a.SUBI(32); a.SKIP('LT'); a.JMP(ease)
        a.LDI(32); a.ST('t1'); a.LD('g'); a.SKIP('LT'); a.JMP(add); a.LDI(-32); a.ST('t1')
        a.label(add); a.LD(acc); a.ADD('t1'); a.JMP(done)
        a.label(ease); a.LD('g'); a.SHR(4); a.ADD(acc); a.JMP(done)
        a.label(snap); a.LD('g'); a.ADD(acc)
        a.label(done); a.ST(acc)

    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3', 'a3'), ('K6', 'h6', 'a6'),
                        ('P12', 'hp', 'ap')):
        hold(inp, h); glide(h, acc)
    hold('K5', 'h5')
    # ---- the Drift speed: glided 1/8 (Q6), then Hypnos's decode: |v| < 24 still, else 2^-12 .. 2^-3 levels
    #      per field; the drift phase wraps modulo 4 levels (2^20, Q18)
    a.LD('h5'); a.SUBI(512); a.MULI(64); a.ST('t0'); a.SKIP('GLIDE'); a.JMP('vsnap')
    a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('vgs'); a.JMP('vdec')
    a.label('vsnap'); a.LD('t0'); a.ST('vgs')
    a.label('vdec')
    a.LD('vgs'); a.SHR(6); a.ST('g'); a.abs_('t1'); a.SUBI(24); a.SKIP('GE'); a.JMP('vdone')
    a.ST('t0'); a.SHR(3); a.ST('t1'); a.LD('t0'); a.SHR(4); a.ADD('t1'); a.ADD('t0'); a.ST('pi')
    a.SHR(6); a.ST('t1'); a.MULI(64); a.ST('t0'); a.LD('pi'); a.SUB('t0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ADD('c32768'); a.SHR(9); a.ST('pf')
    a.LD('t1'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('pf'); a.ST('pf')
    a.LD('g'); a.SKIP('LT'); a.JMP('vpos'); a.LD('lamd'); a.SUB('pf'); a.JMP('vacc')
    a.label('vpos'); a.LD('lamd'); a.ADD('pf')
    a.label('vacc')
    a.SKIP('GE'); a.ADD('c2p20'); a.ST('lamd'); a.SUB('c2p20'); a.SKIP('LT'); a.ST('lamd')
    a.label('vdone')

    # ---- P12: the five columns, linear between 33 points
    a.LD('ap'); a.SHR(11); a.ST('pi'); a.MULI(256); a.ST('t0'); a.LD('ap'); a.SHR(3); a.SUB('t0'); a.ST('pf')
    interp(0, False); a.ADD('c32768'); a.ST('l')
    interp(1, True); a.ST('lr')
    interp(2, True); a.ST('ex')
    interp(3, True); a.ST('ey')
    interp(4, False); a.MULI(16); a.SUB('cFV'); a.ST('cnq')

    # ---- the eye (virtual px Q2): piecewise so 0 / 100 % reach +-940 (x) and +-520 (y)
    a.LD('a1'); a.SUB('c32768'); a.ST('t2'); a.SKIP('LT'); a.JMP('xpos')
    a.LD('ex'); a.ADD('c3760'); a.JMP('xsl')
    a.label('xpos'); a.LD('c3760'); a.SUB('ex')
    a.label('xsl'); a.ST('t0'); a.LD('t2'); a.MUL('t0'); a.SHR(15); a.ADD('ex'); a.ST('ex')
    a.LD('a2'); a.SUB('c32768'); a.ST('t2'); a.SKIP('LT'); a.JMP('ypos')
    a.LD('c2080'); a.SUB('ey'); a.JMP('ysl')
    a.label('ypos'); a.LD('ey'); a.ADD('c2080')
    a.label('ysl'); a.ST('t0'); a.LD('t2'); a.MUL('t0'); a.SHR(15); a.ST('t0'); a.LD('ey'); a.SUB('t0'); a.ST('ey')

    # ---- the video mode: log2 p, 1/p, the pixel-0 x, the line-0 y, the steps (Q16 AA units)
    a.IN('MODE'); a.MULI(8); a.ADDI(MTB); a.ST('t2')
    a.OUT('ADR'); a.IN('ROM'); a.MULI(16); a.ST('l2p')
    a.LD('t2'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM'); a.MULI(2); a.ST('invp')
    a.LD('t2'); a.ADDI(2); a.OUT('ADR'); a.IN('ROM'); a.sx16(); a.MUL('c65536'); a.ST('xb')
    a.LD('t2'); a.ADDI(3); a.OUT('ADR'); a.IN('ROM'); a.sx16(); a.MUL('c65536'); a.ST('yb')
    a.LD('t2'); a.ADDI(4); a.OUT('ADR'); a.IN('ROM'); a.MULI(4); a.ST('xs')
    a.LD('t2'); a.ADDI(5); a.OUT('ADR'); a.IN('ROM'); a.MULI(4); a.ST('ys'); a.OUT('YS')
    a.LD('ex'); a.MUL('invp'); a.SHR(2); a.ST('t0'); a.LD('xb'); a.SUB('t0'); a.ST('xb')
    a.LD('ey'); a.MUL('invp'); a.SHR(2); a.ST('t0'); a.LD('yb'); a.SUB('t0'); a.ST('yb'); a.OUT('YB0')
    a.SKIP('HAND'); a.JMP('hdone')
    a.LDI(0); a.SUB('xb'); a.ST('xb'); a.LDI(0); a.SUB('xs'); a.ST('xs')
    a.label('hdone')
    a.LD('xb'); a.OUT('XB0'); a.LD('xs'); a.OUT('XS')

    # ---- the zoom phase: drift + Scale (+-1.5 levels) + the composition reference, modulo 4 levels
    a.LD('a3'); a.SUB('c32768'); a.MULI(12); a.ADD('lamd'); a.ST('t0'); a.LD('lr'); a.MULI(64); a.ADD('t0')
    a.SKIP('GE'); a.ADD('c2p20'); a.ST('t2'); a.SUB('c2p20'); a.SKIP('LT'); a.ST('t2')
    a.LD('t2'); a.SHR(18); a.OUT('ROT'); a.MULI(512); a.MULI(512); a.ST('t0'); a.LD('t2')
    a.SUB('t0'); a.SHR(1); a.MUL('l'); a.SHR(17); a.ADD('cTW'); a.SUB('l2p'); a.SUB('l'); a.ST('L')

    # ---- 2^-l (Q16), rho - 1/rho (Q14), the stretched DDA (x (rho - 1/rho))
    a.LD('c2p20'); a.SUB('l'); a.CALL('EXP'); a.SHR(2); a.ST('t1')
    a.LD('c14'); a.ADD('l'); a.CALL('EXP'); a.SUB('t1'); a.ST('sk')
    a.LD('xb'); a.SHR(12); a.MUL('sk'); a.SHR(2); a.OUT('XBS0')
    a.LD('xs'); a.MUL('sk'); a.SHR(14); a.OUT('XSS')
    a.LD('yb'); a.SHR(12); a.MUL('sk'); a.SHR(2); a.OUT('YBS0')
    a.LD('ys'); a.MUL('sk'); a.SHR(14); a.OUT('YSS')

    # ---- line widths (Q3 AA units): max(8, (8 w invp + 2^15) >> 16); the ring offset; the camera gain
    def width(idx, out):
        r, s = a.new(), a.new()
        a.SKIP('INK'); a.JMP(r)
        a.LDI(W8[(1, 0)][idx]); a.SKIP('NFINE'); a.LDI(W8[(1, 1)][idx]); a.JMP(s)
        a.label(r); a.LDI(W8[(0, 0)][idx]); a.SKIP('NFINE'); a.LDI(W8[(0, 1)][idx])
        a.label(s); a.MUL('invp'); a.ADD('c32768'); a.SHR(16); a.SUBI(8); a.SKIP('GE'); a.LDI(0); a.ADDI(8)
        a.OUT(out)
    width(0, 'WE'); width(1, 'WA'); a.MUL('cKH'); a.SHR(16); a.OUT('CW'); width(2, 'WC'); width(3, 'WH')
    a.LD('a6'); a.SHR(6); a.MULI(80); a.SHR(10); a.OUT('GST')

    # ---- the thresholds T(m) = 2^(B - m l), m = -4 .. 20 (Q3 units), and the record of entry m >= 0:
    #      R0 T | (A >> 16) << 14, R1 min(T(m-1), 65535), R2 min(T(m-3), 65535), R3 A = T(m-4) - T (low 16),
    #      R4 min(T(m-2) - T, 65535), R5 AK = A K8 (low 16), R7 AK >> 16 | no-arc << 3 | fade << 4
    #      (the output port saturates R1 R2 R4 at 65535, packs A >> 16 into R0 and AK >> 16 / no-arc / the
    #      clamped fade into R7; entry 20, the core, gets fade 0)
    a.LD('l'); a.MULI(4); a.ADD('L'); a.ST('L'); a.LDI(-4); a.ST('m')
    a.label('loop')
    a.LD('T3'); a.ST('T4'); a.LD('T2'); a.ST('T3'); a.LD('T1'); a.ST('T2'); a.LD('T0'); a.ST('T1')
    a.LD('L'); a.CALL('EXP'); a.ST('T0')
    a.LD('m'); a.SKIP('GE'); a.JMP('next')
    a.OUT('RADR')
    a.LD('T4'); a.SUB('T0'); a.OUT('R3'); a.SHR(1); a.MUL('cKH'); a.SHR(14); a.OUT('R5')
    a.LD('T0'); a.OUT('R0'); a.LD('T1'); a.OUT('R1'); a.LD('T3'); a.OUT('R2'); a.LD('T2'); a.SUB('T0'); a.OUT('R4')
    a.LD('L'); a.ADD('cnq'); a.SHR(13); a.OUT('R7')
    a.label('next')
    a.LD('L'); a.SUB('l'); a.ST('L'); a.LD('m'); a.ADDI(1); a.ST('m'); a.SUBI(N_ENT + 1); a.SKIP('GE'); a.JMP('loop')
    a.OUT('DONE')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; nautilus.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. raw knobs, MODE); flags: GLIDE, HAND, INK, FINE.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (outs: dict OUTS name -> last value, recs: {addr: {R0..R7}}, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; radr = 0; xl = 0; ahi = 0; akh = 0
    outs = {}; recs = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    inv_o = {v: k for k, v in OUTS.items()}
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
            elif n == INS['XE']: acc = xl >> 16
            elif n == INS['XI']: acc = (xl >> 8) & 255
            elif n == INS['XR']: acc = xl & 255
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            name = inv_o.get(n, n)
            if name == 'ADR': adr = acc & 511
            elif name == 'RADR': radr = acc & 31
            elif name == 'XL': xl = acc
            elif name == 'R3':
                recs.setdefault(radr, {})['R3'] = acc & 0xFFFF; ahi = (acc >> 16) & 3
            elif name == 'R5':
                if acc >= (1 << 19): recs.setdefault(radr, {})['R5'] = 0xFFFF; akh = 15
                else: recs.setdefault(radr, {})['R5'] = acc & 0xFFFF; akh = (acc >> 16) & 7
            elif name == 'R0':
                recs.setdefault(radr, {})['R0'] = (ahi << 14) | (acc & 0x3FFF)
            elif name in ('R1', 'R2', 'R4'):
                recs.setdefault(radr, {})[name] = 0xFFFF if acc >= 65536 else acc & 0xFFFF
            elif name == 'R7':
                f = 0 if (acc < 0 or radr == N_ENT) else min(acc, 16)
                recs.setdefault(radr, {})['R7'] = (f << 4) | akh
            outs[name] = acc
        elif op == 11:
            n = arg & 31
            acc >>= n
            clocks += (n // 4) + (n % 4) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'HAND', 3: 'INK', 4: 'FINE'}[c]])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 1023
        elif op == 14:
            lr = pc + 1; npc = arg & 1023
        else:
            if arg == 0: npc = lr
            else: break
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    return outs, recs, rf, clocks


def vhd_table(name, typ, depth, width, vals):
    hexw = (width + 3) // 4
    mask = (1 << (hexw * 4)) - 1
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (hexw, v & mask) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by nautilus.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package nautilus_pkg is",
             vhd_table("C_NT_FT", "unsigned", 512, 16, FT),
             vhd_table("C_NT_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_NT_RFI", "std_logic_vector", 256, 32, RF_INIT),
             "end package nautilus_pkg;", ""]
    with open(os.path.join(HERE, "nautilus_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
