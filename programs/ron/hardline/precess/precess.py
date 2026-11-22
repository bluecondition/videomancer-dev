#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
PRECESS v0.1.0 (Hardline) - build hook.  Writes precess_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.hardline_work/rb_precess/imodel.py) imports this file and runs the
same microprograms through emulate(), so the per-field and per-line arithmetic has one source.

Package contents (block-RAM ROMs):
  C_PC_UC   1024 x 16   the engine's per-field microprogram (FIELD at 0)
  C_PC_RFI   512 x 48   the register file at power-up (holds / glides at the TOML defaults, constants)
  C_PC_SIN  1024 x 16   round(32767 sin(2 pi i / 1024)), signed                       (IN SIN)
  C_PC_EXP   512 x 16   0..255 round(16384 2^(i/256)); 256: 32768; 257 + k: 2^k (k 0..15);
                        273 + k: round(4096 log2(k + 1)) (k 0..15)                       (IN EXP, unsigned)

The engine (after cassini / flute, widened): a 48-bit accumulator, a 512 x 48 register file in block RAM, one
adder (split in two halves: four clocks per micro-op), a serial multiplier ACC(19:0) x B(27:0) (20 steps) and a
serial right shift.
Micro-op = opcode (15..12) & operand (11..0).  A register operand with bit 8 set is per-ellipse: address
256 + 128 nb + 8 X + operand(2..0), X the index register (OUT KX), nb the bank the engine writes.

It runs ONCE PER FIELD IN THE BACKGROUND: started at every vblank-zone start, it computes the NEXT field's
per-ellipse seeds into the bank the beam is not using, stalled whenever the line unit (precess.vhd) owns the
register file.  At the next zone start, if it has halted, the banks swap.  The line unit, after every line's
last pixel, loads the beam from the current bank (F0, D0, DDW of every ellipse) and steps the line DDAs there
(F0 += E, E += EE, D0 += G).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

N_ELL = 16
RFW = 48
DEFAULTS = dict(k1=512, k2=512, k3=512, k4=512, k5=660, k6=1023, p12=742)

# ---- ROMs ------------------------------------------------------------------------------------------------
SIN = [int(round(32767 * math.sin(2 * math.pi * i / 1024.0))) for i in range(1024)]
EXPB, E1B, P2B, L2B = 0, 256, 257, 273
EXP = [int(round(16384 * 2 ** (i / 256.0))) for i in range(256)] + [32768] + [1 << k for k in range(16)] + \
      [int(round(4096 * math.log2(k + 1))) for k in range(16)]
EXP += [0] * (512 - len(EXP))

# ---- instruction set ---------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, ANDI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SIN=6, P12=7, XS=8, YS=9, IHY=10, X0=11, LHM=12, HW4=13, EXP=14,
           IHX=15)
CONDS = dict(LT=0, GLIDE=1, LOG=2, FINE=3, SRC=4, GE=8, NGLIDE=9, NLOG=10, NFINE=11, NSRC=12)
OUTS = dict(ADR=0, KX=1, GST=4)
# per-ellipse slots (address 128 + 8 k + slot)
SL = dict(F0=0, E=1, D0=2, EE=3, G=4, DDW=5)   # DDW = DD | t << 17 | H << 21

# per-mode constants (timing ID -> the IN ports the VHDL drives)
#   XS = x step in half virtual px, YS = y step in 1/256 virtual px, IHY = 65536 / hy, IHX = 32768 / hx,
#   X0 = -16 x0 (x0 = the first pixel's x, virtual px), LHM = 4096 log2(max(hx, hy)), HW4 = 4096 * 4 * 2.25 / hmax
def mode_consts(tid):
    if tid in (0, 4, 8, 12):
        xs = 4
    elif tid in (5, 6, 14):
        xs = 3
    else:
        xs = 2
    ys = {0: 1138, 4: 576, 8: 960, 12: 480, 5: 384, 6: 384, 14: 384, 1: 512, 2: 512, 10: 512}.get(tid, 256)
    hx = xs / 2.0
    hy = ys / 256.0
    hm = max(hx, hy)
    x0 = (720 - 1) * 16 if xs == 4 else int(round((960 - hx / 2) * 16))
    return dict(XS=xs, YS=ys, IHY=int(round(65536 / hy)), IHX=int(round(32768 / hx)), X0=x0,
                LHM=int(round(4096 * math.log2(hm))), HW4=int(round(4096 * 9.0 / hm)))


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
            assert len(self.regs) <= 256, "out of global registers"
            self.init[name] = init
        return self.regs[name]

    def new(self):
        self.nl += 1
        return "_L%d" % self.nl

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def emit(self, op, arg=0):
        self.code.append((op, arg))

    def _r(self, r):
        if isinstance(r, int):
            return r & 0xFF                       # absolute address
        if r in SL:
            return 0x100 | SL[r]                  # per-ellipse (indexed)
        return self.reg(r)

    def __getattr__(self, op):
        if op in ("LD", "ADD", "SUB", "MUL", "ST"):
            return lambda r: self.emit(op, self._r(r))
        if op in ("LDI", "ADDI", "MULI"):
            def f(i):
                assert -2048 <= i <= 2047, (op, i)
                self.emit(op, i & 0xFFF)
            return f
        if op == "ANDI":
            def f(i):
                assert 0 <= i <= 4095
                self.emit(op, i)
            return f
        if op == "SUBI":
            return lambda i: self.ADDI(-i)
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

    # macros
    def abs_(self, tmp):
        """ACC = |ACC| (tmp clobbered)"""
        d = self.new()
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def const(self, name, v):
        self.reg(name, v)
        return name


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    for n, k in (('h1', 'k1'), ('h2', 'k2'), ('h3', 'k3'), ('h4', 'k4'), ('h5', 'k5'), ('h6', 'k6'), ('h12', 'p12')):
        a.reg(n, D[k])
    for n, k in (('g1', 'k1'), ('g2', 'k2'), ('g3', 'k3'), ('g4', 'k4'), ('g5', 'k5'), ('g6', 'k6'), ('g12', 'p12')):
        a.reg(n, D[k] * 8)
    a.reg('vgs', (D['k5'] - 512) * 64)
    a.reg('dl', 0)
    a.reg('first', -1)
    C = {}
    for n, v in dict(c4096=4096, cm4096=-4096, cm3019=-3019, c8640=8640, c20098=20098, c32768=32768,
                     c24=1 << 24, c65536=65536, cl2r=2918, c10830=10830, c53248=13 * 4096, cbig=(1 << 36) - 1,
                     c16384=16384, c1023=1023).items():
        C[n] = a.const(n, v)

    def rom_exp(base=0):
        """ACC = EXP[ACC + base]"""
        if base:
            a.ADDI(base)
        a.OUT('ADR'); a.IN('EXP')

    a.JMP('main')

    # ---- EXPI: ACC (0..4095, Q12 fraction) -> 2^(x / 4096) in Q14 (16384 .. 32768), interpolated
    a.label('EXPI')
    a.ST('xf'); a.SHR(4); a.ST('ix'); a.OUT('ADR'); a.IN('EXP'); a.ST('v0')
    a.LD('ix'); a.ADDI(1); a.OUT('ADR'); a.IN('EXP'); a.SUB('v0'); a.ST('dv')
    a.LD('xf'); a.ANDI(15); a.MUL('dv'); a.SHR(4); a.ADD('v0'); a.RET()

    # ---- SINE: ACC = angle (Q16 turns, any value) -> sin (Q15), interpolated
    a.label('SINE')
    a.ST('ang'); a.SHR(6); a.ANDI(1023); a.ST('ix'); a.OUT('ADR'); a.IN('SIN'); a.ST('v0')
    a.LD('ix'); a.ADDI(1); a.ANDI(1023); a.OUT('ADR'); a.IN('SIN'); a.SUB('v0'); a.ST('dv')
    a.LD('ang'); a.ANDI(63); a.MUL('dv'); a.SHR(6); a.ADD('v0'); a.RET()

    # ---- CH: hold + glide of one knob: kin = the knob, hh = its hold, gg = its glide (Q3).  The hold follows the
    # knob only when it moves 2 counts or more, reaches either end, or on the first field.  Glide On: 1/16 of the
    # gap per field, never less than half a count, landing exactly; Glide off or the first field: the held value.
    a.label('CH')
    a.LD('kin'); a.SUB('hh'); a.ADDI(1); a.SKIP('GE'); a.JMP('chtk')
    a.SUBI(3); a.SKIP('LT'); a.JMP('chtk')
    a.LD('first'); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUBI(1); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUB(C['c1023']); a.SKIP('LT'); a.JMP('chtk')
    a.JMP('chgl')
    a.label('chtk'); a.LD('kin'); a.ST('hh')
    a.label('chgl')
    a.LD('hh'); a.MULI(8); a.ST('t1'); a.SUB('gg'); a.ST('t2')
    a.SKIP('GLIDE'); a.JMP('chsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('chsn')
    a.LD('t2'); a.abs_('t3')
    a.SUBI(65); a.SKIP('LT'); a.JMP('ches')
    a.ADDI(60); a.SKIP('LT'); a.JMP('chst')
    a.label('chsn'); a.LD('t1'); a.ST('gg'); a.RET()
    a.label('ches'); a.LD('t2'); a.SHR(4); a.ADD('gg'); a.ST('gg'); a.RET()
    a.label('chst'); a.LDI(4); a.ST('t1'); a.LD('t2'); a.SKIP('GE'); a.LDI(-4); a.SKIP('LT'); a.LD('t1')
    a.ADD('gg'); a.ST('gg'); a.RET()

    a.label('main')
    for n in ('XS', 'YS', 'IHY', 'X0', 'LHM', 'HW4', 'IHX'):
        a.IN(n); a.ST(n.lower())

    # ---- knob holds + glides (K5's glide is unused: Drift has its own speed glide)
    for inp, h, g in (('K1', 'h1', 'g1'), ('K2', 'h2', 'g2'), ('K3', 'h3', 'g3'), ('K4', 'h4', 'g4'),
                      ('K5', 'h5', 'g5'), ('K6', 'h6', 'g6'), ('P12', 'h12', 'g12')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(g); a.ST('gg'); a.CALL('CH')
        a.LD('hh'); a.ST(h); a.LD('gg'); a.ST(g)

    # ---- CAMERA gain step (the gain table T(l) = (l gst) >> 7): 4375 (K6 / 1023)^2 (white = 7500 x 16 Q at
    # 100 %, 1/16 of that at 25 %); Osc goes through the same table.  Applied with the bank swap.
    a.LD('g6'); a.MUL('g6'); a.SHR(10); a.ST('t0'); a.LDI(1096); a.MUL('t0'); a.SHR(14); a.OUT('GST')

    # ---- DRIFT (Hypnos): speed glide 1/8 per field (Q6); |v| < 24 counts still, else 2^(ls/64 - 16) cycle per
    # field, ls = min(512, u + u/16), u = |v| - 24.  One cycle = 180 degrees of twist per ellipse = 2^24.
    a.LD('h5'); a.SUBI(512); a.MULI(64); a.ST('t1')
    a.SKIP('GLIDE'); a.JMP('vsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('vsn')
    a.LD('t1'); a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('vgs'); a.JMP('vdec')
    a.label('vsn'); a.LD('t1'); a.ST('vgs')
    a.label('vdec')
    a.LD('vgs'); a.SHR(6); a.ST('v'); a.abs_('t3'); a.SUBI(24); a.SKIP('GE'); a.JMP('dd')
    a.ST('u'); a.SHR(4); a.ADD('u'); a.SUBI(512); a.SKIP('LT'); a.LDI(0); a.ADDI(512)
    a.ST('ls'); a.SHR(6); a.ST('k'); a.MULI(64); a.ST('t0'); a.LD('ls'); a.SUB('t0'); a.MULI(4); rom_exp(EXPB)
    a.SHR(6); a.ST('m'); a.LD('k'); rom_exp(P2B); a.MUL('m'); a.ST('spd')
    a.LD('v'); a.SKIP('LT'); a.JMP('fwd')
    a.LD('dl'); a.SUB('spd'); a.SKIP('GE'); a.ADD(C['c24']); a.JMP('dst')
    a.label('fwd')
    a.LD('dl'); a.ADD('spd'); a.ST('t0'); a.SUB(C['c24']); a.SKIP('GE'); a.LD('t0')
    a.label('dst'); a.ST('dl')
    a.label('dd')

    # ---- TWIST: PHI = 2 tau (Q16 turns).  P12: +-8-count detent, then 40 degrees per ellipse at the ends
    # (2 tau = 80 degrees = 14564); plus the drift accumulator (2^24 per cycle -> >> 8).
    a.LD('g12'); a.ADD(C['cm4096']); a.ST('v3'); a.abs_('t3'); a.SUBI(64); a.SKIP('GE'); a.LDI(0)
    a.MULI(1853); a.SHR(9); a.ST('t0'); a.LD('v3'); a.SKIP('LT'); a.JMP('php'); a.LDI(0); a.SUB('t0'); a.ST('t0')
    a.label('php'); a.LD('dl'); a.SHR(8); a.ADD('t0'); a.ST('phi')

    # ---- ASPECT: le = log2 e (Q12): K4 0 .. 512: e 0.9 .. 0.6, 512 .. 1023: 0.6 .. 0.3
    a.LD('g4'); a.ADD(C['cm4096']); a.SKIP('LT'); a.JMP('eup')
    a.LDI(0); a.SUB('g4'); a.ADD(C['c4096']); a.MULI(599); a.SHR(10); a.ADD(C['cm3019']); a.JMP('edn')
    a.label('eup'); a.ST('t0'); a.LD(C['cm3019']); a.SUB('t0')
    a.label('edn'); a.ST('le')
    # e^2 (Q15) = 2^(2 le): x = 2 le, xi = x >> 12 (-4 .. -1), m = EXPI(x & 4095) (Q14); e2 = m 2^(xi + 4) >> 3
    a.LD('le'); a.ADD('le'); a.ST('t0'); a.SHR(12); a.ADDI(4); rom_exp(P2B); a.ST('t1')
    a.LD('t0'); a.ANDI(4095); a.CALL('EXPI'); a.MUL('t1'); a.SHR(3); a.ST('e2')
    a.ADD(C['c32768']); a.SHR(1); a.ST('c0')                                   # C0 = (1 + e^2) / 2
    a.LD('e2'); a.SUB(C['c32768']); a.SHR(1); a.ST('c1')                       # C1 = (e^2 - 1) / 2 (< 0)

    # ---- the form's per-field products (Q13): A0 = C0 hx^2, A1 = C1 hx^2, G0 = C0 hy^2, G1 = C1 hy^2,
    # B1 = 2 C1 hx hy.  hx = XS / 2, hy = YS / 256.
    a.LD('xs'); a.MUL('xs'); a.ST('xs2')
    a.LD('ys'); a.MUL('ys'); a.ST('ys2')
    a.LD('c0'); a.MUL('xs2'); a.SHR(4); a.ST('A0')
    a.LD('c1'); a.MUL('xs2'); a.SHR(4); a.ST('A1')
    a.LD('c0'); a.MUL('ys2'); a.SHR(18); a.ST('G0')
    a.LD('c1'); a.MUL('ys2'); a.SHR(18); a.ST('G1')
    a.LD('xs'); a.MUL('ys'); a.ST('t0'); a.LD('c1'); a.MUL('t0'); a.SHR(10); a.ST('B1')

    # ---- CENTRE (Q4 virtual px): xc = (g1 - 4096) 9/2, yc = -(g2 - 4096) 81/32 (down positive).
    # n4 = 4 (n_c + 2) (pixels, Q2; the line's DDA starts two pixels early), m4 = 4 m_c (lines, Q2).
    a.LD('g1'); a.ADD(C['cm4096']); a.MULI(9); a.SHR(1); a.ADD('x0'); a.MUL('ihx'); a.SHR(17); a.ADDI(8)
    a.ST('n4')
    a.LDI(0); a.SUB('g2'); a.ADD(C['c4096']); a.MULI(81); a.SHR(5); a.ST('t0')
    a.LD('ys'); a.SHR(5); a.SUB(C['c8640']); a.ST('t1')                        # y0 (Q4) = -8640 + 8 hy
    a.LD('t0'); a.SUB('t1'); a.MUL('ihy'); a.SHR(18); a.ST('m4')
    a.LD('n4'); a.MUL('n4'); a.ST('N2')
    a.LD('n4'); a.MUL('m4'); a.ST('NM')
    a.LD('m4'); a.MUL('m4'); a.ST('M2')
    a.LDI(4); a.SUB('n4'); a.SUB('n4'); a.ST('P1N')                           # 4 (1 - 2 n)  (Q2)
    a.LDI(4); a.SUB('m4'); a.SUB('m4'); a.ST('P1M')

    # ---- SIZES.  lP = log2 P (Q12), P = 30 8^(K3 / 1023) virtual px.  Tb = e^2 P^2 (Q13), lTb = log2(e^2 P^2).
    a.LD('g3'); a.MULI(1537); a.SHR(10); a.ADD(C['c20098']); a.ST('lP')
    a.ADD('le'); a.ST('t0'); a.ADD('t0'); a.ST('lTb')
    a.SHR(12); a.ADDI(-1); rom_exp(P2B); a.ST('t1')                           # 2^(xi - 1)
    a.LD('lTb'); a.ANDI(4095); a.CALL('EXPI'); a.MUL('t1'); a.ST('Tb')       # m 2^(xi - 1) = e^2 P^2 (Q13)
    a.SHR(2); a.ST('t0'); a.LDI(1024); a.MUL('t0'); a.ST('T')                 # T15 = 256 Tb
    a.LD('Tb'); a.ADD('Tb'); a.ST('Tb2')                                      # 2 Tb (Even's step change)
    a.LD('t0'); a.ST('t1'); a.LDI(124); a.MUL('t1'); a.ST('inc')              # 31 Tb (T15 - T14)
    # Log's floor: Tm = 6.25 hmax^2 (Q13) = (25 hmax^2 (Q16)) >> 5, hmax^2 (Q16) = max(XS^2 16384, YS^2)
    a.LD('xs2'); a.MULI(1024); a.MULI(16); a.ST('t0'); a.SUB('ys2'); a.SKIP('LT'); a.JMP('hmx')
    a.LD('ys2'); a.ST('t0')
    a.label('hmx'); a.LDI(25); a.MUL('t0'); a.SHR(5); a.ST('Tm')
    a.LDI(256); a.MUL('Tm'); a.ST('Tf')                                       # floor at k = 15: 256 Tm
    a.LDI(31); a.MUL('Tm'); a.ST('incf')
    a.LD('Tm'); a.ADD('Tm'); a.ST('Tm2')
    a.LD('lhm'); a.ADD('lhm'); a.ADD(C['c10830']); a.ST('lTm')               # log2 Tm (Q12, T in Q)
    a.LD('lTb'); a.ADD(C['c32768']); a.ST('lT')                              # log2 T15 = lTb + 8 (Q12)

    # ---- the ellipses, k = 15 down to 0.  phi_k = -(15 - k) PHI.
    a.LDI(0); a.ST('ph')
    a.LDI(15); a.ST('kk')
    a.label('loop')
    a.LD('kk'); a.OUT('KX')
    # cos, sin
    a.LD('ph'); a.ADD(C['c16384']); a.CALL('SINE'); a.ST('cs')
    a.LD('ph'); a.CALL('SINE'); a.ST('sn')
    # alpha, gamma, beta (Q13)
    a.LD('A1'); a.MUL('cs'); a.SHR(15); a.ADD('A0'); a.ST('al')
    a.LD('G1'); a.MUL('cs'); a.SHR(15); a.ST('t0'); a.LD('G0'); a.SUB('t0'); a.ST('ga')
    a.LD('B1'); a.MUL('sn'); a.SHR(15); a.ST('be')
    a.LD('al'); a.ADD('al'); a.ST('dd2')
    a.LD('ga'); a.ADD('ga'); a.ST('EE')
    a.LD('be'); a.ST('G')
    # T this ellipse: Even: T; Log: max(T, Tf).  Clamp to 2^36 - 1.
    a.LD('T'); a.ST('Tk'); a.LD('lT'); a.ST('lTk')
    a.SKIP('LOG'); a.JMP('tsel')
    a.LD('Tf'); a.SUB('T'); a.SKIP('GE'); a.JMP('tsel')
    a.LD('Tf'); a.ST('Tk')
    a.LD('kk'); rom_exp(L2B); a.ST('t0'); a.ADD('t0'); a.ADD('lTm'); a.ST('lTk')
    a.label('tsel')
    a.LD(C['cbig']); a.SUB('Tk'); a.SKIP('GE'); a.JMP('tbig'); a.JMP('tok')
    a.label('tbig'); a.LD(C['cbig']); a.ST('Tk')
    a.label('tok')
    # F0 = ((al N2 + be NM + ga M2) >> 4) - T
    a.LD('al'); a.MUL('N2'); a.ST('t0')
    a.LD('be'); a.MUL('NM'); a.ADD('t0'); a.ST('t0')
    a.LD('ga'); a.MUL('M2'); a.ADD('t0'); a.SHR(4); a.SUB('Tk'); a.ST('F0')
    # D0 = (al P1N - be m4) >> 2 ;  E0 = (ga P1M - be n4) >> 2
    a.LD('be'); a.MUL('m4'); a.ST('t0'); a.LD('al'); a.MUL('P1N'); a.SUB('t0'); a.SHR(2); a.ST('D0')
    a.LD('be'); a.MUL('n4'); a.ST('t0'); a.LD('ga'); a.MUL('P1M'); a.SUB('t0'); a.SHR(2); a.ST('E')
    # coverage shift: lU = 1 + lT / 2 + lhm (Q12; U = 2 e A hmax, Q per pixel);  t = round(lU) - 1 (Bold: v in
    # 1/16 px) or + 1 (Fine: v in 1/4 px), clamped to 0 .. 15
    a.LD('lTk'); a.SHR(1); a.ADD('lhm'); a.ADD(C['c4096']); a.ST('lU')
    a.ADDI(2047); a.ADDI(1); a.SHR(12); a.ST('rU'); a.ADDI(-1); a.SKIP('NFINE'); a.ADDI(2)
    a.ST('t0'); a.SKIP('LT'); a.JMP('tpos'); a.LDI(0); a.ST('t0')
    a.label('tpos'); a.LD('t0'); a.SUBI(15); a.SKIP('LT'); a.LDI(0); a.ADDI(15)
    a.ST('tk'); a.LDI(0); a.ST('hk')
    # Fine half-width in window units: H = HW4 2^(lU - 4096 rU) / 4096 + 2 (clamped to 15)
    a.SKIP('FINE'); a.JMP('hdone')
    a.LD('rU'); a.MULI(1024); a.MULI(4); a.ST('t0'); a.LD('lU'); a.SUB('t0'); a.SKIP('LT'); a.JMP('hpos')
    a.ADD(C['c4096']); a.SHR(4); rom_exp(EXPB); a.SHR(1); a.JMP('hm')
    a.label('hpos'); a.SHR(4); rom_exp(EXPB)
    a.label('hm'); a.MUL('hw4'); a.SHR(26); a.ADDI(2); a.SUBI(15); a.SKIP('LT'); a.LDI(0); a.ADDI(11)
    a.SKIP('GE'); a.LDI(0); a.ADDI(4)                                          # clamped to 4 .. 15
    a.ST('hk')
    a.label('hdone')
    # DDW = DD | t << 17 | H << 21
    a.LD('hk'); a.MULI(16); a.ADD('tk'); a.MULI(1024); a.MULI(128); a.ADD('dd2'); a.ST('DDW')
    # next: T and lT down one ellipse
    a.SKIP('LOG'); a.JMP('teven')
    a.LD('T'); a.SHR(10); a.ST('t0'); a.LDI(1250); a.MUL('t0'); a.SHR(1); a.ST('T')   # x rho^-2 = 0.6104 (1250/2048)
    a.LD('lT'); a.SUB(C['cl2r']); a.ST('lT')
    a.LD('Tf'); a.SUB('incf'); a.ST('Tf'); a.LD('incf'); a.SUB('Tm2'); a.ST('incf')
    a.JMP('tnext')
    a.label('teven')
    a.LD('T'); a.SUB('inc'); a.ST('T'); a.LD('inc'); a.SUB('Tb2'); a.ST('inc')
    a.LD('kk'); rom_exp(L2B - 1); a.ST('t0'); a.ADD('t0'); a.ADD('lTb'); a.ST('lT')  # next: lTb + 2 log2(kk)
    a.label('tnext')
    a.LD('ph'); a.SUB('phi'); a.ST('ph')
    a.LD('kk'); a.SUBI(1); a.ST('kk'); a.SKIP('LT'); a.JMP('loop')

    a.LDI(0); a.ST('first')
    a.HALT()

    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 512
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & ((1 << RFW) - 1)


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field and per-line arithmetic; precess.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf, pc=0, max_instr=200000):
    """inp: dict over INS names; flags: GLIDE, LOG, FINE, SRC (0/1); rf: register file (list of 512, updated in
    place; the per-ellipse slots land at 256 + 8 k + slot, i.e. the bank the engine writes).  Returns (outs,
    clocks): outs = dict of OUT side effects ('GST')."""
    acc = 0; lr = 0; adr = 0; x = 0
    outs = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}

    def addr(arg):
        if arg & 0x100:
            return 256 + 8 * x + (arg & 7)
        return arg & 0xFF
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        if op in (0, 1, 2, 3):
            b = _sx(rf[addr(arg)], RFW)
        elif op in (4, 5, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, RFW)
        elif op == 2:
            acc = _sx(acc - b, RFW)
        elif op == 6:
            acc = acc & arg
        elif op in (3, 7):
            acc = _sx(_sx(acc, 20) * _sx(b, 28), RFW)
            clocks += 19
        elif op == 8:
            rf[addr(arg)] = acc & ((1 << RFW) - 1)
        elif op == 9:
            n = arg & 15
            if n == INS['SIN']:
                acc = SIN[adr & 1023]
            elif n == INS['EXP']:
                acc = EXP[adr & 511]
            else:
                acc = inp[inv[n]]
        elif op == 10:
            n = arg & 15
            if n == OUTS['ADR']:
                adr = acc & 1023
            elif n == OUTS['KX']:
                x = acc & 15
            elif n == OUTS['GST']:
                outs['GST'] = acc & 0xFFFF
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'LOG', 3: 'FINE', 4: 'SRC'}[c]])
            if arg & 8:
                t = not t
            if t:
                npc = pc + 2
        elif op == 13:
            npc = arg & 1023
        elif op == 14:
            lr = pc + 1; npc = arg & 1023
        else:
            if arg == 0:
                npc = lr
            elif arg == 1:
                break
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    return outs, clocks


# ---------------------------------------------------------------------------------------------------------
def iir_shift(tid):
    """the camera's vertical one-pole shift: about 64 virtual lines in every mode"""
    if tid in (0, 8):
        return 4
    if tid in (5, 6, 14, 1, 2, 10, 4, 12):
        return 5
    return 6


def vhd_int_table(name, width, vals):
    return "    type t_%s is array (0 to %d) of unsigned(%d downto 0);\n    constant %s : t_%s := (%s);" % (
        name.lower(), len(vals) - 1, width - 1, name, name.lower(),
        ", ".join('to_unsigned(%d, %d)' % (v, width) for v in vals))


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
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by precess.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package precess_pkg is",
             vhd_table("C_PC_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_PC_RFI", "std_logic_vector", 512, 48, RF_INIT),
             vhd_table("C_PC_SIN", "std_logic_vector", 1024, 16, SIN),
             vhd_table("C_PC_EXP", "std_logic_vector", 512, 16, EXP),
             "    -- per timing ID (registers_in(8)(3 downto 0)): the engine's mode inputs and the camera's IIR shift"]
    mcs = [mode_consts(t) for t in range(16)]
    for key, w in (('XS', 3), ('YS', 11), ('IHY', 17), ('IHX', 16), ('X0', 14), ('LHM', 14), ('HW4', 16)):
        parts.append(vhd_int_table("C_PC_M" + key, w, [m[key] for m in mcs]))
    parts.append(vhd_int_table("C_PC_MSH", 3, [iir_shift(t) for t in range(16)]))
    parts.append(vhd_int_table("C_PC_MOYS", 10, [(m['YS'] * 121) >> 8 for m in mcs]))     # the Osc's line step
    parts += ["end package precess_pkg;", ""]
    with open(os.path.join(HERE, "precess_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
