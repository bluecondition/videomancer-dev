#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TRINE v0.1.0 (Vellum) - build hook.  Writes trine_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.vellum_work/rb_trine/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_TR_FT    512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)), 256 .. 271 2^k,
                        272 .. 287 the gasket's grey step per level round(876 * 64 / n), n = 1 .. 12
  C_TR_UC   1024 x 16   the per-field microprogram (see PROGRAM below)
  C_TR_RFI   256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                        constants)

The per-field engine is Hardline flute's: a 37-bit accumulator, a 128 x 32-bit register file, one adder, a
serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode + 12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

EXPB, P2B, FOGB = 0, 256, 272
FT = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
FT += [1 << k for k in range(16)]
FT += [int(round(876 * 64.0 / n)) if 1 <= n <= 12 else 0 for n in range(16)]
FT += [0] * (512 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=650, k5=680, k6=1023, p12=0)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, W=8, H=9, YS=10, ROM=11, CNT=12, XS=13, IVY=14, ZN=15)
CONDS = dict(LT=0, GLIDE=1, INK=2, SRC=3, GAS=4, GE=8, NGLIDE=9, NINK=10, NSRC=11, NGAS=12)
OUTS = dict(ADR=0, D3I=1, CINV=8, L1A=9, L1B=10, L1AI=11, L1BI=12,
            POUP=13, PODN=14, DU=15, DS=16, U0=17, S0=18, ZINV=19, W01S=20, W2S=21, GSH=22, G6=23, TA=24,
            T1=25, T2=26, HT=27)


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
            assert len(self.regs) <= 128
            self.init[name] = init
        return self.regs[name]

    def const(self, v):
        return self.reg("#%d" % v, v)

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
            return lambda r: self.emit(op, self.reg(r) if isinstance(r, str) else self.const(r))
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
    def abs_(self, tmp):
        d = self.new()
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def clamp_lo0(self):
        self.SKIP('GE'); self.LDI(0)


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up = the TOML defaults): knob holds, glides (Q6), drift phase
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h5', D['k5']), ('h6', D['k6']),
                 ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a5', D['k5']), ('a6', D['k6']),
                 ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('ph')

    a.JMP('main')

    # ---- EXPS: ACC = 2^(e / 256) * 2^-sh, e in t0 with (e >> 8) - base in 0 .. 15  (k = 15 - ... )
    #      t0 = e, t1 = base: result = T[e & 255] * 2^((e >> 8) - base) >> sh, sh in t2 (via 2^k from the ROM)
    a.label('EXPS')
    a.LD('t0'); a.SHR(8); a.ST('t3'); a.MULI(256); a.ST('t4'); a.LD('t0'); a.SUB('t4'); a.ADDI(EXPB); a.OUT('ADR')
    a.IN('ROM'); a.ST('t4')
    a.LD('t3'); a.SUB('t1'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('t4'); a.RET()

    a.label('main')
    a.IN('W'); a.ST('W'); a.IN('H'); a.ST('H'); a.IN('YS'); a.ST('Y'); a.IN('XS'); a.ST('XS')
    a.IN('ZN'); a.ADDI(1); a.ST('d'); a.IN('IVY'); a.ST('IVY')

    # ---- knob holds (move only by 2 counts or more, or to either end) and glides (Q6): Glide On = 1/16 of
    #      the gap per field (the Drift speed 1/8), never slower than 1/64 count, landing exactly; Off = the
    #      held value this field
    def hold(inp, h):
        take, keep = a.new(), a.new()
        a.IN(inp); a.SUB(h); a.ADDI(1); a.SKIP('GE'); a.JMP(take)        # moved down by 2 or more
        a.SUBI(3); a.SKIP('LT'); a.JMP(take)                             # moved up by 2 or more
        a.IN(inp); a.SUBI(1); a.SKIP('GE'); a.JMP(take)                  # at 0
        a.IN(inp); a.SUBI(1023); a.SKIP('LT'); a.JMP(take)               # at 1023
        a.JMP(keep)
        a.label(take); a.IN(inp); a.ST(h)
        a.label(keep)

    def glide(h, acc, sh):
        snap, big, pz, keep, st = (a.new() for _ in range(5))
        a.LD(h); a.MULI(64); a.SKIP('GLIDE'); a.JMP(st)                  # Off: the held value
        a.SUB(acc); a.ST('g')                                            # the gap
        a.abs_('t9'); a.SUBI(16); a.SKIP('LT'); a.JMP(big)
        a.LD('g'); a.SKIP('LT'); a.JMP(pz)
        a.LD(acc); a.SUBI(1); a.JMP(st)                                  # small gap: one step
        a.label(pz)
        a.LD('g'); a.SUBI(1); a.SKIP('GE'); a.JMP(keep)
        a.LD(acc); a.ADDI(1); a.JMP(st)
        a.label(keep)
        a.LD(acc); a.JMP(st)
        a.label(big)
        a.LD('g'); a.SHR(sh); a.ADD(acc)
        a.label(st)
        a.ST(acc)

    for inp, h, acc, sh in (('K1', 'h1', 'a1', 4), ('K2', 'h2', 'a2', 4), ('K3', 'h3', 'a3', 4),
                            ('K5', 'h5', 'a5', 3), ('K6', 'h6', 'a6', 4), ('P12', 'hp', 'ap', 4)):
        hold(inp, h); glide(h, acc, sh)

    # ---- DRIFT (Hypnos v1.1.0): v = K5 - 512; |v| < 24 still, else 2^-12 .. 2^-3.9 cycle per field,
    #      exponential (mantissa from the exp table, octave by 2^k); the phase wraps at one cycle (Q18)
    still, dpos, dwrap, ddone = a.new(), a.new(), a.new(), a.new()
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('t9'); a.SUBI(24); a.SKIP('GE'); a.JMP(still)
    a.ST('u'); a.SHR(4); a.ADD('u'); a.ST('ls'); a.SHR(6); a.ST('t5'); a.MULI(64); a.ST('t4')
    a.LD('ls'); a.SUB('t4'); a.MULI(4); a.ADDI(EXPB); a.OUT('ADR'); a.IN('ROM'); a.SHR(9); a.ST('t4')
    a.LD('t5'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('t4'); a.ST('spd')
    a.LD('v'); a.SKIP('LT'); a.JMP(dpos)
    a.LD('ph'); a.SUB('spd'); a.SKIP('GE'); a.ADD(1 << 18); a.JMP(dwrap)
    a.label(dpos)
    a.LD('ph'); a.ADD('spd'); a.ST('t4'); a.SUB(1 << 18); a.SKIP('GE'); a.LD('t4')
    a.label(dwrap)
    a.ST('ph')
    a.label(still)
    a.LD('ph'); a.SHR(10); a.ST('t8')                             # the zoom's octave fraction, Q8
    a.LD('ph'); a.SHR(4); a.ST('dph')                             # the bands' drift, Q12 (mod 4 bands)

    # ---- COMMON: pixel 0 / line 0 positions, the pan, the octaves of Scale
    a.LDI(7); a.SUB('W'); a.MUL('XS'); a.ST('X0')                 # q units: (xs / 2) (7 - W)
    a.LDI(1); a.SUB('H'); a.MUL('Y'); a.SHR(1); a.ST('Y0')        # Q8 lines
    a.LD('a1'); a.SUB(32768); a.MULI(175); a.SHR(11); a.ST('pxq') # +-700 virtual px, q units
    a.LD('a2'); a.SUB(32768); a.MULI(135); a.SHR(5); a.ST('py8')  # +-540 lines, Q8
    a.LD('a3'); a.MUL(24601); a.SHR(21); a.ST('oct')              # 0 .. 767, Q8 octaves (x8 over the knob)

    # ---- NEST scaling: the turn 1/u = 2^(2 P12 - 1) (Q12); 3 d / u (Q8); depth (1 + 1/u) (Q12 bands)
    a.LD('ap'); a.SHR(7); a.ADD(2816); a.ST('t0'); a.LDI(11); a.ST('t1'); a.CALL('EXPS'); a.SHR(4); a.ST('iu')
    a.LD('d'); a.MULI(3); a.ST('D3'); a.MUL('iu'); a.SHR(4); a.OUT('D3I')
    a.LD('d'); a.MULI(64); a.MULI(64); a.ST('dep'); a.MUL('iu'); a.SHR(12); a.ADD('dep'); a.OUT('CINV')
    a.LD(4096); a.SUB('dph'); a.OUT('POUP')
    a.LD(12288); a.SKIP('NINK'); a.LD(8192); a.SUB('dph'); a.OUT('PODN')

    # ---- THE LATTICE FRONT (shared): a + b = S per line, b - a = u along it, in units of the lattice
    #      triangle's height H (Q20).  Nest: H = the cell height (side 467 * 2^((oct - 488) / 256) virtual px),
    #      anchored at the pan; Gasket: H = the frame height S = 1080 * 2^((oct - 488) / 256) zoomed by
    #      2^t8, anchored at the apex (the triangle centred vertically, offset by Y)
    nf, ff = a.new(), a.new()
    a.LD('oct'); a.ADD(2092); a.ST('LS'); a.ADD('t8'); a.ST('LH'); a.ST('LHf')
    a.LD('py8'); a.ST('ay8')
    a.SKIP('GAS'); a.JMP(nf)
    a.LD('LS'); a.ST('t0'); a.LDI(8); a.ST('t1'); a.CALL('EXPS'); a.SHR(7)                       # S, vpx
    a.MULI(128); a.ST('t4'); a.LD('py8'); a.SUB('t4'); a.ST('ay8'); a.JMP(ff)
    a.label(nf)
    a.LD('oct'); a.ADD(1729); a.ST('LHf')                                           # the cell height
    a.label(ff)
    a.LD(5632); a.SUB('LHf'); a.ST('t0'); a.LDI(9); a.ST('t1'); a.CALL('EXPS'); a.SHR(6); a.ST('invH')  # Q22
    a.MUL(7094); a.SHR(12); a.ST('invH3')                         # sqrt 3 / H
    a.LD('XS'); a.MULI(2); a.MUL('invH3'); a.SHR(4); a.ST('du'); a.OUT('DU')     # Q20 per pixel
    a.LD('invH'); a.MUL('Y'); a.SHR(10); a.ST('dS'); a.OUT('DS')  # Q20 per line
    a.LD('X0'); a.SUB('pxq'); a.MUL('invH3'); a.SHR(4); a.OUT('U0')
    a.LD('Y0'); a.SUB('ay8'); a.SHR(2); a.MUL('invH'); a.SHR(8); a.OUT('S0')
    # the Nest's band gradients (L1, Q12 bands per pixel): the two slanted edges and the horizontal one
    a.LD('du'); a.ADD('dS'); a.MUL('D3'); a.SHR(9); a.ST('L1a'); a.OUT('L1A')
    a.LD('dS'); a.MUL('D3'); a.SHR(8); a.ST('L1b'); a.OUT('L1B')
    for src, out in (('L1a', 'L1AI'), ('L1b', 'L1BI')):
        ok = a.new()
        a.LD(src); a.MUL('iu'); a.SHR(12); a.ST('t4'); a.SUB(8191); a.SKIP('GE'); a.JMP(ok); a.LD(8191); a.ST('t4')
        a.label(ok)
        a.LD('t4'); a.OUT(out)

    # ---- GASKET (its frame, AA scale and level-table inputs; skipped in Nest)
    a.SKIP('GAS'); a.JMP('done')
    zi, zd = a.new(), a.new()
    a.LD('t8'); a.SUBI(1); a.SKIP('LT'); a.JMP(zi)
    a.LD(1 << 20); a.JMP(zd)
    a.label(zi)
    a.LD(256); a.SUB('t8'); a.ADDI(EXPB); a.OUT('ADR'); a.IN('ROM'); a.MULI(16)
    a.label(zd)
    a.OUT('ZINV')
    a.LD('du'); a.ADD('dS'); a.MULI(3); a.SHR(1); a.ST('w01'); a.LD('dS'); a.MULI(3); a.ST('w2')
    wm = a.new()
    a.LD('w01'); a.SUB('w2'); a.SKIP('LT'); a.JMP(wm); a.LD('w2'); a.ST('w01m'); a.JMP(wm + 'x')
    a.label(wm); a.LD('w01'); a.ST('w01m')
    a.label(wm + 'x')
    gz = a.new()
    a.LD('w01m'); a.NORM(); a.IN('CNT'); a.SUBI(6); a.SKIP('GE'); a.LDI(0); a.ST('gsh'); a.OUT('GSH')
    a.LDI(15); a.SUB('gsh'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.ST('t4')
    a.LD('w01'); a.MUL('t4'); a.SHR(15); a.OUT('W01S')
    a.LD('w2'); a.MUL('t4'); a.SHR(15); a.OUT('W2S')
    # the holes' turned edge u = 2^(1 - 2 P12) (Q14); the hole height in lines (Q4) at level 0
    a.LD(3840); a.ST('t0'); a.LD('ap'); a.SHR(7); a.ST('t4'); a.LD('t0'); a.SUB('t4'); a.ST('t0')
    a.LDI(13); a.ST('t1'); a.CALL('EXPS'); a.SHR(2); a.ST('uq')
    a.LD('LH'); a.ST('t0'); a.LDI(8); a.ST('t1'); a.CALL('EXPS'); a.SHR(7); a.MUL('IVY'); a.SHR(8); a.ST('hz')
    a.LD('d'); a.ADDI(FOGB); a.OUT('ADR'); a.IN('ROM'); a.ST('fs')                 # grey step per level, Q6
    a.LD('t8'); a.MUL('fs'); a.SHR(8); a.ADD(876 * 64); a.ST('g')                   # level 1's depth grey
    a.LD('uq'); a.MUL('t8'); a.SHR(7); a.ST('v2o')                                  # the opening level's T2
    a.LD('t8'); a.MULI(128); a.ST('v1o')                                            # ... and T1
    a.LD(32768); a.ST('v1'); a.LD('uq'); a.MULI(2); a.ST('v2')
    a.LD('d'); a.ADDI(1); a.ST('dl1'); a.LDI(1); a.ST('L')
    # ---- the per-level table (levels 1 .. depth + 1): C1 = (delta - T1) / 16 (T1 the hole's own edge),
    #      C2 = (delta + T2) / 32 (T2 the turned edge), HT (its depth tone x opacity: holes under 2 .. 4 lines fade)
    lp, o8, ok8, gd, gs, gx, reg, wr, t2k = (a.new() for _ in range(9))
    a.label(lp)
    a.LD('hz'); a.SHR(1); a.ST('hz'); a.SUBI(32); a.clamp_lo0(); a.SHR(2); a.ST('op')
    a.SUBI(8); a.SKIP('LT'); a.JMP(o8); a.JMP(ok8)
    a.label(o8); a.LDI(8); a.ST('op')
    a.label(ok8)
    a.LDI(876); a.SKIP('NINK'); a.JMP(gd)
    a.LD('g'); a.clamp_lo0(); a.ST('t4'); a.SUB(876 * 64); a.SKIP('LT'); a.JMP(gs); a.LD('t4'); a.JMP(gx)
    a.label(gs); a.LD(876 * 64)
    a.label(gx); a.SHR(6)
    a.label(gd)
    a.MUL('op'); a.SHR(3); a.ST('t4')
    a.LD('L'); a.OUT('TA'); a.LD('t4'); a.OUT('HT')
    # C1 = (delta - T1) / 16, C2 = (delta + T2) / 32, delta / 16 = v1 (the regular T1)
    a.SKIP('INK'); a.JMP(reg)
    a.LD('L'); a.SUB('dl1'); a.SKIP('GE'); a.JMP(reg)                               # L < depth + 1: regular
    a.LD('v1'); a.SUB('v1o'); a.OUT('T1'); a.LD('v1'); a.ADD('v2o'); a.SHR(1); a.OUT('T2'); a.JMP(wr)  # opening
    a.label(reg)
    a.LDI(0); a.OUT('T1')
    a.LD('v2'); a.ST('t4'); a.SUB(65535); a.SKIP('GE'); a.JMP(t2k); a.LD(65535); a.ST('t4')
    a.label(t2k)
    a.LD('t4'); a.ADD('v1'); a.SHR(1); a.OUT('T2')
    a.label(wr)
    for r in ('v1', 'v2', 'v1o', 'v2o'):
        a.LD(r); a.SHR(1); a.ST(r)
    a.LD('g'); a.SUB('fs'); a.ST('g')
    a.LD('L'); a.ADDI(1); a.ST('L'); a.SUB('dl1'); a.SUBI(1); a.SKIP('GE'); a.JMP(lp)
    a.label('done')
    a.LD('a6'); a.SHR(3); a.OUT('G6')                                               # camera gain, Q3
    # (the per-level table T1 / T2 / HT is filled in fabric from these after the halt)
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; trine.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


STRICT = False


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. raw knobs, W, H, YS, XS, IVY, ZN); flags: GLIDE, INK, SRC.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (outs dict by OUTS name, rf, clocks, tables) - tables = {L: (T1, T2, HT)}."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; cnt = 0; adr = 0; ta = 0
    outs = {}
    tabs = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        if op in (0, 1, 2, 3):
            b = _sx(rf[arg & 127], 32)
        elif op in (4, 5, 6, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, 37)
        elif op in (2, 6):
            acc = _sx(acc - b, 37)
        elif op in (3, 7):
            if STRICT:
                assert -131072 <= acc < 131072, ("MUL operand (acc) overflow", pc, acc)
                assert -131072 <= b < 131072, ("MUL operand (reg) overflow", pc, b)
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 17
        elif op == 8:
            rf[arg & 127] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            elif n == INS['CNT']: acc = cnt
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 511
            elif n == OUTS['TA']: ta = acc & 15
            elif n in (OUTS['T1'], OUTS['T2'], OUTS['HT']):
                t = list(tabs.get(ta, (0, 0, 0)))
                t[n - OUTS['T1']] = acc & 0xFFFF
                tabs[ta] = tuple(t)
                clocks += 0
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'INK', 3: 'SRC', 4: 'GAS'}[c]])
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
    return {inv_o[k]: v for k, v in outs.items()}, rf, clocks, tabs


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
    assert max(FT) < 65536
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by trine.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package trine_pkg is",
             vhd_table("C_TR_FT", "std_logic_vector", 512, 16, FT),
             vhd_table("C_TR_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_TR_RFI", "std_logic_vector", 256, 32, RF_INIT),
             "end package trine_pkg;", ""]
    with open(os.path.join(HERE, "trine_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
