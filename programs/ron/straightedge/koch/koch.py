#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
KOCH v0.1.0 (Straightedge) - build hook.  Writes koch_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.straightedge_work/rb_koch/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_KO_FT    512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)), 256 .. 271 2^k
  C_KO_UC   1024 x 16   the per-field microprogram (see PROGRAM below)
  C_KO_RFI   256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                        constants)

The per-field engine is Hardline flute's (as in Vellum trine): a 37-bit accumulator, a 128 x 32-bit register
file, one adder, a serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode +
12-bit operand.

Units.  The picture is computed in "heights": distances to straight lines, in LSBs of u = P / 2^k real pixels,
where P is the band period in real pixels and k is chosen per field so that u lies in [1/16, 1/8) pixel.  A
band is then exactly 2^k LSBs, so the band number and the distance to the nearest band edge are bit fields of
the fold distance and need no multiplier.  g = 1 / u (LSBs per real pixel, 8 .. 16) scales every length.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

EXPB, P2B = 0, 256
FT = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
FT += [1 << k for k in range(16)]
FT += [0] * (512 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=409, k4=731, k5=680, k6=1023, p12=1023)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, W=8, H=9, YS=10, ROM=11, CNT=12, XS=13)
CONDS = dict(LT=0, GLIDE=1, INK=2, FINE=3, DIVE=4, GE=8, NGLIDE=9, NINK=10, NFINE=11, NDIVE=12)
OUTS = dict(ADR=0, NK1=1, NK2=2, NK3=3, NK4=4, NK5=5, NK6=6, NK7=7, MD=8, NH=9, P0=10, DP=11, S0=12, DS=13,
            H0=14, DOFF=15, KB=16, MU=17, RW=18, HW=19, VX0=20, G6=21, HZ=22, HF=23)


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
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h4', D['k4']), ('h5', D['k5']),
                 ('h6', D['k6']), ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a4', D['k4']), ('a5', D['k5']),
                 ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('ph')

    a.JMP('main')

    # ---- EXPS: ACC = T[e & 255] * 2^((e >> 8) - base), e in t0, base in t1 ((e >> 8) - base in 0 .. 15)
    a.label('EXPS')
    a.LD('t0'); a.SHR(8); a.ST('t3'); a.MULI(256); a.ST('t4'); a.LD('t0'); a.SUB('t4'); a.ADDI(EXPB); a.OUT('ADR')
    a.IN('ROM'); a.ST('t4')
    a.LD('t3'); a.SUB('t1'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('t4'); a.RET()

    a.label('main')
    a.IN('W'); a.ST('W'); a.IN('H'); a.ST('H'); a.IN('YS'); a.ST('Y'); a.IN('XS'); a.ST('XS')

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
                            ('K4', 'h4', 'a4', 4), ('K5', 'h5', 'a5', 3), ('K6', 'h6', 'a6', 4),
                            ('P12', 'hp', 'ap', 4)):
        hold(inp, h); glide(h, acc, sh)

    # ---- DRIFT (Hypnos v1.1.0): v = K5 - 512; |v| < 24 still, else 2^-12 .. 2^-3.9 cycle per field,
    #      exponential (mantissa from the exp table, octave by 2^k); the phase wraps at one cycle (Q18).
    #      One cycle = four bands of stream (Flake and Dive) and one level (x3) of zoom (Dive).
    still, dpos, dwrap = a.new(), a.new(), a.new()
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

    # ---- the raster: xsl = 256 log2(xs / 2), R2 = 65536 * 2 / xs
    xdone = a.new()
    a.LDI(0); a.ST('xsl'); a.LD(65536); a.ST('R2')
    a.LD('XS'); a.SUBI(3); a.SKIP('GE'); a.JMP(xdone)
    a.LDI(150); a.ST('xsl'); a.LD(43691); a.ST('R2')
    a.LD('XS'); a.SUBI(4); a.SKIP('GE'); a.JMP(xdone)
    a.LDI(256); a.ST('xsl'); a.LD(32768); a.ST('R2')
    a.label(xdone)

    # ---- BANDS: P = 16 * 2^(K4 * 3.5 / 1023) virtual px (16 .. 181); in real px 2^(lpr / 256); k = kq + 4, u = P / 2^k
    #      g = 1 / u = 2^(4 - frac) (Q12), mu = 16 u * 16 (16 .. 31), G2 = 2 g / xs (LSBs per virtual px, Q12)
    gz, gd = a.new(), a.new()
    a.LD('a4'); a.MULI(7); a.SHR(9); a.ADDI(1024); a.SUB('xsl'); a.ST('lpr')
    a.SHR(8); a.ST('kq'); a.MULI(256); a.ST('t4'); a.LD('lpr'); a.SUB('t4'); a.ST('fr')
    a.ADDI(EXPB); a.OUT('ADR'); a.IN('ROM'); a.SHR(11); a.OUT('MU')
    a.LD('kq'); a.SUBI(3); a.OUT('KB')
    a.LD('fr'); a.SUBI(1); a.SKIP('LT'); a.JMP(gz)
    a.LD(65536); a.JMP(gd)
    a.label(gz)
    a.LDI(256); a.SUB('fr'); a.ADDI(EXPB); a.OUT('ADR'); a.IN('ROM')
    a.label(gd)
    a.ST('gq'); a.MUL('R2'); a.SHR(16); a.ST('G2')
    a.LD('kq'); a.ADDI(4 + P2B); a.OUT('ADR'); a.IN('ROM'); a.ST('P2k')            # one band, LSBs
    a.LD('kq'); a.ADDI(6 + P2B); a.OUT('ADR'); a.IN('ROM'); a.ST('P2k4')           # the tone cycle (4 bands)
    a.LD('ph'); a.SHR(4); a.MUL('P2k4'); a.SHR(14); a.OUT('DOFF')                  # the stream, LSBs
    # the rim (Grey / Fine: P / 4; Ink Bold: two pixels) and the Fine half width max(P / 12, one pixel)
    rwi, rwd, hwk = a.new(), a.new(), a.new()
    a.SKIP('NINK'); a.JMP(rwi)
    a.LD('P2k'); a.SHR(2); a.JMP(rwd)
    a.label(rwi)
    a.SKIP('NFINE'); a.JMP(rwi + 'f')
    a.LD('gq'); a.SHR(11); a.JMP(rwd)
    a.label(rwi + 'f')
    a.LD('P2k'); a.SHR(2)
    a.label(rwd)
    a.OUT('RW')
    a.LD('gq'); a.SHR(12); a.ST('g1')
    a.LD('P2k'); a.MUL(5461); a.SHR(16); a.ST('t4'); a.SUB('g1'); a.SKIP('LT'); a.JMP(hwk); a.LD('g1'); a.ST('t4')
    a.label(hwk)
    a.LD('t4'); a.OUT('HW')

    # ---- SCALE and DEPTH
    #      Flake: side S = 199 * 2^(5 K3 / 1023) virtual px (default 795); depth = P12^2 * min(7, log3(S / 2.9))
    #      Dive: S = 2100 * 3^lev, lev = (phase + the Scale offset) mod 1; depth = 6 P12^2 + lev
    dv, sd, dc7, dz = a.new(), a.new(), a.new(), a.new()
    a.LD('ap'); a.MUL('ap'); a.SHR(16); a.ST('psq')                 # P12 squared: the big generations get the travel
    a.SKIP('NDIVE'); a.JMP(dv)
    a.LD('a3'); a.MUL(20500); a.SHR(20); a.ADD(1955); a.ST('lS')
    a.SUBI(393); a.MUL(41347); a.SHR(16); a.ST('t4'); a.SUBI(1792); a.SKIP('GE'); a.JMP(dc7); a.LDI(1792); a.ST('t4')
    a.label(dc7)
    a.LD('psq'); a.MUL('t4'); a.SHR(16); a.ST('D8'); a.JMP(sd)
    a.label(dv)
    a.LD('a3'); a.SUB(26176); a.MULI(808); a.SHR(16); a.ST('t4'); a.LD('ph'); a.SHR(10); a.ADD('t4'); a.ST('t5')
    a.SHR(8); a.MULI(256); a.ST('t4'); a.LD('t5'); a.SUB('t4'); a.ST('lev')
    a.MULI(406); a.SHR(8); a.ADD(2825); a.ST('lS')
    a.LD('psq'); a.MULI(1536); a.SHR(16); a.ADD('lev'); a.ST('D8')
    a.label(sd)
    # S in real px (Q4) -> LSBs; the triangle's altitude H0 = S sqrt(3) / 2
    a.LD('lS'); a.SUB('xsl'); a.ADDI(1024); a.ST('t0'); a.LDI(10); a.ST('t1'); a.CALL('EXPS'); a.SHR(5)
    a.MUL('gq'); a.SHR(16); a.MUL(56756); a.SHR(16); a.ST('H0'); a.OUT('H0')
    a.LD('D8'); a.SHR(8); a.ST('n'); a.MULI(256); a.ST('t4'); a.LD('D8'); a.SUB('t4'); a.ST('f8')

    # ---- THE LEVELS: K_j = 2 H_(j-1) / 3 (out as -K; 0 = the level is off); mode 1 = whole, 2 = growing
    #      (truncated at h).  h: level 1: H' 2 f up to f = 1/2 (the spike is whole there), then
    #      H' 2^(12 f - 6) (its far echoes complete); levels 2 ..: H' f.
    #      HF = the altitude after the last level that is on; HZ = after level 1 (the echo field's tap).
    a.LDI(0); a.ST('md'); a.OUT('NH'); a.LD('H0'); a.ST('Hj'); a.ST('hf')
    for j in range(1, 8):
        full, grow, wr, hs, hk, on, nk = (a.new() for _ in range(7))
        a.LD('Hj'); a.MUL(21845); a.SHR(16); a.ST('Hj')
        if j == 1:
            a.OUT('HZ')
        a.LD('n'); a.SUBI(j); a.SKIP('LT'); a.JMP(full)
        a.ADDI(1); a.SKIP('GE'); a.JMP(wr)                                # j > n + 1: off
        a.LD('f8'); a.SUBI(1); a.SKIP('GE'); a.JMP(wr)                    # nothing growing
        if j == 1:
            a.LD('f8'); a.SUBI(129); a.SKIP('LT'); a.JMP(hs + 'e')
            a.LD('f8'); a.MULI(2); a.MUL('Hj'); a.SHR(8); a.JMP(hs)
            a.label(hs + 'e')
            a.LD('f8'); a.MULI(12); a.SUB(1536); a.ST('t0'); a.LDI(0); a.ST('t1'); a.CALL('EXPS'); a.SHR(7)
            a.MUL('Hj'); a.SHR(8)
        else:
            a.LD('f8'); a.MUL('Hj'); a.SHR(8)
        a.label(hs)
        a.ST('t4'); a.SUB(131071); a.SKIP('GE'); a.JMP(hk); a.LD(131071); a.ST('t4')
        a.label(hk)
        a.LDI(0); a.SUB('t4'); a.OUT('NH')
        a.LD(2 << (2 * (j - 1))); a.JMP(grow)
        a.label(full)
        a.LD(1 << (2 * (j - 1)))
        a.label(grow)
        a.ADD('md'); a.ST('md')
        a.LD('Hj'); a.ST('hf'); a.MULI(-2); a.JMP(nk)
        a.label(wr)
        a.LDI(0)
        a.label(nk)
        a.OUT('NK%d' % j)
    a.LD('md'); a.OUT('MD'); a.LD('hf'); a.OUT('HF')

    # ---- THE FRONT: p = a1 - a2 = sqrt(3) X along the line (the same on every line), s = a1 + a2 per line
    #      positions in quarter virtual px (x) and Q8 virtual lines (y), from the screen centre
    fl, fd = a.new(), a.new()
    a.LDI(-1); a.SUB('W'); a.MUL('XS'); a.ST('X0')                 # DDA pixel 0, quarter virtual px
    a.LDI(1); a.SUB('H'); a.MUL('Y'); a.SHR(1); a.ST('Y0')         # line 0, Q8 lines
    a.SKIP('NDIVE'); a.JMP(fl)
    # Flake: the centre at X (+-960), Y (+-540), minus the slide toward the right notch once S > 795
    a.LD('lS'); a.ADDI(512); a.ST('t0'); a.LDI(8); a.ST('t1'); a.CALL('EXPS'); a.SHR(7)     # 4 S
    a.SUB(3180); a.SKIP('GE'); a.LDI(0); a.MUL(21845); a.SHR(16); a.ST('t5')                # the slide, quarter px
    a.LD('a1'); a.SUB(32768); a.MULI(15); a.SHR(7); a.SUB('t5'); a.ST('cxq')               # 960 * 4 / 32768
    a.LD('a2'); a.SUB(32768); a.MULI(135); a.SHR(5); a.ST('cy8')                           # 540 * 256 / 32768
    a.LD('H0'); a.MUL(43691); a.SHR(16); a.ST('s2r'); a.JMP(fd)                            # 2 r = 2 H0 / 3
    a.label(fl)
    # Dive: the tip at X (+-600), Y (+-337) - 160
    a.LD('a1'); a.SUB(32768); a.MULI(75); a.SHR(10); a.ST('cxq')                           # 600 * 4 / 32768
    a.LD('a2'); a.SUB(32768); a.MULI(337); a.SHR(7); a.SUB(160 * 256); a.ST('cy8')         # 337 * 256 / 32768
    a.LDI(0); a.ST('s2r')
    a.label(fd)
    a.LD('gq'); a.MUL(1774); a.SHR(12); a.OUT('DP')                                        # sqrt(3) g, Q10
    a.LD('G2'); a.MUL(28378); a.SHR(16); a.ST('t4')                                        # sqrt(3) G2 / 4, Q12
    a.LD('X0'); a.SUB('cxq'); a.MUL('t4'); a.SHR(12); a.OUT('P0')
    a.LD('Y'); a.MUL('G2'); a.SHR(10); a.OUT('DS')                                         # per line, Q10
    a.LD('Y0'); a.SUB('cy8'); a.SHR(4); a.MUL('G2'); a.SHR(16); a.ADD('s2r'); a.OUT('S0')

    # ---- the star screen's first column (half virtual px) and the camera gain
    a.LD('W'); a.MUL('XS'); a.ST('t4'); a.LD(3840); a.SUB('t4'); a.SUB('XS'); a.SHR(1); a.clamp_lo0(); a.OUT('VX0')
    a.LD('a6'); a.SHR(3); a.OUT('G6')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; koch.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


STRICT = False


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. raw knobs, W, H, YS, XS); flags: GLIDE, INK, FINE, DIVE.
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
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'INK', 3: 'FINE', 4: 'DIVE'}[c]])
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
    assert max(FT) < 65536
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by koch.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package koch_pkg is",
             vhd_table("C_KO_FT", "std_logic_vector", 512, 16, FT),
             vhd_table("C_KO_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_KO_RFI", "std_logic_vector", 256, 32, RF_INIT),
             "end package koch_pkg;", ""]
    with open(os.path.join(HERE, "koch_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
