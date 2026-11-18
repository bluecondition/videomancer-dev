#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SPLINTER v0.1.0 (Straightedge) - build hook.  Writes splinter_pkg.vhd next to this file (pure python: the
builder runs it with the system python3).  The integer model (.straightedge_work/rb_splinter/imodel.py)
imports this file and runs the same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_SP_FT    512 x 16   the per-field engine's table: EXPB 128 x round(16384 * 2^(i/128)); CUB / SUB / RHB 65
                        points of cos(phi w) , sin(phi w), sqrt5^-phi (Q15, w = atan(1/2)); P2B 2^k
  C_SP_UC   1024 x 16   the per-field microprogram (see PROGRAM below)
  C_SP_RFI   256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                        constants)

The per-field engine is Hardline flute's (as in Vellum trine): a 37-bit accumulator, a 128 x 32-bit register
file, one adder, a serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode +
12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

U0 = 7400.0                     # the top tile's short leg in virtual px at zoom phase 0
OMEGA = math.atan(0.5)
R5 = math.sqrt(5.0)

EXPB, CUB, SUB_, RHB, IRB, P2B = 0, 128, 193, 258, 323, 388
FT = [int(round(16384 * 2 ** (i / 128.0))) for i in range(128)]
FT += [int(round(32768 * math.cos(i / 64.0 * OMEGA))) for i in range(65)]
FT += [int(round(32768 * math.sin(i / 64.0 * OMEGA))) for i in range(65)]
FT += [int(round(32768 * R5 ** (-i / 64.0))) for i in range(65)]
FT += [int(round(16384 * R5 ** (i / 64.0))) for i in range(65)]
FT += [1 << k for k in range(16)]
FT += [0] * (512 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=512, k4=200, k5=660, k6=1023, p12=300)

KDDA = int(round(2 ** 28 / U0))                     # Q28 top units per virtual px at phase 0
KU3 = int(round(2 ** 14 * R5 ** 3 / U0 * 64))       # level 3: 2.14 LSB per virtual px at phase 0, Q6
GCAP = 5898                                         # 0.36 units (2.14)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, W=8, H=9, YS=10, ROM=11, XS=13, AA=14)
CONDS = dict(LT=0, GLIDE=1, ALL=2, SRC=3, GE=8, NGLIDE=9, NALL=10, NSRC=11)
OUTS = dict(ADR=0, X0=1, Y0=2, DXX=3, DYX=4, DXY=5, DYY=6, PHI=7, LH=8, AT=9, TA=10, TW=11, PK7=12, PK8=13,
            RA=14, RX=15, RY=16, EX=17, EY=18, G6=19, KICK=20, RS=21)


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
            assert len(self.regs) <= 128, "register file full"
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
    # state that lives across fields (power-up = the TOML defaults): knob holds, glides (Q6), zoom phase
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h4', D['k4']), ('h5', D['k5']),
                 ('h6', D['k6']), ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a4', D['k4']), ('a5', D['k5']),
                 ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('ph')

    a.JMP('main')

    # ---- TERP: ACC = table base (in ACC) interpolated at the phase: T[i] + (T[i + 1] - T[i]) * tf >> 12
    a.label('TERP')
    a.ADD('ti'); a.ST('t6'); a.OUT('ADR'); a.IN('ROM'); a.ST('t5')
    a.LD('t6'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM'); a.SUB('t5'); a.MUL('tf'); a.SHR(12); a.ADD('t5'); a.RET()

    # ---- GENT: one kerf-table entry.  ACC = the table address; t0 = W (Q4 virtual px), t1 = u (Q6 LSB per px)
    #      G = min(W * u >> 10 + gam, GCAP)
    gcap = a.new()
    a.label('GENT')
    a.OUT('TA'); a.LD('t0'); a.MUL('t1'); a.SHR(10); a.ADD('gam'); a.ST('t2'); a.SUB(GCAP); a.SKIP('LT'); a.JMP(gcap)
    a.LD('t2'); a.OUT('TW'); a.RET()
    a.label(gcap)
    a.LD(GCAP); a.OUT('TW'); a.RET()

    a.label('main')
    a.IN('W'); a.ST('W'); a.IN('H'); a.ST('H'); a.IN('YS'); a.ST('Y'); a.IN('XS'); a.ST('XS'); a.IN('AA'); a.ST('AA')

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
        big, pz, keep, st = (a.new() for _ in range(4))
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

    # ---- DRIFT (Hypnos v1.1.0): v = K5 - 512; |v| < 24 still, else 2^-12 .. 2^-3.9 level per field,
    #      exponential; the zoom phase wraps at one level (Q18): one substitution step
    still, dpos, dwrap = a.new(), a.new(), a.new()
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('t9'); a.SUBI(24); a.SKIP('GE'); a.JMP(still)
    a.ST('u'); a.SHR(4); a.ADD('u'); a.ST('ls'); a.SHR(6); a.ST('t5'); a.MULI(64); a.ST('t4')
    a.LD('ls'); a.SUB('t4'); a.MULI(2); a.ADDI(EXPB); a.OUT('ADR'); a.IN('ROM'); a.SHR(8); a.ST('t4')
    a.LD('t5'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('t4'); a.ST('spd')
    a.LD('v'); a.SKIP('LT'); a.JMP(dpos)
    a.LD('ph'); a.SUB('spd'); a.SKIP('GE'); a.ADD(1 << 18); a.JMP(dwrap)
    a.label(dpos)
    a.LD('ph'); a.ADD('spd'); a.ST('t4'); a.SUB(1 << 18); a.SKIP('GE'); a.LD('t4')
    a.label(dwrap)
    a.ST('ph')
    a.label(still)
    a.LD('ph'); a.SHR(10); a.ST('p8'); a.OUT('PHI')              # the phase in 1/256 level
    a.LD('ph'); a.SHR(12); a.ST('ti'); a.MUL(4096); a.ST('t4'); a.LD('ph'); a.SUB('t4'); a.ST('tf')

    # ---- the spiral at this phase: unit vector (cu, su), rho = sqrt5^-phi (Q15)
    a.LDI(CUB); a.CALL('TERP'); a.ST('cu')
    a.LDI(SUB_); a.CALL('TERP'); a.ST('su')
    a.LDI(RHB); a.CALL('TERP'); a.ST('rho')

    # ---- THE VIEW: top-tile coordinates (Q28) = 1/2 + (P - eye) (cu - i su) rho / U0
    a.LD('cu'); a.MUL('rho'); a.SHR(15); a.MUL(KDDA); a.SHR(15); a.ST('gx')      # per virtual px, Q28
    a.LD('su'); a.MUL('rho'); a.SHR(15); a.MUL(KDDA); a.SHR(15); a.ST('gy')
    a.LD('gx'); a.MUL('XS'); a.SHR(1); a.OUT('DXX')
    a.LD('gy'); a.MUL('XS'); a.SHR(1); a.ST('t4'); a.LDI(0); a.SUB('t4'); a.OUT('DYX')
    a.LD('gy'); a.MUL('Y'); a.SHR(8); a.OUT('DXY')
    a.LD('gx'); a.MUL('Y'); a.SHR(8); a.OUT('DYY')
    # the eye (quarter virtual px from the screen centre): +-480, +-270 virtual px
    a.LD('a1'); a.SUB(32768); a.MULI(480); a.SHR(13); a.ST('exq')
    a.LD('a2'); a.SUB(32768); a.MULI(270); a.SHR(13); a.ST('eyq')
    a.LD('exq'); a.SHR(2); a.ADDI(960); a.OUT('EX')
    a.LD('eyq'); a.SHR(2); a.ADDI(540); a.OUT('EY')
    # DDA pixel 0 (one pixel left of column 0) on line 0, relative to the eye, quarter virtual px
    a.LDI(-1); a.SUB('W'); a.MUL('XS'); a.SUB('exq'); a.ST('xr')
    a.LDI(1); a.SUB('H'); a.MUL('Y'); a.SHR(7); a.SUB('eyq'); a.ST('yr')
    a.LD('xr'); a.MUL('gx'); a.ST('t2'); a.LD('yr'); a.MUL('gy'); a.ADD('t2'); a.SHR(2); a.ADD(1 << 27); a.OUT('X0')
    a.LD('yr'); a.MUL('gx'); a.ST('t2'); a.LD('xr'); a.MUL('gy'); a.ST('t3'); a.LD('t2'); a.SUB('t3'); a.SHR(2)
    a.ADD(1 << 27); a.OUT('Y0')
    a.OUT('KICK')

    # ---- THE SHATTER THRESHOLDS (1/256 level above the floor): LH = the great size (2.5 .. 3.5 levels),
    #      AT = LH - (LH + 320) P12; both go out less 128
    a.LD('a3'); a.SHR(8); a.ADDI(640); a.ST('LH'); a.SUBI(128); a.OUT('LH')
    a.LD('ap'); a.SHR(6); a.ST('t4'); a.LD('LH'); a.ADDI(320); a.MUL('t4'); a.SHR(10); a.ST('t4')
    a.LD('LH'); a.SUB('t4'); a.SUBI(128); a.OUT('AT')

    # ---- THE KERF: half-width 1 .. 6 virtual px (Q4) + a proportional inset 0.17 k^3 units (2.14)
    a.LD('a4'); a.SHR(6); a.ST('k4'); a.MULI(80); a.SHR(10); a.ADDI(16); a.ST('g4')
    a.LD('k4'); a.MUL('k4'); a.SHR(10); a.MUL('k4'); a.SHR(10); a.MUL(2785); a.SHR(10); a.ST('gam')
    # ---- the per-level table: [s][0] = 2.14 LSB per real pixel (s = 3 .. 8)
    a.LD('rho'); a.MUL(KU3); a.SHR(15); a.ST('us'); a.ST('ur')    # level 3, Q6 LSB per virtual px
    for s in range(3, 9):
        a.LDI(s * 16); a.OUT('TA'); a.LD('us'); a.MUL('AA'); a.SHR(7); a.OUT('TW')
        if s == 7:
            a.SHR(4); a.OUT('PK7')
        if s == 8:
            a.SHR(4); a.OUT('PK8')
        if s < 8:
            a.LD('us'); a.MUL(18318); a.SHR(13); a.ST('us')        # x sqrt 5
    # ---- [s][r] = the kerf of a rank-r edge at level s (r = 1 .. 8, s = max(r, 3) .. 7; r = 8: s = 8).
    #      Lines = All: heavier for the edges of bigger triangles (x1 .. x6 by the size of the rank's level)
    rl, fin, lo, hi, il, nr = (a.new() for _ in range(6))
    a.LDI(1); a.ST('r'); a.LDI(3); a.ST('s'); a.LDI(5 * 256 + 563); a.ST('eb')
    a.label(rl)
    a.LD('g4'); a.SKIP('ALL'); a.JMP(fin)
    a.LD('p8'); a.ADD('eb'); a.SUB('LH'); a.MULI(115); a.SHR(8)                 # the exponent, Q7
    a.SKIP('LT'); a.JMP(lo); a.LDI(0)
    a.label(lo)
    a.ST('t0'); a.SUBI(331); a.SKIP('GE'); a.JMP(hi); a.LDI(331); a.ST('t0')
    a.label(hi)
    a.LD('t0'); a.SHR(7); a.ST('t3'); a.MULI(128); a.ST('t4'); a.LD('t0'); a.SUB('t4'); a.ADDI(EXPB); a.OUT('ADR')
    a.IN('ROM'); a.ST('t4'); a.LD('t3'); a.ADDI(P2B); a.OUT('ADR'); a.IN('ROM'); a.MUL('t4'); a.SHR(2)   # Q12
    a.MUL('g4'); a.SHR(12)
    a.label(fin)
    a.ST('t0')
    a.LD('ur'); a.ST('t1'); a.LD('s'); a.MULI(16); a.ADD('r'); a.ST('ad')
    a.label(il)
    a.LD('ad'); a.CALL('GENT')
    a.LD('t1'); a.MUL(18318); a.SHR(13); a.ST('t1')
    a.LD('ad'); a.ADDI(16); a.ST('ad'); a.SUBI(128); a.SKIP('GE'); a.JMP(il)
    a.LD('r'); a.ADDI(1); a.ST('r'); a.LD('eb'); a.SUBI(256); a.ST('eb')
    a.LD('r'); a.SUBI(4); a.SKIP('GE'); a.JMP(nr)
    a.LD('r'); a.ST('s'); a.LD('ur'); a.MUL(18318); a.SHR(13); a.ST('ur')
    a.label(nr)
    a.LD('r'); a.SUBI(9); a.SKIP('GE'); a.JMP(rl)

    # ---- camera gain (Osc x3)
    g3 = a.new()
    a.LD('a6'); a.SHR(9); a.SKIP('SRC'); a.JMP(g3); a.MULI(3)
    a.label(g3)
    a.OUT('G6')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; splinter.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


STRICT = False


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. raw knobs, W, H, YS, XS, AA); flags: GLIDE, ALL, SRC.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (outs dict by OUTS name, rf, clocks, tab, rot): tab = {addr: value}, rot = {addr: (gx, gy)}."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ta = 0; ra = 0; rx = 0
    outs = {}
    tab = {}
    rot = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    kick = None
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
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 511
            elif n == OUTS['TA']: ta = acc & 255
            elif n == OUTS['TW']: tab[ta] = acc & 0xFFFF
            elif n == OUTS['RA']: ra = acc & 31
            elif n == OUTS['RX']: rx = _sx(acc, 10)
            elif n == OUTS['RY']: rot[ra] = (rx, _sx(acc, 10))
            elif n == OUTS['KICK']: kick = clocks
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'ALL', 3: 'SRC'}[c]])
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
    inv_o = {v: k for k, v in OUTS.items()}
    o = {inv_o[k]: v for k, v in outs.items()}
    o['_kick_clock'] = kick
    return o, rf, clocks, tab, rot


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
    parts = ["-- Generated by splinter.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package splinter_pkg is",
             vhd_table("C_SP_FT", "std_logic_vector", 512, 16, FT),
             vhd_table("C_SP_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_SP_RFI", "std_logic_vector", 256, 32, RF_INIT),
             "end package splinter_pkg;", ""]
    with open(os.path.join(HERE, "splinter_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
