#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CASSINI v0.1.0 (Vellum) - build hook.  Writes cassini_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  The integer model (.vellum_work/rb_cassini/imodel.py) imports this file and runs
the same per-field microprogram through emulate(), so that arithmetic has one source.

Package contents (block-RAM ROMs):
  C_CS_SRC    256 x 16   round(65536 * log2(1 + i / 256))          the per-field T table fill's source (x B/2);
                         entry 256, the last one's right neighbour, is 65536 (log2 2 = 1)
  C_CS_LM      64 x 6    round(64 * log2(1 + j / 64))              the AA logs' 6-bit mantissa (fabric)
  C_CS_FT    1024 x 16   the per-field engine's tables: 0 .. 256 a quarter sine (32768 sin(i pi / 512)),
                         257 .. 512 exp (32768 * 2^(j/256)), 513 .. 521 2^0 .. 2^8
  C_CS_UC     512 x 16   the per-field microprogram (see PROGRAM below)
  C_CS_RFI    256 x 36   the engine's register file at power-up (holds and glides at the TOML defaults)

The per-field engine (after Hardline flute): a 37-bit accumulator, a 64 x 36-bit register file, one adder, a
serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode + 12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [min(32767, int(round(32768 * math.sin(i * math.pi / 512.0)))) for i in range(256)]


def src_rom():
    return [int(round(65536 * math.log2(1 + i / 256.0))) for i in range(256)]


def lm_rom():
    return [min(63, int(round(64 * math.log2(1 + j / 64.0)))) for j in range(64)]


# the engine's table ROM
SINB, EXPB, P2B = 0, 257, 513
FT = sin_rom() + [32768] + exp2_rom() + [1 << k for k in range(9)]
FT += [0] * (1024 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=512, k4=512, k5=660, k6=1023, p12=512)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, XS=8, S2=9, LEFT=10, ROM=11, LPIX=13)
CONDS = dict(LT=0, GLIDE=1, TGT=2, SRC=3, GE=8, NGLIDE=9, NTGT=10, NSRC=11)
OUTS = dict(ADR=0, EST=1, KB=2, GST=3, A2=4, ZY=5, DY=6, IY=7, D0=8, LDX=9, LY=10, DLY=11, E0=12, D16=13)
RFW = 36


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
        """ACC = |ACC| (tmp is clobbered)"""
        d = self.new()
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def rom(self, base):
        """ACC = FT[ACC + base]"""
        if base:
            self.ADDI(base)
        self.OUT('ADR'); self.IN('ROM')


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults); 'first' < 0 on the first field
    for n, k in (('h1', 'k1'), ('h2', 'k2'), ('h3', 'k3'), ('h4', 'k4'), ('h5', 'k5'), ('h6', 'k6'), ('h12', 'p12')):
        a.reg(n, D[k])
    for n, k in (('g1', 'k1'), ('g2', 'k2'), ('g3', 'k3'), ('g4', 'k4'), ('g5', 'k5'), ('g6', 'k6'), ('g12', 'p12')):
        a.reg(n, D[k] * 8)
    a.reg('vgs', (D['k5'] - 512) * 64)
    a.reg('dl', 0)
    a.reg('first', -1)

    def sine():
        """ACC = sin(ACC) Q15, ACC = 0 .. 16384 a quarter turn: lo + ((hi - lo) fr) >> 6"""
        a.ST('ang'); a.SHR(6); a.ST('ix'); a.MULI(64); a.ST('t9'); a.LD('ang'); a.SUB('t9'); a.ST('fx')
        a.CALL('INTERP')

    a.JMP('main')

    # ---- INTERP: ACC = FT[ix] + ((FT[ix + 1] - FT[ix]) fx >> 6)
    a.label('INTERP')
    a.LD('ix'); a.OUT('ADR'); a.IN('ROM'); a.ST('v0'); a.LD('ix'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM')
    a.SUB('v0'); a.ST('dv'); a.LD('fx'); a.MUL('dv'); a.SHR(6); a.ADD('v0'); a.RET()

    # ---- CH: hold + glide of one knob (see the loop below)
    a.label('CH')
    a.LD('kin'); a.SUB('hh'); a.ADDI(1); a.SKIP('GE'); a.JMP('chtk')
    a.SUBI(3); a.SKIP('LT'); a.JMP('chtk')
    a.LD('first'); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUBI(1); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUBI(1023); a.SKIP('LT'); a.JMP('chtk')
    a.JMP('chgl')
    a.label('chtk'); a.LD('kin'); a.ST('hh')
    a.label('chgl')
    a.LD('hh'); a.MULI(8); a.ST('t1'); a.SUB('gg'); a.ST('t2')
    a.SKIP('GLIDE'); a.JMP('chsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('chsn')
    a.LD('t2'); a.abs_('t3')
    a.SUBI(65); a.SKIP('LT'); a.JMP('ches')
    a.ADDI(60); a.SKIP('LT'); a.JMP('chst')                # |gap| - 5
    a.label('chsn'); a.LD('t1'); a.ST('gg'); a.RET()
    a.label('ches'); a.LD('t2'); a.SHR(4); a.ADD('gg'); a.ST('gg'); a.RET()
    a.label('chst'); a.LDI(4); a.ST('t1'); a.LD('t2'); a.SKIP('GE'); a.LDI(-4); a.SKIP('LT'); a.LD('t1')
    a.ADD('gg'); a.ST('gg'); a.RET()

    a.label('main')
    a.IN('XS'); a.ST('xs'); a.IN('S2'); a.ST('s2'); a.IN('LEFT'); a.ST('left'); a.IN('LPIX'); a.ST('lpix')
    a.LDI(1024); a.MULI(32); a.ST('c32k')                                  # 32768
    a.LDI(1024); a.MULI(256); a.ST('c18')                                  # 2^18

    # ---- SCALE first (last field's glided K3; the T fill starts on the E step):  B/2 = 2^(oB/256),
    # oB = g3/8 - 384; the E step (B/2) 16384 = MB << (3 + sB), MB the 12-bit mantissa
    a.LD('g3'); a.SHR(3); a.SUBI(384); a.ST('oB'); a.SHR(8); a.ST('sB'); a.MULI(256); a.ST('t0')
    a.LD('oB'); a.SUB('t0'); a.rom(EXPB); a.SHR(4); a.ST('mb')
    a.LD('sB'); a.rom(P2B + 3); a.MUL('mb'); a.ST('est'); a.OUT('EST')
    a.LD('oB'); a.SHR(2); a.ADDI(162); a.ADD('lpix'); a.OUT('KB')

    # ---- knob holds + glides, one subroutine (CH) for all seven knobs: kin = the knob, hh = its hold,
    # gg = its glide (Q3).  Hold: the knob moves the held value only by 2 counts or more, to either end, or
    # on the first field.  Glide On: 1/16 of the gap per field, never less than half a count, landing
    # exactly; Glide off or the first field: the held value.  (K5's glide is unused: Drift has its own.)
    for inp, h, g in (('K1', 'h1', 'g1'), ('K2', 'h2', 'g2'), ('K3', 'h3', 'g3'), ('K4', 'h4', 'g4'),
                      ('K5', 'h5', 'g5'), ('K6', 'h6', 'g6'), ('P12', 'h12', 'g12')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(g); a.ST('gg'); a.CALL('CH')
        a.LD('hh'); a.ST(h); a.LD('gg'); a.ST(g)
    # ---- CAMERA gain step: K6 x 7 (Osc 2.5 x); starts the gain table fill
    a.LD('g6'); a.SHR(3); a.MULI(7); a.SKIP('NSRC'); a.MULI(5); a.SKIP('NSRC'); a.SHR(1)
    a.OUT('GST')

    # ---- DRIFT (Hypnos): speed glide 1/8 per field (Q6); |v| < 24 still, else 2^(ls/64 - 12) cycle per
    # field, ls = u + u/16 (at most 512), u = |v| - 24; the accumulator (Q18 cycles of 4 bands) wraps
    a.LD('h5'); a.SUBI(512); a.MULI(64); a.ST('t1')
    a.SKIP('GLIDE'); a.JMP('vsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('vsn')
    a.LD('t1'); a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('vgs'); a.JMP('vdec')
    a.label('vsn'); a.LD('t1'); a.ST('vgs')
    a.label('vdec')
    a.LD('vgs'); a.SHR(6); a.ST('v'); a.abs_('t3'); a.SUBI(24); a.SKIP('GE'); a.JMP('dd')
    a.ST('u'); a.SHR(4); a.ADD('u'); a.SUBI(512); a.SKIP('LT'); a.LDI(0); a.ADDI(512)
    a.ST('ls'); a.SHR(6); a.ST('k'); a.MULI(64); a.ST('t0'); a.LD('ls'); a.SUB('t0'); a.MULI(4); a.rom(EXPB)
    a.SHR(9); a.ST('m'); a.LD('k'); a.rom(P2B); a.MUL('m'); a.ST('spd')
    a.LD('v'); a.SKIP('LT'); a.JMP('fwd')
    a.LD('dl'); a.SUB('spd'); a.SKIP('GE'); a.ADD('c18'); a.JMP('dst')
    a.label('fwd')
    a.LD('dl'); a.ADD('spd'); a.ST('t0'); a.SUB('c18'); a.SKIP('GE'); a.LD('t0')
    a.label('dst'); a.ST('dl')
    a.label('dd'); a.LD('dl'); a.SHR(2); a.ST('d16'); a.OUT('D16')

    # ---- POSITION (h; Y in Q4 h) and the pre-pixel offsets
    a.LD('g1'); a.SUBI(2047); a.SUBI(129); a.ST('X')
    a.LD('left'); a.SUB('xs'); a.SUB('xs'); a.SUB('X'); a.ST('dx0')
    a.LD('g2'); a.SUBI(1936); a.MULI(8); a.ST('Y4'); a.LDI(0); a.SUB('Y4'); a.ST('dy0')

    # ---- ANGLE: (K4 - 512) past an 8-count detent, 65536 per turn (|th| <= 16380); cos >= 0, |sin|, sign
    a.LD('g4'); a.ADDI(-2048); a.ADDI(-2048); a.ST('dg'); a.abs_('t3'); a.SUBI(64); a.SKIP('GE'); a.LDI(0)
    a.ST('av'); a.SHR(4); a.ST('t0'); a.LD('av'); a.MULI(4); a.ADD('t0'); a.ST('th')
    sine(); a.ST('sn')
    a.LDI(1024); a.MULI(16); a.SUB('th'); sine(); a.ST('cs')

    # ---- SPREAD: below the default a_h = 536 (2^(p12/512) - 1); above it 536 2^(la/256), la 0 .. 434;
    # Targets: at least 128
    a.LD('g12'); a.ADDI(-2048); a.ADDI(-2048); a.SKIP('GE'); a.JMP('below')
    a.MULI(435); a.SHR(12); a.ST('la'); a.SHR(8); a.ST('t0'); a.MULI(256); a.ST('t1'); a.LD('la'); a.SUB('t1')
    a.rom(EXPB); a.ST('t1'); a.LD('t0'); a.rom(P2B); a.MUL('t1'); a.MULI(536); a.SHR(15); a.ST('ah')
    a.JMP('sdone')
    a.label('below')
    a.LD('g12'); a.SHR(4); a.rom(EXPB); a.SUB('c32k'); a.MULI(536); a.SHR(15); a.ST('ah'); a.LDI(0); a.ST('la')
    a.label('sdone')
    a.SKIP('TGT'); a.JMP('nofl'); a.LD('ah'); a.SUBI(128); a.SKIP('LT'); a.JMP('nofl'); a.LDI(128); a.ST('ah')
    a.label('nofl')

    # ---- the DDA constants.  ZA (h^2): line 0 (Q8) = (16 dx0)^2 + (16 a)^2 + dy0^2
    a.LD('ah'); a.MUL('ah'); a.OUT('A2')
    a.LD('dx0'); a.MULI(16); a.ST('t0'); a.MUL('t0'); a.ST('zy')
    a.LD('ah'); a.MULI(16); a.ST('t0'); a.MUL('t0'); a.ADD('zy'); a.ST('zy')
    a.LD('dy0'); a.MUL('dy0'); a.ADD('zy'); a.OUT('ZY')
    a.LD('dy0'); a.MULI(2); a.ADD('s2'); a.MUL('s2'); a.OUT('DY')               # s2 (2 dy0 + s2)
    a.LD('s2'); a.MULI(2); a.MUL('s2'); a.OUT('IY')                              # 2 s2^2
    a.LD('dx0'); a.MULI(2); a.ADD('xs'); a.MUL('xs'); a.OUT('D0')               # xs (2 dx0 + xs)
    # Lx = 2 a (dx cos + dy sin) (Q8 h^2).  Pixel step (a xs cos) >> 6.
    a.LD('ah'); a.MUL('xs'); a.MUL('cs'); a.SHR(6); a.OUT('LDX')
    # line 0, pixel 0: (a cos dx0) >> 6, a cos = 512 hi + lo
    a.LD('ah'); a.MUL('cs'); a.ST('t0'); a.SHR(9); a.ST('hi'); a.MULI(512); a.ST('t1'); a.LD('t0'); a.SUB('t1')
    a.MUL('dx0'); a.SHR(6); a.ST('t2'); a.LD('dx0'); a.MULI(8); a.ST('t1'); a.LD('hi'); a.MUL('t1'); a.ADD('t2')
    a.ST('ly')
    # + (+-a |sin| dy0) >> 10, a |sin| = 512 hs + ls: (hs sdy + ((ls sdy) >> 9)) >> 1, sdy = +-dy0
    a.LD('dy0'); a.ST('sdy'); a.LD('dg'); a.SKIP('LT'); a.JMP('sp'); a.LDI(0); a.SUB('dy0'); a.ST('sdy')
    a.label('sp')
    a.LD('ah'); a.MUL('sn'); a.ST('t0'); a.SHR(9); a.ST('hi'); a.MULI(512); a.ST('t1'); a.LD('t0'); a.SUB('t1')
    a.MUL('sdy'); a.SHR(9); a.ST('t2'); a.LD('hi'); a.MUL('sdy'); a.ADD('t2'); a.SHR(1); a.ADD('ly'); a.OUT('LY')
    # per line: (+-(a s2 / 2) |sin|) >> 9
    a.LD('s2'); a.SHR(1); a.MUL('ah'); a.ST('t0'); a.LD('dg'); a.SKIP('LT'); a.JMP('dp'); a.LDI(0); a.SUB('t0')
    a.ST('t0')
    a.label('dp'); a.LD('t0'); a.MUL('sn'); a.SHR(9); a.OUT('DLY')

    # ---- the E table's start: -(B/2) (36.28 + la/64) (Targets: -(B/2) 18.14) - 4 drift; starts the E fill
    a.LDI(1161); a.SKIP('TGT'); a.ADDI(1161); a.SKIP('TGT'); a.ADD('la'); a.MUL('est'); a.SHR(6); a.ST('t0')
    a.LDI(0); a.SUB('t0'); a.SUB('d16'); a.OUT('E0')
    a.LDI(0); a.ST('first')
    a.HALT()
    return a



_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & ((1 << RFW) - 1)


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; cassini.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=100000):
    """inp: dict over INS names (K1.. raw knobs, XS, S2, LEFT, LPIX); flags: GLIDE, TGT, SRC.
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
            b = _sx(rf[arg & 63], RFW)
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
            rf[arg & 63] = acc & ((1 << RFW) - 1)
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 1023
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'TGT', 3: 'SRC'}[c]])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 511
        elif op == 14:
            lr = pc + 1; npc = arg & 511
        else:
            if arg == 0: npc = lr
            elif arg == 1: break
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


def vhd_table_b(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of unsigned(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['"%s"' % format(v & ((1 << width) - 1), "0%db" % width) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert len(UCODE) <= 512, len(UCODE)
    uc = UCODE + [0xF001] * (512 - len(UCODE))
    parts = ["-- Generated by cassini.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package cassini_pkg is",
             vhd_table("C_CS_SRC", "unsigned", 256, 16, src_rom()),
             vhd_table_b("C_CS_LM", 64, 6, lm_rom()),
             vhd_table("C_CS_FT", "std_logic_vector", 1024, 16, FT),
             vhd_table("C_CS_UC", "std_logic_vector", 512, 16, uc),
             vhd_table("C_CS_RFI", "std_logic_vector", 256, 36, RF_INIT),
             "end package cassini_pkg;", ""]
    with open(os.path.join(HERE, "cassini_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
