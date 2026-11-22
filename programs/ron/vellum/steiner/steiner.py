#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
STEINER v0.1.0 (Vellum) - build hook.  Writes steiner_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.vellum_work/rb_steiner/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_ST_TAB  512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)) (address 512 wraps to entry 0 = 32768 = sin(pi/2))
  C_ST_UC   1024 x 16  the per-field microprogram (PROGRAM below)
  C_ST_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults, constants)
  C_ST_Y    512 x 10   tone: index = mode & level & class & summed coverage (see y_rom)

The per-field engine is Hardline flute's (as in vellum/eclipse): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial 18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift; micro-op =
4-bit opcode + 12-bit operand; plus a two-deep return stack.  It runs once per field, through the whole field,
and writes the NEXT field's circles into a parameter RAM (double buffered: the copy at the field start moves them
into the line engine and the pixel slots).

Geometry (unit great circle, eccentricity a along +x, chain phase al in turns):
  s = sin(pi / n), rho0 = (1 - s) / (1 + s), r0 = (1 - rho0) / 2, h = 1 / s
  chain circle k: E = 1 + 2 a cos al_k + a^2, b = (((1 + a^2) cos al_k + 2 a) / E, (1 - a^2) sin al_k / E),
                  r = (1 - a^2) / ((1 - a^2) + h E), centre (1 - r) b               (al_k = al + k / n)
  inner circle:   through (rho0 + a) / (1 + a rho0) and (a - rho0) / (1 - a rho0)
  the figure turns by beta about the great circle's centre; Nested draws the same construction inside the inner
  circle (n1 + n2 <= NCH discs in all, phase -al, axis -beta).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

NCH = 12                     # chain slots (level 0 + level 1 + the nested inner circle share them)
NSLOT = NCH + 2              # slot 0 outer, 1 inner, 2 .. chain (Nested: 2 = the nested inner circle)
DEFAULTS = dict(k1=512, k2=512, k3=512, k5=680, k6=1023, p12=512)
A_MAX = 55706                # a at P12 = 100 %: 0.85 (Q16)
CAM_LOG = 4262               # 256 log2(8191 / 0.08): C_max = 0.08 R^2 at K6 100 %, white
CLS_TH = (635, 973, 1485)    # tone-class thresholds, x r0 / 1024 (0.62, 0.95, 1.45)

ROLE_OFF, ROLE_OUTER, ROLE_INNER, ROLE_NINNER, ROLE_CH0, ROLE_CH1 = 0, 1, 2, 3, 4, 5


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    return e + s


TAB = tab_rom()

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's, with a two-deep return stack)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, XS=8, YS2=9, XPRE=10, ROM=11, Y0=12, NZ=13,
           ZERO=15)
CONDS = dict(LT=0, GLIDE=1, NEST=2, GE=8, NGLIDE=9, NNEST=10)
OUTS = dict(ADR=0, PA=1, PW=2)


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
            assert len(self.regs) <= 256
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
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def mov(self, src, dst):
        self.LD(src); self.ST(dst)

    def mul(self, a, b, sh, dst=None, rnd=None):
        """dst = (a * b [+ rnd]) >> sh  (a's low 18 bits x b's low 18 bits, both signed)."""
        self.LD(a); self.MUL(b)
        if rnd:
            self.ADD(rnd)
        if sh:
            self.SHR(sh)
        if dst:
            self.ST(dst)

    def ldc(self, v, tmp=None):
        """ACC = v for any v that fits 12 bits; larger constants come from registers."""
        if -2048 <= v <= 2047:
            self.LDI(v)
        else:
            self.LD(self.const(v))

    def const(self, v):
        n = 'C%d' % v if v >= 0 else 'Cm%d' % -v
        self.reg(n, v & 0xFFFFFFFF)
        return n


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h5', D['k5']), ('h6', D['k6']),
                 ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a5', D['k5']), ('a6', D['k6']),
                 ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('al0', 671089)                                      # 0.04 turn
    a.reg('bt', int(16777216 * 200 / 360) & 0xFFFFFF)       # the axis starts at 200 degrees
    for v in (1 << 24, 1 << 22, 65536, 32768, 16384, 1 << 19, 1 << 17, 1 << 15):
        a.const(v)

    a.JMP('main')

    # ---- WRAP: ACC mod 2^24 (one turn), for ACC in [-2^24, 2^25)
    a.label('WRAP')
    a.ST('t'); a.SKIP('LT'); a.JMP('wrhi'); a.ADD('C16777216'); a.RET()
    a.label('wrhi')
    a.SUB('C16777216'); a.SKIP('GE'); a.ADD('C16777216'); a.RET()

    # ---- SINQ: ACC = 32768 sin(2 pi phi / 2^24), phi in ACC (0 .. 2^24 - 1): quarter table, 14-bit interpolation
    a.label('SINQ')
    a.ST('sp'); a.SHR(22); a.ST('q'); a.LD('sp'); a.SHR(14); a.ST('a10')
    a.LD('q'); a.MULI(256); a.ST('x0'); a.LD('a10'); a.SUB('x0'); a.ST('aa')
    a.LD('a10'); a.MULI(128); a.MULI(128); a.ST('x0'); a.LD('sp'); a.SUB('x0'); a.ST('fr')
    a.LD('q'); a.SHR(1); a.ST('qh'); a.MULI(2); a.ST('x0'); a.LD('q'); a.SUB('x0')
    a.SUBI(1); a.SKIP('GE'); a.JMP('sev')
    a.LD('fr'); a.SUBI(1); a.SKIP('LT'); a.JMP('snz')
    a.LDI(256); a.SUB('aa'); a.ST('a2_'); a.LDI(0); a.ST('f2'); a.JMP('slk')
    a.label('snz')
    a.LDI(255); a.SUB('aa'); a.ST('a2_'); a.LD('C16384'); a.SUB('fr'); a.ST('f2'); a.JMP('slk')
    a.label('sev')
    a.LD('aa'); a.ST('a2_'); a.LD('fr'); a.ST('f2')
    a.label('slk')
    a.LD('a2_'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('a2_'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('x0')
    a.LD('f2'); a.MUL('x0'); a.SHR(14); a.ADD('s0'); a.ST('x0')
    a.LD('qh'); a.SUBI(1); a.SKIP('GE'); a.JMP('spos')
    a.LDI(0); a.SUB('x0'); a.RET()
    a.label('spos')
    a.LD('x0'); a.RET()

    # ---- DIVQ: ACC = floor(|dn| 2^(dit - 1) / dd) with dn's sign; needs |dn| < 2 dd, dd > 0 (restoring)
    a.label('DIVQ')
    a.LDI(0); a.ST('dsg'); a.ST('dq'); a.LD('dn'); a.SKIP('LT'); a.JMP('dvl')
    a.LDI(1); a.ST('dsg'); a.LDI(0); a.SUB('dn'); a.ST('dn')
    a.label('dvl')
    a.LD('dn'); a.SUB('dd'); a.SKIP('LT'); a.JMP('dvg')
    a.LD('dq'); a.ADD('dq'); a.ST('dq'); a.LD('dn'); a.ADD('dn'); a.ST('dn'); a.JMP('dvn')
    a.label('dvg')
    a.ST('dn'); a.LD('dq'); a.ADD('dq'); a.ADDI(1); a.ST('dq'); a.LD('dn'); a.ADD('dn'); a.ST('dn')
    a.label('dvn')
    a.LD('dit'); a.SUBI(1); a.ST('dit'); a.SUBI(1); a.SKIP('LT'); a.JMP('dvl')
    a.LD('dsg'); a.SUBI(1); a.SKIP('LT'); a.JMP('dvm')
    a.LD('dq'); a.RET()
    a.label('dvm')
    a.LDI(0); a.SUB('dq'); a.RET()

    def div(num, den, it, dst):
        """dst = DIVQ(num, den, it); num / den are register names (or a callable that leaves ACC)."""
        a.LD(num); a.ST('dn'); a.LD(den); a.ST('dd'); a.LDI(it); a.ST('dit'); a.CALL('DIVQ'); a.ST(dst)

    # ---- NCONST: per-count constants for n = nn: stp = 2^24 / n, hh = 1/s (Q14), rho0 (Q16), r0 (Q16),
    #      the tone-class thresholds t1 .. t3 (Q16)
    a.label('NCONST')
    a.LDI(1); a.ST('dn'); a.LD('nn'); a.ST('dd'); a.LDI(25); a.ST('dit'); a.CALL('DIVQ'); a.ST('stp')
    a.SHR(1); a.CALL('SINQ'); a.ST('sn')
    a.LD('C32768'); a.SUB('sn'); a.ST('dn'); a.LD('C32768'); a.ADD('sn'); a.ST('dd'); a.LDI(17); a.ST('dit')
    a.CALL('DIVQ'); a.ST('rho0')
    a.LD('C65536'); a.SUB('rho0'); a.SHR(1); a.ST('r0')
    a.LD('C32768'); a.ST('dn'); a.LD('sn'); a.ADD('sn'); a.ST('x0'); a.ADD('x0'); a.ST('dd'); a.LDI(17)
    a.ST('dit'); a.CALL('DIVQ'); a.ST('hh')
    for i, c in enumerate(CLS_TH):
        a.LD('r0'); a.MULI(c); a.SHR(10); a.ST('t%d' % (i + 1))
    a.RET()

    # ---- CIRCLE: one slot from (cX, cY, cr) in the current figure's unit frame (Q16) + crole, ccls.
    #      Figure: centre (FOx units, FOy2 Q2 units), radius FR (Q4 units), axis (Fc, Fs) Q15.
    a.label('CIRCLE')
    a.mul('cX', 'Fc', 0, 'x0'); a.mul('cY', 'Fs', 0); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.SHR(15); a.ST('Xr')
    a.mul('cX', 'Fs', 0, 'x0'); a.mul('cY', 'Fc', 0); a.ADD('x0'); a.SHR(15); a.ST('Yr')
    a.LD('Xr'); a.MUL('FR'); a.ADD('C524288'); a.SHR(20); a.ADD('FOx'); a.ST('cCx')
    a.LD('Yr'); a.MUL('FR'); a.ADD('C131072'); a.SHR(18); a.ADD('FOy2'); a.ST('cCy2')
    a.LD('cr'); a.MUL('FR'); a.ADD('C32768'); a.SHR(16); a.ST('cr16')
    # V4 = 16 dx^2 + dy^2 - cr16^2 / 16 (Q4 units^2, modular), U4 = 2 dy ys2 + ys2^2
    a.IN('XPRE'); a.SUB('cCx'); a.ST('x0'); a.MUL('x0'); a.ST('x1')
    for _ in range(4):
        a.ADD('x1'); a.ST('x1')
    a.IN('Y0'); a.SUB('cCy2'); a.ST('dy'); a.MUL('dy'); a.ADD('x1'); a.ST('x1')
    a.LD('cr16'); a.MUL('cr16'); a.SHR(4); a.ST('x0'); a.LD('x1'); a.SUB('x0'); a.ST('V4')
    a.LD('dy'); a.MUL('ys2'); a.ST('x0'); a.ADD('x0'); a.ADD('ysq'); a.ST('U4')
    a.LD('V4'); a.SUB('U4'); a.ADD('vv'); a.ST('V4')
    a.LD('U4'); a.SUB('vv'); a.ST('U4')
    # the pixel step offset: 2 (cCx - Ox) xs
    a.LD('cCx'); a.SUB('Ox'); a.MUL('xs'); a.ST('x0'); a.ADD('x0'); a.ST('dl')
    # coverage scale: T = cr16 xs; sh = floor(log2 T) - 6 with a 3/4 step (m)
    a.LD('cr16'); a.MUL('xs'); a.ST('tv'); a.LDI(2); a.ST('tsh')
    a.label('csl')
    a.LD('tv'); a.SUBI(512); a.SKIP('GE'); a.JMP('csd')
    a.LD('tv'); a.SHR(1); a.ST('tv'); a.LD('tsh'); a.ADDI(1); a.ST('tsh'); a.JMP('csl')
    a.label('csd')
    a.LDI(0); a.ST('tm'); a.LD('tv'); a.SUBI(296); a.SKIP('GE'); a.JMP('cs2')
    a.LD('tv'); a.SUBI(418); a.SKIP('GE'); a.JMP('cs1')
    a.LD('tsh'); a.ADDI(1); a.ST('tsh'); a.JMP('cs2')
    a.label('cs1')
    a.LDI(1); a.ST('tm')
    a.label('cs2')
    a.LD('tsh'); a.SUBI(4); a.SKIP('LT'); a.JMP('cs3'); a.LDI(0)
    a.label('cs3')
    a.ST('tsh'); a.SUBI(9); a.SKIP('GE'); a.JMP('cs4'); a.LDI(9); a.ST('tsh')
    a.label('cs4')
    # CTL = dl * 1024 + role * 128 + class * 32 + m * 16 + sh
    a.LD('crole'); a.MULI(4); a.ADD('ccls'); a.MULI(2); a.ADD('tm'); a.MULI(16); a.ADD('tsh'); a.ST('x0')
    a.LD('dl'); a.MULI(1024); a.ADD('x0'); a.ST('x0')
    a.LD('slot'); a.MULI(4); a.OUT('PA')
    a.LD('V4'); a.OUT('PW'); a.LD('U4'); a.OUT('PW'); a.LD('x0'); a.OUT('PW')
    a.LD('slot'); a.ADDI(1); a.ST('slot')
    a.RET()

    # ---- FIGURE: the chain of the current figure: Fn discs from phase Fal, step Fstp, h = Fhh, class
    #      thresholds Ft1 .. Ft3, role Frole; eccentricity a (global)
    a.label('FIGURE')
    a.mov('Fn', 'fk'); a.mov('Fal', 'alk')
    a.label('fl')
    a.LD('alk'); a.ADD('C4194304'); a.CALL('WRAP'); a.CALL('SINQ'); a.ST('cs')
    a.LD('alk'); a.CALL('SINQ'); a.ST('sn')
    a.mul('ae', 'cs', 15, 'x0'); a.LD('aq'); a.SHR(1); a.ADD('x0'); a.ADD('C32768'); a.ST('E')
    a.mul('opa', 'cs', 16, 'x0'); a.ADD('ae'); a.ST('nx')
    a.mul('oma', 'sn', 16, 'ny')
    div('nx', 'E', 17, 'bx')
    div('ny', 'E', 17, 'by')
    a.mul('Fhh', 'E', 13, 'x0'); a.ADD('oma'); a.ST('x1')
    div('oma', 'x1', 17, 'cr')
    a.LD('C65536'); a.SUB('cr'); a.ST('x2')
    a.mul('x2', 'bx', 16, 'cX'); a.mul('x2', 'by', 16, 'cY')
    # tone class by size
    a.LDI(0); a.ST('ccls')
    for i in range(3):
        sk = a.new()
        a.LD('cr'); a.SUB('Ft%d' % (i + 1)); a.SKIP('GE'); a.JMP(sk); a.LD('ccls'); a.ADDI(1); a.ST('ccls')
        a.label(sk)
    a.mov('Frole', 'crole')
    a.CALL('CIRCLE')
    a.LD('alk'); a.ADD('Fstp'); a.CALL('WRAP'); a.ST('alk')
    a.LD('fk'); a.SUBI(1); a.ST('fk'); a.SUBI(1); a.SKIP('LT'); a.JMP('fl')
    a.RET()

    # ---- INNER: the inner circle (unit frame) of the current figure from rho0 = Frho: ci, ri (Q16)
    a.label('INNER')
    a.mul('ae', 'Frho', 16, 'x3')
    a.LD('Frho'); a.ADD('ae'); a.ST('x0'); a.LD('C65536'); a.ADD('x3'); a.ST('x1')
    a.LD('x0'); a.ST('dn'); a.LD('x1'); a.ST('dd'); a.LDI(17); a.ST('dit'); a.CALL('DIVQ'); a.ST('m1')
    a.LD('ae'); a.SUB('Frho'); a.ST('dn'); a.LD('C65536'); a.SUB('x3'); a.ST('dd'); a.LDI(17); a.ST('dit')
    a.CALL('DIVQ'); a.ST('m2')
    a.LD('m1'); a.ADD('m2'); a.SHR(1); a.ST('cX'); a.LD('m1'); a.SUB('m2'); a.SHR(1); a.ST('cr')
    a.LDI(0); a.ST('cY'); a.ST('ccls')
    a.RET()

    # ---- HG: hold (a knob has to move 2 counts) + glide (Q6, 1/16 of the gap per field, never under half a
    #      count, landing exactly; Glide Off: the held value) for the knob in kin, hold hk, accumulator gk
    a.label('HG')
    a.LD('kin'); a.SUB('hk'); a.ADDI(1); a.SKIP('GE'); a.JMP('hgu'); a.SUBI(3); a.SKIP('GE'); a.JMP('hgk')
    a.label('hgu')
    a.mov('kin', 'hk')
    a.label('hgk')
    a.LD('hk'); a.MULI(64); a.ST('tg'); a.SKIP('GLIDE'); a.JMP('hgs')
    a.SUB('gk'); a.ST('gd'); a.abs_('x0'); a.SUBI(33); a.SKIP('GE'); a.JMP('hgs')
    a.LD('gd'); a.SHR(4); a.ST('gs')
    a.abs_('x0'); a.SUBI(32); a.SKIP('LT'); a.JMP('hga')
    a.LDI(32); a.ST('gs'); a.LD('gd'); a.SKIP('LT'); a.JMP('hga'); a.LDI(-32); a.ST('gs')
    a.label('hga')
    a.LD('gk'); a.ADD('gs'); a.ST('gk'); a.RET()
    a.label('hgs')
    a.mov('tg', 'gk'); a.RET()

    a.label('main')
    # ---- knob holds (a knob has to move 2 counts) and glides (Q6).  Glide On: 1/16 of the gap per field, never
    #      under half a count, landing exactly (the Drift speed: 1/8, landing exactly); Off: the held value.
    def hold(inp, h):
        u, k = a.new(), a.new()
        a.IN(inp); a.SUB(h); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
        a.label(u); a.IN(inp); a.ST(h)
        a.label(k)

    def glide(h, acc, sh, mn, snap):
        sn, ad, dn = a.new(), a.new(), a.new()
        a.LD(h); a.MULI(64); a.ST('tg'); a.SKIP('GLIDE'); a.JMP(sn)
        a.SUB(acc); a.ST('gd'); a.abs_('x0'); a.SUBI(snap + 1); a.SKIP('GE'); a.JMP(sn)
        a.LD('gd'); a.SHR(sh); a.ST('gs')
        if mn:
            a.abs_('x0'); a.SUBI(mn); a.SKIP('LT'); a.JMP(ad)
            a.LDI(mn); a.ST('gs'); a.LD('gd'); a.SKIP('LT'); a.JMP(ad); a.LDI(-mn); a.ST('gs')
        a.label(ad)
        a.LD(acc); a.ADD('gs'); a.ST(acc); a.JMP(dn)
        a.label(sn)
        a.LD('tg'); a.ST(acc)
        a.label(dn)

    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3', 'a3'), ('K6', 'h6', 'a6'),
                        ('P12', 'hp', 'ap')):
        a.IN(inp); a.ST('kin'); a.mov(h, 'hk'); a.mov(acc, 'gk'); a.CALL('HG'); a.mov('hk', h); a.mov('gk', acc)
    hold('K5', 'h5'); glide('h5', 'a5', 3, 0, 8)

    # ---- mode values
    a.IN('XS'); a.ST('xs'); a.IN('YS2'); a.ST('ys2'); a.MUL('ys2'); a.ST('ysq'); a.ADD('ysq'); a.ST('vv')

    # ---- counts: n1 = K4 step + 3, n2 = min(n1, NCH - n1); per-count constants for both
    # Chain: n1 = step + 3.  Nested (n1 + n2 + 1 <= NCH): n1 = 3 + (i + 1) / 2,
    # n2 = max(3, min(3 + i / 2, 11 - n1 - (i & 1))): (3,3) (4,3) (4,4) (5,4) (5,5) (6,4) (6,5) (7,3) (7,4) (8,3)
    a.IN('NZ'); a.ST('nz'); a.ADDI(3); a.ST('n1'); a.SKIP('NEST'); a.JMP('ncd')
    a.LD('nz'); a.ADDI(1); a.SHR(1); a.ADDI(3); a.ST('n1')
    a.LD('nz'); a.SHR(1); a.ST('x0'); a.MULI(2); a.ST('x1'); a.LD('nz'); a.SUB('x1'); a.ST('x1')      # i & 1
    a.LDI(NCH - 1); a.SUB('n1'); a.SUB('x1'); a.ST('n2'); a.LD('x0'); a.ADDI(3); a.SUB('n2')
    sk = a.new()
    a.SKIP('LT'); a.JMP(sk); a.LD('x0'); a.ADDI(3); a.ST('n2')
    a.label(sk)
    a.LD('n2'); a.SUBI(3); a.SKIP('LT'); a.JMP('ncd'); a.LDI(3); a.ST('n2')
    a.label('ncd')
    a.mov('n2', 'nn'); a.CALL('NCONST')
    for nm in ('stp', 'hh', 'rho0', 't1', 't2', 't3'):
        a.mov(nm, nm + '_2')
    a.mov('n1', 'nn'); a.CALL('NCONST')

    # ---- eccentricity a = 0.85 P12 (Q16), a^2, 1 + a^2, 1 - a^2
    a.LD('ap'); a.SHR(3); a.MULI(871); a.SHR(7); a.ST('ae')
    a.MUL('ae'); a.SHR(16); a.ST('aq'); a.ADD('C65536'); a.ST('opa'); a.LD('C65536'); a.SUB('aq'); a.ST('oma')

    # ---- Drift: |v| < 24 still; else 2^(-12 + ls / 64) turn per field, ls = (|v| - 24) * 13/16 (top 2^-5.8)
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(3); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(4); a.ST('x0')
    a.LD('ls'); a.SUB('x0'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('w'); a.SHR(3); a.JMP('spd')
    a.label('still')
    a.LDI(0)
    a.label('spd')
    a.ST('w'); a.SHR(2); a.ST('wq')
    a.LD('v'); a.SKIP('LT'); a.JMP('vp')
    a.LD('al0'); a.SUB('w'); a.CALL('WRAP'); a.ST('al0'); a.LD('bt'); a.ADD('wq'); a.JMP('vd')
    a.label('vp')
    a.LD('al0'); a.ADD('w'); a.CALL('WRAP'); a.ST('al0'); a.LD('bt'); a.SUB('wq')
    a.label('vd')
    a.CALL('WRAP'); a.ST('bt')
    a.ADD('C4194304'); a.CALL('WRAP'); a.CALL('SINQ'); a.ST('cb')
    a.LD('bt'); a.CALL('SINQ'); a.ST('sb')

    # ---- scale: Ls = 256 log2(R in units) = 2807 + (K3 - 512) * 256 / 450; R in Q4 units = 16 * 2^(Ls / 256)
    a.LD('a3'); a.SHR(3); a.SUBI(2047); a.SUBI(2049 - 2000); a.SUBI(2000); a.MULI(291); a.SHR(12); a.ADDI(2047); a.ADDI(760)
    a.ST('Ls')
    a.SHR(8); a.ST('x1'); a.MULI(256); a.ST('x0'); a.LD('Ls'); a.SUB('x0'); a.OUT('ADR'); a.IN('ROM'); a.ST('x2')
    a.LD('x1'); a.SUBI(11); a.SKIP('LT'); a.JMP('rup')
    a.ADDI(1); a.SKIP('LT'); a.JMP('r10')
    a.LD('x2'); a.SHR(2); a.JMP('rst')
    a.label('r10')
    a.LD('x2'); a.SHR(1); a.JMP('rst')
    a.label('rup')
    a.SUBI(1); a.SKIP('GE'); a.JMP('r11')
    a.LD('x2'); a.ADD('x2'); a.JMP('rst')
    a.label('r11')
    a.LD('x2')
    a.label('rst')
    a.ST('R16')
    # ---- the great circle's centre: Ox = (K1 - 512) * 7.5 units, Oy2 = (512 - K2) * 16.875 Q2 units
    a.LD('a1'); a.SHR(3); a.SUBI(2047); a.SUBI(2049 - 2000); a.SUBI(2000); a.MULI(15); a.SHR(4); a.ST('Ox')
    a.LD('a2'); a.SHR(3); a.SUBI(2047); a.SUBI(2049 - 2000); a.SUBI(2000); a.MULI(135); a.SHR(6); a.ST('x0'); a.LDI(0); a.SUB('x0')
    a.ST('Oy2')

    # ---- the camera: C = s << SC, s = the stream of T(l) = min(GT l^2 >> 16, 8191); C_max = 0.08 R^2 K6 at white
    a.LD('Ls'); a.ADD('Ls'); a.SUB(a.const(CAM_LOG)); a.ST('x0'); a.ADDI(255); a.SHR(8); a.ST('csc')
    a.MULI(256); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.ST('cfr')
    a.LD('a6'); a.SHR(3); a.MULI(1033); a.SHR(10); a.ST('g6')
    a.LD('cfr'); a.SKIP('LT'); a.JMP('gfz')
    a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('x0'); a.LD('g6'); a.MUL('x0'); a.SHR(16); a.JMP('gfd')
    a.label('gfz')
    a.LD('g6')
    a.label('gfd')
    a.ST('gt')
    # globals: dq0 = 2 (xpre - Ox) xs + xs^2, GT, SC
    a.LDI(96); a.OUT('PA')
    a.IN('XPRE'); a.SUB('Ox'); a.MUL('xs'); a.ST('x0'); a.ADD('x0'); a.ST('x0'); a.LD('xs'); a.MUL('xs')
    a.ADD('x0'); a.OUT('PW')
    a.LD('gt'); a.OUT('PW'); a.LD('csc'); a.OUT('PW')

    # ---- slot 0: the great circle
    a.mov('Ox', 'FOx'); a.mov('Oy2', 'FOy2'); a.mov('R16', 'FR'); a.mov('cb', 'Fc'); a.mov('sb', 'Fs')
    a.LDI(0); a.ST('slot'); a.ST('cX'); a.ST('cY'); a.ST('ccls'); a.mov('C65536', 'cr'); a.LDI(ROLE_OUTER)
    a.ST('crole'); a.CALL('CIRCLE')
    # ---- slot 1: the inner circle
    a.mov('rho0', 'Frho'); a.CALL('INNER'); a.LDI(ROLE_INNER); a.ST('crole'); a.CALL('CIRCLE')
    a.mov('cCx', 'IOx'); a.mov('cCy2', 'IOy2'); a.mov('cr16', 'IR')
    # ---- Nested: slot 2 = the nested inner circle (the same construction inside the inner circle, its axis
    #      turned the other way)
    a.SKIP('NEST'); a.JMP('nn1')
    a.mov('Fc', 'sFc'); a.mov('Fs', 'sFs')
    a.mov('IOx', 'FOx'); a.mov('IOy2', 'FOy2'); a.mov('IR', 'FR'); a.LDI(0); a.SUB('sb'); a.ST('Fs')
    a.mov('rho0_2', 'Frho'); a.CALL('INNER'); a.LDI(ROLE_NINNER); a.ST('crole'); a.CALL('CIRCLE')
    a.mov('Ox', 'FOx'); a.mov('Oy2', 'FOy2'); a.mov('R16', 'FR'); a.mov('sFs', 'Fs')
    a.label('nn1')
    # ---- the chain (slots 2 .. or 3 ..)
    a.mov('n1', 'Fn'); a.mov('al0', 'Fal'); a.mov('stp', 'Fstp'); a.mov('hh', 'Fhh')
    for i in (1, 2, 3):
        a.mov('t%d' % i, 'Ft%d' % i)
    a.LDI(ROLE_CH0); a.ST('Frole'); a.CALL('FIGURE')
    # ---- Nested: the second chain inside the inner circle, turning the other way
    a.SKIP('NEST'); a.JMP('nonest')
    a.mov('IOx', 'FOx'); a.mov('IOy2', 'FOy2'); a.mov('IR', 'FR'); a.LDI(0); a.SUB('sb'); a.ST('Fs')
    a.mov('n2', 'Fn'); a.LDI(0); a.SUB('al0'); a.CALL('WRAP'); a.ST('Fal'); a.mov('stp_2', 'Fstp')
    a.mov('hh_2', 'Fhh')
    for i in (1, 2, 3):
        a.mov('t%d_2' % i, 'Ft%d' % i)
    a.LDI(ROLE_CH1); a.ST('Frole'); a.CALL('FIGURE')
    a.label('nonest')
    # ---- the rest of the slots: off
    a.label('offl')
    a.LD('slot'); a.SUBI(NSLOT); a.SKIP('LT'); a.JMP('done')
    a.LD('slot'); a.MULI(4); a.ADDI(2); a.OUT('PA'); a.LDI(0); a.OUT('PW')
    a.LD('slot'); a.ADDI(1); a.ST('slot'); a.JMP('offl')
    a.label('done')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; steiner.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=400000, check=False, trace=None):
    """inp: dict over INS names; flags: GLIDE, NEST.  rf: the register file (list of 256 ints, updated in place;
    None = the power-up file).  Returns (param RAM dict addr -> 32-bit word, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = [0, 0]; adr = 0; pa = 0
    pram = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        if op in (0, 1, 2, 3):
            b = _sx(rf[arg & 255], 32)
        elif op in (4, 5, 6, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, 37)
        elif op in (2, 6):
            acc = _sx(acc - b, 37)
        elif op in (3, 7):
            if check:
                assert -(1 << 17) <= acc < (1 << 17) and -(1 << 17) <= b < (1 << 17), (pc, acc, b)
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 17
        elif op == 8:
            rf[arg & 255] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = TAB[adr]
            elif n == INS['ZERO']: acc = 0
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 3
            if n == OUTS['ADR']: adr = acc & 511
            elif n == OUTS['PA']: pa = acc & 127
            elif n == OUTS['PW']:
                pram[pa] = acc & 0xFFFFFFFF
                if trace is not None and (pa & 3) == 2 and pa < 96:
                    R = _ASM.regs
                    trace.append((pa >> 2, _sx(rf[R['cCx']], 32), _sx(rf[R['cCy2']], 32), _sx(rf[R['cr16']], 32)))
                pa = (pa + 1) & 127
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'NEST'}.get(c, 'GLIDE')])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 1023
        elif op == 14:
            lr = [pc + 1, lr[0]]; npc = arg & 1023
        else:
            if arg == 0:
                npc = lr[0]; lr = [lr[1], lr[1]]
            elif arg == 1:
                break
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    return pram, rf, clocks


def decode_slot(pram, j):
    ctl = _sx(pram.get(4 * j + 2, 0), 32)
    return dict(V0=_sx(pram.get(4 * j, 0), 32), U0=_sx(pram.get(4 * j + 1, 0), 32), dl=ctl >> 10,
                sh=ctl & 15, m=(ctl >> 4) & 1, cls=(ctl >> 5) & 3, role=(ctl >> 7) & 7)


# ---------------------------------------------------------------------------------------------------------
# tone ROM: address = mode (2) & level (1) & class (2) & coverage sum (4: 0 .. 8 used)
# ---------------------------------------------------------------------------------------------------------
TB, TG, TM, TN = 64, 239, 467, 239
TCH = (589, 694, 816, 939)
TWASH = (64, 152, 256, 152)        # Grey Fine washes: outside, gap, inner disc, nested inner disc


def y_rom():
    """tone ROM (12-bit signed): 0 .. 127 Grey Bold chain term S x (T_class - T_ground(level)) / 8 (level, class, S);
    128 .. 143 Ink Bold luma by L; 256 .. 319 Grey Fine luma by (region, L); 384 .. 399 Ink Fine luma by L."""
    out = [0] * 512
    for lvl in (0, 1):
        g = TG if lvl == 0 else TM
        for c in range(4):
            for sv in range(16):
                out[lvl * 64 + c * 16 + sv] = (min(sv, 8) * (TCH[c] - g)) // 8
    TW = TCH[3]
    for L in range(16):
        out[128 + L] = TB + (min(L, 8) * (TW - TB)) // 8
        out[384 + L] = TB + (min(L, 8) * (TW - TB)) // 8
        for rg in range(4):
            w = TWASH[rg]
            out[256 + rg * 16 + L] = w + (min(L, 8) * (TW - w)) // 8
    return out


def abc_tabs():
    a = [TB + (min(c, 8) * (TG - TB)) // 8 for c in range(16)]
    b = [(min(c, 8) * (TM - TG)) // 8 for c in range(16)]
    cc = [(min(c, 8) * (TN - TM)) // 8 for c in range(16)]
    return a, b, cc


def vhd_table(name, depth, width, vals, typ="unsigned"):
    digits = (width + 3) // 4
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    mask = (1 << width) - 1
    if width % 4 == 0:
        items = ['x"%0*X"' % (digits, v & mask) for v in vals]
    else:
        items = ['"' + format(v & mask, "0%db" % width) + '"' for v in vals]
    per = 16 if width % 4 == 0 else 8
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by steiner.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package steiner_pkg is",
             vhd_table("C_ST_TAB", 512, 16, TAB),
             vhd_table("C_ST_UC", 1024, 16, uc, "std_logic_vector"),
             vhd_table("C_ST_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             vhd_table("C_ST_Y", 512, 12, y_rom(), "signed"),
             vhd_table("C_ST_A", 16, 12, abc_tabs()[0], "signed"),
             vhd_table("C_ST_B", 16, 12, abc_tabs()[1], "signed"),
             vhd_table("C_ST_C", 16, 12, abc_tabs()[2], "signed"),
             "end package steiner_pkg;", ""]
    with open(os.path.join(HERE, "steiner_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
