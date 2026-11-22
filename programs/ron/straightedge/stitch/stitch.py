#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
STITCH v0.1.0 (Straightedge) - build hook.  Writes stitch_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  The integer model (.straightedge_work/rb_stitch/imodel.py) imports this file and runs
the same microprogram through emulate(), so the per-field arithmetic has one source.

Curve stitching: in each quadrant of a fold (x' = |x - X|, y' = |y - Y|) N straight lines tangent to one conic, at
equal steps of their normal angle phi over a half turn; both ends of the half turn are lines at infinity (Corners)
or on the fold axes (Cross), so the drift (every angle advanced by f steps) wraps exactly at one step.
  Corners  u = H - x', v = H - y' (the corner of an implied square), parabola sqrt(u) + sqrt(v) = sqrt(L):
           d = c u + s v - rho, rho = L s c / (s + c), phi in (-pi/4, 3 pi/4)
  Cross    u = x' + Hc, v = y' + Hc, the hyperbolas u v = +-k (segments and rays):
           rho = sqrt(2k |sin 2 phi|), phi in (-pi/2, pi/2)
Every line is one affine DDA: d = pc * (pixel steps) + lc * (line steps) + seed, folded by reversing the steps.

Package contents (block-RAM ROMs):
  C_ST_TAB  1024 x 16  the engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)); 512 .. 767 reciprocals round(2^30 / (32768 + 128 j))
                       (entry 512 = 32768 = sin(pi/2) closes the sine); 768 .. 1023 round(32768 * sqrt(sin(pi/2 j/256)))
                       (address 1024 wraps to entry 0 = 32768 = sqrt(sin(pi/2)))
  C_ST_UC   1024 x 16  the per-field microprogram (PROGRAM below)
  C_ST_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults, constants)

The per-field engine is Hardline flute's (as in Straightedge windrose): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial 18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift; micro-op = 4-bit
opcode + 12-bit operand.  It runs once per field and writes one bank of the table:
  word l * 8 + 0 .. 4   line l (0 .. 23): the pixel step, the line wrap for a forward line step, the field seed,
                        the wrap for a backward line step, the wrap for a held line; Q12 of the anti-aliasing unit
  192                   x fold: clocks to the flip | hold << 12
  193                   y fold: lines to the flip | hold << 12
  194                   Fine line: half width + 1/2, 1/8 unit
  256 .. 383            the tone table: 4 camera levels x 32 coverage sums, 10-bit luma
The picture of a field uses the bank written during the field before.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    r = [int(round((1 << 30) / (32768 + 128 * j))) for j in range(256)]
    q = [min(int(round(32768 * math.sqrt(math.sin(math.pi / 2 * j / 256.0)))), 32767) for j in range(256)]
    return e + s + r + q


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k5=680, k6=1023, p12=512)
NL = 24                                           # line DDAs
CONST_ADR = 192
TONE_ADR = 256
XOFS = 1                                          # screen column j = the output filter centred on DDA index j + XOFS
NEXTRA = 6                                        # DDA steps per line = width + NEXTRA
PINS = (8, 12, 16, 24)
STEPQ = (65536, 43690, 32768, 21845)              # angle step >> 6 (2^24 = a quarter turn): pi / N, rounded down
Q24 = 1 << 24
RHOMAX4 = 32768                                   # |rho| clamp: 2048 units (Q4)
LEVELS = (64, 356, 648, 940)                      # black, dark, light, white (luma before the 13/16 conditioning)
LIFT = ((0, 1, 2, 3), (1, 2, 3, 3), (2, 3, 2, 3), (3, 2, 3, 2))
GND = (0, 292, 584, 584)                          # Fine ground lift per camera level (x K6)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XS=9, IL=10, ROM=11, W=12, NZ=13, HN=14, ZERO=15)
CONDS = dict(LT=0, GLIDE=1, CROSS=2, INK=3, FINE=4, GE=8, NGLIDE=9, NCROSS=10, NINK=11, NFINE=12)
OUTS = dict(ADR=0, TA=1, TD=2)


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

    def neg(self, tmp):
        self.ST(tmp); self.LDI(0); self.SUB(tmp)

    def const(self, v):
        """a register holding the constant v."""
        n = 'C%d' % v if v >= 0 else 'CM%d' % (-v)
        self.reg(n, v)
        return n


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up = the TOML defaults)
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h5', D['k5']), ('h6', D['k6']),
                 ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a5', D['k5']), ('a6', D['k6']),
                 ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('th', 0)                                 # the drift fraction of one angle step (2^16 = one step)
    a.reg('wrapc', 0)                              # Cross: drift wraps mod 4 (the tone table follows the count)
    C = a.const

    a.JMP('main')

    # ---- SHRU: ACC >>= ush (0 .. 2)
    a.label('SHRU')
    a.ST('u0'); a.LD('ush'); a.SUBI(1); a.SKIP('GE'); a.JMP('shu0'); a.SUBI(1); a.SKIP('GE'); a.JMP('shu1')
    a.LD('u0'); a.SHR(2); a.RET()
    a.label('shu1'); a.LD('u0'); a.SHR(1); a.RET()
    a.label('shu0'); a.LD('u0'); a.RET()

    # ---- EXPQ4: ACC = 16 * 2^(L / 256) = EXP[L & 255] >> (11 - (L >> 8)), L >> 8 in 6 .. 12
    a.label('EXPQ4')
    a.ST('x0'); a.SHR(8); a.ST('x1'); a.MULI(256); a.ST('x2'); a.LD('x0'); a.SUB('x2'); a.OUT('ADR'); a.IN('ROM')
    a.ST('x2'); a.LDI(11); a.SUB('x1'); a.ST('x3'); a.SKIP('GE'); a.JMP('exneg')
    a.label('exl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('exd'); a.ST('x3'); a.LD('x2'); a.SHR(1); a.ST('x2'); a.JMP('exl')
    a.label('exd')
    a.LD('x2'); a.RET()
    a.label('exneg')
    a.LD('x2'); a.ADD('x2'); a.RET()

    # ---- SINS: ACC = 32768 sin(pi/2 * phi / 2^24), phi in ACC (-2^24 .. 2^24): 256-step table, 16-bit
    #      interpolation, odd
    a.label('SINS')
    a.ST('sp'); a.ST('sg'); a.SKIP('GE'); a.JMP('sinn')
    a.label('sinp')
    a.LD('sp'); a.SHR(16); a.ST('si'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('si'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('sd')
    a.LD('si'); a.MULI(256); a.MULI(256); a.ST('x0'); a.LD('sp'); a.SUB('x0')
    a.MUL('sd'); a.SHR(16); a.ADD('s0'); a.ST('x0')
    a.LD('sg'); a.SKIP('LT'); a.JMP('sinr'); a.LDI(0); a.SUB('x0'); a.RET()
    a.label('sinr')
    a.LD('x0'); a.RET()
    a.label('sinn')
    a.LDI(0); a.SUB('sp'); a.ST('sp'); a.JMP('sinp')

    # ---- SQS: ACC = 32768 sqrt(sin(pi/2 * th / 2^24)), th in ACC (0 .. 2^24)
    a.label('SQS')
    a.ST('sp'); a.SHR(16); a.ST('si'); a.ADDI(768); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('si'); a.ADDI(769); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('sd')
    a.LD('si'); a.MULI(256); a.MULI(256); a.ST('x0'); a.LD('sp'); a.SUB('x0')
    a.MUL('sd'); a.SHR(16); a.ADD('s0'); a.RET()

    # ---- RHOC: Corners rho = L s c / (s + c), Q4 units, clamped to +-RHOMAX4.  In: s, c, xx = s + c, L4u.
    #      1 / xx: normalise xx to [2^15, 2^16) by doubling (ee times), a 256-entry reciprocal, one Newton step.
    a.label('RHOC')
    a.LD('s'); a.MUL('c'); a.SHR(15); a.ST('sc')
    a.LD('xx'); a.SUBI(1); a.SKIP('GE'); a.JMP('rcl')
    a.LDI(0); a.ST('ee')
    a.label('rnm')
    a.LD('xx'); a.SUB(C(32768)); a.SKIP('LT'); a.JMP('rnd')
    a.LD('xx'); a.ADD('xx'); a.ST('xx'); a.LD('ee'); a.ADDI(1); a.ST('ee'); a.JMP('rnm')
    a.label('rnd')
    a.SHR(7); a.ADDI(512); a.OUT('ADR'); a.IN('ROM'); a.ST('r0')
    a.LD('xx'); a.MUL('r0'); a.ST('t0'); a.LD(C(1 << 30)); a.SUB('t0'); a.ADD(C(1 << 30)); a.SHR(14)
    a.MUL('r0'); a.SHR(16); a.ST('r1')
    a.LD('sc'); a.MUL('r1'); a.SHR(15); a.MUL('L4u'); a.ST('rp')
    a.label('rsh')
    a.LD('ee'); a.SUBI(1); a.SKIP('GE'); a.JMP('rdn'); a.ST('ee')
    a.LD('rp'); a.ADD('rp'); a.ST('rp'); a.abs_('x0'); a.SUB(C(1 << 30)); a.SKIP('LT'); a.JMP('rcl'); a.JMP('rsh')
    a.label('rdn')
    a.LD('rp'); a.SHR(15); a.RET()
    a.label('rcl')                                 # near the ends s c < 0: the line is at infinity, rho -> -inf
    a.LD('sc'); a.SKIP('LT'); a.JMP('rcp'); a.LD(C(-RHOMAX4)); a.RET()
    a.label('rcp')
    a.LD(C(RHOMAX4)); a.RET()

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

    a.JMP('hgx')
    a.label('HG')                                  # kin = the knob, hh = its hold, aa = its glide
    a.LD('kin'); a.SUB('hh'); a.ADDI(1); a.SKIP('GE'); a.JMP('hgu'); a.SUBI(3); a.SKIP('GE'); a.JMP('hgk')
    a.label('hgu'); a.LD('kin'); a.ST('hh')
    a.label('hgk')
    glide('hh', 'aa', 4, 32, 32)
    a.RET()
    a.label('hgx')
    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3', 'a3'), ('K6', 'h6', 'a6'),
                        ('P12', 'hp', 'ap')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(acc); a.ST('aa'); a.CALL('HG')
        a.LD('hh'); a.ST(h); a.LD('aa'); a.ST(acc)
    hold('K5', 'h5'); glide('h5', 'a5', 3, 0, 8)

    # ---- mode values.  The anti-aliasing unit is 2^ush virtual px: 1 (1080p), 2 (1080i, 720p, 480p, 576p),
    #      4 (480i, 576i).  pw / lh = a pixel's width / a line's height in units (Q14); ivx = pixels per virtual px
    #      (Q14); ivy = lines per virtual px (Q16).
    a.IN('YS'); a.ST('ys'); a.IN('XS'); a.ST('xs'); a.IN('IL'); a.ST('ush')
    a.LD('xs'); a.SUBI(3); a.SKIP('GE'); a.JMP('m2'); a.SUBI(1); a.SKIP('GE'); a.JMP('m3')
    a.LD(C(8192)); a.ST('ivx'); a.LD(C(32768)); a.ST('pw')
    a.LD('ush'); a.ADDI(1); a.ST('ush'); a.JMP('mx')
    a.label('m3')
    a.LD(C(10923)); a.ST('ivx'); a.LD(C(24576)); a.ST('pw'); a.LD('ush'); a.ADDI(1); a.ST('ush'); a.JMP('mx')
    a.label('m2')
    a.LD(C(16384)); a.ST('ivx'); a.ST('pw')
    a.label('mx')
    a.LD('pw'); a.CALL('SHRU'); a.ST('pw')
    a.LD('ys'); a.MULI(64); a.CALL('SHRU'); a.ST('lh')
    for thr, ys in ((300, 256), (100, 384), (100, 480), (50, 512), (200, 576), (300, 960), (None, 1138)):
        nxt = a.new()
        if thr is not None:
            if ys == 256:
                a.LD('ys')
            a.SUBI(thr); a.SKIP('LT'); a.JMP(nxt)
        a.LD(C(int(round((1 << 24) / ys)))); a.ST('ivy')
        a.JMP('my')
        a.label(nxt)
    a.label('my')

    # ---- Drift: |v| < 24 still; else 2^(-12 + ls / 64) step per field, ls = (|v| - 24) * 1.047 (top 2^-4).
    #      th (one step = 2^16) wraps exactly; wrapc counts the wraps mod 4.
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(5); a.ST('x0'); a.LD('u'); a.SHR(6); a.ADD('x0'); a.ADD('u'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('w'); a.SHR(11); a.ST('w')
    a.LD('v'); a.SKIP('LT'); a.JMP('dpos')
    a.LD('th'); a.SUB('w'); a.ST('th'); a.SKIP('LT'); a.JMP('still')
    a.ADD(C(65536)); a.ST('th'); a.LD('wrapc'); a.SUBI(1); a.SKIP('LT'); a.JMP('dwc'); a.ADDI(4); a.JMP('dwc')
    a.label('dpos')
    a.LD('th'); a.ADD('w'); a.ST('th'); a.SUB(C(65536)); a.SKIP('GE'); a.JMP('still')
    a.ST('th'); a.LD('wrapc'); a.SUBI(3); a.SKIP('LT'); a.JMP('dwc'); a.ADDI(4)
    a.label('dwc')
    a.ST('wrapc')
    a.label('still')

    # ---- Scale (H = 330 virtual px x 2^(-1.5 .. +0.7)), the span (Corners L = H x 2.75^(1 - 2 p) x 2) or the Cross
    #      hyperbola (sqrt(k) = 0.6 H, Hc = sqrt(k) x (0.5 .. 0 .. -1.25)), in Q4 units
    a.LD('a3'); a.SUB(C(32768)); a.SKIP('LT'); a.JMP('scp')
    a.MULI(3); a.SHR(8); a.JMP('scd')                           # below the centre: 1.5 octaves
    a.label('scp')
    a.MULI(179); a.SHR(15)                                       # above: 0.7 octave (H 117 .. 330 .. 536)
    a.label('scd')
    a.ADD(C(2142)); a.ST('lgh'); a.CALL('EXPQ4'); a.ST('H4')
    a.CALL('SHRU'); a.ST('H4u')
    a.LD('ap'); a.MULI(187); a.SHR(14); a.ST('x0'); a.LD('lgh'); a.ADDI(630); a.SUB('x0'); a.CALL('EXPQ4')
    a.CALL('SHRU'); a.ST('L4u')
    a.LD('H4'); a.MUL(C(27804)); a.SHR(15); a.CALL('SHRU'); a.ST('R2K4u')
    a.LD('H4'); a.MUL(C(19661)); a.SHR(15); a.CALL('SHRU'); a.ST('SK4u')
    a.LD('ap'); a.SHR(6); a.ST('pp'); a.SUBI(512); a.SKIP('LT'); a.JMP('hcn')
    a.LDI(512); a.SUB('pp'); a.MULI(4); a.JMP('hcm')
    a.label('hcn')
    a.MULI(-10)
    a.label('hcm')
    a.MUL('SK4u'); a.SHR(12); a.ST('Hc4u')

    # ---- Pins: N, the angle step (a multiple of 64; N steps <= a half turn) and the first angle
    a.IN('NZ'); a.ST('nz')
    for i in range(4):
        nx = a.new()
        if i < 3:
            a.LD('nz'); a.SUBI(i + 1); a.SKIP('LT'); a.JMP(nx)
        a.LDI(PINS[i]); a.ST('Nn'); a.LD(C(STEPQ[i])); a.ST('stq'); a.JMP('pnd')
        a.label(nx)
    a.label('pnd')
    a.LD('stq'); a.MULI(64); a.ST('step')
    a.LD('th'); a.MUL('stq'); a.SHR(10); a.ST('ph')
    a.SKIP('CROSS'); a.JMP('ph0c')
    a.LD('ph'); a.SUB(C(Q24)); a.ST('ph'); a.JMP('ph0d')
    a.label('ph0c')
    a.LD('ph'); a.SUB(C(Q24 // 2)); a.ST('ph')
    a.label('ph0d')

    # ---- the fold: Xh = 2 x the fold's DDA index (W - 1 + 2 XOFS at the centre, +-640 virtual px), Yh = 2 x the
    #      fold's line (+-360 virtual px, up = up); n0 = clocks / lines to the flip, h = a held step (odd Xh / Yh)
    a.LD('a1'); a.SUB(C(32768)); a.SHR(1); a.MULI(5); a.MUL('ivx'); a.SHR(20); a.ST('x0')
    a.IN('W'); a.ADDI(2 * XOFS - 1); a.ADD('x0'); a.ST('Xh')
    a.LD('a2'); a.SUB(C(32768)); a.SHR(4); a.MULI(45); a.MUL('ivy'); a.SHR(23); a.ST('x0')
    a.IN('HN'); a.SUBI(1); a.SUB('x0'); a.ST('Yh')
    for ax in ('x', 'y'):
        src = 'Xh' if ax == 'x' else 'Yh'
        a.LD(src); a.SHR(1); a.ST('n0' + ax); a.ADD('n0' + ax); a.ST('x0'); a.LD(src); a.SUB('x0'); a.ST('h' + ax)
        a.LD('n0' + ax); a.ADD('h' + ax); a.ST('n0' + ax)
    a.IN('W'); a.ADDI(NEXTRA); a.ST('M')
    a.LD('n0x'); a.ADD('n0x'); a.SUB('M'); a.SUB('hx'); a.ST('DL')

    # ---- the lines
    a.LDI(0); a.ST('li'); a.ST('tb')
    a.label('loop')
    a.LD('tb'); a.OUT('TA'); a.ADDI(8); a.ST('tb')
    a.LD('li'); a.SUB('Nn'); a.SKIP('LT'); a.JMP('dis')
    # s = sin(ph) (ph in (-pi/2, 3pi/4)), c = cos(ph) = sin(pi/2 - |ph|)
    a.LD('ph'); a.SUB(C(Q24)); a.SKIP('GE'); a.JMP('ls1')
    a.LD(C(2 * Q24)); a.SUB('ph'); a.JMP('ls2')
    a.label('ls1')
    a.LD('ph')
    a.label('ls2')
    a.CALL('SINS'); a.ST('s')
    a.LD('ph'); a.abs_('x0'); a.ST('x0'); a.LD(C(Q24)); a.SUB('x0'); a.CALL('SINS'); a.ST('c')
    a.SKIP('NCROSS'); a.JMP('lcr')
    # Corners: sigma = -1, hof = H
    a.LD('s'); a.ADD('c'); a.ST('xx'); a.ST('cs'); a.CALL('RHOC'); a.ST('rho')
    a.LD('H4u'); a.ST('hof')
    a.LD('c'); a.MUL('pw'); a.SHR(17); a.ST('pc')
    a.LD('s'); a.MUL('lh'); a.SHR(17); a.ST('lc'); a.JMP('lcm')
    # Cross: sigma = +1, hof = Hc; rho = sqrt(2k) sqrt|sin 2 ph|
    a.label('lcr')
    a.LD('s'); a.ADD('c'); a.ST('cs')
    a.LD('ph'); a.abs_('x0'); a.ST('x0'); a.ADD('x0'); a.ST('x0'); a.SUB(C(Q24)); a.SKIP('GE'); a.JMP('lq1')
    a.LD(C(2 * Q24)); a.SUB('x0'); a.JMP('lq2')
    a.label('lq1')
    a.LD('x0')
    a.label('lq2')
    a.CALL('SQS'); a.MUL('R2K4u'); a.SHR(15); a.ST('rho')
    a.LD('Hc4u'); a.ST('hof')
    a.LD('c'); a.MUL('pw'); a.SHR(17); a.neg('x0'); a.ST('pc')
    a.LD('s'); a.MUL('lh'); a.SHR(17); a.neg('x0'); a.ST('lc')
    a.label('lcm')
    # C (Q4 units) = (c + s) hof - rho; the five words
    a.LD('cs'); a.MUL('hof'); a.SHR(15); a.SUB('rho'); a.ST('C4')
    a.LD('pc'); a.OUT('TD')
    a.MUL('DL'); a.ST('t0'); a.LD('lc'); a.SUB('t0'); a.ST('t1'); a.OUT('TD')
    a.LD('pc'); a.MUL('Xh'); a.ST('t0'); a.LD('lc'); a.MUL('Yh'); a.ADD('t0'); a.SHR(1); a.ST('t2')
    a.LD('C4'); a.MULI(256); a.SUB('t2'); a.OUT('TD')
    a.LD('t1'); a.SUB('lc'); a.SUB('lc'); a.OUT('TD')
    a.ADD('lc'); a.OUT('TD')
    a.JMP('lnx')
    # a line beyond N: never drawn (step 0, seed far on the outer side)
    a.label('dis')
    a.LDI(0); a.OUT('TD'); a.OUT('TD'); a.LD(C(-(1 << 24))); a.OUT('TD'); a.LDI(0); a.OUT('TD'); a.OUT('TD')
    a.label('lnx')
    a.LD('ph'); a.ADD('step'); a.ST('ph')
    a.LD('li'); a.ADDI(1); a.ST('li'); a.SUBI(NL); a.SKIP('GE'); a.JMP('loop')

    # ---- the constants block
    a.LDI(CONST_ADR); a.OUT('TA')
    a.LD('hx'); a.MULI(1024); a.MULI(4); a.ADD('n0x'); a.OUT('TD')
    a.LD('hy'); a.MULI(1024); a.MULI(4); a.ADD('n0y'); a.OUT('TD')
    a.LDI(12); a.CALL('SHRU'); a.ADDI(4); a.OUT('TD')

    # ---- the tone table: per camera level q, five anchors at coverage sums 0, 8, 16, 24, 32 (1/8 of a line
    #      each), linear between.  Grey Bold: the count mod 4 as black / dark / light / white, lifted toward light
    #      by q x K6 (Cross: rotated by the drift's wrap count).  Ink: the parity, flipped in the brightest qmax
    #      levels (qmax = K6 in quarters).  Grey Fine: lines 0.62 white, crossings white, over a ground lifted by
    #      q x K6.
    a.LD('a6'); a.SHR(14); a.ST('qmx'); a.SUBI(3); a.SKIP('GE'); a.JMP('qmo'); a.LDI(3); a.ST('qmx')
    a.label('qmo')
    a.LDI(0); a.SKIP('NCROSS'); a.LD('wrapc'); a.ST('wr')
    a.SKIP('NFINE'); a.LDI(0); a.SKIP('NFINE'); a.ST('wr')       # Fine: no rotation (threads ignore sides)
    a.LD('a6'); a.MUL(C(292)); a.SHR(16); a.ST('G1'); a.ADD('G1'); a.ST('G2'); a.ADD('G1'); a.ST('G3')
    a.LDI(0); a.SUB('G1'); a.ST('GN')
    a.LDI(TONE_ADR); a.OUT('TA')
    a.LDI(0); a.ST('qi')
    a.label('tq')
    a.SKIP('NINK'); a.JMP('tink')
    a.SKIP('NFINE'); a.JMP('tfin')
    # Grey Bold: r_k = L[k] + (L[LIFT[q][k]] - L[k]) x g, the differences multiples of 292 x g
    gl = [a.new() for _ in range(4)]
    a.LD('qi'); a.SUBI(1); a.SKIP('GE'); a.JMP(gl[0]); a.SUBI(1); a.SKIP('GE'); a.JMP(gl[1])
    a.SUBI(1); a.SKIP('GE'); a.JMP(gl[2]); a.JMP(gl[3])
    gname = {0: None, 1: 'G1', 2: 'G2', 3: 'G3', -1: 'GN'}
    for q in range(4):
        a.label(gl[q])
        for k in range(4):
            m = (LEVELS[LIFT[q][k]] - LEVELS[k]) // 292
            a.LD(C(LEVELS[k]))
            if m:
                a.ADD(gname[m])
            a.ST('r%d' % k)
        a.JMP('trot')
    # Ink: parity; flipped when q >= 4 - qmax (qmax = 0: never)
    a.label('tink')
    a.LD('qmx'); a.ADD('qi'); a.SUBI(4); a.SKIP('LT'); a.JMP('tif')
    a.LD(C(64)); a.ST('r0'); a.ST('r2'); a.LD(C(940)); a.ST('r1'); a.ST('r3'); a.JMP('trot')
    a.label('tif')
    a.LD(C(940)); a.ST('r0'); a.ST('r2'); a.LD(C(64)); a.ST('r1'); a.ST('r3'); a.JMP('trot')
    # Grey Fine: ground (64 + (0, 1, 2, 2) x 292 g), ground + 0.62 (white - ground), white, white, white
    a.label('tfin')
    fg = [a.new() for _ in range(3)]
    a.LDI(64); a.ST('x0'); a.LD('qi'); a.SUBI(1); a.SKIP('GE'); a.JMP(fg[2]); a.SUBI(1); a.SKIP('GE'); a.JMP(fg[0])
    a.LD('x0'); a.ADD('G2'); a.JMP(fg[1])
    a.label(fg[0])
    a.LD('x0'); a.ADD('G1')
    a.label(fg[1])
    a.ST('x0')
    a.label(fg[2])
    a.LD('x0'); a.ST('r0'); a.LD(C(940)); a.SUB('x0'); a.MULI(635); a.SHR(10); a.ADD('x0'); a.ST('r1')
    a.LD(C(940)); a.ST('r2'); a.ST('r3'); a.ST('r4'); a.JMP('tdo')
    # rotate r0 .. r3 by wr (the count's offset), close the period
    a.label('trot')
    a.LD('wr'); a.ST('x3')
    a.label('trl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('trd'); a.ST('x3')
    a.LD('r0'); a.ST('x0'); a.LD('r1'); a.ST('r0'); a.LD('r2'); a.ST('r1'); a.LD('r3'); a.ST('r2')
    a.LD('x0'); a.ST('r3'); a.JMP('trl')
    a.label('trd')
    a.LD('r0'); a.ST('r4')
    a.label('tdo')
    a.CALL('TONE8')
    a.LD('qi'); a.ADDI(1); a.ST('qi'); a.SUBI(4); a.SKIP('GE'); a.JMP('tq')
    a.HALT()

    # TONE8: 32 entries from the anchors r0 .. r4 (e = 8 r_n + 4, out e >> 3, e += r_n+1 - r_n), four segments
    a.label('TONE8')
    a.LDI(4); a.ST('tn')
    a.label('tsg')
    a.LD('r1'); a.SUB('r0'); a.ST('td'); a.LD('r0'); a.MULI(8); a.ADDI(4); a.ST('te'); a.LDI(8); a.ST('tk')
    a.label('tlp')
    a.LD('te'); a.SHR(3); a.OUT('TD'); a.LD('te'); a.ADD('td'); a.ST('te')
    a.LD('tk'); a.SUBI(1); a.ST('tk'); a.SUBI(1); a.SKIP('LT'); a.JMP('tlp')
    a.LD('r1'); a.ST('r0'); a.LD('r2'); a.ST('r1'); a.LD('r3'); a.ST('r2'); a.LD('r4'); a.ST('r3')
    a.LD('tn'); a.SUBI(1); a.ST('tn'); a.SUBI(1); a.SKIP('LT'); a.JMP('tsg')
    a.RET()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; stitch.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=400000, check=False):
    """inp: dict over INS names (K1.. raw knobs, YS, XS, IL, W, HN, NZ); flags: GLIDE, CROSS, INK, FINE.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (table: dict address -> signed 32-bit word, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ta = 0
    table = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    fl = {1: 'GLIDE', 2: 'CROSS', 3: 'INK', 4: 'FINE'}
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
            if check:
                assert -(1 << 31) <= acc < (1 << 31), (pc, acc)
            rf[arg & 255] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = TAB[adr]
            elif n == INS['ZERO']: acc = 0
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 15
            if n == OUTS['ADR']: adr = acc & 1023
            elif n == OUTS['TA']: ta = acc & 511
            elif n == OUTS['TD']:
                table[ta] = _sx(acc, 32); ta = (ta + 1) & 511
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[fl[c]])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 1023
        elif op == 14:
            lr = pc + 1; npc = arg & 1023
        else:
            if arg == 0: npc = lr
            elif arg == 1: break
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    return table, rf, clocks


def vhd_table(name, depth, width, vals, typ="unsigned"):
    digits = (width + 3) // 4
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    mask = (1 << width) - 1
    items = ['x"%0*X"' % (digits, v & mask) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by stitch.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package stitch_pkg is",
             vhd_table("C_ST_TAB", 1024, 16, TAB),
             vhd_table("C_ST_UC", 1024, 16, uc, "std_logic_vector"),
             vhd_table("C_ST_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             "end package stitch_pkg;", ""]
    with open(os.path.join(HERE, "stitch_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
