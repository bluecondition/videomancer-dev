#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
WINDROSE v0.1.0 (Straightedge) - build hook.  Writes windrose_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.straightedge_work/rb_windrose/imodel.py) imports this file
and runs the same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_WR_TAB  512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)) (address 512 wraps to entry 0 = 32768 = sin(pi/2))
  C_WR_UC   1024 x 16  the per-field microprogram (PROGRAM below)
  C_WR_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                       constants)

The per-field engine is Hardline flute's (as in Vellum eclipse): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial 18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift; micro-op =
4-bit opcode + 12-bit operand.  It runs once per field and writes one bank of the DDA table:
  word d * 8 + 0 .. 4   DDA d (star d / 4: 0 = the great star; A, B, C, D): the pixel step, the line wrap for a
                        forward line step, the field seed, the wrap for a backward line step, the wrap for a held
                        line (the last three: Field form's mirror fold), all Q12 of the anti-aliasing unit
  160 .. 163            K of star 0 .. 3 (1/8 unit; -16384 = not drawn)
  164                   Fine line weight (1/8 unit) | half of it << 8
  165                   x fold: clocks to the first fold | cell width << 12 | odd << 24
  166                   y fold: lines to the first fold | cell height << 12 | odd << 24
  167                   the camera gain
The picture of a field uses the bank written during the field before.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    return e + s


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k5=660, k6=1023, p12=512)
NSTAR = 4
RHO = [2048, 1638, 1270, 983]                     # star radii / R1, Q11: 1, .80, .62, .48
NS_BY_NZ = [1, 2, 3, 4]                           # stars drawn for Points 4, 8, 16, 24
Q24 = 1 << 24                                     # a quarter turn
BASE = [0, Q24 // 2, Q24 // 4, Q24 // 8]          # 0, 45, 22.5, 11.25 degrees
CONST_ADR = 160                                   # K0 .. K3, line weight, x fold, y fold, gain
XOFS = 5                                          # screen column j = the output filter centred on DDA index j + XOFS
NEXTRA = 6                                        # DDA steps per line = width + NEXTRA
FR_Q10 = 5079                                     # Field: R1 = 0.62 cell (x 8 x 1024)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XS=9, IL=10, ROM=11, W=12, NZ=13, HN=14, ZERO=15)
CONDS = dict(LT=0, GLIDE=1, FIELD=2, GE=8, NGLIDE=9, NFIELD=10)
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
        self.ST(tmp); self.SKIP('LT'); self.JMP(d); self.LDI(0); self.SUB(tmp)
        self.label(d)

    def dbl(self, tmp, n):
        """ACC <<= n by doubling (no 18-bit limit)."""
        for _ in range(n):
            self.ST(tmp); self.ADD(tmp)

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
    a.reg('th', 0)                                 # the drift phase, 2^25 = half a turn (every tier's period)
    C = a.const

    a.JMP('main')

    # ---- SHRU: ACC >>= ush (0 .. 2)
    a.label('SHRU')
    a.ST('u0'); a.LD('ush'); a.SUBI(1); a.SKIP('GE'); a.JMP('shu0'); a.SUBI(1); a.SKIP('GE'); a.JMP('shu1')
    a.LD('u0'); a.SHR(2); a.RET()
    a.label('shu1'); a.LD('u0'); a.SHR(1); a.RET()
    a.label('shu0'); a.LD('u0'); a.RET()

    # ---- EXPQ4: ACC = 16 * 2^(L / 256) = EXP[L & 255] >> (11 - (L >> 8)), L >> 8 in 6 .. 10
    a.label('EXPQ4')
    a.ST('x0'); a.SHR(8); a.ST('x1'); a.MULI(256); a.ST('x2'); a.LD('x0'); a.SUB('x2'); a.OUT('ADR'); a.IN('ROM')
    a.ST('x2'); a.LDI(11); a.SUB('x1'); a.ST('x3')
    a.label('exl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('exd'); a.ST('x3'); a.LD('x2'); a.SHR(1); a.ST('x2'); a.JMP('exl')
    a.label('exd')
    a.LD('x2'); a.RET()

    # ---- SIN: ACC = 32768 sin(pi/2 * phi / 2^24), phi in ACC (0 .. 2^24): 256-step table, 16-bit interpolation
    a.label('SIN')
    a.ST('sp'); a.SHR(16); a.ST('si'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('si'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('sd')
    a.LD('si'); a.MULI(256); a.MULI(256); a.ST('x0'); a.LD('sp'); a.SUB('x0')
    a.MUL('sd'); a.SHR(16); a.ADD('s0'); a.RET()

    # ---- W24: ACC mod 2^24 (a quarter turn), any sign
    a.label('W24')
    a.ST('x0'); a.SHR(24); a.MULI(1024); a.MULI(1024); a.dbl('x1', 4); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.RET()

    # ---- DDA: one accumulator's five table words from its pixel step pc and line step lc (Q12)
    a.label('DDA')
    a.LD('tb'); a.OUT('TA'); a.ADDI(8); a.ST('tb')
    a.LD('pc'); a.OUT('TD')
    a.LD('pc'); a.MUL('DL'); a.ST('t0'); a.LD('lc'); a.SUB('t0'); a.ST('t1'); a.OUT('TD')
    a.LD('pc'); a.MUL('X0'); a.ST('t0'); a.LD('lc'); a.MUL('Y0'); a.ADD('t0'); a.SHR(2); a.OUT('TD')
    a.LD('t1'); a.SUB('lc'); a.SUB('lc'); a.OUT('TD')
    a.ADD('lc'); a.OUT('TD')
    a.RET()

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
    #      (Q14); ivy = lines per virtual px (Q16).  The centre is half the measured width and line count.
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

    # ---- Point: phi (the edge normal's angle from the point's axis), 52 .. 79.2 .. 88 degrees; c = cos, s = sin
    a.LD('ap'); a.SUB(C(32768)); a.SKIP('LT'); a.JMP('phi')
    a.LD('ap'); a.MUL(C(9904)); a.SHR(6); a.ADD(C(9693400)); a.JMP('pst')
    a.label('phi')
    a.MUL(C(3210)); a.SHR(6); a.ADD(C(14764248))
    a.label('pst')
    a.ST('phi'); a.CALL('SIN'); a.ST('s')
    a.LD(C(Q24)); a.SUB('phi'); a.CALL('SIN'); a.ST('c')
    a.LD('c'); a.MUL('pw'); a.SHR(14); a.ST('cpw'); a.LD('s'); a.MUL('pw'); a.SHR(14); a.ST('spw')
    a.LD('c'); a.MUL('lh'); a.SHR(14); a.ST('clh'); a.LD('s'); a.MUL('lh'); a.SHR(14); a.ST('slh')

    # ---- Drift: |v| < 24 still; else 2^(-13 + ls / 64) quarter turn per field, ls = (|v| - 24) * 0.92 (top 2^-6)
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(4); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(6); a.ST('x0')
    a.LD('ls'); a.SUB('x0'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('w'); a.SHR(4); a.ST('w')
    a.LD('v'); a.SKIP('LT'); a.JMP('dpos')
    a.LD('th'); a.SUB('w'); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(1 << 25)); a.JMP('dst')
    a.label('dpos')
    a.LD('th'); a.ADD('w'); a.SUB(C(1 << 25)); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(1 << 25))
    a.label('dst')
    a.ST('th')
    a.label('still')

    # ---- the camera gain: 0 .. 147 (the grid holds min(255, luma8 * g >> 7))
    a.LD('a6'); a.SHR(6); a.MULI(148); a.SHR(10); a.ST('g6')

    # ---- Scale and the centre
    a.LD('a3'); a.MULI(205); a.SHR(14); a.ST('lsc')
    a.IN('W'); a.ADDI(NEXTRA); a.ST('N')
    a.SKIP('NFIELD'); a.JMP('field')

    # Rose: R1 = 175 .. 530 .. 1605 virtual px; centre = the screen centre +- 640 / 360 virtual px
    a.LD('lsc'); a.ADDI(1907); a.CALL('EXPQ4'); a.ST('S4')
    a.LD('a1'); a.SUB(C(32768)); a.MULI(10); a.SHR(5); a.MUL('ivx'); a.SHR(16); a.ST('x0')
    a.IN('W'); a.ST('x1'); a.ADD('x1'); a.ADD('x0'); a.ADDI(4 * XOFS); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.ST('X0')
    a.LD('a2'); a.SUB(C(32768)); a.MULI(90); a.SHR(9); a.MUL('ivy'); a.SHR(18); a.ST('x0')
    a.IN('HN'); a.ST('x1'); a.ADD('x1'); a.SUB('x0'); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.ST('Y0')
    a.LD('N'); a.ST('DL')
    a.LD(C(4095)); a.ST('n0x'); a.ST('n0y'); a.LDI(0); a.ST('wc'); a.ST('hc'); a.ST('ox'); a.ST('oy')
    a.JMP('scd')

    # Field: square cells of 89 .. 270 .. 818 virtual px, mirrored at their borders; a rose centre at the screen
    # centre + pan (K1, K2: +- one cell)
    def reduce(n):
        a.CALL('RTRI')

    def tri():
        pass

    # RTRI: ACC mod 2 fw (ACC < 256 fw) = theta -> ft (theta), fx (position, half steps), fd (1 = backward),
    # fn (clocks to the flip)
    a.JMP('rtx')
    a.label('RTRI')
    for k in range(6, -1, -1):
        a.SUB('fm%d' % k); a.SKIP('GE'); a.ADD('fm%d' % k)
    a.ST('fth'); a.SUB('fw'); a.SKIP('LT'); a.JMP('rtf')
    a.LD('fth'); a.ADD('fth'); a.SUB('fw'); a.ADD('fo'); a.ST('fx'); a.LDI(0); a.ST('fd')
    a.LD('fw'); a.SUB('fth'); a.ST('fn'); a.RET()
    a.label('rtf')
    a.LD('fw'); a.ADD('fw'); a.ADD('fw'); a.SUB('fo'); a.SUB('fth'); a.SUB('fth'); a.ST('fx'); a.LDI(1); a.ST('fd')
    a.LD('fw'); a.ADD('fw'); a.SUB('fth'); a.ST('fn'); a.RET()
    a.label('rtx')

    def fold0(pan, anchor):
        """fw = the cell size; anchor = the centre's index; leaves theta0 in ft0 and tri() of it."""
        a.LD('fw'); a.SHR(1); a.ST('x0'); a.ADD('x0'); a.ST('x0'); a.LD('fw'); a.SUB('x0'); a.ST('fo')
        a.LD('fw'); a.ADD('fw'); a.ST('fm0')
        for k in range(1, 7):
            a.ADD('fm%d' % (k - 1)); a.ST('fm%d' % k)
        a.LD(pan); a.SUB(C(32768)); a.MUL('fw'); a.SHR(15); a.ADD(anchor); a.ST('x1')
        a.LD('fw'); a.SUB('fo'); a.SHR(1); a.SUB('x1'); a.ADD('fm5'); reduce(6); a.LD('fth'); a.ST('ft0')

    a.label('field')
    a.LD('lsc'); a.ADDI(1658); a.CALL('EXPQ4'); a.ST('cell4')
    a.MUL('ivx'); a.ADD(C(1 << 17)); a.SHR(18); a.ST('wc')
    a.LD('cell4'); a.MUL('ivy'); a.ADD(C(1 << 19)); a.SHR(20); a.ST('hc')
    a.LD('wc'); a.MUL('xs'); a.MUL(C(FR_Q10)); a.SHR(10); a.ST('S4')
    # x
    a.LD('wc'); a.ST('fw'); a.IN('W'); a.SHR(1); a.ADDI(XOFS); a.ST('x2')
    fold0('a1', 'x2')
    a.LD('fo'); a.ST('ox'); a.LD('fn'); a.ST('n0x'); a.LD('fx'); a.ST('x3'); a.LD('fd'); a.ST('dx')
    a.LD('ft0'); a.ADD('N'); reduce(6)
    n1, n2 = a.new(), a.new()
    a.LD('dx'); a.SUBI(1); a.SKIP('LT'); a.JMP(n1)
    a.LD('x3'); a.ADD('x3'); a.ST('X0'); a.LD('fx'); a.SUB('x3'); a.SHR(1); a.ST('DL'); a.JMP(n2)
    a.label(n1)
    a.LDI(0); a.SUB('x3'); a.SUB('x3'); a.ST('X0'); a.LD('x3'); a.SUB('fx'); a.SHR(1); a.ST('DL')
    a.LDI(0); a.SUB('cpw'); a.ST('cpw'); a.LDI(0); a.SUB('spw'); a.ST('spw')
    a.label(n2)
    # y
    a.LD('hc'); a.ST('fw'); a.IN('HN'); a.SHR(1); a.ST('x2'); a.LD(C(65536)); a.SUB('a2'); a.ST('a2n')
    fold0('a2n', 'x2')
    a.LD('fo'); a.ST('oy'); a.LD('fn'); a.ST('n0y')
    n1, n2 = a.new(), a.new()
    a.LD('fd'); a.SUBI(1); a.SKIP('LT'); a.JMP(n1)
    a.LD('fx'); a.ADD('fx'); a.ST('Y0'); a.JMP(n2)
    a.label(n1)
    a.LDI(0); a.SUB('fx'); a.SUB('fx'); a.ST('Y0')
    a.LDI(0); a.SUB('clh'); a.ST('clh'); a.LDI(0); a.SUB('slh'); a.ST('slh')
    a.label(n2)

    a.label('scd')
    # ---- R1 cos(phi) (Q4 virtual px) and the Fine line weight R1 / 80 within 2.5 .. 6 virtual px, 1/8 unit
    a.LD('S4'); a.MUL('c'); a.SHR(15); a.ST('Sc')
    a.LD('S4'); a.MULI(819); a.SHR(16); a.ST('x0'); a.SUBI(40); a.SKIP('LT'); a.JMP('tlo'); a.LDI(40); a.ST('x0')
    a.label('tlo')
    a.LD('x0'); a.SUBI(96); a.SKIP('GE'); a.JMP('tok'); a.LDI(96); a.ST('x0')
    a.label('tok')
    a.LD('x0'); a.SHR(1); a.CALL('SHRU'); a.ST('t8')

    # ---- per star: a stub sets the angle, radius and table base, the shared body runs and dispatches back
    a.IN('NZ'); a.ST('nz')
    for i in range(NSTAR):
        if i == 0:
            a.LD('th')
        elif i == 1:
            a.LD('th'); a.SHR(1); a.ST('x0'); a.LD(C(BASE[1])); a.SUB('th'); a.SUB('x0')
        elif i == 2:
            a.LD(C(BASE[2])); a.ADD('th'); a.ADD('th')
        else:
            a.LD(C(BASE[i])); a.SUB('th'); a.SUB('th'); a.SUB('th')
        a.ST('al'); a.LDI(i); a.ST('di'); a.LD(C(RHO[i])); a.ST('rho'); a.LDI(32 * i); a.ST('tb')
        a.LDI(NS_BY_NZ.index(next(n for n in NS_BY_NZ if n > i))); a.ST('nmin')
        a.JMP('body')
        a.label('ret%d' % i)
    # ---- the constants block
    a.LDI(CONST_ADR); a.OUT('TA')
    for i in range(NSTAR):
        a.LD('K%d' % i); a.OUT('TD')
    a.LD('t8'); a.SHR(1); a.MULI(256); a.ADD('t8'); a.OUT('TD')
    a.LD('ox'); a.MUL(C(4096)); a.ADD('wc'); a.MUL(C(4096)); a.ADD('n0x'); a.OUT('TD')
    a.LD('oy'); a.MUL(C(4096)); a.ADD('hc'); a.MUL(C(4096)); a.ADD('n0y'); a.OUT('TD')
    a.LD('g6'); a.OUT('TD')
    a.HALT()

    a.label('body')
    a.LD('al'); a.CALL('W24'); a.ST('al'); a.CALL('SIN'); a.ST('sa')
    a.LD(C(Q24)); a.SUB('al'); a.CALL('SIN'); a.ST('ca')
    a.LD('cpw'); a.MUL('ca'); a.SHR(18); a.ST('pc'); a.LD('clh'); a.MUL('sa'); a.SHR(18); a.ST('lc'); a.CALL('DDA')
    a.LD('spw'); a.MUL('ca'); a.SHR(18); a.ST('pc'); a.LD('slh'); a.MUL('sa'); a.SHR(18); a.ST('lc'); a.CALL('DDA')
    a.LD('cpw'); a.MUL('sa'); a.SHR(18); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.ST('pc')
    a.LD('clh'); a.MUL('ca'); a.SHR(18); a.ST('lc'); a.CALL('DDA')
    a.LD('spw'); a.MUL('sa'); a.SHR(18); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.ST('pc')
    a.LD('slh'); a.MUL('ca'); a.SHR(18); a.ST('lc'); a.CALL('DDA')
    # K = rho R1 cos(phi), 1/8 unit; -16384 when the star is not drawn
    a.LD('nz'); a.SUB('nmin'); a.SKIP('LT'); a.JMP('kon'); a.LD(C(-16384)); a.JMP('kst')
    a.label('kon')
    a.LD('Sc'); a.MUL('rho'); a.SHR(12); a.CALL('SHRU')
    a.label('kst')
    a.ST('kv')
    for i in range(NSTAR):
        nx = a.new()
        a.LD('di'); a.SUBI(i + 1); a.SKIP('LT'); a.JMP(nx)
        a.LD('kv'); a.ST('K%d' % i); a.JMP('ret%d' % i)
        a.label(nx)
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; windrose.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000, check=False):
    """inp: dict over INS names (K1.. raw knobs, YS, XS, IL, W, HN, NZ); flags: GLIDE, FIELD.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (table: dict address -> signed 32-bit word, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ta = 0
    table = {}
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
            if n == OUTS['ADR']: adr = acc & 511
            elif n == OUTS['TA']: ta = acc & 255
            elif n == OUTS['TD']:
                table[ta] = _sx(acc, 32); ta = (ta + 1) & 255
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'FIELD'}[c]])
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
    parts = ["-- Generated by windrose.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package windrose_pkg is",
             vhd_table("C_WR_TAB", 512, 16, TAB),
             vhd_table("C_WR_UC", 1024, 16, uc, "std_logic_vector"),
             vhd_table("C_WR_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             "end package windrose_pkg;", ""]
    with open(os.path.join(HERE, "windrose_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
