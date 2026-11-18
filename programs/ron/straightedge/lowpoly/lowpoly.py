#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LOWPOLY v0.1.0 (Straightedge) - build hook.  Writes lowpoly_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.straightedge_work/rb_lowpoly/imodel.py) imports this file
and runs the same per-field microprogram through emulate(), so that arithmetic has one source.

Package contents (block-RAM ROMs):
  C_LP_EXP    256 x 16   round(32768 * 2^(i/256))    (facet size and drift speed are exponential)
  C_LP_UC    1024 x 16   the per-field microprogram (see PROGRAM below)
  C_LP_RFI    256 x 32   the engine's register file at power-up (holds and glides at the TOML defaults)

The per-field engine (after Hardline flute, via Vellum cassini): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial multiplier (18 x 18 signed) and a serial right shift.  Micro-op = 4-bit opcode +
12-bit operand.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def exp_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


FT = exp_rom()
DEFAULTS = dict(k1=512, k2=512, k3=560, k4=640, k5=680, k6=1023, p12=512)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, XS=8, YS=9, ROM=11, MEAN=15)
CONDS = dict(LT=0, GLIDE=1, BUSY=2, SRC=3, GE=8, NGLIDE=9, NBUSY=10, NSRC=11)
OUTS = dict(ADR=0, CWP=1, RHL=2, HR2=3, CWH=4, C4=5, C12=6, LR=7, LPL=8, LPLQ=9, WC=10, RS=11, RHV=12, BASE=13,
            VR=14, RR=15, X0=16, CI=17, XC=18, BX1=19, BX0=20, YA=21, YB=22, GR=23, PH8=24,
            FI0=25, FI1=26, FM=27, FGO=28, PREP=29, OX=30, OY=31)
RFW = 32
NREG = 256
FILL_CLOCKS = 260          # one table pass: 256 entries + start / stop


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
            assert len(self.regs) <= NREG
            self.init[name] = init
        return self.regs[name]

    def const(self, v):
        """a register that holds the constant v (never stored to)"""
        self.reg('k%d' % v, v)
        return 'k%d' % v

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

    def rom(self):
        """ACC = EXP[ACC]"""
        self.OUT('ADR'); self.IN('ROM')

    def wait(self):
        """until the table fill is idle"""
        l = self.new()
        self.label(l); self.SKIP('NBUSY'); self.JMP(l)

    def neg_if(self, flag, src, dst):
        """dst = flag < 0 ? -src : src"""
        p, d = self.new(), self.new()
        self.LD(flag); self.SKIP('LT'); self.JMP(p)
        self.LDI(0); self.SUB(src); self.JMP(d)
        self.label(p); self.LD(src)
        self.label(d); self.ST(dst)


# fill pass codes: destination (0 Y, 1 S, 2 ZX, 3 ZY, 4 G) + 16 * (first half falls)
# + 32 * (second half falls)
F_Y, F_S, F_ZX, F_ZY, F_G = 0, 1, 2, 3, 4


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults; knobs are read as 8 x the knob);
    # 'first' < 0 on the first field
    for n, k in (('h1', 'k1'), ('h2', 'k2'), ('h3', 'k3'), ('h4', 'k4'), ('h5', 'k5'), ('h6', 'k6'), ('h12', 'p12')):
        a.reg(n, D[k] * 8)
    for n, k in (('g1', 'k1'), ('g2', 'k2'), ('g3', 'k3'), ('g4', 'k4'), ('g6', 'k6'), ('g12', 'p12')):
        a.reg(n, D[k] * 8)
    a.reg('vgs', (D['k5'] - 512) * 8)
    a.reg('dlf', 0); a.reg('dli', 0)
    a.reg('first', -1)
    # constants (never stored to)
    for n, v in (('c8184', 8184), ('c4096', 4096), ('c2048', 2048), ('c1024', 1024), ('c16384', 16384), ('c256', 256),
                 ('kA0', 9176), ('kA1', 36704), ('kA2', 18352), ('kzr', 17280), ('kzg', 7040), ('cbx', 30720),
                 ('cyr', 8640), ('c21', 2097152)):
        a.reg(n, v)

    a.JMP('main')

    # ---- CH: hold + glide of one knob (Q3; kin = 8 x the knob, hh = its hold, gg = its glide).  Hold: the
    # knob moves the held value only by 2 counts or more, to either end, or on the first field.  Glide On:
    # 1/16 of the gap per field, never less than half a count, landing exactly; Glide off or the first
    # field: the held value.
    a.label('CH')
    a.LD('kin'); a.SUB('hh'); a.ADDI(8); a.SKIP('GE'); a.JMP('chtk')
    a.SUBI(24); a.SKIP('LT'); a.JMP('chtk')
    a.LD('first'); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUBI(1); a.SKIP('GE'); a.JMP('chtk')
    a.LD('kin'); a.SUB('c8184'); a.SKIP('LT'); a.JMP('chtk')
    a.JMP('chgl')
    a.label('chtk'); a.LD('kin'); a.ST('hh')
    a.label('chgl')
    a.LD('hh'); a.SUB('gg'); a.ST('t2')
    a.SKIP('GLIDE'); a.JMP('chsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('chsn')
    a.LD('t2'); a.abs_('t3')
    a.SUBI(65); a.SKIP('LT'); a.JMP('ches')
    a.ADDI(60); a.SKIP('LT'); a.JMP('chst')                # |gap| - 5
    a.label('chsn'); a.LD('hh'); a.ST('gg'); a.RET()
    a.label('ches'); a.LD('t2'); a.SHR(4); a.ADD('gg'); a.ST('gg'); a.RET()
    a.label('chst'); a.LDI(4); a.ST('t1'); a.LD('t2'); a.SKIP('GE'); a.LDI(-4); a.SKIP('LT'); a.LD('t1')
    a.ADD('gg'); a.ST('gg'); a.RET()

    # ---- POW2: ACC = 2^ACC (ACC = 0 .. 7)
    a.label('POW2')
    for i in range(7):
        a.SUBI(1); a.SKIP('GE'); a.JMP('pw%d' % i)
    a.LDI(128); a.RET()
    for i in range(7):
        a.label('pw%d' % i); a.LDI(1 << i); a.RET()

    # ---- DIV: dq = dn / dd (7 quotient bits, restoring), the remainder stays in dn
    a.label('DIV')
    a.LD('dd'); a.ADD('dd'); a.ST('d1'); a.ADD('d1'); a.ST('d2'); a.ADD('d2'); a.ST('d3'); a.ADD('d3'); a.ST('d4')
    a.ADD('d4'); a.ST('d5'); a.ADD('d5'); a.ST('d6'); a.LDI(0); a.ST('dq')
    for b in range(6, -1, -1):
        a.LD('dn'); a.SUB('d%d' % b if b else 'dd'); a.SKIP('GE'); a.JMP('dv%d' % b)
        a.ST('dn'); a.LD('dq'); a.ADDI(1 << b); a.ST('dq')
        a.label('dv%d' % b)
    a.RET()

    # ---- CLP: ACC = clamp(ACC >> 9, 0, 31): a grid row from a virtual y (Q4)
    a.label('CLP'); a.SHR(9); a.SKIP('GE'); a.LDI(0); a.ST('t0'); a.SUBI(32); a.SKIP('LT'); a.JMP('clh'); a.LD('t0'); a.RET()
    a.label('clh'); a.LDI(31); a.RET()

    # ---- LO: the table pass of amplitude step fm, signed by fsg (+-1): lo = m (64 - phf / 256) with
    # m = fm fsg; fd = which half falls (16: the first, 32: the second)
    a.label('LO')
    a.LD('fm'); a.MUL('fsg'); a.MUL('phc'); a.SHR(8); a.ST('lo')
    a.LDI(32); a.ST('fd'); a.LD('fsg'); a.SKIP('LT'); a.RET(); a.LDI(16); a.ST('fd'); a.RET()
    a.label('LO1')
    a.LD('fm'); a.MUL('phc'); a.SHR(8); a.ST('lo'); a.LDI(32); a.ST('fd'); a.RET()
    # ---- PASS: launch one pass: values fb -+ lo, step fm, code fc + fd
    a.label('PASS')
    a.SKIP('NBUSY'); a.JMP('PASS')
    a.LD('fb'); a.SUB('lo'); a.OUT('FI0'); a.LD('fb'); a.ADD('lo'); a.OUT('FI1'); a.LD('fm'); a.OUT('FM')
    a.LD('fc'); a.ADD('fd'); a.OUT('FGO'); a.RET()
    # ---- SGN: ACC = +1 / -1 by the sign of ACC
    a.label('SGN'); a.SKIP('LT'); a.JMP('sgp'); a.LDI(-1); a.RET()
    a.label('sgp'); a.LDI(1); a.RET()

    def fill(dest, m, sgn=None):
        a.LD(m); a.ST('fm')
        if sgn is None: a.CALL('LO1')
        else: a.LD(sgn); a.ST('fsg'); a.CALL('LO')
        if dest in (F_ZX, F_ZY): a.LD('c21')
        else: a.LDI(0)
        a.ST('fb'); a.LDI(dest); a.ST('fc'); a.CALL('PASS')

    def knob(inp, h, g):
        """one knob: nothing to do when it equals its hold and its glide; else the CH subroutine (g = None:
        hold only)"""
        slow, done = a.new(), a.new()
        a.IN(inp); a.ST('kin'); a.SUB(h); a.SKIP('GE'); a.JMP(slow); a.SUBI(1); a.SKIP('LT'); a.JMP(slow)
        if g is not None:
            a.LD(h); a.SUB(g); a.SKIP('GE'); a.JMP(slow); a.SUBI(1); a.SKIP('LT'); a.JMP(slow)
        a.JMP(done)
        a.label(slow); a.LD(h); a.ST('hh')
        if g is not None: a.LD(g)
        a.ST('gg'); a.CALL('CH'); a.LD('hh'); a.ST(h)
        if g is not None: a.LD('gg'); a.ST(g)
        a.label(done)

    a.label('main')
    # ---- the video mode's constants, by the pixel step xs (half virtual px: 2, 3, 4) and the line step ys
    # (virtual lines Q8): kcw = the column width's scale, nx6 = pixel 0's distance left of the screen centre
    # (px Q6), rm = row height / column width (lines per px, Q16: 7/8 in virtual px), lm = lean 1 in px per
    # line (Q12), p6 = line 0's distance above the centre (lines Q6)
    a.IN('XS'); a.ST('xs'); a.SUBI(3); a.SKIP('GE'); a.JMP('mx2'); a.SUBI(1); a.SKIP('GE'); a.JMP('mx3')
    for lab, kcw, nx6 in (('mx4', 16384, 23008), ('mx3', 21845, 40928), ('mx2', 32768, 61408)):
        if lab != 'mx4': a.label(lab)
        a.LD(a.const(kcw)); a.ST('kcw'); a.LD(a.const(nx6)); a.ST('nx6')
        if lab != 'mx2': a.JMP('mxd')
    a.label('mxd')
    a.IN('YS'); a.ST('ys')
    prev = 0
    for ysv, edge in ((256, 320), (384, 432), (480, 496), (512, 544), (576, 768), (960, 1049)):
        a.SUBI(edge - prev); a.SKIP('GE'); a.JMP('my%d' % ysv); prev = edge
    for ysv, rm, lm, p6 in ((1138, 25800, 9104, 7743), (960, 30583, 7680, 9184), (576, 50972, 4608, 15328),
                            (512, 28672, 8192, 17248), (480, 61167, 3840, 18400), (384, 57344, 4096, 23008),
                            (256, 57344, 4096, 34528)):
        if ysv != 1138: a.label('my%d' % ysv)
        a.LD(a.const(rm)); a.ST('rm'); a.LD(a.const(lm)); a.ST('lm'); a.LD(a.const(p6)); a.ST('p6')
        if ysv != 256: a.JMP('myd')
    a.label('myd')
    a.LD('rm'); a.SHR(3); a.OUT('RS')
    for inp, h, g in (('K1', 'h1', 'g1'), ('K2', 'h2', 'g2'), ('K3', 'h3', 'g3'), ('K4', 'h4', 'g4'),
                      ('K5', 'h5', None), ('K6', 'h6', 'g6'), ('P12', 'h12', 'g12')):
        knob(inp, h, g)

    # ---- DRIFT (Hypnos): speed glide 1/8 per field (Q3); |v| < 24 still, else 2^(ls/64 - 12) cycle per
    # field, ls = u - u/16 - u/32 (at most 443: 2^-5), u = |v| - 24.  The phase = dli (1/256 cycle, the
    # tables' index offset) + dlf / 4096 (the fraction the table fills are shifted by); it steps down for a
    # positive speed (the ripple travels right).
    a.LD('h5'); a.SUB('c4096'); a.ST('t1')
    a.SKIP('GLIDE'); a.JMP('vsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('vsn')
    a.LD('t1'); a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('vgs'); a.JMP('vdec')
    a.label('vsn'); a.LD('t1'); a.ST('vgs')
    a.label('vdec')
    a.LD('vgs'); a.SHR(3); a.ST('v'); a.abs_('t3'); a.SUBI(24); a.SKIP('GE'); a.JMP('dd')
    a.ST('u'); a.SHR(4); a.ST('t0'); a.LD('u'); a.SHR(5); a.ST('t2'); a.LD('u'); a.SUB('t0'); a.SUB('t2')
    a.ST('ls'); a.ADD('ls'); a.ST('t0'); a.ADD('t0'); a.rom(); a.SHR(7); a.ST('m')        # EXP[(4 ls) & 255] >> 7
    a.LD('ls'); a.SHR(6); a.CALL('POW2'); a.MUL('m'); a.ST('spd')
    a.LD('v'); a.SKIP('LT'); a.JMP('fwd')
    a.LD('dlf'); a.ADD('spd'); a.JMP('nrm')
    a.label('fwd'); a.LD('dlf'); a.SUB('spd')
    a.label('nrm'); a.ST('dlf')
    a.label('nl'); a.LD('dlf'); a.SKIP('LT'); a.JMP('nh')
    a.ADD('c4096'); a.ST('dlf'); a.LD('dli'); a.SUBI(1); a.ST('dli'); a.JMP('nl')
    a.label('nh'); a.SUB('c4096'); a.SKIP('GE'); a.JMP('nd')
    a.ST('dlf'); a.LD('dli'); a.ADDI(1); a.ST('dli'); a.JMP('nl')
    a.label('nd'); a.LD('dli'); a.SKIP('GE'); a.ADD('c256'); a.ST('dli'); a.SUB('c256'); a.SKIP('LT'); a.ST('dli')
    a.label('dd'); a.LD('dli'); a.OUT('PH8'); a.LD('dlf'); a.SHR(4); a.ST('phf'); a.LD('c16384'); a.SUB('phf'); a.ST('phc')

    # ---- SCALE: column width = 32 * 2^(oct/256) virtual px, oct = K3 * 919 >> 16 (0 .. 918); in px (Q6)
    # cwp = (M kcw >> 16) 2^n >> 3; rows 7/8 of it, in lines
    a.LD('g3'); a.MULI(919); a.SHR(13); a.ST('oct'); a.rom(); a.ST('M')
    a.LD('oct'); a.SHR(8); a.CALL('POW2'); a.ST('p2')
    a.LD('M'); a.MUL('kcw'); a.SHR(16); a.MUL('p2'); a.SHR(3); a.ST('cwp'); a.OUT('CWP')
    a.MUL('rm'); a.SHR(16); a.ST('rhl'); a.OUT('RHL')
    a.SHR(1); a.ST('hrh'); a.LD('rhl'); a.SUB('hrh'); a.OUT('HR2')

    # ---- RELIEF: displacement amplitude A (rows, Q16) = 0.56 p up to 50 %, then 0.14 + 0.28 p; in lines (Q6)
    # and per px of column (Q16)
    a.LD('g12'); a.SUB('c4096'); a.SKIP('LT'); a.JMP('ahi')
    a.LD('g12'); a.MUL('kA1'); a.SHR(13); a.JMP('adn')
    a.label('ahi'); a.LD('g12'); a.MUL('kA2'); a.SHR(13); a.ADD('kA0')
    a.label('adn'); a.ST('A16')
    a.MUL('rhl'); a.SHR(16); a.ST('My')
    a.LD('A16'); a.MUL('rm'); a.SHR(16); a.ST('Ms')
    fill(F_Y, 'My')

    # ---- the rest of the scale constants (while the first table fills): the column in half virtual px
    # (Q4), its quarter points (the camera samples), the row in virtual lines (7/16 of that)
    a.LD('cwp'); a.MUL('xs'); a.SHR(2); a.ST('cwh'); a.OUT('CWH')
    a.SHR(2); a.ST('c4'); a.OUT('C4'); a.LD('cwh'); a.SUB('c4'); a.OUT('C12')
    a.LD('cwh'); a.SHR(1); a.ST('t0'); a.LD('cwh'); a.SHR(4); a.ST('t1'); a.LD('t0'); a.SUB('t1'); a.ST('rhv'); a.OUT('RHV')
    fill(F_S, 'Ms')

    # ---- LIGHT: lx = 0.5 + 0.75 X, ly = 0.3 + 0.75 Y (Q12), never both near zero; z amplitude Zr = 270 p
    # (stored / 4, Q8), the regular studs Zreg = 110 max(0, 1 - 2p)
    a.LD('g1'); a.SUB('c4096'); a.ST('t0'); a.SHR(2); a.ST('t1'); a.LD('c2048'); a.ADD('t0'); a.SUB('t1'); a.ST('lx')
    a.LD('g2'); a.SUB('c4096'); a.ST('t0'); a.SHR(2); a.ST('t1'); a.LDI(1229); a.ADD('t0'); a.SUB('t1'); a.ST('ly')
    a.LD('lx'); a.CALL('SGN'); a.ST('sx'); a.LD('ly'); a.CALL('SGN'); a.ST('sy')
    a.LD('lx'); a.abs_('t3'); a.ST('alx'); a.LD('ly'); a.abs_('t3'); a.ST('aly')
    a.ADD('alx'); a.SUB('c1024'); a.SKIP('LT'); a.JMP('lok'); a.LD('c1024'); a.SUB('alx'); a.ST('aly')
    a.label('lok')
    a.LD('g12'); a.MUL('kzr'); a.SHR(13); a.ST('zr')
    a.LD('alx'); a.MUL('zr'); a.SHR(12); a.ST('Mzx'); a.LD('aly'); a.MUL('zr'); a.SHR(12); a.ST('Mzy')
    a.LD('c4096'); a.SUB('g12'); a.SKIP('GE'); a.LDI(0); a.MUL('kzg'); a.SHR(12); a.ST('zg')
    a.LD('lx'); a.MUL('zg'); a.SHR(20); a.OUT('OX')
    a.LD('aly'); a.MUL('sy'); a.MUL('zg'); a.SHR(20); a.OUT('OY')
    fill(F_ZX, 'Mzx', 'sx')

    # ---- LEAN: (K4 - 512) past an 8-count dead centre; px per line (Q12), signed
    a.LD('g4'); a.SUB('c4096'); a.ST('lv'); a.abs_('t3'); a.SUBI(64); a.SKIP('GE'); a.LDI(0)
    a.MUL('lm'); a.SHR(12); a.ST('lpl'); a.LD('lv'); a.CALL('SGN'); a.ST('sl')
    a.MUL('lpl'); a.OUT('LPL'); a.SHR(8); a.OUT('LPLQ'); a.abs_('t3'); a.ADDI(16); a.OUT('WC')
    # pixel 0 of line 0, px left of the lattice origin (the screen centre): by the column width
    a.LD('lpl'); a.MUL('p6'); a.SHR(12); a.MUL('sl'); a.ADD('nx6'); a.ST('dn'); a.LD('cwp'); a.ST('dd'); a.CALL('DIV')
    a.LD('dn'); a.SUBI(1); a.SKIP('LT'); a.JMP('xnz')
    a.LDI(0); a.OUT('X0'); a.LD('dq'); a.JMP('xdn')
    a.label('xnz'); a.LD('cwp'); a.SUB('dn'); a.OUT('X0'); a.LD('dq'); a.ADDI(1)
    a.label('xdn'); a.ST('nci'); a.LDI(0); a.SUB('nci'); a.OUT('CI')
    a.LD('nci'); a.MUL('cwh'); a.ST('t0'); a.LDI(0); a.SUB('t0'); a.OUT('XC')
    fill(F_ZY, 'Mzy', 'sy')

    # line 0, lines above the origin + half a row: by the row height
    a.LD('p6'); a.SUB('hrh'); a.ST('dn'); a.LD('rhl'); a.ST('dd'); a.CALL('DIV')
    a.LD('dn'); a.SUBI(1); a.SKIP('LT'); a.JMP('rnz')
    a.LDI(0); a.ST('vrp'); a.LD('dq'); a.JMP('rdn')
    a.label('rnz'); a.LD('rhl'); a.SUB('dn'); a.ST('vrp'); a.LD('dq'); a.ADDI(1)
    a.label('rdn'); a.ST('nr'); a.LD('vrp'); a.SUB('hrh'); a.OUT('VR'); a.LDI(0); a.SUB('nr'); a.OUT('RR')
    # the shear of one row (half virtual px, Q4), the camera sample offsets of rows r and r - 1
    a.LD('lpl'); a.MUL('rhl'); a.SHR(11); a.MUL('xs'); a.SHR(3); a.MUL('sl'); a.ST('LR'); a.OUT('LR')
    a.LD('nr'); a.MUL('LR'); a.ADD('cbx'); a.ST('t0'); a.LD('LR'); a.SHR(1); a.ST('t1'); a.LD('t0'); a.SUB('t1')
    a.OUT('BX1'); a.ADD('LR'); a.OUT('BX0')
    # the camera sample rows: virtual y (Q4) of row r's middle, and the grid rows of rows r - 1 and r
    a.LD('nr'); a.MUL('rhv'); a.ST('t0'); a.LD('cyr'); a.SUB('t0'); a.ST('yr')
    a.LD('rhv'); a.SHR(1); a.ST('t1'); a.ADD('yr'); a.ST('ya'); a.OUT('YA')
    a.SUB('rhv'); a.CALL('CLP'); a.OUT('GR'); a.LD('ya'); a.CALL('CLP'); a.OUT('GR')

    # ---- CAMERA: gain per level (Q16) = 0.75 K6 (Osc: twice), the grid's zero = its mean
    a.LD('g6'); a.ADD('g6'); a.ST('t0'); a.ADD('t0'); a.ADD('t0'); a.ST('gk'); a.SKIP('NSRC'); a.ADD('gk'); a.ST('gk')
    a.wait()
    a.LDI(0); a.OUT('FI0'); a.OUT('FI1'); a.LD('gk'); a.OUT('FM'); a.LDI(F_G); a.OUT('FGO')
    a.IN('MEAN'); a.SHR(1); a.ST('t0'); a.LDI(38); a.SUB('t0'); a.OUT('BASE')
    a.wait()
    a.LDI(0); a.ST('first')
    a.OUT('PREP')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
assert len(UCODE) <= 1024, len(UCODE)
RF_INIT = [0] * NREG
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & ((1 << RFW) - 1)


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; lowpoly.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. = 8 x the raw knobs, XS, YS, MEAN); flags: GLIDE, SRC.
    rf: the register file (list of NREG ints, updated in place; None = the power-up file).
    Returns (outs dict by OUTS name, with 'FILLS' = the table passes (code, FI0, FI1, FM); rf; clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0
    outs = {}
    fills = []
    grs = []
    busy_until = 0
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        if op in (0, 1, 2, 3):
            b = _sx(rf[arg & (NREG - 1)], RFW)
        elif op in (4, 5, 6, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, 37)
        elif op in (2, 6):
            acc = _sx(acc - b, 37)
        elif op in (3, 7):
            assert -(1 << 17) <= acc < (1 << 17) and -(1 << 17) <= b < (1 << 17), ("MUL operand", pc, acc, b)
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 17
        elif op == 8:
            assert -(1 << (RFW - 1)) <= acc < (1 << (RFW - 1)), ("ST overflow", pc, acc)
            rf[arg & (NREG - 1)] = acc & ((1 << RFW) - 1)
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 255
            outs[n] = acc
            if n == OUTS['GR']:
                grs.append(acc & 31)
            if n == OUTS['FGO']:
                assert clocks >= busy_until, "fill launched while busy"
                fills.append((acc & 63, outs[OUTS['FI0']], outs[OUTS['FI1']], outs[OUTS['FM']]))
                busy_until = clocks + FILL_CLOCKS
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            if c == 0: t = acc < 0
            elif c == 2: t = clocks < busy_until
            else: t = bool(flags[{1: 'GLIDE', 3: 'SRC'}[c]])
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
    inv_o = {v: k for k, v in OUTS.items()}
    res = {inv_o[k]: v for k, v in outs.items()}
    res['FILLS'] = fills
    res['GRS'] = grs
    return res, rf, clocks


def vhd_table(name, depth, width, vals):
    hexw = (width + 3) // 4
    mask = (1 << (hexw * 4)) - 1
    lines = ["    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % ("t_" + name.lower(), depth - 1, hexw * 4 - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (hexw, v & mask) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by lowpoly.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package lowpoly_pkg is",
             vhd_table("C_LP_EXP", 256, 16, exp_rom()),
             vhd_table("C_LP_UC", 1024, 16, uc),
             vhd_table("C_LP_RFI", NREG, 32, RF_INIT),
             "end package lowpoly_pkg;", ""]
    with open(os.path.join(HERE, "lowpoly_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
