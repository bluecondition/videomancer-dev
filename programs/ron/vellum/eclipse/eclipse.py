#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ECLIPSE v0.1.0 (Vellum) - build hook.  Writes eclipse_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.vellum_work/rb_eclipse/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_EC_TAB  512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)) (address 512 wraps to entry 0 = 32768 = sin(pi/2))
  C_EC_RV   256 x 16   round(65536 * sqrt(i / 256)), at most 65535       pixel path: square root of a normalised
  C_EC_RS   256 x 16   RV(i + 1) - RV(i) (9 bits)                        r^2 by its top 8 bits + 6 interpolated
  C_EC_Y    512 x 10   tone: index = film (2 bits: 1/2, 1/3, 1/4, 1/5) & summed coverage (7 bits, 16 = one film)
                       -> luma 64 .. 939 (white lands on 775 after the output scaling)
  C_EC_UC   1024 x 16  the per-field microprogram (PROGRAM below)
  C_EC_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                       constants)

The per-field engine is Hardline flute's: a 37-bit accumulator, a 256 x 32-bit register file, one adder, a serial
18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift; micro-op = 4-bit opcode + 12-bit operand.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
FILMS = [2, 3, 4, 5]


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    return e + s


def rv_rom():
    return [min(int(round(65536 * math.sqrt(i / 256.0))), 65535) for i in range(256)]


def rs_rom():
    v = [int(round(65536 * math.sqrt(i / 256.0))) for i in range(257)]
    return [min(v[i + 1] - v[i], 511) for i in range(256)]


def y_rom():
    out = []
    for f in FILMS:
        for s in range(128):
            out.append(min(64 + (s * 875 + 8 * f) // (16 * f), 939))
    return out


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k5=650, k6=1023, p12=512)
NDISC = 4
LK = [0, -47, -92, -132]                 # round(256 log2 k_i): disc sizes 1, .88, .78, .70
RQ = [4096, 3072, 2731, 2048]            # orbit rates, Q12: 1, 3/4, 2/3, 1/2 ...
RSG = [0, 1, 0, 1]                       # ... and directions (1 = against)
SL2 = [1556, 1966, 2785]                 # spread slope above 50 % by discs - 2 (sigma 0.62 -> 1.0, 1.1, 1.3)
STEPN = [8388608, 5592405, 4194304]      # one turn / n, Q24

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XQ=9, XSQ=10, ROM=11, CNT=12, LPX=13, NZ=14,
           ZERO=15)
CONDS = dict(LT=0, GLIDE=1, RINGS=2, GE=8, NGLIDE=9, NRINGS=10)
OUTS = dict(ADR=0, DI=1, V=2, U=3, D0=4, R2=5, EM=6, GT=7, ER=8, RS=9, T1=10, T2=11)


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


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up = the TOML defaults), then constants
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h5', D['k5']), ('h6', D['k6']),
                 ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a5', D['k5']), ('a6', D['k6']),
                 ('ap', D['p12'])):
        a.reg(n, v * 64)
    for i in range(NDISC):
        a.reg('th%d' % i, 0)
    for n, v in (('C2335', 2335), ('C2735', 2735), ('C4915', 4915), ('C3277', 3277), ('C34560', 34560),
                 ('T24', 1 << 24), ('Q22', 1 << 22), ('C16384', 16384), ('OFF0', 1677722)):
        a.reg(n, v)
    for i in range(3):
        a.reg('SL%d' % i, SL2[i]); a.reg('ST%d' % i, STEPN[i])
    for i in range(NDISC):
        a.reg('RQ%d' % i, RQ[i])

    a.JMP('main')

    # ---- WRAP: ACC mod 2^24 (one turn), for ACC in [-2^24, 2^25)
    a.label('WRAP')
    a.ST('t'); a.SKIP('LT'); a.JMP('wrhi'); a.ADD('T24'); a.RET()
    a.label('wrhi')
    a.SUB('T24'); a.SKIP('GE'); a.ADD('T24'); a.RET()

    # ---- EXPQ4: ACC = 16 * 2^(L / 256) = EXP[L & 255] * 2^(e - 7) >> 4, e = L >> 8 in 7 .. 11
    a.label('EXPQ4')
    a.ST('x0'); a.SHR(8); a.ST('x1'); a.MULI(256); a.ST('x2'); a.LD('x0'); a.SUB('x2'); a.OUT('ADR'); a.IN('ROM')
    a.ST('x2'); a.LD('x1'); a.SUBI(7); a.ST('x3')
    a.label('exl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('exd'); a.ST('x3'); a.LD('x2'); a.ADD('x2'); a.ST('x2'); a.JMP('exl')
    a.label('exd')
    a.LD('x2'); a.SHR(4); a.RET()

    # ---- EM: 2^(-T / 256) as 2^-e, 3/4 2^-e or 2^-(e + 1): emE = e, emM = the 3/4 flag (T in ACC)
    a.label('EM')
    a.ST('x0'); a.SHR(8); a.ST('emE'); a.MULI(256); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.ST('x1')
    a.LDI(0); a.ST('emM')
    a.LD('x1'); a.SUBI(53); a.SKIP('GE'); a.RET()
    a.LD('x1'); a.SUBI(181); a.SKIP('GE'); a.JMP('emm')
    a.LD('emE'); a.ADDI(1); a.ST('emE'); a.RET()
    a.label('emm')
    a.LDI(1); a.ST('emM'); a.RET()

    # ---- SINQ: ACC = 32768 sin(2 pi phi / 2^24), phi in ACC (0 .. 2^24 - 1): quarter table, 14-bit interpolation
    a.label('SINQ')
    a.ST('sp'); a.SHR(22); a.ST('q'); a.LD('sp'); a.SHR(14); a.ST('a10')
    a.LD('q'); a.MULI(256); a.ST('x0'); a.LD('a10'); a.SUB('x0'); a.ST('aa')
    a.LD('a10'); a.MULI(128); a.MULI(128); a.ST('x0'); a.LD('sp'); a.SUB('x0'); a.ST('fr')
    a.LD('q'); a.SHR(1); a.ST('qh'); a.MULI(2); a.ST('x0'); a.LD('q'); a.SUB('x0')
    a.SUBI(1); a.SKIP('GE'); a.JMP('sev')
    a.LD('fr'); a.SUBI(1); a.SKIP('LT'); a.JMP('snz')
    a.LDI(256); a.SUB('aa'); a.ST('a2'); a.LDI(0); a.ST('f2'); a.JMP('slk')
    a.label('snz')
    a.LDI(255); a.SUB('aa'); a.ST('a2'); a.LD('C16384'); a.SUB('fr'); a.ST('f2'); a.JMP('slk')
    a.label('sev')
    a.LD('aa'); a.ST('a2'); a.LD('fr'); a.ST('f2')
    a.label('slk')
    a.LD('a2'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('a2'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('x0')
    a.LD('f2'); a.MUL('x0'); a.SHR(14); a.ADD('s0'); a.ST('x0')
    a.LD('qh'); a.SUBI(1); a.SKIP('GE'); a.JMP('spos')
    a.LDI(0); a.SUB('x0'); a.RET()
    a.label('spos')
    a.LD('x0'); a.RET()

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
        hold(inp, h); glide(h, acc, 4, 32, 32)
    hold('K5', 'h5'); glide('h5', 'a5', 3, 0, 8)

    # ---- mode values
    a.IN('YS'); a.ST('ys'); a.SHR(3); a.ST('ys8'); a.LD('ys'); a.SHR(2); a.ST('ys4')
    a.LD('ys'); a.MUL('ys'); a.ST('x0'); a.SHR(10); a.ST('ysk'); a.LD('x0'); a.SHR(7); a.ST('vv')
    a.IN('XSQ'); a.ST('x0'); a.MULI(2); a.ST('xs2'); a.LD('x0'); a.MUL('x0'); a.ST('xsk')
    a.IN('LPX'); a.ST('lpx')

    # ---- scale: Ls = log2 S (h), Q8 = 2184 + (K3 Q3) * 41 / 512  (S = 185 .. 1090 virtual px)
    a.LD('a3'); a.SHR(3); a.MULI(41); a.SHR(9); a.ADDI(2047); a.ADDI(137); a.ST('Ls')
    a.CALL('EXPQ4'); a.ST('S4')
    # ---- spread sigma (Q12): 0.05 .. 0.62 below 50 %, 0.62 .. sigma max(n) above
    a.LD('ap'); a.SHR(3); a.ST('gp'); a.ADDI(-2048); a.ADDI(-2048); a.ST('gpm'); a.SKIP('LT'); a.JMP('shi')
    a.LD('gp'); a.MUL('C2335'); a.SHR(12); a.ADDI(205); a.JMP('sst')
    a.label('shi')
    a.IN('NZ'); a.SUBI(1); a.SKIP('GE'); a.JMP('sl0'); a.SUBI(1); a.SKIP('GE'); a.JMP('sl1')
    a.LD('SL2'); a.JMP('slx')
    a.label('sl0'); a.LD('SL0'); a.JMP('slx')
    a.label('sl1'); a.LD('SL1')
    a.label('slx')
    a.ST('x0'); a.LD('gpm'); a.MUL('x0'); a.SHR(12); a.ADDI(2047); a.ADDI(493)
    a.label('sst')
    a.ST('sig')
    a.LD('S4'); a.MUL('sig'); a.SHR(12); a.ST('rho'); a.MUL('C4915'); a.SHR(12); a.ST('rhx')
    a.LD('rho'); a.MUL('C3277'); a.SHR(12); a.ST('rhy')
    # ---- formation point F (h Q4): x = K1 * 7.5, y = 34560 - K2 * 4.22 (K Q3)
    a.LD('a1'); a.SHR(3); a.MULI(15); a.SHR(1); a.ST('Fx')
    a.LD('a2'); a.SHR(3); a.MULI(135); a.SHR(5); a.ST('x0'); a.LD('C34560'); a.SUB('x0'); a.ST('Fy')
    # ---- Drift: |v| < 24 still; else 2^(-12 + ls / 64) turn per field, ls = (|v| - 24) * 0.906 (top 2^-5)
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(4); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(5); a.ST('x0')
    a.LD('ls'); a.SUB('x0'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ST('mx'); a.JMP('spd')
    a.label('still')
    a.LDI(0); a.ST('mx'); a.ST('kk')
    a.label('spd')
    # ---- the camera's table gain: Flat 1/16 AA px (x 3/4, >> e of the AA pixel), Rings Q11 bands
    a.LD('a6'); a.SHR(3); a.ST('g6')
    a.SKIP('RINGS'); a.JMP('gfl')
    a.MULI(525); a.SHR(11); a.JMP('gout')
    a.label('gfl')
    a.MUL('C2735'); a.SHR(10); a.ST('gx'); a.LD('lpx'); a.CALL('EM')
    a.LD('emM'); a.SUBI(1); a.SKIP('GE'); a.JMP('gsh')
    a.LD('gx'); a.SHR(2); a.ST('x0'); a.LD('gx'); a.SUB('x0'); a.ST('gx')
    a.label('gsh')
    a.LD('emE'); a.ST('x3')
    a.label('gl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('gld'); a.ST('x3'); a.LD('gx'); a.SHR(1); a.ST('gx'); a.JMP('gl')
    a.label('gld')
    a.LD('gx')
    a.label('gout')
    a.OUT('GT')
    # ---- the orbit phase offsets: 0.1 + i / n of a turn
    a.IN('NZ'); a.SUBI(1); a.SKIP('GE'); a.JMP('st0'); a.SUBI(1); a.SKIP('GE'); a.JMP('st1')
    a.LD('ST2'); a.JMP('stx')
    a.label('st0'); a.LD('ST0'); a.JMP('stx')
    a.label('st1'); a.LD('ST1')
    a.label('stx')
    a.ST('step'); a.LD('OFF0'); a.ST('off')

    # ---- per disc: a stub copies the disc's state and constants in, the shared body runs, the dispatch at its
    #      end returns to the stub, which copies the orbit angle back
    for i in range(NDISC):
        a.LDI(i); a.ST('di'); a.OUT('DI'); a.LD('th%d' % i); a.ST('thc'); a.LD('RQ%d' % i); a.ST('rqc')
        a.LDI(LK[i]); a.ST('lkc'); a.LDI(RSG[i]); a.ST('rsg'); a.JMP('body')
        a.label('ret%d' % i)
        a.LD('thc'); a.ST('th%d' % i)
    a.HALT()

    a.label('body')
    # the orbit angle: theta += -+ (mx RQ >> 12) << kk >> 3, one turn = 2^24; against Drift's sign when rsg
    a.LD('mx'); a.MUL('rqc'); a.SHR(12); a.ST('w'); a.LD('kk'); a.ST('x3')
    a.label('dl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('x3'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('w'); a.SHR(3); a.ST('w')
    a.LD('v'); a.SKIP('LT'); a.JMP('vp')
    a.LD('rsg'); a.SUBI(1); a.SKIP('GE'); a.JMP('tsub'); a.JMP('tadd')
    a.label('vp')
    a.LD('rsg'); a.SUBI(1); a.SKIP('GE'); a.JMP('tadd')
    a.label('tsub')
    a.LD('thc'); a.SUB('w'); a.JMP('twr')
    a.label('tadd')
    a.LD('thc'); a.ADD('w')
    a.label('twr')
    a.CALL('WRAP'); a.ST('thc')
    # phi = theta + offset; cos, sin
    a.ADD('off'); a.CALL('WRAP'); a.ST('ph'); a.ADD('Q22'); a.CALL('WRAP'); a.CALL('SINQ'); a.ST('cs')
    a.LD('ph'); a.CALL('SINQ'); a.ST('sn')
    # centre (q = 1/4 h): x = F + rho_x cos, y = F - rho_y sin (h Q4), >> 2
    a.LD('cs'); a.abs_('x0'); a.ST('x0'); a.LD('rhx'); a.MUL('x0'); a.SHR(15); a.ST('x0')
    a.LD('cs'); a.SKIP('LT'); a.JMP('cp'); a.LD('Fx'); a.SUB('x0'); a.JMP('cd')
    a.label('cp')
    a.LD('Fx'); a.ADD('x0')
    a.label('cd')
    a.SHR(2); a.ST('cxq')
    a.LD('sn'); a.abs_('x0'); a.ST('x0'); a.LD('rhy'); a.MUL('x0'); a.SHR(15); a.ST('x0')
    a.LD('sn'); a.SKIP('LT'); a.JMP('sp_'); a.LD('Fy'); a.ADD('x0'); a.JMP('sd')
    a.label('sp_')
    a.LD('Fy'); a.SUB('x0')
    a.label('sd')
    a.SHR(2); a.ST('cyq')
    # radius (h Q4) and R^2 (q^2; the RTL keeps it for discs 0 .. 2)
    a.LD('Ls'); a.ADD('lkc'); a.CALL('EXPQ4'); a.ST('R4'); a.MUL('R4'); a.SHR(4); a.ST('R2'); a.OUT('R2')
    # the pixel DDA's first step: 2 dx0 xs + xs^2 (q^2)
    a.IN('XQ'); a.SUB('cxq'); a.ST('dx0'); a.MUL('xs2'); a.ADD('xsk'); a.OUT('D0')
    # r^2 - R^2 at line 0's pre-pixel, Q2: 4 (dx0^2 + cy^2 - R^2) - (ys / 8) cy + floor(ys^2 / 1024)
    a.LD('cyq'); a.MUL('ys8'); a.ST('x0'); a.LD('ysk'); a.SUB('x0'); a.ST('x1')
    a.LD('dx0'); a.MUL('dx0'); a.ST('x0'); a.LD('cyq'); a.MUL('cyq'); a.ADD('x0'); a.SUB('R2'); a.ST('x0')
    a.ADD('x0'); a.ADD('x0'); a.ADD('x0'); a.ADD('x1'); a.OUT('V')
    # its step to the next line, Q2: 2 ys^2 / 256 - (ys / 4) cy
    a.LD('cyq'); a.MUL('ys4'); a.ST('x0'); a.LD('vv'); a.SUB('x0'); a.OUT('U')
    # the edge scale 2^-(9 + e) (x 3/4 if m): T = log2(2 R pxh) Q8
    a.LD('Ls'); a.ADD('lkc'); a.ADDI(256); a.ADD('lpx'); a.CALL('EM')
    a.LD('emM'); a.MULI(8); a.ST('x0'); a.LD('emE'); a.SUBI(9); a.ADD('x0'); a.OUT('EM')
    # the ring scale (discs 0 .. 2): d_r = P 2^-s mu / 64 = d (24 v px = 512), T = Lr + 406: s, mu = 64 2^-(f/256)
    a.LD('di'); a.SUBI(3); a.SKIP('LT'); a.JMP('nring')
    a.LD('Ls'); a.ADD('lkc'); a.ADDI(406); a.ST('x0'); a.SHR(8); a.ST('rs'); a.MULI(256); a.ST('x1'); a.LD('x0')
    a.SUB('x1'); a.ST('x1'); a.SUBI(1); a.SKIP('GE'); a.JMP('mu0')
    a.LDI(256); a.SUB('x1'); a.OUT('ADR'); a.IN('ROM'); a.ADDI(512); a.SHR(10); a.ST('mu'); a.SUBI(64)
    a.SKIP('GE'); a.JMP('mud')
    a.label('mu0')
    a.LDI(32); a.ST('mu'); a.LD('rs'); a.SUBI(1); a.ST('rs')
    a.label('mud')
    a.LD('mu'); a.MULI(8); a.ST('x0'); a.LD('rs'); a.SUBI(8); a.ADD('x0'); a.OUT('RS')
    # the r / R classes for the rings' anti-aliasing: d_r at the centre Dc = (R^2 >> s) mu / 64;
    # T1 = 7/8 Dc (r / R ~ 0.35), T2 = 1/2 Dc (r / R ~ 0.7)
    a.LD('rs'); a.SUBI(8); a.ST('x3'); a.LD('R2'); a.SHR(8); a.ST('x2')
    a.label('tsl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('tsd'); a.ST('x3'); a.LD('x2'); a.SHR(1); a.ST('x2'); a.JMP('tsl')
    a.label('tsd')
    a.LD('x2'); a.MUL('mu'); a.SHR(6); a.ST('x2'); a.SHR(3); a.ST('x0'); a.LD('x2'); a.SUB('x0'); a.OUT('T1')
    a.LD('x2'); a.SHR(1); a.OUT('T2')
    a.label('nring')
    # the next disc's phase offset
    a.LD('off'); a.ADD('step'); a.CALL('WRAP'); a.ST('off')
    # back to the disc's stub
    a.LD('di'); a.SUBI(1); a.SKIP('GE'); a.JMP('ret0'); a.SUBI(1); a.SKIP('GE'); a.JMP('ret1')
    a.SUBI(1); a.SKIP('GE'); a.JMP('ret2'); a.JMP('ret3')
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; eclipse.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000, check=False):
    """inp: dict over INS names (K1.. raw knobs, YS, XQ, XSQ, LPX, NZ); flags: GLIDE, RINGS.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (outs: dict name -> value or (name, disc) -> value, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; cnt = 0; adr = 0; di = 0
    outs = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    inv_o = {v: k for k, v in OUTS.items()}
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
            elif n == INS['CNT']: acc = cnt
            elif n == INS['ZERO']: acc = 0
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == OUTS['ADR']: adr = acc & 511
            elif n == OUTS['DI']: di = acc & 7
            nm = inv_o[n]
            if nm in ('V', 'U', 'D0', 'R2', 'EM', 'RS', 'T1', 'T2'):
                outs[(nm, di)] = acc
            else:
                outs[nm] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'RINGS'}.get(c, 'GLIDE')])
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
    return outs, rf, clocks


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
    parts = ["-- Generated by eclipse.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package eclipse_pkg is",
             vhd_table("C_EC_TAB", 512, 16, TAB),
             vhd_table("C_EC_RV", 256, 16, rv_rom()),
             vhd_table("C_EC_RS", 256, 16, rs_rom()),
             vhd_table("C_EC_Y", 512, 10, y_rom()),
             vhd_table("C_EC_UC", 1024, 16, uc, "std_logic_vector"),
             vhd_table("C_EC_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             "end package eclipse_pkg;", ""]
    with open(os.path.join(HERE, "eclipse_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
