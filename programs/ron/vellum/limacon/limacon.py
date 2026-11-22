#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
LIMACON v0.1.0 (Vellum) - build hook.  Writes limacon_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.vellum_work/rb_limacon/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_LM_TAB  512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)) (address 512 wraps to entry 0 = 32768 = sin(pi/2))
  C_LM_Y    1024 x 10  tone: index = ink & fine & crowd (2 bits) & summed coverage (6 bits, 16 = one disc)
                       -> luma 64 .. 939 (white lands on 775 after the output scaling)
  C_LM_UC   2048 x 16  the per-field microprogram (PROGRAM below)
  C_LM_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                       the drift state, constants)
  C_LM_BKI  256 x 32   the circle-constant banks at power-up (every circle skipped)

The per-field engine is Hardline flute's (as in Vellum eclipse): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial 18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift; micro-op =
4-bit opcode + 12-bit operand.  It runs once per field from the vblank zone start and writes the circle constants
of the NEXT field into the inactive bank (the banks swap at the next zone start).

Bank words per circle slot: 0 F00 (line 0, pixel -2), 1 KC' (line end, pass 1), 2 D0 | EM (bits 31 .. 27),
3 P2 (line end, pass 2); slot 24: dY0, A_Y, -, W | f0 << 16.

Geometry (F in units of 1/4 virtual px^2; V = a pixel-grid point next to the pole (Cardioid) or the figure
centre (Nephroid), delta = P - V):
  circle k, drawn centre V + g_k (g quantized to 2^-tx px across, 2^-sy px down), passes through P (Cardioid) or
  has radius r_k = kappa R |sin(t_k - psi)| (Nephroid):
  F_k(m, j) = xs^2 (m - mV)^2 + floor(4 hy^2 (j - jV)^2) - lam_k (m - mV) - mu_k (j - jV) + G_k
    lam = 8 hx gx, mu = 8 hy gy (integers), G = 8 g.delta - 4|delta|^2 (Cardioid) or 4|g|^2 - 4 r^2 (Nephroid)
  The hardware steps it per pixel (one shared second difference + the camera) and corrects it once per line.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    return e + s


LV = [0, 1 / 3.0, 2 / 3.0, 1.0]


def tone_level(ink, fine, s6):
    if ink:
        m = s6 & 31
        return (m if m <= 16 else 32 - m) / 16.0
    if fine:
        return min(1.0, s6 * 0.6 / 16.0)
    n0, fr = (s6 >> 4) & 3, s6 & 15
    return (LV[n0] * (16 - fr) + LV[(n0 + 1) & 3] * fr) / 16.0


def y_rom():
    out = []
    for ink in (0, 1):
        for fine in (0, 1):
            for w in range(4):
                for s6 in range(64):
                    lv = tone_level(ink, fine, s6)
                    lv = (lv * (3 - w) + 0.5 * w) / 3.0
                    out.append(64 + int(round(lv * 875)))
    return out


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k5=650, k6=1023, p12=614)
NC = 24                                   # circle slots
PSI0 = 12582912                           # the pole angle at power-up: -1/4 turn (Q24): straight up, the heart
STEPN = [2097152, 1398101, 1048576, 699051]   # one turn / N, Q24 (N = 8, 12, 16, 24)
INVN = [4096, 2731, 2048, 1365]               # 1 / N, Q15
MC_OFF = 3                                # the screen centre shows pixel W/2 - MC_OFF (generated avid + filter)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XS=9, W=10, ROM=11, NZ=13, ZERO=15)
CONDS = dict(LT=0, GLIDE=1, NEPH=2, GE=8, NGLIDE=9, NNEPH=10)
OUTS = dict(ADR=0, CI=1, GT=2, EMR=3, B0=4, B1=5, B2=6, B3=7)


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

    def const(self, v):
        return self.reg('C%d' % v if v >= 0 else 'Cm%d' % -v, v)

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
            def f(r):
                if isinstance(r, int):
                    r = 'C%d' % r if r >= 0 else 'Cm%d' % -r
                    self.reg(r, int(r[1:]) if r[1] != 'm' else -int(r[2:]))
                self.emit(op, self.reg(r))
            return f
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

    def ldk(self, v):
        """ACC = v (any 32-bit constant)."""
        if -2048 <= v <= 2047:
            self.LDI(v)
        else:
            self.LD(v)

    def addk(self, v):
        if -2048 <= v <= 2047:
            self.ADDI(v)
        else:
            self.ADD(v)

    def mulk(self, v):
        """ACC = ACC(17:0) * v."""
        if -2048 <= v <= 2047:
            self.MULI(v)
        else:
            self.MUL(v)



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
    a.reg('al', 0)
    a.reg('ps', PSI0)
    for n, v in (('T24', 1 << 24), ('Q22', 1 << 22), ('C16384', 16384), ('C3048', 3048)):
        a.reg(n, v)
    for i in range(4):
        a.reg('SN%d' % i, STEPN[i]); a.reg('IN%d' % i, INVN[i])

    a.JMP('main')

    # ---- WRAP: ACC mod 2^24 (one turn), for ACC in [-2^24, 2^25)
    a.label('WRAP')
    a.ST('t'); a.SKIP('LT'); a.JMP('wrhi'); a.ADD('T24'); a.RET()
    a.label('wrhi')
    a.SUB('T24'); a.SKIP('GE'); a.ADD('T24'); a.RET()

    # ---- EXP1: ACC = 2^(L / 256) for L (in ACC) in [9 * 256, 15 * 256): EXP[L & 255] >> (15 - (L >> 8))
    a.label('EXP1')
    a.ST('x0'); a.SHR(8); a.ST('x1'); a.MULI(256); a.ST('x2'); a.LD('x0'); a.SUB('x2'); a.OUT('ADR'); a.IN('ROM')
    a.ST('x2'); a.LDI(15); a.SUB('x1'); a.ST('x3')
    a.label('exl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('exd'); a.ST('x3'); a.LD('x2'); a.SHR(1); a.ST('x2'); a.JMP('exl')
    a.label('exd')
    a.LD('x2'); a.RET()

    # ---- SINQ: ACC = 32768 sin(2 pi phi / 2^24), phi in ACC (0 .. 2^24 - 1): quarter table, 14-bit interpolation
    a.label('SINQ')
    a.ST('ssp'); a.SHR(22); a.ST('sq'); a.LD('ssp'); a.SHR(14); a.ST('sa10')
    a.LD('sq'); a.MULI(256); a.ST('x0'); a.LD('sa10'); a.SUB('x0'); a.ST('saa')
    a.LD('sa10'); a.MULI(128); a.MULI(128); a.ST('x0'); a.LD('ssp'); a.SUB('x0'); a.ST('sfr')
    a.LD('sq'); a.SHR(1); a.ST('sqh'); a.MULI(2); a.ST('x0'); a.LD('sq'); a.SUB('x0')
    a.SUBI(1); a.SKIP('GE'); a.JMP('sev')
    a.LD('sfr'); a.SUBI(1); a.SKIP('LT'); a.JMP('snz')
    a.LDI(256); a.SUB('saa'); a.ST('sa2'); a.LDI(0); a.ST('sf2'); a.JMP('slk')
    a.label('snz')
    a.LDI(255); a.SUB('saa'); a.ST('sa2'); a.LD('C16384'); a.SUB('sfr'); a.ST('sf2'); a.JMP('slk')
    a.label('sev')
    a.LD('saa'); a.ST('sa2'); a.LD('sfr'); a.ST('sf2')
    a.label('slk')
    a.LD('sa2'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('ss0')
    a.LD('sa2'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('ss0'); a.ST('x0')
    a.LD('sf2'); a.MUL('x0'); a.SHR(14); a.ADD('ss0'); a.ST('x0')
    a.LD('sqh'); a.SUBI(1); a.SKIP('GE'); a.JMP('spos')
    a.LDI(0); a.SUB('x0'); a.RET()
    a.label('spos')
    a.LD('x0'); a.RET()

    # ---- HG: hold (kin -> hc, a knob has to move 2 counts) and glide (hc -> ac, Q6, 1/16 of the gap per field,
    #      never under half a count, landing exactly; Glide Off: the held value)
    a.label('HG')
    hu, hk = a.new(), a.new()
    a.LD('kin'); a.SUB('hc'); a.ADDI(1); a.SKIP('GE'); a.JMP(hu); a.SUBI(3); a.SKIP('GE'); a.JMP(hk)
    a.label(hu); a.LD('kin'); a.ST('hc')
    a.label(hk)
    gsn, gad = a.new(), a.new()
    a.LD('hc'); a.MULI(64); a.ST('tg'); a.SKIP('GLIDE'); a.JMP(gsn)
    a.SUB('ac'); a.ST('gd'); a.abs_('x0'); a.SUBI(33); a.SKIP('GE'); a.JMP(gsn)
    a.LD('gd'); a.SHR(4); a.ST('gs')
    a.abs_('x0'); a.SUBI(32); a.SKIP('LT'); a.JMP(gad)
    a.LDI(32); a.ST('gs'); a.LD('gd'); a.SKIP('LT'); a.JMP(gad); a.LDI(-32); a.ST('gs')
    a.label(gad)
    a.LD('ac'); a.ADD('gs'); a.ST('ac'); a.RET()
    a.label(gsn)
    a.LD('tg'); a.ST('ac'); a.RET()

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
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hc'); a.LD(acc); a.ST('ac'); a.CALL('HG')
        a.LD('hc'); a.ST(h); a.LD('ac'); a.ST(acc)
    hold('K5', 'h5'); glide('h5', 'a5', 3, 0, 8)

    # ---- mode values.  xs = x step in half virtual px (2 / 3 / 4); ys = Q8 virtual lines per field line
    def setr(pairs):
        for r, v in pairs:
            a.ldk(v); a.ST(r)

    a.IN('XS'); a.ST('xs'); a.MUL('xs'); a.ST('xs2')
    a.LD('xs'); a.MULI(8); a.ST('xs8')
    l3, l4, ld = a.new(), a.new(), a.new()
    a.LD('xs'); a.SUBI(3); a.SKIP('LT'); a.JMP(l3)
    setr((('mx', 8), ('gxm', 2), ('kx', 1), ('rx', 4096))); a.JMP(ld)
    a.label(l3)
    a.LD('xs'); a.SUBI(4); a.SKIP('LT'); a.JMP(l4)
    setr((('mx', 4), ('gxm', 4), ('kx', 3), ('rx', 2731))); a.JMP(ld)
    a.label(l4)
    setr((('mx', 16), ('gxm', 1), ('kx', 1), ('rx', 2048)))
    a.label(ld)
    a.IN('YS'); a.ST('ys'); a.SHR(1); a.ST('ys2'); a.LD('ys'); a.SHR(4); a.ST('hy16')
    a.LD('ys'); a.SHR(5); a.addk(-8640); a.ST('y0')
    # sy = the largest s <= 3 with 32 * 2^s | ys; ky = ys / 2^(5 + sy); my = 2^sy; gym = 16 / my
    a.LD('ys'); a.SHR(5); a.ST('ky'); a.LDI(1); a.ST('my'); a.LDI(16); a.ST('gym')
    a.label('syl')
    a.LD('my'); a.SUBI(8); a.SKIP('LT'); a.JMP('syd')
    a.LD('ky'); a.SHR(1); a.ST('x0'); a.MULI(2); a.ST('x1'); a.LD('ky'); a.SUB('x1'); a.SUBI(1); a.SKIP('LT')
    a.JMP('syd')
    a.LD('x0'); a.ST('ky'); a.LD('my'); a.ADD('my'); a.ST('my'); a.LD('gym'); a.SHR(1); a.ST('gym'); a.JMP('syl')
    a.label('syd')
    # rhy = floor(65536 / hy16) by restoring division (hy16 = 16 .. 72)
    a.LD('hy16'); a.MUL(4096); a.ST('x0'); a.LD(4096); a.ST('x1'); a.LD(65536); a.ST('x2'); a.LDI(0); a.ST('rhy')
    a.label('dvl')
    a.LD('x2'); a.SUB('x0'); a.SKIP('GE'); a.JMP('dvn'); a.ST('x2'); a.LD('rhy'); a.ADD('x1'); a.ST('rhy')
    a.label('dvn')
    a.LD('x0'); a.SHR(1); a.ST('x0'); a.LD('x1'); a.SHR(1); a.ST('x1'); a.SUBI(1); a.SKIP('LT'); a.JMP('dvl')
    a.IN('W'); a.ST('W'); a.SHR(1); a.SUBI(MC_OFF); a.ST('mc')
    a.IN('NZ'); a.ST('nz')
    z1, z2, z3, zd = a.new(), a.new(), a.new(), a.new()
    a.LD('nz'); a.SUBI(1); a.SKIP('GE'); a.JMP(z1)
    a.SUBI(1); a.SKIP('GE'); a.JMP(z2)
    a.SUBI(1); a.SKIP('GE'); a.JMP(z3)
    setr((('N', 24),)); a.LD('SN3'); a.ST('stp'); a.LD('IN3'); a.ST('inv'); a.JMP(zd)
    a.label(z1); setr((('N', 8),)); a.LD('SN0'); a.ST('stp'); a.LD('IN0'); a.ST('inv'); a.JMP(zd)
    a.label(z2); setr((('N', 12),)); a.LD('SN1'); a.ST('stp'); a.LD('IN1'); a.ST('inv'); a.JMP(zd)
    a.label(z3); setr((('N', 16),)); a.LD('SN2'); a.ST('stp'); a.LD('IN2'); a.ST('inv')
    a.label(zd)

    # ---- scale: Lq0 = 256 log2(R0 in Q4) = 3048 + (K3 - 512) * 0.64  (R0 = 99 .. 581 virtual px, 240 at 50 %)
    a.LD('a3'); a.SHR(3); a.addk(-4096); a.MULI(41); a.SHR(9); a.ADD('C3048'); a.ST('Lq0')
    a.CALL('EXP1'); a.ST('R0')
    # ---- the camera's table gain: Gt = K6 * R0 * 0.748 / 1024 (white at 100 %: 4480 R0 in F units)
    a.LD('a6'); a.SHR(6); a.MUL('R0'); a.SHR(10); a.MULI(766); a.SHR(10); a.OUT('GT')

    # ---- P12 (the glided value, with an 8-count detent on the cardioid at 614)
    a.LD('ap'); a.SHR(6); a.ST('pc'); a.SUBI(614); a.ST('x0'); a.abs_('x1'); a.SUBI(9); a.SKIP('LT'); a.JMP('pnd')
    a.LDI(614); a.ST('pc')
    a.label('pnd')
    a.SKIP('NEPH'); a.JMP('card')
    # Nephroid: kappa (Q12) 0.5 .. 1.0 (60 %) .. 1.8; size log2 (Q8) 241 .. 155 .. 16
    a.LD('pc'); a.SUBI(615); a.SKIP('LT'); a.JMP('nhi')
    a.LD('pc'); a.mulk(3415); a.SHR(10); a.addk(2048); a.ST('kap')
    a.LD('pc'); a.MULI(143); a.SHR(10); a.ST('x0'); a.LDI(241); a.SUB('x0'); a.ST('siz'); a.JMP('pdone')
    a.label('nhi')
    a.LD('pc'); a.SUBI(614); a.ST('x2'); a.mulk(8205); a.SHR(10); a.addk(4096); a.ST('kap')
    a.LD('x2'); a.MULI(348); a.SHR(10); a.ST('x0'); a.LDI(155); a.SUB('x0'); a.ST('siz'); a.JMP('pdone')
    # Cardioid: pole p (Q12) 0.15 .. 1.0 (60 %) .. 2.2; centring c(p); size log2 (Q8) 44 .. 0 .. -143
    a.label('card')
    a.LD('pc'); a.SUBI(615); a.SKIP('LT'); a.JMP('chi')
    a.LD('pc'); a.mulk(5807); a.SHR(10); a.ADDI(614); a.ST('pp'); a.JMP('cc')
    a.label('chi')
    a.LD('pc'); a.SUBI(614); a.mulk(12298); a.SHR(10); a.addk(4096); a.ST('pp')
    a.label('cc')
    c1, c2, cd = a.new(), a.new(), a.new()
    a.LD('pp'); a.addk(-2049); a.SKIP('GE'); a.JMP(c1)
    a.LD('pp'); a.addk(-4097); a.SKIP('GE'); a.JMP(c2)
    a.LD('pp'); a.addk(-4096); a.MULI(463); a.SHR(12); a.addk(3072); a.JMP(cd)
    a.label(c1); a.LD('pp'); a.JMP(cd)
    a.label(c2); a.LD('pp'); a.SHR(1); a.addk(1024)
    a.label(cd)
    a.ADD('pp'); a.ST('cp')
    s1, s2, sd = a.new(), a.new(), a.new()
    a.LD('pc'); a.SUBI(289); a.SKIP('GE'); a.JMP(s1)
    a.LD('pc'); a.SUBI(615); a.SKIP('GE'); a.JMP(s2)
    a.LD('pc'); a.SUBI(614); a.MULI(358); a.SHR(10); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.JMP(sd)
    a.label(s1); a.LDI(44); a.JMP(sd)
    a.label(s2); a.LDI(614); a.SUB('pc'); a.MULI(138); a.SHR(10)
    a.label(sd)
    a.ST('siz')
    a.label('pdone')
    a.LD('Lq0'); a.ADD('siz'); a.CALL('EXP1'); a.ST('R')

    # ---- Drift: |v| < 24 still; else 2^(-9 + ls / 64) disc steps per field, ls = (|v| - 24) * 0.906 (top 1/4);
    #      the discs roll (al) and the pole orbits the other way at a quarter of the rate (ps); one turn = 2^24
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(4); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(5); a.ST('x0')
    a.LD('ls'); a.SUB('x0'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.MUL('inv'); a.SHR(15); a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('w'); a.SHR(2); a.ST('w4')
    a.LD('v'); a.SKIP('LT'); a.JMP('vpos')
    a.LD('al'); a.SUB('w'); a.CALL('WRAP'); a.ST('al'); a.LD('ps'); a.ADD('w4'); a.CALL('WRAP'); a.ST('ps')
    a.JMP('still')
    a.label('vpos')
    a.LD('al'); a.ADD('w'); a.CALL('WRAP'); a.ST('al'); a.LD('ps'); a.SUB('w4'); a.CALL('WRAP'); a.ST('ps')
    a.label('still')
    # ---- the pole direction e(psi)
    a.LD('ps'); a.ADD('Q22'); a.CALL('WRAP'); a.CALL('SINQ'); a.ST('cps')
    a.LD('ps'); a.CALL('SINQ'); a.ST('sps')
    # ---- figure centre Z (Q4 virtual px): x = (K1 - 512) * 30, y = (K2 - 512) * 16.875
    a.LD('a1'); a.SHR(3); a.addk(-4096); a.MULI(15); a.SHR(2); a.ST('Zx')
    a.LD('a2'); a.SHR(3); a.addk(-4096); a.MULI(135); a.SHR(6); a.ST('Zy')
    # ---- the target point T (pole P or centre Z) and its grid neighbour V
    a.SKIP('NEPH'); a.JMP('tcard')
    a.LD('Zx'); a.ST('Tx'); a.LD('Zy'); a.ST('Ty')
    a.LD('kap'); a.MUL('R'); a.SHR(12); a.ST('kR')
    a.JMP('tdone')
    a.label('tcard')
    a.LD('cp'); a.MUL('R'); a.SHR(12); a.ST('x2')
    a.MUL('cps'); a.SHR(15); a.ADD('Zx'); a.ST('Tx')
    a.LD('x2'); a.MUL('sps'); a.SHR(15); a.ADD('Zy'); a.ST('Ty')
    a.LD('pp'); a.MUL('R'); a.SHR(12); a.ST('x2')
    a.MUL('cps'); a.SHR(15); a.ST('pRc')
    a.LD('x2'); a.MUL('sps'); a.SHR(15); a.ST('pRs')
    a.label('tdone')
    a.LD('Tx'); a.MUL('rx'); a.SHR(16); a.ST('x0'); a.ADD('mc'); a.ST('mV')
    a.LD('x0'); a.MUL('xs8'); a.ST('x0'); a.LD('Tx'); a.SUB('x0'); a.ST('dx')
    a.LD('Ty'); a.SUB('y0'); a.ST('x1'); a.MUL('rhy'); a.SHR(16); a.ST('jV')
    a.MUL('hy16'); a.ST('x0'); a.LD('x1'); a.SUB('x0'); a.ST('dy')
    a.LD('dx'); a.MUL('dx'); a.ST('x0'); a.LD('dy'); a.MUL('dy'); a.ADD('x0'); a.ST('dd')

    # ---- shared constants
    # S00 = xs^2 (2 + mV)^2 + floor(4 hy^2 jV^2);  D0x = -xs^2 (3 + 2 mV);  KCx = -(W + 4) D0x - (W + 4)(W + 3) xs^2
    a.LD('mV'); a.ADDI(2); a.ST('mV2'); a.MUL('xs'); a.ST('x0'); a.MUL('x0'); a.ST('S00')
    a.LD('mV'); a.ADD('mV'); a.ADDI(3); a.MUL('xs2'); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.ST('D0x')
    a.LD('W'); a.ADDI(4); a.ST('W4'); a.MUL('D0x'); a.ST('x0')
    a.LD('W4'); a.MUL('xs2'); a.ST('x1'); a.LD('W'); a.ADDI(3); a.MUL('x1'); a.ADD('x0'); a.ST('x0')
    a.LDI(0); a.SUB('x0'); a.ST('KCx')
    # the line end's two passes: D(W + 2) = D0 + 2 xs^2 (W + 4) gets KC' + Ysh (-> KC + Ysh), F += D, then
    # P2 - Ysh (-> D0):  KC' = A1 + lam (W + 5) - mu,  P2 = A2 - lam (W + 5) + mu
    a.LD('W'); a.ADDI(5); a.ST('W5')
    a.LD('x1'); a.ADD('x1'); a.ST('x0'); a.LD('KCx'); a.SUB('D0x'); a.SUB('x0'); a.ST('A1')
    a.LD('D0x'); a.SUB('KCx'); a.ST('A2')
    # floor(4 hy^2 jV^2) and its Q12 fraction: v = (ys / 2) jV, Yq Q12 = v^2 (v = vh 2^11 + vl)
    a.LD('ys2'); a.MUL('jV'); a.ST('x0'); a.SHR(11); a.ST('vh'); a.MUL(2048); a.ST('x1'); a.LD('x0'); a.SUB('x1')
    a.ST('vl'); a.MUL('vl'); a.ST('x2'); a.SHR(12); a.ST('x3'); a.MUL(4096); a.ST('x1'); a.LD('x2'); a.SUB('x1')
    a.ST('f0')
    a.LD('vh'); a.MUL('vh'); a.MULI(1024); a.ST('x1'); a.LD('vh'); a.MUL('vl'); a.ADD('x1'); a.ADD('x3')
    a.ADD('S00'); a.ST('S00')
    # dY0 (Q12) = (ys / 2)^2 (1 - 2 jV): u = (ys / 2)(1 - 2 jV) = uh 2^11 + ul;  A_Y = 2 (ys / 2)^2
    a.LDI(1); a.SUB('jV'); a.SUB('jV'); a.MUL('ys2'); a.ST('x0'); a.SHR(11); a.ST('x1'); a.MUL(2048); a.ST('x2')
    a.LD('x0'); a.SUB('x2'); a.MUL('ys2'); a.ST('x2'); a.LD('x1'); a.MUL('ys2'); a.dbl('x3', 11); a.ADD('x2')
    a.ST('dY0')
    a.LDI(24); a.OUT('CI'); a.LD('dY0'); a.OUT('B0')
    a.LD('ys2'); a.MUL('ys2'); a.ST('x0'); a.ADD('x0'); a.OUT('B1')
    a.LD('f0'); a.MUL(65536); a.ADD('W'); a.OUT('B3')

    # ---- per circle: centre angle t_k = al + k / N turn
    a.LD('al'); a.ST('tk'); a.LDI(0); a.ST('k')
    a.label('cl')
    a.LD('k'); a.OUT('CI'); a.SUB('N'); a.SKIP('LT'); a.JMP('cskip')
    a.LD('tk'); a.ADD('Q22'); a.CALL('WRAP'); a.CALL('SINQ'); a.ST('cs')
    a.LD('tk'); a.CALL('SINQ'); a.ST('sn')
    a.LD('cs'); a.MUL('R'); a.SHR(15); a.ADD('dx'); a.ST('ux')
    a.LD('sn'); a.MUL('R'); a.SHR(15); a.ADD('dy'); a.ST('uy')
    a.SKIP('NEPH'); a.JMP('ucard')
    a.JMP('uq')
    a.label('ucard')
    a.LD('ux'); a.SUB('pRc'); a.ST('ux'); a.LD('uy'); a.SUB('pRs'); a.ST('uy')
    a.label('uq')
    # quantize: a = round(ux 2^tx / 16), b = round(uy 2^sy / 16); g (Q4) = a 16 / 2^tx; lam = a kx; mu = b ky
    a.LD('ux'); a.MUL('mx'); a.ADDI(8); a.SHR(4); a.ST('qa'); a.MUL('gxm'); a.ST('gx')
    a.LD('qa'); a.MUL('kx'); a.ST('lam')
    a.LD('uy'); a.MUL('my'); a.ADDI(8); a.SHR(4); a.ST('qb'); a.MUL('gym'); a.ST('gy')
    a.LD('qb'); a.MUL('ky'); a.ST('mu')
    a.SKIP('NEPH'); a.JMP('gcard')
    # Nephroid: r = kappa R |sin(t - psi)|;  G = (gx^2 + gy^2 - r^2) / 64;  r^2 (Q8)
    a.LD('cps'); a.MUL('sn'); a.ST('x0'); a.LD('sps'); a.MUL('cs'); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.SHR(15)
    a.MUL('kR'); a.SHR(15); a.abs_('x0'); a.ST('x0'); a.MUL('x0'); a.ST('r2')
    a.LD('gx'); a.MUL('gx'); a.ST('x0'); a.LD('gy'); a.MUL('gy'); a.ADD('x0'); a.SUB('r2'); a.ADDI(32); a.SHR(6)
    a.ST('G'); a.JMP('aa')
    # Cardioid: G = (2 (gx dx + gy dy) - dd) / 64;  r^2 (Q8) = |g - delta|^2
    a.label('gcard')
    a.LD('gx'); a.MUL('dx'); a.ST('x0'); a.LD('gy'); a.MUL('dy'); a.ADD('x0'); a.ST('x0'); a.ADD('x0'); a.SUB('dd')
    a.ADDI(32); a.SHR(6); a.ST('G')
    a.LD('gx'); a.SUB('dx'); a.ST('x0'); a.MUL('x0'); a.ST('x1'); a.LD('gy'); a.SUB('dy'); a.ST('x0'); a.MUL('x0')
    a.ADD('x1'); a.ST('r2')
    # ---- the edge scale: T = r^2 hAA^2 / 4 (px^2) = r2 xs^2 / 4096; L = floor(log2 T);
    #      L even (2a): >> a, x 3/4; L odd: >> (a + 1); the window shift is 5 + e, e clamped to 0 .. 3
    a.label('aa')
    a.LD('r2'); a.SHR(12); a.MUL('xs2'); a.ST('x0'); a.LDI(-1); a.ST('L')
    a.label('lzl')
    a.LD('x0'); a.SUBI(1); a.SKIP('GE'); a.JMP('lzd'); a.LD('L'); a.ADDI(1); a.ST('L'); a.LD('x0'); a.SHR(1)
    a.ST('x0'); a.JMP('lzl')
    a.label('lzd')
    a.LD('L'); a.SHR(1); a.ST('x1'); a.MULI(2); a.ST('x0'); a.LD('L'); a.SUB('x0'); a.ST('x2')   # x1 = a, x2 = parity
    a.SUBI(1); a.SKIP('LT'); a.JMP('lodd')
    a.LD('x1'); a.SUBI(5); a.ST('ee'); a.LDI(1); a.ST('mm'); a.JMP('lcl')
    a.label('lodd')
    a.LD('x1'); a.SUBI(4); a.ST('ee'); a.LDI(0); a.ST('mm')
    a.label('lcl')
    lc1, lc2 = a.new(), a.new()
    a.LD('ee'); a.SKIP('LT'); a.JMP(lc1); a.LDI(0); a.ST('ee'); a.ST('mm')
    a.label(lc1)
    a.LD('ee'); a.SUBI(4); a.SKIP('GE'); a.JMP(lc2); a.LDI(3); a.ST('ee'); a.LDI(0); a.ST('mm')
    a.label(lc2)
    # skip a circle of radius under 4 px (r2 < 4096)
    sk1, skd = a.new(), a.new()
    a.LD('r2'); a.addk(-4096); a.SKIP('LT'); a.JMP(sk1); a.LDI(1); a.JMP(skd)
    a.label(sk1); a.LDI(0)
    a.label(skd)
    a.ST('x0'); a.LD('mm'); a.ADD('mm'); a.ADD('x0'); a.ST('x0'); a.LD('ee'); a.MULI(4); a.ADD('x0'); a.OUT('EMR')
    # F00 = S00 + lam mV2 + mu jV + G + 2^(10 + e)   (the offset puts the coverage window at F~ = 0 .. 2^(11 + e))
    a.LD(1024); a.ST('x1'); a.LD('ee'); a.ST('x3')
    a.label('ofl')
    a.LD('x3'); a.SUBI(1); a.SKIP('GE'); a.JMP('ofd'); a.ST('x3'); a.LD('x1'); a.ADD('x1'); a.ST('x1'); a.JMP('ofl')
    a.label('ofd')
    a.LD('lam'); a.MUL('mV2'); a.ST('x0'); a.LD('mu'); a.MUL('jV'); a.ADD('x0'); a.ADD('G'); a.ADD('x1'); a.ADD('S00')
    a.OUT('B0')
    # KC' = A1 + lam (W + 5) - mu;  P2 = A2 - lam (W + 5) + mu;  D0 = D0x - lam
    a.LD('lam'); a.MUL('W5'); a.ST('x5'); a.ADD('A1'); a.SUB('mu'); a.OUT('B1')
    a.LD('A2'); a.SUB('x5'); a.ADD('mu'); a.OUT('B3')
    a.LD('D0x'); a.SUB('lam'); a.OUT('B2')
    a.JMP('cnext')
    a.label('cskip')
    a.LDI(0); a.OUT('B0'); a.OUT('B1'); a.OUT('B3'); a.LDI(1); a.OUT('EMR'); a.LDI(0); a.OUT('B2')
    a.label('cnext')
    a.LD('tk'); a.ADD('stp'); a.CALL('WRAP'); a.ST('tk')
    a.LD('k'); a.ADDI(1); a.ST('k'); a.SUBI(NC); a.SKIP('GE'); a.JMP('cl')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; limacon.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=400000, check=False):
    """inp: dict over INS names (K1.. raw knobs, YS, XS, W, NZ); flags: GLIDE, NEPH.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (bank: dict (ci, sel) -> 32-bit word, outs: dict, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ci = 0; em = 0
    bank = {}
    outs = {}
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
            elif n in (12, 14, 15): acc = 0
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 15
            if n == OUTS['ADR']: adr = acc & 511
            elif n == OUTS['CI']: ci = acc & 31
            elif n == OUTS['GT']: outs['GT'] = acc & 0x3FFF
            elif n == OUTS['EMR']: em = acc & 31
            elif 4 <= n <= 7:
                sel = n - 4
                if sel == 2:
                    word = (em << 27) | (acc & 0x7FFFFFF)
                else:
                    word = acc & 0xFFFFFFFF
                bank[(ci, sel)] = word
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'NEPH'}.get(c, 'GLIDE')])
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 2047
        elif op == 14:
            lr = pc + 1; npc = arg & 2047
        else:
            if arg == 0: npc = lr
            elif arg == 1: break
        pc = npc
    else:
        raise RuntimeError("microprogram did not halt")
    return bank, outs, rf, clocks


def bank_init():
    """256 x 32: two banks of (32 circle slots x 4 words); every circle skipped (EM skip bit), W = 0."""
    out = [0] * 256
    for b in range(2):
        for c in range(32):
            out[b * 128 + c * 4 + 2] = 1 << 27
    return out


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
    assert len(UCODE) <= 2048, len(UCODE)
    uc = UCODE + [0xF001] * (2048 - len(UCODE))
    parts = ["-- Generated by limacon.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package limacon_pkg is",
             vhd_table("C_LM_TAB", 512, 16, TAB),
             vhd_table("C_LM_Y", 1024, 10, y_rom()),
             vhd_table("C_LM_UC", 2048, 16, uc, "std_logic_vector"),
             vhd_table("C_LM_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             vhd_table("C_LM_BKI", 256, 32, bank_init(), "std_logic_vector"),
             "end package limacon_pkg;", ""]
    with open(os.path.join(HERE, "limacon_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
