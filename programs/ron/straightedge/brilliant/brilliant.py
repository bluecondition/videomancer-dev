#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
BRILLIANT v0.1.0 (Straightedge) - build hook.  Writes brilliant_pkg.vhd next to this file (pure python: the
builder runs it with the system python3).  The integer model (.straightedge_work/rb_brilliant/imodel.py) imports
this file and runs the same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_BR_TAB  1024 x 16  the engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)); 512 = 32768; 513 .. the cut: t, s, kappa2, kappa3 at 65
                       points of P12 (Q15, Q15, Q9, Q9); then alpha(|kappa|) = 1 / sqrt(1 + kappa^2), 64 points
  C_BR_UC   2048 x 16  the per-field microprogram (PROGRAM below); CALL / RET use a four-deep link stack
  C_BR_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults)

The per-field engine is Hardline flute's (as in windrose / lowpoly): a 37-bit accumulator, a 256 x 32-bit register
file, one adder, a serial 18 x 18 signed multiplier and a serial right shift; micro-op = 4-bit opcode + 12-bit
operand.  It is started at every vblank zone and writes one bank of a two-bank table while the other bank is drawn.
Brilliant adds a grid port: OUT GA starts a read of the camera grid (the bank the pixel path reads), SKIP GRDY
waits for it, IN GD returns the byte (the per-facet camera of Form = Stone and the picture's mean).

Table words (one bank = 256 x 32 bits; all distances Q12 / Q3 of the anti-aliasing unit U = 2^ush virtual px):
  0 .. 4    DDA X: pixel step, step at a cell wrap, line wrap, line wrap at a cell-row wrap, seed (Q12, 24 bits)
            (Stone: the great stone's U, the cell wraps doing nothing; Pave: the lattice's u)
  8 .. 12   DDA Y: the same (V / v)
  24        lattice x: pixels left in the first cell at DDA index 0 (12) | cell width - 1 (12)
  25        lattice y: lines left in the first cell row at line 0 (12) | cell height - 1 (12)
  26        first cell column (8) | first cell row (8)
  27        camera block x of the first cell centre (Q6.10, 16) | per cell step (Q6.10, 16)   (32 vpx blocks)
  28        camera block y of the first cell row (Q6.10, 16) | per cell row step (16)
  29        gamma1 = t R (16) | gamma4 = R (16)              (Q3)
  30        gamma2 (16, signed) | gamma3 (16, signed)        (Q3)
  31        kappa2 (16, signed) | kappa3 (16, signed)        (Q9)
  32        alpha2 (4: x/8) | alpha3 (4) | Fine half width (6, Q3) | glitter dot step (8, Q3) | dot on |
            Pave | Ink | Fine (the pixel path's switches) | bit 8 of T1 .. T4 (29 .. 26)
  48 .. 63  the sector words: sector j's tones (kite, star, girdle, girdle j^1, ray, ray j^1, ray j0; 2 bits each)
  38        the dots' twinkle phase (4 bits)
  39        ramp: the 6-bit luma of tone 0 .. 3 (4 x 8 bits); dot luma (8) | dot twinkle luma (8) in word 40
  40        dot lumas (2 x 8)
  41        size thresholds T1 .. T4 (low 8 bits each; the camera grid holds the raw luma)
  42        3 kappa2 | 3 kappa3 (16 bits each, signed)
  43        glitter dy at line 0 | at a cell row's first line (16 bits each, signed, 1/512 virtual px)
  44        glitter dx at DDA index 0 | at a cell's first pixel (16 bits each, signed, 1/8 virtual px)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

C22 = math.cos(math.pi / 8)
S22 = math.sin(math.pi / 8)
XOFS = 4                       # screen column c = the output filter centred on DDA index c + XOFS
NEXTRA = 6                     # DDA steps per line = width + NEXTRA
GLITTER_C4 = 32 * 16           # the glitter lattice's cell (Stone + camera), Q4 virtual px
CAM_BLOCK = 32                 # camera grid block, virtual px (34 rows x 60 columns; address row 60 + col)

# the cut: P12 0 .. 1 -> table radius t, star-point radius s (fractions of the girdle radius)
CUT_PTS = [(0.0, 0.80, 0.885), (0.5, 0.55, 0.75), (1.0, 0.16, 0.42)]


def cut_ts(p):
    if p <= 0.5:
        f = p / 0.5; a, b = CUT_PTS[0], CUT_PTS[1]
    else:
        f = (p - 0.5) / 0.5; a, b = CUT_PTS[1], CUT_PTS[2]
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])


def cut_kappa(t, s):
    k2 = (t - s * C22) / (s * S22)          # star / kite edge through T0 and S0, a-form
    k3 = (1.0 - s * C22) / (s * S22)        # kite / girdle edge through G0 and S0
    return k2, k3


ROM_T, ROM_S, ROM_K2, ROM_K3, ROM_AL = 513, 578, 643, 708, 773


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    rom = e + s + [32768]
    T, S, K2, K3 = [], [], [], []
    for i in range(65):
        t, sp = cut_ts(i / 64.0)
        k2, k3 = cut_kappa(t, sp)
        T.append(int(round(t * 32768))); S.append(int(round(sp * 32768)))
        K2.append(int(round(k2 * 512)) & 0xFFFF); K3.append(int(round(k3 * 512)) & 0xFFFF)
    rom += T + S + K2 + K3
    assert len(rom) == ROM_AL
    rom += [int(round(32768 / math.sqrt(1 + (i * 64 / 1024.0) ** 2))) for i in range(64)]
    rom += [0] * (1024 - len(rom))
    return rom


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k4=450, k5=680, k6=1023, p12=512)
Q26 = 1 << 26                  # one full turn of the light phase
Q24 = 1 << 24                  # a quarter turn

# facets: base tone (even sector, odd sector) and the anchor (radius as a function of t, s; wedge angle)
TYPES = ['kite', 'star', 'girdle', 'ray']
BASE = dict(kite=(0, 0), star=(2, 2), girdle=(1, 2), ray=(0, 1))
# facet azimuth per angular sector j, in units of 1/32 turn (11.25 degrees) plus a half turn for the pavilion
AZ = dict(kite=lambda j: 4 * ((j + 1) >> 1), star=lambda j: 2 + 4 * (j >> 1), girdle=lambda j: 1 + 2 * j,
          ray=lambda j: 1 + 2 * j + 16, tri=lambda j: 2 + 4 * (j >> 1) + 16)
# Fire: the flash thresholds (cos of the angle to the light) and the tone ramp (Q15 of white), soft .. hard
FLO = [0.00, 0.60, 0.93]
FHI = [0.95, 0.96, 0.97]
RLO = [0.22, 0.45, 0.70, 1.0]
RHI = [0.05, 0.16, 0.30, 1.0]

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's, extended with the grid port)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XS=9, IL=10, ROM=11, W=12, GD=13, HN=14, ZERO=15)
CONDS = dict(LT=0, GLIDE=1, PAVE=2, GRDY=3, GE=8, NGLIDE=9, NPAVE=10, NGRDY=11)
OUTS = dict(ADR=0, TA=1, TD=2, GA=3)


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
            assert len(self.regs) <= 256, "register file full"
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

    def shl(self, n, tmp='sh0'):
        """ACC <<= n by doubling (no 18-bit operand limit)"""
        for _ in range(n):
            self.ST(tmp); self.ADD(tmp)

    def neg(self, tmp):
        self.ST(tmp); self.LDI(0); self.SUB(tmp)

    def const(self, v):
        n = 'C%d' % v if v >= 0 else 'CM%d' % (-v)
        self.reg(n, v)
        return n

    def clamp(self, lo, hi):
        """ACC = min(max(ACC, lo), hi) (registers or constants)"""
        a, b = self.new(), self.new()
        self.ST('cl0'); self.SUB(lo); self.SKIP('LT'); self.JMP(a); self.LD(lo); self.JMP(b)
        self.label(a); self.LD('cl0'); self.SUB(hi); self.SKIP('GE'); self.JMP(b + 'x'); self.LD(hi); self.JMP(b)
        self.label(b + 'x'); self.LD('cl0')
        self.label(b)


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    C = a.const
    # state that lives across fields (power-up = the TOML defaults)
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3', D['k3']), ('h4', D['k4']), ('h5', D['k5']),
                 ('h6', D['k6']), ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a4', D['k4']), ('a5', D['k5']),
                 ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('th', 0)                                 # the light's phase, 2^26 = a full turn
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

    # ---- SINF: ACC = 32768 sin(2 pi phi / 2^26), phi any sign (taken modulo a full turn)
    a.label('SINF')
    a.ST('fp'); a.SHR(26); a.MULI(1024); a.MUL(C(65536)); a.ST('x0'); a.LD('fp'); a.SUB('x0'); a.ST('fp')
    a.SHR(24); a.ST('fq'); a.MULI(256); a.MUL(C(65536)); a.ST('x0'); a.LD('fp'); a.SUB('x0'); a.ST('fr')
    q1, q2, q3 = a.new(), a.new(), a.new()
    a.LD('fq'); a.SUBI(1); a.SKIP('LT'); a.JMP(q1)
    a.LD('fr'); a.CALL('SIN'); a.RET()                                      # quadrant 0
    a.label(q1); a.SUBI(1); a.SKIP('LT'); a.JMP(q2)
    a.LD(C(Q24)); a.SUB('fr'); a.CALL('SIN'); a.RET()                       # 1
    a.label(q2); a.SUBI(1); a.SKIP('LT'); a.JMP(q3)
    a.LD('fr'); a.CALL('SIN'); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.RET()   # 2
    a.label(q3); a.LD(C(Q24)); a.SUB('fr'); a.CALL('SIN'); a.ST('x0'); a.LDI(0); a.SUB('x0'); a.RET()

    # ---- RDI: ACC = ROM[rb + (ap >> 10)] interpolated by ap & 1023 (unsigned table words)
    a.label('RDI')
    a.LD('ap'); a.SHR(10); a.ST('ri'); a.ADD('rb'); a.OUT('ADR'); a.IN('ROM'); a.ST('r0')
    a.LD('ri'); a.ADD('rb'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM'); a.SUB('r0'); a.ST('r1')
    a.LD('ri'); a.MULI(1024); a.ST('x0'); a.LD('ap'); a.SUB('x0'); a.MUL('r1'); a.SHR(10); a.ADD('r0'); a.RET()

    a.label('main')
    # ---- knob holds (a knob has to move 2 counts) and glides (Q6).  Glide On: 1/16 of the gap per field, never
    #      under half a count, landing exactly (the Drift speed: 1/8); Off: the held value, no easing.
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
    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3', 'a3'), ('K4', 'h4', 'a4'),
                        ('K6', 'h6', 'a6'), ('P12', 'hp', 'ap')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(acc); a.ST('aa'); a.CALL('HG')
        a.LD('hh'); a.ST(h); a.LD('aa'); a.ST(acc)
    u, k = a.new(), a.new()
    a.IN('K5'); a.SUB('h5'); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
    a.label(u); a.IN('K5'); a.ST('h5')
    a.label(k)
    glide('h5', 'a5', 3, 0, 8)

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

    # ---- Drift (Hypnos SPEED): |v| < 24 still; else 2^(-11 + ls / 64) turn per field, ls = (|v| - 24) * 0.78
    #      (top 2^-5): the light circles, the stone turns -1/4 as fast (a quarter turn per light turn: the 8-fold
    #      stone looks the same, so the phase wraps exactly at one light turn)
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(2); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(5); a.ADD('ls'); a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('v'); a.SKIP('LT'); a.JMP('dpos')
    a.LD('th'); a.SUB('w'); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(Q26)); a.JMP('dst')
    a.label('dpos')
    a.LD('th'); a.ADD('w'); a.SUB(C(Q26)); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(Q26))
    a.label('dst')
    a.ST('th')
    a.label('still')
    # the stone's angle -th / 4 -> cs, sn (Q15)
    a.LD('th'); a.SHR(2); a.ST('x0'); a.LD(C(Q26)); a.SUB('x0'); a.ST('ths'); a.CALL('SINF'); a.ST('sn')
    a.LD('ths'); a.ADD(C(Q24)); a.CALL('SINF'); a.ST('cs')

    # ---- the cut from P12: t, s (Q15), kappa2, kappa3 (Q10); alpha (x/8) from |kappa|
    def sext16(r):
        nx = a.new()
        a.LD(r); a.SUB(C(32768)); a.SKIP('GE'); a.JMP(nx); a.SUB(C(32768)); a.ST(r)
        a.label(nx)
    a.LD(C(ROM_T)); a.ST('rb'); a.CALL('RDI'); a.ST('tq')
    a.LD(C(ROM_S)); a.ST('rb'); a.CALL('RDI'); a.ST('sq')
    for rb, dst in ((ROM_K2, 'k2'), (ROM_K3, 'k3')):
        a.LD(C(rb)); a.ST('rb')
        # interpolate with sign-extended neighbours
        a.LD('ap'); a.SHR(10); a.ST('ri'); a.ADD('rb'); a.OUT('ADR'); a.IN('ROM'); a.ST('r0'); sext16('r0')
        a.LD('ri'); a.ADD('rb'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM'); a.ST('r1'); sext16('r1')
        a.LD('r1'); a.SUB('r0'); a.ST('r1')
        a.LD('ri'); a.MULI(1024); a.ST('x0'); a.LD('ap'); a.SUB('x0'); a.MUL('r1'); a.SHR(10); a.ADD('r0'); a.ST(dst)
    for kr, al in (('k2', 'al2'), ('k3', 'al3')):
        a.LD(kr); a.abs_('x0'); a.SHR(5); a.ST('x1'); a.SUBI(63); a.SKIP('GE'); a.JMP(al + 'r'); a.LDI(63); a.ST('x1')
        a.label(al + 'r')
        a.LD('x1'); a.ADD(C(ROM_AL)); a.OUT('ADR'); a.IN('ROM'); a.ADD(C(2048)); a.SHR(12); a.ST(al)  # x / 8
        s8, s6, s4, s3 = a.new(), a.new(), a.new(), a.new()                                 # -> 8, 6, 4, 3 or 2
        a.LD(al); a.SUBI(7); a.SKIP('LT'); a.JMP(s8); a.ADDI(2); a.SKIP('LT'); a.JMP(s6)
        a.ADDI(1); a.SKIP('LT'); a.JMP(s4); a.ADDI(1); a.SKIP('LT'); a.JMP(s3); a.LDI(2); a.JMP(al + 'x')
        a.label(s8); a.LDI(8); a.JMP(al + 'x')
        a.label(s6); a.LDI(6); a.JMP(al + 'x')
        a.label(s4); a.LDI(4); a.JMP(al + 'x')
        a.label(s3); a.LDI(3)
        a.label(al + 'x'); a.ST(al)

    # ---- Form: Stone R = 225 .. 520 .. 1200 virtual px; Pave cell 130 .. 266 .. 546, stone radius 0.48 cell.
    #      Centre / pan from X, Y: +-640, +-360 virtual px (up = up)
    a.LD('a3'); a.MULI(618); a.SHR(16); a.ADDI(2000); a.CALL('EXPQ4'); a.ST('R4')          # Q4 virtual px
    a.LD('a3'); a.MULI(530); a.SHR(16); a.ADDI(1798); a.CALL('EXPQ4'); a.ST('P4')
    a.LD('a1'); a.SUB(C(32768)); a.MULI(10); a.SHR(5); a.MUL('ivx'); a.SHR(16); a.ST('xo')   # pixels x 4
    a.LD('a2'); a.SUB(C(32768)); a.MULI(90); a.SHR(9); a.MUL('ivy'); a.SHR(18); a.ST('yo')   # lines x 4
    a.IN('W'); a.ADDI(NEXTRA); a.ST('N')
    a.IN('HN'); a.ST('hn')
    a.SKIP('PAVE'); a.JMP('stone')
    a.LD('P4'); a.MULI(983); a.SHR(11); a.ST('R4'); a.JMP('rdone')                       # 0.48 cell
    a.label('stone')
    a.label('rdone')
    # R in 1/8 units (Q3): R4 / 2 >> ush
    a.LD('R4'); a.SHR(1); a.CALL('SHRU'); a.ST('R8')
    # gamma1 = t R, gamma4 = R; S0 = s R (c22, s22); gamma2 = aS + kappa2 bS, gamma3 = aS + kappa3 bS
    a.LD('R8'); a.MUL('tq'); a.SHR(15); a.ST('g1')
    a.LD('R8'); a.MUL('sq'); a.SHR(15); a.ST('sR')
    a.LD('sR'); a.MUL(C(int(round(C22 * 32768)))); a.SHR(15); a.ST('aS')
    a.LD('sR'); a.MUL(C(int(round(S22 * 32768)))); a.SHR(15); a.ST('bS')
    a.LD('k2'); a.MUL('bS'); a.ADDI(256); a.SHR(9); a.ADD('aS'); a.ST('g2')
    a.LD('k3'); a.MUL('bS'); a.ADDI(256); a.SHR(9); a.ADD('aS'); a.ST('g3')
    # Fine half line width: R / 180 within 1.25 .. 3 virtual px, in 1/8 units
    a.LD('R4'); a.MULI(91); a.SHR(15); a.ST('x0'); a.SUBI(10); a.SKIP('LT'); a.JMP('wlo'); a.LDI(10); a.ST('x0')
    a.label('wlo')
    a.LD('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('wok'); a.LDI(24); a.ST('x0')
    a.label('wok')
    a.LD('x0'); a.CALL('SHRU'); a.ST('hw')

    # ---- rotations: per pixel / per line steps of U = cs dx + sn dy, V = -sn dx + cs dy (Q14 units)
    a.LD('cs'); a.MUL('pw'); a.SHR(15); a.ST('cpw'); a.LD('sn'); a.MUL('pw'); a.SHR(15); a.ST('spw')
    a.LD('cs'); a.MUL('lh'); a.SHR(15); a.ST('clh'); a.LD('sn'); a.MUL('lh'); a.SHR(15); a.ST('slh')
    a.LDI(0); a.SUB('spw'); a.ST('nspw')

    # big stone: dx0 = DDA index 0's centre - the centre, in half pixels: 1 - 2 XOFS - W - xo / 2
    #            dy0 = line 0's centre - the centre, in half lines: 1 - HN + yo / 2
    a.IN('W'); a.ST('x0'); a.LDI(1 - 2 * XOFS); a.SUB('x0'); a.ST('x1'); a.LD('xo'); a.SHR(1); a.ST('x0')
    a.LD('x1'); a.SUB('x0'); a.ST('dx0')
    a.LDI(1); a.SUB('hn'); a.ST('x1'); a.LD('yo'); a.SHR(1); a.ADD('x1'); a.ST('dy0')

    # DDA (Stone): one accumulator's five words: pixel step pc (twice: the cell wraps do nothing), the line wrap
    # (twice), the seed pc dx0 + lc dy0 (half steps)
    a.JMP('ddx')
    a.label('DDA')
    a.LD('tb'); a.OUT('TA'); a.ADDI(8); a.ST('tb')
    a.LD('pc'); a.SHR(2); a.ST('t0'); a.OUT('TD'); a.LD('t0'); a.OUT('TD')               # Q12 step
    a.LD('t0'); a.MUL('N'); a.ST('t1'); a.LD('lc'); a.SHR(2); a.SUB('t1'); a.ST('t1'); a.OUT('TD'); a.LD('t1'); a.OUT('TD')
    a.LD('pc'); a.MUL('dx0'); a.ST('t1'); a.LD('lc'); a.MUL('dy0'); a.ADD('t1'); a.SHR(3); a.OUT('TD')
    a.RET()
    a.label('ddx')

    # ---- lattice (Pave: cell P4 about the pan point; Stone: the glitter's 40 virtual px about the screen centre)
    a.SKIP('PAVE'); a.JMP('lst')
    a.LD('P4'); a.ST('L4'); a.LD('xo'); a.ST('lxo'); a.LD('yo'); a.ST('lyo'); a.JMP('lgo')
    a.label('lst')
    a.LD(C(GLITTER_C4)); a.ST('L4'); a.LDI(0); a.ST('lxo'); a.ST('lyo')
    a.label('lgo')
    # cell width / height in whole pixels / lines (at least 8)
    a.LD('L4'); a.MUL('ivx'); a.ADD(C(1 << 17)); a.SHR(18); a.ST('wc')
    a.LD('L4'); a.MUL('ivy'); a.ADD(C(1 << 19)); a.SHR(20); a.ST('hc')
    # x: a cell centre at pixel xcen = W / 2 + lxo / 4 (the pixel whose centre is nearest); DDA index i is screen
    #    column i - XOFS.  pos(i) = (i - XOFS - xcen + wc / 2) mod wc, measured so the cell spans [xcen - wc/2,
    #    xcen + wc/2).  xl0 = 2 pos(0) + 1 - wc (half pixels from the cell centre); xc0 = wc - 1 - pos(0).
    def modwc(n, reg_out):
        """ACC (>= -64 n) mod reg n -> reg_out (0 .. n - 1); also the quotient change in 'mq' (unused)."""
        lp, ok, up = a.new(), a.new(), a.new()
        a.ST('mx0')
        a.label(up); a.LD('mx0'); a.SKIP('LT'); a.JMP(lp); a.ADD(n); a.ST('mx0'); a.JMP(up)
        a.label(lp); a.LD('mx0'); a.SUB(n); a.SKIP('GE'); a.JMP(ok); a.ST('mx0'); a.JMP(lp)
        a.label(ok); a.LD('mx0'); a.ST(reg_out)
    a.IN('W'); a.SHR(1); a.ST('x0'); a.LD('lxo'); a.SHR(2); a.ADD('x0'); a.ST('xcen')
    a.LD('wc'); a.SHR(1); a.ST('x1'); a.LDI(-XOFS); a.SUB('xcen'); a.ADD('x1'); modwc('wc', 'pos0')
    a.LD('wc'); a.SUBI(1); a.SUB('pos0'); a.ST('xc0')
    a.LD('pos0'); a.ADD('pos0'); a.ADDI(1); a.SUB('wc'); a.ST('xl0')
    # cells crossed in the line window: nwx = number of i in [0, N) with xc(i) == 0
    a.LDI(0); a.ST('nwx'); a.LD('N'); a.SUBI(1); a.SUB('xc0'); a.ST('x2')
    lw, lwd = a.new(), a.new()
    a.label(lw); a.LD('x2'); a.SKIP('GE'); a.JMP(lwd); a.SUB('wc'); a.ST('x2'); a.LD('nwx'); a.ADDI(1); a.ST('nwx'); a.JMP(lw)
    a.label(lwd)
    # first cell's column index: (DDA index 0 relative to the centre cell), floor((pos-free) ...): col0 = 128 - k
    #    where k = cells from index 0 to the centre cell = (xcen - (-XOFS) + wc/2 - pos0 ... ) / wc (a loop)
    a.LDI(-XOFS); a.SUB('xcen'); a.ADD('x1'); a.SUB('pos0'); a.ST('x2'); a.LDI(128); a.ST('col0')   # x2 = -k wc
    lc_, lcd = a.new(), a.new()
    a.label(lc_); a.LD('x2'); a.SKIP('LT'); a.JMP(lcd); a.ADD('wc'); a.ST('x2'); a.LD('col0'); a.SUBI(1); a.ST('col0'); a.JMP(lc_)
    a.label(lcd)
    # y: a cell centre at line ycen = HN / 2 - lyo / 4; ypos(0) = (0 - ycen + hc / 2) mod hc
    a.LD('hn'); a.SHR(1); a.ST('x0'); a.LD('lyo'); a.SHR(2); a.ST('x2'); a.LD('x0'); a.SUB('x2'); a.ST('ycen')
    a.LD('hc'); a.SHR(1); a.ST('x1'); a.LDI(0); a.SUB('ycen'); a.ADD('x1'); modwc('hc', 'ypos0')
    a.LD('hc'); a.SUBI(1); a.SUB('ypos0'); a.ST('yc0')
    a.LD('ypos0'); a.ADD('ypos0'); a.ADDI(1); a.SUB('hc'); a.ST('yl0')
    a.LDI(0); a.SUB('ycen'); a.ADD('x1'); a.SUB('ypos0'); a.ST('x2'); a.LDI(128); a.ST('row0')
    lr_, lrd = a.new(), a.new()
    a.label(lr_); a.LD('x2'); a.SKIP('LT'); a.JMP(lrd); a.ADD('hc'); a.ST('x2'); a.LD('row0'); a.SUBI(1); a.ST('row0'); a.JMP(lr_)
    a.label(lrd)
    # lattice words: u = cs xl + sn yl, v = -sn xl + cs yl (xl, yl from the cell centre)
    a.JMP('ldx')
    a.label('LDDA')                                # pc, lc (Q14) -> five words at tb
    a.LD('tb'); a.OUT('TA'); a.ADDI(8); a.ST('tb')
    a.LD('pc'); a.SHR(2); a.ST('t0'); a.OUT('TD')                                       # step
    a.LD('t0'); a.MUL('wc'); a.ST('t1'); a.LD('t0'); a.SUB('t1'); a.OUT('TD')            # step at a cell wrap
    a.LD('N'); a.SUB('nwxw'); a.MUL('t0'); a.ST('t1'); a.LD('lc'); a.SHR(2); a.ST('t2'); a.SUB('t1'); a.ST('t3')
    a.OUT('TD')                                                                          # line wrap
    a.LD('t2'); a.MUL('hc'); a.ST('t1'); a.LD('t3'); a.SUB('t1'); a.OUT('TD')            # wrap at a row wrap
    a.LD('pc'); a.MUL('xl0'); a.ST('t1'); a.LD('lc'); a.MUL('yl0'); a.ADD('t1'); a.SHR(3); a.OUT('TD')   # seed
    a.RET()
    a.label('ldx')
    a.LD('nwx'); a.MUL('wc'); a.ST('nwxw')
    a.LDI(0); a.ST('tb')
    a.SKIP('PAVE'); a.JMP('dstone')
    a.LD('cpw'); a.ST('pc'); a.LD('slh'); a.ST('lc'); a.CALL('LDDA')                   # X = u
    a.LD('nspw'); a.ST('pc'); a.LD('clh'); a.ST('lc'); a.CALL('LDDA')                  # Y = v
    a.JMP('ddone')
    a.label('dstone')
    a.LD('cpw'); a.ST('pc'); a.LD('slh'); a.ST('lc'); a.CALL('DDA')                    # X = U
    a.LD('nspw'); a.ST('pc'); a.LD('clh'); a.ST('lc'); a.CALL('DDA')                   # Y = V
    a.label('ddone')

    # ---- camera blocks of the cells (34 virtual px blocks, Q6.10): block x of the first cell's centre and the
    #      step per cell; the same down
    #      first centre (virtual px) = (W/2 + lxo/4 - (128 - col0) wc ... ) : computed in pixels then x 1/ivx
    a.LDI(128); a.SUB('col0'); a.MUL('wc'); a.ST('x0'); a.LD('xcen'); a.SUB('x0'); a.ST('x1')     # pixels
    # pixels -> blocks: x xs / 2 / 34 (Q10): xs half-px per px -> x * xs * 1024 / 68
    a.LD('x1'); a.MUL('xs'); a.MULI(16); a.ST('bx0')                                           # px xs/2/32 Q10
    a.LD('wc'); a.MUL('xs'); a.MULI(16); a.ST('bxs')
    a.LDI(128); a.SUB('row0'); a.MUL('hc'); a.ST('x0'); a.LD('ycen'); a.SUB('x0'); a.ST('x1')     # lines
    a.LD('x1'); a.MUL('ys'); a.SHR(3); a.ST('by0')                                # x ys/256/34 Q10
    a.LD('hc'); a.MUL('ys'); a.SHR(3); a.ST('bys')

    # ---- tones: per facet type and angular sector: base + flash (+ the per-facet camera step, Stone)
    #      flash = number of thresholds T(Fire) under cos(azimuth + stone - light)
    a.LD('a4'); a.ST('fire')
    for i in range(3):
        lo, hi = int(round(FLO[i] * 32768)), int(round(FHI[i] * 32768))
        a.LD('fire'); a.MUL(C(hi - lo)); a.SHR(16); a.ADD(C(lo)); a.ST('ft%d' % i)
    a.LD('a6'); a.SHR(8); a.ST('g6')                                                       # the camera gain 0 .. 255
    # light relative angle: -5 th / 4 (stone -th/4 minus light th)
    a.LD('th'); a.SHR(2); a.ST('x0'); a.LD(C(Q26)); a.SUB('th'); a.SUB('x0'); a.ST('rel')
    # the stone's centre in Q4 virtual px from the screen's top-left (for the per-facet camera)
    a.IN('W'); a.MUL('xs'); a.MULI(4); a.ST('x0'); a.LD('a1'); a.SUB(C(32768)); a.MULI(10); a.SHR(5); a.ADD('x0'); a.ST('cx16')
    a.LD('hn'); a.MUL('ys'); a.SHR(5); a.ST('x0'); a.LD('a2'); a.SUB(C(32768)); a.MULI(90); a.SHR(9); a.ST('x1')
    a.LD('x0'); a.SUB('x1'); a.ST('cy16')
    # camera on = Stone with K6 above 0
    a.LDI(0); a.ST('con')
    co = a.new()
    a.SKIP('NPAVE'); a.JMP(co); a.LD('a6'); a.SUBI(64); a.SKIP('GE'); a.JMP(co); a.LDI(1); a.ST('con')
    a.label(co)
    # the picture's mean: 64 samples of the grid (any Form, K6 above 0)
    a.LDI(0); a.ST('mean')
    a.LD('a6'); a.SUBI(64); a.SKIP('GE'); a.JMP('nomean')
    a.IN('W'); a.MUL('xs'); a.SHR(9); a.ST('mstp')                                         # 7 HD, 5 SD
    a.LDI(0); a.ST('msum'); a.LDI(0); a.ST('mi')
    ml = a.new()
    a.label(ml)
    a.LD('mi'); a.SHR(3); a.ST('x0'); a.MULI(8); a.ST('x1'); a.LD('mi'); a.SUB('x1'); a.MUL('mstp'); a.ADDI(3); a.ST('x1')
    a.LD('x0'); a.MULI(4); a.ADDI(2); a.MULI(60); a.ADD('x1'); a.OUT('GA')
    wq = a.new(); a.label(wq); a.SKIP('GRDY'); a.JMP(wq)
    a.IN('GD'); a.ADD('msum'); a.ST('msum')
    a.LD('mi'); a.ADDI(1); a.ST('mi'); a.SUBI(64); a.SKIP('GE'); a.JMP(ml)
    a.LD('msum'); a.SHR(6); a.ST('mean')
    a.label('nomean')

    # per type: the anchor of its facets (radius ra, Q15 of R; wedge angle wa, 2^26 per turn), then 16 sectors,
    # sector 15 first so the word is built by shifting left (sector j in bits 2 j + 1 .. 2 j)
    AZC = dict(kite=(0, 2), star=(2, -2), girdle=(1, 0), ray=(17, 0))
    for ti, ty in enumerate(TYPES):
        if ty == 'kite':          # (t + 1 + 2 s c22) / 4 on the kite axis
            a.LD('sq'); a.MUL(C(int(round(2 * C22 * 16384)))); a.SHR(14); a.ADD('tq'); a.ADD(C(32768)); a.SHR(2); a.ST('ra')
            a.LDI(0); a.ST('wa')
        elif ty == 'star':        # (2 t c22 + s) / 3 on the star axis
            a.LD('tq'); a.MUL(C(int(round(2 * C22 * 16384)))); a.SHR(14); a.ADD('sq'); a.MULI(683); a.SHR(11); a.ST('ra')
            a.LD(C(1 << 22)); a.ST('wa')
        elif ty == 'girdle':      # centroid of (S0, G0, G1): 0.674 + 0.324 s at 18 degrees
            a.LD('sq'); a.MULI(664); a.SHR(11); a.ADD(C(22086)); a.ST('ra')
            a.LD(C(3355443)); a.ST('wa')
        elif ty == 'ray':         # centroid of (O, T0, M): 0.629 t at 10.8 degrees
            a.LD('tq'); a.MULI(1288); a.SHR(11); a.ST('ra')
            a.LD(C(2013266)); a.ST('wa')
        z0, z1 = AZC[ty]
        a.LDI(z0); a.ST('az0'); a.LDI(z1); a.ST('az1'); a.LDI(BASE[ty][0]); a.ST('be'); a.LDI(BASE[ty][1]); a.ST('bo')
        a.CALL('TONE16')
        a.LD('word'); a.ST('tw%d' % ti)
    # the sector words 48 .. 63 (the pixel path reads one per pixel): sector j's kite, star, girdle, girdle of
    # j xor 1, ray, ray of j xor 1, ray of j0 (the neighbour across the kite axis), 2 bits each from bit 0
    a.JMP('swx')
    a.label('M4')                                   # ACC = ACC mod 4 (any width)
    a.ST('m40'); a.SHR(2); a.ST('m41'); a.ADD('m41'); a.ST('m41'); a.ADD('m41'); a.ST('m41'); a.LD('m40'); a.SUB('m41'); a.RET()
    a.label('swx')
    for ti in range(4):
        a.LD('tw%d' % ti); a.ST('sv%d' % ti)
    a.LD('tw2'); a.SHR(30); a.ST('pg'); a.LD('tw3'); a.SHR(30); a.ST('pr')      # sector 15's girdle / ray
    a.LD('pg'); a.CALL('M4'); a.ST('pg'); a.LD('pr'); a.CALL('M4'); a.ST('pr')
    a.LD('tw3'); a.CALL('M4'); a.ST('r00')                                        # sector 0's ray
    a.LDI(0); a.ST('jj'); a.LDI(48); a.OUT('TA')
    lsw, odd_, wr_ = a.new(), a.new(), a.new()
    a.label(lsw)
    a.LD('sv0'); a.CALL('M4'); a.ST('fk'); a.LD('sv1'); a.CALL('M4'); a.ST('fs')
    a.LD('sv2'); a.CALL('M4'); a.ST('fg'); a.LD('sv3'); a.CALL('M4'); a.ST('fr')
    a.LD('sv2'); a.SHR(2); a.CALL('M4'); a.ST('fgn'); a.LD('sv3'); a.SHR(2); a.CALL('M4'); a.ST('frn')
    a.LD('jj'); a.SHR(1); a.MULI(2); a.SUB('jj'); a.SKIP('GE'); a.JMP(odd_)
    a.LD('fgn'); a.ST('tg1'); a.LD('frn'); a.ST('tr1'); a.LD('pr'); a.ST('tr0'); a.JMP(wr_)     # even: j + 1, j - 1
    a.label(odd_)
    a.LD('pg'); a.ST('tg1'); a.LD('pr'); a.ST('tr1'); a.LD('frn'); a.ST('tr0')                  # odd: j - 1, j + 1
    a.LD('jj'); a.SUBI(15); a.SKIP('GE'); a.JMP(wr_); a.LD('r00'); a.ST('tr0')                 # (15 + 1 = 0)
    a.label(wr_)
    a.LD('tr0'); a.MULI(4); a.ADD('tr1'); a.MULI(4); a.ADD('fr'); a.MULI(4); a.ADD('tg1'); a.MULI(4); a.ADD('fg')
    a.MULI(4); a.ADD('fs'); a.MULI(4); a.ADD('fk'); a.OUT('TD')
    a.LD('fg'); a.ST('pg'); a.LD('fr'); a.ST('pr')
    for ti in range(4):
        a.LD('sv%d' % ti); a.SHR(2); a.ST('sv%d' % ti)
    a.LD('jj'); a.ADDI(1); a.ST('jj'); a.SUBI(16); a.SKIP('GE'); a.JMP(lsw)
    a.JMP('dots')

    # TONE16: the 16 sectors of one type (sector 15 first: the word is built by shifting left)
    a.label('TONE16')
    a.LDI(0); a.ST('word'); a.LDI(15); a.ST('jj')
    lj = a.new()
    a.label(lj)
    a.LD('jj'); a.SHR(1); a.MULI(2); a.SUB('jj'); a.ST('jo')                            # 0 even, -1 odd
    a.LD('jj'); a.MULI(2); a.ADD('az0'); a.ST('x0'); a.LD('jo'); a.MUL('az1'); a.ST('x1'); a.LD('x0'); a.SUB('x1')
    a.MULI(32); a.MUL(C(65536)); a.ADD('rel'); a.ADD(C(Q24)); a.CALL('SINF'); a.ST('cv')   # cos (az / 32 turn)
    a.LDI(0); a.ST('tn')
    for i in range(3):
        nx = a.new()
        a.LD('cv'); a.SUB('ft%d' % i); a.SKIP('GE'); a.JMP(nx); a.LD('tn'); a.ADDI(1); a.ST('tn')
        a.label(nx)
    od = a.new(); dn_ = a.new()
    a.LD('jo'); a.SKIP('GE'); a.JMP(od)
    a.LD('tn'); a.ADD('be'); a.ST('tn'); a.JMP(dn_)
    a.label(od); a.LD('tn'); a.ADD('bo'); a.ST('tn')
    a.label(dn_)
    fc = a.new()
    a.LD('con'); a.SUBI(1); a.SKIP('GE'); a.JMP(fc); a.CALL('FCAM')
    a.label(fc)
    c0, c1 = a.new(), a.new()
    a.LD('tn'); a.SKIP('LT'); a.JMP(c0); a.LDI(0); a.ST('tn')
    a.label(c0); a.LD('tn'); a.SUBI(4); a.SKIP('GE'); a.JMP(c1); a.LDI(3); a.ST('tn')
    a.label(c1)
    a.LD('word'); a.ST('x0'); a.ADD('x0'); a.ST('x0'); a.ADD('x0'); a.ADD('tn'); a.ST('word')
    a.LD('jj'); a.SUBI(1); a.ST('jj'); a.SKIP('LT'); a.JMP(lj)
    a.RET()
    a.JMP('dots')

    # FCAM: tn += clamp(round(9 (L - mean) / 256), -2, 2), L = the grid at the facet's anchor in sector jj
    a.label('FCAM')
    a.LD('jj'); a.SHR(1); a.MULI(2); a.SUB('jj'); a.ST('x0')                              # 0 even, -1 odd
    fo, fd = a.new(), a.new()
    a.LD('x0'); a.SKIP('LT'); a.JMP(fo)
    a.LD('jj'); a.MULI(64); a.MUL(C(65536)); a.ADD('wa'); a.JMP(fd)               # 22.5 j + wa
    a.label(fo)
    a.LD('jj'); a.ADDI(1); a.MULI(64); a.MUL(C(65536)); a.SUB('wa')               # 22.5 (j+1) - wa
    a.label(fd)
    a.ADD('ths'); a.ST('fa'); a.ADD(C(Q24)); a.CALL('SINF'); a.ST('fc'); a.LD('fa'); a.CALL('SINF'); a.ST('fs')
    a.LD('R4'); a.MUL('ra'); a.SHR(15); a.ST('rr')
    a.LD('rr'); a.MUL('fc'); a.SHR(15); a.ADD('cx16'); a.ST('ax')
    a.LD('rr'); a.MUL('fs'); a.SHR(15); a.ADD('cy16'); a.ST('ay')
    # block (32 virtual px = 512 in Q4), clamped to 0 .. 59 / 0 .. 33
    a.LD('ax'); a.SHR(9); a.ST('bxx')
    a.LD('ay'); a.SHR(9); a.ST('byy')
    for r, hi in (('bxx', 59), ('byy', 33)):
        o1, o2 = a.new(), a.new()
        a.LD(r); a.SKIP('LT'); a.JMP(o1); a.LDI(0); a.ST(r)
        a.label(o1); a.LD(r); a.SUBI(hi + 1); a.SKIP('GE'); a.JMP(o2); a.LDI(hi); a.ST(r)
        a.label(o2)
    a.LD('byy'); a.MULI(60); a.ADD('bxx'); a.OUT('GA')
    wq = a.new(); a.label(wq); a.SKIP('GRDY'); a.JMP(wq)
    a.IN('GD'); a.SUB('mean'); a.MUL('g6'); a.MULI(9); a.ADD(C(16384)); a.SHR(15); a.ST('x0')       # 9 g (L - m) / 2^15
    f1, f2 = a.new(), a.new()
    a.LD('x0'); a.ADDI(2); a.SKIP('LT'); a.JMP(f1); a.LDI(-2); a.ST('x0')
    a.label(f1); a.LD('x0'); a.SUBI(3); a.SKIP('GE'); a.JMP(f2); a.LDI(2); a.ST('x0')
    a.label(f2)
    a.LD('tn'); a.ADD('x0'); a.ST('tn'); a.RET()

    a.label('dots')
    # ---- word 38: the dots' twinkle phase (the light's phase in sixteenths of a turn): a dot twinkles when its
    #      cell index (col + 3 row) is 0 or 1 past it, so the twinkles sweep across the halftone as the light goes round
    a.LD('th'); a.SHR(22); a.ST('x0'); a.SHR(4); a.MULI(16); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.ST('word')
    a.LDI(38); a.OUT('TA'); a.LD('word'); a.OUT('TD')

    # ---- word 39, ramp (6-bit luma of tones 0 .. 3, tone k in bits 8 k + 5 .. 8 k): Grey Bold by Fire;
    #      Grey Fine 0 0 0 29; Ink Bold 0 0 63 63; Ink Fine 0 0 0 63
    a.IN('SW'); a.ST('sw')
    a.LD('sw'); a.SHR(2); a.ST('x0'); a.SHR(1); a.ST('fine'); a.MULI(2); a.ST('x1'); a.LD('x0'); a.SUB('x1'); a.ST('ink')
    a.LD('fine'); a.SHR(1); a.MULI(2); a.ST('x1'); a.LD('fine'); a.SUB('x1'); a.ST('fine')
    a.LDI(0); a.ST('word')
    rf_, ri_, rd_, rfi = a.new(), a.new(), a.new(), a.new()
    a.LD('ink'); a.SUBI(1); a.SKIP('LT'); a.JMP(ri_)
    a.LD('fine'); a.SUBI(1); a.SKIP('LT'); a.JMP(rf_)
    for i in (3, 2, 1, 0):
        lo, hi = int(round(RLO[i] * 63 * 256)), int(round(RHI[i] * 63 * 256))
        a.CALL('W8')
        a.LD('fire'); a.MUL(C(hi - lo)); a.SHR(16); a.ADD(C(lo)); a.ADDI(128); a.SHR(8); a.ADD('word'); a.ST('word')
    a.JMP(rd_)
    a.label(rf_); a.LD(C(29 << 24)); a.ST('word'); a.JMP(rd_)                             # Grey Fine
    a.label(ri_)
    a.LD('fine'); a.SUBI(1); a.SKIP('LT'); a.JMP(rfi); a.LD(C((63 << 24) | (63 << 16))); a.ST('word'); a.JMP(rd_)
    a.label(rfi); a.LD(C(63 << 24)); a.ST('word')
    a.label(rd_)
    a.LDI(39); a.OUT('TA'); a.LD('word'); a.OUT('TD')

    # ---- word 40: dot luma (46, Ink 63) | twinkle luma 63 << 8
    il_, id_ = a.new(), a.new()
    a.LD('ink'); a.SUBI(1); a.SKIP('LT'); a.JMP(il_); a.LDI(46); a.JMP(id_)
    a.label(il_); a.LDI(63)
    a.label(id_)
    a.ST('x0'); a.LDI(63); a.MULI(256); a.ADD('x0'); a.OUT('TD')

    # ---- word 41 (+ word 32 bits 29 .. 26): size thresholds T1 .. T4, 9 bits, against the raw grid.  The camera
    #      level of a cell = the number of T(k) it reaches.  Stone (glitter): T = ceil(51 k' 128 / g) (256 = never);
    #      Pave: T = mean - floor(delta 128 / g), at least 0 (delta = 74, 53, 32, 11: a stone shrinks below the mean)
    a.LDI(0); a.ST('word'); a.LDI(0); a.ST('thb')
    for i in (3, 2, 1, 0):
        sk, pv, dn_ = a.new(), a.new(), a.new()
        a.CALL('W8')
        a.LD('thb'); a.ST('x0'); a.ADD('x0'); a.ST('thb')
        a.SKIP('PAVE'); a.JMP(sk)
        a.LD(C(int(round((3.5 - i) * 21.25)) * 128)); a.ST('dvn'); a.CALL('DIV')               # floor(delta 128 / g)
        a.ST('x0'); a.LD('mean'); a.SUB('x0'); a.SKIP('LT'); a.JMP(dn_); a.LDI(0); a.JMP(dn_)
        a.label(sk)
        a.LD(C(51 * (i + 1) * 128)); a.ADD('g6'); a.SUBI(1); a.ST('dvn'); a.CALL('DIV')         # ceil(51 k 128 / g)
        a.label(dn_)
        a.ST('x0'); a.SUBI(256); a.SKIP('GE'); a.JMP(pv); a.LD('thb'); a.ADDI(1); a.ST('thb'); a.LD('x0'); a.SUBI(256); a.ST('x0')
        a.label(pv)
        a.LD('x0'); a.ADD('word'); a.ST('word')
    a.LD('word'); a.OUT('TD')

    # ---- words 24 .. 32
    a.LDI(24); a.OUT('TA')
    a.LD('wc'); a.SUBI(1); a.MUL(C(4096)); a.ADD('xc0'); a.OUT('TD')
    a.LD('hc'); a.SUBI(1); a.MUL(C(4096)); a.ADD('yc0'); a.OUT('TD')
    a.LD('row0'); a.MULI(256); a.ADD('col0'); a.OUT('TD')
    a.LD('bxs'); a.MUL(C(65536)); a.ST('x0'); a.LD('bx0')
    a.CALL('LO16'); a.ADD('x0'); a.OUT('TD')
    a.LD('bys'); a.MUL(C(65536)); a.ST('x0'); a.LD('by0'); a.CALL('LO16'); a.ADD('x0'); a.OUT('TD')
    a.LD('R8'); a.MUL(C(65536)); a.ADD('g1'); a.OUT('TD')
    a.LD('g3'); a.MUL(C(65536)); a.ST('x0'); a.LD('g2'); a.CALL('LO16'); a.ADD('x0'); a.OUT('TD')
    a.LD('k3'); a.MUL(C(65536)); a.ST('x0'); a.LD('k2'); a.CALL('LO16'); a.ADD('x0'); a.OUT('TD')
    # word 32: alpha2 | alpha3 << 4 | hw << 8 | dot step << 14 | dot on << 22 | pave << 23 | ink << 24 | fine << 25
    a.LDI(41); a.ST('x0')                                                                 # dot radius per level (diamond, x sqrt 2)
    a.LDI(1); a.SKIP('PAVE'); a.LDI(0); a.ST('x2')
    a.LD('thb'); a.MULI(2); a.ADD('fine'); a.MULI(2); a.ADD('ink'); a.MULI(2); a.ADD('x2'); a.MULI(2); a.ADD('con'); a.ST('x1')
    a.LD('x1'); a.MULI(256); a.ADD('x0'); a.MUL(C(16384)); a.ST('x2'); a.LD('hw'); a.MULI(16); a.ADD('al3'); a.MULI(16); a.ADD('al2'); a.ADD('x2')
    a.OUT('TD')
    # word 42: 3 kappa2 | 3 kappa3 << 16 (the pixel path's radix-4 multipliers)
    a.LDI(42); a.OUT('TA')
    a.LD('k3'); a.ST('x0'); a.ADD('x0'); a.ADD('x0'); a.MUL(C(65536)); a.ST('x1')
    a.LD('k2'); a.ST('x0'); a.ADD('x0'); a.ADD('x0'); a.CALL('LO16'); a.ADD('x1'); a.OUT('TD')
    # word 43: the glitter's vertical offset at line 0, (2 ypos0 + 1 - hc) ys, | at a cell row's first line,
    # (1 - hc) ys << 16 (1/512 virtual px; the pixel path adds 2 ys per line)
    a.LDI(1); a.SUB('hc'); a.MUL('ys'); a.CALL('LO16'); a.MUL(C(65536)); a.ST('x1')
    a.LD('ypos0'); a.ADD('ypos0'); a.ADDI(1); a.SUB('hc'); a.MUL('ys'); a.CALL('LO16'); a.ADD('x1'); a.OUT('TD')
    # word 44: the glitter's horizontal offset at DDA index 0, (wc - 1 - 2 xc0) 2 xs, | at a cell's first pixel,
    # (1 - wc) 2 xs << 16 (1/8 virtual px; the pixel path adds 4 xs per pixel)
    a.LDI(1); a.SUB('wc'); a.MUL('xs'); a.MULI(2); a.CALL('LO16'); a.MUL(C(65536)); a.ST('x1')
    a.LD('wc'); a.SUBI(1); a.SUB('xc0'); a.SUB('xc0'); a.MUL('xs'); a.MULI(2); a.CALL('LO16'); a.ADD('x1'); a.OUT('TD')
    a.HALT()

    # DIV: ACC = min(256, floor(dvn / g6)), 256 when g6 = 0 (restoring division, nine quotient bits)
    a.label('DIV')
    dz, dl, dk, dd_ = a.new(), a.new(), a.new(), a.new()
    a.LD('g6'); a.SUBI(1); a.SKIP('GE'); a.JMP(dz)
    a.LD('g6'); a.MULI(512); a.ST('dvd'); a.LD('dvn'); a.SUB('dvd'); a.SKIP('LT'); a.JMP(dz)     # >= 512 g: cap
    a.LD('g6'); a.MULI(256); a.ST('dvd'); a.LDI(0); a.ST('dvq'); a.LDI(9); a.ST('dvi')
    a.label(dl)
    a.LD('dvq'); a.ADD('dvq'); a.ST('dvq')
    a.LD('dvn'); a.SUB('dvd'); a.SKIP('GE'); a.JMP(dk); a.ST('dvn'); a.LD('dvq'); a.ADDI(1); a.ST('dvq')
    a.label(dk)
    a.LD('dvd'); a.SHR(1); a.ST('dvd'); a.LD('dvi'); a.SUBI(1); a.ST('dvi'); a.SUBI(1); a.SKIP('LT'); a.JMP(dl)
    a.LD('dvq'); a.SUBI(256); a.SKIP('LT'); a.JMP(dz); a.LD('dvq'); a.RET()
    a.label(dz); a.LDI(256); a.RET()

    # W8: word <<= 8
    a.label('W8')
    a.LD('word'); a.shl(8); a.ST('word'); a.RET()

    # LO16: ACC = ACC mod 2^16 (any sign)
    a.label('LO16')
    a.ST('lo0'); a.SHR(16); a.MUL(C(65536)); a.ST('lo1'); a.LD('lo0'); a.SUB('lo1'); a.RET()
    return a


def _build():
    a = PROGRAM()
    return a


_ASM = _build()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; brilliant.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, grid=None, max_instr=400000, trace=None, check=False):
    """inp: dict over INS names (K1.. raw knobs, YS, XS, IL, W, HN); flags: GLIDE, PAVE.
    grid: function addr -> byte (the bank the pixel path reads), or None (all 0).
    Returns (table: dict address -> signed 32-bit word, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = [0, 0, 0, 0]; adr = 0; ta = 0; gd = 0
    table = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        if trace is not None: trace.append(pc)
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
                assert -(1 << 31) <= acc < (1 << 32), (pc, acc)
            rf[arg & 255] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = TAB[adr]
            elif n == INS['ZERO']: acc = 0
            elif n == INS['GD']: acc = gd
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 3
            if n == OUTS['ADR']: adr = acc & 1023
            elif n == OUTS['TA']: ta = acc & 255
            elif n == OUTS['TD']:
                table[ta] = _sx(acc, 32); ta = (ta + 1) & 255
            else:
                gd = grid(acc & 4095) if grid else 0
                clocks += 8
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            if c == 0: t = acc < 0
            elif c == 1: t = bool(flags['GLIDE'])
            elif c == 2: t = bool(flags['PAVE'])
            else: t = True
            if arg & 8: t = not t
            if t: npc = pc + 2
        elif op == 13:
            npc = arg & 2047
        elif op == 14:
            lr = [pc + 1] + lr[:3]; npc = arg & 2047          # a four-deep link stack
        else:
            if arg == 0: npc = lr[0]; lr = lr[1:] + [lr[3]]
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
    assert len(UCODE) <= 2048, len(UCODE)
    uc = UCODE + [0xF001] * (2048 - len(UCODE))
    parts = ["-- Generated by brilliant.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package brilliant_pkg is",
             "    constant C_XOFS  : integer := %d;" % XOFS,
             "    constant C_NEXT  : integer := %d;" % NEXTRA,
             vhd_table("C_BR_TAB", 1024, 16, TAB),
             vhd_table("C_BR_UC", 2048, 16, uc, "std_logic_vector"),
             vhd_table("C_BR_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             "end package brilliant_pkg;", ""]
    with open(os.path.join(HERE, "brilliant_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
