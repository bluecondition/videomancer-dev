#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
COROLLA v0.1.0 (Vellum) - build hook.  Writes corolla_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.vellum_work/rb_corolla/imodel.py) imports this file and runs the
same microprogram through emulate(), so the per-field arithmetic has one source.

The per-field engine is Hardline flute's (flute.vhd / flute.py): a 37-bit accumulator, a 256 x 32-bit register
file in block RAM, one adder, a serial 18 x 18 signed multiplier and a serial right shift; micro-op = 4-bit
opcode + 12-bit operand; three clocks per op, a multiply 20, a shift 2 + n.  It is started from address 0 at
every vblank-zone start.  Here it does the knob holds and glides, the Drift speed (Hypnos), the glide phase and
the turn, the pitch, the turn's sine / cosine, the oval, every DDA step and start value, the disc radius, the
edge scale G and the camera's table constants.

Package contents (block-RAM ROMs unless noted):
  C_CO_FT    1024 x 16  the engine's tables: 0 .. 256 a quarter sine (32768 sin(i pi / 512)), 257 .. 512 exp
                        (32768 2^(i/256)), 513 .. 768 1 / R(bloom) (Q14, by the bloom glide's top byte),
                        769 .. 776 48 v-px in Q6 AA px (hex, square x AA class), 777 .. 786 2^0 .. 2^9
  C_CO_UC    1024 x 16  the per-field microprogram (see PROGRAM below)
  C_CO_RFI   256 x 32   the engine's register file at power-up (holds and glides at the TOML defaults, constants)
  C_CO_TONE  64 x 10    tone6 -> luma code 64 + round(tone6 * 876 / 63)
  C_CO_COLH  12 x 8     the hex candidates' colour offsets (lattice 3-colouring), 2 bits per fold (mu, mw)
  C_CO_COLS  12 x 4     the square candidates' colour offsets (checkerboard), 1 bit per fold (mu, mw)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# candidates in the folded quarter cell: hex (cx pitches, cr rows), square (cx, cw spacings)
CAND_H = [(0, 0), (0.5, 1), (1, 0), (-0.5, 1), (0, 2), (1, 2), (0.5, -1), (-0.5, -1), (-1, 0), (1.5, 1),
          (-1, 2), (1.5, -1)]
CAND_S = {0: (0, 0), 2: (1, 0), 4: (0, 1), 5: (1, 1), 6: (0, -1), 8: (-1, 0)}

KAPQ = 8618          # camera: 0.45 pitch of radius per full luma swing at K6 100 %
DEFAULTS = dict(k1=512, k2=512, k3=740, k4=0, k5=650, k6=1023, p12=716)


def bloom_q15(gT):
    """R / pitch (Q15) by the bloom glide (Q10.6): 0.35 .. 1.0 rising, a detent at 1.0, 1.0 .. 1.3."""
    if gT <= 44564:
        return 11469 + ((gT * 31323) >> 16)
    if gT >= 47186:
        return 32768 + (((gT - 47186) * 35110) >> 16)
    return 32768


# ---------------------------------------------------------------------------------------------------------
# the engine's table ROM
# ---------------------------------------------------------------------------------------------------------
SINB, EXPB, INVRB, KPAB, P2B = 0, 257, 513, 769, 777
FT = [int(round(32768 * math.sin(i * math.pi / 512.0))) for i in range(257)]
FT += [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
FT += [int(round(16384 * 32768.0 / bloom_q15(i * 256))) for i in range(256)]
FT += [6144, 4096, 3072, 2048, 8689, 5793, 4344, 2896]
FT += [1 << k for k in range(10)]
FT += [0] * (1024 - len(FT))

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, XS=8, YS=9, WC=10, ROM=11, CNT=12, LM=13, DXM=14)
CONDS = dict(LT=0, GLIDE=1, SQ=2, FINE=3, GE=8, NGLIDE=9, NSQ=10, NFINE=11)
OUTS = dict(ADR=0, SA=1, SB=2, TA=3, TB=4, U0=5, W0=6, CQ=7, KE2=8, KW1=9, KW2=10, HB=11, T0=12, GS=13,
            H2S=14, ACS=15)


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
        return self.reg("c%d" % v, v)

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

    def LDC(self, v):
        """acc = v (a 12-bit immediate, or a register-file constant)"""
        if -2048 <= v <= 2047:
            self.LDI(v)
        else:
            self.emit("LD", self.const(v))

    def MULC(self, v):
        if -2048 <= v <= 2047:
            self.MULI(v)
        else:
            self.emit("MUL", self.const(v))

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

    def rom(self, base):
        """acc = FT[base + acc]"""
        self.ADDI(base); self.OUT('ADR'); self.IN('ROM')

    def sel(self, cond, a, b):
        """acc = a if cond else b (constants)"""
        d, e = self.new(), self.new()
        self.SKIP(cond); self.JMP(e); self.LDC(a); self.JMP(d)
        self.label(e); self.LDC(b)
        self.label(d)


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults)
    for n, k in (('h1', 'k1'), ('h2', 'k2'), ('h3', 'k3'), ('h4', 'k4'), ('h6', 'k6'), ('hp', 'p12')):
        a.reg(n, D[k])
    for n, k in (('a1', 'k1'), ('a2', 'k2'), ('a3', 'k3'), ('a4', 'k4'), ('a6', 'k6'), ('ap', 'p12')):
        a.reg(n, D[k] * 64)
    a.reg('vgs', (D['k5'] - 512) * 64)
    a.reg('phu', 0); a.reg('th', 0)

    a.JMP('main')

    # ---- INTERP: ACC = T[ix] + ((T[ix + 1] - T[ix]) * fx >> 8)        (fx: 8 fraction bits)
    a.label('INTERP')
    a.LD('ix'); a.OUT('ADR'); a.IN('ROM'); a.ST('v0'); a.LD('ix'); a.ADDI(1); a.OUT('ADR'); a.IN('ROM')
    a.SUB('v0'); a.ST('dv'); a.LD('fx'); a.MUL('dv'); a.SHR(8); a.ADD('v0'); a.RET()

    # ---- SINE: ACC = 32768 sin(ang * 90 / 32768 degrees), ang = ACC in 0 .. 32768
    a.label('SINE')
    a.ST('t5'); a.SHR(7); a.ST('ix'); a.MULI(128); a.ST('t4'); a.LD('t5'); a.SUB('t4'); a.MULI(2); a.ST('fx')
    a.LD('ix'); a.ADDI(SINB); a.ST('ix'); a.JMP('INTERP')

    # ---- SHN: ACC >>= shn (0 .. 3), arithmetic
    a.label('SHN')
    a.ST('t5'); a.LD('shn'); a.SUBI(1); a.SKIP('GE'); a.JMP('sh0'); a.SUBI(1); a.SKIP('GE'); a.JMP('sh1')
    a.SUBI(1); a.SKIP('GE'); a.JMP('sh2'); a.LD('t5'); a.SHR(3); a.RET()
    a.label('sh2'); a.LD('t5'); a.SHR(2); a.RET()
    a.label('sh1'); a.LD('t5'); a.SHR(1); a.RET()
    a.label('sh0'); a.LD('t5'); a.RET()

    # ---- WRAP: ACC modulo the period in 'per' (one period at most outside 0 .. per - 1)
    a.label('WRAP')
    a.SKIP('GE'); a.ADD('per'); a.ST('t5'); a.SUB('per'); a.SKIP('LT'); a.RET(); a.LD('t5'); a.RET()

    a.label('main')

    # ---- knob holds (a knob has to move more than one count) and glides (Q6): 1/16 of the gap per field,
    #      never slower than one count, landing exactly; S7 Off = this field's held value, no easing.
    #      HG works on raw / h / ga; each channel copies its state in and out.
    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3', 'a3'), ('K4', 'h4', 'a4'),
                        ('K6', 'h6', 'a6'), ('P12', 'hp', 'ap')):
        a.IN(inp); a.ST('raw'); a.LD(h); a.ST('h'); a.LD(acc); a.ST('ga'); a.CALL('HG')
        a.LD('h'); a.ST(h); a.LD('ga'); a.ST(acc)
    a.JMP('pitch')

    a.label('HG')
    u, k = a.new(), a.new()
    a.LD('raw'); a.SUB('h'); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
    a.label(u); a.LD('raw'); a.ST('h')
    a.label(k)
    snap, big, pos, add = a.new(), a.new(), a.new(), a.new()
    a.LD('h'); a.MULI(64); a.ST('tg'); a.SKIP('GLIDE'); a.JMP(snap)
    a.SUB('ga'); a.ST('td'); a.abs_('t0'); a.SUBI(65); a.SKIP('GE'); a.JMP(snap)
    a.SUBI(959); a.SKIP('LT'); a.JMP(big)
    a.LD('td'); a.SKIP('LT'); a.JMP(pos); a.LDI(-64); a.JMP(add)
    a.label(pos); a.LDI(64)
    a.label(add); a.ADD('ga'); a.ST('ga'); a.RET()
    a.label(big); a.LD('td'); a.SHR(4); a.ADD('ga'); a.ST('ga'); a.RET()
    a.label(snap); a.LD('tg'); a.ST('ga'); a.RET()

    a.label('pitch')
    # ---- PITCH: oct = Scale * 768 / 65536 (Q8 octaves above 48 v-px); pa = the lattice unit in Q6 AA px;
    #      2^-oct = minv 2^-shn / 32768
    m0, m1 = a.new(), a.new()
    a.LD('a3'); a.MULI(768); a.SHR(16); a.ST('oct'); a.SHR(8); a.ST('shp'); a.MULI(256); a.ST('t0')
    a.LD('oct'); a.SUB('t0'); a.ST('frac'); a.rom(EXPB); a.ST('e1')
    a.IN('WC'); a.ST('t0'); a.LDI(0); a.SKIP('NSQ'); a.LDI(4); a.ADD('t0'); a.rom(KPAB); a.ST('t1')
    a.LD('shp'); a.rom(P2B); a.MUL('t1'); a.ST('t1'); a.LD('e1'); a.MUL('t1'); a.SHR(16); a.ST('pa')
    a.LD('frac'); a.SUBI(1); a.SKIP('GE'); a.JMP(m0)
    a.LDI(256); a.SUB('frac'); a.rom(EXPB); a.ST('minv'); a.LD('shp'); a.ADDI(1); a.ST('shn'); a.JMP(m1)
    a.label(m0); a.LDC(32768); a.ST('minv'); a.LD('shp'); a.ST('shn')
    a.label(m1)
    a.sel('SQ', 15449, 14564); a.MUL('minv'); a.SHR(15); a.ST('kmu')               # Q22 u span per h, x 2^shn
    a.sel('SQ', 15449, 25225); a.MUL('minv'); a.SHR(15); a.ST('kmw')

    # ---- BLOOM: R / pitch (Q15) with a detent at 1; R in lattice units; G = 8 pa / R (edge units, 1/16 px,
    #      per lattice unit^2) from the 1 / R table
    b0, b1, b2 = a.new(), a.new(), a.new()
    a.LD('ap'); a.emit("SUB", a.const(44565)); a.SKIP('GE'); a.JMP(b0)
    a.emit("SUB", a.const(47186 - 44565)); a.SKIP('GE'); a.JMP(b1)
    a.MULC(35110); a.SHR(16); a.emit("ADD", a.const(32768)); a.JMP(b2)
    a.label(b1); a.LDC(32768); a.JMP(b2)
    a.label(b0); a.LD('ap'); a.MULC(31323); a.SHR(16); a.emit("ADD", a.const(11469))
    a.label(b2); a.ST('rp')
    a.LD('ap'); a.SHR(8); a.rom(INVRB); a.ST('ivr')
    r0_, r1_ = a.new(), a.new()
    a.SKIP('SQ'); a.JMP(r0_)
    a.LD('rp'); a.MULC(23170); a.SHR(15); a.ST('rp'); a.LD('ivr'); a.MULC(46341); a.SHR(15); a.ST('ivr')
    a.label(r0_)
    a.LD('pa'); a.MUL('ivr'); a.SHR(17); a.ST('g'); a.OUT('CQ')
    a.MULI(9); a.SHR(2); a.OUT('KE2')
    k0, k1 = a.new(), a.new()
    a.LD('g'); a.SKIP('SQ'); a.JMP(k0); a.OUT('KW1'); a.OUT('KW2'); a.JMP(k1)
    a.label(k0); a.MULI(3); a.ST('t0'); a.SHR(2); a.OUT('KW1'); a.LD('t0'); a.OUT('KW2')
    a.label(k1)

    # ---- the disc's scaled r^2 (+ h^2), the radius cap, h^2, the ring half width (Fine: P / 28)
    f0, f1 = a.new(), a.new()
    a.LDI(0); a.ST('hl2'); a.OUT('HB'); a.SKIP('FINE'); a.JMP(f0)
    a.sel('SQ', 42, 84); a.ST('hl2'); a.sel('SQ', 414, 585); a.MUL('pa'); a.SHR(16); a.OUT('HB')
    a.label(f0)
    a.LD('rp'); a.MUL('rp'); a.SHR(14); a.ADD('hl2'); a.MUL('g'); a.SHR(16); a.ST('a0s')
    a.sel('SQ', 68854, 110756); a.ADD('hl2'); a.MUL('g'); a.SHR(16); a.OUT('ACS')
    a.LD('hl2'); a.MUL('g'); a.SHR(16); a.OUT('H2S')

    # ---- CAMERA: the radius moves about the field's mean level, 0.33 pitch per full swing at K6 100 %:
    #      T(L) = T0 + L Gs (Q8), T0 = A0s 256 - Lm Gs
    a.LD('pa'); a.MUL('a6'); a.SHR(16); a.MULC(KAPQ); a.SHR(16); a.ST('gs'); a.OUT('GS')
    a.IN('LM'); a.MUL('gs'); a.ST('t0'); a.LD('a0s'); a.MULI(256); a.SUB('t0'); a.OUT('T0')
    a.label('drift')
    # ---- DRIFT (Hypnos v1.1.0 SPEED): glide 1/8 (Off: this field's knob), |v| < 24 still, else
    #      2^-12 .. 2^-3 pitch per field, exponential; the glide phase (Q22 of the u span) and the turn (Q20 of
    #      60 / 90 degrees, 1/32 of the glide in pitches) advance once per field
    vs, vz, vn, sdone = a.new(), a.new(), a.new(), a.new()
    a.IN('K5'); a.SUBI(512); a.MULI(64); a.ST('t0'); a.SKIP('GLIDE'); a.JMP(vs)
    a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('t0')
    a.label(vs); a.LD('t0'); a.ST('vgs')
    a.SHR(6); a.ST('sv'); a.abs_('t0'); a.SUBI(24); a.SKIP('GE'); a.JMP(vz)
    a.ST('su'); a.SHR(3); a.ST('t0'); a.LD('su'); a.SHR(4); a.ADD('t0'); a.ADD('su'); a.ST('sls')   # ls
    a.SHR(6); a.ST('soc'); a.MULI(64); a.ST('t0'); a.LD('sls'); a.SUB('t0'); a.MULI(4); a.rom(EXPB)
    a.SHR(7); a.ST('smt')                                                        # mantissa 256 .. 511
    a.LD('soc'); a.rom(P2B); a.ST('sp2')                                         # 2^octave
    a.LD('smt'); a.MUL('sp2'); a.SHR(5); a.ST('srot')                            # turn step, Q20
    a.sel('SQ', 46341, 43691); a.MUL('smt'); a.SHR(15); a.MUL('sp2'); a.ST('sspu')   # glide step, Q22
    a.JMP(sdone)
    a.label(vz); a.LDI(0); a.ST('srot'); a.ST('sspu')
    a.label(sdone)
    a.LDC(1 << 22); a.ST('per')
    a.LD('sv'); a.SKIP('LT'); a.JMP(vn)
    a.LD('phu'); a.SUB('sspu'); a.CALL('WRAP'); a.ST('phu')
    a.LDC(1 << 20); a.ST('per'); a.LD('th'); a.SUB('srot'); a.CALL('WRAP'); a.ST('th'); a.JMP('turn')
    a.label(vn)
    a.LD('phu'); a.ADD('sspu'); a.CALL('WRAP'); a.ST('phu')
    a.LDC(1 << 20); a.ST('per'); a.LD('th'); a.ADD('srot'); a.CALL('WRAP'); a.ST('th')

    a.label('turn')
    # ---- TURN: ang = th * (2/3 hex, 1 square) in Q15 of 90 degrees; sin, cos (Q15)
    a.LD('th'); a.SHR(4); a.ST('t0'); a.sel('SQ', 32768, 21845); a.MUL('t0'); a.SHR(16); a.ST('ang')
    a.CALL('SINE'); a.ST('sn'); a.LDC(32768); a.SUB('ang'); a.CALL('SINE'); a.ST('cs')

    # ---- OVAL: io = 2^(-o / 256), o = Oval's top byte (horizontal stretch 1 .. 2)
    o0, o1 = a.new(), a.new()
    a.LD('a4'); a.SHR(8); a.ST('t0'); a.SUBI(1); a.SKIP('GE'); a.JMP(o0)
    a.LDI(256); a.SUB('t0'); a.rom(EXPB); a.SHR(1); a.JMP(o1)
    a.label(o0); a.LDC(32768)
    a.label(o1); a.ST('io')
    a.LD('cs'); a.MUL('io'); a.SHR(15); a.ST('cio'); a.LD('sn'); a.MUL('io'); a.SHR(15); a.ST('sio')

    # ---- DDA STEPS (Q22 of the span): per pixel u + SA, w - SB; per line u + TA, w + TB
    a.IN('XS'); a.ST('t0'); a.MUL('cio'); a.SHR(2); a.MUL('kmu'); a.SHR(13); a.CALL('SHN'); a.OUT('SA')
    a.LD('t0'); a.MUL('sio'); a.SHR(2); a.MUL('kmw'); a.SHR(13); a.CALL('SHN'); a.OUT('SB')
    a.IN('YS'); a.ST('t0'); a.MUL('sn'); a.SHR(9); a.MUL('kmu'); a.SHR(13); a.CALL('SHN'); a.OUT('TA')
    a.LD('t0'); a.MUL('cs'); a.SHR(9); a.MUL('kmw'); a.SHR(13); a.CALL('SHN'); a.OUT('TB')

    # ---- START VALUES at line 0's pre-pixel, relative to the lattice origin (centre + pan +-512 v-px), in h:
    #      A = -dxm - pan x, B = -1080 - pan y;  u0 = (A cio + B sin) km_u + phase,  w0 = (B cos - A sio) km_w
    a.LD('a1'); a.emit("SUB", a.const(32768)); a.SHR(5); a.ST('t0'); a.IN('DXM'); a.ADD('t0'); a.ST('t0'); a.LDI(0)
    a.SUB('t0'); a.ST('ax')
    a.LD('a2'); a.emit("SUB", a.const(32768)); a.SHR(5); a.ADDI(1080); a.ST('t0'); a.LDI(0); a.SUB('t0')
    a.ST('by')
    a.LD('ax'); a.MUL('cio'); a.ST('t0'); a.LD('by'); a.MUL('sn'); a.ADD('t0'); a.SHR(11); a.MUL('kmu')
    a.SHR(4); a.CALL('SHN'); a.ADD('phu'); a.OUT('U0')
    a.LD('by'); a.MUL('cs'); a.ST('t0'); a.LD('ax'); a.MUL('sio'); a.ST('t1'); a.LD('t0'); a.SUB('t1')
    a.SHR(11); a.MUL('kmw'); a.SHR(4); a.CALL('SHN'); a.OUT('W0')

    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; corolla.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


OUT_CLK = {}


def emulate(inp, flags, rf=None, max_instr=100000):
    """inp: dict over INS names (K1.. raw knobs, XS, YS, WC, LM, DXM); flags: GLIDE, SQ, FINE.
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
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 17
        elif op == 8:
            rf[arg & 255] = acc & 0xFFFFFFFF
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            elif n == INS['CNT']: acc = cnt
            else: acc = inp[inv[n]] & 0xFFFF
        elif op == 10:
            n = arg & 31
            if n == 0: adr = acc & 1023
            outs[n] = acc
            OUT_CLK[n] = clocks
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'SQ', 3: 'FINE'}[c]])
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


# ---------------------------------------------------------------------------------------------------------
# pixel-path tables
# ---------------------------------------------------------------------------------------------------------
def tone_rom():
    return [64 + int(round(t * 876 / 63.0)) for t in range(64)]


def col_hex(k, mu, mw):
    """colour (i - j) mod 3 of candidate k, as an offset added to the u-cell index."""
    cx, cr = CAND_H[k]
    x = (1 - cx) if mu else cx
    j = (2 - cr) if mw else cr
    return int(round(x - 1.5 * j)) % 3


def col_sq(k, mu, mw):
    if k not in CAND_S:
        return 0
    cx, cw = CAND_S[k]
    x = (1 - cx) if mu else cx
    y = (1 - cw) if mw else cw
    return int(x + y) % 2


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['to_unsigned(%d, %d)' % (v, width) for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def vhd_hex(name, depth, width, vals):
    hexw = (width + 3) // 4
    mask = (1 << (hexw * 4)) - 1
    lines = ["    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % ("t_" + name.lower(), depth - 1, hexw * 4 - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (hexw, v & mask) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def vhd_slv_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of std_logic_vector(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['"%s"' % format(v, "0%db" % width) for v in vals]
    for i in range(0, len(items), 6):
        lines.append("        " + ", ".join(items[i:i + 6]) + ("," if i + 6 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    assert len(UCODE) <= 1024, len(UCODE)
    assert max(FT) <= 65535
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    colh = []
    cols = []
    for k in range(12):
        wh = 0
        ws = 0
        for f in range(4):
            wh |= col_hex(k, f >> 1, f & 1) << (2 * f)
            ws |= col_sq(k, f >> 1, f & 1) << f
        colh.append(wh)
        cols.append(ws)
    parts = ["-- Generated by corolla.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package corolla_pkg is",
             vhd_hex("C_CO_FT", 1024, 16, FT),
             vhd_hex("C_CO_UC", 1024, 16, uc),
             vhd_hex("C_CO_RFI", 256, 32, RF_INIT),
             vhd_table("C_CO_TONE", "unsigned", 64, 10, tone_rom()),
             vhd_slv_table("C_CO_COLH", 12, 8, colh),
             vhd_slv_table("C_CO_COLS", 12, 4, cols),
             "end package corolla_pkg;", ""]
    with open(os.path.join(HERE, "corolla_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
