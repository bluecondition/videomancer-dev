#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
GNOMON v0.1.0 (Straightedge) - build hook.  Writes gnomon_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.straightedge_work/rb_gnomon/imodel.py) imports this file
and runs the same microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_GN_TAB  512 x 16   the per-field engine's table: 0 .. 255 round(32768 * 2^(i/256)); 256 .. 511 a quarter sine
                       round(32768 * sin(pi/2 * j/256)) (capped 32767; address 512 wraps to entry 0 = 32768)
  C_GN_UC   1024 x 16  the per-field microprogram (PROGRAM below)
  C_GN_RFI  256 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults,
                       constants)

The per-field engine is Hardline flute's (as in Straightedge windrose): a 37-bit accumulator, a 256 x 32-bit
register file, one adder, a serial 18 x 18 signed multiplier (ACC(17:0) x operand) and a serial right shift;
micro-op = 4-bit opcode + 12-bit operand.  It runs once per field, from every vblank zone start, and writes the
field's constants (OUT TD, in target order T_*) straight into the pixel path's and the line engine's registers;
the last write kicks the line engine for line 0.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def tab_rom():
    e = [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    s = [min(int(round(32768 * math.sin(math.pi / 2 * i / 256.0))), 32767) for i in range(256)]
    return e + s


TAB = tab_rom()
DEFAULTS = dict(k1=512, k2=512, k3=512, k4=300, k5=650, k6=1023, p12=660)
A0 = 87381                                        # rest azimuth: 30 degrees of a 2^20 turn
TURN = 1 << 20
THR = (88, 120, 153)                              # camera step thresholds (8-bit level) at K6 100 %
# the engine's output targets (OUT TA 0, then OUT TD once per target, in this order)
TARGETS = ['P3', 'half3', 'h3', 'W3', 'flags', 'xst', 'Fw', 'Cu', 'lsinc', 'lsincw', 'vsi', 'vsiw', 'ysw',
           'CE0', 'CE1', 'CE2', 'CO0', 'CO1', 'CO2', 'CO3', 'E', 'Ey0', 'nK1', 'nK2', 'Psfp', 'lw3',
           'thr1', 'thr2', 'thr3', 'F00', 'F01', 'Ls00', 'Ls01', 'xh0', 'xh1',
           'ly0', 'par0', 'G0', 'Vs0', 'yh0', 'kick']
T = {n: i for i, n in enumerate(TARGETS)}

# ---------------------------------------------------------------------------------------------------------
# micro-assembler (Hardline flute's)
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, SW=6, P12=7, YS=8, XS=9, ROM=11, ZERO=15)
CONDS = dict(LT=0, GLIDE=1, STAG=2, GE=8, NGLIDE=9, NSTAG=10)
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

    def const(self, v):
        """a register holding the constant v."""
        n = 'C%d' % v if v >= 0 else 'CM%d' % (-v)
        self.reg(n, v)
        return n


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up = the TOML defaults)
    for n, v in (('h1', D['k1']), ('h2', D['k2']), ('h3k', D['k3']), ('h4', D['k4']), ('h5', D['k5']),
                 ('h6', D['k6']), ('hp', D['p12'])):
        a.reg(n, v)
    for n, v in (('a1', D['k1']), ('a2', D['k2']), ('a3', D['k3']), ('a4', D['k4']), ('a5', D['k5']),
                 ('a6', D['k6']), ('ap', D['p12'])):
        a.reg(n, v * 64)
    a.reg('th', 0)                                 # the sun's azimuth drift, 2^20 = one turn
    C = a.const

    a.JMP('main')

    # ---- SIN18: ACC = 32768 sin(pi/2 * a / 2^18), a in ACC (0 .. 2^18): 256-step table, 10-bit interpolation
    a.label('SIN18')
    a.ST('sp'); a.SHR(10); a.ST('si'); a.ADDI(256); a.OUT('ADR'); a.IN('ROM'); a.ST('s0')
    a.LD('si'); a.ADDI(257); a.OUT('ADR'); a.IN('ROM'); a.SUB('s0'); a.ST('sd')
    a.LD('si'); a.MULI(1024); a.ST('x0'); a.LD('sp'); a.SUB('x0')
    a.MUL('sd'); a.SHR(10); a.ADD('s0'); a.RET()

    # ---- RED: ACC into [-hw, hw) by adding 0 .. 31 cells, greedily (cells m0 .. m4 = 1, 2, 4, 8, 16 x the
    #      pitch); rpar = the parity of the number of cells added
    a.label('RED')
    a.ST('rv'); a.LDI(0); a.ST('rpar')
    for k in (4, 3, 2, 1, 0):
        sk = a.new()
        a.LD('rv'); a.ADD('m%d' % k); a.ST('rt'); a.SUB('hw'); a.SKIP('LT'); a.JMP(sk)
        a.LD('rt'); a.ST('rv')
        if k == 0:
            a.LDI(1); a.ST('rpar')
        a.label(sk)
    a.LD('rv'); a.RET()

    a.label('main')
    # ---- knob holds (a knob has to move 2 counts) and glides (Q6).  Glide On: 1/16 of the gap per field, never
    #      under half a count, landing exactly (the Drift speed: 1/8, landing exactly); Off: the held value.
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

    def hold(inp, h):
        u, k = a.new(), a.new()
        a.IN(inp); a.SUB(h); a.ADDI(1); a.SKIP('GE'); a.JMP(u); a.SUBI(3); a.SKIP('GE'); a.JMP(k)
        a.label(u); a.IN(inp); a.ST(h)
        a.label(k)

    a.JMP('hgx')
    a.label('HG')                                  # kin = the knob, hh = its hold, aa = its glide
    a.LD('kin'); a.SUB('hh'); a.ADDI(1); a.SKIP('GE'); a.JMP('hgu'); a.SUBI(3); a.SKIP('GE'); a.JMP('hgk')
    a.label('hgu'); a.LD('kin'); a.ST('hh')
    a.label('hgk')
    glide('hh', 'aa', 4, 32, 32)
    a.RET()
    a.label('hgx')
    for inp, h, acc in (('K1', 'h1', 'a1'), ('K2', 'h2', 'a2'), ('K3', 'h3k', 'a3'), ('K4', 'h4', 'a4'),
                        ('K6', 'h6', 'a6'), ('P12', 'hp', 'ap')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(acc); a.ST('aa'); a.CALL('HG')
        a.LD('hh'); a.ST(h); a.LD('aa'); a.ST(acc)
    hold('K5', 'h5'); glide('h5', 'a5', 3, 0, 8)

    # ---- Drift: |v| < 24 still; else 2^(-12 + ls / 64) turn per field, ls = (|v| - 24) * 0.78 (top 2^-6)
    a.LD('a5'); a.SHR(6); a.SUBI(512); a.ST('v'); a.abs_('x0'); a.SUBI(24); a.SKIP('GE'); a.JMP('still')
    a.ST('u'); a.SHR(2); a.ST('x0'); a.LD('u'); a.SUB('x0'); a.ST('ls'); a.LD('u'); a.SHR(5); a.ADD('ls')
    a.ST('ls')
    a.SHR(6); a.ST('kk'); a.MULI(64); a.ST('x0'); a.LD('ls'); a.SUB('x0'); a.MULI(4); a.OUT('ADR'); a.IN('ROM')
    a.SHR(7); a.ST('w')
    a.label('dl')
    a.LD('kk'); a.SUBI(1); a.SKIP('GE'); a.JMP('dd'); a.ST('kk'); a.LD('w'); a.ADD('w'); a.ST('w'); a.JMP('dl')
    a.label('dd')
    a.LD('v'); a.SKIP('LT'); a.JMP('dpos')
    a.LD('th'); a.SUB('w'); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(TURN)); a.JMP('dst')
    a.label('dpos')
    a.LD('th'); a.ADD('w'); a.SUB(C(TURN)); a.SKIP('LT'); a.JMP('dst'); a.ADD(C(TURN))
    a.label('dst')
    a.ST('th')
    a.label('still')

    # ---- the sun's azimuth A = th + 30 degrees; quadrant q; folded into the first quadrant: cf, sf (Q15) and
    #      the mirror flags sgx (cos < 0), sgy (sin < 0)
    a.LD('th'); a.ADD(C(A0)); a.ST('A'); a.SUB(C(TURN)); a.SKIP('LT'); a.ST('A')
    a.LD('A'); a.SHR(18); a.ST('q'); a.MULI(1024); a.MULI(256); a.ST('x0'); a.LD('A'); a.SUB('x0'); a.ST('aq')
    a.CALL('SIN18'); a.ST('sa')
    a.LD(C(1 << 18)); a.SUB('aq'); a.CALL('SIN18'); a.ST('ca')
    a.LD('q'); a.SHR(1); a.ST('sgy'); a.MULI(2); a.ST('x0'); a.LD('q'); a.SUB('x0'); a.ST('q0')
    qe = a.new(); qd = a.new()
    a.SUBI(1); a.SKIP('LT'); a.JMP(qe)
    a.LD('ca'); a.ST('cf'); a.LD('sa'); a.ST('sf'); a.JMP(qd)
    a.label(qe)
    a.LD('sa'); a.ST('cf'); a.LD('ca'); a.ST('sf')
    a.label(qd)
    a.LD('q0'); a.SUB('sgy'); a.abs_('x0'); a.ST('sgx')         # sgx = q0 xor sgy (q = 1, 2)

    # ---- mode: x step (Q3 per pixel), pixel 0's x (Q3 from the centre, screen column 3), line step (Q8)
    a.IN('XS'); a.MULI(4); a.ST('xst'); a.IN('XS'); a.SUBI(3); m3, m4, mx = a.new(), a.new(), a.new()
    a.SKIP('LT'); a.JMP(m3)
    a.LD(C(-7652)); a.ST('xp0'); a.JMP(mx)
    a.label(m3)
    a.SUBI(1); a.SKIP('LT'); a.JMP(m4)
    a.LD(C(-7638)); a.ST('xp0'); a.JMP(mx)
    a.label(m4)
    a.LD(C(-5704)); a.ST('xp0')
    a.label(mx)
    a.IN('YS'); a.ST('ys')

    # ---- the pitch P = 50 2^(oct / 256) virtual px (Q3, even), oct = 3 g3 / 32 (3 octaves over K3)
    a.LD('a3'); a.MULI(3); a.SHR(8); a.ST('oct'); a.SHR(8); a.ST('oh'); a.MULI(256); a.ST('x0')
    a.LD('oct'); a.SUB('x0'); a.OUT('ADR'); a.IN('ROM'); a.MULI(400); a.SHR(13); a.ST('x1')
    pd = a.new()
    a.LD('oh'); a.SUBI(2); a.SKIP('LT'); a.JMP(pd)
    a.LD('x1'); a.SHR(1); a.ST('x1'); a.LD('oh'); a.SUBI(1); a.SKIP('LT'); a.JMP(pd)
    a.LD('x1'); a.SHR(1); a.ST('x1')
    a.label(pd)
    a.LD('x1'); a.SHR(1); a.ST('half3'); a.MULI(2); a.ST('P3'); a.MULI(32); a.ST('P8')
    a.LD('half3'); a.MULI(32); a.ST('hw8')
    # ---- the post's half size h = P (0.04 .. 0.45), Q16: 2622 + g4 105 / 32;  the shadow L P, L = 0.15 .. 2
    a.LD('a4'); a.SHR(3); a.MULI(105); a.SHR(5); a.ADD(C(2622)); a.MUL('P3'); a.SHR(16); a.ST('h3')
    a.LD('ap'); a.SHR(3); a.MULI(949); a.SHR(10); a.ADD(C(614)); a.MUL('P3'); a.SHR(12); a.ST('LP')
    # ---- products
    a.LD('P3'); a.MUL('sf'); a.SHR(15); a.ST('Psf3')
    a.LD('P3'); a.MUL('cf'); a.SHR(15); a.ST('Pcf3')
    a.LD('cf'); a.ADD('sf'); a.MUL('h3'); a.SHR(15); a.ST('W3')
    a.LD('LP'); a.MUL('cf'); a.SHR(15); a.ADD('h3'); a.ADD('h3'); a.ST('E')
    a.LD('LP'); a.MUL('sf'); a.SHR(15); a.ADD('h3'); a.ADD('h3'); a.ST('Ey0')
    # ---- the beam: F steps xst (xst - P at a crossing), u' = Cu + sgx F, lx sf (Q3.10)
    a.LD('xst'); a.SUB('P3'); a.ST('Fw'); a.ADD('half3'); a.ST('Kf')
    cu = a.new()
    a.LD('sgx'); a.SUBI(1); a.SKIP('LT'); a.JMP(cu)                # Cu = -sgx Kf
    a.LDI(0); a.SUB('Kf'); a.ST('Cu'); a.JMP(cu + 'x')
    a.label(cu)
    a.LD('Kf'); a.ST('Cu')
    a.label(cu + 'x')
    a.LD('xst'); a.MUL('sf'); a.SHR(5); a.ST('lsinc')
    a.LD('Fw'); a.MUL('sf'); a.SHR(5); a.ST('lsincw')
    # ---- the line engine: ly steps ys (ys - P at a row crossing), ly cf (Q8.12)
    a.LD('ys'); a.SUB('P8'); a.ST('ysw'); a.MUL('cf'); a.SHR(3); a.ST('vsiw')
    a.LD('ys'); a.MUL('cf'); a.SHR(3); a.ST('vsi')
    # ---- pan: +-1.25 cells (Y up = up)
    a.LD('P3'); a.MULI(5); a.ST('P5')
    a.LD('a1'); a.SHR(3); a.ST('x0'); a.LD(C(4096)); a.SUB('x0'); a.MUL('P5'); a.SHR(14); a.ST('ox3')
    a.LD('a2'); a.SHR(3); a.SUB(C(4096)); a.MUL('P5'); a.SHR(9); a.ST('oy8')
    # ---- line 0's local y (Q8) into its row, and the row's parity
    a.LD('P8'); a.ST('m0'); a.ADD('m0'); a.ST('m1'); a.ADD('m1'); a.ST('m2'); a.ADD('m2'); a.ST('m3')
    a.ADD('m3'); a.ST('m4'); a.LD('hw8'); a.ST('hw')
    a.LD('ys'); a.SHR(1); a.SUB(C(138240)); a.ADD('oy8'); a.CALL('RED'); a.ST('ly0'); a.LD('rpar'); a.ST('par0')
    # ---- pixel 0's local x (Q3), even rows and odd rows (Stagger: half a cell over)
    a.LD('P3'); a.ST('m0'); a.ADD('m0'); a.ST('m1'); a.ADD('m1'); a.ST('m2'); a.ADD('m2'); a.ST('m3')
    a.ADD('m3'); a.ST('m4'); a.LD('half3'); a.ST('hw')
    a.LD('xp0'); a.ADD('ox3'); a.ST('lxr'); a.CALL('RED'); a.ST('lx0')
    a.LD('lxr'); a.SKIP('NSTAG'); a.SUB('half3'); a.CALL('RED'); a.ST('lx1')
    # ---- per variant: F at pixel 0, lx sf, the home post's x (Q3 from the left edge)
    for v in (0, 1):
        a.LD('lx%d' % v); a.ADD('Kf'); a.ST('F0%d' % v)
        a.LD('lx%d' % v); a.MUL('sf'); a.SHR(5); a.ST('Ls0%d' % v)
        a.LD('xp0'); a.ADD(C(7680)); a.SUB('lx%d' % v); a.ST('xh%d' % v)
    # ---- line 0: ly cf, G (the next line's distance past the row's edge), the row's centre (Q8 from the top)
    a.LD('ly0'); a.MUL('cf'); a.SHR(3); a.ST('Vs0')
    a.LD('ly0'); a.ADD('ysw'); a.ADD('hw8'); a.ST('G0')
    a.LD('ys'); a.SHR(1); a.SUB('ly0'); a.ST('yh0')
    # ---- column constants i P + h; the Stagger middle row's half a cell less
    a.LD('h3'); a.ST('CE0'); a.ADD('P3'); a.ST('CE1'); a.ADD('P3'); a.ST('CE2')
    a.LD('h3'); a.SKIP('NSTAG'); a.SUB('half3'); a.ST('CO0'); a.ADD('P3'); a.ST('CO1'); a.ADD('P3'); a.ST('CO2')
    a.ADD('P3'); a.ST('CO3')
    # ---- the rows' cross offsets, signed by sgx: nK1 = -sgx (P cf + P sf / 2 in Stagger), nK2 = -sgx 2 P cf;
    #      Psfp = sgx P sf
    a.LD('Psf3'); a.SHR(1); a.ST('x0'); a.LD('Pcf3'); a.SKIP('NSTAG'); a.ADD('x0'); a.ST('k1')
    a.LD('Pcf3'); a.ADD('Pcf3'); a.ST('k2')
    kp, kd = a.new(), a.new()
    a.LD('sgx'); a.SUBI(1); a.SKIP('LT'); a.JMP(kp)
    a.LDI(0); a.SUB('k1'); a.ST('nK1'); a.LDI(0); a.SUB('k2'); a.ST('nK2'); a.LD('Psf3'); a.ST('Psfp'); a.JMP(kd)
    a.label(kp)
    a.LD('k1'); a.ST('nK1'); a.LD('k2'); a.ST('nK2'); a.LDI(0); a.SUB('Psf3'); a.ST('Psfp')
    a.label(kd)
    # ---- camera thresholds: 88, 120, 153 + (8191 - g6) 17 / 1024 (511 when K6 is 0); Fine's line weight
    a.LD('a6'); a.SHR(3); a.ST('g6'); a.SUBI(8); t0, td = a.new(), a.new()
    a.SKIP('LT'); a.JMP(t0)
    a.LD(C(511)); a.ST('thr1'); a.ST('thr2'); a.ST('thr3'); a.JMP(td)
    a.label(t0)
    a.LD(C(8191)); a.SUB('g6'); a.MULI(17); a.SHR(10); a.ST('x0')
    a.ADDI(THR[0]); a.ST('thr1'); a.LD('x0'); a.ADDI(THR[1]); a.ST('thr2'); a.LD('x0'); a.ADDI(THR[2]); a.ST('thr3')
    a.label(td)
    lw1, lw2 = a.new(), a.new()
    a.LD('h3'); a.SHR(2); a.ST('lw3'); a.SUBI(16); a.SKIP('LT'); a.JMP(lw1); a.LDI(16); a.ST('lw3')
    a.label(lw1)
    a.LD('lw3'); a.SUBI(48); a.SKIP('GE'); a.JMP(lw2); a.LDI(48); a.ST('lw3')
    a.label(lw2)
    a.LD('sgy'); a.ADD('sgy'); a.ADD('sgx'); a.ST('flags')
    a.LDI(1); a.ST('kick')
    # ---- out, in target order (the last one kicks the line engine)
    a.LDI(0); a.OUT('TA')
    for n in TARGETS:
        a.LD(n); a.OUT('TD')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 256
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & 0xFFFFFFFF


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; gnomon.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000, check=False):
    """inp: dict over INS names (K1 .. K6, P12 raw knobs, YS, XS); flags: GLIDE, STAG.
    rf: the register file (list of 256 ints, updated in place; None = the power-up file).
    Returns (out: dict target name -> signed 32-bit word, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ta = 0
    out = {}
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
                if ta < len(TARGETS):
                    out[TARGETS[ta]] = _sx(acc, 32)
                ta = (ta + 1) & 255
        elif op == 11:
            acc >>= (arg & 31)
            clocks += (arg & 31) - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'STAG'}[c]])
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
    return out, rf, clocks


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
    parts = ["-- Generated by gnomon.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package gnomon_pkg is",
             vhd_table("C_GN_TAB", 512, 16, TAB),
             vhd_table("C_GN_UC", 1024, 16, uc, "std_logic_vector"),
             vhd_table("C_GN_RFI", 256, 32, RF_INIT, "std_logic_vector"),
             "end package gnomon_pkg;", ""]
    with open(os.path.join(HERE, "gnomon_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
