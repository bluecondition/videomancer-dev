#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TANS v0.1.0 (Straightedge) - build hook.  Writes tans_pkg.vhd next to this file (pure python: the builder runs it
with the system python3).  The integer model (.straightedge_work/rb_tans/imodel.py) imports this file and runs the
same per-field microprogram through emulate(), so the per-field arithmetic has one source.

Package contents (block-RAM ROMs):
  C_TN_SIN   256 x 16   round(4096 sin(i pi / 512)), a quarter sine (the hardware sine unit; entry 256 = 4096 is
                        made in fabric)
  C_TN_FT   1024 x 16   the engine's tables: exp 2^(i/256) (Q15), 2^0 .. 2^15, the mode table, the eight figures
                        (per piece x, y (Q12 units), angle k * 8192, turn to the next figure dk * 4096; the figure's
                        scale, Q14) and the scatter constants per piece (turn, slot x, slot y)
  C_TN_UC   1024 x 16   the per-field microprogram (PROGRAM below)
  C_TN_RFI    64 x 32   the engine's register file at power-up (knob holds and glides at the TOML defaults)

The per-field engine (after Vellum cassini / Hardline flute): a 38-bit accumulator, a 64 x 32-bit register file,
one adder, a radix-4 Booth serial multiplier (18 x 18 signed, 9 steps) and a right shift in 2-bit steps; micro-op = 4-bit opcode + 12-bit operand.
It writes the 23 edge functions (pixel step, line wrap, field seed) into the edge table through OUT ED.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------------------------------------
# the eight figures: (scale Q14, [(x Q12 units, y Q12 units, angle k (45 deg), turn to the next figure)] x 7)
# piece order LA LB MT SA SB SQ PG; poses are centroids about the figure's bounding-box centre (y down)
# ---------------------------------------------------------------------------------------------------------
FIG_NAMES = ["square", "cat", "house", "boat", "swan", "fish", "arrow", "pyramid"]
FIGS = [
    (16384, [(0, -5461, 0, 4), (-5461, 0, 6, -4), (5461, 5461, 7, 2), (6827, -4096, 2, -4), (0, 2731, 4, -2), (4096, 0, 1, 0), (-2048, 6144, 0, 0)]),
    (10923, [(-2048, 9557, 4, 0), (3413, 4096, 2, -2), (8875, 9557, 1, 0), (-683, -8192, 6, -4), (4779, -8192, 2, 2), (2048, -4096, 1, 1), (-8192, 6144, 0, 2)]),
    (13107, [(-2896, -683, 4, 3), (-2896, 4779, 0, 0), (-8358, 7509, 1, 4), (3930, 6144, 2, -4), (1200, 8875, 4, 0), (8192, 7344, 0, 0), (3248, -4096, 2, 1)]),
    (10498, [(-3862, 731, 7, 1), (4096, 7324, 0, -4), (-6827, 7324, 5, 4), (-2731, 8689, 6, 2), (0, 11420, 4, -2), (2896, 1697, 0, 1), (2896, -6992, 3, -1)]),
    (13107, [(-6144, 4779, 0, 4), (2048, 7509, 4, -4), (-11605, -683, 1, 1), (10240, -8875, 0, 0), (8875, 6144, 2, 2), (6144, 2048, 1, 0), (8192, -4096, 2, 0)]),
    (9362, [(-2896, -683, 4, -3), (-2896, 4779, 0, 3), (9157, 2048, 2, 0), (5296, -683, 0, 1), (5296, 4779, 4, 1), (1200, 10240, 1, 1), (-848, -8192, 2, -1)]),
    (11585, [(9654, -3862, 1, 3), (9654, 3862, 3, -1), (-13516, 0, 2, -1), (-9654, 965, 1, -3), (-1931, -965, 5, -1), (2896, 0, 0, 1), (-5793, 0, 1, 1)]),
    (14564, [(-8192, 5461, 4, 4), (-2731, 0, 2, -4), (2731, 5461, 1, -2), (5461, 0, 6, -4), (12288, 6827, 4, 0), (8192, 4096, 1, 0), (2048, -2048, 2, 2)]),
]
# scatter at P12 = 100 %: per piece the turn (45-degree units) and the slot (virtual px about the screen centre)
SC_TURN = [-1.5, 1.0, 2.0, -1.0, 3.0, -2.0, 1.5]
SC_SLOT = [(-560, -170), (560, 190), (-150, 300), (760, -330), (-780, 330), (220, -260), (40, 40)]

# the 23 edges: (piece, direction j in 45-degree steps relative to the piece, offset d in Q11 units)
_D3, _D47, _D5, _D67, _D71, _D94 = 683, 965, 1024, 1365, 1448, 1931
DVALS = [_D3, _D47, _D5, _D67, _D71, _D94]
EDGES = []
for _p, (_dh, _dl) in enumerate([(_D67, _D94), (_D67, _D94), (_D47, _D67), (_D3, _D47), (_D3, _D47)]):
    EDGES += [(_p, 6, _dh), (_p, 1, _dl), (_p, 3, _dl)]
EDGES += [(5, 0, _D71), (5, 4, _D71), (5, 2, _D71), (5, 6, _D71)]
EDGES += [(6, 2, _D5), (6, 6, _D5), (6, 1, _D71), (6, 5, _D71)]
assert len(EDGES) == 23
PIECE_EDGES = [[e for e, ed in enumerate(EDGES) if ed[0] == p] for p in range(7)]

# piece greys (Grey tone) in 1/16 of white: LA LB MT SA SB SQ PG
GREY = [13, 9, 6, 11, 7, 15, 4]


def sin_rom():
    return [int(round(4096 * math.sin(i * math.pi / 512.0))) for i in range(256)]


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


# ---------------------------------------------------------------------------------------------------------
# the engine's table ROM
# ---------------------------------------------------------------------------------------------------------
EXPB, P2B, MODEB, FIGB, PCB = 0, 256, 272, 288, 520
FT = exp2_rom()
FT += [1 << k for k in range(16)]
# mode table by x step (2 = 1080, 3 = 720p, 4 = SD): SC (virtual px -> pixel widths, Q15), the screen centre in DDA
# columns (Q4) and in pixel widths down (Q4), the edge inset (Q12 px)
for xs, sc, cxp, cyp, ins in ((2, 32768, 961 * 16, 540 * 16, 6144), (3, 21845, 641 * 16, 360 * 16, 4096),
                              (4, 16384, 361 * 16, 270 * 16, 4096), (5, 0, 0, 0, 0)):
    FT += [sc, cxp, cyp, ins]
assert len(FT) == FIGB
for sc, pcs in FIGS:
    for x, y, k, dk in pcs:
        FT += [x & 0xFFFF, y & 0xFFFF, k * 8192, (dk * 4096) & 0xFFFF]
    FT += [sc]
assert len(FT) == PCB
for p in range(7):
    FT += [int(round(SC_TURN[p] * 8192)) & 0xFFFF, SC_SLOT[p][0] & 0xFFFF, SC_SLOT[p][1] & 0xFFFF]
FT += [0] * (1024 - len(FT))

DEFAULTS = dict(k1=512, k2=512, k3=512, k4=0, k5=650, k6=1023, p12=0)

# ---------------------------------------------------------------------------------------------------------
# micro-assembler
# ---------------------------------------------------------------------------------------------------------
OPC = dict(LD=0, ADD=1, SUB=2, MUL=3, LDI=4, ADDI=5, SUBI=6, MULI=7, ST=8, IN=9, OUT=10, SHR=11, SKIP=12,
           JMP=13, CALL=14, SPC=15)
INS = dict(K1=0, K2=1, K3=2, K4=3, K5=4, K6=5, P12=7, XS=8, RQ=9, M=10, ROM=11, ROMS=12, SIN=13, COS=14, ROMI=15, ROMSI=6)
CONDS = dict(LT=0, GLIDE=1, MIR=2, IL=3, GE=8, NGLIDE=9, NMIR=10, NIL=11)
OUTS = dict(ADR=0, ANG=1, EA=2, ED=3, KC=4, VK1=5, VK2=6, VK3=7, GG2=8, GG3=9, KX=10)
RFW = 32


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
            assert len(self.regs) <= 64, "register file full"
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

    def rom(self, base, signed=False):
        """ACC = FT[ACC + base]"""
        if base:
            self.ADDI(base) if base < 2048 else (self.ADDI(2047), self.ADDI(base - 2047))
        self.OUT('ADR'); self.IN('ROMS' if signed else 'ROM')


def PROGRAM():
    a = Asm()
    D = DEFAULTS
    # state that lives across fields (power-up values = the TOML defaults); 'first' < 0 on the first field
    for n, k in (('h1', 'k1'), ('h2', 'k2'), ('h3', 'k3'), ('h4', 'k4'), ('h5', 'k5'), ('h6', 'k6'), ('h12', 'p12')):
        a.reg(n, D[k])
    for n, k in (('g1', 'k1'), ('g2', 'k2'), ('g3', 'k3'), ('g6', 'k6'), ('g12', 'p12')):
        a.reg(n, D[k] * 8)
    a.reg('gx', 0)
    a.reg('vgs', (D['k5'] - 512) * 64)
    a.reg('ph', 0)
    a.reg('zone', (D['k4'] * 8) >> 10)
    a.reg('first', -1)

    a.JMP('main')

    # ---- CH: hold + glide of one knob: kin = the knob, hh = its hold, gg = its glide (Q3).  Hold: the knob
    # moves the held value only by 2 counts or more, to either end, or on the first field.  Glide On: 1/16 of
    # the gap per field, never less than half a count, landing exactly; Glide off or the first field: the hold.
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

    # ---- POSE: one piece.  pa / pb = the piece's words of figures A and B, pc = its scatter words.
    # x, y (Q12 units) and the angle (65536 per turn) between the two figures by the ease E (Q12); the scatter
    # turn and slot (when s > 0); the centroid in pixel widths (Q4: pi across in DDA columns, pj down); the sine
    # unit's (c0, s0) and the 45-degree pair c1 = (c0 - s0) / sqrt 2, s1 = (c0 + s0) / sqrt 2 (Q12).
    a.label('POSE')
    a.LD('pa'); a.OUT('ADR'); a.ADDI(4); a.ST('pa')
    a.IN('ROMSI'); a.ST('xa'); a.IN('ROMSI'); a.ST('ya'); a.IN('ROMI'); a.ST('th'); a.IN('ROMSI'); a.ST('t1')
    a.LD('pb'); a.OUT('ADR'); a.ADDI(4); a.ST('pb')
    a.IN('ROMSI'); a.SUB('xa'); a.MUL('E'); a.SHR(12); a.ADD('xa'); a.ST('x')
    a.IN('ROMSI'); a.SUB('ya'); a.MUL('E'); a.SHR(12); a.ADD('ya'); a.ST('y')
    a.LD('t1'); a.MUL('E'); a.SHR(11); a.ADD('th'); a.ST('th')
    a.LDI(0); a.ST('xa'); a.ST('ya')
    a.LD('sm'); a.SKIP('LT'); a.JMP('noscat')
    a.LD('pc'); a.OUT('ADR')
    a.LD('th'); a.ST('t1'); a.IN('ROMSI'); a.MUL('s'); a.SHR(10); a.ADD('t1'); a.ST('th')
    a.IN('ROMSI'); a.MUL('G'); a.SHR(10); a.ST('xa')
    a.IN('ROMSI'); a.MUL('G'); a.SHR(10); a.ST('ya')
    a.label('noscat')
    a.LD('th'); a.OUT('ANG')                                    # the sine unit starts (ready 4 micro-ops on)
    a.LD('x'); a.MUL('spf'); a.SHR(14); a.ADD('pcx'); a.ADD('xa'); a.ST('pi')
    a.LD('y'); a.MUL('spf'); a.SHR(14); a.ADD('pcy'); a.ADD('ya'); a.ST('pj')
    a.IN('SIN'); a.ST('s0'); a.IN('COS'); a.ST('c0')
    a.SUB('s0'); a.MULI(181); a.SHR(8); a.ST('c1')
    a.LD('c0'); a.ADD('s0'); a.MULI(181); a.SHR(8); a.ST('s1')
    a.LDI(0); a.SUB('c0'); a.ST('nc0'); a.LDI(0); a.SUB('s0'); a.ST('ns0')
    a.LDI(0); a.SUB('c1'); a.ST('nc1'); a.LDI(0); a.SUB('s1'); a.ST('ns1')
    a.LD('pc'); a.ADDI(3); a.ST('pc')
    a.RET()

    a.label('main')
    a.IN('RQ'); a.ST('rq'); a.IN('M'); a.ST('m')
    # mode table: SC, the screen centre (pcx0, pcy0, Q4) and the inset
    a.IN('XS'); a.MULI(4); a.ADDI(MODEB - 8); a.OUT('ADR')
    a.IN('ROMI'); a.ST('c0'); a.IN('ROMI'); a.ST('pcx'); a.IN('ROMI'); a.ST('pcy'); a.IN('ROMI'); a.ST('s0')

    # ---- knob holds + glides (CH).  (K4 and K5 use only their holds.)
    for inp, h, g in (('K1', 'h1', 'g1'), ('K2', 'h2', 'g2'), ('K3', 'h3', 'g3'), ('K4', 'h4', 'gx'),
                      ('K5', 'h5', 'gx'), ('K6', 'h6', 'g6'), ('P12', 'h12', 'g12')):
        a.IN(inp); a.ST('kin'); a.LD(h); a.ST('hh'); a.LD(g); a.ST('gg'); a.CALL('CH')
        a.LD('hh'); a.ST(h); a.LD('gg'); a.ST(g)

    # ---- K4 FIGURE: 8 zones; a new zone is taken when the held knob sits 8 counts inside it, at either end of
    # the travel, or on the first field
    a.LD('h4'); a.MULI(8); a.ST('t0'); a.SHR(10); a.ST('t1'); a.MULI(1024); a.ST('t2')
    a.LD('t0'); a.SUB('t2'); a.ST('t2')                                    # position inside the zone, 0 .. 1023
    a.LD('first'); a.SKIP('GE'); a.JMP('ztake')
    a.LD('h4'); a.SKIP('GE'); a.JMP('ztake'); a.SUBI(1); a.SKIP('GE'); a.JMP('ztake')
    a.LD('h4'); a.SUBI(1023); a.SKIP('LT'); a.JMP('ztake')
    a.LD('t2'); a.SUBI(64); a.SKIP('GE'); a.JMP('zdone')
    a.LD('t2'); a.SUBI(960); a.SKIP('LT'); a.JMP('zdone')
    a.label('ztake'); a.LD('t1'); a.ST('zone')
    a.label('zdone')

    # ---- DRIFT (Hypnos): speed glide 1/8 per field (Q6); |v| < 24 still, else 2^(ls/256 - 9) step per field,
    # ls = (|v| - 24) * 1024 / 487 (four octaves); the phase (Q21 steps, 8 steps = 2^24) wraps exactly
    a.LDI(1024); a.MULI(4); a.ST('t4'); a.MUL('t4'); a.ST('t4')           # 2^24 = 4096 * 4096 (18-bit operands)
    a.LD('h5'); a.SUBI(512); a.MULI(64); a.ST('t1')
    a.SKIP('GLIDE'); a.JMP('vsn')
    a.LD('first'); a.SKIP('GE'); a.JMP('vsn')
    a.LD('t1'); a.SUB('vgs'); a.SHR(3); a.ADD('vgs'); a.ST('vgs'); a.JMP('vdec')
    a.label('vsn'); a.LD('t1'); a.ST('vgs')
    a.label('vdec')
    a.LD('vgs'); a.SHR(6); a.ST('x'); a.abs_('t3'); a.SUBI(24); a.SKIP('GE'); a.JMP('dd')
    a.MULI(1077); a.SHR(9); a.SUBI(1023); a.SKIP('LT'); a.LDI(0); a.ADDI(1023)
    a.ST('y'); a.SHR(8); a.ST('t0'); a.MULI(256); a.ST('t2'); a.LD('y'); a.SUB('t2'); a.rom(EXPB); a.ST('t2')
    a.LD('t0'); a.rom(P2B); a.MUL('t2'); a.SHR(3); a.ST('t2')
    a.LD('x'); a.SKIP('LT'); a.JMP('fwd')
    a.LD('ph'); a.SUB('t2'); a.SKIP('GE'); a.ADD('t4'); a.JMP('dst')
    a.label('fwd')
    a.LD('ph'); a.ADD('t2'); a.ST('t0'); a.SUB('t4'); a.SKIP('GE'); a.LD('t0')
    a.label('dst'); a.ST('ph')
    a.label('dd')

    # ---- the step and the ease: idx = ph >> 21, u = the step fraction (Q12); dwell for u < 1/2, then the
    # flight: t = 2u - 1, E = t^2 (3 - 2t) (Q12)
    a.LD('ph'); a.SHR(9); a.ST('t0'); a.SHR(12); a.ST('t3'); a.MULI(1024); a.MULI(4); a.ST('t1')
    a.LD('t0'); a.SUB('t1')                                                # u (Q12) = the step fraction
    a.ADDI(-2048); a.SKIP('GE'); a.LDI(0); a.MULI(2); a.ST('t1')            # t (Q12), 0 .. 4094
    a.MUL('t1'); a.SHR(12); a.ST('t2'); a.LDI(1536); a.MULI(8); a.SUB('t1'); a.SUB('t1'); a.MUL('t2'); a.SHR(12)
    a.ST('E')
    # figures A (this step) and B (the next): FIGB + 29 f
    a.LD('t3'); a.ADD('zone'); a.ST('t0'); a.SUBI(8); a.SKIP('LT'); a.ST('t0')
    a.LD('t0'); a.MULI(29); a.ADDI(FIGB); a.ST('pa')
    a.LD('t0'); a.SUBI(7); a.SKIP('LT'); a.LDI(-8); a.ADDI(8); a.MULI(29); a.ADDI(FIGB); a.ST('pb')
    # the figure scale between them (Q14)
    a.LD('pa'); a.ADDI(28); a.OUT('ADR'); a.IN('ROM'); a.ST('t0')
    a.LD('pb'); a.ADDI(28); a.OUT('ADR'); a.IN('ROM'); a.SUB('t0'); a.MUL('E'); a.SHR(12); a.ADD('t0'); a.ST('x')

    # ---- SCALE: K3S = 2^((K3 - 512) / 512 * 1.25) (Q14); S_px = 248 K3S sf SC (pixel widths per unit, Q6)
    a.LD('g3'); a.ADDI(-2048); a.ADDI(-2048); a.MULI(5); a.SHR(6); a.ADDI(512); a.ST('t0')
    a.SHR(8); a.ST('t1'); a.MULI(256); a.ST('t2'); a.LD('t0'); a.SUB('t2'); a.rom(EXPB); a.ST('t2')
    a.LD('t1'); a.rom(P2B); a.MUL('t2'); a.SHR(3); a.ST('s1')
    a.MUL('x'); a.SHR(14); a.MUL('c0'); a.SHR(15); a.MULI(248); a.SHR(8); a.ST('c1')
    # Mirror: the figure at 0.72
    a.SKIP('MIR'); a.JMP('nmir1'); a.MULI(737); a.SHR(10); a.ST('c1')
    a.label('nmir1')
    # scatter s (Q10); f = 1 - 0.8 s; spf = S_px f; spg = S_px (1 + 0.15 s); slot gain G = s K3S SC (Q14)
    a.LD('g12'); a.SHR(3); a.ST('s'); a.LDI(0); a.SUB('s'); a.ST('sm')
    a.LD('s'); a.MULI(819); a.SHR(10); a.ST('t0'); a.LDI(1024); a.SUB('t0'); a.MUL('c1'); a.SHR(10); a.ST('spf')
    a.LD('s'); a.MULI(154); a.SHR(10); a.ADDI(1024); a.MUL('c1'); a.SHR(10); a.ST('spg')
    a.LD('s'); a.MUL('s1'); a.SHR(10); a.MUL('c0'); a.SHR(15); a.ST('G')
    a.SKIP('MIR'); a.JMP('nmir2'); a.MULI(737); a.SHR(10); a.ST('G')
    a.label('nmir2')
    # the figure centre (Q4): X 1.875 virtual px per count, Y 1.0547; Mirror: + 480 virtual px
    a.LD('g1'); a.ADDI(-2048); a.ADDI(-2048); a.MULI(15); a.MUL('c0'); a.SHR(17); a.ADD('pcx'); a.ST('pcx')
    a.SKIP('MIR'); a.JMP('nmir3'); a.LD('c0'); a.MULI(240); a.SHR(10); a.ADD('pcx'); a.ST('pcx')
    a.label('nmir3')
    a.LD('g2'); a.ADDI(-2048); a.ADDI(-2048); a.MULI(135); a.SHR(4); a.MUL('c0'); a.SHR(17); a.ADD('pcy'); a.ST('pcy')
    # the edge offsets d * S + 1/2 pixel (interlaced: 1 pixel) - the inset (Q12)
    a.LDI(1024); a.MULI(2); a.SKIP('NIL'); a.ADDI(2047); a.SKIP('NIL'); a.ADDI(1); a.SUB('s0'); a.ST('nc0')
    for i, d in enumerate(DVALS):
        a.LD('spg'); a.MULI(d); a.SHR(5); a.ADD('nc0'); a.ST('ds%d' % i)

    # ---- the seven pieces and their 23 edges.  Edge j (45-degree steps from the piece angle): normal
    # (cos a, sin a) from (c0, s0) / (c1, s1); pixel step DX = -cos a (Q12), line step DY = -(sin a rq) >> 10,
    # wrap = DY - M DX, seed = d S + bias + (cos a pi + sin a pj) >> 4.  Three words per edge.
    a.LDI(PCB); a.ST('pc')
    a.LDI(0); a.OUT('EA')
    # (cos a, sin a, -cos a) as registers for each direction j
    SEL = {0: ('c0', 's0', 'nc0'), 1: ('c1', 's1', 'nc1'), 2: ('ns0', 'c0', 's0'), 3: ('ns1', 'c1', 's1'),
           4: ('nc0', 'ns0', 'c0'), 5: ('nc1', 'ns1', 'c1'), 6: ('s0', 'nc0', 'ns0'), 7: ('s1', 'nc1', 'ns1')}
    prev = None
    for p in range(7):
        a.CALL('POSE')
        for e in PIECE_EDGES[p]:
            _, j, d = EDGES[e]
            ca, sa, nca = SEL[j]
            dsr = 'ds%d' % DVALS.index(d)
            if prev is not None and prev[0] == p and (prev[1] + 4) % 8 == j and prev[2] == d:
                # the opposite edge of the one just written: every word negated about the same offset
                a.LD(nca); a.OUT('ED')
                a.LDI(0); a.SUB('x'); a.OUT('ED')
                a.LD(dsr); a.SUB('th'); a.OUT('ED')
                prev = None
                continue
            a.LD(nca); a.OUT('ED')                                               # DX = -cos a
            a.LD(sa); a.MUL('rq'); a.SHR(10); a.ST('t1')                         # (sin a rq) >> 10
            a.LD(ca); a.MUL('m'); a.SUB('t1'); a.ST('x'); a.OUT('ED')           # wrap = M cos a - that
            a.LD(ca); a.MUL('pi'); a.ST('y'); a.LD(sa); a.MUL('pj'); a.ADD('y'); a.SHR(4); a.ST('th')
            a.ADD(dsr); a.OUT('ED')                                              # seed
            prev = (p, j, d)

    # ---- CAMERA: k = K6 (0 .. 16); the tones of the block levels (tone8 x 16 per unit coverage):
    # inside k V_L (V = .26 .56 .90), ground k (0, 0, .11, .22); Ink flips L >= 2 blocks when k >= 4
    a.LD('g6'); a.ADDI(256); a.SHR(9); a.ST('t0'); a.OUT('KC')
    a.MULI(66); a.SHR(4); a.OUT('VK1')
    a.LD('t0'); a.MULI(143); a.SHR(4); a.OUT('VK2')
    a.LD('t0'); a.MULI(230); a.SHR(4); a.OUT('VK3')
    a.LD('t0'); a.MULI(28); a.SHR(4); a.OUT('GG2')
    a.LD('t0'); a.MULI(56); a.SHR(4); a.OUT('GG3')
    a.LD('t0'); a.SUBI(4); a.OUT('KX')
    a.LDI(0); a.ST('first')
    a.HALT()
    return a


_ASM = PROGRAM()
UCODE = _ASM.words()
RF_INIT = [0] * 64
for _n, _i in _ASM.regs.items():
    RF_INIT[_i] = _ASM.init[_n] & ((1 << RFW) - 1)


# ---------------------------------------------------------------------------------------------------------
# the hardware sine unit (tans.vhd mirrors it): angle (65536 per turn) -> sin, cos (Q12, signed)
# ---------------------------------------------------------------------------------------------------------
SIN = sin_rom()


def _sq(theta):
    theta &= 0xFFFF
    q = theta >> 14
    p = theta & 16383
    if q & 1:
        p = 16384 - p
    i, fr = p >> 6, p & 63
    t0 = 4096 if i >= 256 else SIN[i]
    t1 = 4096 if i + 1 >= 256 else SIN[i + 1]
    v = t0 + (((t1 - t0) * fr) >> 6)
    return -v if q & 2 else v


def sincos(theta):
    return _sq(theta), _sq(theta + 16384)


# ---------------------------------------------------------------------------------------------------------
# emulator (the integer model's per-field arithmetic; tans.vhd's engine does exactly this)
# ---------------------------------------------------------------------------------------------------------
def _sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def emulate(inp, flags, rf=None, max_instr=200000):
    """inp: dict over INS names (K1.. raw knobs, XS, RQ, M); flags: GLIDE, MIR, IL.
    rf: the register file (list of 64 ints, updated in place; None = the power-up file).
    Returns (outs dict by OUTS name, edge table dict addr -> 32-bit value, rf, clocks)."""
    if rf is None:
        rf = list(RF_INIT)
    acc = 0; pc = 0; lr = 0; adr = 0; ea = 0; ang = 0; ang_at = -100
    outs = {}; etab = {}
    clocks = 1
    inv = {v: k for k, v in INS.items()}
    n_ins = 0
    for _ in range(max_instr):
        w = UCODE[pc]; op = w >> 12; arg = w & 0xFFF
        npc = pc + 1
        clocks += 3
        n_ins += 1
        if op in (0, 1, 2, 3):
            b = _sx(rf[arg & 63], RFW)
        elif op in (4, 5, 6, 7):
            b = _sx(arg, 12)
        if op in (0, 4):
            acc = b
        elif op in (1, 5):
            acc = _sx(acc + b, 38)
        elif op in (2, 6):
            acc = _sx(acc - b, 38)
        elif op in (3, 7):
            acc = _sx(acc, 18) * _sx(b, 18)
            clocks += 8
        elif op == 8:
            rf[arg & 63] = acc & ((1 << RFW) - 1)
        elif op == 9:
            n = arg & 15
            if n == INS['ROM']: acc = FT[adr]
            elif n == INS['ROMS']: acc = _sx(FT[adr], 16)
            elif n == INS['ROMI']: acc = FT[adr]; adr = (adr + 1) & 1023
            elif n == INS['ROMSI']: acc = _sx(FT[adr], 16); adr = (adr + 1) & 1023
            elif n in (INS['SIN'], INS['COS']):
                assert n_ins - ang_at >= (5 if n == INS['SIN'] else 7), "sine unit read too soon after OUT ANG"
                acc = sincos(ang)[0 if n == INS['SIN'] else 1]
            else: acc = inp[inv[n]]
        elif op == 10:
            n = arg & 31
            if n == OUTS['ADR']: adr = acc & 1023
            elif n == OUTS['ANG']: ang = acc & 0xFFFF; ang_at = n_ins
            elif n == OUTS['EA']: ea = acc & 127
            elif n == OUTS['ED']:
                etab[ea] = acc & 0xFFFFFFFF; ea = (ea + 1) & 127
            outs[n] = acc
        elif op == 11:
            acc >>= (arg & 31)
            clocks += ((arg & 31) + 1) // 2 - 1
        elif op == 12:
            c = arg & 7
            t = (acc < 0) if c == 0 else bool(flags[{1: 'GLIDE', 2: 'MIR', 3: 'IL'}[c]])
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
    return {inv_o[k]: v for k, v in outs.items()}, etab, rf, clocks


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


def main():
    assert len(UCODE) <= 1024, len(UCODE)
    uc = UCODE + [0xF001] * (1024 - len(UCODE))
    parts = ["-- Generated by tans.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tans_pkg is",
             vhd_table("C_TN_SIN", "std_logic_vector", 256, 16, SIN),
             vhd_table("C_TN_FT", "std_logic_vector", 1024, 16, FT),
             vhd_table("C_TN_UC", "std_logic_vector", 1024, 16, uc),
             vhd_table("C_TN_RFI", "std_logic_vector", 64, 32, RF_INIT),
             "end package tans_pkg;", ""]
    with open(os.path.join(HERE, "tans_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
