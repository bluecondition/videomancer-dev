#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
COLORFIELD v0.1.0 - build hook.  Writes colorfield_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  Also the reference model of the per-field work: the integer model
(.patchbay_work/rb_colorfield/imodel.py) imports the tables, the microcode interpreter and the table fill from here.

The patch (see SUMMARY.md):
  H ramp, V ramp -> + slow noise (four ridge terms from 512-entry ROMs) + source luma + dither -> the ADDRESS
  of two 512-entry profile tables: the long axis (key K and paint colour Y, U, V) and the cross axis (key Kc).
  out = ground + K * Kc * (paint - ground).
  The tables are redrawn every field by a table fill (window keys from one accumulator and 8 constants per
  entry) whose constants come from a small microcoded engine (regfile RAM + one serial multiplier): knob
  glides, LFOs, layout, feather gain, colour path lookups.

Package contents:
  C_CF_UC    256 x 32  microcode
  C_CF_RF    256 x 32  register file initial contents (constants, default state)
  C_CF_DROM  1024 x 8  chord colours (16 chords x 16 path points x Y, U, V) + grounds + 16 layouts
  C_CF_EXP2  256 x 16  round(32768 * 2^(i/256))
  C_CF_SROM  512 x 8   smoothstep key (0..255) and the halo skirt (256..511)
  C_CF_NA..ND 512 x 8  edge noise (signed)
  C_CF_FCX/Y 256 x 32  fill constants, initial contents;  C_CF_PF / C_CF_CR  initial profile tables
Colours are in the hardware convention (U register = Cr, V register = Cb); luma pre-scaled 16/13 for the
shell's 13/16 output scale, capped at 940.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ chords: name, dark ground, light ground, 4 anchors
CHORDS = [
    ("Maroon",   (16, 6, 10),    (214, 150, 120), [(128, 18, 34), (74, 10, 30), (170, 40, 30), (60, 16, 56)]),
    ("Ember",    (10, 6, 4),     (240, 200, 130), [(240, 96, 8), (200, 28, 16), (250, 160, 20), (120, 14, 20)]),
    ("Saffron",  (120, 30, 8),   (250, 206, 40),  [(245, 120, 8), (228, 60, 14), (250, 170, 20), (200, 36, 20)]),
    ("Rust Blue", (10, 12, 44),  (186, 196, 214), [(176, 62, 26), (28, 60, 176), (120, 36, 30), (70, 50, 150)]),
    ("Blue Green", (6, 56, 30),  (150, 210, 120), [(24, 50, 190), (16, 120, 140), (40, 30, 150), (20, 150, 110)]),
    ("Hot Pink", (8, 6, 12),     (255, 170, 215), [(254, 10, 187), (120, 20, 170), (250, 90, 30), (200, 10, 90)]),
    ("Seagram",  (40, 6, 12),    (150, 60, 50),   [(110, 14, 22), (160, 30, 22), (70, 8, 20), (190, 60, 30)]),
    ("Ochre",    (30, 20, 8),    (236, 214, 160), [(206, 150, 30), (150, 80, 20), (230, 190, 70), (110, 50, 16)]),
    ("Sunset",   (104, 10, 22),  (246, 208, 96),  [(250, 186, 16), (246, 112, 8), (214, 32, 26), (236, 70, 66)]),
    ("Violet",   (14, 30, 20),   (200, 180, 230), [(120, 40, 190), (60, 20, 120), (190, 80, 200), (30, 110, 70)]),
    ("Ashes",    (8, 8, 10),     (200, 200, 196), [(120, 118, 116), (40, 40, 44), (90, 60, 44), (170, 168, 160)]),
    ("Coral",    (4, 40, 50),    (180, 235, 225), [(250, 100, 80), (20, 170, 170), (240, 60, 90), (40, 120, 160)]),
    ("Acid",     (12, 24, 4),    (220, 250, 120), [(150, 230, 10), (60, 110, 10), (230, 240, 30), (20, 150, 60)]),
    ("Chapel",   (6, 4, 10),     (90, 80, 110),   [(40, 20, 60), (70, 16, 40), (24, 22, 70), (90, 30, 70)]),
    ("Meadow",   (10, 30, 70),   (240, 230, 150), [(245, 210, 30), (30, 80, 190), (60, 170, 60), (250, 150, 20)]),
    ("Spectrum", (8, 8, 8),      (235, 235, 235), [(230, 30, 30), (240, 200, 20), (30, 170, 80), (40, 60, 210)]),
]
# layouts: 4 x (centre, half-size) in table entries (0..511 along the long axis); absent fields wait at 1000
OFF = (1000, 0)
LAYOUTS = [
    [(256, 196), OFF, OFF, OFF], [(190, 120), OFF, OFF, OFF], [(300, 150), OFF, OFF, OFF], [(256, 44), OFF, OFF, OFF],
    [(168, 116), (400, 64), OFF, OFF], [(148, 96), (364, 96), OFF, OFF], [(108, 56), (328, 132), OFF, OFF], [(72, 24), (296, 164), OFF, OFF],
    [(144, 84), (276, 18), (392, 70), OFF], [(108, 60), (256, 60), (404, 60), OFF], [(84, 28), (252, 108), (420, 28), OFF], [(152, 104), (324, 44), (428, 28), OFF],
    [(88, 44), (200, 44), (312, 44), (424, 44)], [(104, 60), (208, 16), (272, 16), (392, 76)], [(120, 76), (252, 40), (344, 32), (424, 24)], [(76, 16), (196, 16), (316, 16), (436, 16)],
]
CX, HX = 256, 200                      # cross window: centre, half-size (entries)
DEFAULTS = dict(k1=560, k2=700, k3=512, k4=680, k5=512, k6=546, sw=0, p12=430)

def rgb_to_hw10(rgb):
    r, g, b = [c / 255.0 for c in rgb]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb, cr = (b - y) / 1.772, (r - y) / 1.402
    return (64 + y * 876, 512 + cr * 896, 512 + cb * 896)

def col8(rgb):
    """8-bit table colour: luma pre-scaled 16/13 and capped at 940, all three >> 2"""
    y, u, v = rgb_to_hw10(rgb)
    y = min(940.0, max(64.0, 64 + (y - 64) * 16.0 / 13.0))
    return [max(0, min(255, int(round(c / 4.0)))) for c in (y, u, v)]

def drom():
    out = [0] * 1024
    for c, (_, gd, gl, anc) in enumerate(CHORDS):
        a = [col8(x) for x in anc]
        for i in range(16):
            p, q, f = a[i // 4], a[(i // 4 + 1) % 4], (i % 4) / 4.0
            for ch in range(3):
                out[c * 64 + i * 4 + ch] = int(round(p[ch] + (q[ch] - p[ch]) * f))
        for k, v in enumerate(col8(gd) + col8(gl)):
            out[c * 64 + k * 4 + 3] = v
    for l, lay in enumerate(LAYOUTS):
        for r, (cc, hh) in enumerate(lay):
            out[l * 64 + (8 + r) * 4 + 3] = cc // 4
            out[l * 64 + (12 + r) * 4 + 3] = hh // 4
    return out

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)

def srom():
    key = [int(round(255 * smooth(i / 255.0))) for i in range(256)]
    skirt = [int(round(64 * smooth((i + 0.5) / 256.0))) for i in range(256)]
    return key + skirt

class _Rnd:                              # small LCG so the tables do not depend on a library
    def __init__(self, s): self.s = s
    def rand(self):
        self.s = (self.s * 1103515245 + 12345) & 0x7FFFFFFF
        return self.s / float(0x80000000)

def noise_rom(seed):
    r = _Rnd(seed)
    ph = [r.rand() for _ in range(64)]
    n = [sum((1.0 / k ** 1.5) * math.sin(2 * math.pi * (k * x / 512.0 + ph[k])) for k in range(2, 28)) for x in range(512)]
    m = max(abs(v) for v in n)
    return [int(round(v / m * 127)) for v in n]

# ------------------------------------------------------------------ microcode
HALT, ADD, SUB, MUL, GLD, ROM, EXP, SHR, SHL, IN, OUT, SRM, MXN, AND, TRI, LOOP = range(16)
P_FCX, P_FCY, P_G, P_C0Y, P_C0U, P_C0V, P_GY, P_GU, P_GV, P_AMP, P_SCI, P_NZ = 0, 8, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
WORD = 28                               # engine word (two's complement)
NGLOB = 112

class Asm:
    def __init__(self):
        self.code, self.g, self.f, self.k = [], {}, {}, {}
        self.init = [0] * 256
    def G(self, name, init=0):
        if name not in self.g:
            self.g[name] = len(self.g) + len(self.k); self.init[self.g[name]] = init & 0xFFFFFFFF
            assert self.g[name] < NGLOB, "out of global registers"
        return self.g[name]
    def K(self, v):
        if v not in self.k:
            self.k[v] = len(self.g) + len(self.k); self.init[self.k[v]] = v & 0xFFFFFFFF
            assert self.k[v] < NGLOB, "out of global registers"
        return self.k[v]
    def F(self, name, init=(0, 0, 0, 0)):
        if name not in self.f:
            self.f[name] = NGLOB + len(self.f)
            assert len(self.f) <= 16
            for r in range(4): self.init[NGLOB + 16 * r + len(self.f) - 1] = init[r] & 0xFFFFFFFF
        return self.f[name]
    def e(self, op, d=0, a=0, b=0, n=0, ix=0):
        assert 0 <= n < 32 and 0 <= a < 128 and 0 <= b < 128 and 0 <= d < 128
        self.code.append((op << 28) | (n << 23) | (a << 16) | (b << 9) | (d << 2) | ix)
    def loop(self, target):
        self.code.append((LOOP << 28) | (target << 16))

def program():
    A = Asm(); G, K, F, e = A.G, A.K, A.F, A.e
    # commit block (pc 0, run at the zone start before the fill): last field's results go live
    for i, nm in enumerate(('ogy', 'ogu', 'ogv', 'oamp', 'osci', 'onz0', 'onz1', 'onz2', 'onz3')):
        e(OUT, a=G(nm), n=P_GY + i)
    e(HALT)
    A.main = len(A.code)
    t0, t1, t2, t3, t4 = [G('t%d' % i) for i in range(5)]
    D = DEFAULTS
    def lfo(ph, out):                    # smoothstep(triangle(phase)) - 1/2, +-16384
        e(TRI, t0, ph)
        e(MUL, t1, t0, t0, 15)
        e(SHL, t2, t0, n=1); e(SUB, t2, K(98304), t2)
        e(MUL, t2, t2, t1, 15)
        e(SUB, out, t2, K(16384))
    # knob glides (Q4)
    for name, port in (('gk2', 1), ('gk3', 2), ('gk5', 4), ('gp', 7)):
        e(IN, t0, n=port); e(SHL, t0, t0, n=4); e(GLD, G(name), G(name), t0)
    gk2, gk3, gk5, gp = G('gk2'), G('gk3'), G('gk5'), G('gp')
    # speed: bipolar, +-8 deadband, sv = ms * (|ms| + 480) >> 2  (25600 at the default 680)
    e(IN, t0, n=3); e(SUB, t0, t0, K(512))
    e(SUB, t1, t0, K(8)); e(MXN, t1, t1, K(0))
    e(SUB, t2, K(0), t0); e(SUB, t2, t2, K(8)); e(MXN, t2, t2, K(0))
    e(SUB, t3, t1, t2); e(ADD, t4, t1, t2); e(ADD, t4, t4, K(480))
    sv = G('sv'); e(MUL, sv, t3, t4, 2)
    # layout number (+-8 hysteresis) and chord number (16 labels: the top four bits)
    lay = G('lay', D['k1'] >> 6)
    e(IN, t0, n=0); e(SUB, t1, t0, K(8)); e(SHR, t1, t1, n=6); e(ADD, t2, t0, K(8)); e(SHR, t2, t2, n=6)
    e(MXN, lay, lay, t1); e(MXN, lay, lay, t2, n=1); e(SHL, G('lay64'), lay, n=6)
    e(IN, t0, n=5); e(SHR, t0, t0, n=6); e(SHL, G('chord64'), t0, n=6)
    lay64, ch64 = G('lay64'), G('chord64')
    # feather: xf = 3.0 k2 + 6.4 p12 octaves (Q8); g = 65536 / F per entry; F in Q4 entries
    xf, g, Fh = G('xf'), G('g'), G('Fh')
    e(MUL, t0, gk2, K(3075), 16); e(MUL, t1, gp, K(6559), 16); e(ADD, xf, t0, t1)
    e(SUB, t0, K(4095), xf); e(AND, t1, t0, K(255)); e(EXP, t1, t1); e(SHR, t2, t0, n=8); e(SUB, t2, K(15), t2)
    e(SHR, g, t1, t2, n=31); e(OUT, a=g, n=P_G)
    e(AND, t1, xf, K(255)); e(EXP, t1, t1); e(SHR, t2, xf, n=8); e(ADD, G('osci'), t2, K(0)); e(SUB, t3, K(11), t2)
    e(SHR, Fh, t1, t3, n=31); e(SHR, Fh, Fh, n=1)
    # swell (60 p^2 entries, Q4) and edge-noise amplitude
    esw = G('esw')
    e(MUL, t0, gp, gp, 14); e(MUL, esw, t0, K(3848), 16)
    e(MUL, G('oamp'), gp, K(192), 16)
    # breath depth (Q14, 0.6 at full), colour spread and wander (path units Q8)
    dep, spr, wq = G('dep'), G('spr'), G('wq')
    e(MUL, dep, gk3, K(39359), 16)
    e(MUL, t0, gk5, K(6150), 16); e(ADD, spr, t0, K(256))
    e(MUL, wq, gk5, K(3075), 16)
    # noise drift (Q8.8 entries; the staged offset is bits 24..8)
    for i, (c, op, init) in enumerate(((66, ADD, 0), (49, SUB, 170 << 16), (55, ADD, 0), (40, SUB, 90 << 16))):
        nz = G('nz%d' % i, init)
        e(MUL, t0, sv, K(c), 8); e(op, nz, nz, t0); e(SHR, G('onz%d' % i), nz, n=10)
    # ground colour (Dark / Light = switch bit 2), glided
    e(IN, t0, n=6); e(SHR, t0, t0, n=2); e(AND, t0, t0, K(1))
    e(SHL, t1, t0, n=3); e(SHL, t2, t0, n=2); e(ADD, t1, t1, t2); e(ADD, t1, t1, ch64); e(ADD, t1, t1, K(3))
    for i, nm in enumerate(('gy', 'gu', 'gv')):
        r = G(nm)
        e(ROM, t2, t1); e(SHL, t2, t2, n=8); e(GLD, r, r, t2); e(SHR, G('o' + nm), r, n=6)
        if i < 2: e(ADD, t1, t1, K(4))
    # cross window: breath on its half-size (a quarter of the depth), swell, feather
    phc, lf, ac0, dc0 = G('phc', int(0.6 * (1 << 24))), G('lf'), G('ac0'), G('dc0')
    e(MUL, t0, sv, K(166), 8); e(ADD, phc, phc, t0); lfo(phc, lf)
    e(MUL, t0, K(HX * 16), dep, 14); e(MUL, t0, t0, K(16384), 16); e(MUL, t1, lf, t0, 14)
    e(ADD, t1, t1, K(HX * 16)); e(ADD, t1, t1, esw); e(SUB, t1, t1, Fh); e(MXN, t1, t1, K(0))
    e(SUB, t2, K(CX * 16), t1); e(MUL, t2, t2, g, 4); e(SUB, ac0, K(65536), t2)
    e(ADD, t3, K(CX * 16), t1); e(MUL, t3, t3, g, 4); e(ADD, dc0, K(65536), t3)
    # ---------------- per-field loop
    qb, prevb, he, ap, bp, fq, a0, a1 = [G(n) for n in ('qb', 'prevb', 'he', 'ap', 'bp', 'fq', 'a0', 'a1')]
    pv = [G('pvy'), G('pvu'), G('pvv')]
    dd = [G('ddy'), G('ddu'), G('ddv')]
    lay = LAYOUTS[D['k1'] >> 6]
    Fc = F('c', [c * 16 for c, h in lay]); Fhh = F('h', [h * 16 for c, h in lay])
    Fphb = F('phb', [int((0.13 + 0.29 * r) * (1 << 24)) for r in range(4)])
    Fphq = F('phq', [int(((0.4 * r) % 1.0) * (1 << 24)) for r in range(4)])
    Foc = F('offc', [35 + 4 * r for r in range(4)]); Foh = F('offh', [51 + 4 * r for r in range(4)])
    Frb = F('rb', (383, 307, 239, 201)); Frq = F('rq', (122, 90, 76, 65))
    Fcol = [F('cy'), F('cu'), F('cv')]
    e(SUB, qb, qb, qb)
    top = len(A.code)
    e(ADD, t0, lay64, Foc); e(ROM, t0, t0); e(SHL, t0, t0, n=6); e(GLD, Fc, Fc, t0)
    e(ADD, t0, lay64, Foh); e(ROM, t0, t0); e(SHL, t0, t0, n=6); e(GLD, Fhh, Fhh, t0)
    e(MUL, t0, sv, Frb, 8); e(ADD, Fphb, Fphb, t0); lfo(Fphb, lf)
    e(MUL, t0, Fhh, dep, 14); e(MUL, t1, lf, t0, 14); e(ADD, t1, t1, Fhh); e(ADD, t1, t1, esw); e(SUB, t1, t1, Fh)
    e(MXN, he, t1, K(0))
    e(SUB, ap, Fc, he); e(ADD, bp, Fc, he)
    e(MUL, t0, ap, g, 4); e(SUB, t0, K(65536), t0); e(OUT, a=t0, n=P_FCX, ix=1)
    e(MUL, t0, bp, g, 4); e(ADD, t0, K(65536), t0); e(OUT, a=t0, n=P_FCY, ix=1)
    e(ADD, t0, prevb, ap); e(SHR, t0, t0, n=1); e(MUL, t0, t0, g, 4); e(SUB, t0, K(16384), t0); e(OUT, a=t0, n=P_FCX + 4, ix=1)
    e(ADD, prevb, bp, K(0))
    # colour: path position q = r * spread + wander * lfo (Q8, mod 16 points), linear between path points, glided
    e(MUL, t0, sv, Frq, 8); e(ADD, Fphq, Fphq, t0); lfo(Fphq, lf)
    e(MUL, t1, lf, wq, 14); e(ADD, t1, t1, qb); e(AND, t1, t1, K(4095)); e(ADD, qb, qb, spr)
    e(SHR, t2, t1, n=8); e(AND, fq, t1, K(255)); e(SHL, t3, t2, n=2); e(ADD, a0, t3, ch64)
    e(ADD, t2, t2, K(1)); e(AND, t2, t2, K(15)); e(SHL, t3, t2, n=2); e(ADD, a1, t3, ch64)
    for ch in range(3):
        e(ROM, t0, a0); e(ROM, t1, a1); e(SUB, t1, t1, t0); e(MUL, t1, t1, fq, 0); e(SHL, t0, t0, n=8); e(ADD, t0, t0, t1)
        e(GLD, Fcol[ch], Fcol[ch], t0)
        e(SHR, t2, Fcol[ch], n=(6 if ch == 0 else 9)); e(SUB, dd[ch], t2, pv[ch]); e(ADD, pv[ch], t2, K(0))
        if ch < 2: e(ADD, a0, a0, K(1)); e(ADD, a1, a1, K(1))
    e(AND, t0, dd[0], K(0x7FF)); e(SHL, t0, t0, n=16); e(AND, t1, dd[1], K(255)); e(SHL, t1, t1, n=8); e(AND, t2, dd[2], K(255))
    e(ADD, t0, t0, t1); e(ADD, t0, t0, t2); e(OUT, a=t0, n=P_FCY + 4, ix=1)
    A.loop(top)
    # field 0's colour is the base of the crossfade chain; the cross window goes in slot 4
    e(SHR, t0, Fcol[0], n=6); e(OUT, a=t0, n=P_C0Y)
    e(SHR, t0, Fcol[1], n=9); e(OUT, a=t0, n=P_C0U)
    e(SHR, t0, Fcol[2], n=9); e(OUT, a=t0, n=P_C0V)
    e(OUT, a=ac0, n=P_FCX + 4); e(OUT, a=dc0, n=P_FCY + 4)
    e(HALT)
    assert len(A.code) <= 256, len(A.code)
    return A

# ------------------------------------------------------------------ engine model (bit-exact with the RTL)
def s32(x):
    x &= (1 << WORD) - 1
    return x - (1 << WORD) if x >> (WORD - 1) else x

class Engine:
    def __init__(self, snap=False):
        self.A = program()
        self.uc = self.A.code + [0] * (256 - len(self.A.code))
        self.rf = [s32(v) for v in self.A.init]
        self.drom, self.exp2, self.srom = drom(), exp2_rom(), srom()
        self.snap = snap
        self.port = [0] * 32
        self.clocks = 0
    def commit(self):
        self.run(None, 0)
    def run(self, knobs, pc=None):
        """knobs: [k1..k6, switches, p12] -> runs the main program once (pc = 0: the commit block)"""
        pc = self.A.main if pc is None else pc
        fidx, rf = 0, self.rf
        while True:
            ir = self.uc[pc]
            op, n, a, b, d, ix = ir >> 28, (ir >> 23) & 31, (ir >> 16) & 127, (ir >> 9) & 127, (ir >> 2) & 127, ir & 1
            ea = lambda r: r + 16 * fidx if r >= NGLOB else r
            X, Y = rf[ea(a)], rf[ea(b)]
            self.clocks += 10
            if op == HALT: return
            if op == LOOP:
                fidx = (fidx + 1) & 3
                pc = ((ir >> 16) & 255) if fidx else pc + 1
                continue
            if op == OUT:
                self.port[n + (fidx if ix else 0)] = X; pc += 1; continue
            if op == ADD: H = s32(X + Y)
            elif op == SUB: H = s32(X - Y)
            elif op == MUL:
                H, M, B = 0, X, Y & 0xFFFF
                for i in range(16):
                    t = s32(H + M) if (B >> i) & 1 else H
                    if i < n: H = t >> 1
                    else: H = t; M = s32(M << 1)
                self.clocks += 16
            elif op == GLD:
                df = s32(Y - X)
                H = Y if (self.snap or -8 < df < 8) else s32(X + (df >> 3))
            elif op == ROM: H = self.drom[X & 1023]
            elif op == EXP: H = self.exp2[X & 255]
            elif op == SHR: H = X >> ((Y & 31) if n == 31 else n)
            elif op == SHL: H = s32(X << n)
            elif op == IN: H = knobs[n]
            elif op == SRM: H = self.srom[X & 511]
            elif op == MXN: H = min(X, Y) if n else max(X, Y)
            elif op == AND: H = X & Y
            elif op == TRI:
                t = (X >> 8) & 0xFFFF
                H = (~t & 0x7FFF) if t & 0x8000 else t
            rf[ea(d)] = H
            pc += 1

SROM = srom()

def fill(port, glow):
    """the table fill: returns (profile words, cross bytes) from the engine's OUT ports.
    Profile word: E_Y(10) E_U(10) E_V(10) kq(2), E = K * (paint - ground) / 2 (signed), kq = K >> 6."""
    def s28(v):
        v &= 0xFFFFFFF
        return v - (1 << 28) if v & 0x8000000 else v
    g = port[P_G] & 0xFFFF
    c0 = (port[P_C0Y] & 1023, port[P_C0U] & 127, port[P_C0V] & 127)
    gnd = (port[P_GY] & 1023, port[P_GU] & 1023, port[P_GV] & 1023)
    pf, cr = [], []
    for p in range(512):
        Q = p * g
        def key(slot, gl):
            arg = min(s28(port[P_FCX + slot] + Q), s28(port[P_FCY + slot] - Q))
            if arg >= 65536: s = 255
            elif arg >= 0: s = SROM[arg >> 8]
            else:
                i = (arg >> 10) + 256
                return SROM[256 + i] if (gl and i >= 0) else 0
            return min(255, 64 + s - (s >> 2)) if gl else s
        u = 256
        for r in range(4):
            c = 255 - key(r, glow)
            u = (u * (c + (c >> 7))) >> 8
        k8 = min(255, 256 - u)
        acc = [c0[0] << 8, c0[1] << 8, c0[2] << 8]
        for r in range(1, 4):
            a = s28(port[P_FCX + 4 + r] + Q)                  # half the crossfade ramp (it runs at twice the gain)
            t = 0 if a < 0 else (255 if a >= 32768 else SROM[a >> 7])
            t += t >> 7
            w = port[P_FCY + 4 + r]
            dy = (w >> 16) & 0x7FF; dy -= 2048 if dy & 0x400 else 0
            du = (w >> 8) & 255; du -= 256 if du & 128 else 0
            dv = w & 255; dv -= 256 if dv & 128 else 0
            acc[0] += t * dy; acc[1] += t * du; acc[2] += t * dv
        P = (max(0, min(1023, acc[0] >> 8)), max(0, min(127, acc[1] >> 8)) << 3, max(0, min(127, acc[2] >> 8)) << 3)
        kp = k8 + (k8 >> 7)
        E = [max(-512, min(511, (kp * (P[i] - gnd[i])) >> 9)) & 1023 for i in range(3)]
        pf.append((E[0] << 22) | (E[1] << 12) | (E[2] << 2) | (k8 >> 6))
        cr.append(key(4, False))
    return pf, cr

def knobs_of(c):
    return [c['k1'], c['k2'], c['k3'], c['k4'], c['k5'], c['k6'], c['sw'], c['p12']]

# ------------------------------------------------------------------ package
def vhd_table(name, depth, width, vals):
    lines = ["    type t_%s is array (0 to %d) of std_logic_vector(%d downto 0);" % (name.lower(), depth - 1, width - 1),
             "    constant %s : t_%s := (" % (name, name.lower())]
    items = ['x"%0*X"' % (width // 4, v & ((1 << width) - 1)) for v in vals]
    for i in range(0, len(items), 16):
        lines.append("        " + ", ".join(items[i:i + 16]) + ("," if i + 16 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)

def settled(snap=False):
    """the engine as the bitstream starts it: glides settled at the default knobs, LFOs at their start phases"""
    eng = Engine(snap=True)
    st = knobs_of(DEFAULTS); st[3] = 512                                                # settle with the speed at rest
    for _ in range(2): eng.run(st)
    eng.snap = snap
    return eng

def main():
    eng = settled()
    eng.commit()
    rf_init = [v & 0xFFFFFFFF for v in eng.rf]              # sign-extended to 32 bits
    port = eng.port
    pf, cr = fill(port, glow=True)
    fcx = [port[P_FCX + i] & 0xFFFFFFFF for i in range(8)] + [0] * 248
    fcy = [port[P_FCY + i] & 0xFFFFFFFF for i in range(8)] + [0] * 248
    nz = [noise_rom(s) for s in (11, 23, 37, 51)]
    parts = ["-- Generated by colorfield.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package colorfield_pkg is",
             vhd_table("C_CF_UC", 256, 32, eng.uc),
             vhd_table("C_CF_RF", 256, 32, rf_init),
             vhd_table("C_CF_DROM", 1024, 8, eng.drom),
             vhd_table("C_CF_EXP2", 256, 16, eng.exp2),
             vhd_table("C_CF_SROM", 512, 8, eng.srom),
             vhd_table("C_CF_NA", 512, 8, nz[0]), vhd_table("C_CF_NB", 512, 8, nz[1]),
             vhd_table("C_CF_NC", 512, 8, nz[2]), vhd_table("C_CF_ND", 512, 8, nz[3]),
             vhd_table("C_CF_FC", 256, 32, fcx[:8] + fcy[:8] + [0] * 240),
             vhd_table("C_CF_PF", 512, 32, pf), vhd_table("C_CF_CR", 512, 8, cr)]
    for nm, w, v in (("C_CF_I_G", 16, port[P_G]), ("C_CF_I_C0Y", 10, port[P_C0Y]), ("C_CF_I_C0U", 7, port[P_C0U]),
                     ("C_CF_I_C0V", 7, port[P_C0V]), ("C_CF_I_GY", 10, port[P_GY]), ("C_CF_I_GU", 10, port[P_GU]),
                     ("C_CF_I_GV", 10, port[P_GV]), ("C_CF_I_AMP", 6, port[P_AMP]), ("C_CF_I_SCI", 4, port[P_SCI]),
                     ("C_CF_I_NZ0", 15, port[P_NZ]), ("C_CF_I_NZ1", 15, port[P_NZ + 1]), ("C_CF_I_NZ2", 15, port[P_NZ + 2]),
                     ("C_CF_I_NZ3", 15, port[P_NZ + 3]), ("C_CF_MAIN", 8, eng.A.main)):
        parts.append("    constant %s : unsigned(%d downto 0) := to_unsigned(%d, %d);" % (nm, w - 1, v & ((1 << w) - 1), w))
    parts += ["end package colorfield_pkg;", ""]
    with open(os.path.join(HERE, "colorfield_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))

if __name__ == "__main__":
    main()
