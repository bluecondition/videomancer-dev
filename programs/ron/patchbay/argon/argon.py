#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
ARGON v0.1.0 - build hook.  Writes argon_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  The integer model (.patchbay_work/rb_argon/imodel.py) imports this file for the same tables,
the same microcode and the reference emulator of the patch computer.

The patch (virtual 1920 x 1080 coordinates, centre 960, 540):
  circle generator: two squared ramps -> (x^2 + y^2 - R^2) / 2R            = the ring's distance
  triangle / hexagon: rotated ramps through analog max (|.| for the hexagon) = the polygons' distances
  each distance -> window comparator + glow lowpass = a per-tube table of light against |distance|
  three tubes -> additive RGB mixer -> the camera's shoulder (white cores) -> Y / U / V

The patch computer: a small microcoded engine (one serial multiplier) that runs once per field.  Part 1 (in
the vertical-blanking zone) computes the geometry: the edge DDA coefficients, the ring's quadratic DDA, the
apothems, the double-tube gap and the camera gain.  Part 2 (during the field) computes the glow-table
constants; the table writer then rebuilds the three tube tables in the back bank.

Instruction word (32 bits, two 16-bit ROM words): op[31:28] d[27:20] a[19:12] b[11:4] c[3:0]; the shift x =
XS[c] (16 values).  A = RF[a], B = RF[b]:
  0 END   stop                        8 MAX   max(A, B)
  1 MULU  (A * B[15:0]u) >>> x        9 SEL   N ? A : B   (N = sign of the previous result; SEL keeps N)
  2 MULS  (A * B[15:0]s) >>> x       10 EXP2  E[A & 255] shifted left by (A >>> 8) + x - 15 (right if < 0)
  3 ADD   A + (B << x)               11 SIN   Q15 sine of A[31:16] (65536 per turn); x = 1: cosine
  4 ADDR  A + (B >>> x)              12 IN    input b (0..19)
  5 SUB   A - (B << x)               13 LDX   RF[(A + b) & 255]   (b = the field value)
  6 SUBR  A - (B >>> x)              14 (unused)
  7 MIN   min(A, B)                  15 SYNC  geometry ready: wait for the sweep
Results are 32-bit two's complement (wrapping); every result except SEL's sets N; RF[0] = 0 is never written.

Tables:
  C_AR_EXP2   256 x 16  round(32768 * 2^(i/256))
  C_AR_SIN    512 x 16  round(32767 * sin(pi/2 * (j + 1/2) / 512)), a quarter wave
  C_AR_UC0..1 256 x 16  the microcode (word bits 15:0, 31:16)
  C_AR_RF     256 x 32  the register file's power-up image (constants, persistent variables)
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

OPS = dict(END=0, MULU=1, MULS=2, ADD=3, ADDR=4, SUB=5, SUBR=6, MIN=7, MAX=8, SEL=9, EXP2=10, SIN=11,
           IN=12, LDX=13, SYNC=15)
XS = [0, 1, 2, 3, 6, 8, 10, 11, 12, 14, 15, 16, 19, 20, 25, 28]   # shift codes

# gas wheel (emission colour, 0..1): argon blue-violet, pink, neon red-orange, mercury green, white
GAS = [(0.30, 0.20, 1.00), (1.00, 0.16, 0.60), (1.00, 0.24, 0.03), (0.14, 1.00, 0.26), (1.00, 0.86, 0.72)]
GAS_OFF = (0, 2, 3)          # ring, triangle / square, hexagon / diamond (gas steps)
TU = 384                     # table units: light 1.0 = 384

# register-file map
A_EDGE = 0x80                # 4 edges (0, 1, 3, 4) x {wrap, line step, pixel step, seed}
A_DEDGE = 0x90               # the ring's D as a fifth bus edge: wrap (-N DD), 0, DD, D0
A_F0, A_G0, A_GG, A_HA, A_HB, A_GAP, A_CG = range(0x9A, 0xA1)
A_PC, A_ICW, A_KB, A_KH, A_VB, A_VH, A_VW = range(0xB0, 0xB7)
A_VC = 0xB8                  # 3 tubes: core (white) light
A_CR, A_CGN, A_CB = 0xBB, 0xBE, 0xC1   # 3 tubes each: coloured light per channel
KW = 338                     # wash exponent constant (tau_w = 280 px)

U32 = 0xFFFFFFFF

def s32(v):
    v &= U32
    return v - (1 << 32) if v & 0x80000000 else v

def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]

def sin_rom():
    return [int(round(32767 * math.sin(math.pi / 2 * (j + 0.5) / 512))) for j in range(512)]

def deg(a):
    return int(round(a / 360.0 * 2 ** 32)) & U32

class Asm:
    def __init__(self):
        self.code = []
        self.rf = [0] * 256
        self.names = {'ZERO': 0}
        self.consts = {}
        self.nxt = 1
        self.fixed = set(range(0x80, 0xC4))

    def alloc(self):
        while self.nxt in self.fixed:
            self.nxt += 1
        r = self.nxt; self.nxt += 1
        assert r < 256, 'register file full'
        return r

    def const(self, v):
        v = s32(v)
        if v not in self.consts:
            r = self.alloc(); self.rf[r] = v; self.consts[v] = r
        return self.consts[v]

    def var(self, name, init=0):
        r = self.alloc(); self.names[name] = r; self.rf[r] = s32(init)
        return r

    def r(self, n):
        if isinstance(n, int):
            return n
        if n not in self.names:
            self.names[n] = self.alloc()
        return self.names[n]

    def op(self, op, d=0, a=0, b=0, x=0):
        d, a = self.r(d), self.r(a)
        if op == 'IN':
            b, x = x, 0
        b = b if (op in ('LDX', 'IN') and isinstance(b, int)) else self.r(b)
        assert 0 <= b < 256 and x in XS, (op, b, x)
        self.code.append((OPS[op] << 28) | (d << 20) | (a << 12) | (b << 4) | XS.index(x))

def program():
    A = Asm()
    C = A.const
    o = A.op
    # persistent variables (power-up values)
    A.var('TH', deg(90))            # triangle angle: pointing up
    A.var('PH', 0)                  # hexagon angle
    A.var('BPH', 0)                 # breathing LFO phase
    # gas wheel (table units), entry 5 = entry 0, and the step to the next gas
    for ch in range(3):
        vals = [int(round(GAS[i % 5][ch] * TU)) for i in range(6)]
        A.names['GAS%d' % ch] = A.nxt
        for i in range(5):
            A.var('gas%d_%d' % (ch, i), vals[i])
        A.names['DGAS%d' % ch] = A.nxt
        for i in range(5):
            A.var('dgas%d_%d' % (ch, i), vals[i + 1] - vals[i])
    # ---------------- part 1: geometry (vertical-blanking zone) ----------------
    o('IN', 'sxh', x=12); o('IN', 'ys', x=13); o('IN', 'sw', x=14); o('IN', 'wh', x=15)
    o('SUB', 'nsy', 'ZERO', 'ys'); o('SUB', 'nsw', 'ZERO', 'sw'); o('SUB', 'nwh', 'ZERO', 'wh')
    # Motor: bipolar, +-8 deadband, square law -> the triangle turns, the hexagon the other way at 3/4
    o('IN', 't', x=1)
    o('SUB', 'm', 't', C(512))
    o('SUB', 'am', 'ZERO', 'm')
    o('MAX', 'am', 'am', 'm')
    o('SUB', 'am', 'am', C(8))
    o('MAX', 'am', 'am', 'ZERO')
    o('MULU', 'sq', 'am', 'am')
    o('MULU', 'inc', 'sq', C(141))
    o('SUB', 'ninc', 'ZERO', 'inc')
    o('ADD', 't', 'm', 'ZERO')              # N = (m < 0)
    o('SEL', 'dth', 'ninc', 'inc')
    o('ADD', 'TH', 'TH', 'dth')
    o('SUBR', 'dph', 'dth', 'dth', 2)       # 3/4
    o('SUB', 'PH', 'PH', 'dph')
    # edge angles: Round = triangle normals at TH + 180 + 120k, hexagon PH + 60k;
    #              Square = square TH + 90k, diamond PH + 45 + 90k (the third edge repeats the first)
    # (the third edge of each figure is formed in the pixel path: triangle -(u0 + u1), hexagon w1 - w0,
    #  square / diamond = the first edge again)
    o('ADD', 'aR0', 'TH', C(deg(180))); o('ADD', 'aR1', 'TH', C(deg(300)))
    o('ADD', 'aS1', 'TH', C(deg(90)))
    o('ADD', 'bR1', 'PH', C(deg(60)))
    o('ADD', 'bS0', 'PH', C(deg(45))); o('ADD', 'bS1', 'PH', C(deg(135)))
    o('IN', 't', x=7)                       # N = S7 (Square)
    o('SEL', 'e0', 'TH', 'aR0'); o('SEL', 'e1', 'aS1', 'aR1')
    o('SEL', 'e3', 'bS0', 'PH'); o('SEL', 'e4', 'bS1', 'bR1')
    # edges: u = cos(a) X - sin(a) Y (virtual px, Q13): pixel step, line step, wrap (-N pixel steps), seed
    for k in (0, 1, 3, 4):
        e = 'e%d' % k; base = A_EDGE + 4 * (k if k < 2 else k - 1)
        o('SIN', 'c', e, x=1)
        o('SIN', 's', e, x=0)
        o('MULS', base + 2, 'c', 'sxh', 3)          # pixel step = c sx
        o('MULS', base + 1, 's', 'nsy', 10)         # line step = -s sy
        o('MULS', base + 0, base + 2, 'nsw', 0)     # wrap = -N pixel steps
        o('MULS', 't3', 'c', 'nwh', 2)              # c X0
        o('MULS', 't4', 's', C(540), 2)             # -s Y0
        o('ADD', base + 3, 't3', 't4')              # seed
    # ring: K = 1 / 2R; DDA f += D >> 8 (Q20), D += DD (Q28); line DDA F0 += G >> 8, G += GG
    o('IN', 'g1', x=0)
    o('SUB', 'k', 'g1', C(512))
    o('MIN', 'kn', 'k', 'ZERO'); o('MAX', 'kp', 'k', 'ZERO')
    o('MULS', 't', 'kn', C(192), 8); o('MULS', 't2', 'kp', C(87), 8); o('ADD', 'eS', 't', 't2')
    o('MULS', 'eR', 'eS', C(31), 8)                 # the ring follows the nest at 0.12 of its octaves
    o('ADD', 'eRS', 'eR', 'eS')
    o('SUB', 't', C(12), 'eR')
    o('EXP2', 'Kq', 't', x=28)                       # K 2^38
    o('MULU', 'sx2', 'sxh', 'sxh')
    o('MULU', A_DEDGE + 2, 'Kq', 'sx2', 11)          # DD
    o('MULS', A_DEDGE + 0, A_DEDGE + 2, 'nsw', 0)    # wrap = -N DD
    o('ADD', A_DEDGE + 1, 'ZERO', 'ZERO')
    o('SUB', 'u', 'sxh', 'wh', 2)
    o('MULU', 'T4', 'u', 'sxh')
    o('MULS', A_DEDGE + 3, 'Kq', 'T4', 12)           # D0
    o('MULU', 'w2', 'wh', 'wh')
    o('ADD', 'c0', 'w2', C(291600))
    o('ADDR', 'c0s', 'ZERO', 'c0', 6)
    o('MULU', 'F0a', 'Kq', 'c0s', 12)
    o('EXP2', 'Rh', 'eR', x=20)
    o('MULU', 'Rh2', 'Rh', C(248))
    o('SUB', A_F0, 'F0a', 'Rh2')
    o('SUB', 'u', 'ys', C(276480))
    o('MULU', 'ty', 'u', 'ys', 14)
    o('MULS', A_G0, 'Kq', 'ty', 12)
    o('MULU', 's2', 'ys', 'ys', 6)
    o('MULU', A_GG, 'Kq', 's2', 19)
    # apothems (Q4): triangle 0.5 R s, hexagon 0.52 R s cos 30; square 0.7071 R s, diamond 0.76 of that
    o('EXP2', 'v', 'eRS', x=15)
    o('IN', 't', x=7)
    o('SEL', 'cA', C(5612), C(3968)); o('SEL', 'cB', C(4265), C(3574))
    o('MULU', A_HA, 'v', 'cA', 15); o('MULU', A_HB, 'v', 'cB', 15)
    # tube core half-width tc = 1.6 2^(ec/256) px; the double-tube gap 2.2 tc + 10
    o('IN', 'g5', x=4); o('IN', 'gp', x=6)
    o('SUB', 'pm', 'gp', C(512))
    o('MULU', 't', 'g5', C(218), 8); o('MULS', 't2', 'pm', C(135), 8); o('ADD', 'ec', 't', 't2')
    o('EXP2', 'v', 'ec', x=15)
    o('MULU', 'tcq', 'v', C(26214), 25)
    o('MULU', 't', 'tcq', C(2253), 10)
    o('ADD', A_GAP, 't', C(160))
    # camera gain: Q4 px = (cell mean luma x gain) >> 8; full white at 100 % = 120 px
    o('IN', 'g6', x=5)
    o('SUB', 'g6', 'g6', C(4)); o('MAX', 'g6', 'g6', 'ZERO')      # a 4-count deadband: 0 % is exactly off
    o('MULU', A_CG, 'g6', C(563), 8)
    o('SYNC')
    # ---------------- part 2: glow-table constants (during the field) ----------------
    # core edge ramp width cw = 1.35 2^(0.55 ec/256); body (gaussian) width tb = 5.88 2^(0.72 ec/256)
    o('MULU', 'ec55', 'ec', C(141), 8)
    o('SUB', 't', 'ZERO', 'ec55'); o('EXP2', 'v', 't', x=15)
    o('MULU', A_ICW, 'v', C(12136), 15)
    o('EXP2', 'v', 'ec55', x=15)
    o('MULU', 'cwq', 'v', C(22118), 25)
    o('ADDR', A_PC, 'tcq', 'cwq', 1)
    o('MULU', 'ec72', 'ec', C(184), 8)
    o('SUB', 't', 'ZERO', 'ec72'); o('EXP2', 'v', 't', x=15)
    o('MULU', A_KB, 'v', C(13389), 15)
    # halo length tau = 5.2 2^(etau/256): K3 over 5.2 octaves, P12 +-1.5 octaves
    o('IN', 'g3', x=2)
    o('MULU', 't', 'g3', C(333), 8); o('MULS', 't2', 'pm', C(192), 8); o('ADD', 'etau', 't', 't2')
    o('SUB', 't', 'ZERO', 'etau'); o('EXP2', 'v', 't', x=15)
    o('MULU', A_KH, 'v', C(18181), 15)
    # light levels (Q10): lv = 0.4 + 0.6 min(1, 2v)
    o('ADD', 't', 'ZERO', 'gp', 1); o('MIN', 't', 't', C(1023))
    o('MULU', 't', 't', C(154), 8); o('ADD', 'lv', 't', C(410))
    # vc = (0.25 + 1.05 min(1, 1.7v) + 0.6 max(0, v - 0.6)) lv
    o('MULU', 't', 'gp', C(435), 8); o('MIN', 't', 't', C(1023))
    o('MULU', 't', 't', C(269), 8); o('ADD', 'vc0', 't', C(256))
    o('SUB', 't', 'gp', C(614)); o('MAX', 't', 't', 'ZERO')
    o('MULU', 't', 't', C(154), 8); o('ADD', 'vc0', 'vc0', 't')
    o('MULU', 'vc', 'vc0', 'lv', 10)
    # vb = (0.75 + 0.45 v) lv
    o('MULU', 't', 'gp', C(115), 8); o('ADD', 't', 't', C(768))
    o('MULU', A_VB, 't', 'lv', 10)
    # vh = 1.2 (0.5 + 0.95 v^2) breathe lv 2^((480 - K3) 0.00203)
    o('MULU', 'gsq', 'gp', 'gp', 10)
    o('MULU', 't', 'gsq', C(292), 8); o('ADD', 'h0', 't', C(614))
    o('ADD', 'BPH', 'BPH', C(7953544))           # 9 s at 60 fields / s
    o('SIN', 's', 'BPH')
    o('MULS', 't', 's', C(225), 15); o('ADD', 'br', 't', C(1024))
    o('MULU', 'h0', 'h0', 'br', 10)
    o('MULU', 'h0', 'h0', 'lv', 10)
    o('SUB', 't', C(480), 'g3'); o('MULS', 't', 't', C(133), 8)
    o('EXP2', 'kc', 't', x=10)
    o('MULU', A_VH, 'h0', 'kc', 10)
    # vw = Wall Wash ? 0.34 lv : 0
    o('MULU', 't', 'lv', C(87), 8)
    o('IN', 't2', x=10)
    o('SEL', A_VW, 't', 'ZERO')
    # gas: position on the wheel g4 * 5/1023 (Q8), per tube + its offset, wrapped, lerp between gases;
    # times the tube's brightness (Ballast)
    o('IN', 'g4', x=3)
    o('MULU', 'pq', 'g4', C(320), 8)
    o('MULU', 'vc96', 'vc', C(96))
    for t in range(3):
        if GAS_OFF[t]:
            o('ADD', 'p', 'pq', C(GAS_OFF[t] * 256))
            o('SUB', 'q', 'p', C(1280))
            o('SEL', 'p', 'p', 'q')
        else:
            o('ADD', 'p', 'pq', 'ZERO')
        o('ADDR', 'ga', 'ZERO', 'p', 8)
        o('SUB', 'f', 'p', 'ga', 8)
        o('IN', 'bt', x=16 + t)
        for ch, dst in enumerate((A_CR, A_CGN, A_CB)):
            o('LDX', 'c0', 'ga', A.names['GAS%d' % ch])
            o('LDX', 'dd', 'ga', A.names['DGAS%d' % ch])
            o('MULS', 'dd', 'dd', 'f', 8)
            o('ADD', 'c0', 'c0', 'dd')
            o('MULU', dst + t, 'c0', 'bt', 8)
        o('MULU', A_VC + t, 'vc96', 'bt', 16)
    o('END')
    assert len(A.code) <= 256, 'microcode too long: %d' % len(A.code)
    return A

# ------------------------------------------------------------------------------------------------
# reference emulator (the integer model runs this; the RTL implements the same semantics)
def run(A_or_code, rf, inp, exp2=None, sinr=None):
    code = A_or_code.code if hasattr(A_or_code, 'code') else A_or_code
    exp2 = exp2 or exp2_rom(); sinr = sinr or sin_rom()
    n = 0; pc = 0; sync_at = None
    while True:
        w = code[pc]; pc += 1
        op = w >> 28; d = (w >> 20) & 255; a = (w >> 12) & 255; b = (w >> 4) & 255; x = XS[w & 15]
        A = rf[a]; B = rf[b]
        R = None
        if op == 0:
            break
        elif op == 1 or op == 2:
            bb = B & 0xFFFF
            if op == 2 and bb & 0x8000: bb -= 0x10000
            R = s32((A * bb) >> x)
        elif op == 3: R = s32(A + (B << x))
        elif op == 4: R = s32(A + (B >> x))
        elif op == 5: R = s32(A - (B << x))
        elif op == 6: R = s32(A - (B >> x))
        elif op == 7: R = min(A, B)
        elif op == 8: R = max(A, B)
        elif op == 9: R = A if n else B
        elif op == 10:
            fr = A & 255; sh = (A >> 8) + x - 15; v = exp2[fr]
            R = s32(v << sh) if sh >= 0 else (v >> (-sh) if -sh < 32 else 0)
        elif op == 11:
            ang = ((A >> 16) + (0x4000 if x & 1 else 0)) & 0xFFFF
            q = ang >> 14; idx = (ang >> 5) & 511
            if q & 1: idx = 511 - idx
            v = sinr[idx]
            R = -v if q >= 2 else v
        elif op == 12: R = s32(inp[b])
        elif op == 13: R = rf[(A + b) & 255]
        elif op == 15:
            sync_at = pc
            continue
        if op != 9:
            n = 1 if R < 0 else 0
        if d != 0:
            rf[d] = R
    return sync_at

# ------------------------------------------------------------------------------------------------
def vhdl_pkg():
    A = program()
    e2 = exp2_rom(); sn = sin_rom()
    L = []
    L.append('-- Generated by argon.py (build hook).  Do not edit.')
    L.append('library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n')
    L.append('package argon_pkg is')
    L.append('    type t_ar_rom16 is array (natural range <>) of std_logic_vector(15 downto 0);')
    L.append('    type t_ar_rom32 is array (natural range <>) of std_logic_vector(31 downto 0);')
    def rom16(name, vals):
        L.append('    constant %s : t_ar_rom16(0 to %d) := (' % (name, len(vals) - 1))
        items = ['x"%04X"' % (v & 0xFFFF) for v in vals]
        for i in range(0, len(items), 12):
            L.append('        ' + ', '.join(items[i:i + 12]) + (',' if i + 12 < len(items) else ''))
        L.append('    );')
    def rom32(name, vals):
        L.append('    constant %s : t_ar_rom32(0 to %d) := (' % (name, len(vals) - 1))
        items = ['x"%08X"' % (v & U32) for v in vals]
        for i in range(0, len(items), 8):
            L.append('        ' + ', '.join(items[i:i + 8]) + (',' if i + 8 < len(items) else ''))
        L.append('    );')
    code = A.code + [0] * (256 - len(A.code))
    rom16('C_AR_EXP2', e2)
    rom16('C_AR_SIN', sn)
    rom16('C_AR_UC0', [w & 0xFFFF for w in code])
    rom16('C_AR_UC1', [(w >> 16) & 0xFFFF for w in code])
    rom32('C_AR_RF', A.rf)
    L.append('end package argon_pkg;')
    return '\n'.join(L) + '\n', A

if __name__ == '__main__':
    txt, A = vhdl_pkg()
    with open(os.path.join(HERE, 'argon_pkg.vhd'), 'w') as f:
        f.write(txt)
    print('argon_pkg.vhd: %d instructions, %d registers used' % (len(A.code), A.nxt))
