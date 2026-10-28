#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
SEISMIC - build hook.  Generates seismic_pkg.vhd (pure Python, no numpy).

The photo (WIN_20230610_00_10_54) is one FM horizontal oscillator (a chirp: the frequency is a
horizontal ramp folded near x 1100-1350) against a clipped vertical triangle, keyed per channel,
with the frame assembled from 24 strips of 45 lines whose sweep parameters step through an
8-strip staircase.  The data below was fitted in .heirloom_work/rb_seismic/:
  C_TABR/G/B 128 x 8   the gap colourizer: gap-plateau colour by the UNWRAPPED carrier phase,
                       psi = -8 .. 24 cycles in quarter cycles (fit_tab1d.py)
  C_SEG      256 x 16  per strip (24 x 8 entries): segment end (2-px units, bits 15..6) and the
                       R, G, B gap levels (2 bits each: 0, .35, .7, 1.05) (fit_seg.py / mkseg.py)
  C_HD       256 x 8   cusp half-width (1/256 cycle) by the vertical position W (circular lobes)
  C_SHUF      32 x 12  Shuffle: strip phase mu (Q9) per strip;  C_HSH 32 x 8: per-strip slip hash
  C_UC       256 x 32  the engine microcode (field program at 0, line program at U_LINE / U_LCOMP), assembled
                       below from FIELD / LINE (see "Engine" in seismic.vhd for the instruction set)
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
TABR = [
    71, 71, 71, 60, 40, 39, 66, 101, 124, 137, 155, 175, 190, 201, 208, 208,
    209, 216, 227, 232, 229, 221, 213, 207, 211, 223, 235, 238, 230, 223, 225, 225,
    220, 217, 219, 217, 207, 201, 201, 197, 187, 179, 176, 171, 162, 153, 148, 145,
    141, 132, 120, 111, 109, 105, 97, 93, 90, 81, 74, 72, 70, 60, 52, 56,
    45, 25, 18, 36, 56, 54, 64, 98, 126, 136, 138, 143, 140, 99, 36, 10,
    14, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
]
TABG = [
    31, 31, 31, 66, 154, 225, 236, 227, 226, 234, 242, 235, 222, 226, 227, 208,
    191, 201, 224, 226, 209, 200, 201, 194, 186, 197, 224, 229, 209, 204, 221, 226,
    217, 220, 236, 239, 228, 229, 242, 245, 236, 235, 245, 248, 240, 237, 245, 249,
    239, 226, 218, 212, 198, 185, 186, 189, 183, 177, 178, 175, 168, 185, 212, 210,
    208, 225, 232, 216, 171, 133, 147, 192, 230, 245, 247, 244, 236, 206, 149, 94,
    53, 26, 17, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
]
TABB = [
    77, 77, 77, 96, 155, 217, 238, 237, 231, 218, 207, 201, 194, 179, 161, 152,
    153, 146, 128, 119, 129, 134, 120, 115, 134, 149, 140, 137, 156, 163, 140, 123,
    131, 132, 109, 96, 105, 104, 83, 75, 92, 103, 93, 91, 109, 124, 119, 109,
    117, 124, 110, 100, 126, 158, 158, 157, 172, 173, 170, 179, 178, 182, 216, 239,
    238, 237, 234, 236, 207, 140, 118, 144, 159, 150, 128, 101, 75, 73, 88, 71,
    35, 14, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
]
SEG = [
    12480, 19903, 35455, 41344, 50959, 56896, 61502, 65472, 12544, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    14016, 23935, 25941, 39422, 43712, 45174, 61471, 65472, 19840, 47167, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47402, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 14121, 15701, 37823, 61440, 65472, 65472, 65472,
    12480, 47418, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472, 12800, 15871, 20193, 47103, 61440, 65472, 65472, 65472,
    12736, 20223, 35243, 50917, 56816, 61503, 65472, 65472, 12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 47419, 61440, 65472, 65472, 65472, 65472, 65472,
    12736, 27711, 29498, 34367, 41279, 47296, 61462, 65472, 12480, 47419, 61440, 65472, 65472, 65472, 65472, 65472,
    12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472, 12480, 47423, 61440, 65472, 65472, 65472, 65472, 65472,
    12352, 41467, 51071, 55153, 61502, 65472, 65472, 65472, 12416, 29419, 36907, 41361, 51067, 61481, 65472, 65472,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
]
HD = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 7, 12, 17, 22, 26,
    31, 35, 39, 42, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 36, 35,
    34, 33, 32, 31, 31, 30, 29, 28, 28, 27, 26, 25, 25, 24, 24, 23,
    22, 22, 21, 20, 20, 19, 19, 18, 18, 17, 17, 16, 16, 15, 15, 14,
    14, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8,
    7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3,
    3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
]
SHUF = [
    3347, 3464, 3581, 3698, 3815, 3932, 4049, 70, 187, 304, 421, 538, 655, 772, 889, 1006,
    1123, 1240, 1357, 1474, 1591, 1708, 1825, 1942, 0, 0, 0, 0, 0, 0, 0, 0,
]
HSH = [
    224, 52, 136, 219, 47, 131, 215, 43, 127, 211, 38, 122, 206, 34, 118, 202,
    29, 113, 197, 25, 109, 193, 20, 104, 0, 0, 0, 0, 0, 0, 0, 0,
]


# ------------------------------------------------------------------------------------------------
# Engine microcode.  One 32-bit accumulator ACC; register file RF (64 x 32, block RAM); input ports
# (knobs / beam state) and output registers (field values, next-line values) in fabric.
#   word: op[31:28]  sh[27:24]  reg[23:18]  imm19[18:0] (MULI)  imm28[27:0] (LDI / ADDI)
#   NOP | LDI imm | ADDI imm | LD r,s | ADD r,s | SUB r,s   (operand = RF[r] >>> s, s 0..15)
#   ST r | OUT o | IN p | MAX0 | END
#   MUL r,s | MULI imm,s | MULP p,s :  ACC <- (ACC[24:0] x B[18:0]) >>> s  (signed; Booth radix 2)
# Programs: FIELD (vblank); LINE = a copy prologue (the next line's values, computed into RF during the
# line before, go to the beam registers in hblank) falling through into LCOMP (compute the line after).
# Pipeline: one instruction per clock (a multiply stalls ~22 clocks).  An RF register stored by
# instruction i may be read from instruction i + 4 on: the assembler inserts NOPs.
OPS = dict(NOP=0, LDI=1, ADDI=2, LD=3, ADD=4, SUB=5, ST=6, OUT=7, IN=8, MAX0=9, MUL=10, MULI=11, MULP=12, END=15)
PORTS = dict(X1=0, X2=1, X3=2, X5=3, P=4, TSX=5, V=6, Y4=7, EA=8, HSH=9)
OUTS = dict(S1R=0, FLR=1, FRR=2, DX0=3, K2X=4, KXO=5, LV1=6, LV2=7, LV3=8, XLW=9, XRW=10, LOB=11,
            L0=12, CA=13, G0=14, XA=15, XF=16, PSI=17, PHV=18, HGW=19, CY=20, OY=21, GINC=22)
RF_HAZARD = 4

FIELD = """
    IN X1; ADDI -512; MAX0; MULI 4,0; ST T0
    IN X1; MULI 4,0; ADDI 2048; ADD T0; MULI 81741,12; ST KV
    IN X2; ADDI -512; MULI 37,3; ADDI 4096; ST T1
    IN X2; ADDI -512; MAX0; MULI 49,3; ADD T1; ST SW
    MULI 27914,10; ST S1V; MULP TSX,10; MULI 1,10; ST S1X; MULP TSX,12; OUT S1R; MULI -67,1; OUT GINC
    LD SW; MULI 30199,12; ST FLV; MULP TSX,12; OUT FLR
    LD SW; MULI 26844,12; MULP TSX,12; OUT FRR
    LD S1V; MULI 61009,9; ADDI -119453778; ST PSIA
    IN X3; ADDI -511; MAX0; ST T2
    IN X3; ADDI -512; MAX0; ST WIDE
    IN X3; SUB T2,0; MULI 1,1; ST LOB; OUT LOB
    LD WIDE; MULI -780,10; ADDI 390; OUT XLW
    LD WIDE; MULI 880,10; ADDI 1480; OUT XRW
    LD LOB,2; ADD LOB,4; ADD LOB,5; OUT LV1
    LD LOB,1; ADD LOB,3; ADD LOB,4; OUT LV2
    LD LOB,0; ADD LOB,4; OUT LV3
    IN X5; ADDI -512; MULI 29,3; ADDI 4096; ST WID
    IN TSX; MULP TSX,15; ST K2; OUT K2X
    IN TSX; MULI 93,12; OUT KXO
    IN TSX; MULI -120,0; ADD K2,1; OUT DX0
    IN V; MULI 1,3; MUL KV,8; ADD VROLL,0; ST VROLL
    IN P; ADDI -512; MAX0; MULI 45,0; ST PQ
    LD OY0; ADDI -197; ST OY0
    END
"""

LINE = """
    LD NL0; OUT L0; LD NCA; OUT CA; LD NG0; OUT G0; LD NXA; OUT XA; LD NXF; OUT XF
    LD NPSI; OUT PSI; LD NPHV; OUT PHV; LD NHGW; OUT HGW; LD NCY; OUT CY; LD NOY; OUT OY
"""

LCOMP = """
    IN EA; MULP P,9; ST QA
    IN EA; MULI 5,2; ADDI 10854; MULP EA,9; MULP P,14; ADDI -1019; ST D4
    LD PQ; MULP HSH,12; ST SL
    LDI 20000; ADD QA,1; ADD QA,3; ST UF4
    LDI 4880; SUB QA,2; SUB QA,4; ST XL4
    LDI 112; SUB QA,7; SUB QA,11; MUL WID,12; MAX0; ST NHGW
    IN Y4; ADD SL,0; ADDI -2032; MUL KV,4; ADD VROLL,0; ST NPHV
    LDI -4480; SUB D4,0; MUL S1X,4; ST NL0
    LDI 4480; SUB XL4,0; MUL S1X,4; ST NCA
    LD D4; ADD XL4,0; ADDI 1104; MUL FLV,4; ST T1
    LDI 8432; SUB XL4,0; ST NN
    LD UF4; ADDI -4480; MUL S1X,4; MULI 99,1; ST T3
    LD NL0; MULI -67,1; ADD T3,0; ST NG0
    LD D4; ADDI 8432; ST NXA
    LD UF4; ADD D4,0; ST NXF
    LD NN; MUL NN,4; ADDI -76176; MUL S1V,13; ST T2
    IN Y4; ADDI -26400; ST DY
    IN Y4; MULI 126,4; ADD OY0,0; ST NOY
    LD PSIA; SUB T1,0; SUB T2,0; ST NPSI
    LD DY; MUL DY,6; MULI 23,0; ADDI -65440000; ST NCY
    END
"""


def _parse(src):
    out = []
    for line in src.strip().splitlines():
        for ins in line.split(';'):
            ins = ins.strip()
            if ins:
                p = ins.split(None, 1)
                out.append((p[0], [a.strip() for a in p[1].split(',')] if len(p) > 1 else []))
    return out


def assemble():
    """-> (words[256], entries{name: addr}, listing[(addr, text)], regs{name: idx})"""
    regs = {}

    def reg(n):
        if n not in regs:
            regs[n] = len(regs)
        assert regs[n] < 64
        return regs[n]
    words, listing, entries = [], [], {}
    for name, src in (('FIELD', FIELD), ('LINE', LINE), ('LCOMP', LCOMP)):
        entries[name] = len(words)
        if name != 'LCOMP':
            last_st = {}
        for op, a in _parse(src):
            rd = None
            if op in ('LD', 'ADD', 'SUB', 'MUL'):
                rd = reg(a[0])
            while rd is not None and rd in last_st and len(words) < last_st[rd] + RF_HAZARD:
                words.append(0); listing.append((len(words) - 1, 'NOP'))
            w = OPS[op] << 28
            if op in ('LDI', 'ADDI'):
                v = int(a[0]); assert -(1 << 27) <= v < (1 << 27), (op, a)
                w |= v & ((1 << 28) - 1)
            elif op == 'MULI':
                v, s = int(a[0]), int(a[1]); assert -(1 << 18) <= v < (1 << 18) and 0 <= s < 16, (op, a)
                w |= (s << 24) | (v & ((1 << 19) - 1))
            elif op in ('LD', 'ADD', 'SUB', 'MUL'):
                s = int(a[1]) if len(a) > 1 else 0; assert 0 <= s < 16
                w |= (s << 24) | (rd << 18)
            elif op == 'MULP':
                s = int(a[1]); assert 0 <= s < 16
                w |= (s << 24) | (PORTS[a[0]] << 18)
            elif op == 'IN':
                w |= PORTS[a[0]] << 18
            elif op == 'OUT':
                w |= OUTS[a[0]] << 18
            elif op == 'ST':
                r = reg(a[0]); w |= r << 18; last_st[r] = len(words)
            words.append(w); listing.append((len(words) - 1, op + (' ' + ','.join(a) if a else '')))
    assert len(words) <= 256
    return words + [OPS['END'] << 28] * (256 - len(words)), entries, listing, regs


def emit():
    def arr(name, n, bits, vals):
        s = ['"' + format(v & ((1 << bits) - 1), '0%db' % bits) + '"' for v in vals]
        rows = ['        ' + ', '.join(s[i:i + 8]) for i in range(0, n, 8)]
        return ('    type t_%s is array(0 to %d) of std_logic_vector(%d downto 0);\n' % (name.lower(), n - 1, bits - 1)
                + '    constant %s : t_%s := (\n' % (name, name.lower()) + ',\n'.join(rows) + '\n    );')
    pad = lambda t: t + [t[-1]] * (256 - len(t))          # colourizer: 256 deep so it maps to block RAM
    uc, ent, _, _ = assemble()
    consts = [arr('C_TABR', 256, 8, pad(TABR)), arr('C_TABG', 256, 8, pad(TABG)), arr('C_TABB', 256, 8, pad(TABB)),
              arr('C_SEG', 256, 16, SEG), arr('C_HD', 256, 8, HD), arr('C_SHUF', 32, 12, SHUF), arr('C_HSH', 32, 8, HSH),
              arr('C_UC', 256, 32, uc),
              '    constant U_LINE : integer := %d;' % ent['LINE'], '    constant U_LCOMP : integer := %d;' % ent['LCOMP']]
    txt = ('-- seismic_pkg.vhd -- GENERATED by seismic.py; edit the generator, not this file.\n\n'
           'library ieee;\nuse ieee.std_logic_1164.all;\n\npackage seismic_pkg is\n'
           + '\n'.join(consts) + '\nend package seismic_pkg;\n')
    with open(os.path.join(HERE, 'seismic_pkg.vhd'), 'w') as fh:
        fh.write(txt)


if __name__ == '__main__':
    emit()
