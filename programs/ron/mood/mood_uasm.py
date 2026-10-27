#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
#
# This file is part of Videomancer Community Programs.
# See LICENSE file in the repository root for full license text.
#
"""MOOD microcode assembler (DAMIER's engine, 16-bit datapath, 1024 words).

One small engine runs the virtual-time sample program once per virtual line
(15.734 kHz): a single adder ALU, a serial shifter, a radix-4 Booth serial
multiplier, a ROM unit (quarter sine + raw tables), a block-RAM register file
(two copies for dual read) and the program in block RAM.

Instruction: 32 bits
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6

Immediates overlap the fields an op does not use:
  LDI            imm14 = [13:0]  (sign-extended)
  AND            imm13 = [12:0]  (zero-extended; srcB unused)
  JMP JNZ        target = [9:0]
  JLT JGE        target = [23:20] & [5:0]   (a branch writes no register)
  SHR SHL        count [3:0]
  MRD            shift [4:0]  (0..16: dst = product >> shift, low 16 bits)
  SIN            [1] = 0: sine of A, [0] = cos
                 [1] = 1: raw table read, ROM 512 + ((A + 16 * bank) mod 512),
                          bank = [5:2] & [0]
  PRT            port [4:0], value = A
  CTL            input [3:0]
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'MUL': 10,   # start A * B (16 x 16 signed)
    'MRD': 11,   # dst = product >> imm (stalls until the multiply is idle)
    'SIN': 14,   # dst = sin(A) Q15 (cos when imm=1); imm=2: raw ROM table
    'LDI': 15,   # dst = imm14
    'PRT': 17,   # port imm <= A
    'JMP': 18, 'JNZ': 19, 'JLT': 20, 'JGE': 21,
    'END': 24,   # halt until the next sample tick
    'CTL': 25,   # dst = control input imm
    'JR':  26,   # pc = A
    'AND': 27,   # dst = A and imm13
}

# output ports (fabric registers)
PORTS = {
    'E1': 0,      # channel 1 ENERGY, Q5.8 (held)
    'E2': 1,      # channel 2 ENERGY; writes {E1, E2} into the line ring
    'MODE': 2,    # [1:0] H mult, [3:2] V mult, [5:4] shape (pending)
    'U1': 3, 'V1': 4, 'U2': 5, 'V2': 6,   # chroma offsets, unswapped BT.601
    'AAS': 7,     # CLEAN: pixels per reference unit, Q8.8
    'AAW': 8,     # CLEAN: ring width in pixels, Q8.8
    'FRM': 9,     # frame-task pulse (debug / bring-up)
}

# control inputs
CTLS = {
    'k1': 0, 'k2': 1, 'k3': 2, 'k4': 3, 'k5': 4, 'k6': 5, 'p12': 6,
    'sw': 7,      # [1:0] H array (Gray-decoded), [3:2] V array, [4] SOURCE ext,
                  # [7:5] format class (0 1080p, 1 1080i, 2 720p, 3 480i,
                  #       4 480p, 5 576i, 6 576p)
    'rnd': 8,     # free-running 16-bit LFSR
}

ROM_RAW = 512     # raw tables start here in the 1024-word ROM


def raw(bank):
    """SIN immediate for a raw ROM read from a 16-word bank (0..31)"""
    assert 0 <= bank < 32
    return 2 | (bank >> 1) << 2 | (bank & 1)


DEPTH = 1024      # program words


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []
        self.regs = {}
        self.init = {}
        self._n = 0

    # register allocation by name; init = power-up value (EBR initial content)
    def r(self, name, init=None):
        if name not in self.regs:
            assert len(self.regs) < 128, 'register file full'
            self.regs[name] = len(self.regs)
        if init is not None:
            assert -32768 <= init < 65536, (name, init)
            old = self.init.get(name)
            assert old is None or old == init, (name, old, init)
            self.init[name] = init
        return self.regs[name]

    def const(self, value):
        """a register that holds a constant from configuration"""
        name = f'#{value}'
        self.r(name, value)
        return name

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def lbl(self):
        self._n += 1
        return f'__m{self._n}'

    def emit(self, op, dst=None, a=None, b=None, imm=0):
        R = lambda x: 0 if x is None else (self.r(x) if isinstance(x, str) else x)
        if op == 'ABS':
            l1, l2 = self.lbl(), self.lbl()
            self.emit('JLT', a=a, b='zero', imm=l1)
            self.emit('MOV', dst, a)
            self.emit('JMP', imm=l2)
            self.label(l1)
            self.emit('SUB', dst, 'zero', a)
            self.label(l2)
            return
        if op == 'NEG':
            return self.emit('SUB', dst, 'zero', a)
        assert op in OPS, f'{op} is not implemented'
        if op in ('SHR', 'SHL'):
            assert 0 <= imm <= 15, imm
        if op == 'MRD':
            assert 0 <= imm <= 16, imm
        if op == 'AND':
            assert 0 <= imm < 8192, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -8192 <= imm < 8192, imm
        if op == 'PRT' and isinstance(imm, str):
            imm = PORTS[imm]
        if op == 'CTL' and isinstance(imm, str):
            imm = CTLS[imm]
        if isinstance(imm, str):
            self.fixups.append((len(self.code), imm))
            imm = 0
        self.code.append([OPS[op], R(dst), R(a), R(b), imm])
        return len(self.code) - 1

    def resolve(self):
        for idx, name in self.fixups:
            assert name in self.labels, f'undefined label {name}'
            self.code[idx][4] = self.labels[name]
        return self.code

    def words(self):
        inv = {v: k for k, v in OPS.items()}
        out = []
        for op, dst, a, b, imm in self.resolve():
            name = inv[op]
            w = op << 27
            if name in ('JLT', 'JGE'):
                assert 0 <= imm < DEPTH
                w |= ((imm >> 6) & 0xF) << 20 | a << 13 | b << 6 | (imm & 0x3F)
            elif name in ('JMP', 'JNZ'):
                assert 0 <= imm < DEPTH
                w |= a << 13 | imm
            elif name == 'LDI':
                w |= dst << 20 | (imm & 0x3FFF)
            elif name == 'AND':
                w |= dst << 20 | a << 13 | (imm & 0x1FFF)
            else:
                assert 0 <= imm < 64, (name, imm)
                w |= dst << 20 | a << 13 | b << 6 | imm
            out.append(w)
        return out

    def rf_init(self):
        v = [0] * 128
        for name, idx in self.regs.items():
            v[idx] = self.init.get(name, 0) & 0xFFFF
        assert self.init.get('zero', 0) == 0
        return v
