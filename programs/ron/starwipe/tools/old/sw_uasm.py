#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE microcode assembler (damier/cubist lineage, 24-bit datapath).

One small engine runs the per-frame work in vertical blanking: an adder ALU,
a serial shifter, a radix-4 Booth serial multiplier (24 x 24 -> 48), a serial
divider ((A << 16) / B, unsigned), a quarter-sine ROM, a block-RAM register
file (one copy, operands read on successive clocks) and the program in ROM.

Instruction: 32 bits
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6
Immediates overlap the fields an op does not use:
  LDI            imm20 = [19:0] (sign-extended)
  JMP            target = [12:0]
  JNZ JZ         A = [19:13], target = [12:0]
  JLT JGE        A, B; target = [26:20] & [5:0]
  SHR SHL        count = i6 (0..23)
  MUL            dst = (A * B) >> i6, low 24 bits (stalls until the product is in)
  DIV            dst = (A << 16) / B, unsigned, A < 256 B (stalls)
  ADDI ANDI      dst = A + / & imm13 = [12:0] (sign-extended)
  CALL           r127 = pc + 1, pc = [12:0]
  CTL            input = i6
  PRT            port = i6, value = A, aux = B
  MIN MAX        dst = min / max(A, B), signed
  LDM            dst = low 24 bits of program word A (data tables live in the ROM)
  DATA (pseudo)  a raw 24-bit data word
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5, 'AND': 6,
    'LDI': 7, 'MUL': 8, 'ADDI': 9, 'DIV': 10, 'ANDI': 11, 'SIN': 12, 'CTL': 13,
    'PRT': 14, 'JMP': 15, 'JNZ': 16, 'JLT': 17, 'JGE': 18, 'JR': 19, 'END': 20,
    'JZ': 21, 'SHLV': 22, 'SHRV': 23, 'MIN': 24, 'MAX': 25, 'LDM': 26, 'CALL': 27,
}
LINK = 127            # CALL writes the return address here; RET = JR link

PORTS = {
    'DUX': 0, 'DVX': 1, 'DUY': 2, 'DVY': 3, 'SU': 4, 'SV': 5,
    'FLAGS': 8, 'T': 9, 'DK': 12, 'TK': 13, 'BA': 14, 'BB': 15,
    'SN': 16, 'SS': 17, 'SAL': 18, 'BCY': 19, 'BCUV': 20, 'RCY': 21, 'RCUV': 22,
    'WALK': 23,
}

CTLS = {
    'k1': 0, 'k2': 1, 'k3': 2, 'k4': 3, 'k5': 4, 'k6': 5, 'sw': 6, 'p12': 7,
    'tid': 8, 'W': 9, 'H': 10, 'fld': 11, 'WBUSY': 12, 'DMIN': 13, 'DMAX': 14,
}

M24 = 0xFFFFFF


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []
        self.regs = {}
        self.aliases = {}
        self._n = 0

    def alias(self, **kw):
        """section-local names for pooled temporaries"""
        self.aliases.update(kw)

    def r(self, name):
        name = self.aliases.get(name, name)
        if name == 'link':
            return LINK
        if name not in self.regs:
            assert len(self.regs) < 127, 'register file full'
            self.regs[name] = len(self.regs)
        return self.regs[name]

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def lbl(self):
        self._n += 1
        return f'__m{self._n}'

    def data(self, v):
        self.code.append(['DATA', v & M24])
        return len(self.code) - 1

    def emit(self, op, dst=None, a=None, b=None, imm=0):
        R = lambda x: 0 if x is None else (self.r(x) if isinstance(x, str) else x)
        assert op in OPS, f'{op} is not an instruction'
        if op in ('SHR', 'SHL'):
            assert 0 <= imm <= 23, imm
        if op == 'MUL':
            assert 0 <= imm <= 47, imm
        if op in ('ADDI', 'ANDI') and not isinstance(imm, str):
            assert -4096 <= imm < 4096, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -(1 << 19) <= imm < (1 << 19), imm
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
        for ins in self.resolve():
            if ins[0] == 'DATA':
                out.append(ins[1])
                continue
            op, dst, a, b, imm = ins
            name = inv[op]
            w = op << 27
            if name in ('JLT', 'JGE'):
                assert 0 <= imm < 8192
                w |= ((imm >> 6) & 0x7F) << 20 | a << 13 | b << 6 | (imm & 0x3F)
            elif name in ('JMP', 'JNZ', 'JZ', 'CALL'):
                assert 0 <= imm < 8192
                w |= a << 13 | imm
            elif name in ('ADDI', 'ANDI'):
                w |= dst << 20 | a << 13 | (imm & 0x1FFF)
            elif name == 'LDI':
                w |= dst << 20 | (imm & 0xFFFFF)
            else:
                assert 0 <= imm < 64, (name, imm)
                w |= dst << 20 | a << 13 | b << 6 | imm
            out.append(w)
        return out


def decode(w):
    """word -> (op, dst, a, b, imm) exactly as the RTL decodes it"""
    op = w >> 27
    dst, a, b, i6 = (w >> 20) & 127, (w >> 13) & 127, (w >> 6) & 127, w & 63
    inv = {v: k for k, v in OPS.items()}
    name = inv.get(op, 'NOP')               # data words decode as harmless ops
    if name in ('JLT', 'JGE'):
        imm = (dst << 6) | i6
    elif name in ('JMP', 'JNZ', 'JZ', 'CALL'):
        imm = w & 0x1FFF
    elif name in ('ADDI', 'ANDI'):
        imm = w & 0x1FFF
        imm = imm - (1 << 13) if imm & 0x1000 else imm
    elif name == 'LDI':
        imm = w & 0xFFFFF
        imm = imm - (1 << 20) if imm & 0x80000 else imm
    else:
        imm = i6
    return name, dst, a, b, imm
