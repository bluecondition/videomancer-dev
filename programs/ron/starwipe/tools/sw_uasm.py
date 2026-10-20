#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""STARWIPE microcode assembler (damier/cubist lineage, 32-bit datapath, v2).

The engine: an adder ALU, a serial shifter, a radix-4 Booth serial multiplier
(32 x 32 -> 64, result shifted by i6), a serial divider ((A << 16) / B,
unsigned), a serial square root (floor(sqrt(A << 16)) = sqrt(A) in Q.8), a
quarter-sine ROM, a 128 x 32 block-RAM register file, a 512 x 32 block-RAM
DATA memory (ray records, read by the line builder) and the program in ROM.

Instruction: 32 bits
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6
Immediates overlap the fields an op does not use:
  LDI            imm20 = [19:0] (sign-extended)
  JMP CALL       target = [12:0]
  JNZ JZ         A = [19:13], target = [12:0]
  JLT JGE        A, B; target = [26:20] & [5:0]
  SHR SHL        count = i6 (0..31)
  MUL            dst = (A * B) >> i6, low 32 bits (signed; stalls)
  DIV            dst = (A << 16) / B, unsigned, A < 2^16 B (stalls)
  SQRT           dst = floor(sqrt(A << 16)), A unsigned (stalls)
  ADDI ANDI      dst = A + / & imm13 = [12:0] (sign-extended)
  LDD STD        dst = DATA[A + i6] / DATA[A + i6] = B
  LDM            dst = program word A (data tables in the ROM)
  CTL            dst = control input i6
  PRT            port i6 = A (aux B)
  CALL           r127 = pc + 1, pc = target   (RET = JR link)
  MIN MAX        dst = min / max(A, B), signed
  DATA (pseudo)  a raw 32-bit word in the program ROM

Registers 0..94 are global, 95 is the CALL link, and 96..127 are a WINDOW
into one of four 32-register banks selected by the WIN port (physical
128 + 32 bank + r - 96).  Each program phase names its own bank.
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5, 'AND': 6,
    'LDI': 7, 'MUL': 8, 'ADDI': 9, 'DIV': 10, 'ANDI': 11, 'SIN': 12, 'CTL': 13,
    'PRT': 14, 'JMP': 15, 'JNZ': 16, 'JLT': 17, 'JGE': 18, 'JR': 19, 'END': 20,
    'JZ': 21, 'SHLV': 22, 'SHRV': 23, 'MIN': 24, 'MAX': 25, 'LDM': 26, 'CALL': 27,
    'LDD': 28, 'STD': 29, 'SQRT': 30,
}
LINK = 95
WIN0 = 96

PORTS = {
    # pixel path, per frame
    'DK': 0, 'BCY': 1, 'BCUV': 2, 'RCY': 3, 'RCUV': 4, 'TK': 5, 'MODE': 6,
    'QA': 7, 'QB': 8,
    # line builder: list descriptors (record index << 8 | count) and the
    # band switch line per accumulator
    'LA_UP': 9, 'LA_DN': 10, 'LA_AY': 11, 'LB_UP': 12, 'LB_DN': 13, 'LB_AY': 14,
    'WIN': 31,
}
# ports 0..14 are WORDS of the DATA memory at PBASE + 16 * set (+ index); the
# hardware loads them at the anchor.  WIN is the only real port.
PBASE = 896

CTLS = {
    'k1': 0, 'k2': 1, 'k3': 2, 'k4': 3, 'k5': 4, 'k6': 5, 'sw': 6, 'p12': 7,
    'tid': 8, 'W': 9, 'H': 10, 'fld': 11, 'set': 12,
}

M32 = 0xFFFFFFFF


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []
        self.regs = {}
        self.aliases = {}
        self.wbank = [{} for _ in range(4)]
        self.bank = 0
        self._n = 0

    def win(self, bank):
        """switch the window bank (emits the port write)"""
        self.bank = bank
        self.emit('LDI', 'wtmp', imm=bank)
        self.emit('PRT', a='wtmp', imm='WIN')

    def inbank(self, bank):
        """assemble the following code for callers in this bank (no emit)"""
        self.bank = bank

    def wreg(self, bank, *names):
        """declare window names for a bank"""
        for n in names:
            if n not in self.wbank[bank]:
                assert len(self.wbank[bank]) < 32, f'bank {bank} full'
                self.wbank[bank][n] = WIN0 + len(self.wbank[bank])

    def alias(self, **kw):
        self.aliases.update(kw)

    def r(self, name):
        name = self.aliases.get(name, name)
        if name == 'link':
            return LINK
        if name in self.wbank[self.bank]:
            return self.wbank[self.bank][name]
        if name not in self.regs:
            assert len(self.regs) < 95, f'global register file full at {name}'
            self.regs[name] = len(self.regs)
        return self.regs[name]

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def lbl(self):
        self._n += 1
        return f'__m{self._n}'

    def data(self, v):
        self.code.append(['DATA', v & M32])
        return len(self.code) - 1

    def emit(self, op, dst=None, a=None, b=None, imm=0):
        R = lambda x: 0 if x is None else (self.r(x) if isinstance(x, str) else x)
        assert op in OPS, f'{op} is not an instruction'
        if op in ('SHR', 'SHL'):
            assert 0 <= imm <= 31, imm
        if op == 'MUL':
            assert 0 <= imm <= 63, imm
        if op in ('ADDI', 'ANDI') and not isinstance(imm, str):
            assert -4096 <= imm < 4096, imm
        if op in ('LDD', 'STD'):
            assert 0 <= imm < 64, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -(1 << 19) <= imm < (1 << 19), imm
        if op == 'PRT' and isinstance(imm, str):
            imm = PORTS[imm]
        if op == 'PRT' and imm != PORTS['WIN']:
            op, dst, b, a = 'STD', None, a, 'pbase'
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
    name = inv.get(op, 'NOP')
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
