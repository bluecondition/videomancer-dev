#!/usr/bin/env python3
"""Emulator for the CUBIST microcode engine.

The GHDL render loop is ~20 minutes.  This runs the same instruction
semantics in about a millisecond, so microcode bugs surface immediately
instead of after a render.  Semantics must track ueng.vhd exactly.
"""
import sys
sys.path.insert(0, '.')
from uasm import OPS

INV = {v: k for k, v in OPS.items()}
M32 = 0xFFFFFFFF


def s16(x):
    x &= M32
    return x - 0x100000000 if x & 0x80000000 else x


class Emu:
    def __init__(self, code, sin_rom, ctl=None):
        self.code = code
        self.sin = sin_rom
        self.ctl = ctl or {}
        self.rf = [0] * 128
        self.slots = {}
        self.gram = {}
        self.pc = 0
        self.mul = 0
        self.div = 0
        self.steps = 0

    def run(self, limit=200000):
        while self.steps < limit:
            self.steps += 1
            op, dst, sa, sb, imm = self.code[self.pc]
            name = INV[op]
            A, B = s16(self.rf[sa]), s16(self.rf[sb])
            self.pc += 1
            r = None
            if name == 'MOV': r = A
            elif name == 'ADD': r = A + B
            elif name == 'SUB': r = A - B
            elif name == 'SHR': r = A >> (imm & 15)
            elif name == 'SHL': r = A << (imm & 15)
            elif name == 'NEG': r = -A
            elif name == 'ABS': r = abs(A)
            elif name == 'MIN': r = min(A, B)
            elif name == 'MAX': r = max(A, B)
            elif name == 'AND': r = A & imm
            elif name == 'BIT': r = (A >> (imm & 15)) & 1
            elif name == 'CLP': r = 0 if A < 0 else (B if A > B else A)
            elif name == 'LDI': r = imm - 0x4000 if imm & 0x2000 else imm
            elif name == 'SIN': r = self.sin[A & 0xFF]
            elif name == 'MUL': self.mul = A * B
            elif name == 'MRD': r = self.mul >> (imm & 31)
            elif name == 'DIV': self.div = 0 if B == 0 else int((A << (imm & 31)) / B)
            elif name == 'DRD': r = self.div
            elif name == 'SLW': self.slots[(imm, B & 3)] = A
            elif name == 'GWR': self.gram[imm + A] = B
            elif name == 'CTL': r = self.ctl.get(imm, 0)
            elif name == 'LDX': r = s16(self.rf[(A + imm) & 127])
            elif name == 'STX': self.rf[(A + imm) & 127] = B & M32
            elif name == 'JMP': self.pc = imm
            elif name == 'JNZ':
                if A != 0: self.pc = imm
            elif name == 'JLT':
                if A < B: self.pc = imm
            elif name == 'JGE':
                if A >= B: self.pc = imm
            elif name == 'END': return True
            elif name == 'NOP': pass
            if r is not None:
                self.rf[dst] = r & M32
        raise RuntimeError(f'no END after {limit} steps (pc={self.pc})')
