#!/usr/bin/env python3
"""Emulator for the CUBIST microcode engine (semantics track cubist.vhd).

The GHDL render loop takes minutes; this runs a frame of microcode in
milliseconds, so microcode bugs surface immediately.
"""
import sys
sys.path.insert(0, '.')
from uasm import OPS

INV = {v: k for k, v in OPS.items()}
M32 = 0xFFFFFFFF


def s32(x):
    x &= M32
    return x - 0x100000000 if x & 0x80000000 else x


class Emu:
    def __init__(self, code, sin_rom, ctl=None, gam_rom=None, table=None):
        self.code = code
        self.sin = sin_rom
        self.gam = gam_rom or [0] * 256
        self.ctl = ctl or {}
        self.table = list(table) if table is not None else [0] * 256
        self.rf = [0] * 128
        self.slots = {}          # (port, slot) -> last value
        self.slw = []            # every SLW in order: (port, slot, value)
        self.pc = 0
        self.mul = 0
        self.div = 0
        self.steps = 0
        self.cycles = 0
        self._md = self._dd = 0
        self.prof = None         # optional: {pc: cycles}

    def run(self, limit=400000):
        self.pc = 0
        while self.steps < limit:
            self.steps += 1
            pc0, cyc0 = self.pc, self.cycles
            op, dst, sa, sb, imm = self.code[self.pc]
            name = INV[op]
            A, B = s32(self.rf[sa]), s32(self.rf[sb])
            self.pc += 1
            self.cycles += 3                       # DECODE, LATCH, EXEC
            r = None
            if name == 'MOV': r = A
            elif name == 'ADD': r = A + B
            elif name == 'SUB': r = A - B
            elif name == 'SHR': r = A >> imm; self.cycles += imm
            elif name == 'SHL': r = A << imm; self.cycles += imm
            elif name == 'AND': r = A & imm
            elif name == 'LDI': r = imm
            elif name == 'SIN': r = self.sin[A & 0xFF]; self.cycles += 2
            elif name == 'GAM': r = self.gam[A & 0xFF]; self.cycles += 2
            elif name == 'SRD': r = self.table[(A + imm) & 0xFF]; self.cycles += 2
            elif name == 'SWR': self.table[A & 0xFF] = B & 0xFFFF
            elif name == 'MUL':
                assert -(1 << 19) <= A < (1 << 19), f'MUL A {A} at {self.pc-1}'
                assert -(1 << 15) <= B < (1 << 15), f'MUL B {B} at {self.pc-1}'
                self.mul = A * B
                self._md = self.cycles + 8
            elif name == 'MRD':
                r = s32(self.mul) >> imm
                self.cycles = max(self.cycles, self._md)
            elif name == 'DIV':
                assert A >= 0 and B >= 0, f'DIV of a negative operand at pc {self.pc - 1}'
                n = (A << imm) & ((1 << 34) - 1)
                d = (B & ((1 << 21) - 1)) or 1
                self.div = min(n // d, (1 << 20) - 1)
                self._dd = self.cycles + 36
            elif name == 'DRD':
                r = self.div
                self.cycles = max(self.cycles, self._dd)
            elif name == 'SLW':
                self.slots[(imm, B & 7)] = A
                self.slw.append((imm, B & 7, A, list(self.rf)))
            elif name == 'CTL': r = self.ctl.get(imm, 0)
            elif name == 'LDX': r = s32(self.rf[(A + imm) & 127]); self.cycles += 2
            elif name == 'STX': self.rf[(A + imm) & 127] = B & M32
            elif name == 'JMP': self.pc = imm; self.cycles += 1       # refetch
            elif name == 'JR': self.pc = A & 1023; self.cycles += 1
            elif name == 'JNZ':
                if A != 0: self.pc = imm; self.cycles += 1
            elif name == 'JLT':
                if A < B: self.pc = imm; self.cycles += 1
            elif name == 'JGE':
                if A >= B: self.pc = imm; self.cycles += 1
            elif name == 'END': return True
            elif name == 'NOP': pass
            else:
                raise RuntimeError(f'op {name}')
            if r is not None:
                self.rf[dst] = r & M32
            if self.prof is not None:
                self.prof[pc0] = self.prof.get(pc0, 0) + self.cycles - cyc0
        raise RuntimeError(f'no END after {limit} steps (pc={self.pc})')
