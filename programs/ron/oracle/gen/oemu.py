#!/usr/bin/env python3
"""Emulator for the ORACLE microcode engine (semantics track oracle.vhd).

Runs a field's worth of microcode in milliseconds.  Models the ISA, not
the engine's pipeline: bit-exact here is necessary, not sufficient (see the
cubist notes -- a block-RAM register file reads a cycle late).
"""
from oasm import OPS, NXF

INV = {v: k for k, v in OPS.items()}
M32 = 0xFFFFFF            # the engine datapath is 24 bits


def s32(x):
    x &= M32
    return x - 0x1000000 if x & 0x800000 else x


class Emu:
    def __init__(self, code, rf_init, qsin, pves, pcir, ctl, wrap_ok=(), names=None):
        self.code = code
        self.qsin, self.pves, self.pcir = qsin, pves, pcir
        self.ctl = ctl                     # dict: index -> value or callable
        self.rf = [rf_init.get(i, 0) & M32 for i in range(256)]
        self.ports = {}
        self.xfers = []                    # list of NXF-value lists
        self.pc = 0
        self.mul = 0
        self.div = 0
        self.cycles = 0
        self._md = self._dd = 0
        self.wrap_ok = set(wrap_ok)
        self.names = names or {}
        self.overflows = {}

    def sin(self, a):
        a &= 1023
        i = a & 255
        if a & 256:
            i = 255 - i
        v = self.qsin[i]
        return -v if a & 512 else v

    def run(self, event, limit=200000):
        """Run until the next WAIT; event = 'field' or 'line' satisfies it.
        Returns the number of cycles spent."""
        c0 = self.cycles
        waited = False
        for _ in range(limit):
            op, dst, sa, sb, imm = self.code[self.pc]
            name = INV[op]
            A, B = s32(self.rf[sa]), s32(self.rf[sb])
            if name == 'WAIT':
                if waited:
                    return self.cycles - c0
                if event == 'field' and imm & 1:
                    self.pc = 0              # a pending field tick restarts
                    self.cycles += 3
                    continue
                assert event == 'field' or imm == 1, 'line event at a field wait'
                waited = True
                self.pc += 1
                self.cycles += 3
                continue
            self.pc += 1
            self.cycles += 3
            r = None
            if name == 'MOV': r = A
            elif name == 'ADD': r = A + B
            elif name == 'SUB': r = A - B
            elif name == 'SHR': r = A >> imm; self.cycles += imm
            elif name == 'SHL': r = A << imm; self.cycles += imm
            elif name == 'AND': r = A & imm
            elif name == 'MIN': r = A if A < B else B; self.cycles += 1
            elif name == 'MAX': r = B if A < B else A; self.cycles += 1
            elif name == 'LDI': r = imm
            elif name == 'SIN': r = self.sin(A); self.cycles += 4
            elif name == 'PRV': r = self.pves[A & 255]; self.cycles += 4
            elif name == 'PRC': r = self.pcir[A & 255]; self.cycles += 4
            elif name == 'MUL':
                assert 0 <= A < (1 << 16), f'MUL A {A} at {self.pc-1}'
                assert 0 <= B < (1 << 12), f'MUL B {B} at {self.pc-1}'
                self.mul = A * B
                self._md = self.cycles + 12
            elif name == 'MRD':
                # >>12 takes bits 27:12 of the full 28-bit product (RTL);
                # >>0 is its low 24 bits
                r = (self.mul >> 12) if imm else s32(self.mul)
                self.cycles = max(self.cycles, self._md)
            elif name == 'DIV':
                A = self.rf[sa] & M32                # numerator read unsigned
                assert 0 < B < (1 << 12), f'DIV B {B} at pc {self.pc - 1}'
                assert imm == 0
                self.div = min(A // B, (1 << 20) - 1)
                self._dd = self.cycles + 27
            elif name == 'DRD':
                r = self.div
                self.cycles = max(self.cycles, self._dd)
            elif name == 'OUT':
                self.ports[imm] = A
            elif name == 'CTL':
                v = self.ctl.get(imm, 0)
                r = v() if callable(v) else v
            elif name == 'XFER':
                self.xfers.append([s32(self.rf[(sa + i) & 255]) for i in range(NXF)])
                self.cycles += NXF + 3
            elif name == 'JMP': self.pc = imm; self.cycles += 1
            elif name == 'JR': self.pc = A & 2047; self.cycles += 1
            elif name == 'JNZ':
                if A != 0: self.pc = imm; self.cycles += 1
            elif name == 'JLT':
                if A < B: self.pc = imm; self.cycles += 1
            elif name == 'JGE':
                if A >= B: self.pc = imm; self.cycles += 1
            elif name == 'NOP': pass
            else:
                raise RuntimeError(f'op {name}')
            if r is not None:
                if not (-(1 << 23) <= r < (1 << 23)) and dst not in self.wrap_ok:
                    key = (self.pc - 1, self.names.get(dst, dst))
                    self.overflows[key] = r
                self.rf[dst] = r & M32
        raise RuntimeError(f'no WAIT after {limit} steps (pc={self.pc})')
