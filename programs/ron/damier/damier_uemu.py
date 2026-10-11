#!/usr/bin/env python3
"""Emulator for the DAMIER microcode engine (semantics track damier.vhd).

16-bit register file, 32-bit multiply product, quarter-sine unit with the
same integer interpolation as the RTL.  Runs a field of microcode in well
under a second, so microcode bugs surface without a hardware round trip.
"""
from damier_uasm import OPS, PORTS

INV = {v: k for k, v in OPS.items()}
PINV = {v: k for k, v in PORTS.items()}
M16 = 0xFFFF


def s16(x):
    x &= M16
    return x - 0x10000 if x & 0x8000 else x


def s32(x):
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


def sine_unit(eng, ang, cos):
    """RTL quarter-sine: fold, 512-entry ROM (Q15), no interpolation."""
    ang &= M16
    if cos:
        ang = (ang + 16384) & M16
    q = ang >> 14
    p = ang & 16383
    pm = (16384 - p) if (q & 1) else p
    v = 32767 if pm >= 16384 else eng[pm >> 5]
    if q & 2:
        v = -v
    return s16(v)


class Emu:
    def __init__(self, code, eng, ctl):
        self.code = code
        self.eng = eng
        self.ctl = ctl
        self.rf = [0] * 128
        self.ports = {}
        self.writes = []         # (port name, value, aux)
        self.tsel = 0
        self.whi = 0
        self.tabA = [0] * 128
        self.tabB = [0] * 128
        self.lens = [0] * 512
        self.cycles = 0
        self.steps = 0
        self.prod = 0
        self._md = 0
        self.fdone_cycle = None

    def run(self, limit=2_000_000):
        pc = 0
        while self.steps < limit:
            self.steps += 1
            op, dst, sa, sb, imm = self.code[pc]
            name = INV[op]
            A, B = s16(self.rf[sa]), s16(self.rf[sb])
            pc += 1
            self.cycles += 3
            r = None
            if name == 'MOV': r = A
            elif name == 'ADD': r = A + B
            elif name == 'SUB': r = A - B
            elif name == 'SHR': r = A >> imm; self.cycles += imm
            elif name == 'SHL': r = A << imm; self.cycles += imm
            elif name == 'AND': r = A & imm
            elif name == 'LDI': r = imm - 16384 if imm & 0x2000 else imm
            elif name == 'MUL':
                self.prod = A * B
                self._md = self.cycles + 11
            elif name == 'MRD':                 # two-stage shifter: +1 at least
                self.cycles = max(self.cycles + 1, self._md)
                r = s32(self.prod) >> imm
            elif name == 'SIN':
                r = sine_unit(self.eng, A, imm & 1)
                self.cycles += 3
            elif name == 'CTL': r = self.ctl[imm]
            elif name == 'PRT':
                pn = PINV[imm]
                self.writes.append((pn, A, B))
                self.ports[pn] = A
                if pn == 'WHI':
                    self.whi = A & M16
                elif pn == 'TSEL':
                    self.tsel = A & 1
                elif pn == 'WTAB':
                    (self.tabB if self.tsel else self.tabA)[B & 127] = A & M16
                elif pn == 'WLN':
                    self.lens[B & 511] = (self.whi << 16) | (A & M16)
                elif pn == 'FDONE':
                    self.fdone_cycle = self.cycles
            elif name == 'JMP': pc = imm; self.cycles += 1
            elif name == 'JR': pc = A & 511; self.cycles += 1
            elif name == 'JNZ':
                if A != 0: pc = imm; self.cycles += 1
            elif name == 'JLT':
                if A < B: pc = imm; self.cycles += 1
            elif name == 'JGE':
                if A >= B: pc = imm; self.cycles += 1
            elif name == 'END':
                return True
            elif name == 'NOP':
                pass
            else:
                raise RuntimeError(f'op {name}')
            if r is not None:
                self.rf[dst] = r & M16
        raise RuntimeError(f'no END after {limit} steps (pc={pc})')
