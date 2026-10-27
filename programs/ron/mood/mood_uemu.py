#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Emulator for the MOOD microcode engine (semantics track mood_core.vhd).

16-bit register file (persistent across samples, power-up = assembler init),
32-bit multiply product, ROM unit with the RTL's quarter-wave fold.  One
call to sample() runs the program from PC 0 to END, exactly as one sample
tick does on the chip, and returns the clocks it took.
"""
from mood_uasm import OPS, PORTS, CTLS

INV = {v: k for k, v in OPS.items()}
PINV = {v: k for k, v in PORTS.items()}
M16 = 0xFFFF


def s16(x):
    x &= M16
    return x - 0x10000 if x & 0x8000 else x


def s32(x):
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


def rom_unit(rom, ang, imm):
    """RTL ROM unit: raw table read, or quarter-sine fold (no interpolation)"""
    if imm & 2:
        bank = (imm >> 2) << 1 | (imm & 1)
        return s16(rom[512 + ((ang + 16 * bank) & 511)])
    ang &= M16
    if imm & 1:
        ang = (ang + 16384) & M16
    q = ang >> 14
    p = ang & 16383
    pm = (16384 - p) if (q & 1) else p
    v = 32767 if pm >= 16384 else rom[pm >> 5]
    if q & 2:
        v = -v
    return s16(v)


class Emu:
    def __init__(self, code, rom, rf_init):
        self.code = code
        self.rom = rom
        self.rf = list(rf_init)
        self.ctl = {k: 0 for k in CTLS}
        self.ports = {k: 0 for k in PORTS}
        self.writes = []
        self.prod = 0
        self.cycles = 0
        self.maxcyc = 0
        self.rnd_state = 0xACE1

    def rnd(self):
        # fabric LFSR stepping every clock: model as a fresh value per read
        s = self.rnd_state
        for _ in range(16):
            b = ((s >> 0) ^ (s >> 2) ^ (s >> 3) ^ (s >> 5)) & 1
            s = (s >> 1) | (b << 15)
        self.rnd_state = s
        return s16(s)

    def sample(self, limit=5000):
        pc = 0
        self.cycles = 0
        md = 0
        steps = 0
        self.writes = []
        while steps < limit:
            steps += 1
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
                md = self.cycles + 11
            elif name == 'MRD':
                self.cycles = max(self.cycles + 1, md)
                r = s32(self.prod) >> imm
            elif name == 'SIN':
                r = rom_unit(self.rom, A, imm)
                self.cycles += 3
            elif name == 'CTL':
                key = [k for k, v in CTLS.items() if v == imm][0]
                r = self.rnd() if key == 'rnd' else self.ctl[key]
            elif name == 'PRT':
                pn = PINV[imm]
                self.ports[pn] = s16(A)
                self.writes.append((pn, s16(A)))
            elif name == 'JMP': pc = imm; self.cycles += 1
            elif name == 'JR': pc = A & 1023; self.cycles += 1
            elif name == 'JNZ':
                if A != 0: pc = imm; self.cycles += 1
            elif name == 'JLT':
                if A < B: pc = imm; self.cycles += 1
            elif name == 'JGE':
                if A >= B: pc = imm; self.cycles += 1
            elif name == 'END':
                self.maxcyc = max(self.maxcyc, self.cycles)
                return self.cycles
            elif name == 'NOP':
                pass
            else:
                raise RuntimeError(f'op {name}')
            if r is not None:
                self.rf[dst] = r & M16
        raise RuntimeError(f'no END after {limit} steps (pc={pc})')
