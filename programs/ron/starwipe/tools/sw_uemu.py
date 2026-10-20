#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Emulator for the STARWIPE microcode engine v2 (semantics track starwipe.vhd).

Runs the assembled words (decoded as the RTL decodes them), counts clocks with
the RTL's state timing, keeps the register file and the DATA memory (ray
records) across frames, and exposes the ports the pixel path and the line
builder read.  Unit contracts are asserted (divider / sqrt magnitudes).
"""
from sw_uasm import decode, PORTS, CTLS, PBASE
from sw_tables import SIN

M32 = 0xFFFFFFFF
PINV = {v: k for k, v in PORTS.items()}
CINV = {v: k for k, v in CTLS.items()}


def s32(x):
    x &= M32
    return x - (1 << 32) if x & 0x80000000 else x


def sin_unit(a):
    """quarter-wave ROM, 256 points, sampled on the 64-unit grid"""
    a &= 0xFFC0
    q, p = a >> 14, a & 16383
    pm = 16384 - p if q & 1 else p
    i = pm >> 6
    v = 32767 if i >= 256 else SIN[i]
    return -v if q & 2 else v


def isqrt(n):
    x = int(n ** 0.5)
    while x * x > n:
        x -= 1
    while (x + 1) * (x + 1) <= n:
        x += 1
    return x


class Engine:
    def __init__(self, words):
        self.words = list(words)
        self.prog = [decode(w) for w in words]
        self.rf = [0] * 256
        self.data = [0] * 1024
        self.ports = {}
        self.bank = 0

    def ra(self, r):
        return r if r < 96 else 128 + 32 * self.bank + (r - 96)

    def run_frame(self, ctl, limit=400_000):
        rf, data = self.rf, self.data
        P = self.ports
        pc = 0
        clk = 0
        steps = 0
        while True:
            steps += 1
            assert steps < limit, f'runaway microcode at pc={pc}'
            name, dst, sa, sb, imm = self.prog[pc]
            A, B = s32(rf[self.ra(sa)]), s32(rf[self.ra(sb)])
            dst = self.ra(dst)
            pc += 1
            clk += 4
            r = None
            if name == 'NOP':
                pass
            elif name == 'MOV':
                r = A
            elif name == 'ADD':
                r = A + B
            elif name == 'SUB':
                r = A - B
            elif name == 'AND':
                r = A & B
            elif name == 'MIN':
                r = min(A, B); clk += 1
            elif name == 'MAX':
                r = max(A, B); clk += 1
            elif name == 'LDM':
                r = self.words[A & 0x1FFF]; clk += 3
            elif name == 'SHR':
                r = A >> imm; clk += imm
            elif name == 'SHL':
                r = A << imm; clk += imm
            elif name == 'SHRV':
                n = B & 31
                r = A >> n; clk += n
            elif name == 'SHLV':
                n = B & 31
                r = A << n; clk += n
            elif name == 'LDI':
                r = imm
            elif name == 'MUL':
                r = (A * B) >> imm
                clk += 16 + 3 + (imm >> 1) + (imm & 1)   # result shift: 2 bits per clock
            elif name == 'DIV':
                a, b = A & M32, B & M32
                assert 0 < b and a < (b << 16), f'DIV contract a={a} b={b} pc={pc-1}'
                r = (a << 16) // b
                clk += 48 + 3
            elif name == 'SQRT':
                r = isqrt((A & M32) << 16)
                clk += 24 + 4
            elif name == 'ADDI':
                r = A + imm
            elif name == 'ANDI':
                r = A & imm
            elif name == 'LDD':
                r = data[(A + imm) & 1023]; clk += 2
            elif name == 'STD':
                ad = (A + imm) & 1023
                data[ad] = B & M32
                if PBASE <= ad < PBASE + 32 and (ad & 15) in PINV:
                    P[PINV[ad & 15]] = B & M32
            elif name == 'SIN':
                r = sin_unit(A); clk += 3
            elif name == 'CTL':
                r = ctl[CINV[imm]]
            elif name == 'PRT':
                P[PINV[imm]] = A & M32
                if imm == 31:
                    self.bank = A & 3
            elif name == 'JMP':
                pc = imm; clk += 1
            elif name == 'CALL':
                rf[95] = pc
                pc = imm; clk += 1
            elif name == 'JR':
                pc = A & 0x1FFF; clk += 1
            elif name == 'JNZ':
                if A != 0: pc = imm; clk += 1
            elif name == 'JZ':
                if A == 0: pc = imm; clk += 1
            elif name == 'JLT':
                if A < B: pc = imm; clk += 1
            elif name == 'JGE':
                if A >= B: pc = imm; clk += 1
            elif name == 'END':
                return clk
            else:
                raise RuntimeError(name)
            if r is not None:
                rf[dst] = r & M32
