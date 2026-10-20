#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Emulator for the STARWIPE microcode engine (semantics track starwipe.vhd).

Runs the assembled words (decoded exactly as the RTL decodes them), counts
clocks with the RTL's state timing, and services the WALK port with the
pixel-path model so the coverage bounds come from the same arithmetic as the
hardware.  Asserts every unit contract (divider magnitudes, shift counts).
"""
import numpy as np
from sw_uasm import decode, PORTS, CTLS
from sw_tables import SIN
import sw_pixel as px

M24 = 0xFFFFFF
PINV = {v: k for k, v in PORTS.items()}
WALK_FLUSH = 48            # pipeline depth the walk waits out


def s24(x):
    x &= M24
    return x - (1 << 24) if x & 0x800000 else x


def sin_unit(a):
    """quarter-wave ROM, 256 points, no interpolation (the microcode interpolates)"""
    a &= 0xFFC0                       # sample on the 64-unit grid (the microcode interpolates)
    q, p = a >> 14, a & 16383
    pm = 16384 - p if q & 1 else p
    i = pm >> 6
    v = 32767 if i >= 256 else SIN[i]
    return -v if q & 2 else v


class Engine:
    def __init__(self, words):
        self.words = list(words)
        self.prog = [decode(w) for w in words]
        self.rf = [0] * 128               # persistent across frames (EBR)
        self.ports = {}

    def run_frame(self, ctl, limit=200_000):
        """ctl: CTL inputs by name -> ports dict; returns clocks used"""
        rf = self.rf
        prod = 0
        quo = 0
        pc = 0
        clk = 0
        busy_until = 0          # multiplier / divider completion clock
        wbusy_until = 0
        stats = None            # (min, max) of the last walk
        wpts = []               # walk points (u, v)
        acc = [0, 0]
        P = self.ports
        steps = 0
        while True:
            steps += 1
            assert steps < limit, f'runaway microcode at pc={pc}'
            name, dst, sa, sb, imm = self.prog[pc]
            A, B = s24(rf[sa]), s24(rf[sb])
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
                r = self.words[A & 0x1FFF] & M24; clk += 3
            elif name == 'SHR':
                r = A >> imm; clk += imm
            elif name == 'SHL':
                r = A << imm; clk += imm
            elif name == 'SHRV':
                n = B & 31; assert n <= 23, n
                r = A >> n; clk += n
            elif name == 'SHLV':
                n = B & 31; assert n <= 23, n
                r = A << n; clk += n
            elif name == 'LDI':
                r = imm
            elif name == 'MUL':
                r = (A * B) >> imm
                clk += 12 + 3
            elif name == 'DIV':
                a, b = rf[sa] & M24, rf[sb] & M24
                assert 0 < b and a < (b << 8), f'DIV contract a={a} b={b} pc={pc-1}'
                r = (a << 16) // b
                clk += 24 + 3
            elif name == 'ADDI':
                r = A + imm
            elif name == 'ANDI':
                r = A & imm
            elif name == 'CALL':
                rf[127] = pc
                pc = imm; clk += 1
            elif name == 'SIN':
                r = sin_unit(A); clk += 3
            elif name == 'CTL':
                key = {v: k for k, v in CTLS.items()}[imm]
                if key == 'WBUSY':
                    r = 1 if clk < wbusy_until else 0
                elif key in ('DMIN', 'DMAX'):
                    clk = max(clk, wbusy_until)   # holds until the walk has flushed
                    if stats is None:
                        stats = self._walk_stats(wpts)
                    r = stats[0] if key == 'DMIN' else stats[1]
                else:
                    r = ctl[key]
            elif name == 'PRT':
                pn = PINV[imm]
                P[pn] = A
                if pn == 'WALK':
                    v = A & M24
                    n = v & 0xFFF
                    if v & 0x800000:          # load the top seed, clear the stats
                        acc = [s24(P['SU']) << 8, s24(P['SV']) << 8]
                        wpts = []
                    for _ in range(n):
                        wpts.append((acc[0] >> 10, acc[1] >> 10))
                        acc[0] += s24(P['DUX']); acc[1] += s24(P['DVX'])
                    stats = None
                    self._wflags = dict(P)
                    clk += n + 2                  # the engine holds until the edge has stepped
                    wbusy_until = clk + WALK_FLUSH
            elif name == 'JMP':
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
                rf[dst] = r & M24

    def _walk_stats(self, pts):
        """the merge-stage min/max trackers over the walk's points, with the
        field flags as they stood at the LAST walk command"""
        Q = port_dict(self._wflags)
        u = np.array([p[0] for p in pts], dtype=np.int64)
        v = np.array([p[1] for p in pts], dtype=np.int64)
        D = px.field(u, v, Q)
        return int(D.min()), int(D.max())


def port_dict(P):
    """raw 24-bit port registers -> the pixel model's parameter dict"""
    g = lambda k, d=0: s24(P.get(k, d))
    fl = P.get('FLAGS', 0)
    bcuv, rcuv = P.get('BCUV', 0), P.get('RCUV', 0)
    return dict(
        DUX=g('DUX'), DVX=g('DVX'), DUY=g('DUY'), DVY=g('DVY'),
        SEED=[(g('SU'), g('SV'))],
        PSEL=fl & 15, REV=(fl >> 4) & 1,
        T=g('T'), DK=P.get('DK', 0) & 0x3FFFF,
        TK=P.get('TK', 0) & 511, BA=g('BA'), BB=g('BB'),
        SN=P.get('SN', 5) & 15, SS=P.get('SS', 0) & 0xFFFFF, SAL=P.get('SAL', 0) & 0x3FFFF,
        BC=(P.get('BCY', 64) & 1023, (bcuv >> 10) & 1023, bcuv & 1023),
        RC=(P.get('RCY', 64) & 1023, (rcuv >> 10) & 1023, rcuv & 1023),
    )
