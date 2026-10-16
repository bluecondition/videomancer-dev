#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
dasm.py -- the DODECA board's game CPU: ISA, macro assembler, emulator.

The CPU is a 16-bit accumulator machine (A, X, flags Z/N/C, an 8-deep return
stack).  One instruction = one 16-bit word: op[15:11] k[10:0].  Program ROM is
up to 4096 words (Harvard; LDC reads it as data): JMP/CALL targets are even
(pc = k*2), conditional branches and LOOP are pc-relative (pc+1+sext(k)),
LDC reads ROM[X+k].  Work RAM is 256 x 16.  SYS k with k[10:8] != 0 is a
memory-direct op on RAM[k[7:0]]: 1 INC, 2 DEC, 3 CLR, 4 LDXM, 5 STXM.

This file is the ISA reference: attract.vhd's CPU must match emulate() bit for
bit.  The emulator also models the board (tilemap, palette/row-scroll RAM,
double-buffered sprite table, stars) and renders the 256x240 canvas.

Pure Python (no numpy/PIL) so the SDK build hook can import it; rendering to
PNG is done by tools/preview.py with PIL.
"""

# --------------------------------------------------------------------- ISA
OPS = ['SYS', 'LDI', 'LDH', 'LD', 'ST', 'ADD', 'SUB', 'AND', 'OR', 'XOR',
       'CMP', 'ADDI', 'ANDI', 'ORI', 'CMPI', 'LDX', 'STX', 'LXI', 'ADX',
       'LDC', 'XORI', 'JMP', 'JZ', 'JNZ', 'JC', 'JNC', 'JN', 'JP', 'CALL',
       'IN', 'OUT', 'LOOP']
OPC = {n: i for i, n in enumerate(OPS)}
SYSOPS = ['NOP', 'RET', 'WAIT', 'TAX', 'TXA', 'INX', 'DEX', 'SHL', 'SHR',
          'ASR', 'NOT', 'NEG', 'SWAPB', 'SHL4', 'SHR4', 'CLA']
SYSC = {n: i for i, n in enumerate(SYSOPS)}

MEMOPS = {'LD', 'ADD', 'SUB', 'AND', 'OR', 'XOR', 'CMP', 'LDX'}   # read work RAM
ROM_WORDS = 4096
RAM_WORDS = 256

# I/O ports
P_TADDR, P_TDATA, P_CADDR, P_CDATA, P_SIDX, P_SPRY, P_SPRA, P_YSCR, \
    P_STAR, P_SWAP, P_SEED = range(11)
I_TDATA, I_K1, I_K2, I_K3, I_K4, I_K5, I_K6, I_SW, I_P12, I_RAND, \
    I_FRAME = range(11)


def sext11(k):
    return k - 2048 if k & 1024 else k


def lfsr_step(s):
    """xorshift16 (7, 9, 8), full period 65535.  Same in the VHDL.  (A plain
    Galois LFSR's low bits just shift along, so successive reads were
    correlated and the maze walk could never reach some cells.)"""
    s ^= (s << 7) & 0xFFFF
    s ^= s >> 9
    s ^= (s << 8) & 0xFFFF
    return s


# --------------------------------------------------------------- assembler
class AsmError(Exception):
    pass


class Asm:
    """Macro assembler.  Game code is Python that calls these methods.

    Operands may be ints, variable names (work RAM), labels (code), or
    ('lo', name)/('hi', name) style tuples are not supported -- keep simple.
    """

    def __init__(self):
        self.code = []          # list of (op, k or symbol, src-note)
        self.labels = {}
        self.vars = {}
        self.nvar = 0
        self.consts = {}
        self._uid = 0
        self.dtables = []
        self.addr = {}

    # ---- symbols
    def var(self, name, n=1):
        if name in self.vars:
            return self.vars[name]
        if self.nvar + n > RAM_WORDS:
            raise AsmError('work RAM full allocating %s' % name)
        self.vars[name] = self.nvar
        self.nvar += n
        return self.vars[name]

    def uid(self, stem='L'):
        self._uid += 1
        return '_%s%d' % (stem, self._uid)

    def label(self, name):
        if name in self.labels:
            raise AsmError('duplicate label ' + name)
        self.labels[name] = len(self.code)

    def here(self):
        return len(self.code)

    # ---- emit
    def emit(self, op, k=0):
        self.code.append((op, k))

    def data(self, *words):
        for w in words:
            self.code.append(('.word', w))

    def table(self, label, *words):
        """A data table placed in the data section at the bottom of ROM
        (so LDC's 11-bit offset always reaches it)."""
        self.dtables.append((label, list(words)))

    # memory-direct ops (SYS group k[10:8], RAM address k[7:0])
    def _memop(self, g, var):
        self.code.append(('SYS', ('memop', g, var)))

    def incm(self, var):
        self._memop(1, var)        # M += 1; A = M; flags

    def decm(self, var):
        self._memop(2, var)        # M -= 1; A = M; flags

    def clr(self, var):
        self._memop(3, var)        # M = 0

    def ldxm(self, var):
        self._memop(4, var)        # X = M

    def stxm(self, var):
        self._memop(5, var)        # M = X

    def __getattr__(self, name):
        u = name.upper()
        if u in OPC and u != 'SYS':
            return lambda k=0: self.emit(u, k)
        if u in SYSC:
            return lambda: self.emit('SYS', SYSC[u])
        raise AttributeError(name)

    # ---- resolve
    def _val(self, k, op):
        if isinstance(k, int):
            return k
        if isinstance(k, str):
            if op in ('JMP', 'JZ', 'JNZ', 'JC', 'JNC', 'JN', 'JP', 'CALL',
                      'LOOP', 'LDC'):
                if k in self.addr:
                    return self.addr[k]
                if k in self.consts:
                    return self.consts[k]
                raise AsmError('undefined label %s' % k)
            if k in self.vars:
                return self.vars[k]
            if k in self.addr:
                return self.addr[k]
            if k in self.consts:
                return self.consts[k]
            raise AsmError('undefined symbol %s (op %s)' % (k, op))
        if isinstance(k, tuple):
            if k[0] == 'memop':
                ad = self._val(k[2], 'LD')
                if not 0 <= ad <= 255:
                    raise AsmError('memop address %d' % ad)
                return (k[1] << 8) | ad
            if k[0] == 'lo':            # low byte of a symbol's value
                return self._val(k[1], 'LDC') & 255
            if k[0] == 'hi':            # high byte
                return (self._val(k[1], 'LDC') >> 8) & 255
            return self._val(k[0], op) + k[1]       # (symbol, offset)
        raise AsmError('bad operand %r' % (k,))

    def assemble(self):
        # layout: code[0] (the reset jump), the data section, then the rest
        # of the code.  Labels used by JMP/CALL are aligned to even words.
        items = [self.code[0]]
        lab_at = {}
        nd = sum(len(w) for _, w in self.dtables)
        for name, idx in self.labels.items():
            lab_at.setdefault(idx if idx == 0 else idx + nd, []).append(name)
        flat = []
        for label, words in self.dtables:
            lab_at.setdefault(len(items) + len(flat), []).append(label)
            flat.extend(('.word', w) for w in words)
        items += flat + self.code[1:]
        targets = {k for (op, k) in self.code if op in ('JMP', 'CALL') and isinstance(k, str)}
        out = []
        self.addr = {}
        for i, it in enumerate(items + [None]):
            names = lab_at.get(i, [])
            if any(n in targets for n in names) and len(out) % 2:
                out.append(('SYS', 0))
            for n in names:
                self.addr[n] = len(out)
            if it is not None:
                out.append(it)
        self.labels_final = dict(self.addr)
        words = []
        for i, (op, k) in enumerate(out):
            if op == '.word':
                v = self._val(k, '.word') if not isinstance(k, int) else k
                words.append(v & 0xFFFF)
                continue
            v = self._val(k, op)
            if op in ('JMP', 'CALL'):
                if v & 1 or not 0 <= v < 4096:
                    raise AsmError('%s target %d not even/in range at %d' % (op, v, i))
                v >>= 1
            elif op in ('JZ', 'JNZ', 'JC', 'JNC', 'JN', 'JP', 'LOOP'):
                v = v - (i + 1)
                if not -1024 <= v <= 1023:
                    raise AsmError('branch out of range at %d (%s %r)' % (i, op, k))
                v &= 2047
            elif op in ('LDI', 'ADDI', 'CMPI', 'LXI', 'ADX'):
                if not -1024 <= v <= 1023:
                    raise AsmError('immediate %d out of range at %d (%s)' % (v, i, op))
                v &= 2047
            elif op in ('ANDI', 'ORI', 'XORI'):
                if not 0 <= v <= 2047:
                    raise AsmError('unsigned imm %d out of range at %d' % (v, i))
            else:
                if not 0 <= v <= 2047:
                    raise AsmError('operand %d out of range at %d (%s %r)' % (v, i, op, k))
            words.append((OPC[op] << 11) | v)
        if len(words) > ROM_WORDS:
            raise AsmError('program is %d words, ROM is %d' % (len(words), ROM_WORDS))
        return words


# ---------------------------------------------------------------- emulator
class Board:
    """CPU + board model.  Cycle counts approximate the VHDL FSM."""

    def __init__(self, rom, patterns, knobs=None):
        self.rom = list(rom) + [0] * (ROM_WORDS - len(rom))
        self.pat = patterns            # list of 8 row words (16-bit) per pattern
        self.ram = [0] * RAM_WORDS
        self.a = 0
        self.x = 0
        self.z = self.n = self.c = 0
        self.pc = 0
        self.stack = [0] * 8
        self.sp = 0
        self.tmap = [0] * 1024
        self.ctl = [0] * 128
        self.obj = [[0] * 128, [0] * 128]   # two banks of 64 x 2 words
        self.front = 0
        self.swap_req = 0
        self.taddr = 0
        self.traddr = 0
        self.caddr = 0
        self.sidx = 0
        self.yscr = 0
        self.star = 0
        self.rng = 1
        self.frame = 0
        self.knobs = knobs or {}
        self.waiting = False
        self.cycles = 0
        self.tick_cycles = 0
        self.max_tick_cycles = 0

    # ---- ports
    def port_in(self, p):
        if p == I_TDATA:
            return self.tmap[self.traddr]
        if I_K1 <= p <= I_K6:
            return self.knobs.get('k%d' % (p - I_K1 + 1), 512) & 1023
        if p == I_SW:
            return self.knobs.get('sw', 0) & 31
        if p == I_P12:
            return self.knobs.get('p12', 512) & 1023
        if p == I_RAND:
            self.rng = lfsr_step(self.rng)
            return self.rng
        if p == I_FRAME:
            return self.frame & 0xFFFF
        return 0

    def port_out(self, p, v):
        v &= 0xFFFF
        if p == P_TADDR:
            self.taddr = v & 1023
            self.traddr = v & 1023
        elif p == P_TDATA:
            self.tmap[self.taddr] = v & 0xFFF
            self.taddr = (self.taddr + 1) & 1023
        elif p == P_CADDR:
            self.caddr = v & 127
        elif p == P_CDATA:
            self.ctl[self.caddr] = v & 0xFF
            self.caddr = (self.caddr + 1) & 127
        elif p == P_SIDX:
            self.sidx = v & 63
        elif p == P_SPRY:
            self.obj[1 - self.front][self.sidx * 2] = v
        elif p == P_SPRA:
            self.obj[1 - self.front][self.sidx * 2 + 1] = v
            self.sidx = (self.sidx + 1) & 63
        elif p == P_YSCR:
            self.yscr = v & 255
        elif p == P_STAR:
            self.star = v & 0x3FF
        elif p == P_SWAP:
            self.swap_req = 1
        elif p == P_SEED:
            self.rng = v if v else 1

    # ---- tick: what the hardware does at each logic tick
    def tick(self, frame_start=True):
        if frame_start and self.swap_req:
            self.front ^= 1
            self.swap_req = 0
        if self.waiting:
            back = self.obj[1 - self.front]
            for i in range(64):
                back[i * 2] = 0xF000      # y = 240: hidden
            self.swap_req = 0
            self.waiting = False
            self.frame = (self.frame + 1) & 0xFFFF
            self.max_tick_cycles = max(self.max_tick_cycles, self.tick_cycles)
            self.tick_cycles = 0

    def flags(self, r):
        r &= 0xFFFF
        self.z = int(r == 0)
        self.n = (r >> 15) & 1
        return r

    def step(self):
        """Execute one instruction.  Returns False if blocked in WAIT."""
        if self.waiting:
            return False
        w = self.rom[self.pc]
        op = OPS[w >> 11]
        k = w & 2047
        npc = (self.pc + 1) & 4095
        rel = (npc + sext11(k)) & 4095
        cyc = 4
        a = self.a
        if op in MEMOPS:
            cyc = 6
            ea = ((self.x + k) if op == 'LDX' else k) & (RAM_WORDS - 1)
            m = self.ram[ea]
        if op == 'SYS' and (k >> 8):
            # memory-direct group: k[10:8] = op, k[7:0] = RAM address
            g = k >> 8
            ma = k & 255
            m = self.ram[ma]
            cyc = 6
            if g == 1:                     # INC m
                r = (m + 1) & 0xFFFF
                self.ram[ma] = r
                self.a = self.flags(r)
            elif g == 2:                   # DEC m
                r = (m - 1) & 0xFFFF
                self.ram[ma] = r
                self.a = self.flags(r)
            elif g == 3:                   # CLR m
                self.ram[ma] = 0
                cyc = 4
            elif g == 4:                   # LDXM m
                self.x = m
            elif g == 5:                   # STXM m
                self.ram[ma] = self.x
                cyc = 4
        elif op == 'SYS':
            s = SYSOPS[k & 15]
            if s == 'RET':
                self.sp = (self.sp - 1) & 7
                npc = self.stack[self.sp]
            elif s == 'WAIT':
                self.waiting = True
            elif s == 'TAX':
                self.x = a
            elif s == 'TXA':
                self.a = self.flags(self.x)
            elif s == 'INX':
                self.x = (self.x + 1) & 0xFFFF
            elif s == 'DEX':
                self.x = (self.x - 1) & 0xFFFF
            elif s == 'SHL':
                self.c = (a >> 15) & 1
                self.a = self.flags(a << 1)
            elif s == 'SHR':
                self.c = a & 1
                self.a = self.flags(a >> 1)
            elif s == 'ASR':
                self.c = a & 1
                self.a = self.flags((a >> 1) | (a & 0x8000))
            elif s == 'NOT':
                self.a = self.flags(~a)
            elif s == 'NEG':
                self.a = self.flags(-a)
            elif s == 'SWAPB':
                self.a = self.flags(((a << 8) | (a >> 8)))
            elif s == 'SHL4':
                self.a = self.flags(a << 4)
            elif s == 'SHR4':
                self.a = self.flags(a >> 4)
            elif s == 'CLA':
                self.a = self.flags(0)
        elif op == 'LDI':
            self.a = self.flags(sext11(k))
        elif op == 'LDH':
            self.a = self.flags(((k & 255) << 8) | (a & 255))
        elif op in ('LD', 'LDX'):
            self.a = self.flags(m)
        elif op == 'ST':
            self.ram[k & (RAM_WORDS - 1)] = a
        elif op == 'STX':
            self.ram[(self.x + k) & (RAM_WORDS - 1)] = a
        elif op == 'ADD':
            r = a + m
            self.c = (r >> 16) & 1
            self.a = self.flags(r)
        elif op == 'SUB':
            self.c = int(a < m)
            self.a = self.flags(a - m)
        elif op == 'AND':
            self.a = self.flags(a & m)
        elif op == 'OR':
            self.a = self.flags(a | m)
        elif op == 'XOR':
            self.a = self.flags(a ^ m)
        elif op == 'CMP':
            self.c = int(a < m)
            self.flags(a - m)
        elif op == 'ADDI':
            r = a + (sext11(k) & 0xFFFF)
            self.c = (r >> 16) & 1
            self.a = self.flags(r)
        elif op == 'ANDI':
            self.a = self.flags(a & k)
        elif op == 'ORI':
            self.a = self.flags(a | k)
        elif op == 'XORI':
            self.a = self.flags(a ^ k)
        elif op == 'CMPI':
            b = sext11(k) & 0xFFFF
            self.c = int(a < b)
            self.flags(a - b)
        elif op == 'LXI':
            self.x = sext11(k) & 0xFFFF
        elif op == 'ADX':
            self.x = (self.x + sext11(k)) & 0xFFFF
        elif op == 'LDC':
            self.a = self.flags(self.rom[(self.x + k) & 4095])
            cyc = 6
        elif op == 'JMP':
            npc = k << 1
        elif op == 'JZ':
            npc = rel if self.z else npc
        elif op == 'JNZ':
            npc = rel if not self.z else npc
        elif op == 'JC':
            npc = rel if self.c else npc
        elif op == 'JNC':
            npc = rel if not self.c else npc
        elif op == 'JN':
            npc = rel if self.n else npc
        elif op == 'JP':
            npc = rel if not self.n else npc
        elif op == 'CALL':
            self.stack[self.sp] = npc
            self.sp = (self.sp + 1) & 7
            npc = k << 1
        elif op == 'IN':
            self.a = self.flags(self.port_in(k & 15))
        elif op == 'OUT':
            self.port_out(k & 15, a)
        elif op == 'LOOP':
            self.x = (self.x - 1) & 0xFFFF
            if self.x != 0:
                npc = rel
        self.pc = npc
        self.cycles += cyc
        self.tick_cycles += cyc
        return True

    def run_until_wait(self, limit=3_000_000):
        n = 0
        while not self.waiting:
            self.step()
            n += 1
            if n > limit:
                raise RuntimeError('no WAIT within %d instructions (pc=%d)' % (limit, self.pc))
        return n

    # ---- render the canvas (what the VHDL draws) -> list of 240 rows of
    # 3-3-2 bytes, or None for transparent-never (always full)
    def star_at(self, x, y):
        if not (self.star >> 8) & 1:
            return 0
        key = ((((y + (self.star & 255)) & 255) << 8) | x) & 0xFFFF
        s1 = key ^ (key >> 7)
        s2 = (s1 + (s1 << 5)) & 0xFFFF
        s3 = s2 ^ (s2 >> 9)
        s4 = (s3 + (s3 << 3)) & 0xFFFF
        if (s4 >> 9) != 0:
            return 0
        col = s4 & 3
        if col == 0:
            return 0
        if (self.star >> 9) & 1 and ((s4 >> 2) & 1) and ((self.frame >> 4) & 1):
            return 0
        return col

    def render(self):
        img = []
        bank = self.obj[self.front]
        for v in range(240):
            # sprite line buffer for line v
            lb = [0] * 256
            hits = 0
            for i in range(64):
                w0 = bank[i * 2]
                w1 = bank[i * 2 + 1]
                y = (w0 >> 8) & 255
                size16 = (w1 >> 14) & 1
                h = 16 if size16 else 8
                dy = (v - y) & 255
                if dy >= h:
                    continue
                hits += 1
                if hits > 16:
                    break
                x9 = (w0 & 255) | (((w1 >> 15) & 1) << 8)
                vf = (w1 >> 13) & 1
                hf = (w1 >> 12) & 1
                pal = ((w1 >> 10) & 3) | (((w1 >> 9) & 1) << 2)
                p = w1 & 511
                rv = (h - 1 - dy) if vf else dy
                w = 16 if size16 else 8
                for c in range(w):
                    cc = (w - 1 - c) if hf else c
                    if size16:
                        pn = p + ((rv >> 3) << 1) + (cc >> 3)
                    else:
                        pn = p
                    word = self.pat[pn][rv & 7] if pn < len(self.pat) else 0
                    px = (word >> (14 - 2 * (cc & 7))) & 3
                    col = (x9 + c) & 511
                    if px and col < 256 and lb[col] == 0:
                        lb[col] = (pal << 2) | px
            rs = self.ctl[64 + (v >> 3)]
            ty = ((v + self.yscr) & 255) if 16 <= v < 232 else v
            row = []
            for x in range(256):
                if lb[x]:
                    idx = 32 | lb[x]
                else:
                    tx = (x + rs) & 255
                    cell = self.tmap[((ty >> 3) << 5) | (tx >> 3)]
                    t = cell & 255
                    pal = (cell >> 8) & 3
                    hf = (cell >> 10) & 1
                    vf = (cell >> 11) & 1
                    r = (ty & 7) ^ (7 if vf else 0)
                    cc = (tx & 7) ^ (7 if hf else 0)
                    word = self.pat[t][r] if t < len(self.pat) else 0
                    px = (word >> (14 - 2 * cc)) & 3
                    if px:
                        idx = (pal << 2) | px
                    else:
                        s = self.star_at(x, v)
                        idx = s << 2
                row.append(self.ctl[idx])
            img.append(row)
        return img


def rgb332(c):
    r = (c >> 5) & 7
    g = (c >> 2) & 7
    b = c & 3
    return (r * 255 // 7, g * 255 // 7, b * 255 // 3)
