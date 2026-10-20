#!/usr/bin/env python3
"""DICEFALL microcode assembler and emulator (NAT20's engine: CUBIST's, adapted).

One small datapath runs the per-field geometry in vertical blanking: a
single-adder ALU, a serial shifter, a radix-4 Booth serial multiplier, a
block-RAM register file (two copies for dual read), the program in block RAM.

Instruction: 32 bits
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6

Immediates overlap the fields an op does not use:
  LDI            imm14 = [13:0]  (sign-extended)
  AND            imm13 = [12:0]  (zero-extended)
  JMP JNZ        target = [9:0]
  JLT JGE        target = [23:20] & [5:0]   (a branch writes no register)
  LDX SRD        imm7 = [6:0]
  STX            imm6 = [5:0]   (srcB occupies [12:6])
  SLW            port = [4:0], index = srcB
  SHR SHL        count [3:0]
  CTL            input [4:0]
  MRD            [3] set = product >> 12

Changes from CUBIST: the table RAM is 512 words (SRD/SWR take 9 address
bits); CTL selects one of 32 inputs; one 256-word engine ROM (ROM op)
holds the quarter sine, 1/m and the face gamma curve; DIV and GAM are gone.
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'MUL': 10,   # start serial multiply A * B  (A 20-bit, B 16-bit signed)
    'MRD': 11,   # dst = low 32 bits of the product (>> 12)  (stalls until idle)
    'MULSH': 12, # dst = (A * B) >> (12 or 0): MUL and MRD in one (stalls)
    'DIV': 8,    # dst = A / B (A in 0 .. 2^26 - 1, B in 1 .. 2^22 - 1), 26 clocks
    'MIN': 6,    # dst = min(A, B)
    'SRDS': 23,  # dst = table RAM[(A + imm7) & 511], sign-extended
    'ROMS': 28,  # dst = engine ROM[A & 255], sign-extended
    'MAX': 7,    # dst = max(A, B)
    'ROM': 14,   # dst = engine ROM[A & 255]
    'LDI': 15,   # dst = imm14
    'SRD': 16,   # dst = table RAM[(A + imm7) & 511]
    'SLW': 17,   # output port imm <= A, index = B
    'JMP': 18,   # pc = target
    'JNZ': 19,   # pc = target if A /= 0
    'JLT': 20,   # pc = target if A < B
    'JGE': 21,   # pc = target if A >= B
    'SWR': 22,   # table RAM[A & 511] = B
    'END': 24,   # field setup complete
    'CTL': 25,   # dst = control input imm
    'JR':  26,   # pc = A  (subroutine return, jump tables)
    'AND': 27,   # dst = A and imm13
    'LDX': 30,   # dst = rf[A + imm7]
    'STX': 31,   # rf[A + imm6] = B
}

# SLW output ports (must match the VHDL decoder, p_slotw).  Records go to
# the ring entry at the VHDL's write pointer: edges at 4 e + word, header
# at 80 + word, numeral sprites at 8 (s + 11) + word.
PORT = {
    'e0': 0, 'e1': 1, 'e2': 2, 'e3': 3,    # edge record words, index = edge 0..19
    'h0': 4, 'h1': 5, 'h2': 6,             # die header: lines + tag, slot x0, counts
    'p0': 8, 'p1': 9, 'p2': 10, 'p3': 11,  # numeral sprite words, index = sprite + 11
    'p4': 12, 'p5': 13, 'p6': 14,
    'done': 17,                            # the entry is complete (value = field tag)
    'bgs': 20,                             # backdrop per-line luma step
}

# CTL inputs
CTLR = {
    'k1': 0, 'k2': 1, 'k3': 2, 'k4': 3, 'k5': 4, 'k6': 5,
    'W': 6, 'hf': 7, 'ilace': 8, 'field': 9,
    'sw': 11, 'slider': 12, 'xa': 13,
    'rhf': 14,          # 2^20 / frame height (from the timing ID)
    'wfree': 15,        # 1: the ring entry at the write pointer is free
    'fld': 16,          # the renderer's field counter (8 bits)
    'ncap': 17,         # dice the engine keeps up with in this mode
}

# engine ROM regions: sine 0..64; numeral corner weights per face shape
# 65..100 (triangle, square, kite: 4 for P, 4 for UPV, 4 for RT, Q12);
# numeral constants per die 101..115 (TC, SU, SV); inks 116..121
# (white Y U V, black Y U V); black-ink colour mask 122; face gamma 128..255
ROM_SIN, ROM_SHW, ROM_NUM, ROM_INK, ROM_INKM, ROM_GAM = 0, 65, 101, 116, 122, 128


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []
        self.regs = {}
        self.nreg = 0
        self._n = 0

    # ------------------------------------------------------------ registers
    def r(self, name):
        """register index by name (allocated on first use)"""
        if isinstance(name, int):
            return name
        if name not in self.regs:
            self.regs[name] = self.nreg
            self.nreg += 1
            assert self.nreg <= 128, 'out of registers'
        return self.regs[name]

    def new(self, name):
        """a register that must not exist yet (scoped temporaries)"""
        assert name not in self.regs, f'register name {name} already in use'
        return self.r(name)

    def block(self, name, n):
        """n consecutive registers; returns the first index"""
        base = self.nreg
        for i in range(n):
            self.regs[f'{name}[{i}]'] = base + i
        self.nreg += n
        assert self.nreg <= 128, 'out of registers'
        self.regs[name] = base       # alias for the base
        return base

    def scope(self):
        """with a.scope(): registers named inside are freed at the end"""
        asm = self

        class _S:
            def __enter__(self):
                self.n, self.names = asm.nreg, set(asm.regs)

            def __exit__(self, *exc):
                for k in set(asm.regs) - self.names:
                    del asm.regs[k]
                asm.peak = max(getattr(asm, 'peak', 0), asm.nreg)
                asm.nreg = self.n
        return _S()

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def lbl(self, stem='m'):
        self._n += 1
        return f'__{stem}{self._n}'

    # ------------------------------------------------------------ emit
    def emit(self, op, dst=0, a=0, b=0, imm=0):
        r = self.r
        dst, a, b = r(dst), r(a), r(b)
        z = r('zero')
        if op == 'NEG':
            self.emit('SUB', dst, z, a)
            return
        if op == 'ABS':
            l1, l2 = self.lbl(), self.lbl()
            self.emit('JLT', a=a, b=z, imm=l1)
            self.emit('MOV', dst, a)
            self.emit('JMP', imm=l2)
            self.label(l1)
            self.emit('NEG', dst, a)
            self.label(l2)
            return
        if op == 'CLP':                              # clamp(a, 0, b)
            self.emit('MAX', dst, a, z)
            self.emit('MIN', dst, dst, b)
            return
        if op in ('SHRL', 'SHLL'):                   # shifts past 15
            base = op[:3]
            src = a
            while True:
                k = min(imm, 15)
                self.emit(base, dst, src, imm=k)
                src, imm = dst, imm - k
                if imm == 0:
                    break
            return
        assert op in OPS, f'{op} is not implemented'
        if op in ('MRD', 'MULSH'): assert imm in (0, 12), f'{op} shift {imm}'
        if op in ('SHR', 'SHL'): assert 0 <= imm <= 15, imm
        if op == 'AND': assert 0 <= imm < 8192, imm
        if op in ('LDX', 'SRD', 'SRDS'): assert 0 <= imm < 128, imm
        if op == 'STX': assert 0 <= imm < 64, imm
        if op == 'CTL': assert 0 <= imm < 32, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -8192 <= imm < 8192, imm
        if isinstance(imm, str):
            self.fixups.append((len(self.code), imm))
            imm = 0
        self.code.append([OPS[op], dst, a, b, imm])
        return len(self.code) - 1

    def ldc(self, dst, v, scratch):
        """any 32-bit constant (scratch is clobbered)"""
        assert self.r(scratch) != self.r(dst)
        if -8192 <= v < 8192:
            self.emit('LDI', dst, imm=v)
            return
        hi, lo = v >> 12, v & 0xFFF
        self.ldc(dst, hi, scratch)
        self.emit('SHL', dst, dst, imm=12)
        if lo:
            self.emit('LDI', scratch, imm=lo)
            self.emit('ADD', dst, dst, scratch)

    def resolve(self):
        for idx, name in self.fixups:
            assert name in self.labels, f'undefined label {name}'
            self.code[idx][4] = self.labels[name]
        return self.code

    def words(self):
        inv = {v: k for k, v in OPS.items()}
        out = []
        for op, dst, a, b, imm in self.resolve():
            name = inv[op]
            w = op << 27
            if name in ('JLT', 'JGE'):
                assert 0 <= imm < 1024
                w |= ((imm >> 6) & 0xF) << 20 | a << 13 | b << 6 | (imm & 0x3F)
            elif name in ('JMP', 'JNZ'):
                assert 0 <= imm < 1024
                w |= a << 13 | imm
            elif name == 'LDI':
                w |= dst << 20 | (imm & 0x3FFF)
            elif name == 'AND':
                w |= dst << 20 | a << 13 | (imm & 0x1FFF)
            elif name in ('LDX', 'SRD', 'SRDS'):
                w |= dst << 20 | a << 13 | imm
            elif name == 'STX':
                w |= a << 13 | b << 6 | imm
            elif name == 'MRD':
                w |= dst << 20 | (8 if imm == 12 else 0)
            elif name == 'MULSH':
                w |= dst << 20 | a << 13 | b << 6 | (8 if imm == 12 else 0)
            else:
                assert 0 <= imm < 64, (name, imm)
                w |= dst << 20 | a << 13 | b << 6 | imm
            out.append(w)
        return out


def vhdl_ucode(words, depth):
    words = list(words) + [0] * (depth - len(words))
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:08X}"')
        if len(row) == 6 or i == depth - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_ucode is array (0 to {depth-1}) of std_logic_vector(31 downto 0);\n'
            f'    constant C_UCODE : t_ucode := (\n' + ',\n'.join(rows) + ' );')


def vhdl_rom16(name, words, typ):
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w & 0xFFFF:04X}"')
        if len(row) == 12 or i == len(words) - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    constant {name} : {typ} := (\n' + ',\n'.join(rows) + ' );')


# ====================================================================== emulator
M32 = 0xFFFFFFFF
# ops that read register B: the single-copy register file reads it a clock late
B_OPS = {'ADD', 'SUB', 'MIN', 'MAX', 'MULSH', 'MUL', 'JLT', 'JGE', 'STX', 'SLW', 'SWR', 'DIV'}
INV = {v: k for k, v in OPS.items()}


def s32(x):
    x &= M32
    return x - 0x100000000 if x & 0x80000000 else x


class Pause(Exception):
    """raised by a CTL callback to stop the emulator at that instruction"""


class Emu:
    """Runs the microcode.  Cycle counts follow the VHDL engine: 3 clocks an
    instruction (+1 when it reads register B), +1 for a taken jump, shifts
    1 clock a bit, ROM/SRD/LDX +2, MULSH ~9, DIV 27.  The engine never ends:
    run() returns when the CTL callback raises Pause (the instruction is
    re-executed on the next run()), or at END."""

    def __init__(self, code, rom, table, ctl):
        self.code, self.rom, self.ctl = code, rom, ctl
        self.table = list(table)
        self.rf = [0] * 128
        self.slw = []
        self.cycles = 0
        self.steps = 0
        self.pc = 0

    def run(self, rf=None, limit=2000000):
        if rf is not None:
            self.rf = list(rf)
        pc, mul, md = self.pc, 0, 0
        self.slw = []
        while self.steps < limit:
            self.steps += 1
            op, dst, sa, sb, imm = self.code[pc]
            name = INV[op]
            A, B = s32(self.rf[sa]), s32(self.rf[sb])
            pc += 1
            self.cycles += 3 + (name in B_OPS)       # B is read a clock later
            r = None
            if name == 'MOV': r = A
            elif name == 'ADD': r = A + B
            elif name == 'SUB': r = A - B
            elif name == 'SHR': r = A >> imm; self.cycles += imm
            elif name == 'SHL': r = A << imm; self.cycles += imm
            elif name == 'AND': r = A & imm
            elif name == 'LDI': r = imm
            elif name == 'ROM': r = self.rom[A & 255]; self.cycles += 2
            elif name == 'SRD': r = self.table[(A + imm) & 511]; self.cycles += 2
            elif name == 'SRDS':
                r = self.table[(A + imm) & 511]; self.cycles += 2
                r = r - 65536 if r & 0x8000 else r
            elif name == 'ROMS':
                r = self.rom[A & 255]; self.cycles += 2
                r = r - 65536 if r & 0x8000 else r
            elif name == 'SWR': self.table[A & 511] = B & 0xFFFF
            elif name == 'MUL':
                assert -(1 << 19) <= A < (1 << 19), f'MUL A {A} at {pc-1}'
                assert -(1 << 15) <= B < (1 << 15), f'MUL B {B} at {pc-1}'
                mul = A * B
                assert -(1 << 31) <= mul < (1 << 31), f'product {mul} at {pc-1}'
                md = self.cycles + 8
            elif name == 'MRD':
                r = s32(mul) >> imm
                self.cycles = max(self.cycles, md)
            elif name == 'MULSH':
                assert -(1 << 19) <= A < (1 << 19), f'MULSH A {A} at {pc-1}'
                assert -(1 << 15) <= B < (1 << 15), f'MULSH B {B} at {pc-1}'
                mul = A * B
                assert -(1 << 31) <= mul < (1 << 31), f'product {mul} at {pc-1}'
                r = s32(mul) >> imm
                self.cycles += 9
            elif name == 'DIV':
                assert 0 <= A < (1 << 26) and 0 < B < (1 << 22), f'DIV {A} / {B} at {pc-1}'
                r = A // B
                self.cycles += 27
            elif name == 'MIN': r = min(A, B)
            elif name == 'MAX': r = max(A, B)
            elif name == 'SLW':
                self.slw.append((imm, B & 31, s32(A)))
            elif name == 'CTL':
                try:
                    r = self.ctl(imm) if callable(self.ctl) else self.ctl.get(imm, 0)
                except Pause:
                    self.pc = pc - 1
                    return False
            elif name == 'LDX': r = s32(self.rf[(A + imm) & 127]); self.cycles += 2
            elif name == 'STX': self.rf[(A + imm) & 127] = B & M32
            elif name == 'JMP': pc = imm; self.cycles += 1
            elif name == 'JR': pc = A & 1023; self.cycles += 1
            elif name == 'JNZ':
                if A != 0: pc = imm; self.cycles += 1
            elif name == 'JLT':
                if A < B: pc = imm; self.cycles += 1
            elif name == 'JGE':
                if A >= B: pc = imm; self.cycles += 1
            elif name == 'END': return True
            elif name == 'NOP': pass
            else:
                raise RuntimeError(f'op {name}')
            if r is not None:
                self.rf[dst] = r & M32
        raise RuntimeError(f'no END after {limit} steps (pc={pc})')
