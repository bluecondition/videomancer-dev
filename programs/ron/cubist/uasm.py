#!/usr/bin/env python3
"""CUBIST microcode assembler.

One small datapath runs the per-frame geometry in vertical blanking: a
single ALU, a block-RAM register file, and the program in block RAM.

Instruction: 32 bits (1024 words = 8 block RAMs)
  [31:27] op   [26:20] dst   [19:13] srcA   [12:6] srcB   [5:0] i6

Immediates overlap the fields an op does not use:
  LDI            imm14 = [13:0]  (sign-extended; overlaps srcA bit 0 + srcB)
  AND            imm13 = [12:0]  (zero-extended; overlaps srcB)
  JMP JNZ        target = [9:0]
  JLT JGE        target = [23:20] & [5:0]   (a branch writes no register)
  LDX SRD        imm7 = [6:0]
  STX            imm6 = [5:0]   (srcB occupies [12:6])
  SLW            port = [4:0]
  SHR SHL CTL    [3:0]
  MRD            [3] set = product >> 12
  DIV            [4] set = numerator << 16
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'MUL': 10,   # start serial multiply A * B  (A 20-bit, B 16-bit signed)
    'MRD': 11,   # dst = product (>> 12)        (stalls until idle)
    'DIV': 12,   # start divide: (A << 0|16) / B, MAGNITUDES only
    'DRD': 13,   # dst = quotient, clamped at 2^20 - 1 (stalls until idle)
    'SIN': 14,   # dst = quarter-sine ROM at A
    'LDI': 15,   # dst = imm14
    'SRD': 16,   # dst = table RAM[A + imm7]
    'SLW': 17,   # per-slot output port <= A, slot = B
    'JMP': 18,   # pc = target
    'JNZ': 19,   # pc = target if A /= 0
    'JLT': 20,   # pc = target if A < B
    'JGE': 21,   # pc = target if A >= B
    'SWR': 22,   # table RAM[A] = B
    'GAM': 23,   # dst = gamma ROM at A
    'END': 24,   # frame setup complete
    'CTL': 25,   # dst = control input imm
    'JR':  26,   # pc = A  (subroutine return)
    'AND': 27,   # dst = A and imm13
    'LDX': 30,   # dst = rf[A + imm7]
    'STX': 31,   # rf[A + imm7] = B
}

# per-slot output ports for SLW (must match the VHDL decoder)
SLOTW = {
    'gu': 0, 'wu': 1, 'gv': 2, 'wv': 3, 'su': 4, 'sv': 5,   # DDA table
    'fd': 6,     # face descriptor: base<<10 | shape<<7 | gap<<4 | sil
    'h1': 7,     # hu << 6 | hn[5:0]
    'h2': 8,     # hv << 6 | hn[9:6]
    'lf': 9,     # (gamma(lf) >> 3) << 8 | flat face light (linear Q8)
    'nslot': 15,
    'pxm': 17, 'pxe': 18,
    'eu': 19, 'ev': 20,      # block-float flags
    'kick': 25,              # add the field seeds into the DDAs
}

# control inputs for CTL
CTLR = {
    'k1': 0, 'k2': 1, 'k3': 2, 'W': 3, 'H': 4, 'hf': 5,
    'cx': 6, 'cy': 7, 'ilace': 8, 'field': 9, 'zfirst': 10,
    'sw': 11, 'slider': 12, 'rand': 13, 'k6': 14,
}


class Asm:
    zero = None          # register holding 0, for the macros

    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def _lbl(self):
        self._n = getattr(self, '_n', 0) + 1
        return f'__m{self._n}'

    def emit(self, op, dst=0, a=0, b=0, imm=0):
        # NEG/MIN/MAX/CLP/ABS/BIT are macros: the ALU keeps one adder and a
        # small result mux, and the engine has plenty of clocks in blanking.
        if op == 'NEG':
            self.emit('SUB', dst=dst, a=self.zero, b=a)
            return
        if op in ('MIN', 'MAX'):
            l1, l2 = self._lbl(), self._lbl()
            self.emit('JLT', a=a, b=b, imm=l1)
            self.emit('MOV', dst=dst, a=(b if op == 'MIN' else a))
            self.emit('JMP', imm=l2)
            self.label(l1)
            self.emit('MOV', dst=dst, a=(a if op == 'MIN' else b))
            self.label(l2)
            return
        if op == 'ABS':
            l1, l2 = self._lbl(), self._lbl()
            self.emit('JLT', a=a, b=self.zero, imm=l1)
            self.emit('MOV', dst=dst, a=a)
            self.emit('JMP', imm=l2)
            self.label(l1)
            self.emit('NEG', dst=dst, a=a)
            self.label(l2)
            return
        if op == 'CLP':                              # clamp(a, 0, b)
            lz, la, le = self._lbl(), self._lbl(), self._lbl()
            self.emit('JLT', a=a, b=self.zero, imm=lz)
            self.emit('JLT', a=a, b=b, imm=la)
            self.emit('MOV', dst=dst, a=b)
            self.emit('JMP', imm=le)
            self.label(lz)
            self.emit('MOV', dst=dst, a=self.zero)
            self.emit('JMP', imm=le)
            self.label(la)
            self.emit('MOV', dst=dst, a=a)
            self.label(le)
            return
        if op == 'BIT':                              # (a >> imm) and 1
            if imm:
                src = a
                while imm:
                    k = min(imm, 15)
                    self.emit('SHR', dst=dst, a=src, imm=k)
                    src, imm = dst, imm - k
                self.emit('AND', dst=dst, a=dst, imm=1)
            else:
                self.emit('AND', dst=dst, a=a, imm=1)
            return
        assert op in OPS, f'{op} is not implemented'
        if op == 'MRD': assert imm in (0, 12), f'MRD shift {imm}'
        if op == 'DIV': assert imm in (0, 16), f'DIV shift {imm}'
        if op in ('SHR', 'SHL'): assert 0 <= imm <= 15, imm
        if op == 'AND': assert 0 <= imm < 8192, imm
        if op in ('LDX', 'SRD', 'STX'): assert 0 <= imm < 128, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -8192 <= imm < 8192, imm
        if isinstance(imm, str):
            self.fixups.append((len(self.code), imm))
            imm = 0
        self.code.append([OPS[op], dst, a, b, imm])
        return len(self.code) - 1

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
            elif name in ('LDX', 'SRD'):
                w |= dst << 20 | a << 13 | imm
            elif name == 'STX':          # srcB is [12:6]; imm7 must fit [5:0]
                assert imm < 64, imm
                w |= a << 13 | b << 6 | imm
            elif name == 'MRD':
                w |= dst << 20 | (8 if imm == 12 else 0)
            elif name == 'DIV':
                w |= a << 13 | b << 6 | (16 if imm == 16 else 0)
            else:           # register ops; SLW/SHR/SHL/CTL carry a small imm
                assert 0 <= imm < 64, (name, imm)
                w |= dst << 20 | a << 13 | b << 6 | imm
            out.append(w)
        return out


def emit_vhdl(words, name='C_UCODE', depth=1024):
    words = list(words) + [0] * (depth - len(words))
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:08X}"')
        if len(row) == 6 or i == depth - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_ucode is array (0 to {depth-1}) of std_logic_vector(31 downto 0);\n'
            f'    constant {name} : t_ucode := (\n' + ',\n'.join(rows) + ' );')
