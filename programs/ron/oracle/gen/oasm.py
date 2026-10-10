#!/usr/bin/env python3
"""ORACLE microcode assembler (adapted from cubist/uasm.py, HW-validated).

One small datapath does every per-field and per-line computation: one ALU,
a block-RAM register file (two copies, dual read), the program in block RAM.

Instruction: 32 bits, 256 registers
  [31:27] op   [26:19] dst   [18:11] srcA   [10:3] srcB   [2:0] i3

Immediates overlap the fields an op does not use:
  LDI            imm19 = [18:0]  (sign-extended)
  AND            imm11 = [10:0]  (zero-extended)
  JMP JNZ        target = [10:0]
  JLT JGE        target = [26:19] & [2:0]   (a branch writes no register)
  OUT            port = [26:19]
  SHR SHL        shift = [6:3]
  MRD            [0] set = product >> 12
  DIV            [0] set = numerator << 16
  CTL            select = [14:11]
  WAIT           [11]: 0 = field tick, 1 = line trigger (a pending field
                 tick restarts the program at 0 instead)
  XFER           block-copies rf[srcA field .. +NXF-1] into the pixel-path line
                 registers (stalls until done)
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'MUL': 10,   # start serial multiply A * B  (A 20-bit signed, B 16-bit signed)
    'MRD': 11,   # dst = product (>> 12)        (stalls until idle)
    'DIV': 12,   # start divide: (A << 0|16) / B, MAGNITUDES only
    'DRD': 13,   # dst = quotient, clamped at 2^20 - 1 (stalls until idle)
    'SIN': 14,   # dst = sin(A[9:0]) (10-bit angle, signed 13-bit result)
    'LDI': 15,   # dst = imm14
    'PRV': 16,   # dst = vesica profile[A & 255]
    'OUT': 17,   # output port <= A
    'JMP': 18,   # pc = target
    'JNZ': 19,   # pc = target if A /= 0
    'JLT': 20,   # pc = target if A < B
    'JGE': 21,   # pc = target if A >= B
    'XFER': 22,  # line-register block transfer from rf[A ..]
    'PRC': 23,   # dst = circle profile[A & 255]
    'WAIT': 24,  # 0 = field tick, 1 = line trigger
    'CTL': 25,   # dst = control input imm
    'JR':  26,   # pc = A  (subroutine return)
    'AND': 27,   # dst = A and imm11
    'MIN': 28,   # dst = A < B ? A : B  (signed)
    'MAX': 29,   # dst = A < B ? B : A  (signed)
}

# output ports (must match the VHDL decoder)
PORTS = {
    'cx': 0, 'cA': 2, 'kS1': 4, 'kS2': 5, 'kW1x': 6,
    'kW2x': 7, 'tM': 10, 'tG': 11, 'tRg': 12, 'spk': 13,
    'lenm': 14, 'as4': 15, 'kR': 16, 'se': 17, 'sm': 18, 'g3': 19, 'rsh': 20,
    'pup2': 21, 'boost': 22, 'wx0': 23, 'wx1m': 24, 'xhm1': 25, 'tseed': 26,
    'yA': 29, 'yC': 30, 'wy0': 31, 'wy1': 32,
    'hs2': 35, 'hxm1': 36,   # rho^2 DDA
}

# control inputs for CTL
CTLR = {
    'k0': 0, 'k1': 1, 'k2': 2, 'k3': 3, 'k4': 4, 'k5': 5, 'sw': 6, 'k7': 7,
    'fh': 8, 'mh': 9, 'ilace': 10, 'botf': 11, 'rand': 12,
}

# line-register block (XFER order; must match the VHDL)
XFER_FIELDS = ['rc', 'phA', 'w1', 'w2', 'v2', 'lb', 'w4', 'eye', 'k', 'phC',
               'swA', 'ah', 'aref', 'ref', 'rx0', 'rx1', 'd', 'dc',
               # per-field DDA start values, reloaded into the DDAs each line
               'frA', 'tw1', 'tw2', 'hd0', 'hq0']
NXF = len(XFER_FIELDS)


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
        # NEG/MIN/MAX/CLP/ABS are macros: the ALU keeps one adder and a
        # small result mux; the engine has clocks to spare.
        if op == 'NEG':
            self.emit('SUB', dst=dst, a=self.zero, b=a)
            return
        if op == 'ABS':                              # max(a, -a)
            assert dst != a, 'ABS needs dst != src'
            self.emit('NEG', dst=dst, a=a)
            self.emit('MAX', dst=dst, a=a, b=dst)
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
        if op in ('SHR', 'SHL') and imm > 15:      # long shifts: chain
            k = imm
            src = a
            while k:
                s = min(k, 15)
                self.emit(op, dst=dst, a=src, imm=s)
                src, k = dst, k - s
            return
        assert op in OPS, f'{op} is not implemented'
        if op == 'MRD': assert imm in (0, 12), f'MRD shift {imm}'
        if op == 'DIV': assert imm in (0, 16), f'DIV shift {imm}'
        if op in ('SHR', 'SHL'): assert 0 <= imm <= 15, imm
        if op == 'AND': assert 0 <= imm < 2048, imm
        if op == 'LDI' and not isinstance(imm, str):
            assert -(1 << 18) <= imm < (1 << 18), imm
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
            assert 0 <= dst < 256 and 0 <= a < 256 and 0 <= b < 256
            w = op << 27
            if name in ('JLT', 'JGE'):
                assert 0 <= imm < 2048
                w |= (imm >> 3) << 19 | a << 11 | b << 3 | (imm & 7)
            elif name in ('JMP', 'JNZ'):
                assert 0 <= imm < 2048
                w |= a << 11 | imm
            elif name == 'LDI':
                assert -(1 << 18) <= imm < (1 << 18)
                w |= dst << 19 | (imm & 0x7FFFF)
            elif name == 'AND':
                assert 0 <= imm < 2048
                w |= dst << 19 | a << 11 | imm
            elif name in ('SHR', 'SHL'):
                assert 0 <= imm < 16
                w |= dst << 19 | a << 11 | imm << 3
            elif name in ('MRD', 'DIV'):
                w |= dst << 19 | a << 11 | b << 3 | (1 if imm else 0)
            elif name == 'OUT':
                assert 0 <= imm < 256
                w |= imm << 19 | a << 11
            elif name == 'CTL':
                assert 0 <= imm < 16
                w |= dst << 19 | imm << 11
            elif name == 'WAIT':
                w |= (imm & 1) << 11
            else:           # register ops
                w |= dst << 19 | a << 11 | b << 3
            out.append(w)
        return out


def emit_vhdl(words, name='C_UCODE', depth=512):
    assert len(words) <= depth, f'program {len(words)} > {depth}'
    words = list(words) + [0] * (depth - len(words))
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:08X}"')
        if len(row) == 6 or i == depth - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_ucode is array (0 to {depth-1}) of std_logic_vector(31 downto 0);\n'
            f'    constant {name} : t_ucode := (\n' + ',\n'.join(rows) + ' );')


def emit_rf_init(init, name='C_RFINIT'):
    vals = [init.get(i, 0) & 0xFFFFFFFF for i in range(256)]
    rows, row = [], []
    for i, v in enumerate(vals):
        row.append(f'x"{v:08X}"')
        if len(row) == 6 or i == 255:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_rf is array (0 to 255) of std_logic_vector(31 downto 0);\n'
            f'    constant {name} : t_rf := (\n' + ',\n'.join(rows) + ' );')
