#!/usr/bin/env python3
"""CUBIST microcode assembler.

The frame setup is ~110 hand-written FSM states, each carrying its own
adders, comparators and address arithmetic.  Netlist attribution put that
at 2,176 cells plus 490 more in the muxes feeding the shared multiplier --
a quarter of the program, for math that runs once per frame during
blanking with ~99,000 spare clocks.

This replaces it with one datapath: a single ALU, the scratch RAM as a
register file, and the program in a block RAM (of which 21 are spare).
The arithmetic is identical, so nothing visual moves.

Instruction: 40 bits
  op(5) | dst(7) | srcA(7) | srcB(7) | imm(14)
"""

OPS = {
    'NOP': 0, 'MOV': 1, 'ADD': 2, 'SUB': 3, 'SHR': 4, 'SHL': 5,
    'NEG': 6, 'ABS': 7, 'MIN': 8, 'MAX': 9,
    'MUL': 10,   # start serial multiply of A*B
    'MRD': 11,   # dst = mu_p >> imm  (stalls until the multiplier is idle)
    'DIV': 12,   # start divide: numerator = A << imm, denominator = B
    'DRD': 13,   # dst = quotient (stalls until the divider is idle)
    'SIN': 14,   # dst = quarter-sine ROM at A (raw index, no folding)
    'LDI': 15,   # dst = imm (sign-extended)
    'GWR': 16,   # gram[imm + A] = B
    'SLW': 17,   # per-slot output register imm <= A
    'JMP': 18,   # pc = imm
    'JNZ': 19,   # pc = imm if A /= 0
    'JLT': 20,   # pc = imm if A < B
    'JGE': 21,   # pc = imm if A >= B
    'CLP': 22,   # dst = clamp(A, 0, B)
    'GAM': 23,   # dst = gamma ROM at A (linear Q8 -> video Q8)
    'END': 24,   # frame setup complete
    'CTL': 25,   # dst = control input imm (knobs, raster measurements)
    'SLR': 26,   # dst = per-slot input imm
    'AND': 27,   # dst = A and imm      (bit-field extraction)
    'BIT': 28,   # dst = (A >> imm) and 1
    'SET': 29,   # dst = A with bit imm forced to 1
    'LDX': 30,   # dst = rf[A + imm]   (indexed load)
    'STX': 31,   # rf[A + imm] = B     (indexed store)
}

# per-slot output ports for SLW (must match the VHDL decoder)
SLOTW = {
    'gux': 0, 'guy': 1, 'gvx': 2, 'gvy': 3, 'px0': 4, 'py0': 5,
    'face': 6, 'sil': 7, 'cu': 8, 'cv': 9, 'ly': 10, 'lf': 11,
    'hn': 12, 'hu': 13, 'hv': 14, 'nslot': 15,
    # port 16+ are globals, not slot-indexed
    'qx0': 16, 'pxm': 17, 'pxe': 18,
}

# control inputs for CTL
CTLR = {
    'k1': 0, 'k2': 1, 'k3': 2, 'W': 3, 'H': 4, 'hf': 5,
    'cx': 6, 'cy': 7, 'ilace': 8, 'zoom': 9, 'zfirst': 10,
}


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fixups = []

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def emit(self, op, dst=0, a=0, b=0, imm=0):
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
        out = []
        for op, dst, a, b, imm in self.resolve():
            imm &= 0x3FFF
            out.append((op << 35) | (dst << 28) | (a << 21) | (b << 14) | imm)
        return out


def emit_vhdl(words, name='C_UCODE', depth=512):
    words = list(words) + [0] * (depth - len(words))
    rows, row = [], []
    for i, w in enumerate(words):
        row.append(f'x"{w:010X}"')
        if len(row) == 4 or i == depth - 1:
            rows.append('        ' + ', '.join(row))
            row = []
    return (f'    type t_ucode is array (0 to {depth-1}) of std_logic_vector(39 downto 0);\n'
            f'    constant {name} : t_ucode := (\n' + ',\n'.join(rows) + ' );')


if __name__ == '__main__':
    a = Asm()
    a.emit('LDI', dst=40, imm=1234)
    a.emit('END')
    print(emit_vhdl(a.words(), depth=8))
