#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
WOODGRAIN - 6502 instruction table (documented set + the stable illegal opcodes
that Atari 2600 code uses).  Shared by the assembler (asm6502.py), the reference
model (vcs_model.py) and the decode-ROM generator (gen_decode.py).

OPS[opcode] = (mnemonic, mode, cycles, page_penalty)
  mode: imp acc imm zp zpx zpy abs abx aby izx izy rel ind
  page_penalty: +1 cycle when an indexed READ crosses a page (abx/aby/izy)
"""

MODES_LEN = {'imp': 1, 'acc': 1, 'imm': 2, 'zp': 2, 'zpx': 2, 'zpy': 2, 'abs': 3,
             'abx': 3, 'aby': 3, 'izx': 2, 'izy': 2, 'rel': 2, 'ind': 3}

OPS = {}


def _grp(mn, table, pen=True):
    for mode, (op, cyc) in table.items():
        OPS[op] = (mn, mode, cyc, pen and mode in ('abx', 'aby', 'izy'))


# group-1 read ops (ORA AND EOR ADC LDA CMP SBC)
for mn, base in (('ORA', 0x00), ('AND', 0x20), ('EOR', 0x40), ('ADC', 0x60),
                 ('LDA', 0xA0), ('CMP', 0xC0), ('SBC', 0xE0)):
    _grp(mn, {'izx': (base + 0x01, 6), 'zp': (base + 0x05, 3), 'imm': (base + 0x09, 2),
              'abs': (base + 0x0D, 4), 'izy': (base + 0x11, 5), 'zpx': (base + 0x15, 4),
              'aby': (base + 0x19, 4), 'abx': (base + 0x1D, 4)})
_grp('STA', {'izx': (0x81, 6), 'zp': (0x85, 3), 'abs': (0x8D, 4), 'izy': (0x91, 6),
             'zpx': (0x95, 4), 'aby': (0x99, 5), 'abx': (0x9D, 5)}, pen=False)

# shifts / inc / dec (RMW)
for mn, base in (('ASL', 0x00), ('ROL', 0x20), ('LSR', 0x40), ('ROR', 0x60)):
    _grp(mn, {'zp': (base + 0x06, 5), 'acc': (base + 0x0A, 2), 'abs': (base + 0x0E, 6),
              'zpx': (base + 0x16, 6), 'abx': (base + 0x1E, 7)}, pen=False)
_grp('DEC', {'zp': (0xC6, 5), 'abs': (0xCE, 6), 'zpx': (0xD6, 6), 'abx': (0xDE, 7)}, pen=False)
_grp('INC', {'zp': (0xE6, 5), 'abs': (0xEE, 6), 'zpx': (0xF6, 6), 'abx': (0xFE, 7)}, pen=False)

# loads / stores of X and Y, compares
_grp('LDX', {'imm': (0xA2, 2), 'zp': (0xA6, 3), 'abs': (0xAE, 4), 'zpy': (0xB6, 4), 'aby': (0xBE, 4)})
_grp('LDY', {'imm': (0xA0, 2), 'zp': (0xA4, 3), 'abs': (0xAC, 4), 'zpx': (0xB4, 4), 'abx': (0xBC, 4)})
_grp('STX', {'zp': (0x86, 3), 'abs': (0x8E, 4), 'zpy': (0x96, 4)}, pen=False)
_grp('STY', {'zp': (0x84, 3), 'abs': (0x8C, 4), 'zpx': (0x94, 4)}, pen=False)
_grp('CPX', {'imm': (0xE0, 2), 'zp': (0xE4, 3), 'abs': (0xEC, 4)})
_grp('CPY', {'imm': (0xC0, 2), 'zp': (0xC4, 3), 'abs': (0xCC, 4)})
_grp('BIT', {'zp': (0x24, 3), 'abs': (0x2C, 4)})

# branches
for mn, op in (('BPL', 0x10), ('BMI', 0x30), ('BVC', 0x50), ('BVS', 0x70),
               ('BCC', 0x90), ('BCS', 0xB0), ('BNE', 0xD0), ('BEQ', 0xF0)):
    OPS[op] = (mn, 'rel', 2, False)

# control flow / stack
OPS[0x00] = ('BRK', 'imp', 7, False)
OPS[0x20] = ('JSR', 'abs', 6, False)
OPS[0x40] = ('RTI', 'imp', 6, False)
OPS[0x60] = ('RTS', 'imp', 6, False)
OPS[0x4C] = ('JMP', 'abs', 3, False)
OPS[0x6C] = ('JMP', 'ind', 5, False)
OPS[0x08] = ('PHP', 'imp', 3, False)
OPS[0x28] = ('PLP', 'imp', 4, False)
OPS[0x48] = ('PHA', 'imp', 3, False)
OPS[0x68] = ('PLA', 'imp', 4, False)

# implied register / flag ops
for mn, op in (('CLC', 0x18), ('SEC', 0x38), ('CLI', 0x58), ('SEI', 0x78), ('CLV', 0xB8),
               ('CLD', 0xD8), ('SED', 0xF8), ('DEY', 0x88), ('TXA', 0x8A), ('TYA', 0x98),
               ('TXS', 0x9A), ('TAY', 0xA8), ('TAX', 0xAA), ('TSX', 0xBA), ('INY', 0xC8),
               ('DEX', 0xCA), ('INX', 0xE8), ('NOP', 0xEA)):
    OPS[op] = (mn, 'imp', 2, False)

# ---- stable illegal opcodes ----
for mn, base in (('SLO', 0x00), ('RLA', 0x20), ('SRE', 0x40), ('RRA', 0x60),
                 ('DCP', 0xC0), ('ISB', 0xE0)):
    _grp(mn, {'izx': (base + 0x03, 8), 'zp': (base + 0x07, 5), 'abs': (base + 0x0F, 6),
              'izy': (base + 0x13, 8), 'zpx': (base + 0x17, 6), 'aby': (base + 0x1B, 7),
              'abx': (base + 0x1F, 7)}, pen=False)
_grp('SAX', {'izx': (0x83, 6), 'zp': (0x87, 3), 'abs': (0x8F, 4), 'zpy': (0x97, 4)}, pen=False)
_grp('LAX', {'izx': (0xA3, 6), 'zp': (0xA7, 3), 'abs': (0xAF, 4), 'izy': (0xB3, 5),
             'zpy': (0xB7, 4), 'aby': (0xBF, 4)})
OPS[0x0B] = ('ANC', 'imm', 2, False)
OPS[0x2B] = ('ANC', 'imm', 2, False)
OPS[0x4B] = ('ALR', 'imm', 2, False)
OPS[0x6B] = ('ARR', 'imm', 2, False)
OPS[0xCB] = ('SBX', 'imm', 2, False)
OPS[0xAB] = ('LXA', 'imm', 2, False)
OPS[0xEB] = ('SBC', 'imm', 2, False)
for op in (0x1A, 0x3A, 0x5A, 0x7A, 0xDA, 0xFA):
    OPS[op] = ('NOP', 'imp', 2, False)
for op in (0x80, 0x82, 0x89, 0xC2, 0xE2):
    OPS[op] = ('NOP', 'imm', 2, False)
for op in (0x04, 0x44, 0x64):
    OPS[op] = ('NOP', 'zp', 3, False)
for op in (0x14, 0x34, 0x54, 0x74, 0xD4, 0xF4):
    OPS[op] = ('NOP', 'zpx', 4, False)
OPS[0x0C] = ('NOP', 'abs', 4, False)
for op in (0x1C, 0x3C, 0x5C, 0x7C, 0xDC, 0xFC):
    OPS[op] = ('NOP', 'abx', 4, True)
# Everything else (JAM and the unstable SHA/SHX/SHY/TAS/LAS/ANE) runs as a
# 1-byte, 2-cycle NOP: no 2600 game we target relies on them.
for op in range(256):
    if op not in OPS:
        OPS[op] = ('NOP', 'imp', 2, False)

# operation classes for the decoder
READ_OPS = {'ORA', 'AND', 'EOR', 'ADC', 'LDA', 'CMP', 'SBC', 'LDX', 'LDY', 'CPX', 'CPY',
            'BIT', 'LAX', 'NOP'}
WRITE_OPS = {'STA', 'STX', 'STY', 'SAX'}
RMW_OPS = {'ASL', 'ROL', 'LSR', 'ROR', 'DEC', 'INC', 'SLO', 'RLA', 'SRE', 'RRA', 'DCP', 'ISB'}

# preferred encodings for the assembler (documented first)
ENC = {}
for op in sorted(OPS):
    mn, mode, _, _ = OPS[op]
    if (op in (0xEB,) or (mn == 'NOP' and op != 0xEA)):
        continue
    ENC.setdefault((mn, mode), op)
