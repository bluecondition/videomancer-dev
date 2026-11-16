#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
FRESNEL v0.2.0 (Hardline) - build hook.  Writes fresnel_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.hardline_work/rb_fresnel/imodel.py) reads the same
tables and runs the same microcode through run() below.

Package contents:
  C_FR_EXP   256 x 16    round(32768 * 2^(i/256))                (the zone scale is exponential)
  C_FR_SIN   1024 x 16   round(32767 * sin(i pi / 2048))         (a quarter wave, 4096 points per cycle: Turn)
  C_FR_MC    512 x 16    microcode of the field engine           (op[15:12] x[11:8] y[7:0])
  C_FR_RF    128 x 48    register-file start values (constants)
  C_MC_FIELD / C_MC_LSTEP / C_MC_LLOAD   entry points

The field engine is a small accumulator machine: a 128 x 48 register file in block RAM, A (48, shifts left),
B (18, signed, shifts right) and ACC (48).  Instructions:
  NOP                    LD dest, reg     dest <- RF[reg]           (A, B, or one of the beam / constant registers)
  LDX src                B <- an outside value (a knob glide, a ROM output, a video-mode constant)
  ST reg                 RF[reg] <- ACC
  ADD sign               ACC <- ACC +- A                            MUL sign    ACC <- ACC +- A * B  (18 clocks)
  CLR                    ACC <- 0                                   SHR n       ACC >>= n (arithmetic, n clocks)
  J cond, label          SPC code         latches and flags
A register number carries a bank in its top two bits: 0 = global, 1 = per plate (plate 1 reads 64 + reg),
2 = "swap with the neighbour when Turn is in an odd quadrant" (sin / cos).
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- sequencer constants -------------------------------------------------------------------------------
E0_12 = 105800        # log2(k0 * 2^38) at K3 = 0, Q12        (k0 = 1 / (2 rz^2) cycles per vpx^2)
ESLOPE = 25083        # e12 = (E0 << 16 - g3 * ESLOPE) >> 16
XP0_12 = 74457        # log2(P8) = XP0 - e12 / 2              (Raw lattice pitch P = rz / 10, P8 in 1/8 vpx)
FM0 = 3441            # (1 - beta) / 2 at K4 = 0, Q15          (the round end is a 9:8 ellipse)
FMK = 29356
SDEAD = 24 * 64       # Speed: |P12 - 512| < 24 counts is still (Q6)


def exp2_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def sin_rom():
    return [int(round(32767 * math.sin(i * math.pi / 2048.0))) for i in range(1024)]


# ---- instruction set -----------------------------------------------------------------------------------
NOP, LD, LDX, ST, ADD, MUL, CLR, SHR, J, SPC = range(10)
# LD destinations
D_A, D_B, D_BS, D_BDX, D_BGY, D_CDDX, D_CGXY, D_P8, D_BX0 = range(9)
# LDX sources
X_G1, X_G2, X_G3, X_G4, X_G5, X_G6, X_G7, X_EXP, X_LO, X_POW2, X_SIN, X_XMC, X_TXC, X_TYC, X_X8 = range(15)
# signs
S_P, S_M, S_NC, S_PC, S_NS, S_PL, S_NPL = range(7)      # +, -, - if cos < 0, - if cos >= 0, - if sin < 0, - on plate 1, - on plate 0
# jump conditions
C_AL, C_NEG, C_POS, C_RAW, C_PLT, C_PAIR = range(6)
# SPC codes
P_END, P_EXA, P_SHA, P_SNA0, P_SNA1, P_ROW0, P_ROW1 = range(7)

# ---- register file -------------------------------------------------------------------------------------
_g = ['ZERO', 'ONE', 'TWO', 'C16', 'C15', 'C384', 'C4320', 'C8640', 'C16384', 'C32768', 'C65536', 'CESL', 'CE0S',
      'CFMK', 'CFM0S', 'CXP0', 'VS', 'T0', 'E12', 'K0', 'P8', 'FM', 'SN0', 'SN1', 'FMC', 'FMS', 'VF', 'FA', 'FC',
      'FB', 'CX', 'CY', 'C21', 'C1536', 'U', 'XM', 'YT', 'TX', 'TY', 'BX0', 'BY0', 'BY', 'P128', 'TYL']
_p = ['OFF', 'S0', 'E', 'EE', 'EE8', 'DX0', 'DDX', 'GXY', 'GY', 'KP', 'CA', 'CA2', 'CC', 'CC2', 'CB', 'DXM', 'DYT',
      'GX0', 'GY0', 'AT']
assert len(_g) == 44 and len(_p) == 20
R = {n: i for i, n in enumerate(_g)}
R.update({n: 0x40 | (44 + i) for i, n in enumerate(_p)})      # bank 1: per plate
R['SNC'] = 0x80 | R['SN1']       # |cos|: SN1, or SN0 in an odd quadrant
R['SNS'] = 0x80 | R['SN0']       # |sin|
assert R['SN0'] % 2 == 0


def rf_init():
    rf = [0] * 128
    for n, v in dict(ONE=1, TWO=2, C16=16, C15=15, C384=384, C21=21, C1536=SDEAD, C4320=4320, C8640=8640, C16384=16384, C32768=32768,
                     C65536=65536, CESL=ESLOPE, CE0S=E0_12 << 16, CFMK=FMK, CFM0S=FM0 << 16, CXP0=XP0_12,
                     ).items():
        rf[R[n]] = v
    off = R['OFF'] & 63
    rf[off] = 3 << 29            # plate a: 3/8 cycle
    rf[64 + off] = 5 << 29       # plate b: 5/8 (no plate centre sits on a zone edge; a quarter cycle apart)
    return rf


# ---- the program ---------------------------------------------------------------------------------------
def program():
    """Returns (words, entry points)."""
    code = []          # (op, x, y or label name)
    labels = {}

    def emit(op, x=0, y=0):
        if op == LD and code and code[-1][0] == ST and (code[-1][2] & 63) == (y & 63):
            code.append((NOP, 0, 0))            # never read a register in the clock it is written
        code.append((op, x, y))

    def label(n):
        if len(code) % 2:
            code.append((NOP, 0, 0))            # jump targets are even
        labels[n] = len(code)

    def lda(r): emit(LD, D_A, R[r])
    def ldb(r): emit(LD, D_B, R[r])
    def ldx(s): emit(LDX, s)
    def st(r): emit(ST, 0, R[r])
    def add(s=S_P): emit(ADD, s)
    def mul(s=S_P): emit(MUL, s)
    def clr(): emit(CLR)
    def shr(n): emit(SHR, 0, n)
    def jmp(c, l):
        emit(J, c, l)
        if c == C_PLT:
            code.append((NOP, 0, 0))            # the plate flips in the jump's X clock: nothing may decode a register then
    def spc(c): emit(SPC, c)
    def nop(): emit(NOP)
    def setacc(r): clr(); lda(r); add()
    def getx(s, k='ONE', sg=S_P): lda(k); ldx(s); mul(sg)          # ACC +- = k * outside value

    # ================= FIELD (vblank) =================
    label('FIELD')
    # centre: cx = (g1 - 32768) * 2, cy = g2 - 32768           (1/16 vpx: 8 and 4 vpx per knob count)
    clr(); getx(X_G1, 'TWO'); lda('C65536'); add(S_M); st('CX')
    clr(); getx(X_G2); lda('C32768'); add(S_M); st('CY')
    # SPEED (P12, bipolar, the Hypnos shape): U, the ring phase (Q32 cycles, kept in the register file from
    # field to field and used modulo one cycle), moves by 2^-12 .. 2^-2 cycle per field.  v = P12 - 512
    # (glided, Q6); |v| < 24 counts is still; past that the step doubles every 64 / 1.3125 counts.
    # Up = U falls = plate a's rings stream outward (plate b, which takes -U, streams inward); Pair halves the
    # step, so the beat stripes (which move by twice the plates' step) march at the decoded speed.
    clr(); getx(X_G7); lda('C32768'); add(S_M); st('VS')          # v, Q6
    jmp(C_POS, 'SABS')
    st('T0'); clr(); lda('T0'); add(S_M)                          # |v|
    label('SABS')
    lda('C1536'); add(S_M); jmp(C_NEG, 'SDONE')                   # u = |v| - 24 counts (inside: still)
    st('T0'); clr(); lda('T0'); ldb('C21'); mul(); shr(4)         # octaves, Q12 (1.3125 / 64 per count)
    spc(P_EXA); shr(12); spc(P_SHA)
    clr(); getx(X_EXP); st('T0')                                  # 2^frac, Q15
    clr(); lda('T0'); ldx(X_POW2); mul(); st('T0')                # << octave
    clr(); lda('T0'); ldb('C16'); mul(); lda('T0'); ldb('C16'); mul()   # x 32: Q32 cycles per field
    jmp(C_PAIR, 'SHALF')
    jmp(C_AL, 'SSTEP')
    label('SHALF')
    shr(1)
    label('SSTEP')
    st('T0')
    setacc('VS'); jmp(C_NEG, 'SDN')
    setacc('U'); lda('T0'); add(S_M); st('U'); jmp(C_AL, 'SDONE')
    label('SDN')
    setacc('U'); lda('T0'); add(S_P); st('U')
    label('SDONE')
    # zone scale: e12 = log2(k0 2^38), Q12;  k0 = 2^e
    setacc('CE0S'); getx(X_G3, 'CESL', S_M); shr(16); st('E12')
    spc(P_EXA); shr(12); lda('C15'); add(S_M); spc(P_SHA)
    clr(); getx(X_EXP); st('T0')
    clr(); lda('T0'); ldx(X_LO); mul(); shr(16); st('T0')
    clr(); lda('T0'); ldx(X_POW2); mul(); st('K0')
    # Raw lattice pitch P8 = 2^(XP0 - e12 / 2)
    setacc('E12'); shr(1); st('T0')
    setacc('CXP0'); lda('T0'); add(S_M)
    spc(P_EXA); shr(12); spc(P_SHA)
    clr(); getx(X_EXP); st('T0')
    clr(); lda('T0'); ldx(X_POW2); mul(); lda('C16384'); add(); shr(15); st('P8')
    clr(); lda('P8'); ldb('C16'); mul(); st('P128')
    clr(); getx(X_TYC); st('TYL')
    # Form: fm = (1 - beta) / 2, Q15
    setacc('CFM0S'); getx(X_G4, 'CFMK'); shr(16); st('FM')
    # Turn: |sin|, |cos| of twice the angle
    spc(P_SNA0); nop(); clr(); getx(X_SIN); st('SN0')
    spc(P_SNA1); nop(); clr(); getx(X_SIN); st('SN1')
    clr(); lda('FM'); ldb('SNC'); mul(); shr(15); st('FMC')
    clr(); lda('FM'); ldb('SNS'); mul(); shr(15); st('FMS')
    setacc('C32768'); lda('FM'); add(S_M); st('VF')
    lda('FMC'); add(S_NC); st('FA')                               # Fa = (1 - fm) + fm cos
    setacc('VF'); lda('FMC'); add(S_PC); st('FC')                 # Fc = (1 - fm) - fm cos
    clr(); lda('FMS'); add(S_NS); add(S_NS); st('FB')             # Fb = 2 fm sin
    # geometry of the sampling lattice
    jmp(C_RAW, 'GRAW')
    clr(); getx(X_XMC); st('XM')
    setacc('C8640'); st('YT')
    clr(); getx(X_TXC); st('TX')
    clr(); getx(X_TYC); st('TY')
    clr(); st('BX0'); st('BY0')
    jmp(C_AL, 'GDONE')
    label('GRAW')
    clr(); getx(X_X8, 'ONE', S_M); lda('P8')
    label('LX')
    add(); jmp(C_NEG, 'LX')
    st('BX0')                                                     # kx P8 - X8
    clr(); getx(X_X8, 'TWO'); lda('BX0'); add(); add(); lda('P8'); add(); st('XM')   # 2 kx P8 + P8
    clr(); lda('C4320'); add(S_M); lda('P8')
    label('LY')
    add(); jmp(C_NEG, 'LY')
    st('T0')                                                      # ky P8 - 4320
    clr(); lda('T0'); ldb('C16'); mul(); st('BY0')
    setacc('C8640'); lda('T0'); add(); add(); lda('P8'); add(S_M); st('YT')          # 2 ky P8 - P8
    setacc('P128'); st('TX'); st('TY')
    label('GDONE')
    emit(LD, D_P8, R['P8']); emit(LD, D_BX0, R['BX0'])
    setacc('BY0'); st('BY'); spc(P_ROW1)
    # ---- per plate ----
    label('PLATE')
    setacc('K0'); st('KP')
    clr(); lda('KP'); ldb('FA'); mul(); shr(15); st('CA'); lda('CA'); add(); st('CA2')
    clr(); lda('KP'); ldb('FC'); mul(); shr(15); st('CC'); lda('CC'); add(); st('CC2')
    clr(); lda('KP'); ldb('FB'); mul(); shr(15); st('CB')
    clr(); lda('XM'); add(S_M); lda('CX'); add(S_NPL); st('DXM')  # first sample minus centre
    clr(); lda('YT'); add(S_M); lda('CY'); add(S_NPL); st('DYT')
    clr(); lda('CA2'); ldb('DXM'); mul(); lda('CB'); ldb('DYT'); mul(); shr(10); st('GX0')
    clr(); lda('CB'); ldb('DXM'); mul(); lda('CC2'); ldb('DYT'); mul(); shr(10); st('GY0')
    clr(); lda('GX0'); ldb('DXM'); mul(); lda('GY0'); ldb('DYT'); mul(); shr(5)
    lda('OFF'); add(); lda('U'); add(S_PL); st('S0')
    clr(); lda('CA'); ldb('TX'); mul(); shr(13); st('AT'); lda('GX0'); add(); st('T0')
    clr(); lda('T0'); ldb('TX'); mul(); shr(7); st('DX0')
    clr(); lda('AT'); ldb('TX'); mul(); shr(6); st('DDX')
    clr(); lda('CC'); ldb('TY'); mul(); shr(13); st('AT'); lda('GY0'); add(); st('T0')
    clr(); lda('T0'); ldb('TY'); mul(); shr(7); st('E')
    clr(); lda('AT'); ldb('TY'); mul(); shr(6); st('EE'); shr(8); st('EE8')
    clr(); lda('CB'); ldb('TX'); mul(); shr(13); st('AT')
    clr(); lda('AT'); ldb('TY'); mul(); shr(7); st('GXY')
    clr(); lda('GY0'); ldb('TY'); mul(); shr(15); st('GY')
    emit(LD, D_CDDX, R['DDX']); emit(LD, D_CGXY, R['GXY'])
    jmp(C_PLT, 'PLATE')
    # ================= LLOAD (after each line, and after FIELD): the next line's first values =================
    label('LLOAD')
    emit(LD, D_BS, R['S0']); emit(LD, D_BDX, R['DX0']); emit(LD, D_BGY, R['GY'])
    jmp(C_PLT, 'LLOAD')
    spc(P_END)
    # ================= LSTEP (once per line, while the beam runs) =================
    label('LSTEP')
    jmp(C_RAW, 'LRAW')
    label('LDO')
    setacc('S0'); lda('E'); add(); st('S0')
    setacc('E'); lda('EE'); add(); st('E')
    setacc('DX0'); lda('GXY'); add(); st('DX0')
    setacc('GY'); lda('EE8'); add(); st('GY')
    jmp(C_PLT, 'LDO')
    spc(P_END)
    label('LRAW')                                                 # Raw: step only into a new lattice row
    setacc('BY'); lda('TYL'); add(); lda('P128'); add(S_M); jmp(C_NEG, 'LNO')
    st('BY'); spc(P_ROW1); jmp(C_AL, 'LDO')
    label('LNO')
    add(); st('BY'); spc(P_ROW0); spc(P_END)

    words = []
    for op, x, y in code:
        if op == J:
            assert labels[y] % 2 == 0
            y = labels[y] // 2
        assert 0 <= y < 256 and 0 <= x < 16
        words.append((op << 12) | (x << 8) | y)
    assert len(words) <= 512, len(words)
    return words + [0x9000] * (512 - len(words)), labels          # pad with END


# ---- instruction-level interpreter (the model runs the engine through this) ---------------------------------
M48 = (1 << 48) - 1


def s48(v):
    v &= M48
    return v - (1 << 48) if v >> 47 else v


def s18(v):
    v &= 0x3FFFF
    return v - (1 << 18) if v >> 17 else v


def run(words, pc, rf, ext, st):
    """Run from pc to END.  rf: 128 ints (signed 48).  ext(src, st) -> the outside value.
    st: dict with raw, q0, nC, nS, and the latches (exa, lo, sh, sn2, row, live registers).  Returns clocks used."""
    acc = st.get('acc', 0); A = st.get('A', 0); B = st.get('B', 0)
    pl = 0
    clocks = 0

    def addr(y):
        b, r = y >> 6, y & 63
        if b == 1: return r + (64 if pl else 0)
        if b == 2: return r ^ (1 if st['q0'] else 0)
        return r

    def neg(s):
        return [False, True, bool(st['nC']), not st['nC'], bool(st['nS']), bool(pl), not pl][s]

    while True:
        w = words[pc]; op, x, y = w >> 12, (w >> 8) & 15, w & 255
        pc += 1; clocks += 1
        if op == LD:
            v = rf[addr(y)]
            if x == D_A: A = v
            elif x == D_B: B = s18(v)
            else: st[('live', x, pl)] = v
        elif op == LDX:
            B = s18(ext(x, st))
        elif op == ST:
            rf[addr(y)] = acc
        elif op == ADD:
            acc = s48(acc - A if neg(x) else acc + A)
        elif op == MUL:
            acc = s48(acc - A * B if neg(x) else acc + A * B)
            A = B = None; clocks += 17
        elif op == CLR:
            acc = 0
        elif op == SHR:
            acc >>= y; clocks += y - 1
        elif op == J:
            if x == C_AL: t = True
            elif x == C_NEG: t = acc < 0
            elif x == C_POS: t = acc >= 0
            elif x == C_RAW: t = bool(st['raw'])
            elif x == C_PAIR: t = bool(st['pair'])
            else:
                t = pl == 0
                pl ^= 1
            if t:
                pc = y * 2; clocks += 2
        elif op == SPC:
            if x == P_END: break
            elif x == P_EXA: st['exa'] = (acc >> 4) & 255; st['lo'] = acc & 15
            elif x == P_SHA: st['sh'] = acc & 31
            elif x == P_SNA0: st['sn2'] = 0
            elif x == P_SNA1: st['sn2'] = 1
            elif x == P_ROW0: st['row'] = 0
            elif x == P_ROW1: st['row'] = 1
    st['acc'] = acc; st['A'] = A; st['B'] = B
    return clocks


def vhd_table(name, typ, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['x"%0*X"' % (width // 4, v & ((1 << width) - 1)) for v in vals]
    for i in range(0, len(items), 8 if width > 16 else 16):
        n = 8 if width > 16 else 16
        lines.append("        " + ", ".join(items[i:i + n]) + ("," if i + n < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    words, labels = program()
    parts = ["-- Generated by fresnel.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package fresnel_pkg is",
             vhd_table("C_FR_EXP", "unsigned", 256, 16, exp2_rom()),
             vhd_table("C_FR_SIN", "unsigned", 1024, 16, sin_rom()),
             vhd_table("C_FR_MC", "std_logic_vector", 512, 16, words),
             vhd_table("C_FR_RF", "signed", 128, 48, rf_init()),
             "    constant C_MC_FIELD : integer := %d;" % labels['FIELD'],
             "    constant C_MC_LSTEP : integer := %d;" % labels['LSTEP'],
             "    constant C_MC_LLOAD : integer := %d;" % labels['LLOAD'],
             "end package fresnel_pkg;", ""]
    with open(os.path.join(HERE, "fresnel_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
