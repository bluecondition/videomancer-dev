#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
AQUEDUCT v0.1.0 (Vellum) - build hook.  Writes aqueduct_pkg.vhd next to this file (pure python: the builder
runs it with the system python3).  The integer model (.vellum_work/rb_aqueduct/imodel.py) imports this file
and executes the same microcode, so the per-field constants of model and RTL are identical by construction.

Package contents:
  C_AQ_SQ    256 x 16   h * h + h            (the pixel squarers' high part:
                                              v^2 ~ 4096 (h^2 + h) + 64 (2h + 1)(l - 32) + 1365, v = 64 h + l)
  C_AQ_ROM   512 x 16   0 .. 255  round(32768 * 2^(i/256))    (pitch, depth ratio, pan scale, drift speed)
                        256 .. 287 min(31, round(1024 / (32 + f)))  (the distance normaliser's mantissa)
                        288 ..     CT, the field engine's constants (some chosen by shape / mode / switches)
  C_AQ_PROG  512 x 16   the field engine's program: op(15:11) x(10:8) a(7:0)

The field engine (one accumulator, a 256 x 32 register file, one serial 24 x 16 multiplier):
  END LDC LD ST ADD SUB RSUB ADDC KNB SRA SHL SRAV SHLV MULR MULC MRD EXR MAXC CSUB ADDN ABS NORM OUT LOOP
  x selects the CT entry: 0 none, 1 shape, 2 mode, 3 weight, 4 build * 2 + weight, 5 source, 6 glide, 7 tone.
  mode = 0 1080p, 1 720p, 2 1080i, 3 480p / 576p, 4 480i / 576i.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

OPS = ["END", "LDC", "LD", "ST", "ADD", "SUB", "RSUB", "ADDC", "KNB", "SRA", "SHL", "SRAV", "SHLV", "MULR",
       "MULC", "MRD", "EXR", "MAXC", "CSUB", "ADDN", "ABS", "NORM", "OUT", "LOOP", "MRD8", "TST", "LDF", "MINC",
       "SETK"]
OP = {n: i for i, n in enumerate(OPS)}
SEL = {None: 0, "shape": 1, "mode": 2, "weight": 3, "lw": 4, "osc": 5, "glide": 6, "tone": 7}
SELN = {0: 1, 1: 6, 2: 5, 3: 2, 4: 4, 5: 2, 6: 2, 7: 3}   # entries per selector
NTIER = 2                                                 # tiers: 0 far, 1 near (the loop runs near first)
TK0 = NTIER - 1
BANK = {"B_SPR": 0, "B_K": 1, "B_LS": 2, "B_RC2": 3, "B_TOP": 4, "B_BASE": 5, "B_CUT": 6, "B_G": 8, "B_H": 9}
# (LD / ST with x = 1 address RF 128 + 16 tk + i: the tier constants for the line unit, the knob state)
GLIDED = {"G_X": 128 + 16 * 4 + 8, "G_Y": 128 + 16 * 3 + 8, "G_S": 128 + 16 * 2 + 8, "G_C": 128 + 16 * 1 + 8,
          "G_P": 128 + 8, "H_V": 128 + 16 * 5 + 9}
KNOBS = {"theta": 5, "knob": 8}          # KNB 8 = the raw knob of loop channel tk: 5 K5, 4 K1, 3 K2, 2 K3, 1 K6, 0 P12
KNOB_DEF = {5: 660, 4: 512, 3: 512, 2: 512, 1: 1023, 0: 512}    # TOML defaults: the held / glided values start there
TSTX = {"set": 0, "orneg": 1, "and": 2}
OUTS = ["A0", "STP", "HP5", "E", "QQ", "M", "HB", "CB", "SSH", "SMAX", "GAIN", "SPD", "VNEG", "SPDZ"]
TIER_OUTS = OUTS[:10]
OUT = {n: i for i, n in enumerate(OUTS)}

# CT tables: name -> (selector, values)
CT = {
    "GSH":   ("glide", [0, 3]),
    "GL01":  ("glide", [0, 1]),
    "CA":    ("mode", [965, 965, 965, 965, 965]),         # swell gain: 0.22 P (Q4) at white for the near tier
    "OSCSH": ("osc", [31, 1]),                             # Osc: gain * 1.5
    "STC":   ("mode", [32, 48, 32, 64, 64]),               # DDA step = xs * 16 (Q5 virtual px per pixel)
    "XPIX0": ("mode", [112, 168, 112, 7904, 7904]),        # DDA pixel 0: xbase + 3.5 xs (Q5)
    "AA":    ("mode", [0, 1, 1, 1, 2]),                    # log2 of the anti-alias width (virtual px)
    "RHO":   ("weight", [82, 108]),                        # opening half-width: 0.32 / 0.42 of the pitch
    "RCF":   ("shape", [256, 416, 320, 256, 256, 256]),    # head radius / R (x256)
    "EF":    ("shape", [0, 160, 0, 0, 0, 0]),              # pointed: arc centres 5/8 R across
    "KF":    ("shape", [0, 0, 192, 0, 0, 0]),              # horseshoe: narrows 3/4 R below the centre
    "LSF":   ("shape", [0, 0, 0, 512, 0, 0]),              # stadium: 2 R straight
    "APF":   ("shape", [256, 384, 320, 256, 256, 256]),    # apex (spring below the crown) / R
    "DVA":   ("shape", [256, 256, 256, 256, 256, 362]),    # normaliser divisor = ((Rc DVA) >> 7) << AA: 2 Rc,
                                                           # gable (an L1 distance) 2 sqrt(2) R
    "WR":    ("weight", [9, 3]),                           # archivolt width / p (x256)
    "WL":    ("weight", [8, 3]),                           # lattice line width / p (x256)
    "HM0":   ("tone", [1, 1, 0]),                          # line half-width: archivolt (Grey, Ink) ...
    "HM1":   ("tone", [0, 0, 1]),                          # ... or lattice line
    "CG":    ("tone", [1, 0, 0]),                          # line centre: archivolt gap (Grey) ...
    "CH":    ("tone", [1, 1, 0]),                          # ... + half-width (Grey, Ink: hugging the opening)
}
CONSTS = [0, -512, -24, 769, 429, -4096, 17, 8640, 60, 5, 316, 276, 486, 46, -11, 7, 30720, -8, 256, 13, 32, 128, 24,
          -2, -1, 1022, 127, 8]


def build_ct():
    """CT layout: returns (rom words from 288 on, name -> base index)"""
    words, base = [], {}
    for v in CONSTS:
        base["c%d" % v] = len(words)
        words.append(v)
    for name, (sel, vals) in CT.items():
        assert len(vals) == SELN[SEL[sel]], name
        base[name] = len(words)
        words.extend(vals)
    assert 288 + len(words) <= 512
    return words, base


CTW, CTB = build_ct()


def program():
    """the per-field program, as (op, x, a) with symbolic a"""
    P = []
    lab = {}

    def I(op, a=0, x=None):
        P.append((op, x, a))

    def C(op, name):          # an op with a CT operand
        sel = CT[name][0] if name in CT else None
        I(op, ("ct", name), sel)

    def L(name):
        lab[name] = len(P)

    # ---- the knobs: a +-1 hold (the ends always get through), then Glide On: 1/16 of the gap per field, never
    # slower than one count, landing exactly; Glide Off: the held value.  Channel tk = 5 .. 0: K5 (hold only),
    # K1, K2, K3, K6, P12; state in the register file (G at bank item 8, Q3; H at item 9)
    I("SETK", 5)
    L("GLIDE")
    I("LD", "B_G", "bank"); I("ST", "GT"); I("LD", "B_H", "bank"); I("ST", "HT")
    I("KNB", ("k", "knob")); I("ST", "GR")
    I("SUB", "HT"); I("ABS"); C("ADDC", "c-2"); I("TST", 0, "set")
    I("LD", "GR"); C("ADDC", "c-1"); I("TST", 0, "orneg")
    C("LDC", "c1022"); I("SUB", "GR"); I("TST", 0, "orneg")
    I("LD", "HT"); I("LDF", "GR"); I("ST", "B_H", "bank")
    I("SHL", 3); I("SUB", "GT"); I("ST", "GD")
    C("ADDC", "c128"); I("TST", 0, "set")
    C("LDC", "c127"); I("SUB", "GD"); I("TST", 0, "and")
    I("LD", "GD"); I("SRA", 4); I("ST", "T1")
    I("LD", "GD"); C("MAXC", "c-8"); C("MINC", "c8"); I("ST", "T2")
    I("LD", "T1"); I("LDF", "T2")
    I("SUB", "GD"); C("MULC", "GL01"); I("MRD"); I("ADD", "GD")
    I("ADD", "GT"); I("ST", "B_G", "bank")
    I("LOOP", ("l", "GLIDE"))
    I("SETK", TK0)
    # ---- drift speed for the next field (Hypnos law): glide, |v| < 24 still, 2^-12 .. 2^-3 cycle per field
    C("LDC", "GSH"); I("ST", "GSH")
    I("LD", "H_V"); C("ADDC", "c-512"); I("SHL", 6); I("ST", "VTG")
    I("SUB", "VGS"); I("SRAV", "GSH"); I("ADD", "VGS"); I("ST", "VGS")
    I("SRA", 6); I("ST", "V"); I("OUT", ("o", "VNEG"))
    I("ABS"); C("ADDC", "c-24"); I("OUT", ("o", "SPDZ")); C("MAXC", "c0"); I("ST", "U")
    I("SRA", 3); I("ST", "T1"); I("LD", "U"); I("SRA", 4); I("ADD", "T1"); I("ADD", "U"); I("ST", "LS")
    I("SRA", 6); I("ST", "OCT"); I("SHL", 6); I("RSUB", "LS"); I("SHL", 2); I("EXR"); I("SRA", 9)
    I("SHLV", "OCT"); I("OUT", ("o", "SPD"))
    # ---- globals
    I("LD", "G_S"); C("MULC", "c769"); I("MRD"); I("SRA", 13); I("ST", "EN")
    I("LD", "G_P"); C("MULC", "c429"); I("MRD"); I("SRA", 13); I("ST", "LDP")
    I("LD", "G_Y"); C("ADDC", "c-4096"); C("MULC", "c17"); I("MRD"); I("SRA", 3); C("ADDC", "c8640")
    I("ST", "Y0")
    I("LD", "G_X"); C("ADDC", "c-4096"); C("MULC", "c60"); I("MRD"); I("SRA", 3); I("ST", "XP")
    C("LDC", "AA"); I("ST", "A")
    I("LD", "EN"); I("ST", "E")
    C("LDC", "c0"); I("ST", "ES")

    def pitch():              # P5 = (EX[E & 255] * 5) >> (5 - (E >> 8))
        I("LD", "E"); I("SRA", 8); I("ST", "IE"); I("SHL", 8); I("RSUB", "E"); I("EXR"); C("MULC", "c5"); I("MRD")
        I("ST", "T1"); C("LDC", "c5"); I("SUB", "IE"); I("ST", "T2"); I("LD", "T1"); I("SRAV", "T2")

    pitch(); I("SRA", 1); I("ST", "PN4")
    I("LD", "PN4"); C("MULC", "c316"); I("MRD"); I("SRA", 8); I("RSUB", "Y0"); I("ST", "YH")
    I("LD", "PN4"); I("MULR", "G_C"); I("MRD8"); I("SRA", 5); I("ST", "G1")
    C("LDC", "CA"); I("ST", "T1"); I("LD", "G1"); I("MULR", "T1"); I("MRD8"); I("SRA", 2); I("ST", "GL")
    C("LDC", "OSCSH"); I("ST", "T2"); I("LD", "GL"); I("SRAV", "T2"); I("ADD", "GL"); I("OUT", ("o", "GAIN"))
    # ---- the three tiers, near first (the constants shift into per-tier chains: near ends in slot 2)
    L("TIER")
    pitch(); I("ST", "P5")
    I("SRA", 1); I("ST", "P4"); I("OUT", ("o", "HP5"))
    C("LDC", "STC"); I("SUB", "P5"); I("OUT", ("o", "STP"))
    C("LDC", "RHO"); I("ST", "T1"); I("LD", "P5"); I("MULR", "T1"); I("MRD"); I("SRA", 9); I("ST", "R4")
    C("MULC", "RCF"); I("MRD8"); I("ST", "RC")
    I("LD", "R4"); C("MULC", "EF"); I("MRD8"); I("OUT", ("o", "E"))
    I("LD", "R4"); C("MULC", "KF"); I("MRD8"); I("ST", "K"); I("ST", "B_K", "bank")
    I("LD", "R4"); C("MULC", "LSF"); I("MRD8"); I("ST", "B_LS", "bank")
    I("LD", "R4"); C("MULC", "APF"); I("MRD8"); I("ST", "AP")
    I("LD", "RC"); I("MULR", "RC"); I("MRD8"); C("ADDC", "c-11"); I("ST", "B_RC2", "bank")
    I("LD", "P4"); C("MULC", "c276"); I("MRD8"); I("ADD", "YH"); I("ST", "TOP")
    I("SRAV", "A"); I("ST", "B_TOP", "bank")
    I("LD", "P4"); C("MULC", "c486"); I("MRD8"); I("ADD", "TOP"); I("SRAV", "A"); I("ST", "B_BASE", "bank")
    I("LD", "P4"); C("MULC", "c46"); I("MRD8"); I("ADD", "TOP"); I("ADD", "AP"); I("ST", "B_SPR", "bank")
    I("ADD", "K"); I("SRAV", "A"); I("ST", "B_CUT", "bank")
    # horizontal: x0 = 960 + Xpan (1 + s) / 2, s = 2^(ES / 256); the cell phase at pixel 0
    I("LD", "ES"); I("SRA", 8); I("ST", "IES"); I("SHL", 8); I("RSUB", "ES"); I("EXR"); I("ST", "T1")
    I("LD", "XP"); I("MULR", "T1"); I("MRD8"); I("ST", "T2"); C("LDC", "c7"); I("SUB", "IES"); I("ST", "T3")
    I("LD", "T2"); I("SRAV", "T3"); I("SRA", 1); I("ST", "T1")
    I("LD", "XP"); I("SRA", 1); I("ADD", "T1"); C("ADDC", "c30720"); I("ST", "X0")
    C("LDC", "XPIX0"); I("SUB", "X0"); I("ADD", "P4"); I("ST", "R")
    I("LD", "P5"); I("ST", "Q0")
    for j in range(1, 8):
        I("SHL", 1); I("ST", "Q%d" % j)
    I("ADD", "R")
    for j in range(7, -1, -1):
        I("CSUB", "Q%d" % j)
    I("ST", "R")
    I("KNB", ("k", "theta")); I("SRA", 2); I("MULR", "P5"); I("MRD8"); I("SRA", 8); I("RSUB", "R"); I("ADDN", "P5")
    I("OUT", ("o", "A0"))
    # the distance normaliser: Dv = ((Rc DVA) >> 7) << AA = 2^q (1 + f/32): QQ = q - 8, M = 32 / (1 + f/32)
    I("LD", "RC"); C("MULC", "DVA"); I("MRD"); I("SRA", 7); I("SHLV", "A"); I("NORM"); I("ST", "NQ")
    I("SRA", 5); I("ST", "T1"); C("ADDC", "c-8"); I("OUT", ("o", "QQ"))
    I("LD", "T1"); I("SHL", 5); I("RSUB", "NQ"); C("ADDC", "c256"); I("EXR"); I("OUT", ("o", "M"))
    # widths in sixteenths of the anti-alias width
    I("LD", "P4"); I("SRAV", "A"); I("ST", "PD")
    # the tier's line: Grey = the archivolt (half-width HR, centred GAP + HR outside the opening), Ink = the
    # same band hugging the opening, Lattice = the outline of the wall (half-width WLH, on the edge)
    C("LDC", "WR"); I("ST", "T1"); I("LD", "PD"); I("MULR", "T1"); I("MRD8"); C("MAXC", "c32"); I("SRA", 1)
    I("ST", "HR")
    C("LDC", "WL"); I("ST", "T1"); I("LD", "PD"); I("MULR", "T1"); I("MRD8"); C("MAXC", "c32"); I("SRA", 1)
    I("ST", "WLH")
    I("LD", "PD"); C("MULC", "c13"); I("MRD8"); I("ST", "GAP")
    I("LD", "HR"); C("MULC", "HM0"); I("MRD"); I("ST", "T1"); I("LD", "WLH"); C("MULC", "HM1"); I("MRD")
    I("ADD", "T1"); I("OUT", ("o", "HB"))
    I("LD", "GAP"); C("MULC", "CG"); I("MRD"); I("ST", "T1"); I("LD", "HR"); C("MULC", "CH"); I("MRD")
    I("ADD", "T1"); I("ST", "T1"); C("LDC", "c0"); I("SUB", "T1"); I("OUT", ("o", "CB"))
    # the camera's swell for this tier (Q4, taken off |dx| before the squarer): L4 >> SSH (SSH = the tier's
    # octaves below the near tier), at most the pier's half-width less 1.5 px (hairline piers at white)
    C("LDC", "c128"); I("SUB", "ES"); I("SRA", 8); I("OUT", ("o", "SSH"))
    I("LD", "P4"); I("SRA", 1); I("SUB", "R4"); I("ST", "T1"); C("LDC", "c24"); I("SHLV", "A"); I("RSUB", "T1")
    C("MAXC", "c0"); I("OUT", ("o", "SMAX"))
    # next tier: one depth step further
    I("LD", "E"); I("SUB", "LDP"); I("ST", "E"); I("LD", "ES"); I("SUB", "LDP"); I("ST", "ES")
    I("LOOP", ("l", "TIER"))
    I("END")
    return P, lab


# register file: fixed address for the persistent speed glide state, the rest allocated in order
RF_FIXED = {"VGS": 0}
VGS_INIT = (660 - 512) * 64


def rf_init():
    """the register file's power-up contents: the knob state at the TOML defaults, the speed glide"""
    r = {RF_FIXED["VGS"]: VGS_INIT}
    for tk, v in KNOB_DEF.items():
        r[128 + 16 * tk + 8] = v * 8
        r[128 + 16 * tk + 9] = v
    return r


def assemble():
    P, lab = program()
    rf = dict(RF_FIXED)
    for n, i in BANK.items():
        rf[n] = 128 + i
    rf.update(GLIDED)
    words = []
    for op, x, a in P:
        selc = 0
        if isinstance(a, tuple) and a[0] == "ct":
            av = CTB[a[1]]
            selc = SEL[x]
        else:
            if x == "bank":
                selc = 1
            elif x in TSTX:
                selc = TSTX[x]
            if isinstance(a, tuple):
                kind, n = a
                av = {"k": KNOBS, "o": OUT, "l": lab}[kind][n]
            elif isinstance(a, str):
                if a not in rf:
                    rf[a] = len([v for v in rf.values() if v < 128])
                av = rf[a]
            else:
                av = a
        assert 0 <= av < 256, (op, x, a)
        words.append((OP[op] << 11) | (selc << 8) | av)
    assert len(words) <= 512, len(words)
    return words + [0] * (512 - len(words)), rf


def ex_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def mt_rom():
    return [min(31, int(round(1024.0 / (32 + f)))) for f in range(32)]


def data_rom():
    w = ex_rom() + mt_rom() + [v & 0xFFFF for v in CTW]
    return w + [0] * (512 - len(w))


def sq_rom():
    return [h * h + h for h in range(256)]


def rf_init_vhd():
    r = rf_init()
    items = ", ".join("%d => to_signed(%d, 24)" % (k, v) for k, v in sorted(r.items()))
    return ("    type t_aq_rf is array (0 to 255) of signed(23 downto 0);\n"
            "    constant C_AQ_RFINIT : t_aq_rf := (%s, others => (others => '0'));" % items)


def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of unsigned(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['to_unsigned(%d, %d)' % (v, width) for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    prog, rf = assemble()
    parts = ["-- Generated by aqueduct.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package aqueduct_pkg is",
             "    constant C_AQ_VGS_ADDR : integer := %d;" % RF_FIXED["VGS"],
             "    constant C_AQ_VGS_INIT : integer := %d;" % VGS_INIT,
             "    constant C_AQ_TK0 : integer := %d;" % TK0,
             rf_init_vhd(),
             vhd_table("C_AQ_SQ", 256, 16, sq_rom()),
             vhd_table("C_AQ_ROM", 512, 16, data_rom()),
             vhd_table("C_AQ_PROG", 512, 16, prog),
             "end package aqueduct_pkg;", ""]
    with open(os.path.join(HERE, "aqueduct_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
