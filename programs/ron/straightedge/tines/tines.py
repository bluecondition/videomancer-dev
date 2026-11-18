#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
TINES v0.1.1 (Straightedge) - build hook.  Writes tines_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  The integer model (.straightedge_work/rb_tines/imodel.py) imports this file and
executes the same microcode, so the per-field constants of model and RTL are identical by construction.

Package contents:
  C_TN_ROM   512 x 16   0 .. 255   round(32768 * 2^(i/256))        (pitch, drift speed)
                        256 .. 263 the edge normaliser's reciprocal mantissa (8 / 1.f, by the top 3 fraction bits)
                        264 ..     CT, the field engine's constants (some chosen by video mode / Glide)
  C_TN_PROG  1024 x 16  the field engine's program: op(15:11) x(10:8) a(7:0)
  C_TN_RFINIT           the register file's power-up contents (knob state at the TOML defaults)

The field engine (a 32-bit accumulator, a 256 x 32 register file, one serial 32 x 16 multiplier, a divider):
  END LDC LD ST ADD SUB RSUB ADDC KNB SRA SHL SRAV SHLV MULR MULC MRD EXR MAXC ABS NORM OUT LOOP DIV TST LDF MINC
  SETK MAXR MINR
  x: LDC / ADDC / MULC / MAXC / MINC: the CT selector (0 none, 1 video mode, 2 Glide);  LD / ST: 1 = the knob
  bank (128 + 16 tk + a);  MRD: the product's right shift (0, 4, 12);  TST: 0 set, 1 or-negative, 2 and.
  video mode = 0 1080-line, 1 720p, 2 SD.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

OPS = ["END", "LDC", "LD", "ST", "ADD", "SUB", "RSUB", "ADDC", "KNB", "SRA", "SHL", "SRAV", "SHLV", "MULR",
       "MULC", "MRD", "EXR", "MAXC", "MAXR", "MINR", "ABS", "NORM", "OUT", "LOOP", "DIV", "TST", "LDF", "MINC",
       "SETK"]
OP = {n: i for i, n in enumerate(OPS)}
SEL = {None: 0, "mode": 1, "glide": 2}
SELN = {0: 1, 1: 3, 2: 2}
MRDX = {0: 0, 4: 1, 12: 2}
TSTX = {"set": 0, "orneg": 1, "and": 2}
CT0 = 264                                     # ROM index of the first CT word
# knob channels (loop index tk): 6 K5 Drift (hold only), 5 K1 X, 4 K2 Y, 3 K3 Scale, 2 K4 Skew, 1 K6 Camera, 0 P12
KNOB_REG = {6: 4, 5: 0, 4: 1, 3: 2, 2: 3, 1: 5, 0: 7}         # registers_in index by channel
KNOB_DEF = {6: 700, 5: 512, 4: 512, 3: 640, 2: 512, 1: 1023, 0: 800}
GLIDED = {"G_X": 128 + 16 * 5 + 8, "G_Y": 128 + 16 * 4 + 8, "G_S": 128 + 16 * 3 + 8, "G_K": 128 + 16 * 2 + 8,
          "G_C": 128 + 16 * 1 + 8, "G_P": 128 + 8, "H_V": 128 + 16 * 6 + 9}
BANK = {"B_G": 8, "B_H": 9}
RF_FIXED = {"VGS": 0, "TH_T": 1, "TH_B": 2, "TH_L": 3, "TH_R": 4}
# the line engine reads these 13 words (register file 33 .. 45) after every line, in this order
LINE_RF = ["L_SLT", "L_SRT", "L_SLB", "L_SRB", "L_DPSI", "L_PHT0", "L_PHB0", "L_ALL0", "L_ARL0", "L_ALR0", "L_ARR0",
           "L_XCT0", "L_XCB0"]
RF_FIXED.update({n: 33 + i for i, n in enumerate(LINE_RF)})
VGS_INIT = (KNOB_DEF[6] - 512) * 64
KNB_IN = {"knob": 8, "ys": 1}
# the engine's outputs: name -> (index, width); the pixel / line path reads these registers
# (the ..P outputs are the line accumulators' values one line BEFORE line 0: the line engine's first pass steps
# them onto line 0)
OUTS = ["DPHI", "DLL", "DRL", "DLR", "DRR", "PQ5", "ECT", "ECB", "ECL", "ECR", "THP", "THB", "THT",
        "ALT0P", "ART0P", "ALB0P", "ARB0P", "PSL0P", "PSR0P", "YCL0P", "YCR0P"]
OUT = {n: i for i, n in enumerate(OUTS)}

C1 = (1 << 40) // 1080                        # the reach divider's numerator: tv = (C1 << 13) / D   (Q28 per vpx)
MTAB = [8, 7, 6, 6, 5, 5, 4, 4]               # round(8 / (1 + (f + 1/2) / 8))

CT = {
    "GSH":  ("glide", [0, 3]),
    "GL01": ("glide", [0, 1]),
    "XS":   ("mode", [2, 3, 4]),                  # h (half virtual px) per pixel
    "X0Q":  ("mode", [14, 21, 988]),              # pixel 0's centre, Q2 virtual px (screen column 3; SD + 240)
    "X1Q":  ("mode", [7666, 7659, 6692]),         # 1920 - that
    "X0Q5": ("mode", [112, 168, 7904]),           # pixel 0's centre, Q5
    "THD":  ("mode", [128, 85, 64]),              # Fine line: virtual px Q5 -> sixteenths of a pixel (x / 256)
}
CONSTS = []                                   # the plain constants, collected from the program


def build_ct():
    words, base = [], {}
    for v in CONSTS:
        assert -32768 <= v < 32768
        base["c%d" % v] = len(words)
        words.append(v)
    for name, (sel, vals) in CT.items():
        assert len(vals) == SELN[SEL[sel]], name
        base[name] = len(words)
        words.extend(vals)
    assert CT0 + len(words) <= 512
    return words, base


def program():
    P = []
    lab = {}

    def I(op, a=0, x=None):
        P.append((op, x, a))

    def C(op, name):
        if isinstance(name, int):
            if name not in CONSTS:
                CONSTS.append(name)
            name = "c%d" % name
        sel = CT[name][0] if name in CT else None
        I(op, ("ct", name), sel)

    def L(name):
        lab[name] = len(P)

    def MRD(sh=0):
        I("MRD", 0, ("mrd", sh))

    # ---- the knobs: a +-1 hold (the ends always get through), then Glide On: 1/16 of the gap per field, never
    # slower than one count, landing exactly; Glide Off: the held value.  State in the register file (G at bank
    # item 8, Q3; H at item 9).  Channel 6 (K5 Drift) only uses the hold.
    I("SETK", 6)
    L("GLIDE")
    I("LD", "B_G", "bank"); I("ST", "GT"); I("LD", "B_H", "bank"); I("ST", "HT")
    I("KNB", ("k", "knob")); I("ST", "GR")
    I("SUB", "HT"); I("ABS"); C("ADDC", -2); I("TST", 0, "set")
    I("LD", "GR"); C("ADDC", -1); I("TST", 0, "orneg")
    C("LDC", 1022); I("SUB", "GR"); I("TST", 0, "orneg")
    I("LD", "HT"); I("LDF", "GR"); I("ST", "B_H", "bank")
    I("SHL", 3); I("SUB", "GT"); I("ST", "GD")
    C("ADDC", 128); I("TST", 0, "set")
    C("LDC", 127); I("SUB", "GD"); I("TST", 0, "and")
    I("LD", "GD"); I("SRA", 4); I("ST", "T1")
    I("LD", "GD"); C("MAXC", -8); C("MINC", 8); I("ST", "T2")
    I("LD", "T1"); I("LDF", "T2")
    I("SUB", "GD"); C("MULC", "GL01"); MRD(); I("ADD", "GD")
    I("ADD", "GT"); I("ST", "B_G", "bank")
    I("LOOP", ("l", "GLIDE"))
    # ---- Drift (the Hypnos law): the speed glides 1/8 of the gap (Glide On); |v| < 24 still; 2^-12 .. 2^-4 pitch
    # per field (Q18).  The four rows' phases advance by 1, -3/4, 21/32, -1/2 of it and wrap one pitch.
    C("LDC", "GSH"); I("ST", "GSH")
    I("LD", "H_V"); C("ADDC", -512); I("SHL", 6); I("ST", "VTG")
    I("SUB", "VGS"); I("SRAV", "GSH"); I("ADD", "VGS"); I("ST", "VGS")
    I("SRA", 6); I("ST", "V")
    I("ABS"); C("ADDC", -24); I("ST", "U0"); C("MAXC", 0); I("ST", "U")
    I("SRA", 5); I("ADD", "U"); I("ST", "LS")
    I("SRA", 6); I("ST", "OC"); I("SHL", 6); I("RSUB", "LS"); I("SHL", 2); I("EXR"); I("SRA", 9)
    I("SHLV", "OC"); I("ST", "SP")
    # still: speed 0
    I("LD", "U0"); I("TST", 0, "set"); C("LDC", 0); I("LDF", "SP"); I("ST", "SP")
    # signed speed
    I("LD", "V"); I("TST", 0, "set"); C("LDC", 0); I("SUB", "SP"); I("LDF", "SP"); I("ST", "SV")
    I("ADD", "TH_T"); I("ST", "TH_T")
    I("LD", "SV"); I("SRA", 2); I("RSUB", "SV"); I("RSUB", "TH_B"); I("ST", "TH_B")
    I("LD", "SV"); I("SRA", 1); I("ST", "T1"); I("RSUB", "TH_R"); I("ST", "TH_R")
    I("LD", "SV"); I("SRA", 3); I("ADD", "T1"); I("ST", "T1"); I("LD", "SV"); I("SRA", 5); I("ADD", "T1")
    I("ADD", "TH_L"); I("ST", "TH_L")
    # ---- Scale: oct = 0 .. 999 (1/256 octave);  P = 32 * 2^(oct/256) virtual px (Q5);  1 / P = M 2^(-20 - sh)
    I("LD", "G_S"); C("MULC", 125); MRD(); I("SRA", 10); I("ST", "OCT")
    I("SRA", 8); I("ST", "IO"); I("SHL", 8); I("RSUB", "OCT"); I("EXR"); I("SHLV", "IO"); I("SRA", 5)
    I("ST", "PQ"); I("OUT", ("o", "PQ5"))
    C("LDC", 0); I("SUB", "OCT"); I("ST", "NE"); I("SRA", 8); I("ST", "T1"); I("SHL", 8); I("RSUB", "NE"); I("EXR")
    I("ST", "M"); C("LDC", 0); I("SUB", "T1"); I("ST", "SH")
    # the cell phase per pixel (Q28) = (M xs) << (7 - sh); per field line = (M ys) >> sh
    I("LD", "M"); C("MULC", "XS"); MRD(); I("ST", "T1"); C("LDC", 7); I("SUB", "SH"); I("ST", "T2")
    I("LD", "T1"); I("SHLV", "T2"); I("OUT", ("o", "DPHI")); I("ST", "DPHI")
    I("KNB", ("k", "ys")); I("ST", "YS")
    I("LD", "M"); I("MULR", "YS"); MRD(); I("SRAV", "SH"); I("ST", "L_DPSI"); I("ST", "DPSI")
    # the phase of pixel 0's centre / line 0's centre from the screen's corner
    I("LD", "M"); C("MULC", "X0Q"); MRD(); I("SHL", 6); I("SRAV", "SH"); I("ST", "PX0")
    I("LD", "DPSI"); I("SRA", 1); I("ST", "PY0")
    # ---- Reach and balance: R = 1080 rq fq / 2^25, rq = max(P12 + 8, 512) (Q13), fq = 4096 -+ 7/8 (Y - 4096);
    # t per virtual px (Q28) = (C1 << 13) / max(rq fq, 2^21)   (reach >= 67.5)
    I("LD", "G_P"); C("ADDC", 8); C("MAXC", 512); I("ST", "RQ")
    I("LD", "G_Y"); C("ADDC", -4096); I("ST", "T1"); I("SRA", 3); I("RSUB", "T1"); I("ST", "B78")
    C("LDC", 1); I("SHL", 21); I("ST", "K21")
    C("LDC", 4096); I("SUB", "B78"); I("MULR", "RQ"); MRD(); I("MAXR", "K21"); I("ST", "DT")
    C("LDC", 4096); I("ADD", "B78"); I("MULR", "RQ"); MRD(); I("MAXR", "K21"); I("ST", "DB")
    C("LDC", C1 >> 15); I("SHL", 15); C("ADDC", C1 & 32767); I("ST", "C1")
    I("DIV", "DT"); I("ST", "TVT")
    I("LD", "C1"); I("DIV", "DB"); I("ST", "TVB")
    # ---- Skew: the apex at a of the base (Q12), 1/2 inside +-12 counts of the centre, 0 and 1 at the ends
    I("LD", "G_K"); C("ADDC", -4096); I("ST", "T3"); I("ABS"); C("ADDC", -96); C("MAXC", 0); I("ST", "T1")
    I("SHL", 5); I("ADD", "T1"); I("SRA", 6); C("MINC", 2048); I("ST", "T2")
    C("LDC", 2048); I("SUB", "T2"); I("ST", "T1"); C("LDC", 2048); I("ADD", "T2"); I("ST", "T2")
    I("LD", "T3"); I("TST", 0, "set"); I("LD", "T1"); I("LDF", "T2"); I("ST", "A12")
    # the edges' insets per virtual px of depth (Q28): top / left rows a and 1 - a, bottom / right 1 - a and a
    I("LD", "TVT"); I("MULR", "A12"); MRD(12); I("ST", "GLT"); I("RSUB", "TVT"); I("ST", "GRT")
    I("LD", "TVB"); I("MULR", "A12"); MRD(12); I("ST", "GRB"); I("RSUB", "TVB"); I("ST", "GLB")
    # ---- top and bottom rows: the insets per field line (Q24 in the register file, Q20 out) and on line 0
    for g, s in (("GLT", "SLT"), ("GRT", "SRT"), ("GLB", "SLB"), ("GRB", "SRB")):
        I("LD", g); I("MULR", "YS"); MRD(12); I("ST", s); I("SRA", 4); I("ST", "L_" + s)
    I("LD", "L_SLT"); I("SRA", 1); I("SUB", "L_SLT"); I("OUT", ("o", "ALT0P"))
    I("LD", "L_SRT"); I("SRA", 1); I("SUB", "L_SRT"); I("OUT", ("o", "ART0P"))
    for g, s, o in (("GLB", "L_SLB", "ALB0P"), ("GRB", "L_SRB", "ARB0P")):
        I("LD", s); I("SRA", 1); I("ST", "T1"); I("LD", g); C("MULC", 1080); MRD(4); I("SRA", 4); I("SUB", "T1")
        I("ADD", s); I("OUT", ("o", o))
    # ---- left and right rows (reach x 16/9): the insets per pixel (Q24 in the register file, Q20 out) and at
    # pixel 0
    for g, d, a0, xq in (("GLT", "DLL", "ALL0", "X0Q"), ("GRT", "DRL", "ARL0", "X0Q"),
                         ("GLB", "DLR", "ALR0", "X1Q"), ("GRB", "DRR", "ARR0", "X1Q")):
        I("LD", g); I("SRA", 1); I("ST", "T1"); I("LD", g); I("SRA", 4); I("ADD", "T1"); I("ST", "GS")
        C("MULC", "XS"); MRD(); I("SRA", 5); I("ST", d); I("SRA", 4); I("OUT", ("o", d))
        I("LD", "GS"); C("MULC", xq); MRD(4); I("SRA", 6); I("ST", "L_" + a0)
    # ---- the rows' phases at pixel 0 / line 0: X slides the top (and left) row +-3/8 pitch; the bottom and
    # right rows sit half a pitch off; each row has its own drift phase (Q18)
    I("LD", "G_X"); C("ADDC", -4096); I("ST", "T1"); I("SHL", 14); I("ST", "T2"); I("LD", "T1"); I("SHL", 13)
    I("ADD", "T2"); I("ST", "XK")
    C("LDC", 1); I("SHL", 27); I("ST", "HALF")
    I("LD", "TH_T"); I("SHL", 10); I("ST", "T1"); I("LD", "PX0"); I("SUB", "XK"); I("SUB", "T1")
    I("ST", "L_PHT0"); I("ST", "PT0")
    I("LD", "TH_B"); I("SHL", 10); I("ST", "T1"); I("LD", "PX0"); I("ADD", "HALF"); I("SUB", "T1")
    I("ST", "L_PHB0"); I("ST", "PB0")
    I("LD", "TH_L"); I("SHL", 10); I("ST", "T1"); I("LD", "PY0"); I("SUB", "XK"); I("SUB", "T1")
    I("ST", "PL0"); I("SUB", "DPSI"); I("OUT", ("o", "PSL0P"))
    I("LD", "TH_R"); I("SHL", 10); I("ST", "T1"); I("LD", "PY0"); I("ADD", "HALF"); I("SUB", "T1")
    I("ST", "PR0"); I("SUB", "DPSI"); I("OUT", ("o", "PSR0P"))
    # ---- the camera anchors: the first cell's quarter point (Q5 virtual px) = centre of pixel 0 / line 0
    # - frac(phase) P + P / 4
    I("LD", "PQ"); I("SRA", 2); I("ST", "PQ4")
    I("LD", "YS"); I("SRA", 4); I("ST", "Y0Q5")
    for ph, o, base in (("PT0", "L_XCT0", None), ("PB0", "L_XCB0", None), ("PL0", "YCL0P", "Y0Q5"), ("PR0", "YCR0P", "Y0Q5")):
        I("LD", ph); I("SRA", 16); I("ST", "T1"); I("SRA", 12); I("SHL", 12); I("RSUB", "T1")
        I("MULR", "PQ"); MRD(12); I("RSUB", "PQ4")
        if base is None:
            C("ADDC", "X0Q5"); I("ST", o)
        else:                      # the first line pass adds a pitch if the strip phase wraps: take it off here
            I("ADD", base); I("ST", "T2")
            I("LD", ph); I("SRA", 28); I("SHL", 28); I("RSUB", ph); I("SUB", "DPSI"); I("TST", 0, "set")
            I("LD", "T2"); I("SUB", "PQ"); I("LDF", "T2"); C("ADDC", -896); I("OUT", ("o", o))   # (- 28 lines: the grid's row 0)
    # ---- the edges' normalisers: w = cycles (Q16) per pixel of distance ~ max + 3/8 min of the edge function's
    # steps per pixel and per line = 2^q (1 + f/8);  shift = q - 4 (3 .. 8), mantissa m = 8 / (1 + f/8) (4 .. 8)
    I("LD", "DPHI"); I("SRA", 12); I("ST", "GX")
    I("LD", "DPSI"); I("SRA", 12); I("ST", "GY")
    for o, lat, sl, sr in (("ECT", "GX", "SLT", "SRT"), ("ECB", "GX", "SLB", "SRB"),
                           ("ECL", "GY", "DLL", "DRL"), ("ECR", "GY", "DLR", "DRR")):
        for k, s in enumerate((sl, sr)):
            I("LD", s); I("SRA", 8); I("ST", "T1"); I("MAXR", lat); I("ST", "T2")
            I("LD", "T1"); I("MINR", lat); I("ST", "T3"); I("SRA", 2); I("ST", "T1"); I("LD", "T3"); I("SRA", 3)
            I("ADD", "T1"); I("ADD", "T2"); I("NORM"); I("ST", "T1")
            I("SRA", 3); I("ST", "T2"); I("SHL", 3); I("RSUB", "T1"); C("ADDC", 256); I("EXR"); C("ADDC", -4)
            I("ST", "T3")
            I("LD", "T2"); C("ADDC", -7); C("MAXC", 0); C("MINC", 5); I("SHL", 3); I("ADD", "T3")
            if k == 0:
                I("SHL", 6); I("ST", "EC")
            else:
                I("ADD", "EC"); I("OUT", ("o", o))
    # ---- Fine: the line's width, P / 20 clamped to 2.5 .. 6.5 virtual px, in sixteenths of a pixel
    I("LD", "PQ"); C("MULC", 13); MRD(); I("SRA", 8); C("MAXC", 80); C("MINC", 208); C("MULC", "THD"); MRD()
    I("SRA", 8); I("OUT", ("o", "THP"))
    # ---- Camera: the gain g = K6 x 214 (0.6 at 25 %, 0.75 at 50 %, 1 at 100 %) sits in front of the films' fixed
    # thresholds, so it is applied to the thresholds instead: light film 327680 / g, dark film 524288 / g of the
    # filtered luma (Q13, white 3504): 0.73 / never at 25 %, 0.58 / 0.93 at 50 %, 0.44 / 0.70 at 100 %
    I("LD", "G_C"); I("SRA", 3); I("ST", "T1"); C("ADDC", -256); I("TST", 0, "set"); C("MAXC", 0)
    C("MULC", 29); MRD(); I("SRA", 8); C("ADDC", 128); C("MINC", 214); I("ST", "T2")
    I("LD", "T1"); I("SRA", 1); I("LDF", "T2"); C("MAXC", 1); I("ST", "T2")
    C("LDC", 40); I("DIV", "T2"); C("MINC", 8191); I("OUT", ("o", "THB"))
    C("LDC", 64); I("DIV", "T2"); C("MINC", 8191); I("OUT", ("o", "THT"))
    I("END")
    return P, lab


def rf_init():
    r = {RF_FIXED["VGS"]: VGS_INIT}
    for tk, v in KNOB_DEF.items():
        r[128 + 16 * tk + 8] = v * 8
        r[128 + 16 * tk + 9] = v
    return r


def assemble():
    del CONSTS[:]
    P, lab = program()
    ctw, ctb = build_ct()
    rf = dict(RF_FIXED)
    for n, i in BANK.items():
        rf[n] = 128 + i
    rf.update(GLIDED)
    words = []
    for op, x, a in P:
        selc = 0
        if isinstance(x, tuple) and x[0] == "mrd":
            selc = MRDX[x[1]]; av = 0
        elif isinstance(a, tuple) and a[0] == "ct":
            av = ctb[a[1]]
            selc = SEL[x]
        else:
            if x == "bank":
                selc = 1
            elif x in TSTX:
                selc = TSTX[x]
            if isinstance(a, tuple):
                kind, n = a
                av = {"k": KNB_IN, "o": OUT, "l": lab}[kind][n]
            elif isinstance(a, str):
                if a not in rf:
                    rf[a] = min(i for i in range(5, 128) if i not in rf.values())
                av = rf[a]
            else:
                av = a
        assert 0 <= av < 256, (op, x, a)
        words.append((OP[op] << 11) | (selc << 8) | av)
    assert len(words) <= 1024, len(words)
    return words + [0] * (1024 - len(words)), rf, ctw, len(P)


def ex_rom():
    return [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]


def data_rom(ctw):
    w = ex_rom() + MTAB + [v & 0xFFFF for v in ctw]
    assert len(MTAB) == CT0 - 256
    return w + [0] * (512 - len(w))


def rf_init_vhd():
    r = rf_init()
    items = ", ".join("%d => to_signed(%d, 32)" % (k, v) for k, v in sorted(r.items()))
    return ("    type t_tn_rf is array (0 to 255) of signed(31 downto 0);\n"
            "    constant C_TN_RFINIT : t_tn_rf := (%s, others => (others => '0'));" % items)


def vhd_table(name, depth, width, vals):
    lines = ["    type %s is array (0 to %d) of unsigned(%d downto 0);" % ("t_" + name.lower(), depth - 1, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    items = ['to_unsigned(%d, %d)' % (v, width) for v in vals]
    for i in range(0, len(items), 8):
        lines.append("        " + ", ".join(items[i:i + 8]) + ("," if i + 8 < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    prog, rf, ctw, n = assemble()
    parts = ["-- Generated by tines.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package tines_pkg is",
             "\n".join("    constant C_TN_O_%s : integer := %d;" % (n, i) for i, n in enumerate(OUTS)),
             rf_init_vhd(),
             vhd_table("C_TN_ROM", 512, 16, data_rom(ctw)),
             vhd_table("C_TN_PROG", 1024, 16, prog),
             "end package tines_pkg;", ""]
    with open(os.path.join(HERE, "tines_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
