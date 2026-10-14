#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""
GAMUT micro-assembler: the vblank (frame) and per-line programs for the
accumulator engine in gamut_engine.vhd, emitted as gamut_prog_pkg.vhd.

Engine model: one 20-bit signed accumulator; operands come from a 128-entry
register FILE (EBR, shared by both engines, index 0..127) or from ~40
EXTERNAL sources (index 128+, a small mux in the top); stores go to the file
and, for the frame engine, straight to the named FF registers the pixel path
reads.  The line engine computes the NEXT line into the file during the
current line; a copy sequencer moves the per-line values into the named FFs
at each hsync (see COPY_LIST).

Pipeline: fetch -> A -> D1 (file address / ext select, both registered by
the top) -> D2 (operand shift) -> X (execute).  Distance >= 3 between a store
and a read of the same file register, 4 for a ROM read after its address
store (the assembler pads with NOPs); taken jumps cost 5 bubbles; MUL stalls
~18 cycles and needs 2 instructions before an MRES* read.

Word (24 bits): op[23:19] sh[18:15] src[14:7] dst[6:0]
  jumps: addr = word[9:0]; immediates: imm = word[14:0] sign-extended.
Shift field (on the source operand): 0..13 -> >>0..13, 14 -> <<4, 15 -> <<8
(<<1 is done with ADD ACC).
"""
import sys

OPS = ["NOP", "LD", "ADD", "SUB", "MIN", "MAX", "LDI", "ADDI", "SUBI", "MINI", "MAXI",
       "ST", "MUL", "JMP", "JNEG", "JPOS", "JZ", "ABS", "LOG2", "END", "JNZ"]
OPC = {n: i for i, n in enumerate(OPS)}
SHCODE = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 11, 12: 12, 13: 13, -4: 14, -8: 15}

# register file entries (index 0..127); values with a named FF twin are marked *
FILE = """T0 T1 T2 T3 K1G SHELF* SHELFPREV CARDH* CARDW GAP PITCH FOCAL COLX0* COLX1* COLX2* R* RH* COUNT FIRST
CURSOR CURROW CURCOL* NROWS SPOS SVEL DPOS DVEL P P2 S HX0 HX1 HY0 HY1 EX0* EX1* EY0* EY1* SH FADE INSETLIFT
RINGG RINGT RG RGT R2L R2H RG2 RGT2 RS2L RS2H SHSL* SHSH* SHIFT RSTART ROWTOP0 HBASE* HN* SCAN DOMU DOMV TINTU* TINTV*
LY* LROW LROWTOP LDIST LINSET* LCRI* LDIM* LY0* LY1* LY0F* LW* KL* KSL* KH* KG* KGT* KSH* LEX0* LEX1* LEX2*
LBASE0* LBASE1* LBASE2* LN0* LN1* LN2* LROWOK""".split()
# named-only destinations (no file twin), index 96..
NAMED = """DENS B1 B2 RLT2 RHLT2 SHL SHH SHG RINGA HW CX2 CY2 ROMADDR PALADDRF
PALADDR SQADDR X1M10 X1M11 X1M12""".split()
# external sources, index 128..
EXT = """ACC MRES0 MRES4 MRES8 MRES10 MRESFRAC W H K1 K3 K4 K6 P12 GLIDE LIGHT KH KW KG COLS COLXCUR
SHFIRST SHCOUNT POW2SHSL POW2SHSH ROMROLE ROMU ROMV PALBASE PALN SQ PIXELY YFIRST CX CY INSETPX LIFT SL OYL OYH""".split()

FILE_NAMES = [n.rstrip("*") for n in FILE]
TWINS = [n.rstrip("*") for n in FILE if n.endswith("*")]
assert len(FILE_NAMES) <= 96 and len(NAMED) <= 32 and len(EXT) <= 128
SRC = {n: i for i, n in enumerate(FILE_NAMES)}
SRC.update({n: 128 + i for i, n in enumerate(EXT)})
DST = {n: i for i, n in enumerate(FILE_NAMES)}
DST.update({n: 96 + i for i, n in enumerate(NAMED)})
# per-line values copied file -> named FF at each hsync, in this order
COPY_LIST = """LY LINSET LCRI LDIM LY0 LY1 LY0F LW KL KSL KH KG KGT KSH LEX0 LEX1 LEX2 LBASE0 LBASE1 LBASE2 LN0 LN1 LN2""".split()
RF_INIT = {"TINTU": 512, "TINTV": 512, "SHELFPREV": 9, "K1G": 300, "HN": 1, "LN0": 1, "LN1": 1, "LN2": 1}

class Asm:
    def __init__(self, name, base=0):
        self.name = name
        self.base = base
        self.code = []
        self.labels = {}
        self.recent = []            # names stored by the last two emitted instructions
    def _emit(self, op, sh=0, src=0, dst=0, imm=0, ref=None, stored=None):
        self.code.append([op, sh, src, dst, imm, ref])
        self.recent = (self.recent + [stored])[-2:]
    def nop(self): self._emit("NOP")
    def _rd(self, name, op, shift):
        if name in FILE_NAMES:
            while name in self.recent:
                self.nop()
        if name.startswith("MRES"):
            while any(c[0] == "MUL" for c in self.code[-2:]):
                self.nop()
        self._emit(op, SHCODE[shift], SRC[name])
    def ld(self, name, shift=0):  self._rd(name, "LD", shift)
    def add(self, name, shift=0): self._rd(name, "ADD", shift)
    def sub(self, name, shift=0): self._rd(name, "SUB", shift)
    def min(self, name, shift=0): self._rd(name, "MIN", shift)
    def max(self, name, shift=0): self._rd(name, "MAX", shift)
    def ldi(self, v):  self._emit("LDI", imm=v)
    def addi(self, v): self._emit("ADDI", imm=v)
    def subi(self, v): self._emit("SUBI", imm=v)
    def mini(self, v): self._emit("MINI", imm=v)
    def maxi(self, v): self._emit("MAXI", imm=v)
    def shr(self, k):  self.ld("ACC", k)
    def shl(self, k):
        if k == 1: self.add("ACC")
        else: self.ld("ACC", -k)
    def st(self, name): self._emit("ST", dst=DST[name], stored=name)
    def mul(self, name): self._rd(name, "MUL", 0)
    def abs(self): self._emit("ABS")
    def log2(self): self._emit("LOG2")
    def end(self): self._emit("END")
    def label(self, name): self.labels[name] = self.base + len(self.code)
    def _j(self, op, lab): self._emit(op, ref=lab)
    def jmp(self, lab):  self._j("JMP", lab)
    def jneg(self, lab): self._j("JNEG", lab)
    def jpos(self, lab): self._j("JPOS", lab)
    def jz(self, lab):   self._j("JZ", lab)
    def jnz(self, lab):  self._j("JNZ", lab)
    def words(self):
        out = []
        for op, sh, src, dst, imm, ref in self.code:
            w = OPC[op] << 19 | (sh & 15) << 15
            if ref is not None:
                w |= self.labels[ref] & 0x3FF
            elif op.endswith("I"):
                w |= imm & 0x7FFF
            else:
                w |= (src & 255) << 7 | (dst & 127)
            out.append(w)
        return out

# ----------------------------------------------------------------------------- frame program
def frame_program():
    a = Asm("frame")
    a.ld("K1"); a.sub("K1G"); a.shr(2); a.add("K1G"); a.st("K1G")
    a.ld("K4"); a.subi(4); a.jneg("dens_idx")
    a.ld("K4", 8); a.st("DENS"); a.jmp("dens_done")
    a.label("dens_idx"); a.ld("K4"); a.st("DENS")
    a.label("dens_done")
    a.ld("K3"); a.subi(14); a.jneg("shelf_idx")
    a.ldi(14); a.st("T0"); a.ld("K3"); a.mul("T0"); a.ld("MRES10"); a.st("SHELF"); a.jmp("shelf_done")
    a.label("shelf_idx"); a.ld("K3"); a.st("SHELF")
    a.label("shelf_done")
    a.ld("H"); a.mul("KH"); a.ld("MRES8"); a.st("CARDH")
    a.ld("W"); a.mul("KW"); a.ld("MRES8"); a.st("CARDW")
    a.ld("H"); a.mul("KG"); a.ld("MRES8"); a.st("GAP")
    a.ld("CARDH"); a.add("GAP"); a.st("PITCH")
    a.ld("H", 1); a.sub("CARDH", 1); a.st("FOCAL")
    a.ld("CARDW"); a.add("GAP"); a.st("T1")
    a.ld("COLS"); a.subi(1); a.st("T2")
    a.ld("T1"); a.mul("T2"); a.ld("MRES0"); a.add("CARDW"); a.st("T3")
    a.ld("W"); a.sub("T3"); a.shr(1); a.add("SHIFT"); a.st("COLX0")
    a.add("T1"); a.st("COLX1")
    a.add("T1"); a.st("COLX2")
    a.ldi(4095); a.st("B1"); a.st("B2")
    a.ld("COLS"); a.subi(2); a.jneg("b_done")
    a.ld("COLX1"); a.sub("GAP", 1); a.st("B1")
    a.ld("COLS"); a.subi(3); a.jneg("b_done")
    a.ld("COLX2"); a.sub("GAP", 1); a.st("B2")
    a.label("b_done")
    a.ld("K6"); a.mul("CARDH"); a.ld("MRES10", 1); a.mini(255); a.st("R")
    for c in range(3):
        a.ld("COLX%d" % c); a.add("CARDW"); a.subi(1); a.st("X1M1%d" % c)
    a.ld("SHFIRST"); a.st("FIRST"); a.ld("SHCOUNT"); a.st("COUNT")
    a.ld("SHELF"); a.sub("SHELFPREV"); a.jz("shelf_same"); a.jneg("shelf_down")
    a.ld("W", 4); a.add("W", 5); a.st("SHIFT"); a.jmp("shelf_upd")
    a.label("shelf_down"); a.ldi(0); a.sub("W", 4); a.sub("W", 5); a.st("SHIFT")
    a.label("shelf_upd"); a.ld("SHELF"); a.st("SHELFPREV")
    a.label("shelf_same")
    a.ld("K1G"); a.mul("COUNT"); a.ld("MRES10"); a.sub("CURSOR"); a.jz("cur_done"); a.jneg("cur_dn")
    a.ld("MRESFRAC"); a.subi(96); a.jneg("cur_done"); a.ld("MRES10"); a.st("CURSOR"); a.jmp("cur_done")
    a.label("cur_dn"); a.ld("MRESFRAC"); a.subi(928); a.jpos("cur_done"); a.ld("MRES10"); a.st("CURSOR")
    a.label("cur_done")
    a.ld("CURSOR"); a.sub("COUNT"); a.jneg("cur_ok"); a.ld("COUNT"); a.subi(1); a.st("CURSOR")
    a.label("cur_ok")
    a.ld("COLS"); a.subi(2); a.jneg("cols1"); a.jz("cols2")
    a.ldi(171); a.st("T0"); a.ld("CURSOR"); a.mul("T0"); a.ld("MRES0", 9); a.st("CURROW")
    a.ld("COUNT"); a.addi(2); a.mul("T0"); a.ld("MRES0", 9); a.st("NROWS")
    a.ld("CURROW"); a.shl(1); a.add("CURROW"); a.st("T1"); a.ld("CURSOR"); a.sub("T1"); a.st("CURCOL"); a.jmp("cols_done")
    a.label("cols2"); a.ld("CURSOR", 1); a.st("CURROW"); a.shl(1); a.st("T1"); a.ld("CURSOR"); a.sub("T1"); a.st("CURCOL")
    a.ld("COUNT"); a.addi(1); a.shr(1); a.st("NROWS"); a.jmp("cols_done")
    a.label("cols1"); a.ld("CURSOR"); a.st("CURROW"); a.ldi(0); a.st("CURCOL"); a.ld("COUNT"); a.st("NROWS")
    a.label("cols_done")
    a.ld("GLIDE"); a.jz("scroll_snap")
    a.ld("CURROW", -8); a.sub("SPOS"); a.shr(5); a.add("SVEL"); a.sub("SVEL", 2); a.st("SVEL")
    a.ld("SPOS"); a.add("SVEL"); a.st("SPOS"); a.jmp("scroll_done")
    a.label("scroll_snap"); a.ld("CURROW", -8); a.st("SPOS"); a.ldi(0); a.st("SVEL")
    a.label("scroll_done")
    a.ld("P12"); a.subi(64); a.maxi(0); a.mini(895); a.st("T0"); a.add("T0", 3); a.add("T0", 6); a.mini(1023); a.st("T1")
    a.ld("GLIDE"); a.jz("dive_snap")
    a.ld("T1"); a.sub("DPOS"); a.shr(4); a.add("DVEL"); a.sub("DVEL", 2); a.st("DVEL")
    a.ld("DPOS"); a.add("DVEL"); a.maxi(0); a.mini(1023); a.st("DPOS"); a.jmp("dive_done")
    a.label("dive_snap"); a.ld("T1"); a.st("DPOS"); a.ldi(0); a.st("DVEL")
    a.label("dive_done")
    a.ld("DPOS"); a.st("P"); a.mul("P"); a.ld("MRES10"); a.st("P2"); a.mul("P")
    a.ld("P2"); a.add("P2"); a.add("P2"); a.sub("MRES10"); a.sub("MRES10"); a.maxi(0); a.mini(1023); a.st("S")
    a.ld("CURROW", -8); a.sub("SPOS"); a.mul("PITCH"); a.ld("MRES8"); a.add("FOCAL"); a.sub("LIFT"); a.st("HY0")
    a.add("CARDH"); a.add("LIFT"); a.add("LIFT"); a.st("HY1")
    a.ld("COLXCUR"); a.sub("LIFT"); a.st("HX0"); a.add("CARDW"); a.add("LIFT"); a.add("LIFT"); a.st("HX1")
    a.ldi(0); a.sub("HX0"); a.mul("S"); a.ld("MRES10"); a.add("HX0"); a.st("EX0")
    a.ld("W"); a.sub("HX1"); a.mul("S"); a.ld("MRES10"); a.add("HX1"); a.st("EX1")
    a.ldi(0); a.sub("HY0"); a.mul("S"); a.ld("MRES10"); a.add("HY0"); a.st("EY0")
    a.ld("H"); a.sub("HY1"); a.mul("S"); a.ld("MRES10"); a.add("HY1"); a.st("EY1")
    a.ld("R"); a.mul("S"); a.ld("R"); a.sub("MRES10"); a.st("RH")
    a.ld("CARDH"); a.mul("S"); a.ld("MRES0", 13); a.st("INSETLIFT")
    a.ld("MRES0", 11); a.add("CARDH", 2); a.st("SH")
    a.ld("SHIFT"); a.abs(); a.shr(3); a.mini(15); a.st("T0"); a.ld("S", 6); a.max("T0"); a.st("FADE")
    a.ld("H", 8); a.addi(2); a.st("RINGG"); a.ld("H", 9); a.addi(1); a.st("RINGT")
    a.ld("RH"); a.add("RINGG"); a.st("RG"); a.add("RINGT"); a.st("RGT")
    for src, dst in (("RH", "R2H"), ("RG", "RG2"), ("RGT", "RGT2"), ("R", "R2L")):
        a.ld(src); a.shl(1); a.st("T0"); a.mul("T0"); a.ld("MRES0"); a.st(dst)
    a.ld("RH"); a.shl(1); a.add("SH"); a.st("T0"); a.ld("SH"); a.mul("T0"); a.ld("MRES0"); a.log2(); a.subi(3); a.add("LIGHT"); a.maxi(0); a.mini(15); a.st("SHSH")
    a.nop(); a.nop(); a.nop(); a.ld("R2H"); a.add("POW2SHSH"); a.st("RS2H")
    a.ld("R"); a.shl(1); a.add("SL"); a.st("T0"); a.ld("SL"); a.mul("T0"); a.ld("MRES0"); a.log2(); a.subi(3); a.add("LIGHT"); a.maxi(0); a.mini(15); a.st("SHSL")
    a.nop(); a.nop(); a.nop(); a.ld("R2L"); a.add("POW2SHSL"); a.st("RS2L")
    a.ld("R"); a.subi(2); a.jneg("r_small"); a.ldi(0); a.st("RLT2"); a.ld("R"); a.log2(); a.subi(1); a.st("SHL"); a.jmp("r_done")
    a.label("r_small"); a.ldi(1); a.st("RLT2"); a.ldi(0); a.st("SHL")
    a.label("r_done")
    a.ld("RH"); a.subi(2); a.jneg("rh_small"); a.ldi(0); a.st("RHLT2"); a.ld("RH"); a.log2(); a.subi(1); a.st("SHH"); a.jmp("rh_done")
    a.label("rh_small"); a.ldi(1); a.st("RHLT2"); a.ldi(0); a.st("SHH")
    a.label("rh_done")
    a.ld("RG"); a.log2(); a.subi(1); a.maxi(0); a.st("SHG")
    a.ldi(16); a.sub("S", 4); a.maxi(0); a.st("RINGA")
    a.ld("EX1"); a.sub("EX0"); a.st("HW")
    # row walker start, written straight into the line engine's file entries
    a.ld("SPOS", 8); a.subi(6); a.st("RSTART")
    a.ld("RSTART", -8); a.sub("SPOS"); a.mul("PITCH"); a.ld("MRES4"); a.add("FOCAL", -4); a.st("ROWTOP0")
    a.label("adv"); a.ld("ROWTOP0"); a.add("PITCH", -4); a.jpos("adv_done")
    a.st("ROWTOP0"); a.ld("RSTART"); a.addi(1); a.st("RSTART"); a.jmp("adv")
    a.label("adv_done"); a.ld("RSTART"); a.st("LROW"); a.ld("ROWTOP0"); a.st("LROWTOP")
    a.ld("FIRST"); a.add("CURSOR"); a.st("PALADDRF"); a.nop(); a.nop(); a.nop()
    a.ld("PALBASE"); a.st("HBASE"); a.ld("PALN"); a.addi(1); a.st("HN")
    a.ldi(0); a.st("SCAN")
    a.label("scan"); a.ld("HBASE"); a.add("SCAN"); a.st("ROMADDR"); a.nop(); a.nop(); a.nop()
    a.ld("ROMROLE"); a.jz("dom_found")
    a.ld("SCAN"); a.addi(1); a.st("SCAN"); a.sub("HN"); a.jneg("scan")
    a.ld("HBASE"); a.st("ROMADDR"); a.nop(); a.nop(); a.nop()
    a.label("dom_found"); a.ld("ROMU"); a.st("DOMU"); a.ld("ROMV"); a.st("DOMV")
    a.ld("DOMU"); a.subi(512); a.shr(3); a.addi(512); a.sub("TINTU"); a.shr(4); a.add("TINTU"); a.st("TINTU")
    a.ld("DOMV"); a.subi(512); a.shr(3); a.addi(512); a.sub("TINTV"); a.shr(4); a.add("TINTV"); a.st("TINTV")
    a.ld("GLIDE"); a.jz("shift_snap"); a.ld("SHIFT"); a.sub("SHIFT", 3); a.st("SHIFT"); a.jmp("shift_done")
    a.label("shift_snap"); a.ldi(0); a.st("SHIFT")
    a.label("shift_done")
    a.ld("CX"); a.mul("CX"); a.ld("MRES0"); a.st("CX2")
    a.ld("CY"); a.mul("CY"); a.ld("MRES0"); a.st("CY2")
    a.end()
    return a

# ----------------------------------------------------------------------------- line program (computes the NEXT line)
def line_program():
    a = Asm("line", base=640)
    a.ld("PIXELY"); a.addi(1); a.sub("YFIRST"); a.st("LY")
    a.ld("LROWTOP", 4); a.add("PITCH"); a.sub("LY"); a.jpos("walk_done")
    a.ld("LROW"); a.addi(1); a.st("LROW"); a.ld("LROWTOP"); a.add("PITCH", -4); a.st("LROWTOP")
    a.label("walk_done")
    a.ld("LROW", -8); a.sub("SPOS"); a.abs(); a.mini(512); a.st("LDIST")
    a.mul("INSETPX"); a.ld("MRES0", 9); a.add("INSETLIFT"); a.st("LINSET")
    a.ld("CARDW"); a.sub("LINSET"); a.sub("LINSET"); a.st("LW")
    a.ld("R"); a.add("LINSET"); a.st("LCRI")
    a.ldi(64); a.sub("LDIST", 4); a.sub("FADE"); a.sub("FADE"); a.maxi(0); a.st("LDIM")   # 64 - dist/16 - 2 fade
    a.ld("LROWTOP", 4); a.add("LINSET"); a.st("LY0")
    a.ld("LROWTOP", 4); a.add("CARDH"); a.sub("LINSET"); a.st("LY1")
    a.ld("LROWTOP"); a.st("LY0F")
    a.ldi(0); a.st("LROWOK")
    a.ld("LROW"); a.jneg("rowok_done"); a.sub("NROWS"); a.jpos("rowok_done"); a.jz("rowok_done"); a.ldi(1); a.st("LROWOK")
    a.label("rowok_done")
    def ey(y0, y1, r, oy, stores):
        a.ld(y0); a.add(r); a.sub("LY")
        if oy: a.add(oy)
        a.st("T0")
        a.ld("LY"); a.sub(y1); a.addi(1); a.add(r)
        if oy: a.sub(oy)
        a.max("T0"); a.maxi(0); a.mini(255); a.shl(1); a.subi(1); a.maxi(0); a.st("SQADDR"); a.nop(); a.nop(); a.nop()
        a.ld("SQ"); a.st("T0")
        for rad, dst in stores:          # K = radius^2 - EY^2 ; pixel path: cov from K - EX^2, ring inner edge from EX^2 - KG
            a.ld(rad); a.sub("T0"); a.st(dst)
    ey("LY0", "LY1", "R", None, [("R2L", "KL")])
    ey("LY0", "LY1", "R", "OYL", [("RS2L", "KSL")])
    ey("EY0", "EY1", "RH", None, [("R2H", "KH"), ("RG2", "KG"), ("RGT2", "KGT")])
    ey("EY0", "EY1", "RH", "OYH", [("RS2H", "KSH")])
    # row*cols
    a.ld("LROW"); a.st("T2"); a.ld("COLS"); a.subi(2); a.jneg("rc_done"); a.ld("T2"); a.add("LROW"); a.st("T2")
    a.ld("COLS"); a.subi(3); a.jneg("rc_done"); a.ld("T2"); a.add("LROW"); a.st("T2")
    a.label("rc_done")
    for c in range(3):
        a.ldi(0); a.st("LEX%d" % c)
        a.ld("LROWOK"); a.jz("col%d_done" % c)
        a.ld("COLS"); a.subi(c + 1); a.jneg("col%d_done" % c)
        a.ld("T2"); a.addi(c); a.st("T1"); a.sub("COUNT"); a.jpos("col%d_done" % c); a.jz("col%d_done" % c)
        a.ld("LROW"); a.sub("CURROW"); a.jnz("col%d_ok" % c); a.ld("CURCOL"); a.subi(c); a.jz("col%d_done" % c)
        a.label("col%d_ok" % c); a.ldi(1); a.st("LEX%d" % c)
        a.ld("T1"); a.add("FIRST"); a.st("PALADDR"); a.nop(); a.nop(); a.nop()
        a.ld("PALBASE"); a.st("LBASE%d" % c); a.ld("PALN"); a.addi(1); a.st("LN%d" % c)
        a.label("col%d_done" % c)
    a.end()
    return a

# ----------------------------------------------------------------------------- emit
def emit(path):
    f = frame_program(); l = line_program()
    fw, lw = f.words(), l.words()
    assert len(fw) <= 640 and len(lw) <= 384, (len(fw), len(lw))
    out = []
    w = out.append
    w("-- Generated by gamut_asm.py -- DO NOT EDIT")
    w("-- frame program: %d words, line program: %d words" % (len(fw), len(lw)))
    w("library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n")
    w("package gamut_prog_pkg is")
    w("  type t_prog1024 is array (0 to 1023) of std_logic_vector(23 downto 0);")
    w("  constant C_LINE_BASE : natural := 640;")
    w("  type t_rfile is array (0 to 127) of std_logic_vector(19 downto 0);")
    w("  type t_copy is array (0 to 31) of unsigned(6 downto 0);")
    w("  constant C_COPY_LEN : natural := %d;" % len(COPY_LIST))
    for i, n in enumerate(OPS): w("  constant OP_%-5s : natural := %d;" % (n, i))
    for n, i in SRC.items(): w("  constant S_%-9s : natural := %d;" % (n, i))
    for n, i in DST.items(): w("  constant D_%-9s : natural := %d;" % (n, i))
    w("  constant C_COPY_LIST : t_copy := (")
    for i in range(32):
        v = DST[COPY_LIST[i]] if i < len(COPY_LIST) else 0
        w("    %2d => to_unsigned(%d, 7)%s" % (i, v, "," if i < 31 else ""))
    w("  );")
    w("  constant C_RF_INIT : t_rfile := (")
    for i in range(128):
        v = RF_INIT.get(FILE_NAMES[i], 0) if i < len(FILE_NAMES) else 0
        w("    %3d => x\"%05X\"%s" % (i, v & 0xFFFFF, "," if i < 127 else ""))
    w("  );")
    w("  constant C_PROG : t_prog1024 := (")
    for i in range(1024):
        if i < 640: v = fw[i] if i < len(fw) else 0
        else: v = lw[i - 640] if i - 640 < len(lw) else 0
        w("    %4d => x\"%06X\"%s" % (i, v, "," if i < 1023 else ""))
    w("  );")
    w("end package gamut_prog_pkg;")
    open(path, "w").write("\n".join(out) + "\n")
    return len(fw), len(lw)

if __name__ == "__main__":
    import os
    n = emit(os.path.join(os.path.dirname(os.path.abspath(__file__)), "gamut_prog_pkg.vhd"))
    print("frame %d words, line %d words" % n)
