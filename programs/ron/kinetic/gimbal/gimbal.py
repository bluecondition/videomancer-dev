#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
GIMBAL v0.1.0 (Kinetic) - build hook.  Writes gimbal_pkg.vhd next to this file (pure python: the builder runs
it with the system python3).  The integer model (.kinetic_work/rb_gimbal/imodel.py) imports this file: it uses
the same tables, the same microcode and the engine emulator below, so model and RTL share one definition.

Package contents:
  C_GB_EXP   512 x 16   0 .. 255: round(32768 * 2^(i/256));  256 + palette * 12 + colour * 3 + (0 Y, 1 U, 2 V):
                        the 16 shared Kinetic palettes x (ground, A, B, accent), hardware convention (U = Cr)
  C_GB_SIN   512 x 16   round(65535 * sin(j pi / 1024)), a quarter wave (T[512] = 65535 is supplied by logic)
  C_GB_UC    512 x 22   the per-field engine's microcode (op 5 | R 1 | sw 2 | a 9 | n 5)
  C_GB_RF    512 x 48   the register file's initial contents (constants, glide start values, the start phase)

The per-field engine (one serial multiplier, one adder, a 48-bit accumulator) runs the program below once per
field during the active lines (in windows the per-line engine leaves free) and leaves the NEXT field's values in
shadow slots of the register file; the vblank copy moves them into the live slots and the pixel constants.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# The 16 shared Kinetic palettes (copied from .kinetic_work/tools/kpalette.py so this folder is self-contained)
PALETTES = [
    # label        ground     A          B          accent
    ("Ink",        "#0E0E10", "#F2EFE6", "#8C8C8C", "#E23B2E"),
    ("Paper",      "#F2EFE6", "#141414", "#9A9A94", "#1F4E9E"),
    ("Bauhaus",    "#EDE4CF", "#D1362F", "#1F4E9E", "#F0B81C"),
    ("Vega",       "#0B1E5B", "#1BB5E8", "#D81E8C", "#F5D90A"),
    ("Outrun",     "#12002B", "#FF2E97", "#00D9F5", "#FFB000"),
    ("Ultra",      "#0D0221", "#7B2CBF", "#E0AAFF", "#3CF2C0"),
    ("Ocean",      "#03045E", "#0096C7", "#90E0EF", "#F4F1DE"),
    ("Ice",        "#06141B", "#9BD1E5", "#F1F7FA", "#4A7C9B"),
    ("Moss",       "#0F2A1D", "#7FB069", "#E6F0A3", "#F18F01"),
    ("Citrus",     "#14213D", "#FCA311", "#E5E5E5", "#E63946"),
    ("Terra",      "#2B1A12", "#C8553D", "#F2D0A4", "#588B8B"),
    ("Ember",      "#1A0A00", "#FF4500", "#FFB347", "#8C1C13"),
    ("Gold",       "#0A0A0A", "#D4A017", "#F6E7B0", "#7A5C00"),
    ("Candy",      "#FFF1E6", "#FF6B9D", "#56CBF9", "#FFD23F"),
    ("Mint",       "#F7F3E9", "#2EC4B6", "#FF6B6B", "#1D3557"),
    ("Plum",       "#2D1E2F", "#F7B2AD", "#9ABCA7", "#F2E94E"),
]

# TOML power-up defaults (the glides start there)
DEF_K1, DEF_K2, DEF_K3, DEF_K6, DEF_P12 = 571, 341, 784, 1023, 523
START_PHASE = 0.2            # cycles: the power-up frame is a good one

# ring geometry at scale 1 (virtual px): NR rings; the hub disc lies in ring 0's plane
NR = 5
RING_RO = [int(round(256 + (900 - 256) * k / (NR - 1))) for k in range(NR)]
HUB_R = 80
RING_WIDE = 116
RING_WIRE = 31
RIM_FRAC = 0.16              # rim width as a fraction of the ring width (in the ring's own plane)
TAU = 3                      # hole closure (a ring seen edge-on is a bar, not two slivers)


def hw10(h):
    """sRGB hex -> (y, u, v) 10-bit, hardware convention (u = Cr, v = Cb), BT.601, Y = 64 + 876 y."""
    h = h.lstrip("#")
    r, g, b = [int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    return (int(round(64 + y * 876)), int(round(512 + cr * 896)), int(round(512 + cb * 896)))


def pal_table():
    return [[hw10(c) for c in p[1:]] for p in PALETTES]


def sin_rom():
    return [int(round(65535 * math.sin(j * math.pi / 1024.0))) for j in range(513)]


PAL_BASE, EXP_BASE = 576, 768


def rom():
    """1024 x 16: the quarter sine 0 .. 512 (T[512] = 65535), the palettes from 576 (palette * 12 + colour * 3
    + channel), 2^(i/256) * 32768 from 768"""
    out = sin_rom() + [0] * (PAL_BASE - 513)
    for p in pal_table():
        for (y, u, v) in p:
            out += [y, u, v]
    out += [0] * (EXP_BASE - len(out))
    out += [int(round(32768 * 2 ** (i / 256.0))) for i in range(256)]
    assert len(out) == 1024
    return out


ROM = rom()
SIN = ROM[0:513]
EXP = ROM[EXP_BASE:EXP_BASE + 256]

# ---------------------------------------------------------------------------------------------------------------
# microcode
# ---------------------------------------------------------------------------------------------------------------
OPS = dict(NOP=0, LD=1, ADD=2, SUB=3, ST=4, MUL=5, DIV=6, SHL=7, SHR=8, SIN=9, EXP=10, GLIDE=11, TST=12,
           NEGF=13, ABS=14, MAX=15, MIN=16, IN=17, SMP=18, LOOP=19, JS=20, JMP=21, END=22)
OPN = {v: k for k, v in OPS.items()}
SW_NONE, SW_MIRROR, SW_WIRE, SW_OSC = 0, 1, 2, 3          # address-bit-0 select by switch
JS_SNAP, JS_MIRROR, JS_TUMBLE, JS_WIRE, JS_OSC = 0, 1, 2, 3, 4

# register file map (512 x 48)
#   block A, ring k (0..NR-1) at 16 k: 0 Q0  1 E  2 D0  3 z0  4 E2  5 Dy  6 Zy  (live line state, per-line engine)
#                                   7 .. 13 the same seven, the engine's shadow for the next field
#                                   14 K2A  15 Kz  (live copies of block B 0 / 1, reloaded every line)
#   block B, ring k at 16 NR + 16 k:  0 K2A  1 Kz  2 Wh16  3 Kr  4 lvl  5 face   (pixel-constant shadows)
#                                   6 ro24  7 cm0  8/9 ri24 wide/wire  10/11 idx chase/mirror  12 kk
#                                   13 Kh (the hub's threshold, ring 0's is used)  14/15 rw24 wide/wire
#   globals from 32 NR
A_Q0, A_E, A_D0, A_Z0, A_E2, A_DY, A_ZY = range(7)
S_OFF = 7
B = 16 * NR
B_K2A, B_KZ, B_WH, B_KR, B_LVL, B_FACE, B_RO, B_CM, B_RI, B_IDX, B_KK, B_KH, B_RW = (
    B + 0, B + 1, B + 2, B + 3, B + 4, B + 5, B + 6, B + 7, B + 8, B + 10, B + 12, B + 13, B + 14)

GLOBALS = ["PHASE", "G1", "G2", "G3", "GP", "G6", "SC", "ISC", "DC", "CS", "FAN", "CAMG", "KC",
           "X0", "Y0", "S8", "SY8", "CX2", "CXY", "CY2", "CD1", "CD2", "CE1", "CE2", "CDY", "CK2", "CE2Y",
           "HMIN", "HMAX",
           "T0", "T1", "TRO23", "TRO8", "TRI8", "TCM", "TTH", "TAL", "TRR", "TC", "TS", "TCP", "TTO",
           "TN", "TPHI", "TS2", "TC2", "TSPHI", "TCPHI", "TCC", "TCCM1", "TCOS2", "TSIN2", "TA", "TB", "TCQ",
           "TT", "TZX", "TZY", "TWH"]
CONSTS = [("C0", 0), ("C1", 1), ("C3", 3), ("C8", 8), ("C13", 13), ("C80", 80), ("C19", 19), ("C16", 16), ("C48", 48), ("C64", 64),
          ("C220", 220), ("C512", 512), ("C897", 897), ("C1023", 1023), ("C2662", 2662), ("C3156", 3156),
          ("C4096", 4096), ("C7550", 7550), ("C8191", 8191), ("C9321", 9321), ("C65536", 65536),
          ("CM65536", -65536), ("C138240", 138240), ("C262144", 262144), ("C270336", 270336), ("C245760", 245760),
          ("C253952", 253952), ("CDMIN", 671089), ("CQ4", 1 << 22), ("CONE24", 1 << 24),
          ("CTAU2", TAU * TAU * 65536), ("CWHMIN", 3 << 23), ("CRH24", HUB_R << 24)]
RF = {}
_a = 32 * NR
for g in GLOBALS:
    RF[g] = _a; _a += 1
for (c, v) in CONSTS:
    RF[c] = _a; _a += 1
if _a & 1:
    _a += 1
RF["COSC"] = _a; _a += 2                                  # pair: Video 1 / Osc 2 (camera gain)
RF["CRIMW"] = _a; _a += 2                                 # pair: rim 6 px (Wide) / 3 px (Wire), Q24
assert _a <= 256


def rf_init():
    rf = [0] * 256
    for (c, v) in CONSTS:
        rf[RF[c]] = v
    rf[RF["COSC"]] = 1
    rf[RF["COSC"] + 1] = 2
    rf[RF["CRIMW"]] = 6 << 24
    rf[RF["CRIMW"] + 1] = 3 << 24
    rf[RF["G1"]] = DEF_K1 * 8
    rf[RF["G2"]] = DEF_K2 * 8
    rf[RF["G3"]] = DEF_K3 * 8
    rf[RF["GP"]] = DEF_P12 * 8
    rf[RF["G6"]] = DEF_K6 * 8
    rf[RF["PHASE"]] = int(round(START_PHASE * (1 << 24)))
    for k in range(NR):
        b = B + 16 * k
        ro = RING_RO[k]
        rf[b + 6] = ro << 24
        rf[b + 7] = int(math.ceil(3 * (1 << 25) / ro))                   # (3 / ro) Q25: an edge-on ring is a 6-px bar
        rf[b + 8] = (ro - RING_WIDE) << 24
        rf[b + 9] = (ro - RING_WIRE) << 24
        rf[b + 10] = 2 * k                                                # Chase: index k (half steps)
        rf[b + 11] = abs(2 * k - (NR - 1))                                # Mirror: |k - (NR - 1) / 2|
        rf[b + 12] = k
        rf[b + 14] = int(round(RIM_FRAC * RING_WIDE * (1 << 24)))
        rf[b + 15] = int(round(RIM_FRAC * RING_WIRE * (1 << 24)))
    return rf


class Asm:
    def __init__(self):
        self.code = []
        self.labels = {}
        self.fix = []

    def label(self, name):
        self.labels[name] = len(self.code)

    def emit(self, op, a=0, n=0, R=0, sw=0):
        if isinstance(a, str) and a not in RF:
            self.fix.append((len(self.code), a))
            a = 0
        elif isinstance(a, str):
            a = RF[a]
        self.code.append([OPS[op], R, sw, a, n])

    def __getattr__(self, op):
        if op in OPS:
            return lambda a=0, n=0, R=0, sw=0: self.emit(op, a, n, R, sw)
        raise AttributeError(op)

    def words(self):
        for (i, lab) in self.fix:
            self.code[i][3] = self.labels[lab]
        out = []
        for (op, R, sw, a, n) in self.code:
            assert 0 <= a < 512 and 0 <= n < 32
            out.append((op << 17) | (R << 16) | (sw << 14) | (a << 5) | n)
        return out


def program():
    m = Asm()
    # ---- glides (1/8 of the gap per field, +-2 code deadband) ----
    for (g, n) in (("G1", 0), ("G2", 1), ("G3", 2), ("GP", 3), ("G6", 4)):
        m.GLIDE(g, n); m.ST(g)
    # ---- rate: bipolar, +-8 deadband, exponential; phase += step ----
    m.IN(0, 0); m.SUB("C512"); m.TST(); m.ABS(); m.SUB("C8"); m.MAX("C0"); m.ST("T0")
    m.SHL(0, 2); m.MUL("C897", 10); m.EXP(); m.MUL("C9321", 16); m.ST("T1")
    m.LD("T0"); m.SHL(0, 28); m.ST("TA"); m.LD("T1"); m.MIN("TA")
    m.NEGF(); m.ADD("PHASE"); m.ST("PHASE")
    # ---- scale 0.55 .. 1.6 (exponential), its inverse ----
    m.LD("G1"); m.MUL("C3156", 12); m.SHR(0, 4); m.SUB("C220"); m.ST("T0"); m.EXP(); m.ST("SC")
    m.LD("C0"); m.SUB("T0"); m.EXP(); m.ST("ISC")
    m.LD("G6"); m.MUL("C3", 2); m.ADD("C1023"); m.SHR(0, 11); m.ST("KC")
    # the accent rim on every ring's outer edge: 6 px (Wire 3 px) x scale, Q4; packed with kc as kc + 4 rim
    m.LD("CRIMW", sw=SW_WIRE); m.MUL("SC", 16); m.SHR(0, 19); m.SHL(0, 2); m.ADD("KC"); m.ST("KC")
    # ---- depth (Rock amplitude, cycles Q24): 0.04 + 0.46 k3^2;  cascade: 0.46 p12^2 ----
    m.LD("G3"); m.SHL(0, 13); m.MUL("G3", 13); m.MUL("C7550", 13); m.SHR(0, 3); m.ADD("CDMIN"); m.ST("DC")
    m.LD("GP"); m.SHL(0, 13); m.MUL("GP", 13); m.MUL("C7550", 13); m.SHR(0, 3); m.ST("CS")
    # ---- fan step 0 .. 90 degrees per ring ----
    m.LD("G2"); m.SHL(0, 9); m.ST("FAN")
    # ---- camera: tilt gain (Osc x 2), face-tint gain kc 0 .. 3 ----
    m.LD("G6"); m.SHL(0, 2); m.MUL("COSC", 2, sw=SW_OSC); m.ST("CAMG")
    # ---- video mode: s, sy, x0, y0 (Q8 virtual px) and the products the DDA start values need (Q23) ----
    m.IN(0, 2); m.SHL(0, 7); m.ST("S8")
    m.IN(0, 1); m.ST("SY8")
    m.IN(0, 3); m.SHL(0, 8); m.ST("T0"); m.LD("S8"); m.SHR(0, 1); m.SUB("T0"); m.ST("X0")
    m.LD("SY8"); m.SHR(0, 1); m.SUB("C138240"); m.ST("Y0")
    m.LD("C245760"); m.SUB("T0"); m.ADD("C8191"); m.SHR(0, 13); m.ST("HMIN")
    m.LD("C245760"); m.ADD("T0"); m.SHR(0, 13); m.SUB("C1"); m.ST("HMAX")
    m.LD("X0"); m.SHL(0, 15); m.MUL("X0", 18); m.SHL(0, 10); m.ST("CX2")
    m.LD("X0"); m.SHL(0, 15); m.MUL("Y0", 18); m.SHL(0, 10); m.ST("CXY")
    m.LD("Y0"); m.SHL(0, 15); m.MUL("Y0", 18); m.SHL(0, 10); m.ST("CY2")
    m.LD("X0"); m.SHL(0, 1); m.ADD("S8"); m.SHL(0, 15); m.MUL("S8", 10); m.SHL(0, 2); m.ST("CD1")
    m.LD("Y0"); m.SHL(0, 15); m.MUL("S8", 10); m.SHL(0, 2); m.ST("CD2")
    m.LD("Y0"); m.SHL(0, 1); m.ADD("SY8"); m.SHL(0, 15); m.MUL("SY8", 11); m.SHL(0, 3); m.ST("CE1")
    m.LD("X0"); m.SHL(0, 15); m.MUL("SY8", 11); m.SHL(0, 3); m.ST("CE2")
    m.LD("S8"); m.SHL(0, 15); m.MUL("SY8", 11); m.SHL(0, 3); m.ST("CDY")
    m.LD("S8"); m.SHL(0, 16); m.MUL("S8", 10); m.SHL(0, 2); m.ST("CK2")
    m.LD("SY8"); m.SHL(0, 16); m.MUL("SY8", 11); m.SHL(0, 3); m.ST("CE2Y")

    # ================= per ring (RK = 0 .. NR - 1) =================
    m.label("RING")
    m.LD(B_RO, R=1); m.MUL("SC", 16); m.ST("TRO23")                        # Ro Q23
    m.SHR(0, 15); m.ST("TRO8")                                             # Ro Q8
    m.LD(B_RI, R=1, sw=SW_WIRE); m.MUL("SC", 16); m.SHR(0, 15); m.ST("TRI8")   # Ri Q8
    m.LD(B_CM, R=1); m.MUL("ISC", 16); m.ST("TCM")                         # c_min = 3 / Ro, Q24
    # wave phase theta = phase + cascade * index
    m.LD("CS"); m.SHL(0, 3); m.MUL(B_IDX, 4, R=1, sw=SW_MIRROR); m.ADD("PHASE"); m.ST("TTH")
    m.JS("SNAP", JS_SNAP)
    m.SIN(); m.JMP("WAVED")
    m.label("SNAP")                                                        # dwell at the ends, eased flips
    m.SIN(); m.SHL(0, 2); m.MUL("C2662", 12); m.MIN("C65536"); m.MAX("CM65536"); m.SHL(0, 6); m.SIN()
    m.label("WAVED")                                                       # wave Q16
    m.SHL(0, 8); m.MUL("DC", 24)                                           # Rock: alpha = depth * wave
    m.JS("TUMB", JS_TUMBLE)
    m.JMP("ALDONE")
    m.label("TUMB")                                                        # Tumble: theta + (k/4) phase + depth/2 * wave
    m.SHR(0, 1); m.ADD("TTH"); m.ST("T0"); m.LD("PHASE"); m.SHL(0, 22); m.SHR(0, 22)   # phase mod 4 cycles
    m.SHL(0, 1); m.MUL(B_KK, 3, R=1); m.ADD("T0")
    m.label("ALDONE")
    m.ST("TAL")
    # camera: the luma at four points of the ring (on the axes, mid radius) adds to the tilt
    m.LD("TRO8"); m.ADD("TRI8"); m.SHR(0, 1); m.ADD("C4096"); m.ST("TRR")
    m.LD("C245760"); m.ADD("TRR"); m.SHR(0, 13); m.MIN("HMAX"); m.SMP(); m.ST("T1")
    m.LD("C253952"); m.SUB("TRR"); m.SHR(0, 13); m.MAX("HMIN"); m.SMP(); m.ADD("T1"); m.ST("T1")
    m.LD("C262144"); m.ADD("TRR"); m.SHR(0, 13); m.MIN("C48"); m.ADD("C64"); m.SMP(); m.ADD("T1"); m.ST("T1")
    m.LD("C270336"); m.SUB("TRR"); m.SHR(0, 13); m.MAX("C16"); m.ADD("C64"); m.SMP(); m.ADD("T1")
    m.SHL(0, 14); m.MUL("CAMG", 15); m.ADD("TAL")
    m.SHL(0, 24); m.SHR(0, 24); m.ST("TAL")                                # alpha mod 1 cycle
    # cos, sin; face; c' = max(|c|, c_min)
    m.ADD("CQ4"); m.SIN(); m.ST("TC")
    m.LD("TAL"); m.SIN(); m.ST("TS")
    m.LD("TC"); m.TST(); m.ABS(); m.SHL(0, 8); m.MAX("TCM"); m.ST("TCP")
    m.LD("TC"); m.SHR(0, 31); m.SHR(0, 16); m.ST(B_FACE, R=1)
    m.LD("TCP"); m.MUL("C19", 5); m.SHR(0, 19); m.ADD("C13"); m.ST(B_LVL, R=1)   # brightness 13 .. 31
    # T_out = c' Ro / 2 (Q24); N = 1 / (2 c' Ro) (Q25)
    m.LD("TRO23"); m.MUL("TCP", 24); m.ST("TTO")
    m.SHR(0, 14); m.DIV("C512", 24); m.ST("TN")                             # 512 * 2^24 / (2 c' Ro, Q8)
    # hinge angle phi = 1/4 + k * fan; sin / cos of phi and 2 phi
    m.LD("FAN"); m.SHL(0, 3); m.MUL(B_KK, 3, R=1); m.ADD("CQ4"); m.ST("TPHI")
    m.SHL(0, 1); m.ST("T0"); m.SIN(); m.ST("TS2")
    m.LD("T0"); m.ADD("CQ4"); m.SIN(); m.ST("TC2")
    m.LD("TPHI"); m.SIN(); m.ST("TSPHI")
    m.LD("TPHI"); m.ADD("CQ4"); m.SIN(); m.ST("TCPHI")
    # A = sin^2 + c'^2 cos^2,  C = cos^2 + c'^2 sin^2,  B = (c'^2 - 1) sin 2phi;  then x N (Q25)
    m.LD("TCP"); m.MUL("TCP", 24); m.ST("TCC")
    m.SUB("CONE24"); m.ST("TCCM1")
    m.LD("TC2"); m.ADD("C65536"); m.SHL(0, 7); m.ST("TCOS2")
    m.LD("CONE24"); m.SUB("TCOS2"); m.ST("TSIN2")
    m.LD("TCOS2"); m.MUL("TCC", 24); m.ADD("TSIN2"); m.MUL("TN", 24); m.ST("TA")
    m.LD("TSIN2"); m.MUL("TCC", 24); m.ADD("TCOS2"); m.MUL("TN", 24); m.ST("TCQ")
    m.LD("TS2"); m.SHL(0, 8); m.MUL("TCCM1", 24); m.MUL("TN", 24); m.ST("TB")
    # T_in = (c'^2 Ri^2 - tau^2) N;  W = T_out - T_in (Q4 px);  rim = c' rw scale (Q4 px)
    m.LD("TRI8"); m.SHL(0, 19); m.MUL("TRI8", 19); m.MUL("TCC", 24); m.SUB("CTAU2"); m.SHL(0, 7)
    m.MUL("TN", 24); m.ST("T0")
    m.LD("TTO"); m.SUB("T0"); m.SHR(0, 1); m.MAX("CWHMIN"); m.ST("TWH")    # W / 2 (Q24), a band >= 3 px
    m.SHR(0, 20); m.ST("T0")                                                # W / 2 (Q4 px)
    m.LD("TCP"); m.MUL("C80", 7); m.SHR(0, 17); m.MIN("C8")                 # og = min(1/2 px, 5 c' px):
    m.ADD("T0"); m.ST(B_WH, R=1)                                            # the AA offset, small on thin rings
    # the hub disc in this ring's plane (ring 0's is used): Kh = T_hub - T_out + W / 2 (Q4 px)
    m.LD("CRH24"); m.MUL("SC", 16); m.SHR(0, 15); m.ST("T1")              # R_hub Q8
    m.SHL(0, 19); m.MUL("T1", 19); m.MUL("TCC", 24); m.SHL(0, 7); m.MUL("TN", 24)
    m.SUB("TTO"); m.ADD("TWH"); m.SHR(0, 20); m.ST(B_KH, R=1)
    # depth z = v tan(alpha):  t = s / c (signed c), zx = -t sin phi, zy = t cos phi (Q26)
    m.LD("TRO23"); m.MUL("TN", 24); m.SHR(0, 9); m.ST("T0")                # 1 / c'  Q14
    m.LD("TS"); m.SHL(0, 20); m.MUL("T0", 24); m.NEGF(); m.ST("TT")
    m.LD("TT"); m.MUL("TSPHI", 16); m.ST("T0"); m.LD("C0"); m.SUB("T0"); m.ST("TZX")
    m.LD("TT"); m.MUL("TCPHI", 16); m.ST("TZY")
    m.LD("TZX"); m.MUL("S8", 10); m.SHR(0, 13); m.ST(B_KZ, R=1)            # Kz Q11 per pixel
    m.LD("TZY"); m.MUL("SY8", 11); m.SHR(0, 12); m.ST(A_ZY + S_OFF, R=1)   # Zy Q11 per line
    m.LD("TZX"); m.MUL("X0", 18); m.ST("T0")
    m.LD("TZY"); m.MUL("Y0", 18); m.ADD("T0"); m.SHR(0, 5); m.ST(A_Z0 + S_OFF, R=1)
    # quadratic DDA start values (Q24) at pixel 0, line 0, and their differences
    m.LD("CX2"); m.MUL("TA", 24); m.ST("T0")
    m.LD("CXY"); m.MUL("TB", 24); m.ADD("T0"); m.ST("T0")
    m.LD("CY2"); m.MUL("TCQ", 24); m.ADD("T0"); m.SUB("TTO"); m.ADD("TWH"); m.ST(A_Q0 + S_OFF, R=1)
    m.LD("CD1"); m.MUL("TA", 24); m.ST("T0")
    m.LD("CD2"); m.MUL("TB", 24); m.ADD("T0"); m.ST(A_D0 + S_OFF, R=1)
    m.LD("CK2"); m.MUL("TA", 24); m.ST(B_K2A, R=1)
    m.LD("CE1"); m.MUL("TCQ", 24); m.ST("T0")
    m.LD("CE2"); m.MUL("TB", 24); m.ADD("T0"); m.ST(A_E + S_OFF, R=1)
    m.LD("CE2Y"); m.MUL("TCQ", 24); m.ST(A_E2 + S_OFF, R=1)
    m.LD("CDY"); m.MUL("TB", 24); m.ST(A_DY + S_OFF, R=1)
    m.LOOP("RING", NR)
    m.END()
    return m


PROG = program()
UC = PROG.words()


# ---------------------------------------------------------------------------------------------------------------
# engine emulator (the RTL's p_eng mirrors this exactly)
# ---------------------------------------------------------------------------------------------------------------
M48 = (1 << 48) - 1


def w48(v):
    v &= M48
    return v - (1 << 48) if v >> 47 else v


def sin_q16(acc):
    ang = acc & 0xFFFFFF
    i, f = ang >> 13, ang & 8191
    q, j = i >> 9, i & 511
    T = lambda x: SIN[x]
    if q & 1:
        v1, v2 = T(512 - j), T(511 - j)
    else:
        v1, v2 = T(j), T(j + 1)
    mag = ((v1 << 13) + (v2 - v1) * f) >> 13
    return -mag if q >= 2 else mag


def run_engine(rf, knobs, inputs, switches, samples, trace=None):
    """One field's program.  rf: list of 512 ints (modified in place).  knobs: raw 10-bit [K1, K2, K3, P12, K6];
    inputs: [K4 raw, ys, xs, X0]; switches: [S7 snap, S8 mirror, S9 tumble, S10 wire, S11 osc];
    samples: 128 luma8 values (0 .. 59 the centre line, 64 .. 97 the centre column).  Returns the step count."""
    pc, rk, acc, F = 0, 0, 0, 0
    swsel = {SW_MIRROR: switches[1], SW_WIRE: switches[3], SW_OSC: switches[4]}
    steps = 0
    while True:
        steps += 1
        op, R, sw, a, n = (UC[pc] >> 17) & 31, (UC[pc] >> 16) & 1, (UC[pc] >> 14) & 3, (UC[pc] >> 5) & 511, UC[pc] & 31
        addr = (a + 16 * rk) & 511 if R else a
        if sw and swsel[sw]:
            addr ^= 1
        name = OPN[op]
        pc += 1
        if name == "LD":
            acc = rf[addr]
        elif name == "ADD":
            acc = w48(acc + rf[addr])
        elif name == "SUB":
            acc = w48(acc - rf[addr])
        elif name == "ST":
            rf[addr] = acc
        elif name == "MUL":
            assert -(1 << 46) < acc < (1 << 46), ("MUL operand", pc - 1, acc)
            y = rf[addr]
            neg = y < 0
            ym = ((-y) if neg else y) & 0xFFFFFF
            ym &= (1 << n) - 1
            p = (acc * ym) >> n
            acc = w48(-p if neg else p)
        elif name == "DIV":
            num = rf[addr]
            assert 0 <= num < acc, ("DIV range", num, acc, rk)
            acc = (num << n) // acc
        elif name == "SHL":
            acc = w48(acc << n)
        elif name == "SHR":
            acc = acc >> n
        elif name == "SIN":
            acc = sin_q16(acc)
        elif name == "EXP":
            fr, it = acc & 255, acc >> 8
            v = EXP[fr]
            acc = (v << it) if it >= 0 else (v >> (-it))
        elif name == "GLIDE":
            g = rf[addr]
            d = (knobs[n] << 3) - g
            if d > 16 or d < -16:
                g = g + (d >> 3)
            acc = g
        elif name == "TST":
            F = 1 if acc < 0 else 0
        elif name == "NEGF":
            if F:
                acc = w48(-acc)
        elif name == "ABS":
            if acc < 0:
                acc = w48(-acc)
        elif name == "MAX":
            acc = max(acc, rf[addr])
        elif name == "MIN":
            acc = min(acc, rf[addr])
        elif name == "IN":
            acc = inputs[n]
        elif name == "SMP":
            acc = samples[acc & 127]
        elif name == "LOOP":
            rk += 1
            if rk < n:
                pc = a
            else:
                rk = 0
        elif name == "JS":
            if switches[n]:
                pc = a
        elif name == "JMP":
            pc = a
        elif name == "END":
            return steps
        if trace is not None:
            trace.append((pc - 1, name, rk, acc))


# ---------------------------------------------------------------------------------------------------------------
# VHDL emission
# ---------------------------------------------------------------------------------------------------------------
def vhd_table(name, depth, width, vals, signed=False):
    typ = "signed" if signed else "unsigned"
    lines = ["    type %s is array (0 to %d) of %s(%d downto 0);" % ("t_" + name.lower(), depth - 1, typ, width - 1),
             "    constant %s : %s := (" % (name, "t_" + name.lower())]
    nd = (width + 3) // 4
    items = []
    for v in vals:
        v &= (1 << width) - 1
        if width % 4 == 0:
            items.append('x"%0*X"' % (nd, v))
        else:
            items.append('"' + format(v, '0%db' % width) + '"')
    per = 16 if width % 4 == 0 else 4
    for i in range(0, len(items), per):
        lines.append("        " + ", ".join(items[i:i + per]) + ("," if i + per < len(items) else ""))
    lines.append("    );")
    return "\n".join(lines)


def main():
    uc = UC + [0] * (512 - len(UC))
    parts = ["-- Generated by gimbal.py -- DO NOT EDIT.",
             "library ieee;", "use ieee.std_logic_1164.all;", "use ieee.numeric_std.all;", "",
             "package gimbal_pkg is",
             vhd_table("C_GB_ROM", 1024, 16, ROM),
             vhd_table("C_GB_UC", 512, 22, uc),
             vhd_table("C_GB_RF", 256, 48, rf_init(), signed=True),
             "    constant C_GB_UCLEN : integer := %d;" % len(UC),
             "    constant C_GB_NR : integer := %d;" % NR,
             "    constant C_GB_KC : integer := %d;" % RF["KC"],
             "    constant C_GB_PAL : integer := %d;" % PAL_BASE,
             "end package gimbal_pkg;", ""]
    with open(os.path.join(HERE, "gimbal_pkg.vhd"), "w") as f:
        f.write("\n".join(parts))


if __name__ == "__main__":
    main()
