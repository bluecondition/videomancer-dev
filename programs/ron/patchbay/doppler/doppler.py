#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
DOPPLER v0.1.0 - build hook.  Writes doppler_pkg.vhd next to this file (pure python: the builder runs it with the
system python3).  imodel.py (.patchbay_work/rb_doppler/) imports this file for the microcode, the tables and the
palette, so the model and the RTL share one source.

The patch: wavefronts from a moving source.  Ring m = n + f (n = 0..15) is centred at S - m d with radius m L:
    F_m(p) = |q|^2 + 2 m (d.q) + m^2 (|d|^2 - L^2),   q = p - S,  |d| = Mach * L
Per field and per line a small microcoded engine (one serial multiply-accumulate, a 64-word register file in block
RAM) computes every constant; the pixel path runs two DDAs (P = F_0, H ~ the first difference in m), scales them by
2^-e and walks the 16 rings in a 16-stage adder chain F_{n+1} = F_n + J_n, J_{n+1} = J_n + 2K.

Package contents:
  C_DP_IROM  512 x 48   microcode (see ASM below for the encoding)
  C_DP_EXP / C_DP_SIN / C_DP_SQRT  3 x 256 x 16  engine tables: 16384*2^(i/256), 32767 sin(2 pi i/256),
                        32767 sqrt(i/255)
  C_DP_COL   1024 x 24  colouriser {Y8, U8, V8} (hardware convention U = Cr), index = hue * 16 + band * 4 + line
  C_DP_HTAB  128 x 8    doppler coordinate (u7 + 64) -> hue * 256 (Shift colours)
  C_DP_FADE  128 x 4    band fade: index = fade_on * 64 + clamp((32 V - f) >> 3 + 2, 0, 63)
  C_DP_RLEV  64 x 7     Rings: tot -> brightness (0..16) * 4 + whitening level
"""
import math, os

# --------------------------------------------------------------------------------------------------------------
# Colour helpers (hardware convention: U register = Cr, V register = Cb; Y = 64 + Y' 876; C = 512 + c 896)
# --------------------------------------------------------------------------------------------------------------
def hsv(h, s, v):
    h = h % 1.0
    i = int(math.floor(h * 6)) % 6
    fr = h * 6 - math.floor(h * 6)
    p = v * (1 - s); q = v * (1 - s * fr); t = v * (1 - s * (1 - fr))
    r, g, b = [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][i]
    return r, g, b


def rgb_hw(r, g, b):
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = (b - y) / 1.772
    cr = (r - y) / 1.402
    Y = 64 + y * 876
    Y = 64 + (Y - 64) * 16.0 / 13.0                 # the shell's output scales luma by 13/16
    Y = min(Y, 940.0)
    U = 512 + cr * 896
    V = 512 + cb * 896
    q = lambda x: max(0, min(255, int(round(x / 4.0))))
    return q(Y), q(U), q(V)


# band staircase (Shift colours): value, saturation, hue offset per band identity
BANDV = [0.06, 1.00, 0.14, 0.88]
BANDS = [1.00, 1.00, 1.00, 0.90]
BANDH = [0.00, 0.00, 0.00, 0.17]
LINEA = [0.0, 0.27, 0.53, 0.80]                     # line blend per line level


def col_rom():
    """1024 x 24: index = hue (0..63) * 16 + band * 4 + line.  Bands: band identity colour blended toward the pale
    line colour by line level; Rings uses band 1 (the vivid hue) and the line levels as overlap whitening, scaled by
    the brightness multiplier."""
    rom = []
    for h in range(64):
        hue = h / 64.0
        for sub in range(16):
            band, ln = sub >> 2, sub & 3
            base = hsv(hue + BANDH[band], BANDS[band], BANDV[band])
            lc = hsv(hue, 0.25, 1.0)
            a = LINEA[ln]
            rom.append(rgb_hw(*tuple(base[i] * (1 - a) + lc[i] * a for i in range(3))))
    return rom


def htab():
    """doppler coordinate u7 = clamp(u * 32, -64, 63) (+64) -> hue * 256 (8 bits; behind red, sides magenta,
    ahead blue).  The pixel path adds K5's rotation and a 4-phase horizontal dither, then keeps the top 6 bits."""
    t = []
    for i in range(128):
        u = math.tanh((i - 64) / 32.0)
        if u < 0: hue = 0.86 + (1.03 - 0.86) * (-u)
        else: hue = 0.86 + (0.56 - 0.86) * u
        t.append(int(round(hue * 256)) % 256)
    return t


def fade_tab():
    """band fade by age x = V - f (bands outside the line's own band in Rings): index = fade_on * 64 +
    clamp(((32 V - f32) >> 3) + 2, 0, 63), the bin's lower edge (never brighter than the true age; 0 for x <= 0,
    so the region outside the oldest front stays black through the f wrap).  Off: clip(x); On: clip(x / 7.5)^0.8.
    0..15, 15 = full."""
    t = []
    for on in range(2):
        for i in range(64):
            x = (i - 2) * 8 / 32.0
            if on: v = min(max(x / 7.5, 0.0), 1.0) ** 0.8
            else: v = min(max(x, 0.0), 1.0)
            t.append(int(round(v * 15)))
    return t


def rlev_tab():
    """Rings: tot (0..63, 15 = one ring at full) -> brightness 0..16 (5 bits) and whitening line level 0..3"""
    t = []
    for tot in range(64):
        v = min(tot / 16.0, 1.0) ** 0.75
        b = int(round(16 * v))
        wl = min(3, max(0, tot - 16) // 4)
        t.append((b, wl))
    return t


def tab_rom():
    t = []
    for i in range(256): t.append(int(round(16384 * 2 ** (i / 256.0))))
    for i in range(256): t.append(int(round(32767 * math.sin(2 * math.pi * i / 256.0))))
    for i in range(256): t.append(int(round(32767 * math.sqrt(i / 255.0))))
    for i in range(256): t.append(0)
    return t


# --------------------------------------------------------------------------------------------------------------
# Microcode.  48-bit word: op[47:44] d[43:37] a[36:30] c[29:23] sh[22:18] b[17:0]  (128 registers)
#   b = a register index (MAC / MACE), an 18-bit signed immediate (MACI / MACEI), a table (TAB), a jump target;
#   LDI imm = [31:0].
#   MAC  : r[d] = r[c] + ((r[a] * sext21(r[b])) >> sh)        (the product must fit 44 bits)
#   MACI : r[d] = r[c] + ((r[a] * imm18) >> sh)
#   MACE / MACEI : as MAC / MACI with sh = the e register
#   LDI  : r[d] = imm                    TAB : r[d] = sext16(TAB[b * 256 + (r[a] & 255)])
#   MIN / MAX : r[d] = min / max(r[a], r[c])      OUT : port[d] <= r[a]      IN : r[d] = input[a]
#   SETE : e <= r[a] & 31                WAIT : wait for the line event      JMP / JNZ (r[a] /= 0)
#   ADD  : r[d] = r[c] + r[a]            SUB  : r[d] = r[c] - r[a]      (single-cycle, 32-bit wrap)
# --------------------------------------------------------------------------------------------------------------
OPS = dict(SUB=0, MAC=1, MACI=2, LDI=3, TAB=4, MIN=5, MAX=6, OUT=7, IN=8, SETE=9, MACE=10, MACEI=11, WAIT=12,
           JMP=13, JNZ=14, ADD=15)
T_EXP, T_SIN, T_SQRT = 0, 1, 2
# input sources
I_K1, I_K2, I_K3, I_K4, I_K5, I_K6, I_P12, I_SW, I_SX, I_YS, I_X0 = range(11)
# output ports
PORTS = ['P', 'H', 'HU', 'CR', 'AL',                         # line DDA loads (hblank)
         'D0', 'HS', 'E', 'K2', 'EC', 'HUS', 'CRS', 'ALS',     # per-field
         'T0', 'T1S', 'CAPL', 'WM', 'FE', 'WE', 'CENV', 'GO']
PORT = {n: i for i, n in enumerate(PORTS)}


class Asm:
    def __init__(self):
        self.code = []; self.regs = {}; self.labels = {}; self.consts = {}

    def r(self, name):
        if name not in self.regs:
            self.regs[name] = len(self.regs)
            assert len(self.regs) <= 128, 'out of registers'
        return self.regs[name]

    def emit(self, op, d=0, a=0, b=0, c=0, sh=0, imm=None, tgt=None):
        self.code.append(dict(op=op, d=d, a=a, b=b, c=c, sh=sh, imm=imm, tgt=tgt))

    def label(self, n): self.labels[n] = len(self.code)

    def K(self, v):                                             # a constant register (for MIN / MAX)
        n = 'K%d' % v if v >= 0 else 'Km%d' % (-v)
        if n not in self.consts: self.consts[n] = v
        return n

    # instructions (register operands by name; b may be an int = immediate)
    def MAC(self, d, a, b, c='ZERO', sh=0):
        assert 0 <= sh < 32
        if isinstance(b, int):
            assert -(1 << 17) <= b < (1 << 17), b
            self.emit('MACI', self.r(d), self.r(a), b, self.r(c), sh)
        else:
            self.emit('MAC', self.r(d), self.r(a), self.r(b), self.r(c), sh)
    def MACE(self, d, a, b, c='ZERO'):
        if isinstance(b, int): self.emit('MACEI', self.r(d), self.r(a), b, self.r(c))
        else: self.emit('MACE', self.r(d), self.r(a), self.r(b), self.r(c))
    def LDI(self, d, v): self.emit('LDI', self.r(d), imm=int(v))
    def TAB(self, d, a, t): self.emit('TAB', self.r(d), self.r(a), t)
    def MIN(self, d, a, c): self.emit('MIN', self.r(d), self.r(a), 0, self.r(c))
    def MAX(self, d, a, c): self.emit('MAX', self.r(d), self.r(a), 0, self.r(c))
    def OUT(self, port, a): self.emit('OUT', PORT[port], self.r(a))
    def IN(self, d, src): self.emit('IN', self.r(d), src)
    def SETE(self, a): self.emit('SETE', 0, self.r(a))
    def WAIT(self): self.emit('WAIT')
    def JMP(self, l): self.emit('JMP', tgt=l)
    def JNZ(self, a, l): self.emit('JNZ', 0, self.r(a), tgt=l)

    def ADD(self, d, a, c): self.emit('ADD', self.r(d), self.r(a), 0, self.r(c))     # d = c + a
    def SUB(self, d, c, a): self.emit('SUB', self.r(d), self.r(a), 0, self.r(c))     # d = c - a
    # macros
    def ADDI(self, d, a, k):                                    # d = a + k (k from a constant register)
        if k == 1: self.ADD(d, a, 'ONE')
        elif k == -1: self.ADD(d, a, 'M1')
        else: self.ADD(d, a, self.K(k))
    def SHR(self, d, a, sh): self.MAC(d, a, 1, 'ZERO', sh)      # d = a >> sh
    def MUL(self, d, a, b, sh=0): self.MAC(d, a, b, 'ZERO', sh)

    def sinq(self, d, x):
        """d = 32767 sin(2 pi x / 65536), x in Q16 turns (any integer), linear interpolation"""
        self.SHR('_si', x, 8)
        self.MAC('_sf', '_si', -256, x)                         # frac = x - (x >> 8) * 256
        self.TAB('_s0', '_si', T_SIN)
        self.ADDI('_si', '_si', 1)
        self.TAB('_s1', '_si', T_SIN)
        self.SUB('_sd', '_s1', '_s0')
        self.MAC(d, '_sd', '_sf', '_s0', 8)

    def bitof(self, d, x, k):
        """d = bit k of x (x >= 0)"""
        self.SHR('_bt', x, k)
        self.SHR('_bu', '_bt', 1)
        self.MAC(d, '_bu', -2, '_bt')

    def pow2s(self, d, n, nmax):
        """d = 2^n for n in 0..nmax (nmax <= 3) without a shifter"""
        self.MIN('_p1', n, 'ONE'); self.ADDI(d, '_p1', 1)
        if nmax >= 2:
            self.ADDI('_p2', n, -1); self.MAX('_p2', '_p2', 'ZERO')
            if nmax >= 3: self.MIN('_p2', '_p2', 'ONE')
            self.MAC(d, '_p2', 2, d)
        if nmax >= 3:
            self.ADDI('_p3', n, -2); self.MAX('_p3', '_p3', 'ZERO')
            self.MAC(d, '_p3', 4, d)

    def recip(self, r, x, xs, rs, iters):
        """Newton: r ~ 2^(xs + rs) / x; r holds the seed.  t = x r >> xs (~ 2^rs), r = (2^(rs+1) - t) r >> rs"""
        for _ in range(iters):
            self.MUL('_rt', x, r, xs)
            self.SUB('_rt', 'TWO%d' % rs, '_rt')
            self.MUL(r, '_rt', r, rs)

    def assemble(self):
        words = []
        for i, ins in enumerate(self.code):
            op = OPS[ins['op']]
            w = op << 44 | ins['d'] << 37
            if ins['op'] == 'LDI':
                w |= ins['imm'] & 0xFFFFFFFF
            elif ins['op'] in ('JMP', 'JNZ'):
                w |= ins['a'] << 30 | self.labels[ins['tgt']]
            else:
                w |= ins['a'] << 30 | ins['c'] << 23 | ins['sh'] << 18 | (ins['b'] & 0x3FFFF)
            words.append(w)
        assert len(words) <= 512, 'program too long: %d' % len(words)
        return words


def program():
    B = Asm()
    for n in ['ZERO', 'ONE', 'M1']: B.r(n)
    # ---------------- per field ----------------
    for src, n in [(I_K1, 'k1'), (I_K2, 'k2'), (I_K3, 'k3'), (I_K4, 'k4'), (I_K6, 'k6'), (I_P12, 'p12'),
                   (I_SW, 'sw'), (I_SX, 'sx'), (I_YS, 'ys'), (I_X0, 'x0')]:
        B.IN(n, src)
    B.bitof('rings', 'sw', 0); B.bitof('burst', 'sw', 1); B.bitof('osc', 'sw', 4)
    # LFOs (persistent): heading sway 23 s, Mach breathe 9.7 s (at 60 fields/s)
    B.ADDI('lfo1', 'lfo1', 47); B.ADDI('lfo2', 'lfo2', 113)
    # Mach (Q12): three slopes through (0, 0) (409, 0.5) (665, 1.0) (1023, 2.2), times (1 + 0.05 sin)
    B.MUL('M', 'p12', 20510, 12)
    B.ADDI('_t', 'p12', -409); B.MAX('_t', '_t', 'ZERO'); B.MAC('M', '_t', 12258, 'M', 12)
    B.ADDI('_t', 'p12', -665); B.MAX('_t', '_t', 'ZERO'); B.MAC('M', '_t', 23470, 'M', 12)
    B.sinq('_s', 'lfo2'); B.MUL('_t', 'M', '_s', 15); B.MAC('M', '_t', 1638, 'M', 15)
    # heading (Q16 turns) = k2 * 64 + 14 deg * sin; dx, dy Q15
    B.sinq('_s', 'lfo1'); B.MUL('th', '_s', 2549, 15); B.MAC('th', 'k2', 64, 'th')
    B.sinq('dy', 'th'); B.ADDI('th', 'th', 16384); B.sinq('dx', 'th')
    # wavelength L (Q4 h) = 80 h * 2^(3 k1 / 1023)
    B.MUL('_o', 'k1', 3, 2); B.SHR('_n1', '_o', 8); B.MAC('_f1', '_n1', -256, '_o')
    B.TAB('_m', '_f1', T_EXP); B.pow2s('_p', '_n1', 2); B.MUL('_m', '_m', '_p'); B.MUL('L', '_m', 1280, 14)
    # zoom above Mach 1: z = 1 / (1 + 0.6 (M - 1)) (Q14), Newton
    B.ADDI('_u', 'M', -4096); B.MAX('_u', '_u', 'ZERO'); B.MAC('zz', '_u', 2458, 'ZERO', 12); B.ADDI('zz', 'zz', 4096)
    B.MUL('z', 'zz', -2); B.ADDI('z', 'z', 24576); B.recip('z', 'zz', 12, 14, 2)      # seed 1.5 - zz / 2
    B.MUL('L', 'L', 'z', 14)
    # line half-width w (Q4 h) = 2 h * 2^(4 k4 / 1023)
    B.SHR('_n', 'k4', 8); B.MAC('_f', '_n', -256, 'k4'); B.TAB('_m', '_f', T_EXP)
    B.pow2s('_p', '_n', 3); B.MUL('w', '_m', '_p', 9)
    B.MUL('_t', 'w', 3); B.ADDI('_t', '_t', 96)                                          # we = 3 w + 96 (Q4 h)
    B.ADDI('_u', '_t', -600); B.MAX('_u', '_u', 'ZERO'); B.MIN('_u', '_u', 'ONE')        # we >= 600 ?
    B.MAC('iwe', '_u', -1668, B.K(2621)); B.recip('iwe', '_t', 0, 20, 4)                  # 2^20 / we, 2-piece seed
    B.ADDI('_t', 'rings', 1); B.MUL('w', 'w', '_t')                                       # Rings: twice as wide
    # T1 = 2 L w (h^2); e = floor(log2 T1) - 4 (binary search); T1s = T1 >> e in [16, 32)
    B.MUL('T1', 'L', 'w', 7)
    B.LDI('e', 0)
    for bb in (8, 4, 2, 1):
        B.ADDI('_k', 'e', bb + 4); B.SETE('_k'); B.MACE('_t', 'T1', 1)
        B.MIN('_t', '_t', 'ONE'); B.MAC('e', '_t', bb, 'e')
    B.SETE('e'); B.MACE('T1s', 'T1', 1)
    # ring phase: rate = sign(v) 3 a^2 rings/s, a = (|v| - 8) / 503, v = k3 - 512 (Q16 per field at 60 Hz)
    B.ADDI('_v', 'k3', -512); B.MUL('_t', '_v', -1); B.MAX('_a', '_v', '_t')
    B.ADDI('_a', '_a', -8); B.MAX('_a', '_a', 'ZERO'); B.MUL('_a', '_a', '_a'); B.MUL('_a', '_a', 849, 16)
    B.MIN('_s', '_v', 'ONE'); B.MAX('_s', '_s', 'M1'); B.MUL('_a', '_a', '_s')
    B.ADD('ph', '_a', 'ph')
    B.SHR('W', 'ph', 16); B.MAC('f', 'W', -65536, 'ph')
    B.SHR('_wb', 'ph', 18); B.MAC('_fb', '_wb', B.K(-262144), 'ph'); B.SHR('_fb', '_fb', 2)
    B.SUB('_t', '_fb', 'f'); B.MAC('_fe', '_t', 'burst', 'f'); B.SHR('_fe', '_fe', 11)
    B.SUB('_t', '_wb', 'W'); B.MAC('_we', '_t', 'burst', 'W')
    B.OUT('FE', '_fe'); B.OUT('WE', '_we'); B.OUT('WM', 'W')
    B.MUL('_t', 'f', 'T1s', 16); B.OUT('T0', '_t'); B.OUT('T1S', 'T1s')
    B.SHR('_t', 'f', 8); B.SUB('_t', B.K(256), '_t'); B.MUL('_t', '_t', 15, 8); B.MIN('_t', '_t', B.K(15))
    B.OUT('CAPL', '_t')                                            # the oldest ring fades out over its last wrap
    # source velocity d = M L (dx, dy), Q4 h
    B.MUL('ML', 'M', 'L', 12); B.MUL('dvx', 'ML', 'dx', 15); B.MUL('dvy', 'ML', 'dy', 15)
    # K = (M^2 - 1) L^2, Q6 h^2
    B.MUL('_t', 'M', 'M', 12); B.ADDI('_t', '_t', -4096); B.MUL('_l2', 'L', 'L'); B.MUL('Kq', '_l2', '_t', 14)
    # source position (h): S = centre + min(8 M L, 760 px | 400 px) (dx, dy)
    B.SHR('_o', 'ML', 1); B.MIN('_t', '_o', B.K(1520)); B.MUL('_t', '_t', 'dx', 15); B.ADDI('Sx', '_t', 1920)
    B.MIN('_t', '_o', B.K(800)); B.MUL('_t', '_t', 'dy', 15); B.ADDI('Sy', '_t', 1080)
    B.SUB('qx0', 'x0', 'Sx')
    # P line coefficients: P0(qy) = cP + ((qy + aP) qy) >> 8
    B.MUL('aP', 'f', 'dvy', 12)
    B.MUL('cP', 'qx0', 'qx0', 0); B.MUL('cP', 'cP', 64)
    B.MUL('_fx', 'f', 'dvx', 10)                                   # f dvx, Q10 h
    B.MUL('_t', '_fx', 'qx0', 3); B.ADD('cP', '_t', 'cP')
    B.MUL('_ff', 'f', 'f', 16); B.MUL('_t', 'Kq', '_ff', 16); B.ADD('cP', '_t', 'cP')
    # D0 = (2 sx qx0 + sx^2) 64 + 2 f dvx sx  (Q6)
    B.MUL('_t', 'qx0', 'sx'); B.MAC('_t', 'qx0', 'sx', '_t'); B.MAC('_t', 'sx', 'sx', '_t'); B.MUL('D0', '_t', 64)
    B.MUL('_t', '_fx', 'sx', 3); B.ADD('D0', '_t', 'D0'); B.OUT('D0', 'D0')
    # H = 2 d.q + (2 f + 1) K:  H0(qy) = cH + (qy aH) >> 5, Hs = 8 dvx sx
    B.MUL('aH', 'dvy', 2)
    B.MUL('cH', 'dvx', 'qx0', 0); B.MUL('cH', 'cH', 8)
    B.MUL('_t', 'Kq', 'f', 15); B.ADD('_t', 'Kq', '_t'); B.ADD('cH', '_t', 'cH')
    B.MUL('_t', 'dvx', 'sx'); B.MUL('_t', '_t', 8); B.OUT('HS', '_t')
    # chain (units of 2^e h^2): K2 = 2K >> (e + 6) = K >> (e + 5)
    B.ADDI('_k', 'e', 5); B.SETE('_k'); B.MACE('_t', 'Kq', 1); B.OUT('K2', '_t')
    B.ADDI('_k', 'e', 6); B.OUT('EC', '_k')                        # the P / H barrel shift (Q6 -> chain)
    # camera: dent = 80 px * k6 / 1023 for full luma (219), Osc 2.5x: J += (l E) >> 8, E = 2 L dent / 2^e (Q8)
    B.MUL('_t', 'L', 'k6'); B.MUL('_t', '_t', 1504, 16)
    B.MUL('_u', '_t', 3, 1); B.MAC('_t', '_u', 'osc', '_t')
    B.SETE('e'); B.MACE('_t', '_t', 1); B.OUT('E', '_t')
    # hue: u_lin = M (dir . q) / (4 L):  kh (Q24 per h) = M invL >> 10, invL = 2^24 / L (Newton, seeded from k1)
    B.SUB('_t', B.K(255), '_f1'); B.TAB('_m', '_t', T_EXP)                               # 2^((255 - f1)/256) Q14
    B.MIN('_a', '_n1', 'ONE'); B.MUL('_a', '_a', -2); B.ADDI('_q', '_a', 4)
    B.ADDI('_a', '_n1', -1); B.MAX('_a', '_a', 'ZERO'); B.SUB('_q', '_q', '_a')       # 4 / 2^n1
    B.MUL('invL', '_m', '_q'); B.MUL('invL', 'invL', 13107, 17); B.MUL('invL', 'invL', 'zz', 12)
    B.recip('invL', 'L', 10, 14, 2)
    B.MUL('kh', 'M', 'invL', 10)
    B.MUL('_t', 'kh', 'dx', 15); B.MUL('_t', '_t', 'sx', 8); B.OUT('HUS', '_t')          # Q16 u per pixel
    B.MUL('chu', 'kh', 'dx', 15); B.MUL('chu', 'chu', 'qx0', 8)                         # Q16
    B.MUL('ahu', 'kh', 'dy', 15)                                                          # (qy ahu) >> 15 = Q16
    # envelope (Mach cone): e' = (|cross| ca - along sa) 64 / we (Q8); cross = dy qx - dx qy, along = -(dx qx + dy qy)
    B.MAX('_t', 'M', B.K(4096)); B.MUL('sa', '_t', -7094, 12); B.ADDI('sa', 'sa', 22528)  # seed 1.375 - 0.433 M
    B.recip('sa', '_t', 12, 14, 3)                                                          # 1 / M (Q14), M >= 1
    B.MUL('_t', 'sa', 'sa', 14); B.MUL('_t', '_t', -1); B.ADDI('_t', '_t', 16384); B.MUL('_t', '_t', 255)
    B.SHR('_u', '_t', 14); B.MIN('_u', '_u', B.K(254)); B.MAC('_fr', '_u', -16384, '_t'); B.SHR('_fr', '_fr', 6)
    B.TAB('_s0', '_u', T_SQRT); B.ADDI('_u', '_u', 1); B.TAB('_s1', '_u', T_SQRT)
    B.SUB('_t', '_s1', '_s0'); B.MAC('ca', '_t', '_fr', '_s0', 8)                     # Q15
    B.MUL('gca', 'ca', 'iwe', 15); B.MUL('gsa', 'sa', 'iwe', 14)                       # e' = (x gca) >> 2 per h
    B.MUL('_t', 'gca', 'dy', 15); B.MUL('_t', '_t', 'sx', 2); B.OUT('CRS', '_t')
    B.MUL('_t', 'gsa', 'dx', 15); B.MUL('_t', '_t', 'sx', 2); B.MUL('_t', '_t', -1); B.OUT('ALS', '_t')
    B.MUL('ccr', 'gca', 'dy', 15); B.MUL('ccr', 'ccr', 'qx0', 2)
    B.MUL('acr', 'gca', 'dx', 15); B.MUL('acr', 'acr', -1)                              # (qy acr) >> 9
    B.MUL('cal', 'gsa', 'dx', 15); B.MUL('cal', 'cal', 'qx0', 2); B.MUL('cal', 'cal', -1)
    B.MUL('aal', 'gsa', 'dy', 15); B.MUL('aal', 'aal', -1)
    B.ADDI('_t', 'M', -4096); B.MUL('_t', '_t', 15, 10); B.MAX('_t', '_t', 'ZERO'); B.MIN('_t', '_t', B.K(15))
    B.OUT('CENV', '_t')
    B.OUT('GO', 'ONE')
    # ---------------- line 0 (now, in vblank), then one line ahead of the beam ----------------
    B.MUL('qy', 'Sy', -128)
    def line_vals(B):
        B.ADD('_t', 'aP', 'qy'); B.MUL('_t', '_t', 'qy', 8); B.ADD('nP', '_t', 'cP')
        B.MUL('_t', 'aH', 'qy', 5); B.ADD('nH', '_t', 'cH')
        B.MUL('_t', 'ahu', 'qy', 15); B.ADD('nHU', '_t', 'chu')
        B.MUL('_t', 'acr', 'qy', 9); B.ADD('nCR', '_t', 'ccr')
        B.MUL('_t', 'aal', 'qy', 9); B.ADD('nAL', '_t', 'cal')
    def outs(B):
        for p in ['P', 'H', 'HU', 'CR', 'AL']: B.OUT(p, 'n' + p)
    line_vals(B); outs(B)
    B.label('L_COMP')
    B.ADD('qy', 'ys', 'qy'); line_vals(B)
    B.WAIT(); outs(B)
    B.JMP('L_COMP')
    # constants first
    pre = []
    A = Asm(); A.regs = B.regs
    A.LDI('ZERO', 0); A.LDI('ONE', 1); A.LDI('M1', -1)
    for n, v in B.consts.items(): A.LDI(n, v)
    for rs in (14, 20): A.LDI('TWO%d' % rs, 2 << rs)
    off = len(A.code)
    A.code += B.code
    for l, p in B.labels.items(): A.labels[l] = p + off
    return A


def vhd_pkg(path):
    A = program()
    words = A.assemble()
    words += [0] * (512 - len(words))
    tab = tab_rom(); col = col_rom(); ht = htab(); fd = fade_tab()
    L = []
    L.append('-- doppler_pkg.vhd: generated by doppler.py (do not edit)')
    L.append('library ieee;\nuse ieee.std_logic_1164.all;\nuse ieee.numeric_std.all;\n')
    L.append('package doppler_pkg is')
    L.append('    type t_dp_irom is array (0 to 511) of std_logic_vector(47 downto 0);')
    L.append('    type t_dp_tab is array (0 to 255) of std_logic_vector(15 downto 0);')
    L.append('    type t_dp_col is array (0 to 1023) of std_logic_vector(23 downto 0);')
    L.append('    type t_dp_ht is array (0 to 127) of unsigned(7 downto 0);')
    L.append('    type t_dp_fd is array (0 to 127) of unsigned(3 downto 0);')
    L.append('    type t_dp_rl is array (0 to 63) of unsigned(6 downto 0);')
    L.append('    constant C_DP_NINS : integer := %d;' % len(A.code))
    L.append('    constant C_DP_IROM : t_dp_irom := (')
    L.append(',\n'.join('        x"%012X"' % w for w in words) + ');')
    for nm, base in (('EXP', 0), ('SIN', 256), ('SQRT', 512)):
        L.append('    constant C_DP_%s : t_dp_tab := (' % nm)
        L.append(',\n'.join('        x"%04X"' % (v & 0xFFFF) for v in tab[base:base + 256]) + ');')
    L.append('    constant C_DP_COL : t_dp_col := (')
    L.append(',\n'.join('        x"%02X%02X%02X"' % c for c in col) + ');')
    L.append('    constant C_DP_HTAB : t_dp_ht := (' + ', '.join('to_unsigned(%d, 8)' % v for v in ht) + ');')
    L.append('    constant C_DP_RLEV : t_dp_rl := (' + ', '.join('to_unsigned(%d, 7)' % (b * 4 + wl) for b, wl in rlev_tab()) + ');')
    L.append('    constant C_DP_FADE : t_dp_fd := (' + ', '.join('to_unsigned(%d, 4)' % v for v in fd) + ');')
    L.append('end package doppler_pkg;')
    open(path, 'w').write('\n'.join(L) + '\n')
    return A


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    A = vhd_pkg(os.path.join(here, 'doppler_pkg.vhd'))
    print('doppler_pkg.vhd: %d instructions, %d registers' % (len(A.code), len(A.regs)))
