#!/usr/bin/env python3
"""IRIS micro-sequencer (v0.4): assembler, program, emulator, VHDL emitter.

ISA (16-bit word): op(15:11) arg(10:0): reg = arg(6:0), imm = arg signed, jump target = arg(10:0).
Accumulator machine: 32-bit signed acc, 128 x 32-bit register file (EBR), two-deep call stack.
Four clocks per instruction (fetch / register read / special-operand decode / execute).
"""
import math, sys

OPS = dict(LINE0=0, LD=1, ST=2, ADD=3, SUB=4, LDI=5, ADDI=6, SHR=7, SHL=8, MUL=9,
           LDP=10, DIV=11, SHRV=12, NEG=13, ABS=14, JMP=15, JZ=16, JN=17, JNZ=18,
           JP=19, TW=21, ROM=22, WAITF=23, LDQ=24, SHLV=25, LDF=26,
           LDX=27, STX=28, AND=29, CALL=30, RET=31)

# special inputs (LDX)
X = dict(K0=0, K1=1, K2=2, K3=3, K4=4, K5=5, SLIDER=6, ACTW=7, ACTH=8, VSTEP=9, FLAGS=10)
# special outputs (STX)
Y = dict(XA0=0, YA0=1, XB0=2, YB0=3, XASTEP=4, YASTEP=5, XBSTEP=6, YBSTEP=7,
         XBLSTEP=8, YBLSTEP=9, LP=10, CDOT=11, UP=12, INVN=13, INVE=14, N=15,
         T=16, PH=17, FADE=18, CA=19, TWADDR=24, TWD16=25, FL=26)
# tables (TW)
TW = dict(T1=1, T2=2, TM=3, TV=4, TS=5, TK=6)

# ---------------------------------------------------------------- registers
REGS = """g_kx g_ky g_ks g_ka g_kr g_kt g_sp g_fb ph a_cur n_cur
af t0 t1 t2 cnt hq cnt2 cx cy rfr rfb R f p pref lim tx ty d px py ex ey
qv ax ay am nd np eps sx sy
t1x t1y dl lg inv idx r0 r1 lo fv tv
g13 kr ka k k2 lgk ktd T x2 i sp step
C90 C45 C120 C19899 C3932 C39322 C2400 FILL M1 M2 M4 M8 M255 M4095
C8192 C32768 C4096 M65535 SPQ SPL C42598 C49152 k2m sk rk M511 M2047 C65536 C196608 K94560 K145162 C32767 CM32768 C2048 C131072 C524288 C1M C64M C2G sq_n sq_x sq_t
invn invo cs lgx lge nx sl cur la vst
lcg sct sxt syt sox soy hst hsc C69069""".split()
R = {n: i for i, n in enumerate(REGS)}
assert len(REGS) <= 128, len(REGS)

# ---------------------------------------------------------------- assembler
class Asm:
    def __init__(self):
        self.code = []      # (op, reg, imm, label_ref)
        self.labels = {}
    def L(self, name):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)
    def emit(self, op, reg=0, imm=0, ref=None):
        self.code.append([OPS[op], reg, imm, ref])
    def __getattr__(self, op):
        opu = op.upper()
        if opu not in OPS:
            raise AttributeError(op)
        def f(arg=None):
            if opu in ('LD', 'ST', 'ADD', 'SUB', 'MUL', 'DIV', 'AND', 'SHRV', 'SHLV'):
                self.emit(opu, R[arg])
            elif opu in ('JMP', 'JZ', 'JN', 'JNZ', 'JP', 'CALL'):
                self.emit(opu, 0, 0, arg)
            elif opu == 'LDI' and not (-1024 <= arg <= 1023):
                self.ldc(arg)
            elif opu == 'ADDI' and not (-1024 <= arg <= 1023):
                v = arg
                while v:
                    c = max(-1024, min(1023, v)); self.emit('ADDI', 0, c); v -= c
            elif opu in ('LDI', 'ADDI', 'SHR', 'SHL', 'LDP'):
                assert -1024 <= arg <= 1023, (opu, arg)
                assert opu != 'LDP' or arg in (0, 16), 'LDP decodes only 0 and 16'
                self.emit(opu, 0, arg)
            elif opu == 'TW':
                self.emit(opu, 0, TW[arg])
            elif opu == 'LDX':
                self.emit(opu, 0, X[arg])
            elif opu == 'STX':
                self.emit(opu, 0, Y[arg])
            else:
                self.emit(opu)
        return f
    def ldc(self, val):
        """load an arbitrary 32-bit constant into acc"""
        if -1024 <= val <= 1023:
            self.LDI(val); return
        neg = val < 0
        v = -val if neg else val
        chunks = []
        while v:
            chunks.append(v & 0x3FF); v >>= 10
        chunks.reverse()
        first = True
        for c in chunks:
            if first:
                self.LDI(c); first = False
            else:
                self.SHL(10)
                if c: self.ADDI(c)
        if neg: self.NEG()
    def words(self):
        out = []
        for op, reg, imm, ref in self.code:
            if ref is not None:
                imm = self.labels[ref]
            if reg: w = (op << 11) | (reg & 0x7F)
            else:   w = (op << 11) | (imm & 0x7FF)
            out.append(w)
        return out

# ---------------------------------------------------------------- program
FLOW_RATE = 28          # Q12 ring per frame
FILL_Q16 = 60293        # 0.92

def build():
    a = Asm()
    # the glide registers start from the TOML defaults, preloaded in RFINIT
    a.L('frame')
    a.WAITF()
    # ---- glides
    # arms and rings are integers behind a hysteresis band, so they reject pot noise
    # without a glide; the rest are glided (about 8 frames, first order)
    for k, g in [('K0', 'g_kx'), ('K1', 'g_ky'), ('K2', 'g_ks'), ('K5', 'g_kt'), ('SLIDER', 'g_sp')]:
        a.LDX(k); a.SHL(6); a.SUB(g); a.SHR(3); a.ADD(g); a.ST(g)
    for k, g in [('K3', 'g_ka'), ('K4', 'g_kr')]:
        a.LDX(k); a.SHL(6); a.ST(g)
    # bleed glide (Q12 of 1.0 = 4096): a LINEAR ramp of 256 per frame, 16 frames end to
    # end.  R itself is then formed geometrically (see below), so the dolly runs at a
    # constant zoom rate; a first-order ease on R spends its last frames creeping, and
    # the rim ring -- which only enters the screen in the last few percent -- appeared
    # to pop in long after the resize looked finished.
    a.LDX('FLAGS'); a.AND('M2'); a.JZ('fb0'); a.LD('C4096'); a.JMP('fb1')
    a.L('fb0'); a.LDI(0)
    a.L('fb1'); a.SUB('g_fb'); a.ST('t0')
    a.ADDI(-256); a.JN('fb2'); a.LDI(256); a.ST('t0'); a.JMP('fb3')
    a.L('fb2'); a.LD('t0'); a.ADDI(256); a.JP('fb3'); a.LDI(-256); a.ST('t0')
    a.L('fb3'); a.LD('t0'); a.ADD('g_fb'); a.ST('g_fb')
    # flow phase (advance once per frame; interlaced: top field only)
    a.LDX('FLAGS'); a.AND('M1'); a.JZ('phdone')
    a.LDX('FLAGS'); a.AND('M4'); a.JZ('phadv')
    a.LDX('FLAGS'); a.AND('M8'); a.JZ('phdone')
    a.L('phadv'); a.LD('ph'); a.ADDI(FLOW_RATE); a.AND('M4095'); a.ST('ph')
    # living eye (Flow only).  HIPPUS: each time the flow phase wraps (~2.4 s) the pupil
    # picks a new random size within +-12.5 %, and eases there slowly (below), so it
    # breathes irregularly.  SACCADES: after a random 0.7-2.8 s fixation the gaze jumps
    # to a new random offset within +-12.5 % of the pupil travel, and darts there in ~3
    # frames.  Random numbers: LCG x*69069 + 1 (mod 2^32), high bits only.
    a.ADDI(-(FLOW_RATE - 1)); a.JP('ey_h')
    a.LD('lcg'); a.MUL('C69069'); a.LDP(0); a.ADDI(1); a.ST('lcg'); a.SHR(18); a.ST('hst')
    a.L('ey_h'); a.LD('sct'); a.ADDI(-1); a.ST('sct'); a.JP('phdone')
    a.LD('lcg'); a.MUL('C69069'); a.LDP(0); a.ADDI(1); a.ST('lcg'); a.SHR(19); a.ST('sxt')
    a.LD('lcg'); a.SHL(13); a.SHR(19); a.ST('syt')
    a.LD('lcg'); a.SHR(5); a.AND('M255'); a.SHR(1); a.ADDI(40); a.ST('sct')
    a.L('phdone')
    # Still: the eye settles back onto the knobs.  The glides run every field.
    a.LDX('FLAGS'); a.AND('M1'); a.JNZ('ey_live')
    a.LDI(0); a.ST('sxt'); a.ST('syt'); a.ST('hst')
    a.L('ey_live')
    a.LD('sxt'); a.SUB('sox'); a.SHR(1); a.ADD('sox'); a.ST('sox')
    a.LD('syt'); a.SUB('soy'); a.SHR(1); a.ADD('soy'); a.ST('soy')
    a.LD('hst'); a.SUB('hsc'); a.SHR(5); a.ADD('hsc'); a.ST('hsc')
    # ---- arms / rings with hysteresis
    for g, c, base, cur, lab in [('g_ka', 'C90', 1536, 'a_cur', 'a'), ('g_kr', 'C45', 768, 'n_cur', 'n')]:
        a.LD(g); a.SHL(8); a.MUL(c); a.LDP(16); a.ADDI(base); a.ST('af')
        a.LD(cur); a.SHL(8); a.ST('t0')
        a.LD('af'); a.SUB('t0'); a.ST('t1'); a.ADDI(-192); a.JP(lab + '_upd')
        a.LD('t1'); a.ADDI(192); a.JN(lab + '_upd')
        a.JMP(lab + '_keep')
        a.L(lab + '_upd'); a.LD('af'); a.ADDI(128); a.SHR(8); a.ST(cur)
        a.L(lab + '_keep')
    # ---- screen geometry.  The iris is FIXED and CENTRED, so normalise the screen so
    # that the iris is the UNIT circle at the origin.  Then the two limiting points of
    # the coaxal pencil are inverse in that circle, the Moebius map is the disc
    # automorphism w = (z - a)/(1 - conj(a) z), and its denominator is bounded
    # everywhere -- no far focus, no wide accumulators and no probes.
    a.LDX('VSTEP'); a.ST('vst')
    a.LDX('ACTH'); a.MUL('vst'); a.LDP(0); a.ST('hq')
    a.LDX('FLAGS'); a.AND('M4'); a.JZ('hq_ok'); a.LD('hq'); a.SHL(1); a.ST('hq')     # interlaced: act_h is the field height
    a.L('hq_ok')
    a.LDX('ACTW'); a.SHL(3); a.ST('cx')                                              # centre, Q4 square-pixel units
    a.LD('hq'); a.SHR(1); a.ST('cy')
    a.LD('hq'); a.SHL(8); a.MUL('C120'); a.LDP(16); a.ST('rfr')                      # framed: 94% of half height
    a.LD('cx'); a.MUL('cx'); a.LDP(0); a.ST('t0')
    a.LD('cy'); a.MUL('cy'); a.LDP(0); a.ADD('t0'); a.CALL('sqrt'); a.ST('rfb')      # full bleed: out to the corners
    # R = rfr * 2^(g_fb/4096 * log2(rfb/rfr)): equal ratios per frame, so the bleed
    # transition is a steady dolly rather than a fast lunge followed by a crawl.
    a.LD('rfb'); a.SHL(12); a.DIV('rfr'); a.LDQ(); a.CALL('log2'); a.SUB('C49152'); a.ST('t0')
    a.LD('t0'); a.MUL('g_fb'); a.LDP(0); a.SHR(12); a.ST('t0')                       # octaves, Q12
    a.LD('t0'); a.AND('M4095'); a.SHL(4); a.CALL('exp2'); a.ADD('C65536'); a.ST('t1')
    a.LD('rfr'); a.MUL('t1'); a.LDP(16); a.ST('R')
    a.LD('t0'); a.SHR(12); a.JZ('r_ok'); a.LD('R'); a.SHL(1); a.ST('R')
    a.L('r_ok')
    # pupil radius p = R * (262 + 0.6*ks^2) Q16 ; reference pupil 0.06 R
    a.LD('g_ks'); a.MUL('g_ks'); a.LDP(16); a.MUL('C39322'); a.LDP(16); a.ADDI(262); a.ST('f')
    a.LD('R'); a.MUL('f'); a.LDP(16); a.ST('p')
    a.LD('R'); a.MUL('C3932'); a.LDP(16); a.ST('pref')
    # travel limit (the larger of the pupil and the reference pupil must stay inside)
    a.LD('p'); a.SUB('pref'); a.JP('lim_p'); a.LD('pref'); a.JMP('lim_go')
    a.L('lim_p'); a.LD('p')
    a.L('lim_go'); a.ST('t1')
    # ... and then only 65% of that.  Past about 0.6 of the radius the near side of the
    # ring stack is squeezed into a few pixels and goes dark, so the knobs stop short.
    a.LD('R'); a.SHR(6); a.ST('t0'); a.LD('R'); a.SUB('t1'); a.SUB('t0'); a.ADDI(-16)
    a.MUL('C42598'); a.LDP(16); a.ST('lim')
    a.ADDI(-16); a.JP('limok'); a.LDI(16); a.ST('lim')
    a.L('limok')
    # hippus: p *= 1 + hsc (Q16).  Applied after the travel limit so the breathing cannot
    # move the pupil; lim keeps >= 0.15 R of slack beyond the largest pupil, so +12.5 %
    # can never reach the rim.
    a.LD('p'); a.MUL('hsc'); a.LDP(16); a.ADD('p'); a.ST('p')
    # pupil offset t (Q4 px), circularly clamped to the travel limit
    a.LD('g_kx'); a.SUB('C32768'); a.ADD('sox'); a.SHL(1); a.MUL('lim'); a.LDP(16); a.ST('tx')
    a.LD('g_ky'); a.SUB('C32768'); a.ADD('soy'); a.SHL(1); a.MUL('lim'); a.LDP(16); a.ST('ty')
    a.LD('tx'); a.MUL('tx'); a.LDP(0); a.ST('t0')
    a.LD('ty'); a.MUL('ty'); a.LDP(0); a.ADD('t0'); a.CALL('sqrt'); a.ST('d')
    a.LD('d'); a.ADDI(-1); a.JP('dok')
    a.LDI(1); a.ST('d'); a.ST('tx'); a.LDI(0); a.ST('ty')
    a.L('dok')
    a.LD('lim'); a.SUB('d'); a.JN('clampit'); a.JMP('clamped')
    a.L('clampit'); a.LD('lim'); a.SHL(16); a.DIV('d'); a.LDQ(); a.ST('t0')
    a.LD('tx'); a.MUL('t0'); a.LDP(16); a.ST('tx'); a.LD('ty'); a.MUL('t0'); a.LDP(16); a.ST('ty')
    a.LD('lim'); a.ST('d')
    a.L('clamped')
    # unit vector e = t/d (Q16)
    for t, e, lab in [('tx', 'ex', 'x'), ('ty', 'ey', 'y')]:
        a.LD(t); a.ABS(); a.SHL(16); a.DIV('d'); a.LDQ(); a.ST(e)
        a.LD(t); a.JN(lab + 'neg'); a.JMP(lab + 'ok')
        a.L(lab + 'neg'); a.LD(e); a.NEG(); a.ST(e)
        a.L(lab + 'ok')
    # normalised offset and pupil radius (Q16 of the iris radius)
    a.LD('d'); a.SHL(16); a.DIV('R'); a.LDQ(); a.ST('nd')
    a.LD('p'); a.SHL(16); a.DIV('R'); a.LDQ(); a.ST('np')
    # |a| = 2D / (Q + sqrt(Q^2 - 4 D^2)),  Q = 1 + D^2 - P^2.  The rationalised form has
    # no cancellation as the pupil approaches the centre, where Q -> 1 and the root -> 1.
    a.LD('nd'); a.MUL('nd'); a.LDP(16); a.ST('t1')                                   # D^2
    a.LD('np'); a.MUL('np'); a.LDP(16); a.ST('t2')                                   # P^2
    a.LD('C65536'); a.ADD('t1'); a.SUB('t2'); a.ST('qv')                             # Q
    a.LD('qv'); a.MUL('qv'); a.LDP(16); a.ST('t0')                                   # Q^2
    a.LD('t1'); a.SHL(2); a.ST('t2'); a.LD('t0'); a.SUB('t2'); a.ST('t0')            # Q^2 - 4 D^2 (Q16, >= 0)
    a.JP('disc_ok'); a.LDI(1); a.ST('t0')
    a.L('disc_ok'); a.LD('t0'); a.SHL(12); a.CALL('sqrt'); a.SHL(2); a.ST('t2')      # sqrt in Q16
    a.LD('qv'); a.ADD('t2'); a.SHR(2); a.ST('t2')                                    # (Q + root) in Q14: the divisor is 16 bits
    a.LD('nd'); a.SHL(15); a.DIV('t2'); a.LDQ(); a.ST('am')                          # |a| (Q16)
    a.LD('am'); a.MUL('ex'); a.LDP(16); a.ST('ax')
    a.LD('am'); a.MUL('ey'); a.LDP(16); a.ST('ay')
    # pupil image radius at the reference pupil's outer edge: eps = |(De - a)/(1 - a De)|
    # with De = D + Pref (Pref = 0.06 exactly).  This anchors the ring lattice.
    a.LD('nd'); a.ADD('C3932'); a.CALL('wmag'); a.ST('eps')                          # Q16, < 1
    a.LD('eps'); a.JNZ('eps_ok'); a.LDI(1); a.ST('eps')
    a.L('eps_ok')
    # Lref = -log2(eps) in Q12 octaves (eps is Q16, so log2(eps) = log2(int) - 16)
    a.LD('eps'); a.CALL('log2'); a.SUB('C65536')                                  # log2 of a Q16 value
    a.ABS(); a.ST('dl')                                                              # Lref (Q12, > 0)
    a.ADDI(-64); a.JP('dlok'); a.LDI(64); a.ST('dl')
    a.L('dlok')
    # ring spacing and the normalised rings-per-octave scale
    a.LD('dl'); a.SHL(4); a.DIV('n_cur'); a.LDQ(); a.ST('lg')                        # octaves per ring (Q16)
    a.LD('n_cur'); a.SHL(24); a.DIV('dl'); a.LDQ(); a.ST('invo'); a.SHR(18); a.JZ('invok')
    a.LDI(1); a.SHL(18); a.ADDI(-1); a.ST('invo')
    a.L('invok'); a.LD('invo'); a.ST('inv'); a.LDI(0); a.ST('cnt')
    a.L('iloop'); a.LD('inv'); a.SHR(11); a.JZ('idone')
    a.LD('inv'); a.SHR(1); a.ST('inv'); a.LD('cnt'); a.ADDI(1); a.ST('cnt'); a.JMP('iloop')
    a.L('idone'); a.LD('inv'); a.ST('invn'); a.STX('INVN'); a.LD('cnt'); a.STX('INVE')
    a.LD('invn'); a.SHLV('cnt'); a.ST('invo')                                        # exact normalised INVO
    # engine scan: lane A is (z - a), lane B is (1 - conj(a) z), both affine in the pixel
    # coordinates, so each gets a plain accumulator with constant per-sample and per-line
    # steps.  z = ((px - cx)/R, (py - cy)*asp/R) in Q16 of the iris radius.
    a.LDI(1); a.SHL(23); a.DIV('R'); a.LDQ(); a.ST('sx')                             # 8 px step (Q4 px in, Q16 out)
    a.LD('vst'); a.SHL(16); a.DIV('R'); a.LDQ(); a.ST('sy')                          # one line step, Q16
    # lane A start at pixel 0 of line 0: (-cx/R - ax, -cy*asp/R - ay)
    a.LD('cx'); a.SHL(16); a.DIV('R'); a.LDQ(); a.NEG(); a.SUB('ax'); a.ST('t0'); a.STX('XA0')
    a.LD('cy'); a.SHL(16); a.DIV('R'); a.LDQ(); a.NEG(); a.SUB('ay'); a.ST('t1'); a.STX('YA0')
    a.LD('sx'); a.STX('XASTEP'); a.LD('sy'); a.STX('YASTEP')
    # lane B start: (1 - ax*x0 - ay*y0, ay*x0 - ax*y0) with (x0,y0) the lane-A start plus a
    a.LD('t0'); a.ADD('ax'); a.ST('t1x'); a.LD('t1'); a.ADD('ay'); a.ST('t1y')       # (x0, y0)
    a.LD('ax'); a.MUL('t1x'); a.LDP(16); a.ST('t2')
    a.LD('ay'); a.MUL('t1y'); a.LDP(16); a.ADD('t2'); a.ST('t2')
    a.LD('C65536'); a.SUB('t2'); a.STX('XB0')
    a.LD('ay'); a.MUL('t1x'); a.LDP(16); a.ST('t2')
    a.LD('ax'); a.MUL('t1y'); a.LDP(16); a.ST('t0'); a.LD('t2'); a.SUB('t0'); a.STX('YB0')
    # lane B steps: d/dx = (-ax, +ay) * sx ,  d/dy = (-ay, -ax) * sy
    a.LD('ax'); a.MUL('sx'); a.LDP(16); a.NEG(); a.STX('XBSTEP')
    a.LD('ay'); a.MUL('sx'); a.LDP(16); a.STX('YBSTEP')
    a.LD('ay'); a.MUL('sy'); a.LDP(16); a.NEG(); a.STX('XBLSTEP')
    a.LD('ax'); a.MUL('sy'); a.LDP(16); a.NEG(); a.STX('YBLSTEP')
    # K (dot radius): g = 2^(octaves per ring)
    a.LD('lg'); a.AND('M65535'); a.CALL('exp2'); a.ADD('C65536'); a.ST('g13')
    a.LD('lg'); a.SHR(16); a.JZ('g_sh3'); a.ADDI(-1); a.JZ('g_sh2'); a.LD('g13'); a.SHR(1); a.ST('g13'); a.JMP('g_ok')
    a.L('g_sh2'); a.LD('g13'); a.SHR(2); a.ST('g13'); a.JMP('g_ok')
    a.L('g_sh3'); a.LD('g13'); a.SHR(3); a.ST('g13')
    a.L('g_ok')
    a.LD('g13'); a.ADD('C8192'); a.ST('t0'); a.LD('g13'); a.SUB('C8192'); a.SHL(16); a.DIV('t0'); a.LDQ(); a.ST('kr')
    a.LD('C524288'); a.DIV('a_cur'); a.LDQ(); a.CALL('sin'); a.SHL(1); a.ST('ka')
    a.LD('ka'); a.SUB('kr'); a.JN('kmin_a'); a.LD('kr'); a.JMP('kmin_ok')
    a.L('kmin_a'); a.LD('ka')
    a.L('kmin_ok'); a.MUL('FILL'); a.LDP(16); a.ST('k')
    a.LD('k'); a.MUL('k'); a.LDP(16); a.ST('k2')
    a.LD('k'); a.MUL('k'); a.LDP(0); a.ST('k2m'); a.LDI(0); a.ST('sk')
    a.L('skloop'); a.LD('k2m'); a.SHR(16); a.JZ('skdone')
    a.LD('k2m'); a.SHR(1); a.ST('k2m'); a.LD('sk'); a.ADDI(1); a.ST('sk'); a.JMP('skloop')
    a.L('skdone'); a.LD('C2G'); a.DIV('k2m'); a.LDQ(); a.ST('rk')
    # S7 Mesh: the dots grow to 1.24x (1/sqrt(0.65)) of the touching radius so they
    # overlap and the black between them becomes the figure.  Only the TABLES scale
    # (rk = 1/K^2), not K itself: K also sets the anti-alias normaliser and the
    # resolution fade, and the fade must stay calibrated to the sampler, not the dot.
    a.LDX('FLAGS'); a.SHR(4); a.AND('M1'); a.JZ('rk_ok')
    a.LD('rk'); a.MUL('C42598'); a.LDP(16); a.ST('rk')
    a.L('rk_ok')
    a.LD('k'); a.CALL('log2'); a.SUB('C65536'); a.ST('lgk')                     # log2 K (Q12)
    # lattice anchor: u = ((lg_a - lg_b) - c_lp) << inve, and u = N at the rim where
    # lg_a - lg_b = 0, so c_lp = -(N * 4096 >> inve)
    a.LD('n_cur'); a.SHL(12); a.SHRV('cnt'); a.NEG(); a.STX('LP')
    # pupil edge and softness, in Q12 ring units, with the flow phase folded in
    a.LD('p'); a.SHL(16); a.DIV('R'); a.LDQ(); a.ST('t0')                            # P (Q16)
    a.LD('nd'); a.ADD('t0'); a.CALL('wmag'); a.ST('t1')                              # |w| at the pupil edge
    a.JNZ('up_ok'); a.LDI(1); a.ST('t1')
    a.L('up_ok'); a.LD('t1'); a.CALL('log2'); a.SUB('C65536')        # log2|w| at the pupil edge (Q12)
    a.SHL(4); a.MUL('invo'); a.LDP(16); a.ST('t0')
    a.LD('n_cur'); a.SHL(12); a.ADD('t0'); a.SUB('ph'); a.STX('UP')                  # u at the pupil edge
    # anti-alias normaliser.  |dw/dz| = (1 - |a|^2) / |1 - conj(a) z|^2 per normalised
    # unit, so per PIXEL it is that times 1/R, and the engine only needs lane B's log:
    #   rd = (CDOT - 2*log2 r2) >> 12,  CDOT = (log2(1-|a|^2) - log2 R - log2 K) * 4096 + bias
    a.LD('am'); a.MUL('am'); a.LDP(16); a.ST('t0'); a.LD('C65536'); a.SUB('t0'); a.ST('t0')
    a.LD('t0'); a.CALL('log2'); a.SUB('C65536'); a.ST('t0')                      # log2(1-|a|^2)
    a.LD('R'); a.SHR(4); a.CALL('log2'); a.ST('t1')                                  # log2 R (pixels)
    a.LD('t0'); a.SUB('t1'); a.SUB('lgk'); a.ADD('K145162'); a.STX('CDOT')
    # split: sep_rim (Q11 arm) = 0.50 s^2 + 0.17 s (max 0.67 arm at the rim) ; cs = sep_rim / N
    # (Q11 arm per ring).  The TS table CLAMPS the separation at 1/3 arm: there red (+1/3),
    # green (0) and blue (-1/3) sit evenly between each other's dots -- a clean RGB triad,
    # three times the dot count.  Past 1/3 the layers would close in on the NEIGHBOURING
    # dots (red and blue merge at 1/2) and dissolve into confetti, so instead the triad band
    # grows inward from the rim: at 100 % the outer half of the iris is pure triad.
    # split-is-zero flag for the pixel path (red and blue become copies of green there):
    # taken from the GLIDED slider so it engages as the glide lands, never with a jump
    a.LD('g_sp'); a.SUB('M2047'); a.JN('sp_z'); a.LDI(0); a.JMP('sp_f')     # under 3.1 %: sub-pixel, and clear of ADC noise
    a.L('sp_z'); a.LDI(1)
    a.L('sp_f'); a.STX('FL')
    a.LD('g_sp'); a.MUL('g_sp'); a.LDP(16); a.MUL('SPQ'); a.LDP(16); a.ST('t0')
    a.LD('g_sp'); a.MUL('SPL'); a.LDP(16); a.ADD('t0'); a.DIV('n_cur'); a.LDQ(); a.ST('cs')
    # twist: dead zone, T = tan(psi) A Lg ln2/2pi (Q12), knob right = clockwise
    a.LD('g_kt'); a.SUB('C32768'); a.ST('ktd')
    a.ADDI(-2047); a.JP('kt_pos'); a.LD('ktd'); a.ADDI(2047); a.JN('kt_neg'); a.LDI(0); a.ST('ktd'); a.JMP('kt_ok')
    a.L('kt_pos'); a.ST('ktd'); a.JMP('kt_ok')
    a.L('kt_neg'); a.ST('ktd')
    a.L('kt_ok')
    a.LD('ktd'); a.MUL('a_cur'); a.LDP(0); a.MUL('lg'); a.LDP(16); a.MUL('C2400'); a.LDP(16); a.NEG(); a.ST('T')
    a.ADDI(-2048); a.JN('t_hi_ok'); a.LDI(2047); a.ST('T')
    a.L('t_hi_ok'); a.LD('T'); a.ADDI(2047); a.JP('t_lo_ok'); a.LDI(-2047); a.ST('T')
    a.L('t_lo_ok'); a.LD('T'); a.SHR(2); a.STX('T')
    a.LD('T'); a.SHL(4); a.MUL('ph'); a.LDP(16); a.ST('t2')                  # tph = T*ph/4096 (Q12 arm), folded into the twist table
    a.LD('T'); a.SHR(2); a.SHL(2); a.ST('step')                              # 4*(T>>2): per-ring increment of 2*(T>>2)*(2k+1)
    a.LD('step'); a.SHR(1); a.ADD('t2'); a.ST('tv'); a.LDI(0); a.ST('i')        # k = 0: 2*(T>>2) + tph
    a.L('tkloop')
    a.LD('tv'); a.AND('M4095'); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TK')
    a.LD('tv'); a.ADD('step'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-128); a.JN('tkloop')
    a.LD('step'); a.SHL(7); a.NEG(); a.ST('tv'); a.LD('step'); a.SHR(1); a.ADD('tv'); a.ADD('t2'); a.ST('tv')   # k = -128: -510*(T>>2) + tph
    a.L('tkloop2')
    a.LD('tv'); a.AND('M4095'); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TK')
    a.LD('tv'); a.ADD('step'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-256); a.JN('tkloop2')
    a.LD('ph'); a.STX('PH')
    # the outermost ring's fade level: it advances with the flow phase, so the ring dims
    # uniformly as it drifts out and has gone by the time it would cross the rim.  Not
    # gated on Flow -- a phase frozen mid-drift leaves the ring correctly part-faded.
    a.LD('ph'); a.SHR(4); a.STX('FADE')
    a.LD('n_cur'); a.STX('N')
    # ---- TM table (256 entries): scaled log2 mantissa, value(11) = V(i)*invn >> 12, slope(5) = value(i+1) - value(i)
    a.LDI(0); a.ST('i'); a.ST('cur')
    a.L('tmloop')
    a.LD('i'); a.ADDI(257); a.ST('idx'); a.ADDI(-512); a.JZ('tm_top')
    a.LD('idx'); a.ROM(); a.LDF(); a.SHR(4); a.MUL('invn'); a.LDP(0); a.SHR(12); a.JMP('tm_go')
    a.L('tm_top'); a.LD('invn')
    a.L('tm_go'); a.ST('nx'); a.SUB('cur'); a.ST('sl'); a.ADDI(-31); a.JN('tm_sl'); a.LDI(31); a.ST('sl')
    a.L('tm_sl'); a.LD('cur'); a.SHL(5); a.ADD('sl'); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TM')
    a.LD('nx'); a.ST('cur'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-256); a.JN('tmloop')
    # ---- TV table: 0..127 (i*A*32) mod 2^16 ; 128..255 (j*A) >> 2   (theta*A in Q12 arm, 4 int bits)
    a.LD('a_cur'); a.STX('CA')                                                     # arms mod 16 for the engine's cut correction
    a.LD('a_cur'); a.SHL(5); a.ST('step'); a.LDI(0); a.ST('tv'); a.ST('i')
    a.L('tvloop')
    a.LD('tv'); a.AND('M65535'); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TV')
    a.LD('tv'); a.ADD('step'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-128); a.JN('tvloop')
    a.LDI(0); a.ST('tv')
    a.L('tlloop')
    a.LD('tv'); a.SHR(2); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TV')
    a.LD('tv'); a.ADD('a_cur'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-256); a.JN('tlloop')
    # ---- TS table: split(i) = (i * cs) >> 1 (Q12 arm), i = u in Q2 rings, clamped at 1/3 arm (1365)
    a.LDI(0); a.ST('tv'); a.ST('i')
    a.L('tsloop')
    a.LD('tv'); a.SHR(1); a.ST('t0'); a.ADDI(-1365); a.JN('ts_ok'); a.LDI(1365); a.ST('t0')
    a.L('ts_ok'); a.LD('t0'); a.STX('TWD16'); a.LD('i'); a.STX('TWADDR'); a.TW('TS')
    a.LD('tv'); a.ADD('cs'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-256); a.JN('tsloop')
    # ---- T1 table (256 entries): B(du) = (1 - E^2/K2)/ex, du = (i-128)/256 (Q11 of K2)
    a.LDI(0); a.ST('i'); a.LD('lg'); a.SHL(7); a.NEG(); a.ST('x2'); a.LD('lg'); a.ST('step')   # x2 Q24 = (i-128)*lg
    a.LDI(0); a.ST('cnt2'); a.LD('sk'); a.ADDI(-4); a.JP('e2cnt'); a.NEG(); a.ST('cnt2'); a.LDI(0)
    a.L('e2cnt'); a.ST('cnt')                                                  # E2*rk >> (12+sk): pre-shift E2 left by cnt2 = max(0, 4-sk), then >> 16 >> cnt
    a.L('t1loop')
    a.LD('x2'); a.SHR(8); a.AND('M65535'); a.CALL('exp2'); a.ST('t2')          # m = 2^frac - 1, Q16
    a.LD('x2'); a.SHR(24)                                                    # integer part ip (-2..1)
    a.JZ('ex_ok'); a.ADDI(-1); a.JZ('ex_sl1'); a.ADDI(2); a.JZ('ex_sr1')
    a.LD('t2'); a.SUB('C196608'); a.SHR(2); a.ST('t2'); a.JMP('ex_ok')   # ip=-2: (m-3)/4
    a.L('ex_sl1'); a.LD('t2'); a.SHL(1); a.ADD('C65536'); a.ST('t2'); a.JMP('ex_ok')                      # ip=1: 1+2m
    a.L('ex_sr1'); a.LD('t2'); a.SUB('C65536'); a.SHR(1); a.ST('t2')                                     # ip=-1: (m-1)/2
    a.L('ex_ok')                                                              # t2 = E Q16
    a.LD('t2'); a.SHL(8); a.MUL('t2'); a.LDP(16); a.ST('t1')                 # E2 Q24
    a.SHR(26); a.JZ('e2_ok'); a.LD('C64M'); a.ST('t1')                # saturate E2 at 4.0
    a.L('e2_ok'); a.LD('t2'); a.ADD('C65536'); a.SHR(2); a.ST('t2')  # ex Q14 (divisor)
    a.LD('t1'); a.SHLV('cnt2')
    a.MUL('rk'); a.LDP(16); a.SHRV('cnt'); a.ST('t1')                        # E2(Q24)*rk >> (12+sk) -> Q11
    a.LD('C2048'); a.SUB('t1'); a.ST('t0')                              # n = 1 - E2/K2 (Q11, signed)
    a.ABS(); a.SHR(17); a.JZ('n_ok'); a.LD('C131072'); a.ST('t1'); a.JMP('n_div')
    a.L('n_ok'); a.LD('t0'); a.ABS(); a.ST('t1')
    a.L('n_div'); a.LD('t1'); a.SHL(14); a.DIV('t2'); a.LDQ(); a.ST('t1')      # |n|/ex Q11
    a.LD('t0'); a.JN('b_neg'); a.LD('t1'); a.JMP('b_ok')
    a.L('b_neg'); a.LD('t1'); a.NEG()
    a.L('b_ok'); a.ST('t1')
    a.SUB('C32768'); a.JN('b_hi_ok'); a.LD('C32767'); a.ST('t1')
    a.L('b_hi_ok'); a.LD('t1'); a.ADD('C32768'); a.JP('b_lo_ok'); a.LD('CM32768'); a.ST('t1')
    a.L('b_lo_ok'); a.LD('i'); a.STX('TWADDR'); a.LD('t1'); a.STX('TWD16'); a.TW('T1')
    a.LD('x2'); a.ADD('step'); a.ST('x2'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-256); a.JN('t1loop')
    # ---- T2 table: S(dv) = 4 sin^2(pi dv/A), dv = j/512 ; word = S(Q14,13b)<<3 | slope
    a.LD('C1M'); a.DIV('a_cur'); a.LDQ(); a.ST('step'); a.LDI(0); a.ST('tv'); a.ST('i')
    a.L('t2loop')
    a.LD('tv'); a.SHR(10); a.CALL('sin'); a.ST('t0'); a.MUL('t0'); a.LDP(0); a.MUL('rk'); a.LDP(16); a.SHRV('sk'); a.ST('t1')  # sn^2*rk >> 16 >> sk -> S' Q11
    a.SUB('C8192'); a.JN('s_ok'); a.LDI(1); a.SHL(13); a.ADDI(-1); a.ST('t1')                  # saturate 4.0
    a.L('s_ok')
    a.LD('i'); a.JZ('t2_first')
    a.LD('t1'); a.SUB('sp'); a.SHR(3); a.ST('t0'); a.ADDI(-7); a.JN('ss_ok'); a.LDI(7); a.ST('t0')
    a.L('ss_ok'); a.LD('i'); a.ADDI(-1); a.STX('TWADDR'); a.LD('sp'); a.SHL(3); a.ADD('t0'); a.STX('TWD16'); a.TW('T2')
    a.L('t2_first'); a.LD('t1'); a.ST('sp')
    a.LD('tv'); a.ADD('step'); a.ST('tv'); a.LD('i'); a.ADDI(1); a.ST('i'); a.ADDI(-257); a.JN('t2loop')
    a.JMP('frame')

    # ---- subroutines ----
    # wmag: acc = a point on the pupil axis (Q16) -> acc = |w| there (Q16)
    a.L('wmag'); a.ST('t0'); a.SUB('am'); a.ABS(); a.ST('t1')
    a.LD('am'); a.MUL('t0'); a.LDP(16); a.ST('t2'); a.LD('C65536'); a.SUB('t2'); a.SHR(1); a.ST('t2')
    a.LD('t1'); a.SHL(15); a.DIV('t2'); a.LDQ(); a.RET()
    # log2: acc = magnitude (18-bit unsigned) -> acc = msb*4096 + log2 mantissa (Q12), ROM + 9-bit lerp
    a.L('log2'); a.ST('lgx'); a.JNZ('l2nz'); a.LDI(1); a.ST('lgx')
    a.L('l2nz'); a.LDI(17); a.ST('lge')
    a.L('l2n'); a.LD('lgx'); a.SHR(17); a.JNZ('l2ok'); a.LD('lgx'); a.SHL(1); a.ST('lgx'); a.LD('lge'); a.ADDI(-1); a.ST('lge'); a.JMP('l2n')
    a.L('l2ok'); a.LD('lgx'); a.SHR(9); a.AND('M255'); a.ADDI(256); a.ST('idx'); a.ROM(); a.LDF(); a.SHR(4); a.ST('r0')
    a.LD('idx'); a.ADDI(1); a.ST('idx'); a.ADDI(-512); a.JZ('l2top'); a.LD('idx'); a.ROM(); a.LDF(); a.SHR(4); a.ST('r1'); a.JMP('l2l')
    a.L('l2top'); a.LD('C4096'); a.ST('r1')
    a.L('l2l'); a.LD('lgx'); a.AND('M511'); a.ST('lo'); a.LD('r1'); a.SUB('r0'); a.SHL(7); a.MUL('lo'); a.LDP(16); a.ADD('r0'); a.ST('r0')
    a.LD('lge'); a.SHL(12); a.ADD('r0'); a.RET()
    # sqrt: acc = n (unsigned 32) -> acc = sqrt(n) (16 bits), Newton from a power-of-two overestimate
    a.L('sqrt'); a.ST('sq_n'); a.ST('t2'); a.LDI(1); a.ST('sq_x')
    a.L('sq_m'); a.LD('sq_n'); a.SHR(2); a.ST('sq_n'); a.JZ('sq_md'); a.LD('sq_x'); a.SHL(1); a.ST('sq_x'); a.JMP('sq_m')
    a.L('sq_md'); a.LD('sq_x'); a.SHL(1); a.SUB('M65535'); a.JN('sq_g0'); a.LD('M65535'); a.ST('sq_x'); a.JMP('sq_g1')
    a.L('sq_g0'); a.LD('sq_x'); a.SHL(1); a.ST('sq_x')
    a.L('sq_g1'); a.LD('t2'); a.ST('sq_n')
    a.LDI(5); a.ST('sq_t')
    a.L('sq_i'); a.LD('sq_n'); a.DIV('sq_x'); a.LDQ(); a.ADD('sq_x'); a.SHR(1); a.ST('sq_x'); a.LD('sq_t'); a.ADDI(-1); a.ST('sq_t'); a.JNZ('sq_i')
    a.LD('sq_x'); a.RET()
    # exp2: acc = 16-bit fraction f -> acc = 2^(f/65536) - 1, Q16 (128-entry ROM at 128..255, lerp on 9 bits)
    a.L('exp2'); a.ST('fv'); a.SHR(9); a.ADDI(128); a.ST('idx'); a.ROM(); a.LDF(); a.ST('r0')
    a.LD('idx'); a.ADDI(-255); a.JZ('e2_top'); a.LD('idx'); a.ADDI(1); a.ROM(); a.LDF(); a.ST('r1'); a.JMP('e2_lerp')
    a.L('e2_top'); a.LD('C65536'); a.ST('r1')
    a.L('e2_lerp'); a.LD('fv'); a.AND('M511'); a.ST('lo')
    a.LD('r1'); a.SUB('r0'); a.SHL(7); a.MUL('lo'); a.LDP(16); a.ADD('r0'); a.RET()
    # sin: acc = t (turns Q20, 0..1/8) -> acc = sin(2 pi t) Q15 (128-entry quarter-wave ROM, lerp on 11 bits)
    a.L('sin'); a.ST('fv'); a.SHR(11); a.ST('idx'); a.ROM(); a.LDF(); a.ST('r0')
    a.LD('idx'); a.ADDI(1); a.ROM(); a.LDF(); a.ST('r1')
    a.LD('fv'); a.AND('M2047'); a.ST('lo')
    a.LD('r1'); a.SUB('r0'); a.SHL(5); a.MUL('lo'); a.LDP(16); a.ADD('r0'); a.RET()
    return a

# ---------------------------------------------------------------- emulator
class Emu:
    """Functional emulator with a float model of the pixel-pipeline probes."""
    def __init__(self, asm, env):
        self.asm = asm; self.words = asm.words(); self.env = env
        self.rf = list(rf_init()); self.acc = 0; self.pc = 0; self.lnk = [0, 0]
        self.mp = 0; self.dq = 0; self.fun_addr = 0; self.fun_q = 0
        self.out = {}; self.tables = {1: {}, 2: {}, 3: {}, 4: {}, 5: {}, 6: {}}
        self.cycles = 0
        self.frames = 0
    def s32(self, v):
        v &= 0xFFFFFFFF
        return v - (1 << 32) if v & 0x80000000 else v
    def run(self, max_frames=1, max_steps=2_000_000):
        rom = self.env['rom']
        steps = 0
        names_x = {v: k for k, v in X.items()}
        names_y = {v: k for k, v in Y.items()}
        names_o = {v: k for k, v in OPS.items()}
        while steps < max_steps:
            w = self.words[self.pc]
            op = w >> 11; reg = w & 0x7F; imm = w & 0x7FF
            if imm & 0x400: imm -= 2048
            tgt = w & 0x7FF
            self.pc += 1; steps += 1; self.cycles += 4
            rq = self.s32(self.rf[reg])
            o = names_o[op]
            if o == 'LD': self.acc = rq
            elif o == 'ST': self.rf[reg] = self.acc & 0xFFFFFFFF
            elif o == 'ADD': self.acc = self.s32(self.acc + rq)
            elif o == 'SUB': self.acc = self.s32(self.acc - rq)
            elif o == 'AND': self.acc = self.s32(self.acc & rq)
            elif o == 'LDI': self.acc = imm
            elif o == 'ADDI': self.acc = self.s32(self.acc + imm)
            elif o == 'SHR': self.acc = self.acc >> (imm & 63); self.cycles += imm
            elif o == 'SHL': self.acc = self.s32(self.acc << (imm & 63)); self.cycles += imm
            elif o == 'SHRV': self.acc = self.acc >> (rq & 63); self.cycles += rq & 63
            elif o == 'SHLV': self.acc = self.s32(self.acc << (rq & 63)); self.cycles += rq & 63
            elif o == 'MUL':
                b = rq & 0xFFFFF
                if b & 0x80000: b -= 1 << 20
                self.mp = self.acc * b; self.cycles += 21
            elif o == 'LDP': self.acc = self.s32(self.mp >> imm)
            elif o == 'DIV':
                n = self.acc & 0xFFFFFFFF; d = rq & 0xFFFF
                self.dq = (n // d) & 0xFFFFFFFF if d else 0xFFFFFFFF; self.cycles += 33
            elif o == 'LDQ': self.acc = self.s32(self.dq)
            elif o == 'NEG': self.acc = self.s32(-self.acc)
            elif o == 'ABS': self.acc = abs(self.acc)
            elif o == 'JMP': self.pc = tgt
            elif o == 'JZ':
                if self.acc == 0: self.pc = tgt
            elif o == 'JNZ':
                if self.acc != 0: self.pc = tgt
            elif o == 'JN':
                if self.acc < 0: self.pc = tgt
            elif o == 'JP':
                if self.acc > 0: self.pc = tgt
            elif o == 'CALL': self.lnk = [self.pc, self.lnk[0]]; self.pc = tgt
            elif o == 'RET': self.pc = self.lnk[0]; self.lnk = [self.lnk[1], self.lnk[1]]
            elif o == 'TW':
                t = imm
                ad = self.out['TWADDR'] & 511; d = self.out['TWD16'] & 0xFFFF
                self.tables[t][ad & 255] = d
            elif o == 'ROM': self.fun_addr = self.acc & 511
            elif o == 'LDF': self.acc = rom[self.fun_addr]
            elif o == 'WAITF':
                if self.frames >= max_frames: return
                self.frames += 1
            elif o == 'LDX':
                self.acc = self.env[names_x[imm]]
            elif o == 'STX':
                self.out[names_y[imm]] = self.acc
            elif o == 'LINE0': pass
        raise RuntimeError('step limit')

RFINIT = dict(C90=90, C45=45, C120=120, C19899=19899, C3932=3932, C39322=39322,
              C2400=2400, FILL=FILL_Q16, M1=1, M2=2, M4=4, M8=8, M255=255, M4095=4095,
              C8192=8192, C32768=32768, C4096=4096, M65535=65535, SPQ=1017, SPL=349, M511=511, M2047=2047, C65536=65536, C196608=196608, K94560=94560, K145162=171783, C32767=32767, CM32768=-32768, C2048=2048, C42598=42598, C49152=49152, C131072=131072, C524288=524288, C1M=1048576, C64M=67108864, C2G=-2147483648, a_cur=48, n_cur=30, lcg=0x2545F491, sct=60, C69069=69069,
              g_kx=307 * 64, g_ky=563 * 64, g_ks=317 * 64, g_ka=478 * 64, g_kr=614 * 64,
              g_kt=716 * 64, g_sp=325 * 64)

def rf_init():
    return [RFINIT.get(n, 0) for n in REGS] + [0] * (128 - len(REGS))

def rom_contents():
    sn = [min(32767, round(math.sin(2 * math.pi * i / 512) * 32768)) for i in range(128)]
    ex = [min(65535, round((2 ** (i / 128) - 1) * 65536)) for i in range(128)]
    V = [round(math.log2(1 + m / 256) * 4096) for m in range(257)]
    lw = []
    for m in range(256):
        S = (V[m + 1] - V[m]) / 16 * 32
        S4 = max(0, min(15, round((S - 16) / 2)))
        lw.append(V[m] * 16 + S4)
    return sn + ex + lw

def emit_vhdl(words):
    lines = ["    type t_c_prog is array(0 to 1279) of unsigned(15 downto 0);",
             "    constant C_PROG : t_c_prog := ("]
    body = []
    padded = words + [0] * (1280 - len(words))
    for i in range(0, 1280, 8):
        body.append('        ' + ', '.join(f'to_unsigned({v},16)' for v in padded[i:i + 8]))
    lines.append(',\n'.join(body) + ');')
    lines.append("    type t_c_rfinit is array(0 to 127) of integer;")
    lines.append("    constant C_RFINIT : t_c_rfinit := (" + ', '.join(str(v) for v in rf_init()) + ");")
    return '\n'.join(lines)

if __name__ == '__main__':
    a = build()
    words = a.words()
    print(f'program: {len(words)} words', file=sys.stderr)
    assert len(words) <= 1280
    if len(sys.argv) > 1 and sys.argv[1] == 'vhdl':
        print(emit_vhdl(words))
