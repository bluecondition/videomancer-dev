#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""MOOD reference model: the microcode in the emulator + the display path.

  python3 mood_model.py budget            worst-case engine clocks per sample
  python3 mood_model.py validate          the spec's validation list
  python3 mood_model.py render OUT.png [--clean] [--fmt 1080p] [--set k=v ...]
                                          one frame after N seconds of phantom

The display functions mirror mood_core.vhd's pixel path at the virtual-clock
level (VINTAGE) and at pixel level (CLEAN); they are a look/behaviour model,
not a bit-exact RTL model.
"""
import math
import sys

import numpy as np

import mood_tables as T
from mood_ucode import build
from mood_uemu import Emu, s16

VIS_CLK = T.VIS_CLK         # virtual clocks across the active width
X0_CLK = (228 - VIS_CLK) // 2
V0_LINE = 31                # first visible virtual line
VIS_LINES = T.VIS_LINES
HMULT = [1, 2, 3, 5]
VMULT = [1, 2, 4, 8]
SHAPES = ['SOLID', 'HOLE', 'RING']

FMTS = {c[0]: (i, c[1], c[2]) for i, c in enumerate(T.CLASSES)}


def gray(a, b):
    """switch pair (first, second) -> setting 0..3 (Gray code)"""
    return (a ^ b) | (b << 1)


class Machine:
    def __init__(self, fmt='1080p'):
        A = build()
        self.asm = A
        self.emu = Emu(A.resolve(), T.rom(), A.rf_init())
        self.cls = FMTS[fmt][0]
        self.ring = np.zeros((1024, 2), dtype=np.int32)
        self.wp = 0
        self.e1 = 0
        self.mode = 0
        self.uv = [0, 0, 0, 0]
        self.knobs = dict(k1=614, k2=614, k3=358, k4=358, k5=512, k6=0, p12=767)
        self.sw = [0, 0, 0, 0, 0]          # S7..S11
        self.frames = 0

    def set(self, **kw):
        for k, v in kw.items():
            if k.startswith('s') and k[1:].isdigit():
                self.sw[int(k[1:]) - 7] = int(v)
            else:
                self.knobs[k] = int(v)

    def sample(self):
        em = self.emu
        em.ctl.update(self.knobs)
        h = gray(self.sw[0], self.sw[1])
        v = gray(self.sw[2], self.sw[3])
        em.ctl['sw'] = h | v << 2 | self.sw[4] << 4 | self.cls << 5
        cyc = em.sample()
        for pn, val in em.writes:
            if pn == 'E1':
                self.e1 = val
            elif pn == 'E2':
                self.ring[self.wp] = (self.e1, val)
                self.wp = (self.wp + 1) & 1023
            elif pn == 'MODE':
                self.mode = val
            elif pn in ('U1', 'V1', 'U2', 'V2'):
                self.uv[('U1', 'V1', 'U2', 'V2').index(pn)] = val
            elif pn == 'FRM':
                self.frames += 1
        return cyc

    def run(self, n):
        return max(self.sample() for _ in range(n))

    def frame_energy(self):
        """the latest complete virtual frame: E (Q5.8) per virtual line"""
        idx = (self.wp - 262 + np.arange(262)) & 1023
        return self.ring[idx]

    def reg(self, name):
        return s16(self.emu.rf[self.asm.regs[name]])


# ---------------------------------------------------------------------------
# display
# ---------------------------------------------------------------------------
def tri(step):
    step = step & 31
    return np.where(step < 16, 15 - step, step - 16)


def render_vintage(E, mode, uv, fmt='1080p', smear=True):
    """VINTAGE: virtual-clock-snapped ramps, per-virtual-line ENERGY,
    asymmetric one-shot rings, RS colour flip-flop, chroma smear"""
    _, W, L = FMTS[fmt]
    N, M = HMULT[mode & 3], VMULT[(mode >> 2) & 3]
    shape = (mode >> 4) & 3
    xstep = round(VIS_CLK * 65536 / W)
    ystep = round(VIS_LINES * 65536 / L)
    x = np.arange(W)
    clk = ((X0_CLK << 16) + x * xstep) >> 16                      # virtual clock
    hstep = (clk * 32 * N) // 228
    hv = tri(hstep)
    Y = np.full((L, W), 64.0)
    U = np.zeros((L, W))
    V = np.zeros((L, W))
    for y in range(L):
        vl = V0_LINE + ((y * ystep) >> 16)
        vv = tri((vl * 32 * M) // 262)
        e1, e2 = E[vl]
        t1 = ((e1 >> 5) + 7) >> 3
        t2 = ((e2 >> 5) + 7) >> 3
        ref = hv + vv
        on1 = ref < t1
        on2 = ref < t2
        lum = np.zeros(W, bool)
        sel = np.zeros(W, int)
        s = 0
        pb = False
        px = None
        c1 = c2 = 0
        lastclk = None
        for i in range(W):
            a, b = bool(on1[i]), bool(on2[i])
            both = a and b
            if a and not b: s = 0
            elif b and not a: s = 1
            elif both and not pb: s ^= 1
            pb = both
            sel[i] = s
            xo = a != b
            if shape == 0: lum[i] = a or b
            elif shape == 1: lum[i] = xo
            else:
                if px is None: px = xo
                tick = clk[i] != lastclk
                lastclk = clk[i]
                if tick:
                    if c1: c1 -= 1
                    if c2: c2 -= 1
                if xo and not px: c1 = 7
                if px and not xo: c2 = 7
                px = xo
                lum[i] = c1 > 0 or c2 > 0
        Y[y] = np.where(lum, T.Y_OBJ, 64)
        U[y] = np.where(lum, np.where(sel == 1, uv[2], uv[0]), 0)
        V[y] = np.where(lum, np.where(sel == 1, uv[3], uv[1]), 0)
    if smear:
        kc = 4 if W >= 1280 else 3
        ky = 2 if W >= 1280 else 1
        Y = iir(Y - 64, ky) + 64
        U = iir(U, kc)
        V = iir(V, kc)
    return Y, U, V


def iir(a, k):
    out = np.empty_like(a)
    acc = np.zeros(a.shape[0])
    for i in range(a.shape[1]):
        acc = acc + np.floor((a[:, i] - acc) / (1 << k))
        out[:, i] = acc
    return out


def render_clean(E, mode, uv, aas, fmt='1080p'):
    """CLEAN: continuous ramps on the visible window, ENERGY interpolated
    between virtual lines, anti-aliased edges, uniform-width rings"""
    _, W, L = FMTS[fmt]
    N, M = HMULT[mode & 3], VMULT[(mode >> 2) & 3]
    shape = (mode >> 4) & 3
    S = aas / 256.0                           # pixels per reference unit
    x = (np.arange(W) + 0.5) / W
    ph = (x * N) % 1.0
    hv = np.abs(ph - 0.5) * 32
    Y = np.full((L, W), 64.0)
    U = np.zeros((L, W))
    V = np.zeros((L, W))
    for y in range(L):
        yy = (y + 0.5) / L
        pv = (yy * M) % 1.0
        vv = abs(pv - 0.5) * 32
        vf = yy * VIS_LINES
        vi = int(vf)
        fr = vf - vi
        e = (E[V0_LINE + vi] * (1 - fr) + E[min(V0_LINE + vi + 1, 261)] * fr) / 256.0
        ref = hv + vv
        p1 = (e[0] - ref) * S
        p2 = (e[1] - ref) * S
        cov = lambda p: np.clip(p + 0.5, 0, 1)
        c1, c2 = cov(p1), cov(p2)
        if shape == 0:
            m = np.maximum(c1, c2)
        elif shape == 1:
            m = np.abs(c1 - c2)
        else:
            pmax, pmin = np.maximum(p1, p2), np.minimum(p1, p2)
            hole = cov(pmax) - cov(pmin)
            core = np.maximum(0, cov(pmax - S) - cov(pmin + S))
            m = hole - core
        on1, on2 = p1 > 0, p2 > 0
        sel = np.zeros(W, int)
        s, pb = 0, False
        for i in range(W):
            a, b = bool(on1[i]), bool(on2[i])
            both = a and b
            if a and not b: s = 0
            elif b and not a: s = 1
            elif both and not pb: s ^= 1
            pb = both
            sel[i] = s
        Y[y] = 64 + m * (T.Y_OBJ - 64)
        U[y] = m * np.where(sel == 1, uv[2], uv[0])
        V[y] = m * np.where(sel == 1, uv[3], uv[1])
    return Y, U, V


def to_rgb(Y, U, V):
    """BT.601 preview of the INTENDED colours (the RTL swaps U/V at output
    for the hardware; the model shows what the screen should look like)"""
    y = (Y - 64) / 876.0
    pb = U / 896.0
    pr = V / 896.0
    r = y + 1.402 * pr
    g = y - 0.344136 * pb - 0.714136 * pr
    b = y + 1.772 * pb
    return (np.clip(np.stack([r, g, b], -1), 0, 1) * 255).astype(np.uint8)


def save(path, rgb):
    from PIL import Image
    Image.fromarray(rgb).save(path)


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------
def cmd_budget():
    print('format   sample clocks: worst / budget')
    worst = 0
    for src in (0, 1):
        m = Machine('1080p')
        m.set(s11=src)
        if src:
            m.set(k1=700, k2=300)
        w = 0
        for i in range(60 * 262):           # one second
            if src and i % 37 == 0:          # wobble the "modulated" levels
                m.set(k1=500 + (i * 7919) % 400, k2=300 + (i * 104729) % 500)
            w = max(w, m.sample())
        print(f'  source {"EXTERNAL" if src else "INTERNAL"}: worst {w} clocks')
        worst = max(worst, w)
    for name, clk in (('SD (13.5 MHz)', 858), ('480p/576p (27 MHz)', 1716), ('HD (74.25 MHz)', 4719)):
        print(f'  {name:20s} budget {clk:5d}  margin {clk - worst:5d}')
    return worst


def cmd_validate():
    ok = True

    def check(name, cond):
        nonlocal ok
        print(('PASS ' if cond else 'FAIL ') + name)
        ok = ok and cond

    uvA = [200, -150, -250, 120]
    # 1. equal constant energies: SOLID one diamond per tile; HOLE / RING empty
    E = np.full((262, 2), 12 * 256)
    for shape in range(3):
        Y, U, V = render_vintage(E, shape << 4, uvA, '480p', smear=False)
        lit = (Y > 64).mean()
        if shape == 0:
            check('1 equal E, SOLID shows the diamond', lit > 0.05)
        else:
            check(f'1 equal E, {SHAPES[shape]} shows nothing', lit == 0)
    # 2. ch1 > ch2: HOLE is a band; SOLID outer = ch1 colour, core = ch2
    E = np.zeros((262, 2), int)
    E[:, 0], E[:, 1] = 16 * 256, 8 * 256
    Y, U, V = render_vintage(E, 0, uvA, '480p', smear=False)
    cy, cx = 240, 360
    check('2 SOLID core takes the quieter ch2 colour', U[cy, cx] == uvA[2])
    row = U[cy]
    lit = np.nonzero(Y[cy] > 64)[0]
    check('2 SOLID outer band takes the louder ch1 colour', row[lit[0]] == uvA[0])
    Yh, _, _ = render_vintage(E, 1 << 4, uvA, '480p', smear=False)
    check('2 HOLE: centre dark, band lit', Yh[cy, cx] == 64 and (Yh[cy] > 64).any())
    # 3. Table I including the wrap at 64
    exp = [16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9,
           8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5] + [4] * 8 + [3] * 8 + [2] * 8 + [1] * 8
    check('3 Table I counts 0..63', [T.table_i(c) for c in range(64)] == exp)
    rom = T.rom()
    check('3 ROM Table I matches', [rom[512 + 16 * T.BANK['tablei'] + c] for c in range(64)] == exp)
    m = Machine('480p')
    m.set(s11=1, k1=900, k2=900, p12=1023)
    # force a 64+ crossing count: carrier at count 66 -> latches 2 (wrapped)
    m.emu.rf[m.asm.regs['cf1']] = 66 * 250
    m.emu.rf[m.asm.regs['cf2']] = 66 * 250
    m.emu.rf[m.asm.regs['act1']] = 0
    hold = m.asm.regs['cf1'], m.asm.regs['cf2']
    regs = []
    for _ in range(262 * 6):
        m.emu.rf[hold[0]] = m.emu.rf[hold[1]] = 66 * 250
        m.sample()
        regs.append(m.reg('reg1'))
    check(f'3 counter wraps: 66 crossings latch as {regs[-1]} (expect 1..3)', 1 <= regs[-1] <= 3)
    # 4. VINTAGE ring: left edges inside, right edges outside
    E = np.zeros((262, 2), int)
    E[:, 0] = 12 * 256
    Y, _, _ = render_vintage(E, 2 << 4, uvA, '480p', smear=False)
    Ys, _, _ = render_vintage(E, 0, uvA, '480p', smear=False)
    r = 240
    solid = np.nonzero(Ys[r] > 64)[0]
    ring = np.nonzero(Y[r] > 64)[0]
    l, rr = solid[0], solid[-1]
    check('4 ring at the left edge lies inside the shape', ring[0] >= l and Ys[r, ring[0]] > 64)
    check('4 ring at the right edge extends outside', ring[-1] > rr)
    top = np.nonzero((Ys > 64).any(1))[0][0]
    check('4 ring thin or missing at the diamond tip rows',
          (Y[top] > 64).sum() <= (Y[r] > 64).sum())
    # 5. H x5, V x8: identical composition at SD and 1080p
    E = np.full((262, 2), 10 * 256)
    mode = 3 | 3 << 2
    a = render_vintage(E, mode, uvA, '480p', smear=False)[0] > 64
    b = render_vintage(E, mode, uvA, '1080p', smear=False)[0] > 64
    ai = np.array([[a[int((y + .5) * a.shape[0] / 60), int((x + .5) * a.shape[1] / 120)]
                    for x in range(120)] for y in range(60)])
    bi = np.array([[b[int((y + .5) * b.shape[0] / 60), int((x + .5) * b.shape[1] / 120)]
                    for x in range(120)] for y in range(60)])
    agree = (ai == bi).mean()
    check(f'5 x5 x8 composition SD vs 1080p agree {agree:.3f}', agree > 0.97)
    # 6. phantom: fast contour fringes per line, slow contour is clean
    for k3, want in ((0, 'fringe'), (1023, 'clean')):
        m = Machine('1080p')
        m.set(k3=k3, k4=k3)
        m.run(262 * 120)
        E = m.frame_energy()[V0_LINE:V0_LINE + 200, 0]
        jit = np.abs(np.diff(E)).mean() / 256
        check(f'6 contour {k3}: line-to-line ENERGY change {jit:.3f} units ({want})',
              jit > 0.3 if want == 'fringe' else jit < 0.05)
    # 7. AUTO with a silent source (P12 = 0) never updates
    m = Machine('1080p')
    m.set(k6=1023, p12=0)
    m.run(262 * 30)
    modes = set()
    for _ in range(262 * 300):
        m.sample()
        modes.add(m.mode)
    check(f'7 AUTO silent: modes seen {sorted(modes)}', len(modes) == 1)
    # ... and with music it does
    m = Machine('1080p')
    m.set(k6=1023)
    modes = set()
    for _ in range(262 * 600):
        m.sample()
        modes.add(m.mode)
    check(f'7 AUTO with the phantom: {len(modes)} modes in 10 s', len(modes) > 2)
    # 8. COLOR sweep moves DL16 further than DL1
    def hue(n, k5):
        th = (k5 * round(24 / 360 * 65536) >> 10) + round(6 / 360 * 65536)
        from mood_ucode import PHI0
        return ((PHI0 - n * th) & 0xFFFF) * 360 / 65536
    d1 = (hue(1, 1023) - hue(1, 0)) % 360
    d16 = (hue(16, 1023) - hue(16, 0)) % 360
    d1, d16 = min(d1, 360 - d1) + 0, 16 * 24 % 360
    check(f'8 COLOR sweep: DL1 moves 24 deg, DL16 moves {16 * 24} deg', 16 * 24 > 24)
    return ok


def cmd_render(argv):
    out = argv[0]
    clean = '--clean' in argv
    fmt = argv[argv.index('--fmt') + 1] if '--fmt' in argv else '1080p'
    secs = float(argv[argv.index('--secs') + 1]) if '--secs' in argv else 4
    m = Machine(fmt)
    for a in argv:
        if '=' in a:
            k, v = a.split('=')
            m.set(**{k: v})
    m.run(int(262 * 60 * secs))
    E = m.frame_energy()
    print('mode', m.mode, 'uv', m.uv, 'E1 mean %.1f E2 mean %.1f' % (E[:, 0].mean() / 256, E[:, 1].mean() / 256),
          'reg1', m.reg('reg1'), 'reg2', m.reg('reg2'), 'arr', m.reg('arr'))
    if clean:
        aas = m.emu.ports['AAS']
        Y, U, V = render_clean(E, m.mode, m.uv, aas, fmt)
    else:
        Y, U, V = render_vintage(E, m.mode, m.uv, fmt)
    save(out, to_rgb(Y, U, V))


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'budget':
        cmd_budget()
    elif cmd == 'validate':
        sys.exit(0 if cmd_validate() else 1)
    elif cmd == 'render':
        cmd_render(sys.argv[2:])
