#!/usr/bin/env python3
"""Bit-exactness check for the per-face microcode (phase 9).

The reference here is a transcription of cubist.vhd's frame FSM states
58-101, NOT cubist_model.py -- the model's conventions differ and comparing
against it once produced a phantom error that cost an afternoon.  Phases 1-8
are already verified, so this feeds the oracle the register state the
emulator reaches at label 'p9' and checks only what the face loop produces.
"""
import math
import sys
sys.path.insert(0, '.')
from uasm import OPS, SLOTW
from uemu import Emu, s16
import frame_ucode as fu

SIN = [round(4096 * math.sin(k * math.pi / 128)) for k in range(256)]
GAM = [min(255, round(255 * (i / 255.0) ** 0.45)) for i in range(256)]

C_FCORN = [[2, 6, 7, 3], [0, 1, 5, 4], [1, 3, 7, 5],
           [0, 4, 6, 2], [4, 5, 7, 6], [1, 0, 2, 3]]
C_FADJ = [[5, 4, 3, 2], [3, 2, 5, 4], [1, 0, 5, 4],
          [5, 4, 1, 0], [3, 2, 1, 0], [1, 0, 3, 2]]
C_FN, C_FU, C_FV = [2, 3, 0, 1, 4, 5], [4, 0, 2, 4, 0, 2], [0, 4, 4, 2, 2, 0]
C_STY = [995, 807, 332, 513, 416, 276]
C_STU = [512, 91, 460, 244, 496, 757]
C_STV = [512, 654, 811, 848, 238, 330]
COEF = [-107, 154, 184, 141, -141, 161]


def sdiv(num, den, shift, cap):
    """The restoring divider: magnitudes in, sign carried alongside."""
    if den == 0:
        return 0
    q = (abs(num) << shift) // abs(den)
    q = min(q, cap)
    return -q if (num < 0) != (den < 0) else q


def oracle(rf, ctl_cx):
    """FSM states 58-101 from the post-phase-8 register file."""
    # the register file holds raw 32-bit words; read them back as signed
    CX = [s16(rf[fu.R_CX + k]) for k in range(8)]
    CY = [s16(rf[fu.R_CY + k]) for k in range(8)]
    BAS = [s16(rf[fu.R_BAS + k]) for k in range(9)]
    H = [s16(rf[fu.R_H + k]) for k in range(3)]
    vis = [s16(rf[fu.R_VIS + k]) for k in range(6)]

    slots, gram, nslot = {}, {}, 0
    for i in range(6):
        if not vis[i] or nslot == 3:
            continue
        fc = C_FCORN[i]
        px0, py0 = CX[fc[0]], CY[fc[0]]
        dx1, dy1 = (CX[fc[1]] - px0) >> 2, (CY[fc[1]] - py0) >> 2
        dx2, dy2 = (CX[fc[3]] - px0) >> 2, (CY[fc[3]] - py0) >> 2
        det = dx1 * dy2 - dy1 * dx2

        for gn, port in ((dy2, 'gux'), (-dx2, 'guy'),
                         (-dy1, 'gvx'), (dx1, 'gvy')):
            slots[(SLOTW[port], nslot)] = sdiv(3 * gn, det, 20, 65535)
        slots[(SLOTW['px0'], nslot)] = px0 >> 2
        slots[(SLOTW['py0'], nslot)] = py0 >> 2

        ys = [CY[fc[c]] for c in range(4)]
        base = nslot * 32
        gram[base + 0] = max(min(ys) >> 2, 0)
        gram[base + 1] = max((max(ys) >> 2) + 1, 0)

        for c in range(4):
            n = (c + 1) % 4
            sxa, sya = CX[fc[c]], CY[fc[c]]
            dxe, t = CX[fc[n]] - sxa, CY[fc[n]] - sya
            gram[base + 2 + 3 * c] = sya
            gram[base + 3 + 3 * c] = sxa
            den = t if abs(t) >= 2 else (-2 if t < 0 else 2)
            gram[base + 4 + 3 * c] = sdiv(dxe, den, 6, 32000)

        # flat face light
        fn = C_FN[i]
        bs = [BAS[(fn // 2) * 3 + p] for p in range(3)]
        if fn & 1:
            bs = [-v for v in bs]
        lacc = sum((bs[c % 3] * COEF[c]) >> 12 for c in range(3))
        ac1 = sum((bs[c % 3] * COEF[c]) >> 12 for c in range(3, 6))
        lf = 62
        if lacc > 0:
            lf += lacc - (lacc >> 2)
        if ac1 > 0:
            lf += ac1 >> 2
        lf = min(lf, 255)

        slots[(SLOTW['face'], nslot)] = i
        slots[(SLOTW['sil'], nslot)] = sum(
            (0 if vis[C_FADJ[i][j]] else 1) << j for j in range(4))
        for axis, port in ((C_FN[i], 'hn'), (C_FU[i], 'hu'), (C_FV[i], 'hv')):
            v = H[axis // 2]
            slots[(SLOTW[port], nslot)] = -v if axis & 1 else v
        slots[(SLOTW['ly'], nslot)] = C_STY[i]
        slots[(SLOTW['lf'], nslot)] = lf
        g = GAM[lf]
        for tab, port in ((C_STU, 'cu'), (C_STV, 'cv')):
            v = 512 + (((tab[i] - 512) * g) >> 8)
            slots[(SLOTW[port], nslot)] = min(max(v, 0), 1023)
        nslot += 1

    qx0 = (ctl_cx * ctl_cx) & 0x3FFFFF
    slots[(SLOTW['qx0'], 0)] = qx0
    gram[127] = nslot
    return slots, gram


def run(ctl):
    code = fu.a.resolve()
    p9 = fu.a.labels['p9']
    snap = Emu([list(c) for c in code[:p9]] + [[OPS['END'], 0, 0, 0, 0]],
               SIN, ctl, GAM)
    snap.run()
    full = Emu(code, SIN, ctl, GAM)
    full.run()
    return snap.rf, full


CASES = [
    dict(zip(range(11), (880, 68, 0, 1920, 1080, 1080, 960, 540, 0, 512, 1))),
    dict(zip(range(11), (512, 300, 700, 1920, 1080, 1080, 960, 540, 0, 512, 1))),
    dict(zip(range(11), (123, 456, 789, 720, 486, 486, 360, 243, 1, 700, 1))),
    dict(zip(range(11), (0, 0, 0, 1920, 1080, 1080, 960, 540, 0, 512, 1))),
]

def main():
    bad = 0
    for n, ctl in enumerate(CASES):
        rf, emu = run(ctl)
        want_s, want_g = oracle(rf, ctl[6])
        for key in sorted(set(want_s) | set(emu.slots)):
            got, exp = emu.slots.get(key), want_s.get(key)
            if got != exp:
                bad += 1
                print(f'  case {n} slot{key}: got {got} want {exp}')
        for key in sorted(set(want_g) | set(emu.gram)):
            got, exp = emu.gram.get(key), want_g.get(key)
            if got != exp:
                bad += 1
                print(f'  case {n} gram[{key}]: got {got} want {exp}')
        print(f'case {n}: nslot={want_g[127]} steps={emu.steps}')
    print('per-face loop bit-exact vs the FSM' if bad == 0
          else f'{bad} MISMATCHES')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
