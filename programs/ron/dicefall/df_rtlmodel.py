#!/usr/bin/env python3
"""DICEFALL RTL model: the microcode in the emulator, the renderer and the
display in Python, following the VHDL's integer maths (NAT20's renderer,
per die, two slots).

The engine is run one field at a time (it pauses at its next field sync).
Each ring entry it completes ("done") is one die: header, crossing records,
numeral sprites.  The renderer draws each line from the dice whose lines
cover it (never more than two: the slots), each into its own 128-px slot
buffer; the display adds a slot's running sum over the backdrop at the
die's x.

    python3 df_rtlmodel.py --out x.png                  # one 1080p frame
    python3 df_rtlmodel.py --mix --count 1023 --fields 30 --out x.png
"""
import argparse
import os

import numpy as np
from PIL import Image

import df_glyphs as GL
import df_tables as T
import df_ucode as U
from df_uasm import Emu, Pause, PORT, CTLR

HERE = os.path.dirname(os.path.abspath(__file__))
X_BITS = 26
BGU8, BGV8 = 130, 127
XSHIFT = 3
SLOTW = 128
PRE = 128                                # dry lines before line 0

MODES = {   # W, frame lines, interlaced, xa (Q12 offset), line clocks, SD
    '1080p': (1920, 1080, 0, 0, 2200, 0),
    '720p': (1280, 720, 0, 0, 1650, 0),
    '480i': (720, 486, 1, 455, 858, 1),
    '576i': (720, 576, 1, -256, 864, 1),
}
NCAP = {(0, 0): 20, (0, 1): 20, (1, 0): 4, (1, 1): 6}   # dice the engine keeps up with: (SD, mix)


def wrap(v, bits):
    m = 1 << bits
    v &= m - 1
    return v - m if v >> (bits - 1) else v


def sfield(w, lo, n):
    return wrap(w >> lo, n)


class Engine:
    """the microcode, one field at a time"""

    def __init__(self, ctl):
        U.build()
        self.em = Emu(U.a.resolve(), U.engine_rom(), T.build()[0], self.ctlf)
        self.ctl = ctl
        self.fld = 0
        self.syncs = 0

    def ctlf(self, imm):
        if imm == CTLR['fld']:
            self.syncs += 1
            if self.syncs > 1:
                raise Pause()
            return self.fld
        if imm == CTLR['wfree']:
            return 1
        return self.ctl.get(imm, 0)

    def field(self, f):
        """run the engine through field f's dice; returns the dice"""
        self.fld = (f - 1) & 255             # the renderer one field behind
        self.syncs = 0
        c0 = self.em.cycles
        self.em.run()
        port = {v: k for k, v in PORT.items()}
        dice, cur = [], None
        for p, idx, val in self.em.slw:
            n = port[p]
            val &= 0xFFFFFFFF
            if cur is None:
                cur = dict(e={}, s={}, h={})
            if n in ('e0', 'e1', 'e2', 'e3'):
                cur['e'].setdefault(idx, [0] * 4)[int(n[1])] = val
            elif n[0] == 'p':
                cur['s'].setdefault(idx - 11, [0] * 7)[int(n[1])] = val
            elif n in ('h0', 'h1', 'h2'):
                cur['h'][n] = val
            elif n == 'done':
                h0, h2 = cur['h']['h0'], cur['h']['h2']
                ne, ns = h2 & 31, (h2 >> 5) & 7
                dice.append(dict(top=sfield(h0, 0, 12), bot=sfield(h0, 12, 12),
                                 tag=(h0 >> 24) & 255, x0=cur['h']['h1'] & 4095,
                                 edges=[cur['e'][i] for i in range(ne)],
                                 sprs=[cur['s'][i] for i in range(ns)], xline={}, run={}))
                cur = None
            elif n == 'bgs':
                self.bgs = wrap(val, 20)
        return dice, self.em.cycles - c0


GLY = GL.glyph_set()


def texel(ga, gb, X, Y):
    if Y < 0 or Y >= GL.TH or X < 0 or X >= 32:
        return 15
    gi = ga if X < 16 else gb
    if gi >= GL.NG:
        return 15
    return int(GLY[gi, Y, X & 15])


def sprite_alpha(ga, gb, k, u, v):
    ix, fx = u >> 12, (u >> 8) & 15
    iy, fy = v >> 12, (v >> 8) & 15
    t00, t10 = texel(ga, gb, ix, iy), texel(ga, gb, ix + 1, iy)
    t01, t11 = texel(ga, gb, ix, iy + 1), texel(ga, gb, ix + 1, iy + 1)
    top = t00 * 16 + (t10 - t00) * fx
    bot = t01 * 16 + (t11 - t01) * fx
    d = top * 16 + (bot - top) * fy
    return min(16, max(0, 8 - ((k * (d - 2048)) >> 11)))


def flush(buf, p, acc):
    buf[0, p] = wrap(acc[0], 16)
    buf[1, p] = wrap(acc[1] >> 4, 16)
    buf[2, p] = wrap(acc[2], 16)
    buf[3, p] = wrap(acc[3], 16)


def render_edges(die, jb, bgy, buf):
    """the die's crossing records on one field line into its slot buffer
    (4 x 128: DY FY DU DV); returns the RTL clocks"""
    clocks = 0
    for e, (w0, w1, w2, w3) in enumerate(die['edges']):
        y0, y1 = sfield(w0, 0, 16), sfield(w0, 16, 16)
        d0, d1 = y0 - jb, y1 - jb
        if not (d0 < 16 and d1 > 0):
            clocks += 5
            continue
        klo, khi = max(d0, 0), min(d1, 16)
        clocks += 9 + (khi - klo)
        x = wrap(w1, X_BITS) if d0 >= 0 else die['xline'][e]
        s = wrap(w2, X_BITS)
        dy, du, dv = sfield(w3, 0, 11), sfield(w3, 11, 9), sfield(w3, 20, 9)
        if (w3 >> 29) & 1:
            sg = 1 if (w3 >> 30) & 1 else -1
            dy, du, dv = dy + sg * bgy, du + sg * BGU8, dv + sg * BGV8
        acc_p, acc = None, None
        for k in range(klo, khi):
            if x < 0:
                p, fi = 0, 0
            elif (x >> 16) > SLOTW - 1:
                p, fi = SLOTW - 1, 15
            else:
                p, fi = x >> 16, (x >> 12) & 15
            if acc_p is not None and p == acc_p:
                acc[0] += dy; acc[1] += dy * fi; acc[2] += du; acc[3] += dv
            else:
                if acc_p is not None:
                    flush(buf, acc_p, acc)
                acc = [buf[0, p] + dy, (buf[1, p] << 4) + dy * fi, buf[2, p] + du, buf[3, p] + dv]
                acc_p = p
            x = wrap(x + s, X_BITS)
        if acc_p is not None:
            flush(buf, acc_p, acc)
        die['xline'][e] = x
    return clocks


def render_sprites(die, line, buf):
    clocks = 0
    for s, (w0, w1, w2, w3, w4, w5, w6) in enumerate(die['sprs']):
        ly0, ly1 = sfield(w0, 0, 12), sfield(w0, 12, 12)
        ga, gb = (w0 >> 24) & 15, (w0 >> 28) & 15
        if not (ly0 <= line < ly1):
            clocks += 4
            continue
        sx0, sw, k = w1 & 255, (w1 >> 9) & 255, (w1 >> 17) & 255
        dux, dvx = sfield(w2, 0, 16), sfield(w2, 16, 16)
        duy, dvy = sfield(w3, 0, 16), sfield(w3, 16, 16)
        dl = (sfield(w4, 0, 11), sfield(w4, 11, 9), sfield(w4, 20, 9))
        u, v = die['run'].get(s, (sfield(w5, 0, 20), sfield(w6, 0, 20)))
        die['run'][s] = (wrap(u + duy, 20), wrap(v + dvy, 20))
        clocks += 13 + sw
        cp = (0, 0, 0)
        for i in range(sw + 1):
            p = sx0 + i
            al = 0 if i == sw else sprite_alpha(ga, gb, k, u, v)
            c = (al * dl[0], 16 * dl[1] if al >= 8 else 0, 16 * dl[2] if al >= 8 else 0)
            if p < SLOTW:
                buf[0, p] = wrap(buf[0, p] + c[0] - cp[0], 16)
                buf[2, p] = wrap(buf[2, p] + c[1] - cp[1], 16)
                buf[3, p] = wrap(buf[3, p] + c[2] - cp[2], 16)
            cp = c
            u, v = wrap(u + dux, 20), wrap(v + dvx, 20)
    return clocks


def render(fields=4, mode='1080p', count=600, tumble=300, light=340, size=878,
           fall=200, mix=False):
    """run `fields` fields; returns the last one's (Y, U, V) and info"""
    W, HF, il, xa, lclk, sd = MODES[mode]
    ctl = {CTLR['k1']: count, CTLR['k2']: tumble, CTLR['k3']: light, CTLR['k4']: size,
           CTLR['slider']: fall, CTLR['sw']: int(mix), CTLR['W']: W, CTLR['hf']: HF,
           CTLR['ilace']: il, CTLR['field']: 0, CTLR['xa']: xa, CTLR['ncap']: NCAP[(sd, int(mix))],
           CTLR['rhf']: int(round((1 << 20) / HF))}
    eng = Engine(ctl)
    for f in range(fields):
        dice, cyc = eng.field(f)
    lines = HF // 2 if il else HF
    Y = np.zeros((lines, W), np.int64)
    Uc = np.full((lines, W), BGU8 * 4, np.int64)
    Vc = np.full((lines, W), BGV8 * 4, np.int64)
    bga = U.YTOP << 16
    worst, maxact = 0, 0
    slots = {}                                  # die index -> slot buffer
    # lines -128..-1 are rendered dry at the field start (vertical
    # blanking): dice straddling the top arrive at line 0 in step
    for l in range(-PRE - 1, lines):
        r = l + 1                               # the line rendered now
        bgy = bga >> 16
        if l >= 0:
            Y[l] = bgy
            for di, buf in slots.items():
                d = dice[di]
                S = np.cumsum(buf[[0, 2, 3]], axis=1)
                S[0] -= buf[1]
                x0 = d['x0'] + XSHIFT
                xs = slice(x0, min(x0 + SLOTW, W))
                n = xs.stop - xs.start
                Y[l, xs] = np.clip(bgy + ((S[0, :n] + 8) >> 4), 0, 1023)
                Uc[l, xs] = np.clip(4 * BGU8 + ((S[1, :n] + 2) >> 2), 0, 1023)
                Vc[l, xs] = np.clip(4 * BGV8 + ((S[2, :n] + 2) >> 2), 0, 1023)
            bga += eng.bgs
        if r >= lines:
            break
        act = [i for i, d in enumerate(dice) if d['top'] <= r < d['bot']]
        maxact = max(maxact, len(act))
        slots = {}
        clk = 0
        for i in act:
            buf = np.zeros((4, SLOTW), np.int64)
            clk += render_edges(dice[i], 16 * r, bgy, buf)
            clk += render_sprites(dice[i], r, buf)
            slots[i] = buf
        if r >= 0:
            worst = max(worst, clk)
    info = dict(dice=len(dice), engine_clocks=cyc, render_max=worst, max_per_line=maxact,
                line_budget=lclk, sprites=sum(len(d['sprs']) for d in dice))
    return Y, Uc, Vc, info


def to_rgb(Y, Uc, Vc, il=False):
    y = (Y - 64) / 876.0
    u = (Uc - 512) / 896.0
    v = (Vc - 512) / 896.0
    rgb = np.stack([y + 1.402 * v, y - 0.344136 * u - 0.714136 * v, y + 1.772 * u], -1)
    img = (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)
    if il:
        img = np.repeat(img, 2, 0)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='1080p')
    ap.add_argument('--fields', type=int, default=4)
    ap.add_argument('--count', type=int, default=1023)
    ap.add_argument('--tumble', type=int, default=300)
    ap.add_argument('--light', type=int, default=340)
    ap.add_argument('--size', type=int, default=878)
    ap.add_argument('--fall', type=int, default=200)
    ap.add_argument('--mix', action='store_true')
    ap.add_argument('--out', default=os.path.join(HERE, 'df_preview.png'))
    args = ap.parse_args()
    Y, Uc, Vc, info = render(args.fields, args.mode, args.count, args.tumble, args.light,
                             args.size, args.fall, args.mix)
    print(info)
    Image.fromarray(to_rgb(Y, Uc, Vc, MODES[args.mode][2])).save(args.out)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
