#!/usr/bin/env python3
"""NAT20 RTL model (v0.2.0): the microcode run in the emulator, the scanline
renderer and the display in Python, following the VHDL's integer maths.

Renderer (per field line, 16 sub-lines): every crossing record active on
the line steps its x once a sub-line and deposits the colour step at each
crossing into the line buffer as two terms per channel at the crossing's
pixel p: D += step, F += step * frac (frac in 1/16 px; F kept /16).  A
crossing's deposits accumulate in a register while it stays in one pixel
and are written once when it moves on (one read-modify-write a run).
Display: a running sum of D across the window, minus F, over 16:
    value(p) = backdrop + (sum(D[0..p]) - F[p]) / 16
which is the exact box-filtered coverage along each sub-line.

    python3 nat20_rtlmodel.py --die 4 --yaw 600 --pitch 450 --out x.png
    python3 nat20_rtlmodel.py --sheet          # all five dice, a few poses
"""
import argparse
import os

import numpy as np
from PIL import Image

import nat20_glyphs as GL
import nat20_tables as T
import nat20_ucode as U
from nat20_uasm import Emu, PORT, CTLR

HERE = os.path.dirname(os.path.abspath(__file__))
X_BITS = 26
BGU8, BGV8 = 130, 127            # backdrop chroma, 8-bit (C_BGU / C_BGV >> 2)
XSHIFT = 3                       # the display path is 3 clocks shorter than the sync tap

MODES = {   # W, frame lines, interlaced, xa (Q12 offset), line clocks, timing id
    '1080p': (1920, 1080, 0, 0, 2200, 3),
    '720p': (1280, 720, 0, 0, 1650, 6),
    '480i': (720, 486, 1, 455, 858, 0),
    '576i': (720, 576, 1, -256, 864, 8),
}


def wrap(v, bits):
    m = 1 << bits
    v &= m - 1
    return v - m if v >> (bits - 1) else v


def sfield(w, lo, n):
    return wrap(w >> lo, n)


def run_engine(ctl, fields=2):
    U.build()
    code = U.a.resolve()
    rom = U.engine_rom()
    table, _ = T.build()
    em = Emu(code, rom, table, {})
    rf = [0] * 128
    for f in range(fields):
        c = dict(ctl)
        c[CTLR['zfirst']] = 1 if f == 0 else 0
        em.ctl = c
        em.cycles = em.steps = 0
        em.run(rf)
        rf = em.rf
    return em


def engine_outputs(em):
    """(edge records, globals); the sprite records are in g['spr']"""
    port = {v: k for k, v in PORT.items()}
    rec, spr = {}, {}
    g = {}
    for p, idx, val in em.slw:
        n = port[p]
        if n in ('e0', 'e1', 'e2', 'e3'):
            rec.setdefault(idx, [0, 0, 0, 0])[int(n[1])] = val & 0xFFFFFFFF
        elif n in ('p0', 'p1', 'p2', 'p3', 'p4', 'p5'):
            spr.setdefault(idx - 4, [0] * 6)[int(n[1])] = val & 0xFFFFFFFF
        else:
            g[n] = val
    g['spr'] = [spr[i] for i in range(g.get('ns', 0))]
    return [rec[i] for i in range(g['ne'])], g


GLY = GL.glyph_set()                     # (12, 20, 16) codes


def texel(ga, gb, X, Y):
    if Y < 0 or Y >= GL.TH or X < 0 or X >= 32:
        return 15
    gi = ga if X < 16 else gb
    if gi >= GL.NG:
        return 15
    return int(GLY[gi, Y, X & 15])


def sprite_alpha(ga, gb, k, u, v):
    """ink coverage x16 at texel coordinates (u, v), Q12 (texel centres at
    integers): bilinear over the 4-bit field, 1-px ramp at k px a texel (Q4)"""
    ix, fx = u >> 12, (u >> 8) & 15
    iy, fy = v >> 12, (v >> 8) & 15
    t00, t10 = texel(ga, gb, ix, iy), texel(ga, gb, ix + 1, iy)
    t01, t11 = texel(ga, gb, ix, iy + 1), texel(ga, gb, ix + 1, iy + 1)
    top = t00 * 16 + (t10 - t00) * fx
    bot = t01 * 16 + (t11 - t01) * fx
    d = top * 16 + (bot - top) * fy                  # Q8 code
    return min(16, max(0, 8 - ((k * (d - 2048)) >> 11)))


def render_sprites(sprs, line, buf, run, stats):
    """the numerals on one field line: per pixel of each span, the ink's
    colour step times its coverage, deposited as the change from the last
    pixel (D only); returns the RTL clocks"""
    clocks = 0
    for s, (w0, w1, w2, w3, w4, w5) in enumerate(sprs):
        ly0, ly1 = w0 & 2047, (w0 >> 11) & 2047
        ga, gb = (w0 >> 22) & 15, (w0 >> 26) & 15
        if not (ly0 <= line < ly1):
            clocks += 4
            continue
        sx0, sw, k = w1 & 511, (w1 >> 9) & 255, (w1 >> 17) & 255
        dux, dvx = sfield(w2, 0, 16), sfield(w2, 16, 16)
        duy, dvy = sfield(w3, 0, 16), sfield(w3, 16, 16)
        dl = (sfield(w4, 0, 11), sfield(w4, 11, 9), sfield(w4, 20, 9))
        if line == ly0:
            u, v = sfield(w5, 0, 16) << 4, sfield(w5, 16, 16) << 4
        else:
            u, v = run[s]
        run[s] = (wrap(u + duy, 20), wrap(v + dvy, 20))
        clocks += 14 + sw
        cp = (0, 0, 0)
        for i in range(sw + 1):
            p = sx0 + i
            al = 0 if i == sw else sprite_alpha(ga, gb, k, u, v)
            # luma: step x coverage; chroma whole at half cover
            c = (al * dl[0], 16 * dl[1] if al >= 8 else 0, 16 * dl[2] if al >= 8 else 0)
            if p < 256:
                for ch in range(3):
                    buf[2 * ch, p] = wrap(buf[2 * ch, p] + c[ch] - cp[ch], 16)
            cp = c
            u, v = wrap(u + dux, 20), wrap(v + dvx, 20)
    return clocks


def render_line(recs, jb, bgy, buf, xline, stats):
    """one field line: the renderer's deposits into buf (6 x 256: DY FY DU FU
    DV FV), x_line updated; returns the clocks the RTL sequencer spends"""
    clocks = 0
    for e, (w0, w1, w2, w3) in enumerate(recs):
        y0, y1 = w0 & 0x7FFF, (w0 >> 15) & 0x7FFF
        d0, d1 = y0 - jb, y1 - jb
        if not (d0 < 16 and d1 > 0):
            clocks += 5
            continue
        klo, khi = max(d0, 0), min(d1, 16)
        clocks += 9 + (khi - klo)
        x = wrap(w1, X_BITS) if d0 >= 0 else xline[e]
        s = wrap(w2, X_BITS)
        dy, du, dv = sfield(w3, 0, 11), sfield(w3, 11, 9), sfield(w3, 20, 9)
        if (w3 >> 29) & 1:
            sg = 1 if (w3 >> 30) & 1 else -1
            dy, du, dv = dy + sg * bgy, du + sg * BGU8, dv + sg * BGV8
        dl = (dy, du, dv)
        acc_p = None
        acc = None
        for k in range(klo, khi):
            if x < 0:
                p, fi = 0, 0
            elif (x >> 16) > 255:
                p, fi = 255, 15
            else:
                p, fi = x >> 16, (x >> 12) & 15
            if acc_p is not None and p == acc_p:
                for c in range(3):
                    acc[2 * c] += dl[c]
                    acc[2 * c + 1] += dl[c] * fi
            else:
                if acc_p is not None:
                    flush(buf, acc_p, acc)
                acc = []
                for c in range(3):
                    acc.append(buf[2 * c, p] + dl[c])
                    acc.append((buf[2 * c + 1, p] << 4) + dl[c] * fi)
                acc_p = p
            x = wrap(x + s, X_BITS)
        if acc_p is not None:
            flush(buf, acc_p, acc)
        xline[e] = x
    return clocks


def flush(buf, p, acc):
    for c in range(3):
        buf[2 * c, p] = wrap(acc[2 * c], 16)
        buf[2 * c + 1, p] = wrap(acc[2 * c + 1] >> 4, 16)


def render(die=4, yaw=512, pitch=512, light=340, col=0, size=325, mode='1080p',
           field=0, fields=2, rhf=None, nlines=None):
    """rhf / nlines override the backdrop step and the lines drawn (to match
    a sim whose engine still used the power-up frame height)"""
    W, HF, il, xa, lclk, tid = MODES[mode]
    bw = 128 if tid in (0, 4, 8, 12) else 256
    ctl = {CTLR['k1']: die * 205 + 100, CTLR['k2']: yaw, CTLR['k3']: pitch,
           CTLR['k4']: col * 85 + 40, CTLR['k5']: size, CTLR['slider']: light,
           CTLR['W']: W, CTLR['hf']: HF, CTLR['ilace']: il,
           CTLR['field']: field, CTLR['xa']: xa, CTLR['bw']: bw,
           CTLR['rhf']: rhf if rhf else int(round((1 << 20) / HF))}
    em = run_engine(ctl, fields)
    recs, g = engine_outputs(em)
    wx0 = g['wx0'] + XSHIFT
    lines = nlines if nlines else (HF // 2 if il else HF)
    buf = np.zeros((6, 256), np.int64)
    xline = [0] * 32
    Y = np.zeros((lines, W), np.int64)
    Uc = np.full((lines, W), BGU8 * 4, np.int64)
    Vc = np.full((lines, W), BGV8 * 4, np.int64)
    bga = (U.YTOP << 16)
    worst = 0
    stats = {}
    # line 0 renders in vertical blanking, then each line renders the next
    # after its own window has gone by (with the current line's backdrop)
    run = {}
    worst = max(worst, render_line(recs, 0, bga >> 16, buf, xline, stats)
                + render_sprites(g['spr'], 0, buf, run, stats))
    for l in range(lines):
        bgy = bga >> 16
        Y[l] = bgy
        # display line l: running sum across the window, clear behind
        S = np.cumsum(buf[0::2, :bw], axis=1)
        F = buf[1::2, :bw]
        dv = S - F
        Y[l, wx0:wx0 + bw] = np.clip(bgy + ((dv[0] + 8) >> 4), 0, 1023)
        Uc[l, wx0:wx0 + bw] = np.clip(4 * BGU8 + ((dv[1] + 2) >> 2), 0, 1023)
        Vc[l, wx0:wx0 + bw] = np.clip(4 * BGV8 + ((dv[2] + 2) >> 2), 0, 1023)
        buf[:, :bw] = 0
        worst = max(worst, render_line(recs, 16 * (l + 1), bgy, buf, xline, stats)
                    + render_sprites(g['spr'], l + 1, buf, run, stats))
        bga += g['bgs']
    info = dict(edges=len(recs), sprites=len(g['spr']), wx0=g['wx0'], bw=bw, cycles=em.cycles,
                render_clocks_max=worst, line_budget=lclk - bw - 16)
    return Y, Uc, Vc, em, info


def to_rgb(Y, Uc, Vc, il=False):
    y = (Y - 64) / 876.0
    u = (Uc - 512) / 896.0
    v = (Vc - 512) / 896.0
    rgb = np.stack([y + 1.402 * v, y - 0.344136 * u - 0.714136 * v, y + 1.772 * u], -1)
    img = (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)
    if il:
        img = np.repeat(img, 2, 0)
    return img


def crop(img, W, H, half):
    cx, cy = W // 2, H // 2
    return img[max(cy - half, 0):cy + half, max(cx - half, 0):cx + half]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--die', type=int, default=4)
    ap.add_argument('--yaw', type=int, default=512)
    ap.add_argument('--pitch', type=int, default=512)
    ap.add_argument('--light', type=int, default=340)
    ap.add_argument('--col', type=int, default=0)
    ap.add_argument('--size', type=int, default=325)
    ap.add_argument('--mode', default='1080p')
    ap.add_argument('--sheet', action='store_true')
    ap.add_argument('--crop', type=int, default=0, help='half-size of a centre crop, px')
    ap.add_argument('--zoom', type=int, default=1, help='nearest-neighbour zoom of the crop')
    ap.add_argument('--out', default=os.path.join(HERE, 'rtl_preview.png'))
    args = ap.parse_args()
    if args.sheet:
        tiles = []
        for die in range(5):
            row = []
            for yaw, pitch, light in ((512, 512, 340), (600, 470, 200), (380, 600, 800)):
                Y, Uc, Vc, em, info = render(die, yaw, pitch, light, col=die * 2,
                                             size=args.size, mode=args.mode)
                print(f'd{[4,6,8,10,20][die]} yaw {yaw} pitch {pitch}: {info}')
                W, HF = MODES[args.mode][:2]
                img = to_rgb(Y, Uc, Vc, MODES[args.mode][2])
                row.append(crop(img, W, HF, 80))
            tiles.append(np.concatenate(row, 1))
        out = np.concatenate(tiles, 0)
        if args.zoom > 1:
            out = out.repeat(args.zoom, 0).repeat(args.zoom, 1)
        Image.fromarray(out).save(args.out)
        print('wrote', args.out)
        return
    Y, Uc, Vc, em, info = render(args.die, args.yaw, args.pitch, args.light, args.col,
                                 args.size, args.mode)
    print(info)
    W, HF = MODES[args.mode][:2]
    img = to_rgb(Y, Uc, Vc, MODES[args.mode][2])
    if args.crop:
        img = crop(img, W, HF, args.crop)
        if args.zoom > 1:
            img = img.repeat(args.zoom, 0).repeat(args.zoom, 1)
    Image.fromarray(img).save(args.out)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
