"""Spiroscope v6 design model: vertex generation + per-row scan/stamp + readout.

Not bit-accurate -- used to choose constants (harmonograph mapping, kaleido
copies, stamp reach) and to check the per-row clock budget of the scanner.
Usage: python3 model.py OUTDIR
"""
import sys, math
import numpy as np
from PIL import Image

N = 4096          # vertex ring
CH = 16           # vertices per chunk
CYC, DET, DEC = 24.0, 0.5, 1500.0
TURN = 1 << 24    # angle accumulator units per turn


def gcd(a, b):
    while b:
        a, b = b, a % b
    return a


def gen(machine, a, b, arm, psi, spin=0.0, n=N, start=0, rbase=500, ncopy=1):
    """Return list of copies, each an (n/ncopy, 2) float array of offsets."""
    R2 = arm * rbase
    R1 = rbase - R2
    nb = n // ncopy
    i = np.arange(start, start + nb, dtype=np.float64)
    if machine == 'spiro':
        g = gcd(a, abs(b)) if b else a
        ia, ib = a // g, b // g
        tA = 2 * np.pi * ia * i / N + spin
        tB = 2 * np.pi * ib * i / N + spin
        X = R1 * np.cos(tA) + R2 * np.cos(tB)
        Y = R1 * np.sin(tA) + R2 * np.sin(tB + psi)
    else:  # harmonograph: decaying Lissajous; Arm blends lateral -> rotary
        m = max(a, abs(b))
        k = CYC / m
        tA = 2 * np.pi * a * k * i / N + spin
        tB = 2 * np.pi * b * k * i / N + spin
        det = 2 * np.pi * DET * i / N           # x/y detune of pendulum A
        env = 2.0 ** (-(i % N) / DEC)
        X = env * (R1 * np.cos(tA) + R2 * np.cos(tB))
        Y = env * (R2 * np.sin(tA + det) + R1 * np.sin(tB + psi))
    X = np.round(X)
    Y = np.round(Y)
    tf = [(1, 0, 0, 1), (-1, 0, 0, 1), (1, 0, 0, -1), (-1, 0, 0, -1),
          (0, 1, 1, 0), (0, -1, 1, 0), (0, 1, -1, 0), (0, -1, -1, 0)]
    out = []
    for c in range(ncopy):
        p, q, r, s = tf[c]
        out.append(np.stack([p * X + q * Y, r * X + s * Y], 1))
    return out


def octL(dx, dy):
    a, b = abs(dx), abs(dy)
    hi, lo = max(a, b), min(a, b)
    return hi + (3 * lo) / 8.0


def render(copies, W=1920, H=1080, weight=3, mode='crisp', reach_px=None,
           count_budget=False):
    """Stamp segments into an intensity image like the hardware stamper.
    crisp: c = 15 - |2d| max-combined; glow: c = 7 core, falloff, add-sat."""
    cx, cy = W // 2, H // 2
    img = np.zeros((H, W), np.int32)
    hue = np.zeros((H, W), np.int32)
    if reach_px is None:
        reach_px = (weight + 1) // 2 + (4 if mode == 'glow' else 1)
    jobs_per_row = np.zeros(H, np.int64)
    words_per_row = np.zeros(H, np.int64)
    segs = []
    for cp in copies:
        pts = cp + np.array([cx, cy])
        for k in range(len(pts) - 1):
            segs.append((pts[k], pts[k + 1], k / len(pts)))
    for (p0, p1, age) in segs:
        x0, y0 = p0
        dx, dy = p1[0] - x0, p1[1] - y0
        L = octL(dx, dy)
        if L == 0:
            u = v = 0.0
        else:
            u = 2 * dy / L
            v = 2 * dx / L
        xmin, xmax = min(x0, p1[0]) - reach_px, max(x0, p1[0]) + reach_px
        ymin, ymax = min(y0, p1[1]) - reach_px, max(y0, p1[1]) + reach_px
        ylo, yhi = int(max(0, ymin)), int(min(H - 1, ymax))
        wlo, whi = int(max(0, xmin)) >> 2, int(min(W - 1, xmax)) >> 2
        if ylo > yhi or wlo > whi:
            continue
        xs = np.arange(wlo * 4, whi * 4 + 4)
        for R in range(ylo, yhi + 1):
            jobs_per_row[R] += 1
            words_per_row[R] += whi - wlo + 1
            E = (xs - x0) * u - (R - y0) * v
            e = np.floor(np.abs(E)).astype(np.int32)
            if mode == 'glow':
                c = np.clip(np.minimum(7, 7 + weight - e), 0, 7)
                img[R, xs] = np.minimum(15, img[R, xs] + c)
            else:
                c = np.clip(15 - e, 0, 15)
                img[R, xs] = np.maximum(img[R, xs], c)
            h = int(age * 15.99)
            sel = c > 0
            hue[R, xs[sel]] = np.maximum(hue[R, xs[sel]], h)
    return img, hue, jobs_per_row, words_per_row


def to_rgb(img, hue, weight, mode, paper=False):
    if mode == 'glow':
        lum = np.where(img >= 7, 0.75 + (img - 7) / 32.0, img / 7.0 * 0.6)
        lum = np.clip(lum, 0, 1)
    else:
        lum = np.clip((img - (15 - weight) + 1) / 2.0, 0, 1)
    h = hue / 15.0
    # magenta -> cyan gradient
    col = np.stack([1 - h * 0.9, 0.2 + 0.3 * h, 0.9 + 0.1 * h], -1)
    if paper:
        bg = np.array([0.95, 0.92, 0.82])
        ink = col * 0.45
        rgb = bg * (1 - lum[..., None]) + ink * lum[..., None]
    else:
        rgb = col * lum[..., None]
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def budget(jobs, words, nchunk_hit_rows, clk_line):
    pass


if __name__ == '__main__':
    out = sys.argv[1]
    cases = [
        ('spiro_5_-8', 'spiro', 5, -8, 0.40, 0.0, 1),
        ('spiro_5_8', 'spiro', 5, 8, 0.40, 0.0, 1),
        ('spiro_7_-12_psi', 'spiro', 7, -12, 0.45, 0.9, 1),
        ('spiro_15_-16', 'spiro', 15, -16, 0.5, 0.0, 1),
        ('harm_2_3', 'harm', 2, 3, 0.5, 0.6, 1),
        ('harm_3_-2', 'harm', 3, -2, 0.5, 1.2, 1),
        ('harm_1_1', 'harm', 1, 1, 0.5, 1.5, 1),
        ('kal8_5_-8', 'spiro', 5, -8, 0.4, 0.0, 8),
        ('kal4_5_-8', 'spiro', 5, -8, 0.4, 0.0, 4),
        ('kal8_harm_2_3', 'harm', 2, 3, 0.5, 0.6, 8),
    ]
    for name, mach, a, b, arm, psi, nc in cases:
        cps = gen(mach, a, b, arm, psi, ncopy=nc)
        for mode in ('crisp', 'glow'):
            img, hue, jobs, words = render(cps, weight=3, mode=mode)
            rgb = to_rgb(img, hue, 3, mode)
            Image.fromarray(rgb[::2, ::2]).save(f'{out}/{name}_{mode}.png')
            print(f'{name:16s} {mode:5s} jobs/row max {jobs.max():4d} mean {jobs.mean():5.1f} '
                  f'words/row max {words.max():5d} mean {words.mean():6.1f}')
