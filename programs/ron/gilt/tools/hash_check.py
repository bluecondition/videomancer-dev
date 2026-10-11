"""Numerical spot-check of the GILT hash (CLAUDE.md: check every new hash before use).
h16(a, b, s): 3 pipeline stages, adds for nonlinearity (xor/rotate alone is GF(2)-linear).
Bit-exact model of f_hash in gilt.vhd."""
import numpy as np
M = 0xFFFF
def rotl(v, k): return ((v << k) | (v >> (16 - k))) & M
def h16(a, b, s):
    v = ((a ^ s) + rotl(b, 7) + 0x6D2B) & M          # stage 1
    v = (v + (v << 4)) & M; v ^= v >> 7              # stage 2
    v = (v + (v << 9) + (v << 2)) & M; v ^= v >> 8   # stage 3
    return v
h = np.vectorize(h16)
def corr(p, q):
    p = p - p.mean(); q = q - q.mean(); d = np.sqrt((p*p).sum()*(q*q).sum()); return 0 if d == 0 else (p*q).sum()/d
N = 256
x, y = np.meshgrid(np.arange(N), np.arange(N))
for s in (0x3A91, 0xC0DE):
    g = h(x, y, s).astype(np.int64)
    print(f"seed {s:#06x}: unique {len(np.unique(g))}/{N*N} (random ~{int(65536*(1-np.exp(-1)))}), "
          f"bit bias {min(((g>>b)&1).mean() for b in range(16)):.3f}..{max(((g>>b)&1).mean() for b in range(16)):.3f}")
    worst = 0
    for b in range(16):
        bb = ((g >> b) & 1).astype(float)
        for (dy, dx) in ((0,1),(1,0),(1,1),(1,-1),(0,2),(2,0)):
            p = bb[:N-dy, max(0,-dx):N-max(0,dx)]; q = bb[dy:, max(0,dx):N-max(0,-dx) if dx<0 else N]
            q = bb[dy:, max(0,dx):][:, :p.shape[1]] if dx >= 0 else bb[dy:, :N+dx]
            worst = max(worst, abs(corr(p, q)))
    print(f"   worst |corr| of any bit with a neighbour (h, v, diag, anti-diag, 2-step): {worst:.3f}")
    # slices the design uses: 4-bit LUT index, 8-bit threshold byte
    for lo, w in ((0, 4), (4, 8), (12, 3)):
        v = (g >> lo) & ((1 << w) - 1)
        hist = np.bincount(v.ravel(), minlength=1 << w) / v.size * (1 << w)
        print(f"   bits[{lo+w-1}:{lo}] uniformity min/max {hist.min():.2f}/{hist.max():.2f}; "
              f"h-corr {corr(v[:, :-1].astype(float), v[:, 1:].astype(float)):+.3f} v-corr {corr(v[:-1].astype(float), v[1:].astype(float)):+.3f}")
# 1-D sequences used by the walker: (band j, tile k) with consecutive k, and (j, 0) for consecutive j
k = np.arange(4096); seq = h(k & 0xFF, k >> 8, 0x51F3).astype(np.int64)
lo4 = seq & 15
print("tile sequence: corr(k,k+1) of 4-bit width index", round(corr(lo4[:-1].astype(float), lo4[1:].astype(float)), 3))
