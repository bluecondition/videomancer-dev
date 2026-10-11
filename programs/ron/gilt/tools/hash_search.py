"""Search cheap 16-bit hash variants: v=(a^s)+rotl(b,8); v*=C1; v^=v>>r1; v*=C2; v^=v>>r2.
C1, C2 are odd constants with 3 set bits (2 adders each). Score = worst neighbour bit
correlation over (x,y) grids + uniqueness."""
import itertools, numpy as np
M = 0xFFFF
N = 128
x, y = np.meshgrid(np.arange(N, dtype=np.int64), np.arange(N, dtype=np.int64))
def rotl(v, k): return ((v << k) | (v >> (16 - k))) & M
def run(c1, c2, r1, r2, s=0x3A91, a=x, b=y):
    v = ((a ^ s) + rotl(b, 8)) & M
    v = (v * c1) & M; v ^= v >> r1
    v = (v * c2) & M; v ^= v >> r2
    return v
def score(g):
    worst = 0
    for bit in range(16):
        bb = ((g >> bit) & 1).astype(np.float32); bb -= bb.mean()
        for p, q in ((bb[:, :-1], bb[:, 1:]), (bb[:-1], bb[1:]), (bb[:-1, :-1], bb[1:, 1:]), (bb[:-1, 1:], bb[1:, :-1]), (bb[:, :-2], bb[:, 2:]), (bb[:-2], bb[2:])):
            d = np.sqrt((p*p).sum()*(q*q).sum())
            if d: worst = max(worst, abs((p*q).sum()/d))
    return worst
cands = [(1 << i) + (1 << j) + 1 for i in range(2, 15) for j in range(i + 1, 16)]
rng = np.random.default_rng(1)
best = []
for _ in range(400):
    c1, c2 = rng.choice(cands, 2)
    r1, r2 = rng.integers(5, 10, 2)
    g = run(int(c1), int(c2), int(r1), int(r2))
    sc = score(g)
    u = len(np.unique(g)) / g.size
    best.append((sc, -u, int(c1), int(c2), int(r1), int(r2)))
best.sort()
for b in best[:6]:
    sc, nu, c1, c2, r1, r2 = b
    g2 = run(c1, c2, r1, r2, s=0xC0DE)
    print(f"C1={c1:#06x} C2={c2:#06x} r1={r1} r2={r2}: worst corr {sc:.3f} (seed2 {score(g2):.3f}), unique {-nu:.3f}")

print("--- per-bit worst for the best 2-round candidate, then 3-round search ---")
def perbit(g):
    out = []
    for bit in range(16):
        bb = ((g >> bit) & 1).astype(np.float32); bb -= bb.mean(); w = 0
        for p, q in ((bb[:, :-1], bb[:, 1:]), (bb[:-1], bb[1:]), (bb[:-1, :-1], bb[1:, 1:]), (bb[:-1, 1:], bb[1:, :-1]), (bb[:, :-2], bb[:, 2:]), (bb[:-2], bb[2:])):
            d = np.sqrt((p*p).sum()*(q*q).sum()); w = max(w, abs((p*q).sum()/d)) if d else w
        out.append(round(w, 2))
    return out
_, _, c1, c2, r1, r2 = best[0]
print(perbit(run(c1, c2, r1, r2)))
def run3(c1, c2, c3, r1, r2, r3, s=0x3A91, a=x, b=y):
    v = ((a ^ s) + rotl(b, 8)) & M
    v = (v * c1) & M; v ^= v >> r1
    v = (v * c2) & M; v ^= v >> r2
    v = (v * c3) & M; v ^= v >> r3
    return v
best3 = []
for _ in range(500):
    c1, c2, c3 = (int(c) for c in rng.choice(cands, 3))
    r1, r2, r3 = (int(r) for r in rng.integers(5, 10, 3))
    g = run3(c1, c2, c3, r1, r2, r3)
    best3.append((score(g), c1, c2, c3, r1, r2, r3))
best3.sort()
for b in best3[:5]:
    sc, c1, c2, c3, r1, r2, r3 = b
    g2 = run3(c1, c2, c3, r1, r2, r3, s=0xC0DE)
    g3 = run3(c1, c2, c3, r1, r2, r3, s=0x0000)
    print(f"C={c1:#06x},{c2:#06x},{c3:#06x} r={r1},{r2},{r3}: worst {sc:.3f} seed2 {score(g2):.3f} seed0 {score(g3):.3f} unique {len(np.unique(g))/g.size:.3f}")
