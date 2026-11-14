#!/usr/bin/env python3
"""model.py -- integer model of the SUGARCOAT v2 fill (candy generator, grain, sprinkles, relief,
palette), mirroring sugarcoat.vhd operation for operation, plus an approximate model of the
matte pipeline (shared with VENEER / APPLIQUE) so the candies can be previewed on any image at
true 1080p scale without a simulation.

  python3 model.py IMAGE OUT.png [--theme N] [--bank] [--scale 0..1023] [--gloss 0..15]
                   [--shade 0..15] [--sprinkles 0..255] [--levels 2..8] [--luma] [--bias]
                   [--nolines] [--fixed]
  python3 model.py --sheet OUT.png [--scale K]      every candy, one swatch each
  python3 model.py --themes OUT.png [--scale K]     the 16 themes, class by class
  python3 model.py --stats                          mean luma / hue of every candy

Needs numpy + Pillow (the SDK image tester's venv has both)."""
import sys, pathlib
import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tables

W_ = np.where

# ----------------------------------------------------------------------------------------
# coordinates: virtual 1080-line units, Scale (K3) folded into the accumulator steps
# ----------------------------------------------------------------------------------------
M25 = (1 << 25) - 1

def vstep(scale_k):
    """pattern positions (1/64 cell, Q.10) per virtual pixel; scale_k = settled K3 (0..1023)"""
    kf = scale_k << 4
    o = kf >> 12; f = kf & 4095
    return (8192 - f) >> (1 + o)

def steps(scale_k, timing=13):
    v = vstep(scale_k)
    if timing in (14, 6, 5):   xs = v + (v >> 1)                 # 720p
    elif timing in (0, 8, 4, 12): xs = v << 1                    # SD
    else: xs = v
    ys = {0: (v << 2) + (v >> 1), 8: (v << 2) - (v >> 2), 4: (v << 1) + (v >> 2), 12: (v << 1) - (v >> 3),
          14: v + (v >> 1), 6: v + (v >> 1), 5: v + (v >> 1), 10: v << 1, 2: v << 1, 1: v << 1}.get(timing, v)
    return xs, ys

def coords(W, H, scale_k=384, timing=13):
    """the four 15-bit coordinates x, y, x+y, x-y of every pixel (T0)"""
    xs, ys = steps(scale_k, timing)
    xa = (np.arange(W, dtype=np.int64) * xs) & M25
    ya = (np.arange(H, dtype=np.int64) * ys) & M25
    X, Y = np.meshgrid(xa, ya)
    gu = (((X >> 8) + (Y >> 8)) & 0x1FFFF) >> 2
    gv = (((X >> 8) - (Y >> 8)) & 0x1FFFF) >> 2
    return X >> 10, Y >> 10, gu, gv

# ----------------------------------------------------------------------------------------
# hashes: 16-bit substitution / permutation network (4 rounds of 4-bit S-boxes: no carries)
# ----------------------------------------------------------------------------------------
SBOX = np.array([0xC, 5, 6, 0xB, 9, 0, 0xA, 0xD, 3, 0xE, 0xF, 8, 4, 7, 1, 2])
RC_CELL = (0xA53C, 0x3C96, 0x69A5, 0x5AC3)
RC_SPRK = (0x1D2E, 0xB74A, 0x52F1, 0xE08D)
RC_GRAIN = (0x7C15, 0x29E3, 0xC6B8, 0x914F)

def spn16(x, rc):
    x = np.asarray(x, np.int64)
    for r in range(4):
        x = x ^ rc[r]
        y = np.zeros_like(x)
        for i in range(4):
            y |= SBOX[(x >> (4 * i)) & 15] << (4 * i)
        if r == 3: return y
        z = np.zeros_like(x)
        for i in range(16):
            j = (i * 4) % 15 if i < 15 else 15
            z |= ((y >> i) & 1) << j
        x = z

def cell_key(ca, cb):
    return ca | (cb << 5) | ((ca & 7) << 10) | ((cb & 7) << 13)

# ----------------------------------------------------------------------------------------
# candy generator: recipe words -> per pixel reg (0 ground A, 1 piece B, 2 detail C, 3 white),
# k (assortment / band index), ht (relief height), sv (signed shade), spec (white highlight)
# ----------------------------------------------------------------------------------------
def tri4(x):
    """5-bit phase -> 4-bit triangle"""
    return W_((x >> 4) & 1, ~x & 15, x & 15)

FACET = np.array([13, 11, 10, 8, 7, 5, 4, 2])      # gem facets by (na, nb, hg)

def candy(g0, g1, cx, cy, cu, cv, angle_sw=False):
    shape = g0 & 31; v = (g0 >> 5) & 3; size = (g0 >> 7) & 3
    bias = ((g0 >> 9) & 1) ^ (1 if angle_sw else 0); turn = (g0 >> 10) & 1
    stg = (g0 >> 11) & 1; hst = (g0 >> 12) & 1; wide = (g0 >> 13) & 1; jit = (g0 >> 14) & 3
    shd = g1 & 3; rot = (g1 >> 5) & 1; h1d = (g1 >> 6) & 1
    # C1: axes
    if bias: a, b = (cv, cu) if turn else (cu, cv)
    else:    a, b = (cy, cx) if turn else (cx, cy)
    # C2: size (11-bit field: 5 cell bits + 6 position bits), stagger
    fa = (a >> size) & 2047; fb = (b >> size) & 2047
    if wide:
        pb = (fb & 31) << 1; cb = (fb >> 5) & 31
    else:
        pb = fb & 63; cb = (fb >> 6) & 31
    st = 0
    if stg: st = st ^ ((fb >> (5 if wide else 6)) & 1)
    if hst: st = st ^ ((fb >> 5) & 1)
    fa = (fa + 32 * st) & 2047
    pa = fa & 63; ca = (fa >> 6) & 31
    ia0 = ca & 1; ib0 = cb & 1
    # C3: cell hash
    h = spn16(cell_key(ca, cb * (0 if h1d else 1)), RC_CELL)
    hk = h & 7
    # C4: jitter (sign-extended slices of the hash), folds, random quarter turn
    def sx(x, n): return x - ((x >> (n - 1)) & 1) * (1 << n)
    if jit == 0:   ja = 0; jb = 0
    elif jit == 1: ja = sx((h >> 3) & 7, 3);  jb = sx((h >> 8) & 7, 3)
    elif jit == 2: ja = sx((h >> 3) & 15, 4); jb = sx((h >> 8) & 15, 4)
    else:          ja = sx((h >> 3) & 31, 5); jb = sx((h >> 8) & 31, 5)
    pa = (pa + ja) & 63; pb = (pb + jb) & 63
    na = 1 - ((pa >> 5) & 1); nb = 1 - ((pb >> 5) & 1)
    da = W_(pa & 32, pa & 31, ~pa & 31); db = W_(pb & 32, pb & 31, ~pb & 31)
    sw = ((h >> 13) & 1) if rot else 0
    da, db = W_(sw, db, da), W_(sw, da, db)
    na, nb = W_(sw, nb, na), W_(sw, na, nb)
    lmode = 0 if not bias else (2 if turn else 1)      # which axis faces the light
    # C5: features (every carry chain)
    sqa = da * da; sqb = db * db
    r2 = sqa + sqb; r2e = sqa + 4 * sqb
    l1 = da + db; dd = da - db
    hg = (da > db).astype(np.int64)
    w1 = da + (db >> 1); w2 = db + (da >> 1)
    sm = pa + pb; df = (pa - pb) & 63
    ch = (pb + da) & 63; sc = ((~pb & 31) - (sqa >> 5)) & 31
    gs = ((pb >> 1) > da).astype(np.int64)
    # chip radius (by variant and hash) and the three chip compares
    hr = (h >> 11) & 3
    cs = hr if v == 3 else [3, 1, 0, 0][v]           # chip size index
    cr = np.array([10, 12, 17, 20])[cs]; cr2 = np.array([8, 9, 13, 15])[cs]
    cp1 = w1 < cr; cp2 = w2 < cr; cpa = da < cr2; cpb = db < cr2
    # C6: shape
    oct_ = W_(hg, w1, w2)
    li = W_(hg, da, db)
    same = na == nb
    if lmode == 0:
        lit = W_(same, W_(na, l1, ~l1), W_(na, dd, ~dd))
    else:
        useb = (sw == 1) ^ (lmode == 2)                # the axis facing the light, after the quarter turn
        d_ = W_(useb, db, da); n_ = W_(useb, nb, na)
        lit = W_(n_, d_ << 1, ~(d_ << 1))
    lita = W_(na, da, ~da)
    L7 = lit >= 8; L10 = lit >= 12; L14 = lit >= 16      # (named after the first thresholds tried)
    q = lambda n: r2 < n
    e = lambda n: r2e < n
    wdg = na ^ nb ^ hg
    Z = np.zeros_like(pa)
    reg = Z.copy(); k = hk + Z; ht = Z.copy(); lsel = Z.copy(); lsh = Z.copy()
    limb = Z.copy(); spc = Z.copy(); r4 = Z + 8; linv = Z.copy()
    B = lambda x: np.asarray(x).astype(np.int64)
    S = tables.SHAPES[shape] if shape < len(tables.SHAPES) else 'full'
    if S == 'full':
        reg = Z + 1
        if v == 1:   lsel = Z + 3; r4 = tri4(ch & 31)
        elif v == 2: lsel = Z + 1
        elif v == 3: lsel = Z + 3; r4 = 8 | (tri4((sm >> 1) & 31) >> 2)
    elif S == 'disc':
        big = (h >> 14) & 1
        ins = [q(768), q(512), q(256), W_(big, q(512), q(256))][v]
        lim = [~q(512), ~q(384), ~q(192), W_(big, ~q(384), ~q(192))][v]
        sp = [L14 & q(256), L10 & q(128), L7 & q(64), W_(big, L10 & q(128), L7 & q(64))][v]
        reg = B(ins); ht = 2 * B(ins); lsel = Z + 1; lsh = Z + (0 if v == 0 else 1)
        limb = B(ins & lim); spc = B(sp)
    elif S == 'ring':
        if v == 0:
            ins = q(768); reg = W_(q(256), 2, B(ins)); ht = W_(q(256), 3, 2 * B(ins)); limb = B(ins & ~q(512)); spc = B(L7 & q(128))
        elif v == 1:
            ins = q(768) & ~q(128); reg = B(ins); ht = 2 * B(ins); limb = B(ins & (~q(512) | q(192))); spc = B(L14 & q(512) & ~q(256))
        elif v == 2:
            ins = q(512); reg = W_(q(128), 2, B(ins)); ht = W_(q(128), 3, 2 * B(ins)); limb = B(ins & ~q(384)); spc = B(L7 & q(64))
        else:
            ins = q(512) & ~q(64); reg = B(ins); ht = 2 * B(ins); limb = B(ins & ~q(384)); spc = B(L10 & q(384) & ~q(192))
        lsel = Z + 1; lsh = Z + (0 if v < 2 else 1)
    elif S == 'bean':
        big = (h >> 14) & 1
        ins = [e(768), e(512), e(256), W_(big, e(768), e(512))][v]
        lim = [~e(512), ~e(384), ~e(192), W_(big, ~e(512), ~e(384))][v]
        sp = [L10 & e(192), L7 & e(128), L7 & e(64), W_(big, L10 & e(192), L7 & e(128))][v]
        reg = B(ins); ht = 2 * B(ins); lsel = Z + 1; lsh = Z + 1; limb = B(ins & lim); spc = B(sp)
    elif S == 'chip':
        sil = (h >> 14) & 3
        circ = W_(cs == 0, q(128), W_(cs == 1, q(192), W_(cs == 2, q(384), q(512))))
        ins = circ & W_(sil == 1, cpb, True) & W_(sil == 2, cpa, True) & W_(sil == 3, cp1 & cp2, True)
        ins = ins & (((h >> 9) & 3) != 0)
        reg = B(ins); ht = 3 * B(ins); lsel = Z + 1; lsh = Z + 1; spc = B(ins & L7 & (l1 < 6))
    elif S == 'square':
        ins = [W_((h >> 14) & 1, li < 30, li < 28) & W_((h >> 15) & 1, l1 < 54, l1 < 46), (li < 21) & (l1 < 36), (li < 15) & (l1 < 26), (li < 23) & (l1 < 40)][v]
        lim = [li >= 26, li >= 18, li >= 12, li >= 20][v]
        reg = B(ins)
        if v == 3: reg = W_(ins, W_(db < 8, 2, 1), 0)
        ht = 3 * B(ins); lsel = Z + 1; lsh = Z + (1 if v == 2 else 0); limb = B(ins & lim); spc = B(ins & L10 & (li < 12))
    elif S == 'wedge':
        if v == 0 or v == 2 or v == 3:
            ins = q(512) if v == 2 else q(768)
            w = (na ^ nb) if v == 3 else wdg
            rim = ~q(384) if v == 2 else ~q(640)
            reg = W_(ins, W_(w & ~rim, 1, 3), 0); ht = 2 * B(ins); lsel = Z + 1; lsh = Z + (1 if v == 2 else 0)
            limb = B(ins & (~q(384) if v == 2 else ~q(512))); spc = B((L10 & q(128)) if v == 2 else (L14 & q(256)))
        else:
            ins = q(768); rind = ins & ~q(512)
            mem = ((dd >= -1) & (dd <= 1)) | (da < 2) | (db < 2)
            reg = W_(ins, W_(~rind & mem, 3, 1), 0); ht = 2 * B(ins); lsel = Z + 3
            r4 = W_(rind, 6, W_(mem, 8, 12))
    elif S == 'target':
        if v == 0:
            ins = q(768); reg = B(ins); k = (oct_ >> 2) & 7; ht = 2 * B(ins); lsel = Z + 1
            limb = B(ins & ~q(512)); spc = B(L14 & q(256))
        elif v == 1:
            ins = oct_ < 28; reg = W_(ins, W_((oct_ >> 2) & 1, 1, 2), 0); ht = 2 * B(ins); lsel = Z + 1; limb = B(ins & (oct_ >= 24))
        else:
            ins = q(768); reg = B(ins); ht = 2 * B(ins); lsel = Z + 3
            crimp = W_(((l1 >> 2) ^ (dd >> 2)) & 1, 12, 4)
            if v == 2: inner = W_(q(256) & ~q(192), 5, 8)
            else:      inner = W_(q(64), 8, W_(q(384), W_((oct_ >> 1) & 1, 11, 6), 8))
            r4 = W_(q(512), inner, crimp)
    elif S == 'heart':
        hs = W_(nb, dd, l1); hd = W_(nb, l1, dd)
        if v < 2:
            ins = (hs < 26) & (hs >= -10) & ((nb == 0) | (db < 16)) & (da < 23) & (hd < 34)
            reg = B(ins); ht = 2 * B(ins); lsel = Z + 1; spc = B(ins & L14 & (l1 < 22) & (na == 1) & (nb == 1))
            if v == 1:
                txt = ((nb == 1) & (db >= 4) & (db < 6) & (da < 10)) | ((nb == 0) & (db >= 2) & (db < 4) & (da < 7))
                reg = W_(ins & txt, 2, reg); spc = Z.copy()
        else:
            ins = (l1 < 28) if v == 2 else (li < 22)
            reg = B(ins); ht = 3 * B(ins); lsel = Z + 3
            r4 = FACET[(1 - na) * 4 + (1 - nb) * 2 + (1 - hg)]
            if v == 3: r4 = W_(li < 10, 10, r4)
            spc = B(ins & (na == 1) & (nb == 1) & (l1 < 7))
    elif S == 'corn':
        ins = gs == 1
        if v == 0:
            band = pb >> 4
            reg = W_(ins, W_(band == 3, 1, W_(band == 2, 2, 3)), 0); lsel = Z + 1
        elif v == 1:
            reg = B(ins); lsel = Z + 3; r4 = W_(na, 11, 5)
        elif v == 2:
            reg = B(ins); lsel = Z + 1
        else:
            reg = W_(ins, W_(pb >= 56, 2, W_(pb >= 52, 3, 1)), 0); lsel = Z + 1
        ht = (3 if v == 1 else 2) * B(ins)
    elif S == 'stripe':
        if v == 0:   reg = (pa >> 5) & 1
        elif v == 1: reg = W_(pa < 24, 1, W_(((pa >= 36) & (pa < 42)) | ((pa >= 50) & (pa < 56)), 2, 0))
        elif v == 2: reg = np.array([1, 0, 2, 3])[pa >> 4]
        else:        reg = Z + 1; k = pa >> 3
    elif S == 'tube':
        ins = da < 29
        s6 = sm & 63
        if v == 0:   reg = B(ins)
        elif v == 1: reg = W_(ins, W_(s6 < 20, 1, W_(((s6 >= 28) & (s6 < 34)) | ((s6 >= 42) & (s6 < 48)), 2, 3)), 0)
        elif v == 2: reg = B(ins)
        else:        reg = W_(ins, W_(da < 5, 2, 1), 0)
        ht = 2 * B(ins); lsel = Z + 2; lsh = Z + 1; limb = B(ins & (da >= 25)); spc = B((na == 1) & (da >= 11) & (da < 14))
        if v == 2:
            lsel = Z + 3; r4 = tri4(sm & 31); spc = B((na == 1) & (da >= 8) & (da < 16) & (r4 == 15))
    elif S == 'check':
        line = ((pa & 31) < 2) | ((pb & 31) < 2)
        if v == 0:   reg = W_(line, 0, W_(((pa ^ pb) >> 5) & 1, 1, 2)); ht = W_(line, 0, 2)
        elif v == 1: reg = W_(((pa ^ pb) >> 5) & 1, 1, 2)
        elif v == 2: reg = W_(((pa ^ pb) >> 4) & 1, 1, 2)
        else:        reg = np.array([0, 2, 1])[((pa >> 5) & 1) + ((pb >> 5) & 1)]
    elif S == 'pips':
        reg = Z + 1; lsel = Z + 3
        ht = W_(li < 25, 3, W_(li < 27, 2, W_(li < 29, 1, 0)))
        frame = (li >= 14) & (li < 16)
        if v == 0 or v == 3: r4 = W_(frame, 5, 8)
        if v == 1 or v == 3: reg = W_(q(64), 2, 1)
    elif S == 'wafer':
        if v == 0 or v == 2: ridge = (da >= 26) | (db >= 26)
        else:
            fa5 = tri4(pa & 31); fb5 = tri4(pb & 31)
            ridge = (fa5 >= 13) | (fb5 >= 13)
        reg = W_(ridge, 1, 2 if v >= 2 else 1); ht = 3 * B(ridge); lsel = Z + 3; r4 = W_(ridge, 10, 6)
    elif S == 'ridge':
        two = ia0 if v == 1 else ((pa >> 5) & 1)
        reg = W_(two, 1, 2) if v in (1, 3) else Z + 1
        lsel = Z + 2; lsh = Z + 1
        limb = B((da >= 27) | ((da < 2) if v >= 2 else False))
        ht = W_(da >= 27, 0, 2); spc = B((na == 1) & (da >= 11) & (da < 14))
    elif S == 'zigzag':
        c5 = ch & 31
        if v == 0:   reg = Z + 1; lsel = Z + 3; r4 = tri4(c5)
        elif v == 1: dz = c5 < 5; reg = W_(dz, 2, 1); ht = 3 * B(dz)
        elif v == 2: reg = W_((ch >> 4) & 1, 1, 2)
        else:        dz = (ch < 6) | ((ch >= 30) & (ch < 33)); reg = W_(dz, 2, 1); ht = 3 * B(dz)
    elif S == 'scallop':
        edge = (sc >> 2) == 0
        if v == 0:   reg = W_(edge, 2, 1); ht = 3 * B(edge)
        elif v == 1: reg = Z + 1; lsel = Z + 3; r4 = sc >> 1; ht = W_(edge, 0, 2)
        elif v == 2: reg = W_((sc >> 4) & 1, 1, 2)
        else:        reg = np.array([1, 0, 2, 0])[sc >> 3]
    elif S == 'basket':
        d = (ia0 ^ ib0) if v == 2 else (((pa ^ pb) >> 5) & 1)
        x = W_(d == 1, pa, pb)
        reg = Z + 1; lsel = Z + 3
        if v == 0:   r4 = x & 15
        elif v == 1: r4 = W_(((x >> 1) & 7) == 0, 2, 10)
        elif v == 2: r4 = (x >> 1) & 15
        else:
            sa_ = (pa & 31) >= 12; sb_ = (pb & 31) >= 12
            top = ((pa ^ pb) >> 5) & 1
            reg = W_(sa_ | sb_, 1, 2); r4 = Z + 9
            ht = np.maximum(W_(sa_, W_(top, 3, 2), 0), W_(sb_, W_(top, 2, 3), 0))
    elif S == 'quilt':
        line = ((df >> 2) == 0) | (((sm >> 2) & 15) == 15)
        reg = Z + 1; lsel = Z + 3
        if v != 3:
            r4 = W_(line, 5, 8); ht = W_(line, 0, 2)
        pearl = q(64) if v == 3 else (q(32) if v >= 1 else (Z == 1))
        if v == 2: pearl = pearl | ((da >= 27) & (db >= 27))
        reg = W_(pearl, 3, reg); ht = W_(pearl, 3, ht); lsel = W_(pearl, 1, lsel); lsh = B(pearl)
        spc = B(pearl & L7 & q(16)) if v == 3 else Z.copy()
    elif S == 'rosette':
        ins = q(960)
        nb3 = (~(oct_ >> 2) & 7) << 1
        if v == 0:   reg = B(ins); lsel = Z + 3; r4 = nb3 | ((oct_ >> 1) & 1)
        elif v == 1: reg = B(ins); lsel = Z + 3; r4 = nb3 | wdg
        elif v == 2: reg = W_(ins, W_((oct_ >> 2) & 1, 1, 2), 0); lsel = Z + 3; r4 = nb3
        else:        reg = B(ins); lsel = Z + 1; spc = B(q(16))
        ht = 2 * B(ins)
    elif S == 'holes':
        hs2 = (h >> 14) & 3
        hole = [W_(hs2 == 0, q(256), W_(hs2 == 1, q(128), W_(hs2 == 2, q(64), Z == 1))), q(64),
                W_(hs2 & 1, q(128), q(64)), q(256)][v]
        reg = W_(hole, 2, 1); ht = W_(hole, 0, 2); lsel = B(hole); lsh = Z + 1; linv = B(hole)
    # C7: shade.  base light term, shading model, constants
    L = W_(lsel == 1, lit, W_(lsel == 2, lita << 1, W_(lsel == 3, ((r4 & 15) - 8) << 3, 0)))
    L = W_((lsh == 1) & (lsel != 3), L << 1, L)
    L = W_(linv == 1, ~L, L)
    L = W_(reg == 0, 0, L)
    piece = reg != 0
    if shd == 0:   sv = L >> 1
    elif shd == 3: sv = W_(L < 0, ~L, L)
    else:          sv = L
    cst = W_(limb == 1, -32, 0) + (-24 if shd == 3 else 0)
    sv = sv + cst
    spec = (spc == 1) & (shd >= 2)
    return dict(reg=reg, k=k & 7, ht=ht, sv=sv, spec=spec, piece=piece)

# ----------------------------------------------------------------------------------------
# grain (straight coordinates, 4-position micro cells) and the sprinkle overlay (64-position cells)
# ----------------------------------------------------------------------------------------
def grain(cx, cy):
    g = spn16(((cx >> 2) & 255) | (((cy >> 2) & 255) << 8), RC_GRAIN)
    dot = ((cx & 3) != 0) & ((cy & 3) != 0)
    return dict(dark=dot & ((g & 7) == 0), light=dot & (((g >> 3) & 15) == 0), np=dot & (((g >> 7) & 3) == 0), k=(g >> 9) & 7)

def sprinkles(cx, cy, dens):
    """dens 0..255 (array or scalar): a cell carries a sprinkle when its hash byte is below it"""
    pa = cx & 63; pb = cy & 63
    h = spn16(cell_key((cx >> 6) & 31, (cy >> 6) & 31), RC_SPRK)
    sx = lambda x: x - ((x >> 3) & 1) * 16
    pa = (pa + sx((h >> 3) & 15)) & 63; pb = (pb + sx((h >> 7) & 15)) & 63
    na = 1 - ((pa >> 5) & 1); nb = 1 - ((pb >> 5) & 1)
    da = W_(pa & 32, pa & 31, ~pa & 31); db = W_(pb & 32, pb & 31, ~pb & 31)
    l1 = da + db; dd = da - db; ad = W_(dd < 0, ~dd, dd)
    same = na == nb
    big = W_(same, l1, ad); small = W_(same, ad, l1)
    o = (h >> 11) & 3
    rod = W_(o == 0, (da < 12) & (db < 3), W_(o == 1, (db < 12) & (da < 3),
          W_(o == 2, (small < 4) & (big < 17), (big < 4) & (small < 17))))
    on = rod & (((h >> 8) & 255) < dens)
    return on, h & 7

# ----------------------------------------------------------------------------------------
# relief (as APPLIQUE): height against the sample 2..3 pixels left and two lines up
# ----------------------------------------------------------------------------------------
def relief_of(ht, il=0):
    H, W = ht.shape
    dy = 1 if il else 2
    up = np.vstack([ht[:1].repeat(dy, axis=0), ht[:-dy]])
    xs = np.maximum((np.arange(W) - 2) & ~1, 0)
    rel = 21 * (ht - up[:, xs])
    rel[:, :12] = 0
    return rel

# ----------------------------------------------------------------------------------------
# fill: class map (H x W, candy class 0..15) -> 10-bit Y, U, V
# ----------------------------------------------------------------------------------------
PALW = np.array(tables.pal_words())

def render(cand, cx, cy, cu, cv, gloss=9, sprk=0, angle=False, il=0, ybias=None):
    """cand: H x W candy ids.  ybias: optional H x W luma offset (the picture's shade)."""
    H, W = cand.shape
    reg = np.zeros((H, W), np.int64); k = reg.copy(); ht = reg.copy(); sv = reg.copy()
    spec = np.zeros((H, W), bool); idx = reg.copy(); dens = reg + sprk
    gr = grain(cx, cy)
    for ci in np.unique(cand):
        c = tables.CANDIES[ci]; mk = cand == ci
        o = candy(c['g0'], c['g1'], cx, cy, cu, cv, angle)
        g1 = c['g1']; grn = (g1 >> 2) & 3; gp = (g1 >> 4) & 1; shd = g1 & 3
        A = c['c0'] & 63; Bc = (c['c0'] >> 6) & 63; asst = (c['c0'] >> 12) & 3; C = c['c1'] & 63
        m = (1 << asst) - 1
        r = o['reg']
        ix = W_(r == 0, A, W_(r == 1, (Bc & ~m) | (o['k'] & m), W_(r == 2, C, tables.WHITE)))
        s = o['sv'].copy(); sp = o['spec'].copy()
        gz = (o['piece'] == (gp == 1))              # grain zone
        if grn == 1:
            ix = W_(gz & gr['dark'], C, ix); s = s + W_(gz & gr['light'] & ~gr['dark'], 40, 0)
        elif grn == 2:
            s = s + W_(gz & gr['light'], 72, 0)
            sp = sp | (gz & gr['light'] & gr['dark'])
        elif grn == 3:
            ix = W_(gz & gr['np'], gr['k'], ix)
        if (g1 >> 7) & 1: dens[mk] = np.maximum(dens[mk], 150)
        reg[mk] = r[mk]; ht[mk] = o['ht'][mk]; sv[mk] = s[mk]; spec[mk] = sp[mk]; idx[mk] = ix[mk]
    # sprinkle overlay
    on, sk = sprinkles(cx, cy, dens)
    idx = W_(on, sk, idx); ht = W_(on, 3, ht); sv = W_(on, 16, sv); spec = spec & ~on
    if gloss < 2: spec = spec & False
    idx = W_(spec, tables.WHITE, idx); sv = W_(spec, 0, sv)
    rel = relief_of(ht, il)
    e = sv + 2 * rel
    w = PALW[idx]
    Y = ((w >> 10) & 63) << 4; U = ((w >> 5) & 31) << 5; V = (w & 31) << 5
    Y = Y + ((e * gloss) >> 3)
    if ybias is not None: Y = Y + ybias
    return np.clip(Y, 64, 940), U, V

def to_rgb(Y, U, V):
    f = lambda q: (np.hstack([q[:, :1], q[:, :-1]]) + 2 * q + np.hstack([q[:, 1:], q[:, -1:]])) >> 2
    Y, U, V = f(Y), f(U), f(V)
    d = np.maximum(Y - 64, 0); Y = Y - (d >> 3) - (d >> 4)
    y = (np.clip(Y, 64, 940) - 64) / 876.0; cb = (U - 512) / 896.0; cr = (V - 512) / 896.0
    rgb = np.stack([y + 1.402 * cr, y - 0.344136 * cb - 0.714136 * cr, y + 1.772 * cb], -1)
    return (np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8)

# ----------------------------------------------------------------------------------------
# class map (as p_h stage 29 -> 30): label {lum, hue, sat} -> class 0..15
# ----------------------------------------------------------------------------------------
SPREAD = {2: [0, 7], 3: [0, 4, 7], 4: [0, 2, 5, 7], 5: [0, 2, 4, 5, 7], 6: [0, 1, 3, 4, 6, 7],
          7: [0, 1, 2, 4, 5, 6, 7], 8: [0, 1, 2, 3, 4, 5, 6, 7]}
HMASK = {2: 4, 3: 4, 4: 6, 5: 6, 6: 7, 7: 7, 8: 7}

def classes(lab, nlev, luma_map=False):
    lum = np.array(SPREAD[nlev] + [7] * (8 - nlev))[lab['lum']]
    if luma_map: return lum
    return W_(lab['sat'] != 0, 8 + (lab['hue'] & HMASK[nlev]), lum)

def theme_candies(theme, bank=0):
    _, luma, hue = tables.THEMES[bank * 8 + theme]
    return np.array(luma + hue)

# ----------------------------------------------------------------------------------------
# approximate matte pipeline (labels), as APPLIQUE / VENEER model.py
# ----------------------------------------------------------------------------------------
FDEL = {2: 110, 3: 73, 4: 55, 5: 44, 6: 37, 7: 31, 8: 27}

def _mode5(a, axis):
    pads = [(0, 0), (0, 0)]; pads[axis] = (2, 2)
    p = np.pad(a, pads, mode='edge'); n = a.shape[axis]
    taps = [np.take(p, range(k, k + n), axis=axis) for k in range(5)]
    cnt = [sum((taps[i] == taps[j]).astype(np.int32) for j in range(5)) for i in range(5)]
    best = taps[2].copy(); bc = cnt[2].copy()
    for i in (1, 3, 0, 4):
        mm = cnt[i] > bc
        best[mm] = taps[i][mm]; bc[mm] = cnt[i][mm]
    return best

def mattes(img, nlev=4, adaptive=True, level=0):
    a = np.asarray(img.convert('RGB')).astype(np.float64) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    Yf = np.clip((0.299 * r + 0.587 * g + 0.114 * b) * 219 + 16, 16, 235).astype(np.int32)
    Uf = np.clip((-0.168736 * r - 0.331264 * g + 0.5 * b) * 224 + 128, 16, 240).astype(np.int32)
    Vf = np.clip((0.5 * r - 0.418688 * g - 0.081312 * b) * 224 + 128, 16, 240).astype(np.int32)
    H, W = Yf.shape; W2 = W // 2
    pa = lambda q: (q[:, 0:2 * W2:2] + q[:, 1:2 * W2:2] + 1) >> 1
    y, u, v = pa(Yf), pa(Uf), pa(Vf)
    p = np.pad(y, 1, mode='edge')
    c3 = p[:-2, :] + 2 * p[1:-1, :] + p[2:, :]
    ys = (c3[:, :-2] + 2 * c3[:, 1:-1] + c3[:, 2:]) >> 4
    if adaptive:
        hist = np.bincount((ys >> 2).ravel(), minlength=64)[:64]; cum = np.cumsum(hist); tot = cum[-1]
        thr = [min(235, int(np.searchsorted(cum, tot * k / nlev)) * 4 + 4) for k in range(1, nlev)]
        cen = [min(235, int(np.searchsorted(cum, tot * (k + 0.5) / nlev)) * 4 + 2) for k in range(nlev)]
    else:
        d = FDEL[nlev]; thr = [16 + d * k for k in range(1, nlev)]; cen = [16 + d * k + d // 2 for k in range(nlev)]
    ysl = np.clip(ys + level, 0, 255)                 # K5 Level: after the histogram, before the quantizer
    lum = np.zeros_like(ys)
    for t in thr: lum += (ysl > t).astype(np.int32)
    W4 = W2 // 2
    uq = (u[:, 0:2 * W4:2] + u[:, 1:2 * W4:2] + 1) >> 1; vq = (v[:, 0:2 * W4:2] + v[:, 1:2 * W4:2] + 1) >> 1
    def box(q):
        q2 = q + np.vstack([q[:1], q[:-1]])
        return (q2 + np.hstack([q2[:, :1], q2[:, :-1]])) >> 2
    du = box(uq) - 128; dv = box(vq) - 128; au, av = np.abs(du), np.abs(dv)
    sn = np.maximum(au, av) + (np.minimum(au, av) >> 1); gt = au > av
    hue = W_((dv >= 0) & (du >= 0), W_(gt, 0, 1), W_((dv >= 0) & (du < 0), W_(gt, 3, 2),
          W_((dv < 0) & (du < 0), W_(gt, 4, 5), W_(gt, 7, 6))))
    sat = W_(sn < 10, 0, W_(sn < 36, 1, 2))
    hue = np.repeat(hue, 2, axis=1)[:, :W2]; sat = np.repeat(sat, 2, axis=1)[:, :W2]
    chroma = hue * 4 + sat
    lum = _mode5(_mode5(lum, 1), 0); chroma = _mode5(_mode5(chroma, 1), 0)
    up = lambda q: np.repeat(q, 2, axis=1)
    return dict(lum=up(lum), hue=up(chroma >> 2), sat=up(chroma & 3)), np.array(cen + [235] * (8 - len(cen)))

def picture(img, theme=0, bank=0, nlev=5, luma_map=False, scale_k=384, gloss=9, shade=6, sprk=0,
            angle=False, lines=True, adaptive=True, level=0):
    lab, cen = mattes(img, nlev, adaptive, level)
    H, W = lab['lum'].shape
    cls = classes(lab, nlev, luma_map)
    cand = theme_candies(theme, bank)[cls]
    cx, cy, cu, cv = coords(W, H, scale_k)
    yb = ((cen[lab['lum']] - 128) * shade) >> 2          # K4 Shade: (cen - 128) x shade / 4, 10-bit units
    Y, U, V = render(cand, cx, cy, cu, cv, gloss, sprk, angle, 0, yb)
    if lines:
        bd = (cls != np.hstack([cls[:, :2], cls[:, :-2]])) | (cls != np.vstack([cls[:1], cls[:-1]]))
        Y = W_(bd, 64, Y); U = W_(bd, 512, U); V = W_(bd, 512, V)
    return to_rgb(Y, U, V)

# ----------------------------------------------------------------------------------------
# contact sheets
# ----------------------------------------------------------------------------------------
def swatch(ci, S, scale_k, gloss=9, sprk=0, angle=False):
    cx, cy, cu, cv = coords(S, S, scale_k)
    Y, U, V = render(np.full((S, S), ci, np.int64), cx, cy, cu, cv, gloss, sprk, angle)
    return to_rgb(Y, U, V)

def sheet(out, scale_k=384, S=224, cols=8, ids=None, gloss=9, sprk=0, angle=False):
    ids = list(range(len(tables.CANDIES))) if ids is None else ids
    rows = (len(ids) + cols - 1) // cols
    img = Image.new('RGB', (cols * (S + 6) + 6, rows * (S + 18) + 6), (24, 24, 24))
    d = ImageDraw.Draw(img)
    for n, ci in enumerate(ids):
        x = 6 + (n % cols) * (S + 6); y = 6 + (n // cols) * (S + 18)
        img.paste(Image.fromarray(swatch(ci, S, scale_k, gloss, sprk, angle)), (x, y + 12))
        d.text((x, y), f'{ci} {tables.CANDIES[ci]["name"]}'[:36], fill=(235, 235, 235))
    img.save(out)

def themes_sheet(out, scale_k=384, S=150):
    n = len(tables.THEMES)
    img = Image.new('RGB', (16 * (S + 4) + 4, n * (S + 16) + 4), (24, 24, 24))
    d = ImageDraw.Draw(img)
    for ti, (name, luma, hue) in enumerate(tables.THEMES):
        for j, ci in enumerate(luma + hue):
            img.paste(Image.fromarray(swatch(ci, S, scale_k)), (4 + j * (S + 4), 16 + ti * (S + 16)))
        d.text((6, 3 + ti * (S + 16)), f'{ti} {name}   (8 luma slots dark -> light, then 8 hue sectors)', fill=(235, 235, 235))
    img.save(out)

def stats(scale_k=384, S=256):
    """mean 10-bit luma and mean hue sector of every candy (for sorting the themes)"""
    res = []
    cx, cy, cu, cv = coords(S, S, scale_k)
    for ci, c in enumerate(tables.CANDIES):
        Y, U, V = render(np.full((S, S), ci, np.int64), cx, cy, cu, cv, 9, 0)
        du = U.mean() - 512; dv = V.mean() - 512
        ang = (np.degrees(np.arctan2(dv, du)) + 360) % 360
        res.append((ci, c['name'], Y.mean(), np.hypot(du, dv), ang, int(ang // 45)))
    return res

if __name__ == '__main__':
    a = sys.argv[1:]
    opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    if a[0] == '--sheet':
        ids = [int(x) for x in opt('--ids', '').split(',')] if '--ids' in a else None
        sheet(a[1], opt('--scale', 384), opt('--size', 224), opt('--cols', 8), ids, opt('--gloss', 9),
              opt('--sprinkles', 0), '--bias' in a); print('wrote', a[1]); sys.exit(0)
    if a[0] == '--themes':
        themes_sheet(a[1], opt('--scale', 384)); print('wrote', a[1]); sys.exit(0)
    if a[0] == '--stats':
        import json
        st = stats()
        for r in sorted(st, key=lambda r: r[2]):
            print(f'{r[0]:3d} {r[1]:26s} Y {r[2]:5.0f}  sat {r[3]:4.0f}  hue {r[4]:4.0f} deg  sector {r[5]}')
        (pathlib.Path(__file__).resolve().parent / 'candy_luma.json').write_text(json.dumps({r[1]: round(float(r[2])) for r in st}, indent=0))
        sys.exit(0)
    img = Image.open(a[0]).convert('RGB').resize((1920, 1080), Image.LANCZOS)
    rgb = picture(img, opt('--theme', 0), 1 if '--bank' in a else 0, opt('--levels', 5), '--luma' in a,
                  opt('--scale', 384), opt('--gloss', 9), opt('--shade', 6), opt('--sprinkles', 0),
                  '--bias' in a, '--nolines' not in a, '--fixed' not in a, opt('--level', 0))
    Image.fromarray(rgb).save(a[1])
    print('wrote', a[1])
