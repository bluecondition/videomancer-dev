#!/usr/bin/env python3
"""CUBIST layer-turn model: stickers, quarter-turn permutations, box faces.

The turning cube is drawn as two boxes: the 3x3x1 SLICE being turned (by
theta about the turn face's normal axis) and the 3x3x2 BLOCK that stays put.
Everything here is in the cube's own frame (cubie units, cube spans +/-1.5),
using the same face tables as the microcode and the same rotation sense as
the renderer: the slice's local axis b goes to cos(t) b + sin(t) c, with
(a, b, c) cyclic.

Generates the ROM contents of the sticker/table block RAM (see TABLE_*).
"""
import itertools

# face tables (as frame_ucode.py): axis codes 0..5 = +x,-x,+y,-y,+z,-z
C_FN = [2, 3, 0, 1, 4, 5]   # normal axis code  (faces U D R L F B)
C_FU = [4, 0, 2, 4, 0, 2]   # U axis code
C_FV = [0, 4, 4, 2, 2, 0]   # V axis code
FACE_OF_NORMAL = {c: f for f, c in enumerate(C_FN)}


def axis_vec(code):
    v = [0, 0, 0]
    v[code >> 1] = -1 if code & 1 else 1
    return v


def add(a, b): return [x + y for x, y in zip(a, b)]
def scale(a, k): return [x * k for x in a]


def rot90(p, axis, d):
    """Rotate p by d*90 degrees about cube axis `axis` (right-handed)."""
    b, c = (axis + 1) % 3, (axis + 2) % 3
    q = list(p)
    if d > 0:            # b -> c, c -> -b
        q[c], q[b] = p[b], -p[c]
    else:                # b -> -c, c -> b
        q[c], q[b] = -p[b], p[c]
    return q


def sticker_geom():
    """(centre, normal) of each of the 54 stickers, index 9F + 3j + i."""
    out = []
    for f in range(6):
        n, u, v = axis_vec(C_FN[f]), axis_vec(C_FU[f]), axis_vec(C_FV[f])
        o = add(add(scale(n, 1.5), scale(u, -1.5)), scale(v, -1.5))
        for j in range(3):
            for i in range(3):
                c = add(add(o, scale(u, i + 0.5)), scale(v, j + 0.5))
                out.append((tuple(c), tuple(n)))
    return out


STICKERS = sticker_geom()
INDEX = {g: k for k, g in enumerate(STICKERS)}


def turn_perm(face, d):
    """dst <- src pairs for a quarter turn of `face` in direction d."""
    code = C_FN[face]
    axis, side = code >> 1, (-1 if code & 1 else 1)
    moves = []
    for src, (c, n) in enumerate(STICKERS):
        if c[axis] * side > 0.5:                       # in the slice
            dst = INDEX[(tuple(rot90(c, axis, d)), tuple(rot90(n, axis, d)))]
            if dst != src:                             # the centre stays
                moves.append((dst, src))
    return moves


def turn_cycles(face):
    """The d=+1 permutation as five 4-cycles (i0 -> i1 -> i2 -> i3 -> i0),
    meaning new[i1] = old[i0] etc.  d=-1 walks each cycle backwards."""
    nxt = {src: dst for dst, src in turn_perm(face, +1)}
    seen, cyc = set(), []
    for s in sorted(nxt):
        if s in seen or nxt[s] == s:
            continue
        c, x = [], s
        while x not in seen:
            seen.add(x); c.append(x); x = nxt[x]
        assert len(c) == 4, c
        cyc.append(c)
    assert len(cyc) == 5
    return cyc


def boxes(face):
    """[(centre along axis a, half extents)] for slice then block."""
    code = C_FN[face]
    axis, side = code >> 1, (-1 if code & 1 else 1)
    hs = [1.5, 1.5, 1.5]; hs[axis] = 0.5
    hb = [1.5, 1.5, 1.5]; hb[axis] = 1.0
    return axis, side, [(side * 1.0, hs), (-side * 0.5, hb)]


def box_face_base(face, box, bf):
    """Sticker base index 9F + 3 ov + ou of box face bf, or 63 (interior)."""
    axis, side, bx = boxes(face)
    ca, h = bx[box]
    cen = [0.0, 0.0, 0.0]; cen[axis] = ca
    n, u, v = C_FN[bf], C_FU[bf], C_FV[bf]
    sn = -1 if n & 1 else 1
    if abs(sn * (cen[n >> 1] + sn * h[n >> 1]) - 1.5) > 1e-9:
        return 63
    su = -1 if u & 1 else 1
    sv = -1 if v & 1 else 1
    ou = su * cen[u >> 1] - h[u >> 1] + 1.5
    ov = sv * cen[v >> 1] - h[v >> 1] + 1.5
    assert ou == int(ou) and ov == int(ov)
    return 9 * bf + 3 * int(ov) + int(ou)


# ------------------------------------------------------------ table RAM map
# One 256 x 16 block RAM holds the live sticker state and the ROM tables the
# engine reads (the pixel path reads only the state, during active video):
TABLE_STATE = 0      # 0..53   colour of each sticker (0..5), live
TABLE_CYC = 56       # 56 + 20*face + 4*k + m   cycle k member m (sticker index)
TABLE_BASE = 176     # 176 + 12*face + 6*box + bf   sticker base (63 = interior)
TABLE_ADJ = 248      # 248 + face: neighbour faces across -U, +U, -V, +V (3 bits each)

# sticker colours by index (= the face they start on): U white, D yellow,
# R red, L orange, F green, B blue.  10-bit BT.601, chroma centred on 512.
C_STY = [995, 807, 332, 513, 416, 276]
C_STU = [512, 91, 460, 244, 496, 757]
C_STV = [512, 654, 811, 848, 238, 330]


def table_rom():
    t = [0] * 256
    for s in range(54):
        t[TABLE_STATE + s] = s // 9
    for f in range(6):
        for k, c in enumerate(turn_cycles(f)):
            for m, idx in enumerate(c):
                t[TABLE_CYC + 20 * f + 4 * k + m] = idx
        for box in range(2):
            for bf in range(6):
                t[TABLE_BASE + 12 * f + 6 * box + bf] = box_face_base(f, box, bf)
    for f in range(6):
        u, v = C_FU[f], C_FV[f]
        nb = [FACE_OF_NORMAL[c] for c in (u ^ 1, u, v ^ 1, v)]
        t[TABLE_ADJ + f] = nb[0] | nb[1] << 3 | nb[2] << 6 | nb[3] << 9
    return t


def apply_turn(state, face, d):
    new = list(state)
    for dst, src in turn_perm(face, d):
        new[dst] = state[src]
    return new


if __name__ == '__main__':
    # sanity: every turn is a permutation of 20 stickers, 4 turns = identity,
    # + then - = identity, and the cycle form reproduces the permutation.
    solved = [s // 9 for s in range(54)]
    for f in range(6):
        for d in (1, -1):
            assert len(turn_perm(f, d)) == 20
        s = solved
        for _ in range(4):
            s = apply_turn(s, f, 1)
        assert s == solved
        assert apply_turn(apply_turn(solved, f, 1), f, -1) == solved
        st = list(range(54))
        ref = apply_turn(st, f, 1)
        cyc = list(st)
        for c in turn_cycles(f):
            for m in range(4):
                cyc[c[(m + 1) % 4]] = st[c[m]]
        assert cyc == ref, f
    t = table_rom()
    ext = sum(1 for f in range(6) for b in range(2) for bf in range(6)
              if t[TABLE_BASE + 12 * f + 6 * b + bf] != 63)
    print('turn model OK; exterior box faces over all turns:', ext)
