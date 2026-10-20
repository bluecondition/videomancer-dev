#!/usr/bin/env python3
"""NAT20 engine data: the dice as triangle lists, packed for the table RAM.

Every die is drawn as triangles (one face slot each in the pixel path):
d4/d8/d20 faces are triangles already; each d6 square and each d10 kite is
split along a diagonal into two halves that share that diagonal as a FAKE
edge (never a silhouette, never bevelled).  Squares split corner A->C, kites
across the two side vertices, so a kite is an apex half (T, Sa, Sb) and a
foot half (Sa, F, Sb).

Each die is pre-rotated into its hero pose (the model's present() of its top
number, the face/vertex a player wants to see), so the knobs at their centre
positions show that pose and turn the die away from it.

Table RAM, 512 x 16 (word addresses):
  0..39     header, 8 words per die: vbase nv tbase nt ebase ne nbase 0
  40..75    12 body colours x 3: Y' (video), U, V (10-bit, BT.601, unswapped)
  76..      per die: vertices (x, y, z Q14), tris, edges, face normals (Q14)
  tri word:  v0 | v1<<4 | v2<<8 | up<<12   (edge k runs v_k -> v_(k+1) mod 3,
             CCW seen from outside; up: the numeral's top -- the corner k
             of a d8/d20 triangle, the edge k (f[k] -> f[k+1]) of a d6
             square; 0 elsewhere)

Faces are stored in NUMBER order (index = number - 1; d10: index = number)
and d4 vertices in number order (vertex i shows i + 1), so the microcode
gets each numeral from an index.
  edge word: tA | kA<<5 | tB<<7 | kB<<12 | fake<<14
             (tri tA has the edge as its edge kA, tri tB as its edge kB, in
             the opposite direction; fake = the diagonal inside a split face;
             the fake edges come first)
  normals are per FACE: tri t belongs to face t (d4/d8/d20) or t>>1 (d6/d10)
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import nat20_model as M  # noqa: E402

DICE = ["d4", "d6", "d8", "d10", "d20"]
HERO_FACE = {"d4": 4, "d6": 6, "d8": 8, "d10": 0, "d20": 20}    # number shown
Q14 = 16384
HDR, COLB, DATA = 0, 40, 76

# body colours (sRGB) from the look model -> 10-bit Y'UV (BT.601)
COLOR_ORDER = ["crimson", "midnight", "obsidian", "bone", "emerald", "amethyst",
               "frost", "ember", "lagoon", "bubblegum", "smoke", "sunflower"]


def yuv10(rgb):
    r, g, b = (c / 255.0 for c in rgb)
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = (b - y) / 1.772
    v = (r - y) / 1.402
    return (int(round(64 + 876 * y)), int(round(512 + 896 * u)), int(round(512 + 896 * v)))


def hero_matrix(die):
    D = M.Die(die)
    num = HERO_FACE[die]
    if die == "d4":
        # present() shows a d4 by its face's FIRST vertex: pick a face whose
        # first vertex is the one numbered 4
        vnum = M.DICE["d4"]["vertex_numbers"]
        fi = next(i for i, F in enumerate(D.faces) if vnum[F["verts"][0]] == num)
    else:
        fi = next(i for i, F in enumerate(D.faces) if F["number"] == num)
    return M.present(die, fi)


def numbered(die):
    """dice.json's faces in number order, and (d4) the vertices renumbered
    so vertex i shows i + 1.  Returns (vertex array, faces)."""
    raw = M.DICE[die]
    V = np.array(raw["verts"], float)
    faces = [dict(F) for F in raw["faces"]]
    if die == "d4":
        vn = raw["vertex_numbers"]                  # old vertex -> number
        order = sorted(range(len(vn)), key=lambda i: vn[i])
        remap = {old: new for new, old in enumerate(order)}
        V = V[order]
        for F in faces:
            F["verts"] = [remap[v] for v in F["verts"]]
        return V, faces
    base = 0 if die == "d10" else 1
    faces.sort(key=lambda F: F["number"])
    assert [F["number"] for F in faces] == list(range(base, base + len(faces)))
    return V, faces


def up_code(die, V, F):
    """the numeral's top: the corner (triangles) or edge (squares) index"""
    vs = F["verts"]
    if die in ("d8", "d20"):
        return vs.index(F["up_vertex"])
    if die == "d6":
        cen = V[vs].mean(0)
        u = np.array(F["up_dir"], float)
        best = max(range(4), key=lambda k: ((V[vs[k]] + V[vs[(k + 1) % 4]]) / 2 - cen) @ u)
        return best
    if die == "d10":
        assert vs[0] == F["up_vertex"], "kites list their apex first"
    return 0


def build_die(die):
    raw = M.DICE[die]
    V, faces = numbered(die)
    R = hero_matrix(die)
    V = V @ R.T                                   # pre-rotated into the hero pose
    tris, tface, fakes, normals, ups = [], [], [], [], []
    for fi, F in enumerate(faces):
        vs = F["verts"]
        n = np.cross(V[vs[1]] - V[vs[0]], V[vs[2]] - V[vs[0]])
        n /= np.linalg.norm(n)
        assert n @ V[vs].mean(0) > 0, "faces must be CCW from outside"
        normals.append(n)
        up = up_code(die, V, F)
        if len(vs) == 3:
            tris.append(tuple(vs)); tface.append(fi); fakes.append(0); ups.append(up)
        elif raw["shape"] == "square":
            a, b, c, d = vs
            tris.append((a, b, c)); tface.append(fi); fakes.append(3); ups.append(up)  # c->a fake
            tris.append((a, c, d)); tface.append(fi); fakes.append(1); ups.append(up)  # a->c fake
        else:                                                           # kite T Sa F Sb
            t, sa, f, sb = vs
            tris.append((t, sa, sb)); tface.append(fi); fakes.append(2); ups.append(up)  # sa->sb fake
            tris.append((sa, f, sb)); tface.append(fi); fakes.append(3); ups.append(up)  # sb->sa fake
    # undirected edges: the two tris that share each one
    owner = {}
    for ti, (v0, v1, v2) in enumerate(tris):
        for k, (a, b) in enumerate(((v0, v1), (v1, v2), (v2, v0))):
            owner[(a, b)] = (ti, k)
    edges = []
    for (a, b), (ta, ka) in sorted(owner.items()):
        if a < b:
            tb, kb = owner[(b, a)]
            edges.append((ta, ka, tb, kb, int(tface[ta] == tface[tb])))
    # fake diagonals first: the engine's pairing pass then pairs a split
    # face's two halves before anything else
    edges.sort(key=lambda e: -e[4])
    for ti in range(len(tris)):
        if len(faces[0]["verts"]) > 3:
            assert tface[ti] == ti >> 1
        else:
            assert tface[ti] == ti
    return dict(V=V, tris=tris, fakes=fakes, edges=edges, normals=np.array(normals),
                tface=tface, R=R, ups=ups, faces=faces)


def q14(x):
    v = int(round(x * Q14))
    assert -32768 <= v < 32768
    return v & 0xFFFF


def build():
    words = [0] * 512
    meta = {}
    p = DATA
    for d, die in enumerate(DICE):
        g = build_die(die)
        nv, nt, nf, ne = len(g["V"]), len(g["tris"]), len(g["normals"]), len(g["edges"])
        vbase = p
        for v in g["V"]:
            for c in v:
                words[p] = q14(c); p += 1
        tbase = p
        for (v0, v1, v2), up in zip(g["tris"], g["ups"]):
            words[p] = v0 | v1 << 4 | v2 << 8 | up << 12; p += 1
        ebase = p
        for ta, ka, tb, kb, fk in g["edges"]:
            words[p] = ta | ka << 5 | tb << 7 | kb << 12 | fk << 14; p += 1
        nbase = p
        for n in g["normals"]:
            for c in n:
                words[p] = q14(c); p += 1
        split = int(nt != nf)
        words[HDR + 8 * d: HDR + 8 * d + 8] = [vbase, nv, tbase, nt, ebase, ne, nbase, split]
        meta[die] = dict(vbase=vbase, nv=nv, tbase=tbase, nt=nt, ebase=ebase, ne=ne,
                         nbase=nbase, nf=nf)
    assert p <= 512, p
    for i, name in enumerate(COLOR_ORDER):
        y, u, v = yuv10(M.COLORS[name][0])
        words[COLB + 3 * i: COLB + 3 * i + 3] = [y, u, v]
    meta["used"] = p
    return words, meta


if __name__ == "__main__":
    w, meta = build()
    print(json.dumps(meta, indent=1))
