#!/usr/bin/env python3
"""NAT20 die definitions: geometry, numbering and numeral placement, checked.

Builds the five dice (d4, d6, d8, d10, d20) as data, verifies every rule in
the brief face by face, and writes dice.json (consumed by the preview model
and, later, by the table/microcode generators) plus DICE_CHECK.md.

Conventions
  * Die space is right-handed; every die is scaled to circumradius 1.
  * Each face lists its vertex indices counter-clockwise seen from OUTSIDE,
    so the outward normal is (v1-v0) x (v2-v0).
  * Each face has an "up" vector: the direction (in the face plane) from the
    numeral's base to its top.  Triangles point up at a vertex, squares at an
    edge midpoint, kites at their pole apex.
  * d4 faces carry three corner numerals: each corner shows the number of
    that VERTEX, top pointing at the corner.

Run:  python3 gen_dice.py      (writes dice.json and DICE_CHECK.md)
"""
import itertools
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PHI = (1 + 5 ** 0.5) / 2


# ---------------------------------------------------------------- helpers
def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def hull_faces(verts, tol=1e-9):
    """Faces of a convex polytope as CCW-from-outside vertex index lists."""
    n = len(verts)
    faces = set()
    for i, j, k in itertools.combinations(range(n), 3):
        a, b, c = verts[i], verts[j], verts[k]
        nrm = np.cross(b - a, c - a)
        if np.linalg.norm(nrm) < 1e-9:
            continue
        nrm = unit(nrm)
        d = nrm @ a
        s = verts @ nrm - d
        if np.all(s <= tol):
            pass
        elif np.all(s >= -tol):
            nrm, d, s = -nrm, -d, -s
        else:
            continue
        on = tuple(sorted(np.nonzero(np.abs(s) <= tol)[0]))
        faces.add(on)
    out = []
    for f in faces:
        pts = verts[list(f)]
        cen = pts.mean(0)
        nrm = unit(np.cross(pts[1] - pts[0], pts[2] - pts[0]))
        if nrm @ cen < 0:
            nrm = -nrm
        ax = unit(pts[0] - cen)
        ay = np.cross(nrm, ax)
        ang = [math.atan2((p - cen) @ ay, (p - cen) @ ax) for p in pts]
        order = [f[i] for i in np.argsort(ang)]
        out.append(order)
    return out


def face_normal(verts, f):
    a, b, c = verts[f[0]], verts[f[1]], verts[f[2]]
    return unit(np.cross(b - a, c - a))


def edges_of(faces):
    e = {}
    for fi, f in enumerate(faces):
        for k in range(len(f)):
            key = tuple(sorted((f[k], f[(k + 1) % len(f)])))
            e.setdefault(key, []).append(fi)
    return e


def opposite_face(verts, faces):
    nrm = [face_normal(verts, f) for f in faces]
    opp = []
    for i, n in enumerate(nrm):
        j = int(np.argmin([n @ m for m in nrm]))
        opp.append(j if abs(nrm[j] @ n + 1) < 1e-6 else -1)
    return opp


# ---------------------------------------------------------------- solids
def tetrahedron():
    v = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]], float)
    return v / np.linalg.norm(v[0])


def cube():
    v = np.array(list(itertools.product((-1, 1), repeat=3)), float)
    return v / np.linalg.norm(v[0])


def octahedron():
    return np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                     [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)


def icosahedron():
    v = []
    for s1 in (-1, 1):
        for s2 in (-1, 1):
            v += [[0, s1, s2 * PHI], [s1, s2 * PHI, 0], [s2 * PHI, 0, s1]]
    v = np.array(v, float)
    return v / np.linalg.norm(v[0])


def trapezohedron10():
    """Pentagonal trapezohedron = polar dual of the UNIFORM pentagonal
    antiprism (all edges equal).  Returns (verts, info)."""
    rp = 1 / (2 * math.sin(math.pi / 5))           # pentagon circumradius, s = 1
    # lateral edge length 1: 2 rp^2 (1 - cos 36) + H^2 = 1
    H = math.sqrt(1 - 2 * rp * rp * (1 - math.cos(math.pi / 5)))
    anti = []
    for k in range(5):
        a = 2 * math.pi * k / 5
        anti.append([rp * math.cos(a), rp * math.sin(a), H / 2])
    for k in range(5):
        a = 2 * math.pi * k / 5 + math.pi / 5
        anti.append([rp * math.cos(a), rp * math.sin(a), -H / 2])
    anti = np.array(anti)
    # polar reciprocation in the unit sphere: face plane n.x = d -> vertex n/d
    dual = []
    for f in hull_faces(anti):
        n = face_normal(anti, f)
        d = n @ anti[f[0]]
        dual.append(n / d)
    dual = np.array(dual)
    # check: the antiprism really is uniform
    elen = sorted({round(float(np.linalg.norm(anti[a] - anti[b])), 9)
                   for a, b in edges_of(hull_faces(anti))})
    dual /= np.max(np.linalg.norm(dual, axis=1))
    return dual, {"antiprism_edge_lengths": elen, "antiprism_H": H, "antiprism_rp": rp}


# ---------------------------------------------------------------- numbering
def number_d4(verts, faces):
    """Vertex-read d4: vertex k carries number k+1.  (Handedness: seen from
    outside at vertex 4, 1 -> 2 -> 3 run the way check() reports.)"""
    t = np.dot(np.cross(verts[0] - verts[3], verts[1] - verts[3]), verts[2] - verts[3])
    return [1, 2, 3, 4] if t < 0 else [1, 3, 2, 4]


def number_d6(verts, faces):
    """Opposite faces sum to 7; 1, 2, 3 counter-clockwise round their shared
    corner seen from outside (Western, right-handed)."""
    nrm = [face_normal(verts, f) for f in faces]
    ax = {}
    for i, n in enumerate(nrm):
        k = int(np.argmax(np.abs(n)))
        ax[(k, int(np.sign(n[k])))] = i
    num = [0] * 6
    # 1 = +z, 2 = +x, 3 = +y : (x cross y).z ... check below and swap if needed
    a1, a2, a3 = ax[(2, 1)], ax[(0, 1)], ax[(1, 1)]
    n1, n2, n3 = nrm[a1], nrm[a2], nrm[a3]
    if np.dot(np.cross(n1, n2), n3) < 0:
        a2, a3 = a3, a2
    num[a1], num[a2], num[a3] = 1, 2, 3
    for (k, s), i in ax.items():
        if num[i] == 0:
            num[i] = 7 - num[ax[(k, -s)]]
    return num


def number_d8(verts, faces, opp):
    """Opposite faces sum to 9.  Equal vertex sums (18 each) are impossible on
    an octahedron (each vertex would need faces {2,3,5,8} or {1,4,6,7}, and
    the x, y and z vertex sets cannot all be unions of one of those), so take
    the most even spread: vertex sums 14 16 18 18 20 22."""
    vfaces = [[fi for fi, f in enumerate(faces) if v in f] for v in range(len(verts))]
    reps = [(i, j) for i, j in enumerate(opp) if i < j]
    best, best_key = None, None
    for perm in itertools.permutations(range(1, 5)):
        for flips in itertools.product((0, 1), repeat=4):
            num = [0] * 8
            for (i, j), p, fl in zip(reps, perm, flips):
                lo, hi = p, 9 - p
                num[i], num[j] = (hi, lo) if fl else (lo, hi)
            vs = [sum(num[f] for f in vf) for vf in vfaces]
            key = (max(abs(x - 18) for x in vs), sum((x - 18) ** 2 for x in vs))
            if best_key is None or key < best_key:
                best, best_key = num, key
    return best


def number_d10(verts, faces, apex_top):
    """0-9 (0 reads as 10); opposite faces sum to 9; odd numbers ring the top
    apex, even numbers the bottom.  Top ring, counter-clockwise seen from
    above: 1 7 3 9 5 (alternating low/high)."""
    top = [fi for fi, f in enumerate(faces) if apex_top in f]
    cen = [verts[f].mean(0) for f in faces]
    top.sort(key=lambda fi: math.atan2(cen[fi][1], cen[fi][0]))
    num = [None] * 10
    for fi, n in zip(top, [1, 7, 3, 9, 5]):
        num[fi] = n
    nrm = [face_normal(verts, f) for f in faces]
    for fi in top:
        j = int(np.argmin([nrm[fi] @ m for m in nrm]))
        num[j] = 9 - num[fi]
    return num


def number_d20(verts, faces, opp, seed=1):
    """1-20, opposite faces sum to 21, balanced: minimise the spread of the
    twelve vertex sums (ideal 52.5) and of each face's neighbour sums (ideal
    31.5).  Deterministic local search from a fixed seed."""
    rng = np.random.default_rng(seed)
    vfaces = [[fi for fi, f in enumerate(faces) if v in f] for v in range(len(verts))]
    E = edges_of(faces)
    nbr = [[] for _ in faces]
    for (a, b), fs in E.items():
        nbr[fs[0]].append(fs[1])
        nbr[fs[1]].append(fs[0])
    reps = sorted({tuple(sorted((i, j))) for i, j in enumerate(opp)})

    def build(vals, flips):
        num = [0] * 20
        for (i, j), v, fl in zip(reps, vals, flips):
            num[i], num[j] = (21 - v, v) if fl else (v, 21 - v)
        return num

    def cost(num):
        cv = sum((sum(num[f] for f in vf) - 52.5) ** 2 for vf in vfaces)
        cn = sum((sum(num[g] for g in nbr[f]) - 31.5) ** 2 for f in range(20))
        return cv + 0.25 * cn

    best_num, best_c = None, 1e18
    for restart in range(60):
        vals = list(rng.permutation(np.arange(1, 11)))
        flips = list(rng.integers(0, 2, 10))
        cur = build(vals, flips)
        c = cost(cur)
        T = 40.0
        for it in range(6000):
            v2, f2 = vals[:], flips[:]
            if rng.random() < 0.5:
                a, b = rng.choice(10, 2, replace=False)
                v2[a], v2[b] = v2[b], v2[a]
            else:
                a = rng.integers(10)
                f2[a] ^= 1
            n2 = build(v2, f2)
            c2 = cost(n2)
            if c2 <= c or rng.random() < math.exp((c - c2) / T):
                vals, flips, cur, c = v2, f2, n2, c2
            T *= 0.999
        if c < best_c:
            best_c, best_num = c, cur
    # canonical: put 20 on the face whose normal is closest to +z
    return best_num, best_c


# ---------------------------------------------------------------- up vectors
def tri_up_vertex(verts, f, axis):
    """Numeral top points at the face's vertex highest along `axis`."""
    h = [verts[v] @ axis for v in f]
    return f[int(np.argmax(h))]


def build():
    dice = {}

    # ---- d4
    v = tetrahedron()
    F = hull_faces(v)
    vnum = number_d4(v, F)
    d4faces = []
    for f in F:
        d4faces.append({"verts": [int(i) for i in f], "corner_numbers": [vnum[i] for i in f]})
    dice["d4"] = {"verts": v, "faces": d4faces, "vertex_numbers": vnum, "shape": "tri"}

    # ---- d6
    v = cube()
    F = hull_faces(v)
    num = number_d6(v, F)
    faces = []
    for f, n in zip(F, num):
        nrm = face_normal(v, f)
        # numeral up: +z on the four side faces, +y on the top/bottom faces
        up = np.array([0, 0, 1.0]) if abs(nrm[2]) < 0.5 else np.array([0, 1.0, 0])
        faces.append({"verts": f, "number": n, "up_dir": up})
    dice["d6"] = {"verts": v, "faces": faces, "shape": "square"}

    # ---- d8
    v = octahedron()
    F = hull_faces(v)
    opp = opposite_face(v, F)
    num = number_d8(v, F, opp)
    faces = []
    for f, n in zip(F, num):
        nrm = face_normal(v, f)
        pole = np.array([0, 0, 1.0 if nrm[2] > 0 else -1.0])
        faces.append({"verts": f, "number": n, "up_vertex": tri_up_vertex(v, f, pole)})
    dice["d8"] = {"verts": v, "faces": faces, "shape": "tri"}

    # ---- d10
    v, info = trapezohedron10()
    F = hull_faces(v)
    apex_top = int(np.argmax(v[:, 2]))
    apex_bot = int(np.argmin(v[:, 2]))
    num = number_d10(v, F, apex_top)
    faces = []
    for f, n in zip(F, num):
        apex = apex_top if apex_top in f else apex_bot
        # rotate the vertex list so the kite's pole apex comes first
        k = f.index(apex)
        f = f[k:] + f[:k]
        faces.append({"verts": f, "number": n, "up_vertex": apex})
    dice["d10"] = {"verts": v, "faces": faces, "shape": "kite", "info": info,
                   "apex_top": apex_top, "apex_bot": apex_bot}

    # ---- d20
    v = icosahedron()
    F = hull_faces(v)
    opp = opposite_face(v, F)
    num, c = number_d20(v, F, opp)
    # numeral tops point at the vertex highest along a fixed, slightly tilted
    # axis through face 20 (tilt breaks the ties between shared vertices)
    f20 = num.index(20)
    ax = unit(face_normal(v, F[f20]) + 0.13 * unit(v[F[f20][0]] - v[F[f20][1]]))
    faces = []
    for f, n in zip(F, num):
        faces.append({"verts": f, "number": n, "up_vertex": tri_up_vertex(v, f, ax)})
    dice["d20"] = {"verts": v, "faces": faces, "shape": "tri", "balance_cost": c}
    return dice


# ---------------------------------------------------------------- checks
def check(dice):
    rep = []
    ok_all = True

    def ok(cond, msg):
        nonlocal ok_all
        ok_all &= bool(cond)
        rep.append(("PASS " if cond else "FAIL ") + msg)

    for name, d in dice.items():
        v = d["verts"]
        F = [f["verts"] for f in d["faces"]]
        E = edges_of(F)
        rep.append(f"\n## {name}: {len(v)} vertices, {len(E)} edges, {len(F)} faces")
        ok(len(v) - len(E) + len(F) == 2, f"{name} Euler V-E+F = 2")
        ok(all(len(fs) == 2 for fs in E.values()), f"{name} every edge joins exactly two faces")
        ok(np.allclose(np.linalg.norm(v, axis=1).max(), 1), f"{name} circumradius 1")
        # planarity + outward CCW + face-transitive distances
        dists = []
        for f in F:
            n = face_normal(v, f)
            ds = [n @ v[i] for i in f]
            dists.append(ds[0])
            ok(np.ptp(ds) < 1e-9, f"{name} face {f} planar") if np.ptp(ds) >= 1e-9 else None
            ok(n @ v[f].mean(0) > 0, f"{name} face {f} wound CCW from outside") if n @ v[f].mean(0) <= 0 else None
        ok(np.ptp(dists) < 1e-9, f"{name} all faces at one distance from centre (inradius {dists[0]:.6f}) -> fair")
        ok(all(np.ptp([np.linalg.norm(v[f[k]] - v[f[(k + 1) % len(f)]]) for k in range(len(f))]) < 1e-9
               for f in F) or name == "d10", f"{name} faces equilateral" if name != "d10" else "d10 (kites)")
        opp = opposite_face(v, F)
        if name != "d4":
            ok(all(o >= 0 for o in opp), f"{name} centrally symmetric (every face has an opposite)")

    # ---- d4
    d = dice["d4"]
    v = d["verts"]
    vn = d["vertex_numbers"]
    rep.append("\nd4 faces (corner numbers, CCW from outside):")
    for f in d["faces"]:
        rep.append(f"  verts {f['verts']} -> corners {f['corner_numbers']}")
    for vi in range(4):
        shown = {f["corner_numbers"][f["verts"].index(vi)] for f in d["faces"] if vi in f["verts"]}
        ok(shown == {vn[vi]}, f"d4 vertex {vi}: every face meeting it shows {vn[vi]} there ({len([1 for f in d['faces'] if vi in f['verts']])} faces)")
    ok(sorted(vn) == [1, 2, 3, 4], "d4 vertex numbers are 1-4")
    p = [v[vn.index(k)] for k in (1, 2, 3, 4)]
    t = np.dot(np.cross(p[0] - p[3], p[1] - p[3]), p[2] - p[3])
    ok(t < 0, "d4 seen from outside at vertex 4, 1->2->3 run counter-clockwise (same hand as the d6)")
    for f in d["faces"]:
        missing = ({1, 2, 3, 4} - set(f["corner_numbers"])).pop()
        rep.append(f"  face showing {f['corner_numbers']}: rests on it -> top vertex reads {missing}")

    # ---- d6
    d = dice["d6"]
    v = d["verts"]
    F = [f["verts"] for f in d["faces"]]
    num = [f["number"] for f in d["faces"]]
    opp = opposite_face(v, F)
    ok(sorted(num) == list(range(1, 7)), "d6 numbers 1-6")
    ok(all(num[i] + num[opp[i]] == 7 for i in range(6)), "d6 opposite faces sum to 7")
    i1, i2, i3 = num.index(1), num.index(2), num.index(3)
    shared = set(F[i1]) & set(F[i2]) & set(F[i3])
    ok(len(shared) == 1, "d6 faces 1, 2, 3 share one corner")
    n1, n2, n3 = (face_normal(v, F[i]) for i in (i1, i2, i3))
    ok(np.dot(np.cross(n1, n2), n3) > 0,
       "d6 1->2->3 counter-clockwise seen from outside their shared corner (Western / right-handed)")

    # ---- d8
    d = dice["d8"]
    v = d["verts"]
    F = [f["verts"] for f in d["faces"]]
    num = [f["number"] for f in d["faces"]]
    opp = opposite_face(v, F)
    ok(sorted(num) == list(range(1, 9)), "d8 numbers 1-8")
    ok(all(num[i] + num[opp[i]] == 9 for i in range(8)), "d8 opposite faces sum to 9")
    vs = [sum(num[fi] for fi, f in enumerate(F) if vi in f) for vi in range(6)]
    ok(max(abs(s - 18) for s in vs) <= 4,
       f"d8 most even vertex sums possible (all-18 is impossible on an octahedron): {sorted(vs)}")

    # ---- d10
    d = dice["d10"]
    v = d["verts"]
    F = [f["verts"] for f in d["faces"]]
    num = [f["number"] for f in d["faces"]]
    opp = opposite_face(v, F)
    info = d["info"]
    ok(len(info["antiprism_edge_lengths"]) == 1,
       f"d10 source antiprism is uniform (edge lengths {info['antiprism_edge_lengths']})")
    ok(len(v) == 12 and all(len(f) == 4 for f in F), "d10 has 12 vertices and 10 kite faces (not a bipyramid: 4-sided faces)")
    ok(sorted(num) == list(range(10)), "d10 numbers 0-9")
    ok(all(num[i] + num[opp[i]] == 9 for i in range(10)), "d10 opposite faces sum to 9")
    at, ab = d["apex_top"], d["apex_bot"]
    ok(all((n % 2 == 1) == (at in f) for n, f in zip(num, F)),
       "d10 odd numbers ring the top apex, even numbers the bottom apex")
    # kite proportions
    f = F[0]
    T, A, L, B = (v[i] for i in f)
    ls = [np.linalg.norm(A - T), np.linalg.norm(L - A), np.linalg.norm(B - L), np.linalg.norm(T - B)]
    ang_T = math.degrees(math.acos(unit(A - T) @ unit(B - T)))
    ang_L = math.degrees(math.acos(unit(A - L) @ unit(B - L)))
    ang_s = math.degrees(math.acos(unit(T - A) @ unit(L - A)))
    rep.append(f"  kite edges (apex->side, side->foot): {ls[0]:.5f}, {ls[1]:.5f}  ratio {ls[0] / ls[1]:.5f}")
    rep.append(f"  kite angles: apex {ang_T:.3f} deg, foot {ang_L:.3f} deg, sides {ang_s:.3f} deg")
    ok(abs(ang_T - ang_L) > 1, "d10 kite is a true kite (apex and foot angles differ)")
    # long axis T->L, side vertices: fraction along axis
    ax = L - T
    t_side = ((A - T) @ ax) / (ax @ ax)
    rep.append(f"  side vertices sit {t_side:.5f} of the way from apex to foot")
    rep.append(f"  height/equator: apex z {v[at][2]:.5f}, ring z +-{abs(v[f[1]][2]):.5f}")
    top = sorted([fi for fi in range(10) if at in F[fi]],
                 key=lambda fi: math.atan2(v[F[fi]].mean(0)[1], v[F[fi]].mean(0)[0]))
    rep.append("  top ring (CCW from above): " + " ".join(str(num[fi]) for fi in top))

    # ---- d20
    d = dice["d20"]
    v = d["verts"]
    F = [f["verts"] for f in d["faces"]]
    num = [f["number"] for f in d["faces"]]
    opp = opposite_face(v, F)
    ok(sorted(num) == list(range(1, 21)), "d20 numbers 1-20")
    ok(all(num[i] + num[opp[i]] == 21 for i in range(20)), "d20 opposite faces sum to 21")
    vs = [int(sum(num[fi] for fi, f in enumerate(F) if vi in f)) for vi in range(12)]
    rep.append(f"  vertex sums (ideal 52.5): {sorted(vs)}")
    ok(max(vs) - min(vs) <= 6, f"d20 balanced: vertex sums within {max(vs) - min(vs)} of each other")
    E = edges_of(F)
    nbr = [[] for _ in F]
    for (a, b), fs in E.items():
        nbr[fs[0]].append(fs[1])
        nbr[fs[1]].append(fs[0])
    hi = [n > 10 for n in num]
    allsame = [i for i in range(20) if all(hi[j] == hi[i] for j in nbr[i])]
    ok(len(allsame) == 0, f"d20 interleaved: no face is surrounded only by its own half (high/low) ({len(allsame)} such faces)")
    ns = [sum(num[j] for j in nbr[i]) for i in range(20)]
    rep.append(f"  neighbour sums (ideal 31.5): min {min(ns)} max {max(ns)}")
    rep.append("  20's neighbours: " + str(sorted(int(num[j]) for j in nbr[num.index(20)])) +
               "   1's neighbours: " + str(sorted(int(num[j]) for j in nbr[num.index(1)])))
    return ok_all, rep


def to_json(dice):
    out = {}
    for name, d in dice.items():
        o = {k: (x.tolist() if isinstance(x, np.ndarray) else x) for k, x in d.items()
             if k not in ("faces",)}
        o["faces"] = []
        for f in d["faces"]:
            o["faces"].append({k: (x.tolist() if isinstance(x, np.ndarray) else x) for k, x in f.items()})
        out[name] = o
    return out


def main():
    dice = build()
    ok_all, rep = check(dice)
    with open(os.path.join(HERE, "dice.json"), "w") as fh:
        json.dump(to_json(dice), fh, indent=1, default=lambda o: o.item() if hasattr(o, "item") else o.tolist())
    lines = ["# NAT20 die check (generated by gen_dice.py)", ""]
    lines += [r for r in rep]
    lines.append("")
    lines.append("RESULT: " + ("ALL PASS" if ok_all else "FAILURES"))
    with open(os.path.join(HERE, "DICE_CHECK.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(rep))
    print("\nRESULT:", "ALL PASS" if ok_all else "FAILURES")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
