#!/usr/bin/env python3
"""Generate bishop_mesh_pkg.vhd from a flat SVG wireframe (vertices + edges).

This is the "switch to the dense model" path: a single STATIC mesh, no
expressions, no open/closed variants, no EOR boundary classification, no
tip-stripping.  Every edge is a plain detail edge (bnd=0) drawn as a line;
the grid fill is dropped in bishop.vhd.  All 12 mesh accessors collapse to
4 arrays (the dedup taken to its limit).

Edit face12.svg, then run this, then ./build_programs.sh ron bishop.
"""
import re
from pathlib import Path

SVG = Path(__file__).with_name("face12.svg")
OUT = Path(__file__).with_name("bishop_mesh_pkg.vhd")

FP_BITS = 7
FP_SCALE = 1 << FP_BITS          # 128 (Q9.7)
TARGET_H = 760                   # scaled head HEIGHT in program px (drives scale)
# HD runs at half horizontal resolution (hd_clock_divisor=2), so the picture
# is stretched 2x wide on screen AND the rasterizer gets half the cycles per
# line.  Squeeze x by 0.5: this restores proportions AND shrinks the clear
# phase (2*HEAD_HALF_W), freeing per-line budget so busy rows stop dropping
# edges at high thickness.
X_COMPENSATE = 1.0
FP_INT_MAX = 255
MAX_STAMP_M1 = 127
THICK_BUILD = 4
INCLUDE_SILHOUETTE = False       # the clip-path "ring" — dropped (facets form the outline)

# Edges to DELETE, as (pointA, pointB) pairs using the numbers from
# face12_labeled.svg / face12_points.txt.  Order within a pair doesn't matter.
# Numbers are stable (derived from the original SVG), so they keep referring
# to the same points even as edges are removed.
REMOVED_EDGES = [
    (20, 45), (21, 46), (23, 48), (24, 49), (42, 63),
    (110, 131), (112, 133), (131, 127), (131, 126), (132, 128),
    (128, 133), (95, 77), (91, 75),
    # batch 2
    (80, 82), (88, 92), (88, 94), (80, 63), (82, 62), (79, 68), (72, 68),
    # batch 3
    (80, 60),
    # batch 4
    (110, 85), (112, 87), (109, 130), (113, 134), (83, 66),
    # batch 5
    (80, 81), (81, 82),
    # batch 6
    (81, 76),
    # batch 7
    (101, 99), (99, 98), (98, 100), (100, 102), (59, 73),
    # batch 8
    (73, 49), (72, 45),
    # batch 9
    (62, 57), (46, 48), (110, 118), (112, 119), (111, 117),
    (92, 110), (94, 112), (104, 111),
    # batch 10
    (46, 47), (47, 48), (48, 55), (54, 49),
    # batch 11
    (57, 49),
    # batch 12
    (45, 34), (37, 49), (66, 73),
    # batch 13
    (45, 58), (46, 56),
    # batch 14
    (35, 22), (36, 22), (47, 22), (47, 51),
]

# Edges to ADD that aren't in the SVG, as (pointA, pointB) by the same
# numbering.  Drawn as a straight line between the two points' coords.
ADDED_EDGES = [
    (55, 49),
    (35, 36), (35, 42), (43, 36),
]

# Points to MOVE: {number: (new_x, new_y)} in SVG coords.  Applied as an
# overlay AFTER numbering, so point numbers stay stable; every edge touching
# a moved point follows it.
MOVED_POINTS = {
    3: (410.0, 130.0),   # just right of 6
    1: (450.0, 118.0),   # just right of 7
    4: (560.0, 130.0),   # just left of 10
    2: (520.0, 118.0),   # just left of 9
}


def floats(s):
    n = [float(x) for x in re.findall(r'-?\d+\.?\d*', s)]
    return list(zip(n[0::2], n[1::2]))


def main():
    svg = SVG.read_text()
    polylines = re.findall(r'<polyline points="([^"]+)"', svg)
    polygons = re.findall(r'<polygon points="([^"]+)"', svg)

    segs = []
    for pl in polylines:
        p = floats(pl)
        for i in range(len(p) - 1):
            segs.append((p[i], p[i + 1]))

    sil = floats(polygons[0])    # 1st polygon = head clip path (silhouette)
    if INCLUDE_SILHOUETTE:
        for i in range(len(sil)):
            segs.append((sil[i], sil[(i + 1) % len(sil)]))

    # de-duplicate undirected edges (the wireframe retraces shared edges)
    def key(a, b):
        a = (round(a[0], 2), round(a[1], 2))
        b = (round(b[0], 2), round(b[1], 2))
        return (a, b) if a <= b else (b, a)
    uniq = {}
    for a, b in segs:
        k = key(a, b)
        if k[0] != k[1]:
            uniq[k] = (a, b)
    edges = list(uniq.values())

    # Number vertices identically to make_labeled_svg.py (top-to-bottom,
    # then left-to-right) from the FULL edge set, so the numbering is stable,
    # then drop any edge whose endpoints are in REMOVED_EDGES.
    def rnd(p):
        return (round(p[0], 2), round(p[1], 2))
    allv = set()
    for a, b in edges:
        allv.add(rnd(a)); allv.add(rnd(b))
    num = {p: i + 1 for i, p in enumerate(sorted(allv, key=lambda p: (p[1], p[0])))}
    removed = {frozenset(pair) for pair in REMOVED_EDGES}
    if removed:
        before = len(edges)
        edges = [(a, b) for (a, b) in edges
                 if frozenset({num[rnd(a)], num[rnd(b)]}) not in removed]
        print(f"removed {before - len(edges)} of {len(removed)} requested edges")
    # add edges not present in the SVG (straight line between two points)
    inv = {n: p for p, n in num.items()}
    present = {frozenset({num[rnd(a)], num[rnd(b)]}) for a, b in edges}
    added = 0
    for pa, pb in ADDED_EDGES:
        if frozenset({pa, pb}) not in present and pa in inv and pb in inv:
            edges.append((inv[pa], inv[pb])); added += 1
    if ADDED_EDGES:
        print(f"added {added} of {len(ADDED_EDGES)} requested edges")
    # MOVE points: remap every edge endpoint that matches a moved point
    move = {inv[n]: (float(x), float(y)) for n, (x, y) in MOVED_POINTS.items()
            if n in inv}
    if move:
        edges = [(move.get(rnd(a), a), move.get(rnd(b), b)) for a, b in edges]
        print(f"moved {len(move)} points")

    # center + scale using the silhouette bbox (consistent head sizing).
    # y scales to TARGET_H; x scales by the same factor * X_COMPENSATE so the
    # head is half-width in program space (undone by the div2 2x on screen).
    xs = [p[0] for p in sil]
    ys = [p[1] for p in sil]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    scale_y = TARGET_H / (max(ys) - min(ys))
    scale_x = scale_y * X_COMPENSATE

    def scaled(p):
        return ((p[0] - cx) * scale_x, (p[1] - cy) * scale_y)

    # A horizontal edge stamps its full WIDTH on one row, so a wide one can
    # exceed the per-edge stamp cap.  Split such edges into equal pieces
    # (visually identical — a horizontal line is a horizontal line).
    SAFE_PX = MAX_STAMP_M1 - 2 * THICK_BUILD - 2     # leave margin under 127

    def split_edge(a, b):
        (x1, y1), (x2, y2) = a, b
        if round(y1) == round(y2) and abs(x2 - x1) > SAFE_PX:
            k = int(abs(x2 - x1) // SAFE_PX) + 1
            pts = [(x1 + (x2 - x1) * i / k, y1 + (y2 - y1) * i / k)
                   for i in range(k + 1)]
            return [(pts[i], pts[i + 1]) for i in range(k)]
        return [(a, b)]

    def to_dda(a, b):                                # a, b already scaled
        x1, y1 = a
        x2, y2 = b
        if y1 < y2:
            ymin, ymax, xt, xb = y1, y2, x1, x2
        elif y2 < y1:
            ymin, ymax, xt, xb = y2, y1, x2, x1
        else:
            ymin = ymax = y1; xt, xb = x1, x2
        yi0, yi1 = int(round(ymin)), int(round(ymax))
        if yi0 == yi1:
            slope = int(round((xb - xt) * FP_SCALE))          # horizontal: width
        else:
            slope = int(round((xb - xt) * FP_SCALE / (yi1 - yi0)))
        return {"y_min": yi0, "y_max": yi1,
                "x_top": int(round(xt * FP_SCALE)), "slope": slope}

    mesh = []
    for a, b in edges:
        for pa, pb in split_edge(scaled(a), scaled(b)):
            mesh.append(to_dda(pa, pb))
    N = len(mesh)

    # guards (same ceilings as build_face_mesh.check_fpga_limits)
    maxx = 0.0
    for e in mesh:
        xb = e["x_top"] + e["slope"] * (e["y_max"] - e["y_min"])
        maxx = max(maxx, abs(e["x_top"]) / FP_SCALE, abs(xb) / FP_SCALE)
    maxstamp = max(abs(e["slope"]) // FP_SCALE + 2 * THICK_BUILD for e in mesh)
    # HEAD_HALF_W must cover the head's x extent AND be >= ceil((N+pad)/2):
    # the R_CLEAR phase (2*HEAD_HALF_W cycles) is where all N edges are walked
    # and the active list is built, so 2*HEAD_HALF_W must exceed N (+pipeline).
    half_w = max(int(maxx) + 6, (N + 12 + 1) // 2)
    print(f"edges N = {N}   max|x| = {maxx:.1f} (limit {FP_INT_MAX})   "
          f"max stamp = {maxstamp} (limit {MAX_STAMP_M1})   "
          f"suggest HEAD_HALF_W = {half_w}  (>= ceil((N+12)/2) for the edge walk)")
    errs = []
    if maxx > FP_INT_MAX:
        errs.append(f"x overflow {maxx:.1f} > {FP_INT_MAX} (reduce TARGET_H/X_COMPENSATE)")
    if maxstamp > MAX_STAMP_M1:
        errs.append(f"stamp overflow {maxstamp} > {MAX_STAMP_M1}")
    if errs:
        raise SystemExit("LIMIT EXCEEDED: " + "; ".join(errs))

    # ---- emit ----
    def arr(name, vals):
        body = ", ".join(str(v) for v in vals)
        return (f"    constant {name} : t_int_array(0 to C_NUM_EDGES - 1) := (\n"
                f"        {body}\n    );\n")

    L = []
    L.append("-- Auto-generated by build_svg_mesh.py from face12.svg.  Do not edit.")
    L.append("-- Static dense wireframe: ONE mesh, no expressions / variants / EOR.")
    L.append("library ieee;")
    L.append("use ieee.std_logic_1164.all;")
    L.append("use ieee.numeric_std.all;\n")
    L.append("package bishop_mesh_pkg is\n")
    L.append(f"    constant C_NUM_EDGES : natural := {N};")
    L.append("    constant C_NUM_EXPR  : natural := 1;\n")
    L.append("    constant C_GRP_STATIC : natural := 0;")
    L.append("    constant C_GRP_BROW   : natural := 1;")
    L.append("    constant C_GRP_EYE    : natural := 2;")
    L.append("    constant C_GRP_MOUTH  : natural := 3;\n")
    L.append("    constant C_EXPR_NEUTRAL : natural := 0;\n")
    L.append("    type t_int_array is array (natural range <>) of integer;\n")
    L.append(arr("C_EDGE_BND", [0] * N))
    L.append(arr("C_EDGE_GROUP", [0] * N))
    L.append(arr("C_EDGE_Y_MIN", [e["y_min"] for e in mesh]))
    L.append(arr("C_EDGE_Y_MAX", [e["y_max"] for e in mesh]))
    L.append(arr("C_EDGE_X_TOP", [e["x_top"] for e in mesh]))
    L.append(arr("C_EDGE_SLOPE", [e["slope"] for e in mesh]))
    fns = ["f_y_min", "f_y_min_mc", "f_y_min_ec", "f_y_max", "f_y_max_mc", "f_y_max_ec",
           "f_x_top", "f_x_top_mc", "f_x_top_ec", "f_slope", "f_slope_mc", "f_slope_ec"]
    for f in fns:
        L.append(f"    function {f}(expr, idx : natural) return integer;")
    L.append("\nend package bishop_mesh_pkg;\n")
    L.append("package body bishop_mesh_pkg is\n")
    src = {"f_y_min": "C_EDGE_Y_MIN", "f_y_min_mc": "C_EDGE_Y_MIN", "f_y_min_ec": "C_EDGE_Y_MIN",
           "f_y_max": "C_EDGE_Y_MAX", "f_y_max_mc": "C_EDGE_Y_MAX", "f_y_max_ec": "C_EDGE_Y_MAX",
           "f_x_top": "C_EDGE_X_TOP", "f_x_top_mc": "C_EDGE_X_TOP", "f_x_top_ec": "C_EDGE_X_TOP",
           "f_slope": "C_EDGE_SLOPE", "f_slope_mc": "C_EDGE_SLOPE", "f_slope_ec": "C_EDGE_SLOPE"}
    for f in fns:
        L.append(f"    function {f}(expr, idx : natural) return integer is")
        L.append(f"    begin")
        L.append(f"        return {src[f]}(idx);   -- expr/variant ignored (static mesh)")
        L.append(f"    end function;\n")
    L.append("end package body bishop_mesh_pkg;")
    OUT.write_text("\n".join(L) + "\n")
    print(f"wrote {OUT}  (HEAD_HALF_W should be set ~{half_w} in bishop.vhd)")


if __name__ == "__main__":
    main()
