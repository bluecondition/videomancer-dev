#!/usr/bin/env python3
"""Generate bishop_mesh_pkg.vhd from a flat SVG wireframe (vertices + edges).

This is the "switch to the dense model" path: a single STATIC mesh, no
expressions, no open/closed variants, no EOR boundary classification, no
tip-stripping.  Every edge is a plain detail edge (bnd=0) drawn as a line;
the grid fill is dropped in bishop.vhd.  All 12 mesh accessors collapse to
4 arrays (the dedup taken to its limit).

Edit face12.svg, then run this, then ./build_programs.sh ron bishop.
"""
import math
import re
from pathlib import Path

SVG = Path(__file__).with_name("face12.svg")
OUT = Path(__file__).with_name("bishop_mesh_pkg.vhd")

FP_BITS = 7
FP_SCALE = 1 << FP_BITS          # 128 (Q9.7)
TARGET_H = 950                   # scaled head HEIGHT in program px (drives scale)
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
    # batch 12  (66-73 un-removed in batch 16)
    (45, 34), (37, 49),
    # batch 13  (45-58, 46-56 un-removed in batch 16)
    # batch 14
    (35, 22), (36, 22), (47, 22), (47, 51),
    # batch 15 - drop the 118/119 upper-lip kinks (lines sticking in toward
    # center); run the upper lip straight from the mouth corner to the bow.
    (118, 120), (118, 122), (119, 121), (119, 123),
    # batch 17 - re-route 43 from 64 to 57
    (43, 64),
    # batch 19 - trim 4 edges to fit under the 256-edge ceiling (symmetric).
    (67, 79), (67, 80), (70, 82), (70, 83),
]

# Edges to ADD that aren't in the SVG, as (pointA, pointB) by the same
# numbering.  Drawn as a straight line between the two points' coords.
ADDED_EDGES = [
    (55, 49),
    (35, 36), (35, 42), (43, 36),
    (122, 120), (123, 121),   # batch 15 - upper lip corner->bow, straight
    # batch 16 - new lines (45-58, 46-56, 66-73 un-removed above)
    (22, 51), (65, 72), (49, 59), (48, 57), (130, 137), (134, 140),
    # batch 17 - re-route 43 from 64 to 57
    (43, 57),
    # batch 18
    (46, 42), (43, 48),
]

# Points to MOVE: {number: (new_x, new_y)} in SVG coords.  Applied as an
# overlay AFTER numbering, so point numbers stay stable; every edge touching
# a moved point follows it.
MOVED_POINTS = {
    3: (410.0, 130.0),   # just right of 6
    1: (450.0, 118.0),   # just right of 7
    4: (560.0, 130.0),   # just left of 10
    2: (520.0, 118.0),   # just left of 9
    # batch 16 - narrow the nose bridge for more contour: left rail +10x,
    # right rail -10x (both toward center).
    42: (465.0, 283.0),  60: (462.0, 320.0),  75: (458.0, 360.0),  85: (454.0, 400.0),
    43: (495.0, 283.0),  62: (498.0, 320.0),  77: (502.0, 360.0),  87: (506.0, 400.0),
}

# ---- Morph overlays: per-point (dx, dy) SVG-px deltas (+y = down) ----
# The neutral mesh = fully-open mouth & eyes.  Each morph moves a set of points;
# every edge endpoint that is (or interpolates toward) a moved point follows it.
# Expressions adapted from the tuned named-vertex deltas in face_mesh.py, mapped
# onto face12 point numbers.  Mouth corners 122/123, upper lip 115/116/117,
# lower lip 126/127/128; eyes: upper lid 52/53/54/55, lower lid 67/68/69/70,
# outer 58/65/59/66, inner 56/63/57/64; brows: inner 30/34/31/37, peak 38/41,
# outer 32/44/33/50; nose sides 91/92/94/95, tip 88/104.
# Mouth close (K1=0%): lower lip rises and upper-lip lower edge dips so the lips
# meet around the mouth centerline (~y510); corners pinch in slightly; chin lifts
# a touch for the jaw-up realism cue.  Neutral (K1=100%) = these all zero = open.
MOUTH_CLOSED = {
    124:(0,-9), 127:(0,-12), 126:(0,-12), 128:(0,-12), 125:(0,-9),  # lower lip up
    117:(0,4), 120:(0,3), 121:(0,3),                                # upper lip dips
    122:(3,0), 123:(-3,0),                                          # corners pinch
    131:(0,-2), 132:(0,-3), 133:(0,-2),                            # chin lifts
}
# Eye blink (P8=Closed): upper lids drop and lower lids rise to a near-flat line
# at mid-eye (~y317); outer/inner corners ease in.  Neutral (P8=Open) = zero.
EYES_CLOSED = {
    52:(0,15), 53:(0,15), 54:(0,15), 55:(0,15),     # upper lids down
    67:(0,-14), 68:(0,-14), 69:(0,-14), 70:(0,-14),  # lower lids up
    60:(0,-3), 63:(0,-4), 62:(0,-3), 64:(0,-4),      # mid-lower lids up
    65:(0,-6), 66:(0,-6),                            # lower outer up
    56:(0,4), 57:(0,4),                              # inner-upper ease down
}

EXPR_HAPPY = {
    122:(-8,-10), 123:(8,-10), 117:(0,-6), 115:(0,-6), 116:(0,-6),
    126:(0,3), 127:(0,2), 128:(0,2),
    67:(0,-5), 68:(0,-5), 69:(0,-5), 70:(0,-5),
    52:(0,2), 53:(0,2), 54:(0,2), 55:(0,2),
    58:(0,-2), 65:(0,-2), 59:(0,-2), 66:(0,-2),
    91:(0,-2), 92:(0,-2), 94:(0,-2), 95:(0,-2),
}
EXPR_SAD = {
    30:(3,-10), 34:(3,-10), 31:(-3,-10), 37:(-3,-10), 38:(0,-2), 41:(0,-2),
    32:(0,5), 44:(0,5), 33:(0,5), 50:(0,5),
    52:(0,4), 53:(0,4), 54:(0,4), 55:(0,4),
    67:(0,2), 68:(0,2), 69:(0,2), 70:(0,2), 58:(0,2), 65:(0,2), 59:(0,2), 66:(0,2),
    122:(3,10), 123:(-3,10), 117:(0,-2), 115:(0,-2), 116:(0,-2),
    126:(0,-4), 127:(0,-3), 128:(0,-3),
}
EXPR_ANGRY = {
    30:(3,10), 34:(3,10), 31:(-3,10), 37:(-3,10), 38:(0,3), 41:(0,3),
    32:(0,-8), 44:(0,-8), 33:(0,-8), 50:(0,-8),
    52:(0,2), 53:(0,2), 54:(0,2), 55:(0,2), 67:(0,-2), 68:(0,-2), 69:(0,-2), 70:(0,-2),
    56:(2,0), 63:(2,0), 58:(2,0), 65:(2,0), 57:(-2,0), 64:(-2,0), 59:(-2,0), 66:(-2,0),
    47:(0,5), 51:(0,5), 91:(0,-3), 92:(0,-3), 94:(0,-3), 95:(0,-3), 88:(0,-3), 104:(0,-3),
    122:(0,2), 123:(0,2), 117:(0,2), 115:(0,2), 116:(0,2),
    126:(0,-4), 127:(0,-3), 128:(0,-3),
}
EXPR_SURPRISED = {
    32:(0,-11), 44:(0,-11), 38:(0,-14), 30:(0,-11), 34:(0,-11),
    33:(0,-11), 50:(0,-11), 41:(0,-14), 31:(0,-11), 37:(0,-11),
    52:(0,-5), 53:(0,-5), 54:(0,-5), 55:(0,-5),
    67:(0,4), 68:(0,4), 69:(0,4), 70:(0,4),
    117:(0,-5), 115:(0,-5), 116:(0,-5),
    126:(0,12), 127:(0,10), 128:(0,10), 122:(6,2), 123:(-6,2),
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
    inv = {n: p for p, n in num.items()}
    # Edge list as stable (point_number, point_number) pairs — keeps the point
    # identity so morph deltas (keyed by number) can follow each endpoint.
    edge_nums = []
    seen = set()
    n_removed = 0
    for a, b in edges:
        key = frozenset({num[rnd(a)], num[rnd(b)]})
        if key in removed:
            n_removed += 1; continue
        if key in seen:
            continue
        seen.add(key); edge_nums.append((num[rnd(a)], num[rnd(b)]))
    print(f"removed {n_removed} edge-instances")
    added = 0
    for pa, pb in ADDED_EDGES:
        key = frozenset({pa, pb})
        if pa in inv and pb in inv and key not in seen:
            seen.add(key); edge_nums.append((pa, pb)); added += 1
    print(f"added {added} of {len(ADDED_EDGES)} requested edges")

    # center + scale using the silhouette bbox (consistent head sizing).
    xs = [p[0] for p in sil]
    ys = [p[1] for p in sil]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    scale_y = TARGET_H / (max(ys) - min(ys))
    scale_x = scale_y * X_COMPENSATE

    def scaled(p):
        return ((p[0] - cx) * scale_x, (p[1] - cy) * scale_y)

    def neutral_pos(n):                              # SVG pos (MOVED overlay)
        return MOVED_POINTS.get(n, inv[n])

    # A horizontal edge stamps its full WIDTH on one row, so a wide one can
    # exceed the per-edge stamp cap.  Split such edges; morph deltas at a split
    # point interpolate between the two original endpoints' deltas.
    SAFE_PX = MAX_STAMP_M1 - 2 * THICK_BUILD - 2     # leave margin under 127

    def split_fracs(sa, sb):
        (x1, y1), (x2, y2) = sa, sb
        if round(y1) == round(y2) and abs(x2 - x1) > SAFE_PX:
            k = int(abs(x2 - x1) // SAFE_PX) + 1
            return [(i / k, (i + 1) / k) for i in range(k)]
        return [(0.0, 1.0)]

    def lerp(p, q, f):
        return (p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f)

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

    # Build the mesh + per-morph per-endpoint px deltas, aligned 1:1 with mesh[]
    # (same split + same by-y top/bottom swap, so a delta stays on its endpoint).
    MORPHS = ("mouth", "eye", "happy", "sad", "angry", "surprised")
    morph_src = {"mouth": MOUTH_CLOSED, "eye": EYES_CLOSED, "happy": EXPR_HAPPY,
                 "sad": EXPR_SAD, "angry": EXPR_ANGRY, "surprised": EXPR_SURPRISED}

    def clamp8(v):
        return max(-127, min(127, int(round(v))))

    mesh = []
    deltas = {m: [] for m in MORPHS}
    for na, nb in edge_nums:
        sa, sb = scaled(neutral_pos(na)), scaled(neutral_pos(nb))
        for f0, f1 in split_fracs(sa, sb):
            p0, p1 = lerp(sa, sb, f0), lerp(sa, sb, f1)
            mesh.append(to_dda(p0, p1))
            top_is_p0 = (p0[1] <= p1[1])
            for m in MORPHS:
                da = morph_src[m].get(na, (0, 0))
                db = morph_src[m].get(nb, (0, 0))
                e0 = lerp(da, db, f0)
                e1 = lerp(da, db, f1)
                d0 = (e0[0] * scale_x, e0[1] * scale_y)
                d1 = (e1[0] * scale_x, e1[1] * scale_y)
                dt, dbt = (d0, d1) if top_is_p0 else (d1, d0)
                deltas[m].append((clamp8(dt[0]), clamp8(dt[1]),
                                  clamp8(dbt[0]), clamp8(dbt[1])))
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
    if N > 256:
        # bishop.vhd addresses edges with 8-bit indices and 256-deep BRAMs/
        # ROMs (act_*_ram, act_list_ram, the x-extent ROMs).  >256 edges index
        # out of bounds -> "index out of bounds" at yosys ghdl import.
        errs.append(f"edge count {N} > 256 (8-bit edge addressing ceiling)")
    if errs:
        raise SystemExit("LIMIT EXCEEDED: " + "; ".join(errs))

    # Per-edge x-extent [x_lo, x_hi] in head-relative px.  The stamp loop is
    # clamped to this range so the K2 thickness pad can't push a near-horizontal
    # edge LENGTHWISE past its endpoints (the "spike" artifact).  Vertical-
    # dominant edges (|slope_int| == 0) get their thickness from the horizontal
    # pad and never overshoot, so we mark them inert with a wide sentinel.
    # +-1 px margin keeps rounding from clipping the genuine endpoint pixel.
    X_INERT = 511
    X_MARGIN = 1

    def x_extent(e):
        xt, sl = e["x_top"], e["slope"]
        if e["y_max"] == e["y_min"]:        # horizontal: slope holds the WIDTH
            xo = xt + sl
        else:
            xo = xt + sl * (e["y_max"] - e["y_min"])
        if abs(sl) // FP_SCALE == 0:        # vertical-dominant -> don't clamp
            return (-X_INERT, X_INERT)
        lo = min(xt, xo) / FP_SCALE
        hi = max(xt, xo) / FP_SCALE
        return (int(math.floor(lo)) - X_MARGIN, int(math.ceil(hi)) + X_MARGIN)

    xext = [x_extent(e) for e in mesh]

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
    L.append("    constant C_NUM_EXPR  : natural := 5;\n")
    L.append("    constant C_GRP_STATIC : natural := 0;")
    L.append("    constant C_GRP_BROW   : natural := 1;")
    L.append("    constant C_GRP_EYE    : natural := 2;")
    L.append("    constant C_GRP_MOUTH  : natural := 3;\n")
    L.append("    constant C_EXPR_NEUTRAL   : natural := 0;")
    L.append("    constant C_EXPR_HAPPY     : natural := 1;")
    L.append("    constant C_EXPR_SAD       : natural := 2;")
    L.append("    constant C_EXPR_ANGRY     : natural := 3;")
    L.append("    constant C_EXPR_SURPRISED : natural := 4;\n")
    L.append("    type t_int_array is array (natural range <>) of integer;\n")
    L.append(arr("C_EDGE_BND", [0] * N))
    L.append(arr("C_EDGE_GROUP", [0] * N))
    L.append(arr("C_EDGE_Y_MIN", [e["y_min"] for e in mesh]))
    L.append(arr("C_EDGE_Y_MAX", [e["y_max"] for e in mesh]))
    L.append(arr("C_EDGE_X_TOP", [e["x_top"] for e in mesh]))
    L.append(arr("C_EDGE_SLOPE", [e["slope"] for e in mesh]))
    L.append(arr("C_EDGE_X_LO", [v[0] for v in xext]))
    L.append(arr("C_EDGE_X_HI", [v[1] for v in xext]))
    # x_bot = x at y_max (the second endpoint), Q9.7 like x_top.  Needed by the
    # noise engine to jitter both endpoints.  For horizontal edges (y_max ==
    # y_min) the slope field holds the WIDTH, so the far end is x_top + slope;
    # using the general x_top + slope*(y_max-y_min) would collapse to x_top and
    # make horizontal lines vanish once noise jitters them.
    def x_bot_of(e):
        if e["y_max"] == e["y_min"]:
            return e["x_top"] + e["slope"]
        return e["x_top"] + e["slope"] * (e["y_max"] - e["y_min"])
    L.append(arr("C_EDGE_X_BOT", [x_bot_of(e) for e in mesh]))

    # ---- morph deltas (mouth open/close, eye blink, expressions) ----
    # Per edge, two packed words (top endpoint, bottom endpoint).  Each word is
    # (dy<<8) | (dx & 0xFF), 8-bit signed px deltas applied additively to the
    # endpoint in the vblank engine.  PHASE 0: zero-filled (RTL plumbing only);
    # later phases fill these from the morph overlays defined above.
    def pack_delta(dx, dy):
        return ((int(dy) & 0xFF) << 8) | (int(dx) & 0xFF)

    def emit_morph(prefix, dl):                 # dl = deltas[m] (dxt,dyt,dxb,dyb)
        L.append(arr(prefix + "_DTOP", [pack_delta(d[0], d[1]) for d in dl]))
        L.append(arr(prefix + "_DBOT", [pack_delta(d[2], d[3]) for d in dl]))

    emit_morph("C_MOUTH", deltas["mouth"])
    emit_morph("C_EYE", deltas["eye"])
    emit_morph("C_EXPR_HAPPY", deltas["happy"])
    emit_morph("C_EXPR_SAD", deltas["sad"])
    emit_morph("C_EXPR_ANGRY", deltas["angry"])
    emit_morph("C_EXPR_SURPRISED", deltas["surprised"])
    nz = sum(1 for m in MORPHS for d in deltas[m] if any(d))
    print(f"morph deltas: {nz} non-zero edge-endpoints across {len(MORPHS)} morphs")

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
