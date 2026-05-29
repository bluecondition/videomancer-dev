#!/usr/bin/env python3
"""Convert face_mesh.py (named-vertex / named-edge wireframe + named
expressions) into the bishop_mesh_pkg.vhd that the FPGA reads.

Workflow:
  1. Edit face_mesh.py (move/add/remove VERTICES, EDGES, or EXPRESSIONS).
  2. Run this script.
  3. Inspect face_labels_<expression>.svg previews.
  4. Build with ./build_programs.sh ron bishop.

Vertex coords are in SVG pixel space (viewBox 960x720); the script
normalizes them around the center of the used vertices and scales to
TARGET_FACE_HEIGHT pixels of FPGA pixel space.

Each edge inherits its animation group from the vertex name prefix:
  brow_*   -> 1
  eye_*    -> 2
  mouth_*  -> 3
  anything else -> 0
If the two endpoints disagree, the edge falls back to group 0.

Each expression in EXPRESSIONS produces an independent set of mesh
arrays.  All sets are padded to the same length (C_NUM_EDGES) with
degenerate no-op edges (y_min > y_max so the rasterizer never activates
them).  The FPGA mux's the active table via a vsync-latched expression
index (K4 top-3-bits).
"""

import subprocess
import sys
from collections import defaultdict
from pathlib import Path

from face_mesh import VERTICES, EDGES, EXPRESSIONS

VHDL_OUT = Path(__file__).with_name("bishop_mesh_pkg.vhd")
TARGET_FACE_HEIGHT = 600

# Q9.7 fixed-point: 9-bit integer (±256) + 7-bit fractional (1/128 px).
# Same 16-bit storage as the old Q12.4 path so current_x_ram fits one EBR.
FP_BITS  = 7
FP_SCALE = 1 << FP_BITS    # 128

# Tip-strip math is sized for THICK_BUILD = 4 (the max K2 thickness)
# so the strip has enough margin at any runtime thickness.  At runtime
# THICK<4, a few extra rows are painted by phantom detail edges instead
# of boundary edges — invisible because both buffers OR into the display.
THICK_BUILD = 4

# Degenerate no-op edge used to pad each expression's mesh up to
# C_NUM_EDGES.  y_min > y_max so the rasterizer never activates it.
NO_OP_EDGE = {
    "y_min": 32767, "y_max": -32768,
    "x_top": 0, "slope": 0,
    "group": 0, "bnd": 0,
}

GROUP_STATIC = 0
GROUP_BROW   = 1
GROUP_EYE    = 2
GROUP_MOUTH  = 3

def group_for(name: str) -> int:
    if name.startswith("brow_"):
        return GROUP_BROW
    if name.startswith("eye_"):
        return GROUP_EYE
    if name.startswith("mouth_"):
        return GROUP_MOUTH
    return GROUP_STATIC

def edge_group(a: str, b: str) -> int:
    ga, gb = group_for(a), group_for(b)
    return ga if ga == gb else GROUP_STATIC

# Edges that should be visible but NOT contribute to even-odd fill
# parity.  These are "interior detail" lines that would otherwise read
# as enclosing a region.
DETAIL_EDGE_NAMES = {
    frozenset({"mouth_l", "mouth_r"}),
    frozenset({"nose_l",  "nose_r"}),
}

def is_boundary_edge(a: str, b: str) -> bool:
    """True if this edge should contribute to EOR parity."""
    if frozenset({a, b}) in DETAIL_EDGE_NAMES:
        return False
    if a.startswith("brow_") and b.startswith("brow_"):
        return False
    return True

def apply_expression(deltas):
    """Return a new VERTICES dict with (dx, dy) deltas applied."""
    out = {}
    for name, (x, y) in VERTICES.items():
        dx, dy = deltas.get(name, (0, 0))
        out[name] = (x + dx, y + dy)
    return out

def normalize(verts):
    """Scale and center VERTICES around the head's bounding box."""
    used = set()
    for a, b in EDGES:
        used.add(a); used.add(b)
    xs = [verts[v][0] for v in used]
    ys = [verts[v][1] for v in used]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    height = max(ys) - min(ys)
    scale = TARGET_FACE_HEIGHT / height
    return {v: ((verts[v][0] - cx) * scale, (verts[v][1] - cy) * scale)
            for v in used}

def to_dda(a, b, group, bnd):
    """Convert an edge to the DDA descriptor the FPGA expects.  x_top
    and slope in Q9.7 (signed * 128)."""
    x1, y1 = a
    x2, y2 = b
    if y1 < y2:
        y_min, y_max = y1, y2
        x_top, x_bot = x1, x2
    elif y2 < y1:
        y_min, y_max = y2, y1
        x_top, x_bot = x2, x1
    else:
        y_min = y_max = y1
        x_top, x_bot = x1, x2
    yi_min = int(round(y_min))
    yi_max = int(round(y_max))
    if yi_min == yi_max:
        slope_fp = int(round((x_bot - x_top) * FP_SCALE))
    else:
        slope_fp = int(round((x_bot - x_top) * FP_SCALE / (yi_max - yi_min)))
    x_top_fp = int(round(x_top * FP_SCALE))
    return {
        "y_min":  yi_min,
        "y_max":  yi_max,
        "x_top":  x_top_fp,
        "slope":  slope_fp,
        "group":  group,
        "bnd":    1 if bnd else 0,
    }

def find_boundary_tips(boundary_edges, norm):
    """Find vertices that are strict top/bottom tips considering only
    boundary edges.  Detail edges don't contribute to EOR, so their
    tips don't need stripping."""
    top_strict = defaultdict(int)
    bot_strict = defaultdict(int)
    horiz = defaultdict(int)
    for a, b in boundary_edges:
        ya, yb = norm[a][1], norm[b][1]
        if ya < yb:
            top_strict[a] += 1; bot_strict[b] += 1
        elif yb < ya:
            top_strict[b] += 1; bot_strict[a] += 1
        else:
            horiz[a] += 1; horiz[b] += 1
    top_tips = set()
    bot_tips = set()
    for v in set(top_strict) | set(bot_strict) | set(horiz):
        if top_strict.get(v, 0) >= 2 and bot_strict.get(v, 0) == 0 \
                and horiz.get(v, 0) == 0:
            top_tips.add(v)
        if bot_strict.get(v, 0) >= 2 and top_strict.get(v, 0) == 0 \
                and horiz.get(v, 0) == 0:
            bot_tips.add(v)
    return top_tips, bot_tips

def build_one_mesh(verts, label, log):
    """Run the full per-expression pipeline on `verts` and return the
    edges_dda list (boundary + detail + phantom)."""
    norm = normalize(verts)
    boundary_edges = [(a, b) for a, b in EDGES if is_boundary_edge(a, b)]
    top_tips, bot_tips = find_boundary_tips(boundary_edges, norm)

    # Pass 1: initial DDA for every edge.
    edges_dda     = []
    edge_top_name = []
    edge_bot_name = []
    for a_name, b_name in EDGES:
        a = norm[a_name]; b = norm[b_name]
        bnd = is_boundary_edge(a_name, b_name)
        dda = to_dda(a, b, edge_group(a_name, b_name), bnd)
        if a[1] < b[1]:
            top_name, bot_name = a_name, b_name
        elif b[1] < a[1]:
            top_name, bot_name = b_name, a_name
        else:
            top_name = bot_name = None
        edges_dda.append(dda)
        edge_top_name.append(top_name)
        edge_bot_name.append(bot_name)

    # Pass 2: tip strip.  N = ceil((|s_int_L| + |s_int_R| + 2*THICK_BUILD + 1)
    # * FP_SCALE / |slope_diff|).  THICK_BUILD=4 keeps the strip valid for
    # any runtime K2 thickness.
    def find_N(tip, which):
        slopes = []
        for i, dda in enumerate(edges_dda):
            if dda["bnd"] != 1:
                continue
            if which == "top" and edge_top_name[i] == tip:
                slopes.append(dda["slope"])
            elif which == "bot" and edge_bot_name[i] == tip:
                slopes.append(dda["slope"])
        if len(slopes) < 2:
            return 1
        diff = abs(slopes[0] - slopes[1])
        if diff == 0:
            return 1
        s_int_a = abs(slopes[0] // FP_SCALE)
        s_int_b = abs(slopes[1] // FP_SCALE)
        needed = (s_int_a + s_int_b + 2 * THICK_BUILD + 1) * FP_SCALE
        return max(1, (needed + diff - 1) // diff)

    top_tip_N = {}
    bot_tip_N = {}
    for tip in sorted(top_tips):
        N = find_N(tip, "top")
        top_tip_N[tip] = N
        for i, dda in enumerate(edges_dda):
            if dda["bnd"] == 1 and edge_top_name[i] == tip \
                    and dda["y_min"] + N <= dda["y_max"]:
                dda["y_min"] += N
                dda["x_top"] += N * dda["slope"]
        log.append(f"  [{label}] top tip {tip}: stripped {N} row(s)")

    for tip in sorted(bot_tips):
        N = find_N(tip, "bot")
        bot_tip_N[tip] = N
        for i, dda in enumerate(edges_dda):
            if dda["bnd"] == 1 and edge_bot_name[i] == tip \
                    and dda["y_min"] + N <= dda["y_max"]:
                dda["y_max"] -= N
        log.append(f"  [{label}] bot tip {tip}: stripped {N} row(s)")

    # Pass 3: phantom detail edges.
    phantom_edges = []
    n_boundary = len(edges_dda)
    for i in range(n_boundary):
        dda = edges_dda[i]
        if dda["bnd"] != 1:
            continue
        tip = edge_top_name[i]
        if tip in top_tip_N:
            N = top_tip_N[tip]
            phantom_edges.append({
                "y_min": dda["y_min"] - N,
                "y_max": dda["y_min"] - 1,
                "x_top": dda["x_top"] - N * dda["slope"],
                "slope": dda["slope"],
                "group": dda["group"],
                "bnd":   0,
            })
        tip = edge_bot_name[i]
        if tip in bot_tip_N:
            N = bot_tip_N[tip]
            phantom_y_min = dda["y_max"] + 1
            phantom_y_max = dda["y_max"] + N
            phantom_x_top = dda["x_top"] + (phantom_y_min - dda["y_min"]) * dda["slope"]
            phantom_edges.append({
                "y_min": phantom_y_min,
                "y_max": phantom_y_max,
                "x_top": phantom_x_top,
                "slope": dda["slope"],
                "group": dda["group"],
                "bnd":   0,
            })

    edges_dda.extend(phantom_edges)
    log.append(f"  [{label}] edges: {len(edges_dda)} "
               f"(orig {n_boundary} + phantoms {len(phantom_edges)})")
    return edges_dda

def emit_vhdl(meshes_per_expr, expr_names, out_path: Path):
    """Emit bishop_mesh_pkg.vhd.

    The varying fields (y_min, y_max, x_top, slope) are emitted as ONE
    1D constant array per expression (e.g. C_EDGE_Y_MIN_0 .. _4) and
    selected at runtime by case-statement lookup functions.  This keeps
    yosys synthesizing them as distributed LUT ROMs (fast) instead of
    BRAM (a single 2D / flattened array got BRAM-inferred and dropped
    Fmax from ~78 to ~58 MHz).

    group and bnd are expression-independent (boundary classification
    and animation grouping don't change with pose, and the phantom-edge
    order is identical across expressions), so they're emitted once as
    plain 1D arrays.

    All expressions are padded to the same length with NO_OP_EDGE."""
    max_edges = max(len(m) for m in meshes_per_expr)
    n_expr    = len(meshes_per_expr)
    padded    = [m + [NO_OP_EDGE] * (max_edges - len(m)) for m in meshes_per_expr]

    lines = []
    lines.append("-- Auto-generated by build_face_mesh.py.  Edit face_mesh.py, not this file.")
    lines.append(f"-- Expressions ({n_expr}): {', '.join(expr_names)}")
    lines.append(f"-- Edges per expression (padded to max): {max_edges}")
    lines.append("")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("use ieee.numeric_std.all;")
    lines.append("")
    lines.append("package bishop_mesh_pkg is")
    lines.append("")
    lines.append(f"    constant C_NUM_EDGES : natural := {max_edges};")
    lines.append(f"    constant C_NUM_EXPR  : natural := {n_expr};")
    lines.append("")
    lines.append("    constant C_GRP_STATIC : natural := 0;")
    lines.append("    constant C_GRP_BROW   : natural := 1;")
    lines.append("    constant C_GRP_EYE    : natural := 2;")
    lines.append("    constant C_GRP_MOUTH  : natural := 3;")
    lines.append("")
    for i, name in enumerate(expr_names):
        lines.append(f"    constant C_EXPR_{name.upper()} : natural := {i};")
    lines.append("")
    lines.append("    type t_int_array is array (natural range <>) of integer;")
    lines.append("")

    def emit_1d(name, values, comment):
        lines.append(f"    -- {comment}")
        lines.append(f"    constant {name} : t_int_array(0 to C_NUM_EDGES - 1) := (")
        chunks = []
        for i in range(0, len(values), 8):
            row = ", ".join(f"{v:>6}" for v in values[i:i + 8])
            chunks.append("        " + row)
        lines.append(",\n".join(chunks))
        lines.append("    );")
        lines.append("")

    # Varying fields: one 1D array per expression.
    VARY = [("y_min", "C_EDGE_Y_MIN", "y_min"),
            ("y_max", "C_EDGE_Y_MAX", "y_max"),
            ("x_top", "C_EDGE_X_TOP", "x at y_min, Q9.7 (128x px)"),
            ("slope", "C_EDGE_SLOPE", "slope dx/dy, Q9.7 (128x px/row)")]
    for field, base, desc in VARY:
        for ei in range(n_expr):
            emit_1d(f"{base}_{ei}", [e[field] for e in padded[ei]],
                    f"{desc} — {expr_names[ei]}")

    # Expression-independent fields (use expression 0).
    emit_1d("C_EDGE_GROUP", [e["group"] for e in padded[0]],
            "animation group per edge (expression-independent)")
    emit_1d("C_EDGE_BND", [e["bnd"] for e in padded[0]],
            "1=boundary (EOR), 0=detail/no-op (expression-independent)")

    # Lookup functions — case statement keeps the per-expression arrays
    # as separate LUT ROMs and adds a small 5:1 output mux.
    for _, base, _ in VARY:
        fn = "f_" + base[len("C_EDGE_"):].lower()   # C_EDGE_Y_MIN -> f_y_min
        lines.append(f"    function {fn}(expr, idx : natural) return integer;")
    lines.append("")
    lines.append("end package bishop_mesh_pkg;")
    lines.append("")
    lines.append("package body bishop_mesh_pkg is")
    lines.append("")
    for _, base, _ in VARY:
        fn = "f_" + base[len("C_EDGE_"):].lower()
        lines.append(f"    function {fn}(expr, idx : natural) return integer is")
        lines.append("    begin")
        lines.append("        case expr is")
        for ei in range(n_expr):
            sel = "when others" if ei == n_expr - 1 else f"when {ei}"
            lines.append(f"            {sel} => return {base}_{ei}(idx);")
        lines.append("        end case;")
        lines.append("    end function;")
        lines.append("")
    lines.append("end package body bishop_mesh_pkg;")
    out_path.write_text("\n".join(lines) + "\n")

def main():
    expr_names = list(EXPRESSIONS.keys())
    print(f"expressions ({len(expr_names)}): {', '.join(expr_names)}")
    print(f"THICK_BUILD = {THICK_BUILD}  (strip math sized for max K2 thickness)")

    meshes = []
    log    = []
    for name in expr_names:
        deltas = EXPRESSIONS[name]
        verts  = apply_expression(deltas)
        mesh   = build_one_mesh(verts, name, log)
        meshes.append(mesh)
    for line in log:
        print(line)

    sizes = [len(m) for m in meshes]
    print(f"per-expression edge counts: {sizes} -> padding to max={max(sizes)}")

    emit_vhdl(meshes, expr_names, VHDL_OUT)
    print(f"wrote {VHDL_OUT}")

    # Regenerate preview SVGs so they always match the emitted mesh.
    svg_script = Path(__file__).with_name("make_labels_svg.py")
    if svg_script.exists():
        subprocess.run([sys.executable, str(svg_script)], check=True)

if __name__ == "__main__":
    main()
