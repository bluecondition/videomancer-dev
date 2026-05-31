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

from face_mesh import VERTICES, EDGES, EXPRESSIONS, close_mouth, close_eyes

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
#
# CRITICAL: these y values are stored on the FPGA as signed(12 downto 0)
# (13-bit, range -4096..+4095).  The old sentinels 32767 / -32768
# OVERFLOWED that width — to_signed(32767,13) wraps to -1 and
# to_signed(-32768,13) wraps to 0 — turning a "never active" no-op into
# an edge ACTIVE for rows y=-1..0 that stamps at x_top=0 (screen
# centre).  That produced a 2px dot at the nose centre whenever a
# variant padded eye-phantom slots with no-ops (i.e. eyes closed).
# Use in-range values that v_y_target (≈ ±frame_height/2, well under
# 4095) can never equal, so the edge stays inactive every line.
NO_OP_EDGE = {
    "y_min": 4095, "y_max": -4096,
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

def slope_int_of(slope_fp):
    """Signed integer part of a Q9.7 slope, i.e. dx per row in whole pixels."""
    return slope_fp // FP_SCALE if slope_fp >= 0 else -((-slope_fp) // FP_SCALE)

def thickness_extra_y(slope_fp):
    """Disabled for now (returns 0).  Thickness phantoms tripled the
    mesh size and starved HD timing — even at cap=1 the per-(expr,
    variant, field) LUT-ROM tables added too much LC pressure.

    Re-enable once the BRAM-prefetch architecture frees up the LCs
    currently held by the LUT-ROM mesh tables."""
    return 0

def build_one_mesh(verts, label, log, canonical_template=None,
                   thick_phantoms_per_slot=None):
    """Run the full per-expression pipeline on `verts`.

    Returns (edges_dda, template).

    When `canonical_template` is None, phantoms are emitted naturally
    in boundary-iteration order, and the resulting (parent_idx, side)
    template is returned so other variants can be slot-aligned to it.

    When given, phantoms are emitted in the template's slot order; any
    slot whose parent isn't a stripped tip in THIS variant is filled
    with NO_OP_EDGE.  This keeps every variant the same edge count and
    every feature at the same indices, so the FPGA can pick open vs
    closed per edge without remapping.

    thick_phantoms_per_slot, when given, is a per-slot list of `extra_y`
    counts.  For each slot p with extra_y = N, the function appends 2*N
    parallel detail edges (offsets -N..-1 and +1..+N) AFTER the tip-strip
    phantoms.  Slots where THIS variant's slope wouldn't naturally need
    that many extensions emit NO_OP padding, so every variant has the
    same total edge count and the same per-slot semantics."""
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

    # Pass 3: phantom detail edges (template-aware).
    def top_phantom(dda, N):
        return {"y_min": dda["y_min"] - N,
                "y_max": dda["y_min"] - 1,
                "x_top": dda["x_top"] - N * dda["slope"],
                "slope": dda["slope"], "group": dda["group"], "bnd": 0}

    def bot_phantom(dda, N):
        p_y_min = dda["y_max"] + 1
        return {"y_min": p_y_min,
                "y_max": dda["y_max"] + N,
                "x_top": dda["x_top"] + (p_y_min - dda["y_min"]) * dda["slope"],
                "slope": dda["slope"], "group": dda["group"], "bnd": 0}

    phantom_edges = []
    n_boundary = len(edges_dda)
    if canonical_template is None:
        template_out = []
        for i in range(n_boundary):
            dda = edges_dda[i]
            if dda["bnd"] != 1:
                continue
            tip = edge_top_name[i]
            if tip in top_tip_N:
                phantom_edges.append(top_phantom(dda, top_tip_N[tip]))
                template_out.append((i, "top"))
            tip = edge_bot_name[i]
            if tip in bot_tip_N:
                phantom_edges.append(bot_phantom(dda, bot_tip_N[tip]))
                template_out.append((i, "bot"))
    else:
        template_out = canonical_template
        for parent_idx, side in canonical_template:
            dda = edges_dda[parent_idx]
            phantom = None
            if dda["bnd"] == 1:
                if side == "top":
                    tip = edge_top_name[parent_idx]
                    if tip in top_tip_N:
                        phantom = top_phantom(dda, top_tip_N[tip])
                else:
                    tip = edge_bot_name[parent_idx]
                    if tip in bot_tip_N:
                        phantom = bot_phantom(dda, bot_tip_N[tip])
            phantom_edges.append(phantom if phantom is not None
                                 else dict(NO_OP_EDGE))

    edges_dda.extend(phantom_edges)
    real = sum(1 for p in phantom_edges if p["y_min"] <= p["y_max"])
    log.append(f"  [{label}] edges: {len(edges_dda)} "
               f"(orig {n_boundary} + phantoms {real} real / "
               f"{len(phantom_edges) - real} no-op)")

    # Pass 4: thickness phantoms.  For each slot (parent edge), emit
    # 2*extra_y parallel detail rows offset perpendicular-ish (Y only,
    # which is the correct perpendicular direction for near-horizontal
    # edges — the only ones that get extensions).  Slot count is fixed
    # across variants by thick_phantoms_per_slot so per-edge indexing
    # stays uniform; variants whose own slope wouldn't naturally need
    # that many extensions get NO_OP fillers.
    if thick_phantoms_per_slot is not None:
        thick_real = 0
        thick_nop  = 0
        for parent_idx, max_extra in enumerate(thick_phantoms_per_slot):
            if max_extra <= 0:
                continue
            parent = edges_dda[parent_idx]
            # NO_OP parents (used to pad eye-phantom slots in EC variants
            # etc.) shouldn't emit thickness phantoms — they don't render.
            parent_active = (parent["y_min"] <= parent["y_max"])
            this_extra = thickness_extra_y(parent["slope"]) if parent_active else 0
            for i in range(1, max_extra + 1):
                for sign in (-1, +1):
                    if i <= this_extra:
                        edges_dda.append({
                            "y_min": parent["y_min"] + sign * i,
                            "y_max": parent["y_max"] + sign * i,
                            "x_top": parent["x_top"],
                            "slope": parent["slope"],
                            "group": parent["group"],
                            "bnd":   0,
                        })
                        thick_real += 1
                    else:
                        edges_dda.append(dict(NO_OP_EDGE))
                        thick_nop += 1
        log.append(f"  [{label}]  + thickness: {thick_real} real / "
                   f"{thick_nop} no-op  (total {len(edges_dda)})")
    return edges_dda, template_out

# Variant suffix -> (function suffix, comment tag).  "" = open mouth +
# open eyes (selected when S7=open and S8=open or when the edge isn't a
# mouth/eye edge).  "MC" = mouth-closed override (selected for mouth
# edges when S7=closed).  "EC" = eye-closed override (selected for eye
# edges when S8=closed).
VARIANTS = [
    ("",   "",     "open mouth+eyes (base, also used for non-mouth-non-eye edges)"),
    ("MC", "_mc",  "mouth closed (used for mouth edges when S7=closed)"),
    ("EC", "_ec",  "eyes closed (used for eye edges when S8=closed)"),
]

def emit_vhdl(variants_per_expr, expr_names, out_path: Path):
    """Emit bishop_mesh_pkg.vhd.

    variants_per_expr is a dict keyed by (expr_idx, variant_suffix in
    {"", "MC", "EC"}) returning that variant's mesh edge list.  All
    meshes are padded to a common length so every (expression, variant,
    edge) tuple addresses the same slot.

    The varying fields (y_min, y_max, x_top, slope) are emitted as one
    1D LUT-ROM per (expression, variant) and selected at runtime by
    case-statement lookup functions (f_y_min / f_y_min_mc / f_y_min_ec
    etc.).  This keeps yosys synthesizing them as distributed LUT ROMs
    instead of inferring BRAM (which previously dropped Fmax from ~78
    to ~58 MHz).

    group and bnd come from the "" variant — they describe the slot's
    feature identity (which is what the FPGA needs to pick which
    variant to read) and don't differ between variants in any way that
    matters for the override decision."""
    all_meshes = list(variants_per_expr.values())
    max_edges  = max(len(m) for m in all_meshes)
    n_expr     = len(expr_names)

    def pad(mesh):
        return mesh + [NO_OP_EDGE] * (max_edges - len(mesh))
    padded = {key: pad(m) for key, m in variants_per_expr.items()}

    lines = []
    lines.append("-- Auto-generated by build_face_mesh.py.  Edit face_mesh.py, not this file.")
    lines.append(f"-- Expressions ({n_expr}): {', '.join(expr_names)}")
    lines.append(f"-- Edges per (expression, variant): {max_edges}")
    lines.append("-- Variants per expression: open (base), MC (mouth-closed), EC (eyes-closed).")
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

    VARY = [("y_min", "C_EDGE_Y_MIN", "y_min"),
            ("y_max", "C_EDGE_Y_MAX", "y_max"),
            ("x_top", "C_EDGE_X_TOP", "x at y_min, Q9.7 (128x px)"),
            ("slope", "C_EDGE_SLOPE", "slope dx/dy, Q9.7 (128x px/row)")]

    for field, base, desc in VARY:
        for vsuffix, _, vdesc in VARIANTS:
            for ei in range(n_expr):
                cname = f"{base}{('_' + vsuffix) if vsuffix else ''}_{ei}"
                emit_1d(cname, [e[field] for e in padded[(ei, vsuffix)]],
                        f"{desc} — {expr_names[ei]} / {vdesc}")

    # Group and bnd come from the open variant (slot identity).
    emit_1d("C_EDGE_GROUP", [e["group"] for e in padded[(0, "")]],
            "animation group per edge slot — used by FPGA to pick variant")
    emit_1d("C_EDGE_BND", [e["bnd"] for e in padded[(0, "")]],
            "1=boundary (EOR), 0=detail/no-op (slot identity)")

    # Declare per-variant lookup functions.
    for _, base, _ in VARY:
        for _, fnsuffix, _ in VARIANTS:
            fn = "f_" + base[len("C_EDGE_"):].lower() + fnsuffix
            lines.append(f"    function {fn}(expr, idx : natural) return integer;")
    # And the per-edge "auto-select" wrappers that pick the right
    # variant for this edge based on C_EDGE_GROUP and the open-mouth /
    # open-eyes toggles.  Calling _sel hides the 3-way mux from the
    # render so Stage 0 and latch_held both stay one-liners.
    for _, base, _ in VARY:
        fn = "f_" + base[len("C_EDGE_"):].lower() + "_sel"
        lines.append(f"    function {fn}(expr, idx : natural; "
                     f"om, oe : std_logic) return integer;")
    lines.append("")
    lines.append("end package bishop_mesh_pkg;")
    lines.append("")
    lines.append("package body bishop_mesh_pkg is")
    lines.append("")
    for _, base, _ in VARY:
        for vsuffix, fnsuffix, _ in VARIANTS:
            fn = "f_" + base[len("C_EDGE_"):].lower() + fnsuffix
            lines.append(f"    function {fn}(expr, idx : natural) return integer is")
            lines.append("    begin")
            lines.append("        case expr is")
            for ei in range(n_expr):
                cname = f"{base}{('_' + vsuffix) if vsuffix else ''}_{ei}"
                sel = "when others" if ei == n_expr - 1 else f"when {ei}"
                lines.append(f"            {sel} => return {cname}(idx);")
            lines.append("        end case;")
            lines.append("    end function;")
            lines.append("")
    # _sel bodies — pick variant per edge based on group + toggles.
    for _, base, _ in VARY:
        stem = base[len("C_EDGE_"):].lower()
        fn = "f_" + stem + "_sel"
        lines.append(f"    function {fn}(expr, idx : natural; "
                     f"om, oe : std_logic) return integer is")
        lines.append("    begin")
        lines.append("        case C_EDGE_GROUP(idx) is")
        lines.append("            when C_GRP_MOUTH =>")
        lines.append(f"                if om = '0' then return f_{stem}_mc(expr, idx);")
        lines.append(f"                else            return f_{stem}(expr, idx); end if;")
        lines.append("            when C_GRP_EYE =>")
        lines.append(f"                if oe = '0' then return f_{stem}_ec(expr, idx);")
        lines.append(f"                else            return f_{stem}(expr, idx); end if;")
        lines.append("            when others =>")
        lines.append(f"                return f_{stem}(expr, idx);")
        lines.append("        end case;")
        lines.append("    end function;")
        lines.append("")
    lines.append("end package body bishop_mesh_pkg;")
    out_path.write_text("\n".join(lines) + "\n")

def main():
    expr_names = list(EXPRESSIONS.keys())
    print(f"expressions ({len(expr_names)}): {', '.join(expr_names)}")
    print(f"THICK_BUILD = {THICK_BUILD}  (strip math sized for max K2 thickness)")

    log = []
    # Canonical phantom slot template comes from neutral / open — every
    # other variant aligns to its (parent_idx, side) layout so slot
    # indices match across (expression, variant) and the FPGA can pick
    # variants per edge without remapping.
    canon_verts = apply_expression(EXPRESSIONS[expr_names[0]])
    _, canon_template = build_one_mesh(canon_verts, "canonical", log)

    transforms = [("",   lambda v: v),
                  ("MC", close_mouth),
                  ("EC", close_eyes)]

    # Pass 1: build each variant once without thickness phantoms so we
    # can survey per-slot slopes and decide how many parallel detail
    # rows each slot needs across the whole variant set.  Eye edges
    # need ~4 extras in the EC variant (horizontal) but only ~1 in the
    # open variant, so we take the per-slot max so slot indexing stays
    # uniform when the FPGA picks variants per edge.
    pass1_meshes = {}
    for ei, name in enumerate(expr_names):
        base = apply_expression(EXPRESSIONS[name])
        for vsuffix, transform in transforms:
            mesh, _ = build_one_mesh(transform(base),
                                     f"{name}/{vsuffix or 'open'}/pass1",
                                     log, canon_template)
            pass1_meshes[(ei, vsuffix)] = mesh
    n_slots = max(len(m) for m in pass1_meshes.values())
    thick_phantoms_per_slot = [0] * n_slots
    for mesh in pass1_meshes.values():
        for s, e in enumerate(mesh):
            if e["y_min"] <= e["y_max"]:   # ignore NO_OPs
                thick_phantoms_per_slot[s] = max(
                    thick_phantoms_per_slot[s],
                    thickness_extra_y(e["slope"]))
    n_thick = sum(thick_phantoms_per_slot) * 2
    print(f"thickness phantoms (max per slot, across variants): "
          f"{sum(1 for x in thick_phantoms_per_slot if x>0)} slots fattened, "
          f"+{n_thick} edges per variant")

    # Pass 2: rebuild with the agreed thickness plan, so every variant
    # has the same total edge count and same per-slot semantics.
    variants_per_expr = {}
    for ei, name in enumerate(expr_names):
        base = apply_expression(EXPRESSIONS[name])
        for vsuffix, transform in transforms:
            label = f"{name}/{vsuffix or 'open'}"
            mesh, _ = build_one_mesh(transform(base), label, log,
                                     canon_template,
                                     thick_phantoms_per_slot)
            variants_per_expr[(ei, vsuffix)] = mesh

    for line in log:
        print(line)

    sizes = sorted(set(len(m) for m in variants_per_expr.values()))
    print(f"variant edge counts (should be uniform): {sizes}")

    emit_vhdl(variants_per_expr, expr_names, VHDL_OUT)
    print(f"wrote {VHDL_OUT}")

    # Regenerate preview SVGs so they always match the emitted mesh.
    svg_script = Path(__file__).with_name("make_labels_svg.py")
    if svg_script.exists():
        subprocess.run([sys.executable, str(svg_script)], check=True)

if __name__ == "__main__":
    main()
