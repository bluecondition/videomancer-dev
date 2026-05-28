#!/usr/bin/env python3
"""Parse the user's clean wireframe SVG, normalize to FPGA pixel space,
tag every edge with an animation group, curate to a target edge count
that prioritises the head silhouette, and emit a VHDL mesh package
consumed by the bishop renderer.

Output: bishop_mesh_pkg.vhd

Group encoding:
    0  HEAD/STATIC  (head silhouette + interior wireframe)
    1  BROW         (Y-shift on K5)
    2  EYE          (suppress on blink)
    3  MOUTH        (Y-shift on K4)
"""
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

SVG = Path(__file__).with_name("wireframe_face_11.svg")
VHDL_OUT = Path(__file__).with_name("bishop_mesh_pkg.vhd")

DEDUP_TOL = 1.5
TARGET_EDGES = 120         # silhouette + features at full resolution
                           # plus ~28 short interior wireframe edges.
                           # Higher counts overflowed the per-scanline
                           # cycle budget even with cap=30 stamps/edge.
TARGET_FACE_HEIGHT = 600   # ~55% of HD frame (fits nicely)

# Interior polylines in the SVG jump between feature regions; those
# jumps appear as long cross-face diagonals when drawn unclipped.  We
# keep only edges shorter than this threshold (in SVG pixels) to
# suppress those jumps while keeping legitimate short wireframe lines.
MAX_INTERIOR_EDGE = 55.0

# Stride used to subsample the silhouette polygon.  The full SVG
# polygon has 46 vertices; stride=2 keeps every other one and lets
# straight chords fill the gaps.  Saves edges and per-scanline cycles
# at the cost of a slightly more "facetted" outline.
SILHOUETTE_STRIDE = 2

GROUP_STATIC = 0
GROUP_BROW   = 1
GROUP_EYE    = 2
GROUP_MOUTH  = 3

# Curation priority ranks.  Lower number = kept first.  Order chosen
# empirically to keep timing closing on HX4K — different orderings
# placed badly even at the same edge count.
PRI_HEAD     = 0   # head silhouette (group 0 but always kept)
PRI_EYE      = 1   # eye outlines
PRI_MOUTH    = 2   # mouth outline
PRI_BROW     = 3   # brow outlines
PRI_INTERIOR = 4   # interior wireframe polylines (group 0, leftover)


def NSDROP(t):
    return t.split("}", 1)[-1]


def parse_points(s):
    nums = re.findall(r"-?\d+(?:\.\d+)?", s)
    return [(float(nums[i]), float(nums[i + 1])) for i in range(0, len(nums), 2)]


def harvest_edges(root):
    """Return edges as list of (a, b, group, priority) tuples."""
    edges = []

    # 1) Head silhouette from <clipPath> polygon.  Subsample by stride
    # — fewer, longer chords give a slightly faceted but recognisable
    # outline while saving rasterizer cycles per scanline.
    for el in root.iter():
        if NSDROP(el.tag) == "clipPath":
            for child in el.iter():
                if NSDROP(child.tag) == "polygon":
                    pts = parse_points(child.attrib.get("points", ""))[::SILHOUETTE_STRIDE]
                    for a, b in zip(pts[:-1], pts[1:]):
                        edges.append((a, b, GROUP_STATIC, PRI_HEAD))
                    if pts:
                        edges.append((pts[-1], pts[0], GROUP_STATIC, PRI_HEAD))

    # 2) Visible polylines / polygons elsewhere
    for el in root.iter():
        tag = NSDROP(el.tag)
        if tag not in ("polyline", "polygon", "line"):
            continue
        # Skip masks (filled black)
        fill = el.attrib.get("fill", "").strip().lower()
        if fill in ("#000", "#000000", "black"):
            continue
        # Skip clipPath children — already harvested above
        # (xml ElementTree has no parent ref; check by id pattern)

        eid = (el.attrib.get("id", "") or "").lower()
        if "brow" in eid:
            group, pri = GROUP_BROW, PRI_BROW
        elif eid.startswith("eye"):
            group, pri = GROUP_EYE, PRI_EYE
        elif "mouth" in eid:
            group, pri = GROUP_MOUTH, PRI_MOUTH
        else:
            group, pri = GROUP_STATIC, PRI_INTERIOR

        if tag == "polyline":
            pts = parse_points(el.attrib.get("points", ""))
            for a, b in zip(pts[:-1], pts[1:]):
                if pri == PRI_INTERIOR:
                    dx = b[0] - a[0]
                    dy = b[1] - a[1]
                    if dx * dx + dy * dy > MAX_INTERIOR_EDGE * MAX_INTERIOR_EDGE:
                        continue  # cross-face jump, drop
                edges.append((a, b, group, pri))
        elif tag == "polygon":
            pts = parse_points(el.attrib.get("points", ""))
            for a, b in zip(pts[:-1], pts[1:]):
                edges.append((a, b, group, pri))
            if pts:
                edges.append((pts[-1], pts[0], group, pri))
        elif tag == "line":
            x1 = float(el.attrib.get("x1", 0))
            y1 = float(el.attrib.get("y1", 0))
            x2 = float(el.attrib.get("x2", 0))
            y2 = float(el.attrib.get("y2", 0))
            edges.append(((x1, y1), (x2, y2), group, pri))
    return edges


def dedupe_vertices(edges, tol=DEDUP_TOL):
    canonical = []

    def find_or_add(p):
        for i, c in enumerate(canonical):
            if abs(p[0] - c[0]) <= tol and abs(p[1] - c[1]) <= tol:
                return i
        canonical.append(p)
        return len(canonical) - 1

    edge_dict = {}  # (ia, ib) -> (group, priority, original_index)
    next_idx = 0
    for a, b, g, p in edges:
        ia = find_or_add(a)
        ib = find_or_add(b)
        if ia == ib:
            continue
        key = (min(ia, ib), max(ia, ib))
        if key in edge_dict:
            old_g, old_p, old_i = edge_dict[key]
            # Prefer lower priority number (more important)
            if p < old_p:
                edge_dict[key] = (g, p, old_i)
            # Otherwise keep the existing entry
        else:
            edge_dict[key] = (g, p, next_idx)
            next_idx += 1

    return canonical, [(a, b, g, p, i)
                       for (a, b), (g, p, i) in edge_dict.items()]


def curate(canonical, edges, target):
    """Sort by (priority, original_index) ascending, take first `target`."""
    sorted_e = sorted(edges, key=lambda e: (e[3], e[4]))
    return [(a, b, g) for a, b, g, _, _ in sorted_e[:target]]


def normalize_to_screen(canonical, edges):
    used = {a for a, _, _ in edges} | {b for _, b, _ in edges}
    xs = [canonical[i][0] for i in used]
    ys = [canonical[i][1] for i in used]
    cx_svg = (min(xs) + max(xs)) / 2
    cy_svg = (min(ys) + max(ys)) / 2
    height_svg = max(ys) - min(ys)
    scale = TARGET_FACE_HEIGHT / height_svg

    def to_screen(idx):
        x, y = canonical[idx]
        return ((x - cx_svg) * scale, (y - cy_svg) * scale)

    return [(to_screen(a), to_screen(b), g) for a, b, g in edges]


def to_dda_edge(a, b, group):
    """Encode an edge as an integer-slope DDA descriptor.

    For a steep (mostly-vertical) edge, integer slope = round(dx/dy).
    For a shallow edge, slope rounds toward 0 → it ends up vertical.
    Acceptable for 60+ edge meshes where shallow edges are rare.
    """
    x1, y1 = a; x2, y2 = b
    if y1 < y2:
        y_min, y_max = y1, y2
        x_top, x_bot = x1, x2
    elif y2 < y1:
        y_min, y_max = y2, y1
        x_top, x_bot = x2, x1
    else:
        # Horizontal: stretch to a 2-row span so the DDA samples it.
        y_min = y1 - 0.5
        y_max = y1 + 0.5
        x_top, x_bot = x1, x2
    dy = max(1, y_max - y_min)
    slope_int = int(round((x_bot - x_top) / dy))
    return {
        "y_min":  int(round(y_min)),
        "y_max":  int(round(y_max)),
        "x_top":  int(round(x_top)),
        "slope":  slope_int,
        "group":  group,
    }


def emit_vhdl(edges_dda, out_path):
    n = len(edges_dda)
    lines = []
    lines.append("-- Auto-generated by extract_mesh.py.  Do not edit by hand.")
    lines.append(f"-- Edges: {n}")
    counts = defaultdict(int)
    for e in edges_dda:
        counts[e["group"]] += 1
    for g in sorted(counts):
        gname = {0: "STATIC", 1: "BROW", 2: "EYE", 3: "MOUTH"}.get(g, str(g))
        lines.append(f"--   group {g} ({gname}): {counts[g]} edges")
    lines.append("")
    lines.append("library ieee;")
    lines.append("use ieee.std_logic_1164.all;")
    lines.append("use ieee.numeric_std.all;")
    lines.append("")
    lines.append("package bishop_mesh_pkg is")
    lines.append("")
    lines.append(f"    constant C_NUM_EDGES : natural := {n};")
    lines.append("")
    lines.append("    constant C_GRP_STATIC : natural := 0;")
    lines.append("    constant C_GRP_BROW   : natural := 1;")
    lines.append("    constant C_GRP_EYE    : natural := 2;")
    lines.append("    constant C_GRP_MOUTH  : natural := 3;")
    lines.append("")
    lines.append("    type t_int_array is array (natural range <>) of integer;")
    lines.append("")

    def emit_const(name, values, comment):
        lines.append(f"    -- {comment}")
        lines.append(f"    constant {name} : t_int_array(0 to C_NUM_EDGES - 1) := (")
        chunks = []
        for i in range(0, len(values), 8):
            row = ", ".join(f"{v:>5}" for v in values[i:i + 8])
            chunks.append("        " + row)
        lines.append(",\n".join(chunks))
        lines.append("    );")
        lines.append("")

    emit_const("C_EDGE_Y_MIN", [e["y_min"] for e in edges_dda], "y_min per edge")
    emit_const("C_EDGE_Y_MAX", [e["y_max"] for e in edges_dda], "y_max per edge")
    emit_const("C_EDGE_X_TOP", [e["x_top"] for e in edges_dda], "x at y_min")
    emit_const("C_EDGE_SLOPE", [e["slope"] for e in edges_dda], "integer slope dx/dy (rounded)")
    emit_const("C_EDGE_GROUP", [e["group"] for e in edges_dda], "animation group")

    lines.append("end package bishop_mesh_pkg;")
    out_path.write_text("\n".join(lines) + "\n")


def main():
    tree = ET.parse(SVG)
    root = tree.getroot()
    raw = harvest_edges(root)
    canonical, edges_idx = dedupe_vertices(raw)
    print(f"raw edge instances : {len(raw)}")
    print(f"unique vertices    : {len(canonical)}")
    print(f"unique edges       : {len(edges_idx)}")

    pri_counts = defaultdict(int)
    for e in edges_idx:
        pri_counts[e[3]] += 1
    print(f"by priority: HEAD={pri_counts[PRI_HEAD]} EYE={pri_counts[PRI_EYE]} "
          f"MOUTH={pri_counts[PRI_MOUTH]} BROW={pri_counts[PRI_BROW]} "
          f"INTERIOR={pri_counts[PRI_INTERIOR]}")

    curated = curate(canonical, edges_idx, TARGET_EDGES)
    print(f"curated to {TARGET_EDGES}: {len(curated)}")
    counts = defaultdict(int)
    for _, _, g in curated:
        counts[g] += 1
    for g in sorted(counts):
        print(f"  group {g}: {counts[g]} edges")

    screen = normalize_to_screen(canonical, curated)
    edges_dda = [to_dda_edge(*e) for e in screen]

    y_mins = [e["y_min"] for e in edges_dda]
    y_maxs = [e["y_max"] for e in edges_dda]
    x_tops = [e["x_top"] for e in edges_dda]
    slopes = [e["slope"] for e in edges_dda]
    print(f"y range : [{min(y_mins)}, {max(y_maxs)}]")
    print(f"x range : [{min(x_tops)}, {max(x_tops)}]")
    print(f"slope   : [{min(slopes)}, {max(slopes)}]")

    emit_vhdl(edges_dda, VHDL_OUT)
    print(f"wrote {VHDL_OUT}")


if __name__ == "__main__":
    main()
