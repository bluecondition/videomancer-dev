#!/usr/bin/env python3
"""Render a labeled diagram of the Bishop face mesh.

Reads VERTICES and EDGES from face_mesh.py and emits face_labels.svg
showing the wireframe with every vertex labeled.  Boundary edges are
green, detail edges (brows, mouth/nose centerlines) are blue."""

from pathlib import Path
from face_mesh import VERTICES, EDGES
from build_face_mesh import is_boundary_edge

OUT = Path(__file__).with_name("face_labels.svg")

W, H = 1200, 760
MARGIN_X = 540   # extra room on the right for the legend / labels

# Label offset hints — by default labels go to the right of the vertex,
# but for verts on the right side of the face it reads better to put
# them on the LEFT.  Custom anchors per vertex name.
LABEL_HINTS = {
    "crown":     ("middle", -16),    # above the vertex
    "temple_l":  ("end",     -8),
    "temple_r":  ("start",    8),
    "side_l":    ("end",     -8),
    "side_r":    ("start",    8),
    "cheek_l":   ("end",     -8),
    "cheek_r":   ("start",    8),
    "jaw_l":     ("end",     -8),
    "jaw_r":     ("start",    8),
    "chin_l":    ("end",     -8),
    "chin_r":    ("start",    8),
    "brow_l_o":  ("end",     -6),
    "brow_l_p":  ("middle", -12),
    "brow_l_i":  ("start",    6),
    "brow_r_i":  ("end",     -6),
    "brow_r_p":  ("middle", -12),
    "brow_r_o":  ("start",    6),
    "eye_l_o":   ("end",     -6),
    "eye_l_t":   ("middle", -10),
    "eye_l_i":   ("start",    6),
    "eye_l_b":   ("middle",  16),
    "eye_r_i":   ("end",     -6),
    "eye_r_t":   ("middle", -10),
    "eye_r_o":   ("start",    6),
    "eye_r_b":   ("middle",  16),
    "nose_top":  ("start",    8),
    "nose_l":    ("end",     -6),
    "nose_r":    ("start",    6),
    "nose_tip":  ("middle",  16),
    "mouth_l":   ("end",     -8),
    "mouth_t":   ("middle", -10),
    "mouth_r":   ("start",    8),
    "mouth_b":   ("middle",  16),
}

# Center the face in the left portion of the canvas.
xs = [v[0] for v in VERTICES.values()]
ys = [v[1] for v in VERTICES.values()]
src_w = max(xs) - min(xs)
src_h = max(ys) - min(ys)
scale = min((W - MARGIN_X - 80) / src_w, (H - 80) / src_h)
ox = 40 + (W - MARGIN_X - 80 - src_w * scale) / 2 - min(xs) * scale
oy = 40 + (H - 80 - src_h * scale) / 2 - min(ys) * scale

def project(v):
    x, y = VERTICES[v]
    return x * scale + ox, y * scale + oy

lines = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}">',
    '<rect width="100%" height="100%" fill="#000"/>',
    '<style>'
    '.bnd{stroke:#3f3;stroke-width:1.5;fill:none}'
    '.det{stroke:#5af;stroke-width:1.5;fill:none}'
    '.vert{fill:#fc3;stroke:none}'
    '.lbl{fill:#eee;font-family:monospace;font-size:11px}'
    '.title{fill:#fff;font-family:monospace;font-size:14px;font-weight:bold}'
    '.legend{fill:#bbb;font-family:monospace;font-size:11px}'
    '</style>',
]

# Edges
for a, b in EDGES:
    ax, ay = project(a)
    bx, by = project(b)
    cls = "bnd" if is_boundary_edge(a, b) else "det"
    lines.append(f'<line class="{cls}" x1="{ax:.1f}" y1="{ay:.1f}" '
                 f'x2="{bx:.1f}" y2="{by:.1f}"/>')

# Vertices + labels
for name in VERTICES:
    if not any(name == a or name == b for a, b in EDGES):
        continue
    x, y = project(name)
    lines.append(f'<circle class="vert" cx="{x:.1f}" cy="{y:.1f}" r="2.5"/>')
    anchor, dy = LABEL_HINTS.get(name, ("start", 4))
    dx = {"start": 6, "end": -6, "middle": 0}[anchor]
    lines.append(f'<text class="lbl" x="{x + dx:.1f}" y="{y + dy:.1f}" '
                 f'text-anchor="{anchor}">{name}</text>')

# Legend (right side)
lx = W - MARGIN_X + 20
ly = 60
lines.append(f'<text class="title" x="{lx}" y="{ly}">Bishop face mesh — vertex names</text>')
ly += 28
groups = [
    ("Silhouette (boundary)", "#3f3", [
        "crown, temple_l/r, side_l/r,",
        "cheek_l/r, jaw_l/r, chin_l/r",
    ]),
    ("Brows (detail)", "#5af", [
        "brow_l_o (outer) — brow_l_p (peak) — brow_l_i (inner)",
        "brow_r_i (inner) — brow_r_p (peak) — brow_r_o (outer)",
    ]),
    ("Eyes (boundary diamond)", "#3f3", [
        "eye_*_o (outer), eye_*_t (top),",
        "eye_*_i (inner), eye_*_b (bottom)",
    ]),
    ("Nose (boundary triangle + detail nostril)", "#3f3", [
        "nose_top (peak)",
        "nose_l / nose_r (base corners, connected by",
        "  a detail nostril line)",
        "nose_tip (bottom)",
    ]),
    ("Mouth (boundary diamond + detail centerline)", "#3f3", [
        "mouth_l (left), mouth_t (top),",
        "mouth_r (right), mouth_b (bottom)",
        "mouth_l→mouth_r is a detail centerline",
    ]),
    ("Tip stripping", "#bbb", [
        "Each convex tip strips N boundary rows so the",
        "two paired edges' stamps don't overlap (which",
        "would break EOR fill parity).  Phantom detail",
        "edges redraw the stripped rows so the wireframe",
        "still meets at the vertex.",
        "",
        "N: crown=2, eye/mouth corners=2, nose_tip=3,",
        "   nose_top=15 (shallow slope)",
    ]),
]
for header, color, body in groups:
    lines.append(f'<text class="legend" x="{lx}" y="{ly}" fill="{color}" style="font-weight:bold">{header}</text>')
    ly += 16
    for line in body:
        lines.append(f'<text class="legend" x="{lx + 12}" y="{ly}">{line}</text>')
        ly += 14
    ly += 6

lines.append('</svg>')
OUT.write_text("\n".join(lines) + "\n")
print(f"wrote {OUT}")
