#!/usr/bin/env python3
"""Render labeled diagrams of the Bishop face mesh, one SVG per
expression in EXPRESSIONS.

Outputs face_labels_<expression>.svg under the bishop directory.  The
SVG projection (scale + offset) is computed once from the NEUTRAL
vertex bounds so all expressions share the same coordinate frame and
visual comparisons across expressions are easy.

Boundary edges drawn green; detail edges blue."""

from pathlib import Path
from face_mesh import VERTICES, EDGES, EXPRESSIONS, close_mouth, close_eyes
from build_face_mesh import is_boundary_edge

HERE = Path(__file__).parent

W, H = 1200, 760
MARGIN_X = 540   # extra room on the right for the legend

LABEL_HINTS = {
    "crown":     ("middle", -16),
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

# Project from VERTICES (neutral) bounds so every expression uses the
# same coordinate system.  Morphs only ever move brows/eyes/mouth a few
# px, so they all stay well within the neutral bounding box.
xs = [v[0] for v in VERTICES.values()]
ys = [v[1] for v in VERTICES.values()]
src_w = max(xs) - min(xs)
src_h = max(ys) - min(ys)
SCALE = min((W - MARGIN_X - 80) / src_w, (H - 80) / src_h)
OX = 40 + (W - MARGIN_X - 80 - src_w * SCALE) / 2 - min(xs) * SCALE
OY = 40 + (H - 80 - src_h * SCALE) / 2 - min(ys) * SCALE

def project(point):
    x, y = point
    return x * SCALE + OX, y * SCALE + OY

def apply_deltas(deltas):
    return {name: (x + deltas.get(name, (0, 0))[0],
                   y + deltas.get(name, (0, 0))[1])
            for name, (x, y) in VERTICES.items()}

def render_svg(expr_name, deltas, out_path, verts=None):
    if verts is None:
        verts = apply_deltas(deltas)
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}">',
        '<rect width="100%" height="100%" fill="#000"/>',
        '<style>'
        '.bnd{stroke:#3f3;stroke-width:1.5;fill:none}'
        '.det{stroke:#5af;stroke-width:1.5;fill:none}'
        '.vert{fill:#fc3;stroke:none}'
        '.lbl{fill:#eee;font-family:monospace;font-size:11px}'
        '.title{fill:#fff;font-family:monospace;font-size:16px;font-weight:bold}'
        '.subtitle{fill:#aaa;font-family:monospace;font-size:11px}'
        '.legend{fill:#bbb;font-family:monospace;font-size:11px}'
        '</style>',
    ]

    # Edges
    for a, b in EDGES:
        ax, ay = project(verts[a])
        bx, by = project(verts[b])
        cls = "bnd" if is_boundary_edge(a, b) else "det"
        lines.append(f'<line class="{cls}" x1="{ax:.1f}" y1="{ay:.1f}" '
                     f'x2="{bx:.1f}" y2="{by:.1f}"/>')

    # Vertices + labels
    used = set(v for e in EDGES for v in e)
    for name in VERTICES:
        if name not in used:
            continue
        x, y = project(verts[name])
        moved = name in deltas and deltas[name] != (0, 0)
        # Vertices that moved this expression are highlighted in cyan.
        fill = "#0ff" if moved else "#fc3"
        lines.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3" fill="{fill}"/>')
        anchor, dy = LABEL_HINTS.get(name, ("start", 4))
        dx = {"start": 6, "end": -6, "middle": 0}[anchor]
        lines.append(f'<text class="lbl" x="{x + dx:.1f}" y="{y + dy:.1f}" '
                     f'text-anchor="{anchor}">{name}</text>')

    # Right-side panel: expression name + delta list
    lx = W - MARGIN_X + 20
    ly = 60
    lines.append(f'<text class="title" x="{lx}" y="{ly}">'
                 f'Bishop face — {expr_name}</text>')
    ly += 22
    lines.append(f'<text class="subtitle" x="{lx}" y="{ly}">'
                 f'Cyan dots = vertices moved by this expression</text>')
    ly += 22

    if deltas:
        lines.append(f'<text class="legend" x="{lx}" y="{ly}" '
                     f'style="font-weight:bold">Per-vertex deltas (dx, dy):</text>')
        ly += 18
        for name in sorted(deltas):
            dx, dy = deltas[name]
            if (dx, dy) == (0, 0):
                continue
            lines.append(f'<text class="legend" x="{lx + 12}" y="{ly}">'
                         f'{name:<10} ({dx:+d}, {dy:+d})</text>')
            ly += 14
    else:
        lines.append(f'<text class="legend" x="{lx}" y="{ly}">'
                     f'(baseline pose — no deltas)</text>')
        ly += 14

    ly += 14
    lines.append(f'<text class="legend" x="{lx}" y="{ly}" '
                 f'style="font-weight:bold">Edge colors</text>')
    ly += 16
    lines.append(f'<text class="legend" x="{lx + 12}" y="{ly}" fill="#3f3">'
                 f'green: boundary (EOR fill)</text>')
    ly += 14
    lines.append(f'<text class="legend" x="{lx + 12}" y="{ly}" fill="#5af">'
                 f'blue:  detail (visible only)</text>')
    ly += 14

    lines.append('</svg>')
    out_path.write_text("\n".join(lines) + "\n")

def main():
    for name, deltas in EXPRESSIONS.items():
        # Open (default) pose.
        out = HERE / f"face_labels_{name}.svg"
        render_svg(name, deltas, out)
        print(f"wrote {out}")
        # Closed-mouth + closed-eyes preview (S7 + S8 both on).
        verts_closed = close_eyes(close_mouth(apply_deltas(deltas)))
        out_c = HERE / f"face_labels_{name}_closed.svg"
        render_svg(name + " (mouth+eyes closed)", deltas, out_c, verts=verts_closed)
        print(f"wrote {out_c}")

if __name__ == "__main__":
    main()
