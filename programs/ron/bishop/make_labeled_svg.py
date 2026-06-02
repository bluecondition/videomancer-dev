#!/usr/bin/env python3
"""Emit face12_labeled.svg: the dense wireframe with every vertex numbered,
so the user can reference point numbers to request adjustments.  Also writes
face12_points.txt (number -> SVG x,y) as the lookup for applying them.

Vertices are numbered top-to-bottom, then left-to-right.  Same coordinate
space as face12.svg (viewBox 960x720) so it matches the Edge view.
"""
import re
from pathlib import Path

SVG_IN = Path(__file__).with_name("face12.svg")
SVG_OUT = Path(__file__).with_name("face12_labeled.svg")
TXT_OUT = Path(__file__).with_name("face12_points.txt")


def floats(s):
    n = [float(x) for x in re.findall(r'-?\d+\.?\d*', s)]
    return list(zip(n[0::2], n[1::2]))


def main():
    svg = SVG_IN.read_text()
    polylines = re.findall(r'<polyline points="([^"]+)"', svg)

    # unique vertices + the edge list (interior wireframe only, no clip ring)
    verts = {}
    edges = []
    for pl in polylines:
        pts = floats(pl)
        for i in range(len(pts) - 1):
            a = (round(pts[i][0], 2), round(pts[i][1], 2))
            b = (round(pts[i + 1][0], 2), round(pts[i + 1][1], 2))
            if a != b:
                verts[a] = True
                verts[b] = True
                edges.append((a, b))

    # number top-to-bottom, then left-to-right
    ordered = sorted(verts.keys(), key=lambda p: (p[1], p[0]))
    num = {p: i + 1 for i, p in enumerate(ordered)}

    xs = [p[0] for p in ordered]
    ys = [p[1] for p in ordered]
    vx0, vx1 = min(xs) - 20, max(xs) + 20
    vy0, vy1 = min(ys) - 20, max(ys) + 20
    vw, vh = vx1 - vx0, vy1 - vy0

    L = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vx0} {vy0} {vw} {vh}" '
         f'width="{int(vw*2)}" height="{int(vh*2)}">']
    L.append(f'<rect x="{vx0}" y="{vy0}" width="{vw}" height="{vh}" fill="#000"/>')
    # wireframe
    L.append('<g stroke="#2a7a30" stroke-width="0.8" fill="none">')
    for a, b in edges:
        L.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}"/>')
    L.append('</g>')
    # vertices + numbers
    L.append('<g font-family="monospace" font-size="7" fill="#ffd000">')
    for p in ordered:
        L.append(f'<circle cx="{p[0]}" cy="{p[1]}" r="1.3" fill="#ff3030"/>')
        L.append(f'<text x="{p[0]+2}" y="{p[1]-2}">{num[p]}</text>')
    L.append('</g>')
    L.append('</svg>')
    SVG_OUT.write_text("\n".join(L) + "\n")

    TXT_OUT.write_text(
        "# point  x  y  (SVG coords, viewBox 960x720; +y is down)\n" +
        "".join(f"{num[p]:>4}  {p[0]:>7.2f}  {p[1]:>7.2f}\n" for p in ordered))

    print(f"{len(ordered)} vertices, {len(edges)} edge-instances")
    print(f"wrote {SVG_OUT}")
    print(f"wrote {TXT_OUT}")


if __name__ == "__main__":
    main()
