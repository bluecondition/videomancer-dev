#!/usr/bin/env python3
"""Render the CURRENT wireframe (face12.svg minus build_svg_mesh.REMOVED_EDGES)
to /tmp/face_preview.png at true proportions, for judging edge edits.
Pass 'labels' as arg to overlay the point numbers."""
import re
import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import build_svg_mesh as B

SVG = Path(__file__).with_name("face12.svg")


def floats(s):
    n = [float(x) for x in re.findall(r'-?\d+\.?\d*', s)]
    return list(zip(n[0::2], n[1::2]))


def main(labels=False):
    svg = SVG.read_text()
    verts, edges = {}, []
    for pl in re.findall(r'<polyline points="([^"]+)"', svg):
        p = floats(pl)
        for i in range(len(p) - 1):
            a = (round(p[i][0], 2), round(p[i][1], 2))
            b = (round(p[i + 1][0], 2), round(p[i + 1][1], 2))
            if a != b:
                verts[a] = 1; verts[b] = 1; edges.append((a, b))
    ordered = sorted(verts, key=lambda q: (q[1], q[0]))
    num = {p: i + 1 for i, p in enumerate(ordered)}
    removed = {frozenset(pr) for pr in B.REMOVED_EDGES}
    kept = [(a, b) for a, b in edges
            if frozenset({num[a], num[b]}) not in removed]
    inv = {n: p for p, n in num.items()}
    for pa, pb in getattr(B, "ADDED_EDGES", []):
        if pa in inv and pb in inv:
            kept.append((inv[pa], inv[pb]))
    move = {inv[n]: (float(x), float(y))
            for n, (x, y) in getattr(B, "MOVED_POINTS", {}).items() if n in inv}
    def mv(p):
        return move.get((round(p[0], 2), round(p[1], 2)), p)
    kept = [(mv(a), mv(b)) for a, b in kept]

    xs = [p[0] for p in ordered]; ys = [p[1] for p in ordered]
    x0, y0 = min(xs) - 15, min(ys) - 15
    S = 2.6
    W = int((max(xs) - min(xs) + 30) * S); H = int((max(ys) - min(ys) + 30) * S)
    img = Image.new('RGB', (W, H), (0, 0, 0)); d = ImageDraw.Draw(img)
    def T(p): return ((p[0] - x0) * S, (p[1] - y0) * S)
    for a, b in kept:
        d.line([T(a), T(b)], fill=(60, 230, 75), width=2)
    if labels:
        try:
            fnt = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 13)
        except Exception:
            fnt = ImageFont.load_default()
        for p in ordered:
            x, y = T(mv(p))
            d.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(230, 40, 40))
            d.text((x + 3, y - 14), str(num[p]), fill=(255, 210, 0), font=fnt)
    img.save('/tmp/face_preview.png')
    print(f"{len(kept)} edges kept ({len(edges)-len(kept)} removed) -> /tmp/face_preview.png {img.size}")


if __name__ == "__main__":
    main(labels=("labels" in sys.argv))
