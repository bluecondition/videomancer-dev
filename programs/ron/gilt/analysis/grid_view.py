"""Render a painting (or a crop of it) with a labelled 5 %/10 % grid for tracing zone polygons.
usage: grid_view.py <img> <out.png> [x0 y0 x1 y1 (fractions)] [maxside]"""
import sys
from PIL import Image, ImageDraw
Image.MAX_IMAGE_PIXELS = None
src, out = sys.argv[1], sys.argv[2]
x0, y0, x1, y1 = (map(float, sys.argv[3:7]) if len(sys.argv) >= 7 else (0, 0, 1, 1))
maxside = int(sys.argv[7]) if len(sys.argv) >= 8 else 1000
im = Image.open(src).convert("RGB")
W, H = im.size
im = im.crop((int(x0*W), int(y0*H), int(x1*W), int(y1*H)))
im.thumbnail((maxside, maxside), Image.LANCZOS)
w, h = im.size
d = ImageDraw.Draw(im)
step = 0.05 if (x1 - x0) > 0.5 else 0.025
def frange(a, b, s):
    v = round(a / s) * s
    while v <= b + 1e-9:
        yield round(v, 4); v += s
for fx in frange(x0, x1, step):
    px = (fx - x0) / (x1 - x0) * w
    major = abs(fx * 10 - round(fx * 10)) < 1e-6
    d.line([(px, 0), (px, h)], fill=(0, 255, 255) if major else (0, 140, 140), width=1)
    if major: d.text((px + 2, 2), f"{fx:.1f}", fill=(255, 255, 0))
for fy in frange(y0, y1, step):
    py = (fy - y0) / (y1 - y0) * h
    major = abs(fy * 10 - round(fy * 10)) < 1e-6
    d.line([(0, py), (w, py)], fill=(255, 0, 255) if major else (140, 0, 140), width=1)
    if major: d.text((2, py + 2), f"{fy:.1f}", fill=(255, 255, 0))
im.save(out)
