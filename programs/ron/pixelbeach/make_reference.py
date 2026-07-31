#!/usr/bin/env python3
"""Render color_reference.png: every pixelbeach colour as a numbered swatch.

Numbering is GLOBAL and stable: PAL_ROM colours first (per palette), then the
sky gradient keyframes, then the sun journeys.  Refer to colours by number
(e.g. "#37") when requesting changes; genpal.py is the source of truth.
"""
import genpal
from PIL import Image, ImageDraw

SW, SH, GAP, LM = 66, 52, 4, 8
sections = []
n = 1
for pname, pal in genpal.PALS:
    row = []
    for cn in genpal.ORDER[:13]:
        row.append((n, cn, pal[cn])); n += 1
    sections.append((f"PALETTE {pname.upper()}", [row]))
g2 = [(n0+i, p, pal["GULL2"]) for i,(p,pal) in enumerate(genpal.PALS)] if False else None
for name, ramp in (("SKY DAY upper", genpal.DAY_HI), ("SKY DAY horizon", genpal.DAY_LO),
                   ("SKY NIGHT upper", genpal.NGT_HI), ("SKY NIGHT horizon", genpal.NGT_LO),
                   ("SUN journey (day)", genpal.SUN_DAY), ("MOON journey (night)", genpal.SUN_NGT)):
    row = []
    for i, c in enumerate(ramp):
        row.append((n, f"k{i}", c)); n += 1
    sections.append((name + "  (0 = slider down ... 15 = up)", [row]))
row = []
for pname, pal in genpal.PALS:
    row.append((n, f"G2 {pname[:5]}", pal["GULL2"])); n += 1
sections.append(("GULL2 (white gull plumage, per palette)", [row]))

cols = max(len(r) for _, rows in sections for r in rows)
W = LM*2 + cols*(SW+GAP)
H = LM + sum(22 + len(rows)*(SH+GAP) + 10 for _, rows in sections)
img = Image.new("RGB", (W, H), (24, 24, 28))
d = ImageDraw.Draw(img)
y = LM
for title, rows in sections:
    d.text((LM, y), title, fill=(230, 230, 235)); y += 22
    for row in rows:
        x = LM
        for num, name, (r, g, b) in row:
            d.rectangle([x, y, x+SW, y+SH], fill=(r, g, b), outline=(90, 90, 96))
            tf = (0, 0, 0) if (r+g+g+b) > 500 else (255, 255, 255)
            d.text((x+4, y+3), str(num), fill=tf)
            d.text((x+4, y+SH-14), name[:9], fill=tf)
            x += SW+GAP
        y += SH+GAP
    y += 10
img.save("color_reference.png")
print(f"color_reference.png: {n-1} numbered colours, {W}x{H}")
