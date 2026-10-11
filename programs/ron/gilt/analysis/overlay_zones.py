"""Render the proposed zone polygons over each zoned painting for approval.
Writes figs/zones_<painting>.png and figs/zones_sheet.png."""
import os

import numpy as np
from PIL import Image, ImageDraw

import lab

COL = {"flesh": (255, 120, 160), "hair": (40, 40, 255), "robe": (255, 40, 40),
       "field": (0, 220, 255), "meadow": (40, 255, 40)}
SUB = {"robe_rect": (255, 0, 0), "robe_dress": (255, 140, 0), "robe_gown": (255, 60, 60),
       "robe_halo": (255, 0, 200), "robe_panel": (160, 0, 255), "robe_chair": (255, 220, 0),
       "robe_ovals": (255, 60, 60), "robe_checker": (160, 0, 255), "robe_dress_w": (255, 160, 0)}


def render(key, side=900):
    rgb, box, full = lab.load(key, long_side=side)
    L, names = lab.label_map(key, rgb.shape, box, full)
    tint = np.zeros_like(rgb, dtype=np.float32)
    for i, n in enumerate(names):
        c = SUB.get(n) or COL[lab.ZONES["group"][n]]
        tint[L == i] = c
    out = (0.55 * rgb + 0.45 * tint).astype(np.uint8)
    # outline each label region
    edge = np.zeros(L.shape, bool)
    edge[:-1] |= L[:-1] != L[1:]
    edge[:, :-1] |= L[:, :-1] != L[:, 1:]
    out[edge] = (255, 255, 255)
    im = Image.fromarray(out)
    d = ImageDraw.Draw(im)
    y = 4
    for i, n in enumerate(names):
        frac = float((L == i).mean())
        c = SUB.get(n) or COL[lab.ZONES["group"][n]]
        d.rectangle([4, y, 16, y + 12], fill=c, outline=(255, 255, 255))
        d.text((20, y), f"{n} ({lab.ZONES['group'][n]}) {frac*100:.1f}%", fill=(255, 255, 255))
        y += 16
    return im


def main():
    os.makedirs(lab.FIGS, exist_ok=True)
    ims = []
    for key in lab.ZONED:
        im = render(key)
        im.save(os.path.join(lab.FIGS, f"zones_{key}.png"))
        ims.append(im)
    h = max(i.height for i in ims)
    sheet = Image.new("RGB", (sum(i.width for i in ims) + 10 * (len(ims) - 1), h), (30, 30, 30))
    x = 0
    for i in ims:
        sheet.paste(i, (x, 0))
        x += i.width + 10
    sheet.save(os.path.join(lab.FIGS, "zones_sheet.png"))
    print("wrote", os.path.join(lab.FIGS, "zones_sheet.png"), sheet.size)


if __name__ == "__main__":
    main()
