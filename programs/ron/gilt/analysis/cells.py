"""Phase 1 cell geometry: segment the ornament cells of the robe zones and measure
size, aspect, shape class and rim width.

A "cell" here = a connected patch whose colour contrasts with the leaf around it
(a black bar, a silver tablet, a ring-disc, an oval, a square). Gold-on-gold cells
that differ only in engraved line texture are not separable by colour and are not
counted -- these stats describe the cells that READ at a distance.

Method (per region):
  1. rescale so the robe group is 1200 px wide; CIELAB; Gaussian smooth (1.5 px)
  2a. "ground" mode: the region's GROUND colour is named per region (gold leaf, or
      the red-orange enamel between Fulfillment's ovals); cells = everything else,
      opened (r=2 px), then touching cells are split by a watershed on the distance
      transform (h-maxima markers, h = 3 px) -- an oval with an eye inside stays one cell
  2b. "classes" mode (checker block: black and white squares on white grout):
      k-means (k=7) colour classes, connected components per class
  3. drop components smaller than 0.8 % of robe width (equivalent diameter) and any
      larger than 25 % of the region
  4. measure equivalent diameter, bbox long side, bbox aspect h/w, extent,
     solidity, circularity, holes -> shape class; rings give rim width

usage: python cells.py   -> results/cells.json + figs/cells_<region>.png
"""
import json
import os

import numpy as np
from PIL import Image
from scipy import ndimage as ndi
from skimage import measure, morphology

import lab

OUT = os.path.join(lab.HERE, "results")
ROBE_W = 1200          # px the robe group is rescaled to
MIN_EQD = 0.008        # of robe width

GOLD, RED = 0, 4         # lab.FAMILIES indices
# ground: "leaf" = gold family with L* >= 40 (bright leaf; the Kiss's "black" bars are a
# dark umber that the gold family would otherwise swallow), "red" = red family.
# Adele I is NOT segmented here: her cells are gold-on-gold, separated only by dark
# outlines that do not close, and every automatic method tried (colour classes, leaf
# ground, enclosed-by-outline watershed) carved arbitrary blobs. Her motifs are
# measured by hand instead: manual_measurements.json.
REGIONS = [               # (painting, region, mode, ground)
    ("kiss", "robe_rect", "ground", "leaf"), ("kiss", "robe_dress", "ground", "leaf"),
    ("stoclet_fulfillment", "robe_ovals", "ground", "red"),
    ("stoclet_fulfillment", "robe_checker", "classes", None),
    ("stoclet_fulfillment", "robe_dress_w", "ground", "leaf"),
]
GROUND_FRAC = 0.25       # components above this fraction of the region are dropped


def shape_class(p, filled_ratio):
    if filled_ratio > 1.4:
        return "ring"
    ext, sol = p.extent, p.solidity
    circ = 4 * np.pi * p.area / max(p.perimeter, 1) ** 2
    if ext >= 0.80 and sol >= 0.90:
        return "rect"
    if circ >= 0.70 and 0.68 <= ext < 0.85 and sol >= 0.90:
        return "disc"
    if 0.40 <= ext < 0.65 and sol >= 0.85:
        return "triangle"
    return "irregular"


def components(Lab3, zone, mode, ground):
    """Yield (label image, n) of candidate cells."""
    from skimage.segmentation import watershed
    if mode == "ground":
        fam = lab.classify(Lab3)
        if ground == "leaf":
            g = (fam == GOLD) & (Lab3[..., 0] >= 40)
        else:
            g = fam == RED
        g = morphology.closing(g & zone, morphology.disk(2))
        fg = zone & ~g
        fg = morphology.opening(fg, morphology.disk(2))
        dt = ndi.distance_transform_edt(fg)
        markers = measure.label(morphology.h_maxima(dt, 3))
        yield watershed(-dt, markers, mask=fg)
    else:
        C, _ = lab.kmeans(Lab3[zone], 7, seed=11)
        cls = (((Lab3[..., None, :] - C[None, None]) ** 2).sum(-1)).argmin(-1)
        cls[~zone] = 255
        for k in range(len(C)):
            yield measure.label(morphology.opening(cls == k, morphology.disk(2)), connectivity=1)


def analyse(key, region, mode, ground, rng):
    rgb, box, full = lab.load(key, long_side=None)
    labels, names = lab.label_map(key, rgb.shape, box, full)
    robe = lab.group_masks(labels, names)["robe"]
    xs = np.where(robe.any(0))[0]
    robe_w_native = xs.max() - xs.min() + 1
    s = ROBE_W / robe_w_native
    im = Image.fromarray(rgb).resize((round(rgb.shape[1] * s), round(rgb.shape[0] * s)), Image.BOX)
    rgb = np.asarray(im)
    labels, names = lab.label_map(key, rgb.shape, box, full)
    if region not in names:
        return None
    zone = labels == names.index(region)
    ys, xs = np.where(zone)
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    zone = zone[y0:y1, x0:x1]
    sub = rgb[y0:y1, x0:x1]
    Lab3 = lab.to_lab(sub)
    Lab3 = np.stack([ndi.gaussian_filter(Lab3[..., i], 1.5) for i in range(3)], -1)
    zone_area = zone.sum()
    # px at 1080-equivalent per px here (canvas short side -> 1080)
    canvas_short = min(box[2] - box[0], box[3] - box[1]) * s
    to1080 = 1080.0 / canvas_short
    min_area = np.pi / 4 * (MIN_EQD * ROBE_W) ** 2
    cells = []
    vis = np.zeros(sub.shape[:2], np.int32)
    for lbl in components(Lab3, zone, mode, ground):
        for p in measure.regionprops(lbl):
            if p.area < min_area or p.area > GROUND_FRAC * zone_area:
                continue
            filled = p.area_filled if hasattr(p, "area_filled") else p.filled_area
            fr = filled / p.area
            h = p.bbox[2] - p.bbox[0]
            w = p.bbox[3] - p.bbox[1]
            eqd = np.sqrt(4 * filled / np.pi)
            rim = None
            if fr > 1.4:
                # ring thickness: 2 x the median distance-to-edge along its skeleton
                img = p.image
                dt = ndi.distance_transform_edt(np.pad(img, 1))[1:-1, 1:-1]
                sk = morphology.skeletonize(img)
                if sk.any():
                    rim = 2 * float(np.median(dt[sk]))
            # touches the region border?
            bb = p.bbox
            pad = zone[max(bb[0] - 2, 0):bb[2] + 2, max(bb[1] - 2, 0):bb[3] + 2]
            touch = not pad.all()
            cells.append(dict(eqd_pct=100 * eqd / ROBE_W, long_pct=100 * max(h, w) / ROBE_W,
                              eqd_px1080=eqd * to1080, aspect=h / max(w, 1), shape=shape_class(p, fr),
                              rim_px1080=rim * to1080 if rim else None,
                              rim_ratio=rim / eqd if rim else None, touch=touch))
            vis[lbl == p.label] = len(cells)
    # figure: cells coloured randomly over a dimmed region
    col = rng.integers(60, 255, (len(cells) + 1, 3)).astype(np.uint8)
    out = (sub * 0.35).astype(np.uint8)
    out[vis > 0] = col[vis[vis > 0]]
    out[~zone] = (0, 0, 0)
    both = np.concatenate([sub, out], 1)
    Image.fromarray(both).save(os.path.join(lab.FIGS, f"cells_{key}_{region}.png"))
    return dict(mode=mode, robe_width_native_px=int(robe_w_native), px1080_per_robe_pct=round(ROBE_W / 100 * to1080, 2),
                region_area_frac_of_robe=float(zone_area / (robe.sum() * s * s)), cells=cells)


def summarise(cells, interior_only=False):
    c = [x for x in cells if not (interior_only and x["touch"])]
    if len(c) < 5:
        return None
    e = np.array([x["eqd_pct"] for x in c])
    lg = np.array([x["long_pct"] for x in c])
    a = np.array([x["aspect"] for x in c])
    p1080 = np.array([x["eqd_px1080"] for x in c])
    shapes = {}
    for x in c:
        shapes[x["shape"]] = shapes.get(x["shape"], 0) + 1
    rims = [x for x in c if x["rim_ratio"]]
    q = [10, 25, 50, 75, 90, 99]
    r = lambda v: [round(float(t), 2) for t in v]
    return dict(n=len(c),
                eqd_pct_robe_width=r(np.percentile(e, q)),
                long_side_pct_robe_width=r(np.percentile(lg, q)),
                eqd_px1080=r(np.percentile(p1080, q)),
                lognormal_eqd=dict(mu_ln_pct=round(float(np.log(e).mean()), 3),
                                   sigma_ln=round(float(np.log(e).std()), 3)),
                aspect_hw=r(np.percentile(a, [10, 50, 90])),
                shapes={k: round(v / len(c), 3) for k, v in sorted(shapes.items())},
                rings=len(rims),
                rim_ratio=r(np.percentile([x["rim_ratio"] for x in rims], [25, 50, 75])) if len(rims) >= 3 else None,
                rim_px1080=r(np.percentile([x["rim_px1080"] for x in rims], [25, 50, 75])) if len(rims) >= 3 else None)


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(5)
    res = {}
    for key, region, mode, ground in REGIONS:
        r = analyse(key, region, mode, ground, rng)
        if r is None:
            continue
        cells = r.pop("cells")
        r["all"] = summarise(cells)
        r["interior"] = summarise(cells, interior_only=True)
        res[f"{key}/{region}"] = r
        a = r["all"]
        print(f"{key}/{region}: n={a['n']} eqd%={a['eqd_pct_robe_width']} aspect={a['aspect_hw']} shapes={a['shapes']} rim={a['rim_ratio']}")
    json.dump(res, open(os.path.join(OUT, "cells.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
