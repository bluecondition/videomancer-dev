"""Phase 1 measurements: palettes, coverage, luma, band energy, two golds, field texture.

usage: python measure.py            -> results/measure.json
Run from this directory with the giltlab venv (numpy, scipy, scikit-image).
"""
import json
import os

import numpy as np
from scipy import ndimage as ndi

import lab

OUT = os.path.join(lab.HERE, "results")
PCTS = [2, 10, 50, 90, 98]


def pct(v):
    if v.size == 0:
        return None
    return [int(round(float(x))) for x in np.percentile(v, PCTS)]


def gold_stats(Lab3, Y, Cb, Cr, mask):
    """Median colour + spread of the gold-family pixels inside mask."""
    if mask.sum() < 500:
        return None
    L, C, h = lab.lch(Lab3)
    sel = mask & (lab.classify(Lab3) == 0)
    if sel.sum() < 500:
        return None
    med = [float(np.median(Lab3[..., i][sel])) for i in range(3)]
    rgb = lab.lab_to_rgb8(*med)
    return dict(n_frac=float(sel.sum() / mask.sum()),
                median_hex=lab.hexrgb(rgb), median_lab=[round(v, 1) for v in med],
                Lstar_pct=pct(L[sel]), Cstar_pct=pct(C[sel]), hue_pct=pct(h[sel]),
                y10_pct=pct(Y[sel]), cb10_median=int(np.median(Cb[sel])),
                cr10_median=int(np.median(Cr[sel])))


def field_texture(Y, Lab3, field):
    """Grain amplitude and fleck statistics on the FIELD zone, at 1080-equivalent
    (all sizes in px at that scale)."""
    if field.sum() < 5000:
        return None
    er = ndi.binary_erosion(field, iterations=12)
    fine = Y - ndi.gaussian_filter(Y, 1.0)                       # 1-2 px texture
    blotch = ndi.gaussian_filter(Y, 2.0) - ndi.gaussian_filter(Y, 8.0)   # 4-16 px
    bg = ndi.gaussian_filter(Y, 5.0)
    res = dict(grain_std_y10=float(fine[er].std()),
               grain_p98_abs_y10=float(np.percentile(np.abs(fine[er]), 98)),
               blotch_std_y10=float(blotch[er].std()),
               field_y10_pct=pct(Y[er]))
    L, C, h = lab.lch(Lab3)
    for thr in (40, 80):
        m = er & ((Y - bg) > thr)
        lbl, n = ndi.label(m)
        sizes = np.bincount(lbl.ravel())[1:] if n else np.array([])
        sizes = sizes[sizes >= 2]
        eqd = np.sqrt(4 * sizes / np.pi) if sizes.size else np.array([])
        area = float(er.sum())
        per_10k = sizes.size / area * 1e4 if area else 0     # flecks per 100x100 px
        silver = float(np.mean(C[m] < 15)) if m.any() else None
        res[f"flecks_gt{thr}"] = dict(
            pixel_frac=round(float(m.sum() / area), 4), count_per_100x100px=round(per_10k, 2),
            eqdiam_px_p50=round(float(np.median(eqd)), 2) if eqd.size else None,
            eqdiam_px_p90=round(float(np.percentile(eqd, 90)), 2) if eqd.size else None,
            silver_frac=silver)
    return res


def painting(key):
    rgb, box, full = lab.load(key, long_side=2048)
    H, W = rgb.shape[:2]
    Lab3 = lab.to_lab(rgb)
    fam = lab.classify(Lab3)
    Y, Cb, Cr = lab.ycbcr709_10(rgb)
    labels, names = lab.label_map(key, rgb.shape, box, full)
    groups = lab.group_masks(labels, names)
    subs = lab.label_masks(labels, names)
    flat = Lab3.reshape(-1, 3)

    res = dict(size_analysed=[W, H], canvas_box=list(box), full_size=list(full))
    res["coverage"] = lab.coverage(fam)
    res["luma_pct"] = pct(Y.ravel())
    C, w = lab.kmeans(flat, 10, seed=1)
    res["palette_k10"] = lab.palette_entries(C, w)
    # texture-scale measures at 1080-equivalent
    rgb8, box8, _ = lab.load(key, short_side=1080)
    Y8 = lab.ycbcr709_10(rgb8)[0]
    Lab8 = lab.to_lab(rgb8)
    lbl8, names8 = lab.label_map(key, rgb8.shape, box8, full)
    groups8 = lab.group_masks(lbl8, names8)
    subs8 = lab.label_masks(lbl8, names8)
    res["size_1080eq"] = [rgb8.shape[1], rgb8.shape[0]]
    res["band_energy_canvas"] = lab.band_energy(Y8, {"canvas": np.ones_like(Y8, bool)})

    if key in lab.ZONED:
        res["zone_area"] = {g: float(m.mean()) for g, m in groups.items()}
        res["zone_area_sub"] = {s: float(m.mean()) for s, m in subs.items()}
        res["coverage_by_zone"] = {g: lab.coverage(fam, m) for g, m in groups.items()}
        res["coverage_by_sub"] = {s: lab.coverage(fam, m) for s, m in subs.items()}
        res["luma_pct_by_zone"] = {g: pct(Y[m]) for g, m in groups.items()}
        res["palette_by_zone"] = {}
        for g, m in list(groups.items()) + [(s, subs[s]) for s in subs if s.startswith("robe_")]:
            if m.sum() < 2000:
                continue
            k = 4 if g in ("flesh", "hair") else 6
            C, w = lab.kmeans(Lab3[m], k, seed=2)
            res["palette_by_zone"][g] = lab.palette_entries(C, w)
        res["band_energy_by_zone"] = lab.band_energy(Y8, {**groups8, **{s: m for s, m in subs8.items() if s.startswith("robe_")}})
        res["gold_field"] = gold_stats(Lab3, Y, Cb, Cr, groups.get("field", np.zeros_like(Y, bool)))
        res["gold_robe"] = gold_stats(Lab3, Y, Cb, Cr, groups.get("robe", np.zeros_like(Y, bool)))
        res["field_texture"] = field_texture(Y8, Lab8, groups8.get("field", np.zeros_like(Y8, bool)))
        # flesh: overall + flush (most saturated red-ish flesh pixels: lips, cheeks)
        fm = groups.get("flesh")
        if fm is not None and fm.sum() > 500:
            L_, C_, h_ = lab.lch(Lab3)
            sel = fm & (L_ > 35)
            med = [float(np.median(Lab3[..., i][sel])) for i in range(3)]
            flush = sel & (C_ > np.percentile(C_[sel], 97)) & ((h_ < 60) | (h_ > 340))
            fl = [float(np.median(Lab3[..., i][flush])) for i in range(3)] if flush.sum() > 30 else None
            res["flesh"] = dict(
                median_hex=lab.hexrgb(lab.lab_to_rgb8(*med)), median_lab=[round(v, 1) for v in med],
                Cstar_pct=pct(C_[sel]), y10_pct=pct(Y[sel]),
                cb10_median=int(np.median(Cb[sel])), cr10_median=int(np.median(Cr[sel])),
                flush_hex=lab.hexrgb(lab.lab_to_rgb8(*fl)) if fl else None,
                flush_lab=[round(v, 1) for v in fl] if fl else None)
    return res, Lab3, fam, groups


def jewels(collected):
    """Pool the jewel-family pixels (red/violet/blue) and the high-chroma oranges of
    the ornament zones, cluster them, and report the enamel set."""
    out = {}
    X = np.concatenate([c["jewel"] for c in collected.values() if c["jewel"].size])
    C, w = lab.kmeans(X, 8, seed=3)
    out["jewel_k8"] = lab.palette_entries(C, w)
    O = np.concatenate([c["orange"] for c in collected.values() if c["orange"].size])
    C, w = lab.kmeans(O, 3, seed=4)
    out["orange_enamel_k3"] = lab.palette_entries(C, w)
    G = np.concatenate([c["green"] for c in collected.values() if c["green"].size])
    C, w = lab.kmeans(G, 4, seed=5)
    out["green_k4"] = lab.palette_entries(C, w)
    out["sources"] = {k: dict(jewel_px=int(v["jewel"].shape[0]), orange_px=int(v["orange"].shape[0]),
                              green_px=int(v["green"].shape[0])) for k, v in collected.items()}
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    results, collected = {}, {}
    for key in lab.PAINTINGS:
        r, Lab3, fam, groups = painting(key)
        results[key] = r
        print(key, "coverage", {k: round(v * 100, 1) for k, v in r["coverage"].items()})
        # where to collect enamel colours: ornament + meadow zones of the zoned
        # paintings; the whole canvas of Hope II and Water Serpents I.
        if key in lab.ZONED:
            m = groups.get("robe", np.zeros(fam.shape, bool)) | groups.get("meadow", np.zeros(fam.shape, bool))
        elif key in ("hope2", "water_serpents1", "danae"):
            m = np.ones(fam.shape, bool)
        else:
            continue
        L, C, h = lab.lch(Lab3)
        jew = m & np.isin(fam, [4, 5, 6])
        orange = m & (fam == 0) & (C > 50) & (h < 70)
        green = m & (fam == 3)
        rng = np.random.default_rng(7)
        def take(mask, n=40000):
            v = Lab3[mask]
            return v[rng.choice(len(v), min(n, len(v)), replace=False)] if len(v) else v
        collected[key] = dict(jewel=take(jew), orange=take(orange), green=take(green))
    results["_enamels"] = jewels(collected)
    json.dump(results, open(os.path.join(OUT, "measure.json"), "w"), indent=1)
    print("wrote results/measure.json")


if __name__ == "__main__":
    main()
