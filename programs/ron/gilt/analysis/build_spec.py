"""Assemble style_constants.json (the single source of truth for the model and the
VHDL constants package) and results/tables.md (the tables quoted in STYLE_SPEC.md)
from results/measure.json, results/cells.json and manual_measurements.json.

usage: python build_spec.py
"""
import json
import os

import numpy as np

import lab

R = json.load(open(os.path.join(lab.HERE, "results", "measure.json")))
CELLS = json.load(open(os.path.join(lab.HERE, "results", "cells.json")))
MAN = json.load(open(os.path.join(lab.HERE, "manual_measurements.json")))


def colour(hexs=None, labv=None, note=None):
    """One palette entry in every representation we use. sRGB is the source of truth."""
    if labv is not None:
        rgb = lab.lab_to_rgb8(*labv)
    else:
        rgb = tuple(int(hexs[i:i + 2], 16) for i in (1, 3, 5))
        labv = lab.to_lab(np.array([[rgb]], np.uint8))[0, 0].tolist()
    Y, Cb, Cr = lab.ycbcr709_10(np.array(rgb, np.float32))
    e = dict(hex=lab.hexrgb(rgb), lab=[round(float(v), 1) for v in labv],
             y10=int(round(float(Y))), cb10=int(round(float(Cb))), cr10=int(round(float(Cr))))
    if note:
        e["note"] = note
    return e


def pick(entries, pred):
    c = [e for e in entries if pred(e)]
    return max(c, key=lambda e: e["weight"]) if c else None


def chroma(e):
    return float(np.hypot(e["lab"][1], e["lab"][2]))


def main():
    K, A, F = R["kiss"], R["adele1"], R["stoclet_fulfillment"]
    pz = lambda r, z: r["palette_by_zone"].get(z, [])

    # --- ink and silver from the ornament / hair palettes -------------------------
    ink_c = [e for r in (K, A) for z in ("robe", "hair") for e in pz(r, z) if e["lab"][0] < 25]
    ink = min(ink_c, key=lambda e: e["lab"][0])
    silver = pick(pz(K, "robe_rect"), lambda e: chroma(e) < 20 and e["lab"][0] > 65)
    silver_cool = pick(pz(F, "robe_checker"), lambda e: chroma(e) < 8 and 60 < e["lab"][0] < 80)
    white = pick(pz(F, "robe_checker"), lambda e: chroma(e) < 5 and e["lab"][0] > 85)
    umber = pick(pz(K, "robe_rect"), lambda e: 20 <= e["lab"][0] < 40)

    pal = {
        "ground_gold_kiss": colour(K["gold_field"]["median_hex"], note="Kiss field (brass leaf) median"),
        "figure_gold_kiss": colour(K["gold_robe"]["median_hex"], note="Kiss robe gold median"),
        "ground_gold_adele": colour(A["gold_field"]["median_hex"], note="Adele field (orange gold) median"),
        "figure_gold_adele": colour(A["gold_robe"]["median_hex"], note="Adele ornament gold median"),
        "cream_stoclet": colour(F["gold_field"]["median_hex"], note="Stoclet cartoon ground (paper + gold paint)"),
        "ink": colour(ink["hex"], note="darkest ornament/hair cluster"),
        "umber": colour(umber["hex"], note="Kiss dark bar umber") if umber else None,
        "silver_warm": colour(silver["hex"], note="Kiss silver tablet cluster (aged, warm)"),
        "silver_cool": colour(silver_cool["hex"], note="Fulfillment checker silver"),
        "white": colour(white["hex"], note="Fulfillment checker white"),
        "flesh_kiss": colour(K["flesh"]["median_hex"], note="Kiss flesh median"),
        "flesh_adele": colour(A["flesh"]["median_hex"], note="Adele flesh median"),
        "flesh_stoclet": colour(F["flesh"]["median_hex"], note="Fulfillment flesh median"),
        "flush": colour(K["flesh"]["flush_hex"], note="Kiss lips/cheek (top 3 % chroma in flesh)"),
        "jewels": [dict(colour(e["hex"]), weight=e["weight"]) for e in R["_enamels"]["jewel_k8"]],
        "orange_enamel": [dict(colour(e["hex"]), weight=e["weight"]) for e in R["_enamels"]["orange_enamel_k3"]],
        "meadow_greens": [dict(colour(e["hex"]), weight=e["weight"]) for e in R["_enamels"]["green_k4"]],
    }
    # LEAF knob axis (spec 5.6): silver -> olive brass -> warm gold -> orange gold
    pal["leaf_axis"] = [dict(pal["silver_cool"], stop="silver / platinum"),
                        dict(pal["ground_gold_kiss"], stop="olive brass"),
                        dict(pal["figure_gold_kiss"], stop="warm gold"),
                        dict(pal["ground_gold_adele"], stop="orange gold")]

    gold = {k: {"field": R[k]["gold_field"], "robe": R[k]["gold_robe"]} for k in lab.ZONED}
    two_golds = {}
    for k in ("kiss", "adele1"):
        f, r = R[k]["gold_field"], R[k]["gold_robe"]
        two_golds[k] = dict(dY10_median=r["y10_pct"][2] - f["y10_pct"][2],
                            dCb10=r["cb10_median"] - f["cb10_median"],
                            dCr10=r["cr10_median"] - f["cr10_median"],
                            dCstar_median=r["Cstar_pct"][2] - f["Cstar_pct"][2],
                            dHue_median=r["hue_pct"][2] - f["hue_pct"][2],
                            field_y10_p10_p90=[f["y10_pct"][1], f["y10_pct"][3]],
                            robe_y10_p10_p90=[r["y10_pct"][1], r["y10_pct"][3]])

    cov = {k: {f: round(v, 4) for f, v in R[k]["coverage"].items()} for k in lab.PAINTINGS}
    cov_zone = {k: {z: {f: round(v, 4) for f, v in c.items()} for z, c in R[k]["coverage_by_zone"].items()}
                for k in lab.ZONED}
    luma = {k: R[k]["luma_pct"] for k in lab.PAINTINGS}
    luma_zone = {k: R[k]["luma_pct_by_zone"] for k in lab.ZONED}
    be = {k: R[k]["band_energy_by_zone"] for k in lab.ZONED}
    ratios = {}
    for k in lab.ZONED:
        m = be[k]["motif"]
        ratios[k] = {z: round(m[z] / m["field"], 2) for z in m if m.get(z) and z != "field"}

    out = {
        "_meta": dict(
            generated_by="programs/ron/gilt/analysis/build_spec.py",
            phase=1, status="zone polygons (analysis/zones.json) approved by Ron 2026-10-10",
            colour_convention="sRGB is the source of truth; lab = CIELAB D65; y10/cb10/cr10 = BT.709 10-bit legal range (reporting only, hardware constants are generated in Phase 4)",
            scale_convention="px1080 = px when the painting's short side is shown as 1080 lines; pct_robe_width = % of the robe zone's horizontal extent",
            references="refs/SOURCES.md"),
        "palette": pal,
        "two_golds": two_golds,
        "gold_by_zone": gold,
        "coverage": cov,
        "coverage_by_zone": cov_zone,
        "luma_y10_p2_p10_p50_p90_p98": luma,
        "luma_by_zone": luma_zone,
        "band_energy_px1080": dict(bands=lab.BANDS, by_zone=be, motif_ratio_to_field=ratios),
        "field_texture_px1080": {"nebula_kiss": K["field_texture"], "leaf_adele": A["field_texture"],
                                 "stoclet_cartoon": F["field_texture"]},
        "cells": CELLS,
        "manual": MAN,
        "zone_area": {k: R[k]["zone_area"] for k in lab.ZONED},
    }
    json.dump(out, open(os.path.join(lab.GILT, "style_constants.json"), "w"), indent=1)

    # ---------------- tables.md ----------------
    T = []
    T.append("## Palette anchors\n\n| role | hex | L* a* b* | Y10 | Cb10 | Cr10 |\n|---|---|---|---|---|---|")
    for k, e in pal.items():
        if isinstance(e, dict) and "hex" in e:
            T.append(f"| {k} | {e['hex']} | {e['lab'][0]:.0f} {e['lab'][1]:.0f} {e['lab'][2]:.0f} | {e['y10']} | {e['cb10']} | {e['cr10']} |")
    for grp in ("jewels", "orange_enamel", "meadow_greens"):
        T.append(f"\n**{grp}** (pooled k-means, weight = share of that pool)\n\n| hex | L* a* b* | Y10 | Cb10 | Cr10 | weight |\n|---|---|---|---|---|---|")
        for e in pal[grp]:
            T.append(f"| {e['hex']} | {e['lab'][0]:.0f} {e['lab'][1]:.0f} {e['lab'][2]:.0f} | {e['y10']} | {e['cb10']} | {e['cr10']} | {e['weight']:.2f} |")
    T.append("\n## Coverage (% of canvas)\n\n| painting | gold | black | neutral | green | red | violet | blue |\n|---|---|---|---|---|---|---|---|")
    for k in lab.PAINTINGS:
        c = cov[k]
        T.append(f"| {k} | " + " | ".join(f"{100*c[f]:.1f}" for f in lab.FAMILIES) + " |")
    T.append("\n## Coverage by zone (% of zone)\n\n| painting / zone | area % | gold | black | neutral | green | red | violet | blue |\n|---|---|---|---|---|---|---|---|---|")
    for k in lab.ZONED:
        for z, c in cov_zone[k].items():
            T.append(f"| {k} / {z} | {100*R[k]['zone_area'][z]:.1f} | " + " | ".join(f"{100*c[f]:.1f}" for f in lab.FAMILIES) + " |")
    T.append("\n## Luma Y10 percentiles 2/10/50/90/98\n\n| painting | canvas | field | robe | flesh |\n|---|---|---|---|---|")
    for k in lab.PAINTINGS:
        lz = luma_zone.get(k, {})
        T.append(f"| {k} | {luma[k]} | {lz.get('field', '')} | {lz.get('robe', '')} | {lz.get('flesh', '')} |")
    T.append("\n## Band energy, mean |DoG| in Y10 codes at 1080-equivalent\n\n| painting / zone | grain 1-3 px | detail 3-10 px | motif 10-40 px | motif / field |\n|---|---|---|---|---|")
    for k in lab.ZONED:
        for z in be[k]["motif"]:
            T.append(f"| {k} / {z} | {be[k]['grain'][z]} | {be[k]['detail'][z]} | {be[k]['motif'][z]} | {ratios[k].get(z, '1.00' if z == 'field' else '')} |")
    T.append("\n## Two golds (robe minus field)\n\n| painting | dY10 | dCb10 | dCr10 | dC* | dhue | field Y10 p10-p90 | robe Y10 p10-p90 |\n|---|---|---|---|---|---|---|---|")
    for k, d in two_golds.items():
        T.append(f"| {k} | {d['dY10_median']:+d} | {d['dCb10']:+d} | {d['dCr10']:+d} | {d['dCstar_median']:+d} | {d['dHue_median']:+d} | {d['field_y10_p10_p90']} | {d['robe_y10_p10_p90']} |")
    T.append("\n## Field texture at 1080-equivalent\n\n| painting | grain std Y10 (1-2 px) | blotch std (4-16 px) | flecks >80 codes: px % | per 100x100 px | eq. diam p50/p90 px | silver share |\n|---|---|---|---|---|---|---|")
    for k in lab.ZONED:
        t = R[k]["field_texture"]
        f8 = t["flecks_gt80"]
        T.append(f"| {k} | {t['grain_std_y10']:.0f} | {t['blotch_std_y10']:.0f} | {100*f8['pixel_frac']:.1f} | {f8['count_per_100x100px']:.0f} | {f8['eqdiam_px_p50']}/{f8['eqdiam_px_p90']} | {f8['silver_frac']:.2f} |")
    T.append("\n## Cells (contrasting cells, automatic)\n\n| region | n | eq. diam % robe width p10/25/50/75/90/99 | eq. diam px1080 p10/50/90 | long side % p50/p90 | aspect h/w p10/50/90 | ln-normal sigma | shapes |\n|---|---|---|---|---|---|---|---|")
    for k, v in CELLS.items():
        a = v["all"]
        e = a["eqd_px1080"]
        T.append(f"| {k} | {a['n']} | {a['eqd_pct_robe_width']} | {e[0]}/{e[2]}/{e[4]} | {a['long_side_pct_robe_width'][2]}/{a['long_side_pct_robe_width'][4]} | {a['aspect_hw']} | {a['lognormal_eqd']['sigma_ln']} | {a['shapes']} |")
    open(os.path.join(lab.HERE, "results", "tables.md"), "w").write("\n".join(T) + "\n")
    print("wrote style_constants.json and results/tables.md")


if __name__ == "__main__":
    main()
