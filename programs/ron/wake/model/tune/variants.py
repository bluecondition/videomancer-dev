#!/usr/bin/env python3
"""Separability of moving vs still cells for engine variants (float exploration).
d = |dY| + |dU| + |dV| (CHROMA 1x) in 8-bit LSB. One global threshold = still-cell p99.5
pooled over all clips (one FLOOR must serve all content); report the detection rate of
moving cells per clip."""
import numpy as np
import tune
import wake_model as wm

geo = wm.Geometry(tune.AW, tune.AH, 32, wm.ROWS)
VARIANTS = {"V0 int, prev frame": (0, 1.0), "V1 1/4 LSB, prev": (2, 1.0),
            "V2 1/4, a=1/2": (2, 0.5), "V3 1/4, a=1/4": (2, 0.25), "V4 1/4, a=1/8": (2, 0.125)}
data = {}
for name in tune.CLIPS:
    acc, frames = tune.load(name)
    now, near = tune.gt_masks(frames)
    cm = now.reshape(len(now), 18, 5, 32, 5).mean(axis=(2, 4))
    still = ~near.reshape(len(near), 18, 5, 32, 5).any(axis=(2, 4))
    mean = acc * geo.recip / 65536.0                       # float cell means, 8-bit units
    data[name] = (mean, cm, still)
res = {}
for vname, (frac, a) in VARIANTS.items():
    mv_all, st_all = {}, []
    for name, (mean, cm, still) in data.items():
        q = np.round(mean * 4) / 4 if frac else np.round(mean)
        ref = q[0].copy()
        mv, st = [], []
        for t in range(1, len(q)):
            d = np.abs(q[t] - ref).sum(-1)
            ref = ref + a * (q[t] - ref)
            if frac:
                ref = np.round(ref * 4) / 4
            if t >= 2:
                mv += list(d[cm[t] > 0.25]); st += list(d[still[t] & (cm[t] == 0)])
        mv_all[name] = np.array(mv); st_all += st
    thr = np.percentile(st_all, 99.5)
    res[vname] = (thr, {n: (v > thr).mean() for n, v in mv_all.items()})
print(f"{'variant':22s} {'thr':>5s} " + " ".join(f"{n[:9]:>9s}" for n in tune.CLIPS))
for vname, (thr, det) in res.items():
    print(f"{vname:22s} {thr:5.2f} " + " ".join(f"{det[n]:9.2f}" for n in tune.CLIPS))
