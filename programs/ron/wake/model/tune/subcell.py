#!/usr/bin/env python3
"""Does a 2x2 luma sub-cell split help? d = sum over 4 sub-cells |dY_sub| / 2 + |dU|+|dV| (cell),
1/4-LSB precision, alpha 1/4 reference; same detection metric as variants.py."""
import numpy as np
import tune
import wake_model as wm

g1 = wm.Geometry(tune.AW, tune.AH, 32, wm.ROWS)
g2 = wm.Geometry(tune.AW, tune.AH, 64, 2 * wm.ROWS)
data, st_all = {}, {"cell": [], "sub": []}
mv_all = {"cell": {}, "sub": {}}
for name, (video, start, dur) in tune.CLIPS.items():
    acc, frames = tune.load(name)
    now, near = tune.gt_masks(frames)
    cm = now.reshape(len(now), 18, 5, 32, 5).mean(axis=(2, 4))
    still = ~near.reshape(len(near), 18, 5, 32, 5).any(axis=(2, 4))
    ysub = []
    for rgb in tune.decode(video, start, dur, tune.AW, tune.AH):
        yuv = wm.rgb_to_yuv10(rgb / 255.0, "709")
        ysub.append(wm.accumulate((yuv[..., 0] >> 2).astype(np.int64), g2) * g2.recip / 65536.0)
    ysub = np.round(np.array(ysub) * 4) / 4
    mean = np.round(acc * g1.recip / 65536.0 * 4) / 4
    n = min(len(ysub), len(mean))
    for kind in ("cell", "sub"):
        refc, refs = mean[0].copy(), ysub[0].copy()
        mv, st = [], []
        for t in range(1, n):
            dc = np.abs(mean[t] - refc)
            if kind == "cell":
                d = dc.sum(-1)
            else:
                ds = np.abs(ysub[t] - refs).reshape(18, 2, 32, 2).sum(axis=(1, 3)) / 2
                d = ds + dc[..., 1] + dc[..., 2]
            refc = np.round((refc + 0.25 * (mean[t] - refc)) * 4) / 4
            refs = np.round((refs + 0.25 * (ysub[t] - refs)) * 4) / 4
            if t >= 2:
                mv += list(d[cm[t] > 0.25]); st += list(d[still[t] & (cm[t] == 0)])
        mv_all[kind][name] = np.array(mv); st_all[kind] += st
for kind in ("cell", "sub"):
    thr = np.percentile(st_all[kind], 99.5)
    print(f"{kind:5s} thr {thr:5.2f} " + " ".join(f"{n[:9]}={(v > thr).mean():.2f}" for n, v in mv_all[kind].items()))
