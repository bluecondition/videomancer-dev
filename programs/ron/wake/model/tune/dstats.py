#!/usr/bin/env python3
"""Raw cell difference d (FLOOR 0, CHROMA 512) for cells with vs without GT motion."""
import numpy as np
import tune
import wake_model as wm
from dataclasses import replace

ctl = replace(tune.DEFAULTS, floor=0)
print(f"{'clip':12s} {'moving cells: p25  p50  p75  p90':>36s}   {'still cells: p50  p99  max':>28s}  frac_moving")
for name in tune.CLIPS:
    acc, frames = tune.load(name)
    now, near = tune.gt_masks(frames)
    mv, st = [], []
    for t, key, raw, m, out in tune.run(acc, lambda t: ctl, tune.GW, tune.GH):
        if t < 2:
            continue
        d = out["d"]
        # cell motion fraction from the GT mask (160x90 -> 32x18 cells of 5x5)
        cm = now[t].reshape(18, 5, 32, 5).mean(axis=(1, 3))
        mv += list(d[cm > 0.25]); st += list(d[(cm == 0) & ~near[t].reshape(18, 5, 32, 5).any(axis=(1, 3))])
    mv, st = np.array(mv), np.array(st)
    pm = np.percentile(mv, [25, 50, 75, 90]) if len(mv) else [0] * 4
    ps = np.percentile(st, [50, 99]) if len(st) else [0, 0]
    print(f"{name:12s} {pm[0]:14.1f} {pm[1]:4.1f} {pm[2]:4.1f} {pm[3]:4.1f}   {ps[0]:20.1f} {ps[1]:4.1f} {st.max() if len(st) else 0:4d}  {len(mv)/(len(mv)+len(st)+1e-9):.2f}")
