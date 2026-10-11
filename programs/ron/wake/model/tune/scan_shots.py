#!/usr/bin/env python3
"""Find shots and their motion level in a video (thumbnail scan).
Usage: scan_shots.py VIDEO  -> prints shots: start/end s, mean motion, cut flags."""
import sys
import numpy as np
import imageio_ffmpeg

path = sys.argv[1]
W, H = 160, 90
gen = imageio_ffmpeg.read_frames(path, output_params=["-vf", f"scale={W}:{H}"])
meta = next(gen)
fps = meta["fps"]
prev, diffs = None, []
for buf in gen:
    f = np.frombuffer(buf, np.uint8).reshape(H, W, 3).astype(np.float32).mean(-1)
    diffs.append(0.0 if prev is None else float(np.abs(f - prev).mean()))
    prev = f
d = np.array(diffs)
cuts = np.flatnonzero(d > max(12.0, 6 * np.median(d))).tolist()
bounds = [0] + cuts + [len(d)]
print(f"{path}: {len(d)} frames @ {fps} fps")
for a, b in zip(bounds, bounds[1:]):
    if b - a < fps * 3:
        continue
    seg = d[a + 1:b]
    print(f"  {a / fps:7.1f}-{b / fps:7.1f}s  len {(b - a) / fps:5.1f}s  motion {seg.mean():5.2f}  p90 {np.percentile(seg, 90):5.2f}")
