#!/usr/bin/env python3
"""Golden check for motion_key: RTL (tb_motion_key.vhd) vs model/wake_model.py.

  golden.py stim  DIR   write DIR/stim.txt (NTSC fields, 10-bit YUV) + DIR/expected.npz
  golden.py check DIR   compare DIR/tb_out.txt with DIR/expected.npz (bit-exact)

Hardware warm-up: top field 0 only measures geometry, top field 2 seeds. So
model update u (run from field 2 on) = RTL update counter u in the log.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "model"))
import wake_model as wm  # noqa: E402

N_FIELDS = 20
PARAMS = dict(wake=528, floor=154, chroma=700)   # must match the TB generics; wake 528 -> e16 % 16 == 0 (mantissa 32)
import os  # noqa: E402
STEADY = int(os.environ.get("GOLDEN_STEADY", "0"))   # 1: STEADY on, scene cut at field ~12
T0 = (wm.T_CUT - 12 / 59.94) if STEADY else 0.5


FMT_TID = {"ntsc": None, "hd": 2}    # "hd": NTSC raster, 1080i59.94 timing ID (C=32, HD field logic)


def stim(d):
    tid, W, H, il, hz, cols, mx = wm.FORMATS["ntsc"]
    if __import__("os").environ.get("GOLDEN_FMT") == "hd":
        tid, cols = 2, 32
    syn = wm.Synth(W, H, mx)
    fields = []
    with open(d / "stim.txt", "w") as f:
        for k in range(N_FIELDS):
            rgb, _, _ = syn.frame(T0 + k / hz)
            yuv = wm.rgb_to_yuv10(rgb[k & 1::2], mx).astype(np.int64)
            fields.append(yuv)
            packed = (yuv[..., 0] << 20) | (yuv[..., 1] << 10) | yuv[..., 2]
            for row in packed:
                f.write(" ".join(f"{p:08x}" for p in row) + "\n")
    eng = wm.MotionKey3(tid, cols, one_field=True)
    ctl = wm.Controls(**PARAMS, steady=STEADY)
    ry, ru, rv, M, dsum, cuts = [], [], [], [], [], []
    for k in range(2, N_FIELDS):
        out = eng.update(fields[k], ctl, field=k & 1)
        if not out["skipped"]:
            ry.append(eng.ref_y.copy()); ru.append(eng.ref_u.copy()); rv.append(eng.ref_v.copy())
            M.append(out["M"]); dsum.append(out["d_sum"]); cuts.append(out.get("cut", False))
    np.savez(d / "expected.npz", ref_y=np.array(ry), ref_u=np.array(ru), ref_v=np.array(rv),
             M=np.array(M), d_sum=np.array(dsum), cut=np.array(cuts))
    print(f"model cut updates: {[i for i, c in enumerate(cuts) if c]}")
    print(f"stim: {N_FIELDS} fields {H // 2}x{W}; {len(M)} expected updates; geometry {eng.geo.describe()}")


def check(d):
    exp = np.load(d / "expected.npz")
    R, C = exp["M"].shape[1:3]
    hw_m, hw_r, hw_e = {}, {}, {}
    for line in open(d / "tb_out.txt"):
        k, u, a, *v = line.split()
        u, a = int(u), int(a)
        if k == "M":
            hw_m.setdefault(u, {})[a] = int(v[0])
        elif k == "R":
            hw_r.setdefault(u, {})[a] = int(v[0])
        else:
            hw_e[u] = a
    n = min(len(exp["M"]), len(hw_e))
    bad = 0
    for u in range(n):
        m = np.array([[hw_m[u].get(r * 32 + c, -1) for c in range(C)] for r in range(R)])
        ref = np.array([[[hw_r[u].get((r * 32 + c) * 6 + v, -1) for v in range(6)]
                         for c in range(C)] for r in range(R)])
        ey, eu, ev = exp["ref_y"][u], exp["ref_u"][u], exp["ref_v"][u]
        want = np.zeros((R, C, 6), np.int64)
        for v in range(4):
            want[..., v] = ey[(v >> 1)::2, (v & 1)::2]
        want[..., 4], want[..., 5] = eu, ev
        dm = int((m != exp["M"][u]).sum())
        dc = int((ref != want).sum())
        de = hw_e[u] != exp["d_sum"][u]
        bad += dm + dc + de
        print(f"update {u:2d}: M mismatches {dm:3d}  ref mismatches {dc:4d}  "
              f"energy rtl {hw_e[u]:6d} model {exp['d_sum'][u]:6d}  "
              f"M>0 cells {int((exp['M'][u] > 0).sum()):3d}")
        if dm and u < 3:
            r, c = np.argwhere(m != exp["M"][u])[0]
            print(f"    first M diff at row {r} col {c}: rtl {m[r, c]} model {exp['M'][u][r, c]}")
        if dc and u < 3:
            r, c, v = np.argwhere(ref != want)[0]
            print(f"    first ref diff at row {r} col {c} v {v}: rtl {ref[r, c, v]} model {want[r, c, v]}")
    print(f"{n} updates compared: {'PASS (bit-exact)' if bad == 0 and n else 'FAIL'}")
    return 0 if bad == 0 and n else 1


if __name__ == "__main__":
    mode, d = sys.argv[1], Path(sys.argv[2])
    d.mkdir(parents=True, exist_ok=True)
    sys.exit(stim(d) if mode == "stim" else check(d))
