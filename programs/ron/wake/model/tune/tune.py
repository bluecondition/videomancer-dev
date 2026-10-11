#!/usr/bin/env python3
"""WAKE tuning harness: real footage -> cached cell sums -> fast control sweeps.

  tune.py prep                 decode clips once (cell sums + 640x360 frames) to CACHE
  tune.py sweep PARAM [clips]  metrics vs PARAM (others at defaults) -> table + PNG
  tune.py tour NAME            labelled video of one control sweeping over a clip

The engine is model/wake_model.py (bit-exact with the RTL grid). Analysis runs on the
960x540 frame (32x18 cells, 1080p24 timing); the matte renders at 160x90 for metrics
and 640x360 for tours. Ground truth motion = per-pixel |L_t - L_t-1| > 4 at 160x90.
"""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import wake_model as wm  # noqa: E402

CACHE = Path.home() / ".cache" / "wake_tune"
TOS = str(Path.home() / ".cache" / "wake_footage" / "tears_of_steel_720p.mov")
CLIPS = {                       # name: (video, start s, duration s)
    "city_still":  (TOS, 70.5, 3.5),
    "seated":      (TOS, 135.5, 4.0),
    "room_walk":   (TOS, 183.5, 6.0),
    "bridge_talk": (TOS, 26.0, 8.0),
    "trio":        (TOS, 239.5, 8.0),
    "action":      (TOS, 445.5, 8.0),
    "bridge_walk": (TOS, 258.5, 8.0),
}
CUT_CLIPS = {                   # longer spans with real scene cuts (STEADY tuning)
    "cuts_a": (TOS, 150.0, 26.0),
    "cuts_b": (TOS, 440.0, 25.0),
}
AW, AH = 960, 540            # analysis raster
TID, FPS = 3, 24.0           # 1080p24 timing (footage is 24 fps)
GW, GH = 160, 90             # metrics raster
DEFAULTS = wm.Controls(floor=256)   # v3: f4 = FLOOR >> 4 = 16 quarter steps (25%)


def decode(video, start, dur, w, h):
    import imageio_ffmpeg
    vf = f"scale={w}:-2,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2"
    gen = imageio_ffmpeg.read_frames(video, input_params=["-ss", f"{start}"],
                                     output_params=["-t", f"{dur}", "-vf", vf])
    next(gen)
    for buf in gen:
        yield np.frombuffer(buf, np.uint8).reshape(h, w, 3)


def prep(names=None):
    CACHE.mkdir(parents=True, exist_ok=True)
    geo = wm.Geometry(AW, AH, 32, wm.ROWS)
    allc = {**CLIPS, **CUT_CLIPS}
    for name, (video, start, dur) in allc.items():
        if names and name not in names:
            continue
        accs, frames, a3y, a3u, a3v = [], [], [], [], []
        k3 = wm.MotionKey3(TID, 32)
        k3.set_geometry(AW, AH)
        for rgb in decode(video, start, dur, AW, AH):
            yuv = wm.rgb_to_yuv10(rgb / 255.0, "709")
            accs.append(np.stack([wm.accumulate((yuv[..., c] >> 2).astype(np.int64), geo)
                                  for c in range(3)], -1))
            y3, u3, v3 = wm.accumulate3(yuv, k3.geo)
            a3y.append(y3); a3u.append(u3); a3v.append(v3)
            small = wm.box_resize(rgb.astype(np.float32), 360, 640)
            frames.append(np.clip(np.rint(small), 0, 255).astype(np.uint8))
        np.savez(CACHE / f"{name}.npz", acc=np.array(accs), frames=np.array(frames),
                 a3y=np.array(a3y), a3u=np.array(a3u), a3v=np.array(a3v))
        print(f"{name}: {len(accs)} frames")


ENGINE = "v3"


def load(name):
    z = np.load(CACHE / f"{name}.npz")
    if ENGINE == "v3":
        return list(zip(z["a3y"], z["a3u"], z["a3v"])), z["frames"]
    return z["acc"], z["frames"]


def gt_masks(frames):
    """moving-now and moved-within-1-s masks at GW x GH."""
    L = np.array([wm.box_resize(f.astype(np.float32), GH, GW).mean(-1) for f in frames])
    now = np.zeros(L.shape, bool)
    now[1:] = np.abs(L[1:] - L[:-1]) > 4
    pad = np.pad(now, ((0, 0), (1, 1), (1, 1)))          # 1-px dilation
    now = pad[:, 1:-1, 1:-1] | pad[:, :-2, 1:-1] | pad[:, 2:, 1:-1] | pad[:, 1:-1, :-2] | pad[:, 1:-1, 2:]
    recent = now.copy()
    for k in range(1, int(FPS)):
        recent[k:] |= now[:-k]
    near = recent.copy()                                  # + one cell (5 px at 160x90)
    for _ in range(5):
        p = np.pad(near, ((0, 0), (1, 1), (1, 1)))
        near = p[:, 1:-1, 1:-1] | p[:, :-2, 1:-1] | p[:, 2:, 1:-1] | p[:, 1:-1, :-2] | p[:, 1:-1, 2:]
    return now, near


def run(acc, ctl_at, w, h, frames=None):
    """Yields (t, key, matte, M) per frame; ctl_at(t) -> Controls (may vary over time).
    Frame t renders the grid from update t-1, as the RTL does (one-update lag).
    frames (RGB at w x h) enable DETAIL; a STEADY cut mutes the key until re-seeded."""
    if ENGINE == "v3":
        eng = wm.MotionKey3(TID, 32)
        eng.set_geometry(AW, AH)
    else:
        eng = wm.MotionKey(TID, 32)
        eng.geo = wm.Geometry(AW, AH, 32, wm.ROWS)
    rgeo = wm.Geometry(w, h, 32, wm.ROWS)
    M_prev, mute = None, False
    for t in range(len(acc)):
        ctl = ctl_at(t)
        if M_prev is None:
            m = np.zeros((h, w), np.int32)
        else:
            m = wm.render_matte(M_prev, rgeo, wm.softness_params(ctl.softness)[0])
        gate = None
        if frames is not None and ctl.detail > 0:
            y = wm.rgb_to_yuv10(frames[t] / 255.0, "709")[..., 0]
            gate = wm.detail_gate(y, ctl.detail, px_scale=max(1, 1920 // w))
        key, raw = wm.shape_key(m, ctl, gate)
        if mute:
            raw = np.zeros_like(raw)
            key = np.full_like(raw, 1023) if ctl.key else raw
        out = eng.finalize(acc[t], ctl, seed=(t == 0))
        M_prev, mute = out["M"], out.get("mute", False)
        yield t, key, raw, m, out


def metrics(name, ctl):
    acc, frames = load(name)
    now, near = gt_masks(frames)
    rec, fal, area, flick, prev = [], [], [], [], None
    for t, key, raw, m, out in run(acc, lambda t: ctl, GW, GH):
        k = raw > 511
        if t < 2:
            prev = raw
            continue
        if now[t].sum() > 20:
            rec.append((k & now[t]).sum() / now[t].sum())
        fal.append((k & ~near[t]).sum() / max(1, (~near[t]).sum()))
        area.append(k.mean())
        flick.append(np.abs(raw.astype(np.int32) - prev).mean() / 1023)
        prev = raw
    return dict(recall=np.mean(rec) if rec else np.nan, false=np.mean(fal),
                area=np.mean(area), flicker=np.mean(flick), moving=now[2:].mean())


def sweep(param, names):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    vals = list(range(0, 1024, 64)) + [1023]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    print(f"{'clip':12s} " + " ".join(f"{v:>5d}" for v in vals))
    for name in names:
        res = [metrics(name, replace(DEFAULTS, **{param: v})) for v in vals]
        for ax, key in zip(axes, ("recall", "false", "area")):
            ax.plot([v / 10.23 for v in vals], [r[key] for r in res], label=name)
            ax.set_title(f"{key} vs {param} (%)")
        print(f"{name:12s} recall " + " ".join(f"{r['recall']:5.2f}" for r in res))
        print(f"{'':12s} false  " + " ".join(f"{r['false']:5.3f}" for r in res))
    axes[0].legend(fontsize=7)
    out = CACHE / f"sweep_{param}.png"
    fig.savefig(out, dpi=80, bbox_inches="tight")
    print("wrote", out)


def cutstats():
    """Per update: max running-mean raw d (rows >= 2) and whole-frame mean, split into
    true cuts (big global luma change) vs everything else."""
    cut_v, other_v = [], []
    for name in list(CUT_CLIPS) + ["action", "bridge_walk"]:
        acc, frames = load(name)
        L = np.array([wm.box_resize(f.astype(np.float32), GH, GW).mean(-1) for f in frames])
        gd = np.zeros(len(L)); gd[1:] = np.abs(L[1:] - L[:-1]).mean(axis=(1, 2))
        eng = wm.MotionKey3(TID, 32); eng.set_geometry(AW, AH)
        for t in range(len(acc)):
            out = eng.finalize(acc[t], replace(DEFAULTS, steady=0), seed=(t == 0))
            if t < 2:
                continue
            d = out["d"]
            run = np.cumsum(d.sum(axis=1)) / (d.shape[1] * np.arange(1, d.shape[0] + 1))
            v = run[2:].max()
            (cut_v if gd[t] > 20 else other_v).append(v)
            if gd[t] > 20:
                print(f"  cut in {name} at t={t / FPS:5.2f}s: global luma diff {gd[t]:5.1f}, max running mean d {v:6.1f}")
    o = np.array(other_v)
    print(f"non-cut updates: n={len(o)} running-mean p50 {np.percentile(o, 50):.1f} p99 {np.percentile(o, 99):.1f} max {o.max():.1f}")
    print(f"cut updates:     n={len(cut_v)} min {min(cut_v):.1f}")


def detailstill(name, t_frame, eff="Mono", inv=0):
    """input | effect at DETAIL 0/50/100% for one frame, rendered at 960x540."""
    from PIL import Image
    acc, _ = load(name)
    video, start, dur = {**CLIPS, **CUT_CLIPS}[name]
    frames = np.array(list(decode(video, start, dur, AW, AH)))[:len(acc)]
    panels = [_label(frames[t_frame], [f"input {name}"])]
    for det in (0, 512, 1023):
        ctl = replace(DEFAULTS, detail=det, key=inv)
        for t, key, raw, m, _ in run(acc[:t_frame + 1], lambda t: ctl, AW, AH, frames):
            pass
        panels.append(_label(_panel(frames[t_frame], key, raw, m, eff), [f"{eff}{' +Invert' if inv else ''}  DETAIL {det / 10.23:.0f}%"]))
    top = np.concatenate(panels[:2], 1); bot = np.concatenate(panels[2:], 1)
    out = CACHE / f"detail_{name}_{eff.replace(' ', '')}.png"
    Image.fromarray(np.concatenate([top, bot], 0)).resize((1920, 1080)).save(out)
    print("wrote", out)


# ---------------------------------------------------------------------------
# Tours: labelled videos of one control sweeping while footage plays
# ---------------------------------------------------------------------------
OUT = Path(__file__).resolve().parent.parent / "clips"
TOURS = {   # name: (clips, param, lo, hi, effect, note)
    "stir":     (["bridge_talk", "trio"], "stir", 0, 1023, "Heat", "STIR fader 0 -> 100% (sensitivity)"),
    "wake":     (["bridge_walk", "trio"], "wake", 0, 1023, "Heat", "WAKE knob 0 -> 100% (trail length)"),
    "softness": (["action", "bridge_talk"], "softness", 0, 1023, "Heat", "SOFTNESS 0 -> 100% (blocky -> glow)"),
    "floor":    (["seated", "bridge_talk"], "floor", 1023, 0, "Heat", "FLOOR 100 -> 0% (noise coring)"),
}


def _label(img, lines):
    from PIL import Image, ImageDraw
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    for i, s in enumerate(lines):
        d.rectangle([0, 18 * i, 8 + 7 * len(s), 18 * i + 16], fill=(0, 0, 0))
        d.text((4, 18 * i + 2), s, fill=(255, 255, 255))
    return np.asarray(im)


def _panel(rgb, key, raw, m, eff):
    yuv = wm.rgb_to_yuv10(rgb / 255.0, "709")
    return wm.yuv10_to_rgb8(wm.compose(eff, yuv, key, m, raw), "709")


def _writer(name, w, h):
    import imageio_ffmpeg
    OUT.mkdir(parents=True, exist_ok=True)
    wr = imageio_ffmpeg.write_frames(str(OUT / f"tour_{name}.mp4"), (w, h), fps=FPS, codec="libx264",
                                     pix_fmt_out="yuv420p", quality=8, macro_block_size=4,
                                     ffmpeg_log_level="error")
    wr.send(None)
    return wr


PRESET_CLIPS = [("First Look", "bridge_talk"), ("Color Pop", "trio"), ("Long Wake", "bridge_walk"),
                ("Nervous", "action"), ("Contour", "trio"), ("Steady Smear", "action")]


def preset_segment(i):
    """Preset i of PRESET_CLIPS on its clip at 960x540: input | WAKE output."""
    pname, clip = PRESET_CLIPS[i]
    ctl = wm.load_preset(str(Path(__file__).resolve().parent.parent.parent / "wake.toml"), pname)
    eff = wm.EFFECTS[wm.effect_index(ctl.effect)]
    acc, _ = load(clip)
    video, start, dur = CLIPS[clip]
    frames = np.array(list(decode(video, start, dur, AW, AH)))[:len(acc)]
    wr = _writer(f"preset{i}", 2 * AW, AH)
    for t, key, raw, m, _ in run(acc, lambda t: ctl, AW, AH, frames):
        lab = [f"preset {pname}: {eff}" + (" +KEY Invert" if ctl.key else "") +
               (" +STEADY" if ctl.steady else "") + f"  DETAIL {ctl.detail / 10.23:.0f}%"]
        strip = np.concatenate([_label(frames[t], [f"input ({clip})"]),
                                _label(_panel(frames[t], key, raw, m, eff), lab)], 1)
        wr.send(np.ascontiguousarray(strip))
    wr.close()


def tour(name):
    global ENGINE
    if name == "compare":                    # old engine vs new, both at their own defaults
        wr = _writer(name, 3 * 640, 360)
        for clip in ["seated", "bridge_talk", "trio", "action"]:
            ENGINE = "v2"
            wm.STIR_V2 = True
            a2, frames = load(clip)
            old = list(run(a2, lambda t: replace(wm.Controls(), floor=205), 640, 360))
            ENGINE = "v3"
            wm.STIR_V2 = False
            a3, _ = load(clip)
            new = list(run(a3, lambda t: DEFAULTS, 640, 360))
            for (t, k2, r2, m2, _), (_, k3, r3, m3, _) in zip(old, new):
                rgb = frames[t]
                strip = np.concatenate([_label(rgb, [f"input  {clip}"]),
                                        _label(_panel(rgb, k2, r2, m2, "Heat"), ["v0.2 engine (on your Videomancer now)"]),
                                        _label(_panel(rgb, k3, r3, m3, "Heat"), ["v0.3 engine (proposed)"])], 1)
                wr.send(np.ascontiguousarray(strip))
        wr.close()
        return
    if name == "effects":                    # each effect for 2 s, then Mono with KEY Invert
        seq = [("Matte", 0), ("Heat", 0), ("Mono", 0), ("Quarter Turn", 0), ("Negative", 0), ("Mono", 1)]
        wr = _writer(name, 2 * 640, 360)
        acc, frames = load("bridge_walk")
        acc2, frames2 = load("trio")
        acc, frames = acc + acc2, np.concatenate([frames, frames2])
        per = len(frames) // len(seq)
        ctl_at = lambda t: replace(DEFAULTS, key=seq[min(t // per, len(seq) - 1)][1])
        for t, key, raw, m, _ in run(acc, ctl_at, 640, 360):
            eff, inv = seq[min(t // per, len(seq) - 1)]
            lab = f"EFFECT {eff}" + ("  +  KEY Invert" if inv else "")
            strip = np.concatenate([_label(frames[t], ["input"]),
                                    _label(_panel(frames[t], key, raw, m, eff), [lab])], 1)
            wr.send(np.ascontiguousarray(strip))
        wr.close()
        return
    clips, param, lo, hi, eff, note = TOURS[name]
    acc, frames = [], []
    for c in clips:
        a, f = load(c)
        acc += list(a); frames.append(f)
    frames = np.concatenate(frames)
    n = len(frames)
    val = lambda t: int(round(lo + (hi - lo) * t / (n - 1)))
    wr = _writer(name, 2 * 640, 360)
    for t, key, raw, m, _ in run(acc, lambda t: replace(DEFAULTS, **{param: val(t)}), 640, 360):
        pct = val(t) / 10.23
        bar = np.zeros((10, 640, 3), np.uint8)
        bar[2:8, :int(6.4 * pct)] = (255, 200, 0)
        heat = _panel(frames[t], key, raw, m, eff)
        heat[-10:] = bar
        strip = np.concatenate([_label(frames[t], ["input", note]),
                                _label(heat, [f"{param.upper()} {pct:5.1f}%", eff])], 1)
        wr.send(np.ascontiguousarray(strip))
    wr.close()


if __name__ == "__main__":
    import os
    ENGINE = os.environ.get("WAKE_ENGINE", "v3")
    cmd = sys.argv[1]
    if cmd == "prep":
        prep(sys.argv[2:] or None)
    elif cmd == "cutstats":
        cutstats()
    elif cmd == "detailstill":
        detailstill(sys.argv[2], int(sys.argv[3]), *(sys.argv[4:5] or []), *[int(x) for x in sys.argv[5:6]])
    elif cmd == "sweep":
        sweep(sys.argv[2], sys.argv[3:] or list(CLIPS))
    elif cmd == "preset":
        preset_segment(int(sys.argv[2]))
    elif cmd == "tour":
        for n in sys.argv[2:]:
            tour(n)
            print("wrote", OUT / f"tour_{n}.mp4")
    elif cmd == "metrics":
        for n in sys.argv[2:] or CLIPS:
            print(n, {k: round(float(v), 3) for k, v in metrics(n, DEFAULTS).items()})
