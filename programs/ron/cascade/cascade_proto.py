#!/usr/bin/env python3
"""
HYPNOS CASCADE v3 -- SEQUENTIAL CASCADE prototype (exact-circle render).

Each RING k has its own centre C[k]; a whole ring shares one offset and stays
circular.  Chain:
  - ring 0 glides toward the knob target.
  - ring k stays put until ring k-1 pulls > T ahead, then follows (dead-zone).
  - NO accumulated cap: the push wave travels the full chain (v2's 1.5-ring cap
    froze rings 3+ into lockstep -- the user-rejected motion).
catch_up=True : rings glide home concentric once the inner ring settles.
catch_up=False: on arrival the chain FREEZES -> rings stay dragged.

RENDER (matches the v3 RTL structure): the first NR discs are EXACT circles
|P - C_k| < R_k (the RTL rasterises them as per-scanline spans via an integer
sqrt); everything outside disc NR-1 is a concentric field around the LAST
ring's centre (the RTL tracks r^2 incrementally with integer band brackets).
Colour = parity of the innermost covering disc.  Integer-exact in HW: no
CORDIC, no cos/sin, no projection -- the v1/v2 jaggy and sliver artifact
classes are structurally impossible.
"""
import numpy as np
from PIL import Image, ImageDraw
import os

W, H = 360, 270
NR = 24                      # modelled disc slots (matches C_NR)
SPACING = 14.0               # ring spacing, px (dense: shows the 24-deep wave)
SLACK = 0.875                # axis slack A as a fraction of spacing (octagon clamp)

def render(C):
    xs = np.arange(W); ys = np.arange(H)
    X, Y = np.meshgrid(xs, ys)
    # outer field: concentric around the last modelled ring's centre
    r_out = np.sqrt((X - C[NR - 1][0]) ** 2 + (Y - C[NR - 1][1]) ** 2)
    disc = np.maximum(np.floor(r_out / SPACING).astype(int), NR)
    # nest: innermost covering disc wins (painter's order, outside-in)
    for k in range(NR - 1, -1, -1):
        rk = (k + 1) * SPACING
        inside = (X - C[k][0]) ** 2 + (Y - C[k][1]) ** 2 < rk * rk
        disc[inside] = k
    key = ((disc % 2) == 0)
    return (key * 255).astype(np.uint8)

def label(img, txt):
    im = Image.fromarray(img).convert("RGB")
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, len(txt) * 6 + 4, 12], fill=(0, 0, 0))
    d.text((2, 1), txt, fill=(255, 230, 0))
    return np.asarray(im)

def simulate(catch_up=True, g=0.26, gset=0.16, move=(170, 90), move_frame=2):
    T = SPACING * SLACK
    C0 = np.array([W * 0.5, H * 0.5])
    C = np.tile(C0, (NR, 1))
    target = C0.copy()
    caps = [0, 5, 9, 14, 20, 28, 40, 61]
    frames = []
    for f in range(0, 62):
        if f == move_frame:
            target = C0 + np.asarray(move, float)
        C[0] += g * (target - C[0])                    # inner ring follows the knob
        active = np.linalg.norm(target - C[0]) > 2.0
        CT = T * 1.414                                 # 45-degree octagon bound
        for k in range(1, NR):
            if catch_up and not active:
                C[k] += gset * (C[k - 1] - C[k])
            # OCTAGON coupling: per-axis +/-T and 45-degree +/-T*sqrt2 from
            # the ring inside (never-overlap in every drag direction; NO
            # accumulated cap -- the wave travels the chain).
            d = np.clip(C[k] - C[k - 1], -T, T)
            u = np.clip(d[0] + d[1], -CT, CT)
            v = np.clip(d[0] - d[1], -CT, CT)
            C[k] = C[k - 1] + np.array([(u + v) / 2, (u - v) / 2])
        if f in caps:
            frames.append(label(render(C), f"f{f}"))
    return frames

if __name__ == "__main__":
    outdir = os.path.dirname(os.path.abspath(__file__))
    rows = [("CATCH-UP on  (glides home to concentric)", simulate(catch_up=True)),
            ("CATCH-UP off (stays dragged / frozen)",     simulate(catch_up=False))]
    cols = len(rows[0][1])
    RH = H + 18
    strip = np.zeros((2 * RH, cols * W, 3), np.uint8)
    for ri, (name, frames) in enumerate(rows):
        for i, fr in enumerate(frames):
            strip[ri * RH + 18: ri * RH + 18 + H, i * W:(i + 1) * W] = fr
    im = Image.fromarray(strip); d = ImageDraw.Draw(im)
    d.text((4, 4), rows[0][0], fill=(255, 230, 0))
    d.text((4, RH + 4), rows[1][0], fill=(255, 230, 0))
    im.save(os.path.join(outdir, "chain_strip.png"))
    print("wrote chain_strip.png", strip.shape)
