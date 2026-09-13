#!/usr/bin/env python3
"""Does the frame-recursive field behave under motion?

Pans the source across the frame one frame at a time, running the field
recursion exactly as the hardware would (one diffusion step per frame), and
renders the last frame with PERSIST off and on.  This is the test that decides
whether the K8 / bleed persistence approach is worth the BRAM.
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
import livewire_look as lw

img = lw.load('/home/ron/showcase_src.png', 0.5)
FR = 26
SPD = 16          # px/frame at 960 wide == 32 px/frame at HD == a fast pan


def panned(k):
    return np.roll(img, int((k - FR // 2) * SPD), axis=1)


out = []
for persist, label in ((0.0, "PERSIST 0"), (0.55, "PERSIST 50%"), (0.95, "PERSIST max")):
    for mode, mname in ((0, "NEON"), (2, "KIRLIAN")):
        p = lw.P(mode=mode, emerge=0.62, thresh=0.55, colour=0.15, persist=persist)
        p.field_state = np.zeros((lw.H_CELLS, lw.W_CELLS))
        for k in range(FR):
            frame = lw.render(panned(k), p, frame=k)
        out.append((f"{mname}  {label}", frame))
lw.sheet(out, 2, "/home/ron/livewire_work/motion.png", tw=760)
