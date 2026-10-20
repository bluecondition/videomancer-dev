#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Compare a GHDL image-tester render with the emulator render at the same raster.

    sw_simcmp.py <sim.png> <emu.npy> [<out.png>]

The emulator output (10-bit YUV, hardware order) is converted to RGB the way
sw_sheet does; the sim PNG is the tester's RGB render.  Reports the fraction
of pixels whose max channel difference exceeds 8 and 32, and writes a
side-by-side (sim | emu | diff) image.
"""
import sys
import numpy as np
from PIL import Image
import sw_sheet as sh

sim = np.asarray(Image.open(sys.argv[1]).convert('RGB')).astype(np.int64)
emu = np.load(sys.argv[2])
emu_rgb = sh.to_rgb8(emu).astype(np.int64)
if sim.shape != emu_rgb.shape:
    print('shape mismatch', sim.shape, emu_rgb.shape)
    sim = np.asarray(Image.fromarray(sim.astype(np.uint8)).resize((emu_rgb.shape[1], emu_rgb.shape[0]))).astype(np.int64)
d = np.abs(sim - emu_rgb).max(-1)
print('pixels %d  diff>8: %.4f  diff>32: %.4f  mean %.2f  max %d' % (d.size, (d > 8).mean(), (d > 32).mean(), d.mean(), d.max()))
rows = np.where((d > 32).any(1))[0]
if len(rows):
    print('rows with big diffs: %d (first %d, last %d)' % (len(rows), rows[0], rows[-1]))
    cols = np.where((d > 32).any(0))[0]
    print('cols with big diffs: %d (first %d, last %d)' % (len(cols), cols[0], cols[-1]))
if len(sys.argv) > 3:
    dd = np.clip(d * 4, 0, 255).astype(np.uint8)
    side = np.concatenate([sim.astype(np.uint8), emu_rgb.astype(np.uint8), np.stack([dd] * 3, -1)], 1)
    Image.fromarray(side).save(sys.argv[3])
    print('wrote', sys.argv[3])
