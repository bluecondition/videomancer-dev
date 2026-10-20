#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Dead-zone exactness: bottom 2 % == A bit-exact, top 2 % == B bit-exact
(no slivers, no border hairlines) over every pattern x 9 ORIGIN anchors x
MODIFIER extremes x SPIN x REVERSE x EDGE extremes, in every mode."""
import sys, itertools
from multiprocessing import Pool
import numpy as np
import sw_frame as fr
import sw_sheet as sh

ANCH = [0, 57, 200, 302, 404, 506, 608, 710, 812, 914, 1023]   # centre, glide, 8 anchors, end


def job(args):
    mode, name = args
    A = sh.source(mode)
    fails = []
    n = 0
    for k3, k2, s9, k4, s8 in itertools.product(ANCH, [0, 512, 1023], [0, 1], [0, 300, 512, 1023], [0, 1]):
        for p12, want in ((0, 'A'), (20, 'A'), (1004, 'B'), (1023, 'B')):
            ctl = sh.pat_ctl(name, k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12, k6=500)
            out, P, clk = fr.render_emu(mode, ctl, A)
            ref = A if want == 'A' else np.broadcast_to(np.array(P['BC']), A.shape)
            n += 1
            bad = int((out != ref).any(-1).sum())
            if bad:
                fails.append((mode, name, dict(k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12), bad))
    return n, fails


if __name__ == '__main__':
    modes = sys.argv[1:] or ['480i', '720p', '1080p']
    jobs = [(m, p) for m in modes for p in fr.CLASSIC + fr.CHEESE]
    tot, allf = 0, []
    with Pool(min(22, len(jobs))) as pool:
        for n, f in pool.imap_unordered(job, jobs):
            tot += n; allf += f
    for f in allf[:40]:
        print('FAIL', *f)
    print(f'{tot} end checks, {len(allf)} failures')
