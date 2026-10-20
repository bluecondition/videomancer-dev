#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Dead-zone exactness against the EMULATED frame program: bottom 2 % of
travel == A bit-exact (through the 4:2:2 packer), top 2 % == B bit-exact,
over every pattern x ORIGIN anchors x MODIFIER extremes x SPIN x REVERSE x
EDGE extremes.  python3 sw_ends2.py [mode ...]"""
import sys, itertools
from multiprocessing import Pool
import numpy as np
import sw_run2 as r2, sw_ucode2 as uc, sw_sheet as sh, sw_sheet2 as sh2, sw_scan as sc

ANCH = [0, 57, 200, 302, 404, 506, 608, 710, 812, 914, 1023]


def job(args):
    mode, name = args
    A = sh.source(mode)
    Ap = sc.pack422(A)
    fails, n = [], 0
    for k3, k2, s9, k4, s8 in itertools.product(ANCH, [0, 512, 1023], [0, 1], [0, 300, 512, 1023], [0, 1]):
        for p12, want in ((0, 'A'), (20, 'A'), (1004, 'B'), (1023, 'B')):
            ctl = sh2.ctl_for(name, k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12, k6=500)
            try:
                out, info = r2.render(mode, ctl, A)
            except Exception as ex:
                fails.append((mode, name, dict(k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12), 'EXC ' + str(ex)[:60]))
                n += 1
                continue
            P = info[0]['P']
            BC = (P['BCY'] & 1023, (P['BCUV'] >> 10) & 1023, P['BCUV'] & 1023)
            ref = Ap if want == 'A' else np.broadcast_to(np.array(BC), A.shape)
            n += 1
            bad = int((out != ref).any(-1).sum())
            if bad:
                fails.append((mode, name, dict(k3=k3, k2=k2, s9=s9, k4=k4, s8=s8, p12=p12), bad))
    return n, fails


if __name__ == '__main__':
    sh2.fr.CLASSIC, sh2.fr.CHEESE = uc.CLASSIC, uc.CHEESE
    modes = sys.argv[1:] or ['480i', '1080p']
    jobs = [(m, p) for m in modes for p in uc.CLASSIC + uc.CHEESE]
    tot, allf = 0, []
    with Pool(min(20, len(jobs))) as pool:
        for n, f in pool.imap_unordered(job, jobs):
            tot += n; allf += f
            print('.', end='', flush=True)
    print()
    import collections
    cat = collections.Counter()
    for f in allf:
        c = f[2]
        cat[(f[0], f[1], 'rev' if c['s8'] else 'fwd', 'k4=%d' % c['k4'], 'k2=%d' % c['k2'], 'lo' if c['p12'] < 100 else 'hi')] += 1
    for k, v in sorted(cat.items()):
        print('CAT', v, *k)
    for f in allf[:60]:
        print('FAIL', *f)
    print(f'{tot} end checks, {len(allf)} failures')
