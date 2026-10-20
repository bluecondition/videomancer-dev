#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Contact sheets for the scanline engine: every pattern of both banks at
~50 % with a gold border, per mode.  python3 sw_sheet2.py OUTDIR [mode]"""
import sys, os
import numpy as np
from PIL import Image
from multiprocessing import Pool
import sw_frame2 as fr
import sw_sheet as sh

BASE = dict(k1=0, k2=512, k3=0, k4=640, k5=900, k6=0, s7=0, s8=0, s9=0, s10=0, s11=0, p12=530)


def ctl_for(name, **kw):
    lst, bank = (fr.CLASSIC, 0) if name in fr.CLASSIC else (fr.CHEESE, 1)
    if name not in lst:
        raise KeyError(name)
    i = lst.index(name)
    c = dict(BASE, s7=bank, k1=(i * 1024 + 512) // len(lst)); c.update(kw)
    return c


def one(args):
    mode, name, kw = args
    A = sh.source(mode)
    M = fr.MODES[mode]
    try:
        out = fr.render(mode, ctl_for(name, **kw), A)
        im = sh.display(mode, out)
    except Exception as e:
        im = Image.new('RGB', (M['W'], M['HF']), (120, 0, 0))
        print('ERR', name, e)
    return name, sh.label(im.resize((480, 270), Image.LANCZOS), name + ''.join(f' {k}={v}' for k, v in kw.items()))


def sheet(outdir, mode, items, tag):
    with Pool(12) as p:
        tiles = p.map(one, [(mode, n, kw) for n, kw in items])
    cols = 6
    rows = (len(tiles) + cols - 1) // cols
    S = Image.new('RGB', (cols * 480, rows * 270), (30, 30, 30))
    for i, (n, t) in enumerate(tiles):
        S.paste(t, ((i % cols) * 480, (i // cols) * 270))
    S.save(os.path.join(outdir, f'{tag}_{mode}.png'))


if __name__ == '__main__':
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    modes = sys.argv[2:] or ['1080p']
    for mode in modes:
        sheet(outdir, mode, [(n, {}) for n in fr.CLASSIC], 'classic')
        sheet(outdir, mode, [(n, {}) for n in fr.CHEESE], 'cheese')
        print('wrote', mode)
