#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Previews (system python3 + PIL; not used by the SDK build).

  preview.py sheet            contact sheet of every asset -> contact/sheet_1x.png, sheet_4x.png
  preview.py run [ticks] [--every N] [--k1 V] ...
                              run the real ROM in the emulator, save canvas frames
                              -> contact/run_XXXX.png (+ run.gif)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, '..', 'contact')

from PIL import Image, ImageDraw   # noqa: E402
import assets                       # noqa: E402
import dasm                         # noqa: E402

# preview colours per family (3-3-2 values are the real palette space)
def c332(r, g, b):
    return (r << 5) | (g << 2) | b


FAMILY_PAL = {
    'blip':   [0, c332(1, 6, 3), c332(0, 0, 1), c332(7, 7, 3)],
    'moth':   [0, c332(7, 4, 0), c332(3, 1, 0), c332(7, 7, 2)],
    'luna':   [0, c332(4, 7, 2), c332(1, 3, 1), c332(7, 3, 3)],
    'grub':   [0, c332(7, 6, 1), c332(4, 2, 0), c332(0, 0, 0)],
    'item':   [0, c332(7, 5, 0), c332(4, 2, 0), c332(7, 7, 3)],
    'beetle': [0, c332(0, 5, 3), c332(0, 2, 1), c332(7, 7, 3)],
    'spark':  [0, c332(7, 7, 0), c332(7, 3, 0), c332(7, 7, 3)],
    'fx':     [0, c332(7, 2, 0), c332(7, 5, 0), c332(7, 7, 3)],
    'tile':   [0, c332(1, 2, 3), c332(0, 1, 2), c332(5, 6, 3)],
    'font':   [0, c332(7, 7, 3), 0, 0],
}


def fam(name):
    for k in ('blip', 'moth', 'luna', 'grub', 'item', 'beetle', 'spark', 'spool'):
        if name.startswith(k):
            return 'spark' if k == 'spool' else k
    if name.startswith(('pop', 'boom', 'sparkle', 'bolt', 'blob', 'zig', 'ember',
                        'car', 'rocket', 'fuel')):
        return 'fx'
    return 'tile'


def draw_pat(img, pat, x0, y0, pal, scale):
    px = img.load()
    for r in range(8):
        w = pat[r]
        for c in range(8):
            v = (w >> (14 - 2 * c)) & 3
            if v:
                col = dasm.rgb332(pal[v])
                for dy in range(scale):
                    for dx in range(scale):
                        px[x0 + c * scale + dx, y0 + r * scale + dy] = col


def sheet():
    pats, idx = assets.load()
    entries = []
    for fn in assets.FILES:
        p = os.path.join(assets.ASSETS, fn)
        if os.path.exists(p):
            for e in assets.parse(p):
                entries.append(e)
    for scale in (1, 4):
        cell = 20 * scale
        cols = 16
        rows = (len(entries) + cols - 1) // cols
        img = Image.new('RGB', (cols * cell, rows * cell), (24, 24, 32))
        for i, e in enumerate(entries):
            x0 = (i % cols) * cell + 2 * scale
            y0 = (i // cols) * cell + 2 * scale
            name = e['name']
            if e['kind'] == 'font':
                base = int(name, 16) - 0x20
                draw_pat(img, pats[base], x0, y0, FAMILY_PAL['font'], scale)
                continue
            pal = FAMILY_PAL[fam(name)]
            base = idx[name]
            if e['kind'] == 'sprite':
                for q, (dr, dc) in enumerate(((0, 0), (0, 8), (8, 0), (8, 8))):
                    draw_pat(img, pats[base + q], x0 + dc * scale, y0 + dr * scale, pal, scale)
            else:
                draw_pat(img, pats[base], x0, y0, pal, scale)
        os.makedirs(OUT, exist_ok=True)
        fn = os.path.join(OUT, 'sheet_%dx.png' % scale)
        img.save(fn)
        print('wrote', fn, img.size)


def canvas_png(board, scale=2):
    rows = board.render()
    img = Image.new('RGB', (256 * scale, 240 * scale))
    px = img.load()
    for y in range(240):
        for x in range(256):
            col = dasm.rgb332(rows[y][x])
            for dy in range(scale):
                for dx in range(scale):
                    px[x * scale + dx, y * scale + dy] = col
    return img


def run(argv):
    import game
    ticks = 600
    every = 60
    knobs = {}
    i = 0
    tag = 'run'
    while i < len(argv):
        a = argv[i]
        if a == '--every':
            every = int(argv[i + 1]); i += 2
        elif a == '--tag':
            tag = argv[i + 1]; i += 2
        elif a.startswith('--'):
            knobs[a[2:]] = int(argv[i + 1]); i += 2
        else:
            ticks = int(a); i += 1
    rom, info = game.build()
    pats, _ = assets.load()
    b = dasm.Board(rom, pats, knobs)
    frames = []
    os.makedirs(OUT, exist_ok=True)
    for t in range(ticks):
        b.run_until_wait()
        b.tick(frame_start=True)
        if (t + 1) % every == 0:
            img = canvas_png(b)
            fn = os.path.join(OUT, '%s_%04d.png' % (tag, t + 1))
            img.save(fn)
            frames.append(img)
    if frames:
        frames[0].save(os.path.join(OUT, tag + '.gif'), save_all=True,
                       append_images=frames[1:], duration=200, loop=0)
    print('ticks %d, max cycles/tick %d, ROM %d words' % (ticks, b.max_tick_cycles, len(rom)))


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'run':
        run(sys.argv[2:])
    else:
        sheet()
