#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""Load the text-grid assets and lay out the shared pattern ROM.

Pattern ROM = N x 8 rows x 16 bits (8 pixels x 2 bits, leftmost pixel in
bits 15:14).  Layout:
    0..63    font, pattern = ASCII - 0x20 (so text tile = char - 32)
    64..255  tiles (tile layer can only address 0..255)
    then     sprites, packed straight after the tiles: 16x16 = 4
             consecutive patterns (TL TR BL BR, base multiple of 4),
             sprite8 = 1 pattern (fills alignment gaps)
Pure Python (the SDK build hook imports it).
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, '..', 'assets')

# files in load order (tiles before sprites keeps tiles under 256)
FILES = ['font.txt', 'tiles.txt', 'walls.txt', 'enemies.txt', 'blip.txt',
         'moths.txt', 'items.txt', 'effects.txt']
# enemies.txt comes first among the sprites so the car / rocket / fuel
# patterns stay under 256 and can be laid down as tile blocks too

PIX = {'.': 0, '1': 1, '2': 2, '3': 3}


class AssetError(Exception):
    pass


def parse(path):
    entries = []
    cur = None
    with open(path) as f:
        for ln, raw in enumerate(f, 1):
            line = raw.rstrip('\n').rstrip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('@'):
                m = re.match(r'@(tile|font|sprite|sprite8)\s+([0-9A-Za-z_]+)$', line)
                if not m:
                    raise AssetError('%s:%d bad header %r' % (path, ln, line))
                cur = {'kind': m.group(1), 'name': m.group(2), 'rows': [],
                       'where': '%s:%d' % (os.path.basename(path), ln)}
                entries.append(cur)
                continue
            if cur is None:
                raise AssetError('%s:%d pixels before a header' % (path, ln))
            cur['rows'].append(line)
    for e in entries:
        size = 16 if e['kind'] == 'sprite' else 8
        if len(e['rows']) != size:
            raise AssetError('%s %s: %d rows, want %d' % (e['where'], e['name'], len(e['rows']), size))
        for r in e['rows']:
            if len(r) != size or any(ch not in PIX for ch in r):
                raise AssetError('%s %s: bad row %r' % (e['where'], e['name'], r))
    return entries


def rowword(s):
    w = 0
    for ch in s:
        w = (w << 2) | PIX[ch]
    return w


def quad(rows):
    """16x16 -> four 8x8 patterns TL TR BL BR, each 8 row words."""
    out = []
    for (r0, c0) in ((0, 0), (0, 8), (8, 0), (8, 8)):
        out.append([rowword(rows[r0 + i][c0:c0 + 8]) for i in range(8)])
    return out


def load():
    """Returns (patterns, index) -- patterns is a list of 8-word lists;
    index maps 'name' -> pattern number (font glyphs as 'ch_HH')."""
    pats = [None] * 64
    index = {}
    tiles, sprites, sprite8s = [], [], []
    for fn in FILES:
        p = os.path.join(ASSETS, fn)
        if not os.path.exists(p):
            continue
        for e in parse(p):
            if e['name'] in index or any(x['name'] == e['name'] for x in tiles + sprites + sprite8s):
                raise AssetError('duplicate name ' + e['name'])
            if e['kind'] == 'font':
                code = int(e['name'], 16)
                if not 0x20 <= code <= 0x5F:
                    raise AssetError('font code out of range ' + e['name'])
                pats[code - 0x20] = [rowword(r) for r in e['rows']]
                index['ch_%02X' % code] = code - 0x20
            elif e['kind'] == 'tile':
                tiles.append(e)
            elif e['kind'] == 'sprite':
                sprites.append(e)
            else:
                sprite8s.append(e)
    for i in range(64):
        if pats[i] is None:
            pats[i] = [0] * 8
    for e in tiles:
        index[e['name']] = len(pats)
        pats.append([rowword(r) for r in e['rows']])
    if len(pats) > 256:
        raise AssetError('tiles overflow the 256 tile-addressable patterns (%d)' % len(pats))
    # sprites pack straight after the tiles (they can live anywhere);
    # 16x16 bases are 4-aligned, sprite8s fill the alignment gaps first
    s8 = list(sprite8s)
    for e in sprites:
        while len(pats) % 4:
            if s8:
                e8 = s8.pop(0)
                index[e8['name']] = len(pats)
                pats.append([rowword(r) for r in e8['rows']])
            else:
                pats.append([0] * 8)
        index[e['name']] = len(pats)
        pats.extend(quad(e['rows']))
    for e8 in s8:
        index[e8['name']] = len(pats)
        pats.append([rowword(r) for r in e8['rows']])
    return pats, index


if __name__ == '__main__':
    pats, idx = load()
    nt = sum(1 for k, v in idx.items() if v < 256)
    print('patterns: %d (%d font+tile entries)' % (len(pats), nt))
