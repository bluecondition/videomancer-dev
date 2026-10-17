#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""How well the demo pilot plays: run one game in the emulator and report
deaths per minute, games and best score (a game should last ~1-2 min).

  playstats.py K1 [TICKS] [SEED]      K1 = game index (1 = DOT MATRIX ...)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assets  # noqa: E402
import dasm    # noqa: E402
import game    # noqa: E402

k1 = int(sys.argv[1])
ticks = int(sys.argv[2]) if len(sys.argv) > 2 else 7200
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 300
pats, _ = assets.load()
rom, info = game.build()
V = info['vars']
b = dasm.Board(rom, pats, {'k1': k1, 'k4': seed, 'sw': 0})
deaths = games = best = 0
last = None
for t in range(ticks):
    b.run_until_wait()
    b.tick()
    lv = b.ram[V['lives']]
    if last is not None and lv < last:
        deaths += 1
    if last is not None and lv > last:
        games += 1
    last = lv
    best = max(best, sum(b.ram[V['score'] + i] * 10 ** i for i in range(6)))
print('k1=%d seed=%d ticks=%d: deaths %d (%.1f/min), games %d, best score %d, max cycles/tick %d'
      % (k1, seed, ticks, deaths, deaths * 3600.0 / ticks, games, best, b.max_tick_cycles))
