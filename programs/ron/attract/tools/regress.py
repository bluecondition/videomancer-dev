#!/usr/bin/env python3
"""Emulator regression: hash the rendered canvas + RAM at fixed ticks."""
import hashlib, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import game, dasm, assets
only = '--dm' in sys.argv
knobs = {'k4': 300, 'sw': 0, 'k1': 0}
for arg in sys.argv[1:]:
    if '=' in arg:
        k, v = arg.split('='); knobs[k] = int(v)
rom, info = game.build(only_dm=only)
pats, _ = assets.load()
b = dasm.Board(rom, pats, knobs)
out = []
for t in range(1, 401):
    b.run_until_wait(); b.tick()
    if t % 50 == 0:
        h = hashlib.md5(repr(b.render()).encode()).hexdigest()[:10]
        out.append('%d:%s' % (t, h))
print(len(rom), ' '.join(out))
