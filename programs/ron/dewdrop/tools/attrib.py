#!/usr/bin/env python3
"""Histogram gate-level cells of an RTLIL dump by dewdrop.vhd source line."""
import sys, re, collections
il = sys.argv[1]; vhd = sys.argv[2]; top = int(sys.argv[3]) if len(sys.argv) > 3 else 50
lines = open(vhd).read().split('\n')
cnt = collections.Counter(); nosrc = collections.Counter()
src = None; inmod = False
for l in open(il):
    l = l.strip()
    if l.startswith('module '): inmod = ('program_top' in l)
    if not inmod: continue
    if l.startswith('attribute \\src'):
        ms = re.findall(r'dewdrop\.vhd:(\d+)', l); src = int(ms[-1]) if ms else None
    elif l.startswith('cell '):
        t = l.split()[1]
        if t.startswith('$__ICE40_CARRY') or 'CARRY' in t: k = 'C'
        elif 'DFF' in t: k = 'D'
        elif t.startswith('$_'): k = 'G'
        else: src = None; continue
        if src is None: nosrc[k] += 1
        else: cnt[(k, src)] += 1
        src = None
    elif not l.startswith('attribute'):
        src = None
print("no-src:", dict(nosrc))
tot = collections.Counter()
for (k, ln), v in cnt.items(): tot[ln] += v
print(f"totals: G={sum(v for (k,l),v in cnt.items() if k=='G')} C={sum(v for (k,l),v in cnt.items() if k=='C')} D={sum(v for (k,l),v in cnt.items() if k=='D')}")
for ln, v in tot.most_common(top):
    print(f"{v:5d} L{ln:5d} G={cnt[('G',ln)]:4d} C={cnt[('C',ln)]:4d} D={cnt[('D',ln)]:4d} | {lines[ln-1].strip()[:95]}")
