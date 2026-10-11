#!/usr/bin/env python3
"""Encoder pin skew per routed build: when HSYNC and data reach the pins vs the forwarded clock.

The encoders get no data-enable and pair Cb/Cr by counting from HSYNC.  The
core forwards their clock through a fabric LUT inverter, and none of these pin
paths are timing-checked, so the HSYNC-vs-clock skew is a placement lottery.
HSYNC landing within ~0.7 ns of the clock = random per-line U/V swaps (thin
lines over coloured areas).  See memory hdmi_tx_placement_skew.

Usage: route with
    nextpnr-ice40 ... --report r.json --detailed-timing-report
then  python3 tools/hdmi_pin_skew.py r1.json r2.json ...

LIMITATION: nextpnr's detailed report lists only part of the netlist -- for
these pins just the final hop (core LUT -> pin), not the FF -> LUT net before
it -- so these are final-route comparisons, not full arrivals.  Their value is
empirical (pixelbeach: the +0.48 build showed lines; three builds screened to
+1.4..+1.8 were clean on HW).  Do not build an all-paths STA on this report.

Columns are ns relative to the forwarded clock's arrival at its own pin
(board/IO delays not included, so only compare builds against each other):
  hs-clk   HSYNC arrival - clock arrival   (HD HDMI known-good +1.0..+1.7,
                                            known-bad +0.48 / ~-0.5)
  data     min..max data-bit arrival - clock arrival (keep all on one side)
"""
import json
import sys

LUT = 0.45   # SB_LUT4 in -> out
CQ = 0.54    # DFF clk -> q


def arrival(into, cell, depth=0):
    if 'DFF' in cell and 'LUT4_O' not in cell.split('.')[-1][-12:]:
        return CQ
    ins = [x for x in into.get(cell, []) if x[1].startswith('I')]
    if 'SB_LUT4' in cell and ins and depth < 4:
        return max(arrival(into, drv, depth + 1) + dl for drv, _, dl in ins) + LUT
    return 0.0


def encoder(into, pfx, n_data):
    def pin(name):
        return [x for x in into.get(name + '$sb_io', []) if x[1] == 'D_OUT_0'][0]
    drv, _, dl = pin(pfx + '_CLK')
    clk = arrival(into, drv) + dl
    drv, _, dl = pin(pfx + '_HSYNC')
    hs = arrival(into, drv) + dl
    data = []
    for i in range(n_data):
        try:
            drv, _, dl = pin(f'{pfx}_D[{i}]')
            data.append(arrival(into, drv) + dl)
        except IndexError:
            pass
    return hs - clk, min(data) - clk, max(data) - clk


for path in sys.argv[1:]:
    d = json.load(open(path))
    into = {}
    for n in d['detailed_net_timings']:
        for e in n['endpoints']:
            into.setdefault(e['cell'], []).append((n['driver'], e['port'], e['delay'][1]))
    fm = list(d['fmax'].values())[0]
    fm = fm['achieved'] if isinstance(fm, dict) else fm
    h1, a1, b1 = encoder(into, 'ADV7513', 24)
    h2, a2, b2 = encoder(into, 'ADV7393', 16)
    print(f"{path.split('/')[-1]:28s} {fm:5.1f} MHz | HDMI hs-clk {h1:+5.2f} "
          f"data {a1:+5.2f}..{b1:+5.2f} | analog hs-clk {h2:+5.2f} data {a2:+5.2f}..{b2:+5.2f}")
