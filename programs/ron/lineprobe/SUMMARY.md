# Line Probe — input timing glitch detector + HDMI stress patterns

Diagnostic for the cross-program "bars" glitch (bands ~10 lines tall that start
partway along a line and run to the right edge, solid colours that change, flashing
in and out). Built 2026-10-02; 6/6 seed 1, 24% LC, no EBR, latency 2. No core changes.

## What it detects (from data_in timing, every line)

| Event | Colour | Meaning |
|---|---|---|
| E1 | red | avid dropout: avid falls and rises again within one line |
| E2 | green | early hsync: hsync edge during an active line, 8+ clocks short of the last period |
| E3 | blue | total active length differs from the previous active line |
| E4 | yellow | hsync→avid start offset differs from the previous line |

S8 Tolerance: Strict = any change, Loose = more than ±1 clock (analog sources wander ±1).
Each event lights a tick in its own 32-px column at the left edge for 8 lines, and its
square in the top-right corner for ~1 s (grey = idle). S7 turns markers off.

## Patterns (K1)

Video · Black · Grey · Orange · Stripes (1-px B/W) · Checker (1-px orange/blue) ·
Noise · Bars. Output avid/syncs pass through delayed, like every normal program.

## How to read the result

- Bars over **Black/Grey** = not a U/V swap (neutral chroma is immune) → corruption
  downstream of the FPGA logic: HDMI link / monitor TMDS resync, or TX pin timing.
- Bars over **Orange** that are blue/purple = U/V swap; random colours = link corruption.
- Bars that coincide with red/green/yellow markers = input timing glitches are real
  (program-side fixes possible: own avid, per-line U/V repair). No markers while bars
  show = the input timing is clean and the problem is on the output side.
- Bars only with Stripes/Checker/Noise = content-triggered (signal integrity).
