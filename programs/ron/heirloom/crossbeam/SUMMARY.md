# Crossbeam — Heirloom #37

**Source photo**: `photos/WIN_20230527_01_43_26_Pro.jpg` (1920×1080 capture, 2023-05-27)

## Breakdown of the photo

- **Composition**: a soft cyan-green X — two broad diagonal bands crossing at the screen centre — laid over a
  field of Mondrian-like colour blocks (magenta, yellow, blue, green, black, white).
- **Blocks**: coarse rectangles, torn sideways in horizontal bands (row groups slid by different amounts).
- **Texture**: where the blocks are dark, a fine red ripple (horizontal zigzag stripes) fills them.
- **Palette**: saturated primaries and secondaries on black, the X in a pale cyan with a luma gradient.

## Design

| Photo element | Program |
|---|---|
| Colour blocks | square cells (K1, virtual units), colour = 8-entry palette[hash(cell, seed)] (hash spot-checked: no row/column/diagonal correlation), or the source luma posterized through the same palette (S10 Fill) |
| Row tears | per-8-line random offset (K2) applied to the cell phase (an hblank subtraction loop) and to the line-buffer read start |
| Red ripple in dark blocks | tri(x·K3 + tri(y)) > threshold where the block luma < 300 (S8) |
| The X | two linear fields d = a·(x−cx) ∓ 256·dy (phase accumulators, a from K5); level = W − \|d\| clipped; core solid cyan, rim half-mixed (S7) |
| X width | P12 CROSS: W = p12·64 |
| Motion | K6 slides the X up / down per field |

Source Video / Osc on S11 (oscillator at the line-buffer write). Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Blocks | block size 16–271 virtual px |
| K2 Tear | sideways tearing of blocks and picture |
| K3 Ripple | ripple pitch |
| K4 Hue | palette rotation (also the source posterization) |
| K5 Angle | X slope |
| K6 Drift | X slides up / down |
| S7 Cross | X on / off |
| S8 Ripple | red ripple on / off |
| S9 Palette | Mondrian / Candy |
| S10 Fill | Source (posterized picture) / Flat (hashed blocks) |
| S11 Source | Video / Osc |
| **P12 Cross** | X band width: hairline → the whole screen glows cyan |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Dual seed 2, others seed 1), HD 76.7–80.7 MHz, 4.22k LC (55 %), 2 EBR
  (only the posterized luma bits of the line buffer survive pruning). **Not yet flashed.**
