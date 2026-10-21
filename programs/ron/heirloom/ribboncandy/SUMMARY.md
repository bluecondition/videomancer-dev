# Ribboncandy — Heirloom #39

**Source photo**: `photos/WIN_20230530_22_27_05_Pro.jpg` (1920×1080 capture, 2023-05-30)

## Breakdown of the photo

- **Ribbons**: six or seven vertical bands (green, blue, magenta, orange, red) whose x position wobbles down the
  screen in S-curves, each shaded from one colour at its left edge to another at its right.
- **Ground**: a black-and-white glyph-block field (coarse cells of white, black and grays) with yellow, cyan and
  magenta dashes scattered through it.
- **Palette**: saturated ribbon pairs on a monochrome field with CMY accents.

## Design

| Photo element | Program |
|---|---|
| Ribbons | up to 8 ribbons (K1); per line each ribbon's left edge = base_i + amp·tri(y·K3 + i·32 + drift) − w/2, computed by a 3-stage per-line sequencer in hblank; per pixel 8 parallel membership tests + priority select |
| Two-colour gradient | pos × (65536/w) → 8-bit t; colour = A + (B − A)·t (three 12×9 products); S7 Flat disables |
| Block field | hashed cells (K4) through an 8-entry white / black / gray / CMY palette; S9 Dashes swaps the CMY entries for grays |
| Source ground | S8: the source held per block column and posterized (3-bit luma, 2-bit chroma) |
| Mirror | S10: the ribbon field uses cx − \|x − cx\| |
| Wobble | P12 WOBBLE: amplitude 0 → ±1023 virtual px |

Ribbon pitch vw/N from a 16-entry ROM (1920 and 1440 virtual widths); width ladder 16..192 with a reciprocal ROM
(no divides). Source Video / Osc on S11. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Ribbons | 1–8 ribbons |
| K2 Width | ribbon width (8 steps, 16–192 virtual px) |
| K3 Pitch | wobble wavelength |
| K4 Blocks | block size |
| K5 Hue | ribbon palette rotation (8 colour pairs) |
| K6 Drift | the wobble travels, bipolar |
| S7 Ribbons | Flat / Gradient |
| S8 Ground | Blocks / Source |
| S9 Dashes | CMY dashes on / off |
| S10 Mirror | ribbon field mirrored |
| S11 Source | Video / Osc |
| **P12 Wobble** | straight ribbons → wild S-curves |

## Status

- v0.1.0 (2026-10-20): first build missed HD timing at 47 MHz (the per-line ribbon sequencer did a 10×7 product
  plus three adds and a variable-index write in one clock); split into three stages → built, 6/6 (HD HDMI seed 3, others seed 1), HD 77.4–83.9 MHz, 5.39k LC (70 %), 4 EBR. **Not yet flashed.**
