# Spearhead — Heirloom #5

**Source photo**: `photos/WIN_20230103_01_29_30_Pro.jpg` (1920×1080 capture, 2023-01-03)

## Breakdown of the photo

- **Composition**: two large spearheads (bullet / teardrop shapes pointing down) side by side, each a stack of
  nested outlines — black, magenta, yellow, blue, white — with stripes inside; the two copies are a 2× horizontal
  tiling of one shape.
- **Backdrop**: a coarse pastel mosaic — big rectangular blocks in diagonal colour bands (green, pink, lavender,
  blue, purple): a sampled-and-held, row-held picture under a diagonal colour ramp.
- **Palette**: pastel mosaic, saturated outline colours, hard edges everywhere.

## Design

| Photo element | Program |
|---|---|
| Spearhead shape | parabolic field f = dx²/2^K2 + (y − y0) < 0 inside each of 1 / 2 / 4 cells (K1); dx² by second differences per cell |
| Nested outlines | ring index = (−f) × K3 >> 11, 8-colour outline palette (S9: photo palette / cool) |
| Two copies | cells across (K1 count: 1 / 2 / 4) |
| Pastel mosaic | video sampled-and-held per cell across, rows held via the line buffer (written only on a cell row's first line), K4 cell size; washed toward pastel white (K5); S10 adds a diagonal pastel band tint |
| Spearheads descending | P12 Plunge moves the tip from the top of the screen to past the bottom; K6 bobs them (triangle LFO); S7 points them up |

Video on S11 Source (Osc = oscillator texture). Generated avid; output conditioning; blanking gate. Latency 16.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Cells | spearheads across: 1 / 2 / 4 |
| K2 Width | parabola opening, narrow → wide |
| K3 Rings | outline ring density |
| K4 Mosaic | mosaic cell size 8–263 px (square at every mode) |
| K5 Wash | pastel wash of the backdrop |
| K6 Bob | tip bobbing amplitude |
| S7 Point | Down / Up |
| S8 Mosaic | On / Off (plain picture behind) |
| S9 Palette | A (photo) / B (cool) |
| S10 Bands | diagonal pastel band tint |
| S11 Source | Video / Osc |
| **P12 Plunge** | tip position: small tips at the top → giant rings past the bottom |

Resolution: cells are counts (narrow on 4:3 SD); mosaic cells and the parabola run in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 (SD HDMI seed 2, others seed 1), HD 75.4–83.4 MHz, 4.87k LC (63 %), 15 EBR. Not flashed.
