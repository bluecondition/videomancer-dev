# Terrace — Heirloom #11

**Source photo**: `photos/WIN_20230305_23_38_39_Pro.jpg` (1920×1080 capture, 2023-03-05)

## Breakdown of the photo

- **Composition**: three panels across, each a grid of wide cells (~8 columns × 17 rows per panel) in which every
  column drops the row sequence by one cell, so the colours run as diagonal staircases.
- **Sequence**: yellow, magenta, cyan, white, light gray, gray, dark gray, black — an 8-step palette cycling down
  the diagonals; each panel is dropped one cell relative to the last.
- **Character**: hard-edged, flat cells; the signature of a quantized diagonal ramp through a palette.

## Design

| Photo element | Program |
|---|---|
| Cell grid | cells K1 px wide (real pixels, a count) × K2 virtual lines tall |
| Diagonal staircase | colour index = (row·256 + column-phase + scroll) >> 8 mod 8; the column phase steps by the P12 slope (Q.8) at every cell column |
| Three panels, each dropped one cell | K5 panels (1 / 2 / 4); the panel index adds one row to the phase |
| 8-colour sequence | 4 palettes (K3): CMYK (photo), Warm, Cool, Neon; S7 swaps the halves |
| The picture | sampled as a mosaic on the same cells (S&H across, row-held via the line buffer) and mixed into the cell colour by K6 (S10 luma-only / full colour) |

Video on S11 Source (Osc = oscillator texture). Generated avid; output conditioning; blanking gate. Latency 16.

## Controls

| Control | Function |
|---|---|
| K1 Width | cell width 16–271 px |
| K2 Height | cell height 8–263 lines (1080-line canvas) |
| K3 Palette | CMYK / Warm / Cool / Neon |
| K4 Scroll | pattern scroll, bipolar |
| K5 Panels | 1 / 2 / 4 |
| K6 Mix | picture mixed into the cells, 0–100 % |
| S7 Swap | palette halves exchanged |
| S8 Grid | dark grid lines |
| S9 Direction | staircases descend right / left |
| S10 Tint | Colour / Luma |
| S11 Source | Video / Osc |
| **P12 Slope** | row step per column: flat bands → steep staircases |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.5–80.5 MHz, 4.1k LC (53 %), 15 EBR. Not flashed.
