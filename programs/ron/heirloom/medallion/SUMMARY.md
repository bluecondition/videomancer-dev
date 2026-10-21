# Medallion — Heirloom #12

**Source photo**: `photos/WIN_20230305_23_51_38_Pro.jpg` (1920×1080 capture, 2023-03-05)

## Breakdown of the photo

- **Composition**: a field of horizontal stripes in orange, yellow and red, folded into two nested chevron
  systems — V's with their apexes near the left and right edges, stacked down the screen — so every stripe bends
  twice. A ramp oscillator added to a rectified horizontal ramp.
- **Centre**: a hard-edged circle, a porthole, showing a different texture: a CMY / gray plaid of hard rectangles
  (cyan, magenta, yellow, white, black, gray) — a second patch keyed through a circle wipe.
- **Palette**: warm (dark red, orange, yellow) outside; CMY primaries inside.

## Design

| Photo element | Program |
|---|---|
| Stripes folded into two chevron systems | stripe phase = line phase + 2·slope·dist(x, nearest centre), accumulated per pixel with a direction flag that flips at c1, the midpoint and c2 |
| Warm stripe palette | 8-band palette (K4: Warm / Cool / Rainbow), S8 4 or 8 bands |
| Circle porthole | r² = dx² (second differences) + dy² (serial per line) ≤ R² (serial per frame); P12 sets R |
| CMY plaid inside | the video posterized hard (8 lumas, 1-bit chroma) — S7 Soft gives 4×4 chroma; Source = Osc substitutes a generated plaid of square waves |
| Chevron centres | K3 Spread moves c1 / c2 from the edges to the middle; S9 Point flips the V direction |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks. Colours in the hardware
convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Pitch | stripe period |
| K2 Slope | chevron slope |
| K3 Spread | chevron centres from the edges to the middle |
| K4 Palette | Warm / Cool / Rainbow |
| K5 Offset | circle x position, bipolar |
| K6 Speed | stripe scroll, bipolar |
| S7 Plaid | Hard / Soft posterize inside the circle |
| S8 Bands | 8 / 4 stripe levels |
| S9 Point | chevrons outward / inward |
| S10 Ring | white rim on the circle |
| S11 Source | Video / Osc (plaid oscillator; fully generative) |
| **P12 Medallion** | circle radius: a coin → the whole screen |

Resolution: all geometry in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 80.3–87.1 MHz, 4.1k LC (54 %), 9 EBR (the line buffer is pruned to the 9 posterize bits). **Not yet flashed.**
