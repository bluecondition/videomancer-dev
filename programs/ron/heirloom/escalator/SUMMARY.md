# Escalator — Heirloom #1

**Source photo**: `photos/WIN_20220629_23_49_20_Pro.jpg` (1920×1080 capture, 2022-06-29)

## Breakdown of the photo

- **Composition**: the frame is a grid of about 7 columns × 4.3 rows of near-square tiles (~270 px wide, ~250 px
  tall). Every tile holds the same scaled-down picture: a lavender/periwinkle field with a soft magenta-white blob —
  a classic multiplier/tiler patch (the frame replicated by a divided ramp).
- **Row structure**: across the lower quarter of every tile row runs a pink horizontal band. Inside the band the
  same content turns vivid — yellow → orange → magenta gradient blobs — while outside it is a pale pastel wash.
  So the picture carries two renderings of itself, keyed by a horizontal ramp comparator.
- **Green staircases**: acid-green bands descend diagonally to the right across the entire grid, with stepped
  (sample-and-hold) edges about 25 px per step, one band per tile row, parallel. Where they cross the vivid band
  they darken to blue. Edges show an interlace comb.
- **Palette**: pastel lavender (desaturated blue-violet), pale pink, saturated green, bright yellow/orange/magenta,
  deep blue at overlaps. Overall light, high-key.

## Design

| Photo element | Program |
|---|---|
| Tiled copies of the picture | each line written decimated into a line buffer (DDA, fraction f) and read wrapped; P12 sets f continuously from 1 tile to 8 |
| Square tiles | tile row height = tile width in virtual 1080-line units (round at SD/720p/1080p) |
| Pink band with vivid content | lower K1 of each row: chroma gain K3 (0–400 %) + pink cast (Cr +64) |
| Pastel wash elsewhere | mix toward lavender (Y 747, Cr 516, Cb 594) by K2 |
| Green staircases | stepped diagonal ramp (steps = 1/8 tile, K5 rise per step), period = one tile row, thickness K4, marching with K6; S9 adds a mirrored family |
| Blue where stairs cross the band | S10 Tinted: the picture's luma under green chroma |
| Interlace comb on the stairs | S11 Comb: steps offset half a step on alternate lines |

The video enters from the line buffer (1-line old content, invisible); generated colours are authored in the
hardware convention (u = Cr, v = Cb). Generated avid (cubist/aurora recipe); output conditioning
(luma [1 2 1], chroma 9-tap, luma to ~777) before the blanking gate. Latency 16 clocks, all modes.

## Controls

| Control | Function |
|---|---|
| K1 Band | pink band height, 12–88 % of the tile row |
| K2 Wash | lavender wash outside the band, 0–100 % |
| K3 Vivid | chroma gain inside the band, 0–400 % |
| K4 Stairs | staircase thickness, 6–100 % of the row period |
| K5 Slope | rise per step, bipolar (centre = flat bands) |
| K6 Speed | staircase march, bipolar (centre = still) |
| S7 Brick | odd tile rows offset by half a tile |
| S8 Flip | odd tiles mirrored |
| S9 Twin | second staircase family, opposite slope |
| S10 Stairs | Solid green / Tinted (picture luma, green chroma) |
| S11 Comb | stair steps comb on alternate lines |
| **P12 Tiles** | 1 tile (the plain picture) → 8 tiles across, continuous |

Resolution: tile count is a COUNT (tiles narrow on 4:3 SD); rows are square in virtual units everywhere.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 75.1–79.4 MHz, 4.9k LC (64 %), 15 EBR. **Not yet flashed.**
