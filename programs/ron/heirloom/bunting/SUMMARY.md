# Bunting — Heirloom #35

**Source photo**: `photos/WIN_20230521_23_37_25_Pro.jpg` (1920×1080 capture, 2023-05-21)

## Breakdown of the photo

- **Composition**: a white ground; big yellow and cyan triangles — a fan of wedges radiating from the screen centre,
  mirrored top and bottom (two crossed diagonal boundaries); rows of stepped glyph blocks laid out four across and
  four down, red above the centre line, blue below, gray / black at the right.
- **Character**: flat flags and digital glyphs — sign tests of linear ramps plus a bitmap pattern on a cell grid.

## Design

| Photo element | Program |
|---|---|
| Wedge fan | L1 = a·dx + b·dy and L2 = a·dx − b·dy (accumulators; b·dy per line, a·dx per pixel); the two sign bits classify the sector; P12 Fan sets a : b |
| Yellow / cyan / white | K3 picks which sector classes are painted (K4 colour pair), the rest white; S7 mirrors top / bottom via \|dy\| |
| Stepped glyph blocks | a 4×4-cell staircase bitmap on a cell grid: K1 cell size, K2 column pitch, K5 row pitch; S9 drifts the columns one cell per field |
| Red above / blue below / gray right | glyph colour by screen half (S10 swaps) and the right quarter |
| The picture in the glyphs | the glyph cells carry the picture as a mosaic (S&H per cell, rows held), mixed in by K6 |

Video on S11 Source (Osc = oscillator texture). Generated avid; output conditioning; blanking gate. Latency 16.

## Controls

| Control | Function |
|---|---|
| K1 Cell | glyph cell size 8–135 px |
| K2 Columns | glyph column pitch 5–12 cells |
| K3 Wedges | painted sector classes (15 combinations) |
| K4 Pair | Yellow Cyan / Magenta Green / Red Blue / Orange Violet |
| K5 Rows | glyph row pitch 5–20 cells |
| K6 Picture | picture mixed into the glyph cells |
| S7 Mirror | wedges mirrored top / bottom |
| S8 Glyphs | On / Off |
| S9 Drift | glyph columns step every field |
| S10 Rows | Red over Blue / Blue over Red |
| S11 Source | Video / Osc |
| **P12 Fan** | fan angle: steep → flat |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.6–80.6 MHz, 4.55k LC (59 %), 15 EBR. Not flashed.
