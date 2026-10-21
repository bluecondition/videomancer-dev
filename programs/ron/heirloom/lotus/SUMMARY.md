# Lotus — Heirloom #33

**Source photo**: `photos/WIN_20230515_23_23_36_Pro.jpg` (1920×1080 capture, 2023-05-15)

## Breakdown of the photo

- **Two zones** split by a horizontal seam a little below the middle, both tiled two across.
- **Above**: nested rounded diamonds in orange and green bands with yellow-green edges — a rectified ramp pair
  quantized into rings, two cells across.
- **Below**: soft pink-white lobes (rounded squares with white cores) on an acid-green ground — a radial field
  keyed at two thresholds. Row tears along the seam and in the lobes.

## Design

| Photo element | Program |
|---|---|
| Two cells across | K1 Tiles 1..4; cell widths from serial divides; the video mirrored per cell |
| Diamond rings above | s = a·(\|x\| + \|y − ctop\|) (K2 zoom), ring index s >> 14 → 4 palette bands; S9 Soft shades each ring; the video luma etches s |
| Lobes below | r² = dx² (second differences restarted per cell) + (y − cbot)²; white core / pink body / green ground by two thresholds from K3; the luma swells the lobes |
| The seam | P12 sets the seam line (all lobes → all diamonds); S7 swaps the zones; S10 draws dark grid lines on the cell edges and the seam |
| Hue sets | K4: Orange Green / Blue Pink / Red Cyan / Mono (32-entry palette); S8 inverts the band order |
| Tears | 8-line random read offsets (K6) |

Generated avid; output conditioning before the gate. Latency 20 clocks. Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Tiles | cells across 1–4 |
| K2 Rings | diamond ring density |
| K3 Lobes | lobe size |
| K4 Hue | palette set |
| K5 Drift | rings flow, bipolar |
| K6 Tear | tear bands |
| S7 Zones | Diamonds up / Lobes up |
| S8 Invert | palette inverted |
| S9 Rings | Hard / Soft |
| S10 Grid | cell / seam lines |
| S11 Source | Video / Osc |
| **P12 Seam** | all lobes → all diamonds |

## Status

- v0.1.0 (2026-10-20): built 6/6 — HD Analog seed 2 (76.1), HD HDMI seed 6 (76.3), HD Dual seed 1 (74.6, marginal); 4.42k LC (58 %), 5 EBR. **Not yet flashed.** HD margins are thin (the 13×8 field multiply + the lobe compare chain); try router1 if a config misbehaves.
