# Magma — Heirloom #29

**Source photo**: `photos/WIN_20230515_00_46_28_Pro.jpg` (1920×1080 capture, 2023-05-15)

## Breakdown of the photo

- **Field**: a smooth, streaky cyan / magenta / blue field with yellow-cored blobs — the look of summed
  oscillators pushed through a colouriser, with horizontal streaking.
- **Red flats**: large flat red regions where the field saturates (comparator / clipped colouriser).
- **Mirror**: a horizontal seam through the middle where the pattern reflects.

## Design

| Photo element | Program |
|---|---|
| Smooth field | sum of three triangle waves (x, y, x + y; K3 scale; K5 drift) written to the line buffer as the Osc source; Source = Video uses the video luma instead |
| Streaking | IIR smear along the line (K1) |
| Colouring | 256-entry palette ROM (cyan → blue → magenta → yellow; S8 green / orange / violet), K2 hue offset + K5 cycling, K6 contrast; S10 posterizes |
| Red flats | field level above the threshold (P12) → flat red; S9 draws red only at the threshold edge |
| Mirror | left / right about K4 (S7) |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Streak | horizontal smear |
| K2 Hue | palette offset |
| K3 Blobs | field scale |
| K4 Axis | mirror axis |
| K5 Speed | drift / cycling, bipolar |
| K6 Contrast | field contrast |
| S7 Mirror | Off / On |
| S8 Palette | Cyan-magenta / Green-orange-violet |
| S9 Red | Flats / Outlines |
| S10 Poster | Off / On |
| S11 Source | Video / Osc |
| **P12 Lava** | red threshold: islands → a red sea |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (SD HDMI seed 2, others seed 1), HD 83.7–97.0 MHz, 3.2k LC (42 %), 9 EBR (yosys pruned the unused chroma bits of the line buffer). **Not yet flashed.**
