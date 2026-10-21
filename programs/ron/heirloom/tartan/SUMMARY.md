# Tartan — Heirloom #25

**Source photo**: `photos/WIN_20230410_01_13_24_Pro.jpg` (1920×1080 capture, 2023-04-10)

## Breakdown of the photo

- A diagonal stripe field in cyan / green / orange / magenta / black.
- The field is sliced into horizontal bands of varying height; each band is slid sideways by its own
  amount and recoloured (the stripe colours rotate), and solid black bars cut across several bands.
- A ramp-oscillator stripe field passed through a sample-and-hold displacement keyed by a slow oscillator.

## Design

| Photo element | Program |
|---|---|
| Diagonal stripes | phase = a·x + b·y, 8 bands of the phase through an 8-entry palette (4 palettes) |
| Sliced bands | band height K3; each band adds a random (or stepped) phase offset × P12 |
| Recoloured bands | a random palette rotation per band (S8) |
| Black bars | a slow triangle of y with a 1/4 duty (S9) |
| The picture | the source luma below K5 paints black (the picture keys the field) |

Generated avid; output conditioning before the blanking gate. Latency 16 clocks. 5 EBR.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | stripe pitch |
| K2 Angle | diagonal angle, bipolar |
| K3 Band | band height |
| K4 Palette | Tartan / Neon / Earth / Mono |
| K5 Black | key level |
| K6 Speed | stripe scroll, bipolar |
| S7 Offsets | Random / Stepped |
| S8 Rotate | palette rotation per band |
| S9 Bars | black bars |
| S10 Flip | angle flipped |
| S11 Source | Video / Osc |
| **P12 Slice** | band offset amount: unbroken → shredded |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 75.0–82.2 MHz, 3.3k LC (43 %), 5 EBR. **Not yet flashed.**
