# Waterfall — Heirloom #45

**Source photo**: `photos/WIN_20230610_00_51_13_Pro.jpg` (1920×1080 capture, 2023-06-10)

## Breakdown of the photo

- **Composition**: pastel colour blocks (olive, cyan, green bands) top-left; a curtain of thin green vertical
  lines across the right two thirds that bend near the bottom and spill into orange / yellow ripples — a
  rainbow fan along the bottom edge.
- **Palette**: pastels above, saturated orange / yellow / red below, thin green lines.

## Design

| Photo element | Program |
|---|---|
| Curtain | lines where tri(x·K1 + bend + drift) < K2; bend = (y − fold)²·P12 (per-line square on the serial unit, scaled at the line start) below the fold at 640 virtual |
| Blocks | hashed cells (K3) through a pastel 8-entry palette; S8 Source posterizes the source luma through the same palette |
| Fan | below the fold (S9), stripes of tri(x·K4 + y) → orange / yellow / red / amber |
| Palette | S10 Pastel / Vivid (second half of the ROM) |

Source Video / Osc on S11. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Lines | curtain line pitch |
| K2 Width | line width |
| K3 Blocks | block size |
| K4 Fan | fan stripe pitch |
| K5 Hue | palette rotation |
| K6 Drift | curtain scroll, bipolar |
| S7 Curtain | on / off |
| S8 Ground | Blocks / Source |
| S9 Fan | on / off |
| S10 Palette | Pastel / Vivid |
| S11 Source | Video / Osc |
| **P12 Fall** | bend below the fold: straight → swept flat |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.4–84.4 MHz, 3.46k LC (45 %), 2 EBR. **Not yet flashed.**
