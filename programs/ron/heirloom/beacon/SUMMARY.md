# Beacon — Heirloom #26

**Source photo**: `photos/WIN_20230424_23_53_49_Pro.jpg` (1920×1080 capture, 2023-04-24)

## Breakdown of the photo

- Black and ultramarine blocks (a coarse grid), rows of horizontal dashes tearing into them left and right.
- In the centre a white flame: a soft glowing head above a bright cone widening to the bottom.

## Design

| Photo element | Program |
|---|---|
| Cone | \|dx\| < (y − tip)·K4; tip row from P12 (900 → 120) |
| Glow | r² to a centre K5 above the tip, level = (R² − r²) shifted by the radius class, flickered per field (K6) |
| Blocks | hash of (block column, block row, field seed) → blue / black; blue luma = (K1 + source luma)/2 |
| Dashes | 8-line rows whose hash is under K3 get the block grid shifted sideways |
| Flame colour | White / Warm (chroma pulled toward the flame colour by the glow) |

Generated avid; output conditioning; latency 20; 16 EBR line buffer.

## Controls

| Control | Function |
|---|---|
| K1 Blue | level of the blue blocks |
| K2 Blocks | 16 / 32 / 64 / 128 |
| K3 Dashes | how many rows tear |
| K4 Cone | cone width |
| K5 Glow | glow radius |
| K6 Flicker | per-field flicker |
| S7 Blocks | On / Off |
| S8 Dashes | On / Off |
| S9 Flame | White / Warm |
| S10 Cone | Solid / Filled with the source |
| S11 Source | Video / Osc |
| **P12 Flame** | flame height: a stub → a spire to the top |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Analog seed 2, others seed 1), HD 80.4–83.7 MHz, 5.0k LC (65 %), 15 EBR. **Not yet flashed.**
