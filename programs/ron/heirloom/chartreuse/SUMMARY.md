# Chartreuse — Heirloom #51

**Source photo**: `photos/WIN_20231211_00_38_58_Pro.jpg` (1920×1080 capture, 2023-12-11)

## Breakdown of the photo

- **Field**: a single-hue (lime / green / white) field of smooth glossy folds — vertical ribbons bent into S-curves,
  shaded like chrome with thin bright highlight lines along the folds.
- **Orb**: a soft bright orb at the left.
- **Palette**: monochrome green with white highlights.

## Design

| Photo element | Program |
|---|---|
| Fold field | tri(x·K1 + bend·tri(y)) + a slower second fold, written to the line buffer as the Osc source; Source = Video uses the video luma |
| Bend | P12 = the bend depth |
| Chrome shading | luma = 64 + 3·level; thin white lines where the level's low bits fall under K2 (S7) |
| Orb | r² falloff from a point at the left (dx² by second differences, dy² per line), K4 size (S8) |
| Hue | K3 picks one of 8 hues; chroma scaled by the level; S9 Duo alternates with the field |
| Mirror | S10 left / right about the centre |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Folds | fold pitch |
| K2 Shine | highlight width |
| K3 Hue | hue |
| K4 Orb | orb size |
| K5 Drift | fold drift, bipolar |
| K6 Contrast | field contrast |
| S7 Shine | highlights Off / On |
| S8 Orb | Off / On |
| S9 Hue | Mono / Duo |
| S10 Mirror | Off / On |
| S11 Source | Video / Osc |
| **P12 Fold** | bend depth: straight ribbons → deep S-folds |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Dual seed 2, others seed 1), HD 77.8–84.1 MHz, 3.9k LC (51 %), 4 EBR (luma-only line buffer). **Not yet flashed.**
