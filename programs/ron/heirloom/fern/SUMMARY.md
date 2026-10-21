# Fern — Heirloom #46

**Source photo**: `photos/WIN_20230713_00_17_11_Pro.jpg` (1920×1080 capture, 2023-07-13)

## Breakdown of the photo

- **Composition**: mirrored about the centre line, nested pink / orange / yellow arcs pointing right (parabolas
  opening to the right) like fronds, over a fine teal / green ripple field; a flat blue zone at the left with a
  small chevron.
- **Palette**: flame colours (pink, orange, yellow) on teal / green, ultramarine flat.

## Design

| Photo element | Program |
|---|---|
| Fronds | field f = x − a·(y − cy)² + drift (per-line square on the serial unit, × P12 at the line start); band index = f >> K1, band fraction < K2 → an arc |
| Arc colours | 8-entry flame palette by band index + K4 (S8 Ice swaps the palette) |
| Ripple field | tri(x·K3 + tri(y)) in teal between the arcs; S9 Source shows the source luma tinted teal and widens the arcs by its luma |
| Blue zone | x < K5 flat blue (S10) |
| Mirror | S7: \|y − cy\| (centre) or y (parabolas from the top) |
| Curvature | P12 REACH: a = p12 (0 = vertical bars) |

Source Video / Osc on S11. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | arc spacing (8 octaves) |
| K2 Duty | arc width within the pitch |
| K3 Ripple | ripple pitch |
| K4 Hue | palette rotation |
| K5 Edge | blue zone boundary |
| K6 Drift | arcs flow, bipolar |
| S7 Mirror | Top / Centre |
| S8 Palette | Flame / Ice |
| S9 Field | Ripple / Source |
| S10 Blue | on / off |
| S11 Source | Video / Osc |
| **P12 Reach** | vertical bars → tight nested parabolas |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 75.9–81.2 MHz, 3.75k LC (49 %), 5 EBR. **Not yet flashed.**
