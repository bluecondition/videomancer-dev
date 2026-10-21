# Vase — Heirloom #27

**Source photo**: `photos/WIN_20230424_23_59_47_Pro.jpg` (1920×1080 capture, 2023-04-24)

## Breakdown of the photo

- A single tall vase / figure silhouette standing centre-right on black: a narrow neck, a bulge, a waist, a wide base — a slowly varying half-width down the screen.
- Inside it, vertical rainbow stripes (magenta / red / yellow / green / cyan) under a glossy shading: bright down the middle, dark toward both rims (a cosine-like gloss across the width).
- Rows tear sideways here and there (the shape shifts by a few pixels for a band of lines).

## Design

| Photo element | Program |
|---|---|
| Silhouette | per-line half-width = 48 + profile × P12, profile = sum of two triangle waves of y (K4 period, K5 drift) |
| Rainbow stripes | x-phase palette ROM (rainbow / warm), luma averaged with the source so the picture shows through |
| Squeezed picture (Fill = Picture) | read address = centre ± t·cx/256 where t = across-shape position from a per-line 65536/half-width divide |
| Glossy shading | luma × (1 − t·K2/256) toward the rims |
| Row tears | per-8-line random centre offset × K6 |
| Two vases (S9) | the nearer of two centres ± vw/4 |

Generated avid; output conditioning before the gate; latency 20; 16 EBR line buffer + 4 EBR palette.

## Controls

| Control | Function |
|---|---|
| K1 Stripes | stripe pitch 1–64 |
| K2 Shading | rim darkening 0–100 % |
| K3 Hue | palette offset |
| K4 Period | profile waves down the screen 1–16 |
| K5 Drift | profile drift, bipolar |
| K6 Tear | sideways row tears |
| S7 Fill | Stripes / Picture (squeezed into the shape) |
| S8 Shading | off / on |
| S9 Vases | one / two |
| S10 Palette | Rainbow / Warm |
| S11 Source | Video / Osc |
| **P12 Waist** | profile amplitude: a thin column → bulbous, screen-filling |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 2, others seed 1), HD 77.1–82.9 MHz, 4.6k LC (60 %), 19 EBR. Two builds missed HD (66–69 MHz) on the per-line profile/width product chains; both pipelined. **Not yet flashed.**
