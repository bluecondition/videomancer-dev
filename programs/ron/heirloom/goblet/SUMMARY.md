# Goblet — Heirloom #49

**Source photo**: `photos/WIN_20230916_01_08_00_Pro.jpg` (1920×1080 capture, 2023-09-16)

## Breakdown of the photo

- Vertical rainbow stripes (green / purple / red / cyan / yellow) fill the frame, mirrored left/right.
- A cyan glass goblet — bowl, stem, foot — is drawn over them as a smooth outline band with chrome highlights along the glass.

## Design

| Photo element | Program |
|---|---|
| Goblet profile | 64-row half-width table (bowl / stem / foot), linearly interpolated per line, wobbled by a triangle of y (K4), scaled by P12 |
| Outline band | \|\|dx\| − w(y)\| < K2 → cyan, luma 940 − distance·K6 (bright highlight at the band centre) |
| Filled goblet (S9) | inside tinted cyan |
| Rainbow stripes | x-phase palette ROM (rainbow / warm), mirrored about the axis (S7), luma averaged with the source |
| Picture (S8) | the source replaces the stripes under the glass |

Generated avid; output conditioning; latency 20; 16 EBR line buffer + 4 EBR palette.

## Controls

| Control | Function |
|---|---|
| K1 Stripes | stripe pitch |
| K2 Width | band width |
| K3 Hue | palette offset |
| K4 Wobble | profile wobble |
| K5 Drift | stripes scroll, bipolar |
| K6 Highlight | highlight brightness |
| S7 Mirror | stripes mirrored about the axis |
| S8 Fill | Stripes / Picture |
| S9 Goblet | Outline / Filled |
| S10 Palette | Rainbow / Warm |
| S11 Source | Video / Osc |
| **P12 Goblet** | profile scale: a tiny glass → wider than the screen |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.9–86.5 MHz, 4.3k LC (56 %), 19 EBR. **Not yet flashed.**
