# Sinew — Heirloom #18

**Source photo**: `photos/WIN_20230315_00_53_56_Pro.jpg` (1920×1080 capture, 2023-03-15)

## Breakdown of the photo

- Vertical bars in yellow / green / orange at the top, pink / blue / cyan at the bottom.
- Across the middle band the same bars are wrung into S-curves: every line is shifted sideways by a
  vertical sine, with a fine per-line ripple riding on top — a horizontal displacement driven by a
  vertical oscillator, gated to a band.

## Design

| Photo element | Program |
|---|---|
| Sideways wring | each line's read pointer starts at −A·sin(y) (64-entry sine, per-line product) |
| Only the middle band | \|y − 540\| < K2 (S7 All: everywhere) |
| Row ripple | per-line LFSR × K3 added to the shift |
| Combs | S10 Fold: the shift changes sign every 8 lines |
| Warm bars | the oscillator substitute: vertical bars in yellow / green / orange / red with a triangle shading |
| Colour | hue rotation (16-step cos/sin table, 4 multiplies), luma contrast gain, warm tint |

Generated avid; output conditioning before the blanking gate. Latency 16 clocks. 15 EBR.

## Controls

| Control | Function |
|---|---|
| K1 Period | vertical period of the sine |
| K2 Band | height of the wrung band |
| K3 Ripple | per-line ripple |
| K4 Hue | hue rotation |
| K5 Speed | sine travel, bipolar |
| K6 Contrast | luma gain 0.25–3.25× |
| S7 Band | Band / All |
| S8 Ripple | Off / On |
| S9 Tint | warm cast |
| S10 Fold | sign folds every 8 lines |
| S11 Source | Video / Osc |
| **P12 Wring** | sine amplitude: flat → ±500 px |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Dual seed 3), HD 74.4–75.8 MHz (margin thin: HD Analog 74.56, HD HDMI 74.35), 4.7k LC (61 %), 15 EBR. **Not yet flashed.**
