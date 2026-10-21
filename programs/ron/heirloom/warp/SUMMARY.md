# Warp — Heirloom #48

**Source photo**: `photos/WIN_20230909_23_29_18_Pro.jpg` (1920×1080 capture, 2023-09-09)

## Breakdown of the photo

- An orange / green / violet stripe field bent into a large arc (the stripes curve about a point above
  the screen) and sheared sideways in horizontal blocks, so the lower half reads like a perspective fan.
- The left edge fades to black. A stripe oscillator whose phase carries a squared-radius term, with a
  sample-and-hold horizontal shear.

## Design

| Photo element | Program |
|---|---|
| Stripes | phase = a·x + b·y (accumulators) |
| Arc | + c·r² with r² from dx² (second differences per pixel) + dy² (serial per line), c from P12 |
| Block shear | bands of K3 lines each add a random phase offset and shift the source read (K4, S7) |
| Colours | trio orange / green / violet from the phase, or a rainbow ROM (S8), then a hue rotation (K5) |
| Left fade | dx thresholds at 1/8 and 3/16 of the width → black / half luma |
| The picture | the source luma adds to the phase (half a period at white) |

Generated avid; output conditioning before the blanking gate; virtual 1080 geometry. Latency 20.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | stripe pitch |
| K2 Angle | diagonal angle |
| K3 Block | block height |
| K4 Shear | per-block shear |
| K5 Hue | hue rotation |
| K6 Drift | stripes flow, bipolar |
| S7 Shear | Off / On |
| S8 Palette | Trio / Rainbow |
| S9 Arcs | Convex / Concave |
| S10 Mirror | x term mirrored about the centre |
| S11 Source | Video / Osc |
| **P12 Curve** | r² weight: straight stripes → tight arcs |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 3), HD 74.9–81.0 MHz (thin margins on HD Analog / HD HDMI), 5.1k LC (66 %), 6 EBR. **Not yet flashed.**
