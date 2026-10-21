# Vesica — Heirloom #13

**Source photo**: `photos/WIN_20230307_01_04_17_Pro.jpg` (1920×1080 capture, 2023-03-07)

## Breakdown of the photo

- **Ground**: pale green, crossed by a diamond lattice of pastel outlines — thin multi-band lines (blue / pink /
  yellow) running at ±45°, two crossed stripe oscillators drawn as thin keys.
- **Centre**: an orange vesica — the lens-shaped intersection of two circles, pointed left and right — filled with
  nested chevron-like rings following the lens shape; the circles' remaining parts show as orange crescents above
  and below it. Horizontal tearing on some rows.

## Design

| Photo element | Program |
|---|---|
| Pastel diamond lattice | two diagonal phase fields (px ± py), thin where the phase is below the line width, three pastel sub-bands inside each line; K5 drifts them |
| Vesica (eye) | two circles centred at 540 ∓ e (K4 Aspect) with radius R (P12), sharing one dx² second-difference accumulator; eye = inside both, crescent = inside exactly one |
| Nested rings in the eye | iso-lines of the lens shape: (R² − max(r1², r2²)) scaled by K3, three orange tones cycling; the video luma shifts the ring index (etch) |
| Crescents | red-orange, shaded by the other circle's distance; S9 hides them |
| Tearing | 8-line random read offsets of the video (K6) |

Generated avid; output conditioning before the gate. Latency 20 clocks. Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | lattice pitch |
| K2 Width | lattice line width |
| K3 Rings | ring density inside the eye |
| K4 Aspect | circle offset: round eye → thin lens |
| K5 Drift | lattice drift, bipolar |
| K6 Tear | video tearing in 8-line bands |
| S7 Lattice | on / off |
| S8 Eye | Rings / Solid |
| S9 Crescents | on / off |
| S10 Tint | lattice lines pastel / the video's chroma |
| S11 Source | Video / Osc |
| **P12 Eye** | opening of the vesica (never fully closed) |

## Status

- v0.1.0 (2026-10-20): built 6/6 — HD Analog seed 3 (76.6 MHz), SD HDMI seed 2, others seed 1; HD 76.3–87.8 MHz, 4.46k LC (58 %), 12 EBR. **Not yet flashed.**
