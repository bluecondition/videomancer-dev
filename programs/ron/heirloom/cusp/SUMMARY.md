# Cusp — Heirloom #17

**Source photo**: `photos/WIN_20230315_00_30_57_Pro.jpg` (1920×1080 capture, 2023-03-15)

## Breakdown of the photo

- Horizontal rainbow stripes (blue / cyan / green / magenta / red / yellow) are displaced vertically by a
  function of x that rises and falls three times across the screen, bending every stripe into a
  right-pointing cusp.
- Inside the deep cusps the stripes give way to dark navy; the pattern repeats in two rows top and bottom.
- A horizontal ramp quantised and folded (triangle) driving the vertical phase of a stripe oscillator.

## Design

| Photo element | Program |
|---|---|
| Rainbow stripes | phase = y·pitch (per-line accumulator) through a 256-entry rainbow ROM |
| Cusp bend | phase += tri(x·g) × A, A from P12 and a vertical envelope (K3 Rows) |
| Two pattern rows | the envelope = tri of a slow y ramp, 64–127 % of the bend |
| Navy inside the cusps | phase under the Dark width → navy; the source luma widens it where bright |
| Oscilloscope look | S9 Picture: the bend source is the source luma instead of the triangle |

Generated avid; output conditioning before the blanking gate; virtual 1080-line geometry. Latency 20.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | stripe pitch |
| K2 Cusps | cusps across the screen |
| K3 Rows | vertical envelope of the bend |
| K4 Hue | hue cycling speed |
| K5 Dark | navy band width |
| K6 Speed | stripe scroll, bipolar |
| S7 Cusps | Right / Left |
| S8 Palette | Rainbow / Duo |
| S9 Bend | Tri / Picture |
| S10 Comb | half-cusp offset on alternate lines |
| S11 Source | Video / Osc |
| **P12 Bend** | flat stripes → cusps four stripes deep |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (SD Analog seed 2), HD 85.1–95.8 MHz, 3.4k LC (45 %), 6 EBR. **Not yet flashed.**
