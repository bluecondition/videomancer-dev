# Curtain — Heirloom #34

**Source photo**: `photos/WIN_20230521_01_34_12_Pro.jpg` (1920×1080 capture, 2023-05-21)

## Breakdown of the photo

- **Drapes**: a red field hung with magenta vertical folds, each fold wrung into an S-curve — a vertical
  oscillator whose phase is modulated by a slow vertical sine, shaded light-to-dark across every fold.
- **Pools**: flat blue regions where the drapes part (a second slow oscillator keyed low).
- **Piping**: lime-green lines along the fold crests (the fold wave's peak keyed thin).
- **Tears**: rows displaced sideways in bands.

## Design

| Photo element | Program |
|---|---|
| Folds | t = tri(x·K1 + A·tri(y) + video luma); A from P12 |
| Shading | luma 280 + 3t (S10 Light inverts), chroma red → magenta by t (K4 picks 4 hue pairs) |
| Pools | a slow 2-D triangle field below K2 → flat blue (S7) |
| Piping | t within K3 of its peak → lime (S8) |
| Tears | 8-line random phase offsets (K6) |
| Mirror | S9 reverses the fold phase past the centre |
| The picture | the video luma is added to the fold phase, so the drapes follow it; Source = Osc substitutes the oscillator |

Generated avid; output conditioning before the gate. Latency 20 clocks. Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | fold pitch |
| K2 Pools | pool size |
| K3 Piping | crest width |
| K4 Hue | Red Magenta / Blue Cyan / Green Yellow / Violet Pink |
| K5 Drift | fold phase drift, bipolar |
| K6 Tear | tear bands |
| S7 Pools | on / off |
| S8 Piping | on / off |
| S9 Mirror | folds mirrored about the centre |
| S10 Shade | Dark / Light |
| S11 Source | Video / Osc |
| **P12 Drape** | the wring: straight folds → deep S-curves |

## Status

- v0.1.0 (2026-10-20): built 6/6 seed 1, HD 79.1–82.7 MHz, 3.45k LC (45 %), 4 EBR (only the luma of the buffer is read). **Not yet flashed.**
