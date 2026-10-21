# Seismic — Heirloom #44

**Source photo**: `photos/WIN_20230610_00_10_54_Pro.jpg` (1920×1080 capture, 2023-06-10)

## Breakdown of the photo

- **Composition**: horizontal rows of two kinds, alternating down the screen: soft rainbow smear bands
  (green / yellow / cyan / blue gradients streaked along the line) and rows of scalloped lenses — yellow,
  cyan and magenta blobs — on black.
- **Glow**: every row is brightest along its middle and fades to its edges; the lens rows read like a
  seismograph trace of blobs.
- Mechanism: a vertical square-wave selecting between two oscillator products, horizontally smeared.

## Design

| Photo element | Program |
|---|---|
| Rows | row pitch K1 (40–295 virtual lines); row parity picks the kind (S7 swaps) |
| Smear band | the picture (Source) one-pole smeared along the line (K4, 8 steps), luma × a row glow envelope, chroma from a rainbow that cycles across (K3) and drifts (K6) |
| Lens row | the same smeared luma inside lenses (r² < R², P12) on a cell pitch (K2, 2–33 across); black (or white, S10) elsewhere; S8 Bars drops the dy term |
| Glow | per-line envelope 255 − \|dy\|·K5/16 |
| Palette | Rainbow / CMY (2 × 256-entry ROMs) |

Generated avid; output conditioning before the blanking gate. Latency 16 clocks, all modes.
Only the luma is read from the line buffer (5 EBR + 4 palette). Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Rows | row pitch |
| K2 Lenses | lenses across, 2–33 |
| K3 Hue | rainbow cycles across the screen |
| K4 Smear | horizontal smear of the picture, 8 steps |
| K5 Glow | row glow: soft wide → tight bright middle |
| K6 Drift | rainbow drift, bipolar |
| S7 Rows | Smear first / Lenses first |
| S8 Lenses | Round / Bars |
| S9 Palette | Rainbow / CMY |
| S10 Ground | Black / White |
| S11 Source | Video / Osc |
| **P12 Blobs** | lens size: dots → lenses that merge into tubes |

Resolution: lens count is a COUNT; row pitch and lens geometry in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.1–85.4 MHz, 3.9k LC (51 %), 8 EBR. **Not yet flashed.**
