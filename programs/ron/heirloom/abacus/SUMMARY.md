# Abacus — Heirloom #42

**Source photo**: `photos/WIN_20230605_01_03_50_Pro.jpg` (1920×1080 capture, 2023-06-05)

## Breakdown of the photo

- **Composition**: horizontal rows of glossy red ovals (beads) on a near-black ground, with rows of smaller
  blue and green blobs in the bands between; the picture is mirrored top to bottom.
- **Shading**: every blob is lit from above — a bright rim on top fading to a dark bottom — the look of a
  differentiated / relief-shaded oscillator product, with a thin chrome edge where the bead meets the ground.
- **Colour**: saturated red beads, cobalt blue and green blobs, yellow and white highlights.

## Design

| Photo element | Program |
|---|---|
| Bead rows | N cells across (K1) × M rows (K2); cell width from a serial divide, row height 1080/M |
| Beads | r² = dx² + dy² from the cell centre < R²; R = P12 × half the smaller cell dimension (serial products) |
| Lit from above | bead luma = picture luma + 192 − K3·dy/256 (dy signed, per line) |
| Row colours | 8-entry palette per row (K4 rotates; S8 RGB / Warm cycle); S7 mirrors the row index about the middle row |
| Chrome rim | white where R² − r² < K6·R²/256 (S9) |
| Tubes | S10 drops the dx term: horizontal tubes |
| Motion | K5 Drift slides the cells sideways (the first partial cell gets its own start constants) |
| The picture | the bead luma is the picture's (Source Osc swaps in the oscillator) |

Generated avid; output conditioning before the blanking gate. Latency 16 clocks, all modes.
Only the luma is read from the line buffer (5 EBR). Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Cells | beads across, 1–16 |
| K2 Rows | bead rows, 1–8 |
| K3 Shade | lighting gradient |
| K4 Hue | rotates the row colours (8 steps) |
| K5 Drift | beads slide sideways, bipolar |
| K6 Rim | rim width |
| S7 Mirror | row colours mirror about the middle row |
| S8 Palette | RGB cycle / Warm cycle |
| S9 Rim | Off / On |
| S10 Shape | Beads / Tubes |
| S11 Source | Video / Osc |
| **P12 Beads** | bead size: dots → beads that merge into tubes |

Resolution: counts stay counts (cells narrow on 4:3 SD); bead geometry in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.6–84.4 MHz, 4.2k LC (55 %), 5 EBR. **Not yet flashed.**
