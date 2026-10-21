# Shield — Heirloom #38

**Source photo**: `photos/WIN_20230529_22_56_41_Pro.jpg` (1920×1080 capture, 2023-05-29)

## Breakdown of the photo

- Two heraldic shields side by side, each mirror-symmetric: a rounded arch filled with orange / red horizontal stripes, and below it nested blue / red / black V chevrons.
- The cell edges fall off to dark.

## Design

| Photo element | Program |
|---|---|
| Cells | N across (K1), widths from per-frame serial divides; the picture mirrored per cell (S7) |
| Arch | dx² ≤ (y − 60)·gain, gain from P12 (squared), dx² by second differences restarting at every cell |
| Stripes | per-line parity of y·K2 → orange / red (or cyan / blue), luma averaged with the source; S9 Picture shows the source itself |
| Chevrons | (\|dx\| + y·K4) × K3 → 4-band palette blue / red / black / red, pointing down or up (S10), drifting (K5) |
| Edge darkening | luma × (1 − (\|dx\| − 3cw/8)·K6) |

Generated avid; output conditioning; latency 20; 16 EBR line buffer.

## Controls

| Control | Function |
|---|---|
| K1 Cells | 1–4 |
| K2 Stripes | stripe pitch |
| K3 Chevrons | band pitch |
| K4 Slope | chevron slope |
| K5 Drift | chevrons flow |
| K6 Edge | edge darkening |
| S7 Mirror | picture mirrored per cell |
| S8 Palette | Heraldic / Cool |
| S9 Fill | Stripes / Picture |
| S10 Chevrons | Down / Up |
| S11 Source | Video / Osc |
| **P12 Arch** | chevrons only → tall arches filling the cells |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Analog seed 2, others seed 1), HD 79.6–83.2 MHz, 4.9k LC (63 %), 15 EBR. First build missed HD at 68 MHz on a per-line chevron product chain; pipelined. **Not yet flashed.**
