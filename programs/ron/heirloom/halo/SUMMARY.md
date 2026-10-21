# Halo — Heirloom #47

**Source photo**: `photos/WIN_20230801_00_19_00_Pro.jpg` (1920×1080 capture, 2023-08-01)

## Breakdown of the photo

- **Rings**: two glowing yellow-green ovals on a dark olive field, mirrored left/right — the glow is brightest
  on the ring and falls off both inward and outward (a rectified radial difference), rust-red inside the rings.
- **Chevrons**: nested blue / violet / cyan bands between the rings, pointed like diamonds at the cell seams, with
  luma gradients across each band.
- Small nested diamonds sit exactly on the seams; some rows are torn sideways at the left.

## Design

| Photo element | Program |
|---|---|
| Mirrored cells | K1 Cells 1..4 (serial divides), S10 adds as many rows; the video mirrored per cell (S9) |
| Glowing rings | r² per cell (dx² restarted per cell + per-line dy²); glow = sat(\|r² − R²\| × K2) → 64-step palette from the bright ring hue to dark olive (K4 picks Lime / Gold / Cyan / Magenta); S7 Hard draws a plain ring instead |
| Chevrons at the seams | a·dist(x, nearest seam) + b·\|y − row centre\| (K3), two alternating bands (blue / violet) with a luma gradient, drawn outside the ring zone (r² > 1.5 R²); S8 hides them |
| The picture | the video luma brightens the glow and shades the chevrons |
| Tears | 8-line random read offsets (K6) |

Generated avid; output conditioning before the gate. Latency 20 clocks. Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Cells | cells across 1–4 |
| K2 Width | glow width |
| K3 Chevrons | chevron density |
| K4 Hue | Lime / Gold / Cyan / Magenta |
| K5 Drift | chevrons flow, bipolar |
| K6 Tear | tear bands |
| S7 Rings | Glow / Hard |
| S8 Chevrons | on / off |
| S9 Mirror | video mirrored per cell / plain |
| S10 Rows | one row / a grid |
| S11 Source | Video / Osc |
| **P12 Ring** | ring radius: a dot → beyond the cell |

## Status

- v0.1.0 (2026-10-20): built 6/6 — HD HDMI seed 2 (75.5), HD Dual seed 1 (74.4, marginal), others seed 1; HD 74.4–81.4 MHz, 4.57k LC (59 %), 6 EBR. **Not yet flashed.**
