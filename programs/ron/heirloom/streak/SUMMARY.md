# Streak — Heirloom #7

**Source photo**: `photos/WIN_20230129_00_40_34_Pro.jpg` (1920×1080 capture, 2023-01-29)

## Breakdown of the photo

- **Texture**: everything is smeared horizontally into streaks — a leaky integrator along each line.
- **Bands**: the hue changes band by band down the screen (green / violet / orange / magenta / cyan), each band
  a few dozen lines tall, with hard horizontal seams.
- **Diagonal**: a boundary runs from lower-left to upper-right; below it the colour freezes into vertical
  magenta / cyan / blue barcode blocks (chroma sampled-and-held in column blocks).
- **Palette**: full rainbow, high saturation.

## Design

| Photo element | Program |
|---|---|
| Horizontal smear | first-order IIR along the line, leak 1/2 … 1/128 by P12 (0 = clean); S7 keeps chroma clean |
| Band hues | chroma rotated by band × K2 + spin (K5) with a 32-entry sin/cos table; K1 band height; S10 scrolls the bands |
| Diagonal + barcode | boundary x = vw/2 + slope·(y − 540) (K3), per-line DDA; on one side (S8) chroma is held per K4-wide column block |
| Contrast | K6 luma contrast; S9 replaces luma with a per-band horizontal ramp |
| Source | S11 Video / Osc |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Bands | band height |
| K2 Hue | hue step from band to band |
| K3 Slope | diagonal boundary, bipolar |
| K4 Blocks | barcode block width |
| K5 Spin | hue rotation speed, bipolar |
| K6 Contrast | luma contrast 0–200 % |
| S7 Smear | Colour / Luma only |
| S8 Barcode | Below / Above the diagonal |
| S9 Luma | Keep / Ramp |
| S10 Bands | Still / Scroll |
| S11 Source | Video / Osc |
| **P12 Smear** | 0 = clean → everything streaks to the right edge |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Dual seed 2, others seed 1), HD 74.7–80.6 MHz (HD HDMI 74.74 — tight), 5.0k LC (65 %), 15 EBR. **Not yet flashed.**
