# Kiln — Heirloom #32

**Source photo**: `photos/WIN_20230515_23_17_11_Pro.jpg` (1920×1080 capture, 2023-05-15)

## Breakdown of the photo

- **Colour**: a picture mapped through a fire ramp — blue-violet in the shadows, red through the mids, orange and
  yellow in the highs, with yellow blocks top and bottom.
- **Bands**: vertical columns glow brighter (a vertical oscillator added before the colouriser).
- **Texture**: horizontal streak smear throughout.

## Design

| Photo element | Program |
|---|---|
| Fire ramp | luma (K3 contrast) → 256-entry palette ROM (fire; S8 ice), K4 rotates it, S10 posterizes |
| Glowing columns | triangle bands (K1 pitch, K6 drift) × K5 depth added to the palette index (S7) |
| Streaks | IIR smear along the line (K2) |
| Heat | P12 pushes the whole index up the ramp toward yellow / white |
| Mirror | S9 left / right about the centre |
| Source | S11 Video / Osc |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Bands | band pitch |
| K2 Smear | horizontal smear |
| K3 Contrast | luma contrast |
| K4 Hue | palette rotation |
| K5 Depth | band glow depth |
| K6 Drift | band drift, bipolar |
| S7 Bands | Off / On |
| S8 Palette | Fire / Ice |
| S9 Mirror | Off / On |
| S10 Poster | Off / On |
| S11 Source | Video / Osc |
| **P12 Heat** | palette push: cool shadows → white hot |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Dual seed 2, others seed 1), HD 79.6–94.2 MHz, 3.2k LC (42 %), 9 EBR. **Not yet flashed.**
