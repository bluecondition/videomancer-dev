# Marmalade — Heirloom #14

**Source photo**: `photos/WIN_20230307_23_36_05_Pro.jpg` (1920×1080 capture, 2023-03-07)

## Breakdown of the photo

- **Composition**: a Mondrian-like field of orange, magenta, yellow, brown and dark-olive blocks — a vertical
  colour-block pattern cut into row bands of random heights, every band slid sideways by a different amount, so
  the blocks step and tear; slivers of cyan, white and pink appear at the tears.
- **Character**: flat saturated colours, hard edges, the signature of a sample-and-hold horizontal offset per band.

## Design

| Photo element | Program |
|---|---|
| Row bands of random heights | band DDA in virtual lines, height = K2 + hash(band)·K2/16 (S8 Even disables the variation) |
| Sideways slide per band | hashed per-band offset × P12, odd bands the other way (S7) — the line-buffer read pointer starts at the offset and wraps |
| Vertical colour blocks | block pitch K1 (16 / 32 / 64 / 128 px), colour = palette[hash(block, band, seed)] (K3: Marmalade / Mint / Berry / Ash) |
| The picture in the blocks | the video is read through the same band offsets (block-torn), its chroma held per block and its luma posterized to K5 levels (S10 Smooth off), and the block colour is painted over it by K6 |
| Reshuffle | the hash seed re-rolls at K4's rate |
| Outlines | S9 dark lines on block and band edges |

Video on S11 Source (Osc = oscillator texture). Generated avid; output conditioning; blanking gate. Latency 16.

## Controls

| Control | Function |
|---|---|
| K1 Blocks | block pitch 16 / 32 / 64 / 128 px |
| K2 Bands | band height 16–271 lines |
| K3 Palette | Marmalade / Mint / Berry / Ash |
| K4 Reshuffle | hash re-roll rate |
| K5 Levels | luma posterize levels 2–64 |
| K6 Paint | block colour over the picture, 0–100 % |
| S7 Slide | Alternate / Same direction |
| S8 Bands | Random / Even heights |
| S9 Outlines | dark edge lines |
| S10 Luma | Posterized / Smooth |
| S11 Source | Video / Osc |
| **P12 Slide** | sideways tear amount: aligned → shredded |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD Analog, SD Analog, HD HDMI seed 2 after router2 stalls on seed 1; others seed 1), HD 76.6–78.5 MHz, 4.44k LC (58 %), 15 EBR. Not flashed.
