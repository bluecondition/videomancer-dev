# Ticker — Heirloom #20

**Source photo**: `photos/WIN_20230319_01_27_56_Pro.jpg` (1920×1080 capture, 2023-03-19)

## Breakdown of the photo

- **Composition**: monochrome — columns of gray and white vertical bars of varying widths, shifted sideways per row
  band so they step and tear; cyan dashes scattered through the bars; a dark staircase region at the upper left.
- **Character**: a digital readout gone wrong — quantized grays, hard edges, a few saturated cyan accents.

## Design

| Photo element | Program |
|---|---|
| Bars of hashed widths in grays | block pitch K1, gray = hash(block, band, seed) quantized to K4 levels; S10 Columns makes the bars identical in every band |
| Per-band sideways shifts | hashed per-band offsets × P12 (odd bands the other way); S8 Static zeroes them |
| Cyan dashes | where a second hash passes the K3 density, in the middle half of a block and of its band; the picture's near-white parts also light as dashes; S7 white |
| The picture | read through the same offsets, luma quantized to the same levels, chroma killed, mixed with the bars by K6; S9 inverts |

Derived from marmalade's band / block engine. Video on S11 Source (Osc = oscillator texture). Generated avid;
output conditioning; blanking gate. Latency 16. The line buffer holds luma only (5 EBR).

## Controls

| Control | Function |
|---|---|
| K1 Bars | bar pitch 16 / 32 / 64 / 128 px |
| K2 Bands | band height 16–271 lines |
| K3 Dashes | dash density |
| K4 Levels | gray levels 2–64 |
| K5 Reshuffle | hash re-roll rate |
| K6 Bars | bars over the picture, 0–100 % |
| S7 Dashes | Cyan / White |
| S8 Bands | Slide / Static |
| S9 Invert | luma inverted |
| S10 Bars | Blocks / Columns |
| S11 Source | Video / Osc |
| **P12 Scramble** | sideways offset amount |

## Status

- v0.1.0 (2026-10-20): first build missed HD Dual (70.3 MHz on seed 6): the 7-operand block hash adder; fixed by accumulating bi·157 in the beam and band·61 + seed per line. Rebuilt: 6/6 (HD Analog seed 3, SD Analog / HD HDMI / HD Dual seed 2, router2 stalls on seed 1), HD 76.4–79.0 MHz, 3.4k LC (44 %), 5 EBR (luma-only buffer). Not flashed.
