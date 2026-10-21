# Idol — Heirloom #31

**Source photo**: `photos/WIN_20230515_23_14_37_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: a flat yellow ground. Down the centre a column of blue / orange / red interlocking bulbous
  shapes like stacked masks, mirrored left/right, smeared horizontally — a relief-shaded picture (every edge a
  blue or orange rim).
- **Palette**: yellow, blue, orange, red, white highlights.

## Design

| Photo element | Program |
|---|---|
| Mirrored column | the line buffer read mirrored about the centre (S7) inside \|dx\| < P12 width; the column sits at K5 and sways with K6 |
| Smear | one-pole IIR along the line on the source luma (K1) |
| Relief | derivative over 3 px: rising → colour A, falling → colour B (K3 picks the pair), luma 300 + \|d\| × K2 |
| Flat alternative | S8: the posterized picture (K4 depth) with its own chroma |
| Sides | S10: flat yellow / the plain picture |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Smear | horizontal smear |
| K2 Relief | relief gain |
| K3 Palette | Blue Orange / Green Magenta / Cyan Red / Violet Yellow |
| K4 Poster | posterize depth (Flat mode) |
| K5 Centre | column position |
| K6 Sway | column sways |
| S7 Mirror | on/off |
| S8 Shading | Relief / Flat |
| S9 Lit | Left / Right |
| S10 Sides | Yellow / Picture |
| S11 Source | Video / Osc |
| **P12 Column** | column width: a sliver → the whole screen |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 76.4–89.1 MHz, 3.7k LC (48 %), 15 EBR. **Not yet flashed.**
