# Greenroom — Heirloom #54

**Source**: `photos/television.jpg` (a painted paper collage, 4786×5993 scan — not synth output)

## Breakdown of the collage

- Green and teal paint washed and scraped over magazine cuttings; the paint pools dark in places and thins to
  show the paper beneath; orange paint splatters at the top-left and bottom.
- Objects half-buried in the wash: a clock face, a vase, a man's suit, a strip of text ("picture yourself on
  this page"), spirals.
- In the middle a portable television set (dark bezel, dials at the side) showing a smiling woman's face —
  the one clean, bright image in the picture.

## Design

| Collage element | Program |
|---|---|
| Green / teal wash | a painterly field: three triangle-wave octaves (chevron, diagonal, crossed; K2 scale, K3 contrast, K5 drift) summed and mapped through a 256-entry green/teal palette ROM; S9 swaps a blue/violet palette |
| Orange splatters | where the field index passes K1 Splatter → orange (S8 keeps the palette colour instead) |
| The television | a rounded rectangle (radius-48 corners from a 64-entry quarter-circle ROM) with a dark bezel (K4 width) at K6 position; P12 Tune sets its size from a small set to the whole screen |
| The face on the screen | the video (or oscillator) inside the window, washed toward teal (S10 Wash) or clean |
| S7 Full | the video everywhere with the paint splattered over it (the field above the splatter threshold paints over the picture) |

Video on **S11 Source** (Video / Osc). Virtual 1080-line geometry; generated avid; output conditioning
before the blanking gate. Latency 20 clocks. EBR: 16 line buffer + 4 palette. Colours in the hardware
convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Splatter | how much of the field turns orange |
| K2 Scale | brush scale |
| K3 Contrast | field contrast |
| K4 Bezel | bezel width, 0–127 px |
| K5 Drift | field drift, bipolar |
| K6 Position | window left → right |
| S7 Window | TV / Full |
| S8 Splatter | Orange / Palette |
| S9 Palette | Green / Blue |
| S10 Wash | Green / Clean |
| S11 Source | Video / Osc |
| **P12 Tune** | window size |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (SD Analog on seed 2, all others seed 1), HD 88.6 / 91.2 / 88.6 MHz (Analog / HDMI / Dual), 4.18k LC (54 %), 19 EBR. **Not yet flashed.**
