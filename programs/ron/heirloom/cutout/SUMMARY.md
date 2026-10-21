# Cutout — Heirloom #55

**Source**: `photos/tezcatlipoca.jpg` (a paper collage, 4776×5987 scan — not synth output)

## Breakdown of the collage

- A flat, brushed cadmium-yellow ground filling the whole sheet.
- A head in profile (a skull-like face turned to the right, cranium at the top, jaw at the bottom) cut out of
  the yellow and filled with torn magazine pieces: blues, teals and near-blacks (a plate of food, a cup, cars,
  skies, the word "Love.") — the pieces overlap with ragged torn edges.

## Design

| Collage element | Program |
|---|---|
| Yellow ground | flat yellow, K4 Cream mixes it toward cream |
| The head cut-out | a 36-row profile ROM (half width + centre offset per row; head or oval by S7) read per line, linearly interpolated between rows and scaled by P12 Head (25–125 % of the screen height) on the serial unit (4 products a line, a divide per field for the rows-per-line step); K3 moves it, K6 sways it; S10 draws a black outline |
| Torn magazine pieces | the video posterized to 3–4 tones (K2 Levels) in a blue/teal/navy palette (S9: a warm palette), on a hashed piece grid (K5 size) that offsets each piece's luma, with horizontal tears in 16-line bands (K1 Tear); S8 Plain shows the picture itself inside the head |

Video on **S11 Source** (Video / Osc). Virtual 1080-line geometry (the head keeps its shape at SD);
generated avid; output conditioning before the blanking gate. Latency 20 clocks. EBR 16. Colours in the
hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Tear | sideways tearing in 16-line bands |
| K2 Levels | posterize threshold spread |
| K3 Centre | head position left → right |
| K4 Cream | ground yellow → cream |
| K5 Pieces | piece size, 32–256 px |
| K6 Sway | the head sways, bipolar rate |
| S7 Profile | Head / Oval |
| S8 Pieces | Torn / Plain |
| S9 Palette | Blue / Warm |
| S10 Outline | On / Off |
| S11 Source | Video / Osc |
| **P12 Head** | size, 25–125 % |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 79.8 / 76.1 / 82.9 MHz (Analog / HDMI / Dual), 4.43k LC (58 %), 15 EBR. **Not yet flashed.**
