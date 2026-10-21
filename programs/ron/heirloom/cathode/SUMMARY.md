# Cathode — Heirloom #23

**Source photo**: `photos/WIN_20230407_23_48_05_Pro.jpg` (1920×1080 capture, 2023-04-07)

## Breakdown of the photo

- **Composition**: a glitchy pixel mosaic in magenta, green, yellow and orange — coarse blocks, rows torn sideways
  — with two large purple parabolic "tube" shapes sweeping down through it, their edges soft and dark-rimmed.
- **Character**: four-colour posterization of a picture on a coarse grid; the tubes are generated shapes over it.

## Design

| Photo element | Program |
|---|---|
| Coarse mosaic | video sampled-and-held per cell across, rows held via the line buffer (K1 cell size, square); S8 Off = plain picture |
| Four-colour posterization | luma's top two bits index a 4-colour palette (K5: Cathode / Phosphor / Amber / Ice); S10 Off = full colour |
| Row tears | per-8-line random read offset × K3 (the mosaic cells tear with it) |
| Two purple tubes | \|(y − yv) − dx²/2^K2\| < width for two centres ±K4 from the middle; dx² by second differences; vertices sweep down with K6; S9 cyan; S7 off |
| Tube width | P12: hairlines → a purple flood |

Video on S11 Source (Osc = oscillator texture). Generated avid; output conditioning; blanking gate. Latency 16.

## Controls

| Control | Function |
|---|---|
| K1 Blocks | mosaic cell size 8–263 px |
| K2 Curve | parabola opening |
| K3 Tear | sideways tearing in 8-line bands |
| K4 Spacing | distance between the tubes |
| K5 Palette | Cathode / Phosphor / Amber / Ice |
| K6 Sweep | vertex sweep speed, bipolar |
| S7 Tubes | On / Off |
| S8 Mosaic | On / Off |
| S9 Tube | Purple / Cyan |
| S10 Poster | On / Off |
| S11 Source | Video / Osc |
| **P12 Tubes** | tube width: hairlines → flood |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 76.6–87.6 MHz, 4.3k LC (56 %), 15 EBR. Not flashed.
