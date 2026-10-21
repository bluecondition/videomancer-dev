# Starburst — Heirloom #3

**Source photo**: `photos/WIN_20221122_23_37_36_Pro.jpg` (1920×1080 capture, 2022-11-22)

## Breakdown of the photo

- **Composition**: a four-fold mirror kaleidoscope. Big X shapes sit at (480,270), (1440,270), (960,540),
  (480,810), (1440,810), small diamonds between them — a checkerboard lattice of 480×270 mirror cells.
- **Field**: the bands are level curves of |x| − |y| (a saddle): near zero they draw the X diagonals with nested
  gray V's inside the arms; further out along x they become "<" chevrons, along y "^" chevrons.
- **Bands**: hard-edged, posterized: white, light gray, gray, dark gray, black in the X arms; yellow, cyan,
  magenta, white, black further out — a CMYK-ish printer palette cycling with distance.
- **Edges**: every band edge is serrated with spiky horizontal teeth about 8 lines tall (a horizontal sawtooth
  added to the field at a short vertical period), plus a 1-px interlace comb.
- **Palette**: pure CMY + grays, maximal contrast.

## Design

| Photo element | Program |
|---|---|
| Mirror cells | N×N cell grid (K1), coordinates folded about each cell's centre; S7 Tile unfolds to sawtooth |
| Saddle field | s = a·\|x\| − b·\|y\| with a = tilt·zoom, b = (255−tilt)·zoom (per-frame products); S8 Diamond uses + |
| Posterized bands | band = s >> 14, palette[\|band\| mod 16] from 8 ROM palettes (K6) |
| Serrations | (line mod 8) × K3 added to the field per line |
| Band drift | phase added per field (K5) |
| Interlace comb / fringing | S9 Noise: LFSR grain added to the field |
| The picture | video mirrored horizontally in every cell (line-buffer read = cell start + min(u, cw−1−u)); its luma × P12 pushes the field, so the picture appears as warped contour bands; S11 takes the band chroma from the video |

Cell widths come from two serial divides per frame (w/N real px for the buffer, vw/N virtual for the field);
the per-line b·|y| product shares the serial unit. Generated avid; output conditioning before the gate.
Latency 20 clocks. Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Cells | cells across and down, 1–8 |
| K2 Spread | band zoom: a few wide bands → hundreds of fine ones |
| K3 Spikes | serration of the band edges |
| K4 Tilt | X opening angle |
| K5 Drift | bands flow outward / inward, bipolar |
| K6 Palette | CMYK / RGB / Warm / Cool / Mono / Neon / Pastel / Ember |
| S7 Lattice | Mirror / Tile |
| S8 Field | Saddle (X's and chevrons) / Diamond rings |
| S9 Noise | grain on the field |
| S10 Negative | palette inverted |
| S11 Chroma | Palette / Video |
| **P12 Etch** | video depth: 0 = the pure pattern → the picture carved in bands |

Resolution: the cell count is a COUNT (cells narrow on 4:3 SD); cell heights are 1080/N virtual lines.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 78.8–84.1 MHz (SD HDMI 61 MHz vs its 27 MHz Fmin), 4.45k LC (58 %), 17 EBR. **Not yet flashed.**
