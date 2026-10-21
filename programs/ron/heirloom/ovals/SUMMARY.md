# Ovals — Heirloom #50

**Source photo**: `photos/WIN_20231003_23_01_50_Pro.jpg` (1920×1080 capture, 2023-10-03)

## Breakdown of the photo

- **Composition**: a hot-pink ground streaked horizontally, carrying a lattice of overlapping ovals two rows
  deep, the rows offset so the ovals interlock and small diamonds appear where their rims cross.
- **Lenses**: each oval runs from a white / yellow core through orange to a green, then blue, then purple rim.
- **Texture**: horizontal streaking everywhere (a smeared source), soft edges.

## Design

| Photo element | Program |
|---|---|
| Lattice | N cells across (K1, 2–17) × M rows (K2, 1–8); S7 offsets odd rows by half a cell |
| Lens colour | palette index = r² × zoom, saturating: white core → yellow → orange → green → blue rim (S8 Cool: cyan / blue / violet / magenta); S9 off stops the palette at the orange |
| Pink ground | beyond the rim (index saturated): hot pink, or the smeared picture (S10) |
| Streaks | one-pole smear of the picture along the line (K4) |
| Lens shape | K6 Aspect scales dy² (wide → tall); P12 LENS sets the zoom (separate ovals → an overlapping lattice) |
| The picture | its smeared luma fills the lens cores (K3 sets how much of the lens is core) |
| Motion | K5 Drift slides even rows sideways |

Generated avid; output conditioning before the blanking gate. Latency 16 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Cells | lenses across, 2–17 |
| K2 Rows | lens rows, 1–8 |
| K3 Core | how much of each lens is the picture's core |
| K4 Smear | horizontal streak of the picture, 8 steps |
| K5 Drift | lenses slide sideways, bipolar |
| K6 Aspect | lens shape: wide → tall |
| S7 Offset | odd rows offset by half a cell |
| S8 Palette | Warm / Cool |
| S9 Rim | Off / On |
| S10 Ground | Pink / Picture |
| S11 Source | Video / Osc |
| **P12 Lens** | lens size: separate ovals → an overlapping lattice |

Resolution: counts stay counts; lens geometry in virtual 1080-line units.

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 3, HD Dual seed 2, others seed 1), HD 78.1–80.6 MHz, 4.6k LC (60 %), 19 EBR. **Not yet flashed.**
