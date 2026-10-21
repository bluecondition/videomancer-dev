# Argyle — Heirloom #8

**Source photo**: `photos/WIN_20230207_23_50_54_Pro.jpg` (1920×1080 capture, 2023-02-07)

## Breakdown of the photo

- **Composition**: a diamond (rhombus) lattice, eight cells across and about four and a half down, the rows
  staggered so the diamonds interlock like a quilt.
- **Rings**: every diamond is a set of nested rhombus rings — a hot magenta core, then fine alternating bands
  of blue, green, pink and white, dozens of them, thinning toward the cell edge.
- **Colour**: saturated magenta cores; the rings are a cycling pastel/neon palette.
- Mechanism: two folded ramps summed (|x| + |y| within a cell) and run through a fast colorizer ramp.

## Design

| Photo element | Program |
|---|---|
| Lattice | N square cells across (K1, 1–8), rows of the same height; S7 Brick offsets odd rows by half a cell |
| Rhombus rings | field = a·\|dx\| + b·\|dy\| per cell (a/b from K2 Aspect), cut by a cyclic 256-entry palette |
| Magenta core + fine rings | palette A: magenta core → white → pink → blue → green → cyan → blue → pink → magenta; palette B amber |
| Ring density | P12 RINGS (zk 2–257, two partial products) |
| Motion | K4 Drift adds a per-field phase to the ring index |
| The picture | luma × K5 added to the field (etch); S10 Tile uses the cell ramp instead of \|dx\| (fish-scale) |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks, all modes.
Only the luma is read from the line buffer (yosys trims it to 5 EBR). Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Cells | cells across, 1–8 (rows square) |
| K2 Aspect | diamond aspect: wide → tall |
| K3 Hue | palette rotation |
| K4 Drift | rings flow outward / inward, bipolar |
| K5 Etch | video depth into the field |
| K6 Glow | ring brightness, 50–100 % |
| S7 Brick | odd rows offset by half a cell |
| S8 Rings | Smooth / Hard (8 bands) |
| S9 Palette | Magenta / Amber |
| S10 Lattice | Mirror (diamonds) / Tile (sawtooth scales) |
| S11 Source | Video / Osc |
| **P12 Rings** | ring density: three rings → sixty fine rings |

Resolution: cell count is a COUNT (cells narrow on 4:3 SD); rows are square in virtual units.

## Status

- v0.1.0 (2026-10-20): first build missed HD Analog (57 MHz, seed 6): the 21×9 ring-zoom multiply in one
  stage and a per-line \|dy\|·b product fed straight from the subtract. Split the zoom product into two partial
  products and registered \|dy\| first. Second build: 6/6, HD 78.6–88.0 MHz (HD Dual seed 2 after a router stall), 4.1k LC (53 %), 9 EBR. **Not yet flashed.**
