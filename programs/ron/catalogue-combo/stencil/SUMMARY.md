# Stencil: summary

Catalogue combo #2 (seed 20260912): a cell-averaged mosaic keyed against the
original by luma or saturation. Ron flashed v0.1.1 on 2026-09-11 and said
"stencil is good". v0.2.0 (2026-10-12) is the first finalizing batch. It
comes from a code review of v0.1.1 plus the shelf audit's notes.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Cells | cell width 2 to 256 px, in 8 octave steps (deadbanded) |
| K2 Aspect | cell height = width / 1, 2, 4 or 8 (Square, Wide, Wider, Strip) |
| K3 Grout | dark lines between cells, 1/16 of a cell thick (min 1 px), floored at black |
| K4 Tint | 0–3% off. Above that, mosaic cells are washed toward a hue that goes once round the colour wheel |
| K5 Soft | key edge, from hard (±16 codes) to very soft (±512), over 5 octaves |
| K6 Source | key from luma (0) to saturation \|U\|+\|V\| (100), blended |
| S7 Mask | Normal: the mosaic eats the shadows first. Invert: it eats the highlights first |
| S8 Key From | key from the original, or from the mosaic (cell by cell) |
| S9 Reveal | the uncovered picture is the original, or its negative |
| S10 Luma | Mosaic, or Sharp (cells keep the original luma; only colour is averaged) |
| S11 Grout | grid everywhere, or only on the mosaic |
| P12 Stencil | key threshold, glided. 0% = only the deepest shadows are mosaic; 100% = all but the brightest highlights |

## v0.2.0 changes (from the v0.1.1 review)

Bugs:
- **Mosaic colours were wrong.** At each line start the box sum was reset
  to 0, but it still subtracted the blanking samples (Y 64, U/V 512) that
  preceded the line. Every cell came out about 64 codes too dark, and its
  chroma sat about 512 codes low and wrapped, so neutral cells went
  saturated green. The bug is confirmed in sim on v0.1.1. The sum is now an
  exact N-sample window, with the line's first pixel standing in for the
  samples before it.
- **Cells averaged the wrong pixels.** The stored window landed 2 px late,
  and the display read the cell centre. Each cell therefore took half its
  colour from its left neighbour. Line-buffer address a now holds the
  average of pixels a−N+1 .. a. The display reads the cell's last pixel
  (`x | mask`, no adder), so each cell shows its own average.
- **The original sat 8 px right of the mosaic grid** (dry ring delay 12, should
  have been 4). The key and the reveal are now aligned to the grid.
- **The first line of every cell row showed the row above.** It was a 1-line
  seam per row at Grout 0. Each row is now captured on the last line of the
  row before it, ping-ponged by bank. The frame's first line has no capture
  yet, so it shows the original.
- **The saturation key used only |U−512|** (the blue/yellow axis), so reds
  and cyans read as grey. It now uses (|U|+|V|) × 1.5.
- **K5 Soft at max was a flat 50% crossfade** with no key at all, over the
  top third of the knob. The gain is now 8 × 2^(−5k/63), a 64-entry G/2^S
  table, and never flat.
- **The top 3/8 of K1 Cells was dead** (clamped at 64 px). K1 now gives 8
  steps over its full travel, 2 to 256 px (256 px = 7.5 cells across HD).
- **Grout was a fixed 2 px.** At 2-px cells it covered every pixel, and at
  Strip every line. It is now 1/16 of the cell, with none on 1-line rows.
- **The key had dead travel at both ends** (luma lives in 64..940). Luma is
  now normalized: (Y−64) × 1.125.
- **S9 Negative inverted luma around 1023** (black → 959, over the luma cap).
  It now flips within the legal range: 1004 − Y, so black and white swap.

Controls:
- **P12 Tint → P12 Stencil.** The key threshold is on the slider, glided
  (1/8 per frame), at 48..1007 key units. At 0% the deepest shadows are
  already mosaic, so K1–K3 and S8/S10/S11 act. At 100% all but the
  brightest highlights are mosaic, so K5/K6/S7/S9 still act. S7 flips which
  end gets eaten first, so the slider always means "more mosaic".
- **K4 Level → K4 Tint.** The mosaic chroma becomes u/2 + (512 + 400·cos)/2,
  a wash toward the hue. It is applied before the blend, so it fades through
  soft key edges. The old tint was a hard step at the key midpoint and had
  one fixed hue.
- K3 Grout defaults to 35%. Before, S11 did nothing at power-on.
- 6 hand-written presets for the new controls. The redundant "Default"
  preset is gone.

Hardware contracts:
- Replayed rows swap U/V on rows whose hsync→avid parity differs from the
  capture row's (drape v1.3). This only matters on analog sources.
- HD HDMI bar fix: [1 2 1] on Y/U/V plus luma scaled to white ~775, before
  the gate. Mosaic cell edges are crisp, full-swing and move with the picture.
- Dry video comes from two EBR rings, tapped where the key (stage 6) and the
  blend (stage 16) need it. Raw video is never carried deep in FFs (the
  rollthebones lesson).
- All switches and knobs are latched per frame. K1/K2/K4 are deadbanded
  (±12), so the cell size cannot flicker at a step edge.
- `no_rw_check` on all 9 RAMs.
- Latency 24 clocks in all modes; blanking-gated.

## Next tiers (offered 2026-10-12, not done)

Tier 2: features (Ron's call):
- **Tile shapes on K3:** square grout → round dots (LED wall / Lite-Brite) →
  diamonds. The cells would be redrawn as primitives.
- **Any-size cells:** quarter-octave steps (2, 3, 4, 5, 6, 7, 8, 10, 12 ...
  256) for smooth size sweeps, via a reciprocal shift-add per cell.
- **True 2-D cell average:** accumulate the row's lines, so a cell averages
  its whole block rather than one sampled line. Thin horizontal detail would
  stop twinkling. It costs one cell row of lag.
- **SD interlace:** rows count per field (the core ties field_n high), so on
  NTSC/PAL cells are twice as tall. Halve the rows when the measured height
  is below 300 lines.
- **More aspects:** tall cells and columns.
- **Stencil outline:** a thin cut line along the key boundary.

Tier 3: polish:
- Defaults and presets captured from hardware (capture_preset.sh).
- K6 "Source" → "Key Src" (audit naming note).
- After sign-off: v1.0.0, shelf card refresh, commit.

## Build

`./build_combo.sh stencil` → `out/rev_b/ron/catalogue-combo/stencil.vmprog`.

v0.1.1 (2026-09-11): HD 80.2 / 82.3 / 76.6 MHz (an / hdmi / dual), ~4,500 LC
(59%), 23/32 EBR, seed 1.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default). HD
Analog 78.0, HD HDMI 80.7, HD Dual 76.3 MHz; SD 74.5 / 74.5 / 75.4 MHz.
About 5,790 LC (75%), 24/32 EBR (18 line buffer, 2 box ring, 2 + 2 dry
rings). In a standalone synth, `emulate_transparency` = 0.

HW status: v0.2.0 built, not yet flashed.

Sim check (480×270 colour bars, 2026-10-12): v0.1.1's mosaic renders
saturated green, confirming the box-sum bug. v0.2.0 renders true cell
averages on the 16- and 64-px grids, with grout and tint. In the sim P12's
glide is still settling after 3 warmup frames, which is expected.
