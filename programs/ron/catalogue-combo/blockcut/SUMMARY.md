# Blockcut — summary

Catalogue combo #32 (seed 20260917): edge detect + datamosh / block
displacement. v0.1.0 (2026-09-11) was flashed and liked ("blockcut is good",
2026-09-12). v0.2.0 (2026-10-12) is the first finalizing pass, built from a
review of v0.1.0. v0.3.0 (same day) adds the Tier 3 items: Sobel edges and
colour-only blocks. Backups: `.blockcut_work/v010_backup/` (also git 3fa438d)
and `.blockcut_work/v020_backup/` (sources + vmprog).

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Gain | edge gain (Sobel magnitude / 4, same calibration as v0.1's 2-tap) |
| K2 Thresh | edges below this are dropped |
| K3 Blend | edges on black (0) .. edges over the cut-up picture (100) |
| K4 Block | block size 8..64 px, continuous (column/row DDA counters) |
| K5 Shift | moshed-block displacement, ±48..±1008 px HD (±24..±504 SD), wraps inside the picture |
| K6 Rate | re-roll every ~2 s .. every field, exponential |
| S7 Edges | white / tinted (picture chroma at double saturation, ink luma 768) |
| S8 Blocks | plain / moshed blocks go negative (luma 64↔940, chroma mirrored) |
| S9 Hash | per block / per band (whole rows of blocks share a shift) |
| S10 Mosh | shift / hold (block repeats its first column) |
| S11 Ink | light edges on black / dark edges on white |
| P12 Density | fraction of blocks moshed, ~5% .. ~98%; half as many again are colour-only |

## v0.3.2 (2026-10-17): resolution scaling

The block grid ran in pixels / frame lines, so blocks were 1.5x too big at
720p and about 2.2x at SD, and the HD shift switch (width bit 10) gave 720p
the 1080p pixel offsets. Now:
- The block column DDA (Q.1) and row DDA (Q.8) count virtual 1080-line units,
  stepped from the timing ID (registers_in(8)): 1.0 at 1080p (bit-identical),
  1.5 at 720p, 2.0 x 1080/486 at 480i (the y step is per field line and
  already doubled, replacing the old interlace +2). SD is a 1440-unit 4:3
  window (left-anchored, same hash layout as the left of HD), blocks square.
- Shift: SD keeps its half offsets; 720p scales the amount by 2/3 in the
  vblank sequencer, so displacements are in the same virtual units.
- Sobel edges, the C_EDGE inset and line buffers stay in real pixels.

Verified: 1080p diff 0 px; 720p r = 0.94 against 1080p (was 0.64); SD blocks
the same height fraction as 1080p. Sheets `.scaling_work/sheets/blockcut{,_v2}.png`.
Build `./build_combo.sh blockcut` (router2, seed 1, all six passed first try):
HD Analog 86.3, HD HDMI 84.0, HD Dual 82.5 MHz; SD Analog 83.5, SD HDMI 75.4,
SD Dual 79.0 MHz. ~5,090 LC (66%), 23/32 EBR.

## v0.3.0 changes (Tier 3)

- 3×3 Sobel edges, (|Gx| + |Gy|) / 4, replace the 2-tap |gx| + |gy|. A step now
  draws a 2-px line at full strength instead of 1 px, and the [1 2 1] weighting
  along the edge cuts speckle. The magnitude is divided by 4, so Gain and
  Threshold keep their old calibration. The two lines above come from one
  single-bank 2048×20 RAM holding {y[n-1], y[n-2]}, rewritten each line from
  fabric registers (pipelined read-modify-write). The edge is written at the
  window centre (x − 1). No edges on a field's first two lines.
- Colour-only blocks (codec-error look). A block whose hash byte lies in
  [Density, 1.5 × Density) keeps its luma and edges in place but reads its colour
  from the displaced spot. Luma/edges and chroma now have separate read
  addresses after the shared wrap. That is visible at Blend > 0, or with Tinted
  edges. There are about 2% of these at the P12 floor, about 30% at mid slider,
  and almost none at 100% (where nearly every block is fully moshed).
- Read moved to stage 12 (picture write stage 4 → 2, lead 10; edge write stage
  8 → 10, lead 1). Still collision-free with multiple-of-4 offsets.
- Latency 21 → 23 clocks, all modes. EBR 18 → 23.

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- K3 Blend at 100% crossfaded to the un-moshed dry picture, so the edges, the
  block shifts and five controls vanished (shelf audit). The picture now has its
  own line buffer read at the same displaced address as the edges. Blend fades
  paper → picture underneath, and the edge strength keys the ink on top. At 100%
  the picture itself is cut into the blocks, with the edges still drawn over it.
- The re-roll did not re-randomize. The roll was added to the hash key, so each
  new pattern was an exact copy of the previous one, shifted 8 block-rows and
  11 columns (numerically 100% match). The blocks slid diagonally. A per-roll seed
  now enters after the first hash round (frame-to-frame match 0.637 against 0.625
  random).
- With 8-px blocks in HD (240 columns), the row key was spaced 128. The right half
  of the screen was an exact copy of the left half one block-row down. The key is
  now `row & column` (8 + 8 bits, no adder).
- `abs(g)` narrowed with `resize`, so a luma step ≥ 512 (black↔white) folded:
  876 came out as 364. The strongest edges drew weaker than medium ones at low Gain.
- K4 snapped between four sizes (8/16/32/64) while the display claimed a continuous
  8–64 px. A DDA block counter now gives every integer size.
- Negative used `not y`, so black came out at 959 (above white). It is now
  1004 − y, and the chroma is mirrored too.

Hardware contracts / quality:
- No edges in the C_EDGE (12 px) side columns: the capture's black blanking
  columns drew a bright vertical line at each side. No gy on a field's first line
  (it compared against the previous field's last line).
- Interlace (field height < 600 lines): block rows count frame lines, so blocks
  stay square, and the hash re-rolls only ahead of a top field (no per-field comb).
  SD uses field_n. HD 1080i (field_n tied high) uses a per-field toggle.
- Tinted ink luma is capped at 768 (dark tinted 192) so the tint survives RGB
  clipping. White ink stays 940 (neutral chroma).
- All line buffers single-bank and collision-free (offsets and wrap span are
  multiples of 4; edge write leads the read by 2, picture by 6), `no_rw_check`
  on every RAM.
- Latency 18 → 21 clocks, the same in all modes; blanking gate kept.

Controls:
- P12 Density floor ~5% and K5 Shift floor (±48 px HD), so K4/K5/K6/S8/S9/S10 are
  live at any slider position.
- K6 Rate is exponential (period 128 → 1 fields over 32 steps). Before, 90% of
  the knob covered 2 s .. 0.27 s.
- K5's top half no longer piles half of all moshed blocks onto ±511 (clamp
  removed; wraps instead of going black).
- Defaults/presets: Block and Rate values remapped so each keeps its old block
  size and re-roll speed.

## Build

v0.3.0, 2026-10-12, `./build_combo.sh blockcut` (router2). All six configs
passed on seed 1:

| Config | Fmax |
|---|---|
| HD Analog | 82.7 MHz |
| HD HDMI | 79.6 MHz |
| HD Dual | 80.2 MHz |
| SD Analog / HDMI / Dual | 82.8 / 75.4 / 87.1 MHz |

~5,000 LC (65%), 23/32 EBR. yosys `emulate_transparency` = 0. iceunpack OK
on all six. Vmprog: `out/rev_b/ron/catalogue-combo/blockcut.vmprog`.
v0.2.0 (HD 85.4 / 76.3 / 83.9, 18 EBR) was never flashed; it is kept in
`.blockcut_work/v020_backup/`. Not committed: commit at hardware sign-off.

## Hardware checklist (v0.3.0)

- Edges: lines about 2 px wide and smoother/less speckly than v0.1. Gain and
  Threshold should sit about where they did.
- Colour-only blocks: at Blend 100% (or with Tinted edges), some blocks keep
  their shapes but show colour from elsewhere. Most of them at mid Density,
  almost none at 100%.

- Blend 0 should look like v0.1 (same ink levels). At 100% the picture itself
  should be cut into blocks, with the edges still drawn on top.
- Rate: blocks re-randomize in place (v0.1 slid diagonally).
- Block: sweeps smoothly 8 → 64 px (v0.1 had four steps).
- Density at 0: a few blocks (~5%) still moshed.
- Sides: no bright vertical line at the left and right edges in edge mode.
- S8 Negative: black paper turns white (not super-white) and colours invert.
- S7 Tinted: the line colour is visible, including at Blend 100%.
- HD HDMI at high Rate: watch for thick bright bars (the damier fix would soften
  the 1-px lines, so it is held back until they show up).
- SD interlaced: square blocks, no comb on block edges at high Rate.
