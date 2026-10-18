# Crush: summary

Catalogue combo #42 (seed 20260919): horizontal collapse / sync loss plus a
mosaic with per-cell colour averaging. v0.1.0 (2026-09-11) was flashed, and
Ron said "crush is good" (2026-09-12). v0.2.0 (2026-10-12) is the first
finalizing batch. It comes from a code review of v0.1.0 plus the shelf
audit's notes. v0.1.0 is at git 3fa438d.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Line | where the picture collapses to, 0–100% of the width (also in Right Side mode now) |
| K2 Sync | sync loss: random per-line jumps up to ±¼ of the width, the same size on screen at any Collapse |
| K3 Roll | the collapse line drifts right on its own and wraps; speed (0–4 px/frame) |
| K4 Cells | cell width 2, 4, 8, 16, 32, 64, 128, 256 px (labelled, deadbanded) |
| K5 Aspect | cell height = width / 1, 2, 4, 8: Square, Wide, Wider, Strip |
| K6 Grout | grid lines along the cells' real edges, 1/8 of a cell thick (1–8 px / lines); none on cells under 4 px |
| S7 Tiles | Square: screen-fixed cells over the squeezed picture. Squeeze: the cells squeeze with the picture |
| S8 Collapse | Both Sides squeeze toward the line. Right Side: the left of the line stays put and the right side squeezes into it (K1 = 0 is v0.1's "Left") |
| S9 Loss | Lines: every line jumps. Rows: whole cell rows jump together (bands of 8+ lines) |
| S10 Beyond | black outside the picture / the first and last cells stretched |
| S11 Grout | black / white |
| P12 Collapse | the picture squeezes toward the line, ×1 to ×64, exponential: each sixth of the travel doubles it. 100% = a ~30 px sliver on HD |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Mosaic colours were wrong.** The box sum was reset to 0 at each line
  start, but it still subtracted the N blanking samples before the line
  (luma 64; chroma held from the previous line's right edge). Every cell came
  out ~64 codes too dark, below black in the shadows. Its chroma was measured
  relative to that edge colour and wrapped, so neutral cells went saturated
  green and mild colours flipped. (Same bug as stencil v0.1.1 / benday v0.1.)
  The sum is now an exact N-sample window, with the line's first pixel
  standing in for the samples before it.
- **Cells averaged the wrong pixels.** The stored window landed 1 px late and
  the display read the cell centre, so each cell took about half its colour
  from its left neighbour. The picture also slid half a cell right as Cells
  grew. Address a now holds pixels a−N+1 .. a, and a cell reads its last
  pixel (`x | mask`): exactly its own average.
- **Ghost copies at high Collapse.** The collapsed address was cut to 14 bits,
  so a far-off address folded back into the picture. With the line centred at
  P12 100%, squeezed copies appeared at x 420–476 and 1444–1500 beside the
  real sliver (904–1016). From ~50% they showed at both screen edges. The Drain
  Left preset drew three extra copies. The address is now 20 bits. (If the
  copies were part of the look you liked, Repeat is offered as a real mode in
  Tier 2.)
- **Beyond = Stretch stretched broken samples.** The right side repeated the
  line's last buffer entry, which held a zero sample (Y 0, U = V = 0:
  saturated green below black). The left side repeated a 2-pixel partial
  window. Stretch now repeats the first and last cells' true averages.
- **S7's labels were swapped.** "Squeeze" gave square screen cells and
  "Square" gave squeezed cells. The behaviour is unchanged; the labels now
  say what each position does (S7 renamed Tiles: Square / Squeeze).
- **Squeeze tiles read past the line end** on a part cell at the right edge
  (SD 720 px with 32/64-px cells). Now clamped to the last pixel.
- **Row seam.** Line 0 of every cell row showed the row above, so the grout
  line took the colour of the wrong row (a 1-line seam at Grout 0). Each row
  is now captured on the last line of the row before it. The frame's first
  line is black.
- **Grout.** The grid was screen-fixed, so in squeezed-cell mode it cut
  through cells at random. It was a fixed 2 px, which covered every pixel at
  2-px cells and half of them at 4 px. Grout is now drawn where the read
  address changes, so it follows the cells in both modes and tears with
  Sync. Thickness is 1/8 of the cell on screen (1–8 px), and there is none on
  cells narrower or shorter than 4.
- **The top 3/8 of K4 Cells was dead** (clamped at 64 px). It now has eight
  steps over the full travel, up to 256 px. Positions below 5/8 are
  unchanged.
- Line width is latched per frame (analog ±1 px line-length jitter no longer
  moves the line or the right edge every line).

Controls:
- **P12 is exponential, ×1 → ×64.** v0.1 was linear ×1 → ×17: the first ~20%
  of travel did most of the squeeze (to ×4) and the rest crept to ×17. Now
  each sixth doubles it, and 100% collapses the picture to a sliver. The
  default and presets were translated to keep their v0.1 squeeze.
- **Sync is applied on screen, after the squeeze.** In v0.1 the jumps
  shrank as P12 rose (±510 px at 0%, ±30 px at 100%). Now they keep their
  size, so a collapsed sliver tears. The amplitude is a fraction of the width:
  ±478 px on HD at 100%. SD used to jump ±70% of the screen.
- **S8 Right Side** replaces Left: K1 and Roll were dead in Left mode. Now
  the left of the line is untouched and the right side squeezes into it.
  With K1 = 0 it is exactly v0.1's Left; rolling the line makes it a wipe.
- **S9 Rows** replaces 8-line Bands. The jumps re-roll on a cell row's
  first line (bands of 8+ lines), so whole tile rows tear together.
- K4 and K5 have value labels. K4 uses the labelled-default index guard
  (infrared/edgelab). K4 and K5 are deadbanded (±12).
- Defaults: Sync 64 (~6%, ±30 px), so S9 shows at power-on. P12 347 (= v0.1's
  ×4.1 at 200). Presets: Drain Left K1 = 0 and K2 = 32 (the same on-screen
  jitter as v0.1), Tile Crush / Wide Mosaic K2 = 0, Wide Mosaic K4 = 704
  (64 px, as before).

Hardware contracts:
- HD HDMI bar fix: [1 2 1] on Y/U/V plus luma scaled to white ~775, before
  the gate. Crush draws crisp, full-swing cell edges that jump per line. White
  grout is 940 before the scale (~775 after).
- Replayed rows swap U/V where their hsync→avid parity differs from the
  capture row's (drape v1.3). This only matters on analog sources.
- `no_rw_check` on all 7 RAMs. Every control is latched per frame, in
  vblank.
- Latency 14 → 19 clocks in all modes, blanking-gated.

## Next tiers (offered 2026-10-12, not done)

Tier 2: features (Ron's call):
- **Repeat** as a real Beyond mode: the squeezed picture tiles across the
  screen (×N copies, a video wall) instead of black or stretch. This is the
  v0.1 ghost copies made deliberate. It needs a third state: Beyond on a knob,
  or another switch given up.
- **Acid colour** as a deliberate mode, if v0.1's wrapped colours were part
  of the appeal: cells' chroma measured against the line's edge colour.
- **Anti-aliased squeeze**: the box width follows the squeeze (max of cell
  and ×g), so a collapsed picture averages instead of skipping columns. Less
  sparkle at high P12.
- **Hold loss** as a third Sync form: a slanted tear that accumulates down
  the screen and rolls (real horizontal-hold failure).
- **Bipolar Roll** (centre = still, either direction).
- **Tall cells** (aspect 1:2, 1:4) for vertical streak mosaics.
- **2-D cell average**: accumulate a row's lines, so thin horizontal detail
  stops twinkling. It costs one cell row of lag.
- **SD interlace**: rows count per field, so cells are twice as tall on
  NTSC/PAL. Halve the rows when the measured height is under 300 lines.

Tier 3: polish:
- Defaults and presets captured from hardware (capture_preset.sh).
- After sign-off: v1.0.0, shelf card refresh, commit.

## v0.2.2 resolution scaling (2026-10-17)

The baseline sheet showed cells 1.5× too big at 720p and much bigger at SD
(cells were fixed pixels, and rows counted field lines, so SD cells were also
twice as tall). Cell sizes now follow the screen height:

- The read-address chain (screen x, sync jumps, Square hold, collapse line,
  Squeeze hold, edges) runs in virtual 1080-line units: 1.0 / 1.5 / 2.0 units
  per pixel at 1080 / 720p / SD, from the timing ID in `registers_in(8)`. SD
  is a 1440-wide 4:3 window. At the end, the virtual address becomes a
  line-buffer pixel: 720p `floor((2V+1)/3)` = `((V+1)·5·17·257) >> 15` (three
  shift-adds), SD `V/2`, 1080 `V`. This adds 4 clocks of latency, now 24 in
  all modes.
- The box average spans 2^(n−1) pixels at 720p and SD. That is the whole cell
  at SD, and 3/4 of it at 720p (a 2N/3-pixel cell). It is never under 2.
- Rows count virtual lines: a Q8 accumulator stepping 2·1080/486 per 480i
  field line. The wrap is a mask, because rows are 2^rr units. The
  next-line-ends-its-row check runs one clock after the avid fall. S9 Rows
  bands count 8 virtual lines.
- Grout: horizontal grout stays 1/8 of the screen cell, at least 1 px (720p
  1/1/1/3/5 px, SD 1/1/1/2/4 px). The row band is at least one line. Rows
  under 2 screen lines can't show a band, so their grout falls on every R-th
  screen line, which keeps the 1080 coverage.
- Sync jumps are a size, so at SD they are sized from 1920 units, not the
  1440-unit window. Line position (K1) and Roll stay fractions of the
  virtual width.
- 1080p is bit-identical (diff1080 = 0).

## Build

`./build_combo.sh crush` → `out/rev_b/ron/catalogue-combo/crush.vmprog`.

v0.1.0 (2026-09-11): HD 79.0 / 78.5 / 79.6 MHz (an / hdmi / dual), 4,173 LC,
seed 1.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default). HD
Analog 81.5, HD HDMI 78.1, HD Dual 80.1 MHz; SD 82.2 / 67.6 / 81.3 MHz.
About 5,440–5,500 LC (71%), 20/32 EBR (10 luma line buffer, 8 chroma, 2 box
ring). In a standalone synth, `emulate_transparency` = 0.

HW status: v0.2.0 built, not yet flashed.

v0.2.2 (2026-10-17, resolution scaling): all 6 configs pass on seed 1 (router2
default). HD Analog 81.4, HD HDMI 79.0, HD Dual 78.2 MHz; SD 80.8 / 73.4 / 82.2
MHz (SD Fmin is 27). About 5,780–5,820 LC (76%), 20/32 EBR. Not flashed.
