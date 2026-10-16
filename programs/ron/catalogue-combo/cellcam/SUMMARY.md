# Cellcam — summary

Catalogue combo #86 (seed 20260923): edge detect + chroma subsampling +
pixelation. v0.1.0 (2026-09-11) was flashed and liked ("cellcam is good",
2026-09-12), with no later hardware notes. v0.2.0 (2026-10-12) is the
finalizing pass, built from a review of v0.1.0. v0.3.0 (2026-10-13) adds
Ron's three Tier 2 picks on top. Neither is **flashed yet**.

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Crawl | NTSC dot crawl on the edges. Past 50% it spreads over the whole picture, ±126 at 100% (0 = clean) |
| K2 Shape | cell height as a ratio of its width, exponential: 1/4 (flat) at −100, square at 0, ~4× (tall) at +100 |
| K3 Gain | edge gain, exponential: 1/8x at 0, 1/2x at 25%, 2x at 50%, 8x at 75%, 31x at 100% |
| K4 Width | outline width 1..8 px on the left side of each cell, 1..8 lines on the top. Raw: edges thickened Width px right and Width lines down |
| K5 Hold | chroma held across 1..16 cells (1 = each cell its own colour) |
| K6 Smear | chroma lag 0..127 px; past 25% the chroma also bleeds toward each new block (8 / 16 / 32 px at 25 / 50 / 75%, doubled at HD) |
| S7 Edges | Over: drawn over the cells / Only: edges alone on plain neutral paper |
| S8 Source | Cells: steps between cells (left and top outlines) / Raw: gradient of the live picture, \|dx\| + \|dy\| |
| S9 Chroma | Cells: colour from the stored cell row / Live: colour latched from the current line, so it flows down inside the held luma blocks |
| S10 Ink | black / white |
| S11 Tint | edges keep the cell colour / edges mono |
| P12 Cell | cell size 2..129 px, quadratic (10 px at 25%, 34 at 50%, 74 at 75%). The grid is centred on the screen, so the slider zooms the pixelation about the centre |

## v0.3.0 changes (Ron's Tier 2 picks, 2026-10-13)

- **P12 = Cell, a live pixelation zoom.** A vblank sequencer sizes the
  grid once per field: W = 2 + (P12/4)²/512, and the cell height R = W ×
  Shape (in field lines on interlaced sources, detected as field height
  < 7/16 of the width). It centres the grid by starting each line at column
  phase −(width/2) mod W, and each field at row phase −(height/2) mod R. The
  chroma hold blocks are centred too, using the cell count mod Hold. So the
  centre cell stays put while the slider zooms. P12, Shape and Hold have a
  ±4 LSB deadband, so pot noise never jitters the grid. The grid changes
  only between fields, so a sweep never tears. Biggest cells are 129 px
  (15 across at HD).
- **K2 Rows → Shape**, so the zoom scales both dimensions. Square at the
  centre.
- **Crawl moved from P12 to K1** (same curve as v0.2).
- **S9 = Chroma Cells / Live.** Live latches the current line's colour at
  each hold block's first column, so the colour moves line by line inside
  blocks whose luma is held. K5 Hold still sets the block width, and no U/V
  parity swap is needed (it is the current line).
- **Bleed moved from S9 into K6, now "Smear".** The first quarter is pure
  lag (0..31 px). Past 25% the bleed fades in, and the lag keeps growing to
  127 px.
- **Raw edges thickened vertically too.** A 2048×8 RAM (+4 EBR) holds, per
  column, the level (5 bits) and lines left (3 bits) of the last strong
  edge. It is read at stage 8 and written back at stage 11, and gives a
  solid hold Width lines down, matching the Width-px hold to the right. It
  is suppressed on the field's first line.
- Latency 22 → 23, all modes. Field line-state resets moved from the vsync
  edge into the sequencer's commit, so a serrated vsync edge after the
  commit cannot undo the centring phase.
- Presets reworked for the new layout: Comic Cells, White Wire, Raw Crawl,
  plus Live Blocks (Live chroma, big cells, heavy smear) and Thick Sketch
  (Raw, Only, white ink, Width 8). All are hand-written.

Checked before building: a Python sweep of the sequencer at the RTL's bit
widths (P12 × Shape × Hold, at 1080p / 1080i / 720p / 480i / 576i). The
cell, hold-block and row boundaries land on the screen centre in every
case, with no overflow. I haven't simulated it.

## v0.2.0 changes (from the v0.1.0 review; still in v0.3.0 except where the v0.3.0 list moves a control)

Bugs:
- **The outlines were one pixel off, and at Width 1 they showed the wrong
  boundary.** The cell-start test read the column counter one stage early
  and the near-edge mask one stage late. At Width 1 the outline pixel took
  the step of the *previous* boundary (cells c-2 → c-1 drawn between c-1
  and c). At wider widths the first pixel of each outline had that stale
  value. The new pipeline latches the step and the band on the same pixel.
- **The picture lagged a whole cell row.** Each row showed the stored line
  of the row *above* (up to 32 lines late at Rows max), so the cells sat a
  row low against the raw edges. Every row now reads its own stored line.
  The first line reads columns already written on that line.
- **Black ink drove luma below black.** It clamped at 0, not 64. White ink
  and white paper reached 940–1023, above the ~768 drawn-luma cap. Luma is
  now clamped to 64..940, then the highlight knee (below).
- **"On black" was not black.** The paper kept each cell's full chroma, so
  it showed dark coloured blocks (and pastel blocks on white paper). The
  paper is now neutral. With Tint Colour the edges take the cell colour,
  scaled by edge strength (full at about half-scale).
- **A spurious edge ran down the left border.** The first cell of each
  line was compared with the last cell of the line before. It is now
  suppressed, and so is the top outline on the field's first row and the
  raw gradient on each line's first pixel and the field's first line.
- **Crawl did not crawl.** The dot phase advanced 2 per hsync (serrated
  vsync included), so the pattern stood still or flickered at 30 Hz. It now
  inverts every active line and steps a quarter-cycle every field (guarded
  vsync edge), so the dots crawl.

Dead or gated controls (bold, always-on):
- **K5 Hold and S9 did nothing at the defaults.** The picture is already
  pixelated in chroma, so holding chroma for fewer pixels than a cell
  changed nothing. v0.1 defaults were Hold 2 px under 8 px cells, so K5 did
  nothing below the cell width. S9's two modes were identical there too.
  Hold now counts *cells* (1..16): every step makes wider colour blocks.
  S9 is now Hold / Bleed: hard blocks, or chroma easing toward each block's
  colour (a VHS chroma-bandwidth smear). It is visible at any setting on
  colour sources.
- **The cells had no top outline.** Only the left boundary of each cell was
  drawn, so the picture got vertical rules but never horizontal ones. The
  other line-buffer bank already holds the row above, so the top outline
  (`|cell − cell above|` over the top Width lines) costs no extra RAM. The
  corner pixels take the stronger of the two steps.
- **Raw edges were horizontal-only.** The gradient was `|y(x) − y(x−1)|`,
  so horizontal lines in the picture were never outlined. It is now
  `|dx| + |dy|`, with a 1-line luma delay (+4 EBR).
- **K4 Width did nothing in Raw.** Raw edges now widen to Width px
  (one-sided peak hold).
- **Gain 0 removed the edges completely.** That left Width, Ink, Tint and
  Crawl dead. The gain is now exponential, 1/8x..31x. The v0.1 default (2x)
  sits at 50%, and the top is far bolder: at 31x every cell boundary with
  a step inks fully.
- **Lag was 0..15 px**, barely visible at HD. It is now 0..127 px, through
  a one-EBR ring that also replaces 320 FFs of delay line.
- **Crawl reached only edge pixels and, at HD, drew 4-px dots.** Its
  amplitude is now `(edge + base) × Crawl`. The base is 0 below 50% and
  ramps to ±126 chroma at 100%, so the slider's top end is full-screen dot
  crawl (strongest on the edges). At HD the dots are 8 px a cycle (4 px at
  SD, the NTSC proportion).

Display honesty:
- v0.1 K1 showed 2..64 px but had 6 sizes (two knob ranges duplicated). K2
  had 6 sizes, K4 had 4, and K5 showed 1..16 px but had 5 sizes. Cell and
  Rows are now continuous and quadratic, shown in % because no programs use
  the easing display modes, so their firmware behaviour is unverified. Width (1..8 px), Hold
  (1..16 ×) and Lag (0..127 px) are continuous and linear, and match their
  displays.
- S7 "On black" is now "Only": with black ink the paper is white. S11 was
  a second switch named "Edges" and is now "Tint".

Hardware contracts:
- **HD HDMI bar fix:** [1 2 1] on Y/U/V and a highlight knee
  (600 + (y − 600)/2, so white 940 → 770; mid-tones untouched). Only mode
  with white ink draws exactly the trigger, full-swing edges in motion.
- **U/V swap on parity-mismatched rows** (drape v1.3). Every line of a
  cell row replays the stored first line, so rows whose hsync→avid parity
  differs from the stored line's get U↔V swapped.
- Neutral outside avid before the smoothing, and gated again at the
  output. Latency 16 → 22, the same in all modes. Row state (bank, outline
  band, swap, crawl phase) is frozen at each line's avid rise, so a line's
  tail never sees the next line's state.

Optimizations:
- `no_rw_check` on all 8 RAMs (no RAM ever reads the address it is
  writing that clock). Standalone yosys: `emulate_transparency` count 0.
- Cell luma stored at 8 bits (4 EBR per bank, was 5). EBR 18 → 21: −2 from
  that, +4 for the raw delay line, +1 for the lag ring.
- The 16-tap × 2 × 10-bit lag shift register and its two 16:1 muxes are
  now the EBR ring.

## Hardware checks for v0.3.0

1. P12 sweep: does the pixelation zoom smoothly about the screen centre,
   with no grid jitter when the slider is still?
2. K2 Shape: square at the centre, flat bars at −100, tall columns at +100.
   On a 1080i source the "square" may read 2:1 if the interlace detection
   misses. Check one.
3. S9 Live with tall cells (Shape +) and Hold 1: colour flowing inside the
   blocks.
4. K6 Smear: lag only below 25%, then the bleed. Is the 75%+ smear too
   much?
5. Raw + Width 8: edges thick in both directions.
6. The v0.2.0 list: outlines on the cell boundaries (Width 1), Gain sweep,
   Only mode paper, Crawl climax, brightness (~770 whites), HD HDMI bars
   (Only + white ink + fast motion), U/V lines on an analog colour source.

## Review backlog (not done)

Tier 2, your call (each changes the look or the control layout):
- (Done in v0.3.0: P12 Cell zoom, Live chroma, vertical Raw thickening.)
- **Solid comic ink:** above ~75% Gain, outlines go binary (full ink past a
  threshold) rather than grey-scaled by step size.
- **Brick grid:** odd cell rows offset by half a cell.
- **Box-averaged cells** instead of point samples: less shimmer on motion,
  but much heavier (running-sum ring + column accumulators).
- Coloured ink (the complement of the cell colour, or a fixed palette).

Tier 3:
- Capture defaults and presets from hardware. The current 6 are
  hand-written for the v0.3 layout.

## Build

`./build_combo.sh cellcam` → `out/rev_b/ron/catalogue-combo/cellcam.vmprog`.

v0.3.0 (2026-10-13): all 6 configs pass (router2 default). HD Analog 90.0,
HD HDMI 82.4 (**seed 2**, after seed 1 stalled the router), HD Dual 87.0
MHz; SD Analog 87.8, SD HDMI 85.7, SD Dual 83.6 MHz (seed 1 except HD
HDMI). 4,679–4,704 LC (61%), 25/32 EBR. Standalone yosys: 2,121 LUT4,
`emulate_transparency` 0, sequencer not FSM-extracted.

v0.2.0 (2026-10-12): all 6 configs pass (router2 default). HD Analog 93.4,
HD HDMI 91.3, HD Dual 85.7 MHz, all seed 1; SD Analog 81.9 (**seed 2**,
after seed 1 stalled the router under load from other tabs' builds), SD HDMI
90.8, SD Dual 86.9 MHz (seed 1). 3,866–3,879 LC (50%), 21/32 EBR.
Standalone yosys: 1,628 LUT4, `emulate_transparency` 0.

v0.1.0 (2026-09-11): HD 90.7 / 91.7 / 97.9 MHz, seed 1, 3,395–3,417 LC,
18 EBR.

## Backups

v0.1.0: git 3fa438d (sources unchanged from 2026-09-11). v0.2.0 was never
committed; its sources and vmprog are in the 2026-10-12/13 session
scratchpad (cellcam_v020/) only.
