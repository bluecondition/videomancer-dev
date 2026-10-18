# Stratamix — summary

Catalogue combo #132 (seed 20260927): depth-from-luma parallax +
posterization + multi-source mixing. v0.1.0 (2026-09-13) was flashed and
liked ("stratamix is good"), with no later hardware notes. v0.2.0
(2026-10-13) is the first finalizing pass, built from a review of v0.1.0.
**Not flashed yet.** The v0.1.0 sources and vmprog are in `.stratamix_work/v010/`
(also git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Depth | parallax gain, bipolar (glided, 10-bit) |
| K2 Pivot | luma level that does not move |
| K3 Levels | **now labelled** 2 / 4 / 8 / 16 / 32 / 64 levels (sixths of the travel; default 8) |
| K4 Tint | chroma tint per luma band |
| K5 Mode | Add / Screen / Multiply / Difference / Overlay (fifths of the travel, as the display shows) |
| K6 Mix | how far the blend replaces the displaced layer |
| S7 Order | posterize before the shift (terraces) / after the mix |
| S8 Layers | same / alternate levels shift opposite ways |
| S9 Chroma | colour from the displaced layer / from the still picture |
| S10 Key | blend everywhere / only above the pivot |
| S11 Contour | plain / black contour bands at every level boundary |
| P12 Strata | depth and mix rise together (glided) |

## v0.2.0 changes (Tier 1 of the review)

Bugs:
- **The Mode default was really Add.** The firmware loads a labelled default
  as its label INDEX (3 for "Difference"), and the eighths decode read 3 as
  Add. So v0.1 always opened in Add until K5 was turned. The index is now
  honoured until the knob first reads a real position.
- **The Mode labels lied over much of the knob.** The display divides the
  knob into fifths; the program decoded eighths (Multiply and Overlay only got
  one eighth each). At 60%, for example, the display said Difference while the
  picture was Multiply. It now decodes fifths, with ±10 counts of hysteresis.
- **Levels had the same mismatch.** It displayed 1..6 (rounded) over a 16-zone
  decode. It is now labelled by level count, in sixths. The Relief Map preset
  moved 550 → 430 so that it keeps 8 levels.
- **Three delay chains were one pixel late** (the "stage k" arrays started a
  stage late). The base layer and the blend reference lagged the blend by
  1 px, so luma sat 1 px right of chroma at low Mix, and contours lagged too.
- **The blend modes were normalised to 1024, but white is 876** (black-relative).
  Multiply took white × white to 813 (−15%). Overlay jumped ~62 codes where A
  crosses ~54% luma, which drew a false contour through the mid-tones. The
  products are now divided by W = 876 (three shift-adds), and the inversions
  use W, so Overlay is continuous and white × white stays white. Add and
  Difference are unchanged.
- **The contour ink carried the band tint**, so "black" lines were dark
  coloured. They are now neutral black.
- **Posterize-after could reach 960 luma.** That is now under the knee (below).

Hardware contracts:
- **Per-row U/V swap for the replayed line (drape v1.3, as vhold).** Operand A
  is the previous line. On analog sources whose hsync→avid parity wanders,
  rows whose parity differs from the stored row's had Cb/Cr swapped (thin
  coloured lines) with S9 Shifted, which is the default.
- **The tint swaps on off-phase rows.** The tint is colour the program draws on
  top of passthrough chroma. On rows whose parity is not the usual one
  (saturating vote), it landed on the other axis, which flips the hue in two
  of the four bands. Its U/V offsets now swap on those rows.
- **Chroma fetched late.** The chroma read address (with its bank and swap bit)
  travels to stage 23, and the still picture's chroma comes from a second dry
  ring read 23 back. Raw chroma now enters the FF pipe ~5 stages before the
  output (v0.1 carried it ~14 deep; the rollthebones/cascade trap). The bank
  travelling with the address also covers the SD line tail, which now reads
  past hsync (SD front porch is 16 clocks).
- **HD HDMI bar guard, built in.** A highlight knee (600 + (y − 600)/2:
  940 → 770, mid-tones untouched, as vhold) and [1 2 1] on Y/U/V before the
  gate. Posterize, contours and depth tears all make sharp moving edges.
- One latency (23 → 29) in all modes; neutral outside avid before the
  smoothing, and gated again at the output.

Controls and mode scaling:
- **Depth and P12 glided** (a quarter of the way per field), and the gain kept
  at 10-bit knob resolution (v0.1: 9-bit). One gain step was up to 3.5 px of
  shift on high-contrast pixels, so pot noise could shimmer the displaced edges.
- **SD reach halved.** The offsets are HD pixels (±511 px clamp). At SD the
  same knob was 2.7× stronger relative to the frame; now it is ~1.3×.

Optimizations:
- `no_rw_check` on all 8 RAMs (each bank is read only while the other is
  written; the rings read 8 / 23 behind the write pointer).
- The contour test runs early on the still picture's luma and travels as one
  bit (v0.1 carried 10-bit luma 11 stages). The dry luma copy is gone: the
  base is the clamped A plus 64.

## Hardware checks for v0.2.0

1. **Default look.** Mode now really opens on Difference. v0.1 opened on Add
   until K5 moved. If the look you liked was Add, say so and I'll make Add the
   default.
2. K5 sweep: the label on the display should match the look in every fifth.
   K3 sweep: 2 / 4 / 8 / 16 / 32 / 64 should match the steps.
3. Multiply: whites stay white. Overlay: no seam through the mid-greys.
4. S11 contours are black, not tinted.
5. Analog colour source, S9 Shifted, Tint up: any thin coloured lines?
6. HD HDMI: any thick bars?
7. Depth and P12 sweeps are smooth with no edge shimmer (a slight glide lag
   is expected). Whites are a little dimmer (knee) and edges a touch softer
   ([1 2 1]).

## Review backlog (not done)

Tier 2, look changes (your call):
- **Tint follows the strata.** The tint keys on fixed luma bits 8 and 9, so
  there are only four tint bands whatever Levels says. At 64 levels the
  "alternating hues" change only every 256 codes. Proposal: key the tint to
  the current band index (alternating per stratum, a 4-hue cycle).
- **Real contour lines.** S11 blackens the bottom eighth of each band of the
  still picture. On flat areas that makes black slabs, on gradients bands,
  and they don't follow the displaced terraces. Proposal: 1–2 px lines where
  the output's band index changes (left neighbour or the line above).
- **S10 Key: does the background stay put?** "Above" sets Mix to 0 below the
  pivot, which shows the displaced layer, not the still picture. Option: show
  the still picture below the pivot, so the background really doesn't move.
- **P12 climax.** 100% is full mix plus double depth, and terraces appear only
  with S7 Before. Proposal: the top quarter of P12 also forces stepped depth
  and coarsens toward 4 levels, so 100% always separates into terraces.
- **Occlusion.** The shift is a gather at the destination's own luma, so layer
  edges tear and duplicate instead of the nearer layer covering the farther
  one. A nearer-wins pass is a bigger change.

Tier 3, optional:
- Capture defaults and presets from hardware (the 4 presets are hand-written).
- Full-resolution 10-bit chroma in the line buffer (now 8-bit, half
  resolution). Probably invisible.
- Width-exact SD reach (3/8 of HD rather than 1/2).

## v0.2.2 (2026-10-18): resolution scaling

- **The depth shift follows the screen height.** The offsets are pixels, and
  v0.2.0 only halved the gain at SD (a measured-width switch), so 720p shifted
  1.5x as far as 1080p relative to the picture. The gain now follows the
  timing ID's x step: x1 at 1080, x2/3 at 720p ((1/2 + 1/8) x 17/16 = 0.664,
  over vblank states 5..8), x1/2 at SD. The reach clamp scales the same way:
  511 / 341 / 255 px. SD looked the same as before at normal settings; only
  the clamp is new there. This also settles the "width-exact SD reach"
  backlog item: in height units, half is right.
- The 720p adds take one operand from a copy register (`s_ga`). x + (x >> k)
  on one signed register puts the sign bit on both carry inputs, which is the
  router1 self-add hang. dupin: 0.
- 1080p is bit-identical.

## Build

`./build_combo.sh stratamix` → `out/rev_b/ron/catalogue-combo/stratamix.vmprog`.

v0.2.0 (2026-10-13): all 6 configs pass on seed 1 (router2 default).
HD Analog 84.9, HD HDMI 85.5, HD Dual 83.3 MHz; SD Analog 77.2, SD HDMI
79.4, SD Dual 82.5 MHz. 5,003–5,045 LC (65–66%), 21/32 EBR. vmprog copy:
`.stratamix_work/stratamix_v020.vmprog`.

v0.2.2 (2026-10-18, router2): HD Analog 77.81 (seed 1), HD HDMI 79.76
(seed 1), HD Dual 77.93 MHz (seed 2: seed 1 stalled the router, a timeout and
not a timing miss); SD Analog 81.25, SD HDMI 83.83, SD Dual 81.58 MHz (seed 1).
5,204–5,224 LC (68%), 21/32 EBR. Standalone: 2,299 LUT4, `emulate_transparency`
0, dupin 0.

Standalone yosys (synth_ice40): v0.1.0 1,835 LUT4 / 1,427 FF / 20 EBR
(3 `emulate_transparency`); v0.2.0 2,242 LUT4 / 1,561 FF / 21 EBR
(`emulate_transparency` 0). The +400 LUT is mostly the labelled-knob
decoders, the 1/W product scaling and the bar guard.
