# Rorschach — Heirloom #4, rebuilt from the photo

**v0.5.0 (2026-10-21): built, NOT flashed.** Rebuilt from `photos/WIN_20221124_22_54_05_Pro.jpg`
after Ron judged v0.1.0 too abstract. Osc mode now redraws the photo itself:
- the folded pastel inkblot with the blue "head" at the right edge;
- the blue arrowheads and the red/blue cross on the fold;
- the blue-and-cyan "river";
- the busy torn strips either side of the fold;
- the red/blue stair-steps.

Video mode puts the camera through the same processing chain. Work files:
`.heirloom_work/rb_rorschach/` (`notes.md`, `protof.py`, `imodel.py`, `sheet.png`, `cmp_final.png`).

## Reading of the photo (measured)
- **A fold, not a quad mirror.** Flip-correlation finds one axis: rows r and 1051 − r (axis 525.5).
  - The mean |diff| there is 24.6, against 60 for random pixel pairs.
  - The best left/right axis scores 50, which is baseline: there is no left/right mirror.
  - Smooth rows match to within 3–10; busy rows differ by 30–47. The lower half is the same scene
    with different fine detail: a frame-store mirror showing a later frame.
- **Camera content.**
  - Organic shapes: a blue head-and-shoulder at the right edge, a spiky blue/cyan river,
    curtain-like vertical bands.
  - Edges are soft, ~10–15 px horizontally and ~4 rows vertically: an SD source upscaled by the
    capture.
  - Bands are perfectly flat inside: a posterizing colourizer.
- **Palette** (flat pixels, 60 % of the frame):
  - cream (254,255,178) 26 %, peach (255,231,183) 26 %, pink (254,193,228) 11 %;
  - orchid (253,195,251) 3 %, lavender (195,195,254) 2.3 %, pale cyan (188,254,253) 1.6 %;
  - blue (30,23,249) 5.6 %, red (254,21,3) 1.9 %, magenta 0.2 %, green 0.1 %.
- **Cyclic colourizer.**
  - Colour adjacencies (runs ≥ 3 px) form one chain: cream–peach 2400, peach–pink 5774,
    pink–orchid 3616, orchid–lavender 2496, lavender–pale cyan 1701.
  - Pale cyan meets blue as a pure two-colour blend; cream meets the dark band directly (blue 1032,
    red 581).
  - Order: dark band → cream → peach → pink → orchid → lavender → pale cyan → back to the dark
    band. That is 7 bands on a wrapping scalar.
- **The red/blue stair-steps are the camera's hue, sample-and-held.**
  - Red/blue risers sit on a fixed grid: period **42.25 px**, phase 10.5 (concentration 0.995).
    That is 16 SD px, 45 cells per line. The treads fall on any row (no y grid).
  - Only blue↔red boundaries are on the grid (100 %); every other colour pair is at chance (15–24 %).
  - Within one grid cell, row 200 runs cream → blue while row 240 runs cream → red → blue. So red
    versus blue is not a band of the scalar: it is the camera's hue (the head's blue hair and red
    face), held per cell, colouring the darkest band.
- **Texture.**
  - The vertical spectrum runs to Nyquist while the horizontal one falls off: the picture is
    horizontally streaked.
  - Busy strips sit at |y − fold| ≈ 75–230 rows (row busyness 0.28 there against 0.11).
  - No row is shifted as a whole. The tears are local strips (rows from elsewhere, smeared), not
    line tearing.

## What it does
1. **Store.** The half above the fold is held as a 64 × 128 wrapped-phase field (30 px × ~4 lines
   per cell at 1080).
   - Osc: a ROM fitted to the photo. The photo's band labels were turned into a ramped phase (POCS),
     then the 8192 cell values were optimised (hinge loss on the bilinear readout). 70 % of
     half-res pixels land in their photo band.
   - Video: the camera written live, one value per 30-px column per store row: frac(luma × 2 ×
     Bands), 4 bits, from an IIR-smoothed luma, plus a 2-bit hue class.
   - Four banks by (row parity, column parity), so all four bilinear corners are read in one clock.
2. **Fold.** Each output line bounces through the store rows (Mirror) or wraps (Repeat). The fold
   line is K5.
3. **Readout.** A wrap-aware bilinear lerp that steps instead of interpolating across jumps of more
   than 13/64 cycle. Then × Bands (Osc), Negative, + fragments, + Level and the colour cycle.
4. **Colourizer.** 7 bands of an 8-bit phase. The dark band takes the hue map (camera hue, or the
   photo's in Osc), sampled at the first pixel of each 42.25-px cell and held across the cell.
5. **Fragments.**
   - Where the field is busy (the corners disagree by more than 12/64), the phase gets a hashed
     offset per (cell, 4-line group), up to ±1.2 cycles at the default K3.
   - The dark band's hue in busy cells is hashed too: blue 45 %, red 27 %, magenta 18 %, green 10 %.
6. **Tears.** A background tear-group unit draws groups of 1–37 virtual lines.
   - At the default 35 %, a group tears with p = 30 % in the busy zone (70–235 lines from the fold),
     5 % near the fold and 17 % far from it. At 100 % the rates rise to 100 % / 62 % / 74 %.
   - A torn group jumps ±27 lines (±78 at 100 %) and shifts ±67 px (±500 at 100 %). 30 % of torn
     groups smear, holding the colour for 2–9 columns.
7. **Colourway (K6).** A chroma or luma transform of the photo palette: Ink (no chroma), Swap,
   Comp, Turn, Twist (U/V swaps and quarter turns), Night (luma negative), Vivid (2× chroma).

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | number of colour bands: colourizer gain 0.5–4 cycles, square law (1 = the photo; Video ×2) |
| K2 Level | colourizer offset: the bands crawl through the cream → … → pale cyan → dark cycle (0 = the photo) |
| K3 Detail | the busy-strip fragments: 0 = clean blobs, 60 % = the photo; above that they spill into the smooth bands, and at 100 % the inkblot dissolves into fragments |
| K4 Speed | colour cycling, bipolar, still at the centre (±8); default slow forward (one cycle per ~33 s) |
| K5 Fold | the fold line, 25–100 % of the frame height (48.7 % = the photo; low = repeated folds, 100 % = unfolded) |
| K6 Palette | Rorschach (the photo) / Ink / Swap / Comp / Turn / Twist / Night / Vivid |
| S7 Mirror | Mirror (the photo) / Repeat: the half above the fold reflected, or repeated |
| S8 Stairs | Off (the photo: only the hue stair-steps) / On: the whole dark band sample-and-held into 42-px blocks |
| S9 Split | Hue (red / blue / magenta / green dark band) / Blue (all blue) |
| S10 Negative | the scalar inverted: the band order reversed |
| S11 Source | Video (default) / Osc |
| P12 Tear | frame-store tears: 0 = a clean fold, 35 % = the photo, 100 % = torn into displaced slabs |

## Video
Video mode is the photo's own processing chain applied to the camera.
- The camera's top part, down to the fold, is written into the store live, so it is folded and
  posterized by the same cyclic colourizer.
- Its dark band takes the camera's hue: a 2-bit class of the column's last pixel (red, magenta,
  green, else blue), held per 42-px cell, so warm objects stair-step red against blue.
- Fragments and tears act on it as on the scene.

Osc substitutes the photo's scene. Every control acts the same in both modes.

The HX4K has no frame store, so the camera is stored at 64 × 128 cells with 4-bit phase. Video
shows soft blobs of the camera, not its fine detail.

## Engine
- **Per field (vblank zone):**
  - 5 glides (K1, K2, K3, K5, P12; 1/8 per field, ±4 deadband).
  - 9 slots on one 20 × 18 serial shift-add multiplier, with operands registered a phase early:
    the gain, the video gain, speed² and the colour-cycle rate, the fold, t², t⁴, the spill.
  - The P12 tear curves by shift-add.
  - 4 serial divisions: columns and S&H cells per pixel from the measured width; the virtual-line
    step from the measured active lines; store rows per line from the fold.
  - The picture is therefore the same at SD, 720p and 1080, and in decimated sims.
  - The colour-cycle accumulator; reseed every 4 fields.
- **Per line:**
  - An 8-state engine, 24 clocks after the source avid fall, writes the next line's registers:
    advance and fold, tear probability, tear group, store row / fraction / bank addresses, the hue
    row, the video write row.
  - The tear-group unit draws the next group in the background (xorshift32 plus a 22-bit serial
    multiplier, ~45 clocks).
- **Beam:** column and S&H-cell accumulators.
- **Pixel path, 24 stages:**
  - bank and hue addresses, and the fragment hash;
  - EBR, then corners and the Osc/Video select, with the hue held from the cell start;
  - wrapped differences and skips, then y products and lerps, then the x difference and product;
  - phase → gain → Negative → + fragments + offset → band;
  - Stairs, hue and the smear hold → palette → colourway;
  - conditioning: luma [1 3 4 4 3 1]/16 smear, chroma [1 2 2 2 2 2 2 2 1]/16, luma 13/16, then the
    blanking gate on the generated avid.
- **Video write:** per source pixel, a 1/16 IIR (1/8 at SD); one event per 30-px column; gain
  product; 4-bit phase and hue class into one bank. `no_rw_check` is on the camera RAMs.
- **EBR (24):** scene 4 × (2048×4 + 2048×2) = 12, scene hue 4096×2 = 2, camera 4 × 2048×4 = 8,
  camera hue 2.
- **Palette:** case-function logic. Luma is stretched to 940 and chroma scaled by 13/16, so the
  output conditioning's luma × 13/16 keeps the photo's hue ratios.
- **Contracts:** damier's generated avid; one 24-clock latency in every mode; colours in the hardware
  convention (U = Cr); per-field work anchored on the vblank zone (no avid for 1024 clocks).

## Status
- **v0.5.0 built 2026-10-21, NOT flashed.** router2. Build seeds: **HD Dual = seed 3**; the other
  five are seed 1.
- Fmax (post-route; every config's last line PASS):
  - HD Analog 78.06, HD HDMI 75.47 (thinnest), HD Dual 80.37 MHz;
  - SD Analog 76.10, SD HDMI 76.35, SD Dual 76.25 MHz (SD Fmin 27).
- 6928–6948 / 7680 LC (90.4 %), 24/32 EBR. Every bitstream passed iceunpack.
- Fit history: the first RTL (value-noise knot unit, address holds, 8 palettes as logic) placed
  9129 LC (118 %). The restructure brought it to 90 %, and the serial multiplier's operand mux
  (71.9 MHz) was pipelined.
- Verified in GHDL (960 × 540) against the integer model (`imodel.py`): mean colour equal within 1
  level. Motion: 28.5 % of palette labels change between frames 3 and 7, about 50 % in the busy
  strips and about 17 % in the bands.
- Presets are hand-written placeholders: Photo, Ink Card, Kaleido, Shredded.

## Deviations from the photo
- **Fragments are procedural.** The busy strips' small fragments come from one hashed value per
  sample-and-hold cell, gated by the busy detector. They are a different realisation, and a little
  finer and less red/green/magenta than the photo's.
- **Exact mirror.** The photo's lower half differs in fine detail (a later frame); here the
  stored half is mirrored exactly (the tears and fragments still differ).
- **About 84 % as bright.** The required HD HDMI bar fix caps drawn luma at ~777. The palette's
  chroma is scaled to match, so hues are the photo's, just dimmer.
- **Fixed sample-and-hold cell.** The cell is fixed at the photo's 42.25 px. A Hold knob was too
  subtle on the control sheet, so K2 became Level.
- **No separate seam line.** The photo's 3-row seam line at the fold isn't drawn; tears near the
  fold occasionally make one.
- **Soft Video picture.** The camera store is 64 × 128 cells, because there is no frame store.
- **SD 4:3.** The same composition is squeezed to 4:3, so the head goes oval. This matches the
  original SD patch: the cell is exactly 16 SD px.
- **Video read-during-write.** Top-half store rows are read on the same line they are written; a
  pixel may read a cell during its write (undefined EBR data: at most a 1-px speckle, no state
  affected). Watch for it on hardware.
