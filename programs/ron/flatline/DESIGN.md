# FLATLINE — design notes (proof of concept: image flattening / cel matte / traced ink)

Status: see SUMMARY.md for the build record. This file is the design: budget, pipeline,
what each build contains, and every deviation from the brief.

## 1. Budget (measured on this toolchain, iCE40 HX4K, 74.25 MHz)

| Resource | Chip | SDK core alone (passthru, measured) | Routable ceiling (CLAUDE.md) | Left for the program |
|---|---|---|---|---|
| Logic cells | 7680 | 1137 LC | ~94% = 7220 LC | **~6000 LC** |
| Block RAM (EBR, 4 kbit each) | 32 | 0 | 30 (router congestion cliff; edgelab shipped at 30) | **30 EBR** |
| DSP | none on the HX4K | | | every multiply is LUTs |

Measured conversion on this design (standalone yosys → routed): routed program LC ≈ 0.85 ×
(LUT4 + SB_CARRY + flip-flops). Flip-flops cost almost a full LC each here; a 40-stage
pipeline carrying 20 bits is 800 LC before any arithmetic.

Line-buffer costs (one full-width line, 1920 px needs 2048 entries):
- 2048 x 8 bit (full-res 8-bit luma) = 4 EBR; 2048 x 10 = 5; 2048 x 1 = 1.
- 1024 x 8 (half horizontal resolution) = 2 EBR; 1024 x 4 = 1; 1024 x 16 = 4.
- 512 x 16 (quarter horizontal resolution, U+V packed) = 2 EBR.

So at full resolution the whole chip holds 7 luma lines and nothing else. The brief's
pipeline needs, in the vertical direction: a 5-row smoothing window (4 lines), a 5-row
label window for the mode filter (4 lines of 8-bit labels), one cleaned-label line for the
boundary test, a 2-line ink delay, chroma, plus a shadow store and a temporal store.
That only fits with **horizontal decimation of the whole analysis path**, which the brief
explicitly prefers over adding line buffers:

- Luma is analysed at **half horizontal resolution** (pixel pairs averaged; 960 samples
  across at 1080p). Every luma/label line buffer is 1024 deep.
- Chroma is analysed at **quarter horizontal resolution** (4-pixel average, U and V packed in
  one 16-bit word; 512 deep).
- The composite runs at full rate: matte colours are held per pixel pair, ink alpha is
  interpolated between half-res samples (so lines are anti-aliased horizontally).

EBR map (all 1W1R, `no_rw_check`, read every clock / written the clock after the half-rate
enable so a read never collides with a write):

| Memory | Word | EBR | Present in |
|---|---|---|---|
| Raw luma rows r-1..r-4 (4 chained) | 1024 x 8 | 8 | all |
| Chroma rows r-1, r-2 (2 chained) | 512 x 16 | 4 | all |
| Label rows r-1..r-4 (row-modes, 4 chained) | 1024 x 8 | 8 | all |
| Cleaned label row above | 1024 x 8 | 2 | all |
| Ink history {line alpha x2, boundary bit x2} | 1024 x 2..6 | 1-2 | all (1 EBR without XDoG/Canny) |
| Delay ring {centre luma, chroma class}, 12 steps | 32 x 16 | 1 | all |
| Luma histogram (64 bins) | 256 x 16 | 1 | all |
| Dry video + sync ring | 128 x 32 | 2 | all |
| Shadow silhouette, 16 lines at quarter res | 8192 x 1 | 2 | builds with the shadow |
| Temporal label store, 512 cells | 512 x 8 | 1 | builds with the temporal vote |
| Canny magnitude rows {rho, rho-1} | 1024 x 8 | 2 | Canny builds |
| Canny direction + accept row | 1024 x 4 | 1 | Canny builds |

Base = 26-27 EBR; the richest build is 30.

## 2. Pipeline

Two clock domains in one clock: F (full rate, every pixel) and H (half rate, one step per
pixel pair, a free-running enable `ce` that is re-aligned at every active-video rise so the
pairing is stable on wandering analog lines). All H registers are plain clock-enabled flops;
the H pipeline never stalls and keeps draining through blanking, so each stream carries its
own valid/first bits and windows replicate the edge sample at both line ends.

F front end: input latch (U/V swapped to BT.601 order internally; the dry path never
swapped), previous-pixel register, active-pixel counter, hsync->avid parity vote, dry ring
write. The ring carries video AND syncs, so the output syncs are aligned with the video by
construction whatever the latency.

H pipeline, stage k holds half-res column m during step m+k (one step = 2 clocks):

| Stage | What |
|---|---|
| 0 | pair average of F0 and the previous pixel (Y, U, V to 8 bits); line-buffer reads at column m |
| 1 | newest luma column rows r-4..r (5 x 8 bit); line-buffer chain writes; chroma quarter sample |
| 2..13 | Stage 1 smoother of the build, from the newest column: median 3x3 (sorted-column network, 19 compare-exchanges), separable bilateral 5x5 (vertical then horizontal, 4+4 range-weighted terms), or Kuwahara 5x5 (range criterion). The identity centre rides the delay ring. All land at stage 14 |
| 15..18 | K1 select, K2 blend `c + delta x {1/4, 1/2, 3/4, 1}` (bilateral: full) -> smoothed luma ys18 |
| 2..8 | chroma: 2(v) x 2(h) quarter-res box on U/V, hue sector (8 / 4) + saturation level (3 / 2); the class rides the delay ring to stage 20 |
| 19..21 | Stage 2: luma bin = count of thresholds passed; +-3 band gives the two candidate bins; temporal vote, then left-neighbour vote; label = {lum3, hue3, sat2}. Histogram sample every 8th column |
| 22..26 | Stage 3 horizontal: 5-sample mode per component (lum / hue / sat), always on (this is the minimum-run-length filter); the row-mode is what the label line buffers store |
| 27..29 | Stage 3 vertical (S9): 5-row mode of the row-modes -> cleaned label CL (row = input row - 4) |
| 30..35 | Stage 4: boundary (CL differs from left / above / right), 1-3 sample footprint via a 2-row bit history in the ink RAM; XDoG alpha delayed 2 lines through the same RAM so it lands on the matte row; Canny (build C) delayed 1 line the same way. Soft blend toward the runner-up bin (builds with soft) |
| 30..37 | Stage 6 prep: silhouette (lum bin != 0) written to the 16-line shadow ring, read back at (row - dy, col - dx), soften = popcount over 2/4/8/16 samples |
| 36/37/38 | hand-off: {matte luma, hue, sat, ink alpha (2 bits: 0 / quarter / half / full), shadow alpha, background flag} as next / current / previous for the F-domain interpolation |

F composite (7 stages): representative-colour table, ink alpha interpolation (odd pixel =
rounded mean of this and the next sample), background/shadow/matte compose, ink blend by
shift (alpha 0/4/8/16 sixteenths), lines-only view, [1 2 1] output
conditioning on Y/U/V with luma scaled to white ~775 (the HD HDMI bar fix, processed path
only), P12 wipe against the dry ring, blanking gate on the parity-corrected avid.

Program latency: 84 clocks (`processing_delay_clks = 84`), identical in every mode.
Vertical registration: the processed picture is 4 input lines behind the raw (two 5-row
windows). The P12 wipe therefore shows a 4-line vertical offset at the seam; the raw half is
not line-delayed (that would need a full-width 30-bit buffer of several lines).

Per-frame sequencer (vblank, after the guarded vsync edge): fixed thresholds by repeated
addition; adaptive thresholds by ONE pass over the 64-bin histogram (equal-population
placement; see section 4); shadow offsets and the wipe column on one shared serial
multiplier. Everything the sequencer writes into pixel-path registers goes through a value
bus with a registered one-hot enable (threshold / centroid tables) or an op code that the
unit fetches itself (multiplier operands): the state decode fanning into scattered register
files was a 31 ns routing path, and into the multiplier's operand muxes a 17 ns one.

## 3. Control inventory (12 controls; see CONTROLS.md for the card)

K1 smoother (two labels: Off / this build's smoother), K2 strength, K3 levels (8 labels: 2..8
then one coarse-chroma step), K4 lines (4 labels: None / Edge / Edge White / XDoG, Canny or
Edge Bold), K5 ink weight, K6 shadow (0 = off, offset grows to the centre, past the centre it
softens; only in the build that has it). S7 Quant Hard/Soft (build E), S8 Levels
Fixed/Adaptive, S9 Cleanup Off/On, S10 Temporal Off/On (build C: Chroma gradient Off/On for
Canny), S11 View Final/Lines, P12 Wipe.

The brief's "line colour black / white" had no switch left, so it rides on K4 as the Edge
White label. Every stepped knob has a power-of-two zone count so the decode is a bit slice
with +-8 hysteresis and no compare chains.

## 4. What changed from the brief, and why

The first complete version of this pipeline synthesized to about twice the chip (10,500 LC
for the base plus the bilateral); every item below is what it took to get a build that fits.

- **Half-res analysis.** Matte boundaries quantize to 2 px horizontally at 1080p (1 px
  vertically); ink is interpolated so lines stay anti-aliased. Chroma classes quantize to 4 px.
- **Chroma window is 2 rows x 8 px (quarter res), centred 0.5 line above the luma window
  centre**, with no chroma hysteresis. The 2 x 20 binomial and the near-boundary vote logic
  cost ~400 LC; the horizontal mode pass cleans chroma specks instead.
- **Bilateral is separable (vertical 5 taps then horizontal 5 taps)**, the unnormalised form
  `c + sum w_i (x_i - c) / 32` with range weights {8,4,2,0} at |d| <= sigma, 2 sigma, 3 sigma
  (distance-2 taps halved). A full 24-term 5x5 bilateral was ~2200 LC on this part (24
  subtract/abs/compare/shift chains). Separable is what Winnemoeller's real-time abstraction used.
- **Kuwahara uses the quadrant RANGE (max - min) as the variance proxy**, not sum of squares
  (25 squarers do not fit). Range-Kuwahara is a known cheap variant; K2 blends toward the centre.
- **K2 blends in quarters** (1/4, 1/2, 3/4, 1), not with a multiplier (−120 LC).
- **Adaptive thresholds are equal-population (percentile), not k-means / Otsu.** A k-means
  engine (serial multiplier, divider, cluster register files) was ~1000 LC; the one-pass
  percentile placer with a 3-bit divider is ~350. Thresholds and the bin medians (used as the
  representative luma) are low-passed 1/4 per frame and kept monotonic, as the brief asked.
- **Soft quantization is a count-weighted blend, not Winnemoeller's tanh.** The tanh form needs
  the smoothed luma at the matte row, two lines after it was computed; carrying it costs 4 EBR
  we do not have. Instead the vertical mode pass reports the runner-up bin and its vote count
  (0..2 of 5), and the matte luma blends toward the runner-up's centroid by that fraction; the
  composite also half-steps luma across horizontal label changes. Ramps are ~4 lines / ~4 px.
- **Mode filter is separable** (5-sample mode per row stored in the line buffers, then the mode
  of 5 row-modes), per component (lum / hue / sat). A true 5x5 mode over 192 labels would need
  popcounts of 25 samples per symbol. Separable modes can mint a label that did not exist at a
  three-region junction; everywhere else they agree with the 2D mode.
- **The minimum-run-length filter IS the horizontal mode pass**, always on (a label survives
  only with 3 of the 5 samples around it); S9 toggles the vertical pass. A separate run-length
  FIFO was redundant with it and cost four pipeline steps.
- **Hysteresis votes: temporal, then left neighbour. The "pixel above" vote was dropped**: the
  label line buffer has one read port and the mode filter owns it at a different column.
- **Temporal stabilisation is a label vote, not an IIR, and it is self-gating**: a pixel only
  takes last frame's cell label when it is within 3 codes of a bin boundary AND the stored label is
  one of the two candidates. Moving content crossing a boundary by more than the band updates
  normally, so there is no ghosting and no motion mask is needed. Cells are 128 px x 64 lines
  (512 cells, 1 EBR); the stored label is the cleaned label at the cell centre.
- **Ink alpha is 2 bits** (0 / quarter / half / full) blended by shifts; the three 11x5
  multipliers of a 16-level alpha were ~400 LC. Horizontal interpolation still gives 7 levels.
- **Open/close is not built** (needs 2 more label line-buffer pairs = 8 EBR); the horizontal
  mode plus the vertical mode cover island removal. **Median+bilateral two-pass is not built**
  (needs a second 5-row window). Both are the two lowest items in the brief's priority list.
- **"XDoG" is the DoG edge, not the thresholded sharpened image.** It runs on the raw luma
  rows (the window stores raw luma because the median and Kuwahara need raw rows); the two
  scales are fixed by the 5-row window (3x3 vs 5x5 binomial). Ink goes on the dark side where
  -DoG exceeds a K5 threshold of 0.5 .. 4.4 codes (the 99th percentile of |DoG| on a photo
  is ~10 codes), alpha 1/4 .. 1 over the next 2 codes. The Winnemoeller form (G1 - tau G2 < eps,
  which also fills every dark region) was what v0.1.0 built; with eps = 0..31 codes it inked
  the whole picture, and the fills are not what an ink layer over mattes wants anyway.
- **Shadow silhouette is stored at quarter resolution, 16 lines**: offset up to 15 lines /
  15 px at 1080p in 1080-line units (scaled per mode), softened by a horizontal popcount.
- **Diagnostic view is lines-only on grey** (the brief's own fallback). The 2x2 split (smoothed
  input / labels in a designer palette / lines / final) was built first and cost ~150 LC it
  turned out the chip did not have; the designer palette went with it.
- **K3 has 8 labels** (2..8 and one coarse-chroma step), K4 has 4 and K1 has 2, so every zone
  is a bit slice of the knob and needs no compare tree. XDoG/Canny therefore have no white
  variant (Edge does); builds without them get "Edge Bold" (3x3 footprint, full ink) instead.
- **Bilateral sigma is quantized to octaves** (4..128 codes, six K2 steps): the range weight
  is then a bit test on |d| >> log2(sigma) instead of three 9-bit compares per tap.
- **Line colour and designer palette** folded into K4 labels and the split view (no switch left).
- **Canny uses 4-bit magnitudes in its NMS window** (two rows in one 1024x8 RAM) with a
  K5-controlled pre-gain of 2..17, thresholds fixed at 8/4 of 15: a strong edge is a step of
  256/gain codes (128 at K5 = 0, ~25 at 50%, 15 at 100%). 6-bit magnitudes would cost 1 more
  EBR. (v0.1.0 scaled the product 4x lower, so nothing under a 114-code step drew a line.)
- **Builds are narrower than planned.** With the base at ~73% of the chip, each build holds one
  smoother plus one line algorithm (plus the cheap extras where they fit); see SUMMARY.md.

### Fixed after the first renders (v0.1.1, 2026-11-09)

The GHDL preview renders (see SUMMARY.md, "Previews") caught four things no build report shows:

- The dry path read U/V back from the delay ring in the wrong order: the raw half of the wipe
  would have been hue-swapped on hardware.
- The representative-colour table gave the odd sectors the small cosine. True for sectors 1
  and 5 but not 2 and 6 (and vice versa for 3/7), so red rendered as yellow-orange and blue as
  teal. Found with a synthetic hue wheel (16 hues x 2 saturations) through the hardware-order
  emulation; the fixed table maps all 32 patches to their own sector centres.
- The XDoG threshold inked the whole picture (above).
- The Canny gain range was 4x too low (above).

Lesson for the next program: render a hue wheel and a wipe through the sim with the input
pre-swapped and the output post-swapped (that is what the hardware does) before calling a
palette done; the sector/table symmetry can hide a crossed table on real photos.

## 5. Resolution consistency (warnings for Ron)

Geometry that follows the screen height: the shadow offset (1080-line units). Everything else
is pixel-scale and the brief treats kernels as pixel textures: smoothing/mode windows, run
length, line widths, temporal cells. At SD a 1-sample line is 2 of 720 px (about 1.5x heavier
relative to the frame than at 1080p) and the 5x5 windows cover 3x more of the picture. 720p sits
between. HD HDMI is the target; SD/720p will look coarser.

## 6. Per-variant cost
See SUMMARY.md (standalone yosys per feature, and the routed result per build).

## 7. Timing closure (what it took at 74.25 MHz, 86-89% LC)

Every build routes at 86-89% of the HX4K, so a path is ~60% routing: the rule that held
was **one carry chain or one barrel per stage, nothing fused**. The paths that missed, and
the fixes (all at unchanged latency):

- **Sequencer register-file writes** (first full route, 43-57 MHz): per-state writes into
  the threshold/centroid files decoded from the state register were a 31 ns routing cone.
  Now every write goes through one value bus (`thr_wd`, `cen_wd`) plus a registered one-hot
  enable, and each sequencer state does one operation (compare OR subtract OR write).
  Multiplier operands are fetched by a 2-bit op-code (`mul_op`), not muxed from the state.
- **Vertical mode pick** (54 MHz): a sequential 5-way best-count compare chain became the
  parallel threshold argmax `f_argmax5` (all candidates compared at once, shallow tree).
- **Bilateral range level** (build A, 70-72 MHz on every HD config, router1 seeds 1-6):
  `difference -> |d| -> barrel shift by sigma -> level` was one stage per pass (4.7 ns
  logic, 9 ns routing). Split into difference (stage 2 / 11) and level (3 / 12), paid for
  out of the vertical pass's alignment delay (two delay stages instead of four), with a
  private `keep` copy of the sigma shift per pass loaded under a different enable so yosys
  cannot merge them. Diagnostic route 72.3 -> 82.2 MHz (HD Analog, router1 seed 1).
- **Shadow soften popcount** (build E, 52-54 MHz on both routers): a 16-sample serial
  `v_n := v_n + 1` count in one stage was 7 ns of logic (a 16-deep incrementer chain).
  Now group counts (LUT-only 2- and 4-bit popcounts) -> pair sums -> window sums -> select,
  three extra stages; the shadow fetch moved two quarter-columns earlier and the window
  centre one sample later, so the hand-off alignment is unchanged.
- **Router choice**: B, C, D close on the builder default (router2) with a few seed
  retries. A is built with `NEXTPNR_ROUTER="--router router1"`: on the un-split bilateral
  router2 missed every seed and router1 got within 2-4 MHz, so the fixed A kept router1.
  Seeds and routers per config are in SUMMARY.md.
