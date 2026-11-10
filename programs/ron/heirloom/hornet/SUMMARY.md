# Hornet — a band of hornet faces rolling up through screen-locked cuts

**v0.1.1 built 2026-11-10, not flashed** (see [v0.1.1](#v011-review-fixes)). Heirloom phase 3 (video clips), from clip `WIN_20240207_23_34_28` at 0.5 s
(`photos/vid/WIN_20240207_23_34_28_t0p5.png`). Across the middle of the frame runs a band of "hornet faces": two red
eyes with grey pupils, four leaning yellow lemons set in dark crescents, yellow Y-ridges and cyan brows. Above it, fine
red / cyan / black stripes stepped by a square wave; below it, wavy white / red / cyan bands and rounded cyan / red
blocks; the whole picture is cut every 65 lines by a screen-locked sawtooth. Presets are hand-written placeholders.

## v0.1.1 (review fixes)
The orchestrator's review of v0.1.0: the lemons read as narrow yellow slivers in mostly-black jagged crescents, the
pupils as specks, the band too red, and the default camera bend frayed the eye band. Changes:
- **Lemons.** The key fill now folds: from orange at the key edge it rises to yellow at a depth of 0.095 cycles and
  folds back to an orange core, so each lemon is a fat yellow body with an orange centre filling most of its crescent.
  The key path delay is 28 virtual px (was 44), so the black rim on the left is thinner and the dark red stays on the
  right; the key noise is halved (±1/128 cycle) so the edges jag less.
- **Pupils.** Flat grey discs in the still's grey (158, 141, 142), about 40 × 25 px (the still's are 42 × 26), drawn
  over everything; they were 0.45-cycle dips that came out as dark specks.
- **Reds.** The palette's dark-red group is wider and the bright-red group narrower (17 / 15 % of the cycle, was 10 / 22 %).
- **Camera.** The luma is now an 8-px box, every second sample kept, and a vertical IIR (1/4 per line) in a 1024 × 8
  line memory whose one read port alternates between the IIR's read-modify-write and the beam. Default K6 is 150
  (square law: a quarter of v0.1.0's bend); the Osc multiplier went from 3× to 12× the camera's, so the Osc flip is
  as strong as before (the same gain; the smoothing takes some lattice detail off, so the measured flip, the mean
  |Osc − black| of the RTL renders, is 92 vs v0.1.0's 118). On selfie.jpg the contours stay clean curves (camera bend
  |camera − black| 29, was 42).
- **Measured (integer model vs the still, 1080p default).** Colour shares dark / red / yellow / cyan / white: still
  24 / 22 / 13 / 17 / 6 %, v0.1.0 24 / 31 / 13 / 11 / 7 %, v0.1.1 26 / 24 / 15 / 11 / 7 %. 4 × 4 region-mean error
  13.8 → 11.6 (max 29.3 → 25.4). The RTL 720p renders agree: red 31 → 25 %, 4 × 4 error 15.1 → 12.9.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_hornet/` (`notes.md` has the full log).
- **Palette.** A 48-colour cyclic 1-D palette (SOM fit, `som.py`) reconstructs the still to 15.7 mean RGB error. In
  order: dark red → red (238, 7, 0) → orange (251, 136, 5) → yellow (252, 193, 16) → pale yellow → white
  (234, 238, 228) → light cyan → cyan (45, 226, 244) → steel blue → grey (137, 126, 133) → green → dark green → black.
  Pixel shares (k = 14): black 16 %, cyan 12 %, red 12 %, light cyan / white 10 %, dark reds 15 %, yellow 6 %,
  orange 6 %, dark green 5 %. Channel split: R is mostly saturated with sawtooth ramps that reset at hard, line-jittered
  edges; G and B are smooth bands of one field (B a higher threshold of it) — an analog colouriser on one field.
- **Periods.** Horizontal autocorrelation: the eye band repeats every 950 px (two faces), the stripe steps and the
  blocks every 574 px (edges at x = 265 + 287 k, ~30 px slews).
- **Field.** Per-row fit (`gfit2.py`: wrapped IRLS of the palette position of every pixel, lemon interiors
  un-inverted) of g = V + Σ harmonics of the 950-px oscillator + D·square. In the band the 475-px 2nd harmonic
  dominates (0.21–0.24 cycles: lemons and ridges), the 950-px term runs −0.18 at the eye row to +0.12 below it, the
  3rd harmonic reaches −0.21 at row 360 (the V under the eyes). Above and below, V sweeps the palette every ~17 lines
  at the top, ~30 lines at row 200, and goes flat in the block rows (780–1030), where the square wave shows as blocks.
- **Lemon edges.** Measured across lemon 1: outside-left (25, 0, 0) / inside (250, 100, 0); inside-right
  (255, 67, 0) / outside-right (214, 18, 0). The jump differs on the two sides, which fits a window key whose path is
  delayed and which replaces the field with its own fill (orange at the edge, yellow body, orange core). The delay
  puts the black crescent on the left of each lemon and dark red on the right; comparator noise makes the edges jag
  by ±3–6 px. The pupils are flat grey (158, 141, 142) discs, 42 × 26 px.
- **Cuts.** Hard row discontinuities on a fixed grid, rows 53 + 65.2 k; strong at the top and bottom, invisible in the
  middle of the eye band (rows 377–511).

## Motion in the clip
- All 623 frames were extracted. Tracking the eye band: 424 (0.4 s) → 368 (1.0) → 288 (2.0) → 180 (3.0) → 80 (4.0 s),
  ~90 lines/s upward (56 lines/s in the first second); the next band enters at the bottom and passes 864 (8.0 s) →
  700 (10.0 s). One period of the vertical structure is ~1150 lines, a little taller than the frame.
- The cut grid does not move (phase 50–54 lines mod 65.2 in every sampled frame): it is locked to the screen while
  the picture rolls through it.
- Ron's knob over the first 2 s is the lemons' lean: at 0.0 s each pair mirrors ("\ /", an angry face), at 0.5 s (the
  still) all four lean "/", by 1–2 s they lean further and stretch, and by 2–4 s they fold into W shapes.
- In the program: content period 1150 virtual lines rolling 1.5 lines/field up at the default Roll (90 lines/s at 60 Hz),
  the cuts locked to the screen, and the lean on P12 (0 % = the 0.0 s angry face, 16 % = the still, 100 % = swept W's).
  `cmp_motion.png` puts the clip at 0.5 / 1.0 / 1.5 / 2.0 s over the integer model at fields 1 / 31 / 61 / 91.

## What it does
1. **Content row.** yr = (screen virtual line + roll) mod 1150. The per-line engine reads three fitted table ROMs
   (256 entries over the period, linearly interpolated): V, E1, E2, E3, D, the cut depth KC and the key-row flag.
2. **Field.** θ = (x − 535) / 950; g = V' + E1 cos θ + E2 cos(2θ + lean (yr − 470) / 475) + E3 cos 3θ + D sq(x)
   + KC saw65(y) + Etch·luma (palette cycles, mod 1). V' = V_band + Stripes·(V − V_band).
3. **Pupils.** Flat grey discs (the still's grey), about 40 × 25 px, at every eye-oscillator peak on content row 300
   (the disc test uses cos θ and a 16-entry row profile, so no extra multiplier).
4. **Key.** frac(g + noise) > 0.70 on the lemon rows selects the fill: with d = −g, f = 0.33 + d up to d = 0.095,
   then 0.52 − d (orange edge → yellow body → orange core); elsewhere f = g. The key bit rides a 28-virtual-px delay
   (28 / 19 / 14 px at 1080 / 720 / SD); the noise is a new ±1/128-cycle draw every second computed line.
5. **Colour.** 256-entry palette (Fire or Wasp) at f; chroma through two [1 2 2 2 2 2 2 2 1]/16 boxes (running sums
   with the resync), luma [1 2 1] and 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eyes | the eye oscillator (950 px) and its 3rd harmonic: eyes, brows and the V under them; 0 = no eyes (lemons and ridges only) .. 512 = the still .. 2× = eyes swelling over the band |
| K2 Lemons | the lemon / ridge harmonic (475 px): 0 = no lemons (red eye band) .. 512 = the still .. 2× = big lemons with cyan cores |
| K3 Stripes | the vertical sweep: stripe density at the top, band spacing below, the band's own vertical shading; 0.3× (wide bands) .. 1× (512) .. 3× (hairline stripes) |
| K4 Roll | bipolar (±8 deadband, square law): the whole structure rolls up (right) or down (left); 742 = the clip's 90 lines/s up, full = 7.7 lines/field |
| K5 Blocks | the 574-px square wave: steps on the stripes and the rounded cyan / red blocks; 0 = plain bands .. 512 = the still .. 2× = deep steps |
| K6 Etch | how far the camera (or the Osc) bends the field: 0 .. 2.5 cycles at full luma (square law), 150 = 0.054 cycles (camera, smoothed); the Osc gets 12× the camera's gain |
| S7 Cuts | Fine (the still's 65-line screen-locked cuts) / Shred: 16-line cuts across the whole picture, eyes and lemons included |
| S8 Colours | Fire (the still) / Wasp: yellow, black, white and cyan (the palette of clip 23_33_13) |
| S9 Twin | 2 faces across / 4 faces (8 lemons, smaller pupils) |
| S10 Key | On (keyed yellow lemons, the still) / Off: the raw field, lemons become dark-cored rings |
| S11 Source | Video / Osc |
| P12 Lean | the lemons' lean (Ron's knob in the clip): 0 % = mirrored pairs, the 0.0 s angry face; 16 % (160) = the still; 100 % = 1.6 px/line shear, lemons swept into diagonal strokes and W's |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16, smoothed (an 8-px box, every second sample, a
  vertical IIR of 1/4 per line), is added to the field, so every contour (stripes, eyes, lemon keys) bends around the
  picture in clean curves. At the default Etch the bend is gentle and the faces stay intact; K6 at max bends the
  band into the camera's shapes. With no input, Video mode is the still.
- **Osc mode.** The shell's drifting 640 × 540 triangle lattice feeds the same path at 12× the camera's gain (as
  strong as v0.1.0's Osc), so the faces wobble, split and tilt at the default Etch; Etch scales it.

## Engine
- **Camera path.** 8-px running box (cleared off the source avid), every second sample to a 1024 × 8 line memory
  with a vertical IIR (new = old + (box − old + 2) / 4, a pipelined read-modify-write, written back 3 clocks after its
  read); the beam reads the same port on the other clocks.
- **Per-field sequencer (vblank zone).** Shared glide for K1, K2, K3, K5, K6 and P12; one serial shift-add multiplier
  (6 slots: roll speed², the Stripes lower slope, etch², etch gain, lean, the cut-saw step per line); the knob scales,
  the oscillator start phases per mode (beam starts 8 px early for the output filters), Twin, the palette black.
- **Per-line engine** (kicked at each line start, computes the NEXT line in ~230 clocks, fits SD's 858): content row,
  table position (yr × 14589), two reads of each table, five interpolations, the Stripes / Eyes / Lemons / Blocks
  scales, the cut (KC × |saw|), the lean phase, the pupil profile (16-entry ROM), the key noise (LFSR, a new draw every second computed line).
  One 30-bit serial multiplier, 17 clocks per product. **Line pairs:** at 1080p and 720p the engine runs on even
  lines only and odd lines repeat their constants, so no feature of the field is ever one row tall (thin-line
  checks: 0 runs at the defaults, P12 0 / 100, Shred, K3 max, Twin, Wasp, Key off, K5 max); SD and interlaced
  lines are already ≥ 2 virtual rows.
- **Pixel path (25 stages).** θ and square-phase accumulators; 2θ + lean and 3θ; quarter-wave addresses; three cosine
  ROMs (EBR) + fabric copies; envelope × cosine as hi / lo 6-bit partial products; terms; three adder stages; key
  window, the folded fill and the pupil flag; the 32-bit key delay; three padding stages (one latency); palette ROM
  (EBR) + fabric copy; compose; chroma
  boxes A / B with their 8-sample delays in one 16-deep block-RAM ring (`no_rw_check`); luma [1 2 1]; 13/16; gate.
- **ROMs.** Cosine 3 × 256 × 12, tables 256 × 32 + 256 × 32 + 256 × 16, palettes 512 × 32, the 1024 × 8 camera
  line memory, the box ring: 18 EBR.
- **Contracts.** Generated avid; one latency (C_LAT 24) in every mode; virtual 1080-line coordinates (SD shows the
  centre 4:3); colours in the hardware convention (U = Cr); the chroma running-sum resync; the HD HDMI conditioning
  (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16) before the blanking gate.
- **Verified.**
  - imodel.py (RTL widths) against the float proto: the same picture (`cmp_im2.png`).
  - GHDL 720p60 dec 1 against imodel: 98.90 % of the v0.1.1 default Video frame (black input) is bit-exact once the
    tester's 4-px capture offset is taken out (the generated avid starts 3–4 clocks after the source's; v0.1.0:
    98.65 %). Every mismatch is on the key rows (the key-noise LFSR phase). The frame matches the model's field 3 and
    not field 4, so the roll steps once per field.
  - The Osc path is not modelled bit-exactly (the model's lattice phase differs); the camera smoothing was checked on
    selfie.jpg in the model and in the RTL render.
  - Thin-line checks on the model (`thincheck.py`, `thincheck2.py`): 0 runs.

## Status
- **v0.1.1 built 2026-11-10, NOT flashed.** Router1 (`NEXTPNR_ROUTER="--router router1"`), all six configs on
  **seed 1**. Post-route Fmax: HD Analog 85.54, SD Analog 88.92, HD HDMI 88.24, SD HDMI 86.06, HD Dual 83.74,
  SD Dual 85.39 MHz. 7147–7167 / 7680 LC (93 %, about 60 over the ~7,100 guide; timing has 9–15 MHz of margin), 18 / 32
  EBR. Every bitstream passed iceunpack; the packed config was
  regenerated. `out/rev_b/ron/heirloom/hornet.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh hornet`.
- v0.1.0 (same day): all six on seed 1, HD 84.9–86.6 MHz, 6877–6903 LC, 20 EBR. The v0.1.1 camera smoothing, key
  fill fold and pupil discs cost ~+270 LC; the camera line memory halved (−2 EBR). An earlier v0.1.0 build used
  7392 LC (96 %) and was restructured before shipping (chroma box delays into a block-RAM ring, luma [1 2 1], a 30-bit
  line multiplier, 8-bit palette padding).
- Presets are hand-written placeholders (Photo, Angry Face, Swept, Wasp Nest, Shredded, Hollow); capture Ron's looks
  with `./capture_preset.sh`.
- Working files: `.heirloom_work/rb_hornet/` (`notes.md`, `proto4.py`, `gfit2.py` / `fit5.py` fits, `imodel.py`,
  `sheet.png`, `cmp_final.png`, `cmp_motion.png`, `metrics.py`, `thincheck.py`, `thincheck2.py`; v0.1.0's renders in
  `v010_imgs/`).

## Deviations
- The field is a fitted approximation of an analog patch: contours sit within ~10–40 px of the still's, the crescents
  around the lemons are redder (the still has more dark green there) and cyan is short in the brow and wavy-band rows
  (colour shares of the default frame vs the still: dark 26 / 24 %, red 24 / 22 %, yellow 15 / 13 %, cyan 11 / 17 %,
  white 7 / 6 %). Mean RGB error 45, 4 × 4 region-mean error 11.6.
- The still's dark-green "X" glows in the black band below the blocks are not drawn (the band is plain black); the
  hidden 70 lines of the 1150-line period (below the still's bottom) are black.
- In the clip the face shapes morph as the band rolls (lemons → W's → big white-ringed eyes) because Ron was also
  turning knobs; the program's band keeps its shape while rolling — the morph is on P12 (Lean) and K1 / K2.
- The lemons are bean-shaped (a rounded "C" with a jagged left edge) rather than the still's clean teardrops: the
  key follows the fitted field's contour.
- Line pairs halve the field's vertical resolution at 1080p / 720p (2-row steps on the stripes, barely visible);
  the still's 1-px cut slivers and white block-bottom lines come out 2 rows tall.
- The whole picture sits 3–4 px right of the source's active window (the generated avid starts 3–4 clocks after the
  source's); immaterial.
- Resolutions: 720p is the same picture; SD shows the centre 4:3 (the outer quarter of each outer face is off-screen),
  and its stripes are coarser per line (one SD line = 2.25 virtual rows, so SD gets no line pairs).
