# Ribboncandy — twisted ribbons over a sample-and-hold mosaic, Heirloom #39

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/ribboncandy.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Ribboncandy Diag`, `out/rev_b/ron/_diag/ribboncandy_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 is untouched. Phase-2 working files: `.heirloom_work/rb_ribboncandy/p2/` (sweep.py, check.sh,
psim.sh, renders).
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230530_22_27_05_Pro.jpg` (1920×1080 capture, 2023-05-30). Replaces v0.1.0 (S-curved two-colour ribbons
over a hashed glyph-block field), which Ron judged too abstract and not like the photo. Presets are hand-written
placeholders.

## Phase 2 (v0.6.0)
Ron left no notes for ribboncandy (free hand, 2026-11-03 batch). The v0.5 knobs were already bold oscillator controls
(ribbon / bend / face frequencies, cell clock, patch, FM depth), so they keep their names, ranges and defaults. What
phase 2 adds is element-level parameters on top of them, plus three switches that were moderate turned into
whole-picture flips:
- **each ribbon gets its own width** (K1);
- **each cell row gets its own cell size** (K5);
- **the thin ribbons weave loose from the thick ones** (K4).

New motion and new per-element spreads are all zero at the default knob positions, so the defaults draw the v0.5
picture.

| Control | Was (v0.5) | Now (v0.6.0) | Full sweep in screen terms |
|---|---|---|---|
| K1 Ribbons | ribbon-oscillator frequency ×0.4 .. ×2.6 (3.7 across = the photo at 50 %) | **same count range, plus away from 50 % each ribbon takes its own width.** The shift is ±D on the thick body end, with the thin ribbon following. D = 5·\|K1 − 50 %\| in phase units of 4096 (0 .. 1280 = ±0.31 period), on an 8-ribbon pattern +1, −½, +¼, −1, +¾, −¼, +½, −¾ counted from the left edge | 50 %: the photo. 25 % / 75 %: fewer / more ribbons whose widths run 0.24 .. 0.55 of the period (photo 0.395), so a 2.3× spread between the fattest and slimmest. 0 % / 100 %: 0.08 .. 0.71: some ribbons shrink to the lead and a sliver while others nearly fill their period, squeezing the cells to slits |
| K2 Bends | slow-wave frequency ×0.3 .. ×3.5 | unchanged | 0.44 .. 5 S-bends per frame |
| K3 Twist | face-oscillator frequency ×0.45 .. ×2.5, cells follow | unchanged | — |
| K4 Speed | bipolar roll of the bends and twists, still in the centre, default 640 | **same roll; past the default speed (\|K4 − 50 %\| > 12.5 %, either direction) the thin ribbons weave across the thick ones.** Their phase is shifted per line by W = depth·sin(tw), at 3.1 cycles per frame. tw rolls against the bends at 160·\|sp\| per field. Depth = 6 phase units per step, capped at 1536 = ±0.375 period from \|sp\| ≥ 384 | 37.5 .. 62.5 % (incl. the default): no weave. 75 %: ±0.19 period (~±100 px at 1080p). ≥ 87.5 % or ≤ 12.5 %: ±0.375 period (~±195 px). The thin magenta / red ribbon swings across its thick ribbon, the cells and into the next ribbon, drawn on top: the candy's second strand |
| K5 Cells | cells per face period 2 .. 8 .. 24, rows scale with them | **same mean, plus away from 50 % each cell row detunes its own cell size.** mq_row = mean ± D5 on the same 8-entry pattern (row index scrambled). D5 = mean·\|K5 − 50 %\|·¾. Rows keep the mean's height | 50 %: the photo's 8 cells per face period on every row. 75 %: rows of 7.5 .. 16.5 cells. 100 %: rows of 6 .. 32 cells (fine noise next to coarse bricks). 25 %: 4 .. 9. 0 %: 1 .. 3.5 (big blocks of mixed widths). A staggered "brick wall" instead of a uniform grid |
| K6 Patch | RGB cable patch, 6 permutations | unchanged | — |
| S7 Faces | Twist (photo) / Solid (every ribbon blue) | **Twist (photo) / Cells.** The faces are keyed by the ground mosaic: each ribbon pixel shows its blue face (and a magenta thin ribbon) where the cell behind it is lit, and its green face (red thin ribbon) where it is black. In Video the camera's S&H mosaic picks the faces; in Osc the photo's cell table does | the ribbons become blue / green windows onto the mosaic. **Renamed value** (Solid → Cells) |
| S8 Ground | Cells / Black | unchanged | with S7 Cells: ribbons on black, mosaic-patterned faces |
| S9 Kinks | On (photo) / Off | **Photo / All.** All repeats the top 256 lines of the kink table every 256 virtual lines. That is ~8 zigzag kinks down the frame instead of 2 at the top and 1 at the bottom. P12 still scales them | the whole frame zigzags (sharp jogs every ~128 lines, hardest on the right). **Renamed values** (On / Off → Photo / All; Kinks Off is gone, P12 0 % shrinks them to the 25 % floor) |
| S10 Flip | Down (photo) / Up (vertical modulators flipped) | **Mirror: Off (photo) / On.** The right half is mirrored into the left (a candy-wrapper symmetry). Both sides show the strong right-hand swing, and the ribbons meet at the centre. A one-line buffer (3 block RAMs) stores the right half's colour words and plays them back reversed on the next line's left half (1-line offset, invisible) | a symmetric picture about the centre line. **Renamed control** (Flip → Mirror; the old Flip is retired) |
| S11 Source | Video / Osc | unchanged | — |
| P12 Wave | FM depth: straight .. photo .. whiplash | unchanged | — |

How it's built:
- No new pipeline stage: latency stays 16 in every mode.
- The per-ribbon width and the weave are two subtractions at pixel stage 1:
  - the thick body end uses the width-shifted phase;
  - the thin windows and the red ramp use the width- and weave-shifted phase (mod 1);
  - the ribbon lead stays on the plain phase;
  - the G ramp stays on the plain phase.
- The ribbon index is the wrap count of the ribbon accumulator.
- The weave is one more sine read and one more serial product in the line engine (~15 clocks).
- The row detune is two states in the line engine.
- The S9 kink address is a mux.
- The S7 key reads the cell ROM / RAM outputs at stage 2.
- The mirror buffer is a single-bank 1024 × 12 RAM with no_rw_check. Reads happen on the left half and writes on the
  right half, and right-half reads go to the never-written address 1023.

Presets (re-aimed): **Photo** (= the defaults), Wrapper (Mirror, P12 70 %), Weave (K4 94 %, Osc), Scatter (K1 88 %,
K5 80 %), Zigzag (Kinks All, P12 78 %), Mosaic Faces (Faces Cells + Ground Black), Straight (P12 0, K1 20 %),
Whiplash (P12 100 %, K4 98 %, Kinks All, Mirror).

Build (router2, 2026-11-03, `.heirloom_work/diag_build.sh ribboncandy`): HD Analog s2 79.09 (s1 missed), SD Analog s1 74.94, HD HDMI s2 77.71 (s1 missed), SD HDMI s1 59.40, HD Dual s4 77.60 (s1-s3 missed), SD Dual s1 66.00 MHz (builder's routed Fmax); 7,395-7,416 / 7,680 LC (96.3-96.6 %, +~590 over v0.5.1's 6,807-6,833), 11 / 32 EBR (+3: the mirror buffer). iceunpack OK x6; standalone yosys: 0 emulate_transparency (no_rw_check took on the new RAM). No pin-skew screen (diag preview); the builder's seeds are kept. Latency 16 (unchanged).

Checks:
- GHDL analyse / elaborate / synth front end (`p2/check.sh`): clean.
- `p2/sweep.py`, the new arithmetic at RTL widths over every knob value: no wraps or overflows. At every K1 the thick body end, the thin windows and the lead never overlap or wrap. The K4 weave depth stays at or below 1536 and is 0 from 37.5 to 62.5 % (default included). The K5 row MQ stays within 16..511, and the row spread is 0 at 50 %.
- Default frame, GHDL 720p60 decimation 1, Osc, K4 512 (still, so the roll phase can't differ): **bit-identical to production v0.5.1** (0 of 921,600 pixels differ; `p2/prod_still.png` vs `p2/diag_still.png`).
- One extreme render (`p2/diag_ext.png`: K1 88 %, K5 80 %, K4 94 %, S7 Cells, S9 All, S10 Mirror, Osc). The left half pairs with the right half of the line above for 100 % of pixels, centred on the active width (after the tester's 4-px capture offset). Rows show distinct cell sizes, the thin ribbons weave across the faces, and the faces follow the mosaic.
- Not verified: Video mode with S7 Cells; the weave / roll over time (motion); SD and interlaced renders; hardware.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_ribboncandy/` (`notes.md` has every number).
- **Composition.** Four repeats across the frame, each [a thick twisted ribbon] + [a thin magenta / red ribbon], over a
  background of 32.5-line rows of sample-and-hold cells, with large black areas.
- **Channel split.** Inside every thick ribbon, G is a linear ramp (rms 0.01–0.03), B is a binary "face" signal, and
  R is 0. The thin ribbon is R on, with B on (magenta) or off (red-orange).
- **Ribbon oscillator.** Per row, the G-ramp zero crossings fall at x = −49 + n·P(y), with P 489..551 px; the start does
  not move. As an hsync-reset oscillator: u = (x + 91)·f_A(y) − 0.080, where
  f_A = 1.926e-3 − 0.1095e-3·cos(2π(y − 931.6)/740.7) + three raised-cosine bumps:
  +0.109e-3 at y 33.7 (half-width 58), +0.203e-3 at 165.8 (72), +0.142e-3 at 1055 (56).
  The fit is rms 2.0 px of P. The bumps are the sharp zigzags of the thin ribbons at the top and bottom of the frame. The
  displacement grows with x, so the right-hand ribbons swing hardest.
- **Ribbon profile (against frac u).**
  - G = 10 + 600·f (0..255).
  - Thick ribbon: green face [−0.06, 0.405); blue face [−0.06, 0.335). The blue face is narrower, and its leading
    0.06 is a bright blue edge (dark for the green face).
  - Thin ribbon: magenta (242,22,248) on [0.452, 0.522) when the face is on; red-orange (250, 26 + 430(f − 0.46), 14) on
    [0.447, 0.598) when it is off.
- **Face oscillator.** A square, duty 0.5, phase (x + 91)·f_F(y) − 0.564 (the same reset instant), where
  f_F = min(3.278e-3, 2.833e-3 − 0.719e-3·cos(2π(y − 220.4)/1390.9)). A per-row fit of period, phase and duty agrees
  with 98–100 % of the ribbon pixels. The clip makes the face boundaries vertical below y ≈ 740; above it they twist
  into "/" diagonals (the ribbon-candy twist).
- **Background.** The cells are 8× the face frequency: the cell edges sit within 1–4 px of the model (38.1 px at the
  bottom), slanting parallelograms where the face frequency changes within a row. Rows are 32.544 lines from y 29.23
  (residual < 1.2 lines). The cell colours follow no XOR-of-counter-bits law (best 63 %): they are S&H content.
  Background palette and shares: black (19,16,7) 47 %, white (235,241,235) 21 %, light grey (198,221,226) 12 %,
  yellow (239,247,3) 6 %, cyan (11,244,242) 4 %, magenta (240,24,248) 4 %, grey (139,154,160) 3 %. Cells in one half
  of every face period (n mod 8 in 1..4) are 42–88 % black: these are the big black areas.
- **Whole-frame palette (k-means 14).** Black 27 %, white 11 %, blue (7,27,248) 7 %, red-orange (243,52,13) 6.5 %,
  magenta (234,28,235) 6.5 %, sky (7,188,245) 6 %, mid blue (1,108,251) 6 %, dark green 6 %, green (2,205,6) 6 %,
  mid green 6 %, yellow 4 %.
- **Edges.** Horizontal 7–12 px (10–90 %), vertical 1 line.
- **Fit.** Float proto against the photo: 4×4 region means max 7.1, mean 3.2 (of 255); luma local contrast 1.01 of
  the photo; colour-class shares within 1.7 %.

## What it does
1. **Ribbon oscillator.** A 28-bit phase accumulator per pixel. Per line:
   incA = F0 − A·cos(θA) + B·bump(y), and the start phase XR·incA − 0.080.
2. **Face oscillator.** incF = min(FM, CF − AF·cos(θF)), start XR·incF − 0.564. Face = phase fraction < 0.5.
3. **Cell counter.** M × the face phase (M = 8 in the photo), n = cell 0..63; row = a 32.5-line row counter.
4. **Colour.** The thin ribbon over the thick ribbon over the ground.
   - Thick: (2 or 4, G ramp, 252 or 10) by face, with face-dependent ends.
   - Thin: magenta, or red with its own G ramp.
   - Ground: Osc = the cell table; Video = the camera cells; S8 = black.
5. **Output.** RGB cable patch → 10-bit YUV (u = Cr, v = Cb), luma ×1.07 so the photo's white lands at 940 →
   [1 2 2 2 2 2 2 2 1]/16 on Y, U and V → luma 13/16 (white 777) → blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Wave** | The ribbons' waviness: the FM depth of both oscillators. 0 % = near-straight ribbons, upright faces and an upright cell grid (floors keep a gentle sway, so Bends, Speed, Kinks and Flip stay live); 50 % = the photo; 100 % = deep sway, sharp kinks and dense twists. Glided. |
| K1 Ribbons | The ribbon count: ribbon-oscillator frequency ×0.4..1..2.6 (3.7 ribbons across = the photo). Glided. |
| K2 Patch | The whole palette: RGB cable patch RGB (photo) / RBG / GRB / GBR / BRG / BGR (moved from K6 in v0.6.1). A labelled default (index < 6) is read as its index. The S-bends (was K2 Bends) are fixed at the photo's 1.46 per frame. |
| K3 Twist | The blue/green twist: face-oscillator frequency ×0.45..1..2.5; the background cells follow it. Glided. |
| K4 Speed | Bipolar, still in the centre ±8. The bends roll down the frame (default 0.63 lines/field) and the twists roll with them. |
| K5 Cells | The background blocks: 2..8..24 cells per face period (8 = the photo, 38 px × 32.5 lines); rows scale with them. Glided. |
| K6 Camera | Camera strength (v0.6.1), 0..100 %, default 50 %. Video: a gain on the camera ahead of the cell thresholds: 0 = no camera (black cells, the picture a black input gives; with S7 Cells every face turns green), 25 % = ×2/3, 50 % = the shipped look, 100 % = ×1.6. Osc: the photo's cells drop out to black below 50 % (k/2 of every 256, per-cell hash), and above it up to half of the black cells light in the photo's colours; S7 Cells keys the faces off the same cells. Glided. |
| S7 Faces | Twist (photo) / Solid: every ribbon on its blue face, magenta thin ribbons. |
| S8 Ground | Cells (photo) / Black: the ribbons on black. |
| S9 Kinks | On (photo) / Off: the zigzags at the top and bottom. |
| S10 Flip | Down (photo) / Up: the vertical modulators flipped top to bottom (the kinks move to the bottom, the twist pattern inverts). |
| S11 Source | Video (default) / Osc. |

Defaults: K1, K3, K5, K6 and P12 at 512, K4 640, K2 and all switches at label 0 (S11 Video).

## Video
- **Video:** the cells are a sample-and-hold mosaic of the camera on the same FM'd cell grid. Each cell is sampled at
  its first pixel on the first line of its row band and held for the band (64-entry cell RAM), then thresholded per RGB
  channel into the photo's colour family: black, blue, green, cyan, red, magenta, yellow, white, plus light grey for
  dimmer whites. The threshold is 395/876 in the bright half of each face period and 653/876 in the dark half, so the
  photo's black half-periods survive and only bright camera colours show through them. The ribbons are unchanged, so
  the composition still reads (selfie test: the red / green / blue bears sit in the cells between the ribbons).
- **Osc:** the photo's own cells. A 64 × 64 cell table holds the 861 cells read off the photo; the rest are drawn from
  the photo's per-half / per-zone / checker-parity statistics. K3, K5 and P12 move the cell grid in both modes.

## Engine
- **Shell:** heirloom v0.2.0 — timing-ID virtual coordinates (x spans 1920 virtual px on every mode, y = 1080
  virtual lines), a vblank zone (no avid for 1024 clocks), source-width measurement, and damier's generated avid.
- **Field sequencer** (one serial shift-add multiplier, ~33 states):
  - knob curves (square-law halves) and the P12 depths (with floors);
  - F0, A, B, CF, AF, FM in Q28 cycles per real px (the virtual-px step folded in);
  - the row increment and the vertical-phase increments;
  - the rolls, and the start phases (S10 Flip).
- **Glides:** a 3-clock pipeline (gap → step and ±4 deadband → apply) on K1, K2, K3, K5 and P12, snapping on the
  first field.
- **Per-line engine** (kicked one clock after the source avid rise, computes the next line in ~110 clocks):
  - two cosine reads (1024 × 12 sine ROM) and the bump (512 × 8 ROM, virtual line >> 2);
  - incA, incF (clipped) and the start phases;
  - the cell increment and start; the row number and its first-line flag.
  The results are committed at the next avid rise.
- **Pixel path** (16 stages, one latency in every mode):
  - three 28-bit accumulators;
  - 12-bit ribbon-phase thresholds; the G ramp (×75 >> 9); the red ramp (×27 >> 8);
  - cell ROM (4096 × 3) and cell RAM;
  - colour mux, patch, RGB → YUV (Q6 shift-adds);
  - [1 2 2 2 2 2 2 2 1]/16 on Y/U/V as two 8-sums; luma 13/16; gate on the filter-centre pixel's avid.
- **Tables:** generated by `ribboncandy.py` into `ribboncandy_pkg.vhd` (C_RSIN, C_RBMP, C_RCEL).
- **Verification** (GHDL 720p60 decimation 1 against the RTL-exact `imodel.py`):
  - Osc defaults: 93 % of pixels within 24 (sum of |dY|+|dU|+|dV|) after the tester's 4 px / 1 line capture offset;
    per-row 0.85–0.99. The integer model agrees with the float proto on 98.1 % of pixel classes.
  - Motion (K4 max): the frame matches the model's third roll step (93.7 % of pixels) against 80 % for the static model.
  - Video (selfie): verified visually.
- **Fixes on the way:**
  - |cos| was taken with a narrowing `resize(-x, 11)`, which folded values ≥ 1024 (rows where cos < −0.5 were wrong);
    the same trap folded |Speed| = 512 to 0. Both are slices now.
  - The P12 glide in one clock was the critical path (74.37 MHz); it is now pipelined and the engine kick registered.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The Y box resyncs too (to 8 x 64). Post-route Fmax: HD Analog s1 77.86, SD Analog s1 74.53, HD HDMI s2 75.32, SD HDMI s1 80.01, HD Dual s2 82.82, SD Dual s1 72.45 MHz; 6807-6833 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`./build_heirloom.sh ribboncandy`).
- **Build seeds:** HD Analog = seed 2 (seed 1 missed); HD HDMI = seed 3 (seed 1 missed, seed 2 stalled the router);
  HD Dual = seed 3 (seeds 1–2 stalled); SD Analog, SD HDMI, SD Dual = seed 1.
- **Fmax (last post-route line):** HD Analog 83.56, HD HDMI 78.94, HD Dual 78.93, SD Analog 77.80, SD HDMI 64.89,
  SD Dual 67.74 MHz.
- 6793–6812 / 7680 LC (88.7 %), 8/32 EBR. All bitstreams pass iceunpack.
  `out/rev_b/ron/heirloom/ribboncandy.vmprog` (447,294 bytes).
- router2 stalls on some HD seeds of this netlist. If a later version regresses, try
  `NEXTPNR_ROUTER="--router router1"` before restructuring.

## Deviations from the photo
- **Background cells.** The visible cells are the photo's own; cells hidden behind ribbons in the photo are drawn to
  its statistics, so they differ cell for cell when motion or the controls reveal them.
- **Brightness.** Drawn whites peak at 777 after the HD HDMI bar fix's 13/16 luma, so the picture reads darker than the
  photo (RTL 4×4 region means max 20 against the proto's 7; luma contrast 0.86). A 1.07 luma pre-gain keeps the light
  grey distinct from white.
- **Edges.** The [1 2 2 2 2 2 2 2 1]/16 conditioning gives ~7 px edges against the photo's 7–12 px.
- **Kinks.** The top and bottom zigzags are modelled as fixed vsync ringing: they stay at the frame edges while the
  bends roll.
- **Not modelled:** the photo's faint ghosting and noise, and the occasional red tint at a green-ribbon notch.
- **SD 4:3:** the same picture (four ribbon groups, same rows), compressed horizontally by 3/4, so the cells come out
  narrower than tall. Interlaced modes draw the same picture in both fields.
- **Video** can't be "the photo's processing on the camera" exactly, because the photo's cells are frozen content: the
  camera enters as the cells' S&H content through per-channel thresholds.

## Working files
`.heirloom_work/rb_ribboncandy/`:
- `notes.md`: look, measurements, proto log, controls, model, RTL, verification, builds.
- Measurement: `ref.jpg`, `classify.py`, `gramp.py`, `phasefit.py`, `unwrap.py` (phase-space view), `facefit2.py`,
  `fitall.py` → `model.json`, `celltab3.py` + `filltab.py` → `celltab_full.npy`, `rowfit.py`, `ribprof.py`, `bgpal.py`.
- Proto: `proto2.py` / `proto_final.png` / `cmp_proto.png`; controls `ctl.py`, `proto_ctl.py`, `sheet.py` / `sheet.png`.
- `imodel.py` (RTL-exact integer model), `tables.py` and `gen_hook.py` (write `ribboncandy.py`), `rtlcmp.py`.
- GHDL renders: `rtl_osc_final.png`, `rtl_vid_final.png`, `rtl_mot.png`; `cmp_final.png` (photo / proto / RTL Osc / RTL Video).
- `check_local.sh` (GHDL check including the package), `old_v010_*` (the replaced v0.1.0).

## Open questions for Ron
1. **K1 Ribbons.** The per-ribbon widths ride on the count knob (marmalade-style: away from 50 % each ribbon takes its
   own share). Would you rather have the widths on their own control, so the photo's 3.7 ribbons can be scattered
   without changing the count? It could replace S8 Black, or go on K2 with Bends kept at the photo's.
2. **K4 weave.** The thin ribbons only start to weave past the default speed (either direction), so the default stays
   the photo. Should the weave get its own control, or start right from the still centre?
3. **S9 / S10.**
   - S9: Kinks Off is gone (All replaced it). Keep All, or bring Off back?
   - S10 Mirror mirrors the right half (strong swings) into the left. Alternatives: the calm left half into the
     right, or a top / bottom mirror.
4. **S7 Faces.** Solid (every ribbon blue) became Cells (faces keyed by the mosaic). Is Solid missed?

## Hardware watch list
- **Renamed:**
  - S7 Faces: value Solid → **Cells**;
  - S9 Kinks: On / Off → **Photo / All**;
  - S10 **Flip → Mirror** (Off / On).
  K1 / K4 / K5 keep their names; their new behaviour only shows away from the defaults.
- **Chip nearly full:** 96.6 % LC. A second round needs cuts first. The cheapest candidates:
  - the K5 row detune (~60 LC);
  - moving the chroma-filter delay lines into a block-RAM ring (PHASE2 step 5, ~−80 LC).
- **Thin router2 closure:**
  - HD seeds 2 / 2 / 4; HD Dual missed seeds 1-3 before closing at 77.60.
  - If a config misbehaves (HD HDMI lines), re-seed it, or try router1.
- **Mirror:**
  - The left half shows the previous line's right half (a 1-line offset). The first line of a frame shows the last
    line of the previous field on the left.
  - Watch for a seam at the centre, where the output filter mixes the two halves.
- **S7 Cells:** the keyed face is decided straight from the cell ROM / RAM outputs (timing closed). Flag any speckle on
  the face edges in Cells mode.
- **K1 widths** are counted from the left edge, so the 8-ribbon pattern repeats past 8 ribbons (K1 high).

## Next step
- On approval: `./heirloom_diag.sh merge ribboncandy`.
- Then repack the previewed diag bitstreams instead of re-routing (router2 is nondeterministic; HD seeds 2 / 2 / 4
  here):
  1. Copy `build/programs/ron/ribboncandy_diag/rev_b/bitstreams/*.bin` into a scratch dir.
  2. Add the production TOML's `program_config.bin` to it.
  3. Run `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/heirloom/ribboncandy/ribboncandy.toml <dir>
     out/rev_b/ron/heirloom/ribboncandy.vmprog`.
  4. Check the result: the same size as the preview, about 140 bytes differ, and the id has no `_diag`.
- Then update SUMMARY / README / PHASE2, and commit only ribboncandy's files and rows
  (`.heirloom_work/p2_batch/stage_rows.py ribboncandy`).

## K6 camera pass (2026-11-15)
Ron: K6 is the camera strength in every program, default 50 %. The same behaviour as production v0.5.2 in a leaner
build (the diag was at 96.6 % LC; the first K6 build overflowed at 7730 LC). **v0.6.1.**
- **K6 Camera** (0..100 %, default 512), glided in the glide slot the fixed Bends no longer needs.
  - **Video:** the cell thresholds × 1/gain from a 16-step per-field table (k / 64): below 50 %
    s = 1 + 4·((512 − k)/512)³, above s = 1 − 0.375·(k − 512)/512. 0 % = black cells (a black input's picture; with
    S7 Cells every face is then green / red), 25 % = gain 0.75, 50 % = the shipped look (395 / 653 exactly),
    100 % = gain 1.49. One comparator set instead of two: its threshold is picked per pixel by the cell half the
    stage-2 sample-and-hold write sees one clock later (the beam's cell bits), the same decision as before, bit-exact
    at the default.
  - **Osc:** cells drop out by an 8 × 8 ordered-dither (Bayer) rank of (cell, row): k/8 of the 64 stay below 50 %,
    and above it up to half of the photo's black cells light (colour from the rank); formed at stage 3 so S7 Cells
    keys the faces and the S10 mirror stores the same cells.
- **K6's old function (Patch) moved to K2; K2 Bends** (still the weakest knob in the diag: K1 and K5 gained element
  parameters, Bends did not) **is fixed at 512** = the photo's 1.46 S-bends per frame.
- Standalone yosys against the v0.6.0 diag: +87 LUT, −15 carry, +38 FF.
- **Build (v0.6.1, 2026-11-15):** router1, all six configs seed 1. HD Analog 77.45, HD HDMI 80.87, HD Dual 76.70 MHz
  (worst HD); SD Analog 82.89, SD HDMI 76.62, SD Dual 82.98 MHz; 7433–7466 / 7680 LC (97.2 %), 11/32 EBR. Preview
  `out/rev_b/ron/_diag/ribboncandy_diag_v0.6.1.vmprog`. NOT flashed.
