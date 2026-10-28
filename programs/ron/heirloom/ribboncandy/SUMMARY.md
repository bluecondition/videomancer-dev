# Ribboncandy — twisted ribbons over a sample-and-hold mosaic, Heirloom #39

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230530_22_27_05_Pro.jpg` (1920×1080 capture, 2023-05-30). Replaces v0.1.0 (S-curved two-colour ribbons
over a hashed glyph-block field), which Ron judged too abstract and not like the photo. Presets are hand-written
placeholders.

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
| K2 Bends | The ribbons' S-bends: the slow wave's frequency ×0.3..1..3.5 (1.46 bends per frame = the photo). Glided. |
| K3 Twist | The blue/green twist: face-oscillator frequency ×0.45..1..2.5; the background cells follow it. Glided. |
| K4 Speed | Bipolar, still in the centre ±8. The bends roll down the frame (default 0.63 lines/field) and the twists roll with them. |
| K5 Cells | The background blocks: 2..8..24 cells per face period (8 = the photo, 38 px × 32.5 lines); rows scale with them. Glided. |
| K6 Patch | The whole palette: RGB cable patch RGB (photo) / RBG / GRB / GBR / BRG / BGR. A labelled default (index < 6) is read as its index. |
| S7 Faces | Twist (photo) / Solid: every ribbon on its blue face, magenta thin ribbons. |
| S8 Ground | Cells (photo) / Black: the ribbons on black. |
| S9 Kinks | On (photo) / Off: the zigzags at the top and bottom. |
| S10 Flip | Down (photo) / Up: the vertical modulators flipped top to bottom (the kinks move to the bottom, the twist pattern inverts). |
| S11 Source | Video (default) / Osc. |

Defaults: K1–K3, K5 and P12 at 512, K4 640, K6 and all switches at label 0 (S11 Video).

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
