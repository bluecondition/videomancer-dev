# Crossbeam — a glowing cross beam keyed over torn colour bars

**v0.6.0 (2026-11-01, phase 2): built, NOT flashed** (see [Phase 2](#phase-2-v060)). v0.5.0 (2026-10-27) rebuilt it from the photo.
Photo: `photos/WIN_20230527_01_43_26_Pro.jpg`. The v0.1 version, "soft cyan X over hashed Mondrian blocks", was judged too abstract.
The picture is three layers:
- a grid of colour bars;
- a mirrored contour field (red/black zebra stripes and green→red ramps) seen through the bars' "transparent" cells;
- a glowing cyan cross beam, luma-keyed over both.
Brighter bars push into the beam (the bullet-shaped side wedges, the yellow sliver in the core) and a dark keyer outline runs along every cut.
Presets are hand-written placeholders.

## Phase 2 (v0.6.0)
**Ron's brief (2026-10-31, on v0.5.0, sight unseen, free hand):** "it needs more". Change K2 / K6 from global grid scales; make
the beam more dynamic and adjustable; warp the table lookup and let the beam push the contours; do something more interesting
with S8.

**What changed:**
- **K2 Rows and K6 Columns** scale the bar rows one by one instead of scaling the grid. Both are bipolar with the photo at the
  centre. Left is **Scatter**: each row gets its own height (K2) or its own block size (K6) from a per-row weight table. Right is
  **Lens**: the factor follows the line's distance from the beam's crossing line. Rows swell to ×3.8 tall at the beam and thin to
  ×0.3 away from it; blocks swell around the beam line and shrink away, so the grid bulges into a barrel (K6 changes per line,
  which bows the column edges). The factor is 64 · 2^(k·g / 256), ×0.26 to ×3.5. Columns scale about the screen centre.
- **K3 Warp** (new) ripples the zebra / ramp table lookup with one wave in quadrature: up to 255 px across (added to the folded
  coordinate) and 8 table rows (272 lines) down, 1.7 cycles per frame height. The wave travels with K4. Zebra density (the old K3)
  is now fixed at the photo's, since it was a global scale of the field.
- **K4 Flow** now also rolls the bars (a quarter as fast, as at the v0.5 defaults) and moves the warp wave.
- **K5 Swing** (new) moves the beam. It is bipolar and still at the centre (the photo). Right: **scissor**. The arm slope swings
  ±m5 knob units about K1 on a triangle LFO. The rate (1.5 m5 per field) and the depth both grow with the turn. Left: **travel**.
  The cross climbs through its 731-line period, so the next cross rises in from below, at up to ~0.6 periods/s (square law). The
  K2 / K6 lenses ride with it. Back at the centre the cross snaps home.
- **P12 Beam** also sets the **push**. The beam's own log level, from three octaves below visibility and independent of P12's
  gain, offsets the stripe phase by up to ±8 stripes, signed about the middle. At 0 % the glow is gone and the cross is drawn in
  the contours. At 50 % (the photo) there is no push. At 100 % the beam swallows the bars and shoves the contours.
- **S8 Tear: Still / Slip.** Still is the photo's tear, now always on. **Slip** slides each bar row sideways at its own speed:
  −3..+3 × 1.5 px per field, wrapping for free at the 3-period superperiod. The zebra row tears swing 0..127 px.
- **K1 Angle** widened to 0.02–0.77 (the photo's 0.232 is at 289).

**Engine changes:**
- The colour-bar block coordinate is now 2^17 units per superperiod (3 × 612 px), so all of its wraps are free.
- The new **line engine** has its own serial multiplier (signed 18 × unsigned 17, 17 steps) in nine 32-clock windows after each
  source avid rise, plus a 512 × 8 line ROM (scale factors, per-row weights and slip speeds, sine). Per line it computes the
  warped table row (first, so the table engine starts by clock 53), the warp across, the K2 row step, the K6 block step and the
  centre-anchored block start, slip, and dy². It finishes by clock 311 (SD lines are 858).
- The vblank sequencer runs 8 products (scissor swing, kS, SDXm, m4², m5², SDXm², kS², SDXm·kS). The K6 ROM, density mantissa,
  row-count and block-scale slots are gone.
- Push in the pixel path: B clamp at stage 8, B × kp (8 × 7) at 9, add at 10. The zebra fold, amplitude and red move one stage
  later into the slack that p13_zr had. Latency stays 24.
- **Fit.** The first phase-2 RTL packed 6307 LC (program only, about 7740 placed, over capacity). Ablation put the cost at warp 210
  LUT, swing 218, push 134, slip 119, K6 31 and K2 5. The economies changed no feature:
  - one quadrature warp phase;
  - travel accumulated straight into line 0's dy;
  - ROM reads timed into each window, which drops six capture registers;
  - window/phase decode;
  - block DDA at Q.4;
  - scissor applied in knob space;
  - shift-approximated flow and travel rates;
  - SDXm·kS folded into F0;
  - no 2kS² register;
  - green ramp part as a 32-entry lookup, with its 64 / 512 offsets folded in;
  - camera ×75/256 as shifts;
  - a 20-bit vblank multiplier;
  - one's-complement magnitudes;
  - the ridge distance on top bits;
  - the outline halfway to neutral (was 7/16).

  It packs 5542 LC (program only).

## Reading of the photo
Measured in `.heirloom_work/rb_crossbeam/` (`m1`–`m14.py`, `st.py`, `integ2.py`, `cmaps.py`, `fitx5.py`, `cellkeys.py`; `notes.md`).
- **Palette** (k-means, 14 colours): black 17 %, maroon 11 %, olive 10 %, cyan core (0,219,222) 9 %, dark teal 9 %, teal 8 %, red (196,20,12) 7 %, magenta (249,8,207) 7 %, yellow 5 %, white 4 %, green 2 %, blue 2 %.
- **Colour bars.** The bars are translational. The mirror-symmetric parts (beam and zebra) sit on top of them.
  - Rows: boundaries at y = 2.9 + 39.04 k (residual < 1 line), 28 rows per frame.
  - Columns: the translation correlation peaks at 612 px. Each period is three columns A / B / C = 305 / 230 / 77 px, with A starting at x = 117 + 612 k.
  - Torn rows: rows 6–9 and 22–25 are shifted +376 px (comb-phase fit 493–504 against 117).
  - Colours: the cell colours (magenta, yellow, white, blue, navy, olive, maroon, black, cyan; photo medians) form a near-16-row cycle with exceptions. They are stored as a measured 28 × 9 table (3 periods). The table also marks ramp cells, zebra cells and one "ramp over white" cell type: the white strip in row 11.
- **Beam.** The beam is cyan (G ≈ B, R ≈ 0), peaking at (0,240,247) and fading to dark teal.
  - Profile: fitted as G = A(|dx|) · (1 − |t|)^p with t = (dy² − s² dx²) / c². That is a hyperbolic X with A0 250.8, a 882.5, falloff exponent 2.17, centre (984.7, 466.8), s 0.232, c 191.4, p 0.62 between the arms and 0.68 outside.
  - Repeat: the beam repeats vertically every 731 lines, which is why the next cross's arms show at the bottom edge.
  - Keying: the beam wins where G exceeds a key level for the colour underneath. YEL 221 > MAG 158 > OLV 132 > NAV 102 > BLU 80 > MAR 72 > BLK 38; white never loses. Where a cut crosses a cell, the key is refined per cell. The fit classifies 96.3 % of pixels correctly.
- **Zebra.** Red stripes (R up to 245) on black, pitch 16–45 px (median 25).
  - They are a wavefolder on a smooth field, mirror-symmetric about x = 984.5 (the left half overlays the flipped right half).
  - Near the beam the contours run parallel to the arm edges. In the corners they wrap round blob-like plateaus with rounded right-angle bends. They are torn row by row.
  - The field was reconstructed from the stripes' local orientation and spacing: structure tensor, sign flood, zero gradient on the plateaus, weighted least-squares integration.
- **Ramp cells.** Smooth green → ochre → red with R and G varying with position and mirror-consistent: x = 700 and x = 1270 at y 120–160 are both (148,103,0).
- **Mixing.**
  - Zebra and beam meet in a per-channel max: salmon lines on teal (the photo's "pink" cluster (153,88,95)).
  - Ramps and bars are keyed with a sharp cut and a dark outline.
  - Horizontal edges are soft (~4 px); row edges are sharp.

## What it does
1. **Bars.**
   - A row accumulator (K4 roll) gives row r and the half-row flag. Its per-line step is the photo's 27.7 rows per frame times the
     K2 factor (Scatter: the row's weight; Lens: the line's distance from the beam line).
   - A block-x DDA gives column j. It runs at Q.4 of a 2^17-unit superperiod (3 × 612 px), with a per-line step (the K6 factor) and
     a start that keeps the screen centre anchored, so it wraps for free.
   - The cell ROM at r·16 + j holds the colour code and key code.
   - Rows 6–9 and 22–25 start 376 block px later (the photo's tear). Under Slip every row also slides by its speed × the field
     phase.
2. **Contour field.** Its coordinate is the folded, torn u = |x − 984.5| + tear(r) + jitter + warp-across (+ the Slip swing). The
   table row is warped down by the same wave in quadrature.
   - Zebra phase: a 64 × 32 table (16 px × 34 lines, 16 levels per stripe). The camera adds up to 1 stripe, then K4 flow and the
     P12 push (beam log level × gain). Then the stripe profile clip(5 |f − 128| − 192), × the amplitude^0.7.
   - Amplitude and ramp red/green: a 32 × 32 table (32 px × 34 lines).
3. **Beam.** In the log domain: log A(|dx|) + log F(t) + log P12 + ridge + camera bloom, then an exp ROM. K5 scissors the slope
   (per field) or travels line 0's dy (per field).
   - t comes from exact squares: a 2nd-order DDA of (s·dx)² per pixel against a serial square of dy per line.
   - A ridge boost along the arms (+6 % within 45 lines) makes the yellow sliver taper.
4. **Composite.**
   - Zebra cells: the beam colour plus the zebra red, a per-channel max.
   - Ramp cells and bars: the beam where G3 > key, otherwise the ramp or bar colour.
   - A dark outline (halfway to black / neutral) wherever the key flag changes within ±2 px or from the line above.
5. **Colour.** In YUV per source: a beam colour ROM, a shift-add red part, a 32-entry green-part lookup and a YUV palette. Luma is
   pre-boosted 16/13 and kneed above 724.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Angle | the beam's arms | arm slope s 0.02–0.77; 0.232 = the photo (default 289) |
| K2 Rows | the bar rows, one by one | bipolar, centre = the photo (512). Left **Scatter**: each row its own height (×0.26..×3.5 from a per-row table). Right **Lens**: rows swell at the beam's crossing line and thin away from it |
| K3 Warp | the zebra / ramp lookup | 0 = the photo (default). One wave in quadrature ripples the lookup up to 255 px across and 8 table rows down; it travels with K4 |
| K4 Flow | the zebra contours, the bar rows, the warp | bipolar, still in the centre ±8, square law. The contours stream up to ±4 stripes/s, the bars roll a quarter as fast, the warp wave travels. Default 680 = the photo's 0.4 stripes/s and 0.1 rows/s |
| K5 Swing | the beam | bipolar, still in the centre ±8 (the photo, default 512). Right: the arms scissor (slope swings about K1, depth and rate up to ±503 knob units at 0.7 Hz). Left: the cross travels up through its period (to ~0.6 periods/s, square law) |
| K6 Columns | the bar blocks, row by row | bipolar, centre = the photo (512). Left **Scatter**: each row its own block size. Right **Lens**: blocks swell around the beam line, the grid bulges (the scale changes per line, about the screen centre) |
| S7 Bars | the colour bars | On (default) / Off: the beam over the zebra field alone |
| S8 Tear | the torn rows | Still (default, the photo's tear: the 376 px row shifts, the per-row zebra tears, ±1–2 px jitter) / Slip: each row slides at its own speed (−3..+3 × 1.5 px per field) and the zebra row tears swing 0..127 px |
| S9 Stack | the second cross | Off (the photo: period 731 lines) / On: half the period, three crosses stacked |
| S10 Palette | the colours | Photo / Swap (Cr and Cb exchanged: amber beam, blue zebra, cyan bars) |
| S11 Source | the zebra field and the beam | Video (default) / Osc |
| P12 Beam | the cross beam and its push | log fade: 0 % = no glow, 50 % = the photo, 100 % = 2× (the beam swallows everything but white, yellow and the zebra peaks). The beam's level also pushes the stripe phase, signed about 50 %: ±8 stripes over the beam at the ends, so at 0 % the X is drawn in the contours alone |

K1, K2, K3, K6 and P12 glide (1/8 of the gap per field, ±4 deadband).

## Video
The photo is synthetic, so the camera is woven into the patch rather than replacing a layer:
- **Contours.** The camera's luma, slewed horizontally like an analog low-pass (s += y − s/8), is added to the zebra phase, up to one stripe. The zebra contours bend around the picture.
- **Bloom.** The same luma blooms the beam (log + up to ×1.37). The cross spreads into the bars where the picture is bright.
- **Black camera.** Video mode with a black camera is exactly the photo.
- **Osc mode** uses the stored contour field (the photo's own) with no camera term. Every control stays live in both modes.

## Engine
- **Per-field sequencer.** One serial shift-add multiplier (20 × 20), slots of 32 clocks in the vblank zone. It computes:
  - the K5 scissor depth (m5 × |triangle|);
  - the beam's per-pixel s·dx step and start;
  - the flow speed (m4², shifted) and the travel speed (m5², shifted; line 0's dy climbs, two clocks for the wrap);
  - SDX0², kS² and F0 = kS² − 2 SDX0·kS for the square DDA.
  - The P12 log gain and push gain, the knob curves and the K2 / K6 / K3 decodes are shift-adds.
- **Per-line engine.** Started after each source avid rise (line 0 in the vblank zone), it prepares the next line. Window 0 holds
  the row accumulator (mod 28), beam dy (wrapped at the period), table row, warp phase, |dy|, lens weight and tear + jitter. Then
  nine 32-clock windows on its own serial multiplier (signed 18 × unsigned 17, 17 steps) compute:
  - the warped table row, after which the table engine starts at clock 53;
  - the warp across;
  - the K2 exponent, then the row step;
  - the K6 exponent, then the block step and the centre offset;
  - slip;
  - dy² (left in the multiplier until the commit).
  - Operands come from a 512 × 8 line ROM read just before each window: scale factors 64 · 2^(e/32), per-row weights, slip
    speeds, sine. It finishes by clock 311.
- **Table engine.** It lerps the next line between two table rows into ping-pong line RAMs: zebra phase as 64 {value, slope} words; amplitude and ramp as 32 entries in even/odd banks. It takes 10 clocks per entry with a serial radix-4 product, 640 clocks per line, which fits SD's 858.
- **Pixel path, 17 stages, then output conditioning.**
  - **Beam:** beam DDAs (virtual x, s·dx, block x, the (s·dx)² DDA); |dy² − (s·dx)²| → t; log-profile and log-falloff ROMs; the log-sum; the exp ROM → G3.
  - **Cells and zebra:** cell ROM; zebra lerp; push (B clamp, B × kp, add); fold; amplitude and ramp lerps (3-bit fraction).
  - **Keying and colour:** key compare; beam colour ROM; YUV composite; outline with a key-flag line buffer; Cr/Cb swap.
  - **Output stage:** the luma knee is spread over two of the output stage's luma delay registers. Then chroma [1 2 2 2 2 2 2 2 1]/16 from an EBR ring (re-neutralised in vblank), luma [1 2 1], luma 13/16, and the blanking gate on the generated avid.
  - Latency is 24 clocks in every mode. Syncs are delayed in an EBR ring.
- **ROMs** (generated by `crossbeam.py` into `crossbeam_pkg.vhd`):
  - phase 2048 × 8;
  - amplitude/ramp 1024 × 16;
  - cells 512 × 8;
  - log profile 512 × 11;
  - log falloff 256 × 11;
  - exp 1024 × 8;
  - beam colour 256 × 30;
  - line ROM 512 × 8 (scale factors, per-row Scatter weights and Slip speeds, sine);
  - small LUTs: amplitude gamma, bloom log, tears, row shifts, YUV palette, ramp green part.
- **Contracts:**
  - generated avid (parity-locked start);
  - virtual 1080-line geometry from the timing ID;
  - engines started per field and per line;
  - every RAM collision-free with `no_rw_check` (0 emulated transparency);
  - no self-add carries (checked with dupin).

## Status
- **v0.6.0 built 2026-11-01, not flashed. ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh crossbeam`); every
  config is seed 1.
  - Fmax (post-route): HD Analog 85.44, HD HDMI 96.20, HD Dual 90.01, SD Analog 92.82, SD HDMI 90.70, SD Dual 96.13 MHz.
  - 6958–6973 / 7680 LC (91 %), 23 / 32 EBR. All bitstreams pass iceunpack, and the new labels are in program_config.bin.
    `out/rev_b/ron/heirloom/crossbeam.vmprog` is 490,288 bytes. dupin: 0 self-add carries; 0 emulated transparency.
  - Verification: GHDL 720p60 (decimation 1, Osc, K4 still, sim-only copies with the glides pre-set) against `p2/imodel8.py`:
    - defaults: mean |diff| Y 1.75 / U 2.74 / V 1.94, 3.1 % of pixels > 16;
    - K2 Scatter + K6 Lens + K3 800 + P12 900: 1.42 / 2.29 / 1.91, 2.2 % of pixels > 16.

    Both after the 4 px generated-avid offset. The structure is identical (`p2/cmp_def.png`, `p2/cmp_feat.png`). The residue is
    zebra stripe-edge speckle, from the per-row jitter that the model leaves out, and one-pixel shifts along the beam cuts, from
    the one's-complement magnitudes. Swing, Slip and flow were not simulated (field accumulators feeding verified paths).
- **v0.5.0 built 2026-10-27, not flashed. ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh crossbeam`); every config is seed 1.
  - Fmax (post-route): HD Analog 82.05, HD HDMI 89.79, HD Dual 85.37, SD Analog 76.24, SD HDMI 91.40, SD Dual 85.70 MHz.
  - 6999–7024 / 7680 LC (91 %), 23 / 32 EBR. All bitstreams pass iceunpack. `out/rev_b/ron/heirloom/crossbeam.vmprog` is 488,336 bytes.
- **Fit history.**
  - The first RTL (RGB colour path, per-pixel multipliers) came to 8299 LC.
  - Ablations found the beam multipliers at 840 LUT, RGB → YUV at 580, the zebra multipliers at 540, the d1·d2 product at 400, the lerps at 390 and the engine multipliers at 390.
  - Fixes: a log-domain beam, YUV per source, an exact square DDA, a serial engine, narrower lerps, a sync EBR ring.
  - The luma knee was the critical path (45 → 67 MHz) until it was spread over the luma delay stages.
  - router1 stalled on four self-add carries (3d = d + 2d in the engine), rewritten as 4d − d.
- **Verification.**
  - GHDL 720p60 (decimation 1), Osc, defaults, matches the integer model `imodel7.py` to a mean |diff| of 0.80 (Y) / 0.17 / 0.14, with 0.00 % of pixels > 16 after the 4 px generated-avid offset.
  - Motion test (K4 = K5 = 1023): the frame matches the moving model (flow 0.2 stripe, roll 0.2 rows at field 4) to 0.00 % of pixels > 16, against 28 % for the still model.
  - The Video sim (selfie) shows the beam blooming toward the face and the zebra bent by the picture, with the layout intact.
  - Region-mean error against the photo (4 × 4 grid): proto 6.4, RTL 6.7.
  - SD was not simulated.

## Deviations from the photo
- **Zebra contours.** They are a reconstruction: the stripe orientation, spacing, plateaus and bundle placement match, but individual contours differ.
  - The field is stored at 16 px × 34 lines, so detail finer than about 16 px is smoothed.
  - The per-row zebra tears are a designed table of similar size (±10–40 px) plus jitter, not the photo's exact offsets.
- **White level.** Whites and yellows are compressed to the HD HDMI bar-fix level: output luma 719 / 689 against the photo's ~832 / ~773. Greys, cyan and the mid colours match.
- **Yellow sliver.** It tapers correctly but is thinner: about 18 lines against 33 at its thick end.
- **Wedge outlines.** The cuts are stepped per row as in the photo, but the noses sit within about ±10–20 px of the photo's. The beam is an analytic hyperbolic X with per-cell keys, at 96 % fit accuracy.
- **Outline.** It is one-sided vertically (the line above only). v0.6 darkens it halfway to black / neutral (v0.5: 7/16).
- **Amplitude and ramp resolution.** The zebra amplitude envelope and the ramp colours are tabled at 32 px × 34 lines (5–6 bits).
- **Palette Swap.** It exchanges Cr and Cb, a hue reflection, rather than a designed palette.
- **Cross-resolution.** Geometry is in virtual 1080-line units, so 720p and SD frame the same picture. That includes the v0.6
  row heights, block scales, warp and slip, which are in virtual units. The outline width and the output softness are in output
  pixels, so they are relatively thicker at SD. SD squeezes the full 16:9 width into 720 px, so it is horizontally compressed and
  round shapes are not round. That is unchanged from v0.5. SD has not been simulated or flashed.
- **Phase-2 trade-offs.** Zebra density is fixed at the photo's (the old K3). The bars' roll rides on K4 at a quarter of the flow
  (v0.5 had a separate K5 Roll). The S8 Tear Off option is gone; the photo's tear is always on.

## Working files
Phase 2 is in `.heirloom_work/rb_crossbeam/p2/`:
- `imodel8.py`, the v0.6 integer model, and `sheet8.py` with `rows_*.txt`, which give the control sheets `sheet_*.png`;
- `syn.sh` / `synp.sh`, standalone yosys with a nextpnr pack-only LC count, plus the `ab_*.vhd` ablations;
- `mksim.sh` / `sim.sh` (sim-only copies with the glides pre-set) and `cmp8.py` (RTL against the model).

`.heirloom_work/rb_crossbeam/`:
- `ref.jpg` and `notes.md`.
- Measurement and fit scripts:
  - `m1`–`m14.py`;
  - `st.py`, `integ2.py` (zebra field), `cmaps.py` (amplitude and ramp maps);
  - `fitx2`/`fitx5.py` (beam), `cellkeys.py`, `tables.py` and `geom.py` (the cell table).
- `proto.py` → `pfinal.py` (the float proto with controls), `proto_final.png`, `cmp_proto.png`, `sheet.py` → `sheet.png`.
- `hwtables.py` and `hwtables2.py` (the quantised tables); `imodel4`–`imodel7.py` (the integer models; `imodel7` is the shipped RTL); `mkhook.py` (writes `crossbeam.py`).
- `parts/` (the RTL source pieces), `abl/` (ablations), `prof/` (critical-path profiles), `rdiff.py` (RTL against model).
- Renders: `rtl720_osc7.png`, `rtl720_vid9.png`, `rtl720_mot9.png`, `cmp_final.png`.
