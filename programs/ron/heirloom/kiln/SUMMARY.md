# Kiln — camera fired in a frame-store kiln

**v0.6.1 phase 2 FINAL (2026-11-03): the v0.6.1 diag merged into production.** Ron flashed the v0.6.0 diag ("kiln is
pretty much the perfect program") and asked for one change: replace Quench with an option that makes K1 Tiles glide and
lock into whole tile counts. That is v0.6.1's S10 Tile Lock; Ron then said "finalize kiln". `out/rev_b/ron/heirloom/kiln.vmprog`
is the v0.6.1 diag's six bitstreams repacked with the production TOML. See [v0.6.1](#v061-s10-tile-lock) and
[Phase 2](#phase-2-v060).

## v0.6.1: S10 Tile Lock
Ron's note on v0.6.0 (2026-11-02): "in place of the quench knob, make an option that changes the behavior of K1 Tiles.
Make the X scale glide and lock in to the next quantized set of tiles so that there's always an even set of tiles on
the screen in that mode."
- **S10 Tile Lock: Free / Lock** (Quench is gone).
  - Free is K1 exactly as v0.6.0 (bit-identical tile scale at rest).
  - Lock gives whole copies across: 1 .. 6, in six equal bands of the K1 travel (1 + K1 × 6 / 1024; ~17 % each, K1 default
    551 = 4 copies). A copy count always fits the frame width exactly (3 copies at 1920 lands within 0.05 px), so the
    tiles meet the right edge evenly in every mode, and Drift scrolls them seamlessly.
- **The glide.** The tile scale the divider uses (`f_nl`, copies × 256) eases 1/16 of the gap per field toward the locked
  count, then steps the last 15/256 one unit per field, so it lands exactly with no final jump. 2 → 3 copies: 63 % in 16
  fields, exact in 67 (1.1 s at 60 Hz); 1 → 6: 1.55 s. It chains after K1's own knob glide, giving an S-curve. Flipping
  to Lock glides from the current scale; flipping back to Free glides back to the K1 value, then tracks K1 directly.
- **Why equal bands, not the nearest whole count.** K1 is exponential (0.5 .. 6.4), so rounding to the nearest count left
  "1 copy" on the bottom 42 % of the knob and "6" on 6 %: half the travel would do nothing in Lock. The nearest-count map
  is a one-line change if wanted (then the default sits at 2 copies, the photo's 1.93 rounded).
- **Cost.** Two per-field sequencer states (zc 101 target and gap, zc 102 step), an 11-bit register; Quench's `768 − s`
  subtractor left pixel stage 13. Arithmetic checked at RTL widths (every start/target pair converges exactly,
  monotonically, no wrap: `.heirloom_work/rb_kiln/p2/lock_sweep.py`). GHDL analyse / elaborate / synth clean. Not
  simulated (per-field control behaviour; the default frame path is unchanged).
- **Presets** are unchanged (all have Tile Lock Free, the old Quench Off).

v0.5.0 (2026-10-27): built, NOT flashed. Rebuilt from the photo WIN_20230515_23_17_11: camera video in a
frame store zoomed to a quadrant, repeated across the frame (1.94 copies) with the bottom row of quadrants
mirrored, run through a fire colouriser (royal blue, violet, purple, magenta, red, orange, yellow) with a
brightness lift, and torn into horizontal fibres with an interlace comb.  Presets are hand-written placeholders.

## Phase 2 (v0.6.0)
Ron left no notes for kiln (free hand, 2026-11-02, built while he was away). The map follows the phase-2 brief:
wider ranges, element-level motion instead of more zoom, a real look-flip on the near-dead switch, and no control
dead in either Source mode. **The default Osc frame is bit-exact with v0.5** (integer model, 960 × 540, fields 4 and
40); the default Video frame gains faint pale rims (K5, see below).

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Tiles: 1 .. 4 copies, linear, photo 1.94 at 31 % | **0.5 .. 6.4 copies**, exponential (piecewise-linear exp2 of K1 × 230 / 256 on the per-field multiplier), photo 1.934 at **54 % (551, bit-exact)** | 0.5 = one half of the camera quadrant fills the frame; 6.4 = a dense frieze. Display now 0–100 % |
| K2 Fibres: fibre texture 0 .. 2x (the same factor on the colour noise and the sideways displacement) | **colour noise 0 .. 2x as before; the sideways tear 0 .. 3.9x**, square law above the photo (8 + (K2 − 512)² / 8192 eighths, clipped at 31: ×1.5 at 68 %, ×2.6 at 83 %, ×3.9 from 92 %) | The tear has its own factor in the knot multiply; at the top the alternate SD lines pull ~±35 px apart (stronger venetian blinds). A ×17 version (mantissa + shift) was built and cut for fit. Display 0–100 % |
| K3 Blocks: 0.5 .. 2x, photo at 50 % | **0.25 .. 4x**, exponential (an octave per quarter), photo still 1x at 50 % (bit-exact) | 0 % melts everything into the red mids; 100 % posterises into blue, yellow and white-hot |
| K4 Drift: bipolar scroll (still at the centre, square law) | **unchanged** | A sideways wave past the default speed (chevrons, ±34 cells) was built and rendered in the model, then cut: with it the chip sat at 97.5 % and HD Analog missed on all five seeds (65–73 MHz, routing-bound). Kept as `.heirloom_work/rb_kiln/p2/kiln_p2d_wave.vhd` |
| K5 Rims: lift 0 .. 2x (**dead in Video**: the lift came only from the stored Osc map) | **−1x .. 2.9x** (K5 / 32 − 8 eighths), photo 1x at 50 % (bit-exact); **Video: rims traced on the camera's edges** | Negative etches the rims dark; 25 % = no lift. In Video each bilinear corner's lift = its two neighbours' \|code difference\| on the camera grid (min(e, 15) × 2), through the same 2-bit interpolation as the Osc lift, so pale rims outline the camera picture (default: faint) |
| K6 Glaze | unchanged | |
| S7 Mirror, S8 Kaleido, S10 Quench, S11 Source, P12 Heat | unchanged (v0.6.1: S10 Quench → Tile Lock) | The drift accumulator now wraps at two copies (Kaleido's period), fixing a flip-pop every copy of drift with Kaleido on |
| S9 Weave: Comb (photo) / Smear (4x longer fibres, no comb: nearly invisible) | **S9 Strips: Off (photo) / On** | 8 bands per half (by the mirrored store row, so the strips mirror too), each slid by its own quarter-copy offset; bands 1, 2, 4, 7 scroll with Drift while the rest stand still, so with any drift the strips slide past each other. Per line: the store-column start = the drifted (or, for a standing band, undrifted) base + the band offset in the top 3 bits. The comb is now always on |

**Per-field additions.** The K1 exponent (multiplier slot 2: K1 × 230 / 256) and the tear's square (slot 13) share the
existing serial multiplier; K3 / K5 decode to bit-slices (no adders).

**Fit (the first full feature set synthesised to ~110 % LC).**
- The chroma filter's 8-back delay lines (2 × 8 × 10 FF) moved into two 256 × 16 block-RAM rings.
- The C_LAT sync delay (3 × 32 FF) moved into a 256 × 4 ring.
- All rings share one address counter.
- The line and smooth noise layers share one 3-stage hash pipeline: at a smooth crossing its two rows follow the
  line knot on the next two clocks, and the pre-roll runs 9 states.
- The Video and Osc lifts share one corner mux.
- v0.5's Glaze / clamp stage (the routed critical path at 97 %) is split: Glaze moved into stage 21, and the
  chroma clamp is done in the signed domain (±448, then + 512 as a bit flip). Same arithmetic, same depth.
- The K4 wave and the ×17 tear were cut (see the table). Result: 7,250 LC (94 %) against v0.5.1's 7,164; HD Analog
  closed on router2 seeds 1–4 (76.7–79.4 MHz, v0.5.1: 76.4 on seed 3).
- Pixel-path depth unchanged (U/V parity), latency 32 in every mode, generated avid / gate / conditioning
  unchanged.

**Presets:** Photo (= defaults, Osc), Cold Kiln, White Hot, Kaleidoscope (re-aimed), and new Torn (K2 high), Frieze (6 tiles,
Kaleido), Strips (S9 On), Etched (dark rims, Blocks high, Video).

Build (router2, `.heirloom_work/diag_build.sh kiln`, 2026-11-02): HD Analog s1 78.14, SD Analog s1 73.18, HD HDMI s1
76.23, SD HDMI s1 74.29, HD Dual **s6** 76.02 (seeds 1–5 missed), SD Dual s1 76.31 MHz; 7,225–7,253 / 7,680 LC (94 %),
27 / 32 EBR (+3: two chroma-delay rings and the sync ring). No pin-skew screen (builder seeds kept).

**Checks:**
- GHDL analyse / elaborate / synth front end clean.
- Integer model `p2/imodel_p2.py` (the v0.6.0 spec): default Osc bit-exact with v0.5's `imodel.py`.
- Knob arithmetic swept at RTL widths (`p2/sweep.py`): no wraps.
- GHDL default-frame render (Osc, 960 × 540, warm-up 3, K4 centred) against v0.5's `rtl_osc2`: identical except the first 4
  visible pixels of each line, which differ in chroma only. That is the v0.5.1 resync fix: rtl_osc2 predates it, and
  the box filter now fills from neutral at the line start (U 640 = 512 + 9/16 × 223 at the first pixel). The render
  was made on an intermediate build (wave engine present but idle, before the Glaze split and the cut). Its default
  path is the same as the final one, since the split is equivalent arithmetic and the cut parts are zero at the
  default.
- Control sheet from the model: `.heirloom_work/rb_kiln/diag/sheet_v060.png`.
- Not simulated: strips, tear and Video rims (model only; the GHDL render covers the default path, the shared hash
  pipeline and the block-RAM rings).

## Reading of the photo
The measurements and fits are in `.heirloom_work/rb_kiln/notes.md`.
- **Repeat.** Horizontal translation period 992 px (0.517 W, 1.94 copies): shift MAD 34.5 against a 58
  baseline, the minimum at 990–994 in every row band; low-pass correlation between the copies 0.77.
- **Mirror.** Top/bottom about row 514 (MAD 35.1 against 61.6; correlation 0.74).  There is no clean
  left/right fold inside a copy: a triangle-fold model scores 24.3 against 15.6 for plain translation and
  33 for a null, so the partial in-copy symmetry is camera content.
- **Reading.** A camera frame shrunk into a 992 × 540 quadrant, tiled across, the bottom row reflected.
- **Colouriser.** The colour is two-dimensional (RGB PCA 64/26/9 %, unchanged by low-passing), so it is not a
  1-D colourizer of luma.  It fits as colour = P(s) + b·D(s) (alternating least squares, rms 19 luma
  levels):
  - s = position along a 64-step fire ramp: royal blue → violet → purple → magenta → red → orange → yellow;
  - b = an achromatic brightness lift (≈ +0.2 Y per unit) carrying the pink rims and ring edges.
  The yellow blocks are the clipped top of the ramp (yellow Y 790, flat inside, about 13 % of the frame).
- **Fibres.** Added after the store: the high-pass correlation between the two copies is only 0.10.
  - One SD line tall (2.25 rows): v-ACF 0.75/0.51/0.31 of the colouriser residual; luma high-pass std 15.5;
    h-ACF 0.92/0.75/0.44/0.20 at 4/8/16/32 px.
  - An interlace comb: a vertical spectral peak at 0.232 cycles/row (two SD lines), concentrated beside the
    yellow blocks (the "venetian blinds").
  - The residual grows with |∂s/∂x| (4.4 → 9.5 steps) faster than with |∂s/∂y|, so part of it is a sideways
    displacement of the content (~8–10 px).
  - No line tearing: row-to-row best shift 0 ± 1.
- **Capture.** The photo is an upscaled SD capture: ~10–15 px soft edges, chroma softer than luma.

## What it does
1. **Store.** A 128 × 64-cell scene of one quadrant: 4-bit s codes, a 3-bit lift b and a 1-bit comb map.
   - Osc: a ROM least-squares-fitted to the photo's top half (the left copy).
   - Video: the camera frame, written live as a 4-bit luma code per cell, IIR-smoothed along the line.
2. **Read-back.** Four bank reads give a bilinear s (4-bit fractions).
   - The column wraps every 128 cells (the tile repeat); the copy width is W / Tiles.
   - Store rows count down below mid-frame (Mirror) or restart (Repeat).
   - Kaleido flips every other copy.
3. **Lift.** The b field is read bilinearly with 2-bit fractions; the comb map is nearest-neighbour.
4. **Fibres.** Per-SD-line hashed noise:
   - a line layer with knots every 32 virtual px (128 in Smear), giving s and b offsets;
   - a smooth layer with knots every 64 vpx × 32 virtual rows (y-interpolated per knot), giving a
     displacement of the store column (56 D, in cell units) and a comb term whose sign alternates on alternate
     SD lines;
   - in the comb-map cells, ± 8.25 palette steps per alternate line.
   Fibres scales every knot value (16 steps) when the knot is hashed.
5. **Colour.** s' = (s − 40)·Blocks + 40 + Heat + breathing + noise, giving the index into a 128-entry
   palette: the photo ramp (0–63), yellow headroom (64–95) and yellow → white-hot (96–127).  Y gets b·DY;
   Glaze then transforms U/V.
6. **Conditioning.** Luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma toward black (~777), the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the horizontal repeat: 0.5–6.4 copies across, exponential (1.94 = the photo, at 54 %) |
| K2 Fibres | the fibre texture (colour noise 0–2x) and the sideways tear of the venetian blinds (0–17x, steep above the photo at 50 %) |
| K3 Blocks | the yellow blocks and blue patches: colouriser contrast 0.25–4x about the red mids (photo 1x at 50 %) |
| K4 Drift | the frame-store scroll (bipolar, still in the centre ± 8, square law, up to ~2 cells per field); past the default speed a sideways wave runs down (right) / up (left) the frame, up to ±34 cells |
| K5 Rims | the pink rims and ring edges: lift −1x..2.9x (1x at 50 %, none at 25 %); in Video, rims on the camera's edges |
| K6 Glaze | the fire palette's colourway: Fire (the photo) / Ice / Ember / Toxic / Violet / Mono / Neon / Rust |
| S7 Mirror | the top/bottom symmetry: Mirror (the photo) / Repeat |
| S8 Kaleido | the partial left/right symmetry: Off / On (every other copy flipped) |
| S9 Strips | Off (the photo) / On: 8 mirrored bands per half slid apart; half of them stand while the rest scroll |
| S10 Tile Lock | K1's behaviour: Free (continuous, the photo) / Lock (whole copies 1 .. 6 in equal K1 bands, the X scale gliding into each new count) |
| S11 Source | Video / Osc (default Video) |
| P12 Heat | the colouriser offset, glided: 0 % = a cold kiln (blue and violet, red embers only in the hottest spots); 50 % = the photo; 100 % = white-hot (yellow ground with pale white cores, orange fibres tracing the picture) |

**Motion at defaults.**
- The fibres rise one SD line every two fields: the noise keys and the smooth layer's rows move up.
- The glow breathes ± 3 palette steps (a 512-field triangle).
- The tiles drift slowly (K4 default +25).
- Positive Drift moves the copies to the right.

## Video
- **Into the store.** The camera frame is written into the 128 × 64 store (one 4-bit luma code per cell, IIR-smoothed along
  the line, the first line of each store row).  It is read back exactly as the scene is: squeezed into the quadrant,
  repeated 1.94× across, the bottom row mirrored, the fibres and comb on top, through the same fire palette.
  - Dark = blue, mids = red, highlights = yellow.
  - Heat and Blocks set where the camera's greys land on the ramp.
- **Osc.** The stored scene is fitted to the photo's own camera content.  Every control acts in both modes;
  in Video there is no stored lift and no comb map.
- **Detail.** Video is a 64 × 128-camera-pixel picture per copy (bilinear), soft in the way the photo's is.

## Engine
(v0.5 description below. v0.6.0 differences are in [Phase 2](#phase-2-v060): one shared hash pipeline, the chroma and
sync delays in block-RAM rings, the per-line Strips base, the Video edge-map lift, the tear factor, and Glaze moved to stage 21.
Smear and its 128-vpx knots are gone.)
- **Per field** (vblank zone, measured width W and active lines H):
  - six serial divisions:
    - cells per pixel (Tiles × 2^15 / W) and virtual px per pixel;
    - virtual rows, SD lines and store rows per line;
    - camera columns per pixel;
  - a serial multiplier for the drift speed (K4²);
  - Blocks, Rims, Fibres (16 steps), the comb-map term, Heat with breathing;
  - the motion counters (drift accumulator, rise, breathing phase).
  - Glides (± 4 deadband, 1/8 per field) on K1, K2, K3, K5 and P12.
- **Per line** (24 clocks after the source avid fall, computing the next line):
  - the store row and fraction (mirror / repeat, half-cell offset), the noise keys (SD line + rise) and the
    smooth-layer row and fraction, the comb sign (per SD line; per line in interlaced SD), the camera-write row;
  - a knot pre-roll (knots 0..2 of both layers).
- **Knot engine.** Two 3-stage 16-bit add-shift hash pipelines (seeds 0x5A3C, 0x9E37).  At each knot crossing the
  three knot registers shift and knot + 2 is requested.  The smooth layer hashes rows j0 and j0 + 1, then
  y-interpolates, and every value is scaled by Fibres at the knot, so the pixel path has no Fibres multiply.
- **Pixel path (22 stages + 6 conditioning + gate, C_LAT 32 in every mode):**
  - x-interpolation of the four noise layers; the displacement, s-noise and b-noise sums (shift-adds);
  - the store column (a column accumulator aligned four stages behind the beam);
  - bank addresses, the store read, a fabric register, the corners, the level decode;
  - y and x interpolation, Quench, Blocks, Heat, the noise and the comb term;
  - the palette index, the palette ROMs, a fabric register, the luma lift, Glaze.
  - The s/b noise rides from stage 5 to stage 15 in a 256 × 16 block-RAM ring.
- **Stores** (24 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | scene s | 4 banks × 2048 × 4 | 8 |
  | scene b + comb | 4 × 1024 × 4 | 4 |
  | camera | 4 × 2048 × 4 | 8 |
  | palette | 3 × 256 × 16 | 3 |
  | noise ring | 256 × 16 | 1 |

  `no_rw_check` on the camera banks and the ring (`emulate_transparency` = 0).
- **Contracts.** Generated avid (damier); one 32-clock latency in every mode; conditioning before the gate
  against HD HDMI bars; geometry from the measured raster (resolution-independent: the same picture at 1080,
  720 and 480i in the model renders); palette in the hardware convention (U = Cr, V = Cb), luma pre-boosted
  16/13 and tanh-compressed to ≤ 777 on screen.
- **The spec.** The integer model `imodel.py` is the spec.  The GHDL render (960 × 540, Osc, warmup 3) is
  bit-exact against it (mean |d| 0.00 at the 4-px avid framing offset), as is a motion render (K4 max, field 6).
- **Fit history.**
  - v1 (five noise layers incl. jitter, per-pixel Fibres multiplies, bilinear lift with chroma, a 4-ROM
    palette) rendered bit-exact but placed 8,938 LC (116 %).
  - Ablation showed the noise system ~2,260 LUT / 1,080 FF and the lift path ~1,800 LUT / 540 FF.
  - v2 moved Fibres to the knots, dropped the jitter layer, made the displacement cell-based, gave the lift
    2-bit interpolation and luma only (the photo's mean chroma lift baked into the palette), made the comb map
    nearest-neighbour and moved the noise delay into an EBR ring: 4,147 LUT / ~2,650 FF, no visible change in
    the model.

## Status
- **v0.6.1 FINAL, 2026-11-03 ("finalize kiln"); merged.**
  - `out/rev_b/ron/heirloom/kiln.vmprog` holds the v0.6.1 diag's six bitstreams, repacked with the production TOML (id
    `com.czartroniks.kiln`, name "Kiln"; every bitstream passes iceunpack). Staging: `.heirloom_work/rb_kiln/final_v061/`.
  - The merged sources match the diag's byte for byte, apart from the header comments of `kiln.vhd` and `kiln.toml`.
  - Hardware: v0.6.0 was flashed and approved. v0.6.1 adds only Tile Lock (default frame path unchanged).
  - Seeds (router2): HD Analog s4, HD HDMI s2, HD Dual s6, the SD configs s1 (Fmax below). To rebuild from source:
    `./build_heirloom.sh kiln`. router2 is nondeterministic, so a rebuild won't reproduce these routes; if HD Dual
    can't close, swap in the one-adder step (below) first.
  - Presets are still the hand-written placeholders. Capture Ron's looks with `./capture_preset.sh`.
- **v0.6.1 diag built 2026-11-02: S10 Tile Lock replaces Quench.** Router2
  (`.heirloom_work/diag_build.sh kiln`): HD Analog **s4** 76.41, SD Analog s1 74.82, HD HDMI **s2** 77.75, SD HDMI s1
  76.57, HD Dual **s6** 76.58, SD Dual s1 74.63 MHz; 7,341–7,361 / 7,680 LC (96 %), 27 EBR.
  `out/rev_b/ron/_diag/kiln_diag_v0.6.1.vmprog`.
  - Missed seeds were routing noise at 96 %. HD Analog s1–3 failed on the pre-existing stage-20 luma-lift multiply by
    0.24 MHz. HD Dual's path is congestion (stage 2, 9 ns routing / 4.4 logic); a profile route at seed 7 passed at 74.63.
  - Standalone yosys: the lock costs +98 LUT / +47 carry / +24 FF over v0.6.0. A logically identical one-adder step
    (`p2/kiln_v061_compact_step.vhd`) is +53 LUT / +39 carry. Swap it in if the production merge build can't close HD Dual.
- **v0.6.0 diag built 2026-11-02 (phase 2); flashed and approved by Ron.** Router2: HD Analog s1 78.14, SD Analog s1
  73.18, HD HDMI s1 76.23, SD HDMI s1 74.29, HD Dual s6 76.02, SD Dual s1 76.31 MHz; 7,225–7,253 LC (94 %), 27 EBR.
  `out/rev_b/ron/_diag/kiln_diag_v0.6.0.vmprog`. Changes: [Phase 2](#phase-2-v060).
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s3 76.41, SD Analog s1 75.09, HD HDMI s1 77.20, SD HDMI s1 69.09, HD Dual s2 79.31, SD Dual s1 73.84 MHz; 7142-7164 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 (2026-10-27), router2, `build_heirloom.sh`.**  Built, NOT flashed.
- **Build seeds:** **HD Analog = seed 3** (seeds 1–2 missed), **HD Dual = seed 2** (seed 1 missed); the other four are
  seed 1.

  | Config | Fmax (post-route) |
  |---|---|
  | HD Analog | 75.31 MHz |
  | HD HDMI | 80.16 MHz |
  | HD Dual | 77.14 MHz |
  | SD Analog | 71.16 MHz |
  | SD HDMI | 80.50 MHz |
  | SD Dual | 76.84 MHz (SD Fmin 27) |

- 7,119–7,139 / 7,680 LC (93 %), 24/32 EBR.  Every bitstream passed iceunpack.
  `out/rev_b/ron/heirloom/kiln.vmprog`.
- HD Analog is thin.  If a later version regresses, rebuild with `NEXTPNR_ROUTER="--router router1"` before
  restructuring.  LC to reclaim: share the two hash pipelines, narrow the store-column accumulator.

## Deviations from the photo, and why
- **The bottom half is an exact mirror of the top half.**  The photo's halves differ in detail: a chin-like
  silhouette at the bottom, and the bottom yellow blocks reach x ≈ 840 instead of 700.  Its mirror axis
  (row 514) is moved to the frame centre.
- **The right copy repeats the left one exactly.**  The photo's copies correlate 0.77 and differ in fine detail.
- **Rings and blocky glitches are softer.**  The scene is stored at 7.75 px × 8.4 rows per cell (the EBR budget),
  so the C-rings and the crisp orange rectangles are softer than the photo's.
- **The fibres are a procedural realisation** with matched statistics (high-pass std, h-ACF, comb concentration
  by map).  The comb is a little more regular than the photo's.
- **The lift changes luma only.**  The photo's mean chroma lift per palette entry is baked into the palette, which
  keeps saturation (p90 0.43 vs 0.46).
- **Overall luma is ~5 % lower.**  The yellow ground sits at ~760 on screen versus 790 (the 777 luma cap of the
  HD HDMI bar fix), and the centre band reads slightly more purple, less red.  The 4 × 4 grid mean |dRGB| of
  the RTL against the photo is 10.1; the hue histogram is within L1 0.13.

## Working files
`.heirloom_work/rb_kiln/`:
- `notes.md`: the reading, measurements and history.
- Measurement scripts: `sym*.py`, `foldfit.py`, `colfit.py`, `fibre.py`, `combmap.py`, `lsstoreC.py`, `pal.py`.
- Prototypes: `protof.py` (the float proto), `proto_final.png`, `cmp_proto.png`.
- The model: `imodel.py` (v2 = the spec) and `imodel_v1.py`.
- `mkdata.py`, `mkhook.py`: generate `kiln.py`.
- `sheet.png`: the control sheet.
- RTL renders: `rtl_osc2.png`, `rtl_mot2.png`, `rtl_vid2.png`.
- `cmp_final.png`.
- `kiln_v1_8938lc.vhd`: the first RTL.
- `synth.sh`, `abl/`: the standalone-yosys ablations.

## Settled questions (2026-11-02 batch)
Ron's approval of v0.6.0 settles both as built:
1. The K4 sideways wave stays cut, and the Video rims stay. (The wave code is kept in `rb_kiln/p2/kiln_p2d_wave.vhd`.)
2. The Video default keeps its faint rims.

**Watch on hardware:** no pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config; the next suspect
is the hsync delay running through block RAM.
