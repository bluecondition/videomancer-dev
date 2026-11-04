# Chalice — a mirrored chalice of rainbow contour rings

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230305_00_05_43_t0p5.png` (clip WIN_20230305_00_05_43 at 0.5 s; the clip is static).
A goblet drawn by thin rainbow contour lines on olive and black bands: a bowl of nested U rings with two
dark-red lobes and a pink ridge at the top and green/cyan rings in its wall, a pinched neck, a trapezoid
foot with chevron rings and an orange eye; yellow/orange/pink/purple/blue bands bending up into the bowl,
an olive plateau above them, black below the blue line. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_chalice/` (`notes.md`, `model.py` + `final_params.json` = the float
proto, `imodel.py` = the integer model, `diag.py` = the landmark comparer, `fit.py`/`fit2.py` = the table fits).
- **Mirror axis** x = 983 (flip-correlation of blue; red 985). Green's axis is 1020: G is delayed ~40 px
  through the chain (the pink/purple fringes on every edge).
- **Three colourisers of one field.** Every channel is a waveshaper of a single scalar φ, in ring units
  (1 = one blue period; higher φ = lower field):
  - R = clamp(255 − 58·|φ − 1.525|), a slow triangle (its peak is the orange band);
  - G = clamp(128 − 42·(φ − 1.2), 0, 128), a clamp: 128 over the whole top, 0 below the blue line;
  - B = clamp(2·tri(φ) − ½), a fast triangle peaking at integer φ with 50 % dead zones: the contour
    lines. B's input has the ramp clipped at −0.5 (the plateau is ring-free) and its total at 5.5 (the
    foot's blue edge).
- **Vertical ramp** (outer bands, x = 60): φ = −0.62 for y < 220 (plateau), 1.35 at 455, 3.97 at 685,
  6.0 = black at 780 (0.0084 / 0.0114 / 0.0214 rings per line). Ring peaks at y = 289, 401, 509, 580,
  685, 739.
- **The chalice**, subtracted from the ramp: K(y)·h(|u|/w(y)) − Kd(y)·dimple(|u|/w(y)) − S(y)·(1 − |u|/1000)².
  - h is the dome profile measured on the foot's red channel: 1 − 0.55 v² at the top, ½ at v = 1, 0 at 1.33.
  - The dimple exp(−(v/0.265)²) carves the crater top; through the fold it makes the two lobes at
    u = ±110, y = 90 (true minima, G = 41) and the pink ridge between them.
  - The skirt S·(1 − |u|/1000)² is the squared horizontal ramp. It bends the bands up into the bowl (the
    first ring rises from y = 289 at x = 20 to 178 at x = 540) and makes the black wedges beside the neck.
  - K: 10.4 (y 0) → 7.2 (480) → 3.1 (650, the neck) → 6.6 (790) → 3.9 (1080) rings.
  - w: 345 (0) → 210 (480) → 72 (700) → 320 (1000) virtual px.
  - S: 0.3 (y < 100) → 2.1 (300) → 2.3 (480) → 2.5 (1080) rings.
  - Kd: 2.0 (0) → 0.42 (300) → 0 (600) rings.
- **Wavefolder** at φ = −3: the rim (the red trough ring around the bowl, 370 px from the axis at the top,
  closing at y = 490) and the lobes being minima both need it.
- **Rings**: 11 blue rings on the axis from the lobes (y = 90) to the neck (643). The foot is nested loops
  pinching at (983, 640), with an eye (R = 255 flat) from y = 620 to 760 and straight edges at 0.80 px/line.
- **Texture**: per-line jitter mostly 0, 1–2 px on ~2 % of lines; row-to-row noise 0.45.
- **Fit**: float proto 23 mean |RGB| vs the still (`cmp_proto.png`); integer model 26.

## Motion in the clip
- The patch holds still. Every landmark (axis rings, outer rings, foot edge, lobes) stays within ±1 px /
  ±2 lines from t − 0.5 to + 1.5 s, and the mean frame difference is 2.8 (noise and jitter only).
- The other stills of the same patch show Ron's knobs: the camera patched into the field (a head inside
  the bowl, bands tearing around it; clips 05_15 t12 and 10_36 t0.5) and the colouriser rates raised
  (06_06 t3).
- The program keeps the static picture and its jitter: ±1 virtual px on 1 line in 64, so two consecutive
  RTL frames differ by 0.39 mean |RGB|.
- It adds one slow moving part, K5 Breathe: the ramp slides under its fixed plateau and black clips, so
  the bands rise and fall ±0.29 ring (±34 lines) with a 20 s period (integer model: rings at 279 / 245 /
  280 / 313 lines at 0 / 5 / 10 / 15 s). It starts at its zero crossing, so the first frames are the still.
- In Osc mode the source lattice drifts at about 75 virtual px/s.

## What it does
1. **Per line** the engine evaluates five piecewise-linear tables (ramp, K, w, S, Kd) at the virtual
   line, one serial product each, with one shared add. It then forms 1/w by a serial divide, the v start
   |u0|/w, and the skirt's DDA constants (three products of S).
2. **Per pixel** two mirrored DDAs carry v = |u|/w (exact) and the skirt parabola (second order, its
   increment negated at the axis). v reads the profile ROM (h 12 bits, dimple 8 bits). Two multipliers
   give K·h and Kd·dimple, and φ_raw = ramp + skirt − K h + Kd dimple + camera.
3. **Fold**: φ = −3 + |φ_raw + 3| (Rim on). B's copy has the ramp clipped at −0.5 and the total at 5.5.
4. **Shapers**: R and G as measured (slopes 15/64 and 5/32 per 1/256 ring), B a triangle whose phase is
   scaled by K4. G passes through a 40-virtual-px delay ring. Ice swaps the R and B colourisers.
5. **Output**: matrix RGB → hardware YUV (Y 5/10/2 over 4, Cr 56/−47/−9 over 32, Cb −19/−37/56 over 32;
   luma pre-scaled 16/13, capped 940), then the shell's two [1 2 2 2 2 2 2 2 1]/16 boxes and 13/16 luma.
   Drawn whites come out at about 690.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Ramp | the vertical ramp's slope: band density and where the black starts (×0.4 .. ×2.4; 512 = the photo) |
| K2 Bowl | the bowl's width (×0.5 .. ×2) |
| K3 Foot | the foot's flare (×0.4 .. ×2.4) |
| K4 Rings | the blue colouriser's rate: contour-ring density (×0.25 .. ×4); 0 = soft bands, 1023 = dense fine rings |
| K5 Breathe | a slow triangle LFO (20 s) sliding the ramp under its plateau: the bands breathe (0 .. 1.5 rings; 200 = 0.29) |
| K6 Camera | how far the camera (or the Osc) bends the field (0 .. 1.8 rings per full luma; 256 = 0.47) |
| S7 Lobes | the two lobes and the pink ridge at the top of the bowl: On (the photo) / Off (plain U rings) |
| S8 Rim | the wavefolder: On (the rim ring, the photo) / Off (a deep well of green/cyan rings) |
| S9 Flip | the field upside down: the bowl becomes a dome on a black sky |
| S10 Palette | Chalice (the photo) / Ice (red and blue colourisers swapped: a teal chalice with red rings) |
| S11 Source | Video / Osc |
| P12 Chalice | the chalice amplitude (K, Kd, S together): 0 % = plain rainbow bands; 410 = the photo; 100 % = a towering chalice of dense rings over a black well |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, times K6, is added
  to the field before the fold. The rings bend around the picture, the way the clip's other stills show
  Ron's camera inside the bowl. With no input the frame is the still.
- **Osc mode.** The shell's 640 × 540 virtual-px drifting triangle lattice (~75 px/s) feeds the same path.
  At the default Camera the rings wobble gently and the upper corners wash lavender; K6 scales it.

## Engine
- **Vblank-zone sequencer.** A shared 7-channel glide (K1–K6, P12, 1/8 of the gap per field) and four
  exp2-ROM knob curves (zc 30–46). Then 32 slots on one serial shift-add multiplier (zc 46–814, 24 clocks
  each, operands pre-muxed), writing the knob-scaled starts and slopes of the five tables into a 64 × 16
  block-RAM table (`no_rw_check`), plus the breathe offset.
- **Line engine.** Kicked at each g_start (and at zc 1300 for line 0), it computes the NEXT line in
  ~177 clocks, which fits SD's 858-clock line:
  - segment select and the table read just in time;
  - 5 serial products with one shared evaluator add;
  - a 29-step serial divide 2^28 / w;
  - the v start (± 2/w on 1 line in 64);
  - the skirt constants f0, D0, i2.
- **Beam.** A = |u|·g (± g·xs per pixel, mirrored at the axis step). The skirt DDA runs five clocks
  behind as F += D, with D ← −D at the axis, else D += i2.
- **Pixel path (23 stages).** 2 profile ROM address, 3 ROM, 4 copy, 5 K·h / Kd·d partials, 6 sums, 7 base
  + skirt and camera × gain, 8 raw phase, 9 + camera, 10 fold, 11 distances and B copy, 12 |dr| and B clip,
  13 shaper products and B rate partials, 14 R / G and the B product, 15–16 B triangle, 17 B shaper and
  palette, 18 G ring write, 19 ring read, 20 copy / early mux, 21–22 matrix, 23 offsets, cap, black outside
  the avid. Then boxes 24–27, luma 28, gate 29. C_LAT 28.
- **Block RAM (17 EBR).**
  - profile ROM 1024 × 20 (5)
  - exp2 256 × 16 (1)
  - line buffer 2048 × 8 (4)
  - table 64 × 16 (1)
  - G ring 256 × 8 (1)
  - output-box 8-back rings 4 × 256 × 16 (4)
  - sync delay ring 256 × 4 (1)

  All rings carry `no_rw_check`; the only transparency emulation is the line buffer's.
- **Contracts.**
  - generated avid;
  - one latency (28) in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - [1 2 2 2 2 2 2 2 1]/16 Y/U/V boxes and 13/16 luma before the gate;
  - the running sums snap to neutral off the avid (resync);
  - colours in the hardware convention (U = Cr).
- **Verified.**
  - GHDL 720p60, decimation 1, default knobs, Osc and Video (selfie.jpg): the RTL matches the integer model
    at 7.4 / 8.6 mean |RGB| once the box-filter delay is aligned (R and G within 1–4 levels; the remainder
    is sub-pixel blue edges and different jitter lines).
  - A second capture (output frame 4 vs 3) moves only by the jitter and one field of Osc drift.
- **Fit history.**
  - The first build overflowed at 8,623 LC (112 %).
  - Restructured: the per-field constants moved into the table RAM, the output-box taps and syncs into
    block-RAM rings, the line engine shares one adder, the skirt DDA runs at 2^16 precision, the camera
    gain is 8 × 5, and the R / G shaper slopes take one adder each.
  - Bugs found by the GHDL render and fixed before the build: table-index adds wrapping at 2 bits
    (`resize(e_kseg + 9, 6)`), the w(1000) operand, and one-pixel misalignments of the camera read, the G
    ring and the early-G counter.

## Status
- **v0.1.0 built 2026-11-04 through the shared queue, router2 (builder default), all six configs on
  seed 1.** NOT flashed.
- Post-route Fmax: HD Analog 84.02, HD HDMI 84.12, HD Dual 84.82, SD Analog 88.94, SD HDMI 87.59,
  SD Dual 86.84 MHz.
- 6,923–6,952 / 7,680 LC (90.5 %), 17 / 32 EBR. iceunpack OK ×6; the packed config carries the new labels
  and presets.
- Output: `out/rev_b/ron/heirloom/chalice.vmprog`.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh chalice` (or `./build_heirloom.sh chalice`).
  If a later version gets tight, try `NEXTPNR_ROUTER="--router router1"` first.

## Deviations from the photo
- The bowl is a V-ish U rather than the still's round heart, and its lower sides are straighter. The
  green/cyan rings sit inside the bowl wall rather than forming a thick outer rim.
- The foot shows a heart-shaped lower ring and one more ring pair than the still: in the still, the blue
  channel's foot dome is narrower than the red one's, and the program draws both from one field.
- The olive plateau is flat. The still's plateau drifts olive → ochre toward the bowl, a red-only effect
  no single field reproduces.
- The axis is centred (960) where the still has it at 983.
- The default frame breathes slowly (K5 = 200, ±0.29 ring over 20 s) while the clip is static; K5 = 0
  gives exactly the clip's static picture plus its jitter.
- Cut for fit, invisible after the output blur: R and G shaper slopes 15/64 and 5/32 (3–5 % off the
  fit), a 32-step camera gain, the skirt at 2^16 precision, S clamped at 4000.
- Resolutions: 720p shows the same geometry as 1080p. SD shows the centre 4:3 (1440 virtual px), so the
  outer bands are cut at the sides; the G delay is 20 px there. The line jitter is ±1 virtual px (sub-pixel
  at 720p and SD).
