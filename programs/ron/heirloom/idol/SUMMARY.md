# Idol — a camera idol folded in a frame-store mirror, colourized on flat yellow (Heirloom #31)

**v0.6.1 (2026-11-03): phase 2 FINAL except presets.** Ron approved the v0.6.1 diag after flashing it, and it was
merged into production (see [v0.6.1](#v061-bold-axis), [Phase 2](#phase-2-v060) and the
[sign-off](#phase-2-sign-off-2026-11-03)). Built 2026-11-03 with router2, all six configs on seed 1:

| Config | Post-route Fmax |
|---|---|
| HD Analog | 75.13 MHz (thin) |
| HD HDMI | 74.79 MHz (thin) |
| HD Dual | 79.83 MHz |
| SD Analog | 76.21 MHz |
| SD HDMI | 69.87 MHz |
| SD Dual | 78.31 MHz |

Fit: 7,199–7,224 / 7,680 LC (94 %), 27 / 32 EBR. Output: `out/rev_b/ron/heirloom/idol.vmprog`. The production
program itself has not been flashed; Ron flashed the diag builds.
**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source: `photos/WIN_20230515_23_14_37_Pro.jpg`.
v0.1.0 (a relief-shaded column of the picture) was judged too abstract. This version measures the photo, fits
a store of its field and reproduces it through the measured colourizer, mirror, key, smear and streaks.
Working files: `.heirloom_work/rb_idol/`: `notes.md`, `protof.py` (float proto), `gfit.py` / `quant.py`
(store fit), `imodel.py` (integer model), `sheet.png` (control sheet), `cmp_final.png`.

## v0.6.1: bold Axis
Ron's note from flashing v0.6.0 (2026-11-02): "not seeing much happening from P3 Axis".

**Why it was weak.** Every band of the old fold showed the same store at slope ±1, so moving the axis only slid
seams. In Osc the store spans just virtual rows −40 .. 520, and its own edge already folds at row 520.
- From the photo up to 92 %, the top 520 rows never changed; only a strip at the bottom did (at 100 %, 8 % of
  the rows).
- Below the photo, the idol just scrolled down.
- With Mirror off, K3 did nothing at all, which broke the "every control always live" rule.
- Preview of old against new (rows of the default render re-mapped through both line engines):
  `.heirloom_work/rb_idol/p2/axis/axis_preview.png`.

**Now:** the idol rides on the mirror line. The line engine folds a **store-side row** (a DDA, `n_vs`) about the
photo's line, not the screen row about the knob's line. Its step decides where the fold lands on screen:
- **Top half:** step = ys × 512 / axis. One more division per field on the shared divider (zone clocks 135 .. 159)
  stretches or squeezes the idol's top half to end exactly at the axis.
- **Below the photo setting:** the same step continues past the line, so the whole fold shrinks. The store's
  reflections then stack idols down the screen: 2 at 25 %, 4 at 0 % (period = twice the axis height).
- **Above the photo setting:** the reflection below the line stays 1:1. At 100 % the idol's top half fills 90 % of
  the height and the diamond sits on the bottom edge.
- **Mirror off:** the store's middle row follows the axis the same way, so K3 is live in both S7 positions.
- **Range:** the line sits at 13 .. 91 % of the height (axis divisor 143 + 0.797 × K3 rows), the photo's 48.1 % at 463.
  At that setting the step is exactly ys in every mode, so the default frame is unchanged from v0.6.0 (`n_vs` ≡ the
  old screen row, the compare constant = the old default axis).
- **Engine:** 7 reflection states (was 4, worst case 7 with Mirror off at 0 %), so the line-engine states after
  them moved +3. The last l_* write is now 61 clocks after the source avid fall: hardware has 138 .. 370, the
  sim's fake hblank 64.
- **Checks:** GHDL front end clean. `.heirloom_work/rb_idol/p2/axis/sweep_axis.py` checks every K3 position at ys
  256 / 384 / 553 / 576 / 1152 / 1382, Osc and Video, Mirror on and off, slips ±38 rows: 0 problems; the default
  step equals ys in every mode; `n_vs` ≤ 988,364 < 2^20.
- Preset Kaleido (K3 80) now gives about 3 stacked idols with Twin.
- **Build** (router2, 2026-11-02, all seed 1, no pin-skew screen):

  | Config | Post-route Fmax |
  |---|---|
  | HD Analog | 75.13 MHz (thin) |
  | HD HDMI | 74.79 MHz (thin) |
  | HD Dual | 79.83 MHz |
  | SD Analog | 76.21 MHz |
  | SD HDMI | 69.87 MHz |
  | SD Dual | 78.31 MHz |

  Fit: 7,199–7,224 / 7,680 LC (94 %; v0.6.0 was 93 %), 27 / 32 EBR. If an HD config glitches, reseed it before
  restructuring.

## Phase 2 (v0.6.0)
Ron left no notes on idol (2026-11-02, free hand), so the map follows the phase-2 brief and the finished rows: wider
ranges each side of the photo, the subtle controls made bold, a palette family on a switch, and new motion that is
zero at the default settings (curtain lesson). The idol is a fitted store, so the new parameters act on its modules:
the tear generators (streaks, slips), the mirror, the colourizer bands (swell) and the frame-store readout (ripple).

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Feather: streaks 0 .. 2.5x, photo at 40 % | **Feather: the tears: streaks 0 .. 4x and slips 0 .. 24x as often**, photo at 40 % (bit-exact) | Up to the photo the streak curve is v0.5's; above it, 1.25 per step to F = 4x. Above the photo the four busy levels grow at 102 / 154 / 205 / 225 (v0.5: 205 / 205 / 205 / 225), so the calm areas only reach v0.5's top noise and the idol stays legible while the busy columns shred (amplitudes 305 / 521 / 746 / 898 at 100 %, 11 bits; a first cut at 5x with even growth was noise soup). Slips follow the same F: none below 20 %, the photo's 1/256 per line from 20 to 40 %, then +1/256 per 1/8x, to 24/256 at 100 % (more than half the lines slipped). 0 = a clean idol |
| K3 Axis: mirror line 30 .. 70 %, photo 48 % at 45 % | **12 .. 92 %**, photo still at 45 % (463, bit-exact) | Twice the reach each side. Low settings fold the store into stacked mirror copies (two reflections); 92 % is nearly the unfolded store |
| K4 Speed: colour cycling only | **Speed: colour cycling (unchanged) + band swell + row ripple**, depth = \|speed\| past the default | Depth d = \|f_sp\| − 88 (0 for K4 416 .. 608, so the default 600 is still; 416 at either end). **Swell:** the palette index gets + sin(2 x palette position + phase) x (d / 32) / 8 (to ±206 of 2048, past the monotone limit at the top, so bands fold over; 14 depth steps): the colour bands swell and thin in sequence. **Ripple:** each line's DDA start gets + sin(folded row / 256 rows + phase) x (d / 16) x 128 (to ±6.4 store cells = ±129 px at 1080p Osc): the rows writhe sideways, mirror-symmetric. Both phases ride the colour-cycle accumulator, so they travel with K4's direction |
| K6 Ground: key window 0 .. 0.28 cycle each side, photo at 30 % | **0 .. 0.44 cycle each side**, photo at 30 % (bit-exact) | Above the photo 1.19 per step (was 0.72): at 100 % 89 % of the cycle is flat ground, only a blue band left |
| S9 Slips: On / Off (a few displaced slices: subtle) | **S9 Palette: Photo / Rainbow** | Rainbow: the hue once round the wheel on the photo's anchors (yellow at 0, blue at 0.5, red at 0.84), luma half the photo profile, half the hue's own, chroma at 0.95 of the gamut; the key fill turns black. Slips moved onto K1 |
| P12 Bands: gain 0.5 .. 1 .. 4, linear halves | **0.25 .. 1 .. 4**, exponential (piecewise-linear octaves), photo at 50 % (bit-exact) | Twice the low side: 0 % is now near-flat colour (a quarter cycle across the field); the top is unchanged |

Unchanged: K2 Hue (already a full cycle), K5 Smear (already 1 .. 128 px, the ring's limit), S7 Mirror, S8 Twin,
S10 Negative, S11 Source. Control names are unchanged except S9 (Slips -> Palette).

Engine changes:
- Per field: the K1, K3, K6 and P12 curves (one zone clock each), the slip threshold, the K4 depth and the
  swell / ripple phases (zone clocks 137-138).
- Per line: a 16-entry quarter-wave sine and a signed 7-step serial multiply beside the row multiply. They give
  the ripple, which is folded into the DDA start with the centring offset.
- Pixel path: two new stages after the palette index (14a: sine of the palette position, 14b: the 8 x 5 swell
  product, both kept in LUTs) and the add in the palette-address stage.
- Palette ROMs: 512 deep (family x 256), +1 EBR.
- Fit: the output chroma filter's two 8-deep FF delay lines went into a 256 x 16 block-RAM ring (shares the smear
  rings' pointer: read write − 6, one fabric register), the PHASE2 budget tip; +1 EBR. The first build without it
  (and with a 9-bit swell depth and an add/sub ripple) was 95.8–96.3 % LC with HD Analog on seed 2 at 75.26 MHz.
- Latency: 28 -> **30** in every mode. The pixel depth and the sync latency both moved by 2, so the
  generated-avid U/V parity is unchanged; the shell's avid taps moved +2.

Presets: Photo (the defaults), Idol Osc, Totem, Shredded (was Feathers: K1 1000, K5 900), and new Writhe (K4 960:
swell + ripple), Rainbow (S9, P12 640), Kaleido (K3 80, Twin), Calm Flood (K1 0, K6 800, P12 150). All but Photo
are in Osc.

Build (router2, 2026-11-02, builder seeds, no pin-skew screen): HD Analog s1 **74.29** (thin: a profile re-route at s2
gives 72.61, critical path routing-bound in the zone-decode -> serial-multiplier enable at s1 and in the core SPI
peripheral at s2), SD Analog s1 72.91, HD HDMI s2 76.79 (s1 missed), SD HDMI s1 69.41, HD Dual s1 83.28, SD Dual s1
76.82 MHz; 7,131–7,155 / 7,680 LC (93 %, v0.5.1 89 %), 27 / 32 EBR. If HD Analog regresses on a rebuild: register
the serial multiplier's zone window one clock early, or reseed. `out/rev_b/ron/_diag/idol_diag_v0.6.0.vmprog`
(copy in `out/rev_b/ron/heirloom/_diag/`).

Checks: GHDL analyse / elaborate / synth front end clean; a Python sweep of the new knob arithmetic at RTL widths
(`.heirloom_work/rb_idol/p2/sweep.py`: defaults bit-exact, no wraps, monotone, the fold needs at most 2 of the
engine's 4 reflections); **default frame bit-identical to production v0.5.1** (GHDL, Osc, 960 x 540, 0 of 518,400
pixels differ, before and after the fit changes); renders at K4 100 % (the totem writhes sideways, mirror-symmetric,
bands swell: red fat, blue thin) and K1 100 % + Rainbow (shredded but legible: neon rings on black, slices slipped)
in `.heirloom_work/rb_idol/p2/` (k4max.png, k1max_rb.png). Not verified: Video mode with the new controls, the
Rainbow palette's hues on hardware, motion over time (swell / ripple travel), SD / 720p.

## Reading of the photo (measured)
- **Camera content.** The blue channel shows a small idol figure (hat, eyes, nose, an open "O" mouth) twice,
  at x ≈ 600 and x ≈ 920 (324 px apart, correlation 0.81 in red). There is no global horizontal repeat:
  translation and left/right mirror searches score 58–62 against a 74 baseline.
- **Mirror.** Top/bottom only. Flip-correlation puts the axis at row 518–519 in every 160-px column band
  (mean |diff| 26 against a 74 baseline; 10–15 in the smooth columns). That is 0.48 of the picture height.
  Fine detail differs between the halves, so the streaks are added after the mirror.
- **Colourizer.** The colours form one 1-D cycle. A 64-node cyclic self-organising map fits them as well as
  unconstrained k-means with the same node count (median error 10.8). Re-colouring the photo through it
  reproduces the photo. In order: yellow (239,243,0) → light orange → orange (242,182,84) → peach
  (238,190,123) → cream (236,219,141) → pale (207,192,170) → grey (133,133,135) → light blue (117,166,194)
  → blue (12,164,236) at 0.5 → navy (45,101,146) → dark brown (66,40,31) → dark red (89,25,5) → red
  (191,23,2) at 0.84 → orange (244,160,33) → yellow. The sparsest point, the red → orange step at 0.87,
  is the gain pivot. The inverted phase field spans about one cycle.
- **Yellow ground = a key.** The flat yellow is noise-free and covers 19 % of the frame. Horizontally,
  yellow↔blue edges are straight YUV blends (row 300, x 1420–1480: Y 96 → 209, Cb +76 → −118), giving a
  whitish/cream fringe. Vertically the same colours sweep the palette. So the ground is a keyed fill over a
  smooth field, with the smear applied after the colourizer and chroma wider than luma.
- **Streaks.** Per-line colour streaks fill the columns between the figures and the top/bottom bands, and
  fray the key edge. In busy blocks the phase residual is 0.16 cycle rms. Along the line it correlates
  0.64 / 0.45 / 0.23 / 0.12 at 4 / 8 / 16 / 24 px. Vertically it correlates +0.19 / −0.27 / +0.11 at 1 / 2 / 4
  rows: an interlace comb of period about 4.3 rows (spectral peak at 0.233 cycle/line).
- **Slips.** Short horizontal slices of displaced content, for example rows 544–559 and 672–684.
- **Edges.** Horizontal transitions take 40–100 px, vertical ones 4–6 rows. Region-mean colours
  (4 × 4 grid) are used as the check.

## What it does
1. **Store.** A 96 × 128 cell store of the photo's top half covers virtual rows −40 .. 520; the bottom of the
   picture folds back past the top.
   - **Osc:** a ROM of 4-bit codes, fitted by gradient descent through the full forward model and quantised
     to 15 Lloyd phase levels plus code 15 = the yellow key. It is held as 4 banks (row parity × column
     parity) × 3 blocks of 1024 × 4.
   - **Video:** the camera luma, 64 × 128 × 4 bits over the full height, written live. The luma is
     IIR-smoothed and sampled once per column on the first line of each store row.
2. **Readout.** Each line, the line engine folds the virtual row about the mirror line, adds any slip,
   reflects it into the store's range and multiplies by the row scale. Each pixel reads the four corner
   cells and interpolates them bilinearly, wrap-aware: 8-bit modular differences, and a key corner takes a
   neighbour's phase. The nearest corner decides the key.
3. **Streaks.** Value noise with knots every 16 virtual px, redrawn every 2 virtual rows. Half of each knot is
   shared with the line pair at opposite sign, which gives the comb. The amplitude comes from a 48 × 32 busy
   map (Osc) or from the local camera gradient (Video), times K1.
4. **Colourizer.** The phase plus noise (2048 = one cycle), optionally negated, then the key test
   (|phase| < window, or the stored key). Then the gain about the pivot (Osc) or from black (Video), plus the
   hue, into a 256-entry palette. Its luma is pre-boosted 16/13 with a knee, so the output luma scale lands
   the photo's levels; the yellow tops out at about 764.
5. **Smear.** Box filters from 256-entry rings: chroma N, luma N/4 (powers of two). The luma is centred by
   the DDA start; the chroma trails like an analog chroma delay.
6. **Output.** The shell's conditioning: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, blanking
   gate. Latency 30 clocks in every mode (28 in v0.5).

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Feather | the feathery per-line streaks and the slips | streaks 0 .. 5× and slips 0 .. 16× as often (the photo at 40 %); above the photo, streaks spread over the whole idol and slices slip on about half the lines |
| K2 Hue | the colour bands | colourizer phase, 0 .. 1 cycle (0 = the photo); the yellow ground stays yellow |
| K3 Axis | the mirror line | 13 .. 91 % of the height (48 % = the photo), the idol riding on it: below the photo the fold shrinks into stacked idols (4 at 0 %), above it the top half stretches down to the line (v0.6.1) |
| K4 Speed | the bands' colours, widths and rows | bipolar colour cycling, still at the centre (±8); default +17 % ≈ 14 s per cycle. Past the default either way the bands swell in sequence and the rows ripple sideways, deeper with speed |
| K5 Smear | the sideways smear and whitish fringes | chroma box 1 .. 128 px in power-of-two steps (64 = the photo), luma box a quarter of it |
| K6 Ground | the flat yellow ground | key window: 0 = no ground, 30 % = the photo, 100 % = 89 % of the cycle flat (only a blue band left) |
| S7 Mirror | the fold | Fold (the photo) / Off (Osc: the top half repeats; Video: the plain camera) |
| S8 Twin | the idol's symmetry | adds a left/right mirror: a symmetric totem |
| S9 Palette | the colourizer family | Photo (yellow ground) / Rainbow (the hue once round the wheel, black ground) |
| S10 Negative | the band order | colourizer input inverted (blue rings ↔ red rings; Video: bright → yellow ground) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Bands** | the contour bands | colourizer gain 0.25× (near-flat colour) → 1× at 50 % (the photo) → 4× at 100 % (dense rings), exponential |

K1, K2, K3, K6 and P12 glide (1/8 of the gap per field, ±4 hold). K5 is latched per field. Every control
changes the picture at any P12 position (`sheet.png`).

## Video
The photo is camera content, so Video puts the camera through exactly the photo's processing:
- the camera luma is stored at 64 × 128;
- it is folded about the mirror line and read back bilinearly;
- black maps to the yellow key and brighter levels run once through the palette at the default gain;
- streaks are drawn where the camera has edges and texture;
- the same smear and slips apply.

Osc substitutes the stored field fitted to the photo. Every control works in both modes. The photo's yellow
ground appears in Video only where the camera has black areas; a bright scene needs K6 raised or S10 on.

## Engine
- **Vblank zone** (no avid for 1024 clocks), one small operation per zone clock:
  - switches and the speed latch;
  - five glides on one shared datapath;
  - two serial divisions, 2^22 / W and 276480 / lines. The DDA steps come from the measured raster, so
    every mode and decimated sims render the same picture;
  - derived terms: axis, gain, key window, hue and cycle;
  - one serial shift-add multiplier for the four streak amplitudes and the luma-centring offset;
  - the smear-integrator reset (zeroed, then N clocks with the leaving sample forced to 0).
- **Line engine:** 32 clocks after each source avid fall it computes the next line's mirror, slip, fold and
  store row (an 11-step serial multiply), the busy row, the streak seeds (the two first knots are pre-drawn
  on the beam's own LFSRs) and the camera-store write row.
- **Pixel path, 30 stages** (v0.6.0: + swell sine and product after the palette index):
  - beam DDAs (column, reversing for Twin; knots);
  - clamp and bank addresses;
  - ROM / RAM reads, then the block select into the fabric register;
  - corner permutation and level LUT;
  - key substitution and wrap-aware differences;
  - x products, rows, y product, phase;
  - streaks and Negative, key test, gain product, hue;
  - palette and key fill;
  - box sums, normalise;
  - conditioning and gate.
- **Camera writer:** luma convert (W1), scale (W2), IIR and column events (W3). Collision-free with the
  reads: writes land half a cell behind the read corners, and the RAMs carry `no_rw_check`.
- **Build hook:** `idol.py` (pure Python) carries the scene codes, busy map, levels, palette and yellow as
  data and emits `idol_pkg.vhd`.
- **EBR (27, v0.6.0):** scene 12, camera store 8, palette 3 (two families), smear rings 2, chroma filter ring 1,
  busy map 1.
- **Contracts:** generated avid; one 30-clock latency in every mode; output conditioning before the
  blanking gate; colours in the hardware convention (U = Cr, V = Cb).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 79.31, SD Analog s1 81.03, HD HDMI s1 77.71, SD HDMI s1 76.79, HD Dual s3 79.34, SD Dual s1 78.27 MHz; 6823-6851 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-27, router2, not flashed.**
  - Build seeds: **HD Dual = seed 3** (seeds 1–2 missed); the other five are seed 1.
  - Post-route Fmax: HD Analog 77.59, HD HDMI 83.07, **HD Dual 74.48 (thin)**, SD Analog 72.93,
    SD HDMI 70.91, SD Dual 82.29 MHz (SD floor 27).
  - Fit: 6813–6855 / 7680 LC (89 %), 25/32 EBR. All six bitstreams pass iceunpack.
  - If HD Dual regresses, rebuild with `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh idol` first.
- **Fit history:** the first build overflowed at 7746 LC. It was slimmed with power-of-two smear boxes and a
  fill reset, a serial multiplier for the per-field terms, one shared fold, line-start knots on the beam's
  LFSRs, a 16-bit slip RNG, and the ROM block mux merged into the fabric register.
- **Timing:** the critical path was the camera writer (luma convert + clamp + IIR in one clock, 14 ns),
  now three stages.
- **Verified in GHDL (960 × 540, hardware hues):** Osc and Video both match the integer model.
  - Region means are within about 10 of the photo on a 4 × 4 grid.
  - Motion: at K4 100 %, 41 % of pixels change over 3 fields while the yellow ground holds (21.6 → 21.0 %).

## Deviations from the photo
- **Store resolution.** The store is 96 × 128 cells at 4 bits (20 px × 4.4 rows), the most the EBR budget
  allows next to the camera store. The faces' fine features (eyes, mouth) and the thinnest dark outlines
  are softer than in the photo, and the centre reads as a red/orange diamond rather than two legible faces.
- **Procedural textures.** The streaks and slips are a procedural realisation, matched in statistics
  (vertical and horizontal contrast) but not in placement detail.
- **Chroma delay.** Chroma lags luma by about (N_c − N_y)/2 + 4 px (≈ 28 px at 1080p default): the luma
  is centred and the wider chroma box trails.
- **Saturation and highlights.** Blues are a little less saturated (blue share 4 % against 9 %). The yellow
  ground is capped at about Y 764 by the HD HDMI bar fix's 13/16 luma scale.
- **Resolution.** The picture spans the full width in every mode, so shapes are 25 % narrower on 4:3 SD.
  The smear drops one power-of-two step at 720p and SD, and the streak knots scale with the width.

## Phase 2 sign-off (2026-11-03)

Ron flashed v0.6.0 (note: K3 Axis did too little), then approved the v0.6.1 diag as a whole. That leaves the
2026-11-02 open questions settled as built: swell and ripple stay on K4 past the default speed, and Rainbow on
black stays the second palette. Merged into production with `./heirloom_diag.sh merge idol` (id and name
restored; the diag copy stays in `.heirloom_work/rb_idol/diag/idol_diag/`).

**Remaining:** presets are still hand-written; capture real ones from the hardware with `./capture_preset.sh`.
