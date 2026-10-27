# Idol — a camera idol folded in a frame-store mirror, colourized on flat yellow (Heirloom #31)

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source: `photos/WIN_20230515_23_14_37_Pro.jpg`.
v0.1.0 (a relief-shaded column of the picture) was judged too abstract. This version measures the photo, fits
a store of its field and reproduces it through the measured colourizer, mirror, key, smear and streaks.
Working files: `.heirloom_work/rb_idol/`: `notes.md`, `protof.py` (float proto), `gfit.py` / `quant.py`
(store fit), `imodel.py` (integer model), `sheet.png` (control sheet), `cmp_final.png`.

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
   gate. Latency 28 clocks in every mode.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Feather | the feathery per-line streaks | streak amount 0 .. 2.5× (the photo at 40 %); above the photo, streaks spread over the whole idol |
| K2 Hue | the colour bands | colourizer phase, 0 .. 1 cycle (0 = the photo); the yellow ground stays yellow |
| K3 Axis | the mirror line | 30 .. 70 % of the height (48 % = the photo); out-of-range rows reflect again |
| K4 Speed | the bands' colours | bipolar colour cycling, still at the centre (±8); default +17 % ≈ 14 s per cycle |
| K5 Smear | the sideways smear and whitish fringes | chroma box 1 .. 128 px in power-of-two steps (64 = the photo), luma box a quarter of it |
| K6 Ground | the flat yellow ground | key window: 0 = no ground, 30 % = the photo, 100 % = yellow floods the light bands |
| S7 Mirror | the fold | Fold (the photo) / Off (Osc: the top half repeats; Video: the plain camera) |
| S8 Twin | the idol's symmetry | adds a left/right mirror: a symmetric totem |
| S9 Slips | the displaced slices | frame-store slips on (the photo) / off |
| S10 Negative | the band order | colourizer input inverted (blue rings ↔ red rings; Video: bright → yellow ground) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Bands** | the contour bands | colourizer gain 0.5× (calm, few bands) → 1× at 50 % (the photo) → 4× at 100 % (dense rings) |

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
- **Pixel path, 28 stages:**
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
- **EBR (25):** scene 12, camera store 8, palette 2, smear rings 2, busy map 1.
- **Contracts:** generated avid; one 28-clock latency in every mode; output conditioning before the
  blanking gate; colours in the hardware convention (U = Cr, V = Cb).

## Status
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
