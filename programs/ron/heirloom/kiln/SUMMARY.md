# Kiln — camera fired in a frame-store kiln

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from the photo WIN_20230515_23_17_11: camera video in a
frame store zoomed to a quadrant, repeated across the frame (1.94 copies) with the bottom row of quadrants
mirrored, run through a fire colouriser (royal blue, violet, purple, magenta, red, orange, yellow) with a
brightness lift, and torn into horizontal fibres with an interlace comb.  Presets are hand-written placeholders.

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
| K1 Tiles | the horizontal repeat: 1–4 copies across (1.94 = the photo) |
| K2 Fibres | the streaky fibre texture and venetian blinds: 0–2× in 16 steps (1× = the photo) |
| K3 Blocks | the yellow blocks and blue patches: colouriser contrast 0.5–2× about the red mids; low melts the picture into reds, high posterises it into yellow and blue |
| K4 Drift | the frame-store scroll: the copies slide sideways; bipolar, still in the centre ± 8, square law, up to ~2 cells per field |
| K5 Rims | the pink rims and ring edges: the brightness lift 0–2× in 16 steps |
| K6 Glaze | the fire palette's colourway: Fire (the photo) / Ice / Ember / Toxic / Violet / Mono / Neon / Rust |
| S7 Mirror | the top/bottom symmetry: Mirror (the photo) / Repeat |
| S8 Kaleido | the partial left/right symmetry: Off / On (every other copy flipped) |
| S9 Weave | the interlace comb: Comb (the photo) / Smear (no alternation, 4× longer smeared fibres) |
| S10 Quench | the fire itself: Off / On (the colouriser scalar inverted: yellow ground, blue and red seams) |
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
