# Vinyl — L-shaped bands with sawtooth red fringes and record-groove moire

**v0.1.0 built 2026-11-04, not flashed.** Heirloom phase 3 (video clips), from the clip
`WIN_20230428_23_30_39` at 0.5 s (`photos/vid/WIN_20230428_23_30_39_t0p5.png`). The still shows nested
L-shaped bands, near-vertical at the top-left and running out nearly flat to the right. Green and blue
plateaus alternate with a thin red fringe at every crossing. A fine hatch cuts sawtooth teeth into the blue
side of each red fringe and twists the blue bands into rope. In the wide blue band at the bottom, fine curved
grooves run like the grooves of a record. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_vinyl/` (`notes.md`, `fit*.py`, `proto2.py`, `imodel.py`).
- **Main field.** Every band edge is a contour of one field, fitted to 2328 band-centre points (component
  tracking of the green and blue plateaus) with an rms of 0.046 cycles (0.074 near the left edge):
  `F_A = 4.332 * 2^(-y/257.2) * softplus(x - 199.1 + 0.379 y; 149.4)^0.191 - 10.152 * 2^(-y/304.7) + 0.199`.
  - It is a log-like horizontal shaper times an exponential vertical ramp, minus a second exponential.
  - Simpler forms fitted worse: exponential FM linear in x 0.24 cycles, a sqrt shaper 0.090.
  - Row periods (FFT along the rows) run from 303 px at y = 10 to 819 px at y = 370, about 250 lines per
    octave.
  - Along the top row the green bands start at x = 0, 124, 268, 420, 590, 810, 1032, 1372 and 1816.
- **Colouriser.** Down a column the order is G, R, B, R, G: a triangle colouriser, with green at the peak,
  blue at the trough and red at both crossings. Binned by fitted phase u:
  - green plateau u 0.84 .. 0.13 (G 35-64, p90 50-92);
  - red fringes u 0.19 .. 0.28 and 0.66 .. 0.78 (R 100-110 median, p90 170);
  - blue plateau u 0.34 .. 0.62 (B 90-180, p90 140-250), peaked in the middle;
  - dark gaps between.
- **Envelope (p95 per 120 px block).** Blue is 255 (clipped) at the top and 60-150 in the lower half. Green
  is 130 at the top-left, 50 at the top-right and 20-40 at the bottom. Red is 130-190 everywhere and 240 in
  the bottom-left block.
- **Teeth and hatch.**
  - The teeth are 55-65 px apart along the fringe at the centre and the red "lenses" 75-85 px apart at the
    top-right.
  - They sit only on the blue side of each red fringe; the red-to-green edge is smooth.
  - The hatch streaks rise to the right ("/"), with a perpendicular period of 30-44 px.
- **Grooves.** Along the rows the spacing is 35 px at y = 760, 37 at 820, 40 at 880, 43 at 940, 50 at 1000
  and 55 at 1050. The grooves lean down-right: steep at the bottom-left, nearly flat at the bottom-right.
- **Left strip.** Green for y < 505 (fading 130 -> 30), dim red at 520-600, red at 620-760 and bright red
  from 780 down. Each step is a hard horizontal edge, and the red block widens to x ~130 at the bottom.

## Motion in the clip
Measured from the m-frames (0.5 s apart) and one frame per second over the whole 12.5 s clip
(`seq_sheet.png`).
- **Upper bands:** static to within ±4 px over 2 s (row periods within 1 %, row phases ±0.1 cycle).
- **Teeth:** they crawl and jitter along the fringes by up to half a tooth (±25 px per 0.5 s), with no
  steady direction.
- **Grooves:** at the bottom-left they rise about 30 px per 0.5 s and re-shape. The crop correlation falls
  from 0.70 to 0.29 over 2 s, because Ron was turning the groove oscillator.
- **Over the whole clip:** from t = 3 s the groove family coarsens and swings. At t = 6-7 s it crosses the
  main bands as a coarse second family, a rhombic lattice of sawtooth-shaded cells (`seq/f6.0.png`,
  `f7.0.png`). It is back to the still's look by t = 12 s.
- **What the program does with them:**
  - At the default Speed (K4 = 640) the grooves run 1.5 cycles/s and the teeth crawl 0.4 cycles/s
    (about 25 px/s). The bands drift 0.016 cycles/s, static to the eye.
  - In GHDL frames 0.1 s apart the bands moved 1 px and the grooves about 4 px (about 45 px/s at 1080
    scale), rising and running right.
  - P12 Moire is the knob Ron turned: it sweeps the groove family from fine grooves at the bottom to the
    coarse crossing lattice.

## What it does
1. **Bands.** Per line, the amplitudes `A·G·2^(-f y/257)` and `C·G·2^(-f y/305)` come from multiplicative
   decays: exact exponentials with no per-line exp2. Per pixel, `R(t) = softplus^0.191` is read from a ROM
   by an index DDA (t = x - xa + 0.379 y). Then `F_A = A R - C + c0 + flow + etch * luma`, a 12-bit phase.
2. **Strip.** Left of `x = 52 + max(0, y - 760)/4`, the phase is a stepped per-line value: green, dim red,
   red, bright red.
3. **Grooves.** `F_G = gG(y) x + flowG` with `gG = gG0 2^(-y/LG)`, one accumulator per pixel. Its
   sawtooth, times an amplitude that fades in from line 600, is added to the phase.
4. **Hatch.** `F_H = x/62 + y/75 + flowH`. Its sawtooth is added with a weight that is non-zero only on the
   blue half of the colouriser, so the teeth cut only the blue side of each red fringe. It fades out below
   line 594..850.
5. **Colour.** A 256-entry palette over the phase (the still's colours binned by fitted phase), then a
   brightness envelope: blue brighter at the top, green brighter at the top-left. The envelope is
   `e = a + b` in dithered 1/8 steps.
6. **Output.** Chroma `[1 2 2 2 2 2 2 2 1]/16`, luma `[1 2 1]`, luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | the bands' density: x0.33 .. x3 (512 = the photo) |
| K2 Corner | where the bands turn: the vertical asymptote of the L slides -517 .. +915 virtual px (512 = the photo). Right of centre a second, mirrored family opens on the left |
| K3 Fall | how fast the bands open going down: vertical rate x0.44 .. x2.3 (512 = the photo); low = long near-vertical bands, high = everything crushed to the top |
| K4 Speed | bipolar (±8 deadband): the grooves run (the record turns), the teeth crawl, the bands drift; 640 = the clip's motion, full = grooves 6 cycles/s, bands 0.25 cycles/s |
| K5 Teeth | the hatch on the red fringes: 0 = clean fringes .. 380 = the photo (0.11 cycle) .. 1023 = deep saw teeth (0.30) |
| K6 Etch | how far the camera (or the Osc lattice) bends the bands: 0 .. 2 cycles per full luma (200 = 0.16) |
| S7 Wave | Tri (G R B R, the photo) / Saw (G R B, then a hard reset: sawtooth-shaded bands) |
| S8 Mirror | Off / On: the bands and the strip fold about the screen centre (two corners, a valley) |
| S9 Palette | Primaries (the photo) / Sepia (the same session's other look: pale green, olive brown, teal fringe, mauve gaps) |
| S10 Grooves | Low (the photo: the grooves fade in at the bottom) / High (the groove family flipped: dense at the bottom, fading in from the top, so the grooves cross the upper bands) |
| S11 Source | Video / Osc |
| P12 Moire | the groove oscillator. 0 % = no grooves (clean bands). 30 % (307) = the photo's fine grooves at the bottom. 100 % = a coarse second family (period 300 px, a flatter vertical chirp, amplitude 0.45) crossing every band in a lattice: the clip's t = 6-7 s look |

## Video
- **Video mode (the default).** Camera luma, as `(Y >> 2) - 16` from a one-line buffer, is added to the
  band phase times Etch. The bands bend and fray around the picture, while the grooves and the hatch run on
  over it. At the default Etch (0.16 cycles per full luma) the L-shaped composition reads clearly
  (`rtl_video3.png`, with the selfie). With no input, Video mode is the photo.
- **Osc mode.** A 640 x 540 virtual-px triangle lattice, drifting, feeds the same path. At the default it
  gives a gentle wobble of the lower bands, and Etch scales it.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier: 27 slots, decoded two clocks ahead, with the operands pre-muxed.
  - A shared glide for K1, K2, K3, K5, K6 and P12.
  - One shared knob-deviation subtractor.
  - A 256-entry exp2 ROM.
  - Per field it computes: the Rings gain (A·G, C·G), the Fall rate and the per-line decay constants kA and
    kC, and the groove chirp. The chirp is log2 LG from P12, then 2^-LG, kG, 900/LG and 1079/LG, log2 of
    the period at line 900, and gG0.
  - It also computes the groove and hatch amplitudes and the groove fade start, the etch gain, the three
    flows (K4, square law for the bands), the R-index start for Corner, and the per-line steps.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in about 106 clocks, so
  it fits SD's 858-clock line.
  - A <- A - A kA, C <- C - C kC and gG <- gG -/+ gG kG, on a 20-iteration serial multiplier.
  - The groove and hatch amplitude ramps.
  - The strip threshold and phase.
- **Pixel path (C_LAT 24).**
  - Beam (R index with Mirror reversal, virtual x, groove and hatch phases, green envelope).
  - R-ROM address, ROM, fabric copy.
  - a·R as two 14 x 7 products, then the sum to a 12-bit phase.
  - + offset + etch (one 8 x 8 product).
  - Strip mux and triangle; hatch weight.
  - Hatch and groove products (sign and magnitude, 5 x 6 and 6 x 4).
  - u, palette address (Tri = u/16, Saw = u/32), palette ROM, fabric copy.
  - Envelope (one adder per channel), compose.
  - Pad; chroma box sums, luma [1 2 1], 13/16, gate.
- **Memories (18 EBR).** R table 2048 x 16 (8), palettes 512 x 32 (4), exp2 256 x 16 (1), line buffer
  2048 x 8 (4), and a chroma 8-sum ring plus a sync ring (block RAM, `no_rw_check`).
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 24, in every mode.
  - The blanking gate.
  - The HD HDMI output conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates.
  - Colours in the hardware convention (U = Cr).
- **Verified (GHDL, 720p60, decimation 1).**
  - The Osc frame matches `imodel.py` (with the Osc lattice modelled) at a mean |dRGB| of 2.8, with 1.3 %
    of pixels above 20 (a few-pixel texture phase).
  - The integer model matches the float proto within 0.01 cycle of band phase on every line.
  - `cmp_final.png` = still | proto_final | RTL Osc | RTL Video.

## Status
- **v0.1.0 built 2026-11-04 with router2, all six configs on seed 1.**
  - Post-route Fmax: HD Analog 78.29, HD HDMI 77.86, HD Dual 83.59, SD Analog 70.65, SD HDMI 81.43,
    SD Dual 82.45 MHz (SD Fmin 27).
  - 7140-7177 / 7680 LC (93.0-93.5 %), 18 / 32 EBR. Every bitstream passed iceunpack.
  - Output: `out/rev_b/ron/heirloom/vinyl.vmprog`. **Not flashed.**
- To rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh vinyl` (router2).
- **Fit history.**
  - The first cut was 9262 LC (120 %). Standalone-yosys ablation found these costs: envelope multipliers
    665 cells, groove and hatch products 508, a·R 367, Mirror 342, etch 156.
  - The cuts: the envelope became one adder per channel, the textures became sign-and-magnitude narrow
    products read from the beam, Mirror now folds only the bands, and the output stage and sync ring moved
    to the fern-style block RAM.
  - The GHDL render then caught a sequencer bug that zeroed the teeth and grooves at their default
    settings. It was fixed before this build.
- **Watch on hardware:**
  - No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.
  - The red/blue pixel-rate fringes are the kind of edge that bars on HD HDMI. The chroma box is the
    standard fix. If bars appear, widen the chroma filter first.

## Deviations from the photo
- **Green bands.** They are clean. The photo's green bands carry faint teal hatch streaks; the hatch acts
  only on the blue half, which is where the teeth are.
- **Bottom-left.** The photo is blue with dark grooves beside the red block. The program has the red-fringe
  region there, with red grooves. The red block is approximated by the widening sync strip.
- **Hatch.** It is one straight family (62 x 75 px). In the photo the tooth spacing varies from about
  50 px (top-left) to 85 px (top-right).
- **Grooves.** They are an exponential family that is linear in x. In the photo the spacing is not
  monotonic (35 -> 55 px over the bottom third, with alternate grooves weaker in places). In the program
  they drift up and to the right; the clip's drifted up and to the left while the knob was also re-shaping
  them.
- **Envelope.** Seven levels with a 2 x 2 dither, instead of a continuous envelope. Blue has no left-right
  falloff (green does).
- **Blur.** The photo's capture blur is replaced by the output filter, so the luma edges are crisper than
  the photo's.
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the left three quarters, virtual x 0..1439. The view is left-anchored because the corner and
    the strip are at the left; the right quarter of near-flat bands is cut on 4:3.
  - Interlaced inputs draw both fields alike.
- **Budget.** 93 % LC, slightly over the ~92 % target. It routed and closed with 3.6-9 MHz of HD margin.
