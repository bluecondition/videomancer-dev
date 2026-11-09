# Coastline — a rainbow ridge on a curved coastline, its bands waving harder away from the shore

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20240117_22_55_32` at 0.5 s
(`photos/vid/WIN_20240117_22_55_32_t0p5.png`). A blue field cut by a clean curved coastline. On the coast sits a
rainbow ridge: a cyan fringe, lime, yellow with a thin white streak, salmon, orange, grey-pink, orange-red, olive,
red, then black. Its bands wave more and more the further they lie from the coast. Beyond it lie a gulf of blue, a
second clean arc, and deep blue offshore rising into soft cyan with the same waves. In the clip the waves travel
down, one every 2.6 s, and the coast and the arc never move. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_coastline/`: `notes.md`, `edges.py`, `waves.py`, `ufit.py`, `rfit2.py`,
`palbin*.py`, `register.py`, `tmpl.py`, `motion.py`, `proto.py`, `imodel.py`, `isheet.py`, `thincheck.py`.
- **Palette** (k = 16, whole still):
  - azure (0,88,255) 16.8 %, deep blue (0,7,253) 10.6 %, blue (0,64,255) 10.4 %, cyan-blues (0,93..148,255) ~28 %;
  - yellow (237,233,24) 6.2 %, red (223,21,0) 6.0 %, dark olive (51,29,2) 4.8 %, orange (231,130,8) 3.2 %;
  - lime (126,239,1) 2.3 %, navy (1,27,164) 2.0 %.
  - The camera clipped it: B = 255 over every blue, R = 255 over the warm bands.
- **Two clean curves.** The coast (the ridge's inner edge, 1068 rows) is fitted about y = 540 by
  x = 618.59 − 0.4960 dy − 4.102e-4 dy² (rms 2.3 px). The arc is x = 1445.09 − 0.6468 dy − 5.135e-4 dy²
  (rms 3.6 px). The outer curve bends more, so the curves are not concentric circles. Both are reset edges: G drops
  78 → 6 across the arc in under 20 px, and neither curve waves (0.6 / 1.0 px rms wiggle).
- **The field.** u = (x − XI(y)) / (XA(y) − XI(y)): a per-line affine ramp, 0 on the coast and 1 on the arc.
- **The wave is a VCA.**
  - Six contours inside the ridge and the gulf share one per-row scale factor, to within ~3 %. So the colouriser
    sees w = frac(u) · m(y).
  - Fit: m = 1.015 + 0.176 sin(2π y/380 + 0.76) + 0.062 sin(2π y/820 + 2.38), rms 0.018 (one sine gives 0.044).
  - The multiplication grows the waviness outward from each reset and leaves the resets themselves clean. That is
    exactly the still's look: about 32 px of wave at the yellow/orange edge, 66 px at the red/black edge, 0 on the
    arc.
- **Per-cycle palettes.** Binned by the wrapped, modulated field, every zone is near-pure:
  - offshore by (u − 1)·m: sd 1.3 (by u: 5.0);
  - bay by (u + 1)·m: sd 0.74.
  - So the sawtooth resets at every integer of u, and the palette depends on the cycle:
    - **bay** (u < 0): G 127 → 88.
    - **ridge** (0..1), in w: cyan 0–0.012, lime 0.02–0.07, yellow 0.08–0.12, white 0.13, salmon 0.19, orange 0.22,
      grey-pink 0.255, orange-red 0.30, olive 0.35, red 0.40–0.43, fading to black 0.50–0.52, blue from 0.53, then
      G 30 → 80 out to 1.24.
    - **offshore** (1..2): deep blue to v = 0.22, cyan rising to G 150 by 0.55, down to 99 by 1.1.
    - **far** (u ≥ 2, the bottom-right corner only): G 112 → 97.
  - Registered against the raw rows, the thin lines are sharper than their bins: white (240,245,190), salmon
    (246,205,115), grey-pink (183,125,72), olive (93,58,16). They are restored with an unsharp mask in w.
- **Texture.** Soft edges (10–15 px), with no noise, tearing or comb. A dark-green notch at y 180–300 widens the
  black after the red at the deepest trough.

## Motion in the clip
Measured by `motion.py` (per-row wave fit on m0 / still / m1 / m2 / m3):
- **Coast and arc.** Both are static: their positions agree within 2 px over 2 s.
- **Wave 1** (380 lines). Its phase reads +109.5, +43.5, −23.9, −95.4 and −168.2° at t −0.5 .. +1.5 s. That is
  −69.6° per 0.5 s = 0.387 Hz, one cycle every 2.6 s: the waves travel DOWN at ~147 lines/s.
  - Cross-correlating the whole m(y) gives 75–98 lines per 0.5 s, the slow wave included.
  - The slow wave's phase fit is unstable (a2 0.06–0.15).
- **No knob is turned in this clip.**
  - 22_56_18 is the same patch with slightly deeper waves.
  - 22_57_37 has a horizontal oscillator patched in: zigzag stripes in the bay, broken cells in the ridge, chevrons
    offshore, while the coast and the arc stay clean.
- **The program:**
  - At the default Speed (700) wave 1 steps 0.00627 cycle per field (0.38 Hz at 60 Hz). Wave 2 steps at half that
    cycle rate (~160 lines/s).
  - The integer model at +0.5 / +1.0 / +1.5 s shows wave-1 phase steps of −69.8° per 0.5 s, against the clip's
    −69.6°, and 70–78 lines per 0.5 s (`cmp_motion.png`, model next to the m-frames).

## What it does
1. **Ramp.** Per line, XI = coast and WD = arc − coast (quadratics in dy; Bend scales the curvature). The field gain
   G = Reach / WD comes from a divider, and u = (x − XI)·G runs along the line as a DDA. Mirror turns the DDA back at
   the centre.
2. **Camera / Osc.** u += Etch · luma (a 4-px box of a one-line buffer); the Osc is centred on its mean.
3. **Sawtooth.** k = floor(u), f = frac(u).
4. **Wave.** m = 1.015 + D·s1 + 0.352·D·s2, two sines per line (1024-step quarter-wave table), D from Surf.
   Ripple adds 0.1875·s1 to u, so the resets snake too.
5. **Colouriser input.** w = |f·|m| ± Weave·sin(2π x/240 + 1.6 s1)|.
6. **Colour.** palette[zone(k)][w]: ridge 256 entries per unit of w, offshore 128, bay and far 64. Rainbows uses the
   ridge entries for every cycle; Sunset swaps U and V.
7. **Output.** Chroma goes through two [1 2 2 2 2 2 2 2 1]/16 boxes (with the running-sum resync) and luma through
   [1 2 1]/4 on the same centre. Luma is scaled 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bend | the coastline's curvature (the arc's follows it): bowed the other way (0, b = −1.5) .. straight diagonal (~205) .. the still (512) .. a hooked "<" bay (1023, b = 4) |
| K2 Reach | the sawtooth's period, i.e. the ridge's width and the arc's distance: ×0.25 (0: the ridge fills the frame, huge waves) .. the still (512) .. ×4 (1023: a thin ridge, the arc close behind, faint repeat arcs in the bay) |
| K3 Waves | the wave count: 0.35 .. 22.7 waves per frame height (±3 octaves; 2.84 = the still, at 512) |
| K4 Speed | bipolar (±8 deadband, square law): the waves travel down (+) or up (−); 700 = the clip's 0.38 Hz (2.6 s per wave); full = 2.9 Hz |
| K5 Weave | a horizontal chevron oscillator (240 px, bent by the wave) added to the colouriser: 0 = the still; turned up = zigzag stripes in the bay and offshore, the ridge broken into zigzag bands (the sibling clip 22_57_37) |
| K6 Etch | how far the camera (or the Osc) bends the coast, the arc and every band: 0 .. 1 cycle per full luma (square law); 200 = 33 px for full white |
| S7 Coast | Clean (the still: the resets never wave) / Ripple: wave 1 also moves the ramp, so the coast and the arc snake ±160 px (a meandering rainbow river; independent of Surf) |
| S8 Rainbows | Ridge (the still) / All: every sawtooth cycle gets the rainbow, giving parallel rainbow coastlines |
| S9 Mirror | Off / On: mirrored about the centre, two coasts meeting in a rainbow canyon |
| S10 Sky | Day (the still) / Sunset: U and V swapped, an amber sea with a cyan / blue-violet ridge |
| S11 Source | Video / Osc |
| P12 Surf | the wave depth: 0 = a calm ridge of clean bands (a 0.04 swell stays, so Speed and Waves still show) .. 464 = the still (0.175) .. 100 % = storm surf (0.70): rainbow tongues reaching out to the arc, folding back at the tips |

Glides (1/8 of the gap per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is box-filtered over 4 px and
  added to the ramp before the sawtooth.
  - The coast, the arc and every band bend around the picture.
  - At the default Etch (200) the ridge stays readable on selfie.jpg; the camera bends it rather than shredding it
    (`cmp_final.png`, bottom). The first default, 320, was ragged.
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 320 × 270 virtual-px triangle lattice feeds the same path, centred on its mean, at 6× the
  camera's gain. At the default Etch the clean curved coast becomes a jagged zigzag coast, and the arc steps with it.

## Engine
- **Per-field sequencer** (vblank zone).
  - One serial shift-add multiplier (15 slots), which is the line engine's multiplier, idle in the zone.
  - The slot number is decoded two clocks ahead and the operands are pre-muxed (22 bits).
  - The shared glide.
  - A 256 × 16 exp2 table for Reach (±2 octaves) and Waves (±3 octaves).
  - Per field it computes: the bend curvature terms; Reach; wave 1's frequency per line and wave 2's (× 0.4632);
    Speed² and the phase steps; Surf² and the depths D1, D2; Weave²; Etch² and the gain (× 6 for Osc), and the Osc
    centring.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the NEXT line in
  162 clocks, so it fits SD's 858-clock line: dy²; XI and WD by Horner (5 products); G = Reach · 2^20 / WD by an
  18-step restoring divider; the ramp start (xb0 − XI)·G − Osc offset, and the step G·xs; the two wave sines, then
  m = 1039 + D1 s1 + D2 s2 (Q10), |m| and its sign; the Ripple offset; the weave's line phase.
- **Beam.** The u DDA (Q24 cycles) and the weave-phase DDA, both reversing past x = 1920 h for Mirror. The turn flag
  is registered a clock ahead.
- **Pixel path (25 stages, C_LAT 24).** 1 beam; 2–6 line-buffer read, then the 4-px luma box; 7–8 etch nibble
  products; 9 u' = u + etch; 7–9 the weave sine read (from the delayed phase); 10 f × |m| and |H| × Weave partials,
  and the zone; 11–12 sums, then w = f|m| ± weave; 13 |w|; 14–16 palette address, carried two stages on 10 bits;
  17 ROM data; 18 fabric copy; 19 compose (Sunset swap; black off the generated avid); 20–23 chroma boxes and luma
  [1 2 1]; 24 luma 13/16; 25 gate.
- **Stores** (15 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | palette | 1024 × 32 (Y10 Cr10 Cb10) | 8 |
  | line buffer | 2048 × 8 | 4 |
  | sine (line engine, weave) | 2 × 256 × 10 | 2 |
  | exp2 | 256 × 16 | 1 |

- **Contracts:** damier's generated avid; one latency, C_LAT 24, in every mode; virtual 1080-line coordinates (SD
  shows the centre 4:3); colours in the hardware convention (U = Cr, V = Cb; `coastline.py` writes the palette);
  luma pre-scaled 16/13 and capped at 940; the chroma running sums snap back to neutral after 8 blank samples.
- **Verified** (GHDL at 720p60, decimation 1, against `imodel.py`):
  - **Default frame, black input: bit-exact (100 % of pixels).** It is also bit-exact one field later, which
    confirms the per-field phase step; 48 % of the pixels change between those two fields.
  - Ripple test (Bend 700, Waves 600, Speed 900, Weave 300, Surf 800, Ripple): bit-exact. This was run on the
    revision before the final LC refactor, which is logically identical.
  - testA (Bend 900, Reach 300, Waves 700, Speed 300, Weave 600, Surf 700, Mirror, Sunset): bit-exact, on an earlier
    revision whose datapath was the same apart from luma.
  - Osc default: 99.96 %. testB (Bend 100, Reach 900, Rainbows, Osc, Etch 700): 99.87 % (row 0 only).
  - Integer model against the still at 1080p: mean abs RGB 5.27 (float proto 4.63). All 16 cells of the 4 × 4
    region-mean grid agree within 1–11 levels.
  - `thincheck.py`: no single-row line ≥ 24 px at the defaults, at P12 0 / 100 (720p and 1080p), or with Ripple, Osc,
    Mirror, Rainbows or Weave max, each at Surf 100.

## Status
- **v0.1.0 built 2026-11-09 (router2), not flashed.** `out/rev_b/ron/heirloom/coastline.vmprog`.

| Config | Seed | Post-route Fmax | LC | EBR |
|---|---|---|---|---|
| HD Analog | 1 | 84.92 MHz | 6844 / 7680 (89.1 %) | 15 / 32 |
| SD Analog | 1 | 72.10 MHz (Fmin 27) | 6852 | 15 |
| HD HDMI | **2** (seed 1 stalled router2 under machine load) | 82.81 MHz | 6848 | 15 |
| SD HDMI | 1 | 83.88 MHz (Fmin 27) | 6864 | 15 |
| HD Dual | 1 | 81.65 MHz | 6851 | 15 |
| SD Dual | 1 | 84.33 MHz (Fmin 27) | 6861 | 15 |

- iceunpack OK × 6; the packed config carries the new labels.
- dupin: 0 `CARRY I0==I1`, so router1 is safe if a rebuild stalls.
- **Rebuild:** `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh coastline`. If router2 stalls, prefix
  `NEXTPNR_ROUTER="--router router1"`.
- **Fit history.**
  - First build: 7353 LC (95.7 %). Cuts: luma [1 2 1] instead of two 9-tap boxes, two palette-address stages
    instead of two 32-bit data copies, and the sequencer's products moved onto the line engine's multiplier
    (7124 LC).
  - Then the curve and wave terms were folded straight into XI / WD / m, and one scratch register holds the four
    knob squares: 6844 LC.

## Deviations
- **The notch.** The still's dark-green widening of the black zone at the top trough (y 180–300) is not modelled: it
  is one trough only.
- **Thin lines.** The white streak and the salmon / grey-pink / olive lines are sharpened in the palette. The shell's
  luma cap (drawn whites ~777) leaves the white streak cream rather than white, and luma [1 2 1] plus the chroma
  9-tap soften them a little.
- **Geometry.** The coast and arc are quadratics (max error 7 / 11 px against the measured edges); cubics would fit to
  1 px.
- **Slow wave.** It is a second sine (rms 0.018 in m). Its drift is a choice: half wave 1's cycle rate, because the
  clip's estimate was unstable. The pattern moves 70–78 lines per 0.5 s against the clip's 75–98.
- **Edges.** The left edge fades in from black over ~12 px (filter warm-up), the capture borders are not drawn, and the
  RTL picture sits 4 px right of its virtual position. There is no camera grain.
- **Resolutions.** 720p shows the same picture as 1080p. **SD shows the centre 4:3**: the arc and the offshore cyan
  are mostly cropped on the right, and the bay's far left too.
- **Extreme corner.** At Waves 1023 + Surf 1023 the waves are so dense that one-row contour stripes appear (30–50 runs
  ≥ 24 px per 720p frame). They read as a fine texture, and the defaults and P12 0 / 100 have none.
- **Cut for budget.** The luma 9-tap boxes became [1 2 1] (the contract minimum).
- **Replaced on review.** A "Terrace" waveform switch (clipped sines) changed only 12.7 % of the pixels at the
  defaults, so it was replaced by Ripple.
