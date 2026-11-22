# Separatrix: the phase portrait of a pendulum

**v0.1.0 built 2026-11-22, not flashed.** Patchbay (wave 2). An analog-computer plot: a squared V ramp (the kinetic
term) plus an H-locked cosine VCO and a second VCO at twice the rate (the potential) mixed into the energy H, banded
by a staircase colouriser, with a window comparator at the saddle energy drawing the separatrix. Presets are
hand-written placeholders.

## The patch

    V ramp (centred, virtual y - 540) ---------> multiplier wired as a squarer -> p^2      (per line: 2nd-order DDA)
    H-locked cosine VCO (K1 Period; P12 Gravity = amplitude) -> -G cos q               \
    VCO at twice the rate (K2 Detent = amplitude; slow LFO) -> -D cos 2q               -> potential table V(q), V'(q)
    S9 Tilted: linear ramp in q (F = 0.3 G) -> -F q                                    /   (rewritten every field)
    camera luma -> 8-px cells -> vertical slew -> 2 horizontal slews -> x K6 --------> +    (S11 Osc: drifting lattice, 2x)
    mixer -> H = p^2 - G cos q - D cos 2q - F q + camera
    H -> staircase quantiser (K4 Bands, K3 flow) -> colouriser (S10 palette pair) -> chroma transform (K5 Colour)
    H -> window comparator at Hc (the saddle energy, found by the table writer) -> the separatrix: white core,
         pale-cyan glow, halo; and the colouriser's inside / outside key (closed orbits warm, open orbits cool) (S8)
    double well (D < -G/4): a second comparator at the hump energy -> the white figure-eight inside each eye
    S7 Orbits: the staircase edges drawn as thin lines on black (a scope plot)

## What it draws

At the defaults: three eye-shaped nests of closed orbits along the middle of the screen (warm colouriser bands:
crimson, scarlet, orange, magenta with bright fringes, a pink pupil at each centre), a bright white lens-shaped
separatrix through the saddles between them, and above and below it wavy navy / teal / blue / sea-green bands
streaming across the whole width, crowding toward the top and bottom edges (equal steps of energy in p^2). The
orbits are about 580 virtual px tall (288 px half-height) and 640 px wide.

Motion at the defaults: the bands flow outward in energy (new rings are born at the eye centres, travel out, peel
off at the separatrix and stream away toward the top and bottom edges, about 0.37 band/s), the whole portrait
drifts right along q (about 22 s per eye), and a 34-s triangle LFO leans the detent by +-0.06 G, so the eyes'
flanks breathe between lemon and lozenge. Speed at its centre stops the drift and the flow; the LFO keeps going.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Period | H cosine VCO (frequency) | eyes across the screen, 1 .. 6, detented (flat for ~100 codes around each integer, a glide between); 410 = 3 |
| K2 Detent | 2x VCO (amplitude), bipolar, linear, +-8 deadband | left: the eyes flatten into lozenges (d = -0.25), then pinch into pairs with a white figure-eight inside (double well, d = -1.1 at 0); right: cusped eyes, then small secondary eyes appear between them (d = +0.6 at 1023) |
| K3 Speed | phase drift + staircase offset, bipolar, square law, +-8 deadband | the portrait drifts along q (right / left) and the bands flow out of (into) the eye centres; full = 0.45 eye/s and 3.6 bands/s; 680 = the default drift |
| K4 Bands | staircase quantiser (step) | energy per band, square law: 0.26x (two or three wide bands per eye) .. 3.2x (tight lace; bands under 2 px fade to a flat blend); 512 = six rings in the default eye |
| K5 Colour | colouriser output (chroma transform) | eight colourings: U / V swap and inversions of the colouriser's chroma (ember / teal, blue / rust, green / magenta, violet / green, magenta / green, green / violet, cyan / red, orange / blue); zone 4 (the centre, default) = the designed palette; 8-code hysteresis, the two ends always select the end zones |
| K6 Camera | source level into the mixer | 0 % = no camera (the black-input picture, exactly) .. 100 % (default) = full white adds ~0.6 of the default eye's energy span: the eyes bend toward and away from the subject, the lens line breaks where the picture is bright, contours stay clean curves |
| S7 Look | colouriser mode | Bands (default) / Orbits: thin iso-energy lines on black, warm inside, cool outside, white separatrix (a scope plot); dense areas fade to a dim wash |
| S8 Separatrix | window comparator | On (default) / Off: no white line and no inside / outside split, the whole portrait in the cool family |
| S9 Drive | potential (linear term) | Level (default) / Tilted: the washboard: fish-shaped eyes with the saddle at their right tip, open bands sloping downhill |
| S10 Colours | colouriser (setting) | Ember (crimson / orange eyes in navy / teal water) / Moss (greens in plum) |
| S11 Source | mixer's source input | Video (default) / Osc: a drifting 640 x 540 px triangle lattice at 2x the camera's gain: rounder, emptier eyes, a broken lens line, rolling outer bands |
| P12 Gravity | cosine VCO amplitude (headline) | eye half-height 32 .. 543 virtual px (G grows with its square): 0 % = nearly level chirped bands with a thin rippling lens chain; 50 % = the classic portrait; 100 % = eyes as tall as the screen, the separatrix touching the top and bottom edges, rings packed tight (the climax) |

## Video

- **Video (default).** Camera luma, (Y >> 2) - 16, averaged over 8-pixel cells, slewed down the screen (one pole,
  1/32 per line; one 12-bit state per cell in a block RAM) and across (two one-pole filters, 1/16 per pixel), times
  K6, is added to H. The orbits hug the picture's large shapes: bright areas raise the energy, so eyes shrink or
  split over them and the bands swell; the slews keep every contour a smooth curve. With a black input the patch
  gives its full picture.
- **Osc.** The shell's drifting triangle lattice takes the camera's place at twice the gain (the flip is clear at
  K6 100 %).

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Glide (K1, K2, K4, K6, P12, 1/8 per field); knob
  decodes; 15 slots on ONE 32-bit serial multiplier (density, G, D, slope coefficients, phase step, speed^2,
  tilt step and saw start, camera gain, the line DDA's start values), with the cheap laws (eye height, detent,
  tilt, flow, drift, start phase) as plain adds. Then the **table writer** (zone clocks 400 .. 2450): for 512 bins
  over q = 0 .. pi it computes V = -(Ga cos q + Gb cos 2q) and V' = (Ka sin q + Kb sin 2q) (already scaled by
  Gravity, Detent, Bands and Period, slope per output pixel) on one pipelined 16 x 10 multiplier, four products
  per bin, writes them to a 512 x 32 block RAM, and tracks the saddle energy max(V - F q_loc) and the hump energy.
  No per-pixel multiplier anywhere in the field path.
- **Per line.** p^2 - Hc by second differences in Q24 bands (exact quadratic), latched at each line start; its
  difference is the vertical slope.
- **Per pixel (14 stages to the colour).** Window phase DDA (2^26 per cycle) with its wrap flag; table read
  (folded onto 0 .. pi); H = p^2 + V + camera; the tilt saw and window offset run as a delayed copy of the beam at
  stages 6 and 7; |grad H| = max + min / 2 of |V' - F| and the line slope; band phase = H - offset (4 bands per
  palette cycle, 16 steps a band); density class (bands > 8 / 4 / 2 px / flat) picks one of four box-filtered
  copies of the palette in the colouriser ROM; the nearer of |H - Hc| and |H - Hc2| (the latter only in a double
  well) against 1x / 2x / 4x the window (4 virtual px x |grad H|) gives the line level (core, glow, halo), blended
  toward white / pale cyan by 1, 1/2, 1/4; the Orbits look; the K5 chroma transform; black off the generated avid.
- **Output.** As the shell: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma
  13/16, a vertical [1 1]/2 with the line above, the blanking gate.
- **Contracts.** Generated avid; one latency (C_LAT 20) in every mode; virtual 1080-line coordinates; colours in
  the hardware convention (U = Cr, V = Cb); drawn whites ~777.
- **Block RAM (23).** Colouriser 8, potential table 4, writer sine 2, luma cells 1, box delay rings 4, line store 4.
  Every written RAM carries `no_rw_check` (0 transparency emulations in the yosys log).

## Status

- **v0.1.0 built 2026-11-22 with router2 (the build default), all six configs on seed 1, not flashed.**
  Rebuild: `./build_set.sh patchbay separatrix` -> `out/rev_b/ron/patchbay/separatrix.vmprog` (472,587 bytes).
- Post-route Fmax (the builder's last report, PASS on the first seed):

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 76.89 | 6,990 |
  | SD Analog | 1 | 79.07 | 6,999 |
  | HD HDMI | 1 | 79.68 | 6,982 |
  | SD HDMI | 1 | 78.47 | 6,987 |
  | HD Dual | 1 | 86.18 | 7,011 |
  | SD Dual | 1 | 72.98 | 6,993 |

  6,982 - 7,011 / 7,680 LC (91 %), 23 / 32 EBR. iceunpack OK x6; the packed config carries the new labels and
  presets. HD Analog's margin is 2.6 MHz: if a rebuild misses, let the seed retry run, or use router1
  (`NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay separatrix`; dupin: 0 CARRY I0==I1, 0 duplicate LUT
  inputs). No pin-skew screening.
- **Checks** (working files in `.patchbay_work/rb_separatrix/`):
  - `check.sh` clean; `scan.py`: 0 integer-minus-signed sites; standalone yosys: 23 SB_RAM40_4K, 0 transparency
    emulations.
  - GHDL 720p60, decimation 1, black input, every control at its default: **bit-exact with the integer model
    `imodel.py`** (0 of 917,444 compared pixels, rows 1 .. 719, differ; the tester's capture sits 4 px left of the
    generated avid, as in every shell program). `rtl_default.png`.
  - A second 720p frame with Orbits + Tilted + Moss + Detent gliding toward 0 (double well, so the hump comparator
    is live) + Colour zone 1: **0 differing pixels** against the model with the RTL's glide emulated (`rtl_combo.png`).
  - A later default frame (field 10): **0 differing pixels** against the model at field 10; 697,677 pixels changed
    against the field-3 frame, so the portrait moves exactly as the model moves it (`rtl_motion.png`).
  - Both 480i fields (720 x 243, `--video-mode ntsc`): **0 differing pixels** against the model (rows 1 on). SD draws
    the same portrait with the Period count across the 4:3 width (`rtl_ntsc.png`).
  - Both 1080i fields (1920 x 540, `--video-mode 1080i5994`): **0 differing pixels** against the model (rows 1 on);
    the same picture as 720p / 1080p (`rtl_1080i.png`).
  - Osc (`rtl_osc.png`) and selfie (`rtl_selfie.png`) frames match the model's renders in structure (egg-shaped
    eyes with fewer rings and a broken lens line under Osc; eyes bent toward the bears with clean contours under
    the camera) but not bit for bit: the model's camera-path filter (from chladni's model) does not reproduce the
    RTL's cell / slew start-up or the lattice phase exactly (64 % of Osc pixels differ by a band-phase offset).
  - `thincheck.py` on the model (720p and 480i, 14 settings x 2 times): no single-row lines of 24 px or more except
    3 runs (24 - 25 px) in the Orbits look at 480i. Without the shell's vertical mix P12 0 % had hundreds.
  - `sheet.png`, `strip.png`, `camera.png`: control sheet, 8-frame motion strip and camera strengths, all from the
    integer model.
- First RTL (CORDIC hue rotator, 29 multiplier slots) was bit-exact but projected at ~8,100 LC; see notes.md,
  "Fit rounds", for the cuts.
- **Watch on hardware:** crisp moving colour edges at HD HDMI (the shell's conditioning is in; if bars show, widen
  the chroma boxes first). The zone engine needs about 2,500 clocks after the 1,024-clock idle test; every real
  timing has 14,000 or more.

## Cut / deviations

- **K5 Colour is eight discrete colourings, not a continuous hue rotation.** A six-stage CORDIC rotator was built
  and verified bit-exact, but with the table writer it pushed the program to ~8,100 LC; the chroma transform
  (swap / invert) costs almost nothing and gives eight bold palettes per S10 setting (16 in all).
- **Detent is linear** (the draft was square law) and **Bands, Gravity and the camera gain use shift-add laws**,
  to free multiplier slots. Ranges are as designed.
- **Period keeps its count at SD**: three eyes span the 4:3 width, so on SD each eye is 25 % narrower than at HD
  (the same height). 720p and 1080p draw the same picture.
- **Under Tilted only each well's loop is drawn** (the separatrix branches that run off to the right are not); the
  bands carry the tilt seamlessly.
- **Dense regions**: where bands fall under 2 px the colouriser shows the palette's flat blend; bands of 2 - 8 px
  are box-filtered by class (one octave per class), so edge softness steps by octaves rather than exactly 1 px.
- **The camera bends the energy but not the comparator window's width** (the analytic gradient ignores the
  camera's own slope), so the line thins or thickens a little where the picture changes fast.
- Row 0 of each field mixes with the last row of the field before (the line store is not cleared), as in chladni.
