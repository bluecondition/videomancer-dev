# Tumbler (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed.

## What it draws

The tumbling-blocks (rhombille) quilt tiling and the reversible Necker cube. Every hexagonal cell is a cube of
three rhombi in three tones. Each cube carries a light angle; turning it rotates the three tones round the
faces, so the cube appears to tumble, and a third of a turn swaps which tone is "top" so it flips from standing
out to hollow. At the defaults (Bauhaus palette) about ten columns of cream / red / blue cubes with black
hairline seams and a darker inset frame on every face fill the screen, and a slow wave travels across them along
a lattice diagonal, shading each cube through the blends on its way to the next permutation.

## Element and state

- Element: a cube (hexagonal cell). Half column pitch Hc, column pitch W = 2 Hc; rows are staggered by Hc.
- State: its light angle psi, 14 bits per turn. Face f (top 0, right 1, left 2) reads the colour wheel at
  psi + f/3 turn. The wheel is ground -> A -> B -> ground, 32 steps per segment, so at psi = 0 the faces are the
  three pure palette colours (top = ground, right = A, left = B).
- theta = Spread x index - phase. Linear: index = column + row / 2 (wave fronts lie along a lattice line).
  Radial: index = max + min / 2 of (|column|, 7/8 |row|) from the cube on the screen centre.
- psi = Flip x wave(theta) + source + face / 3. wave is a sine, or Snap (the first sixteenth of the sine
  stretched over the quarter cycle: 75 % dwell, eased flips).
- Tone = Three snaps psi + face/3 to the nearest third, so only the three pure colours appear and the cubes
  click from one permutation to the next.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | column pitch 48 .. 400 virtual px, exponential, glided (default 200: 9.6 columns) |
| K2 | Shape | cube proportion: flat tiles (lattice slope 3) .. tall blocks (1.125); default 69 % = regular hexagons |
| K3 | Inset | a framed panel inside every rhombus: frame width 0 .. 0.31 Hc, drawn 6/16 toward the line colour |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis: ground, A, B = the three face tones; accent = the edge line (was K6 until v0.1.1) |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: gain k / 256 on the held luma (0 = as with a black input, 25 % = the v0.1.0 strength, 50 % = twice it, 100 % = four times); scales Osc too. Spread (K5 until v0.1.1) is fixed at 520 (0.13 cycle per column) |
| S7 | Wave | Smooth / Snap |
| S8 | Field | Linear (diagonal fronts) / Radial (octagonal rings from the centre cube) |
| S9 | Edges | Off (black hairline seam, 2 virtual px) / On (accent-colour line, Hc/8 + 3 px wide) |
| S10 | Tone | Shaded (continuous blends round the wheel) / Three (three flat tones) |
| S11 | Source | Video / Osc |
| P12 | Flip | swing of the light angle 0 .. 1.5 turns, square law, glided: 0 % = still cubes, 100 % = every cube runs through nine tone steps each way |

Presets: Tumbling Blocks (defaults), Necker Snap, Neon Ripple, Tall Blocks, Flat Tiles, Cube Picture.

## Video

- Source = Video: luma (>>2, less 16) is smeared along x by a 1/16 one-pole filter and sampled ONCE per cube
  into a 256-entry block RAM addressed by half-column index, then held for the whole cube, so every cube stays
  rigid. The sample is taken on the first line of the row band above the cube (at the cube's centre column; at
  the last / first visible pixel for cubes cut by the right / left edge); the cubes cut by the top edge are
  sampled on the field's first line. psi += 48 x luma: white turns a cube 0.64 turn (two tone steps), so the
  picture is posterised into cube orientations. Black input: offset zero, the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at twice
  the camera's gain (full scale 1.3 turns).
- The picture never colours the faces (no Face = Video mode in the spec).

## Engine

Chassis lifted from louver ([A] timing / zone / glides / serial-multiplier sequencer, [B] generated avid,
[C] source oscillator and smear, [F] output conditioning and delay rings); the lattice and the look are new.

- The row axis is carried in x units: v = m y (m = Shape slope; per-line accumulator, step m x ystep, wrap at
  3 Hc). The three edge families are then x = k Hc, x + v = k W, v - x = k W: adders only, no per-pixel
  multiply. In a cell (lx in [-Hc, Hc), v in [0, 3 Hc)), ea = lx + v - W and eb = v - lx - W: both negative =
  this row's cube (right / left face by the sign of lx); one negative = the next row's cube, top face; neither =
  its left / right face.
- Edge distance: each rhombus is bounded by two of the three families, so the distance to its own boundary is
  the smaller of two folded forms. Slanted distances stay in x units; the line and inset thresholds are scaled
  by sqrt(1 + m^2) (a 256-entry ROM by Shape) once per field. Coverage in eighths from the linear forms on all
  three edge directions; ramp = 1 real pixel rounded to a power of two in the form's units.
- Colour: psi -> a 512-entry ROM (wheel position, shaded and three-tone) -> a 96-entry wheel table refilled
  every field with adders (2 block RAMs) -> blend toward the line colour by a weight in sixteenths (gated-sum
  multiply in its own two stages). Luma 10 bits, chroma 8 bits until the output filter.
- The only other per-pixel multiply is Flip x wave (8 x 10 gated sums over three stages).
- Per field: pitch (exp ROM), slope, the two origins (a cube centred on the screen centre, found by stepping
  back a column / a row band at a time), the odd-row start values, thresholds, the wheel table, the phase step.
  Per line: v, row parity, Spread x row, the forms' line constants. Per pixel: everything else.
- One latency in every mode: C_LAT = 30 = processing_delay_clks (23 pixel stages + 4 output stages + 4 px of
  filter delay). 16 block RAMs; no RAM reads the address it is writing (no_rw_check on all, 0
  emulate_transparency in the yosys log).

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now **Camera**, the camera strength (Ron's rule for the Heirloom / Kinetic / Patchbay sets).

- What it does: K6 scales the per-cube held source (the smeared luma, or the Osc lattice) that turns each
  cube's light angle psi (48 x source, 14 bits per turn). Source = Osc keeps its 2x on top.
- Gain law: linear, gain = k / 256 (k = the knob, 0 .. 1023). Decoded per field in the vblank sequencer (zone
  clocks 40 .. 43): the knob is latched with a +-4 deadband (the ends snap to 0 / 1023), then the gain eases 1/8
  of the gap per field. Applied with one 8 x 10 gated-sum multiply over pixel stages 7 .. 9 on the value read
  from the hold RAM; the {face, source} delay ring is now written at stage 9 (rc + 2 instead of rc + 4) so it
  still lands at stage 13. Same latency (30); the hold RAM is unchanged; psi wraps modulo a turn.
  - 0 % = gain 0: exactly what a black input draws (the wave alone);
  - 25 % (256) = 1x, the v0.1.0 strength (white = 0.64 turn, two tone steps);
  - 50 % (512, default) = 2x (white = 1.28 turns): the picture spreads over every orientation of the cube;
    every cube stays rigid and clean;
  - 100 % (1023) = 4x (white = 2.5 turns): the luma range runs round the colour wheel two and a half times,
    so the cubes read as a contour map of the picture; in a still it looks orientation-scattered. No washout.
- Frozen: K5 Spread at 520 (its default: 0.13 cycle per column). Spread was the weakest knob on the control
  sheet (min / default / max look alike in a still; Rate, the only speed control, was not touchable).
- K6's old function (Colour, all 16 palettes) moved to K5 with its hysteresis. Presets: Colour values moved
  to K5; their Spread values (300, 480, 620) are gone, so every preset now runs at Spread 0.13.
- Renders: `.kinetic_work/rb_tumbler/k6sheet.png` (selfie, 720p60 at decimation 2: K6 0 / 256 / 512 / 1023
  and Source = Osc at 512).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic tumbler`): router1, every config on seed 1. Post-route
  Fmax HD Analog 87.40, HD HDMI 86.74, HD Dual 78.46 (worst HD), SD Analog 85.72, SD HDMI 79.16, SD Dual
  85.98 MHz; 6,983 / 7,680 LC (90.9 %, was 6,656), 16 / 32 EBR. vmprog: out/rev_b/ron/kinetic/tumbler.vmprog
  (v0.1.1). Not flashed.

## Status

- Built with router1 through `NEXTPNR_ROUTER="--router router1" .kinetic_work/tools/build_queue.sh kinetic tumbler`
  (rebuild command: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic tumbler`), 2026-11-14.
  router2 (the default) STALLED on HD Analog seed 1 (overused=2 for 3,400+ iterations; log kept as
  .kinetic_work/rb_tumbler/pnr_router2_stall.log), so use router1. dupin.py on the synth json: no CARRY I0==I1.

| Config | Router | Seed | Post-route Fmax | LC |
|---|---|---|---|---|
| HD Analog | router1 | 1 | 82.46 MHz | 6638 |
| SD Analog | router1 | 1 | 80.53 MHz | 6647 |
| HD HDMI | router1 | 1 | 82.08 MHz | 6642 |
| SD HDMI | router1 | 1 | 87.40 MHz | 6656 |
| HD Dual | router1 | 1 | 83.04 MHz | 6651 |
| SD Dual | router1 | 1 | 86.56 MHz | 6649 |

- 6,656 LC at most (86.7 %), 16 of 32 block RAMs. Never flashed. Every config closed on seed 1.
- vmprog: out/rev_b/ron/kinetic/tumbler.vmprog.
- GHDL render = integer model, 0 mismatching pixels in every case rendered:
  - the full 720p frame (1280 x 720, decimation 1) at the shipped defaults, black input;
  - 640 x 360 windows of 720p: defaults with black input; Source = Osc; Video = selfie.jpg (model fed the
    tester's own input image); and Snap + Radial + Edges On + Three with Shape 300, Inset 700, Spread 600,
    Rate 800, Colour 5.
  The capture sits 4 columns right of the model (generated avid start), aligned before diffing.
- Rasters rendered in GHDL: 720p (full frame and 640 x 360 windows), NTSC (720 x 243 fields, decimation 1) and
  1080i (960 x 270 window of each field). NTSC and 1080i at the defaults with black input: 0 mismatching pixels
  in both fields of each. The model shows the same picture at 720p, NTSC (the centre 4:3) and 1080i. 1080p was
  not rendered. Motion: the two NTSC / 1080i fields and the 720p frames match the model at consecutive phase
  steps. Knob settings away from the defaults for the two glided knobs (Scale, Flip) were checked in the model
  only (the control sheet), not in GHDL.
- Longest single-row hard run in the model (default, Edges On, flat, Scale 0, Inset max): 14 px (limit 24).

## Cut / deviations

- Face brightness is not max(0, cos): the three faces read a three-colour wheel (ground, A, B) 120 degrees
  apart, which gives the three-fabric block at rest and exact tone rotation; blends between palette colours
  appear in Shaded mode only.
- Edges = Off still draws a 2-virtual-px black seam on every edge. Faces of neighbouring cubes carry different
  states, and the seam is what lets every edge be anti-aliased with one cube evaluated per pixel. Edges = On
  widens it and draws it in the accent colour. The Inset frame is a tint toward the same line colour.
- The field's first line is drawn black: the cubes cut by the top edge take their video sample on that line.
- The video sample point is one row band above the cube's centre and about 8 real px left of its centre column
  (the smear's lag); the smear is 16 real pixels at every resolution.
- Only Scale and Flip are glided; Shape, Inset and Spread read the knob directly.
- SD: the anti-alias ramp is 4 virtual px (2 real px across, about 1 field line down), the hairline is 4
  virtual px wide; 720p / 1080i: ramp 2 virtual px, hairline 3.
- No float prototype: per the run notes the design went straight to the integer model at RTL widths.
