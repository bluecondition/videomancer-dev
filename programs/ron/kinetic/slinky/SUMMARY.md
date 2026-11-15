# Slinky (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed.

## What it draws

One shape followed by a train of graded copies along a swinging path, so the train reads as an extruded,
coiling tube. The art it comes from: motion-graphics echo tubes and the Slinky. At the defaults (Outrun
palette) about eleven overlapping discs, amber at the head through cyan to pink at the tail, each ringed in
the dark ground colour, crawl round a figure-eight like a caterpillar; the newest copy fades in at the tail.

## Element and state

- Element: copy j = 0 .. 12 (13 copies; the head is j = 0 and is drawn on top).
- State: its centre = path(phase - Spread x j). path = (Ax sin a, Ay sin b), Ay = Ax / 2.
  - Swing: a = theta, b = 2 theta (a figure-eight). Orbit: a = theta + 1/4 cycle, b = theta (an ellipse).
  - Snap: theta dwells on 8 stations per cycle for 75 % of the time and eases (half-cosine) to the next, so
    the head hops and each copy follows one Spread later.
- Size: R_j = R0 + j x dR (Shape), the tail between R0 / 2 and 2 R0.
- Colour: the grade runs accent (head) -> B -> A -> halfway to the ground (tail) over the 13 copies.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | head radius 32 .. 255 virtual px, exponential, glided (default 112) |
| K2 | Shape | taper along the train: cone (tail half the head) .. cylinder at the centre .. flare (tail twice the head) |
| K3 | Depth | how far the path swings: 0 .. 900 px across, half that down (0 = all copies on the centre) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Spread | phase step per copy 0 .. 1/8 cycle, square law: a tight tube .. separate echoes (at the top the 13 copies fold onto 8 places) |
| K6 | Colour | the 16 shared palettes with hysteresis: ground = background and outlines, accent / B / A = the grade |
| S7 | Wave | Smooth / Snap (hops between 8 stations) |
| S8 | Field | Swing (figure-eight) / Orbit (ellipse) |
| S9 | Form | Ring / Square (rounded corners) |
| S10 | Body | Solid (filled, occluding, outlined) / Wire (outlines alone, see-through) |
| S11 | Source | Video / Osc |
| P12 | Extrude | 1 .. 13 copies, glided; the newest copy fades in in eighths |

Presets: Slinky (defaults), Coil, Wire Tunnel, Hop Squares, Echo Windows, Horn.

## Video

- The picture does not move the geometry. It lights the copies: half the incoming luma (less black) is added to
  the colour of every drawn copy, times that pixel's coverage, so each copy is a window onto the camera tinted by
  its grade colour (the chroma stays the grade's). In Wire the outlines carry it.
- Black input: nothing is added and the plain graded tube runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  full scale (about twice the camera's level).

## Engine

Sections in slinky.vhd, marked [A] .. [F] (chassis lifted from louver):

- [A] Timing ID -> virtual-1080 steps; vblank zone (no avid for 1024 clocks); source width; glides on Scale and
  Extrude; a per-field sequencer with one 14-step serial multiplier (products 16 clocks apart), then a per-copy
  loop of 90 clocks: phase, snap easing and two path sines from the quarter-wave ROM, centre, radius, and the
  line engine's start values, written to a 256 x 32 block RAM.
- [B] Generated avid (parity-locked start 3 / 4 clocks after the source's, field-latched width).
- [C] Source luma or the Osc lattice, through a block-RAM delay ring.
- [D] The copies. Per pixel and copy: f = x^2 - L, with L = 2 x cx - cx^2 + H2 + M one 23-bit linear accumulator
  per copy against a shared x^2 (second differences), tested in units of 8 px^2: outer hit f < 0, inner hit
  f + T + 2M < 0. H2 comes per copy and line from a line engine (13 clocks per copy during the active line):
  E = R^2 - dy^2 stepped by second differences; Ring H2 = E; Square H2 = R^2 on the body rows and
  R^2 - T + E on the cap rows (the rounded corners). After every line the start value and the x step of each
  copy go through the copy's step register into its accumulator (2 clocks per copy, 30 clocks of hblank).
- [E] Priority: the lowest-numbered hit and the one below it (carry-chain lowest-set-bit), the winner's two
  7-bit coverage windows muxed out; coverage in eighths by a per-field power-of-two shift (ramp = M = 2^n >
  the largest radius, so 1 .. 2 px on the largest copy, up to 4 px on the smallest of a full cone). Colour =
  T(copy, coverage), a 512-word table refilled every field from the grade ROM: ground + (grade - ground) x
  coverage / 8. Solid: on the top copy's rim the copy below shows through; inside, the fill by the inner
  coverage (the outline is the ground colour). Wire: min(outer, 8 - inner) of the top outline.
- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums with the per-line resync, luma [1 2 1]/4
  delayed to the chroma's group delay, luma 13/16 toward black, blanking gate.

One latency in every mode: C_LAT = 16 = processing_delay_clks; 9 pixel stages + 4 output stages + 4 px of filter
delay. Per field: all knob-derived constants, the copies' centres and radii, the colour table, the phase step.
Per line: every copy's H2 and start value. Per pixel: 13 accumulators, 26 tests, priority, table read.
Luma is 10 bits, chroma 8 bits until the output filter. Block-RAM data gets a fabric register before
arithmetic; no RAM reads the address it is writing (no_rw_check on all of them).

## Status

- Built with router2 through `./build_set.sh kinetic slinky` (rebuild command), 2026-11-14.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 2 | 77.66 MHz | 6966 |
| SD Analog | 1 | 80.97 MHz | 6957 |
| HD HDMI | 1 | 84.16 MHz | 6957 |
| SD HDMI | 1 | 74.37 MHz | 6961 |
| HD Dual | 2 | 76.01 MHz | 6965 |
| SD Dual | 1 | 81.20 MHz | 6960 |

- 6,966 LC (90.7 %), 17 of 32 block RAMs. Never flashed. HD Analog and HD Dual missed on seed 1 and closed on
  seed 2 (record: seeds 2 / 1 / 1 / 1 / 2 / 1), so the HD margin is seed-dependent.
- GHDL render = integer model (.kinetic_work/rb_slinky/imodel.py), 0 mismatching pixels in every frame compared:
  - 720p decimation 1 (1280 x 720), defaults, black input;
  - 720p decimation 2 (640 x 360, the top-centre quarter of the picture): defaults; Snap + Orbit + Square + Wire
    with a flare, a large Scale, reverse Rate and a fractional copy count; Solid + Square + Orbit at a small Scale
    with all 13 copies; Source = Osc at the defaults (the lattice included).
  - Every capture matches the model at phase step 2, so the phase advances per field as modelled (motion).
  - With selfie.jpg the render was viewed, not diffed: the picture shows inside the copies, edges stay clean.
  - NTSC decimation 1 (720 x 243 per field) and 1080i decimation 2 (960 x 270 per field), defaults, black input:
    both fields of each, at consecutive phase steps.
- Rasters rendered: 720p (full and decimation 2), NTSC, 1080i (decimation 2). 1080p was not rendered. NTSC shows
  the same train, round on a 4:3 screen, with the path 3/4 as wide.

## Cut / deviations

- 13 copies, not the spec's "up to 24". One copy costs about 143 logic cells (23-bit accumulator, 23-bit step
  register, two 20-bit tests, its share of the priority mux) and the first RTL with 24 was 114 % of the chip.
- The colour grade runs head = accent -> B -> A -> halfway to the ground at the tail (the spec lists it from the
  ground up; the head on top needed the strongest colour). In Solid the outline is the ground colour, so over
  the background a copy shows as its fill and the outline appears where copies overlap.
- The outline width is one shared T = 3/32 Rmax^2 + 4 Rmax in r^2 units, so it is about R/21 + 2 px on the largest
  copy and up to twice that on the smallest (thicker lines on small copies).
- The anti-aliasing ramp is one power of two per field (not a per-copy reciprocal): 1 .. 2 px on the largest
  copy, up to 4 px on the smallest copy of a full cone or flare. Centres move in whole virtual pixels.
- Square: straight sides are anti-aliased; the top and bottom edges are whole-line (no vertical AA), the outer
  corners are rounded by a short arc and the inner corners are square. On SD the top and bottom outline of a
  small square can be a single field line.
- Field = Swing / Orbit is one figure-eight and one ellipse (no pendulum or rose-curve variants).
- Video does not bend the geometry here: it lights the copies (added luma, the grade's chroma). Osc adds the
  lattice's luma only.
- K3 Depth at 0 and K5 Spread at 0 both stack every copy on one place (one shape visible, the taper hidden
  under the head unless Shape is a flare).
- Only Scale and Extrude are glided; the path phase is 12 bits per cycle (about 1.4 px steps at full Depth).
- SD shows the centre 4:3 of the 1920-wide canvas; the path's width is reduced to 3/4 there so the train stays
  on screen.
