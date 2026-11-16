# Dazzle (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** Flash `out/rev_b/ron/hardline/dazzle.vmprog`.

## What it draws

First World War dazzle camouflage: Norman Wilkinson's ship schemes and Edward Wadsworth's "Dazzle-ships in
Drydock at Liverpool" (1919). The screen is cut by straight lines into hard-edged shards and every shard is
filled with bold black and white stripes that run against its neighbours'. Black and white only (U = V = 512 on
every pixel).

    cut k (k < Cuts):  s_k = cos(a_k) (x - X) + sin(a_k) (y - Y) - Scatter * dist_k,   a_k = base_k + rate_k * Turn
    code  = the sign bits of the s_k (one bit per cut)
    fill  = ROM[Angles, code]: phase = (U, U/2 or 0) +- (V, V/2 or 0) + q/4,  U = (x - X) / pitch,  V = (y - Y) / pitch
    ink   = |frac(phase + camera) - 1/2| < weight / 2

The fill ROM was designed offline (simulated annealing over the 256 codes) so that flipping any ONE code bit,
that is, crossing any one cut, turns the stripes by at least 26 degrees: two shards that share an edge never
run the same way. The twelve fills are stripes at 0, 90, +-45, +-26.6 and +-63.4 degrees, built from U and V by
shifts only, which also gives each direction its own pitch (x0.71, x0.89, x1, x1.41 or x2 of Period), plus a
quarter-period phase per shard. The default frame is six cuts at Scatter 63 %: about twenty shards of all
sizes, stripe pitch 35 to 98 virtual px.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the pivot the cuts fan around, +-1536 virtual px (0.8 screen widths) from the centre; the whole figure, stripes included, moves with it. Glided |
| K2 | Y | the same, +-1024 virtual px (0.95 screen heights). Glided |
| K3 | Period | stripe pitch of the x1 fills, 12 .. 200 virtual px, exponential (default 49); the stripes scale about the pivot, the cuts stay. Glided |
| K4 | Cuts | 2 .. 8 cuts, stepped with hysteresis (default 6) |
| K5 | Turn | swings every cut about the pivot at its own rate (x1, x-1, x2, x-2, x3, x-3, x4, x-4; x1 = +-45 degrees over the knob), centre = the base arrangement, so the shards reshape instead of turning rigidly. Glided |
| K6 | Camera | how far the camera (or Osc) pushes the stripe phase: 0 .. 4 periods at full white, default 100 %. Glided |
| S7 | Glide | Off = the knob value of this field, no easing at all / On = 1/16 of the gap per field, never slower than one step, landing exactly (default On) |
| S8 | Fill | Stripes / Checks (the stripes XOR the perpendicular stripes of the same pitch: a checkerboard turned to the shard's direction) |
| S9 | Angles | Free / Square (every cut snapped to the nearest 45 degrees and only the 0 / 90 / +-45 fills: a hard-edge, Vorticist picture; with Turn the cuts flip from step to step) |
| S10 | Weight | Bold (equal bands) / Fine (white lines of 1/6 duty on black, never under 2 pixels; with Checks a fine grid) |
| S11 | Source | Video / Osc |
| P12 | Scatter | the headline. 0 % = every cut passes through the pivot: a pinwheel of striped wedges. 100 % = each cut pushed off the pivot by its own distance (up to 0.6 screen heights, both ways): scattered shards of all sizes. Default 63 %. Glided |

Every knob has a +-1-count hold against pot jitter (the two ends always get through). Period and Weight read as
on the rest of the set.

## Camera

The camera is never shown. Its luma (minus black) goes through a per-field gain table (K6), a horizontal box
(64 px at 1080, 32 px at 720p and SD) and a vertical one-pole (1/64 per line at 1080p, 1/32 at 720p and 1080i,
1/16 at 480i / 576i) and is ADDED TO THE STRIPE PHASE of every shard; the cuts stay rigid. Black input, or
K6 = 0, is exactly the generated picture.

- 25 %: up to 1 period at full white; a gentle swerve.
- 50 %: up to 2 periods.
- 100 % (default): up to 4 periods at full white (about +-2 around a mid-grey): the stripes pile up and open
  out around the subject like contour lines.

The anti-aliasing follows the bent stripes: the camera's own phase gradient (horizontal from the line store,
vertical from the one-pole's step, stored beside it) is added to each fill's analytic gradient, so bent edges
stay one pixel soft. Source = Osc puts the shell's drifting 640 x 540 triangle lattice at the same point at
twice the camera gain (8 periods peak to peak at 100 %), so the flip is unmistakable; it is the only thing in
the program that moves by itself.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the hold + glide of six knobs (five clocks each), then
  a sequencer of 72 slots of 24 clocks on ONE serial 16-step multiplier: the pitch (2^(i/256) ROM), the pivot in
  pixel units, the stripe phases at pixel 0 / line 0, and for each of the eight cuts its angle, sine and cosine
  (1024-point quarter-wave ROM, linearly interpolated to 16-bit angles so Turn moves the far end of a cut by a
  fraction of a pixel), the pixel step, the line step, the line wrap and the field seed, written to a small cut
  table. The gain table is refilled by an accumulator. About 1,800 zone clocks.
- **Per line**: no multiplier. Each cut is one 28-bit DDA in pixel units (Q14) with its own increment
  register: it adds the pixel step on every clock of the line window (W + 8 steps), then once more with the
  wrap (line step - steps x pixel step) parked in that register from the cut table, so the adder has no input
  mux. The stripe phase V steps once per line; U runs along the line.
- **Per pixel** (26 stages, no multiplier): the eight signs make the code; the distance to the nearest cut
  (0 .. 1/2 pixel in sixteenths) and the mask of near cuts come from a 3-level minimum tree; two copies of the
  fill ROM give this shard's fill and the fill of the shard across the near cut(s); phases by shift-select
  and one add; the phase gradient |d/dx| + |d/dy| analytically; coverage by a 4-step divider (17 levels), widened
  5x past 1/4 cycle per pixel so stripes fade to flat grey instead of aliasing; the cut edge blends this shard
  (anti-aliased) with the shard across (taken as plain ink or ground) by the cut distance, as min / max.
- **Output**: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (32) in every mode, virtual 1080-line coordinates from the timing ID.

Chassis: Kinetic riley's [A] timing / zone / width / generated avid, the [B] glide datapath (rewritten for
the Hardline Glide rule), the serial multiplier of [C], the [D] source field (with the gain table, a wider
store and the 64-px option added) and the luma half of [F]. The wave engine, palettes, blend and every
chroma path are gone.

## Status

- v0.1.0 built 2026-11-15 through the shared queue, **router2** (the builder default), **every config on seed 1**.
  Not flashed.
- Post-route Fmax: HD Analog 82.73 MHz, HD HDMI 78.52, HD Dual 81.89 (HD needs 74.25); SD Analog 76.38,
  SD HDMI 73.29, SD Dual 77.15 (SD needs 27).
- 7,010 .. 7,032 / 7,680 LC (91.5 %), 21 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin: 0).
  The HD Dual route took 455 s (router2 was slow on it, not stalled).
- Rebuild: `./build_set.sh hardline dazzle`.
- The first RTL built at 7,354 LC (95.8 %) and needed seed 2 on HD HDMI (74.76 MHz) and HD Dual. A register
  diet (distance and coverage merged onto one carry chain, two pipeline stages gone, sequencer temporaries
  reused; latency 36 -> 32) brought it to the numbers above with the same arithmetic.
- **Model = RTL** (GHDL, the built sources): pixel diff 0 of 918,720 on a full 720p60 frame at the defaults with a
  black input, 0 with Source = Osc, 0 with bear.png as the camera at K6 100 %; 0 of 173,988 on each field of an
  NTSC frame (720 x 243); 0 on 1080p-timing renders (decimation 4: the 64-px box path) with the camera; 0 on
  720p quarter-rasters with other knob values and every switch (X, Y, Period, Cuts 2 and 8, Turn, Scatter 0 and
  100 %, Checks, Square, Fine, Osc, K6 60 %). Glide On: the third field of a glide from the defaults (X, Period,
  Camera, Scatter moving, Osc) matches the model with three steps of 1/16 easing, 0 diff, and a +-1-count
  knob step on Turn is held. The GHDL frame is the
  model's shifted 4 columns right with 4 blank columns at the left (the shell's generated-avid start).
- NTSC shows the same picture (the centre 4:3 of the 16:9 one, same shards top to bottom).
- Renders: `.hardline_work/rb_dazzle/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (P12 travel),
  `camera.png` (K6 = 25 / 50 / 100 %), all from the integer model `imodel.py` except the RTL panels of cmp_final.

## Cut / deviations

- **X / Y reach +-1536 / +-1024 virtual px, not "about 2 screens".** With the pivot two screens away the frame
  held one plain stripe field and no cut at all; at this range the ends still show two to four shards.
- **Fills are shift-only, so pitch rides on direction.** The spec asked for six directions normalised to one
  pitch plus a x1 / x1.5 variant. Here the twelve fills are (U, U/2 or 0) +- (V, V/2 or 0): 0, 90 at x1 and
  x2, +-45 at x0.71 and x1.41, +-26.6 and +-63.4 at x0.89. That needs one phase DDA instead of six and no
  per-pixel multiplier, the pitch range across a frame is wider (2.8 : 1), and every fill has its
  perpendicular partner in the set, which Checks needs. Period therefore starts at 12 virtual px (finest
  stripes 8.5), not 8.
- **The shard across a cut is not anti-aliased inside the blend.** Within half a pixel of a cut the pixel is
  min / max of this shard's anti-aliased coverage and the cut coverage, with the other shard taken as plain
  ink or ground. Cut edges are clean at 4x zoom; where a stripe end of the far shard meets the cut the last
  half pixel is a step. A second coverage divider (about 250 LC) would remove it.
- **Fine + Checks is a grid** (the union of the two line sets), not their XOR.
- **Camera blur is wider at 1080**: a 64-px box and a 1/64 one-pole (the shell's 32-px box shredded the stripes
  into ragged blobs at 1080p in the model; 720p and SD keep 32 px). The blur is about 3 % of the picture
  width in every mode.
- **S9 reads Free / Square** (Free is the default and the dazzle picture; the spec listed Square first).
- **Interlaced modes** (480i, 576i, 1080i): the cut edge ramp is 2 pixels wide, because a field line is two
  pixel widths tall. At SD the finest fills of Period 0 % (8.5 virtual px = 1.9 field lines) fade to grey;
  720p and 1080p hold every Period.
- `field_n` is passed through one register (the core ties it high on every input). Latency 32, not the
  shell's 36.
- Chassis note for the other riley-based programs: in riley the Osc lattice's x phase on the FIRST line of a
  field still carries the previous field's drift (it is reset at the avid fall, before the field's drift
  step). It is a one-pixel shift of one line; here both lattice phases restart in the vblank zone.

## Not sure of on hardware

- Nothing on this chassis has been flashed yet (riley has not either). The generated avid, zone and width
  logic are the heirloom shell's, which is hardware-validated.
- The picture is all sharp, full-swing luma edges in motion: the [1 2 1] + 13/16 conditioning is built in,
  but if HD HDMI bars show, Period at minimum with Fine is the first place to look.
- The Cuts knob's hysteresis band (8 counts) and the +-1-count hold are untested against real pot noise.
