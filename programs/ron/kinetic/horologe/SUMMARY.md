# Horologe (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. v0.1.1 (2026-11-15): K6 camera pass, see the end.

## What it draws

A wall of analog clocks whose hands are choreographed by a travelling wave, after "A Million Times" by Humans
since 1982 (a grid of clocks whose hands are driven so that lines, waves and figures run across the wall).
At the defaults (Bauhaus palette) a 16 x 9 grid of faint dial faces on cream, each with a red hand, a blue
hand and a gold hub; the two hands mirror each other, so every clock is a pair of scissors, and a slow wave
running along the diagonal opens and closes them in turn.

## Element and state

- Element: a clock, one cell of a grid of pitch P. The centre clock sits on the screen centre, so Scale zooms
  about the centre. Grid = Brick shifts every other row half a clock.
- State: the angle of hand 1 (signed, Q14 turns; 0 = pointing up, clockwise positive).
- theta = Spread x index - phase. Linear: index = column + row (the wave runs along the diagonal).
  Radial: index = max + min/2 of (|column|, |row|) from the centre clock (octagonal rings).
- hand 1 = Depth x sin(theta) + source. With Wave = Snap the angle ticks between the 8 compass points: it
  dwells on a point and makes a quick linear move (1/32 turn of input) to the next.
- hand 2 = 1/2 turn + (1 - Hands/32) x hand 1, Hands = 0 .. 127: at 0 the two hands are one straight line
  turning together, at the centre they mirror each other, at the top hand 2 counter-runs at 3x.
- A hand is a capsule from the centre: length 0.43 .. 0.28 P, half-width 0.018 .. 0.079 P (never under
  1.5 virtual px at 1080p, 2.25 at 720p / 1080i / 480p, 4 at 480i / 576i), round tip. Hub radius = 1.125 x
  half-width + 1.5 px. Dial face radius 0.459 P.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | clock pitch 56 .. 267 virtual px, exponential, glided (default 120: 16 x 9 clocks) |
| K2 | Colour | the 16 shared palettes with hysteresis (was K6): ground = wall, A = hand 1, B = hand 2, accent = hub; face = 7/8 ground + 1/8 A |
| K3 | Depth | swing 0 .. 1.25 turns, square law (default 1/4 turn each way) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Hands | hand 2 against hand 1: 0 % locked in one line .. 50 % mirrored .. 100 % counter-running at 3x (glided) |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: the source term = luma x 0 .. 80 (Osc x2 that); 25 % = the v0.1.0 gain (glided, +-4 deadband) |
| S7 | Wave | Smooth / Snap (hands tick between the 8 compass points) |
| S8 | Field | Linear (diagonal wave) / Radial (octagonal ripple from the centre clock) |
| S9 | Dial | Faces (dial discs drawn) / Hands (hands and hubs alone on the wall) |
| S10 | Grid | Square / Brick (alternate rows offset half a clock) |
| S11 | Source | Video / Osc |
| P12 | Spread | phase step per clock 0 .. 1/2 cycle, square law, glided: 0 % all clocks in unison, a little = one slow wave across the wall, 100 % = neighbours in antiphase (standing chequer / ring figures) |

Presets: Scissor Wave (defaults), Line Field, Ripple Wall, Semaphore, Standing Figs, Portrait Hands.

## Video

- Source = Video: the luma (>> 2, less 16) is smeared along x by a 1/64 one-pole filter and HELD per clock:
  on the last line of each row of clocks the smeared value at each clock's right edge is written into the
  other bank of a 2 x 256 block RAM (banks by row parity) and the row below reads it for all of its lines, so
  every clock is rigid. hand 1 += K6 gain x luma (v0.1.1: x40 at the default; v0.1.0 was a fixed x20, white
  0.27 turn), so the picture shows as hand directions.
- Black input: the offset is zero and the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  twice the camera's gain.

## Engine

Sections in horologe.vhd: [A] timing ID -> virtual-1080 steps, vblank zone, glides (Scale, Spread, Hands),
per-field sequencer with one 24-step serial multiplier (9 products), palette read, the three grid-origin
searches (rows, columns, brick-shifted columns) and the blend-table fill; [B] generated avid; [C] source
field and hold; [D] wave engine; [E] the clocks; [F] output conditioning (lifted from louver).

- Per field: all knob-derived constants, the 16 x 9-step blend table (under layer, top layer, coverage), the
  phase step, the grid origins.
- Per line: the row accumulators (offset in the row, row number, Spread x row), the line's y^2 (read from the
  squares ROM in the blanking).
- Per clock and line: a serial machine, 25 clocks long, started at each clock's first pixel (and once before
  each line for the line's first clock): fetch |cos|, |sin| of both hands from a 512-step quarter-turn ROM,
  then 9 + 9 add-shift steps build the seeds of two affine forms per hand, along' = |sin| x -+ |cos| y and
  across' = |cos| x +- |sin| y (Q11 px), with the magnitude bit as the adders' clock enable.
- Per pixel: no multipliers. The four forms advance by one add each; distance to the hand = |across| along
  it and max + 3/8 min of (along - length, |across|) past the tip; coverage in eighths over a ramp of
  1 / 2 / 4 virtual px (by the raster's line pitch). Dial face and hub from r^2 = x^2 + y^2 (squares ROM)
  against two per-field thresholds, anti-aliased by a per-field shift. Colour = one read of the blend table:
  the top layer (hub > hand 1 > hand 2 > face) over the highest layer under it that covers at least half.
  Luma 10 bits, chroma 8 bits until the output filter.
- One latency in every mode: C_LAT = 57 = processing_delay_clks.
- Every multiply is a sum of gated operands or serial; block-RAM data gets a fabric register before
  arithmetic; no RAM reads the address it is writing (no_rw_check on all of them).

## Status

- Built with router2 through `./build_set.sh kinetic horologe` (rebuild command), 2026-11-14.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 2 | 77.12 MHz | 6937 |
| SD Analog | 1 | 74.04 MHz | 6959 |
| HD HDMI | 4 | 77.74 MHz | 6960 |
| SD HDMI | 1 | 67.75 MHz | 6953 |
| HD Dual | 3 | 81.43 MHz | 6945 |
| SD Dual | 1 | 77.24 MHz | 6972 |

- 6,972 LC at most (90.8 %), 18 of 32 block RAMs. Never flashed.
- Timing is seed-marginal on router2: HD Analog missed on seed 1, HD HDMI on seeds 1 .. 3, HD Dual on seeds
  1 and 2 (the SD configs only need 27 MHz). Record: seeds 2 / 4 / 3 for the three HD configs. router1 was
  not tried.
- GHDL render = integer model (.kinetic_work/rb_horologe/imodel.py), 0 mismatching pixels in each of:
  the full 720p frame at the defaults with a black input (918,720 px); 640 x 360 windows of the 720p raster
  (decimation 2) at the defaults, with Snap + Radial + Brick and Scale / Shape / Depth / Hands / Colour /
  Spread all moved (black input), and with Source = Osc at the defaults (the whole hold path).
- With selfie.jpg as the input 5 % of the pixels differ: the model resamples the photo itself, so its input
  luma is not bit-identical to the tester's; the hold path is the one the Osc render matches exactly.
- Also 0 mismatching pixels at 1080p timing (a 480 x 270 window, decimation 4, defaults) and in both fields of
  an NTSC 480i capture at the defaults (the second field one phase step on: the motion check). SD shows the
  centre 4:3 of the wall, 12 x 9 round clocks at the default Scale. 1080i was not rendered.
- Single-row hard line check on the model (runs of 24+ px one row tall): none at 1080p or 720p in 9 settings;
  at 480i, two runs of 24 px in two of the nine (Shape 0 with the hands lying horizontal).

## Cut / deviations

- The dial is a filled faint face disc, not a ring: one edge instead of two, and a far bolder Dial switch.
- Snap is the spec's "tick between 8 positions", not the set's dwell-at-the-extremes wave: the ANGLE is
  quantised (so a picture on Source is drawn in 8 directions too). The move between positions is linear, not
  eased.
- Tip caps use the octagonal metric max + 3/8 min (up to 7 % from a true circle: under 1 px at every size).
  The hands are cut square at the centre under the hub (the hub covers the cut).
- Where two anti-aliased edges cross, the blend takes the top layer over ONE under layer (the highest that
  covers at least half), not a full three-colour mix.
- The r^2 anti-alias ramp is a power of two of the radius, so its width varies between about 0.7 and 1.4 px
  across Scale.
- Minimum clock pitch is 56 virtual px (the per-clock machine needs 25 clocks: 28 pixels on SD).
- Spread per clock is quantised to 1/16384 cycle (the bottom 1 % of P12 is unison).
- Hands (K5) runs to -3x, in 128 steps of 1/32.
- Source: the top row of clocks reads a sample taken at the bottom of the previous field (rigid, but from the
  wrong place); in Brick the leftmost partial clock of even rows can get no sample (it stays at its last
  value). The sample is taken at each clock's top edge, half a clock above its centre, and the smear is 64
  real pixels at every resolution.
- SD / 720p: thin needles are thicker than at 1080p (the minimum weight follows the line pitch so a
  horizontal needle never sits on one scan line), and edges are softer. Same layout otherwise.
- First RTL (8 parallel serial products + a combine stage, 28-bit spread accumulators, pitch from 40 px) was
  7,421 LC (96 %) and missed timing on router2; the machine was rebuilt around 4 accumulators.

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now Camera: how hard the held camera luma turns the hands.

- Gain law (linear): the source term (Q14 turns) = luma x 5/16 K6 / 4, i.e. x0 at 0 %, x20 at 25 % (exactly the
  v0.1.0 fixed gain: white = 0.27 turn), x40 at 50 % (default, white = 0.53 turn) and x80 at 100 % (white = 1.07
  turns, so bright areas spin the hands past a full turn). Source = Osc keeps its x2: x80 at the default.
  0 % draws exactly what a black input draws.
- Twice the v0.1.0 gain stays clean (every clock is rigid by construction, so there are no per-line jaggies), so
  50 % is the full 2x.
- K6 is read once per field through the shared glide (a fourth slot: 1/8 of the gap per field, +-4 deadband) and
  turned into the gain in the vblank sequencer (zc 80: K6/4 + K6/16, x2 for Osc). The gain is applied where the
  held sample is READ, through a 256-entry gain table (one block RAM) refilled every field in the vblank zone by
  repeated addition (zc 768 .. 1023, address = zc's low byte: gt(n) = n x gain / 16, one 18-bit add per entry):
  no multiplier at all. The hold RAM's read data (stage 9) addresses the table directly and the table's output
  (stage 10) takes the place of v0.1.0's fabric copy of the hold, so the latency, the hold (512 x 8) and its
  sampling are all unchanged; stage 11 is a plain x4. K6 changes reach every clock at once. The hand-1 sum is one
  bit wider (17 bits, up to 3.6 turns) so hand 2 never sees a wrap. Shape's two per-field fractions are now
  constants (hand length 370 / 1024 P, half-width 46 / 1024 P), which frees their registers.
- Fit history: the first K6 RTL (a gated-sum 8 x 10 multiply on the hold's write path, as on cogwork / pendula,
  7,167 LC) and a second (the gain table on the write path with a 16-bit hold, 7,033 - 7,058 LC, 20 block RAMs)
  both stopped nextpnr's placer at its first legalisation ("Unable to find legal placement for all cells", router1
  and router2 alike, seeds 1 and 2), although v0.1.0 (6,937 LC, 18 RAMs) places. The design has ~220 flip-flop
  control sets (many 1 - 7 FF sets from the per-field sequencer and gated sums), so its real fill is higher than
  the LC count says. The read-side table (7,024 LC, 19 RAMs) placed HD Analog but not HD HDMI (6,994 LC).
- To get back under v0.1.0's fill, the generated-avid delay line was shortened: taps 40 and 47 now ride the cvr
  delay ring's spare bits (written as taps 1 and 8; 39 clocks to the fabric copy) and taps 48 .. 56 follow tap
  47 in flip-flops, so only taps 0 .. 8 remain a flip-flop chain (37 fewer flip-flops). It is bit-identical (the
  K6 50 % render is 0 pixels different) and places on seed 1 in all three HD configs (placement-only probes:
  6,951 - 6,961 LC).
- Frozen: K2 Shape at 450 (its default: hand length 0.43 - 0.07 P, half-width 0.018 + 0.027 P) - the C_KFIX_2
  constant. K2 is now Colour (all 16 palettes, unchanged). Shape was the assigned knob; the v0.1.0 sheet shows no
  clearly weaker one (Rate is the only speed control; Scale, Depth and Hands each change the figure more).
- Presets: the Shape values in Line Field (200), Semaphore (800), Standing Figs (300) and Portrait Hands (350) are
  gone (they all draw at 450 now); their colours moved from K6 to K2; Camera is at its 50 % default in all of them.
- GHDL look (720p60, decimation 2, selfie.jpg, glides bypassed in a sim-only copy), final RTL:
  .kinetic_work/rb_horologe/k6/k6sheet.png (K6 0 / 25 / 50 / 100 %, Osc at 50 %, v0.1.0 Osc). The multiply
  version's renders are in k6/mult/ (same gain law; its sample was taken 2 px earlier, so a few clocks differ by a
  small angle).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic horologe`): router1, every config on seed 1.
  Post-route Fmax HD Analog 81.80, SD Analog 78.35, HD HDMI 80.33, SD HDMI 78.78, HD Dual 78.17, SD Dual 72.06 MHz
  (worst HD 78.17; v0.1.0's router1 build was 74.43). 6,978 LC at most (90.9 %, +6 over v0.1.0), 19 of 32 block
  RAMs (+1: the gain table). vmprog: out/rev_b/ron/kinetic/horologe.vmprog. Not flashed.
- Unsure on hardware: whether 100 % (white spins a clock 1.07 turns, so the picture no longer maps one way onto the
  hand angles) reads as the climax Ron wants, and whether a noisy camera makes the hands jitter more at 2x; the
  hold has no smoothing along y or time.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron, after the fern / kiln K6 = 100 % renders: a slightly stronger camera effect at the top of the knob.
The orchestrator set the K6 default to 100 % (1023) and bumped the version; this pass changed the logic.

- Classification: **element** (K6 scales the held luma that turns the hands: it moves the drawing, it never
  shows the camera image).
- Change: the table's top extended. A new sequencer slot (zc 82, before the gain table fill at zc 768 .. 1023)
  scales the per-field gain by 1.25 (c_cg += c_cg >> 2): Video = luma x 0 .. 100 per step (was 0 .. 80; 50 % =
  x50), Osc x2 that (max gain 795 of 1023). The table's top word is now 255 x 795 / 16 = 12,670 (14-bit word,
  18-bit fill sum: both fit). The hand-1 sum grew from 17 to 18 bits and hand 1 from 14 to 15 bits (Q11 turns),
  since Osc white plus full Depth now reaches 4.3 turns and a wrap would jump hand 2 (Hands multiplies hand 1);
  Snap's compass index is 7 bits. At 100 % white turns a clock 1.33 turns (was 1.07).
- Look (GHDL, bear.png, 720p60 / 4, glides bypassed in the scratch copies): `.kinetic_work/rb_horologe/k6b/
  k6b_cmp.png`. Same clock wall; the hands sit at different angles (more turn); every clock stays rigid. No wrap
  or new artifacts.
- Build (`.kinetic_work/tools/k6build.sh kinetic horologe`): router1, every config on seed 1 (it placed first
  time). Post-route Fmax HD Analog 79.83, HD HDMI 76.38, HD Dual 77.16, SD Analog 74.74, SD HDMI 80.33, SD Dual
  82.60 MHz (worst HD 76.38). 7,014 LC at most (91.3 %, +36), 19 / 32 block RAMs. vmprog:
  out/rev_b/ron/kinetic/horologe.vmprog. Not flashed.
