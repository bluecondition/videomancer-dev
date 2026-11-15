# Skyline — a sample-and-hold bar graph that dances like a spectrum analyser

**v0.1.1 built 2026-11-15 (K6 camera pass), not flashed.** Patchbay. Columns of LED segments in a green / yellow / red ladder
dance to slow noise (or to the picture), each with a peak cap floating above it and a dim rippled reflection
below a violet horizon line. Presets are hand-written placeholders.

## The patch

    noise (LFSR) --S&H clock (Speed)--> one random level per column
        x  LFO 1 + LFO 2: two sines across the columns, drifting at different rates, tilted   (Shape = Spectrum)
        or LFO W: a travelling sine, two cycles across, plus an eighth of the noise           (Shape = Wave)
    Source: camera luma (or the Osc lattice) -> mean of each column over the field -> AC-coupled -> x4
    mixer: shaped noise + source + Level (DC) -> clip at 0 -> + bias (the top quarter of Drive) -> x Drive
        -> H-rate sample-and-hold: one held value per column, drooping a little across the column
        -> envelope follower (attack: half the gap per field; release: Decay)   = the bar
        -> peak-hold (hold, then a fall under gravity)                          = the cap
    V ramp from the horizon, mirrored below it (twice as steep)  vs  the bar -> comparator = the bar key
    V ramp + a little H ramp + heat (how far the mixer overdrives the S&H) -> colouriser (the ladder wheel)
    bar key -> VCA: lit / dim ladder / reflection (fading with depth, rippled)
    peak key -> the colouriser's complement (128 steps round the wheel), lifted toward white  = cap / trail

Every element of the picture is one line of that sheet: the columns are the S&H, the LED segments are the V
ramp quantised before the comparator, the ladder colours are the colouriser, the caps are the peak-hold, the
reflection is the V ramp folded at the horizon, the hot tops are the overdrive.

## What it draws

(virtual 1080-line coordinates; "lines" are virtual lines)
- **Layout.** The horizon is at line 780. hy = 780 − y above it and 2·(y − 780) below it, so the reflection is
  the same bars squeezed to half height. Full scale is 640 lines, caps reach 700, nothing is drawn above 720.
  A 10-line violet horizon line. Mirror = Centre puts the axis at 540 with hy = 1.375·|y − 540| both ways.
- **Columns.** N columns span the real width (a count: they narrow on 4:3). A quarter of each column is the
  gap. The held level droops up to 7 lines across a column and the lit level shades from 16/16 to 11/16
  across it, so each bar looks like a glowing tube rather than a flat rectangle.
- **Segments.** The V ramp is cut into LEDs of the K2 pitch; a quarter of the pitch is the gap. An LED lights
  when the bar passes its middle and takes one colour (the colouriser reads the LED's middle). The cap is one
  whole LED. Solid (top of K2): continuous bars, the cap a 12-line slab sitting on the drooping bar top.
- **Colouriser.** A 256-step cyclic wheel of flat bands with thin fringe bands between them: green, lime
  fringe, yellow to amber, pale fringe, red, amber fringe, hot pink, white hot, violet, pink fringe, blue, ice
  fringe, cyan, teal. Index = height × 0.164 + x / 128 (the H ramp leak: band edges step down a little across
  the screen) + 8 × heat, saturating at white hot, then + the Colour rotation. Heat is how far the mixer
  exceeds full scale (0..15, fast attack, decaying a step per field): overdriven columns turn hot from red
  through pink to white, and their caps rise up to 60 lines above full scale.
- **Levels.** Lit 16/16 above the horizon; the unlit ladder shows at 2/16 (the dim grid behind everything);
  the reflection starts at 11/16 and loses 1/16 every 32 lines of depth, with ripple bands (−2/16, one LED or
  16 lines tall) that step every 4 fields.
- **Motion.** At the defaults each column is resampled about 3.6 times a second, bars jump up in two or three
  fields and fall 12 lines a field, caps hold 0.7 s and then drop under gravity, and the two formant humps
  drift across in about 17 s and 9 s in opposite directions.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Columns | S&H clock (H rate) | 8 .. 96 columns across the width (default 34) |
| K2 Segments | V ramp quantiser | LED pitch 32 lines (20 LEDs) .. 283 lines (2 LEDs); the top eighth of the travel = solid bars. Default 60 lines |
| K3 Level | mixer DC offset | bipolar: −0.44 (only the tallest humps poke up: sparse towers) .. 0 (512) .. +0.44 (a high wall with a dancing top) |
| K4 Speed | S&H clock + LFO rates | bipolar, ±8 deadband, square law. Magnitude: resample chance per column per field 1.2 % .. 50 %, LFO drift ×1 .. ×30. Sign: which way the humps and the wave travel. Never fully still |
| K5 Colour | colouriser rotation | bipolar: −100 = violet / blue / cyan ladder with warm caps .. 0 = green / yellow / red (512) .. +100 = pink / white / violet / blue with green caps (K6 until v0.1.0) |
| K6 Camera | source gain (camera or Osc) | 0 % = no source (the pure noise analyser, as with a black input) .. 25 % = v0.1.0's fixed x4 .. 50 % (default) = x8 .. 100 % = x16. Decay (K5 until v0.1.0) is fixed at its old default 600: release 12 lines a field, caps hold 0.7 s |
| S7 Shape | noise shaping | Spectrum (noise under two drifting formant humps) / Wave (a clean travelling sine, two cycles across, lightly noised) |
| S8 Mirror | V ramp fold | Reflection (horizon at 72 % height, dim mirror below) / Centre (bars grow both ways from the mid-line, both halves bright) |
| S9 Peaks | peak key | Caps (one LED floating at the peak) / Trails (the whole span from the bar top to the peak glows in the cap colours, brighter with depth) |
| S10 Gap | column gate | LEDs (a quarter-column gap) / Solid (columns fused into one skyline) |
| S11 Source | mixer input | Video (the picture's column profile) / Osc (the triangle lattice) |
| P12 DRIVE | mixer gain | square law 0.3 .. 6.3 (1.8 at the default 50 %); the top quarter adds a DC bias. 0 % = a murmur of one or two LEDs under a row of resting caps; 100 % = every column pinned, tops flickering red / pink / white, caps bouncing between full scale and the ceiling |

## Video

- **Video (default).** Camera luma, (Y >> 2) − 16, is summed along each row while the beam is inside a
  column and added to that column's cell in a block RAM on 128 rows of the field (every 8th virtual line), so
  each column ends the field with its mean luma. The sequencer scales the sums to means, subtracts the mean of
  all columns (AC coupling, like a capacitor on the input) and feeds 4× the difference into the mixer. The
  bars become a live profile of the picture: columns brighter than the picture's average rise, darker ones
  sink, and the noise still dances on top. Being one value per column per field it cannot shred anything.
  With a black (or flat) input the term is zero and the patch is the pure noise analyser.
- **Osc.** The shell's 640 × 540 px triangle lattice, drifting about 75 px/s, goes into the same accumulators.
  Its vertical triangle averages out and the horizontal one survives: a three-tower triangle skyline about
  ±0.87 of full scale that slides across the screen, clipping hot at its peaks. That is 2–4× what a camera
  picture gives.

## Engine

- **Per field (vblank zone, one serial shift-add multiplier, shared glide for K1 K2 K3 K5 K6 P12):** column
  count, column phase step (count × 2^22 / width by timing ID), Speed², (1 − Decay)², Drive², the colour
  rotation, pitch, gap thresholds, line steps. All square laws; no exp ROM.
- **Column engine (zone, from clock 512; 64 clocks per column, at most 6,144 clocks):** reads the column's two
  state words and its video sum, two sine lookups, resamples the noise from a 32-bit LFSR, three serial
  products (sum → mean, noise × envelope, mixer × Drive), then the envelope follower, heat and peak-hold, and
  writes the state and display words. Started by the zone only (a field start); the field mean for the AC
  coupling is one more product at the end.
- **Per line (hblank, 6 clocks):** hy steps down to the axis and up again beyond it (Q8), its LED phase wraps,
  and the line's constants are latched: three comparator levels, the ladder index, gap / ceiling / horizon
  flags, the reflection level. The segment gap is a quarter of the pitch but never under 2.25 raster rows.
- **Per pixel (13 stages; the beam starts two clocks after the generated avid so they meet the shell's stage 15):** column phase accumulator (7.22) → display RAM →
  fabric copy → droop / index / shade → three comparators against the line's levels → candidate levels →
  keyer → colouriser ROM (EBR, fabric copy) → VCA (level 0..16 as gated shift-adds) → codes.
- **Video accumulator:** row sum per column, read-modify-write (read, fabric copy, add, write: 5 clocks; a
  column is at least 7 px) on the sample rows.
- **Memories:** colouriser 512 × 32 (4 EBR), sine + reciprocal 512 × 16 (2), display 128 × 32 (2), state
  256 × 32 (2), video sums 128 × 32 (2): 12 EBR, all `no_rw_check`.
- **Contracts:** generated avid, one latency (C_LAT 20 = processing_delay_clks) in every mode, blanking gate,
  Y / U / V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16, virtual 1080-line
  coordinates, colours in the hardware convention (U = Cr).

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now **Camera**, the strength of the source term in the mixer (the camera's column profile, or the
Osc lattice when S11 = Osc), 0 .. 100 %, default 50 %.

- **Law.** The glided K6 (k, 0 .. 1023; deadband +-4 codes, ease 1/8 per field, the shared glide's channel 3,
  which used to carry Decay) scales the column-sum-to-mean factor once per field on the sequencer's serial
  multiplier (zone slot 3, which used to square Decay): `c_vsc = c_inc(19:8) x k / 256`. The column means, the
  field mean and so the AC-coupled source term all scale by k / 256; the source then goes through the old x4
  and the old +-2047 clip. So the source gain is linear in K6, x(k / 64):
  - 0 % = no source (the pure noise analyser, exactly the black-input picture);
  - 25 % (k 256) = v0.1.0's fixed x4;
  - 50 % (default) = x8: the picture's profile clearly leads the bars (the bright columns stand tall and hot,
    the dark ones sink to the dim ladder) while the noise still dances on the rest;
  - 100 % = x16: nearly a two-level skyline of the picture, bright columns pinned hot with caps at the ceiling,
    dark columns empty. Never washed out: one value per column per field, it cannot shred.
  The mean / field-mean path widened from 10 to 12 bits (e_m, c_vmean, e_sr 14-bit) so 4x the mean does not
  clip before the AC coupling.
- **Osc** goes through the same accumulators, so it keeps its 2-4x extra level and scales with K6 too: at 50 %
  two (SD) or three (HD) towers pinned hot with empty gaps.
- **K5 Decay frozen** at its default 600 (release 190 Q4 = 12 lines a field, hold 43 fields = 0.7 s, gravity 25:
  constants now). Decay was the weakest knob on the sheet (it only changes the motion; the stills barely
  differ). **K6's old function Colour moved to K5** (same bipolar rotation, default 512). Presets: old K6 values
  moved to K5, old K5 (Decay) values dropped (Pinned 420, Night City 860).
- **Sims** (`.patchbay_work/rb_skyline/k6/`): the program draws at its timing ID's scale, so a decimated sim
  shows only a quarter of the picture and starves the AC coupling (the unseen columns read black); the K6 sims
  are NTSC at decimation 1 instead. Sim-only copy: glide bypassed, Decay 0 (so the bars reach their heights in
  six fields) and the start-up kick suppressed. `k6sheet.png` = K6 0 / 25 / 50 / 100 % and Osc at 50 %.
- **Build (v0.1.1, router1, all six configs on seed 1).** HD Analog 99.03, SD Analog 99.48, HD HDMI 102.70,
  SD HDMI 96.10, HD Dual 99.41, SD Dual 99.44 MHz; 6,697 - 6,725 / 7,680 LC (88 %), 12 / 32 EBR. Worst HD 99.03.
  `out/rev_b/ron/patchbay/skyline.vmprog` (436,979 bytes).

## Status

- **v0.1.0 built 2026-11-14 with router2. NOT flashed.** `out/rev_b/ron/patchbay/skyline.vmprog` (443,347 bytes,
  six bitstreams, iceunpack OK x6; the packed config carries the labels and presets).
- Rebuild: `./build_set.sh patchbay skyline` (the build ran as
  `MAKE_TIMEOUT=480 .kinetic_work/tools/build_queue.sh patchbay skyline`).
- Post-route Fmax and seeds:

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 83.93 | 6,699 |
  | SD Analog | 1 | 89.29 | 6,687 |
  | HD HDMI | 1 | 85.94 | 6,696 |
  | SD HDMI | 1 | 65.21 (Fmin 27) | 6,674 |
  | HD Dual | 1 | 88.37 | 6,705 |
  | SD Dual | **2** | 86.37 | 6,712 |

  SD Dual seed 1 stalled router2 (the builder's timeout), seed 2 routed in a minute. 6,674-6,712 / 7,680 LC
  (87 %), 12 / 32 EBR. No transparency emulation on any RAM; `dupin.py` reports no shared-operand carries.
- **Build history worth knowing.** A first version of the RTL was 7,046 LC (91 %): nextpnr's placer legaliser
  hung on it for some (config, seed) pairs (HD Analog seed 1, HD HDMI seed 2: the first placer iteration took
  111 s against the usual 0.5 s and the second never returned), while HD Analog seed 2 placed in 15 s and
  routed at 78.4 MHz. The RTL was then slimmed with no change of behaviour (state words read straight from
  the RAM, the engine's intermediate terms free-running instead of step-enabled, the VCA in one stage with the
  beam started two clocks later, derived constants as wiring): 6,699 LC, and every config placed at once. If a
  later version grows back past about 90 % and a build sits in "[n/6]" for minutes, that is the symptom.
- **Verification (GHDL, `sim2.sh` / `diff.py` in the work dir).** The integer model `imodel.py` and the RTL
  agree pixel for pixel (0 differing pixels of Y, U, V; the RTL raster sits 4 px right in the tester's
  capture, the generated avid's offset):
  - 720p60, defaults, black input: fields 4 and 8 (the built RTL; 11 % of the pixels change between them);
  - 720p60, field 4 of two knob sets mid-glide: (Wave, Centre, Trails, Solid gap, K2 1023, K3 700, K6 800,
    P12 900) and (K1 900, K2 0, K4 100, K5 200, Trails, P12 1023) (the RTL before the slimming; the slimmed
    RTL's default frame is byte-identical to that version's);
  - NTSC 480i, defaults: fields 11 and 12;
  - NTSC 480i, K2 1023 (solid bars), P12 700: fields 19 and 20.
  With a source the model is approximate (it does not model the tester's image scaling or the accumulator's
  few pixels of lag): Source Osc differs on 5.3 % of the pixels at field 8, selfie.jpg on 0.85 %.
  `cmp_final.png`: prototype, model, RTL black / Osc / selfie at 8 fields, and the model's steady-state Osc and
  selfie frames. `sheet.png` (controls) and `strip.png` (8 frames over 3.5 s) are from the integer model.
  - 1080i59.94, defaults: fields 7 and 8 (the same picture on both fields; the program does not use field_n).
- `sweep.py`: every per-field, per-column and per-line value stays inside its RTL width over the knob
  extremes in all five rasters. `thincheck.py`: see Cut / deviations.
- Presets are hand-written placeholders: Analyser, Pinned, Night City, Wave Trails, Centre Hot, Big Blocks.

## Cut / deviations

- **Source.** The spec's Video mode made each column's height the picture's mean luma alone. Here the camera is
  mixed into the noise path (AC-coupled, ×4) so a black input still gives the full dancing picture, as the
  set's brief requires. Osc is the shell's triangle lattice through the same accumulators, not "the internal
  noise" (the noise is always in the patch).
- **K3 Level** is the mixer's DC offset (bipolar), so it stays distinct from P12 Drive (the gain).
- **K6 Colour** is bipolar with the classic ladder at the centre, so both ends are rotated looks.
- **Caps bounce** at 100 % by being re-kicked to a new height by each resample (60 lines of travel above full
  scale), not by a physical rebound.
- **Resolutions.** The same picture at 1080p, 720p and SD: heights are virtual lines, the columns span the
  width (a count, so they are 3/4 as wide against their height on 4:3). Differences to expect:
  - the horizontal softness is per output pixel, so 96 columns at SD (7.5 px each) nearly fuse; the column gaps
    read clearly up to about 48 columns at SD and 96 at 1080p;
  - on SD (480i / 576i) the segment gap is widened to 2.25 field rows (10 lines instead of 8 at the finest
    pitch), and the reflection drops its segment gaps below a 40-line pitch (it cannot show them in under
    two rows); the solid cap is 20 lines thick in the SD reflection;
  - the H ramp leak into the colouriser uses the centre 4:3 of the ramp on SD.
- **Thin lines.** The model's single-row check (`thincheck.py`) is clean in LED mode at 1080p, 720p and NTSC.
  In solid mode a cap closing onto its bar passes through a one-row gap, and a reflected bar tip can sit one
  row past a ripple band (2/16 contrast); both are transient.
- **Start-up / cut kick.** The AC coupling subtracts LAST field's mean, so for one field after the program
  loads with a bright source, after Source is switched, or after a hard cut in the picture, the whole mean
  goes into the mixer: every cap jumps (to the ceiling for a bright picture) and falls back after its hold.
  It reads as the meter reacting to the cut; if it annoys on hardware, the fix is to hold the source term at
  zero for a field after the mean moves by more than a threshold.
- The first 8 px at the left edge fade in from black (the shell's filter warm-up).
- **Unsure on hardware:** the video accumulator's read-modify-write runs at pixel rate with `no_rw_check`
  RAMs (reads and writes are at least 5 clocks apart by construction, but only GHDL has seen it); the
  heat-driven colour flicker at Drive 100 % may be harsher on a real display than in stills.
