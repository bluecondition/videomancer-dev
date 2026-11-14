# Pulsar — stacked scope traces with hidden lines (the PSR B1919+21 ridgeline plot)

**v0.1.0 built 2026-11-14, not flashed.** Patchbay set. A patch that was never on the rack: a complex H-rate
waveform under a burst envelope, one comparator per trace against the V ramp, a priority chain so nearer traces
hide farther ones, and an edge differentiator that draws each trace as a bright line over its black body.
Presets are hand-written placeholders.

## The patch

    H ramp -> VCO 1 (sine, 300 px) --\
    H ramp -> VCO 2 (sine, 113 px) ---+-> mixer (5/8 + 3/8) -> waveshaper (cube) --\
    noise -> AND gate -> slew (1/2) ------------------------ crossfader (K5 Rough) --> VCA 1 <-- burst envelope
    H ramp -> window -> burst envelope (cosine bump^1.5; K3 width, S8 One / Twin, a slow sway LFO on its centre)
    VCA 1 x P12 BURST  +  camera luma along the trace's own row, slewed (S11; Osc = a triangle lattice)
                                                                 -> VCA 2 (K2 Height) = the trace's height
    V ramp + trace baselines (K1 Lines, S9 Tilt, K4 Speed travel) -> one comparator per trace (7 in parallel)
    comparators -> priority chain (nearest wins: hidden lines) -> edge differentiator (the line) / S7 Glow ramp
    height above the baseline -> colouriser (K6 Colour: by height .. by trace) -> S10 Paper inverter

Every trace has its own identity: VCO 1 phase (0.137 cycle per trace), VCO 2 phase (-0.274), noise seed, sway
phase and colour offset all come from one identity accumulator, so no two traces match. The identities ride
the baselines down the screen (K4), and the VCO phases and the sway also drift on their own (three rates), so
the peaks morph while they travel.

## What it draws

- Black ground, 16 to 83 traces stacked down the screen (43 at the defaults). Outside the burst each trace is a
  flat ruled line; inside it the waveform lifts the line into peaks up to 4.7 pitches tall (6 at full Height),
  so the peaks overlap the traces above and hide them: the nearest trace that covers a pixel wins.
- Lines are 2.5 virtual rows thick, plus 3 pixels of horizontal weight on slopes so steep flanks keep their
  stroke; their top and bottom edges are anti-aliased (16 coverage levels).
- Colour by height above the trace's own baseline, in units of the local pitch: blue baselines, violet and
  magenta flanks, red-orange, yellow, white-hot above 3 pitches.
- The burst column sways slowly; everything travels toward the viewer at about one trace a second.

## Controls

| Control | Module it belongs to |
|---|---|
| K1 Lines | V ramp / trace baselines: 16 .. 83 traces (exponential), default 43 |
| K2 Height | VCA 2: the tallest peak, 1.06 .. 6 pitches (exponential, 2.5 octaves); default 4.7. The video relief scales with it |
| K3 Envelope | burst envelope width: half-width 110 .. 3,500 virtual px (exponential): a narrow central ridge .. the full line; default 470 px |
| K4 Speed | travel of the baselines: bipolar, +-8 deadband, square law; right = toward the viewer; up to 26 traces/s with BURST at 100 % |
| K5 Rough | crossfader: VCO swells (0) .. jagged noise (100 %), in 9 steps; default half |
| K6 Colour | colouriser: white (0) .. pale .. full colour by height (38 .. 59 %, default) .. blended .. colour by trace (97 %+): a slow rainbow that rides the traces |
| S7 Fill | Black (hidden line) / Glow: a ramp under each trace, 10/16 at the line fading out over one pitch, in the colour of the height |
| S8 Burst | burst envelope: One bump / Twin (two bumps, each half as wide, side by side) |
| S9 Tilt | trace baselines: Flat (even pitch) / Perspective: the pitch grows by a constant ratio per trace below a horizon above the screen (1 : 6 from top to bottom), and the heights with it |
| S10 Paper | output inverter: Off / On: dark inks on warm paper (the colouriser becomes ink colours) |
| S11 Source | Video / Osc |
| P12 BURST | VCA 1 level, (x)^1.25: 0 % = flat ruled lines (with a black input) .. 100 % = towering overlapping peaks; it also multiplies the travel by up to 3.4, so 100 % races forward. Default 76 % |

## Video

- **Video (the default).** Camera luma is low-passed along the line and sampled into a 64 x 30 store (one row
  per 16 virtual rows, one column per 64 virtual px). Each trace reads the store row under its own baseline
  (the previous field's picture), slews it along the trace (a one-pole lag, so edges become smooth arcs, never
  ragged) and adds it to its height at 3/8 of the window: the picture stands up as a relief behind and through
  the burst, and lifts the colours with it. With a black input the patch is unchanged.
- **Osc.** The shell's drifting triangle lattice (640 x 540 virtual px) is written into the same store and
  enters at twice the camera's level: the ruled lines swell into drifting diamonds.
- The video term is not scaled by BURST: at 0 % a camera picture is drawn as a pure relief.

## Engine

- **Everything is octaves.** Gains are attenuations in 1/1024 octave; EXPU(t) = 32768 x 2^(-t/1024) is the
  256-entry exp2 ROM plus a shift (4 clocks), L[] a 256-entry log ROM. Heights are
  EXPU(L[waveform] + L[envelope] + C) + EXPU(L[video] + CV), clipped to the trace's window, where C adds up
  Height, BURST and the trace's pitch. So neither the fill engine nor the trace set-up has a multiplier; one
  small serial multiplier does two products a field.
- **Per field (vblank zone, about 100 steps):** shared glide of K1, K2, K3, K5, K6, P12; knob exponents by
  shift-adds; EXPU look-ups for the octaves per trace (Tilt), the envelope step and the travel; the travel
  phase fz (wraps move the identity accumulator by one trace); fz x octaves and pitch x (1 - fz) on the serial
  multiplier give the first trace's exponent and baseline.
- **Per trace (10 steps + a fill, on demand):** pitch = 4 EXPU(A), baseline += pitch, A -= octaves per trace;
  10240 / pitch and the window by two more look-ups; phases, sway (sine ROM), noise seed and colour offset from
  the identity accumulator. The fill engine then writes the trace: 256 samples 8 output pixels apart (128 at
  SD), two clocks each, 20 stages, through one sine ROM (both VCOs), the shaper ROM, the log ROM (waveform,
  then video), the envelope ROM and the EXPU. A bank holds the first sample and the 240 slopes between samples.
  Eight traces are made in vblank (about 4,600 clocks), then one each time the beam passes a baseline.
- **Per line (line engine, about 150 clocks, computing the next line):** for each of the 8 banks, from a small
  constant file in block RAM: X = row + R - baseline (written to the bank's word 255), the zone advance, and
  the colour index = offset + rows above the baseline x slope (one small serial multiplier).
- **Per pixel:** in hblank each bank's accumulator is preset to 8 (X + first sample) by reading two words for 8
  clocks each; during the line it adds the cell's slope every pixel, so e = height - rows to the baseline, exact.
  Seven of the eight banks are live (the past trace and six coming ones); the nearest with e > -R wins
  (rotating priority), and its e gives the coverage min(e, T - e), the glow ramp and, through the palette ROM
  (colour minus ground, 4 saturations), the colour; level x colour is two gated-add stages.
- **Shell:** virtual 1080-line coordinates from the timing ID, vblank zone, generated avid, one latency
  (C_LAT 24), output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums of 8-bit samples
  with the per-line resync, luma 13/16), blanking gate. The chroma sums' 8-back taps and the sync delay ride
  two small block-RAM delays. Colours in the hardware convention (u = Cr, v = Cb).
- **Block RAM (22):** 8 trace banks (256 x 16), luma store (2048 x 8, 4), palette (2), exp2, log, envelope,
  sine, shaper, trace-constant file, chroma delay, sync delay.

## Status

- **v0.1.0 built 2026-11-14 with ROUTER1, not flashed.** Rebuild:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay pulsar`.
- All six configs on seed 1. Post-route Fmax: HD Analog 92.95, SD Analog 80.16, HD HDMI 89.99, SD HDMI 90.65,
  HD Dual 86.77, SD Dual 90.82 MHz. 6,891 .. 6,911 / 7,680 LC (90 %), 22 / 32 EBR.
  `out/rev_b/ron/patchbay/pulsar.vmprog`.
- A first build with router2 gave HD Analog 86.55 and HD HDMI 75.28 on seed 1 and stalled on HD Dual (seed 1
  timed out); stay on router1. `dupin.py`: 0 `CARRY I0==I1`.
- **Checks** (working files in `.patchbay_work/rb_pulsar/`; GHDL raw 10-bit output against the integer model
  `imodel.py`, black input):
  - 720p60, defaults: 0 differing pixels of 921,600.
  - NTSC, both fields of a frame (two consecutive fields of motion): 0 differing pixels at the defaults, with
    Glow + Twin + Perspective, and with Paper + all six knobs moved + reverse travel (glide bypassed in a
    sim-only copy for that one).
  - The generated avid starts 4 pixels after the source's, so the tester sees the picture 4 px to the right
    (the shell's behaviour); the comparison allows for it.
  - Osc and camera (selfie.jpg) frames were rendered at 720p60 (`cmp_final.png`) and checked by eye only: the
    model's store timing is approximate, so its Osc frame is not pixel-equal to the RTL's (45 % of pixels differ).
  - NOT simulated: 1080i and 1080p (the model renders them; the RTL was only run at 720p60 and NTSC).
  - `thincheck.py` on the model: no single-row hard lines at 720p, 1080p or 1080i. At NTSC a flat line can sit
    on one field row (4.4 virtual rows) with faint neighbours.
- The last source change (two adders rewritten to remove a self-add carry, no change to the arithmetic) came
  after the three NTSC runs above. On the built sources: NTSC with Glow + Perspective 0 differing pixels, and
  the 720p60 default frame above.

## Cut / deviations

- **Seven live traces, not eight:** the past trace and six coming ones (the eighth bank is being refilled), so
  the tallest peak is 6 pitches.
- **Perspective has no horizontal shrink:** the traces crowd and the peaks shrink toward the top, but the burst
  column keeps its width (the per-trace x scale cost three products a trace).
- **Stepped controls:** K5 Rough has 9 steps, K6 Colour 4 saturation steps and 5 by-height / by-trace blends
  (the saturation lives in the palette ROM). The glow ramp's length follows the local pitch in steps of
  1 or 1.5 x 2^n.
- **Video is low resolution and one field late:** 64 x 30 cells, slewed; it stops at virtual row 1024 (the
  bottom 5 % of the screen reads the last row).
- **Resolution:** the samples of a trace are 8 OUTPUT pixels apart, so the polyline and the noise grain are
  1.5 x coarser relative to the picture at 720p and 2 x at SD (SD shows the centre 4:3: the burst sits in the
  same place, the flat line ends are cropped). At SD a line is at least one field row (4.4 virtual rows), so
  lines read thicker than at HD. No input carries field parity, so both fields of an interlaced source draw
  the same rows.
- The line's weight term uses the slope one pixel ahead of the pixel it draws (invisible; modelled).
- The exp2 ROM steps heights by 0.27 %: a 300-row peak moves in 0.8-row steps as a knob glides.
