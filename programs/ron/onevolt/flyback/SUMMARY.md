# Flyback: bars that restart on every contour

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 4. A shape gradient (and the camera) summed in Passage, folded by
Staircase and patched into Prismatic Ray's Sync / Reset input: on every scan line the oscillator restarts each time
the folded signal rises through mid-grey, so a fresh train of bars sets off to the right from every contour, each
bar a copy of the contour it left. Presets are hand-written placeholders.

## The rack

- **LZX Visual Cortex**: the H ramp, the shape gradient (H+V triangles = diamond, or the two parabolic ramps mixed
  = ellipse), and the camera decoder's luma. Its animation generator in cycle mode is the 13 s LFO.
- **LZX Passage**: sums the crossfade of the H ramp and the shape (P12), the camera (K6) and the LFO.
- **LZX Staircase**: folds that sum (Frequency = K2 Contours, 1 .. 12), so it crosses mid-grey once per contour band;
  its Phase is swayed by a slow modulator.
- **LZX Prismatic Ray**: the hard-synced VCO. Sync / Reset in from Staircase (0.5 V comparator, rising edge, with a
  little hysteresis), Sync mode Trigger / Gate (S8), Frequency (K1), Pedestal (K4), Gain (K3), Multiply (driven
  by the density fade, see Engine), Frequency CV (S9). Its Triangle, Parabolic and Exponential outputs feed the
  colouriser.
- **LZX Color Chords**: three layers (triangle, parabolic, exponential), each with an RGB colour and an opacity; K5
  sweeps eight colourings.
- **LZX Staircase (second)**: the S10 alternative colouriser, fed by the triangle, /1 /2 /4 into R, G, B.
- **LZX Pendulum**: the two slower modulators (21 s Staircase Phase sway, 37 s lean of the figure's centre).

## The patch

    Visual Cortex H ramp ------------------> Passage ch1 (gain 1 - P12)
    Visual Cortex SUM (diamond) or parabolic ramps mixed (ellipse; S7) -> Passage ch2 (gain P12)
    Visual Cortex decoder luma (S11 Osc: a drifting triangle lattice, 2x) -> Passage ch3 (gain K6)
    Visual Cortex animation LFO (13 s) ----> Passage THRU (summed)
    Passage OUT -> Staircase Source (Frequency K2; Phase <- Pendulum A, 21 s)
    Staircase /1 -> Prismatic Ray Sync / Reset (Trigger / Gate: S8)
    the S7 shape gradient alone -> Prismatic Ray Frequency CV (S9 Out / In)
    Pendulum B (37 s) -> the shape's vertical centre (+-40 virtual lines)
    Prismatic Ray Triangle / Parabolic / Exponential -> Color Chords layers 1 / 2 / 3 -> encoder
    S10 Staircase: Prismatic Ray Triangle -> Staircase 2 -> /1 /2 /4 -> R G B (Color Chords unplugged)

## What it draws

At the defaults (Diamond, Gate, Chords): nested diamond tiers across the whole screen, alternate tiers flat black
(the oscillator held while the sync signal is high) and the others filled with 1-2 shaded bars each, magenta and
orange with violet fringes, every bar a copy of the contour that launched it, trailing right. Straight vertical bars
anchored at the left edge fill the rows that have not met a contour yet, and each diamond tip cuts the train behind
it with a short horizontal shingle edge. The contours breathe in and out (the 21 s phase sway plus the 13 s LFO), and
the whole figure leans up and down over 37 s.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Frequency | Prismatic Ray Frequency | bars per contour interval, 0.4 .. 12 (exponential); the absolute frequency follows K2 so K1 always means "bars per band", soft-limited at 1/32 cycle per virtual px. 0 = each band one dim shaded ramp (scales), 100 % = pinstripes |
| K2 Contours | Staircase Frequency | 1 .. 12 fold periods across the figure (exponential); contours are born at the edges and crowd in |
| K3 Gain | Prismatic Ray Gain | 1 .. 16 (exponential): soft shaded bars .. hard-edged flat bars |
| K4 Pedestal | Prismatic Ray Pedestal (bipolar) | the offset ahead of the gain = the duty: thin dim bars on black (0) .. fat bars, and under Gate the held tiers light up from black to violet to magenta (100 %) |
| K5 Colour | Color Chords (S10: Staircase 2 Frequency) | Chords: a continuous sweep through eight colourings (prism, sunset, ice, acid, RGB sum, ember, lagoon, neon), crossfaded per field. Staircase: its Frequency x1 .. x2.5, the colour bands multiply |
| K6 Camera | Passage ch3 gain | camera luma into the sync source, 0 .. 100 % (default); at 100 % full white shifts the sync source by ~0.86 of the shape's whole span (both scaled by Kc): the subject's outline becomes contours that launch their own trains |
| S7 Shape | Visual Cortex output | Diamond (H+V triangles) / Ellipse (parabolic ramps): nested ellipses whose rings crowd outward |
| S8 Sync | Prismatic Ray Sync mode | Trigger (reset on the rising edge: bars everywhere) / Gate (default: held while the sync signal is high: dark-backed tiers) |
| S9 Freq CV | the Frequency CV cable | Out / In: the shape gradient also drives the frequency (exponential, x4 at the centre): bars tighten toward the figure's middle |
| S10 Colouriser | Color Chords / Staircase 2 | Chords (the three waveshapes as three priority layers) / Staircase (the triangle folded /4 /2 /1 into B G R: each bar subdivided into binary colour bands) |
| S11 Source | Passage ch3 input | Video / Osc (a drifting 640 x 540 px triangle lattice at 2x the camera's gain: a field of small nested cells) |
| P12 Shape | Passage crossfade | 0 % = pure H ramp: vertical seams, ordinary bars between them .. the seams bow into the figure .. 100 % = pure shape: nested contours over the whole screen, each launching bars (the climax) |

## Video

- **Video (default).** Camera luma, (Y >> 2) - 16, summed over 16-pixel cells, each cell slewed down the screen by two
  poles (1/16 per line each, the states in a block RAM), scaled by the source gain once per cell (a serial
  multiplier), then slewed across by two one-pole filters (1/32 per pixel), is added to the sync source ahead of the
  Staircase. Under Gate the picture becomes a contour map: the bear's ears, eyes and muzzle come out as black islands
  with bar trains around them. With a black input the patch gives its full picture (K6 0 % is exactly that picture).
- **Osc.** The shell's drifting triangle lattice takes the camera's place at twice the gain.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Glide (K1, K2, K3, K4, K6, P12; 1/8 per field), knob laws
  as shift-adds and a shared exp2 ROM (Kc, Gain, log2 f0 with the soft limit), triangle LFOs, then 13 slots on one
  serial multiplier (42-bit accumulator with preload / keep): KM = Kc P12, the writer's gain start, the pixel DDA's
  start value (phase sway + LFO + shape + ramp in one accumulation), step and second difference, the line DDA's start
  values (cy-dependent), the camera gain. Then the **palette writer** (zone clocks 400 .. 4495): for each of 256
  levels of the faded triangle it runs Pedestal + Gain as a DDA, reads the two colouriser tables either side of K5
  at that level and crossfades them (a serial shift-add) into a 256-entry live palette.
- **Per line.** The shape's vertical part by second differences (the diamond as |r|), and for the next line two
  products on the same multiplier (the phase's line term and the Frequency CV's), the line-to-line phase change and
  its log2 (a serial normaliser).
- **Per pixel (14 stages to the colour).** Sync phase = line term + pixel DDA (+ camera), 16 bits; the Staircase
  comparator on its fold (set above 0.65, clear below 0.35); log2 f with the Frequency CV, exp2 ROM -> f; the
  oscillator loop (Trigger: reset on the rise; Gate: held while high; reset at the line start); the density fade:
  log2(f |phi_y| / |phi_x|) at the train's launch point (the bars copy the contour there), latched per train,
  fades the triangle toward the pedestal (the core's Multiply) where the restart column runs away from line to line
  (above ~0.07 cycles per line, gone at ~0.21); the live palette.
- **Output.** As the shell: two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16, a vertical
  [1 1]/2 with the line above, the blanking gate. C_LAT 20 in every mode; generated avid; virtual 1080-line
  coordinates; colours in the hardware convention (U = Cr).
- **Block RAM (21).** Colouriser tables 8, live palette 2, exp2 1, camera cells 2, box delay rings 4, line store 4.

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router1 seed 4 (HSYNC +1.36 ns, data <= -1.21 ns, 80.65 MHz); the builder's router2 seed 1 was outside the +1.0..+2.3 ns window. A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh flyback router1 4 hd_hdmi. Screen data: .heirloom_work/rb_flyback/skew/.

- **v0.1.0 built 2026-11-25 with router1, all six configs on seed 1, not flashed.**
  Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt flyback` -> `out/rev_b/ron/onevolt/flyback.vmprog`.
  (router2 missed HD Analog on seed 1 and stalled on seed 2 at 90 % utilisation; router1 closed every config.)

  | Config | Router | Seed | Fmax (MHz) | LC |
  |---|---|---|---|---|
  | HD Analog | router1 | 1 | 89.37 | 6,742 |
  | SD Analog | router1 | 1 | 86.51 | 6,737 |
  | HD HDMI | router1 | 1 | 90.42 | 6,737 |
  | SD HDMI | router1 | 1 | 87.67 | 6,740 |
  | HD Dual | router1 | 1 | 90.36 | 6,739 |
  | SD Dual | router1 | 1 | 86.09 | 6,733 |

- 6,733 - 6,742 / 7,680 LC (88 %), 21 / 32 EBR. iceunpack OK x6; the packed config carries the labels and presets
  (vmprog 459,310 bytes). No pin-skew screening.
- **Checks** (`.onevolt_work/rb_flyback/`): GHDL analysis / synth front end clean; `scan.py` 0 sites; dupin 0
  duplicate LUT inputs, 0 CARRY I0==I1; TOML pre-check clean.
- **GHDL vs the integer model `imodel.py`** (720p60, decimation 1, black input): **0 differing pixels** (of 917,444,
  rows 1 .. 719) for the final sources at the defaults (`rtl_default.png`) and with S7 Ellipse. Before the last
  change (the fade threshold, -28 -> -36, one constant) these were also 0 differing pixels against the model with
  the same threshold: S8 Trigger, S9 In, S10 Staircase, P12 0 % and 100 % (gliding from the default), K3 + K5 at
  100 %, Trigger + Ellipse + Freq CV together, a later field (field 9; 440,405 pixels moved since field 3), NTSC
  (720 x 243) and 1080i (1920 x 540). Osc and camera frames match the model
  in structure, not bit for bit (the camera path's start-up at the left edge is not modelled), as in the other shell
  programs.
- `thincheck.py` (model, 23 settings x 3 times): no single-row hard runs of 24 px or more at 720p; at 480i three
  settings (Ellipse + Staircase, and Osc) show one or two runs of 27 .. 38 px in one of the three fields.
- `sheet.png`, `strip.png`, `camera.png`, `modes.png`: control sheet (each knob 0 / 25 / 50 / 75 / 100 %, each switch
  flipped, P12, all 16 switch combinations), an 8-frame motion strip, camera strengths, the four timings, all from
  the integer model.

## Cut / deviations

- **S8 defaults to Gate**, not Trigger: only under Gate does the camera read as a recognisable contour map; in Trigger
  every camera-made contour cuts the trains into shingles (a busy, shredded picture with the selfie). Trigger stays
  one flip away and is the "bars everywhere" look.
- **The camera at K6 100 % with selfie.jpg is busy**: its textured background makes many small contours, so the
  picture shows short horizontal shingle edges (straight, not per-line ragged). The bear reads clearly.
- **The density fade is Prismatic Ray's Multiply driven by an internal measure** (a CV no real patch cable carries):
  it stands in for what a real rack would show as moire and one-row slivers where contours run nearly horizontal.
- **S10 hands K5 to the second Staircase's Frequency** (Color Chords is unplugged in that position), so K5 stays live.
- **The Frequency CV is exponential (x4 at the centre) and capped at 1/32 cycle per pixel**: the shell's output
  filter washes out anything finer.
- **LFOs are triangles** (no multiplier slots), so the slow motions reverse without easing.
- **Resolutions.** 720p and 1080p draw the same picture; SD shows the centre 4:3 (the diamond's side tips fall off
  the edges); 480i's 2.25-virtual-line field rows alias a few near-horizontal contour crossings (see thincheck).
- Row 0 of each field mixes with the last row of the field before (the shell's line store is not cleared).
