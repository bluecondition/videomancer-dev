# Agate

**Patchbay. v0.1.0 built 2026-11-14; v0.1.1 (K6 camera pass) built 2026-11-15; not flashed.** Marbling by cascaded cross-modulation: the H and V ramps
phase-modulate each other through three octaves, and the folded X ramp drives a wavefolder and a banded
colouriser. Paper marbling, suminagashi, cut agate.

Flash `out/rev_b/ron/patchbay/agate.vmprog`; rebuild with `./build_set.sh patchbay agate`.
Work files: `.patchbay_work/rb_agate/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`, `strip.png`).

## The patch

    H ramp --(Scale)--> X                       V ramp --(Scale)--> Y
    stage 0   VCO 1 (PM in = Y)      --VCA ax0--> X      once per line: the whole line of X shifts
              VCO 2 (PM in = X)      --VCA ay0--> Y
    stage 1   VCO 3 (2x, PM in = Y)  --VCA ax1--> X
              camera luma (Source: or the Osc lattice at 4x the level) --> X
              VCO 4 (2x, PM in = X)  --VCA ay1--> Y
    stage 2   VCO 5 (4x, PM in = Y)  --VCA ax2--> X
    X --(Bands gain)--> wavefolder (triangle) --(+ Colour)--> colouriser (256-entry palette, ten bands)
    differentiator |dX/dx| + |dX/dy| --> colouriser softness (sharp -> blurred palette -> the window's mean)

Every VCO is a phase accumulator read through the other axis (cross-modulation), an octave above the one
before, and drifts at its own rate: Speed times 1, -3/4, 5/4, -5/8, 7/8. One waveform switch serves all five
(sine / triangle). All five VCAs hang off STIR; Lean pans the control voltage between the three X VCAs and the
two Y VCAs; Detail is the attenuator ratio from each stage to the next.

In numbers (one unit = one cycle of the base frequency f0 = 1.5 cycles per screen width at the default):

    X = (x - centre) f0,  Y = (y - centre) f0
    X += A lx w(Y + p0);            Y += A ly w(X + p1)
    X += A r lx w(2 Y + p2);  X += 0.2 luma (0.8 Osc);  Y += A r ly w(2 X + p3)
    X += A r^2 lx w(4 Y + p4)
    U = X g;  entry = fold(U) * 128 + rot      (the fold sweeps half the palette cycle, forwards and back)
    A = 0.62 (STIR)^2,  r = 0.25 .. 0.75,  lx = 1 .. 0.2,  ly = 0.2 .. 1,  g = 0.5 .. 8

## What it draws

Default: deep blue, pale teal, cream, amber, yellow and rust bands folded into combs, eyes and S-shaped
veins: a slab of cut agate about two swirls wide. Each band has a slight gradient and the edges are a little
soft horizontally (the rack's softness); where the field is steep the bands melt into a blurred version of
the palette instead of aliasing. The whole sheet flows slowly: the big swirls slide one way, the finer combs
the other, at five different rates, so the picture never repeats.

STIR at 0 % is a ruled sheet of straight vertical colour bands (bent by the camera if there is one); at 50 %
the bands wave; at 100 % they are folded several times over into fine marbling.

## Controls

| Control | Label | Module | What it does |
|---|---|---|---|
| K1 | Scale | H / V ramp gain | base frequency of both ramps, 0.53 .. 4.2 cycles per screen width (3 octaves, exponential): one giant swirl .. a field of small ones |
| K2 | Lean | pan between the X VCAs and the Y VCAs | left: the V ramp modulates H (bands combed sideways, lying down); right: the H ramp modulates V (upright pillars with flame edges); centre: both |
| K3 | Detail | attenuator ratio between stages | 0.25 (only the big swirl) .. 0.5 (default) .. 0.75 (the fine stages as strong as the coarse: tight combing) |
| K4 | Speed | drift of the five VCO phases | bipolar, square law, +-8 deadband at the centre (frozen); each phase at its own rate and direction |
| K5 | Colour | colouriser rotate (moved from K6 in v0.1.1) | which half of the ten-band palette the fold sweeps: blue / cream / amber .. rust / maroon / teal .. teal / black / blue. Default 0. (K5 Bands is fixed at its old default 512 = wavefolder gain 2.0) |
| K6 | Camera | gain of the luma into X | camera strength 0 .. 100 %, default 50 %: 0 % = no camera (the black-input stone), 25 % = the v0.1.0 depth (0.2 swirl at white), 50 % = x2, 100 % = x4; the Osc lattice keeps its 4x over the camera. Glided, +-4 deadband |
| S7 | Shape | VCO waveform | Round (sine: curved swirls) / Angular (triangle: zig-zag chevrons) |
| S8 | Bookmatch | H ramp rectifier | Off / On: the H ramp becomes \|x - centre\|, the stone mirrored left / right like a split slab |
| S9 | Veins | edge keyer on the colouriser | Off / On: a line at every band edge, pale between dark bands and dark between light ones |
| S10 | Colours | colouriser palette | Stone (blue, teal, cream, amber, rust, maroon, white) / Neon (violet, hot pink, yellow, green, cyan, blue on black) |
| S11 | Source | X adder input | Video (camera luma) / Osc (a drifting triangle lattice at four times the level) |
| P12 | Stir | all five VCAs | modulation depth, square law: 0 % straight ruled bands .. 78 % the default .. 100 % deep folded marbling |

K1, K2, K3, K5 (Colour), K6 (Camera) and P12 glide (1/8 of the gap per field).

## Video

Camera luma is added to X between stage 1 and stage 2, so the picture displaces the bands (bright areas push
them sideways by up to 0.2 of a swirl) and the finest stage then combs that displacement: the picture is
stirred into the stone, it does not replace it. The luma is smoothed first (two horizontal one-poles of 1/16
per pixel, a vertical one-pole of 1/16 per line, 12-bit store) so edges stay clean curves; the gain is fixed
and modest. With a black input the stone is complete and exactly the model's. Osc substitutes the shell's
drifting triangle lattice (640 x 540 virtual px) at the same point at four times the camera's level: the
stone reorganises around the lattice, an unmistakable flip.

## Engine

- Per field (vblank zone, one serial multiplier, 17 products): the six glides; Scale and Bands through an
  exponent ROM; the X and Y steps and origins (virtual 1080-line coordinates: SD shows the centre 4:3); the five
  amplitudes; the phase drifts; the window's mean colour (read from the soft palette ROM).
- Table fill (vblank zone, about 3,100 clocks): amplitude x waveform for the four pixel-path stages into four
  256-entry quarter-wave tables, one entry per three clocks through a 16 x 4 multiplier. So the pixel path
  has no multiplier per stage: lookups and adds. The two largest tables also store the forward difference.
- Per line (line engine, computes the NEXT line): the Y phase and stage 0's X term (two neighbouring
  waveform entries, interpolated with 4 fraction bits, times the amplitude).
- Per pixel, 41 stages, one latency (processing_delay_clks = 40) in every mode: beam ramp (bookmatch =
  down / hold / up) -> table 1 by X (interpolated, 3 fraction bits) -> table 2 by Y (interpolated) -> + luma
  -> table 3 by X -> table 4 by Y -> X x Bands (three nibble partials) -> fold + Colour -> footprint (step
  to the pixel on the left + step to the line above, from a quarter-rate store) -> sharp palette ROM, soft
  palette ROM -> veins blend -> sharp / soft / mean blend (8 levels each) -> the shell's two
  [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the per-line resync, luma 13/16, blanking gate.
- X and Y wait for each other's lookups in block-RAM delay rings (the pattern kiln and fern use), the X of
  stage 12 is a second copy of the beam ramp, and the syncs and both box filters' 8-clock delays are rings
  too. The first version, with these as FF chains and a fourth stage, needed 9,107 logic cells and did
  not fit.
- Generated avid (parity-locked), colours in the hardware convention (U register = Cr).
- Memories (24 block RAMs): stage tables 6, sine ROM 1, exponent ROM 1, luma store 1, colouriser-phase store
  2, sharp palette 4, soft palette 2, rings 7.

## Status

v0.1.0 built 2026-11-14 with ROUTER1, all six configs on seed 1; not flashed.
Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay agate`.

| Config | Router | Seed | Post-route Fmax | Needed | Logic cells |
|---|---|---|---|---|---|
| HD Analog | router1 | 1 | 81.98 MHz | 74.25 | 7,159 |
| HD HDMI | router1 | 1 | 94.38 MHz | 74.25 | 7,171 |
| HD Dual | router1 | 1 | 90.11 MHz | 74.25 | 7,166 |
| SD Analog | router1 | 1 | 85.95 MHz | 27 | 7,150 |
| SD HDMI | router1 | 1 | 92.05 MHz | 27 | 7,176 |
| SD Dual | router1 | 1 | 84.15 MHz | 27 | 7,180 |

A first build with router2 was thin (HD Analog 75.64, HD HDMI 81.71 on seed 2 after seed 1 stalled, HD Dual 76.74);
the shipped vmprog is the router1 build. An NTSC frame of the shipped RTL (both fields, default knobs, black
input) was rendered and viewed by the orchestrator: the same picture as HD (the centre 4:3), fields consistent.

Logic cells 7,150 .. 7,180 of 7,680 (93 %), 24 of 32 block RAMs. HD HDMI: seed 1 stalled router2 (15-minute
timeout), seed 2 routed. HD Analog's margin is thin (1.9 %), HD Dual's 3.4 %. An earlier build of the same
design before the last area trims (7,316 cells) closed HD HDMI only on seed 5, so expect seed retries on a
rebuild. dupin.py on the synthesised netlist: no self-adds (router1 was not tried).
Rebuild: `./build_set.sh patchbay agate`.

Verification (GHDL image sims of exactly the built sources):
- 720p60, decimation 1, default knobs, Source Video with a black input: **0 of 918,720 pixels differ** from
  the integer model (`imodel.py`, field 2), compared 4 columns to the right (the generated avid's start).
- 720p60 decimation 2 (640 x 360), all of S7, S8, S9, S10 on and K1 .. K6 and P12 off their defaults (Speed
  negative), with the knob glides emulated: 0 pixels differ. So every knob map, both waveforms, bookmatch,
  veins, both palettes, the table fill and the phase drifts (two fields of motion, both directions) match
  the model bit for bit.
- Source Osc and Source Video fed `selfie.jpg` were rendered at 720p60 and looked at (`cmp_final.png`); the
  model's luma path is not bit-exact with the RTL's source timing, so these were not pixel-diffed.
- **SD (ntsc) was rendered once, by eye only** (default knobs, black input, both fields: the same picture as
  HD); it was not pixel-diffed against the model. **1080i was not rendered in RTL.** No second, later
  capture was made for motion either; the motion evidence is the phase match above and the model's `strip.png`.

## Cut / deviations

- **Three stages, not the spec's 4-5.** A fourth stage (8x) was built and verified but the design then
  needed 9,107 logic cells. The last stage's Y update never reaches the picture, so the patch is five VCOs.
  Detail's range was widened (ratio up to 0.75, was 0.61) so the finest stage can still comb tightly.
- **The stage multiplies are per field, not per pixel**: amplitude x waveform tables refilled in the vblank
  zone instead of one multiplier per stage in the pixel path (the lozenge v0.6 trick). Same picture, about
  a third of the logic.
- **Fade to the local mean** is done in two steps: to a blurred copy of the palette (blur about 16 entries)
  while the field moves 0 .. 4 palette entries per pixel, then to the mean colour of the half-palette window
  on screen up to 36 entries per pixel. The footprint is horizontal step + vertical step; the vertical one
  compares with the line above at a quarter of the horizontal rate.
- **Stage 0's tables are interpolated.** At 1,024 phase steps per cycle the two largest lookups moved the
  band edges in steps of up to 1.2 palette entries from one line to the next (a fine row hatch, worst on
  veins). Linear interpolation brought the zig-zag-row count from 2.5 % to about 0.3 % of pixels with Veins
  on and to 0.01 % at the defaults. A trace of it remains on steep diagonal veins.
- **No video gain control**: all six knobs belong to the patch, so the camera's depth is fixed (0.2 swirl).
- The camera's luma is stored at 1/8 of the horizontal rate (one block RAM; a finer store would have been
  the 25th). Where the picture or the Osc lattice changes quickly the bands step every 8 pixels: visible as
  fine beads along steep contours in the Osc render and as slight notching with selfie.jpg. The first line
  of each field uses the current line (no stored line above it yet).
- The generated avid starts 4 clocks after the source's, as in every shell program: the first 4 columns of a
  simulated frame are blank and the picture is 4 columns narrower on the right.
- Logic cells are 1 % over the set's 7,100 guide (7,159 .. 7,180).

Not checked on hardware: everything. Most likely to need a look: the thin HD Analog timing margin; the syncs
running through a block-RAM ring; SD and 1080i (never rendered); the 8-pixel beading in Osc; Bands, Detail
and STIR all at maximum together give mostly the mean colour (each alone keeps the picture).

## K6 camera pass (2026-11-15), v0.1.1

- **K6 is now Camera** (0 .. 100 %, default 512 = 50 %): the gain on the smoothed luma (or the Osc lattice)
  before it is added to X between stage 1 and stage 2. Law: luma x 13 k / 4096 (gain register `s_cgq` = 13 k >> 7,
  0 .. 103, k = the glided knob; video >> 5, Osc >> 3): 0 % = 0 (the black-input stone), 25 % = x13/16 (the
  v0.1.0 fixed gain, 0.2 swirl at white), 50 % = x2, 100 % = x4. One 12 x 7 multiply in its own stage, so the
  luma reaches X one pixel later than before (invisible: it is stored at 1/8 of the pixel rate). The term is
  16 bits now (the Osc at 100 % displaces up to 3 swirls). K6 rides the shared glide (channel 3, formerly K5's),
  +-4 deadband; latency unchanged (40).
- **K6's old function (Colour rotate) moved to K5; K5 Bands is fixed at 512** (wavefolder gain 2.0, its default):
  K1 Scale already sets the band density on screen. With the gain a constant the pixel path's X x Bands
  multiplier folds into a shift: standalone yosys 4,305 -> 4,081 LUT4 and 1,540 -> 1,483 carries, so the
  camera multiply costs nothing net. Bit-exact with v0.1.0 at K5 = 512. Presets lose their Bands values
  (Suminagashi 300, Neon Comb 700, Ruled 640); the Colour values moved from K6 to K5. No new block RAM (24).
- **Seen in GHDL** (720p60, decimation 2, selfie.jpg; `.patchbay_work/rb_agate/k6/k6sheet.png`): 0 % the clean
  stone; 25 % the bands bend around the picture; 50 % strong bending, the veins zig-zag with the picture's
  structure, soft horizontal smears at band edges (the luma store's 1/8 rate and the one-pole lags, now twice
  as visible) but no per-line breakup; 100 % the picture's contours carve the stone (rings and eyes from the
  picture), smears more visible, colours intact: a bold climax. Osc at 50 %: the stone reorganises into the
  lattice's diamonds, with the known 8-pixel beading on steep contours.
- **Build v0.1.1 (2026-11-15), router1, all six configs on seed 1**: HD Analog 80.17, SD Analog 83.06, HD HDMI
  88.24, SD HDMI 83.54, HD Dual 79.76, SD Dual 74.40 MHz post-route (worst HD 79.76; HD Analog's margin is now
  8 %, was 1.9 %). 6,928 .. 6,953 / 7,680 LC (91 %, down from 93 %), 24 / 32 EBR.
  `out/rev_b/ron/patchbay/agate.vmprog`. Not flashed.
