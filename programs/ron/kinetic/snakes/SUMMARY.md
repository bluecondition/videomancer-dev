# Snakes (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/snakes.vmprog`.

## What it draws

Akiyoshi Kitaoka's "Rotating Snakes" (2003): discs built of concentric rings of segments that step
black -> dark colour -> white -> light colour around each ring. That luminance order makes a still disc seem to
turn in peripheral vision; here the rings also really turn. The default frame (Vega palette) is a staggered
lattice of discs 315 virtual px across on a navy ground, about five across and three and a half rows: each disc
is a light hub and six rings of 32 segments (black, magenta, pale yellow, cyan), adjacent rings offset half a
segment, a thin ground-coloured line between rings. Alternate discs carry the sequence mirrored, so neighbours
seem to turn opposite ways. At the default Depth and Spread the wave has wound each disc into a pinwheel,
left- and right-handed on alternate discs.

## Element and state

Element: one ring of one disc. State: its rotation angle A (8192 units per turn).

    theta = Spread * index + (Spread / 2) * ring - phase       index: Linear i + j, Radial max + min / 2 of (|i|, |j|)
    A     = polar angle + Depth * wave(theta) +- drive         (+ on even rings, - on odd rings)
    phi   = Segments * A + (odd ring: 1/2 step - source; even ring: + source)      8192 = one four-step cycle
    step  = phi >> 11  (0 black, 1 dark, 2 white, 3 light);  alternate discs use the mirrored phi (Order)

i, j are the disc's column and row counted from the disc at the screen centre (Scale zooms about the centre).
drive is an angle that advances every field by P12^2; phase advances by Rate.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | disc pitch 180 .. 1018 virtual px, exponential, glided (default 360: five discs across; 100 % = one wheel filling the screen) |
| K2 | Segments | 3 .. 16 four-step cycles per ring = 12 .. 64 segments (default 8 = 32 segments), with hysteresis |
| K3 | Depth | wave swing 0 .. 1/4 turn, square law (default 0.117 turn = 3.7 segments at 32 segments) |
| K4 | Rate | wave speed, bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Spread | phase step per disc 0 .. 1/2 cycle (square law; default 0.28), and half of that per ring, so a swing winds each disc into a pinwheel |
| K6 | Colour | the 16 shared palettes with hysteresis; the page is the palette ground, the four tones are the palette's colours shaded (see deviations) |
| S7 | Wave | Smooth (sine) / Snap (dwell and quick eased flips) |
| S8 | Field | Linear (the wave crosses the lattice diagonally) / Radial (octagonal rings of discs from the centre) |
| S9 | Lattice | Staggered (rows 7/8 pitch apart, alternate rows offset half a pitch) / Square |
| S10 | Order | Alternate (the sequence is mirrored on alternate discs: neighbours seem to, and do, turn opposite ways) / Same |
| S11 | Source | Video / Osc |
| P12 | DRIVE | the headline: continuous crawl, speed on a square law, 0 .. about 1/4 turn per second, odd and even rings in opposite directions (default 40 % = 0.036 turn per second). 0 % = no crawl |

Presets: Snakes (defaults), Pure Illusion (Depth 0, Rate centre, Spread 0, Drive 0: nothing moves), Counter Crawl,
Iris Ripple, Clockwork, Big Wheel.

## Video

`program_type = "processing"`. The picture's luma is the second modulator and adds to the same state: luma
(8 bits, less black) -> 1/16 one-pole smear along the line (about 16 px) -> ONE sample per disc, taken at the
disc's centre column on the first line of its cell row, kept in a 256-entry RAM by disc column and read back
for the rest of the row, so every disc is rigid. The sample turns the disc's odd and even rings against each
other (phi +- luma * 8: white is about 0.9 of a step each way), which slides the brick pattern of segments
toward aligned spokes: bright parts of the picture show as discs whose segments line up. With a black input the
offset is zero and the wave and drive alone run. Source = Osc substitutes the shell's drifting two-axis triangle
lattice (640 x 540 virtual px) at twice the camera's gain.
The sample point is the top of the disc's cell (its first line), not the disc's centre row, and the first line
of each cell row still uses the row above's sample (that line is inside the ground line around the disc).

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): glides on Scale and Drive; one serial 24-step
  multiplier (pitch exponent through a 2^(i/256) ROM, the v step, the 8/7 row pitch, Depth^2, Spread^2, Drive^2,
  Segments * pixel size, Rate); the wave phase and the drive angle; the anti-aliasing ramp shifts; the blend
  table (40 words: tone pair x 5 coverages, less the ground), filled by adders straight from the palette ROM;
  the lattice origin, by stepping back from the screen centre to pixel 0 and line 0 (W/2 + H/2 subtractions).
- **Per line**: v accumulator (24 fraction bits, a carry = a new row of discs), |dy|, row parity, Spread * j.
- **Per pixel** (39 stages): u accumulator -> |dx| -> octant fold -> normalise -> reciprocal ROM (1024) ->
  ratio (11 x 12 multiply as gated sums over 4 stages) -> atan ROM (1024) and sec ROM (512) -> radius
  (11 x 12 multiply) and unfolded angle -> ring number, ring-edge coverage (0 .. 8) -> theta -> quarter-wave
  sine ROM -> Depth * wave -> A -> * Segments -> phi -> step, side, step-edge coverage (4 .. 8) -> blend table
  -> * ring coverage / 8 + ground -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums,
  luma [1 2 1]/4 delayed to the same group delay, luma 13/16 toward black) -> blanking gate.
- Anti-aliasing without dividers: step edges ramp over a power-of-two span of phi chosen per field from
  Segments * pixel size and per ring from its radius band (x 2/3 in the upper half of a band), so the ramp is
  about 0.7 .. 2 pixels everywhere; the changes of ramp fall on ring boundaries. Ring edges ramp over 8 << kr
  units of radius (0.7 .. 1.4 px) around a ground-coloured line at least about 2 px wide.
- One latency in every mode: C_LAT = 46 = processing_delay_clks. Generated avid (the shell's parity-locked
  start), virtual 1080-line coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware
  convention (U register = Cr); luma 10 bits, chroma 8 bits until the output filter. Four 256 x 16 delay rings
  carry the larger coordinate, the angle, theta and the coverage bits past the multipliers.
- Chassis blocks are louver's ([A] timing / zone / glides / sequencer, [B] generated avid, [C] Osc and smear,
  [F] output conditioning), unchanged apart from the sequencer's slots.

## Status

- v0.1.0 built 2026-11-14 through the shared queue with router2 (the builder default). Not flashed.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | **3** (1 and 2 missed timing) | 77.24 MHz | 6935 |
| SD Analog | 1 | 72.53 MHz | 6934 |
| HD HDMI | **3** (1 and 2 missed timing) | 78.38 MHz | 6937 |
| SD HDMI | **2** (seed 1 stalled the router: 900 s timeout) | 76.34 MHz | 6940 |
| HD Dual | **2** (1 missed timing) | 76.37 MHz | 6940 |
| SD Dual | 1 | 71.27 MHz | 6931 |

- HD needs 74.25 MHz, SD 27. 6,940 / 7,680 LC (90.4 %), 23 / 32 block RAMs. Six bitstreams pass iceunpack.
  Standalone yosys: no `emulate_transparency`; no carry self-adds (dupin: none).
- The design is seed-marginal under router2: a seed-1 profile of HD Analog routed at 71.7 MHz, its critical
  path c_hg -> p20_a (the ring-coverage stage: subtract, shift by kr, + 4, clamp). If a rebuild misses, that
  stage reduces exactly to clamp((e - 8) >> kr) because hg = (4 << kr) + 8, which drops one adder.
- The Depth and Spread DEFAULTS were raised after the bitstreams were built (TOML only: 512 -> 700 and
  300 -> 760, so the default frame shows pinwheels and the Wave / Field / Order switches read in a still). The
  vmprog was repacked from the same six bitstreams with the new program_config.bin (vmprog_pack.py); only config
  and hash bytes differ. `./build_set.sh kinetic snakes` rebuilds everything from the folder as it is.
- Model = RTL (GHDL): pixel diff 0 of 918,720 on the full 720p60 decimation-1 frame (black input, the first
  defaults); 0 of 228,960 on 640 x 360 windows of the 720p raster for the same frame, for a capture six fields
  later (motion: phase and drive advance exactly as the model's), and with selfie.jpg as the camera; Source =
  Osc differs only on row 0 (165 px: the top row's samples on line 0 come from the previous field's lattice
  position). At the final defaults: 0 of 228,960 with a
  black input, 0 with selfie.jpg, Osc row 0 only (162 px); NTSC (720 x 243, both fields): 0 of 173,988 each.
  1080i (1920 x 540, both fields): 0 of 1,034,640 each. Coverage renders
  (640 x 360 windows): Snap + Radial + Square + Order = Same: 0 px; Segments 60, Depth 29 %, Spread 39 %,
  palette 10, reverse Rate, Radial: 0 px.
- The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the generated avid
  starts 4 clocks after the source's, as in the pilots).
- Renders: `.kinetic_work/rb_snakes/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion); the
  integer model is `imodel.py` there.
- Rebuild: `./build_set.sh kinetic snakes`.

## Cut / deviations

- **Tones are shaded palette colours.** The four ring tones are the palette's four colours sorted by luma and
  shaded into fixed luma bands so the order black < dark < light < white is exact on every palette: darkest
  scaled to luma <= 0.05, lightest mixed toward white to luma >= 0.93, the two middle colours brought into
  0.22 .. 0.34 and 0.62 .. 0.74 (scaled toward black or mixed toward white). The page stays the palette ground.
  On the light-ground palettes (Paper, Bauhaus, Candy, Mint) the white tone equals the page colour.
- **P12 = 0 is "no crawl", not "nothing moves".** The standard wave (Depth, Rate, Spread) stays live at every
  P12 setting so those knobs always act; the picture is perfectly still when Rate is also at its centre detent
  (preset "Pure Illusion").
- Discs do not touch: radius 7/16 of the pitch (hub + six rings of equal width), so one cell test per pixel is
  enough. Kitaoka's discs overlap and his rings widen outwards.
- The source turns odd and even rings against each other (a rigid turn of a whole disc would be invisible in
  a still), and it is sampled at the top of the disc's cell.
- The drive runs odd and even rings in opposite directions at every setting, as the spec asks for 100 %.
- A thin ground-coloured line separates the rings (it is what anti-aliases the ring edges with one ring
  evaluated per pixel) and the hub is a plain dot in the light tone.
- Polar accuracy, measured on the model against atan2 / hypot at 1080p: angle error as arc length 0.05 px rms
  at Scale 0 %, 0.09 px rms at the default, 0.24 px rms (1 px worst case, at the 900 px rim) at Scale 100 %;
  radius error 1.2 units rms (a unit is 1/4096 pitch: 0.1 px at Scale 0 %, 0.2 px at the default, 0.6 px at
  Scale 100 % on 1080p, where one unit is half a pixel). Edges looked clean at 4x zoom at Scale 100 %.
- Segments and Colour use hysteresis; Depth, Spread and Segments are not glided.
- Stills cannot show Rate, and Depth / Spread / Wave / Field / Order change only the rings' relative phase, so
  those panels of the control sheet differ less than the Scale, Segments, Lattice, Colour and Source panels.
- **SD looks heavier.** NTSC shows the same picture (the centre 4:3, discs round), but a field has 243 lines,
  so the ground lines between rings are about 2.5 times heavier relative to the rings than at 720p (the ramp
  follows the line pitch) and a ring is only about 5 field lines wide at the default Scale; below about Scale
  30 % the rings are 2 .. 4 field lines wide and read as texture. 720p and 1080p hold every Scale.
- Rasters rendered in GHDL: 720p60 full frame (decimation 1), 640 x 360 windows of it (decimation 2), NTSC
  720 x 243 x 2 fields, 1080i 1920 x 540 x 2 fields. Not rendered: 1080p, PAL, 480p.
  Scale and Drive were only rendered at their defaults in GHDL (they are glided; other values need a sim-only
  copy with the glides bypassed), so the Scale range and the SD / 1080p extremes are model-only.
