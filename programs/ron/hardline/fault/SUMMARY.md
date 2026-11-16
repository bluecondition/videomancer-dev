# FAULT: a ring target cut into strips that slip (Hardline)

v0.1.0 built 2026-11-15, not flashed. Timing-closed on all six configs (router1, seed 1); GHDL render equals
the integer model pixel for pixel.

## What it draws

Marina Apollonio's "Dinamica circolare" put through a paper cutter, with Soto's dislocations in mind: a perfect
black-and-white ring target is cut into parallel strips and the strips are slid along the cut, so every ring
meets some other ring across the cut (a Vernier). At the default a cut halves the central disc and the two
halves slide apart into a yin-yang, with checkerboard zones left and right where rings land half a period out.
Black and white only (U = V = 512 on every pixel); nothing moves unless a control, a modulator or the camera
moves it.

- The cuts pass through the screen centre and stay there; X / Y move the rings under them.
- A strip's sample point is shifted along the cut BEFORE the distance to the centre is taken (the shift goes
  into the coordinates, not the radius), so the rings inside a strip stay perfect arcs.
- Slip is measured in ring periods, so the same Slip gives the same Vernier at any Period.
- Every cut carries an anti-aliased black kerf line about 1.75 px wide (full inside 0.4 px, gone at 1.4 px),
  so a cut is clean at any angle. The kerf opens with the first 2 px of slip: Slip at centre is an unbroken
  target with no lines on it.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | Ring centre, horizontal. Detent (+-14) = screen centre; the ends are 2 screens off-frame (arcs, then near-parallel stripes) |
| K2 | Y | Ring centre, vertical, same range |
| K3 | Period | Ring spacing, exponential, 448 px down to 14 px of a 1080-line frame (clockwise = finer, as Hypnos). Default 56 px |
| K4 | Slices | Strip height, exponential, 640 px (two slabs) down to 20 px (fine strips that weave a second figure). Default 108 px |
| K5 | Angle | Cut direction, -90 to +90 degrees about horizontal (detent +-6 = horizontal cuts, both ends vertical) |
| K6 | Camera | How far the video bends the rings. Default 100 %; 0 % = the clean figure |
| S7 | Glide | On: X, Y, Period, Slices, Angle, Camera and Slip ease (1/16 of the gap per field; centre and slip never slower than 1 px per field and land exactly). Off: no easing, the values latched at this field's vsync draw this field |
| S8 | Cut | Strips / Tiles. Tiles cuts both ways: rows slide along the cut and columns slide across it, so the four tile classes go four diagonal ways |
| S9 | Order | Alternate (+-Slip by strip parity: checkerboard Vernier) / Staircase (each strip half a Slip further than the last, wrapping every 8 strips: the whole figure leans) |
| S10 | Weight | Bold (equal bands) / Fine (white lines 1/6 of the period on black) |
| S11 | Source | Video / Osc (a slow triangle lattice in the camera's place) |
| P12 | Slip | Headline. Bipolar: centre (+-12) = unbroken rings; either way the strips shear apart, up to 3.9 ring periods. Default 0.73 period |

Every pot has a +-1 count hold (jitter is held, not eased). At both ends of P12 the two half-figures sit 8
periods apart (Alternate), a big leaning S (Staircase), or four ring centres in rounded squares (Tiles).

## Camera

The camera is never shown. Its luma (10-bit Y, black at 64 removed) goes through a 64-pixel horizontal box
(32 pixels at 720p and SD, which is 48 and 64 px of the 1080-line frame) and a vertical one-pole of 1/32 per
line, held in a line store with one entry per 4 pixels and interpolated across them. The result times K6 is
ADDED TO THE RING PHASE: bright areas push the rings outward, so the rings swerve around the subject.

- Gain: phase = level x G / 8 with G = 3 K6 / 8. At white: 0.64 period at 25 %, 1.28 at 50 %, 2.56 at 100 %.
- Black input or K6 = 0 gives exactly the generated figure (zero phase added).
- The filter width of the edge anti-aliasing follows the camera field's own gradient (a 4-bit half-octave code
  stored with each line-store entry), so rings squeezed by a hard camera edge stay anti-aliased.
- Osc: a triangle lattice 640 x 540 px (of the 1080-line frame), levels 0 .. 254 (white is 219, so up to 3
  periods at 100 %), drifting one cell in about 8.5 s across and 11 s down. It enters at the same point as
  the luma, through the same box and one-pole. It is the only thing in the program that moves by itself.

## Engine

Pipeline of 40 clocks, one latency in every mode (`processing_delay_clks = 40`). Stage Pk below.

Per field (sequencer started at the first vsync edge after active video; one small operation per state, one
serial 17 x 12 multiplier with subtract and accumulate):
- knob holds and glides; ring centre = W (128 + (K1 - 512)) / 256 and the same for Y;
- log2 of the ring frequency per pixel (Q14) and its half-octave index; strip height in pixels (Q11) and the
  ring period, their mantissas read from the pixel path's own 2^x ROM while it is idle;
- sin / cos of the cut angle (512-step quarter-wave ROM), pixel and line steps of the two strip coordinates,
  and their values at the first pixel reduced into one strip height while counting strips;
- slip in pixels = Slip x period, times (cos, sin), glided; the 8-entry shift table (16 serial products);
- camera gain, its log in half octaves, and the 256-entry camera phase table.

Per line: the two line-start strip coordinates step and wrap (two clocks on one adder each); dy steps by the
mode's line pitch.

Per pixel:
- P0 strip coordinates step and wrap at the strip height (two parallel adders each), 3-bit class counters;
- P1 .. P3 class -> shift table (block RAM, fabric register), Tiles sum; P1 .. P4 distance to the nearest
  cut -> kerf coverage (0 .. 16), which rides a block-RAM delay to P33;
- P4 .. P19 the Hypnos log-domain polar engine at 14 bits: |dx|, |dy| -> hi, lo -> log2 (priority encoder +
  256-segment table) -> u = log2(hi / lo) -> 0.5 log2(1 + 4^-u) -> log2 r + log2 f;
- P20 .. P25 2^x: mantissa table, one shift, a 16-bit window = ring phase mod 1 cycle;
- P19 .. P25 camera: line-store entry, interpolation across 4 pixels, phase table + a 9 x 4 product;
- P26 .. P32 the one-pixel box filter of the band square-wave (Hypnos), filter width from a 32-entry
  half-octave table (the ring gradient, or the camera field's where it is steeper);
- P33 .. P39 luma 64 + 3.42 x coverage, cut to the kerf; [1 2 1] / 4; 13/16 toward black (white 777);
  blanking gate on the generated avid.

Multipliers on the pixel path: two 7 x 5, one 11 x 8 (Booth), one 8 x 6, one 9 x 4 (gated adds).
Virtual 1080-line coordinates as in Hypnos: the engine works in real pixels with dy in x-pixel units, and
log2 of the x step (0, 0.585, 1 octave) is added to the ring frequency and taken off the strip height.
Generated avid (parity-locked to hsync, fixed width), vsync by edge with a saw-active guard, 22 block RAMs:
8 engine ROMs, 2 for the log2 delay line, 4 shift-table copies (two read ports x two components), 2 line
store, 1 box, 1 camera phase table, 2 sine, 1 kerf delay, 1 sync delay.

## Status

- Built 2026-11-15 with `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline fault` (router2 not tried:
  the chassis is Hypnos, which only closes on router1). All six configs on seed 1:
  HD Analog 78.21 MHz, HD HDMI 81.78, HD Dual 79.81; SD Analog 79.34, SD HDMI 83.81, SD Dual 76.69.
- 7,106 .. 7,134 / 7,680 LC (92.5 .. 92.9 %), 22 / 32 block RAMs. No `emulate_transparency`, no duplicate-input
  carry cells. vmprog: `out/rev_b/ron/hardline/fault.vmprog`.
- GHDL against the integer model (`.hardline_work/rb_fault/imodel.py`, luma compared at 10 bits, the capture
  sits 4 columns right of the model because the generated avid starts 4 clocks after the source's):
  0 differing pixels on every frame compared:
  - 720p decimation 1 (1280 x 720), default controls, black input: 0 of 918,720;
  - 720p decimation 1, bear.png at K6 100 %: 0 of 918,720 (model fed the tester's own stimulus luma);
  - 720p decimation 2 (the centre 640 x 360): Tiles + Staircase + Fine + Angle / X / Y / Period / Slices /
    negative Slip off default, 0; Staircase + Angle + Slip, 0; Tiles + Staircase + camera at K6 800, 0;
  - NTSC decimation 1 (720 x 243 fields): 0 on both fields, and the picture is the centre 4:3 of the 720p one
    (`ntsc_check.png`);
  - Glide On, 720p decimation 4, 60 fields: lands on exactly the Glide Off picture (0 of 56,880). The easing
    path itself was not compared against a model.
  Osc was rendered (clean, bold) but its lattice phase is not in the model, so it was not diffed. 1080i and
  1080p were not simulated.
- Not flashed. Never run on hardware: everything the sims cannot see (sync timing, block-RAM mapping, real
  blanking budgets) is unproven.

## Cut / deviations from the spec

- Angle is -90 .. +90 degrees about horizontal rather than 0 .. 180: the same set of cut directions, with
  the default at the knob's centre. The two knob ends are both vertical cuts and mirror each other.
- Staircase is an 8-strip sawtooth at half a Slip per strip (at a full Slip per strip it looked like Alternate
  at the default). Tiles applies the same law on both axes instead of a separate four-direction table.
- Slip reaches 3.9 periods, not 4.0. Slices runs 640 .. 20 px (spec: 540 .. 16).
- Camera smoothing is a 64-px box and a 1/32 pole (brief: 32 px and the shell's 1/8): a phase bend turns fur
  and noise into confetti at the shell's width. The bend therefore trails the subject by about 16 pixels and
  about 30 lines.
- The cos table is read one step (0.18 degrees) short, so n and t are not exactly perpendicular; Tiles are
  rhombi by that much.
- Hypnos's Q6 gradient chain (L2 combine, widening above 1/4 cycle per pixel, saturation) is replaced by a
  half-octave table: with periods of 14 px (7 real pixels at SD) and up, nothing here reaches 1/4 cycle per
  pixel, so the flat-grey case of the set rules cannot occur.
- Fit: the first RTL was 8,535 LC (111 %). Cut without touching a control (see `notes.txt` in the work dir).
- SD and 720p show the same picture (the centre 4:3 at SD); the kerf and the edge ramps are one REAL pixel
  wide, so they are relatively heavier at SD, and the camera smoothing is 48 px wide at 720p against 64.
- If a source put its vsync edge inside active video, the last lines of a field could glitch: the sequencer
  borrows the pixel path's 2^x ROM and rewrites the tables as soon as it starts.
