# Cogwork (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed.

## What it draws

A wall of meshed cogs on a square lattice, turning as one train. The art it comes from: clockwork, watch
movements and gear walls. Alternate gears (checkerboard parity) turn opposite ways and sit half a tooth apart,
so every gear meshes with its four neighbours. At the defaults (Bauhaus palette) nine by five red and blue
twelve-tooth gears with gold hubs fill a cream screen, creeping round together, while a slow wave travels
diagonally through the wall, swinging each gear's spoked wheel against its rim and swelling its hub.

Each gear, from the centre out: a hub hole, an accent-coloured hub, a collar, four bar spokes with windows
between them, a rim with a darker bevel groove, and trapezoid teeth.

## Element and state

- Element: one gear per lattice cell of pitch P (pitch radius P / 2); the centre gear sits on the screen centre,
  so Scale zooms about the centre.
- The train angle is common to all gears (one rigid train); only its sign alternates.
- State = Depth x wave(theta) + source. It does two things to its gear: turns the spoked wheel against the rim
  (14 bits per turn) and swells the hub and collar (state / 16, limited to -230 .. +420 of 4096 per pitch).
- theta = Spread x index - phase. Linear: index = column + row (the wave crosses on the diagonal). Radial:
  index = max + min / 2 of (|column|, |row|) from the centre gear (an octagonal ripple).
- wave is a sine, or Snap: the first sixteenth of the sine stretched over the quarter cycle (75 % dwell).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | gear pitch 64 .. 512 virtual lines, exponential, glided (default 216: five rows) |
| K2 | Teeth | 6, 8, 10, 12, 16, 20, 24 or 32 teeth, with hysteresis (tooth depth follows: 0.95 pitch / N) |
| K3 | Depth | spoke swing 0 .. 1/2 turn and hub swell, square law |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Spread | phase step per gear 0 .. 1/2 cycle, square law |
| K6 | Colour | the 16 shared palettes with hysteresis: ground, A / B = gears by parity, accent = hubs |
| S7 | Wave | Smooth / Snap |
| S8 | Field | Linear / Radial |
| S9 | Body | Spoked (windows show the ground) / Disc (windows filled with a darker shade of the gear) |
| S10 | Ink | Fill / Line (every edge drawn as a 3.5-px outline: a drawing-office gear train) |
| S11 | Source | Video / Osc |
| P12 | Drive | train speed, 1/256 tooth per field .. 1 tooth per field, exponential, glided |

P12 is in teeth per field, so at every tooth count the top of the slider is where the teeth strobe: just under
100 % they seem to run backwards, at 100 % (0.995 tooth per field) they all but stand while the spokes whirl.

Presets: Gear Wall (defaults), Blueprint, Sprockets, Watch Movement, Strobe, Halftone Hubs.

## Video

- Source = Video: the luma (>> 2, less 16) is smeared along x by a 1/64 one-pole filter and HELD per gear: on
  the first line of each cell row the smeared value at each cell's centre column goes into a RAM indexed by the
  column, and is read back for the rest of the cell row. So every gear stays rigid. state += 8 x luma: white
  turns the spokes about 1/8 turn and swells the hub by about a quarter, so the picture shows as hub sizes (a
  coarse halftone) and spoke angles.
- Black input: the offset is zero and the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  three times the camera's gain.
- The picture does not colour the gears (no Face switch on this one).

## Engine

Chassis lifted from louver ([A] timing / zone / glides / sequencer, [B] generated avid, [C] source field,
[F] output conditioning); [D] and [E] are new.

- Cell coordinates: one 32-bit accumulator per axis = {cell index, 24-bit phase in the cell}; everything about
  the gear is in 1/4096 of the pitch, so one gear description serves every Scale.
- Polar engine (no CORDIC, no square root): fold (dx, dy) into the first octant; ratio = min x recip(max) (512-
  word table, 11 x 12 multiply); angle = 1024-word atan table, unfolded to 14 bits per turn; radius = max x sec
  (512-word table, 11 x 10 multiply).
- Teeth: N x angle is a two-term shift-add; the tooth is a clipped triangle of that phase times the half-depth
  (7 x 8 multiply). The neighbour's teeth inside this cell are this gear's own profile mirrored at the cell
  edge (r + P - 2 max against the opposite tooth sign): exact on the cell edge, so cells join without a seam.
- Spokes: bars, distance from the spoke line = radius x angle offset (9 x 8 multiply).
- Edges: each pixel takes ONE signed distance, to the nearest edge of its radial zone (hole / hub / windows /
  bevel / teeth); coverage in eighths = distance x (1 or 1.5) x 2^-n, a ramp of 1 .. 1.33 px, halved on the
  steep tooth flanks; Line turns it into a band about the edge. Colour = a 9-pair x 9-step blend table refilled
  every field from the palette ROM. Luma 10 bits, chroma 8 bits until the output filter.
- Every multiply is a sum of gated operands over two or three stages; block-RAM data gets a fabric register
  before arithmetic.

One latency in every mode: C_LAT = 32 = processing_delay_clks (25 pixel stages + output conditioning). Per field:
all knob-derived constants (one 24-step serial multiplier, 10 products), the lattice origin, the blend table,
the wave phase and the train angle. Per line: the row accumulator and Spread x row. Per pixel: everything else.

## Status

- Built with router1 through the shared queue (`NEXTPNR_ROUTER="--router router1"`), 2026-11-14. Rebuild:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic cogwork` (router2 was not tried).

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 86.87 MHz | 6916 |
| SD Analog | 1 | 83.86 MHz | 6930 |
| HD HDMI | 1 | 86.40 MHz | 6926 |
| SD HDMI | 1 | 90.11 MHz | 6931 |
| HD Dual | 1 | 88.98 MHz | 6937 |
| SD Dual | 1 | 83.86 MHz | 6927 |

- 6,937 LC at most (90.3 %), 18 of 32 block RAMs. Never flashed. Every config closed on seed 1.
- vmprog: out/rev_b/ron/kinetic/cogwork.vmprog
- GHDL render = integer model, black input, defaults, final sources (the ones the vmprog was built from):
  0 mismatching pixels at 720p (full 1280 x 720 frame, decimation 1), at NTSC (720 x 243 field) and at 1080i
  (1920 x 540 field), after aligning the 4-column generated-avid offset. The NTSC and 1080i captures are four
  fields in and match the model's field 4, so the wave phase and train angle advance as modelled.
- Rasters rendered in GHDL: 720p (black input full frame; Osc and selfie.jpg as 640 x 360 decimation-2
  windows), NTSC, 1080i. Not rendered: 1080p, PAL, 480p / 576p; the other switch and knob settings (Radial, Snap,
  Disc, Line, tooth counts, Scale) were checked in the integer model only.
- The Video / Osc source path has no exact model (the model's hold is approximate); it was checked by eye on the
  two RTL renders: every gear rigid and clean, hubs and spokes following the picture.
- Same picture at SD / 720p / 1080i: yes (virtual 1080-line coordinates; SD shows the centre 1440 of the
  1920-wide canvas, 6.7 gears across instead of 8.9, five rows). Differences: edge softness and Line strokes are
  in real pixels, so they are heavier on SD; in Line mode on SD the thin-line check found one marginal 25-px
  single-row run in one of two frames.

## Cut / deviations

- Idlers (the spec's S10) are cut, as the spec allowed; S10 is Ink Fill / Line instead.
- The spec's "rocking within the backlash" is not applied to the rims: neighbours' teeth are drawn from the
  gear's own mirrored profile, which needs the rims to stay one rigid train. The wave swings the spoked wheel
  and (added, not in the spec) swells the hub, which is what makes the wave readable in a still.
- The neighbour's teeth inside a cell are an approximation away from the cell edge: an intruding tooth tapers
  a little toward its tip, and at 6 teeth the big teeth lean like paddles.
- Radius is max x sec from a table, not r^2; angular resolution is about 0.001 rad (0.1 px at the default
  gear's rim, 0.25 px on the largest gears).
- Coverage gain is quantised to 1 or 1.5 times a power of two (edges 1 .. 1.33 px soft); Line strokes are 3.5
  real pixels at every resolution, so they are relatively heavier on SD.
- Only Scale and Drive are glided. The source is sampled on ONE line per gear (the cell row's first line) with
  no filter along y or time, so a noisy camera may make spokes and hubs shimmer; the smear is 64 real pixels.
- The integer model was written directly at RTL widths (no separate float prototype).
