# Cogwork (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. v0.1.1 (2026-11-15): K6 camera pass, see the end.

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
| K5 | Colour | the 16 shared palettes with hysteresis (was K6): ground, A / B = gears by parity, accent = hubs |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: state += luma x 0 .. 32 (Osc x3 that); 25 % = the v0.1.0 gain (glided, +-4 deadband) |
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
  column, and is read back for the rest of the cell row. So every gear stays rigid. state += K6 gain x luma
  (v0.1.1: x16 at the default; v0.1.0 was a fixed x8: white turned the spokes about 1/8 turn and swelled the hub by
  about a quarter), so the picture shows as hub sizes (a coarse halftone) and spoke angles.
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

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now Camera: how hard the held camera luma swings the spokes and swells the hubs.

- Gain law (linear): state += luma x K6 / 32 (14 bits per turn), i.e. x0 at 0 %, x8 at 25 % (exactly the v0.1.0
  fixed gain), x16 at 50 % (default: white swells the hub by 219 of its 420 limit and swings the spokes 0.21 turn)
  and x32 at 100 % (white reaches the hub limit; spokes swing 0.43 turn). Source = Osc keeps its x3: x48 at the
  default. 0 % draws exactly what a black input draws.
- K6 is read once per field through the shared glide (a third slot: 1/8 of the gap per field, +-4 deadband),
  turned into the gain in the vblank sequencer (zc 24: K6/4, + K6/2 for Osc), and applied by ONE gated-sum multiply
  (3 stages) on the smeared sample BEFORE it goes into the hold RAM (which was already 16 bits wide: no new block
  RAM). The read side is a plain x2. The written sample is the smear 2 px before the write point: invisible.
- Frozen: K5 Spread at 450 (its default: phase step (450/1023)^2 / 2 = 0.097 cycle per gear) - the C_KFIX_5
  constant. K5 is now Colour (all 16 palettes, unchanged). Spread was the assigned knob and the v0.1.0 sheet agrees
  it is the weakest: its 0 / default / 100 % panels are almost identical in a still.
- Presets: Watch Movement's Spread 620 is gone (it draws at 450 now); the colours moved from K6 to K5; Camera is at
  its 50 % default in all of them.
- GHDL look (720p60, decimation 2, selfie.jpg, glides bypassed in a sim-only copy): .kinetic_work/rb_cogwork/k6/
  k6sheet.png (K6 0 / 25 / 50 / 100 %, Osc at 50 %).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic cogwork`): router1, every config on seed 1. Post-route
  Fmax HD Analog 81.14, SD Analog 82.69, HD HDMI 83.04, SD HDMI 83.04, HD Dual 82.41, SD Dual 82.27 MHz (worst HD
  81.14). 7,174 LC at most (93.4 %, +237 over v0.1.0: the 8 x 10 product and the third glide slot), 18 of 32 block
  RAMs. vmprog: out/rev_b/ron/kinetic/cogwork.vmprog. Not flashed.
- Unsure on hardware: whether 100 % reads as enough of a climax over 50 %. The hub swell hits its 420 / 4096 limit
  at white only near 100 % (K6 about 980), so the top of the knob mostly adds spoke swing (up to 0.43 turn) and fills more hubs
  to the limit; with Osc the hubs plateau at the limit over much of the lattice from 50 % up.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron, after the fern / kiln K6 = 100 % renders: a slightly stronger camera effect at the top of the knob.
The orchestrator set the K6 default to 100 % (1023) and bumped the version; this pass changed the logic.

- Classification: **element** (K6 scales the held luma that swings the spokes and swells the hubs: it bends the
  drawing, it never shows the camera image).
- Change: the per-field gain is x1.25 at every knob position (new sequencer slot zc 25: c_cg += c_cg >> 2), so
  Video = luma x 0 .. 40 (was 0 .. 32; 50 % = x20), Osc x3 that (max gain 957 of 1023). The hold's gated-sum
  product still fits its 14-bit word (255 x 957 / 16 = 15,252). The state sum (Depth wave + source) grew from
  16 to 17 bits, because the Osc source at x1.25 reaches 30,504 + 8,191 and would have wrapped the hub swell's
  sign; the hub clamp (420 / -230) now reads bits 16 .. 4 and the spoke angle still takes the low 12 bits.
  At 100 % white now reaches the hub limit at K6 about 790 and swings the spokes 0.53 turn.
- Look (GHDL, bear.png, 720p60 / 4, glides bypassed in the scratch copies): `.kinetic_work/rb_cogwork/k6b/
  k6b_cmp.png`. Same gear wall; the spokes sit at different angles (more swing) and the hubs are a little
  fuller. No wrap or new artifacts.
- Build (`.kinetic_work/tools/k6build.sh kinetic cogwork`): router1; HD HDMI on **seed 2**, the rest seed 1.
  Post-route Fmax HD Analog 82.27, HD HDMI 87.62, HD Dual 90.56, SD Analog 91.90, SD HDMI 83.04, SD Dual
  86.45 MHz (worst HD 82.27). 7,208 LC at most (93.9 %), 18 / 32 block RAMs. vmprog:
  out/rev_b/ron/kinetic/cogwork.vmprog. Not flashed.
