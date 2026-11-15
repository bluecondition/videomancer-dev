# Vega (Kinetic)

**v0.1.1 (K6 camera pass) 2026-11-15, not flashed** (v0.1.0 built 2026-11-14). Flash `out/rev_b/ron/kinetic/vega.vmprog`.

## What it draws

Victor Vasarely's Vega series ("Vega-Nor" 1969, "Vega 200"): a regular grid of inset shapes that swells as if
stretched over a sphere pushing out of the canvas, the cells growing at the pole and crowding at the rim. The
default frame (Vega palette) is a 28 x 16 grid of discs on deep blue, coloured in a diagonal scale cyan ->
magenta -> yellow, with a globe 1,024 virtual lines across bulging out of it: discs nearly twice the size at its
pole, squeezed to slivers at its rim. A slow wave of swelling and shrinking travels diagonally through the cells,
and the globe drifts slowly from side to side on its own.

## Element and state

- Element: a grid cell of pitch P. Cell (0, 0) is centred on the screen centre, so Scale zooms about the centre.
- State: the size of the cell's inset shape, as a fraction of the half cell (0 .. 512).
- theta = Spread x index - phase. Linear: index = i + j (the wave runs along the diagonal). Radial: index =
  max(|i|, |j|) + min / 2 (an octagonal ripple from the centre cell).
- size = 280 + Depth x wave(theta) + source, clamped 56 .. 472 (so the shapes never vanish and never touch: the
  gap is at least 8 % of the pitch). wave is a sine, or Snap: the first sixteenth of the sine stretched over the
  quarter cycle (75 % dwell, eased flips).
- Lens: u' = u - (u - centre) x m, m = Bulge x h(r^2 / R^2), R = 512 virtual px. Bulge: h = 0.70 sqrt(1 - s), the
  height of a hemisphere, so the magnification is 1 / (1 - 0.70 Bulge) at the pole (3.3x at 100 %) and the
  compression grows without bound at the rim; the map is continuous with the flat grid outside. Pinch:
  m = -1.36 Bulge (1 - s)^3: the cells crowd at the centre (2.4x smaller at 100 %) inside a ring of stretched ones.
- The lens centre wanders on its own: +-256 virtual px in x (29 s), +-32 in y (40 s).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | grid pitch 24 .. 255 virtual lines, exponential, glided (default 68: 28 x 16 cells) |
| K2 | Shape | the inset shape in 24 steps: square .. chamfered square .. regular octagon .. disc (default disc) |
| K3 | Depth | wave swing of the size, square law, 0 .. +-43 % of the half cell (default +-16 %) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis: ground, A = the cells, B and accent = the Grade scale (K6 until v0.1.0; Spread is now fixed at its old default 274, a phase step of 0.036 cycle per cell) |
| K6 | Camera | camera (and Osc) strength, 0 .. 100 %, default 50 %: 0 % = no camera (the black-input picture), 25 % = half the v0.1.0 gain, 50 % = the v0.1.0 gain (luma x 1.5), 100 % = 1.37x it (see "K6 camera pass") |
| S7 | Wave | Smooth / Snap (cells dwell at big and small and flip quickly) |
| S8 | Field | Linear (diagonal wave and colour scale) / Radial (rings from the centre) |
| S9 | Lens | Bulge (a globe) / Pinch (a funnel) |
| S10 | Grade | Off (every cell is colour A) / On (Vasarely's colour scale: A -> B -> accent -> B -> A in 16 steps every 32 cells, along the Field's index) (default On) |
| S11 | Source | Video / Osc |
| P12 | BULGE | the headline, glided: 0 % a flat grid .. 100 % a globe filling the screen height, 3.3x cells at its pole and a crowded rim |

Presets: Vega (defaults), Vega-Nor, Ink Globe, Funnel, Flip Dots, Halftone Lens.

## Video

`program_type = "processing"`. The picture's luma is the second modulator and ADDS TO THE STATE (the cell's size):
luma (8 bits, less black) -> a 1/16 one-pole smear along the line -> x 1.5 -> held per cell, so white swells a
cell by 64 % of the half cell (up to the 92 % cap) and the picture shows as a swelling halftone that the lens
then magnifies. The hold: a 256-word block RAM addressed by {cell row parity, cell column} and tagged with
{field parity, cell row}; the first pixel of a cell on its first line stores the smeared luma there and every
later line of that cell reads it back, so each shape is rigid (no per-line edges). With a black input the term is
zero and the wave alone runs. Source = Osc substitutes the shell's drifting two-axis triangle lattice (640 x 540
virtual px) at twice the gain, so big diamonds of full-size cells drift across.

## Engine

Chassis sections lifted from louver.vhd ([A] timing / zone / glides / sequencer, [B] generated avid, [C] source,
[F] output conditioning); the geometry is new.

- Per field (vblank zone, "no avid for 1024 clocks"): glides on Scale and Bulge; one 38-bit serial multiplier,
  15 products 32 clocks apart (pitch exponent, 1 / mantissa, the u / v start values and steps, the lens centre,
  dx^2 and dy^2 at the first pixel / line and their first differences, depth^2, spread^2, rate); the palette;
  the blend table (16 colour steps x 16 coverages, adders only); the wave table (256 entries by signed half-cell
  index: 280 +- Depth x wave and the colour step; 12 clocks per entry, the Depth multiply done there by
  shift-adds, about 3,100 clocks); the lens centre's two sines from the same ROM.
- Per line: v, dy^2 (second differences) and v - centre for the next line.
- Per pixel (26 stages): u (norm px = virtual px / pitch mantissa, so a cell is 16 x 2^n) and dx^2 accumulators
  -> r^2 -> lens ROM (1,024 steps of r^2 per profile) and ramp ROM -> m = Bulge x h (8 x 10 bits) -> two
  products (u - centre) x m, (v - centre) x m (13 x 10 bits each, gated sums over four stages) -> u', v' ->
  cell id (high bits) and |sub-cell coordinate| (low bits) -> index -> wave table; source hold -> size;
  metric max(hi, c45 (hi + lo), k x 16-gon) -> metric - size -> x ramp slope 2^(L/4) (L = pitch mantissa and video
  mode + a ROM of r^2 and Bulge holding -4 log2 of the rms of the lens's radial and tangential stretch, so
  edges stay about one pixel wide at the pole and soften toward the crowded rim instead of aliasing) ->
  coverage in sixteenths -> blend table -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced
  8-sums, luma [1 2 1]/4 delayed to the same group delay, luma 13/16 toward black) -> blanking gate.
- One latency in every mode: C_LAT = 33 = processing_delay_clks. Generated avid (parity-locked start), virtual
  1080-line coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention
  (U register = Cr). The output is neutral until the first field's tables are filled. No RAM uses data read
  from the address being written (no_rw_check on all of them).

## Status

**Shipped vmprog: the ROUTER1 rebuild of 2026-11-14** (same sources). Post-route Fmax: HD Analog 84.85 MHz; SD Analog 86.03 MHz; HD HDMI 88.04 MHz; SD HDMI 94.14 MHz; HD Dual 83.13 MHz; SD Dual 83.65 MHz. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic vega`. The router2 figures below are the first build, kept for the record; it was timing-marginal.


- Built 2026-11-14 through the shared queue with router2 (the builder default), every config on seed 1.
  Rebuild: `./build_set.sh kinetic vega`.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 74.73 MHz | 7183 |
| SD Analog | 1 | 83.37 MHz | 7173 |
| HD HDMI | 1 | 78.09 MHz | 7187 |
| SD HDMI | 1 | 83.21 MHz | 7187 |
| HD Dual | 1 | 75.08 MHz | 7184 |
| SD Dual | 1 | 77.30 MHz | 7183 |

- 7,187 / 7,680 LC (93.6 %, a little over the set's 7,100 target), 22 / 32 block RAMs. Six bitstreams pass
  iceunpack. HD needs 74.25 MHz: HD Analog (+0.5 MHz) and HD Dual (+0.8 MHz) pass with little margin, and router2
  is not deterministic, so a rebuild may need another seed or router1 (the synth netlist has no carry self-adds:
  dupin 0). Standalone yosys: no `emulate_transparency`.
- Model = RTL (GHDL, the sources the vmprog was built from, timing 720p60, frame 3 after power-up, the capture
  aligned 4 columns for the generated avid's start):
  - defaults, black input, 1280 x 720 (decimation 1): 0 mismatching pixels of 918,720;
  - Snap + Radial + Pinch, Scale 200, Shape 400 (chamfered), Depth 900, Spread 600, palette 2, BULGE 900
    (640 x 360, glides bypassed): 0 of 228,960;
  - square, Scale 900, Depth 300, reverse Rate, Spread 100, palette 13, Grade Off, BULGE 100 %: 0 of 228,960;
  - Source = Osc (640 x 360): 0 of 228,960;
  - Video = selfie.jpg (640 x 360): 15,648 pixels differ (6.8 %, mean 0.38 code, max 57), all as thin rings
    around some cells: those cells' sizes differ slightly. Put down to the tester scaling the photo differently
    from the model's resize (the Osc render, which has no such input, is exact); not proven.
  The model matches at frame 3 only (frames 2 and 4 differ in 10 % of pixels), so the per-field phase and
  lens-wander steps are exercised. The 640 x 360 renders are the tester's decimation 2: the top-left quarter of
  the 720p canvas at full scale.
- NOT rendered: NTSC / 480p / 1080i / 1080p rasters (the sim queue was saturated; the virtual-1080 steps are the
  pilot's and the model has those modes, but no RTL frame was compared). No second, later capture either.
- Single-row hard lines: none in the model's default, square, Scale 0 / max, BULGE 0 / 100 %, Pinch frames
  (720p and 1080p, run counter after louver's thincheck).
- Renders: `.kinetic_work/rb_vega/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion), all from
  the integer model `imodel.py` except the RTL panels of cmp_final.

## Cut / deviations

- The lens table holds the hemisphere profile h(s), not g itself: m = Bulge x h is one more (8 x 10) multiply per
  pixel, so the per-pixel count is three small multiplies, not two. The Depth multiply moved into the vblank table.
- The disc is a 16-sided polygon (faces at 0, 22.5 and 45 degrees), not a true circle: up to 1.9 % of the radius
  out of round, which shows as faint facets once a disc is more than about 100 px across (Scale near the top,
  or the pole at BULGE 100 %). A 32-gon version matched the model too but did not fit.
- First RTL: 8,221 LUT + FF (about 108 % of the chip). Cut to fit: lens products 14 x 12 -> 13 x 10 bits, the Depth
  multiply into the wave-table fill, 32 -> 16 colour steps in the blend table, the 32-gon -> 16-gon, three
  pipeline stages, per-field scratch registers. It still ends at 93.6 %, over the 7,100 LC target.
- Grade's scale has 16 steps (A -> B in 8, B -> accent in 8; the last step stops 1/8 short of the pure accent).
- Shape is not glided and has 24 steps; Depth and Spread read the knob directly. Only Scale and BULGE glide.
- The lens radius is fixed (512 virtual px); BULGE sets the strength only. The lens is 0.2 % taller than round
  on 480i (its y step is truncated to 1/16 half-pixel); other modes are exact.
- Pinch uses its own profile (1 - s)^3 so the map never folds; it is not the Bulge profile negated.
- Source gain: with the base size at 55 % of the half cell and luma x 1.5, the size reaches its 92 % cap at
  about 40 % grey, so a bright picture reads as "all cells full" with only its darker areas shrinking. A lower
  gain (x 1) or a lower base would give the halftone more range; not changed for lack of time.
- S7 Smooth / Snap is mainly a difference in MOTION (dwell and flip); on a still frame at the defaults it is
  the weakest of the five switches.
- The per-cell hold can fail in the most crowded rim cells: its RAM is addressed by {row parity, column}, so two
  cells of one column two rows apart that share a scanline (rows squeezed under about half a column's width)
  re-sample each other every line. In the model with the selfie: 0 of 493 cells at the defaults, 0 of 3,642 at
  Scale 0, 11 of 1,476 at Scale 200 + BULGE 100 % (rim slivers). Two row bits in the address would halve it again.
- The source smear is 16 real pixels at every resolution (twice as wide in picture terms on SD), and the luma
  is sampled at the first pixel of each cell's top line (its top-left corner), not its centre.
- The edge ramp follows the rms of the lens's two stretches, so close to the globe's rim edges that run along
  the radius are a little soft and edges across it a little sharp; the last few pixels inside the rim fade
  toward the mean colour by design.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 = Camera**, 0 .. 100 %, default 50 % (initial 512): the gain of the camera's (and the Osc lattice's)
  term in the cell size. size = 280 + Depth x wave + smear x c, clamped 56 .. 472 (of 512), c = 1.5 g for
  Video and 2 g for Osc (the old ratio), g = K6 / 512 up to 50 % and 3/8 of that slope above
  (g = 1 + 3 (K6 - 512) / 4096):
  - 0 % (g 0): no source term, exactly the black-input picture (the wave alone);
  - 25 % (g 0.5, luma x 0.75): a calm halftone, almost no cell reaches the cap;
  - 50 % (g 1, luma x 1.5, default): the v0.1.0 gain, bit for bit, and twice the 25 % point. v0.1.0 was
    already "too hot" (cells full at ~40 % grey), so twice the shipped gain would wash out: the 25 % point
    sits at half the shipped gain instead, and the default is the strongest clean level, the shipped look;
  - 100 % (g 1.37, luma x 2.06): bold, most cells of a mid-key picture swell to the cap and only the darker
    areas keep small cells. A straight half-slope (g 1.5, luma x 2.25) left the selfie almost uniformly full,
    so the top of the range was pulled in. (A numpy check on the selfie at the mean base size: 14 % of
    pixels at the cap for x 1.5, 40 % for x 2.0, 50 % for x 2.25, 81 % for x 3.)
- How: K6 is glided as a third channel of the shared glide (deadband +-4 codes, 1/8 of the gap per field);
  the knee value and c (Q5, 0 .. 87) are formed once per field (zone clocks 40, 41). The source path: the
  1/16 smear x c as two gated partial sums, then their sum / 32 (one register more than v0.1.0's x1.5 shift-add,
  so the hold samples the smear one pixel later; the main pixel pipeline and the latency (33) are unchanged).
- **K5 = Colour** (the 16 palettes, was K6); presets keep their palettes on K5.
- **Spread is fixed** at its old default 274 (0.036 cycle per cell). Its sheet panels at 0 / 274 / 1023 were the
  closest of the knobs (Rate is the only speed control and stays). Presets lost their Spread values
  (Vega-Nor 180, Ink Globe 420, Funnel 350, Flip Dots 560).
- Look check: `.kinetic_work/rb_vega/k6sheet.png` (GHDL, selfie, 720p60 / 2 = the top-left quarter of the
  frame, glides snapped; K6 = 0 / 256 / 512 / 1023 and Osc at 512).
- Build (router1 through the queue, 2026-11-15): all six configs on seed 1. Post-route Fmax HD Analog
  86.27 MHz, SD Analog 90.65, HD HDMI 86.39, SD HDMI 92.34, HD Dual 85.11, SD Dual 84.05 (worst HD 85.11).
  7,366 / 7,680 LC (95.9 %, up from 93.6 %: the two partial-sum adders and the third glide channel), 22 / 32
  EBR. It closed with 15 % margin despite the LC, but there is little room left for anything else.
  Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic vega`.

