# Miura (Kinetic wave 2)

v0.1.0 built 2026-11-22, not flashed.  Flash `out/rev_b/ron/kinetic/miura.vmprog`; rebuild with `./build_set.sh kinetic miura`.

## What it draws

A folded paper sheet: Koryo Miura's map fold (Miura-ori), as in the origami tessellations of Ron Resch and
Tomohiro Tachi. Horizontal creases cut the screen into rows; zigzag creases cut every row into parallelogram
facets that lean right in one row and left in the next, so the zigzags run down the sheet as a herringbone.
Lit from the side, the facets take four tones (two column parities x two row parities). A wave of folding
travels down the sheet: a row that folds gets shorter, its creases lean harder (they still meet exactly on the
shared crease), and its facets split into a bright and a dark tone; a row that opens grows tall, its facets
stand nearly upright and go one tone. Row 0 is centred on the screen centre and the sheet breathes about the
middle like an accordion.

Default frame (Terra palette): terracotta and sand paper rows, teal mountain creases and dark valley creases, a
band of tightly folded rows in the upper half and tall, flat rows below; the band rolls down the sheet, about
11 s per cycle.

## Element and state

- Element: a row j (index Linear = j, Radial = |j|). Spread is frozen at 0.085 cycle per row.
- State: the fold z_j = Fold (P12) + Depth * wave(phase - 0.085 index) + camera row mean, in logit units,
  clamped to +-4; the fold angle phi = 1/8 turn * (1 + sin(pi/2 z/4)), an S-curve from flat to shut.
- Geometry: row height H = max(12, H0 cos phi) virtual lines (H0 = the facet pitch); crease x offset A v / H on
  even rows and A - A v / H on odd rows (v = line offset in the row), so the zigzags meet exactly at every
  horizontal crease.
- Tones: sin phi * (+-cx -+ cy), Side light (cx 27, cy 4 in 32nds) or Top (7, 24). Colour: paper A on even rows,
  B on odd, through a per-field table: tone < 0 toward black, tone > 0 toward white.
- Creases: mountain = accent, valley = the paper at its darkest; weight 8 virtual px (at most 0.3 H), scaled
  1/2 .. 1 by the fold (thin on flat rows). M/V: zigzag crease k by k parity, horizontal segments by (k + row)
  parity, as in a real Miura-ori.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | facet pitch 56 .. 317 virtual px (exponential, glided); a flat row is one pitch tall |
| K2 | Shape | zigzag amplitude A = 0 .. 0.62 pitch: 0 = straight pleats (an accordion of rectangles), top = sharp herringbone |
| K3 | Depth | fold swing 0 .. 3 logit, square law (0 = every row folds alike: a static chevron sheet) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared Kinetic palettes (A, B = the two papers, accent = mountain creases), with hysteresis |
| K6 | Camera | camera (and Osc) strength 0 .. 100 %, default 100 %: per-facet luma steps that facet's tone, the row mean adds to the row's fold |
| S7 | Wave | Smooth (sine) / Snap (rows dwell open or shut and flip quickly) |
| S8 | Field | Linear (the wave runs down the sheet) / Radial (rows fold from the middle out, mirror-symmetric) |
| S9 | Creases | Off (tone edges only; the row edge blends toward the neighbouring row) / On (default) |
| S10 | Light | Side (bright facets form zigzag columns) / Top (mountain / valley rows alternate bright and dark) |
| S11 | Source | Video / Osc |
| P12 | Fold | the mean fold: 0 % = an almost flat sheet with faint creases and a gentle ripple, 100 % = folded nearly shut: thin rows, razor zigzags, hard contrast, still breathing |

Presets: Folding Sheet (defaults), Accordion, Herringbone, Ripple Map, Snap Fold, Paper Portrait.

## Video

`program_type = "processing"`. The luma (Y/4 less 16, 8 bits) enters on the "sample line" of each row (the line
just before it, or line 0 for the top row): it is summed per facet column and each column's sum times the K6
gain is stored for the row (two block-RAM banks by row parity); the line total gives the row mean. A facet's tone
moves by 4 tone units (of +-32) per unit luma above or below the row mean, so a grey picture leaves the relief
alone and nothing washes out, and the row mean (up to 1.6 logit) folds bright rows harder in the next field.
Black input = no offset, K6 0 % = exactly the black-input picture. Osc substitutes the shell's two-axis
triangle lattice (640 x 540 virtual px, drifting diagonally) at twice the camera gain: diagonal bands of dark
and pale facets and rows folding in a drifting pattern.

## Engine

- Per field (vblank zone, "no avid for 1024 clocks"; the engine restarts from state 1 every field, which is
  its watchdog): knob glides (Scale, Fold, Camera), a 58-state sequencer on one 16-step serial multiplier and
  one 28-step serial divider: pitch, amplitude, depth, fold, rate (phase step), camera gains, the column origin;
  the walk (row 0 centred on line 540, rows above stacked until one reaches the top); the first two rows; the
  line-0 kick. In parallel a fill writes the facet table (2 papers x 64 tones, one channel per pass).
- Per row (when the beam enters a row, the row unit computes the next one, ~250 clocks): wave -> fold -> S-curve
  -> sin, cos (one quarter-wave ROM, four reads) -> height, two tones, lean (divide), AA ramp (divide), crease
  weights.
- Per line (line engine, own 17-step multiplier, kicked at every line start for the next line): row advance,
  sample-line flag, crease x offset -> column start, row-edge weight, horizontal crease coverage; after the line
  leaves the pixel path it reads the neighbouring row's colours and the valley colour from the facet table and
  preloads the camera window. At the line end the camera writer's row products use the same multiplier.
- Per pixel (18 stages): column accumulator (pos += 1 px, wrap at the pitch) -> distance to the nearer crease ->
  coverage over a ramp of (1 + lean) px -> the two facet tones + camera -> tone AA -> facet table -> crease or
  row-edge overlay -> output conditioning (luma [1 2 1]/4, chroma [1 2 2 2 2 2 2 2 1]/16 with resync, luma 13/16,
  blanking gate). One latency, C_LAT = 25, in every mode; generated avid; virtual 1080-line coordinates.

## Status

- v0.1.0 built 2026-11-22 through the shared queue with router1 (`NEXTPNR_ROUTER="--router router1"`); every
  config closed on seed 1. Not flashed. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic miura`.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 90.27 MHz | 6784 |
| SD Analog | 1 | 90.24 MHz | 6793 |
| HD HDMI | 1 | 83.89 MHz | 6789 |
| SD HDMI | 1 | 92.07 MHz | 6793 |
| HD Dual | 1 | 74.72 MHz (thin margin) | 6784 |
| SD Dual | 1 | 81.45 MHz | 6786 |

- 6,793 / 7,680 LC (88.5 %), 13 / 32 block RAMs. Six bitstreams pass iceunpack. HD Dual passes by 0.5 MHz on
  this routing (the previous build of nearly the same netlist gave 88.5 MHz there), so a rebuild may need a
  second seed for HD Dual.
- History: the first RTL was ~9,100 LC (standalone estimate); cuts brought it to 7,463 (placement failed),
  then 7,036 (HD Dual failed placement on seed 1; seeds 2 and 3 placed and passed at 81.6 / 85.3 MHz in a
  direct nextpnr run), then 6,794. A router2 build of an intermediate version reached only 61 MHz (the
  engine's clock-enable cone); the engine's stall became a registered wait flag, after which router1 closed
  everything on seed 1. router1 never stalled; dupin on an intermediate netlist found no duplicated-input
  cells.
- Model = RTL, decimation 2 (GHDL, 720p60 -> 640 x 360, an earlier RTL): pixel diff 0 at the defaults with a
  black input; 0 with Creases Off + Light Top + Snap + Radial + K5 0 (Ink); 0 at field 30 (motion); with
  selfie.jpg 340 px differed, all in an unsampled sliver column at the right edge (fixed since: each sample
  line copies its edge columns outward). Osc + K5 1023 at decimation 2: same palette and structure, ~27 % of
  pixels differ in tone (Osc is matched at full raster only, below).
- Model = RTL, full raster, the shipped RTL (md5 ec23619b, GHDL, decimation 1; the 4-px generated-avid offset
  excluded): 720p defaults, black input (rtl_default.png): 0 / 918,720 px; 720p Osc: 0 / 918,720;
  NTSC 480i (the tester stacks the two fields): both fields 0 / 173,988 (model fields 4 and 5);
  1080i59.94: both fields 0 / 1,034,640. field_n is ignored, so a hardware 1080i / SD source (field_n tied
  high) draws the same.
- 720p selfie.jpg, frame 3: 141,488 px differ, all above row -1 (the top 294 lines). Traced with an
  instrumented copy of the RTL (sim only): in the sim's FIRST field the camera's line totals match no source
  line, so that field's rowcam writes differ from the model's, and the camera feedback (row mean -> row
  height -> which line samples the next row) carries the difference upward for a few fields. Seeded with the
  RTL's field-0 rowcam writes, the model matches frame 3 exactly (0 / 918,720), and the RTL's field-1 writes
  equal the model's. So it is a one-field start-up transient (why the first field's totals differ was not
  traced; likely the source width / generated avid not yet settled in that field). On hardware it lasts a
  few fields after loading.
- Renders: `.kinetic_work/rb_miura/cmp_final.png`, `sheet.png`, `strip.png`, `camera.png`, `presets.png`,
  `rtl_default.png` (720p60 / 1, defaults, black input).

## Cut / deviations

- Highlights shade toward WHITE, not toward the accent as the spec said: renders of four palettes showed the
  accent muddied highlights (Ink went pink, Bauhaus blue went olive). The accent colours the mountain creases.
- The fold is a logit z with an S-curve to the angle (not Fold + Depth * wave on the angle directly), so P12 at
  100 % keeps nearly every row shut while Depth still breathes them; a linear angle clipped half the wave at
  "shut" and the 100 % frame did not read as thin rows. P12 0 % still ripples gently, so Shape and Light stay
  visible (that is the spec's floor).
- Light Top's row term is sin(phi), not (1 - cos phi): with (1 - cos) the Top/Side flip was invisible at the
  default fold.
- Camera: the facet value is the luma along the facet's TOP edge (the row's sample line), not its centre (no
  frame store fits); the facet tone moves with the luma relative to the row's mean, so a grey picture leaves
  the relief alone. The facet-edge anti-aliasing uses this facet's camera value on both sides (a half-step at
  a facet edge in camera mode, under the crease line when creases are on). A sliver column that enters the
  frame later in a row takes its neighbour's value.
- Creases: the zigzag crease's half-width is a fixed fraction of the AA ramp (1 + lean) px, so its
  perpendicular weight is 1 .. 1.41 x the nominal 8 virtual px (exact at vertical and near-horizontal leans);
  crease "strength" with the fold is a width (1/2 .. 1), not an opacity. Chroma of the crease blend is in
  quarters (luma in sixteenths).
- Creases Off: row edges are clean horizontal steps (a whole field line), not anti-aliased: the row-edge blend
  (neighbouring row colours, ~200 LC) was cut to fit HD Dual. Facet edges stay anti-aliased.
- The AA ramp reciprocal is 7 bits in the pixel multiply (ramp width within a few % except at the steepest
  leans).
- Interlaced SD: the horizontal crease ramp is three field lines (two elsewhere) so a thin row's crease is never
  a hard one-line run; the thin-line check finds 0 hard runs at 720p and NTSC.
- Spread is frozen at 0.085 cycle per row (K5 = Colour, per the wave-2 rules).
- Same picture at SD / 720p / 1080p (virtual 1080-line coordinates); SD shows the centre 4:3, so fewer facet
  columns; interlaced SD rows are at most ~2.7 field lines at the thinnest (12 virtual lines).
