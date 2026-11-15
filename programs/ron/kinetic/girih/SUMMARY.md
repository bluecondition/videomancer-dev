# Girih (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. **v0.1.1 (K6 camera pass, 2026-11-15) built, not flashed**: see the last section.

## What it draws

Islamic star patterns drawn by Hankin's "polygons in contact" method: from the midpoint of every tile edge two
rays leave at the contact angle, and where rays meet they form stars. A small angle change opens thin sharp
stars into fat rosettes and crosses. At the defaults (Octagon lattice, Filled look, Bauhaus palette) the screen
is a 4.8.8 tiling about 8.5 tiles wide: red eight-pointed stars in the octagons, small red four-pointed stars in
the squares, a blue field between them and gold straps along every ray. A slow wave travels diagonally through
the tiles, so the stars on one side of the screen are sharp while those on the other have opened into rosettes.

## Element and state

- Element: a tile of pitch P. Square lattice: the square tiles. Octagon lattice: the octagons and the small
  squares of the 4.8.8 tiling (each small square is its own element). Tile 0 is centred on the screen centre, so
  Scale zooms about the centre.
- State: the tile's contact angle a (14 bits per turn).
- theta = Spread x index - phase. Linear: index = column + row (the wave sweeps along the diagonal). Radial:
  index = max + min / 2 of (|column|, |row|) (an octagonal ripple from the centre). The small squares sit half
  a tile off on both axes and take the wave at that index.
- a = base (P12) - Depth x wave(theta) - source. The picture uses |sin a| and |cos a|, so an angle swung past 0
  or past a quarter turn folds back. wave is a sine, or Snap: the first sixteenth of the sine stretched over
  the quarter cycle (75 % dwell, eased flips).
- Neighbouring tiles have different angles, but every ray starts exactly on the shared edge midpoint, so the
  straps always stay joined (with a kink at the edge).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | tile pitch 60 .. 480 virtual lines, exponential, glided (default 225: about 5 rows) |
| K2 | Shape | strap half-width: hairline (1.5 px; 2.5 px on SD) .. broad bands (+ 3/32 of the pitch) |
| K3 | Depth | angle swing 0 .. +-90 degrees, square law (default about +-20 degrees) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis (was K6 until the K6 camera pass; the old K5 Spread, the phase step per tile, is fixed at its default 330) |
| K6 | Camera | how far the picture (or the Osc) closes each tile's angle, linear: 0 % = none (the black-input picture), 25 % = v0.1.0's fixed x6 (white = 29 degrees), 50 % (default) = twice it, 100 % = four times. See "K6 camera pass" |
| S7 | Wave | Smooth / Snap (tiles dwell and flip quickly) |
| S8 | Field | Linear (diagonal sweep) / Radial (rings from the centre) |
| S9 | Lattice | Square (four-pointed stars and crosses) / Octagon (4.8.8: eight-pointed stars); default Octagon |
| S10 | Look | Lines (straps in colour A on the ground) / Filled (stars A, field B, straps accent); default Filled |
| S11 | Source | Video / Osc |
| P12 | Angle | base contact angle, glided: 0 % = 81.6 degrees from the tile edge (thin sharp stars with long crossing straps) .. 80 % = 0 (the straps lie along the tile edges) .. 100 % = folded back to 19.6 degrees (fat rosettes) |

Presets: Eight Stars (defaults), Lattice Lines, Ripple Rosette, Snap Tiles, Hairline Screen, Picture Stars.

## Video

- Source = Video: the luma (>>2, less 16) is smeared along x by a 1/64 one-pole filter; on the last line before
  each tile row one sample per tile column is written to a 256-entry block RAM and read back for the whole row,
  so every tile holds ONE value and stays rigid. a -= 6 x luma: white closes the angle by 29 degrees, so bright
  parts of the picture show as fatter stars.
- The top (partial) tile row has its own bank in that RAM, written on line 0 and read from then on, so line 0
  matches the rest of its row (it shows the previous field's line-0 samples).
- Black input: the offset is zero and the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  x16 (2.7 times the camera's gain; full scale = 78 degrees).
- The small squares of the octagon lattice take the wave only (no source), so their four quarters, which lie in
  four different tile cells, always agree.

## Engine

Built on the Louver chassis; sections in girih.vhd are marked [A] .. [F].

- [A] Timing ID -> virtual-1080 steps; vblank zone (no avid for 1024 clocks); source width; glides on Scale and
  Angle; a per-field sequencer with one 24-step serial multiplier (pitch exponent, strap width, depth^2,
  pitch / sqrt 2, spread^2, rate), a serial shifter, the palette read, the 18-entry blend table fill and two
  adder-only origin searches (x, then y).
- [B] Generated avid (parity-locked start 3 / 4 clocks after the source's, field-latched width).
- [C] Source field: Osc lattice, smear, the per-tile hold.
- [D] Tile accumulators (offset in the tile and Spread x tile number: per pixel along x, per line along y); the
  fold (stages 2 .. 9): |x|, |y|, swap about the diagonal, and in Octagon the reflection about the 22.5 degree
  line as (x + y, x - y) x 181 / 256 (shift-adds); then u = |X - P/2|. The wave engine (stages 6 .. 17): index
  -> theta -> quarter-wave sine ROM -> Depth x wave (gated-sum multiply) -> contact angle.
- [E] The look (stages 18 .. 29): |cos|, |sin| ROM (8 bits each); the ray's signed distance
  d = Y |sin a| - u |cos a| (two 10 x 8 gated-sum multiplies over three stages, quarter-pixel units); strap =
  |d| < w; coverage in eighths from |d| - w over 2 virtual px (4 on SD); colour = T(side, coverage), a per-field
  block-RAM table of strap + (side colour - strap) x coverage / 8. Luma 10 bits, chroma 8 bits to the filter.
- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums with the per-line resync, luma [1 2 1]/4
  delayed to the chroma's group delay, luma 13/16 toward black, blanking gate.

One latency in every mode: C_LAT = 36 = processing_delay_clks. Per field: all knob-derived constants, the blend
table, the phase step, the tile origins. Per line: the y offset in the tile row, |y|, Spread x row. Per pixel:
everything else. Every multiply is a sum of gated operands with a stage per level; block-RAM data gets a fabric
register before arithmetic; no RAM ever reads the address it is writing (no_rw_check on all of them).

## Status

- Built with router2 through `./build_set.sh kinetic girih` (rebuild command), 2026-11-14. Every config closed
  on seed 1.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 79.15 MHz | 6000 |
| SD Analog | 1 | 82.69 MHz | 5990 |
| HD HDMI | 1 | 84.20 MHz | 6001 |
| SD HDMI | 1 | 89.62 MHz | 5995 |
| HD Dual | 1 | 94.83 MHz | 6000 |
| SD Dual | 1 | 82.45 MHz | 5990 |

- 6,001 LC (78 %), 15 of 32 block RAMs. Never flashed.
- GHDL render = integer model, 0 mismatching pixels in every case rendered:
  - 720p full frame (1280 x 720), black input, defaults: 0 of 918,720 px
  - 720p timing, 640 x 360 window (decimation 2), defaults, selfie.jpg as the camera: 0 px
  - the same window, Source = Osc: 0 px
  - NTSC (720 x 243 per field, both fields): 0 px; 1080i (960 x 270 window per field, both fields): 0 px
  - the same window, Square + Lines + Radial + Snap with Shape 700, Depth 800, Spread 600, Colour 600: 0 px
  The 4 blank columns at the left of every capture are the generated avid's start offset. Full-frame RTL
  renders of the Osc and selfie cases were not made (the sim slots were full); cmp_final.png shows their
  640 x 360 windows.
- Rasters rendered in GHDL: 720p (full and windowed), NTSC, 1080i (windowed). The SD frame shows the centre
  4:3 of the same picture with the wider SD hairline and edge ramp; nothing else differs.

## Cut / deviations

- No float prototype: the design went straight to the integer model (run notes), which was right on its
  first render.
- The defaults are Lattice = Octagon and Look = Filled (the spec lists Square and Lines first): that frame is
  the poster. Both switches are full look-flips either way.
- Octagon is the true 4.8.8 tiling, not the fallback second ray family.
- The small squares of the octagon lattice ride the wave at their own index but take no Source.
- P12 folds back only 19.6 degrees past zero at 100 %, so the top of the slider is the fat end, not a second
  pass through the thin stars.
- Coverage ramp is 2 virtual px at every HD mode (1.3 px at 720p, 2 px at 1080p: slightly soft at 1080p).
- Depth is quantised to 1/255 of a quarter turn, the wave ROM to 10 bits, |sin| and |cos| to 8 bits over 512
  steps per quarter turn (a ray's far end moves in steps of up to about 1 px at the largest tiles).
- Only Scale and Angle are glided; Shape, Depth and Spread read the knob directly.
- The source smear is 64 real pixels at every resolution (twice as wide in picture terms on SD), and a tile's
  sample is taken near its top right corner (the smear looks left from there).
- With a moving picture, line 0 of the frame shows the previous field's samples for the top tile row (one line,
  same position, one field late).
- At Shape = 0 a near-horizontal hairline is a soft 2 .. 3 px line (the model's run check finds no hard
  single-row lines at 1080p, 720p or NTSC).
- Each tile draws only its own half of a strap: where two neighbouring tiles have different angles the strap
  kinks at the shared edge and its ends are cut square by the tile edge (the centre lines stay joined). With
  the wave alone the step is small; with Source = Osc (or a contrasty camera picture), broad straps and angles
  near zero it shows as flat cuts along tile edges. Not fixed: it needs the neighbour tile's ray per pixel.

## K6 camera pass (2026-11-15), v0.1.1

- **K6 = Camera** (new, 0 .. 100 %, default 50 %): how far the held source closes each tile's contact angle.
  The gain is applied to the smeared sample before it is written to the per-tile hold RAM (which now stores the
  finished 14-bit angle term), as one 8 x 10 multiply split into two 8 x 5 halves and a sum, each in its own
  stage: term = sample x m / 16, m = 3 K6 / 8 for the camera, m = K6 for the Osc (the Osc keeps its 16 / 6 of
  the camera's gain). Linear law:
  - 0 % = none: the black-input picture (with Osc as well);
  - 25 % (K6 256) = Video x6 / Osc x16 = exactly v0.1.0 (white closes the angle by 29 degrees);
  - 50 % (K6 512, default) = twice (white 63 degrees: bright tiles pass through the zero angle);
  - 100 % (K6 1023) = four times (white 126 degrees: the angle folds back, bright tiles cycle through the whole
    star family; the Osc sweeps about 313 degrees).
  K6 has its own glide in the vblank zone (1/8 of the gap per field, +-4 deadband; it honours C_SIM_SNAP).
- Looks (integer-model preview, `.kinetic_work/rb_girih/k6/model/msheet.png`, and the GHDL sheet
  `.kinetic_work/rb_girih/k6/k6sheet.png`: 720p60 decimation 2, the top-left quarter, K6 0 / 256 / 512 / 1023
  and Osc at 512): 25 % is the old subtle shift; 50 % reshapes the stars across the picture (thin stars to
  crosses and octagons where it is bright); 100 % turns most tiles into fat rosettes with small stars between,
  still a girih pattern, not washed out. Every tile stays rigid (one value per tile), so there is no per-line
  noise at any gain; the tile-edge cuts noted below (neighbouring tiles at very different angles) show more often.
- **Frozen: K5 Spread at its default 330** (the serial spread^2 slot now reads the constant). **K6's old
  function, Colour, moved to K5** (same 16 palettes with hysteresis, default Bauhaus); the presets' Colour
  values moved with it and their old Spread values (Ripple Rosette 420, Hairline Screen 600) are dropped.
- The stored sample is the smear 2 px earlier than before (invisible: the smear is 64 px). Latency unchanged (36).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic girih`): **router1, every config on seed 1**.
  Post-route Fmax HD Analog 96.16, HD HDMI 102.01, HD Dual 102.01 MHz (worst HD 96.16); SD Analog 91.99, SD
  HDMI 98.29, SD Dual 99.65. 6,274 .. 6,287 / 7,680 LC (82 %), 15 / 32 EBR. Not flashed.
  Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic girih`.
