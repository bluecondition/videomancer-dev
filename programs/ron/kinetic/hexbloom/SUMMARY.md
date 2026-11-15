# Hexbloom (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/hexbloom.vmprog`.

## What it draws

Honeycomb op-art: a lattice of hexagonal cells, each one a nest of concentric hexagons that pumps outwards
and back. The default frame (Vega palette) is about 8 cells across and 5 rows down, every cell three rings
deep in cyan and magenta on deep blue, the cells divided by a yellow grout line. A travelling wave runs
across the lattice: cells along one lattice axis move in step, so diagonal ranks of cells bloom one after
another (about 7 s per cycle at the default Rate).

## Element and state

Element: a hexagonal cell (pointy-top; a cell centre sits on the screen centre, so Scale zooms about it).
State: the ring phase of its nest.

    cell   = the nearer of two lattice centres (this row's or the next row's): one linear test, exact hexagons
    a, b, c = the three hexagonal axes of the position inside the cell
    T1 = max(a, b, c)    T2 = max(-a, -b, -c)                 (the two triangle metrics)
    d      = T1 + g (T2 - T1),  g = alpha where T2 >= T1, beta otherwise
             hexagram (0, 1) .. hexagon (1, 0) .. triangle (0, 0), 16 steps each way
    PHI    = d * K - state                                     ring = floor(PHI)
    state  = Depth * wave(phase + Spread * index) + luma held per cell
    index  = (h + j) / 2  (Linear)  /  hexagonal distance from the centre cell  (Radial)

Rings alternate colour A / colour B over the ground (duty 1/2 in Bands, 1/4 in Lines). The hexagon metric
max(T1, T2) is also the distance to the cell edge, which draws the grout.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 64 .. 512 virtual px, exponential, glided (default 235 px: 8 cells across) |
| K2 | Shape | the nest's metric: hexagram (star of David) at the bottom, hexagon at the centre (with a dead band), triangle at the top; 16 steps each way |
| K3 | Depth | how far the rings pump, 0 .. 4 rings, square law (default 0.74 ring) |
| K4 | Rate | wave travel, bipolar, centre = still; exponential, 60 s per cycle just off the dead band to 2 cycles per second at either end |
| K5 | Colour | the 16 shared Kinetic palettes (ground, ring A, ring B, grout = accent), with hysteresis; default Vega (was K6 until v0.1.1) |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: gain k / 256 on the held luma (0 = as with a black input, 25 % = the v0.1.0 strength, 50 % = twice it, 100 % = four times); scales Osc too. Spread (K5 until v0.1.1) is fixed at 560 (0.15 cycle per cell) |
| S7 | Wave | Smooth (sine) / Snap (3/4 of the cycle dwells at the extremes, eased flips between) |
| S8 | Field | Linear (the wave crosses the lattice; cells along one lattice axis move together) / Radial (the wave radiates from the centre cell in hexagonal rings of cells) |
| S9 | Rings | Bands (filled half-period rings) / Lines (thin outline rings on the ground) |
| S10 | Grout | On (an accent-colour line between cells, 3/64 of the pitch wide) / Off (cells butt together with a soft seam) |
| S11 | Source | Video / Osc |
| P12 | BLOOM | the headline: ring density, exponential, from 2 rings per cell (one hexagon in a frame, flipping colour as the wave passes) to one ring every 12 virtual px (at the default scale about 20 rings per cell: fine nests shimmering as they pump). The top of the range follows the cell size, so 100 % is always the finest rings that still hold their colour |

## Video

`program_type = "processing"`. The picture's luma is the second modulator and adds to the same state, the
ring phase: luma -> 32-pixel box -> ONE sample per cell, taken where the beam crosses the cell's centre
column on the row's sample line (the first line 1/4 of a row pitch below the row above, before any of the
cell is drawn), written into a 2 x 128 block RAM (row parity, cell column) and read back for the whole cell.
So every nest stays rigid: a bright cell's rings sit up to 2 rings further out than a dark cell's ("cells
bloom where the picture is bright"), and the wave pumps on top. With a black input the offset is zero and
the wave alone runs. Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same
point at twice the gain (up to about 3.5 rings), so broad diamonds of shifted nests drift across.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): knob glides (K1, K2, K3, K5, P12), one serial
  16-step multiplier in 14 slots (pitch through a 2^(i/256) ROM, row pitch, row step per line, the first
  line's row coordinate, the beam's start cell and step, spread, the wave index at the line start and at
  line 0, ring density, rings per pixel, depth, rate), the phase step, the Depth x magnitude table refill
  (64 entries by accumulation), the palette, then the line-0 kick. About 600 zone clocks.
- **Per line** (horizontal blanking, 5 clocks, no multiplier): the row coordinate steps; from it the row
  parity, the vertical axis term of this row's and the next row's cells, the cell-test threshold, the two
  rows' wave indices (accumulated when the row changes) and the sample-line flag.
- **Per pixel** (33 stages): beam accumulator (integer part = cell column, fraction = position across the
  cell) and a second accumulator for the wave index that steps when the first wraps -> cell test and select
  -> the two triangle metrics -> shape blend (5-bit weights, gated shift-adds) -> d * K (the only
  multiplier, 11 x 11 in two halves) -> minus held luma -> minus Depth * wave (quarter-wave sine ROM, 4096
  points per cycle, Snap by address saturation, Depth table) -> PHI -> ring coverage by a 4-step divider
  (17 levels; rings per pixel is constant over the field and never above 1/6) -> grout coverage from the
  hexagon metric (17 levels, edge width by a per-field shift) -> one shift-add blend base + (target - base)
  * weight -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 at
  the same group delay, luma 13/16 toward black) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (40) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention (U = Cr).
- Chassis from `riley.vhd` ([A] timing, [B] controls, [C] sequencer, [D] Osc + box, [E] divider + blend,
  [F] output); the line engine, beam, cell geometry, wave index and luma hold are new.

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now **Camera**, the camera strength (Ron's rule for the Heirloom / Kinetic / Patchbay sets).

- What it does: K6 scales the per-cell held source (the luma box, or the Osc lattice) that is subtracted from
  the ring phase, so it sets how far a bright cell's rings sit out from a dark cell's. Source = Osc keeps its
  2x on top.
- Gain law: linear, gain = k / 256 (k = the knob, 0 .. 1023). Decoded per field in the vblank sequencer (zone
  clocks 24 .. 27): the knob is latched with a +-4 deadband (the ends snap to 0 / 1023), then the gain eases 1/8
  of the gap per field. Applied with one 8 x 10 gated-sum multiply over three pixel stages on the value read
  back from the hold RAM (stages 8 .. 10, in place of the old s_ld delay; same latency, the RAM is unchanged).
  The ring phase wraps modulo 16 rings as before.
  - 0 % = gain 0: exactly what a black input draws;
  - 25 % (256) = 1x, the v0.1.0 strength (white = 1.7 rings of offset);
  - 50 % (512, default) = 2x (white = 3.4 rings): neighbouring cells' nests sit at clearly different phases;
    every nest stays rigid and clean;
  - 100 % (1023) = 4x (white = 6.8 rings): the luma range runs through more than three A / B colour cycles, so
    the cells look phase-scattered in a still and flicker through their rings as the picture moves. Still
    hexagonal nests; no washout.
- Frozen: K5 Spread at 560 (its default: 0.15 cycle per cell, a wave about 7 cells long). Spread was the
  weakest knob on the control sheet (min / default / max look alike in a still; Rate, the only speed control,
  was not touchable).
- K6's old function (Colour, all 16 palettes, value labels, default Vega) moved to K5 with its hysteresis.
  Presets: Colour values moved to K5; their Spread values (420, 640, 300, 500, 380) are gone, so every preset
  now runs at Spread 0.15.
- Renders: `.kinetic_work/rb_hexbloom/k6sheet.png` (selfie, 720p60 at decimation 2: K6 0 / 256 / 512 / 1023
  and Source = Osc at 512; the labelled K5 Colour sims at 512 = Moss there, not the hardware default Vega).
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic hexbloom`): router1, every config on seed 1.
  Post-route Fmax HD Analog 81.56 (worst HD), HD HDMI 88.37, HD Dual 95.20, SD Analog 82.88, SD HDMI 83.71,
  SD Dual 94.45 MHz; 6,914 / 7,680 LC (90.0 %, was 6,622), 10 / 32 EBR. vmprog:
  out/rev_b/ron/kinetic/hexbloom.vmprog (v0.1.1). Not flashed.

## Status

- v0.1.0 built 2026-11-14 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`),
  every config on seed 1. Not flashed.
- Post-route Fmax (builder's figure): HD Analog 101.71 MHz, HD HDMI 104.16, HD Dual 97.07, SD Analog 95.14,
  SD HDMI 101.27, SD Dual 101.13. HD needs 74.25, SD 27.
- 6,604 .. 6,622 / 7,680 LC (86 %), 10 / 32 EBR. Six bitstreams pass iceunpack. Standalone yosys: no
  `emulate_transparency`, no carry self-adds (dupin: none).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic hexbloom` (router2 was not tried).
- Model = RTL (GHDL, 720p60): pixel diff 0 of 918,720 on the default frame (black input, decimation 1) and
  0 of 918,720 on the Source = Osc frame (decimation 1; the per-cell luma hold is modelled exactly); 0 of
  228,960 on a decimation-2 render with Radial + Snap after 8 fields (it matches model step 8 only, so the
  phase advances as modelled). The selfie frame differs from the model in 14.5 % of pixels (0.9 code on
  average): the tester scales the photo its own way, so the held lumas differ by a few codes; every cell is
  rigid in the render. The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the
  left (the generated avid starts 4 clocks after the delayed source avid, as in riley).
- Other rasters rendered in GHDL (decimation 1, black input, defaults): NTSC 480i, both fields, pixel diff 0
  of 173,988 each; 1080i, both fields, pixel diff 0 of 1,034,640 each. 1080p was not rendered (same steps
  as 1080i with half the line step).
- Renders: `.kinetic_work/rb_hexbloom/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion).

## Cut / deviations

- **Shape order**: hexagram .. hexagon .. triangle with the hexagon at the knob's centre (the spec listed
  hexagon .. hexagram-like .. triangle), so min / default / max are three different pictures.
- **BLOOM 0 %** is two rings per cell (one hexagon inside a frame), not a uniform cell; the top is tied to
  the cell size (one ring per 12 virtual px) so no setting washes out to the mean. The "no ring thinner
  than 2 virtual px" rule holds by construction (thinnest band 6 px, thinnest line 3 px).
- **AA width** comes from the horizontal pixel step only (the gradient is constant per field); on
  interlaced sources the vertical edges are filtered for the frame line pitch, not the field's.
- **Grout Off** is not a true cell-to-cell anti-alias: the last half pixel at the cell edge blends half-way
  to the ground colour (a soft hairline seam).
- **Luma hold**: one line of the 32-px box per cell (no vertical filtering), sampled 3/4 of a row pitch
  above the cell centre, about 12 px left of it; cells whose centre column is off-screen keep whatever
  was last written (0 after power-up); line 0 of each field is drawn without the hold (its cells are
  being sampled on that line).
- **SD** shows the same picture (the centre 4:3: 6 cells across, the same 5 rows), but the grout is only
  2 .. 3 pixels wide there, so its vertical runs come out dimmer and less saturated than the slanted ones
  under the 9-tap chroma filter, and at BLOOM 100 % a ring is 6 pixels across but under 3 field lines
  tall, so the flat tops and bottoms of the finest nests soften toward the mean in 480i. 720p and 1080
  hold every setting.
- The float prototype was skipped for time: the design was iterated directly on the integer model.
- `field_n` is passed through one register instead of the 40-clock delay (the core ties it high).
