# Neon Beach

Pixel Beach's side-on shorebreak drawn by Neonscape's wire engine: a cross-section of a beach in neon wires.
Sky with a sun or moon, clouds, gulls and stars; under the horizon a strip of distant sea that mirrors them; in
front of it the water cut through like a glass tank, its layers (each its own blue / green, Pixel Beach's bands)
following a moving wave surface that steepens over the rising bed and breaks on the beach, foam as squares that
turn white; under everything the sand, a mesh of quads. When a wave lands, a wash runs over the sand: the cells
it covers turn into triangles in the water's colour, and as it drains they revert a triangle at a time, leaving
the sand dark and wet. A small faceted crab walks the sand and runs from the wash.

**Status: v0.5.1 (2026-11-22): the wash wets only the sand its water sheet touches (the top row of cells);
v0.5.0 made K6 Speed (no input video: a synthesis program now), the Crab switch Off / On, and drying sand fade
back smoothly; see "v0.5.0" and "v0.5.1" below.** Flash
`out/rev_b/ron/neonbeach.vmprog`. Build plan and design history: `PLAN.md` (side-on version). v0.1.0 (ca04ddf)
was the still scene, v0.3.0 (e419f45) the first with waves, wash and crab; v0.3.1 tapers the wash toward the
shoreline and fixes the engine's timing; v0.3.2 (HW-captured defaults) was the first one flashed.

## How it draws

- Neonscape's engine: every pixel gets a region ID (object + facet bits); a wire is drawn where the ID differs
  from the pixel 1 .. 3 px left or 1 .. 2 lines up (one line store); a wire takes the colour of the nearest
  object among the neighbours; with Fill on, regions fill from their object's colour ramp. Glow and the output
  conditioning (chroma 9-tap, luma [1 2 1], luma 13/16, blanking gate, generated avid) are Neonscape's.
- Virtual 1080-line coordinates throughout, so SD, 720p and 1080 show the same picture (SD = the centre 4:3).
- **The water and sand** are read per pixel from a knot table (block RAM, 128 knots 16 virtual px apart,
  double-buffered by field): each knot holds the surface row, its step to the next knot, the bed row, its step,
  the foam and a water flag. Surface and bed are lerped between knots. Water = surface .. bed; its eight layers
  are at k / 8 of the local depth (a three-stage divide), each layer one flat colour (t = 127 - 16 k on a
  four-segment ramp: neon top, surf, aqua, mid, deep). Its columns are every 64 px of x + (sea level - surface) / 2,
  so they sway with each passing wave. Foam: 16 px squares (one knot wide) in the top four 16-row bands under the
  surface; a square is foam when the knot's foam / 4 passes its row plus its own jitter (reshuffled 4 times a
  second), and turns white over the next 8 rows of foam. Sand = under the bed line, cells 32 px wide and 32 rows
  deep with the rows parallel to the sand's surface.
- **The waves** come from a small microcoded engine (`nbengine.py`: assembler, emulator, program) that fills the
  knot table bank that is not on screen, once a field, starting at the field start and running on through the
  field (about 195k clocks worst case; an NTSC field is 225k). Per knot from the right edge: the wave phase
  (Rhythm) less a per-knot offset that shortens the waves over the rising bed, scaled by Speed (x m / 128,
  m = 128 / (1 + 2 K6) by a quadratic fit: longer waves at the same rhythm run in up to 3 times as fast), the
  wave's size by its number
  (waves come in sets), a shoaling gain, the depth cap (a wave taller than 3/8 of the depth breaks: the excess
  becomes foam and the profile turns into a breaker), the profile from a shape table (4 classes x 128 phases),
  two short chop waves (Chop), the surge (slow, from Swell), and the foam (Foam): breaking, whitecaps on steep
  fronts, the shoreline, 5 a field of decay. On the beach the surface is a thin swash sheet tapering to the
  run-up's front; a crest reaching the shoreline starts a run-up (fast up, slow drain).
- The engine: a 16-bit ALU (one adder for add / sub / abs / min / max), sixteen registers kept in the top of its
  scratch block RAM (no register-file multiplexers), a serial 16 x 8 multiplier (10 x 8 until v0.5.0, while
  the emulator modelled 16 x 8: no product reached 2^10 until Speed), the program in block RAM
  (512 x 24). Ten clocks an instruction; every enable and select is decoded into a register two or more clocks
  before it is used (v0.3.0's decode-to-result paths missed HD HDMI on three router2 seeds). The scratch RAM's
  initial contents hold the per-knot phase offsets and the bed. Verified (v0.5.0): the emulator matches a
  plain-Python statement of the maths over 750 fields at five knob settings including Speed 0 / 30 / 50 / 100 %
  (0 mismatches, `proto/spec.py`), and the RTL matches the emulator word for word on every knot write at Speed
  50 % (GHDL testbench `.neonbeach_work/tb5`, `cmp.py`: 0 mismatches over 3 fields; the same compare against
  Speed 0 gives 204, so it discriminates).
- **Wet sand**: a 4-bit counter per sand cell (2048 cells, block RAM). A sweep in vblank works out each column's
  reach from the engine's swash envelope and shoreline and rewrites every cell. Since v0.5.1 only the top row of
  cells takes the wash: whole under the sheet, the upper triangle at its front column (v0.5.0 and earlier soaked
  e rows deep with drifting lobes). 15 = under the wash, 14 = only its upper triangle, then 12 .. 1 drying one step every 12 fields
  (13 = damp and held: the sea bed's top row, the crab's hole). The
  reach thins over the last columns before the shoreline (v0.3.1; it ended in a vertical cut). The
  pixel path draws 15 and 14 as wash triangles; drying cells keep their diagonals (the upper triangle's longer)
  and lose them by a per-cell hash, so the triangles disappear piecemeal. Wet sand fills darker: 8 ramp steps a
  wetness step less the fields since the last step (round(8 x div / 12)), so fresh wet sand is dark brown and
  fades back smoothly to the (light) dry sand over about 2.4 s. (v0.4 and earlier stepped every drying cell at
  once, 5 times a second: Ron's "constant subtle pulsing" along the top of the sand.)
- **Per-field colours.** The colour table is a block RAM (seeded from `C_NB_PAL`). In vblank the colour path is
  run on nine made-up pixels from the keyframe slot, and the results are written into the live palette as ramp
  segments: the water's five keys (keyframes night / dusk / golden / day over the slider's lower three quarters,
  day above; night mode = the night keyframe), its wire and the wash's wire (top key -> white), and the sky
  (zenith and horizon keyframes lerped by the elevation). So the water changes colour gradually as the sun sets
  (v0.3.2 jumped from blue to magenta at the low-sun threshold); the other objects still switch palettes at the
  day / low sun / night thresholds. The wash takes the water's top colour.
- **The sea strip**: every line under the horizon draws the sky objects at the same distance from the horizon
  (a mirror), dimmed, with drifting ripple gaps.

## Objects

| ID | Object | Facets | What moves |
|---|---|---|---|
| 0 | sky (and the sea strip) | none | stars twinkle; ripple gaps drift |
| 1 | sun / moon and its mirror | none | the sun's slits roll; the moon is a crescent |
| 2 | clouds, gulls and their mirrors | sprite zone (gulls: left / right wing plank) | drift; gulls flap |
| 5 | water | layer parity, column parity | everything rides the wave surface; the columns sway |
| 6 | foam squares | knot parity, square-row parity | squares turn white and fade, reshuffled 4 times a second |
| 7 | wash (covered sand triangles) | column, row, upper triangle | arrives and drains a triangle at a time |
| 8 | sand | column, row, the diagonal while damp | diagonals vanish as it dries |
| 9 | crab | sprite zone, creases (facet bands) | walks, legs alternate |

Spray and stars are overlays.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Rhythm | how often the waves come; also the chop's and the surge's tempo (and so the wash) |
| K2 | Tide | the water level: moves the shoreline along the beach |
| K3 | Foam | breaking foam, whitecaps on steep fronts, the shoreline foam, the spray |
| K4 | Chop | two short waves riding on the swell |
| K5 | Swell | wave height, the surge's depth, how far the wash runs |
| K6 | Speed | how fast the waves run in to the shore, x1 .. x3 at the same rhythm (they get that much longer); default 50 % = x2 |
| S7 | Fill | Wires / Fill (the gradients roll in object by object) |
| S8 | Colour | Neon / Ice (each with day, low-sun and night palettes) |
| S9 | Crab | Off / On (1 = On since v0.5.0; it burrows; on again it comes up somewhere new; its hole is a damp cell) |
| S10 | Sky | Day / Night |
| P12 | Sun | elevation of the sun or moon; the sky, the palette and the stars follow it; gulls roost when it is down |

## Resolution differences

- SD shows the centre 4:3 (x 240 .. 1680): the back of the beach and the deepest water are cut.
- Wires are 3 x 2 px at 1080p, 2 x 2 at 720p, 3 x 1 at 1080i, 1 x 1 at SD. On 480i a field line is 4.4 virtual
  rows, so the water's layers near the shore, thin foam and the gulls are coarse.

## Status

**v0.3.1** built 2026-11-18 with **router1**: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron neonbeach`
(router2 gave 69 .. 76 MHz on HD HDMI where router1 gave 76 .. 83, as CLAUDE.md predicts for engine designs):

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | **2** (seed 1 missed) | 83.17 MHz | 7265 |
| SD Analog | 1 | 90.22 MHz | 7278 |
| HD HDMI | 1 | 82.84 MHz | 7263 |
| SD HDMI | 1 | 83.17 MHz | 7280 |
| HD Dual | 1 | 77.98 MHz | 7269 |
| SD Dual | 1 | 86.26 MHz | 7254 |

7,280 / 7,680 LC (94.8 %), 28 / 32 block RAMs. Anything added needs a matching removal; measured candidates:
the per-object sheen variety (~87 LUT + 26 FF), the stars (~82 LUT + 44 FF).

v0.3.0 (router2, the builder default):

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 79.18 MHz | 7171 |
| SD Analog | 1 | 83.76 MHz | 7156 |
| HD HDMI | **4** (seeds 1 .. 3 missed) | 82.67 MHz | 7166 |
| SD HDMI | 1 | 70.09 MHz | 7162 |
| HD Dual | 1 | 75.77 MHz | 7176 |
| SD Dual | 1 | 59.91 MHz | 7166 |

7,176 / 7,680 LC (93.4 %), 28 / 32 block RAMs. HD needs 74.25 MHz, SD 27. v0.1.0 (ca04ddf): 5,636 LC, 21 block
RAMs, all six on seed 1.

Rendered in GHDL at 720p60 (decimation 1), hardware-hue previews in `.neonbeach_work/prev/`: `c_*` = v0.3.0
(wires, fill, storm = Swell / Foam / Chop at 100 %, night + Ice + fill), with the swash envelope preset so the
wash shows in a three-frame render; `a_*` = v0.1.0. Motion was not rendered (the look model's GIFs in
`.neonbeach_work/proto/out/` show it, from the pre-hardware formulation).

## Build notes

- **yosys abc9 box-order assert.** With ABC's `&mfs`, abc9 asserts (`Boxes are not in a topological order`, then
  `count_id(cell->name) == 0`) depending only on the order cells reach ABC: the same logic passed or failed by
  the order of the processes in the file (6 of 11 shuffles passed all six configs). The order in
  `neonbeach.vhd` is deliberate. After any edit run `.neonbeach_work/check6.sh` (full hardware_top synthesis of
  the six configs, about 20 s); if a config fails, try orders with `.neonbeach_work/abc9_order.py`.
- The engine's execute step was the critical path in v0.2.0 (instruction decode -> 17-bit adder -> result mux,
  14 ns): its adder controls are decoded two phases early and its sum is registered.
- v0.2.0 (waves, foam, spray) was not packed: superseded by v0.3.0 while its HD HDMI seeds were running.

## Known limits

- Wires are not anti-aliased.
- Where the water gets shallow its eight layers crowd into a solid bright edge (no level of detail: switching
  the layer count drew stair-step verticals).
- Presets are hand-written placeholders: capture real ones with `./capture_preset.sh`.
- Only the water, its wire and the wash (and the sky, as before) change colour gradually with the sun; the sun,
  sand, clouds, gulls, foam and crab still switch palettes at the day / low sun / night thresholds.

## Tools

`.neonbeach_work/`: `sim.sh` (one GHDL render, hardware hues), `simsnap/` (glides snapped, the swash preset),
`check6.sh`, `stat.sh`, `proto/model.py` (the look model), `proto/spec.py` (spec vs emulator), `tb/` (RTL vs
emulator testbench).

## v0.3.2: HW-captured defaults (2026-11-22)

Ron dialled the look in on hardware (v0.3.1), captured with `./capture_preset.sh --defaults`:
K1 Rhythm 580, K2 Tide 984, K3 Foam 449, K4 Chop 84, K5 Swell 152, K6 Camera 0, S7 Fill = Fill, P12 Sun 924
(S8 Colour, S9 Crab, S10 Sky, S11 Source unchanged). The vmprog was repacked from the v0.3.1 bitstreams (only the
settings block differs, 196 bytes), so timing, seeds and LC are as in v0.3.1. Presets are unchanged. The v0.3.1
vmprog and TOML are in `.heirloom_work/rb_neonbeach/`.

## v0.4.0: Ron's hardware feedback on v0.3.2 (2026-11-22)

Ron's notes after flashing v0.3.2, and what changed:

| Note | Change |
|---|---|
| Crab 50 % smaller, more wireframe, less rounded | Redrawn in straight lines: hexagonal shell cut into facet bands (alternate bands fill darker) and a dark shoulder facet, legs as square-capped bars (thigh out, shin straight down), angular pincers, diamond eyes on stalks. Half size (about 122 x 48 virtual px): the pixel path doubles x into the ROM and reads one ROM row a line; its feet stay where v0.3's were. |
| Gulls: no circle in the middle, two long rectangular planes | Each gull is two flat planks (34 units, 68 virtual px each) meeting at the centre, left and right wing in two shades; V up / droop down for the flap. |
| A constant subtle pulsing down the sand | Taken to be the sheen on the sand's wires (bands moving down); the sand no longer takes it. Wrong guess: see v0.5.0. |
| Wash should discolour the sand more; lighten the sand | Dry sand ramps are lighter; wet sand darkens 8 ramp steps a wetness step (was 4). |
| More dynamic water; crest / foam as full squares turning white; more blue / green colours fading down the grid (Pixel Beach's layers) | Each water layer its own flat colour on a five-key ramp (Pixel Beach's DEEP, MID, AQUA, SURF and a neon top); foam = 16 px squares turning white one by one; the water's columns sway with each passing wave. |
| Waves should change colour gradually at sunset, not jump blue -> magenta | The water's colours (and its wire, and the wash) are written into the palette every field from four keyframes lerped by the sun's elevation. |

Built 2026-11-22 with **router1** (`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron neonbeach`), all six on
seed 1, 28 / 32 block RAMs:

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 83.94 MHz | 7349 |
| SD Analog | 1 | 76.98 MHz | 7346 |
| HD HDMI | 1 | 78.63 MHz | 7332 |
| SD HDMI | 1 | 81.65 MHz | 7335 |
| HD Dual | 1 | **74.40 MHz** (0.15 over Fmin: the thinnest margin; try another seed first if it ever misbehaves) | 7330 |
| SD Dual | 1 | 80.08 MHz | 7350 |

7,350 / 7,680 LC (95.7 %, up from 94.8 %): past the ~94 % comfort line; anything added needs a removal first.

The wave engine (microcode, `nbengine.py`) is unchanged, so its spec / emulator / RTL checks still stand. Resource
cost against v0.3.1 (standalone yosys): +56 LUT, -3 FF, block RAMs unchanged (the colour table became a RAM, same 3
EBRs, no transparency logic). GHDL renders (720p, hardware hues) in `.neonbeach_work/prev/`: `e_day` (Ron's
defaults), `e_sunset` (Sun 41 %, before the sunset keyframes were retuned to avoid a grey midpoint), `e_sunset2` (after;
the wash was then moved from halfway to 3/4 of the way to the top key, also grey at sunset), `e_storm`.

## v0.5.0: Ron's notes on v0.4.0 (2026-11-22)

| Note | Change |
|---|---|
| The pulsing is on the sand itself, across the whole top edge, maybe the wash's discolouring | It was: every drying cell stepped one wetness step (8 ramp steps) in the same field, every 12 fields, so the whole wet strip brightened in a jump 5 times a second. The pixel path now subtracts the fields since the last step as a fraction (round(8 x div / 12), 0 .. 7), so the colour fades back smoothly. A retreating wash leaves 12 (drying at once); 13 is kept as the held damp value (the sea bed's top row, the crab's hole), which takes no fraction (it would saw-tooth). |
| The crab switch is reversed | S9 labels are now Off / On (1 = On), as in every other program and Pixel Beach; the VHDL inverts the bit. |
| Replace K6 with an option to make the water roll in faster; no incoming video | K6 Speed: the per-knot phase offsets are scaled by m / 128 (m 128 .. 42), so at the same Rhythm the waves are up to 3 times as long and run in up to 3 times as fast. Glided (the old camera glide slot). The camera, its sky lift, the Osc lattice and S11 Source are gone; `program_type = "synthesis"` (Pixel Beach and Damier run that way on hardware; the program already makes its own avid). S11 is free. |

The engine's multiplier was widened from 10 x 8 to the 16 x 8 the emulator and the docstring always described
(the phase offsets reach 52,788). Engine worst case 195k clocks a field (v0.4.0: 188k; +4 instructions and a
multiply a knot, plus the per-field Speed fit); an NTSC field has 225k.

Built 2026-11-22 with **router1**, 28 / 32 block RAMs:

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 87.83 MHz | 7214 |
| SD Analog | 1 | 80.32 MHz | 7225 |
| HD HDMI | **2** (seed 1: 73.66, a routing-bound path through the wave / wash logic, 10.3 ns routing to 3.3 logic) | 83.41 MHz | 7230 |
| SD HDMI | 1 | 79.25 MHz | 7219 |
| HD Dual | 1 | 79.25 MHz (v0.4.0: 74.40) | 7223 |
| SD Dual | 1 | 75.75 MHz | 7237 |

7,237 / 7,680 LC (94.2 %, down from 95.7 %): the camera path cost more than Speed, the wider multiplier and the
drying fraction together.

## v0.5.1: the wash wets only what it touches (2026-11-22)

Ron on v0.5.0: "still seeing the constant washing over of the entire top of the beach sand section, 2 or 3 times
a second. it is darkening where water is not touching it over and over." A model of the sweep driven by the engine
emulator (Ron's defaults) showed why: the visible swash sheet is at most 14 rows thick, but each run-up soaked the
sand to e rows (about 112 at the defaults: four rows of cells), and drifting lobes pushed the reach past the
sheet's front. As the swash drained (e falls 2 a field), the cell rows flipped from wash colour to dark one after
another every 8 .. 16 fields, all along the run-up. Now the sweep wets only the top row of cells: whole under the
sheet, the upper triangle at its front column, nothing past the front or below. The lobes, the shoreline cap and
their logic are gone (-110 LUT standalone).

Also in v0.5.1: router1 stalled on seed 1 of both HDMI configs. dupin (`~/czartroniks_port/tools/dupin.py`) found
one LUT with a duplicated input: the Tide sea level, `626 - (v_s >> 3 + v_s >> 4)` on a signed `v_s`, adds two
sign-extended shifts of one value (CLAUDE.md's kelp shape). It is now `722 - (K2 >> 3) - (K2 >> 4)` on the
unsigned knob, the same value for all 1024 settings; dupin reports 0 on all six configs.

Built 2026-11-22 with **router1**, all six on seed 1 (no stalls), 28 / 32 block RAMs:

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 85.66 MHz | 7088 |
| SD Analog | 1 | 69.83 MHz | 7075 |
| HD HDMI | 1 | 83.50 MHz | 7063 |
| SD HDMI | 1 | 78.00 MHz | 7071 |
| HD Dual | 1 | 80.26 MHz | 7070 |
| SD Dual | 1 | 83.65 MHz | 7088 |

7,088 / 7,680 LC (92.3 %).
