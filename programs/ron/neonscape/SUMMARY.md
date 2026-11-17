# Neonscape

Wireframe neon landscapes: the 1980s computer-graphics scene (grid floor, sun, mesh mountains, skyline, palms, a
car, clouds, birds, rain, stars), with gradients that roll in object by object. A proof of concept that grew out
of Outrun (Kinetic): more colour, crisp wires, many more objects, motion inside every object.

**v0.2.0 built 2026-11-17, all six configs, not flashed.** Flash `out/rev_b/ron/neonscape.vmprog`.
v0.2.0 (same day, Ron's notes on the v0.1.0 renders): the cars redrawn as wire structures and the side view
un-squashed, the rain falls instead of rising, K3 = Sheen (was Horizon).

## How it draws

Every object is drawn the same way.

- Each pixel gets a **region ID**: 4 bits of object (also its depth: a nearer object has a higher number and owns
  the pixel) and 4 bits of facet.
- A **wire** is drawn wherever the ID differs from the pixel 1 .. 3 px to the left or 1 .. 2 lines above (one line
  store of IDs). Its colour is the wire colour of the nearest object among those neighbours. Hidden lines come for
  free, and a new object only has to produce an ID.
- A region is **filled** from its object's colour ramp when that object's gradient is switched in (P12); otherwise
  it is black, so an unfilled object in front of a filled one is a black cut-out with a bright outline.
- Colour: one table read per pixel (8 palettes x 32 entries: 8 ramps of 3 keys, 16 wire colours), then
  `c0 + delta x t`, so ramps are smooth with no dither. A wire's `t` is the sheen (K3), which lifts
  the wire toward white.

All geometry is in virtual 1080-line coordinates, so SD, 720p and 1080 show the same picture (SD = the centre 4:3).

## Objects

Sky half (d = rows above the horizon):

| Object | How | What moves |
|---|---|---|
| Sky | vertical ramp over 508 rows | - |
| Stars | hashed 32-px cells, one in sixteen holds a 4-px star | twinkle |
| Rain | slanted 16-px columns, dashes of 96 rows in 256 | falls 24 rows a field (it rose in v0.1.0) |
| Sun | half-chord table, radius 256, centre 144 rows up | slits roll down the lower half |
| Moon | radius 128, off to the right; a second disc bites it into a crescent | - |
| Clouds, birds | span-coded sprites in lattice cells, one or two bands of clouds, one of gulls | drift; gulls flap |
| Far range | hashed polyline ridge, knots 64 px apart | contour lines flow downhill |
| City | hashed towers of 64 / 128 px; each has its own wire pattern (floors, bays, both, none); masts | lit windows climb |
| Near range | hashed polyline ridge, knots 128 px apart; a draped mesh: columns at the knots, rows at k / 8 of the local height (a 9-stage divide per pixel); lit and shaded slopes | mesh rows flow uphill |
| Palms | span-coded sprite, 256 x 384, three each side of the centre; ringed trunks | crowns sway |

Floor half:

| Object | How | What moves |
|---|---|---|
| Grid | one divide per line (du = G / d); lanes from a per-pixel accumulator, rows from z = 1024 du. Level of detail: toward the horizon the lanes and rows double their pitch at 128 / 96 / 64 / 48 / 32 / 24 rows, and stop 16 rows short of it | flight (Drive) or sideways slide (Pan) |
| Road | the four centre lanes, a dashed centre line (Drive); a band near the bottom of the screen (Pan) | dashes run |
| Car | span-coded sprite drawn as a wire structure: louvred rear glass, panel seams (thin dark slivers, so they show in wire and in fill), crease lines, light bar, plate, exhaust. Rear view 464 x 176 (Drive); side view 512 x 160 (Pan; it was 488 x 80 in v0.1.0) | bobs; lights blink |
| Lake (S10) | the sky half mirrored (every sky object is a function of d), dimmed, with ripple gaps that widen with depth and a per-line wobble | ripples drift |
| Tunnel (Land) | the grid on both halves, the sun at its end | flight |

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Land | Open / Hills (far range) / Ranges (far + near) / Metro (city) / Valley (city + near range) / Palms (near range, palms, road, car; default) / Strip (city, palms, road, car) / Tunnel (road, car) |
| K2 | Sky | Sun / Sun Clouds / Sun Birds / Dusk (sun, stars, clouds, birds; default) / Moon (+ stars) / Moon Clouds / Storm (moon, two cloud bands, rain) / Stars (+ birds) |
| K3 | Sheen | the light that runs along the wires (see "Sheen"). 0 % = none; 50 % = natural (default); 100 % = the single bright diagonal sweep of v0.1.0. Was Horizon until v0.2.0: the horizon height now comes with the Land (Open 56 %, Hills / Ranges 58 %, Metro 62 %, Valley 60 %, Palms 54 %, Strip 56 %, Tunnel 50 %) and switches with it |
| K4 | Speed | bipolar, centre = still (+-8 deadband). Drive: flight, up to 15 cells a second. Pan: sideways travel, palms fastest, far range slowest |
| K5 | Colour | Sunset / Miami / Laser / Arcade / Chrome / Toxic / Ice / Ember |
| K6 | Camera | camera (or Osc) strength, default 100 %. Three effects: it lights the sky ramp (the picture shows in the sky, even with the gradients off), bends the grid's rows (up to 1.8 cells), and lifts the ridges and roofs (up to 236 rows) |
| S7 | Gradient | On / Off (pure wireframe) |
| S8 | Shade | Smooth / Sharp (every ramp cut to 8 hard bands) |
| S9 | View | Drive (first person, flying forward) / Pan (travelling sideways, parallax between the layers, side-view car) |
| S10 | Ground | Grid / Lake |
| S11 | Source | Video / Osc (a drifting triangle lattice) |
| P12 | FILL | the rolling gradient scale. 0 % = wires only. Sliding up, each object's gradient wipes outward from the horizon (8 rows a count), one after another: sun, sky, far range, city, near range, floor, sprites. 100 % = everything filled |

Presets: Sunset Drive (defaults), Wire City, Storm Ranges, Mirror Lake, Tunnel Run, Beach Cruise, Chrome Poster.
Hand-written placeholders, to be replaced by captures from the hardware.

## Sheen (K3)

A wire's colour is lerped toward the palette's hot colour (white) by a pulse: a triangle 64 px long in every 256
of a phase.

- **Uniform** (the top of the knob, the v0.1.0 look): every wire reads the same phase, x + y + 4 t, so one set of
  "/" fronts sweeps the whole picture and every pulse shows.
- **Natural** (the middle): each kind of object reads its own phase and scale, and a table keyed by the pulse's
  number and the object lets about three pulses in five through, so they come and go.

| Object | Direction | Period |
|---|---|---|
| Sun, road | bands moving down, 2 rows a field | 256 rows |
| City, grid | bands moving down (on the grid: toward the viewer) | 512 rows |
| Near range | "/" fronts, 4 px a field | 512 px |
| Far range, clouds, birds | upright bands moving left, 4 px a field | 512 px |
| Palms, car | upright bands moving left | 256 px |

- The knob, glided: 0 .. 50 % raises the natural sheen from nothing to full strength in nine steps. 50 .. 100 %:
  the objects join the uniform sweep one by one, far to near (sun, clouds, far range, city, near range, palms,
  grid, road, car), until at 100 % everything is on it.

## Video

`program_type = "processing"`. The source is the camera's luma (>> 4, less 4) or the Osc lattice, smoothed along
the line by a one-pole filter (1/8) and scaled by K6 (4-bit gain). With a black input all three effects are zero
and the program is purely generative.

## Engine

- Per field (in the vblank zone): glides (sheen, camera, fill), flight and pan phases, layer scrolls.
- Per line (48 clocks of horizontal blanking, started when the line's last pixel has left stage 1): the row's
  distance from the horizon, one 22-step divide for the grid, ten table reads (sun and bite chords, a sprite row
  for each of the two sprite slots), the ramp positions, the fill flags, each layer's start x.
- Per pixel (34 stages): a world-x counter per layer, loaded at staggered clocks so every object's bits meet at
  stage 14; merge (15); wire detection against the ID line store (16, 17); colour table and lerp (18 .. 23); glow
  (24); output conditioning.
- Glow: a wire's chroma is held over the 4 black pixels after it and the luma is delayed 2 px into the middle of
  that, so the wide chroma filter returns a 3-px wire at about 87 % saturation (it would be 37 % without) and
  leaves a faint coloured halo on black.
- Output conditioning as in Outrun: chroma [1 2 2 2 2 2 2 2 1]/16 (its delay line in a block RAM, resynced every
  line), luma [1 2 1]/4, luma 13/16 toward black, blanking gate, generated avid.
- Sprites are span-coded: a row is six x boundaries and seven 2-bit zones (4 table words), so edges stay crisp at
  any scale; a spare bit in each row is a crease parity, which draws a wire across the body there (the cars'
  creases, the palms' trunk rings). Art is drawn by `neonscape.py` (pure Python) into `neonscape_pkg.vhd`; `python3 neonscape.py
  --preview DIR` writes the sprite sheet.
- Block RAM (21 of 32): line tables 8, ID line store 8, colour table 3, chroma delay 1, luma delay 1.

## Resolution differences

- SD shows the centre 4:3 of the scene: the outer palms and the edges of the ranges are cut.
- Wires are 3 x 2 px at 1080p, 2 x 2 at 720p, 3 x 1 at 1080i, 1 x 1 at SD. On 480i a field line is 4.4 virtual
  rows, so mesh rows, slits and the gulls (32 rows tall) are coarse.
- Stars are 4 virtual px: 2 px wide on SD and present on about half the field lines they should be.

## Known limits (proof of concept)

- Wires are not anti-aliased: shallow slopes show stair-steps (softened along the line by the luma filter only).
- Fill gradients run vertically only. A fill wipe ends in a hard horizontal cut.
- The camera's lift acts per pixel with no vertical smoothing, so a busy picture makes ragged ridges.
- The city's lit windows show only when the city's gradient is in.
- In Drive the far layers do not move (they are at the horizon); only the floor flies. Pan moves everything.
- There is no horizon knob any more (see K3). Sun, moon and car are fixed in x. The moon has no slits; the tunnel has no sky objects but the sun.
- K1 / K2 / K5 are label knobs: a pot resting at 1 .. 7 counts at the very bottom of its travel reads as that label
  (the load-time index fix).
- Just under the horizon (16 .. 60 rows) the thinned rows are still about 6 rows apart, which reads as a bright
  striped band, busier at 720p and SD than at 1080p. If it is too heavy on hardware, move the level-of-detail
  thresholds up one step (state 32 of the line engine).

## Status

- v0.2.0 built 2026-11-17 with router2 (the builder default), `./build_programs.sh ron neonscape`.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 78.81 MHz | 7129 |
| SD Analog | 1 | 85.83 MHz | 7119 |
| HD HDMI | 2 (seed 1 stalled the router) | 76.18 MHz | 7111 |
| SD HDMI | 1 | 65.66 MHz | 7124 |
| HD Dual | 1 | 76.62 MHz | 7128 |
| SD Dual | 1 | 77.51 MHz | 7131 |

- 7,131 / 7,680 LC (92.9 %), 21 / 32 block RAMs. HD needs 74.25 MHz, SD 27. The program alone is about 4,190 LUT
  and 2,600 FF (standalone yosys); no block RAM carries read-during-write emulation. HD HDMI's margin is thin
  (76.18 on its one routed seed).
- v0.1.0 (the first cut, commit f0cdb09) closed at 7,094 LC with HD Analog on seed 2.
- Rendered in GHDL at 720p60 (decimation 1), hardware-hue previews in `.neonscape_work/prev/`. v0.2.0 (`g_*`):
  the default scene wire-only at Sheen 50 % and 78 %, Valley + Moon Clouds fully filled at Sheen 100 %, Pan +
  Strip + Storm fully filled. v0.1.0 (`b_*`, `c_*`, `d_*`): the camera at 40 % and 100 %, the lake, the tunnel,
  Sharp, one NTSC frame (both fields; same picture, centre 4:3). 1080p and 1080i were not rendered. The renders
  use a copy with the glides snapped (`.neonscape_work/simsnap`), since a glide covers only a third of its
  travel in the three fields a render runs.
- Not verified at all: motion (every render is one still: the rain's fall, the sheen's travel and the way the
  objects join the sweep are reasoned, not seen), hue on hardware, the HD HDMI output with these crisp moving
  wires (the output conditioning is Outrun's, which is also unflashed).
- v0.2.0 logic budget: the first sheen (three directions with their own speeds, two crossfaded pulses) took the
  design to 7,496 LC (97.6 %). It was cut back to one pulse path and the horizon's glide, the label knobs'
  subtract-and-compare hysteresis and a third of the star hash were traded for it.
- Tools: `.neonscape_work/sim.sh <tag> [--video IMG] [tester options]` (one render), `stat2.sh <dir> <tag>`
  (standalone size).
