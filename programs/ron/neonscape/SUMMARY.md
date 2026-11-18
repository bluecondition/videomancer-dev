# Neonscape

Wireframe neon landscapes: the 1980s computer-graphics scene (grid floor, sun, mesh mountains, skyline, palms, a
car, clouds, birds, rain, stars), with gradients that roll in object by object. A proof of concept that grew out
of Outrun (Kinetic): more colour, crisp wires, many more objects, motion inside every object.

**v0.3.1 built 2026-11-18, all six configs.** Flash `out/rev_b/ron/neonscape.vmprog`. Ron flashed v0.2.0.
v0.3.0 (Ron's notes from the hardware): controls moved (K3 = Fill, S7 = Sheen on / off, P12 = Horizon), sheen
fixed at the old knob's 75 %, the road no longer drifts in Drive, the grid is uniform to the horizon.
v0.3.1: the sheen's diagonal sweep restored (see "Status"), a spoiler on the car in both views.
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
| Grid | a clean plane to the horizon: uniform cells, no level of detail (v0.3.0). Two divides per line (du = G / d, and du / d = how fast a line moves from one row to the next); lanes and rows are parity bits in the region ID. Where the lines get 4 lines apart their colour starts to run into the floor's glow, and at 2 lines apart they are replaced by the glow, so the grid converges into a band of light under the horizon (about 75 rows at 1080p) instead of aliasing | flight (Drive) or sideways slide (Pan) |
| Road | the four centre lanes, a dashed centre line (Drive); a band near the bottom of the screen (Pan) | dashes run |
| Car | span-coded sprite drawn as a wire structure: louvred rear glass, panel seams (thin dark slivers, so they show in wire and in fill), crease lines, light bar, plate, exhaust, and a spoiler (v0.3.1): in the rear view a full-width blade above the roof with end plates, on struts down to the deck; in the side view a blade and end plate over the tail on a strut. Rear view 464 x 192 (Drive); side view 512 x 160 (Pan; it was 488 x 80 in v0.1.0). The side view's glass now fills the cabin under the roof (no frame): that freed the two spans a row needs for the strut | bobs; lights blink |
| Lake (S10) | the sky half mirrored (every sky object is a function of d), dimmed, with ripple gaps that widen with depth and a per-line wobble | ripples drift |
| Tunnel (Land) | the grid on both halves, the sun at its end | flight |

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Land | Open / Hills (far range) / Ranges (far + near) / Metro (city) / Valley (city + near range) / Palms (near range, palms, road, car; default) / Strip (city, palms, road, car) / Tunnel (road, car) |
| K2 | Sky | Sun / Sun Clouds / Sun Birds / Dusk (sun, stars, clouds, birds; default) / Moon (+ stars) / Moon Clouds / Storm (moon, two cloud bands, rain) / Stars (+ birds) |
| K3 | Fill | the rolling gradient scale (on P12 until v0.3.0). 0 % = wires only. Turning up, each object's gradient wipes outward from the horizon (8 rows a count), one after another: sun, sky, far range, city, near range, floor, sprites. 100 % = everything filled. Default 50 % |
| K4 | Speed | bipolar, centre = still (+-8 deadband). Drive: flight, up to 15 cells a second. Pan: sideways travel, palms fastest, far range slowest |
| K5 | Colour | Sunset / Miami / Laser / Arcade / Chrome / Toxic / Ice / Ember |
| K6 | Camera | camera (or Osc) strength, default 100 %. Three effects: it lights the sky ramp (the picture shows in the sky, even with the gradients off), bends the grid's rows (up to 1.8 cells), and lifts the ridges and roofs (up to 236 rows) |
| S7 | Sheen | On / Off: the light that runs along the wires (see "Sheen"). Was the gradient kill switch until v0.3.0; K3 at 0 % does that now |
| S8 | Shade | Smooth / Sharp (every ramp cut to 8 hard bands) |
| S9 | View | Drive (first person, flying forward) / Pan (travelling sideways, parallax between the layers, side-view car) |
| S10 | Ground | Grid / Lake |
| S11 | Source | Video / Osc (a drifting triangle lattice) |
| P12 | HORIZON | 25 % .. 75 % of the screen, glided (default 56 %) |

Presets: Sunset Drive (defaults), Wire City, Storm Ranges, Mirror Lake, Tunnel Run, Beach Cruise, Chrome Poster.
Hand-written placeholders, to be replaced by captures from the hardware.

## Sheen (S7)

A wire's colour is lerped toward the palette's hot colour (white) by a pulse: a triangle 64 px long in every 256
of a phase. Fixed at what v0.2.0's Sheen knob gave at 75 % (Ron's pick); S7 switches it off.

- Sun, clouds, birds, far range, city and near range share one phase, x + y + 4 t: "/" fronts sweep across them
  together, every pulse shown.
- Palms, grid, road and car each read their own phase, and a table keyed by the pulse's number and the object
  lets about three pulses in five through, so they come and go.

| Object | Direction | Period |
|---|---|---|
| Grid | bands moving down, toward the viewer, 2 rows a field | 512 rows |
| Road | bands moving down | 256 rows |
| Palms, car | upright bands moving left, 4 px a field | 256 px |

- A grid wire only takes the sheen where the lines are well apart; further out its colour runs into the glow.

## Video

`program_type = "processing"`. The source is the camera's luma (>> 4, less 4) or the Osc lattice, smoothed along
the line by a one-pole filter (1/8) and scaled by K6 (4-bit gain). With a black input all three effects are zero
and the program is purely generative.

## Engine

- Per field (in the vblank zone): glides (fill, camera, horizon), flight and pan phases, layer scrolls. Drive
  holds the pan position and the floor's sideways phase at zero: until v0.3.0 whatever Pan had built up stayed
  applied in Drive, which pushed the road (and the palm gap and the range's valley) off the centre line or off
  the screen.
- Per line (52 clocks of horizontal blanking, started when the line's last pixel has left stage 1): the row's
  distance from the horizon, two 22-step divides for the grid, ten table reads (sun and bite chords, a sprite row
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
- Sun, moon and car are fixed in x. The moon has no slits; the tunnel has no sky objects but the sun.
- K1 / K2 / K5 are label knobs: a pot resting at 1 .. 7 counts at the very bottom of its travel reads as that label
  (the load-time index fix).
- The glow band under the horizon is where the grid's lines would be closer than 2 lines apart. Its depth and
  the fade into it are set by two bit tests each (the rows in state 51 of the line engine, the lanes at stage 14);
  one bit lower doubles the band, one higher lets the lines run on until they alias.

## Status

- v0.3.1 built 2026-11-18 with router2 (the builder default), `./build_programs.sh ron neonscape`.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 3 (seed 1 stalled the router, seed 2 missed) | 81.37 MHz | 7279 |
| SD Analog | 1 | 79.67 MHz | 7257 |
| HD HDMI | 1 | 75.17 MHz | 7258 |
| SD HDMI | 2 (seed 1 stalled the router) | 82.33 MHz | 7272 |
| HD Dual | 1 | 78.34 MHz | 7285 |
| SD Dual | 1 | 79.74 MHz | 7268 |

- 7,285 / 7,680 LC (94.9 %), 21 / 32 block RAMs. HD needs 74.25 MHz, SD 27. HD HDMI has under 1 MHz of margin
  on its one routed seed, and two configs needed a second or third seed: the design is at the limit. Anything
  added needs a matching removal, and a trim of 100 to 150 LC would make the build dependable.
- **v0.3.0's sheen bug** (0f5a0df, flashed by Ron): a scripted edit deleted the line that runs the sheen's
  diagonal phase counter, so the counter sat at zero. Everything on the shared sweep (sun, clouds, far range,
  city, near range) had no sheen, and the palms' and car's bands did not travel. The tools said nothing and the
  logic count fell (7,219 LC), which is why v0.3.0 looked smaller than it is. Found from Ron's report; a scan
  for never-assigned signals (`.neonscape_work/undriven.py`) now runs after every scripted edit.
- Earlier builds: v0.1.0 (f0cdb09) 7,094 LC; v0.2.0 (3c51646) 7,131 LC.
- Rendered in GHDL at 720p60 (decimation 1), hardware-hue previews in `.neonscape_work/prev/`. v0.3.1 (`i_*`):
  Palms and Valley wire-only, rendered with the sheen fix and before the spoiler. The spoiler was checked in the
  sprite sheet only (`.neonscape_work/art/prev/sprites.png`, decoded from the same table words the engine
  reads), not in an engine render. v0.3.0 (`h_*`): the clean grid, the tunnel, the horizon slider. Older
  (`b_*` .. `g_*`): the camera at 40 % and 100 %, the lake, Sharp, Pan, one NTSC frame. 1080p and 1080i were not
  rendered. The renders use a copy with the glides snapped (`.neonscape_work/simsnap`).
- Not verified: motion (every render is one still), the grid's glow band in motion and at 1080p, the road fix on
  hardware, hue on hardware.
- The glow band is heaviest in the tunnel, where the floor's and the ceiling's meet (about a fifth of the
  picture at 720p).
- A first version of the clean grid drew lanes and rows directly (a per-pixel width accumulator and two 20-bit
  compares): 4,559 LUT standalone, about 99 % full. The shipped one keeps the wire detector and only adds the
  second divide, a 24-bit crowding accumulator and the glow entry in the colour table.
- Tools: `.neonscape_work/sim.sh <tag> [--video IMG] [tester options]` (one render), `stat2.sh <dir> <tag>`
  (standalone size), `undriven.py <file.vhd>` (signals that lost their driver).
