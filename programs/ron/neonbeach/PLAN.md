# Neon Beach: build plan (side-on)

A handoff for the model that builds this program. Written 2026-11-18 from a read of neonscape v0.3.1
(commit d712b01) and pixelbeach v3.24.2. The repo's `CLAUDE.md` and `TECHNIQUES.md` apply in full and are
not repeated here; this file holds only what is specific to this program.

An earlier draft of this plan used a first-person view. Ron chose the side-on view instead. The old draft
is at `.neonbeach_work/PLAN_first_person.md`; do not build from it.

## 1. The request

Ron's words:

> take the functionality and concept of Pixelbeach and create a new program using the look of neonscape, so a
> stylized wireframe beach scene with rolling and modulated waves, crests, wash on the sand that fades, maybe
> the wash itself is not color based but wireframe based with triangles appearing and receding/disappearing
> from the wireframe of the sand, a rising sun and moon and night/day modes, reflections, seagulls, ocean
> spray, and a crab.

And, on the viewpoint: "the same style as pixelbeach", not first person.

## 2. The program in one paragraph

Neon Beach is Pixel Beach's picture redrawn by neonscape's wire engine. It is a side-on cross-section: sky
with a sun or moon, gulls and stars at the top; a strip of distant sea under the horizon that mirrors them;
in front of that, the water cut through like a glass tank, its surface a moving wave profile that travels
from the right, steepens over the rising sea bed and breaks on a beach at the left; under everything, the
sand. The water is a wire mesh whose layers follow the surface and squeeze together in the shallows. The
sand is a mesh of quads. When a wave lands, a wash sheet runs over the sand: the quads it covers split into
triangles in the water's colour, and as it drains they revert one triangle at a time. A crab walks the sand
and runs from the wash.

Folder `programs/ron/neonbeach/`, program name "Neon Beach", `program_id = "com.czartroniks.neonbeach"`,
`program_type = "processing"`.

## 3. Decisions made in this plan

The viewpoint is Ron's. The rest are mine; say the first three back to him at the start and carry on unless
he objects.

1. **Waves are kinematic, behind an interface that a solver could fill later.** Pixel Beach's waves are a
   shallow-water solver: a large state machine on a half-rate clock enable, which never meets full-rate
   timing and needed hand-picked seeds on every HD build. That does not go into a new program on day one.
   Instead a small per-field sequencer (the "wave filler", section 7.2) writes the surface profile into a
   block RAM of knots, and the renderer only ever reads that RAM. If Ron later wants the real solver, it
   replaces the filler and nothing else changes. `programs/ron/pixelbeach/beach_proto.py` is the integer
   reference for it.
2. **Controls.** Ron wants the waves' parameters on controls even though the waves are animated. So K1 to
   K5 are Pixel Beach's five wave knobs in Pixel Beach's positions (Rhythm, Tide, Foam, Chop, Swell), K6 is
   Camera (the standing rule; Pixel Beach's Time knob set the solver's speed and has no meaning here), and
   the slider is the sun. That leaves no knob for neonscape's Fill and Colour, so both become switches:
   Fill is Wires / Fill and rolls the gradients in when flipped; Colour picks one of two themes. To make
   room, the Storm and Gulls switches are gone: Swell and Foam at the top are the storm, and gulls are
   always out by day. Rain and the second cloud band go with Storm. The sheen is always on. Shade, Pan
   view, the Horizon slider and the Land / Sky scene knobs are gone.
3. **The distant sea strip is a mirror, not a perspective grid.** Neonscape's lake mirror redraws every sky
   object below the horizon for almost nothing, which gives the reflections Ron asked for. The grid floor
   and its two per-line divides are removed, and that logic pays for the new parts. If there is room at the
   end, far-sea swell lines can go back into the strip.
4. Night is a switch and the slider is elevation, as in Pixel Beach.
5. Sky colour follows the sun by sliding along one long colour strip (section 7.1), so it changes smoothly
   as the slider moves. The other colours step between palettes, as Pixel Beach's did.
6. Wet sand is a per-cell counter in block RAM. Pixel Beach tried a formula first and Ron rejected it; the
   counter version is the one he called "looks perfect" (TECHNIQUES, the per-cell wet-sand RAM entry).

## 4. What each parent gives

**From neonscape (fork it; this is the code base):**

| Piece | Where (neonscape.vhd at d712b01) | Use |
|---|---|---|
| Timing chassis: virtual 1080-line steps, vblank zone, width latch, generated avid | `p_ctrl` top, `p_gavid` | unchanged |
| Line engine in horizontal blanking | `p_line` | keep, without the two divides and the grid states |
| Region-ID merge, ID line store, wire detector, colour table read, lerp, glow | `p_mix` stages 15 to 24 | unchanged in structure |
| Output conditioning (chroma 9-tap, luma [1 2 1], luma 13/16, blanking gate) | `p_out` | unchanged |
| Near range: a ridge with a draped mesh, columns at the knots and rows at fractions of the local height, by a pipelined per-pixel divide | `p_obj` near block | becomes the water; its knots come from a RAM instead of a constant table |
| Far range: a ridge from a constant knot table | `p_obj` far block | becomes the sand's profile |
| Sun and moon from the half-chord table, crescent bite, slits | `p_line` states 2 to 4, `p_obj` sun block | keep; elevation becomes a signed per-field value |
| Stars | `p_obj` | keep; density becomes variable |
| Span-coded sprite slots A and B | `p_line` states 3, 34 to 44; `p_obj` slot block | A = clouds and gulls, B = the crab |
| Lake mirror: sky objects are functions of `d`, the distance from the horizon, so they redraw below it | `e_sky`, `e_mir`, `l_rip`, the wobble in state 5, the dimming at stage 23 | the distant sea strip |
| Camera source, smoothing, gain | `p_src` | unchanged |
| Build hook that writes the package (palettes, sprite rows, tables) | `neonscape.py` | fork as `neonbeach.py` |

Remove: the floor grid (both serial divides, `b_u`, `b_w`, the glow-band logic), city, palms, both cars,
road, tunnel, rain, the second cloud band, Pan view and its parallax scrolls, the Land, Sky and Colour zone
decoders, the sheen and sharp switches. The fill glide stays, but its target is the Fill switch (0 or full).

**From Pixel Beach (the picture, the behaviour and the lessons; none of its code carries over):**

- The composition: beach at the left, deep water at the right, waves travelling right to left; sea level at
  about 58 % of the screen height; a flat sea strip between the horizon and the waves.
- The wash sheet: it spreads down into the sand from the sand's surface line, over the left half of the
  screen, with a wavy edge. It runs out fast, drains slower, then pauses; about 3.4 s a cycle at the default
  tempo; taller waves and a higher tide send it further.
- Wet sand set while the wash covers a cell and drying by time: 15 steps, one every 12 fields, about 3 s.
- The slider as sun elevation; stars appear as the sun sets; gulls roost when it is dark; gulls fly just
  above the horizon so their reflections land in the sea strip.
- No two waves alike (it re-rolled each wave's height by up to 40 %), and a slow surge that raises and
  drops the whole water level.
- The crab: wanders the sand, pauses, runs from the wash; switched off it burrows, switched on it comes up
  somewhere new.
- Measured costs there, as a warning: crab 619 logic cells, gulls 216, wet sand 185, sun reflection 170,
  stars 130, spray 76. The crab must be far cheaper here (section 7.6).

## 5. The picture

Positions are virtual rows on a 1920 x 1080 canvas, measured from the top unless stated. Water and sand
heights are measured upward from the bottom edge and called `yup`. SD shows the centre 4:3 (x = 240 to
1680), so everything that matters stays inside that.

| Element | Position | Notes |
|---|---|---|
| Sky | rows 0 to 356 | horizon fixed at row 356 (33 %) |
| Sea strip | row 356 down to the water or sand in front | the sky mirrored about the horizon, with ripple gaps |
| Still sea level | row 626 (58 %) at Tide 50 % | Tide moves it about 96 rows either way |
| Sea bed and beach | one profile across the screen | deep at the right edge (row 1010); rises to the left; meets sea level near x = 860; a beach crest near x = 420 at row 560; dips a little to the left edge |
| Water | between the surface profile and the bed | waves enter at the right and break toward the left |
| Sand | everything under the bed line | mesh cells 32 px; the wash runs over it left of about x = 960 |
| Sun, moon | radius 128, centred on x = 1200 | both use the half-scale chord lookup the moon uses now |
| Gulls | a band 60 to 124 rows above the horizon | 64 rows tall (neonscape's are 32) |

Objects and their ID numbers (the 4-bit object field; a wire takes the colour of the highest number among
its neighbours; keep to 0 to 9, because table entries 26 to 28 are neonscape's glow, star and rain colours,
of which the star entry stays and the other two are free for the spray):

| ID | Object | Facet bits | What moves |
|---|---|---|---|
| 0 | sky, and the sea strip behind everything | none | stars twinkle, ripple gaps drift |
| 1 | sun / moon and its mirror | none | slits roll; the mirror wobbles |
| 2 | clouds, gulls and their mirrors | sprite zone | drift, flap |
| 5 | water | layer parity, column parity | the whole mesh rides the surface |
| 6 | crest and foam | hashed bit, diagonal | churns; thickness follows the foam value |
| 7 | wash sheet (covered sand triangles) | column, row, diagonal | appears and recedes by triangle |
| 8 | sand | column, row, and the diagonal while damp | diagonals vanish as it dries |
| 9 | crab | sprite zone, crease | walks, legs alternate |

Spray and stars are overlays (the `s15_ovl` path), not objects. Per pixel, nearest first: crab, then
sand or wash (below the bed line), then foam, then water (below the surface, above the bed), then the sky
objects or their mirrors.

## 6. Controls

| Control | Name | What it does | Default |
|---|---|---|---|
| K1 | Rhythm | tempo of the waves and of the slow surge, and so of the wash; 0 % is slow, never stopped | 40 % |
| K2 | Tide | water level: walks the shoreline up and down the beach, and widens or narrows the wash | 45 % |
| K3 | Foam | how much foam the waves make: the thickness of the churned band under a crest, whitecaps on steep fronts out at sea, the height and density of the spray | 40 % |
| K4 | Chop | a second, short, faster wave riding on the swell, plus a jitter on each crest | 40 % |
| K5 | Swell | sea energy: wave height, how steep the fronts get, the depth of the surge, how far the wash runs. 100 % is the designed climax: faces walling up to the horizon and a wash that crosses most of the sand | 50 % |
| K6 | Camera | camera or Osc strength, default 100 %: lights the sky strip, raises the water surface, raises the sand line | 100 % |
| S7 | Fill | Wires / Fill. Flipping it rolls the gradients in (or out) object by object over about a second and a half: sun, sky, water, foam, wash, sand, sprites | Wires |
| S8 | Colour | two themes, one warm and one cool | warm |
| S9 | Crab | On / Off (burrows and re-emerges, never just vanishes) | On |
| S10 | Sky | Day / Night: sun or moon, the sky strip pinned to its dark end, full stars, gulls roost | Day |
| S11 | Source | Video / Osc | Video |
| P12 | Sun | elevation of the sun or moon, glided: 0 % fully set, 100 % high. The sky strip, the sun's colour, the palette and the star density follow it | 60 % |

K1 to K5 are in Pixel Beach's positions with Pixel Beach's meanings. Gulls have no switch: they fly whenever
it is Day and the sun is not fully set.

Every control must change the picture at any setting of the others. That needs care here, because the five
wave knobs feed one profile:

- Swell and Rhythm never reach zero: the bottom of Swell is a low, clean swell, not a flat line.
- Foam must show at any Swell. Besides the foam that breaking makes, it sets a whitecap term (steep fronts
  anywhere) and a foam line at the shoreline, so it is visible on a calm sea too.
- Chop is added to the height on its own gain, not multiplied by Swell.
- Tide moves the level directly.
- None of the five multiplies another knob that already spans its full range. Swell scales the breaking
  amplitude; Foam scales what the breaking injects. Those are different quantities.

## 7. The new parts

Starting values are given so that the first render is close; tune them from renders, not from this text.

### 7.1 Sky, the colour strip, stars, the mirror

- `c_sunc` (the body's centre, rows above the horizon) becomes signed and comes from the glided slider:
  about -150 (fully below) to +220. The line engine's `e_dyu = d - c_sunc` already works for either sign,
  and on mirror lines the same `d` gives the reflection. A sun sitting on the horizon then shows as a full
  disc, half sky and half mirror.
- Sky colour: give the sky two ramp slots (ramp 0 and the city's ramp 3), making one strip of five keys and
  256 lerp positions: darkest night, dusk, the hot band, pale, brightest. A sky row's position on the strip
  is its distance from the top, scaled to 0 to 127, plus an elevation term of 0 (set) to 128 (high), clamped.
  So the sky is the same gradient at every elevation, slid along the strip. Night pins the elevation term at
  0 and subtracts a little, so the upper sky sits on the darkest key.
- The sun's own ramp position gets the same kind of offset, so a low sun is redder.
- Palettes: the colour table still holds eight palettes of 32 entries. The palette number is three bits:
  the Colour switch, the Sky switch, and "the body is low" (elevation under about a third, with a few counts
  of hysteresis). So each theme has four palettes: day, low sun, night, low moon. The sky slides smoothly;
  wire colours and the water and sand ramps step at the threshold, as Pixel Beach's palette did.
- Stars: the hash test `v_hh(11 downto 8) = "0000"` becomes `< density`, a 4-bit per-field value that rises
  as the sun goes below about +40 and is full in Night.
- The mirror: every line below the horizon is a mirror line (what `s_lake` selects now). Two things in
  neonscape act on the whole line and must become per pixel, because water, sand and the crab sit in front
  of the mirror on the same lines: the ripple gap (`l_rip` forcing black) and the 25 % dimming (`l_mir`
  at stage 23). Apply both only where the pixel's object number is below 5.

### 7.2 The wave filler and the knot RAM

**Knot RAM.** 256 words of 32 bits (two block RAMs). Knots are 16 virtual px apart, so 120 of them cross
the screen. Word `k` holds the surface height at knot `k`, the height at knot `k + 1` (so the pixel path
gets both ends of a segment in one read), and a 4-bit foam value. Heights are `yup` in half rows (11 bits).
The pixel path reads it during active video; the filler reads and writes it in the vblank zone. One read
statement with a muxed address, one write statement.

**Wave filler.** A sequencer in the vblank zone, one small operation per state, that walks the knots from
the right edge (open sea) to the left (beach). About 16 states a knot, so about 2,100 clocks a field. Each
of the five wave knobs is latched once per field and enters at exactly one place below. For each knot:

1. Still depth = sea level (Tide plus the surge) minus the bed height at the knot. The bed comes from the
   same constant table the pixel path uses; give the filler its own small copy at knot spacing rather than a
   second read of the pixel path's table.
2. Phase: a running sum that starts from a per-field value (stepped by Rhythm) and grows by a step for every
   knot. The step is larger in shallower water (a table of eight steps indexed by depth), so the waves
   shorten as they come in. Start with a deep-water wavelength of about 500 px.
3. The integer part of the phase is the wave's number. A 16-entry constant table indexed by it gives that
   wave's size (3 bits), so waves come in sets and each keeps its size all the way in. No state.
4. Amplitude = Swell x that size x a shoaling gain that rises as the depth falls, then capped at three
   quarters of the still depth. The cap is the breaking rule: waves grow toward the shore and then shrink
   with the water. Write every product as gated shift-adds.
5. Shape: a sawtooth whose front (the left, shoreward side) takes 1/2 of the wavelength in deep water, 1/4
   in the shoaling zone and 1/8 where the cap is active. Pick the split by depth; all three are shifts.
6. Chop: a second phase sum with a wavelength about a quarter of the swell's and its own, faster per-field
   step. Its triangle, times the Chop gain, is added to the height; it is not capped by depth, but it fades
   to nothing over the last few knots before the shoreline. Chop also adds a per-knot jitter to the swell's
   amplitude (a hash of the knot and the wave's number, times the Chop gain), so crests are uneven.
7. Height = sea level + amplitude x shape + chop, then a [1 2 1] smoothing across neighbouring knots to
   round the corners. Never below the bed.
8. Foam: three sources, each times the Foam gain, largest wins. Breaking: the amount the cap removed.
   Whitecaps: the height step to the left-hand neighbour, where it exceeds a threshold (a steep front
   anywhere). The shoreline: a fixed amount at the last wet knot. If none applies, the knot takes its
   right-hand neighbour's foam from the last field, less a decay, which carries foam shoreward with the
   wave. This is a read, modify, write on the RAM; keep it in the pipelined shape and write two clocks after
   the address.
9. Swash: when a wave's number changes at the shoreline knot, a run-up starts. A front position runs up the
   beach quickly (about 0.6 s), by a distance set by Swell, that wave's size and Tide, and drains back
   slowly (about 1.5 s). Between the shoreline and the front, the surface is the bed plus a thin sheet that
   tapers to nothing at the front. The same event and the same reach drive the wash on the sand (7.4).

The surge is one slow triangle added to the sea level, paced by Rhythm, with its depth from Swell.

Where each knob enters: Rhythm, the per-field phase steps (swell, chop, surge); Tide, the sea level and the
wash's limit; Swell, the amplitude before the cap, the surge depth and the run-up distance; Chop, step 6;
Foam, step 8 and the spray band in 7.3. If the solver ever replaces this filler, the same five knobs map
onto it as they did in Pixel Beach.

Start the filler from the zone counter each field, after the field constants. It has no free-running state
that could strand it, but give it a done flag that the next field's start does not depend on.

### 7.3 Water, crest, foam and spray (pixel path)

This is neonscape's near range with a different height source. Suggested stage layout, ending at stage 14
as every object does now:

| Stage | Work |
|---|---|
| 1 | knot RAM address from the water's world-x counter (the knot index) |
| 2, 3 | RAM output register, then a fabric register |
| 4, 5 | surface height = knot 0 + (knot 1 - knot 0) x fraction, as gated operands; the bed's table lookup and lerp run alongside (the far range's code) |
| 6 | add the camera lift to both |
| 7 | water test (`bed <= yup < surface`), sand test (`yup < bed`), numerator `yup - bed`, denominator `surface - bed` |
| 8 to 10 | a 3-bit divide: layer = floor(8 x numerator / denominator) |
| 11 to 13 | facet bits, foam band, spray flag; the sand's work (7.4) shares these stages |

- The mesh layers are fractions of the local water depth, so they bunch in the shallows and stretch under a
  crest. That is where the picture of a moving wave comes from. Neonscape's divide is nine stages because
  its bands also flow; here three quotient bits are enough, which frees about six stages of registers.
- Level of detail: use all eight layers where the depth is over 128 rows, four where it is over 64, two
  over 32, none below. Otherwise the wire detector turns the shallows solid.
- Columns: a parity bit of world x every 64 px.
- Crest and foam (object 6): the top rows under the surface, as thick as the foam value says (0 to about 48
  rows). Its facets are a hashed bit per small cell plus a diagonal bit, which reads as churned triangles.
  Because 6 is above 5 the surface line over foam is drawn in the foam's colour: that is the bright crest.
- Spray: an overlay dot where `yup` is within a band above the surface (band height from the knot's foam
  value, which already carries the Foam knob) and a hash of the 8 px cell and a few bits of the field counter
  passes. Reuse the star hash; give the dots a free overlay colour entry.

### 7.4 Sand, the wash and wet sand

**Mesh.** The sand's depth at a pixel is `bed - yup`. Cells are 32 px wide and 32 rows of depth, so the rows
run parallel to the sand's surface. The diagonal bit is a compare of the two 5-bit fractions.

**Wetness RAM.** 2048 x 4 bits (two block RAMs), address = depth row (5 bits) and column (6 bits). Value 15
= fully under the wash this field, 14 = only the upper triangle is, 13 to 1 = drying, 0 = dry.

**Sweep.** Once per field in the vblank zone, visit every cell (one a clock, a pipelined read, modify,
write, about 2,100 clocks):

- the reach for this column, in quarter cells of depth = the per-field wash reach, plus two triangle waves
  of the column index with drifting phases (the wavy edge), tapering to zero over the last columns before
  the wash's right-hand limit near x = 960 (Tide shifts the limit a little);
- columns that are under water (right of the shoreline column, which the wave filler records) have a reach
  of one cell, so the sea bed's top row is always wet;
- if the cell's whole depth is inside the reach, write 15; if the reach ends inside it, 14; otherwise, if
  the value is 14 or 15 write 13, and on a dry tick (every 12 fields) subtract 1 down to 0.

**Reach.** It follows the swash front from the wave filler: the same trigger, run-out and drain, scaled to
rows of depth (up to about 8 cells at full Swell).

**Drawing.** With the diagonal splitting a cell into an upper and a lower triangle, and `h` a 2-bit hash
of the cell:

| Value | Upper triangle | Lower triangle |
|---|---|---|
| 15 | wash (object 7) | wash |
| 14 | wash | sand, diagonal shown |
| 13 down to 0 | sand; diagonal shown while value >= 3 + h | sand; diagonal shown while value >= 8 + h |

So the wash arrives a triangle at a time along a ragged front, and when it drains the sand keeps its
triangles for a while and loses them piecemeal. In fill, lower the sand ramp's position by the value so wet
sand is darker.

**Port sharing.** One read statement, its address muxed between the pixel path (active video) and the sweep
(vblank zone). One write statement, from the sweep only. Read address at stage 8, value in a fabric register
by stage 11. Set `no_rw_check` (the sweep's write trails its read by a fixed number of cells) and give the
read register an init value.

### 7.5 Reflections

They come from the mirror in 7.1: the sun or moon, gulls, clouds and stars all redraw in the sea strip,
dimmed and broken by ripple gaps. Nothing more is needed for a first version. If logic allows at the end:
a sand or wash pixel with a wetness of 8 or more, in the columns under the sun, takes the sun's colour, so a
streak of reflection runs down the wet sand and fades as it dries.

### 7.6 Crab

Slot B holds only the crab. Two frames of art (legs alternating), about 256 x 128 virtual px, drawn in the
build hook as a wire structure: shell with crease lines, eye stalks, two claws, legs. One size.

It moves in screen coordinates inside a fixed region of the foreground sand, chosen in the build hook so
that every position is below the bed line (about x = 280 to 800, rows 700 to 1000). Per-field behaviour, one
small step per state, driven by a free-running LFSR. Budget 200 logic cells for all of it:

- WALK: step sideways, sometimes a row up or down; PAUSE at random.
- FLEE: when the wash reach, in rows, comes within about 64 rows of the crab, run down the screen until it
  is clear.
- BURROW (switch off): raise the sprite's clip row a few rows a field until it is gone. HIDDEN. EMERGE (switch
  on): pick a new place, lower the clip.
- The hole, if logic allows: when the beam passes the crab's foot pixel, latch the wetness address there;
  the sweep forces that cell to 13 while the crab is hidden, so the hole is a triangle that fades like wet
  sand.

Pixel Beach bugs to avoid: a 2-bit sink counter that wrapped (the body reappeared a row lower), and a pop-up
check that mixed the new candidate's position with the old one's ground.

### 7.7 Camera and Osc

Keep `p_src` as is. The three effects: add the gained source to the sky strip position (the picture shows
in the sky, as now), add it to the water's surface height, and add it at a lower gain to the sand's line.
With a black input all three are zero. The lift has no vertical smoothing, so a busy picture makes a ragged
surface; that is neonscape's known limit and is accepted.

### 7.8 The Fill and Colour switches

Fill: neonscape's fill position (`c_fp`, which each line compares against per-object offsets to set the fill
flags) stays, with its glide. The glide's target is 0 for Wires and the top for Fill, and its rate is fixed
so the whole roll takes about a second and a half, instead of an eighth of the gap per field. The camera
still lights the sky strip in Wires, as it does in neonscape at Fill 0 %.

Colour: one bit of the palette number (7.1). Latch it per field with the other switches.

## 8. Budget

Measured, neonscape v0.3.1: 7,285 of 7,680 logic cells (94.9 %), 21 of 32 block RAMs, HD HDMI 75.17 MHz on
its one good seed. The latest standalone yosys log there (`stat_v14.log`): 4,269 LUT, 1,721 carry, 2,657 FF.

Measured on an early neonscape (the ablation logs in `.neonscape_work/stat_*.log`): the near range's
nine-stage divide is worth 120 LUT + 243 FF; the stars 55 LUT + 170 FF; output conditioning 163 LUT + 478 FF.

Everything below is my estimate and has not been measured:

| Change | Logic cells |
|---|---|
| Remove the floor grid, its divides and accumulators | about -500 |
| Remove city, palms, cars, road, tunnel, rain, Pan, scene and colour decoders | about -480 |
| Shorten the water's divide from nine stages to three | about -200 |
| Knot RAM read path, wider lerp, bed and water tests | +150 |
| Wave filler with chop and the three foam sources | +380 |
| Crest, foam, spray | +100 |
| Sand mesh, wetness read path, sweep, reach | +250 |
| Elevation, sky strip, palette select, star density, per-pixel mirror flags | +110 |
| Crab behaviour | +200 |
| Camera changes, the Fill switch's fixed-rate glide | +30 |

By that count it comes to about 7,300, which is too full. **Target 7,200 or less (94 %) on the fullest
config**; stage 1 replaces these guesses with a measurement. Order of cuts if it does not fit: the crab's
hole, the wet-sand sun streak, the clouds, the per-object sheen table (one shared sweep for everything),
the surge. Do not cut a wave knob to make it fit; ask Ron first.

Block RAMs: 21 + 2 for knots + 2 for wetness = 25. Stay under 28.

Vblank work per field: field constants, then the wave filler (about 2,100 clocks), then the wetness sweep
(about 2,100). NTSC has about 14,500 clocks of vblank, so it fits on hardware in every mode.

## 9. Build order

Each stage ends in a full build (`./build_programs.sh ron neonbeach`), a report of the last post-route
"Max frequency" and logic cells for all six configs, and a version Ron can flash. Do not stack stages
before the previous one has built.

**Stage 0: a look model.** An integer Python model of the wave filler and the cross-section (surface, bed,
water layers, foam, the sand mesh and its wetness states), written with the bit widths the hardware will
use and taking the five wave knobs as 10-bit inputs. Render a handful of 720p stills and a six-second
animation at Swell 25, 50 and 100 %, plus a contact sheet that sweeps each of the five knobs alone from 0
to 100 % with the others at their defaults, in `.neonbeach_work/proto/`. Every frame of that sheet must
differ visibly from its neighbours; that is the test of "every knob shows at any setting". The wave shape is the part of this program most likely to look wrong, and here
a change costs seconds. Settle the wavelength, the shoaling steps, the cap, the front splits and the wash
timing. Show Ron and start stage 1 without waiting. Keep the model as the reference for a pixel diff later.

**Stage 1: fork and strip.** Copy neonscape to `programs/ron/neonbeach/`, rename the architecture, package
and tables, and delete what section 4 lists. Fix the horizon at row 356 and make every lower line a mirror
line. Build. Record the logic cells: that number decides how much of section 7 fits.

**Stage 2 (v0.1.0): the still scene.** Slider elevation, moon and Night, the sky strip, star density, gulls
and their mirror, the per-pixel mirror flags, the Fill switch, the sand profile and its plain mesh, and flat
water at the level Tide sets (the knot RAM written with a constant), with its layers and columns.

**Stage 3 (v0.2.0): waves.** The wave filler with all five knobs live, foam, crest, spray, the swash sheet
on the beach face.

**Stage 4 (v0.3.0): the wash.** Wetness RAM, sweep, reach, the triangle states.

**Stage 5 (v0.4.0): crab, camera effects, palettes, presets.** Two themes of four palettes each (start from
neonscape's Sunset and Ice and rework them for the five-key sky strip and the new objects); presets are
hand-written placeholders (names of 15 characters or fewer) until Ron captures real ones from the device.

Renders: one set per stage for Ron to look at, at 720p60 with decimation 1. The image tester's vblank is a
fake eight lines (10,752 clocks at that raster), which holds the filler and the sweep; the default small
raster's does not. Use a copy with the glides snapped, as neonscape does (`.neonscape_work/sim.sh` and
`simsnap/`; copy them to `.neonbeach_work/` and change the program name). A render is one still, and the
waves only start to look right after a few seconds of fields, so judge motion on hardware and from the
stage 0 model.

## 10. Traps specific to this build

- **Scripted edits.** Neonscape v0.3.0 shipped with a counter's only driver deleted by a patch script; the
  build was clean and the logic count simply fell. Run `.neonscape_work/undriven.py` on the file after any
  scripted edit, and after the strip in stage 1.
- **One latency.** Keep `C_LAT` = 33 and `processing_delay_clks` = 33 whatever is removed. Shortening the
  pipe is a separate change with its own hardware check.
- **Two RAMs with two users each.** The knot RAM and the wetness RAM are read by the pixel path in active
  video and by a sequencer in the vblank zone. The address mux select must be the registered zone flag, and
  both sequencers must be finished before the first active line. Sims run the behavioural array and cannot
  show a read collision; reason it out from the code.
- **Block RAM start-up.** The knot RAM powers up as zeros, which is a surface at the bottom of the screen.
  The filler rewrites every knot every field, so it clears itself in one field; do not add state that
  depends on the previous field's heights without a reset path.
- **Foam must not pump.** The foam carry (neighbour's value less a decay) is a feedback term: use a
  subtract-and-clamp, not a floor shift. Pixel Beach's physics charged and collapsed on a signed floor
  shift. Run the stage 0 model with Swell, Chop and Foam all at 100 % for 600 fields before trusting it.
- **Vsync.** The filler, the swash, the reach, the crab and the sweep all step once per field. Step them
  from the existing zone counter (`s_zc`), never from a vsync edge or level.
- **Small products.** Write every gain as gated shift-adds, each partial sum one add of gated operands. A
  `case` of adders has broken abc9 twice in this repo, and `*` on small operands costs 150 to 250 cells.
- **One read site per constant table.** The bed table is wanted by the pixel path and by the filler. A
  second read site is a second copy of the table; give the filler its own small table on purpose.
- **Neutral chroma.** Any new register that can reach the screen (spray colour, the per-pixel dimming)
  needs a neutral reset: chroma code 0 is saturated green.
- **Resolution.** Everything is in virtual rows, so SD, 720p and 1080 show the same picture, with SD the
  centre 4:3: the back of the beach at the left and the deepest water at the right are cut. On 480i a field
  line is 4.4 virtual rows, so the water's layers in the shallows, the gulls and thin foam are coarse.
  Report what differs.
- **Bars on HD HDMI.** The output conditioning that prevents them is inherited. Do not thin it to save
  logic.

## 11. Done means

- `out/rev_b/ron/neonbeach.vmprog` holds six bitstreams, each with a passing last post-route frequency line.
- Logic at or under 94 %, block RAMs under 28.
- `SUMMARY.md` in the program folder, in the shape of neonscape's: how it draws, objects, controls, a
  resolution-differences section, known limits, the seed table (any seed other than 1 recorded), and the
  hardware status stated plainly. Until Ron has flashed it that status is "timing-closed only".
- The TOML passes the converter pre-check; description under 127 bytes; categories from the fixed list
  (`["Shape", "Render", "Demo"]` are valid).
- Committed on the working branch (own hunks only), a status line added to the memory index of programs, and
  this file deleted once `SUMMARY.md` covers it.
- TECHNIQUES entries for "a ridge whose knots come from a RAM a sequencer fills each field" and "per-cell
  state that subdivides a mesh" only after Ron confirms them on hardware.

## 12. Read first

1. `programs/ron/neonscape/SUMMARY.md`, then `neonscape.vhd` end to end (1,600 lines; it is the code base).
2. `programs/ron/neonscape/neonscape.py` for the palette entry layout and the span-coded sprite rows.
3. `programs/ron/pixelbeach/SUMMARY.md`, the header comment of `pixelbeach.vhd` (lines 1 to 90) and
   `beach_proto.py` for the composition and the behaviours. The rest of that file is not needed.
4. TECHNIQUES.md: "Region-ID wireframe", "Span-coded sprite rows", "Virtual 1080-line coordinates", and the
   Pixel Beach entries on the per-cell wet-sand RAM and on feedback terms that pump.
