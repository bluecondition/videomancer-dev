# ORRERY (v0.2.1 — the solar system, with planet surfaces and Focus)

**STATUS: HW-iterating — v0.2.0 flashed by Ron 2026-11-14 (it runs; his one
note: "saturn appears much larger than jupiter"). v0.2.1 (Saturn cut to 0.71
of Jupiter, ring with it) is built, timing-closed, NOT flashed. No RTL sim.** Its arithmetic is mirrored
by an integer model (`.moon_work/model_orrery.py`), but nothing has compared
the RTL against it.

The flashed v0.1.0 is kept whole in `.moon_work/orrery_v010/` (vhd, toml,
pkg, SUMMARY, vmprog).

## What it draws

The Sun, eight planets on their orbits at Kepler's relative speeds, each a
ball lit from the Sun's side and carrying its own small surface picture that
turns with the planet: Earth's continents (from NASA's Blue Marble),
Jupiter's bands and red spot, Mars's dark markings and polar caps, Saturn's
bands, Neptune's dark spot. Saturn's ring has striations and a gap. Asteroid
belt, stars, one-pixel orbit lines. The whole system tilts from straight-down
to nearly edge-on, and Focus puts any body at the centre of the screen so
the rest of the system wheels round it. Not to scale.

| | |
|---|---|
| K1 Zoom | whole system .. inner planets |
| K2 Speed | centre = still; right = forward in time, left = backward |
| K3 Stars | star density, 4 steps |
| K4 Orbits | brightness of the orbit lines (0 = none) |
| K5 Spin | the planets turning on their axes; centre = still, either way |
| K6 Focus | 9 steps: Sun, Mercury, Venus, Earth, Mars, Jupiter, Saturn, Uranus, Neptune |
| S7 Sky | Black / Video |
| S8 Belt | Off / On |
| S9 Rings | Off / On |
| S10 Shading | Sunlit / Flat |
| S11 Planets | Small / Large (planets x 4, Sun x 2 — the surfaces show at Large) |
| P12 TILT | straight down -> nearly edge-on |

### Changed from v0.1.0 (to pay for the surfaces and Focus)

- **Size knob -> S11 Small / Large switch.** Planet radii are now powers of
  two (8 / 16 / 32 virtual lines at Small; 32 / 64 / 128 at Large), which is
  what makes the surface lookup a shift instead of a multiply.
- **Glow knob removed**: the Sun's glow is fixed (it grows with Large).
- **Motion Kepler / Locked switch removed.**
- **Zoom and Tilt no longer glide**: they follow the knob directly (with a
  two-count hysteresis against jitter), so a slow turn moves in small steps.
- Stars stayed, as a 4-step knob (it costs almost nothing).
- The Sun is one plain colour (no surface picture).

### v0.2.1 (after Ron's first look at v0.2.0)

- **Saturn was the same ball as Jupiter, and its ring made it look far
  bigger.** Saturn's r^2 is now half its power of two (ball 0.71 of
  Jupiter's) and the ring field is read one bit lower, so the ring keeps its
  1.41 r .. 2.24 r proportions on the smaller ball. The ring still reaches
  about 1.6 x Jupiter's radius (was 2.2 x). Saturn's picture keeps its
  power-of-two mapping, so the ball shows the middle 71% of it (bands only,
  so the crop does not show).
- r^2 is looked up and shifted in its own sequencer state (24), subtracted
  from cx^2 in the next (25), and cy^2 added in the store state (26): with
  all three in one state the Saturn change cost 10 MHz on most seeds.
- Shade: 64 brightness levels for luma, 4 for chroma; shade range 32 .. 255
  (was 40 .. 240, base 144 -> 160).

## How it works (26 EBR)

- **Half virtual units** everywhere (X = xs x pixel with xs = 2 / 3 / 4,
  Yh = 2 x virtual line): exact integers in every mode.
- **Bodies:** e = (X - cx)^2 + (Yh - cy)^2 - r^2 < 0. One 29-bit accumulator
  per body; all share ONE D accumulator (2 xs X + xs^2) plus a per-body
  constant. The sign bit is "inside".
- **State RAM** (256 x 32, `no_rw_check`): orbit angles at 0 .. 8, line
  values E at 16 .. 24. The line job (hblank) reads each E, steps it
  (E += 4 Delta dy + 4 Delta^2), writes it back and loads that body's
  accumulator from the same write. The dy values stay in a register ring.
- **Orbit field** ec = (X - CX)^2 + U^2, U = (Yh - CY) / tilt. Eight
  comparators against A_i^2 / 256 give "inside orbit i"; the lines are where
  that count changes versus the pixel to the left or the line above (4-bit
  line buffer). The belt is count = 4 sprinkled by a hash.
- **Own-orbit terminator:** shade = 160 + (A^2 - ec) >> n, clamped by bit
  tests. No light vector, no multiplier.
- **Surfaces:** eight pictures, 64 columns x 32 rows x 4-bit, in one ROM
  (`C_OR_TEX`), each with its own 16-colour palette (`C_OR_PAL`, which also
  holds the Sun's colour and the ring's 16-step striation palette).
  Column = (X << sc) + a per-body constant worked out in vblank
  (scroll - (cx << sc)); row = dy << sc, clamped. sc depends only on the
  body's radius, so there is no divide and no multiply, in any video mode.
  The visible face is 32 columns wide; Spin scrolls the column.
- **Saturn's ring:** the tilted field centred on Saturn, between 2 r^2 and
  5 r^2; its radius indexes the ring palette. Over the ball below its centre
  line, behind it above.
- **Focus:** the chosen body's offset from the Sun (px, py, taken from the
  field before) is subtracted from the centre of everything. A one-field lag,
  invisible at these speeds. This is "that planet held at the centre", not a
  first-person view from its surface.
- **Depth:** front-half planets, then the Sun, then back-half planets.
- **Vblank sequencer:** one serial 17 x 16 multiplier, a serial divider
  (65536 / tilt), 1024 x 15 sine ROM with linear interpolation, about 14
  products per body. Busy flags (not counter zero-tests) gate the states.
- **Edges:** the finished picture goes through the horizontal [1 2 1] / 4 on
  Y, U, V (the HD HDMI bars cure), folded into the gated output register.
- Clean avid as LUNA; latency 8 in every mode.

## Files

- `orrery.vhd`, `orrery.toml`, `orrery_pkg.vhd` (sine table, pictures,
  palettes), `make_pkg.py` (writes the package; Earth from
  `.moon_work/src_earth/bm_svs.png`, everything else procedural; preview in
  `.moon_work/model_out/_orrery_textures.png`).
- `orrery.vhd` was assembled by scaffolding scripts
  (`.moon_work/gen_orrery_v02*.py`, a stack of patches on v0.1.0). **From
  here on the VHDL is the source: edit it directly.**
- `.moon_work/model_orrery.py` — integer model (`model_out/_o2sheet.png`).

## Build

`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_programs.sh ron orrery`

v0.2.1, 2026-11-14, router1: **HD Analog 80.2 (seed 5)** / HD HDMI 79.2
(seed 1) / **HD Dual 77.7 (seed 3)** MHz, SD Analog 71.7 (seed 1) / **SD
HDMI 69.1 (seed 2)** / SD Dual 70.3 MHz (seed 1, target 27); 6872-6901 /
7680 LC (89-90%), 26 / 32 EBR.

**Build seeds (router1):**
- HD Analog = seed 5, installed by hand: the builder's seed 1 passed at only
  74.66 MHz, so that one config was rebuilt with `make ... CONFIG=hd_analog
  SEED=5` and the vmprog repacked with `vmprog_pack.py`. A plain re-run of
  the builder goes back to seed 1 (still a pass, 0.4 MHz of margin).
- HD Dual = seed 3 (seeds 1 and 2 missed timing).
- SD HDMI = seed 2 (seed 1 stalled the router until the 900 s cap).
- HD HDMI, SD Analog, SD Dual = seed 1.

(v0.2.0, the build Ron flashed: HD Analog 77.2 seed 1 / HD HDMI 77.6 seed 3 /
HD Dual 84.3 seed 1.)

Margins are thin and seed-dependent: on HD Analog trial routes of v0.2.1,
four of the five seeds that finished passed (74.7 - 80.2 MHz). Any addition
needs a matching cut.

## Fit and timing history (v0.2.0)

- First cut (mantissa radii with multiplies, register rings, 4-step Size):
  ~900 cells too big.
- Power-of-two radii, E and angles into the state RAM, Size as a switch, a
  plain Sun: fits at 95%, 67 - 74 MHz.
- No glides, one sine temporary, 22-bit carried bus, shade x colour by
  gated shift-adds: 91%, 69 - 73 MHz, four different critical paths.
- Those four: the shade clamp as bit tests; Zoom / Tilt hysteresis as two
  small units; the Speed / Spin rate from a registered knob; multiplier and
  divider busy flags: 90%, three seeds of six pass.
- The picture column was the last one (select -> 9:1 constant mux -> add,
  then x 85 / 512 for 720p -> barrel -> add). Rewritten as
  (X << sc) + per-body constant: four seeds of six pass, and 720p needs no
  special case.

## Not verified / known limits

- No GHDL render, and the image tester cannot run it as-is (the line job
  needs ~95 clocks of hblank; the tester gives 64). First check is the
  hardware.
- Sky = Video is untested (as in v0.1.0).
- At Small the planets are 8 - 32 lines in radius, so the surfaces read as a
  few blotches; they are meant to be seen at Large or zoomed in with Focus.
- At Large with Zoom low the planets overlap each other and the Sun.
- Zoom and Tilt step (no glide); Focus jumps between bodies (no travel).
- Planet size does not follow Zoom (only the orbits scale).
- Shade contrast varies up to 2x across a size octave (the normalising
  shift is a power of two).
- Presets are hand-typed placeholders.
- SD and 720p draw the same layout from the same virtual coordinates, but
  have not been looked at on hardware.
