# Outrun (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. Flash `out/rev_b/ron/kinetic/outrun.vmprog`.
v0.1.1 (2026-11-15): the K6 camera pass (K6 = Camera, Colour moved to K2, Shape fixed); see "K6 camera pass".

## What it draws

The neon perspective grid of 1980s computer graphics: a glowing grid floor running to a horizon, a banded
sun sitting on it, optionally a mirrored ceiling. At the defaults (Outrun palette) a cyan grid on dark
purple flies toward the viewer, its rows rolling as V-shaped ridgelines; the horizon is a two-tone glow
(pink haze above, cyan below); an amber-to-pink sun with scrolling stripes stands on the horizon.

## Element and state

- Element: a z row of the grid. State: the terrain height h under it, in cells.
- Perspective, per line: s = |row - horizon| (virtual rows, never under 8); z = Zk / s cells deep;
  u = (x - centre) Uk / s cells across (the screen centre sits mid-lane).
- theta = phase + Spread (z + |u|)                      Field = Linear: V-shaped fronts rolling toward the viewer
        = phase - Spread (|z - zc| + |u| / aspect)      Field = Radial: diamond ripples spreading from the floor
                                                        point 200 rows below the horizon
- h = Depth wave(theta) + source. wave is a sine, or Step: the sine quantised to four levels (terraces).
- The height displaces the sampled z: z' = z + flight - h. Grid lines are drawn where frac(z') or frac(u) is
  near 0: an anti-aliased core (half-width max(1 px, 1/64 cell)) with an 8-px halo; the line alpha falls to 0
  where a cell is under about 6 px / 6 lines, so the far field fades into the glow instead of aliasing.
  The rows' weight, softness and fade are scaled by 1 + 2 pi Depth Spread (1 .. 4), the wave's worst
  compression of the rows, so a row squeezed on a steep slope is still at least a line thick.
- Flight: the grid advances 8 cells per wave cycle, in the wave's direction.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | grid pitch: a cell is 60 .. 600 virtual px wide 512 rows below the horizon, exponential, glided (default 190) |
| K2 | Colour | the 16 shared palettes with hysteresis: ground; A = sky haze, sky tint, sun bottom; B = grid and floor glow; accent = sun top (default Outrun). Was K6 until v0.1.1; Shape (the cell aspect) is now fixed at squares |
| K3 | Horizon | horizon height 25 % .. 75 % of the screen, glided (default 56 %); the sun rides on it |
| K4 | Rate | flight and wave speed, bipolar, centre = still (+-8 deadband); 60 s per wave cycle .. 2 cycles per second |
| K5 | Spread | wave cycles per cell 0 .. 1/2, square law (default 1/6: hills six cells long) |
| K6 | Camera | camera (and Osc) strength 0 .. 100 %, default 50 %: gain K6 / 256 on the source field (0 % = a black input, 25 % = the v0.1.0 strength, 50 % = twice it, 100 % = four times), glided; see "K6 camera pass" |
| S7 | Wave | Smooth / Step (terraced hills: the rows break along fault lines). Step replaces the standard Snap: a dwell-and-flip wave only reads on discrete elements |
| S8 | Field | Linear (hills roll toward the viewer) / Radial (diamond ripples from a point on the floor) |
| S9 | Space | Sky (floor, horizon glow, sky) / Tunnel (floor and mirrored ceiling) |
| S10 | Sun | On / Off |
| S11 | Source | Video / Osc |
| P12 | TERRAIN | the headline: wave depth on a square law, 0 % a dead-flat grid .. 100 % 1.5 cells (the rows fold over into steep ranges); default 58 % = half a cell |

Depth is the headline, so it sits on P12 and K3 became Horizon (the slate's rule for this program).
Presets: Sunset Drive (defaults), Flatland, Tunnel, Ripple Pool, Terraces, Ranges.

## K6 camera pass (2026-11-15)

v0.1.1. Ron's set-wide rule: K6 = camera strength, 0 .. 100 %, default 50 %.

- **K6 Camera** (new): the gain on the source field (camera luma, or the Osc lattice, which keeps its 4x height gain
  over the camera). Glided on the shared glide's channel 1 (the one K2 Shape used; 1/8 of the gap per field, +-4
  code deadband); gain = (glide - 32) / 32 = about K6 / 4 - 1, 64 = 1, so the glide's resting band at 0 % gives
  exactly 0. One 8 x 8 multiply (s9_g = s9_l x gain / 64) in its own stage after the field's interpolation; the
  field then sits one pixel later (invisible on a 64-px box).
- **Law**: gain g = K6 / 256: 0 % = 0 (exactly the black-input picture), 25 % = 1 (the v0.1.0 strength), 50 % = 2,
  100 % = about 4.
  - Terrain lift: g x the v0.1.0 lift. Full white lifts 0.43 / 0.86 / 1.7 cells at 25 / 50 / 100 %; the Osc 1.7 /
    3.5 / 6.9 cells (the height acts modulo one cell, so a large lift folds the rows round the picture's contours).
  - Sky tint: linear up to 25 %. Above it the extra gain is centred on field value 96 (about a picture's mean):
    tint = (field x g - (g - 1) x 96) / 16, clamped 0 .. 15 (s_toff = 1.5 (gain - 64), per field). The sky gains
    contrast at the v0.1.0 mean instead of flooding with colour A: in the sim, a plain 2x turned half the sky flat
    pink and a plain 4x the whole sky.
- **Looks** (selfie.jpg, 720p decimation 2 with the horizon lowered to K3 = 57 so the quadrant holds sky, sun and
  floor: `.kinetic_work/rb_outrun/k6sheet.png`): 25 % the shipped soft pink camera haze in the sky; 50 % the camera
  clearly in the sky (pink highlights, dark shadows) and the near rows bending more; 100 % a hard two-tone camera
  picture in the sky with the near rows rippling, grid and sun intact; Osc at 50 % a bold pink lattice diamond
  over the horizon (its 16 tint steps show as soft contour bands) and mounds rolling the floor.
- **K2 Shape frozen** at its default 341 (constant C_KFIX_2; the aspect is Q8 -1, squares to 0.3 %), as assigned.
  On the sheet K5 Spread changes a still about as little as K2, but Spread sets the hills' wavelength (0 = no
  hills), so K2 stayed the choice. **K6's old function, Colour** (the 16 palettes, hysteresis, default Outrun),
  **moved to K2**. Presets: their Colour values moved to K2; their Shape values are gone (Flatland and Ripple
  Pool were at the default; Terraces 600 and Ranges 500 lose their longer lanes).
- **Build** (router1 through the K6 queue, 2026-11-15): all six configs on seed 1; HD Analog 80.54, HD HDMI
  83.04, HD Dual 83.67 MHz (worst HD 80.54); SD 86.61 / 86.66 / 80.33 MHz; 7,131 / 7,680 LC (92.9 %, +186 over
  v0.1.0), 13 / 32 block RAMs. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic outrun`.
- Unsure on hardware: the sky tint's centre (96) suits an average picture; a very dark camera at 100 % leaves the
  sky dark, a very bright one floods it. The height lift does not saturate (it wraps a cell), so at 100 % with a
  busy camera the near rows can fold into the picture's contours; read in a 720p / 2 still, never at full HD.

## Video

`program_type = "processing"`. Source = Video: the picture's luma (>> 2, less 16) goes through a 64-pixel
box and a vertical 1/8 IIR (half-resolution line store), so it bends the lines without shredding them, and
(at K6 = 25 %, the v0.1.0 strength; K6 Camera scales both, see "K6 camera pass" below)
- adds to the height in place: full white lifts the terrain 0.43 cell (same state as the wave);
- tints the sky toward colour A (up to 14/16), read 11 pixels ahead, so the picture shows in the sky.
With a black input both are zero and the wave alone runs. Source = Osc substitutes the shell's drifting
640 x 540 triangle lattice at four times the camera's height gain (up to 1.7 cells): diamond mounds drift
through the floor and diamonds of haze through the sky.

## Engine

Built on the riley chassis ([A] timing / virtual coordinates / generated avid, [B] controls, [D] source
field, [F] output conditioning are riley's; the box is 64 px instead of 32).

- Per field (vblank zone): glides (K1, K2, K3, K5, P12), eleven products on one 16-step serial multiplier
  (pitch and aspect through the 2^(i/256) ROM, spread, depth, rate, horizon, the Radial centre, the row
  widening), the wave phase, the Depth x magnitude table (64 entries), the palette.
- Per line: an engine kicked at every line start computes the NEXT line's constants in about 355 clocks:
  q = 2^26 / s on a 26-step divider, fourteen products on one 24-step serial multiplier (du, du per pixel,
  the lane start, z, cells per line and its widening, theta at the centre, the theta step and start, the
  sun's half-chord^2 and its three gradient colours), a serial normaliser that turns "cells per pixel / per line" into the
  two anti-aliasing gains (a shift and a 2-bit mantissa), line alphas and half-widths, the glow level and
  the stripe alpha. The sun's values are for the line in flight and are written before its first sun pixel.
- Per pixel (21 stages + blend): DDAs for the lane coordinate, the x part of theta (|.| of a signed DDA) and
  x^2 for the sun; theta -> quarter-wave sine ROM -> Depth table -> + source -> z'; for z' and u: distance
  to the nearest line -> barrel shift and mantissa -> core coverage / halo, clamped to the line alpha; the
  two add. Sky rows take the tint instead. The horizon glow is added to that alpha, checkerboard-dithered. The sun's
  disc (AA from R^2 - r^2, times the stripe alpha) wins where it is at least as strong. One 17-step shift-add
  blend ground + (target - ground) x alpha / 16, then output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 with
  the per-line resync, luma [1 2 1]/4 at the same group delay, luma 13/16 toward black) and the blanking gate.
- No per-pixel multiplier. One latency (36) in every mode; virtual 1080-line coordinates from the timing ID
  (SD shows the centre 4:3); generated avid; colours in the hardware convention (U register = Cr).

## Status

- Built 2026-11-14 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`); every
  config closed on seed 1. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic outrun`.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 86.98 MHz | 6942 |
| SD Analog | 1 | 86.93 MHz | 6945 |
| HD HDMI | 1 | 79.04 MHz | 6936 |
| SD HDMI | 1 | 87.40 MHz | 6933 |
| HD Dual | 1 | 84.96 MHz | 6939 |
| SD Dual | 1 | 84.86 MHz | 6938 |

- 6,945 / 7,680 LC (90.4 %), 13 / 32 block RAMs. HD needs 74.25 MHz, SD 27. No carry self-adds in the first build's netlist (dupin: none; not re-run on the final one).
- The first build, with router2 (the builder default), missed 74.25 MHz on HD Analog on seeds 1 .. 5 and was
  stopped; its critical path was the sun's stripe-alpha arithmetic in the line engine (three carry chains in
  one step). That step was split in three, the sun's disc compare in two and the barrel clip rewritten as bit
  tests before the router1 build; router2 was not retried on the final source.
- Model = RTL (GHDL, 720p60, decimation 1, defaults, black input, final source): pixel diff 0 of 918,720.
  An earlier revision also diffed 0 of 228,960 at decimation 2 with the horizon lowered (K3 = 57: sky, sun,
  glow and the near floor in one quadrant). The frame matches the model at exactly 2 phase steps, so the
  phase advances once per field as modelled. The GHDL frame sits 4 columns right of the model with 4 blank
  columns (the generated avid's start), as in riley.
- Osc (decimation 2, final source): 5.8 % of pixels differ from the model, 0.27 code mean in luma, 50 at
  most: the lattice's phase against the line start is idealised in the model (as in riley). Camera
  (selfie.jpg, decimation 2, final source): 6.1 % of pixels differ, 39 codes at most (the tester's image
  scaling and the source filter's phase); both renders show the same picture as the model.
- NTSC (GHDL, decimation 1, final source, both fields of 720 x 243): pixel diff 0 of 173,988 on each field.
  Same picture as HD (the centre 4:3: sun, horizon and grid in the same places). SD differences: the sun's
  stripes are 2 .. 8 virtual rows = 0.5 .. 1.8 field lines, so the upper ones are thinner than a line on
  480i, and the far rows of the grid fade out sooner (a field line is 4.4 virtual rows).
- Rasters rendered in GHDL: 720p (decimation 1 and 2) and NTSC. 1080i was started and stopped unfinished (too slow for the time left): NOT rendered.
- Renders: `.kinetic_work/rb_outrun/cmp_final.png`, `sheet.png` (control sheet, from the model), `strip.png`
  (8 frames across one wave period, from the model).

## Cut / deviations

- The lane correction was cut for fit: a real height also spreads the lanes (u' = u (1 - h / z)), which
  needs a 10 x 11 per-pixel multiplier; with it the first RTL was estimated at about 7,750 LC (standalone yosys, LUT + FF). The lanes are straight
  and only the rows roll; to keep the hills readable the across-screen term of theta was raised to the full
  Spread, so every row is a ridgeline.
- The horizon glow is an alpha added to the grid / sky colour (cyan below, pink above), not a per-line
  base colour (estimated at about 300 LC more).
- Radial is a diamond (|dz| + |u|), not a circle; its centre is fixed 200 rows below the horizon.
- Step replaces Snap (see S7).
- Anti-aliasing gains are per line, not per pixel: the rows are widened by the wave's worst compression
  instead (see above), so on flat stretches of a deep terrain they are fat and soft, and the far rows fade
  out earlier as P12 and Spread go up. Depth tops out at 1.5 cells, not more, for the same reason.
- Video lift does not scale with P12: with a camera, P12 = 0 shows the camera's relief alone.
- In Tunnel the sun is drawn over the ceiling grid; the sky tint is not shown (no sky).
- The source box is 64 real pixels at every resolution, so it is wider in picture terms on SD.
- K6's default is Outrun (palette 5 of 16), not Ink.

## Not verified / unsure on hardware

- Never flashed. Rendered rasters: see Status.
- On a dead-flat grid (P12 = 0, black input) the far rows are 2 lines thick with a 1-line soft edge each side:
  a row centred on a scan line is a full-bright line between two half-bright ones. That meets the set's
  2-row minimum but is the thinnest horizontal feature in the program; if it twinkles on hardware, raise the
  minimum half-width in f_hw from r to r + r / 4.
- The sun's per-line values are used in the line they are computed in (written within 103 clocks of the line
  start, first used 230 pixels in on SD). A source whose lines are much shorter than standard would break that.
- The horizon glow is dithered between adjacent pixels (a checkerboard, removed by the [1 2 1] luma filter);
  look for texture in the glow band on HD HDMI.
