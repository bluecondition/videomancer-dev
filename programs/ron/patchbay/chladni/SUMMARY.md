# Chladni: standing waves on a plate

**v0.1.1 built 2026-11-15 (K6 camera pass), not flashed.** Patchbay. Two H-locked and two V-locked sine VCOs, two multipliers and
a mixer make the field of a vibrating square plate; a window comparator at zero draws the sand along the nodal
lines and a colouriser bands the antinodes. Presets are hand-written placeholders.

## The patch

    H-locked sine VCO N (n half-cycles across the plate) --\
    V-locked sine VCO M (m half-cycles down the plate)  ---- multiplier 1 --\
    H-locked sine VCO M --------------------------------\                    mixer (cos / sin of the Mix angle) -> F
    V-locked sine VCO N --------------------------------- multiplier 2 --/
    camera luma -> slew (down, then across) ----------------------------------> + F     (S11: or the Osc lattice)
    tone generator (P12) -> both VCOs' frequency CV, and the comparator's window width
    F   -> window comparator at F = 0 -> the sand key (dark fringe, cream body, bright core), over everything
    |F| -> colouriser (one palette cycle per period of |F|; the lobe's sign picks the palette half)
    plate key (a bordered square, or the whole screen) -> black outside

    F = cos(th) sin(n X + a) sin(m Y) + sin(th) sin(m X) sin(n Y + d)        X, Y = pi x position / plate side

Every VCO is hard-locked at the plate's left / top edge, so integer modes put nodes on the plate's edges (the
classic figures) and a mode in mid-glide makes the lines travel in from the right and the bottom.

## What it draws

At the defaults: a 1000-px square plate, centred on black, with a thin brass rim. Modes (3, 5) mixed as the
minus figure: a cream sand cross on the diagonals with sand along the plate's edges, and twelve lobes between
them, each ringed by colouriser bands. Positive lobes run indigo / cyan fringe / magenta / yellow fringe /
orange; negative lobes run indigo / pink fringe / violet / aqua fringe / blue. The sand has a thin dark fringe at
the window's edge, so the lines read as drawn over the bands.

Motion at the defaults: VCO N's phase drifts, across at one rate and down at a third of it the other way, so
plate A slides sideways under plate B while B creeps vertically. The sand lines travel, reconnect and pass
through the plus and the minus figure in turn (about 19 s per horizontal cycle), and the colouriser's bands flow
out of the nodes at a third rate. Speed at its centre locks all three: the still classic figure.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Mode N | VCO N (frequency) | 1 .. 9 half-cycles, detented: flat for 32 codes either side of each integer, a glide between; knob glided 1/8 per field. 256 = 3 |
| K2 Mode M | VCO M (frequency) | 1 .. 9, as K1. 512 = 5. The two VCOs never sit on the same harmonic: M is pushed one away (F would vanish) |
| K3 Mix | mixer | an angle, constant power: 0 = plate B alone (-90 deg, an m x n grid of cells) .. 256-383 detent = the minus figure (-45 deg, the default 320) .. 640-767 detent = plate A alone (an n x m grid) .. 1023 = the plus figure (+45 deg) |
| K4 Speed | VCO N phase, colouriser offset | bipolar, square law, +-8 deadband: phase drift of VCO N across (full = 0.68 cycles/s) and down (a third, opposite), band flow (0.9 periods/s). In the deadband the drift phases snap to zero: the still figure. 660 = the default drift |
| K5 Bands | colouriser (gain) | 0.4 .. 3.4 palette cycles per unit of F: one or two wide bands per lobe .. tight rings (K6 until v0.1.0) |
| K6 Camera | source gain (camera or Osc) | 0 % = no source (the black-input figure) .. 25 % = v0.1.0's fixed gain (0.30 of F's unit for full white) .. 50 % (default) = 2x .. 100 % = 4x. Sand (K5 until v0.1.0) is fixed at its old default 512: the window still tracks the tone generator |
| S7 Plate | plate key | Square (bordered square plate, black outside) / Screen (the field fills the picture: n across the width, m down the height) |
| S8 Look | colouriser (palette bank) | Bands / Sand: pale sand on black, with a thin dust ring where each colouriser period ends (so Bands still counts rings) |
| S9 Twin | H VCOs (triangle sweep) | one plate / two plates mirrored about the screen centre (Square: two 900-px plates; Screen: two half-width fields) |
| S10 Colours | colouriser (setting) | Ember (indigo, magenta, orange / violet, blue) / Verdigris (green, jade, amber / red, plum, cream fringes) |
| S11 Source | F summing input | Video (slewed camera luma) / Osc (a drifting triangle lattice at 3.5x the camera's level) |
| P12 Resonance | tone generator | steps both modes together through -2 .. +7 (10 steps, +-12 codes of hysteresis; each step glides for about half a second and lands exactly). 0 % = modes (1, 3): the simplest cross. 25 % = the dialled modes. 100 % = (10, 12): a dense lace of nodes |

## Video

- **Video (the default).** Camera luma, (Y >> 2) - 16, is averaged over 8-pixel cells, slewed down the screen
  (one pole, 1/32 per line; the state lives in a block RAM, one word per cell), slewed across (two one-pole
  filters, 1/16 per pixel) and added to F at 0.30 of F's unit amplitude for full white. The nodal lines bend toward the dark
  side of every large shape in the picture and the bands swell over the bright areas; the slews keep every edge
  a clean curve. With a black input the patch gives its full picture.
- **Osc.** A 640 x 540 virtual-px triangle lattice, drifting about 75 px/s, takes the camera's place at 3.5x
  the level: a third standing wave under the plate, which pulls the cross apart and floods alternate lobes.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Knob glides (K1, K2, K3, K5, K6), the resonance
  step with its hysteresis and glide, the mode maps, the Mix angle and its cos / sin (from the V sine table),
  then 18 slots on ONE serial multiply-accumulate (acc = preload +- a x b, 16 iterations, slot number and
  operands registered ahead): the mixer gains, the sand window (and its msb and reciprocal for the pixel path),
  speed squared, the three drifts, the phase steps per pixel and per field line, the phases at the first pixel
  and at line 0, the source gain. Phases are 2^26 per cycle. Everything is started per field.
- **Per line (kicked at each line start, computes the NEXT line, 40 clocks).** Two phase DDAs down the screen,
  the 12-bit sine table for both V oscillators, and their products with the mixer gains on a second serial
  multiplier; the plate's row flags.
- **Per pixel (18 stages to the plate key).** Two phase DDAs across (Twin runs them back down from the screen
  centre), two 512-entry quarter-wave sine tables (block RAM, then a fabric register), the two multipliers
  (10 x 14 bits, operands split low / high, a stage each for the partial products, their sum, the signs and the
  mixer), + the source, |F| and sign, colouriser phase minus flow, the window distance normalised to 8 bits and
  scaled to t = 32 (1 - |F| / window), the colouriser address (inside the window the sand's cross-section,
  outside it the band), the 1024 x 32 colouriser ROM, the plate key.
- **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync (their 8-pixel delay
  lines are two 16-deep block-RAM rings), luma 13/16, a vertical [1 1]/2 mix with the line above (the line is
  stored as 8-bit pair averages: Y per 2 pixels, U and V per 4), and the blanking gate.
- **Contracts.** Generated avid; one latency (C_LAT 24) in every mode; virtual 1080-line coordinates; colours
  in the hardware convention (U = Cr, V = Cb); drawn whites about 777.
- **Block RAM (24).** Colouriser 8, three sine tables 6, luma cells 2, box delay rings 4, line store 4. All the
  RAMs that are written carry `no_rw_check` (no transparency emulation in the yosys log).

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now **Camera**, the strength of the source term added to F (the slewed camera luma, or the Osc
lattice when S11 = Osc), 0 .. 100 %, default 50 %.

- **Law.** The glided K6 (k, 0 .. 1023; deadband +-4 codes, ease 1/8 per field, shared glide channel 3)
  multiplies v0.1.0's source gain on the vblank sequencer's serial multiplier (a new slot 18 after the old
  source-gain slot 17): `c_gl = gain_v010 x k / 1024`, and the pixel path's existing camera multiply now keeps
  4x more of its product (`s_lterm = v_p(20 downto 4)`, was `(20 downto 6)`). So the term is linear in K6:
  - 0 % = no source at all (the black-input figure, bit for bit);
  - 25 % (k 256) = v0.1.0's fixed gain, 0.30 of F's unit amplitude for full white;
  - 50 % (default) = 2x: the sand cross bends well away from the diagonals, the plate's edge sand turns to bands;
  - 100 % = 4x (1.2 of F's unit for full white): bright areas lift F off zero, so the sand only survives over the
    darker parts of the picture and the bands follow the picture there; still a clean banded plate.
  No new pixel-path multiplier: the gain rides the existing 4 x 13-bit split multiply. |F| is 17 bits, so even
  Osc at 100 % does not overflow (the colouriser phase wraps, as it always did).
- **Osc** keeps its 3.5x level relative to the camera and scales with K6 too: at 50 % it floods every lobe
  (the sand mostly disappears, the lattice's own bands take over the plate), at 25 % it is v0.1.0's Osc.
- **K5 Sand frozen** at its default 512 (window `612` before the tone-generator tracking, `z_wk` is now a
  constant). Sand was the weakest knob on the sheet (hairline .. heavy line weight of the same figure).
  **K6's old function Bands moved to K5** (same law, 0.4 .. 3.4 palette cycles, default 400). Presets: the old
  K6 values moved to K5, the old K5 (Sand) values were dropped.
- **Sims** (`.patchbay_work/rb_chladni/k6/`, sim-only copy with the glide bypassed, NTSC full raster,
  selfie.jpg): `k6sheet.png` = K6 0 / 25 / 50 / 100 % and Osc at 50 %. Every step is a distinct, clean figure;
  no ragged edges.
- **Build (v0.1.1, router1, all six configs on seed 1).** HD Analog 77.65, SD Analog 89.35, HD HDMI 87.18,
  SD HDMI 94.92, HD Dual 93.72, SD Dual 94.51 MHz; 6,999 - 7,011 / 7,680 LC (91 %), 24 / 32 EBR. Worst HD 77.65
  (HD Analog); HD HDMI, the thin config of v0.1.0 at 75.27 (router2), is now 87.18.
  `out/rev_b/ron/patchbay/chladni.vmprog` (473,636 bytes).

## Status

- **v0.1.0 built 2026-11-14 with router2 (the build default), not flashed.**
  Rebuild: `./build_set.sh patchbay chladni` -> `out/rev_b/ron/patchbay/chladni.vmprog`.
- Post-route Fmax and seeds:

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 78.83 | 6,945 |
  | SD Analog | 1 | 83.31 | 6,942 |
  | HD HDMI | 1 | 75.27 | 6,939 |
  | SD HDMI | 1 | 66.67 | 6,941 |
  | HD Dual | 1 | 84.50 | 6,946 |
  | SD Dual | 1 | 79.44 | 6,948 |

  All six on seed 1. 6,939-6,948 / 7,680 LC (90 %), 24 / 32 EBR. No pin-skew seed screening.
  HD HDMI's margin is 1 MHz on this run; the build before the camera-slew change (the same netlist but for
  that filter) gave HD HDMI 81.34 on seed 1, HD Dual 81.59 on seed 2 after a seed-1 miss, and SD HDMI stalled
  router2 on seed 1 (seed 2: 83.09). If a rebuild misses, let the builder's seed retry run, or use router1.
- **Checks** (working files in `.patchbay_work/rb_chladni/`):
  - `check.sh` clean; standalone yosys: 24 SB_RAM40_4K, 0 transparency emulations.
  - GHDL 720p60, decimation 1, default knobs, black input: **bit-exact with the integer model `imodel.py`**
    (0 of 921,600 pixels differ; the tester's capture sits 4 px left of the generated avid, as in every shell
    program).
  - The same on the shipped sources at other rasters, all **0 differing pixels** against the model (rows 1 on;
    row 0 mixes with the previous field's last row): a second 720p capture nine fields later (the figure has
    moved, 88,270 px differ from the first, exactly as the model moves it), both 480i fields (720 x 243) and
    both 1080i fields (1920 x 540). So SD and 1080i draw the same picture as 720p / 1080p.
  - `cmp_final.png`: float prototype | integer model | RTL black input | RTL Osc | RTL with selfie.jpg. With
    the camera every edge stays a clean curve (the first camera filter left ragged teeth in the plate's
    corners; see notes.md, round 3).
  - `thincheck.py` on the model (720p / 1080p / 480i, 13 settings x 3 times, 117 frames): single-row lines of
    24 px or more in 2 frames only (5 runs, 24-96 px, about 11 % luma step, where a bright fringe row sits on a
    dark fringe row at the plate's top / bottom edge). Before the vertical mix there were dozens per frame.
  - `sheet.png`, `strip.png`: control sheet and 8-frame motion strip from the integer model.
- First RTL (alpha-blended sand, 32-bit phases) overflowed at 8,734 LC; see notes.md, round 2.
- **Watch on hardware:** the HD HDMI picture has crisp moving colour edges; the shell's conditioning is in, but if
  bars show, widen the chroma boxes first. The vertical mix reads a line store that is written two columns
  behind its read: collision-free by construction, but new in this shell family.

## Cut / deviations

- **Mode range.** The spec asked for modes 1 .. 12 per knob; the knobs give 1 .. 9 and P12 adds -2 .. +7, so the
  top is mode 16. Past that the cells are narrower than the output filter can keep apart at 720p and SD.
- **Mix is an angle**, -90 .. +45 degrees, instead of a plain + .. - crossfade: the sheet then has three
  different designed pictures (grid, minus figure, plus figure) and both pure plates are reachable.
- **P12 at 0 % is the simplest cross only at the default K1 / K2** (it is two steps below whatever is dialled).
- **Sand is a hard key** with a dark fringe, softened by the output filter, not an alpha blend: the first RTL
  (blended, 8,734 LC) did not fit. The vertical [1 1]/2 mix replaced the blend's anti-aliasing.
- **Speed's lock snaps** the drift phases to zero rather than easing them.
- **The camera's gain is K6** (v0.1.1; it was fixed in v0.1.0). Its slews trail it about 30 px to the right and about
  30 lines down, so the picture leans on the figure a little late and below itself.
- **Row 0** of each field is mixed with the last row of the field before (the line store is not cleared).
- **Resolutions.** 720p and 1080p show the same picture. SD shows the centre 4:3: the single square plate is
  the same; Screen fits its modes to the 4:3 width; **Twin's plates are 680 px on SD against 900 px on HD**
  (two 900-px squares do not fit 4:3). SD is softer (the filters are per output pixel) and thin colour fringes
  at the densest settings fall under a field line.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. **Bend / element**: K6 scales the source term added to F, so the camera bends the nodal figure and its
bands; nothing of the camera image itself is shown.

- **Change:** the per-field source gain is x1.25 at every K6 position: sequencer state 482 now loads
  `c_gl = g + (g >> 2)` with `g = gain_v010 x k / 1024` (was `g`). So 25 % = 1.25x v0.1.0, 50 % = 2.5x,
  100 % = 5x (was 4x). Widths checked: c_gl max 4918 -> 6147 (Osc, Bands at its top) < 8192 (13 bits); the pixel
  multiply's 21-bit sum stays under 2^21; |F| max about 106k < 2^17 (p9_f / p10_af do not wrap). No pixel-path change, latency unchanged.
  K6's default is 100 % (set by the orchestrator).
- **Look check** (NTSC full raster, bear.png, glide bypassed in scratch copies, K6 100 % old vs new;
  `.patchbay_work/rb_chladni/k6b/k6b_cmp.png`): the same plate figure, with the bands bent further by the picture. No wrap
  or new artifacts. (A 720p decimation-4 sim draws only the dark corner of the plate, so it isn't useful here.)
- **Build (router1, all six configs on seed 1):** HD Analog 99.37, SD Analog 102.33, HD HDMI 93.92, SD HDMI
  101.56, HD Dual 94.60, SD Dual 94.42 MHz (worst HD 93.92); 6,997 .. 7,014 / 7,680 LC, 24 / 32 EBR.
  `out/rev_b/ron/patchbay/chladni.vmprog`. Not flashed.
