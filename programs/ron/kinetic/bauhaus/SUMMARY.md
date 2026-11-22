# Bauhaus (Kinetic)

v0.1.1 (K6 camera pass) built 2026-11-15, all six configs, not flashed. Flash `out/rev_b/ron/kinetic/bauhaus.vmprog`.
(v0.1.0 built 2026-11-14; its build notes are kept below.)

## What it draws

A grid of shape tiles, after Bauhaus geometric posters and the generative "shape tile" grids that descend from
them: every cell holds one primitive in two flat palette colours (a cell colour and a figure colour). At the
defaults (Bauhaus palette, about 14 x 8 cells, full-bleed) it is a cream / red / blue / yellow poster of quarter
discs, quarter targets, half discs, discs, donuts, dots, triangles, half rectangles, bars, diamonds and squares.
A diagonal wave travels through the grid: where it passes, a figure shrinks to nothing, is swapped for another
(new motif, orientation and colours) and grows back, so the poster re-rolls in bands, never with a pop.

Figures (all = one anchor in the figure box + one radius):
- curved: quarter disc (4 corners), quarter target (ring + core, 4 corners), half disc (4 edges), and centred
  disc / donut / dot / target;
- straight: diagonal half (4 corners), half rectangle (4 edges), bar (across / down, thin / thick), and centred
  diamond / square.
Half the cells take the palette's ground as their cell colour, the rest A, B or the accent; the figure is always
a different colour.

## Element and state

- Element: a cell of pitch P; a cell boundary sits on the screen centre, so Scale zooms about the centre.
- State: its generation, a fixed-point number (10 fraction bits).
- theta = Spread x index - phase. Linear: index = column + row (a diagonal sweep). Radial: index = max + min / 2
  of (|column|, |row|) (an octagonal ripple from the centre). Layout = Mirrored uses |column|, |row| for the
  wave as well, so the motion stays four-fold symmetric (Linear then gives diamonds from the centre).
- state = 1/2 + Shuffle x wave(theta); wave = sine, or Snap (the first sixteenth of the sine stretched over the
  quarter cycle: 75 % dwell, quick eased flips).
- hash(column, row, floor(state)) picks curved / straight (against Mix), the motif, its orientation and the two
  colours. The hash is four add-and-xorshift rounds on 18 bits; its output bits were checked on a 64 x 64 x 16
  grid: bit means 0.499 .. 0.503, worst bit-pair correlation between neighbours 0.034 (rows), 0.010 (columns),
  0.012 (diagonals), 0.011 (successive generations); equal neighbouring motifs 6.5 % / 6.2 % (ideal 6.25 %).
- Figure size = min(1, 4 x distance of frac(state) from the re-roll): full for the middle half of a generation,
  shrinking to nothing at the swap.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 38 .. 332 virtual lines, exponential, glided (default 135: 14 x 8 cells) |
| K2 | Shape | figure inset: 0 % = full-bleed (default) .. 100 % = figures at 40 % of the cell with margins of the cell colour |
| K3 | Mix | share of straight motifs, 0 (all curved) .. 16 sixteenths (all straight); default 5/16 |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes with hysteresis (ground, A, B, accent: all four are used as cell and figure colours). Was K6 until v0.1.1 |
| K6 | Camera | camera / Osc strength, default 50 % (v0.1.1, see "K6 camera pass"). 0 % = the poster alone, 25 % = the v0.1.0 halftone, 50 % / 100 % = its contrast x 2 / x 4 |
| S7 | Wave | Smooth / Snap (the grid dwells, then re-rolls in bursts) |
| S8 | Field | Linear (diagonal sweep) / Radial (octagonal ripple from the centre) |
| S9 | Gutters | Off / On: ground-colour lines between the cells, 4 px + 1/8 of the cell wide |
| S10 | Layout | Random / Mirrored (four-fold mirrored composition, figures and wave) |
| S11 | Source | Video / Osc |
| P12 | SHUFFLE | the headline: how many generations the wave spans, square law, glided. 0 % = a still poster whose figures only breathe (+-0.44 generation, never a re-roll) .. 32 % (default) = each cell re-rolls about twice per half cycle .. 100 % = +-4.4 generations, the whole grid re-rolling in waves |

Depth is the headline, so it sits on P12 and K3 became Mix (the spec's rule for this program).
Presets: Poster (defaults), Mirror Quilt, Flip Board, Slot Machine, Shape Halftone, Diamond Drift.

## Video

`program_type = "processing"`. The picture's luma picks the motif by coverage, so the picture shows as a
halftone of shapes:
- luma (>> 2, less 16) is smeared along x (1/16 one-pole) and HELD per cell: one sample per cell per line at
  mid-cell runs through a 1/8 filter down the lines in a block-RAM word per cell column, and a cell row uses
  the filter value of the line before the row (the field's first line: the smear at the cell start). Every
  cell is rigid: one level per cell, no per-line edges.
- level = held x 5 / 64 (0 .. 15) at K6 Camera = 25 % (v0.1.1: see "K6 camera pass"). Level 0 (black, under about 6 % luma) leaves the hash poster untouched, so
  with a black input or nothing plugged in the wave alone runs. Levels 1 .. 13 replace the cell with a disc
  (or a square, by Mix) on the ground colour that grows with the level: dot at the centre (3 sizes) -> quarter
  at a corner (3 sizes) -> half at an edge (3 sizes) -> full at the centre (3 sizes) -> the whole box. The
  orientation and the figure colour (A, B or accent) still come from the hash, and the figure still shrinks
  and swaps with the wave.
- On the light-ground palettes (Paper, Bauhaus, Candy, Mint) the level is inverted, so bright picture areas
  always show more of the lighter colour.
- Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at full swing at the same point:
  diamonds of graded dots / quarters / halves / full tiles drift across the poster.
The picture does not colour the cells (no Face switch here: both own switches are spent on Gutters and Layout).

## Engine

Chassis from louver.vhd ([A] timing / zone / glides / sequencer, [B] generated avid, [F] output conditioning);
[C], [D], [E] are this program's.

- Per field (vblank zone, "no avid for 1024 clocks"): glides (K1, P12), one 24-step serial multiplier in 10
  slots (pitch exponent, spread^2, cells per line, first pixel, first line, spread x first column / row,
  shuffle^2, rate), 2^-e and the pitch through the 2^(i/256) ROM, the wave phase step, and the blend table
  T(colour a, colour b, coverage 0 .. 8) = 144 words refilled by adders straight from the palette ROM.
- Per line: the row accumulator (cells, Q20), Spread x row by one adder at each row crossing, the three |dy|
  anchor distances, the box and gutter y distances, the row's phase terms.
- Per pixel (35 stages): column accumulator -> Spread x |column| -> Linear / Radial combine -> quarter-wave
  sine ROM -> Shuffle x wave (gated sums) -> state -> hash (4 stages) and size x box -> motif decode -> |dx|,
  |dy| -> two square ROMs + an R^2 / log2 R ROM -> edge function (disc, ring, core, diamond, rectangle) ->
  x (8 - k) / 8 and a two-step shift by the scale exponent (quarter octaves: cell px / 2R) -> coverage in
  eighths -> clipped to the figure box, gutter coverage (two small gated-sum multiplies by the cell size) ->
  blend table -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4
  delayed to the same group delay, luma 13/16 toward black) -> blanking gate.
- One latency in every mode: C_LAT = 42 = processing_delay_clks. Generated avid (parity-locked start), virtual
  1080-line coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention
  (U register = Cr); chroma is carried at 8 bits up to the output filter.
- No per-pixel multiplier wider than 8 x 10 bits, no square root, no divider; every block RAM has no_rw_check
  (0 emulate_transparency in a standalone yosys run of the first RTL; the RAMs did not change after it).

## Status

**Shipped vmprog: the ROUTER1 rebuild of 2026-11-14** (same sources). Post-route Fmax: HD Analog 75.39 MHz; SD Analog 81.93 MHz; HD HDMI 80.37 MHz; SD HDMI 78.77 MHz; HD Dual 76.16 MHz; SD Dual 72.86 MHz. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic bauhaus`. The router2 figures below are the first build, kept for the record; it was timing-marginal.


- Built 2026-11-14 through the shared queue with router2 (the builder default); rebuild with
  `./build_set.sh kinetic bauhaus`. Never flashed.

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | **4** (seed 1 stalled the router, 2 and 3 missed) | 78.17 MHz | 6722 |
| SD Analog | 1 | 75.84 MHz | 6723 |
| HD HDMI | **3** (1 and 2 missed) | 74.34 MHz | 6728 |
| SD HDMI | 1 | 61.90 MHz | 6724 |
| HD Dual | **3** (1 and 2 missed) | 80.43 MHz | 6731 |
| SD Dual | **2** (seed 1 stalled the router) | 70.36 MHz | 6718 |

- HD needs 74.25 MHz, SD 27. 6,731 LC at most (88 %), 23 of 32 block RAMs.
- **Timing is seed-marginal on HD**: HD HDMI passes by 0.09 MHz. This is the second build. The first build
  (sources and vmprog kept in `.kinetic_work/rb_bauhaus/build1_backup/`) closed HD Analog 78.04 (seed 1),
  HD HDMI 81.11 (seed 1), HD Dual 76.98 (seed 2) at 6,701 LC; the only RTL difference is the rectangle
  coverage rule in stage 31 (four compares after the 4-way shift instead of two) plus one ROM word, added to
  remove a one-row seam (see below). If a rebuild misses, the fix is to register the stage-31 shift first and
  do the clamp in a stage of its own (latency + 1), or try router1; the critical path was not profiled.
- The two router "stalls" were ended by hand, not by the builder's 900 s timeout: the seed-1 router sat at
  `overused=2` (HD Analog, 7 min) and `overused=4` (SD Dual, 3 min; a normal run takes under 2), so its make
  was killed and the builder went on to the next seed.
- GHDL render = integer model, black input, pixel diff 0, on the RTL as built:
  - 720p60 640 x 360 window, defaults: 0 of 228,960;
  - 720p60 640 x 360 window, Mix 900, Shape 200, Gutters On: 0 of 228,960.
- The same, on the first build's RTL (before the rectangle rule changed; the model was changed with it):
  - 720p60 full frame (1280 x 720), defaults: 0 of 918,720;
  - 720p60 640 x 360 window, Snap + Radial + Gutters + Mirrored, Shape 400, Mix 800, Spread 600, Rate 900,
    Vega: 0 of 228,960;
  - NTSC (480i, 720 x 243, both fields), a sim-only copy with the glides bypassed, Scale 300, Shuffle 800,
    Shape 300, Gutters On, Rate at centre: 0 of 173,988 on each field.
  The full 720p frame, NTSC and the Snap / Radial / Mirrored combination were NOT re-rendered after the change.
- The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the generated avid
  starts 4 clocks after the source's); the geometry is offset to match, so the grid's centre sits on the
  screen centre.
- Source = Osc and Video = selfie.jpg were rendered in GHDL (640 x 360 window): the same picture as the model
  with some cells one halftone level apart (7 % / 9 % of pixels differ on the first build's renders): the
  model does not reproduce the hold's exact sample phase (pipeline offset of the smear, the lattice's drift
  count).
- Rasters rendered in GHDL: 720p60 and NTSC 480i. No 1080i and no 1080p frame was rendered (time). The integer
  model draws the same picture at 1080p, 720p, 480p and NTSC (the centre 4:3 on SD): `modes_model.png`.
- Scale and Shuffle were simulated at two values each (defaults; 300 / 800 in the glide-bypassed copy); the
  glide itself is louver's code and was not exercised in a sim.
- Single-row check on the model (1-row-tall hard runs, 5 phases each): defaults 17 px, Gutters 4, Shape 600 2,
  Scale 0 % 11, Scale 100 % 11, Shuffle 100 % 15, Mirrored + Radial 22, Mix 100 % 43 (see the deviations).
- Renders: `.kinetic_work/rb_bauhaus/cmp_final.png`, `sheet.png` (control sheet, model), `strip.png` (motion, model).

## Cut / deviations

- Radial uses the octagonal cell metric max + min / 2, not a true distance.
- The concentric quarter rings are one ring (R/2 .. R) plus a core (R/4), radii in powers of two, not three
  equal bands.
- The "bar" shrinks along its length as well as its width while it re-rolls (so it can never become a
  1-px-tall full-width line).
- Source: level 0 keeps the hash poster instead of drawing the smallest dot, so black picture areas show the
  poster (needed so a black input is not a dead field of dots). Source-driven changes of a cell's level are
  immediate (no shrink-swap-grow, no hysteresis): a cell sitting on a level threshold can flip with camera
  noise. The held luma is taken just above each cell row (about 8 lines of filter), not at the cell centre.
- The halftone is coarse at the default Scale (14 x 8 cells); the Shape Halftone preset uses small cells.
- Edge softness is quantised: the coverage scale is rounded to quarter octaves (ramp width within about +-9 %
  of one pixel), and the box / gutter edges use the x pixel size, so on SD (non-square virtual pixels) the
  horizontal edges are slightly softer or harder than the vertical ones.
- Only Scale and Shuffle are glided; Shape, Mix and Spread read the knob directly.
- Cell edges with Shape = 0 and Gutters Off are hard (pixel-snapped), not anti-aliased: the figure box clip
  and the rectangles' own edges ramp on the OUTSIDE (full coverage at the edge), which at full-bleed belongs
  to the next cell. Ramping inside left a one-row lighter seam where a full-size bar or half rectangle met a
  neighbour of its own colour (42-px runs at the defaults); that is what the second build fixed. Rectangles
  are therefore half a pixel larger per side than their nominal size.
- Known thin features that remain: (a) while a rectangle shrinks or grows, its end passes within a pixel of
  the cell edge for a frame or two, a one-row gap up to half a cell wide (the 43-px run at Mix 100 %);
  (b) where a disc is tangent to its cell edge and the neighbour has the disc's colour, the wedge between
  them is one row tall over about 2 sqrt(2R) px (17 px at the defaults, more at large Scale).
- The picture's 8 filter columns at the left / right edge blend into the ground colour (louver's chassis).

## K6 camera pass (2026-11-15)

v0.1.1. Ron: K6 is the camera strength in every program, default 50 %.

- **K6 Camera** (0 .. 100 %, default 512) sets how strongly the picture (or the Osc lattice, which goes through
  the same path) drives the shape halftone. The held cell luma `held` (0 .. 239) becomes the level by
  `level = (held x g - off) / 256`, clamped to 0 .. 15, with `g = 5 k / 64` (k = the glided knob, 0 .. 79):
  - 0 .. 25 %: a plain gain (off = 0). 0 % = level 0 everywhere = exactly the black-input poster; the halftone
    fades in from the brightest cells; **25 % (k 256, g 20) = v0.1.0 exactly** (held x 5 / 64).
  - past 25 %: a contrast about the middle level (level 7 at held 90, about 41 % luma): off = (g - 20) x 90, and
    every cell that v0.1.0 would draw (held >= 13) keeps level 1 or more, so the shadows stay halftone and do
    not fall back to the poster. **50 % = contrast x 2** (the default), **100 % = x 4**: darker cells go to the
    full box, brighter ones to the dot (on the light palettes, inverted as before), a near two-tone halftone.
  - A plain 2 x gain was tried first and rejected: on the default (light) palette it pushed every mid-grey cell
    to the smallest dot, washing the frame out to the cream ground (selfie render). The contrast law keeps
    both ends.
- K6 is decoded per field in the vblank zone (states 40 .. 48: glide 1/8 of the gap per field with a +-4
  deadband, the gain, the offset by shift-adds). The pixel side is `held x g` as gated sums over three
  stages, the offset subtract and the level clamp: five single-op stages running while `k_held` is steady
  (it is latched at the cell start and read 10 clocks later), so no latency change (still 42).
- **Frozen**: K5 Spread at 354 (its default: a phase step of 0.06 cycle per cell, square law). It was the weakest
  knob: in a still it only re-phases which cells re-roll (sheet.png K5 row); it mainly shapes the motion.
- **K6's old function, Colour (all 16 palettes), moved to K5.** Presets: their Colour values moved to K5;
  Mirror Quilt (Spread 420) and Slot Machine (Spread 150) lose their own Spread.
- Renders (GHDL, 720p60 at decimation 2, selfie.jpg, glide-bypassed copy): `.kinetic_work/rb_bauhaus/k6sheet.png`
  (K6 0 / 256 / 512 / 1023 and Osc at 512). K6 0 = the poster; 256 = the v0.1.0 halftone; 512 and 1023 = more
  full boxes and small dots.
- **Build v0.1.1** (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic bauhaus`): **router1, every config on
  seed 1**. Post-route Fmax: HD Analog 80.10 MHz, HD HDMI 83.04, HD Dual 80.58 (worst HD 80.10, up from
  v0.1.0's 75.39); SD Analog 80.85, SD HDMI 75.45, SD Dual 75.00. 7,058 .. 7,075 / 7,680 LC (92 %, v0.1.0
  6,731), 23 / 32 EBR (unchanged). Log `.kinetic_work/rb_bauhaus/build_v0.1.1.log`.


## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023, set with the version bump), and on bend / element programs the camera is 25 % stronger at the top of the knob (here x 1.25 at every position, 0 stays 0).

- **Element-type**: K6 sizes each cell's figure by the held camera luma (the shape halftone), so it got the +25 %.
- Change (per-field decode, state 44): the gain c_cg = 25 k / 256 (was 5 k / 64), 0 .. 99 (was 0 .. 79). The contrast
  knee now follows the gain (c_cg >= 20, about 20 % of the knob instead of 25 %), so the law stays continuous and still
  pivots on level 7 (held 90); 100 % = contrast x 5 (was x 4), 50 % = x 2.5. Widths checked: c_off <= 79 x 90 = 7,110
  (13 bits), held x c_cg <= 23,661 (15 bits), the level clamp unchanged. Pixel path and latency unchanged.
- Look: `.kinetic_work/rb_bauhaus/k6b/k6b_cmp.png` (GHDL, bear, 720p60 / 4, K1 = 0 for more cells, K6 glide snapped in
  scratch copies): the same poster; some mid-grey cells move one step further toward the full box or the dot.
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic bauhaus`): **router1, every config on seed 1**. Post-route Fmax HD Analog 75.25, SD Analog 66.92, HD HDMI 84.74, SD HDMI 76.73, HD Dual 79.54, SD Dual 76.52 MHz (worst HD 75.25); 7,080 / 7,680 LC (92.2 %). Not flashed.

## v0.1.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`N - x` as `x - N`) while the GHDL
simulator computes it correctly, so every render showed the intended picture and the hardware did not (CLAUDE.md
VHDL traps; audit in `.heirloom_work/ghdl_sub_check/`). Rewritten as `to_signed(N, w) - x` / `-x + N`:

- **line 531** (none): Integer cell index of the first pixel, negated (= +ceil of the half-width in cells), as the
  multiplier operand for Spread x first column (slot 5). On hardware it was: v_i = +floor(c_x0). Because c_x0 itself
  is also reversed (line 639: +X instead of -X), this gives the first column of the HW's shifted grid, and line
  641's second reversal makes c_si0 = +spread*first column. Net: consistent with the shifted origin, no visible
  effect of its own.
- **line 534** (none): Negated integer row of the first line, operand for Spread x first row (slot 6). On hardware
  it was: Same as 531 for rows: cancels with 642 and tracks the shifted origin from 640. No visible effect of its
  own.
- **line 639** (major): First pixel's x in cells = -(c_du x (W/2 - 4)), so x = 0 (a cell boundary) sits on the
  screen centre: Scale zooms about the centre, Mirrored layout mirrors about the centre column, Radial ripples from
  the centre. On hardware it was: c_x0 = +X, so the column accumulator runs X .. ~3X: the cell origin lands about
  half a screen to the LEFT of the left edge (and, with 640, half a screen above the top). On screen: K1 Scale zooms
  about that off-screen top-left point (the grid flows toward/away from the upper-left instead of breathing about
  the centre); …
- **line 640** (major): First line's y in cells = -(c_dv x half height): row 0 on the screen centre. On hardware it
  was: c_y0 = +Y: rows run Y .. 3Y, the origin sits about half a screen above the top edge. Vertical half of the 639
  problem (zoom pivot, Mirrored has no top/bottom symmetry, Radial centre off-screen).
- **line 641** (none): Spread x first column = -(c_s x v_i), seeds the per-line Spread-x-column accumulator q2_si.
  On hardware it was: c_si0 = +(c_s x v_i); with v_i also reversed (531) this equals spread x the HW's (positive)
  first column, so the wave term matches the shifted grid. No visible effect of its own.
- **line 642** (none): Spread x first row, seeds the row term. On hardware it was: Cancels with 534 (see 641). No
  visible effect of its own.
- **line 891** (none): Spread x |row| for Mirrored / Radial: negate the (negative) spread term on rows above the
  centre. On hardware it was: Would leave the term negative (no abs) on negative rows. On HW today rows are never
  negative (origin moved above the top by 640), so this branch is never taken: no visible effect now. LATENT: if
  639/640 are fixed without this, Radial and Mirrored break on the top half (wave phase wrong above the centre).
- **line 949** (none): Spread x |column| for Mirrored / Radial (negate on columns left of the centre). On hardware
  it was: Only reached on HW at the smallest Scale where columns wrap past 63; there q3_t is already positive (q2_si
  does not wrap), so the missing negation happens to keep the wave continuous. Otherwise never taken: no visible
  effect now. LATENT: once 639 is fixed, Radial/Mirrored break on the left half.
- **line 1190** (visible): Outside-the-figure-box distance |v_fb| (1..127, 1/1024 cell) for the box clip s_cb = 8 -
  ob*kq/64: a one-pixel anti-aliased edge where a figure is clipped by the box. On hardware it was: q22_ob = 128 -
  |v_fb|: just outside the box the clip jumps straight to 0 (no AA), then about 1/8 cell (127/1024) outside the box
  the coverage ramps back up to full for ~1 px and cuts hard. Wherever a figure overflows its box (quarter discs,
  half discs, bars...), a thin ghost echo line of the figure appears in the …

**Build** v0.1.3, router1: HD Analog 78.46, SD Analog 72.87, HD HDMI 76.17 (builder seed, replaced: see HD skew), SD HDMI 81.11, HD Dual 76.16, SD Dual
74.83. 7243-7258 / 7680 LC, 23 / 32 EBR; all six seed 1. iceunpack OK. Not simulated, not flashed.
**HD skew**: NO WINDOW SEED (this netlist places HSYNC low) in r1 s1-200 + r2 s1-100 (router2 deterministic on re-route, misses timing): HD
HDMI router1 s90 (+1.74 / <= -0.42, 74.42 MHz; data 0.18 short, thin timing, cf. tartan); spare router1 s183 (+0.86 /
<= -0.98, 81.47 MHz). A full builder rebuild re-routes seed 1: re-install the seeds with
`.heirloom_work/skew_tools/install.sh` afterwards. If HD HDMI shows thin flashing lines or noise, try the spare
(sub-window HSYNC but clean data and timing), then report which one is clean.
