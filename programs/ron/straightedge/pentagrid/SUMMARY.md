# Pentagrid (Straightedge)

**v0.1.0 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/pentagrid.vmprog`.

## What it draws

N. G. de Bruijn's pentagrid (1981, the construction behind the Penrose tilings), Francois Morellet's
superimposed grids ("4 doubles trames", 1958) and Islamic quasi-periodic star patterns. Three to eight families
of crisp straight parallel bands are fanned evenly through a half turn. Every family is a translucent film, so
the tone at a pixel is the NUMBER of families whose band covers it, in flat grey steps: five-fold stars,
decagons and pointed rhombi appear and never repeat. It fills the screen; nothing is centred except the one
point (X, Y) that every band axis passes through when the drift phases are zero. Luma only (U = V = 512).

    family k (k < N):  u_k = distance (pixels, modulo the period P) from the nearest band axis of
                             n_k . (x - X, y - Y) + P d_k,     n_k at angle (k - (N - 1) / 2) * Fan
    region  = |u_k| < T        Bold: the black gap between two bands, T = P / 4;  Fine: the white line, T = P / 12
    S       = the regions' summed coverage (each edge anti-aliased over one pixel from the distance itself)
    tone    = a table of S:  Grey / Bold  1 - S / N in flat steps (Order 6, 7, 8: the middle 5, 6, 5 levels)
                             Grey / Fine  every line a film "over" at 50 %: 1 - 2^-S
                             Ink          the parity of S (a triangle of the summed coverage)

The default frame is Order 5 at a period of 216 virtual px (0.2 screen heights), Fan 50 %: six flat greys,
stars about a third of the screen height across.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pan of the whole lattice, +-1024 virtual px; it is also the pivot the families fan about. Glided |
| K2 | Y | the same, +-576 virtual px. Glided |
| K3 | Scale | band period 54 .. 864 virtual px, exponential, 216 at the centre; the lattice scales about (X, Y). Glided |
| K4 | Order | N = 3 .. 8 families, stepped with hysteresis: 3 = triangles and hexagons, 4 = squares and octagon stars, 5 = the Penrose stars (default), 6 .. 8 = finer stars |
| K5 | Drift | every family streams along its own normal at its own rate (1, -3/4, 2/3, -1/2, 5/4, -7/8, 9/8, -5/8 of the speed), so the stars form, dissolve and re-form somewhere else. Hypnos SPEED: still within +-24 counts of the centre, then 2^-12 .. 2^-4 period per field either way. Default 680 (about 13 s per period) |
| K6 | Camera | how far a bright block of the picture narrows the gaps there (Fine: widens the lines); default 100 %. Glided |
| S7 | Glide | Off = the knob value of this field, no easing at all / On = 1/16 of the gap per field, never slower than one count, landing exactly; the Drift speed 1/8 of the gap (default On) |
| S8 | Step | Flat (two-level bands) / Stairs: each family is a four-level sawtooth staircase (a cliff, then three treads), so the sum is a terraced relief of stacked blocks. In Ink: the parity of the summed step index, a finer check |
| S9 | Tone | Grey (the count as flat steps) / Ink (pure black and white, the layers XOR) |
| S10 | Weight | Bold (equal bands) / Fine (lines of 1/6 of the period; in Grey each line is a 50 % film, so crossings step up) |
| S11 | Source | Video / Osc |
| P12 | Fan | the headline: the angle between neighbouring families. 0 % = all parallel: a barcode whose bars beat, because the periods spread (ratios 0.82 .. 1.23) and the drifts differ. 50 % = fanned through a half turn: the star lattice (default). 100 % = a full turn: a perfect lattice again with the families exchanged. Between: pointed harlequins and sheared quasi-lattices with long moire. Detents of 8 counts at 0, 50 and 100 % make the three pictures exact. Glided |

A knob starts to move its value only when it is two counts or more from where the value rests (pot jitter), or
sits at an end of its travel.

## Motion

Each family has a drift phase (20 bits = one period) that advances once per field (in the vblank zone, "no
avid for 1024 clocks") by speed x its rate; the phase wraps modulo exactly one period, so nothing ever jumps.
The speed runs 2^-12 .. 2^-4 period per field (the top is capped at 1/16 period per field: a quarter period per
field strobes on bands). Because the rates and directions differ this is not a pan: the pattern reshuffles
endlessly. Only Drift at the centre stops it. The motion is rigid: every band slides along its normal.

## Camera

The camera never bends a line. Luma -> 32-px box (16 px at 720p and SD) -> vertical one-pole of about half a
block of lines -> sampled once per block of a 34 x 34 virtual-px grid (57 x 32 blocks) into a two-bank block
RAM (8 bits; the scan writes one bank, the next field reads the other; the bank bit toggles per field). The
pixel path reads the pixel's own block, registers it, quantises it to FOUR levels (at 33, 47 and 62 % of
white) and holds that across the block; it sets the half width T of every family's region in the block:

- Bold: T = T0 (1 - 0.28 K6 level): bright blocks close the black gaps, down to 0.16 of their width at level 3,
  so the bands widen and the sum lightens;
- Fine: T = T0 (1 + 0.58 K6 level): bright blocks widen the lines up to nearly equal bands.

T is constant across a block and enters every edge function as a constant, so every edge stays straight; band
edges step parallel to themselves at block borders, and only where the level changes. Black input (or K6 = 0)
is exactly the generated picture.

- 25 %: gaps up to 21 % narrower in the brightest blocks: a visible lean.
- 50 %: up to 42 %.
- 100 % (default): the bright ground is thin lines on white, the dark subject stays full lattice: the subject
  reads as a posterised mass of the program's own bands. With bear.png the head, ears and body show as full
  lattice on a ground of thin lines. It reads best below Scale 50 % (several bands per block).

Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point, x 4 about its
middle (nearly two-level): stepped diamonds of full lattice on a ground of thin lines.

## Engine

- **Per field** (vblank zone): the glide of five knobs and the Drift speed (five clocks each), then a
  sequencer of (N + 1) x 16 slots of 24 clocks on ONE serial 16-step multiplier with one shared 28-bit
  add / subtract register: the period (2^(i/256) ROM, interpolated to the Q6 glide so a zoom is smooth far from
  the pan point), its reciprocal, the region width and its camera step, the fan step, the pan point in pixel
  units; then per family its angle, sine and cosine (1024-point quarter-wave ROM interpolated to 18-bit
  angles), the period spread, the pixel step, the line step, the field seed and the line wrap, both reduced
  modulo the period exactly (quotient estimate from the reciprocal, exact remainder, two conditional +- P),
  and the drift phase. At most 3,456 zone clocks at Order 8. A 512-entry tone table is refilled in parallel.
- **Per line**: no multiplier. A loader puts each family's line wrap through its DDA from a small table.
- **Per pixel** (9 stages to the luma, no multiplier, no divider): eight 25-bit DDAs in PIXEL units (Q14) that
  wrap modulo the period themselves (the step, or the step less a period on the clock after the sum reaches
  0), so the anti-aliasing is a clamp of the distance; two edges per family (T/2 and 3T/2 for Stairs, T for
  Flat), coverage 0 .. 16 each; an adder tree; the tone table (one block RAM pair).
- **Output**: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the
  shell's parity-locked start), one latency (20) in every mode, virtual 1080-line coordinates from the timing
  ID. Interlaced modes soften the edges of steep families over one field line.

Chassis: Kinetic riley's [A] timing / zone / width / generated avid, the Hardline / Vellum glide datapath (the
+-1 hold is a per-channel "moving" flag here), Hardline dazzle's sequencer pattern, DDA loader and Osc lattice
(with its first-line fix), the [D] box and one-pole (8-bit samples, shorter time constants), the luma half of
[F]. No wave engine, palettes or chroma paths.

## Status

- v0.1.0 built 2026-11-17 through the shared queue, **router1**, **every config on seed 1**. Not flashed.
- Post-route Fmax: HD Analog 91.77 MHz, HD HDMI 92.25, HD Dual 88.98 (HD needs 74.25); SD Analog 94.92,
  SD HDMI 92.87, SD Dual 89.25 (SD needs 27).
- 6,940 .. 6,948 / 7,680 LC (90.5 %), 20 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin: 0).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge pentagrid`. (router2 on the same
  netlist gave 72.4 MHz on seed 1 and 81.2 on seed 3 for HD Analog: congestion-bound; router1 routes it at 92.)
- The first RTL was 8,346 LC (108 %). Cuts, with the same arithmetic: one shared add / subtract register for
  the sequencer's sums, the DDA wrap taken one step late (one adder instead of two), a "moving" flag in place of the glide
  holds, per-mode constants as functions, a few holding registers removed.
- **Model = RTL** (GHDL, the built sources): pixel diff 0 of 918,720 on a full 720p60 frame at the defaults
  with a black input, 0 with Source = Osc, 0 with bear.png as the camera at K6 100 %; 0 of 173,988 on each field
  of an NTSC frame (720 x 243) with a black input and on one field with bear.png; 0 on 1080p-timing (decimation 4, bear.png, Stairs:
  the 32-px box path) and on both fields of 1080i-timing renders; 0 on 720p quarter-rasters with every switch
  and other knob values (X, Y, Scale 0 / 20 / 30 / 80 / 100 %, Order 3, 4, 6, 7, 8, Fan 0 / 20 / 60 / 70 / 100 %,
  Stairs, Ink, Fine and their combinations, Osc, K6 25 and 60 % with bear.png).
  Motion: at the default Drift, captures 2 and 8 fields in match the model at 2 and 8 phase steps (0 diff), and
  Drift 10 % (fast, reversed) matches after 4. Glide On: at the power-up defaults 0 diff; a glide in flight
  (X, Scale, Fan and the Drift speed all moving) matches the model's third easing step, 0 diff.
  The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the shell's
  generated-avid start).
- NTSC shows the same picture (the centre 4:3 of the 16:9 one).
- Renders: `.straightedge_work/rb_pentagrid/cmp_final.png`, `sheet.png`, `strip.png`, `motion.png`,
  `camera.png` (all from the integer model `imodel.py` except the RTL panels of cmp_final).

## Cut / deviations

- **Camera: four posterised levels, not a continuous duty.** A continuous duty per block made every band edge
  a ragged saw tooth wherever the luma has a gradient (busy, torn-looking). Four levels with a steep curve
  keep the lattice exact inside large regions and step the edges only along the posterised outline. Spec's
  target (band duty per block) is otherwise kept.
- **Stairs is a sawtooth built from two symmetric regions**, value 3, 2, 1, 0 across the period with a cliff:
  a symmetric (ziggurat) staircase read as soft blobs in the renders.
- **Grey / Bold clips the count at Order 6, 7, 8** (levels 1 .. 5, 1 .. 6, 2 .. 6) so the picture keeps full
  contrast; at those Orders the thin gap lines of a bright camera region are white.
- **The period spread exists only below Fan 50 %** (it makes Fan 0 % a barcode instead of a plain stripe);
  from 50 % up every period is equal, so the lattices are exact.
- **Drift top speed 2^-4 period per field**, not 2^-2.
- The band axes pass through (X, Y) at half a period from each region's centre, so at power-up (all drift
  phases zero) the point (X, Y) is a white decagon in Bold and a black one in Fine.
- X / Y reach +-1024 / +-576 virtual px.
- Per-field phase advance is on the vblank-zone pulse (as every Hardline / Vellum program), not on a vsync edge.

## Not sure of on hardware

- Nothing on this chassis has been flashed yet. The generated avid, zone and width logic are the heirloom
  shell's, which is hardware-validated.
- The camera level of a block can flicker between two levels when its filtered luma sits on a threshold
  (the filter averages about 500 pixels, so only blocks within a fraction of a code of a threshold would):
  if it shows, a level hysteresis needs a third read of the grid.
- All sharp, full-swing luma edges in motion: the [1 2 1] + 13/16 conditioning is built in; if HD HDMI bars
  show, Scale at minimum with Ink is the first place to look.
- The Order knob's hysteresis band (8 counts) and the two-count hold are untested against real pot noise.
