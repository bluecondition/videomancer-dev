# Confetti — magenta petal cards sliced by slanted cuts over a striped fan and a red zone

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom phase 3 (video clips), from the clip `WIN_20231123_23_27_50`
at 5.5 s (`photos/vid/WIN_20231123_23_27_50_t5p5.png`; 10.7 s clip, the same patch in `23_29_46`). Three columns
of tall rounded petals, each wider than the last, are sliced by slanted cuts into big magenta cards, with thin
slivers above and triangles below; a big magenta mass fills the top-right corner. Behind them orange / teal
stripes fan out from the top-left (arcs on the left, fine diagonals on the right) and a red zone with navy
horizontal stripes and lens-shaped ends lies across the top. A card shows the ground through it: magenta over
orange or red, violet over teal or navy. In the clip the petal oscillator scrolls down under the fixed cuts, so the
cards swell, shrink to slivers and come back every ~5.2 s. Presets are hand-written.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_confetti/` (`notes.md` has every number).
- **Palette** (k = 14): magenta (250, 0, 254) 24.5 %, (207, 0, 254) 15 %, violet (154, 1, 253) 8.3 %, red
  (150, 7, 2) 8.0 %, brown (85, 29, 5) 8.5 %, teal (47, 96, 139) / (22, 65, 97) 10.5 %, orange (127, 67, 27) /
  (204, 94, 21) 7.6 %, grey (65, 63, 60) 6.4 %. Every card colour fits (1.3 R_under + 150, 0, 254): a keyed colour
  offset on whatever is under the card.
- **Columns.** Outer edges x 76-560, 582-1114, 1150-1836 and a 4th from 1890 (widths 484 / 532 / 686). The centres
  fit a linearly FM'd H oscillator u = 0.5 + 0.0022069 (x - 315) - 2.816e-7 (x^2 - 315^2) (cell boundaries predicted
  570 / 1160, gap centres measured 569 / 1131). Resampled into cell coordinates h = frac(u), the three columns are
  the same picture (`cellmap.png`).
- **Cards.** Fitted by label purity on the card mask in cell coordinates (200 x 540 grid, `fitcard*.py`): one
  comparator on ellipse + slanted triangle gives lens-shaped cards (11.7 % wrong); ellipse AND an enveloped window
  gives 6.8 %; with power-law shapes and asymmetric half-heights **4.5 %**: Eh = |(h - 0.482)/0.486|^1.378,
  Ev = |dy/b|^1.378 (b 511 below / 433 above the centre y 649), V oscillator period 1116 lines; window
  S = (y - 254.7 h - 397.6)/262.4, card iff V2 + 1.039 (1 - tri(S)) < 1 with V2 = |dy2/b2|^1.521 (b2 629 / 456,
  centre 20 lines up). The gaps follow |y - yc|: the upper gap narrows 136 -> 48 px, the middle one is ~15 px, the
  lower one widens 67 -> 118 px.
- **Corner mass.** Left edge x 1358-1382 for y 40-240, then a straight diagonal (1432, 280) -> (1822, 480) (slope
  0.51), horizontal violet bands inside (the zone under it).
- **Fan.** phi = 4.938 P + 1.863 P^2, P = (x + 152.5)(y + 599.5)/1e6 (phase correlation on R - B over the visible
  ground 0.272; the plain product 0.187). Periods 23 px at (1800, 940), 33 px at (1150, 1010), ~85 px on the left.
- **Zone.** Red (153, 3, 0) solid in the core, navy (5, 38, 55) bands of period 40.5 lines (autocorrelation 0.76),
  lens-shaped band ends, thin dark diagonals (the fan's teal phase); an orange patch top-left, a red / navy grid
  bottom-left.
- **Brightness.** Block means (154, 130, 116) top-left, (19, 55, 68) beside the zone, (70..87, 70..82, 70..91)
  bottom middle, (63, 59, 44) bottom right.

## Motion in the clip
All 639 decoded frames (`fr/`, `fitmotion.py`, `gk_t.py`, `motion3.py`).
- Frame to frame nothing jumps (correlation 0.96-0.99): smooth morphing, no tearing.
- **The petal oscillator scrolls down.** Refitting the card model every 0.25 s with only the V centre, the window
  phase and the depth free: yc 666 (5.5 s) -> 1716 (10.5 s), linear, 210 lines/s (one V period per ~5.3 s), while
  the window phase stays at 397 +- 2: the cuts are fixed, the petals slide under them. The whole clip repeats every
  ~5.1-5.3 s.
- **The fan breathes** at the same period: at (1800, 940) the stripe angle swings 50.5 deg (t 0, 5.5 s) -> 35.5 deg
  (3.0, 8.7 s), period 23 -> 30 px (y0 swings -600 .. -1680, the still at one end).
- **The red zone drifts** sideways at half that rate: centroid x 251 (0.5 s) -> 1182 (7.0 s) -> 329 (10.5 s).
- **The program.** One LFO (5.4 s per turn at the default Speed, 60 Hz fields) scrolls the V oscillator (petals
  slide down under fixed cuts), swings the fan's y0 with a triangle and drifts the zone with a half-rate triangle,
  anchored on the still at power-up. `cmp_motion.png` (top = the clip at -0.5 / 0 / +0.5 / +1.0 s, bottom = the
  integer model) shows the cards shrinking into slivers and swelling the same way.

## What it does
1. **Columns.** A 2nd-order DDA (Q26) runs u across the line; h = frac(u); no petal left of u = 0.
2. **Cards.** Eh(h) (256-entry ROM) + Ev(line) < T + camera, AND w < Wthr(line), where w = 1 - tri(S),
   S = Sy(line) - (m/P) h (an 8 x 10 product). Ev, V2 and Wthr come per line from a 512-entry power-law ROM.
3. **Corner petal.** A 1024-entry x-ROM gives its x-term; per line 1 - Epy and the diagonal cut position.
4. **Fan.** A 2nd-order DDA (Q24) with per-line start phase and steps; + 1.5 camera; top 3 bits = stripe phase.
5. **Zone.** max(oblique blob from a 2nd-order DDA, corner lobe from the x-ROM) + camera + a stripe push, against
   two per-line levels (0.36 / 0.05 + 0.32 cos of the 40.5-line stripes): red / navy / ground.
6. **Colour.** A 1024 x 24 ground palette by {palette, zone, stripe phase, brightness} (brightness from a 64 x 64
   virtual-px grid ROM) and a 256 x 24 card palette by {palette, zone, stripe phase, brightness / 4}.
7. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (block-RAM taps, running-sum resync), luma 13/16,
   the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | the petal column count: x0.42 (one huge petal) .. x2.4 (a dense picket of petals), exponential; 512 = the still's 3.4 columns |
| K2 Chirp | how much the columns widen to the right: 0 = they narrow to the right, 512 = even columns, 768 = the still, 1023 = x2 (the right column swallows the edge) |
| K3 Slant | the cuts' slant: 0 = tilted the other way, 512 = flat cuts, 760 = the still, 1023 = twice as steep |
| K4 Speed | bipolar (+-8 deadband, square law): the petals scroll down (+) or up (-) under the fixed cuts, with the fan breathing and the zone drifting; 700 = the clip (~5.4 s per cycle), full = 0.7 s |
| K5 Size | the petals' size: 0 = small round chips, 512 = the still, 1023 = fat cards merging into a slanted lattice |
| K6 Warp | how far the camera (or the Osc) bends the fan, swells the cards and spreads the red zone (square law); 300 = the default |
| S7 Zone | On (the still) / Off: no red zone, the fan everywhere |
| S8 Petal | On (the still) / Off: no big corner mass, the fourth column shows |
| S9 Palette | Magenta (the still) / Citrus (yellow cards, violet and green ground) |
| S10 Chevron | Off / On: the cuts reverse on the left half of the screen and meet the right half's in chevrons |
| S11 Source | Video / Osc |
| P12 Confetti | the cuts' frequency: 0 % = whole uncut petals, 50 % = the still, 100 % = a shower of thin slivers (P = 2100 .. 262 .. 33 lines) |

**Presets:** Still; Held Still (Speed off); Whole Petals (P12 0); Shower (P12 930, Size 640); Chevrons (Chevron,
Slant 900); Citrus Lattice (Citrus, Size 900, Columns 700); Bare Fan (Zone and Petal off, Size 300); Osc Storm (Osc,
Warp 640, Speed 900).

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16 from a one-line buffer (every second pixel at HD), times
  the Warp gain, is added three ways, as one CV multed to three modules: to the fan's phase (1.5 periods per full
  luma x warp: the stripes bend around the picture), to the card threshold (bright areas swell the cards) and to the
  zone level (bright areas turn red). At the default Warp the composition still reads (`cmp_final.png`, last panel).
  With no input, Video mode is the still.
- **Osc mode.** The shell's 640 x 540 virtual-px triangle lattice, drifting, feeds the same path at four times the
  depth: the fan folds into angular zigzags, the cards swell along the lattice ridges, the zone spreads. Warp stays
  live in both modes.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 19 slots of 24 clocks, operands staged in
  two registers by the slot number (registered strobes): f1 ub xs/2, f1 ub (315 - x0), uc |kc| and f1 uc kc, the fc
  terms of the column start, first and second differences, |slant|/P and 1/P, warp^2 and the warp gain, speed^2 and
  the LFO rate, the V centre, the fan y0 and the zone centre (triangle LFOs), the zone's dx0^2 and first difference.
  Exponentials are a linear mantissa shifted (2^(E/256) in Q12, three knobs). Shared glide (1/8 per field) on K1,
  K2, K3, K5, K6, P12.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in ~480 clocks (fits SD's 858) on a
  second serial multiplier with 22 slots of 21 clocks: the V fold, Ev and V2 (power-law ROM), Wthr, the window
  phase, the fan's beta, start phase and both steps, the zone's start value (Q12) and first difference with the
  tilt, the lobe and corner-petal levels, the stripe level (a 32-entry table) and the diagonal cut.
- **Beam / pixel path (15 stages, C_LAT 20).** Stage 1: pixel count, virtual x, x-ROM and brightness-grid
  addresses, the left-half and cut flags. Stage 5: the column, fan and zone DDAs. 6: cell phase (Chevron flip) ->
  petal ROM, fan + camera, zone and lobe levels, corner petal, card threshold. 7: window product, zone max.
  8: window phase, stripe phase, zone level. 9: shape and window compares, stripe push. 10: keys and zone levels.
  11: palette addresses. 12-13: ROMs and fabric copies. 14: card over ground. 15: compose. Boxes 16-19, luma 20,
  gate 21.
- **ROMs / RAMs (22 EBR):** ground palette 3 x 1024 x 8 (6), card palette 256 x 24 (2), petal shape 256 x 12 (1),
  power law 512 x 12 (2), x-terms 1024 x 16 (4), brightness grid 1024 x 4 (1), line buffer 1024 x 8 (2), two box
  rings 256 x 32 (4, `no_rw_check`).
- **Contracts:** the generated avid (parity-locked); one latency (C_LAT 20) in every mode; virtual 1080-line
  coordinates (SD shows the centre 4:3); colours in the hardware convention (U = Cr) from `confetti.py`, luma
  pre-scaled 16/13, capped 940; output conditioning with the running-sum resync before the blanking gate.
- **Verified (GHDL).** 720p60 decimation 1, black input, default knobs (`rtl3_video_black.png`) against the still:
  mean colour (157, 25, 148) vs (152, 26, 152); card share 0.480 vs 0.491; red share 0.116 vs 0.110; card mask
  agreement 93.6 %; 4 x 4 region means within 11 levels (median), 59 (max). Against the integer model `imodel.py`
  the structure matches with differences only along edges and in the fine fan phase (a 1-px / 1-line alignment
  offset of the comparison; not bit-exact). Motion: two 1080p-raster captures 4 fields apart at full Speed match
  the model at LFO phases 2 and 6 (card masks 99.5 % / 99.7 % equal), the petals moving down. Osc and camera frames
  checked by eye.
- **Fit and timing history.** The first build (true Mirror, all shape terms as DDAs, brightness ramps with
  multipliers, polynomial exp2) came to 9885 LC (128 %). Restructured: Mirror became Chevron (no reverse DDA),
  the lobe / corner-petal x-terms and the brightness field moved into two x-indexed ROMs, the line buffer halved,
  exp2 became a linear mantissa, both engines got staged operands, narrower DDAs (column Q26, fan Q24, zone start
  Q12): 6893-6914 LC. A first v2 build had a 12-bit / 13-bit compare bug in the corner cut (the petal covered the
  right edge low down; seen in the sim, fixed). Profiling then showed the line engine's slot decode into its result
  enables (10 ns routing) and the stripe table lookup in one clock: registered strobes / selectors and a deferred,
  pre-offset table moved HD Analog from 76.7 to 82.3 MHz.

## Status
- **v0.1.0, built 2026-11-09, router2 (the default).** **HD HDMI = seed 5** (seeds 1, 3, 4 missed, seed 2 stalled
  the router); the other five configs on seed 1.
  `cd /home/ron/videomancer-dev && MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh confetti`. Not flashed.
- Post-route Fmax: HD Analog 82.33, SD Analog 83.49, HD HDMI 78.57 (seed 5), SD HDMI 84.78, HD Dual 88.00,
  SD Dual 83.22 MHz.
- 6890-6914 / 7680 LC (89.7-90.0 %), 22 / 32 EBR. iceunpack OK x 6; the packed config carries the labels and the
  8 presets. `out/rev_b/ron/heirloom/confetti.vmprog`.
- dupin: 0 `CARRY I0==I1` (2 LUT duplicate inputs, harmless). If a rebuild of HD HDMI gets stuck on router2, try
  `NEXTPNR_ROUTER="--router router1"` (a router1 profile of HD HDMI was still routing after 10 minutes and was
  stopped, so it is untested).
- No pin-skew seed screen was run. If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Card edges.** The still's cards have a blue fringe on the left / bottom and a red one on top (camera chroma
  offset) and soft edges; the RTL's edges are the shell's horizontal blur only, no fringe. The slivers and
  triangles are a little fatter / rounder than the still's thin parallelograms (fit residual 4.5 %).
- **Corner mass.** A static disc cut by a straight diagonal; in the clip its left edge breathes 1360 <-> 1200 as
  the third column's petals pass under it, here only the scrolling third column adds to it.
- **Fan.** a P + b P^2 of the hyperbolic product: the stripes are a little too wide at the bottom right and the left
  arcs a little too regular; the breathing is a triangle sweep of y0, not a raised cosine.
- **Zone.** An oblique blob plus a corner lobe: the still's zone reaches further down the right edge (to y ~860)
  and its navy bands are denser above the cards; the bottom-left grid is a short stripe of red / navy bars. In the
  clip the zone also grows navy bands as it moves; here it drifts as one shape.
- **Brightness.** A 64 x 64 virtual-px grid of 16 levels: soft blocky steps show in the orange top-left patch.
- **Mirror.** A true column mirror did not fit the budget: S10 Chevron reverses the cuts on the left half instead
  (the column layout stays asymmetric).
- **Borders and offset.** The output conditioning delays the picture ~12 px right (the left edge fades in from
  black); the capture's dark borders are not drawn.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (the outer quarter of the first and
  last columns and part of the corner mass fall outside). The camera's line buffer stores every second pixel at HD
  (the warp is coarser horizontally there).
