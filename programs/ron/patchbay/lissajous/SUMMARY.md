# Lissajous — an X-Y scope figure rescanned as a raster

**v0.1.1 built 2026-11-15 (K6 camera pass), not flashed.** Patchbay. The patch nobody builds on a raster rack because it needs two
arcsine shapers: the H and V ramps are bent so that two function generators running at a ratio a : b draw the
Lissajous figure x = cos(b t + phi), y = cos(a t) as a contour of the raster. A colouriser on the same signal
gives the glow bands around it, and a drifting phi turns the figure as if in 3-D. Presets are hand-written
placeholders.

## The patch

    H ramp -> arcsine shaper -> function generator A (x a) ---------> Cx  = cos(a acos x + luma)
    V ramp -> arcsine shaper -> function generator B (x b), +phi ---> Cy+ = cos(b acos y + phi)
                                                          , -phi ---> Cy- = cos(b acos y - phi)
    Cx - Cy+ -> window comparator at 0 --+
    Cx - Cy- -> window comparator at 0 --+-> the figure (warm white), keyed over everything
    |Cx - Cy+|, |Cx - Cy-| -> non-additive mixer (the smaller) -> colouriser -> the glow bands
    Twin:  Cx + Cy+, Cx + Cy- (generator B inverted) -> two more window comparators (cyan, dark edge)
           and into the non-additive mixer
    phi <- a free-running ramp (Speed)        a, b <- K1, K2 + two detune LFOs (Detune)
    camera luma (or the Osc) -> two horizontal slews, one vertical slew -> generator A's phase input
    the comparators' window = the line weight (fixed) x the local slope of the signal (so the line keeps an even weight)

- The two difference amplifiers are the front and the back strands of the figure. With phi on generator B they
  slide opposite ways as phi drifts, which is what reads as a turn in depth. Their zero sets together are exactly
  the Lissajous figure: (Cx - Cy+)(Cx - Cy-) = 0 is the ellipse equation in (Cx, Cy).
- At whole-number a and b the figure closes. Between them the strands run off the frame's edge, so the figure
  opens and tumbles.
- Shaper = Linear pulls both arcsine shapers out: the generators see the bare ramps and the same patch gives a
  lattice of diamonds over the whole screen.

## What it draws

- **Default frame.** A 3 : 2 figure in a square in the middle of the screen, a warm-white line of even weight.
  Orange hugs the line; red, a thin pink fringe, magenta, violet, a cyan fringe and blue step away from it into
  pointed lens- and diamond-shaped cells that fall to flat black at their centres. Left and right of the square the
  arcsine shaper clips, so the bands at the square's edge run on as dim horizontal bars.
- **Motion.** phi drifts (one turn in about 14.6 s at the default Speed), so the two strand families slide through
  each other and the cells open and close. Two slow LFOs (23 s and 37 s at Detune 0, faster with Detune) pull a and
  b off the whole numbers, so the figure also opens at one side and closes again. Nothing loops visibly.
- **Colouriser.** 64 positions per cycle (wide saturated bands, 2-position fringes, a black gap) by 8 brightness
  tiers. Position = the mixer's output times the Glow gain plus Colour; tier = the mixer's output times the Glow
  falloff, so the bands dim away from the figure.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 A | function generator A | its multiple, 1 .. 8, continuous, with a detent (+-18 knob steps) at every whole number: the figure's lobes across. 292 = 3 |
| K2 B | function generator B | its multiple, 1 .. 8, the same way: the lobes down. 146 = 2 |
| K3 Glow | colouriser | a tight orange halo on black (0) .. the default 3 to 4 bands .. a dozen thin rainbow bands filling every cell (1023). Gain 0.6 .. 4.5 cycles (square law); the falloff has 7 settings |
| K4 Speed | phi ramp | bipolar, +-8 deadband, square law: the figure turns one way or the other, up to 1.3 turns/s. 640 = one turn in 14.6 s |
| K5 Colour | colouriser | rotate, +-2/3 of a cycle: which band sits on the figure (orange at 512, violet / blue left, green / yellow right). Was K6 before v0.1.1 |
| K6 Camera | generator A phase input | how far the camera (or the Osc) bends the figure: 0 % = none (the black-input picture), 25 % = the v0.1.0 level, 50 % (default) = twice, 100 % = four times; 8 steps. See "K6 camera pass" |
| S7 Shaper | arcsine shapers | Arcsine (the true figure in a square) / Linear (the shapers pulled out: a lattice of diamonds over the whole screen) |
| S8 Twin | summing amplifiers | Off / On: the figure half a turn away (its mirror), in cyan with a dark edge; the bands hug both figures, so the cells become facets |
| S9 Frame | ramps | Square / Wide (both ramps stretched to the screen) |
| S10 Look | colouriser | Bands / Line: the figure alone on black, with a bloom in the colour the colouriser has at the line (Colour picks it, Glow sets its reach) |
| S11 Source | generator A phase input | Video / Osc |
| P12 Detune | detune LFOs | 0 % = the ratio locked, the figure only turns .. 260 = a slow breathing (+-0.19) .. 100 % = a swept +-3 and b +-2.25 at three times the LFO rate: the figure tumbles through the whole family. The ratios fold at 1, so the picture never degenerates |

## Video

(Levels below are the v0.1.0 ones, K6 Camera at 25 %; v0.1.1 defaults to twice that, see "K6 camera pass".
The line weight is fixed at the old K5 default, 5 virtual px.)

- **Video (default).** Camera luma goes through two horizontal slews (16 px each) and a vertical slew (1/8 per
  line, a line store at pair resolution) and is added to generator A's phase: 0.11 cycle at full white. The strands
  and bands shift sideways with the picture's brightness, so the figure bends through the picture; the slews keep
  the bends smooth. The slew's own line-to-line step widens the comparators' window where the picture has a
  vertical gradient, so the line never breaks there. A black input gives the full picture.
- **Osc.** A drifting 640 x 540 px triangle lattice at three times the camera's full level feeds the same input:
  the figure billows like a flag.

## Engine

- **Per field (vblank zone).** A sequencer: shared glide (K1, K2, K3, K5, K6, P12; the first field after
  configuration loads the knobs directly), the ratio detents, the two LFOs, phi, detune depth, the colouriser's
  gain as mantissa and exponent, the line weight as mantissa and exponent, the x origin from the measured width.
  Seven products, run on the line engine's serial multiplier.
- **Per line.** An engine run at every line end (four times in the zone to prime lines 0 and 1), about 105 clocks.
  It works two lines ahead: |y| scaled into the arcsine table's index, the table, b times theta, the two phases,
  four cosine reads with interpolation (16-bit phase), the line-to-line steps of both cosines times the line
  weight. It shares the arcsine and cosine table ports with the pixel path, taking them for 4 clocks in hblank.
- **Per pixel (30 stages).** x DDA; arcsine table (1024 x 16, half the ramp, mirrored); a times theta (14 x 12);
  plus luma; cosine table (quarter wave, 1024 x 16); Cx 8 px back from a block-RAM ring for the horizontal slope;
  slope times weight; threshold per strand family (larger + 3/8 smaller of the horizontal and vertical parts);
  F and the twin's F; four window comparators at 1/2 and 1 of the weight (full and half blend), the twin's dark
  edge at 2; the non-additive mix; gain and falloff; palette (512 x 32); compose.
- **Shell (kiln's).** Generated avid; one latency (36 clocks) in every mode; chroma [1 2 2 2 2 2 2 2 1]/16 as
  running 8-sums that resync to neutral in every blanking interval, luma [1 2 1], luma 13/16; blanking gate;
  delay rings in block RAM for Cx, chroma and the syncs; virtual 1080-line coordinates from the timing ID;
  colours in the hardware convention (U = Cr).
- **Stores (19 EBR).** Arcsine 4, cosine 4, palette 4, luma line store 4, rings 3 (the Cx ring shares one).

## Status

- **v0.1.0 built 2026-11-14 with router2, all six configs on seed 1. Not flashed.**
- Post-route Fmax: HD Analog 84.22, HD HDMI 81.91, HD Dual 82.76, SD Analog 75.38, SD HDMI 74.39, SD Dual
  70.57 MHz. 7,046 to 7,095 / 7,680 LC (92 %), 19 / 32 EBR.
- Output: `out/rev_b/ron/patchbay/lissajous.vmprog`. Rebuild: `./build_set.sh patchbay lissajous`.
- **Model = RTL.** GHDL at the 720p timing ID against the integer model `imodel.py`
  (`.patchbay_work/rb_lissajous/`):
  - 720p60, decimation 1 (1280 x 720), black input, default knobs: **0 differing pixels** (the picture sits 4 px
    right of the tester's window, the shell's generated avid).
  - 720p60 decimation 1, Shaper Linear: 0 differing pixels.
  - 720p timing ID at decimation 4: default; default four fields later (motion: phi and both LFOs step as the
    model's); Linear + A 5 + Colour; Wide + Twin + six knobs off their defaults + negative Speed: all 0 differing
    pixels.
  - NTSC (decimation 1, 720 x 243 per field) and 1080i (decimation 2): both fields 0 differing pixels against the
    model for that raster, and the same picture as 720p (the square is 510 px of the 720 at SD).
  - Source Osc and Source Video with `selfie.jpg` (720p): same structure as the model; 68 % and 82 % of pixels
    exact (the model places the luma path's samples approximately). Every edge stays a smooth curve.
  - Working files: `cmp_final.png`, `sheet.png` (control sheet, integer model), `strip.png` (8 frames over 4 s).
- Thin-row check on the model (`thincheck.py`): no single-row hard lines at the defaults, at Detune 0 and 100 %,
  Wide, Linear, Twin, Look Line, Glow 0, Line 0 and max, A 8. Glow max and B 8 still have a few one-row fringes
  where the bands run flat.

## K6 camera pass (2026-11-15) — v0.1.1

- **K6 Camera** (0 .. 100 %, default 50 %) scales the level added to generator A's phase: camera luma above
  black, or the Osc lattice (which keeps its three-times-the-camera level). It is applied on the source side,
  before the slews, as an 8-step shift-select. The gain is round(K6 / 128) / 2: 0, 1/2, 1, 3/2, 2, 5/2, 3, then 4
  from K6 = 832 up. One add of {0, y, 2y, 4y} + {0, y/2, y} in its own stage, then a clip at 4095 (half a cycle
  of phase). The selects are latched at the zone start. K6 rides the freed Line glide channel (±4 deadband; the
  first field after load takes the knob directly). A 12 x 6 multiply cost +205 LC in standalone yosys, so the
  shift-select replaced it to stay under ~94 %.
  - 0 % = no camera: the black-input figure.
  - 25 % = x 1 (0.11 cycle at full white), the v0.1.0 level.
  - 50 % (default) = x 2: the strands bend twice as far through the picture.
  - 100 % = x 4 (0.43 cycle at white): the figure is pulled hard through the picture, a bold tangle that still
    reads as the figure.
  - Osc: x 2 at the default, clipped at half a cycle, so the brightest lattice tops flatten. It billows more,
    still smooth.
- **Jaggies:** the vertical slew is 1/8 per line, so the line-to-line steps depend on the raster. In the 720p
  decimation-1 sim, x 2 adds some per-line raggedness where two strands cross near-tangent (`d1_cmp.png`). At
  1080p the lines are 1.5x finer, so x 2 was kept as the default (HD-first). 720p, and SD (240-line fields)
  even more so, show more of it: about 40 % (x 1.5) is the cleaner setting there. The decimation-2 sim (360
  lines) shows tears from x 2 up.
- **K5 Colour** is the old K6, unchanged. **Line (old K5) is fixed at its default 520** (half-width 5 virtual px).
  Its per-field weight arithmetic became per-raster constants (84 / 54 / 42 Q4 px at 1080 / 720 / SD, exactly the
  values v0.1.0 computed for 520). Presets: the old K6 values moved to K5; the Line values were dropped ("Scope"
  had 380, a thinner line; "Neon Weave" had 700).
- Sheet: `.patchbay_work/rb_lissajous/k6sheet.png` (720p60 decimation 2, `selfie.jpg`, K6 = 0 / 256 / 512 /
  1023, Osc at 512, then 720p decimation 1 at 256 | 512). The panels were rendered with the 12 x 6 multiply,
  which gives the same gains at those points (3.94 instead of 4 at 1023).
- **Build v0.1.1 (2026-11-15): router1, all six configs on seed 1.** Post-route Fmax HD Analog 77.89, HD HDMI
  87.70, HD Dual 82.77, SD Analog 87.11, SD HDMI 73.87, SD Dual 84.88 MHz (worst HD 77.89; v0.1.0 on router2 had
  84.22 there). 7,026 .. 7,065 / 7,680 LC (92 %, as v0.1.0), 19 / 32 EBR. `out/rev_b/ron/patchbay/lissajous.vmprog`.
  Not flashed.

## Cut / deviations

- **Two difference amplifiers, not the spec's one.** The spec's single F = cos(a acos x + phi) - cos(b acos y) is
  zero on only half of the figure once phi leaves 0. The pair F+ / F- is the whole figure and gives the
  counter-sliding strands. phi sits on generator B.
- **Bands on the smaller of |F+| and |F-|, not on a product.** The product (a multiplier module) gave rounder
  cells in the prototype but cost about 600 LC; the non-additive mix gives pointed, faceted cells.
- **Twin is the figure half a turn away, not a quarter.** Quadrature needed two more generator outputs per line
  and two more threshold chains. Half a turn is generator B inverted, which is free, and the pair is mirror
  symmetric.
- **Line weight** is even to about +-6 % across directions where the picture is flat. With video the vertical
  part is an upper bound, so the line thickens a little where the picture has strong vertical edges. Weight and
  colouriser gain move in mantissa steps (6 to 12 %), the falloff in 7 steps.
- **First-order window.** At a shallow crossing of two strands of the same family the window covers a short
  flat patch around the crossing.
- The first 4 px of every line, the square's outermost rows (index 1016 and up of 1024) and everything outside
  the square carry no line.
- **SD.** Same picture, coarser: the line weight has a floor of 1.25 real px, and with interlaced fields the
  thinnest fringes are one field line. Wide at SD spans the SD screen, so the figure is 4:3 there.
- (v0.1.0: camera gain fixed; v0.1.1 puts it on K6.) The model treats the luma path's horizontal alignment
  approximately, so video frames match the RTL in structure, not bit for bit.
- **Unsure on hardware:** hue (sim-verified only as structure), the look of the dim bars beside the square, and
  whether the Osc level wants trimming. No pin-skew seed screening was run.

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. **Bend / element**: K6 scales the camera (or Osc) term in generator A's phase, so the camera bends the
Lissajous figure and its glow bands; the picture itself is not shown.

- **Change:** the gain is an 8-step shift-select code (`round(K6 / 128)`), so I added steps at the top instead
  of a multiply. Step 7 (k 832 .. 959) is now x4.5 (was x4), and step 8 (k >= 960) is x5 (was x4: `c_cga` "11"
  + `c_cgb` "10" = 4 + 1). Steps 0 .. 6 are unchanged (0 .. x3), so 100 % = x1.25 the old top. The sum `v_ys`
  (15 bits) has room for 5 x 4095, and `s_yin` still clips at 4095 (half a cycle of phase). Bright camera
  pixels (Y above about 883) now reach that clip at the top, and the Osc already clipped there at x4. Latency
  unchanged. K6's default is 100 % (set by the orchestrator).
- **Look check** (720p60 decimation 4, bear.png, glide bypassed in scratch copies, K6 100 % old vs new;
  `.patchbay_work/rb_lissajous/k6b/k6b_cmp.png`): the same banded figure, bent further by the picture (more
  folds in the bands). No wrap or new artifacts.
- **Build (router1, all six configs on seed 1):** HD Analog 76.93, SD Analog 87.64, HD HDMI 86.32, SD HDMI
  85.29, HD Dual 76.43, SD Dual 84.03 MHz (worst HD 76.43); 7,042 .. 7,062 / 7,680 LC, 19 / 32 EBR.
  `out/rev_b/ron/patchbay/lissajous.vmprog`. Not flashed.

## v0.1.3: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`2048 - v` as `v - 2048`); the simulator
is right, so every render above showed the intended picture, but the v0.1.2 bitstreams did not. Fixed (each
rewritten as `v_f16 := to_signed(2048, 16) - z_av`, same operands and width, the value the simulator always
computed):
- line 602 (sequencer state 75): the fold of ratio a (generator A, K1 A plus LFO 1) at 1.0, 2 - a when the detuned
  ratio drops below 1;
- line 614 (state 100): the same fold for ratio b (generator B, K2 B minus 3/4 LFO 2).
On hardware v0.1.2 (never flashed) the folded value came out as z_av - 2048 and the 14-bit slice gave a ratio of
~12..15 instead of 1..2: whenever the detune LFO pulled a ratio below 1, the figure snapped to a dense ~12-15-lobe
scribble for that part of the LFO cycle and snapped back. That never happens at the defaults (A 3, B 2, Detune
260), but it does at high P12 Detune and whenever K1 or K2 sits at 1 with any detune (half of each 23 s / 37 s LFO
cycle). v0.1.3 folds smoothly at 1, so the figure never degenerates, as designed. No TOML remaps (group 1).
Nothing left as-is (no other audited site in this program). `scan.py` lists no site. Backups (v0.1.2 vhd / toml /
vmprog): .heirloom_work/rb_lissajous/.

Build (2026-11-21, shared queue, router1, every config seed 1): HD Analog 85.34, SD Analog 83.64, HD HDMI 82.67,
SD HDMI 80.21, HD Dual 84.88, SD Dual 81.91 MHz; 7,057-7,094 / 7,680 LC (92.4 %), 19 / 32 EBR; all six pass
iceunpack.

HD HDMI skew: the builder's HD HDMI router1 s1 was +0.05 / <= -1.42 (HSYNC below the window). Screened HD HDMI
router1 + router2 seeds 1-40 (77 routed, 3 timeouts, 67 timing-clean, 10 in the window). Installed **HD HDMI
router2 s4 (+2.02 / <= -0.79, 77.89 MHz)**. HD Analog router1 s1 (-0.90 / <= -2.46) kept. HD Dual router1 s1 had
data <= -0.44: installed **HD Dual router1 s7 (-2.52 / <= -1.50, 85.29 MHz)**. So the shipped seeds are HD Analog
r1 s1, HD HDMI r2 s4, HD Dual r1 s7, SD configs r1 s1. Spare: HD HDMI router1 s35 (+1.15 / <= -0.97, 85.29 MHz),
.heirloom_work/rb_lissajous/skew/lissajous_v013_spare_hd_hdmi_router1_s35.vmprog. Replaced bins kept in
.heirloom_work/rb_lissajous/skew/. Shipped vmprog: out/rev_b/ron/patchbay/lissajous.vmprog (v0.1.3). Not flashed.
