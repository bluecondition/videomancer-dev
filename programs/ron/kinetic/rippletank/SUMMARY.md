# Rippletank (Kinetic)

**v0.1.1 (K6 camera pass, 2026-11-15), not flashed.** Flash `out/rev_b/ron/kinetic/rippletank.vmprog`.
(v0.1.0 built 2026-11-14; its build notes are kept below.)

## What it draws

A physics ripple tank seen from above: two to four wave sources whose ring systems interfere, with the
sources phased like an array so the fringes steer. Art reference: the school ripple tank (Young's two
dippers, phased-array beam steering) and the zone-plate moire of op art. The default frame (Ink palette) is
two sources 86 virtual px apart: one family of off-white rings on near-black, red on the crest tips, cut by
straight vertical nodal bands where the two sources cancel. The rings run outwards (about 7 s per ring at
the default Rate), the nodal bands swing from side to side at half that rate as the steering wave moves the
sources' phases, and the pair turns slowly about the centre (one turn in about two minutes).

**The sources are quadratic ponds, not true circular waves** (see "Cut / deviations"): each source's phase
is q r^2, so its rings crowd outwards and the nodal lines of two sources are straight and parallel, not
hyperbolae.

## Element and state

Element: a source. State: its phase, phi_j = Depth * sin(lfo + Spread * j).

    theta_j(x, y) = q |x - s_j|^2 - phase - phi_j
    F = sum_j cos(theta_j)      (2 sources: / 2;  3 or 4: * 3/8, clipped)       Mode = Sum
      = 2 max_j cos(theta_j) - 1                                                 Mode = Max
    s_j: Line  = an array through the centre, one Spacing apart (-1/2, +1/2;  -1, 0, +1;  -3/2 .. +3/2)
         Ring  = on a circle of radius Spacing, a quarter turn on, 1/N turn apart
    phase advances by Rate once per field; lfo = phase / 2; the layout turns by phase / 16.

q is the headline (Wavelength). F goes to a 256-entry colour table: ground at a trough, colour A at
F = +0.75, the accent on the crest tips. Colour B of the palette is not used.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Spacing | distance between the sources, 11 .. 362 virtual px, exponential, glided (default 86). More spacing = finer fringes |
| K2 | Sources | 2 / 3 / 4 (with hysteresis) |
| K3 | Colour | the 16 shared Kinetic palettes (ground, A, accent), with hysteresis. Was K6 until v0.1.1 |
| K4 | Rate | rings run outwards (right of centre) / inwards (left), centre = still; exponential, 60 s per ring just off the deadband to 2 rings per second. The steering wave runs at half the ring rate, the orbit at 1/16 |
| K5 | Spread | phase step between neighbouring sources, square law, 0 .. 1/2 cycle (default 1/4). 0 = all sources in step (the rings only pump), 1/2 = neighbours in antiphase |
| K6 | Camera | camera / Osc strength, default 50 % (v0.1.1, see "K6 camera pass"): the luma added to F: 0 .. 25 % a gain up to v0.1.0's 3/4, then a contrast about 44 % luma (50 % = slope x 2, 100 % = x 4), 8 levels |
| S7 | Wave | Smooth (shaded) / Hard (two-tone fringes, anti-aliased). **Replaces the standard Smooth / Snap**: the spec gives S7 to the shading, the steering wave is always a sine |
| S8 | Field | Line (an array) / Ring (a polygon, turned a quarter and twice as far out, so even two sources flip visibly) |
| S9 | Mode | Sum (interference) / Max (each pond keeps its own rings: rain on a pond) |
| S10 | Look | Fill / Lines (the nodal contour F = 0 drawn as a line: accent core in Smooth, colour A in Hard) |
| S11 | Source | Video / Osc |
| P12 | WAVELENGTH | the headline: ring density, exponential, 1.2 rings from the centre to the top edge (a few long swells) .. 18 (fine rings, the fringe lattice takes over); rings finer than 4 pixels fade to the mean tone |

The headline is the wavelength, so K1 (Scale) became Spacing, as the spec says.

## Video

`program_type = "processing"`. The picture's luma adds to the summed field: luma -> 32-pixel box -> vertical
1/8 IIR (the riley source chain) -> F + 3/4 luma (v0.1.0; K6 Camera scales it since v0.1.1), limited so that luma alone lifts the field no further than
colour A (the accent stays on the wave crests). Bright areas fill the troughs (Smooth) or fatten the light
fringes (Hard) or push the contour lines aside (Lines); with a black input the offset is zero and the waves
alone run. Source = Osc substitutes the shell's drifting triangle lattice at the same point at full swing.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): glides (K1, K3, K5, P12), one serial 16-step
  multiplier in 48 slots of 24 clocks (12 common: wavelength and spacing through a 2^(i/256) ROM, depth,
  spread, rate, the zone plate's three DDA constants, the fringe frequency; 8 per source: cos / sin of its
  angle and the steering sine from a quarter-wave ROM, its plane wave's steps per pixel and per line and its
  value at the first pixel), the palette, the colour-table fill by accumulators. About 1,560 zone clocks.
- **Per line**: a small engine computes the next line's zone-plate start value (dy^2, q dy^2: two serial
  multiplies, 39 clocks) and the zone plate's step per line; the four plane waves just add their line step.
- **Per pixel** (21 stages, no multiplier): a second-order DDA (the shared zone plate, 36 bits) and four
  first-order DDAs (24 bits) -> four phases -> four quarter-wave cosine ROMs -> sum or max -> + luma ->
  fade -> Fill / Lines -> Smooth index, or Hard gain 2^(7 - n) with n = floor(log2 G) -> colour table ->
  output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 at the same
  group delay, luma 13/16 toward black) -> blanking gate. G is the local ring frequency in 1/1024 cycle per
  pixel, from the DDAs' own first differences (octagonal) plus the sources' fringe frequency, so Hard edges
  are 1 .. 2 pixels wide everywhere without a divider, and rings under 4 pixels fade (3/4, 1/2, 1/4, 0).
- Generated avid (the shell's parity-locked start), one latency (28) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention.

## Status

- v0.1.0 built 2026-11-14 through the shared queue, router2 (the builder default). Not flashed.
- Post-route Fmax: HD Analog 81.99 MHz (seed 1), **HD HDMI 75.85 (seed 2)**, HD Dual 77.58 (seed 1),
  SD Analog 61.27 (seed 1), SD HDMI 70.49 (seed 1), SD Dual 76.34 (seed 1). HD needs 74.25, SD 27.
  HD HDMI seed 1 never left the analytic placer (7 minutes, where the other configs take under 2 in all);
  that attempt was expired by hand and the builder went on to seed 2. Its 1.6 MHz margin is run-marginal.
- 7,019 .. 7,066 / 7,680 LC (91 .. 92 %), 17 / 32 EBR. Six bitstreams pass iceunpack. No
  `emulate_transparency` in a standalone yosys run. router1 was not tried (dupin on the standalone netlist:
  no carry self-adds, two LUTs with a doubled sign bit).
- Rebuild: `./build_set.sh kinetic rippletank`.
- Model = RTL (GHDL, 720p60, black input), pixel diff 0 on five renders: the defaults at decimation 1
  (918,720 pixels) and at decimation 2; Hard + Lines + Ring + 4 sources + Vega; Max + 3 sources + reverse
  Rate + Citrus; and two sim-only copies with other glide start values (all four glided knobs at the top
  with 4 sources and Hard, captured after 5 fields of motion; all four near the bottom with 3 sources,
  Ring and Lines). The GHDL frame is the model's shifted 4 columns right (generated avid), as on riley.
- Source = Osc differs from the model in 17 % of pixels by 0.44 code on average (largest 11 of 1023); the
  cause was not isolated (riley puts its own, smaller, Osc difference down to the lattice / filter phase).
  The camera render was looked at, not diffed.
- Rasters rendered: 720p60 only. **No SD (ntsc) and no 1080i frame was rendered**, and 1080p only in the
  model.
- Renders: `.kinetic_work/rb_rippletank/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion).

## Cut / deviations

- **Quadratic ponds instead of circular waves (the spec's per-source square root is not in).** Four true
  distances per pixel need four normalise + interpolated-table roots (about 360 LC, 2 block RAMs and a small
  multiplier each) plus much wider DDAs; it was not attempted in this run's time and fit budget. With
  theta = q r^2 every source is exact on DDAs. What is lost: evenly spaced rings (they crowd outwards, and
  the wavelength control is a ring density), and the hyperbolic nodal lines: two quadratic ponds interfere in
  straight parallel fringes, three or four on a ring in a hexagonal / square lattice. Steering moves the
  fringes sideways instead of swinging lobes.
- S7 is Smooth / Hard shading (the spec's substitution for Snap).
- Colour B of the palettes is unused.
- Hard edges use a per-pixel power-of-two gain from the ring frequency, not a true gradient: where two
  sources cancel (the nodal bands) the edge goes soft instead of staying crisp.
- With Spread at 0 the Depth knob only pumps the rings in and out together (the engine's "unison").
- At small Spacing with the sources in antiphase the centre of the picture goes to the mean tone for part
  of the steering cycle (the two ponds cancel).
- SD: the same picture (the centre 4:3), but a 480i line is 4.4 virtual lines, so the 4-pixel fade starts
  much nearer the centre: at high Wavelength settings the outer part of an SD picture is the mean tone. 720p
  fades the far corners at the top of the slider; 1080p holds nearly the whole frame.
- `field_n` is passed through one register (the core ties it high on every input).

## K6 camera pass (2026-11-15)

v0.1.1. Ron: K6 is the camera strength in every program, default 50 %.

- **K6 Camera** (0 .. 100 %, default 512) sets the lift the source luma l (0 .. 219; or the Osc lattice, same
  path) adds to the field before the colour-A limit. 8 levels of the glided knob (16 steps of k / 64): gain
  g = 1/4, 1/2, **3/4 (k 192 .. 319, so 25 % = v0.1.0)**, 1, **3/2 (k 448 .. 575, the 50 % default)**, 2,
  **3 (k >= 832, 100 %)**, each = (1 or 3) x 2^n quarters (a shift and one gated add per pixel):
  - 0 .. 25 %: a plain gain, lift = l g. **0 % = no lift** (exactly the black-input picture: coded as g = 1/4
    with an offset no luma reaches).
  - past 25 %: a contrast about l = 96 (about 44 % luma): lift = l g - (g - 3/4) x 96 (a negative lift does
    nothing: stage 11 keeps max(F, .)). The lift at the pivot stays v0.1.0's; only the slope grows:
    **50 % = slope x 2**, **100 % = x 4**. Darker picture areas lift less and show the full ripples, brighter
    ones fill to colour A, so the picture reads as bold light / dark masses over the waves.
  - Why not a plain x 2 gain: the lift is capped at colour A, so any extra gain only whitens. Plain x 1.5 l
    (50 %) washed most of the selfie frame to flat colour A; x 3 (100 %) left only the red crest lines
    (`.kinetic_work/rb_rippletank/k6/k6_512_plaingain.png`, `k6_1023_plaingain.png`).
- Decode: the old K3 Depth channel of the shared glide now carries K6 (1/8 of the gap per field, the shared
  +-2 deadband: with 8 levels a wider one bought nothing for its LUTs); a case on its top 4 bits gives the
  shift, the x 3 flag and the offset (registered every clock).
- Pixel side: one stage (s9_l shifted, plus twice that when x 3), then the stage-10 add takes F + lift - offset
  (a three-operand add, as v0.1.0's F + l/2 + l/4). The source lags the waves by one more pixel; the program's
  latency is unchanged (28).
- **Fit** (rippletank was 91 .. 92 % LC): the first version (a 6-bit gated-sum gain, a floor at 0, the offset by
  adders) reached 7,251 LC and the placer failed on both routers ("unable to find legal placement", HD Analog);
  a 4-bit version still failed SD Analog's placement at 7,130. The shipped version (standalone yosys: 4,027 LUT
  + 2,843 FF against v0.1.0's 4,033 + 2,848) has the 8-level shift gain, the offset folded into stage 10,
  Depth's square as a constant (720^2 / 16; its sequencer slot falls into the default operands), the shared
  glide deadband, and **the source
  interpolation dropped**: v0.1.0 averaged neighbouring words of the half-resolution luma store; now each
  word feeds two pixels as is. The field is 32-px box blurred, so neighbouring words differ by a fraction of
  a code and the step is not visible in the renders.
- **Frozen**: K3 Depth at 720 (its default: a steering swing of 0.5 cycle, square law). Its agent reported
  Depth only acts at Spread 0, and in a still it changes the picture least of the knobs.
- **K6's old function, Colour (all 16 palettes, value labels Ink .. Plum), moved to K3.** Presets: their
  Colour values moved to K3; Array (Depth 850) and Lattice (500) lose their own Depth.
- Renders (GHDL, 720p60 at decimation 2, selfie.jpg, glide-bypassed copy, Sources 2, Ink; the shipped RTL):
  `.kinetic_work/rb_rippletank/k6sheet.png` (K6 0 / 256 / 512 / 1023 and Osc at 512). 0 = the waves alone;
  256 = v0.1.0's lift (bright areas fill the troughs); 512 = darker areas keep full ripples, brighter ones fill;
  1023 = near two-tone light / dark masses over the waves; Osc 512 = one smooth filled lobe of the lattice.
- **Build v0.1.1** (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic rippletank`): **router1**. Seeds: HD
  Analog **seed 2** (seed 1 missed timing), SD Analog **seed 2** (seed 1 stalled the router), the rest seed 1.
  Post-route Fmax: HD Analog 79.57 MHz, HD HDMI 81.99, HD Dual 77.62 (worst HD 77.62, v0.1.0 75.85); SD Analog
  70.53, SD HDMI 75.77, SD Dual 77.62. 7,056 .. 7,081 / 7,680 LC (92 %, v0.1.0 7,019 .. 7,066), 17 / 32 EBR
  (unchanged). Log `.kinetic_work/rb_rippletank/build_v0.1.1.log`.


## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron, after the fern / kiln K6 = 100 % renders: a slightly stronger camera effect at the top of the knob.
The orchestrator set the K6 default to 100 % (1023) and bumped the version; this pass changed the logic.

- Classification: **element** (K6 sets how hard the camera luma lifts the ripple field toward colour A: it
  re-shapes the drawn waves, it never shows the camera image as such).
- Change: the gain is a step code, so one step was added at the top: g = 4 (16 quarters) for k >= 896 (the
  nearest representable step to 1.25 x the old top g = 3; slope x 5.3 against v0.1.0 instead of x 4). g = 3 now
  holds only k 832 .. 895; every lower step is unchanged. The step is built as 8 + 8 quarters: the gated add
  that made x 3 (v_ls + 2 v_ls) gets a second flag (c_cb2) adding v_ls itself, still one LUT per bit; k_g max
  4,080 < 2^12. The contrast offset (g - 3/4) x 96 is 312 for the new step, so its register grew from 5 to 6
  bits; the stage-10 sum stays inside 11 bits (at most 190 + 1,020 - 312).
- Look (GHDL, bear.png, 720p60 / 4, glides bypassed in the scratch copies): `.kinetic_work/rb_rippletank/k6b/
  k6b_cmp.png`. Nearly the same frame: the bright areas were already filled to colour A at the old top, so the
  steeper contrast shows only as slightly more light / dark separation in the mid-tones. No wrap or new artifacts.
- Doubt: the lift caps at colour A, so the top step reads subtly on a bright picture.
- Build (`.kinetic_work/tools/k6build.sh kinetic rippletank`): router1. Seeds: HD HDMI **seed 2** (seed 1 missed
  timing), SD HDMI **seed 3** (seeds 1 and 2 stalled the router; no CARRY I0 == I1 self-add in the netlist,
  so congestion), the rest seed 1. Post-route Fmax HD Analog 77.62, HD HDMI 75.13, HD Dual 76.09, SD Analog
  78.51, SD HDMI 75.77, SD Dual 77.58 MHz (worst HD 75.13: a thin margin). 7,084 LC at most (92.2 %, +3),
  17 / 32 block RAMs. vmprog: out/rev_b/ron/kinetic/rippletank.vmprog. Not flashed.
