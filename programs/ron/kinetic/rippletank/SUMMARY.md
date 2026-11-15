# Rippletank (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/rippletank.vmprog`.

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
| K3 | Depth | phase swing of the steering wave, square law, 0 .. 1 cycle (default 0.5). How far the fringes swing |
| K4 | Rate | rings run outwards (right of centre) / inwards (left), centre = still; exponential, 60 s per ring just off the deadband to 2 rings per second. The steering wave runs at half the ring rate, the orbit at 1/16 |
| K5 | Spread | phase step between neighbouring sources, square law, 0 .. 1/2 cycle (default 1/4). 0 = all sources in step (the rings only pump), 1/2 = neighbours in antiphase |
| K6 | Colour | the 16 shared Kinetic palettes (ground, A, accent), with hysteresis |
| S7 | Wave | Smooth (shaded) / Hard (two-tone fringes, anti-aliased). **Replaces the standard Smooth / Snap**: the spec gives S7 to the shading, the steering wave is always a sine |
| S8 | Field | Line (an array) / Ring (a polygon, turned a quarter and twice as far out, so even two sources flip visibly) |
| S9 | Mode | Sum (interference) / Max (each pond keeps its own rings: rain on a pond) |
| S10 | Look | Fill / Lines (the nodal contour F = 0 drawn as a line: accent core in Smooth, colour A in Hard) |
| S11 | Source | Video / Osc |
| P12 | WAVELENGTH | the headline: ring density, exponential, 1.2 rings from the centre to the top edge (a few long swells) .. 18 (fine rings, the fringe lattice takes over); rings finer than 4 pixels fade to the mean tone |

The headline is the wavelength, so K1 (Scale) became Spacing, as the spec says.

## Video

`program_type = "processing"`. The picture's luma adds to the summed field: luma -> 32-pixel box -> vertical
1/8 IIR (the riley source chain) -> F + 3/4 luma, limited so that luma alone lifts the field no further than
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
