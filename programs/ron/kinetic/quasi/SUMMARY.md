# Quasi (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/quasi.vmprog`.

## What it draws

Quasicrystal interference: the sum of N plane waves of one wavelength at the angles j * pi / N, drawn as flat
contour bands. N = 5 gives Penrose-like ten-fold rosettes, N = 4 and 8 the Ammann-Beenker family, N = 3 a plain
honeycomb. The default frame (N = 5, Ink palette) is a field of about twenty rosettes: off-white ground-level
band, black wells with red cores, grey islands. The waves' phase offsets breathe (the "phason" motion), so the
rosettes dissolve and reform in place instead of the pattern sliding: about 7 s per cycle at the default Rate.

## Element and state

Element: a plane wave (up to eight). State: its phase offset.

    F(p)   = sum_j a_j * w(k_j . p + phi_j) + source        k_j = k (cos, sin)(j * pi / N + spin)
    a_j    = clamp(N - j, 0, 1)                             wave j fades in as N passes j
    phi_j  = Depth * sin(phase + Spread * j) + glide_j
    G      = F * bands;  contour e = round(G);  coverage = clamp(1/2 + distance / |grad G|), 17 steps

p is measured from the screen centre in virtual 1080-line pixels, and with Depth = 0 every wave is a cosine
about the centre, so the centre is the pattern's symmetric rosette. Field = Phason runs `phase`; Field = Drift
freezes it and runs `glide_j` = k_j . d(t) instead, a rigid translation along (1, 1/2).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | wavelength, 64 .. 724 virtual px, exponential, glided (default 138 px) |
| K2 | Shape | the colourizer: bands per 4 wave amplitudes, 1 .. 15.5, exponential in 32 steps per octave. 0 % = two-tone (the sign of F), default 1.8, 100 % = a dense contour map |
| K3 | Depth | phason depth, square law, 0 .. 1 cycle of phase swing (default 1/4). 0 = the still, centre-symmetric pattern |
| K4 | Rate | bipolar, centre = still; exponential, 60 s per cycle just off the deadband to 2 cycles per second |
| K5 | Spread | phase step from wave to wave, square law, 0 .. 1/2 cycle. 0 = all waves breathe in unison; more = the waves take turns, so the rearrangement travels through the pattern |
| K6 | Colour | the 16 shared Kinetic palettes, with hysteresis. Fill: the bands step through ground, A, B, accent. Lines: lines in A, B, A, accent on the ground |
| S7 | Wave | Smooth (sine: round rosettes) / Angular (triangle: straight-edged polygonal tiles). **This replaces the standard Snap**: here the switch shapes the plane waves themselves; the phason modulation stays a sine |
| S8 | Field | Phason (the offsets breathe: tiles flip in place) / Drift (the whole pattern glides rigidly at Rate; Depth and Spread still set the frozen offsets). **Replaces Linear / Radial**, which have no meaning for eight superposed waves |
| S9 | Look | Fill (flat bands) / Lines (the contours alone, 3 virtual px, on the ground colour) |
| S10 | Spin | Off / On: the lattice turns about the centre, one turn in 102 s. Off returns it to upright at once |
| S11 | Source | Video / Osc |
| P12 | SYMMETRY | the headline: N from 3 to 8, continuous, 20 % of the travel per integer (0 % honeycomb, 20 % four-fold, 40 % Penrose (default), 60 % twelve-fold, 80 % fourteen-fold, 100 % sixteen-fold Ammann-Beenker rosettes). Between integers pi / N slides and the newest wave fades in, so the lattice shears from one symmetry into the next |

## Video

`program_type = "processing"`. The picture's luma is the second modulator and **adds to F** (to the same
state as the waves): luma -> 32-pixel box (edge-replicated) -> vertical 1/8 IIR in a half-resolution line store
-> added at 1 x (full white = 0.86 wave amplitudes). Bright areas lift the whole field, so the bands widen,
merge and bend around the picture's masses while every edge stays a smooth contour. With a black input the
term is zero and the waves alone run. Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice
at the same point at 4 x (up to 3.5 amplitudes): large diamonds of extra contour rings drift across.
The source field lags the picture by about 12 pixels to the right.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks", about 2,900 clocks): knob glides (K1, K2, K3, K5, P12);
  seven preamble products and, for each of the 8 waves, six products on one serial 16-step multiplier
  (cos / sin of the wave's direction from a quarter-wave ROM, 4,096 points per turn; per-pixel and per-line
  phase steps; the seed that puts the wave's zero phase on the screen centre, from the timing ID; the phason
  offset; the glide step). Results go to a 32-word register file in block RAM. Then the colour table
  (4 contours x 17 coverage steps x Y / U / V, by accumulation: no per-pixel blend) and the 8 wave tables
  (half a cycle, 512 x 8 bits, sine or triangle, already scaled by a_j) are refilled.
- **Per line**: after each line the 8 phase accumulators are loaded from the register file through one bus
  (clear, add the seed, leave the per-pixel step in the operand register: 20 clocks), then the 8 seeds are
  advanced by their per-line steps (33 clocks). No per-line multiplier.
- **Per pixel** (29 stages): 8 x 20-bit accumulate -> 8 table lookups -> signed sum -> + source -> 2-tap box
  (halves the table quantisation noise) -> x bands (6-bit mantissa as gated shifts, then a 4-way shift) ->
  G (Q2.10 bands) -> gradient by finite differences per REAL pixel (previous pixel; the line above from a
  1,024 x 8 half-resolution store) -> coverage by a 4-step divider (17 levels; past 1/4 band per pixel the
  transition widens 5 x so bands that outrun the raster fade to their mean) -> colour table -> output
  conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]/4 at the same group delay,
  luma 13/16 toward black) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (36) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention (U = Cr).
- Chassis from `riley.vhd`: [A] timing, [B] controls / glides, the serial multiplier, [D] source field (its
  second read port replaced by a delayed copy of the IIR stream), the coverage divider of [E], [F] output.

## Status

- v0.1.0 built 2026-11-14 through the shared queue, router2 (the builder default), every config on seed 1.
  Not flashed.
- Post-route Fmax: HD Analog 85.72 MHz, HD HDMI 87.97, HD Dual 80.55, SD Analog 79.97, SD HDMI 78.59,
  SD Dual 81.61. HD needs 74.25, SD 27.
- 6,779 .. 6,803 / 7,680 LC (88 .. 89 %), 24 / 32 EBR (8 wave tables, 2 register file, 2 G line store,
  3 colour tables, 4 sine ROM, 2 exp + palette ROM, 2 luma store, 1 box delay). Six bitstreams pass iceunpack.
- Standalone yosys on the final sources: 3,656 LUT, 3,060 FF, 24 RAM, `emulate_transparency` 0.
  `dupin.py` was not run (router2 build, no router1 fallback needed).
- Rebuild: `./build_set.sh kinetic quasi`.
- Model = RTL (GHDL), pixel diff against `.kinetic_work/rb_quasi/imodel.py`:
  - 720p60, decimation 1, defaults, black input: **0 of 918,720**.
  - NTSC, decimation 1, defaults: **0 of 173,988 on each of the two fields** (720 x 243; consecutive motion steps).
  - 720p timing at 640 x 360 (decimation 2: the top-left quarter), defaults: 0 of 228,960.
  - Same raster, Angular + Drift + Lines + Spin + reverse Rate + Vega palette: 0 of 228,960 when the model
    leaves waves 6 and 7 out of the first field's glide step; with all eight in, 0.9 % of pixels differ by one
    coverage step. The likely cause is the tester's first vertical blanking being cut short before the
    sequencer reaches wave 6 (inferred from the exact fit, not observed). Real blanking (CLAUDE.md: about
    14,500 clocks NTSC, 99,000 at 1080p) is 4 x to 25 x the 3,900 clocks the sequencer needs.
  - Source = Osc (640 x 360): 10.8 % of pixels differ from the model (up to 284 codes); Video =
    selfie.jpg: 5.9 % (up to 104 codes). Not investigated: the model does not try to reproduce the lattice's
    frame phase, the tester's image resampling or the IIR's history from earlier fields, so the source path
    (including the new delayed-stream read of the luma store) is checked by eye only: both renders show the
    pattern bent smoothly, with clean edges.
  - Matching frames sit at one specific motion step each (the neighbouring steps differ in 22 % of pixels),
    which is the motion check. 1080i and 1080p were not rendered; knob values other than the defaults were not
    rendered in GHDL (the glides need a bypass), only in the model.
- The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left, as in riley.
- Model checks: no single-row hard line of 24 px or more in 12 settings (`thincheck.txt`).
- Renders: `.kinetic_work/rb_quasi/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion).

## Cut / deviations

- **Zigzag-style substitution on S7**: Smooth / Angular shapes the plane waves (spec); there is no Snap.
- **S8 is Phason / Drift**, not Linear / Radial (spec).
- **Spread** is the phase step from wave to wave (8 elements), so it has no spatial seam to show.
- The camera enters at 1 x, not 2 x: at 2 x the collage test image left wobbly outlines on the flattest blobs.
  Osc is 4 x the camera level.
- Scale spans 3.5 octaves, not 4: at 1,024 px the frame held two blobs.
- Drift direction is fixed along (1, 1/2); switching back to Phason drops the glide (the pattern recentres).
- Spin has one speed. Directions are quantised to 4,096 per turn (0.09 degrees): at the far screen edge a wave
  front moves in steps of about 1.5 px while spinning.
- Symmetry: pi / N is interpolated linearly inside each slider segment rather than as 1 / N, and the fading
  wave's amplitude has 64 steps.
- 8-bit wave tables: at 4x zoom the thinnest Lines contours show sub-pixel roughness where F is nearly flat.
- `field_n` is passed through one register (the core ties it high on every input).
