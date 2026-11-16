# Cartouche (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/cartouche.vmprog`.

## What it draws

One nested figure filling the screen: bands of constant distance from a core rectangle. Inside the core the
bands are sharp nested rectangles; outside they are rounded rectangles whose corners round off band by band
until the far bands are near circles. After Egyptian cartouches, Frank Stella's concentric stripe paintings,
the rounded-rectangle TV screen and Josef Albers' nested squares, softened. The default frame: a wide 2.8:1
core (432 x 152 virtual px) and about eleven bands from it to the top edge, stepping black, dark grey, light
grey, white, streaming slowly outward.

    D     = sdBox(p, b') + e,   b' = max(b + e, 0) per axis
    phase = D / P - drift + camera          Grey: tone (n mod 4) / 3    Ink: n mod 2
    e > 0  (Round < 50 %): square-cornered (Chebyshev) bands out to distance e, rounding beyond it
    e = 0  (Round 50 %):   the true signed distance of the core box
    e < 0  (Round > 50 %): a rounded core (corner radius -e), then a stadium, then a fat oval core

The sum is smooth (C1) everywhere, so every band is a clean curve; every edge is anti-aliased over one pixel.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the figure's centre, +-1920 virtual px (one screen width either way). Glided |
| K2 | Y | +-1088 virtual px (one screen height). Glided |
| K3 | Scale | band period 12 .. 200 virtual px, exponential (default 49: about 11 bands to the top edge). Glided |
| K4 | Core | the core box, continuous: horizontal line (stadium bands) .. 2.8:1 (default 26 %) .. square (50 %: rounded squares to circles) .. vertical line. The half sides sum to 292 virtual px. Glided |
| K5 | Drift | bipolar: up = the bands stream outward, down = inward; +-24 counts round the centre = still; else 2^-12 .. 2^-4.1 of four bands per field, exponential (default 680: about 0.3 band per second) |
| K6 | Camera | the luma added to the band phase, 0 .. 3.4 bands at white (default 100 %). Glided |
| S7 | Glide | On (default): X, Y, Scale, Core, Camera and Round ease 1/16 of the gap per field (at least 1/8 count; X and Y 1/4 count = 0.94 / 0.53 virtual px), landing exactly; the Drift speed eases 1/8 per field. Off: the held knob value, this field. A +-1-count hold against pot jitter in both |
| S8 | Pair | Single (default) / Pair: the same cartouche turned 90 degrees at the same centre, a translucent film streaming the other way at 3/4 speed. Grey: the two films averaged (a woven plaid); Ink: XOR moire |
| S9 | Tone | Grey (default: black, dark, light, white steps) / Ink (black and white bands) |
| S10 | Weight | Bold (default: equal bands) / Fine: a line of 1/6 band on every band edge (at least 2 virtual px); Grey lines step white, light, dark, light (an engraving), Ink lines are white |
| S11 | Source | Video (default) / Osc |
| P12 | Round | the headline: how the corners round. 0 % = square-cornered rectangles out to the farthest corner the screen can show (Albers); the rounding then creeps in from the screen corners band by band; 50 % (default) = the true distance (sharp inside, rounder outward); 100 % = round fast: a fat oval stadium core (corner radius 3/4 of the long half side) and near-circular bands within a few periods |

## Motion

The band phase drifts: one accumulator per film (Q18 of four bands = one Grey tone cycle), advanced once per
field in the vblank zone, wrapping modulo exactly four bands, so the stream never jumps in Grey, Ink or Fine.
Speed from Hypnos v1.1.0's SPEED decode (|v - 512| < 24 still; exponential mantissa from the 2^(i/256) ROM, one
shift), with the top capped at 2^-4.1 of four bands per field (0.23 band per field) instead of 2^-2: at a full
band per field the Ink bands would strobe (invert every field). Default 680 = 0.0049 band per field (about 14
virtual px per second at the default period, clearly moving). The Pair film runs at -3/4 of the same speed.
Glide eases the speed, never the accumulated phase.

## Camera

The camera is never shown. Its luma (minus black) x K6 goes through the shell's box (128 px at 1080, 64 at 720p
and SD, edge-replicated, restarted every line) and vertical one-pole (1/32 per line at 1080p / 720p, 1/16 at
1080i, 1/8 at 480i / 576i) and is added to the band phase of both films, so the bands bulge round bright areas
and crowd where the picture changes: `camera.png`.

- 25 %: up to 0.85 band at white, a gentle swerve;
- 50 %: 1.7 bands;
- 100 % (default): 3.4 bands at white: a bold bend, the subject readable as contour-like bulges (on bear.png
  the ears, eyes and muzzle), every edge still a clean curve. K6 = 0 or a black input = the clean figure, bit
  for bit.

The anti-aliasing follows the bend (the camera's horizontal and vertical phase steps are added to the pixel's
footprint). Where the bands fall under about 2 pixels (finest Scale at SD with a steep camera) they fade to
their mean grey instead of aliasing. Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice
at twice the level (6.9 bands peak at 100 %): big diamond pinches drifting across.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks", about 500 zone clocks): the hold + glide of six knobs,
  the Drift hold and speed glide, then a sequencer on ONE 24 x 21 serial shift-add multiplier (12 slots;
  signed operands sign-extended, so no sign logic): the period from a 256-point 2^(i/256) ROM (H = bands per
  virtual px for the footprint and the Round offset, G = the CORDIC scale), the field-line step, the pixel
  footprint, the core half sizes (shift-adds), the Round offset e and b' = max(b + e, 0), b' in CORDIC units,
  e / P, the pixel-0 and line-0 coordinates, the pixel step; then the Drift step and the per-film offsets
  e / P - drift.
- **Per line**: the y DDA (Q20) and |y| - b' for both figures, computed right after each line's values are
  taken.
- **Per pixel** (34 stages + output): the x DDA -> |x| (one's complement) -> qx = |x| - b'x -> the CORDIC input
  (max(qx, 0), max(qy, 0)), or (not max(qx, qy), 0) inside the core -> an 8-stage vectoring CORDIC (the
  length, gain K8 folded into the per-field scale; units float by the period's octave so a 21/22-bit word
  covers every Scale) -> D / P mod 4 bands (Q12) -> + e / P - drift. With Pair the one CORDIC takes the
  figure on even pixels and the turned figure on odd ones, and each film's phase on the other pixels is the
  mod-4 mean of its two neighbours (the phase is smooth: the error is a small fraction of a pixel) -> + the
  camera -> the nearest band edge and the tones either side -> coverage by a 4-step divider per film
  (17 levels, footprint + camera gradient) -> tone (single: the film; Pair Grey: mean; Pair Ink: XOR triangle)
  -> fade to the mean past 1/4 band per pixel (1/8 for Fine) -> luma 64 + 9.125 t.
- **Output**: luma [1 2 1] / 4, 13/16 toward black (white = 777), blanking gate; U = V = 512 on every pixel.
  Generated avid (the shell's parity-locked start), one latency (40) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3).

Chassis: Kinetic riley's [A] timing / zone / width / generated avid and the serial multiplier of [C], the
Hardline [B] hold + glide (dazzle) and [D] source field (scallop's, with dazzle's Osc fix: both lattice phases
restart in the zone), the luma half of [F]. No wave engine, palettes or chroma paths.

## Status

- v0.1.0 built 2026-11-16 through the shared queue, **router2** (the builder default), **every config on seed
  1**. Not flashed.
- Post-route Fmax: HD Analog 87.94 MHz, HD HDMI 77.56, HD Dual 81.97 (HD needs 74.25); SD Analog 82.07,
  SD HDMI 81.97, SD Dual 76.20 (SD needs 27).
- 6,983 .. 7,008 / 7,680 LC (91 %), 6 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`, no
  carry self-adds (dupin: 0).
- Rebuild: `./build_set.sh vellum cartouche`.
- Model = RTL (GHDL, the built sources): pixel diff 0 on every render made. 720p60 decimation 1 (918,720
  pixels): the defaults with a black input, Source = Osc, and bear.png at K6 100 % (diffed against the tester's
  own stimulus luma; the model's own image-to-luma conversion differs slightly from the tester's). 720p60
  decimation 2: Pair + Ink with X 700, Scale 300, Round 300; Fine with X 820, Y 760, Core 600, Round 150;
  Pair Grey with Y 300, Scale 700, Core 100, Round 900. NTSC decimation 1: both fields (720 x 243 each) 0.
  Motion: two captures at the default Drift 20 fields apart match the model at 3 and 23 drift steps (2 or 4
  steps differ on 7 % of the pixels, so a single field's travel is resolved); the travel is outward
  (`motion_rtl.png`). Glide On at the power-up defaults with X 600, Scale 700, Round 300 set (the default
  Drift running): 0 against the model after three fields of 1/16 easing (the un-eased model differs on 84 % of
  the pixels). Not rendered in RTL: 1080p and 1080i (the model covers them). The GHDL frame is the
  model's shifted 4 columns right with 4 blank columns at the left (the shell's generated-avid start).
- Renders (work dir `.vellum_work/rb_cartouche/`): `cmp_final.png`, `sheet.png` (control sheet), `strip.png`
  (8 frames across Round), `motion.png` (Drift), `camera.png` (bear.png at K6 0 / 25 / 50 / 100 %), all from
  the integer model `imodel.py` except the RTL panels of cmp_final and `motion_rtl.png` (RTL, fields 3 / 23).
  Design notes and the fit history: `notes.md`; float sketch `proto.py`.

## Cut / deviations

- **Round's square reach adapts to the view.** Below 50 % the square-cornered zone reaches e = E u, E = the
  farthest corner the screen can show from the figure's centre (960 + |X|, 540 + |Y| virtual px, the smaller):
  0 % is Albers in every position and the slider acts from its first count. A fixed reach left the bottom 14 %
  dead in the centred view. Moving X or Y below 50 % therefore also moves where the rounding starts.
- **Round 100 % = corner radius 3/4 of the long half side**, not a full circle core (that read as a plain
  target and lost the cartouche).
- **Pair films are computed on alternate pixels** (one shared CORDIC) and interpolated in phase; the edges are
  still anti-aliased per pixel from the interpolated phase. Two CORDICs did not fit (about 9,000 LC).
- **Drift top capped at 2^-4.1** of four bands per field (see Motion).
- **Pair is busy by nature** (a woven plaid in Grey, an op-art XOR moire in Ink); it is the program's look-flip.
- **Y reaches +-1088 virtual px** (a shift-add 4.25 per count).
- **SD**: the same picture (the centre 4:3), but a 480i field line is 4.4 virtual px, so the finest Scale gives
  2.7-line bands, and the steep vertical camera gradient there (1/8 one-pole) can push bands into the mid-grey
  fade. 720p and 1080p hold every Scale.
- `field_n` is passed through one register (the core ties it high on every input).

## Not sure of on hardware

- Nothing on this chassis family has been flashed yet; the generated avid, zone and width logic are the heirloom
  shell's, which is hardware-validated.
- The picture is full-swing moving edges (white next to black every four bands in Grey, every band in Ink):
  the [1 2 1] + 13/16 conditioning is built in; if HD HDMI bars show, Ink + Pair at a small Scale is the first
  place to look.
