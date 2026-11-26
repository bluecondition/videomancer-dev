# Gyre: a lattice wound into a vortex

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 2. Visual Cortex ramps through a Navigator whose Rotation CV is
driven at video rate by a shape gradient (and by the camera), mirrored and combined in Arch, folded into bands by
Staircase (or outlined by Doorway) and coloured by Color Chords: a lattice of nested diamonds that is rigid at the
middle of the screen and winds up toward the edges. Presets are hand-written placeholders.

## The rack

- **LZX Visual Cortex**: the H and V ramps, each through a gain into the Navigator (K2 Width, K3 Height); the input
  decoder (camera luma); its key generator, AC-coupled (the threshold is the picture's own average luma), and its
  compositor in Fade mode with B = black, so the weave goes black where the camera is darker than average.
- **LZX Navigator**: the 2-D rotator (four multipliers and two sums in the module; here a CORDIC). Rotation knob
  (K1); the Rotation CV input at video rate.
- **LZX DSG3** (or the Visual Cortex ramp mixes): the shape gradient into the Rotation CV: Ellipse (two parabolic
  ramps, xc^2 + yc^2) or the V ramp (S7).
- **LZX Arch**: Mirror x2 (S8 In / Out) and the analog logic, Average / Absolute (S9).
- **LZX Staircase**: Frequency (K4, the gain ahead of the doublers) and Phase; the /1, /2, /4 outputs.
- **LZX Doorway** (S10 swaps it in for Staircase, same cables): Outline mode; Gain (K4) = outline width; Threshold
  from the phase LFO's triangle.
- **LZX Color Chords**: the three Staircase outputs key three coloured layers over black (magenta /4 at the
  bottom, amber /2, blue /1 on top); K5 turns the three colours around the hue circle together.
- **Two LFOs** (a Pendulum): a 13 s sway of the base angle (+-7 degrees) and a 7 s saw on Staircase Phase (its
  triangle, 14 s, on Doorway Threshold).
- **The encoder**: horizontal softness, luma 13/16, no wrap.

## The patch

    Visual Cortex H ramp -> gain (K2 Width) -> Navigator H in
    Visual Cortex V ramp -> gain (K3 Height) -> Navigator V in
    Navigator Rotation knob (K1) + LFO A (13 s, +-7 deg)                          -> the base angle
    DSG3 gradient (S7: Ellipse / V ramp) -> Rotation CV attenuverter (P12 Twist) -> Rotation CV
    camera luma (8-px cells, vertical slew, two horizontal slews) x K6       -> Rotation CV    (S11: Osc lattice, 2x)
                                                                             -> V. Cortex key generator (AC-coupled)
    Navigator H out, V out -> Arch Mirror x2 (S8: In = ramp -> triangle / Out = the saws pass)
    -> Arch logic (S9: Average (a + b) / 2 / Absolute |a - b|)
    -> Staircase source (K4 Frequency; LFO B saw -> Phase)  or  Doorway outline (K4 Gain; LFO B triangle -> Threshold)
    Staircase /4 /2 /1 -> Color Chords layers 1 2 3 (K5)    or  Doorway key -> Color Chords layers 1 2 + white core
    Color Chords RGB -> Visual Cortex compositor A (B = black, VC = the camera key) -> encoder

## What it draws

At the defaults: a weave of nested magenta / amber / blue diamonds (Staircase bands inside each Arch cell, each
output subdividing the last) across the whole screen. At the centre the lattice is square and rigid; going out,
the rotation angle grows with the squared radius (half a turn more at the corners), so the cells shear into curved,
swept, pinwheel-like arcs and the outer ring of the screen becomes fine concentric swirls of colour.

Motion at the defaults: the bands flow outward through every cell (Staircase phase, one cycle of four bands in
7 s) while the whole vortex sways back and forth by +-7 degrees over 13 s.

## Controls

| Control | Module | What it does and its range |
|---|---|---|
| K1 Rotation | Navigator Rotation | the base angle, 0 .. 180 degrees (the mirrored lattice repeats every half turn: the ends meet, no jump); default 3.5 deg |
| K2 Width | Visual Cortex H ramp gain | cells across the screen, 1 .. 12 (square law); default 5.4 |
| K3 Height | Visual Cortex V ramp gain | cells per screen width down the V axis, 1 .. 12 (square law); default 3.8 |
| K4 Fold | Staircase Frequency (Doorway Gain) | bands per half cell 1 .. 12 (square law: 1 = plain soft cells); with Doorway the outline gain 4 .. 64 (wide soft rings .. hairlines) |
| K5 Colour | Color Chords colour knobs | the three layer colours turned around the hue circle together: four quarter turns of the chroma, each held for half its quarter of the travel and crossfaded over the other half; magenta / amber / blue at 0 and again at 100 % (the ends meet) |
| K6 Camera | decoder level | the camera into the Rotation CV (up to about 0.11 turn for full white; Osc 2x) and into the key (AC-coupled; darker than average -> black). 0 = the black-input picture exactly; default 100 % |
| S7 Twist | DSG3 gradient into the Rotation CV | **Ellipse** (the squared radius: a circular vortex) / **Ramp** (the V ramp: the angle grows down the screen and the lattice fans open like a page turning) |
| S8 Mirror | Arch Mirror | **In** (triangles: closed diamond cells) / **Out** (the unmirrored saws pass: sawtooth bands, the vortex becomes spiral arms with hard seams) |
| S9 Logic | Arch analog logic | **Average** ((a + b) / 2: nested diamonds) / **Absolute** (|a - b|: saltire / woven cells, double the band density) |
| S10 Folder | Staircase / Doorway | **Staircase** (filled soft colour bands) / **Doorway** in Outline mode (only the contour around the threshold passes: the lattice as luminous amber-and-magenta lines with a white-hot core on black; the threshold sweeps so the lines breathe from a dense net to small separate rings) |
| S11 Source | decoder / oscillator | **Video** / **Osc**: the shell's drifting triangle lattice at twice the camera's level |
| P12 Twist | Rotation CV attenuverter | the gradient's depth, 0.5 p + p^2 turns from centre to corner: 0 % = a rigid diagonal lattice at the K1 angle; 50 % = half a turn; 100 % = 1.5 turns: a deep vortex whose outer cells are swept into long arcs (the climax) |

## Video

- **Video (default)**: camera luma, (Y >> 2) - 16, averaged over 8-pixel cells, slewed down the screen (one pole,
  1/64 per line at HD, 1/32 at progressive SD, 1/16 at interlaced SD; one 12-bit state per cell in a block RAM) and
  across (two one-pole filters, 1/16 per pixel), times K6. It goes to two jacks: the Navigator Rotation CV (bright
  areas turn the lattice locally, up to about 0.11 turn: the subject shows as a swirl in the weave) and the Visual
  Cortex key generator, AC-coupled: the field's average luma is the threshold (gain about 7), so everything darker
  than average keys the weave to black. With bear.png the bear's face reads in the weave and its dark body is a
  black silhouette; with selfie.jpg the bears and faces show as dark keyed shapes, edges stay clean curves.
- **Osc**: the shell's drifting 640 x 540 virtual-px triangle lattice on the same jack at twice the level: the
  vortex is bent into a quilt of diamond swirls and the key opens dark diamonds where the lattice is low.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks")**: shared glide (K1-K6, P12; 1/8 per field); per-mode
  constants from a block-RAM table indexed by the timing ID; 19 slots on one 36-bit serial multiplier (knob
  squares, the sway parabola, the twist coefficient Aq, the gradient DDA start values and differences, the ramp
  steps and starts with the CORDIC gain folded in, the Doorway offset, the camera mean = the cell-state sum x
  2^24 / cells, the key level), the laws as plain adds decoded by sub-step x slot.
- **Per line**: the gradient's y part Aq yq^2 by second differences (48 bits, in two 24-bit halves) or the linear
  V ramp; the V ramp coordinate.
- **Per pixel (53 stages)**: the gradient's x part by second differences + the line part + the base angle + the
  camera (stage 3); the rotator at 1/16 of the pixel rate (one 12-iteration CORDIC unit, add / subtract selected
  on the carry-in), linearly interpolated over 16 pixels by an accumulator (stage 33); Arch Mirror and logic
  (34, 35); Staircase / Doorway gain (36, 37); phase index / outline (38, 39); the palette ROM (1024 words:
  Staircase / Doorway x four baked key levels x 128 steps; the camera key level dithered along the line in a
  4-pixel cycle, which the output boxes average away exactly) (40 - 42); K5 hue turn (43 - 45); black off the
  generated avid (46); output conditioning as the shell: two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U, V with the
  per-line resync (47 - 50), luma 13/16 (51), vertical [1 1]/2 with the line above (52), blanking gate (53).
- **Contracts**: generated avid; one latency (C_LAT 52 = processing_delay_clks; the output register is stage 53,
  as in the shell) in every mode, the syncs and the
  late avid taps through block-RAM rings; virtual 1080-line coordinates (SD = the centre 4:3); colours in the
  hardware convention (U = Cr, V = Cb); drawn whites about 777.
- **Block RAM (21)**: palette ROM 8 (1024 x 32), per-mode table 2, camera cells 1, box delay rings 4 (2 x 16 x 32),
  vertical-mix line stores 4, sync ring 1, avid ring 1; every written RAM carries `no_rw_check` (0 transparency
  emulations in the standalone yosys log).

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI pin-skew screen: the built placement (router1 seed 1) is inside the window (HSYNC +1.35 ns, data <= -1.25 ns); kept.

- **v0.1.0 built 2026-11-25 with router1 (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1, not
  flashed.** Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt gyre` ->
  `out/rev_b/ron/onevolt/gyre.vmprog` (443,930 bytes). The default router2 stalled twice on HD HDMI (seeds 1 and 2,
  15-min timeouts under a loaded machine; an earlier router2 build of the netlist before the timing fixes routed
  every config on seed 1 at 74.4 - 87.1 MHz). dupin: 0 `CARRY I0==I1`, 0 duplicate LUT inputs.
- Post-route Fmax (the builder's last report per clock, PASS on the first seed):

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 89.77 | 6,670 |
  | SD Analog | 1 | 92.88 | 6,677 |
  | HD HDMI | 1 | 86.42 | 6,671 |
  | SD HDMI | 1 | 88.35 | 6,676 |
  | HD Dual | 1 | 87.75 | 6,682 |
  | SD Dual | 1 | 90.24 | 6,679 |

  6,670 - 6,682 / 7,680 LC (87 %), 21 / 32 EBR. iceunpack OK x6; the packed config carries the new labels and
  presets. No pin-skew screening.
- **Checks** (working files in `.onevolt_work/rb_gyre/`):
  - `check.sh` clean; `scan.py`: 0 integer-minus-signed sites; standalone yosys: 0 transparency emulations.
  - GHDL 720p60, decimation 1, black input, every control at its TOML default: **bit-exact with the integer model
    `imodel.py`** (0 of 917,444 compared pixels, rows 1 .. 719; the tester's capture sits 4 px left of the generated
    avid, as in every shell program). `rtl_default.png`.
  - The same at field 9 (`rtl_motion.png`): 0 differing pixels against the model at field 9; 893,755 pixels changed
    against the field-3 frame, so the patch moves exactly as the model moves it.
  - Both 480i fields (`--video-mode ntsc`, 720 x 243) and both 1080i fields (1920 x 540): 0 differing pixels against
    the model (rows 1 on). SD draws the centre 4:3 of the same picture; 1080i the same picture as 720p / 1080p.
  - Each switch flipped alone (S7, S8, S9, S10), P12 at 0 and 1023, K5 at 1023: bit-exact with the model except the
    first output column (those renders ran one revision earlier, before the output gate was moved to stage C_LAT + 1;
    the pixel data path is unchanged).
  - Source Osc (`rtl_osc.png`) and selfie.jpg (`rtl_selfie.png`) match the model's renders in structure, not bit for
    bit: the model's camera path is approximate (cell / slew start-up, the camera's pixel alignment).
  - Thin-line check (`thincheck.py`, model): 16 switch states x P12 0 / 50 / 100 % at 720p, 2 times each: 3 runs of
    1-row lines >= 24 px, all in S7 Ramp + S9 Absolute + S10 Doorway at P12 100 %; at 480i 4 and at 1080i 61 runs
    (field rows) in that same state; every other state clean.
  - `sheet.png` (each knob 0 / 25 / 50 / 75 / 100 %, K6 with bear.png, P12 0 / 50 / 100 %, S11 both ways, all 16
    S7-S10 combinations), `strip.png` (8 frames over 3.5 s), `camera.png` (selfie and bear at K6 0 / 25 / 50 / 100 %),
    `presets.png`: all from the integer model at 720p.
- **Fit**: the first RTL (per-field palette writer, full-rate rotator, multiplying key) packed to 8,542 LC (111 %);
  see notes.md "Fit rounds" for the four rounds of cuts to 6,685 (pack-only of the full design) / 6,670 - 6,682
  built.
- **Watch on hardware:** crisp moving colour edges at HD HDMI (the shell's conditioning is in); the camera key's
  threshold follows the picture's average luma field by field, so a scene cut moves the key at once.

## Cut / deviations

- **K5 is a hue turn of the three Color Chords colours, not a re-ordering of which output goes to which colour.**
  The first RTL had a per-field palette writer that re-mixed the layers for nine arrangements; with the camera key
  and the rotator it put the program at 111 % of the device. The fixed palette plus a chroma quarter-turn with
  crossfades costs a tenth of that and still gives four distinct colourings with smooth blends between them.
- **No density fade.** The spec asked for fading bands to their mean where the wound-up pattern gets finer than
  about 3 px. A measured-gradient fade was built and modelled; it darkened the outer swirls into grey smears and
  cost about 250 LC. Without it the dense outskirts read as fine colour hatching; the output boxes (9-px
  horizontal) and the vertical mix keep it from aliasing into noise, but at P12 100 % with K4 or K2 / K3 high the
  far corners show fine moire-like hatching rather than a flat mean.
- **The twist depth breathing (+-5 %, 19 s) is dropped** (one multiplier slot); the sway and the phase LFO remain.
- **The rotator runs at 1/16 of the pixel rate** with linear interpolation. Against a full-rate model the
  difference is invisible except in regions already denser than a band per 2-3 px.
- **The camera key is 4 baked levels, dithered** (effectively 13 levels after the output boxes) instead of a
  16-level multiply; **K6 has 32 steps** in the camera products (glided).
- **S7 Ramp + S9 Absolute + S10 Doorway at P12 100 %**: the bottom of the screen packs into horizontal outline
  lines a field row thin; the thin-line check counts 3 runs >= 24 px at 720p (61 at 1080i, field rows); every other
  switch state and P12 setting is clean.
- **K1 covers 0 .. 180 degrees**, one period of the mirrored lattice, so the ends meet (the spec's 0 .. 360 would
  repeat the picture twice).
- Doorway's palette is amber / magenta / white whatever K5 says before the hue turn (the turn recolours it).
