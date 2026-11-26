# Stairwell: LZX "Your First Patch" taken through Stairs

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 1. Two DSG3 Shape sections in series fold the Scrolls ramps into
one large figure, Stairs folds it into nested bands, and three adjacent Stairs outputs feed ESG3's R, G and B, so
each colour subdivides the one before it. Presets are hand-written placeholders.

## The rack

- **LZX Scrolls**: the two ramps (H, V) and their motion. Its Mirror waveform makes a ramp driven past 0..1 reflect, so
  a narrow figure repeats; its Ping Pong motion moves the coordinate origin.
- **LZX DSG3**, both Shape sections in series. Shape 1 has Fold (rectifier: ramp -> triangle, x2 / x4) and Curve on
  both inputs. Shape 2 adds two Shape 1 outputs together and its Average is taken.
- **LZX Stairs**: the luma-weighted R/G/B mixer (G = the shape, R = the camera), Steps (the pre-fold gain), Phase (the
  starting phase) and six doublers. Taps 2X .. 32X are used.
- **LZX Ribbons** (S10, in Stairs' place): a 3-bit flash ADC with Center and Span, and bit outputs D0 D1 D2.
- **LZX ESG3**: the encoder. Its three cables are R, G and B, through a fixed x4 contrast about 0.5 V with soft
  clipping, and the output clips at 1 V.
- **LZX TBC2**: the camera's luma.
- **LZX DWO3** (S11): a drifting triangle lattice patched to the same jack as the camera.

## The patch

    Scrolls ramp 1 (H) -> DSG3 Shape 1 H in            K1 Width = ramp 1 gain (Mirror waveform)
    Scrolls ramp 2 (V) -> DSG3 Shape 1 V in            K2 Height = ramp 2 gain
    Shape 1: Fold on both inputs (S9: Fold 1 / Fold 2), Curve on both (K4, made continuous)
    Shape 1 Average out (S7: Absolute out) -> Shape 2 H in
    Shape 1 Minimum out (S8: Maximum out)  -> Shape 2 V in
    Shape 2 Average out                    -> Stairs G in (the base shape)
    TBC2 Y out (S11: DWO3 triangle out)    -> Stairs R in, through K6
    Scrolls LFO (triangle, 9 s)            -> Stairs Phase CV; Scrolls Ping Pong (23 s / 17 s) on the ramps' origin
    Stairs: Steps <- P12, Phase <- K3; three outputs -> ESG3 R, G, B (K5: which three, ESG3 invert switches)
    S10 Ribbons: Shape 2 + camera -> Ribbons ADC in; Span <- P12, Center <- K3 (+ the LFO); D0 D1 D2 -> ESG3 R G B

## What it draws

At the defaults the frame holds one large octagonal figure: the average of a diamond pyramid and a rectangular
pyramid, with eight facets. It is cut into nested Gray-code colour bands. Red carries the broad bands, green splits
each in two and blue splits them again, so the bands run black, blue, cyan, green, yellow, white, magenta and red,
then back, with a black seam at each red period. The screen corners are flat black, where the fold has not begun.
The figure is bright at its centre.

Motion at the defaults:
- The LFO on the Phase makes the bands breathe in and out by about one band every 9 s.
- Scrolls' Ping Pong carries the figure 72 x 40 virtual px on 23 s / 17 s triangles.

The four S7 / S8 combinations draw four figures: octagon, plus, four-point star, and diamond lattice with a saltire.
S9 repeats the figure 2 x 2. S10 turns the soft triangle bands into hard Ribbons colour bars.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Width | Scrolls ramp 1 gain | Figure width from 0.38x the screen (about 2.6 figures across through the Mirror waveform) to 3.0x (one figure wider than the screen, near-horizontal bands). Default 1.07x. Exponential law, piecewise linear per octave. |
| K2 Height | Scrolls ramp 2 gain | Figure height from 0.37x (three rows) to 2.95x. Same law as K1. |
| K3 Phase | Stairs Phase / Ribbons Center | Starting phase from -0.30 to 0.75 V of the shape. Low: bands wall to wall. High: the black ground closes in on a small banded core. Ribbons Center = Phase + 0.35. |
| K4 Curve | DSG3 Curve, both inputs | Continuous crossfade of the two curves, c = t + k t (1 - t). Left: t^2 (exponential), pinched four-point stars. Centre: linear octagon. Right: 2t - t^2 (logarithmic), a rounded cushion. |
| K5 Colour | the three cables into ESG3 | Eight cable arrangements (R, G, B; - = ESG3 invert): 8X 8X 2X / 8X 4X 2X / 2X 4X 2X / **2X 4X 8X** (default, 438) / -2X 4X 8X / -2X 4X 32X / 16X 4X 32X / 16X -4X 32X. Neighbours differ in one cable. Between plateaus that cable dims to 0.5 V (ESG3 contrast) and comes back on its new source, so the knob never jumps. Inverted cables colour the black ground. |
| K6 Camera | attenuator into Stairs R | 0 % = the black-input picture exactly. 100 % (default) = white adds 0.53 V of shape: the bands become a contour map of the subject. |
| S7 H In | DSG3 Shape 1 **Average** / **Absolute** out into Shape 2 H | Diamond + rectangle (octagon) / saltire + rectangle (a plus of nested crosses). (Abs + Min) / 2 = Max / 2, taken at full scale. |
| S8 V In | DSG3 Shape 1 **Minimum** / **Maximum** out into Shape 2 V | With S7 Average: octagon / four-point star. With S7 Absolute: plus / offset diamonds with a saltire. |
| S9 Fold | DSG3 **Fold 1** / **Fold 2** | One figure / the figure repeated 2 x 2 (frequency x4). |
| S10 Folder | **Stairs** / **Ribbons** | Soft triangle bands in three taps / hard 3-bit colour bars: the eight pure colours in binary order (black, blue, green, cyan, red, magenta, yellow, white), white above the window. |
| S11 Source | **Video** (TBC2) / **Osc** (DWO3 lattice) | The camera / a drifting 640 x 540 virtual px triangle lattice at twice the camera's level: lobed, interlocking figures. |
| P12 Steps | Stairs Steps / Ribbons Span | Steps 0.25 .. 8 (5 octaves, default 600 = 1.93). At 0 % the figure is unfolded: one soft cyan / green glow. At 50 % about a dozen bands. At 100 %, dozens of fine nested octagons. Ribbons Span runs 2 .. 0.125: from a few broad bars to eight thin bars around Center. |

## Video

- **Video (default).** The TBC2 luma, (Y >> 2) - 16, is processed in four steps:
  1. summed over 16-pixel cells;
  2. slewed down the screen by a one-pole filter (1/64 per line, a 12-bit state per cell in a block RAM);
  3. slewed across by a one-pole filter (1/16 per pixel);
  4. multiplied by K6 x Steps.

  The result is added to the shape before the Phase, which is where Stairs' R input sums into its mixer. Bright parts
  of the picture push the bands outward and keep the black ground back. The subject reads as a contour map, and the
  slews keep its edges as curves.
- **Osc.** The shell's drifting triangle lattice takes the camera's place at twice its gain.

## Engine

**Per field** (vblank zone, "no avid for 1024 clocks"):
- The glide moves all seven knobs 1/8 per field.
- The knob laws are shift-add, piecewise-linear exponentials.
- Triangle LFOs drive Scrolls Ping Pong and the Phase.
- Three slots on one 32-bit serial multiplier compute S k, S Phase and S K6.
- A per-field band-density level per cable is computed analytically, from the exponents of Steps, the ramp-2 gain,
  the Curve and the Fold.

**Table writer.** A radix-16 serial multiplier, 13 clocks per entry, evaluates X = t (A - B t), which is Steps x
Curve(Fold(ramp)):
- the ramp phase is Q32 and wraps (Scrolls Mirror);
- t is the triangle of the phase, held 3/4 of an entry step off its peak and floor (a rectifier knee, so no fold
  apex draws a one-row line);
- it walks out from the centre entry, over x (8 virtual px per entry) and y (8 virtual lines per entry), into one
  512 x 32 block RAM (Q14).

**Per line.** In hblank the line engine sends the y entries through the pixel path's interpolation stages, which
interpolate the y function at the line.

**Per pixel** (19 stages):
1. interpolate the x function;
2. Shape 2's Average of Shape 1's Average / Absolute and Minimum / Maximum;
3. add the camera minus Steps x Phase;
4. five Stairs taps (triangles of the phase's bits; 0 below the starting phase) or the Ribbons bits;
5. per cable, select the source, apply the ESG3 invert, the K5 gain and the per-field density fade;
6. the ESG3 channel ROMs, with G one pixel and B two pixels behind R (path delay);
7. sum and clip at 1 V.

**Output.** The shell's conditioning: two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the per-line resync, luma
13/16, then the blanking gate.

**Contracts:**
- generated avid;
- one latency (C_LAT 24) in every mode;
- virtual 1080-line coordinates (SD shows the centre 4:3);
- colours in the hardware convention (U = Cr).

**Block RAM (14):** patch table 4, ESG3 ROMs 6, camera cells 1, box rings 2 or more (as the shell).

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI re-placed for the encoder pin skew: router2 seed 36 (HSYNC +2.11 ns, data <= -1.08 ns, 78.30 MHz); the builder's router2 seed 1 was outside the +1.0..+2.3 ns window. A plain rebuild gives the builder's placement again: re-run .heirloom_work/skew_tools/install.sh stairwell router2 36 hd_hdmi. Screen data: .heirloom_work/rb_stairwell/skew/.

- **Defaults changed by the orchestrator after the build (TOML only; the bitstreams are as built).** Phase 512 -> 256, Colour 438 -> 768, Steps 600 -> 760: the figure now fills the screen wall to wall, in broad colour bands each subdivided by fine ones (the builder's default was one six-band octagon on black; that frame is kept as rtl_default_builder.png in the work dir). rtl_default.png is a GHDL render at the new defaults. The sheets and the text below were made at the builder's defaults.

- **v0.1.0 built 2026-11-25 with router1** (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1, not
  flashed. Router2 stalled (overused=2) on HD Analog in an earlier netlist, so the final build went straight to router1.
  dupin on the synth json: no `CARRY I0==I1`, no duplicate LUT inputs.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt stairwell` gives
  `out/rev_b/ron/onevolt/stairwell.vmprog` (431,681 bytes). Plain `./build_set.sh onevolt stairwell` uses router2.
- Post-route Fmax (the builder's report, PASS on the first seed):

  | Config | Router / seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | router1 / 1 | 86.95 | 6,748 |
  | SD Analog | router1 / 1 | 87.32 | 6,750 |
  | HD HDMI | router1 / 1 | 87.37 | 6,750 |
  | SD HDMI | router1 / 1 | 86.34 | 6,750 |
  | HD Dual | router1 / 1 | 85.95 | 6,758 |
  | SD Dual | router1 / 1 | 92.89 | 6,753 |

- Totals: 6,748 - 6,758 / 7,680 LC (88 %), 14 / 32 EBR, 0 transparency emulations. iceunpack is OK on all six
  bitstreams, and the packed config carries the TOML's labels and presets.
- No pin-skew screen was run.
- **Fit:** the first RTL was ~8,850 LC (full design). See notes.md, "Fit rounds", for the cuts.
- **Timing:** the first closing attempt routed at 67.8 MHz. Its critical path was the per-field density sum and the
  multiplier operand negations, each done in one zone clock. They are now spread one add per clock, and the
  operands are pre-registered.
- **Checks** (working files in `.onevolt_work/rb_stairwell/`):
  - `check.sh` is clean, and `scan.py` reports 0 integer-minus-signed sites.
  - GHDL 720p60, decimation 1, black input, every control at its default: **bit-exact with the integer model
    `imodel.py`** (0 differing pixels; the tester's capture sits 4 px left of the generated avid, as on this shell).
    The frame is `rtl_default.png`.
  - Also bit-exact with the model (0 px), all rendered from the built `stairwell.vhd` (GHDL's recorded checksum):
    each switch flipped alone (S7, S8, S9, S10 Ribbons), P12 at 0 % and 100 %, K5 at 0 and 1023, one combination
    (S7 Absolute + S9 Fold 2 + S10 Ribbons, Curve 200, K5 900), a motion capture after 8 warmup fields (model field
    9), NTSC (decimation 1) and 1080i5994. The knob renders use a sim copy whose glides snap (`simprog/`, one line).
  - Motion: from field 2 to field 9, 79.6 % of the pixels change (max 197 codes) with the glides already settled, so
    the change is the Ping Pong drift and the Phase LFO alone.
  - SD and 1080i: the tester writes the two fields one above the other (`rtl_ntsc.png`, `rtl_1080i.png`);
    `rtl_ntsc_woven.png` and `rtl_1080i_woven.png` interleave them. 1080i draws the 720p picture. NTSC draws its
    centre 4:3, so the octagon's left and right points fall off the screen at SD. The program does not read field_n
    (it only passes it through), so the hardware's field_n tied high cannot change the picture.
  - S11 Osc and the camera are not bit-exact with the model (the lattice start-up and the camera cells are not
    modelled cycle-exactly: Osc differs on 99 % of the pixels). They are judged by eye: `rtl_osc.png` (lobed,
    interlocking figures), `rtl_selfie.png` and `rtl_bear.png` (the subject as a contour map; some line-to-line
    jitter on the selfie's gravel).
  - Thin-line check on the model: no one-row runs of 24 px or more, over 16 switch states x P12 0 / 50 / 59 / 100 %
    x 2 times, at 720p and at 480i.
  - `sheet.png`, `sheet16.png`, `strip.png`, `camera.png` and `presets.png` come from the integer model.

## Cut / deviations

- **K5 transitions dim and re-plug a cable** rather than crossfading two sources. A full A/B blend per cable cost
  ~450 LC.
- **The band-density fade is per field and vertical only.** It is computed analytically from Steps, ramp-2 gain,
  Curve and Fold, not per pixel from the gradient (~530 LC). Fine horizontal bands are left to the output box filter.
  At SD (480i rows are 4.4 virtual lines) the finest taps fade earlier than at HD.
- **Motion:**
  - Scrolls runs in **Ping Pong** (triangles, 23 s / 17 s) instead of Orbital sines.
  - The LFO on the Phase is a triangle.
  - The third modulator, the 31 s lean on the Curve, was cut for LC.
- **Abs + Min is taken at full scale** (the real Shape 2 Average gives Max / 2), so Phase keeps its whole travel.
- **Every fold apex and valley is flat for about 1.5 table entries** (12 virtual lines), a rectifier knee, so no crest
  draws a single-row line.
- **The camera filter is the brief's minimum:** 16-px cells, one vertical pole (1/64), one horizontal slew (1/16).
  Mild line-to-line contour jitter remains on heavily textured video (the selfie's gravel) at K6 100 %.
- **No vertical [1 1] mix.** The lozenge shell has none, and the thin-line check passes without it.
- **ESG3 luma hard-clips at 1 V** (yellow and white share luma). There is no soft knee.
- **Widths in virtual px.** K1 / K2 count repeats on the 1920-virtual-px canvas, so SD shows the centre 4:3 of the
  same picture.
