# Overshoot: horizontal-only filters, lit edges with spectral comet tails

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 5. A field of hard-keyed flat rhombi turns slowly. The figures
themselves are hidden. Three horizontal high-passes show only what the beam sees crossing their edges: a white
head at every edge, then a tail that runs right through yellow and orange to deep red. Edges that lie close to
horizontal give nothing. The scan direction is the subject. Presets are hand-written placeholders.

## The rack

- **LZX Navigator**: rotates the H and V ramps (K2 Rotation, plus an internal 50 s sway of +-25 degrees).
- **LZX DSG3**: folds the rotated H ramp once and the V ramp twice (a 2:1 cell). The Average output gives
  rhombi; S9 moves the cable to the Maximum output, which gives bricks.
- **LZX Keychain** (ch 1): the comparator that keys the DSG3 output (K1 Size = Thresh, a 9 s LFO). The camera goes
  into its CV input.
- **LZX DWO3** (S10): a triangle core in the Locked Horizontal range, hard reset at every line. A V-range sine
  (free running, 7 s roll) and the Navigator's H output FM it. It replaces DSG3 at the Keychain input.
- **LZX Contour**: three high-pass filters with the input normalled to all three. P12 Width sets the master cutoff
  and K3 Spread the ratio, with a 23 s lean on the spread.
- **Syntonie Detail** (S7): Blur mode in Contour's place (the same filters' low-pass outputs).
- **LZX Curtain**: the rectifier (S8 Shape: Both edges / Signed on a pedestal) and its Mix section (K4: the dry key
  added to the filter outputs).
- **LZX SMX3**: which filter feeds R, G and B, crossfading between neighbours along an 8-colouring path (K5).
- **LZX TBC2**: the camera decoder. Luma goes through an 8-px box, a vertical slew and a horizontal slew.
- **Encoder**: soft clip (knee at 3/4) and RGB to YUV.

## The patch

1. Ramp generator H, V -> Navigator H in, V in.
2. Navigator H out -> DSG3 shape 1 H in (Fold 1); Navigator V out -> DSG3 shape 1 V in (Fold 2).
3. DSG3 Average (S9: Maximum) -> Keychain key in 1. (S10: DWO3 triangle -> Keychain key in 1 instead; with S9
   on, the DWO3 goes into DSG3's H input and the Maximum output keys it: bars cut into blocks.)
4. Navigator H out (mult) -> DWO3 FM in; DWO3 #2 (V range sine) -> DWO3 FM in.
5. Camera (TBC2 luma, slewed) -> Keychain CV in (inverted: bright areas shrink the figures) and -> the mixer with
   the key. S11 swaps in the Osc.
6. Keychain out 1 + camera -> Contour filter in 1 (normalled to 2 and 3). S7: -> Detail (Blur) instead.
7. Contour out 1, 2, 3 -> Curtain rectifiers (S8). Curtain Mix In <- Keychain out 1 (K4). -> gain (1, 1.25,
   1.5: the longer channels louder) -> SMX3 columns A, B, C.
8. SMX3 rows -> encoder R, G, B.

## What it draws

At the defaults the screen is a field of rhombi about 300 virtual px wide in rows. The head on each left
(entry) flank is a white hairline. Behind it the inside of the figure fills with a tail running yellow ->
orange -> deep red. From each right (exit) flank a second comet streams out to the right into the black. The
top and bottom edges lie near horizontal and stay dark, so black lanes cross the screen between the rows. The
three colour channels leave the filters 0, 1 and 2 px apart (path delay), a faint fringe on every head.

Motion at the defaults:
- The Navigator sway turns the field +-25 degrees over 50 s. Edges flare as they turn toward vertical and go
  dark as they turn toward horizontal.
- The 9 s Keychain LFO breathes the figures' size.
- The 23 s lean stretches and shrinks the colour spread of the tails.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Size | Keychain Thresh | figure size 0.08 .. 0.47 of the cell: small islands (0) .. nearly touching (1023); default 820 |
| K2 Rotation | Navigator Rotation | 0 .. 360 deg, ends meet; the 2:1 cell makes 0 / 90 / 180 / 270 deg four pictures (flat / tall rhombi, shifted); default 48 (17 deg) |
| K3 Spread | Contour cutoff ratio | 1 : 1 : 1 (white tails, 0) .. 1 : 4 : 16 (long chromatic tails, 1023); default 720 |
| K4 Mix | Curtain Mix level | the dry key into the three filter outputs (short channel loudest): 0 = edges and tails on black .. full = figures lit in the colouring's complement (Signed: sculpted relief); default 0 |
| K5 Colour | SMX3 | 8 routings blended along the travel: Fire, Lime, Ice, Violet, Orchid, Rose (magenta heads), Gold (yellow heads), Teal (cyan heads) |
| K6 Camera | camera level (TBC2) | into the filter input (x1.28) and the Keychain CV (0.43): 0 = the black-input picture exactly; 100 % (default) = the subject's edges light and trail, and the figures shrink over bright areas |
| S7 Filter | Contour / Syntonie Detail (Blur) | high-pass: edges and tails on black / low-pass: the figures themselves, smeared right, blue leading edges and red trailing glows |
| S8 Shape | Curtain rectifier | Both (every edge a bright head) / Signed (unrectified on a per-channel pedestal: left flanks bright, right flanks dark, an amber-ground bas-relief) |
| S9 DSG3 | DSG3 output jack | Average (rhombi) / Maximum (bricks: rectangles whose vertical sides light) |
| S10 Source | DSG3 / DWO3 | Figures / Bars: a curtain of wavy vertical edges hinged at the left border, fanned by the Navigator, each with its tail |
| S11 Input | TBC2 / Osc | Video / Osc |
| P12 Width | Contour master cutoff (Curtain Width) | exponential, tau 4 .. 400 virtual px (1:100): 0 % = white hairline outlines, almost no tail; 50 % = tails ~1/9 of the screen; 100 % = every edge drags a streak to the next, layered spectral smear with the heads still the brightest (input gain eases 2.0 -> 1.5 over the top half) |

## Video

- **Video (default).** Camera luma, (Y >> 2) - 16, is processed in this order: an 8-pixel cell average, a
  vertical slew down the screen (one pole, 1/16 per line, one 12-bit state per cell in a block RAM), then a
  horizontal slew (1/8 per pixel). The result is scaled by K6 and patched to two jacks:
  - the Contour input, beside the key (the subject's vertical edges light and trail: a side-lit drawing);
  - the Keychain CV, inverted (the figures shrink where the picture is bright, so a dark subject keeps its
    figures).

  The integrators pre-charge to the camera's first sample on each line, so a bright picture makes no streak
  from the left border. With bear.png at 100 % the bear's ears, eyes and nose read clearly. The figures stay
  clean curves.
- **Osc.** The shell's drifting 640 x 540 px triangle lattice, bipolar about its mean, replaces the camera. It
  goes into the Keychain CV at twice the camera's gain, growing the figures, which merge into a crystal field in
  places, and into the filter at 1/4. The flip is unmistakable at the defaults.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").**
  - Glide (K1, K2, K5, K6, P12; 1/8 per field). K3 and K4 are read directly: their pot noise is invisible.
  - Sine ROM reads: the sway, then cos and sin of the angle.
  - 13 slots on ONE 32-bit serial multiplier, with the constant operand from a ROM indexed by slot and timing
    id. They give the Navigator's per-pixel and per-line steps, the line-0 start values (summed in place), the
    DWO3's base frequency, per-line FM step and the Osc bias.
  - Plain adds for the threshold, the cutoffs (3/64-octave units), the gain and the Curtain Mix levels.
  - K5 from a 256-entry ROM.
  - Then a kick of the line engine for line 0.
- **Per line.**
  - The rotated ramps' start values advance by their line steps.
  - The DWO3's next-line frequency = fA + j fB + depth x sin(V phase): the sine from the ROM, the depth by
    shifts.
- **Per pixel (28 stages).**
  1. Stage 1: the Navigator accumulators u, v (2^21 per fold cycle) and the DWO3 phase (2^20).
  2. The folds, then Average or Maximum.
  3. The key with a 1-px coverage edge (gain 2^5 at 1080, 2^4 at 720p / SD).
  4. Plus the camera; minus the camera's first sample; the input gain (2.0, easing to 1.5 over P12's top half in
     single-shift steps; Blur x 1).
  5. The bank: integrators 2^-3, 2^-6, 2^-9 held at zero off the generated avid (the per-line reset and
     resync), plus a one-pixel delay.
  6. Each channel takes the low-pass BEFORE the pixel from two adjacent members and crossfades them (4-bit), so
     heads stay full height at every cutoff. hp = y - lp; lp + the first sample for Blur.
  7. The rectifier, then Curtain Mix + gain, clamped at 511; the 0 / 1 / 2 px path delay.
  8. SMX3: each row crossfades two neighbouring sources (3-bit).
  9. Soft clip; Y' = (5R + 9G + 2B) / 16; Y, U (Cr), V (Cb).
  10. Black off the generated avid.
- **Output.** Chroma through one [1 2 2 2 2 2 2 2 1]/16 box with the per-line resync. Luma through [1 2 1]
  centred on the same pixel (read from a second 16-entry ring). Then luma 13/16 and the blanking gate.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 28 = processing_delay_clks, in every mode. The sync delay is a 256 x 4 block-RAM ring
    read 27 back, exactly the shift registers' delay.
  - Virtual 1080-line coordinates. Tails are in virtual px: the cutoffs are offset by log2 of the pixel size.
  - Colours in the hardware convention (U = Cr).
  - Drawn white is ~776.
- **Block RAM (12).** Sine 4, constants 2, K5 1, camera cells 1, box ring 2, luma ring 1, sync ring 1. Every
  written RAM carries `no_rw_check` (0 transparency emulations in the yosys log).

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI pin-skew screen: the built placement (router1 seed 1) is inside the window (HSYNC +1.14 ns, data <= -1.24 ns); kept.

- **v0.1.0 built 2026-11-25 with router1** (`NEXTPNR_ROUTER="--router router1"`): all six configs on **seed 1**.
  Not flashed. Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh onevolt overshoot`. Plain
  `./build_set.sh onevolt overshoot` (router2) stalled once on HD HDMI (overused=2) with this netlist.
  Output: `out/rev_b/ron/onevolt/overshoot.vmprog` (441,898 bytes).

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 88.59 | 6,858 |
  | SD Analog | 1 | 92.76 | 6,865 |
  | HD HDMI | 1 | 92.00 | 6,863 |
  | SD HDMI | 1 | 94.94 | 6,858 |
  | HD Dual | 1 | 101.68 | 6,877 |
  | SD Dual | 1 | 96.75 | 6,882 |

  6,858 - 6,882 / 7,680 LC (89 %, about 120 over the 6,750 target), 12 / 32 EBR. iceunpack OK x6. The packed
  config carries the new labels and presets. dupin on the program netlist: 0 CARRY I0==I1, 0 duplicate LUT
  inputs. No pin-skew screening.

**Checks** (working files in `.onevolt_work/rb_overshoot/`):
- `check.sh` is clean, and `scan.py` reports 0 sites.
- GHDL renders at 720p60, decimation 1, from a sim-only copy with the glide bypassed (`simprog/`). At the
  defaults it is identical to the shipped RTL, because the glide starts at the defaults.
  - Black input, every control at its default: **bit-exact with the integer model `imodel.py`** (0 of 917,504
    pixels on rows 1 .. 719 differ, Y, U and V; the capture sits 4 px left of the generated avid).
    `rtl_default.png`.
  - With the same check (bit-exact against the model of the same source version): S7 Blur, S8 Signed, S9
    Maximum, S10 Bars (the DWO3 line engine and ROM sine), P12 0 and P12 1023 (the gain taper): each 0
    differing pixels.
  - S11 Osc and the camera (selfie) agree with the model in structure only. The model's cell, slew and lattice
    timing is approximate, giving a few-px offset (`osc_cmp.png`, `cam_cmp.png`).
  - A later default frame (field 10): 0 differing pixels against the model; 611,632 pixels changed against the
    field-3 frame (the sway and the LFOs move it as modelled).
  - Both 480i fields (720 x 243, `--video-mode ntsc`) and both 1080i fields (1920 x 540, `--video-mode
    1080i5994`): **0 differing pixels** against the model (rows 1 on). SD draws the centre 4:3 of the virtual
    canvas with the same figure size relative to the screen height; 1080i is the 1080p picture.
- `thincheck.py` on the final model (720p; default, P12 0 / 100 %, all 16 S7-S10 states, Osc, K1 0 / 1023, K2
  256; 3 times each; any 1-row bright or dark line of 24 px or more): no runs, except one 25-px run in Osc mode
  at one of the three times. The check passes without the vertical mix.
- `sheet.png`, `strip.png` and `camera.png` all come from the integer model.

## Cut / deviations

- **S9 is DSG3's Average / Maximum jack, not Syntonie Detail's negative resonance.** The resonance (a feedback
  comb, 4-5 echo lines per edge) was built and rendered. Fit cost: it took ~270 LC, and the whole rack measured
  8,190 LC standalone at first (about 9,600 with the core). The core adds ~1,400 LC, measured on separatrix and
  flyback. To reach the budget the Detail resonance went, and S9 became a cable move on DSG3: rhombi to bricks.
- **K4 is Curtain's Mix level (the dry key added to the three filter outputs), not a separate SMX3 column.**
  The short channel is loudest, so the fill takes the colouring's complement (Fire: pink-mauve). This is
  cheaper than the per-row column it replaced.
- **K5 has no polarity (negative SMX3 gains).** Each row takes one source or crossfades two neighbours
  (including "off"), along 8 designed routings.
- **Filter bank:** members 3 octaves apart (2^-3, 2^-6, 2^-9) with a 4-bit crossfade. A crossfade between
  exponentials that far apart is not exactly scale-invariant, so SD's tails come out slightly more amber than
  720p / 1080p (the red stage is shorter). Shapes and tail lengths in screen terms match; SD shows the centre 4:3
  of the virtual canvas (bigger figures), and its chroma box is twice as wide in screen terms.
- **Output conditioning differs from the shell:**
  - luma [1 2 1] instead of the two 9-tap boxes (the heads are hairlines);
  - chroma one 9-tap box instead of two (wrinkle's HW-validated chroma fix);
  - no vertical mix with the line above (the thin-line check passes without it);
  - all three freed LC and 5 EBR.
- The DWO3's FM by the Navigator is per line (the bars fan from the left border); the per-pixel chirp term was
  cut for fit.
- **S8 Signed = pedestal + half the filter output** (Both = |x| at full scale). The relief gain is lower, to keep the
  rectifier stage one adder deep: it was the HD critical path (74.4 -> 88.6 MHz after the change).
- The camera's two jacks are fed from the same slewed signal 3 px apart (the filter term is taken later in
  the pipe).
- Figures cut by the left border get a head at the border: the key starts from blanking, as Contour would see
  it. The camera is pre-charged, so a camera picture gets no head there.
