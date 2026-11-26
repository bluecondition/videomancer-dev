# Ondine: the Syntonie oscillator voice

**v0.1.0 built 2026-11-25, not flashed.** Onevolt 3. Two Ondes oscillators, shaped by Formes, combined by Combines,
driven through Cadrans into Solaire's rectifier chain and coloured by Matrice. The picture is a lattice of chevron
cells, hinged at the left edge and fanned by FM, with alternate rows flipped by AM and a nest of rectifier rings in
every cell. Presets are hand-written placeholders.

## The rack

| Maker | Module | Its job here |
|---|---|---|
| Syntonie | Ondes, oscillator A (SYNC H) | the vertical bars: hard reset at every line start, FM from Ondes B and the camera, AM from Ondes B, NORM switch (S7) |
| Syntonie | Ondes, oscillator B (SYNC V) | the horizontal bars; its linear out feeds A's FM, A's AM and Solaire's injection |
| Syntonie | Formes, ch 1 and ch 2 | sine or trapezoid outputs of A and B (S8) |
| Syntonie | Combines, ch 1 | AVG or XOR of the two shaped oscillators (S9): diamonds or X cells |
| Syntonie | Cadrans, ch 1 and ch 2 | the Fold gain (P12): the drive into Solaire, and (multed) the level injected between the folds |
| Syntonie | Solaire | four full-wave rectifiers with three mixers, which inject Ondes B (plus Animate and the camera) between the folds; mixer switches ADD / SUB (S10) |
| Syntonie | Matrice | three taps (Combines out, rectifier 2, rectifier 4) to R, G, B: eight designed crosspoint settings, blended (K5) |
| Syntonie | Animate | a quadrature LFO (triangle outputs, 13 s): 0 degrees leans Ondes B's frequency, 90 degrees scrolls the injected level |
| Syntonie | Entree | the camera decoder; its level (K6) goes to A's FM and to Solaire's injection |
| Syntonie | Sortie | the encoder (clip to legal, the shell's output conditioning) |

## The patch

    Ondes B linear --> FM A attenuverter (K3) --+
    Entree Y (K6) ----------------------------+--> Ondes A FM in        Ondes A: SYNC H, FREQ A (K1), NORM A (S7)
    Ondes B linear --> attenuverter (K4) -------> Ondes A AM in (four-quadrant: full, null, inverted)
    Animate 0 deg -----------------------------> Ondes B FM in          Ondes B: SYNC V, FREQ B (K2)
    Ondes A linear --> Formes ch 1 \
    Ondes B linear --> Formes ch 2 -> Sine or Trapezoid (S8) -> Combines ch 1 A, B -> AVG or XOR (S9)
    Combines --> Cadrans ch 1 (P12) --> Solaire rectifier 1
    Ondes B linear (AC) + Animate 90 deg --> Cadrans ch 2 (P12) --+
    Entree Y (K6) -------------------------------------------------+--> Solaire mixer B inputs 1, 2, 3 (ADD / SUB, S10)
    Combines out, rectifier 2 out, rectifier 4 out --> Matrice (K5) --> Sortie R, G, B

Matrice reads rectifier 2 one pixel later and rectifier 4 two pixels later (path delay through the extra modules),
which gives a faint colour fringe on vertical edges.

## What it draws

At the defaults: six rows of chevron and heart-shaped cells across the screen, three warm (yellow and orange cores
with green rings and dark outlines) alternating with three cool (white and cyan with blue rings), on a crimson
ground. The columns start wide at the left edge, where oscillator A resets, and fan out to the right, tighter on
the rows where Ondes B is high (FM). The cool rows are the AM-inverted ones. Every cell carries a nest of rings
from the rectifier chain.

Motion at the defaults: Animate's 0-degree output leans Ondes B's frequency, so the rows breathe (about 4 % over
13 s). Its 90-degree output scrolls the level injected between the folds, so the rings flow in and out of the cells
and the ring colours cycle.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Freq A | Ondes FREQ A | 1 .. 19.8 cycles across the width (exponential, 4.3 octaves), continuous; a partial last cycle at the right edge. Default 560 is about 5 |
| K2 Freq B | Ondes FREQ B | 1 .. 11.3 cycles down (exponential, 3.5 octaves). Default 455 is 3 (six rows of cells) |
| K3 FM A | Ondes FM A attenuverter | bipolar, square law, ±8 deadband: B's level swings A's frequency by up to ±10 cycles across, so the columns fan and wave, hinged at the left edge. Centre gives a rigid lattice; the left side fans the other way. Default 772 is ±2.5 cycles |
| K4 AM A | attenuverter into Ondes A's AM | bipolar, ±8 deadband: towards 1023, B fades A through the null and into inversion on B's high rows (a plaid); towards 0, the same on B's low rows. Default 960 is near-full inversion |
| K5 Colour | Matrice | eight designed crosspoint settings (the three taps assigned to R, G, B with inverts), each held for 60 % of its span and blended over the other 40 % |
| K6 Camera | Entree level | camera luma into A's FM (the columns crowd over bright areas, where Ondes' bandwidth fades A to its mean) and into Solaire's injection (the fold contours trace the picture). 0 % gives exactly the black-input picture. Default 100 % |
| S7 Norm A | Ondes NORM A | +1V (triangle) / SQR (A's own square into its AM: a saw at twice the rate, so twice the columns and one-sided ramp cells) |
| S8 Formes | Formes outputs | Sine (round, soft cells) / Trapezoid (gain 4 and clip: flat plateaus with steep sides, boxy rings with hard shoulders) |
| S9 Combines | Combines output | AVG (heart and diamond cells) / XOR (\|A - B\|: X cells, light ground) |
| S10 Solaire | Solaire mixer switches | ADD / SUB: the injected oscillator adds to or subtracts from the signal between folds (darker, wine-and-cyan rings) |
| S11 Source | Entree / Osc | Video (the camera) / Osc: a drifting triangle lattice fed at twice the camera's gain into the same two jacks, giving ringed diamonds over the whole screen |
| P12 Fold | Cadrans gain into Solaire | x1/16 .. x1 (exponential, glided). 0 % is the unfolded lattice (soft green and white cells, every other control alive). 100 % drives all four rectifiers fully, so each cell becomes a tight nest of rings (the climax). Over-fine rings fade to their mean |

All continuous knobs and P12 are glided (1/8 of the gap per field).

## Video

The camera (Entree) is patched into two jacks: Ondes A's FM input and Solaire's mixer B inputs. Luma is averaged over
8-pixel cells, slewed down the screen (one pole, 1/16 per line, a 12-bit state per cell in a block RAM) and across
(two one-pole filters, 1/16 per pixel), scaled by K6 once per cell, then added to A's step and to the injected level.
A's bandwidth limit fades A's oscillation where the camera pushes it past about 1/32 cycle per pixel. With bear.png
at K6 100 % the head, ears, eyes and snout read as clean fold contours. Osc feeds the shell's drifting triangle
lattice into the same path at twice the gain.

## Engine

- **Per field (vblank zone, "no avid for 1024 clocks").** Glide on 7 channels; Animate's phase; knob decodes with
  shift-add laws for Freq A, Freq B, AM depth and the Colour position. Nine slots on one 32-bit serial multiplier:
  step A, Freq B, B's line step, FM depth (square law), the injected level, the camera gain, the fold count and the
  ring fade. Then the **table writer** (zone clocks 400 .. ~3140): for each Matrice tap and channel it makes one
  crosspoint blend (ROM reads plus a product on the field multiplier), then writes a 256-entry colour table,
  T(v) = x · cv(v) / 256 with the contrast curve cv = clip(2v - 128, 0, 254). This is exact with one accumulator lane and
  no multiplier. Nine tables, then the three offsets. A line-0 kick at zone clock 3200.
- **Per line.** A line engine is kicked at each line start and computes the next line: Ondes B's phase (a DDA) and
  triangle; the AM gain; the FM step; the injected level (three products on the field multiplier, which is free
  during active video); Formes B through the sine ROM in hblank. Its values load 20 clocks after the generated avid
  falls.
- **Per pixel (22 stages to the compose, C_LAT 28).** Ondes A's phase is accumulated from 0 at the line start by the
  camera stream's pixel step. Then: the triangle or saw; the bandwidth; AM on a radix-4 Booth multiplier; the
  bandwidth shift-adds; Formes (sine ROM or trapezoid); Combines; the Fold gain (Booth); four rectifiers with three
  injections; ring fades; three table addresses with path delay; nine table reads; the Matrice sums; the encoder clip.
- **Output.** As the shell: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma
  13/16, a vertical [1 1]/2 with the line above, and the blanking gate.
- **Contracts.** Generated avid; one latency (28) in every mode; virtual-1080 coordinates (A's count spans the width,
  B's count spans 1080 virtual lines); colours in the hardware convention (U = Cr); drawn whites about 777.
- **Block RAM (22).** Colour tables 9, sine ROM 2, exp ROM 1, colouring ROM 1, luma cells 1, box delay rings 4, line
  stores 4. Every written RAM carries `no_rw_check`.

## Status

- **Pin-skew placement (orchestrator, after the build).** HD HDMI pin-skew screen: NO placement in the +1.0..+2.3 ns window was found (NONE router2 s1 hs +0.75 data -1.25); the builder's placement is shipped. See .onevolt_work/skew/ondine_rank.txt.

- **v0.1.0 built 2026-11-25, router2 (the build default), all six configs on seed 1, not flashed.**
  Rebuild: `./build_set.sh onevolt ondine`, giving `out/rev_b/ron/onevolt/ondine.vmprog` (465,424 bytes).

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 79.09 | 6,937 |
  | SD Analog | 1 | 80.61 | 6,943 |
  | HD HDMI | 1 | 79.42 | 6,939 |
  | SD HDMI | 1 | 83.25 | 6,934 |
  | HD Dual | 1 | 78.46 | 6,946 |
  | SD Dual | 1 | 77.52 | 6,959 |

  6,934 - 6,959 / 7,680 LC (90.6 %). That is about 200 LC over the set's 6,750 target; see Cut / deviations.
  22 / 32 EBR. iceunpack OK on all six; the packed config carries the labels and presets. No pin-skew screening.
- **Checks** (working files in `.onevolt_work/rb_ondine/`):
  - `check.sh` clean; `scan.py`: 0 integer-minus-signed sites.
  - GHDL 720p60, decimation 1, black input, every control at its default: **bit-exact with the integer model
    `imodel.py`** (0 of 917,444 compared pixels differ, rows 1 on; the tester's capture sits 4 px left of the
    generated avid). This is `rtl_default.png`.
  - S7, S8, S9 and S10 each flipped alone, P12 at 0 and 1023, K5 at 1023 and K5 at 200 (mid-blend): **all 0
    differing pixels** against the model (rendered from a sim-only copy without the knob glide).
  - One 480i field (`--video-mode ntsc`) and one 1080i field: **0 differing pixels** against the model.
  - A later default frame (field 9): **0 differing pixels** against the model at field 9; 889,740 pixels changed
    against the field-3 frame, so the patch moves exactly as the model moves it (`rtl_motion.png`).
  - Osc and camera frames (`rtl_osc.png`, `rtl_selfie.png`, `rtl_bear.png`) match the model's structure, but not
    bit for bit. The RTL takes the camera's injection and the fades from the camera stream, a few pixels apart.
  - `thincheck.py` on the model: no single-row hard runs of 24 px or more at the defaults, at P12 0 % or at P12
    100 %. In 8 of the 32 switch / P12 cases there are 1 to 3 runs of 24 - 36 px, at chevron tips on B's turnaround
    rows.
  - `sheet.png` (knob sweeps, switch flips, the 16 switch combinations), `strip.png`, `camera.png`: all from the
    integer model.

## Cut / deviations

- **Over budget by about 200 LC (90.6 %).** Five fit rounds took the standalone pack-only count from 8,485 to 5,563.
  These rounds:
  - removed the table writer's multipliers;
  - used Booth for the per-pixel multiplies;
  - moved the camera product to cell rate;
  - put the line engine's and the writer's products on the shared field multiplier;
  - used one writer lane with nine tables;
  - merged pipeline stages.

  Getting further would mean cutting a specified module. Candidates are the Formes trapezoid (S8) or Ondes A's
  bandwidth limit, which is what keeps camera regions clean.
- **Cut: the third modulator (VU006 sway, 29 s).** The motion is Animate's two outputs: 0 degrees leans B's
  frequency, 90 degrees scrolls the injection.
- **Animate's period** is 13 s for both outputs (a quadrature pair can't be 17 s and 11 s), and it uses the triangle
  outputs.
- **S10 is ADD / SUB, not AVG / SUB.** In the proto, AVG halved the injected signal and the folds collapsed into a
  flat purple.
- **The camera also goes into Solaire's injection.** This is a second cable from Entree. FM alone did not make the
  subject recognisable.
- **Ondes A's bandwidth limit** (amplitude fades between 1/32 and 1/16 cycle per pixel) is added. It stands for the
  oscillator's finite top frequency. It is set in output pixels, so SD fades the camera's FM earlier than HD.
- **Ring fades are per field, from the field's fastest row,** not per line. Slow rows fade a little early at a high
  Freq A and P12.
- **Ranges:** Freq A runs 1 .. 19.8 cycles and Freq B 1 .. 11.3 cycles (the spec asked for 20 and 12).
- **Matrice settings are pure assignments with inverts,** not partial crosspoints, plus a x2 contrast curve on each
  tap.
- **SD keeps the counts across the 4:3 width,** so the cells are 25 % narrower. SD is softer (the output boxes act
  per output pixel). 720p and 1080p give the same picture.
- Row 0 of each field mixes with the last row of the field before, because the shell's vertical-mix line store is
  not cleared.
