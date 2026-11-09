# Gable — a fan of pointed cyan chevrons torn into rolling strokes

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230129_23_23_56` at 0.5 s
(`photos/vid/WIN_20230129_23_23_56_t0p5.png`). A gable of nested pointed cyan chevrons on black, apex top centre,
every band torn into short horizontal strokes. Red hairlines cross the top and thicken going down into red flanks
left and right. Low in the gable sits a red / white core, with a dark arch at the bottom centre. In the clip the
stroke pattern rolls down (69 lines/s) while the chevrons stand still. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_gable/`: `notes.md`, `fit1.py`–`fit3.py`, `prof.py`, `align.py`,
`wfit*.py`, `motion*.py`, `proto.py`, `imodel.py`, `mkpy.py`, `cmpstat.py`, `runstat.py`, `thin2.py`, `sheet.py`.
- **Two channels.** The picture is a red channel and a cyan channel (G = B, B = 0.89 G).
  - Palette (k = 12): black (12,10,6) 28 %; reds (57,10,7) 8.7 %, (107,10,8) 8.0 %, (164,15,14) 4.1 %;
    teal to cyan (10,43,31) .. (14,187,172) 7.9 % .. (13,223,211) 6.6 %; white-pink (147,191,181) 3.8 % where both
    channels are lit.
- **The field: a chevron fan.** `phi = y − (0.62 + 0.00048 y)·|x − 1022|`.
  - This is a V ramp minus an H triangle whose level (a VCA) rises with the V ramp, so the lower chevrons are
    steeper and every chevron arches.
  - Fitted by the correlation ratio of the smoothed channels per field bin: 1.150, against 1.108 for a plain
    chevron and 1.109 for a quadratic.
  - The mirror axis is x = 1022 by flip-correlation (red reads 1034).
- **The colouriser over phi (tear-aligned).**
  - Outer dashed cyan rings at phi −568, −524, −468, −348, −308, −284, −180, −148 and −104.
  - Dark from −84 to 12. The gable rim at 52–92 (lit 0.5), 100–212 (0.35), rim B at 220–300 (0.9), 308–420 (0.5),
    and the inner gable at 428–520 (0.95).
  - The red / white core at 560–960. The dark arch at 960–1000 is another chevron band.
- **Red is a line PWM.** It locks to a 52.6-row stroke period in every 120-px slab (lock 0.35..0.47).
  - The hairlines sit at rows 76, 126, 178, 230, 284 and 334, and grow from 4 to 12 rows going down.
  - Red duty depends on the row as well as phi: 0.10 above y 270, 0.5–0.6 at y 450–780, 0.3 below. So the red key's
    threshold gets a vertical offset vr(y).
- **Red and cyan mix three ways.** They are exclusive for phi < 100 (P(cyan | red) = 0), independent for 100–780,
  and cyan follows red (white streaks) for 780–980.
- **Tear.** Each run of rows gets its own phi offset; left and right halves move together (0.73), so it is a field
  shift, not an x shift.
  - 42 % of rows sit within ±4, and 20 % beyond 20. Runs have a median of 3 rows.
  - There is a systematic dip of −29 near the red gap phase.
- **The texture is set by the field and the stroke phase.** Along constant-phi lines it repeats every 52 rows:
  correlation 0.757, against 0.45–0.5 at other lags.
  - Strokes: horizontal runs with a median of 46 px (rim zone) and 21 px (outer rings), vertical runs with a median of
    7 rows.
  - Edges go 10–90 % in 14 px horizontally and 2–3 rows vertically.
  - The core's streaks have a period of 72–80 px.

## Motion in the clip
- **The chevrons never move.** The axis stays at 1020–1024 for the whole 8.5 s, with no x drift.
- **The stroke pattern rolls down.** The hairline phase moves −2.17 rows per captured frame, about 65–70 rows/s.
  - The whole picture returns to itself every 22.3 captured frames (0.76 s; frame correlation 0.94). That is one
    52.6-row period, so the roll is 69 rows/s.
  - The capture repeats each source frame about 3 times.
- **No knob is turned in this clip.** The alternative still, `WIN_20230129_23_50_10`, 27 min later, shows the same
  family turned elsewhere: round arcs and a magenta / cyan / purple palette. These became S10 Arch and S9 Colours.
- **The program.** At the default Speed (700) the stroke phase steps 1435 / 65536 of a period per field, which is
  69 rows/s at 60 Hz.
  - The integer model at fields 0 / 30 / 60 puts the hairline phase at 23.3 / 4.9 / 39.7 rows, against the clip's
    21.9 / 2.2 / 35.2. That is an advance of 34.2 and 68.9 rows, against 32.9 and 65.8.
  - In GHDL, two captures 6 fields apart moved 4.48 output rows (expected 4.61), and the chevrons stayed put.

## What it does
1. **Field.** Per line, slope m = Pitch + Fan·(y − 540). Per pixel, a DDA gives v = Bands·(y + tear(run) −
   m·|x − 1022|). Round replaces |x| with x²/512 (a second-order DDA), and Twin folds at 542 / 1022 / 1502.
2. **Camera / Osc.** The camera luma (or the Osc lattice), 4-px boxed and times Etch, is added to v.
3. **Colouriser.** It is a ROM of 512 entries, 4 phi units each (wrapping every 2048): cyan threshold Lc and
   intensity Ic, red threshold Lr and intensity Ir, and the mix-mode bits.
4. **Cyan.** A pixel is lit when Lc > N. N is a fixed threshold per (stroke run, 32-unit cell), stratified so each
   cell is lit in the right fraction of runs. Runs hold N over 1–3 cells.
5. **Red.** A soft key (4 levels over 13 / 128): Lr − (W(phase) − vr(y)), times a streak level per (run, cell). W
   is the stroke waveform, with flat extrema 3.2 rows wide.
6. **Mix.** Red over cyan outside the gable, both inside, red forcing white in the deep core.
7. **Softening.** Two one-pole low-passes along the line (α 1/4 each, about the still's 14-px edges), then a vertical
   [1 1]/2 against the previous line.
8. **Colour.** Two colour ROMs give each channel's Y / Cr / Cb contribution, which are summed.
9. **Output.** Chroma through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma through [1 2 1], luma 13/16, then the blanking
   gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Fan | how much the chevrons fan, i.e. the VCA depth k: 0 = an inverted fan (a bell); 344 = parallel straight chevrons; 512 = the still (k 0.00048); 1023 = 4× (an hourglass: V's at the top, steep spires low) |
| K2 Bands | the colouriser's gain, i.e. the number of nested chevron bands: ×0.25 .. ×4 (exponential, the still at 512) |
| K3 Strokes | the depth of the stroke texture (the red line PWM and the cyan brick thresholds): 0 = clean solid bands with no strokes or hairlines; 512 = the still; 1023 = 2× (thin shreds) |
| K4 Speed | bipolar (±8 deadband, square law): the strokes roll down (+) or up (−); 700 = the clip's 69 lines/s (0.76 s per period); full = 540 lines/s. The chevrons stay put |
| K5 Tear | the per-run sync slips of the field (square law): 0 = clean parallelogram strokes; 512 = the still; 1023 = 4×, the bands broken into staircases |
| K6 Etch | how far the camera (or the Osc) bends the chevrons (square law): 600 = 164 rows of field per full luma; 1023 = 478 |
| S7 Twin | One / Two gables side by side |
| S8 Flip | Gable / Valley: the field and the roll flipped top to bottom |
| S9 Colours | Red Cyan (the still) / Magenta: magenta and cyan, the 23_50_10 clip's family |
| S10 Arch | Pointed (the still) / Round: a parabolic arch (a square law in place of the H triangle) |
| S11 Source | Video / Osc |
| P12 Pitch | the roof's slope at mid-height: 0 % = a low wide roof of strata (0.35); 384 (37.5 %) = the still (0.88); 100 % = a needle spire (4.4) |

Glides (1/8 of the gap per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is box-filtered over 4 px and
  added to the field index.
  - Every chevron band bends around the picture, and the strokes and hairlines follow.
  - At the default Etch (600) the gable stays readable on selfie.jpg (`cmp_final.png`, bottom). The first default,
    450, barely moved it.
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 320 × 270 virtual-px triangle lattice feeds the same path. It is centred on its mean and
  runs at 1.5× the camera's gain.
  - At the default Etch the gable's outline steps into a hexagonal wobble, and every band zigzags.
  - The gable still reads, so the look survives with no input.

## Engine
- **Per-field sequencer (vblank zone).**
  - Seven products on the line engine's serial multiplier, which is idle in the zone: pitch, tear², speed², the roll
    step, etch², the etch gain and the Osc centring.
  - A 256 × 16 exp2 table for Bands, and the fan by shift-add.
  - The fold constants per mode (single / twin × x step).
  - The shared glide.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in 157 clocks, so it fits SD's
  858-clock line. Nine serial products:
  - the stroke phase p = p0 + 1246·y (Q16), then the row table gives W, tear and run, and the vr table its offset;
  - m = pitch ± |fan|·|y − 540| >> 8, and |D1| = |m|·G;
  - the tear times its gain;
  - the base (y + tear)·G·256;
  - the pointed start A0 = base − D1·fx / 32 and the step S = D1·xs / 32;
  - the round start A0 = base − c·u0² / 32, with D0 = c·|K0|, DD = −2c·xs² and the twin jump J = 3840·c·xs
    (c = D1 >> 10, Q5);
  - the red threshold 64 + (W − 64)·depth / 128 − vr.
- **Beam.** A 28-bit modular DDA: pointed ±S, reversing at the folds, or round A += D >> 5, D += DD. The flags for
  the next step are registered a clock ahead.
- **Pixel path (33 stages, C_LAT 32).**
  1. Beam.
  2–8. Index pipe, line-buffer luma, 4-px box, etch nibble products.
  9. Index + etch.
  10. ROM addresses.
  11. ROM data (EBR). 12. Fabric copy.
  13. Decode, red key distance, brick threshold product.
  14. Keys and levels.
  15. Mix.
  16. Channel levels.
  17–18. Two low-passes.
  17–20. Vertical store (read with the pixel entering 17, written 3 pixels behind, `no_rw_check`).
  21. Vertical average.
  22–24. Colour ROMs. 25. Sums.
  26. Compose.
  27–30. Chroma boxes and luma [1 2 1].
  31. Luma 13/16. 32. Register. 33. Gate.
- **Stores** (19 EBR in the build):

  | Store | Shape |
  |---|---|
  | line buffer (camera luma) | 2048 × 8 |
  | colouriser | 512 × 32 |
  | bricks / streaks | 1024 × 8 |
  | vertical store | 1024 × 14 |
  | colour ROMs (red, cyan) | 2 × 256 × 32 |
  | row table, vr table, exp2 | 64 × 32, 64 × 8, 256 × 16 |

- **Contracts.**
  - Damier's generated avid.
  - One latency, C_LAT 32, in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb); `gable.py` writes the colour ROMs.
  - Luma pre-scaled 16/13 and capped at 940.
  - The chroma running sums snap back to neutral after 8 blank samples.
- **Verified.**
  - **GHDL at 720p60, decimation 1, black input, against `imodel.py`:** mean abs RGB 1.34 once the RTL is shifted
    4 px left (it sits 4 px right of its virtual position). Only row 0 differs, because the vertical store holds the
    previous frame's last line there.
  - This sim found a signed-narrowing bug in |y − 540| (rows above y 29 were wrong), fixed before the final build.
  - Osc and camera renders are in `cmp_final.png`.
  - Integer model against the still at 1080p: 4 × 4 region-mean grid within 19.6 levels (mean 7.7). Zone lit
    fractions are within ~0.1 (`cmpstat.py`).
  - `thin2.py` (a row 'on' between 'off' rows or the reverse, runs ≥ 24 px; 720p and 1080p, fields 0 and 23):
    - **0 at the defaults and at P12 100 %.**
    - P12 0 %: 0 at 1080p; 13 soft dark separations (≤ 85 px) at 720p.
    - Also listed under Deviations: Fan 1023 at 720p (up to 12 runs, ≤ 540 px), Strokes 1023 (≤ 10, ≤ 38 px),
      Tear 1023 (≤ 7, ≤ 80 px) and Osc (≤ 8, ≤ 115 px).

## Status
- **v0.1.0 built 2026-11-09 (router2), not flashed.** `out/rev_b/ron/heirloom/gable.vmprog`.

| Config | Seed | Post-route Fmax | LC | EBR |
|---|---|---|---|---|
| HD Analog | 1 | 82.67 MHz | 6870 / 7680 (89.5 %) | 19 / 32 |
| SD Analog | 1 | 78.90 MHz (Fmin 27) | 6836 | 19 |
| HD HDMI | 1 | 76.24 MHz | 6839 | 19 |
| SD HDMI | 1 | 80.62 MHz (Fmin 27) | 6837 | 19 |
| HD Dual | **2** (seed 1 missed) | 83.76 MHz | 6850 | 19 |
| SD Dual | 1 | 85.38 MHz (Fmin 27) | 6850 | 19 |

- iceunpack OK × 6, and the packed config carries the new labels and presets.
- **HD HDMI margin is thin** (76.24 MHz). An earlier build of the same design, before the one-line |y − 540| fix,
  gave 82.88 MHz on seed 1, so this is seed noise.
- **Rebuild:** `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh gable`. If router2 stalls or misses, prefix
  `NEXTPNR_ROUTER="--router router1"`. dupin was not run, because the synth json is not kept.

## Deviations
- **Texture.** The strokes are a generated equivalent of the still's, not its exact strokes: fixed brick thresholds
  per (run, cell), streak cells and a per-run tear, all exactly periodic with the 52.6-row roll as in the clip.
  - The result is crisper, more regular and more symmetric than the soft camera capture.
  - The red flanks read less dense and less streaky.
  - The core's red / white zigzags are a 72-unit chevron ripple.
- **Asymmetry.** Below y 400 the still's right side is about 10 % wider than the left. The program draws the gable
  symmetric; the global fit preferred symmetry.
- **Red misregistration.** The still's red sits ~12 px right of the cyan; this is not drawn.
- **Cut for the thin-line rule:** the still's long left-edge streaks (a line-start hold), which made 1-row lines.
- **P12 0 %** is a low wide roof (pitch 0.35), not perfectly flat strata. Flat strata cut by the per-run tear made
  long 1-row slivers.
- **Remaining thin lines** at some extreme settings. Fan 1023 makes the top rows' slope cross zero, so flat strata
  there give up to 12 dark separations at 720p. The Osc and P12 0 % at 720p leave short soft ones. All are softened
  by the vertical [1 1]/2.
- **Edges.** The RTL picture sits 4 px right of its virtual position. The first output row is averaged with the
  previous frame's last line.
- **Resolutions.**
  - 720p shows the same picture as 1080p; 720p's vertical softening spans 1.5 virtual rows.
  - **SD shows the centre 4:3**: the gable is in view, but the outer rings and the flanks' outer parts are cropped.
- **Magenta** takes the alternative clip's colour family, but not its round arcs (S10 Arch gives those separately)
  or its green / blue blocks.
