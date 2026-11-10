# Harlequin — orange lemon diamonds in violet / red stripes that hug them, under glass, rolling up

**v0.1.1 built 2026-11-10, not flashed.** v0.1.1: the default camera bend is smooth. Etch is on a square law and the default gives a quarter of v0.1.0's bend, the camera luma goes through an 8-px box and a vertical smoothing, and Osc is fed at 16× so it flips as strongly as before. A lighter camera product brings timing back to seed 1 everywhere. Heirloom (video phase), from the clip `WIN_20231218_00_10_10` at 0.5 s
(`photos/vid/WIN_20231218_00_10_10_t0p5.png`). Two tall orange "lemon" diamonds (an olive band down the middle,
red-orange shoulders) stand in a field of violet and red vertical stripes. Each diamond's left side cuts the stripes;
its right side drags them along, packed tight against the edge and relaxing into gentle waves further right. A big
soft "glass" diamond washes the stripes lavender / olive around each lemon, and a thin outline (the lemon key's edge)
crosses the stripes. Below a torn seam the same patch runs squashed: flat orange diamonds half a period over, with
chevron stripes. In the clip the whole picture rolls up through a V-hold slip. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_harlequin/` (`notes.md`, `palfit` labels, `chan_R/G/B.png`, `gshape.py`,
`dshape*.py`, `bandshape.py`, `track.py`, `fitS*.py`, `fitPD.py`, `colmeas.py`, `mshape.py`, `proto.py`, `imodel.py`,
`sheet.py`, `thincheck.py`).
- **It is an RGB colouriser with one layer per channel** (the channel split shows it):
  - B carries the stripes: a clean square wave, 238 / 3, switched off inside the orange core.
  - G carries the glass and the lemon: about 9 outside, 90 inside the glass, 140 inside the lemon.
  - R is about 127 outside (the red stripes only +7 on the violet ones), 97 under the glass, 90 in the lemon. Inside
    the core it is a horizontal V ramp: 142 at x 485, +0.45 per px, clipped at 252.
  - Colours: violet (127,7,238) and red (127,13,3) with purple fringes; under glass (98,87,231) / (96,93,2); in the
    lemon (92,136,226) / (88,140,2); core (142..252, 140, 0): olive band → orange → red-orange shoulders.
- **Geometry** (per-row half-widths about x 525; the second diamond sits at 1467, so the period is 942 px):
  - Core (B key): narrow necks of 21–62 px at y 20–140 (olive), flanks 1.1 px per line, 254 px at y 380, necks of
    15–24 px at y 680–740.
  - Lemon (G key): tip at y 40, 231 px at y 280, 197–201 px sides from y 400 to 560, tip at y 770. It is wider than
    the core at the top, so its edge line crosses the stripes; it is narrower at the shoulders, so the core shows
    red-orange there.
  - Glass: 395 px at y 380, bottom tip at y 895 (1.0 px per line), top above the frame.
  - Band (y 830–1060): lemon tip at y 865, 189 px at y 980, core 174 px; keys centred at x 995 (half a period over).
- **Stripes.** Right of each core they are offset copies of the core's right edge, compressed (period 37–45 px
  against the edge at y 400, against 60–70 px 300 px further on). They are periodic with the 942-px period: every
  tracked edge right of both cores repeats at +942 ± 2 px in every row. Left of each core they end on its edge.
  - Away from the cores the period is about 80 px at the top, 64 px at y 380 and 100–106 px in the bottom rows,
    where the stripes slant right.
  - Model: S = F·d + A·(1 − e^(−d / 170 px)), where d is the distance from the core's right edge. The stripe
    oscillator is hard-synced at the core's trailing edge and gets a decaying frequency kick there.
    A = F·(hb − Dfar), with Dfar a smooth far-field displacement (0–130 px).
- **Seam.** y 770–830: strips of about 25–30 lines with sideways slips and sheared stripe "commas", and a second seam
  under the band.
- **Texture.** Soft: the B edges take about 16 px from 10 to 90 %. The lemon edge is a dark / bright relief line.

## Motion in the clip
- **The picture rolls up (a V-hold slip).** The band diamond's centre is at y 1180 / 980 / 820 / 640 / 475 at
  t 0 / 0.5 / 1.0 / 1.5 / 2.0 s (200, 160, 180 and 165 px per 0.5 s), which is about 352 lines / s.
- One roll cycle (main row → seam → band → seam) is about 1150 lines: a 740-line lemon, 220 lines to the band's
  peak, then 190 lines to the next lemon tip.
- In the strip's first frames the seam sits near the frame edges; then it rolls. Ron's hand is on the V-hold.
- The widths breathe about ±15 % from frame to frame (main lemon 232 / 271 / 265 px). This is not modelled.
- **In the program:** P12 V-Hold at its default (400) rolls the picture up at 366 lines / s, which is 6.1 virtual
  lines per field over the 1150-line cycle. The GHDL frames at fields 3 and 9 moved up 24 rows at 720p: 36 virtual lines, 6.0 per field. At 0 % the roll
  locks with the main row centred, and above 50 % it speeds up and the cycle squashes (2.5 cycles per screen at
  100 %). The seam strips' sideways jitter is redrawn every 48 fields.

## What it does
1. **Coordinates.**
   - hp is the horizontal diamond phase (Q24 per period, 942 virtual px at the default), with u the signed phase
     about a diamond centre.
   - v is the position in the rolling 1150-line cycle, read from a 288 × 32 vertical table (4-line steps,
     interpolated in the line engine). The table gives the core, lemon and glass half-widths, the band flag (keys
     half a period over) and the seam strip. A 72-entry part (16-line steps, interpolated) gives the stripe frequency
     F and the far displacement Dfar.
2. **Keys.**
   - Core: |u| < hb.
   - Lemon: |u| < hg, with the relief line where ||u| − hg| < 8.6 px.
   - Glass: a 4-level soft key (gl − |u|) / 15 px, centred on the main diamonds.
   - The camera (or the Osc) term lowers |u| for all of them.
3. **Stripes.** d = (u − hb − camera) mod 1 period; S = (F/8)·d/512 + A·E(d)/256 + line shear + camera.
   E(d) = 255·(1 − e^(−d/τ)) is a 512-entry ROM. The square wave has 8 soft levels.
4. **Colour.** A palette ROM, 1024 × {Y, U, V}: bank (S10) × region (stripes / core ramp / hot core stripes) × the
   stripe level, glass level, lemon and outline bits, or the 32-step ramp.
5. **Seam.** In the four seam strips the line's start phase slips (90, −60, 70 and −40 px, plus a ±16 px jitter
   from a per-field hash) and the stripe phase shears 9, −5, 8 and 0/256 of a cycle per line.
   - Torn: every 32-virtual-line band gets a hashed ±128 px slip half the time, redrawn every 6 fields, with the
     seam strips doubled.
6. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes. Their 8-sample delays are block-RAM rings,
   and the running sums reload 8 × black after 8 blank samples. Then luma is scaled 13/16, and the blanking gate is
   applied on the generated avid.

## Controls
| Control | Element it drives |
|---|---|
| K1 Diamonds | how many diamonds across: 0.7 (0, one giant lemon) .. 2.04 (512, the still) .. 5.8 (1023), ±1.5 octaves about the screen centre; the stripes stay the same width |
| K2 Stripes | the stripe frequency: ×0.35 (fat bands) .. ×1 (512) .. ×2.8 (hairlines), exponential |
| K3 Waist | the orange cores' width: none, a lemon outline over hugging stripes (0) .. the still (512) .. ×2, cores merging into a bow-tie band (1023); the stripes' far waves scale with it |
| K4 Bend | the stripes' kick after each core: ×−1, the stripes bow the other way (0) .. the still (512) .. ×4, a dense fan pressed against each core's right side (1023) |
| K5 Glass | the glass diamonds' size: none, pure violet / red (0) .. the still (512) .. ×2, lavender / olive everywhere (1023) |
| K6 Etch | how far the camera (or the Osc) widens the keys and bends the stripes, square law: 0 .. 320 (default, up to 20 px of key and 0.35 stripe cycles at full white) .. 1023 (the same flood as v0.1.0) |
| S7 Shape | Lemon (the still's fitted V shapers) / Rhombus (triangle V shapers: straight-sided diamonds, chevron stripes everywhere) |
| S8 Tear | Calm (the still's seam strips) / Torn (32-line bands slip sideways all over the picture, redrawn every 6 fields) |
| S9 Core | Orange (the still) / Stripes (the hidden stripes run hot pink-orange through the cores) |
| S10 Patch | RGB (the still) / GBR (the colour patch rotated: green-cyan stripes on navy, violet cores) |
| S11 Source | Video / Osc |
| P12 V-Hold | 0 % = locked, the main row centred and still .. 400 = the clip's roll (366 lines / s up) .. 100 % = a fast roll with 2.5 squashed cycles per screen |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16, does two things once it has been smoothed and scaled
  by Etch. It is smoothed by an 8-pixel running box before the line buffer (emptied at each line start), then
  vertically: s = prev + (luma − prev)/4, with the previous lines' state in a half-resolution 1024 × 8 block RAM. The
  two effects:
  - it lowers the key distance, so the cores, lemons and glass swell over bright parts of the picture;
  - it adds to the stripe phase, so the stripes ripple along the picture's contours.
  A black input adds nothing, so with no camera Video mode is the still. At the default Etch, with
  `photos/selfie.jpg`, the lemons swell gently over the bright face. The lemon outlines and stripes stay clean curves
  (v0.1.0 showed per-line jaggies along the cores' sides). Etch 640 bends harder, and full Etch floods the cores over
  the face.
- **Osc mode.** A drifting triangle lattice (320 × 270 virtual px), made bipolar (−110 .. +112), through the same
  smoothing and fed at 16× the camera gain (capped at 511, the same depth as v0.1.0). The lemons bulge and pinch into lobed shapes, horizontal orange wings grow on the lattice ridges, and
  the stripes wave by up to ±2.7 cycles. Every video-path control stays live.

## Engine
- **Per-field sequencer (vblank zone).**
  - A shared glide over K1–K6 and P12.
  - Exponents for K1 and K2 come from an exp2 ROM. The diamond step c_hd = 8905·2^e1 per half virtual px, and the
    first-pixel phase is taken about the screen centre.
  - The stripe factor 4096·2^(e2 − e1) keeps the stripes' px width when K1 changes.
  - Speed = 1.25·P12² / 128, and the squash factor and the per-line cycle step come from P12.
  - The roll advances, or locks toward home below P12 48. A seed advances every 6 fields.
  - All products run on one serial multiplier, with the operands pre-muxed two clocks ahead.
- **Line engine.** Kicked at each line start (and once in the zone for line 0), it computes the NEXT line in about
  170 clocks, which fits SD's 858-clock line, on one 32-bit serial multiplier:
  - four table reads;
  - five interpolations;
  - Rhombus: three triangle widths;
  - hb × Waist, Dfar × Waist, glass × Glass, F × K2 / K1, A = F·(hb − Dfar), A × Bend;
  - the seam shear;
  - a 16-bit add / rotate / xor hash for the slips.
  The pixel path loads the results at the next line start.
- **Pixel path (25 stages).**
  - Phase and distances. The line-buffer and vertical-state reads are addressed with the stage-1 pixel. Then the
    vertical smoothing (written back on even pixels; the read four pixels ahead never meets the write address).
    Then the camera product (luma × a 4-bit mantissa of the gain, shifted by its exponent at the next stage), keys,
    then the F·d product (split 6 / 6).
  - The E ROM with a fabric copy, then the A·E product (split 4 / 4), the stripe phase and the soft square.
  - The palette address, the palette ROM with two fabric copies, the two boxes, 13/16 and the gate.
- **ROMs / RAMs.** Vertical table 512 × 32 (4 EBR), E 512 × 8 (1), exp2 256 × 16 (1), palette 1024 × 32 (8), line
  buffer 2048 × 8 (4), camera vertical state 1024 × 8 (2), two box rings 256 × 32 (4): 24 EBR.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 24, in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr); luma pre-scaled 16/13 and capped at 940.
  - The box sums snap back to black after 8 blank samples.
- **Verified.**
  - `imodel.py` mirrors the arithmetic, including v0.1.1's camera box, vertical smoothing and square-law Etch. GHDL
    720p60 decimation 1 frames match it in structure, palette and texture: black input and a second black frame
    6 fields later (v0.1.0; the camera path adds nothing with no input), and Osc and the selfie camera (v0.1.1).
  - The first sim caught a resync bug: the box history lagged its input by one sample, so each line's first sample
    was dropped from the running sums, giving a per-line level offset and dark bands. It was fixed before the build.
  - `thincheck.py`: 0 single-row luma runs ≥ 24 px in 1080p and 720p, at the defaults, P12 0 / 100, Torn, Rhombus,
    the K1 / K2 / K3 / K4 extremes, Fill and Osc, plus the selfie camera at Etch 320 and 640.

## Status
- **v0.1.1 built 2026-11-10 through the shared queue with ROUTER1; all six configs on seed 1. Not flashed.**
  - Post-route Fmax (the last "Max frequency" line per config): HD Analog 81.68 MHz (6,998 LC), SD Analog 79.63
    (7,006), HD HDMI 91.54 (7,004), SD HDMI 87.83 (6,998), HD Dual 81.35 (7,013), SD Dual 87.80 MHz (7,012; SD Fmin
    27). 24 / 32 EBR. iceunpack OK ×6; the packed config carries the labels and presets.
  - Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh harlequin`.
  - Output: `out/rev_b/ron/heirloom/harlequin.vmprog`.
  - No self-adds (dupin: 0 `CARRY I0==I1`). One transparency emulation (the line buffer, read-before-write kept as in
    the shell); the two box rings and the camera's vertical state carry `no_rw_check`.
  - **v0.1.1 timing history.** The first v0.1.1 netlist kept v0.1.0's 10 × 10 per-pixel camera product.
    - At 92 % LC that product was the critical path (luma → product, 71.4 MHz on HD Analog seed 1).
    - HD Analog closed only on seed 4 (75.07) and HD HDMI on seed 3 (76.11). HD Dual missed all six seeds (best
      effort 72.11 MHz), so that build was stopped.
    - The product became the luma times a 4-bit mantissa of the Etch gain, with the exponent applied as a
      per-field shift at the next stage. That is −86 LUT, all six configs on seed 1 with 7–17 MHz of margin, and an
      Etch gain within 1/16 of the exact value.
- **v0.1.0 (2026-11-10)** was built with all six configs on seed 1 (HD 74.73–76.23 MHz, 6,837–6,881 LC, 22 EBR).
  - The first v0.1.0 netlist was 7,504 LC. It was cut to ~6,860 by moving the output boxes' 8-sample delays into
    block-RAM rings, loading the line's values straight from the engine, a 32-bit line-engine multiplier, a 12-bit
    stripe frequency in the pixel path, and writing the Rhombus widths in place.

## Deviations
- **Band diamonds.** They have a narrow neck on top (bell-shaped). The still's are rhombi whose tops the seam cuts
  flat.
- **Seam strips.** They are cleaner than the still's torn rows: whole strips slip and shear, with no interlace comb.
- **Breathing.** The widths' ±15 % breathing between frames is not modelled. The glass diamonds do not continue as
  the light diagonal bands that cross the band in some frames.
- **Camera smoothing.** The vertical smoothing lags the camera a few lines downward, and the box about 4 px to
  the right. Neither is visible.
- **Stripes.** They are a little crisper and more regular than the still's. The far-field waves follow a smooth fit,
  so individual stripe positions differ by up to about half a period in places.
- **Output.** As in the shell, the picture sits about 9 px right of the virtual grid (the filters and the generated
  avid's start), so the left ~8 px fade in from black.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3 (1.5 diamonds across). With 4.4
  virtual lines per SD line, the seam strips' shear steps are coarser.
