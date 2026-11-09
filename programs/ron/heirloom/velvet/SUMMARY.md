# Velvet — magenta and red velvet drapes fanning into red chevrons, with a rippled reflection rolling up

**v0.1.1 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20240216_23_50_17` at 0.5 s
(`photos/vid/WIN_20240216_23_50_17_t0p5.png`). Soft vertical drapes in magenta, red-orange, yellow, pink-white and
violet, with dark-green shadow lines. The strands hang straight in the top half and fan apart lower down, opening into
two nested red chevrons with a dark-green tongue beside each. In the lower right a "reflection" band tears the drapes
sideways into horizontal smears, dark-green shelves and green glints. In the clip the drapes never move; the band rolls
up through them, one band every 6 s. Presets are hand-written placeholders.

## v0.1.1 (2026-11-09, review fixes)
- **Shelves instead of single-row lines.**
  - The v0.1.0 band drew its dark-green shelves as 1-px rows that ran to the frame edge, which would read as sync
    glitches.
  - Each 40-line strip now draws one finite dark-green lens, with round ends, over its first half (~20 lines at 1080p):
    - centred at the strip's phase x 4 px;
    - half-length = A1 x (|offset| - 24) x profile, capped at 320 px;
    - green glints in a halo twice that length, over the strip's first 5/8.
  - Inside the band, dark strands smeared flat by the hold regions, and the tongue core, glow violet instead of dark
    green.
  - `thincheck.py`: no single-row dark run of 24 px or more at the default, P12 0 / 100 %, Flood, Flood + P12 100 or
    Swell 0 / 1023, at four roll phases.
- **Osc at 4x the camera's gain** (was 2x). Osc vs a black input: mean abs 52 YUV, 79 % of pixels differ. The camera
  default (Etch 240) is unchanged.
- **P12 drives coverage above the still.** With d = P12 - 507 (0 .. 512):
  - the band's on-length is 1070 + 1.81 d lines;
  - the left fade's slope and start shrink by (1 - d/512);
  - the wrap threshold is 1536 + 0.94 d.
  - Band coverage over a 6 s roll is 7..81 % (mean 42 %) at P12 <= 507, and 93..100 % at P12 1023.
  - P12 507 keeps v0.1.0's gate exactly.
- **Texture.** The band's diagonal wave is gone (fewer concentric outlines). The displacement is A1 (t2 + 7/16 of),
  mostly strip holds (smears) and blobs.
- **Fit.**
  - The line engine's spare multiplier slots carry the shelf length.
  - The sequencer's two free slots carry the on-length and the wrap threshold.
  - The multiplier is 28 bits wide and the roll is in Q4 lines.
  - 7,135-7,143 LC (v0.1.0: 7,035-7,052).

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_velvet/` (`notes.md`, `palvit.py`, `blockflow.py`, `fit2.py`, `vit2.py`,
`gd2ms.py`, `proto.py`, `imodel.py`, `isheet.py`, `thincheck.py`). A clean, band-free frame of the clip (`clean.png`,
rows 0..560 at 6.1 s, 640..1080 at 5.6 s) served as the fitting target, because the drapes are identical in every frame.
- **Palette** (k = 12, whole still):
  - magenta (253,58,246) 31.0 %, red-orange (250,70,4) 16.4 %, pink (252,109,246) 10.2 %;
  - red-magenta (241,59,176) 9.7 %, violet (195,61,243) 6.5 %, red (245,62,102) 5.6 %;
  - yellow (249,175,5) 5.2 %, pink-white (243,177,235) 4.8 %, purple (138,65,212) 3.9 %;
  - dark green (12,66,10) 3.7 %; bright green (84,220,13) 1.0 %, in the band only.
- **Colouriser.** Split into channels, R sits at ~250 except along thin dark lines (R and B dip together: violet when
  shallow, dark green when deep), and G never drops below ~55-60, so the colouriser's black is dark green. G carries
  thin highlight lines (white over magenta, yellow over red). B is a bold square-ish wave: the solid magenta and red
  masses.
- **Drapes.** The strands are the top row's colours read at a displaced position X(x, y).
  - Fitted per column as X = x + S1(x) (y/1080)^2 + S2(x) (y/1080): a 2-D Viterbi over (S1, S2) at quarter resolution,
    a gradient refinement with the chevron wedges masked out, and a 12 px blur so no column seams show.
  - Residual mean abs 18.8 RGB on the half-resolution clean frame. At the bottom row S1 spans -320 .. +272 px and S2
    -93 .. +268 px.
  - No smooth warp from one reference row could open the chevrons on the right row (they appear on row ~650 at both
    axes at once), so they are an element of their own.
- **Chevrons.** The structure repeats every 944 px: chevron axes at x = 518 and 1462, U-tongues on the half period
  (47, 992).
  - Apex row 655; the arms open 0.295 px per line on each side.
  - The colour sequence down both axes is the same: magenta, red (phi 4..52 px), magenta (56..92), a violet dip
    (96..108), magenta.
- **Tongues.** A dark-green lens with a violet rim, 95 px left of each axis, from row 455 to 995, at most 26 px
  half-width.
- **Band** (the "reflection"; 118 frames at 10 fps, `kymo.py`):
  - a vertical gate of ~2040 lines, ~1070 on, rolling UP at 330-390 lines/s (one band every 6.0 s; t = 0.2 s and
    6.2 s are identical);
  - its leading edge slants, the right side ~0.153 line per px ahead;
  - in the still the leading edge is at y ~712 on the right and ~880 at x = 800, and the left is untouched apart from
    two thin tear lines (y 955, 1020);
  - inside it, rows are displaced sideways by up to 120 px in strips of 20-80 lines, with thick dark-green blobs and
    shelves, white streaks and green glints.
- **Texture.** Edges are soft everywhere (10-90 % over ~15-25 px).

## Motion in the clip
- The drapes are static: rms 1.2 between frames 5.5 s apart.
- The band rolls up at ~340 lines/s:
  - its top edge enters at the bottom at t ~ -0.5 s and reaches the top at ~2.7 s;
  - its trailing edge follows from ~3.4 s to ~5.9 s.
- The strip pattern rides with the band (it moved up 33 lines between frames 0.1 s apart).
- No knob is turned in this clip. The other clips of the patch show the strands bent much harder with ripple over the
  whole frame (23_46_16, 23_47_33), and a near-black ground with glowing red / green strand edges (23_53_14).
- **The program:**
  - At the default Tide (700) the band coordinate advances 90/16 = 5.63 virtual lines per field: 338 lines/s at 60 Hz
    (281 at 50 Hz), one band per 2048 lines = 6.1 s.
  - The strips ride with the band.
  - At power-up the band sits at the still's phase.
  - `cmp_motion.png` shows the integer model (bit-exact with the RTL) at +0.5 / +1.0 / +1.5 s next to the m-frames.

## What it does
1. **Drapes.** X0 = x + (S1(x) F1 + S2(x) F2), where the line engine computes F1 = Sway sgn(u) u^2 (u = v - Hem) and
   F2 = Sway v. The column tables are 480 entries of 4 virtual px.
2. **Band.**
   - Its displacement is dr = level x A1 (t2 + 7/16 of):
     - t2 is the strip's H-triangle (280 px period, slope 1 = the profile holds = horizontal smears);
     - of is the strip's offset.
   - The strip phase and offset come from a 256-entry sample-and-hold ROM, indexed by the 40-line strip number and
     slewed 1/4 per line.
   - The level is a half-octave gate: x1, 3/4, 1/2, ... 1/8, off. It rises over ~150 lines on a coordinate that slants
     0.153 line/px and falls behind a further 0.2 line/px left of x = 1100 (the band fades in toward the right; P12
     opens both). The trailing edge is hard after the on-length.
3. **Colour.**
   - X = X0 + dr + etch is scaled by Pleats and indexes the 1024-entry profile, which is mirror-repeated past its ends.
   - Inside phi = 0.295 (y - ya) - |distance to the axis| > -16 the colour is the chevron sequence instead.
   - The tongue core is dark green (violet inside the band) and its rim violet.
   - Each strip draws a finite dark-green shelf lens and a green-glint halo.
   - Dark strands smeared inside the band glow violet.
4. **Output.** Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Sway | the strands' fan: bent the other way (0) .. straight hanging (256) .. the still (512) .. 3x, big arcs (1023) |
| K2 Pleats | the strands' width: x3 wide (0) .. the still (512) .. x1/3 fine (1023), 1/8-octave steps |
| K3 Tide | bipolar (+-8 deadband, square law): the band rolls up (+) or down (-); 700 = the clip's 340 lines/s; full = ~44 lines/field |
| K4 Swell | the band's strips: long swells (x0.35) .. the clip's 40-line strips (512) .. fine chatter (x2.8) |
| K5 Hem | where the drapes start to fan and the chevrons open: high, deep nested chevrons (0) .. the still (340) .. low, chevrons gone (1023) |
| K6 Etch | how far the camera pushes the strands sideways: 0 .. ~400 px per full luma (square law), 240 = ~20 px; the Osc pushes 4x as far |
| S7 Mirror | Off (the still) / On: the curtain's strands mirrored about the centre |
| S8 Hang | Top (the still) / Floor: the curtain hangs from the floor (fan and V-chevrons at the top) |
| S9 Reflect | Band (the still: a rolling band) / Flood: the reflection ripples the whole frame (the sibling clips) |
| S10 Light | Day (the still) / Night: the sibling clip's near-black ground with red / pink / orange / green glowing strands |
| S11 Source | Video / Osc |
| P12 Ripple | the band's depth 0.35 .. 3.0 (square law; 507 = the still), and above 507 its coverage: 0 % = a faint shimmering band, 100 % = smears and shelves over 93-100 % of the frame at any roll phase |

Glides (1/8 of the gap per field) on K1, K2, K4, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, is box-filtered over 4 px (the
  camera's bandwidth: it bends rather than shreds) and added to X and to the chevron coordinate.
  - The strands bend around the picture.
  - At the default Etch the drapes and chevrons still read on selfie.jpg (`cmp_final.png`, bottom).
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice feeds the same path, centred on its mean, at 4x the
  camera's gain. At the default Etch the strands wave and zigzag across the whole curtain.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier (28 bits) with 16 slots, the slot decoded two clocks ahead and the operands
    pre-muxed;
  - the shared glide.
  - Per field it computes:
    - the strips per line and per field of roll;
    - P12^2, the depth and the band amplitude A1 (and x 3/4);
    - P12's coverage terms: the on-length, the fade slope and start, the wrap threshold;
    - Tide^2 and the roll; Etch^2 and the etch gain;
    - Hem, the apex row and the chevron phi of line 0.
  - Pleats and Swell are 1/8-octave exponents built by shift-adds (no exp ROM).
- **Per-line engine.** It is kicked at each line start, computes the NEXT line in ~106 clocks (it fits SD's 858) and
  shares the sequencer's multiplier:
  - u^2, F1 = Sway u^2, F2 = Sway v;
  - the strip ROM read and slew;
  - the tongue half-width 104 - 0.328 |phi - 82| and offset 428 + 0.203 phi, both from the chevron line term;
  - the shelf's centre, lens profile and half-length (two products).
- **Beam DDAs:**
  - x;
  - the strip-triangle phase;
  - the band coordinate and its fade-shifted twin;
  - the signed distance to the nearest chevron axis (wrap 944 px).
- **Pixel path (25 stages):**
  1. beam;
  2. mirrored x, table address, strip triangle;
  3. gate level (constant thresholds), the luma box;
  4. EBR -> fabric;
  5. level amplitude, the band sum;
  6. products S1 F1, S2 F2, sum x A1, luma x etch;
  7. X0, the band inner, the etch;
  8. dr = inner >> level;
  9. X, the displaced axis distance, the band-displaced x;
  10-11. Pleats (X - 960)(8 + m), the wrap, the shelf distance;
  12. the profile offset 2^s, |distance|, the tongue distance, the shelf / halo compares;
  13. the index, phi, the keys;
  14. the ROM addresses;
  15-16. ROM data and fabric copies;
  17. select (shelf, tongue, violet remap, chevron, profile);
  18. Night and green;
  19. filter input;
  20-25. output conditioning and the gate.
- **Stores** (20 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | profile | 1024 x 32 (Y10 Cr9 Cb9 + Night class) | 8 |
  | chevron | 256 x 32 | 2 |
  | column tables S1 / S2 | 512 x 16 | 2 |
  | strip sample-and-hold | 256 x 16 | 1 |
  | line buffer | 2048 x 8 | 4 |
  | chroma 8-back ring (`no_rw_check`) | 256 x 20 | 2 |
  | C_LAT sync ring (`no_rw_check`) | 256 x 3 | 1 |
- **Contracts:**
  - the generated avid;
  - one latency, C_LAT 24, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the chroma running sums snap back to dark green after 8 blank samples (resync);
  - luma pre-scaled 16/13 and capped at 940 in the ROMs.
- **Verified:**
  - GHDL at 720p60, decimation 1, black input: the default frame is **bit-exact with `imodel.py`** (100 % of pixels;
    v0.1.0 also matched across a field of roll).
  - Osc and camera frames rendered (`rtl_osc.png`, `rtl_cam.png`).
  - `sheet.png` and `presets.png` are rendered from the bit-exact model.
  - `thincheck.py` passes; `dupin.py` reports 0 duplicate-input cells.
- **Fit history:**
  - The first RTL synthesized to 6,609 LUTs standalone (~9,400 LC).
  - These changes brought v0.1.0 to 7,035-7,052 LC:
    - quantized exponents and per-mode constants;
    - one merged band multiply and a level-shift gain;
    - narrower phases;
    - the line engine on the sequencer's multiplier;
    - the tongue derived from the chevron term;
    - the Osc line step as a constant.
  - v0.1.1 added the shelves, coverage and violet remap (+~100 LC), partly paid for by removing the diagonal wave.
  - A gate built from min / subtract / compare ran at 62 MHz. Constant thresholds on a fade-shifted coordinate replace
    it.

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1; all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 84.76 |
  | SD Analog | 88.04 |
  | HD HDMI | 90.36 |
  | SD HDMI | 82.96 |
  | HD Dual | 88.24 |
  | SD Dual | 84.18 |

- 7,135-7,143 / 7,680 LC (93.0 %, ~40 LC above the ~7,100 target, with 8.7+ MHz of timing margin), 20 / 32 EBR.
  iceunpack OK x6. The packed config carries the labels and presets.
- Output: `out/rev_b/ron/heirloom/velvet.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh velvet`.
- v0.1.0 (same day): router1, all seed 1, 79.50-92.28 MHz, 7,035-7,052 LC.
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Drapes.** A smooth rank-2 warp of the top row.
  - The top half matches strand for strand. Lower down, individual strands drift by up to a few tens of px.
  - The red U-tongue at x ~990 (ending on row 320) continues as a thin red line instead of closing.
  - The left-edge blob is approximate.
  - The profile has 4 virtual px per entry, softened by the output filter.
- **Chevrons and tongues** are explicit elements. Both chevrons are identical with straight arms; the still's left
  chevron is slightly wider. The tongues are a fixed lens, violet inside the band.
- **Band.** A styled reflection, not the clip's exact distortion.
  - Strip holds and blobs, finite dark-green shelf lenses and green glints. The still's band has bigger, softer
    dark-green blobs and more white streaks.
  - Dark strands smeared inside the band glow violet (so that no 1-px dark lines appear).
  - The green comes from a chroma key on bright pixels near each shelf.
  - Its amplitude steps in half-octave levels across the leading edge.
  - At P12 <= 507 the left fade keeps the band weaker on the left.
- **Mirror** mirrors the strands only; the chevrons, tongues and band keep their places.
- **Night** is a classified 8-colour remap of the drape colours, not a fit of the sibling clip.
- **Cut for budget:**
  - a travelling "Breeze" sway switch (Hang = Floor took S8);
  - a smooth multiplier band gain and an x ramp;
  - the band's diagonal wave (v0.1.1);
  - the tongue's own width / offset DDAs.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3 (x 240..1680): both chevrons and the band are in view; the outer strands are cropped.
  - SD's shelves are ~9 lines tall (fewer lines per strip).
