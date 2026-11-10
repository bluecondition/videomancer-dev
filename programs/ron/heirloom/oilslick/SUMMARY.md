# Oilslick — an oil-slick kaleidoscope of orange domes, blue cores and green-edged black voids over a dark reflection

**v0.1.0 built 2026-11-10, NOT flashed.** Heirloom from the video clips (phase 3b), made from the still
`photos/vid/WIN_20231022_00_05_49_t0p5.png` (clip WIN_20231022_00_05_49, frame at 0.5 s; the same patch in clip
WIN_20231022_00_04_37 at 0.5 s and 9.0 s, in a green / magenta / cyan palette). A camera-feedback frame seen through a
horizontal kaleidoscope: red / orange domes with magenta and peach arcs around blue cores and two navy "eye" blobs
with cyan rings, black inverted-heart voids with wavy green edges between them, a purple sky along the top, and under
a horizon a dark lower tier with red flames, a scalloped green line and an orange band. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_oilslick/` (`notes.md`, `palfit` `pal16_*`, `adj.py`, `foldfit.py`,
`invert.py`, `mkmap5.py`, `proto5.py`, `imodel.py`, `thincheck.py`).
- **Kaleidoscope.** Horizontal autocorrelation peaks at 936–946 px in every band; a fold fit (minimum within-bin
  variance) gives a period of **942 px with the dome axis at x = 1020** (void axes 549 / 1491). The two halves of a
  tile are only 0.66–0.83 mirror-correlated: a kaleidoscope that the feedback then skews.
- **Two tiers.** A horizon seam at y ≈ 600–615: the vertical ramp resets there. Row luminance: bright 260–500 (domes),
  dark 520–850, an orange band at 860–940.
- **Domes** (x 600–1350 per tile): an outer red arc topping at y ≈ 210, orange / peach arcs (250–300), a thin navy arc
  on the left (300–330), pink / magenta (320–400), a tilted blue oval core (x 900–1180, y 360–460), and below it two
  navy blobs with cyan / green rings at (900, 545) and (1110, 540), r ≈ 115 × 72. An orange / peach hourglass neck
  (x 880–1100, y 100–200) rises to a pink cap under the sky.
- **Voids:** black inverted hearts between the domes (y 30–330), dim purple / maroon inside, a wavy green outline.
- **Lower tier:** a black dome-shaped mass under each dome (x 720–1300, y 610–840) bounded by a scalloped green line
  (period ≈ 110 px), an orange arch band (850–950), a row of dark ovals (period ≈ 113 px), red flames under the void
  axes.
- **Colour model.** The colours are fitted (rms 11/255) by two scalars per pixel: a hue phase φ through ONE palette
  (red → orange → salmon → peach → pink → magenta → violet → cyan line → blue, extended cyclically) and a brightness V
  (the voids, the dark lower tier, the blobs fading to navy), with a green line wherever V crosses ≈ 0.15 (green next
  to red, cyan next to blue). R and B are smooth; G is a spike channel (the lines).
- **Palette** (palfit k = 16): black 28.6 %, maroon 8.8, dark red 8.4, red (243, 19, 18) 8.3, orange (242, 112, 7) 6.8,
  red 5.7, navy (23, 9, 96) 4.9, violet (118, 8, 167) 4.3, salmon 4.1, ochre 3.7, dark green 3.4, blue (45, 4, 210) 3.2,
  magenta (249, 16, 176) 3.1, peach (246, 150, 148) 2.9, green (5, 150, 24) 2.2, cyan (7, 164, 149) 1.7.
- **Why stored maps.** A quadratic dome explains only 0.20 of the phase variance; analytic dome / egg-crate / teardrop
  protos read as "shapes put together". The program stores the frozen feedback frame instead (see Engine).
- **Integer model (= the RTL) vs the still** (colour classes, % of the frame, `shares.py`): black 24.5 vs 31.0, red
  13.3 vs 17.4, orange 9.1 vs 8.7, peach 2.8 vs 3.1, magenta 5.8 vs 5.2, violet 3.2 vs 4.3, blue 4.8 vs 4.3, green 9.1
  vs 3.6 (the lines are drawn bolder than the still's to keep every crest at least two rows thick).

## Motion in the clip
- The strip's first three frames are clip-start duplicates; m0 = 0.0 s, still 0.5 s, m1–m3 = 1.0 / 1.5 / 2.0 s.
- **Nothing translates:** edge-map cross-correlation peaks at (0, 0) for every pair, so the kaleidoscope axes and the
  horizon are screen-fixed; the content boils in place (dome-edge correlation 0.11–0.14 between frames).
- **Ron sweeps a level knob:** the lower tier's dark fraction goes 73.6 % (m0) → 41.6 (still) → 7.2 → 4.4 → 5.8 %, its
  white 0 → 0 → 0.1 → 18 → 29 %; it rises from black through red, orange and yellow to white. The upper tier cycles
  backwards through the palette (red domes → blue cores → pink / white → pink / blue → red), about a cycle in 2 s.
- **In the program:** P12 Tide is that knob (the lower tier lifts and the whole palette phase runs backwards; at 100 %
  the lower tier blazes yellow / white / blue and the domes run red / orange like m2–m3). The free-running motion is
  K4 Flow (a slow palette drift, 0.045 cycle/s at the default, up to 0.8 cycle/s), an oil ripple and a gentle upper
  wobble drifting at −0.31 cycle/s, and a water shimmer on the lower tier (23-line ripple, −0.9 cycle/s).
  `cmp_motion.png`: clip frames | the integer model with P12 swept 380 → 930 | the integer model free-running.

## What it does
1. **Maps.** Upper tier 64 rows × 64 cells (600 virtual lines × one 942 px period), lower tier 32 × 64 (480 lines from
   the horizon); each cell holds φ (6 bits, 63 = 0.8 palette cycle) and V (4 bits). Rows are mirrored past their span
   (Horizon above 600 gives the domes a mirror image; a high horizon gives a tall lower tier).
2. **Row per line.** The line engine weights rows rc − 1, rc, rc + 1 by a quadratic B-spline (C1 in y, so no contour
   pinches to a single row) into a ping-pong row buffer of {cell, next cell} pairs.
3. **Pixel.** A tile phase (Q8, 2^24 per period; K1 scales the step, the per-line wobble offsets the start) picks the
   cell and a 6-bit fraction; φ and V are interpolated across the row.
4. **Colour.** phase = 0.1 + (φ − 0.1)·Rings + flow + tide + ripple + Etch·luma → a 256-entry palette (Oil or
   Contour). V' = V + Voids (lower tier + the tide lift; Slick Light = 1 − V') scales the colour toward black, and a
   green line (magenta for Contour) is added where the un-lifted V is within 0.092 of 0.15 (a linear profile).
5. **Output conditioning:** U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, Y through
   [1 2 1] on the same centre, luma 13/16, blanking gate. Palette luma pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Domes | the kaleidoscope period: one wide dome (0, ×2.83) .. the still's 2.04 domes across (512) .. 5.8 narrow domes (1023), exponential |
| K2 Horizon | the horizon / tier seam: y 294 (the domes half sunk, a tall dark lower tier) .. 600 (512, the still) .. 906 (the domes gain a mirror image, a thin lower tier) |
| K3 Rings | the domes' contour density about the red rim: ×0.35, an ember wash with green lines (0) .. ×1 (512) .. ×2.83, fine oil-slick rings (1023) |
| K4 Flow | bipolar palette flow through every ring (±8 still at the centre, square law): 640 = 0.045 cycle/s (the default drift), 1023 = 0.8 cycle/s, left reverses |
| K5 Voids | the black voids and dark tier: none, the voids filled with dim colour (0) .. the still (512) .. black swallowing all but the dome cores (1023) |
| K6 Etch | how far the camera (or the Osc, ×4) bends the rings: 0 .. 0.9 palette cycle per full luma, square law; 350 ≈ 0.1, a bend |
| S7 Slick | Dark (the still) / Light: the negative, the voids glow with colour and the domes sink into shadow |
| S8 Colours | Oil (the still) / Contour: the green / magenta / cyan / black look of clip 00_04_37 (magenta lines) |
| S9 Reflect | Tier (the still's own lower tier) / Mirror: the domes reflected upside-down in the water |
| S10 Boil | Calm / Boil: the oil ripple ×10 and the wobble ×11 (upper) / ×7 (water): the slick boils and the water heaves |
| S11 Source | Video / Osc |
| P12 TIDE | Ron's sweep: moonlit at 0 (white-hot cores in blue / violet domes over black water) .. the still at 50 % .. high tide at 100 % (the lower tier blazes yellow / white / blue, the domes run red / orange) |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a half-resolution line buffer, times the Etch gain is
  added to the phase, so every ring bends around the picture; at the default Etch the domes, voids and tiers still
  read clearly (the selfie render), at 1023 the picture dominates. With no input, Video mode is the still.
- **Osc mode.** A drifting 2-D triangle lattice (240 × 180 virtual px) replaces the camera, fed at 4× the camera gain
  (a per-field gain), so the rings crinkle into a lattice of hot spots; Etch stays live.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier (right-shifting accumulator, 16 steps per
  product, then it holds; 16 slots), a shared glide (K1, K2, K3, K5, K6, P12; 1/8 per field, ±2 deadband) and an exp2
  ROM (Domes, Rings). It computes the
  tile step and start, the rings gain, the flow step and phase, the etch gain (Osc ×4), the wobble steps and drifting
  phases, the tide lift (and its ramp slope), the voids offset, the horizon and the tier phase offsets.
- **Per-line engine** (kicked at each line start, computes the NEXT line on its own serial multiplier, done by e_lc
  189): tier and map row, the B-spline weights, the tide ramp, two parabolic wobble sines, the wobble × tile step →
  the line's tile start, the lower-tier lift. The **row builder** runs alongside (e_lc 54 .. ~255, three map reads
  per cell). Fits SD's 858-clock line.
- **Pixel path (16 stages):** beam → row-buffer / line-buffer addresses and ripple phase → EBR → fabric copy, ripple
  product → row differences, etch product → ×fraction → φ, V, ripple + etch → φ − pivot, V, + line offset → rings
  partial products → rings, V', line bump → phase, Slick → palette address, V clamp → palette EBR, brightness →
  fabric copy → palette × brightness, line colour × bump → compose. Then the conditioning (stages 17–21).
- **EBR (24):** maps 4096 × 10 (10) + 2048 × 10 (5), palettes 512 × 32 (4), row buffer 256 × 32 (2, `no_rw_check`),
  video line buffer 1024 × 8 (2), exp2 256 × 16 (1).
- **Contracts:** generated avid, one latency (C_LAT 20) in every mode, blanking gate, virtual 1080-line coordinates
  (SD shows the centre 4:3), hardware colour convention (U = Cr), the chroma box resync.
- **Checks:** GHDL analyse / elaborate / synth front end clean (`check_pkg.sh`, which adds the _pkg). dupin on the
  synth json: 0 `CARRY I0==I1` (one LUT with a duplicated input, from an exponent self-add that was then rewritten).
  `thincheck.py` counts 1-row luma bumps > 60 over both neighbours, in runs ≥ 24 px, at three field phases:
  - 1080p: default 0 / 0 / 0, P12 100 % 0 / 0 / 0, P12 0 % 4 runs of ≤ 30 px;
  - 720p: default 2 runs of exactly 24 px (crests of a green line), P12 0 % 2–7 runs (≤ 37 px), P12 100 % 0–2.

## Status
- **v0.1.0 built 2026-11-10 with ROUTER1**, all six configs on **seed 1** (none needed a retry). Post-route Fmax:
  HD Analog 85.03, SD Analog 84.53, HD HDMI 86.92, SD HDMI 89.94, HD Dual 83.87, SD Dual 81.45 MHz (SD Fmin 27).
  7,145–7,161 / 7,680 LC (93 %, about 50 over the ~7,100 guideline), 24 / 32 EBR. Every bitstream passed iceunpack,
  and the packed config carries the new labels.
- Output: `out/rev_b/ron/heirloom/oilslick.vmprog` (507,507 bytes). **Not flashed.**
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh oilslick`.
- **GHDL verification (720p60, decimation 1, real timing ID):**
  - the default Video frame with a black input is **bit-exact with `imodel.py`** (921,600 of 921,600 pixels, after
    the tester's 4-px capture offset of the generated avid), and the next frame is bit-exact too, with 46 % of the
    pixels changed between them (flow, wobble, ripple drift);
  - Osc: 47.9 % exact and 89.9 % within 8 codes of the model (the model's lattice is approximate);
  - camera (selfie): 78.2 % exact, 98.5 % within 8.
- **Build history.** The first cut was 8,348 LC (108 %). It went to a single chroma box with luma [1 2 1] (owlet's
  conditioning), right-shifting 18-bit serial multipliers, no lower-wobble envelope, the ripple derived from the
  tile phase, an offset-only tide lift, 6-bit brightness and a 5-bit line weight. The GHDL renders then caught two
  RTL bugs before the final build: the right-shifting multipliers kept shifting past 16 steps (every product read a
  clock late was halved), and a narrowing signed `resize` folded the φ / V interpolation sums and the rings phase
  (blue cores came out orange). Both were fixed; the final build is the verified netlist.
- HD margins are comfortable (84–87 MHz). If a later version tightens, stay on router1. LC to reclaim: the
  row-builder tags (a write-cell counter instead of 6-bit cells in every tag) and the wobble phase steps (a
  per-timing-ID table instead of sequencer slots).
- No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.

## Deviations
- The still's fine feedback detail (wiggles under ~30 px, the "M" ghost letters in the voids, the horizontal camera
  streaks) is below the maps' 14.7 px × 9.4-line cells and is not drawn; the arcs are softer than the still's.
- One period is stored and repeated: the screen shows two identical tiles, where the still's left and right domes
  differ from the centre one (the centre period is the one kept).
- The green lines are drawn bolder than the still's (half-width 0.092 V, a linear profile), so no crest is a single
  row. Green covers 9.1 % of the frame against the still's 3.6 %.
- Black covers 24.5 % against 31 %.
- S10 Boil with a high tide gives the lit lower tier a strong zigzag water texture (the 23-line shimmer at ±40 px):
  intended, but busy.
- The low tide (P12 near 0) can leave a few soft single-row glints in the lower tier where a brightness ridge just
  clears the night darkening (the thin-line counts above).
- The clip's boiling (shapes re-forming within 0.5 s) is approximated by the palette flow, the ripple and the wobble;
  the stored frame itself does not change shape.
- The tide sweep reproduces the clip's trend (lower tier dark → yellow / white, domes cycling backwards), not its
  exact colours.
- Resolutions: 720p shows the same picture; SD shows the centre 4:3 (the outer quarter-domes fall outside).
