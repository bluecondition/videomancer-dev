# Streak — smeared colour streaks over a frozen mosaic (Heirloom #7)

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/streak.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.2 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`. Ron's hardware-captured defaults from the heirloom v0.5.2 (2026-11-18) are carried over, converted to the diag's knob curves: Taper 900→962, Blocks 738→723 (Bands 250, Speed 342, Edge 267, Flip Right, Streak 536 unchanged; Streak 536 is past 43 %, so the bands tear gently). The captured S7 Mosaic Lines has no diag equivalent (the Lines option was dropped), so S7 stays at Below.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Streak Diag`, `out/rev_b/ron/_diag/streak_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.0 (`programs/ron/heirloom/streak/`) is untouched.
v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230129_00_40_34_Pro.jpg`. v0.1.0 (an IIR smear with rotating hue bands) was judged too
abstract; this version is fitted to the photo (and retuned once after review: upper-right blue, red slant).
Work files: `.heirloom_work/rb_streak/`:
- `notes.md`: every measurement and the fit history;
- `proto2.py`: the float prototype;
- `imodel9.py`: the integer model of the RTL;
- `sheet_model.png`: the control sheet.

## Phase 2 (v0.6.0)
No notes from Ron for streak (free hand, 2026-11-09 batch). The v0.5 sheet showed K1, K2, K5 and P12 strong.
Four things were weak:
- K3 Blocks barely changed over its lower half (8 → 22 px both read as fine noise);
- S7 Mosaic Lines looked almost like the default;
- S8 Strips Dense looked almost like the default;
- nothing moved sideways (K4 only rolls the bands down).

The round:
- **Band tear.** Each band gets its own sideways motion: above the photo's setting, P12 tears every band
  left / right on its own triangle LFO (four rates, random phase per band). That band's slice of the diagonal
  moves with it: the mosaic edge, the green ridge, the edge stripes and the blue / green / red colour ramps.
- **Taper.** K2 is bipolar, so the band clock can sweep the other way (bands thickening down the frame).
- **Blocks.** K3's top end is doubled: blocks up to a quarter of the screen width.
- **Switches.** The two subtle ones become look-flips: S7 Mosaic Above inverts the wipe key, and S8 Red
  Plaid steps the red sequence across x as well as down.

Every new effect is exactly zero at the default knob value; the default frame is byte-identical to v0.5.0
(checks below). Phase-2 working files: `.heirloom_work/rb_streak/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| P12 Streak: noise depth 0 .. storm (photo 43 %) | **P12 Streak**: the same depth, **plus above 43 % each band tears sideways** on its own LFO (rate 1, 2, 4 or 8 per 8.5 s by band, phase per band), depth (P12 − 445) × 11/32 | 0 % = clean colour bands, no tear (as v0.5). 43 % = the photo, still. 100 % = storm noise and every band swinging up to ±396 px (21 % of the width; neighbours up to ~790 px apart): the diagonal shatters into a sliding staircase, mosaic and red corner tearing with it |
| K2 Taper: 0 (equal bands) .. 9.5:1 (photo 2.4:1 at 51 %) | **bipolar** (−100 .. 100): 0 % = 9.7:1 *reversed* (bands **thicken** down the frame), 50 % = equal bands, 76 % = the photo (**default moved 526 → 775, same picture**, exact), 100 % = 9.5:1 | at 23.9 bands: 0 % = 25-line bands at the top swelling to ~240 at the bottom; 100 % = ~234 at the top, ~25 at the bottom |
| K3 Blocks: 8 .. 240 px (square law, photo 41 at 59 %) | **7.4 .. 41 (photo, exact at 59 %) .. 478 px** | top end doubled: from a fine comb to blocks a quarter of the screen wide (slew edges ~50 px) |
| S7 Mosaic: Blocks / Lines (no S&H: a barcode; near the default look) | **S7 Mosaic: Below / Above** (relabelled): Above inverts the wipe key | Below = the photo (mosaic lower-left). Above = the mosaic takes the top half and the right side and the streaks stay only in the lower-left triangle; K5 still moves the corner |
| S8 Strips: Sparse / Dense (every other band clean; subtle) | **S8 Red: Bands / Plaid** (renamed): Plaid adds one red-sequence step per 256 px across x | red levels step diagonally across the band seams in 256-px columns: a red tartan over the bands and the mosaic |

Unchanged: K1 Bands (4 .. 64), K4 Speed, K5 Edge, K6 Hue, S9 Flip, S10 Smear, S11 Source (Video / Osc,
default Video). Dropped: the Lines option (K3 at 0 now gives 7-px blocks, close to it) and the Dense strips.

**Presets** (preset 0 = the defaults):
- **Photo**;
- **Photo Osc**;
- **Calm Bands** (P12 0, less mosaic);
- **Torn Bands** (P12 85 %, slower roll);
- **Thick Floor** (K2 0: reversed taper, fewer bands);
- **Sky Mosaic** (S7 Above, wide blocks);
- **Red Plaid** (S8 Plaid, more bands, less noise);
- **Storm** (P12 100 %, Long smear, fast).

Dropped: Frozen (K5 at 100 %).

**Engine (new / changed).**
- **Tear.** Runs in the line engine in parallel with the band hashes, states 41 .. 51.
  - The LFO phase is the field clock (+8 per field) × 1 / 2 / 4 / 8 (from hash bits), plus a per-band phase.
  - That phase is folded to a triangle −512 .. 511 and multiplied serially by the depth (8 states).
  - The line's diagonal `l_thr` is written at state 51 as edge(y) + offset.
  - Depth 0 (P12 at or below 445) adds exactly 0.
- **Taper.** `t10 = (k − 512) × 13/8`, signed.
  - Slot 1 multiplies by 1024 − t10, slot 3 by |t10|.
  - K is stored as a 24-bit two's complement for the line DDA.
  - The first-line half-step correction stays an add. Its sign error on a reversed taper is |K| on F0, about
    1/1000 of a band over the frame.
- **Blocks.** `inc = 137 + (1023 − k)² × 8732 / 2^20` (was 273 + … × 7919): the same 1592 at the default.
- **Mosaic Above.** The stage-11 key is inverted.
- **Plaid.** The red ROM address adds x / 256 to the band step.
- **Fit (no feature lost):**
  - the key is formed at stage 11 from the existing stage-10 d, so the stage-0 x accumulator and the stage-1 d
    are gone;
  - the stage-27 x, the edge ramp (12 → 18) and the red noise (16 → 20) ride three 256 × 16 block-RAM delay
    rings (`no_rw_check`, read 16 / 4 / 2 behind the write, plus a fabric register), replacing the x27
    accumulator and two FF copy chains;
  - the stage-36 pad and the stage-44 output copy are gone, so **latency 44 → 42** (an even change, U/V parity
    kept), and the sync ring reads 39 behind.
- **Unchanged contracts:**
  - generated avid;
  - one latency in every mode;
  - chroma 9-tap + luma [1 2 1] + 13/16 conditioning before the gate;
  - virtual 1080-line coordinates.

**Build** (router2, `.heirloom_work/diag_build.sh streak`, 2026-11-09):

| Config | Seed | Fmax (MHz) |
|---|---|---|
| HD Analog | 1 | 82.34 |
| HD HDMI | **2** (seed 1 stalled the router) | 79.00 |
| HD Dual | **2** (seed 1 missed timing) | 77.27 |
| SD Analog | 1 | 81.63 |
| SD HDMI | 1 | 84.16 |
| SD Dual | 1 | 77.38 |

7,211–7,221 / 7,680 LC (93.9 %, below v0.5.0's 7,284), 23 / 32 EBR (+3: the delay rings); no RAM
transparency emulation; iceunpack OK ×6; the packed config carries the new labels and presets.

**Checks** (working files in `.heirloom_work/rb_streak/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: K2 through every serial-multiplier slot for all modes and K1 values, with no overflow and F
  positive to the end of the field (min 12,213 at 1080p); K3's curve (7.4 / 41.2 / 478 px); the tear depth and
  serial product (exact, ±396 px); the torn diagonal inside its 14-bit range. The defaults are exact (t10 427,
  inc 1592).
- Default frame (GHDL 720p60 dec 1, Osc, 2 warm-up + 1 capture, production v0.5.0 simulated the same way):
  raw 10-bit output **byte-identical**.
- One extreme frame (sim-only copy with the glides starting at the extremes: P12 100 %, K2 0 %, S8 Plaid,
  Osc; `r06_x.png`): bands thin at the top and thick at the bottom, red stepping in 256-px columns, and each
  lower band's mosaic edge at its own x.
- Not verified: S7 Above, Video mode, and the tear's motion over time (one frame only).

## Reading of the photo (measured)
- **Bands from a swept clock.** Hard horizontal seams fall at:
  - y = 31, 102, 168, 229, (287, 342, 394, 444), 491, 537, 582, 625, 667, 707, 747, 785, 823, 859,
    895, 930, 964, 998, 1031, 1064;
  - they fit `phase(y) = -0.398 + 0.012903 y + 8.567e-6 y^2`, rms error 0.024 band (~1 line);
  - the band height falls from 78 lines at the top to 32 at the bottom: a band oscillator swept by the
    vertical ramp (23.9 bands per frame, taper 0.42).
- **Three channels.**
  - R follows an **8-step sequence** per band: full-width means 30, 93, 166, 221, 213, 171, 117, 92.
    Step 0 = the quiet blue/green strips at y 31, 491, 823.
  - Within the upper bands R shifts by about ±1.5 steps from left to right: bands 5-7 fade toward the
    right, bands 1-3 brighten.
  - G and B are complementary folds of one source: the G/B scatter fills G + B <= ~280; both are dark
    in only 0.2 % of pixels. Their levels are spread, not binary (a soft fold, gain ~1.2).
- **Streak texture.** Horizontal ACF half-width: G 20 px, B 24 px, R 30-70 px; vertical: G 4-5 lines.
  The row spectrum falls ~8 dB/octave below a 60-150 px corner: smeared noise, with no S&H period.
- **Mosaic.**
  - Below y = 548 and left of a diagonal, each band holds one line, with a slow drift (2 -> 15 levels
    over the band).
  - Block edges are spaced in multiples of **41 px**: a horizontal S&H clock whose phase differs per
    band, with 16-24 px slewed edges.
  - B is pinned at 255, so the blocks are blue, magenta, cyan and white.
- **One diagonal ramp, four jobs.** The mosaic edge is x = 1.74 y - 90 (rms 79 px, ragged ±40 px per
  band). Along r = x - 1.74 y:
  - B is ~240 at r < -500, holds a ~120-145 plateau for r 100-700, and falls to ~30 by r = 1150;
  - G falls from ~110 to 0 (r 300 -> 1400), so the top-right corner is red with a little blue;
  - a green ridge follows r ~ 0;
  - just right of the mosaic edge the ramp folds into parallel pink/white contour stripes.

## What it does
1. **Source.**
   - Fine noise: xorshift16 white noise seeded per line by a hash of (line + scroll), then a 2-pole
     horizontal IIR, then a 1-pole vertical IIR through a line RAM. The texture restarts at every band
     seam.
   - A slower "blob" octave (x 1/128, y 1/32).
   - Osc: the blob octave stands in for the camera. Video: the camera luma, smeared 1/8 along the line.
2. **Band clock.**
   - A per-line 2nd-order DDA `phi += F; F += K`, exact for the fitted parabola.
   - The band index picks the red sequence step and a per-band hash (colour offset, edge jitter, S&H
     phase).
   - Step-0 bands are clean strips: noise / 4, plus a full-width line hold.
3. **Diagonal ramp** `d = x - edge(y)`:
   - mosaic key `d < jitter` (below cy);
   - ridge `0.42 cycle x (1 - |d - 90| / 384)`;
   - edge fold stripes `2.15 cycles x (1 - d/128)`;
   - blue bias `+(400 - d)/2` left of d = 400, 0 on [400, 850], `-(d - 850)/2` beyond (in 1/512);
   - green VCA `(1550 - d) / 1024`.
4. **Mosaic.** In the key the source deviation (x 1.41) is sampled-and-held at the block clock, slewed
   (1/2 .. 1/16, scaled with the block width), and held per band in a line RAM with a 1/64 per-line
   drift toward the live picture. Blue is full and red gets +16.
5. **Colourizer** (theta in 1/1024 cycle):
   - G = fold - 0.15 (x VCA);
   - B = 1 - fold + bias;
   - R = the sequence read at (band + (x - 960)/19.7 sub-steps) through a 256-entry interpolated ROM,
     plus blob noise / 2.
   - RGB -> YUV through three contribution ROMs.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Bands | the horizontal seams | 4 .. 64 bands per frame (default 23.9 = photo) |
| K2 Taper | bands thinning down the frame | sweep of the band clock: equal bands .. 9.5:1 (2.4:1 = photo) |
| K3 Blocks | the mosaic blocks | S&H block width 8 .. 240 px (41 = photo); the edge slew follows the width |
| K4 Speed | (motion) | bipolar, still at the centre ±8: the bands roll through the frame and the streak texture rides with them |
| K5 Edge | the diagonal | wipe position, from a corner of mosaic to the whole frame frozen; the ridge, edge stripes, blue/green ramps and red corner move with it |
| K6 Camera | the camera's weight in the colourizer and red terms | camera strength (v0.6.1), 0..100 %, default 50 %: a gain on the camera luma above black, 0 = no camera (the picture a black input gives), 25 % = the shipped ×1, 50 % = ×2, 100 % = ×2.9 (k / 128 up to 50 %, then 2 + (k − 512) / 512; 24 steps). Osc: the blobs ×0 / 0.5 / 1 / 2 (0, < 19 %, < 38 %, above). The colourizer phase (was K6 Hue) is fixed at the photo's centre |
| S7 Mosaic | the blocks | Blocks / Lines (no S&H: each band holds a raw line, a vertical barcode) |
| S8 Strips | the clean blue/green strips | Sparse (every 8th band, photo) / Dense (every other band) |
| S9 Flip | the diagonal | mosaic on the left (photo) / mirrored to the right |
| S10 Smear | the streak length | Short (photo) / Long comet streaks |
| S11 Source | — | Video (default) / Osc |
| **P12 Streak** | the noise texture | noise depth: 0 = clean colour bands and smooth blocks; 43 % = photo; 100 % = a shredded static storm |

Glides (1/8 per field, ±4 deadband) on K1, K2, K3, K5 and P12.

## Video
- The camera luma replaces the Osc blob octave as the colourizer's slow source (0.69 cycle across the
  luma range, smeared 1/8 along the line) and adds to the red level (0.19 x).
- The camera is tapped ~12 px ahead of the beam, which is invisible in the smear.
- Everything else stays: the streak noise rides on the picture, the bands and the red sequence cut it,
  and below the diagonal the camera picture is frozen into held blocks.
- Osc substitutes large-scale blob noise with the same statistics as the photo's camera content.

## Engine
- **Per field** (vblank, one serial 26 x 22 multiplier, 12 slots):
  - band density and taper -> F0 and K of the band DDA (per-mode `ys`, `ys^2`);
  - the S&H step and slew; the wipe corner;
  - the P12 depth as a 5-bit mantissa and a shift, normalised per mode;
  - band-phase roll and noise scroll, one add per zone clock;
  - pole selects and noise gains per mode class (1080p / 1080i / 720p / 480p-576p / 480i-576i).
- **Per line** (states 35..60 after the source avid fall):
  - band index and seam; band hashes and the line seed on one shared 16-bit hash unit;
  - step / strip / offsets / jitter; then the DDA advance;
  - the engine writes the line registers directly once the previous line has left every stage that
    reads them.
- **Pixel path** (44 stages):
  - per-line xorshift; recursive x IIRs;
  - noise RAM read-modify-write (1024 x 16, half resolution);
  - P12 product, deviation, key-path gain, S&H + slew;
  - hold RAM read-modify-write (1024 x 16: dev 10, red 5, keyed 1);
  - fold, VCA, red sequence ROM, RGB -> YUV ROMs;
  - output conditioning, gate.
- **EBR (20).** All are collision-free and carry `no_rw_check`:
  - noise and hold RAMs (4 + 4);
  - a 256 x 32 ring for the live theta / deviation / red noise (2);
  - three RGB->YUV ROMs (6);
  - the red-sequence ROM (1);
  - the sync delay ring (1);
  - the chroma 8-delay ring of the output conditioning (2); its sums are re-anchored to 8 x 512 in
    blanking.
- **Contracts:**
  - generated avid;
  - one 44-clock latency in every mode;
  - chroma 9-tap + luma [1 2 1] + 13/16 conditioning before the gate;
  - chroma neutral outside the line;
  - virtual 1080-line coordinates from the timing ID.

## Status
- **v0.5.0 built 2026-10-21 (after the colour retune). Not flashed.**
- Router2, **seed 1 on all six configs**.
- Post-route Fmax: HD Analog 85.16, HD HDMI 85.45, HD Dual 82.77, SD Analog 76.79, SD HDMI 80.10,
  SD Dual 80.64 MHz.
- 7238-7284 / 7680 LC (94-95 %), 20/32 EBR. Every bitstream passes iceunpack.
- **Verification.** GHDL renders at 720p60 decimation 1 (Osc and Video):
  - against the integer model: mean absolute difference 3.5/255, 82 % of pixels within 6 levels
    (after the shell's 4-px capture shift);
  - the captured frame matches the model at field 3, so the per-field motion works;
  - 4x4 grid of cell means against the photo: max 25, rms 11.8 (before the colour retune: 58 / 22.5).
- **Fit history:** 137 % LC (v1 RTL) -> 109 -> 101 -> 99 -> 93 -> 94 %, through:
  - address counters instead of index delays;
  - late x accumulators;
  - half-resolution line RAMs;
  - EBR rings for the syncs, the conditioning chroma and the late live values;
  - RGB->YUV ROMs;
  - xorshift instead of a 5-round pixel hash;
  - no sequencer scratch registers.

  The first 720p RTL sim caught a hold-RAM read one pixel early (held blocks sheared diagonally); it
  is fixed.

## Deviations from the photo, and why
- The noise is a different realisation of the same statistics, so the photo's specific camera blobs
  are not reproduced. The line-seed salt (0x51F1) and band salt (11) were chosen by grid and
  per-band colour scores against the photo. A few 320 x 180 cells still differ by 21-25 in R or G
  (the upper half reads slightly more purple, less green, than the photo).
  The realisation differs between 1080p and 720p (1080p grid max 35).
- No horizontal flow of the streaks: the per-line xorshift noise cannot be offset per field, so K4
  moves the bands and the texture vertically only.
- The partial upper-right holds in bands 31-167 are not reproduced.
- Noise and hold state are stored at half horizontal resolution (2-px pairs; invisible at 20-px
  streaks and 41-px blocks). The fine vertical pole is 1/8 per virtual line.
- Chroma is blurred over 9 px and white is capped at ~777 by the mandatory output conditioning.
- LC is 94-95 %, over the ~92 % heirloom budget; the HD timing margins are 8-11 MHz.
- At SD and 720p the picture is designed to match 1080 (poles and gains normalised per mode); only
  720p was rendered in GHDL.

## Open questions for Ron (2026-11-09 batch)
1. The band tear rides P12 above the photo's 43 %. Would you rather it had its own control (for example in
   place of S10 Smear's knob-free switch)?
2. Should the tear also slide the held mosaic blocks (the S&H clock phase), not only the edge and the colour
   ramps?
3. The Lines barcode option is gone (K3 at 0 gives 7-px blocks, close to it). Do you want it back?
4. Plaid adds the column step to the band step (a diagonal tartan). Would columns only (vertical red bars
   crossing the bands) be better?

## Hardware watch list
- Renamed / relabelled:
  - S7 Mosaic Blocks / Lines → Below / Above;
  - S8 Strips Sparse / Dense → Red: Bands / Plaid;
  - K2 Taper is bipolar, with the default moved 526 → 775 (same picture);
  - preset Frozen → Torn Bands / Thick Floor / Sky Mosaic / Red Plaid.
- Latency 44 → 42 in every mode, and three new block-RAM delay rings: watch for green or noise lines, or a
  sheared mosaic.
- Thinnest HD margin: HD Dual 77.27 MHz on seed 2; HD HDMI 79.00 on seed 2.
- At P12 100 % the torn bands should read as a deliberate glitch, not a sync fault.
- No features cut for fit.

## Next step
Wait for Ron's preview notes. On approval: `./heirloom_diag.sh merge streak`, then repack the previewed diag
bitstreams under the production TOML instead of re-routing:
1. copy `build/programs/ron/streak_diag/rev_b/bitstreams/*.bin` into the production build dir;
2. regenerate `program_config.bin` from the production TOML;
3. run `vmprog_pack.py --no-sign --hardware rev_b`.

Then update the production SUMMARY / README / PHASE2 row and commit only streak's files.

## K6 camera pass (2026-11-15)
Ron: K6 is the camera strength in every program, default 50 %. The same design as production v0.5.1. **v0.6.1.**
- **K6 Camera** (0..100 %, default 512), a sixth glide slot: the camera's luma above black × g (Q3, one multiply in
  its own camera-path stage) ahead of the shipped −438 offset, the colourizer term and the red term. g = k/128 up to
  50 %, then 2 + (k − 512)/512: 0 % = a black input's picture, 25 % = the shipped ×1, 50 % = ×2, 100 % = ×2.9.
  Osc blobs ×0 / 0.5 / 1 / 2.
- **K6 Hue dropped** (fixed at 512, the photo); it was the weakest control on the sheet, and the diag left it
  unchanged.
- **Router1 fix (bit-exact):** the v0.6.0 K2 Taper term `(k − 512) × 13/8` was written on the signed value, which
  added two copies of one sign bit in a carry chain (a CARRY with I0 = I1: nextpnr router1 stalled on every seed).
  It is now `(13k − 6656) / 8` on the unsigned knob.
- **Build (v0.6.1, 2026-11-15):** router1. HD Analog 83.47 (seed 2), HD HDMI 85.98 (seed 2), HD Dual 84.22 (seed 1);
  SD Analog 74.23 (s1), SD HDMI 85.51 (s1), SD Dual 74.31 (seed 2) MHz. Seed 1 stalled router1 (600 s timeout) on
  HD Analog, HD HDMI and SD Dual even with the self-add fixed (no CARRY I0 = I1 left in the netlist). 7472–7514 /
  7680 LC (97.8 %), 23/32 EBR. Preview `out/rev_b/ron/_diag/streak_diag_v0.6.1.vmprog`. NOT flashed.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023, v0.6.2) and its top end is 25 % stronger. **Element type**
(recolour): K6 Camera sets how far the camera luma drives the colourizer phase and the red term, recolouring the
streaks by the camera's brightness; the camera image itself is not shown. Same change as production streak: the
per-field gain is now c_g5 = g + g / 4 (rounded in the upper half; one 5-bit add at zone clock 30), so 50 % = x2.5
and 100 % = x3.6 (was x2.9). The Osc blob select is unchanged, and the deviation clamp (2900) and red clamp (+-383)
still make a high gain saturate instead of wrap. Widths: camera product <= 27.8k of 17 bits signed. Production
render: `.heirloom_work/rb_streak/k6b/k6b_cmp.png`.
**Build:** router1, seed 1 on all six; HD Analog 82.27, HD HDMI 82.36, HD Dual 81.96 MHz (worst HD 81.96); SD 81.32 /
79.67 / 82.27 MHz; 7508-7542 / 7680 LC (98.2 %), 23/32 EBR. Preview `out/rev_b/ron/_diag/streak_diag_v0.6.2.vmprog`.
NOT flashed.
