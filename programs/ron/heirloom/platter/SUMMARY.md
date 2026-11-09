# Platter — a pink dotted platter with orbiting balls over breathing yellow and pink tubes

**v0.1.1 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20231204_21_01_30_t0p5.png` (clip WIN_20231204_21_01_30, 11.1 s, the frame at 0.5 s). The still shows
a large pink-and-orange disc of horizontally smeared noise dots (a 4:3 circle shown 16:9) with a black spindle hole,
a big magenta ball on its rim and a grey and a small magenta ball near the hole, over a black ground of shaded tubes:
a wide yellow-peach tube on the left, an S-shaped yellow tube on the right, pink tubes passing behind the disc above
and below it. Presets are hand-written placeholders.

## v0.1.1 (review revision)
- **No sweep line at the right tube's pipe end.**
  - In v0.1.0 a thin hard line ran from the disc's right rim to the frame's right edge (virtual line ~465). The
    warp tables had been smoothed across the right zone's step, so a few rows swept the whole tube profile.
  - The tables are now low-passed only on each side of the step, never across it: 121 lines left, 61 centre, 61 per
    side on the right. That also removes the small hooks along the left tube's rim.
  - A flag in the warp ROM marks the right zone's steep rows, table rows 205-231 (virtual lines 437-495): the upper
    arc's steep exit, the step and the lower arc's steep start. The right zone (its mirror image too, under Mirror)
    is ground there whenever |m| > 0.05.
  - The upper arc now ends with a clean cut, a black gap follows (the still is black there too), and the lower arc
    starts with a flat top at the disc rim.
  - Checked at the default, Warp 0 and Warp 100 %, with Flip off and on: no single-row line remains. At Warp 100 % a
    slanted pink ghost "needle" about 10 lines thick stays below the gap.
  - A short-way unwrap with an 81-line spread was tried first; it still left thin slanted streaks.
- **Camera default 384 -> 128** (gain 12 -> 4). The selfie card no longer shreds the tubes: they read as shaded tubes
  and visibly bend round the picture.
- **Osc fed at 3x the camera's level**, so its chevrons keep v0.1.0's +-107 px bend at the new default. K6 stays live
  in both modes.
- Build: router1, seed 1 everywhere; 7,108-7,116 LC (v0.1.0: 7,040-7,066). The RTL still equals `imodel.py`
  bit-exact.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_platter/`: `notes.md`; `geo3.py` (the disc), `balls.py`, `tex.py`, `dischue.py`;
`rowfit*.py`, `tubefit2.py`, `tubeprep3.py` (the tube field); `colprof.py` (tube colours); `track.py`, `kymo*.py` (the
motion); `proto.py`, `imodel.py`, `sheet.py`.
- **Disc.** An ellipse fitted to 575 edge points: centre (982.5, 538.1), semi-axes 561.4 x 426.6 px (ratio 1.316),
  edge residual 2.7 px.
  - Texture luma percentiles 1/5/25/50/75/95/99 % = Y 114/151/198/289/420/569/638.
  - Autocorrelation 0.89/0.65/0.41/0.23 at 4/8/12/16 px across, 0.65/0.30/0.15 at 2/4/6 lines down: blobs about
    24 px x 7 lines.
  - Frame to frame the dots are new noise (correlation 0.15).
  - Chroma: Cr about 680 everywhere. Cb follows a horizontal cosine, 415 (orange) at x 945 and about 590 (magenta /
    lavender) at x 480 and 1350: Cb = 500 - 85 cos(2 pi (x - 950) / 820). Chroma does not depend on luma.
- **Spindle hole.** Black, 32 x 22 px at the disc centre.
- **Balls** (flat soft spheres keyed inside the disc):
  - big magenta (485, 457), 77 x 58 px, Cr 875 / Cb 555;
  - grey (829, 473), 31 x 22 px;
  - small magenta (1101, 590), 35 x 24 px, Cr 820 / Cb 596;
  - faint dark red (830, 640), 26 x 18 px.
- **Tubes.** One field: luma = profile((F(x) - W(zone, y)) mod 1300 px), with F = |x - 560| + 422 (= |x - 982| left
  of x 560).
  - The profile, from rows 200 / 350 of the left tube: a glow, then a pink rim line (Y 440) at s = 0, a dark gap, the
    body rising to Y 708 at s = 90 px and gone by s = 230, and a narrower pink "ghost" tube at s = 450-570.
  - Colour by s: rim orange-pink (Cr 600 / Cb 462), body peach (Cb 404 at the peak), outer side yellow-olive (Cb 333).
    The centre tubes are pinker (Cr +125, Cb +100).
  - Three zones (x < 560 / 560-1320 / >= 1320), each with a per-line warp measured by a row-wise luma fit:
    - left: the tube's rim at |X| 652-762, one gentle wave;
    - right: the S, with its upper and lower arcs about a column at x ~1828 and a step at the pipe end (y ~465);
    - centre: hand-placed on the top tube (ridge x = 1165 - 1.7 y) and the bottom-left tube (705 + 1.1 (y - 960)).

## Motion in the clip
- **Balls.** Tracked at 10 fps, they orbit the hole counter-clockwise.
  - The small magenta ball is rightmost (x 1159) at 1.6 s, top (y 469) at 3.8 s, leftmost (x 785) at 6.0 s and at the
    bottom at 8.5 s: an **8.8 s** orbit about (972, 538), half-axes 187 x 69 px.
  - The grey ball rides the same cycle 0.58 rad past opposite (half-axes 240 x 72).
  - The big magenta ball creeps round the lower-left rim at about 7 deg/s (1/7.6 of that rate).
- **Tubes.** They breathe as a standing wave.
  - The left tube's centreline fits a lambda ~1100-line wave (residual 3.5 px). Its amplitude runs 41, 55, 48, 26, 5,
    32, 43 px at 0.5 s steps and flips sign through zero: A(t) = 55 cos(2 pi (t - 0.6) / 5.6 s).
  - The still sits at the warp's maximum. At +1.5 s the tubes stand as straight columns.
  - The right side morphs most, on the same cycle.
- **Dots.** New noise in every image (the capture repeats frames in pairs).
- **The program.** Each field:
  - the ball phase steps 124/65536 turn at Orbit 700 (8.8 s at 59.94 Hz), the big ball at exactly 1/8;
  - the breathing phase steps 195/65536 turn (5.6 s), and m = cos(phase) x Warp scales each zone's measured warp
    about its base (707 / 707 / 1600 px; m = 0 gives straight columns);
  - the dots get a new seed every two fields.
  The model at +0.5 / +1.0 / +1.5 s moves like the m-frames (`cmp_motion.png`, `cmp_rtl_motion.png`): the small balls
  swing round, the left tube straightens and the right S opens into a column.

## What it does
1. **Per field** (vblank sequencer):
   - the disc size kd, b = 427 kd, 256/b by a 9-step divider, a = 561 kd;
   - three ball positions (A kd |sin| / |cos|);
   - the warp depth and the breathing value m x 50.41;
   - the texture cell steps (1/grain by an exp2 ROM);
   - the colour-oscillator step and start phase;
   - the tube gain KF = 6453 x 2^e and the tube field's start value.
2. **Per line** (the NEXT line):
   - the warp-table row (vl x 15/32; Flip reads 505 - row), three products base +- m x |D|, and the row's ground flag
     (on when |m| > 0.05);
   - the disc chord a sqrt(1 - (Y/b)^2) from a 512-entry sqrt ROM indexed by (Y/b)^2;
   - two texture row seeds;
   - in hblank, the ball and hole row half-widths from a per-row table.
3. **Per pixel:**
   - the tube field |xa - 1120| KF as a DDA (reversing at the mirror axis), + 844 KF - W(zone) - camera, into the
     256-entry tube profile ROM (Y, U, V); the right zone on flagged rows reads the profile's black;
   - the centre zone's pink offset, weighted by the tube's luma;
   - the disc test |x - 982| < chord;
   - value noise: xorshift bytes per 16 x 5.2 px cell, lerped in quarter steps across and down, through the still's
     luma curve, + camera; Cr 680 and Cb from a cosine ROM;
   - the balls: -hw <= x - bx < hw (the hole by |x - 982|), priority hole > small magenta > grey > big magenta, flat
     colours, drawn only inside the true disc;
   - compose: ball / disc (inverted by Window) / tubes.
4. **Output:** two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the running-sum resync, luma 13/16, and the blanking
   gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Platter | The disc's size, x0.3 .. x1.35 (512 = the still). The balls' orbits scale with it. 1023 = a record filling the frame height |
| K2 Grain | The dots' cell size, x0.5 .. x3 (exponential, 512 = the still's 16 x 5.2 px). 0 = fine dust, 1023 = big blobs. Not glided |
| K3 Bands | The disc's colour oscillator. 0 = one flat orange; 512 = the still's single orange band at x 950 with magenta sides; 1023 = six bands |
| K4 Orbit | The balls' orbit speed, bipolar (+-8 deadband at 512). 700 = the clip (8.8 s counter-clockwise; the big ball at 1/8 of it). 1023 is about 2.7x counter-clockwise, 0 about 2.7x clockwise |
| K5 Tubes | The tube field gain, x0.5 .. x3 (exponential, 512 = the still). 0 = fat single tubes, 1023 = many thin tubes |
| K6 Camera | The camera gain, 0 .. 31 steps (K6 / 32; default 128 = 4). It brightens the dots (lm x g / 8) and bends the tubes (lm x g x 4 profile units). The Osc runs at 3x this level |
| S7 Flip | The tubes' measured warps read bottom-up: the S, the bends and the centre tubes turn upside down |
| S8 Colours | Rose (the still) / Neon: green, cyan and magenta tubes, a cyan-blue-green disc. The balls keep their colours |
| S9 Mirror | The left half shows the right half's tubes (the S on both sides) |
| S10 Window | Disc (the still) / Window: the tubes inside the circle, the dots outside. The balls stay on the circle |
| S11 Source | Video / Osc |
| P12 Warp | The warp depth w. 0 % = still straight columns; 440 (43 %) = the still, breathing through +-1 on a 5.6 s cycle; 100 % = a 2.4x warp, the tubes thrashing in deep S-curves and breathing twice as fast |

## Video
- **Video (the default).** The camera's luma, (Y >> 2) - 16 at half horizontal resolution from a one-line buffer,
  times K6:
  - brightens the dots, so bright parts of the picture show as pale pink dots on the platter;
  - is subtracted from the tube field, so the tubes bend round the picture.
  At the default (128) the tubes stay readable as shaded tubes on a busy picture. A black input gives exactly the
  still's patch: the RTL equals `imodel.py` bit-exact.
- **Osc.** The shell's drifting 640 x 540 virtual-px triangle lattice, made zero-mean (-112) and fed at 3x the
  camera's level, takes the same path. At the default the tubes zigzag into bold chevrons (+-107 px) and the dots
  brighten and darken in a diamond lattice.

## Engine
- **Vblank sequencer** (zc 20-691):
  - the shared 4-channel glide (K1, K3, K5, P12);
  - one right-shifting 24 x 16 serial multiplier with staged operands: the slot is decoded at L - 2, the operands
    muxed from it at L - 1, the multiplier loaded at L, and the product held after 16 iterations;
  - a 9-step divider;
  - a sine / exp2 / sqrt ROM, shared with the line engine outside the zone.
- **Line engine:**
  - kicked at each g_start (and at zc 1300 for line 0);
  - six staged serial products and the table read with its ground flag;
  - texture row seeds, finished by about clock 170 (it fits SD's 858-clock line);
  - ball rows read over 2 clocks per ball in hblank, starting 8 clocks after the generated avid falls.
- **Pixel path, stages 2-15.** cb, the ball index, the zone and the disc flag (stage 6 to stage 11) ride two
  256 x 16 block-RAM delay rings. Then boxes at 16-19, luma at 20, the gate at 21. **C_LAT 20.**
- **Block RAM: 24 EBR.**
  - line buffer 1024 x 8 (2);
  - tube profile 512 x 32, Rose / Neon (4);
  - warp table 512 x 32, sign-magnitude D0 / D1 / D2 plus the ground flag (4);
  - sine / exp2 / sqrt 1024 x 16 (4);
  - disc cosine (1); luma curve (1); ball rows (1);
  - output-box rings (4); delay rings (2); sync ring (1).
- **Contracts.** Generated avid; one latency (20) in every mode; virtual 1080-line coordinates; the boxes and 13/16
  luma before the gate, with the resync; colours in the hardware convention (U = Cr); engines started per field and
  per line.
- **Verified** (GHDL, 720p60, decimation 1, default knobs):
  - Video with black input equals `imodel.py` zones=3 on 100.00 % of pixels (4-px output alignment);
  - the Osc and camera renders are in `cmp_final.png`;
  - check (with the package) clean; TOML pre-check passes; dupin 0 `CARRY I0==I1`.

## Status
- **v0.1.1 built 2026-11-09 through the shared queue with ROUTER1.** NOT flashed.
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh platter`.
  - All six configs on seed 1. Post-route Fmax: HD Analog 80.78, SD Analog 78.09, HD HDMI 75.63 (thin), SD HDMI 85.51,
    HD Dual 79.85, SD Dual 81.49 MHz.
  - 7,108-7,116 / 7,680 LC (92.6 %), 24 / 32 EBR. iceunpack OK x6.
  - Output: `out/rev_b/ron/heirloom/platter.vmprog`.
- **v0.1.0 (same day):** router1, seed 1 everywhere; HD Analog 74.40, HD HDMI 86.29, HD Dual 86.12 MHz; 7,040-7,066 LC.
- **Router2** missed HD HDMI on seed 1 (57-67 MHz, routing-bound sequencer paths). Stay on router1. If a rebuild
  misses, try router1 seeds 2-3 before restructuring.
- **Fit history.**
  - The first cut came to 8,405 LC (109 %): an r^2 DDA disc, two noise octaves, five balls with edge rings, an
    interpolated warp table, 40-bit multipliers.
  - Yosys ablations ranked the cost: texture about 1,500 LC, balls about 950, line engine about 1,000.
  - The restructures listed above brought it to 7,058 (v0.1.0).

## Deviations
- **Tube shapes.** The tubes come from per-row warps measured on the still, scaled by the breathing.
  - The right tube's horizontal pink "pipe end" (1590-1700, 450-560) is not drawn as a capped tube. Its rows (virtual
    lines 437-495) are ground on the right while the warp is on, so the upper arc ends with a clean cut, the lower
    arc starts flat at the disc rim, and black lies between.
  - At Warp 100 % a slanted pink ghost needle, about 10 lines thick, shows just below that gap.
  - The centre tubes are hand-placed approximations; the bottom-right one is the profile's ghost tube. A small pink
    ghost fin shows above the disc's upper-right shoulder.
  - In the clip the right side passes through ")(" cusps while breathing; the model keeps the still's shapes scaled
    toward straight columns.
- **Tube colours** come from the left tube's average profile, so the left tube's lower pink stripe and the right tube's
  strong yellow come out more uniformly peach-yellow.
- **Zone seams.** The zone borders at x 560 and 1320 are hidden under the disc at the default size. They show as seams
  at small Platter settings and inside the Window porthole.
- **Not drawn:** the black fins cutting the tubes, the sync tick at the left edge, the ragged top rows.
- **Cut for budget:**
  - the faint dark-red fourth ball;
  - a second, finer noise octave;
  - ball shading and soft edges (the balls are flat discs, softened only by the output blur);
  - the disc's slightly more orange top and bottom;
  - the right zone's yellow chroma offset;
  - S7, planned first as zone-plate Grooves and then as an orange centre Label, became Flip.
- **Dots** are quarter-step value noise with a new pattern every two fields (30 Hz); the clip refreshed about 15 times
  a second.
- **Camera.** Busy pictures still fray the tube edges a little at the default 128; higher K6 shreds them.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 1440 virtual px (4:3), so the right S tube
  and most of the left tube fall outside the frame: SD shows the disc, the balls, the pink centre tubes and the inner
  edge of the left tube.
