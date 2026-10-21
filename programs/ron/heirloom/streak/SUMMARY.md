# Streak — smeared colour streaks over a frozen mosaic (Heirloom #7)

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230129_00_40_34_Pro.jpg`. v0.1.0 (an IIR smear with rotating hue bands) was judged too
abstract; this version is fitted to the photo (and retuned once after review: upper-right blue, red slant).
Work files: `.heirloom_work/rb_streak/`:
- `notes.md`: every measurement and the fit history;
- `proto2.py`: the float prototype;
- `imodel9.py`: the integer model of the RTL;
- `sheet_model.png`: the control sheet.

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
| K6 Hue | the green/blue/magenta balance | colourizer phase ±½ cycle, photo at the centre |
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
