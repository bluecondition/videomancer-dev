# Dropout — summary

Catalogue combo #50 (seed 20260920): datamosh bars + head-switch noise band.
v0.1.0 (2026-09-11) was flashed and liked ("dropout is good", 2026-09-12),
with no later hardware notes. v0.2.0 (2026-10-12) is the finalizing pass,
built from a review of v0.1.0. It is **not flashed yet**.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Bars | fraction of bands displaced; 100% now displaces every band |
| K2 Distance | bar shift, 0..100% of half the measured line width (±960 px at HD, ±360 at SD); each displaced bar moves 25..100% of it, hashed sign |
| K3 Hold | frames a displacement pattern is kept, 1..64 (P-frame repeat) |
| K4 Band | head-switch band height, 8..256 lines (8-line floor) |
| K5 Noise | Snow: speckle density (Noise², full static at 100%). Lines: per-line luma offset up to ±510 |
| K6 Skew | rows in the band shift progressively right (up to 32 px per row) |
| S7 Roll | band fixed at the bottom / rolling up, wrapping round (~5.7 s a cycle in every mode) |
| S8 Dropout | displaced bars lose their chroma |
| S9 Smear | displaced bars hold 16-px runs / plain shift |
| S10 Burst | bands touching the head-switch band (±32 lines) are always displaced |
| S11 Noise | Snow: 2-px grey speckles / Lines: per-line noise lines |
| P12 Bands | band height, exponential: 8 / 16 / 32 / 64 / 128 lines at 0 / 25 / 50 / 75 / 100% (default 30% = 18 lines) |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **The rolling band vanished at the top of the screen.** Its top went
  negative (11-bit wrap) once it reached the top edge, so it disappeared for
  band/3 fields (up to 1.4 s) every cycle and then popped in full-size at
  the bottom. It now rolls off the top while it re-enters at the bottom.
  The burst zone wraps with it.
- **P12 Bands had 4 steps (8/16/32/64 lines)** while its display claimed a
  continuous 8..128. It is now continuous and exponential, 8..128 lines.
  The old steps sit at 0/25/50/75%, so v0.1 slider positions map to about
  the same sizes.
- **K2 Distance showed 0..512 px but shifted up to ±1020 px.** It was also a
  fixed pixel count, so at SD (720 wide) high settings pushed bars off the
  line. It is now a fraction of the measured width (serial multiply per
  field), shown as %.
- **The snow was correlated and repeating.** It took 8 bits of a 16-bit
  LFSR that moves one bit a clock, so each pixel was about twice its left
  neighbour, and the pattern repeated every ~30 HD lines (65535 clocks). Now
  a xorshift32 generator, every bit new every clock.
- K1 at 100% still left 1 band in 256 in place. It now displaces all.
- Interlace (SD): a new pattern could start between the two fields of a
  frame (a combed frame on every change). The seed now changes only on the
  top field. HD has no field flag, so HD is unchanged.
- Roll speed was a fixed 3 lines a field: 6 s a cycle on HD but 1.3 s on
  SD. It is now 3/1024 of the height per field, ~5.7 s everywhere.

Hardware contracts:
- **U/V swap on parity-mismatched rows.** Every row shows the line stored
  one row earlier, so on an analog source whose hsync→avid offset wanders,
  rows where the parity differs from the stored row's had Cb/Cr swapped
  (thin coloured lines over colour). The drape v1.3 fix: latch the parity
  per line and per stored line, swap U↔V where they differ (~3 LC).
- **HD HDMI bar fix:** [1 2 1] on Y/U/V and a highlight knee
  (600 + (y − 600)/2, so white 940 → 770; mid-tones untouched, as in
  Reflector). The new snow is exactly the bar trigger (full-swing pixel
  steps in motion).
- Luma is clamped to 64..940 before the knee. Line noise could push luma
  to 1023, where it clips all RGB and kills chroma, or below black.
- Neutral outside avid before the smoothing, and gated again at the output.
- Latency 16 → 13, the same in all modes.

Controls (bold, always-on):
- K4 Band has an 8-line floor. At 0 the band vanished and K5, K6, S7 and
  S11 all did nothing.
- Snow is now 2-px grey speckles (64..702) that replace the picture at
  density Noise²: sparse sparkle low on the knob, full static at 100%. It
  was ±127 additive grain at most. Speckles have neutral chroma. Lines mode
  is unchanged.
- Displaced bars always move at least 25% of Distance. The old uniform draw
  gave many displaced bars a near-zero shift, so they read as untouched.
  25..50% is drawn twice as often as 50..100%, which keeps the variety.

Optimizations:
- `no_rw_check` on all 6 line-buffer RAMs. A bank's reads are only used
  while the other bank is written. Standalone yosys: `emulate_transparency`
  count is 0.
- The per-line and per-field maths is one op per state. The skew, offset
  and line-noise products are split hi/lo. The band hash, a constant
  multiply (`band × 0x9E37`, ~10 adders), is now a 256×16 random ROM in one
  EBR, keyed by band xor seed. v0.1's HD HDMI was 75.2 MHz.
- Removed the 5-stage dead delay chain and the per-stage mosh/band flag
  pipes (the row flags only change in hblank).

## Hardware checks for v0.2.0

1. Rolling band (S7 Roll): does it now wrap smoothly off the top?
2. Snow (S11 Snow, K5): speckle static instead of grain. Is it the look you
   want? Is the default (K5 49% → ~24% density) right?
3. Brightness: whites now peak at ~770 (highlight knee). Does the picture
   read too dim?
4. HD HDMI: any thick bars, especially with heavy snow and Hold at 1?
5. Analog source with colour: any thin coloured lines (U/V parity)?
6. P12 sweep: smooth band-size changes from 8 to 128 lines.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback (each needs a switch or a
knob, so they are your call):
- **Blocks:** break displaced bands into ~128-px macroblocks, each with its
  own shift (per-line 16-entry offset table built in hblank, read by x>>7).
  It reads much more like a codec glitch than full-width bars. ~250 LC.
- **Streak:** a displaced band repeats its first line all the way down
  (vertical P-frame smear). It is almost free: stop swapping the
  line-buffer banks for the band's rows.
- **Wrap:** shifted bars wrap round the line (content from the other edge,
  datamosh-style) instead of revealing black (analog tear). This could be a
  mode of S9.
- **Tracking jitter:** each row in the head-switch band also shifts a few
  random pixels (classic tracking noise), scaled by Skew.
- **S8 Dropout on mono sources:** it does nothing on a black-and-white
  feed. Option: dropout bars also get luma dropout streaks (white flecks).
- Roll speed is fixed. It could ride Hold, or the roll could slow down and
  catch the way a real V-hold does.

Tier 3, optional:
- Capture defaults and presets from hardware (the current 4 are
  hand-written and translated to v0.2 ranges).
- Head-switch chroma: the band loses chroma or gets rainbow chroma noise.
- A real flagging curve (bend strongest at the band's top, quadratic)
  instead of a linear skew ramp.

## v0.2.2: resolution scaling (2026-10-17)

The bars were sized in field lines and pixels, so they were 1.5x too tall at
720p and ~4.4x too tall on 480i (field lines, no interlace correction), and
the smear runs 2x too wide on SD. Now the sizes live on the 1080-line canvas
and are converted per mode from the timing ID:

- P12 Bands and K4 Band: round(lines x ky) whole field lines, at least 1
  (ky = 1080p 1, 1080i 1/2, 720p 2/3, 480i 0.225, 576i 0.267). The burst
  zone's 32-line margin scales the same way (a per-mode constant).
- K6 Skew is a slope: px per field line x (ys x kx) (480i 2.22, 1080i 2,
  720p 1), so the flagging leans the same on every screen. The skew
  product is widened to a 10-bit slope (hi/lo split plus a 2-bit part).
- S9 Smear runs are 16 canvas units: a run starts where the canvas x
  crosses a multiple of 16 (16 px at 1080, 10-11 at 720p, 8 at SD).
- Unchanged: Distance and Roll (already fractions of the screen), snow and
  line noise (pixel scale).
- Bug fix: under interlace a new pattern was meant to start only on the top
  field, keyed on field_n, which the hardware ties high, so the guard was
  dead. It now uses a per-field toggle when the timing ID is interlaced.
- 1080p is bit-identical (pixel diff 0).

## Build

`./build_combo.sh dropout` → `out/rev_b/ron/catalogue-combo/dropout.vmprog`.

v0.2.2 (2026-10-17, resolution scaling): all 6 configs pass on seed 1
(router2 default). HD Analog 82.6, HD HDMI 87.9, HD Dual 85.8 MHz; SD Analog
85.3, SD HDMI 80.8, SD Dual 86.4 MHz. 4,377–4,385 LC (57%; +~835 for the
three free-running per-mode products, which could move onto the per-field
serial multiplier if LC is ever needed), 19/32 EBR.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 106.4, HD HDMI 102.1, HD Dual 96.2 MHz; SD Analog 99.5, SD HDMI
96.7, SD Dual 92.3 MHz. 3,542–3,551 LC (46%), 19/32 EBR (18 line buffer +
1 hash ROM). Standalone yosys: 1,470 LUT4, `emulate_transparency` 0.

v0.1.0 (2026-09-11): HD 84.7 / 75.2 / 85.7 MHz (analog / HDMI / dual),
seed 1, 3,264 LC, 18 EBR.

## v0.1.0 backup

Git 3fa438d (sources unchanged from 2026-09-11).
