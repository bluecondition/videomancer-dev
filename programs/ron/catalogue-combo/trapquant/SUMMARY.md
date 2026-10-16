# Trapquant — summary

Catalogue combo #120 (seed 20260926): perspective keystone / corner-pin +
posterization. v0.1.x was liked on hardware ("trapquant is good",
2026-09-12). That build missed HD timing (66–68 MHz). The v0.1.3 timing fix
and the v0.1.4 edge clamp came later and were never flashed. v0.2.0
(2026-10-13) is the first finalizing pass, built from a review of v0.1.4.
v0.2.0 was flashed 2026-10-13: it bars on HD HDMI at 1-bit chroma, so
v0.2.1 (below) is the follow-up; not committed. v0.1.4 sources and vmprog are in
`.trapquant_work/v014/` (also git 3fa438d).

## v0.2.1 (2026-10-13): HD HDMI bars at 1-bit chroma

Ron flashed v0.2.0 and saw noisy bars on HD HDMI with K5 Chroma at 1 b.
At 1–2 bits, chroma jumps between 64, 512 and 960, and the dither flips it
every 2 px. After the guard's [1 2 1] that is still ~224 codes a pixel.
(The default 3 bits gives at most 64 a pixel, which did not bar.)

The fix: at 1–2 bits, and only when S11 is on Posterized, the chroma
smoothing becomes a 9-tap [1 2 2 2 2 2 2 2 1]/16 instead of [1 2 1]. That
caps a full-swing step at 56 codes a pixel and cancels the 2-px dither
cells. Dithered chroma now shows as softened colour band edges rather than
a colour texture. At 3+ bits, and with Keep, chroma is exactly as in
v0.2.0. Latency is still 16: the filter fits in the existing chroma delay
slots (two running 8-sums per channel, resynced to neutral in every blank).

Check: K5 at 1 b and 2 b on HD HDMI, with K6 at 0 and at 100%. If it still
bars, also try K4 Luma at 1 b. Black/white luma edges are 64↔775, which
is ~356 a pixel after [1 2 1]. Ron has not reported trouble there yet, and
smoothing luma would soften the crisp black/white look, so that is his call.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Top | width of the picture at the top, 25..225% (the readout is now the real width), glided |
| K2 Bottom | width at the bottom, 25..225%, glided |
| K3 Slide | horizontal slide, ±½ of the measured width, in source space (wide rows move further: the trapezoid shears), glided |
| K4 Luma | luma depth 1..8 bits; levels always include black and white |
| K5 Chroma | chroma depth 1..8 bits; centred, so greys stay grey |
| K6 Dither | dither amount, 0..100% of a step, centred (no brightness shift) |
| S7 Edges | Fill (the row's own edge pixel) / Mirror (mirrored tiles at any width) |
| S8 Order | dither cells keyed to the source (they stretch with the keystone) / to the screen (they stay put) |
| S9 Ramp | quantizer thresholds shift with each row's scale; bands slide as the trapezoid moves |
| S10 Dither | 2x2 Bayer (2-px cells) / alternate lines |
| S11 Chroma | posterized / kept |
| P12 Tilt | ±100% width: up narrows the top and widens the bottom (floor 25%), glided |

## v0.2.0 changes (from the v0.1.4 review)

Bugs:
- **The right side of narrow HD rows wrapped.** The source x was kept to
  12 bits (±2047). Any HD row narrower than ~88% reads past 2047, so the far
  right showed the LEFT edge's colour in Fill and wrong content in Mirror.
  At the defaults that is the last ~70 px of the bottom rows. The 480×270
  sims never reach x = 2047, so they could not show it. The source x is now
  15 bits.
- **Chroma posterize had no neutral level.** It was a mid-rise quantizer, so
  at the default 3 bits every grey became U = V = 576, a pink/magenta cast,
  and at 1 bit it became 768. It is now a centred mid-tread quantizer, as in
  flutter v0.2: levels at 512 ± k·step, clamped 64..960.
- **Luma levels never reached black or white.** 1 bit gave greys 256/768,
  and from 3 bits the top level (960) broke the ~768 drawn-luma cap. Luma is
  now normalized, posterized and bit-replicated, so black and white are
  always levels, then scaled to white ≈ 775.
- **Dither brightened the picture.** The offsets were 0..¾ of a step, all
  positive, so K6 at 100% lifted the mean by ⅜ step. Lines used 0/¾ (one
  sided). Offsets are now centred: Bayer (2b−3)/8, Lines ±¼.
- **Ramp brightened the picture.** It added up to +287 codes (at 1 bit about
  half the picture flipped to white), and at high depths it was sub-step
  noise. It is now a zero-mean threshold phase, at the same strength at
  every depth: it cycles 4× per unit of row scale, so the bands slide when
  Top, Bottom or Tilt move.
- **K1/K2 ran backwards against their labels.** The knob set the source
  step (1/width), so "Top 225%" made the top 44% wide. The knob is now the
  width. The default and the presets were translated to the same pictures.
- **The trapezoid's sides were curved and jagged.** The step was
  interpolated linearly, which makes the width hyperbolic down the screen.
  The step was also 8-bit, so the sides moved in 3.75-px stairs every
  ~10 lines at HD. Each row's width is now linear (straight sides), and a
  per-line restoring divider gives the step at Q12 (sub-pixel stairs).
- **Mirror ran out.** It reflected only once each side, so narrow rows
  showed edge smear beyond it. The DDA now runs modulo 2W' in Mirror, and the
  reflection is inside the C_EDGE inset, so there are mirrored tiles at any
  width and no flat strip at the seams.
- **Slide was ±512 px in every mode**: 27% of the width at HD, 71% at SD.
  It is now ±½ of the measured width.
- **The K4/K5 readout disagreed with the depth** in about half of each zone
  (e.g. "2 b" over 74..127 was 1 bit). Zones now follow the readout's
  rounding, with ±12-count hysteresis.

Hardware contracts:
- **Per-row U/V parity swap (drape v1.3).** Each row replays the row above,
  so analog rows whose hsync→avid parity differs had Cb/Cr swapped.
- **HD HDMI bar guard (damier v0.4.4).** [1 2 1] on Y/U/V before the gate,
  luma white ≈ 775. Dithered posterize steps are full-swing pixel edges that
  move with the keystone: the bar trigger. The 13/16 luma scale is folded
  into the level mapping (64 + level·0.695).
- **Dither cells are now 2 px wide.** The guard's [1 2 1] flattens a 1-px
  checker into a flat grey, as found on posterduo. Lines are untouched (the
  smoothing is horizontal only).
- Banks are latched per line at the avid rise. `no_rw_check` is set on all
  6 line-buffer RAMs. Width is latched once a field. One latency (16) in all
  modes.
- **Glides (¼ per field) on K1, K2, K3 and P12.** One knob step moves the
  picture's edge by several pixels, so pot noise shimmered the sides.

Optimizations:
- One read-side quantizer serves both orders: v0.1 had two, write side and
  read side. A point-sampled gather commutes with a per-pixel quantizer, so
  Source order only needs the dither cell keyed to the read address.
- Per-frame maths runs on one shared, operand-split multiplier. Per-line
  maths is one op per state (26 clocks after the avid fall, inside every
  mode's hblank and the sim's 64-clock fake one).

## Hardware checks for v0.2.0

1. **HD default:** the same trapezoid as v0.1 (the top fills the screen,
   the bottom is ~82% wide). Greys should now be neutral (v0.1 tinted them
   pink at the default 3 bits), blacks black and whites ~775. Dither is a
   little coarser and edges a touch softer (the guard).
2. **Narrow rows in Fill** (K2 low, or P12 up): the right fill should be the
   right edge's colour all the way out. v0.1 switched to the left edge's
   colour near the right edge.
3. **S7 Mirror + P12 at either end:** mirrored tiles everywhere, no smear.
4. **P12 sweep:** the sides should stay straight and smooth, with no stair
   steps, and the motion should glide.
5. **K6 sweep:** the picture should not get brighter as dither rises.
6. **S9 Ramp on + P12 sweep:** bands slide through the picture. Toggling S9
   should not jump the brightness.
7. **K4/K5:** the readout matches the depth; 1 bit luma = black and white.
8. **HD HDMI:** any thick bars? Analog colour source: any thin coloured
   lines?

## Review backlog (not done)

Tier 2, look changes (your call):
- **A. S8 Order and S10 Dither do nothing at K6 Dither 0.** A point-sampled
  keystone commutes with a per-pixel posterize, so only the dither
  pattern's coordinates differ between the orders. The Screen Bands preset
  has K6 = 0, so both its switch settings are dead. Recommended fix: an
  **interpolated keystone** (linear between neighbouring source pixels).
  Source = posterize, then interpolate: the bands get soft gradient edges
  that stretch with the zoom. Screen = interpolate, then posterize: crisp
  contour lines, and no blocky zoom. This puts the quantizer back on the
  write side for Source order and adds even/odd line-buffer banks plus
  three small lerp multipliers (~400–600 LC; there is room). Cheaper
  alternative: a dither floor of ⅛ step, so a dithered fringe always runs
  along band edges.
- **B. P12 climax: vanishing point.** Let the narrow end go below 25%
  (to ~6%), so P12 100% collapses the far end into a strip of tiny mirrored
  tiles, a tunnel to the horizon. The modular DDA already handles it; the
  source range needs two more bits.
- **C. Coarser dither.** 4x4 Bayer, or 2×2-px square cells, would give a
  more retro look. S10 is binary, so this would replace Lines or 2x2.
- **D. Ramp as a screen gradient.** Today Ramp follows the row scale, so
  it is strongest with a strong keystone and only a constant offset when
  Top = Bottom. The alternative is a fixed top→bottom threshold gradient
  that shows at any shape.
- **E. Hue posterize.** S11 Keep could instead posterize hue and
  saturation (gouache's octants), giving flatter poster colours than
  independent U/V levels.

Tier 3, optional:
- Capture defaults and presets from hardware. The 4 presets are
  hand-written, translated to v0.2.
- SD interlace: dither rows follow the field line, so in the frame the
  Lines pattern is 2 lines tall. HD is progressive and unaffected.
- The top row shows the previous field's last line (1-line replay). It is
  invisible in practice.
- Not feasible: vertical foreshortening (rows nearer the narrow end
  squashed together). It needs a frame store, and there is only a line
  buffer.

## Build

`./build_combo.sh trapquant` → `out/rev_b/ron/catalogue-combo/trapquant.vmprog`.

v0.2.1 (2026-10-13): all 6 pass. HD Analog 81.8 (**seed 2**; seed 1 missed),
HD HDMI 81.5 (seed 1), HD Dual 88.2 (**seed 3**; seeds 1–2 stalled router2);
SD 82.6 / 81.6 / 79.5 MHz. 5,177–5,198 LC (68%), 18/32 EBR. If HD Dual stalls
again, try `NEXTPNR_ROUTER="--router router1"`.

v0.2.0 (2026-10-13): all 6 configs pass on seed 1 (router2 default).
HD Analog 86.5, HD HDMI 77.1, HD Dual 84.5 MHz; SD Analog 77.5, SD HDMI
82.0, SD Dual 71.4 MHz. 4,989–5,025 LC (65%), 18/32 EBR. Standalone yosys:
2,688 LUT4, `emulate_transparency` 0. HD HDMI has the thinnest margin
(2.8 MHz); if a later change pushes it under, profile before restructuring.
The +850 LC over v0.1.4 is mostly the bar guard, the two DDAs' wider (27-bit)
datapath, the per-line divider and the normalized luma quantizer.

v0.1.4 (2026-09-13): HD 81.8 / 87.7 / 79.3 MHz (analog / HDMI / dual),
seed 1, 4,160 LC (54%), 18 EBR.
