# Vhsmix — summary

Catalogue combo #87 (seed 20260923): posterization + head-switch noise band
+ multi-source mixing (add / screen / multiply / difference / overlay).
v0.1.0 (2026-09-11) was flashed and liked ("vhsmix is good", 2026-09-12),
with no later hardware notes. v0.2.0 (2026-10-12) is the finalizing pass,
built from a review of v0.1.0. v0.3.0 (2026-10-13) adds colour blends (the
Tier 2 item Ron picked). Neither is **flashed yet**.

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Luma | posterize luma, 1..8 bits (labelled). Black and white are always levels |
| K2 Chroma | posterize chroma, 1..8 bits (labelled). Centred: greys stay grey |
| K3 Band | head-switch band height, 1..50% of the measured picture height (default 4.8% = 52 HD lines) |
| K4 Noise | noise in the band, 5..100% (100% = ±half the luma range) |
| K5 Blend | Add / Screen / Multiply / Difference / Overlay (labelled, fifths) |
| K6 Mix | blend amount: 0 = the original, 100% = the blend |
| S7 Ghost | off / the posterized layer is read 1/16 of the width behind (120 px HD, 45 px SD), a tape ghost trailing right |
| S8 Band | normal / the band is the negative of the posterized layer |
| S9 Roll | band fixed at the bottom / rolling up and wrapping round (~5.7 s a cycle in every mode) |
| S10 Noise | luma only / chroma too (independent U and V noise, held 4 px: colour streaks) |
| S11 Chroma | Posterize: the posterized chroma mixed on (K2 at 8 bits = the original chroma) / Blend: chroma goes through the blend mode too (see below) |
| P12 Static | noise over the whole picture, squared: fine grain low on the slider, snow (±half the luma range) at 100% |

## v0.3.0: colour blends

S11 was Blend / Keep. It is now **Posterize / Blend**. Posterize is v0.2's
behaviour: the posterized chroma is mixed on by Mix. Keep is gone because
K2 at 8 bits already gives the original chroma (to within 2 codes). Blend
sends chroma through the same blend mode as luma. A is the posterized
chroma (or its ghost with S7), B is the original, both treated as a signed
U/V vector about neutral:

| Blend | Chroma | Without Ghost (A ≈ B) |
|---|---|---|
| Add | A + B, hard clip | saturation ×2 |
| Screen | A + B − A·\|B\|/512 (= \|A\|+\|B\|−\|A\|\|B\|/512 when the signs agree) | soft saturation boost, never clips |
| Multiply | complex A·B / 256: hue angles add | every hue doubles (psychedelic remap), weak colour fades, greys stay grey |
| Difference | (A − B) × 2 | colour contours on a grey picture; colour emboss with Ghost |
| Overlay | follows the luma: Multiply colour where the posterized luma is dark, Screen where light | hue-doubled shadows, boosted highlights |

With S7 Ghost and S11 Blend, the chroma ghosts too: the ring is now 32
bits wide (valid + Y + U + V, 2 EBR). With S11 Posterize, the chroma stays
aligned, as in v0.2. The negative band still takes the inverted posterized
colour. The colour products use the top 8 bits of each operand, one 8×8
multiply a stage. Chroma is mixed with the same row Mix as luma, then
noise, then clamped to 64..960.

Presets: Overlay Roll's old S11 Keep is now K2 at 8 bits. Two new presets:
Hue Twist (Multiply + Blend, K2 8 bits) and Colour Ghost (Difference +
Ghost + Blend).

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **S7 Order did nothing in 4 of the 5 blends.** Add, screen, multiply and
  difference are symmetric, so swapping the operands changed nothing. Only
  Overlay responded. S7 is now **Ghost**: the posterized layer is read 1/16
  of the width behind the original, so every blend mixes two different
  pictures. Difference turns into an embossed relief, Add into a double
  image, Multiply into a dark shadow. This is a new function for the
  switch, so it is your call (the alternative was an Invert switch, see
  the backlog).
- **Chroma posterize had no neutral level.** Greys came out pink: U=V=576
  at the default 3 bits, 768 at 1 bit. Chroma is now a centred quantizer
  (levels at 512 ± k·step), so greys stay grey at every depth. The same fix
  as flutter v0.2.
- **Luma posterize never reached black or white.** At 1 bit the picture
  was two greys (256 / 768). It now keeps L bits of the normalized luma and
  bit-replicates them, so black and white are always levels.
- **The blend maths pivoted on the wrong white.** Operands were
  black-relative (0..876) but divided by 1024, so Multiply turned white ×
  white into ~750 and the Overlay/Screen pivots sat off-centre. Blends now
  work on normalized luma (0..1023 = black..white).
- **The rolling band vanished at the top.** Its top went negative (11-bit
  wrap) once it reached the top edge, so it disappeared until the roll
  counter reset, then popped in at the bottom. At SD, any Band taller than
  the field made it vanish entirely. It is now placed modulo the field
  height and wraps smoothly. (Same bug and fix as dropout v0.2.)
- **The noise was correlated and repeating.** The old 16-bit LFSR moved
  one bit a clock, so each pixel was about twice its left neighbour
  (diagonal streak texture), and the pattern repeated every ~30 HD lines.
  It is now xorshift32. Chroma noise was one axis (U up, V down: only
  blue↔orange). U and V are now independent.
- Band rows ran 1 px late and posterized chroma lagged luma by 1 px
  (off-by-one delay taps). All taps are aligned now.
- **K5 Blend showed numbers 0..4, in zones that didn't match the
  hardware** (it read "1" while still on Add, "2" while still on Screen).
  It is now labelled, in fifths.
- K1/K2 showed "8" only at the very top of the knob. The hardware reached
  8 bits from 88%. They are now labelled eighths that match the decode.
- P12 above K4 made the band *quieter* than the rest of the picture. The
  band now takes the larger of the two.
- Roll speed was a fixed 3 lines a field: 6 s a cycle on HD, 1.3 s on SD.
  It is now 3/1024 of the height a field, ~5.7 s everywhere.

Hardware contracts:
- **Luma could reach 1023** (Add/Screen + noise), which clips all RGB and
  kills chroma. Output is now 64 + n·0.695: white ~775, black 64, never
  outside that.
- **HD HDMI bar fix:** [1 2 1] smoothing on Y/U/V plus luma scaled to white
  ~775. Posterized edges and full-swing noise in motion are exactly the bar
  trigger. The scale is folded into the de-normalize, so it costs nothing
  extra.
- Chroma is clamped to the legal 64..960 after the noise.
- Neutral outside avid before the smoothing, and gated again at the output.
- Band row flags are committed at the avid rise, not at hsync. The
  pipeline tail runs past hsync on SD front porches.
- Labelled knobs honour the firmware's label-INDEX default until the knob
  first moves (labelled-default trap), with zone hysteresis on K1, K2 and
  K5 so pot noise can't flicker the picture at a boundary.
- Latency 15 → 22, the same in all modes.

Controls (bold, always-on):
- K3 Band has a 1% floor. At 0, v0.1's band vanished and K4, S8 and S9
  all did nothing. The band is now a fraction of the measured height, so
  HD and SD look the same. v0.1 maxed out at 255 lines (24% of HD); it now
  reaches 50%.
- K4 Noise has a 5% floor, for the same reason: at 0 with S8 Normal, the
  band was invisible.
- P12 Static is squared and reaches snow at 100% (±half the luma range).
  v0.1's top was ±255 codes of grain. The 50% point is about where v0.1's
  was.

Optimizations:
- Every multiply has its own stage, split hi/lo. v0.1 had a subtract
  feeding the overlay multiply ((959 − a)·(959 − b)). On normalized luma
  that is just `not a · not b`.
- The chroma mix happens early (stages 2–7), so one mixed chroma pair is
  carried to the noise stage instead of raw + posterized pairs.
- The ghost ring is one EBR with `no_rw_check` (read = write − offset,
  offset ≥ 2, so they never collide).

## Hardware checks for v0.3.0

- S11 Blend in each mode: is the colour bold enough, or too much? Is
  Multiply's hue doubling the wild look you wanted?
- S7 Ghost + S11 Blend (Colour Ghost preset): colour emboss.
- S11 Posterize should look exactly like v0.2.

## Hardware checks for v0.2.0

1. Default look (Difference, 3 bits): greys should be neutral now (v0.1 had
   a pink cast). Black and white posterize levels are real black/white.
2. S7 Ghost in each blend: is the 1/16-width ghost the right size? Does it
   read as VHS ghosting?
3. S9 Roll: does the band wrap smoothly off the top and back in at the
   bottom?
4. P12 sweep: fine grain → snow. Is 100% the right climax?
5. S10 Colour: VHS-like colour streaks, instead of v0.1's blue/orange-only
   noise?
6. Brightness: whites now peak at ~775 (the bar fix). Does it read too dim?
7. HD HDMI: any thick bars (try Static high, Blend Add)? Any green cast or
   noise lines? (Raw video is carried into the mix stage, the same as v0.1.)
8. K1/K2/K5 labels: does the display match the picture at every step?

## Review backlog (not done)

Tier 2, worth doing after hardware feedback (your call: each changes a look
or a control):
- **Head-switch tear:** band rows shift sideways, the shift growing toward
  the bottom of the band. This is the iconic VHS head-switch look; today's
  band is only noise. It needs a ~256-px delay ring (2 EBR) and could ride
  on K4 with the noise, or on S8.
- ~~Chroma blends~~: done in v0.3.0.
- **Invert instead of Ghost:** S7 could instead blend with the original's
  negative. That is bold in Multiply/Screen/Difference (solarization), but
  Add degenerates to a near-white field, which is why Ghost was chosen.
- **Ghost distance:** fixed at 1/16 width. It could ride Mix's upper range,
  or use a second offset for a double ghost.
- **Chroma delay/bleed:** VHS colour smears right of edges. Cheap to add,
  using the same ring idea on chroma.
- **Band chroma:** the band could lose its chroma (head-switch chroma kill)
  in Normal mode.

Tier 3, optional:
- Capture defaults and presets from hardware (the current 5 are v0.1's,
  translated, plus "Ghost Relief").
- A band position knob (fixed mode only), or a roll speed.
- Luma noise held 2 px (VHS luma bandwidth) for a softer grain.

## Build

`./build_combo.sh vhsmix` → `out/rev_b/ron/catalogue-combo/vhsmix.vmprog`.

v0.3.0 (2026-10-13): all 6 configs pass on seed 1 (router2 default).
HD Analog 88.8, HD HDMI 84.8, HD Dual 85.9 MHz; SD Analog 88.2, SD HDMI
80.5, SD Dual 80.9 MHz. 6,786–6,806 LC (89%), 2/32 EBR. Standalone yosys:
3,847 LUT4 (+1,043 for the colour blends: four 8×8 signed products, the
wider chroma mix, the select), `emulate_transparency` 0. Headroom is now
~5% to the ~94% closure ceiling, so the head-switch tear (Tier 2) would
need LC reclaimed first: e.g. 6-bit colour products, or carrying the raw
chroma in a 16-deep EBR delay instead of FFs.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 82.5, HD HDMI 85.1, HD Dual 83.6 MHz; SD Analog 88.7, SD HDMI
82.0, SD Dual 87.1 MHz. 5,542–5,565 LC (72%), 1/32 EBR. Standalone yosys:
2,804 LUT4, `emulate_transparency` 0.

v0.1.0 (2026-09-11): HD Analog 76.5 (seed 3), HD HDMI 74.4 (seed 2),
HD Dual 76.6 MHz (seed 2); SD 72.7 / 70.5 / 78.6. 4,719 LC (61%), 0 EBR.

## v0.1.0 backup

Git 3fa438d (sources unchanged since 2026-09-11). The sources and vmprog are
also in `.vhsmix_work/v010/`.
