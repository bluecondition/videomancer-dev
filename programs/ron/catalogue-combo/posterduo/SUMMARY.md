# Posterduo — summary

Catalogue combo #112 (seed 20260925): posterization + duotone / tritone.
v0.1.0 was flashed and liked ("posterduo is good", 2026-09-12). v0.2.0
(2026-10-12) is the first finalizing pass, built from a review of v0.1.0.
The v0.1.0 sources are in git 3fa438d (copies in `.posterduo_work/`:
`posterduo_v010.vhd`, `duotone_v010.vhd`); its vmprog is
`.posterduo_work/posterduo_v010.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Luma | posterize depth, 1–8 bits. Black and white are always levels |
| K2 Dither | 2×2 ordered dither, 0–100% (±⅜ of a step at full). Default 30% |
| K3 Pitch | dither cell 2 / 4 / 8 / 16 px |
| K4 Shadow | shadow ink, 8 hues 45° apart |
| K5 Light | highlight ink, 8 hues |
| K6 Contrast | tone-ramp contrast, 0–400% (100% = the ramp spans black..white) |
| S7 Order | Post-Tone: inks from the posterized luma (one ink mix per band) / Tone-Post: inks from the smooth luma (a smooth ink ramp under the luma bands) |
| S8 Tritone | midtones fade to neutral grey |
| S9 Luma | Keep (posterized luma) / Flat (luma = the tone ramp; posterized and dithered in Tone-Post) |
| S10 Bands | alternate posterize bands swap the two inks |
| S11 Chroma | Smooth / Poster: each chroma channel snapped to −200 / 0 / +200 (nine colours, grey included), dithered by K2. Works in both orders |
| P12 Balance | the ink split sweeps through the picture: 0% high-key (mostly light ink), 50% split at mid-grey, 100% low-key (mostly shadow ink). Glided |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- The posterizer lifted black and capped white. It kept the top L bits and
  added half a step, so 1 bit gave two greys (256 / 768) and 2 bits gave
  128..896. It is now flutter v0.2.0's quantizer: luma normalized to the
  video range ((y − 64) × 1.168), cut to L bits, bit-replicated, scaled back
  (× 0.855 + 64). Black (64) and white (~940) are always levels.
- Dither brightened the picture. The 2×2 offsets were 0, ¼, ½, ¾ of a
  step, one-sided, into a quantizer that already rounds, so full dither
  added ⅜ of a step on average (+190 codes at 1 bit). The offsets are now
  centred (−⅜, −⅛, +⅛, +⅜) and scaled by K2.
- K2 Dither and K3 Pitch were dead in Tone-Post. Its luma came from a
  second, undithered quantizer. Tone-Post now uses the dithered posterized
  luma, and Tone-Post Flat dithers the ramp before posterizing it.
- S11 Poster had no neutral level. It masked U/V with the luma step mask
  and added half a step, so grey became U = V = 768 at 1 bit (640 at 2
  bits). That is a strong cast on every neutral, including the tritone
  midtones. It also did nothing in Post-Tone. Poster now snaps each channel
  to −200 / 0 / +200 about 512, in both orders, dithered by K2. In
  Post-Tone it turns the band colours into a 3-4 ink screen print; in
  Tone-Post it breaks the smooth ramp into hard ink zones.
- S10 Swap was misaligned. One band bit was used at two duotone stages 2
  and 4 clocks apart, so wrong-colour fringes ran 2–4 px into each band at
  Pitch ≥ 2. The swap now inverts the ramp for the inks at the exact stage
  (shadow + (light − shadow)·t with t → 1 − t is the swapped pair).
- P12 Balance wasted its top end. The midpoint ran from black (0%) to past
  white (from ~85% the whole picture flooded to the shadow ink), and 0%
  never reached the light ink. The midpoint now runs 48..975 on the
  normalized scale, so the split always sits inside the picture, with 50%
  at mid-grey. A ¼-per-field glide stops pot noise from moving the ink
  edges.
- Flat luma topped out at 959 (above white). It now uses the same 64..~940
  scale-back as the posterizer.

Hardware contracts:
- Generated avid (cubist / aurora recipe). The output chroma is drawn by the
  program (the input chroma is discarded), which is the aurora case: a
  source avid that lands a clock off its usual hsync parity swaps U/V for
  that line on HD HDMI (thin coloured lines). The start is parity-locked and
  the width is the field's longest line.
- HD HDMI bar conditioning, as on damier v0.4.4: [1 2 1] on Y/U/V, then luma
  scaled to white ~777. Posterized and dithered live video is all sharp,
  near-full-swing edges in motion, the known trigger. Drawn luma now stays
  under ~780 (the ~768 cap).
- Latency 14 → 20, the same in all modes. Blanking gate on both the source
  and the generated avid.

Controls:
- K3 Pitch is 2 / 4 / 8 / 16 px, labelled (was 1 / 2 / 4 / 8). The [1 2 1]
  conditioning would flatten a 1-px dither to a flat mid-level, and 1-px
  dither is invisible at 1080p anyway. The labelled default is honoured as
  the firmware's label index until the knob first moves.
- K2 Dither defaults to 30% (was 0), so Pitch is live at power-on: thin
  dithered fringes along the band edges, flat bands otherwise.
- K1 / K3 / K4 / K5 zones are hysteretic (a value changes only 12 counts
  inside a new zone), so a pot resting on a boundary doesn't flicker the
  whole picture. K4 default 500 → 448 (500 sat in the hysteresis margin).
- Interlaced SD: dither cells key on the frame line, not the field line.

Timing:
- v0.1.0 only just closed HD: HD Analog on seed 6 (75.7 MHz), HD Dual on
  seed 2 (74.3). The duotone ran 12×9, 10×11 and 11×11 multiplies one per
  stage. Now: contrast product split on the K6 nibbles (12×5 ×2), ink
  products on a 6-bit ramp (10×7), tritone weight 7 bits (11×8). The input
  chroma pipe is gone (it was never used).

## Build

`./build_combo.sh posterduo` → `out/rev_b/ron/catalogue-combo/posterduo.vmprog`.
v0.2.0 (2026-10-12): 6/6 on seed 1 with router2. HD 80.9 / 85.0 / 89.1 MHz
(Analog / HDMI / Dual); SD 77.8 / 87.4 / 84.4 MHz (27 MHz needed). ~3,970 LC
(52%), no RAM. v0.1.0 was 3,724 LC, HD Analog 75.7 on seed 6, HD Dual 74.3 on
seed 2. NOT FLASHED, NOT COMMITTED.

## Open / next (not built)

- K4 / K5 hue names (value_labels). Needs a hardware check of which colour
  each index shows (the U/V swap convention), or a calibrated, named 8-ink
  table (scope-verified rainbow palette, saturation tuned per hue).
- 4×4 Bayer instead of 2×2: 17 levels between two steps instead of 5,
  smoother gradients, and a recognisable halftone at 16 px.
- Tritone third ink: midtones take a hue (e.g. halfway round the wheel
  between the inks) instead of grey.
- Ink path round the hue wheel (rainbow / Warhol bands between the two
  inks) as a mode.
- Inks are ±200 (~45% saturation); a bolder set, or saturation per hue.
- In Tone-Post Flat the band swap keys on the luma bands, not the visible
  ramp bands.
- Presets are hand-written (4 of 8, "Default" = power-on). Capture them
  from the device once the look settles.
