# Gouache — summary

Catalogue combo #48 (seed 20260920): posterize + hue rotation + saturation.
v0.1.1 (2026-09-11) was flashed and liked ("gouache is good"). There were no
later hardware notes. v0.2.0 (2026-10-12) is the finalizing pass, built from a
review of v0.1.1 while Ron was away. It is **not flashed yet**.
v0.1.1 sources: git 3fa438d, and `.gouache_work/v011/`.

Build: `./build_combo.sh gouache` → `out/rev_b/ron/catalogue-combo/gouache.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Luma | luma levels, 1–8 bits |
| K2 Chroma | chroma levels, 1–8 bits |
| K3 Dither | dither before quantizing, 0–100% of a step, centred on the threshold (default 25%) |
| K4 Hue | base hue rotation, 0–360° |
| K5 Sat | saturation 0–400%, 100% = unity (default 150%) |
| K6 Spread | extra hue rotation per band: rainbow bands (default 40%) |
| S7 Order | Post-Rot: posterize, then rotate. Rot-Post: rotate, then posterize |
| S8 Sat | gain Before the quantizer (crush/boost the bands) / After |
| S9 Bands | Spread keyed by Luma band / by Hue sector (8 × 45°) |
| S10 Dither | Bayer 4×4 / Noise |
| S11 Invert | alternate luma bands go negative |
| P12 Spin | hue spin speed, bipolar, squared. Centre (±8 codes) = still; the ends ≈ 0.55 s per turn |

## v0.2.0 changes (Tier 1 of the review)

Bugs:
- **Dither brightened the picture.** Luma dither added 0 to +1 step before
  a floor quantizer. At 1 luma bit, Dither 100% turned mid-grey from 256 into
  mostly 768. Dither is now centred (−½ to +½ step) on luma and chroma, so
  it adds texture without shifting brightness. With Dither up, pictures are
  darker than v0.1 (correct exposure).
- **S11 Invert flattened the picture at 1–2 luma bits.** At 1 bit both bands
  landed near 256. At 2 bits the middle two bands merged. Below 3 bits it now
  alternates on 128-code bands of the dithered luma, which gives a zebra. At
  3 bits and above it is the same as before, including the Op Art preset.
  The negative is now a mirror about the video range (64 ↔ 940, clamped).
  Before, it was 1023 − y, which sent inverted blacks superwhite (959).
- Luma ran one clock behind chroma.
- Noise dither read 4 adjacent LFSR bits. Each pixel's value was then about
  2× the previous pixel's value (streaky). The taps are now spaced (15, 10, 5, 0).

Controls (bold, always-on):
- **Switches did nothing at power-on.** This was the shelf audit's open
  issue. Dither 0, Sat 100% and Spread 0 left S8, S9 and S10 inert, and S7
  was inert too, since nothing rotated. The defaults are now Dither 25%,
  Sat 150% and Spread 40%. The presets pin K3 and K6, so they look as they
  did in v0.1.
- **S9 Chroma was weak.** It keyed bands on the top bits of U alone, so
  real video split into about 2 bands (bluish vs yellowish). S9 is now
  **Hue**: 8 × 45° hue sectors in angular order, with near-greys
  (|U|, |V| < 16) in sector 0. At Spread 100%, each sector turns 45° more
  than the one before it, which roughly doubles every hue.
- **P12 Spin topped out at about 4.3 s per turn.** It now uses a squared
  curve with a centre deadband. The ends run about 8× faster (≈0.55 s per
  turn), and the centre region is finer. The preset P12 values were
  re-solved for the same speeds: Rainbow Bands 600 → 595, Crushed Spin
  900 → 678.
- S9's labels are now Luma / Hue.
- The categories are now Color / Material / Print. Colorizer, Stylize and
  Retro are not in the schema list.

Timing / fit:
- In v0.1.1, the HD Analog critical path was the 11×10 rotation multiply,
  done in one stage. It ran at 75.2 MHz, under 1 MHz of margin. The
  sine-ROM operand is now split hi/lo: eight 11×5/11×6 products, then a
  recombine stage. Latency went from 18 to 19, the same in all modes.
- Each quantizer's "+½ step" is now an OR, since the masked low bits are
  zero. This removes 3 adders.

Build results: see the bottom of this file.

## Review backlog

### Tier 2: needs Ron's call (these change the look)
- **A. Grey cast (chroma quantizer is mid-rise).** Zero chroma rounds to
  +½ step on both U and V, so greys get a tint. At the default 3 bits this
  is U = V = +64, a purple cast. At 1 bit every pixel is one of 4 strong
  colours. The same tint is also what lets Spread and Spin colour a
  monochrome source. Options:
  - keep it as is;
  - a neutral mid-tread quantizer, so greys stay grey (mono sources stay mono);
  - an explicit Tint seed (a fixed chroma vector that the bands rotate).
- **B. Luma levels are mid-rise too.** At 1–2 bits, black lifts to 256/128
  and white sits at 768/896. Colour stays visible in every band, which suits
  a colour program, but there is no true black. The option is to anchor the
  levels on 64 and 940.
- **C. HD HDMI bars check.** 1-px Bayer dither at 1–2 luma bits on moving
  video matches the bar trigger profile exactly. Look at HD HDMI with
  Dither up, Luma at 1–2 bits, and motion. If it bars, there are two fixes:
  - the damier guard ([1 2 1] + luma scale), which also blurs the 1-px
    dither horizontally;
  - a 2-px dither grain on HD (see D).
- **D. Dither grain on HD.** A 4×4 Bayer at 1 px on 1080p reads as almost
  flat grey from the couch. A 2-px grain (8×8 px cells) would make the
  retro texture visible. This could apply on HD only.
- **E. True negative.** S11 inverts luma only. Inverting chroma on the same
  bands (complementary colour) would make it a full negative, visible at
  Sat 0.
- **F. Presets.** Capture them from hardware once looks are dialled in
  (`./capture_preset.sh`). 4 slots are free.

### Tier 3: polish and optimizations (small)
- **a. K1/K2 readouts can be off by one bit.** The hardware changes bit
  depth every 128 codes, but the display rounds 1 + 7v/1023. Fixes: value
  labels (1–8 bits) with the labelled-default index guard, or centre the
  hardware zones.
- **b. Sat knob.** Unity sits at 25% of travel, so boost takes ¾ of the
  knob. A proc-amp curve (unity at centre) would be symmetric. Presets would
  need translating.
- **c. LC.** The Rot-Post dither travels 13 stages as an 11-bit word
  (143 FFs). Carrying the 4-bit code instead and multiplying at the end
  would save about 80 LC. Only worth it if LC is needed.
- **d. 10-bit hue angle** would make very slow spins smoother. Marginal.
- **e. SD interlace.** The Bayer row is keyed on the field line, so both
  fields share rows and the pattern doubles in height on 480i. A fix needs
  interlace detection. HD is unaffected.

## Hardware check list (v0.2.0)
1. Power-on default: posterized, mild dither, rainbow hue steps per luma
   band. Each switch S7–S10 should visibly change it.
2. S9 Hue at Spread 50–100%: colours re-hued per 45° sector (hue doubling).
3. P12: still near centre; slow creep just off centre; fast colour cycling
   at both ends.
4. S11 at Luma 1–2 bits: a zebra, not a flat picture.
5. Dither up: same overall brightness as Dither 0, just textured.
6. HD HDMI: any thick bars with Dither up at Luma 1–2 bits in motion (C).

## Build results
v0.2.0 (2026-10-12), router2, every config on seed 1:

| Config | Fmax (MHz) | v0.1.1 |
|---|---|---|
| HD Analog | 81.59 | 75.18 |
| HD HDMI | 84.62 | 78.75 (seed 2) |
| HD Dual | 75.57 | 75.51 (seed 2) |
| SD Analog / HDMI / Dual | 82.75 / 72.88 / 87.21 | — |

LC is 5,740–5,761 of 7,680 (~75%, was 5,537), with 2 EBR. HD Dual is the
thin one, at 1.3 MHz of margin. It is a real seed-1 pass, but it is the
config to watch if Tier 2 adds logic.
