# GRIST — film grain to mezzotint

Status: **v0.3.0 timing-closed, not yet flashed** (2026-10-01).
Processing program. Morsel's dough-grain hash promoted to a whole instrument:
three hash octaves + pores feed one comparator,
`out = clamp(512 + (Y - thr + N) * gain)`, and P12 DEVELOP rides the gain
from transparent additive grain (1x) to a hard mezzotint (~72x).

## Build (v0.3.0, router2, all seed 1)

| config | Fmax | LC |
|---|---|---|
| HD Analog | 82.7 MHz | 5324 (69%) |
| HD HDMI | 83.4 MHz | 5325 |
| HD Dual | 85.1 MHz | 5329 |
| SD Analog / HDMI / Dual | 74.9 / 81.0 / 81.3 MHz | ~5330 |

0 EBR. v0.2.0 was 3735 LC (HD 77–83 MHz); v0.1.0 3120 LC (HD Analog 76.2,
HD Dual seed 3 only).

## Controls

K1 Fine / K2 Coarse (grain amounts; at the climax, the octave MIX: fine spray
vs clumped aquatint), K3 Block 8/16/32/64 px, K4 Response (mid-tone weighting),
K5 Pits (bipolar: left dark pits, centre none, right bright sparkle), K6
Exposure, P12 Develop; S7 Animate, S8 Soft, S9 Mono, S10 Etch, S11 Invert.

## v0.3.0 — S8 Soft (2026-10-01)

The pixel grain had two problems: it is one pixel wide, and morsel's
xor-rotate-add hash has strong structure. The fine nibble's spectrum peaks
at the vertical Nyquist (row alternation), 254× the mean, against ~12× for
white noise, which reads as a woven checker. Two- and three-round variants
of the same family were still 65–155×.
- **S8 Soft** box-filters the fine and 4 px octaves over 4×4 px. Four hash
  rows (frame rows y..y+3) give the column sums and horizontal delay taps
  give the 4-tap sum. There is no line buffer, so the field stays coherent
  under interlace. The 4-tap box has a zero at Nyquist, which nulls the
  row-alternation peak. The sum is re-centred (>>2) to the pixel nibble's
  std and clamped to −8..7, so the climax renormalisation is unchanged.
- The K3 block octave is bilinearly interpolated from 4 corner hashes, with
  ×1.5 to restore the spread that interpolation loses.
- Pits come from a third box-filtered field (pore nibble), thresholded at
  normal quantiles, so they print as blobs.
- Sparkle moved off S8: K5 is bipolar, and `|K5−512|>>5` is a bit-slice.
- The latency is 20 in both S8 positions; pixel-mode values ride a delay line.
- Presets: K5 remapped to keep their pit density and polarity; all have
  Soft off.
- Prototype (Python, climax threshold over a ramp): soft box4 prints organic
  ink-spatter blobs; binomial [1 3 3 1] gave smaller, pixel-ier blobs.

## v0.2.0 changes (finalization review)

- **Mezzotint climax fixed.** For stipple density to track the picture, the
  dither must span the whole luma range. v0.1 spanned ±92 at defaults (±240
  with K1/K2 max) against ±448 of luma, so full Develop was a hard two-tone
  posterize with a thin grainy contour. Now a vblank serial divider computes
  `A_full = 7040 / span` and Develop ramps `A_eff = 16 → A_full`. The octave
  gains are renormalised per frame (`gf' = gf·A_eff/16`), so the pixel path
  still uses small 5×8 multiplies. Climax span is ±416–436 at any K1/K2
  setting (sweep-tested at RTL widths). Pore push ramps 144 → 576 so pits
  survive the wider dither.
- **Colour climax.** Over the upper half of Develop, dark dots drain chroma
  (>>1, >>2, >>3, then 0) and the white ceiling drops from 940 to 813, so
  colour mode becomes coloured stipple on black rather than tinted mud.
  Below 50% Develop, colour is untouched.
- **Hardware contracts.** Blanking gate (64/512/512 on the latency-aligned
  avid). Luma clamped to 64..940 (v0.1 output 0..1023). Invert is now
  `940 − (y − 64)`, and Etch is 7 plates inside the legal range.
- **Vsync serration.** A saw-active guard on the field counter and interlace
  detector; v0.1's detector flipped `s_ilace` off on the second serration edge.
- **1024-px hash repeat.** `f_pack` sees only x(9:0)/y(9:0), so on 1080p the
  fine octave and pores repeated exactly at x+1024 and y+1024. A per-region
  constant (from x(11:10), y(10)) is now ADDED to the pack; an XOR would only
  xor-translate the texture, because the pack is GF(2)-linear. Max
  cross-correlation between regions is 0.012 (was 1.000). Region 0 keeps
  morsel's exact texture.
- **Animate** reseeds every second field/frame (~30/25 Hz, film-like rather
  than 60 Hz snow); under interlace it reseeds on one field, so both fields
  share a texture.
- S8 Sparkle got a 1/16 pore floor (superseded in v0.3: Sparkle moved to
  the right half of a bipolar K5).
- Gain normalisation top branch fixed (`m = g10(10:5)`, was 63 → 2016).
- Latency 10 → 12, the same in every mode.

## Still to do

- First flash: sweep Develop (check the climax reads as tonal stipple), Mono
  on/off at the climax, Animate rate, Etch, Invert, SD analog interlace.
- Retune presets after HW eyes. Develop now also widens the grain, so the
  low-Develop presets (Film Stock 180, Star Dust 250) carry ~1.6–1.9× more
  grain than in v0.1. Capture retuned looks straight from the device with
  `./capture_preset.sh --preset N --name "Name"` (`--dry-run` first), then
  rebuild.

## Exploration history (2026-08-01)

`explore_grain.py` isolates morsel's hash bit-exactly and pushes each axis to
an extreme (`ext_*.png`, synthetic ramp+disc). The standout was **g, the
binary mezzotint** (luma vs dithered threshold, threshold span ±384). Other
looks: d corroded pores, e sparkle, f 4-octave concrete, h film response,
i carve-posterize. Weak: b amplitude-max (generic noise), c 64-px blocks
(turns into a mosaic).
