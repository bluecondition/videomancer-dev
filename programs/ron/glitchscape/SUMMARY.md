# GLITCHSCAPE (v1.0.0 — pixel-sort glitch landscape)

**STATUS: FINAL except presets — signed off by Ron 2026-10-09** ("this all
looks good and can be finalized"). Remaining: the four presets are
hand-typed placeholders from v0.1. Capture real ones from the device with
`./capture_preset.sh` and rebuild. All 6 configs closed on seed 1 (router2).

## What it does

It databends the live source into the "Glitch Landscape" reference: the
bright parts of the image become a dense mass of vertically streaked,
saturated colour bars floating on a flat sky. The sky is the signature teal
#18A9A6, or a matching colour for each palette. White hot-zone cores, white
contour filaments, a torn top edge and coloured bars pouring down the frame
complete the look.

## Engine

* **Phase field.** A per-pixel DDA (`xacc += step`; or a per-row DDA in
  Horizontal mode) plus luma bend (`lc8 * bk`), two diagonal triangle-wave
  ripples and a leaky per-row random walk. Every offset is expressed in
  PHASE (65536 = one bar pitch), so its amplitude is measured in pitches.
  The bits 24..16 are the bar index, and its xor-hash picks one of 8 palette
  hues. The bar is 3/4 duty: `not (F15 and F14)`. Bar luma is
  64 + lc·11/16 (≤ 765), so the colour survives on hardware.
* **Mass.** Crushed luma (black point K2, ×1.5) ≥ K1 + a torn-edge offset.
  The offset is a 4-px column hash (×0x6D+0x3B, ^>>4, ×0x35, ^>>5) × 1..8,
  set by P12. Anything outside the mass is sky.
* **Contour filaments (K6).** Luma band = (lc·K6>>4)>>10, giving 0..63
  bands. A pixel turns white where its band differs from its left
  neighbour's or from the band above it. The band above comes from a
  2048×4 single-bank line buffer (the write data never depends on the read).
* **Drips (S9) = vertical pixel sort.** Each column stores (hue index, bar
  bit, energy = lc>>6 of mass pixels) in 2× 2048×8 EBR, ping-ponged by row
  parity, because the new state depends on the read and yosys can't infer a
  read-modify-write RAM. The state from above wins whenever its energy,
  minus a stochastic decay of 1 with odds 16..1/128 set by P12, beats this
  pixel's energy. Coloured bars therefore pour down over darker image and
  sky and end raggedly; drips run ~120 lines at P12=0 and ~1900 at full.
  They are substituted before the palette and saturation stages, so they
  share them.
* **Compress (S10) = macroblocks.** 16×16 blocks at HD, 8×8 at SD. Each
  block's colour is point-sampled on its first row into 256×16 (u8,v8) +
  256×8 (y8) EBR and held for the whole block. Output = block chroma, with
  luma = (posterized + block)/2. The posterize depth comes from P12 in
  4 zones, deadbanded.
* **Hardware contracts.** Blanking gate; a generated, parity-locked avid
  (the aurora/cubist recipe; width = max_px per field); height captured on
  active lines; a serrated-vsync `vs_armed` guard; hot zones keyed on RAW
  luma; xorshift32 noise with disjoint bit fields. No raw video is carried:
  the sync pipe holds syncs only. LATENCY 27, the same in every mode.

## Controls

| Ctl | Label | Does |
|---|---|---|
| K1 | Mass Key | luma level above which the coloured mass appears |
| K2 | Contrast | black-point crush before keying |
| K3 | Saturation | palette chroma gain 0–400 % (100 % at 1/4 turn), continuous |
| K4 | Palette | Jewel / Neon / Vaporwave / Inferno / Toxic / Glacier / Candy / Mono, each with its own sky |
| K5 | Line Spacing | bar pitch 64 → 4 px, exponential, glide-filtered |
| K6 | Filaments | number of white luma-contour lines, 0–63 |
| S7 | Channel Sep | U/V fringe 4–7 px (amount follows P12) |
| S8 | Hot Zones | blown-white cores + bloom halo at the brightest source |
| S9 | Drips | bars pour down the frame (vertical pixel sort) |
| S10 | Compress | JPEG-style macroblocks + posterize |
| S11 | Sort Axis | Vertical / Horizontal streaks |
| P12 | Glitch Intensity | bend, ripple, row wander, edge tear, fringe, bloom, hot-zone reach, drip length, grain, all continuous |

## Status / builds

v1.0.0, 2026-10-09: 6/6 pass on seed 1 (router2). HD Analog 87.3, SD Analog
86.7, HD HDMI 83.9, SD HDMI 86.7, HD Dual 79.1, SD Dual 78.6 MHz. 5460 LC
(71 %), 12/32 EBR.

## History

* **v0.1 (2026-06-28)**: streaming iso-bar fake pixel-sort, 0 BRAM; never flashed.
* **v0.2 (2026-10-09)**: blanking gate + generated avid; Bypass removed
  (16-deep raw tap) and replaced by S11 Sort Axis; the act_h=0 drip bug
  fixed; hot key on raw luma; luma cap 768; vsync guard; phase-DDA field;
  continuous P12/K3/K5; deadbanded K4/P12 zones; xorshift32. The first build
  missed HD HDMI because the final luma stage did quantize+dim+grain+clamp in
  one cycle; splitting it fixed that.
* **v0.3 (2026-10-09)**: contour filaments, line-buffer drips, macroblock
  Compress, torn edge, 8 palettes. The first build missed HD at 65–67 MHz on
  every seed: `16 - P12/68` in the frame latch is a 15 ns divider; /64 fixed it.
* **v0.3.1**: a sky colour per palette (dark skies' chroma boosted so they
  read as colour). **v1.0.0**: version bump at sign-off.

Backups of v0.1/v0.2/v0.3 (vhd, toml, vmprog) are in `.glitchscape_work/`.
