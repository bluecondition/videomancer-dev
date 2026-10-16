# Flutter — summary

Catalogue combo #60 (seed 20260921): magnetic warp (wobble) + posterize.
v0.1.0 was flashed and liked ("flutter is good", 2026-09-12); v0.1.2
(2026-09-13) swapped the edge fill for the address clamp and reworked the
clamp's timing, and was never re-checked on hardware. v0.2.0 (2026-10-12)
is the first finalizing pass, built from a review of v0.1.2. The v0.1.2
sources are in git 3fa438d; its vmprog is in `.flutter_work/flutter_v012.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Wobble | displacement amplitude, 0–255 px |
| K2 Waves | vertical wavelength: line-phase step, 0 (the whole frame sways together) to 64 lines a cycle. Glided |
| K3 Speed | bipolar, squared, ±2.3% centre deadband. The v0.1 top speed (one cycle in ~4 s) is now at ~30% from centre; the top is ~15× that (~3.5 cycles a second) |
| K4 Luma | luma levels, 1–8 bits. Also the bands: each posterized level slides as one band |
| K5 Chroma | chroma levels, 1–8 bits (1 bit = −/grey/+ per channel) |
| K6 Spread | how far apart the bands are: up to (N−1)/N of a cycle (Phase), or a gain of 0.125–1.12 (Amp) |
| S7 Bands | Phase: band k of N is k/N of a cycle × Spread ahead / Amp: band gain 0.125 + level × Spread (dark bands calm, bright bands swing) |
| S8 Wave | Smooth: sine / Snap: a flat-topped wave (255·tanh(4s)/tanh(4)), so lines jump between two offsets |
| S9 Chroma | Together / Quarter: chroma reads a quarter wave apart (fringes) |
| S10 Edges | Fill: reads off the line take its own edge pixel / Mirror |
| S11 Dither | Off / On: a 2×2 ordered dither, applied before the wobble so it travels with the picture |
| P12 Twist | horizontal phase step: 0 = pure line wobble, up = 2-D ripple (257 px a cycle at 100%). Glided |

## v0.2.0 changes (from the v0.1.2 review)

Bugs:
- The chroma posterizer had no neutral level: it put levels at (k + ½) ×
  step, so grey (512) became U = V = 576 at the default 3 bits, a
  pink/magenta cast over every neutral, and U = V = 768 at 1 bit. It is now a
  centred mid-tread quantizer: levels at 512 ± k × step, clamped 64..960.
- The luma posterizer lifted black at low depths (64 → 128 at 2 bits, → 256
  at 1 bit) and topped out at 768 at 1 bit, so 1 bit gave two greys. Luma is
  now normalized to the video range ((y − 64) × 1.168), cut to L bits,
  bit-replicated, and scaled back (× 0.855 + 64), so black (64) and white
  (~940) are always levels. 3 bits looks nearly as before.
- S8 Order did nothing unless Dither was on: a pixel shift commutes with a
  per-pixel quantizer, so "posterize then wobble" and "wobble then
  posterize" gave the same picture. Posterize now happens once, on the
  write side, and S8 is Wave Smooth / Snap.
- Pot noise on K2 and P12 jumped the picture. One step of the 8-bit Waves
  value moved the phase at the bottom of a 1080-line frame by 0.066 of a
  cycle, which is up to ~100 px at full Wobble. One step of Twist moved the
  right edge by up to ~47 px. Both now use all 10 bits (4× finer) and glide
  by ⅛ of the gap per frame (the phases are 18 bits).

Hardware contracts:
- U/V are swapped on rows whose hsync→avid parity differs from the row
  above. Each row replays the row above from the line buffer, the same case
  as drape v1.3 and benday.
- Read and write banks are latched per line at the avid rise. The read
  pipeline's tail now runs ~19 clocks past the avid fall, and that is past
  hsync on the SD/PAL front porch (12–16 clocks).
- HD HDMI bar conditioning, as on damier v0.4.4: [1 2 1] on Y/U/V, then luma
  scaled to white ~777. Posterized steps are near-full-swing edges that move
  every frame, which is the known trigger.
- no_rw_check on all six line-buffer RAMs. A bank is only read while the
  other one is written, and the write bank's read data is discarded.
- Latency 14 → 20, the same in all modes.

Controls:
- Bands follow K4: the band is the pixel's posterized luma level, so each
  flat poster region slides as one piece. At 1 bit, light and dark split
  half a cycle apart at full Spread. At 8 bits the phase follows luma
  smoothly. The default 3 bits gives the same 8 bands as v0.1.
- Speed is squared, as above. The default (600 → 584) and Ripple Fringe
  (700 → 611) were moved so they keep their v0.1 speeds.
- K4/K5 change depth only 12 counts inside a new zone (hysteresis), so a pot
  resting on a boundary doesn't flicker the whole picture.

## Build

`./build_combo.sh flutter` → `out/rev_b/ron/catalogue-combo/flutter.vmprog`.
v0.2.0 (2026-10-12): 6/6 on seed 1 with router2. HD 79.4 / 81.6 / 81.1 MHz
(Analog / HDMI / Dual); SD 72.6 / 74.3 / 82.2 MHz (27 MHz needed). 4,161 LC
(54%), 22 EBR, and no transparency emulation (yosys `emulate_transparency`
count 0). v0.1.2 was 3,304 LC, and its HD HDMI barely passed at 74.3 MHz.
NOT FLASHED, NOT COMMITTED.

## Open / next (Tier 3, not built)

- K1 range: 255 px is only 13% of an HD line. A squared curve to ±511 px
  would keep fine control low and give a bigger top.
- Amp mode bipolar: alternate bands swing opposite ways, so the bands shear
  apart.
- Twist / Waves in screen units (cycles per width / height), so SD and HD
  show the same ripple count.
- The top row shows the previous frame's bottom row. The first active line
  reads a bank last written in the previous frame. It's 1 line, and v0.1
  did the same.
- Chroma is stored at 2-px resolution. Full resolution costs +4 EBR.
- Presets are still hand-written (4 of 8). Capture them from the device once
  the look settles.
