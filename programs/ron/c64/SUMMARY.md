# C64 — Commodore 64 re-render (v2.0)

**Status: v2.0.0 FINAL except presets — signed off by Ron on hardware 2026-10-07.**
Defaults captured from the device (8x, Border 296, Scanlines 442, Color 552,
Contrast 604, Stability Off, Multicolor On, Nearest, Loading Off, Ready On,
Dither 84). Presets are hand-written placeholders.

## What it does
Redraws incoming video as a Commodore 64 screen: chunky blocks in the 16
Colodore colours on a native 320×200 grid sized from the measured raster,
optional multicolour (double-wide) pixels, a C64 border sized like the real
machine's, per-block ordered dither on the slider, tape-loading stripes in
the border, a READY. prompt with blinking cursor, and block-row scanlines.

## How it works
- Pixelation is the CGA program's HW-proven scheme: each cell averages its
  first 2^k pixels on the first line of its cell row; the average goes
  through a mid-pivot contrast, a chroma gain, an optional value snap and a
  Bayer offset, then a 5-bit Manhattan nearest-colour match against all 16
  colours (or an 8-zone luma ramp). The index is stored per column in BRAM
  and replicated down the row.
- Stability: ping-pong 32×16-px tile grids (8 bits per tile, 16 EBR).
  Dither off → decision hysteresis (margin + two-field debounce). Dither on
  → value hysteresis (tile stores luma bits 9:2; a cell snaps within ±20
  codes before dithering). Any decision-side control change re-seeds the
  grids for two fields.
- Palette: Colodore, BT.601 re-levelled to black 64 / white 768, stored
  U/V-swapped for the hardware.
- Border: fractions of the measured raster (K2 = k/1024 of the width per
  side, 1.6× that of the height — the C64's 32/384 : 36/272 proportions).
- Output is gated to neutral outside active video; the field's first line is
  blanked. Latency 7 in all modes. No raw video is carried past the input
  register (Mix/Bypass removed).

## Controls
- K1 Size — 2x / 4x / C64 (native 320×200) / 8x / 16x
- K2 Border — border size; 0 = none, default = authentic C64 proportions
- K3 Border Col — 16 C64 colours; also the READY. text colour
- K4 Scanlines — dims the last line of every block row (0 = off, 100% ≈ black)
- K5 Color — chroma gain before matching: 0 → greys only, 50% = unchanged,
  100% → 5×
- K6 Contrast — around mid-grey, 50% = unchanged
- S7 Stability — anti-flicker on/off
- S8 Multicolor — double-wide pixels (160×200 at native size)
- S9 Color Mode — Nearest (all 16 colours) / Ramp (luma → black, blue,
  purple, lt blue, lt red, lt grey, yellow, white)
- S10 Loading — tape-loading stripes in the border (forces at least the
  default border width)
- S11 Ready — READY. prompt + blinking cursor at the top-left of the screen
- P12 Dither — ordered 4×4 Bayer per block, 0 → flat, 100% → ±240 codes

## Presets
Boot Screen · Game Art · Tape Loading · Demo Ramp · Big Pixels · Hi-Res
(placeholders — recapture from the hardware with capture_preset.sh)

## History
- v1.x (June 2026): per-pixel + pixelate paths, 2D vertical accumulator,
  Mix/Bypass. Never confirmed flicker-free; HEAD's 2D accumulator never
  flashed.
- v2.0.0 (2026-10-07): rework on the CGA v2.0.6 architecture (tiers 0–2 of
  the finalization review): valid categories, blanking gate, legal levels,
  U/V-swapped Colodore palette, Mix/Bypass/Pixelate removed, Stability,
  native size, Multicolor, Ramp, Loading, Ready, P12 Dither, labelled-
  default index fix on K1/K3.

## Build
v2.0.0 (2026-10-07), router2, all 6 configs seed 1 (post-route):
HD Analog 84.8 · HD HDMI 82.0 · HD Dual 83.0 · SD Analog 82.6 · SD HDMI 79.1 ·
SD Dual 81.0 MHz. ~5770/7680 LC (75%), 17/32 EBR (1 column buffer + 2×8 grid).
HW-validated; defaults rebuild 2026-10-07 identical timing.
