# C64 — Commodore 64 re-render (v2.0)

**Status: v2.0.0 FINAL except presets — signed off by Ron on hardware 2026-10-07.**
Defaults captured from the device (8x, Border 296, Scanlines 442, Color 552,
Contrast 604, Stability Off, Multicolor On, Nearest, Loading Off, Ready On,
Dither 84). Presets are hand-written placeholders.

## What it does
Redraws incoming video as a Commodore 64 screen: chunky blocks in the 16
Colodore colours on a native 320×200 grid (of the 1080p raster; every size
scales with the screen since v2.0.2),
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
- K4 Scanlines — dims the last line of every block row (0 = off, 100% ≈ black;
  half depth for rows under 2 lines, alternate lines for rows under 1.5)
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

## Resolution scaling (v2.0.2, 2026-10-17)
Block sizes are virtual 1080-line units: the cell counters step per pixel /
per field line from the video timing ID (1.0 at 1080p, 1.5 at 720p, SD 2.0
across × 1080/486 per frame line), so 8x is 8 px at 1080p, 5.3 px at 720p
and 4 px × 3.6 lines at SD (4:3); the native C64 detent is 6×5 units
everywhere and the READY. glyphs (one block per glyph pixel) scale with the
blocks. Non-integer sizes alternate pixel widths / line counts. The row
wrap runs in the blanking (a field line can cross several rows at SD),
stability tiles are 32×16 virtual units, averaging uses the largest power
of 2 that fits the narrowest cell, Loading stripes change on ~1 virtual
line in 16, and the Bayer grain / scanline style follow the real block
size. The border was already a fraction of the measured raster (unchanged).
1080p output is bit-identical to v2.0.1; palette unchanged.

## Build
v2.0.2 (2026-10-17), router2, seed 1 except HD Analog seed 2 (seed 1
missed, core sync-generator path) and HD Dual seed 2 (seed 1 router2
stall), post-route: HD Analog 77.5 · HD HDMI 87.8 · HD Dual 80.2 ·
SD Analog 81.6 · SD HDMI 84.0 · SD Dual 85.4 MHz. 5876/7680 LC (77%),
17/32 EBR.

v2.0.0 (2026-10-07), router2, all 6 configs seed 1 (post-route):
HD Analog 84.8 · HD HDMI 82.0 · HD Dual 83.0 · SD Analog 82.6 · SD HDMI 79.1 ·
SD Dual 81.0 MHz. ~5770/7680 LC (75%), 17/32 EBR (1 column buffer + 2×8 grid).
HW-validated; defaults rebuild 2026-10-07 identical timing.
