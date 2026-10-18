# CGA — IBM Color Graphics Adapter re-render (v2.0)

## What it does
Redraws incoming video in the CGA's own display modes on a block grid:
mode-4 four-colour palettes (with the real selectable background colour),
mode-6 640-dot mono, and composite artifact colour (mono dots read four at a
time as the NTSC colour nibble → the 16 composite colours). The "CGA" block
size gives a native 320×200 (640×200 for mono/composite) grid of the 1080p
raster (6×5 / 3×5 px); like every size it scales with the screen (v2.0.9). P12 adds per-block ordered dither, the classic CGA
game-art checkerboard.

## How it works
- Each cell averages its first 2^k pixels on the first line of its cell row;
  the result goes through a mid-pivot contrast, a chroma gain, a
  value-snap (dither on) and a Bayer offset, then a 5-bit Manhattan
  nearest-colour match (or luma-zone pick). The index — or the 4-dot nibble
  in Composite — is stored per column in BRAM and replicated down the row.
- Stability: ping-pong 16×16-px tile grids. Dither off → decision
  hysteresis (margin + two-field debounce, HW-validated v1.3). Dither on →
  value hysteresis (tile stores luma in 16-code steps modulo 256; cells snap
  within ±24 codes before dithering), so dither patterns survive shared tiles. Any
  decision-side control change re-seeds the grids for two fields.
- Palettes: the 16 RGBI colours, BT.601 re-levelled to black 64 / white 768,
  U/V swapped for hardware. Composite table = old-style CGA artifact colours.
- Output chroma is clamped to the legal 64–960 range and gated to neutral
  outside active video; the field's first line
  (which shows last field's bottom row) is blanked. Latency 7 in all modes.

## Controls
- K1 Mode — Grn/Red, Cyn/Mag, Cyn/Red (mode 4), Mono 640, Composite
- K2 H Size / K3 V Size — 2x / 4x / CGA (native) / 8x / 16x
- K4 Contrast — around mid-grey, 50% = unchanged
- K5 Color — Nearest: source chroma gain before matching (0 → luma only,
  100% → 5×); Luma / Composite: palette saturation, grey → true palette
  (never beyond: the CGA colours already sit on the gamut edge)
- K6 Ink — RGBI colour (with T8 as the intensity bit): background in the
  palette modes and Composite, foreground in Mono (Black → white)
- T7 Stability — anti-flicker on/off
- T8 Intensity — low/high palette (and the Ink's intensity bit)
- T9 Scanlines — dim the last line of every block row, 50% (25% for rows
  under 2.5 lines; alternate lines at 25% for rows under 1.5 lines)
- T10 Color Mode — Nearest / Luma; in Composite: colour-burst phase flip
- T11 Snow — CGA snow: random character-wide dashes each field
- P12 Dither — ordered 4×4 Bayer per block, 0 → flat, 100% → ±240 codes

## Presets
Mode 4 Hi · Blue Screen · Composite · Green Mono · Snowstorm

## Resolution scaling (v2.0.9, 2026-10-17)
Block sizes are virtual 1080-line units: the cell counters step per pixel /
per field line from the video timing ID (1.0 at 1080p, 1.5 at 720p, SD 2.0
across × 1080/486 per frame line), so 8x is 8 px at 1080p, 5.3 px at 720p
and 4 px × 3.6 lines at SD (4:3), and the native CGA detent is 6×5 units
everywhere. Non-integer sizes alternate pixel widths / line counts. The
row wrap runs in the blanking (a field line can cross several rows at SD),
stability tiles are 16×16 virtual units, averaging uses the largest power
of 2 that fits the narrowest cell, and the Bayer grain / scanline style
follow the real block size. 1080p output is bit-identical to v2.0.8
(diff 0 px); palettes unchanged.

## Build
v2.0.9 (2026-10-17), router2, seed 1 (HD Analog seed 2 after a seed-1
router2 stall), post-route: HD Analog 80.8 · HD HDMI 77.9 · HD Dual 83.2 ·
SD Analog 80.6 · SD HDMI 83.8 · SD Dual 78.6 MHz. 5412/7680 LC (70%),
18/32 EBR.
