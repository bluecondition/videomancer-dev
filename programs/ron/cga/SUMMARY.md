# CGA — IBM Color Graphics Adapter re-render (v2.0)

## What it does
Redraws incoming video in the CGA's own display modes on a block grid:
mode-4 four-colour palettes (with the real selectable background colour),
mode-6 640-dot mono, and composite artifact colour (mono dots read four at a
time as the NTSC colour nibble → the 16 composite colours). The "CGA" block
size gives a native 320×200 (640×200 for mono/composite) grid measured from
the incoming raster. P12 adds per-block ordered dither, the classic CGA
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
- T9 Scanlines — dim the last line of every block row (V≤2: alternate lines, 25%)
- T10 Color Mode — Nearest / Luma; in Composite: colour-burst phase flip
- T11 Snow — CGA snow: random character-wide dashes each field
- P12 Dither — ordered 4×4 Bayer per block, 0 → flat, 100% → ±240 codes

## Presets
Mode 4 Hi · Blue Screen · Composite · Green Mono · Snowstorm
