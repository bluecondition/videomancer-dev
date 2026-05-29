# CGA — IBM CGA palette emulator

## What it does
Processing effect that quantises incoming video to one of the classic 4-colour IBM CGA palettes (Green/Red, Cyan/Magenta, or Cyan/Red), with independent horizontal and vertical pixel blocking, low/high intensity, and optional scanlines. The Color Mode switch chooses between nearest-colour matching and luma-mapped selection.

## How it works
Downsamples each cell on the first scanline of its cell row, palette-matches the average via 5-bit Manhattan distance against the active 4-colour palette, and stores the 2-bit index in a per-cell-column BRAM (single iCE40 BRAM holds up to 2048 cells). Upscaling reads the same index for every output pixel inside the cell — solid nearest-neighbour blocks. 6 palettes (3 modes × 2 intensities) are constants. 9-stage pipeline; no anti-flicker code, so chroma near a 32-quantum palette boundary may shimmer.

## Controls
- K1 Palette — Green/Red, Cyan/Magenta, Cyan/Red
- K2 H Size — horizontal pixelation Off / 2× / 4× / 8× / 16×
- K3 V Size — vertical pixelation Off / 2× / 4× / 8× / 16×
- K4 Contrast — luma expand before matching
- K5 Saturation — chroma scale (512 = unity)
- K6 Reserve — unused
- T7 Pixelate — enable blocking
- T8 Intensity — Low or High palette variant
- T9 Scanlines — scanline dimming
- T10 Color Mode — Nearest (chroma match) vs Luma (luma-mapped index)
- T11 Bypass — passthrough
- Slider Mix — wet/dry between original and CGA-processed

## Presets
(none defined in toml)
