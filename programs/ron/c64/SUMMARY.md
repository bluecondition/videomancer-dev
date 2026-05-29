# C64 — Commodore 64 display processor

## What it does
Processing effect that makes incoming video look like it's running on a Commodore 64: quantises to the 16-colour C64 palette, optionally pixelates into chunky blocks, surrounds the image with a coloured border, and adds CRT-style scanline dimming. A wet/dry Mix slider blends back toward the original.

## How it works
Per pixel, Manhattan-distance match against the 16-entry Colodore-derived palette in YUV space, computed with the top 5 bits of each channel (32 levels) for noise robustness. All 16 distances are produced in parallel, then reduced 16→4→1 to a winner index. Pixelation samples once per cell and replays via a per-cell-column buffer. Zero BRAM aside from the column buffer; no DSPs. 10-stage pipeline, with the final 4 stages running `interpolator_u` for the wet/dry mix.

## Controls
- K1 Pixel Size — Off / 2× / 4× / 8× / 16× cells
- K2 Border Wid — border thickness (0..255 px)
- K3 Border Col — border colour (16 palette entries)
- K4 Scanline — scanline dimming strength
- K5 Saturation — chroma scale (512 = unity)
- K6 Contrast — luma expand (pre-match)
- T7 Pixelate — enable cell sample/hold
- T8 Border — draw coloured border
- T9 Scanlines — enable scanline darkening
- T11 Bypass — passthrough
- Slider Mix — wet/dry between original and C64-processed

## Presets
- Classic C64
- Palette Only
- Blocky
- Blue Border
