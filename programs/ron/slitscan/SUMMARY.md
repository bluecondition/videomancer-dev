# Slit Scan — Temporal slit-scan video processor

## What it does
A time-displacement processor that divides the frame into N discrete slices (horizontal bands or vertical columns) and gives each slice a different time-delay from a circular buffer. Moving objects in the input leave staircase-stepped motion smears across the slices, since different parts of the image are showing different moments. Slice counts from 2 to 128 are selectable; patterns can be ramped, mirrored (fold), reversed, or scattered randomly.

## How it works
Processing program. Three `variable_delay_u` BRAM buffers (one per YUV channel, 10-bit × 2048 deep, ~60 Kbit total) hold ~2 frames of history. A DDA-based slice counter quantises the current pixel position to a slice index without division, then an LUT/bit-reverse maps the index to a delay: ramp (linear across slices), sine (half-sine bump, peaks at the centre), folded (V-shape from centre), reversed (now/past flipped), or random (bit-reversed scatter for non-monotonic shuffle). Input luma can additively modulate the per-pixel delay, and UV diverge offsets U vs V for chroma-split effects. A final `interpolator_u` crossfades dry/wet. 10-clock pipeline.

## Controls
- **K1 Delay** — base time depth into the history (0–2046 samples).
- **K2 Chroma Lag** — extra delay on U/V beyond Y.
- **K3 Slices** — number of discrete slices (2, 4, 8, 16, 32, 64, 128).
- **K4 Offset** — common base delay added to all channels.
- **K5 Luma Mod** — input-luma scales the per-pixel delay (image self-warps).
- **K6 UV Diverge** — 512 = equal, <512 U leads V, >512 V leads U.
- **T7 Axis** — Horiz (horizontal bands) vs Vert (vertical columns).
- **T8 Reverse** — flip now-end and past-end of the delay ramp.
- **T9 Fold** — V-shape pattern: centre slice = present, edges = past.
- **T10 Pattern** — Random (bit-reversed scatter) vs Sine (half-sine bump).
- **T11 Bypass** — pass input video straight through.
- **S12 Mix** — dry/wet crossfade.

## Presets
- Wide Bands
- Fine Slices
- Vert Columns
- Fold Mirror
- Chroma Split
