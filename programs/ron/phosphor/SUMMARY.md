# Phosphor — Rainbow edge-trails

## What it does
An edge-detection processor that paints a ROYGBIV trail to the right of every detected horizontal edge in the input. At each edge the output snaps to red, then steps through orange/yellow/green/blue/indigo/violet/black over the following pixels with a configurable band width. The pipeline resets at hsync so trails never cross scanlines, producing a CRT-phosphor-meets-spectrum look that follows the contours of the source video.

## How it works
Processing path. Input video is low-pass filtered (2-tap pre-filter that convolves with a 4-tap smoothing to give a 5-tap triangular kernel), optionally posterized and contrast-boosted, then a horizontal `|Y[x] - Y[x-1]|` gradient is compared against a threshold to detect edges. A per-pixel 3-bit band index selects from an 8-entry rainbow LUT (7 colours + black) in 10-bit YUV with U/V swapped for Videomancer hardware convention. A minimum-run filter rejects edge noise; the curve mode reduces the per-band pixel width across the trail for an exponential decay shape. Optional video-key blends the original luma back as the trail's brightness modulator.

## Controls
- **K1 Band W** — pixel width of each colour band in the trail (1–128 px).
- **K2 Edge Thr** — gradient magnitude required to trigger a new trail.
- **K3 Noise** — minimum run length, suppresses spurious short edges.
- **K4 Poster** — number of low Y bits to drop before edge detect (0–7).
- **K5 Contrast** — pre-edge contrast boost (1.0× to ~2.0×).
- **K6 Curve** — per-band step reduction for tapered trail decay.
- **T7 Video Key** — modulate trail brightness with the source luma.
- **T8 Curve En** — enable the K6 curve shaping.
- *(toggles 9–11 and slider 12 are unused)*

## Presets
- *(none defined)*
