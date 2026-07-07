# Phosphor — Rainbow edge-trails

## What it does
An edge-detection processor that paints a ROYGBIV trail to the right of every detected edge in the input. At each edge the output snaps to red, then steps through orange/yellow/green/blue/indigo/violet/black over the following pixels with a configurable band width. The pipeline resets at hsync so trails never cross scanlines, producing a CRT-phosphor-meets-spectrum look that follows the contours of the source video.

## How it works
Processing path. Input luma is contrast-boosted around mid-grey, optionally posterized (128..2 levels, Edge Lab mapping), then fed through a 3x3 Sobel detector pulled from Edge Lab: four chained BRAM line buffers give a 5-row window whose three center rows — raw, 3x3 Gaussian-blurred when Smooth (T9, default on) is enabled, or separable-3x3-median denoised when Median (T10) is on (wins over Smooth; kills speckle without softening edges) — feed per-column `[1 2 1]` sums and column histories forming gx/gy; the magnitude `|gx| + |gy|` (normalized to 0..255) is compared against a threshold. The thresholded edge bit is then vertically solidified: OR'd with the same column of the previous two lines (two 1-bit line buffers), so a scanline whose magnitude dips just under threshold still triggers where its neighbours detected the contour — no gaps in the rainbow sheet. The qualified trigger is held for the entire solid run, pinning the band machine at red: the outline renders as a solid red region exactly as thick as the detected edge, with the rainbow starting at its trailing side. Luma-only and 2-D — much cleaner than the old 1-D horizontal Y+U+V difference. A per-pixel 3-bit band index selects from an 8-entry rainbow LUT (7 colours + black) in 10-bit YUV with U/V swapped for Videomancer hardware convention. A minimum-run filter rejects edge noise; the curve mode grows the band widths geometrically across the trail (thin red first, each colour wider toward violet) via a per-band size table indexed by band index. Optional video-key blends the original luma back as the trail's brightness modulator.

## Controls
- **P12 Band W** — pixel width of each colour band in the trail (1–128 px); on the slider as the primary performance control.
- **K1 Color** — rainbow cycle phase (7 steps/turn): rotates the colours through the bands, so sweeping the knob animates the trail colours. Geometry (band widths, curve, key) is unaffected.
- **K2 Edge Thr** — Sobel magnitude required to trigger a new trail.
- **K3 Noise** — minimum run length, suppresses spurious short edges.
- **K4 Poster** — luma posterize before edge detect (off, 128..2 levels).
- **K5 Contrast** — pre-edge contrast boost (1.0× to ~2.0×).
- **K6 Curve** — curve steepness 0–3: with T8 on, band widths grow geometrically from thin red to full-width violet (0 = even, 1 = doubling per colour, 3 = steepest).
- **T7 Video Key** — modulate trail brightness with the source luma.
- **T8 Curve En** — enable the K6 curve shaping.
- **T9 Smooth** — 3x3 Gaussian pre-blur feeding the Sobel (default on; stabilizes per-line magnitudes for solid outlines).
- **T11 Reverse** — plays the rainbow violet-first (composes with the K1 colour phase).
- **T10 Median** — separable 3x3 median denoise feeding the Sobel (overrides T9; removes speckle while keeping edges sharp, so magnitudes stay strong).

## Presets
- *(none defined)*
