# Prism — Rainbow edge-trails

## What it does
An edge-detection processor that treats every edge in the image as a dispersion point: at each detected edge the output snaps to red, then steps through orange/yellow/green/blue/indigo/violet/black over the following pixels with a configurable band width — white light split into an ordered spectrum streaming off the contours of the source video. The pipeline resets at hsync so trails never cross scanlines. (Formerly named "Phosphor", after the CRT-phosphor edge simulator it grew out of.)

## How it works
Processing path. Input luma is contrast-boosted around mid-grey (up to a ~9× "blowout" that clips the image toward a binary black/white mask for the cleanest possible edges), optionally posterized (128..2 levels, Edge Lab mapping), then fed through a 3x3 Sobel detector pulled from Edge Lab: four chained BRAM line buffers give a 5-row window whose three center rows — raw, or 3x3 Gaussian-blurred when Smooth (T9, default on) is enabled — feed per-column `[1 2 1]` sums and column histories forming gx/gy; the magnitude `|gx| + |gy|` (normalized to 0..255) is compared against a threshold. The thresholded edge bit is then vertically solidified: OR'd with the same column of the previous two lines (two 1-bit line buffers), so a scanline whose magnitude dips just under threshold still triggers where its neighbours detected the contour — no gaps in the rainbow sheet. The qualified trigger is held for the entire solid run, pinning the band machine at red: the outline renders as a solid red region exactly as thick as the detected edge, with the rainbow starting at its trailing side. Luma-only and 2-D — much cleaner than the old 1-D horizontal Y+U+V difference. A per-pixel 3-bit band index selects from an 8-entry rainbow LUT (7 colours + black) in 10-bit YUV with U/V swapped for Videomancer hardware convention. A minimum-run filter rejects edge noise; the curve mode grows the band widths linearly across the trail (thin red first, each colour wider toward violet) via a per-band size table indexed by band index. The display palette itself is built by a background engine: Rainbow scheme (K1 = phase rotation), or a single-colour scheme (K6) whose trail K1 lerps toward black or white. Optional video-key passes the source through under (or, with T8 Over, on top of) the trails.

## Controls
- **P12 Band W** — pixel width of each colour band in the trail (1–128 px); on the slider as the primary performance control.
- **K1 Col/Fade** — in Rainbow scheme: cycle phase (7 steps/turn) that rotates the colours through the bands, animating the trail. In mono schemes: fade depth/direction — centre (50) is a flat solid colour, turning down fades the trail smoothly to black, turning up bleaches it toward white. Geometry (band widths, curve, key) is unaffected.
- **K2 Edge Thr** — Sobel magnitude required to trigger a new trail.
- **K3 Noise** — minimum run length, suppresses spurious short edges.
- **K4 Poster** — luma posterize before edge detect (off, 128..2 levels).
- **K5 Contrast** — pre-edge contrast/blowout (1.0× to ~9.0×) whose pivot slides from mid-grey down to BT.601 black level as the knob rises: the low range is a natural centered contrast boost, while the top blows everything above ~12% luma to white and crushes the black-level noise floor to black — a video-level-aware binarizer ("3× value multiplier" style, without amplifying dark noise).
- **K6 Scheme** — colour scheme: Rainbow (default), or Red/Orange/Yellow/Green/Blue/Indigo/Violet — mono schemes render every band in that colour, with K1 lerping the trail toward black or white (T11 Reverse flips the fade direction). Built by a 5-stage background palette engine (fetch / depth-multiply / weight / lerp-multiply / store, one band per clock).
- **T7 Video Key** — pass the source video through wherever the trail has gone past violet (the black region); T8 selects whether it also rides above the bands.
- **T8 Key Pos** — with T7 on: Under keeps trails over the video; Over lets the source ride on top of the colour bands wherever its preprocessed luma is ≥ 50% (stateless luma-key matte; K5 blowout hardens the cutout).
- **T9 Smooth** — 3x3 Gaussian pre-blur feeding the Sobel (default on; stabilizes per-line magnitudes for solid outlines).
- **T11 Reverse** — plays the rainbow violet-first (composes with the K1 colour phase).
- **T10 Curve En** — enable the linear band-width curve.

## Presets
- *(none defined)*
