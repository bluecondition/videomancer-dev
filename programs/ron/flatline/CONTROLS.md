# FLATLINE — control card

Five builds share one panel layout; each carries one smoother and one line algorithm
(see SUMMARY.md for the matrix): **A** flatline (bilateral), **B** flatline_ink (median + XDoG),
**C** flatline_canny (median + Canny-lite), **D** flatline_kuwa (Kuwahara), **E** flatline_finish
(median + soft quantization + drop shadow).

## Knobs

| Knob | Name | Positions / travel | What it does |
|---|---|---|---|
| K1 | Smoother | two labels: Off / <this build's smoother> | Stage 1. Off = pair-averaged input, no smoothing. Median = true 3x3 median (sorting network). Bilateral = separable 5x5 bilateral (range weights from K2). Kuwahara = 5x5, mean of the lowest-range 3x3 quadrant. Chroma is always smoothed by a 2-line x 8-px box except in Off (then a 2-line average only). |
| K2 | Strength | 0..100% | Bilateral: range sigma in octaves 4 / 8 / 16 / 32 / 64 / 128 codes (low = texture preserved, high = plain blur). Median / Kuwahara: blend from the centre pixel toward the filter result in quarters, 25%..100%. No effect in Off. |
| K3 | Levels | 8 labels: 2, 3, 4, 5, 6, 7, 8, 8 Coarse | Luma bin count. `8 Coarse` keeps 8 luma bins and coarsens chroma: 4 hue sectors (90 degrees) instead of 8 and 2 saturation levels (grey / colour) instead of 3. |
| K4 | Lines | 4 labels: None / Edge / Edge White / <XDoG, Canny or Edge Bold> | Stage 4 algorithm and ink colour. Edge = region boundary (cleaned label differs from left / above / right): 1-px, closed, registered to the mattes. Edge White = the same in white ink (Y 775). XDoG (build B) = difference of 3x3 and 5x5 binomials, ink on the dark side of edges where it passes the K5 threshold (soft over 2 codes); no dark fills. Canny (build C) = Sobel (+ chroma gradient when S10) -> NMS -> hysteresis, thin connected lines. Edge Bold (other builds) = the 3x3 footprint at full ink regardless of K5. |
| K5 | Ink | 0..100% | Ink weight. Edge lines: footprint thin (0..33%) / 2x2 (33..66%) / 3x3 (66..100%) in half-res samples (2/4/6 px at 1080p); ink at half strength in the bottom quarter, full above. XDoG: edge threshold 0.5 .. 4.4 codes (up = fewer, stronger strokes; 50% is a sketch, 0% dense hatching). Canny: gradient pre-gain 2..17 (up = more lines; a strong edge is a 128-code step at 0%, ~25 at 50%, 15 at 100%). |
| K6 | Shadow | 0..100% | 0 = off. 0..50%: shadow offset grows down-right, 1..15 lines in 1080-line units (1080p: 1 px per unit; 720p 2/3; SD 1/2 px, 480i 1 field line per 4.4 units). 50..100%: offset stays at max and the shadow softens (horizontal box 2 / 4 / 8 / 16 samples). Shadow = the non-background silhouette, grey on the black background. |

## Switches

| Switch | Name | Off / On | What it does |
|---|---|---|---|
| S7 | Quant | Hard / Soft | Hard: nearest bin. Soft (builds with it): the matte luma blends toward the runner-up bin by its share of the 5-row vote (0.2..0.5), and half-steps across horizontal label changes; ramps about 4 lines / 4 px. Chroma stays hard. |
| S8 | Levels | Fixed / Adaptive | Fixed: evenly spaced bins 16..235. Adaptive: equal-population thresholds from last frame's 64-bin luma histogram (each bin gets the same number of pixels), low-passed 1/4 per frame and kept monotonic; the representative luma of a bin is its median sample. Switching to Adaptive starts from the fixed table. |
| S9 | Cleanup | Off / On | Stage 3 vertical pass of the separable 5x5 mode filter (per component: luma bin, hue, saturation). The horizontal 5-sample mode is always on: it is the minimum-run-length filter (a label needs 3 of the 5 samples around it). |
| S10 | Temporal | Off / On | Stage 5: a pixel within 3 codes of a luma bin boundary takes last frame's label of its 128x64 cell when that label is one of the two candidates. Self-gating: moving content that crosses a boundary by more than the band updates normally. Builds without the store: unused (C: Canny chroma gradient). |
| S10 | Chroma Grad (C) | Off / On | Canny gradient = luma Sobel only / max of luma and chroma difference. |
| S11 | View | Final / Lines | Lines = the ink alone on mid grey (the brief's diagnostic fallback; the 2x2 split did not fit). |

## Slider

| P12 | Wipe | 0..100% | Vertical wipe: processed picture on the left, raw input on the right. 100% = fully processed, 0% = all raw. The processed side sits 4 input lines lower than the raw (two 5-row windows), visible only at the seam. Latency 84 clocks. |

## Notes for the bench

- Every control acts at any position of the others, except: K2 does nothing in Smoother Off (Off is a reference view), and S10/K6 do nothing in build C (no store). The `8 Hue4*` levels change chroma only.
- Default (power-up): the build's smoother on, Strength 40%, 4 levels, Edge lines, Ink 60%, Shadow off (where built), Hard, Adaptive, Cleanup on, Temporal on, Final, Wipe 100%.
- Presets (hand-written, capture real ones with `./capture_preset.sh`): Cel Flat (4 levels, edge), Cut Paper (strength 70%, 5 levels, no lines, shadow + soft where built), Ink Trace (3 levels, XDoG / Canny / Edge, fixed thresholds), Lines Only.
- Luma in the mattes is capped by the output conditioning (white -> ~775); the raw half is untouched.
