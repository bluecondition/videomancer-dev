# Parallax — anaglyph edge slabs over sharp monochrome

## What it does
The source is shown as clean luma-only monochrome at full bandwidth — nothing filters it, so it is exactly as sharp as the input — and on each scanline the **leftmost** and the **rightmost** detected edge each throw a flat, hard-edged glow outward: the leftmost to the LEFT in the pair's first colour, the rightmost to the RIGHT in the second. Interior detail stays silent. The subject ends up crisp black-and-white wearing a blue glow down one outline and a red one down the other — a stereo separation of the whole silhouette that never resolves. Glow width is the performance gesture and lives on P12.

## How it works
**Two paths, completely separate.** The display path is `data_in.y` → one point-operation contrast stage (K5 BASE, unity at 50% so it is bit-exact) → compose. Chroma is discarded and replaced with neutral or slab colour. No blur, lowpass or line buffer ever touches it.

**The analysis path is destroyed on purpose.** It runs at half horizontal resolution (pixel pairs averaged): a centred box blur of 1–16 analysis samples (2–32 px), posterisation to 256..4 levels, then a centred gradient `gx = p(m+B/2) − p(m−B/2)` whose baseline B tracks the blur. Every box and gradient window is symmetric about a *fixed* tap, so turning CLEAN changes the character without sliding the edge map sideways. `gx` is halved into an 8-bit signed word and run through a per-column IIR across scanlines, then compared against two thresholds with hysteresis, a debounce, and a lowered threshold wherever the previous line found an edge at the same column — that vertical term is what stops slab boundaries crawling. Both marks are taken at the leading corner of their gradient run and delayed by a CLEAN-tracked constant, which lands them on the middle of the run, i.e. on the edge itself: blue ends one column short of the edge and red starts on it, with no dark halo between slab and object.

**Only the two outline edges participate**, and that is what makes the geometry cheap. Throwing leftward from an *arbitrary* edge needs lookahead, which a raster does not have — the per-edge version needed a whole reverse-scan pass over a mark buffer plus a paint-map buffer to make it causal. With only the extremes mattering, both are found in the single forward analysis pass (first mark wins the left, every mark overwrites the right), so a line's entire geometry is two column numbers. They are latched at the line turn-around, resolved there into four painted bounds, and the pixel path is nothing but compares against registered values. The display path is never delayed: nothing shifts, nothing blurs, and the program's latency is a flat 7 clocks in every mode. The glow map lags the picture by one scanline.

A mark at analysis column `c` draws on source samples `c−16`…`c+16`, and at the start of a line those registers still hold the previous line's tail — the step between lines reads as an edge at column ~0. Harmless when every edge threw its own band; fatal here, because that one spurious mark wins "leftmost" and pins the left glow to the frame edge. Marks are therefore rejected inside an 18-sample margin.

**Buffers** — all canonical 1W1R, ping-ponged by line parity so no bank is ever read and written in the same line (an iCE40 EBR returns undefined data on a same-address read-during-write, which no simulation can catch): two analysis line buffers for the Sobel column sum, `vg` 1024×8 smoothed gradient, `hb` 1024×4 previous-line edge state. 10 EBR of 32.

Either polarity counts toward the silhouette: the leftmost edge of a line is the subject's left boundary whether the subject is lighter or darker than its ground. In the ordinary case (bright subject, dark ground) that coincides exactly with "left-facing edge throws left". **Collisions** are additive — where the two glows overlap (a thin subject at a wide setting) the colours are summed as light rather than one painting over the other, which is what a real anaglyph overlap does (blue over red goes magenta) and generalises to every pair. The sum is computed once per field, so the overlap is one flat colour.

The per-field sequencer walks one shared multiplier through a fixed 12-slot schedule to derive `w_max = width × RANGE`, `w_cur = w_max × P12`, and the two balanced widths — so every dimension is a fraction of the runtime-measured width, not a pixel constant, and none of them can change part-way down a picture.

## Controls
The panel has six rotaries, five toggles and the slider, not eight rotaries, so SETTLE and DENSITY — the two controls whose ends the brief names explicitly and which therefore degrade cleanly to two states — took switches, and the six that genuinely need a pot kept one.

- **P12 Width** — master slab width, 0 to the full RANGE. At the floor the output is clean monochrome and nothing else; pushing it up blooms colour out of every edge at once. Glided at one eighth of the error per field (~0.13 s) so a fast shove cannot tear, and the width only ever changes between fields.
- **K1 Edge** — detector threshold. Down, only the strongest object boundaries throw; up, fine texture joins in.
- **K2 Clean** — analysis blur, posterisation and gradient baseline together, in eight steps. Down is literal edge-following; up is chunky abstract silhouettes.
- **K3 Range** — maximum glow width, 0.4%–2.0% of the measured screen width, about 8–38 px at HD and labelled in px (BALANCE at an extreme takes one side to twice that). FILL runs on a 16× boosted cap, or flooding could never cross a region.
- **K4 Balance** — bipolar width split with a detented centre; full travel takes one side to 2× and the other to nothing.
- **K5 Base** — monochrome contrast, 0.25× to ~6×. 50% is unity and bit-exact. Display path only.
- **K6 Pair** — Blue/Red, Cyan/Orange, Green/Magenta, Yellow/Violet, Teal/Amber, Azure/Rose, Lime/Purple, White/Black.
- **S7 Settle** — Lively / Calm: the vertical gradient IIR (1/2 vs 1/8 per line), the vertical hysteresis depth and the debounce length.
- **S8 Density** — Wash (a quarter of the slab luma, three eighths of its chroma, so the picture reads through as a tint) / Solid (flat colour, no video through at all).
- **S9 Fill** — Off, the glows are RANGE×P12 wide; On, they flood outward from the silhouette with a 32× reach (there is no interior edge left for a flood to stop at), clearing the frame from any ordinary subject at full P12.
- **S10 Swap** — exchanges the two sides' colours.
- **S11 Layer** — Over / Behind; Behind lets the picture's own bright areas knock holes in the colour.

## Presets
- **Anaglyph** — the default look, moderate width.
- **Big Slabs** — low EDGE, high CLEAN, Calm: three or four large flat slabs on a face.
- **Shimmer** — high EDGE, low CLEAN: a dense field on a busy source.
- **Flood** — FILL on at full range: colour floods whole regions.

## Detector rework (after the first hardware-eye look)
The original detector read badly and the slabs looked blocky. Three faults, found by measuring rather than guessing, with livewire as the reference:

1. **No symmetric vertical support.** Livewire builds `ss = qr2 + 2·qr1 + y8d` and takes the horizontal difference across *that*. This had only a single-row horizontal difference, with all vertical support coming from the lagging IIR — which reached just **41% of a gradient on a contour four lines tall**. Curved contours (a face) were attenuated hardest, exactly the case that should give the boldest result. Fixed with two analysis line buffers and a real `[1 2 1]` column sum.
2. **The IIR cost sensitivity to buy stability.** It now **rises instantly and only decays slowly**: a new edge fires at full strength on its first line while persistence still smooths the boundary downward. SETTLE sets the decay rate.
3. **The EDGE threshold was mis-scaled.** A linear sweep demanded a **104-luma step at the knob's midpoint** — bigger than almost any real edge — so two thirds of the travel did nothing. Now a squared curve: 132 luma at the floor, 36 at the centre, 6 at the top. Measured on a contrast wedge, EDGE 25/50/75/95% fires at luma steps of 64/45/20/12.

Livewire's DESIGN.md also warns that per-scale gain is mandatory (blurring shrinks the gradient). Checked: the gradient baseline here already tracks the blur width, so an ideal step keeps full amplitude across all eight CLEAN zones.

Then the whole premise narrowed: only the **leftmost and rightmost** edge of each scanline throws anything, so the subject carries one glow off each outline instead of every interior edge emitting a band. That made the program cheaper, not dearer — the reverse-scan pass, the mark buffer, the paint-map buffer and both countdown engines all went, taking 4 EBR and a scanline of lag with them.

Before that, a separate look had established that the slabs were simply **too wide**. At hundreds of pixels they swallowed one another and whole regions went flat magenta, reading as blocks rather than edges; a Python model of the detector confirmed marks fire on 99–100% of the lines a contour crosses, so nothing was being dropped. RANGE was cut to a fringe (8–38 px at HD).

## Build
All six close on **seed 1**: HD Analog 76.63, HD HDMI 78.93, HD Dual 77.21, SD Analog 78.23, SD HDMI 73.03, SD Dual 74.25 MHz against a 74.25 MHz HD target. 4997/7680 LC (65%), 10/32 EBR, 107/256 IO. All six bitstreams `iceunpack` clean.

The first build missed HD on every seed (67.8–70.9 MHz) at only 56% LC — depth, not congestion. The critical path was the `vg` EBR output feeding the vertical-IIR subtract→add→clamp chain directly: 2.15 ns of clock-to-q plus 1.49 ns getting out of the RAM column were spent before any logic ran. Three changes fixed it, all on seed 1 afterwards:
1. the bank-muxed gradient-history read lands in a fabric FF before any arithmetic, and the IIR is split into a subtract stage and an accumulate/clamp stage;
2. buffer columns became free-running per-tick counters instead of a 13-bit subtract feeding a 13-bit range compare in the same stage as the hysteresis state machine;
3. the parameter block's 3–4 deep carry chains were broken by registers — a vblank cone is still fully STA-timed at the pixel clock.

Adding the Sobel column sum later put HD Dual back under (66.9 MHz), this time with the path starting at the IIR's magnitude-compare register and running the whole compare→3-mux→add→clamp cone. Resolving the rise/decay decision and the decayed step one stage early collapsed it to a single add plus a mux, and the gradient's clamp turned out to be provably unreachable (`±1020 >> 3` always fits signed 8) so it came out entirely.

## Verification
- Slab geometry was pixel-identical before and after the first timing rework (0 mismatched pixels over the slab mask).
- EDGE sensitivity measured against a contrast wedge, not eyeballed (see the table above).
- Two consecutive frames of a static source at 720p, decimation 1, EDGE 96% / CLEAN 6% (the settings most likely to chatter) are **bit-identical** — 0 of 921600 pixels differ, on a frame that is dense with slabs.
- Every control was exercised individually in the image tester; all are live.

## Notes
- **Name clash:** `programs/ron/catalogue-combo/parallax` is a different, hardware-passed program also called "Parallax" (depth-from-luma displacement + channel swap). This one took `program_id = "com.ron.parallaxslab"` so the two never collide, but both still show as "Parallax" on the panel — worth renaming one.
- The palette is stored U/V-swapped for the hardware, so simulation previews show the hues exchanged (blue slabs read red). Judge geometry in the sim and hue on hardware.
- `program_type = "processing"` — the program reads `data_in` in every mode.
- The fringe **tapers to nothing at the top and bottom of a rounded object**. A horizontal contour has no left- or right-facing gradient, so there is nothing to throw sideways; this is what a real anaglyph does. Throwing from vertical gradients too would wrap the fringe all the way round, but it would stop being a parallax separation and become an outline.
- The slab map lags the picture by two scanlines. On a contour within a few degrees of horizontal that maps to a large horizontal displacement — but the gradient there is weak and the fringe has usually tapered out already, so it is self-limiting.
- With P12 at the floor there are no slabs at all, so the other slab controls do nothing there. That is what the brief asked for ("clean monochrome and nothing else"), and it is the one place this program departs from the always-on-controls rule.
- Status: **timing-closed, not yet hardware-tested.** Stillness re-verified on the shipped logic: 0 of 921600 pixels differ between consecutive frames at EDGE 86% / CLEAN 15%.
