# Turpentine — chroma as pigment (v3)

Two rejected predecessors, same fiction, three engines:

- **v1 — chroma-directed feedback smear.** Rejected: "knobs don't feel like
  they do anything." A few px/line of recursive smear is inherently subtle,
  and every knob scaled a gain on an already-subtle effect.
- **v2 — posterize + impasto relief + outlines.** Rejected: "visuals just
  aren't interesting." Still a *filter* — the image stayed the image, and
  the controls decorated it.
- **v3 — the picture is REBUILT from discrete paint dabs.** Structural, like
  the programs that stuck (gumball / triangulator / quilt): nothing of the
  original raster survives, so every control changes what the painting *is*.

A cell grid samples the video. Each cell gets one glossy blob of paint whose
colour is the cell's hue quantized to a limited palette and whose SIZE is the
cell's saturation (paint load). **Gray cells get no paint — bare canvas.**

## Controls (each one restructures the image)

| Ctrl | Name | Range of looks |
|---|---|---|
| K1 | Pigments | 2..32 hue sectors — the palette collapses |
| K2 | Dab | 8/16/32/64 px cells — pointillism → chunky impasto (the most drastic knob) |
| K3 | Light | fake-Phong highlight position on every dab (S11 orbits it) |
| K4 | Scatter | perfect grid → hand-thrown: dabs shrink AND wander |
| K5 | Load | everything painted → only vivid colours survive as sparse dots on bare canvas |
| K6 | Smear | round dabs → long horizontal knife strokes |
| P12 | Solvent | dry video → dabs over the photo → photo strips to canvas → glossy varnish |
| S7 | Palette | Free (sampled saturation) / Vivid (full-load poster) |
| S8 | Canvas | Paper (warm white) / Velvet (near-black) |
| S9 | Shape | Round dabs / Square tiles (mosaic) |
| S10 | Rim | dark contour ring around every dab |
| S11 | Orbit | light auto-rotation (keeps static input alive) |

## Architecture

Streaming, **C_LATENCY = 9** (render E1..E6 + a 3-clock inline lerp).

- **Hue sectors without atan2/CORDIC** (kept from v2): octant fold (|cu|/|cv|
  swap + signs) + three PARALLEL shift-add tangent compares → 5-bit sector.
  Runs per pixel but is USED once per cell: on the first line of each cell
  row the centre pixel's {luma8, idx5, ring2, gray} is written to a 256×16
  cell-sample BRAM — **the whole program uses 1 EBR**.
- **Cell engine**: each cell's paint is fully precomputed during the
  *previous* cell's 8 pixel phases (BRAM land → palette angle → registered
  qsin → 2 multiplies → colour/threshold registers), redshift-prefetch
  style, with an hblank sequence priming cell 0 of every line.
- **Per pixel** the dab test is only: |x−cx| → squared-distance logic LUT →
  compare against per-cell-per-line thresholds (body / rim / highlight).
  A **k_\* context pipe** carries each pixel's own cell params down the
  render lanes — the c_\* registers advance to the next cell while pixels
  are still in flight, so testing against them directly would mis-shade the
  last pixels of every cell.
- **Multiplies: 5** (2 palette per-cell, 3 mix). EBR 1. ~4870 LC (63%).

## Timing / correctness battle log

1. **The SDK's `interpolator_u` was the bottleneck.** Its single-stage 10×11
   `s_product` multiply, ×3 instances, held HD Dual to a routed 69.17 MHz
   (5 seeds missing, HD HDMI marginal at 74.98 = structural, not seed luck).
   Replaced with an inline 3-stage lerp, t cut to 8 bits (mercurial squeeze):
   **HD Dual 69.17 FAIL → 87.0 PASS at seed 1**, HD HDMI 74.98 → 89.5, and
   ~400 LCs freed. `result = a + (t8·(b−a))>>8`; worst-case error at full
   solvent is (b−a)/256 ≤ 4 LSB (invisible), and t=0 is still EXACT dry
   video, which is the property that matters. **Rule: this interpolator is
   the wrong tool whenever a program is near a timing wall.**
2. **Clipped "pac-man" dabs at high Scatter** (sim-caught): a dab is only
   drawn within its own cell's pixels, so a jitter of ±8 on a cell of
   half-width 8 pushed dabs past their cell edge and rendered half-moons.
   Fixed by deriving the throw from the cell size (chalf>>z) and having
   **Scatter shrink the dab as it throws it** — r' + |j|max + 1 ≤ chalf in
   every zone, so dabs are mathematically always whole. It also gives K4 a
   real identity: hand-thrown = smaller, wandering, size-varied dabs.
3. Unsigned-underflow guard on the scatter size jitter: `r>>1 − hash` wrapped
   to a giant radius that the upper clamp happily accepted.

## Verification

Sim-checked (lzx-vhdl-cli, dec 4, synthetic hue-wheel scene); renders kept
beside this file:

- `sim_001_pointillist` — dense fine dot spray over a half-stripped photo
- `sim_002_impasto` — long horizontal knife strokes on paper
- `sim_003_mosaic` — vivid square tiles with rims on velvet (stained glass)
- `sim_004_velvet_dots` — scattered colour dots on black; gray areas stay bare

Note: at dec 4 small dabs alias into star/diamond shapes — that is the
decimated sim, not the hardware output (full-res dabs are round).

**HW check items**: hue-sector ↔ pigment mapping may mirror on hardware
(BT.601 sim convention — K1/palette family flips, aesthetically equivalent);
ring thresholds (C_RHI 320) decide what counts as "vivid" for K5 and are
tuned against a synthetic scene — real video may want them lower; dab-vs-cell
gaps at max Dab size.

## Status

v0.3.0 — all 6 configs routed-closed at full clock, **every config exiting
the seed sweep early** (HD Analog 83.9 seed 4, HD HDMI 85.0 seed 4, HD Dual
78.5 seed 1, SD 83.7/85.4/86.8 seed 1). ~4870 LC, 1 EBR. Built + packaged
(out/rev_b/ron/turpentine.vmprog, hash-validated, config verified fresh).
Sim-verified, NOT yet HW-tested.

## Gotchas that cost cycles here

- Preset names ≤ 15 chars; description ≤ 127 bytes.
- **Never ship a reversed display range** (display_min > display_max): the
  TOML validator accepts it but the hardware refused to load the v0.2.0
  vmprog. Invert the mapping in VHDL instead (K1 is knob-up = more pigments).
- A config that "Completes" on the sweep's LAST seed may be a timing MISS —
  the displayed Fmax is the pre-route estimate. Re-verify manually.
- Editing the TOML *during* a build packages the stale config; and
  `program_config.bin` is only regenerated when absent.
