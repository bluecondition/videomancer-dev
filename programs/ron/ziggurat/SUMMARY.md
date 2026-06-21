# Ziggurat

Orthographic / isometric colored & shaded box structures. The renderer fills the
**entire screen** with a dense isometric field of stacked boxes (an Escher-style
city), each flat-shaded with a brighter top and darker sides.

## Status

**v0.4 — per-pixel heightfield ray-march (screen-filling).** Replaced the earlier
sprite/line-buffer engine (which could only draw ~9 centered boxes) with a
from-scratch per-pixel renderer that has **no box-count limit** and covers the
whole frame.

- **Render:** sim-verified (GHDL image tester) — dense 3D iso field, 3 shaded faces,
  per-cell colors, varied heights, correct occlusion.
- **Resources:** very lean — LCs ≈ 4000/7680 (52%), **BRAM 0/32**, builds clean in
  ~30 s/config, no router crashes.
- **Timing:** SD passes comfortably. **Full-clock HD is marginal** (~64–78 MHz vs the
  74.25 target) — a dense per-pixel ray-march is at the edge of this HX4K. The HDMI
  HD config gets closest (~74–76); the dual-output config is congestion-bound (~72).
  Pushed from 50→78 MHz by pipelining the color/classify stage and parallelizing
  the march; the remaining gap is fabric/placement-limited.

## Architecture (per-pixel isometric heightfield)

No sprites, no line buffer, no framebuffer. Every output pixel marches the fixed
isometric view-ray through a procedural heightfield in a fixed-depth pipeline and
keeps the nearest box hit.

- **Iso coords, no multiply:** two accumulators give `U = b·x − a·y`,
  `V = b·x + a·y` (step `+b`/pixel, `∓a`/line). With `W2 = 2ab`, the cell under a
  pixel is `i = floor(V/W2)`, `j = floor(−U/W2)` — kept as accumulators with
  remainders (no division). Box width `a` and the per-cell heights scale with
  resolution via shifts.
- **Heights:** discrete levels, `aH = level·ab` (a shift of `W2/2`, no multiply),
  from a cheap per-cell hash → flat / 1 / 2 / 4 cells tall.
- **Cover test (per candidate cell):** `V + aH ≥ W2·i` and `U + W2·j ≤ aH`. Marching
  `i` from front (`iv+KF`) down to `iv` and taking the first cover = nearest visible
  box (occlusion). Back cell always covers ⇒ screen is always filled.
- **Parallel march:** stage A computes every candidate cell's two cover thresholds
  at once (the `KF+1` hashes run side-by-side); stage B does all the cover compares
  and priority-picks the nearest. Both stages shallow.
- **Face/shade:** for the winning box, `u' = U−U_C`, `v' = V−V_C` (corner C from the
  stored thresholds: `U_C = Uhi−W2`, `V_C = Vlo+W2`) → TOP (`u'>0,v'<0`), RIGHT
  (`v'>0, u'+v'>0`), LEFT. Top brightest → sides darker (Shade Contrast); per-cell
  hue from a hash (Color Mode = Varied) or a single Base Hue.

## Controls (active in this renderer)

- **K1 Base Hue** — palette.
- **K2 Box Scale** — cell size / how chunky the blocks are. The lower half (0–100%)
  is the original range; the upper half (100–200%) adds extra reach so the top of
  the knob is ~2x the old max cell size (huge blocks, only a few across the frame).
- **K3 Height Spread** — overall tower height: low → flat slab tops, high → tall
  towers everywhere (scales the three discrete height levels, 1/3/6 cells).
- **K4 Shade Contrast** — top-vs-side brightness (depth of the shadows).
- **K6 Height Pattern** — seed for the height hash (different cities).
- **T7 Outline** — dark face-seam edges.
- **T8 Color Mode** — Solid vs per-cell Varied hue.
- **T9 View Angle** — standard (2:1) vs shallow projection.
- **T10 Luma Mod** — gate buildings by the incoming video: a building is drawn only
  where the video has positive luma in it (sampled/held once per building); dark
  video → that building is hidden (background). The city becomes a relief mask cut
  out of the live video's bright regions.
- **Slider Brightness** — wired.

> Building size is a compile-time constant (`BSHIFT`, 2-cell buildings) — a live
> size knob put a select mux in the timing-critical march and cost ~8 MHz, so it
> was baked instead. K5 and T11 are spare in this renderer.

## Timing journey (HX4K, no DSP)

Keep EVERY per-pixel path shifts/adds/compares — no multiply (one `a·b` per frame
only), no chained adds. The real bottleneck was the **color/classify** stage (3-term
`up'/vp'` adds + outline compares); pipelining it took Fmax 50→72. The parallel
march removed the hash from the cover path. A subtle trap: a running `v_w2i -= W2`
inside the unrolled march loop created a `KF`-deep subtract chain — compute each
cell's `s·W2` as an independent constant multiple instead.
