# Penrose

A **never-ending staircase** — a tribute to M.C. Escher. One large impossible
loop of 32 stairs fills the frame, drawn as an isometric woodcut in paper and
ink, with the courtyard terraced down to its centre.

## Status

**v2.0 — built, timing-closed on all 6 configs, sim-verified. Not yet HW-tested.**

- **Render:** sim-verified via the GHDL image tester (32-step loop, four stair
  sides, terraced courtyard, hatching, outlines, print schemes).
- **Resources:** LCs ≈ 5792/7680 (75%), **BRAM 0/32**, no general multiplier
  (one 12×4 for face shading only).
- **Timing:** all 6 configs pass **post-route** at the default seed (1), tightest
  HD is hd_analog at 86.02 MHz vs the 74.25 target — 16% margin.

## The trick

A Penrose staircase is not a 2D cheat — it is a *real* 3D spiral that only looks
impossible in orthographic projection: moving +1 cell in both i and j while
rising one full cell height projects to the identical screen pixel. So a spiral
of `N = q*D` steps whose net drift is `(D,D)` closes visually onto its own start,
and the seam is invisible. The whole illusion is a plain isometric heightfield;
the renderer is a per-pixel ray-march (forked from [ziggurat](../ziggurat/)).

## v2 — what changed from v1

v1 was one small 12-step loop that filled ~50% of the frame. v2 makes it one
large, coherent loop that fills the frame *and* terraces the courtyard, so the
whole image is stepped and geometrically consistent:

- **32-step ring.** `q=8, D=4, S=10` → N=32 steps, an 11×11 cell ring at
  `(i0,j0)=(6,-3)`, `KF=5`. **All four sides are now full staircases** — v1's 4th
  side was degenerate (1 cell), so v1 only coded 3 sides; v2 codes all four.
- **Terraced courtyard.** The open interior is filled with concentric
  rectangular terraces stepping *down* to its centre (`r = min(fi, gj)`, tread
  drop `r·tstep`). A single-valued heightfield, so it stays consistent.
- **The walking monks are gone.** The impossible loop itself is the
  "never-ending" — no figures, no sprite engine. (~1600 LCs freed.)

A lattice of loops **sharing walls** was considered and is geometrically
impossible here: a Penrose loop needs its (D,D) drift, so a shared wall would
need two different stair profiles at once. Loops can tile but not interlock — so
v2 is one large loop, which reads far better than a repeating wallpaper of small
ones.

Framing: `a = 3*mh/64`, `b = 5*mv/128`, which self-frames to 84%×89% with no
clipping at SD/720p/1080i/1080p (normalized cell units make the fraction
resolution-independent).

## Architecture

Retains the v1 innovations: **exact DDA march** (the ring is mostly empty, and a
diagonal-only march shatters its off-diagonal walls into holes — the fix is one
pixel-global rail bit; KF=5 → 16 candidates), and **normalized cell units** (one
cell ≡ 1024 at every resolution, so every stair constant is compile-time,
`tvw(m)=(m<<10)+R` is free wiring, cover compares against a constant, words are
15-bit). Per-candidate payload is just the two face offsets.

Pipeline: `pix → a1 → a2a → a2b → b1 → b2 → c1 → c2 → c3 → io`.

## Controls

- **K1 Ink** — hatch density.
- **K2 Shade** — face tone depth, shading paper→ink (so dark schemes keep form).
- **K3 Well** — courtyard base depth: shallow dish → deep pit.
- **K4 Scheme** — Parchment / Gallery / Night / Blueprint.
- **K5 Spare.**
- **K6 Hatch** — Fine / Medium / Coarse / Off (line spacing; K1 keeps the weight).
- **S7 Outline** — face-seam ink lines.
- **S8 Video Sky** — engrave the incoming video's luma into the sky as line
  thickness (the only use of the input).
- **S9 View** — Standard / Shallow projection.
- **S10 Terraces** — Stepped courtyard vs Flat floor.
- **S11 Spare.**
- **P12 Terrace Depth** — headline: 0 = flat plaza, up to a deep stepped trough.

## Timing (HX4K, no DSP)

All 6 configs pass **post-route** at the default seed (1), all completing cleanly:

| config | target | routed | LCs |
|---|---|---|---|
| hd_analog | 74.25 | **86.02** | 5792 |
| hd_hdmi | 74.25 | **91.72** | 5790 |
| hd_dual | 74.25 | **87.78** | 5791 |
| sd_analog | 27 | 83.43 | 5787 |
| sd_hdmi | 27 | 80.93 | 5787 |
| sd_dual | 27 | 85.62 | 5800 |

> The build-script Fmax is nextpnr's pre-route ESTIMATE (~10 MHz high) and a hung
> nextpnr still prints a good one — always read the *last* `Max frequency` line
> of a re-run AND check the exit code.

### The timing fight (this is where v2's effort went)

The new geometry blew the area and timing budget several times; every fix was a
congestion-relief move, per the HX4K high-util rule (*remove/relocate logic,
don't add pipeline stages hoping routing improves*):

| step | LCs | routed |
|---|---|---|
| v2.0 pixel-unit-heavy p_a2, KF5, terraces, highlight | 6510 | *hung / ~55 MHz* |
| split p_a2 → a2a (classify) + a2b (offsets) | 6566 | 62 |
| a2a: compute stair-side and terrace heights in **parallel** | 6511 | 62 |
| drop the highlight sweep (was my addition, not requested) | 6041 | 64 |
| **move the 24 side-height adders out of p_a1 into a2a** | 5924 | 69 |
| **precompute the 3 terrace tread heights once/frame** | 5595 | **86** |

The last two were decisive. `p_a1` was fanning `iv`/`ju` out to 24 side-height
adders + 10 range bits + terrace mins in one stage — the worst path was 23 ns of
*routing* against 4 ns of logic (pure congestion). Moving the side-heights to
a2a (compute only the selected side, from a lightweight shifted index) and
reducing the terrace to a 3-way select of pre-computed heights (r is only 0/1/2)
took the routed clock 62→86 MHz.

A **highlight sweep** (a lit band travelling the loop forever, as the motion
element) was built, then cut: even in its cheapest form (carry the winner's step
index, compare once) it routed at only 67–73 MHz across seeds — never clearing
74.25. It costs ~13 MHz of routing margin for motion that was never requested, so
the static piece ships. It could return if the budget is found.

## Lessons

**At high utilization the enemy is routing, not logic.** Every v2 timing fix was
about *where* logic sits and how wide its fanout is, not how deep it is. The
worst path was consistently ~5 ns logic / ~23 ns routing. A stage that fans one
register out to 24 wide adders (p_a1) congests; splitting that fanout across
stages, or collapsing many per-candidate computes into a few precomputed
constants (the 3 terrace heights), is what moved the clock.

**Precompute per-frame, select per-pixel.** The terrace tread height is
`KT - r·tstep` with `r ∈ {0,1,2}` — only three possible values. Computing them
16× per pixel (once per candidate) was the critical path; computing the three
once per frame in vblank and selecting made a2a a 3-way mux. +24 MHz.

**Know when a nice-to-have is too expensive.** The highlight sweep was elegant
and suited "never-ending," but it never routed above 73 across a full seed
sweep. Motion wasn't requested; the impossible loop is already the never-ending.
Shipping the static piece with 16% margin beats shipping motion at −2%.
