# Pipedream — a generative network of glossy 3D pipes

**Status:** v2.1 — bends are now clean rounded 90° elbows (the ball joints were
removed to make room), with the highlight **tracking the light around each bend**
(`light · radial`), and the straights use the identical projection so every
highlight lines up.  All 6 configs routed and timing-closed (real passes, but
TIGHT): HD Analog 75.1, HD HDMI 74.3, HD Dual 75.7 MHz (vs 74.25 Fmin); SD ≥70
(Fmin 27).  ~7.25k/7680 LC (**≈94%**), **0 BRAM**.  `.vmprog` builds.
**Not yet HW-tested** — HD margin is thin (~0.1–1.5 MHz), so HW-test carefully;
buying more margin would require dropping a feature (e.g. spin-light halves the
light dot-product), not just rearranging logic (an on-demand rho² reclaim was
tried and made timing *worse* — the 4 pixel multiplies in one stage).

Rounded-elbow note (v2.1): a pure elbow (exactly one of N/S + one of E/W) is drawn
as a quarter-circle of radius `wh` centred on the cell corner it wraps.
Membership is an *exact* squared-distance band `(wh-r)² ≤ ρ² ≤ (wh+r)²` — so the
tube OUTLINE matches the straights at every joint — and the shading cross-section
`ρ-wh` is got from `ρ²-wh²` by a per-frame **shift** (`>>round(log2 2wh)`), not a
multiply.  That shift is the load-bearing fit trick: the exact-multiply version
was 100–102% LC and 55 MHz; dropping the ball (~1k LC) got it to 94% but HD HDMI
still congestion-died; the shift freed another ~440 LC AND shortened the path →
89% and all-HD ≥ 77.8.  The only cost is that the gloss dome's *extent* varies
subtly with scale on elbows (outline is exact).  `ρ²` itself is kept incrementally
(qxW/qxE per pixel, qyN/qyS per line — adds only, no multiply).

## What it is
A pure **synthesis** program: it ignores the incoming video and fills the screen
with a network of brightly-coloured, glossy 3D pipes that connect in every
configuration — straight runs, clean rounded 90° elbows, tees, crosses — with a
specular sheen, on a near-black field.  Everything is shaped by the front
panel: size, how densely the pipes connect, fatness, gloss, colour, and a Flow
fader that sends liquid-light pulses coursing along the network.

## How it works
- **Connectivity for free, by hashing shared edges.**  The screen is tiled by an
  invisible cell grid.  A cell opens a pipe on each of its four edges iff that
  edge's hash beats a threshold.  Because an edge is shared by two cells and both
  hash the *same edge coordinate* (a horizontal edge is `f_hash(cx, cy, H)` for
  the cell below and `f_hash(cx, (cy-1)+1, H)` for the cell above — identical),
  neighbours always agree.  So pipes connect by construction with **no stored
  grid and no neighbour lookups** — and the four edge-bits give every join type:
  0 edges = empty, 2 opposite = straight-through, 2 adjacent = elbow, 3 = tee,
  4 = cross, 1 = capped dead-end.  **Connections** (K2) is the hash threshold
  (sparse strands → dense mesh); **Morph** (S9) adds a slow phase to the hash so
  the network reshuffles over time.
- **Rendering** reuses the fake-Phong tube shading: a horizontal pipe (E/W ports)
  and a vertical pipe (N/S ports), each a 1-D rounded cross-section with an
  offset-light diffuse band + tight specular highlight.  A pure elbow is instead
  drawn as a **rounded quarter-turn** (same shading applied to the radial distance
  from the corner it wraps), so bends curve smoothly instead of butting square;
  **Corners** (S7) switches Round vs Square.  The elbow highlight is offset by
  `(light · radial)` — the light vector dotted with the pixel's radial direction
  from the bend corner (two small `9×7` multiplies, shifted down) — so as the tube
  curves the highlight **slides from the outer edge to the inner edge**, tracking
  the light exactly the way each straight tube shifts its highlight toward the
  light.  Crucially the **straights use the identical model**: their highlight
  offset is `wh · light` (per-frame, the same projection the elbow evaluates at the
  port), so every highlight — straight and elbow — comes from ONE unified light
  and they meet flush at every junction, with no step in depth.  (Earlier tries —
  a centred elbow highlight, then a *fixed* radial offset, then a proportional
  elbow against still-*fixed* straights — each left a visible step; unifying the
  model on both is what finally makes them line up.)  Tees, crosses and caps stay
  the straight-tube overlap.  The
  tube's luma comes purely from the shading (bright ambient floor → white
  highlight) — there is no video to modulate it.
- **Bright colour** is generated straight into chroma: a hue angle → `(U,V)` on
  the chroma circle at high saturation (vivid), the luma from the shading.  Hue
  is a flowing rainbow across the screen (**Spread** K6 sets how many cycles;
  the diagonal `x+y` gradient scrolls via a per-frame phase) or a distinct colour
  **Per-Cell** (S8), offset by **Hue** (K5).  Highlights desaturate to white.
- **No divide on the pixel path**; per-line dy and the elbow's corner-distance
  squares advance incrementally (adds only).  The per-frame radius / light-vector
  / range maths + two serial divides (the two shading gains) run in a ~60-cycle
  FSM on one shared 13×13 multiplier in vblank; the elbow's `1/2wh` is a
  power-of-two shift, so no third divide.

## Controls
- **K1 Scale** — pipe/cell size 48..176 px
- **K2 Connections** — sparse strands → dense mesh of tees & crosses
- **K3 Fatness** — tube radius
- **K4 Shine** — broad sheen → pinpoint gloss
- **K5 Hue** — base colour (0..360°)
- **K6 Spread** — one solid colour → full rainbow across the screen
- **S7 Corners** — Round (clean 90° elbows) / Square (butted overlap)
- **S8 Colour** — Rainbow flow / Per-Cell
- **S9 Morph** — Static / slowly reshuffling network
- **S10 Light** — Fixed / Spinning highlight
- **S11 Bypass** — show the input instead
- **P12 Flow** — bright liquid-light pulses coursing along the pipes; max = the
  whole network alight

## History
v0.x/v1 were a *video-driven* version (each cell a pixelated video-colour tube
following the image's contours), with OR-port junction connectivity.  Abandoned
per direction — the video coupling and the visible grid weren't the goal.  v2.0
keeps only the glossy-tube shading and drops all the video machinery (sample,
double-buffer grid, neighbour caches), which frees a lot of logic; the edge-hash
scheme gives richer connectivity far more cheaply.

v2.1 makes bends into clean rounded 90° elbows.  The v2.0 ball joints are gone —
they were the price paid: at 82% base util a full elbow renderer (~1.4k LC) would
not fit with any margin, so the ball renderer was removed to make room, and S7
was repurposed from Joints (Sharp/Beaded) to **Corners** (Round/Square).  Getting
timing to close across all six configs also needed the multiply→shift linearise
(see the status note).  Caps at dead-ends are now flat stubs rather than rounded
(a consequence of dropping the ball); could be rounded later by treating a single
port as a half-elbow.
