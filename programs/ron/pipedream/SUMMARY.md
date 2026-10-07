# Pipedream — a woven Truchet network of glossy 3D pipes

**Status:** v3.2.0 (2026-10-04: no lone rings, 12.5% minimum thickness); v3.1.0 — joint accuracy, smooth highlights, bipolar
Image on S11/P12.  v3.0 HW-seen by Ron ("very interesting now"); v3.1 not flashed.
v2.3 and v3.0 source + vmprog kept in `.pipedream_work/`.

## v3.2 (2026-10-04)
- **No lone rings (Random pattern).**  A ring closes round a vertex exactly
  when the 4 cells around it are elbows with pairings above-left NW/SE, above
  NE/SW, left NE/SW, this NW/SE.  Each cell checks its own top-left vertex
  against the FINAL tiles of left/above/above-left and, if it would close a
  ring, takes NE/SW instead.  A left-to-right, top-to-bottom greedy scan
  checks every vertex once with the other 3 cells final, so none survive
  (Python model: ~121 rings/frame at Weave 0 → 0; also 0 with crossings).
  The row above's final tiles live in a 256×8 EBR (written on each cell row's
  first line, read back on its other lines so all lines agree; writes delayed
  2 clocks to avoid EBR read-during-write).  Applies with Morph on too;
  Rings pattern unaffected.  Larger closed loops can still occur.
- **Minimum thickness**: the per-frame radius coefficients are squeezed to
  `r' = 7/8 r + rmax/8` (floor 12.5% of max, at least 1.25 px), so a pipe never
  vanishes, at no pixel cost.
- Build: 6/6 seed 1 router1 — HD Analog 78.06, HD HDMI 84.88, HD Dual 79.74;
  SD 81.82 / 80.66 / 80.57 MHz; ~6640 LC (86%), 9/32 EBR.

## v3.1 (from Ron's v3.0 hardware notes)
1. *"Pipes at all sizes could meet better and more accurately."*  Measured: the
   v3.0 elbow cross-section was off by 1.4–3.2 px at EVERY scale (`u>>6`
   truncation before the krec multiply = 64/(2wh) px steps, plus the
   first-order series error), and odd cell widths put neighbouring cells' shared
   vertex 1 px apart.  Fix: cells are always EVEN (`wh = 16 + K1>>4`, w = 2wh, so
   every grid vertex is a pixel centre), and the elbow section is now EXACT —
   a 2048×16 EBR table of `round(4·rho) − 4wh` against `rho² >> sqs`, rebuilt
   every vblank by an add-only integer-sqrt walk (advance r4 while
   `(2r4+1)² ≤ 64·rho²`, ≤ 2560 clocks).  Bit-exact model: worst 0.17 px over
   the whole 0.41·wh band at every wh (sqs thresholds 992/1984/3968 so the table
   reaches 2.05·wh²; 1024/2048/4096 left 0.37 px at the range ends).  Deleted the
   krec·u multiply, the etb² and correction multiplies and two stages.
   Membership is one shared test, `|d4| + 2 < rp4` (quarter px), for both
   families.
2. *"Highlights should be smoother and follow the actual curve better."*  The
   hard `clamp(4p, ±omax)` kinked where an elbow's normal crossed the light
   (mocked in NumPy: jagged S-crossovers).  Now the projection goes through a
   smooth saturating EBR ROM, `off = O·p/√(p²+c²)` (O = 22/64 radius, c = 55),
   which stays concentric round most of a bend and crosses over in a gentle S.
   The straights feed their per-frame projection through the SAME ROM (its
   address is muxed per pixel), so junctions stay flush by construction.
3. *"Luma mod on the bypass switch, slider bipolar."*  S11 = Image Off/On
   (Bypass removed).  P12 is bipolar: radius = c0 + c1·L'/64, with c0 = 4r(1−a),
   c1 = 4·rmax·a, a = |P12 − centre|, L' = luma right of centre (pipes start
   thin and grow where bright) or 63 − luma left of centre (pipes start fat and
   thin where bright).  Centre = plain Fatness.  With Image off, P12 drives the
   same thickness from a slow diagonal swell wave, so the slider is never dead.


## Why v3
v2.x hashed every grid EDGE independently: connected by construction, but the
result was a scatter of fragments, flat dead-end stubs and tees/crosses whose
two tubes met along a diagonal miter seam — "kind of silly".  v3 keeps the
glossy-tube rendering and the light model but changes the topology and adds a
video input.

## What it is
- **Truchet tiles.**  Every cell holds two quarter-circle elbows (NE+SW or
  NW+SE pairing) or a straight over/under **crossing**.  Every tile uses each of
  its four edge midpoints exactly once, so every pipe is a continuous closed
  loop or runs off-screen — no dead ends, no tees, no seams.
- **Weave (K2)** = fraction of crossing tiles: 0 = pure looping Truchet, 100% =
  a basket weave.  The over strand alternates on the cell checkerboard
  (`x_idx ^ y_idx`), so a strand goes over, under, over...; the under strand is
  shadowed (half luma + chroma) for 1.5 radii beside the over strand.
- **Pattern (S7) Rings**: the elbow pairing follows the checkerboard instead of
  the hash, closing every elbow pair into a ring around alternate grid
  vertices — chain mail once Weave adds crossings.
- **Image (S11 + bipolar P12)**: see v3.1 above.  The thickness is set per
  pixel, so the outline follows luma continuously across cell joints.
  Program type is **processing** (blanking-gated).
- **Flow (K4)**: pulses circulate ALONG each loop.  Phase = `along * krec >> 5`
  with period 4wh; every N/S port sits at phase 0 and every E/W port at half a
  period, so it is continuous at every joint.  Direction follows the Truchet
  2-colouring (keep parity-0 regions on the left): arcs and H strands use
  sign = parity, V strands the opposite.  At a crossing, H pulses sink into
  the centre and V pulses emerge, as if the light turned the corner.
- **Finish (S8)**: Gloss (offset-light Phong) or Chrome (white streak, bright
  sky, dark horizon band, dim ground).

## How it renders (load-bearing tricks)
- **Cell hash** is a 4-round add/xorshift (`(5,3),(4,10),(9,7),(8,4)` after
  `a*37 + b*146 + seed`); worst neighbour/diagonal correlation 0.04 (the v2
  edge hash measured 0.46/−0.65 with a constant bit 15).  It changes only once
  per cell, so it runs as a 7-deep pipe on the NEXT cell index (x_idx+1 in
  active video, x_idx = 0 in blanking) and is latched at each cell boundary —
  zero cost on the pixel path.
- **Elbow corner** = the nearer of the tile's pair: `dx > dy` (NE vs SW) or
  `dx + dy < 0` (NW vs SE); then the v2 incremental corner-distance squares
  give rho² (adds only), and the vblank-built sqrt table gives the exact
  cross-section (v3.1; v3.0's linearised `(rho²−wh²)/2wh` + correction was
  1.4–3.2 px off).
- **Unit-radius normalisation**: `dn = e * K >> 6`, `K = 2^14 / r_px4` from an
  EBR ROM (r in quarter px).  |dn| = 64 at the wall for ANY radius, so ONE
  256-entry shading ROM (EBR, `{finish, |dn - off|}` → Y8, chroma factor)
  shades every thickness identically.  This replaced the v2 per-frame divides,
  the e² multiply, two barrel normalisers and two gain multiplies.
- **Profiles must be EVEN in et**: an arc's cross-section axis points opposite
  to the straight's at some ports (e = −dx at an NE arc's N port), so only even
  profiles shade both sides of a joint identically.  The look's asymmetry comes
  from the light offset.
- **Highlight** = light·radial (light pre-scaled by krec) through the smooth
  saturating offset ROM (v3.1), in unit-radius units.
- **Rim fall-off** at |dn| ≥ 53 / 60 softens the tube edge against black.
- **Fit savings that made v3.0 fit** (first build was 103%): bypass picture
  delay in an EBR (removed in v3.1 with Bypass) instead of 16×30 flops;
  syncs on a 4-bit pipe; hue angle taken late from the live raster counters
  (a few px offset in a smooth gradient is invisible) instead of an 11-stage
  carry; saturation baked into a scaled sine table (`C_QSAT`, no multiply);
  chroma factor 6 bits; light vector 8 bits; corner select moved into c1.

## Controls
- **K1 Scale** — cell size 32..158 px (always even)
- **K2 Weave** — pure loops → all crossings (over/under)
- **K3 Fatness** — tube radius 0.12..0.40 of the half-cell (elbow pairs kiss
  at the cell centre at 0.41)
- **K4 Flow** — pulses circulating along the loops (speed + width + glow)
- **K5 Hue**, **K6 Spread** — base colour; solid → rainbow across the screen
- **S7 Pattern** — Random / Rings
- **S8 Finish** — Gloss / Chrome
- **S9 Morph** — Static / reshuffling (tiles flip as a hash phase advances)
- **S10 Light** — Fixed (upper-left) / Spinning
- **S11 Image** — thickness driven by the input's luma (On) or a swell wave (Off)
- **P12 Image** — bipolar: left = fat pipes thin where bright; centre = plain
  Fatness; right = thin pipes grow where bright

## Changes vs v2.3 (for the record; v3.1 also removed S11 Bypass)
- Removed: per-edge hash network (tees/caps), S7 Corners Round/Square (square
  Truchet elbows degenerate into a cross), S8 Per-Cell colour (cuts loops into
  patchwork), K4 Shine (now fixed per finish in the shading ROM — Flow took the
  knob; P12 went to Image).
- Added: Truchet weave, Rings, Image luma thickness, Chrome finish, crossing
  shadow, loop-circulating Flow.

## Build
v3.1.0, 2026-10-04 — all 6 configs seed 1, real passes (router1): HD Analog
82.05, HD HDMI 81.08, HD Dual 80.30; SD Analog 79.90, SD HDMI 85.19, SD Dual
83.49 MHz.  ~6520/7680 LC (85%), 8/32 EBR (the sqrt table prunes to its 10 read
bits = 5 uniform 2048×2 EBR slices).  LATENCY 14, the same for all modes.

v3.0 history: `NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron pipedream` (router1
routes this netlist better: hd_analog router1 77.6/80.2 vs router2 72.5–75.9).
v3.0.0, 2026-10-04 — all 6 configs seed 1, real passes: HD Analog 79.76,
HD HDMI 81.73, HD Dual 85.38; SD Analog 81.59, SD HDMI 67.99, SD Dual 84.48 MHz.
~6900/7680 LC (90%), 4/32 EBR.  LATENCY 16, the same for all modes.
Timing fixes that mattered: e·K normalise split on K's operand bits plus its own
clamp stage (the first route was 59.9 MHz); elbow-section clamp after its
multiply; abs-free flow coordinates; keep'd per-stage copies of `s_wh`;
precomputed per-line qy deltas; FSM multiplier registered twice (reads at N+3);
flow pulse split into two stages.
