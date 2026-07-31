# HYPNOS — concentric-CIRCLE / bullseye generator (v0.3)

1960s op-art target (Vasarely / Bridget Riley / the *Vertigo* spiral). For every
pixel: **true Euclidean distance** from a freely-movable centre → gamma warp →
band it. Pure black & white, **no input video** — a straight generator.

v0.3 adds the **Catch-Up** motion mode (S8), a single **Glide** switch (default
on), an **Invert** switch (S9), a centred default, and a solid (dot-free) centre.

v0.1 used a cheap `max + k·min` metric, which drew octagons. v0.2 computes the
real circle with a **CORDIC magnitude pipeline** (shift-adds only — no multiply,
no sqrt), so the rings are genuinely round. The same CORDIC yields the polar
**angle** almost for free, which drives the **Vertigo spiral** on K5.

The behavioural pillar is unchanged: **the centre travels ±2 screens off-frame**
— on screen a target, at the edge a shallow arc, further out near-parallel
stripes. `dx,dy` are never clamped to the raster.

## Control map (6 knobs + 5 switches + 1 slider)

| Phys | Control | Range / detent |
|---|---|---|
| K1 | **Center X** | bipolar ±2 W, **detent (default) = frame centre** |
| K2 | **Center Y** | bipolar ±2 H, **detent (default) = frame centre** |
| K3 | **Period** | exp: ~½ ring across the frame → alias limit |
| K4 | **Warp** | bipolar, detent = flat. CCW → infinite tunnel, CW → sphere dome (`p = K4/512` gamma on the radius) |
| K5 | **Spiral** | bipolar, detent = pure circles. CCW/CW twist the rings into the Vertigo spiral |
| K6 | **Duty** | band-A : band-B width, 5–95 %, after the warp |
| P12 | **Zoom** | exp dolly — fly into the rings |
| S7 | **Glide** | one-pole slew on X/Y/Z (on the register, so CV drifts too). **Default ON** |
| S8 | **Catch-Up** | Uniform vs Catch-Up: while the centre glides, inner rings lead and outer rings lag, then lock back concentric |
| S9 | **Invert** | swaps figure/ground (black ↔ white) |
| S10 | **Output** | Key (hard 1-bit) / Gradient (soft radial ramp) |
| S11 | **Animate** | off (static) / on (slow outward ring travel) |

The circle→square/diamond **metric morph** from v0.1 lives on in the sibling
program **`hypnos_square`** (it doesn't fit the circle version's timing).

**Centre is a solid filled disc** — the phase is biased by duty/2 so the exact
centre (phase 0) sits mid-band, not on a band edge (which rendered one grey
pixel — the old "centre dot"). No pixel is force-held any more.

## Renderer
Streaming, **no BRAM/line buffer**, 88 % LC, ~26 stages.
- **CATCH-UP (S8)** happens up front (`p_pre1..p_pre4`, before the CORDIC): the
  glide lag `(ex,ey) = target − current centre` is captured each frame; each
  pixel's `(dx,dy)` is nudged toward the target by `weight(r)·(ex,ey)` where
  `weight = clamp(128 − rp/8)` (Q7, `rp ≈ max+min/2` ≈ circular). The CORDIC then
  measures distance from that per-ring shifted centre — inner rings lead
  (weight~1), outer lag (weight~0), no cos/sin/divide needed. S8 off = identity.
- **METRIC = CORDIC** magnitude: `(hi,lo)` vectored to the x-axis over 8 packed
  iterations (2/stage), 5 guard bits; `r = cordx·312 >> 14` undoes the CORDIC
  gain with sub-pixel precision. Verified round to <1.5 px (identical to `sqrt`).
- **ANGLE**: the same CORDIC accumulates `atan(2^-i)` steps → the first-octant
  angle, folded to a full 0–4096 polar angle by the dx/dy signs + swap flag.
- **SPIRAL**: `phase += angle · spiral` (K5). Only the twist mod one cycle
  matters downstream, so an 8-bit term is delayed to meet the radial phase.
- **WARP / PHASE / BAND / SOFTEN** unchanged from v0.1 (log2 → gamma → exp2 →
  `r'·f` → duty compare, with adaptive frequency-based softening).

## Timing (all 6 configs pass, routed-verified where noted)
CORDIC is logic-hungry on this DSP-less HX4K. Key moves to close 74.25 MHz:
1. **Pack 2 CORDIC iterations per stage** — halves the pipeline registers.
2. **Narrow the datapath** (5 guard bits, 20-bit) — shorter carry chains + less
   LC; a 2-iteration stage of 23-bit adds was the first wall.
3. **8 iterations, not 14** — still pixel-perfect circles (and a clean angle for
   the spiral), and the LC drop eases the routing congestion that was the real
   limiter (the shared vblank centre-multiplier surfaced at ~81 % LC).
4. **Unscale via a constant-multiply** (`·312`, i.e. shift-adds) not a shift-sum,
   so small radii keep their low bits (round circles near the centre).
5. **Catch-up = one op per stage.** The first cut computed the weight AND
   multiplied by it in one cycle → an 18 ns path (55 MHz). Splitting into
   PRE1 (rp) → PRE2 (weight) → PRE3 (multiply) → PRE4 (subtract), each a single
   op, closes it. At ~88 % LC timing is seed-sensitive; the build sweeps seeds.

88 % LC, 0 BRAM. **Not yet HW-tested.**

## Verify before HW
`hypnos_proto.py` models the exact integer CORDIC (magnitude + angle) and renders
a contact sheet of every state, including the spiral — regenerate after any
change to the metric / angle / warp math.
