# Kudzu — luma as life

Third of the "luma as X" family (mercurial = *material*, redshift =
*mass-energy*, kudzu = **life**). The picture is a habitat: brightness is
fertile ground and a living organism colonizes it. **P12 "Season"** sweeps
winter → spring → summer → autumn:

| Zone | Look |
|---|---|
| 0–25% WINTER | dormant: die-back, video graded cold/desaturated (shift-only grade) |
| ~31% SPRING | vigorous growth, fresh palette, rising spores that **seed new colonies** where they respawn |
| ~56% SUMMER | full canopy, deep palette, blossom dots on mature growth |
| ~81% AUTUMN | palette turns amber, growth stops, falling leaves **pile up as litter** on bright surfaces; slider end = first frost |

## The three mechanisms (vs. its siblings' IIR fields)

1. **Contagion growth field** — vitality can only appear where the ground is
   fertile AND life is adjacent: a living E/W neighbour (3-tap shift of the
   row read stream, Jacobi old-values), the per-column **lighthouse**, the
   ground row (vines rise from the bottom of frame), a spore seed, or
   spontaneously where fertility is extreme. Colonies have moving frontiers
   instead of appearing everywhere at once.
2. **Vertical lighthouse** (256×16 {val u8, row u8}) — each column remembers
   its strongest growth site; influence decays with row *distance*
   (shift-only), so vines climb up/down across fields without a second row
   read port. Spring spores couple particles→field by writing lighthouse
   entries {200, random row} when they respawn.
3. **Coverage-weighted sway** — warp dx = qsin(line phase) × (bilinear
   coverage × wind): the canopy waves, uncovered picture is rock-still.
   Wind also scales a per-line hash rustle (storm = shimmer chaos).

## Controls

K1 Vigor (growth rate + spread threshold + climb reach) · K2 Wind (sway,
lean, particle drift; center = calm) · K3 Foliage (leaf lattice 8–64 px +
edge raggedness; threshold auto-compensates the jitter mean) · K4 Species
(leaf hue via 6 borrowed qsin taps; autumn pulls all species toward amber) ·
K5 Spores (particle density) · K6 Fertility (luma threshold) · S7 Polarity
(Bright/Dark Soil) · S8 Blooms · S9 Spores+litter · S10 Evergreen (no
seasonal die-back; video-change die-back stays) · S11 Timelapse (2×).

## Architecture

Streaming pipeline S0..S14 (`C_LATENCY = 15`), redshift chassis: single-bank
full-color warp buffer (11 EBR), coarse cell grid ({vitality, maturity} in
one 2048×16 RAM), per-column fertility vacc IIR, particle engine (spores
rise / leaves fall+accumulate, one per 8-px column), single-add vblank
sequencer. Growth updater: one cell row per hblank, 1 cell/clk, **8-deep**
(issue/land/shift/inter/decide/apply/write). Leaf render: coverage+jitter
gates, diamond-lattice hash veins, maturity two-tone + darkening, sub-cell
diamond bloom dots (second hash track), half-blend soft edge.

**EBR 23/32**: warp 11 + grid 8 + vacc 1 + vert 1 + particles 2.
**Multiplies (8)**: 6 bilinear + amp (coverage×wind) + sway (qsin×amp).
Winter grade is shift-only (no grade mults). ~7400 LC (96%).

## Timing battle log (74.25 target, all routed numbers)

1. v0.1: 46.3 MHz — the whole grow/decay/mature decision in ONE updater
   stage (max → threshold compares → add/sub → clamp → maturity). Split into
   U4 booleans / U5 vitality / U6 maturity + hoisted the lighthouse row
   abs-distance into U2 → 69.1. **Updater paths are STA-timed at full clock
   even though they only run in hblank** (ziffern lesson, again).
2. v0.2: critical path = display prefetch address cone: inline
   `v_col+2 >= s_cols` / `s_cols-1` arithmetic + a 4 ns routing hop from the
   frame-latched geometry FFs. All prefetch addresses are now registered
   per-line values (dsp_rb+1/+2/+last as single adds of registered values at
   hsync; per-frame constants latched with the geometry decode) → 71.3.
3. v0.3: critical path = particle FSM PU_DECIDE fused range-compare chain
   into the seed-write enable. Added PU_CALC2 state registering every range
   compare as a 1-bit flag → pre-route 76+, routed pass seed-dependent at
   ~96% LC. Final packaged set (all genuine ROUTED passes, verified at the
   accepted seed per the second-"Max frequency"-line rule): HD Analog s4
   81.75, HD HDMI s5 78.56, HD Dual s7 74.95 (found by extending the sweep
   past the script's 6 seeds — at this utilization the seed lottery
   matters), SD all s1 (75.8–79.7). hd_dual repackaged via the standalone
   vmprog_pack.py.

## Display/sim bugs found (1080i dec-1 sim)

- **Span-0 rolling-prefetch collision**: on cell-row-end lines the growth
  updater's walk can still own the grid read port when span 0's rolling
  prefetch fires → 1-line canopy dashes at cell col ~2. Fixed structurally:
  the hblank prefetch now also loads col 2's corners (pf_seq 8 steps) and
  span 0 skips the rolling prefetch, so the line's first display read lands
  ~36 px in.
- **1080i bottom-row address wrap**: the last cell row's lower-corner base
  (2040) + col wraps the 11-bit address into row 0 → phantom canopy at the
  bottom edge. Fixed: when the new top row is the ground row, the bottom
  corner row replicates instead of advancing.
- **Parked-particle wrap**: spores in dark columns park at the 1023
  bright-surface sentinel; the 2-row hit window wraps onto lines 0–1 →
  dashes across the top of frame. Fixed with a per-column visibility flag
  (pos < measured field height), computed one prefetch phase after the BRAM
  data lands.

## Status

v0.1.0 — all 6 configs routed-closed at full clock (no hd_clock_divisor),
HD passes verified at the accepted seeds. Built + packaged
(out/rev_b/ron/kudzu.vmprog); **not yet hardware-tested**. Sim-verified
(GHDL image-tester): spring colonization from the ground up, summer canopy
with blooms + old-growth two-tone, autumn olive shift + falling leaves,
winter cold grade, wind sway localized to coverage, clean frame with growth
suppressed (no gate false-positives).

HW check items: leaf/bloom/litter hue family (constants are BT.601-standard
like the sim; if hardware shows swapped hues mirror the U/V pairs — leaf
palette comes from qsin taps, so swap the +256 tap to the V capture), sway
feel at speed, spore seeding rate (s_seed8 = spr8>>2), litter accumulation
rate on real footage.
