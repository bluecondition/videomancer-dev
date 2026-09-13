# SANGUINE — engine summary

v0.4.0 — realistic blood dripping from the top edge. Horror-film practical
blood: randomly-sized beads swell at the top edge, surface tension breaks,
heads fall at individual speeds with stuttering gravity and occasional
diagonal jogs, painting permanent solid bright-red paths that new drips
fall on top of.

## v0.4 changes (2026-08-20 HW feedback round 3)

- **No colour changes at all**: darkening and top-down fading removed; one
  solid bright red everywhere (trail, record, sheen base).  The aging
  palettes, freshness index, wet-top erosion and stain decay are all
  disabled (fidx/erosion machinery left in place but constant-pruned).
- **Permanent paint layer**: the stain record now stores {max depth, bead
  width} and renders as solid bright red along the lane's path.  A finished
  drip respawns immediately, so more blood falls on top and accumulates;
  S10 PURGE is the canvas clear.
- **Diagonal jog is lane-static** (hash), not per-spawn, so the painted
  record lies exactly on every drip's path (no snap when a drip ends).
- K5 TRAIL now only shapes the live falling segment; K6 COAGULATE is
  parked (no effect) until the accumulated look gets its tweak pass.

## v0.3 changes (2026-08-07 HW feedback round 2)

- **Trail = drop width, exactly**: trail half-width equals the bead radius
  at the contact point (solid through the radius, dither 1px outside),
  widening only +1/+2 far above the head (512/1024 rows).  The y-based
  taper is gone; the drop is never wider than the trail touching it.
- **VISCOSITY doubled**: threshold 240→19, gravity 1..16, launch 2..33,
  pauses 8..255 — K2 now sweeps slow fat syrup to fast thin runnels.
  Size mapping r = 3 + mass/32 so big blobs only appear at thick settings.
  (Also fixed: 1.5x-jitter beads at max thickness could exceed the mass
  cap and never release — c_bt3 clamps at 245.)
- **COAGULATE = darkening behaviour**: band 0 pure bright-red trails
  (never darken), band 1 distance-only, bands 2-3 age+distance; darkening
  paced much slower everywhere (distance kicks in at 512/1024 rows;
  c_dry 1..8).  Sheen follows effective freshness.
- **Glints softened**: 870 luma pink-white, ~half the previous size.

## v0.2 changes (2026-08-07 HW feedback round)

- **Opaque compositing**: no translucency anywhere; in video mode the
  picture stays clean except under blood.
- **Top-down trail fade**: per-lane wet-top edge (word E) erodes downward
  (rate from COAGULATE, 2x once the drip ends) until the trail is gone;
  the lane only respawns after its trail has fully cleared.
- **Per-bead variation**: release threshold 0.5-1.5x (size), launch speed +
  gravity jitter, 25% slow-fill beads (quarter rate), shape draw
  (round/taller/wider via pre-scaled squares LUTs), 25% get a diagonal jog
  (~32-row segment at a hash-picked height, offset saturates at 8 px).
- **Mass runout**: mass depletes every 4th frame with no floor; the head
  shrinks (r = 4 + mass/16, trail width = r - 1 tracks its bead), a
  mass-coupled drag cap slows emptying beads, and at <=9 mass the drip ends
  mid-fall and a new random bead respawns after the trail clears.
- **Sheen**: chroma pulled 1/4 toward white + luma, extra +96 luma on the
  diagonal segment; head luma boost reduced to +90 so bead and trail read
  as the same liquid.

## Architecture

- **Drip lanes**: 16 px pitch HD / 8 px SD (max 128 lanes; `lane = x >> lshift`).
  Per-lane state in four 128×16 EBRs: position (signed Q12.4 rows),
  {mass, velocity Q4.4}, {phase, age, per-spawn seed}, {stain depth, stain
  strength}. Fifth EBR: per-lane "cling" surface luma sampled render-side.
- **Lifecycle walk**: vblank FSM, one lane per 7 cycles
  (REQ/W1/W2/LATCH/CALC/WRITE) after a per-frame parameter sequencer
  (8 power-of-2 slots: hemorrhage glide, palette derive, physics consts).
  Phases: IDLE →(hash spawn ∝ FLOW+HEMORRHAGE)→ BEAD (mass swells, bulge
  protrudes) →(mass ≥ viscosity threshold)→ FALL (gravity ∝ mass/viscosity,
  hash-timed pauses = creep/pause/surge stutter, mass deposits into trail,
  stain recorded) → DONE (trail dries in place) → IDLE.
- **Render**: sliding 3-lane window (prev/cur/next) prefetched during the
  scan via the shared BRAM read port (line-start prefetch in hblank; crossing
  shift keyed on the S0 pixel, consumed one stage later). Each pixel tests 3
  candidate rivulets so wandering paths cross lane borders and visually merge.
- **Wander**: two per-lane-phased triangle waves of y (periods 256/128),
  amplitude from VISCOSITY (thick = meander). Same path every respawn =
  "grooves in the surface"; stains align with repeat drips.
- **Pipeline** (9 stages): S0 coords → S1 window snapshot + tri raw →
  S1.5 wander shift+sum, raw diffs → S2 clamps + blob shaping + taper →
  S3 region flags (squares-LUT head test, glint, trail, sheen, stain) →
  S4 priority encode + candidate combine → S5 palette (4-step fresh→dried)
  → S6 composite vs vd tap 6 / syncp tap 6 + blanking gate → p_io.
- **Colour**: parametric path — 16-entry hue table (chroma offsets around
  arterial red, stored U/V-swapped), DEPTH shapes luma+sat per frame; other
  liquids = new table entries. Sim shows swapped hue (blue) — expected.

## Timing lessons (54 → 77 MHz routed, hd_analog)

1. Walk-FSM one-cycle USE (BRAM out → physics case → writeback) was the
   18 ns critical path → split LATCH/CALC/WRITE (vblank slack is free).
2. S2 cone (variable wander shift + two 13-bit subtracts + clamp) → insert
   S1.5 register stage; C_SYNCD 5→6.
3. S3 (blob shaping + squares + priority encode) → shaping moved to S2,
   priority encode moved to shallow S4; S3 registers raw flags.
4. `s_frame(3:0) = s_ucnt(3:0)` stain-decay compare precomputed at U_W1.

## Hemorrhage lessons (v0.1 iteration)

- Glide computed with a narrowing `resize(signed, 10)` wrapped hem > 511
  (591→79) — compute in 12 bits and clamp.
- A surge routed through the bead-swell wait takes ~50 frames to appear;
  hem > 512 spawns straight into FALL (mass 96+jitter, vel += hem(9:5)) so
  the gush reads instantly.

## v0.2 timing lessons (54 -> 76 MHz again)

The v0.2 features initially collapsed timing to ~54 MHz; recovered with the
same precompute discipline (at ~84% LC every new serial cone matters):
1. Walk FSM: CALC split into CALC (vel/mass/phase) + CALC2 (pos/stain/age);
   erosion moved to WRITE; release/runout/drag/gravity-jitter tests resolved
   at LATCH against last frame's BRAM outputs (1-frame lag is invisible).
2. Diagonal jog: y-dependent but constant per (lane, line) -> computed once
   per window-slot event in p_window (2 cycles after the field latch),
   stored in the record, shifted along; zero per-pixel logic.
3. S4 priority: OR-based region flags instead of encoded-code magnitude
   compares; fidx/sts/drf select independently.
4. Bead shape: pre-scaled vertical squares LUTs (C_SQT/C_SQW) instead of
   ady arithmetic ahead of the head test.

## Status

- v0.4 build (2026-08-20): all 6 configs timing-closed, no last-retry
  accepts — HD Analog 74.44 (s1), HD HDMI 75.55 (s3), HD Dual 76.68 (s3);
  SD 68-74 vs 27 needed.  ~77.5% LC (fade/palette prune freed ~500 LC),
  6 EBR.
- Sim-verified: solid single-colour render, permanent accumulation with
  immediate respawn, size/speed/shape variety, opaque video mode.
  v0.1 HW-approved (movement/colours); v0.4 not yet HW-tested.
