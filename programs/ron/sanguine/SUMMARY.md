# SANGUINE — engine summary

v1.0.0 — realistic blood dripping from the top edge. Horror-film practical
blood: randomly-sized beads swell at the top edge, surface tension breaks,
heads fall at individual speeds with stuttering gravity and occasional
diagonal jogs, painting permanent solid bright-red paths that new drips
fall on top of.

**Status: REWORK — marked by Ron 2026-10-06 after flashing v1.0.0 on HD
HDMI ("it's not what I want and tired of trying").**  The accumulated-paint
direction (v0.4 → v1.0) did not land; the HD HDMI build also showed thin
blue line flashes (see Status at the bottom).  The direction for the rework
is still to be decided — do not iterate on the paint look without a new
brief.  v1.0.0 = v0.4 plus the finalization pass below (controls, contracts,
timing); that engineering stands and is reusable.

## v1.0.0 finalization pass (2026-10-06)

Controls (bold-always-on audit):
- **K5 TRAIL → WANDER**: the permanent paint covered the live trail, so
  trail length changed nothing in Matte and only the sheen-stripe length in
  Gloss.  K5 now sets the meander amplitude (tri shift 6..3, the same range
  that used to ride VISCOSITY), so VISCOSITY is a clean thickness/speed axis.
  The live trail is always full length (it only fills the one-frame gap
  between paint and head, and carries the sheen stripe).
- **K6 COAGULATE (parked) → SIZE**: base bead radius 1..8 HD (1..4 SD) added
  to the mass term, clamped at 15; trail and paint widths follow r exactly.
- **S8 CLING** no longer gated behind S7 Video: it samples the INPUT luma,
  so drips pool on an unseen surface over black too.

Hardware contracts:
- **Serrated-vsync saw-active guard**: only the first vsync edge after
  active video restarts the frame sequencer.  Unguarded, each extra edge
  bumped the frame counter, re-stepped the Hemorrhage glide, could launch a
  second lane walk per field, and could cut a PURGE off mid-walk at SD.
- Blanking gate, single fixed latency (now 9 stages, C_SYNCD 7), 12-bit row
  arithmetic — unchanged and re-checked.

Engine:
- **Paint record exact**: word D = {deepest head row (12 b), widest radius
  (4 b)}; the painted path ends with a rounded cap (squares test on rows
  past the end) instead of a flat 8-row-quantized edge.
- **mem_e dropped** (6 → 5 EBR): shape moved into word C's dead age bits;
  the stored drift was already overridden by the lane hash, wet-top was
  always zero.  Ageing palettes, freshness index, decay/erosion constants,
  trail-length compare and the +1/+2 trail widening all pruned.

Timing (HD Analog was 74.44 on seed 1 = 0.2 MHz margin):
1. CALC2 cone (pos add → bottom-edge compare → phase enable) → bottom test
   resolved at U_LATCH from last frame's pos against a per-frame `s_lim`.
2. Paint width add (c_rbase + mass, clamp, max) moved to U_LATCH (`u_wid`).
3. S3 LUT fetches (shared between head and paint-cap squares tests) were the
   per-pixel critical path → new S2.5 stage registers |dx|, |dy| and all
   five squares values; S3 is adds + compares.  Seed 1 HD Analog 71 → 83.

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
  {mass, velocity Q4.4}, {phase, shape, per-spawn seed}, {paint depth rows,
  paint radius}. Fifth EBR: per-lane "cling" surface luma sampled render-side.
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
  amplitude from K5 WANDER. Same path every respawn = "grooves in the
  surface"; the paint record lies exactly under repeat drips.
- **Pipeline** (10 stages): S0 coords → S1 window snapshot + tri raw →
  S1.5 wander shift+sum, raw diffs → S2 clamps + blob shaping + paint-end
  rows → S2.5 |dx| |dy| + squares LUT fetches → S3 region flags (head,
  glint, trail, sheen, paint incl. rounded cap) → S4 priority + candidate
  combine → S5 palette + sheen → S6 composite vs vd tap 7 / syncp tap 7 +
  blanking gate → p_io.
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

- v1.0.0 build (2026-10-06, router2 builder): all 6 configs timing-closed,
  no best-effort accepts.  HD Analog 81.92 (**seed 2** — seed 1 was a
  router2 stall, not a miss), HD HDMI 78.34 (s1), HD Dual 77.67 (s1);
  SD Analog 70.45 (s1), SD HDMI 76.81 (**seed 2**, stall on s1), SD Dual
  73.43 (s1).  ~77% LC, 5 EBR.  Standalone HD Analog seed spread 70-83.
  vmprog packed with the v1.0 labels (Wander/Size confirmed in
  program_config.bin).
- **HW 2026-10-06 (v1.0.0, HD HDMI, seed 1 build): thin single-line BLUE
  flashes across the full width, over black too.**  The program cannot emit
  blue in Black mode (stored Cb never exceeds 596; black is 0/512/512), so
  the lines are downstream of the program (TX pin/link).  Standing process
  = re-seed hd_hdmi + HW A/B: `.sanguine_work/sanguine_hdmi_s2.vmprog`
  (87.1 MHz) and `sanguine_hdmi_s4.vmprog` (83.2) are the same RTL with only
  hd_hdmi re-routed.  Pin-delay report (CLK ~4.6-4.8 ns, data 1.4-4.2,
  HSYNC 4.0/6.1/4.9/6.7 for seeds 1-4) shows nothing that singles out
  seed 1.  Pending program-side tidy once the A/B is read: black/blanking
  luma 0 → 64 (legal level; held back so the A/B stays a pure re-seed).
- v0.4 build (2026-08-20): all 6 configs timing-closed, no last-retry
  accepts — HD Analog 74.44 (s1), HD HDMI 75.55 (s3), HD Dual 76.68 (s3);
  SD 68-74 vs 27 needed.  ~77.5% LC, 6 EBR.
- v0.1 HW-approved (movement/colours); v0.4 and v1.0 not yet HW-tested.
