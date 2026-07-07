# Fireworks

A night-sky pyrotechnics show. Four launch slots fire rockets from staggered
pads; each ascends on an eased (decelerating) trajectory with a slight sinusoidal
sway and a glowing trail, then bursts at its apogee into one of five shell
types, plus an optional continuous ground fountain. Rendering reuses the
**spiroscope phosphor engine** verbatim: a 320×180 × 2-bit canvas in BRAM
(29 EBRs), displayed 4×4 through the bilinear upscaler so every spark glows,
with the dithered decay sweep (Persistence slider) providing trails and fading
embers for free.

## Shell types (K3; Mix picks per-shell at random, weighted toward the first three)

- **Peony** — filled sphere; per-spark speeds spread 50–100% of the size knob.
- **Ring** — thin expanding circle (speeds pinned to ~95–100%).
- **Willow** — slower sphere (56–81%) with **doubled gravity droop** — drooping
  golden streamers with long trails.
- **Palm** — 10 thick arms (fixed ~90% speed, forced plus-shaped heads, 1.5×
  droop) — rising comet fronds.
- **Crackle** — peony geometry, but each spark strobes at 50% duty via a
  hash⊕frame gate — crackling glitter.

## Particle engine (memoryless — zero per-particle state)

Every frame each live spark's position is **recomputed from its shell's age**:
two 128-entry ROM curves (one EBR: `ease[t] = 1-(1-t)²` for expansion/ascent,
`droop[t] = t²` for gravity) give `r` and fall; θ and speed come from
`hash(shell seed, i)` (the speed uses a **bit-reversed** particle index —
sequential indices step 6.3° in angle, and without the reversal the
angle/speed correlation draws a visible spiral). The **previous** frame's
position is recomputed the same way at age−1 and the spiroscope 8-connected
line-walker connects the two, so fast sparks draw solid streaks, with an
optional plus-shaped head (S8 / palm / burst-flash). One shared 10×11 signed
multiplier sequences everything on the spiro operand@k → product@k+2 schedule
(~14 cycles/spark; worst case ≈ 5k cycles/frame — trivial). Pistil (S9) is
free: the **midpoint of (spark, center) is exactly the half-radius point**, so
the inner shell needs no extra multiplies. Sparks dim (write 2, then 1) in the
last third of shell life; Twinkle (S7) gates late-life sparks at ~60% duty.
The fountain runs 32 particles whose ages are phase-staggered by hash so the
spray is continuous; a ±22° cone around straight-up, doubled droop gain.

## Color: nearest-shell attribution (zero extra BRAM)

The canvas stores only brightness, so chroma is decided at readout: each pixel
takes the color of the **nearest shell center** — octagonal distance
(max + min/2) to 5 candidates (4 slots + fountain), pipelined as
abs/minmax (stage 3) → distance + argmin level 1 (stage 4) → final argmin
(stage 5) → palette mux (stage 6). Slots keep claiming their fading embers
after death; rockets show gold during ascent and flip to the shell color at
burst. 8-color palette (spiro hues, stored U/V-swapped for hardware); K5 Mix
picks per-shell from the seed.

## Gotchas hit

1. **The SDK `sin_cos_full_lut_10x10` inferred as 6 EBRs** here (yosys packs
   the combinational ROM behind the engine's output registers), overflowing
   the device next to the 29-EBR canvas (36/32). Spiroscope got away with it;
   fireworks did not. Fix: a local **64-entry quarter-wave folded sine LUT in
   logic** — only 256 angles are used anyway (low 2 bits tied "00").
2. **Walker handshake race**: `s_w_active` lags an accepted strobe by one
   clock, so back-to-back issues (pistil right after the main spark) saw a
   stale idle flag and the second request was dropped. All issue sites gate on
   `s_w_active='0' AND s_pl_stb='0'`.
3. **Rocket rise needs 8 product bits**: `ease×height` reaches 138, so a
   7-bit product slice wrapped tall rockets back to the ground.
4. **Full-scale luma reads as WHITE on hardware** (user-reported): luma
   `t*21` peaks at 1008/1023, where every RGB channel clips and saturated
   chroma washes out — all shells looked white. Fix: cap luma at `t*16`
   (768). Spiroscope has the same latent issue (never HW-tested).
5. **Hardcoded 720p layout** left SD rasters half-empty; pads/ground/apogee/
   fountain now recomputed each vsync from the measured raster.

## Timing playbook (57 → 77 MHz routed)

The engine FSM missed HD timing by a mile at first (routed 57.5 MHz,
~80% routing delay). What closed it, in order of impact — the shared theme is
**one registered step per state; never chain hash→add→mux or array-mux→
compare inside one state window**:

1. **Multiplier back to spiro's 8×10** (was 10×11): physics curves stored
   halved (0..126), consumers shift `>>7` instead of `>>8`, speed fractions
   halved. Two fewer partial-product rows.
2. **Hash gets its own state** (Q0/F0): hash → angle/vfrac/table-address
   chained in one state was the worst path.
3. **Every multiplier operand is a plain register**: vfrac registered a state
   before use; rocket height (`ground − apy`) registered at R0.
4. **The 8-comparator range check gets its own state**; the issue state's
   plot-register CEN cone reads a 1-bit flag.
5. **Age/type/seed/center hoisted to slot-stable registers at P0** (s_ccx,
   s_ctyp, s_cseed, …): the inner spark loop never touches a slot-array mux;
   age-derived thresholds (dim level, twinkle, pistil-enable) precomputed.
6. **router2 is load-sensitive** (multithreaded-nondeterministic): configs
   that miss by 1–6 MHz in a full build with a competing build running pass
   solo — HD Dual went 68.5-estimate/miss → 77.5 routed PASS at the same
   netlist. Rebuild stragglers solo + repack (oubliette trick).

## v1.1 (user-driven on-hardware iteration)

- **Two-tone shells**: at readout, the winning slot's distance is also compared
  against that shell's *current half-radius* (published by the engine at P4,
  compared per-candidate in stage 4, flag muxed through the argmin) — inner
  zone = color A, outer = color B. The boundary expands with the shell, so
  spark trails visibly hand off A→B as they fly outward. Mix picks random
  pairs (col2 = col + 2 + seed(15:14)); pinned K5 = white-hot core into the
  chosen color. Fountain: gold core → red tips.
- **Persistent core**: a bright plus replotted at the shell center every frame
  (val 3 to 60% life, val 2 to 87%, then out).
- **SMOKE phase**: burst → 96-frame smoke state — 16 hash-placed dots in a
  flat ±15×±4 cloud around the center, which drifts sideways (seed direction,
  1 cell/4 frames) and rises (1 cell/8 frames); dots thin out with age and
  flicker at 25% duty (continuous replot drew solid sideways streaks); slot
  attribution goes neutral-chroma while smoking/idle, so the cloud and dying
  embers render dark gray and fade to black.
- **P12 = Density** (10..60 sparks + auto-pistil ≥62%), **K6 = Trails**
  (decay thr = 64 + (1023−K6)/2 — bounded, no burn-in mode). The old
  P12=Persistence read as "sideways feedback" and did too little.
- **32-cell edge spawn margin** — near-edge shells lost half their sparks
  off-canvas and looked cut off at the screen sides.

## v1.2 / v1.3 (second + third on-hardware feedback rounds)

- **Crisp sparks** (v1.2): bright cells (2/3) step down EVERY decay sweep, so
  abandoned trail cells collapse in ~2 frames; only val-1 cells (smoke, final
  embers) linger on the dithered slow path. K6 scales that hang time.
- **Isotropic dither** (v1.2): the fade hash was `hash(addr) + framecount` —
  the qualifying set TRANSLATED across the screen every frame, reading as a
  rightward scroll/smear of everything fading. Now `hash(addr) xor
  lfsr-per-frame`.
- **Cell/phase counter rasterization** (v1.3): no divides/shifts — phase
  counters wrap and cell counters increment. 4-px cells on ≤1280 rasters,
  **6-px cells on 1080-class** (1920/6 = 320, 540 field-lines/3 = 180: the
  FULL canvas; the v1.2 8×8 fallback was "very low resolution" per user, and
  its 1280-max 4×4 mode left black side bars on 1080i). Blend runs at 1/8
  sub-cell precision with per-divisor weight LUTs; the span-boundary blend
  commit needs tag3(÷4)/tag5(÷6). Cell dims are MEASURED from the counters
  (cwm/chm) — no division for layout either.
- **Radius-weighted color attribution** (v1.3): raw nearest-center was a
  Voronoi diagram — overlapping shells carved each other into color patches
  ("color pattern overlay" per user). Now stage 5 subtracts 2× the shell's
  split radius (clamped, 8-bit) so a shell owns its own ball; argmin at
  stage 6 (C_LATENCY 6→ +1 stage). 8-bit distances keep the argmin off the
  critical path (10-bit × 3 serial compares was 65-70 MHz).
- **Wind** (v1.3, user-requested): P12 is bipolar wind — center dead-zone =
  calm (DEFAULT: nothing moves sideways, just gravity), left/right =
  direction + strength (global fractional accumulator; carry steps burst +
  smoke centers 1 cell; rockets never drift). Smoke's old always-on random
  side-drift is gone. Density folded into **K6 = Intensity** (sparks 10..60 +
  smoke hang + auto-pistil ≥62%).
- **Fountain REMOVED** (v1.3c): its 14 FSM states + 5th attribution candidate
  were the LC/congestion margin HD timing needed (7000→6530 LCs, 85%);
  **S10 = Smoke On/Off** instead. At 91% util, routed ceilings sat at 65-72
  MHz across 27 seeds with core-infrastructure (SPI) routing as the critical
  path — pure congestion; at 85% all three HD configs pass within 4 seeds.

## Control map

| Ctrl | Function |
|------|----------|
| K1 | Launch rate (relaunch delay 24..535 frames + seed jitter) |
| K2 | Shell size (also scales fountain height ×1.5) |
| K3 | Shell type: Mix / Peony / Ring / Willow / Palm / Crackle |
| K4 | Gravity (0 = space fireworks — nothing falls) |
| K5 | Color: Mix (random pairs) / fixed = white core into that color |
| K6 | Intensity (sparks per shell + smoke hang + auto-pistil) |
| S7 | Twinkle — late-life shimmer |
| S8 | Big Sparks — plus-shaped heads everywhere |
| S9 | Pistil — dimmer half-radius inner shell (auto at high Intensity) |
| S10 | Smoke On / Off |
| S11 | Background Black / Video |
| Slider | Wind (bipolar; center = calm — the default) |

Presets: Grand Finale (everything on, max rate), Golden Willows, Space Garden
(zero-g rings + pistil, long persistence), Quiet Night.

## Status

v1.3 built + packaged (2026-07-06, `fireworks.vmprog` 448299 B,
md5 9de1b5b3…). 30/32 EBRs, ~6530 LCs (85%). **All 6 configs pass routed
timing**: HD Analog 76.1 (seed 7), HD HDMI 75.7 (seed 6), HD Dual 79.8
(seed 9), SD 71-75 MHz. Sim-verified narrow + full-res 1080i.
**HW-iterating with user** across three feedback rounds (white wash → two-tone
+ smoke → resolution/color-patches/wind). Fountain cut in v1.3c for timing.
