# SPILLWAY — dam-overflow luma cascade

Luma is water behind a dam. K1 sets the crest; content below it is the calm
reservoir (passthrough), content above it spills: bright pixels pour downward
over the crest, accelerating into vertical streaks, aerating into whitewater,
throwing mist where they land, optionally pooling in a rippling reflective
basin at the bottom. P12 is the master FLOODGATE — haul it open and the whole
frame goes over the falls; let go and the water settles with inertia (damped
slosh, ~1.5 s), never a snap.

The spill direction is the raster scan direction, so the whole engine is
per-column state carried down the frame in scan order — no frame buffer.

## Architecture plan (v0.1 — signed off before RTL)

### Data structures / BRAM budget

Column state lives at HALF horizontal resolution: state column `c = x >> 1`
(1024 entries covers 1920-wide HD; SD uses the low entries). Only the even
pixel of each pair updates state; the odd pixel renders from the registered
result. Every per-column update therefore gets 2 clocks — this is the whole
timing strategy.

All arrays canonical 1W1R, single bank (state written on line y is read on
line y+1 — reader and writer are the same descending scan, no clobber):

| array   | packing (16 b, power-of-2 padded)         | size      | EBR |
|---------|-------------------------------------------|-----------|-----|
| st_wv   | w[15:8] water flux, v[7:0] fall velocity  | 1024×16   | 4   |
| st_uv   | carried U[15:8], carried V[7:0] (top 8s)  | 1024×16   | 4   |
| st_fi   | foam[15:8], impact strength[7:0]          | 1024×16   | 4   |
| st_iy   | impact row (y>>3)[15:8], dark acc[7:0]    | 1024×16   | 4   |
| basin ring | 4 lines × 1024 × 8 b OUTPUT luma (x>>1)| 4×1024×8  | 8   |
| undertow height | per-4px-column tendril height       | 512×8     | 1   |

Total 25 EBR nominal (30/32 is the congestion cliff). Degradation ladder if
routing complains: basin ring → 2 lines (−4), impact row → global basin row
only (−4 merged into st_fi).

### Per-column update (even-pixel cadence, ~4 pipeline stages)

Read of column c issued ~8 px ahead (prefetch the ADDRESS not the data —
inferno lesson). Per active line, at column c:

1. head `h = sat0(Ysrc − crest_eff(c))`; `crest_eff = crest_base +
   gate_lift(c>>6) − floodgate_drop` (gates modulate the WEIR HEIGHT, not the
   flow — mult-free and physically right: a closed gate raises the crest past
   white).
2. discharge `dq = (h>>fs0) + (h>>fs1)` — two-term shift pair precomputed per
   frame from FLOW × floodgate curve (× surge). No per-pixel multiply.
3. `w' = sat8(w − (w>>dsh) − leak + dq)` — dsh (streak persistence) from
   PLUNGE + height class. `v' = sat8(v + g)` while wet, reset when dry.
   Rendered flux thins with speed: `wr = w >> (v>>6)` (necking).
4. `foam' = sat8(foam − (foam>>fdec) + (v>>cs))` — aeration grows with fall
   speed, rate from CHURN.
5. carried colour chases source U/V by `>>2` steps while discharging.
6. impact: when the beam crosses the basin surface row (or a hard dark
   reservoir edge under a heavy streak), write {strength=w, row=y>>3} into
   st_fi/st_iy — read back NEXT frame to render mist ABOVE the impact
   (upward mist can't be done in scan order same-frame; one frame late is
   invisible).

### Render composite (streaming, shift/mux only)

reservoir passthrough (+crest shimmer when armed) → streak overlay (source
dimmed, `wr<<2` luma lift, chroma toward carried-colour/DEPTHS-tint mix) →
foam whitening (LFSR cell noise, 1 col × 4 lines cells, scrolled downward by
a global fall-phase accumulator so foam FALLS; threshold vs foam level) →
freeze glint → mist lift/desat above last frame's impacts → undertow dark
tendrils (dark accumulator per column, vblank-swept coarse heights — the
document approximation for against-scan motion) → basin (mirror-wobbled
read of the 4-line output-luma ring, DEPTHS tint, depth darkening, foam
crown at the surface) → floodgate-top white-out roar.

### Slow dynamics (vblank sequencer, per field)

- FLOODGATE spring: 2nd-order (pos, vel 16 b signed; stiffness/damping =
  shifts), asymmetric — stiff pull-up, ~1.5–2 s under-damped release with 1–2
  visible slosh overshoots. pos drives crest_drop, fs pair, churn/mist boost,
  white-out.
- SURGE: 24 b phase accumulator, ~0.1–0.4 Hz triangle → modulates fs pair and
  churn together (louder = whiter). Rides ON TOP of floodgate position.
- FREEZE: env 0→255 at +2/field (~2 s), thaw −4/field (~1 s), hysteresis.
  Scales g and fall-phase advance toward 0, freezes the LFSR (intentional
  crystallization — static pattern is the LOOK, unlike the inferno jitter
  bug), adds icicle glints, stills basin ripple. Slider position is captured
  in the ice because the spring keeps integrating while frozen output holds.
- BASIN level: 24 b spill-volume accumulator during frame → level += vol>>k,
  level −= level>>drain; surface row = H − level.
- 16 gate envelopes: damped ramps (~1 s full travel) toward LFSR-scheduled
  open/close targets.
- DEPTHS palette: 4 anchors (clear → glacial cyan → teal → flood brown),
  interpolated per frame (mults fine in blanking, registered sequencer steps).
- Column sweep FSM (~512 steps): undertow heights chase dark accumulators,
  impact strengths decay, accumulators clear.

### Fixed point / normalization

Video 10 b. w, v, foam 8 b saturating; v is Q4.4 stretch units, g is Q0.4.
Envelopes Q8.8. Height class (0/1/2 from measured active height ≥540/≥864)
shifts g, dsh and mist band so 480p/720p/1080 look identical in character.
Height measured by counting active lines (taffy idiom — never latch at vsync).
Interlace: basin wobble DDA advances on active lines only (screen-phase rule).
Noise: 32 b xorshift free-running through blanking (never avid-gated), plus a
2-stage per-column mix hash; foam cells correlated vertically, decorrelated
across columns.

### Per-scanline timing at 1080p

Active 1920 px = 960 state updates, each with 2 clocks; prefetch depth 8 px.
Per-pixel logic is shift/add/mux against pre-registered per-frame constants
only. All division-free, multiply-free outside blanking sequencers. Target
74.25 MHz all 6 configs at full clock (no hd_clock_divisor, ever).

## Controls

| ctl | name      | action |
|-----|-----------|--------|
| K1  | CREST     | weir height: luma threshold where spilling begins; spill ∝ head above it |
| K2  | FLOW      | discharge rate: how much of an over-crest pixel converts to falling water |
| K3  | PLUNGE    | gravity: acceleration + terminal streak length (necking with speed) |
| K4  | CHURN     | aeration: how fast falling water goes white; foam grows with fall distance |
| K5  | MIST      | impact spray: rising desaturating fog + sparkle above landings |
| K6  | DEPTHS    | water palette: clear → glacial cyan → teal → flood brown (reservoir stays untinted) |
| S7  | GATES     | continuous weir ↔ 16 flood-gates opening/closing in slow damped sequences |
| S8  | UNDERTOW  | shadows below the waterline climb upward (coarse per-column, frame-late) |
| S9  | BASIN     | plunge pool: fills with spill volume, drains slowly, wobble-mirrored reflection |
| S10 | SURGE     | pulsed dam releases (~0.2 Hz) riding on the floodgate position |
| S11 | FREEZE    | falls ice over (~2 s), LFSR crystallizes, icicle glints; thaw lurch (~1 s) |
| P12 | FLOODGATE | master gate: closed → highlight drips → full breach white-out; releases settle with slosh |

## Prototype findings (spillway_proto.py, 960x540, integer/shift-faithful)

- Rendering is a quarter-step CROSSFADE (alpha 0..3 via shift-add mux), not
  additive: additive water reads as speckle mush.
- Filament PRESENCE must come from long-period cells (4x the brightness
  cell) that scroll with the fall and never re-roll per frame -- per-frame
  re-roll reads as rain/static, not water. Only the fine foam noise re-rolls
  (boiling).
- Coverage thins with velocity (continuity): cthr = min(240,w) - v/2. The
  sheet necks down into fewer, brighter filaments as it accelerates.
- Streak decay dsh must be SLOW (6-9); water that goes over the crest falls
  all the way down. PLUNGE controls stretch + breakup, not evaporation.
- Rock impacts fire ONE-SHOT at bright->dark ledge lips (dp bit), else every
  dark region eats the whole fall in 2 lines.
- Floodgate spring: asymmetric stiffness (open err>>2, release err>>5) with
  SHARED damping (vel>>5 + vel>>6, integrate >>3). Branching the damping by
  error sign kills the ring the moment an overshoot flips the sign. Measured:
  open t63 ~15 f, release dies ~0.6 s, one ~17% backwash lobe at ~1 s
  (the sun visibly re-drips), settled ~2 s.
- Foam scroll ~7-30 px/frame; 100+ px/frame is temporal aliasing
  (indistinguishable from re-rolled noise).
- Basin ring freezes once the beam enters the basin -> stable grazing-angle
  stretch reflection; recursive reflection-of-reflection is thereby avoided.

## Resources / timing (v0.1, 2026-07-21)

All 6 configs built, packaged (out/rev_b/ron/spillway.vmprog) and
ROUTED-verified at full clock -- no hd_clock_divisor:

| config    | seed | LCs       | routed Fmax (74.25 target) |
|-----------|------|-----------|----------------------------|
| HD Analog | 1    | 6921/7680 | 77.08 PASS                 |
| HD HDMI   | 2    | 6930/7680 | 77.01 PASS                 |
| HD Dual   | 2    | 6925/7680 | 79.17 PASS                 |
| SD x3     | 1-2  | ~6920     | 73-85 (27 MHz target)      |

~90% LC, 26/32 EBR. GHDL image sim (lzx-vhdl-cli) verified the render
matches the prototype (rtl_sim_final.png): organic filament curtains, no
diagonal drift, basin + foam correct.

### Timing war story (44 -> 77 MHz routed, in order of discovery)

1. First fit attempt overflowed (7825/7680). LC diet: late-port BRAM reads
   landing directly at the consuming stage (deleted 4 stages of pipeline
   copies), pcell/y8 sliced from finer cells instead of extra barrels,
   rotate-ring gate arrays (head-only, no index muxes), chroma crossfade at
   3 levels, foam clusters from the filament hash's mid-bits.
2. "✓ Completed" lied twice: displayed Fmax is the PRE-ROUTE estimate and
   the retry ladder accepts a best-effort miss on its last seed. Every
   accepted seed was re-run and gated on the post-route Warning line.
3. Routed fails were CONES, not congestion: (a) the state ALU
   (barrel-shift decay + 4-term add + rock attenuate in one clock) ->
   precompute at pair-rate R6, ALU = mux+add+saturate; (b) the rock/impact
   decision + impact-strength compare -> R6 flags; (c) the basin
   reflection darken (variable barrel in E7) -> folded into an idle ry
   delay slot; (d) THE DESIGN-WIDE KILLER: the floodgate spring's one-clock
   19-bit-error + 4-term 20-bit accumulate, and the shared 13x11 multiplier
   -- vblank sequencer logic is still timed single-cycle by STA. Staged the
   spring across three sequencer steps and pipelined the multiplier
   internally (partials + sum; all consumers already read 2 steps after
   load). That one fix moved every config's estimate up 5-10 MHz and ended
   the seed lottery.

### Accepted degradations

Half-res state columns (2 px), chroma water blend 3-level (dither-smoothed),
14 px dry left margin (prefetch-less columns, hidden in overscan), basin
reflection = 2-line output ring frozen at the surface (grazing-angle
stretch, hidden by the deep-water fade), mist one frame late, undertow
frame-late coarse columns, fil/f2 hashes at 2 rounds. Not HW-tested yet.

## v0.2 — render polish ("looks unfinished/glitched" feedback)

Root causes found: (1) every compositor decision was a hard binary
threshold — on/off 2 px filaments (barcode), quarter-alpha posterization,
binary foam blobs, 16-line mist terraces, obvious 4-line basin repetition;
(2) a REAL bug: pipelining the shared multiplier added a cycle, so every
vblank consumer read the PREVIOUS product — crest_drop was stale garbage
and the crest ignored the slider (rendered as a huge dry "wedge" at the
sky's luma contour); (3) the spring pinned at the top clamp for ~40 frames
on fast pulls (integrator windup); (4) the ALU kept firing through hblank
on a frozen phase, clobbering tail columns (right-edge garbage).

Fixes: GRADED opacity from the threshold margin (soft filament edges,
op = min(clip(margin<<2), clip((w-8)<<1))) with the three clipped legs in
R5 and the combines in R6; quarter-alpha + PIXEL-RATE dither of the 6-bit
fraction (smooth 8-bit blend at 60 fps, grain reads as aeration); foam
whiteness graded by cluster margin; mist band-edge dither; drain at 3/4;
basin deep-water fade + surface waterline on the ring path (idle delay
slots); spring anti-windup (clamp zeroes velocity); en_pair/ev_go gated on
active video. Feature trims for timing: 8 gates (spec allows 8-16), 2-line
ring, no spray floor, no mist 3-tap smooth (band dither covers it), glint
luma-only.

v0.2 timing war (all cones found via routed critical-path census):
- Shared-mult REGISTERED sum = stale-product bug; the sum must be
  combinational off the registered partials (same consumer latency).
- The R5 op combine as one serial chain = new cone -> parallel legs.
- The gate rotate-ring's shift wiring + per-pixel lift mux = 44 of the
  critical path's routing segments -> 8 gates, 8-bit lift, 2-substep FSM.
- E5 dither-select + f_am muxes + 12-bit 3-term sum in one clock = the
  final cone -> split into select stage and product stage (C_LAT 7->8),
  with the LFSR dither bits locally re-registered (cross-die net).

Final v0.2: 6830-6856 LCs (89%), 22/32 EBR, all 6 configs ROUTED-verified
full clock: HD Analog 74.87 (s1) / HD HDMI 78.62 (s3) / HD Dual 76.21 (s3).
GHDL image sim confirms the graded render (rtl_sim_v02_final.png).
