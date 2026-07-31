# PYRO — Performable Night-Sky Pyrotechnics

**Status: v0.5 built, packaged, all 6 configs pass routed timing.
HW-iterating. Intended to replace `fireworks` after HW validation.**

## v0.5 — full rainbow + white launches (2026-07-12)

1. **Per-color luma ceiling** (the real "full rainbow" fix): the palette
   chroma was already the scope-verified ROYGBIV set, but sparks rendered
   at a uniform bright luma — and red/orange (or blue/indigo) differ
   mostly by BRIGHTNESS, so everything washed toward the same pale tones.
   `C_PAL_YC` caps each spark's luma at its hue's natural ceiling (red
   ~400, yellow ~840, blue ~320...); the white-hot bloom still hits 768.
   Hue bank labels now Warm / Primaries / Pastel / Red-Wht-Blue /
   Rainbow (default Rainbow).
2. **Launch color**: rockets draw a white head with a yellow fringe
   (aci=White, aco=Yellow). This deliberately reverts v0.2's
   hold-old-colors ascent — white is chromatically neutral so re-claimed
   embers barely shift.
3. **Timing at 89.7% (chip full)**: closing needed three passes:
   (a) line-rate pre-composites `s_bgf`/`s_wf` (no help — yosys had
   already merged them); (b) **split p_color into stages 6a/6b**
   (C_LATENCY 7): the argmin->pastel->palette->luma-cap cone ended inside
   the CORE's blanking register at 15 ns; (c) **free-running engine
   decodes**: relaunch `age>=delay` compares -> `s_expd` flags, drift
   pacing from shared frame-counter decodes (`s_pc_*`) instead of
   per-slot age bits. After (c): hd_analog 74.80 (s3), hd_hdmi 75.29
   (s14), hd_dual 74.42 (s3) — seed hunts collapsed from 24+ misses to
   single digits. Lesson: at ~90% the critical path is whack-a-mole;
   fix the CONES (register the decodes), don't chase seeds.

v0.5: 6850-6873 LC (89.4%), 32/32 EBR. SD 70-72 (s1). Package 473240 B.

## v0.4 — third feedback round (2026-07-12)

1. **Nebula cloud sky** (starfield technique, scaled to the 2 spare EBRs):
   64x32 4-bit seamless 2-octave value-noise tile generated at
   elaboration (`f_neb_init`), 8x16-px texels (512x512-px wrap), read per
   pixel (`p_neb`, free-running; 1-2 px lag is a fixed shift of a
   wrapping texture). Adds up to +60 luma under the gradient with a
   slight desaturation where dense = moonlit clouds.
2. **Comet shell type** (C_TY_COMET): 14 max-speed straight streaks
   (vfrac ~122-125), forced bright heads, half gravity; salted into both
   TYPE banks (~1 in 4, seed(8:7)). The two-tone split recolors the
   trail behind each head.
3. **Color variety**: Random hue bank salts in pastel shells (~1 in 4).
4. **Graduated wakes**: the line-walker now writes walked cells one
   brightness level below the head (head "11", wake "10" -> decays
   through "01") -- every particle drags a fading tail along its own
   motion vector. (Response to "trails always go right": the geometry
   is radial by construction; if a rightward bias persists on HW,
   identify which layer moves.)

v0.4 resources: **6860-6868 LC (89%!), 32/32 EBR — the chip is FULL.**
Routed: hd_analog 74.62 (s20), hd_hdmi 74.46 (s11), hd_dual 74.36 (s28)
-- thin margins, long seed hunts (20/11/28). Any further feature needs a
matching cut. SD 64-70 (s1). Repackaged 473006 B.

## v0.3 — second feedback round (2026-07-11)

1. **Drifting burst centers**: clouds no longer hang on a static anchor —
   the BASE center integrates per-frame in the vsync lifecycle (sink 1
   cell/8 frames when gravity > 0, doubled on the knob's heavy half, none
   at zero; seed-picked sideways momentum at 1 cell/8 or /16). Sparks,
   core dot and color attribution all follow for free. FREEZE halts the
   drift; scrub collapses the pattern around the drifted position.
   NOTE: the first attempt derived drifted-center COPIES per frame
   (s_dcx/s_dcy arrays + adders) = +180 LC -> 86.7% -> zero passing seeds
   in 24. Integrating the base center instead costs a few comparators in
   the once-per-frame cone. Same lesson as micro-bursts: architecture,
   not seeds.
2. **P12 full throw = real launch**: slider reaching the top hands the
   shell off to normal flight (age jumps to 52 and apy is set to the
   hand altitude, so it keeps rising smoothly, sways, blooms and bursts
   on the standard timer). Below full throw, release still detonates in
   place. Reload gesture = pull to 0, push up again (the existing re-arm).
3. **Sky look**: the "dim flashing horizontal lines" were star RUNS — the
   XOR star hash left ccx(1:0) as don't-cares so every star spawned as a
   4-cell dash that twinkled as a unit. A rotate-by-3 XOR term folds the
   low column bits into the compared bits: single scattered stars
   (sim-verified).

v0.3 routed: hd_analog 78.00 (s9), hd_hdmi 75.39 (s11), hd_dual 75.39
(s19), SD 65-72 (s1). Repackaged 463664 B.

## v0.2 — fixes from first hardware session (2026-07-11)

HW feedback: water + sky work; bursts drew 1/4..3/4 ARCS; a color wash
over the whole show; everything appeared to scroll right.

1. **Arc bursts**: the spark angle's TOP two bits were i(5:4) of the spark
   index (via `i<<4` in the hash), so N<=16/32/48 sparks covered only a
   quarter/half/three-quarter circle — and K2 SIZE drives N (default ~42).
   Fix: bit-REVERSED index in the top angle bits (i0 flips sides every
   spark), plain index for the speed scramble (keeps angle/speed
   decorrelated; the angle now registers in `s_ang8` and the sin/cos LUT
   schedule shifted one state later — products still land on time).
2. **Color wash**: (a) default HUE was Gold → all-gold show; now Random.
   (b) C_EMBER cooled everything toward red/gold; now cools within hue
   families (green stays green, blue→indigo). (c) a relaunching slot
   instantly re-claimed its old burst's fading embers as rocket gold;
   attribution now HOLDS the previous shell's colors through the ascent
   and switches only at the new burst.
3. **Rightward scroll**: the star-twinkle "rotating dim quarter" selected
   stars by raw COLUMN residue (`v_sa(1:0)` — the row base is a multiple
   of 4), so the dim set marched left→right across the whole starfield
   (+ its reflection) every 32 frames. Selector now uses row+column-mixed
   bits `v_sa(7:6)`. If a drift is still visible on HW, identify WHICH
   elements move — everything else is frame-static or per-frame-random.

v0.2 routed: hd_analog 77.01 (s1), hd_hdmi 77.14 (s8), hd_dual 78.32 (s4),
SD 56-77 (s1). Repackaged 448088 B.

A flagship, playable fireworks show forked from fireworks v1.3: the
memoryless table-driven particle engine and 320×180 phosphor canvas,
rebuilt around a performance control map — hand-fired shells on the
slider, FREEZE + time scrub, water reflection, and a night-sky backdrop.

## The big idea

Fireworks' engine stores **no per-particle state**: every spark position is
recomputed each frame as a pure function of its shell's age (ROM ease/droop
curves + hash). PYRO exploits that for its signature trick — when FREEZE
(S11) is on, the slider **scrubs the show's timeline ±128 frames**: bursts
collapse back into rising gold rockets and re-explode under manual control
(a unified launch 0..63 / burst 64..191 timeline, computed in two new FSM
states `EST_AGE`/`EST_AGE2`, feeds the replot in place of the frozen ages;
attribution colors follow the same effective timeline).

## Control map

| Ctrl | Name | Behavior |
|------|------|----------|
| K1 | LAUNCH | auto-launch rate; full CCW = manual-only silence |
| K2 | SIZE | burst radius **and** sparks per shell (10..60) |
| K3 | GRAVITY | heavy fast-falling → floaty near-zero-g |
| K4 | HUE | banks: Gold / Primaries / Pastel / Red-Wht-Blue / Random |
| K5 | TAIL | trail persistence: clean dots → ~2 s willow streaks |
| K6 | SPARK | 0..3: smooth / shimmer / heavy shimmer / + 50% crackle strobe |
| S7 | TYPE | bank A peony/chrysanthemum ↔ bank B willow/palm |
| S8 | RING | mixes in ring shells (~25% of launches) |
| S9 | WATER | reflection below a 78% horizon, per-line shimmer wobble |
| S10 | FINALE | 4× launch rate, tight salvo jitter, lingering sky-flash |
| S11 | FREEZE | physics halts mid-air (flicker continues); slider = scrub |
| P12 | FIRE | push up = hand-fly a shell (slot 3, altitude tracks the slider); ease back 48 counts = detonate at that height; LAUNCH-at-zero + slider-at-bottom = blackout |

Performance moves: freeze mid-hand-flight captures the shell as a burst at
that altitude (then scrub it); FINALE + Random hue + K6 max = the climax.

## Architecture

- **Canvas:** 320×180 × 2-bit phosphor BRAM (29 EBR), bilinear 4×4 (6×6 on
  1080-class rasters) upscale, dithered decay sweep during blanking (TAIL).
- **Engine:** 4 shell slots (slot 3 doubles as the hand-fire shell); per-
  frame replot from *effective* age via two 128-entry ROM curves (1 EBR)
  through ONE shared signed 8×10 multiplier. Spark angle/speed from
  hash(seed, i) with bit-reversed decorrelation; line-walker streaks.
- **Night sky:** line-rate gradient + XOR-fold per-cell star hash (~225
  stars, slow twinkle) + 2-bit ambient flash on every detonation.
- **Water:** readout row reflected about the horizon via line-rate row-base
  registers (row*320 precomputed, wobble folded in — stage 2 does ONE
  16-bit add per address), inverted blend weights, stars/sparks reflect
  automatically; mirror dims to 5/8 and shifts blue. Launch pads sit on
  the waterline.
- **Color:** brightness-only canvas; chroma attributed to the nearest shell
  (octagonal distance − 2× radius, 4-way argmin). 16-entry palette: 8 hues
  + 8 pastel variants (pastel = index bit 3, zero extra mux depth). Ember
  cooling by effective age: white-hot <6 → two-tone <88 → C_EMBER map.
  Slow inner sparks hold brightness a tier longer (volumetric read).
- **Geometry:** everything measured from sync edges per vsync
  (registers_in(8) timing ID untrusted on this hardware).

## Cuts (85% LC congestion ceiling; pre-planned order)

Crackle micro-bursts (−170 LC, the big one), flat-ring orientation,
explicit full-throw flick detect (release detector covers the gesture),
negative gravity. Water, night sky, hand-fire, cross-phase scrub, and
FINALE all survived.

## Resources / timing (v0.1, routed Fmax at the accepted seed)

~6480 LCs (84.4%), 30/32 EBR (29 canvas + 1 physics ROM).

| Config | Seed | Routed Fmax | Floor |
|--------|------|-------------|-------|
| hd_analog | 2 | 74.88 MHz | 74.25 ✓ |
| hd_hdmi | 13 | 74.64 MHz | 74.25 ✓ |
| hd_dual | 6 | 74.78 MHz | 74.25 ✓ |
| sd_analog | 1 | 68.71 MHz | 27 ✓ |
| sd_hdmi | 1 | 64.89 MHz | 27 ✓ |
| sd_dual | 1 | 60.32 MHz | 27 ✓ |

Package: `out/rev_b/ron/pyro.vmprog` (451465 B, unsigned, hash-validated).

## Timing lessons (this program)

- **Never read SPI registers inside the vsync decision cone** — the slider
  compare carry chains through `v_hf_go` cost ~5 MHz on every HD config;
  fixed by free-running 1-bit flag decode (`s_sf_*`).
- **Line-rate row bases**: reflection/wobble/clamp folded into `s_rowb/
  s_rowb1` so pixel-rate stage 2 keeps one add per address. Putting the
  wobble adder in front of `f_addr` missed timing on all HD configs.
- **At ~85% LC, remove logic; don't restructure.** Parallel-clamp
  "optimization" and `--opt-timing` both made timing WORSE. Cutting the
  flat-ring mux off the multiplier product path + XOR star hash + line-rate
  row-in flag closed what 36 reseeds could not.
- **nextpnr logs have TWO "Max frequency" lines** (placement estimate,
  routed final). build_programs.sh greps FAIL anywhere so it reseeds —
  and under-reports — passes. Gate on the LAST line via solo make runs.
- **Wrap hand seed-sweeps in `/usr/bin/timeout -k 5 360`** — router2 can
  stall forever (one seed burned 4+ CPU-hours).
- hd_hdmi is the tightest config (HDMI RX pin placement); it needed both
  cuts AND seed 13.

## Build

```
./build_programs.sh ron pyro     # full flow (reseeds from scratch)
```
v0.1's exact bitstreams were built per-config (seeds above) and packaged
manually with `toml_to_config_binary.py` + `vmprog_pack.py --no-sign`.

## Next

- Flash `out/rev_b/ron/pyro.vmprog` via microSD; validate on hardware:
  hand-fire feel (thresholds 96/48/64 are tunable constants), scrub range
  (±128 frames), water shimmer strength (s_wob ±1), star density (<4/1024),
  HUE bank aesthetics, FINALE pacing.
- After HW validation: retire `fireworks`.
- GHDL image-tester sim available for framing checks on request.
