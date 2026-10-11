# PYRO — Performable Night-Sky Pyrotechnics

**Status: v1.0.0 FINAL — signed off by Ron on 2026-10-10 ("finalize and
add to report").** Defaults were captured from the device; the 5 presets are
hand-written (capture more with ./capture_preset.sh if wanted).

## v1.0.0 — final (2026-10-10)

TOML-only repack of the v0.10 build (version 1.0.0). The bitstreams are
byte-identical to the v0.10 build below (HD Analog seed 2, HD Dual seed 2,
the rest seed 1). Package 464428 B. Added to the Program Shelf report as
Completed.

## v0.10 — sharp heads at 1080p (2026-10-10)

Ron wanted higher-resolution fireworks in HD. A finer canvas is physically
impossible: 4x4-px cells at 1080p would need 2x the chip's BRAM. So the
soft 320x172 phosphor canvas stays for glow, trails and embers, and every
burning spark's HEAD is drawn on top at native 1080p.

1. **Head overlay** (1080p only; 720p/SD unchanged):
   - The engine keeps quarter-cell positions (radius fraction rf x cos/sin,
     droop in quarter cells) and writes one head-table entry per frame for
     each bright spark (val = 11), each rocket and each full-bright core:
     `[30] bold, [26:16] y px, [10:0] x px`, x px = 1.5 x quarter cells.
   - A per-line renderer (`p_head`) at each active line's start stamps
     line y+1's heads into the non-displayed bank of a 2-bank line buffer:
     normal heads are a 3x3 plus, bold ones a 5x5 diamond. It costs 5
     clocks per entry + the span, ~1.3k of the 2200-clock line. During
     the vblank before the frame's first line it stamps line 0.
   - The displayed bank is cleared one pixel behind the beam. The table is
     final (`s_hrdy`) once the engine pass ends, which is inside vblank at
     1080p.
   - Head pixels: luma 768 in the shell's palette colour (whitish hot core),
     never on reflection lines.
   - Alignment is by construction: the line buffer is read at stage-2
     x_count, and with 6-px cells the canvas peaks cell S at x_count = 6S
     (traced through the prefetch/commit pipeline). Rows likewise.
2. **EBR for it** (32/32): canvas 180 -> 172 rows (rows below are reflection
   with Water on, empty sky with it off; pads at row 168); the physics ROM
   is now computed per pass on the shared multiplier (EST_T0..T6:
   ease = 126 - (~i)^2/128, droop = i^2/128, exactly the old table); the
   nebula tile is 32x32 at 16-px texels (same 512 px wrap).
3. **Time-multiplexed colour attribution** (what made it fit). The first
   build was 7053 LC (91%) and HD missed on every seed (58-72 MHz).
   Distance inputs change once per cell, so ONE pipeline now walks the 4
   slots on sub-phases pfx 0..3 for the next cell (`s_pcx1` lockstep
   counter), writes result bank `s_ar` and commits it to `s_ac` when a new
   cell's first pixel reaches stage 5. Standalone: 3651 LUT / 1097 carry
   vs v0.9's 3588 / 1183, so the head overlay is net-free.

6685-6727 LC (87%), 32/32 EBR. Package 464428 B.

| Config | Seed | Routed Fmax |
|--------|------|-------------|
| hd_analog | 2 | 77.69 MHz (s3 78.32; s1/s4/s5 miss) |
| sd_analog | 1 | 73.41 MHz |
| hd_hdmi | 1 | 78.45 MHz |
| sd_hdmi | 1 | 77.94 MHz |
| hd_dual | 2 | 75.72 MHz (s1 router stall) |
| sd_dual | 1 | 73.30 MHz |

Not sim-verified (the default build-and-flash loop). HW checks: heads sit on
their glow, not offset; heads move smoothly (no 6-px stepping); no
stuck/ghost pixels; colours still attribute correctly with the
time-multiplexed pipeline; 720p/SD look unchanged.

## v0.9 — one colour per shell (2026-10-10)

Ron: explosions were always two colours, and colours mixed with other
explosions and changed over time. He wants an explosion to stay one colour
the entire time.

1. **One colour per shell, for its whole life.** No two-tone inner/outer
   split, no white-hot start, no ember recolour (C_EMBER, s_sl_col2,
   s_aco, the far-zone flags and the p_attr timeline flags are all gone).
   Hue banks are single-colour picks: Warm = white/yellow/orange,
   Primaries = R/G/B, RWB, Pastel/Rainbow = any of the 8.
2. **A colour region holds** from a shell's burst until its slot bursts
   again (p_attr updates only while bursting, plus a look-ahead on the
   detonation frame: an ascent at age 63 latches the apogee). This was the
   main cause of colour mixing: on relaunch the region used to jump to the
   new rocket, and the old burst's fading embers got claimed by whatever
   shell was nearest.
3. **Rockets draw white** via a per-slot launch-column test (|x - rocket x|
   <= 2, at/below the head; flag piped stage 3 -> 6a), independent of the
   held region.
4. **Overlaps split at "deeper inside whose ball"**: weighted distance =
   dist - radius + 128, clamped to 8 bits. Before, both shells scored 0
   inside an overlap and the lowest slot took all of it.
5. Still not perfect: colour is attributed per pixel (the canvas stores
   brightness only). Where two bursts physically overlap, the sparks deep
   inside the other shell's ball take its colour, and extremely long tails
   can still recolour when their slot bursts again. A true per-pixel
   colour memory would need EBR the chip doesn't have (32/32). That's only
   possible by cutting the nebula (2 EBR) for a coarse 4-bit colour map
   (offered, not done).

6406-6422 LC (83%), 32/32 EBR. Package 451720 B.

| Config | Seed | Routed Fmax |
|--------|------|-------------|
| hd_analog | 4 | 76.07 MHz (s1 router stall, s2/s3/s5 miss) |
| sd_analog | 1 | 74.99 MHz |
| hd_hdmi | 2 | 77.26 MHz |
| sd_hdmi | 1 | 62.48 MHz |
| hd_dual | 1 | 80.48 MHz |
| sd_dual | 1 | 70.10 MHz |

## v0.8 — Blast slider (2026-10-10)

Ron: "swirl is kind of a weird option, fireworks do not swirl -- maybe an
intensity boost, or live scaling of the blast".

1. **P12 = BLAST** (outside FREEZE): live scale of every burst on screen,
   8..102 cells (`s_sizeK = 8 + b/16 + b/32`, b = glided slider). Slider
   800 = the old K2-max size (83), so the captured look is the default.
   The **top eighth (896..1023) is the climax zone**: a rising share of
   sparks (up to 15/16) gets the bold plus head (`s_hboost` vs hash bits
   9:6), and detonations reload the sky flash at full strength (like
   FINALE). Glided diff/8 per frame; held while FROZEN. Swirl removed and
   the Q schedule reverted to v0.6's single sin/cos pair.
2. **K2 = SPARKS** (sparks per shell, 10..60). The slider owns the size,
   so there's no knob-times-slider multiply. Presets got P12 values that
   keep their old K2-driven sizes (Harbor 469, Willows 555, Pastel Drift
   501, Quiet Night 395); Grand Finale sits in the climax zone (960).
3. **Timing fixes** (the first v0.8 build missed HD Analog on 5/6 seeds,
   71-74 MHz):
   - `p_attr`'s add + 3-compare colour-timeline cone into the colour
     regs' set/reset now goes through free-running per-slot flags
     (`s_at64/70/152`).
   - The stage-2 prefetch column min(pcx+2, 319) is a lockstep counter
     (`s_pcx2`), not clamp -> +2 -> clamp -> add.
   Remaining worst paths are routing-dominated engine-FSM fan-out
   (`s_est`/`s_ei`); wide seed spread (69-80 MHz).

6462-6501 LC (84%), 32/32 EBR. Package 451909 B.

| Config | Seed | Routed Fmax |
|--------|------|-------------|
| hd_analog | 3 | 78.36 MHz (s4 80.45; s1/s2/s5 miss) |
| sd_analog | 1 | 75.00 MHz |
| hd_hdmi | 1 | 79.38 MHz |
| sd_hdmi | 1 | 74.93 MHz |
| hd_dual | **2** | 76.02 MHz (builder took s1 = 74.97, 0.7 MHz margin; s2 was rebuilt alone with `make CONFIG=hd_dual SEED=2`, hand-installed and repacked with `vmprog_pack.py --no-sign`. A plain builder rerun lands s1 again.) |
| sd_dual | 1 | 78.75 MHz |

## v0.7 — Swirl slider, full hue wheel (2026-10-10)

Ron didn't like the hand-fire mechanism.

1. **P12 = SWIRL** (outside FREEZE). Every burst spins about its own
   centre: spark angle += +-S*age/128 (1/256-turn units, 8-bit wrap). S is
   slider/2 glided by diff/8 per frame, and the direction comes from seed
   bit 0. At 0 the bursts are classic radial; at 100% each burst turns ~2
   full times over its 128-frame life, so trails draw pinwheel spirals.
   Moving the slider twists every live burst at once, and scrub still
   works because the angle is a function of age. Cost: offsets computed in
   EST_SW0..SW3 on the shared multiplier, plus a second sin/cos pair for
   last frame's angle in the existing Q schedule (no extra Q states).
2. **Hand-fire removed**, along with blackout and K1 silence (with no
   hand-fire, silence was a dead show). K1's minimum is the slowest
   rate, ~9 s per slot.
3. **FREEZE scrub is relative** to the slider position at the freeze edge
   (+-255 frames), so flipping FREEZE holds the exact frame wherever the
   slider rests. Swirl is held while frozen.
4. **Hue wheel**: the palette had a 120 deg hole at cyan and a 78 deg hole
   at magenta, while Indigo/Violet sat 7.6 deg apart. Index 5 is now Cyan
   (180 deg; U 64 / V 663, luma cap 700) and index 6 is Purple (295 deg;
   U 852 / V 821, cap 470). Both were computed with the recipe that
   reproduces the scope-verified rows within 2 codes; they are not
   scope-checked themselves. Pastels are chroma-halved. Embers: blue,
   cyan and purple hold their hue.
5. **K4 labelled-default trap**: the firmware loads the label INDEX (0..4).
   Values < 5 now read as that bank. Before this, every fresh load opened
   on Warm whatever the TOML said.
6. **Defaults captured from the device** (v0.6 settings): Launch 1023,
   Size 1023, Gravity 134, Hue Pastel, Tail 681, Spark 1023, Ring on,
   Finale on, P12 407 (now = ~40% swirl).

6531-6570 LC (85%), 32/32 EBR. Package 455727 B.

| Config | Seed | Routed Fmax |
|--------|------|-------------|
| hd_analog | 1 | 75.87 MHz |
| sd_analog | 1 | 76.71 MHz |
| hd_hdmi | 1 | 77.07 MHz |
| sd_hdmi | 1 | 77.35 MHz |
| hd_dual | 3 | 77.01 MHz (s1 router stall, s2 miss) |
| sd_dual | 1 | 72.75 MHz |

## v0.6 — bug fixes, hardware contracts, LC reclaim (2026-10-10)

1. **Decay sweep off-by-one (the "trails always go right" bug).** Canvas
   read data lands two clocks after issue (raddr reg + BRAM output reg),
   but the sweep wrote back at a one-clock tap, so cell n got cell n-1's
   decayed value: the whole canvas slid one cell right per frame. Every
   spark trailed right and embers drifted right and wrapped down a row.
   This explains v0.1's "everything scrolls right" and v0.4's "trails
   always go right", which were blamed on stars and dither. Fireworks had
   the same code. Fix: `s_decay_rd2`/`s_decay_k_d2`.
2. **NTSC starvation.** The 57,600-cell sweep never finished in an NTSC
   field (~47k blanking clocks), and EST_SLOT waits for it, so no shells
   were ever plotted on NTSC. The sweep now ends at the last displayed row
   +1 (`s_sw_last`; ~19.5k cells on NTSC, the full canvas on 180-row
   rasters).
3. **Hardware contracts.** Saw-active per-field tick (`s_ftick`: ages,
   frame counter and sky flash advance once per field on serrated analog
   vsync). Generated avid (aurora/cubist recipe). Output gated to
   64/512/512 outside it (the canvas window stayed open into the SD front
   porch). Black is Y=64, not 0.
4. **Slot lifecycle serialized** into the engine FSM (EST_LIFE hoist ->
   EST_LIFED decide -> EST_LIFE1 apply -> EST_LIFE2 dispatch), one copy
   indexed by `s_ei`. It used to be four unrolled copies in the vsync
   cone. The first attempt (decide + write in one state) drove ~300 slot
   FFs' CEN through 5 LUT levels: 12.7 ns routing, HD 63-67 MHz. With
   registered 1-bit action flags + one-hot slot, every CEN is flag AND
   one-hot. The hand-fire FSM runs with slot 3's apply (same priority as
   before). Sky flash reloads at the next tick (1 frame later).
5. `no_rw_check` on the canvas (transparency emulation gone). Plus-head
   arms are head address +-1/+-320 through one adder.

Standalone yosys vs v0.5: LUT4 4087 -> 3755, carry 1584 -> 1315.
Real build: 6578-6624 LC (86%, was 89.4%), 32/32 EBR.

| Config | Seed | Routed Fmax |
|--------|------|-------------|
| hd_analog | 2 | 76.86 MHz (s3 75.80, s1 72.84 miss) |
| sd_analog | 1 | 80.41 MHz |
| hd_hdmi | 4 | 78.83 MHz (s7 72.18 / s8 67.42 missed) |
| sd_hdmi | 1 | 70.56 MHz |
| hd_dual | 2 | 75.48 MHz (s7/8/9 also pass: 74.34/76.37/76.87) |
| sd_dual | 1 | 75.87 MHz |

Remaining near-critical paths are routing-dominated pixel paths that v0.5
had too: the 29-EBR canvas read mux and attribution stage 3. Package:
458850 B. v0.5 vmprog kept at `.pyro_work/pyro_v0.5.vmprog`.

**HW check list for v0.6:** trails/embers should no longer drift right
(they now fade in place, or along each spark's own motion); shells, hand
fire, FREEZE + scrub and FINALE should behave as in v0.5; no thin
coloured lines on HD HDMI; same show speed on analog and HDMI inputs.

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
