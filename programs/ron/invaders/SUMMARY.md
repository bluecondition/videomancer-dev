# INVADERS (v0.7.6 — resolution scaling; v0.7.3 = trail only while moving, fills the gaps between body parts)

**STATUS: v0.7.3 FINAL except presets — signed off by Ron 2026-10-10**
("OK, finalize"; v0.4.0 had been signed off 2026-10-09, and v0.5–v0.7 are his
follow-ups). Remaining: the three presets are hand-written placeholders;
capture real ones with `./capture_preset.sh` and rebuild. Ship build:
HD Analog seed 3, every other config seed 1. The chip is full (~94 % LC):
any future feature needs a trim first. Earlier ship builds are saved in
.invaders_work/ (invaders_v040 … v072.vmprog);
invaders_v073_TIMING_FAIL.vmprog must never be flashed.

## v0.7.6 — resolution scaling (2026-10-17, not flashed)

- **Before:** sizes were fixed pixels (cells 128, formation 768 x 384, wizard
  128, bunkers 128 x 112, death cloud 384) on a height measured per FIELD.
  At SD 480i (243 lines a field) the shields clamped to the top of the
  screen, the formation started off-screen and was wider than 720 px, and
  every sprite was 2x tall. 720p drew the 1080p sizes on a 720-line screen
  (the "HD" switch W >= 1280 lumped it with 1080p).
- **Now:** the whole game runs in virtual 1080-line units (the
  .scaling_work/RECIPE.md technique). p_vcoord keeps a Q.1 x and a Q.8 y
  accumulator on the same events as pixel_counter, stepped from the timing
  ID (1080p 1.0; 720p 1.5; SD 2.0 across = a 1440-wide 4:3 window, 1080/486
  per frame line; interlaced fields step 2 frame lines, the bottom field
  starts half a step down). Render coordinates, measured W/H, layout,
  speeds and the wizard's zoom DDA all use them, so at 1080p every step is
  1 and the picture is unchanged, 720p is the 1080p game at 2/3 scale, and
  SD is the same game in a 4:3 window (shields at W/8 fractions, formation
  centred, sizes keyed to the height).
  - Wizard DDA: u adds k or 2k per pixel (2k at SD; alternating at 720p,
    from the x half bit). v adds k at avid_start as before, plus the extra
    (y step − 1) k's on the clocks after the previous line's avid_end, so no
    pixel ever sees a half-stepped v (s_vrep; also adds a bottom field's
    0–2 unit start after the vblank load).
  - The 4x4 Bayer dissolve still uses REAL pixel counts (pixel-scale
    texture). The floor line is 5 units tall at 480i (4 elsewhere), so it
    can't fall between that mode's 4.44-unit field lines.
  - The s_hd speed doubling is gone (every raster is virtual-HD; speeds are
    the old HD values in units/frame). Height is measured on top fields only
    (the bottom field ends half a step lower).
  - The march step is the same 12 units at every resolution, so on the
    narrower 4:3 SD window the formation reaches the edge (and drops) in
    fewer steps than at HD — the look per step is identical.
  - On hardware the core ties field_n high on every input, so SD fields are
    line-doubled (both fields draw the same rows), as in mondrian.
- Renders (.scaling_work, "Video Raid"): 720p matches 1080p (r720 0.99,
  edges 0.98); SD shields measure 0.101 of the height at 0.753 down (1080p:
  0.102 at 0.755), centred on the W/8 fractions; wizard, formation and floor
  all whole on the 4:3 screen. (The SD sim runs 14 fields, so it shows the
  post-boot-reset board; the 7-frame HD sims show the pre-reset one.)
- Cost: +82 LUT4 / +15 carry / +26 FF standalone (≈ +100 packed LC).

## Build v0.7.6 (2026-10-17, router2, full clock, ~7080 LC / 92 %, 28/32 EBR)

`./build_programs.sh ron invaders` (default router2, seed retry from 1).

| Config | Seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | 1 | 82.68 MHz | 7079 |
| SD Analog | 1 | 78.44 MHz | 7080 |
| HD HDMI | 1 | 81.35 MHz | 7048 |
| SD HDMI | 1 | 76.88 MHz | 7051 |
| HD Dual | **2** (seed 1 missed) | 75.57 MHz | 7069 |
| SD Dual | 1 | 78.73 MHz | 7059 |

Hardware checklist for v0.7.6: 1080p looks exactly like v0.7.5; at 720p
and SD the game is the same picture scaled (4:3 at SD) — shields above the
wizard, formation centred under the top band, nothing 2x tall; the death
TV and win flight centred and proportional; the floor line steady at 480i.

## v0.7.3 changes (2026-10-10)

- **The trail fills the outside pockets between body parts** (under the
  raised arm, between the legs, between the pointing hand and the eye). It
  "emits" from every trailing-facing surface:
  - `WIZARD_GAP_MASK` = unlit pixels that are outside a 5×5-closed silhouette
    AND lie between two body parts in their row. The closing keeps the art's
    linework out.
  - A gap pixel is shadow when a body group lies within the band toward the
    body (window base..tap).
  - The mask lives in the spare bits of the eye ROM (eye bits only matter on
    lit pixels, gap bits only on unlit ones), so it costs no EBR. It was
    modelled in .invaders_work/trail_model2.py before building.
- **The trail only shows while he moves.** Target length = K2 max (now
  K2(9:3), 0–127 px) × min(speed − 1, 16) / 16. Speed is |Δx| px/frame,
  averaged over 2 frames, with a 1 px deadband; ≥ 17 px/frame (~1000 px/s
  on HD) gives the full K2 length. It eases at +6 / −4 px per frame. At 0
  it is hidden entirely (s_trail_on), and it is forced to 0 during the death
  and to 120 for the win flight. The K2 default went from 384 to 640 (80 px).
- **Timing fixes this needed:**
  - Writing the gap window as an if/elsif accumulation loop synthesised as a
    ~10-LUT ripple (HD 50–55 MHz). It is now explicit term vectors, each
    reduced with one balanced `/= 0`.
  - The outer-trail and gap terms are registered separately in S5 and ORed
    in S6.
  - p_measure now latches pixel_counter's own counts instead of keeping
    duplicate counters.
- At ~7215 LC (94 %) the remaining misses are routing-bound FSM paths, i.e.
  congestion: the chip is FULL. Any further feature needs a trim first
  (debris 3 → 2 slots frees ~150–200 LC).

## Build v0.7.3 (2026-10-10, router2, full clock, ~7215 LC / 94 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | **3** | 77.45 MHz |
| SD Analog | 1 | 72.96 MHz |
| HD HDMI | 1 | 77.21 MHz |
| SD HDMI | 1 | 74.70 MHz |
| HD Dual | 1 | 81.65 MHz |
| SD Dual | 1 | 79.95 MHz |

HD Analog seed sweep: 70.2 / 72.2 / 77.5 / 77.8 / 79.4 MHz (seeds 1–5).
(A first v0.7.3 build failed HD at 50–55 MHz on the ripple; it is kept as
.invaders_work/invaders_v073_TIMING_FAIL.vmprog and must not be flashed.)

## Hardware checklist for v0.7.3

- Standing still: no trail. Slow moves: a short trail (longer with K2 up).
  Fast moves: the full K2 length. It goes away within ~0.5 s of stopping.
- The black pocket under the raised arm (and between the legs) fills with
  rainbow while moving; the art's internal lines stay black.

## v0.7.2 changes (2026-10-10)

- **Death sequence halved (~4.7 s → ~2.4 s, 142 frames).**
  - dt 0–15: the world dissolves out (1 level/frame).
  - The wizard eases to the centre at 1/8 of the distance per frame (was
    1/16) and grows k −8/frame (was −4).
  - dt 16–31: the TV dissolves in.
  - dt 20–79: rumble, 15 frames per shake level, strobe 65–79.
  - dt 80–111: the explosion, 4 frames × 8 vsyncs (C_HUGE_FRAME_TIME 7, was
    15).
  - dt 112–141: the respawn blink, on dt(2).
- **The TV fades over the explosion's second half.** The explosion starts
  with the TV fully up (dt 80); the TV/world crossfade runs dt 96–111, so the
  TV is completely gone exactly as the explosion animation ends.
- **Bug fixed: a mid-size TV at the bottom.** The TV is drawn in the
  wizard's DDA view space, and the view snapped back to 1× at the ship when
  the explosion started while the TV was still up, drawing a 4×-of-1× (256 px)
  TV at the bottom. The zoomed death view is now held until the TV has fully
  dissolved (dt < D_TV_OUT + 16); the wizard is already hidden from the
  explosion on. The rumble shake also stops at the explosion, so the TV holds
  still while it fades.

## Build v0.7.2 (2026-10-10, router2, full clock, ~7150 LC / 93 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 76.03 MHz |
| SD Analog | 1 | 76.93 MHz |
| HD HDMI | 1 | 84.83 MHz |
| SD HDMI | 1 | 77.94 MHz |
| HD Dual | 1 | 82.86 MHz |
| SD Dual | 1 | 73.50 MHz |

## Hardware checklist for v0.7.2

- The death takes ~2.4 s; the explosion starts with the big TV fully up; the
  TV is completely gone exactly as the explosion ends, with the aliens and
  shields dissolving back in over that second half.
- No mid-size TV appears at the bottom.

## v0.7.1 changes (2026-10-10)

- **The whole world dissolves, not just the aliens.** Shields, floor line,
  frozen shots and falling pieces fade with the aliens, so only the giant TV
  (and the wizard in it) is on screen.
- **The explosion goes off inside the TV.** The TV stays fully up through the
  first half of the 8× cloud (dt 160–191). At the halfway point (dt 192–207)
  the TV crossfades into the returning aliens and shields. The respawn blink
  is unchanged (from 224).
- **A per-pixel crossfade through ONE sprite path.** There are two dissolve
  levels, TV and world (0–16). In S2, each pixel's 4×4 Bayer value b picks
  "draw the TV" if b < tv, and the world shows if 15 − b < world. With
  tv + world = 16 these are exact complements, so every pixel is either a TV
  pixel or a world pixel, and the shared alien ROM path serves both during
  the crossfade. The decisions are registered in S2, so the S3 source mux
  stays shallow.
- Cost: about +15 LC (~7150, 93 %); HD margins improved to 6.7–10.8 MHz.

## Build v0.7.1 (2026-10-10, router2, full clock, ~7150 LC / 93 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 81.00 MHz |
| SD Analog | 1 | 69.47 MHz (27 MHz target) |
| HD HDMI | 1 | 85.08 MHz |
| SD HDMI | 1 | 72.82 MHz |
| HD Dual | 1 | 80.94 MHz |
| SD Dual | 1 | 74.95 MHz |

## Hardware checklist for v0.7.1

- During the death only the TV (with the wizard) is on screen: no shields,
  floor or frozen shots.
- The rainbow explosion fills the TV screen while the TV is still up; about
  halfway through, the TV dissolves into the returning aliens and shields.

## v0.7.0 changes (2026-10-10)

- **Death inside a giant TV.** On a bomb hit (or the aliens landing):
  - The formation dissolves out (dt 0–31) as the wizard flies to the centre.
  - One giant CRT-TV dissolves in around him (dt 48–63). He fills its
    screen, with the sawtooth/lightning waveform animating behind him, and
    rumbles there.
  - When he explodes (dt 160) the TV dissolves out (160–175) under the 8×
    cloud, which exactly fills its screen. The formation then dissolves back
    in (176–191) before the respawn blink at 224.
- **The TV costs no sprite memory:** while the formation is faded out, the
  alien sprite path (body ROM + screen ROM, and the S4 alive check forced on)
  draws the TV instead. It lives in the wizard's DDA sprite space at 4× his
  scale with its screen centre (src 27, 34) on his centre (64, 64):
  src = (u + 44) >> 2, (v + 72) >> 2 (two S2 constant adds), in range when the
  high bits are zero. So it grows, settles and rumbles with him: about 820 px
  tall at his 3.2×.
- **Fades** are an ordered-dither dissolve (4×4 Bayer vs a 0–16 level) on
  whatever the alien path draws, so there are no per-pixel multiplies. The
  fade schedule starts on multiples of 16 so dt(3..0) is the ramp step.
- **Layering**, top first: wizard → rainbow cloud → aliens/TV → band-coloured
  objects → edge-shadow. A fresh shot now emerges from behind the wand hand
  (it used to draw over the wizard).
- Cost: ~7010 → ~7130 LC (92–93 %). This is the practical ceiling: HD Analog
  seeds 1–4 gave 76.9 / stall / 71.6 / 77.3 MHz.

## Build v0.7.0 (2026-10-10, router2, full clock, ~7130 LC / 93 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 76.91 MHz |
| SD Analog | 1 | 77.35 MHz |
| HD HDMI | 1 | 84.92 MHz |
| SD HDMI | 1 | 78.11 MHz |
| HD Dual | **2** | 86.02 MHz |
| SD Dual | **2** | 74.59 MHz (27 MHz target; seed 1 = router2 stall) |

## Hardware checklist for v0.7.0

- Bomb hit: the aliens dissolve away while he glides to the centre; a huge TV
  dissolves in around him with the waveforms running behind him; he rumbles
  inside it; the explosion fills the TV screen as the TV dissolves away; the
  aliens dissolve back in; he blinks back in at the bottom.
- Does the dither dissolve read as a fade on the monitor?

## v0.6.0 changes (2026-10-09)

- **New death sequence** (bomb hit or aliens landing), ~4.7 s, world frozen
  until the respawn:
  1. 0–0.67 s: the wizard eases from where he was hit to the screen centre
     (velocity = 1/16 of the remaining distance, one frame late, which still
     settles smoothly) while growing to 3.2× (k 256 → 80, −4/frame), on the
     win flight's zoom DDA and its fx/fy/fvx/fvy/zk registers.
  2. 0.67–2.67 s: rumble, a random per-frame shake on both axes that builds
     ±2 → ±4 → ±8 → ±16 px every 0.5 s, plus the rainbow-band strobe in
     the last 0.5 s.
  3. 2.67–3.73 s: the explosion art at 8× (384 px, rainbow diamond rings)
     centred on screen; the wizard is hidden (new "huge" explosion mode,
     4 frames × 16 vsyncs).
  4. 3.73–4.73 s: he blinks back in at the bottom, invulnerable, and the
     world runs again. When the aliens landed, the game resets instead.

  This replaces v0.4's 1× electrified shake + 4× cloud. The win flight is
  unchanged.
- Gotcha: the screen centre in 12.4 is width·8, built with
  `shift_left(resize(w,16),3)`; `'0' & unsigned & "000"` is ambiguous here
  (the array-of-unsigned trap).
- Cost: ~6780 → ~7010 LC (91 %); HD margins 1.8–4.8 MHz.

## Build v0.6.0 (2026-10-09, router2, full clock, ~7010 LC / 91 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 79.09 MHz |
| SD Analog | 1 | 77.78 MHz |
| HD HDMI | **2** | 76.02 MHz |
| SD HDMI | 1 | 67.34 MHz (27 MHz target) |
| HD Dual | 1 | 76.18 MHz |
| SD Dual | 1 | 71.04 MHz |

At 91 % LC, further features will need trimming first (see the v0.5.0
ablation: 5 → 4 shots and 3 → 2 debris slots each free ~150–200 LC).

## Hardware checklist for v0.6.0

- Bomb hit: the wizard glides to the centre growing large, shakes harder and
  harder (rainbow strobe at the end), blows up in a big rainbow cloud, then
  blinks back in at the bottom.
- Aliens landing: the same, then a reset.

## v0.5.1 changes (2026-10-09)

- **Shot-out shield chunks fall too.** The shot×bunker latch is merged into
  the shield-damage latch (shot / bomb / alien × bunker, priority shot >
  bomb > alien at the same pixel), which records the source plus the
  shot/bomb slot. Every knocked-out chunk now takes the same falling-piece
  path; a shot or bomb that did it is retired. At most ONE chunk per frame is
  knocked out across all three sources (was one for shots plus one for
  bombs/aliens); a simultaneous second hit lands the next frame. This
  removed a latch and a clear-mask term (~6830 → ~6780 LC).

## Build v0.5.1 (2026-10-09, router2, full clock, ~6780 LC / 88 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 80.28 MHz |
| SD Analog | 1 | 81.77 MHz |
| HD HDMI | 1 | 83.75 MHz |
| SD HDMI | **2** | 76.82 MHz (27 MHz target; seed 1 = router2 stall, timed out) |
| HD Dual | 1 | 82.63 MHz |
| SD Dual | 1 | 81.39 MHz |

## v0.5.0 changes (2026-10-09)

- **Aliens keep their colour.** They always draw in the middle-band colour
  (white at K3 = 0), including inside the green shield band, and in front of
  the shields.
- **Aliens crunch the shields.** Alien×bunker lit-pixel overlap is folded
  into the bomb×bunker collision latch (a flag says whether to retire a
  bomb), so each frame the first touched chunk is knocked out and drops as a
  falling piece, exactly like a bomb hit. A separate alien×bunker latch was
  tried first: its third clear-mask term and second debris spawn cost
  ~240 LC (94 %, seed 1 failed).
- **Game over at the bottom of the shields.** The landing test is now "the
  bottom of the lowest row with a LIVE alien ≥ bunker bottom" (was: the
  formation's bounding box reaching the wizard). It is pipelined: live-row
  offset → bottom → compare.
- **Animated TV screens.** The 32×30 screen interior (source rows 18–49,
  cols 12–41 of CRT_TV) is drawn from a 5-frame waveform loop
  (CRT_SCREEN_ANIM: a sawtooth scrolling left over a jagged lightning trace
  scrolling right; both periods divide evenly into 5 shifts, so it loops
  seamlessly) instead of the body frames' "+" and bars. It advances every 6
  vsyncs. Each alien starts at its own frame (C_SCR_OFF, random, with no two
  touching aliens, diagonals included, on the same frame). The body keeps
  its 2-frame antenna/leg wobble. Art generated by
  .invaders_work/screen_art.py. Cost: +2 EBR, a small amount of logic.
- **Wand-shots 5 → 4** (Ron's choice) to get back from 91 % LC / flaky HD
  seeds to 89 % with every tested seed passing.

## Build v0.5.0 (2026-10-09, router2, full clock, ~6830 LC / 89 %, 26/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 76.93 MHz |
| SD Analog | 1 | 77.13 MHz |
| HD HDMI | 1 | 81.09 MHz |
| SD HDMI | 1 | 87.75 MHz |
| HD Dual | 1 | 77.30 MHz |
| SD Dual | 1 | 71.20 MHz (27 MHz target) |

All seed 1 (no hand-installed bitstreams). HD Analog seed sweep: 76.9 /
86.5 / 86.1 / 80.7 MHz (seeds 1–4).

## Hardware checklist for v0.5.0 / v0.5.1

- (v0.5.1) Chunks shot out by the wizard drop away like bomb hits.

- Aliens stay white (middle-band colour) when they reach the green shield
  band, with S9 Cellophane on.
- Descending aliens eat into the shields, with pieces falling away.
- Game over (death, then reset) when the lowest live row reaches the bottom
  of the shields.
- TV screens show scrolling sawtooth + lightning traces, with neighbours out
  of step.

## v0.4.0 changes (2026-10-09)

- **Death instead of reset** (no lives). A bomb hitting a lit wizard pixel
  plays ~2.3 s:
  - 0.5 s electrified: the view shakes ±4 px and the sprite strobes in 16-px
    rainbow bands racing up it.
  - 0.8 s rainbow cloud: the explosion art at 4× (192 px) with diamond
    rainbow rings flowing outward; the wizard is hidden.
  - 1 s respawn blink: invulnerable.

  The world (march, bombs, shots) is frozen through the electrified and cloud
  parts, and bombs in flight are cleared at the hit. When the aliens land,
  the same death plays (frozen throughout), then the game resets. That is
  now the only game over.
- **Falling bunker pieces.** Each chunk a bomb knocks out drops as its own
  32×32 piece of bunker art (a −4 px/frame pop, then 1 px/frame² gravity)
  off the bottom of the screen. There are 3 slots; a full pool skips the
  piece. The art comes from a second, per-chunk-laid-out bunker ROM
  (C_DEBRIS_ROM, +4 EBR). A piece's column is fixed, so it renders off the
  bunker's existing S2 delta, and only y needs a subtractor. Shot hits on
  bunkers still just remove the chunk.
- **Win.** Clearing the wave puffs a rainbow cloud, then the wizard grows
  from 1× to ~6.4× (k 256 → 40, −3/frame) while accelerating up and toward
  the far side, facing his flight with the edge-shadow stretched to 120 px
  (×6.4 on screen). He is gone by ~1.4 s; the game resets at 2 s.
- **Sprite-space DDA for the wizard** (p_zoom / p_dda): u = u_start +
  h_count·k and v = v_start + v_count·k, with u/v_start = 64·256 − centre·k
  from one shared 9-step serial multiplier per frame. It replaces the ship
  subtractors, handles 1× exactly, and gives the win zoom with no per-pixel
  multiply. The ship view is now latched once per frame.
- **Structure:** new p_story (death/win timers, flight, frame flags),
  p_debris, p_zoom and p_dda talk to p_physics only through registered
  one-clock strobes. A keep-duplicated chunk index per bunker fixed a
  seed-dependent 8.5 ns fanout hop into the chunk-alive muxes.
- Cost: ~5780 → ~6870 LC (89 %), 20 → 24 EBR. A first cut was 7253 LC;
  each debris slot is ~200 LC.

## Build v0.4.0 (2026-10-09, router2, full clock, ~6870 LC / 89 %, 24/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 84.68 MHz |
| SD Analog | 1 | 80.20 MHz |
| HD HDMI | **3** (hand-installed) | 83.28 MHz (seed 1 = 74.54) |
| SD HDMI | 1 | 78.52 MHz |
| HD Dual | 1 | 89.57 MHz |
| SD Dual | 1 | 73.49 MHz (27 MHz target) |

HD HDMI seed 3 was hand-installed and repacked (vmprog_pack.py --no-sign);
a plain builder rerun gives seed 1 (74.54, a thin pass).

## Hardware checklist for v0.4.0

- Bomb hit: shake + rainbow strobe, rainbow cloud, blink back in; the aliens
  and bunkers are kept.
- Aliens landing: the same death, then a full reset.
- Bomb-hit chunks fall off the bottom as bunker pieces.
- Clearing the wave: puff, the wizard grows and flies off with a long trail,
  then a new wave with fresh bunkers.
- Pause freezes the death / win animations too.

## v0.3.0 changes (2026-10-09, from Ron's v0.2.0 HW review; HW-approved)

- **Bombs came from one alien at high Bomb Rate.** The 8-bit LFSR stepped
  every clock and was sampled once per spawn. A 1080p frame is 2,475,000
  clocks = 225 (mod 255), so the samples walked a 17-state cycle in which
  columns 3/4 took ~11 of 17 picks. Now a 16-bit Galois LFSR is stepped 3×
  per FRAME (FSM phase 1); the column is the first 3-bit field < 6. A model
  gives a roughly even 14–19 % per column.
- **Maxed Trail was cut flat at the sides.** The shadow was confined to the
  128-px sprite box. It now runs in a "trail box" widened by 128 px each side,
  with the group indices clamped to −1..32, so it follows the silhouette at
  every K2 setting.
- **Wizard colours fixed:** purple body (Y 427 / U 618 / V 747, U/V-swapped)
  and a white eyeball, unaffected by K3 Hue and the cellophane bands.
  `WIZARD_EYE_MASK` (in wizard_invaders_sprites.vhd) is a flood fill of the
  eyeball's lit pixels from inside the white ring, bounded by a disc at
  (col 60, row 57) r 22; it is stable for r ≥ 22, i.e. the art's outlines
  close it. Band-coloured objects (a shot leaving the wand) draw over the
  wizard.
- **Lean pass (6520 → ~5780 LC, 22 → 20 EBR, same look and controls):**
  - Sprite ROMs are stored as 16-pixel words ({row, word} addressing) with a
    16:1 bit select, replacing 128:1 (wizard, bunker), 64:1 (alien) and 48:1
    (explosion) muxes over whole registered rows. The edge-shadow's 4-px
    group ORs come precomputed from a 128×32 group ROM.
  - Background video is tapped 2 clocks deep (gated to black outside its own
    avid) instead of an 8-deep 30-bit FF delay. The picture leads the sync by
    6 px, which is invisible; it also keeps raw video shallow (the
    rollthebones lesson).
  - The ship-x multiply is a serial shift-add (11 clocks per result).
  - The three band-colour LUTs share one read, round-robin.
  - The formation in-grid tests now use the registered S2 deltas, and the
    floor end is precomputed.
- **Timing fix:** the bunker bits' write enables were the new HD critical
  path (FSM phase → hit decode → 64 CENs, ~11 ns routing). `p_bunkers` now
  applies a registered one-hot clear mask on two registered strobes, one LUT
  per bit. HD Analog seed sweep went from 2/4 passing to 6/6 passing
  (76.6–88.6 MHz).

Videomancer-themed Space Invaders. The wizard (128×128) slides along the
bottom on the P12 fader, and any edge of S7 fires a wand-shot. A 3×6 wave of
CRT-TV aliens (64×64 art drawn 2×, 128-px power-of-two cells) marches and
drops behind 4 destructible bunkers (4×4 grid of 32-px chunks). A rainbow
edge-shadow trails the wizard's trailing side and stretches with speed.

## Controls

| Control | Label | Effect |
|---|---|---|
| K1 | Bomb Rate | frames between bomb drops, 71 → 8 (4 bomb slots) |
| K2 | Trail | trail length at full speed, 0 (off) – 127 px; shown only while moving (v0.7.3) |
| K3 | Hue | 16-step sprite colour wheel (0 = white); rotates the cellophane palette |
| K4 | Alien Spd | march period 12 → 1 frames per step (+ kill boost) |
| K5 | Bomb Drop Spd | 2–9 px/frame (×2 on HD) |
| K6 | Shot Spd | 6–13 px/frame (×2 on HD) |
| S7 | Fire | any edge fires (round-robin over 4 shots; 5 before v0.5.0) |
| S8 | BG | Video / Black |
| S9 | Overlay | Solid / Cellophane (arcade strip overlay: top 1/8 red, bunkers + wizard green) |
| S10 | Pause | freezes the game |
| S11 | Reset | rising edge restarts the wave |
| P12 | Ship X | wizard position |

## v0.2.0 changes (2026-10-09)

- **Blanking gate.** Output is forced to y=64/u=v=512 whenever the
  latency-aligned avid is low. Before this, the full-width floor line drew
  through hblank, and sprites at h_count/v_count = 0 drew into the porches.
- BG Black is y=64 (was 0, below black). Drawn luma is capped at 768 (was 940).
- **Render-time collisions** (arcade sprite-collision-register style): S5
  pixel flags feed four first-hit-per-frame latches (shot×alien,
  shot×bunker, bomb×bunker, bomb×wizard). They are snapshotted at vsync and
  committed in FSM phases 1/2. These replace the serial one-projectile-per-frame
  hit scan, which missed bunker hits, killed the wizard on bombs in the
  transparent corners of its box, and read a 3-entry bomb array with a 0–4
  index. An overlap left unresolved latches again next frame, so one latch per
  type is enough. Projectile speeds stay < 32 px/frame, so no shot can jump a
  16-px chunk row.
- Bomb-spawn liveness used `row*6+col`, but aliens are packed `row*8+col`.
  Fixed, so bombs now drop from the lowest LIVE alien.
- Removed the SIM-TEMP formation-init hack, which put SD NTSC in a reset loop
  on every frame.
- HD pacing: the march step and projectile speeds double when the measured
  width is ≥ 1280. The formation now starts just under the red band; defaults
  are K4 40 %, K5 25 %, K6 38 %.
- Explosion is drawn 2× (96 px) and centred in the hit cell.
- S9 was a dead "Mono/Hue" switch (hue hardcoded 512 → U 512 / V 511). It is
  now the cellophane overlay, with K3 Hue live in both modes.
- C_BOMBS 3 → 4.

- Timing work: three new paths came from this pass and were fixed.
  (1) The K2-driven trail easing was a long add-and-compare chain; it is now
  pipelined one op per stage. (2) The collision snapshot hung off the shared
  vsync_start net; p_collide now has a private registered copy (keep), plus
  an FSM settle phase. (3) The real HD killer: the raw vsync_start edge went
  through the reset decision into the 64 bunker-bit write enables (~12 ns of
  routing). Phase 0 now only latches the reset/run/fire flags, and the reset
  and timer ticks run in phase 1 off registered flags. HD Analog seed sweep
  after the fix: 72.9 / 76.8 / 72.5 / 76.8 / 79.4 MHz (seeds 1–5).

## Build v0.3.0 (2026-10-09, router2, full clock, ~5780 LC / 75 %, 20/32 EBR)

| Config | Seed | Post-route Fmax |
|---|---|---|
| HD Analog | 1 | 88.57 MHz |
| SD Analog | 1 | 86.57 MHz |
| HD HDMI | 1 | 81.53 MHz |
| SD HDMI | 1 | 88.53 MHz |
| HD Dual | 1 | 80.08 MHz |
| SD Dual | 1 | 84.54 MHz |

All seed 1. vmprog: `out/rev_b/ron/invaders.vmprog`.
(v0.2.0 was ~6520 LC, 22 EBR, HD Analog seed 2 at 76.78 MHz.)

## Hardware checklist for v0.3.0

- Bombs at max K1 come from all columns.
- Max K2 trail follows the wizard's outline with no flat sides.
- Wizard is purple with a white eyeball at every K3 / S9 setting.
- BG Video background looks the same as before (it now leads by 6 px).
- Then the v0.2.0 items:

- No hblank/vblank garbage (floor line at the left/right porches, wizard at
  the far left).
- Shots erode bunkers on every hit; bombs only kill on a lit wizard pixel.
- Bombs drop only from live aliens.
- S9 Cellophane: red top strip, green bunkers + wizard; K3 rotates the hues.
- Pace at defaults feels right on HD (formation reaches the wizard in ~2 min
  if left alone).
- Hue check: the palette is U/V-swapped for hardware, so hues are only
  trustworthy on HW.

## Community port — fit for the stock SDK (v0.7.5, 2026-10-15)
- The stock LZX SDK builds 8 configs; its SD Standalone core is ~600 LC larger
  than HD. v0.7.4 packed to 7920/7680 LC there (HD Dual 7491).
- v0.7.5, three bit-exact changes:
  1. **Bunker chunks in EBR.** The 64 chunk bits live in two 32 x 1 EBRs
     (even and odd bunkers) instead of 64 FFs plus a registered clear mask and
     per-bit update logic. A pixel needs at most one chunk bit per RAM:
     neighbouring bunkers only overlap on rasters under 512 px, and they are
     never in the same RAM. Erosion is one write per frame; a game reset
     refills in 32 clocks, all in vblank.
  2. **Serial bunker x.** The four positions (1/8, 3/8, 5/8, 7/8 of the
     width) come from one accumulator stepping h/4.
  3. **Bit-test range checks.** The per-pixel object tests "0 <= d < N" are
     bit tests: d(11 downto k) = 0, plus a low-bit compare when N is not a
     power of 2. yosys built a carry-chain comparator for each one, even
     against a constant.
- Proven bit-exact against v0.7.4: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 7920 → 7607, HD Dual 7491 → 7212.
