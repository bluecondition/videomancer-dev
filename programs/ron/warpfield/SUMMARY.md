# Warpfield — radial starfield: stars fly at you (or away)

## What it does
Starfield's radial sibling. Instead of 8 sideways parallax layers, 6 zoom
*shells* stream stars out of (or into) a vanishing point: each star spawns
far — small and dim — grows through the four glow-kernel levels as its
shell approaches, and sweeps radially outward with speed proportional to
its distance from centre, the classic warp/hyperspace field. Star shapes,
twinkle, the four shimmer palettes, B&W mode, the Deep Space nebula
background and Luma Mode (video brightness → star density and size) are
inherited from starfield. The vanishing point can wander in a slow
Lissajous (K6 Drift), scaled to the measured picture so it never leaves
the frame. Direction reverses the flow: stars recede and vanish into the
deep.

## How it works
Per shell, scaled coordinates S = BIAS + (pixel − centre)·k are pure
accumulators (+k per pixel, +k per active line); k is a 4.12 zoom step
swept one octave (4096 → 2048) per cycle from a 256-entry exp2 EBR table
(linearly interpolated via the shared multiplier — raw 256-step zoom
would judder ~2 px radially at slow warp),
so cells are 16–32 px on screen and the kernel magnifies 1×–2× for free.
Shell phases sit ⅙ apart on a global 24-bit phase accumulator; bits 19:16
of the per-shell sum form a re-seed epoch XOR-folded into the hash seed,
so a wrapping shell respawns a fresh population exactly while its fade
envelope passes through zero. Depth drives the glow level (with per-star
dither between adjacent levels; a brightness ramp per level is baked into
the kernel ROM — all 6 ROMs are identical since depth rotates).
Twinkle × shell-fade form the existing 8×5 envelope multiply; no new
per-pixel multipliers. All frame set-up (quadratic Warp speed curve,
Lissajous parabola-sine, amplitude and half-dimension scaling, centre×k
line origins) runs through ONE shared 13×13 multiplier in a ~256-step
vblank sequencer (power-of-2 slot layout). Raster width/height are
measured cascade-style (avid run / active-line count). Pipeline
LATENCY = 17.

EBR: 12 kernel + 2 shimmer + 8 nebula + 1 exp2 ≈ 23/32. Six shells
(not starfield's 8) is the HX4K fit budget: 8 shells synthesized to 115%.

## Controls
- **K1 Layers** — 1..6 active zoom shells
- **K2 Density** — star density (in Luma Mode: the luma → density gain, 0..7.5×)
- **K3 Brightness** — output brightness (linear multiply)
- **K4 Palette** — 8 colour sets (Ron's spec 2026-10-07): Silver (white, silver, blue), Candle (white with yellow/orange), Twilight (blue/purple/pink), Neon (green and magenta), Ember (red/pink/orange), Honey (all yellow hues), Rainbow (R/G/B, ±45° shimmer), B&W. Entries carry a per-entry luma dim (×1, 7/8, 3/4, 1/2) for silver/gold/amber, applied to star pixels only.
- **K5 Size** — adds 0..3 glow levels (×3/4 scaled; dithered, smooth growth)
- **K6 Drift** — vanishing-point Lissajous amplitude (0 = locked centre)
- **T7 Direction** — Toward / Away
- **T8 Video** — Off / On: stars composited OVER the input picture (alpha 'over', not additive): alpha from the brightest star's kernel weight in 5 bands (>=160 opaque, 3/4, 1/2, 1/4, 0 for the glow edges), out = video + (star − video)·a by shift-adds, no clamps needed (convex). Deep Space = faint blue-violet cast on the video. Lively mood rides the Warp slider at >= 50%.
- **T9 Luma Mode** — video luma → star density, and a gentle size boost: luma adds up to one dithered glow level (~+40% at white, ~+20% mid grey; was +2 whole levels)
- **T10 Background** — Off / Deep Space (nebula + gradient)
- **T11 Gate** — Luma Mode: steeper luma-to-density map ((luma−256)×2); manual mode: thins the field to the two bold magnitude classes
- **Slider Warp** — flow speed, quadratic: ~30 s/cycle at 10 %, ~1 s at
  50 %, ~0.3 s hyperspace at full — the headline performance control

## Presets
- Cruise
- Hyperspace
- Star Drift
- Nebula Dive
- Retreat
- Tunnel Weave

## v1.1.0 (2026-10-06) — finalization pass
- `program_type` → `processing`: Luma Mode reads input luma, and `synthesis`
  routes no input (T9/T11 were dead on hardware).
- Blanking gate: neutral black (Y 64, U=V 512) outside the latency-aligned
  avid (accumulators freeze in blanking and repeated the last star).
- Saw-active vsync guard: per-field steps (phase, drift, twinkle, nebula,
  height capture) run once per field under serrated analog vsync.
- Levels: Y = 64 + 11/16·product capped at 768 (was 0-based, peaks 1019+
  killed star chroma on hardware).
- K2 Density live in Luma Mode as the gain; T11 Gate has a manual-mode
  meaning (bold stars only).
- Shell fade over 32 phase counts (16 = one field at full Warp → pops).
- Hash `× 0x9E37` in canonical signed-digit form (6 terms): −384 LUT4
  standalone, bit-identical hash. Net LUT4 4389 → 4100 standalone.
- Build with `NEXTPNR_ROUTER="--router router1"`.
- HW feedback 2026-10-07 (colours approved): big stars were sliced FLAT at
  cell edges — kernel reach 7 units vs offsets 2..13 in a 16-unit cell, and
  the neighbouring cell draws only its own star. Fix: offsets 4..11 and
  every kernel invisible beyond 4 units (window 2.5→4.0, sigmas scaled;
  level-3 gauss 2.8 → 1.8), spike offset clamp removed. Top glow level is
  ~65% of before, which also answers "largest stars too big"; Size knob
  scaled ×3/4 (at +4 levels everything sat at max level all cycle).
- Video overlay (T8, 2026-10-07, Ron's request): first built ADDITIVE (star luma
  added, chroma shifted) — Ron: 'blends with the video too much'. Rebuilt as an
  alpha OVER composite: 5a star colour, 5b under-layer + differences, 5c mix
  (f_mix shift-adds) + blanking gate; Brightness pre-scaled ×11/16 free-running
  so no 12-bit scaling adds in the pixel path; LATENCY 15 → 17 for all modes. Mood moved off the switch onto Warp.
  Video sample comes through a 16-deep EBR delay line (2 EBR): reading
  pipe(14).u/v un-pruned ~450 FFs of the FF pipe and overflowed to 7722 LC.
- Palettes: K4's upper four positions were the same four colour sets with
  Lively on, and Warm/Cool were one hue at three saturations. Now 8 distinct
  palettes on K4 (B&W is position 8), Lively moved to T8. Entries authored
  as sRGB hues → BT.601 → hardware U/V-swapped (red = U 960 / V 360), scaled
  to chroma magnitude 300–360; 512-entry shimmer ROM.

## Build (v1.1.0, 2026-10-07, router1, alpha-over video + luma boost)
All 6 configs seed 1, first try. HD Analog 80.5 MHz / 7595 LC, HD HDMI
78.2 / 7588, HD Dual 80.1 / 7611; SD Analog 75.8 / 7582, SD HDMI 74.0 /
7597 (SD Fmin is 27 MHz), SD Dual 82.8 / 7600. 24/32 EBR. **99% LC -- the
chip is full.** Any further feature must remove something first (candidates:
manual-mode Gate, palette luma dim, Deep Space video cast). No non-1 seeds.

## Status
**v1.1.0 FINAL except presets — Ron signed off 2026-10-07** ("I think we can
finalize Warpfield"). HW-validated across the session: palettes ("colors look
good"), cell-safe kernels ("looks really good now"), alpha-over Video. Presets
are hand-written; capture from hardware with `./capture_preset.sh`.
Identity fixed 2026-10-11 (Ron): `program_id` `com.lzxindustries.warpfield` →
`com.ron.warpfield`, `author` "ron", url → bluecondition/videomancer-dev
(TOML-only; vmprog repacked from the same bitstreams). Starfield got the
same cell-edge kernel fix in its v3.1.0 pass.

## Community port — fit for the stock SDK (v1.1.2, 2026-10-14)
- The stock LZX SDK builds 8 configs, and its SD Standalone core is ~600 LC
  larger than HD. v1.1.1 packed to 8190/7680 LC there (HD Dual 7902).
- v1.1.2: six bit-exact changes.
  1. **Serial multiplier.** The shared 13x13 multiplier is now serial (13
     shift-add steps, started by `s_mgo` when a step writes `s_ma`/`s_mb`). The
     vblank sequencer steps every 4th clock (256 steps = 1024 clocks, still long
     before the first active line). Every product is read exactly 4 steps after
     its operands are written, with no write in between.
  2. **Line origins in EBR.** `x0` (6 x 24 FF, feeding nothing but the hsync
     reload) lives in a 8 x 32 EBR. It is read back one shell per clock after
     each hsync.
  3. **Dropped no-op reload.** Tick 18's `s_mb <= k16(v_li)` reloaded the value
     tick 14 had just written.
  4. **Cheaper distance test.** `|sub - offset|` is one signed subtract plus a
     conditional negate (it was two subtracts and a mux).
  5. **Envelope multiply.** env is 0..16, and 16 passes w unchanged, so
     the rest is an 8x4 multiply.
  6. **Split star hash.** The hash input is (x cell, y cell), so
     `(hx*256 + hy) * 0x9E37 mod 2^16 = ((hx*0x37 + Y_hi) mod 256) & Y_lo` with
     `Y = hy * 0x9E37 mod 2^16`. Y is computed once per line per shell in the
     hsync walk, which leaves an 8-bit multiply-add per pixel.
- Proven bit-exact against v1.1.1: GHDL side-by-side, 40 frames (403 x 201
  raster), knobs and switches changing mid-picture, 0 mismatches in 3.24 M clocks.
- Packed LC (stock SDK): SD Standalone 8190 → 7502, HD Dual 7902 → 7219.
  EBR 24 → 26 (HD Dual 27).
- Tried and dropped (no gain): one shared write port for `sy_acc` (+90 LC: the
  per-shell adders already absorb the seed mux), and `sy_acc` in EBR with a
  per-line snapshot (-30 LC only: FF-only snapshot registers each take an LC).

## v1.1.3 (2026-10-17) — K1/K4 labelled defaults (community fix, folded in)
The firmware loads a labelled default as its label INDEX: Layers "5" loaded
as 4 (below 171 = 1 shell) and Palette "Twilight" as 2 (bits 9:7 = 0 =
Silver). K1 below 6 and K4 with bits 9:3 = 0 are now taken as the label.

## v1.1.4 (2026-10-17) — resolution scaling
- The shell coordinates run in virtual 1080-line units, stepped from the video
  timing ID (`registers_in(8)`). Two more ticks in each shell's sequencer slot
  run the shared serial multiplier: `ky = k * y step` (tick 22 -> 26, Q.4 per
  FIELD line: 1.0 / 1.5 / 2.0 (1080i) / 4.4375 (480i) / 3.75 (576i)) and
  `k16 = k * x step` (tick 26 -> 30, 1.0 / 1.5 / 2.0 per pixel at 1080 / 720p
  / SD). The width and height measurements (centre, drift) count the same
  units (`s_xcnt` Q.1, `s_lcnt` Q.4), and so does the nebula raster
  (`s_xcnt` replaces the duplicate `pixel_x`, `pix_ya` Q.4). At 1080p every
  step is exactly 1 (bit-identical); at 720p and SD the cells, stars and
  nebula keep their size relative to the screen height, stars stay round on
  4:3 and SD is a 1440-unit window of the 1080 canvas.
- Cost: six 15-bit `ky` registers + a 14th `k16` bit; standalone LUT4
  3694 -> 3733, DFF 2605 -> 2711 (incl. the v1.1.3 label fix).
- The width counter `s_xcnt` also replaces the duplicate `pixel_x` for the
  nebula, so it resets at hsync (not at avid fall): the nebula path runs ~7
  pixels early and samples the frozen end-of-line count at the right edge.
- Build (router1, `NEXTPNR_ROUTER="--router router1"`): **HD HDMI seed 2**
  (seed 1 = 70.66 MHz: the vblank sequencer's tick decode -> `s_mb` enable,
  10.6 of 14.2 ns routing; an earlier build of the same logic, before the
  `s_xcnt` reset moved, got 77.84 on seed 1 -- placement noise), all others
  seed 1: HD Analog 78.97 MHz / 7133 LC, HD HDMI 78.00 / 7126, HD Dual
  87.06 / 7125; SD Analog 83.81 / 7126, SD HDMI 86.94 / 7128, SD Dual
  84.35 / 7130. 26/32 EBR, 93% LC. (v1.1.2 in this SDK, same router/seed: 75.87 / 6979, 82.30 /
  6987, 78.78 / 6985; 79.11 / 6999, 79.59 / 6991, 84.09 / 7002 -- 91%; the
  chip is no longer full since the v1.1.2 serial multiplier.)
