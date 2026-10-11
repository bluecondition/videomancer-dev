# WAKE — progress log

Spec: Ron's WAKE prompt (motion-keyed effects + reusable `motion_key` module), phased 0–5, stop after each phase.
Location: `programs/ron/wake/` (repo rule; prompt said `programs/wake/`). program_id `com.ron.wake`.

## Phase 0 — recon (2026-10-10) — DONE, awaiting go for Phase 1

### Grid (cols from timing ID, cell edges from MEASURED active W/H)
Column count comes from the timing ID (SD 24, HD 32), rows fixed 18. Cell edges come from a Bresenham
counter over the measured active width/height (latched per update, restarted at each avid rise, so
analog ±1 px line jitter can't pile up). Reasons: the image tester feeds the real timing ID with a
decimated raster (default 1080p29.97 id 13 at 480×270), so a per-ID pixel LUT would put 8 columns on
screen in sim. CLAUDE.md also requires screen geometry from measured dims. Partial edge cells
disappear: Bresenham spreads the remainder, and cells differ by at most 1 px/line. The normalization
reciprocal = serial divide of the measured cell area, once per update in vblank.

| Timing IDs | Active per update | Grid | Cell (px × lines) | Updates/s |
|---|---|---|---|---|
| 1080p30/29.97 (7,13) | 1920×1080 | 32×18 | 60×60 | 30 / 29.97 |
| 1080p25 (11) | 1920×1080 | 32×18 | 60×60 | 25 |
| 1080p24/23.98 (3,9) | 1920×1080 | 32×18 | 60×60 | 24 / 23.98 |
| 1080i60/59.94/50 (10,2,1) | 1920×540 field | 32×18 | 60×30 field lines | 60 / 59.94 / 50 |
| 720p60/59.94/50 (14,6,5) | 1280×720 | 32×18 | 40×40 | 60 / 59.94 / 50 |
| NTSC (0) | 720×~243 field (measured) | 24×18 | 30×13–14 | 59.94 |
| PAL (8) | 720×288 field | 24×18 | 30×16 | 50 |
| 480p (4) | 720×480 | 24×18 | 30×26–27 | 59.94 |
| 576p (12) | 720×576 | 24×18 | 30×32 | 50 |
| sim default (id 13, dec 4) | 480×270 | 32×18 | 15×15 | — |

Display aspect is roughly square in every mode, including SD with its non-square pixels.
There are no 1080p60/50 IDs, so update rates span 23.98–60 Hz.

### BRAM / port map (engine = 10 EBR, all 1W1R inferred, no_rw_check)
- **Accumulators, 3 EBR (256×16, addr = col*4+ch, 96 used).** The per-line cell sums of 8-bit Y/U/V
  are pre-shifted per format so a column sum fits 16 bits. yosys can't infer read-modify-write on a
  single RAM (it becomes FFs), so banks A/B ping-pong by line parity: read A, write B. On the last
  line of a cell row the write goes to FIN instead. FIN is read by the finalizer during the next cell row.
- **Reference, 4 EBR (512×8 ×4, addr = cell*3+ch, 1728 used), 8-bit per channel.** Presence-mode small α
  uses LFSR stochastic rounding instead of fractional bits. That gives unbiased creep to 60 s, and the
  1-LSB jitter sits under FLOOR. Read port: finalizer, 3/cell; Cloak display, 6/boundary (Phase 4).
  Write port: finalizer only.
- **Motion grid M, 3 EBR (256×16, 576 used), 8.8 fixed point.** An 8-bit M stalls on long releases
  because the rounding step is bigger than the decay. Read port: display, 2 reads per column boundary
  (rows r and r+1, read one column ahead for the DDA step), with priority; the finalizer's M_old read
  waits for a free slot. Write port: finalizer only. In-place, single-buffered, tear accepted per spec.
- **WAKE coefficient: no ROM.** Release works in the log domain: e = f(WAKE) + rate_offset[timing ID],
  applied as M − (M >> e)·(1+mant) at cell rate.
- Budget headroom: SD finalizer ≥ 465 clk/cell (NTSC); sim default 255 clk/cell. Every boundary op
  (3 accumulator RMWs, 2 M reads) fits a 15-px cell.

### Corrections to the prompt
1. `C_PROCESSING_DELAY_CLKS` / "divisible by 4" is not an SDK or core rule. It is a howler-local
   constant and comment. The image tester regex-parses it as a LITERAL integer (or reads TOML
   `pipeline_delay`) for sim alignment. The real contract is one fixed latency in all modes, with
   syncs delayed to match. I'll keep it a literal multiple of 4 anyway.
2. The SD clock is 13.5 MHz for NTSC/PAL (858/864 clk/line), not 27. The builder builds all 6
   configs; HD 74.25 is the gate.
3. A grid from a timing-ID pixel LUT breaks in sim and on wandering analog lines → measured
   geometry (above).
4. The reference "~5 BRAM with fractional bits" doesn't work out: 60 s at 60 Hz needs α ≈ 1/3600
   (~12 frac bits, about 9 EBR). Stochastic rounding → 4 EBR.
5. The motion grid needs 3 EBR, not 2: 576 entries > 512 and fractional bits are needed. A 32×16
   grid would get it to 2.
6. Raw-video depth: with renderer L plus interpolator 4 plus I/O registers, the dry path lands at
   ~14–16 clk. That is the depth that went green on HD HDMI three times. Plan: keep L short, or
   carry the dry video in a 16-deep EBR ring (warpfield technique, +2 EBR).
7. WAKE draws its own colour (Heat, Track), so it needs the generated-avid hsync-parity fix
   (aurora lesson, ~25 LC) plus the blanking gate. The Heat palette is authored U/V-swapped.
   Quarter Turn's direction flips on HW (harmless).
8. Interlace: CLAUDE.md says refresh cell state on ONE field only. The zero-cost fallback for
   opposite-field false motion is one-field updates (half rate), before per-parity grids (+4 EBR).
9. Build/sim file collection: the build globs `<prog>/*.vhd` (GHDL resolves order; oubliette
   precedent), and the tester toposorts them. The golden testbench must live in `wake/tb/`, not the
   program root, or it gets synthesized.
10. Tester limits: still image (→ G_TEST_OBJECT), fake hblank 64 clk, fake vblank 8 lines
    (~4.3k clk at the default raster). Vblank work (reciprocal and centroid divides) must fit in it.

### Open questions for Ron
- STIR = 0 → nothing keyed, so every other control is dead at the bottom of the fader. That
  conflicts with the bold-always-on standing rule. Ron didn't object, so it's kept as specced (fade-in).

## Phase 1 — Python model (2026-10-10) — DONE, awaiting go for Phase 2
`model/wake_model.py` is the golden model: integer-only, with the formulas in its docstring. Run it
with `~/.venvs/wake/bin/python` (numpy + imageio-ffmpeg + pillow; no system ffmpeg on this box).
Clips are in `model/clips/` as input | Heat | effect strips, with stills and a golden NPZ (1080p30_matte.npz).

Mappings (these become the VHDL constants):
- FLOOR: f = K2>>5 (0..31). CHROMA: d = |dY| + (((|dU|+|dV|)*K3)>>9). Gain: M_int = min(255, (d−f)<<2).
- WAKE: e16_sec = (((K1>>3)*3)>>2) − 95 → τ = 1/61 s .. 1 s (6 octaves). Add RATE_OFF16[tid]
  (round(16·log2 Hz)), subtract 16 in one-field mode.
  Release dec = max(1, ((M>>e_int)*MANT_DN[e_frac])>>5), MANT_DN[i] = round(32·2^(−i/16)).
  The max(1) prevents stalls.
- STIR: e16 = ((1023−P12)*11)>>6; thr = min(1024, ((MANT_UP[e16&15] << (e16>>4))>>5) − 1),
  MANT_UP[i] = round(32·2^(i/16)). Bottom ~8% of the throw = off; centre thr 42; 60% → 19; top → 0.
- SOFTNESS: K4 < 128 → nearest cell, gain 256. Otherwise kq = 16 − ((K4−128)>>6) and
  gain = 2^(kq>>1)·(1.5 if kq odd).
- Heat: palette index = matte>>4 (motion strength); alpha = min(1023, raw_key<<2) over 50%-dimmed
  mono. It uses the UN-inverted key; Matte shows the final key.

Findings:
- Synthetic static noise: HD cell d ≤ 4 (p99 2), so FLOOR 15% (4) is just clean.
- NTSC comparing every field (previous-field compare): p99 8, max 11 from fine texture, and 1-line
  stripes hit 132. **One-field updates bring it to max 4** at no BRAM cost (29.97 Hz SD updates).
  The model defaults to one-field on interlaced formats; `--both-fields` restores the spec.
- CHROMA proven: the same-luma patch gives d = 2 / 21 / 40 at CHROMA 0 / 512 / 1023.
- Small-object limit: objects < ~1 cell (60 px @1080) register only when they cross cell edges.
  A 24 px square is missed on 66% of updates, 48 px on 13%, 96 px on 0%. WAKE bridges the gaps.
  Possible later fix: per-cell first moments via double-integrator accumulators (no multiply),
  +2 ref EBR.
- Pan → ~40% screen keyed; cut → full-screen flash for ~0.5 s. Fade at ≤1.7 LSB/frame stays under
  FLOOR. All of these are expected without STEADY.
- Keyed after the ball stops (default STIR/SOFTNESS): WAKE 40% → 0.27 s, 60% → ~0.75 s,
  88% + soft → 1.9 s, 100% → ~4 s.

Suggested defaults: as specced, except **WAKE 60%** (40% gives barely visible 0.27 s trails).

## Phase 2 — hardware PoC (2026-10-10) — v0.2.0 built, awaiting bench (seeds: SUMMARY.md)
Files: `motion_key.vhd` (motion_key_pkg + entity in ONE file: the image tester sorts by entity
instantiation, then alphabetically, so a separate `motion_key_pkg.vhd` analysed after its user),
`wake.vhd` (host), `wake.toml`, `tb/` (golden testbench).
- **Build: ROUTER1 required** (`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=1200`). Router2
  missed seeds 1–3 and stalled on seed 4 at 88% LC. Router1 closed all 6 configs on seed 1:
  HD Analog 86.0, HD HDMI 83.8, HD Dual 82.7, SD 78–86 MHz. ~6,800/7,680 LC (88.5%), 9/32 EBR.
- Engine alone (standalone yosys): 2,499 LUT4 + 685 carry + ~1,730 FF (est. ~2.8k LC packed),
  9 EBR. Rest of WAKE: ~1,520 LUT (3 interpolators, Heat ROM, effect muxes, video pipe,
  generated avid). The SDK core takes ~2.5k LC.
- Latency 12 (not a multiple of 4: padding would only carry raw video deeper). Render L = 6.
- Golden TB (`tb/run_golden.sh`, ~3.5 min, NTSC one-field): bit-exact on cur, M and energy
  over 9 updates at WAKE 614 and 528 (528 hits mantissa 32).
- Bugs found after the first build: (1) `shift_left(g_c, 1)` on 6-bit g_c = 32 wrapped to 0,
  so the HD renderer never advanced hc (full-width band; SD 24 → 48 fit, so the SD golden TB
  missed it); (2) `to_unsigned(32, 5)` for MANT_DN(0) (release collapsed whenever e16 % 16 = 0).
  Both fixed. The TB now runs without `--ieee-asserts=disable`.
- Image tester (G_TEST_OBJECT scratch copy, 1080p dec 4): Heat shows a localized two-lobed
  blob tracking the square; Negative + KEY Invert = negative everywhere except around the motion.
- Yosys finding: pipelined RMW (registered read → fabric reg → modify → separate write
  process) infers EBR. CLAUDE.md rule corrected.
- LC is the Phase 3/4 risk: Track + Cloak + Presence + Steady won't fit at 88%. Plan an
  optimization pass first: Heat ROM → EBR, share one serial multiplier between geometry and
  finalizer, recip = recip_c·recip_r >> (16−sh) (drops the W·H multiply and wide divide),
  sequential per-channel finalizer (fewer holding regs), 8-bit interpolator t.

### Bench 1 (2026-10-10, Ron): FLOOR
- Static source at FLOOR 0: "perlin-noise blue blobs" everywhere. They clear at ~37% (f = 11).
  Synthetic noise cleared at 4, so the real cell noise peaks ~3x higher. WAKE peak-hold makes
  the floor track noise PEAKS.
- v0.2.1 (TOML only, same bitstreams): FLOOR default 15% → 40% (First Look too).
- Open: which noise? Candidates: chroma noise (CHROMA 1x doubles it); global flicker/AGC
  (STEADY's g-compensation would remove it); W/H measurement jitter shifting cell edges
  (W comes from the LAST update line, with no hysteresis; an SD row-edge shift of 1 line =
  up to contrast/13.5 ≈ 10+ steps). Fix candidate regardless: latch W on a fixed early line
  and add ±2 hysteresis on W/H, as chantilly does.

### Bench 2 (2026-10-10): frozen computer image over HDMI still blobbed
- At FLOOR 0 the screen was mostly blue blobs; they cleared only at 72% (d ≈ 23), clustered on
  edges, with CHROMA 0 too. Cause: the computer sends 1080i (no 1080p60 timing exists), and
  core_top ties field_n = '1' on every HD input, so one-field mode compared opposite fields
  (interlace twitter on every edge). Reproduced in the image tester (1080i5994) once field_n
  was tied high in a scratch copy.
- v0.2.2 fixes:
  - HD interlace = timing ID OR the SDK video_field_detector OR a field_n that really toggles.
    Without a live field_n, a per-field toggle on the guarded vsync edge picks every other
    field (same-parity compares).
  - The update decision is latched once per field.
  - Generation-tagged seeding: re-seed on load, timing-ID change or any geometry change;
    cleared only when an update seeded all rows under the current generation. The old
    2-count warm-up could be eaten by a partly-qualified field (H = 1), leaving an unseeded
    reference that saturated the grid.
  - W measured on update line 2, ±2 hysteresis on W/H. MGRID write deferred while the
    renderer reads (same-cycle EBR read was undefined). fsm_encoding "none" on all three
    FSMs. FLOOR default back to 20%.
- Golden TB: SD and HD-1080i (`GOLDEN_FMT=hd tb/run_golden.sh ~/.cache/wake_golden_hd`) both
  bit-exact. Image tester 1080i clean at FLOOR 0 with field_n toggling and with it tied high.

### Bench 3 (2026-10-10, v0.2.2): clean at FLOOR 17% (f = 5)
Default 20% (f = 6) sits just above it, so no change. If that was the FROZEN image, a residual
d ≈ 5 is a bit more than rounding flips alone (≤ 3 with CHROMA 1x): worth confirming which source.

## Footage tuning (2026-10-10) — v3 engine proposed (model only so far)
Ron found the bench hard to judge, so I tuned on real footage instead: Tears of Steel 720p
(CC-BY, ~/.cache/wake_footage) gave 7 clips from still to action, cached as cell sums
(model/tune/tune.py). Metrics vs per-pixel ground-truth motion: recall = real motion keyed,
false = keyed with no motion within 1 s plus 1 cell.
- **v0.2 keys almost nothing on real footage**: recall 0–24% at defaults (gestures 6%). Moving
  cells change their mean by only 1–3 LSB (median), which is within integer rounding and grain.
- **v3 engine** (model `MotionKey3`):
  - 2×2 luma sub-cells (64×36, reusing the renderer's half-cell grid), with chroma per cell;
  - means at 1/4 LSB;
  - reference ref += (cur − ref) >> 2 at 1/16 LSB, so it doesn't stall;
  - d = (Σ4|ΔY_sub|)>>1 + ((|ΔU|+|ΔV|)·CHROMA)>>9 in 1/4 LSB;
  - dp = min(255, (d − FLOOR>>4)·GAIN3>>2), GAIN3 = 4;
  - new STIR curve: e16 = ((1023−S)·17)>>7, thr = min(1024, (MANT_UP<<i)>>3), giving
    8 octaves, thr 4 at the top and bottom ~6% off.
  - Separability at a fixed false rate: room walk 5% → 62%, seated 2% → 75%, trio 28% → 86%.
- Defaults (FLOOR 25% = f4 16, STIR 60%): recall city 5, seated 25, room 15, talk 69, trio 66,
  action 86, walk 80; false ≈ 0. STIR 100%: recall 25–95%, false ≤ 1.1%.
  Recall now rises across 6–100% of the fader.
- Tours in model/clips/tour_{compare,stir,wake,softness,floor,effects}.mp4.
- HW cost estimate (not built): ref ~12 EBR (3456 values × 12 bit), acc banks unchanged
  (192 addr/bank), finalizer ~130 clk/cell (fits the sim vblank barely), est. +300 LC. LC is
  92% now, so this needs the optimization pass.

## Phase 3 (2026-10-11) — v0.3.0: v3 engine + DETAIL + STEADY + Solarize/Crush/Smear + presets 1–7
Ron: "the videos don't look great, but I'm curious what this will look like when used to key
incoming video", so Phase 3 includes the v3 engine, with DETAIL as the centrepiece.
- **v3 engine in RTL** (motion_key.vhd, bit-exact with model MotionKey3):
  - luma 2×2 sub-cells per cell (half-column/half-row Bresenham), U/V per cell;
  - 1/4-LSB means; 12-bit reference with ref += (cur−ref)/4;
  - a sequential finalizer, 6 values per cell (~170 clk/cell);
  - REF RAM 4096×12 (12 EBR); 17 EBR total.
- **DETAIL**: edge gate on the SHAPED key (min), not on the matte as specced. Gating the matte
  only bit in the top ~5% of the knob, because the matte is far above the key threshold in
  moving areas. Edge energy = |Y[x]−Y[x−2]|<<3, peak-held rightward −64/px;
  gate = clamp(e + 1023 − DETAIL). 50% = half effect away from edges; 100% = the effect
  clings to moving edges/texture ("chrome emboss" on the movers). Render L 6 → 7.
- **STEADY**: dc = max(0, d − g_prev), g_prev = ((Σd>>4)·recipN)>>14. Cut guard: running Σd
  > 224·C·(row+1) at any row ≥ 2. Then the remaining rows re-seed at once, the next update
  re-seeds fully (generation-tagged), and the key is muted until the seed completes.
  Footage: cuts ≥ 236, action p99 180 (one false trip in 1600 updates).
- **Mixer**: 3 SDK interpolators replaced by a 3-stage 11×7 mixer (t7 = (t+8)>>4, exact b at
  1023); ~700 LUT saved, which is what made v3 + Phase 3 fit. Program latency stays 12.
- Effects: Solarize (Λ fold), Crush (3-bit Y / 2-bit chroma, neutral kept), Smear
  (sample-and-hold, W>>7 px).
- STIR v3 curve; FLOOR = K2>>4 quarter-steps, default 25%.
- Golden TB: SD, HD-1080i and SD+STEADY-with-cut all bit-exact. Bug found: the update-field
  decision lagged one pixel at line 0, leaking a stale partial sum into sub-cell (0,0); v2
  hid it in integer rounding.
- Resources: 4,259 LUT4 / 2,548 FF / 17 EBR (yosys); build ~7,170 LC (93.7%).
- Timing (router1): HD Analog 75.6 (s1), HD HDMI 75.1 (s5), HD Dual 78.3 (s1), SD 73–83.
  HD margin is thin (~1–2%).
- Footage preset tour: model/clips/keyed_presets.mp4 (input | WAKE output per preset).
- Phase 4 will NOT fit as-is (93.7%): needs logic removal or feature trade-offs, so ask Ron.
