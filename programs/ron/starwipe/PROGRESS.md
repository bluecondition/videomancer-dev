# STARWIPE — progress

Brief: switcher wipe/pattern section (A wipes to generated B; P12 = T-bar), 38 CLASSIC + 30 CHEESE
patterns, analytic distance fields, ≥1-px AA everywhere, clean ends under any setting.

- Checkpoint 1 (2026-10-17): per-pixel log-polar design — correct but 127 % LC. Archived as
  `tools/old/starwipe_v010_perpixel.vhd` (+ `tools/old/` v1 tools).
- Checkpoint 2 (2026-10-18): the **scanline engine** (v0.2) — fits (89 % LC, 27 EBR), RTL-sim-validated,
  18 patterns, HD timing closure in progress.

## Architecture (v0.2)

A pattern is a per-field table of **rays** from an anchor; between rays the field is linear (n·p + c) or
a circle q-form (g(ex·u² + ey·v²) + c). Along a scanline the field is quadratic, so the pixel path is two
quadratic **accumulators** (A, B; union = min/max) + key = 128 − D>>8 + border key + compose.

- **Engine** (`p_ueng`): 32-bit microcoded, one register window (4 banks), serial Booth MUL (result shift
  = empty Booth steps), 48-step DIV, 24-step SQRT, quarter-sine ROM; shares ONE register set
  (mu_a/mu_ph/mu_pl) for all serial units. Program `tools/sw_ucode2.py` → `starwipe_pkg.vhd`
  (1263 / 1280 words). Runs during active lines (paused while the line builder owns DATA); double-buffered
  record sets (set 0 @256, set 1 @576); per-field ports are 15 WORDS in DATA (896 + 16·set) loaded by the
  builder at the anchor (LBS_PORTS) — no shadow registers.
- **Line builder** (`p_lb`): steps every record of the active list once per line (adds only), emits
  (x, D(x), s) breakpoints into a 256×64 table (bank = line parity, 64 per accumulator). ~46 clocks per
  record per line: HD fine; SD overruns above ~18 records/line (falls back to the stale bank for that
  line — documented limit, SUNBURST-class).
- **Pixel path**: fetchers share the table port; combine (one comparator); key; border; compose with
  9-bit weight (256 = far endpoint exactly); A delayed in a 32×32 EBR ring; co-sited chroma; generated
  window at a fixed hsync offset; per-line U/V swap by hsync parity; neutral gate; latency 20 all modes.
- **REVERSE** = sweep u = 1−T AND key inversion (MODE bit 3 → 128 + D): A stays at the bottom.

## Verification

- `tools/sw_run2.py` renders through the emulator + integer model; `tools/sw_ends2.py` = ends grid
  (76 032 checks: every pattern × 11 anchors × 3 modifiers × spin × 4 edges × reverse × 4 T-bar ends,
  480i + 1080p). First full run: 37 004 failures → all traced to three bugs, fixed 2026-10-18:
  REVERSE without key inversion; circle dD/ddD from finite differences of truncated values (drifts when
  the line step is not 1.0 px, i.e. SD); ellipse bound in Euclidean instead of ellipse metric. Second run
  (`ends4.txt`): 4 170 failures, the visible class being CLOCK/STAR/CIRCLE under REVERSE at the hard end
  (k4 = 1023) — the border clearances h0/h1 were not swapped with the sweep direction. Fixed (ROM 1263
  words); third run in `ends5.txt`.
- **RTL image sim** (`vhdl-image-tester`, 1080p dec 4 = 480×270, CIRCLE mid-travel, 30 warmup frames for
  the T-bar glide): matches the emulator render structurally (`.starwipe_work/cmp_circle.png`;
  `tools/sw_simcmp.py`). Border hue differs only by the sim's BT.601-vs-hardware U/V convention. Two RTL
  bugs found on the way and fixed: DATA reads captured one clock early in LBS_RD (2-clock EBR latency),
  LBS_MULW prep condition re-firing at the 12th step (infinite loop for every q-form record).
- Sim gotcha: the tester globs every `*.vhd` in the program dir (dotfiles too) — keep archives in `tools/old/`.
  GHDL needs `LIBRARY_PATH=.starwipe_work/lib` (libz.so link).

## Fit / timing log (HD HDMI, single-config bare make = router1, seed 1)

| step | LC | Fmax |
|---|---|---|
| v0.2 first full synth | 7627 (99 %) | router timeout |
| − shadow ports, compose endpoints, FF field ring, shared serial regs | 7100 est. | — |
| − engine mux trees (one tree per register, unified step forms) | 6831 (88 %) | 52.7 |
| LB ceil chain registered; MULW split | 6796 | 66.7 |
| LB ENTRY flags registered | 6868 | 60.7 / 64.0 |
| IDLE next-line pre-registered; Booth adder decoupled from decode; +128 fold; address descriptors | 6855 | 67.5 |
| registered op-class + cycle flags; result value by class, u_wd/u_wa unconditional | 6900 | 66.6 |
| NON-restoring DIV/SQRT (no wide mux after the carry chain) | 6960 (90 %) | **74.81 PASS** |

Also fixed on the way (STAR WIPE RTL sim, `cmp_star_n7.png`): DATA-port arbitration — the engine resumed in the
cycle the builder released the port (an LDD read the builder's word) and a registered STD pulse could be
dropped in the builder's first busy cycle; now the engine resumes one cycle late and its pending write has
priority.

Full 6-config builds: `build_v020c.log` (builder default router2) — sd_analog closed, hd_analog missed on
seeds 1–5 and stalled on 6 (last post-route 71.5 MHz), build aborted; the single-config pass above was a
bare make, i.e. router1. `dupin.py` finds no self-add carry cells, so the engine-program rule applies:
`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron starwipe` → `build_v021_r1.log`: **6/6 on seed 1**,
packed `out/rev_b/ron/starwipe.vmprog` (v0.2.0, ROM 1263 words), all bitstreams iceunpack OK.

| config | Fmax (router1, seed 1) | LC |
|---|---|---|
| hd_analog | 75.20 | 6953 |
| hd_hdmi | 74.81 | 6960 |
| hd_dual | 77.80 | 6973 |
| sd_analog / sd_hdmi / sd_dual | 70.70 / 78.45 / 76.89 (SD clock) | 6954 / 6986 / 6951 |

**Build this program with router1 only.** HD margins are 0.5–3.5 MHz; a config that misses in a future
build should be retried alone (`make ... CONFIG=<cfg> SEED=1`) before any restructuring.

Ablation (standalone yosys): engine EX datapath ≈ 1900 LUT before the mux restructuring (now ≈ 1450);
three 11×10 compose multipliers ≈ 490 LUT (candidate: share U/V, +1 latency); LB multipliers cheap.

## First hardware pass (2026-10-18, v0.2.0 flashed)

Ron: the wipes look good and work well; two defects: hysteresis on some knobs (MODIFIER named), and
places on STAR WIPE where the lines don't meet.

- Hysteresis: the pattern slot (K1) and the point/side count (K2) carried a designed ±8-count
  hysteresis (1.6 % of travel); reduced to ±2 (pot jitter only). Nothing else has hysteresis; the T-bar
  has a 2-count deadband + glide.
- Lines not meeting — found by a pixel-exact RTL-vs-emulator diff (the earlier compare was fooled by
  the border hue and by a frame-count mismatch: the RTL capture after 8 warmup frames corresponds to
  emulator run **5**, not 7 — the first frames measure the raster). Two RTL defects:
  1. **Late loads in the fetcher.** A line-table entry needs ~5 clocks to fetch after the previous
     one loads, so an entry within 5 px of its predecessor loads late with D evaluated at the wrong x:
     the anchor row (every ray crosses at the anchor x) showed a ~10 px notch, and whenever a ray's
     crossing leaves the screen at x = 0 its entry piles onto the START entry and the edge jumps 5–7 px
     (the edge "not meeting" where it reaches the left/right edge). Fix: the line builder holds one
     PENDING entry and writes it only when the next entry lies ≥ 5 px right of it (bunched entries
     collapse to the last, which is the piece that applies anyway); `sw_scan.build_line` does the same
     (`MIN_GAP`). The first entry of a line also loaded 3 px late (fetch started at the window start):
     the fetch is now primed `C_PREF` = 6 clocks before the window (g_pre).
  2. **Window 2 px early.** The compose pipeline was shortened during the fit work but G_K stayed at 6:
     the key field sat 2 px left of A. G_K = 8 (an even move: hsync→avid parity, hence hardware colour
     pairing, unchanged). The image tester aligns by the declared delay, so it cannot see this class;
     only the model diff can.
- **Engine pause hazard** (found with a GHDL VCD of the line builder + DATA port, `.starwipe_work/vcd_*.py`):
  from the second run on, the engine is paused by the builder (`lb_busy`), and a pause landing in ST_RDA
  handed operand A the value of operand B — `rf_ra` had already moved to srcB and `rf_q` is refreshed
  every clock. Record 4's x came out as record 2's x on every row of frame 4 (deterministic, not a
  port collision: the dump showed no engine write while the builder was busy). Fix: `rf_ra` stays on
  srcA while RDA is held. Rule: every value the engine consumes AFTER a wait must be held or re-read
  during the wait (the ROM, the sine ROM and the DATA read already were; the register file was not).
  The first run of each boot is never paused (p_ready = 0 keeps the builder idle), which is why MIX and
  the first STAR frame looked right.
- Entry-spacing rule, three iterations against the ends grid: (1) "drop the pending, keep the
  newcomer" → 18 277 end failures (a newcomer 2 px past the last written entry loaded late);
  (2) "first-eligible" (keep pending, drop close newcomers) → SUNBURST still failed: on row 538 it dropped
  the LAST entry of a bunch (956, 958, 960, 961, 962, 965 → 956, 961), whose piece spans the rest of the
  line, so the 961 piece ran 900 px and crossed into B. (3) **Final: shift-and-replace** — a newcomer's
  x is raised to (last COMMITTED x + 5) and its D/s evaluated there (LBS_SHIFT, one extra builder cycle
  per record), and the newcomer always takes over: the next slot when ≥ 5 px past the pending entry
  (which is thereby committed), else the pending slot (pending dropped). No long-span piece is ever lost,
  nothing loads late, and the only error is the committed piece extending over a bunch of narrow ones.
  Slot-in-place writes: only the pending x and the committed x are held (no 64-bit pending word).
  One more case from the SUNBURST ends (12 px at columns 0..3 on four rows): a ray crossing at x = 2
  replaced the x = 0 START entry, leaving pixels 0..1 without an entry — before any commit a newcomer at
  1..4 is raised to 5 instead, so slot 0 always holds x = 0.
  Same rule in `sw_scan.build_line` (`MIN_GAP`); RTL ≡ model re-verified (`sim_star_540p_v5.png`).
- **Dead zones forced exact.** With any entry spacing, a committed piece extended over a bunch of
  dropped narrow entries can un-saturate within the extension (PINWHEEL/SUNBURST bunches reach 15–30 px
  against a 6 px bound margin): `ends7.txt` 1 600 failures, all of that class. The T-bar mapping already
  clamps T to 0 at p12 ≤ 20 and to ≥ 65533 at p12 ≥ 1004, so the frame program now forces MIX mode with
  TK = 0 / 256 there (10 ROM words, 1274/1280): the ends are bit-exact A / B by construction, and the
  bunch approximation only ever shows mid-travel as a few-pixel blob at the anchor on the one or two
  rows where crossings coincide. **Ends grid `ends8.txt`: 76 032 checks, 0 failures** (480i + 1080p).
- **Second hardware pass (2026-10-19, v0.2.1 flashed): wipes better, but the incoming video's hue was
  swapped over the whole picture (blue→red, yellow→cyan), no control changed it.** Not the window move
  (sim measured the output hsync→avid offset = the source's 32, parity unchanged; a window-6 A/B was built
  as the control). Cause: the offset latch `m_off` started at 200 and only updated when the measured
  offset differed by ≥ 2 — Ron's HDMI source sits within 1 of 200, so the latch never loaded, every line's
  parity read as "different" and the per-line U/V swap fired on every line. Invisible in sim (stimulus
  offset 32). Fix: `m_offv` — the first field latches `m_off` / `m_w` unconditionally. HW-confirmed
  (`starwipe_latch.vmprog`, HD HDMI seed 1, 75.8 MHz). v0.2.0 presumably had it too and it went unnoticed.
- **Final v0.2.1 build (2026-10-20, router1, `build_v021g_r1.log`)** → `out/rev_b/ron/starwipe.vmprog`,
  all six iceunpack OK:

  | config | seed | Fmax | LC |
  |---|---|---|---|
  | hd_analog | 3 | 77.89 | 7102 |
  | hd_hdmi | 1 | 75.82 | 7110 |
  | hd_dual | 2 | 74.42 | 7121 |
  | sd_analog / sd_hdmi / sd_dual | 1 | 72.47 / 73.39 / 74.13 (SD clock) | ~7110 |

  HD Dual's margin is 0.17 MHz; router1 is deterministic per seed, so the bitstream is what was timed.
- HD HDMI single-config (router1, seed 1) on the final RTL before the latch fix: **78.41 MHz PASS**, 7106 LC (92 %), 27 EBR.
  Earlier variants spread 67–77 MHz across seeds; the pre-registered LBS_ACC selects and the
  registered spacing flags (`lb_fnew`, `lb_wx5`) are what brought seed 1 back.
- **Validation** (`sim_star_540p_v4.png` vs `emu_540p_star_n5.npy`, `.starwipe_work/cmp_star_v2.py`): every
  measured edge at offset 0, big-diff fraction 3.4 % → 0.2 %, and what remains is the test picture's own
  swatch chroma edges (tester 4:2:2 conversion vs mine), column 0 and the last row. RTL ≡ model.
- Timing: the merge logic exposed a 13 ns path inside LBS_ACC (lb_acc → line-vs-anchor compare → list
  descriptor mux → state); the two compares are now pre-registered in p_gen (r_sel*, r_sel0*).
- Ends grid after the REVERSE fix (`ends5.txt`): 76 032 checks, 210 failures, all 1080p at EDGE fully
  soft (k4 = 0) on CIRCLE / DOUBLE SWEEP / PINWHEEL / SPOTLIGHT — the per-frame coverage bound is
  short for the 30 %-FH ramp on those shapes. Open.

## v0.2.2 (2026-10-20) — A black floor for downstream keying

Ron reported v0.2.1 good on hardware (committed f8ce53e). Lars's suggestion: clamp the incoming
video's black at one or two 8-bit codes above black so the generated BLACK B (and BLACK border) is
the only Y 64 in the picture — a downstream mixer can then luma-key the wipe shape cleanly.
Implemented as an always-on clamp of A's luma to >= 72 (10-bit) in `p_ring` on the EBR -> `ra_q2`
re-register hop (a 10-bit compare + mux where there was no logic; chroma and syncs untouched);
`sw_scan.compose` mirrors it (`A_BLACK_FLOOR`). Codes 64..71 of A are crushed to 72 — invisible.
The lerp across the AA edge still passes through 64..72, so a downstream threshold near 68 is sharp
at the edge width, which is inherent.

Build (router1, `.starwipe_work/build_v022_r1.log`), all six post-route PASS, iceunpack OK:

  | config | seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | hd_analog | 2 | 75.49 | 7126 |
  | hd_hdmi | 2 | 77.46 | 7096 |
  | hd_dual | 2 | 74.40 | 7139 |
  | sd_analog / sd_hdmi / sd_dual | 1 | 71.29 / 68.06 / 77.48 (SD clock) | ~7120 |

## Patterns built (18)

CLASSIC: MIX, BAR >, BAR v, DIAGONAL, BARN DOOR <> / ^v / diag, BOX, DIAMOND, CIRCLE, POLYGON, CLOCK,
PINWHEEL, DOUBLE FAN, DOUBLE SWEEP. CHEESE: STAR WIPE, SUNBURST, SPOTLIGHT. EDGE soft/hard/border;
B = black / matte hue / white; border WHITE / hue / BLACK / GOLD (solid); ORIGIN, MODIFIER, SPIN, REVERSE.
Not yet: unions (FLOWER, HEART, KEYHOLE, ROUND RECT, EYE), CROSS, TRIANGLE, BOW TIE, VEE, X DOORS,
STARBURST, APERTURE; REPEAT, AUTO; B sources past WHITE; border specials past GOLD; per-line hooks.
Preview catalogue of all 68: `.starwipe_work/preview/` + `starwipe_menu.html`.

## Deviations to confirm
- WHITE (B and border) is Y 765, not 940: full-scale luma clips on this hardware (house rule).
- MATTE / border hue wheel is a YUV chroma circle (amplitude 242, luma 393 ± 166), in gamut for every hue.
- Pattern knob carries only the built patterns.
