# CASCADE DIAG — working fork of Hypnos Cascade

Cascade has been hard to change safely, so changes land here first, under a
separate program_id (`com.bluecondition.cascade_diag`, "Cascade Diag"). It
installs alongside the shipped Cascade for A/B on hardware. Merge back into
`programs/ron/cascade/` only once it's HW-confirmed. The full history,
architecture notes and sim traps live in `programs/ron/cascade/SUMMARY.md`.

Deliverable: `out/rev_b/ron/_diag/cascade_diag_v1.7.vmprog`

## v1.7.0 — Tier 1 finalization (2026-10-07), not yet flashed

No intended change to the look:

- **Ring luma is studio range, 64..942.** v1.6 drove 0..1023, and codes 0
  and 1023 are the BT.656/1120 sync-word codes. The mapping is
  `64 + 4k − k/2 − k/16`, monotonic, computed in the existing output stage,
  so latency stays 6.
- **The blanking gate covers both modes.** Rings mode used to emit the last
  pixel's key (often white) through every hblank. Only Video mode was gated.
- **The dead multi-ring smooth fade is removed.** This was the old S11 Smooth:
  `ph_acc`, the Q decode and multiply in sequencer slots 5/4/1, and the
  E_LINE seed in slots 36–40. Its slots are kept as no-ops, so the per-line
  engine timing is unchanged.
- **K5/K6 ("Spare") are removed from the TOML**, along with their preset
  entries. Stale header comments are fixed.
- **Not done: padding the delay-line word to 32 bits.** Tie-off pad bits get
  pruned (they would have to be live bits), and Video mode is already
  HW-confirmed on this exact 16+14 mapping.

### Build: router1, all 6 configs on seed 1

| Config | Fmax | margin | LC |
|---|---|---|---|
| HD Analog | 84.34 MHz | +13.6 % | 7423 |
| HD HDMI | 83.17 MHz | +12.0 % | 7418 |
| HD Dual | 83.02 MHz | +11.8 % | 7412 |
| SD Analog | 80.94 MHz | — | 7405 |
| SD HDMI | 84.15 MHz | — | 7393 |
| SD Dual | 79.06 MHz | — | 7414 |

That's 96.6 % LC, 6 EBR. **SUPERSEDED: this router1 build showed GREEN
video on HD HDMI** (see HW rounds below). Don't use router1 for this
program. The shipped build is router2 with HD HDMI on seed 2.

### HW checklist

1. Rings mode looks identical to v1.6: same black and white, no edge or
   line changes.
2. Video mode: passthrough at Wave 0, and the left edge fades in cleanly
   (v1.6's fix was never checked on hardware).
3. All five presets load, and K5/K6 no longer appear on the unit.

### HW round 1 (2026-10-07): Video mode came in GREEN on the router1 build

The Video-path RTL is byte-identical to the HW-confirmed v1.6. The only
Video-mode differences are the placement/routing and the new program
install. A/B packs are in `out/rev_b/ron/_diag/`, all installing as
"Cascade Diag":

| Pack | Contents | Tests |
|---|---|---|
| `cascade_diag_v1.7_router1` | v1.7 RTL, router1, seed 1 (the build that showed green) | — |
| `cascade_diag_A_v16bits` | the shipped v1.6 bitstreams + diag TOML | install vs bitstream |
| `cascade_diag_B_v17_router2` | v1.7 RTL, router2 | router1 placement vs v1.7 RTL |

Pack B timing: HD Analog 80.92 (seed 2), HD HDMI 78.41 (+5.6 %),
HD Dual 76.39 (+2.9 %), SD 77.47 / 74.47 / 71.93 (seed 1). The bins for
all three packs are saved in `.cascade_diag_work/{v16bits,r1_v17,r2_v17}/`.

### HW round 2 (2026-10-08): Pack B (router2) better, still a faint green tinge

- **The 32-bit pad is a no-op.** yosys logs `removing const-0 lane 30/31`
  and maps the memory back to the 16+14 split. Inferno v3.7's identical
  `"00"` pad would have been stripped the same way, so its HW-confirmed
  "fix" was most likely just a new placement. The CLAUDE.md BRAM
  "pad to power-of-2" evidence is suspect. The pad was reverted, and the
  diag VHDL is back to Pack B's source.
- **Placement A/B packs**: Pack B's bitstreams with only HD HDMI re-placed
  (router2):
  `cascade_diag_C_hdhdmi_seed2` (79.07 MHz), `_seed3` (77.10),
  `_seed4` (76.58). All pass post-route. The bins are in
  `.cascade_diag_work/hdhdmi_s{2,3,4}/`.

### HW round 3 (2026-10-08): HD HDMI seed 2 is CLEAN in Video mode

Ron confirmed `cascade_diag_C_hdhdmi_seed2` looks right with video. That is
now the diag deliverable, `out/rev_b/ron/_diag/cascade_diag_v1.7.vmprog`,
with its bitstream set saved in `.cascade_diag_work/release_v17/`.

**v1.7 seeds (router2):** HD Analog **seed 2** (80.92), HD HDMI **seed 2**
(79.07, HW-clean; seed 1 at 78.41 showed a green tinge, and router1 seed 1
showed strong green), HD Dual seed 1 (76.39), SD seed 1 for all three.
**A plain `./build_programs.sh ron cascade_diag` lands HD HDMI on seed 1 —
the green layout.** Rebuild with `SEED=2`, or hand-install HD HDMI seed 2 and
repack, then A/B Video mode on hardware before shipping any rebuild.

Lesson: HD HDMI Video-mode green on this program is a PLACEMENT lottery.
Timing passes on every layout, so only a hardware A/B separates good from
bad. router1's far better Fmax gave the WORST layout.

## v1.8.0 — Tier 2 round 1: P12 Stir (2026-10-08), awaiting HW

- **P12 Stir** makes the centre ORBIT its K1/K2 point. It's a Minsky circle:
  `ox −= oy>>6; oy += ox>>6`, stepped once per field behind an avid-armed
  guard (analog vsync serrates). One turn takes ~6.7 s (402 fields at 59.94).
  Offset = `(ox, oy) · p12 >> C_ORS` on the shared multiplier: radius 0..~511 px
  in HD and ~255 in SD (slider < 8 = still). The offset is added at the publish
  (slots 2/1), after the glide, so the orbit isn't low-passed. P12 is eased
  (diff>>3 per field). The orbit uses only free sequencer slots (10, 5, 4, 3).
- **Wave depth is continuous**, riding the same slider:
  `du = L·key/64` with L = the eased P12's top 3 bits as three gated terms,
  giving 8 levels from 0 to 25 px (was 4 zones up to 18). The 32-tap line
  holds it, and the left-edge clamp still applies.
- **First cut was 7629 LC (99.3 %)** with an 8-slot sequencer extension and a
  p12×H radius multiply; HD Dual came in at +0.4 %. **Slimmed** to fixed
  HD/SD radius shifts in the existing free slots, with the products read at
  the publish: **7545–7572 LC (98.5 %)**, +140 over v1.7.
- Known: K4 steps / S9 Smooth show the mid-drag tangent streaks while
  orbiting (never settled). Classic hard rings are exact.

**Seeds (router2):** HD Analog **7** (78.79), HD Dual **7** (78.67), SD 1/1/1
(75.52 / 74.54 / 77.85). HD HDMI A/B candidates: **seed 1** (78.54) and
**seed 3** (77.55). Both packs are in `out/rev_b/ron/_diag/cascade_diag_v1.8_hdmi_seed{1,3}.vmprog`.
Sweep: HD Analog s3 77.2, s4 77.7, s5/6/8 FAIL; HD Dual s2 76.1, s3 76.7,
s4 FAIL, s5 76.9, s6 74.4; HD HDMI s2 75.3, s4 FAIL. Bins are in
`.cascade_diag_work/v18_sweep/` + `v18_builder/`.

### v1.8 REJECTED (2026-10-09) — reverted to v1.7

Ron: "I don't like the stir option, can we go back". The diag source is v1.7
again, regenerated from the main `programs/ron/cascade` v1.7 files with only
the diag identity swapped in. The v1.8 source is archived in
`.cascade_diag_work/v18_rejected/`. **Don't re-offer the P12 orbit/Stir.**
The continuous-Wave change was bundled with it and went too.

## v1.8.0 (2026-10-09) — K1/K2 range, P12 Zoom (Rings), bigger Wave (Video); awaiting HW

Ron's direction: **no automated animation** in this program (Catch-Up is the
only exception). Manual control of the rings is its strength.

- **K1/K2 range is ±0.625 W/H** (was ±2 W/H), so the centre can go at most
  ~12 % past each edge and never gets lost. The change is a `1.25·W` operand
  inline in sequencer slots 20/16 plus `v_ct >> 2`.
- **P12 in Rings mode = ZOOM.** Up to 3 octaves (`P12·3/16`, Q6) come off K3's
  log-spacing in new slot 21, with a clamp at Lf −704 (spacing ≤ 2048). The
  rings expand up to ×8 from the centre. It's off in Video mode. There's no
  ease, the same as K3.
- **P12 in Video mode = wave depth**, 8 levels (`L·key/32`, three gated
  terms) up to 53 px (was 4 zones up to 18). The delay line goes from 32 to 64
  taps on the same 2 EBR, and the left-edge clamp counter is now 6-bit.
- **Exact band-WIDTH was built first and didn't fit.** It needed ~7775 LC in
  the real build (slimmed to ~+290 LUT over v1.7). Ablations: Width engine +
  ease ~250 LUT, K4/S9 gradients ~200, K range ~25 (an in-place knob prescale
  cost 73!), Video ~30. Ron chose Zoom over dropping the gradients. The
  shelved source is in `.cascade_diag_work/v18_width_shelved/`; it's exact
  (nest/outer-field colour model 0 mismatches over 231k cases).

**Build: 7503–7538 LC (98 %), 6 EBR, router2.** Seeds: HD Analog 1 (79.71),
HD Dual **5** (78.03; s1 74.90 too thin), SD 1/1/1 (70.89 / 76.40 / 68.74).
The HD HDMI A/B packs are `out/rev_b/ron/_diag/cascade_diag_v1.8z_hdmi_seed{2,3,5}.vmprog`
(76.59 / 77.11 / 78.49; seed 1 missed, seed 4 failed, seed 6 at 74.81 too
thin). Bins are in `.cascade_diag_work/v18z_builder/`, `v18z_sweep/` and
`v18z_pack_hdmi_s*/`.

**HW (2026-10-09): v1.8 HD HDMI seed 2 "works well" (Ron).** It's saved as
`out/rev_b/ron/_diag/cascade_diag_v1.8.vmprog`, with its bins in
`.cascade_diag_work/release_v18/` (HD Analog s1, HD HDMI s2, HD Dual s5,
SD s1). Follow-up ask: glide a little quicker, which became v1.8.1.

### Left-edge gradient during catch-up (2026-10-09) — known limitation, no fix

Ron: with K4/S9 gradient on, bands at the LEFT edge "come to a point" while
the rings are catching up; they line up once settled. Hard rings are fine.
SD sims reproduce nothing when rings are concentric (Uniform and settled
Cascade, centre in the middle or at the corner, K4 0 and max). It's the
known catch-up artifact of the per-ring ramp: inside a band the shading is
measured from the OUTER centre and only re-anchors at visible edges, so the
edge band, whose left boundary is off-screen, is the worst. The tried fix
(v1.8.2: seed each line from the previous line's error at its first edge,
~60 LC) did NOT help in a before/after sim (`.cascade_diag_work/sim/cmp_s.png`)
and was reverted. The source is in `.cascade_diag_work/cascade_diag_v182_seedfix_rejected.vhd`.
A real fix needs a per-line distance to the edge band's own circle (a second
sqrt in E_LINE, ~120–150 LC), which doesn't fit at 98 %.

## v1.8.1 (2026-10-09) — Glide 2× quicker (C_GSH 6 → 5); awaiting HW

Seeds (router2): HD Analog **8** (77.64), HD Dual 1 (80.83), SD 1. HD HDMI
A/B packs are `out/rev_b/ron/_diag/cascade_diag_v1.8.1_hdmi_seed{11,9,7}.vmprog`
(79.76 / 77.38 / 76.66). Seeds 1–6 were all under +2.7 % or failed; seeds 8
and 10 failed; seed 12 got 76.64. Bins are in `.cascade_diag_work/v181_*`.

## v1.9.0 (2026-10-09) — lean pass: multiplier merge + Width (K5) + left-edge fix + K4 trim

Ron asked for a deep optimization pass. He wants the clean look, cascading,
catch-up and some gradients kept, and allowed Zoom/Video to go and S9 to be
freed.

**Measured profile** (standalone synth, LUT deltas from stub ablations on v1.8.1):

| Subsystem | Cost |
|---|---|
| Vblank shared multiplier (14×14 LUT array) | **~491 LUT** |
| Engine multiplier (15×15 LUT array) | **~572 LUT** |
| Video mode (delay line, 60 hold FFs, mux) | ~140 LUT + 201 FF |
| K4 7 zones + S9 Smooth (vs 4 settings) | ~53 LUT |
| Zoom | ~38 LUT |
| K3 `×144` constant multiply | ~21 LUT |

**What was done:**
1. **One multiplier instead of two.** The vblank sequencer and frame tail now
   copy their operands into the engine multiplier while `mul_sel` is high. The
   two users are time-disjoint, and the sequencer and tail stall if the
   engine isn't idle. Product latency goes T−3 → T−4, so the read slots
   moved 17/13/8 → 16/12/7 and the tail phase 4 → 5. This saved
   **−495 LUT / −69 FF** and is **bit-exact**: SD sims at two scenes are
   0 pixels different from v1.8.1. A full build of just this change gave
   7014 LC (91.3 %).
2. **K3 decode `×144/256` is now `(k + 8k) >> 4`** (−21 LUT).
3. **K4 has 4 settings** by knob quarter: hard, 2 steps, 8 steps, smooth.
   S9 Smooth is folded into the top setting and S9 is unused.
4. **WIDTH on K5** (the shelved exact version, re-timed onto the T−4
   multiplier: operands at slot 13, |δ| read at slot 9). The slack decode
   moved to 6..2, so the chain starts at `s_seq = 1`. It's eased, deadbanded
   at the centre, and forced off in Video mode. 720p sims: K5 150 = thin white
   lines, 880 = thin black lines, exact circles.
5. **LEFT-EDGE FIX.** E_SPAN phase 8 tracks the nearest disc edge ≤ 0
   (`xl_w`, published as `xl_done`). E_LINE (now 61 clocks) keeps `dy'²` in
   `le_r0sq` after the first root, adds `(x_L − outer cx)²` at slot 40, takes a
   second root at slots 41–55, and seeds `ph1` with `(r0 − r_L)·Q1` at 56–59,
   the same rule every on-screen band follows from its own left edge. The
   old outer-centre seed is still used in Uniform mode or when x=0 is outside
   the nest. The concentric regression matches the old seed to ≤ 1 radial px
   (max 13/255 levels).

**Line budget is now 15·(nreff+1) + 61:** HD 1966/2200, SD 796/858. SD SIMS
NOW STARVE at `--video-mode ntsc --decimation 1` (784 clk line), so sim the HD
config at 720p60 dec 1 instead.

**Size:** 4896 LUT / 1665 carry / 1922 FF standalone, vs v1.8.1's
4951 / 1491 / 1863. That's Width and the left-edge fix *plus* a slightly
smaller design.

**In reserve:** serialize the remaining engine multiplier (~450 LC; it
re-times E_SPAN and SD needs ~44 rings). Smaller trims: fold the two 27-bit
r² adders into one, and read the video delay line one deeper to drop r14.

### v1.9.0 real build: 98.5 % and a timing wall

Real LC was 7518–7593. The +174 carry cells packed badly, so the standalone
LUT count understated it. HD Analog passed only on seed 6 at 74.60, and a
13-seed sweep under machine load failed every one. Test pack (HD HDMI s3,
78.75): `out/rev_b/ron/_diag/cascade_diag_v1.9.0_test.vmprog`. Ron chose the
serial multiplier to get lean.

## v2.0.0 (2026-10-09) — serial multiplier (Phase 2)

The engine's 15×15 LUT multiplier is replaced by a **shift-add serial
multiplier**: a signed 15-bit multiplicand times an UNSIGNED 14-bit
multiplier, one 17-bit add per clock, 14 steps, product readable at T+15.
Every user has a non-negative multiplier operand; squares use |x|; Q1 is
h1s·fnum.
- **E_SPAN pipeline:** phase 4 stages ring k−2's edges, roots ring k−1's
  product and multiplies ring k (all pre-edge reads). Marks are at phase 5,
  sp_rr steps at phase 7, and there are two drain groups → 15·(nreff+2).
- **E_LINE (89 clocks):** the dy'² load is at 1; r0² and r0's root at 16/17
  run in parallel with dxL²; at 32 come r0 and the tracker seeds, r_L², and
  r0·fnum; r_L's root is at 33; k·s at 49; corrections at 66–71; the seed
  product at 72; ph1 at 87.
- **Vblank sequencer (slots 21..4):** one product at a time; it stalls on
  `mu_busy`/`s_mreq` and reads in the next slot. The chain starts at
  `s_seq = 4`.
- **Line budget** 15·(nreff+2)+89: HD 2009/2200, SD 839/858 (48 rings kept).
- **Standalone size:** 4536 LUT / 1672 carry / 1887 FF (v1.9 was
  4896 / 1665 / 1922).
- **Bit-exact vs v1.9:** 0 differing pixels in four 720p scenes (concentric
  smooth, catch-up, Width 150, Width 880), run on a COPY of the image tester
  with a 280-clk hblank (`.cascade_diag_work/vit_hb`, run with the SDK venv
  python). E_LINE no longer fits the stock tool's fake 64-clk hblank, so on
  the stock tool everything past the first line renders white. That's a sim
  artifact only.

### v2.0.0 timing closure (2026-10-09)

The first full build failed HD Analog on every seed (~68–72 MHz). The
critical paths were all control cones, and fixing them got it to 93 % LC:
1. **The nearest-edge tracker was compare→enable in one clock.** It's split
   now: phase 6 registers the candidate, phase 7 compares and updates. The
   E_SPAN exit moved there too.
2. **Sequencer step-enable:** the avid / engine-idle / mu_busy / request cone
   fanned out to every slot register. It's now a registered `seq_go`, and
   request slots hold it low through `v_req`.
3. **Multiplier load mux:** operand arithmetic and the `e_cnt<nreff` gating
   sat in front of `mu_*`. There's now a next-operand register stage
   (`nx_m/nx_q/nx_ld`), so a request at T loads at T+1 and the product is at
   T+16. E_SPAN requests at phase 4, stages/roots at 5, marks at 6, and
   nearest-edge/exit/radius at 7. E_LINE is 93 clocks (HD 2013/2200,
   SD 843/858).
4. **Per-group flags:** `e_cnt` vs `s_nreff` compares are registered at
   phase 0 (f_lt/f_mid/f_ge2/f_last), and the E_CLEAR exit compare is
   registered (`cl_hit`).

**Bit-exact vs v1.9** after (3) in 4 scenes, and after (4) in the catch-up
scene, all on the 280-hblank tool.

**Result:** 7143–7150 LC HD (93 %), 7122–7130 SD. Seeds (router2):
HD Analog **10** (76.21, +2.6 %; only s1 75.32 also passes in 1–12),
HD Dual **8** (81.47; 7 of 12 pass), HD HDMI **3** (78.52) / **6** (76.37);
seed 1 stalled, 2 and 5 stalled, 4 and 7 fail. SD Analog **4** (s1 stalled),
SD HDMI 1, SD Dual 1.
**Router2 stalls a lot on this netlist** (`overused=2` forever), which points
to a congestion hot spot. TRAP: a stalled route's log only holds the PRE-ROUTE
"Max frequency" line. Trust a seed only if its `.asc` exists; `route.sh`
now checks. TRAP 2: the packer validates and packs with missing configs, so
three incomplete packs were made and deleted. Count six bins before packing.

Test packs (all 6 configs): `out/rev_b/ron/_diag/cascade_diag_v2.0_hdmi_seed{3,6}.vmprog`.
Bins are in `.cascade_diag_work/v20_pack_hdmi_s{3,6}/`.
Next timing step if needed: split E_LINE's ring-index correction into
2-clock steps and give E_LINE its own copy of s_spac. That's HD Analog's
recurring path.

## v2.0.1 (2026-10-09) — E_LINE correction split, chain/strobe timing, S9 green DIAG

- **HW:** v2.0 HD HDMI seeds 3 AND 6 both showed BRIGHT GREEN video. The
  video-path RTL is identical to v1.8.1 (HW-clean on seed 2), so it's
  placement-dependent again.
- **S9 "Video Test" (DIAG):** in Video mode, S9 = Raw outputs a 1-clock
  passthrough that bypasses the s_vd delay-line EBR (gated at its input,
  ~5 px shift). Raw clean / Ring Wave green = EBR read path; green either
  way = HDMI in/out side.
- **The E_LINE ring-index correction is 2 clocks per step** (candidates, then
  select), using its own `s_spl` copy of the spacing (keep). E_LINE is now 99
  clocks: HD 2019/2200, SD 849/858. Bit-exact vs v1.9 (catch-up scene).
- **Chain FSM:** a registered `ch_blank` (not avid) and `ch_go4` (slot-4
  strobe) replace the raw core avid / `s_seq` compare in its enable cones.
- **Build** (7169–7217 LC, 94 %), router2:

| Config | Passing seeds | Best |
|---|---|---|
| HD Analog | 1, 2, 4, 6, 8 | 4 (80.06) |
| HD Dual | 1, 5, 7, 9 | 9 (79.75) |
| HD HDMI | 9 only (1–9) | 9 (75.63); seeds 10–17 sweeping |
| SD | 1 / 1 / 1 | |

  HD HDMI's recurring paths are the Width per-frame regs (`s_dabs`/`s_seff`/
  `s_spac`, which feed sequencer + E_SPAN + p_par far apart), plus the chain
  and the tracker. Next fix: per-consumer copies of those registers.
- Test pack: `out/rev_b/ron/_diag/cascade_diag_v2.0.1_hdmi_seed9.vmprog`.

## v2.0.2 (2026-10-09) — per-consumer register copies; ROUTER1 closes everything

- **Per-consumer `keep` copies** of the per-frame Width/spacing registers:
  `s_dabs_e/p`, `s_dneg_e/p`, `s_seff_p`, `s_spac_e`, refreshed at
  sequencer slots 9/14. Router2 HD HDMI went from 1/17 to 3/8 passing seeds,
  all marginal. LC is 7240–7279 (94.5 %).
- **ROUTER1 passes every seed on every config:** HD HDMI 1–8 at
  74.8–87.6 MHz (s8 87.64, s6 86.79, s3 82.73), HD Analog s1 84.83, HD Dual
  s3 80.62, SD 1/1/1 (72–83). As with cubist, router1 routes this serial-
  multiplier netlist far better than router2. **Build v2.x with router1.**
  The old "never router1" note was really about the HD HDMI green, which is a
  layout lottery under either router.
- HD HDMI A/B packs (with the S9 green diagnostic):
  `out/rev_b/ron/_diag/cascade_diag_v2.0.2_hdmi_seed{8,6,3}.vmprog`.
  Bins are in `.cascade_diag_work/v202_pack_hdmi_s*/`.

**HW (2026-10-09): v2.0.2 HD HDMI seed 3 is CLEAN (Ron: "seems fine")**,
saved in `.cascade_diag_work/v202_pack_hdmi_s3/`. Follow-up: K4's second
setting (2 steps = pure black/white bands) looked too much like the hard
one. v2.0.3 makes it 4 steps (black / 2 grays / white). Building with
router1, with HD HDMI A/B to follow.
v2.0.3 build (router1, all seed 1): HD Analog 83.15, HD Dual 79.13, HD HDMI
88.01 (seeds 2–6 also pass, 78–85), SD 78–81 MHz; 7230–7280 LC (95 %).
A/B packs: `out/rev_b/ron/_diag/cascade_diag_v2.0.3_hdmi_seed{1,5,3}.vmprog`.

## v2.1.x (2026-10-09) — S9 MOIRÉ (Ron picked option C over Checker / Two-pebble)

S9 XORs the rings with a **half-phase copy of the rings around the outer
centre**: `key ^= s9 & (outer parity ⊕ (r mod s ≥ s/2))`. At rest the nest
parity equals the outer parity, so the result is exactly doubled rings
(floor(2r/s) mod 2). While the cascade has the nest offset you get true
per-pixel moiré arcs, which dissolve as Catch-Up settles.
Lessons from three passes, all caught by MEASURING the sim, not eyeballing it:
1. XOR with just `(r mod s ≥ s/2)` gives the SAME rings shifted by s/2, not
   doubled (k ⊕ h toggles only at half radii). The reference must be the
   half-phase RINGS, `k_o ⊕ h`. Band-width run lengths along a row showed it;
   by eye it "looked doubled".
2. That XOR left ~72 isolated 1-px dots per frame at **Pythagorean points**:
   pixels at an exact integer multiple of s from the centre (e.g. (28,45) for
   s=53). The nest counts dist² ≤ R² as inside, but the tracker's floor(√d²)
   puts them in the next band. Fix: the tracker works on **d² − 1** (frame
   tail stores dx0'² − 1; the dy'² recovery is unaffected), which matches the
   nest's convention. 0 isolated px at rest and in drag.
3. A mark-forced/scanline-state variant also killed the dots, but it made
   horizontal hold-then-snap streaks during drags. Rejected: stay per-pixel.
K4 setting 2 is 4 steps (v2.0.3). The S9 green diagnostic was removed.
v2.1.1 build (router1, seed 1 everywhere): HD Analog 84.08, HD Dual 87.64,
HD HDMI 81.08 (seeds 2–6 also pass, 79.8–82.6), SD 86–87 MHz; 7241–7296 LC (95 %).
A/B packs: `out/rev_b/ron/_diag/cascade_diag_v2.1.1_hdmi_seed{2,6,1}.vmprog`.

**HW (2026-10-10):** v2.1.1 HD HDMI **seed 2** has clean video. With Moiré on,
a 1-px horizontal line ran from the centre circle to the right. That was the
d² − 1 tracker convention: on the centre row every pixel sits at an integer
distance, so the whole row shifts by a pixel. Ron then asked for **option A
(Checker)** instead.

## v2.2.0 — S9 CHECKER

The rings XOR a fixed square grid at the ring spacing, anchored top-left: a
px counter per line (`gx`/`gxb`) and a line counter per frame (`gy`/`gyb`,
half the lines when interlaced), both in p_geo and aligned with s_kx → g2_k,
`r10_h = g2_g & s9`. The Moiré pieces (h1/h2, s_shalf_p) and the d² − 1
change are reverted. Moiré source: `.cascade_diag_work/cascade_diag_v211_moire.vhd`.
v2.2.0 build (router1): HD Analog s1 80.28, HD Dual s1 80.65, HD HDMI s1 74.40
(too thin) / s2 85.90 / s3 84.65 / s4 80.78 / s5 84.88 / s6 87.27, SD s1 72–84;
7306–7345 LC (95.6 %). The sims show the checker as designed; along the axes through the
ring centre the rings align with the grid and cancel into a cross (inherent).
No centre-row line. A/B packs: `out/rev_b/ron/_diag/cascade_diag_v2.2.0_hdmi_seed{6,2,5}.vmprog`.

**HW (2026-10-10):** Ron didn't like the Checker and went back to the Moiré.

## v2.1.2 — S9 Moiré, centre-row line fixed

This is v2.1.1 without the d² − 1 tracker hack. The XOR reference
`k_o ⊕ h` = round(r/s) mod 2 only changes at HALF radii, because its two terms
flip together at full radii, so it never meets the true ring edges. The
Pythagorean-point dots came from v2.1.0's use of `h` alone, not from the final
formula, so the hack was never needed, and it caused the centre-row line
(every centre-row pixel is at an integer distance). Sim: 0 isolated pixels,
0 one-px row lines, doubled rings (corner cascade + centred uniform scenes).
Checker source: `.cascade_diag_work/cascade_diag_v220_checker.vhd`.
v2.1.2 build (router1): HD Analog s1 76.58, HD Dual s1 83.54, HD HDMI s1 73.82 FAIL /
s2 81.50 / s3 79.79 / s4 81.91 / s5 88.07 / s6 82.30, SD s1 76.9–82.3; 7222–7271 LC (94.6 %).
A/B packs: `out/rev_b/ron/_diag/cascade_diag_v2.1.2_hdmi_seed{5,6,2}.vmprog`.

**FINALIZED 2026-10-10:** v2.1.2 (HD HDMI seed 5, HW-clean) was merged into
`programs/ron/cascade` with HW-captured defaults. The main vmprog is packed from
`.cascade_diag_work/v212_pack_hdmi_s5`. Rebuilding from the main source with router1
reproduces the bitstreams bit-for-bit.
