# Cusp — Heirloom #17, rebuilt from the photo

**v0.6.0 (2026-11-01, phase 2): the black-core and orange rows became separate modules (own cusp
rate, height, drift, pointing); K4 also scrolls the stripes and swings the rainbow; K6 Bend reaches
~1 cycle and curls the stripes; S9 Rainbow cores. Built, NOT flashed.** See
[Phase 2](#phase-2-v060-2026-11-01).

**v0.5.0 (2026-10-27): built, NOT flashed.** Rebuilt from `photos/WIN_20230315_00_30_57_Pro.jpg` after
Ron judged v0.1.0 too abstract. It now redraws the photo: rows of fine green/navy/blue stripes around
black cores, cut by right-pointing arrowheads with blue X tips; cyan "fish" that grow from a needle to fat
bullets left to right; broad rainbow bands (cyan, purple, white-pink, magenta) running up to the seams;
orange/yellow rows with nested cyan arcs and a red X after every reset; the left sync strip. Work files:
`.heirloom_work/rb_cusp/` (`notes.txt`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **Rows.** Period 520.0 lines (2.08 rows per frame); mirror axes at y = 259.5 (black-core row) and 519
  (orange row). Hard seams at y = 90, 443, 609, 963: black-core rows (A zone) 353 lines, orange rows
  (B zone) 166 lines, 7 lines off the mirror symmetry.
- **Two layers, one key.** Every pixel is either the L layer (the broad rainbow of the left strip) or the
  S layer (the fine stripes / black core). Both are functions of the vertical position only:
  - L: red 74 (A centre) → 248 (B centre); blue saturated in the A rows, falling to 27 at the B centre;
    green folded 2.9 times over the half period (peaks 36, 146, 234 lines from the A centre).
  - S: black below 82 lines from the A centre, then green folding every ~28–30 lines; blue 20 + 1.26 per
    line (saturating in the B rows); red 0 in the A rows, 40–70 in the B rows.
- **The key.** A horizontal oscillator (saw) against a threshold:
  - The oscillator period per row is 456–486 px in the A rows (486 at the row centre) and 392–410 px in
    the B rows (392 at the centre). It chirps across the frame (A 482 → 464 px, B 389 → 402 px). Its phase
    follows 1 − cos of the vertical position (~92 px deep): the pointed ">" chevrons at the resets (sharp
    tips from an extra |d| term).
  - The threshold is a horizontal ramp minus that vertical cosine, onset x ≈ 138 + 1450 (1 − cos) px,
    clipped between a floor of 0.24–0.30 (rising toward the seams) and a ceiling of ~0.78.
  - A rows: L where phase < threshold, starting at each reset. B rows: the mirrored threshold, so the L
    share of a cycle shrinks while the A rows' grows: A 0.38 / 0.76 / 0.77 (needle at x ≈ 795, bullet tip
    at 1452, fat bullet to the edge), B 0.24 / 0.24 / 0.45 / 0.78 / 0.76.
  - The left sync strip (the oscillator held) ends at x = 206 (far from the row centre) to 253 (at it).
- **Edges.** At every key edge the key is low-passed per colour channel before the channel's clip / fold:
  roughly red 5, green 11–15, blue 2–4 px. That makes the green rims, the blue X at each black tip and the
  nested arcs. After each reset the L layer's vertical gain rings: A +0.62 e^(−x/22) cos(2πx/51 − 0.34),
  B −0.76 e^(−x/29) cos(2πx/45 − 0.94). That makes the X's, red in the orange rows.
- **No tearing** (mirror-row shift test: 0/1 px).
- Fit: edge least squares, then image-rms coordinate descent on spline tables (`opt5..opt10.py`). The
  integer model (hardware hues, output filters) is 30.3 RGB rms from the photo at Bend 0; 4×4 region means
  within 7 levels; local luma contrast 28.0 (photo) vs 28.9.

## What it does
1. **Per line:**
   - Line phase q (virtual lines × Rows); zone A/B from q; distance from the zone centre.
   - Nine piecewise-linear tables (knots every 1/64 of the row period):
     - phase start, scaled by Point, plus the drift;
     - phase increment, scaled by Cusps × kx;
     - threshold floor, ramp start and ceiling, all scaled by Point;
     - sync-strip width, scaled by Point;
     - the S layer's red, green and blue pre-signals (green × Stripes; B rows −2 fold cycles).
2. **Per pixel:**
   - Key and threshold:
     - phase DDA (second order, the chirp); floor and ramp DDAs;
     - u = frac(phase + source × Bend), Flip reverses it;
     - threshold = max(floor, min(ceiling, ramp));
     - key: A rows L = u < threshold (or inside the sync strip); B rows L = u ≤ 1 − threshold.
   - L layer:
     - virtual distance since the last S → L edge → ringing gain → L layer index;
     - L ROM: red, green fold argument, blue, red tint; red tint × (u − 0.35).
   - Mix and colour:
     - three soft keys (two-pole low-pass per channel);
     - s = S + K (L − S);
     - red and blue clipped, green folded by a triangle (dark navy below zero);
     - palette swap; RGB → YUV.

## Controls
v0.6.0 (phase 2). The defaults are v0.5's photo; the model is bit-exact with v0.5 at the defaults
(flat video, t = 0).

| Control | Photo element it drives |
|---|---|
| K1 Cusps | cusps across the **black-core rows only**: key-oscillator rate ×0.3 … ×2.9 (1× at the centre). The orange rows keep the photo's rate, so the two row types detune and their bands stop lining up |
| K2 Split | **orange-row height**: 0 = no orange rows (every row a black-core row) … 167 of 520 lines at the centre (the photo) … 430 lines at 100 % (the black-core rows a sliver). Row count fixed at the photo's 2.08 |
| K3 Fish | how fast the fish grow across the frame: threshold ramp 0 … 1× (centre) … 4× (fat fish from the first cusp; 0 = thin bands only) |
| K4 Speed | bipolar, still in the centre ±16, square law; default ≈ 0.5 px/field. The black-core rows stream right (left), **the orange rows the other way at the same rate**; **the fine stripes flow two periods into the cores and back out** (rate ∝ Speed²: ~16 s in and out at the default, under a second at the maximum); **the rainbow's green fold swings** by a triangle LFO (rate and depth ∝ Speed², up to ~0.93 cycle; ±0.05 at the default). All from one drift accumulator |
| K5 Stripes | density of the fine stripes ×0.4 … ×2.5 |
| K6 Bend | how far the source (camera luma / Osc) bends the cusps: 0 … 30 % (the default, as v0.5) … ~1.1 cycles at full-scale luma. **Above 30 % the key also curls the fine stripes**: the S green fold slides with the key phase (0 … 2 cycles across a cycle) outside the cores, so the stripes form chevrons around the black arrowheads |
| S7 Flip | Right / Left: the key ramp runs backwards, the arrowheads point left |
| S8 Rows | Alternate (the photo) / **Opposed**: the orange rows' key is flipped, so they point left against the black-core rows |
| S9 Core | Black (the photo) / **Rainbow**: the core lines' S layer reads table 14, a yellow → orange → magenta → pink glow sampled from the L layer, so the arrowhead cores light up |
| S10 Palette | Photo / Swap: red and blue exchanged, an amber / orange version |
| S11 Source | Video (default) / Osc (now ±full range, ~1.9 cycles across, 2 down, so Bend is clearly live) |
| P12 Point | the vertical modulation (chevron depth, needle and fish shape, strip edge): 0 % = flat vertical bars; 47 % = the photo; 100 % = 2.4× deep needle spikes |

Glides: K1, K2, K3, K5, K6 and P12 move 1/8 of the gap per field (deadband 4).

v0.5.0 had K1 = both row types, K2 = row period ×0.35 … ×2.8, K3 0 … 2×, K6 0 … ~0.3 cycle,
S8 = Alternate / Same, S9 = Glow Soft / Hard.

## Video
- **How the camera enters:** camera luma is added to the key oscillator's phase, so the arrowheads, fish
  and bands bend around the picture. At the default Bend the composition still reads clearly (GHDL render
  fed the selfie collage in `cmp_final.png`).
- **Osc mode:** two slow drifting triangles (~0.9 cycle across, ~1 cycle down, ±0.6 of the camera range)
  stand in for the camera on the same path; Bend stays live.
- **What it never does:** the video tap is luma into geometry only; it never reaches the output.

## Engine
- **Per field (vblank zone, one serial multiplier, 15 slots):**
  - glides and knob curves (two segments about the centre);
  - Speed² × 349 (drift), Cusps × kx, (Cusps × kx)² → chirp per zone;
  - ramp slope × Fish per zone, line step (Rows), top-line phase;
  - floor / ramp slopes × kx are per-timing-class constants; drift accumulator.
- **Per line (kicked at g_start, ~280 clocks, fits SD's 858; line 0 in the vblank zone):**
  - |q|, zone, distance;
  - nine table ops, each: two knots from a 1024×16 knot ROM, 8-bit interpolation and a control scaling
    on a 28-bit serial multiplier;
  - the DDAs load the results at g_start; the per-line constants are committed there too.
- **Pixel path (32 stages):**
  - 32-bit phase DDA with chirp, 28-bit floor / ramp DDAs, 8×6 bend product, 16-bit threshold compares;
  - key, Flip, virtual distance counter, env ROM → 10×8 L index product → L ROM (256×48) plus a fabric
    copy, 8×7 red tint product;
  - three two-pole soft keys (one rate per channel: red τ ≈ 5.9, green 12.3, blue 2.2 px at HD; SD and
    720p one shift faster);
  - 7-bit K × (L − S) as gated shift-add pairs;
  - clip / fold, swap, RGB → YUV (2-term constants, luma pre-boosted 16/13, summed directly);
  - output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777); blanking gate on the
    generated avid.
- **Memory:** 11 EBR.

  | Block | EBR |
  |---|---|
  | knot ROM | 4 |
  | L ROM | 3 |
  | env ROM | 1 |
  | sync delay ring (32 clocks) | 1 |
  | chroma box delay rings (8 clocks, U and V) | 2 |

  The rings carry `no_rw_check`; the yosys log has 0 `emulate_transparency`.
- **Contracts:**
  - heirloom v0.2.0 shell: virtual 1080-line coordinates, generated avid, vblank zone, measured width;
  - one 32-clock latency in every mode (the syncs go through the block-RAM ring, exactly 32);
  - colours in the hardware convention (U = Cr, V = Cb).
- **Fit history:**
  - The first RTL (a direct port of the integer model) came to 9589 LC (124 %).
  - Standalone yosys ablations: soft-key alternatives 1194 LUT, K × (L − S) multipliers ~1000, other
    pixel multipliers ~920, line engine ~640.
  - Fixes: one rate per soft key, narrower products, blue tint dropped, line engine kicked at g_start
    with a 28-bit multiplier, slopes as class constants, delay lines in block-RAM rings.
  - Result: 7204–7224 LC.
  - A shared-post-adder rewrite of the line engine was larger and was reverted.

## Phase 2: v0.6.0 (2026-11-01)
Ron had no notes on v0.5.0 (never flashed). He picked from a proposed control map (2026-10-31):
K1 / K2 were global X / Y scales, K6 Bend did almost nothing in Osc mode, and the whole pattern
drifted as one block. His picks: "Cusps + Split", all four motions (rows slide apart, stripes scroll,
rainbow cycles, key bends stripes), S8 Opposed + S9 Core, and wider Fish / Bend / Osc.

What changed (every new term is per field or per line except the stripe curl and the rainbow swing):
- **Two row modules.** The B (orange) rows load the line engine's phase-increment multiply with
  `f_incsb` = kx / 16 (the photo rate) and a per-class constant chirp; only the A (black-core) rows
  take K1.
- **One drift accumulator drives all the motion.** `f_dr` (33 bits) integrates ±Speed² × 349 per field
  as in v0.5:
  - the A rows' phase origin is PM0A + bits 31..0;
  - the B rows' is PM0B + not(bits 31..0), so they run the other way at the same rate;
  - the stripe scroll is a triangle of bits 31..22;
  - the rainbow LFO is a triangle of bits 32..24.
  A first cut with separate origin adders, a half-rate B drift and separate scroll / LFO accumulators
  packed to 7525 LC (98 %; HD HDMI needed seed 4 for 74.29 MHz).
- **Split.** The zone test is now `(P + hh − C_BC) mod 2^24 < 2 hh`, with hh = K2's B half height
  (slot 2) and C_BC = the photo's B centre (267 lines from the A centre). It takes its own line-engine
  state, so the table-op states moved up by one. At the centre it reproduces v0.5's [183.5, 350.5).
  The B tables simply extend flat past their fitted range, and the A tables already did.
- **Stripe scroll.** The line engine adds 0 … 511 to the S green fold argument wherever that argument
  is ≥ 0, so the core edge stays put while the stripes flow two periods into the cores and back out.
  The rate is ≈ Speed² / 12000 per field: ~1 per field at the default (16 s in and out), ~20 at the
  maximum. n_sg / l_sg are 14 bits now (Stripes ×2.5 + scroll reaches 4589).
  - It is a ping-pong rather than a wrap because the soft-key blends mix S and L green before the fold,
    so a wrapping offset makes every rim and arc lose a ring at once (`p6/scrollwrap.png`). The
    ping-pong grows the orange rows' arcs smoothly from ~2 to ~4–5 rings at the far end
    (`p6/pingpong.png`).
- **Rainbow swing.** The LFO triangle times Speed² / 1024 on the serial multiplier (slot 13) gives
  0 … 238 (±0.05 cycle at the default). It is added to the L green fold at stage 14, positive only so
  no fold sign flips.
- **Bend.** The bend factor is the full 8-bit `f_m6` (0 … 13 … 159; the product is 8 × 9). The Osc
  source is tx + ty − 512 (twice v0.5's swing) at twice the spatial rate.
- **Stripe curl.** `kb` = 0 at ≤ 30 % … 255 (slot 11). Stage 13 forms u(11:6) × kb, and stage 14 adds
  it / 32 to S green when the line's S green is ≥ 0 (the cores stay black); p14..16_sg carry it to the
  blend. Without that guard the cores filled with green (model comparison `p6/g.png`).
- **Fish to 4×.** The B ramp then passes 8 threshold units across an SD line, so `b_r` is 30 bits and
  `p1_r` saturates.
- **Opposed / Core.** S8 flips u on B rows only (`f_flip xor (f_opp and B)`). S9 points ops 6..8 at
  table 14 (red knots 0..10, green 16..26, blue 32..42) on the core lines (d_A knot ≤ 9). Knot 10 equals
  tables 11..13, so the core joins the stripes. The table is generated in `cusp.py` from the L layer,
  index 255 → 134 (the B centre halfway toward the A centre); the full sweep (`p6/core_255_13.png`)
  striped the cores too finely.
- **Removed:** S9 Glow Hard (the soft keys are always on), S8 Same (K2 = 0 does it), the K2 row
  scale (slots 18 / 19, `f_step` / `f_ptop`).
- **Per-field bases ride in the serial multiplier.** At load, `m_h` is set to base << k, which adds the
  base at v_pr bit 0 after 18 shifts. That replaces the conditional base adders on f_m1 / f_hh / f_m5 /
  f_m12 / f_m6 / f_m3 (−132 LUT, −120 carry, −108 FF standalone, together with the drift rework).
- **Standalone yosys costs** (v0.6 draft): stripe curl 108 LUT, scroll 80, Split 41, rainbow 7; the
  whole drift path 233 (`p6/abl/`).

Integer model: `.heirloom_work/rb_cusp/imodel6.py` (`GUARD=0` shows the unguarded curl). The control sheet is
`p6/sheet6.png`; the comparisons are `p6/s1.png`, `p6/s2.png`, `p6/g.png` and `p6/cores.png`.

## Status
- **v0.6.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s4 75.83, SD Analog s1 79.40, HD HDMI s2 74.40, SD HDMI s1 75.11, HD Dual s1 78.50, SD Dual s1 74.32 MHz; 7422-7435 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.6.0, built 2026-11-01, router2. NOT flashed.**
  - 7419–7448 / 7680 LC (96.6–97.0 %, FULL), 11 / 32 EBR, iceunpack OK × 6.
  - Builder seeds and post-route Fmax: HD Analog s3 77.17, HD HDMI s3 75.27, HD Dual s2 78.25,
    SD Analog s1 71.99, SD HDMI s1 72.81, SD Dual s1 78.84 MHz.
  - **HD seeds re-picked from a pin-skew screen** (`.heirloom_work/rb_cusp/skew/screen.txt`), then
    hand-installed and repacked. The builder's HD picks were all unsafe: HD HDMI s3 HSYNC −0.57 / data
    −0.30, HD Analog s3 −0.07 / −0.07, HD Dual s2 a data bit at +0.08.
    - HD HDMI **s138**: 78.71 MHz, HSYNC **+0.96** / data ≤ −0.78. Router2 seeds 1–144 (60
      timing-clean) gave nothing at ≥ +1.0 with data ≤ −0.6. Router1 seeds 1–24 all closed (75–84 MHz)
      but placed HSYNC at −2.8 … +0.4.
    - HD Analog **s48**: 75.07 MHz, HDMI HSYNC +1.05 / data ≤ −1.32 (seeds 1–60).
    - HD Dual **s11**: 75.99 MHz, −2.14 / ≤ −1.76 (seeds 1–12).
    - Spare: `skew/cusp_v060_hdmi_s19_spare.vmprog` (HD HDMI s19, +1.07 / ≤ −0.34, 79.31 MHz). Try it
      if the shipped one shows the hsync lines on HD HDMI.
  - To reproduce the shipped HD bitstreams: `skew/synth.sh <config>`, then
    `RT=router2 skew/route.sh <config> <seed>`. The builder's s3 / s3 / s2 re-routed byte-identical.
    The builder's vmprog is `skew/cusp_v060_builder.vmprog`.
  - Verification: GHDL 720p60 renders (Osc, Speed still) match `imodel6.py` bit for bit (0.00 mean abs
    after the 4 px avid offset). Two renders: the defaults, and Split 800 + Cusps 700 + Fish 700 +
    Bend 650 (curl) + Opposed + Rainbow core together (`p6/cmp_rtl6.png`). The model at the defaults
    equals the v0.5 model bit for bit.
  - Presets are placeholders: Photo, Flat Bars, Needles, Orange Tide, Sunset Cores, Chevrons, Amber Swarm.
  - If a rebuild regresses HD: router1 closes this netlist easily but its HD HDMI HSYNC sits negative.
    Reclaim LC from the stripe-curl multiply (108 LUT) or the chirp (~150 LC, 2.3 rms).

**v0.5.0:**
- v0.5.0 built 2026-10-27 with router2. **All six configs at seed 1.** Every bitstream passed iceunpack.

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 77.14 |
  | HD HDMI | 78.88 |
  | HD Dual | 78.15 |
  | SD Analog | 79.41 |
  | SD HDMI | 83.69 |
  | SD Dual | 77.30 |

- Size: 7204–7224 / 7680 LC (94 %), 11 / 32 EBR.
- Verification:
  - GHDL 720p60 Osc render vs the integer model: 0.53 mean abs after the avid / drift offset.
  - Motion checked: max Speed moves the cusps right ~8–9 px per field.
  - Video mode checked (selfie collage).
- **Not flashed.** Presets are hand-written placeholders (Photo, Flat Bars, Needles, Amber Swarm, Hard Left).
- If timing regresses: LC is at 94 %; try router1 or other seeds before restructuring. The next LC to
  reclaim is the chirp (~150 LC, costs 2.3 rms) or the red tint (~120).

## Deviations from the photo
- The fine stripes stop straight at the left sync strip instead of curling into it.
- The local wobble on fish 3's outline (a capture glitch) is not modelled.
- The orange rows' cyan bands show about 2 nested arcs instead of 3–4; the red X is a little smaller.
- Soft-key rise and fall rates are unified, and the blue tint along the bands is dropped (needed to fit;
  0.15 rms). The left strip's red bump along x is approximated by a constant tint.
- The camera's softness is reproduced only by the output filters, so hardware will look crisper than
  the photo.
- The default Bend (30 %) wiggles the cusps in Osc mode compared with the static photo: about ±20 px in
  v0.5, about ±40 px in v0.6 (the Osc source swings twice as far). Video mode (the default) and Bend 0
  are unchanged; Bend 0 reproduces the fit exactly.
- v0.6 motion at the default Speed: the orange rows slide against the black-core rows (~1 px/field
  relative), the stripes breathe in and out of the cores over ~16 s (the arcs gain up to two rings at
  the far end) and the rainbow swings ±0.05 cycle. Speed at the centre freezes all of it.
- Resolution: 4:3 SD shows the whole layout about 19 % narrower relative to the height. The glow / rim /
  arc widths come out about 1.33× wider than HD at SD and about 0.75× at 720p (the soft-key rate has two
  settings).
