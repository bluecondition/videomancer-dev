# Curtain — red drapes, swept folds and pleat cuts

**v0.6.1 (2026-11-01, phase 2): K1 Seam dissolves the seam, K3 Weave, S7 Swap, S8 Ribbons, S9 Fire;
the pleats stand still at the default Speed again. Built, NOT flashed.** (v0.6.0 flashed: Ron "liked
the previous one more", the default no longer looked like v0.5.0. v0.5.0, 2026-10-27: rebuilt from
the photo.) Heirloom #34, from
`photos/WIN_20230521_01_34_12_Pro.jpg`. The v0.1.0 version (magenta folds wrung by a vertical sine,
blue pools, lime piping) was judged too abstract and replaced. The photo is two line-locked oscillators
feeding a keyed colourizer: swept folds in the upper half, cut by pleats that also own the red drapes
of the lower half, all striated by a fine third oscillator. Presets are hand-written placeholders.

## Phase 2 (v0.6.0)
Ron (2026-10-31) approved a proposed map without flashing v0.5.0, and asked for something with the
seam height, or a blend that removes the seam. K1 was traded for the seam (his pick).
- **K1 Folds → Seam.** The seam becomes a wipe. A pixel shows the fold half when
  `t(y) < f(x)`, where:
  - `t = (2 s + 1) G / 256`, with s the row's distance below the seam;
  - G is the per-field gain for a wipe half-height `D = K1² / 951` lines: 0 at K1 0 (the photo's hard
    seam), 69 at 25 %, 276 at 50 %, ~1100 at 100 %;
  - `f = pleat phase[15:8] + a 0..31 stripe triangle − 144`: the pleat sawtooth, roughened by the stripes.

  As K1 rises, the drapes climb into the folds as tall pointed hems (one per pleat, peaking just after
  each cut) and the folds hang into the drapes. At 100 % the halves interleave in near-vertical strips.
  Each half's table is mirrored past the seam (fold rows reflect about 535.5, drape rows about 543.5),
  so no horizontal line appears anywhere. The mirroring shows as a soft reflection axis at mid-height:
  domes become lenses and chevrons become Xs. The fold spacing is fixed at the photo's 608 px.
- **K3 Stripes → Weave.** One depth value w drives four things: 0 at K3 0, 40 at 50 % (the photo),
  159 at 100 %.
  - stripe amplitude: stripe blue `120 + 3 (S − 128) w / 256`, clamped;
  - stripe bite into every key edge: key jitter `(S − 128) w / 256`;
  - chevron depth: the walk slope × w / 40;
  - drape slant: `w · 59` per line.

  0 = flat colour fields; 100 % = hard two-level stripes, edges chewed ±80 key units, chevrons 4×
  deeper, drape stripes near horizontal. The stripe period is fixed at 36 px.
- **K4 Speed** also drifts the pleats, by 0.625 (K4 − 600)/65536 cycle per field: right as K4 rises
  above its default, left as it falls. The cuts slide through the folds, so flame lengths (set by the
  fold phase at the cut) and arc gates re-evaluate as they pass, and the drapes slide with the
  pleats. **At the default (600 ± 8) the pleats sit at the photo's position (15 px into the cycle)**,
  and they snap back there whenever K4 returns to it.
  - v0.6.0 drifted them at 0.625 sp, so they moved at the default Speed too (~27 px/s at 60 Hz).
    Ron flashed v0.6.0: "I liked the previous one more, the default does not look like the last
    version". Several photo elements are fitted to the cut positions but locked to screen x: the
    orange left tile, the drape narrowing, the right-hand shadow, the dome darkening. Within seconds
    of loading the composition left the photo. GHDL showed it once the pleat phase was advanced by
    10 s (`.heirloom_work/rb_curtain/v060chk/cmp_drift_vid.png`). The single-frame check two fields
    after load had matched v0.5.0, as had the camera (selfie) render.
- **S7 Tear → Swap.** Folds Up (the photo) / Drapes Up: the drapes hang from the top (rows from 0),
  the folds start at line 544. The seam wipe and the mirroring work the same way. The tears are
  always on now, at the photo's rate.
- **S8 Piping On/Off → Arcs / Ribbons.** Ribbons: the arcs 2.5× wide (a second distance table in the
  arc ROM), at full width with no taper, ungated (from the top down to the seam, across the cuts).
- **S9 Flames On/Off → Teeth / Fire.** Fire: flames 3× longer, domes twice as wide and starting at
  fold row 255 instead of 355.
- Unchanged: K2 Pleats, K5 Pools, K6 Melt, S10 Velvet, S11 Source, P12 Drape.
- **Defaults reproduce v0.5.0** (GHDL 720p, `.heirloom_work/rb_curtain/v060/cmp_v060.png`: photo /
  v0.5.0 / v0.6.0 defaults / four new looks; the selfie render matches v0.5.0 at the pixel-texture
  level, `v060chk/crops_vid.png`). As in v0.5.0, only the folds and the stripes move at the default
  Speed.
- **Lesson.** Check a default for drift over SECONDS, not just the first frame. Any per-field motion
  added to an element that the fitted, x-locked tables depend on will leave the photo at the default.

### Engine changes
- **Line engine.** Every line now interpolates BOTH halves' table rows into the line RAM:
  - the row loop runs 384 reads (drape main + first B cycle 256, fold 128);
  - the line RAM is 2 × 512 × 16 (slot 00 drape, 01 drape first cycle, 10 fold);
  - new states: the fold/drape row coordinates (Swap-aware), their mirroring, two row products,
    the wipe product `|2 s + 1| G`;
  - the sweep, domes, shadow and upper ramp all use the fold-local mirrored row;
  - the drape slant uses the unmirrored signed drape row (continuous across the seam).
- **Pixel path.** Stage 1 compares the per-line threshold with the per-pixel edge f. The result
  picks the line RAM slot, the phase (A for the fold, B for the drape) and the interpolation fraction,
  and travels as `p_fold` to gate the ramps, the shadow, the domes, the flames, the arcs and the drape
  fringe. There is one stripe oscillator. The drape stripes add a per-line offset (slant minus walk)
  at stage 2.
- **Fit (first draft 102 % pack-only).** Ablation-led savings with no feature lost:
  - melt `(L − mean)·g` → `L·g − mean·g`: unsigned 8×6, with mean·g a per-field offset;
  - weave `(S/4 − 32)·w/4` → `S/4 · w/4`, its constant part per field; one key (the fold half's
    1.2× pool-key jitter dropped);
  - phase interpolation in eighths, the upper ramp's y factor in 16 levels;
  - the pleat drift as a phase accumulator (no wrap logic);
  - the chevron gain folded into a per-field slope scale;
  - **RGB → YUV in luma-difference form.** `ys = 19 R + 38 G + 7 B`, `Y = 64 + 271 ys / 4096`,
    `Cr = 512 + (64 R − ys)·0.039`, `Cb = 512 + (64 B − ys)/32`. Nine adders instead of 23; colours
    within ~1.5 % of v0.5;
  - **the output chroma filter's 8-sample U/V delay in one 256 × 16 RAM** (8-bit U & V, write every
    clock, read at wa − 6, `no_rw_check`): −160 FF-only cells. Chroma is forced neutral on blank pixels,
    and the two 8-sums reset to neutral after 8 blank samples, so power-up RAM contents cannot leave a
    lasting colour cast.

  The result: 7292 LC (94.9 %), 27 EBR.

## Reading of the photo (measured)
The work is in `.heirloom_work/rb_curtain/` (`notes.md` has every number). The analysis scripts are
`acorr.py`, `arcfit.py`, `edges.py`, `teeth.py`, `domes.py`, `darkz.py`, `demod*.py`, `classx.py`
and `fit_kr2.py`.

**Composition.**
- A horizontal seam at y ≈ 536 splits the frame.
- Horizontal autocorrelation finds a period of 606–608 px in the upper half (R/G correlation 0.76)
  and 542 px in the lower half (0.63).
- Torn rows cluster at y 520–600, with a few elsewhere (y ≈ 30–60, 215–245 and 365–415 on the right).

**Fold oscillator A (upper half).**
- **Arcs.** Three parallel green arcs per fold, at u = 0, 135 and 245 px. All six tracked arcs follow
  one sweep, dx(y) = 0.0011987 y² + 0.25954 y − 3.6, with rms 3 px. The copies repeat at 605–611 px.
  The green core is 26–32 px wide, with a cyan fringe on the left and a navy fringe on the right.
- **Other content.** Red/orange columns run between the arcs, with cyan pools and magenta stripe bands
  on "/" diagonals.
- **Content model.** In the sheared coordinate u = x − dx(y), the content is not a 1-D colourizer of
  one coordinate (purity 0.6). It is therefore a fitted 2-D table.

**Pleat oscillator B.**
- **Cuts.** A sawtooth reset at x = −15 + 542k: x = 527, 1069 and 1610 at y = 700, drifting 4–10 px
  left at the bottom.
- **Arc gate.** An arc is drawn only while (pixels since the last cut) − dx(y) ≥ 0.35 PB. It ends
  about 10 px before the next cut. This holds for all 9 arc copies.
- **Flames.** Red flames run on after every cut. Lengths grow from 0 to about 137 px at roughly 1 px
  per line, then drop. As a function of the A phase at the cut, the two measured cuts agree: teeth for
  u_r in (118, 222) and (−50, 100) px.
- **Domes.** Grey domes sit before the cuts at 527 and 1069 for y ≥ 360. Their width is 40 (y 360),
  65 (390), 95 (420) and 115 px (480 and below). They have a ~10 px orange rim. The second one is
  dark slate.
- **Lower half.**
  - Red pleats with dark pleat lines at θB ≈ 0.13 and 0.28, dotted with blue specks.
  - A striped drape on the right of each pleat; its left boundary moves right going down.
  - A dark shadow dome at the bottom left of the drape.
  - A blue fringe at each cut.
  - The first pleat (x < 527) is an orange variant with a wider drape.

**Striation (B channel).**
- Horizontal period 34–41 px everywhere (median 36).
- R carries almost none of it: amplitude 4–14 against 19–33 on B. B spans 66–176 (5–95 %).
- Upper half: the per-row phase is piecewise linear. Runs of 10–30 lines are flat or slanted
  (|slope| 0.02–0.08 cycle/line), with occasional jumps; together they make the chevrons.
- Lower half: a steady slant of −0.01 to −0.045 cycle/line.

**Palette (k = 12, share).** Red (248,3,2) 31.6 %, dark magenta (149,3,92) 11.5, pink-red (228,1,88)
10.7, magenta (219,1,150) 8.9, purple (129,4,158) 7.1, dark red (161,11,11) 6.2, near-black (61,11,29)
5.7, orange (241,61,6) 5.5, azure (17,157,242) 4.5, navy (36,43,145) 3.9, grey (115,125,150) 2.5,
green (6,227,100) 1.8.
- Orange appears only at x < 720.
- Pools go light cyan → azure → navy (x > 1200).
- Same-phase fold copies darken to the right: R 100 → 52 → 15 across the three tiles.

**Fit quality** (class agreement with the photo on 8×8 blocks; 4×4 region means):
| | Agreement | 4×4 mean error (mean / max) | Local contrast |
|---|---|---|---|
| Float proto | 0.772 | 8.6 / 55 | 1.04 |
| Integer model | 0.757 | — | — |
| RTL (GHDL 720p) | 0.755 | 8.4 / 62 | 1.18 |

The RTL's largest error is the top-right shadow cell.

## What it does
(As at v0.5.0. The phase-2 section above lists what v0.6.0 changes: the seam wipe picks each pixel's
half, both halves' rows are on every line, and the stripe, melt and colour maths are restructured.)
1. **Oscillators.** Q16 DDAs per pixel:
   - A: 608 px, started at (T − sw·dx(y) − 260)·incA;
   - B: 542 px, a sawtooth whose wrap is a cut, started 15 px into its cycle (v0.6: drifting with K4);
   - the stripe oscillator: 36 px;
   - the Osc source.
   A counter tracks the pixels since the last cut.
2. **Key and red shade.** Each half has two photo-fitted fields, K (key) and R (red shade), stored as
   4-bit tables over (phase, row):
   - upper half: 64 × 64 over the A phase;
   - lower half: 64 × 32 over the B phase;
   - a separate 64 × 32 table for the line's first B cycle.
   K is blurred [1 2 1]. Rows are interpolated by the line engine and phase per pixel.
3. **Keys.**
   - k = K + ramp + melt + jitter·(S − 0.5), where S is the stripe wave.
   - The pool key uses a larger jitter in the upper half.
   - Upper ramp: x·y, so pools shrink to the right. Lower ramp: x only, so the drapes narrow to the
     right.
   - Classes: pool (k < kp), purple fringe, stripes, red (k ≥ 163).
4. **Colourizer.**
   - Red: R = shade, G = orange ramp × bright red, B small.
   - Stripes: R = shade, B = 60 + 0.47 S.
   - Pool: cyan → azure → navy by x.
5. **Pleat effects.**
   - Domes: pool pixels within wd(y) of the next cut, a grey gradient with an orange rim, darkened
     by x.
   - Right-hand shadow: x > 1450, around y ≈ 380.
   - Flames: red for L(A phase at the cut) px after each cut.
   - Lower-half blue fringe before each cut.
   - Arcs: an arc-distance ROM by A phase, gated by the B-ramp comparator and tapered before the cut.
6. **Tears.** One xorshift word per line drives both the stripe walk and the tears. Tears hit about
   12 % of lines in the seam band and 1.2 % elsewhere, in bursts of 1–4 lines. They shift the folds
   and stripes by up to ±63 px.
7. **Output.** Velvet (R↔B), then RGB → 10-bit YUV in the hardware convention (luma pre-boosted
   16/13), the shell's conditioning, and the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| **P12 Drape** | the folds' sweep (the arcs' curve) | Sweep 0–2×. 0 % = straight hanging folds (vertical arcs ending at different heights); 50 % = the photo; 100 % = whipped folds, arcs near horizontal and cut short. Glided. |
| K1 Seam | the seam between the folds and the drapes | 0 = the photo's hard seam (default). Up: a wipe along the pleat sawtooth (stripe-roughened), half-height K1²/951 lines (69 at 25 %, 276 at 50 %, ~1100 at 100 %). The drapes climb into the folds as pointed hems and the folds hang into the drapes, until the halves interleave in near-vertical strips; each table is mirrored past the seam. Glided. |
| K2 Pleats | cuts, flames, domes, lower drapes | Pleat period 1092–216 px (542 at 512 = the photo). Glided. |
| K3 Weave | magenta striation | Depth w 0 / 40 (at 512, the photo) / 159: stripe amplitude, stripe bite into every key edge, chevron depth, drape slant. 0 = flat colour fields; max = hard stripes, chewed edges, deep chevrons, near-horizontal drape stripes. |
| K4 Speed | motion | Bipolar, still in the centre ±8. Positive: folds slide left past the cuts (default 600 ≈ 0.8 px/field, a fold every ~12 s), stripes stream, tears re-seed. The pleats stand at the photo's position at the default (600 ± 8) and drift with K4 − 600: right above it (up to ~2.2 px/field), left below it (up to ~3.1). |
| K5 Pools | cyan pools | Pool key level: 0 = none, 512 = the photo, max = pools flood the stripe bands. |
| K6 Melt | camera / Osc into the patch | 0 = none; default 384; max = the source reshapes the keys (±0.5) and bends the stripes (±2 cycles). |
| S7 Swap | which half hangs on top | Folds Up (photo) / Drapes Up (drapes from the top, folds from line 544) |
| S8 Piping | green arcs | Arcs (photo) / Ribbons (2.5× wide, full width, ungated: from the top to the seam, across the cuts) |
| S9 Flames | red flames after each cut, grey domes | Teeth (photo) / Fire (flames 3× longer, domes twice as wide and taller) |
| S10 Velvet | palette | Red (photo) / Blue (R ↔ B: a blue curtain with gold pools) |
| S11 Source | camera input | Video (default) / Osc |

Defaults: K1 0, K2 512, K3 512, K4 600, K5 512, K6 384, P12 512, switches at label 0.

## Video
- **Video.** The photo is fully synthetic, so the camera is woven into the patch. The camera luma
  minus its running mean (one sample per 64 active px, τ 1024 samples) × Melt is added to both keys
  and to the stripe phase. Pools, stripes and red regions reshape around the picture and the
  striation bends over it. A flat or absent input leaves the photo untouched. At default Melt the
  composition reads clearly (selfie test).
- **Osc.** Two drifting triangles, 700 px across and 520 lines down, drive the same path, so Melt
  stays live.

## Engine
- **Field sequencer** (vblank zone, one serial multiplier, one serial divider). v0.6 adds: the seam
  gain G (a division), the weave depth and its chevron/slant/offset terms, the pleat phase and the
  pixels since the cut at the line start, the melt mean × gain. The original states:
  - incA, incB and incS from K1–K3, square law (v0.6: incB only);
  - PBpx = 65536/incB and 0.35 PB;
  - the pool levels;
  - the Melt gain;
  - per-real-pixel increments;
  - constant phase offsets: 2048 incA − drift, 64 incS − drift, 15 incA, 15 incB;
  - drifts and the tear re-seed.
- **Line engine** (next line, kicked at the source avid rise; about 200 clocks plus a row loop in
  parallel):
  - the row and quarter-row;
  - one xorshift32 word (stripe walk and tears);
  - dx from three serial products, then dxs = dx·sw;
  - the A start, the first-flame phase, the stripe start (walk, or −0.036 cycle/line in the lower
    half) plus the tear term;
  - the y factor, the dome width (an ogive as min of three lines), the shadow tent, and the Osc y
    triangle.
- **Row loop.** 128 (upper) or 256 (lower) table reads interpolate two rows into a ping-pong line-RAM
  pair (v[i], v[i+1]), so the pixel path interpolates along the phase with one read. (v0.6: all 384
  reads on every line.)
- **Pixel path.** 24 stages, one latency in every mode (C_LAT = processing_delay_clks = 24). Every
  ROM/RAM read is re-registered in fabric:
  - line RAMs and x-LUTs: pool colour; orange; upper ramp; shadow; lower ramp; dome darkness;
  - the arc-distance ROM;
  - flame ROM;
  - stripe-wave ROM.
  Domes and shadow are saturating subtracts, so there are no multipliers.
- **Memories (27 EBR at v0.6).**
  - C_TAB 8192 × 8 (16);
  - line RAMs 2 × 512 × 16 (`no_rw_check`; read and write are always in opposite banks);
  - wave 256 × 8;
  - flames 128 × 8;
  - x-LUTs 3 × 128 × 16;
  - arc distance 1024 × 8 (the second half for Ribbons);
  - the output chroma delay, 256 × 16 (`no_rw_check`, read 6 behind the write).
  The ROMs are generated by `curtain.py` into `curtain_pkg.vhd`.
- **Shell.** Cathode/escalator v0.2.0:
  - timing-ID virtual-1080 steps;
  - the vblank zone;
  - width measurement;
  - the generated avid;
  - chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16;
  - the blanking gate.
- **Fit history.**
  - First RTL: 8346 LC (108 %). Ablation showed the line engine's scalar program was about 1300 LUT.
  - Lean engine: B untorn, a constant lower slant, fewer products. 7801 LC.
  - Further cuts:
    - x-LUTs read late;
    - an arc-distance ROM instead of three subtract/abs/compare chains;
    - subtractive shading;
    - arithmetic in place of small LUTs;
    - no 2-px box;
    - K3 latched;
    - row interpolation in quarters.
    7059 LC.
  - Timing: the first profile reached 46.9 MHz (the dome-width logic in one engine state). After
    splitting the long engine and pixel chains: 83.55 MHz.
- **RTL bugs found in GHDL:** a narrowing `resize` on K (red came out as stripes), a signed slice used
  as an index, and the arc ROM read a stage late.

## Status
- **v0.6.1 built 2026-11-01, router2, NOT flashed.** v0.6.0 plus one change: the pleats stand still
  at the default Speed.
  - 7315–7357 / 7680 LC (95.2–95.8 %, FULL), 27 / 32 EBR, iceunpack OK × 6.
  - **First try (pool-key jitter restored + a glide-home spring + a busy-flag sequencer): 7416–7460 LC.**
    HD timing collapsed: 2 of 28 HD Analog seeds met 74.25, and every critical path was routing-bound
    (the field sequencer, the knob glides). The pool jitter proved invisible (the selfie renders already
    matched v0.5.0's pools). The busy flag cost 20 LC and the spring 62 (pack-only variants in
    `.heirloom_work/rb_curtain/ys/var/`). The shipped netlist has none of them; the pleats snap home
    instead of gliding.
  - Builder seeds and post-route Fmax: HD Analog s5 80.08, SD Analog s1 73.91, HD HDMI s6 **62.62
    (best-effort MISS, replaced)**, SD HDMI s1 70.98, HD Dual s6 74.38, SD Dual s1 76.55 MHz.
  - **Pin-skew screen** (router2; HD Analog s1–40, HD HDMI s1–30, HD Dual s1–16 on make-regenerated
    netlists, which the builder's bins match byte-exact: `.heirloom_work/rb_curtain/skew62/screen.txt`).
    Timing is thin: 4 of 36 HD Analog seeds, 6 of 29 HD HDMI seeds and 1 of 15 HD Dual seeds met 74.25.
    - HD Analog **s5** (the builder's): 80.08 MHz, HDMI HSYNC −1.22 / data ≤ −0.73 ns.
    - HD HDMI **s11**, hand-installed: 76.44 MHz, HSYNC +1.88 / data ≤ −0.67 ns, the closest to
      v0.6.0's s5 (+1.90), which Ron flashed without reporting lines.
    - HD Dual **s6** (the builder's, the only passing seed): 74.38 MHz, −2.11 / ≤ −1.36 ns.
  - To reproduce HD HDMI: `skew62/synth.sh hd_hdmi`, then `RT=router2 skew62/route.sh hd_hdmi 11`.
    The builder's vmprog (HD HDMI missing timing) is `skew62/curtain_v061_builder.vmprog`.
  - `out/rev_b/ron/heirloom/curtain.vmprog` (514,968 bytes). Presets are placeholders.
  - If a rebuild regresses: this netlist is congestion-bound at 95 %, and HD Dual has one passing seed
    in 16. Screen more seeds before restructuring, and remove logic rather than pipelining.

**v0.6.0:**
- **v0.6.0 built 2026-11-01, router2.** Flashed by Ron: the default left the photo within seconds
  (the pleats drifted at the default Speed). Superseded by v0.6.1.
  - 7282–7321 / 7680 LC (94.8–95.3 %, FULL), 27 / 32 EBR, iceunpack OK × 6.
  - Builder seeds and post-route Fmax: HD Analog s2 74.42 (s1 stalled the router), SD Analog s1
    68.11, HD HDMI s5 77.51, SD HDMI s1 72.69, HD Dual s2 75.75, SD Dual s1 71.76 MHz.
  - **Pin-skew screen** (router2; HD HDMI s1–24, HD Analog s1–36, HD Dual s1–16:
    `.heirloom_work/rb_curtain/skew/screen*.txt`). HD timing is thin: 3 of 22 routed HD HDMI seeds
    and 9 of 31 HD Analog seeds met 74.25.
    - HD HDMI **s5** (the builder's, byte-exact): 77.51 MHz, HSYNC +1.90 / data ≤ −0.99 ns.
    - HD Analog **s22**, hand-installed: 78.59 MHz, HDMI HSYNC −2.87 / data ≤ −1.15 ns. The builder's s2
      had a data bit at +0.59.
    - HD Dual **s2** (the builder's, byte-exact): 75.75 MHz, −2.43 / ≤ −1.68 ns.
  - A plain rebuild gives the builder seeds. To reproduce the shipped HD Analog, route
    `skew/hd_analog/bitstreams/hd_analog.json` with `RT=router2 skew/route.sh hd_analog 22`. The
    builder's vmprog is `skew/curtain_v060_builder.vmprog`.
  - `out/rev_b/ron/heirloom/curtain.vmprog` (513,623 bytes). Presets are placeholders: Photo,
    Hanging, Whipped, Blue Velvet, Dissolve, Hems, Bonfire, Flat.
  - The first pack failed on a 135-byte description (max 127). The TOML converter's own check passed
    it; only the packer catches it.
  - **Check first on hardware:**
    - Colour casts or chroma noise point at the new RAM chroma delay, which sims can't verify. Its
      neutral resync runs every line.
    - Thin per-line hue lines on HD HDMI would be the pin-skew problem: re-seed.
  - If a rebuild regresses HD: try `NEXTPNR_ROUTER="--router router1"`. To reclaim LC, the stripe
    roughening in the seam edge (~25 LC) and the luma delay are the next cuts.

**v0.5.0:**
- **v0.5.0 built 2026-10-27, router2, NOT flashed.**
- **Build seeds: HD HDMI = seed 2** and **SD Dual = seed 2** (seed 1 stalled the router on both). The
  other four are seed 1.
- Fmax (post-route): HD Analog 83.55, HD HDMI 81.31, HD Dual 80.04, SD Analog 76.82, SD HDMI 73.61,
  SD Dual 73.54 MHz.
- 7173–7193 / 7680 LC (93.5 %), 23 / 32 EBR. iceunpack OK × 6.
  `out/rev_b/ron/heirloom/curtain.vmprog` (492,710 B).
- **Source vs build:** after the build, unused constants were removed from `curtain.vhd` and the build
  hook (no synthesis change). The committed text differs slightly from what the vmprog was built from;
  rebuild before release so they match.
- **Verified in GHDL** at 720p60 decimation 1:
  - Osc defaults match the float proto and the integer model;
  - Video (selfie) bends the keys and stripes, and the composition reads;
  - motion at K4 max over 3 fields: the arcs move 12 virtual px and 39 % of pixels change.
- 93.5 % LC is tight. If a rebuild regresses, try `NEXTPNR_ROUTER="--router router1"` before
  restructuring.

## Deviations from the photo
(v0.6.0 at its defaults adds: colours within ~1.5 % from the luma-difference RGB → YUV, chroma
filtered at 8 bits, key interpolation in 1.2 px steps, the fold half's pool-key jitter 40/256 instead of
48/256, and the still frame slowly drifting at the default Speed. The list below is as at v0.5.0.)
- **One fold tile repeated.** The photo's three copies differ in detail. The dark zone after the
  second cut and the torn red bars at the top right are only approximated, by a right-hand shadow and
  x-ramps.
- **Pools** are flat cyan → navy by x, without the photo's internal gradients. **Domes** are a grey
  gradient with an orange rim, with a harder outline than the photo's.
- **Lower drape.** Some diagonal hatching appears inside, where the key sits near the red threshold.
  The drape's stripes are straight diagonals: the per-pixel slant toward each cut was cut to fit. The
  lower shadow dome is lighter than the photo's near-black.
- **Textures.** The stripe walk and the tears are random realisations, matched in statistics. Tears
  shift the folds and stripes but not the cut lines.
- **Brightness.** Whites peak at about 777 after the shell's output scaling. The black capture border
  isn't reproduced.
- **Resolution.** 720p is the 1080p picture. On SD (4:3, 1440 virtual px) the frame shows the left
  three quarters of the composition: two full folds and part of the third.

## Working files
`.heirloom_work/rb_curtain/`:
- `ref.jpg`: the photo.
- `notes.md`: look, measurements, proto log, controls, model, RTL, build.
- `fit_kr2.py`, `gen_hook.py`, `hook_template.py`: the table fit and the build-hook generator.
- `proto_final.py` / `proto_final.png`: the float proto. `sheet.py` / `sheet.png`: the control sheet
  (rendered from the integer model).
- `imodel.py`: the integer model of the RTL, which imports the tables from `curtain.py`.
- `rtl_osc_final.png`, `rtl_vid_final.png`, `rtl_motA.png`, `rtl_motB.png`: GHDL renders.
- `cmp_final.png`: photo / proto / RTL Osc / RTL Video.
- `ys/`: LC ablation and pack scripts.
- `old_v010_*`: the replaced v0.1.0.
