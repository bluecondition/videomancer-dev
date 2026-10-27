# Curtain — red drapes, swept folds and pleat cuts

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Heirloom #34, from
`photos/WIN_20230521_01_34_12_Pro.jpg`. The v0.1.0 version (magenta folds wrung by a vertical sine,
blue pools, lime piping) was judged too abstract and replaced. The photo is two line-locked oscillators
feeding a keyed colourizer: swept folds in the upper half, cut by pleats that also own the red drapes
of the lower half, all striated by a fine third oscillator. Presets are hand-written placeholders.

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
1. **Oscillators.** Q16 DDAs per pixel:
   - A: 608 px, started at (T − sw·dx(y) − 260)·incA;
   - B: 542 px, a sawtooth whose wrap is a cut, started 15 px into its cycle;
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
| K1 Folds | fold tiles: arcs, pools, stripe bands | Fold period 1214–243 px, square law (608 at 512 = the photo, 3.2 folds). Glided. |
| K2 Pleats | cuts, flames, domes, lower drapes | Pleat period 1092–216 px (542 at 512 = the photo). Glided. |
| K3 Stripes | magenta striation | Stripe period 72–14 px (36 at 512). |
| K4 Speed | motion | Bipolar, still in the centre ±8. Positive: folds slide left past the cuts (default 600 ≈ 0.8 px/field, a fold every ~12 s), stripes stream, tears re-seed. |
| K5 Pools | cyan pools | Pool key level: 0 = none, 512 = the photo, max = pools flood the stripe bands. |
| K6 Melt | camera / Osc into the patch | 0 = none; default 384; max = the source reshapes the keys (±0.5) and bends the stripes (±2 cycles). |
| S7 Tear | torn rows and the seam band | On (photo) / Off |
| S8 Piping | green arcs | On (photo) / Off |
| S9 Flames | red flames after each cut | On (photo) / Off |
| S10 Velvet | palette | Red (photo) / Blue (R ↔ B: a blue curtain with gold pools) |
| S11 Source | camera input | Video (default) / Osc |

Defaults: K1 512, K2 512, K3 512, K4 600, K5 512, K6 384, P12 512, switches at label 0.

## Video
- **Video.** The photo is fully synthetic, so the camera is woven into the patch. The camera luma
  minus its running mean (one sample per 64 active px, τ 1024 samples) × Melt is added to both keys
  and to the stripe phase. Pools, stripes and red regions reshape around the picture and the
  striation bends over it. A flat or absent input leaves the photo untouched. At default Melt the
  composition reads clearly (selfie test).
- **Osc.** Two drifting triangles, 700 px across and 520 lines down, drive the same path, so Melt
  stays live.

## Engine
- **Field sequencer** (vblank zone, one serial multiplier, one serial divider):
  - incA, incB and incS from K1–K3, square law;
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
  pair (v[i], v[i+1]), so the pixel path interpolates along the phase with one read.
- **Pixel path.** 24 stages, one latency in every mode (C_LAT = processing_delay_clks = 24). Every
  ROM/RAM read is re-registered in fabric:
  - line RAMs and x-LUTs: pool colour; orange; upper ramp; shadow; lower ramp; dome darkness;
  - the arc-distance ROM;
  - flame ROM;
  - stripe-wave ROM.
  Domes and shadow are saturating subtracts, so there are no multipliers.
- **Memories (23 EBR).**
  - C_TAB 8192 × 8 (16);
  - line RAMs 2 × 256 × 16 (`no_rw_check`; read and write are always in opposite banks);
  - wave 256 × 8;
  - flames 128 × 8;
  - x-LUTs 3 × 128 × 16;
  - arc distance 512 × 8.
  All are generated by `curtain.py` into `curtain_pkg.vhd`.
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
