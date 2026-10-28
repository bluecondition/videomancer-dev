# Goblet — a glass goblet over RGB stripe oscillators, Heirloom #49

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230916_01_08_00_Pro.jpg` (1920×1080 capture, 2023-09-16). Replaces v0.1.0 (a 64-row goblet outline
over palette stripes), which Ron judged too abstract and not like the photo. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_goblet/` (`notes.md` has every number).
- **Blend rule: additive RGB, channel by channel.**
  - Along the cyan rim band (row 258, x 1104–1232) the photo's R equals the stripe's R (35/27, 18/8, 1/0, 138/148,
    151/169), while G gains ~+85 and B ~+100 (clipping at 245–255).
  - The orange line adds R +140..160, G −25, B −55.
  - So photo = clip(stripes(x) + glass(x, y)).
- **Stripes: three line-locked oscillators, one per channel.**
  - Measured as column medians over the glass-neutral pixels.
  - R is a pulse train (autocorrelation peaks 124 / 253 / 375 / 631 px).
  - G is a square-ish wave of ~327 px; B is a shaped wave of ~182 px.
  - The pulse heights vary from pulse to pulse, so the profile is stored whole.
  - The red columns carry wobbly yellow/white edges and chain-link loops.
- **Glass: a polar colourizer.**
  - Every pixel classified by nearest centroid of (photo − stripes); shares: neutral 45 %, blue-grey
    (−3,32,55) 17 %, olive (13,−18,−60) 13 %, cyan (−9,76,118) 6.7 %, blue (−4,−41,67) 5.2 %, orange
    (108,−31,−87) 4.9 %, pink (118,−28,37) 4.4 %.
  - Adjacency counts (N–L 45k, N–g 44k, g–b 24k, g–C 21k, L–O 15k, P–b 11k, O–P 5.6k) close a single wheel:
    neutral → olive → orange → pink → blue → cyan / blue-grey → neutral.
  - Per channel this is window comparators on one phase: R on over olive..pink, B up from pink to blue-grey,
    G on at cyan.
  - The cells (bowl, lenses) are the wheel's dim section, so they carry the photo's concentric olive / blue-grey
    shading.
- **Geometry.**
  - Martini bowl: rim y ≈ 255, x 820–1250; apex (1040, 595).
  - Stem x 1000–1100, y 600–780; foot along y ≈ 785.
  - Lenses tilted \ and / around (580, 500) and (1490, 500); X bands from the stem to the top corners.
  - Bottom cells below y ≈ 800; a C-curve at x 1380–1500; a top "smile".
  - Thin rainbow rim lines across the frame at y ≈ 255 and 785.
  - Mirrored about x = 1040: flip correlation 0.55 (B) / 0.47 (R); the axis is 1018..1051 by row band. Column
    correspondence is 0.85–0.96 near the axis and at u 720–840, but only 0.24–0.7 at u 280–680, so the halves
    genuinely differ.
- **Why a fitted field.**
  - No separable or analytic form fits (closed tilted lenses, sharp rims).
  - The wheel phase can't be unwrapped: median error 0.25 against 0.014 on a synthetic test, because the
    cells' phase flips 0 ↔ 1.
  - Single-scalar level fits reached only 81–85 % level agreement.
  - The glass is therefore a 2-vector field m·(cos 2πφ, sin 2πφ) on a bilinear knot grid. That needs no
    unwrapping; hue = angle, key = magnitude.
  - Full-frame mean |error| by grid: 32 px 17.1, 24 px 15.5, 16×34 15.9, mirrored 16.5×17 19.4,
    16×16 (8+8 bit) 14.0, 120×67 with 8-bit knots (5-bit angle + 3-bit magnitude) 15.65.
  - Joint gradient descent of knots + LUT + stripes through the exact hardware interpolation (Huber,
    clip-aware), then quantisation-aware fine-tuning, gave 9.3–9.7.
- **Line-start block.** A stadium at x ≤ 88 + √(72² − dy²), |y − 984| ≤ 73. It holds a colourized line-start
  ramp: blue 4–24 | white 28–40 | blue-grey 42–50 | green 55–100 | cyan 100–155. The left-edge stubs are its
  first 52 px on rows 176–215 and 330–371, purple (R +120) on 188–204 and 344–349. The bottom 7 lines are
  black, as in the capture.
- **Fit scores.** Float proto vs photo: 4×4 region means max 2.5 / mean 1.4 (of 255), luma local contrast 0.85,
  mean |error| 8.96. RTL-exact integer model: 3.4 / 1.8, mean 9.06.

## What it does
1. **Stripes.** The photo's stripe profile, one sample every 4 virtual px (480 per period), read by a Q16 DDA
   and interpolated linearly. Speed drifts it; the source bends it (FM, in 1/8-sample steps).
2. **Glass field.** 120 × 67 knots, 16.13 × 16.36 virtual px, one byte each (angle 5 bits, magnitude 3 bits).
   - A row engine interpolates the line's two knot rows vertically (angle wrap-aware) into a ping-pong row
     RAM: angle 10 bits, magnitude 6 bits.
   - The pixel path reflects / mirrors x in Q5 knots, then interpolates along the row: angle A (1/1024 turn)
     and magnitude M (0..224).
3. **Colourizer.**
   - M' = M·gain + source·Melt.
   - A' = A + Hue + flow + source·Melt + ring·M' (P12 above 50 %).
   - A 64 angle × 8 magnitude LUT gives the glass's additive R, G, B (halved). It is added to the stripes and
     clamped.
4. **Line-start block.** Opaque, with the ramp read from the spare entries of the stripe banks.
5. **Output.** RGB cable patch → 10-bit YUV (u = Cr, v = Cb; luma ×1.07) → [1 2 2 2 2 2 2 2 1]/16 on Y/U/V →
   luma 13/16 → blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Glass** | The glass layer: the goblet, its lenses, bands and rims. 0 % = a faint ghost of the glass over the bare stripes (gain 0.25); 50 % = the photo; 100 % = the glass swells (gain 1.5) and its colour rings multiply (magnitude → hue, up to 1.5 turns). Glided. |
| K1 Stripes | The stripe columns: frequency ×0.5..×2 (square-law halves, the photo at the centre). Glided. |
| K2 Width | The goblet's width: the field's x-scale about its axis, ×0.6..×1.6. Field coordinates reflect at the frame edges, so a narrow setting repeats the lenses. Glided. |
| K3 Hue | The glass colours turned round the wheel: a full turn moves every band's orange / pink / blue / cyan through the glass. Glided. |
| K4 Speed | Bipolar, still at the centre ±8, square law. The glass colours flow round the wheel (default 640: one turn ≈ 21 s) and the stripe oscillators drift (≈ 30 virtual px/s). Max is 16× the default. |
| K5 Melt | How far the source bends the stripes (up to ±63 px), shifts the glass hue (±0.4 turn) and swells or thins the glass. Default 35 %. Glided. |
| K6 Patch | The RGB cable patch: RGB (the photo) / RBG / GRB / GBR / BRG / BGR. A labelled default (index < 6) is read as its index. |
| S7 Mirror | Off (the photo's own halves) / On: the left half folded onto the right, an exactly symmetric goblet. |
| S8 Stripes | On (the photo) / Off: the glass alone on grey. |
| S9 Block | On (the photo) / Off: the bottom-left line-start block and the left-edge stubs. |
| S10 Flip | Off / On: the field upside down (bowl at the bottom, the X opening downward). |
| S11 Source | Video (default) / Osc. |

Defaults: K1, K2, P12 at 512; K3 0; K4 640; K5 358; K6 RGB; all switches label 0. Presets: Default, Ghost Glass
(P12 0), Rainbow Melt (P12 max, K4 760), Hourglass (Mirror + Flip, stripes off).

## Video
- **Video:** the camera luma, as a deviation (Y − 502)·0.28 and tapped straight from `data_in`, is patched like
  an analog CV:
  - into the stripe oscillators' phase, so the columns bend around the picture;
  - into the glass hue and magnitude, so the bands shift colour and swell over bright areas.
  - At default Melt the goblet, lenses and rims still read clearly.
- **Osc:** two triangle oscillators (700 px in x, 500 lines in y, ±0.06), drifting with Speed, feed the same
  path. At the default Melt they are a gentle living wobble, so Osc at defaults is the photo. Melt stays live.

## Engine
- **Shell:** heirloom v0.2.0.
  - Timing-ID virtual coordinates: x 1 / 1.5 / 2 virtual px per px, SD starting at x 240 (the central 1440);
    1080 virtual lines.
  - Vblank zone (no avid for 1024 clocks), source-width measurement, damier's parity-locked generated avid.
- **Glides:** a 3-clock pipeline (gap → step and ±4 deadband → apply) on K1, K2, K3, K5 and P12, snapping on
  the first field.
- **Field sequencer:** one serial shift-add multiplier, ~27 states. It computes:
  - the square-law knob curves (sf, iw);
  - the knot DDA step and start (Q16), the stripe step and start (mod 480 samples), the knot-row step;
  - the P12 gain and ring, the Melt gains;
  - the hue, the speed² drifts (hue, stripes, oscillator phases) and the oscillator steps.
- **Per-line engine** (kicked at the source avid rise; computes line n + 1): the knot row and its 5-bit fraction
  (flip-aware), the even / odd row bases, the line-start block's edge (128 × 8 table), the stub / purple /
  blank flags, the oscillator's y phase and the row-RAM half.
- **Row engine:** 60 clocks reading all four knot banks (both row and column parities) per clock. It does the
  wrap-aware vertical lerp and writes 2 × 16 bits per clock into the free half of two 128 × 16 RAMs
  (`no_rw_check`, 0 `emulate_transparency`).
- **Pixel path** (32 stages, one latency in every mode):
  - Q5 knot x, mirror, reflect, column and fraction.
  - Row-RAM read and fabric register, left / right by parity.
  - Wrap-aware angle and magnitude differences × tx.
  - Gain / source / hue products (7-bit operands), clamp, magnitude index, ring product.
  - LUT (512 × 24) and fabric register.
  - Stripe path started 10 clocks late, so it arrives aligned: FM bend, even/odd bank reads, 3-bit-fraction
    lerp, block / grey select.
  - Composite, patch, RGB → YUV, conditioning, gate.
- **Memories (25 EBR):** knot banks 4 × 2048 × 8 (16), LUT 512 × 24 (3), stripe banks 2 × 256 × 24 (4), row
  RAMs 2 × 128 × 16 (2). Tables come from `goblet.py` → `goblet_pkg.vhd`.
- **Verification** (GHDL 720p60 decimation 1, against the RTL-exact `imodel.py` through the shell's output
  stage):
  - Osc defaults: 99.97 % of pixels within 24 (sum of |dR|+|dG|+|dB|), mean 3.03, after the tester's 4-px
    capture offset.
  - Motion (K4 max, 6 warmup frames): the frame matches the model advanced 6 fields (93.2 %, against 41–43 % for
    ±1 field).
  - Video (selfie) and NTSC checked visually.
- **Fit history.**
  - The first RTL (per-pixel bilinear with six products, a stripe VCA, 36 stages) came to 8315 LC (108 %).
  - Ablations priced the VCA at 521 LUT, the glass gain products at 701 and the bilinear products at 478.
  - Narrowing the operands, adding the row engine and turning the VCA into FM brought it to 7104–7113 LC.

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** router2 (`./build_heirloom.sh goblet`).
- **Build seeds:** HD HDMI = seed 2 (seed 1 stalled the router); HD Dual = seed 2 (seed 1 missed timing);
  HD Analog, SD Analog, SD HDMI and SD Dual = seed 1.
- **Fmax (last post-route line):** HD Analog 81.92, HD HDMI 85.34, HD Dual 80.88, SD Analog 79.94,
  SD HDMI 82.06, SD Dual 75.41 MHz.
- 7104–7113 / 7680 LC (92.6 %), 25/32 EBR. All bitstreams pass iceunpack.
  `out/rev_b/ron/heirloom/goblet.vmprog` (523,503 bytes).
- router1 routes the same HD HDMI netlist at 92.70 / 90.65 MHz (seeds 1 / 2). If a later version tightens,
  rebuild with `NEXTPNR_ROUTER="--router router1"`.

## Deviations from the photo, and why
- **Fine highlights.** The 1–3 px lines are softer: the white/yellow X sparkles where bands cross the stripe
  edges, and the crispest orange rim lines. Band edges are slightly lumpier than the photo's smooth curves. The
  16-px knot grid is what 16 EBR holds; a 32-px grid lost the rims and thin bands.
- **Bubbly stripe edges.** The yellow edges and chain links on the red columns are partly carried by the fitted
  field, so they stay in place rather than being a stripe-edge texture.
- **A fitted field, not an oscillator.** The glass shapes are fitted (no analytic form or unwrappable phase
  fits), so they don't drift by themselves. Motion is the colourizer's flow, the stripe drift and the
  controls (width, mirror, flip, the source bending).
- **Colour flow.** At the default Speed the glass colours cycle (one turn ≈ 21 s), so the photo's exact palette
  recurs periodically.
- **Brightness.** The HD HDMI bar fix (luma 13/16) makes the picture darker, and the 9-tap chroma filter
  softens colour edges.
- **SD (4:3)** shows the central 1440 virtual px: the bottom-left line-start block and the outer ~240 px each
  side are cropped. 720p and 1080 show the same full picture.
- **EBR.** 25 EBR, one over the 24 budget. Fitting 24 would need unsmoothed 4-px stripes or a coarser glass LUT.
- **Video.** The camera enters as stripe bends and glass modulation (the photo is fully synthetic). Its pattern
  sits ~10–20 px off the camera picture (tapped early), which is invisible.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware).

## Working files
`.heirloom_work/rb_goblet/`:
- `notes.md`: look, measurements, fits, controls, model, RTL, verification, builds.
- Measurement and fits: `stripes3.py` → `stripe3.npy`, `cls3.py` / `lab3b.png` (classes),
  `flip2.py` / `asym.py` (mirror), `wheel.py` / `unwrap2.py` (failed phase unwrapping), `vecfit3.py` /
  `angfit.py` (grid comparisons), `jopt*.py` → `jopt4q.npz` (joint fit).
- Proto: `proto.py` → `proto_final.png`, `cmp_proto.png`; controls `sheet.py` → `sheet.png`.
- Model and tables: `imodel.py`, `tables.py`, `gen_hook.py` (writes `goblet.py`), `pkgcheck.py`.
- RTL checks: `rtlcmp.py`, `abl.sh` (standalone yosys ablations), `check_local.sh`.
- Renders: `rtl_osc2.png`, `rtl_vid.png`, `rtl_mot.png`, `rtl_ntsc.png`; `cmp_final.png`, `cmp_motion.png`.

## Notes
- The Hourglass preset also sets S8 Stripes Off (toggle_switch_8 = 1): it shows the mirrored, flipped glass on grey. Not intended by the agent; left as is because changing the TOML needs a rebuild.
