# Lotus — twin contour eyes over pink cushions, torn by a stuttering sync

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/lotus.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-02, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Lotus Diag`, `out/rev_b/ron/_diag/lotus_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/lotus/`) is untouched.

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Rebuilt from the capture
`photos/WIN_20230515_23_23_36_Pro.jpg`: two orange/green contour "eyes" over a yellow-green zigzag
band, a pink/white cushion keyed into the bright green bottom of each copy, and a stuttering line-start
staircase at the left edge. The v0.1 version (an abstract colourized field) is replaced. Presets are
hand-written placeholders.

## Phase 2 (v0.6.0)
Ron left no notes for lotus, so this round follows the phase-2 brief on a free hand: element-level controls over a
global X scale, about twice the range on each side of the photo setting, bolder switches, more video. Built as a diag
copy (`.heirloom_work/rb_lotus/diag/lotus_diag/`); the defaults draw the v0.5 picture.

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Lobes: copies across 0.7 .. 5.8 (64-step table), photo 2.04 at 50 %; the whole picture (eyes AND cushions) scales in X | **The eyes alone: 0.51 .. 7.5 eye copies across**, exponential (4 octaves, piecewise-linear exp2, smooth 10-bit knob), photo 2.04 at 50 % (bit-exact) | The cushions now run on their own oscillator at the photo's 2.04 across, so turning Lobes multiplies or stretches the eyes over two fixed cushions. The cushion interiors show the field at the eyes' period, so their X pattern repeats with Lobes. Below ~0.7 copies the contours step in ~7 px (64 field entries per period) |
| K2 Stutter: 0 .. 2×, linear, photo at 50 % | **0 .. 4×**: 0 .. 1× over the lower half (as v0.5), 1 .. 4× over the upper half | At 4× the line-start staircase reaches ~880 px in, the frozen runs ~680 px, the seam jumps ×4 and the tears 120 px (middle) / 48 px (bottom). The stutter datapath was widened (start / freeze / jump 10 bits, 23-bit stutter error) |
| K5 Ripple: the middle band's 8× zigzag and the seam stripes, 0 .. 2×, photo at 50 % | **0 .. 50 % as before (band zigzag 0 .. 1×); above 50 % the band keeps going to 2× AND a zigzag spreads over the whole field**, eyes and cushions included, up to ~1.5 rings peak-to-peak at 100 % (Rings at the photo) | A sixth product per field entry: (X − 128) × spread, centred so it adds no phase offset; zero at and below 50 %, so the default is bit-exact. It scales with Rings like every other field term |
| K6 Etch: how far the source bends the contours | **+ in Video mode the picture's bright areas swell the cushions sideways**: ~50 px at full white at the default Etch, ~150 px at the top | Unipolar (black input = no change), so Video with no source and all of Osc mode are unchanged. Also fixed: Etch × Rings wrapped (the etch gain was an 8-bit slice of the product), so Etch's top collapsed whenever Rings sat above the photo; it now saturates |
| S8 Mirror: Off / On (every eye's tip points left) | **S8 Mirror: Off / Pair**: every second eye mirrored about its core, so neighbours face each other | Seamless across the copy seams (T is symmetric about the eye core). Counted from the eye at the screen centre (parity 0), so at the default the left eye points right and the right eye points left, tips meeting in the middle |
| P12 Rings: 0.25 .. 3× (0.25 + 2.75 p²), photo at 534 | **0.25 .. 3.95×**: the same curve to 534 (bit-exact), plus (p − 534)/2 above it | Denser moiré at 100 % |

Unchanged: K3 Bloom, K4 Speed, S7 Palette, S9 Contour, S10 Fold, S11 Source. Control names are unchanged; only S8's
second label changed (On → Pair).

Presets: **Photo** (the defaults; was "Lotus"), Broad Petals, Moire Bloom, Clean Sync, Night Eyes (K1 760 is now
4.0 eyes, was 3.3), new **Facing Eyes** (S8 Pair, K1 380: ~1.5 eyes), **Ripple Field** (K5 960, P12 700) and
**Sync Storm** (K1 820, K2 1000, K4 820).

Build (ROUTER1, 2026-11-02, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh lotus`): HD Analog s1
76.58, SD Analog s1 88.82, HD HDMI s1 86.64, SD HDMI s1 82.27, HD Dual s1 81.70, SD Dual s1 88.15 MHz (6/6 seed 1);
7,359–7,373 / 7,680 LC (96.0 %, +~210 over v0.5.1), 22 / 32 EBR. HD Analog's margin fell from 86.8 to 76.6 MHz
(thin). No pin-skew screen; builder seeds kept. Preview: `out/rev_b/ron/_diag/lotus_diag_v0.6.0.vmprog`
(488,084 B; copy in `out/rev_b/ron/heirloom/_diag/`).

Checks (2026-11-02, work files in `.heirloom_work/rb_lotus/p2/`):
- GHDL analyse / elaborate / synth front end clean (`p2/check.sh`).
- `p2/sweep.py`: every new curve swept at the RTL widths, all rasters (xs 2 / 3 / 4). No wraps. K1 64..940
  (0.51..7.49 copies, 256 at 50 %), K2 0/256/1022 (v0.5's k/2 below 50 %), P12 gain 255 at 534 (= v0.5) .. 1010,
  stutter start / freeze / jump / tear widths, 23-bit stutter error, spread × Rings < 4096, cushion half-width +
  swell < 2^15. The S8 Pair parity puts the screen-centre eye at parity 0 at every K1 and raster.
- Default frame **bit-exact** with production v0.5.1: GHDL NTSC decimation 1, S11 Osc, defaults, 2 warm-up
  frames, raw 10-bit diff 0 of 349,920 pixels (`p2/prod_osc.npy` vs `p2/diag_osc.npy`).
- One extreme render (S8 Pair, K5 1000, K2 1000; `p2/cmp_def_ext.png`): the staircase and frozen runs cover
  the left third, the band zigzags deeply, the spread ripple scallops the cushion interiors and eyes; no wrap
  artifacts.
- Not simulated: the camera swell of the cushions (Video mode), K1 at its extremes, Rings above 3×, HD rasters.

## Reading of the photo (measured)
Work dir: `.heirloom_work/rb_lotus/` (`notes.md`, `fit_rows.py`, `rowfitB3.py`, `dfit.py`, `dlowrank.py`,
`holdtab.py`, `model.py`, `cfg_final.json`, `imodel.py`, `gen_hook.py`, `sheet.py`).
- **Layout.** Two copies across. Row shift-correlation gives a period of 943 px (2.04 copies across 1920).
  The copies agree to 1–3 levels for x ≥ 300; only the left copy's x < 300 (the line-start hold)
  differs.
  - Eye: an olive core diamond at (530 / 1467, 335) in nested dark-green / orange / green rings. The
    ring tip points right to x ≈ 930 / 1875 and the left side closes tight (~340 px from the tip). An
    X sits where the rings above the eye cross (x ≈ 1475, y ≈ 130), with V-bands along the top edge.
  - Flip-correlation axis 524 / 1467 for the eye and 532 / 1475 for the cushion. The eye is not
    mirror-symmetric more than ~150 px left of the axis: that is the seam zone.
  - Middle band y 460–630: yellow / green / dark-green stripes with V/Λ zigzag contours, period
    ~115 px, dark-green lines at y 465 / 520 / 565 / 603.
  - Cushion (one per copy): x 245–830 / 1185–1775, y 630–1037, barrel sides (half-width 274 at the
    top, 313 at y 770, 205 at the bottom). Inside, pink corners, a white X and a pink centre lens, set
    in bright green with dark-green / yellow V-streaks.
- **Field.** A separable H(p) + V(y) sum fails. A per-row affine of one horizontal waveform,
  F = a(y) T(p) + b(y), fits the top (rows 0–300, rms 6–15). T is a symmetric triangle peaking at
  x 1469 with slope 1/471.5 (skew tested: 1.0).
  - The eye's tight left side is a 2-D correction on a 48 px B-spline grid, fitted on the colour error.
    SVD rank 3 holds 96 % of it: G_k(y) h_k(p).
  - The middle band and the seam zone are not a per-row affine of anything (seam, spike and warp fits
    all fail). Per-block warps show frozen phase, jumps and fast sweeps tied to the staircase blocks.
- **Colourizer** (per-row regressions of R and G on cos / sin of the field).
  - R = R0 + RA cos 2πF, clipped: orange plateaus at 250, green troughs 55–90.
  - G = G0 + GA cos 2π(F + PG), with G0 ~160 at the top → 245 in the middle and GA ~20 at the top,
    30–65 in the cushion.
  - B is a soft key on the cushion (222). R is also keyed (to ~241), its edge 15–20 px outside B's.
  - Palette (k = 16): green (5,251,1) 13 %, orange (246,153,2) 12.6 %, pink (240,178,220) 9.8 %,
    olive (158,173,5) 8.1 %, white (236,224,216) 8 %.
- **Line-start hold and stutter.** The left edge is a smooth yellow-olive → green gradient up to a
  staircase of line starts S = 172 / 117 / 222 px, lasting 85 / 61 / 16 lines, period 162 lines.
  - Each start carries a ~20 px dark-red / pink / white / yellow flash, then frozen orange segments.
  - Between the eyes (x 960–1330, top half) the same blocks freeze the phase for 120 / 170 / 150 px,
    then jump and sweep after each seam.
  - The hold gradient colours were measured per 8 lines; they are not a function of the field (fit
    error 75–80).
- **Tears.** Bottom: per-row offsets of −19..+8 px in blocks of 6–20 lines. Middle band: ±30..60 px.
- **Mechanism.** A camera-free patch: a horizontal triangle oscillator through a per-line VCA plus a
  vertical-waveform offset into a cosine colourizer, an 8× triangle mixed in, a soft key for the
  cushions, and a line-start hold with sync stutter and horizontal tears.
- **Fit** (480 × 270 resample):
  - proto_final vs photo: mean |RGB| 19.7; 4×4 region means median 4.2, max 18.2; 16-px local luma
    std 15.8 (photo) vs 15.0; luma correlation 0.71.
  - The RTL (720p, black input, Speed still) vs photo: 21.9 / 4.7 / 17.6 / 15.2 / 0.67.

## What it does
1. **Per line (the engine).**
   - Interpolates the fitted 256-entry table (12 words: b, a, GA, G1, G2, z1..z3, R0, RA, G0, PG, kw,
     hold start / end colours) at the line's virtual row.
   - Folds the Rings gain into the coefficients.
   - Fills two ping-pong block-RAM tables for the next line: F over one period (64 entries of F and
     slope, flow folded in), and the palette (128 phases of R and G).
2. **The stutter**, per pixel. The beam freezes the phase at the line-start release S for L1, then
   snaps back. After each seam crossing it jumps, holds a plateau, then catches up. The block type
   comes from (row + roll) mod 162.
3. **Tears.** A 16-bit LFSR per block of 6–28 lines offsets the phase by 0 / 30 / 12 px (top / middle
   / bottom zones) × Stutter.
4. **The pixel path.** It reads F at the stuttered / torn / mirrored phase, interpolates it (1/8 entry),
   adds the etch (source luma, or Osc), and looks up the palette. It then keys the cushion with its own
   R and B alphas plus the green ground's boost, and composes the hold gradient and the 4-colour flash
   at the line starts.
5. **Motion.**
   - Speed streams the rings out of (or into) the eye cores.
   - The staircase crawls down one line every two fields.
   - The tears re-seed every 16 fields.

## Controls
| Control | Photo element | Range / notes |
|---|---|---|
| K1 Lobes | the eyes' copies across (the triangle oscillator's period); the cushions keep their own | 0.51..7.5 copies, exponential; centre = the photo's 2.04 |
| K2 Stutter | line-start staircase, frozen blocks, seam sweeps, tears | 0 = clean sync, 50 % = the photo, 100 % = 4× |
| K3 Bloom | the pink/white cushions | half-width × height 0..2×; 0 = green ground only, full = a pink band |
| K4 Speed | ring flow out of the eyes | bipolar; centre ±8 = still; (|v| − 8)² law |
| K5 Ripple | 8× zigzag band and the seam stripes; above 50 % a zigzag over the whole field | band 0..2×; spread 0 at ≤ 50 % .. ~1.5 rings p-p |
| K6 Etch | how far the source (or Osc) bends the contours; Video also swells the cushions | (Y − 64)/2 × gain, default 300 |
| S7 Palette | Photo / Night | Night exchanges red and blue (cyan eyes, violet cushions) |
| S8 Mirror | eye direction | Off / Pair: every second eye mirrored, neighbours face each other |
| S9 Contour | ring shading | Soft (the photo's cosine) / Hard two-level bands |
| S10 Fold | bottom half | rows below the band mirror the top: four eyes, no cushions |
| S11 Source | etch source | Video / Osc |
| P12 Rings | ring density about the eye core | gain 0.25 + 2.75 p² (+ (p − 534)/2 above 534), to 3.95×; 534 = the photo |

Presets: Photo (defaults), Broad Petals (P12 120), Moire Bloom (K3 760, P12 1000), Clean Sync (K2 0,
K4 700), Night Eyes (K1 760, S7 Night, S10 Fold), Facing Eyes (K1 380, S8 Pair), Ripple Field (K5 960,
P12 700), Sync Storm (K1 820, K2 1000, K4 820).

## Video
`program_type = "processing"`: the input is used only as the etch.
- Its luma, (Y − 64)/2 with a 2-px average, times Etch, is added to the field phase, so the picture
  bends the contours (a face appears etched into the rings).
- With S11 Osc, an internal triangle (one per lobe, drifting sideways) etches instead.
- With no source the picture is the photo's.

## Engine
- **The engine.** One shared 13 × 10 shift-add multiplier, three pipeline stages: operands set in
  state s are read in state s + 4.
  - Phase A, states 1..49: row, table position, the 12 words at entries j and j + 1, interpolated
    with an 8-bit fraction. Then the Rings gain into the coefficients and the constant term.
  - F row: 64 entries × 6 products (v0.6.0: the sixth is the spread ripple, (X − 128) × K5's spread × Rings).
  - Palette: 128 phases × 2 products.
  - Phase C: about 20 clocks after the line leaves stage 12. It computes the cushion, stutter, tear
    and hold scalars.
- **Per field (v0.6.0 additions).** K1's piecewise-linear exp2 (replaces the 64-entry table), K2's
  two-slope curve, K5's spread (0.75 (k − 512)), P12's extra (p − 534)/2; serial-multiplier slots for the
  cushion oscillator's start phase (photo period × half-width) and spread × Rings; the line-start copy
  parity for S8 Pair (bit 20 of centre phase − half-width phase).
- **Per pixel (v0.6.0 additions).** A second 20-bit phase accumulator for the cushions at the photo's
  period; a copy-parity bit toggled at each eye seam (S8 Pair mirrors the odd copies); in Video mode the
  luma × {0, 2, 4, 6} (Etch's top two gain bits) widens the cushion key's horizontal half-width.
- **Tables.** lotus.py carries the fitted tables (pure Python; emits lotus_pkg.vhd).
- **Geometry.** Virtual 1080-line coordinates from the timing ID: 720p and 1080p are identical; SD
  shows the centre 1440 of the 1920 virtual columns.
- **Output.** Generated avid. Output conditioning before the blanking gate: chroma
  [1 2 2 2 2 2 2 2 1]/16 through an EBR ring, luma [1 2 1], luma ×13/16 (white 940 → ~777).
  Latency 20 clocks in every mode.
- **Colour convention.** Generated colours are in the hardware convention (U carries Cr).

## Status
- **v0.6.0 diag (2026-11-02): see [Phase 2](#phase-2-v060).**
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 86.82, SD Analog s1 84.57, HD HDMI s1 85.78, SD HDMI s1 85.57, HD Dual s1 86.06, SD Dual s1 88.32 MHz; 7122-7164 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
v0.5.0 built 2026-10-27, NOT flashed.
- **Router:** ROUTER1. router2 reaches 71 MHz on HD Analog, so rebuild with
  `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh lotus`.
- **Seeds:** all seed 1 except HD Dual seed 2 (seed 1 stalled the router).
- **Fmax:** HD Analog 84.30, HD HDMI 88.81, HD Dual 87.33, SD Analog 84.96, SD HDMI 85.98, SD Dual
  85.36 MHz.
- **Resources:** LC 7128–7174 / 7680 (93.4 % at most), 22 / 32 EBR.
- **Output:** `out/rev_b/ron/heirloom/lotus.vmprog` (476431 B, unsigned).

## Deviations
- The seam zone (stripes and frozen blocks between the eyes) and the middle zigzag band are designed
  rather than fitted.
- The magenta frozen slab in the top-left block (rows 146–167) and the small green flags at the
  cushion sides are not reproduced.
- The eye correction is fitted at rank 3; the RTL evaluates the first two modes.
- Whites are dimmer than the photo's (236,224,216 → ~210,208,192): Y capped at 940 then ×13/16, the
  HD HDMI bar fix.
- Tears are pseudo-random: the photo's zones and amplitudes, not its exact tears.
- The default Speed animates the ring phase; the exact still look is K4 at centre.
- SD shows the centre 1440 virtual columns at 4:3 (round stays round): 1.53 copies across instead of
  2.04. 720p matches 1080p.

## Notes
- The VHDL header comments were tidied after the final build; the logic is unchanged.
- `s_oxl` / `s_oys` are a dead per-line Osc component that synthesis prunes; safe to remove at the next
  rebuild.

## Open questions for Ron (2026-11-02 batch, asked in the hand-over report)

1. Keep S8 Pair, or go back to all eyes mirrored?
2. Should the cushions get their own control?

**Watch on hardware:**
- 96 % LC: little headroom for a second round.
- No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.

**Next step:** wait for Ron's preview notes. Rebuild the diag with `.heirloom_work/diag_build.sh lotus` (scoped clean + shared build queue). On approval: `./heirloom_diag.sh merge lotus`, `./build_heirloom.sh lotus`, update the production SUMMARY / README / PHASE2 row, commit that program's files.

## K6 top +25 % and default 100 % (2026-11-15)

**Bend type**: K6 drives two element effects, the contour etch and (in Video) the camera swell of the cushions. Both were raised 25 %. The K6 default is now 100 % (orchestrator).
- The change is the same as in the main program: f_e8 x1.25 (9 bits), and f_e8g in 9 bits saturating at 319 (was an 8-bit saturation at 255).
- The pixel etch code is 5 bits, 0 .. 19 (was 0 .. 15), with bit 4 as a mux.
- The swell gets a third step, bit 8: 8 x lum (was at most 6 x).
- Not rendered; checked by GHDL analysis and the build.
- Build: v0.6.2, router1, all six on seed 1; worst HD 82.92 MHz (HD Analog 87.27, HD HDMI 83.89, HD Dual 82.92); 7,437 / 7,680 LC. `.kinetic_work/k6_builds.txt`: diag lotus OK router1 worst-HD 82.92 LC 7437.
