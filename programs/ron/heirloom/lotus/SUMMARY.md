# Lotus — twin contour eyes over pink cushions, torn by a stuttering sync

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Rebuilt from the capture
`photos/WIN_20230515_23_23_36_Pro.jpg`: two orange/green contour "eyes" over a yellow-green zigzag
band, a pink/white cushion keyed into the bright green bottom of each copy, and a stuttering line-start
staircase at the left edge. The v0.1 version (an abstract colourized field) is replaced. Presets are
hand-written placeholders.

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
| K1 Lobes | copies across (the triangle oscillator's period) | 0.7..5.8 copies; centre = the photo's 2.04 |
| K2 Stutter | line-start staircase, frozen blocks, seam sweeps, tears | 0 = clean sync, 0..2× the photo |
| K3 Bloom | the pink/white cushions | half-width × height 0..2×; 0 = green ground only, full = a pink band |
| K4 Speed | ring flow out of the eyes | bipolar; centre ±8 = still; (|v| − 8)² law |
| K5 Ripple | 8× zigzag band and the seam stripes | 0..2× |
| K6 Etch | how far the source (or Osc) bends the contours | (Y − 64)/2 × gain, default 300 |
| S7 Palette | Photo / Night | Night exchanges red and blue (cyan eyes, violet cushions) |
| S8 Mirror | eye direction | the ring tip points left |
| S9 Contour | ring shading | Soft (the photo's cosine) / Hard two-level bands |
| S10 Fold | bottom half | rows below the band mirror the top: four eyes, no cushions |
| S11 Source | etch source | Video / Osc |
| P12 Rings | ring density about the eye core | gain 0.25 + 2.75 p²; 534 = the photo |

Presets: Lotus (defaults), Broad Petals (P12 120), Moire Bloom (K3 760, P12 1000), Clean Sync (K2 0,
K4 700), Night Eyes (K1 760, S7 Night, S10 Fold).

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
  - F row: 64 entries × 5 products.
  - Palette: 128 phases × 2 products.
  - Phase C: about 20 clocks after the line leaves stage 12. It computes the cushion, stutter, tear
    and hold scalars.
- **Tables.** lotus.py carries the fitted tables (pure Python; emits lotus_pkg.vhd).
- **Geometry.** Virtual 1080-line coordinates from the timing ID: 720p and 1080p are identical; SD
  shows the centre 1440 of the 1920 virtual columns.
- **Output.** Generated avid. Output conditioning before the blanking gate: chroma
  [1 2 2 2 2 2 2 2 1]/16 through an EBR ring, luma [1 2 1], luma ×13/16 (white 940 → ~777).
  Latency 20 clocks in every mode.
- **Colour convention.** Generated colours are in the hardware convention (U carries Cr).

## Status
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
