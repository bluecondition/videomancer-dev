# Seismic — a chirp across five ribbons, torn into staircase strips

**v0.5.0 (2026-10-27): rebuilt from the photo; built 6/6 (router1, all seed 1), GHDL-rendered, NOT flashed.** Heirloom #44,
from `photos/WIN_20230610_00_10_54_Pro.jpg`. Replaces v0.1.0 (smeared picture rows alternating with lens rows), which Ron
judged too abstract. Presets are hand-written placeholders.

## Reading of the photo (measured)
Work: `.heirloom_work/rb_seismic/` (`notes.md` has every number).
- **Ribbons.** Five G tubes centred y = 127 + 205.25 n (G = 128 half-width 51 − 1.7 m lines, soft over ~36 lines); the
  vertical oscillator is a triangle clipped at both ends (gap plateau |v| > ~80 is vertically flat).
- **Carrier (R).** One FM horizontal oscillator: per-strip R stripe centres fit f(u) = 2.6e-5 (u − 280) cycle/px
  (period 120 px at x 520 → 42–46 px at x 1050–1290), folded at u = 1180 + 20 m, falling at 2.1×, regrowing ×0.5;
  left of u 527 the ramp origin is 340 − 10 m with a 0.0018 floor. Middle-zone rms phase error 0.05–0.1 cycle.
- **Strips.** 24 seams every 45.15 lines (first at 9.85). Each strip is the carrier rigidly shifted by
  D(m) = −122.6 + 12.45 m + 1.25 m² px (rms 0.002–0.03 cycle), m = (k + 2) mod 8: an 8-strip staircase resetting at
  y 282, 642, 1004. Fold, left ramp and ribbon width move with m (time samples of the patch).
- **R key.** Off where |ψ − cR| < hR: hR = 0.24 in the core shrinking to 0 at W 0.66; cR leans 0.42 → 0.22 onto the
  G cusp. R level 0.78.
- **B envelope.** Fold of ρ = sqrt((x − 960)² + (0.6 (y − 1650))²) about 1085 (gains 0.8 in / 0.9 out; mean abs err 15).
- **Gap layer.** Window x ≈ 390–1480 (hard edges where the plateau meets the key). Plateau colour is a 1-D function of
  the unwrapped carrier phase (fit_tab1d.py): cyan (ψ −6.5) → white → yellow (−3.5..3) → lime → mint → cyan (8–9) →
  yellow-green (10–11) → black (12+). One dark magenta cusp per cycle at ψ 0.10, circular lobes from W 0.56.
- **Blocks.** Per-strip segment table (fit_seg.py/mkseg.py): edges recur at x ≈ 390, 495, 620, 1105, 1290, 1480, 1590,
  1778, with 2-bit R/G/B levels (black, cyan, yellow and navy blocks).
- **Palette (k = 14).** Black 20 %, cyan (17,231,242) 13 %, pale cyan-white 9 %, light cyan 8 %, lavender 7 %, yellow
  6 %, pale yellow 6 %, mint 6 %, blue 5 %, green 4 %, ...
- **Fit.** proto_final 4×4 region-mean error 8.75 (max 17.9), local contrast 1.10; integer model of the shipped RTL
  (imodel2.py) 8.55; GHDL render (720p Osc, upscaled) 9.54 (max 17.1), local contrast 1.01.

## What it does
1. Per line (microcoded engine, the line before): strip k, phase μ (staircase or shuffle, rolled by Speed),
   q·(μ − 3.5) → shift D, fold, left ramp, ribbon width; P12 above 50 % adds per-strip V-hold slips.
2. Carrier: Q24 linear frequency L, fold term g as its own Q28 DDA, three-segment law in Q20 (left: max(L + cA,
   floor); middle: L; right: g ≥ 0 ? g : −g/2, floored); phase ψ from a closed-form anchor
   ψ0 = ψA − FL·(D + xl + 69) − S1/2·((527 − xl)² − 69²).
3. Vertical phase per line → W (0 ribbon centre .. 255 gap centre); Melt adds the source luma into ψ and W.
4. Channels: R = carrier key × max(0.78 ribbon mask, gap colour in the window); G = max(ribbon key, gap lobes),
   darkened by the cusp; B = max(Benv × mask, blend of Benv and the gap colour × level). Benv from a squared-distance
   DDA (|ρ² − R0²| / 2R0, no square root).
5. Output: RGB → 10-bit YUV (luma-difference form, U = Cr, V = Cb, luma ×16/13), shell conditioning, blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| **P12 Quake** | the staircase strips | 0 = calm aligned strips (no blocks); 50 % = the photo; 100 % = triple shifts + per-strip V-hold slips. Glided. |
| K1 Rows | the five ribbons | 0.5–2× ribbons (5.3 at 512). Glided. |
| K2 Sweep | the chirp (stripe density) | 0.42–2.34× the frequency ramp. Glided. |
| K3 Lobes | the gap lobes / blocks | 0 = black gaps; 512 = the photo; max = the window spans the frame. Glided. |
| K4 Speed | motion | Bipolar, still ±8: the staircase rolls (about 1 strip/s at the default 600, 6/s at max), stripes drift, ribbons roll. |
| K5 Width | ribbon width | 0.55–1.45×. Glided. |
| K6 Melt | camera / Osc into the patch | bends the stripes (≤ 2 cycles) and the ribbons (≤ 0.585 cycle); 64 steps. Default 384. |
| S7 Blocks | hard-edged gap blocks | On (photo) / Off (plain window) |
| S8 Sweep Shape | the fold of the chirp | Fold (photo) / Ramp (frequency keeps rising) |
| S9 Palette | palette | Photo / Swap (R ↔ B) |
| S10 Steps | strip order | Staircase (photo) / Shuffle (hashed strip phases) |
| S11 Source | camera input | Video (default) / Osc |

## Video
- **Video:** camera luma (tapped from data_in) minus its running mean × Melt is added to the carrier phase and to the
  vertical phase: stripes and ribbons bend around the picture; a flat input leaves the photo untouched.
- **Osc:** two drifting triangles (700 px across, 520 lines down) drive the same path, so Melt stays live.

## Engine
- Microcoded engine: one 32-bit accumulator, a 64 × 32 register file and the 256 × 32 microcode in block RAM, a
  4-bit barrel shifter, one Booth radix-2 serial multiplier (25 × 19, ~22 clocks); one instruction per clock (an RF
  register stored by instruction i is read from i + 4 on; the assembler inserts NOPs). seismic.py assembles FIELD /
  LINE / LCOMP (213 instructions, 1 NOP) into seismic_pkg.vhd (C_UC).
- Field program (vblank zone clock 64; 635 clocks, 25 multiplies): row/sweep curves, S1, floors, ψA, Benv DDA constants,
  fold step, gap levels × Lobes, widen bands, ribbon roll; then LCOMP computes line 0 and LINE copies it.
- Line program (11 clocks after the generated avid falls; 489 clocks incl. the copy, 18 multiplies, < SD 858): a copy
  prologue moves the next line's values (computed into RF during the line before) into the beam registers in hblank,
  then LCOMP computes the line after.
- Beam: carrier L / g DDAs + 3-stage Q20 frequency pipeline (ψ starts when the first valid F arrives), squared-distance
  DDA, Osc phase, segment walker (prefetches the next line's first entries in hblank).
- Pixel path stages 1–16 (ROM reads with fabric re-registers; R key over stages 6–9, cusp over 9–11), conditioning
  17–24 (chroma 8-back from a block-RAM ring; syncs delayed in a block-RAM ring): C_LAT 24 in every mode.
- ROMs: C_TABR/G/B 256 × 8, C_SEG 256 × 16, C_HD 256 × 8, C_UC 256 × 32 (EBR); C_SHUF, C_HSH 32-entry (logic).

## Fit
The first build was 10,255 LC (133 %). Standalone yosys ablations showed the pixel multipliers (−1719 LUT), the
hardwired line/field sequencer (−1861 LUT, −896 FF), the carrier pipeline and YUV/conditioning as the hogs, and the
128 × 8 colourizer ROMs synthesised as logic. Restructured: the microcoded engine above (replaces the 60-state
sequencer and the next-line double buffer); 6-bit keys/masks/levels in the pixel multiplies and one multiply for R;
carrier frequency path in Q20 with a fold DDA; truncated ψ / E / phv in the pixel path; cusp darkening as a min;
luma-difference RGB → YUV; chroma and sync delays in block-RAM rings; colourizer padded to 256 (EBR). Also fixed a
segment-walker bug in the pre-fit RTL (entry 1 of every strip skipped → black gap rows). Result 7,004–7,024 LC.

## Status
- v0.5.0 built 2026-10-27: router1 (NEXTPNR_ROUTER="--router router1"), all six configs seed 1. Post-route Fmax:
  HD Analog 91.99, HD HDMI 97.71, HD Dual 97.12, SD Analog 95.56, SD HDMI 96.07, SD Dual 94.98 MHz. 7,006–7,024 /
  7,680 LC (91 %), 11/32 EBR. router2 stalled on this netlist in a trial; use router1.
- GHDL renders (720p60 decimation 1) match the integer model (imodel2.py): Video 4×4 error 1.6. Osc and Video
  renders: `.heirloom_work/rb_seismic/cmp_final.png`. Motion checked over frames 3 → 10. Not flashed.

## Deviations from the photo
- Hardware black (photo black ~15).
- Gap-row B less blocky than the photo; right-zone ribbon colours approximate (post-fold carrier rms 0.3–0.5 cycle).
- Gap lobes a little softer/paler than the photo's; cusps over dim gap colour slightly lighter (min instead of product).
- The first ~4 px of each line repeat the line-start phase.
- K5 does not scale edge softness.
- Fit rounding: key/mask products at 64 levels (R and B up to ~1.5 % darker at full keys), Melt in 64 steps, YUV within
  ±9 codes of BT.601, strip shift to 1/16 px, chirp rate ~0.2 % low for K2 below 50 %.
- SD 4:3: the same picture compressed horizontally (nothing round).
