# Seismic — a chirp across five ribbons, torn into staircase strips

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/seismic.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Seismic Diag`, `out/rev_b/ron/_diag/seismic_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/seismic/`) is untouched. Phase-2 working files:
`.heirloom_work/rb_seismic/p2/` (imodel3.py = the integer model on the diag hook, sweep.py, check.sh, psim.sh, renders).

## Phase 2 (v0.6.0)
No notes from Ron for seismic (free hand, 2026-11-03 batch). The v0.5 controls were all live, so the round turns the
one global scale (K1 Rows, a Y zoom of the ribbons) into per-ribbon heights, widens the two element knobs that
stopped short (Sweep, Width), and gives the strips and the Shuffle switch structural moves. Almost everything is
microcode: the engine's field and line programs compute the new terms, so the fabric only gained a strip-height
register, a wrapping strip counter and wider carrier registers.

| Control | Was (v0.5) | Now (v0.6.0) | Full sweep at 1080p |
|---|---|---|---|
| **K1** | Rows: ribbon count 0.5x .. 2x (a vertical zoom of the ribbons) | **Heights** (renamed), bipolar, count fixed at the photo's 5.3, photo at 512 (bit-exact) | **Left = cascade:** the ribbon frequency ramps down the frame, 1.75x at the top (~117-line ribbons, crowded) to 0.25x at the bottom (one ~820-line band). **Right = lens:** the middle ribbons swell to ~680 lines (0.3x), the top and bottom ones crowd to ~78 lines (2.6x). Ribbon width scales with each ribbon, and K4's ribbon roll now carries the ribbons through the warp (they grow and shrink as they travel) |
| **K2** | Sweep: chirp rate 0.42x .. 2.34x (two slopes) | **0.25x .. 4x** on (1 + u)^2 (0.56x at 25 %, 2.25x at 75 %), photo at 512 (bit-exact) | 0 = wide stripes, the fold's period ~170 px (about six stripes across the window) and the gap colours stretched into long bands; 1023 = ~11-px stripes, and the gap colour cycle compresses into the left of the window (the rest of the gap layer goes black). SD aliases above ~3x |
| **K5** | Width: ribbon width 0.55x .. 1.45x | **0.25x .. 2.25x** on (1 + u)^2, photo at 512 (bit-exact) | 0 = thin ~25-line threads on black gaps; 1023 = the ribbons swallow the gaps (they fill ~95 % of each 205-line period), a pale cyan / yellow wash with only seams of black |
| **P12** | Quake: 0 calm .. 50 % photo .. 100 % triple shifts + V-hold slips | **+ the strips thin above 50 %:** 45 lines (24 strips) at 50 % .. 15 lines (72 strips) at 100 %. Below 50 % as before | The climax now shatters the frame into 72 slivers, each shifted and slipping; the 8-strip staircase repeats every 120 lines instead of 361 |
| **S10** | Steps: Staircase / Shuffle (hashed strip phases) | **Shuffle also tears the ribbons:** each strip's ribbons jump up or down by its own hashed offset (up to ~49 lines; more with P12's slips above 50 %) | A look-flip: the ribbons break into misaligned bricks at every seam (v0.5's Shuffle only re-ordered the stripe shifts) |

Unchanged: K3 Lobes, K4 Speed, K6 Melt, S7 Blocks, S8 Sweep Shape, S9 Palette, S11 Source.

**Presets:** Photo (= the defaults), Calm, Aftershock and Blue Ribbons as before (Aftershock now also shatters the
strips; Blue Ribbons' K1 700 is now a partial lens). New: **Perspective** (K1 0 cascade, wider ribbons),
**Lens** (K1 1023, slower sweep), **Shatter** (Shuffle, P12 900, faster) and **Threads** (K5 0, K2 4x, K3 max).

**Engine changes.**
- **Field program:** KV is now the photo's constant; K1 gives CJ (cascade, K1 < 512: 0 .. -11648) and CK (lens,
  K1 > 512: 0 .. 22931); K2 and K5 are squared, (512 + u)^2 / 64 with u = d (up) or d / 2 (down) for K2 and d / 2
  for K5, so 512 lands on 4096 exactly; S1H = S1V / 2 is stored for the line program's multiply, whose 19-bit B
  operand would overflow at 4x; SH = 720 − (P12 − 512)·61/64 above 50 % (Q4 lines) goes to the new OUT 23; SHK =
  25600 when S10 is Shuffle (new input port 10).
- **Line program:** DM = the line − 540 (Q4); G = CJ·DM²/2^28 + CK·(DM³/8192² − DM)/2^15 is added to the line
  before the ribbon frequency KV (four more multiplies); the per-strip slip is (PQ + SHK)·hash. 589 clocks with the
  copy (was 489; SD line 858). 249 of the 256 microcode words.
- **Strips:** the seam step is the field register f_sh (was the constant 720), the first seam f_sh + 158, and the
  strip index wraps mod 24 instead of stopping at 23 (the staircase is 8 strips, so it continues seamlessly).
- **Carrier widths:** the linear frequency b_L is 26 bits (it reached 8.46 M, past 24 bits, at SD with K2 at 4x and
  S8 Ramp), the Q20 frequency registers 22 bits, and the phase b_psi 29 bits; past ±128 cycles the pixel path
  holds the colourizer's cycle count at ±96 (black) while the stripe fraction stays exact.
- Pixel path depth and C_LAT 24 are unchanged.

**Build** (router1, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh seismic`, 2026-11-03): all six on
seed 1. HD Analog 97.41, SD Analog 89.69, HD HDMI 92.31, SD HDMI 93.13, HD Dual 92.25, SD Dual 88.83 MHz.
7,074–7,084 / 7,680 LC (92 %, +~50 over v0.5.1), 11 / 32 EBR (unchanged). iceunpack OK ×6; the packed config
carries the new labels and presets. No pin-skew screen.

**Checks.**
- GHDL analyse / elaborate / synth front end: clean.
- Integer model (`p2/imodel3.py`: imodel2 running the diag microcode in the bit-exact engine emulator): the default
  1080p frame is pixel-identical to the v0.5.1 model (0 px differ). Extremes rendered (`p2/sheet_model1.png`): K1 0 /
  1023, K2 0 / 1023, K5 0 / 1023, P12 1023, S10 Shuffle, P12 1023 + Shuffle.
- `p2/sweep.py`: field and line programs over K1 × K2 × K5 × P12 × S10 grids in 1080p / 720p / NTSC / PAL, all
  lines: no multiplier operand overflow, every OUT register and carrier DDA in range at the new widths, line
  program ≤ 589 clocks. It found the two overflows fixed above (b_L, b_psi).
- Fitted gap colourizer (stretched up to 4x by K2 at 0 %): no isolated second-difference spike (the one large
  entry is the monotone cyan onset).
- GHDL default frame (720p60 decimation 1, Osc, 2 warm-up frames, `p2/rtl_def_osc.png`): pixel-identical to the
  v0.5.0 baseline `rtl_osc_final.png` for x ≥ 8 (0 px differ); only columns 4–7 differ, the v0.5.1 line-start
  resync. No GHDL render of the new controls: the model runs the same microcode bit-exactly, but the RTL strip
  counter (f_sh step, mod-24 wrap) and the Shuffle port are checked only by the model and by reading.

## v0.5 summary (production)
**v0.5.0 (2026-10-27): rebuilt from the photo; built 6/6 (router1, all seed 1), GHDL-rendered, NOT flashed.** Heirloom #44,
from `photos/WIN_20230610_00_10_54_Pro.jpg`. Replaces v0.1.0 (smeared picture rows alternating with lens rows), which Ron
judged too abstract. Presets are hand-written placeholders.

### Reading of the photo (measured)
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

### What it does
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

### Controls
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

### Video
- **Video:** camera luma (tapped from data_in) minus its running mean × Melt is added to the carrier phase and to the
  vertical phase: stripes and ribbons bend around the picture; a flat input leaves the photo untouched.
- **Osc:** two drifting triangles (700 px across, 520 lines down) drive the same path, so Melt stays live.

### Engine
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

### Fit
The first build was 10,255 LC (133 %). Standalone yosys ablations showed the pixel multipliers (−1719 LUT), the
hardwired line/field sequencer (−1861 LUT, −896 FF), the carrier pipeline and YUV/conditioning as the hogs, and the
128 × 8 colourizer ROMs synthesised as logic. Restructured: the microcoded engine above (replaces the 60-state
sequencer and the next-line double buffer); 6-bit keys/masks/levels in the pixel multiplies and one multiply for R;
carrier frequency path in Q20 with a fold DDA; truncated ψ / E / phv in the pixel path; cusp darkening as a min;
luma-difference RGB → YUV; chroma and sync delays in block-RAM rings; colourizer padded to 256 (EBR). Also fixed a
segment-walker bug in the pre-fit RTL (entry 1 of every strip skipped → black gap rows). Result 7,004–7,024 LC.

### Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 91.31, SD Analog s1 91.99, HD HDMI s1 94.07, SD HDMI s1 96.46, HD Dual s1 98.79, SD Dual s1 95.11 MHz; 7028-7034 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- v0.5.0 built 2026-10-27: router1 (NEXTPNR_ROUTER="--router router1"), all six configs seed 1. Post-route Fmax:
  HD Analog 91.99, HD HDMI 97.71, HD Dual 97.12, SD Analog 95.56, SD HDMI 96.07, SD Dual 94.98 MHz. 7,006–7,024 /
  7,680 LC (91 %), 11/32 EBR. router2 stalled on this netlist in a trial; use router1.
- GHDL renders (720p60 decimation 1) match the integer model (imodel2.py): Video 4×4 error 1.6. Osc and Video
  renders: `.heirloom_work/rb_seismic/cmp_final.png`. Motion checked over frames 3 → 10. Not flashed.

### Deviations from the photo
- Hardware black (photo black ~15).
- Gap-row B less blocky than the photo; right-zone ribbon colours approximate (post-fold carrier rms 0.3–0.5 cycle).
- Gap lobes a little softer/paler than the photo's; cusps over dim gap colour slightly lighter (min instead of product).
- The first ~4 px of each line repeat the line-start phase.
- K5 does not scale edge softness.
- Fit rounding: key/mask products at 64 levels (R and B up to ~1.5 % darker at full keys), Melt in 64 steps, YUV within
  ±9 codes of BT.601, strip shift to 1/16 px, chirp rate ~0.2 % low for K2 below 50 %.
- SD 4:3: the same picture compressed horizontally (nothing round).

## Open questions for Ron
1. **K1 Heights:** is cascade (left) / lens (right) the right pair, or would you rather have the ribbon count back
   (on K1, with the warp elsewhere)? The count is fixed at the photo's 5.3 in this diag.
2. **Strips:** the strip height rides on P12 (thinner as the quake rises). Should it be its own control instead,
   e.g. in place of S7 Blocks or as a second function on K3?
3. **S10 Shuffle:** the ribbon tear is ±49 lines. Bolder (a whole half-ribbon), or keep v0.5's plain Shuffle and
   put the tear on P12 only?
4. **K2 Sweep at 4x:** the gap colours compress into the left of the window and the rest of the gap layer goes
   black. Keep, or stop at ~3x?

## Hardware watch list
- **Renamed control:** K1 Rows → **Heights** (the diag and production labels differ). K2 Sweep and K5 Width now
  display −100 .. 100 with the photo at 0 (v0.5 showed 42..234 % and 55..145 %).
- **SD:** K2 above ~3x aliases (stripes under 4 real px) and P12 at 100 % gives ~3-line strips per field on 480i.
- Timing: see the build line.
- Nothing was cut for fit.

## Next step
On approval: `./heirloom_diag.sh merge seismic`, then repack the previewed diag bitstreams under the production TOML
instead of re-routing (copy `build/programs/ron/seismic_diag/rev_b/bitstreams/*.bin` into the production build dir,
regenerate `program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`), update
SUMMARY / README / PHASE2 and commit seismic's files only.

## K6 top +25 % and default 100 % (2026-11-15)

The same change as production seismic (bend program): the per-field melt code `f_m6` = (K6 + K6 / 4) / 16, 7 bits, 0 .. 79; p2_pm 15 bits; p3_m1 takes a slice of p2_pm / 4 (mod 4096). K6 default 1023, v0.6.2. No render.
- **Build** v0.6.2 (k6build diag), router1, all six configs seed 1. HD Analog 92.40, HD HDMI 92.95, HD Dual 92.77 MHz (worst HD 92.40). 7109–7126 / 7680 LC, 11 / 32 EBR. out/rev_b/ron/_diag/seismic_diag_v0.6.2.vmprog. Not flashed.
