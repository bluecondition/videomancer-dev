# Jukebox — a wall of rainbow capsules

**v0.6.1 (2026-11-03): phase 2 final. Ron approved the diag after flashing it, and it is merged into
production** (`out/rev_b/ron/heirloom/jukebox.vmprog`; see [Production build](#production-build-v061)). The
diag went through two rounds:
- **v0.6.0:** free-hand phase-2 rework, Ron's first flash;
- **v0.6.1:** his notes on that flash (see [v0.6.1](#v061-ron-s-first-flash-notes)), then approved.

Presets are still hand-written placeholders; capture real ones with `./capture_preset.sh`.

## v0.6.1: Ron's first-flash notes

Ron flashed v0.6.0 (2026-11-02) and reported:
1. Thin vertical lines spread out over the image, multicoloured, sharing the image's colours, a little wider
   than one pixel.
2. Wider range in both directions on K1.
3. Wider range on K2.
4. K3 Nose further than 100 %, maybe up to 200 %.

| Note | Cause / change |
|---|---|
| Lines | **One bad column-table entry.** In the A columns, E (the band offset) at position 250 of 256 read −0.73 folds between −3.97 and +0.02. The photo fit had linearly interpolated across a 4-fold unwrap there. That shifted the bands at one pixel, 5 px before every A → B cut: 5 full-height rainbow lines at 1080p (x ≈ 86, 506, 926, 1346, 1766), widened to ~3 px by the luma [1 2 1] and to 9 px in chroma by the output filter. It was in v0.5 too (never flashed), and is visible in the v0.6.0 renders `p2/x1_*`, `x2_*` as a pale line left of a cut. Fixed in the hook (`T_E[1][250] = −1017` ≡ the neighbours mod 2 folds) and in `tables.json`. Checked with the integer model: the fix changes only those 5 columns. A mod-512 second-difference scan of all seven tables finds no other single-entry spike. |
| K1 Widths | **Lens and Fan both run to L = 1.5** (were 0.81 / 1.0), linear in the knob, so the old extremes are now at ~54 % (Lens) and ~67 % (Fan) of each half. Past L = 1 the step goes negative across part of the screen and the columns run backwards there: the wall folds into mirrored columns around a stationary crease. **Lens 100 %:** the centre column is reversed at 2× width, creases at ±320 px from the centre, edge columns ~52 px. **Fan 100 %:** the left ~650 px is one giant reversed column, the crease sits at x ≈ 650 and the columns shrink to ~52 px at the right edge. The step is signed (Q12, 32 bits; the beam adds it sign-extended). 9.1 columns still span the screen and the centre stays put (Fan within 0.05 columns). |
| K2 Heights | **Cascade:** the step falls from ×(1 + 9c) at the top (was 1 + 4c) to ×(1 − 0.8c) at the bottom (was 1 − 0.75c). At 100 %, 1080p shows ~9 capsules growing from 57 lines at the top to 183, then a partial one (v0.6.0: ~5.4, from 125 lines). **Alternate:** the thin kind goes to ×20.4 (was ×10.7) and the tall kind to ×1.5 (was ×1.25). At 100 %, 26-line strips sit between 341-line capsules (v0.6.0: 49 / 414). The thin-kind factor and B step are 13 / 26 bits wide now. |
| K3 Nose | **Readout 0–200 %, the photo at 100 % (512).** The lower half is unchanged (0.31× at 0 %). The upper half now runs to 21.3× at 200 % (was 10.7× at the old 100 %, now at ~175 %). The stretch is Q8 (10 bits, was Q6 8 bits), so the long end moves in ~10 % steps instead of jumping between s = 3 and 4. Beyond ~11× each column shows only the start of the nose's decay, spread across the whole column. The curl gets longer and gentler, a sweep rather than a tight ⊂. |
| (new) E interpolation | **Columns no longer step when magnified.** The 256-entry tables stepped visibly where K1 or K3 spread one entry over many pixels: v0.6.0's Lens cap existed because of this (`p2/x1_*`). T0's spare bits now hold E's slope to the next entry (7 bits; the largest is 62). The column position carries 4 fraction bits (8.4 from the phase, or from the nose product in the tail: d in 8.2 × s in Q8), and E is interpolated at stage 8. a, c, o, r, bo and bg are still read at whole positions, so blocky colour steps can show where a crease or long nose lands in the reset strip (the first ~45 px of a column). |

**Fit.** The interpolation, wider nose path and signed DDA cost +206 LUT in standalone yosys. Three block-RAM delay rings on one write pointer pay it back:
- r / bo from stage 7 to 15 (read 6 back + 2 registers);
- bb from 8 to 15 (read 5 back);
- the hsync / vsync / field taps (read 26 back + 2 registers = the 28-clock latency).

That is −190 FF net for +3 EBR, at 21 EBR. The latency (28) and the generated-avid taps are unchanged.

**Presets.** Cylinder now uses K1 791 (the old full Lens, no fold) and Perspective uses K1 170 / K2 689 (the old full Fan and the old Cascade look). The others are unchanged and read a little stronger under the new mappings.

**Checks (2026-11-02):**
- GHDL front end (analyse, elaborate, synth): clean.
- Sweep test `p2/sweep061.py` at the RTL widths covered every K1 value × 3 width classes, K2 × 7 timings, all K3 LUT values × all 8.4 positions, and the E slope packing. 0 failures.
- GHDL render of the default frame (720p dec 1, Osc, `p2/def4_osc.png`): the line at x = 1181 is gone (curvature 41.5 → 0.7). At 720p only that one A column samples position 250; at 1080p all five do. The rest of the frame differs from v0.6.0 by 3.5 levels / pixel on average, the interpolation smoothing the nose. The look is unchanged.
- **Not checked:** the folds and the long noses (sweep arithmetic only), SD and 1080 renders, motion.

**Diag build** (router2, the builder default, 2026-11-02; this is the build Ron flashed and approved):

| Config | Seed | Fmax (MHz) |
|---|---|---|
| HD Analog | 1 | 83.14 |
| SD Analog | 1 | 83.44 |
| HD HDMI | 1 | 86.57 |
| SD HDMI | 1 | 79.83 |
| HD Dual | 1 | 85.53 |
| SD Dual | 1 | 73.26 |

7,303–7,333 / 7,680 LC (95.5 %), 21 / 32 EBR, iceunpack OK ×6. No pin-skew screen.

### Production build (v0.6.1)

2026-11-03, after `./heirloom_diag.sh merge jukebox`: a scoped clean, then `./build_heirloom.sh jukebox`. Router2, all six configs on seed 1:
- HD Analog 83.14, SD Analog 83.44, HD HDMI 86.57, SD HDMI 79.83, HD Dual 85.53, SD Dual 73.26 MHz;
- 7,303–7,333 LC, 21 EBR, iceunpack OK ×6.

**All six bitstreams are byte-identical to the approved diag's.** Only the comments changed in the VHDL, so the netlist and the routing are the same, and production is exactly what Ron flashed. The program id and name are in the config block, not the bitstream.

**Watch on hardware:**
- the lines should be gone at every setting;
- K1's last third is the new fold, and blocky colour steps can show where a crease lands in a column's first ~45 px;
- K2's ends are aliasing-prone at SD, where the strips are 6 field lines and Cascade's top rows 13;
- K3 above 100 %.

**v0.5.0 (2026-10-21): built, NOT flashed.** Rebuilt from the photo WIN_20230315_00_28_42 (Heirloom #16):
a wall of tall rainbow capsules, 9.15 columns by 2.08 rows. In each column the contour bands run flat
across and curl into rounded noses at the column's left side, right after a hard cut. The cores on the
row centres are violet/blue and the seams between rows are red/yellow. The columns alternate between
two kinds, half a band apart. Presets are hand-written placeholders.

## Phase 2 (v0.6.0)
Ron left no notes for jukebox; it was done free-hand in the 2026-11-02 batch from the phase-2 brief. The brief
asks for per-element parameters over a global X/Y zoom, wider ranges, bold switches and no near-dead controls.
v0.5's K1 Columns and K2 Rows were exactly the global zooms. So the grid is now fixed at the photo's
9.15 × 2.08, and the two knobs size the capsules column by column and row by row. Built as a diag copy;
production is unchanged.

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Columns: 2–24 columns across (a global X zoom), photo 9.15 at 57 % | **K1 Widths**, bipolar, centre (512) = the photo's equal 210 px columns. **Right = Lens:** the column-phase step follows screen x, step = hstep (1 + L (3t² − 1)), so the centre columns swell and the edge columns squeeze (a curved jukebox front). At 100 % (L = 0.81) the centre column is ~600 px and the edge columns ~100 px. L is capped at 0.81: nearer 1 the centre magnification runs to hundreds and the 256-step column tables show as stepped blocks in the noses. **Left = Fan:** step slope 0.8625 t² + 1.45 t − 0.2875, so the columns shrink from ~630 px on the left to ~80 px on the right (a wall in perspective). The centre column stays put (+23/32 of the shift is taken back on the start phase) | **Renamed.** Both slopes integrate to 0, so 9.15 columns still span the screen. Exact photo within ±8 of the centre |
| K2 Rows: 1–6 rows per frame (a global Y zoom), photo 2.08 at 46 % | **K2 Heights**, bipolar, centre = the photo's 2.08 equal rows. **Right = Cascade:** rows small at the top growing down the screen. The row step starts at ×4.9 and falls linearly to ×0.27 at the bottom; about 5.4 rows, with capsules of 125, 144, 180 and 268 lines at 1080, then one huge partial capsule. **Left = Alternate:** tall capsules between thin strips. The thin kind's step is ×(1 + v²/102), up to ×10.7; the tall kind's is ×(1 + v/512), up to ×1.25. At the end that is 414-line capsules with 49-line strips at 1080 | **Renamed.** Top-anchored, as in v0.5 |
| K3 Nose: the nose tail stretched 0.31× (0 %) .. 3.3× (100 %), photo at 50 % | **0.31× .. 10.7×**. The lower half is unchanged (bit-exact with v0.5 up to 50 %); the upper half has twice the octaves, and at 100 % the curl spreads across the whole column | The short end was already flat (no noses) in v0.5 |
| K5 Tint: ±0.8 fold on the red / blue ramps | **K5 Detune**, bipolar: the two column kinds' bands per capsule against each other, linear in the knob. Right = the red-topped (B) columns up to ×2.84 bands while the yellow-topped (A) go down to ×0.36; left = the reverse; centre = the photo. The kinds then also stream at different speeds with K4 | **Renamed; Tint was dropped.** S9 Palette still flips the colours |
| K6 Etch: 0–3 folds per full luma, photo 0.61 at 45 % (460) | **0–6 folds**, square law; the photo's 0.61 is now at 32 % (326, the same integer etch, 155) | Also fixes a v0.5 wrap of Bands × Etch when both are near the top |
| S8 Pairs: On / Off (all column kinds alike; subtle) | **S8 Fold**: Tri (the photo) / Saw (every band a hard-edged ramp, one per 2 folds: the same band count, sharp edges) | **Renamed** |

Unchanged: K4 Speed, S7 Mirror, S9 Palette, S10 Stack, S11 Source, P12 Bands. The defaults draw the v0.5 picture (see
Checks). K1, K2 and K5 now default to 512, and K6 to 326.

**Presets:** Photo (the defaults); Capsules (Mirror, long noses); Soft Lens (Bands 6 %, Lens); Filigree; Ember Wall
(Swap + Detune right); **Cylinder** (full Lens + Mirror); **Perspective** (full Fan + Cascade); **Sawtooth**
(Saw + Detune left + Alternate).

**Engine changes:**
- **Per-field sequencer.**
  - The column / row counts and the vertical step are per-timing constants (C_HD, C_RQ, f_vstepc). That frees the
    slots that computed them.
  - One bipolar knob decoder is time-shared in the zone, where v0.5 decoded each knob free-running. It reads K1 at
    zone clock 4, K2 at 7, K5 at 13 and K4 at 20, and is pipelined three clocks deep: input, |v| − 8 / sign / centre
    zone, then the knob's values.
  - The slot-load decision is registered one zone clock ahead, so only a registered selector drives the multiplier's
    operand mux. With 18 slots the comparison chain had been the critical path (70.5 MHz).
  - New products, one slot each:
    - Lh = L × hstep;
    - the DDA's first and second step differences, Lh × a 16-bit constant per (Lens / Fan, width class);
    - L × the half-screen span (Fan centre fix), chained from the previous product;
    - v² (Alternate thin-row factor);
    - Bands × the two detune factors;
    - vstep × the two row-kind factors;
    - c × a per-timing constant (the Cascade decrement per line).
  - The first step is a shift-add: hstep + 2 Lh (Lens), hstep − 7/8 Lh (Fan).
- **Per line.**
  - The vertical phase adds row kind A's step, or kind B's constant step on B rows in Alternate. It is 29 bits: a
    25-bit phase whose bit 24 is the row kind, plus 4 fraction bits.
  - Kind A's step falls by the decrement every line (Cascade).
  - Bands × Vg is computed twice, once per column kind (49 of the line's clocks). The field kick moved from zone
    clock 360 to 508.
- **Beam.** The column-phase step follows a second-order DDA (step += d, d += d2, Q12) restarted at every line, so
  the step traces the Lens / Fan quadratic exactly in step with the beam. At L = 0 it is hstep with d = d2 = 0, the
  photo bit for bit.
- **Pixel path.** The depth is unchanged (28, so the U/V parity holds). The per-kind Bands × Vg mux sits at stage 5,
  Saw at stage 9, and the Etch path is 1 bit wider.
- **EBR:** 18 (unchanged).
- **Fit.** The first build (per-pixel slope multipliers from a 2048 × 12 slope ROM, LUT-ROM knob curves) overflowed at
  7,891 LC. Standalone-yosys ablations ranked the costs: the slope side path (~540 LUT), the four 64-entry LUT ROMs
  (~220), and the row-kind steps (~210). Every feature was kept, by switching to the DDA, shift-add knob curves,
  chained sequencer products, the shared decoder and narrower fractions.

**Resolution.** SD still shows the centre 6.85 columns of the HD picture, as in v0.5. The Lens and Fan profiles span
each screen's own width, so SD's warp is the same shape across a narrower crop, not a crop of HD's warp. Heights are
normalised per timing, so 720p, 1080 and SD show the same rows.

**Checks (2026-11-02):**
- **GHDL front end** (analyse, elaborate, synth): clean.
- **Sweep test** (`.heirloom_work/rb_jukebox/p2/sweep.py`) of every new piece of knob arithmetic at the RTL widths:
  - **K1:** the DDA over all 1024 knob values and the 3 width classes. The step stays positive with no overflow, and
    9.10–9.14 columns span the screen. The Fan's centre column stays within 0.04 columns, and the default is exact
    (hstep, d = d2 = 0).
  - **K2:** the per-line walk for every timing. No negative step, no overflow, and the default rows are bit-exact.
  - **K5 / K6:** the widths fit.
  - **K3:** the nose LUT equals v0.5's for K3 ≤ 512.
- **GHDL renders** (720p dec 1, Osc, warm-up 2):
  - The default frame matches the v0.5 baseline bit for bit from x = 8 on. x = 4–7 differ because of v0.5.1's
    output-filter resync, which the v0.5.0 baseline predates.
  - The final RTL's default frame is identical to the earlier diag renders.
  - Two extreme renders from glide-preset sim copies, both as designed:
    - Lens 100 % + Alternate 100 % + Detune 900 + Saw, rendered before the Lens cap. It showed the stepped centre
      that led to the cap.
    - Fan 100 % + Cascade 100 %.
  - Files: `p2/def3_osc.png`, `p2/x1_lens_alt_det_saw.png`, `p2/x2_fan_cascade.png`.
- **Not checked:**
  - Video-mode, SD and 1080 renders;
  - motion over time (K4 with Detune / Heights);
  - the capped Lens extreme (its arithmetic only);
  - HD HDMI bars with S8 Saw.


**Build** (router2, the builder default, 2026-11-02; diag, NOT flashed):

| Config | Seed | Fmax (MHz) |
|---|---|---|
| HD Analog | 1 | 85.73 |
| SD Analog | 1 | 83.44 |
| HD HDMI | 1 | 83.72 |
| SD HDMI | 1 | 73.81 |
| HD Dual | 1 | 83.91 |
| SD Dual | 1 | 80.59 |

7,302–7,318 / 7,680 LC (95.3 %, about +820 over v0.5.1), 18 / 32 EBR. iceunpack OK ×6. No pin-skew screen.
Earlier rounds:
- the first build failed placement at 7,891 LC;
- the second fitted at 7,264 LC but missed HD Analog at 70.5 MHz (the slot decode and knob decoder paths);
- both fixed as described above.


## Reading of the photo
Measured on a 420 × 261 tile. The photo is exactly one 2-column × 520-line tile, mirrored about the
row centres; the repeats deviate by 1–3.6 RGB levels.
- **Grid.**
  - 2-column period 419.8 px (4.574 per 1920 px, so 9.15 columns).
  - Vertical period 520 lines (2.08 rows); row centres on lines 259.45 + 520 k, seams halfway between.
  - Column cuts at x = 91 + 419.8 k (B columns, ~207 px wide, red-topped) and 298 + 419.8 k
    (A columns, ~213 px, yellow-topped).
- **Three channels, one vertical field.** Each RGB channel is its own function of a vertical field
  V(d), where d is the distance from the row centre and dn = d / 260.
  - **G (the rainbow contours):** a sharp triangle fold, `G = 10 + 233 tri(f)`, with 5.3 folds from
    centre to seam. The fold spacing narrows outward: 63.5, 54.6, 48.1, 42.7, 38.8 px.
  - **The field:** `V = 0.0078 + 3.625 dn + 1.711 dn²` folds, minus a seam dip
    `0.768 clip(1 − 31.5 (1 − dn))`. The dip makes the thin red seam line in the B columns. Joint fit
    over both column kinds: rms 0.036.
  - **R:** `R = r(x) + 36.5 V − 0.09 G` (rms 1.2). R sees 0.35 of the seam dip.
  - **B:** `B = clip(bo(x) − 79 bg(x) V − 0.085 G)` (rms 1.8).
- **Column kinds.**
  - The G field differs by half a fold: E_A = 0.016, E_B = −0.474.
  - The B offset is 434 in A columns and 394 in B columns.
  - R falls from 94 to 52 across B columns and rises (accelerating) from 64 to 103 across A columns.
- **Column transient (every cut).**
  - For about 45 px the vertical gain drops to ~0.35 and the contrast to 0.1–0.6, giving a teal strip.
  - Then a nose offset of ~1 fold at 46–52 px decays with a time constant of about 15–20 px. This
    curls the bands into the "⊂" noses.
  - The noses were measured by tracking contours: the fit with gain a ≈ 1 holds from 46 px on.
- **No textures.** There is no tearing and no noise. The picture is soft horizontally only (analog
  bandwidth); vertical blur is under 1 line.
- **Patch reading.**
  - A vertical triangle runs through a squaring stage. It feeds a many-fold wavefolder on G, a rising
    ramp on R and an inverted, clipped ramp on B.
  - A horizontal oscillator at half the column rate adds a square (the two column kinds) and a
    triangle-ish wave on R.
  - It restarts each column with a reset transient. The transient kicks the fields (the noses) and
    gates the vertical gain (the strip).

## What it does (v0.5; the v0.6 changes are listed under Phase 2)
1. **Per line.**
   - `dn` comes from the vertical phase: a triangle, or a saw for Stack.
   - `V0 = 2 + 928 dn + 438 dn²` (Q8 folds), then the seam dip.
   - `Bands × Vg` for G, and the red and blue ramps `36.5 (Vr + tint)` and `79 (V − tint)`.
2. **Per pixel.**
   - The 2-column phase gives the column kind and the position (0..255).
   - Mirror reverses the B columns; Pairs Off makes every column kind A; Nose stretches the
     position past the reset strip.
   - The measured column tables give E, a, c, o (G), r (R), and bo, bg (B).
3. **G.** `f = a (Bands Vg + Bands·etch·luma) + E − t` (mod 2 folds), then
   `G = 10 + 233 (o + c tri(f))`.
4. **R and B.** `R = r + Rv − 0.09 G` and `B = 184 + bo − bg Bv − 0.086 G`. Palette Swap exchanges
   R and B.
5. **Output.** BT.601 to 10-bit YUV through three colour-matrix ROMs, with luma pre-scaled 16/13
   against the output stage's 13/16 and clamped at 940. Then the output conditioning and the
   blanking gate.

## Controls (v0.6.1)
| Control | Photo element it drives |
|---|---|
| K1 Widths | per-column widths, bipolar: centre = the photo's equal columns (9.15 across); right = Lens (centre columns swell, edge columns squeeze); left = Fan (columns wide on the left, narrow on the right); the last third of each side folds the wall into mirrored columns around a crease (L 1 .. 1.5) |
| K2 Heights | per-row heights, bipolar: centre = the photo's 2.08 equal rows; right = Cascade (small rows at the top growing down, ~10 rows from 57 lines); left = Alternate (341-line capsules between 26-line strips at the end) |
| K3 Nose | how far the rounded noses reach into each column: the nose tail stretched 0.31× (0 %) – 1× (100 %, the photo) – 21× (200 %); the reset strip is kept |
| K4 Speed | band flow, bipolar, still within ±8 of centre; right = bands stream out of the cores; (\|v\|−8)²/4 folds per field (600 = 0.38 folds/s) |
| K5 Detune | band counts of the two column kinds against each other: centre = the photo; right = red-topped columns up to 2.83× the bands, yellow-topped down to 0.35×; left = the reverse |
| K6 Etch | picture luma pushed into the contours: 6 k² folds per full luma (326 = 0.61) |
| S7 Mirror | every other column reversed: closed ⊂⊃ capsules |
| S8 Fold | Tri (the photo's soft folded bands) / Saw (hard-edged sawtooth bands) |
| S9 Palette | Photo / Swap (red and blue exchanged: red cores, blue seams) |
| S10 Stack | Mirror (the photo) / Saw: rows repeat and are cut flat at every seam |
| S11 Source | Video / Osc |
| **P12 Bands** | contour bands per capsule, 0.25–5× (406 = the photo): 0 % = one soft rainbow gradient per capsule, 100 % = fine rainbow filigree |

## Video
- **How the camera enters.** Its luma (line buffer, `(y >> 2) − 16`) is added to the G field, scaled
  by Etch × Bands. The contours bend around the picture while the palette and the capsule layout
  stay. At the default Etch the face reads inside the capsules and the photo's composition holds.
- **Osc.** Source = Osc substitutes the shell oscillator, a sum of slow x and y triangles drifting
  per field, through the same path. Every video control stays live.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks), one serial shift-add multiplier,
  14 product slots every 24 zone clocks:
  - k1², horizontal step, line-start phase.
  - k2², rows × the timing ID's virtual lines, vertical step.
  - p², Bands; k6², Etch; Bands × Etch; speed², flow.
  - Tint by shift-add; nose stretch from a 64 × 8 LUT.
  - One shared glide (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.
- **Per-line engine**, kicked at each line start, computes the NEXT line in ~34 clocks (fits SD's
  858-clock line):
  - dn² and Bands × Vg on a serial multiplier.
  - V0, the seam dip and the R/B ramps as one shift-add per clock.
- **Pixel path**, 21 stages plus 7 of output conditioning = 28, the same latency in every mode:
  - Column kind and position, Mirror / Pairs, nose stretch (two 8×4 partials).
  - Four column-table ROMs, 512 × 16, address = kind & position.
  - Etch partials, a × gs and bg × Bv partials, fold phase and triangle, c × tri.
  - G (×233/256 by shift-add), R and B with the G crosstalk, Palette Swap.
  - Three colour-matrix ROMs (256 × 32), the Y/U/V sums.
  - Every ROM output gets a fabric register before any arithmetic.
- **EBR:** line buffer 4, column tables 8, colour ROMs 6 = 18.
- **Contracts:**
  - damier's generated avid (parity-locked).
  - Output conditioning before the blanking gate: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1],
    luma 13/16.
  - Colours in the hardware convention (U register = Cr, V = Cb).
- **Resolution.** Geometry is in virtual 1080-line units from the timing ID, so 720p shows the same
  picture. **SD (4:3) shows the centre of the HD picture: 6.9 columns of the same capsule shape.**
  Interlaced modes get no field offset (field_n is tied high on hardware).
- **Build hook.** `jukebox.py` holds the measured tables as integers (pure python, as the builder
  needs) and writes `jukebox_pkg.vhd`.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 87.49, SD Analog s1 84.80, HD HDMI s1 80.10, SD HDMI s1 62.26, HD Dual s2 82.39, SD Dual s1 62.91 MHz; 6488-6500 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, router2 (the builder default), NOT flashed.**
- **Seeds:** HD Dual = seed 4 (seed 1 stalled the router, seeds 2–3 missed timing); the other five
  are seed 1.
- **Fmax (post-route, last line):** HD Analog 79.16, HD HDMI 80.04, HD Dual 76.94, SD Analog 81.55,
  SD HDMI 84.36, SD Dual 85.40 MHz.
- **Resources:** 6467–6483 / 7680 LC (84 %), 18 / 32 EBR. Every bitstream passed iceunpack.
- **Verification:**
  - The RTL is bit-exact against the integer model `imodel.py` in 720p GHDL sims (black video,
    warm-up 2 and 10).
  - The default motion moves the contours outward from the row centres.
- **If HD Dual regresses:** try `NEXTPNR_ROUTER="--router router1"` before restructuring.

## Deviations from the photo, and why
- **The reset strip.** In the first ~45 px of every column the photo has 2–3 thin bowed lines and
  small loops near the seams. The program draws a smooth teal/olive band there.
  - The column tables are separable (per-position offset, gain and contrast), and that structure
    cannot carry the strip's 2-D detail.
  - Eight non-separable models (crossfade, fold shift, reset pulse, d-weighted contrast, monotone
    offset, blur-only and others) fitted no better than the chosen tables. Those reach rms 0.079,
    against 0.02–0.03 in the column interiors.
- **Corners.** The top-left and bottom-left corners of the outer bands are a little squarer than
  the photo's sweep.
- **Pale line.** A thin pale line runs the full height at the B-column cuts; in the photo it is
  visible near the seams only.
- **Column widths.** Both column kinds are 209.9 px wide; the photo alternates 207 / 213 px.
- **Capture borders.** The capture's black top rows and dark left edge are not drawn.
- **Motion is designed.** The photo is a still: the bands stream out of the cores and the colours
  slide through the R/B ramps.

## Working files
`.heirloom_work/rb_jukebox/`:
- `notes.md`: measurement log.
- `tile.npy`: the averaged tile.
- `tables.json`: the fitted column tables, in float.
- `protoF.py`: float prototype in the hardware's structure.
- `imodel.py`: integer model of the RTL; it imports `jukebox.py`.
- `gen_hook.py`, `hook_template.py`: write `jukebox.py`.
- `sheet.png`, `isheet.png`: control sheets.
- `cmp_proto.png`, `cmp_final.png`, `cmp_motion.png`: comparisons.

## Open questions (closed 2026-11-03)

v0.6.0 asked Ron two questions: bring a column/row count back (on P12 or a switch), and bring Tint back as a palette switch. His notes asked for neither, and he approved v0.6.1 without them, so the v0.6 control map stands. The diag copy stays at `.heirloom_work/rb_jukebox/diag/jukebox_diag/` for reference.

**Next step:** capture real presets from the hardware (`./capture_preset.sh`), then rebuild.
