# Rorschach — Heirloom #4, rebuilt from the photo

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/rorschach.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-03, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Rorschach Diag`, `out/rev_b/ron/_diag/rorschach_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/rorschach/`) is untouched. Phase-2 working files:
`.heirloom_work/rb_rorschach/p2/` (sweep.py, sweep2.py, check.sh, psim.sh, prof/, cmp_p2.png).
v0.5.0 (2026-10-21): built, NOT flashed. Rebuilt from `photos/WIN_20221124_22_54_05_Pro.jpg`
after Ron judged v0.1.0 too abstract. Osc mode redraws the photo itself:
- the folded pastel inkblot with the blue "head" at the right edge;
- the blue arrowheads and the red/blue cross on the fold;
- the blue-and-cyan "river";
- the busy torn strips either side of the fold;
- the red/blue stair-steps.

Video mode puts the camera through the same processing chain. Work files:
`.heirloom_work/rb_rorschach/` (`notes.md`, `protof.py`, `imodel.py`, `sheet.png`, `cmp_final.png`).

## Phase 2 (v0.6.0)
No notes from Ron for rorschach (free hand, 2026-11-03 batch). v0.5 already had no global X/Y zoom, but five of
its twelve controls were colourizer settings and two switches were nearly invisible on the control sheet (Stairs
On held only the dark band; Split only turned the few red cells blue). The round gives the photo's own elements
their parameters and adds motion. The dark shapes (head, arrowheads, river) get a size control, the frame-store
mirror gets a second axis, the copies can slide and slip against each other, and the sample-and-hold covers the
whole picture.
- **The store is now read with mirrored edges** (period two widths, about columns −0.5 and 63.5), so sideways
  drift, slip, Quad and tears never show a seam or a smeared edge column.
- **The default frame is bit-exact with v0.5.1 away from the tears.** In GHDL (1080p, decimation 2, Osc, defaults)
  527 of 540 rows are identical to the production render. The other 13 are torn rows: a torn line shifted past a
  screen edge now reads mirrored content instead of the clamped edge column, which also re-phases that line's smear.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms |
|---|---|---|
| K2 Level: colourizer offset (the bands crawl through the cycle; 0 = the photo) | **K2 Ink** (renamed): the dark band's width, square law, photo at 50 % (bit-exact). The six pastel bands share the rest equally | 0 = no dark band at all: the head, arrowheads, river and red/blue stairs (~15 % of the frame) turn pastel. 50 % = the photo (37/256 of each colour cycle). 75 % = 2× (29 %). 100 % = 4× (58 %): blue/red masses cover over half the frame, the pastels thin to contour lines. The static colour offset is gone; K4 still cycles the bands through every offset |
| K4 Speed: bipolar colour cycling, still at the centre | **+ sideways drift.** Beyond ±92 of the centre (the default 600 sits just inside), the inkblot also slides sideways in the cycle's direction, square law, through the mirrored edges | 0 for K4 418..606 (the photo stays put at the default). 60 s per screen width at 700, 21 s at 768, 9 s at 850, 3.2 s at either end. With Quad on, the halves flow into / out of the centre line |
| K5 Fold: fold line 25 .. 100 % of the height | **12.5 .. 100 %**, linear; default moved 323 → 423 (the same 48.7 % fold, bit-exact) | 0 = eight folded copies stacked down the frame (v0.5: four). 100 % = unfolded |
| S8 Stairs: Off / On (On held the dark band per 42-px cell: nearly invisible) | **Stairs: Hue / All.** Hue = the photo (only the dark band's hue stair-steps). All = every colour sample-and-held into 84-px blocks (two of the photo's 42.25-px cells) | All: every contour becomes a staircase with 84-px risers and the busy strips turn into block mosaics |
| S9 Split: Hue / Blue (Blue turned the red/magenta/green dark cells blue) | **S9 Quad** (renamed): Off / On. The whole scene squeezed into the left half and mirrored into the right | On: a four-way symmetric inkblot. The blue head meets its mirror image at the centre (top and bottom), the river and arrows pair off. The all-blue option is gone |
| P12 Tear: frame-store tears (0 = clean fold, 35 % = the photo, 100 % = slabs) | **+ slip.** Above the photo's 35 %, every second copy (the mirrored half; alternate copies in Repeat / multi-fold) tears away sideways, linear | 0 at ≤ 35 %. ~216 px at 50 %, ~575 px at 75 %, ~935 px (half the width) at 100 %. 0 % and 35 % look as before |

Unchanged: K1 Bands, K3 Detail, K6 Palette, S7 Mirror (drift and slip follow the copies in Repeat too), S10 Negative,
S11 Source.

**Presets** (hand-written placeholders):
- **Photo**: the defaults in Osc.
- **Ink Card**: a real Rorschach card. Quad, mono Ink colourway, wide ink, fewer bands, clean, no tears, still.
- **Kaleido**: eight folds, Quad, drifting right, Comp colourway.
- **Shredded**: P12 100 % (slabs plus a half-width slip), Detail high.
- **Mosaic**: Stairs All, more ink and bands, Vivid.
- **Ink Flood**: Ink 100 %, Night colourway, fold higher.
- **Slipstream**: Repeat, fast drift, P12 63 % slip.
- **Camera Quad**: Video, Quad, gentle drift, a little more ink.

**Engine (new).**
- **Mirrored edges.** The beam stage folds the store column (Q16, s = u + 0.5) modulo 128 columns, back from 64
  (one xor level after the existing adder, registered as `b_t`). Stage 1 forms the bank columns (c + 1) / 2 and
  c / 2 straight from the folded bits (m ± 1 on 5 bits), with v0.5's clamps from raw bits. A sweep over the whole
  default range matches v0.5's clamp bit for bit.
- **Drift.** Serial-multiplier slot 10 squares the depth (|K4 − 512| − 92). The product / 8 accumulates into a
  23-bit column offset that wraps at two widths.
- **Slip.** At zc 326..327, (P12 − 358) × 3072 Q16. The line engine adds it to the start of every line whose copy
  parity is odd (`n_odd` flips at each bounce or wrap).
- **Quad.** The read step is doubled and the line starts at half a step, so the right half mirrors about the
  centre. The camera write side keeps its own step.
- **Ink.**
  - Slots 11–12: d = K2² × 150 / 2²⁰. Slot 13: (256 − d) / 6 in Q8.
  - At zc 512..767 a fill engine writes a 256 × 4 block-RAM lookup, phase → {dark, band − 1}, one phase per clock.
    The thresholds are T₁ = d, T_k = d + ⌈(k − 1)(256 − d)/6⌉.
  - The lookup replaces the six per-pixel comparators. At 50 % it equals the photo's decoder for all 256 phases.
- **Stairs All.** Stage 14 holds the whole palette index from the start of each cell pair.
- **Smear.** The tear's smear now takes a new colour once the column has *changed* l_hc times since the last take,
  so it works in either direction across a mirror line. For v0.5's monotonic columns this is identical.

**Fit.** The first build placed 7,526 LC (98 %), and HD Analog missed on all 6 router2 seeds (best 66.5 MHz;
critical paths all routing). It was fitted with no feature lost:
- the output chroma filter's delay lines moved into a block-RAM ring, written from its own copy register so
  `p16` stays beside the colourway logic;
- the stage 5 → 13 fields (smear take, cell-pair start, dark hue) moved into a second ring on the same pointer;
- the Ink lookup RAM replaces six variable comparators, and the Ink curve runs on the serial multiplier;
- the line engine reads the tear draws directly. The next draw starts on the group's last line, so the draw
  sequence is unchanged and the 77 copy registers are gone;
- the colourway is a per-entry precomputed table (`f_palx`: Y, 251 − Y and the clamped U, V, −U, −V, 2U, 2V)
  muxed in stage 16. It is exact for all 16 entries × 8 colourways and replaces the subtract / negate / clamp
  chain, which was the critical path;
- the line engine's store-row subtract and clamp are split across two states.

Latency (24) and pixel depth are unchanged, so the U/V parity holds. Router2 still scattered (63–82 MHz on HD
Analog seeds) while router1 gave 85.85 MHz, so the diag is built with router1.

**Build** (router1, `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh rorschach`, 2026-11-03): all six on seed 1.
HD Analog 85.85, SD Analog 82.39, HD HDMI 82.01, SD HDMI 81.92, HD Dual 84.26, SD Dual 85.27 MHz. 7,209–7,218 / 7,680 LC (94 %,
+~270 over v0.5.1), 27 / 32 EBR (+3: two delay rings and the Ink lookup), no `emulate_transparency`. iceunpack OK ×6, and the
packed config carries the new labels and presets. No pin-skew screen.

**Checks.**
- GHDL analyse / elaborate / synth front end: pass.
- Python sweeps at RTL widths (`p2/sweep.py`, `p2/sweep2.py`):
  - Ink square law and fill engine: no overflow, monotonic at every K2, all six bands present, the photo's
    thresholds exact at 50 %;
  - fold f_av 135..1081, 526 at the default, kstep fits 18 bits at every raster;
  - drift: 0 across K4 418..606, max 21,945 Q16 / field;
  - slip: max 2,042,880 Q16 (22 bits);
  - the stage-1 column mapping is bit-exact with v0.5 over the whole default range, Quad mirrors to 0.03 column,
    and a drifted mirror seam is continuous.
- GHDL renders (1080p decimation 2, Osc), `p2/cmp_p2.png`:
  - the default frame against production v0.5.1 in the same settings: 527 / 540 rows identical; the 13 torn rows
    differ as described above;
  - the final RTL is pixel-identical to the previous fit render;
  - one extreme frame: Quad + Stairs All, with Ink / Fold / P12 three fields into their glides. The mirrored head
    is in the centre, the blocks show, and the lower copy is slipped.
- Not simulated: drift and slip in motion (the per-field accumulators), Video mode with the new features, SD.

## Reading of the photo (measured)
- **A fold, not a quad mirror.** Flip-correlation finds one axis: rows r and 1051 − r (axis 525.5).
  - The mean |diff| there is 24.6, against 60 for random pixel pairs.
  - The best left/right axis scores 50, which is baseline: there is no left/right mirror.
  - Smooth rows match to within 3–10; busy rows differ by 30–47. The lower half is the same scene
    with different fine detail: a frame-store mirror showing a later frame.
- **Camera content.**
  - Organic shapes: a blue head-and-shoulder at the right edge, a spiky blue/cyan river,
    curtain-like vertical bands.
  - Edges are soft, ~10–15 px horizontally and ~4 rows vertically: an SD source upscaled by the
    capture.
  - Bands are perfectly flat inside: a posterizing colourizer.
- **Palette** (flat pixels, 60 % of the frame):
  - cream (254,255,178) 26 %, peach (255,231,183) 26 %, pink (254,193,228) 11 %;
  - orchid (253,195,251) 3 %, lavender (195,195,254) 2.3 %, pale cyan (188,254,253) 1.6 %;
  - blue (30,23,249) 5.6 %, red (254,21,3) 1.9 %, magenta 0.2 %, green 0.1 %.
- **Cyclic colourizer.**
  - Colour adjacencies (runs ≥ 3 px) form one chain: cream–peach 2400, peach–pink 5774,
    pink–orchid 3616, orchid–lavender 2496, lavender–pale cyan 1701.
  - Pale cyan meets blue as a pure two-colour blend; cream meets the dark band directly (blue 1032,
    red 581).
  - Order: dark band → cream → peach → pink → orchid → lavender → pale cyan → back to the dark
    band. That is 7 bands on a wrapping scalar.
- **The red/blue stair-steps are the camera's hue, sample-and-held.**
  - Red/blue risers sit on a fixed grid: period **42.25 px**, phase 10.5 (concentration 0.995).
    That is 16 SD px, 45 cells per line. The treads fall on any row (no y grid).
  - Only blue↔red boundaries are on the grid (100 %); every other colour pair is at chance (15–24 %).
  - Within one grid cell, row 200 runs cream → blue while row 240 runs cream → red → blue. So red
    versus blue is not a band of the scalar: it is the camera's hue (the head's blue hair and red
    face), held per cell, colouring the darkest band.
- **Texture.**
  - The vertical spectrum runs to Nyquist while the horizontal one falls off: the picture is
    horizontally streaked.
  - Busy strips sit at |y − fold| ≈ 75–230 rows (row busyness 0.28 there against 0.11).
  - No row is shifted as a whole. The tears are local strips (rows from elsewhere, smeared), not
    line tearing.

## What it does
1. **Store.** The half above the fold is held as a 64 × 128 wrapped-phase field (30 px × ~4 lines
   per cell at 1080).
   - Osc: a ROM fitted to the photo. The photo's band labels were turned into a ramped phase (POCS),
     then the 8192 cell values were optimised (hinge loss on the bilinear readout). 70 % of
     half-res pixels land in their photo band.
   - Video: the camera written live, one value per 30-px column per store row: frac(luma × 2 ×
     Bands), 4 bits, from an IIR-smoothed luma, plus a 2-bit hue class.
   - Four banks by (row parity, column parity), so all four bilinear corners are read in one clock.
2. **Fold.** Each output line bounces through the store rows (Mirror) or wraps (Repeat). The fold
   line is K5.
3. **Readout.** A wrap-aware bilinear lerp that steps instead of interpolating across jumps of more
   than 13/64 cycle. Then × Bands (Osc), Negative, + fragments, + Level and the colour cycle.
4. **Colourizer.** 7 bands of an 8-bit phase. The dark band takes the hue map (camera hue, or the
   photo's in Osc), sampled at the first pixel of each 42.25-px cell and held across the cell.
5. **Fragments.**
   - Where the field is busy (the corners disagree by more than 12/64), the phase gets a hashed
     offset per (cell, 4-line group), up to ±1.2 cycles at the default K3.
   - The dark band's hue in busy cells is hashed too: blue 45 %, red 27 %, magenta 18 %, green 10 %.
6. **Tears.** A background tear-group unit draws groups of 1–37 virtual lines.
   - At the default 35 %, a group tears with p = 30 % in the busy zone (70–235 lines from the fold),
     5 % near the fold and 17 % far from it. At 100 % the rates rise to 100 % / 62 % / 74 %.
   - A torn group jumps ±27 lines (±78 at 100 %) and shifts ±67 px (±500 at 100 %). 30 % of torn
     groups smear, holding the colour for 2–9 columns.
7. **Colourway (K6).** A chroma or luma transform of the photo palette: Ink (no chroma), Swap,
   Comp, Turn, Twist (U/V swaps and quarter turns), Night (luma negative), Vivid (2× chroma).

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | number of colour bands: colourizer gain 0.5–4 cycles, square law (1 = the photo; Video ×2) |
| K2 Level | colourizer offset: the bands crawl through the cream → … → pale cyan → dark cycle (0 = the photo) |
| K3 Palette | Rorschach (the photo) / Ink / Swap / Comp / Turn / Twist / Night / Vivid (moved from K6 in v0.6.1; the busy-strip fragments, was K3 Detail, are fixed at the photo's 60 %) |
| K4 Speed | colour cycling, bipolar, still at the centre (±8); default slow forward (one cycle per ~33 s) |
| K5 Fold | the fold line, 25–100 % of the frame height (48.7 % = the photo; low = repeated folds, 100 % = unfolded) |
| K6 Camera | camera strength (v0.6.1), 0..100 %, default 50 %. Below 50 % the source (camera in Video, the photo's scene in Osc) shows only where the photo's own inkblot phase is under a level, chosen so the revealed share grows linearly with the knob; the rest is the flat ground a black input gives (phase 0: the dark band, colour-cycled). 0 = no camera, 25 % = about half the frame, 50 % = the shipped look; above 50 % the chroma is boosted, ×1.875 at 100 % (clamped at the palette's own ±508). Glided |
| S7 Mirror | Mirror (the photo) / Repeat: the half above the fold reflected, or repeated |
| S8 Stairs | Off (the photo: only the hue stair-steps) / On: the whole dark band sample-and-held into 42-px blocks |
| S9 Split | Hue (red / blue / magenta / green dark band) / Blue (all blue) |
| S10 Negative | the scalar inverted: the band order reversed |
| S11 Source | Video (default) / Osc |
| P12 Tear | frame-store tears: 0 = a clean fold, 35 % = the photo, 100 % = torn into displaced slabs |

## Video
Video mode is the photo's own processing chain applied to the camera.
- The camera's top part, down to the fold, is written into the store live, so it is folded and
  posterized by the same cyclic colourizer.
- Its dark band takes the camera's hue: a 2-bit class of the column's last pixel (red, magenta,
  green, else blue), held per 42-px cell, so warm objects stair-step red against blue.
- Fragments and tears act on it as on the scene.

Osc substitutes the photo's scene. Every control acts the same in both modes.

The HX4K has no frame store, so the camera is stored at 64 × 128 cells with 4-bit phase. Video
shows soft blobs of the camera, not its fine detail.

## Engine
- **Per field (vblank zone):**
  - 5 glides (K1, K2, K3, K5, P12; 1/8 per field, ±4 deadband).
  - 9 slots on one 20 × 18 serial shift-add multiplier, with operands registered a phase early:
    the gain, the video gain, speed² and the colour-cycle rate, the fold, t², t⁴, the spill.
  - The P12 tear curves by shift-add.
  - 4 serial divisions: columns and S&H cells per pixel from the measured width; the virtual-line
    step from the measured active lines; store rows per line from the fold.
  - The picture is therefore the same at SD, 720p and 1080, and in decimated sims.
  - The colour-cycle accumulator; reseed every 4 fields.
- **Per line:**
  - An 8-state engine, 24 clocks after the source avid fall, writes the next line's registers:
    advance and fold, tear probability, tear group, store row / fraction / bank addresses, the hue
    row, the video write row.
  - The tear-group unit draws the next group in the background (xorshift32 plus a 22-bit serial
    multiplier, ~45 clocks).
- **Beam:** column and S&H-cell accumulators.
- **Pixel path, 24 stages:**
  - bank and hue addresses, and the fragment hash;
  - EBR, then corners and the Osc/Video select, with the hue held from the cell start;
  - wrapped differences and skips, then y products and lerps, then the x difference and product;
  - phase → gain → Negative → + fragments + offset → band;
  - Stairs, hue and the smear hold → palette → colourway;
  - conditioning: luma [1 3 4 4 3 1]/16 smear, chroma [1 2 2 2 2 2 2 2 1]/16, luma 13/16, then the
    blanking gate on the generated avid.
- **Video write:** per source pixel, a 1/16 IIR (1/8 at SD); one event per 30-px column; gain
  product; 4-bit phase and hue class into one bank. `no_rw_check` is on the camera RAMs.
- **EBR (24):** scene 4 × (2048×4 + 2048×2) = 12, scene hue 4096×2 = 2, camera 4 × 2048×4 = 8,
  camera hue 2.
- **Palette:** case-function logic. Luma is stretched to 940 and chroma scaled by 13/16, so the
  output conditioning's luma × 13/16 keeps the photo's hue ratios.
- **Contracts:** damier's generated avid; one 24-clock latency in every mode; colours in the hardware
  convention (U = Cr); per-field work anchored on the vblank zone (no avid for 1024 clocks).

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 75.78, SD Analog s2 73.52, HD HDMI s1 78.31, SD HDMI s1 80.74, HD Dual s1 79.59, SD Dual s1 76.17 MHz; 6930-6955 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21, NOT flashed.** router2. Build seeds: **HD Dual = seed 3**; the other
  five are seed 1.
- Fmax (post-route; every config's last line PASS):
  - HD Analog 78.06, HD HDMI 75.47 (thinnest), HD Dual 80.37 MHz;
  - SD Analog 76.10, SD HDMI 76.35, SD Dual 76.25 MHz (SD Fmin 27).
- 6928–6948 / 7680 LC (90.4 %), 24/32 EBR. Every bitstream passed iceunpack.
- Fit history: the first RTL (value-noise knot unit, address holds, 8 palettes as logic) placed
  9129 LC (118 %). The restructure brought it to 90 %, and the serial multiplier's operand mux
  (71.9 MHz) was pipelined.
- Verified in GHDL (960 × 540) against the integer model (`imodel.py`): mean colour equal within 1
  level. Motion: 28.5 % of palette labels change between frames 3 and 7, about 50 % in the busy
  strips and about 17 % in the bands.
- Presets are hand-written placeholders: Photo, Ink Card, Kaleido, Shredded.

## Deviations from the photo
- **Fragments are procedural.** The busy strips' small fragments come from one hashed value per
  sample-and-hold cell, gated by the busy detector. They are a different realisation, and a little
  finer and less red/green/magenta than the photo's.
- **Exact mirror.** The photo's lower half differs in fine detail (a later frame); here the
  stored half is mirrored exactly (the tears and fragments still differ).
- **About 84 % as bright.** The required HD HDMI bar fix caps drawn luma at ~777. The palette's
  chroma is scaled to match, so hues are the photo's, just dimmer.
- **Fixed sample-and-hold cell.** The cell is fixed at the photo's 42.25 px. A Hold knob was too
  subtle on the control sheet, so K2 became Level.
- **No separate seam line.** The photo's 3-row seam line at the fold isn't drawn; tears near the
  fold occasionally make one.
- **Soft Video picture.** The camera store is 64 × 128 cells, because there is no frame store.
- **SD 4:3.** The same composition is squeezed to 4:3, so the head goes oval. This matches the
  original SD patch: the cell is exactly 16 SD px.
- **Video read-during-write.** Top-half store rows are read on the same line they are written; a
  pixel may read a cell during its write (undefined EBR data: at most a 1-px speckle, no state
  affected). Watch for it on hardware.


## Open questions for Ron (2026-11-03 batch)

1. Ink replaced Level (the static colour offset). Is K4's colour cycling enough to reach every arrangement, or
   should Level come back (for example as a half-cycle offset on a switch)?
2. Quad squeezes the whole scene into each half, so the head meets its mirror in the centre. The alternative is
   unsqueezed: the left half mirrored at full scale, with the head off-screen. Which one?
3. Stairs All uses 84-px blocks (two of the photo's cells). Would you rather have the photo's own 42-px cell
   (finer), or coarser?
4. Should P12's slip start at 0 % instead of above the photo's 35 %? 0..35 % would then also slide the copies,
   and the photo would sit at a small slip.

**Hardware watch list:**
- **Renamed:** K2 Level → Ink, S9 Split → Quad. S8's values Off / On → Hue / All. K5's default raw value moved
  323 → 423 (the same fold), so production presets or captured values for K5 don't carry over as-is.
- **Router1** build, all seed 1, HD 82–86 MHz. No pin-skew seed screening was run; if HD HDMI shows lines,
  re-seed that config.
- **27 EBR** (+3).
- **Output chroma filter:** the 8-sum now subtracts a block-RAM ring's sample, with the resync kept. If thin
  wrong-colour lines or a constant cast appear, suspect the ring first.
- **Drift and slip in motion** were not simulated (the per-field accumulators). Check that the inkblot slides
  smoothly with K4 beyond ±92 of the centre, and that the mirror seams stay seamless at the screen edges.
- The v0.5 note on Video read-during-write still applies.

**Next step:** wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh rorschach` (scoped clean + shared build queue).
On approval: `./heirloom_diag.sh merge rorschach`, then repack the previewed diag bitstreams under the production
TOML instead of re-routing:
- copy `build/programs/ron/rorschach_diag/rev_b/bitstreams/*.bin` into a scratch dir with the production TOML's
  `program_config.bin`;
- `vmprog_pack.py --no-sign --hardware rev_b --toml-path programs/ron/heirloom/rorschach/rorschach.toml <dir>
  out/rev_b/ron/heirloom/rorschach.vmprog`.

Then update the production SUMMARY / README / PHASE2 row and commit that program's files.

## K6 camera pass (2026-11-15)
Ron: K6 is the camera strength in every program, default 50 %. The same design as production v0.5.2. **v0.6.1.**
- **K6 Camera** (0..100 %, default 512), a sixth glide slot. Below 50 % the source (camera, or the Osc scene) shows
  only where the photo scene's phase is under a level (32-step table from the scene's phase histogram, so the
  revealed share grows linearly); elsewhere the flat ground of a black input (phase 0, blue dark band). 0 % = flat
  ground, 25 % ≈ half the frame, 50 % = the shipped look. Above 50 % the chroma is boosted ×(1 + n/8), ×1.875 at
  100 %, clamped at 512 ± 508. The key follows the diag's mirrored edges, Quad and folds (it reads the same store
  addresses).
- **K6's old function (Palette) moved to K3.** K2 is Ink in the diag (strong), so the weakest knob here is **K3
  Detail** (the sheet: min and default look the same; only its top end changes the picture) — **fixed at 614** (the
  photo's fragments, no spill). P12 Tear remains the destruction climax.
- **Build (v0.6.1, 2026-11-15):** router1, all six configs seed 1. HD Analog 78.44 (worst HD), HD HDMI 83.30, HD Dual
  85.39 MHz; SD Analog 83.15, SD HDMI 82.39, SD Dual 85.49 MHz; 7478–7490 / 7680 LC (97.5 %), 27/32 EBR. (The first
  K6 build did the whole chroma boost in stage 23 and missed at 67 MHz; the boost is split over stages 23/24 and the
  reveal compare moved to stage 4.) Preview `out/rev_b/ron/_diag/rorschach_diag_v0.6.1.vmprog`. NOT flashed.
