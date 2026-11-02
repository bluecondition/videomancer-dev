# Bunting — bow-tie pennants under a grid of keyed glyph marks

**v0.6.0 (2026-10-30, phase 2 round 1): built, NOT flashed** (see [Phase 2](#phase-2)).
v0.5.0 (2026-10-27) was the rebuild from the photo. Heirloom #35, from
`photos/WIN_20230521_23_37_25_Pro.jpg`. The v0.1.0 version (a wedge fan from the screen centre plus
a staircase glyph grid carrying a video mosaic) was judged too abstract and replaced. The photo is
two bow-tie "X"s from one horizontal triangle oscillator, keyed through a two-zone colourizer
(cyan / white inside a band that flares open to the right, white / yellow outside it), under a
grid of identical blocky glyph marks whose ink depends on the ground beneath them. Presets are
hand-written placeholders.

## Reading of the photo (measured)
The work is in `.heirloom_work/rb_bunting/` (notes.md):
- `masks.py`, `bgclass.py`, `fit1.py`, `fit2.py`: the class map and the field fit.
- `glyph.py`, `gedge*.py`: the glyph grid. `oneshot.py`, `tex*.py`: the ink.
- `edgew.py`: edge widths. `chan.py`: the channel split.

**Palette (k-means, k = 10).**
- White (241,243,235) 33.8 %, yellow (252,211,2) 25.2 %, cyan (4,253,247) 21.2 %.
- Red (249,7,1) 5.1 %, grey (153,152,151) 4.1 %, blue (6,86,248) 4.0 %, dark (37,25,34) 2.8 %.
- The rest are transition colours.

**Per-RGB-channel law.** R falls only where the ground is cyan, B only where it is yellow, and G
carries the glyph mask everywhere. So a glyph on yellow keeps R (red), on cyan keeps B (blue), and
on white all three fall (black / grey).

**Bow-ties (class-map fit, 99.1 % pixel agreement, the residual only on edge pixels).**
- One horizontal triangle oscillator, period 944.6 px. Tx = 0 at the X centres x 526.7 and 1471.3,
  and 1 at the diamond centre x 999.
- Ty = |y − 516.2|.
- Horizontal wedges: Ty < a·(Tx − 49.9), with a = 0.661 opening left and 0.647 opening right.
  Apexes at x 477, 577, 1421, 1521.
- Vertical wedges: Tx < 0.539·(Ty − 70.1). Apexes at y 446 and 586.
- In units of the half period: Ty < 312 / 306·(Txn − 0.106) and Ty > 70.1 + 876·Txn.
- Line-start hold: the first 65 px of each line hold the oscillator, so the band edge and the
  wedge run flat at the left edge.

**Band.**
- |y − 519.1| < 178.2 + 6.89e-5·max(x − 42.5, 0)²: an H ramp squared. Its edges run at y 337 / 701
  at the left and 108 / 930 at x 1880, mirrored about y 519.
- Colours: inside the band, cyan where a wedge is, else white. Outside it, white where a wedge is,
  else yellow.
- The H wedge reaching past the band edge gives the little white bump above and below the central
  diamond.

**Glyphs.**
- Pitch 449.9 × 232.75 px: 4.27 across, 4.64 down, with a 5th row peeking in at y 1070.
- First column at x 130.5, first row at y 138.85. Sub-rows 29 / 29 / 29.5 lines.
- Runs (px from the mark's left): row 0 [0,110.2) ∪ [177.8,283.8); row 1 [104.4,285.0);
  row 2 [0,287.5). That is roughly an 8 × 3 bitmap XXX..XXX / ...XXXXX / XXXXXXXX of 35-px blocks.
- Identical in all 16 cells: a synthetic pattern, not camera content.

**Ink.**
- On yellow: red, with the mask widened 3.8 px each side (horizontally only).
- On cyan: blue, with the mask narrowed 3.8 px.
- On white: black (luma about 18) for 33 px from every run start. A run starts at the mark's left
  edge, after the top-row gap, or after a yellow / cyan edge; the last case draws dark diagonal
  streaks along the triangle edges.
- After the black, the luma steps to about 105, then rises with a time constant of about 25 px to
  grey 160–178. Faint horizontal streaks (line-mean sd 2–4).

**Edges.** Horizontal 10–90 % over 12–19 px; vertical 2–4 lines.

**Proto.** `proto.py` with the fitted constants: MAE 4.3, and the 4 × 4 region means are within
±6 RGB of the photo.

## What it does
1. **X phase.** A 24-bit phase φ (2^24 per oscillator period), held while x < 65 virtual px.
   - Video / Osc bend: φ += L · Bend · 64.
   - Fold: Txn = 0 at the X centres, 1 midway, 12 bits. A side flag selects the left or right slope.
2. **Wedge key.** K = (Txn > th) or (Txn < tv). Per line:
   - th = h0 + Ty·sh / (A·sv), with A = 312 / 306 and h0 = 1 − 0.894·sh (0.106 at sh = 1);
   - tv = (Ty − 70.1)·sh·sv / 876;
   - P12 sets the horizontal spread sh (0.25 → 1 over its lower half; the diamond narrows about its
     own centre) and the vertical spread sv (1 → 1.8 over its upper half).
3. **Band.** F(x) = T·(x − 960) + C·(x − xv)², in Q22 lines, stepped per pixel (F += G; G += H;
   34 / 26 / 14 bits). The pixel is in the band when F > Tz − 178.2. T is the K2 taper (signed), C
   the K3 flare magnitude, and xv the flare vertex: 42.5 px (opening right) or 1877.5 px (opening
   left).
4. **Ground.** In the band: cyan / white. Outside: white / yellow. S8 swaps yellow and cyan.
5. **Glyphs.** A 28-bit glyph phase (4-bit cell column . Q24) and a per-line row with a 4-bit cell
   row. Each glyph is an 8 × 3 block bitmap. The line engine writes the next line's 16 column masks
   (8 bits each) into a line-parity EBR bank. Per pixel: block = (x − 64)·25 / 8192 within the cell,
   read at x and at x ± 35/4096 of the pitch for the widened / narrowed ink masks. S9 mirrors each
   mask.
   - The photo glyph is XXX..XXX / ...XXXXX / XXXXXXXX on the block grid.
   - **Morph.** Each block = photo bit xor a re-roll. A hash of (column, cell row, sub-row, block,
     zoom side) staggers each block's generation g = (dz + h mod 256) / 256, with dz = |K5 − 621|·8
     − 32 (Q3, floored at 0). Once g > 0 a second hash of h and g re-rolls the block: 50 % on, a
     fresh roll every 32 knob units, staggered per block. That is about 0.4 block changes per glyph
     per knob step, different in every cell and different zooming in and out.
6. **Ink.**
   - Red on yellow (widened mask), blue on cyan (narrowed).
   - On white: a run counter gives black for 66 half-px (S7).
   - Then an RC slew g ← g − g/32 + target/32 (1/16 at 720p and SD) from grey 95 toward the target.
   - Plus ±4 streak noise per 32 px.
7. **Colours** {Y, U = Cr, V = Cb}, luma pre-scaled so the shell's 13/16 gives the photo's luma:
   - white 940 / 511 / 499 (777 out);
   - yellow 865 / 644 / 121 and cyan 785 / 76 / 649 (top end compressed to 715 / 650 out);
   - red 397 / 939 / 358, blue 405 / 325 / 844, black 170 / 525 / 516;
   - grey Y = 64 + 4.25·g.
8. **Softness.** A [1 2 … 2 1]/32 box (a 16-tap pair) on Y, U and V at 1080, an 8-tap pair at
   720p / SD. S10 Hard bypasses it.
9. **Shell conditioning.** Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16, then the
   blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Flags | the bow-tie X's | oscillator frequency: 0.375 → 12 X's across, 1638 + 22741 t² + 28049 t⁴ phase per half-px (2.03 = the photo at 506) |
| K2 Band | the middle band's taper | bipolar, straight in the centre ±8 (the photo at 512). The band tilts about x 960 by up to 0.55 lines per px: right of centre it widens to the right and closes into a pennant on the left; left of centre, the mirror |
| K3 Flare | the band's flare | bipolar, straight in the centre ±8. Right of centre the band flares open to the right (vertex x 42.5), left of centre to the left (vertex x 1877.5); curvature up to 4× the photo, square law (the photo at 768, its mirror at 255) |
| K4 Speed | motion + glyph position | bipolar, still in the centre ±8: the X's drift sideways (4 virtual px / field at full). The glyph grid glides (1/32 of the gap per field, about 0.5 s) to a position 2 px per knob step from home at 600: −1200 → +846 px |
| K5 Glyphs | the glyph grid | 1.5 → 9 columns, square law (4.26 at 621), scaled about the grid centre. Away from 621 (±4) every glyph's blocks re-roll as you zoom, a different glyph in each cell |
| K6 Bend | the X edges | how far the video (or Osc) bends the X phase: 0 → 0.25 period at full white (0.083 ≈ 78 px at the default 340) |
| S7 Shot | the black one-shot in the grey glyph ink | On (the photo) / Off (plain grey) |
| S8 Swap | yellow and cyan | exchanged; the inks follow (red on yellow, blue on cyan) |
| S9 Flip | the glyph marks | mirrored |
| S10 Edges | the photo's horizontal softness | Soft (the photo) / Hard |
| S11 Source | grey ink target, X bend | Video (default) / Osc |
| P12 Spread | the bow-ties (headline) | 0 → 50 %: the wedges widen (sh 0.25 → 1); the diamonds grow from thin vertical lozenges to the photo's about their own centres, and the V wedges widen with them. 50 → 100 %: they grow tall (sv 1 → 1.8, v0.5's upper half), ending in wide cyan fans on a white field with a yellow zig-zag string. 50 % = the photo |

Defaults: K1 506, K2 512, K3 768, K4 600, K5 621, K6 340, P12 512, every switch at label 0.
Glides (1/8 of the gap per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12; K4's glyph slide
glides at 1/32.

## Video
The glyph marks are a synthetic block pattern, identical in every cell, so the camera does not
replace them. It enters the patch in two places:
- **Bend.** Its luma (0..255, tapped from `data_in` a few px early) is added to the X oscillator's
  phase, scaled by K6 Bend. It is unipolar, so black adds nothing and Video mode with no camera is
  the photo. The wedge and diamond edges bend and fray around the picture; at the default Bend the
  composition reads clearly.
- **Grey ink.** The luma is the target of the grey glyph ink. After each black one-shot the ink
  slews toward 40 + 0.69·L, so the picture shows in grey through the glyph windows.

**Osc.** A drifting 2-D triangle lattice (about 2520 × 2160 virtual px, 0.12 / 0.06 periods per
second) feeds the same two paths:
- the grey target is 158 ± 32 (the photo's grey is 160–178);
- the bend input is ±31, which gives a gentle ±9 px sway at the default Bend.
K6 stays live in Osc mode.

## Engine
- **Vblank zone.** No avid for 1024 clocks; vsync is never used.
- **Field sequencer.**
  - One serial shift-add multiplier: 20 × 18 bits, 18 iterations, 26 product slots, one every
    32 zone clocks (zone clocks 40 → 864), with the operands pre-muxed a clock ahead. A slot's
    multiplicand can be signed and negated, and a slot can accumulate onto the previous product
    (mod 2^38). That is how the X step, the glyph x origin with the K4 slide, and the band DDA
    start F0 / G0 are summed without adders of their own.
  - One restoring divider (35 clocks) for 1/sv.
  - Products: k1², k1⁴ and the X phase step and origin; sh / sv; |k3|²; the glyph phase steps and
    origins; the drift step; the K4 glyph slide (|offset| × du); the wedge reciprocals sh/(A·sv),
    sh·sv/876 and the apex 0.894·sh; the flare C and taper T; the band DDA start C·u0², C·|2u0·s +
    s²| and T·|x0 − 960| (per-mode constants for the flare side and the x step).
  - Motion: the drift accumulator steps once per field; the K4 slide glides once per field.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the
  NEXT line in 59 clocks on its own serial multiplier:
  - Ty and Tz, and the band offset Tz − 178.2;
  - th_R and th_L, saturated to 4095;
  - tv;
  - the glyph sub-row and cell row.
- **Glyph mask engine.** From line-engine clock 2: 128 keys (16 columns × 8 blocks), one per clock,
  through a 7-stage hash pipeline. After each column's 8th block it writes the mask into the
  free line-parity bank. Banks swap at each line start.
- **Beam.** X phase (held left of 65 px), the band quadratic DDA (F 34 / G 26 bits signed, H 14
  bits), the glyph phase (28 bits: cell column . Q24).
- **Pixel path, 20 stages, one latency in every mode:**
  1. Video tap → 0..255; band compare; glyph x − 64 (± dilation). The cell's mask is read from
     EBR at the same time.
  2. Bend input and grey target; three block indices ((x − 64)·25 / 8192); band flag.
  3. Three glyph masks (plain, widened, narrowed) from the cell's mask; bend partial products.
  4. Bend product.
  5. Bent phase.
  6. Fold.
  7. Wedge key.
  8. Ground class and inks.
  9. One-shot counter and RC slew.
  10. Colour code and streak noise.
  11. Colour table.
  12–14. Soft box (EBR ring).
  15–17. Chroma box (EBR ring) and luma [1 2 1].
  18. Luma 13/16 and the blanking gate.
  19–20. Registers.
- **EBR.** Two rings: soft {Y, U, V} 256 × 32 and chroma {U, V} 256 × 20. Both carry
  `no_rw_check`; the reads sit at write − 13 / 5 / 1. The ring inputs are forced neutral outside
  the generated avid, and the sums are re-seeded in the zone. Glyph masks: 256 × 8, `no_rw_check`
  (the engine writes the other bank from the one the beam reads).
- **Contracts.**
  - damier's generated avid.
  - Virtual 1080-line coordinates from the timing ID.
  - Colours in the hardware convention (u = Cr, v = Cb).
  - No program-side pkg or build hook; all constants are inline.
- **Timing history.** The first router2 build ran at 76.7 MHz on HD Analog. The critical path was
  the video tap's clamp plus the grey-target adds in one stage, which now sit in stages 1 and 2.
  HD HDMI stalled under router2. The Osc bend's (os>>2)+(os>>5) add (a LUT with a duplicated sign
  input) became os>>2.

## Phase 2
Ron's notes 2026-10-30 (on v0.5.0):
- **K1 Flags:** 100 % more range on both sides.
- **K2 Band:** bipolar, so it flips which way the shape faces.
- **K3 Flare:** the same; enough range that it flips itself, equal on both sides of the knob.
- **K4 Speed:** move the glyphs side to side with a glide, with a good range of movement.
- **K5 Glyphs:** randomize the glyphs; make them morph and evolve in a crude way as you zoom.
- **P12 Spread:** scale horizontally as well as vertically. When the vertical scaling ends, the
  horizontal starts, smoothly: the diamond (and whatever else scales with Spread) scales first
  horizontally, then vertically.

**v0.6.0 (round 1), all six in:**
- **K1** 0.375 → 12 X's across (was 0.75 → 6), on a t² + t⁴ law that keeps the photo at 506.
- **K2** is now a bipolar taper of the band about the screen centre, straight at 512. Turned
  either way, the band widens on one side and closes into a pennant on the other. The band's base
  height is fixed at the photo's 178 lines.
- **K3** is bipolar: right of centre it flares open to the right (the photo at 768), left of centre
  to the left (mirrored, 255 = the photo's mirror). Straight at 512, up to 4× the photo's curvature
  at each end. K2 and K3 combine: opposite sides give an hourglass band.
- **K4** keeps the X drift and now also places the glyph grid: 2 px per knob step from home at the
  default 600, gliding there in about half a second. The continuous glyph scroll is gone.
- **K5** gives every cell its own glyph once it leaves 621 (±4). The 8 × 3 blocks re-roll as you
  zoom, about 0.4 block changes per glyph per knob step, differently in and out; back at 621 every
  cell shows the photo's glyph again.
- **P12** lower half: the diamonds and V wedges widen from 0.25× to the photo's width at 50 %. Upper
  half: v0.5's vertical growth to 1.8×. The handover at 50 % is continuous (both scales = 1).
- **Implementation:**
  - The band's exact (x − 42.5)² DDA and per-line divide became a per-pixel quadratic F(x) with a
    per-line offset.
  - The 12 run comparators became the block-mask EBR and its hash engine.
  - The sequencer grew from 17 to 26 product slots.
  - **Fit.** The first build came out at 7642 / 7680 LC (99 %), and router1 stalled on every seed.
    Standalone-yosys ablation found that the zone states' own adders cost 548 LUTs: the band-start
    sums, the signed F0 / G0, the slide add and the per-mode shift-adds. The knob-magnitude states
    cost 152 more. Fixed by:
    - moving all of those sums into the multiplier (signed, negating, accumulating slots);
    - one's-complement knob magnitudes;
    - one ×25 block index with edge logic for the dilation, instead of three;
    - a write-side counter instead of the hash pipeline's index registers.
    Standalone LUTs went from 4346 to 3677 (v0.5: 3184).
- **GHDL check** (720p60, decimation 1, Osc; `rtl6_osc_def.png`, `rtl6_osc_ext.png`, `cmp6.png`
  in the work dir):
  - The default frame matches v0.5 (MAE 1.0 against `rtl_osc_final.png`, re-rendered after the
    fit restructure; the differences are the block-grid glyph edges).
  - The first render put the glyph grid 176 px right of the photo, because K4's home was at 512.
    Fixed before the build: home is at 600.
  - K2 1023 / K3 252 / K5 900 / P12 150 renders the pennant band, the narrowed diamonds and
    re-rolled glyphs as designed.

## Status
- **v0.6.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Both the soft box and the chroma box resync per line (on top of the field reseed). Post-route Fmax: HD Analog s1 87.75, SD Analog s1 88.50, HD HDMI s1 86.95, SD HDMI s1 92.59, HD Dual s1 85.03, SD Dual s1 93.77 MHz; 6783-6806 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.6.0** built 2026-10-30 with **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh bunting`), all six on seed 1. Then the
  three HD bitstreams were re-seeded for HDMI pin skew and hand-installed, and the vmprog was
  repacked (`vmprog_pack.py --no-sign`). A plain rebuild lands on seed 1 again, so re-install
  these from `.heirloom_work/rb_bunting/skew/out/` or re-route them.
- Fmax (last post-route line):
  - HD Analog **s5** 89.65, HD HDMI **s7** 98.25, HD Dual **s2** 92.95 MHz (router1, hand-installed);
  - SD Analog 94.06, SD HDMI 91.11, SD Dual 89.85 MHz (seed 1).
- 6759–6772 / 7680 LC (88 %), 5 / 32 EBR, iceunpack OK × 6.
- `out/rev_b/ron/heirloom/bunting.vmprog` (422,687 bytes). Not flashed.
- **HDMI pin-skew screen** (`tools/hdmi_pin_skew.py`, HSYNC − clock / closest data bit, ns):
  - Seed 1 failed all three HD configs: HD HDMI +0.47 / −0.74, HD Analog −0.37 / −0.57, HD Dual
    −1.37 / +0.07.
  - Bunting's anchor is v0.5.0's HD HDMI s1, which Ron flashed without reporting lines. It
    re-routes byte-identical at **−0.94 / −1.21**.
  - No router1 HD HDMI seed of 1–28 reached the usual +1.0..+2.2 window, so HD HDMI ships s7
    (**−0.86 / −1.91**), the closest to the anchor.
  - HD Analog ships s5 (−1.37 / −1.03); HD Dual ships s2 (−1.81 / −1.06).
  - **A/B spare if HD HDMI shows lines:**
    `.heirloom_work/rb_bunting/skew/bunting_v060_hdhdmi_r2s9.vmprog` is the same build with
    HD HDMI router2 s9 (+1.58 / −0.78, 75.83 MHz).
- **GHDL check:** see [Phase 2](#phase-2).
- **v0.5.0** built 2026-10-27 with **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh bunting`), all six configs on **seed 1**.
- Fmax (last post-route line): HD Analog 98.89, HD HDMI 108.84, HD Dual 108.08, SD Analog 86.37,
  SD HDMI 103.24, SD Dual 110.78 MHz.
- 6315–6368 / 7680 LC (82–83 %), 4 / 32 EBR, iceunpack OK × 6.
- `out/rev_b/ron/heirloom/bunting.vmprog` (398,599 bytes). Not flashed.
- router2 on an earlier netlist: HD Analog 76.7 MHz, and HD HDMI stalled. Rebuild with router1.
- **Verified in GHDL** at 720p60, decimation 1, default knobs:
  - Osc matches the float proto and the integer model (MAE 2.3 after the 5-px filter shift);
  - Video (selfie) bends the X's and fills the grey glyph ink;
  - motion at K4 = 1023: the X field moves 10 px right and the glyphs 5 px left over 4 fields;
  - NTSC shows the centre 4:3 crop, each field the full picture.

## Deviations from the photo
- **Top end.** White is capped at output luma 777 (the HD HDMI bar fix), so the white ground reads
  light grey. Yellow and cyan are compressed to 715 / 650 (photo 749 / 675) to stay distinct from
  it. Red, blue, black and grey keep the photo's luma. The RTL's region means sit about 15–20 RGB
  levels below the photo, uniformly.
- **Vertical edges** are 1 line (the photo's are 2–4); a vertical filter would need line buffers.
- **Causal filters.** The soft and chroma boxes shift the picture about 5 px (720p) / 8 px (1080)
  right. Each line starts with a short dark ramp, like the photo's dark left border. S10 Hard moves
  the picture by half a box.
- **Glyph runs** sit on a uniform block grid since v0.6 (8 blocks of 36 px): the top-row gap
  starts at 108 px (photo 110), the middle row at 108 px (photo 104). The ink widening and narrowing
  scale with K5.
- **The grey's rise** uses a single-shift RC: a time constant of 32 virtual px at 1080 and SD,
  24 at 720p (photo about 25). The streak noise is a hashed realisation.
- **Defaults** land on 2.034 X's across (photo 2.033) and 4.264 glyph columns (photo 4.268). The
  edges are fitted straight; the photo's wander ±5 px locally.
- **Osc** sways the X edges by about ±9 px at the default Bend; the photo is a still. At default
  Speed the X's drift right at about 38 px/s; the glyph grid stands at the photo's position.
- **Resolution.** 720p is the 1080p picture. On SD (4:3) the program shows the centre 1440 virtual
  px. The line-start hold and the band's flat left end are off-screen, and the outer glyph columns
  are cut.

## Working files
`.heirloom_work/rb_bunting/`:
- `ref.jpg`: the photo.
- `notes.md`: look, measurements, proto, controls, model, RTL, build.
- `proto.py` (fitted), `protok.py` / `proto_final.py` / `proto_final.png` (the knob-level proto),
  `sheet.py` / `sheet.png` (the control sheet), `p12strip.png`.
- `imodel.py`: the integer model of the RTL (`im*.png`).
- `rtl_osc_final.png`, `rtl_vid_final.png`, `rtl_mot_a/b.png`, `rtl_sd1.png`: GHDL renders.
  `cmp_final.png`, `cmp_motion.png`, `cmp_sd.png`. `simx.sh`: a tagged sim wrapper.
- `build1.log` (router2), `build3.log` (router1, final). `bunting_v050_built.*`: the built sources.
- `old_v010_*`: the replaced v0.1.0.
- Phase 2:
  - `proto6.py`, `sheet6.py` and `sheet6.png`: the v0.6 float proto and its control sheet.
  - `hashcheck.py`: the glyph re-roll hash and its correlation check.
  - `simy.sh`: simx.sh with `PROGDIR`.
  - `rtl6_*.png`, `cmp6.png`: the GHDL renders. `build6.log`: the build.
