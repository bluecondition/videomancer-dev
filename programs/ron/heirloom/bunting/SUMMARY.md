# Bunting — bow-tie pennants under a grid of keyed glyph marks

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Heirloom #35, from
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
   - th = 0.106 + Ty / (A·s), with A = 312 / 306;
   - tv = (Ty − 70.1)·s / 876;
   - s = the P12 spread.
3. **Band.** S = (x − 42.5)² exactly, in half-px² (S += D; D += 2k²). The pixel is in the band when
   the line is within z0 of the axis, or when S > R2, with R2 = 4·(Tz − z0)/kz per line.
4. **Ground.** In the band: cyan / white. Outside: white / yellow. S8 swaps yellow and cyan.
5. **Glyphs.** A 24-bit glyph phase plus a per-line row index. Three masks come from 12 comparators
   against the row's runs: base, widened by 35/4096 of the pitch, and narrowed. S9 selects mirrored
   runs.
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
| K1 Flags | the bow-tie X's | oscillator frequency: 0.75 → 6 X's across, square law (2.03 = the photo at 506) |
| K2 Band | the middle cyan band | its half-height at the left edge, 0 → 360 lines (178 at 507) |
| K3 Flare | the band opening to the right | curvature 0 → 3× the photo, square law (the photo at 591); below 6 % it is straight |
| K4 Speed | motion | bipolar, still in the centre ±8. The X's drift sideways (4 virtual px / field at full); the glyph grid scrolls the other way at half the speed. Default 600 ≈ 38 px/s |
| K5 Glyphs | the glyph grid | 1.5 → 9 columns, square law (4.26 at 621), scaled about the grid centre |
| K6 Bend | the X edges | how far the video (or Osc) bends the X phase: 0 → 0.25 period at full white (0.083 ≈ 78 px at the default 340) |
| S7 Shot | the black one-shot in the grey glyph ink | On (the photo) / Off (plain grey) |
| S8 Swap | yellow and cyan | exchanged; the inks follow (red on yellow, blue on cyan) |
| S9 Flip | the glyph marks | mirrored |
| S10 Edges | the photo's horizontal softness | Soft (the photo) / Hard |
| S11 Source | grey ink target, X bend | Video (default) / Osc |
| P12 Spread | the bow-ties (headline) | wedge opening 0.4 → 1 → 1.8×. 0 % = furled flags (flat cyan lozenges, yellow nearly solid with thin white spikes); 50 % = the photo; 100 % = wide cyan fans, a white field with a yellow zig-zag string |

Defaults: K1 506, K2 507, K3 591, K4 600, K5 621, K6 340, P12 512, every switch at label 0.
Glides (1/8 of the gap per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12.

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
  - One serial shift-add multiplier: 20 × 18 bits, 18 iterations, 17 product slots, one every
    32 zone clocks, with the operands pre-muxed a clock ahead.
  - One restoring divider (35 clocks) for 1/s and 4/kz.
  - Products: the X phase step and origin, the spread, the wedge reciprocals, kz, z0, the glyph
    phase steps and origins, the drift and scroll steps.
  - Motion: the drift and scroll accumulators step once per field.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the
  NEXT line in 78 clocks on its own serial multiplier:
  - Ty and Tz;
  - th_R and th_L, saturated to 4095;
  - tv;
  - R2 = (Tz − z0)/4 · ikz >> 6, saturated, or "whole line in the band";
  - the glyph row and its four run boundaries (S9 flip table).
- **Beam.** X phase (held left of 65 px), the exact band parabola DDA (24-bit S, 17-bit signed D),
  the glyph phase.
- **Pixel path, 20 stages, one latency in every mode:**
  1. Video tap → 0..255; band compare; glyph phase ± dilation.
  2. Bend input and grey target; three glyph masks; band flag.
  3–4. Bend product (nibble partials).
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
- **EBR.** Two rings: soft {Y, U, V} 256 × 32 and chroma {U, V} 256 × 20, 4 EBR in all. Both carry
  `no_rw_check`; the reads sit at write − 13 / 5 / 1. The ring inputs are forced neutral outside
  the generated avid, and the sums are re-seeded in the zone.
- **Contracts.**
  - damier's generated avid.
  - Virtual 1080-line coordinates from the timing ID.
  - Colours in the hardware convention (u = Cr, v = Cb).
  - No program-side pkg or build hook; all constants are inline.
- **Timing history.** The first router2 build ran at 76.7 MHz on HD Analog. The critical path was
  the video tap's clamp plus the grey-target adds in one stage, which now sit in stages 1 and 2.
  HD HDMI stalled under router2. The Osc bend's (os>>2)+(os>>5) add (a LUT with a duplicated sign
  input) became os>>2.

## Status
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
- **Glyph runs** are the measured positions, not a perfect uniform grid: the top-row gap starts at
  110 px, the middle row at 104 px. The ink widening and narrowing scale with K5.
- **The grey's rise** uses a single-shift RC: a time constant of 32 virtual px at 1080 and SD,
  24 at 720p (photo about 25). The streak noise is a hashed realisation.
- **Defaults** land on 2.034 X's across (photo 2.033) and 4.264 glyph columns (photo 4.268). The
  edges are fitted straight; the photo's wander ±5 px locally.
- **Osc** sways the X edges by about ±9 px at the default Bend; the photo is a still. At default
  Speed the X's drift right at about 38 px/s and the glyphs scroll left at about 19 px/s.
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
