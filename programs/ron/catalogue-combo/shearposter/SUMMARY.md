# Shearposter — summary

Catalogue combo #110 (seed 20260925): pixelation + scanline shear / tearing +
posterization. v0.1.0 was flashed and liked ("shearposter is good",
2026-09-12). v0.2.0 (2026-10-12) is the first finalizing pass, built from a
review of v0.1.0. **It is not flashed yet.** The v0.1.0 sources and vmprog are
in `.shearposter_work/v010_backup/` (and in git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Cell | cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px, labelled, deadbanded |
| K2 Rows | cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines, labelled, deadbanded |
| K3 Dither | 2×2 ordered dither: Off / 2 / 4 / 8 / 16 / 32 / 64 px / Cell (one threshold per cell; it moves with the shear) |
| K4 Rate | Zigzag: waves down the frame, squared. 0 = the whole frame sways as one; the default gives 38 waves on a 1080 frame, as v0.1; the top is an 8-line zigzag. Tear: share of lines torn, squared, 1.6–100%. Glided |
| K5 Luma | luma levels 1–8 bits, labelled |
| K6 Chroma | chroma levels 1–8 bits, labelled (1 bit = −/grey/+ per channel) |
| S7 Shear | Zigzag (a triangle wave rolling down the frame, so straight shear segments) / Tear (a random offset per line, new every field) |
| S8 Shift | Free / Cells (offsets rounded to whole cells: the cell grid stays on the screen and the picture steps a cell at a time) |
| S9 Cells | Lean (every line takes its own offset, so cells lean) / Slide (one offset per cell row, so rows slide as rigid strips) |
| S10 Edges | Fill (the line's own edge pixel) / Mirror |
| S11 Colour | Together / Split (chroma shears the opposite way, so colour tears away from the picture) |
| P12 Shear | amount, glided, squared: width/64 at 0 (30 px HD), v0.1's full ±255 px at ~50%, width/2 (±958 px HD) at 100% |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Grey turned pink.** The chroma posterizer put its levels at (k + ½) ×
  step, so grey (512) came out as U = V = 576 at the default 3 bits. That is a
  pink/magenta cast over every neutral, and 768 at 1 bit. It is now a centred
  mid-tread quantizer (levels at 512 ± k × step, clamped 64..960). Greys stay
  grey at every depth (checked numerically). This is flutter v0.2's fix.
- **Black was lifted.** The luma posterizer lifted black (64 → 256 at 1 bit)
  and topped out at 768, so 1 bit gave two greys. Luma is now normalized to
  the video range, cut to L bits, bit-replicated, and scaled back. Black (64)
  and white (942) are levels at every depth.
- **The picture sat a whole cell row low.** Each cell row showed the row
  above's capture: up to 32 lines low at Rows 32. The top cell row showed the
  previous frame's bottom. Rows are now captured on their own first line and
  shown from the next line on (the snowglobe v0.2 scheme). The shift is 1
  line at every Rows setting.
- **Dead switch combinations.** S8 Shift did nothing when S9 was on Stay.
  Mathematically, "grid fixed to the screen" and "shift in whole cells" are
  the same picture, so (Cells, Stay) could never differ from (Cells, Shear).
  S9 is now Cells Lean / Slide (rigid row strips). That is new, and it is
  distinct in every S8 position. v0.1's Stay look (cells fixed, the picture
  sliding under them) is close to S8 Cells; Tier 2 can bring it back.
- **S11 Chroma Clean was the same as K6 at 8 bits**, so it switched K6 off
  (a gated control). S11 is now Colour Split.
- **Cells snapping floored instead of rounding.** Small negative offsets
  jumped a whole cell left, so a gentle ±16 px wave over 64-px cells became
  0 / −64. It now rounds to the nearest cell.
- **Every cell sampled its first pixel.** With large cells, the leftmost
  column of cells held the capture's black blanking columns: a black stripe
  up to 64 px wide. Cells now sample their centre, and every read is kept
  inside the 12-px inset.
- **The dither row flipped for the last ~5 pixels of every line.** The line
  counter steps at the avid fall while the pipe is still draining. The dither
  row is now latched at the line start.
- **K1/K2 had uneven steps.** 6 sizes were spread over 8 knob zones, and the
  display showed a linear 2–64 px / 1–32 ln value that didn't match. They are
  now 8 labelled octaves, with the labelled-default index trap handled.
  K5/K6 are labelled too, and all five stepped knobs have hysteresis.
- **K4 Rate had only 32 steps.** One step moved the bottom of a 1080 frame by
  4 waves, so pot noise at a step edge jumped the picture. It is now
  10-bit, squared, and glided.
- **"Sine" was a triangle wave.** It is relabelled Zigzag. The triangle stays,
  because its straight segments are a literal shear.
- **Tears came from a 16-bit LFSR** with overlapping bit fields. They now come
  from xorshift32 (dropout lesson).
- "Retro" and "Colorizer" are not valid categories. The categories are now
  Pixel / Glitch / Color.

Hardware contracts:
- **Edges:** Black → Fill (Ron's 2026-09-13 edge-fill rule, which missed
  this program), and Mirror reflects about the 12-px inset (no black seam).
- **U/V parity swap on replayed rows** (drape v1.3). Every output row is a
  stored line.
- **HD HDMI bar guard:** [1 2 1] on Y/U/V plus luma scaled to white ~777,
  before the gate. Posterized, dithered steps that tear every field are the
  trigger profile. Side effects: whites ~17% dimmer, and a 1-px dither would
  be flattened by the [1 2 1], so the finest dither is now 2 px. The default
  moved from 1 px to 2 px.
- Banks, offsets, swap and dither row are latched per line at the avid rise.
- Shear arithmetic moved off the pixel path: a per-line serial shift-add
  multiply (15 clocks in hblank) and a free-running frame multiplier.
- `no_rw_check` on all six line-buffer RAMs.
- Latency 16 → 18 clocks, the same in all modes; blanking-gated.

Controls (layout change):
- **P12 is now Shear**, the headline (the standing rule: the performance
  control goes on P12). It is glided (¼ of the gap per field), squared, and
  has a floor, so every switch still shows at slider 0. Its top is a designed
  climax: half the screen width of tear. v0.1's P12 (dither pitch in 4 steps)
  moved to K3.
- The defaults keep v0.1's look on HD: 8-px × 4-line cells, ~74 px of zigzag
  at 38 waves a frame, 3/3 bits. The dither is 2 px (was 1 px).
- Presets: the three v0.1 looks are remapped to the new controls (Torn Mosaic
  = 200 px, 49% tears, 16×8 cells snapped; Sliding Cells now uses Slide; Two
  Bit Wave). There are two new ones, Misprint (Slide + Colour Split + 1-bit
  chroma + cell dither) and Shredder (95% tears at ±750 px, mirrored). All are
  hand-written; recapture them with capture_preset.sh.

## Hardware check list (v0.2.0)

1. Neutrals: greys and whites should carry no pink cast at any K6 depth.
2. Rows 32+: the cells should line up with the picture, with no row-high
   offset and no foreign strip at the top.
3. P12 sweep: a gentle sway at 0, v0.1's look at ~25%, and edge-to-edge
   tearing at 100%. No black side bars in Fill.
4. S9 Slide with Rows 16+: rigid strips. S11 Split: colour tears the
   opposite way.
5. K3 Cell with Cell 16–32 px: dithered whole cells that move with the shear.
6. HD HDMI: any thick bars? (The guard is in from the start.) Are the whites
   too dim?

## Next tiers (offered, not done)

Tier 2 (look, Ron's call):
- **Roll speed.** The zigzag rolls at a fixed ~1.4 s a cycle, as in v0.1.
  Option: P12 also speeds the roll, so the climax churns.
- **Tears that hold.** Tears re-roll every field (60 Hz per-line shimmer).
  VHS-style tears would persist a few fields, drift, and bunch into bands.
- **Wrap edges:** the picture wraps round like a horizontal-hold slip. It
  needs a third edge mode or a switch.
- **Stay** (v0.1's S9): bring it back if it's missed.
- **SD interlace:** Rows count per field, so SD cells are twice as tall in
  frame terms.

Tier 3 (bigger, ask first):
- Cell averaging (box average instead of point sample, the stencil EBR ring).
  The cells would be smoother and less flickery under the shear.
- A retro palette quantizer (the cartridge/croupier LUT) as a posterize mode.
- Per-cell random displacement (mosh-like), on top of the per-line shear.
- Presets captured from hardware; v1.0.0 sign-off; commit.

## Build

`./build_combo.sh shearposter` → `out/rev_b/ron/catalogue-combo/shearposter.vmprog`.

v0.1.0 (2026-09-12): HD 89.8 / 88.8 / 87.7 MHz, 2,897 LC (38%), 18 EBR, seed 1.

v0.2.0 (2026-10-12, router2 default): all 6 configs on seed 1. HD Analog 83.9 /
HD HDMI 90.1 / HD Dual 86.2 MHz; SD 88.3 / 88.8 / 93.1 MHz. ~3,905–3,952 LC
(51%), 18/32 EBR. Standalone yosys: `emulate_transparency` = 0. TOML converted
fresh (labels checked in program_config.bin). NOT FLASHED, NOT COMMITTED.
