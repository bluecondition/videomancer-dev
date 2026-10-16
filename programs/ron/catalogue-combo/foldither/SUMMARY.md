# Foldither — summary

Catalogue combo #117 (seed 20260926): ordered dithering (Bayer) + mirror /
kaleidoscope. v0.1.0 was flashed and liked ("foldither is good",
2026-09-12). v0.2.0 (2026-10-13) is the first finalizing pass, built from a
review of v0.1.0. The v0.1.0 sources are in git 3fa438d and in
`.foldither_work/foldither_v010.{vhd,toml}`; its vmprog is in
`.foldither_work/foldither_v010.vmprog`.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Segment | pane width 1024 / 512 / 256 / 128 / 64 px |
| K2 Slide | the panes slide across the picture, bipolar. Squared, with a centre detent. Top ~7.75 px a frame (v0.1: 4) |
| K3 Skew | per-line lean of the panes, bipolar. Squared, with a centre detent. 45° at ~35% from centre, top ~7.75 px a line |
| K4 Bits | luma levels 1–6 bits. Black and white are always levels |
| K5 Dither | dither amount, 0–100% of a step |
| K6 Matrix | 2x2 / 4x4 / 8x8 Bayer / Diagonal (45° line screen) |
| S7 Fold | Mirror / Repeat (tiles are two segments wide) |
| S8 Pattern | Source: the dither pattern folds with the picture / Screen: it stays put |
| S9 Chroma | Clean / Dithered (quantized like luma, centred on grey) |
| S10 Alt Neg | alternate panes (tiles in Repeat) negative |
| S11 Alt Bits | alternate panes 2 bits coarser (2 bits finer at 1–2 bits) |
| P12 Panes | pane width ÷ 1 / 2 / 4 / 8 |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **S9's labels were inverted.** The switch read "Dithered" when chroma was
  clean. They are now Clean / Dithered, and the default is still Clean, so
  the default look is unchanged.
- **Dithered chroma had no neutral level.** Levels sat at (k + ½)·step, so
  grey became U = V = 576 at 3 bits and 768 at 1 bit, a pink cast over
  everything. It is now a centred mid-tread quantizer (levels at 512 ± k·step,
  clamped 64..960), so greys stay grey at every depth and dither setting.
- **Luma levels didn't span the video range.** 1 bit gave two greys
  (256 / 768), 2 bits lifted black to 128, and the top level at 3 bits was 960
  (superwhite). Luma is now normalized, cut to L bits, and bit-replicated
  back. Black (64) and white (942) are always levels, so 1 bit is true
  black/white dither.
- **The dither was biased upward.** v0.1 added 0..D·step and then half a
  step. At the default Dither about 14% of pure-black pixels speckled up to
  the next level, and at full dither everything brightened by half a step.
  The dither is now centred, with (N−1)/N scaling. Full dither is
  mean-preserving (within 8 codes at 1 bit, ≤3 at 2–6 bits) and spans black
  to white evenly.
- **Wrong power-on defaults** (the labelled-default index trap). Segment
  came up at 1024 px instead of 512, Matrix at 2x2 instead of 8x8, and Depth
  at 1 instead of 2. A value below the label count is now read as the label
  index.
- **Segment switched at different points than its display.** The code split
  the knob into eighths (1024 got 25% of the travel, 128 and 64 got 12.5%),
  while the firmware shows five even label zones. Bits had the same problem
  against its 1..6 number display. Both now decode the firmware's even zones,
  and Bits is labelled.
- **Repeat showed black bands.** At HD with Segment 1024 there was a 576 px
  black band left of centre. On SD, Repeat at 512 / 1024 and Mirror at 1024
  also blacked out. Reads that leave the line now reflect back inside it,
  inset 12 columns to skip the capture's black edge. The repeat tile is now
  centred on the picture's centre.
- **Video lagged avid by one clock.** The first column was blanking and the
  last column was lost. The alternate-pane flag was one pixel early, which
  left a wrong-polarity column at every Alt Neg pane edge. Re-timed stage by
  stage.
- **The dither row jumped at the right edge.** The line counter advanced at
  the avid fall while the last ~15 pixels were still in the pipeline. That
  strip used the next line's Bayer row. The row is now latched per line.
- **Alt Drop was wrong in Source mode and dead at 1 bit.** In Source mode it
  re-dithered alternate panes with the screen pattern, so the pattern
  stopped folding there. At 1 bit it couldn't drop. There is now one
  quantizer, after the fetch (below), so Source-mode alternate panes keep the
  folded pattern. S11 is now Alt Bits: 2 bits coarser, or 2 bits finer at 1–2
  bits, so it always shows.
- **S8 was dead for the Lines matrix.** A horizontal line screen has no x to
  fold, so Source and Screen differed only by a 1-line shift. Lines is now a
  45° Diagonal screen. In Source mode it turns into chevrons at the mirror
  seams.
- **Alt Neg inverted to superwhite.** `not y` sent black (64) to 959. It is
  now 1006 − y (64 ↔ 942).
- **Repeat + Alt Neg flipped half of each tile.** The parity was taken at the
  segment, but a tile is two segments wide. Whole tiles now alternate.

Hardware contracts:
- U/V are swapped on rows whose hsync→avid parity differs from the row
  above, because each row replays the row above (drape v1.3, flutter).
- The write bank, dither row, and pane offset are latched per line at the
  avid rise. The read tail ran past hsync on PAL's 12-clock front porch.
- The fold centre comes from a field-latched width. A per-line width moved
  every pane edge by a pixel whenever an analog line ran ±1 px long.
- HD HDMI bar conditioning: luma is scaled to white ~775 (damier), and U/V
  get [1 2 1]/4. **Luma is deliberately NOT smoothed.** A [1 2 1] on Y turns
  a 1-px checker into flat grey, which would erase the dither. If the bars
  show, see Tier 2 B.
- no_rw_check on all six line-buffer RAMs. One latency (20) in all modes,
  blanking-gated before and after the smoothing.

Controls:
- **Slide and Skew get a 16-count centre detent and a squared curve.** Pot
  noise at centre used to make the picture creep, and the panes leaned by
  ~67 px over the HD frame. v0.1's Skew spent 87% of its travel past 45°;
  45° is now at ~35%. The Skewed Mirror preset values moved (600→672,
  800→910) to keep its v0.1 speeds.
- Segment, Bits, Matrix and Panes have zone hysteresis: a zone changes only
  once the knob is 12 counts inside it.
- Renames: P12 Depth → **Panes** ×1/×2/×4/×8 (what it does, see below);
  S8 Dither → **Pattern** (two controls were both called Dither);
  Lines → Diagonal; Alt Drop → Alt Bits.

Optimizations:
- **The nested folds are one fold.** A halving fold of a triangle wave is the
  same triangle wave at the smaller width (tri(tri(a, W), W/2) =
  tri(a, W/2)), and nested repeats are one modulo. v0.1's 4 fold stages (8
  pipeline stages, 4 constant sets) did the work of one. Checked
  exhaustively against v0.1's cascade (every Segment × Depth × mode): 0
  mismatches. So "Depth" never nested anything; it divided the pane width,
  hence the rename.
- **One quantizer instead of two.** v0.1 quantized on the write port for
  Source mode and again after the fetch for Screen mode. v0.2 stores the raw
  picture and quantizes once after the fetch. Source mode takes the Bayer
  cell from the fetch address and line − 1, which is the same pattern.
- The dither scale is a shift (the step is a power of 2), so the vblank
  multiply sequencer is gone.

## Build

`./build_combo.sh foldither` → `out/rev_b/ron/catalogue-combo/foldither.vmprog`.
v0.2.0 (2026-10-13): 6/6 on seed 1 with router2. HD 93.3 / 92.1 / 81.4 MHz
(Analog / HDMI / Dual); SD 86.7 / 96.3 / 90.3 MHz. 3,962 LC (52%), 18 EBR,
and no transparency emulation (standalone yosys `emulate_transparency`
count 0). v0.1.0 was 4,080 LC at HD 86.4 / 81.7 / 82.5 MHz. The build log is
`.foldither_work/build_v020.log`.
NOT FLASHED, NOT COMMITTED.

## Hardware check (v0.2.0)

1. Default look vs v0.1 should be the same family. Blacks are clean (no
   speckle) and whites are dimmer (~775, the bar conditioning).
2. Bits 1 bit: true black/white dither (v0.1 gave two greys).
3. S9 Dithered: no pink cast on greys. The label now matches.
4. Repeat at Segment 1024 on HD: no black band.
5. Alt Neg: clean pane edges. In Repeat, whole tiles alternate.
6. Slide / Skew: they stop dead at centre. Skew is fine near centre.
7. Matrix Diagonal: S8 Source gives chevrons, Screen gives straight diagonals.
8. Alt Bits at 1 bit: alternate panes now at 3 bits.
9. **HD HDMI: watch for thick bright bars** at 1–2 bits with Slide moving.
   This is the known trigger (1-px full-swing steps in motion), and luma is
   only scaled, not smoothed.
10. Power-on defaults: 512 px, 3 bits, 8x8, Panes ×2.
11. No thin coloured lines over coloured areas (U/V parity).

## Tier 2 (recommended next; needs your pick)

- **A. Continuous P12.** K1 and P12 set the same exponent today (pane =
  2^(10 − K1 − P12)), and P12 is a 4-step switch. Two ways to make the
  headline slider a real sweep:
  (a) *Crop*: the pane narrows continuously from Segment to Segment/8 at 1:1
  scale. This needs reflector v0.2's per-pixel up/down counter (any width,
  per-line start phase).
  (b) *Zoom*: panes multiply continuously, each still showing the whole
  Segment window (minified). A per-pixel DDA on the fold input; the
  power-of-2 fold stays. This is the true kaleidoscope multiply.
- **B. Grain (dither dot size 1–4 px).** At HD a 1-px Bayer reads as fine grey
  texture, and 2–4 px dots read as retro dither. 2-px grain is also the
  CGA-proven cut for HD HDMI bars, and it would make the full [1 2 1] luma
  guard safe to add. It needs a control slot: fold it into K6 (8 positions:
  4 matrices × 2 sizes) or give it S11.
- **C. Skew pivot at the screen centre.** Panes would rotate about the middle
  instead of swinging from the top line. One per-frame product.
- **D. Full negative** on Alt Neg (chroma inverted too), or a third state.
- **E. Segment as a fraction of the width**, so SD and HD show the same pane
  count. It pairs naturally with A(a).
- **F. Subject at the centre.** The mirror seam sits at the screen centre
  showing the window's edge, and the centred subject appears twice at ±W/2.
  An option would put the subject (unmirrored) at the centre.

## Tier 3 (polish / later)

- Interlaced SD: key the screen dither and the skew on the true frame line.
  Today both fields use the field line, so 480i/576i repeat each dither row
  twice and the skew stair-steps in pairs.
- Chroma is stored at 2-px resolution. Full resolution costs +4 EBR (18 →
  22 of 32).
- The top row shows the previous frame's last line (1-line replay; v0.1 did
  the same).
- Temporal dither: alternate the threshold offset per frame for more
  apparent levels.
- Presets are still hand-written (4 of 8). Capture them from the device once
  the look settles.
