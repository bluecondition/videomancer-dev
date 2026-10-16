# Teletext — summary

Catalogue combo #47 (seed 20260920): datamosh bars + ASCII / block-dither
glyphs + scanline shear. v0.1.0 (2026-09-11) was flashed and liked
("teletext is good", 2026-09-12), with no later hardware notes. v0.2.0
(2026-10-12) is the first finalizing pass (Tier 1 of the review below). It
is **not flashed yet**. Backup of v0.1.0: `.teletext_work/v010_backup/`
(sources + vmprog; also git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Bars | fraction of glyph bands displaced (datamosh), re-rolled every frame |
| K2 Distance | band displacement, up to ±1020 px; wraps inside the picture |
| K3 Cell | glyph cell 8 / 16 / 32 / 64 px (labelled; default 8) |
| K4 Contrast | luma → glyph density; 100% maps white to the densest glyph, up to 400% |
| K5 Shear | per-line sideways shear, ±30 px |
| K6 Rate | Wave: wave speed. Tear: fraction of lines torn |
| S7 Ink | white glyphs on black / black glyphs on white |
| S8 Colour | mono glyphs / glyphs take the cell's colour |
| S9 Mosh | bands displace the video / bands displace whole glyph cells |
| S10 Shear | Wave (triangle) / Tear (random per line) |
| S11 Glyphs | ASCII ramp / block-dither set |
| P12 Reveal | crossfade from the moshed, sheared video to the glyph rendering |

## v0.2.0 changes (Tier 1)

Bugs:
- **Right edge of the glyph layer was background.** The glyph position was
  `x − delay`, with x reset to 0 the moment active video ended. The last
  ~delay px of every line therefore drew as paper. x now keeps counting
  through blanking.
- **Top glyph row showed the previous frame's bottom.** Each band read the
  bank written one band earlier, and band 0 found the previous frame's last
  odd band there. Each band is now sampled on the line directly above it.
  The grid starts two lines down, so the top band samples the first real
  line. Glyphs also sit about a cell higher than before, about 1 line under
  their source line instead of a whole cell + 1. That shows at mid Reveal.
- **The glyph row/bank came from a separately delayed line counter**, so at
  large shear the last px of a line could pick up the next line's row. Each
  ring word now carries its own line's glyph row, bank and valid bits.
- **SD right edge:** a new line's shear was applied at hsync, which comes
  only ~16 clocks after SD active video. The last px of a sheared SD line
  read the ring at the next line's delay. It now switches as the new line's
  first pixel enters the ring.
- **Cells mosh read stale cells.** Displaced cells past the line's real
  cell count (240..255 at 8 px HD, and more at larger cells) read cells left
  over from another Cell size. They now wrap on the line's own cell count.
- **SD partial cell:** the last, partial cell of an SD line (32/64 px) was
  never written, so it showed stale data. It is now written at the line's
  last pixel.
- **Contrast 100% did not reach the densest glyphs.** White (876 above
  black) mapped to glyph 13 of 15. Luma above black is now scaled ×1.156
  first.
- First output line: it showed a stale line-buffer bank (the previous frame's
  bottom). It is now black.
- The ring delay is now a slice instead of a narrowing signed `resize`.
  Synthesis truncates either way, so hardware is unchanged, but GHDL
  simulation folded delays 32..63 and would have misrendered any sim.

Hardware contracts / quality:
- **Edges (shelf note "moshed or sheared reads off the line paint black").**
  Moshed reads wrap inside the C_EDGE (12 px) inset, like blockcut. Sheared
  edges fill with the line's own inset edge pixel: the last column is held
  for 64 clocks after the line, then the first column leads into the next
  line. The capture's black side columns are never read. On the glyph layer,
  a sheared edge shows paper, so it is seamless.
- **HD HDMI bar conditioning** (damier v0.4.4): [1 2 1] on Y/U/V before the
  gate, and the luma cut 64 + 13/16 (y − 64) applied to the glyph levels
  only. White ink / white paper is 776 (was 940), so the video keeps its own
  level at low Reveal. Glyphs are full-swing pixel edges moved every line by
  the shear. This softens 1-px glyph detail (the block-dither set at 8 px in
  HD is mostly 1-px checker) and greys the white. It is the one Tier 1
  change that alters the look; drop it if the glyphs read too soft and HD
  HDMI does not bar without it.
- Colour + black ink: the dark coloured ink is luma 168 instead of 64, so its
  colour survives RGB clipping (coloured white ink 776 likewise keeps more
  chroma than 940 did).
- K3 has value labels and a 64 px cell (30 columns in HD, close to real
  teletext's 40). The firmware loads a labelled default as the label INDEX;
  0..3 is honoured as an index only until the knob first reads a position.
  Default and presets keep their v0.1 sizes ("Torn Blocks" 700 → 384 = 16 px).
- K2 display corrected to 0–1020 px (it said 512; the offset is ±1020).
- S10 labels Wave / Tear (it was "Sine", but the wave is a triangle).
- `no_rw_check` on every RAM. The cell RAM reads both banks every pixel; the
  bank being written is never the one selected.

Latency 53 → 59 clocks, all modes (+1 inset clamp, +1 cell wrap, +1
contrast scale, +3 conditioning).

## Build (v0.2.0)

Seed 1 on all six configs (router2), 2026-10-12, log `.teletext_work/build3.log`:
HD Analog 81.2, HD HDMI 84.9, HD Dual 84.6 MHz; SD Analog 83.2, SD HDMI 81.6,
SD Dual 82.9 MHz. ~4,790–4,840 LC (62–63%, v0.1 was 56%), 25/32 EBR (same as v0.1).
All 9 RAMs report "don't care on collision" (no transparency emulation).

## Review backlog (not done; Ron's call)

Tier 2 (look changes):
1. P12 Reveal as a per-cell dissolve: each cell flips from video to glyph at
   its own hashed threshold, like a teletext page loading. The mid range
   then reads as a patchwork, not a double exposure. The shelf to-do asked
   whether the middle of Reveal reads well.
2. True cell averages: accumulate every row of the band (pipelined
   read-modify-write, +2 EBR) instead of sampling one line. Calmer glyphs, and
   thin features are no longer missed or twinkling. Also averages the chroma.
3. S8 Colour → Mono / Teletext: classify each cell's colour into the 7
   broadcast teletext colours (R/G/B/C/M/Y/W) instead of the cell's own tint.
4. Bars hold: bars re-roll every frame now (60 Hz jitter). Hold a pattern for
   a few frames, with the rate on K6 or a held-frames curve.

Tier 3 (new features):
5. Sixel mosaic set: real teletext 2×3 block graphics per cell (each
   sub-block thresholded), as the S11 alternative or a third set.
6. Continuous Cell size (DDA cell counters, any 8..64 px).
7. Square cells under SD interlace (count frame lines).
8. Generated avid, only if Colour mode shows thin coloured lines on HD HDMI.
9. Capture defaults + presets from the device (`./capture_preset.sh`).

## Hardware checks wanted (v0.2.0)

- Glyphs reach the right edge (v0.1 left ~33 px of paper there).
- No black at the sides with Shear and Bars up (fill / wrap).
- Top glyph row matches the top of the picture.
- HD HDMI with Shear moving: no thick bars. Are the glyphs too soft?
- 64 px cells, and Cells mosh at each size.
