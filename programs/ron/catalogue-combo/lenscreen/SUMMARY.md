# Lenscreen: summary

Catalogue combo #106 (seed 20260925): composite luma/chroma crosstalk +
halftone + a sphere or tunnel row lens. It was derived from snowglobe. Ron
flashed v0.1.0 in September 2026 and said "lenscreen is good" (EXPLORE_LIST,
2026-09-12). v0.2.0 (2026-10-13) is the first finalizing batch. It comes from
a code review of v0.1.0. The v0.1.0 sources are at git 3fa438d.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Bulge | lens strength, bipolar (pinch .. bulge), glided |
| K2 Radius | lens radius, 3–140% of the half-height (was a fixed 16–783 lines; HD is about the same), glided |
| K3 Cell | halftone cell 4 / 8 / 16 / 32 px, deadbanded |
| K4 Dot | circle / diamond / square / line, deadbanded |
| K5 Cross-Y | chroma leaking into luma (dot crawl): a U/V subcarrier, period 8 px at HD, 4 px in SD, that crawls up a line per field |
| K6 Cross-C | fine luma detail leaking into chroma, on every pixel |
| S7 Lens | Sphere / Tunnel |
| S8 Screen | Flat (cells on the screen grid) / Lensed (cells warped with the lens) |
| S9 Ink | Mono (black ink) / Colour (ink takes the picture's colour) |
| S10 Cross-Y | Dots (before the screen: rattles the dots) / Print (after: flickers the print) |
| S11 Fringe | cross-colour V flips on alternate rows (rainbow comb) |
| P12 Squeeze | horizontal scale, 50% (2× stretch) .. 200% (two picture widths across), glided. Was labelled Zoom: same behaviour, the old name read backwards |

## v0.2.0 changes (from the v0.1.0 review)

Bugs (the first three were the snowglobe v0.1 lens bugs, also present here):
- **Wrong mirror and ghost copies on squeezed rows.** The source address
  was 12 bits and the DDA 21 bits. At HD default settings 492 of the lens's
  rows (every row more than 47% of the radius from the centre) read wrong
  addresses: the right-hand mirror jumped 257 px where the address wrapped
  at 2048, and the pole rows wrapped further (up to 1,242 wrong pixels a row)
  and showed copies of the picture where black belongs. Checked with a model
  of the v0.1 arithmetic. The DDA is
  now 24 bits and the address saturates outside ±4096 px.
- **Squeeze flipped pole rows to 16× magnify.** The row step was a 12-bit
  signed value; above 8× it wrapped negative and was clamped to the minimum,
  so with P12 above about half the pole rows jumped from squeezed to hugely
  magnified. The step is now 13 bits and clamps at 16×.
- **Pole rows snapped to 8× magnify at Bulge above ~92% (and below ~6%).**
  The lens term `256 + fk/128` was narrowed to 12 bits and folded. It is
  now computed at 14 bits before the clamp.
- **K3 Cell opened at 4 px instead of its 8 px default.** At program load
  the firmware writes a labelled knob's label INDEX (1), which the zone
  decode read as the bottom zone. The index is now honoured until the knob
  first moves (presets carry real positions and decode normally).
- **The halftone never reached solid black or clean white.** Every black
  area (letterbox bars, the lens's black sides) showed a lattice of single
  white pixels at the cell centres; full white still had ink specks at the
  cell corners; Square and Line at 4 px were mid-grey at full white. The
  luma is now scaled so that black (≤ 72) inks the whole cell and white
  (about 895 and up) clears it, for every cell size and shape.
- **Output levels.** Ink was drawn at Y = 0 (below black, and a reserved
  timing code) and the Print crosstalk drove paper to 1023. Ink is now 64,
  paper 768, luma clamped to 64..775 and chroma to 64..960.
- **K6 Cross-C and S11 Fringe were dead in Mono,** and never touched the
  paper in Colour (the leak went into the picture chroma, which only ink
  pixels show). The cross-colour now lands on every output pixel.
- **The right edge used the next line's cell row.** The row counter
  advances at the end of the line while the last ~8–11 pixels are still in
  the pipeline, so the right edge's dots were offset a line (and its Fringe
  sign flipped). The pixel path now latches its row at the line start.
- **Mirror seams.** The mirror reflected the source's black blanking
  columns, which the screen printed as solid ink stripes in squeezed rows.
  It now reflects about a 12-px inset (black beyond the mirrored copy).
- **The top line showed the previous frame's bottom line.** It is black now.
- **Lens knobs were read live.** Bulge, Radius and Zoom fed the per-frame
  1/r divider and the per-line tests directly, so pot noise could shimmer the
  lens edge or mis-index the table on a row. They are latched per frame and
  glided (ease 1/8 a frame). Cell and Dot are deadbanded (±12) so a knob
  parked at a step edge can't flicker the screen. The switches and K5/K6 are
  latched per frame.
- **The dot crawl didn't crawl.** Cross-luma was ±(U−512) alternating per
  pixel column: static 1-px stripes, blind to the V axis. It now rides a
  4-phase U/V subcarrier (U, V, −U, −V; 2 px a quarter at HD, 1 px in SD)
  that turns 90° a line and a field, the NTSC frame-line relationship, so
  the pattern crawls up one line per field and every hue leaks.

Hardware contracts:
- **HD HDMI bar guard:** [1 2 1] on Y/U/V just before the blanking gate,
  and white kept at or under 775 by drawing paper at 768 (rather than
  scaling). A halftone is the bar trigger profile: sharp near-full-swing
  edges, moving with the picture. Visible side effects: dot edges are about
  half a pixel softer and the paper is ~12% dimmer than v0.1's 876.
- **U/V parity swap on replayed rows** (drape v1.3). Every row shows the
  previous line, so rows whose hsync→avid parity differs from their capture
  row's would show swapped hue. This only matters on analog sources whose
  parity wanders.
- `no_rw_check` on all 6 line-buffer RAMs (the bank being written is never
  shown). Standalone yosys: `emulate_transparency` 3 → 0.
- Latency 20 clocks in all modes (was 17); blanking-gated.

Presets: the four v0.1 presets are unchanged (Radius values give nearly the
same radius at HD under the new mapping). All are hand-written; recapture
them with capture_preset.sh once the look is settled.

## v0.2.2 (2026-10-17): resolution scaling

The halftone and the lens now run in virtual 1080-line units, stepped from
the video timing ID (`registers_in(8)`): x 1.0 / 1.5 / 2.0 units a pixel
(1080 / 720p / SD), y 1080 / active lines a field line (interlace included).
- **Halftone cells** were fixed 4–32 px and field lines: 1.5× too coarse at
  720p, and in SD ~4× too coarse and 2:1 tall. The flat screen now takes its
  cell-local x from a virtual x counter, the Lensed screen scales the source
  column to virtual units (×1 / ×1.5 / ×2 on its low 7 bits), and the rows
  come from the virtual y. A cell is the same fraction of the screen height
  in every mode and square on a 4:3 SD screen. (SD 480i has only 243 lines
  a field, so an 8-unit cell is under 2 lines tall: small cells alias there.)
- **Lens**: centre, radius and the 16-line minimum radius are virtual too
  (the band was already a fraction of the field height; the minimum now
  scales instead of being 4× bigger in SD).
- Unchanged at pixel / line scale: the dot-crawl carrier (8 px HD, 4 px SD,
  720p counts as HD), the Fringe comb, the 12-px mirror inset.
- 1080p is bit-identical (0 differing pixels against v0.2.1). Tier 2's "SD
  interlace" item is covered by this.

## Next tiers (offered 2026-10-13, not done)

Tier 2 (look, Ron's call):
- **S10 as "Composite: Before / After" for both crosstalks.** After = the
  cross-colour comes from the printed dots' own edges, so the whole screen
  gets a rainbow moiré, even on mono sources: a composite camera filming a
  printed page.
- **Real cross-colour rainbows:** demodulate the luma high-pass with the
  same subcarrier and low-pass it, so fine detail swirls through hues
  instead of the current green/magenta edge tint.
- **Colour ink with body:** ink keeps part of the picture's own luma (dark
  to mid coloured dots) instead of chroma over black.
- **Round dots both ways (Euclidean dot):** black circles in the highlights
  that grow to a checkerboard at 50%, white holes in the shadows. Today's
  highlight ink is concave four-point stars between the white discs.
- **Screen angle:** a 45° screen (the classic print angle), or per-channel
  angles in Colour mode for a CMYK rosette.
- **SD interlace:** cells and Fringe on frame lines, so SD dots are round
  rather than 2:1 tall.

Tier 3 (bigger, ask first):
- **P12 Screen:** continuous cell size, 2..128 px exponential, with the
  cells on a DDA phase so any size works. The climax is giant pop-art dots.
  Squeeze would move to K3.
- **Round globe:** fit the lens to a circle (black outside the disc) rather
  than a full-width band of rows.
- Optimization if room is ever needed: time-share the per-line sequencer's
  four LUT multipliers through one shift-add unit (~400 LUT).
- Presets captured from hardware; v1.0.0 sign-off; shelf card; commit.

## Build

`./build_combo.sh lenscreen` → `out/rev_b/ron/catalogue-combo/lenscreen.vmprog`.

Standalone yosys (program_top only): v0.1.0 2,578 LUT; v0.2.0 3,126 LUT,
18/32 EBR. The growth is the glided per-frame controls, the 24-bit DDA, the
clamp stage and the [1 2 1] smoothing.

v0.2.0 (2026-10-13, router2 default): all 6 configs pass on seed 1. HD
Analog 81.3, HD HDMI 77.5, HD Dual 78.8 MHz; SD Analog 79.3, SD HDMI 81.3,
SD Dual 81.2 MHz. About 5,600 LC (73%), 18/32 EBR. The first build had HD
HDMI at 74.6 MHz, with a 70–76 MHz spread over four seeds. The critical
path was the per-line `cx × srow` product (11×13, one clock; STA times the
hblank sequencer at the pixel clock). Splitting it and `ady × inv_r` into
hi/lo halves over two sequencer states moved HD HDMI to 77.5–82.7 MHz on
all six seeds and saved 165 LUT. The next path is the 11×6 half of
`ady × inv_r` at 77.5 MHz.

v0.2.2 (2026-10-17, router2 default, built under heavy machine load): all 6
configs pass. HD Analog 81.5 (seed 1), HD HDMI 85.2 (seed 2; seed 1
missed), HD Dual 78.5 (seed 1); SD Analog 83.8, SD HDMI 73.4, SD Dual
79.5 MHz (seed 1, Fmin 27). About 5,650 LC (74%), 18/32 EBR. All six .bin
files unpack.

HW status: v0.2.0 built, not yet flashed.
