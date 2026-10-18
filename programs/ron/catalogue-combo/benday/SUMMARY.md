# Benday — summary

Catalogue combo #31 (seed 20260917): halftone screen + datamosh bands.
v0.1.0 (2026-09-11) was flashed and liked ("benday is good"), with no later
hardware notes. v0.2.0 (2026-10-12) is the finalizing pass, built from a
review of v0.1.0. It is **not flashed yet**.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Cell | halftone cell 4 / 8 / 16 / 32 px (labelled; default 16) |
| K2 Dot Gain | dot weight; at 100% black prints solid and white prints clean paper |
| K3 Screen | Dot / Line / Diamond / Square (labelled) |
| K4 Height | band height 8 / 16 / 32 / 64 lines (labelled) |
| K5 Shift | sideways tear of a moshed band: ±1020 virtual units (1020 px at 1080, 680 at 720p, 510 on SD) |
| K6 Rate | re-roll rate, from every ~2 s to every frame |
| S7 Ink | Dark: black dots on white paper. Light: white dots on black (both positive) |
| S8 Colour | Mono / the bright side (paper, or the light ink) takes the cell colour at 2× saturation |
| S9 Mosh | Picture: bands tear the picture. Screen: bands shift the screen phase and print the next screen pattern |
| S10 Band FX | Colour: bands carry 32-px held, U/V-swapped, 2× cell colour. Negative: bands print negative |
| S11 Band Fill | Halftone / Flat cells (macroblocks) in the bands |
| P12 Density | moshed fraction of bands: 1/16 at 0, every band at 100% (default 30%) |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Colour was broken on hardware.** The running box sum subtracted N
  blanking samples that it never added. Hardware blanking is Y 64 / C 512, so
  every cell's luma read 64 low and its chroma wrapped through code 0
  (saturated green). This hit S8 Cell and S11 Flat. The 480×270 sim shows it
  too (its stimulus also blanks at 64/512), but v0.1 was never simmed in
  colour. The ring is now written with zeros outside avid. A cell never reads
  a window that includes the zeros, because it reads at its last pixel.
- Cell averages were misregistered. The box window for a cell was centred
  N/2−2 px left of the cell, so the picture sat up to 14 px right of its
  dots. Each address now holds the mean of the N pixels ending there, and a
  cell reads its last pixel (an OR, not an add), so the window is exactly
  the cell.
- The first line of every cell row showed the row above's luma (a 1-line
  seam). Capture is now the last line of each cell row, plus line 0 for the
  top row.
- S8 Colour put the colour on the dark background in Light mode (Y 64 plus
  chroma = mud). Colour now always goes on the bright side.
- S7 Light was a negative print. It is now a positive light-ink print: white
  dots grow with brightness.
- Dot-gain tone: black left a white speck at each cell corner, and white
  left a dot at each cell centre at the default Ink. Luma is now taken above
  black and scaled ×1.125, so 100% prints black solid and white clean.
- P12 at 255/256 never moshed every band. 100% now moshes all of them.

Controls (bold, always-on):
- S10 Chroma did nothing in Mono (the default), because it held and swapped
  neutral chroma. Crush was nearly invisible on halftone bands (64→128).
  S10 is now Colour / Negative, and both read strongly in every mode.
- S9 Screen was subtle (a phase shift only). Its bands now also print the
  next screen pattern.
- P12 at 0 disabled six controls (K4–K6, S9–S11). It now has a floor of 1/16
  of the bands, and the default is 30%.
- K1, K3 and K4 have value labels. Before, they were 4-step knobs shown as
  linear ranges. The labelled-default index guard is on all three.
- S10/S11 were both named "Bands" and are now Band FX / Band Fill. K2 is
  now "Dot Gain", so it no longer shares the name "Ink" with S7.
- Categories are Print + Glitch. "Dither" and "Retro" are not valid
  categories.

Hardware contracts:
- HD HDMI bar fix (damier v0.4.4) before the gate: [1 2 1] on Y/U/V, and luma
  scaled to white ~777. Paper is drawn at 940 before scaling (was 880).
- Rows are replayed from a stored line, so U/V are swapped on rows whose
  hsync→avid parity differs from their capture row's (drape v1.3).
- Neutral outside avid before the smoothing, so its edge taps read black.
- Latency 12 → 15, the same in all modes.

Optimizations:
- `no_rw_check` on all 7 RAMs. The ring reads N−1 back, and a line-buffer
  bank is only read while the other bank is written.
- The dot-gain multiply (8×9, built from LUTs) was the HD Analog critical
  path at 76.0 MHz. The gain is now split hi/lo across two stages:
  81.8 MHz.
- The off-line test for the torn read moved one stage earlier, against a
  per-line [lo, hi) range. Removed dead code: s_swap_row, the unused raw
  input registers, the +half adder.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback:
- Wrap torn bands around the line instead of revealing black (sync-tear
  look). This needs the tear kept under one line width; per-line precompute
  of x+off±W, then one select.
- A 64 px cell for HD (a 5th Cell label). It is cheap: the ring is already
  64 deep. The 16×16 ROM gives 4-px stair-steps on the dot edges, though.
- Continuous Cell size: counters + reciprocal instead of power-of-2 masks.
  It makes Cell a smooth zoom, but it is a real rework of the read and
  screen addressing.
- Analytic screen shapes (dot = r², diamond = |dx|+|dy|, ...): smooth dot
  edges at any cell size, and they free the 2-EBR ROM.

Tier 3, optional:
- A true N×N cell average (read-modify-write accumulate down the cell row)
  to stop vertical aliasing and shimmer at big cells.
- A 45° screen angle (the classic newspaper ruling), or CMY separations at
  different angles.
- ~~SD interlace: cells are field-line based, so they are 2× tall on 480i/576i.~~ Fixed in v0.2.2.
- Rate at 100% re-rolls every frame (a 60 Hz strobe); cap it at every 2nd
  frame if it reads as flicker.
- Capture defaults and presets from hardware (current presets are
  hand-written for v0.2.0 semantics).

## v0.2.2 (2026-10-17): resolution scaling

At 720p the cells, bands and tear were 1.5× too big relative to the screen
(720p took the HD tear by width bit 10), and on 480i the cells were 2× too
wide and 4.4× too tall (field-line rows). Everything generated now runs in
virtual 1080-line units, with steps from the timing ID (registers_in(8)):

- x: a half-unit accumulator (steps 2 / 3 / 4 = 1 / 1.5 / 2 units a pixel);
  only its low 6 bits are needed, because cells and the 32-unit colour blocks
  are powers of 2. The cell's last real pixel = torn x + floor(half units
  left / x step); at 1080 that is exactly the old `x or mask`. The box window
  is N px at 1080, N/2 px at SD (the whole cell) and N/2 px at 720p (3/4 of
  the 2N/3-px cell).
- y: band hash from the true virtual line (Q.8 per field line); cell rows
  from a second accumulator whose SD step is snapped to 4.0 (2.0 progressive),
  so every cell row is a whole number of lines. The exact 4.44 put 1.8 lines
  in an 8-unit cell and the rows beat into horizontal streaks. SD cells are
  therefore ~11% taller than exact.
- Pixels and lines sample their left/top edge, as 1080 always did. With
  centre samples a 2-line cell read screen rows 4 and 12, both inside the
  dot, so mid-tones printed as solid vertical stripes.
- On 480i/576i the 4-px cell is floored to 8 units (2 field lines). A 4-unit
  cell is under one line, every line read the same screen row, and the
  halftone vanished.
- Tear: ±1020 virtual units (K5 × 171/256 at 720p; SD unchanged at ±510 px).

1080p is bit-identical (diff1080 = 0 at the scaling-test settings).

Build (`./build_combo.sh benday`, router2):

| Config | Fmax | Seed |
|---|---|---|
| HD Analog | 91.4 MHz | 2 (seed 1 stalled router2 under machine load) |
| SD Analog | 80.4 MHz | 1 |
| HD HDMI | 90.5 MHz | 1 |
| SD HDMI | 91.3 MHz | 1 |
| HD Dual | 95.4 MHz | 1 |
| SD Dual | 85.4 MHz | 1 |

3,659–3,679 LC (48%, +~260), 22/32 EBR. iceunpack OK on all six.

## Build

`./build_combo.sh benday` → `out/rev_b/ron/catalogue-combo/benday.vmprog`.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 81.8, HD HDMI 91.0, HD Dual 85.6 MHz; SD Analog 89.3, SD HDMI 91.4,
SD Dual 92.3 MHz. 3,402–3,421 LC (44%), 22/32 EBR. Standalone yosys:
`emulate_transparency` count is 0.

v0.1.0 (2026-09-11): HD 75.8 / 84.4 / 83.0 MHz (analog / HDMI / dual seed 3),
3,082 LC, 22 EBR.

## v0.1.0 backup

Git 3fa438d (sources unchanged from 2026-09-11).
