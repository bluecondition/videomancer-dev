# Pressroom — summary

Catalogue combo #40 (seed 20260919): halftone screen keyed into a colour
cell mosaic + head-switch noise band. v0.1.0 (2026-09-11) was flashed and
liked ("pressroom is good", EXPLORE_LIST 2026-09-12), with no later hardware
notes. v0.2.0 (2026-10-12) is the finalizing pass, built from a review of
v0.1.0. It is **not flashed yet**.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Cell | halftone cell 4 / 8 / 16 / 32 px (labelled; default 8) |
| K2 Screen | Dot / Line / Diamond / Square (labelled) |
| K3 Band Size | head-switch band height: 4 lines at 0 .. half the frame at 100% (default ~5%: 56 lines HD, 15 SD) |
| K4 Noise | band noise, tear and break-up; 10% floor so the band never goes quiet |
| K5 Level | key threshold: luma 72..903, or saturation 8..519 |
| K6 Soft | key edge dithered by noise |
| S7 Key | Luma / Sat |
| S8 Halftone | Inside the key / Outside it; the rest is the colour cell mosaic |
| S9 Ink | Dark: black dots on white paper. Light: white dots on black paper (both positive) |
| S10 Band | Bottom / Rolling up the frame (~8.5 s a pass in every mode, wraps round) |
| S11 Band FX | Noise: noisy, torn band. Break: the whole band prints as a broken screen (thresholds scattered by Noise) |
| P12 Ink | how much ink, glided: x0.29 at 0 (a light print, only the shadows dot), x1.17 at 50% (black solid, white clean), x4 at 100% (the ink floods; only the highlights stay open) |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **S10 Rolling showed no band at all.** The band position wrapped below 0
  to line 2046, and the off-screen reset sent it back to 0 every frame. It now
  wraps to the bottom and climbs height/32 sixteenths of a line a frame. A
  band that runs off the bottom continues at the top.
- **The mosaic colours were wrong on hardware.** This is the same box-sum bug
  that Benday and Stencil had. The ring was written during blanking, and its
  N blanking samples were subtracted from every running sum. Hardware blanks
  at Y 64 / C 512, so cell luma read 64 low and cell chroma wrapped through
  code 0, giving acid green. The ring is now zero outside avid. **This changes
  the look Ron liked.** If the acid mosaic is missed, offer it back on
  purpose as a mode, but do not restore the bug.
- Cells were misregistered: each cell's average was centred on its left
  edge, so the picture sat ~N/2 px right of its cells. Address a now holds
  the mean of the N pixels ending at a, and a cell reads its last pixel (an
  OR, not an add).
- The first line of every cell row showed the row above (a 1-line seam).
  Capture is now the last line of each cell row, plus line 0.
- Band noise was not zero-mean. At Noise 0 the band was a flat −128 dark
  stripe, and at 100% it reached only −128..+127. It is now centred, ±248 at
  100%.
- S9 Light was a negative print, with dark areas turning white. It is now a
  positive light-ink print: white dots grow with brightness.
- Dot tone: at unity, black left white specks at the cell corners and white
  left a dot at each cell centre. The ink amount is now taken from black
  (940 − y or y − 64), so 50% prints black solid and white clean.
- Noise source: v0.1's 1-bit LFSR made each 8-bit sample twice the
  previous one, which gave streaky, correlated speckle. It is now xorshift32
  stepped once per clock, with separate bytes for key dither, band noise,
  break-up and tear.

Controls (bold, always-on):
- P12's ends were degenerate. At 0 the halftone printed solid black, and at
  100% everything above mid-grey went to paper. Its direction was also
  backwards: raising "Ink" gave less ink. It is now an exponential ink gain,
  ×0.29 .. ×4, that rises toward more ink and keeps the image at both ends.
  It also behaves the same in Light mode, and a glide hides pot noise.
- At K3 0 there was no band, which killed K4, S10 and S11. The band now has a
  4-line floor and a range scaled to the frame (v0.1 topped out at 63 lines,
  6% of an HD frame).
- At K4 0 the band was invisible. Noise now has a 10% floor. Tear is half
  as wide on SD-width lines.
- S11 Break was visible only where the key happened to overlap the band, and
  it also switched the band's noise off. A Break band now prints the halftone
  across its whole width, with thresholds scattered by K4.
- K5 Level: the bottom 6% and top 8% of the knob sat outside the picture's
  luma range and did nothing. It now spans 72..903.
- K1 and K2 have value labels; before, they were 4-step knobs displayed as
  linear ranges. The labelled-default index guard is on both.
- Three controls were all named "Band". They are now Band Size (K3), Band
  (S10) and Band FX (S11).
- Categories are Print / Mask / Signal. "Dither", "Keyer" and "Retro" are not
  valid categories.

Hardware contracts:
- HD HDMI bar fix (damier v0.4.4) before the gate: [1 2 1] on Y/U/V, and luma
  scaled toward black (white ~777). Halftone dots are full-swing pixel edges,
  and the band moves them every frame. Paper is drawn at 940 before scaling
  (v0.1 used 880 with no scaling).
- Rows are replayed from a stored line, so U/V are swapped on rows whose
  hsync→avid parity differs from their capture row's (drape v1.3). This
  affects the mosaic's chroma.
- Neutral outside avid before the smoothing; final gate after it.
- Latency 12 → 17, the same in all modes.

Optimizations:
- `no_rw_check` on all 7 RAMs. The ring reads N−1 back, and a line-buffer
  bank is only read while the other bank is written. Standalone yosys
  reports `emulate_transparency` 0.
- The dry-pixel "ring" (2 EBR) was a 32-deep RAM read 1 back, i.e. a
  1-clock delay, so it was replaced by FFs (two deep; key only, never output).
  24 → 22 EBR. The key is now pixel-aligned (v0.1 keyed from pixel p−1).
- Per-line tear multiply and per-frame band multiply are each split over
  their own sequencer states; all knob math is pre-registered per frame.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback:
- Swap K5 ↔ P12: make P12 "Press", the key level glided, so the halftone
  floods into the picture as you push the slider (Stencil v0.2.0 made the
  same move). Ink would go to K5. This is a bigger performance gesture than
  dot gain, but it changes the layout Ron liked, so it is his call.
- Show the live photo instead of the cell mosaic outside the key (v0.1's
  header promised "the rest stays photographic"). The dry pixel is already
  in the pipe, and the delay ring could come back to align it. This is a
  look change.
- Colour halftone: paper (or light ink) takes the cell colour at 2× sat, like
  Benday's S8. There's no free switch; it could ride S8 or a K-zone.
- Torn band lines wrap around instead of showing black gaps; a VHS "hook"
  (skew growing toward the band's bottom); chroma flecks in the band.
- 64 px cell for HD (5th Cell label; the ring is already 64 deep).
- The acid mosaic as a deliberate mode, if Ron misses v0.1's wrapped colours.

Tier 3, optional:
- A true N×N cell average (read-modify-write down the cell row) to stop
  vertical aliasing at big cells.
- 45° screen angle / analytic screens (smooth dot edges at any cell size,
  frees the 2-EBR ROM).
- ~~SD interlace: cells are field-line based (2× tall on 480i/576i).~~ Fixed in v0.2.2.
- Capture defaults and presets from hardware. The current presets are
  hand-written for v0.2.0 semantics.

## v0.2.2 (2026-10-17): resolution scaling

At 720p the cells and the band tear were 1.5× too big relative to the screen
(720p took the HD tear by width bit 10), and on 480i the cells were 2× too
wide and 4.4× too tall (field-line rows). The band height already followed
the measured height. Cells and tear now run in virtual 1080-line units, with
steps from the timing ID (registers_in(8)), using the same engine change as
Benday v0.2.2:

- x: a half-unit accumulator (low 6 bits only). The cell's last real pixel
  = torn x + floor(half units left / x step); at 1080 that is exactly the old
  `x or mask`. Box window N px at 1080, N/2 px at SD and 720p.
- y: cell rows from a Q.8 accumulator whose SD step is snapped to 4.0 (2.0
  progressive), so cell rows are whole lines and the dots stay regular. SD
  cells are ~11% taller than exact.
- Pixels and lines sample their left/top edge, as 1080 always did.
- On 480i/576i the 4-px cell (the Front Page preset's) is floored to 8 units
  (2 field lines). At 4 units it is under one line, every line read the
  screen's edge row, and the halftone vanished into flat paper.
- Tear: ±512 virtual units (tear gain × 171/256 at 720p, which is 341 px; SD
  is unchanged at 256 px). The band's 4-line floor is 4 / y step in field
  lines (1080 4, 720p 3, 1080i/480p/576p 2, 480i/576i 1).

1080p is bit-identical (diff1080 = 0 at the scaling-test settings).

Build (`./build_combo.sh pressroom`, router2):

| Config | Fmax | Seed |
|---|---|---|
| HD Analog | 83.9 MHz | 1 |
| SD Analog | 83.5 MHz | 1 |
| HD HDMI | 82.1 MHz | 2 (seed 1 stalled router2 under machine load) |
| SD HDMI | 81.7 MHz | 1 |
| HD Dual | 87.6 MHz | 2 (seed 1 stalled router2 under machine load) |
| SD Dual | 85.3 MHz | 1 |

4,761–4,788 LC (62%, +~300), 22/32 EBR. iceunpack OK on all six.

## Build

`./build_combo.sh pressroom` → `out/rev_b/ron/catalogue-combo/pressroom.vmprog`.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 84.4, HD HDMI 86.3, HD Dual 82.0 MHz; SD Analog 84.6, SD HDMI 89.3,
SD Dual 77.4 MHz. ~4,480 LC (58%), 22/32 EBR; iceunpack OK on all six.
Standalone yosys: `emulate_transparency` count is 0.

v0.1.0 (2026-09-11): HD 80.7 / 80.8 / 79.9 MHz (analog / HDMI / dual seed 2),
3,541 LC, 24 EBR.

## v0.1.0 backup

Git 3fa438d (sources unchanged from 2026-09-11).
