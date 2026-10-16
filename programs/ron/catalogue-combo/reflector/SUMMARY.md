# Reflector — summary

Catalogue combo #8 (seed 20260912): mirror / kaleidoscope + bloom. v0.1.0
(the "v3 sine-ROM sweep" build, 2026-09-11) was flashed and liked
("reflector is good"). v0.2.0 (2026-10-12) is the first finalizing pass,
from a review of v0.1.0. It has not been flashed yet.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Axis | first mirror position, 0–100% of the width. The mirrors keep bouncing, so below 50% the panes multiply (to 16) |
| K2 Folds | None / One / Two / Three: pane width = Axis, /2, /4 |
| K3 Slide | the source scrolls under the mirrors, bipolar; squared speed curve with a centre detent (to 15 px a frame) |
| K4 Glow | bloom gain |
| K5 Threshold | luma above which the blur blooms, black to ~white (64 + 0.8125·K5) |
| K6 Radius | blur width 7–255 px (two cascaded boxes of n = 4–128, exponential; SD caps at n = 64) |
| S7 Glow | glow fetched mirrored with the picture / unmirrored (the original picture's highlights) |
| S8 Panes | mirrored panes shown as negative |
| S9 Tint | warm glow |
| S10 Seams | soft light lines along the mirror seams and the slide's wrap; lit even at Glow 0 |
| S11 Focus | base picture replaced by its blur |
| P12 Sweep | the axis swings on a sine around K1; depth (to ±half the width) and speed (22 s to 4 s a cycle) grow together |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- Axis below 50% blacked out everything right of 2·Axis: the fold mapped
  those columns to negative source addresses. Sweep flashed a black bar at
  the right edge on every swing left of centre. The mirrors are now a
  two-mirror kaleidoscope: a per-pixel up/down counter bounces the source
  column between 0 and the pane width, so every column is filled.
- Odd axis widths with Folds Two/Three put a 1-px black line on the first
  seam (`2·(a>>1) − a = −1`). It flickered under Sweep. The counter has no
  such case.
- Folds came up as None at power-on: the firmware loads a labelled default
  as its label index (1), and `k(9:8)` read that as zone 0. Small values are
  now read as an index until the knob first shows a real position.
- The glow trailed its highlights to the right by 1 / 5 / 13 px (Radius
  min / mid / max): the box blur lags its write address. The glow now has its
  own write port at its centre column (newest sample − n − 7, halation's
  engine). On mirrored panes it had been offset the other way.
- C_BLUR_CENTRE shifted the whole picture 16 px right. It wrapped the source's
  last 16 columns onto the left edge, and with Straight glow those columns had
  no glow. Removed.
- The pipeline was off by one stage against its own comments. Video lagged
  avid by a clock (first column garbage, last column lost). Pane parity was
  one pixel early, which left a wrong-polarity column at each negative-pane
  boundary, and the seam flag sat two pixels off. Re-timed from scratch:
  every signal is named for its stage, latency 14.
- Floor seams showed the source's first columns. On analog captures those
  are black, so there was a ~24 px black stripe down the centre at Folds Two.
  Panes now read the source inset by C_EDGE = 12 columns.

Hardware contracts:
- Luma clamp 1023 is now a highlight knee: 600 + (y − 600)/2, capped at 784,
  so white is ~770. Together with a [1 2 1]/4 smoothing on Y/U/V before the
  gate, that is the HD HDMI bar conditioning. Negative panes and Sweep make
  sharp, near-full-swing edges that move. The knee keeps mid-tones unchanged
  where damier's linear scale dimmed everything.
- no_rw_check on all nine RAMs (8 line buffers + the blur ring).
- One latency (14) in all modes; blanking-gated before and after the
  smoothing.

Controls:
- P12 Sweep was speed only, with a fixed ±127 px depth (6.6% of the HD width)
  and an 8.6 s cycle at 100%. Below ~30% it was invisible. Depth now reaches
  ±half the width and the speed rises with it. At 100% around Axis 50%, the
  panes breathe from 1 to 16 every ~4 s.
- Radius had three steps (two knob quarters were identical) and a 33 px
  maximum, small at HD. It is now continuous 7–255 px via halation's ring
  blur, padded with the line's own inset edge pixels, so the glow reaches
  the borders.
- Threshold was dead above ~92% (beyond white) and lifted the blacks below
  6%. It now spans black to ~white.
- Seams were flat 64 px slabs of +2·gain (dead at Glow 0). They are now a soft
  squared ridge, ±32 px (HD) / ±16 (SD), narrowed for panes under 64 px, with a
  floor of 64 + gain so they show at Glow 0. A seam counts only if it is a real
  mirror inside the screen (no half-glow at the borders). The slide's wrap
  glows too.
- Slide moved at least 1 px a frame (60 px/s). It now uses a squared curve in
  1/16 px steps, with a ±16-code centre detent and the same 15 px/frame top.
- Negative panes inverted about 1023 (black became super-white 959). They now
  invert about black/white (1004 − y) and about 512 for chroma.
- Per-frame width is the widest line of the previous field, so a short analog
  line can't jolt the axis.

Presets: Cathedral Radius 900 → 800 (the new map is much wider at the top);
Drifting Glass Slide 640 → 784 (keeps ~v0.1's 4 px/frame under the squared
curve).

## Build

Build with `./build_combo.sh reflector` (router2 default).

v0.2.0 (2026-10-12): all 6 configs pass on seed 1. HD Analog / HD HDMI / HD Dual
81.8 / 87.7 / 85.1 MHz; SD 91.6 / 82.3 / 84.9. ~5,600 LC (73%), 24/32 EBR.
Standalone synth: 0 `emulate_transparency` (no_rw_check effective).
(v0.1.0: 4,331 LC, 23 EBR, HD 77–80 MHz with HD Dual on seed 2.)

## Open (for Ron)

- Hardware check of everything above, especially: the Tint hue (it should read
  warm; U/V convention), dimmer highlights from the knee, and HD HDMI with
  Negative panes + Sweep.
- Not done, needs a call: P12 as a hand-played Axis (with K1 as the
  auto-sweep); dropping Folds None (it gates Axis, Sweep, S8 and most of
  S10); a fixed-mirror "content flows through" Slide; Screen blend.
- Presets: capture defaults + more looks from hardware.
