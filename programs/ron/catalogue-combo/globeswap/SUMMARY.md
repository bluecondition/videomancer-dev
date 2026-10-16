# Globeswap: summary

Catalogue combo #134 (seed 20260927): a sphere / tunnel row lens + Y/U/V
read from split columns and channel swaps + a Sabattier solar fold. Ron
flashed v0.1.0 in September 2026 and said "globeswap is good" (EXPLORE_LIST,
2026-09-13). v0.2.0 (2026-10-13) is the first finalizing batch. It comes
from a code review of v0.1.0. The v0.1.0 sources and vmprog are in
`.globeswap_work/v010/` (and at git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Lens | lens strength, bipolar (tunnel .. flat .. sphere), glided. About 75% wraps the picture exactly onto the disc |
| K2 Size | lens radius, 3–146% of the half-height (was 16–783 lines), glided. HD is within 1% of v0.1 |
| K3 Swap | None / U<>V / Y>U / U>Y / Y<>V / Rotate / -Y / -YUV, labelled, with hysteresis at the zone edges. Default U<>V (unchanged) |
| K4 Offset | per-channel column offset, ±256 px at HD (±128 in SD), glided |
| K5 Solar | **direction reversed**: 0% = off, 100% = the most solarized (threshold 960 → 386). The default (64%) is v0.1's threshold 600 |
| K6 Fold | fold gain 0.125× .. 7.5×, log, over the whole knob (v0.1 was dead in its bottom 27% and top 25%) |
| S7 Fringe | Even / Rim (split grows toward the lens rim) |
| S8 Swap | All / Solar (swap only inside the solar region) |
| S9 Split | Chroma (U and V pulled apart) / Luma (luma against both chroma) |
| S10 Lens | Row (whole width) / Disc (lens inside the disc, dry picture outside) |
| S11 Fold | Luma / Colour (the solar region also inverts its chroma) |
| P12 Globe | grows the lens (toward sphere) and its radius, spreads the channels, drives the fold, now up to 16× (v0.1 stopped at 8×, and P12 did nothing to the fold once K6 was above 75%). Glided |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Squeezed rows wrapped.** The lens DDA was 21 bits and the source column
  12 bits. At HD, any row squeezed more than ~1.13× ran past column 2047 and
  wrapped. At the defaults that is every lens row more than ~45% of the
  radius from the centre; at high P12 it is most of the globe. The right side of such rows showed the right-edge colour
  and then a band of the LEFT-edge colour (a seam on one side only). At
  strong settings the polar rows showed ragged squeezed copies of the picture
  at a different phase on each row. The DDA is now 24 bits and the column
  saturates, so every row fills with its own edge colour on both sides, as
  documented. **This is a visible change.** If the copies at the poles were
  part of what you liked, Tier 2 item 1 brings them back as a deliberate,
  seamless wrap.
- **K6 Fold was dead in half its travel.** The gain ROM was flat at 0.125×
  for the bottom 17 of 64 steps and at 8× for the top 16. The log ramp now
  spans the whole knob (1× at 50%, unchanged). P12 adds up to 15 steps on
  top, to 16×, so the slider drives the fold at every K6 position.
- **K5 Solar ran backwards.** 100% was threshold 1023 (no solarization at
  all), and the top ~8% and bottom ~37% did nothing useful (above white
  nothing folds; below ~380 the folded region clips to black). Solar is now
  0% = off, 100% = everything above mid-grey reflected (whites to black),
  over thresholds 960 → 386.
- **K3 Swap modes flickered at zone edges.** Eight modes on raw top bits, no
  hysteresis, so pot noise at a boundary flipped hues (e.g. None ↔ U<>V)
  frame to frame. Now 8-count hysteresis bands, labelled modes, and the
  labelled-default index trap handled (a value below 8 right after load is
  the label index).
- **Pot noise shimmered the polar rows.** Lens strength and P12 were read
  live; one ADC count moves a polar row's edge by ~30 px. Lens, Size, Offset
  and P12 are now latched per frame and glided (ease 1/8 a frame).
- **Super-black luma.** The solar fold and the U>Y / -Y modes could drive
  luma below 64 (down to 0). Luma is now clamped to 64..940.
- **SD proportions.** Size was in HD lines (most of the knob was bigger than
  an SD field) and Offset/P12 in HD pixels. Size is now a fraction of the
  measured half-height and the split halves on SD-width lines. HD unchanged.

Hardware contracts:
- **HD HDMI bar guard**: [1 2 1] on Y/U/V plus luma scaled to white ~775,
  before the gate. The 8–16× fold draws sharp full-swing luma bands that
  move with the picture, the bar-trigger profile. Visible side effects:
  whites are ~19% dimmer and the picture very slightly softer. Latency
  18 → 20 clocks, all modes.
- **U/V parity swap on replayed rows** (drape v1.3). Each output row is the
  stored previous line. Where the two lines' hsync→avid parity differs
  (analog sources), U and V are swapped. The read addresses are swapped too,
  so the chroma split keeps its direction on those rows.
- `no_rw_check` on the six line-buffer RAMs and the dry ring.

Cleanups: the tunnel table and the circle table were identical (one ROM read
now). The control decode moved off the per-clock path into the vblank
sequencer. The per-line multiplies are split hi/lo (see Build).

Presets: the three v0.1 looks are kept, remapped. Solar values are mirrored
to the new direction (same thresholds). Fold values are moved to the same
gains on the new ROM. They are still hand-written: recapture with
capture_preset.sh once the look is settled.

## HW checklist (v0.2.0)

1. Defaults: same look as v0.1 except the top/bottom of the lens. Squeezed
   rows should fill with edge colour on both sides, with no left-edge-colour
   band on the right.
2. K6 Fold: something changes across the whole knob; 50% = v0.1's default.
3. K5 Solar: 0% = no solarization, rising to whites-go-black at 100%.
4. K3 Swap: the display shows mode names; parking the knob near a zone edge
   should not flicker.
5. P12 sweep: smooth (glided), no shimmer on the polar rows. At the top the
   fold goes denser than v0.1 could (16×).
6. HD HDMI with motion and Fold high: no thick bright bars.
7. On an analog source: no thin hue-flipped lines over coloured areas.

## Next tiers (offered 2026-10-13, not done)

Tier 2 (look, your call):
1. **Spinning globe.** Wrap the picture around the globe seamlessly (the
   polar copies come back, deliberately and without seams), and let it turn:
   the picture scrolls round the sphere like a rotating globe. This is a
   modulo-width DDA. It could be P12's climax ("the globe starts to spin")
   or take S7.
2. **Shaded sphere in Disc mode.** Limb darkening toward the disc rim and a
   specular highlight, so the disc reads as a lit 3-D ball, not a flat
   cut-out. Needs a per-pixel dx² + dy² (one 11×11 multiply).
3. **Full-range solar bands + Mackie line.** The fold output spans
   [thr − 512, thr], so deep Solar settings go dark. Spanning black → white
   instead, with a thin bright line at the threshold edge (the real
   Sabattier "Mackie line"), would keep 100% bright and banded.
4. **Rim fringe from geometry.** S7 Rim grows the split with the lens
   deviation, so it does nothing on a flat lens and little in mid-rows.
   Growing it with the row's distance from the centre (and 2× bolder) would
   act at every K1.
5. **P12 follows K1's direction.** With K1 on tunnel, P12 flattens the lens
   through zero before turning it into a sphere. It could deepen the tunnel
   instead (with a deadband at K1's centre so noise can't flip it).
6. **Round disc in SD.** Interlaced SD pixels are ~2.25× wider per field
   line, so Disc mode draws a tall ellipse in SD.

Tier 3 (bigger, ask first):
- Full-resolution 10-bit chroma in the line buffer (now 8-bit, half
  horizontal resolution). It costs ~12 more EBR, which do not fit next to
  the ring. Only worth it if chroma banding shows in the split.
- Presets captured from hardware; v1.0.0 sign-off; shelf card; commit.

## Build

`./build_combo.sh globeswap` → `out/rev_b/ron/catalogue-combo/globeswap.vmprog`.

v0.1.0 (2026-09-13): passed all 6 configs (no logs kept).

v0.2.0 (2026-10-13, router2 default): all 6 configs pass on seed 1. HD
Analog 89.8 / HD HDMI 85.5 / HD Dual 87.4 MHz; SD 85.3 / 79.1 / 83.4 MHz.
~5,590 LC (73%), 21/32 EBR (18 line buffer, 2 dry ring, 1 fold-gain ROM).
Standalone yosys: `emulate_transparency` = 0. All six .bin files unpack.

Timing notes. The first v0.2.0 build sat at 73.9 MHz on HD Analog, because
the new fold-gain ROM went into an EBR that absorbed its output register.
A second fabric register fixed that, which left 74.25 MHz exactly (zero
margin). The path was then `s_kabs → s_fk`, v0.1's single-step 12×11
lens-strength multiply in the per-line sequencer. All five per-line
multiplies are now split hi/lo over two steps (14 steps, well inside any
hblank). That gave +15 MHz and −180 LC.
