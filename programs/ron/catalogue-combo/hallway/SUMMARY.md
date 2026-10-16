# Hallway — summary

Catalogue combo #84 (seed 20260923): split-screen / quad + lens distortion
(barrel, chromatic aberration) + mirror / kaleidoscope. v0.1.1 (2026-09-12)
was flashed and liked ("hallways is good", split lens per channel), with no
later hardware notes. v0.2.0 (2026-10-13) is the first finalizing pass, built
from a review of v0.1.1. It has **not been flashed yet**. The v0.1.1 sources
and vmprog are in `.hallway_work/v011/` (and git 3fa438d).

## What the lens actually is

The "barrel" is a per-ROW horizontal scale: each row is magnified or
minified about the lens centre by a factor that grows towards the top and
bottom (profile 256/√(1 − t²), t = distance from the middle row). Vertical
lines bow; seams and pane boundaries stay straight. The fringe gives U and V
their own scale, so colour separates more the further it is from the centre.
v0.2 keeps all of this.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Panes | 1 / 2 / 3 / 4 panes (the knob now changes exactly where the display does) |
| K2 Wipe | pane boundaries slide across the screen, wrapping; glided |
| K3 Barrel | lens strength, bipolar: pincushion (top/bottom rows magnified to 8x) .. barrel (minified to 1/8); glided |
| K4 Fringe | chromatic spread, 3 .. 50% of the lens scale (U wider, V narrower) |
| K5 Segment | mirror segment width, 64 .. 1024 px at HD, continuous and exponential (knob up = wider); a fraction of the width in every mode |
| K6 Slide | strip slide speed, bipolar, squared, ±16-code stop at the centre, up to ~7 px a field |
| S7 Mirror | odd panes mirrored |
| S8 Lens | Screen: every pane looks at the screen centre (copies) / Pane: each pane looks at its own centre |
| S9 Fold | Mirror (triangle) / Repeat (sawtooth), same segment width |
| S10 Edges | sources off the line: black / mirrored back in |
| S11 Fringe | U wide / V wide |
| P12 Depth | segments narrow continuously, up to 16x (8 px floor); glided |

## v0.2.0 changes (from the v0.1.1 review)

Bugs:
- **K1 Panes did nothing at the default switches.** With S8 on Screen, the
  fold ran on the whole line about the screen centre, so pane boundaries only
  mattered when S7 Mirror was on. The default is S7 Off / S8 Screen, so the
  default Panes knob was dead. Each pane now folds its own coordinate. Screen
  mode shows the centre strip in every pane (split-screen copies); Pane mode
  is unchanged (each pane shows the strip around its own centre).
- **P12 Depth was a copy of K5 Segment.** Four nested power-of-two folds are
  exactly one fold at the finest period (checked numerically: 0 mismatches
  over every setting). P12 was four more halvings of the segment, in steps.
  Now one fold of the pane coordinate scaled by g = 1..2 gives any segment
  width, and the lens scale is divided by g, so each strip still shows its
  own width of picture (v0.1's crop, made continuous). K5 and P12 both move
  smoothly; octave crossings are seamless (≤ 1 px).
- **Each Depth step shifted the picture.** A disabled fold skipped a pipeline
  register: every step moved the image 1 px and left 1–3 px of wrong-centre
  columns at pane seams. Gone with the single fold.
- **K5's display was reversed.** It read 64 px at 0 and 1024 at the top; the
  knob did the opposite. Knob up is now wider, shown as 0–100%.
- **The lens was a staircase.** The profile had 64 steps per half-height
  (8.4 rows each at HD). At the default Barrel, the scale jumped up to
  310/256 between bands near the top and bottom, so verticals there were
  jagged. It is now interpolated per row (largest row-to-row step 53/256, on
  the steep last rows of the profile).
- **Extreme Barrel wrapped.** Above ~85% (and below ~15%) the outermost ~8
  rows' scale overflowed a narrowing resize and flipped to a much weaker
  zoom. Now clamped.
- **Fringe could go negative.** The spread was added to the scale, so strong
  pincushion + high Fringe gave V a negative scale, read as a huge one (V
  went grey). Fringe is now a fraction of the lens scale and always positive.
- **Repeat showed an off-centre strip twice as wide.** v0.1's sawtooth
  showed source from −¼ to +¾ of its period. Repeat now shows the same
  centred strip and segment width as Mirror, so S9 is a clean look-flip.
- **K1's zones didn't match its display.** 17–25% showed "2" but gave 1 pane;
  83–100% showed "3" but gave 4.
- Mirrored edges now reflect about the half pixel at both edges (v0.1
  reflected about pixel 0 on the left only).

Mode scaling:
- **Segment and Slide were fixed pixels.** A 256 px segment is 1/7.5 of an HD
  line but 1/3 of an SD line. Both now follow the measured width (HD is
  unchanged).

Hardware contracts:
- **Per-row U/V parity swap (drape v1.3).** Every row shows the line stored
  one row earlier. On analog sources whose hsync→avid offset wanders, rows
  whose parity differs from the stored row's had Cb/Cr swapped (thin
  coloured lines). The U/V read addresses swap with them, so the fringe stays
  on the right channel.
- **Width latched once a field** (the widest line of the last field). v0.1
  used each line's own width for the wipe wrap and the edges, so analog
  ±1 px line jitter shook them, magnified up to 8x by the lens.
- **HD HDMI bar conditioning, built in:** highlight knee (600 + (y − 600)/2,
  capped at 784, so white is ~770) and [1 2 1] smoothing on Y/U/V before
  the gate. Hallway makes moving, near-full-swing edges: black edges, rows
  minified 8x, and fringe.
- One latency (23) in all modes; neutral outside avid before the smoothing,
  and gated again at the output.

Controls:
- **Slide creep.** Pot noise at the centre gave ~4 px/s of drift. There is
  now a ±16-code stop, a squared curve (fine at slow speeds) and a top of
  ~7 px a field (v0.1: 4). The speed is the same on screen at any segment
  size.
- **Glides with a 4-code deadband** on Wipe, Barrel and Segment/Depth. One
  code of Barrel was a 3.5% scale change in the top rows, and continuous
  Segment/Depth would otherwise shimmer the outer seams.
- **Fringe floor (3%)** so S11 always shows.

Optimizations:
- `no_rw_check` on all 6 line-buffer RAMs (a bank is only read while the
  other is written).
- The 4-fold chain is now one fold. The per-field ×21845 constant multiply,
  the per-field wipe multiply and the two per-line parallel multipliers moved
  to two serial shift-add multipliers (one per field, one per line, ~60
  clocks after hsync). The duplicate avid delay chain is removed.

Presets were re-derived to keep their v0.1 segment widths: Four Lenses
K5 768 (512 px); Deep Prism K5 255 + P12 100% (8 px); Tiled Pinch K5 768
(512 px repeat tiles, now centred). The default is K5 512 + P12 0 (256 px,
as v0.1's default) and Slide 650 (~0.37 px a field, as v0.1's 560).

## Build

Build with `./build_combo.sh hallway` (router2 default).

v0.2.0 (2026-10-13): all 6 configs pass, HD on seed 1: HD Analog / HD HDMI /
HD Dual 78.2 / 83.7 / 81.2 MHz; SD 82.1 (seed 2, seed 1 stalled the router) /
78.6 / 79.2. ~5,600 LC (73%), 18/32 EBR, iceunpack OK on all 6. Standalone
synth: 0 `emulate_transparency` (no_rw_check effective).

(v0.1.1: 5,790 LC (75%), 18/32 EBR; HD 76.4 / 76.4 / 74.21 MHz, HD Dual was a
seed-6 miss.)

## Hardware checks for v0.2.0

1. **HD default:** does it still look like the v0.1 you liked? You should now
   see two panes (the default K1 is 2), each with its own copy of the strip.
   Whites are dimmer (knee) and edges a touch softer ([1 2 1]).
2. K1 Panes sweep with S7 and S8 both off: the panes should split at every
   step (v0.1: nothing changed).
3. P12 Depth sweep: the mirror seams should close in smoothly towards each
   pane centre, with no jumps. K5 Segment sweep: the same, wider as you turn
   up.
4. K3 Barrel at ~70%: the verticals near the top and bottom should curve
   smoothly (v0.1 had a staircase every ~8 rows).
5. K6 Slide at the centre: stopped. Slow speeds near the centre, fast at the
   ends.
6. S9 Repeat: the tiles should be the same width as the mirror segments.
7. Strong pincushion (K3 ~10%) + Fringe 100%: colour should stay on both
   channels (v0.1 greyed V).
8. Analog colour source: any thin coloured lines? HD HDMI: any thick bars?

## Review backlog (not done)

Tier 2, look changes (your call):
- **S7 and S8 do nothing at 1 pane** (the pane is the screen). Options: K1
  as 2–5 panes, or at 1 pane S7 book-matches the screen about its centre.
- **S10 Edges is mostly hidden.** It only shows when the lens pushes the
  source off the line (strong Barrel in the top and bottom rows, or the Pane
  lens with 3–4 panes). Options: reflect forever (a triangle fold of the
  source, so Mirror never goes black), or turn S10 into "Seams": thin lines
  along the mirror seams, visible at any setting.
- **A screen lens that bends the panes.** Apply the barrel to the screen
  coordinate before the panes and folds, so pane and mirror seams bow like a
  real lens (today they stay straight and only the content rescales). One
  more per-pixel multiply.
- **Reeded glass.** A source mode where each mirror strip shows the picture
  under itself instead of the centre strip: the picture stays recognisable,
  fluted.
- **P12 as zoom instead of crop:** the strip's content shrinks with the
  segment (more, smaller copies of the same picture) rather than narrowing
  to a sliver.

Tier 3, bigger:
- **A true receding hallway:** copies that shrink towards the centre, each
  reflection smaller by a ratio set by P12 (the infinity-mirror look the name
  suggests). Needs a per-pixel log2 (leading-one detect + a small table) and
  a multiply; ~600–900 LC. It fits (v0.2 leaves ~1,500 LC).

## Build seeds — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

2026-10-14: **seed 3**, set as `[firmware] seed = 3` in the staged community TOML (the dev TOML is unchanged). Why: seed 1 stalled nextpnr's router1 on SD Standalone. Sweep 2-6: 2-6 all build 8/8. Own HD logic with seed 3 (scratch core without the Sync Out generator): 92.61 / 88.98 / 91.60 MHz (Analog / HDMI / Dual).
