# Parallax — summary

Catalogue combo #38 (seed 20260918): depth-from-luma parallax + colour
channel swap / offset. v0.1.0 was flashed and liked on 2026-09-12 ("paralax
is good"). v0.1.2 (2026-09-13) added the edge-clamp fill, which has not been
seen on hardware. v0.2.0 (2026-10-12) is the finalizing pass, built from a
review of v0.1.2. It is **not flashed yet**.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Depth | parallax gain; floored at 1/8, so Position always moves the picture |
| K2 Sway | the camera sways on its own: 0 = still and centred, then 68 s → 0.27 s per cycle (the speed doubles every 1/8 of the knob; the bottom 1/8 fades the sway in). Default 37.5% = 8.5 s |
| K3 Focus | luma level that does not move, now spanning black (64) … white (959) |
| K4 Swap | Off / U<>V / Y>U / U>Y / Y<>V / Rotate / Rotate Back / Complement (labelled) |
| K5 Offset | both chroma channels read ±128 px away (colour ghost) |
| K6 Split | U and V pulled apart in opposite directions, 0 … 127 px |
| S7 Depth | from luma / from saturation |
| S8 Near | bright is near / dark is near |
| S9 Chroma | moves with the luma / stays put |
| S10 Sway | Glide (sine) / Wiggle (snaps between two viewpoints: a wigglegram) |
| S11 Edges | Fill / Wrap |
| P12 Position | camera position, bipolar (centre = flat when Sway is 0) |

## v0.2.0 changes (from the v0.1.2 review)

Bugs:
- **S10 did nothing.** Both arms of the Sine/Tri branch assigned the same
  triangle, so "Sine" was a triangle too. S10 is now Glide (a true sine,
  parabolic, easing at the ends) / Wiggle (a square wave that holds each
  viewpoint and then snaps to the other, like a wigglegram).
- **The gain wrapped.** (Position + sway) × Depth was resized to 10 bits
  without a clamp. Above ±511 it folded, so with the slider near either end
  and high Depth, the parallax collapsed or reversed once per sway cycle.
  It is now clamped at ±511.
- **K6 Split was a switch.** It only read its top bit (V = −U below 50%,
  V = +U above), and at Offset 0 (the default) it did nothing at all. Now
  U offset = Offset + Split and V offset = Offset − Split. K5 moves both
  chroma channels together and K6 pulls them apart, each on its own.
- **K4 Swap showed 0–7, but 5, 6 and 7 were all Rotate.** It now has 8
  labels, adding Rotate Back (Y←V, U←Y, V←U) and Complement (hue +180°).
  The labelled-default index guard is on.

Controls (bold, always-on):
- K2 Sway was linear: 91 s per cycle at the default, and only 8.6 s at
  the top. At 0 the sway froze at whatever phase it had, so the centre of
  P12 was not flat. It is now exponential (68 s → 0.27 s). The bottom
  eighth fades the amplitude in, so 0 means still and centred.
- K1 Depth at 0 disabled P12, K2, K3, S7 and S8. The depth gain now has a
  1/8 floor.
- K3 Focus spanned codes 0–1023, but luma lives in 64–940, so about 15% of
  the knob was dead. It now maps to 64–959 (saturation mode: 0–511, as
  before).

Hardware contracts:
- HD HDMI bar fix (damier v0.4.4), applied before the gate: [1 2 1] on
  Y/U/V, and luma scaled so white lands at ~777. The depth tears are
  full-swing pixel edges, and the sway moves them. Luma is held in
  64–940 before the scaling (super-black from the source, and U>Y mode,
  could read below 64).
- Rows replay the line above, so U and V are swapped on any row whose
  hsync→avid parity differs from the parity of the row it was captured on
  (drape v1.3).
- Neutral outside avid before the smoothing, so its edge taps read black.
- Latency 12 → 15, the same in all modes.

Optimizations / timing:
- `no_rw_check` on all 6 line-buffer RAMs. Both banks are read every clock,
  but only the bank that is not being written is used.
- The 2-clock dry-pixel delay was a 32×32 RAM ring (2 EBR). It is now 30
  FFs.
- Saturation |u−512| + |v−512| uses bit inversion (no carry chain) and
  moved to stage 2, so stage 3 is a single subtract.
- The depth × gain multiply (12×10 in one stage) was the HD Analog critical
  path. The first v0.2.0 build closed at only 74.6 MHz. The multiply is now
  split on the gain's hi/lo bits across two stages.
- Per-frame sequencer: one small op per state, every multiply split hi/lo
  (15 states, all in vblank). It was sweep-checked at RTL widths for every
  K2 value, every phase and the K1/P12 extremes: no overflow.
- Removed the unused `f_clamp10`.

## Hardware checks for v0.2.0

- S10 Wiggle at mid/high Sway: does it read as a wigglegram (depth from
  the two-view alternation), or as flicker? At the top of K2 it flips about
  7 times a second.
- Sway speed range: is 0.27 s at the top too fast for Glide?
- K6 Split on its own (Offset 0): chromatic fringes on every edge.
- Edge seam at full Depth with S11 = Fill. This is still open from v0.1.2.
- Output brightness: the bar fix lowers white from 940 to ~777. Compare it
  against the dry source and say if it reads too dim.
- HD HDMI with strong sway: any thick bars?
- Thin coloured lines across saturated areas would point at the parity
  swap (see Tier 2).

## Review backlog (not done)

Tier 2, worth doing after hardware feedback:
- **Depth smoothing.** Depth is each pixel's own luma, so in textured areas
  the displacement jumps from pixel to pixel (the jagged "tear" texture).
  A horizontal running box of 8–32 px on the depth (one EBR ring) would
  slide objects as coherent slabs, which reads more like real parallax.
  This changes the look, so ask first.
- **Depth layers (multiplane camera).** Quantize depth into 4–8 planes, so
  the picture splits into flat cut-out layers that slide at different
  rates. It could replace S8 Near, which is weak (see below).
- **3D anaglyph as a K4 Swap label.** Two viewpoints (x + d and x − d):
  red from one, cyan from the other. In YUV this is cheap:
  U ≈ 0.169 (R − L), V ≈ 0.5 (L − R). The second view reads luma from the
  U buffer (8-bit, half-res luma written in place of U in this mode), so it
  needs no extra EBR. It works with red/cyan glasses.
- **P12 curve.** A signed square would give fine control near the centre
  and leave the clamp to the ends. Check on hardware whether 100% is a
  designed climax or just smear.
- **S8 Near is weak when P12 is centred.** Flipping the gain's sign is the
  same as shifting the sway by half a cycle, so the switch only shows as a
  jump. It is clear only with P12 off-centre. Candidates: depth layers
  (above), or flip the focus side as well.
- **S11 Wrap shows a seam at the line ends.** A Mirror option would have
  no seam.
- **SD.** ±511 px is 71% of an SD line. Scale the displacement to the
  measured width.
- **Generated colour on parity-wandering sources.** Y>U, Y<>V, Rotate and
  Complement make their own chroma, so the per-row parity swap cannot fix
  an avid that wanders within a line. If thin lines show in those modes,
  generate our own avid (cubist v0.3.3) and correct the input chroma by
  line parity.

Tier 3, optional (rework):
- **True occluding parallax.** Forward-warp (scatter) each line in painter's
  order. Process in the direction where the last write is the nearest, and
  add a reverse-scan pass for the other direction, as the shelved
  standalone parallax did. Fill holes from the far side. The silhouettes
  then move and reveal the background, instead of the texture sliding
  inside a fixed outline. It costs about 2 more line buffers plus a tag bit,
  and it feels like a different program.
- Vertical parallax / tilt. It needs a line buffer per line of reach, which
  the EBR budget can't cover at HD.
- Capture defaults and presets from hardware. The presets are hand-written.
  Colour Ghost now uses Split 300 to keep its two colours apart; Split 0
  used to mean V = −U.

## v0.2.2 (2026-10-19): resolution scaling

- **The displacements follow the screen height.** They are pixels, so at
  720p the parallax moved 1.5x as far relative to the picture and at SD 2x.
  The depth gain (Depth x Position/Sway), K5 Offset and K6 Split now follow
  the timing ID's x step: x1 at 1080, x2/3 at 720p ((1/2 + 1/8) x 17/16 =
  0.664, unsigned adds, so no router1 self-add; dupin 0), x1/2 at SD. The
  gain and per-pixel clamps scale the same way: 511 / 341 / 255. This settles
  the "SD: +-511 px is 71% of an SD line" backlog item.
- The edge clamp and Wrap stay on the real measured line width.
- The K5/K6 TOML ranges still read in px: they are 1080p pixels.
- 1080p is bit-identical. Sims (Relief Map settings) match 1080p at 720p
  (r 0.998) and SD (r 0.966); before: 0.79 / 0.58.

## Build

`./build_combo.sh parallax` → `out/rev_b/ron/catalogue-combo/parallax.vmprog`.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 88.9, HD HDMI 83.2, HD Dual 84.3 MHz; SD Analog 90.0, SD HDMI
87.2, SD Dual 85.9 MHz. 4,008–4,031 LC (52%), 18/32 EBR (was 20).
Standalone yosys: `emulate_transparency` count is 0. The first v0.2.0
build, before the multiply split, closed HD Analog at only 74.6 MHz.

v0.2.2 (2026-10-19, router2): HD Analog 87.51, HD HDMI 89.20 MHz (seed 1),
HD Dual 81.12 MHz (seed 2: seed 1 stalled the router, a timeout and not a
timing miss); SD Analog 92.63, SD HDMI 92.22, SD Dual 84.99 MHz (seed 1).
4,146–4,166 LC (54%), 18/32 EBR. Standalone: 1,960 LUT4 (v0.2.1 1,842),
`emulate_transparency` 0, dupin 0.

v0.1.2 (2026-09-13): min HD 77.5 MHz, seed 1.

## v0.1.2 backup

Git 3fa438d (sources unchanged since 2026-09-13).
