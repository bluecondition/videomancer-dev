# Vhold — summary

Catalogue combo #73 (seed 20260922): rolling shutter / sync roll + sync loss /
horizontal collapse. v0.1.0 (2026-09-11) was flashed and liked ("vhold is
good", 2026-09-12), with no later hardware notes. v0.2.0 (2026-10-12) is the
first finalizing pass, built from a review of v0.1.0. **HW-validated
2026-10-13**: Ron flashed it and likes the look ("Tested on hardware and I
like the look"); committed as the checkpoint. Tier 2 below awaits his pick. The v0.1.0 sources are in git 3fa438d and in `.vhold_work/v010/`
(with its vmprog, `vhold_v010.vmprog`).

## What the HD default actually looks like

v0.1 keeps only 13 bits of the source x coordinate, so once a row is squeezed
past ~4.3x, the picture repeats every 8192 source pixels. At the HD defaults
every row is squeezed 9x–17x, so the whole screen shows 2–4 squeezed copies of
the picture per row. The copies are smaller and denser near the bar, and they
fan out down the screen. This tiling is almost certainly what was liked on
hardware, so v0.2 **keeps it on purpose**. HD keeps the 8192-pixel period.
SD now uses 4096 (5.7 widths), so it tiles like HD; with 8192 (11.4 widths),
SD barely tiled at all.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Roll | roll speed, bipolar; ±16-count deadband at the centre = locked. ~5.7 s a cycle at the default, 0.55 s at the ends, the same in every mode |
| K2 Bar | bar height 0.8 .. 26% of the screen (default 2.9% = 31 lines at HD, as v0.1) |
| K3 Jello | sine lean, up to 1/8 of the width (±240 px HD); 1.5 sine cycles a screen in every mode |
| K4 Collapse | base squeeze 1x .. 9x (display now shows the factor), glided |
| K5 Reach | how far below the bar the pull and the tear reach: 1/32 .. 8 screen heights (quadratic; default ≈ 1.9 heights, as v0.1) |
| K6 Beam | squeezed rows brighten (lift, as v0.1); stretched rows now dim |
| S7 Bar | Black / Show (the bar's rows show the peak-pull picture) |
| S8 Centre | Screen: strips stay centred, jello pans the content / Jello: the collapse centre wobbles, so whole strips swing |
| S9 Edges | Black / Mirror |
| S10 Tear | random per-line tear of up to ±1/8 of the width, strongest just below the bar and fading with Reach |
| S11 Pull | Squeeze / Stretch; Stretch zooms by the same factor that Squeeze squeezes |
| P12 Pull | extra squeeze just below the bar, decaying with Reach; glided, with a small floor (≈1.5x at 0%) |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **K3 Jello folded back.** Its top 9 bits were read as a *signed* number, so
  the amplitude rose to its maximum at 50% and then fell back to ~0 at 100%
  (with the phase flipped). The Jello Stretch preset (K3 900) only got ±31 px.
  It is now 0 → max over the full knob.
- **The pull overflowed.** The 14-bit signed squeeze factor wrapped past 8191
  at high Collapse + P12, and the overflowed rows clamped to the *opposite*
  extreme, an 8x zoom. Example: the Dying Set preset had a ~250-line band of
  zoomed picture just below the seam. It is now 15 bits, with no wrap.
- **S8 Centre was dead below the bar.** The below-bar branch overwrote the
  centre for every row from the bar down to the screen bottom, even with Tear
  off. In Jello mode with no squeeze, jello vanished entirely: it cancelled
  itself. Both are fixed: Jello mode now moves the collapse centre by the
  jello offset at every squeeze.
- **K5 Reach was mostly dead.** The decay length was 256 << (K5/128) lines.
  At HD, settings 3–7 all reached past the bottom of the screen; at SD, only
  setting 0 differed. It is now a continuous fraction of the measured height
  (a per-field serial divide gives the slope).
- **The bar didn't wrap.** Off the bottom edge it was clipped, then it popped
  in whole at the top. Distance is now modulo the height, so the bar rolls
  off the bottom as it re-enters at the top.
- **The peak pull was hidden under the bar.** The pull peaked at the bar's top
  line, so a tall bar hid it. It is now measured from the bar's bottom, and
  the bar rows sit at the peak (S7 Show shows a plateau).
- **S11 Stretch saturated.** It was `q = base − pull`, clamped at 8x zoom, so
  above ~10% P12 nearly every row was at the clamp, and P12 acted as a
  switch. It is now `base · 256 / (256 + pull)` (per-line serial divide), so
  P12 sweeps the zoom over its whole travel (max 16x).
- **Beam lit stretched rows at full brightness.** `(q − 256)` went negative,
  was read as unsigned, and clamped to the maximum lift. Stretched rows now
  dim instead: a luma gain toward black, scaled by Beam.
- **Tear coverage popped once per roll cycle.** Tear covered "bar bottom to
  screen bottom", so it grew to almost the whole screen when the bar wrapped.
  It now follows the pull weight: strongest under the bar, fading with Reach,
  and it wraps with the bar.

Mode scaling (it was all fixed pixel/line constants tuned to nothing):
- **Bar:** 255 lines was 24% of HD but 100% of an SD field. It is now a
  fraction of the height.
- **Roll speed:** v0.1 took 0.56 s a cycle at HD but 0.13 s at SD. It is now
  a 16-bit phase × height, the same cycle time in every mode.
- **Jello:** the amplitude is now a fraction of the width, and the spatial
  frequency is 1.5 cycles a screen. v0.1 gave HD 1.6 cycles and an SD field
  0.35.
- **Tear:** v0.1 was ±256 px (13% of HD, 36% of SD). It is now ±1/8 of the
  width.

Hardware contracts:
- **Luma cap.** The Beam lift drove luma to 1023, which clips all RGB and
  kills chroma. Luma is now clamped to 64..940, then a highlight knee (600 +
  (y − 600)/2) brings white to ~770. Mid-tones are untouched.
- **HD HDMI bar fix, built in.** [1 2 1] smoothing on Y/U/V before the gate.
  Heavily squeezed rows are point-sampled 9–40 source pixels apart, which
  means full-swing pixel-to-pixel edges in motion: exactly the bar trigger.
- **Per-row U/V parity swap (drape v1.3).** Every row shows the line stored
  one row earlier. On analog sources whose hsync→avid offset wanders, rows
  whose parity differs from the stored row's had Cb/Cr swapped.
- **Interlace (SD).** The jello line phase advanced on every hsync, including
  blanking lines. That is the PULP 30 Hz twitter trap. It now advances on
  active lines only, and the bottom field (`field_n = '0'`) starts half a
  step down. HD has no field flag, so HD is unchanged.
- **Width latched once a field.** Analog line-length jitter no longer jitters
  the strip centre line by line.
- One latency (16) in all modes; neutral outside avid before the smoothing,
  and gated again at the output.

Controls (bold, always-on):
- **Bar floor (0.8%).** At Bar 0 with P12 0, Roll moved nothing visible.
- **P12 floor (≈1.5x at the bar).** At P12 0, Reach and S11 did nothing.
- **Roll deadband (±16).** Pot noise at the centre made "locked" creep.
- **P12 and Collapse at 10 bits, glided (¼ per field).** v0.1 used 8-bit
  steps. One P12 step moved the outer copies ~7 px, so knob noise made them
  shimmer.

Optimizations:
- `no_rw_check` on all 6 line-buffer RAMs (a bank is only read while the other
  is written).
- Removed the dead 5-stage delay chain and the 13-stage bar-flag pipe. Row
  flags only change in hblank.
- Per-field maths on one serial shift-add multiplier and one restoring
  divider. Per-line maths is one op per state (~36 clocks after hsync, inside
  every mode's hblank and the sim's 64-clock fake one).

## Hardware checks for v0.2.0

1. **HD default:** does it still look like the v0.1 you liked? The tiled
   strips, roll and bar should match; whites are dimmer (knee) and edges a
   touch softer ([1 2 1]).
2. K3 Jello sweep: should grow all the way to 100% (v0.1 faded back above 50%).
3. Roll the bar off the bottom: it should re-enter at the top continuously.
4. S8 Jello: whole strips should swing side to side.
5. S11 Stretch + P12 sweep: a gradual zoom over the whole slider, with
   stretched rows dimmer as Beam rises.
6. K5 Reach sweep: from a tight band just under the bar to a near-uniform pull.
7. Dying Set preset: no zoomed band under the bar any more.
8. HD HDMI: any thick bars? Analog colour source: any thin coloured lines?

## Review backlog (not done)

Tier 2, look changes (your call):
- **Single strip vs tiles.** A true "dying set" collapse is ONE centred strip
  on black, not tiles. This could be a mode (e.g. S9 Edges: Black /
  Mirror / Single), but switches are binary, so it would replace something.
- **Beam lights only the picture.** The lift also greys the black gaps between
  the strips (as in v0.1). Making Beam a gain would keep blacks black, so
  the strips glow instead of the whole row washing grey.
- **P12 100% climax: full collapse.** Rows at the bar would collapse to a thin
  bright vertical line (100x+), the classic dying-TV signature. Today 100%
  is a ~40x comb of thin tiles.
- **S7 Show → Sync.** The bar would show the vertical interval: a
  blacker-than-black sync band with equalizing-pulse dashes, as on a real
  rolling set. Today Show just means "no bar".
- **Tear as flagging.** Today the tear is white noise per line, every field.
  Real horizontal-hold loss bends the lines below the bar smoothly (a hook
  that relaxes down the screen). This would be a random walk per line, or a
  decaying bend.
- **V-hold hunting.** Near lock, the roll would catch and slip (stick-slip)
  instead of creeping at a constant speed.
- **Exponential Collapse.** K4 is linear in the squeeze factor, so most of
  the visible change happens in the first quarter of the knob.

Tier 3, optional:
- Capture defaults and presets from hardware (the 4 presets are hand-written,
  translated to v0.2 ranges).
- The top row shows the previous field's last line (1-line replay). It is
  invisible in practice.

## Build

`./build_combo.sh vhold` → `out/rev_b/ron/catalogue-combo/vhold.vmprog`.

v0.2.0 (2026-10-12): all 6 configs pass on seed 1 (router2 default).
HD Analog 80.7, HD HDMI 83.6, HD Dual 81.0 MHz; SD Analog 78.2, SD HDMI
86.0, SD Dual 78.1 MHz. 5,585–5,616 LC (73%), 19/32 EBR. Standalone yosys:
3,223 LUT4, `emulate_transparency` 0. A first build at 71 MHz (HD Analog)
had the per-line `dp × slope` (11×16) in one state; splitting it and
`pc × q` hi/lo closed it. The +1,800 LC over v0.1 is mostly the per-line
multipliers (8 parallel products) and the two dividers. If room is ever
needed, time-share one per-line multiplier (the line sequencer has ~100
spare clocks).

v0.1.0 (2026-09-11): HD 85.95 / 77.97 (seed 2) / 79.80 MHz (analog / HDMI /
dual), 3,785 LC (49%), 19 EBR.
