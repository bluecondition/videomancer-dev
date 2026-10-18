# Subsample — summary

Catalogue combo #37 (seed 20260918): datamosh / block displacement + chroma
subsampling artifacts (4:1:1, 4:2:0, composite). v0.1.0 (2026-09-11) was
flashed and liked ("subsample is good", 2026-09-12), with no later hardware
notes. v0.2.0 (2026-10-12) is the first finalizing pass, built from a review
of v0.1.0. It is **not flashed yet**. Backup of v0.1.0 (sources + vmprog):
`.subsample_work/v010_backup/` (also git 3fa438d).

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Block | block size 8..64 px, continuous (column/row DDA counters) |
| K2 Shift | moshed-block displacement, ±48..±1012 px HD (±24..±506 SD), wraps inside the picture |
| K3 Rate | re-roll every ~2 s .. every field, exponential |
| K4 Hold | chroma sampled every 1..64 px, every integer size |
| K5 Bleed | chroma IIR smear, 0 (off) .. 7 (~128 px tail); live in every switch setting |
| K6 Crawl | dot-crawl depth: luma dots where the output has colour (triangle wave, ~4 px SD / ~11 px HD) |
| S7 Blocks | Shift / Hold (a moshed block repeats its first column) |
| S8 Chroma | After the mosh (hold grid fixed to the screen) / Before (blocks carry their hold boxes and smear) |
| S9 Crawl | Checker: dots alternate line to line and crawl / Bars: vertical bars that march sideways |
| S10 Moshed | moshed blocks keep their colour / go grey |
| S11 Hold Shape | Line: chroma held along the line (4:1:1) / Square: the held sample also repeats down Hold lines (4:2:0 and worse: Hold × Hold colour squares) |
| P12 Density | fraction of blocks moshed, ~5% .. ~98% |

## v0.2.2 (2026-10-17): resolution scaling

Blocks and held-chroma strips/squares ran in pixels / frame lines (1.5x too
big at 720p, about 2.2x at SD), and the HD shift switch (width bit 10) gave
720p the 1080p pixel offsets. Now:
- Block column DDA (Q.1), block row DDA (Q.8), the Hold column counter (Q.1)
  and the Square group counter (Q.8) count virtual 1080-line units stepped
  from the timing ID (registers_in(8)): 1.0 at 1080p (bit-identical), 1.5 at
  720p, 2.0 x 1080/486 at 480i (the per-field-line y step replaces the old
  interlace +2). SD is a left-anchored 1440-unit 4:3 window; blocks and
  squares stay square. A hold shorter than one pixel / one field line samples
  every pixel / line (a single subtraction can't wrap a step larger than the
  period).
- The first line of a Square group is "the counter wrapped on the way here"
  (vca < ys), the same as the old vc = 0 test at 1080p.
- Shift: SD keeps its half offsets; 720p scales the amount by 2/3.
- Pixel-scale on purpose (Ron's rule): the Bleed IIR and the dot crawl.

Verified (new hardware-captured defaults in both baseline and v2): 1080p
diff 0 px; 720p and SD blocks/squares the same fraction of the height as
1080p. Sheets `.scaling_work/sheets/subsample{,_v2}.png`.
Build `./build_combo.sh subsample` (router2, seed 1, all six first try):
HD Analog 79.8, HD HDMI 86.6, HD Dual 87.6 MHz; SD Analog 81.4, SD HDMI 83.3,
SD Dual 83.0 MHz. ~4,160 LC (54%), 18/32 EBR.

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- Re-roll didn't re-randomize. The roll counter was added to the hash key,
  so each "new" pattern was the old one shifted 7 block rows and 117 columns:
  blocks slid diagonally. A per-roll seed now enters after the first hash
  round (Blockcut's fix).
- With 8-px blocks in HD (240 columns) the row key was spaced 128, so the
  right half of the screen copied the left half one block row down. The key is
  now `row & column`.
- K1 Block snapped between 8/16/32/64 px while displayed as continuous.
- K4 Hold snapped between 1/2/4/8/16/32 px with uneven knob spans, also
  displayed as continuous. A column counter now gives every size 1..64 (max
  doubled for HD).
- K5 Bleed did nothing unless S11 was on "Bled" (default "Boxes"). Bleed is
  now always live, and Bleed 0 is the old Boxes look.
- The crawl's line parity was reset on the vsync *level*. When vsync reads
  low across active video, Checker collapsed into Bars. Parity is now counted
  on active lines and reset on the vsync edge.
- Grey moshed blocks still drew dot crawl from their hidden colour. The crawl
  now reads the output chroma.
- `abs(du)` was narrowed with `resize`, so |du| = 512 folded to 0. It is now a slice.
- Chroma was written one pixel left of its luma.
- "Retro" is not a valid category. Categories are now Signal / Glitch / Pixel.

Hardware contracts:
- The output row shows the line above it from the buffer, so U/V are swapped
  on rows whose hsync→avid parity differs from their chroma capture row's
  (drape v1.3 / Benday).
- Displaced reads wrap inside the C_EDGE (12 px) inset instead of showing
  black. A moshed block can no longer pull in the capture's black side
  columns.
- The dot crawl was a 1-px checker at up to ±508 luma, flipping every
  frame. That is the HD HDMI thick-bar trigger profile (sharp, near-full-swing
  pixel steps in motion), and at 1920 px it was also far finer than real dot
  crawl. It is now a smooth triangle wave of the right scale.
- Interlace (field < 600 lines): block rows and Square groups count frame
  lines, so both stay square. The re-roll happens only ahead of a top field.
- `no_rw_check` on all six RAMs. A bank's read data is only used while the
  other bank is written.
- Latency 16 → 22 clocks, the same in all modes. Blanking gate kept.

Controls:
- P12 Density floor ~5%. At 0, v0.1 disabled K1, K2, K3, S7 and S10.
- K2 Shift floor. The ±511 clamp is gone: it piled the top half of the knob's
  moshed blocks onto the same two offsets. Moshed blocks wrap instead.
- K3 Rate is exponential. Before, 90% of the knob covered 2 s .. 0.3 s.
- S11 is new. Its old job, turning Bleed on, is K5's own zero.
- The crawl is weighted by output saturation, as before. Its amplitude is
  doubled to make up for the triangle's lower average swing. Luma is
  clamped to 64..940 so the dots never go super-white.
- Defaults/presets remapped so each keeps its old block size, shift, rate,
  hold and density. "Broken Codec" now uses Square.

## Review backlog (not done)

Tier 2, worth doing after hardware feedback:
- HD HDMI bar conditioning (damier: [1 2 1] on Y/U/V + luma to white ~777),
  only if bars show at high Rate. It is held back because it dims a
  processing program's whites, and the likeliest trigger (the 1-px crawl) is
  already gone.
- Cross-colour rainbows: the other composite artefact. Fine luma detail
  (stripes, text, foliage) leaks into chroma as moving rainbow swirls. It
  could be a K6 partner (Crawl drives both cross-luma dots and cross-colour)
  or S9's second mode.
- Chroma delay: the whole colour layer offset right by N px (VHS colour
  misregistration), e.g. folded into the top of the Bleed knob.
- P12 climax: at 100% the blocks become a full scramble while the chroma
  side is unaffected. The slider's top end could also be designed as a codec
  meltdown, without eating K4/K5's travel.

Tier 3, optional:
- True interpolated 4:1:1 (linear ramps between held samples) as an
  alternative to the IIR smear.
- Box-averaged hold (mean of the period, not its first pixel): less sparkle
  on fine colour detail. It needs a per-frame reciprocal.
- Finer Bleed steps (fractional IIR coefficient instead of 2^-n).
- Rate 100% re-rolls every field (60 Hz strobe). Cap it if it reads as flicker.
- Capture defaults and presets from hardware.

## Build

`./build_combo.sh subsample` → `out/rev_b/ron/catalogue-combo/subsample.vmprog`.

v0.2.0, 2026-10-12 (router2). All six configs passed on seed 1:

| Config | Fmax |
|---|---|
| HD Analog | 79.8 MHz |
| HD HDMI | 79.5 MHz |
| HD Dual | 80.1 MHz |
| SD Analog / HDMI / Dual | 84.2 / 82.2 / 84.8 MHz |

~3,955 LC (51%), 18/32 EBR. Standalone yosys: `emulate_transparency` = 0.
iceunpack OK on all six. Timing note: the first build closed HD Dual at
only 75.7 MHz. The Bleed shift register `s_bn` fanned out to all four IIR
barrel shifters, and its first hop was 6.1 of the 13.2 ns. Signal-level
`keep` copies were still merged into one FF by yosys. Loading each copy in a
different sequencer state stops the merge: 80.1 MHz. The next critical path
is the 9×8 crawl-amplitude multiply (`s_amp19`). Split it hi/lo if a later
change needs margin.

v0.1.0 (2026-09-11): HD 79.7 / 87.3 / 82.4 MHz (analog / HDMI / dual, seed 1),
3,406 LC (44%), 18 EBR.

## Hardware checklist (v0.2.0)

- Defaults should look like v0.1 (same block size, shift, rate, 8-px hold,
  no bleed), except: the dots are now soft ~11 px HD dots, and moshed blocks
  never show black.
- Rate: blocks re-randomize in place (v0.1 slid diagonally).
- Block and Hold: both sweep smoothly (no steps).
- Bleed: works with S11 in either position.
- S11 Square: colour turns into Hold × Hold squares that line up with the
  picture (no colour offset below objects).
- S9 Checker: fine dots that crawl. Bars: vertical bars that march sideways.
  Both only where there is colour.
- S10 Grey: grey blocks carry no dots.
- Density at 0: a few blocks (~5%) still moshed.
- Analog sources: no thin coloured lines over coloured areas (per-row U/V fix).
- HD HDMI at high Rate: watch for thick bright bars (Tier 2 fix ready).
