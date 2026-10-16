# Solarcell: summary

Catalogue combo #81 (seed 20260923): solarization (a Sabattier fold) plus a
mosaic with per-cell colour averaging. v0.1.0 (2026-09-11) was flashed, and
Ron said "solarcell is good" (2026-09-12). v0.2.0 (2026-10-12) is the first
finalizing batch. It comes from a code review of v0.1.0 plus the shelf
audit's notes. v0.1.0 is at git 3fa438d and in `.solarcell_work/v010_backup/`
(sources and vmprog).

v0.3.0 (2026-10-13) adds the Tier 2 items Ron picked after liking v0.2.0
("let's try the Tier 2 fixes, as long as we can revert").

**Reverting to v0.2.0:** flash `.solarcell_work/v020_backup/solarcell_v020.vmprog`
directly. To restore the sources, copy `solarcell.vhd`, `solarcell.toml`,
`solar_fold.vhd` and `SUMMARY.md` from `.solarcell_work/v020_backup/` back
into this folder. Then put `README_section.md` back into the combo README,
`rm -rf build/programs/ron/solarcell/` and run `./build_combo.sh solarcell`.

## Controls (v0.3.0)

| Control | Does |
|---|---|
| K1 Level | **new**: fold brightness. The bands top out at 15–80% of white (v0.2 was fixed at 58%, the default) |
| K2 Gain | fold depth ×1 → ×16 (unchanged) |
| K3 Rows | cell height 1–64 lines (unchanged) |
| K4 Cell | cell width 4–131 px (unchanged); cell colours are now exact means |
| K5 Mix | mosaic over the picture, 25–100% (unchanged) |
| K6 Offset | threshold step per cell row (unchanged) |
| S7 Chroma | Keep / Fold (unchanged) |
| S8 Order | Solar-Mos / Mos-Solar (unchanged) |
| S9 Colour | **new**: Average / Vivid (cell colours at double saturation). Replaces Sample |
| S10 Grid | black cell borders (unchanged) |
| S11 Iso | contour lines, now in **negative ink**: black over light, light (768) over dark, with the complementary hue |
| P12 Fold | **now the only threshold**: 0% folds the highlights only (threshold 748), 50% matches v0.2's default (406), 100% folds the whole picture (64) |

## v0.3.0 changes (Tier 2, Ron's picks)

- **K1 Level** (item 2). solar_fold gained a Q6 brightness multiply
  (`64 + tri × level / 64`). Level 16..87/64 gives peaks of 192..760. The bar
  guard's knee softens the top of that (760 → ~680 on screen). K1 has a 4-LSB
  hysteresis. solar_fold latency 7 → 8.
- **P12 is the threshold** (item 3, K1/P12 overlap). Phase = 800 − 25·P/32,
  computed at full width so it falls monotonically, plus the K6 staircase,
  reflected, then mapped onto 64..939 as `64 + 219·tri/256` (also now exact; v0.2's
  three truncating shifts dipped 1–2 codes every 64 steps). Still glided ¼
  per frame.
- **S9 Vivid** (item 5): cell chroma 512 + 2(c − 512), clamped to the rails.
  Bit slices only, no adder. Applied to the mosaic after the U/V parity swap,
  so it scales with Mix.
- **Iso negative ink** (item 4): the line luma is 64 where the picture under
  it is ≥ 502 and 768 where it is darker; chroma is inverted (complementary
  hue). The grid stays black and wins where both draw.
- **Exact cell mean** (item 7): the running sum × 1/n from a 256-entry
  table {e = ⌊log2 n⌋, R = round(2^(e+9)/n)}, the product split hi/lo, then
  `>> (e + 9)`. n is the cell's real pixel count, so the line's last part cell
  is exact too. Worst error is 2 codes, against up to half a cell ignored before
  (a 48-px cell used to average only its first 32). The write side is 5 clocks
  deeper (the cell is written at d15, 8 px behind its read), which doesn't
  touch the output latency.
- Not built, by Ron's choice: square cells (item 6), the Phase knob (item 3's
  alternative), and the block average (item 1; it would push the mosaic about a
  cell lower).
- Latency 25 → 27 clocks (both solar_folds +1), the same in all modes.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Threshold | where the fold starts, 0–100% of the luma range (64..940) |
| K2 Gain | fold depth ×1 → ×16, linear (display 100–1600%) |
| K3 Rows | cell height 1–64 lines, every integer |
| K4 Cell | cell width 4–131 px, every integer |
| K5 Mix | mosaic over the picture, 25–100% |
| K6 Offset | threshold step per cell row (staircase); it now zigzags instead of running off the end |
| S7 Chroma | Keep: the fold keeps the colour. Fold: the colour inverts in the fold |
| S8 Order | Solar-Mos: solarize, then mosaic. Mos-Solar: mosaic, then solarize |
| S9 Cell Hue | Average: cell colour averaged. Sample: taken from one pixel (renamed; S7 and S9 were both "Chroma") |
| S10 Grid | black borders on all four sides of each cell: 1–5 px wide (by cell width), 1–4 lines tall (cells of 4+ lines) |
| S11 Iso | black contour lines where the picture's fold peaks (the bright crest of each band, and the fold's edge) |
| P12 Fold | lowers the threshold by up to half the range: **more** of the picture folds as it rises. At the defaults, 100% folds nearly everything |

## v0.2.0 changes (Tier 1)

Bugs:
- **The top cell row showed the bottom of the previous frame, and the whole
  mosaic sat one cell row low.** Two banks by row parity meant that each row
  displayed the row above's cells. Row 0 got whatever the last odd row of the
  previous frame left behind. At 32-line rows, that was a 32-line strip of the
  bottom of the picture at the top, and the mosaic ghosted 32 lines low
  against the picture at Mix < 100%. Now there is one RAM. Each row is captured
  on the line before it (reads of a cell come 4 px before its write, so that
  line still shows the old row). Every line of a row shows that row. Line 0
  captures its own row and shows the plain picture for that one line.
- **The top 6% of K2 Gain gave a flat grey fold.** `256 + 4·K2` overflowed 12
  bits at K2 ≥ 960 and wrapped to ×0..×1. Also, the display said 100–500% but
  the gain really ran ×1..×17. Now it is `256 + 3.75·K2`, ×1..×16. That is the
  same feel as v0.1, and the display matches. The default 320 and Iso Mosaic's
  960 give exactly v0.1's ×5.69 and ×15.06.
- **The fold could disappear completely.** The threshold K1 + P12/2 +
  staircase was clamped at 1023. High K1 with high P12 (the shelf audit's
  open question) gave no fold at all. So did the lower rows of a big K6
  staircase. The threshold is now a phase that reflects at both ends. It is
  then mapped onto the luma range (64 + 0.855 × phase). Every setting has a
  fold somewhere, and the staircase zigzags.
- **S10 Grid drew only vertical lines.** It now draws real cell borders, in
  black (it was dark luma with the cell's colour). They are drawn after the
  mix, so they stay solid at any Mix. Thickness grows with the cell.
- **Iso drew black cells, not lines.** In Solar-Mos it took the contours of
  a *second* fold of the already-folded mosaic. At Mix 100% the mosaic is
  flat per cell, so whole cells went black in both orders. It also only
  marked luma codes ≡ 0..5 mod 128 (gradient-dependent blobs). Iso is now a
  contour detector. It marks where {in fold, fold period} changes against
  the left pixel or the line above (a 2048×4 line buffer). That gives
  even 2-px / 2-line black lines at the bright crest of every fold band and
  at the fold's edge. In both orders it follows the source picture's fold, so
  the lines trace the real image over the flat cells.
- **The right edge's part cell was never written.** With SD at 32/64 px
  cells, the last 16 px showed stale or zero cells (saturated green at
  power-on). Part cells are now written at the end of the line.
- **Mix also removed the solarization.** In Solar-Mos (the default), Mix
  crossfaded to the *raw* picture, so lowering it faded out the fold too. Both
  orders now mix against the same picture the mosaic is made from. Mix only
  sets the mosaic amount.
- The threshold was recomputed from the live knobs on every line, so moving
  P12 tore the picture mid-frame. K1 and P12 are now taken once per frame
  through a short glide (¼ per frame). That also calms pot noise, which the
  fold gain amplifies (at ×16, 1 LSB of noise moved the bands 16 codes).

Hardware contracts:
- **U/V swap on parity-mismatched rows** (drape v1.3). Each cell stores the
  hsync→avid parity of the line it was captured on. Rows whose parity differs
  swap the cell's U and V.
- **HD HDMI bar guard**: [1 2 1] on Y/U/V plus a highlight knee
  (600 + (y − 600)/2, so white 940 → 770; the fold, which tops out at 576, is
  untouched). Cell edges and fold bands are sharp, near-full-swing steps in
  motion, which is the bar trigger profile.
- Luma clamped to 64..940 before the guard. Neutral outside avid before the
  smoothing, gated again at the output.
- Rows and Cell are latched per frame with a 4-LSB hysteresis, so pot noise
  at a step edge can't shift the grid (at 4-px cells a 1-px width change
  moves the right edge by hundreds of px).
- `no_rw_check` on both RAMs (both read 3–4 clocks ahead of the write).
- Latency 18 → 25 clocks, the same in all modes.

Controls:
- **P12 now rises toward more fold** (v0.1 raised the threshold, so 100% was
  the least folded, close to the plain picture at the defaults). The standing
  rule is that the slider's top is the climax. Flipping back is one sign.
- K3 Rows and K4 Cell are continuous (column and row DDAs). v0.1 snapped to
  6 and 5 power-of-two sizes behind continuous-looking displays. The range
  also grew to 64 lines and 131 px. A cell's colour is the mean of its first
  2^k pixels (the largest power of two that fits). Power-of-two cells are
  exact, as before; a 48-px cell averages its first 32.
- K5 Mix has a 25% floor. At 0, Rows, Cell, Cell Hue and Grid all did
  nothing.
- K1 shows 0–100%.
- Defaults and presets were remapped to keep their v0.1 sizes, gain and fold
  level: 4-line × 8-px cells, ×5.69. Iso Mosaic and Fine Fold move K1 up to keep
  their v0.1 thresholds under the new P12 direction.

Resources: one cell bank (−4 EBR) and plain counters in place of v0.1's
12-deep 11-bit x pipe. The prefix sums, the iso line buffer and the bar guard
add logic. The build table is below.

## Tier 2 (proposed after v0.2.0; items 2, 3, 4, 5 and 7 built in v0.3.0)

1. **True block average.** A cell's colour still comes from one line, the
   last line of the row above. Thin horizontal detail flickers in tall cells as
   it crosses that line. A column accumulator RAM can average every line of
   the row, at the cost of showing the row above's average. That is the
   one-row lag v0.1 had, but now as a real average. Could live on a switch.
2. **Brighter fold.** The fold tops out at Y 576 (mid grey), so folded
   regions are always dimmer than the highlights they replace. An amplitude
   (or base) knob would let the bands reach ~768.
3. **K1 → Phase.** K1 and P12 both move the threshold (shelf audit). K1
   could instead shift the bands inside the fold (cyclic), with P12 as the
   only threshold.
4. **Iso style.** White lines; lines in the cell's colour; or only the fold
   edge (a Mackie line).
5. **Bolder S9.** Sample vs Average is subtle on smooth sources. Average /
   Vivid (cell chroma ×2, stained-glass) would be a real look flip.
6. **Square cells.** Rows follows Cell (one knob freed).
7. **Exact cell mean** (reciprocal multiply, ~500 LC) instead of the 2^k
   prefix. Only matters for widths well past a power of two.

## Tier 3 (polish)

- Capture defaults and presets from the device (`./capture_preset.sh`). The
  4 presets are hand-written, and "Default" is a no-op slot.
- SD interlace: rows count field lines, so cells are twice as tall in frame
  lines. Iso contours compare against the same field's line above.
- Centred cell grid, so the part cells are split evenly between both edges.

## Build

v0.3.0, 2026-10-13, `./build_combo.sh solarcell` (router2). All six configs
passed on seed 1:

| Config | Fmax |
|---|---|
| HD Analog | 80.79 MHz |
| HD HDMI | 81.75 MHz |
| HD Dual | 79.48 MHz |
| SD Analog / HDMI / Dual | 80.93 / 82.77 / 79.54 MHz |

6,945 LC (90%, v0.2 70%), 7/32 EBR (the 1/n table landed in an EBR).
iceunpack OK on all six. Most of the +1,600 LC is the exact cell mean (six
18×5 multipliers plus the wider shift). It is the first thing to take back
out if a later feature needs room: the 2^k prefix mean costs almost nothing.
Not committed.

v0.2.0, 2026-10-12, `./build_combo.sh solarcell` (router2). All six configs
passed on seed 1:

| Config | Fmax |
|---|---|
| HD Analog | 83.98 MHz |
| HD HDMI | 87.69 MHz |
| HD Dual | 85.65 MHz |
| SD Analog / HDMI / Dual | 75.57 / 78.48 / 78.70 MHz |

5,354 LC (70%, v0.1 56%), 6/32 EBR (v0.1 8). yosys `emulate_transparency` = 0.
iceunpack OK on all six. Vmprog: `out/rev_b/ron/catalogue-combo/solarcell.vmprog`.
The first build (one 11×10 mix multiply per stage) routed HD Analog at
71.7 MHz; splitting the mix hi/lo fixed it. Not committed: commit at hardware
sign-off.

## Hardware checklist (v0.3.0)

- K1 Level: the fold bands go from dim (low) to bright (high). The default
  (58%) should look like v0.2.
- P12: 0% folds only the highlights, the middle looks like v0.2's default,
  and 100% folds the whole picture. The sweep should be smooth, with no jumps.
- S9 Vivid: cells jump to stained-glass saturation. Watch for clipped,
  over-saturated cells on strong colour.
- S11 Iso: lines are black over light areas and light over dark ones, in the
  opposite hue. They should be readable everywhere.
- Odd cell widths (K4 between powers of two, e.g. 48 px): the cell colour now
  responds to the whole cell, including its right side.
- Presets: Staircase, Iso Mosaic, Fine Fold (now Vivid), and Stained Glass
  (new: big cells, Vivid, Grid, Level high).

## Hardware checklist (v0.2.0)

- Top of the picture: the first cell row matches what's under it (v0.1
  showed the bottom of the picture there at big Rows).
- Mix 50%: cells sit on the picture, with no downward ghost. Mix at its floor
  still shows the cells, and the fold stays on at every Mix in both orders.
- K2 Gain to the top: bands keep multiplying, with no flat grey at the end.
- K1 high with P12 high: the fold never vanishes. P12 0 → 100% at the
  defaults: the fold spreads over nearly the whole picture.
- K6 Offset high: rows zigzag through thresholds and none go plain.
- K3 / K4: smooth size sweeps, and a still knob gives no grid jitter.
- S10: borders on all four sides, black, thicker on big cells.
- S11: thin black contour lines that follow the picture over the cells.
- Colour: no hue-flipped cell rows on an analog source (parity swap).
- HD HDMI: no thick bright bars. Highlights are slightly softer (white ~770).
