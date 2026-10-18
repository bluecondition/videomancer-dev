# Snowglobe: summary

Catalogue combo #68 (seed 20260922): pixelation + RF snow / VHS tracking +
a sphere or tunnel row lens. Ron flashed v0.1.0 in September 2026 and said
"snowglobe is good" (EXPLORE_LIST, 2026-09-12). v0.2.0 (2026-10-12) is the
first finalizing batch. It comes from a code review of v0.1.0. The v0.1.0
sources are at git 3fa438d.

## Controls (v0.2.0)

| Control | Does |
|---|---|
| K1 Cell | cell width 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 px, labelled, deadbanded |
| K2 Rows | cell height 1 / 2 / 4 / 8 / 16 / 32 / 64 / 128 lines, labelled, deadbanded |
| K3 Snow | snow amount. Up to 50% it is the same as v0.1. Above that the snow keeps growing (to ±382 codes) and from 62.5% the picture fades toward grey (25% left at 100%): a lost channel |
| K4 Tracking | fraction of lines that lose tracking (torn, darkened, desaturated) |
| K5 Bulge | lens strength, bipolar (pinch .. bulge), glided |
| K6 Radius | lens radius, 3–140% of the half-height (was a fixed 16–783 lines), glided |
| S7 Cells | cells in lens (source) space / in screen space |
| S8 Snow | Fine (per pixel) / Cells (one value per picture cell, in both S7 modes) |
| S9 Lens | Sphere / Tunnel |
| S10 Edges | Black / Mirror (once, about a 12-px inset, black beyond) |
| S11 Snow | Mono / Colour |
| P12 Squeeze | horizontal scale, 50% (2× stretch) .. 200% (two picture widths across), glided. Was labelled Zoom: same behaviour, the old name read backwards |

## v0.2.0 changes (from the v0.1.0 review)

Bugs:
- **Ghost copies of the picture beside the globe.** The source address was
  12 bits and the DDA 21 bits, so strongly squeezed rows (near the globe's
  top and bottom, at the default Bulge) wrapped round and showed squeezed
  copies of the picture to the left and right of the globe instead of black.
  The DDA is now 24 bits and the address saturates outside ±4096 px.
- **Squeeze flipped to 16× magnify.** The row step (lens × P12) was a 12-bit
  signed value; above 8× it wrapped negative and was clamped to the minimum,
  so pole rows jumped from squeezed to hugely magnified once P12 passed about
  half. The step is now 13 bits and clamps at 16×.
- **Pole rows snapped back to near-unity at Bulge > ~93%.** The lens term
  `256 + fk/128` was narrowed to 12 bits and folded (the narrowing-resize
  trap). It is now computed at 14 bits before the clamp.
- **The picture sat a whole cell row low, and the top cell row showed the
  previous frame's bottom** (up to 32 lines at Rows 32). Rows are now
  captured on their own first line and shown from the next line on (the
  stencil v0.2 scheme). The shift is 1 line at every Rows setting, and only
  the frame's first line has no capture (it shows black + snow).
- **Snow was a 16-bit LFSR shifted one bit per pixel.** Each pixel's noise
  was its neighbour's bits shifted (strongly correlated), the pattern repeated
  every 65,535 pixels (every ~30 lines at HD, a diagonal texture in the snow),
  and the Y/U/V noise bytes overlapped. Snow is now a two-round shift-add hash
  of (column, row, per-frame seed). Numerically checked: neighbour
  correlations ≤ 0.03 for single pixels, every cell size and negative lens
  cells, Y/U/V uncorrelated, frame-to-frame uncorrelated.
- **Cells snow was not per cell.** It held a value across the cell's width but
  took a new one on every line, so cells were 1-line streaks, and it changed
  one pixel after the picture cell's edge. It now hashes the picture's own
  cell (the lens-space cell in Lens mode, the screen cell in Screen mode)
  with the row index, so each cell is one flat noisy block.
- **Lens parameters were read live.** Bulge, Radius and Zoom fed both the
  per-frame 1/r divider and the per-line tests. A change mid-frame could
  push the table index out of range on a row. They are now latched per frame
  and glided (ease 1/8 a frame), which also stops pot noise from shimmering
  the globe outline. K1/K2 are deadbanded (±12) so cell size can't flicker at
  a step edge.
- **K1/K2 had uneven steps.** 6 sizes were spread over 8 knob zones (2 px and
  32/64 px got 1/8 of travel, 4 and 16 px got 1/4), and the display showed a
  linear "px"/"ln" value that didn't match the step. Both knobs are now 8
  even octave steps with labels (1–128). The defaults are unchanged (8 px,
  4 lines). The labelled-default index trap is handled (values below 8 are
  label indexes).
- **Snow pushed luma to 0 and 1023.** Luma is now clamped to 64..940 and
  chroma to 64..960.
- Tracking tears scale with the picture: ±512 px at HD, ±256 px in SD (was
  ±512 everywhere, more than half an SD line). Radius is a fraction of the
  measured height, so SD globes are proportional. HD is unchanged.

Hardware contracts:
- **U/V parity swap on replayed rows** (drape v1.3). Every output row is a
  stored line, so rows whose hsync→avid parity differs from their capture
  row's would show swapped hue. This only matters on analog sources whose
  parity wanders.
- **HD HDMI bar guard:** [1 2 1] on Y/U/V plus luma scaled to white ~775,
  before the gate. Fine snow is per-pixel full-swing noise in motion, which
  is the bar trigger profile. Visible side effects: the snow is very slightly
  softer and whites are ~19% dimmer.
- `no_rw_check` on all 6 line-buffer RAMs (a bank is never read while it is
  written).
- Mirror reflects about a 12-px inset, so the capture's black blanking columns
  never show as a seam (Ron's edge-fill rule).
- Latency 20 clocks in all modes (unchanged); blanking-gated.

Presets: the four v0.1 looks are kept. K1/K2 values are remapped to the same
cell sizes on the new labels. Chunky Storm's Snow is 760 (about v0.1's
maximum, now with a light fade). There is one new preset: Lost Channel (Snow
100%, Tracking 30%). All are hand-written; recapture them with
capture_preset.sh.

## v0.2.2 (2026-10-17): resolution scaling

Cells, rows, the lens and the tracking tears now run in virtual 1080-line
units, stepped from the video timing ID (`registers_in(8)`): x 1.0 / 1.5 /
2.0 units a pixel (1080 / 720p / SD), y 1080 / active lines a field line
(interlace included).
- **Cells / Rows** were fixed pixels and field lines: 1.5× too big at 720p,
  and in SD ~2.7× too wide and ~4.4× too tall. Now:
  - Rows: a row starts where the line's virtual y crosses a multiple of the
    row height.
  - Screen cells: the source is held where the virtual x crosses a cell edge.
  - Lens cells: the source column is quantized to the virtual grid. That is
    C px at 1080p and C/2 at SD (a mask). At 720p it is 2C/3 px: within each
    2C-px block the three cells start at 0, ceil(2C/3) and ceil(4C/3), two
    per-frame thresholds and an OR, with no divide. This was checked exactly
    against floor(3s/2C) for every size.
  - The Cells-snow keys follow the same virtual cells.
  Cells and rows are the same fraction of the screen height in every mode,
  and square cells stay square on a 4:3 SD screen. This covers the Tier 2
  "SD interlace rows" item.
- **Lens**: centre, radius and the 16-line minimum radius are virtual.
- **Tracking tears** were ±512 px whenever the width had bit 10 set, so
  720p got the HD size. They are now ±512 virtual units: ×1 / ~×0.66 / ×1/2
  in px.
- Fine snow (per pixel, per field line) and the lost lines stay at pixel /
  line scale.
- Not about scaling: `s_amp` (Snow above 50%) was `k(9:2) + k(8:2)`, a
  self-add (a carry with both inputs on one net) that livelocks router1 on
  some seeds. It is now `128 + 2·k(8:2)`, the same value for every input
  (checked exhaustively).
- 1080p is bit-identical (0 differing pixels against v0.2.1).

## Next tiers (offered 2026-10-12, not done)

Tier 2 (look, Ron's call):
- **Analogue snow grain:** specks 2–4 px wide and 1–2 lines tall, like snow on
  an SD set. Per-pixel noise at HD reads as fine digital grain. This could be
  a third S8 mode, or replace Fine.
- **VHS tracking band:** lost lines bunched into a band that slowly rolls,
  with tears growing toward the band's centre, instead of scattered single
  lines.
- **Squeeze range:** extend P12 to 400% (the picture a quarter wide), so the
  top of the slider shrinks the globe into a small ball.
- **SD interlace rows:** rows count per field, so SD cells are twice as tall
  in frame terms. Halve Rows when the measured height is below 300 lines.

Tier 3 (bigger, ask first):
- **P12 Shake:** a snowglobe being shaken. The globe centre swings and
  bounces, and the snow storms up. The climax would be a violent shake in a
  blizzard. Squeeze would move to a knob, or go.
- **Round globe:** fit the lens to a circle, so the picture is squeezed into
  a ball of the Radius rather than a full-width ellipse.
- **Falling flakes:** real snowglobe flakes, as sprites drifting over the
  picture.
- Presets captured from hardware; v1.0.0 sign-off; shelf card; commit.

## Build

`./build_combo.sh snowglobe` → `out/rev_b/ron/catalogue-combo/snowglobe.vmprog`.

v0.1.0 (2026-09-11): HD 79.0 / 80.2 / 77.2 MHz (an / hdmi / dual), ~4,660 LC
(61%), 18/32 EBR, seed 1.

v0.2.0 (2026-10-12, router2 default, built alongside two other tabs'
builds): all 6 configs pass. HD Analog 76.7 (seed 1), **HD HDMI 79.2 on
seed 2** (seed 1 missed), HD Dual 76.8 (seed 1); SD Analog 76.7, SD HDMI
75.5, SD Dual 79.8 MHz (seed 1). About 6,550 LC (85%), 18/32 EBR. Standalone
yosys: `emulate_transparency` = 0 (v0.1.0 had 3), so no_rw_check took
effect. The +1,900 LC are the snow hash (7 × 32-bit stages), the six new
snow multiplies (fade and amplitude on Y/U/V) and the bar guard. If room is
ever needed, the fade could become a shift-add on fewer levels.

v0.2.2 (2026-10-17): **router1, SEED=3** (the community port's seed):
`SEED=3 NEXTPNR_ROUTER="--router router1" ./build_combo.sh snowglobe`. All
6 configs pass on seed 3: HD Analog 84.1, HD HDMI 77.0, HD Dual 83.6 MHz;
SD 89.1 / 83.3 / 83.5 MHz. About 6,700 LC (87%), 18/32 EBR. All six .bin
files unpack. The default router2 build missed HD HDMI on all six seeds
(best 73.5 MHz; a manual seed spread gave 70.5–75.5). Its critical paths
were the old one-clock per-line `cx × srow` product and the per-pixel fade
multiply off the high-fanout `s_fw`. router1 closed the same netlist at
81–85 MHz on seeds 1–4.

HW status: v0.2.0 built, not yet flashed.

## Build seed — community port (stock LZX SDK 0a65d53, router1, all 8 configs)

**seed 3**, set as `[firmware] seed = 3` in the staged
`programs/czartroniks/snowglobe/snowglobe.toml`. Seeds 1–2 hung router1 (a
rip-up livelock) in at least one config; seed 3 builds all 8 (max 6831 LC),
own HD 80.5 / 77.7 / 83.6 MHz (analog / HDMI / dual). This dev TOML carries
the same `[firmware] seed = 3` since the rework was committed as the staged copy.
