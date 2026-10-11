# Slipframe: codec meltdown

## What it does
Slipframe redraws the picture as a broken compressed stream. Damage follows a macroblock grid and grows across a GOP (group of pictures). A keyframe then snaps the picture clean, or with Breathe the damage heals away, and the pattern reseeds. There are four kinds of damage:

- **Slice slips:** an error point on a block row, after which the rest of the slice slides sideways and keeps drifting.
- **Drips:** stuck blocks that streak downward, or melt diagonally.
- **Corrupt blocks:** Walsh "DCT" stripes and checkers, smeared blocks or melted blocks.
- **Chroma errors:** the colour plane slips, and corrupt blocks turn magenta or green.

## How it works
**Line buffers.** There is one single-bank line buffer per channel (Y/U/V, 2048×10, 15 EBR), addressed by active column. Every output pixel is read back at `(x − off) mod W`, so the video leaves through EBR and raw input is only 6 FFs deep. Writes lead reads by 5 columns: addresses up to x+4 hold this line, and x+6 and beyond hold the previous line (or a frozen row). The one colliding offset, `W − 5`, is redirected to x, so every RAM carries `no_rw_check` (0 `emulate_transparency`).

**Damage is stateless except drips.** A 3-round shift-add hash of {block, epoch seed} decides it. The shift constants (11,12,9,11,9,15) were searched for low correlation: the worst neighbour |corr| is 0.068 across 6 seeds, 6 fields and 4 directions.
- A block is corrupt when its top 6 bits fall below the GOP ramp AND a second field falls below the P12 density. Growth is monotone, so blocks join and never flicker.
- A new epoch seed (xorshift32) is drawn at every keyframe.

**Drips** use a 256×32 column-state RAM (2 EBR) that stores the valid bit, direction, length code, capture row and last frozen row.
- On block-row-start lines, at each column's last pixel, a live drip grows or shrinks with the current length (K3 × ramp). An expired column may start a new drip.
- A frozen column skips its line-buffer write, so the capture row repeats downward.
- Flow scrolls the frozen row ±1 or 2 px per row, wrapping inside its own block column (`colbase | (xl ∓ s·n) mod B`), which gives a diagonal melt that only ever shows the drip's own captured row. Frozen columns are never written, so these reads cannot collide.
- Bleed freezes only the U/V RAMs.

**Slices** are computed by a per-line hblank sequencer (≤ 49 clocks). It borrows the pixel hash pipe while avid is low, then runs one serial shift-add multiplier for the error column, the displacement and the drift (age × speed). Mod W is done by repeated subtraction.

**Chroma parity.** The U RAM stores `u(9:1)` plus the hsync→avid parity of the row that wrote it. U/V are swapped at output wherever that parity differs from the display row's, which is drape's rule applied per pixel. The chroma DC error adds equally to U and V, so it is immune to the swap.

**Contracts:**
- The blanking gate is at s17.
- Latency is a fixed 17 clocks in all modes.
- GOP stepping is guarded against vsync serration.
- Rows count active lines only.
- Interlaced SD fields (≤ 400 lines) halve the block height.
- Labelled K1 handles the label-index-at-load trap.

## Controls
- **K1 Block:** 8 / 16 / 32 / 64 px macroblocks.
- **K2 Slip:** how far slices slide (up to half the width), plus drift speed (0–9 px per field from the top 2 bits).
- **K3 Drip:** drip length (0 = no drips; at the top, drips reach the bottom).
- **K4 Garble:** Walsh amplitude (±0–255 luma) and melt depth (luma >> 5..2).
- **K5 Chroma:** chroma plane slip in damaged areas (0–127 px) and the magenta/green DC error (±0–511).
- **K6 GOP:** keyframe interval, 0.13 s → 8.5 s (exponential).
- **S7 Drip:** Freeze / Flow.
- **S8 Keyframe:** Snap (sawtooth) / Breathe (triangle: the damage heals).
- **S9 Quant:** damaged pixels get 8 luma levels and 128-step chroma; clean pixels get 16 levels and 64-step chroma.
- **S10 Bleed:** Full / Color (drips carry only colour over live luma).
- **S11 Comb:** odd lines of slipped rows slip one block further.
- **P12 Mosh:** damage density on a quadratic curve, so the lower half stays readable. Blocks run 6 → 75%. Slices run 19 → 100% (linear). Drips run 0.4 → 25% per block-row roll, and every column at 100%. All of these are then scaled by the GOP ramp. Above 50% the ramp floor rises; at 100% no frame is clean.

## Build
`./build_programs.sh ron slipframe` (router2), all six configs closed on seed 1 (2026-10-10):

| Config | Fmax (MHz) |
|---|---|
| HD Analog | 89.4 |
| SD Analog | 80.5 |
| HD HDMI | 82.2 |
| SD HDMI | 83.7 |
| HD Dual | 85.6 |
| SD Dual | 82.5 |

~4,310 LC (56%), 17/32 EBR, 0 `emulate_transparency`. One earlier pre-fix build saw SD Dual's seed 1 stall the router (router2's known silent stall), and seed 2 passed.

## Presets
P-Frame Drift, Melt, Torn Slices, DCT Garbage, Color Bleed, Meltdown. These are hand-written placeholders; capture them from hardware.

## History
- **v1.0 (2026-05):** a smear/tear/persist/crush stack over the raster, never flashed. It failed HD timing (68.8 MHz), its variable-delay lines read the address being written at default settings, and it pulled blanking into the picture.
- **v2.0 (2026-10-10):** complete rework as a codec-meltdown renderer. Not flashed.
- **SHELVED by Ron on 2026-10-10:** not headed for release. The source and `out/rev_b/ron/slipframe.vmprog` (v2.0.0) are left in place; nothing committed.
