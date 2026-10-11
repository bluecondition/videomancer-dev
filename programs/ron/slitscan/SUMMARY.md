# Slit Scan — true time-strip slit-scan

## What it does
A slit (one column of the picture in Vertical mode, one row in Horizontal mode) is
sampled every frame and pushed into a history canvas. The history flows away from the
slit across the screen, so one axis of the output is **time**. Still scenery turns into
streaks, and anything that moves through the slit is drawn as a stretched or squeezed
time-shape. The live picture stays full resolution on the other side of the slit
(Right/Left layouts), or the whole screen can be history (Mirror, Full).

## Why v1 never worked (rewritten as v2.0.0, 2026-10-11)
v1 used three `variable_delay_u` buffers that were 2048 samples deep. One HD line is 2200
clocks, so the deepest "delay" was less than one line. Its "time" axis was really a
sideways shift of each band, which wrapped the previous line's end and the horizontal
blanking into the picture. A `Y <= 16 → hold pixel` hack hid part of this. The old
SUMMARY's "two frames of history" was wrong. v1 also had no blanking gate, labelled
defaults that load as an index, invalid categories, two knobs that both just added delay,
and P12 as a plain Mix. The v1 sources are in git (0360e56) and in `.slitscan_work/`.

The EBR cannot hold even one HD frame (128 kbit against ~62 Mbit). The only true
slit-scan that fits is the time strip: one slit per frame into a coarse canvas.

## How it works
- **Canvas**:
  - 128 time columns × 128 slit cells of 4-bit luma (16 EBR), and 128 × 32 cells of
    U4 V4 chroma (8 EBR). Chroma is stored canonical and signed, so zero RAM is black
    and neutral.
  - Each store is split into 4 sub-banks by (time parity, slit parity), so one read per
    sub-bank gives all four bilinear taps. This is the same store as feedback.
  - The time axis is a circular buffer. A pixel's age is just an address offset from the
    newest column.
  - The display never reads past age 125, so the column being written is never read
    (`no_rw_check`).
- **Capture**:
  - Vertical: an 8 px window at the slit (4 px below 1024 wide), box-summed over the
    first 2^kl lines of each slit cell.
  - Horizontal: the first line at or below the slit, box-summed over the first 8 (4) px
    of each cell.
  - Samples are pre-shifted so every box lands at the same scale (no divides).
- **Exposure**: each cell runs an IIR, `acc += (sample − acc)·m/64`, held in a 256×16
  accumulator RAM. It is a pipelined read-modify-write at cell rate.
- **Commit**: a per-frame phase accumulator (P12 Speed) commits a column on each carry
  and writes the accumulator with stochastic rounding. Above 1 column/frame it writes two
  columns. The display subtracts the fraction, so the history glides smoothly at any
  speed.
- **Rainbow**: each committed column's chroma is turned by a per-column angle on the way
  in. This uses one multiplier over 7 clocks, with a parabolic sin/cos from the vblank
  sequencer. A luma "bloom" is added, so a mono source still gets colour. The stripes
  scroll with the history.
- **Read**:
  - Lookahead DDAs, 9 px ahead of the raster, give the coordinates in canvas units.
    Layout, flow, wave and time key build the age.
  - The age picks two columns (bilinear in time) and the slit coordinate picks two cells
    (bilinear along the slit). Fade then scales by age.
  - The live picture comes from a 16-deep EBR delay (warpfield pattern), not the FF pipe.
- **Blanking gate** on the output. One latency (9) in every mode.
- **Chroma parity** (drape/feedback pattern): the capture swaps U/V on odd hsync→avid
  lines, and the display swaps them back on odd rows.

## Controls
- **K1 Slit**: slit position (glided, 1/8 per frame).
- **K2 Time Key** (bipolar): right = bright parts of the live picture show older history,
  left = dark parts do. Centre is off.
- **K3 Exposure**: motion blur of each new column (instant → ~1 s).
- **K4 Fade**: history darkens with age (gone within 1/8 screen at max).
- **K5 Rainbow** (bipolar): each new column's hue turns a step further, so candy stripes
  scroll with the history; adds colour from luma. Centre is off.
- **K6 Layout**: Right/Down, Left/Up, Mirror, Full.
- **S7 Axis**: Vertical slit (time runs sideways) / Horizontal slit (time runs down/up).
- **S8 Flow**: Out (newest at the slit) / In (newest at the far edge, flowing into the
  slit).
- **S9 Pixels**: Smooth (bilinear) / Blocks (nearest cell).
- **S10 Ghost**: the live picture is lightened over the history.
- **S11 Wave**: the slit bends into a travelling sine wave (±8 columns).
- **P12 Speed**: 1/32 → 2 columns per frame (quadratic). At 1 column/frame, 2.1 s of
  history spans the screen.

## Presets
Hand-written starting points: Time Strip, Waterfall, Mirror Wave, Ghost Trail, Rainbow
Smear, Time Key. Capture real ones from hardware.

## Status
- v2.0.0 (2026-10-11): full rewrite. Built; NOT flashed.
