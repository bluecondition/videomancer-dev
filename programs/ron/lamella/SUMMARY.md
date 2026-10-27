# Lamella — summary

The picture re-rendered as a wall of glossy metal tiles (after a reference
still of stair-stepped, lensed chrome slats reflecting a copper / lime /
steel landscape). The incoming frame is averaged into a coarse store and
every output pixel is redrawn from it: leaning columns of rounded tiles,
each a small convex lens (magnified middle, ~1.4 tiles crushed into the
rim), a bright rim line and a dark groove, adjacent columns staggered so the
image steps diagonally across the wall. Tone goes through a metal palette;
dark tiles carry a brushed grain.

## Status

- v0.1.0 built 2026-10-16, NOT flashed. Look calibrated against the reference
  with `tools/proto.py` (float look-proto, not bit-accurate; `--emit` writes
  the ROM tables).

## Controls

| Control | Function |
|---------|----------|
| K1 Columns | 3 .. 16 tile columns across |
| K2 Rows | 2 .. 12 tiles over the height |
| K3 Stair | bipolar stagger per column (up to ±½ tile), content steps with it |
| K4 Lean | bipolar column slant (up to one column over the height) |
| K5 Gloss | rim highlight / groove strength, 0 .. 2× |
| K6 Palette | Copper (reference) / Ice / Brass / Chrome |
| S7 Colour | Palette (tone → palette chroma) / Source (stored chroma) |
| S8 Shape | Convex / Concave (lens sign) |
| S9 Shuffle | hashed per-column stagger on top of the stair |
| S10 Brushed | hashed horizontal grain in the dark tones |
| S11 Negative | |
| P12 Depth | lens strength 0 .. 2× (flat tiles .. hard rim crush + centre zoom) |

## Architecture

- **Store**: 64x64 luma (8 bit) + 32x32 chroma (U4 V4), two banks
  ping-ponged by field parity; each grid split into 4 sub-banks by
  (col parity, row parity) so one registered read per sub-bank gives all four
  bilinear taps (feedback's layout, `no_rw_check` on all 8). 20 EBR + 1
  profile ROM + 2 palette ROM = 23 EBR.
- **Write**: input at k=1, one-pole IIR along the line (k=4 wide / 3 narrow,
  chroma +1), captured by a 64-cell DDA; luma rows on the row's middle line,
  chroma rows on the even→odd luma-row line. Chroma stored canonical (swap
  by the capture line's hsync→avid parity, swap back by `line_par` at output).
- **Geometry** (per pixel, Q16 DDAs): column phase `ca` (lean per line), store
  x; per line: tile row phase, store y. Tile phase = row phase + stair·c
  (8×9 multiply) + hashed stagger; content y += stairC·c. Local (lx, ly) →
  distance from the tile centre with a corner-rounding term
  (`d8 + eex²/4`, eex = excess past 7/8 of the half width) → profile ROM
  {delta (signed Q8 tiles), highlight}. Lens: `dv = delta × Sv`, `du = hd ×
  Sh` (operands split hi/lo nibbles, one stage each), sign per tile half,
  added to the store coordinates (wrap mod 64 cells).
- **Per-frame sequencer** (vblank, ~250 clocks): 16×16 serial multiplier +
  23/12 serial divider: `kxc = 2^22/W`, `kyc = 2^22/H`, `cpc = 2^18/ncol`,
  `cpt = 2^18/ntile`, then dcx, dry, dcy, stairC, Sv (clamped 24 cells/tile),
  Sh (16 cells/column).
- **Tone**: negative → grain (hash of {column, tile, ly}: `ly·157 xor
  rotl(c·45 + t·90, 5)`, ×1.0625, xor-fold; spot-checked |corr| ≤ 0.05 on
  all axes) scaled by darkness → contrast ×1.5 about 120 → palette address
  `pal & tone(7..1)` (512×16 EBR) → Y = 64 + 3·tone (+ highlight × gloss ×
  5/16, lower rim halved), clamp 64..800 → parity swap → blanking gate →
  [1 2 1]/4 on Y/U/V (HD HDMI bar conditioning, built in from the start).
- Latency C_LAT = 40 (38 stages + 2 pad), identical in all modes; the output
  is drawn from the store only, no live video carried through the pipe.

## Build

Standalone yosys (program only): 3309 LUT4, 1128 carry, ~3180 FF, 23 EBR,
`emulate_transparency` 0.

v0.1.0, 2026-10-16, builder default router2, all six on seed 1:

| Config    | Fmax (MHz) | LC          | Seed |
|-----------|-----------:|------------:|-----:|
| HD Analog | 90.37      | 6929 (90%)  | 1    |
| SD Analog | 95.27      | 6929        | 1    |
| HD HDMI   | 83.82      | 6933        | 1    |
| SD HDMI   | 91.51      | 6927        | 1    |
| HD Dual   | 76.60      | 6934        | 1    |
| SD Dual   | 85.49      | 6940        | 1    |

23/32 EBR. HD Dual is the tight one (2.3 MHz over Fmin); if a rebuild misses
there, measure the seed spread first.
