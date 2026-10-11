# Solitude — glitch cascade on teal

## What it does
A ragged mass of crawling pixel-sort streaks in jewel tones (magenta /
crimson / cobalt / violet / emerald / amber, with maroon shadow runs) floats
on a flat muted teal background. Every stripe column is cut into runs of its
own length; each run is a single colour that shades dark to bright along its
length, the way a brightness sort leaves a column, and the runs crawl down
(or up) at per-column speeds. Slanted colour passes separated by jagged black
seams shift the palette from pass to pass. The top edge is a sloped, stepped
skyline with towers; the side edges break into columns of uneven height;
drip columns stretch out of the bottom, detach and fall to the floor. White
lightning dashes and speckle cap the top edge and re-strike as the picture
flows; whole rows occasionally flip to inverted colour.

## How it works
Pure synthesis. The program makes its own avid (cubist/aurora parity-locked
recipe) and blanks to neutral outside it. Rows count active lines from the
vsync edge (frame lines under interlace); all geometry is fractions of the
measured raster, and top-edge / pass / run scales double at HD.

- **Column hash**: stripe index → three add-based mixing rounds (the old
  xor-only hash gave ~16-stripe solid blocks and near-identical passes).
  It picks each column's run length (32–512 rows), run phase, crawl speed
  (×1..×4), drip, tower height, edge fringe, seam jitter and zone dither.
- **Runs**: `y_eff = y + phase ∓ offset`; run index = `y_eff >> k`, position
  = the next 6 bits. Run colour = hash(column, run, pass seed) → bin →
  (palette, x-zone + pass) slot. Shading: luma 15/64..63/64, chroma
  32/64..63/64 along the run (three small multiplies on one stage).
- **Passes**: bands of `y − x/2` (64 / 128 / 256 rows for SD / 720 / 1080);
  each pass rotates the zone palette and reseeds the run colours; seams are
  black edges of the band fraction with per-column jitter.
- **Top edge**: H/16 + x/4 slope + x-triangle + 32/64-px steps + towers
  (K1 scales them; 1/4 of columns may rise the full amount).
- **Drips**: body ends at 7H/8; drip columns carry a segment whose head
  travels H/4 per cycle (grows from the mass, detaches, falls off-screen,
  wraps unseen).
- **Motion**: one per-field time accumulator (K4 Flow, guarded vsync) drives
  crawl offsets, drip phase, and the lightning / invert-row epoch.
- Colours authored in BT.601, swapped U↔V at the output compose only.

Latency 18, no EBR.

## Controls
- **K1 Edge** — tower height on the skyline top edge (0 = stepped slope only)
- **K2 Drip** — how many columns drip
- **K3 Lit Dns** — lightning density near the top edge
- **K4 Flow** — crawl speed of runs and drips, lightning re-strike rate (never stops)
- **K5 Stripe** — stripe width 1 / 2 / 4 / 8 px
- **K6 Lit Depth** — how far down the lightning reaches
- **S7 Palette** — Jewel / Cool
- **S8 BG** — Teal / Black
- **S9 Seams** — Off (continuous sort) / On (slanted passes with black seams)
- **S10 Lightning** — Off / On
- **S11 Sort** — Down / Up: run shading and crawl direction
- **P12 Bright** — brightness about black, 50 % = unity

## Presets
- Default
- Tight Mass
- Dissolving
- Cool Corruption (Cool palette, no seams, Sort Up)
- Black Void

## History
- v0.2.0 (2026-04/05): static picture, sim-only milestone (chaos effects:
  slanted passes, stepped top, speckle, invert rows).
- v0.3.0 (2026-10-10): hardware-correctness + motion pass. U/V output swap
  (v0.2 would have shown an ochre background and red↔blue swapped), add-based
  hashes, K1 Edge fixed (was dead), Bypass removed (green on synthesis),
  active-row geometry, generated avid + blanking gate, luma capped, side-
  cascade machinery and the video delay pipe deleted; sorted shaded runs,
  crawl, falling drips, re-striking lightning. K4 → Flow, S9 → Seams,
  S11 → Sort.

## Status
v0.3.0 (2026-10-10): 6/6 closed on seed 1 (router2), ~5100/7680 LC (66%),
0 EBR. HD Analog 85.1, HD HDMI 87.1, HD Dual 81.1, SD 82-88 MHz.
Never flashed. **Shelved by Ron on 2026-10-10** (not headed for release).
