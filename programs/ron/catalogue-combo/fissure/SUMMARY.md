# Fissure — summary

Catalogue combo, 2026-10-20. After Lineage, Ron asked for "another effect that uses rarely used
effects from the list". v0.1.0 is built (6/6 seed 1, 89% LC, 20 EBR), not flashed and not simulated.

## Rarely used catalogue entries

These are the counts over the 46 kept combos (the 45 batch combos plus Lineage).

| Uses | Catalogue entries |
|---|---|
| 0 (and possible without a frame store) | kintsugi-style crack overlays, mycelium / growth patterns, reaction-diffusion, tile & shuffle (every combo removed), CRT scanlines / grille / barrel (every combo removed), masking (every combo removed) |
| 0 (need a frame store) | all Temporal, all Feedback, slit-scan, feedback zoom, phosphor persistence, particles advected by optical flow |
| 1 | lens / chromatic aberration (hallway), displacement mapping (mirage), rolling shutter (vhold), cartoon / cel (cartridge), crosshatch / stipple (hatchglass), Kuwahara (impasto), composite crosstalk (lenscreen), RF noise / snow (snowglobe), PiP with warping (tonecurve), cellular automata and hysteresis glitches (lineage) |

Fissure takes the two never-used generative entries that need no frame store: **kintsugi
cracks** and **mycelium / growth**. Lineage's trick carries over: a process whose
generations run down the scanlines needs only one line of state. A crack network fits
this, because cracks are lines and a crack tip is a point. The LZX kintsugi (gold lines on
detected edges) and mycelium (reaction-diffusion) programs are different engines.

A Python look-prototype came first (`.fissure_work/proto2.py`), and it settled three things:
- **Angular fractures, not smooth random walks.** The velocity stays constant between kinks.
  Smooth jitter read as gold worms or drips.
- **No region slip.** Sliding the picture between cracks always produced long horizontal
  bars, because every crack death or merge cuts a seam across the region beside it.
- **Content-keyed randomness.** Each crack's choices come from its own random state, seeded
  by a hash of the picture around its seed point. After a 3-px picture shift, 99.3% of the
  network landed in the same place, so moving video drags cracks along instead of
  reshuffling them.

## Engine

- **Tip list.** Each line has a sorted list of crack tips in a 512 x 64 RAM (two 256-entry
  banks by line parity). An entry holds {p, pf, v Q4.2, mn, mx (the span the tip swept
  since the line above), hue, xorshift16 state, end}.
- **Drawing.** Line n draws each tip as the span it swept since line n-1. A single walk
  pointer steps through the list as the raster passes. The span starts and ends of the
  entries increase strictly, and the ends are at least 7 px apart. So one step per 7 clocks
  (3-clock fetch) draws the list exactly, at any slope.
- **Retiring a tip.** When the pointer passes a tip, the tip becomes an event:
  - it steps its random state three times;
  - it may die (P12), kink (K3), with magnitude r^2 x vmax and its direction taken from the
    picture gradient at the tip (S7) or at random;
  - it may fork (K2).
  The result goes to the other bank.
- **Merging.** A candidate within 8 px of the last entry written is dropped. That keeps the
  list sorted, and it is how cracks run into each other and stop (T-junctions).
- **Seeds.** Seeds fill the gaps between spans. They start at edges (edge strength above
  the K1 threshold, gated by the content hash) or, in Scatter, as a rain over lit areas.
- **Column RAM.** A per-column RAM (2048 x 16) holds {countdown, luma, hue}. It gives
  shallow strokes their thickness: a core pixel restarts a T-line countdown below it. It
  also provides the Shadow groove and the vertical edge term (luma of the line above).
- **Gold.** The bevel distance to the seam edge, plus the picture's own light, picks one of
  64 levels in a 4-metal ROM (256 x 32 EBR, tools/gen_tables.py, U/V swapped). Prism takes
  a hue per crack.
- **Fresh every frame.** Each field starts with an empty list; nothing carries over.

## Hardware contracts

- Every per-field event keys off the vblank zone (no avid for 1024 clocks).
- The picture is carried in a 256-word EBR ring, not an FF pipe.
- The source avid passes through. Generated colours swap U/V on lines whose hsync->avid
  parity differs from the voted usual parity (aurora's red-line lesson, done the drape way so
  the picture still double-swaps correctly).
- Output conditioning (luma [1 2 1], chroma 9-tap, luma x13/16) is built in.
- Outputs are blanking-gated.
- Every RAM has no_rw_check.
- Latency is 32 clocks in every mode.

## Resolution

- Sizes (seam, groove, seed clearance) are virtual 1080-line units.
- Slopes are scale-invariant.
- Per-line probabilities are scaled by virtual lines per field line, so the network looks
  the same at 720p and SD.
- Known differences at SD:
  - the minimum crack spacing is 8 real px, so the densest settings are about half as
    dense;
  - a 1-px seam can't go thinner, so hairlines are relatively bolder.
- On interlaced sources each field grows its own network from slightly different lines,
  so busy settings can twitter at 30 Hz.

## Status

v0.1.0 (2026-10-20): built and packed (`out/rev_b/ron/catalogue-combo/fissure.vmprog`),
timing-closed, not flashed, not simulated.

| Config | Fmax (MHz) | Seed | LC | EBR |
|---|---|---|---|---|
| HD Analog | 92.3 | 1 | 6835 (89%) | 20 |
| HD HDMI | 92.2 | 1 | 6847 | 20 |
| HD Dual | 77.8 | 1 | 6857 | 20 |
| SD Analog | 86.6 | 1 | 6845 | 20 |
| SD HDMI | 80.4 | 1 | 6846 | 20 |
| SD Dual | 90.0 | 1 | 6840 | 20 |

Build history:
- The first netlist was 98% LC at 58.8 MHz; the critical path was the seed decision fanning
  into about 70 event-register enables.
- Seed sub-tests are now registered one clock ahead. Entries store their span ends instead of a
  delta, the per-crack state is 16-bit, and the metal table is 256 x 32 (it infers into EBR;
  a 64-entry table stayed in LUTs). That reached 89%.
- The sorted-write compares gated the clock enable of 64 data registers. The data now loads
  unconditionally and only the write enable depends on the compares: 71 -> 92 MHz.

Start-up: every field begins with an empty list (vblank zone), and the walk only advances after
a line start, so garbage RAM or registers after configuration clear by the first field. There
is no free-running done flag to strand (the dicefall rule).

## Things to watch on hardware

- **Engine sanity.** If no cracks appear at all, the list walk or event pipe is at fault, not
  the look. Seeds can also be starved: turn K1 Spawn up.
- **Boil on moving or noisy video.** The seed key quantizes luma to 5 bits. A noisy source
  near a step can still flip a crack's genome. If it boils, the next steps are a coarser key
  or hysteresis on seeds.
- **HD HDMI bars.** Thin bright gold lines move with the picture. The conditioning is built
  in. If bars appear, widen the chroma filter before touching luma (the wrinkle lesson).
