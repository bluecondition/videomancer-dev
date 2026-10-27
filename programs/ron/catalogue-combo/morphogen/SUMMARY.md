# Morphogen — summary

Catalogue combo, 2026-10-21. After Lineage and Fissure, Ron asked for another program built from
rarely used catalogue effects ("do another one"). v0.1.0 is built (6/6, 89% LC, 17 EBR), not
flashed and not simulated.

## Choosing the effects

These counts are over the 47 kept combos, including Lineage and Fissure.

| Uses | Entries |
|---|---|
| 0, no frame store needed | **reaction-diffusion driven by input**; tile & shuffle, CRT scanlines / grille / barrel, masking (every combo using these three was removed after Ron's review) |
| 0, needs a frame store | Temporal, Feedback, slit-scan, feedback zoom, phosphor persistence, optical-flow particles |
| 1 | lens / chromatic aberration, displacement mapping, rolling shutter, **cartoon / cel**, crosshatch, Kuwahara, composite crosstalk, RF snow, PiP warp, CA, hysteresis, kintsugi, mycelium |

Reaction-diffusion was the last never-used generative entry. It runs the same way as Lineage and
Fissure: one line of state, with the generations stepping down the screen.

The second effect was a choice between two pairings:
- **Lens / chromatic aberration** would have made "droplets refracting the picture". That repeats
  Dewdrop (glossy lens-drops with spectral rims), so I didn't use it.
- **Cartoon / cel** (flat colour plus ink) fits how reaction-diffusion really works: it is how animal
  coats get their spots and stripes. The ground is the picture flattened into cel bands with ink
  outlines, and the reaction draws beads, spots and blots on top.

The LZX Mycelium program is also Gray-Scott, but a different instrument. It is a 3-species texture
synthesizer with a palette, run as one raster pass a frame. Morphogen is 1-D across each line,
stepped down the screen, fed by the picture, and it redraws the picture as beads.

## What the prototypes showed (`.rd_work/`)

- **Material.** 1-D Gray-Scott stepped down the scanlines draws glossy beads, teardrops, streaks and
  branching arches. The picture comes back as beads.
- **Seeding.** Seeding only at picture edges (`pA.png`) reads best. Rain everywhere (`pC.png`) gives
  the encrusted climax, and that is what P12 Bloom does at the top of its travel.
- **Excitable medium.** I also tried an excitable medium (Barkley), but it gave only small tents and
  a checkerboard instability.
- **Fixed-point model** (`fx.py`, the exact hardware update, Q0.12 state):
  - It reproduces the float look.
  - With 8-bit reaction operands it keeps the same character, slightly sparser (`prec8.png`).
  - dt = 2 on half the lines matches dt = 1 at full height (`fxsheet_dt*.png`), so the pattern
    keeps its shape across modes.
- **Species knob.** The (feed, kill) sheet shows the species differ mainly in density and
  streakiness, so this knob is a texture choice rather than a dramatic switch.

## Engine

- **Cells.** A cell is K2 virtual pixels wide (a DDA on the 1080-line canvas). Each cell is at
  least 2 clocks long in every mode.
- **State.** {u, v} is Q0.12 per cell, in a single-bank 1024 x 32 RAM. The old state of cell k+2 is
  read at the start of cell k, and the new state is written about 9 cells later. Cells 0 and 1 are
  primed from the input avid edge, before the line reaches the cell stage.
- **Update pipeline.** One 7-stage Gray-Scott pipeline, shared by the two sub-steps of each line:
  - slot A (cell start + 1) steps cell k-2 from the old state plus seeds, into a 256-entry
    hand-off RAM;
  - slot B (cell start + 2) steps cell k-9 from that RAM.
- **Update arithmetic.**
  - Reaction terms use 8-bit operands.
  - Diffusion (Du 0.19·dt, Dv half) and drift are shift pairs.
  - Each cell's feed comes from its luma: 0.6 + 0.8·L (inverted for Drive Dark).
- **Second RAM.** A 1024 x 16 RAM keeps each cell's luma and the previous line's v. They provide the
  vertical edge for seeding and the vertical slope for shading.
- **Seeds.** |left step| + |vertical step| of cell luma above the Bloom threshold, or Rain: a hash of
  (cell, virtual line), spot-checked for lattice structure.
- **Display.** Shows the previous line's state, interpolated across each cell.
  - Bead: v > 0.2.
  - Rim band: set by K5.
  - Light: from the horizontal plus vertical slope.
  - Skin colours follow, then the usual output conditioning.
- **Fresh every frame.** The first line of a field starts empty, so nothing carries between frames.

## Hardware contracts

- Every per-field event keys off the vblank zone.
- The picture is carried in a 256-word EBR ring.
- Generated chroma (the Coat and Ink palettes) swaps U/V on lines whose hsync->avid parity is off.
- Output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma x13/16) is built in.
- Outputs are blanking-gated.
- Every RAM has no_rw_check.
- Latency is 24 clocks in every mode.

## Resolution

- dt per line is 1 at 1080p, 1.5 at 720p and 2 elsewhere. The nominal SD values (2.25, 3.75, 4.4)
  are capped at 2 for stability.
- **480p and 576p** evolve about 10% slower: patterns are a touch taller.
- **480i and 576i** evolve about 2x slower: patterns are noticeably taller and streakier.
- 1080i matches 1080p.
- Cell sizes are in virtual units, so beads are the same size on screen everywhere.

## Status

v0.1.0 (2026-10-21): built and packed (`out/rev_b/ron/catalogue-combo/morphogen.vmprog`),
timing-closed, not flashed, not simulated.

| Config | Fmax (MHz) | Seed | LC | EBR |
|---|---|---|---|---|
| HD Analog | 84.4 | 1 | 6862 (89%) | 17 |
| HD HDMI | 87.4 | 1 | 6859 | 17 |
| HD Dual | 89.6 | 1 | 6879 | 17 |
| SD Analog | 83.7 | 1 | 6859 | 17 |
| SD HDMI | 79.7 | 1 | 6859 | 17 |
| SD Dual | 85.8 | **2** | 6869 | 17 |

Build seeds: SD Dual seed 1 stalled the router (timeout); seed 2 passed. Every other config
passed on seed 1.

Build history:
- The first netlist was 107% LC.
- 8-bit reaction operands, shift-pair diffusion and drift, and shift-add feed splits took it to 87%.
- The per-cell seed test (subtract, abs, add, compare) sat in one clock. Split over two clocks:
  77 -> 86 MHz.
- The first rain hash had 45x horizontal neighbour correlation. It was replaced (see Engine).

Start-up: every field starts empty (vblank zone), and every line re-primes cells 0 and 1 from the
input avid edge. No free-running flag can strand the engine after configuration (the dicefall rule).

## Things to watch on hardware

- **No pattern at all.** That means a cell priming or slot bug. Raise P12 Bloom first, to rule out
  seeds simply being starved.
- **Whiteout.** At the top of Bloom some species flood. If it is too much, lower the feed boost
  (now 0.85 to 1.16 x F).
- **Boil on noisy video.** Seeds come from cell-luma steps against a threshold, so noise near it
  flips seeds. Hysteresis on the seed test would be the next step.
- **HD HDMI bars.** Glossy beads move with the picture. The conditioning is built in.
