# Lineage — summary

Catalogue combo, 2026-10-20. Ron asked for a new dual-effect program built only from
EFFECTS_CATALOG.md entries that no catalogue combo had used. v0.1.0 is built, not flashed.

## How the two effects were chosen

The 45 kept combos (README.md in the parent folder) cover all of Color/Tonal and Stylization,
10 of 13 Spatial, 5 of 7 Analog, 2 of 7 Generative and 3 of 4 Compositional. Never used:

| Catalogue entry | Why it was never used |
|---|---|
| Temporal (all 9), Feedback/Recursive (all 5), slit-scan, feedback zoom | excluded from every draw: they need a frame store (128 kbit of EBR; a coarse store is the rejected field-overlay genre) |
| Phosphor persistence | drawn twice (#119, #139), skipped for the same reason |
| Reaction-diffusion, particles, kintsugi, mycelium | excluded from the draw (2-D state / frame store; kintsugi and mycelium exist as LZX programs) |
| **Cellular automata seeded by video** | excluded from the draw, because a 2-D CA needs a frame store. **A 1-D CA whose generations run down the scanlines needs only one line of cells** |
| **Hysteresis-edge state glitches** | added to the catalogue 2026-10-15, after every batch |
| Tile & shuffle | tried in jigsaw, tangram, shuffle and tileflurry; all removed in the 2026-09-12 cleanup |
| Scanlines / aperture grille / barrel | tried in shadowmask, aperture, crtmosh and tubecurve; all removed |
| Masking by luma/chroma/alpha | the dedicated mask combos (matte, tapemask, squall, scratchboard) were removed; keyed masking survives only through the key entry (stencil, pressroom, hatchglass) |

Lineage takes the two entries that had never been built at all and are possible without a
frame store. Both redraw the picture as structure (state machines along and down the
raster), not as a layer over it.

## Engine

- **Schmitt posterize.** A 512-entry LUT (built in vblank) maps each luma code to its
  level {index, value}. Across mode keeps one loop state per line. A pixel reloads the
  state when its luma leaves [held − T, held + T], with T = step/2 + K2 band + this line's
  burst band. Down mode keeps the state per column in a 2048-entry RAM, read-modify-written
  once per pixel. A "jump" is a reload to a different level.
- **Automaton.** Cells are square on the 1080-line canvas (virtual-coordinate DDAs from
  the timing ID). One 1024×32 RAM {life 11, y8, u6, v6, parity} serves every line from a
  single bank: cell c+1 is read at the start of cell c, and cell c is written after it ends.
  - The first line of each cell row is a generation line. New state = rule(l, c, r), which
    needs a living parent. Life = the strongest parent's life minus the cell height, and the
    colour comes from that parent.
  - Seeds are written on every line (life = reach, colour of the held run at the cell's
    end), so 1-line horizontal edges in Down mode seed too.
  - The first line of each field reads all-dead, so nothing carries between frames: the
    picture is recomputed from the video every frame.
- Stored colours (Down-held chroma, cell colours) keep their row's hsync→avid parity and
  swap U/V where it differs from the current row (drape recipe).
- Every per-field event keys off a vblank zone (no avid for 1024 clocks), not vsync.

## Status

v0.1.0 (2026-10-20): built and packed (`out/rev_b/ron/catalogue-combo/lineage.vmprog`), timing-closed, not flashed.

| Config | Fmax (MHz) | Seed | LC | EBR |
|---|---|---|---|---|
| HD Analog | 93.1 | 1 | 5153 (67%) | 19 |
| HD HDMI | 86.7 | 1 | 5154 | 19 |
| HD Dual | 91.9 | 1 | 5164 | 19 |
| SD Analog | 84.0 | 1 | 5158 | 19 |
| SD HDMI | 90.3 | 1 | 5151 | 19 |
| SD Dual | 84.0 | **2** | 5167 | 19 |

Build seeds: SD Dual passed on seed 2, every other config on seed 1. The 19th EBR is the
16-entry level table, which yosys moved into a RAM. It is written only in vblank, so it is
safe, but it is the only RAM without no_rw_check (a few LC of transparency emulation).
The three big RAMs show "don't care on collision".

## Things to watch on hardware

- HD HDMI bars: the conditioning is built in (luma [1 2 1], chroma 9-tap, luma × 13/16). If
  bars appear, widen the chroma filter further before touching luma (wrinkle lesson).
- Flicker: chaotic rules (30, 110) amplify any seed that flickers, so a noisy source near a
  level threshold can boil. K2 Hysteresis is the cure by design. If it still boils at
  moderate K2, a minimum run length before a jump seeds (etchplate's qualification) is the
  next step.
- Left/right edges: seeds are suppressed within 24 px of each end, and the Schmitt state
  reloads in the outer 16 px. If a vertical CA strip appears at an edge, the source's
  blanking columns are wider than that.
