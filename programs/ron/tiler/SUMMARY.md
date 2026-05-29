# Tiler — capture-and-tile a region of incoming video

## What it does
Captures a 64x64 region of the incoming video at a position you set with
K1/K2 and tiles that captured region across the whole active screen. K3
rotates each tile around its centre; K4 selects the tile size (1..64 px).
With Luma Mod on, K5's threshold gates the tiled output against the original
video luma — bright areas show tiles, dark areas show the "under" layer
(black or the passthrough video).

## How it works
Processing program. On the write side, while the active pixel is inside the
capture window, it stores the incoming Y/U/V into a 64x64x30-bit BRAM —
about 120 kbit, nearly all of the HX4K's 128 kbit memory. On the read side,
every active pixel rotates `(active_pixel, active_line)` around the tile
centre using a 64-entry signed 8-bit sin ROM, wraps mod 64, and reads the
captured pixel back out. Rotation is a 4-stage pipeline (R0..R3) feeding a
1-cycle BRAM read; total read latency is 5 clocks, matched by the sync
delay. If routing fails, `C_TILE_BITS` can drop from 6 (64x64) to 5
(32x32).

## Controls
- **K1 Capture X** — x origin of the 64x64 capture window
- **K2 Capture Y** — y origin of the capture window
- **K3 Rotate** — tile rotation (0..360°)
- **K4 Tile Size** — output tile size (1..64 px)
- **K5 Luma Thr** — luma-mod threshold
- **T7 Luma Mod** — Off / On (gate tiles by incoming video luma)
- **T8 Under** — Black / Video (what shows under the gated tile)

(No K6, no T9/T10/T11, no slider, no presets.)

## Presets
None defined in the toml.
