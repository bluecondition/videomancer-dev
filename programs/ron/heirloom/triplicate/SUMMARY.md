# Triplicate — Heirloom #53

**Source**: `photos/selfie.jpg` (a paper collage, 3527×4988 scan — not synth output)

## Breakdown of the collage

- A bear with a skull for a face and a Texas Instruments home computer on its lap, cut out from a magazine.
- The cut-out is repeated four times, each copy stepped diagonally down and to the right: a red-tinted copy at the
  back, then green, then blue, and the full-colour original in front — a hand-made RGB colour separation.
- The background is a wall of grayscale skull photos tiled in a grid, slightly misaligned, like contact prints.

## Design

| Collage element | Program |
|---|---|
| The cut-out figure | the picture keyed by luma (K1; S10 picks bright or dark parts) |
| Red / green / blue copies stepped behind | the key bit and a 4-bit luma travel through a 96-deep shift register tapped at d, 2d, 3d (P12 Spread = d, 1–32 px); each copy painted in its tint with the picture's coarse shading (K5 Detail, K3 Tint saturation); S7 Order swaps red/blue, S9 Tints gives CMY |
| The original in front | the full-colour picture where its own key is on |
| Gray tiled wall of skulls | a second line buffer (1024×32) written decimated (escalator's write DDA) and read wrapped: the picture tiled K2 across, chroma killed, contrast K4 Wall; S8 Backdrop makes it the plain picture instead |
| Diagonal cascade | only horizontal offsets are possible without a frame store (a vertical step would need a field of memory), so the cascade runs sideways; K6 Wobble breathes the offset |

Video on **S11 Source** (Video / Osc): with Osc the internal triangle-wave oscillator feeds both buffers, so
the keyer, the copies and the tiled wall all stay live without an input.

Generated avid (beam counters initialised on the `g_start` edge); output conditioning (luma [1 2 1], chroma
9-tap, luma to ~777) before the blanking gate. Latency 20 clocks, all modes. Colours in the hardware
convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Key | luma threshold of the silhouette |
| K2 Tiles | backdrop tiles across, 2–8 |
| K3 Tint | copy saturation |
| K4 Wall | backdrop contrast (flat gray → full) |
| K5 Detail | copy luma depth (flat silhouettes → the picture's shading) |
| K6 Wobble | the offset breathes, bipolar rate |
| S7 Order | RGB (red furthest back) / BGR |
| S8 Backdrop | Tiled / Plain |
| S9 Tints | RGB / CMY |
| S10 Key | Bright / Dark parts are the figure |
| S11 Source | Video / Osc |
| **P12 Spread** | copy offset 1–32 px |

Resolution: offsets are real pixels (a pixel-scale feature); the tile count is a count (tiles narrow on 4:3 SD).

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 74.9 (HD Analog) / 80.0 (HD HDMI) / 83.7 (HD Dual) MHz,
  4.83k LC (63 %), 18 EBR. **Not yet flashed.** HD Analog closed with only 0.6 MHz margin — run-marginal;
  if a rebuild misses, try more seeds or router1.
