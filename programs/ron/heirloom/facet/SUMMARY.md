# Facet — Heirloom #40

**Source photo**: `photos/WIN_20230602_01_26_17_Pro.jpg` (1920×1080 capture, 2023-06-02)

## Breakdown of the photo

- **Composition**: flat-shaded geometric facets — cyan, blue, purple, green and red wedges and arcs, mirrored
  left and right, a red cone with green sides in the centre, a purple wedge above it.
- **Shading**: every region carries a smooth luma gradient (lighter toward one edge), like faceted glass.
- **Palette**: jewel tones: cyan, ultramarine, violet, emerald, crimson.

## Design

| Photo element | Program |
|---|---|
| Wedges | three linear fields L_i = cos θ_i·(x − cx) + sin θ_i·(y − cy) (phase accumulators; θ_i = tilt + i·K1), a 64-entry quarter-sine table |
| Circle / arcs | r² = dx² + dy² by second differences (dx² per pixel, dy² per line) against K2 |
| Facet colours | 4-bit code from the sign bits → 16-entry palette (4 palettes: K3 hue × S8 Jewel / Earth) |
| Shading | (\|L0\| + \|L1\|)·K4 subtracted from the facet luma; S9 Relief adds the source luma (the picture embossed into every facet) |
| Mirror | S7: after the centre column the field increments reverse sign |
| Tilt | P12 TILT: rotates all three fields (plus K5 continuous spin) |
| Split | K6 moves the centre / mirror column |

Per-frame products (cos·cx, (2cx)², r²) and per-line products (sin·dy ×3) on the serial unit; the job-select
flags are registered one clock ahead of the load mux (the first build's critical path). Source Video / Osc on S11.
Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Spread | angle between the wedge fields |
| K2 Circle | circle size |
| K3 Hue | palette rotation |
| K4 Shading | gradient depth |
| K5 Spin | continuous rotation, bipolar |
| K6 Split | mirror / centre position |
| S7 Mirror | on / off |
| S8 Palette | Jewel / Earth |
| S9 Relief | source luma shades the facets |
| S10 Circle | on / off |
| S11 Source | Video / Osc |
| **P12 Tilt** | upright → a quarter turn |

## Status

- v0.1.0 (2026-10-20): built after two timing fixes (registered job-select flags, then the angle → sin/cos table split into two clocks): 6/6 (HD Analog seed 2, others seed 1), HD 74.8–87.3 MHz, 5.06k LC (66 %), 4 EBR; HD Analog is marginal at 74.8. **Not yet flashed.**
