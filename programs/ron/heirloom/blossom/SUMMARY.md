# Blossom — Heirloom #22

**Source photo**: `photos/WIN_20230406_23_49_00_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: mirrored left/right. Nested red / green / magenta chevron petals open leftward from the
  centre line; the right side is flat magenta; hot-pink lobes at the petal tips; row tears.
- **Mechanism**: a V-shaped ring field (|x| plus the folded row position) cut into bands, with a vertical
  boundary beyond which the screen is flat.
- **Palette**: red, green, magenta, pink, dark red, violet.

## Design

| Photo element | Program |
|---|---|
| Petal rings | field = \|x − centre\| + \|y mod K2 − K2/2\| + drift (+ source luma) → × K1 → band index → 8-colour cycle (Garden / Ember, S8) rotated by K3 |
| Flat magenta | beyond a boundary column from P12 (sweeps −vw/2 … +vw/2), ragged by a per-line sawtooth (K4) |
| Pink lobes | S10: band 0 near the petal tip rows |
| Solid shading | S9: luma ramp in magenta instead of the colour cycle |
| Mirror | S7: folded / one-sided ramp |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Pitch | ring density |
| K2 Period | petal period down the screen |
| K3 Hue | palette rotation |
| K4 Ragged | boundary raggedness |
| K5 Drift | rings flow in/out |
| K6 Centre | petal centre, left → right |
| S7 Mirror | on/off |
| S8 Palette | Garden / Ember |
| S9 Rings | Colour / Solid |
| S10 Lobes | on/off |
| S11 Source | Video / Osc |
| **P12 Open** | the boundary: all magenta → petals fill the screen |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 86.7–91.5 MHz, 3.4k LC (44 %), 5 EBR. **Not yet flashed.**
