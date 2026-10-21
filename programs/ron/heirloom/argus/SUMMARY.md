# Argus — Heirloom #21

**Source photo**: `photos/WIN_20230325_11_04_55_Pro.jpg` (1920×1080 capture)

## Breakdown of the photo

- **Composition**: mirrored four-fold. A central magenta / white striped band; olive-green and cyan stripes
  bending into arrow chevrons above and below; dark eye-like knots left and right of the centre; horizontal
  tearing.
- **Mechanism**: horizontal stripes whose phase is pushed by a triangle wave of x (chevrons), a different palette
  inside a centre band.
- **Palette**: olive, green, cyan, white, magenta, violet.

## Design

| Photo element | Program |
|---|---|
| Bent stripes | phase = row·K1 (folded about the centre line with S8) + P12 × tri(x·K2) (folded about the centre column with S7) + source luma |
| Centre band | \|dy\| < K3 → palette B (magenta/white), else palette A (olive/cyan); S9 swaps |
| Eyes | diamond distance from (±K6, 0) → luma pulled toward black near the centres (S10) |
| Picture | the source luma adds to the stripe phase |

Shared plumbing: generated avid, vblank-zone field latch, virtual 1080-line geometry, Source Video/Osc switch
(oscillator muxed at the line-buffer write), output conditioning, blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Stripes | stripe pitch |
| K2 Bend | chevron pitch across |
| K3 Band | centre band height |
| K4 Hue | palette offset |
| K5 Speed | stripes roll, bipolar |
| K6 Eyes | eye spacing / size |
| S7 Mirror X | fold about the centre column |
| S8 Mirror Y | fold about the centre line |
| S9 Palettes | Normal / Swapped |
| S10 Eyes | on/off |
| S11 Source | Video / Osc |
| **P12 Bend** | chevron depth: flat stripes → arrows |

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 84.9–96.5 MHz, 3.4k LC (45 %), 8 EBR. **Not yet flashed.**
