# Lozenge — Heirloom #24

**Source photo**: `photos/WIN_20230410_01_10_37_Pro.jpg` (1920×1080 capture, 2023-04-10)

## Breakdown of the photo

- **Shape**: nested concentric diamonds (rhombi) filling the screen from the centre — a rectified-ramp field
  |x| + |y| quantized into rings, three colours cycling: black, hot pink, olive.
- **Stripes**: horizontal pink / olive stripes cut across the diamonds; where a stripe is "on" the pink and olive
  rings swap — a square-wave oscillator XOR-mixed with the ring index.
- **Highlights**: thin white / yellow lines along the upper-left side of every diamond edge (a one-sided
  differentiated key), a faint navy diamond in the centre.

## Design

| Photo element | Program |
|---|---|
| Nested diamonds | field s = a·\|x\| + b·\|y\| (K1 Aspect sets a : b, P12 sets the zoom); ring index = s >> 14, 4 bands cycling dark / A / B / A |
| Pink-olive stripes | per-line stripe phase (K2); stripe parity exchanges A and B (S7 off, S9 forces the exchange) |
| One-sided highlights | where the ring's fractional start is within K4 and the pixel is left (or right, S8) of the centre |
| Two diamonds side by side | S10 Twin: cells of half the width (mirror-folded) |
| The picture | the video (mirrored per cell) etches the field: luma × K6 added to s |

Generated avid; output conditioning before the gate. Latency 20 clocks. Colours in the hardware convention.

## Controls

| Control | Function |
|---|---|
| K1 Aspect | wide → tall diamonds |
| K2 Stripes | stripe pitch |
| K3 Palette | Pink Olive / Cyan Navy / Red Gold |
| K4 Highlight | highlight line width |
| K5 Drift | rings flow outward / inward |
| K6 Etch | video depth |
| S7 Stripes | on / off |
| S8 Side | highlight left / right |
| S9 Swap | the two colours exchanged |
| S10 Twin | one / two diamonds |
| S11 Source | Video / Osc |
| **P12 Rings** | one lozenge → dozens of fine rings |

## Status

- v0.1.0 (2026-10-20): built 6/6 seed 1, HD 74.4 (HD Dual, marginal) – 88.3 MHz, 3.65k LC (48 %), 5 EBR. **Not yet flashed.** If HD Dual misbehaves on hardware, rebuild that config with another seed.
