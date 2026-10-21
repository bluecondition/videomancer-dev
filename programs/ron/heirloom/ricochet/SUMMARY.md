# Ricochet — Heirloom #30

**Source photo**: `photos/WIN_20230515_23_08_59_Pro.jpg` (1920×1080 capture, 2023-05-15)

## Breakdown of the photo

- An orange ground; nested cyan / lavender chevrons point to the right, their vertex on the left.
- The chevrons are mirrored about the horizontal centre line (two systems, top and bottom), and beyond
  the last chevron the ground is flat orange — a ramp sum (|y| + k·x) quantised into bands and clipped.

## Design

| Photo element | Program |
|---|---|
| Chevrons | field = \|y − cy\|·pitch + x·angle; ring index = field bits 13:12 → cyan / lavender / orange / lavender |
| Mirror | the y term is the magnitude of a running (y − cy)·pitch (S8 Off: a monotonic ramp = diagonal bands) |
| Flat beyond the last chevron | field ≥ Reach (P12) → flat orange |
| The picture | the source luma × K6 adds to the field (etch) |
| Colour | hue rotation of the result (K3) |

Generated avid; output conditioning before the blanking gate; virtual 1080 geometry. Latency 20. 5 EBR.

## Controls

| Control | Function |
|---|---|
| K1 Pitch | ring pitch |
| K2 Angle | chevron angle |
| K3 Hue | hue rotation |
| K4 Centre | mirror line position |
| K5 Drift | rings flow, bipolar |
| K6 Etch | source luma depth |
| S7 Point | Right / Left |
| S8 Mirror | On / Off |
| S9 Palette | Cyan-lavender / Pink-lime |
| S10 Rings | Hard / Soft |
| S11 Source | Video / Osc |
| **P12 Reach** | how far the chevrons extend before the flat ground |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 3, SD HDMI seed 2), HD 74.7–80.5 MHz, 4.3k LC (56 %), 5 EBR. **Not yet flashed.**
