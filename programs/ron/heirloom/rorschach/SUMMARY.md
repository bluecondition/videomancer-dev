# Rorschach — Heirloom #4

**Source photo**: `photos/WIN_20221124_22_54_05_Pro.jpg` (1920×1080 capture, 2022-11-24)

## Breakdown of the photo

- **Composition**: a picture posterized into a handful of pastels — cream, pale pink, ultramarine, red, magenta,
  with cyan and green slivers — broken into horizontal rows that are torn sideways by random amounts.
- **Symmetry**: the whole frame is mirrored about the horizontal centre line (top and bottom are reflections).
- **Backdrop**: behind the posterized shapes run vertical wavy cream / pink stripes (a wobbled vertical bar
  oscillator), which show wherever the key is off.
- **Palette**: pastel cream and pink dominate; saturated blue and red islands; glitch slivers.

## Design

| Photo element | Program |
|---|---|
| Posterized pastels | 16-colour palette indexed by luma band (3) × Cr/Cb quadrant; K5 scales its saturation; S9 swaps a bold set |
| Mirror | LEFT / RIGHT about an axis (K6) via up/down line-buffer read pointers — a vertical mirror needs a frame store (warned in the plan) |
| Row tearing | per-8-line random read offset scaled by P12, applied to both mirror halves |
| Wavy stripes | vertical stripes (K2 pitch) wobbled by a triangle of y (K3 depth, K4 rate), cream / pink, where the luma key (K1, S10 polarity) is off; S8 flattens them to cream |
| Source | S11 Video / Osc (oscillator texture into the same line buffer) |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks. Colours in the hardware
convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Key | luma threshold for the posterized picture to show |
| K2 Stripes | backdrop stripe pitch |
| K3 Wave | stripe wobble depth |
| K4 Rate | stripe wobble vertical rate |
| K5 Tint | palette saturation 0–200 % |
| K6 Axis | mirror axis position |
| S7 Mirror | Off / On |
| S8 Stripes | Off (flat cream) / On |
| S9 Palette | Pastel / Bold |
| S10 Key | Bright / Dark parts show |
| S11 Source | Video / Osc |
| **P12 Tear** | 0 = clean mirror → shredded in 8-line bands |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (HD HDMI seed 3, others seed 1), HD 78.1–78.2 MHz, 4.1k LC (54 %), 15 EBR. **Not yet flashed.**
