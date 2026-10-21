# Cocoon — Heirloom #2

**Source photo**: `photos/WIN_20220829_23_30_37_Pro.jpg` (1920×1080 capture, 2022-08-29)

## Breakdown of the photo

- **Composition**: three vertical zones — light gray outer columns (~18 % each side), a deep blue field, and in the
  centre a horizontally stretched hexagon (flat top/bottom at ~y 150/930, pointed ends at ~x 330/1580).
- **Glow**: a radial gradient centred on the screen: red core fading through purple to deep blue, then the gray
  flanks — a soft keyed circle / vignette.
- **Weave**: the hexagon is filled with a dense chevron texture — rows of zigzags (~50 px wavelength, ~25 px row
  height) with alternating row phase, reading like woven scales, modulating between red, orange and yellow-green.
- **Video**: at the left and right flanks the source video appears posterized into a handful of colours (olive,
  blue, pink, green, gray) as blobby shapes, with heavy horizontal tearing (rows shifted sideways in bands).
- **Palette**: red/orange/yellow core, deep blue, light gray, posterized foliage colours.

## Design

| Photo element | Program |
|---|---|
| Radial red → blue → gray glow | r² = 3/8·dx² + dy² (dx² by second differences per pixel, dy² per line on the serial multiplier); palette A ROM indexed by r²·K4 |
| Hexagon window | \|dx\| + \|dy\| ≤ R and \|dy\| ≤ 9R/16; S7 swaps in an ellipse (r² ≤ Hh²) |
| Woven chevrons inside | t = tri(x·K2 + tri(y·K3) + phase); palette B indexed by 2t + L/4; S11 makes straight diagonals |
| Red/orange/yellow vs blue | palette A/B ROMs (512×32, 4 EBR); S8 Ice = chroma complement |
| Posterized video blobs | video keyed where luma ≥ K1 (threshold +192 inside the cocoon; S10 flips to dark); S9 posterizes to 8 luma × 4×4 chroma |
| Horizontal tearing | per-8-line random read offset of the line buffer, scaled by K5 |
| Cocoon size | P12: reach R = 96 + 1.4·p12 virtual px (a small core → fills the screen) |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks, all modes.
Colours in the hardware convention (u = Cr, v = Cb).

## Controls

| Control | Function |
|---|---|
| K1 Key | luma threshold for the video to show (raised inside the cocoon) |
| K2 Weave | chevron frequency across, 1–64 |
| K3 Rows | chevron row height |
| K4 Glow | glow radius: 0 = the whole screen warm, 100 = a small red core |
| K5 Tear | sideways tearing of the video in 8-line bands |
| K6 Speed | texture scroll, bipolar |
| S7 Shape | Hexagon / Ellipse |
| S8 Palette | Fire / Ice (complement hues) |
| S9 Poster | video posterized / full colour |
| S10 Source | Video / Osc (internal oscillator texture replaces the video; fully generative) |
| S11 Weave | Chevron rows / Diagonal stripes |
| **P12 Cocoon** | size: small core → fills the screen with the weave |

Resolution: all geometry in virtual 1080-line units (round at every mode); the tear offset is real pixels.

## Status

- v0.1.0 (2026-10-20): built, 6/6 seed 1, HD 77.6–81.3 MHz, 4.3k LC (56 %), 19 EBR. Ron: interesting.
- v0.2.0 (2026-10-20): S10 Key polarity replaced by Source Video/Osc (key is always Bright). Built 6/6 (SD HDMI + HD Dual seed 2): HD 75.4 / 80.3 / 80.2 MHz, 4.45k LC, 19 EBR.
