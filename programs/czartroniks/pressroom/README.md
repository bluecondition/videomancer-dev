# Pressroom

Halftone keyed into a colour cell mosaic by luma or saturation, torn by a noisy head-switch band that can break the screen.

**P12 (Ink):** how much ink, glided: x0.29 at 0 (a light print, only the shadows dot), x1.17 at 50% (black solid, white clean), x4 at 100% (the ink floods; only the highlights stay open)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Cell | 4 px / 8 px / 16 px / 32 px | halftone cell 4 / 8 / 16 / 32 px (labelled; default 8) |
| K2 | Screen | Dot / Line / Diamond / Square | Dot / Line / Diamond / Square (labelled) |
| K3 | Band Size | 0–50% | head-switch band height: 4 lines at 0 .. half the frame at 100% (default ~5%: 56 lines HD, 15 SD) |
| K4 | Noise | 0–100% | band noise, tear and break-up; 10% floor so the band never goes quiet |
| K5 | Level | 0–100% | key threshold: luma 72..903, or saturation 8..519 |
| K6 | Soft | 0–100% | key edge dithered by noise |
| S7 | Key | Luma / Sat | Luma / Sat |
| S8 | Halftone | Inside / Outside | Inside the key / Outside it; the rest is the colour cell mosaic |
| S9 | Ink | Dark / Light | Dark: black dots on white paper. Light: white dots on black paper (both positive) |
| S10 | Band | Bottom / Rolling | Bottom / Rolling up the frame (~8.5 s a pass in every mode, wraps round) |
| S11 | Band FX | Noise / Break | Noise: noisy, torn band. Break: the whole band prints as a broken screen (thresholds scattered by Noise) |
| P12 | Ink | 0–100% | how much ink, glided: x0.29 at 0 (a light print, only the shadows dot), x1.17 at 50% (black solid, white clean), x4 at 100% (the ink floods; only the highlights stay open) |

## Presets

Default, Front Page, Cut Out, Bad Tracking.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
