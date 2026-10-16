# Ridgeline

Drifting sine-noise displacement with Roberts or gradient edges inked over the warped picture or on paper; edges can push.

**P12 (Rows):** noise frequency down the lines, squared, glided. 0 = pure column waves; 35% ≈ v0.1's top (4 cycles a frame); 100% = 34 cycles a frame, a 32-line ripple that shreds the picture (16-line with two octaves). The default (156) is v0.1's default

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Amount | 0–100% | displacement, squared: 0..±511 px (±766 with two octaves). The default (385) is v0.1's 75 px |
| K2 | Scale | 0–100% | noise frequency across x, squared, glided: 0..7.5 cycles across 1920 px. 50% ≈ v0.1's top; the default (190) is v0.1's default |
| K3 | Speed | 0–100% | drift speed (unchanged) |
| K4 | Gain | 0–100% | edge gain: ink alpha = edge × gain / 128 |
| K5 | Edges | Roberts / Horizontal / Vertical / Sobel | Roberts / Horizontal / Vertical / Sobel, now labelled; a zone changes only 12 counts inside it |
| K6 | Ink | 0–100% | ink luma, black (0) to white (100), continuous. Lines are a lerp toward the ink by the edge alpha |
| S7 | Ground | Picture / Paper | Picture: ink over the displaced picture / Paper: ink on plain paper, white under dark ink and black under light ink (switches at 50%, with hysteresis). On paper the lines carry the picture's colour (chroma × alpha) |
| S8 | Order | Warp-Edge / Edge-Warp | Warp-Edge: edges between the displaced line above and the input line (the ridges, v0.1's look) / Edge-Warp: the input's own edges, displaced with the picture |
| S9 | Chroma | Follow / Own | chroma follows luma / on its own noise phase (unchanged) |
| S10 | Push | Off / On | the input gradient adds to the read offset (unchanged) |
| S11 | Octaves | One / Two | one / two (unchanged) |
| P12 | Rows | 0–100% | noise frequency down the lines, squared, glided. 0 = pure column waves; 35% ≈ v0.1's top (4 cycles a frame); 100% = 34 cycles a frame, a 32-line ripple that shreds the picture (16-line with two octaves). The default (156) is v0.1's default |

## Presets

Default, White Ridges, Pushed Lines, Colour Ripple.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 24 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
