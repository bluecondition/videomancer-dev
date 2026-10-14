# Globeswap

Sphere or tunnel lens with Y/U/V read from split columns, channel swaps and negatives, and a Sabattier fold gated by region.

**P12 (Globe):** grows the lens (toward sphere) and its radius, spreads the channels, drives the fold, now up to 16× (v0.1 stopped at 8×, and P12 did nothing to the fold once K6 was above 75%). Glided

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Lens | -100–100% | lens strength, bipolar (tunnel .. flat .. sphere), glided. About 75% wraps the picture exactly onto the disc |
| K2 | Size | 3–146% | lens radius, 3–146% of the half-height (was 16–783 lines), glided. HD is within 1% of v0.1 |
| K3 | Swap | None / U<>V / Y>U / U>Y / Y<>V / Rotate / -Y / -YUV | None / U<>V / Y>U / U>Y / Y<>V / Rotate / -Y / -YUV, labelled, with hysteresis at the zone edges. Default U<>V (unchanged) |
| K4 | Offset | -256–256px | per-channel column offset, ±256 px at HD (±128 in SD), glided |
| K5 | Solar | 0–100% | **direction reversed**: 0% = off, 100% = the most solarized (threshold 960 → 386). The default (64%) is v0.1's threshold 600 |
| K6 | Fold | 0–100% | fold gain 0.125× .. 7.5×, log, over the whole knob (v0.1 was dead in its bottom 27% and top 25%) |
| S7 | Fringe | Even / Rim | Even / Rim (split grows toward the lens rim) |
| S8 | Swap | All / Solar | All / Solar (swap only inside the solar region) |
| S9 | Split | Chroma / Luma | Chroma (U and V pulled apart) / Luma (luma against both chroma) |
| S10 | Lens | Row / Disc | Row (whole width) / Disc (lens inside the disc, dry picture outside) |
| S11 | Fold | Luma / Colour | Luma / Colour (the solar region also inverts its chroma) |
| P12 | Globe | 0–100% | grows the lens (toward sphere) and its radius, spreads the channels, drives the fold, now up to 16× (v0.1 stopped at 8×, and P12 did nothing to the fold once K6 was above 75%). Glided |

## Presets

Default, Prism Globe, Solar Tunnel, Torn Channels.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 20 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
