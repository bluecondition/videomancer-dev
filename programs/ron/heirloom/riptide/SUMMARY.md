# Riptide — Heirloom #15

**Source photo**: `photos/WIN_20230309_00_28_35_Pro.jpg` (1920×1080 capture, 2023-03-09)

## Breakdown of the photo

- **Centre band** (y ≈ 300–740): a horizontally smeared, wildly saturated rainbow flow — red, blue, green, cyan
  blobs stretched sideways, luma flattened into a few levels, with hard tears at the band edges.
- **Above and below**: a fine dithered texture of vertical blue and red streaks on dark blue — a per-column
  speckle pattern repeating down the lines.
- **Palette**: saturated primaries in the band, blue / red speckles on navy outside.

## Design

| Photo element | Program |
|---|---|
| Smeared saturated band | inside |y − centre| < half-height (P12, K5): IIR smear (K1), chroma gain 0.5–8.5× (K2), luma posterized to 4 levels (S10) |
| Speckle streaks | outside: luma × density (K3) against a per-column LFSR hash reseeded every line (so speckles streak vertically); hue pair from K4 (S8 opposite / near); the pattern re-rolls at a rate set by K6 |
| Twin bands | S9 adds a second band half a screen away |
| Source | S11 Video / Osc |

Generated avid; output conditioning before the blanking gate. Latency 20 clocks.

## Controls

| Control | Function |
|---|---|
| K1 Smear | horizontal smear inside the band |
| K2 Saturate | chroma gain inside the band |
| K3 Density | speckle density outside |
| K4 Hue | speckle hue |
| K5 Position | band centre |
| K6 Crawl | speckle re-roll rate, bipolar |
| S7 Smear | Colour / Luma only |
| S8 Speckles | Opposite / Near hues |
| S9 Band | Single / Twin |
| S10 Poster | luma posterized Off / On |
| S11 Source | Video / Osc |
| **P12 Band** | band height: a slit → the whole screen |

## Status

- v0.1.0 (2026-10-20): built, 6/6 (SD Dual seed 2, others seed 1), HD 78.0–80.0 MHz, 4.5k LC (59 %), 15 EBR. **Not yet flashed.**
