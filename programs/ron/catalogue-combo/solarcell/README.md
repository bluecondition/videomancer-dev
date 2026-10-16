# Solarcell

Colour-averaged mosaic cells, solarized before or after averaging; staircase threshold per row, contour lines, grid, vivid.

**P12 (Fold):** **now the only threshold**: 0% folds the highlights only (threshold 748), 50% matches v0.2's default (406), 100% folds the whole picture (64)

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Level | 15–80% | **new**: fold brightness. The bands top out at 15–80% of white (v0.2 was fixed at 58%, the default) |
| K2 | Gain | 100–1600% | fold depth ×1 → ×16 (unchanged) |
| K3 | Rows | 1–64ln | cell height 1–64 lines (unchanged) |
| K4 | Cell | 4–131px | cell width 4–131 px (unchanged); cell colours are now exact means |
| K5 | Mix | 25–100% | mosaic over the picture, 25–100% (unchanged) |
| K6 | Offset | 0–100% | threshold step per cell row (unchanged) |
| S7 | Chroma | Keep / Fold | Keep / Fold (unchanged) |
| S8 | Order | Solar-Mos / Mos-Solar | Solar-Mos / Mos-Solar (unchanged) |
| S9 | Colour | Average / Vivid | **new**: Average / Vivid (cell colours at double saturation). Replaces Sample |
| S10 | Grid | Off / On | black cell borders (unchanged) |
| S11 | Iso | Off / On | contour lines, now in **negative ink**: black over light, light (768) over dark, with the complementary hue |
| P12 | Fold | 0–100% | **now the only threshold**: 0% folds the highlights only (threshold 748), 50% matches v0.2's default (406), 100% folds the whole picture (64) |

## Presets

Default, Staircase, Iso Mosaic, Fine Fold, Stained Glass.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
