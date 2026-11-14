# Sugarcoat

Video re-made as candy. The picture is flattened into clean regions by brightness and hue, and every region is filled from a case of about a hundred drawn confections: chocolate bars with bevelled segments, chocolate-chip cookies, piped frosting, doughnut glaze with sprinkles, candy canes, starlight mints, gumballs, jelly beans, gummy rings, fruit slices, allsorts, caramels and more. Pieces are scattered with their own colour, size and turn, shaded as domes and sticks with a highlight, and sit on the surface with a lit rim and a shadow. Nothing moves on its own: the picture alone decides where each candy goes.

**P12 (Variety):** How many candies are on screen at once: 2 to 8 brightness levels and, in the Chroma map, 2 / 4 / 8 hue groups. At the top the whole case is out, up to sixteen candies.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Theme | Candy Shop / Bakery / Chocolate / Frosting / Liquorice / Pastel / Gummy / Peppermint | Which candies are in the case |
| K2 | Sprinkles | 0–100% | Scatters sprinkles over everything |
| K3 | Scale | 0–100% | Size of every candy, x1 to x16 |
| K4 | Shade | 0–100% | How much of the picture's light and dark the candy keeps |
| K5 | Level | −100–100% | More dark candy or more light candy; centre = unchanged |
| K6 | Gloss | 0–100% | Shading, highlights and embossed edges; 0 = flat cut paper |
| S7 | Lines | Off / On | Thin black line where the candy changes |
| S8 | Split | Fixed / Adaptive | Equal brightness steps, or each level covering the same area |
| S9 | Bank | A / B | A second helping of each theme |
| S10 | Angle | Straight / Bias | Turns every candy 45 degrees |
| S11 | Map | Chroma / Luma | Candies by colour and brightness, or by brightness only |
| P12 | Variety | 2–8 | How many candies are on screen |

## Presets

Candy Shop, Bakery Case, Chocolate Box, Birthday Cake, Gummy Bias, Cut Paper.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 84 clocks (`processing_delay_clks`), the same in every mode.
- Built and timed at the full 74.25 MHz HD pixel clock.
- Candy sizes follow the screen height at every resolution; region shapes and lines are coarser at 720p and SD.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
