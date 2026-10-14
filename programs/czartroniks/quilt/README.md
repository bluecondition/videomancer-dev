# Quilt

A generative patchwork quilt for Videomancer. The screen fills with a loosely
grid-based field of coloured, embossed rectangular patches. Each grid cell is
split again and again into nested sub-pieces, like log-cabin or crazy-quilt
piecing, and every piece gets a fabric colour and one of 16 procedural
textures. The quilt holds still until you move a control, or until Evolve
changes it one piece at a time.

Quilt draws its own picture. A video input is optional: it is only used by
Threshold (K4), which lets the input's brightness sculpt the piecing.

## Controls

| | Control | What it does |
|---|---|---|
| K1 | Grid Scale | Size of the base grid: Large / Big / Medium / Small |
| K2 | Seed | Picks a whole new random layout |
| K3 | Evolve | Centre: frozen. Right: random pieces change colour or texture, one at a time. Left: the change snakes from each piece to a neighbouring one. Further from centre is faster |
| K4 | Threshold | 0 = off. Turned up, the input video's brightness drives how finely each area is pieced, and areas darker than the threshold go black |
| K5 | Palette Tint | Reshuffles the colours (16 variations) |
| K6 | Texture Scale | Fabric texture size: Fine / Small / Medium / Coarse |
| S7 | Seams | Dark seam lines between pieces, on or off |
| S8 | Relief | Flat (soft emboss) or Puffed (deep emboss, with pieces shading darker toward their seams like padding) |
| S9 | Grid | Square, or Not Square (overlapping, off-grid rectangles of mixed sizes) |
| S10 | Piecing | Crazy (random split directions) or Log Cabin (alternating splits that nest into concentric frames) |
| S11 | Palette | Fabric or Rainbow colours |
| P12 | Density | The main performance control: how many times each piece is subdivided |

## Presets

Crazy Quilt, Log Cabin, Off-Grid, Solid Patches, Rainbow Crazy, Luma Mod,
Evolving, Snake.

## Technical notes

- Program type: processing (it reads input luma for Threshold), core `yuv444_30b`.
- Input-to-output latency: 28 clocks (`processing_delay_clks`).
- Every pixel runs through a fixed-depth pipeline (grid cell, then four levels
  of hash-driven splits, then colour and texture), with no per-pixel multiplies.
  Threshold uses a per-frame map of 32×32-pixel brightness averages held in
  block RAM.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
