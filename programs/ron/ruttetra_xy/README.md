# Rutt Etra XY

Rutt Etra with the displacement opened up to both axes. The picture is redrawn as scan lines pushed by brightness, and the Mode knob chooses how: straight down (Classic), diagonally (Slant), diagonally with rungs joining the lines into a wire mesh (Mesh), with a second family of vertical lines pushed sideways and woven over the first (Weave), or outward from the picture centre on both axes at once (Zoom). The slider raises the relief.

**P12 (Relief):** Glided depth of the luma displacement, from flat lines to deep terrain.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Scan Lines | 0–100% | Number of lines (clockwise = denser) |
| K2 | Thickness/Glow | 1–8 | Line thickness; from 3 up, a glow halo fades in around the lines |
| K3 | Line Color | White / Green / Amber / Cyan / Magenta / Video / Fire / Terrain | Line colour; Video takes the source colour, Terrain tints by brightness band |
| K4 | Contrast | 0–200% | Input contrast feeding line displacement and shading |
| K5 | Mode | Mesh < / Slant << / Slant < / Classic / Slant > / Slant >> / Mesh > / Weave / Zoom | How the lines are displaced (see below) |
| K6 | Smoothing | 0–100% | Smoothing along the scan lines (nervous to flowing) |
| S7 | Peaks | Bright / Dark | Bright: bright areas rise (stay put, dark sags); Dark: dark areas rise. In Zoom, Dark zooms the bright areas and Bright zooms the dark ones |
| S8 | Hidden Line | Off / On | Off: see-through lines; On: nearer lines hide ones behind |
| S9 | Shading | Banded / Luma | Banded: two-level shading, and nothing is drawn where the source is black (an image on a black background shows only the image). Luma: shading follows brightness, and lines are drawn everywhere, including dim ones over black |
| S10 | Roll | Run / Freeze | Run: the line grid drifts slowly; Freeze: holds it where it is |
| S11 | Input | Positive / Negative | Positive: normal input; Negative: inverted input |
| P12 | Relief | 0–100% | Relief: how far brightness pushes the lines |

### Mode

- **Classic** — the Rutt Etra picture: horizontal lines pushed straight down.
- **Slant < / > and << / >>** — every point of a line moves sideways as well as down (one or two pixels across for each row down), so the relief leans left or right. Mid-grey stays where it is; brighter and darker material lean opposite ways.
- **Mesh < / >** — a slant with rungs: at every grid node a rung drops from the line to the next one, so the terrain reads as a wire mesh.
- **Weave** — Classic plus a second set of vertical lines pushed sideways by the same brightness, drawn through the first. Each set reaches four line spacings here (eight in the other modes), so dense line settings give shallower relief.
- **Zoom** — luma zoom: every point is pushed away from the picture centre by its distance from the centre times its brightness, sideways and up / down together, so the picture is most zoomed in where it is brightest and not zoomed where it is black. The whole picture sits a little lower in this mode (un-zoomed material is pushed down by a fixed amount that grows with Relief; zoomed material moves up or down from there).

## Presets

Woven Net, Wire Mesh, Sidelong, Classic, Lean Fire, Luma Zoom.

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 16 clocks (`processing_delay_clks`).
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
