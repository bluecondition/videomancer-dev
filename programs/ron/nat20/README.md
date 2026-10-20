# NAT20

One polyhedral die (d4, d6, d8, d10 or d20) as a small, crisp product shot on a soft gradient backdrop. It has flat-lit faces, every outline and crease exactly anti-aliased, and its numbers inked on every visible face.

v0.3.0. Rounded edges, gliding highlights, finishes and the AUTO showcase may come later (see `DESIGN.md`).

**P12 (Light):** swings the key light in an arc from low on the left, through the classic upper-left key (about a third of the way), to low on the right.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Die | d4 / d6 / d8 / d10 / d20 | Which die is shown |
| K2 | Yaw | −180–180 deg | Turns the die left/right; centre shows its top number to the viewer |
| K3 | Pitch | −180–180 deg | Tilts the die toward/away from the viewer; centre is the hero pose |
| K4 | Colour | Crimson / Midnight / Obsidian / Bone / Emerald / Amethyst / Frost / Ember / Lagoon / Bubblegum / Smoke / Sunflower | Body colour (white numerals; black on Bone, Frost, Sunflower) |
| K5 | Size | 5–22% | The die's diameter as a share of the frame height |
| P12 | Light | 0–100% | Key light position, raking-left to raking-right |

## Technical notes

- Program type: processing; core `yuv444_30b`.
- Input-to-output latency: 12 clocks (`processing_delay_clks`).
- The die is sized to the frame height (K5), so the picture is the same at SD, 720p and 1080p.
- The renderer is scanline vector: every visible edge is crossed at 16 sub-lines a line, with exact coverage across each.
- The numerals are distance-field sprites (Lato Bold) mapped onto each face.
- Built and timed at the full 74.25 MHz HD pixel clock.

## License

GPL-3.0-only. Copyright (C) 2026 czartroniks.
