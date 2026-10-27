# Lineage

Schmitt-trigger posterize streaks seed a cellular automaton: Wolfram rules grow down the screen in the picture's colours.

**P12 (Reach):** how long each lineage lives. At 0% every edge grows a short fringe (~24 lines). The curve is squared, and at 100% the cells never die, so the automaton takes over everything below the first edges. Glided.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Levels | 2–16 | posterize levels (continuous step size); black 64 and white ~940 are levels |
| K2 | Hysteresis | 0–100% | the Schmitt band. 0 = a plain posterize; higher, a held level only lets go once the luma has moved well past it, so edges smear into horizontal streaks; 100% freezes each line at its left-edge colour. Squared, glided |
| K3 | Burst | 0–100% | rolling bands of extra hysteresis: zones stick and release as the bands roll down (~3 s a cycle). The knob widens and deepens the bands. Glided |
| K4 | Rule | Rule 90 … Rule 254 | the elementary automaton rule: 90/18/146 Sierpinski, 60/102 one-sided triangles, 150/182/126/22 dense fractals, 54/110/124/106 gliders and particles, 30 chaos, 250/254 floods |
| K5 | Cells | 0–100% | cell size 2–128 px on the 1080-line canvas (log). Cells are square at every resolution |
| K6 | Ink | Carry / Invert / Lit / Shadow | how living cells draw: the colour of the run that seeded them / the negative of the picture under them / seed colour lifted bright / seed colour pushed dark |
| S7 | Smear | Across / Down | the Schmitt state runs along each line (vertical edges seed) / down each column (streaks fall, the tops of shapes seed) |
| S8 | Chroma | Held / Live | the colour is sampled at each jump and held for the run / the colour stays live |
| S9 | Seeds | Edges / Bands | every level jump plants a seed / every odd posterize band is solid seed |
| S10 | Grout | Off / On | dark gaps round the living cells, so the automaton reads as a tiled chart |
| S11 | Mask | Free / Lit | cells live anywhere / only over the brighter half of the picture |
| P12 | Reach | 0–100% | lineage lifetime, see above |

## Presets

Default, Sierpinski Rain, Chaos Flood, Glider Chart, Frozen Lines, Down Drips, Negative Lace.

## Technical notes

- Program type: processing; core `yuv444_30b`; latency 20 clocks in every mode.
- 18 EBR: level LUT 512×14 (rebuilt each vblank), Down-mode column state 2048×15, cell RAM 1024×32.
- Output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma scaled to white ~777 (the HD HDMI bars fix, built in from the start).
