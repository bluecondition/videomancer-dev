# VENEER — control cards

Two builds of one program (see SUMMARY.md): **Veneer** (`veneer.vmprog`: Ink or Texture fill)
and **Veneer Ink** (`veneer_ink.vmprog`: flat ink with three line layers). K1, K2, K4, S8 and
P12 do the same job in both.

## Veneer (Ink / Texture fill)

| Control | Name | Range | What it does |
|---|---|---|---|
| K1 | Texture | Stone / Timber / Tide / Moss / Hide / Ember / Orchid / Paper | The set: eight entries, one per luma level, each a texture recipe and two colours. Stone = veins, marble, agate, strata. Timber = wood grain, burl rings, dunes. Tide = ripples, clouds, islands. Moss = spots, camouflage, crackle. Hide = spots, tiger and zebra stripes, fur. Ember = lava, veins. Orchid = agate and zebra in magenta / teal. Paper = sepia mottle and grain. In Ink fill the set is the wash character (and the palette, K4). |
| K2 | Hue | 0..360 deg, 16 steps | Turns every colour by 22.5 deg per step. |
| K3 | Depth | 0..100% | Texture fill: contrast of the texture (value swing and relief). Ink fill: wash amount; 0 = flat mattes. |
| K4 | Colour | Source / Tint / Palette / Patchwork | Source = the region's own estimated colour (texture swings to a neighbouring hue, one step richer). Tint = the region's hue turned by the set. Palette = the set's two colours for the region's luma level. Patchwork = the set's entry picked by a mix of level, hue and saturation, so same-level regions of different colour get different textures. |
| K5 | Drift | 0..100% | Octave 2 of the noise moves 0..15 px per frame against octave 1, so the textures flow. 0 = still. |
| K6 | Relief | Flat / Soft / Embossed / Deep | Shading from the texture's slope (x0, x1/2, x1, x2). |
| S7 | Fill | Ink / Texture | Ink: region colour, black ground for the darkest level, optional wash (K3). Texture: the set's textures. |
| S8 | Thresholds | Fixed / Adaptive | Fixed: equal luma steps. Adaptive: equal-population thresholds from last frame's histogram (each level covers the same area). |
| S9 | Invert | Normal / Invert | Flips the texture value: dark lines become light, the relief flips with it. |
| S10 | Light | Left / Right | Side the relief is lit from. |
| S11 | Detail | Normal / Dense | Doubles the noise gain of every recipe: twice the bands. |
| P12 | Levels | 2..8 | Luma levels. The headline control: 2 = two big shapes, 8 = most of the picture's structure. |

## Veneer Ink (flat ink, three line layers)

| Control | Name | Range | What it does |
|---|---|---|---|
| K1 | Palette | the eight sets | Colours for K4 Tint / Palette / Patchwork (no effect in Source). |
| K2 | Hue | 0..360 deg, 16 steps | Turns every colour. |
| K3 | Sketch | 0..100% | Off in the bottom 3%. Strokes on the dark side of luma edges (difference of a 3x3 and a 5x5 blur). Low = a few strong strokes (threshold 6 codes), high = dense hatching (1.6 codes). |
| K4 | Colour | Source / Tint / Palette / Patchwork | As above, flat. |
| K5 | Contour | 0..100% | Off in the bottom 3%. Thin connected edge lines (Canny-lite); the knob is the gradient gain (a strong edge is a 128-code step at the bottom, 15 at the top). |
| K6 | Outline | Off / Thin / Medium / Bold | Ink on every region boundary: 1 sample, 2x2, 3x3 (2 / 4 / 6 px wide at 1080p). |
| S7 | Ground | Black / Colour | Black: the darkest level is a black ground. Colour: it keeps its colour. |
| S8 | Thresholds | Fixed / Adaptive | As above. |
| S9 | Ink | Black / White | Ink colour of all three layers. |
| S10 | Strokes | Dark Side / Both | Sketch strokes on the dark side of an edge only, or on both sides. |
| S11 | Paper | Colour / Blank | Blank: the fill is replaced by plain paper (white for black ink, black for white ink): a line drawing. |
| P12 | Levels | 2..8 | Luma levels. |

Notes
- The three ink layers are independent and can run together; the strongest ink wins per pixel.
- Stepped knobs and the Levels slider have 8 counts of hysteresis at every zone edge.
- Presets are hand-written starting points; capture real ones with `./capture_preset.sh`.
- Sims: labelled knobs sit on a zone edge at their default 512, so always `--set` K1, K4, K6
  (and P12) explicitly; see SUMMARY.md.
