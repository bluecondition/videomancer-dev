# Morphogen

Reaction-diffusion grows out of the picture's edges, redrawing it as glossy beads, strands and coats on a cel ground.

**P12 (Bloom):** how much grows. At 0% sparse beads sprout only on the picture's strongest edges. As the slider rises, more edges seed and the reaction is fed harder. Near the top, with Rain on, the whole picture is encrusted in beads. Glided.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Species | Beads, Seeds, Pearls, Streaks, Strands, Arches, Fronds, Waves | the Gray-Scott regime: round beads, sparse seeds, long strands, branching arches, oscillating waves |
| K2 | Grain | 0–100% | reaction cell size, 4 to 16 px on the 1080-line canvas (the size of the beads) |
| K3 | Drift | −100..100% | the pattern leans left, grows straight down at the centre, or leans right |
| K4 | Bands | 2–9 | cel luma bands in the ground |
| K5 | Ink | 0–100% | outline weight on the cel band edges and the bead rims (0 = none) |
| K6 | Skin | Pearl / Toon / Coat / Ink | glossy beads in the picture's colours on a dim cel ground / flat lifted beads with ink rims on the cel picture / dark spots on a tawny cel coat / black blots on paper |
| S7 | Drive | Light / Dark | the picture's light, or its shadows, feeds the reaction |
| S8 | Rain | Off / On | seeds also fall over the whole frame (density follows Bloom) |
| S9 | Flip | Off / On | swaps beads and ground: the pattern becomes a net around holes |
| S10 | Shine | Off / On | specular highlights on the beads |
| S11 | Ground | Picture / Black | the cel picture behind the pattern, or black (the pattern alone; Ink becomes chalk) |
| P12 | Bloom | 0–100% | growth, see above |

## Presets

Default, Pearl Rain, Leopard, Ink Blots, Toon Drips, Chalk, Pearl Net, Waves.

## Technical notes

- Program type: processing; core `yuv444_30b`; latency 24 clocks in every mode.
- EBR (17): cell state 1024×24, cell luma / previous v 1024×16, sub-step hand-off 256×32, picture ring 256×32, cel column 2048×6.
- Output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma scaled to white ~777 (the HD HDMI bars fix, built in).
- Engine notes, the catalogue cross-reference and hardware watch-items: SUMMARY.md.
