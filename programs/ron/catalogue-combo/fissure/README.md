# Fissure

Kintsugi cracks grow from the picture's edges: gold fractures kink, fork and join down the screen.

**P12 (Reach):** how far the cracks run. At 0% they are short splinters hanging off the picture's edges. As the slider rises they run further, fork and join. At 100% they never die: the picture is shattered from the first edges to the bottom of the screen. Glided.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Spawn | 0–100% | how readily the picture's edges start cracks (lower edge threshold, more seeds, closer together) |
| K2 | Branch | 0–100% | fork rate: single fissures at 0, bushy deltas at 100% |
| K3 | Wander | 0–100% | how often a crack kinks and how steeply it can run: straight fractures at 0, lightning at 100% |
| K4 | Seam | 0–100% | gold width, hairline to bold (1–12 px each side on the 1080-line canvas) |
| K5 | Ground | 0–100% | the picture between the veins: full at 0, black at 100%. On black, the gold veins alone draw the picture, lit by it |
| K6 | Metal | Gold / Silver / Copper / Prism | the seam metal; Prism gives every crack its own hue |
| S7 | Seek | Dark / Light | at each kink, a crack turns toward the darker (or lighter) side of the picture, so the network gathers along its shadows (or highlights) |
| S8 | Seeds | Edges / Scatter | cracks start at the picture's edges / in a rain over its lit areas |
| S9 | Mask | Free / Lit | cracks run anywhere / stop where the picture goes dark |
| S10 | Shadow | Off / On | a dark inset groove round every seam |
| S11 | Glint | Off / On | a highlight sweeping across the gold |
| P12 | Reach | 0–100% | crack lifetime, see above |

## Presets

Default, Gold Drawing, Shattered, Silver Roots, Copper Veins, Prism Hyphae, Molten, Lightning.

## Technical notes

- Program type: processing; core `yuv444_30b`; latency 32 clocks in every mode.
- EBR (20): crack-tip list 512×64 (two banks by line parity), column state 2048×16, picture ring 256×32, metal ROM 256×32.
- Output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma scaled to white ~777 (the HD HDMI bars fix, built in).
- Engine notes, the catalogue cross-reference and hardware watch-items: SUMMARY.md.
