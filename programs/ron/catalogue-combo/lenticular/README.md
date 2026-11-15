# Lenticular

A lenticular 3-D postcard: parallax views of the picture behind ridged lenses. Tilt to flip, wink or shimmer.

**P12 (Tilt):** tilts the card. Each ridge shows one of the views, and the slider sweeps through them all, so near things slide against the background (3D), the picture winks to its negative (Flip), or the rainbow shimmers (Holo). Glided.

## Controls

| | Control | Range | What it does |
|---|---|---|---|
| K1 | Pitch | 0–100% | lenticule width, 4 to 48 px on the 1080-line canvas |
| K2 | Views | 2–8 | how many views are interleaved |
| K3 | Sheen | 0–100% | ridge shading: flat at 0, glossy cylinders with dark grooves at 100% |
| K4 | Depth | 0–100% | parallax: at 100% near things travel up to 48 px across the views |
| K5 | Spread | 0–100% | lens crosstalk: one view per ridge at 0, the raw interleave at 100% |
| K6 | Card | 3D / Flip / Holo / Pop | parallax views / odd views negative / holographic rainbow / odd views posterized |
| S7 | Near | Light / Dark | which tones pop forward |
| S8 | Wobble | Off / On | the card rocks by itself (a wigglegram) |
| S9 | Slant | Vertical / Diagonal | lenticule direction |
| S10 | Bands | Off / On | the viewing angle changes across the card, giving view zones |
| S11 | Ghost | Off / On | neighbouring views bleed through at the transitions |
| P12 | Tilt | 0–100% | the viewing angle, see above |

## Presets

Default, Wigglegram, Flip Card, Hologram, Pop Wink, Raw Interleave, Cereal Prize, Deep Dark.

## Technical notes

- Program type: processing; core `yuv444_30b`; latency 24 clocks in every mode.
- EBR: two picture rings 256×32 (main and neighbouring view), depth column 2048×8.
- Output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma scaled to white ~777 (the HD HDMI bars fix, built in).
- Engine notes, the catalogue cross-reference and hardware watch-items: SUMMARY.md.
