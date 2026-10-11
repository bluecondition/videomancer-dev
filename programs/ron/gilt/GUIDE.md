# GILT v2: controls

Live video re-rendered as a Klimt Golden Phase painting. Faces stay photographic but
porcelain-pale. Flat, plain parts of the picture become GOLD PAINT: the image itself in
rich gold and brown (GILT v0.10's colours), keeping its light and shade. Textured or
colourful parts become ROBE cells carrying ornament. Which ornament appears comes from
that area's colour and brightness.

| control | name | what it does |
|---|---|---|
| K1 | **Figure** | How much skin stays photographic (skin-key tolerance). |
| K2 | **Ornament** | Gold paint vs ornament. Low: only the most textured or colourful areas carry ornament. High: ornament nearly everywhere. |
| K3 | **Scale** | Cell size, as a fraction of the picture width. |
| K4 | **Intricacy** | How often cells split and how many carry a motif rather than plain leaf. |
| K5 | **Jewel** | How many rings, irises and squares take enamel colour (0 = only gold, black and silver). |
| K6 | **Leaf** | The metal: silver/platinum → olive brass → warm gold → orange gold. |
| S7 | **Bank** | Kiss: warm areas → ring-discs, cool → silver tablets and bars, green → engraved gold, neutral → dark bars / bright tablets. Adele: warm → red concentric squares and big ovals, cool → eyes and spirals, green → spirals, neutral → bars, checkers, eyes. |
| S8 | **Figure Key** | Skin: any skin-coloured pixel stays photographic. Cameo: only skin inside areas whose overall colour is skin-like. |
| S9 | **Field** | The gold paint. Nebula: as is, fine sparkly dust. Leaf: 3 steps brighter, smoother glitter. |
| S10 | **Meadow** | Green areas and the low frame become a pointillist flower carpet. |
| S11 | **Relief** | Gesso emboss on the ornament (light top-left, shadow bottom-right), plus a slow, rare glint. |
| P12 | **Gild** | The performance gesture (below). |

**Gild (P12):**
- 0%: untouched video.
- 0–15%: the picture's contrast settles toward Klimt's mid-tones.
- 15–60%: cells are laid in gold one at a time, plain areas first and busy ones last.
- 60–85%: splits, motifs and jewels arrive.
- 90–100%: the flesh window tightens to the most certain skin.
