# APPLIQUE — control cards

| Control | Name | Range | What it does |
|---|---|---|---|
| K1 | Pattern | Quilt / Weave / Stripe / Check / Dots / Tile / Deco / Hatch | The set: eight entries, one per luma level, each a pattern and two colours (list below). |
| K2 | Hue | 0..360 deg, 16 steps | Turns every colour by 22.5 deg per step. |
| K3 | Depth | 0..100% | Contrast of the two tones and of the embossed edge. 0 = flat value (the two colours still alternate). |
| K4 | Colour | Source / Tint / Palette / Patchwork | Source = the region's own estimated colour (second tone: a neighbouring hue, one step richer). Tint = the region's hue turned by the set. Palette = the set's two colours for the region's luma level. Patchwork = the set's entry picked by a mix of level, hue and saturation, so same-level regions of different colour get different cloth. |
| K5 | Scale | 0..100% | Continuous zoom of every pattern, x1 at 0% to x16 at 100% (cells 16–128 px at 0%, 43–340 px at the default 37%). Anchored at the top-left corner; glides. |
| K6 | Drift | 0..100% | The cloth slides (mostly sideways). 0 = still; 100% = one small cell every 8 frames. |
| S7 | Seams | Off / On | Thin black line where the fill changes (2 px x 1 line at 1080p). |
| S8 | Thresholds | Fixed / Adaptive | Fixed: equal luma steps. Adaptive: equal-population thresholds from last frame's histogram (each level covers the same area). |
| S9 | Invert | Normal / Invert | Swaps the two tones of every pattern (and with them the colours and the lit side). |
| S10 | Relief | Flat / Embossed | Flat: printed cloth. Embossed: every edge gets a lit and a shaded side (light from the upper left). |
| S11 | Angle | Straight / Bias | Bias turns every pattern 45 degrees (stripes become diagonals, checks become diamonds; the sets' own bias patterns go straight). |
| P12 | Levels | 2..8 | Luma levels. The headline control: 2 = two big pieces of cloth, 8 = most of the picture's structure. |

## Sets (dark level → light level)

| Set | Entries |
|---|---|
| Quilt | pinwheel triangles, log-cabin squares, hourglass, checkerboard, harlequin, flying geese, basketweave, bias stripe |
| Weave | twill, herringbone, basket, chevron, big basket, rounded ribs, canvas, fine ribs (indigo, rust, olive, linen) |
| Stripe | bold, pinstripe, bias, regimental, fine horizontal, bias the other way, wide-set pin, fine vertical |
| Check | tartan (navy / green), houndstooth, red gingham, tartan (red / yellow), diamond check, windowpane, blue gingham, fine check |
| Dots | big dots, rings, polka dots, bullseyes, pin dots, domes, dark dots large and small |
| Tile | brick, fish scales, snowball octagons, lattice, square tiles, nested diamonds, scallop outlines, shaded bricks |
| Deco | chevron (black / gold), nested diamonds, fans, argyle, pyramids, shaded chevron, triangles, nested squares |
| Hatch | sepia print screens, dense to sparse: lattice, crosshatch, bias hatch, bias rule, halftone screen, dots, horizontal rule, canvas |

Notes
- Stepped knobs and the Levels slider have 8 counts of hysteresis at every zone edge.
- Presets are hand-written starting points; capture real ones with `./capture_preset.sh`.
- Sims: labelled knobs sit on a zone edge at their default 512, so always `--set` K1, K4 and
  P12 explicitly, and use a copy with `C_SIM_FAST = true` (see SUMMARY.md).
