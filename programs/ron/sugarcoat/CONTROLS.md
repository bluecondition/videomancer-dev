# SUGARCOAT v2 — control cards

| Control | Name | Range | What it does |
|---|---|---|---|
| K1 | Theme | Candy Shop / Bakery / Chocolate / Frosting / Liquorice / Pastel / Gummy / Peppermint | Which candies are in the case: 8 for the brightness levels (dark to light) and 8 for the hues. |
| K2 | Sprinkles | 0..100% | Scatters sprinkles over everything. 0 = none (a few candies are always sprinkled); 100% = one in every sprinkle cell. |
| K3 | Scale | 0..100% | Continuous zoom of every candy, x1 at 0% to x16 at 100%. Anchored at the top-left corner; glides. |
| K4 | Shade | 0..100% | How much of the picture's light and dark the candy keeps. 0 = every candy in its own colours. |
| K5 | Level | −100..100% | Shifts the picture's brightness before it is sorted: left gives more of the dark candies, right more of the light ones. Centre = unchanged. |
| K6 | Gloss | 0..100% | Strength of the shading, the highlights and the embossed edges. 0 = flat cut paper; highlights appear from about 15%. |
| S7 | Lines | Off / On | Thin black line where the candy changes (2 px x 1 line at 1080p). Off = just the mattes. |
| S8 | Split | Fixed / Adaptive | Fixed: equal brightness steps. Adaptive: each level covers the same area of the picture. |
| S9 | Bank | A / B | A second helping of each theme: same idea, different candies. |
| S10 | Angle | Straight / Bias | Bias turns every candy 45 degrees. |
| S11 | Map | Chroma / Luma | Chroma: coloured regions get a candy of their own hue, grey regions one by brightness. Luma: brightness only. |
| P12 | Variety | 2..8 | How many candies are on screen: 2 to 8 brightness levels, and 2 / 4 / 8 hue groups in the Chroma map. At 8 the whole case is out: up to 16 candies. |

## Themes (dark level → light level, then violet, pink, red, orange / yellow, yellow-green, green, teal, blue)

`python3 model.py --themes out.png` draws all sixteen; the lists are at the end of `tables.py`.

| Theme | Bank A | Bank B |
|---|---|---|
| Candy Shop | chocolate bar, liquorice twists, chocolate lentils, candy corn, allsorts, candy canes, gumballs, jelly beans | milk chocolate, red twists, peanut butter cups, sour belts, starlight mints, lollipops, bullseyes, candy buttons |
| Bakery | double chocolate cookie, sandwich cookies, gingerbread, chocolate chip cookie, stroopwafel, waffle cone, iced sugar cookies, meringue | rocky road, chocolate peaks, marble fudge, lattice pie, wafer layers, shortbread, Battenberg, crinkle cookie |
| Chocolate | dark bar, truffles, milk bar, chocolate frosting, peanut butter cups, nonpareils, caramel drizzle, white bar | dark bar, fingers, rocky road, peaks, double chocolate cookie, lentils, marble fudge, cookies and cream |
| Frosting | chocolate rosettes, chocolate frosting, doughnut glaze, piped buttercream, shell border, funfetti, quilted fondant, meringue | chocolate rosettes, caramel drizzle, strawberry buttercream, rosettes, mint frosting, basketweave, sugar pearls, funfetti |
| Liquorice | capsules, liquorice twists, red twists, sandwich cookies, allsorts, Neapolitan, crinkle cookie, cookies and cream | the dark case: dark bar, capsules, truffles, twists, rocky road, double chocolate cookie, nonpareils, allsorts |
| Pastel | doughnut glaze, gummy rings, marshmallows, speckled eggs, conversation hearts, saltwater taffy, rock candy, candy buttons | bullseyes, candy sticks, marshmallows, Turkish delight, nougat, meringue, fondant stripes, cotton candy |
| Gummy | red twists, sour belts, gummy rings, gumdrops, fruit slices, jelly beans, Turkish delight, rock candy | chocolate lentils, melon slices, butterscotch, gumballs, lollipops, cherry drops, lemon drops, gumdrops |
| Peppermint | dark bar, red twists, gingerbread, candy canes, starlight mints, spearmint, peppermint bark, quilted fondant | liquorice twists, chocolate peaks, gingerbread, cinnamon hearts, candy canes, ribbon candy, peppermint bark, sugar pearls |

Notes
- K1 and P12 have 8 counts of hysteresis at every zone edge; a control resting at full scale
  still selects its last zone.
- Presets are hand-written starting points; capture real ones with `./capture_preset.sh`
  (program_id `com.czartroniks.sugarcoat`).
- v1.2.2's controls (Clean, Contrast, Outline, Shade switch, Sprinkles switch, Strength) are
  gone or moved: cleaning is built into the mattes, Shade and Sprinkles are knobs, Lines replaces
  Outline, Variety replaces Strength. K1 Theme, K3 Scale, K5 Level, K6 Gloss, S9 Bank and S11
  Map are where they were.
