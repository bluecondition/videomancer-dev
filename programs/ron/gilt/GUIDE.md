# GILT v3: controls

Live video re-rendered as Klimt gold paint, swimming in oil. The picture itself becomes
gold-brown paint with its own light and shade (v0.10's ramp). Each area of the picture
is then covered in a Klimt ornament TEXTURE. Which texture an area gets depends on its
brightness (Luma map) or its colour (Chroma map), the way Sugarcoat picks candies. The
ornament lives in a warped space that the picture drags and the waves bend, so it swims
around the picture's shapes like oil.

Textures: dots, circles-in-circles, spirals, hollow squares, checker tablets, eyes,
jewel blobs, tall bars, mosaic, oil marble, chevrons, rosettes, triangles, gold flakes,
waves, and plain leaf (just the gold paint).

| control | name | what it does |
|---|---|---|
| K1 | **Theme** | Which texture goes with which brightness band and colour. 16 themes: Adele, Kiss, Tree of Life, Danae, Stoclet, Frieze, Mosaic, Oil, Spirals, Eyes, Dots, Squares, Garden, Byzantine, Jewels, All. Skin-coloured areas stay plain gold paint in every theme (Chroma map). |
| K2 | **Poster** | Posterization. 0%: 8 brightness bands (8 textures) over smooth gold paint. Up: fewer bands, with the paint flattening into fewer gold tones. 100%: 2 bands, 2 flat golds. In the Chroma map it also merges the hues (8 → 4 → 2). |
| K3 | **Scale** | Texture size: 16 / 32 / 64 / 128 px cells. |
| K4 | **Contrast** | Contrast of the picture before it is sorted into bands (centre = unchanged). Up pushes more of the picture into the darkest and brightest textures; down gathers it into the middle ones. Also sets the gold paint's contrast. |
| K5 | **Ink** | How far the ornament's tones sit from the gold paint under it. Dark parts are the paint taken down toward deep umber, light parts up toward bright gold. Low: tone-on-tone ornament in the gold. High: dark-brown and bright-gold ornament (and stronger jewels). Everything stays in the gold/brown family except the jewels. |
| K6 | **Density** | How many cells carry their motif. Low: sparse ornaments on plain gold. High: every cell drawn. Also the amount of gold flakes. |
| S7 | **Map** | Luma: textures follow brightness. Chroma: textures follow hue (green, olive, cyan, blue, red, amber, pink, violet), with grey areas falling back to brightness. |
| S8 | **Flow** | Swaps which way the picture drags the ornament: luma sideways and colour up/down, or the reverse. |
| S9 | **Invert** | Gold negative: the paint inverts (dark gold where the picture is bright) and the ornament's dark and light parts swap. |
| S10 | **Outline** | Dark contour wherever the texture changes, like cloisonné. |
| S11 | **Jewels** | Off: the jewel parts become bright gold too (pure gold and brown). On: crimson, vermilion, orange, amber, viridian, lapis, silver-blue, rose accents. |
| P12 | **Swirl** | The oil. 0%: flat ornament, still. Up: the picture drags the ornament harder (up to twice v0.10's pull), the waves and the large sway grow (to ¾ of a cell each), and everything drifts faster. 100%: fully melted. |
