# Straightedge (started 2026-11-17)

Ten black, white and grey programs after [Hardline](../hardline/README.md) and [Vellum](../vellum/README.md), all
non-organic: straight lines, sharp and pointed angles, pure geometry. Nothing on the screen is curved, and the
camera never bends a line (it is read as a mosaic of each program's own elements). The scheme and rules are in
[STRAIGHTEDGE.md](STRAIGHTEDGE.md).

Flash from `out/rev_b/ron/straightedge/<name>.vmprog`; rebuild one with `./build_set.sh straightedge <name>`.

Concept sketches (float renders of the idea, not RTL models): [concepts/sheet.png](concepts/sheet.png).
Regenerate with `concepts/sketch.py`.

On every program: K1 X, K2 Y, K3 Scale, K5 Drift (bipolar speed, centre still), K6 Camera (default 100 %),
S7 Glide, S9 Tone (Grey / Ink), S10 Weight (Bold / Fine), S11 Source (Video / Osc).

## The slate

| # | Program | Picture | P12 headline | Status |
|---|---------|---------|--------------|--------|
| 1 | [sliver](sliver/) | Long translucent pointed triangles crossing the frame, each turning about its own point; overlaps make new greys | Spread: a pinwheel of blades out to scattered wedges | v0.1.0 built 2026-11-17, router2 seed 1 all configs, HD 75.7-80.3 MHz (HD Dual 1.9 % margin), 90 % LC, 21 EBR; model = RTL; camera = triangle-mosaic film steps (a layer, not edge widening); slivers wheel about the gathering point; not flashed |
| 2 | crease | A striped sheet folded along straight creases: stripes kink into sharp chevrons and stream round the bends (Pleat: herringbone) | Angle: the stripes turn on the sheet, every kink opens and closes | launched 2026-11-17 |
| 3 | [koch](koch/) | The Koch snowflake as nested polygonal bands; Dive: an endless zoom into its point | Depth: a plain triangle grows its generations of spikes | v0.1.0 built 2026-11-17, router1 seed 1 all configs (router2 stalls HD Analog), HD 87.0-89.1 MHz, 85 % LC, 17 EBR; model = RTL; camera = a screen of small hexagram stars sized by block luma (a layer over the figure); not flashed |
| 4 | [meander](meander/) | Greek-key friezes stacked up the screen, alternate rows running opposite ways; Slant leans them into pointed zigzags | Wind: battlements wind up into a full key | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 84.0-92.1 MHz, 90 % LC, 17 EBR; model = RTL; camera = line weight per node; not flashed |
| 5 | tines | Rows of long needle triangles from the top and the bottom, interleaving like backgammon points | Reach: a serrated frame, tips touching, full interleave | launched 2026-11-17 |
| 6 | [pentagrid](pentagrid/) | Three to eight families of crisp parallel bands; their stepped sum makes quasi-crystal stars that reshuffle | Fan: one barcode fans open into the star lattice | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 89.0-92.3 MHz, 90 % LC, 20 EBR; model = RTL; camera = four posterised band-duty levels per block (bright blocks go to thin lines on white); not flashed |
| 7 | splinter | The Conway-Radin pinwheel tiling: right triangles that split into five, turning every time; an endless spiral zoom | Shatter: a few great triangles break piece by piece into splinters | launched 2026-11-17 |
| 8 | lowpoly | A faceted sheet: a mesh of triangles, every facet one flat lit grey; the camera becomes a low-poly portrait | Relief: a regular lattice crumples into shards | launched 2026-11-17 |
| 9 | [hinge](hinge/) | Rigid squares hinged at their corners; alternate squares turn opposite ways and rhombic holes open between them | Open: a solid sheet opens to a checkerboard and closes turned | v0.1.0 built 2026-11-17, router1 seed 1 all configs (router2 misses / stalls), HD 79.7-84.1 MHz, 90 % LC, 11 EBR; model = RTL; camera = 32-px halftone of small squares inside the figure (busy at 100 %); Pinwheel squares grow up to 21 %; not flashed |
| 10 | windrose | A compass rose: tiers of pointed stars, every point split light and dark, tiers turning against each other (Field: a wall of stars) | Point: fat stars to needles | launched 2026-11-17 |

Each program's K4, S8, camera target and P12 are in its spec (`.straightedge_work/prompts/<name>.txt`) until it is
built, then in its `SUMMARY.md`.

Reserves (not built): ziggurat (stepped pyramids seen from above, apex leaning), hatch (LeWitt lines in four
directions, tone by superposition).

## Decisions

- Ron, 2026-11-17: same style as Vellum and Hardline, but every program non-organic: straight lines, sharp and
  pointed angles, pure geometry.
- Mine (check with Ron after flashing): the Vellum control scheme is kept unchanged; the camera is held per
  element or block so that it never curves a line (a mosaic reading instead of a bend).
