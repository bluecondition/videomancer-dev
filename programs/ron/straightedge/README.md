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
| 2 | [crease](crease/) | A striped sheet folded along straight creases: stripes kink into sharp chevrons and stream round the bends (Pleat: herringbone) | Angle: the stripes turn on the sheet, every kink opens and closes | v0.1.1 built 2026-11-17, router2 (HD Dual seed 3, rest seed 1), HD 76.5-79.3 MHz, 87 % LC, 15 EBR; model = RTL; camera = capped band weight in four flat block levels (figure stays intact; subject is not recognisable at default Scale); not flashed |
| 3 | [koch](koch/) | The Koch snowflake as nested polygonal bands; Dive: an endless zoom into its point | Depth: a plain triangle grows its generations of spikes | v0.1.0 built 2026-11-17, router1 seed 1 all configs (router2 stalls HD Analog), HD 87.0-89.1 MHz, 85 % LC, 17 EBR; model = RTL; camera = a screen of small hexagram stars sized by block luma (a layer over the figure); not flashed |
| 4 | [meander](meander/) | Greek-key friezes stacked up the screen, alternate rows running opposite ways; Slant leans them into pointed zigzags | Wind: battlements wind up into a full key | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 84.0-92.1 MHz, 90 % LC, 17 EBR; model = RTL; camera = line weight per node; not flashed |
| 5 | [tines](tines/) | Rows of long needle triangles from the top and the bottom, interleaving like backgammon points | Reach: a serrated frame, tips touching, full interleave | v0.1.1 built 2026-11-17, router1 seed 1 all configs, HD 80.3-93.3 MHz, 92 % LC, 17 EBR; model = RTL; camera = teeth fatten and lengthen per block row, capped so the zigzag always reads (weak at Reach 0 %); not flashed |
| 6 | [pentagrid](pentagrid/) | Three to eight families of crisp parallel bands; their stepped sum makes quasi-crystal stars that reshuffle | Fan: one barcode fans open into the star lattice | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 89.0-92.3 MHz, 90 % LC, 20 EBR; model = RTL; camera = four posterised band-duty levels per block (bright blocks go to thin lines on white); not flashed |
| 7 | [splinter](splinter/) | The Conway-Radin pinwheel tiling: right triangles that split into five, turning every time; an endless spiral zoom | Shatter: a few great triangles break piece by piece into splinters | v0.1.0 built 2026-11-17, router2 seed 1 all configs, HD 76.1-83.9 MHz (HD HDMI 2.4 % margin), 88 % LC, 18 EBR; model = RTL, zoom wrap verified clean in model and RTL; camera = a white splinter sized by block luma in every finest piece; not flashed |
| 8 | [lowpoly](lowpoly/) | A faceted sheet: a mesh of triangles, every facet one flat lit grey; the camera becomes a low-poly portrait | Relief: a regular lattice crumples into shards | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 82.6-84.0 MHz, 92 % LC, 23 EBR; model = RTL; camera = per-facet tone relative to the picture's mean (a low-poly portrait from about Scale 30 % down); not flashed |
| 9 | [hinge](hinge/) | Rigid squares hinged at their corners; alternate squares turn opposite ways and rhombic holes open between them | Open: a solid sheet opens to a checkerboard and closes turned | v0.1.0 built 2026-11-17, router1 seed 1 all configs (router2 misses / stalls), HD 79.7-84.1 MHz, 90 % LC, 11 EBR; model = RTL; camera = 32-px halftone of small squares inside the figure (busy at 100 %); Pinwheel squares grow up to 21 %; not flashed |
| 10 | [windrose](windrose/) | A compass rose: tiers of pointed stars, every point split light and dark, tiers turning against each other (Field: a wall of stars) | Point: fat stars to needles | v0.1.0 built 2026-11-17, router1 seed 1 all configs, HD 81.6-83.9 MHz, 92 % LC, 21 EBR; model = RTL; camera = weight of echo lines ruled round the great star; 24 points max (not 32); not flashed |
| 11 | [stitch](stitch/) | String art: one arithmetic family of straight lines whose crowding draws parabolic envelopes; regions alternate by parity | Skew: tight corner stars to long shallow envelopes | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 94.2-96.5 MHz, 75 % LC, 18 EBR; model = RTL (0 px: 720p, NTSC, motion, Glide On); pins at equal steps of line angle; P12 = Span; Cross = tangents of the rectangular hyperbola; camera = flat tone lift per 32-px block (the spec's fallback); not flashed |
| 12 | tans | The seven tangram pieces gliding and turning rigidly from one silhouette to the next | Scatter: the assembled figure to pieces spread across the screen | planned 2026-11-22 (wave 2), build running |
| 13 | [gnomon](gnomon/) | A plain of square posts from above, their long shadows swinging round with the sun and locking into bars | Sun: noon stubs to long evening shadows | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 86.7-91.5 MHz, 89 % LC, 18 EBR; model = RTL (0 px: 720p, NTSC, motion, all four sun quadrants); camera = four-step tone per post cell (per-post height did not fit; bear a silhouette); P12 top 2.0 cells; flute-style per-field engine (unflashed); tile borders not anti-aliased; not flashed |
| 14 | [brilliant](brilliant/) | A brilliant-cut gem facet diagram, every facet one flat tone, flashing as a light circles it | Cut: a wide table to a starburst of needle facets | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs (router2 stalls HD HDMI), HD 78.3-87.4 MHz, 90.5 % LC, 26 EBR; model = RTL (0 px: 720p, NTSC, motion, Glide On); camera = +-2 tone steps per facet plus a dot halftone round the stone (Pave: stone size per cell); culet star removed; knobs act one field late; not flashed |
| 15 | [hatch](hatch/) | LeWitt panels: every combination of fine lines in four directions; the camera becomes a cross-hatch | Layers: one direction per panel to the full lattice | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs (router2 thin / stalled), HD 79.0-82.0 MHz, 91 % LC, 14 EBR; model = RTL (0 px, 720p + NTSC); camera = quarter-panel cells blend toward a luma layer ladder (cross-hatch portrait); no 22.5-degree families; not flashed |

Each program's K4, S8, camera target and P12 are in its spec (`.straightedge_work/prompts/<name>.txt`) until it is
built, then in its `SUMMARY.md`.

Reserves (not built): ziggurat (stepped pyramids seen from above, apex leaning), hatch (LeWitt lines in four
directions, tone by superposition).

## How the camera came out (v0.1.x, sims only)

The brief's rule (the camera is held per element and never bends a line) was kept everywhere, but most of the
specified camera targets did not make the test image readable and were changed during the builds:

- Reads well: windrose (weight of echo lines ruled round the star), lowpoly (one flat tone per facet; a low-poly
  portrait from about Scale 30 % down).
- A layer over the figure rather than the figure itself: sliver (triangle-mosaic films), koch (a screen of small
  hexagram stars), hinge (a halftone of small squares, busy at 100 %), splinter (a white splinter in every piece).
- Figure intact but the subject barely readable at the default Scale: crease, tines, pentagrid, meander.

Judge on hardware before changing anything; K6 at 0 % is always the clean figure.

## Decisions

- Ron, 2026-11-17: same style as Vellum and Hardline, but every program non-organic: straight lines, sharp and
  pointed angles, pure geometry.
- Mine (check with Ron after flashing): the Vellum control scheme is kept unchanged; the camera is held per
  element or block so that it never curves a line (a mosaic reading instead of a bend).
