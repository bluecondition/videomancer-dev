# Kinetic (started 2026-11-14)

Twenty generative programs: clean, anti-aliased geometry set in motion by one shared wave engine. Every program
has the same six knob functions (Scale, Shape, Depth, Rate, Spread, Colour), the same Wave and Field switches,
Source = Video / Osc on S11, and its own headline on the P12 slider. The brief, the wave engine and the
acceptance bar are in [KINETIC.md](KINETIC.md).

Flash from `out/rev_b/ron/kinetic/<name>.vmprog`; rebuild one with `./build_set.sh kinetic <name>`.

## Status

- **Pilots: riley and louver** built 2026-11-14 (v0.1.0), not flashed yet.
- **The other 18 were launched 2026-11-14 17:50** on the pilots' chassis, at Ron's word, without waiting for the flash.

## The slate

| # | Program | Picture | P12 headline | Status |
|---|---------|---------|--------------|--------|
| 1 | [riley](riley/) | Bridget Riley bands pinched and swollen by travelling waves | Fold: straight lines to bands folding into moiré | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 88.6-95.1 MHz; 6,579 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 2 | [vega](vega/) | Vasarely grid of squares / circles bulging over a moving sphere | Bulge to a full globe | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 82.6-89.7 MHz; 7,381 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 3 | [metamorph](metamorph/) | Escher tiling: a checkerboard whose edges deform into interlocking tiles | Morph | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 90.9-93.5 MHz; 7,214 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 4 | [cafewall](cafewall/) | Café-wall illusion: offset brick rows whose straight mortar lines appear to tilt | Offset wave | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 88.8-104.4 MHz; 6,165 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 5 | [snakes](snakes/) | Kitaoka "rotating snakes": rings that seem to turn while still, then really crawl | Drift, illusory to real | v0.1.2 built 2026-11-15 (ROUTER1, HD HDMI seed 3, others seed 1; HD 76.3-79.9 MHz; 7,209 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 6 | [louver](louver/) | Slats turning in a wave, two-sided, lit like blinds | Twist: rigid slats to helices | v0.1.2 built 2026-11-15 (ROUTER1, SD Analog seed 2, HD Dual seed 2, others seed 1; HD 83.9-85.7 MHz; 7,225 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 7 | [horologe](horologe/) | A wall of clock hands sweeping in phase patterns | Spread: chaos to synchronized figures | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 76.4-79.8 MHz; 7,014 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 8 | [cogwork](cogwork/) | A lattice of meshed gears counter-rotating | Speed and tooth count | v0.1.2 built 2026-11-15 (ROUTER1, HD HDMI seed 2, others seed 1; HD 82.3-90.6 MHz; 7,208 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 9 | [pendula](pendula/) | Dots orbiting per cell; phase gradients make them read as rolling rings | Phase gradient | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 83.1-97.0 MHz; 6,803 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 10 | [pinwheel](pinwheel/) | Fan cells whose blade count and twist morph | Twist | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 79.4-85.4 MHz; 6,961 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 11 | [hexbloom](hexbloom/) | A honeycomb of concentric hexagons breathing per cell | Bloom depth | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 95.7-101.8 MHz; 6,921 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 12 | [tumbler](tumbler/) | Rhombille cubes whose shading flips in waves (reversible-cube illusion) | Flip wave | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 76.9-84.0 MHz; 6,992 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 13 | [bauhaus](bauhaus/) | Cells of quarter-arcs, half-discs, bars and dots re-rolled by a wave | Re-roll rate | v0.1.3 2026-11-22: GHDL integer-minus-signed fix (grid origin back on the screen centre: Scale zooms about the centre, Mirrored/Radial centred; figure-box clip AA restored), router1 6/6 seed 1, HD HDMI router1 s90 (no window seed exists; spare router1 s183); 7243-7258 LC, 23 EBR; not flashed; v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 75.2-84.7 MHz; 7,080 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 14 | [quasi](quasi/) | Quasicrystal: 5 to 8 plane waves summed into shimmering Penrose-like contours | Symmetry order | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 87.3-94.2 MHz; 7,006 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 15 | [rippletank](rippletank/) | 2 to 4 orbiting point sources, circular-wave interference | Wavelength | v0.1.2 built 2026-11-15 (ROUTER1, HD HDMI seed 2, SD HDMI seed 3, others seed 1; HD 75.1-77.6 MHz; 7,084 LC); K6 camera default 100 %, camera top +25 %; not flashed ; quadratic ponds, no hyperbolic fringes: rework candidate   |
| 16 | [isoline](isoline/) | Metaballs drawn as clean topographic bands | Merge | v0.1.2 built 2026-11-15 (ROUTER1, SD HDMI seed 2, others seed 1; HD 78.9-89.3 MHz; 7,372 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 17 | [girih](girih/) | Islamic star pattern whose contact angle opens stars into rosettes | Angle | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 89.8-94.3 MHz; 6,326 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 18 | [wallpaper](wallpaper/) | One motif pushed through symmetry groups under rotation and zoom | Symmetry sweep | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 74.5-84.4 MHz; 7,522 LC); K6 camera default 100 %, camera top +25 %; not flashed ; sparse default, rework candidate   |
| 19 | [outrun](outrun/) | Neon perspective grid, floor and ceiling, with rolling wave terrain | Terrain height | v0.1.2 built 2026-11-15 (ROUTER1, all six seed 1; HD 82.1-87.2 MHz; 7,134 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 20 | [slinky](slinky/) | A shape with graded echo copies along a swinging axis: an extruded tube | Extrusion length | v0.1.2 built 2026-11-15 (ROUTER1, HD HDMI seed 2, others seed 1; HD 75.5-84.9 MHz; 7,232 LC); K6 camera default 100 %, camera top +25 %; not flashed |
| 21 | miura | Miura-ori paper fold: rows of lit parallelogram facets squeezing shut and opening in a travelling wave | Fold: a flat creased sheet to a tight razor herringbone | planned 2026-11-22 (wave 2), build running |
| 22 | [tufted](tufted/) | Diamond-tufted upholstery: soft lit pillows between buttons, inflating and dimpling in waves | Puff: a flat quilt to over-stuffed | v0.1.0 built 2026-11-22 (wave 2), router2 seed 1 all configs, HD 82.9-92.8 MHz, 6,859 LC (89 %), 22 EBR; model = RTL (0 px: 720p, NTSC, 1080i, motion, control sweeps); K5 Colour, Spread fixed; camera = per-pillow luma minus the field mean on the puff (an emboss, subject not recognisable at default Scale); not flashed |
| 23 | aperture | A wall of 8-blade camera irises opening and closing (the Institut du Monde Arabe facade) | Open: shut star seams to wide-open slivers | planned 2026-11-22 (wave 2), build running |
| 24 | braid | Three-strand plaits of round lit cord, flowing, slipping against their neighbours | Plait: straight cords to one continuous woven fabric | planned 2026-11-22 (wave 2), build running |
| 25 | gimbal | Nested rings tilting in 3-D, two-sided, crossing front over back (armillary / gyroscope) | Cascade: one flipping disc to a storm of hoops | planned 2026-11-22 (wave 2), build running |
