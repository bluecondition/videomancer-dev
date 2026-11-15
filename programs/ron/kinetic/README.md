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
| 1 | [riley](riley/) | Bridget Riley bands pinched and swollen by travelling waves | Fold: straight lines to bands folding into moiré | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 90.1-93.7 MHz; 6,560 LC); not flashed |
| 2 | [vega](vega/) | Vasarely grid of squares / circles bulging over a moving sphere | Bulge to a full globe | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 85.1-86.4 MHz; 7,366 LC); not flashed |
| 3 | [metamorph](metamorph/) | Escher tiling: a checkerboard whose edges deform into interlocking tiles | Morph | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 90.4-99.1 MHz; 7,186 LC); not flashed |
| 4 | [cafewall](cafewall/) | Café-wall illusion: offset brick rows whose straight mortar lines appear to tilt | Offset wave | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, HD Analog seed 2, HD Dual seed 3, others seed 1; HD 97.5-98.5 MHz; 6,144 LC); not flashed |
| 5 | [snakes](snakes/) | Kitaoka "rotating snakes": rings that seem to turn while still, then really crawl | Drift, illusory to real | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 76.2-77.2 MHz; 7,211 LC); not flashed |
| 6 | [louver](louver/) | Slats turning in a wave, two-sided, lit like blinds | Twist: rigid slats to helices | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, SD Analog seed 2, HD HDMI seed 2, others seed 1; HD 91.6-95.5 MHz; 7,217 LC); not flashed |
| 7 | [horologe](horologe/) | A wall of clock hands sweeping in phase patterns | Spread: chaos to synchronized figures | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 78.2-81.8 MHz; 6,978 LC); not flashed |
| 8 | [cogwork](cogwork/) | A lattice of meshed gears counter-rotating | Speed and tooth count | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 81.1-83.0 MHz; 7,174 LC); not flashed |
| 9 | [pendula](pendula/) | Dots orbiting per cell; phase gradients make them read as rolling rings | Phase gradient | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 92.1-95.7 MHz; 6,802 LC); not flashed |
| 10 | [pinwheel](pinwheel/) | Fan cells whose blade count and twist morph | Twist | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 84.3-90.8 MHz; 6,948 LC); not flashed |
| 11 | [hexbloom](hexbloom/) | A honeycomb of concentric hexagons breathing per cell | Bloom depth | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 81.6-95.2 MHz; 6,914 LC); not flashed |
| 12 | [tumbler](tumbler/) | Rhombille cubes whose shading flips in waves (reversible-cube illusion) | Flip wave | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 78.5-87.4 MHz; 6,983 LC); not flashed |
| 13 | [bauhaus](bauhaus/) | Cells of quarter-arcs, half-discs, bars and dots re-rolled by a wave | Re-roll rate | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 80.1-83.0 MHz; 7,075 LC); not flashed |
| 14 | [quasi](quasi/) | Quasicrystal: 5 to 8 plane waves summed into shimmering Penrose-like contours | Symmetry order | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 87.6-97.3 MHz; 6,980 LC); not flashed |
| 15 | [rippletank](rippletank/) | 2 to 4 orbiting point sources, circular-wave interference | Wavelength | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, HD Analog seed 2, SD Analog seed 2, others seed 1; HD 77.6-82.0 MHz; 7,081 LC); not flashed ; quadratic ponds, no hyperbolic fringes: rework candidate  |
| 16 | [isoline](isoline/) | Metaballs drawn as clean topographic bands | Merge | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 84.7-90.8 MHz; 7,361 LC); not flashed |
| 17 | [girih](girih/) | Islamic star pattern whose contact angle opens stars into rosettes | Angle | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 96.2-102.0 MHz; 6,287 LC); not flashed |
| 18 | [wallpaper](wallpaper/) | One motif pushed through symmetry groups under rotation and zoom | Symmetry sweep | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 77.1-84.6 MHz; 7,490 LC); not flashed ; sparse default, rework candidate  |
| 19 | [outrun](outrun/) | Neon perspective grid, floor and ceiling, with rolling wave terrain | Terrain height | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 80.5-83.7 MHz; 7,131 LC); not flashed |
| 20 | [slinky](slinky/) | A shape with graded echo copies along a swinging axis: an extruded tube | Extrusion length | v0.1.1 K6 camera pass built 2026-11-15 (ROUTER1, all six seed 1; HD 79.8-87.5 MHz; 7,225 LC); not flashed |
