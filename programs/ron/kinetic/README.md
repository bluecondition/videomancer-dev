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
| 1 | [riley](riley/) | Bridget Riley bands pinched and swollen by travelling waves | Fold: straight lines to bands folding into moiré | pilot v0.1.0 built 2026-11-14 (router2; HD Dual seed 2, others seed 1; HD 76.0-81.3 MHz; 6,673-6,695 LC, 22 EBR); awaiting Ron's flash |
| 2 | [vega](vega/) | Vasarely grid of squares / circles bulging over a moving sphere | Bulge to a full globe | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 74.7-78.1 MHz, HD Analog and HD Dual under 1 MHz of margin; 7,187 LC, 94 %, 22 EBR); not flashed |
| 3 | metamorph | Escher tiling: a checkerboard whose edges deform into interlocking tiles | Morph | in progress |
| 4 | [cafewall](cafewall/) | Café-wall illusion: offset brick rows whose straight mortar lines appear to tilt | Offset wave | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 87.2-97.9 MHz; 5,833 LC, 76 %, 11 EBR); not flashed |
| 5 | [snakes](snakes/) | Kitaoka "rotating snakes": rings that seem to turn while still, then really crawl | Drift, illusory to real | v0.1.0 built 2026-11-14 (router2; HD Analog + HD HDMI seed 3, HD Dual + SD HDMI seed 2, others seed 1; HD 76.4-78.4 MHz, seed-marginal; 6,940 LC, 90 %, 23 EBR); not flashed |
| 6 | [louver](louver/) | Slats turning in a wave, two-sided, lit like blinds | Twist: rigid slats to helices | pilot v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 77.8-86.5 MHz; 7,021 LC, 91 %, 24 EBR); awaiting Ron's flash |
| 7 | [horologe](horologe/) | A wall of clock hands sweeping in phase patterns | Spread: chaos to synchronized figures | v0.1.0 built 2026-11-14 (router2; HD Analog seed 2, HD HDMI seed 4, HD Dual seed 3, SD seed 1; HD 77.1-81.4 MHz, seed-marginal; 6,972 LC, 91 %, 18 EBR); not flashed |
| 8 | [cogwork](cogwork/) | A lattice of meshed gears counter-rotating | Speed and tooth count | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 86.4-89.0 MHz; 6,937 LC, 90 %, 18 EBR); not flashed |
| 9 | [pendula](pendula/) | Dots orbiting per cell; phase gradients make them read as rolling rings | Phase gradient | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 92.4-98.7 MHz; 6,564 LC, 86 %, 21 EBR); not flashed |
| 10 | [pinwheel](pinwheel/) | Fan cells whose blade count and twist morph | Twist | v0.1.0 built 2026-11-14 (router2; HD HDMI seed 4, SD Analog seed 2, others seed 1; HD 78.1-81.4 MHz; 6,616 LC, 86 %, 23 EBR); not flashed |
| 11 | [hexbloom](hexbloom/) | A honeycomb of concentric hexagons breathing per cell | Bloom depth | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 97.1-104.2 MHz; 6,604-6,622 LC, 86 %, 10 EBR); not flashed |
| 12 | [tumbler](tumbler/) | Rhombille cubes whose shading flips in waves (reversible-cube illusion) | Flip wave | v0.1.0 built 2026-11-14 (ROUTER1, all six seed 1; HD 82.1-83.0 MHz; 6,638-6,656 LC, 87 %, 16 EBR); not flashed |
| 13 | [bauhaus](bauhaus/) | Cells of quarter-arcs, half-discs, bars and dots re-rolled by a wave | Re-roll rate | v0.1.0 built 2026-11-14 (router2; HD Analog seed 4, HD HDMI seed 3 at 74.34 MHz (0.09 margin), HD Dual seed 3, SD Dual seed 2; 6,718-6,731 LC, 88 %, 23 EBR); not flashed |
| 14 | [quasi](quasi/) | Quasicrystal: 5 to 8 plane waves summed into shimmering Penrose-like contours | Symmetry order | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 80.6-88.0 MHz; 6,779-6,803 LC, 89 %, 24 EBR); not flashed |
| 15 | rippletank | 2 to 4 orbiting point sources, circular-wave interference | Wavelength | in progress |
| 16 | isoline | Metaballs drawn as clean topographic bands | Merge | in progress |
| 17 | [girih](girih/) | Islamic star pattern whose contact angle opens stars into rosettes | Angle | v0.1.0 built 2026-11-14 (router2, all six seed 1; HD 79.2-94.8 MHz; ~6,000 LC, 78 %, 15 EBR); not flashed |
| 18 | [wallpaper](wallpaper/) | One motif pushed through symmetry groups under rotation and zoom | Symmetry sweep | v0.1.0 built 2026-11-14 (router2; HD Dual seed 2, others seed 1; HD 77.4-82.2 MHz; 7,371-7,383 LC, 96 %, 20 EBR); not flashed; sparse default, rework candidate |
| 19 | outrun | Neon perspective grid, floor and ceiling, with rolling wave terrain | Terrain height | in progress |
| 20 | [slinky](slinky/) | A shape with graded echo copies along a swinging axis: an extruded tube | Extrusion length | v0.1.0 built 2026-11-14 (router2; HD Analog + HD Dual seed 2, others seed 1; HD 76.0-84.2 MHz; 6,966 LC, 91 %, 17 EBR); not flashed |
