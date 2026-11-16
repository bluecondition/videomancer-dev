# Vellum (started 2026-11-16)

Ten black, white and grey programs after [Hardline](../hardline/README.md): simple, precise, clean figures that
fill the screen, some in translucent layers, all drifting gently on their own. The scheme and rules are in
[VELLUM.md](VELLUM.md).

Flash from `out/rev_b/ron/vellum/<name>.vmprog`; rebuild one with `./build_set.sh vellum <name>`.

Concept sketches (float renders of the idea, not RTL models): [concepts/sheet.png](concepts/sheet.png).
Regenerate with `concepts/sketch.py`.

On every program: K1 X, K2 Y, K3 Scale, K5 Drift (bipolar speed, centre still), K6 Camera (default 100 %),
S7 Glide, S9 Tone (Grey / Ink), S10 Weight (Bold / Fine), S11 Source (Video / Osc).

## The slate

| # | Program | Picture | P12 headline | Status |
|---|---------|---------|--------------|--------|
| 1 | [eclipse](eclipse/) | Large translucent discs of light gliding past each other; overlaps make new greys (Rings: moire targets) | Spread: one disc, a Venn, discs apart | v0.1.0 built 2026-11-16, router1 seed 1 all configs, HD 91.6-93.9 MHz, 93 % LC, 16 EBR; model = RTL; not flashed |
| 2 | [cartouche](cartouche/) | Nested bands round a core rectangle: sharp rectangles inside, rounded rectangles outward, near-circles far out | Round: square-cornered to round | v0.1.0 built 2026-11-16, router2 seed 1 all configs, HD 77.6-87.9 MHz, 91 % LC, 6 EBR; model = RTL; not flashed |
| 3 | trine | Triangles within triangles: a triangular lattice of nested triangles, or an endless zoom into the Sierpinski gasket | Turn: every nest turns about its own centre | launched 2026-11-16 |
| 4 | [whorl](whorl/) | Nested polygons whose shape evolves going out (sharp polygon, circle, soft flower), twisting into a whirlpool | Evolve: one shape everywhere to a full metamorphosis | v0.1.0 built 2026-11-16, router1 seed 1 all configs, HD 79.4-88.3 MHz, 92 % LC, 26 EBR; model = RTL; not flashed |
| 5 | [rosette](rosette/) | Halftone dot screens at the printer's angles; translucent dots form rosettes | Angle: one clean grid, slow moire, rosette | v0.1.0 built 2026-11-16, router1 seed 1 all configs, HD 76.2-85.5 MHz, 88 % LC, 20 EBR; model = RTL; not flashed |
| 6 | decade | Log-log graph paper, an endless zoom through the decades, with plotted exponential curves | Plot: the curves' exponent | launched 2026-11-16 |
| 7 | cassini | Cassini ovals round two points: two ovals, figure eight, peanut, oval, circle (Confocal: ellipses and hyperbolas) | Spread: one point to two far foci | launched 2026-11-16 |
| 8 | phyllo | Sunflower: dots on exponential spirals, growing outward | Angle: the divergence angle, spokes to golden spiral | launched 2026-11-16 |
| 9 | nautilus | Whirling golden rectangles with quarter arcs, an endless zoom into the eye | Ratio: root-2 whirl, golden, double square | launched 2026-11-16 |
| 10 | aqueduct | Tiers of arches at three depths, flat greys sliding at parallax speeds | Depth: one flat tier to deep perspective | launched 2026-11-16 |

Each program's K4, S8, camera target and P12 are in its spec (`.vellum_work/prompts/<name>.txt`) until it is
built, then in its `SUMMARY.md`.

## Decisions (Ron, 2026-11-16)

- Not every program needs a centred X / Y radiating outward; think layers, transparency, greys, independent motion.
- Circles, rounded rectangles, ovals, points, triangles within triangles, exponential plotting, evolving spirals.
- Simple and beautiful is fine: fill the screen, one layer is fine; complex but precise and clean.
- Programs move on their own (Hypnos has movement).
