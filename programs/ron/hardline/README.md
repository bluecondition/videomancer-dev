# Hardline (started 2026-11-15)

Ten black-and-white programs in the family of Fringe, Hypnos and Cascade: one crisp geometric idea each,
played by hand. The shared control scheme, the camera rule and the rendering rules are in
[HARDLINE.md](HARDLINE.md).

Flash from `out/rev_b/ron/hardline/<name>.vmprog`; rebuild one with `./build_set.sh hardline <name>`.

Concept sketches (float renders of the idea, not RTL models): [concepts/sheet.png](concepts/sheet.png) and
[concepts/sheet_camera.png](concepts/sheet_camera.png). Regenerate with `concepts/sketch.py`.

On every program: K1 X, K2 Y, K3 Period, K6 Camera (default 100 %), S7 Glide, S10 Weight (Bold / Fine),
S11 Source (Video / Osc). On the four Shade programs S9 is Shade (Flat / Shade) and, with Shade on, S10 picks
the ramp (Bold = 4 steps, Fine = smooth).

## The slate

| # | Program | Picture | P12 headline | Status |
|---|---------|---------|--------------|--------|
| 1 | [fresnel](fresnel/) | Zone plates: rings that tighten with distance; two crossed plates beat into straight stripes that turn and tighten as X / Y moves | Pump: ring phase, rings breathe in and out | v0.2.0 built 2026-11-16 (P12 Speed: rings stream; Pair = plates opposite), router1 seed 1 all configs, HD 84.5-88.4 MHz, 83 % LC, 16 EBR; model = RTL; not flashed |
| 2 | [dazzle](dazzle/) | Dazzle camouflage: the screen cut into shards, each filled with stripes in a conflicting direction | Scatter: a pinwheel of wedges out to scattered shards | v0.1.0 built 2026-11-15, router2 seed 1 all configs, HD 78.5-82.7 MHz, 91.5 % LC, 21 EBR; model = RTL; not flashed |
| 3 | [tweed](tweed/) | Woven cloth from a real weave draft: houndstooth, shepherd's check, herringbone, pinwheel, log cabin | Turn: rotates the cloth | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 84.6-89.9 MHz, 91 % LC, 10 EBR; model = RTL; not flashed |
| 4 | [scallop](scallop/) | Seigaiha: overlapping fans of concentric arcs, like fish scales (Shade: shells) | Fan: every scale opens from a bud to a full fan | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 82.2-84.6 MHz, 91 % LC, 14 EBR; model = RTL; not flashed |
| 5 | [blaze](blaze/) | Riley's Blaze: a sunburst whose rays zigzag ring by ring (Shade: folded-paper pleats) | Twist: straight rays at centre, deeper zigzag either way | v0.2.0 built 2026-11-16 (K5 Speed: rings stream out; Skew retired), router1 seed 1 all configs, HD 77.2-81.1 MHz, 92 % LC, 23 EBR; model = RTL; not flashed |
| 6 | [multipole](multipole/) | Stripes of z^n: flowers of circles touching at the pole (magnet field lines), or hyperbola saddles; also as a curved checkerboard | Flow: lines pump out of the pole | v0.2.0 built 2026-11-16 (P12 Speed: lines flow through the pole), router1 seed 1 all configs, HD 77.3-90.7 MHz, 93 % LC, 25 EBR; model = RTL; not flashed |
| 7 | [fault](fault/) | Concentric rings sliced into strips, alternate strips slipped sideways | Slip: unbroken rings at centre, shear either way | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 78.2-81.8 MHz, 93 % LC, 22 EBR; model = RTL; not flashed |
| 8 | [flute](flute/) | Rings or stripes seen through reeded glass (Shade: glass rods) | Power: flat glass at centre, magnify one way, minify and flip the other | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 88.1-92.3 MHz, 87 % LC, 24 EBR; model = RTL; not flashed |
| 9 | [tangle](tangle/) | Truchet ribbons: concentric quarter-arcs in every tile joining into endless meandering bands (Shade: pipes) | Bend: Greek-key elbows, round arcs, straight diagonals | v0.1.0 built 2026-11-15, router2 seed 1 all configs, HD 89.0-95.6 MHz, 87 % LC, 15 EBR; model = RTL; not flashed |
| 10 | [kinegram](kinegram/) | Barrier-grid animation: interleaved frames of a ring target or wheel behind sliding black bars | Slide: the hand cranks the animation | v0.2.0 built 2026-11-16 (P12 Speed: the animation plays itself), router1 seed 1 all configs, HD 83.7-84.6 MHz, 92 % LC, 24 EBR; model = RTL; not flashed |
| 11 | [dipole](dipole/) | Bipolar net of two charges: circles through both poles crossed by circles round each; a curved checker with two eyes | Gap: a tight two-lobed flower to a nearly uniform field | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 76.0-85.9 MHz (HD Analog 2.4 % margin), 96.5 % LC (over the budget), 16 EBR; model = RTL (0 px: 720p, NTSC, Like + Fine, N at both ends, Drift both ways); K5 Drift; one Hypnos engine at half rate per pole; K3 = 10 band counts; camera = diagonal displacement (bear only a warped region); not flashed |
| 12 | [streamline](streamline/) | Potential flow past a cylinder: level bands parting round a disc, dashes running along the streamlines (Spin: Magnus) | Body: a bead to a great disc | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 77.8-84.4 MHz, 94 % LC, 23 EBR; model = RTL (0 px: 720p, NTSC, motion, 21 knob cases; the 720p / NTSC / motion renders ran one fix before the built RTL, the default was re-checked on the final RTL); K5 Drift; dash period 4P, quarter-duty cuts; spin in quarter-octave steps and not carried by the dash cuts; camera adds to psi (bear loosely recognisable); not flashed |
| 13 | [precess](precess/) | Sixteen nested ellipses, each turned a step from the last: the spiral that is not drawn, winding into lace | Twist: target at centre, spiral either way, lace at the ends | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 81.4-85.1 MHz, 93 % LC, 22 EBR; model = RTL (0 px: 720p, NTSC, motion, Glide On); K5 Drift (winds the twist); default twist +45 % on P12 so the ellipses cross; Osc at the camera's own gain; background two-bank engine (knobs land one field later), unflashed; not flashed |
| 14 | [guilloche](guilloche/) | Banknote lace: stacked bands of phase-shifted waves filled by parity, three laces overlaid | Cam: straight diamonds, sine lace, pointed arches | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs, HD 82.1-88.7 MHz, 91.6 % LC, 14 EBR; model = RTL (0 px: 720p, NTSC, Drift both ways, Lines at both ends, Glide easing); K5 Drift; edge smoothing from the nearest stripe edge; camera displacement fixed in pixels (face only faintly legible); Osc at a quarter level; not flashed |
| 15 | [kagome](kagome/) | Triaxial basket weave: white strips in three directions, truly over and under, with hexagonal eyes | Weave: a hairline star lattice to a tight basket | v0.1.0 built 2026-11-22 (wave 2), router1 seed 1 all configs (router2 misses / stalls), HD 81.6-91.7 MHz, 91 % LC, 15 EBR; model = RTL (0 px: 720p, NTSC, knobs at both ends, motion); K5 Drift (turn + glide, capped 1.4 deg / field); triple overlaps by a pinwheel rule; P12 top 0.75 pitch; camera = luma widens the strips; thin pinwheel seams may shimmer while turning; not flashed |

**Wave 2 (2026-11-22):** rows 11-15 (dipole, streamline, precess, guilloche, kagome) were planned by Fable and built overnight by Opus agents from the specs in `.hardline_work/prompts/`; shared overrides in `.setwave2_work/addendum.txt`. Unlike the first ten they move on their own: K5 is Drift on all five (Ron, 2026-11-16: motion is allowed). None flashed.

Each program's own K4, K5, S8, S9 and camera target are in its spec (`.hardline_work/prompts/<name>.txt`)
until it is built, then in its `SUMMARY.md`.

Reserves, not built: **guilloche** (banknote lace, sketch `concepts/bench_guilloche.png`), **rule**
(cellular automata; dropped by Ron 2026-11-15 in favour of kinegram), **dipole** (two-pole field lines; needs
two polar engines).

## Decisions (Ron, 2026-11-15)

- Shade is allowed on some programs, stepped or smooth, both on a switch: scallop, blaze, flute, tangle.
- kinegram replaces rule.
- K3 Period and S10 Weight are on every program but need not do rigidly the same thing.
- All ten are built in one run, without a pilot gate.
