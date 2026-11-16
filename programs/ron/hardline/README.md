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
| 1 | fresnel | Zone plates: rings that tighten with distance; two crossed plates beat into straight stripes that turn and tighten as X / Y moves | Pump: ring phase, rings breathe in and out | launched 2026-11-15 |
| 2 | [dazzle](dazzle/) | Dazzle camouflage: the screen cut into shards, each filled with stripes in a conflicting direction | Scatter: a pinwheel of wedges out to scattered shards | v0.1.0 built 2026-11-15, router2 seed 1 all configs, HD 78.5-82.7 MHz, 91.5 % LC, 21 EBR; model = RTL; not flashed |
| 3 | [tweed](tweed/) | Woven cloth from a real weave draft: houndstooth, shepherd's check, herringbone, pinwheel, log cabin | Turn: rotates the cloth | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 84.6-89.9 MHz, 91 % LC, 10 EBR; model = RTL; not flashed |
| 4 | [scallop](scallop/) | Seigaiha: overlapping fans of concentric arcs, like fish scales (Shade: shells) | Fan: every scale opens from a bud to a full fan | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 82.2-84.6 MHz, 91 % LC, 14 EBR; model = RTL; not flashed |
| 5 | [blaze](blaze/) | Riley's Blaze: a sunburst whose rays zigzag ring by ring (Shade: folded-paper pleats) | Twist: straight rays at centre, deeper zigzag either way | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 76.3-81.6 MHz (HD Dual 76.25), 92 % LC, 23 EBR; model = RTL; not flashed |
| 6 | [multipole](multipole/) | Stripes of z^n: flowers of circles touching at the pole (magnet field lines), or hyperbola saddles; also as a curved checkerboard | Flow: lines pump out of the pole | v0.1.0 built 2026-11-16, router1 seed 1 all configs, HD 76.3-86.3 MHz (HD Dual 76.30), 91 % LC, 25 EBR; model = RTL; not flashed |
| 7 | [fault](fault/) | Concentric rings sliced into strips, alternate strips slipped sideways | Slip: unbroken rings at centre, shear either way | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 78.2-81.8 MHz, 93 % LC, 22 EBR; model = RTL; not flashed |
| 8 | [flute](flute/) | Rings or stripes seen through reeded glass (Shade: glass rods) | Power: flat glass at centre, magnify one way, minify and flip the other | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 88.1-92.3 MHz, 87 % LC, 24 EBR; model = RTL; not flashed |
| 9 | [tangle](tangle/) | Truchet ribbons: concentric quarter-arcs in every tile joining into endless meandering bands (Shade: pipes) | Bend: Greek-key elbows, round arcs, straight diagonals | v0.1.0 built 2026-11-15, router2 seed 1 all configs, HD 89.0-95.6 MHz, 87 % LC, 15 EBR; model = RTL; not flashed |
| 10 | [kinegram](kinegram/) | Barrier-grid animation: interleaved frames of a ring target or wheel behind sliding black bars | Slide: the hand cranks the animation | v0.1.0 built 2026-11-15, router1 seed 1 all configs, HD 80.3-87.4 MHz, 92 % LC, 24 EBR; model = RTL; not flashed |

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
