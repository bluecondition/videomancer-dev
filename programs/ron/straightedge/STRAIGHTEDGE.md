# Straightedge: the brief (started 2026-11-17)

The third black-and-white set, after [Hardline](../hardline/HARDLINE.md) and [Vellum](../vellum/VELLUM.md).
Ron, 2026-11-17: "another set of 10 programs in the same style as vellum and hardline, but in this case let's make
every program non-organic. Straight lines, sharp and pointed angles, pure geometry."

The slate and status are in [README.md](README.md); concept sketches in [concepts/](concepts/).
Flash from `out/rev_b/ron/straightedge/<name>.vmprog`; rebuild one with `./build_set.sh straightedge <name>`.

## What a Straightedge program is

1. **Ruler only.** Every edge on the screen is a straight line. No arcs, no circles, no curves, no soft fields:
   triangles, wedges, needles, stars, squares, rhombi, zigzags, right-angle frets, facets. Points are sharp;
   where two lines meet they meet exactly.
2. **Simple, beautiful, precise** (as Vellum). One clear figure or a few clear layers that fill the screen. The
   complexity is in the geometry (exact angles, self-similarity, number), never in clutter. Calm defaults.
3. **Black, white and greys.** Chroma is a constant 512. Flat stepped greys and translucent films whose overlaps
   make new tones. No gradients: a tone changes only across a straight edge. Every edge anti-aliased over one
   pixel; detail finer than about 2 px goes to flat mid-grey, never sparkle.
4. **It moves.** Every program drifts on its own at power-up (bipolar Drift, still at centre). Motion is rigid:
   things slide, turn, hinge, stream and zoom; nothing wobbles or flows.
5. **The camera never bends a line.** See below.
6. **Not always centred.** Lattices, sheets, friezes and layers sit beside the two figures that grow from a point.

## The control scheme (the Vellum scheme, unchanged)

| Control | Function |
|---|---|
| K1 X | a position: the figure, a pivot, a pan, a light, or the offset between layers (each program names it); default centre |
| K2 Y | the same, vertically |
| K3 Scale | element size or band period, exponential |
| K4 | the program's own shape control |
| K5 Drift | bipolar speed, centre (+-24 counts) = still, exponential (Hypnos: 2^-12 .. 2^-2 cycle per field); default a gentle travel so the picture moves at power-up. Layers move at their own related rates. |
| K6 Camera | how strongly the video drives the figure; default 100 %; 0 % = the clean figure |
| S7 Glide | Off / On, default On. On eases the knobs and the slider (and the Drift SPEED, never the accumulated phase). Off = zero easing. |
| S8 | the program's own look-flip |
| S9 Tone | Grey / Ink, default Grey. Grey = flat grey steps and translucent films. Ink = pure black and white (overlapping layers XOR). |
| S10 Weight | Bold / Fine: filled shapes and equal bands, or thin lines / outlines |
| S11 Source | Video / Osc (default Video) |
| P12 | the program's headline gesture; 0, 50 and 100 % are three designed pictures |

## The camera (K6): a mosaic, never a bend

In Hardline and Vellum the blurred luma was added to a phase or a coordinate, which curves every line it touches.
Here that is forbidden. The luma is box-filtered, **sampled once per element** (a lattice cell, a tooth, a facet, a
key, a square) or once per square block of a coarse screen grid (about 32 virtual px), **held constant across
that element**, and it drives one parameter of the element: its size, reach, weight, wind, split depth or flat
tone step. So the subject appears as a mosaic assembled from the program's own straight-edged primitives; every
edge stays a straight line, and the only new edges are the straight borders between elements or blocks.

Because an element starts above and to the left of where its luma is measured, the values come from the previous
field: the filtered source is written into a small block-RAM grid during the scan and read during the next field
(two banks ping-ponged per field so a read never meets a write). One field of lag is invisible.

Black input = exactly the clean figure. The subject is never shown as a photograph.

## Rendering and hardware rules

As Vellum: `program_type = "processing"`, blanking gate, luma [1 2 1] then 13/16 toward black, virtual 1080-line
coordinates (the same picture at SD, 720p and 1080p), one latency in every mode, engines started per field, motion
accumulators per field on the guarded vsync edge, wrapping modulo exactly one period (or the self-similarity
factor) so the drift never jumps.

Straight lines are cheap: an edge is the zero of an affine function a x + b y + c, which is one accumulator per
line and one add per pixel, with its coefficients set once per field. Rotations, shears, folds (reflections) and
integer similarity maps keep everything affine, so most of this set needs no per-pixel multiplier at all.

## Chassis and engine sources

- Shell = Kinetic riley (all ten).
- Hardline dazzle (affine half-plane DDAs with per-field sin / cos coefficients), tweed (lattice coordinates, ROM
  cells); Vellum trine (triangular lattice, per-cell turn, the gasket's exact zoom wrap), nautilus (a per-pixel
  cascade and a self-similar zoom that wraps), cartouche; Hardline flute (microcoded per-field engine).
