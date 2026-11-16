# Vellum: the brief (started 2026-11-16)

The second black-and-white set after [Hardline](../hardline/HARDLINE.md). Ron, on seeing Hardline: a few were good,
a few too much like Cascade and Hypnos; "there doesn't always have to be a centered x/y and radiating outward from
that. We want clean lines, and think about layers. Layers of black, white, different shades of gray, moving
independently, transparent. More circular themes, and into other rounded rectangle, oval explorations, points,
triangles within triangles, exponential plotting, spirals of different shapes that evolve as they go out from the
center." And: "Don't be afraid to fill the screen with something simple and beautiful, it doesn't have to be busy
all the time or even have more than one layer, but be complex, but precise and be clean."

The slate and status are in [README.md](README.md); concept sketches in [concepts/](concepts/).
Flash from `out/rev_b/ron/vellum/<name>.vmprog`; rebuild one with `./build_set.sh vellum <name>`.

## What a Vellum program is

1. **Simple, beautiful, precise.** One clear figure or a few clear layers that fill the screen. The complexity is
   in the geometry (exact curves, self-similarity, number), never in clutter. The default frame is calm: few,
   large, exact elements, generous spacing, nothing busy for the sake of it. Single-layer programs are welcome.
2. **Black, white and greys.** Chroma is a constant 512. Greys are part of the palette now: stepped grey bands,
   and translucent layers whose overlaps make new tones. Every edge is anti-aliased over one pixel; detail finer
   than a pixel goes to flat mid-grey, never sparkle.
3. **It moves.** Every program drifts on its own at power-up, gently: bands stream, layers glide past each other
   at their own rates, self-similar figures zoom forever. The motion is on one bipolar Drift control with a
   still centre (Hypnos Speed, hardware-validated). The Hardline rule "nothing moves by itself" was wrong and
   does not apply here.
4. **Not always centred.** Lattices, sheets, layers and two-point figures sit beside the spirals that do grow from
   a centre.

## The control scheme

| Control | Function |
|---|---|
| K1 X | a position: the figure, the eye, the focus pair, the formation, a pan, or the offset between layers (each program names it); default centre |
| K2 Y | the same, vertically |
| K3 Scale | band period or element size, exponential |
| K4 | the program's own shape control |
| K5 Drift | bipolar speed, centre (+-24 counts) = still, exponential (Hypnos: 2^-12 .. 2^-2 cycle per field); default a gentle travel so the picture moves at power-up. Layers drift at their own related rates. |
| K6 Camera | how hard the video bends or abstracts the figure; default 100 %; 0 % = the clean figure |
| S7 Glide | Off / On, default On. On eases the knobs and the slider (and the Drift SPEED, never the accumulated phase). Off = zero easing. Nothing ever stops the motion except Drift at centre. |
| S8 | the program's own look-flip |
| S9 Tone | Grey / Ink, default Grey. Grey = the program's grey reading (stepped bands, translucent films). Ink = pure black and white (overlapping layers XOR). |
| S10 Weight | Bold / Fine: equal bands or filled shapes, or thin lines / outlines |
| S11 Source | Video / Osc (default Video) |
| P12 | the program's headline gesture; 0, 50 and 100 % are three designed pictures |

## Tone

- Ground black Y 64, white about 775 after output conditioning. Stepped greys: four levels (black, dark grey, light
  grey, white) unless a program says otherwise.
- Translucent layers ("films"): either additive light (each layer adds its own share of white over black, so
  overlaps are lighter) or "over" at a fixed opacity (each layer pulls the picture halfway to its own grey).
  Both are shifts and adds. Anti-aliasing weights a layer by its edge coverage.
- Ink: binary. Overlaps XOR (even-odd), anti-aliased as a triangle of the summed coverage.

## The camera (K6)

The video bends or abstracts the figure through the program's own primitives (a phase, a position, a radius, a
dot size). It never appears as a photograph. Box-filtered luma (at least 32 px) scaled by K6; black input = the
clean figure. A halftone reading (rosette's dot size) counts as abstraction.

## Rendering and hardware rules

As Hardline: `program_type = "processing"`, blanking gate, luma [1 2 1] then 13/16 toward black, virtual
1080-line coordinates, one latency in every mode, engines started per field, motion accumulators per field on the
guarded vsync edge, wrapping modulo exactly one period (or the self-similarity factor) so the drift never jumps.

## Chassis and engine sources

- Shell = Kinetic riley; polar = Hypnos v1.1.0 (with its Speed path for Drift).
- The ten Hardline programs are working engine sources: dazzle (half-plane DDAs, per-shard fills), tangle
  (per-pixel CORDIC distance), scallop (ROM-based distance), tweed (lattice coordinates, weave ROMs), multipole
  (log ROMs on squared distances), flute (microcoded per-field engine), fault (pre-CORDIC shift), blaze and kinegram.
