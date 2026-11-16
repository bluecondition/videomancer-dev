# Nautilus (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/nautilus.vmprog`.

## What it draws

The golden rectangle's whirling squares with their quarter arcs: the Fibonacci spiral, Le Corbusier's
Modulor, the nautilus diagrams of exponential growth drawn with a ruler and a compass. A rho:1 rectangle is
cut into a piece and a smaller rho:1 rectangle, again and again, turning 90 degrees each time; each piece
holds a quarter arc and the arcs join into one spiral. At the default (rho = phi) the pieces are squares and
the arcs quarter circles; the screen-high golden rectangle sits centred with the eye at its golden point.
Grey: the pieces are tinted by direction in four greys (W light, N mid, E dark, S black), so the whirl reads
as a pinwheel of shades, with fine light edge lines and a bold white arc. Ink: white lines on black.
The figure is infinite and self-similar about the eye (a zoom of rho with a quarter turn), so it zooms
forever; the innermost pieces, once narrower than about 6 px, fade into a small grey eye.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the eye (the spiral's pole), horizontally; 50 % = the golden point of the centred rectangle, 0 / 100 % = the screen's left / right edge (+-940 virtual px) |
| K2 | Y | the eye vertically (+-520 virtual px) |
| K3 | Scale | zoom offset about the eye, +-1.5 levels (x rho^+-1.5): the construction's size, turning as it grows |
| K4 | Detail | 1 Arc (Grey: the tints separated by fine black keylines) / 2 Squares (light edge lines, default) / 3 Chords (each square's chord: a polygonal spiral) / 4 Diagonals (plus the diagonal through the arc's centre). Hysteresis, no end dead zones |
| K5 | Drift | endless zoom into (right of centre) or out of (left) the eye; +-24 counts still; 2^-12 .. 2^-3 levels per field (Hypnos SPEED; capped at 1/8 level). Default 680: about 11 s per level at 60 Hz |
| K6 | Camera | the luma sets the weight of the arc's concentric echoes (an engraving); 0 = the clean figure; default 100 % |
| S7 | Glide | On (default): X, Y, Scale, Camera, Ratio ease 1/16 of the gap per field (never under half a count, landing exactly), the Drift speed 1/8; Off = none. +-1-count hold on every knob |
| S8 | Hand | Clockwise / Counter-clockwise (the whirl mirrored) |
| S9 | Tone | Grey (four greys, light edges, white arc; camera rings as films) / Ink (white lines on black; the camera rings XOR the drawing) |
| S10 | Weight | Bold / Fine: all line widths (arc 8 -> 2.5 virtual px); Grey Fine drops the tints to a faint wash with outlines |
| S11 | Source | Video / Osc (the shell's drifting triangle lattice, full swing) |
| P12 | Ratio | the rectangle's ratio rho: 0 % = sqrt 2 (the A-paper whirl, pieces are halves), 50 % = phi (squares, quarter circles), 100 % = 2 (1.5:1 pieces, quarter ellipses); the rectangle stays centred and screen-filling |

## Motion

One accumulator, the zoom phase in levels (Q18, wraps modulo 4 levels = a full turn), advanced once per field
by the Drift speed on the engine's per-field run (started at the zone). A label's threshold depends only on
(phase - label), so the window of 20 entries shifts by one at each integer level and the picture never jumps.
Positive Drift zooms into the eye (pieces grow out of the grey core and leave the screen turning), negative
zooms out.

## Camera

The camera is never shown. Its luma (32-px box, vertical 1/8 IIR, the riley shell's source field) sets the
half-width of concentric echoes of each piece's arc (pitch 19 real px, the two echoes next to the arc left
out so the spiral keeps a clear channel): dark areas stay the clean figure, bright areas are engraved with
arcs. Weight table: 0 below luma8 40, rising to a maximum half-width of 4.4 px; K6 is the slope
(25 %: thin, sparse hatching; 50 %: half; 100 %: bold, the subject reads as a silhouette, most clearly in Ink).
Grey: the rings darken the light tints and lighten the dark ones. Ink: rings XOR the line drawing.
Osc substitutes the drifting lattice at full swing: big diamonds of hatching.

## Engine

- Per field (from the zone start): a microcoded engine after Hardline flute (37-bit accumulator, 64 x 32
  register file, 851-word microprogram, serial 18 x 18 multiply, right shift 4 bits a clock): knob holds and
  glides, the Drift speed and phase, the P12 columns (rho, the composition reference, the eye's default,
  the narrow-side factor; 33-point tables interpolated), the eye, the zoom phase, 2^-l and rho - 1/rho, the
  DDA start values and steps, the line widths, and 25 thresholds by EXP (2^x table, interpolated) into the
  record RAMs (T, the side thresholds, A, the stretched side, A x K8, fade). About 8,700 clocks.
- Then the column map: for each column the first W / E entry hit (5 compares a column), and per line in the
  horizontal blank the first N / S entry for the next line (the line engine).
- Per pixel (48 clocks): k = min(column entry, line entry) -> record -> the piece's side distances, the
  stretched V -> 8-step CORDIC (the arc and its echoes) -> chord and diagonal (|.| x 1/sqrt 2) -> coverages
  (16 steps, AA unit = the real pixel: p = 1 / 1.5 / 1.5 / 2 / 3 virtual px at 1080p / 720p / 1080i /
  480p-576p / 480i-576i) -> tint, films, opaque edge line, arc over white (or Ink XOR) -> fade to the core
  grey -> luma [1 2 1]/4 -> 13/16 toward black (white 777) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (48) in every mode, virtual 1080-line
  coordinates (SD shows the centre 4:3). U = V = 512.

## Status

- v0.1.0 built 2026-11-16 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1. Not flashed.
- Post-route Fmax: HD Analog 77.00 MHz, HD HDMI 82.52, HD Dual 87.21 (HD needs 74.25); SD Analog 74.40,
  SD HDMI 86.48, SD Dual 80.57 (SD needs 27). Six bitstreams pass iceunpack.
- 6,859 .. 6,878 / 7,680 LC (90 %), 26 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin: 0 carry).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum nautilus`.
- Model = RTL (GHDL, 720p60, decimation 1, 918,720 px each), pixel diff 0 on: the defaults with a black input;
  Ink + Detail 4 + bear.png at Camera 100 % (the model fed the tester's own stimulus luma); Glide On at the
  power-up defaults with the default Drift, 3 fields from power-up (the drift step matches exactly; 2 or 4
  fields differ by ~20k px); Source = Osc; NTSC (720 x 243 per field), both fields. A second motion capture
  6 fields from power-up matches the model at 6 fields except 3 pixels on row 3 (the tester's 8-line vertical
  blank is shorter than the engine plus the column map, so in simulation the map is still being refilled
  during the first lines and lags one field there; on hardware both finish inside the vertical blank: about
  8,700 + 5 x (W + 8) clocks, 12,300 at 480i against a ~15,700-clock zone). Edges move outward from the eye
  between the captures, as in the model.
- Renders (work dir `.vellum_work/rb_nautilus/`): `cmp_final.png`, `sheet.png` (control sheet), `strip.png`
  (8 frames across P12), `motion.png` (Drift), `camera.png` (bear.png at Camera 25 / 50 / 100 %, Grey and Ink).
  `imodel.py` (work dir) mirrors the RTL; the per-field arithmetic is the microprogram itself, through
  `nautilus.py`'s emulator.
- Fit history: the first RTL (an FSM sequencer, 20 parallel comparators) packed to 8,565 LC standalone; the
  microcoded engine, the column map and the per-line scan brought it to 5,650 (about 6,870 built).
- Timing history: the first build missed (65 MHz): stage 23 chained a subtract, compare, mux, add and clamp;
  split over two stages (latency unchanged at 48).

## Cut / deviations

- Detail 3 is the chord, not the inscribed circle: the circle needed a second CORDIC (~700 LC) the chip did
  not have. Detail 4 is the diagonal (the spec's alternative).
- The camera engraves (arc echoes weighted by luma) instead of bending the arc radius / zoom phase: a bend
  of the few big lines read as wobble, not as the subject.
- Scale is a zoom offset about the eye; the figure is self-similar, so the knob's range is +-1.5 levels
  (4 levels would return to the same picture).
- No tint anti-aliasing: tint steps are covered by opaque edge lines (light lines, or black keylines at
  Detail 1).
- The 20-entry window leaves a small grey eye at rho = sqrt 2 (about 8-12 px); pieces narrower than 6 px fade.
- Camera rings have a fixed pitch in real pixels (19 px): 1.5x coarser relative to the picture at 720p,
  2-3x at SD. Lines are at least one real pixel wide, so SD lines are relatively thicker.
- 1080i and 480i / 576i use an AA unit of 1.5 / 3 virtual px: edges are a little soft horizontally there.
- The first field after configuration is not valid (the source width is not yet measured).
