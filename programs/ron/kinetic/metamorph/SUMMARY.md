# Metamorph (Kinetic)

**v0.1.0 built 2026-11-14, not flashed.** Flash `out/rev_b/ron/kinetic/metamorph.vmprog`.

## What it draws

M. C. Escher's "Metamorphosis II", "Sky and Water I" and "Day and Night": a checkerboard whose edges deform by
degrees until the squares are interlocking figures. The default frame (Ink palette) is a Metamorphosis strip:
plain squares in a band at the left of centre, their edges bending more and more toward the right until the
tiles are hooked, bird-like creatures that lock into each other, each with an inner echo contour and an eye.
The band of squares travels across the picture (about 7.5 s per cycle at the default Rate), so every tile is
squared and released in turn.

## Element and state

Element: a tile. State: the morph amount A, a continuous field that the wave carries across the picture.

    X' = X + A f(Y)        Y' = Y - A f(X)        colour = parity(floor X') xor parity(floor Y')
    A  = Morph + Depth * wave(theta) + source      (signed; a negative A turns the tiles the other way)
    theta = Spread * X + phase                     Field = Linear
          = Spread * (|X| + |Y|) + phase - 1/4     Field = Radial (diamond rings; the centre starts as squares)

X, Y are in tiles from the screen centre; f is the edge profile, one tile long, so the tiles always
tessellate whatever A does. Using -f for the second family makes the two edge families turn each tile like a
pinwheel instead of shearing it, which is what makes the figures interlock. A = 0 is a checkerboard.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | tile pitch, 64 .. 430 virtual px, exponential, glided (default 166 px: 11.5 tiles across) |
| K2 | Shape | the edge profile, morphing in eighths through four ROM profiles: S-curve > bird hook (default) > zigzag (pinwheel crosses) > jigsaw knob (stars) |
| K3 | Depth | the wave's swing of A, square law, 0 .. 1/2 tile (default 0.17) |
| K4 | Rate | wave travel, bipolar, centre = still; exponential, 60 s per cycle just off the deadband to 2 cycles per second |
| K5 | Spread | phase step per tile, square law, 0 .. 1/2 cycle. 0 = every tile morphs in unison; default = one metamorphosis across the screen; top = neighbours in antiphase |
| K6 | Colour | the 16 shared Kinetic palettes (ground + A; the accent on the outline), with hysteresis |
| S7 | Wave | Smooth (sine) / Snap (2.5 x the sine, clipped: zones of squares and zones of creatures with quick eased flips, about 74 % dwell) |
| S8 | Field | Linear (the metamorphosis sweeps across) / Radial (diamond rings from the screen centre) |
| S9 | Outline | Off / On: an accent-colour line on every tile edge (4 virtual px at 1080p, 2 real px at 720p and SD) |
| S10 | Detail | Off / On (default On): an echo contour and an eye inside every tile, drawn in the other tile colour, in the tile's own deformed coordinates so they bend with it |
| S11 | Source | Video / Osc |
| P12 | MORPH | the headline: base A, 0 .. 0.44 tile. 0 % = a checkerboard (the wave only rocks the edges a little both ways), 34 % default, 100 % = deep interlock: every tile a hooked figure, the wave pushing them to the edge of folding |

## Video

`program_type = "processing"`. The picture's luma is the second modulator and adds to the same state A:
luma -> 32-pixel box -> vertical IIR in a line store (riley's source field, but with a 12-bit state moving
1/32 per line instead of 8 bits and 1/8) -> square law
(320 l^2 >> 16, up to 0.46 tile at white, about 0.1 tile at mid grey). The tiles deform where the picture is
bright. With riley's 1/8 store the field stepped several codes from line to line in textured picture areas
and every steep edge came out ragged (in the model and in GHDL); with the 1/32 store the edges are clean
curves again (GHDL render with selfie.jpg in cmp_final.png). With a
black input the offset is zero and the wave alone runs. Source = Osc substitutes the shell's drifting
640 x 540 triangle lattice at the same point at full swing (up to 0.47 tile), so diamonds of deeper
interlock drift across.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the knob glides (K1, K2, K3, K5, P12), one serial
  16-step multiplier in 15 slots (pitch through a 2^(i/256) ROM, spread, depth, rate, the DDA start values and
  steps), the wave phase step, and four table refills: the blended edge profile (256 points, two neighbouring
  ROM profiles mixed in eighths, 8 clocks per entry, zone clocks 40 .. 2088), Depth x wave magnitude
  (256 entries by accumulation), the source square law (256 entries by second differences), and two colour
  tables of 9 steps each (ground -> A, and the accent's share). The line DDAs are reset at zone clock 2200.
- **Per line**: two DDAs stepped once (Y in tiles, theta along y), one lookup of f(Y) in the shape table and
  its difference from the line above (the vertical gradient of the X' family). No per-line multiplier.
- **Per pixel** (35 stages + output): two beam DDAs (theta along x, X) -> quarter-wave sine ROM (Snap: x 2.5,
  clipped) -> Depth table -> A = base +- wave + source, clamped to +-1 tile -> f(X) from the shape table ->
  A f(X) and A f(Y) (two 10 x 10 signed multipliers, each as hi / lo partial products) -> X', Y' -> gradients
  (finite difference along the line; |A| x |df| down the screen) -> each family's distance to its nearest
  edge in 1/8 pixels WITHOUT a divider (leading-one of the gradient, a 4-entry reciprocal, a 6-bit window of
  the distance) -> soft parity bits (9 levels) and outline coverage -> XOR blend of the two families ->
  XOR blend with the detail coverage (echo ring + diamond eye, same distance method) -> the outline's share
  -> the two colour tables, summed -> output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced
  8-sums, luma [1 2 1]/4 at the same group delay, luma 13/16 toward black) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (42) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3). Colours in the hardware convention (U = Cr).

## Status

- v0.1.0 built 2026-11-14 with router2 (the builder default). Not flashed.
- Post-route Fmax: **HD Analog 76.06 MHz (seed 2**; seed 1 missed), HD HDMI 81.95 (seed 1), HD Dual 90.07
  (seed 1), SD Analog 60.65 (seed 1), SD HDMI 70.96 (seed 1), SD Dual 87.21 (seed 1). HD needs 74.25, SD 27.
- 7,087 .. 7,106 / 7,680 LC (92.5 %, at the set's budget), 23 / 32 EBR. Standalone yosys of the previous
  revision: no `emulate_transparency`, no carry self-adds (dupin: none).
- Rebuild: `./build_set.sh kinetic metamorph`.
- This is the third build of the evening. The first lost its SD HDMI run to a router2 stall (2 overused
  wires on seed 1; killing that run by hand makes the builder fail the whole build, so let its timeout do
  it). The second completed (HD HDMI only 75.12 MHz) but its camera render had ragged edges, so the source
  store was changed (see Video) and the program rebuilt; that last build is the vmprog.
- Model = RTL (GHDL, black input), all pixel diffs 0:
  - 720p60 decimation 1, defaults (Ink): 0 of 918,720 (rendered on the second build's RTL; the final RTL
    differs only in the source store, which is all zero with a black input, and its own render is the next line);
  - 720p60 decimation 2 (the top-left quarter), defaults, final RTL: 0 of 228,960;
  - 720p60 decimation 2, Radial + Snap + Outline + reverse Rate + Bauhaus, captured 5 fields in (so the
    motion is checked too), second build's RTL: 0 of 228,960;
  - NTSC decimation 1, defaults, second build's RTL: 0 of 173,988 on each of the two fields (720 x 243).
  The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the generated
  avid), as in riley.
- With a camera picture (selfie.jpg) or Source = Osc the RTL differs from the model in about 7 % of pixels:
  the model's source side (lattice phase, the store's history from the previous field) is riley's
  approximation; the geometry path is the same. Judged by eye instead: the final RTL's 720p60 render with
  selfie.jpg is the last panel of cmp_final.png (clean edges at 4x zoom); the Osc panel there is from the
  second build's RTL (the 1/8 store), the final RTL was not rendered with Osc.
- Rasters rendered in GHDL: 720p60 and NTSC (SD). 1080i / 1080p were NOT rendered (the integer model has
  the modes, no RTL frame was compared).
- Not sim-checked: any knob away from its default (the glides start at the defaults, so a knob sweep needs a
  sim-only copy); the K1 / K2 / K3 / K5 / P12 ranges are from the integer model's control sheet only.
- Renders: `.kinetic_work/rb_metamorph/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (motion);
  sheet and strip are from the integer model.

Things to look at on hardware: SD, where a 64-px tile is only 14 field lines tall, so at Scale under about
25 % the echo contour and eye crowd the tile (Detail Off reads better there); high Morph + high Depth + Osc
together push A to the clamp (1 tile) and the tiles fold into slivers; the camera response lags the picture
by about 30 lines (the heavier vertical smoothing).

## Cut / deviations

- **Radial is diamond rings (|X| + |Y|), not circles**, and it does not pin the rim to creatures: a per-pixel
  radius or r^2 field with its own second-order DDAs and per-line multiplier was 2,900 LC in the first RTL
  (the whole design 8,742 LC, 114 % of the chip). The centre starts each cycle as squares (the Radial phase
  is offset a quarter cycle), and the rings then radiate.
- **Anti-aliasing is 9 levels with an approximate reciprocal** (the transition width is within about 12 % of
  one pixel) instead of 17 levels from true dividers: three dividers cost about 730 LC.
- The eye is a diamond (L1 metric), not round. The outline width is fixed in real pixels per video mode.
- Colour B of the palettes is not used (ground, A and the accent only).
- The spec's P12 = 0 "plain checkerboard" holds exactly only with Depth at 0: with Depth up, the wave rocks
  the edges both ways around square (A goes negative), so Depth stays visible at every Morph setting.
- The source adds luma^2, not luma, so mid greys bend the tiles only a little and highlights a lot.
- The source line store differs from riley's chassis: 16-bit words (12-bit state), 1/32 per line.
- The beam DDAs count down, so the tiling is the mirror image of the first prototype's (same figures).
- `field_n` is passed through one register (the core ties it high on every input), as in riley.
