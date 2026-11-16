# Tangle (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** Flash `out/rev_b/ron/hardline/tangle.vmprog`.

## What it draws

Truchet tiles in Cyril Stanley Smith's quarter-arc form (1987), drawn with several concentric arcs per tile, so
the tiles join into endless meandering ribbons of parallel lines: loops, teardrops, S-curves and the odd full
set of rings, in crisp black and white. The default frame is about 11 x 6 tiles of 180 virtual px, four white
lines per tile on black, random turns, a lattice vertex on the screen centre. At the two ends of the Bend
slider the arcs become chamfered 45-degree elbows (octagons and squares, like circuit-board traces) and the
straight diagonals of the "10 PRINT" maze.

    tile (i, j) of side c, turn bit o; (u, v) tile-local in 0 .. 1, u mirrored when o = 1, then folded across
    the anti-diagonal onto the nearer of the tile's two arc corners
    E   = sqrt(u^2 + v^2)                                the round arc (an 8-iteration CORDIC per pixel)
    L   = max(u, v, 45/64 (u + v))   Bend below 50 %     a regular octagon: chamfered elbows, even line weight
        = u + v                      Bend above 50 %     straight diagonals
    d   = E + |2 Bend - 1| (L - E)
    tri = triangle(n d)              0 where n d is whole, 1 at n d + 1/2: n lines per tile at d = (k + 1/2) / n
    Flat: ink = tri > 1 - duty       Shade: Y = a ramp of tri        Blobs: (d < 1/2) xor checker xor o, solid

On every tile edge d runs 0 .. 1 along the edge (or 1 .. 0), and the line set is symmetric under that swap, so
the lines meet across tiles for any mix of turns. Every edge is anti-aliased over one pixel from the known
gradient of d (no finite differences); past a quarter cycle per pixel the edge widens so the bands fade to grey
instead of aliasing (only reachable on SD at the smallest tiles with 7 or 8 lines).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pans the lattice, 2 virtual px per knob count (+-1,024 px), glided |
| K2 | Y | the same, vertical |
| K3 | Period | tile side, 40 .. 400 virtual px, exponential (default 180); zooms about the lattice origin (the screen centre at X = Y = 0) |
| K4 | Lines | 1 .. 8 lines per tile, stepped with hysteresis (default 4) |
| K5 | Order | 0 % = every tile turned the same way (diagonal wave trains); up to 50 % tiles flip one by one until the turns are random (default); from 50 % tiles drop one by one into strict alternation; 100 % = a lattice of perfect circles |
| K6 | Camera | camera (and Osc) strength, 0 .. 100 %, default 100 %. 0 % = exactly the black-input picture |
| S7 | Glide | On (default): X, Y, Period, Order, Camera and Bend ease 1/16 of the gap per field, never slower than half a knob count per field, landing exactly. Off: zero easing, the value latched at the field start is used that field (the accumulators still track) |
| S8 | Fill | Lines / Blobs: the classic two-coloured Truchet regions drawn solid white over the lines (white islands, the lines rippling between them) |
| S9 | Shade | Flat / Shade: a triangle ramp across each band, black in the gaps to white on the line centres (round pipes) |
| S10 | Weight | Bold (equal bands) / Fine (lines 1/6 of the period, never under 2 virtual px). In Shade: Bold = 4 flat steps (black, dark grey, light grey, white), Fine = a smooth ramp |
| S11 | Source | Video / Osc |
| P12 | BEND | the headline: 0 % chamfered elbows (octagon rings round the arc corners, squares round the others), 50 % round arcs (default), 100 % straight diagonals (the 10 PRINT maze). Continuous in between: rounded octagons, then arcs that flatten |

Pot jitter is held by a +-1-count hold on every glided knob (never by easing).

## Camera

The camera is never shown. Its luma goes through the shell's source field (luma >> 2, 32-px box, vertical 1/8
IIR) and becomes a weight cw = min(1, 2 g luma^2), g = K6 (0.25 / 0.5 / 1.0 at 25 / 50 / 100 %), from a
256-entry table refilled every field. It does two things:

- **Per tile, the turn.** The weight is sampled once per tile (at the tile's first pixel on the tile row's
  first line, then held for the whole tile) and compared with a third hash byte: where it wins, the tile is
  forced to turn 1. Bright regions comb out into parallel diagonal waves; dark regions keep the Order pattern
  (at Order 0 % bright and dark are the two wave directions). Each tile's last decision is kept in a 64 x 32
  bit map and gives the comparison 3 counts of hysteresis, so a still camera with noise does not make tiles pop.
- **Per pixel, the line weight.** The band threshold falls with the weight: Bold lines swell from 1/2 to 0.87
  of the period, Fine lines from 1/6 to 0.6. In Shade the ramp steepens instead (Y = tri + min(tri, cw / 2),
  clipped), so the pipes fatten and the gaps stay black.

With a mid-grey subject on a bright ground (the bear test picture) at 100 % the ground is solid waves with
hairline gaps and the subject is a darker, tangled silhouette with its features in the line weight; at 25 %
the lines only thicken a little and a third of the bright tiles turn. Tiles are always whole and every edge is
the same clean curve as without a camera. Osc substitutes the shell's drifting 640 x 540 triangle lattice
(0 .. 222 of 255, which saturates the weight over about half its cycle, 2-4 x a camera picture): diamonds of
fat combed lines drift across. Osc is the only thing that moves by itself.

## Engine

- Per field (vblank zone, one serial multiplier, 9 slots): tile size from an exponential ROM with a 1/16-step
  interpolation; the tile coordinate of the beam's pre-pixel and of line 0; its steps per pixel and per line;
  lines per pixel -> the anti-aliasing gain as a 2-bit mantissa and a shift; the Bend table (Bend x 0 .. 63,
  by accumulation); the two camera tables (weight, and threshold / steepening, by second differences).
- Per line (line engine, computes the next line): tile row j, row coordinate v, the row's hash term (three
  adds), and whether the tile row starts on that line (the tile sampler runs there).
- Per pixel (35 stages): tile-coordinate DDA -> tile hash (4 adds between xor-shifts; no row, column,
  diagonal or lattice correlation above noise) -> turn bit (Order thresholds, checker, camera) -> mirror and
  fold -> 8-iteration CORDIC (16-bit, gain undone by two shift-adds) in parallel with the octagon / diagonal
  metric -> Bend blend through the per-field table (two 6-bit lookups) -> n d by gated shifts -> triangle,
  blob coordinate -> edge distance x gain -> coverage -> luma ROMs -> blob over lines.
- Output: luma [1 2 1] / 4, then y - (d >> 3) - (d >> 4) (white 777), blanking gate; U = V = 512 constant.
  One latency in every mode: 48 clocks (`processing_delay_clks = 48`). Generated avid (the shell's
  parity-locked start), virtual 1080-line coordinates (SD shows the centre 4:3), per-field engine start.
- No per-pixel multiplier, divider or square-root ROM.

## Status

- Built 2026-11-15 with router2 (the build default), every config on seed 1. Post-route Fmax: HD Analog
  95.58 MHz, HD HDMI 90.02, HD Dual 88.98 (need 74.25); SD Analog 93.47, SD HDMI 81.49, SD Dual 95.11 (need 27).
- 6,681 .. 6,708 / 7,680 LC (87 %), 15 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin: 0).
  Standalone yosys: 3,917 LUT + 2,880 FF (the pipeline's delay registers rarely pack with a LUT).

- Model = RTL (GHDL), pixel diff 0 everywhere it was run:
  - three full 720p60 decimation-1 frames (918,720 px each): black input, Osc, the bear picture at K6 100 %;
  - eight 640 x 360 renders (720p60, decimation 2) covering Blobs, both Shades, Fine, 1 / 2 / 7 / 8 lines, both
    Bend ends and both sides, Order 20 / 78 / 100 %, pan, the smallest and largest tiles, Osc with three
    fields of hysteresis state, and Glide On in flight (three fields of easing on all six glided controls);
  - NTSC (720 x 243 per field): black input, and both fields with the bear picture (the model has to run the
    tester's field sequence, since the line store and the tile map carry over between fields).
  The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the generated avid
  starts 4 clocks after the delayed source avid; the geometry is offset to match, so the picture is centred).
- Renders: `.hardline_work/rb_tangle/cmp_final.png`, `sheet.png`, `strip.png`, `camera.png`.
- Rebuild: `./build_set.sh hardline tangle`.

## Cut / deviations

- **Line phase.** Lines sit on d = (k + 1/2) / n, not on d = k / n (the spec's "+ 1/4"). With k / n the tile
  corners carry half-dots and four-pointed stars, one line per tile degenerates to polka dots that ignore the
  turn bit, and Fine puts a dot on every vertex. With k + 1/2, one line per tile is the classic Truchet tile
  and "Lines = n" is literally n lines per tile. At Bold the two are the same picture inverted.
- **Bend 0 % is an octagon, not a square elbow.** With max(|u|, |v|) a tile looks the same for either turn
  (concentric squares round all four corners), so Order and the camera's turn went dead at that end of the
  slider. The octagon max(u, v, 45/64 (u + v)) keeps the turn visible (octagons round the arc corners,
  squares round the others) and has a unit gradient, so the lines keep their weight.
- **Blobs** draws the two-coloured regions solid white OVER the lines rather than as a plain two-colouring:
  with 4 lines per tile a two-coloured band pattern is the Bold picture shifted a quarter band, which is not
  a look-flip. With an even line count a hairline of ground separates island and first line; with an odd
  count the first line merges into the island.
- **Period 40 .. 400 px** (spec 24 .. 400): at 24 px and 8 lines the bands would be 3 virtual px, under the
  set's 2-px line rule; at 40 px no line count needs clamping.
- True distance comes from a per-pixel CORDIC, not squares + a square-root ROM (the spec's suggestion): 8
  iterations cost about 260 LC against about 750 for a squarer plus a 13-bit root, and need no per-line squares.
- The anti-aliasing width is taken from the horizontal pixel step. On 480i / 576i a field line is 2.2 x taller
  than a pixel is wide, so near-horizontal edges are crisper there than on HD (same geometry).
- The tile's camera sample is taken at the tile's top-left corner (about half a tile from its centre), from
  the line above; the line-weight map lags the picture by a few pixels. Neither shows, since the camera is
  not displayed.
- Not simulated: the 1080p and 1080i rasters (720p60 and NTSC were).
