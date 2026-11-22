# Monochord: the harmonics of a vibrating string

**v0.1.0 built 2026-11-22, not flashed.** Patchbay wave 2. A V staircase gives each row of the screen a harmonic
number; an H-locked sine VCO through a full-wave rectifier makes that string's standing-wave envelope; a window
comparator keys it as a chain of glowing lenses, and a per-row LFO swings a white-hot string inside every lens.
The default frame is the textbook harmonic series: six strings, 1, 2, 3, 4, 5, 6 lobes from top to bottom.
Presets are hand-written placeholders.

## The patch

    V staircase (row counter, K2 Rows)  ->  harmonic number  h_n = F (1 + n s)          (K1 Fundamental, K3 Partials)
    H-locked sine VCO, hard-synced to H, h_n half-waves across the screen  (+ a travel phase, K4 Speed)
        -> full-wave rectifier |sin| -> VCA (P12 Pluck x a slight per-row unevenness)
        + camera luma: 8-px box, vertical slew, horizontal slew, square law, x K6  -> swells the VCA
        = the envelope  e = K0 |sin|
    in-row V triangle |v|  and  e  -> window comparator (the lens key)
        -> fringe outside the edge, AA edge, bright dwell ramp just inside, translucent body
    per-row LFO (sequencer; rate = (h_n + detune) x Speed) -> VCA on the signed sine: c e sign(sin)
        -> narrow comparator: the white-hot string            (S7 Strobe: the string now + the envelope's
                                                                outline = the string at its two extremes)
    the VCO's phase near zero (node distance) -> node comparators: beads on the nodes, fret ticks through them (S9)
    harmonic number -> colourizer hue (K5 rotation + spread; S10 Amber: one phosphor, K5 picks it)
    two strings per pixel (own row + the nearer neighbour; S8 Fan: fundamental + top row's harmonic on one
    full-height string) -> additive mixer (Y, U, V sums, clipped: flat tops) -> output conditioning

Every row's VCO is hard-locked at the left edge, so integer harmonics put nodes on both screen edges; a
fundamental or partial step between detents slides the last lobe off the right edge.

## What it draws

Black ground, six strings stacked down the screen (row height 180 virtual lines). Row n carries n lens-shaped
lobes pinched at their nodes: one long blue lens, two violet, three magenta-red, four red, five amber, six green.
Each lobe is translucent saturated colour with a brighter band just inside its edge and a thin fringe of the
next hue outside it; a pale white-hot string curve swings inside it; a white bead sits on every node and a thin
cream fret tick runs through each node across its own row. At P12 600 the lobes just touch their neighbours at
the antinodes; where they overlap they add.

Motion at the defaults: each string swings at its own rate, rising with its harmonic (row 1 about 0.26 Hz, row
6 about 1.5 Hz, detuned so the rows never loop together), and the whole set of lobes drifts slowly to the right
(about one screen width a minute; K4 left of centre reverses it, the centre locks the nodes to the edges while
the strings keep swinging).

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Fundamental | H VCO (base frequency) | 1 .. 4 half-waves of row 1 across the screen; flat detents (+-40 codes) at the integers, glided between (the lobes slide off the right edge). 0 = 1 (default) |
| K2 Rows | V staircase (step count) | 2 .. 8 strings, stepped, +-8 codes of hysteresis (0 and 1023 always reach 2 and 8). Default 658 = 6 |
| K3 Partials | V staircase -> VCO CV (step size) | harmonic step per row, 0 .. 2 with detents at 0 / 1 / 2: 0 = every row the fundamental, 512 = the harmonic series 1 2 3 4 .. (default), 1023 = odd partials 1 3 5 7 ..; between, the lobes glide |
| K4 Speed | travel phase, LFO rate | bipolar, +-8 deadband, square law: lobes travel right (left of centre: left) up to 0.3 screens/s, and the string LFOs run up to 3.9x faster. Default 640 = a slow drift |
| K5 Colour | colourizer | rotation 0 .. 0.75 turn and spread 1.5 .. 3.5 hue steps per harmonic (16 hues, stepped). Amber: picks the phosphor (amber at 512) |
| K6 Camera | camera VCA (swell) | box-filtered luma, squared, swells every string's amplitude where the picture is bright: 0 = the black-input picture, 25 % = gentle, 100 % (default) = up to +2 x (A + 0.3 H) for full white, clamped at the row height |
| S7 String | comparator routing | Blur (filled lobes with the swinging string) / Strobe (no fill: the string now and the envelope's outline, i.e. the string at its two extremes: three thin curves per lobe) |
| S8 Stack | V staircase / VCA | Ladder (rows apart) / Fan (one full-height string: the fundamental and the top row's harmonic superposed, at half its amplitude, in glassier lenses) |
| S9 Nodes | node comparators | Off / On (default): white beads on every node of the own row (Fan: the fundamental's) and fret ticks through them |
| S10 Colours | colourizer bank | Spectrum / Amber (one phosphor colour for every string, a scope photo) |
| S11 Source | camera VCA input | Video / Osc (the shell's drifting triangle lattice at 2.5x the camera's gain) |
| P12 Pluck | VCA (amplitude) | half-lobe height / row height: 0.06 (0 %: slivers, every other control still visible) .. 0.5 (50 %: lobes filling their rows) .. 0.8 (100 %: lobes 1.6x the row height, overlapping and adding, flat-topped where the mixer clips). Glided. Default 600 |

## Video

- **Video (the default).** Camera luma, (Y >> 2) - 16, is averaged over 8-pixel cells, slewed down the screen
  (one pole, 1/32 per line, the state in a block RAM), slewed across (one pole, 1/16 per pixel), squared
  through a table and scaled by the per-field gain K6 x (A + 0.3 H). The result is added to every string's
  amplitude before the envelope product, so bright parts of the picture swell the lens chains (multiplicative:
  nodes stay pinched) and the white-hot string follows the swollen envelope. The slews keep every edge a clean
  curve. With a black input the patch gives its full picture.
- **Osc.** The shell's triangle lattice (640 x 540 virtual px, drifting) feeds the same path at 2.5x the
  camera's gain: the lenses swell into a slow travelling diamond pattern.

## Engine

- **Per field (vblank zone).** A shared glide (K1, K3, K6, P12); the rows' hysteresis; one serial shift-add
  multiplier runs 12 per-field products (coarse-raster row height, the two detents, F s, amplitude, camera gain
  chain, speed^2, travel step, LFO gain), then an 8-row loop (112 clocks per row) computes each row's phase
  increment, start phase (h x travel), amplitude, hue, LFO rate, the LFO's read-modify-write, its cosine
  (through string 0's sine ROM port) and the node-distance scale, into a row table in block RAM (8 words per
  row). Every engine restarts at each zone.
- **Per line (hblank, 19 .. 26 clocks after the generated avid falls; line 0 by a zone kick).** A row tracker
  (no divide: a y-in-row accumulator), the two strings' rows and in-row offsets v, 10 table reads straight into
  their line registers and phase DDAs, and the bead / fret widths for string 0 (two tiers of |v|).
- **Per pixel, two strings.** Phase DDA (20 bits per cycle); the folded 12-bit node distance addresses a
  1024-entry quarter-sine ROM and drives the node comparators; K0 = A + swell, clamped; e = K0 |sin|; c e;
  the fill level from e - |v| (fringe, AA, peak, four dwell steps, body; Fan: dim body); the string from
  |+-c e - v|; a 4-bit level into a 512-entry colourizer ROM (palette x hue x level -> dY, dU, dV); the
  additive mixer and clip; the shell's two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16, blanking gate.
- **Contracts.** Generated avid; one latency (C_LAT 24) in every mode; virtual 1080-line rows; the strings span
  the screen width at every resolution; on coarse rasters (1080i, 480p/576p, 480i/576i) the strings run in units of
  384 / ys virtual lines so every thin feature keeps 720p's line count; colours in the hardware convention
  (U = Cr, V = Cb); the output boxes' running sums resync on blank; block RAMs: one write each, registered reads,
  fabric copies before arithmetic, no_rw_check only where reads never meet the write address.

## Status

- **v0.1.0 built 2026-11-22, not flashed.** `out/rev_b/ron/patchbay/monochord.vmprog`; rebuild with
  `./build_set.sh patchbay monochord`.
- router2 (the builder's default), all six configs on seed 1. Post-route Fmax: HD Analog 83.35, HD HDMI 84.89,
  HD Dual 83.89, SD Analog 84.49, SD HDMI 84.48, SD Dual 72.94 MHz (SD Fmin 27). 6,992 .. 7,023 / 7,680 LC (91 %),
  21 / 32 EBR (two 1024-entry sine ROMs, two 512 x 24 colourizers, the square table, the row table, the luma cell
  store, the sync ring and the two output-box rings). iceunpack OK x 6; the packed config carries the labels and
  presets. No pin-skew screen was run.
- Checks: GHDL check clean; `scan.py` 0 files, 0 sites; standalone yosys: no transparency emulation.
  GHDL 720p60 decimation 1, black input, defaults: **0 differing pixels against `imodel.py`** over the whole frame
  (field 2, 4-px capture offset). Also 0 differing pixels: Rows 0 (2 strings) and Rows 1023 (8 strings) at 720p decimation 2
  (field 4, so the LFO / travel motion between fields 2 and 4 matches too), both 480i fields (`rtl_ntsc_view.png`)
  and both 1080i fields (`rtl_1080i_view.png`): the same composition at every raster. The Osc and selfie frames
  match in structure (`cmp_final.png`); the model's camera path is approximate (it restarts the vertical slew
  each field and the lattice / column alignment differs by a few pixels), so those two are not pixel-compared.
- Presets are hand-written placeholders: Series (the defaults), Odd Partials, Fan Pluck, Strobe Scope (Amber),
  Unison (every row two lobes), Lattice Osc.

## Cut / deviations

- **Two strings per pixel, not three or four** (the spec allows two). The first build with three evaluators,
  a bead multiplier, an afterglow line and a three-phase strobe synthesised to 10,620 LC before the core (about
  12,000 with it); the shipped engine is 5,578 before the core. Consequences: a lobe is clamped at the row
  height (a third string would be needed beyond it), so P12 100 % gives lobes 1.6x the row height, not 2x;
  Fan superposes two harmonics (fundamental + the top row's), not three or four.
- **Strobe** shows the string now plus the envelope's outline (the string's two extremes: three curves per
  lobe), not three LFO phases.
- **Beads and frets** are drawn for the own row only (in Ladder that loses nothing; in Fan the top harmonic's
  nodes have no beads). Beads are two-tier rounded blobs, not exact discs.
- The afterglow line of the prototype was cut.
- **SD / 1080i**: thin features (fringes, strings, beads, frets) keep 720p's line count, so on 480i / 576i they are
  about 3x thicker in screen terms (1.33x on 1080i); the strings span the full 4:3 width on SD, so lobes are
  narrower there than on HD but the counts are the same.
- **P12 0 %** (slivers) and Strobe draw some single-row bands by design (a sliver 22 lines tall is made of thin
  level bands); the default frame and every other control setting checked have none (thincheck.txt).
