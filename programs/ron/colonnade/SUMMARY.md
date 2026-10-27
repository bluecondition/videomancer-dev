# Colonnade — chrome sheets hung before a fluted colonnade

**v0.1.0 (2026-10-13): built, NOT flashed.** Made from a reference capture: an analog-synth
picture of mirrored chrome contour sheets (dark navy blobs ringed white/yellow/red, magenta and
cyan plateaus, X-shaped saddles) hanging in front of a colonnade of lit flutes. The sheets'
edges are bowed and carry a thin rainbow outline, the top right breaks into diagonal boxes, and
per-line streaks run through the flat areas. Presets are hand-written placeholders.

## Reading of the reference
- **Field.** The rings around dark centres, the X shapes between them, and their mirror
  symmetry read as a product of a horizontal and a vertical wave (an egg-crate): peaks and
  troughs give rings, and the zero crossings give saddles. Three tiles across, each folded
  about its centre.
- **Colourizer.** The hues follow a cyclic contour ramp: navy → blue → white → yellow →
  orange → red plateau → purple plateau → magenta plateau → violet → cyan plateau → blue →
  black. Flat saturated plateaus are where the analog colourizer clipped.
- **Flutes.** Pillars lit from the left everywhere (not mirrored): a crimson edge, a cream
  highlight, pink-purple, blue, and a black shadow. The field bends their highlight into
  fingers.
- **Sheet edges.** These are key boundaries. Where the key sweeps the palette, it leaves the
  thin rainbow line.
- **Streaks.** Per-line jitter of the colourizer input. They show most where the gradient is
  flat.

## What it does
1. **Tiles.** The phase `pt` gives Tiles cycles across, centred on the screen. Mirror folds it
   into a triangle (half a turn per half tile). Repeat runs a full turn per tile, so the field
   stays seamless but loses the mirror.
2. **Field.** `S = sin(A + B2) + sin(A − B2) = 2 sinA cosB2` (±4094).
   - `A` = the tile coordinate + the line's wobble/8.
   - `B` = the vertical phase, 0.56·Tiles cycles down the frame, so the cells stay round.
   - `B2 = B ± Melt·|sin(2A + q/2)|`, up to a quarter turn.
3. **Sheets.** `key = frac(pt + D) < Sheet`, with `D` = the same wobble × 4 plus centring. The
   sheets therefore sit centred on each tile, and their edges bow as the field melts.
   Lattice XORs in diagonal bands (0.8·Tiles per frame, 0.75·Tiles across, descending to the
   right). The first 6 px after each rise of the key (3 px on SD) sweep the sheet palette at
   43 (86) entries per px: the rainbow outline.
4. **Wobble** (per line): `Melt·(0.31·sin W1 + 0.06·sin W2)`, at 0.6 and 1.7 cycles per frame,
   travelling in opposite directions with Speed.
5. **Flutes.** The phase `pc` gives Flutes per tile. Lit: index = position + S/4. Glass: the
   field is sampled at `A = frac(1.5·pc)` inside the flutes (each flute shows its own
   compressed copy, alternate flutes half a turn apart), and index = position + 0.62·S.
6. **Colour.**
   - Sheet index = `S·G/32 + 2580` (saddles on magenta) + Flow + jitter.
   - Palette: 512 entries = 256 sheet + 256 flute, both cyclic. Even/odd bank ROMs with
     4-bit interpolation.
7. **Tear.** Per-line index jitter of −8..+7 → ±4 entries (half that on the flutes), plus a
   0–3 px start hold (0–1 px on SD).

| Control | Function |
|---|---|
| K1 Tiles | 1–8 mirror tiles across (square law; 3.2 at the default) |
| K2 Flutes | 2–12 flutes per tile (5 at the default) |
| K3 Sheet | Sheet share of each tile, 12–88 % |
| K4 Melt | 0 = a crisp egg-crate grid with straight sheet edges; 100 % = liquid swirls with deeply bowed edges |
| K5 Speed | Bipolar, still in the centre ±24. The field rolls, the sheets and flutes slide in opposite directions (parallax), and the bows travel |
| K6 Palette | Chrome (fitted to the reference) / Oil / Candy / Thermal |
| S7 Mirror | Mirror / Repeat (seamless, unmirrored) |
| S8 Lattice | Diagonal box bands XOR the sheets (the reference's top right) |
| S9 Flutes | Lit (tube shading) / Glass (refracted field) |
| S10 Tear | Per-line jitter streaks, default On |
| S11 Flow | The palette flows through the contours: ~0.1 cycles/s when still, up to ~1.8 with Speed |
| P12 Contours | 0.5–12 palette cycles over the field (square law): broad smooth chrome at 0, dense rainbow rings at 100 % |

## Engine
- **Sequencer:** one serial shift-add multiplier and one serial restoring divider.
  - **Per field** (first vsync edge after active video): `2^24/W`, `2^20/H`, Tiles²,
    `Ft = tiles·2^24/W`, `Fc = Ft·flutes`, `Fg = 1.5·Fc`, `Fd = 0.75·Ft`, and `t256·dy`, which
    gives the field and lattice line steps. Then the wobble steps, the duty and centring
    offset, Contours² → G (8..191), speed², and the time phases.
  - **Per active line** (from the generated avid's fall, ~20 clocks): the DDAs, two wobble
    sines from the line engine's own sine ROM, `wsum × Melt` (one multiply), the key offset,
    and the jitter.
- **Pixel path (26 stages):**
  - Four 24-bit phase accumulators (tile, flute, glass, lattice).
  - Mirror and key test, then the Glass mux.
  - Melt sine ROM, `|sin|×Melt` (operand split hi/lo), `B2`.
  - Two field sine ROMs, then `S`.
  - `S×G` (operand split), the sheet/flute/outline index, the palette ROMs, and the
    interpolation.
  - [1 2 1] + luma 13/16, and the gate.
- **ROMs:** 3 pixel sine ROMs + 1 line-engine sine ROM (quarter-wave 256×12, 1 EBR each);
  palette 6 × 1024×8 bank ROMs (12 EBR).
- **Contracts:** damier's generated avid; one 26-stage latency in every mode; the [1 2 1]
  Y/U/V smoothing and 13/16 luma bar fix; the blanking gate; glides (±4 deadband + ease) on
  K1–K4 and P12; every DDA starts half a line down on the bottom field.

## Build notes
- **v0.1.0 (2026-10-13), router2. Build seeds: HD HDMI = seed 2** (seed 1 missed timing); the
  other five are seed 1. Fmax: HD Analog 85.3, HD HDMI 74.67 (thin), HD Dual 83.3, SD Analog 80.0,
  SD HDMI 77.7, SD Dual 78.3 MHz. ~6660/7680 LC (87 %), 16/32 EBR (12 palette + 4 sine). Every
  bitstream passed iceunpack.
- If HD HDMI regresses on a rebuild: try a seed sweep first, then router1. The likely hot spots
  are the S14 `S×G` partial products (14×5 signed in one stage) and the S7 `|sin|×Melt`
  partials. Split them into 2- or 3-bit slices before restructuring anything else.

## Working files
`~/ref3_work/`: `ref.jpg` (the reference capture), `proto.py` (first float sketch), `proto2.py`
(an integer-flavoured numpy preview of the pixel algorithm that imports the palettes from
`colonnade.py`), `p3.png` (reference above, preview below).
