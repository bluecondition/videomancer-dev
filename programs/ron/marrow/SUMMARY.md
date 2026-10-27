# Marrow — a spine of contoured blobs in signal yellow

**v0.1.0 (2026-10-13): built, NOT flashed.** Made from a reference capture (an analog-synth
frame: a column of rounded rings stacked down the middle, side lobes breaking off into
islands, false-colour contours — dark-red outlines, azure halos, peach and orange
interiors — in a flood of flat yellow, smeared sideways and torn line by line). Presets are
hand-written placeholders.

## Reading of the reference
Held up in a numpy preview (`~/marrow_work/proto2.py`, integer-faithful to the RTL):
- **Field.** Two cosine oscillators (across and down) summed into an egg-crate lattice of
  hills and valleys, plus a diagonal third wave that breaks the regularity. The centre
  rings sit in the valleys; the dark-red "knots" between them are the saddles.
- **Fold.** The outlines are a full-wave fold: the dark-red line is where the field crosses
  zero, and the colours run back up the palette on the other side (the top ring of the
  reference reads red → slate → blue → light blue → peach going inward, the same sequence
  as outward). Deeper valleys come back as rings.
- **Envelope.** A centred, waisted column: outside it the index climbs into the palette's
  clip (flat yellow), so only the deepest valleys poke through as islands.
- **Smear.** Inside each ring the colour ramps left → right (blue on the left, red on the
  right): a causal left-to-right lag on the palette index, not a mirror.
- **Tear.** Per-line horizontal offsets in held runs and bursts, which through the smear draw
  the streaks.

## What it does
1. `egg = −cos(φx) − cos(φy + warp·cos(φv)) − warp·cos(φd)`, with φx = |x − c|·kx + wiggle(y)
   (Mirror off: no |·|), φv the slower warp wave across, φd = diagonal.
2. `G = Depth·egg + Level`, triangle-folded (|G| below one span, folding back above).
3. `index = min(1023, fold + 1.5·max(0, |x − c|/(W/2) − width(y))·1024)`; width(y) is
   Spread × (1 − 0.84 × a slow vertical sine).
4. One-pole lag left to right over the index; Drip makes rises 8× slower.
5. Palette: 8 × 256 entries; entry 0 = the fold colour, 255 = the clip (background).

| Control | Function |
|---|---|
| K1 Cells | 2–16 cells across and down (square law) |
| K2 Warp | regular lattice → melted blobs: warp depth, wiggle (up to 0.3 cycle), diagonal weight |
| K3 Smear | lag 2–256 px (exponential, x-dithered between powers of 2) |
| K4 Speed | bipolar drift, still in the centre ±24; positive = blobs rise. The warp, diagonal, wiggle and waist drift at other ratios |
| K5 Spread | column width before the yellow floods in (0.08–1.3 half-widths) |
| K6 Palette | Marrow (fitted to the reference) / Abyss / Thermal / Acid / Lagoon / Ink / Candy / Venom |
| S7 Mirror | reflect the lattice about the centre — the spine (default On) |
| S8 Tear | per-line tearing: ±8 px on every line, held on half the lines, bursts of 8–39 lines up to ±64 px (HD; half on SD). Default On |
| S9 Drip | rising edges 8× slower than falling: trails out of every blob |
| S10 Shape | Round (cosines) / Diamond (triangles) |
| S11 Breathe | the fold level swings ±200 over ~6 s: rings swell and shrink, the yellow tide breathes (default On) |
| P12 Depth | fold depth ×1–×12 (square law): soft contours at 0, dense op-art rings at 100 % |

## Engine
- **Sequencer** (one serial shift-add multiplier, one serial divider). The multiplier takes an
  op code and fetches its own operands into a register (totem's timing lesson).
  - **Per field** (first vsync edge after active video): 2^24/W, 2^20/H, Cells², kx/ky and the
    derived steps, −c·k start constants, Depth², speed², the time phases, Spread/Warp/Smear
    curves, the lag one-hots, Breathe (sine ROM + multiply).
  - **Per line** (kicked at the line's g_go, computing the NEXT line, ~130 clocks so it fits
    SD's 858-clock line): the vertical phases, wiggle and waist sines (a 256×8 sine ROM)
    and products, the xorshift32 tear draw and burst counter, and the four accumulator
    starts `−c·k + j·k`.
- **Pixel path**: four accumulators (three wave phases, one envelope), abs for the mirror,
  four wave-ROM reads (C_WAVE 512×8: cosine | triangle), two Warp products, the Depth
  product, fold + envelope + clip, then the lag: `L + (i >> s) − (L >> s)` carry-saved into a
  single adder (one-hot shift select; the Drip rise/fall choice compares the index with the
  lag two pixels back). Palette ROMs (3 × 2048×8). Latency 21 stages in every mode.
- **Contracts**: damier's generated avid; [1 2 1] Y/U/V smoothing + 13/16 luma before the
  blanking gate; glides (±4 deadband + ease) on K1/K2/K3/K5/P12; bottom field half a line
  down; palettes U/V-swapped (`marrow.py`).

## Build notes
- **v0.1.0 (2026-10-13), router2. Build seeds: HD Analog = seed 2** (seed 1 missed timing); the
  other five are seed 1. Fmax: HD Analog 82.1, HD HDMI 85.9, **HD Dual 74.8 (thin margin)**,
  SD Analog 76.6, SD HDMI 79.9, SD Dual 67.1 MHz (SD Fmin 27). ~7190/7680 LC (**93.6 %**),
  17/32 EBR (4 wave + 1 sine + 12 palette). Every bitstream passed iceunpack.
- A router1 profile route of the same HD Analog netlist (seed 3) gave 85.4 MHz: if a later
  version gets tight, rebuild with `NEXTPNR_ROUTER="--router router1"` before restructuring.
- LC is much higher than estimated. Trim before adding features. Candidates: compute the
  envelope late, on its own accumulator loaded at the S9 line-start flag (−90 FF of carried
  env), narrow the 30-bit phase accumulators, an 8-bit Depth operand.

## Working files
`~/marrow_work/`: `ref.jpg` (the reference), `proto.py` (float look prototype),
`proto2.py` (integer preview of the RTL pipeline, imports the palettes from `marrow.py`),
comparison grids `g*.png`.
