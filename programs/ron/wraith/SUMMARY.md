# Wraith — neon flame lobes, hard-synced and tiled

**v0.1.0 (2026-10-13): built, NOT flashed.** Made from a reference capture (an analog-synth
frame: glowing neon flame forms — violet rims, cyan cores, pale white, pink, peach and signal
green — on an azure ramp, repeated three times across the frame, with tapered horizontal fins,
a hard colour seam down the middle of every tile, V-hold slips cutting the picture into bands,
and thin magenta tracking lines). Presets are hand-written placeholders.

## Reading of the reference
Held up in numpy previews (`~/silk_work/proto3.py` float, `imodel.py` integer-faithful to the
RTL):
- **Tiles.** Horizontal period 649 px of 1899 (2.93 tiles). The background is a triangle ramp in
  blue (B saturates near the spine, G ≈ B/4) with a dim brown fringe in the trough between
  tiles; the trough wiggles ±25 px down the frame.
- **Seam.** Every tile has a vertical seam at the ramp's peak: shapes are cut by it (the tower at
  the top ends exactly on it), or change colour across it. That is a slave oscillator
  **hard-synced** at the spine: its phase restarts there.
- **Lobes.** The shape mask is crescent wings that taper to points, right-hand spikes and a
  flared tower, i.e. cos lobes of the slave oscillator windowed by the tile triangle: lit only
  near the spine, thinning to a point as they swing away from it.
- **Fins.** Hard horizontal edges with long tapered fins on the other side = a vertical
  sawtooth with a curved (s²) ramp throwing the lobes sideways faster and faster until it
  resets.
- **Colour.** One scalar per pixel into a cyclic neon palette (rim → magenta → sky → cyan →
  white-lavender → pink → peach → yellow-green → green), with a slow per-line hue shift (the top
  fin reads peach/green, the body violet/cyan).
- **Cuts and lines.** The content below y=585 repeats rows from y≈180: V-hold slips (a jump in
  the vertical phase). The magenta lines are a chroma tint (+U +V, luma lift) over 3–5 rows
  with a soft profile; the dark lines are a 2-row luma dip.

## What it does
1. Master tile phase m (0 at the spine, centred on the screen), T = |1 − 2m|.
2. Slave phase φ = w(y) + u·v(y), u = m (Mirror: u = T); F = cos φ.
3. G = (F + 1)/2 + 1.625 (T − 1) + L(y). Lit where G > 0.
4. index = G/2 (Rings: 2G) + hue(y) into the cyclic 256-entry flame palette; else the
   128-entry background ramp by T.
5. Per line: s = frac(Bands·q + t) (Flip: 1 − s); v = Lobes·(1 + 0.34 sin)·(1 + Flare (1 − s)²);
   w = 0.3 sin + Lash·s²; L = Glow + 0.14 sin; hue = 0.125 sin + drift; wiggle 0.035 tile.
6. Tear: V-hold slips (HD 1 line in 256, SD 1 in 64) of up to ±½ frame on the line phase, and
   ±3 px per-line jitter (SD ±1) held on half the lines. Lines: 1-in-256 (SD 1-in-64) events,
   ¾ magenta (5 lines, ¼ ¾ 1 ½ ¼ of a 96..192 peak on U and V, half that on Y), ¼ dark (luma and
   chroma >>1 then >>2). Both patterns are reseeded per field from a seed held 8..39 fields.

| Control | Function |
|---|---|
| K1 Tiles | 1–7 flames across (2.95 = the reference) |
| K2 Lobes | 0.5–3.5 slave cycles per tile: one fat body → ribbons |
| K3 Flare | 0–3: how much the lobes narrow after each fin reset |
| K4 Speed | bipolar drift, still in the centre ±24; positive = fins rise. The six vertical waves and the hue drift at their own ratios |
| K5 Glow | flame level −0.375 … 1.12: thin slivers → flames flooding the tile |
| K6 Palette | Silk (fitted to the reference) / Ember / Venom / Frost / Orchid / Aurora / Spectrum / Mono |
| S7 Mirror | sync on the triangle: symmetric flames, no seam |
| S8 Tear | V-hold slips + per-line jitter (default On) |
| S9 Lines | magenta tracking lines and dark dips (default On) |
| S10 Rings | index = 2G: contour rings through the palette |
| S11 Flip | reverse the fin saw: the fins flare the other way |
| P12 Lash | fin sweep 0.1–2.1 turns and 1.5–4.5 bands per frame: calm flowing flames at 0, shredded lashing fins at 100 % |

## Engine
- **Sequencer** (one serial shift-add multiplier and one serial divider). No state writes
  the multiplier's op code or operands: the op code and one-hot operand-source selects are
  registered every clock from the state number (`p_opmap`) and the fetch is a flat AND-OR.
  - **Per field** (first vsync edge after active video): 2^24/W, 2^24/H, tiles, the tile step
    kT and the centre phase −(W/2)·kT, the control curves, speed², seven time phases, the
    tear/lines seed hold.
  - **Per line** (kicked at the line's g_go, computing the NEXT line, ~300 clocks so it fits
    SD's 858-clock line): xorshift32, line phase (+ slip), event state, six wave phases
    (q × f), five sines (1024×16 ROM), s², (1 − s)², v, w, L, hue, wiggle, jitter, the master
    start phase, the tracking-line tint.
- **Pixel path**: one 24-bit tile accumulator, T fold, u·v (12×16, split into two 12×8),
  1.625 T + E, φ, cosine ROM (1024×8), G, Rings/G/2, + hue, six palette ROMs (flame 3 × 2048×8,
  background 3 × 1024×8), flame/ground mux, dark-line and tint stages. Latency 17 stages in
  every mode.
- **Precision note**: the lobe phase u·v amplifies any per-line step in v or w on the far
  side of the spine (u → 1). With a 256×8 sine and 8-bit (1 − s)² the integer model showed
  horizontal streaks and stair-stepped edges on the left half of every tile, so the sine is
  1024×16 and v is carried in 1/4096 cycles per tile.
- **Contracts**: damier's generated avid; [1 2 1] Y/U/V smoothing + 13/16 luma before the
  blanking gate; glides (±4 deadband + ease) on K1/K2/K3/K5/P12; bottom field half a line
  down; palettes U/V-swapped (`wraith.py`); the magenta tint adds to both chroma axes, so it
  is swap-invariant.

## Build notes
- **v0.1.0 (2026-10-13), ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_programs.sh ron
  wraith`). **Build seeds: HD Dual = seed 3** (seed 1 missed, seed 2 stalled); the other five
  are seed 1. Fmax: HD Analog 80.4, HD HDMI 82.3, **HD Dual 75.9 (thin)**, SD Analog 81.4,
  SD HDMI 75.0, SD Dual 79.1 MHz. ~6230/7680 LC (81 %), 24/32 EBR (2 cosine + 4 sine + 12
  flame + 6 background). Every bitstream passed iceunpack.
- **router2 does not close this netlist**: on the same json it gave 71–72 MHz on HD HDMI
  seeds 1–2 and best-effort 62.5 MHz on HD Dual after six seeds, where router1 gave 82.3 /
  75.2 on HD HDMI seeds 1 / 3. Rebuild with router1.
- Timing history: v1 wrote `mul_op` from 17 states (state decode → `mul_op` CEN critical,
  2 ns logic / 11 ns routing); the registered op map moved it to the binary-decoded operand
  fetch mux; the one-hot selects moved it to state decode → line-engine register CENs
  (routing-bound). If a later version gets tight, write `p_opmap`'s arms explicitly (it
  still computes `q_st − 33`, a carry chain on the path) before restructuring further.

## Working files
`~/silk_work/`: `ref.png` (the reference), `chan.png` / `mask.png` (channel split and the
shape mask), `proto3.py` (float look prototype), `imodel.py` (integer model of the RTL, imports
the palettes from `wraith.py`), comparison grids.
