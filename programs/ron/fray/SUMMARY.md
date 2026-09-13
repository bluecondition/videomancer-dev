# Fray — edge-launched scanline filaments

Livewire's "edges as material" meets prism's rightward trail, rebuilt so that every
scanline gets its OWN filament instead of one even band. Edges are never drawn; they
are only the point where a filament launches. Streaming pipeline, no frame buffer:
filaments can only extend to the right (a leftward/mirror pass would need a
line-reversal buffer — see prism notes).

Status: v0.1 built 2026-09-05, not yet flashed.

## Engine

1. **Sobel** on 8-bit luma over a 3-row window (2 luma line buffers, 8 EBR). No
   smoothing, no solidify, no min-run — per-line dropouts are the aesthetic here and
   K6 Density handles sparsity. Sign of gx gives polarity (falling = bright→dark
   going right).
2. **Spatial hash** of (line-group, column/16, seed): 7 pipelined single-op stages
   (`a=(lg&xg)+seed; b=a^a>>5; c=b·(1+2^5+2^11); d=c^c>>7; e=d·(1+2^9+2^17);
   f=e^e>>13`). Spot-checked in Python: every used field has < 0.006 correlation
   with its horizontal/vertical/diagonal neighbour and with the other fields.
   Bits 23..16 → length, 19..12 → fire gate, 15..0 → run seed. The line group is
   `frame_line >> K5`, so K5 Bundle turns independent per-line lengths into
   sheaves of up to 128 lines sharing a length (which is what makes them read as
   bundles rather than noise — patch-level randomness, per etchplate).
3. **Run state machine** (one filament per line at a time): launch on a fired edge,
   count down `len = max(2, hash8·len_max/256)` where `len_max = P12·width/1024 + 8`
   (screen-fraction, mode-portable), advance a 16-bit phase by K4's step each
   pixel; the run's own 16-bit seed steps at each phase wrap (Morse: xorshift,
   Bits: rotate). Hold colour is the source pixel 2 columns left of the edge (the
   object side of a falling edge). Stutter re-samples it every period.
4. **Pattern** (K1) → ink + intensity 0..256 from the phase. **Colour** (K3) picks
   the base: held pixel, boosted live video, inverted live video, white, or a
   32-entry palette ROM (rainbow / heat / polarity). Output =
   `ground + (base − ground)·int/256`, one signed multiply per channel split into
   hi/lo partial products (each alone in its stage, intensity register duplicated
   per multiplier with `keep`); the lerp form means comets fade INTO whatever the
   ground is.
5. Dry video travels through a 256×32 EBR ring (read 5 back) then a short FF
   chain; one latency (15) for every mode; blanking-gated at the output register.

Timing notes (HD, 74.25 MHz): the first netlist routed 60–79 MHz across seeds.
Three routing-bound paths were removed structurally: the shared intensity
register fanning out to three multipliers (per-consumer keep copies), the phase
adder's carry-out driving the 30-bit hold register's enable straight off the
hsync mux (registered hsync edge + wrap acted on one pixel late), and the 9×12
length multiply (hash chain started one column ahead via a cx0+1 counter so the
product splits into two 4×12 partials).

## Controls

| Ctl | Name | What it does |
|---|---|---|
| P12 | Length | max filament length as % of screen width; 100 % = full-width streaks |
| K1 | Pattern | Dash · Morse (random duty per period) · Comet (sawtooth heads) · Beads (triangle) · Sparks (dim thread + specks) · Bits (the run's 16-bit code) · Stutter (pixel-stretch blocks, 1/16 gap) · Fray (dashes with per-line phase → diagonal weave inside bundles) |
| K2 | Thresh | Sobel threshold 0..255 |
| K3 | Colour | Hold (edge colour) · Glow (video +192 luma, 2× chroma) · Negative (inverted video) · White · Rainbow (ROYGBIV per period) · Polarity (cyan rising / red falling) · Heat (black→red→yellow→white by intensity) · Tint (one random rainbow hue per filament) |
| K4 | Period | pattern pitch, ~2 px (left) to ~228 px (right) |
| K5 | Bundle | line-group size for the hash: 1, 2, 4 … 128 lines |
| K6 | Density | fraction of edges that fire (hash gate), 1/256 .. all |
| S7 | Ground | Black (filaments only) / Video (filaments over the source) |
| S8 | Motion | Frozen hash / Boil (reseed from LFSR every 4th field, ~15 Hz stutter) |
| S9 | Trigger | Both edge polarities / Falling only (bright→dark streams into the dark side) |
| S10 | Retrig | Restart at every edge (prism behaviour) / Hold through edges (longer, fewer) |
| S11 | Negative | invert the whole output (white ground, dark filaments = etching) |

Presets: Pixel Stretch (Stutter+Hold over video, falling edges, Hold retrig),
Data Burst (Bits+Tint, boiling), Ember Trails (Comet+Heat), Etching (Fray+White,
negative).

## Notes / ideas not built

- Drip: vertical strands falling from horizontal edges via a per-column line buffer
  (etchplate v3.1 machinery). Costs ~10 EBR at HD for {life, colour}; would give the
  livewire "material" feel a second axis.
- Mirror/leftward filaments need a two-pass line reversal (prism notes).
- Multiple concurrent filaments per line (a small slot table) so a new edge need
  not kill the previous run; Hold mode is the cheap stand-in.
- Boil cadence (every 4th field) and Glow strength are hand-picked constants.
