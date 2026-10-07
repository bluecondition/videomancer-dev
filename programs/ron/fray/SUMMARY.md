# Fray — edge-launched scanline filaments

Livewire's "edges as material" meets prism's rightward trail, rebuilt so that every
scanline gets its OWN filament instead of one even band. Edges are never drawn; they
are only the point where a filament launches. Streaming pipeline, no frame buffer:
filaments can only extend to the right (a leftward/mirror pass would need a
line-reversal buffer — see prism notes).

Status: **v0.4.2 FINAL except presets** (Ron signed off 2026-10-03). The cross-program colour bars are
not a Fray bug (downstream of FPGA logic) and are tracked separately.

## v0.4.1 / v0.4.2 (2026-10-03) — control ranges from HW

- K2 Thresh: above ~60% nothing fired. Full travel now = old 0..62.5%
  (thr = raw8/2 + raw8/8). Default 128 -> 205, presets x1.6, same effective thresholds.
- K4 Period: below ~30% the ~2-px dash pitch only aliased. Full travel now = old
  30..100% (step = 23200 - g*2.797 via two registered partial sums; 2.8 .. ~210 px).
  Default 900 -> 847; presets 900->847, 700->561, 300/200 -> 0 (bottom of range).

**Build seeds (v0.4.2):** all 6 configs seed 1 (HD Analog 85.2, HD HDMI 84.9, HD Dual
87.3, SD 80.1-85.0 MHz), router2, 5007 LC. Earlier: v0.4 HD Dual seed 2 (seed 1
timed out); v0.2 HD Dual seed 2.

## v0.4 HW result (2026-10-02): the low-pass did NOT fix the bars

Ron (monochrome source, settings Dash/Hold/Black/Period 0/Bundle 256/Density 896/
Restart/Length 512): bars unchanged by Period, saturated GREEN and CYAN, solid from a
garbled start to the right edge, ~6 lines tall, 1-4 per frame. On a mono source every
Fray chroma value is exactly 512 and no filament can exceed ~970 px at Length 50%, so
Fray's logic cannot draw them; the core path after the program (blanking gate, 4:2:2
packer) cannot make green from neutral either. Corruption is downstream of the FPGA
logic (TX pins / ADV7513 / HDMI link / sink). Solid green = YCbCr all-zero = what a sink
shows for blanking data, consistent with a false hsync/line break at the transmitter.
TX pin timing (hd_hdmi, nextpnr detailed report, TX samples on inverted forwarded clock
~5.5 ns): data/HSYNC margins > 2.5 ns on lineprobe and on fray seeds 1-6 -- no obvious
pin-timing fault. Next: Ron compares the analog component output (HD HDMI mode drives
BOTH encoders with the program output). The [1 2 1] chroma filter stays (correct for
4:2:2) but was not the fix.

## v0.4 (2026-10-02) — chroma low-pass (premise disproved, see above)

HW: solid colour-changing bars ~10 lines tall running right, very bad at K4 Period 0%,
gone above ~30%. Cause: the core sends 4:2:2 (U from one pixel, V from the next); at
Period ~2 px the Dash ink alternates colour/ground every pixel, so each pair carries U
and V from different colours = wrong hue, flipping every ~128 px as the 2.008-px period
slips against the pixel pairs; K5 bundles stack it into blocks. Line Probe (input
timing + stress screens) was clean, ruling out input timing and HDMI link. Fix: stages
17-18 apply a horizontal [1 2 1]/4 low-pass to U/V only (luma pixel-sharp), re-gated
at the output. Latency 16 -> 18. Build 6/6 (HD 86-88 MHz; HD Dual seed 2 after a
seed-1 timeout stall), 4911 LC.

## v0.3 (2026-10-01) — HW report on v0.2: black screen at startup

- Thresh rescaled: edge magnitude >>1 (was >>3). A vertical luma step of D codes
  reads 2·D, so K2's sweep now spans steps 0–50%; before, the default (16%) needed a
  31% step and anything above ~half the knob fired nothing.
- Hold over a BLACK ground samples the bright side of the edge (right of a rising
  edge); over video it keeps the object/left side. Rising-edge filaments were dark
  on black = invisible.
- Defaults: Pattern Dash (was Comet), Thresh 128, Period 900 (~15 px), Density 896
  (88%), Length 512 (50%).
- Build: 6/6 seed 1, HD 82–88 MHz, 4762 LC.

## v0.2 changes (2026-09-30, pre-flash audit against the hardware contracts)

- **Tail taper**: every filament fades out over its last 2^(msb(len)-1) pixels
  (a quarter to a half of the run, capped at 256) — `int = min(pattern, taper)`
  in a new stage 10 (latency 15 -> 16, all modes).
- **Heat** takes its palette index from that combined level, so it grades
  black -> red -> yellow -> white along every pattern (v0.1 Heat was identical to
  White on Dash/Morse/Bits/Stutter/Fray, whose intensity is always full).
- **Glow** luma cap 940 -> 768; **S11 Negative** luma now maps 64 <-> 768
  (`832 - y`, clamped) instead of `1023 - y` (black ground went superwhite 959).
- **Progressive line key**: the core holds field_n = '1' in progressive modes,
  so `aline & not field_n` was always even — K5's first two notches were
  identical (max bundle 64), the Fray pattern had 8 phases not 16, and 1080p
  lines 1024+ reused lines 0-55's hash. Now the frame-line key is the active
  line when progressive, the true frame line only when interlace is detected
  (a bottom field seen within the last two fields).
- **Field event** = avid absent for 2048 clocks (was: first hsync edge without
  avid — a doubled hsync on an analog source would have fired it mid-picture).
- **K4 Period / P12 Length**: deadband (2 LSB, ends exempt), glide 1/8 per field
  in 10.3 fixed point, and latched once per field. A 1-LSB wobble of K4 moved the
  far end of a 1000-px filament's pattern by half a period every frame.

v0.2 build: 6/6 timing-closed (HD Analog 81.0, HD HDMI 85.1, HD Dual 87.4 on seed 2
after a seed-1 miss; SD 81.7-85.5), 4776 LC (62%), 10 EBR. The first v0.2 netlist
closed HD at only 74.6 MHz: the deadband compare (SPI register -> two 11-bit
add/compares -> clock enable of the target) was the critical path, 9.5 ns of
routing. Registering the raw pot, one subtract with a top-bits |d|<=1 test, and a
registered "moved" flag restored 82-87 MHz across 6 seeds.


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
   chain; one latency (18) for every mode; blanking-gated at the output register.

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
| K2 | Thresh | Sobel threshold, full travel = luma steps 0..~31% |
| K3 | Colour | Hold (edge colour) · Glow (video +192 luma capped 768, 2× chroma) · Negative (inverted video) · White · Rainbow (ROYGBIV per period) · Polarity (cyan rising / red falling) · Heat (black→red→yellow→white along the tail taper × pattern level) · Tint (one random rainbow hue per filament) |
| K4 | Period | pattern pitch, ~2.8 px (left) to ~210 px (right) |
| K5 | Bundle | line-group size for the hash: 1, 2, 4 … 128 lines |
| K6 | Density | fraction of edges that fire (hash gate), 1/256 .. all |
| S7 | Ground | Black (filaments only) / Video (filaments over the source) |
| S8 | Motion | Frozen hash / Boil (reseed from LFSR every 4th field, ~15 Hz stutter) |
| S9 | Trigger | Both edge polarities / Falling only (bright→dark streams into the dark side) |
| S10 | Retrig | Restart at every edge (prism behaviour) / Hold through edges (longer, fewer) |
| S11 | Negative | invert the whole output (luma 64↔768: white ground, dark filaments = etching) |

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
- Boil cadence (every 4th field), Glow strength and the taper fraction are hand-picked constants.
