# Koch (Straightedge)

**v0.1.0 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/koch.vmprog`.

## What it draws

Helge von Koch's curve (1904) and the snowflake built from it, as drawn in a mathematics plate. Luma only
(U = V = 512 on every pixel); every edge on the screen is a straight line at a multiple of 60 degrees.

- View = Flake (default): the snowflake, 0.85 of the screen height, white-rimmed on black. Inside, bands of the
  fold distance to the curve: nested pointed stars stepping white, light, dark, black toward the centre. Outside,
  bold black / dark-grey echoes of the hexagram (the curve's first generation), a pointed star-burst out to the frame.
- View = Dive: an endless zoom into the flake's top point. Only the 60-degree corner and the two Koch curves
  leaving it are drawn; the figure is exactly self-similar at x3 about the tip, so the zoom wraps at exactly x3.
  Outside, plain chevron echoes of the corner.
- P12 Depth grows the generations: a plain triangle, then each generation's spikes rise out of the edges of the
  last as truncated triangles until they are whole.

The field is a fold cascade in "heights" (distances to straight lines; the barycentric coordinates of the
equilateral triangle on a segment), where every step is a sort, an add or a negation, so the 60-degree
reflections are exact and there is no per-pixel multiplier in the geometry:

    a1, a2, a3   inward heights of the base triangle (a1 + a2 + a3 = H0)
    front        (U, W) = (a_max + a_min, a_mid + a_min)            the nearest side's frame
    level j      hi, lo = sort(U, W); t = hi - 2/3 H(j-1); t < 0: (U, W) = (-t, lo + t), else (t, lo)
    Y            = H(n) - U - W after the n levels that are on (> 0 outside); the growing level is truncated at h

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | Flake: the flake's centre, +-960 virtual px. Dive: the tip, +-600. Glided |
| K2 | Y | +-540 / +-337 virtual px (Dive: about a point 160 px above the centre). Glided |
| K3 | Scale | Flake: side 199 .. 6360 virtual px, exponential (default 795 = 0.85 of the height); above the default the view slides toward the flake's right notch, so the top of the knob is zoomed well into the edge. Dive: a zoom offset of up to 3 levels. Glided |
| K4 | Bands | band period 16 .. 181 virtual px, exponential (default 91). Glided |
| K5 | Drift | bipolar speed (Hypnos law), still within 24 counts of the centre. Flake: the bands stream out of (right) or into (left) the curve, on both sides of it. Dive: the same stream plus the zoom into (right) or out of (left) the tip |
| K6 | Camera | a screen of little hexagram flakes, one per 32-px block, each sized by the block's luma. Default 100 % |
| S7 | Glide | Off: knobs act at once (2-count hold against ADC jitter). On (default): 1/16 of the gap per field on K1 K2 K3 K4 K6 P12, 1/8 on the Drift speed, landing exactly |
| S8 | View | Flake / Dive |
| S9 | Tone | Grey: four flat greys (inside white to black under a white rim, outside black / dark) / Ink: black and white, the bands XOR the flake's silhouette |
| S10 | Weight | Bold: equal bands / Fine: a line on every band edge (1/6 of the period, at least 2 px), light inside, dark outside, the rim the boldest line |
| S11 | Source | Video / Osc (the shell's drifting triangle lattice at twice the camera level) |
| P12 | Depth | Headline. The generations, continuous, on a square law (the big generations get the travel): 0 % = a plain triangle (Dive: the corner with one screen-sized generation), 50 % = the hexagram sprouting its second generation, 100 % = the full curve, as fine as the raster shows (about 3 px segments). Glided |

## Motion

One drift phase (Q18 cycles), stepped once per field by a signed speed from K5: zero within 24 counts of the
centre, then exponential from 2^-12 to 2^-3.9 cycle per field (capped there: a quarter cycle per field strobes).
Default K5 = 680: 332 / 2^18 per field, one cycle every 13 s at 60 Hz. The phase is advanced by the per-field
engine, which is started once per field at the vertical-blank zone ("no active video for 1024 clocks").
- Flake: one cycle = 4 bands of stream (the tone cycle), so the wrap is seamless.
- Dive: one cycle = one level of zoom (x3 about the tip) and 4 bands of stream. At the wrap every generation
  becomes the next one up and the finest one has just finished growing, so the fall never jumps. The triangle is
  kept large enough (side 2100 .. 6300 virtual px) that nothing that differs across the wrap is on screen.
RTL-checked: see Status.

## Camera

Changed from the spec (the block luma setting the depth, or the band phase): neither makes a subject readable
(depth only changes how fine the edge is; a phase step tears the bands). Tone steps and band weight per block
were also tried and dropped. What is built is a halftone made of the program's own primitive:
- luma (or Osc) -> gain table from K6 (dark areas below luma8 64 give nothing) -> 32-px box -> a vertical
  one-pole per block column -> sampled at the end of every 32 x 32 virtual-px block (60 x 34 blocks) as a star
  size 0 .. 15 px, 4 bits, into block RAM: one write statement, two banks swapped once per field at the zone
  start, so the bank being read is never the bank being written; power-up contents 0 = no stars.
- the next field draws one hexagram flake (Koch generation 1) at the centre of every block, its circumradius the
  held size (constant across the block; sizes under 3 px are not drawn). Grey: a white film. Ink: XOR. Fine:
  2.5-px outlines.
- 0 % or a black input = exactly the clean figure. 25 %: small stars in the bright areas only. 50 %: half size.
  100 %: bright areas are a dense screen of large stars, dark areas keep the clean figure, so the subject reads
  as a halftone behind the flake (the flake's own white hides the screen in Grey).
- Osc: the drifting lattice at twice the level: diamonds of large and small stars crossing the picture.

## Engine

- Per field (from the zone start): Hardline flute's microcoded engine (as Vellum trine): 37-bit accumulator,
  128 x 32 register file, 993-word microprogram, serial 18 x 18 multiplier. Knob holds and glides, the Drift
  speed and phase, the unit (u = P / 2^k of a pixel, u in [1/16, 1/8), so a band is 2^k LSBs and the band number
  and the distance to its edge are bit fields of the fold distance), the scale, the depth, the seven fold
  constants, the growing level's truncation, the beam's start values and steps. About 3,200 clocks, then a
  256-clock gain-table fill.
- Per line: s = a1 + a2 of the next line; the star screen's row.
- Per pixel (48 clocks): one beam DDA (p = a1 - a2), the front (3 stages), seven levels of sort / fold
  (2 stages each), the fold distance, band number and edge distance, coverage (two 7 x 6 multiplies by the
  LSB size), three blends (band, rim, stars), luma [1 2 1]/4, 13/16 toward black (white 777), blanking gate.
- Generated avid (the shell's parity-locked start), one latency (48) in every mode, virtual 1080-line
  coordinates (SD shows the centre 4:3), anti-aliasing in real pixels.

## Status

- v0.1.0 built 2026-11-17 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1. Not flashed.
- Post-route Fmax: HD Analog 86.98 MHz, HD HDMI 88.35, HD Dual 89.06 (HD needs 74.25); SD Analog 93.06,
  SD HDMI 88.21, SD Dual 85.93 (SD needs 27). Six bitstreams pass iceunpack.
- 6,504 .. 6,540 / 7,680 LC (85 %), 17 / 32 EBR. No carry self-adds (dupin: none).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge koch`.
- router2 history: two earlier netlists closed on seed 1 with thin HD margins (75.0 .. 81.9 MHz); on the final
  netlist router2 stalled on HD Analog seed 1 (over six minutes), so the build was switched to router1.
- Timing history: the first build missed at 66 MHz (the camera store's subtract, shift and add in one clock: split
  over two states); the star screen's coverage stage was split after it showed as the next critical path.
- Fit history: the first RTL accumulated Y level by level and delayed Z in flip-flops: 7,000 LC. Reading Y as a
  tap after the last level and moving Z's delay into a block-RAM ring brought it to 6,500.
- Model = RTL (GHDL against the integer model `.straightedge_work/rb_koch/imodel.py`; the per-field arithmetic
  is the microprogram itself through `koch.py`'s emulator): 0 mismatching pixels on every frame compared.
  720p / 2 (228,960 px each): defaults; bear.png at Camera 100 % (the model fed the tester's own stimulus luma);
  Osc; Dive; Dive + Fine + Depth 49 % + X / Scale moved; Depth 59 % with fine bands; Ink + Fine + Depth 78 %;
  Ink + Dive + bear.png at Camera 59 %; Fine + bear.png; X / Y / Scale moved in Ink; Depth 34 % at a small scale;
  Scale and Bands at maximum; Glide On at the power-up defaults with the default Drift (3 fields from power-up);
  two motion captures at the default Drift, 3 and 9 fields from power-up (the bands 6 x 332 / 2^18 of a cycle
  further out, as the model: outward for Drift right of centre); Dive at full Drift after 9 fields (zoom + stream).
  Full rasters: 720p decimation 1 (918,720 px each): defaults with a black input, Source = Osc, bear.png at
  Camera 100 %. NTSC (720 x 243 per field, decimation 1): defaults, the same picture as HD; Dive with bear.png
  matches once the model takes the camera grid from the previous field's lines (the tester's stimulus is
  interlaced, so the one-field-old grid comes from the other field; with the same field's lines 1.6 % of the
  pixels differ, all in star sizes).
- Range check of the model at 1080p over 360 knob / phase combinations: no 18-bit value wraps (worst 104,557
  of 131,072).
- Renders (work dir `.straightedge_work/rb_koch/`): `cmp_final.png`, `sheet.png` (control sheet), `strip.png`
  (8 frames across P12, Flake and Dive), `motion.png` (Drift), `camera.png` (bear.png at Camera 25 / 50 / 100 %).
- Hardware doubts: the star screen and the block-RAM camera grid are new (no hardware precedent); the per-field
  engine is flute's, restarted from address 0 at every field's zone; on interlaced SD, Fine lines under one field
  line (Bands near its minimum) can drop out on horizontal edges.

## Cut / deviations

- Depth grows by truncation (spikes rise as truncated triangles), not by blending two distance taps: no per-pixel
  multiplier, every edge stays at a multiple of 60 degrees, and the far field does not jump.
- The outside bands follow the hexagram (generation 1), not the full curve: the fold distance of the fine
  generations serrates every far band and throws thin spurs along the notch axes. In Dive the outside is the
  plain corner. The inside bands and the curve use every generation.
- Camera target changed to the star screen (see Camera). In Grey the screen is hidden where the figure is
  already white (the rim and the white bands).
- Dive: K1 / K2 are limited to +-600 / +-337 so that the zoom's wrap stays off screen. K3 is a zoom offset there.
- Drift tops out at 2^-3.9 cycle per field.
- Seven generations; the finest drawn has segments of about 3 virtual px.
- Anti-aliasing uses the horizontal pixel as its unit: on interlaced SD rasters edges are a little hard vertically.
- The first field after configuration is not valid (the source width is not yet measured).
