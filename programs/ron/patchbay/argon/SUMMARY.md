# Argon: a neon sign of pure geometry

v0.1.0 built 2026-11-22, not flashed. Patchbay (wave 2). Presets are hand-written.

## The patch

    H ramp^2 + V ramp^2 (circle generator) -> - R^2 -> x 1/2R               = the ring's distance field
    H, V ramps -> 3 rotators -> analog max                                    = the triangle's distance field
    H, V ramps -> 3 rotators -> full-wave rectifiers -> analog max            = the hexagon's distance field
        (S7 Square: 2 rotators + rectifiers each = a square and a diamond)
    Motor (K2) -> the rotators' angles: triangle +w, hexagon -3/4 w            (the two counter-rotate)
    camera luma -> cell means (128 virtual px) -> x K6 -> vertical one-pole lowpass -> subtracted from all three fields
        (S11 Osc: a drifting signed triangle lattice in its place)
    each field -> |.| (S8 Double: | |.| - gap |) -> window comparator + glow lowpass
        = a per-tube light table: white core ramp (K5, P12) + gas-coloured body + exponential halo (K3, P12)
          + wide wall wash (S10)
    gas colour (K4: a lerp round five gases, a different gas per tube) x Ballast brightness (S9: LFSR + sequencer)
    LFO (9 s) -> halo level                                                   (the glow breathes)
    three tubes -> additive RGB mixer -> per-channel shoulder (white-hot cores) -> Y / U / V

## What it draws

On black, three nested glass tubes lit from inside: a great ring 990 lines across, an equilateral triangle
inscribed in it (its vertices ride the ring) and a hexagon inside that, whose vertices just reach through the
triangle's sides when they line up. Every tube has a white-hot core, a saturated body in its own gas colour
(argon blue-violet ring, neon red-orange triangle, mercury green hexagon at the default Gas) and an
exponential halo that falls away into the dark; where tubes cross the light adds (orange + green = a yellow
spark, ring + triangle = white flares at the vertices). A faint wide wash of colour lies on the wall behind.
The triangle turns clockwise at about 10 degrees a second and the hexagon the other way at 3/4 of that, so
their crossings never repeat; the halos breathe on a 9-second LFO.

## Controls

| Control | Module | What it does |
|---|---|---|
| K1 Size | rotator gain (the polygon nest) | the triangle + hexagon scale against the ring: x0.35 (a small spark in the middle) .. x1 (inscribed, default 512) .. x1.6 (the triangle overlaps the ring, the hexagon fills it); the ring follows at 0.12 of the octaves |
| K2 Motor | rotator LFO | counter-rotation rate, bipolar, +-8 deadband, square law: full = 180 degrees/s; default 640 = 10 deg/s; left reverses both |
| K3 Glow | glow lowpass | halo length 5 .. 190 px (exponential, default 28 px), its level compensated (0.4 octave per octave) so the wide bloom does not wash out |
| K4 Gas | gas wheel | sweeps continuously round five gases (a lerp between neighbours: argon blue-violet, pink, neon red-orange, mercury green, white); the tubes sit 0 / 2 / 3 gases apart, so every position is a different set; 0 = argon / neon / mercury |
| K5 Tube | core window | core half-width 1.6 .. 17 px (exponential, default 4.4 px); the body widens with it |
| K6 Camera | camera VCA | camera strength, linear: 0 = the black-input picture (a 4-count deadband), 25 % = 30 px of bend per full-white, 50 % = 60, 100 % (default) = 120 |
| S7 Figure | rotator set | Round: ring, triangle, hexagon / Square: ring, square, diamond |
| S8 Tubing | rectifier | Single / Double: every tube becomes two parallel tubes (2.2 x core + 10 px apart) |
| S9 Ballast | LFSR + sequencer | Steady / Flicker: one tube at a time stutters for 0.5 - 1 s (random levels, drop-outs), then a pause |
| S10 Wall | wash | Dark (tubes on black) / Wash (default: the wall lit by a 280-px coloured spill) |
| S11 Source | camera input | Video / Osc |
| P12 Voltage | ballast voltage | core width x2^(+-1), core light, halo length x2^(+-1.5) and halo level: 0 % = barely struck (thin, deep-coloured, dim lines, an occasional one-field dip); 50 % = a clean full-brightness sign; 100 % = over-voltage: fat white cores and a bloom that fills the figure with mixed colour while the corners stay dark |

Presets: Argon (defaults), Barely Struck, Over Voltage, Square Pink, Double Bloom, Pinpoint.

## Video

Video (default): camera luma ((Y >> 2) - 16) is averaged over cells of 128 virtual px (96 at 720p; 15 / 20 /
11 cells per line), scaled by K6, run through a vertical one-pole lowpass in a cell store, and read back as
straight segments between the cells; the result is subtracted from all three distance fields, so every tube
swerves outward round bright parts of the picture. The heavy filter keeps the bends clean curves at
100 % (8-px boxes left the tubes ragged on selfie.jpg). With a black input the term is zero: the full picture.
Osc: a drifting triangle lattice (320 x 270 virtual px), signed (+-127 instead of 0..219), through the same
cells and gain: about +-70 px of wobble at 100 %, so the tubes become wavy loops (an unmistakable flip).

## Engine

- The patch computer: a microcoded engine (argon.py assembles it): one serial 32 x 16 multiplier, one ALU
  adder, a 1-bit shifter, a 256 x 32 register file in block RAM (its power-up image holds the constants and
  the persistent phases), 233 instructions of 32 bits (op, d, a, b and a 4-bit shift code). Ops: MULU,
  MULS, ADD, ADDR, SUB, SUBR, MIN, MAX, SEL, EXP2, SIN, IN, LDX, SYNC, END. It restarts from PC 0 at every
  vblank-zone start (no avid for 1024 clocks).
  - Part 1 (vblank zone, about 2,100 clocks): Motor phases; four edge angles; per edge the pixel step, line
    step, wrap (-N pixel steps) and seed (Q13 virtual px); the ring's quadratic DDA (K = 1/2R from the
    exp2 ROM; D0, DD and the wrap -N DD in Q28; F0, G0, GG for the line DDA in Q20 / Q28); the apothems;
    the double-tube gap; the camera gain. SYNC: the bus sequencer loads the constants and seeds the
    accumulators.
  - Part 2 (during the field, about 2,500 clocks): the table constants: core edge and ramp width, body and
    halo widths, light levels from Voltage, the breathing LFO, gas colours (a lerp on the wheel) x each
    tube's Ballast level.
- The table writer (after END, about 130k clocks): for each of 256 entries (compander bins: 0.5 px to
  64 px, 2 px to 192, 8 px to 448, 32 px to 1472) the core ramp, a gaussian body, the exponential halo and
  wash (through the exp2 ROM), lc = (vb body + vh halo + vw wash) >> 10, then per tube R, G, B =
  (Vc core + colour lc) >> 10, written into the back bank of three 512 x 32 tables; one serial multiplier
  ([acc | B] in one shift register: P = acc + A B). Banks swap at the zone start when it has finished.
  It reads its constants from the register file between the bus sequencer's turns.
- Ballast block: a 16-bit LFSR stepped per field, an episode / gap sequencer.
- Per line: in hblank the bus sequencer adds each edge's wrap and line step and the ring's D wrap (5 x 4
  clocks); at the line start the ring's f loads from the line DDA (F0 += G >> 8, G += GG).
- Per pixel (24 stages): four 26-bit edge accumulators (the triangle's third edge is -(u0 + u1), the
  hexagon's w1 - w0; Square repeats the first) and the ring (f += D >> 8, D += DD, two clocks behind);
  |.| and the max of three; the apothems and the camera term; |d| (and | |d| - gap |); the compander; three
  table reads; the RGB mixer; the shoulder (identity to 0.75, slopes 1/4 and 1/16, flat at 1.0);
  Y' = (5R + 9G + 2B)/16, Y10 = 64 + 2.28125 Y', U10 = 512 + 1.656 (R - Y'), V10 = 512 + 1.3125 (B - Y');
  compose; the shell's output conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the
  per-line resync, luma 13/16; the 8-sample delays and the hsync / vsync delay in block-RAM rings) and the
  blanking gate. C_LAT 24 = processing_delay_clks in every mode.
- Camera block: cell sums of (Y >> 2) - 16 over 128 px (1080) or 64 px (720p, SD), the mean x gain (a
  9-step serial multiply), a vertical one-pole lowpass in a cell store (1/32 per line at 1080p and 720p,
  1/16 at 1080i, 1/8 at SD; the first line of a field seeds it), read back per pixel as straight segments
  between cell anchors.
- Shell (heirloom lozenge, via limelight's copy): generated avid, vblank zone, width measurement, P12 glide,
  Osc lattice, output conditioning, one latency. Virtual 1080-line coordinates; colours in the hardware
  convention (U = Cr, V = Cb).
- Block RAM (24): glow tables 12, register file 2, microcode 2, sine 2, exp2 1, camera store 1, output
  rings 4.

## Status

- v0.1.0, built 2026-11-22 with ROUTER1 (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1.
  Not flashed.
- Post-route Fmax (the builder's last "Max frequency" line per config, each passed on seed 1):
  - HD Analog 75.71 MHz, HD HDMI 77.85 MHz, HD Dual 77.12 MHz (Fmin 74.25)
  - SD Analog 80.85 MHz, SD HDMI 85.19 MHz, SD Dual 77.43 MHz (Fmin 27)
- 6,900 .. 6,971 / 7,680 LC (91 %), 24 / 32 block RAMs. iceunpack OK on all six bitstreams.
  hd_clock_divisor untouched.
- Output: out/rev_b/ron/patchbay/argon.vmprog. Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh patchbay argon
- The HD margins are small (HD Analog +1.5 MHz). router1 is deterministic, so the same sources rebuild to the
  same result; if a later change tightens timing, look first at the patch computer's ALU and the pixel
  path's shoulder / compander stages.
- A first build (2026-11-22, 03:05) failed placement at 9,660 LC (125 %): the EXP2 ROM had been read in an
  if / else (two copies as LUT logic, ~900 LUT) and the output conditioning's delay chains and the camera /
  writer / CPU registers were heavy. The restructure is in the work dir's notes.md (Fit).
- Verified in GHDL against the integer model (.patchbay_work/rb_argon/imodel.py, which runs the same microcode
  through argon.py's emulator), 0 differing pixels in each case:
  - 720p60 (decimation 1), defaults, Source Video with a black input (rtl_default.png)
  - 720p60, K1 200, K2 900, K3 800, K4 300, K5 700, P12 900 (mid-glide), Square, Double, Flicker, Wall Dark
  - NTSC (decimation 1), both fields (two consecutive field counts: the motion advances field by field)
  - 1080i 59.94 (decimation 1), both fields
  - 720p60, defaults, a second capture five fields later (field 8): the counter-rotation and breathing match
- Osc (720p60): the same structure as the model, 53 % of pixels differ by a mean of 0.7 codes (the model's
  lattice phase and the camera block's segment timing are approximate). selfie.jpg at K6 100 %: mean 1.1
  codes from the model; the tubes swerve in clean curves with a few small kinks where a bright edge crosses
  a cell boundary.
- The sim PNGs sit 4 px to the right with 4 black columns at the left (the tester crops on the source avid;
  the generated avid starts 4 clocks later). On hardware the output avid is the generated one.
- Not simulated: 1080p, 720p50, PAL / 576p / 480p (same per-mode tables as the simulated modes).
- Work files: .patchbay_work/rb_argon/ (cmp_final.png, sheet.png, strip.png, camera.png, presets.png,
  proto.py / proto_rgb.py / proto_hw.py, imodel.py, notes.md).

## Cut / deviations

- RGB light model instead of the spec's Y/U/V tables: the tables hold R, G, B per tube, the mixer adds RGB
  and a per-channel shoulder turns the cores white. A Y/U/V knee washed P12 100 % and Glow 100 % to white;
  the shoulder keeps the bloom coloured. Same block RAM, about 150 LC more.
- Ballast strike is a per-tube brightness folded into the table constants (the whole table is rebuilt each
  field), not a bank select; flicker therefore shows one field after the LFSR decides it.
- The tables are rebuilt every field from scratch (ping-pong banks) instead of a few entries per field.
- Polygon corners are mitred (the max-of-edges field), so the halo runs further along a vertex's bisector:
  the triangle's vertices throw short flare streaks. Kept: it reads as lens flare.
- Motor drives a continuous counter-rotation (no separate sway LFOs); at the Motor deadband the polygons
  stand still while the halos keep breathing.
- The ring's distance is (r^2 - R^2)/2R: exact at the tube, so its halo is wider inside than outside (the
  bloom fills the figure, the corners stay dark), as the spec intended.
- Camera: cell means over 128 virtual px (96 at 720p) read back as straight segments between cells, so the
  bend field is a smoothed picture displaced by up to half a cell; invisible, since the picture itself is not
  shown. The bends kink slightly at cell boundaries where a hard bright edge crosses.
- Glide on P12 only (the shell glided six knobs): K4 and K6 are read raw each field (K6 has a 4-count
  deadband so 0 % is exactly the black-input picture); this freed about 120 LC for fit.
- Each figure's third edge is formed from the other two (triangle -(u0 + u1), hexagon w1 - w0) instead of a
  third accumulator; identical to within 1/8 px.
- Resolutions: 720p and 1080 show the same picture. SD shows the centre 1440 virtual px (4:3), so the ring
  and figure are whole and round but the side halos are cut sooner. Interlaced modes repeat one field (the
  core gives no field flag).
