# Pendula (Kinetic)

v0.1.0 built 2026-11-14, all six configs, not flashed. v0.1.1 (2026-11-15): K6 camera pass, see the end.

## What it draws

A lattice of round dots, one per cell, each riding its own small orbit about a gold rest mark, phased by a
travelling wave. The art it comes from: pendulum-wave machines, the Tusi-couple illusion (dots each moving on a
straight line that together read as a rolling ring), and the particle orbits of a water wave. At the defaults
(Bauhaus palette) about 16 x 9 dots on cream swing on flat ellipses, bunching and thinning as a compression wave
that travels across the screen, each dot sliding from red to blue and back with its phase.

## Element and state

- Element: the dot of one lattice cell of pitch P; cell 0 is centred on the screen centre, so Scale zooms about
  the centre. Hex offsets alternate rows by half a pitch (same row spacing).
- State: the dot's displacement on its orbit. The orbit is an ellipse with semi-axes ax, ay turned by psi,
  computed as two counter-rotating circles:
  (dx, dy) = a (cos, sin)(theta + psi) + b (cos, sin)(psi - theta), a = (ax + ay) / 2, b = (ax - ay) / 2.
- theta = phase - Spread x n, n = the cell's index in quarter cells.
  - Linear: n = 4 i + 2 j (the front is tilted: two rows per column).
  - Radial: n = 2 max + min of (|2 i|, |2 j|): the octagonal distance from the centre cell.
- Snap: theta dwells at 0 and at 1/2 (the two ends of the orbit's long axis) and flips in the last quarter of
  each half cycle (75 % dwell; the flip is eased because position = cos of a linear angle).
- Axis Fan: psi = n / 32 turn (1/8 turn per column). With a line orbit, at Phase 50 % the phase step equals the
  axis step and the picture is the Tusi figure: every dot on its own straight line, the field rolling as one.
- The dot never leaves its cell: orbit semi-axis A <= 69 % of (P/2 - 1), dot radius <= Rlim = P/2 - 1 - A, so one
  dot test per pixel is enough.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 32 .. 256 virtual px, exponential, glided (default 120: 16 x 9 dots) |
| K2 | Colour | the 16 shared palettes with hysteresis (was K6): ground, A / B = the dot by phase, accent = rest mark |
| K3 | Depth | orbit size 0 .. 69 % of the half cell, square law; the dots shrink to keep inside their cells |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Size | dot radius 25 % .. 87 % of the room the orbit leaves (Rlim) |
| K6 | Camera | camera strength 0 .. 100 %, default 50 %: the held luma (and the Osc lattice) x 0 .. 3 with a soft knee at white; 25 % = the v0.1.0 picture (glided, +-4 deadband) |
| S7 | Wave | Smooth / Snap (dots dwell at the ends of their orbit and flip across) |
| S8 | Field | Linear (the wave crosses the screen) / Radial (octagonal rings from the centre) |
| S9 | Lattice | Square / Hex (alternate rows offset half a pitch) |
| S10 | Axis | Same (every dot on the same orbit) / Fan (the orbit's axis turns 1/8 turn per column; the colour then follows theta + psi) |
| S11 | Source | Video / Osc |
| P12 | Phase | the phase step per column, 0 .. 1/2 cycle, square law, glided: 0 % unison .. travelling compression / transverse waves .. 100 % neighbours in antiphase (aliased standing figures) |

Presets: Compression (defaults), Pendulum Wave, Tusi Rings, Water Orbits, Ripple Tank, Flip Dots, Halftone.

## Video

- Source = Video: luma (>> 2, less 16) smeared along x by a 1/16 one-pole filter and HELD per cell, so every dot
  stays a rigid circle. It drives the dot's SIZE (a second property, not the orbit): radius^2 = R0^2 + luma x
  (Rlim^2 - R0^2), so dot area follows the picture from the Size radius (black) to the largest dot the cell
  allows (white): a moving halftone.
  - On the last line of each cell row the smear is sampled at the centre of every cell of the row BELOW into a
    256-word block RAM (two banks by row parity x 128 columns); each pixel reads its own cell's word. The top
    row of the field is sampled on the field's first line.
- Black input: luma 0, every dot at the Size radius, the wave alone runs.
- Source = Osc: a two-axis triangle lattice (640 x 540 virtual px) drifting diagonally replaces the luma at
  full scale (0 .. 87 %), so dot sizes swell and shrink in a slow drifting lattice.
- The picture does not colour the dots (not in the spec; no video delay ring).

## Engine

Chassis from louver.vhd (sections [A] .. [F] marked in pendula.vhd):

- [A] Timing ID -> virtual-1080 steps; vblank zone (no avid for 1024 clocks); source width; glides on Scale and
  Phase; per-field sequencer with one 24-step serial multiplier (10 products, 32 clocks apart: pitch, orbit axes,
  Rlim^2, R0^2, spread, rate), the palette read, two origin searches (x, y), and an adder-only fill of the
  512-word blend table T(mark coverage, colour mix, dot coverage), 3 bits each with weights 0 1 2 3 5 6 7 8
  eighths.
- [B] Generated avid (parity-locked start 3 / 4 clocks after the source's, field-latched width).
- [C] Source: Osc lattice, smear, per-cell hold RAM.
- [D] Lattice: y accumulator per line, x accumulator per pixel (Q8 virtual px, wrap flags a step ahead), Hex
  offset applied per pixel; cell index n; theta = phase - Spread x n as a gated-sum multiply modulo one cycle
  (16 bits per cycle); Snap warp; the two circle angles (11 bits per turn).
- [E] |cos|, |sin| table (512 x 16, two read ports) -> four 8 x 8 gated-sum products -> displacement in half
  pixels -> pixel minus dot centre -> table of squares (two ports) -> r^2 -> R^2 - r^2 -> shift by the leading
  bit pair of R^2 (a 1 .. 2 px ramp) -> 8 coverage steps. Rest mark: an octagon (max + min / 2) about the cell
  centre, radius P/32 + 2.5 px, 8 steps. Colour = one read of the blend table; no per-pixel colour multiplier.
  Luma 10 bits, chroma 8 bits until the output filter.
- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums with the per-line resync, luma [1 2 1]/4
  delayed to the chroma's group delay, luma 13/16 toward black, blanking gate.

One latency in every mode: C_LAT = 36 = processing_delay_clks (29 pixel stages + output stages + 4 px of filter
delay). Per field: every knob-derived constant, the blend table, the phase step, the lattice origin. Per line:
the row's local y, row number, parity. Per pixel: everything else. Every multiply is a sum of gated operands
with a stage per adder level; block-RAM data gets a fabric register before arithmetic; no_rw_check on every RAM
(0 emulate_transparency cells).

## Status

- Built 2026-11-14 with router1; rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic pendula`.

| Config | Router | Seed | Post-route Fmax | LC |
|---|---|---|---|---|
| HD Analog | router1 | 1 | 92.40 MHz | 6562 |
| SD Analog | router1 | 1 | 97.28 MHz | 6556 |
| HD HDMI | router1 | 1 | 98.66 MHz | 6558 |
| SD HDMI | router1 | 1 | 98.17 MHz | 6551 |
| HD Dual | router1 | 1 | 95.14 MHz | 6564 |
| SD Dual | router1 | 1 | 94.17 MHz | 6561 |

- 6,564 LC (85.5 %), 21 of 32 block RAMs. Built with
  `NEXTPNR_ROUTER="--router router1" .kinetic_work/tools/build_queue.sh kinetic pendula` (dupin.py: 0 CARRY I0==I1).
- router2 history: the first RTL (before the Fan colour term) closed all six on seed 1 with router2 (HD Analog
  92.40, HD HDMI 91.93, HD Dual 87.54 MHz). The final RTL stalled router2 on HD Analog on seeds 1 and 2
  (`overused=2` never clearing), so use router1 for rebuilds.

- GHDL render = integer model (.kinetic_work/rb_pendula/imodel.py):
  - 720p60, decimation 1 (1280 x 720), defaults, black input, final RTL: 0 mismatching pixels of 918,720
    (the capture sits 4 columns right: the generated avid's start; aligned before the diff).
  - 720p60, decimation 2 (640 x 360 window), final RTL: Fan + Hex + moved knobs 0; Source = Osc 0;
    Video = selfie.jpg 0 (all of 228,960).
  - Same window, the RTL before the last change (the Fan colour term): defaults 0; Snap + Radial + Hex + Fan +
    Osc + six moved knobs 0. Motion: each diff matches only the model's frame 2 (phase = 2 steps).
  - Not rendered: SD (NTSC), 1080i, 1080p. The lattice uses the chassis' virtual-1080 steps unchanged.
- Never flashed.

## Cut / deviations

- Dot positions are quantised to half a virtual pixel (the table of squares takes half-pixel lengths), so at
  the very slowest rates a dot moves in half-pixel steps. Trig is 8 bits over 2048 steps per turn.
- The edge ramp is 1 .. 2 px wide (the division by 2 R is a shift by the leading bit pair of R^2); 2 .. 4 px at SD.
- Linear's index is 4 i + 2 j (a tilted front), not a pure diagonal i + j; Hex keeps the square row spacing.
- The dot's colour slides A .. B with its phase in 8 steps (an addition to the spec: it makes the wave read in
  a still). The rest mark is the accent colour; no rod.
- Source only GROWS the dot (black = the Size radius), and only up to Rlim, so at Size 100 % the picture's range
  is 87 % .. 100 % of Rlim, and at Depth 100 % all dots are small (Rlim is 31 % of the half cell).
- The hold samples ONE line per cell row (the last line of the row above), 16 px of smear to its left; the
  field's first line shows the top row with the previous field's sizes until its own samples land (one line).
- Only Scale and Phase are glided; Shape, Depth and Size read the knob once per field.
- The smear is 16 real pixels at every resolution (twice as wide in picture terms on SD).

## K6 camera pass (2026-11-15)

v0.1.1. K6 is now Camera: how strongly the held camera luma grows the dots.

- The camera drives the dot SIZE, and in v0.1.0 white already grew the dot to the largest that stays inside its
  cell, so more strength can only be contrast gain: the 8-bit held sample x g, with a soft knee (values up to
  223 unchanged, 1/4 slope above, 255 from 348) so the highlights roll off instead of flattening.
- Gain law: g = K6 / 256 up to 50 % (x0 at 0 %, x1 at 25 % = exactly the v0.1.0 picture, since the v0.1.0
  white, 219, sits under the knee; x2 at 50 %, the default), then 1 + K6 / 512 up to x3 at 100 %. 0 % draws
  exactly what a black input draws (every dot at the Size radius). Osc gets the same gain (it had no extra gain).
- x4 at 100 % was tried and rejected: on selfie.jpg it turned nearly every dot into the largest dot (the
  picture washed out to a uniform lattice). At x3 the bright face is mostly full dots and the darker parts still
  show; at x2 the dot sizes still vary clearly.
- K6 is read once per field through the shared glide (a third slot: 1/8 of the gap per field, +-4 deadband),
  turned into the gain in the vblank sequencer (zc 21), and applied by ONE gated-sum multiply (3 stages) plus the
  knee (1 stage) on the smeared sample BEFORE it goes into the hold RAM (8-bit words, unchanged). The written sample
  is the smear 3 px before the write point (16 px smear): invisible.
- Frozen: K2 Shape at 128 (its default: a flat horizontal ellipse, the Compression look) - the C_KFIX_2
  constant. K2 is now Colour (all 16 palettes, unchanged). Shape was the assigned knob and the v0.1.0 sheet agrees
  it is the subtlest in a still (its 0 / default / 100 % panels differ least). It does matter in motion: the
  presets Pendulum Wave (vertical line orbit), Tusi Rings (horizontal line, the Tusi figure with Fan) and Water
  Orbits / Ripple Tank (circles) now all draw the default flat ellipse; their colours moved from K6 to K2 and
  Camera is at its 50 % default in all of them.
- GHDL look (720p60, decimation 2, selfie.jpg, glides bypassed in a sim-only copy): .kinetic_work/rb_pendula/k6/
  k6sheet.png (K6 0 / 25 / 50 / 100 %, Osc at 50 %, v0.1.0 Osc); the rejected x4 law's 75 % / 100 % renders are in
  .kinetic_work/rb_pendula/k6/x4law/.
- Build (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic pendula`): router1, every config on seed 1. Post-route
  Fmax HD Analog 92.07, SD Analog 87.37, HD HDMI 95.68, SD HDMI 98.25, HD Dual 95.65, SD Dual 94.32 MHz (worst HD
  92.07). 6,802 LC at most (88.6 %, +238 over v0.1.0), 21 of 32 block RAMs (unchanged). vmprog:
  out/rev_b/ron/kinetic/pendula.vmprog. Not flashed.
- Unsure on hardware: the 50 % / 100 % levels were judged on one bright test photo (selfie.jpg fills the frame
  with a lit face); a darker camera picture will look less saturated at the same setting. Also whether losing the
  orbit-shape knob is acceptable for the line / circle presets (see above).

## K6 top +25 % and default 100 % (2026-11-15)

v0.1.2. Ron's rule after the K6 = 100 % renders: bend / element programs get a 25 % stronger camera at the
top of K6; picture programs keep their logic. The K6 default is now 100 % (initial 1023).

- **Classification: element.** The camera grows the dots: K6 is the contrast gain on the held luma that
  sets each dot's size.
- **Change:** c_cg = c_cg + c_cg / 4 at a new zone clock 23, after the two-segment law at clock 21. The
  gain is 1.25 x at every position (0 .. 958 in Q8, 10 bits as before):
  - 20 % = v0.1.0;
  - 50 % = x 2.5;
  - **100 % = x 3.74** (was x 3).

  The product reaches 954 at most, inside the 10-bit cm3_x, and the soft knee is unchanged.
- **Look check:** `.kinetic_work/rb_pendula/k6b/k6b_cmp.png` (GHDL, 720p60 / 4, bear.png, glides snapped).
  The crop is a bright area, where the dots were already at the largest size at x 3, so almost nothing
  changes (1.2 % of pixels). No wrap or garbage.
- **Doubt:** the K6 camera pass rejected x 4 at 100 % because a bright picture became nearly all full dots.
  The new x 3.74 is close to that, so the top of K6 may wash a bright camera picture toward a uniform
  lattice on hardware.
- **Build** (2026-11-15, `.kinetic_work/tools/k6build.sh kinetic pendula`): router1, every config on seed 1.
  Post-route Fmax HD Analog 83.13, HD HDMI 96.95, HD Dual 96.23 MHz (worst HD 83.13); SD Analog 98.34,
  SD HDMI 95.17, SD Dual 94.45. 6,796 .. 6,803 / 7,680 LC (88.6 %), 21 / 32 EBR. vmprog:
  out/rev_b/ron/kinetic/pendula.vmprog (v0.1.2). Not flashed.
