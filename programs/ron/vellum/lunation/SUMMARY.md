LUNATION (Vellum)
v0.1.0 built 2026-11-22, not flashed.  Flash out/rev_b/ron/vellum/lunation.vmprog.

WHAT IT DRAWS
The lunar-phase calendar, a wall chart of moons in rows with each moon a day older than the last (after Galileo's
wash drawings of the phases).  At power-up: four rows of seven moons on black, 28 discs lit from the same side.
They run new, crescent, first quarter, gibbous, full, then wane back to new, in reading order.  The lit part is
white and the dark part a dark grey (earthshine, 3/16).  Luma only (U = V = 512).  It is a flat print: an exact
circle, an exact half-ellipse terminator and three tones, with no craters and no shading.  The whole calendar runs:
every moon waxes and wanes, a day apart from its neighbour.

  cell pitch P, disc radius R = d P; moon (i, j) has phase phi0 + s n(i, j)
    Calendar: square grid, n = i + 7 j          Brick: odd rows offset P/2, n = 2 i + (j & 1) + 2 j (diagonals)
  w = sqrt(R^2 - v^2) per line (exact serial square root)
  limb cover = clamp(1/2 + (w^2 - u^2) / 2R)      (squares table per field, w^2 per line: smooth caps)
  terminator u = c w, c = cos(2 pi phi): lit u > c w waxing, -u > c w waning
  terminator cover = clamp(1/2 + kt (s u - c w)), kt = w / sqrt(w^2 + c^2 v^2) (half-ellipse slope, ROM)
  lit = max(0, limb + terminator - 1)  (a new moon has no lit rim; crescent horns taper to points)

CONTROLS
K1  X        Pan across, +-960 virtual px (moons keep their phases).  Default centre.  Glided.
K2  Y        Pan up / down, +-540 virtual px.  Default centre.  Glided.
K3  Scale    Pitch 640 .. 137 virtual px, exponential (3 .. 14 moons across).  Default 56 % (270.8 px: four
             rows fill the 1080 lines, seven columns centred).  Glided.
K4  Disc     Radius 0.14 .. 0.50 of the pitch (pearls .. touching discs).  Default 85 % (0.446).  Glided.
K5  Drift    Bipolar speed of phi0 (Hypnos v1.1.0 decode), still within 24 counts of the centre, 2^-12 .. 2^-5
             lunation per field.  Default +33 % (680): one lunation in about 16 s at 60 fields/s.
K6  Camera   Box-filtered luma bends each moon's terminator toward a halftone of phases (see CAMERA).
             Default 100 %; 0 % = the clean chart.
S7  Glide    Off: the knobs act this field (+-1-count hold).  On (default): 1/16 of the gap per field on K1 K2 K3
             K4 K6 P12, Drift speed 1/8, landing exactly.
S8  Form     Calendar (square grid, reading order) / Brick (odd rows offset half a cell, phases indexed along
             the diagonals: slanting waves).
S9  Tone     Grey (earthshine on the dark side) / Ink (the lit part only: new moons vanish).
S10 Weight   Bold (filled) / Fine (the astronomer's diagram: the limb and the terminator as lines R/14 wide,
             at least 2 virtual px; Grey adds a 5/16 film on the lit part).
S11 Source   Video / Osc (the shell's drifting 640 x 540 triangle lattice at full swing).
P12 Span     The phase step per moon, s = (27/56)(2^(5p) - 1)/31.  0 % = every moon alike (Drift and Disc stay
             live), 34 % (default 352) = 1/28: one lunation over the 28 moons, the classic chart.  66 % = about 1/7:
             every column one phase (vertical bands).  100 % = 27/56: full and new alternate in a checkerboard,
             with a slow standing beat that drifts through the wall.  Glided.

MOTION
phi0 is a Q20 accumulator (2^20 = one lunation), advanced once per field in the vblank zone by the signed
Drift step, so it wraps exactly one lunation and never jumps.  Every moon shares it (phase = phi0 + s n), so
all moons wax together, a day apart.  At Span 1/28 the pattern of phases flows back through the calendar in
reading order.  Speed: Hypnos law, 7 octaves 2^-12 .. 2^-5 lunation per field.  The top is capped at 2^-5
(one lunation in 32 fields): at 2^-2 a moon would change by a quarter cycle a field.  Nothing stops it but
Drift at centre.

CAMERA
Luma (or Osc) -> 64-px box (32 at 720p / SD) -> vertical one-pole IIR (32 .. 48 virtual lines) -> an 8-bit
copy of the line store, read by the cell engine at each cell's centre on each line (one line old).  For each
moon on each line:
    m = min(4 L, 1) x K6,   t = 1 - 2.2 L (lit area = luma),   c1 = c + m (t - c),
    c = clamp(c1, 2c - 1, 2c + 1)   (the band keeps a new moon new and a full moon full)
The terminator leans smoothly line by line and never breaks.  A moon passing new or full never flips its lit
side (the band closes there), so the camera is continuous in motion too.  Black input or K6 = 0 gives exactly
the clean chart.  K6 25 %: the terminators lean gently with the picture.  50 %: half-way to the luma's own
phase.  100 %: mid-cycle moons take the picture's phase (bright = fuller), new and full moons stay as they are.
At the default 28 moons bear.png reads as bright and dark masses and bending terminators, not as a
recognisable bear.  It is plainest at Span 0 with the moons near a quarter, or at a small Scale.
Osc substitutes the drifting triangle lattice at full swing: big diagonal waves of fullness.

ENGINE
- Per field (vblank zone, about 1,400 clocks): holds and glides (6 channels) and the Drift decode, then one
  serial 22 x 20 multiplier in 13 slots: pitch from a 2^(i/256) ROM, radius, R4^2, span, both pans,
  1/(2 R a), the first-cell phase.  One 33-step serial divider (2^32 / R4).  One shared reduction unit puts
  pixel 0 and line 0 into their cells, counting cells (the step decision is a sign bit).  Then the 1024-entry
  squares table (second differences) and the line-0 init.
- Per line (about 72 clocks on the same multiplier, computing the NEXT line): v += step, row wrap (parity,
  first-cell phase, pixel 0's u), v^2, w^2, w by a 13-step serial square root, the limb's w^2/(2R a), and
  rho = w/R.
- Per cell (28 clocks; cells are at least 68 pixels): phase -> quarter-wave cos ROM, the camera level at the
  cell centre, m, the band clamp, |c| w on a radix-4 right-shifting serial multiplier, kt ROM by (rho, |c|).
  At the end of each line it computes the next line's first two cells (56 clocks, inside the tester's 64-clock
  hblank and the hardware's 138+); inside a line, one cell ahead at each crossing.
- Per pixel (stages 3 .. 15): u DDA with cell crossings -> |u| -> squares RAM -> limb distance; u -+ |c w| ->
  |dt| x kt (8 x 6, the only pixel multiplier) -> terminator cover; ring and terminator line (Fine) -> lit =
  max(0, limb + terminator - 1) -> tone x -> luma ROM (64 + 876 x / 256) -> [1 2 1]/4 -> 13/16 toward black
  (white 776) -> blanking gate.
- One latency (24) in every mode.  Generated avid (parity vote).  Virtual 1080-line coordinates from the timing
  ID.  The AA unit is 1 px (1.5 px interlaced).  field_n gets one register only.  Chassis: Kinetic riley via
  Vellum rosette ([A] timing, [B] controls, [D] source field, [F] luma conditioning, dazzle's Osc line-0 fix).

STATUS
Built 2026-11-22 through the shared queue with router1, every config on seed 1 (post-route Fmax, last report):
  HD Analog  77.51 MHz   6839 LC   17 EBR
  SD Analog  80.14 MHz   6828 LC   17 EBR
  HD HDMI    89.52 MHz   6840 LC   17 EBR
  SD HDMI    87.45 MHz   6831 LC   17 EBR
  HD Dual    85.54 MHz   6843 LC   17 EBR
  SD Dual    82.86 MHz   6832 LC   17 EBR
89 % LC.  HD bitstreams pass iceunpack.  dupin: no duplicated inputs.  No emulate_transparency.  The GHDL
integer-minus-signed scan finds 0 sites.
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum lunation   (router2 was not tried)
Model = RTL (GHDL vs .vellum_work/rb_lunation/imodel.py), 0 mismatching pixels on every frame compared:
  - 720p60 decimation 1 (910,080 px compared, 8-px borders excluded): the power-up defaults with a black
    input (Glide On, Drift +33 %, field 3: this is rtl_default.png); Source = Osc (Drift still);
    bear.png at K6 Camera 100 % (Drift still; the model fed the tester's own stimulus luma).
  - 720p60 / 2: defaults (Drift still); Brick + Fine with X, Y, Scale, Disc and Span off their defaults;
    bear.png + Fine with X and Y off centre (the model fed the tester's own stimulus luma).
  - Motion: two captures of the default Drift 8 fields apart (fields 3 and 11), both 0 mismatching pixels.  The
    model one field either side differs on about 3,200 px, and fields 3 vs 11 differ on 5,919 px, so the travel
    and its direction (moons waxing, the pattern flowing back in reading order) are confirmed.
  - NTSC decimation 1, both fields (720 x 243 each): the same picture, the centre 4:3 of the canvas.
  No stepped knobs (the end-of-range hysteresis trap does not apply).

CUT / DEVIATIONS
- Camera: not a plain "adds to c".  A plain c - gL lets the chart's own full moons swallow the picture, and a
  moon the camera has moved off c = 1 flips its lit side when it passes new: a jump in motion.  Built instead:
  a pull toward the luma's own phase inside the band [2c - 1, 2c + 1], which closes at new and full.  It is
  still "adds to c", and continuous in space and time.  The subject is legible only as masses at 28 moons.
- P12 100 % = 27/56, not exactly 1/2: the checkerboard of full and new carries a slow beat, the "standing
  figures drifting through the wall".  At exactly 1/2 the wall would only breathe in place.
- Drift top capped at 2^-5 lunation per field, not 2^-2 (see MOTION).
- Ink Fine draws the limb and the terminator as the union of two lines, not XOR.  They are one diagram, and XOR
  ate the rim where the terminator meets the limb at new and full.  Ink Bold has no overlaps.
- The terminator's anti-aliasing slope kt comes from a 32 x 32 ROM by (w/R, |c|) and is quantised per line.
  The camera's own per-line variation of c is not in kt.  The limb is exact to the squares table's half-pixel
  index (+-1/4 px, constant along a moon: no wobble).
- The camera samples one line old at each cell centre.  The first cell of a line samples at pixel 0 when its
  centre is off-screen.
- SD shows the centre 4:3 of the canvas: five full moons per row plus slivers of two more (the 7-column chart
  is wider than 4:3).  720p and 1080p show the whole chart.  The same picture otherwise (NTSC checked in RTL).
- Fit: the first RTL was 7,712 LC (100 %); the shipped one is 6,840.  Wide shifting multipliers became
  right-shifting ones, the coverage clamps lost their adders, a magnitude-form terminator path, one reduction
  adder, a luma ROM, latency 28 -> 24.  Nothing visible was cut.
- Not verified on hardware: anything.  The per-line / per-cell engines and the line-start scheduling in the
  hblank are new on this chassis.  HD Analog has the least margin (77.5 MHz, 4.4 %).
