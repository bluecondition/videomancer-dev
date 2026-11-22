# Steiner (Vellum)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/vellum/steiner.vmprog`.

## What it draws

The Steiner chain and Steiner's porism, drawn like a Japanese sangaku tablet (circles packed between circles, in
ink, hung in temples): a great circle nearly the height of the screen, a smaller circle inside it and off centre,
and between them a closed chain of seven discs, each touching both circles and both neighbours. The chain turns:
every disc swells as it comes round the wide side and shrinks past the narrow side, the eccentricity axis drifts
slowly the other way, and every tangency holds the whole way (the porism). Grey: black outside, the gap dark grey,
the inner disc mid-grey, the chain discs light in four steps by size (the largest white).

    unit great circle, eccentricity a, chain phase al (turns):
    s = sin(pi / n), rho0 = (1 - s) / (1 + s), r0 = (1 - rho0) / 2, h = 1 / s       (the concentric chain)
    disc k   E = 1 + 2 a cos al_k + a^2,  r = (1 - a^2) / ((1 - a^2) + h E),  al_k = al + k / n
             centre (1 - r) (((1 + a^2) cos al_k + 2 a) / E, (1 - a^2) sin al_k / E)   (the disc map (z + a) / (1 + a z))
    inner    through (rho0 + a) / (1 + a rho0) and (a - rho0) / (1 - a rho0)
    per pixel  P = |z - c|^2 - r^2 + C (C = the camera), edge distance P / 2r over one pixel

The integer engine holds every tangency to 0.29 virtual px or better (300 random knob sets x 1080p / 720p / NTSC).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the great circle's centre, +-960 virtual px |
| K2 | Y | the same, +-540 virtual px |
| K3 | Scale | the great circle's radius 227 .. 1100 virtual px, exponential (default 500; the top runs round the whole screen) |
| K4 | Count | n = 3 .. 12 discs (stepped, 8-count hysteresis, both ends reachable; default 7) |
| K5 | Drift | the chain circulates, bipolar: +-24 counts still, else 2^-12 .. 2^-5.8 turn per field; the eccentricity axis turns the other way at 1/4 of the rate; default 680 (one turn in about 19 s at 60 Hz) |
| K6 | Camera | 0 .. 100 % (default 100 %): the luma is added to every rho^2, so the discs dent away from the light (small ones vanish in bright areas) and the great circle bends with them |
| S7 | Glide | On (default): X, Y, Scale, Camera and Eccentric ease 1/16 of the gap per field (at least half a count, landing exactly), the Drift speed 1/8; a knob has to move 2 counts. Off: this field's knob values |
| S8 | Form | Chain / Nested: a second Steiner chain inside the inner circle (same eccentricity), turning the other way, its axis turned the other way; the 12 chain slots are shared, so Count steps through (outer, nested) = (3,3) (4,3) (4,4) (5,4) (5,5) (6,4) (6,5) (7,3) (7,4) (8,3) |
| S9 | Tone | Grey (gap dark, inner disc mid-grey, discs light by size; Nested discs on the inner disc's grey) / Ink (white discs and inner disc inside a white frame line, black and white, overlaps XOR: Nested cuts its discs black out of the inner disc) |
| S10 | Weight | Bold (discs) / Fine (every circle a thin white line, 4 virtual px; Grey: over a dark wash of the great circle and the inner disc) |
| S11 | Source | Video (default) / Osc (the shell's drifting triangle lattice, at twice the table gain) |
| P12 | Eccentric | a = 0 .. 0.85: 0 % = a symmetric ring of equal discs (still turning), 50 % = the default, 100 % = one enormous disc and a string of pearls round the narrow side |

## Motion

The chain phase is a 24-bit per-field accumulator (one turn = 2^24, wrapping without a jump: the figure at
phase + 1 turn is the same figure) advanced once per field (the engine runs once per field, started at the field
start) by Hypnos's bipolar decode: |K5 - 512| < 24 still, else 2^(-12 + ls / 64) turn per field with
ls = 13/16 (|K5 - 512| - 24), so the top is 2^-5.8 turn per field (capped below 2^-5: at 12 discs that is about a
fifth of a disc spacing per field, no strobing). The eccentricity axis has its own accumulator, advanced the other
way by a quarter of the same step. Nested's chain uses minus the phase and minus the axis. Drift at centre is the
only still state.

## Camera

The luma (or Osc) goes through a per-field table T(l) = min(GT l^2 >> 16, 8191) (dark areas do little), a 64-pixel
box (1080; 32 at 720p and SD) and a vertical one-pole, and the stream is added (shifted by a per-field SC) to the
shared quadratic of every circle: P = |z - c|^2 - r^2 + C. The gain scales with R^2, C_max = 0.08 R^2 at white and
100 %, so the bend is the same at every Scale. A disc of radius r shrinks by about C / 2r: 25 % = a gentle swerve of
the edges, 50 % = clear dents, 100 % = small discs eaten where the picture is bright, the big ones carved round the
subject; every edge stays a clean, anti-aliased curve. A black input or K6 = 0 is exactly the clean figure. Osc:
the lattice through the same table at twice the gain (about 2x a camera's mean level): moving diamond dents.

## Engine

- Per field (double buffered): Hardline flute's microcoded engine (37-bit accumulator, 256 x 32 register file in
  block RAM, one adder, serial 18 x 18 multiplier, serial shift) with a two-deep return stack; the program and its
  emulator are in steiner.py (994 words; the integer model runs the emulator). It runs through the field (about
  50 .. 90 k clocks) and writes the NEXT field's circles into a 128 x 32 parameter RAM: knob holds and glides,
  the counts and the per-count constants (2^24 / n, sin(pi / n), rho0, r0, 1 / s by a microcoded restoring
  divider), a, the Drift accumulators, sin / cos (quarter table, 14-bit interpolation), the scale (2^(i/256)
  table), the camera gain, and per circle (up to 14) the centre, radius, the DDA start values, the coverage shift
  and the tone class. It restarts from address 0 at every field start (so a stalled run cannot outlive a field).
- Field start: a copy moves the parameters into two line RAMs (V, U per circle) and the slots' registers.
- Per line: the line engine steps V += U, U += vv for every circle at each line end and loads each slot's DDA.
- Per pixel (14 slots: the great circle, the inner circle, 12 chain / nested): P accumulates (shared horizontal
  quadratic about the great circle's centre + the camera step) - the slot's linear step -> a four-level shifter
  window (6 bits, saturating) -> x 3/4 when set -> Bold 4 - d / Fine hw + 4 - |d| -> coverage 0 .. 8 -> chain sum,
  the class of the first disc covering half the pixel, tone ROM (Grey Bold adds the great circle, inner and nested
  inner terms) -> luma [1 2 1] / 4 -> 13/16 toward black (white 775) -> blanking gate; U = V = 512 on every pixel.
- Shell: riley / Hardline: timing-ID virtual-1080 steps, vblank zone (no avid for 1024 clocks), generated avid
  (parity-locked), one latency (24) in every mode, SD shows the centre 4:3.

## Status

- v0.1.0 built 2026-11-22 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1 (first try). Not flashed. Not tried with router2.
- Post-route Fmax (the builder's routed figure): HD Analog 93.43 MHz, HD HDMI 91.63, HD Dual 97.38 (HD needs
  74.25); SD Analog 92.64, SD HDMI 91.70, SD Dual 93.80 (SD needs 27).
- 7,112 .. 7,133 / 7,680 LC (93 %), 22 / 32 EBR. All six bitstreams pass iceunpack. No `emulate_transparency`;
  dupin finds no carry self-adds.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum steiner`.
- Model = RTL (GHDL), pixel diff 0 on every render made:
  - 720p60 decimation 1 (918,720 px): the defaults with a black input (Glide Off, Drift still); Source = Osc;
    bear.png at Camera 100 % (the model fed the tester's own luma); the power-up defaults with Glide On and Drift
    680 (= rtl_default.png, the model's second field).
  - 720p60 decimation 2: defaults; Count at 0 (3 discs) and at 1023 (12 discs) from the power-up step (both ends
    reached); Nested + Ink; two captures at the default Drift 6 fields apart (model fields 2 and 8: the travel and
    its direction match).
  - NTSC decimation 1: both fields (720 x 243 each), identical, the same picture (the centre 4:3).
- The per-field engine's emulator equals the RTL engine word for word on the parameter RAM (GHDL testbench).
- Not sure of on hardware: nothing on this chassis has been flashed yet (riley / flute / eclipse are unflashed; the
  generated avid / zone / width logic is the heirloom shell's, which is hardware-validated). The engine now runs
  through the whole field and is double buffered through a parameter RAM (new to this chassis). The picture is
  large flat discs with sharp moving edges; the [1 2 1] + 13/16 conditioning is built in against the HD HDMI bars.

## Cut / deviations

- **Nested shares the 12 chain slots** (n1 + n2 + 1 <= 12), so in Nested the outer count is not K4's: K4 steps
  through ten (outer, nested) pairs. A full nested copy (up to 25 circles) did not fit: one circle slot costs about
  240 LC, and the 15-slot first build packed 8,667 / 7,680 LC.
- **Camera as the spec (added to rho^2)**, at 0.08 R^2: a "great circle floods into the light" variant (x 8 on the
  great circle, with a slope-class anti-aliasing) was tried in the model and cut for fit (about 310 LC).
- Ink Fine joins lines as a union, not XOR (tangent lines would XOR into broken dashes at every tangency); Ink Bold
  XORs the filled discs (the inner disc and the chain; Nested's discs cut black out of the inner disc) and draws
  the great circle as a frame line.
- Disc tone classes are by size relative to r0 (0.62, 0.95, 1.45), so a disc steps its grey as it swells.
- At the anti-aliasing scale a 3/4 step is used (edges 0.8 .. 1.2 px); the Fine line's |d| is the one's complement.
- 480i: field-line step 71/4 units (1138/256 virtual lines rounded to 4.4375): the picture is 0.2 % short.
- `field_n` is passed through one register (the core ties it high on every input).
