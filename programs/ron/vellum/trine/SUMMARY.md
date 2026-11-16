TRINE (Vellum)
v0.1.0 built 2026-11-16, not flashed.

WHAT IT DRAWS
Triangles within triangles, after Islamic triangular tilings, Vasarely's "Tridim" and the Sierpinski triangle of a
mathematics plate. Luma only (U = V = 512 on every pixel). Not a centred figure: a lattice that fills the screen.
- Form = Nest (default): the screen tiled by a triangular lattice (cell side about 0.3 of the height); every cell
  a nest of concentric triangles, bands of 3 x the smallest barycentric coordinate, four grey steps. Up- and
  down-cells are offset in tone so their nests interlock into stars, and the cell across the nearest edge shows
  its outermost band there.
- Form = Gasket: the Sierpinski triangle filling the height (the first carry of a + b in the lattice coordinates
  marks a hole and its level), white on black with the holes in depth greys, on an endless zoom about the apex.
- P12 Turn truncates every nest ring (and every gasket hole) by the turned triangle: triangle -> hexagon (50 %)
  -> the turned triangle (100 %), so the stars of 0 % become a honeycomb and then a star lattice of turned nests.

CONTROLS
K1  X        Nest: lattice pan, +-700 virtual px. Gasket: the zoom point (apex) offset. Glided.
K2  Y        +-540 virtual px, as K1. Glided.
K3  Scale    Nest: cell side 125 .. 997 virtual px. Gasket: height 288 .. 2306 virtual px. Exponential, glided.
K4  Depth    Nest: 1 .. 12 bands per cell (stepped; 8-count hysteresis, both ends reachable). Gasket: 1 .. 12 levels drawn
             before the holes fade to the depth grey.
K5  Drift    Bipolar speed (Hypnos law), still within 24 counts of the centre. Nest: bands stream into (right) or out of (left)
             every cell centre. Gasket: endless zoom into (right) or out of (left) the gasket.
K6  Camera   Luma added to the nest phase (Nest) / eroding the holes (Gasket). Default 100 %. Glided.
S7  Glide    Off: knobs act at once (2-count hold against ADC jitter). On (default): 1/16 of the gap per field on
             K1 K2 K3 K6 P12 (K5 1/8), landing exactly.
S8  Form     Nest / Gasket.
S9  Tone     Grey: four grey steps (Gasket: levels in depth greys) / Ink: black and white (Gasket: the opening
             level grows in from the apex, white lace on black).
S10 Weight   Bold: filled bands / triangles. Fine: outlines along every band edge (Gasket: lines along every edge).
S11 Source   Video / Osc (the shell's drifting 2-D triangle lattice replaces the luma).
P12 Turn     Headline. Nest rings and gasket holes: triangle (0 %) -> hexagon (50 %) -> turned triangle (100 %).
             Glided.

MOTION
One drift phase ph (Q18 cycles), stepped once per field by a signed speed from K5: zero at the centre, then
exponential from 2^-12 to 2^-3.9 cycle per field at the ends (default K5 = 680: 332 / 2^18 per field, about one
cycle every 13 s at 60 Hz). Nest: one cycle = 4 bands of inward flow (the tone cycle), so the wrap is seamless.
Gasket: one cycle = one octave of zoom (x2) about the apex; at the wrap the next level opens, so the fall into the
triangle never ends. RTL-checked: two captures six fields apart land exactly on the model at ph = 3 x 332 and
9 x 332 (0 mismatching pixels each), bands moving inward for K5 > 512.

CAMERA
Luma (or the Osc lattice) -> a gain table filled per field from K6 (T(i) = i x K6 / 8, >> 6; Osc >> 5) -> 32-px
box (64 px on the 1920-wide rasters) -> vertical one-pole (1/32 per line; 1/64 at 1080p; 1/16 interlaced) at half
horizontal resolution in a line store, with its per-line step stored beside it (Hardline dazzle's [D]).
- Nest: Lp is added to the band phase, so the nests bend around the picture; at full white the shift is 3.4 bands
  at 100 %, 1.7 at 50 %, 0.85 at 25 % (Osc: twice that). The store's slopes |Lx| + |Ly| widen the anti-aliasing.
- Gasket: the luma erodes the white lace into the holes (Video: min(2 Lp, 16383); Osc: Lp / 2). Edges move by up
  to about 8 px at 1080p (11 px at 720p); bright areas past about 60 % of the cap fill the hole interiors with
  grey, so the picture shows through the gasket as a halftone. Full white at 100 % greys the holes but keeps the
  gasket readable (the cap is set at the last value that does).
- Osc: the shell's 2-D triangle lattice (640 x 540 virtual px, drifting one step per field).

ENGINE
- Per field (vblank zone, "no avid for 1024 clocks"): Hardline flute's microcoded engine (trine.py assembles the
  microprogram; 37-bit accumulator, 128 x 32 register file and 1024 x 16 microprogram in block RAM, one 18 x 18
  serial multiplier): knob holds, glides (Q6), drift, the lattice front's DDA constants, the Nest's band scaling
  and turned-family constants, the gasket's per-level table (T1 / T2 / hole tone). 1,764 clocks (Nest) to 5,387
  clocks (Gasket, 12 levels), then a 256-clock gain-table fill. HD vblank is about 99k clocks, NTSC about 14.5k.
- Per line: the line engine steps the lattice row coordinate S (and the Gasket frame's base distance).
- Per pixel: one lattice beam u = b - a (a + b = S per line), Q20 cell units. Front U1 .. U8 (a, b, the carries,
  the barycentric triple, min / max); Nest tail U9 .. U14 (turned family, phase, band, nearest edge); Gasket tail
  U9 .. U14 (hole level, truncated hole + erosion, frame, nearest edge); shared back end: a 3-step restoring
  divider (9 coverage levels), blend, the mid-grey fade where bands outrun the raster; output [1 2 1] luma,
  13/16 toward black, blanking gate. Generated avid (parity vote). One latency, 40 clocks, in every mode.
- Virtual-1080 coordinates from the timing ID: the same picture at SD, 720p and 1080p (NTSC checked in RTL).

STATUS
Built 2026-11-16 with router1, every config on seed 1 (post-route Fmax, LC of 7680, EBR of 32):
  HD Analog  80.16 MHz   7187 LC   24 EBR
  SD Analog  81.67 MHz   7182 LC   24 EBR
  HD HDMI    90.82 MHz   7181 LC   24 EBR
  SD HDMI    86.93 MHz   7179 LC   24 EBR
  HD Dual    86.01 MHz   7191 LC   24 EBR
  SD Dual    81.95 MHz   7186 LC   24 EBR
vmprog: out/rev_b/ron/vellum/trine.vmprog (all six bitstreams pass iceunpack). LC is 93.6 %, about 90 over the
set's ~7,100 guide; every HD config still closes with 8 % or more margin.
router2 stalled on HD Analog (overused = 2 for thousands of iterations). router1 hung on SD Analog until two
sign-extended x + 2x adds (3 min(a, b) in the front, 3 (1/z - S) in the line engine) were rewritten as
sign & (low bits & '0' + x), which removed the only two LUTs with a duplicated input (dupin: none left).
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum trine
Model vs RTL (GHDL against the integer model .vellum_work/rb_trine/imodel.py): 0 mismatching pixels on every
frame compared. Full 720p decimation 1 (918,720 px each): Nest black, Nest Osc, Nest bear.png at K6 100 %, Gasket
black. 720p / 2: Nest and Gasket defaults, Nest Ink / Fine / Turn 50 and 100 % variants, Gasket Ink + Fine + Turn,
Osc in both Forms, bear.png through the camera in both Forms, two motion captures 6 fields apart at the default
Drift, Glide On at the power-up defaults. NTSC (720 x 486, decimation 1): Nest black and Gasket bear.png, the
same picture as HD.
Not flashed. Hardware doubts: the Gasket's camera halftone is new (no HW precedent); the per-field engine is
flute's, restarted from PC 0 at every field's zone (so a bad start cannot persist past one field).

CUT / DEVIATIONS
- P12 is a truncation morph (triangle -> hexagon -> turned triangle), not a literal 0 .. 60 degree rotation: it
  ends in the same picture as a 60-degree turn (stars -> hexagons -> stars), with no rotation multiplier pair.
  The Gasket's P12 does the same to every hole.
- Gasket camera erodes the holes instead of bending the lattice coordinates (a coordinate bend breaks the
  carry-chain hole test).
- Gasket K4 = levels drawn (1 .. 12) before the holes fade to their depth grey.
- Coverage has 9 levels (3 divider steps), smooth at HD.
- Drift tops out at 2^-3.9 cycle per field (16 bands / 4 octaves per second).
- The model panels for K1 / K2 on the sheet look alike: a pan of a periodic lattice.
- GHDL renders of the Gasket at K4 >= 10 overrun the simulator's fake 8-line vblank at 720p / decimation 2
  (the engine needs up to 5,643 clocks); hardware vblank is far longer.
- Resolution: the Nest and the Gasket geometry are the same picture at SD, 720p and 1080p (virtual-1080
  coordinates; NTSC RTL-checked). The Gasket camera's erosion is pixel-scaled (edges move up to ~8 px at
  1080p, ~11 px at 720p and SD), so at SD and 720p the camera closes more of the finest holes than at 1080p.
  The Nest camera is in band units and matches across modes (the box is 32 px, 64 px at 1920 wide).
