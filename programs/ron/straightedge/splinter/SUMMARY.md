SPLINTER (Straightedge)
v0.1.0 built 2026-11-17, not flashed.  Flash out/rev_b/ron/straightedge/splinter.vmprog.

WHAT IT DRAWS
The pinwheel tiling of John Conway and Charles Radin (1994), the tiling on the facade of Federation Square,
Melbourne: a right triangle with legs 1 and 2 splits into five copies of itself, each turned by atan(1/2), so
the triangles point in infinitely many directions.  The screen is filled with the tiling at mixed depths: every
triangle one flat grey (four steps by the child index: 292 / 449 / 625 / 800 of 940), separated by a thin black
kerf.  Nothing is centred; the figure is infinite and self-similar about the zoom's eye.  Luma only (U = V = 512).
Every edge on the screen is a straight line: triangle edges and kerfs, and (with the camera) the straight borders
of the 32-px blocks the luma is held in.
The subdivision is the true pinwheel (the rectangle of the four inner triangles is cut by the diagonal from the
altitude's foot to the long leg's midpoint), not the midpoint split of concepts/sketch.py, which gives five
mirrored children, only eight directions and no spiral self-similarity.

CONTROLS
K1  X        The zoom's eye, +-480 virtual px from the centre (the picture slides with it).  Glided.
K2  Y        The eye, +-270 virtual px.  Glided.
K3  Scale    The size of the great triangles (the largest pieces): 2.5 .. 3.5 levels over the floor, short leg
             about 440 .. 990 virtual px.  Glided.
K4  Gap      The kerf: half-width 1 .. 6 virtual px plus an inset proportional to each piece's own size
             (0.17 k^3 of its short leg): a hairline at 0, exploded pointed shards on black at 100 %.  Glided.
K5  Drift    Bipolar speed (Hypnos law), still within 24 counts of the centre: the endless spiral zoom into
             (right) or out of (left) the eye.  Default 660.
K6  Camera   The luma sizes a white splinter (a similar triangle) inside every finest piece.  Default 100 %.  Glided.
S7  Glide    Off: knobs act at once (2-count hold against ADC jitter).  On (default): 1/16 of the gap per field
             on K1 K2 K3 K4 K6 P12 (the Drift speed 1/8), landing exactly.
S8  Lines    Finest: every kerf the same.  All: the kerf of an edge grows with the size of the triangle that made
             it (x1 .. x6), so the hierarchy of triangles within triangles shows.
S9  Tone     Grey: four flat greys by child index.  Ink: black and white by handedness (mirrored pieces black),
             the camera's splinters inverting what is under them.
S10 Weight   Bold: filled pieces.  Fine: outlines only (4 virtual px, inside each piece's kerf), in light greys
             by child index (Ink: white).
S11 Source   Video / Osc (the shell's drifting triangle lattice, at 3x the camera gain).
P12 Shatter  Headline.  How far the triangles have broken.  0 %: a few great triangles.  Rising, pieces split one
             by one (each splits when its size passes a threshold lowered by the slider plus a hash of its own
             path, so large and small pieces sit side by side).  100 %: splinters everywhere (short leg 26 ..
             90 virtual px).  Glided.

MOTION
One accumulator, the zoom phase in levels (Q18), stepped once per field by the signed Drift speed: zero at the
centre, then exponential from 2^-12 to 2^-3.9 level per field (default K5 = 660: 264 / 2^18 per field, one level
every 16.5 s at 60 Hz).  The view is the eye + (z - p*) U0 sqrt5^phase e^(i phase atan(1/2)): it grows by sqrt 5
and turns by atan(1/2) = 26.57 degrees per level.  The phase wraps at exactly one level, where the tiling maps
onto itself about the fixed point (1/2, 1/2) of child 3's map and the levels relabel: sizes, thresholds and the
path hashes (last three digits only) are unchanged by the relabelling, so nothing jumps.  As the pieces grow
past their thresholds they split (a hard split: the five children appear at once).

CAMERA
The camera is never shown.  Luma (or the Osc lattice) -> the mean over each 32 x 33.75 virtual-px block (a box
per block column of the line, then a vertical one-pole per column) -> gain (K6; black to luma 0.26 gives
nothing, full by 0.82) -> a 60 x 32 grid of 4-bit values in block RAM, two banks ping-ponged per field (the
pixel path reads the bank written during the previous field).  The element is the finest piece of the cascade
(level 8, or level 7 for part of each zoom level: short leg 12 .. 59 virtual px): inside it a white similar
triangle (the piece inset toward its incentre), its size set by the luma of the pixel's own block: none at 0,
the piece nearly full at 15.  Bright areas break into white splinters, dark areas stay as the clean great
triangles.  25 %: small sparse splinters; 50 %: half-size; 100 %: bright areas are a dense white mosaic and the
subject reads as a silhouette (clearest in Fine and Ink).  A splinter that crosses a block border between two
different luma values is cut along that (straight) border.  Ink: the splinter inverts the piece.  Osc
substitutes the drifting lattice at 3x gain: big diamonds of splinters.

ENGINE
- Per field (vblank zone, "no avid for 1024 clocks"): Hardline flute's microcoded engine (splinter.py assembles
  the microprogram and emulates it for the integer model; 37-bit accumulator, 128 x 32 register file, 1024 x 16
  microprogram, one serial 18 x 18 multiplier): knob holds and glides, the Drift speed and phase, the spiral
  (cos, sin, sqrt5^-phase from 65-point tables), the view's start point and per-pixel / per-line steps, the
  Shatter thresholds, and the per-level table (pixel size and the kerf of every edge rank at every level).
  Under 8,200 clocks (HD vblank about 99k, NTSC about 14.5k).
- Per line: the line start of the two affine DDAs (top-triangle coordinates, Q28).
- Per pixel (39 stages): the two DDAs -> eight pinwheel steps, two stages each, in the midpoint convention
  (S = X + 2Y + 1, T = 2X - Y, the child from four tests, its coordinates = selected / inverted bits of S and
  T: no per-pixel multiplier), beside them the path's digits, two hashes, the mirror flag and the ranks of the
  piece's three edges -> the split decision at levels 3 .. 6 -> pi, the first unsplit piece (its coordinates
  captured, its tone, its edges' kerfs from the table) -> margin = min(edge distance - kerf) -> coverage (5
  levels, four comparisons) | the finest piece and its block's luma -> the splinter's coverage -> two blends ->
  luma [1 2 1]/4 -> 13/16 toward black (white 776) -> blanking gate.
- Generated avid (the shell's parity-locked start), one latency (40) in every mode, virtual 1080-line
  coordinates (SD shows the centre 4:3).  U = V = 512.

STATUS
Built 2026-11-17 through the shared queue with router2 (the build default), every config on seed 1
(post-route Fmax, LC of 7680, EBR of 32):
  HD Analog  83.86 MHz   6721 LC   18 EBR
  SD Analog  84.28 MHz   6722 LC   18 EBR
  HD HDMI    76.07 MHz   6705 LC   18 EBR
  SD HDMI    84.15 MHz   6719 LC   18 EBR
  HD Dual    82.21 MHz   6721 LC   18 EBR
  SD Dual    77.97 MHz   6739 LC   18 EBR
(HD needs 74.25 MHz, SD 27.)  vmprog: out/rev_b/ron/straightedge/splinter.vmprog (six bitstreams pass iceunpack).
No `emulate_transparency`; no carry with a duplicated input (dupin: none), so router1 is available if a rebuild
misses.  Rebuild: ./build_set.sh straightedge splinter
Model vs RTL (GHDL against the integer model .straightedge_work/rb_splinter/imodel.py, which runs the same
microprogram through splinter.py's emulator): 0 mismatching pixels on every frame compared.  Full 720p
decimation 1 (918,720 px): bear.png at Camera 100 % with Lines All + Ink + Fine and X / Y / Scale / Gap / Shatter
moved.  720p / 2: the defaults with a black input; Source = Osc; bear.png at Camera 100 %; the default Drift
4 fields from power-up (Glide Off) and 10 fields from power-up (Glide On, the power-up defaults): the zoom
phase steps exactly as in the model, pieces growing outward from the eye and turning (K5 > 512).  NTSC
(720 x 243 per field, decimation 1, bear.png): both fields, the same picture as HD (the centre 4:3).
Renders (work dir .straightedge_work/rb_splinter/): cmp_final.png, sheet.png (control sheet), strip.png
(8 frames across P12), motion.png (Drift), camera.png (bear.png at Camera 25 / 50 / 100 %).
Not flashed.  Hardware doubts: the per-field engine is flute's (as in the Vellum set), restarted from PC 0 at
every field's zone with a watchdog; the camera grid (two banks, read and written in opposite banks) is new in
this set; HD HDMI closed with 2.4 % margin.

CUT / DEVIATIONS
- The split is hard.  A first version grew the five children from their incentres as similar triangles (no
  pop at a split) and sampled the luma at the anchor of each finest piece (whole splinters, no block borders),
  through the eye's coordinates carried down the cascade and a per-field rotation table: 12,490 LC (162 %).
  Both were cut to fit: the camera now reads the pixel's own block (the brief's fallback), so splinters are cut
  along block borders where the luma changes.
- Camera target: the spec's "luma added to the Shatter threshold per block" was tried in the prototype and
  rejected (pieces half split along block borders: noisy).  The luma instead sizes a white splinter in every
  finest piece: bright regions break into fine splinters, dark regions stay as great triangles, as the spec
  describes, and the subject is recognisable at 100 %.
- Coverage has 5 levels (four comparisons instead of a divider), plus the output's [1 2 1].
- Grey has no pure white piece (the lightest is 800 of 940): white is the camera's.
- Pan range is +-480 / +-270 virtual px (the top triangle of the cascade must cover the screen from the eye).
- Scale is the size of the great triangles (the split thresholds), not a zoom: the figure is self-similar, so
  a zoom would only shift the Drift phase.
- Resolution: the tiling, the kerfs and the grid are in virtual 1080-line coordinates, the same picture at SD,
  720p and 1080p.  The anti-aliasing unit and the Fine line width follow the real pixel (Fine: 4 px at 1080p,
  3 at 720p / 1080i, 2 at SD); at SD the finest splinters (12 virtual px) are about 3 lines tall.
- The first field after configuration is not valid (the source width is not yet measured).
