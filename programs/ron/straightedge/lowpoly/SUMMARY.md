LOWPOLY (Straightedge)
v0.1.0 built 2026-11-17, not flashed. Flash out/rev_b/ron/straightedge/lowpoly.vmprog.

WHAT IT DRAWS
A faceted sheet, after low-polygon rendering, a sheet of crumpled paper lit from the side, a cut gemstone and
Ron Resch's folded paper reliefs. The screen is an irregular mesh of triangles; every facet is ONE flat grey
given by its slope toward a light, in six flat steps. Luma only (U = V = 512 on every pixel). Nothing is centred:
the mesh fills the screen and zooms about the screen centre.
- The mesh: columns of constant width along a sheared axis (xs = x + lean y). The vertex (column c, row j) keeps
  its column and is displaced only in height, y = j rh + A tri(phase), so a row boundary is a polyline that is
  straight inside a column. Each quad between two polylines is split by one diagonal whose direction alternates
  like a chequerboard. Rows are 7/8 of the column width.
- Every edge is the zero of an affine function: one accumulator per edge (the row polyline through the pixel's
  nearest row, the diagonals of the two candidate quads), reloaded at every column from the vertex tables and
  stepped by one subtraction per pixel. Edges are anti-aliased over one pixel from the edge function's value
  divided by its L1 gradient norm (9 coverage levels per edge); the pixel blends toward the facet across the
  diagonal, then across the polyline, then across the column border.
- Each vertex also has a height z; a facet's tone is 1/2 + lx dz/dx + ly dz/dy taken along its lattice edges
  (differences of the vertices' light bytes: adds only), plus the camera, quantised to 0, 20, 40, 60, 80, 100 %.
- P12 Relief: 0 % = a perfectly regular lattice of four-sided studs (pyramids on the alternating vertices, four
  tones from the light), 50 % = the default crumple (displacement 0.28 row, random z), 100 % = wild shards
  (displacement 0.42 row, z strong enough that most facets go to black or white).

CONTROLS
K1  X        The light, across: lx = 0.5 + 0.75 X (default centre = lit from the left). Glided.
K2  Y        The light, down: ly = 0.3 + 0.75 Y. The light is never straight overhead (|lx| + |ly| >= 0.25). Glided.
K3  Scale    Facet size, exponential: columns 32 .. 384 virtual px (default 125: 15 columns), about the screen
             centre. Glided.
K4  Lean     Shear of the columns, bipolar, up to 45 degrees either way; an 8-count dead centre = upright.
             Default 25 % (upright columns read as stripes). Glided.
K5  Drift    Bipolar speed (Hypnos law), still within 24 counts of the centre: the ripple travels right and down
             (+) or left and up (-).
K6  Camera   The luma held per facet shifts its flat tone, relative to the picture's mean. Default 100 %. Glided.
S7  Glide    Off: knobs act at once (a 1-count hold against pot jitter). On (default): 1/16 of the gap per field
             on K1 K2 K3 K4 K6 P12 (never under half a count, landing exactly), 1/8 on the Drift speed.
S8  Facet    Triangles / Quads (no diagonal: every quad takes its upper triangle's tone; folded strips).
S9  Tone     Grey: six flat steps / Ink: every facet pure black or white (a harlequin of shards).
S10 Weight   Bold: filled facets / Fine: the mesh edges as white lines (1/32 of the column wide, 2 .. 6 px)
             over the facets dimmed to 0 .. 31 %; Ink + Fine: the lines XOR the harlequin (every shard outlined
             in the opposite tone).
S11 Source   Video / Osc (the shell's drifting 2-D triangle lattice replaces the luma, at twice the level).
P12 Relief   Headline: studs (0 %) -> crumple (50 %) -> shards (100 %). Glided.

MOTION
One drift phase (1/256 cycle index + a 12-bit fraction), stepped once per field by the signed speed from K5: zero
at the centre, then exponential from 2^-12 to 2^-5 cycle per field at the ends (capped at 2^-5: a faster bob
strobes). Default K5 = 680: 1056 / 2^20 cycle per field, one cycle every 993 fields (16.5 s at 60 Hz).
Every vertex's displacement AND its z follow a triangle wave of that phase with the vertex's own hashed offset
(half a cycle of spread) on top of a wave 16 columns / 32 rows long, so the bobbing travels across the sheet as a
ripple: the facets tilt and the shading runs over them like light on faceted glass. The tables are refilled every
field with the phase's fraction (the integer part offsets their index), so the motion is continuous and wraps
exactly at one cycle. Facets stay rigid and every edge straight at every instant.
RTL-checked: captures after 4 and after 10 field starts at the default Drift equal the model at exactly those
phases (0 mismatching pixels each).

CAMERA
Luma (or the Osc lattice) -> 32-px box (16 px at SD: 32 virtual px) -> vertical one-pole in a line store (the
shell's [D]) -> a gain table filled per field from K6 -> the CAMERA GRID: 64 x 32 cells of 32 x 32 virtual px
(6 bits), written once per field in one bank of two block RAMs and read during the next field from the other.
Each facet reads ONE cell, chosen by the facet's own lattice position (the column's 1/4 or 3/4 point by the
diagonal's direction, on the quad's middle row, sheared with the row): a pure function of (column, row), fetched
once per column and held, so a facet is one flat tone and the only new edges are the mesh's own.
The cell value is added to the facet's tone before the six-step quantiser, RELATIVE TO THE GRID'S MEAN of the last
field (the centre 32 x 32 cells): tone += 1.5 K6 (luma - mean). At 100 % a facet 85 luma levels (of 219) above the
picture's mean rises 2.5 of the six steps; 50 %: 1.25 steps; 25 %: 0.6 step (a visible lean). A black input, a flat
input or K6 = 0 gives exactly the clean sheet. Ink thresholds the same sum (a black-and-white low-poly portrait);
Fine shows it in the dim facets under the lines. Osc: the lattice at twice the level (broad diamonds of tone).
bear.png at 100 %: the head is a dark mass of facets on a white ground at the default Scale; a recognisable
low-poly bear from about Scale 30 % down (facets under about 60 px).

ENGINE
- Per field (vblank zone, "no avid for 1024 clocks", restarted at every zone start): a microcoded engine after
  Hardline flute / Vellum cassini (lowpoly.py holds the 900-word program and its emulator; 37-bit accumulator,
  256 x 32 register file and 1024 x 16 microprogram in block RAM, one serial 18 x 18 multiplier): the video
  mode's constants, knob holds and glides, the Drift, scale, lean, light, relief, line 0's start values (two
  restoring divisions), and five table passes on one fill accumulator (vertex displacement, its slope share,
  the two light bytes, the camera gain; 256 entries each). 3,300 clocks at rest, 4,400 while every knob glides
  (HD vblank is about 99k clocks, NTSC about 14.5k).
- Per line (20 clocks after the generated avid falls, active lines only): the row phase + 1 line (a new row past
  the wrap point), the first column's phase + the lean; then the line's first column is fetched and its three
  edge accumulators are walked to pixel 0 bit-serially (15 clocks).
- Per column (at each column crossing, at least 16 clocks apart): six vertex reads (hash of (column, row): three
  shift-and-add rounds -> table address -> block RAM -> fabric) accumulate the three edges' bases and steps and
  the four facets' tone sums; four grid reads fetch the camera cells; ready 13 clocks after the crossing.
- Per pixel (stages 16 .. 37): a second column counter, three edge accumulators, two 4 x 5 products for the
  gradient norms, shift-and-add distance scaling, three 9 x 3 / 9 x 4 blends, tone [1 2 1], 13/16 toward
  black (white 771), blanking gate. Generated avid (parity vote); the sync delay is a block-RAM delay line.
  One latency, 40 clocks, in every mode.
- Virtual-1080 coordinates from the timing ID: the same picture at SD, 720p and 1080p.

STATUS
Built 2026-11-17 through the shared queue with router1, every config on seed 1 (post-route Fmax, LC of 7680,
EBR of 32):
  HD Analog  82.58 MHz   7048 LC   23 EBR
  SD Analog  80.80 MHz   7041 LC   23 EBR
  HD HDMI    82.82 MHz   7044 LC   23 EBR
  SD HDMI    72.55 MHz   7046 LC   23 EBR
  HD Dual    84.02 MHz   7059 LC   23 EBR
  SD Dual    78.84 MHz   7060 LC   23 EBR
vmprog: out/rev_b/ron/straightedge/lowpoly.vmprog (all six bitstreams pass iceunpack). 91.9 % LC.
No carry self-adds and no duplicated LUT inputs (dupin: none). An earlier, larger version (7,307 LC) missed HD
HDMI on six router2 seeds; the engine's port strobe was registered after that and router2 was not tried again.
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge lowpoly
Model vs RTL (GHDL against the integer model .straightedge_work/rb_lowpoly/imodel.py, which runs the same
microprogram): 0 mismatching pixels on every frame compared.
- Full 720p60 decimation 1 (921,600 px each): the defaults with a black input; Source = Osc; bear.png at K6 100 %.
- 720p60 / 2 (230,400 px each): defaults, Fine, Ink, Ink + Fine, Quads, Quads + Fine, Lean 0 and 100 %, Scale 0
  and 100 %, Relief 0 and 100 %, light at X 0 / Y 100 %, Osc, bear.png at K6 100 %, two motion captures (4 and
  10 field starts at the default Drift), Glide On at the power-up defaults.
- NTSC (720 x 243 per field, decimation 1): black input, and bear.png on both captured fields (each field shows
  the camera grid of the field before it); the same picture as HD (the centre 4:3).
Not flashed.

CUT / DEVIATIONS
- Camera relative to the picture's mean (the spec adds the luma straight to the tone: bear.png's bright ground
  washed the whole sheet white). Black or flat input is still exactly the clean sheet.
- One camera cell per facet from the quad's middle row: the two triangles of a quad differ by their x sample
  only (two sample rows per quad cost about 70 LC).
- Fine (Grey) keeps the facets, dimmed to 0 .. 31 %, under the white lines, so X / Y, Relief and Camera stay
  alive in Fine (the spec: lines on black).
- Quads take the upper triangle's tone, not the two triangles' average.
- The Drift bob rides a travelling wave with half a cycle of hashed spread (the spec: a fully hashed phase), so
  that the direction of Drift reads; top speed 2^-5 cycle per field.
- Smallest facet 32 virtual px (the spec: about 12): a column must be at least 16 pixel clocks wide for the
  per-column fetch, and SD has 2 virtual px per pixel.
- Coverage has 9 levels per edge; the edges' sub-pixel start is in quarters of a pixel (vertices are exact to
  1/8 px across).
- Six tone steps, no kerf between facets.
- Fit history: the first RTL (hard-wired per-field sequencer) was 8,166 LUT / 3,875 FF standalone; the microcoded
  engine, one fill accumulator, the concatenation hash, eighths of coverage and the 6-bit grid brought it to
  4,387 LUT / 2,451 FF (7,060 LC built).
- Resolution: geometry, tones and camera grid are in virtual-1080 units, the same picture at SD, 720p and
  1080p (NTSC RTL-checked). At 480i a field line is 4.4 virtual px tall, so the smallest facets (K3 near 0:
  28 virtual px rows) are about 6 lines tall and their Fine lines (2 px) are 4 virtual px wide across but 9
  virtual px tall.
