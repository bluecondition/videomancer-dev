GNOMON (Straightedge)
v0.1.0 built 2026-11-22, not flashed. Flash out/rev_b/ron/straightedge/gnomon.vmprog.

WHAT IT DRAWS
The gnomon of a sundial; aerial photographs of a field of columns at low sun; de Chirico's long raking shadows.
A plain of square posts seen from straight above, each casting a long, flat, black shadow, all parallel. The
sun circles, so the shadows swing round together like clock hands; as they lengthen they run under the
neighbouring posts and lock into striped bars at the lattice's rational directions (0, 26.6, 45 degrees ...),
then break apart again. Luma only (U = V = 512 on every pixel); every edge is a straight line.
- The shadow of a post of half size h swept along s = L P (cos a, sin a) is a hexagon. With the sun folded
  into the first quadrant (mirroring x and y): u in [-h, h + L P cf], v in [-h, h + L P sf] and
  |u sf - v cf| <= h (cf + sf). A pixel tests the nine posts behind it (three rows of three; in the Stagger
  middle row, whose posts sit half a cell over, the three nearest the pixel by the sign of u'). Shadow = the
  union; the post's own top is painted over it; shadows do not darken where they overlap.
- Grey (default): light-grey ground, black shadows, white post tops. Ink: white ground and tops, black
  shadows; the posts show only as notches cut out of the shadows.
- Default frame: 13.6 x 7.7 posts (pitch 141 virtual px), posts 0.32 cell wide, shadows 1.34 cells long at
  30 degrees, turning clockwise.

CONTROLS
K1  X        Lattice pan, +-1.25 cells. Glided. Default centre.
K2  Y        The same, vertically (up = up).
K3  Scale    Pitch 50 .. 141 (default) .. 400 virtual px, exponential (3 octaves). Glided.
K4  Post     Post half size 0.04 .. 0.45 cell: pins to blocks that almost touch (default 0.16). Glided.
K5  Drift    The sun's azimuth turns. Bipolar speed: still within 24 counts of the centre, else exponential
             2^-12 .. 2^-6 turn per field (Hypnos law; top capped 4 octaves below Hypnos: at a quarter turn per
             field a 4-fold lattice strobes). Default 650: one turn in about 26 s, clockwise. Wraps exactly at
             one turn.
K6  Camera   Tile tone strength, 0 .. 100 %, default 100 %. 0 % = exactly the black-input picture. See Camera.
S7  Glide    On (default): K1 K2 K3 K4 K6 P12 ease 1/16 of the gap per field (never under half a count, landing
             exactly), the Drift speed 1/8. Off: no easing (a knob must move 2 counts: pot jitter).
S8  Form     Grid / Stagger (alternate rows half a cell over: brickwork; the shadow weave changes).
S9  Tone     Grey / Ink.
S10 Weight   Bold (filled shadows) / Fine (the shadows' outline and the posts' outlines, an architect's plan;
             line h / 4 within 2 .. 6 virtual px; Grey: black shadow lines, white post lines on grey).
S11 Source   Video (default) / Osc.
P12 Sun      Headline: the shadow length, 0.15 .. 2.0 cells (linear). 0 % = short stubs at high noon (the posts
             read as lit relief); 50 % = about one cell, the shadows reach the next post; 100 % = 2 cells, the
             field is mostly shadow and the sun's turn opens and closes bars of light at the rational
             directions. Default 65 % (1.34 cells). Glided.

MOTION
One drift phase (2^20 = one turn), stepped once per field by the microcoded engine by the signed Drift speed and
taken modulo one turn, so the sun never jumps. Only the light moves: the lattice and the posts stay put.
2^-12 .. 2^-6 turn per field (0.09 .. 5.6 degrees per field at 60 Hz; a turn in 68 s .. 1.1 s). Default 650:
0.23 degrees per field. The quadrant fold (mirroring x and y) is exact at 0 / 90 / 180 / 270 degrees (at those
angles the shadow is a rectangle and both folds give the same test).

CAMERA
Luma (or Osc) -> 32-sample box -> a vertical one-pole per 32-vpx block column -> an 8-bit level per 32 x 32
virtual-px block (two block-RAM banks swapped once per field: one field old). Each post's cell (its tile: the
square around it, or the brick in Stagger) holds the level of the block at the post's centre, read once per cell
crossing, and steps the tile's ALBEDO in four steps (thresholds 88 / 120 / 153 of 219 at 100 %, rising with less
K6; K6 0 % never steps):
- Grey: the tile's lit ground 0.625 / 0.75 / 0.875 / 1.0 and its shadow 0 / 0.125 / 0.25 / 0.375.
- Ink: the two brightest steps turn the tile white (its shadows vanish): dark parts of the subject keep the shadow
  pattern, bright parts are blank paper.
- Fine: the tile's ground and its shadow lines step the same way.
Every edge stays straight; the only new edges are the tiles' axis-aligned borders (not anti-aliased; the output
[1 2 1] softens them). 25 %: a few of the brightest tiles lighten; 50 %: the bright background lightens in
steps; 100 %: bear.png reads as a dark mosaic with both ears (clearly at a smaller Scale or with long shadows,
where the tiles show through the shadow tone). Osc: the shell's drifting 640 x 540 triangle lattice at twice the
camera level (big moving diamonds of light and dark tiles).

ENGINE
- Per field: Hardline flute's microcoded engine (lifted from windrose; gnomon.py assembles the microprogram,
  729 words; 37-bit accumulator, 256 x 32 register file, serial 18 x 18 multiplier), started at every vblank zone
  ("no avid for 1024 clocks"), about 3,500 clocks. It does the knob holds and glides, the Drift, the sun's
  azimuth and its fold (256-step quarter sine, 10-bit interpolation), the pitch (2^(i/256)), size and length,
  every product, the pan, line 0 and pixel 0 brought into their cells, and writes 41 registers by OUT (the last
  one starts the line engine on line 0).
- Per line (in the blanking, after the line's last pixel): step to the next line (row crossing by a counter),
  then 18 steps on one 15-bit ALU: the nine slots' cross offsets, the three rows' v-box distances, the top's v
  part; the camera's block row; then two reads load the home post's and the next post's camera step.
- Per pixel (22 stages + 7 of output): a cell counter and lx sf (Q3.10) that step back one cell at each
  crossing -> u' (folded) -> 9 cross terms and 6 column boxes -> saturate (+-8 virtual px, Q3) -> min per slot ->
  max over the slots (union) -> coverage (1080 / 720p / SD pixel sizes) -> tone (tile albedo) -> Fine blend
  (gated shifts) -> luma -> [1 2 1] / 4 -> 13/16 toward black (white 777) -> blanking gate. No per-pixel
  multiplier except the Fine blend. Generated avid (parity vote). One latency, 28 clocks, in every mode.
- Virtual-1080 coordinates from the timing ID: the same picture at SD, 720p and 1080p.

STATUS
Built 2026-11-22 through the shared queue with router1, every config on seed 1 (post-route Fmax, LC of 7680, EBR
of 32):
  HD Analog  86.95 MHz   6813 LC   18 EBR
  SD Analog  87.80 MHz   6817 LC   18 EBR
  HD HDMI    91.52 MHz   6810 LC   18 EBR
  SD HDMI    84.02 MHz   6823 LC   18 EBR
  HD Dual    86.72 MHz   6829 LC   18 EBR
  SD Dual    82.85 MHz   6828 LC   18 EBR
vmprog: out/rev_b/ron/straightedge/gnomon.vmprog (all six bitstreams pass iceunpack). LC is 89 %. No
emulate_transparency; dupin: no duplicated inputs.
The first build (latency 24) missed HD Analog on all six router1 seeds (68.3 MHz best): the coverage stage chained
four carry chains; restaged (latency 28) it routed at 98 MHz in a profile and 82.9 .. 91.5 MHz in the build.
router2 was not tried.
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge gnomon
Model vs RTL (GHDL of the built sources against the integer model .straightedge_work/rb_gnomon/imodel.py, which
runs the same microprogram through gnomon.py's emulator): 0 mismatching pixels on every frame compared.
Full 720p (decimation 1, 918,720 px each): the power-up defaults (Glide On, Drift moving, black input =
rtl_default.png); the defaults with Drift held and Glide off (black input, bear.png, Osc). 720p / 2: defaults;
Stagger + Fine + Post 59 % + Sun 88 %; bear.png (Grey and Ink); Osc; three sim-only copies started in the other
quadrants (sun at 120 degrees + Stagger + Sun 100 %; 250 degrees + Ink + Fine + Scale 29 %; 330 degrees + bear
+ Sun 20 %); two motion captures of the default Drift (fields 4 and 10: the sun 1.4 degrees further clockwise,
both exact). NTSC (720 x 486, both fields, bear.png): each field
matches the model exactly when the camera grid is taken from the other field's luma (the grid is one field old);
the same picture as HD (the centre 4:3, 7.7 rows).
Not flashed. Hardware doubts: the camera grid is read once per cell crossing through a small lookahead FSM (new);
the flute engine is windrose's (not yet flashed either).

CUT / DEVIATIONS
- Camera target: the spec's per-post HEIGHT (shadow length) was cut for fit. Per-post heights need a per-slot
  far edge, a 4:1 constant mux and a v-box mux in every slot (about 280 LUT a slot); the first RTL was 9,694 LC
  (126 %). The camera now steps a flat tone of each post's cell (the brief's "a flat tone step"), which lets the
  u box be shared per column: 8,382 LC; moving the per-field sequencer into flute's microcode: 6,781 LC.
- P12 100 % = 2.0 cells, not 2.5: at 2.5 the frame is about 95 % black at most azimuths and needs 16 candidate
  posts per pixel; at 2.0 nine suffice.
- Tile borders are hard (axis-aligned) edges.
- P12 is linear (0.15 + 1.85 p cells): 50 % = 1.08 cells.
