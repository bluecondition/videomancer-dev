HINGE (Straightedge)
v0.1.0 built 2026-11-17, not flashed.

WHAT IT DRAWS
The rotating-squares auxetic mechanism (Grima and Evans, 2000; Ron Resch's folded sheets; kinetic facades): a
lattice of rigid squares joined corner to corner. Alternate squares turn opposite ways and stay hinged; pointed
rhombic holes open between them and the whole sheet swells as s (cos t + sin t) about the screen centre. Luma
only (U = V = 512 on every pixel); every edge is a straight line. Not a centred figure: a sheet that fills the
screen.
- Grey (default): the two parities of square in white and light grey, holes black. Ink: white on black.
- P12 Open is the turn, 0 .. 90 degrees: 0 % a closed sheet (a checker of white and light-grey squares with
  2-px joints), 50 % a checkerboard of squares and equal square holes on the diagonal, 100 % closed again with
  every square turned a quarter (the same sheet as 0 %). Default 22 % (20 degrees).
- Nest puts 1 .. 6 nested square bands in every square (one step darker in Grey, black in Ink).
- Turn = Pinwheel: every square turns the same way at the same pitch and grows to touch its neighbours (side
  a cos t): each corner meets a neighbour's side and the holes are small tilted squares, the Pythagorean tiling.
- Hinge points are exact point contacts. Where squares lie side by side (the closed sheet, Pinwheel) each is
  inset by up to 1 virtual px, a 2-px joint that fades out as the holes open.

CONTROLS
K1  X        Lattice pan, +-1.5 cells. Glided. Default centre.
K2  Y        The same, vertically.
K3  Scale    Square side 64 .. 724 virtual px, exponential (default 215: about 7 x 4 squares). Glided.
K4  Nest     1 .. 6 nested bands per square (stepped; 8-count hysteresis, both ends reachable). Default 1 (solid).
K5  Drift    Bipolar speed (Hypnos law), still within 24 counts of the centre. The lattice glides on a diagonal
             (right and down for K5 > centre; x at twice the y rate) and the nested bands stream into (right) or
             out of (left) every square's centre. Default 660: about 32 px a second at 1080p.
K6  Camera   Halftone strength. Default 100 %; 0 % = the clean figure. See Camera.
S7  Glide    Off: the knobs act this field (a +-1-count hold against jitter). On (default): 1/16 of the gap per
             field on K1 K2 K3 K6 P12 (Drift speed 1/8), landing exactly.
S8  Turn     Hinged (the mechanism, rhombic holes) / Pinwheel (Pythagorean tiling, small square holes).
S9  Tone     Grey (white / light-grey squares, bands one step darker) / Ink (black and white).
S10 Weight   Bold: filled squares, equal bands. Fine: outlines of the squares and of every nested band (1/8 band
             wide, at least 2 virtual px).
S11 Source   Video / Osc (the shell's drifting triangle lattice at twice the camera level).
P12 Open     Headline: the turn angle 0 .. 90 degrees. The sheet opens, swells, and closes turned. Glided.

MOTION
One Drift accumulator (20 bits), stepped once per field in the vblank zone by a signed speed from K5: zero within
24 counts of the centre, then exponential, 2^-12 .. 2^-4 of two cells per field (the top is 1/8 cell and 1/4 band
per field: fast, no strobing; the Hypnos range was capped two octaves lower at the top). Three things move from
it at related rates: the lattice x glide (wraps at exactly 4 cells), the y glide at half that rate (wraps at 2
cells; the parity period is 2 cells, so neither wrap shows), and the band stream (4 bands per two cells of glide,
wrapping one band period). With Nest at 1 the glide alone moves. Motion is rigid; nothing stops it but Drift at
the centre. RTL-checked: captures at fields 4 and 10 of the default Drift land on the model with 0 mismatching
pixels (travel right and down).

CAMERA
Luma (or Osc) -> 32-sample box -> a vertical one-pole per block column -> a 4-bit level per 32 x 32 virtual-px
screen block (60 x 34 blocks; 4096 x 4 block RAM, two banks swapped once per field at the zone start, so the
bank being read is never the one being written; one field old). Each block holds one small square whose size
is that held level; the level is constant across the block, so every edge stays straight and the only new edges
are the small squares' own.
- Bold: the small square starts as the whole block (levels 0 and 1: the sheet is untouched, black input is
  exactly the clean figure) and shrinks toward its block centre as the luma rises, down to an 8-px square at
  white; the squares of the sheet are cut by this halftone. Bright areas dissolve into small squares, dark
  areas stay solid: the subject reads as a negative halftone inside the hinged sheet.
- Fine: the small squares are dots that GROW with the luma inside the outlined squares (none in the dark, up to
  80 % of the block at white): a positive halftone, so Weight also flips the polarity.
- K6: the shrink (growth) at white is 85 % x K6. 25 %: bright blocks lose a fifth of their side (thin grid
  lines appear in the highlights); 50 %: clear separate squares in the highlights; 100 %: highlights down to
  small squares, the subject bold. bear.png is recognisable at 100 % (clearest with the sheet closed or in Ink).
- Osc: the shell's 2-D triangle lattice (640 x 540 virtual px, drifting one step per field) at twice the
  camera level, saturating.

ENGINE
- Per field (vblank zone, "no avid for 1024 clocks", about 550 zone clocks): +-1 holds and glides (five clocks
  per channel), the Drift speed and accumulator, then a sequencer on ONE serial 21 x 20 multiplier (15 slots):
  size from a 2^(i/256) ROM, sin and cos of the folded turn (129-point tables, linearly interpolated to a 13-bit
  angle), the pitch a = s (cos + sin), a cos and a sin, the start cell and local coordinate of line 0 and of
  pixel 0 (the centre's lattice coordinate x pitch, brought into its cell by 32 conditional steps), their
  products with cos and sin, the per-line steps, the band width (x 1/Nest), the stream offset; then the 16-entry
  dot-size table by an accumulator.
- Per line: two clocks. ly cos t, ly sin t and the row lead step by one line, then back by one cell where the
  line crossed a row; the camera block row and its phase the same way.
- Per pixel (18 stages + 7 of output): a cell lead counter and lx cos t, lx sin t (Q14) that step back by one
  cell at every cell crossing (one add each) -> u, v in the square's frame (Q4) -> Pinwheel: which side the
  pixel is past, the neighbour's offset -> max(|u|, |v|) -> edge distance and band coordinate -> three restoring
  steps (band remainder) -> saturate -> coverage (17 levels; 1080 / 720p / SD pixel sizes) -> tone -> x 73 ->
  x coverage (gated shifts) -> luma -> [1 2 1] / 4 -> 13/16 toward black (white 776) -> blanking gate. The only
  pixel-path multiply is tone x coverage. Generated avid (parity vote). One latency, 24 clocks, in every mode.
- Virtual-1080 coordinates from the timing ID: the same picture at SD, 720p and 1080p (NTSC checked in RTL).

STATUS
Built 2026-11-17 with router1, every config on seed 1 (post-route Fmax, LC of 7680, EBR of 32):
  HD Analog  82.26 MHz   6937 LC   11 EBR
  SD Analog  82.20 MHz   6942 LC   11 EBR
  HD HDMI    84.06 MHz   6946 LC   11 EBR
  SD HDMI    81.57 MHz   6941 LC   11 EBR
  HD Dual    79.70 MHz   6942 LC   11 EBR
  SD Dual    79.28 MHz   6935 LC   11 EBR
vmprog: out/rev_b/ron/straightedge/hinge.vmprog. LC is 90.4 %.
router2 missed HD Analog on four seeds with an earlier netlist and stalled on the final one (overused = 2 for
thousands of iterations); router1 closed all six on seed 1 (dupin: no duplicated inputs).
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge hinge
Model vs RTL (GHDL of the built sources against the integer model .straightedge_work/rb_hinge/imodel.py): 0
mismatching pixels on every frame compared. Full 720p decimation 1 (918,720 px each): defaults with black
input, Osc, bear.png at K6 100 %. 720p / 2: Osc; Pinwheel + bear; Ink + Nest 4 + Open 68 %; Fine + Nest 3 +
bear; X / Y / Scale / Camera / Open off their defaults + bear; Pinwheel + Fine + Nest 6 at a small Scale; two
motion captures (fields 4 and 10 of the default Drift); Glide On at the power-up defaults. 1080p / 2: Nest 3 +
bear. NTSC (720 x 486, decimation 1, both fields): bear.png, the same picture as HD (the centre 4:3).
Not flashed. Hardware doubts: the camera grid and its bank swap are new on hardware (the swap is keyed to the
vblank zone, as every per-field latch of this shell is); the per-field sequencer is the Vellum cartouche pattern.

CUT / DEVIATIONS
- Camera target changed from the spec (one luma sample per hinged square setting its size). At the default scale
  that is 7 x 4 samples, and "bright = full square" would make a black input all small squares. Now: a
  32-px screen-block halftone, luma SHRINKS the small squares (Bold) so black input = the clean figure.
- Two richer camera versions were built and cut for fit: small squares in each square's own turned frame
  (first RTL 9,041 LC, 117 %) and turned dots on the block grid (7,530 LC). The shipped dots are axis aligned,
  they do not turn with the squares, and they sit on the screen while the sheet glides over them.
- Pinwheel: at the same pitch squares of side s do not touch; the squares grow to a cos t (up to 21 % at 22.5
  degrees) so that corners meet sides. They are not rigid in Pinwheel.
- P12 100 % is the same picture as 0 % (a quarter turn of a square).
- Scale steps are 2^(1/256) (0.27 %): a slow Scale sweep moves the far squares in steps of 2-3 px.
- Camera levels: 4 bits (14 steps from black to white).
- Resolution: geometry, bands, joints and the camera grid are in virtual-1080 units; SD shows the centre 4:3
  of the canvas. The box filter is 32 samples in every mode (32 px at 1080p, 48 virtual px at 720p, 64 at SD),
  so the halftone is a little softer at SD.
