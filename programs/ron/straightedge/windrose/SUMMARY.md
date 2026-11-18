WINDROSE (Straightedge)
v0.1.0 built 2026-11-17, not flashed. Flash out/rev_b/ron/straightedge/windrose.vmprog.

WHAT IT DRAWS
The compass rose of portolan charts and engraved maps: tiers of pointed stars, every point split down its axis
into a light half and a dark half so it reads as a faceted pyramid. Luma only (U = V = 512 on every pixel); every
edge on the screen is a straight line.
- Form = Rose (default): one great rose. Four stars about one centre: a 4-point star nearly the height of the
  screen (R), a 4-point star of 0.80 R, an 8-point star of 0.62 R and an 8-point star of 0.48 R, painted one over
  another, the smaller ones greyer (light / dark halves 255 / 77, 204 / 56, 158 / 41, 117 / 28 of 255). At
  power-up they stand at 0, 45, 22.5 and 11.25 degrees (the classic rose) and then turn against each other.
- Form = Field: the same rose in every cell of a square lattice whose cells are MIRROR images of their
  neighbours, so neighbouring roses counter-rotate like gears and a point that crosses a cell border meets its own
  reflection: a woven wall of stars (rose radius 0.62 cell).
Per star, in its turned frame (u, v), with c, s = cos, sin of the edge normal's angle phi from the point's axis:
    4 points   E = (|u| >= |v| ? c|u| + s|v| : c|v| + s|u|) - R c
    8 points   E = min(c max + s min, (c (|u| + |v|) + s ||u| - |v||) 181/256) - R c     (two 4-point stars folded)
    coverage   clamp(1/2 - E) - clamp(1/2 + E - 2 s b), b = the distance from the point's own axis, so needles
               thinner than a pixel and every tip taper to nothing instead of ending in a grey line
    facets     the parity of the folds, anti-aliased across the axis and across the line between two points
All distances are in an anti-aliasing unit of 1 virtual px (1080p), 2 (1080i, 720p, 480p, 576p) or 4 (480i, 576i);
coverage has 9 levels.

CONTROLS
K1  X        Rose: the rose's centre, +-640 virtual px. Field: pan, +- one cell. Glided.
K2  Y        Rose: +-360 virtual px (up = up). Field: pan. Glided.
K3  Scale    Rose: R = 175 .. 530 (default) .. 1605 virtual px (points run off the screen). Field: cell 89 .. 270
             .. 818 virtual px. Exponential, glided.
K4  Points   4 / 8 / 16 (default) / 24: one to four stars. Stepped, 8-count hysteresis, both ends reachable.
K5  Drift    Bipolar speed (Hypnos law): still within 24 counts of the centre, else 2^-13 .. 2^-6 quarter turn
             per field for the great star; the stars turn at 1, -3/2, 2, -3 times that. Default 660.
K6  Camera   The weight of the echo lines (see Camera), 0 .. 100 %, default 100 %. Glided.
S7  Glide    On (default): K1 K2 K3 K6 P12 ease 1/16 of the gap per field (never under half a count, landing
             exactly), the Drift speed 1/8. Off: no easing. A knob must move 2 counts to register (pot jitter).
S8  Form     Rose / Field.
S9  Tone     Grey (light and dark halves, stars painted over each other) / Ink (black and white, the stars
             XOR: where points cross they cut each other into sharp facets).
S10 Weight   Bold (filled points) / Fine (each star's outline and its points' axis lines; Grey: hidden-line,
             Ink: every line; line weight R / 80 within 2.5 .. 6 virtual px; echo lines at half weight).
S11 Source   Video (default) / Osc.
P12 Point    Headline: the sharpness of every point. 0 % = fat (edge normal 52 degrees from the axis: the great
             star is nearly a square and the tiers together a faceted polygon), 50 % = the classic compass point
             (79.2 degrees, waist 0.19 R), 100 % = needles (88 degrees: hairline rays). Glided.

MOTION
One drift phase (2^25 = half a turn), stepped once per field by the signed speed; star angles are 1, -3/2, 2 and
-3 times it plus their rest angles, each taken modulo a quarter turn (the split shading is unchanged by a quarter
turn), so the phase wraps exactly on every star's period and nothing jumps. Speed: +-24 counts still, then
exponential 2^-13 .. 2^-6 quarter turn per field (0.011 .. 1.4 degrees per field for the great star, three times
that for the smallest; a quarter turn of the great star takes 137 s .. 1 s). The top is capped at 2^-6: a
4-fold figure strobes long before 2^-2. Default 660: the great star turns 90 degrees in 39 s (0.039 degrees per
field), clockwise for Drift > 0. In Field every second rose turns the other
way. RTL-checked: captures after 3, 7 and 9 fields at the power-up defaults (Glide On) each land exactly on the
model after the same number of steps (0 mismatching pixels).

CAMERA
The luma (or Osc) is never shown and never bends a line. It is averaged exactly over 32 x 32 virtual px blocks
(60 x 34 at 1080p / 720p, 45 x 34 at SD: block sums over 64 half-pixels with a straddling pixel split by weight,
then a vertical one-pole per block column, 1/16 .. 1/4 per line by mode, loaded on the field's first line), scaled
by K6 and written, 8 bits, into a block-RAM grid of two banks swapped every field. The next field reads the
pixel's own block (both Forms), so the value is constant across the block, and it sets ONE parameter: the WEIGHT
of the echo lines, straight lines ruled around the great star at constant distance from its edges (the level
sets of its E, one every 16 virtual px). So the subject appears as a line engraving that radiates in the shape
of the rose, stepped along straight block borders, with the rose lying clean on top; when the rose turns the
whole screen of lines turns with it.
- 0 % or a black input: exactly the clean figure (a zero weight draws nothing).
- 25 %: hairlines, up to 19 % of the pitch at white.  50 %: up to 37 %.  100 %: up to 75 % (12 of 16 virtual px):
  bear.png is clearly recognisable (ears, head, muzzle against the bright ground).
- Grey: the lines are 3/4 white and lie under the stars. Ink: white, and they show through the XOR's holes.
- Osc: the shell's drifting 640 x 540 triangle lattice (0 .. 222) in place of the luma: the lines swell and thin
  in big moving diamonds, about twice the bear's mean level.

ENGINE
- Per field: Hardline flute's microcoded engine (windrose.py assembles the microprogram, 900 words; 37-bit
  accumulator, 256 x 32 register file, serial 18 x 18 multiplier). It does the knob holds and glides, the drift,
  Point's sin / cos (256-step quarter table, 16-bit interpolation), the scale and centre, the Field lattice's
  fold (cell size in whole pixels / lines, where the first fold falls), and for each of the 16 accumulators its
  pixel step, three line wraps (forward, backward, held), and field seed; 8.3 k .. 9.5 k clocks. It is
  started at every vblank zone ("no avid for 1024 clocks") and writes one bank of a two-bank table WHILE the
  other bank is being drawn; the next zone start loads the constants and seeds from the finished bank. So a
  knob acts one field later than in the other programs of the set, and the engine needs no vblank time.
- Per line: no multiplier. 16 accumulators (24 bits, Q12 of the unit), each with its own increment register:
  the pixel step on every clock of the line window (width + 6 steps), then one add of the line wrap parked in
  the same register. Field: at a cell border the increment registers are complemented with carry-in (a
  mirror fold; one held clock when the cell is an odd number of pixels wide), vertically the loader picks the
  wrap word.
- Per pixel (no multiplier except the 181/256 shift-add in the 8-point stars): |A| .. |D| -> candidates -> E - K
  -> coverage, facet / line coverage (4-point front 3 stages, 8-point 6; back 3) -> the two topmost stars that
  touch the pixel blended over the third or the echo -> luma -> [1 2 1] / 4 -> 13/16 toward black (white 775) ->
  blanking gate. Generated avid (the shell's parity-locked start), one latency (32 clocks) in every mode.
- The centre is half the measured width and half the measured line count (re-latched only when it moves by
  two lines, and on the first field), virtual-1080 steps from the timing ID: the same picture at SD, 720p, 1080p.

STATUS
- Built 2026-11-17 through the shared queue with router1 (NEXTPNR_ROUTER="--router router1"), every config on
  seed 1. Post-route Fmax (the last "Max frequency" line), LC of 7680, EBR of 32:
    HD Analog  81.59 MHz   7092 LC   21 EBR
    SD Analog  85.54 MHz   7076 LC   21 EBR
    HD HDMI    82.72 MHz   7082 LC   21 EBR
    SD HDMI    78.91 MHz   7084 LC   21 EBR
    HD Dual    83.93 MHz   7085 LC   21 EBR
    SD Dual    78.49 MHz   7084 LC   21 EBR
  (HD needs 74.25, SD 27.) All six bitstreams pass iceunpack; no emulate_transparency; dupin: no carry self-adds.
  router2 was not tried.
- Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge windrose
- The first RTL (five stars, five chained blends, 27-bit accumulators) packed at 9,190 LC (120 %).
- Model = RTL (GHDL against the integer model imodel.py): see the list in VERIFIED below.
- Renders (work dir .straightedge_work/rb_windrose/): cmp_final.png, sheet.png, strip.png, motion.png,
  camera.png (all from the integer model except the RTL panels of cmp_final), ntsc_rtl.png.

VERIFIED (GHDL renders of the built sources against the integer model; mismatching pixels)
- 720p60 decimation 1, the full frame (918,720 px): defaults with a black input 0; Source = Osc 0; bear.png at
  Camera 100 % (the model fed the tester's own luma) 0.
- NTSC decimation 1, bear.png at 100 %: 0 on both fields (720 x 243 each); the same picture (the centre 4:3).
- Motion and Glide On: the power-up defaults (Glide On, Drift 660) captured after 3 and after 7 fields match the
  model after 3 and 7 drift steps exactly (0 and 0; 12,470 or more against the neighbouring step counts), so the
  travel and its direction agree. An earlier build of the same motion path gave 0 after 3 and 9 fields.
- 720p60 decimation 2 (640 x 360, the centre crop), on the RTL before two last edits (the line-count deadband
  and the 480i / 576i echo pitch, neither active in these frames): defaults 0; Field 0; Field + Ink with X 300,
  Y 700, Scale 350, 24 points, Point 200: 0; bear.png + Fine 0; Osc + Ink + Fine with X 700, Scale 700, 24 points,
  Point 900: 0; Field + bear.png with Scale 300, Camera 600, 4 points: 0 (after one fix: the one-pole was being
  loaded on the field's first TWO lines; found by this frame, 262 px in one block).
- Not rendered: 1080p / 1080i timing, 576i; the K4 hysteresis and the knob hold against real pot noise.

CUT / DEVIATIONS
- Camera target changed. The spec's "R per block" (the points lengthened inside bright blocks) floods the frame
  with the great star's two facet tones and bear.png was not recognisable at any gain; the block value now sets
  the weight of echo lines around the rose (above). Field uses the pixel's own block too, not one value per rose.
- Field is a mirror fold, not a restart of the accumulators at each cell: continuous across the borders,
  counter-rotating neighbours, and free in the datapath. Cell sizes are whole pixels / lines, so X, Y and Scale
  move the lattice in 1-px steps.
- 24 points at most, not 32: the fourth tier is one 8-point star (the spec allows the cut; a fifth star and its
  four accumulators cost about 1,040 LC). K4 is labelled Points 4 / 8 / 16 / 24.
- K1 / K2 reach +-640 / +-360 virtual px (the accumulators carry 11 integer bits).
- Drift 2^-13 .. 2^-6 quarter turn per field (see Motion).
- Grey paints the two topmost stars at a pixel by coverage over the third taken whole (or the echo): exact
  unless three partial edges share one pixel.
- Fine line weight R / 80 within 2.5 .. 6 virtual px.
- Glide Off has no easing, but every knob acts one field late (the two-bank table).
- Resolution: the geometry is the same picture at SD, 720p and 1080p (NTSC RTL-checked: the centre 4:3). Edges
  are one unit soft: 1 px at 1080p and 480p / 576p, 1.33 px at 720p, 2 px (one field line) at 1080i / 480i / 576i.
  At 480i / 576i the echo lines are ruled every 32 virtual px instead of 16 (16 is under four field lines and
  aliased), so the camera's line screen is twice as coarse there.
- field_n is passed through one register (the core ties it high on every input).
- Source note: the stage clock numbers in windrose.vhd's header ("tone at T + 23, luma at T + 24") are one
  clock short somewhere; the render showed the output filter sits on DDA index column + 5, which is what the
  header's last line, windrose.py's XOFS and the read side's block counter use, and what was built and verified.
  The model panel of cmp_final is scaled to white = 255; the RTL panels show the true 775 white.

NOT SURE OF ON HARDWARE
- Nothing on this chassis has been flashed (riley, flute's engine and the Vellum / Hardline programs are all
  unflashed); the generated avid / zone / width logic is the heirloom shell's, which is hardware-validated.
- With a camera at 100 % the whole screen is sharp near-full-swing lines that turn: the [1 2 1] + 13/16
  conditioning is built in and the Grey echo is 3/4 white, but if HD HDMI bars show, this is where (Ink + camera
  first).
- The line count is measured (g_start pulses per field); a source whose fields differ by two or more lines would
  move the centre every field.
