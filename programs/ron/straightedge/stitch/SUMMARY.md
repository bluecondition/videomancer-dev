STITCH (Straightedge)
v0.1.0 built 2026-11-22, not flashed. Flash out/rev_b/ron/straightedge/stitch.vmprog.

WHAT IT DRAWS
Curve stitching / string art (Mary Everest Boole's sewing cards, Naum Gabo's strung constructions): families of
perfectly straight lines whose envelope the eye reads as a curve. Every edge on the screen is a straight line; the
curves exist only where the lines crowd. Luma only (U = V = 512 on every pixel).
- Form = Corners (default): the four corners of an implied square (+-H about the fold centre) are each stitched:
  in each corner N straight lines tangent to the parabola sqrt(u) + sqrt(v) = sqrt(L) (u, v measured from the
  corner along its two sides; every such line joins a pin at a on one side to a pin at L - a on the other, the
  sewing-card rule). The lines are NOT clipped: they run on across the screen. Bold: the lines are region
  borders and the regions take the number of lines on their far side, mod 4, as black / dark / light / white, so
  the screen is a weave of sharp quadrilaterals that crowds into four parabolic envelopes round a clear
  four-pointed black centre.
- Form = Cross: the second classic figure, Gabo's hyperbolic fans: the tangents of the rectangular hyperbola
  x'y' = k (a four-armed star whose arms run off along the axes) and of its conjugate (rays fanning across the
  diagonals).
One fold (x' = |x - X|, y' = |y - Y|) makes one family serve all four quadrants; the four quadrants meet exactly on
the fold axes.
Pins are spaced at equal steps of the lines' normal angle over a half turn (not equal distance; see Cut /
deviations): both ends of that half turn are lines at infinity (Corners) or on the fold axes (Cross), which is what
lets the drift wrap exactly and lets the lines run off at one end and come back at the other.

CONTROLS
K1  X        the fold centre, +-640 virtual px. Glided.
K2  Y        +-360 virtual px, up = up. Glided.
K3  Scale    the implied square H = 117 .. 330 (default) .. 536 virtual px, exponential (Cross: sqrt(k) = 0.6 H).
             Glided.
K4  Pins     N = 8 / 12 / 16 / 24 (default) lines per quadrant. Stepped, 8-count hysteresis, both ends reachable.
K5  Drift    every line's angle advances by f steps: the lines sweep round the envelopes, run off to infinity and
             come back from the other side. Hypnos law: still within 24 counts of the centre, else 2^-12 .. 2^-4
             line step per field. Default 680 (a step in 13.5 s).
K6  Camera   how strongly the held block level lifts the region tones (Ink: flips the parity). Default 100 %.
             Glided.
S7  Glide    On (default): K1 K2 K3 K6 P12 ease 1/16 of the gap per field (never under half a count, landing
             exactly), the Drift speed 1/8. Off: no easing. A knob must move 2 counts to register (pot jitter).
S8  Form     Corners / Cross.
S9  Tone     Grey (the line count mod 4 as four flat tones) / Ink (the parity: pure black and white).
S10 Weight   Bold (filled regions) / Fine (each line a 3-virtual-px white thread: the classic drawing; Grey: lines
             0.62 white, crossings white, so the envelopes glow where the threads crowd; Ink: crossings XOR black).
S11 Source   Video (default) / Osc.
P12 Span     Headline. Corners: the pin span L against the square: L = 5.5 H at 0 % (a small dense four-pointed
             star in a field of big checks), 2 H at 50 % (the classic: the pins of each corner span the whole side;
             a curved diamond of clear black ringed by the weave), 0.73 H at 100 % (the climax: a wide clear centre
             with the weave crowded into a ring round it and radiating to the screen edges). Cross: Hc = 0.5 sqrt(k)
             (a small star in chevrons) .. 0 (the infinite cross) .. -1.25 sqrt(k) (the arms split apart). Glided.

MOTION
One drift fraction f (2^16 = one line step), stepped once per field by the signed speed (Hypnos SPEED decode:
+-24 counts still, then exponential 2^-12 .. 2^-4 step per field; the top is capped at 1/16 step per field because
a quarter step per field strobes the weave). Every line's normal angle is phi0 + (l + f) pi / N, so the whole
family rotates about its conic: the envelopes stay put while the weave streams along them. At f = 1 the set of
lines is identical to f = 0 (the line leaving at one end of the half turn is at infinity, or on a fold axis, exactly
where the next one enters), so f wraps modulo one step and nothing jumps. Cross: the two ends are the same axis line
with opposite orientation, so the engine counts the wraps (mod 4) and rotates the tone table to match; the wrap is
invisible. Default 680: one step (7.5 degrees at 24 pins) in 13.5 s; Drift > 512 turns the lines one way, < 512 the
other. Only Drift at the centre stops it.

CAMERA
The camera never bends a line. The luma (or Osc) is averaged exactly over 32 x 32 virtual px blocks (60 x 34 at
1080p / 720p, 45 x 34 at SD; windrose's block sums and per-column one-pole), posterised to four levels (block mean
above 33 / 47 / 62 % of white) and written, 2 bits, into a block-RAM grid of two banks swapped every field. The next
field reads the pixel's own block; the level is constant across the block and selects one of four rows of the
per-field tone table: it is a FLAT TONE STEP on the regions.
- Grey Bold: every region's tone is lifted toward light by level x K6 (level 3: the weave becomes a light / white
  ghost of itself). Ink: the parity is flipped in the brightest levels, how many set by K6 in quarters (1 level at
  25 %, all 3 from 75 %). Fine Grey: the black ground under the threads is lifted by level x K6.
- Every line stays straight; the only new edges are straight block borders.
- 0 % or a black input: exactly the clean figure. 25 %: a visible lean (bright areas a quarter lifted). 50 %: half.
  100 %: bear.png is recognisable (ears, nose and eyes, the dark body) as a mosaic of the weave's own tones.
- Osc: the shell's drifting 640 x 540 triangle lattice in place of the luma: big stepped diamonds of all four levels.

ENGINE
- Per field: Hardline flute's microcoded engine as in Straightedge windrose (stitch.py assembles the microprogram,
  976 words; 37-bit accumulator, 256 x 32 register file, serial 18 x 18 multiplier). It does the knob holds and
  glides, the drift and its wrap count, the scale / span / hyperbola, the fold position (whole or half pixels and
  lines), and for each of the 24 lines its angle, sin / cos (256-step quarter table, interpolated), the conic offset
  (Corners: L s c / (s + c) through a normalised reciprocal, 256-entry table + one Newton step, clamped to 2048 units
  for the lines near infinity; Cross: sqrt(2k) x an interpolated sqrt(sin) table), and its five DDA words (pixel
  step, forward / backward / held line wrap, field seed), then the 128-entry tone table; about 36 k clocks. It is
  started at every vblank zone ("no avid for 1024 clocks") and writes one bank of a two-bank table WHILE the other
  bank is being drawn; the next zone start loads the fold constants and seeds from the finished bank. So a knob
  acts one field later (the pixel path's Tone / Weight copies are delayed one field to match the table).
- Per line: no multiplier. 24 accumulators (25 bits, Q12 of the anti-aliasing unit), each with its own increment
  register: the pixel step on every clock of the line window (width + 6 steps), complemented with carry-in at the
  fold column (one held clock when X falls between two pixels), then one add of the line wrap parked in the same
  register (the loader: 48 table reads in the line blanking); the y fold picks the forward, backward or held wrap.
  Lines beyond N get a seed far outside the screen.
- Per pixel (no multiplier): the distance in 1/8 unit, saturated -> coverage 0 .. 8 (Bold clamp(d + 1/2), Fine
  clamp(T - |d| + 1/2)) -> a four-stage adder tree (S = 0 .. 192) -> the tone table (camera level & S mod 32, Fine
  Grey: min(S, 31)) -> luma -> [1 2 1] / 4 -> 13/16 toward black (white 777) -> blanking gate. Generated avid (the
  shell's parity-locked start), one latency (20 clocks) in every mode; output column c is the [1 2 1] filter
  centred on DDA index c + 1.
- The anti-aliasing unit is 1 virtual px at 1080p, 2 at 720p / 1080i / 480p / 576p, 4 at 480i / 576i; the centre
  is half the measured width and line count (re-latched only when the line count moves by two, and on the first
  field); virtual-1080 steps from the timing ID: the same picture at SD, 720p and 1080p.

STATUS
- Built 2026-11-22 through the shared queue with router1 (NEXTPNR_ROUTER="--router router1"), every config on
  seed 1. Post-route Fmax (the builder's figure is the last "Max frequency" line), LC of 7680, EBR of 32:
    HD Analog   96.53 MHz   5725 LC   18 EBR
    SD Analog   99.55 MHz   5710 LC   18 EBR
    HD HDMI     95.52 MHz   5720 LC   18 EBR
    SD HDMI     92.70 MHz   5720 LC   18 EBR
    HD Dual     94.23 MHz   5717 LC   18 EBR
    SD Dual    102.04 MHz   5714 LC   18 EBR
  (HD needs 74.25, SD 27.) All six bitstreams pass iceunpack; no emulate_transparency (standalone yosys); dupin:
  no carry self-adds or duplicate LUT inputs. router2 was not tried.
- Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge stitch
- First RTL fitted at once (standalone yosys 3663 LUT / 2558 FF); no cuts were needed.
- Renders (work dir .straightedge_work/rb_stitch/): cmp_final.png, sheet.png, strip.png, motion.png, camera.png
  (all from the integer model imodel.py except the RTL panels of cmp_final), rtl_default.png, ntsc_rtl.png.

VERIFIED (GHDL renders of the built sources against the integer model; mismatching pixels)
- 720p60 decimation 1, the full frame (918,720 px): defaults with a black input (Glide Off, Drift still, Pins 24)
  0; Source = Osc 0; bear.png at Camera 100 % (the model fed the tester's own luma) 0.
- Power-up defaults exactly as the TOML (Glide On, Drift 680, Pins 24; black input), 720p60 decimation 1: matches
  the model after 3 engine runs (drift fraction 243 / 65536 of a step), 0 mismatching; 39,932 / 27,548 against
  2 / 4 runs. This frame is rtl_default.png.
- Motion (720p60 decimation 2, power-up defaults, Glide On, Drift 680): the capture after 3 fields matches the
  model after 3 drift steps and the capture after 7 fields matches 7 steps (0 and 0; 8,756 .. 12,452 against the
  neighbouring step counts), so the travel and its direction agree.
- NTSC decimation 1, bear.png at 100 %: 0 on both fields (720 x 243 each); the same picture (the centre 4:3).
- 720p60 decimation 2 (640 x 360, the centre crop): Pins 16 defaults 0; Cross + Ink + Fine with Pins 8 (knob at 0),
  X 300, Y 700, Scale 900, Span 200: 0; bear.png + Ink with Camera 600, Pins 12, Scale 200, Span 900: 0; Cross +
  Fine + Osc with Scale 100 % and Span 100 %: 0.
- The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the shell's generated-avid
  start). Chroma is 512 on every pixel; luma 64 .. 777.
- Model checks: the engine's 18-bit operand ranges asserted over 3,888 runs (six video modes, every knob at its
  ends, both Forms, Ink, Fine); the largest accumulator value on screen over the extreme knob corners at 1080p is
  3,179 units of the 4,096 the 25-bit accumulators hold; the drift wrap changes 0 pixels in Corners and, with the
  wrap counter, about 1,300 pixels (the lines next to the axes) in Cross (230,000 without it).
- The renders ran on the sources of the build before last; the final build changed only one header comment of
  stitch.vhd (the Scale range), and the two Pins-16 / Osc / default frames before the Scale change are identical at
  Scale 50 % (the changed microcode gives the same value there).
- Not rendered: 1080p / 1080i timing, 576i; the Pins hysteresis and the knob hold against real pot noise.

CUT / DEVIATIONS
- The pins are spaced at equal steps of the lines' NORMAL ANGLE over a half turn, not at equal distance along the
  sides. With equal distances the drift cannot wrap (the line leaving the window and the one entering it are
  different visible lines: a pop every step) and the lines cannot run on across the screen without leaving the
  screen's corners empty. Every line is still a tangent of the same parabola (a pin at a joined to a pin at L - a),
  so the figure is the sewing-card figure; the pins are finer near the vertex and coarser toward the sides.
- P12 is the span L of the pins against the implied square (the spec's pairing offset K is L / pin pitch); it
  runs so that 0 % is the spec's "small dense star in a field of large diamonds" and 100 % its "widest weave"
  (L / H = 5.5 .. 2 .. 0.73). Labelled "Span".
- Cross is the rectangular hyperbola's tangent family and its conjugate (a four-armed star plus diagonal fans), not
  a fold swap of the second pin: the swap gives a pencil of lines through one point (no envelope). Its drift wrap
  is made seamless by a wrap counter that rotates the tone table (the end lines are one axis line with opposite
  orientations).
- Camera: a flat tone step on the regions (the spec's named fallback), not an offset of f per block: an f offset
  per block needs a different line set per block (impossible with one DDA per line), and the possible substitute
  (all lines shifted along their normals per block) tore every line at every block border and the bear was not
  recognisable. Four posterised levels; a thread-width camera for Fine was tried and was too weak.
- No "two edges in one pixel -> mid-grey" override: modelled, it changed about 0.1 % of the pixels invisibly.
- Scale runs H = 117 .. 330 .. 536 virtual px (1.5 octaves down, 0.7 up): with a 1.5-octave top, Scale 100 % and
  Span 100 % together left the screen 95 % black.
- K1 / K2 reach +-640 / +-360 virtual px (the accumulators carry 12 integer bits).
- Drift 2^-12 .. 2^-4 step per field (see Motion).
- Glide Off has no easing, but every knob acts one field late (the two-bank table).
- Resolution: the geometry is the same picture at SD, 720p and 1080p. Edges are one unit soft: 1 px at 1080p and
  480p / 576p, 1.33 px at 720p, 2 px (one field line) at 1080i / 480i / 576i. The Fine threads are 3 virtual px (3
  px at 1080p, 2 px at 720p, a 1.5-px grey line at SD).
- Cross at the instant a line passes through a fold axis (f = 0 and every half turn of a line) shows a faint
  one-pixel line on that axis for a few fields: the line emerging from the axis (the geometry, not a glitch).
- field_n is passed through one register (the core ties it high on every input).

NOT SURE OF ON HARDWARE
- Nothing on this chassis has been flashed (windrose, flute's engine and the Vellum / Hardline programs are all
  unflashed); the generated avid / zone / width logic is the heirloom shell's, which is hardware-validated.
- The whole screen is sharp near-full-swing edges that move: the [1 2 1] + 13/16 conditioning is built in; if HD
  HDMI bars show, Ink at Span 100 % (the densest, fastest-changing weave) is where to look first.
- The camera level of a block can flicker between two levels when its mean sits on a threshold; at 480i / 576i the
  two fields of a source carry different lines, so near-threshold blocks may alternate field to field (seen as a
  comb in the tester's woven NTSC view, ntsc_rtl.png).
- The Pins hysteresis band (8 counts) and the 2-count knob hold are untested against real pot noise.
