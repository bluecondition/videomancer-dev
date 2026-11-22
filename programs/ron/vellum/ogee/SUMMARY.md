OGEE (Vellum)
v0.1.0 built 2026-11-22, not flashed.  Flash out/rev_b/ron/vellum/ogee.vmprog.

WHAT IT DRAWS
The ogee / lantern tiling, after Moroccan zellige "arabesque" (lantern) tiles and Gothic ogee tracery: a
checkerboard of diamonds whose every edge is an S of two circular arcs, point-symmetric about the edge's midpoint,
so the tiles stay congruent and still tessellate. P12 runs the one-parameter family: plain diamonds (a harlequin)
-> lanterns (onion points top and bottom, round shoulders) -> full round, where the shoulders become half discs of a
square circle packing with pinched four-pointed stars between them. Luma only (U = V = 512 on every pixel).
Default frame: lanterns 1.2 x taller than wide, about five columns across the screen, the two tile colours as a lit
relief (tile A steps light grey -> white toward its centre, tile B dark grey -> near black: pillows and wells),
three inset steps per tile following its shape exactly, a thin black grout line, sliding slowly along the
diagonal.
  fold      x = vx / A + D, y = vy / B + D; X' = |x mod 2| - 1/2 (mirrors at every integer x, y keep the colour)
  S         side 2 when X' > Y' tan b; L = -X' sin b - Y' cos b; F2 = q^2 (X'^2 + Y'^2)
            f = L - F2 (side 2) or L + F2 (side 1) = (r^2 - rho^2) / 2r about that side's arc centre
  distance  d = f g(t), t = s 2 q^2 f, g = 2 / (1 + sqrt(1 - 2t)) (ROM): the true distance from the tile edge,
            exact at every curvature including straight (no centre at infinity); tile colour = sign of f
  steps     phase = (|d| - grout) x 6 +- camera; step = clamp(floor(phase), 0, 3)

CONTROLS
K1  X        pan, +-1920 virtual px. Glided.
K2  Y        pan, +-1088 virtual px. Glided.
K3  Scale    tile size: geometric-mean half diagonal 84 .. 672 virtual px, exponential (default 238). Glided.
K4  Slender  lattice aspect B / A = 0.57 (squat) .. 1.75 (tall) at constant area; the arcs become oval (default
             680 = 1.2, the classic lantern). Glided.
K5  Drift    the tiling slides along its diagonal (right and down for > 0); +-24 counts = still; else
             2^-12 .. 2^-4.15 of a period per field (Hypnos law). Default 660: about 17 s per period at 60 Hz.
K6  Camera   luma^2 moves the insets: tile A's steps toward the grout, tile B's toward its centre, so a lit tile
             flattens (A to white, B to its edge grey): 3.3 steps at white at 100 % (default), 0.8 at 25 %.
S7  Glide    On (default): 1/16 of the gap per field on X, Y, Scale, Slender, Camera, Curve (X, Y at least 2/8
             count), the Drift speed 1/8; landing exactly. Off: the held value this field. +-1-count hold in both.
S8  Inset    Bands (default: the insets as filled steps) / Lines (flat tile greys with the step edges drawn as
             lines, white on A, near black on B, 1/3 step wide).
S9  Tone     Grey (default, stepped greys) / Ink (black and white: Bands alternate per step; Lines in the other
             colour).
S10 Weight   Bold (default) / Fine: with Bands a line drawing on black (inset lines 1/6 step, light / mid grey by
             tile, the grout a white line); with Lines the lines thin to 1/6 step.
S11 Source   Video (default) / Osc: the shell's drifting 640 x 540 triangle lattice through the camera's table.
P12 Curve    0 % straight diamonds (nested diamonds), 50 % (default) lanterns, 100 % full round.

MOTION
One accumulator (Q18 of one period = 2 domain units along the lattice diagonal), advanced once per field in the
vblank zone (no avid for 1024 clocks), wrapping modulo exactly one period, so the slide never jumps. Speed from
Hypnos v1.1.0's decode (|v - 512| < 24 still; exponential 2^-12 .. 2^-4.15 period per field, top capped as in
cartouche: at a quarter period per field the lattice strobes). Glide eases the speed (1/8 per field), never the
phase. motion.png: the default Drift at fields 0 / 60 / 120 / 180 and Drift at both ends.

CAMERA
Never shown. luma -> a per-field 256-entry table T[i] = 4.5 (K6 >> 5) max(i - 16, 0)^2 >> 14 (luma^2, refilled
by second differences in the zone) -> 128-px box (64 at 720p / SD) -> vertical one-pole (cartouche's [D]).
Added to tile A's step phase, subtracted from tile B's: bright = lighter in both colours, every edge still a clean
curve. 25 %: 0.8 step at white (a gentle swerve of the insets); 50 %: 1.6; 100 %: 3.3 (lit tiles flatten). On
bear.png (camera.png) the dark head keeps the relief while the bright background flattens: a coarse silhouette,
the tile being the "pixel". K6 = 0 or a black input = the clean figure, bit for bit. Osc: the triangle lattice
through the same table (up to 2.9 steps: about 3.6 x the camera at mid grey, 0.9 x at full white), drifting zones
where the tiles flatten. At twice that level (the first try) almost every tile flattened and the program's own
picture was lost, so Osc stays at 1 x.

ENGINE
- Per field (vblank zone, about 600 clocks): knob holds and glides, Drift speed, then ONE 40-bit serial shift-add
  multiplier in 10 slots: 1 / A and 1 / B (2^(e/256) ROM, e from Scale and Slender), the steps per pixel / field
  line, pixel 0's x and line 0's y (minus the drift), q^2, the two product-DDA steps and their pixel-0 values;
  the edge divisor; then a 256-clock fill of the camera table.
- Per line: the same multiplier, between the zone runs: the y DDA and four products (Y' cos b, Y' tan b, q Y',
  (q Y')^2) for the next line, 98 clocks.
- Per pixel (27 stages to the luma): the x DDA (Q22 mod 2) + two product DDAs (x~ sin b, x~ q, each wrap taking 2 sin b /
  2 q off) -> fold -> L and q X' on one carry chain each -> (q X')^2 by a 128-entry square ROM + a 7 x 7 product
  -> f -> |t| (12 x 12 product) -> the g ROM (2048 x 12, fine near t = 1/2 where the circle centres sit) -> d
  (14 x 11 product) -> steps -> the nearer of the step edge and the grout edge -> one coverage (17 levels, shift
  divisor 2^k or 2^k x 32/23: edges 1 .. 1.45 px; the shift and the clamp in stages of their own) -> blend ->
  luma 64 + 9.125 t (with the black-outside gate).
- Output: luma [1 2 1] / 4, 13/16 toward black (white 777), blanking gate; generated avid (the shell's
  parity-locked start); one latency (32) in every mode; virtual 1080-line coordinates from the timing ID (SD shows
  the centre 4:3).
Chassis: Kinetic riley's [A] timing / zone / width / generated avid and [C] serial multiplier via Vellum cartouche
([B] hold + glide, [D] source field with dazzle's Osc line-0 fix, the luma half of [F]).

STATUS
- v0.1.0 built 2026-11-22 through the shared queue with ROUTER1 (NEXTPNR_ROUTER="--router router1"), every
  config on seed 1. Not flashed.
- Post-route Fmax (the builder's routed figure, first seed): HD Analog 92.49 MHz, HD HDMI 85.25, HD Dual 81.47
  (HD needs 74.25); SD Analog 94.82, SD HDMI 94.45, SD Dual 100.67 (SD needs 27).
- 7,044 .. 7,052 / 7,680 LC (91.8 %), 18 / 32 EBR. Six bitstreams pass iceunpack. No emulate_transparency in
  the full-design synth log; dupin: no duplicated-input carries or LUTs (checked on the netlist one edit before the
  final one; the final edit only split a shift from its clamp, and router1 routed every config first time).
- router2 (the builder default) on an earlier netlist: HD Analog closed only on seed 2 at 74.47 MHz and HD HDMI
  missed seeds 1 and 2; the critical paths were the glide's hold compare and the coverage shift, both since split
  into two stages. Profile of the final-but-one netlist: router2 seed 1 74.34 MHz, router1 seed 1 85.47 MHz.
- Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum ogee
- Model = RTL (GHDL against the integer model .vellum_work/rb_ogee/imodel.py, the built sources): 0 mismatching
  pixels on every render compared.
  720p60 decimation 1 (918,720 px each): the power-up defaults with Glide On and the default Drift running, field 3
  (rtl_default.png); Source = Osc (r_osc1.png); bear.png at K6 100 % (r_bear1.png, diffed against the tester's own
  stimulus luma).
  720p60 decimation 2 (228,960 px each): Ink + Fine + Curve 100 % + Scale 300 + Slender 820 + X 700 (r_v1); Lines +
  Curve 0 % + Y 300 + Scale 700 + Slender 200 (r_v2); bear.png at K6 100 % (r_bear2); Osc on an earlier netlist.
  NTSC decimation 1 (both 243-line fields, 347,976 px): 0, the same picture as HD (the centre 4:3, the same
  number of tiles top to bottom).
  Motion: the power-up defaults at fields 3 and 9 (720p / 2) both match the model; between them the RTL picture
  moved 2 px right and 2 px down (the expected 1.7 and 2.0 px): motion_rtl.png.
  Not rendered in RTL: 1080p and 1080i (the model covers them; the timing-ID steps are the shell's).
- The GHDL frame is the model's shifted 4 columns right with 4 blank columns at the left (the shell's generated-avid
  start); the camera stream is the shell's, one clock later (an extra input register).
- Renders (work dir .vellum_work/rb_ogee/): rtl_default.png, cmp_final.png (model | RTL black | RTL Osc | RTL bear,
  stacked), sheet.png, strip.png (8 frames across Curve), motion.png, motion_rtl.png, camera.png (bear.png at
  K6 0 / 25 / 50 / 100 % and 100 % Ink), presets.png. Design notes and the fit history: notes.md; float sketch
  proto.py; integer model imodel.py.

CUT / DEVIATIONS
- S8 Double (a second translucent tiling at 1.5 x) is NOT built: it cost about 1,000 LUT (the first RTL was
  7,243 LUT standalone, about 9,500 LC). Spec fallback taken: S8 = Inset Bands / Lines. A proto of the Double film
  (over at 3/8 with a black big grout) is in the work dir (e4_film38.png).
- The camera moves A's and B's insets in opposite directions (the spec: "adds to the inset offset"), so that lit
  areas read lighter in both tile colours; plain addition read as contrast, not tone.
- Edges are 1 .. 1.45 px wide (shift divisor); the camera's own gradient is not added to the edge width (the
  luma field is a 64 / 128-px box and a slow one-pole, so the steps it bends stay about a pixel wide).
- Distance precision: within 0.3 px (99.9th percentile) at every curvature; up to about 1 px only within a few
  pixels of the arc centres at full round (the g ROM's square-root singularity).
- LC: 7,044 .. 7,052 (91.8 %), inside the set's 7,100 budget but above the ~6,900 guide; the timing fixes
  (two pipeline splits) took it from 6,977 to there, and every HD config closes with 9.7 % or more margin.
- Fine lines are 1/6 step (about 7 virtual px at the default Scale); at the smallest Scale with the squattest
  Slender a horizontal stretch of a Fine line falls to about 1.8 virtual px (anti-aliased, never a hard 1-row line).
- SD: the same picture (the centre 4:3, the same tile count top to bottom); at the smallest Scale a 480i field
  line is 4.4 virtual px, so the grout there is under one field line (anti-aliased).
- field_n is passed through one register (the core ties it high on every input).
