# STREAMLINE (Hardline, wave 2)

**v0.1.0 built 2026-11-22, not flashed.** All six configs on router1, seed 1; HD timing closed (77.8 .. 84.4 MHz); 7,178 .. 7,197 LC (94 %), 23 EBR.

## What it draws

Potential flow past a cylinder, full screen, black and white. Art reference: the streamline photographs
in Van Dyke's "An Album of Fluid Motion", Hele-Shaw cells, Prandtl's drawings, the textbook flow net;
with spin, the Magnus effect.

    W = z + R^2/z - i G log z        (y down the screen, G = 2 s R)
    psi = y (1 - R^2/r^2) - G ln(r/R)     streamlines (period P)
    phi = x (1 + R^2/r^2)                 the dash cuts (period 4P)

- Level bands part round a white disc (with a thin black ring), squeeze over its top and bottom, stall
  at the two stagnation points fore and aft and close again behind it. Far from the disc the bands are
  straight; most of the screen stays calm.
- Dashes (default): each band is cut by a quarter-period break every four periods (psi XOR phi), so the
  bands read as long dashes; the cut lines are the flow's equipotentials, bending round the disc. The
  cuts travel along the streamlines (K5 Drift).
- Lines: the streamlines alone (the Hele-Shaw picture). Fine weight: thin streamlines on black; Fine +
  Dashes is a flow net (thin streamlines OR thin equipotentials).
- Open body: the formula continued inside the circle, the doublet's flower of circles (an eye), going
  flat grey where the circles get finer than a pixel.
- Spin lifts the flow: the stagnation points slide down round the disc, meet below it and leave it, and
  closed loops form round the body.
- Lines finer than about 2 px fade to flat grey (near the disc at fine periods); edges are anti-aliased
  over one pixel everywhere.
- Default frame: Dashes, Solid, the disc (180 px radius, a third of the screen high) a little left of
  centre, no spin, the dashes drifting left to right at about 40 px/s (1080p).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the disc centre, +-1/2 screen width (the centre reaches the frame edge); detent at the centre. Default 416 (40 % of the width) |
| K2 | Y | the disc centre, +-1/2 screen height; detent. Default 512 |
| K3 | Period | the streamline spacing, 300 -> 13.3 virtual px, exponential (clockwise = finer). Default 400 (89 px) |
| K4 | Spin | the circulation G = 2 s R, s = 1.6 t abs(t) (bipolar, detent); clockwise lifts the flow (stagnation points slide down). G moves in quarter-octave steps (2-bit mantissa). Default 512 (none) |
| K5 | Drift | bipolar (Hypnos v1.1.0 Speed shape): centre +-24 counts still; up = dashes run left to right, down = right to left; exponential 2^-12 .. 2^-2 dash period per field. In Lines the streamlines creep instead. Wraps exactly one period. Display -100 .. 100 %. Default 680 (+33 %: 0.0019 dash period per field) |
| K6 | Camera | box-filtered luma added to psi: 2 periods at white at 100 %; 0 = the clean figure. Default 100 % |
| S7 | Glide | On: X, Y, Period, Spin, Drift's speed, Camera and Body ease 1/16 of the gap per field, never slower than half a count, landing exactly. Off: no easing (the value latched this field). Glide eases the drift SPEED, never the phase. Default On |
| S8 | Weave | Dashes (psi XOR phi; Fine: a flow net) / Lines (psi alone). Default Dashes |
| S9 | Body | Solid (white disc, 6-virtual-px black ring) / Open (the doublet's eye). Default Solid |
| S10 | Weight | Bold (equal bands, quarter-period cuts) / Fine (1/6-duty streamlines on black; Dashes: thin cross-lines too). Default Bold |
| S11 | Source | Video / Osc (the shell's slow triangle lattice in the camera's place, at twice its level). Default Video |
| P12 | Body | the disc radius: 20 virtual px (a bead that barely dents the bands) .. 180 px at 50 % .. 499 px at 100 % (a great disc: the flow crushed into thin bands above and below, fanning wide fore and aft). Default 50 % |

Every knob has a +-1-count hold against pot jitter (about the glided value; latched unconditionally on the
first field). No stepped knobs.

## Motion

K5 Drift advances one per-field phase accumulator (Q20, one cycle = one dash period, wraps without a seam),
added to phi in Dashes (the cuts travel along the streamlines) and to psi in Lines (the streamlines creep
sideways). Hypnos shape: dead zone +-24 counts round the centre, exponential 2^-12 .. 2^-2 cycle per field
(0.246 at both ends: about 15 dash periods a second, fast but directional). Glide eases the speed through
the knob ring (1/16 per field), never the phase; only Drift at the centre stops the motion. Note: a phase
advance moves a cut at rate / |grad phi| px per field, so cuts move slowest (in px) where the flow is
fastest (the cells are smaller there) and sweep fast round the stagnation points, where the dashes are
long; in cells per second the motion is uniform.

## Camera

The camera is never shown. Its luma (minus black) is scaled by K6 through a per-field gain table, box-
filtered over 32 px, low-passed vertically (1/16 per line on every 8th column: wider than the shell's 1/8,
fewer islands on the bear's vertical edges) and interpolated back to pixel rate, then added to psi: up to
2 periods at full white at 100 % (0.5 at 25 %, 1 at 50 %). The streamlines swerve round the subject's
tonal edges. A black input or K6 = 0 gives exactly the clean figure. Osc substitutes the triangle lattice
(640 x 540 virtual px, drifting) at twice the camera level (up to 4 periods). At 100 % on bear.png the
head outline and the nose read as swerves and loops, but the subject is only loosely recognisable (the
disc sits over the middle of the face at the default X); the bright blotchy background makes islands,
and a few specks of a few pixels appear just right of the disc round the nose (anti-aliased, no per-line
jaggies). 25 % and 50 % are clean swerves.

## Engine

Hypnos v1.1.0 polar chassis by way of hardline multipole (timing-ID virtual-1080 steps, generated avid,
log2 r from two integer logs and a table, one-pixel box-filter anti-aliasing) with the source field of
kinetic riley. No per-pixel multiplier wider than 8 x 5; no divide, square root or CORDIC.

- Per field (vblank sequencer, started by the guarded vsync edge): the seven knobs through one hold +
  glide ring; log2 P (Q8, 1/256-octave steps) and 1/P from a 2^(j/256) table (the same quantisation, so
  the near field 2^(log2|y| + 2 log2 R - 2 log2 r - log2 P) and the far field y/P agree); the disc
  centre, x/4P at pixel -2 and log2 R (serial multiplier); the spin gain as a 2-bit mantissa and a shift;
  the disc-edge scale; the drift decode and accumulator; the camera gain table.
- Per line: y/P = |dy| x 1/P on the serial multiplier (the accumulator starting at the psi offset),
  loaded 23 clocks into the line.
- Per pixel, 47 stages, latency 49 clocks in every mode: S1-S15 log2|x|, log2|y|, log2 r; S16-S24
  dL = log2 r - log2 R, the two exp2 terms y k/P (interpolated) and x k/4P, the spin term (dL x mantissa,
  one shift), the disc distance; S25-S26 psi (18 bits, mod 4 periods) and phi; S27-S33 the gradient
  from the horizontal differences (conformal map: |grad psi| = |dpsi/dx + i dphi/dx|, L1 norm, an
  eighth-octave log, Hypnos's Weight knee and half-octave width; Open: flat where k/P passes 1 cycle
  per px); S34-S39 Hypnos's box filter of each band square wave; S40-S41 the weave (|a - b| or max);
  S42 the disc (white inside, black ring, figure beyond); S43-S47 luma 64 .. 940, [1 2 1]/4, 13/16
  toward black (white about 776), blanking gate. U = V = 512 on every pixel.
- 23 EBR: 10 table ROMs (log2 value + slope for hi and lo, log2 r value + slope, exp2 value + slope for
  y, exp2 value for x, the sequencer's 2^(j/256)), 10 for delay lines (log2 hi / lo 3, spin + flags 2,
  psi / phi 3, disc weights 1, syncs 1), gain table, box delay, IIR store.

## Status

- v0.1.0 (2026-11-22 08:34): router1 (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1.
  Post-route Fmax: HD Analog 79.28, HD HDMI 84.43, HD Dual 77.81 MHz (Fmin 74.25); SD Analog 76.56,
  SD HDMI 80.48, SD Dual 79.15 MHz (Fmin 27). 7,178 .. 7,197 / 7,680 LC (94 %, about 80 .. 100 over the
  ~7,100 target), 23 / 32 EBR. Six bitstreams pass iceunpack; no `emulate_transparency`; dupin.py: no
  duplicate-input cells (standalone synthesis).
- The build before it (07:12, identical but for one register width) had a wrap: for K3 above 910 the
  period jumped coarse (`9 x K3` in 19 bits). Found on a K3 1023 RTL render, fixed, and every knob now
  swept through the sequencer arithmetic at the RTL widths against the model (0 mismatches).
- An earlier build of a larger version (v6 in notes.md: 7,274 .. 7,292 LC) routed HD at 79.2 .. 81.9 MHz
  on router1 seed 1 (HD Dual 80.26).
- vmprog: `out/rev_b/ron/hardline/streamline.vmprog` (built from these sources).

Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline streamline`

Verification (GHDL image tester against the integer model .hardline_work/rb_streamline/imodel.py; the
capture sits 4 columns right of the model: the generated avid):
- 720p decimation 1 (1280 x 720), black input, Drift still, Glide off: 0 differing pixels of 918,720.
- 720p decimation 1 at the power-up defaults (Glide ON, Drift 680; rtl_default.png): 0 of 918,720
  against the model with 2 fields of drift travel (the knob ring snaps on the first field).
- NTSC 480i decimation 1 (a 720 x 243 field): 0 of 173,988; shown 4:3 it is the centre 4:3 of the 720p
  frame (ntsc_vs_720p.png; mean difference 15 / 255 after resizing: the coarser field only).
- 720p decimation 1: Source Osc 0 of 918,720; bear.png at K6 100 % 0 of 918,720 (the stimulus luma).
  Also 0 at 640 x 360 for both.
- 320 x 180 (720p decimation 4), 0 each: the default; Lines with Spin 900; Spin 100 at Period 700;
  Spin 560 (the smallest steps); Open + Lines at Body 800; Fine net with the disc far left (K1 200); X 0 /
  Y 1023 / Body 300 (the centre off the left edge); X 1023 + Spin 300 + Lines; Body 0 with Y 0; Fine Lines
  at Period 400; Period 1023 in Bold Dashes, Bold Lines + Spin 1023, Fine Lines, Open; Period 0 + Spin 0.
- The 1280 x 720, NTSC and motion renders above ran on the 07:12 RTL; the final one differs only in the
  K3 register width, which changes nothing below K3 910 (the default is 400); the default was re-checked
  at 320 x 180 on the final RTL (0).
- Motion (640 x 360, Glide off): the default Drift after 2 and 12 fields matches the model with exactly 2
  and 12 fields of travel (0 differing pixels each), and the RTL frames move the dash cuts 5 px to the
  RIGHT in those 10 fields (4.5 px expected). Drift 900 and 124 after 2 fields: 0 each against the model;
  measured +19 px (right) and -23 px (left) against the still frame.
- Not simulated: 1080p / 1080i rasters, a glide in motion (the tester's knobs are static), knob changes
  mid-run, the slow drift's wrap (it is the same 20-bit accumulator as the fast one).

## Cut / deviations

- Dash period 4P (spec: about 3P): a power of two makes the dash box filter the streamline width >> 2.
  Bold cuts are a quarter of the dash period (a half-duty XOR read as a generic brick checkerboard on the
  renders); Fine + Dashes is OR, a flow net (thin AND half left an empty black screen).
- The dash cuts carry no circulation term (phi = x (1 + k)): with spin they are the equipotentials of the
  spin-free flow, so near a spinning body they cross the circling streamlines at an angle instead of
  square. The m theta term (integer m, seamless) was built first and cut for fit (about 430 LC with its
  angle path and two table ROMs).
- Spin's fine steps cut (the spec's first cut): G moves in quarter-octave steps (2-bit mantissa x 2^e);
  turning Spin steps the picture by up to 25 % of the circulation per step. Spin is quadratic in the knob
  (s = 1.6 t|t|): linear spin swamped the far field with the vortex at a quarter turn.
- X / Y reach +-1/2 screen (the disc centre on the frame edge), not 2 screens: with the disc off frame the
  picture is plain bands. Travel is in virtual px, so SD shows the centre 4:3 (the disc can sit off the
  4:3 frame at the knob ends).
- Camera vertical IIR 1/16 instead of the shell's 1/8.
- Drift: see Motion (a phase advance is uniform in cells, not faster where the bands squeeze).
- Precision trims for fit: the dash (x) exp2 term has no interpolation (its cut lines step by under
  0.3 px at the default, about 1 px at the finest period with the great disc); the log2 r table is
  interpolated with 5 bits (band wobble under 0.5 px only at the finest period round the great disc);
  the AA width comes from an L1 norm and an eighth-octave log (edges 0.8 .. 1.5 px).
- At SD (and in the decimated sims) the near and far field use the integer dy of each line, as the polar
  engine does, so the streamline psi = 0 hugs the disc in every mode.
- Over budget: 7,178 .. 7,197 LC against ~7,100 (94 %); timing closed on all six with router1 seed 1
  (the tightest, HD Dual, at 77.8 MHz).
