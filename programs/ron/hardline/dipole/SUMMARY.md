# DIPOLE (Hardline)

**v0.1.0 built 2026-11-22, not flashed.** All six configs build on router1, seed 1; HD timing closed (HD Analog with a thin margin); 7,394 .. 7,415 / 7,680 LC (96.5 %, over the ~7,100 LC budget), 16 EBR.

## What it draws

The bipolar coordinate chart, full screen, black and white: one family of circles through both poles
(Faraday's lines of force between two charges) crossed at right angles by the circles round each pole.
Art reference: Faraday's lines-of-force drawings and the bipolar coordinate chart.

    poles at the midpoint +- p, p = a e^{it};   z - p = r1 e^{i th1},  z + p = r2 e^{i th2}
    Opposite:  w = log((z - p)/(z + p)):       sigma = th1 - th2,  tau = log2 r1 - log2 r2
    Like:      w = log(z^2 - p^2) - 2it - 2 log a:  sigma = th1 + th2 - 2t,  tau = log2(r1 r2 / a^2)

- N bands per 2 pi on both families, so the cells are curvilinear squares; Checker = sigma band XOR tau
  band, Lines = the sigma family alone (the lines of force).
- Cells shrink into each pole without end and fade to flat grey where they fall under 2 px (the two
  "eyes"); between and around the poles they open into broad curved fields.
- The default frame: Checker, Opposite, N = 12, the axis tilted 20 degrees, the pair a little right of
  and above centre, both eyes on screen, the checker streaming out of the left pole into the right one.
- Like: two sources; the lines repel across a saddle at the midpoint and the outer tau bands are
  Cassini ovals (the lemniscate through the saddle is a band edge).
- P12 Gap is the headline: from a tight two-lobed flower of circles (poles 128 virtual px apart) through
  two eyes to an almost uniform field crossing the screen (poles ~2 screens apart).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the midpoint of the pair, +-0.75 screen widths (0.25 screen past the edge at the ends); detent at centre. Default 560 |
| K2 | Y | the midpoint, +-0.75 screen heights; detent. Default 490 |
| K3 | Period | bands round each pole: 4 5 6 8 10 12 16 20 24 32 (ten steps, hysteresis 8 counts, both ends reachable); tau bands take the matching spacing so cells stay square. Default 12 |
| K4 | Turn | the axis angle, a full turn over the knob (-180 .. 180 deg). Default 569 (+20 deg) |
| K5 | Drift | bipolar (Hypnos v1.1.0 speed): centre +-24 counts = still; the tau bands stream out of one pole into the other and the sigma bands creep round at half the rate the other way; each wraps exactly one band. 2^-12 .. 0.248 cycle per field, exponential. Default 700 (+37 %: 0.0027 cycle per field, a band every ~3 s at 60 Hz) |
| K6 | Camera | how far the box-filtered luma displaces the net, 0 .. 100 %. Default 100 % |
| S7 | Glide | On: X, Y, Turn, Drift speed, Camera and Gap ease 1/16 of the gap per field (never slower than half a count, landing exactly). Off: no easing, the value latched at the field start is used that field. Glide eases the drift SPEED, never the phase. Default On |
| S8 | Weave | Lines (the lines of force) / Checker. Default Checker |
| S9 | Charge | Opposite (a source and a sink) / Like (two sources: a saddle, Cassini ovals). Default Opposite |
| S10 | Weight | Bold: equal bands. Fine: 1/6-duty lines on the integer levels (with Checker a net of both families); lines never thinner than ~2 px, they fade to grey first. Default Bold |
| S11 | Source | Video / Osc (the shell's drifting triangle lattice at 2 x the camera level). Default Video |
| P12 | Gap | the separation 2a, 128 .. 4008 virtual px, exponential. 0 % a tight flower, 50 % two eyes, 100 % an almost uniform field. Default 560 |

Every knob has a +-1-count hold against pot jitter (latched unconditionally on the first field).

## Motion

K5 drives two per-field accumulators (Q20, one cycle = 2^20): tau += step, sigma -= step / 2, each
wrapping modulo exactly one band period, added at the band phase before the compare, so the travel never
jumps.  Positive Drift streams the tau circles out of the left (z + p) pole into the right (z - p) pole
at the default 20 deg axis.  Glide eases the speed, never the phase; nothing stops the motion but the
dead centre of K5.

## Camera

The camera is never shown.  Its luma (minus black) is scaled by K6 through a per-field gain table,
box-filtered over 64 px, low-passed vertically (1/16 per line on every 8th column) and interpolated back
to pixel rate.  The result DISPLACES the sample point of the whole net along the diagonal: about 30 / 60 /
120 virtual px at full white for K6 = 25 / 50 / 100 % (the gain is scaled per mode so SD, 720p and 1080p
move the same distance).  Bright areas push the lines of force and the circles round the poles aside; it
is a true displacement, so every edge stays a smooth curve and broad cells never posterise.  A black input
or K6 = 0 gives exactly the clean figure.  Osc substitutes the shell's slow triangle lattice (640 x 540
virtual px, drifting) at 2 x the camera level (up to ~220 px).

## Engine

Hypnos v1.1.0 polar chassis (timing-ID steps, generated avid, knob rings, log-polar engine, box-filter AA,
output conditioning) with the source field of Kinetic riley.  No per-pixel square root, divide or wide
multiplier.

- Per field (vblank sequencer, started by the guarded vsync edge after active video): the knob ring, then
  one 17-step serial MAC with a table sub-sequencer: the midpoint (cx, cy), a = 2^(log2 a) from an
  interpolated 2^x table, cos t and sin t from an interpolated quarter-sine table, a cos t and a sin t
  (Q8), the Like frame term -2Nt, the gradient constant log2(N / 2pi) + log2(2a), the drift decode and
  accumulators, the K6 gain table refill.
- Per pixel, latency 40 clocks in every mode:
  S1 Q4 coordinates of THIS pixel's pole (k even: z - p, odd: z + p), displaced by the camera;
  S2-S9 |x|, |y| -> hi / lo -> two 17-bit integer logs; S10-S16 u = log2(hi/lo) -> log2 r and the angle
  (interpolated tables);
  S17-S18 the other pole = the average of the neighbouring pixels (one engine runs both poles at half
  rate; its error is felt only within a few px of a pole, where the cells are grey);
  S19-S25 sigma, tau, log2 |grad| (one gradient: the map is conformal; Like adds log2 rho ~ max + min/2),
  N sigma and N (ln2 / 2pi) tau (each one adder of two shifts), the drift offsets;
  S22-S26 the AA width (Hypnos knee, half-octave reciprocal); S27-S32 Hypnos's box filter per family;
  S33-S34 the weave (XOR / OR in 17 levels); S35-S39 luma, [1 2 1]/4, 13/16 toward black (white 775),
  blanking gate.  U = V = 512 on every pixel.
- 16 EBR: log and polar tables, the 2^x / sine table, two delay lines, the gain table, box and IIR stores.

## Status

- v0.1.0 (2026-11-22 07:20): router1 (`NEXTPNR_ROUTER="--router router1"`), all six configs on seed 1.
  Post-route Fmax (builder, routed): HD Analog 76.04, HD HDMI 85.87, HD Dual 84.98 MHz (Fmin 74.25);
  SD Analog 81.73, SD HDMI 81.18, SD Dual 84.43 MHz (Fmin 27).  7,394 .. 7,415 / 7,680 LC (96.5 %),
  16 / 32 EBR.  Six bitstreams pass iceunpack; dupin.py: no duplicate-input cells; scan.py: 0 sites.
- vmprog: `out/rev_b/ron/hardline/dipole.vmprog` (built from these sources).

Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline dipole`

Verification (GHDL image tester against the integer model `.hardline_work/rb_dipole/imodel.py`; the capture
sits 4 columns right of the model):
- 720p60 decimation 1 (1280 x 720), every control at its power-up default (Glide On, Drift 700 moving),
  black input: 0 of 918,720 pixels differ (2 fields of travel).  This frame is rtl_default.png.
- 720p decimation 1, Source Osc, K6 100 %: 0.  720p decimation 1, bear.png as the camera, K6 100 %: 0.
- NTSC 480i decimation 1 (a 720 x 243 field): 0 of 173,988; shown 4:3 it is the centre 4:3 of the 720p
  frame (the poles land in the same places, cmp_ntsc.png).
- 640 x 360 (720p decimation 2), Drift still, Glide Off: defaults; Like + Fine (N 8, Gap 640); Lines at
  N 32 (K3 1023) and Gap 200; K3 at 0 (N 4); Osc; bear.png: 0 each.
- Motion (640 x 360): Drift +85 % and -85 %, captured after 2 and after 4 fields: 0 each against the model
  with exactly that many fields of travel (rtl_motion.png).
- Not simulated: 1080p / 1080i rasters, a glide in motion (the tester's knobs are static; the first field
  snaps), knob changes mid-run, hardware.

## Cut / deviations

- Utilisation is 96.5 %, above the ~92 % target; HD Analog closed on seed 1 with only 2.4 % margin.  If a
  rebuild misses, try more seeds or router2 before restructuring.
- The engine is not the spec's (rho^2 - a^2, 2a y') route.  That route (exact quadratic accumulators, a
  per-line MAC, one 25-bit engine plus a 25-bit third log) was built first and came to ~8,780 LC.  Two
  13-bit per-pole engines lost precision in the far field at small gaps (sigma and tau are tiny differences
  there); the built design runs ONE engine at half rate per pole on Q4 sub-pixel coordinates, which matches
  the exact route's error at Gap 0 (notes.md, m_far_*.png).
- K3 steps through ten band counts (4 5 6 8 10 12 16 20 24 32, ~1.26 x apart) instead of every integer
  4 .. 32: each multiply by N is then one adder of two shifts.
- Camera: a displacement of the sample point along the diagonal (up to ~120 virtual px), not the spec's
  ~2-period phase add.  A plain phase add posterised broad cells into contour blobs; the gradient-scaled
  phase add (the best likeness of the bear) cost ~330 LC and was cut for the fit.  The displacement keeps
  every edge a clean curve, but the subject reads as a warped region with a seam at its outline rather
  than as a recognisable picture.
- X / Y reach 0.25 screen past the edge, not 2 screens: the far field of a dipole is one broad cell, so
  with the pair 2 screens away the picture is nearly blank.
- Like mode's gradient uses log2 rho ~ log2(max + min/2) of |dx|, |dy| with a 4-bit linear mantissa
  (estimated error under ~0.25 octave, not measured): Like-mode AA edges may be up to ~1.7 px wide
  instead of 1 .. 1.4.
- The other pole is interpolated from the neighbouring pixels.  Near a pole at N = 4 (the coarsest cells)
  the rings and rays sit where the float prototype puts them (m_pole_n4.png); at larger N the region
  where the approximation matters is inside the grey eye.
- Drift default 700 is a gentle travel at the low end of the speed curve (the Vellum brief asks ~600-680;
  700 was chosen so the stream is clearly moving at power-up).
- The like-mode sigma frame term (-2Nt) keeps the rays turning with K4 when the poles nearly coincide.
- The phase is a 16-bit cycle fraction; the AA width floors at 2^-16 cycle per px.
