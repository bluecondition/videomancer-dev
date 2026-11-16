# MULTIPOLE (Hardline)

**v0.1.0 built 2026-11-15, not flashed.** All six configs build on router1, seed 1; HD timing closed; 6,982 .. 7,001 LC (91 %), 25 EBR.

## What it draws

The field lines of a magnetic multipole and its conformal checkerboard, full screen, black and white.
Art reference: field-line drawings of magnetic multipoles (the dipole's tangent circles, the
quadrupole's four leaves) and conformal-map checkerboards, where the stripes of Re z^n and Im z^n
cross at right angles everywhere.

    PHI_re = (2/|n|) (r/R0)^n cos(n theta + turn)        PHI_im = the sine partner

- n = -4 .. -1: flowers of 2|n| lobes, closed curves all touching at the pole (n = -1 is the dipole).
  n = 1: plain stripes. n = 2 .. 4: saddles of hyperbola-like curves.
- Lines = the bands of PHI_re. Checker = Re band XOR Im band: a curvilinear checkerboard of
  right-angled cells that shrink into the pole (n < 0) or crowd outwards (n > 1).
- Even space bands PHI itself; Log space bands log2|PHI|, so the lobes nest self-similarly into
  the pole instead of crowding, and the nodal lines become bursts of converging lines.
- Where the lines get finer than about 2 px (the pole for n < 0, the far field for n > 1, the nodal
  seams in Log space) the picture fades to flat grey; it never aliases.
- The default frame is the dipole checkerboard, pole a little left of and below the centre, turned
  -15 degrees.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the pole, +-0.75 screen widths about the centre (0.25 W past the edge at the ends); detent at centre. Default 448 |
| K2 | Y | the pole, +-1 screen height about the centre; detent. Default 558 |
| K3 | Period | Even: the reference radius R0 (the first band edge), 1448 px down to 45 px, exponential, clockwise = denser (as Hypnos). Log: lines per octave of radius, 0.35 to 11, in half-octave steps with hysteresis. Default 435 (R0 = 332 px) |
| K4 | Order | n = -4, -3, -2, -1, 1, 2, 3, 4: eight steps, hysteresis with the two end exceptions. Default -1 |
| K5 | Turn | rotates the figure; the knob spans exactly one lobe period, so 0 and 1023 are the same picture and a sawtooth modulator spins it without a seam. Default 470 (-15 degrees) |
| K6 | Camera | how far the camera bends the linework, 0 .. 100 %. Default 100 % |
| S7 | Glide | On: X, Y, Period, Turn, Camera and Flow ease 1/16 of the gap per field, never slower than half a knob count (about 1 px of pole travel) per field, landing exactly. Off: zero easing, the value latched at the field start is used that field (the accumulators still track). Default On |
| S8 | Weave | Lines / Checker. Default Checker |
| S9 | Space | Even / Log. Default Even |
| S10 | Weight | Bold: equal bands, their edges on the nodal lines. Fine: 1/6-duty lines on the half-integer levels (open black ground); with Checker the two families are OR-ed into a net of field lines and equipotentials. Fine lines never get thinner than about 2 px: they dissolve to grey first. Default Bold |
| S11 | Source | Video / Osc. Default Video |
| P12 | Flow | bipolar, detent at the centre = rest. The Re phase travels +-4 cycles (the lines pump out of or into the pole under the hand) and the band duty swells with it: 0.25 at the bottom (thin white cells), 0.5 at rest, 0.75 at the top (fat white cells). Fine: 0.10 .. 0.23 |

Every knob has a +-1-count hold against pot jitter (latched unconditionally on the first field).

## Camera

The camera is never shown. Its luma (minus black) is scaled by K6 through a per-field gain table,
box-filtered over 32 px, low-passed vertically (1/8 per line on every 8th column) and interpolated
back to pixel rate. The result is added to L = log2|PHI| of both families: a local zoom of the
figure about the pole. Bright areas pull the lines in by up to 1.28 octaves at 100 % (0.32 at 25 %,
0.64 at 50 %); in Log space this is exactly a phase shift of q x bend cycles. A black input or
K6 = 0 gives exactly the clean figure. The nodal lines and the flat far field do not move, so the
skeleton of the figure survives a bold bend.

Osc substitutes the shell's slow triangle lattice (640 x 540 px, drifting diagonally) at the same
point at 1.5 x the camera level (up to 1.9 octaves). It is the only thing that moves by itself.

## Engine

Hypnos v1.1.0 polar chassis (timing-ID steps, generated avid, log-polar engine, one-pixel box-filter
anti-aliasing) with the source field of Kinetic riley. No per-pixel square root, divide or wide
multiplier: nine small table-interpolation products (the largest 11 x 8) and one 5 x 5 for the weave.

- Per field (vblank sequencer, started by the vsync edge after active video): the six knobs go
  through one hold + glide datapath (two rotating rings); the pole position by a serial multiplier;
  log2 R0, the Log step, Turn, the duty, the Flow offsets; the K6 gain table is refilled.
- Per pixel, 47 stages, latency 48 clocks in every mode:
  1-15 hi/lo = max/min(|dx|,|dy|), two integer logs, u = log2(hi/lo) -> log2 r and theta (tables);
  16-25 a = n theta + turn (gated shift-adds), octant fold, log2|cos a| and log2|sin a| (the small
  one through a third integer log so the nodal lines stay exact), L0 = log2(2/|n|) + n (log2 r -
  log2 R0) + bend;
  26-34 two phase chains (Re, Im): Even 2^L by an exp2 mantissa table and one barrel shift, Log
  (2 or 3) L shifted; the sign of the cosine / sine; + Flow;
  27-34 one filter width for both chains from the analytic gradient (Even L0 + log2|n| - log2 r;
  Log log2(q / ln 2) - log2 r - log2 of the smaller of |cos|, |sin|);
  35-40 Hypnos's box filter of the band square wave per chain; 41-42 the weave in 17 coverage
  levels; 43-47 luma 64 .. 940, [1 2 1]/4, 13/16 toward black (white 775), blanking gate.
- U = V = 512 on every pixel. Long-carried values and flags ride two EBR delay lines.
- 25 EBR: 17 table ROMs, 3 delay lines, gain table, box delay, IIR store (the two 32-bit delay
  lines take 2 each).

## Status

- Router: router1 (`NEXTPNR_ROUTER="--router router1"`, as Hypnos), all six configs on seed 1. router2 was
  not tried.
- Post-route Fmax: HD Analog 86.34, HD HDMI 78.05, HD Dual 76.30 MHz (Fmin 74.25); SD Analog 84.75,
  SD HDMI 83.93, SD Dual 87.59 MHz (Fmin 27). HD Dual has 2 MHz of margin.
- 6,982 .. 7,001 / 7,680 LC (91 %), 25 / 32 EBR. Six bitstreams pass iceunpack. No
  `emulate_transparency` in the yosys log (every RAM carries `no_rw_check`); `dupin.py`: no
  duplicate-input cells.
- vmprog: `out/rev_b/ron/hardline/multipole.vmprog` (built 2026-11-16 00:02 from these sources).

Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline multipole`

Verification (GHDL image tester against the integer model `.hardline_work/rb_multipole/imodel.py`,
Glide off): 0 differing pixels in every frame compared (the capture sits 4 columns right of the model: the
generated avid).
- 720p decimation 1 (1280 x 720), default frame, black input: 0 of 918,720.
- 720p decimation 1, Source Osc, K6 100 %: 0. 720p decimation 1, bear.png as the camera, K6 100 %: 0.
- NTSC 480i decimation 1 (a 720 x 243 field): 0 of 173,988; shown 4:3 it is the centre 4:3 of the
  720p frame (mean difference 5.6 / 255 after resizing: the coarser lines only).
- 320 x 180 (720p decimation 4): Log checker, Log + Fine + Lines at n = 3, n = 4 Fine net with Flow
  and Period moved, n = -3 Lines with Turn / Period / Flow moved, n = 2 Log Fine with the pole far
  off-centre, Glide on with knobs off their defaults: 0 each.
- Not simulated: 1080p / 1080i rasters, a glide in motion (the tester's knobs are static; the first
  field snaps), knob changes mid-run.

## Cut / deviations

- X / Y reach 0.25 W / 0.5 H past the frame edge, not 2 screens: the far field of a flower (n < 0)
  is flat, so with the pole 2 screens away the picture is blank. X travel is in virtual pixels, so
  SD shows the centre 4:3 of the HD picture.
- The camera adds to log2|PHI| (a zoom about the pole), not to the phase. A phase add posterises
  the flat far field into blobs that are the camera picture; the log add leaves flat areas flat
  and is a phase add in Log space.
- Flow also swells the band duty. A phase offset of a whole number of cycles returns the same
  picture, so without the swell the slider's ends would look like its centre.
- Log density is stepped in half octaves (mantissa 1 or 1.5), not continuous: two 22 x 9
  multipliers cost about 1,080 LUT. K3 in Log space therefore steps.
- Log space uses one filter width for both families (the narrower one); with Fine + Checker a
  line crossing the other family's nodal seam is slightly over-blurred there.
- The weave (XOR / OR) is computed in 17 coverage levels; Lines keeps all 257.
- The filter width ignores the camera's own gradient: a very hard camera edge at K6 = 100 % is
  anti-aliased a little too sharply (clean at 4x zoom on the test image).
- Phase is a 16-bit cycle fraction: far-field nodal edges of n = -3, -4 beyond about 900 px from
  the pole have fewer than 4 anti-aliasing levels.
- First RTL was 6,142 LUT (about 9,260 LC); see notes.md for the fit history.
