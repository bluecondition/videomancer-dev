# Decade (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/decade.vmprog`.

## What it draws

Keuffel & Esser log-log graph paper, the slide-rule scale and the semilog plot of exponential growth. A sheet
of log paper whose decades converge on an origin just off the lower-left corner: every decade is ruled at
log10(n) (its lines crowding toward the next decade line), bold white decade lines, light-grey 5-lines,
dark-grey fine rulings, about three decades across the screen, each decade 2.4x the size of the one before.
Over it a plotted family of power laws y = c x^k (white, half a decade apart) that bend out of the origin.
Where the rulings crowd closer than about 3 px (into the origin, along an on-screen axis) the paper fades to
flat grey instead of aliasing.

    per axis   s = |x - x0|,  L = P log10(s) - drift        rulings at frac(L) = log10(n)
    curves     phase = 2 (Ly - k Lx) + camera + drift share   (k from P12)

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the origin, horizontally: default 150 virtual px off the left edge; +100 % = the screen centre (the paper mirrors into a log cross, the scales crowding into both axes as grey bars); -100 % = 1,260 px off-screen (about one decade across) |
| K2 | Y | the origin, vertically (default 150 px below the bottom edge, +100 % = the centre) |
| K3 | Scale | decades per screen: P = 0.88 .. 4.4 decades per log10 unit, exponential (default 689 = 2.6: three decades across) |
| K4 | Lines | graduation: Decades / 1-2-5 / 1..9 (default) / Halves; hysteresis with the two end exceptions |
| K5 | Drift | the endless zoom, bipolar, +-24 counts still; 2^-12 .. 2^-5 decade per field (capped below Hypnos's 2^-2: faster strobes the far rulings). Clockwise = the rulings stream into the origin and shrink (a zoom out); counter-clockwise = they stream out of it and grow. Default 670 = 2^-10.1 decade per field (a decade every ~18 s at 60 Hz) |
| K6 | Camera | camera (and Osc) strength 0 .. 100 %, default 100 % |
| S7 | Glide | Off / On (default On): eases X, Y, Scale, Camera, Plot (1/16 of the gap per field; X and Y never slower than 1 px per field, landing exactly) and the Drift speed (1/8); Off = zero easing; a +-1-count hold against pot jitter |
| S8 | Paper | Log-log / Semi-log (x ruled linearly, 512 virtual px per unit: tenths, 5s and units) |
| S9 | Tone | Grey (white decades, grey 5s, dark fine rulings, the curves drawn over in white) / Ink (every ruling white, the curves XOR the paper) |
| S10 | Weight | Bold (decade 6, five 4, fine 2.5, curves 3 virtual px) / Fine (2.5 / 2 / 2, curves 2) |
| S11 | Source | Video / Osc |
| P12 | Plot | the exponent k of the plotted family: 0 % = flat horizontal lines, 50 % = k 1 (rays out of the origin, one at 45 degrees), 100 % = k 3.8 (steep curves crowding the y axis); default 460 (k 0.85) |

## Motion

Drift zooms the paper about the origin forever (at the default the rulings stream into the corner, each
decade line taking ~18 s to shrink to the size of the one before it). One accumulator (Q24 decades) moves both axes' log
coordinates and wraps modulo exactly one decade, which maps the paper onto itself, so it never jumps. The
curves ride the same zoom: their phase takes a second accumulator advanced by 2k x the same step (a power law
with k = 1 is scale-invariant, so at P12 50 % the rays stand still while the paper streams under them). Both
advance once per field at the vblank-zone start (no vsync level or edge involved).

## Camera

The camera is plotted as data. Luma through a per-field gain table (K6, 2.5x for Osc), a horizontal one-pole
(1/32 per pixel, reloaded at every line start) and a vertical one-pole at quarter horizontal resolution (1/64
per line at 1080p, 1/32 at 720p and 1080i, 1/16 at SD; the first line of a field loads it), interpolated back
to pixel rate, adds 2.5 cycles at white (100 %) to the phase of the plotted family: the curves become the
picture's contour lines. 25 % = a gentle swerve of the curves (0.6 cycle), 50 % = the subject's outline runs
through the family, 100 % = a contour drawing of the subject. The paper stays exact. K6 = 0 or a black input =
the clean figure. Osc substitutes the shell's slow triangle lattice (640 x 540 virtual px, drifting) at 2.5x
the camera level: a lattice of nested rounded diamonds drawn by the curves.

## Engine

- Per field (vblank zone, ~3,900 clocks): knob holds and glides on a 16-entry block-RAM register file; one
  serial multiplier (signed x unsigned) in seven slots (the origin, A = P log10 2 through an exp2 table, k,
  2kA, the curves' drift share, the camera gain); the drift; the camera gain table (256 entries); A e - drift by
  octave (16 entries); the log tables T(m), D(m), Tk(m), Dk(m) over a 128-entry mantissa, on the multiplier's
  second accumulator (24 clocks per entry); the y engine's first line.
- Per line: |y - y0| of this line and of the line before go into the x pipeline as two samples ahead of the
  pre-pixel, so the x hardware computes Ly, its per-line gradient and the tone of the horizontal rulings (one
  tone per line).
- Per pixel (stages 2 .. 39): |x - x0| -> leading one -> mantissa -> T/D table + 5-bit interpolation + A e ->
  Lx (Q18 decades) [semi-log: s / 1024] -> 10^frac ROM -> nearest drawn line by K4 -> line position ROM ->
  |dL| / |dL/dpx| (finite difference per real pixel) as a shift and a 3-bit-mantissa multiply -> coverage (17
  levels) -> class tone -> density fade (5 levels).  Curves: phase from 2 Ly, the Tk table and the camera;
  |grad| = max + 3/8 min of the previous-pixel difference and a 2048 x 12 phase store (the line above); the
  same AA.  Compose (Grey: curves over the paper; Ink: XOR triangle), 876/1024 to luma, [1 2 1]/4, 13/16
  toward black (white 776), blanking gate.  U = V = 512 on every pixel.
- Latency 40 in every mode; generated avid (the shell's parity-locked start); hsync / vsync / avid delayed in
  a block-RAM ring; virtual 1080-line coordinates (SD shows the centre 4:3).

## Status

- Built 2026-11-16 (03:12, rebuilt from the final files at 03:57 with identical results) through the shared queue with router1 (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1. Post-route Fmax: HD Analog 74.73 MHz, HD HDMI 74.40, HD Dual 85.06 (Fmin 74.25); SD Analog
  74.88, SD HDMI 76.38, SD Dual 71.01 (Fmin 27). HD HDMI and HD Analog pass with little margin (0.15 and 0.48
  MHz): router1 is deterministic, so a rebuild of these sources reproduces them.
- 6,940 .. 6,969 / 7,680 LC (90.7 %), 25 / 32 EBR. Six bitstreams; no `emulate_transparency`; dupin: no
  duplicate-input cells.
- vmprog: `out/rev_b/ron/vellum/decade.vmprog` (451,758 bytes). Rebuild:
  `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum decade`
- Verification (GHDL image tester against the integer model `.vellum_work/rb_decade/imodel.py`, Glide off,
  Drift still; the capture sits 4 columns right of the model, columns 4 .. 1279 compared):
  - 720p decimation 1, defaults, black input: 0 of 918,720 pixels differ (columns 4 .. 1279).
  - 720p decimation 1, Source Osc, Camera 100 %: 0 of 910,080.
  - 720p decimation 1, bear.png, Camera 100 %: 5.3 % of pixels differ, all on the camera-driven curves
    (sub-pixel contour shifts: the tester converts the pre-swapped image to luma its own way); the paper is exact.
  - NTSC 480i decimation 1 (a 720 x 243 field): 0 of 171,072; it is the centre 4:3 of the 720p picture.
  - Motion, 720p decimation 2, Drift 670: captures 10 frames apart match the model exactly at 3 and 13 drift
    steps (0 of 224,640 each): the rate and the direction (rulings streaming into the corner) are the model's.
  - Glide On at the power-up defaults, 720p decimation 2: 0 of 224,640.
  - Sim note: at decimation 4 the tester's fake 8-line vblank (2,048 zone clocks) is shorter than the ~3,900-clock
    per-field sequence, so dec-4 renders are wrong in sim only (real vblanks are 15,000+ clocks). Use dec 2 or 1.

## Cut / deviations

- The spec mixes two pictures; Decade draws log graduations on a linear screen with a power P (K3): for P > 1
  each decade is ruled at log10(n) and crowds toward the next decade line, decades converge on the origin and
  the drift is a true visual zoom. The plotted power laws are straight in log space and curves on screen, so
  "45 degrees at 50 %" is the k = 1 member of a fan of rays.
- Semi-log paper rules x linearly but keeps plotting the power-law family (power laws curve on semi-log
  paper); the exponential family y = c b^x needed a second per-pixel DDA and was cut for fit. The x unit is
  fixed at 512 virtual px (K3 scales the log axis only).
- The camera does not bend the paper: it is the plotted family's data (contours). A bend of the rulings was
  built and cut for fit; a pixel-space bend with line-weight engraving read as a heavy wobbly grid.
- Box filter replaced by a horizontal one-pole (cheaper; the contours lean a little to the right).
- The AA divide is a shift and a 3-bit mantissa (line widths within +-6 % of exact); fade weights in 5 levels.
- Drift capped at 2^-5 decade per field instead of 2^-2.
- SD: the same picture (the centre 4:3), but a 480i field line is 4.4 virtual lines, so the horizontal rulings
  reach the flat-grey fade sooner (a grey band above the bottom edge at the defaults) and near-horizontal curves
  are a field line thick (wider than at HD).
- Fit history: first RTL 9,912 LC (standalone); shipped 5,631 standalone, 6,965 with the SDK core (notes.md).
