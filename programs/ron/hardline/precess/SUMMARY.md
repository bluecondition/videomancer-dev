# Precess (Hardline)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/hardline/precess.vmprog`.

## What it draws

The op-art "twisted ellipses" figure, Duchamp's Rotoreliefs and the rosette of a precessing orbit: sixteen nested
ellipses about one centre, all the same shape, each turned a step from the one outside it, filled alternately black
and white.  The eye reads a spiral vortex that is not drawn.  As the twist grows the ellipses cross and the bands
break into a lace of crescents; at 90 degrees per ellipse they form a woven lattice, at 180 the picture repeats.

    ellipse k (k = 0 inner .. 15 outer), turned (k - 15) tau (the outermost stays put, so the frame stays full):
    Q_k = e^2 u^2 + v^2 = a x^2 + b x y + c y^2,  a, c = C0 +- C1 cos 2 theta_k,  b = 2 C1 sin 2 theta_k
    inside iff Q_k + camera < T_k = e^2 A_k^2;   Even: A_k = (k + 1) P;   Log: A_k = 16 P 1.28^(k - 15)
    Bands: the parity of the number of ellipses containing the pixel (the triangle of the summed coverage)
    Stack: painter's order, smallest on top, plate k white when k is odd (crossings become stacked plates)
    Fine:  each ellipse a line 4.5 virtual px wide

The default frame: twist 17.7 degrees per ellipse.  The core is the twisted-ellipse spiral; the outer ellipses
cross into a ring of lace crescents.  At power-up it drifts: the twist winds on at about 0.7 degrees per ellipse
per second (the innermost ellipse turns about 10 degrees a second), through lace, the 90-degree lattice and back to
the plain target in about four minutes.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the centre, -100 .. 100 % = +-1,152 virtual px (the centre reaches just past the frame edge); default centred |
| K2 | Y | the centre, +-648 virtual px; default centred |
| K3 | Period | the spacing P = 30 .. 240 virtual px, exponential (default 84.9: the outermost ellipse just fills the frame) |
| K4 | Oval | the aspect e: 0 % = 0.9 (nearly round, the twist subtle), 50 % = 0.6 (default), 100 % = 0.3 (long needles, a dense lace) |
| K5 | Drift | bipolar (Hypnos decode): +-24 counts still, else 2^-16 .. 2^-8 cycle per field, one cycle = 180 degrees of twist per ellipse (the picture repeats exactly); up winds the twist on, down unwinds it; default 660 (+29 %): about 0.7 degrees per ellipse per second |
| K6 | Camera | how hard the luma bends the bands, gain (K6 / 1023)^2; default 100 % |
| S7 | Glide | On (default): X, Y, Period, Oval, Camera and Twist ease 1/16 of the gap per field (never under half a count, landing exactly, behind a +-1-count hold), the Drift speed 1/8. Off: this field's held values, no easing |
| S8 | Spacing | Even (default) / Log: geometric radii (ratio 1.28), small ellipses crowd the centre, a tunnel (bands floored at 2.5 output px) |
| S9 | Fill | Bands (default): the parity, crossings become lace / Stack: painter's order, smallest on top, crossings become stacked plates (a Rotorelief) |
| S10 | Weight | Bold (default): filled bands / Fine: each ellipse a line 4.5 virtual px wide (with Stack, the plates hide the lines behind them) |
| S11 | Source | Video (default) / Osc |
| P12 | Twist | bipolar: centre (+-8-count detent) = the ellipses nested and aligned, a plain oval target; either side = the spiral in one hand or the other; the ends = 40 degrees per ellipse, a dense rosette lace (the climax); default 742 (+45 %) = 17.7 degrees |

## Camera

`program_type = "processing"`; the camera is never shown.  Luma (y >> 2) - 16 goes through a gain table (refilled
per field), a 64-pixel causal box (reloaded at every line start) and a vertical one-pole in a quarter-resolution
line store (about 64 virtual lines: 1/64 per line at 1080p, 1/32 at 720p / 1080i / SD progressive, 1/16 at
480i / 576i; the field's first line loads it), interpolated per pixel, and ADDED TO EVERY ELLIPSE'S Q (the spec's
"luma adds to every Q"): bright areas shrink the ellipses, so the bands bend round the subject.  Inner ellipses
move most (the displacement goes as 1 / A_k), so the subject reads in the core while the outer bands only swerve.
Gain (K6 / 1023)^2: white adds 7,500 Q at 25 % (a gentle swerve of the core), 30,000 at 50 %, 120,000 at 100 %
(bear.png: ears, eyes and muzzle read as bent bands; model render camera.png).  Black input or K6 = 0 is exactly
the clean figure.  Source = Osc puts the shell's drifting 640 x 540 virtual-px triangle lattice through the same
gain table.

The camera enters the DDAs exactly: its second difference V(m) = C(m) - 2 C(m-1) + C(m-2), shared by all sixteen
ellipses, rides the D adders' upper operand bits (above the 17-bit DD), and each line's DDA starts two pixels early
so the camera's first value enters through V.  No per-ellipse camera hardware.

## Engine

- **Per field, in the background.** A microcoded engine after cassini / flute, widened (precess.py PROGRAM: 660
  micro-ops, a 48-bit accumulator, a 512 x 48 register file in block RAM holding the knob holds / glides and two
  banks of per-ellipse slots, one adder split at bit 24 with a registered carry, a serial 20 x 28 multiplier, a
  serial shift; four clocks per micro-op, about 31,700 clocks per run).  Started at every vblank-zone start, it
  computes the NEXT field's seeds into the bank the beam is not using; the line unit stalls it while it owns the
  register file.  At the next zone start, if it halted, the banks swap (with the gain step and the switches it ran
  with).  It does the holds and glides, the Drift speed and accumulator, the twist, the aspect (an interpolated
  2^x table) and per ellipse: cos / sin (an interpolated 1024-entry table), the quantised form alpha, beta, gamma
  (13 fraction bits, in pixel units), the size T (Even: exact (k + 1)^2 multiples; Log: x 1.28^-2 steps, floored),
  the coverage shift t and the Fine half-width H, and the line-0 seeds F0, D0, E0 (two pixels left of the line).
- **Per line** (the line unit, about 140 clocks after each line's last pixel): LOAD gives each ellipse's beam its
  F0, D0 and DD | t | H in three clocks (flip-flop sync reset and enable only: no load multiplexers); STEP does
  F0 += E, E += EE, D0 += G in the register file.
- **Per pixel** (latency 40 in every mode): sixteen 37-bit DDAs F += D, D += DD + V; per ellipse a four-stage log
  shifter v = (F >> 10) >> t keeping a five-bit window; coverage (Bold: a 1/16-px ramp; Fine: clamp(4 (H - |v|)));
  Stack masks from the inside flags (a two-level prefix OR, the top plate's colour); the sum of sixteen; the
  triangle of the sum mod 32 (Bands, Stack + 16 for a black top plate) or min(sum, 16) (Fine); y = 64 + 876 o / 16;
  [1 2 1] / 4; 13/16 toward black (white 777); blanking gate.  U = V = 512 on every pixel.
- No per-pixel multiplier, divider or square root.  Generated avid (the shell's parity-locked start), virtual
  1080-line coordinates (SD shows the centre 4:3, round things round), field_n through one register.
- Chassis: the Kinetic riley shell ([A] timing, zone, width, generated avid; [D] the Osc lattice with dazzle's
  line-0 fix; [F] output conditioning), with the wave engine, palettes and chroma paths removed.

## Status

- **Build v0.1.0** (2026-11-22, shared queue): **router1** (`NEXTPNR_ROUTER="--router router1"`), every config on
  seed 1.  Post-route Fmax as the builder reports it (the last "Max frequency" line; no retries): HD Analog
  85.08 MHz, HD HDMI 81.35, HD Dual 84.68 (HD needs 74.25); SD Analog 85.08, SD HDMI 81.35, SD Dual 83.88.
  7,140 .. 7,169 / 7,680 LC (93 %), 22 / 32 EBR.  Six bitstreams pass iceunpack.  Not tried with router2.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline precess`.
- Standalone yosys: no `emulate_transparency`; dupin.py: 0 carry self-adds, 0 duplicate LUT inputs (router1-safe).
  The standalone pack is about 5,800 LC; the SDK core adds about 1,350.
- Build history: try 1, HD Analog missed all six router1 seeds (best 67.98 MHz): the camera one-pole did three
  adds straight off its block RAM in one clock.  Try 2 (one-pole and box tap pipelined): seed 1 69.3 MHz, seed 2
  78.3 (the engine's 50-bit adder, its operand register placed across the die).  Try 3 (the engine's adder split
  in two 24-bit halves; the engine runs in the background, so the extra clock is free): all six on seed 1.
- **Model = RTL** (GHDL, diffed against `imodel.py`, which runs the engine's own microcode through precess.py's
  emulator; the capture sits 4 columns right of the program's pixel 0, the generated avid's start):
  - 720p decimation 1 (921,600 px): black input, Glide off, Drift still: 0 differing; the power-up defaults
    (Glide On, Drift 660, three fields): 0; Source = Osc: 0; bear.png at Camera 100 %: 0.
  - NTSC decimation 1 (720 x 243 per field): both fields 0 of 174,960, the same picture as the centre 4:3.
  - Motion: two 720p decimation-2 captures at the default Drift (Glide off) 7 fields apart (fields 3 and 10):
    both 0 of 230,400 against the model at the same field counts; 16,984 pixels changed between them, the twist
    winding on (outward rotation of the inner ellipses), as the label says.
  - Rasters rendered: 720p60 (decimation 1 and 2) and NTSC 480i.  1080p was not simulated (build only).
- Renders in `.hardline_work/rb_precess/`: `rtl_default.png` (GHDL, power-up defaults), `cmp_final.png`,
  `sheet.png`, `strip.png`, `camera.png`, `motion.png` (integer model), `notes.md`, the proto renders `p_*.png`.
- Not flashed.  Hardware doubts: the picture is sharp, full-swing luma edges in motion (the [1 2 1] + 13/16
  conditioning is built in against the HD HDMI bars; the lace at Twist 100 % with Fine is the densest frame).
  The background engine with its bank swap and the line unit's flip-flop-reset loading are new on this chassis.

## Cut / deviations

- **Default twist 17.7 degrees per ellipse (P12 742, +45 %), not "a little off centre".**  Below about 12
  degrees nothing crosses, so Stack equals Bands exactly and S9 would be dead at the defaults.  At 17.7 the core
  is still the twisted-ellipse spiral and the outer ellipses cross into a ring of lace, which Stack turns into
  plates.  P12 at the centre is still the plain oval target.
- **The OUTERMOST ellipse stays put and the inner ones turn** (ellipse k by (k - 15) tau): the frame stays filled
  at every twist and the vortex winds at the centre.
- **X / Y reach +-1,152 / +-648 virtual px** (the centre just past the frame edge), not about two screens: further
  out the frame goes black, and the DDAs would need another bit.
- **Drift range 2^-16 .. 2^-8 cycle per field**, not 2^-12 .. 2^-2: the cycle is 180 degrees of twist PER
  ELLIPSE, and the innermost ellipse turns fifteen times that, so the Hypnos range would spin it at about 40
  degrees a second at the slowest setting.  At 2^-8 the figure cycles in about 4 s (the innermost ellipse turns
  10 degrees per field).
- **Camera = luma added to every Q** (as specified), 3x my first gain with about 64 virtual lines of vertical
  smoothing (the spec's mode with weak smoothing shredded the core into blobs; a radius-scaled uniform bend kept
  the rings but lost the subject).  Inner ellipses move most.  Where the camera shrinks an inner ellipse to a
  point, the vanishing spot is a soft grey smudge a few pixels across, and where Osc's steep lattice cancels an
  ellipse's own slope the edge softens: the anti-aliasing width is fixed per ellipse, not per pixel.
- **Osc at the camera's own gain (x1), not 2 - 4x**: at 2x the lattice swamped the figure into one blob; the
  lattice already swings full range, about twice a camera picture's local contrast.  The flip is unmistakable
  (cmp_final.png).
- **Anti-aliased edges 0.7 .. 1.4 px wide on the flat sides** (a power-of-two window per ellipse), up to 1/e of
  that at the tips of long ellipses.
- **Fine lines are 4.5 virtual px wide** (about 1/10 of the default spacing, not 1/6), at least a 1-px ramp at
  480i where a field line is 4.4 virtual px.  Fine + Stack draws outlined plates that hide the lines behind them.
- **Log spacing** is 1.28 per ellipse, floored so no band falls under 2.5 output px.
- Sixteen ellipses kept (the spec allowed twelve if the fit demanded).
- The engine computes each field's figure during the previous field (one extra field of knob latency, 17 ms).
- `field_n` is passed through one register (the core ties it high on every input).
