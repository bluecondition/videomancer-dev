# Limacon (Vellum)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/vellum/limacon.vmprog`.

## What it draws

The cardioid drawn as an envelope of circles, the classic construction: every circle is centred on a base circle and
passes through one fixed point on it, and their outer edges crowd into a heart-shaped curve with its cusp at that
point. Moving the point gives the whole family, the limacons of Pascal (inside the base circle: a rosette of nearly
equal discs; outside: the limacon with its inner loop), and the Nephroid form is Huygens' nephroid drawn the same
way (circles centred on the base circle, tangent to a diameter).

The default frame is 24 translucent discs on a base circle about half the screen high (radius 240 virtual px), the
pole straight up, so the figure is the full heart, cusp at the top, filling the screen height. Grey counts the discs
over each pixel as flat grey steps cycling black, dark, light, white every four discs: a shaded, shell-like solid of
curved quads. Ink takes the parity: a black-and-white lace of crescents. The discs roll round the base circle, so
the lace circulates inside a fixed outline, and the pole orbits the other way at a quarter of the rate, so the
outline turns slowly.

    circle k   centre C_k = O + R e(t_k), t_k = alpha + k / N turn
    Cardioid   radius r_k = |C_k - P|, P = O + p R e(psi) (every circle passes exactly through the pole)
    Nephroid   radius r_k = kappa R |sin(t_k - psi)| (kappa = 1: tangent to the diameter, the nephroid)
    F_k        = |X - C_k|^2 - r_k^2 + camera; coverage clamp(1/2 - F_k / (2 r_k px)) over one pixel
    tone       Grey: sum of coverages, count mod 4 as black / dark / light / white (anti-aliased between steps);
               Ink: the triangle of the sum (anti-aliased XOR); Fine: each circle a 2-pixel ring
    crowd      where 5 or more circle edges fall within about 2 px of a pixel (the pole, the cusps, the
               envelope's crowding) the tone fades to flat mid-grey: never sparkle

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the figure centre, +-960 virtual px (default centre) |
| K2 | Y | the figure centre, +-540 virtual px (default centre) |
| K3 | Scale | base radius 99 .. 581 virtual px, exponential (240 at 50 %); the figure keeps its size across P12 |
| K4 | Count | 8 / 12 / 16 / 24 discs (stepped, hysteresis; a knob resting at either end reaches the end step); default 24 |
| K5 | Drift | bipolar: +-24 counts still, else 2^-9 .. 2^-2 disc steps per field; the discs roll, the pole orbits back at a quarter of the rate; default 650 (about 0.44 disc steps per second, the outline turns once in about 3.6 min) |
| K6 | Camera | box-filtered luma squared added to every disc's F: the discs shrink away from the light (default 100 %) |
| S7 | Glide | On (default): X, Y, Scale, Camera and Pole ease 1/16 of the gap per field (at least half a count, landing exactly), the Drift speed 1/8; a knob has to move 2 counts to register. Off: no easing |
| S8 | Form | Cardioid (discs through the pole) / Nephroid (discs tangent to a diameter) |
| S9 | Tone | Grey (stepped grey count) / Ink (black and white parity lace) |
| S10 | Weight | Bold (discs) / Fine (each circle a 2-pixel ring: the textbook drawing; the envelope shows as a bright caustic) |
| S11 | Source | Video (default) / Osc (3/4 of the shell's drifting triangle lattice through the same camera table) |
| P12 | Pole | Cardioid: the pole's distance from the base centre, 0.15 R (0 %, a 24-petal rosette) .. R at 60 % (the cardioid, cusp and all; an 8-count detent; power-up) .. 2.2 R (100 %, the limacon's inner loop). Nephroid: the radius rule kappa 0.5 (two crescents of beads) .. 1.0 at 60 % (the nephroid) .. 1.8 (a central lens) |

## Motion

Two per-field accumulators in the engine's register file, one turn = 2^24 (both wrap exactly, so the drift never
jumps): alpha (the discs' angle) and psi (the pole's angle). Speed: Hypnos's bipolar decode, +-24 counts still,
then exponential 2^(ls/64 - 9) disc steps per field, ls = 0.906 (|v| - 24): 2^-9 just past the dead zone to 2^-2.1
(a quarter step) at the ends. The top is capped there: a half step per field reads backwards (wagon wheel). alpha
advances by speed / N turn, psi by minus a quarter of that. Drift at centre is the only still state. The displayed
field uses the engine run started at the previous zone start (one field of lag, deterministic in sim and hardware).

## Camera

Luma -> per-field table K6-gain x l^2 (0 .. 8191; the gain scales with Scale so the bend is a fixed fraction of the
figure) -> 64-pixel box (1080; 32 at 720p and SD) -> vertical one-pole in a half-resolution line store -> added to
every disc's F (in 1/4 virtual px^2). White at 100 % is 4480 R0, which moves the edge of a radius-R0 disc about
560 px: bright areas empty out, dark ones keep the lace, so the subject reads as a silhouette carved through the
discs' own edges (bear.png: the ears, the eyes and the snout). 25 % is a gentle swerve of the edges, 50 % clear
bends. A black input or K6 = 0 is exactly the clean figure (the camera enters as an exact second difference on the
pixel step, zero outside the active line). Osc: 3/4 of the drifting 640 x 540 triangle lattice through the same
table (its peak 3/4 of white, about twice a typical picture's mean): the heart buckles into a drifting quatrefoil.

## Engine

- Per field: Hardline flute's microcoded engine (37-bit accumulator, 256 x 32 register file in block RAM, one adder,
  serial 18 x 18 multiplier, serial shift), started at every vblank zone start from address 0; the program and its
  emulator are in limacon.py (1,043 words). It does the knob holds and glides, Scale (2^(i/256) table), the Pole
  functions, the drift, the camera gain, and per circle: sin / cos (quarter table, 14-bit interpolation), the
  quantized centre, the slope integers lam / mu, the line-0 start value, the two line-end constants, the start slope
  and the edge-scale class, written into the inactive half of a 256 x 32 bank RAM (about 56 k clocks; the halves
  swap at the next zone start when the run has halted).
- Exact integer geometry: F in 1/4 virtual px^2, F_k(m, j) = xs^2 (m - mV)^2 + floor(4 hy^2 (j - jV)^2)
  - lam_k (m - mV) - mu_k (j - jV) + G_k, V a pixel-grid point beside the pole; centres quantized to 1/8 px
  (1080; 1/4 at 720p, 1/16 SD) so lam, mu are integers and every Cardioid circle passes exactly through the pole.
- Per field start: F = D = 0, D_k += F00_k, F += D, D_k = D0_k (one 24-slot pass each through one shared bus).
- Per line end (57 clocks, fits the tester's 64-clock blanking): D_k += KC'_k + Ysh, then D_k += P2_k - Ysh with
  F += D on its first load; Ysh is the floor difference of the shared y quadratic (one Q12 DDA).
- Per pixel, 24 circles in parallel: F += D, D += bus (2 xs^2 + the camera's second difference) -> a 6-bit window
  F >> (5 + e), e = 0 .. 3 by the circle's radius class (x 3/4 on half octaves: every edge 0.94 .. 1.4 px soft)
  -> coverage 0 .. 16 (Bold / Fine) -> sum + crowd count -> tone ROM -> luma [1 2 1] / 4 -> 13/16 toward black
  (white 775) -> blanking gate. U = V = 512 on every pixel.
- Shell: riley / eclipse (timing-ID virtual-1080 steps, vblank zone, generated avid, one latency (24) in every
  mode; SD shows the centre 4:3). 480i uses 4.5 virtual lines per field line (1.2 % tall) so the vertical
  quantum is dyadic.

## Status

- v0.1.0 built 2026-11-22 through the shared queue with the default router (router2), every config on seed 1.
  Not flashed.
- Post-route Fmax (the builder's last "Max frequency" report per config): HD Analog 83.08 MHz, HD HDMI 88.15,
  HD Dual 81.49 (HD needs 74.25); SD Analog 79.06, SD HDMI 78.20, SD Dual 83.08 (SD needs 27).
- 6,863 .. 6,880 / 7,680 LC (89 %), 23 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin: 0).
- Rebuild: `./build_set.sh vellum limacon`.
- Model = RTL (GHDL), pixel diff 0 on every render made (the GHDL frame is the model's shifted 4 columns right):
  - 720p60 decimation 1 (918,720 px): the defaults with a black input (Glide Off, Drift still); Source = Osc;
    bear.png at Camera 100 % (the model fed the tester's own luma); the power-up defaults with Glide On and
    Drift 650 (= rtl_default.png, model zone count 1).
  - 720p60 decimation 2: Count at 0 (8 discs) and at 1023 (24 discs) and at 512 (16), the stepped knob's two ends
    reaching their end steps from power-up; the power-up defaults 4 fields apart (zone counts 1 and 5: the
    travel and its direction match; 43,932 pixels changed); Nephroid at the defaults; Nephroid + Ink + Fine with
    Count 16, Pole 88 %, Scale 700, X 300, Y 700; Cardioid Ink with Count 12, Pole 0 %, Scale 200, X 800; Fine with
    Pole 100 %, Scale 1000, Y 200.
  - NTSC decimation 1: both fields (720 x 243 each), the same picture (the centre 4:3).
- `imodel.py` (work dir) mirrors the RTL section by section and runs the microprogram through limacon.py's
  emulator. Renders in `.vellum_work/rb_limacon/`: `cmp_final.png`, `rtl_default.png`, `sheet.png`, `strip.png`,
  `motion.png`, `motion_rtl.png`, `camera.png`.
- Not sure of on hardware: nothing on this chassis or engine family has been flashed yet (eclipse / riley are
  unflashed; the generated avid / zone / width logic is the heirloom shell's, hardware-validated). The per-line
  bank sequencer is new: it needs 57 clocks after the last pixel step (HD has 280, SD 138). The picture is sharp
  luma edges in motion; the [1 2 1] + 13/16 conditioning is built in against the HD HDMI bars; Ink at large Scale
  is the densest moving frame if bars appear.

## Cut / deviations

- Nephroid P12: the spec's "tilts and offsets the tangent line" became the radius rule kappa (0.5 .. 1.0 .. 1.8).
  A tilt only rotates the figure (the drift already turns it); an offset gave a lopsided fan with stray beads.
- The edge scale has 4 window positions (e = 0 .. 3) plus the 3/4 step, not a full barrel: circles with r x the
  pixel size under about 45 virtual px (the one or two tiny discs at the pole) get softer edges, and the largest
  discs at Scale > 75 % slightly sharper ones (0.6 px). An 8-position barrel cost 600 LC.
- Circles of radius under 4 px are skipped (the disc passing through the pole as it rolls).
- Drift range 2^-9 .. 2^-2 disc steps per field (2^-12 .. 2^-5 of an 8-step cycle), top capped (see Motion).
- 480i: 4.5 virtual lines per field line (1.2 % tall).
- Fine rings are 2 AA pixels wide (2 virtual px at 1080, 3 at 720p, 4 at SD).
- `field_n` is passed through one register (the core ties it high on every input).
