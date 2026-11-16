# Rosette (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/rosette.vmprog`.

## What it draws

Offset-printing halftone screens and the rosettes their overlap forms (the printer's screen angles
15 / 45 / 75 degrees), Lichtenstein's Ben-Day dots. Three translucent square screens of round white dots on
black, at 45 - s, 45 and 45 + s degrees. In Grey each screen is a film that adds light: one dot is dark grey,
two overlapping are light grey, all three are white. The default frame is the classic dot-centred rosette:
white rosette centres ringed by trefoils of overlapping grey dots, 15 dots of each screen per screen height.
It churns slowly as the screens slide.

    (u, v)_k = R(theta_k) (x - centre) / P + phase_k (+ offset_k)       cells, modulo 1
    d^2      = distance^2 to the nearest lattice point
    Bold     coverage = clamp(1/2 + (r^2 - d^2) / 2r)                   (the signed distance at the edge)
    Fine     coverage = clamp(1/2 + h - |r^2 + h^2 - d^2| / 2r)        (a ring of half width h)
    Grey     luma = 0 / 0.40 / 0.72 / 1 by summed coverage (Two: 0 / 0.55 / 1), linear in between
    Ink      luma = triangle(summed coverage mod 2): XOR, anti-aliased

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | offset of screen 2 (+) and screen 3 (-) against screen 1, +-1/2 pitch across: changes the rosette type (dot-centred to clear-centred and between); at Angle 0 it splits the single grid into triplets |
| K2 | Y | the same, down |
| K3 | Scale | pitch 14 .. 110 virtual px, exponential (default 78 % = 73 px, 15 dots per height) |
| K4 | Dot | dot radius 0.06 .. 0.52 cell, at least 1.25 virtual px: pinpoints, round dots, dots that touch and merge into lace with star-shaped holes |
| K5 | Drift | bipolar speed, +-24 counts still, 2^-12 .. 2^-4 pitch per field, exponential (Hypnos v1.1.0 decode); screens slide along their own axes at +1, -1, +1/2; default 680 (+33 %) |
| K6 | Camera | halftone: the box-filtered luma swells the dot area (default 100 %) |
| S7 | Glide | Off / On (default On): eases X, Y, Scale, Dot, Camera, Angle (1/16 of the gap per field, at least one count, landing exactly, behind a +-1-count hold) and the Drift speed (1/8). Off = this field's knob values |
| S8 | Screens | Three / Two (the two outer screens, 45 -+ s) |
| S9 | Tone | Grey (additive films) / Ink (black and white, overlaps XOR) |
| S10 | Weight | Bold (filled dots) / Fine (rings: width P/6, at least 2 virtual px; dots smaller than the ring go solid) |
| S11 | Source | Video / Osc |
| P12 | ANGLE | the spread s: 0 = all three screens aligned (one clean grid); 2 .. 10 degrees (about 9 .. 25 %) = giant slow moire rings and octagonal cells with crisp dots; 50 % = 30 degrees (15 / 45 / 75, the default rosette); 100 % = 45 degrees (0 / 45 / 90). 30 (p / 512)^2 degrees up to 50 %, then linear |

## Motion

Each screen has a phase accumulator (Q20, modulo exactly one pitch) that advances once per field at the
start of the vblank zone, at +1, -1 and +1/2 times the Drift step, along the screen's own u axis. The
rosettes therefore churn: the dot-centred rosette dissolves through trefoil and clear-centred rosettes and
re-forms, at the default about one relative cycle in 7 s (each screen 1/800 pitch per field). The speed is the
Hypnos decode (the 2^(i/256) table as the mantissa, a serial shift as the octave), capped at 2^-4 pitch per
field rather than 2^-2 because screens 1 and 2 move against each other (twice the step), which strobes past a
quarter pitch. Drift at centre (+-24 counts) is the only still state.

## Camera

The camera is never shown: it sets the dot size (a halftone). Luma -> 64-virtual-px box (48 at 720p),
reloaded at every line start -> vertical one-pole IIR (32 .. 48 virtual lines) in a line store -> camera
level L (0 .. 219 for black .. white) -> a 256-entry table of dot area, refilled every field:

    r^2 = r0^2 + (L / 219) 0.55 gain ((3/4 P)^2 - r0^2)          (dot AREA linear in luma)

At K6 25 % white swells the dot area 14 % of the way to a 3/4-pitch radius (a gentle reading), at 50 % 28 %,
at 100 % 55 % (bright areas merge into lace with star holes, dark areas keep the K4 dots). 0 %, or a black
input, is exactly the clean figure. Every screen reads the same luma. Osc substitutes the shell's drifting
640 x 540 triangle lattice at the same point at full swing (about twice a typical picture's excursion):
big diamonds of swollen dots drift across.

## Engine

- **Per field** (vblank zone, about 1,650 zone clocks): holds and glides (6 channels), the Drift decode and
  the three phases; one serial 16-step multiplier in 45 slots: pitch from a 2^(i/256) ROM; the spread and the
  sine / cosine of 45 - s from an interpolated quarter wave (screen 3 at 45 + s is screen 1 mirrored about the
  diagonal and reuses its products; screen 2 at 45 degrees has cos = sin); six DDA steps; the six line-0 start
  values through two shared accumulators; the K1 / K2 offsets; dot radius, ring half width, the camera's cap
  and gain; a slope radius r_k = r0 moved toward the camera's running mean level; one 7-step serial divider
  for 1 / 2r_k; then the fills: six identical 512-entry squares tables (second differences) and the camera
  table (an accumulator), all pre-scaled by 1 / 2r_k so they come out in 1/16 px of edge distance.
- **Per line**: six line DDAs (Q20, modulo 1 cell) step at each line start but the field's first.
- **Per pixel** (stages 9 .. 29): three pixel offsets -> six 12-bit adds = the lattice coordinates -> fold ->
  squares RAMs -> d^2 = SQ(u) + SQ(v) -> e = T(L) - d^2 -> Fine: h - |e| -> coverage clamp(8 + x), 17 levels
  per edge -> sum over the screens -> tone ROM (Grey PWL / Ink XOR triangle) -> fade (SD only) -> luma
  [1 2 1]/4 -> 13/16 toward black (white 776) -> blanking gate. No pixel-path multiplier. U = V = 512.
- One latency (32) in every mode; generated avid with the shell's parity-locked start; virtual 1080-line
  coordinates (SD shows the centre 4:3); anti-aliasing unit 1 / 1.5 / 1.5 / 2 / 3 virtual px at 1080p /
  1080i / 720p / 480p-576p / 480i-576i. `field_n` passes through one register.
- Chassis: the Kinetic riley shell via Hardline tweed ([A] timing, [B] controls, [D] source field, [F] luma
  conditioning), with dazzle's Osc line-0 fix; no wave engine, palettes or chroma paths.

## Status

- Built 2026-11-16 through the shared queue with **router1**, every config on **seed 1**. Not flashed.
- Post-route Fmax (last report): HD Analog 85.49 MHz, HD HDMI 76.17, HD Dual 78.60 (HD needs 74.25);
  SD Analog 77.02, SD HDMI 79.87, SD Dual 78.90. HD bitstreams pass iceunpack.
- 6,731 .. 6,754 / 7,680 LC (88 %), 20 / 32 EBR (12 squares copies, camera table, 2 sine / exp ROM, tone ROM,
  box, 3 line store). No `emulate_transparency`, no carry self-adds (dupin: 0).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum rosette`. Router2 was not tried.
- Model = RTL (GHDL), pixel diff 0 on every render compared:
  - 720p60 decimation 1 (921,600 pixels): defaults with a black input; bear.png at K6 100 %;
  - 720p60 / 2: defaults; bear.png at K6 100 %; Source = Osc (the camera and Osc renders with the model fed
    the RTL's running-mean camera level, which the model cannot know: 104 at 720p after 2 warm-up frames);
  - NTSC decimation 1, both fields (720 x 243);
  - motion: default Drift captured 4 fields (Glide Off) and 16 fields (Glide On, power-up defaults) after
    start: both 0; the model one field either side differs on 11 % of pixels, reversed Drift on 21 %.
- Renders in `.vellum_work/rb_rosette/`: `cmp_final.png`, `sheet.png`, `strip.png`, `motion.png`,
  `camera.png` (model), `proto_*.png` (float prototype).

## Cut / deviations

- **Pitch 14 .. 110 virtual px, default 73** (spec: largest about 20 dots per height = 54 px). Three screens
  triple the dot density; at 54 px the default read busy. The largest is now 10 per height.
- **Dot tops out at 0.52 cell**, not "holes" at 0.62: three nearly full films washed Grey to white there.
  At 0.52 the dots touch and merge into lace with dark star holes; the camera can push to 3/4 pitch.
- **Fine rings use one edge slope per field** (1/2r at the ring's centre radius): both ring boundaries are
  exact, but the inner edge is about r / (r - h) = 1.5 px soft and the outer r / (r + h) = 0.75 px at the
  default Dot. An exact two-slope ring cost about 1,100 LC.
- **The camera's anti-aliasing slope is per field**: r_k follows the camera's running mean level (an
  exponential mean over about a million pixels: half a frame at 1080p, a frame at 720p, 6 fields at SD), so a
  black input is exact and with a picture the dot edges are about r_k / r(L) px wide (0.6 .. 1.5 px). Fine
  rings get thinner on swollen dots for the same reason.
- **P12 100 % = 0 / 45 / 90 as specified**: 0 and 90 degrees are the same square lattice, so screens 1 and 3
  coincide (K1 / K2 split them) under the 45-degree screen: a calm square grid of pairs. The wildest frames
  are at 9 .. 25 % (giant moire rings and octagons).
- **Drift top 2^-4, not 2^-2** (see Motion). The phases advance at the vblank-zone start (the shell's "no avid
  for 1,024 clocks"), once per field like a guarded vsync edge and immune to serration.
- Fade to grey has three steps (none / half / flat) and only SD reaches it: below Scale 12 % at 480i / 576i
  the dots fade half way to grey. 720p and 1080p hold every Scale.
- SD: the same picture (the centre 4:3, same dot count top to bottom); a 480i field line is 4.4 virtual px,
  so edges are about 1.5 px wide across and 0.7 field lines down.
- Not verified on hardware: anything. Nothing on this chassis has been flashed. HD HDMI closed with 2.6 %
  margin. The picture is full-swing luma edges in motion; the [1 2 1] + 13/16 conditioning is built in, but if
  HD HDMI bars show, Ink + Fine at a small Scale is the first place to look.
