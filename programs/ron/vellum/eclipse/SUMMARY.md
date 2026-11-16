# Eclipse (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/eclipse.vmprog`.

## What it draws

Kandinsky's "Several Circles" (1926), Josef Albers' overlapping translucent planes, a Venn diagram drawn by a
Bauhaus printer: a few large discs, each a translucent film of light on black, orbiting a formation point at their
own slow rates. As they pass, the overlaps slide and change shape: slow eclipses. The default frame is a calm Venn
of three discs (radii 0.42, 0.37 and 0.33 of the screen height, two of them reaching past the frame). Grey: films
add, so one disc is dark grey, two overlapping are light grey and all three are white. Ink: the discs XOR,
black and white, like a print.

    disc i (n = 2 .. 4)   radius R_i = S k_i, k = 1, .88, .78, .70
    centre                c_i = F + rho (1.2 cos phi_i, -0.8 sin phi_i),  phi_i = theta_i + 0.1 turn + i / n
    edge distance         d = (r^2 - R^2) / 2R   (exact integer r^2 - R^2; no square root)
    Flat                  Bold: clamp(1/2 - (d - swell)), Fine: an 8 virtual px outline at d = swell
    Rings (discs 0 .. 2)  Fresnel zone plates: phase d / 24 virtual px + camera, so rings sit 24 px apart at the
                          rim and open toward the centre (each disc reads as a lens); band 1/2 (Bold) or 1/6 (Fine)
    tone                  Grey: films (1/n each; 1/max(2, n - 1) for Fine, 1/2 for Rings), Ink: even-odd XOR

Every edge is anti-aliased over one pixel: the edge distance is scaled per field to 1/16 of the AA pixel
(a 2^-k shift with a 3/4 step), and the ring bands use the ring's own slope (the rim slope, x2 or x4 toward
the centre by r / R class) plus the camera's slope.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the formation point (the point the discs orbit), +-960 virtual px; default centre |
| K2 | Y | the formation point, +-540 virtual px; default centre |
| K3 | Scale | disc size S = 185 .. 1090 virtual px, exponential (default 450); the orbits scale with it |
| K4 | Discs | 2 / 3 / 4 discs, stepped with hysteresis (default 3) |
| K5 | Drift | orbit speed, bipolar: within +-24 counts of centre = still, else 2^-12 .. 2^-5 turn per field; disc rates +1, -3/4, +2/3, -1/2 (two orbit against the others); default 650 (one turn of the lead disc in about 22 s) |
| K6 | Camera | 0 .. 100 % (default 100 %): Flat swells the disc edges into bright areas, Rings bends the rings |
| S7 | Glide | On (default): X, Y, Scale, Camera and Spread ease 1/16 of the gap per field (never under half a count, landing exactly), the Drift speed 1/8. Off: the held value this field. A knob must move 2 counts to register (pot jitter) |
| S8 | Fill | Flat (filled films) / Rings (Fresnel zone plates; at most 3 discs) |
| S9 | Tone | Grey (additive films) / Ink (black and white, overlaps XOR) |
| S10 | Weight | Bold (filled discs, half-duty rings) / Fine (8 virtual px outline circles, 1/6-duty rings) |
| S11 | Source | Video (default) / Osc |
| P12 | Spread | the orbit radius: 0 % = nearly concentric (one bright disc with thin crescents that rotate; Rings: one target of beating zone plates), 50 % = the Venn (default), 100 % = the discs spread to just touching |

## Motion

Each disc has its own orbit angle, a 24-bit per-field accumulator (one turn = 2^24, so it wraps without a jump),
advanced once per field in the vblank zone by the Drift speed times its rate (+1, -3/4, +2/3, -1/2). The orbits are
ellipses 1.2 x 0.8 of rho about the formation point; with different rates and directions the discs drift past each
other, sometimes line up into a full eclipse and separate again. Speed: Hypnos's bipolar decode, +-24 counts still,
then exponential 2^-12 .. 2^-5 turn per field. The top is capped at 2^-5 (a quarter turn per field strobed: a big
disc jumped a quarter orbit per field); at 2^-5 the lead disc circles twice a second. The default 650 turns the
lead disc once in about 22 s. Even at Spread 0 the orbit keeps 5 % of the scale, so the concentric target moves.

## Camera

The luma (or Osc) goes through a per-field table K6 x l^2 (dark areas do little, highlights a lot), a 64-pixel
box (1080; 32 at 720p and SD) and a vertical one-pole, and is never shown as a picture.
- Flat: the field is a swell added to each disc's radius, up to 500 virtual px at white at 100 %: the discs flood
  into the bright parts of the picture and stop at its dark shapes, so the subject reads as film shapes carved by
  the disc edges. 25 % = a gentle swerve of the edges; 50 % = clear bulges; 100 % = the subject outlined.
- Rings: the field is added to the zone-plate phase, up to 3 rings at white: the rings bend around the subject.
- The anti-aliasing divides by the camera's slope class (x1, 2, 4, 8 from the stream's horizontal and vertical
  steps), so swerved edges stay one pixel soft; where the camera is too steep for the ring bands they fade to grey.
- A black input or K6 = 0 gives exactly the clean figure.
- Osc: the shell's drifting 640 x 540 triangle lattice through the same table (0 .. 254, about twice the
  camera's mean level): the discs bulge into big moving diamonds.

## Engine

- Per field: Hardline flute's microcoded engine (37-bit accumulator, 256 x 32 register file in block RAM, one
  adder, serial 18 x 18 multiplier, serial shift), started at every vblank zone from address 0. The program and its
  emulator are in eclipse.py (871 words; the integer model runs the emulator). It does the knob holds and glides,
  the scale (2^(i/256) table), spread, formation point, speed, the camera table gain, and per disc: the orbit angle,
  sin / cos (quarter table, 14-bit interpolation), centre, radius, R^2, the edge and ring scales, the r / R class
  thresholds and the DDA start values. 8.5 k clocks a field at the defaults (9.1 k worst case); every mode's vblank
  is longer (NTSC about 13.5 k after the zone start).
- Per line: one shared adder steps each disc's vertical DDA (r^2 - R^2 at the line's pre-pixel, Q2 of q^2,
  exact; 480i uses a field-line step of 1136/256 virtual lines, 0.2 % short, so the step is exact).
- Per pixel (4 discs in parallel): the exact integer DDA of r^2 - R^2 (q = 1/4 h, so positions move in 1/8 virtual
  px) -> discs 0 .. 2: a 5-way shift and a 6-bit mantissa (the ring distance, smooth with Scale) -> the edge
  (3/4 step and a shift to 1/16 AA px, / camera slope class, - swell) and the zone-plate band; disc 3: one shift
  (with the slope class) and a 3/4 step -> coverage 0 .. 16 -> sum -> tone ROM (films or the Ink triangle) ->
  luma [1 2 1] / 4 -> 13/16 toward black (white 775) -> blanking gate; U = V = 512 on every pixel.
- Source: luma (or the lattice) -> K6 l^2 table (refilled every field from the engine's gain) -> box ->
  vertical one-pole (line store) -> stream + slope.
- Shell: riley / Hardline: timing-ID virtual-1080 steps, vblank zone (no avid for 1024 clocks), generated avid
  (parity-locked), one latency (24) in every mode, SD shows the centre 4:3.

## Status

- v0.1.0 built 2026-11-16 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on seed 1. Not flashed.
- Post-route Fmax (the last "Max frequency" line per clock): HD Analog 92.44 MHz, HD HDMI 93.90, HD Dual 91.63
  (HD needs 74.25); SD Analog 88.82, SD HDMI 93.84, SD Dual 89.88 (SD needs 27).
- 7,086 .. 7,139 / 7,680 LC (93 %), 16 / 32 EBR. Six bitstreams pass iceunpack. No `emulate_transparency`, no
  carry self-adds (dupin: 0). Not tried with router2 (router1 closed with margin).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum eclipse`.
- Model = RTL (GHDL), pixel diff 0 on every render made:
  - 720p60 decimation 1 (918,720 px), the real design: defaults with a black input; Source = Osc; bear.png at
    Camera 100 % (the model fed the tester's own luma); the power-up defaults with Glide On and Drift 650, and the
    same four fields later (model zone counts 2 and 6: the travel and its direction match; 10,602 pixels changed).
  - 720p60 decimation 2: defaults; Rings + Ink; Rings + Fine; X 300, Y 700, Scale 300, 4 discs, Spread 800,
    Ink + Fine; Osc; bear.png at 100 %.
  - NTSC decimation 1: both fields (720 x 243 each), the same picture (the centre 4:3).
  The engine needs 8.5 k clocks a field; the tester's fake 8-line vblank is shorter at 720p / 2 and NTSC, so those
  renders used a sim-only copy whose engine writes the vertical DDAs' line-0 values through shadow registers at the
  next field start (`.vellum_work/rb_eclipse/mksimcopy.py`); every other bit is the real design. Real vblanks are
  longer than the program (NTSC about 13.5 k clocks after the zone start).
- `imodel.py` (work dir) mirrors the RTL section by section and runs the microprogram through eclipse.py's emulator.
- Renders (work dir `.vellum_work/rb_eclipse/`): `cmp_final.png`, `sheet.png` (control sheet), `strip.png` (8 frames
  across Spread, and Rings Ink), `motion.png` (240 fields apart; Drift at both ends one field apart), `camera.png`
  (bear.png at Camera 25 / 50 / 100 %, Flat, Ink and Rings), `zone.png` (the ring study).
- Not sure of on hardware: nothing on this chassis or engine has been flashed yet (riley and flute are unflashed;
  the generated avid / zone / width logic is the heirloom shell's, which is hardware-validated). The picture is sharp
  luma edges in motion; the [1 2 1] + 13/16 conditioning is built in against the HD HDMI bars. Rings + Ink at large
  Scale is the densest frame if bars appear.

## Cut / deviations

- **Discs 2 .. 4, not 2 .. 5.** The fifth disc did not fit (one flat disc costs about 600 LC; the first full
  build packed 7,678 / 7,680 LC). The default 3 and all ringed discs are kept.
- **Rings are Fresnel zone plates, not equal-spaced rings, and only on discs 0 .. 2** (Rings mode draws at most
  three discs; with K4 = 4 the fourth hides in Rings). Equal spacing needs r per pixel: the first RTL had a ROM
  square root per disc (about 890 LC each) and did not fit. The zone plate comes from the edge distance already
  computed: concentric rings 24 px apart at the rim, opening toward the centre, each disc a lens; overlapping zone
  plates make the classic zone-plate moire. At Spread 0 the three plates of different radii beat.
- The disc edge uses d = (r^2 - R^2) / 2R, exact at the edge; the camera swell is therefore slightly nonlinear
  far outside a disc (a 500 px swell lands about 15 % further out on a small disc).
- The edge AA scale is a shift with an optional 3/4 step (edges 0.85 .. 1.15 px wide); the ringed discs carry a
  6-bit mantissa so the ring spacing changes smoothly with Scale.
- K1 / K2 reach +-960 / +-540 virtual px (the formation to the screen edge), half the spec's "about one screen";
  further, the screen emptied at the knob ends.
- Drift top capped at 2^-5 turn per field (see Motion).
- P12 at 0 keeps a 5 % orbit (nearly, not exactly, concentric) so Drift still moves the picture there.
- Fine outlines are 8 virtual px (4 px at SD's AA pixel); Grey Fine and Rings use a brighter film (1/max(2, n - 1),
  1/2) so thin lines read.
- 480i: field-line step 1136 instead of 1138 / 256 (0.2 % shorter picture) for an exact vertical DDA.
- `field_n` is passed through one register (the core ties it high on every input).
