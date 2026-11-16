# Fresnel (Hardline)

**v0.2.0 built 2026-11-16 (P12 Speed: self-motion), not flashed.** Flash `out/rev_b/ron/hardline/fresnel.vmprog`.

## What it draws

The Fresnel zone plate: rings whose radii go as the square root of the ring number, so they tighten with
distance. And the moire of two zone plates: two displaced plates beat into straight, evenly spaced stripes
perpendicular to the line joining their centres. Black and white only; U = V = 512 on every pixel.

The default frame is two plates XORed, their centres 112 px right / 28 px up of the screen centre and at the
mirror point: a checkered lens in the middle, near-vertical beat stripes filled with fine arcs out to the
edges. At power-up it moves: the rings stream (plate a outward, plate b inward) and the beat stripes march
sideways, about one stripe every 0.8 s.

    PHI_p(x, y) = a dx^2 + b dx dy + c dy^2 +- U + camera + off_p              (dx, dy) = pixel - centre_p
    a, c = k0 ((1 - fm) +- fm cos 2t),  b = k0 2 fm sin 2t                     k0 = 1 / (2 rz^2)
    ink_p = |frac(PHI_p) - 1/2| < duty / 2                                     Pair: ink_a xor ink_b

Plate a sits at (X, Y), plate b at (-X, -Y). off_a = 3/8, off_b = 5/8 cycle: no plate centre sits on a zone
edge, and coincident plates (X = Y = 0) give a double-frequency plate, not a black screen.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | plate centre, 8 virtual px per count, +-4096 (about 2 screens off-frame). Default 526 (+112 px) |
| K2 | Y | plate centre, 4 virtual px per count, +-2048. Default 505 (-28 px) |
| K3 | Period | zone radius rz, 48 .. 400 virtual px, exponential. Default 600 (166 px) |
| K4 | Form | round (a 9:8 ellipse, default) .. ellipse .. parallel chirp bars (about 44 %) .. saddle of hyperbolas (100 %) |
| K5 | Turn | rotates the form's axes, a full half-turn over the knob (0 and 100 % are the same picture) |
| K6 | Camera | how far the video's luma pushes the ring phase, 0 .. 3.4 cycles at full white. Default 100 % |
| S7 | Glide | Off: the value latched at the field start is used that field. On (default): X, Y, Period, Form, Turn and Camera ease 1/16 of the gap per field and Speed 1/8, never slower than 1/4 knob count per field, landing exactly. Glide eases the Speed, never the phase |
| S8 | Pair | Single / Pair (default): one plate, or two XORed |
| S9 | Ghosts | Clean (default): anti-aliased. Raw: binary, sampled on a coarse lattice, so the plate aliases into a grid of ghost plates |
| S10 | Weight | Bold (default): equal zones. Fine: thin rings, 1/6 duty, never under 2 pixels |
| S11 | Source | Video (default) / Osc |
| P12 | Speed | bipolar, centre = still (+-24 counts), exponential: the ring phase U runs on by 2^-12 .. 2^-2 cycle per field (about a minute per ring just off centre, a quarter ring per field at the ends), wrapping modulo one ring. Up = rings stream outward, down = inward. Default 850 (+66 %): one ring every 0.8 s in Single |

Speed is the Hypnos v1.1.0 shape. In Pair the two plates travel OPPOSITE ways (plate a, at X / Y, outward;
plate b, at the mirror point, inward): rings pour out of one centre and into the other, and the beat stripes,
which move by the difference of the two phases, march sideways. Travelling together was tried in the float
prototype (`motion_opts.png`): the beat stripes then stand still, and in the default frame they are most of
the picture. Pair halves each plate's step, so the beat marches at the decoded speed and never passes half a
stripe per field (which would strobe). Glide On eases the Speed knob (1/8 of the gap per field, as Hypnos),
never the phase; nothing but a centred Speed stops the motion.

Period and Weight read as on the rest of the set. In Raw, Period is a pure zoom of blocks and ghosts together.

## Camera

`program_type = "processing"`. The camera is never shown. Its luma goes through a 64-pixel box and a
vertical 1/16 IIR (a quarter-resolution line store, 11 bits) and is added to PHI of both plates with the
same sign, so the rings swerve around the subject. Black input = zero = the clean figure.

Gain: phase = luma x K6, 3.4 cycles at full white at 100 %, 1.7 at 50 %, 0.85 at 25 %. On the test image
(bear.png, model render `camera.png`): 25 % is a gentle swerve; at 100 % in Single the bear reads clearly as
bent rings with clean curves; at 100 % in Pair the head is a disturbed region in the moire but the face is
hard to read; in Raw the camera scrambles the mosaic.

Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice at the same point, at twice the
camera's level (6.8 cycles peak). It drifts by itself, on top of the Speed motion.

The anti-aliasing counts the camera's own gradient (horizontal by finite difference, vertical from the IIR's
increment), so camera-drawn edges are filtered too.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the glide (7 channels, +-1 count hold, 5 clocks
  each), then a microcoded accumulator machine: a 128 x 48 register file in block RAM, A / B / ACC, one
  serial 18-step multiplier, 467 words of microcode (`fresnel.py`). It first steps the Speed phase U (kept in
  the register file from field to field; the zone comes once per field, so it is immune to vsync
  serration), then turns the knobs into both plates' seeds: zone scale through a 2^(i/256) ROM, Form and Turn through a quarter-wave sine ROM, the Raw lattice
  pitch and how many pitches reach the picture's corner, then per plate a, b, c and the DDA start values.
  About 2,000 clocks, up to 3,250 in Raw (counted by the model's instruction-level interpreter).
- **Per line**: the same machine steps the line DDAs in the register file at each line start
  (S0 += E; E += EE; Dx0 += GXY; Gy0 += EE; 56 clocks; in Raw only when the line enters a new lattice row,
  17 or 70 clocks) and after the line's last pixel loads the next line's first values into the beam (12 clocks).
- **Per pixel** (29 stages, latency 32 in every mode): per plate, S += Dx; Dx += DDx (the phase and its
  horizontal gradient) and Gy += GXY (the vertical gradient); in Raw they step only when the pixel crosses
  into the next lattice cell. Then three coverage units (plate a, plate b, the beat PHI_b - PHI_a): |grad|
  as max + 3/8 min, rounded up to a power of two so the box filter's divide is a shift (edges 1 .. 2 pixels);
  past 1/4 cycle per pixel the band fades to grey by 1/2 cycle per pixel. Blend a + b - 2ab, plus the beat
  where both plates have faded. Luma 64 .. 940, [1 2 1] / 4, 13/16 toward black (white = 775), blanking gate.
- No per-pixel square root or divider. Per-pixel multipliers: camera 11 x 8 and 8 x 9, blend 5 x 5, 4 x 4,
  9 x 5, fade 6 x 5 in each unit.
- Virtual 1080-line coordinates from the timing ID; generated avid (the shell's parity-locked start).

## Status

- **Build v0.2.0** (2026-11-16 00:51, shared queue): **router1**, every config on seed 1. Post-route Fmax
  as the builder reports it: HD Analog 87.27 MHz, HD HDMI 84.53, HD Dual 88.40 (HD needs 74.25); SD Analog
  87.48, SD HDMI 87.05, SD Dual 86.84. 6,315 .. 6,346 / 7,680 LC (83 %), 16 / 32 EBR. Six bitstreams pass
  iceunpack.
- v0.1.0 (Pump on P12) built 2026-11-16 00:23, router1, seed 1 everywhere: HD 87.27 / 84.13 / 87.05 MHz,
  6,281 .. 6,308 LC. Its sources and vmprog are kept in `.hardline_work/rb_fresnel/v010/`.
- **router2 does not close this program**: HD Analog missed on all six seeds of the first build (the
  builder's "Completed (seed 6)" was 67.51 MHz) and on seed 1 of two later netlists (69.6, 74.06 MHz).
  Three long paths were shortened on the way (glide, coverage clamp, the engine's subtract control).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh hardline fresnel`.
- No `emulate_transparency` in a standalone yosys run of the same RAMs; `dupin.py`: 0 carry self-adds.
- **Model = RTL** (GHDL), black input: see the list at the end of this section.
- Renders in `.hardline_work/rb_fresnel/`: `cmp_final.png` (Speed at centre; GHDL), `motion.png` (GHDL),
  `sheet.png`, `camera.png` (integer model). `sheet.png` and `strip.png` are v0.1.0's: their P12 panels show
  the old Pump; every other panel is the v0.2.0 picture with Speed at centre.

Model = RTL, pixel for pixel (GHDL captures diffed against `imodel.py`, which runs the engine's own
microcode through an instruction-level interpreter):
- v0.2.0: 720p decimation 1, Speed at centre: 0 of 921,600, and identical to the v0.1.0 default capture.
  720p / 2, Single, Speed +86 % and -86 %, two frames each 3 fields apart: all four 0 of 230,400, and the
  model needed exactly 3 more field steps for frame B (the phase steps once per field). Glide On at the
  power-up defaults (Speed 850): 0 of 230,400 at field 4, already moving.
- v0.1.0 (same pixel path): 720p decimation 1 black input 0 of 921,600; NTSC decimation 1 0 of 174,960
  (both fields identical, the same picture as the centre 4:3 of 720p); 720p decimation 1 with bear.png at
  K6 100 % 0 of 921,600; at 720p / 2: other knob sets (Fine, saddle-ish Form, Turn, off-centre X / Y), two
  Raw settings, the camera in Single + Fine at K6 50 % and in Raw, Osc (lattice phase matched) and Glide On
  part-way through an ease, all 0.
- The GHDL capture's first 4 columns are blank (the generated avid lands 4 clocks after the delayed source
  avid); the model reproduces that, so no alignment is needed.

## Cut / deviations

- **Period bottoms at 48 px, not 24.** At 24 the whole frame was flat grey with a dot.
- **Raw is a coarse sampling lattice (pitch rz / 10), not native-pixel aliasing.** At the default period
  nothing aliases at 1080p, so the switch would have been dead at the default state, and native aliasing
  gives a different picture in every video mode. Ghost plates sit 5 rz apart at every Period.
- **v0.2.0: P12 Pump became Speed** (Ron). Gone with Pump: the -+25 % split of the two plates' scales, which
  made a phantom third plate slide in at the slider's ends.
- **Camera: 3.4 cycles, same sign on both plates, 64-pixel box.** The spec's +-3 cycles (6 peak to peak)
  shredded the picture in the float prototype; opposite signs on the two plates were worse.
- **Clean far field: beat stripes, not flat grey, where both plates are unresolved** (Pair). Single still
  fades to flat grey.
- **Anti-aliased edges are 1 .. 2 pixels wide**, not exactly 1: the filter width is rounded up to a power
  of two (the first RTL divided exactly and came to about 137 % of the chip).
- `field_n` is passed through one register (the core ties it high on every input).
- **SD**: the same picture (the centre 4:3), but 480i has 243 lines per field, 4.4 virtual px each. Zones
  thinner than that fade to grey vertically, so at low Period the plate is resolved only in a band around
  its centre. In Raw the lattice rows are 1 .. 3 field lines tall and uneven below about Period 40 %.
- **720p**: pixels are 1.5 virtual px, so the grey fade starts 1.5 times closer to the centre than at 1080p.
- **Hardware doubts**: Raw (binary, no anti-aliasing) now moves by itself at the default Speed, so its sharp
  block edges are in motion: the exact picture class that triggers the HD HDMI bars (the [1 2 1] and luma
  scaling are in, in both modes). Speed near the ends (a quarter ring per field) may strobe on fine outer
  rings in Single; Pair is held to half that.
