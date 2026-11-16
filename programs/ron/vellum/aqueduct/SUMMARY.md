# Aqueduct (Vellum)

**v0.1.0 built 2026-11-16, not flashed.** Flash `out/rev_b/ron/vellum/aqueduct.vmprog`.

## What it draws

Arcades at two depths in flat greys: de Chirico's arcades, the Pont du Gard, painted stage flats, Bauhaus
silhouettes. A black near arcade with white archivolts stands in front of a light-grey far arcade, against a
pale sky. Each tier is a stage flat, solid below its own base, so through the near arches the floors step back
in grey. The two tiers are scaled copies in perspective about a horizon above the screen: the far tier is
smaller and higher. Drift slides them sideways with parallax (the near one faster); the slider pulls them apart
in depth. Luma only (U = V = 512 on every pixel).

    tier k (0 far, 1 near): scale 1 (near) and 1/g (far), g = 3.2^(P12 / 1023); pitch p = scale x P
    arch centres  x = x0 + (theta + j) p,   x0 = 960 + Xpan (1 + scale) / 2       (theta = the drift phase)
    vertical      horizon yh = y0 - 1.234 P; top = yh + 1.078 p; base = top + 1.898 p; spring = top + 0.18 p + apex
    opening       d = (Rc^2 - dd^2 - X^2) / (2 Rc)     X = |dx| + e - swell;  dd from the spring (shape-clamped)
    wall          min(y - top, -min(d, base - y))
    line          Grey: the archivolt around the head (a band of d outside the opening, cut at the spring);
                  Ink: the outline of the opening; Lattice: the outline of the wall
    coverage      every edge over one anti-alias width (17 steps): 1 virtual px at 1080p, 2 at 720p / 1080i /
                  480p / 576p, 4 at 480i / 576i (about one row, and every unit conversion a shift)

The default frame (Scale 50 %, Depth 50 %): the near arcade has 4.2 arches across the screen, its cornice at
mid-height, white archivolts; the far arcade (g = 1.79, 7.5 arches) shows above it and through it.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pans the picture +-960 virtual px: the near tier by the full amount, the far tier by (1 + 1/g) / 2 of it (some parallax; full parallax would only repeat Drift) |
| K2 | Y | the whole structure up or down, +-544 virtual lines (the horizon height) |
| K3 | Scale | the near pitch P, 160 .. 1280 virtual px, exponential (default 453); zooms about the screen centre |
| K4 | Arch | Round / Pointed (arcs of 13/8 R, centres 5/8 R across) / Horseshoe (a 5/4 R circle narrowing 3/4 R below its centre) / Stadium (vertical windows, 2 R straight) / Porthole / Gable (45 degrees). Stepped with hysteresis, both ends reachable |
| K5 | Drift | bipolar speed: the Hypnos law, +-24 counts still, 2^-12 .. 2^-3 cycle of each tier's own pitch per field (default 660 = 308 / 2^18: about 14 s per near pitch at 60 Hz). The top is capped one octave below the brief's 2^-2: at a quarter pitch per field the arcades strobe |
| K6 | Camera | the swell: up to 0.22 P at white for the near tier (proportional for the far one), at most the pier's half-width less 1.5 px. Default 100 % |
| S7 | Glide | On (default): X, Y, Scale, Camera, Depth ease 1/16 of the gap per field, never slower than one count, landing exactly; the Drift speed eases 1/8. Off: no easing at all. A +-1-count hold on every knob (the ends always get through) |
| S8 | Build | Solid / Lattice: the tiers as outlines only, additive lines (far 1/2, near 7/8 of white) on black, so the two arcades show through each other |
| S9 | Tone | Grey / Ink: sky white, the far tier black, the near tier white with a black outline round every opening; Lattice + Ink = the lines XOR |
| S10 | Weight | Bold (piers 0.36 p, archivolts 0.035 p) / Fine (piers 0.16 p = about 1/6 duty, lines 0.012 p, at least 2 rows) |
| S11 | Source | Video / Osc |
| P12 | Depth | the headline. 0 % = the two tiers coincide: one black arcade on a pale sky. They separate as it rises; 50 % = g 1.79 (default); 100 % = g 3.2: the far tier a band of small arches near the horizon, the near openings framing its floor |

## Motion

Both tiers share one drift phase theta (Q18, 18 bits), counted in each tier's own pitch, so the screen speeds are
in proportion to the tier sizes (parallax: the near arcade moves g times faster than the far one). theta advances
once per field at the vblank zone start (no avid for 512 clocks: once per field, robust to serrated vsync) by the
speed the field engine computed the field before, and wraps modulo one pitch, so the motion never jumps. Default
660: rightward, 0.00117 cycle per field. Drift 0 / 1023: -0.129 / +0.125 cycle per field.

## Camera

The camera never appears as a photograph. Its luma goes through a per-field gain table (K6), a 128-px box (64 px
at 720p and SD), a vertical one-pole (time constant about 128 virtual lines) and a half-resolution line store,
and becomes the swell: it is taken off |dx| before the squarer, so bright areas open the arches sideways into
wide rounded rectangles with hairline piers while the crowns stay at their height; dark areas keep the heavy
round arches. The arcade becomes a halftone of the picture at the scale of its piers. Black input or K6 = 0 is
exactly the clean picture.

- 25 %: the piers thin a little in the highlights (a gentle swerve);
- 50 %: bright areas show slender piers and flattened, wider heads;
- 100 % (default): bright areas reach hairline piers (the clamp), the subject reads as the region of heavy piers.

Source = Osc feeds the shell's drifting 640 x 540 triangle lattice at the same point at 1.5x the camera gain
(it saturates the clamp at its peaks), so diamond-shaped regions of hairline piers drift across. The swell
lags the picture by about 64 px (the box): invisible, since the camera is not shown.

## Engine

- **Field engine** (per field, in the vblank zone): a microcoded accumulator machine (24-bit accumulator, one
  adder, a 256 x 24 register file and the 512-word program and constant ROMs in block RAM, one serial 24 x 16
  multiplier). The program (`aqueduct.py` assembles it; the integer model runs the same words) does the knob
  hold and glide for six channels, the Drift speed (Hypnos law), the near pitch, horizon and camera gain, then
  for each tier: the pitch (2^(i/256) ROM), opening radius and shape constants, the head's squared radius, the
  cornice / base / spring / cut heights, the pan, the cell phase at pixel 0 (a restoring mod and theta x p),
  the distance normaliser (a shift and a 5-bit mantissa), the line widths and the swell limits. About 3,500
  clocks. Per-pixel constants shift into chains of two; the line unit's go into the register file.
- **Line unit** (the first 21 clocks of every line, from the register file): per tier the vertical distance to
  the spring, shape-clamped, its square (one shared ROM squarer), the head term E8 = Rc^2 - dd^2, and the cornice,
  base and archivolt-cut distances in sixteenths of the anti-alias width.
- **Per pixel** (two tiers, stages 20 .. 34): a wrapping DDA in each tier's cell (Q5 virtual px) -> |dx| -> the
  swell (min(L >> octaves, limit)) -> X -> a ROM squarer (256 x 16 for the high bits + the cross term) ->
  N = E8 - X^2 -> >> q, saturated -> x M / 32 = the head distance -> opening, wall and line coverages (17 steps)
  -> composite: sky, the far tier "over" it, the near tier "over" that (Lattice: the two lines added or XOR'd)
  -> luma -> [1 2 1] / 4, 13/16 toward black (white 779), blanking gate.
- Generated avid (the shell's parity-locked start), one latency (48) in every mode, virtual 1080-line
  coordinates from the timing ID (SD shows the centre 4:3).

## Status

- v0.1.0 built 2026-11-16 through the shared queue with **router1**, **every config on seed 1**. Not flashed.
- Post-route Fmax: HD Analog 91.58 MHz, HD HDMI 87.83, HD Dual 92.28 (HD needs 74.25); SD Analog 87.97,
  SD HDMI 92.28, SD Dual 91.52.
- 6,781 .. 6,834 / 7,680 LC (88-89 %), 19 / 32 EBR. No `emulate_transparency`, no carry self-adds (dupin).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh vellum aqueduct`.
- Verified: the integer model (`imodel.py`, running the same microcode words) against the GHDL dump, every
  frame 0 mismatching pixels: 720p decimation 1 default (black input), Source Osc and the bear at K6 100 %
  (921,600 pixels each); 720p / 2 at other knobs (Pointed, Fine, Lattice, Ink); Glide On from the power-up
  defaults with Scale 800 at fields 3 and 9 (the drift phase advancing rightward by the engine's own speed);
  NTSC decimation 1, both fields (174,960 each). Chroma 512 on every pixel of every dump.
- Renders in `.vellum_work/rb_aqueduct/`: cmp_final.png (model | RTL default | RTL Osc | RTL bear), sheet.png,
  strip.png (P12 sweep), motion.png, camera.png, motion_rtl.png (RTL fields 3 and 9), sim/cmp_ntsc.png.

## Cut / deviations

- **Two tiers, not three.** The middle arcade is cut. Three tiers came to about 9,900 LC with the lifting
  camera and about 8,000 LC after every other saving (a microcoded field engine instead of the case-machine
  sequencer, per-line vertical terms, the line unit's constants in the register file, the knob glide moved into
  the engine); two tiers build at 88 %. The picture keeps the brief's layering (sky, a light-grey far arcade, a
  black near arcade with white archivolts) and Depth still goes from one arcade to deep perspective (the ratio
  now reaches 3.2 instead of 2.5, so the far tier shrinks to a band of small arches).
- **Camera = sideways swell, not lift.** The lift (tiers rising and sinking under the image) read best in the
  prototype, but it needs a per-pixel vertical squarer and edges per tier; the swell is the spec's other
  option and is cheap. It opens the arches only sideways so the crowns never cut into the cornice.
- **X pans with partial parallax** ((1 + scale) / 2 of the near offset for the far tier). Full parallax is
  exactly Drift and repeats every pitch, so X's two ends would show the same picture; even so the three X
  panels of the sheet look alike (a pan of a periodic arcade only changes how the tiers line up).
- **Drift tops out at 2^-3 cycle per field** (the brief's 2^-2 strobes).
- **Anti-alias width at 720p is 2 virtual px (1.33 px)**, not 1.5, so every unit conversion is a shift; edges
  are a little softer at 720p than at 1080p.
- **Gable**: the head is an L1 distance squared through the same squarer; its jambs' edges are 1.4 px wide.
- **Ink** draws the outline only round the openings (not the cornice).
- The vblank zone is "no avid for 512 clocks" (the shell used 1024) to give the field engine room in the
  tester's short vblank.
