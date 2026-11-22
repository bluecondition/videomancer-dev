GIMBAL (Kinetic)
v0.1.0 built 2026-11-22, not flashed.  Flash out/rev_b/ron/kinetic/gimbal.vmprog.

WHAT IT DRAWS
The armillary sphere and the gyroscope's gimbals, and the kinetic ring sculptures and desk toys in which nested
hoops each turn about their own axis.  Five flat rings (outer radii 256 .. 900 virtual px at Scale 100 %, the
largest wider than the screen is tall) and a hub disc, each ring hinged across a diameter and tilting in 3-D.
A tilted ring is an ellipse; its two faces carry palette colours A and B (Vega: cyan front, magenta back), its
outer edge an accent rim, and its brightness follows how squarely it faces the viewer, darkening toward its far
half.  The front half of a ring passes in front of the other rings and its back half behind, with a thin
ground-coloured gap where one crosses over another.  At the defaults the wave tilts the rings one after
another, so a ripple of flipping hoops runs outward from the hub (about 7.5 s per cycle); faces flip from cyan to
magenta as rings pass edge-on (a 6-px bar).

ELEMENT AND STATE
- Element: ring k = 0 .. 4 (inner to outer).  The hub disc lies in ring 0's plane and tilts with it.
- State: its tilt.  Rock: alpha_k = Depth * wave(phase + Cascade * index_k) + camera_k.  Tumble:
  alpha_k = (1 + k/4) phase + Cascade * index_k + Depth/2 * wave + camera_k (rings turn continuously at rates
  stepping with the index).  Ring k hinges at 90 deg + k * Fan.
- Field (S8): the index is inherently radial, so Field is Chase (index k: the ripple runs outward) / Mirror
  (index |k - 2|: it runs both ways from the middle ring).
- Geometry: in the hinge frame the ring is c'^2 u^2 + v^2 in [c'^2 Ri^2 - tau^2, c'^2 Ro^2] with
  c' = max(|cos alpha|, 3 / Ro) and tau = 3 px (an edge-on ring is a solid 6-px bar), one quadratic form per
  ring on the screen, normalised so the distance to a threshold is in pixels across the flat side; depth
  z = v tan alpha.

CONTROLS
| Control | Name    | What it does |
|---------|---------|--------------|
| K1      | Scale   | ring radii x 0.55 .. 1.6, exponential, glided (default 571 = x 1.0) |
| K2      | Fan     | hinge step between rings 0 .. 90 deg, glided: 0 = one axis (a slatted disc), 90 = alternating axes (a true gimbal), between = axes spiralling round (default 30 deg) |
| K3      | Depth   | swing, square law, 0.04 .. 0.5 cycle (a half turn) each way, glided; the 0.04 floor keeps a gentle wobble so Fan / Cascade / Wave stay alive (default 0.31: rings pass edge-on and show their backs) |
| K4      | Rate    | bipolar, centre = still (+-8 deadband), 60 s per cycle .. 2 cycles per second (riley's law); default 740 |
| K5      | Colour  | the 16 shared Kinetic palettes (ground, face A, face B, accent rim), hysteresis, both ends reach the first / last palette; default Vega |
| K6      | Camera  | camera (and Osc) strength, default 100 %: the luma under each ring tilts it, bright camera areas tint the faces toward the accent in posterised steps; 0 % = exactly the black-input picture |
| S7      | Wave    | Smooth (sine) / Snap (dwell at +-Depth for ~75 % of the cycle, eased flips) |
| S8      | Field   | Chase / Mirror (see above) |
| S9      | Spin    | Rock (the state swings about zero) / Tumble (continuous turning, rates 1, 1.25 .. 2 x) |
| S10     | Rings   | Wide (116-px bands, 6-px rim) / Wire (31-px hoops, 3-px rim) |
| S11     | Source  | Video / Osc (a drifting diamond lattice in place of the camera, at twice the camera's tilt gain) |
| P12     | CASCADE | phase step per ring 0 .. 0.46 cycle, square law, glided (the standard Spread moved to the slider): 0 % = all rings move as one flipping nest, 50 % = a slow spiral of tilts, 100 % = neighbours nearly in antiphase, a storm of crossing hoops (default 51 %) |

Presets: Armillary (defaults), Gimbal (Fan 90, Outrun), Slatted Disc (Fan 0, Snap, Bauhaus), Orrery (Tumble +
Wire, Ember), Storm (Depth and Cascade full, Mirror, Ultra).

VIDEO
program_type = processing.  The luma goes through riley's 32-px box and vertical 1/8 IIR line store.  With
Source = Osc the smoothed luma is replaced by a 512 x 512 virtual-px diamond lattice (tri(x) + tri(y)) drifting
diagonally, about 1 px per field.  Two uses, both scaled by K6:
- tilt: the smoothed value is sampled once per field at four points of each ring (on the screen axes at the
  ring's mid radius: the centre line every 32 virtual px, the centre column every 32 virtual lines) and the mean
  adds up to 0.25 cycle to the ring's tilt (Osc: 0.5), so the picture bends the geometry, ring by ring;
- tint: bright areas mix the ring faces toward the accent in two posterised steps (0, 1/4, 1/2).
Edges stay clean curves (the camera never moves an edge per pixel).  Black input = no effect.

ENGINE
- Per field: a microcoded 48-bit engine (program, ROMs and an exact emulator in gimbal.py; 412 instructions,
  one adder, a serial multiplier and divider, a 256 x 48 register file in block RAM) runs once per field in
  windows of the active lines and leaves the NEXT field's values in shadow slots: glides, rate, the wave and
  tilts, cos / sin by an interpolated quarter-sine ROM, the quadratic form's coefficients, thresholds, depth
  slopes, and every ring's DDA start values and per-line / per-pixel differences.  It restarts at every field
  start (the watchdog).  Its datapath runs on registered per-state controls (no state decode in the datapath).
  A fill engine rebuilds the 256-entry colour table (face x 32 levels x 4 rim / tint mixes) in the vblank zone.
- Per line: a line engine adds the second differences (Q0 += E, E += E2, D0 += Dy, z0 += Zy for 5 rings) in
  the register file, and loads the pixel DDAs through their own adders (no load multiplexers).  The vblank zone
  copies the shadows into the live slots and loads the pixel constants.
- Per pixel (stages 2 .. 21): per ring three DDAs (the normalised quadratic minus its threshold, 44 bits; its x
  difference; depth, 24 bits), coverage t = W/2 - |Q| (one subtract), outline and rim from bit tests, the hub
  from ring 0's quadratic; front ring F and second ring B by pairwise depth compares (4-px resolution); one
  colour-table read for F (or for B through F's 3-px outline); coverage blend over the ground; output
  conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 with its delay in block RAM, luma [1 2 1]/4, luma 13/16); blanking
  gate.  One latency (28) in every mode; generated avid; virtual 1080-line coordinates (SD shows the centre 4:3).

STATUS
Built 2026-11-22 with ROUTER1, every config on seed 1 (router2 missed: HD Analog failed all six seeds, best
69.1 MHz, at 91 % fill).  Post-route Fmax (Fmin 74.25 MHz HD / 27 MHz SD):
  HD Analog 78.70 | HD HDMI 81.77 | HD Dual 75.15 | SD Analog 72.26 | SD HDMI 73.30 | SD Dual 74.05 MHz.
7,020 .. 7,037 LC of 7,680 (91 %), 21 / 32 EBR, 1 PLL on the SD configs.  All six bitstreams iceunpack clean.
Rebuild: NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic gimbal   (dupin: 0 CARRY I0==I1, 0 LUT
duplicate inputs).  HD Dual has the least margin (+0.9 MHz).
Verified in GHDL against the integer model (imodel.py), decimation 1:
- 720p60, black input, defaults: 0 pixels different.
- 720p60, Source = Osc, Colour at 1023 (Plum, the last palette): 0 pixels different.
- NTSC, black input: both captured fields match the model exactly at consecutive fields (t = 4 and t = 5,
  which differ from each other in 49,674 pixels), so the motion and the SD raster are confirmed.
- 1080i59.94, black input: both captured fields match the model exactly (t = 2 and t = 3).
- 720p60, selfie.jpg as video: the same picture, but 15 % of pixels differ (ring edges a few px apart, tint
  speckle; 11 % once the model takes its samples two fields back as the RTL does): the model's luma store does
  not reproduce the RTL's camera sample values exactly.  The Osc frame drives the same sample, engine and tint
  paths and matches, so the rest is in the box / IIR source side.
SD shows the centre 4:3 of the 16:9 picture (the outer rings run off the sides); rings stay round.
Not flashed.

CUT / DEVIATIONS
- Five rings, not six: six rings plus a separate hub came to 8,279 LC (107 %).  The hub disc lies in ring 0's
  plane (it tilts with ring 0) instead of standing still on its own.
- The edge-on floor is c' = 3 / Ro (a 6-px bar, the hole closing at tau = 3 px) and every band is at least 3 px
  wide, so no ring ever thins to a single aliased row; the anti-alias ramp is normalised across the flat side,
  so the ends of a nearly edge-on ring are a little soft (an offset that shrinks with c' keeps the tails past
  the tips under 5 px).
- The rim is a fixed 6 px (Wire 3 px) times Scale for all rings, not a foreshortened fraction of each band.
- Snap is a clipped-and-eased sine (sin(pi/2 clamp(2.6 sin))), dwelling about 75 % of the cycle.
- Cascade tops out at 0.46 cycle per ring, not exactly half: at exactly half, neighbours share the same |tilt|
  and the rings breathe in step instead of crossing.
- The camera tint mixes toward the accent (bright = more accent) rather than shading brightness: a brightness
  imprint either darkens the default picture on a black input or needs head-room the faces do not have.
- Osc replaces the camera after the smoothing (the lattice is already smooth), not before it.
