# Tufted (Kinetic, wave 2)

v0.1.0 built 2026-11-22, not flashed.

## What it draws

Deep-button tufted upholstery: the Chesterfield sofa, capitonne padded walls, quilted leather. The whole screen is
a lattice of 45-degree diamond pillows meeting at buttons. Every pillow is a soft dome lit from the upper left,
with dark pleats along the seams and a dimple round each button. A travelling wave inflates and deflates the
pillows in turn; past zero a pillow becomes a dimple and its shading inverts (lit on the lower right, dark at the
centre).

At the defaults (Terra palette) about eleven diamonds across and five rows down (diamond 242 virtual px, corner
to corner) of rust leather with cream highlights, dark-brown pleats and cream pearl buttons fill the frame edge to
edge. A slow wave rolls left to right, one crest about every eleven pillows: crest columns are plump with bright
highlights and deep pleats, trough columns flatten to a stitched quilt.

## Element and state

- Element: a diamond pillow. Lattice A = (X + Y) / P + 1/2, B = (X - Y) / P + 1/2 in virtual-1080 units; the
  cell (i, j) = integer parts, the in-cell coordinates p, q = 2 frac - 1 in [-1, 1). A pillow is centred on the
  screen centre, so Scale zooms about the centre.
- State: puff depth d = Puff + Depth x wave(Spread x n - phase) + camera, signed, 64 = 1 (clamped to +-2).
  n = i + j (Linear: vertical wave fronts travelling across) or |i| + |j| (Radial: square rings from the centre
  pillow). Spread is fixed at 0.09 cycle per step (Linear) / 0.13 (Radial). Wave: sine, or Snap (the first
  sixteenth of the sine stretched over the quarter: 75 % dwell, quick flips).
- Shading: s = Tp(p) + Tq(q), two per-field tables of the light term (light from the upper left, a little from
  above) on the profile F = sat(k p), plus a zero-mean height term 1.625 (mean G - G) with G the integral of F.
  k = 1 is a paraboloid dome; a large k gives four flat faces sloping to the buttons with anti-aliased creases
  (k is capped per field so a crease is never sharper than 1.5 px). Tone = 32 + 16 d s: one 8 x 10 multiply per
  pixel. d < 0 inverts the shading (a dimple).
- Pleats: a weight toward the ground colour from the distance to the seam and to the button rim, width
  0.035 + 0.2 |d| cell half-widths, after a solid core of 1.5 px at every scale (the seam is a continuous dark
  line, never a step). Buttons: octagon-metric discs (within 1.7 % of round), 0.21 half-widths, coverage over
  1 .. 2 px, shaded dark B .. B .. light B with a highlight dot.
- Colour: highlight = the brightest of the palette's ground / B / accent (white if none is clearly brighter than
  A); the tone ramp runs black -> A (broad) -> highlight (compact), blended toward the ground by the pleat weight.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | diamond size 96 .. 640 virtual px, exponential, glided (default 500: 242 px, ~11 across) |
| K2 | Shape | pillow profile: 0 = soft paraboloid dome .. 100 % = faceted pyramid, four flat faces with hard AA creases (default 20 %: a slightly squared dome) |
| K3 | Depth | wave depth 0 .. 1 d unit, square law (default 60 %: 0.36, troughs nearly flat; past ~75 % troughs become dimples) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential (default 640: ~19 s per cycle, rightward) |
| K5 | Colour | the 16 shared palettes with hysteresis (default 11 Terra); ground = pleats, A = leather, B = buttons |
| K6 | Camera | camera (and Osc) strength 0 .. 100 %, default 100 %: d += (held luma - field mean) x K6/8 / 32 |
| S7 | Wave | Smooth / Snap (plateaus of equal puff, quick flips) |
| S8 | Field | Linear (fronts sweep across) / Radial (square rings from the centre pillow) |
| S9 | Buttons | Off (plain diamond quilt, seams cross at the corners) / On (buttons in dimples) |
| S10 | Finish | Matte / Gloss (highlight reached early then a blown white spot; pleats 1.5x deeper) |
| S11 | Source | Video / Osc |
| P12 | Puff | the mean inflation 0.1 .. 1.5 d, glided: 0 % = a nearly flat stitched quilt with a ripple of shallow bulges and dimples; 50 % plump; 100 % over-stuffed (deep black pleats, blown highlights, every pillow bulging) |

Presets: Chesterfield (defaults), Patent Gloss, Quilted Ripple, Studded Radial, Overstuffed, Snap Cushions.

## Video

- Source = Video: luma (>>2, less 16) is smeared along x (1/16 one-pole) and written into a per-pillow map (two
  banks of 1024 cells; written at the last pixel of the pillow's inner half, or on the picture's border for cut
  pillows); the next field reads the other bank, so each pillow holds ONE value all field (rigid, never shredded).
  The field mean of the source is measured (sum / count, serial divider in vblank) and subtracted: bright = plump,
  dark = dimpled, whatever the camera's exposure; a black input gives exactly zero (the wave alone runs).
- Source = Osc: the two-axis triangle lattice (640 x 540 virtual px, drifting diagonally) replaces the luma at the
  same gain; its full 0 .. 222 swing is 2 .. 3x a typical camera's deviation from its mean, so Osc reads as bold
  regions of over-stuffed pillows and deep dimples.
- The picture does not colour the pillows (the spec has no face-video switch).

## Engine

Chassis lifted from louver ([A] timing / zone / glides / serial-multiplier sequencer, [B] generated avid, [C]
source oscillator and smear, [F] output conditioning and delay rings); the lattice, fills and look are new.

- Per field (vblank zone, "no avid for 1024 clocks"): 10 products on the shared 24-step serial multiplier (pitch
  exponent, depth^2, puff, 1/P, line step, the two origins, pixel size, rate); the mean divider; then three table
  fills started by registered pulses: colour table (zc 4: six keys, 64 pillow tones x 8 pleat levels and 32 button
  tones x 8 coverages through one time-shared 8-step multiplier, ~4,000 clocks), relief tables (zc 70: two passes
  of the F / G recurrence, ~1,400 clocks), wave table (zc 400: 128 entries, sine ROM + serial multiply, ~1,700
  clocks). Longest finish ~zc 4,100 (NTSC vblank allows ~18,000).
- Per line: the two diagonal DDAs' start values (A += ky, B -= ky).
- Per pixel: A, B += kx; cell id and p, q; table reads (wave by index, map, relief); s; camera term (gated sums);
  d; d x s (gated sums, 3 stages); tone + dither; pleat ROM; button coverage / tone; colour table (1024 x 26).
- One latency in every mode: C_LAT = 24 = processing_delay_clks (17 pixel stages + 4 output stages + 4 px of
  filter delay). Luma 10 bits, chroma 8 bits until the output filter. 22 block RAMs; no RAM reads the address it is
  writing while the read is used (no_rw_check on all; 0 emulate_transparency).

## Status

- Built with router2 (the default) through `./build_set.sh kinetic tufted` (rebuild command), 2026-11-22. Every
  config closed on seed 1 (no other seed needed). dupin.py on the synth json: 0 CARRY I0==I1, 0 duplicated LUT
  inputs, so router1 is also safe if a rebuild ever needs it.

| Config | Router | Seed | Post-route Fmax | LC |
|---|---|---|---|---|
| HD Analog | router2 | 1 | 92.82 MHz | 6834 |
| SD Analog | router2 | 1 | 85.38 MHz | 6850 |
| HD HDMI | router2 | 1 | 83.15 MHz | 6828 |
| SD HDMI | router2 | 1 | 82.73 MHz | 6859 |
| HD Dual | router2 | 1 | 82.85 MHz | 6857 |
| SD Dual | router2 | 1 | 77.67 MHz | 6846 |

- 6,859 / 7,680 LC at most (89.3 %), 22 / 32 block RAMs. vmprog: out/rev_b/ron/kinetic/tufted.vmprog. Not flashed.
- A first build (before a colour-fill restructure) missed timing on HD Analog seeds 1-3 (67.9 MHz on seed 1: a
  quasi-static fill-control path, 12 ns of it routing at 90 % LC); it was stopped, the fill was rewritten as a
  small FSM with one time-shared multiplier and registered start pulses (-120 LC), and the rebuild closed above.

- Model = RTL, 0 mismatching pixels everywhere tested (the 4-column generated-avid offset of the tester capture
  removed): full-raster tester renders at 720p60 (defaults black input; Source = Osc; Video with selfie.jpg), NTSC
  (both stacked fields) and 1080i59.94 (both fields), and a motion pair at Rate max (720p / 2: the captures after 3
  and 6 warm-up frames equal model frames 3 and 6). A mini testbench (320 x 180 at the 720p timing ID, glides
  snapped) also matched with every switch flipped, Shape / Depth max, a luma test pattern on the camera path,
  K6 0, Osc with K6 600, every knob at 0 and every knob at max (K5 at 0 and 1023 select palettes 1 and 16).
- The same picture at SD (the centre 4:3 window, round buttons), 720p and 1080; the camera map's smear is 16 real
  pixels at every resolution (wider in picture terms on SD; it only feeds one value per pillow).
- Single-row check on the model (11 settings): no 1-row features of 24 px or more.

## Cut / deviations

- Highlight colour: the spec's ramp ends at the accent; here it ends at the brightest of ground / B / accent (white
  if none is clearly brighter than A), because for half the palettes the accent is darker than A and the relief
  read inverted (lit side darker) in the first prototype.
- The specular in Gloss is a blown white spot (the highlight reached early, then white over 3 tone steps) rather
  than a thin band: the band read as a ring, the spot reads as gloss.
- The relief carries a height term besides the spec's pure light . gradient term: with the linear gradient alone
  the pillows read as cylinders; the height term turns the iso-shade lines into circles (domes).
- Radial uses |i| + |j| as specified (square rings).
- Camera: the held luma has the field mean subtracted (so "dark = dimpled" holds with any camera); with the
  per-pillow hold at ~55 pillows per screen the subject reads as an emboss pattern, not a portrait.
- Spread fixed at 0.09 / 0.13 cycle per step (K5 carries the palettes, as the set rule says).
