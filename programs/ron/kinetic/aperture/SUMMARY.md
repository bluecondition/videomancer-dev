# Aperture (Kinetic, wave 2)

v0.1.0 built 2026-11-22, not flashed. Flash `out/rev_b/ron/kinetic/aperture.vmprog`.

## What it draws

A wall of iris diaphragms, edge to edge. Each square cell is an 8-blade iris: a regular octagonal hole of
glass in the middle and, around it, eight flat blades that partition the rest of the housing like a pinwheel,
each blade one lit metal tone (palette A shaded toward black, and tinted toward the accent on the side facing the
light), with thin dark seams between the blades. A travelling wave opens and closes the irises in turn.
The art it comes from: the camera iris, and Jean Nouvel's Institut du Monde Arabe facade (a wall of mechanical
irises that open and close with the light). At the defaults (Ice palette) about 8 x 4.5 steel shutters with
rounded corners and a thin steel rim on near-black ground, the holes bright near-white glass; a diagonal band of
shut irises (8-way star seams) and a band of wide-open ones roll across the wall.

## Element and state

- Element: the iris of one square cell of pitch P. Cell (0, 0) is centred on the screen, so Scale zooms about the
  centre. S10 Stagger shifts alternate cell rows half a cell.
- State: the aperture inradius a = f * amax, f = clamp(Open + Depth * wave(theta) + source, 0, 1).
- theta = Spread * index - phase, Spread frozen: Linear 0.1 cycle per cell, index i + j (the wave runs down the
  diagonal); Radial 0.2 cycle per cell, index max + min / 2 of (|i|, |j|) (octagonal rings from the centre);
  staggered rows count i + 1/2. wave = sine, or Snap (dwell at the extremes, quick flips; Depth x 1.5 so the dwells
  reach shut and fully open).
- Geometry, per pixel, cell-centred (p, q): four linear forms L0 = p, L1 = (p + q) 0.7071, L2 = q,
  L3 = (q - p) 0.7071; edge k (normal at k * 45 deg) e_k = sigma L - a. The hole is where every e_k < 0; blade k owns
  the pixel where e_k >= 0 and e_(k+1) < 0 (exactly one blade outside the hole: checked on 8 M random points).
  So each blade is bounded by its own edge line and the next blade's, which is the iris swirl; at a = 0 the blades
  meet in an 8-way star seam, near amax they are slivers at the rim.
- Housing: a rounded square (K2: corner radius from a few px to a full circle) with a rim ring; the frame between
  cells and the corners outside a round housing are the ground colour.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | Scale | cell pitch 150 .. 1120 virtual lines, exponential, glided (default 240: about 8 x 4.5 irises) |
| K2 | Shape | housing: square shutters filling the cell (thin ground frame) .. round portholes with a rim ring and ground between; rim and frame grow with it |
| K3 | Depth | wave depth 0 .. a full aperture, square law (default 0.40: shut stars to wide holes) |
| K4 | Rate | bipolar, centre = still (+-8 deadband); 60 s per cycle .. 2 cycles per second, exponential |
| K5 | Colour | the 16 shared palettes (ground = frame, A = blade metal, B = glass, accent = rim; seams dark A or shaded ground) with hysteresis; Spread is frozen (see above) |
| K6 | Camera | camera (and Osc) strength 0 .. 100 %, default 100 %: the cell's held luma opens its iris (bright = open; white at 100 % adds 1.25 apertures); 0 % = exactly the black-input picture |
| S7 | Wave | Smooth / Snap (irises snap between shut and wide open) |
| S8 | Field | Linear (diagonal wave) / Radial (rings from the centre) |
| S9 | Hole | Glass (palette B) / Video (the camera seen through the holes: a wall of lenses; with Osc, the lattice's colours) |
| S10 | Pack | Square / Stagger (alternate rows offset half a cell: brick bond) |
| S11 | Source | Video / Osc |
| P12 | Open | the mean aperture, glided: 0 % = shut (star seams; Depth and the camera still open some), 100 % = wide open, blades reduced to slivers, the wall almost all glass with the wave flickering it |

Presets: Iris Wall (defaults), Portholes, Shutter Snap, Radial Bloom, Lens Wall, Brass Brick.

## Video

- Source = Video: the luma (>> 2, less 16) is smeared along x by a 1/64 one-pole filter, scaled by the camera gain
  (a per-field 256-entry table: s * 171 K6 / 2^17), and HELD per cell: sampled on the last line of each cell row at
  each cell's last pixel (of the next row's grid in Stagger), in two banks by row parity. It adds to f, so bright
  areas open their irises and the picture becomes a halftone of apertures. Every iris stays a rigid, clean octagon.
- Black input: the offset is zero and the wave alone runs.
- Source = Osc: the two-axis triangle lattice (640 x 540 virtual px) drifting diagonally, BIPOLAR about its centre
  (2 x the camera's gain), so diamonds of shut and wide-open irises drift across the wall (a unipolar Osc at 2-4 x
  only washed every iris open).
- S9 Hole = Video: the hole shows the camera (input delayed in a block-RAM ring), Osc: the lattice colours.

## Engine

- Per field (vblank zone = no avid for 1024 clocks): glides (Scale, Open), the Camera knob glide, a 16-step serial
  multiplier (pitch exponent, depth^2, housing frame / rim / corner radius, rho^2, (rho - rim)^2, camera gain, Osc
  centre, rate), the palette's T2 (rim colour -> ground in 9 steps, the glass) copied from a per-palette ROM, T1
  (8 blade tones x 9 seam-coverage steps, adders only), the aperture table a(f) (256 entries), the camera gain table,
  and three origin searches (even rows, odd rows, y).
- Per cell: a serial engine computes the NEXT cell's aperture ahead of the beam (base phase from a 256-entry
  cell-index ROM, minus the phase, wave ROM, Depth x wave in 9 shift-add steps, + Open + held source, clamp, the
  aperture table), 21 clocks per cell; the next line's first two cells are computed at the end of each line.
- Per line (one line ahead): the housing's half-widths W = c + isqrt(max(rho^2 - u^2, 0)) for the outer and the
  rim edge (a serial squarer and a serial isqrt), the vertical coverages for the straight top / bottom edges, and
  the anti-alias ramp from |W(y) - W(y-1)| rounded up to a power of two.
- Per pixel (stages 2 .. 20, then 4 output stages + 4 px of filter delay): p, the diagonal forms (x 181/256 as
  four adders), four magnitude comparators |L_j| >= a -> the eight edge signs -> the one-hot blade, the two bounding
  edges by and-or select, the seam distance and the hole distance -> coverages in eighths (over 1 px at 1080p,
  2 virtual px otherwise); |p| against the line's half-widths -> the rim and ground coverages; ONE read of T1
  (blade tone toward the seam) and ONE of T2 (rim/ground, or glass; or the camera) and one lerp
  out = I + (Z - I) c / 8; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two resynced 8-sums, luma [1 2 1]
  delayed to the same group delay, luma 13/16 toward black), blanking gate.
- One latency in every mode: C_LAT = processing_delay_clks = 27. Generated avid (parity-locked start), virtual
  1080-line coordinates from the timing ID, colours in the hardware convention (U register = Cr), luma 10 bits,
  chroma 8 bits up to the output filter. No per-pixel multiplier larger than the 4-bit lerp. Block-RAM data gets
  a fabric register before arithmetic; every RAM has one write statement; no_rw_check only where no read meets the
  written address while the picture is drawn (the hold's banks are split by row parity).

## Status

- Built through the shared queue with ROUTER1 (`NEXTPNR_ROUTER="--router router1" ./build_set.sh kinetic aperture`,
  rebuild command), 2026-11-22. A first router2 build missed timing on HD Analog seeds 1-4 and stalled (2 overused
  wires) on seed 5, so it was stopped and rebuilt with router1 (dupin.py on the synth json: 0 carry or LUT
  self-adds).

| Config | Router / seed | Post-route Fmax | LC |
|---|---|---|---|
| HD Analog | router1 / 1 | 75.33 MHz | 7191 |
| SD Analog | router1 / 1 | 81.13 MHz | 7199 |
| HD HDMI | router1 / 1 | 78.86 MHz | 7191 |
| SD HDMI | router1 / 1 | 76.58 MHz | 7198 |
| HD Dual | router1 / **3** | 78.91 MHz | 7203 |
| SD Dual | router1 / 1 | 79.79 MHz | 7201 |

- 7,203 / 7,680 LC at most (93.8 %), 18 / 32 block RAMs. All six bitstreams pass iceunpack. HD Analog's margin is
  thin (75.33 against 74.25); HD Dual needed seed 3 (seeds 1, 2 missed).
- GHDL render = integer model (`.kinetic_work/rb_aperture/imodel.py`, frame 2, the capture 4 columns right):
  - defaults, black input, Rate running, full 720p60 frame (decimation 1): 0 mismatching pixels of 912,960;
  - Snap + Radial + Stagger + Shape 1023 + Depth 1023 + Colour 0 (Ink), 720p: 0 of 912,960;
  - NTSC (720 x 243 per field, Rate still, Colour 1023 = the last palette): 0 of 172,044 in each field. The SD
    frame shows the centre 4:3 of the HD picture (the same rows, about 6 of the 8 columns);
  - 1080i5994 (1920 x 540 per field, Rate still): 0 of 1,030,320 in each field;
  - camera (selfie.jpg, K6 100 %), 720p: 542 pixels differ, all on frame line 0 (that line uses the previous
    field's held values; the model gives it the current ones); Source = Osc: 980, all on line 0 (the lattice moves
    every field); S9 Hole = Video with the selfie: 544, line 0 plus two pixels 1 code apart in luma (the model's
    float RGB -> YUV of the input rounds two exact .5 cases the other way from the tester's matrix product);
  - motion: a capture 6 frames later (frame 8, 720p) = the model's frame 8: 0 of 912,960, and 192,561 pixels
    differ from the frame-2 capture (the wave rolls down the diagonal as in strip.png);
  The model reproduces the tester's input exactly (U/V pre-swapped at full size as sim.sh does, Lanczos, BT.601),
  holds samples 8 px after each cell's last pixel, and counts the Osc lattice one field ahead of the phase.
- The model's output has no single-row hard lines at 720p or 1080p (thincheck.py, 13 settings x 3 phases); on SD
  the horizontal seams are single FIELD rows, which the hardware line-doubles (field_n is tied high).

## Cut / deviations

- The glass's recessed inner shadow (a shaded ring inside the hole) was cut for fit: the hole is flat palette B.
- The camera through the hole (S9 Video) is the input pixel as it is (no box filter) and unshaded.
- Spread is frozen at 0.1 cycle per cell (Linear) and 0.2 (Radial, doubled so the rings read); Colour is on K5.
- Snap multiplies Depth by 1.5 so its dwells reach fully shut and fully open (an unmistakable look-flip).
- The Osc source is bipolar about its centre (the camera stays unipolar, so a black input adds nothing).
- amax = 15/16 of the housing's inner half-width, so the slivers at full open stay at least 2 px tall.
- SD: the seam half-width gets +1.125 virtual px (seams 2.25 px bolder than at HD) so no seam is a single SD row.
- The housing's anti-alias ramp is rounded up to a power of two per line, so the edge softness of a round housing
  varies between 1 and 2 px around the circle; the top / bottom straight edges use a per-line vertical coverage.
- The top line of the frame (line 0) uses the previous field's held camera values (the top cell row is sampled on
  line 0 itself).
- Fit history: the first RTL (per-pixel state pipeline, per-pixel squarer for the housing, three colour
  multiplies) packed to 7,394 LC standalone (about 8,800 with the core); see notes.md for the cuts that brought it
  to 5,711 standalone / 7,203 built.
