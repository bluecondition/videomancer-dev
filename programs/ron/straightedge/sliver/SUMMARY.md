# Sliver (Straightedge)

**v0.1.0 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/sliver.vmprog`.

## What it draws

Carmen Herrera's hard-edged wedges ("Blanco y Verde"), El Lissitzky's Prouns, searchlight beams crossing: a few
long pointed triangles crossing the whole frame. Each sliver is a wedge: a needle point (its apex), an axis and a
small opening angle; it runs from its point across the screen and off the far edge. Grey: each is a translucent
film of light (a third of white), so where two cross the overlap is lighter and where three cross it is white:
the crossings make new flat facets. Ink: black and white, overlaps XOR. Black, white and greys only (U = V = 512
on every pixel). Every edge on the screen is a straight line in every control state.

    sliver w (w < Count)  apex = G + Spread * dist_w * (cos A_w, sin A_w),  A_w = psi_w + theta_w
                          axis = A_w + dphi_w   (pointing back across the frame, past the gathering point G)
                          two edges through the apex at axis +- half-angle (K3 x 1, 3/4, 5/4, 1/2, 1, 3/4)
    edge function         e = cos(th) (x - apex.x) + sin(th) (y - apex.y), in pixels: one accumulator, one add per pixel
    coverage              clamp(c1 + c2 - 1), c = clamp(e + 1/2): the exact area of the slab between the two edges,
                          so a needle thins evenly to nothing at its point (no blob, no shimmer)
    Bowtie                |c1 + c2 - 1|: the wedge and its mirror through the apex
    Fine                  the same wedge with its edges moved out and in by 4 virtual px, subtracted: two 8 px lines
                          meeting at the point (Bowtie: two full crossing lines)
    tone                  Grey: films add in thirds, clipped at white (Fine: a line is two thirds);
                          Ink: a triangle of the summed coverage (even-odd XOR, still anti-aliased)

The default frame is four slivers at Spread 50 %: a broad wedge from the right, a needle from the upper left and two
more from the left and the top, crossing off-centre around the gathering point, with three- and four-sided facets
where they overlap.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the gathering point (where the apexes close on at Spread 0, and the centre the slivers wheel about), +-1024 virtual px; default centre. Glided |
| K2 | Y | the same, +-576 virtual px. Glided |
| K3 | Scale | the opening of every sliver: half-angle 2.8 .. 28 degrees, exponential (default 8.9); each sliver takes it x1, 3/4, 5/4, 1/2, 1 or 3/4, so there is always a needle and a broad wedge. Glided |
| K4 | Count | 2 .. 6 slivers, stepped with hysteresis (default 4) |
| K5 | Drift | bipolar speed: within +-24 counts of centre = still, else 2^-12 .. 2^-6 turn per field; the slivers wheel at x1, -3/4, 11/16, -1/2, 5/4, -5/16 of it. Default 650 (the lead sliver turns once in about 26 s). The speed is glided (1/8 per field) |
| K6 | Camera | how far the camera's luma reaches up the mosaic's film steps (see Camera); default 100 %, 0 % = the clean figure. Glided |
| S7 | Glide | On (default): X, Y, Scale, Camera and Spread ease 1/16 of the gap per field, never slower than half a count, landing exactly; the Drift speed eases 1/8. Off: the held value of this field, no easing. A knob must move 2 counts to register (pot jitter); the two ends always get through |
| S8 | Form | Wedge / Bowtie (every wedge mirrored through its apex: hourglasses crossing; at Spread 0 a full pinwheel) |
| S9 | Tone | Grey (films) / Ink (black and white, XOR) |
| S10 | Weight | Bold (filled wedges) / Fine (the two edges of each wedge as 8 virtual px lines meeting at the point) |
| S11 | Source | Video (default) / Osc |
| P12 | Spread | the headline. 0 % = every apex on the gathering point: a pinwheel of blades fanning from one point. 50 % (default) = the apexes pushed out 345 .. 490 virtual px, each along its own direction, the wedges pointing back across the frame. 100 % = 690 .. 977 virtual px (0.64 .. 0.9 screen heights; some points leave the frame): broad crossing shards. Glided |

## Motion

Every sliver has its own 24-bit angle (one turn = 2^24, so it wraps without a jump), advanced once per field in
the vblank zone by the Drift speed times its rate (+1, -3/4, +11/16, -1/2, +5/4, -5/16; three turn one way, three
the other). The angle turns the sliver rigidly, apex and axis together, about the gathering point: at Spread 0
that is each blade turning about its own apex; with the apexes spread, each needle point circles the gathering
point while its wedge keeps pointing back across the frame, so the crossings open, close and re-form endlessly and
no sliver ever swings off the screen. Speed: Hypnos's bipolar decode, +-24 counts still, then exponential
2^-12 .. 2^-6 turn per field for the lead sliver. The top is capped at 2^-6 (5.6 degrees per field; a quarter turn
per field would strobe a needle that is 6 degrees wide). The six accumulators live in spare words of the edge
table's block RAM (read, stepped and written back by the field sequencer).

## Camera

The slivers themselves stay rigid: the camera never touches an edge function.

- Source field: luma (minus black, 8 bits) -> a 32-pixel box (16 pixels at 720p and SD) -> a vertical one-pole
  (1/16 per line; 1/8 at 1080i and progressive SD, 1/4 at 480i / 576i).
- Grid: 32 x 32 virtual px blocks (60 x 34). On the lower half of every block row each stored column overwrites
  its block's cell, so a cell ends as the filtered luma at the block's last column and line (the filter's own
  delay centres that on the block). 8 bits, one write statement, two banks ping-ponged by the per-field zone
  start: a field writes one bank and reads the other, zero start-up contents.
- Read: the address is the pixel's own block; the data goes through a fabric register and a per-field table
  (K6) and is constant across the block.
- What it drives: a FLAT TONE STEP. Each block is cut by one diagonal into two triangles (alternate blocks take
  the other diagonal), and the held luma lays 0, 1 or 2 films on them; the lower triangle is a quarter step ahead
  of the upper, so flat tone regions meet in saw-tooth borders of half-block triangles. The subject appears as a
  three-tone triangle mosaic under the slivers (Grey: the films add to the slivers'; Ink: they XOR, so the
  subject solarises into contour bands).
- K6 = 0 or a black input: exactly the clean figure. 25 %: one film on blocks brighter than 67 % of white: the
  highlights appear. 50 %: one film from 39 %, two from 78 %. 100 % (default): one film from 30 %, two from 59 %:
  the whole subject as a mosaic, the slivers light on it. (K6 follows 1 - (1 - k)^2, so the low end is not dead.)
- Osc: the shell's drifting 640 x 540 triangle lattice in place of the luma, through the same table: diamonds of
  one and two films with saw-tooth rims, stepping slowly across the frame.

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks"): the hold + glide of six knobs, then a sequencer of
  126 slots of 24 clocks on ONE serial 16-step multiplier (about 3,060 clocks): six global slots (opening from
  the 2^(i/256) ROM, gathering point, spread, speed, camera gain), then per sliver six apex slots (phase step,
  sine and cosine of the apex direction from the 1024-point quarter-wave ROM interpolated to 16-bit angles, the
  apex) and seven slots for each of its two edges (sine and cosine of the edge normal, the line step, the seed
  with the half-pixel and Fine bias folded in, the pixel step, the line wrap) written to the edge table. The
  camera table (256 entries) is refilled by an accumulator.
- **Per line**: no multiplier. Each edge is one 28-bit DDA in pixel units (Q14) with its own increment register:
  it adds the pixel step on every clock of the line window (W + 8 steps), then once more with the wrap parked in
  that register from the edge table, so the adder has no input mux (Hardline dazzle's engine, 12 edges).
- **Per pixel** (9 stages, no multiplier): clamp each edge to 0 .. 255 sixteenths -> outer and inner coverage ->
  per sliver the slab coverage for Wedge / Bowtie / Fine -> the six summed -> plus the mosaic's films (grid RAM ->
  fabric -> camera table -> fabric, beside the diagonal's coverage) -> Grey clip or Ink triangle -> luma.
- **Output**: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (20) in every mode, virtual 1080-line coordinates from the timing ID
  (interlaced modes use a 2-pixel edge ramp, a field line being two pixels tall).

Chassis: Kinetic riley by way of Hardline dazzle: [A] timing / zone / width / generated avid, the [B] hold +
glide datapath, the serial multiplier and DDA loader of [C], the [D] source field (Osc lattice with dazzle's
line-0 fix, box, one-pole) and the luma half of [F]. No wave engine, palettes or chroma paths.

## Status

- v0.1.0 built 2026-11-17 through the shared queue, **router2** (the builder default), **every config on seed 1**.
  Not flashed.
- Post-route Fmax: HD Analog 80.33 MHz, HD HDMI 78.06, HD Dual 75.68 (HD needs 74.25); SD Analog 78.88,
  SD HDMI 81.23, SD Dual 83.76 (SD needs 27).
- 6,870 .. 6,894 / 7,680 LC (89.8 %), 21 / 32 EBR (grid 8, line store 4, sine 4, edge table 2, box, camera
  table, exponent 1 each). No `emulate_transparency` (standalone yosys: 4,095 LUT, 2,732 FF).
- Rebuild: `./build_set.sh straightedge sliver`.
- **Model = RTL** (GHDL, the built sources; the GHDL frame is the model's shifted 4 columns right with 4 blank
  columns at the left, the shell's generated-avid start):
  - full 720p60 frames (decimation 1): pixel diff 0 of 918,720 at the defaults with a black input; 0 with
    Source = Osc; 0 with bear.png as the camera at K6 100 %.
  - NTSC (720 x 243 per field, bear.png, K6 100 %): 0 of 173,988 on each field, with the grid taken from the
    OTHER field's luma, which also confirms the one-field-old bank ping-pong. Same slivers as HD (the centre 4:3
    of the 16:9 figure).
  - 720p quarter rasters (decimation 2), all 0 of 228,960: defaults black / Osc / bear; Bowtie + Fine with six
    slivers, X, Y, Scale, Spread and K6 moved, bear; Ink + Fine + Osc with five slivers; Bowtie + Ink pinwheel
    (Spread 0, two slivers, Scale 100 %).
  - Motion: Drift 650, Glide Off, captures after 2 and 6 warm-up frames match the model at 3 and 7 fields of
    travel (0 diff each; one field either side differs by about 5,000 pixels), so speed and direction agree.
  - Glide On at the power-up defaults: 0 diff against the model at 3 fields of the default Drift.
  - Not rendered: 1080p, 1080i, 576i, and a glide in flight (the easing steps themselves are unverified in RTL).
- Renders: `.straightedge_work/rb_sliver/cmp_final.png`, `sheet.png` (control sheet), `strip.png` (P12 travel),
  `motion.png`, `camera.png` (K6 = 25 / 50 / 100 %), all from the integer model `imodel.py` except the RTL
  panels of cmp_final.

## Cut / deviations

- **Camera target changed** (the spec allows it). The spec's target, the block luma added to both edge functions
  of every sliver (a parallel shift up to 0.25 screen heights), was prototyped: it shredded every edge into ragged
  saw teeth at the block borders, flooded the frame and did not make bear.png recognisable. Per-block
  mini-sliver halftones (five variants) were busy and read no better. The spec's fallback, a flat film step per
  block, is what shipped, cut into two triangles per block so the mosaic is made of pointed primitives; the
  slivers themselves stay exact.
- **The slivers wheel about the gathering point, not each about its own apex.** Turning a spread sliver about its
  own apex swings it off the screen for half of every turn (the motion strip was mostly black); turning apex and
  axis together keeps every frame a full composition. At Spread 0 the two are the same motion.
- **Spread reaches 0.64 .. 0.9 screen heights at 100 %**, not "about 0.6": the 50 % print needs the points
  345 .. 490 virtual px out to read as scattered.
- **Rates** 11/16 and -5/16 stand in for 2/3 and -1/3 (shift-adds; each sliver has its own accumulator, so every
  one still wraps exactly one turn).
- **Per-sliver openings** (x1, 3/4, 5/4, 1/2, 1, 3/4 of Scale) are an addition: one needle and one broad wedge at
  every setting.
- **Osc runs at the camera's gain**, not 2 - 4x: the lattice already swings the full range (a camera picture
  rarely does), and at 2x everything sat on two films.
- **Drift's top speed is 2^-6 turn per field** (the set's 2^-2 strobes).
- The per-field toggle of the grid banks and the phase step are taken at the shell's vblank zone start (no avid
  for 1024 clocks: once per field, immune to vsync serration), not on the vsync edge itself.
- In Grey with a bright camera picture at K6 100 % the mosaic's two films plus one sliver film reach white, so
  sliver overlaps there no longer show; turn K6 down for more sliver, up for more subject.

## Not sure of on hardware

- Nothing on this chassis has been flashed yet. The generated avid, zone and width logic are the heirloom
  shell's, which is hardware-validated.
- HD Dual closed at 75.68 MHz on seed 1: a 1.9 % margin, and router2 is run-to-run noisy near the line.
- The grid is one 4096 x 8 inferred RAM (8 EBR with an output mux); the phase accumulators are read-modify-written
  in the edge table's EBR across several clocks. Sims model both ideally.
- The mosaic is flat steps: a block whose luma sits on a threshold can pop between states on a noisy camera
  (the box and one-pole average about a thousand pixels per block, so it should be rare).
- Moving sharp full-swing edges (Ink, Bowtie pinwheel at full Drift) are the HD HDMI bar trigger; the [1 2 1] +
  13/16 conditioning is built in.
