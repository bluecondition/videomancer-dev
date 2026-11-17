# Meander (Straightedge)

**v0.1.0 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/meander.vmprog`.

## What it draws

Friezes of the Greek key stacked up the screen, after the meander borders of Attic pottery and mosaic floors and
Frank Stella's pinstripe paintings. Each frieze is one continuous line: a square wave (a battlement) whose every
box, merlon and crenel alike, holds a square spiral that leaves the wave at the foot of the box's wall and winds
inward to the eye. Fully wound it is the classic running key, line as wide as the gap. Alternate rows are mirrored,
lean the other way and run the other way. Luma only (U = V = 512 on every pixel); every edge is a straight line.

    unit     16 x 8 nodes; a ROM gives each node its arms (N / E / S / W), its order along the spiral (0 .. 49) and
             the headings in and out. The line is every point within h (Chebyshev) of the path up to arc length L
             (the Wind): it grows along itself, ends in a square cap, and its corners are closed squares.
    row r    u = (x - 960) / P -+ (X + drift_r) +- S (v - 4)        nodes, modulo 16    (S = the slant, |S| <= 1)
             v = (y - 540) / P + pan                                 nodes, modulo the row period (10 or 8)
    coverage every edge is u = const or v = const of its row; a wrapping DDA crosses each node in anti-aliasing
             units (one unit = the pixel's footprint across the edge), so coverage = clamp(distance + 1/2), 17 steps

The default frame (Scale 40 %, Wind 100 %): four friezes of the full key, node pitch 27 virtual px (line 13.5 px),
4.4 units across; rows 1 and 3 white on black, rows 2 and 4 light grey on a dark grey slab, plain black bands
between them; the rows slide slowly against each other.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | slides the alternate rows against each other: even rows +, odd rows -, 3.5 nodes each way |
| K2 | Y | pans the stack, 1.5 row periods each way |
| K3 | Scale | node pitch 10.8 .. 108 virtual px, exponential: about ten friezes .. one giant frieze filling the height (default 27 px: four friezes) |
| K4 | Slant | bipolar shear of every row, a +-16-count dead centre = the right-angle key; either way every vertical leans, up to 45 degrees at the ends, and the frets become pointed zigzags; alternate rows lean opposite ways (chevrons) |
| K5 | Drift | bipolar speed, the Hypnos law: +-24 counts still, then 2^-11 .. 2^-5 unit (16 nodes) per field; rows run at +1, -3/4, +85/128 (2/3), -1/2 of it. Default 660: 0.6 virtual px per field at the default scale (12 s per unit for the fastest row) |
| K6 | Camera | the line's weight per node, from the block-held luma: black = the clean key, white = a heavy line with hairline gaps (7/8 duty). Default 100 % |
| S7 | Glide | On (default): X, Y, Scale, Slant, Camera and Wind ease 1/16 of the gap per field, never slower than one count, landing exactly; the Drift speed eases 1/8. Off: no easing at all. A +-1-count hold on every knob |
| S8 | Form | Frieze (rows of 8 nodes separated by plain bands of 2) / Carpet (the rows abut: an all-over labyrinth) |
| S9 | Tone | Grey (even rows white on black, odd rows light grey 0.67 on dark grey 0.20) / Ink (every row white on black) |
| S10 | Weight | Bold (line = gap) / Fine (line 5/32 of the pitch, at least 2 virtual px) |
| S11 | Source | Video / Osc |
| P12 | WIND | how far the spirals are wound, continuous along the path (49 nodes): 0 % = the bare square wave, 43 % = a single hook (the sketch), 50 % = one full ring inside the box, 100 % (default) = the full triple-wound key |

## Motion

Four phase accumulators (20 bits, one per row class = row modulo 4), advanced once per field at the start of the
vblank zone (no avid for 1024 clocks: once per field, immune to serrated vsync) by the Drift speed times 1, 3/4,
85/128 and 1/2. They wrap modulo exactly one unit (16 nodes), so the slide never jumps. Even rows slide right and
odd rows left for Drift above centre. Range 2^-11 .. 2^-5 unit per field (the brief's 2^-2 top is capped: half a
node per field is where the fine key starts to strobe). Nothing else moves and nothing stops it but Drift at centre.

## Camera

The source (luma, or the Osc lattice) goes through the shell's field (box over 32 virtual px, vertical one-pole)
and is written, 8 bits, into a 60 x 34 grid of 32 x 32 virtual-px blocks in block RAM: two banks, the bank being
written flips every field, the pixel path reads the other one (one field old). Every NODE reads the block under
its own centre: the block row is latched once per node row, the block column comes from the node centre's x,
which is carried exactly from line to line, so the value is the same on every line and pixel of the node, also
when the row is slanted. A per-field table turns it into the node's half line width:

    h = hb + (7/16 - hb) min(1, 1.25 K6 luma)        hb = 1/4 node (Bold), 5/64 (Fine)

So the subject appears as a mosaic of heavier and lighter pieces of the key; every edge stays straight and the
only new edges are the steps in line weight at node borders. Black input or K6 = 0 is exactly the clean key.

- 25 %: highlights thicken the line a little (duty 50 % -> 62 % at white);
- 50 %: bright areas read as heavy key, 73 % duty;
- 100 % (default): bright areas reach the cap (hairline gaps), dark areas keep the plain key: the subject is the
  region of open key. With the default four friezes this is a coarse silhouette (57 x 32 nodes, a quarter of
  the height is plain band); Carpet, Ink or a smaller Scale make it clearer.

Source = Osc feeds the shell's drifting 640 x 540 triangle lattice at twice the camera level: diamond islands of
plain key in heavy key drift across.

## Engine

- **Per field** (vblank zone, about 1,050 clocks): hold + glide (six channels), the Drift speed and phases, then 15
  slots on one serial 34 x 16 multiplier: pitch and 1 / pitch through a 2^(i/256) ROM, the footprint and its
  reciprocal (one bit-serial division), the anti-aliasing units per node across and down, the per-line step, the
  start values, the wind and the tip's reach, the camera slopes; then the two weight tables (256 entries each,
  by accumulation).
- **Per line** (92 clocks from each line start, computing the NEXT line, one serial multiplier): the row and node
  row, the distance to the node's centre line, the row's u at the pre-pixel with its lean, the DDA's start, the
  first node's centre x and (per node row) the grid rows.
- **Per pixel** (stages 2 .. 15, no multiplier): the node DDA (two adds and a select) -> grid RAM -> weight
  tables and node ROM -> four edge coverages of the node's centre square, the node's state (drawn / tip leaving /
  tip entering / not reached) -> bars (min), arms (max), the tip's square end -> tone ROM -> luma [1 2 1] / 4,
  13/16 toward black (white 776), blanking gate.
- Generated avid (the shell's parity-locked start), one latency (24) in every mode, virtual 1080-line coordinates
  from the timing ID (SD shows the centre 4:3).

## Status

- v0.1.0 built 2026-11-17 through the shared queue with **router1**, **every config on seed 1**. Not flashed.
- Post-route Fmax: HD Analog 90.56 MHz, HD HDMI 92.13, HD Dual 84.04 (HD needs 74.25); SD Analog 82.82,
  SD HDMI 90.73, SD Dual 86.50.
- 6,919 .. 6,939 / 7,680 LC (90 %), 17 / 32 EBR (grid 8, source line store 3, box 1, weight tables 2,
  exp + tone ROM 2, key ROM 1). No `emulate_transparency`, no carry self-adds (dupin: none). Six bitstreams
  pass iceunpack. (Built twice: the second build only changed a preset and gave the same numbers.)
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge meander`. Router2 was not tried.
- Verified, integer model (`imodel.py`) against the GHDL dump, 0 differing pixels in every one: 720p decimation 1
  default with a black input, Source Osc and the bear at K6 100 % (921,600 pixels each; see the list below);
  720p / 2 with Slant, X, Y and a partial Wind; Carpet + Fine + Scale 10 % + Slant the other way; the bear;
  the default Drift at fields 3 and 9 (the model after 4 and 10 drift steps: six fields, six steps, even rows
  moving right); Glide On from the power-up defaults; NTSC decimation 1. Chroma 512 on every pixel of every dump.
- Renders in `.straightedge_work/rb_meander/`: cmp_final.png (model | RTL default | RTL Osc | RTL bear),
  sheet.png, strip.png (Wind sweep), motion.png, motion_rtl.png, camera.png.

## Cut / deviations

- **The camera drives the line weight per node, not the wind per key.** Wind per key gives a 7 x 4 mosaic at the
  default scale (the bear is not recognisable), and it would be dead at Wind 100 %. Weight per node uses the
  whole block grid and works at every Wind.
- **The key is a square wave with a spiral in every box** (two T junctions per unit), not a rail with hooks: that
  is what gives a real battlement at Wind 0 % and a line that only ever grows along itself.
- **Default Wind is 100 %** (the full key is the print); 50 % is one full ring plus a stub, the sketch's single
  hook is at 43 % (the "Single Hook" preset).
- Drift tops out at 2^-5 unit per field and starts at 2^-11 (the brief: 2^-2 and 2^-12); the third row rate is
  85/128, not 2/3.
- Fine is 5/32 of the pitch (the brief: about 1/6).
- The camera's cap is 7/16 node (hairline gaps), not solid: the key stays visible in the highlights.
- The edge of the odd rows' dark grey slab against the black band is a hard horizontal edge (not anti-aliased);
  it only moves with Y and Scale.
- X and a still Drift only shift a periodic key, so their sheet panels look alike.
- **SD**: the same picture in the same place (the centre 4:3), but a 480i field line is 4.44 virtual px tall, so
  below about Scale 25 % (pitch under 20 px) horizontal lines soften toward grey, and Fine horizontal lines
  (2 virtual px) are faint at every scale below about 30 px pitch. 720p softens only the very smallest Fine lines.
- Not verified on hardware: anything.
