# Tans (Straightedge)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/straightedge/tans.vmprog`.

## What it draws

The tangram: seven flat pieces cut from one square (two large triangles, a medium triangle, two small triangles,
a square and a parallelogram), each its own flat grey on black, assembled into a figure nearly the height of the
screen with hairline black seams. Every few seconds the pieces glide and turn, rigidly, into the next figure; the
scatter (P12) spreads them across the screen in a composed Lissitzky / Malevich arrangement. Black, white and
greys only (U = V = 512 on every pixel); every edge on the screen is a straight line in every state.

- The eight figures (K4, and the order of the Drift cycle): Square (the classic dissection of the 4 x 4 square),
  Cat (sitting, ears, tail, paws), House (roof, V facade, chimney, annex), Boat (main sail, pennant, cabin,
  trapezoid hull), Swan (raised tail, neck, beak), Fish (diamond body, fan tail, dorsal and ventral fins), Arrow
  (head, shaft, tail fin), Pyramid (the seven pieces as one large triangle). Every figure was checked in the
  prototype: total area 16, zero overlap between any two pieces, edge contact (connected), and the parallelogram
  in its native chirality (no figure needs it mirrored, so no mirror bit: a mirror is not a rigid motion). Swan and
  Pyramid tilings came from an exact-cover solver on the 45-degree grid (.straightedge_work/rb_tans/tiler.py).
- Each figure is scaled to fill 0.92 of the screen height (or 0.92 of the 16:9 width): the scale glides with the
  flight (a zoom), so the pieces never change shape.
- Edges: a piece is the intersection of its 3 or 4 half-planes; each edge is the zero of e = d S - n . (p - P),
  offset 1.5 virtual px inside the true outline, so assembled pieces show a clean black seam (3 virtual px).
  Coverage = clamp(min of the piece's edge values + 1/2 px): anti-aliased over one pixel, sharp exact points.
- Grey: the seven greys LA 13, LB 9, MT 6, SA 11, SB 7, SQ 15, PG 4 sixteenths of white. Where pieces overlap
  (in flight, scattered) the tone is the brightest piece plus a quarter of the others: a lighter film, never
  blown out. Ink: every piece white, overlaps XOR (anti-aliased).
- Fine: each piece a 3 virtual-px outline (from the piece's inner distance; 2.94 px at 1080p), every outline the
  same light grey (14/16 of white), lighter where outlines cross.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | the figure's centre, +-960 virtual px (default centre). Glided |
| K2 | Y | +-540 virtual px. Glided |
| K3 | Scale | the figure's size, exponential x0.42 .. x2.38 (default: the square 0.92 of the height). Glided |
| K4 | Figure | which of the 8 figures the cycle starts from (stepped labels Square .. Pyramid, hysteresis 8 counts inside a zone with both ends always reachable); with Drift still it simply selects the figure |
| K5 | Drift | the cycle: each figure dwells half a step, then the pieces fly to the next; bipolar = the order (negative runs the cycle backwards); +-24 counts of centre = still; speed 2^-12 .. 2^-8 cycle (8 figures) per field, exponential. Default 650 = about 4.4 s per figure at 60 Hz. Speed glided 1/8 per field |
| K6 | Camera | the block luma's levels replace the piece greys by K6 (0 = the clean figure); default 100 % |
| S7 | Glide | On (default): X, Y, Scale, Camera, Scatter ease 1/16 of the gap per field (never slower than half a count, landing exactly), the Drift speed 1/8. Off: the held value of this field. A knob must move 2 counts to register (pot jitter); both ends always get through |
| S8 | Form | Single / Mirror: the screen folded left-right at its centre; the figure moves into the right half at 0.72 scale, so two figures face each other (totem-like) |
| S9 | Tone | Grey (seven greys, lighter films) / Ink (white pieces, XOR) |
| S10 | Weight | Bold (filled, seven greys) / Fine (3 virtual-px outlines in one light grey, the camera at half strength inside) |
| S11 | Source | Video (default) / Osc (the shell's drifting triangle lattice through the same levels) |
| P12 | Scatter | the headline. 0 % (default) = the assembled figure; rising = every piece slides toward its own slot of a composed scatter, turns by its own amount (-1.5 .. +3 eighth-turns) and grows (x1.15); 100 % = the seven pieces spread large across the whole screen, overlapping in lighter films. The figure keeps 20 % of its own layout and its angles, so Drift still moves everything at 100 %. Glided |

## Motion

One 24-bit phase (8 figure steps = 2^24, so the cycle wraps from the Pyramid back to the Square with no jump),
stepped once per field by the signed Drift speed. In each step the first half is the dwell; in the second half
every piece flies from its pose in figure A to its pose in figure B: position linearly, angle the short way (using
the square's 90 and the parallelogram's 180 degree symmetry), both through a smoothstep ease; the figure scale
interpolates too. Negative Drift runs the same paths backwards. Speed: Hypnos's bipolar decode, +-24 counts
still, then 2^-12 .. 2^-8 cycle per field (2^-9 .. 2^-5 figure per field): the top is capped so a flight still
takes 16 fields. All motion is rigid (slide, turn, and the figure zoom).

## Camera

- Source field (the shell's): luma (minus black, 8 bits) or the Osc lattice -> a 32-pixel box (16 at 720p and SD)
  -> a vertical one-pole in a line store at quarter horizontal resolution.
- Grid: 32 x 32 virtual px blocks (60 x 34). On the lower half of every block row each stored column overwrites its
  block's cell with its LEVEL: 0 below 5/219 of white (black), 1, 2 from 100, 3 from 149. 2 bits, one write
  statement, two banks ping-ponged per field (a field writes one bank and reads the other), zero start-up contents.
- Read: the pixel's own block; fabric register; the level travels with the pixel and is constant across the block.
- What it drives: a flat tone step. Inside the pieces, a block with level 1 .. 3 replaces the piece's grey by K6
  with one of three flat greys (.26 .56 .90 of white); on the ground the two upper levels lay two dark steps
  (.11 .22). So the subject shows as a posterised mosaic cut out by the tans, with a dim echo on the ground.
  Level 0 (black) keeps the piece grey, so a black input or K6 = 0 is exactly the clean figure.
- 25 %: a lean (the subject faintly in the pieces); 50 %: clearly there; 100 %: bear.png is recognisable inside
  the square (ears, eyes, muzzle, nose). Ink: blocks of level 2 .. 3 invert (a solarised subject).
  Fine: the subject at half strength inside the outlines.
- Osc: the shell's drifting 640 x 540 triangle lattice in place of the luma, through the same levels (diamonds of
  three greys stepping slowly across the frame).

## Engine

- **Per field** (vblank zone, "no avid for 1024 clocks", restarted from address 0 at every zone start, so a
  stalled run cannot outlive a field): a microcoded engine after Vellum cassini / Hardline flute (tans.py holds the
  1018-word program and its emulator; 59 registers in a 64 x 32 block-RAM register file initialised to the TOML
  defaults; 38-bit accumulator, one adder, a radix-4 Booth serial multiplier (9 steps) and a right shift in 2-bit
  steps, its adder operand prepared one clock ahead; auto-incrementing table reads; a hardware sine unit
  (quarter-sine ROM, interpolated, sine then cosine)): knob holds and
  glides, K4 hysteresis, the Drift speed and phase, the ease, the figure scale, Scale, Scatter, then per piece the
  pose between the two figures, its centroid in pixel widths, sin / cos of its angle and of the angle + 45 deg, and
  per edge the pixel step, line wrap and field seed written into the edge table (opposite edges of the square and
  the parallelogram reuse their partner's products). About 6,800 clocks (7,400 with Scatter and Mirror),
  measured in the emulator; the fake vblank of the 720p sim is 9,700, real NTSC about 17,800.
- **Per line**: no multiplier. Each edge is one 26-bit DDA (Q12 pixel widths) with its own increment register: it
  adds the pixel step on every clock of the line window (W + 8 steps), then once more with the wrap parked in that
  register from the edge table (46 reads per line through one fabric copy of the table word; the loader of
  Hardline dazzle / Straightedge sliver).
- **Per pixel** (15 stages, no wide multiplier): clamp each edge to 0 .. 63 sixteenths -> per piece the minimum of
  its edges -> Bold / Fine coverage -> grey term (shift-adds) -> brightest and sum of the seven (three stages) ->
  film, Ink triangle, camera level tones (three small multiplies) -> tone -> the mirror's line buffer -> luma.
- **Output**: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (26) in every mode, virtual 1080-line coordinates from the timing ID
  (interlaced modes use a 2-pixel edge ramp).

Chassis: Kinetic riley by way of Straightedge sliver: [A] timing / zone / width / generated avid, the [D] source
field (Osc lattice with dazzle's line-0 fix, box, one-pole) and the luma half of [F]; the per-field engine from
Vellum cassini. No wave engine, palettes or chroma paths.

## Status

- v0.1.0 built 2026-11-22 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`), every
  config on **seed 1**. Not flashed.
- Post-route Fmax: HD Analog 90.35 MHz, HD HDMI 89.82, HD Dual 83.20 (HD needs 74.25); SD Analog 78.67,
  SD HDMI 83.42, SD Dual 91.17 (SD needs 27).
- 6,691 .. 6,710 / 7,680 LC (87 %), 20 / 32 EBR (microprogram 4, table ROM 4, register file 2, edge table 2,
  grid 2, line store 2, mirror buffer 2, sine 1, box 1). All six bitstreams pass iceunpack. dupin: no carry
  self-adds, no duplicated LUT inputs. No `emulate_transparency` in the standalone synthesis.
- Router2 is NOT usable on this netlist: on an earlier (larger) version it stalled on SD HDMI and HD Dual seed 1
  and missed HD Dual at 68 MHz; router1 closed every config on seed 1.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge tans`.
- Timing closure history: the first build missed HD (68 MHz): the edge table's read data fanned out from the block
  RAM to all 23 DDAs across the die (now one fabric copy), then the engine's execute step had four LUT levels of
  Booth selection before its 38-bit carry chain (now the operand is prepared one clock ahead). A first router1
  build then failed placement on SD HDMI at 6,973 LC, so the edge clamp went from 7 to 6 bits, the Bold coverage
  sums moved one stage earlier, Fine got one outline grey and the sine unit shares its interpolation registers
  (6,700 LC).
- **Model = RTL** (GHDL against the integer model `.straightedge_work/rb_tans/imodel.py`, which runs tans.py's
  microprogram through its emulator; the GHDL frame is the model's shifted 4 columns right with 4 blank columns at
  the left, the shell's generated-avid start). Full 720p60 frames (decimation 1, 918,720 px each), all 0 mismatches:
  - on the built sources: the defaults with a black input and Drift still; the power-up defaults with Glide On and
    Drift 650 (= rtl_default.png); Fine + Mirror + Scatter 68 % + Cat + bear.png; Ink + Osc + X 300 + Scale 700 +
    Fish + Camera 60 %; Source = Osc; bear.png at K6 100 %.
  - on an earlier RTL version (before the last fit cuts, which the frames above verify): motion at Drift -100 %
    (captures after 2 and 4 warm-up frames equal the model at exactly 3 and 5 fields; one field either side
    differs by 113k - 155k pixels, so speed and direction agree).
  - NTSC (720 x 243 per field, bear.png): 0 of 173,988 on each field, with the camera grid from the OTHER field
    (the one-field-old bank ping-pong). This needed a SIM-ONLY copy (`.straightedge_work/rb_tans/simsd/`): the image
    tester's fake SD vblank is 8 lines x 784 clocks, shorter than the engine's 6,800 clocks, so the copy starts the
    zone after 128 idle clocks and skips the knob holds / K4 hysteresis / Drift (no-ops at those settings). The
    unmodified program cannot be simulated at SD; on real SD timings the vblank holds about 18,000 clocks.
  - Not rendered: 1080p, 1080i, 576i, PAL, a glide in flight, Scatter with Drift moving.
- Renders: `.straightedge_work/rb_tans/cmp_final.png` (model default | RTL black | RTL Osc | RTL bear), `sheet.png`,
  `strip.png` (P12 travel), `motion.png`, `camera.png` (K6 = 25 / 50 / 100 %), all from the integer model except the
  RTL panels of cmp_final; `rtl_default.png` (GHDL, 720p60 dec 1, every control at its power-up default).

## Cut / deviations

- No mirror bit for the parallelogram (the spec's ROM has one): a flip is not a rigid motion, so all seven
  silhouettes were composed with the parallelogram in its native chirality.
- Figures: Square, Cat, House, Boat, Swan, Fish, Arrow, Pyramid (the spec's examples included runner and candle:
  the runner did not come out readable without corner-only contacts, the candle was 9 units tall and would have
  shrunk every piece to half size).
- Camera: the level REPLACES the piece grey by K6 (the spec: the level steps the tone); with the seven base greys
  under the steps bear.png was not recognisable. Black stays the piece grey, so black input = the clean figure.
- Scatter interpolates toward a designed slot per piece while keeping 20 % of the figure (the spec: an additive
  push), so that 100 % is a composition on the screen and Drift still moves everything.
- Mirror moves the figure into the right half at 0.72 scale (the fold alone would mirror half of a centred figure).
- K1 / K2 move the figure by up to half a screen, so at the ends half of it leaves the screen.
- Osc runs through the same three levels (not 2 - 4x a gain: the camera here is a level replacement, not a gain).
- Wide figures (Arrow 1,490, Pyramid 1,770 virtual px at their default scale) are cropped at SD, which shows the
  centre 1,440 virtual px of the 16:9 canvas (the same picture otherwise).
- Drift's top speed is 2^-8 cycle per field (the set's 2^-2 would flip figures every couple of fields).
- Fine draws every outline in one light grey (cut for fit: seven per-piece outline greys cost about 50 LC); the
  1080p outline is 47/16 = 2.94 px instead of 3 px (the 6-bit edge clamp needs it).
- Per-piece flight stagger cut (engine time); all pieces share one ease.

## Not sure of on hardware

- Nothing on this chassis has been flashed. The engine is cassini's with a new radix-4 multiplier, shift steps,
  ROM auto-increment and a sine unit: sims model every one of them (the emulator and the RTL agree pixel for pixel),
  but none has run on hardware.
- The engine needs about 6,800 - 7,400 clocks per vblank: fine on every real timing (NTSC has about 17,800 after
  the zone delay).
- Moving sharp edges are the HD HDMI bar trigger; the [1 2 1] + 13/16 conditioning is built in.
