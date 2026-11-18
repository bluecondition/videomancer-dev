# Tines (Straightedge)

**v0.1.1 built 2026-11-17, not flashed.** Flash `out/rev_b/ron/straightedge/tines.vmprog`.

## What it draws

Rows of long needle triangles: one row hangs from the top edge, one stands on the bottom edge, half a pitch
apart, so the two interleave. After the points of a backgammon board, saw blades, the teeth of a zip and Bridget
Riley's black and white triangles ("Straight Curve"). Every edge on the screen is a straight line; luma only
(U = V = 512 on every pixel).

    cell coordinate  phi = x / P - offset - drift                       (one wrapping DDA per row)
    tooth            eL < frac(phi) < 1 - eR,   eL = a t,  eR = (1 - a) t,   t = depth / reach
                     a = where the apex sits on the base (Skew); t >= 1 is past the tip
    coverage         per edge: the signed distance in cycles -> pixels of distance (a per-field shift and a
                     3-bit mantissa from the edge function's gradient) -> a soft step F (0 .. 16);
                     tooth = clamp(FL + FR + 16 (S - 1)), S = which neighbouring teeth's edges the pixel sees:
                     a needle's tip is its width in sub-pixels, a notch is 16 minus the gap
    Grey             the rows are additive films: top 3/8, bottom 5/8 of white, so ground, each row and the
                     overlap are black, dark grey, light grey, white
    Ink              the rows XOR (a triangle of the summed coverage)

At the default (Reach 78 %, 11 teeth across) the two rows leave a black zigzag band between them; as they
slide against each other the band closes into chains of white lozenges (the overlap) and black ones. At Reach
100 % with the rows half a pitch apart the two films tile the screen exactly (their shared edges add up to
exactly one pixel: the right edge of every tooth runs on the negated distance, so one row's floor is the
other's ceiling).

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | slides the top row (Cross: and the left row) against the others, +-3/8 pitch |
| K2 | Y | the balance: the top row's reach x (1 - 7/8 y), the bottom row's x (1 + 7/8 y) (the line where the tips meet moves up or down; Cross: the left / right rows the same) |
| K3 | Scale | the pitch, 32 .. 479 virtual px (60 .. 4 teeth across), exponential (default 174: 11 teeth) |
| K4 | Skew | where the apex sits on its base: 0 (a right-angle saw tooth) .. 1/2 (centre, held +-12 counts) .. 1; the bottom and right rows lean the opposite way |
| K5 | Drift | the rows slide at 1, -3/4, 21/32, -1/2 of the speed (top, bottom, left, right): +-24 counts still, else 2^-12 .. 2^-4 pitch per field, exponential (default 700 = 396 / 2^18 pitch per field: 11 s per pitch at 60 Hz) |
| K6 | Camera | the gain in front of the two films' thresholds (0.6 at 25 %, 0.75 at 50 %, 1 at 100 %; default 100 %) |
| S7 | Glide | On (default): X, Y, Scale, Skew, Camera, Reach ease 1/16 of the gap per field, never slower than one count, landing exactly; the Drift speed eases 1/8. Off: no easing at all. A +-1-count hold on every knob (the ends always get through) |
| S8 | Rows | Jaws (top and bottom) / Cross: a row of needles from the left edge and one from the right (reach x 16/9, the same pitch), crossing the vertical ones. Grey films 4, 6, 2, 4 sixteenths |
| S9 | Tone | Grey (films) / Ink (black and white, the rows XOR) |
| S10 | Weight | Bold (filled teeth) / Fine: every tooth as an outline drawn inside its two edges (P / 20, 2.5 .. 6.5 virtual px), so the tips stay sharp; Grey films 10/16 and 16/16 |
| S11 | Source | Video / Osc |
| P12 | Reach | the headline. 0 % = teeth 67 virtual px long (a serrated frame round a black field); 50 % = the tips meet along the middle; 100 % = every needle reaches the opposite edge. Default 78 % |

## Motion

Each row has its own drift phase (Q18 of one pitch) in the field engine's register file, advanced once per
field at the vblank zone start ("no avid for 1024 clocks": once per field, robust to serrated vsync) and wrapping
exactly one pitch, so nothing ever jumps. Default 700: the top row slides right 0.0015 pitch per field, the
bottom row left at 3/4 of that. Drift 0 / 1023: -+ 0.057 pitch per field (the top is capped at 2^-4: a
quarter pitch per field strobes a row of identical teeth).

## Camera

The camera is never shown. Its luma (minus black) goes through a 32-sample box and a vertical one-pole and is
sampled at the centre of every block of a 32 x 32 virtual-px grid (60 x 32 blocks; the first and last rows
are 60 lines tall) into 2 bits per block: above the light film's threshold, above the dark film's. Each
flag has a hysteresis of +-2 codes (of 8-bit luma) against the block's own flag of the field before, so
camera noise and the two fields of an interlaced source cannot make a block blink. Block RAM, two banks
swapped every field, written in one field and read in the next (a second copy gives the write side the
flags of the field before).

Each tooth EDGE, per block row, looks up the block at its own quarter point (a quarter pitch either side of
the cell centre; side rows: per block column, a quarter pitch above / below the strip centre). Where its
film's bit is set, the edge steps out: it keeps half of its inset beyond 1/16 of the cell (e = 1/32 + e / 2;
insets under 1/16 cell are left alone). So a flagged tooth segment is fatter and the tooth 1.875 x longer, it
keeps half its taper inside the block, and a clear line of at least 1/8 pitch (4 virtual px at the finest
Scale) always stays between neighbouring teeth of a row: the zigzag of both rows reads over the whole frame at
any luma. Only straight block borders are added (the edge jumps where the flag changes). Three readings:
shadows = the clean figure; mid tones (over 0.44 of white at 100 %) = the bottom / right rows fat;
highlights (over 0.70) = both rows fat (white where they overlap, the rows' own greys in the lines between).
K6 scales the luma in front of the thresholds:

- 25 %: only near-white areas (over 0.73) fatten the light rows;
- 50 %: over 0.58 fattens the light rows, over 0.93 the dark ones;
- 100 % (default): the subject reads as thinner teeth (the clean black zigzag) in fatter ones.

Black input or K6 = 0 is exactly the clean figure. Source = Osc puts the shell's drifting 640 x 540 triangle
lattice in at the same point at 1x: 36 % of the frame clean, 44 % light rows fat, 20 % both (at 2x, 78 % of
the frame is over both thresholds: one flat state, so 1x is kept).

## Engine

- **Field engine** (per field, in the vblank zone): a microcoded accumulator machine (32-bit accumulator, one
  adder, a 256 x 32 register file, a 1024-word program and a 512-word constant ROM in block RAM, one serial
  32 x 16 multiplier, a 45-step divider, a normaliser). `tines.py` assembles the program; the integer model
  runs the same words. It does the knob hold and glide (7 channels), the Drift speed and the four phases, the
  pitch (2^(i/256) ROM), the reach (1 / reach by the divider), the edges' insets per line / per pixel and
  their start values, the rows' phases at pixel 0 / line 0, the first camera anchors, the eight edges'
  normalisers, the Fine line width and the camera thresholds. About 4,300 clocks.
- **Line engine** (31 clocks after every line): reads 13 words from the register file and steps the top /
  bottom insets one line, the side rows' strip phase and anchors, loads the stage-1 DDAs and the first anchors.
  The camera grid is read on one port, one anchor a clock (eight slots).
- **Per pixel** (stages 1 .. 10, no multiplier): stage 1 the cell-phase DDAs (top / bottom) and inset DDAs
  (left / right); 2 the edge distances; 3 the pixel windows (shift, saturate); 4 x the 3-bit mantissa (two
  adds); 5 the soft steps; 6 tooth (Fine: minus the inner tooth); 7, 8 the films' sum or the XOR; 9, 10 luma.
- **Output**: luma [1 2 1] / 4, 13/16 toward black (white 775), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (16) in every mode, virtual 1080-line coordinates from the timing ID (SD
  shows the centre 4:3: in Cross the side rows' bases are off-screen there).

## Status

- v0.1.1 built 2026-11-17 through the shared queue with **router1** (`NEXTPNR_ROUTER="--router router1"`),
  **every config on seed 1**. Not flashed. (v0.1.0, the same day, differed only in the camera law.)
- Post-route Fmax: HD Analog 88.04 MHz, HD HDMI 80.30, HD Dual 93.31 (HD needs 74.25); SD Analog 82.84,
  SD HDMI 82.39, SD Dual 83.03.
- 7,004 .. 7,052 / 7,680 LC (91.2 .. 91.8 %), 17 / 32 EBR (program 4, register file 2, constants 2, grid 2 + 2,
  box 1, line store 4). Six bitstreams pass iceunpack. No `emulate_transparency`, no carry self-adds (dupin: 0).
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge tines`.
- The first RTL packed at 7,863 LC (102 %). What brought it down with the same picture: the line engine's
  constants read from the register file after every line instead of 300 flip-flops of engine outputs; the
  insets at Q5.20; the stage-2 / stage-3 and tooth / coverage stages merged (latency 20 -> 16); the camera gain
  moved from a multiplier into the thresholds; the edge window as a two-level shift.
- **Model = RTL** (GHDL, the built v0.1.1 sources), every frame 0 mismatching pixels: 720p60 decimation 1 at
  the defaults with a black input, with Source = Osc, and with bear.png as the camera at K6 100 % (918,720
  pixels each); NTSC with the bear and Cross, both fields (347,976; each field's grid written by the other
  field); 720p / 2 with Cross + Fine + X, Y, Scale, Skew, Reach off their defaults + the bear. Chroma 512 on
  every pixel of every dump. The GHDL frame is the model's shifted 4 columns right with 4 blank columns at
  the left (the shell's generated-avid start). With a black input the v0.1.1 model is identical to v0.1.0's.
- Checked on v0.1.0 (the field engine, the Drift and the glides are unchanged in v0.1.1), all 0 pixels: both
  fields of an NTSC frame at the defaults; Cross + Ink + Fine + Osc + other knob values; and Motion / Glide
  On: with S7 On, Scale turned to 800 and the default Drift, RTL fields 4 and 10 both match the model (the
  Scale glide has taken 4 and 10 steps of 1/16, the four Drift phases 4 and 10 steps, the top row to the
  right and the bottom row to the left: `motion_rtl.png`).
- NTSC shows the same picture (the centre 4:3 of the 16:9 one: `sim/cmp_ntsc.png`).
- Renders in `.straightedge_work/rb_tines/`: cmp_final.png (model | RTL default | RTL Osc | RTL bear), sheet.png,
  strip.png (P12 travel), motion.png, camera.png (all from the integer model `imodel.py` except the RTL panels).

## Not sure of on hardware

- Nothing on this chassis has been flashed. The generated avid, zone and width logic are the heirloom shell's.
- The camera thresholds (0.44 / 0.70 of white at K6 100 %) were set on bear.png; a real camera's levels may
  want K6 somewhere else. The pattern is all sharp full-swing luma edges in motion: the [1 2 1] + 13/16
  conditioning is built in, but Ink with Scale at minimum is the first place to look for HD HDMI bars.
- The grid read needs a tooth or block of at least 13 clocks (the shortest is 16 pixels, at SD with Scale 0 %)
  and the line pass needs 45 clocks of horizontal blanking (the shortest real one is 138).
- The vblank zone is "no avid for 1024 clocks" (the heirloom shell's value; 1080p24's hblank is 830).

## Cut / deviations

- **Camera target: the tooth segment's width step, not "lengthen the tooth".** One luma sample per tooth
  (the spec's first choice) gives clean needles of different lengths but only a column profile, not a picture.
  Terraces by block row with a continuous reach read as noise at the default pitch (slivers and misaligned
  steps of two films). v0.1.0 let a flagged edge go all the way to its cell border (a full block): that
  erased the figure wherever the picture was bright. v0.1.1 caps the step-out (above): every tooth stays a
  tapered tooth with a clear line to its neighbour. The subject now reads as fatter and thinner teeth, not as
  a block poster; at Reach 0 % the camera only lengthens the short teeth (to 1.9 x), so it is weak there.
- **Osc is fed at 1x**, not 2-4x the camera level: at 2x most of the frame sits over both thresholds.
- **No rows behind the main pair** (the spec's optional finer back rows): the budget went to Cross and Fine.
- **Drift tops out at 2^-4 pitch per field** (the brief's 2^-2 strobes); the left row runs at 21/32 (the
  spec's 2/3).
- **X is +-3/8 pitch**: +-1/2 or more shows the same picture at both ends.
- **Reach never goes under 67.5 virtual px** (1/16 of the height; the insets' word length), so Y at the far
  end with Reach 0 % only lengthens one row.
- **The tip of an obtuse tooth** (short reach, wide pitch) is about a third of a pixel short: the two-edge
  coverage formula is exact for needles and a slight under-estimate for wide angles.
- **Camera block borders are not anti-aliased**: block-row borders (top / bottom rows) sit on line boundaries,
  block-column borders (side rows) on pixel boundaries. The stepped-out edges themselves are tooth edges:
  anti-aliased, and they move with the Drift like any other.
- **Edges are normalised by max + 3/8 min** of the edge function's per-pixel and per-line steps and a 3-bit
  mantissa: the ramp is 0.9 .. 1.1 pixels wide instead of exactly 1.
- Interlaced modes render each field at the field's own lines (field_n is tied high by the core).
