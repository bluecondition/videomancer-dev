# Hatch (Straightedge)

**v0.1.0 built 2026-11-22, not flashed.** Flash `out/rev_b/ron/straightedge/hatch.vmprog`.

## What it draws

Sol LeWitt's "Lines in Four Directions" and the wall drawings that lay out every combination of vertical,
horizontal and the two diagonal lines, superimposed; the engraver's cross-hatch tone scale. The screen is ruled
into square panels. Four families of crisp parallel white lines run continuously across the whole screen
(vertical, horizontal, x + y and x - y, the diagonals passing through every vertical / horizontal crossing, so all
four together make one exact eight-pointed "Union Jack" lattice). Each panel is only a window that switches some
of the families on: one of LeWitt's 15 combinations, laid in his order along the panels ((i + 4 j) mod 15:
V, H, D1, D2, VH, VD1, VD2, HD1, HD2, D1D2, VHD1, VHD2, VD1D2, HD1D2, all four). Tone comes only from how many
families are superimposed (union: a crossing is never brighter), in four even steps (25 / 44 / 58 / 68 % in
Bold). Luma only (U = V = 512).

Default frame: 216-virtual-px panels (five full rows at every raster, the centre row holding the four singles
V, H, D1, D2 side by side), 24-virtual-px pitch, Bold, Grey: a calm catalogue of hatch textures in bold tone areas.

## Controls

| Control | Name | What it does |
|---|---|---|
| K1 | X | pan of the panel grid, +-960 virtual px (the lines stay put). Glided |
| K2 | Y | the same, +-540 virtual px. Glided |
| K3 | Scale | line pitch 16 .. 128 virtual px, exponential; default 200 = 24 px. Glided |
| K4 | Panel | panel size 122 .. 1953 virtual px, exponential: from about 16 x 9 panels to one panel filling the screen; default 211 = 216 px. Glided |
| K5 | Drift | the lattice slides rigidly behind the still panels (V phase rate 1, H -1/2, so D1 = 1/2 and D2 = 3/2 and every crossing stays exact). Hypnos SPEED: still within +-24 counts of the centre, 2^-12 .. 2^-4 period per field either way. Default 680 |
| K6 | Camera | how strongly the held luma of each quarter-panel cell sets its layers; default 100 %. Glided |
| S7 | Glide | Off = this field's knob values, no easing / On = 1/16 of the gap per field (never slower than one count, landing exactly), the Drift speed 1/8 (default On) |
| S8 | Form | Grid (square panels, the catalogue) / Frames (nested square frames about (X, Y), each frame the next combination: V in the centre, then H, D1, D2, VH ...) |
| S9 | Tone | Grey (white lines on black) / Ink (black lines on white: the drawing on paper) |
| S10 | Weight | Bold (lines a quarter of each family's spacing) / Fine (2-pixel hairlines) |
| S11 | Source | Video / Osc |
| P12 | Layers | the headline. Per panel: 0 % = its single direction (a quilt of four textures of equal weight), 50 % = its LeWitt combination (default), 100 % = all four families everywhere: one continuous eight-pointed lattice. Between, each panel moves toward one or four families in proportion, dithered by a 4 x 4 Bayer order (Frames: the bit-reversed ring), so panels change one at a time along the slider. Detent of 8 counts at 50 %. Glided |

A glided knob starts to move only when it is two counts or more from where it rests (pot jitter), or at an end of
its travel. There are no stepped knobs.

## Motion

Two drift phases (20 bits = one period) advance once per field in the vblank zone: V by the speed, H by minus
half of it; D1 = V + H and D2 = V - H are derived exactly, so the four families move as ONE rigid lattice (every
crossing travels, the eight-pointed lattice never breaks) and each family slides across itself at its own rate
(1, -1/2, 1/2, 3/2). Every phase wraps modulo exactly one period. Speed 2^-12 .. 2^-4 period per field (capped
at 1/16: a quarter period per field strobes on lines); the default 680 is about 6.7 s per pitch. Only Drift at
the centre stops it. The panels stay still.

## Camera

The camera never bends a line. Luma -> 32-px box (16 px at 720p and SD) -> vertical one-pole -> sampled once per
32 x 32 virtual-px block into a two-bank block-RAM grid (64 x 64 x 4 bits; the scan writes one bank, the next
field reads the other). The camera cell is a QUARTER panel (each panel split 4 x 4, about 54 virtual px at the
defaults, 35 x 20 cells): the panel grid is the camera grid, subdivided. Each cell reads the block at its centre
(held for the whole cell), maps the 4-bit luma to a layer ladder t = 0 .. 8 (0 .. 4 families in the panel's own
order, then four families thickening to half duty) and moves its level from the panel's own level toward
t + exposure by a = K6 x (0 at black, 1/2 at the first luma step, 1 above). The level is constant across the
cell, so every edge stays straight; the only new edges are cell borders. Black input or K6 0 = exactly the clean
catalogue. P12 stays live under the camera (it shifts the portrait's exposure by +-2 layers). In Ink the ladder
is inverted, so both tones give a positive portrait (Ink: dense black hatching for dark, white paper for light).

- 25 %: a lean (bright regions gain a family).
- 50 %: the subject appears through the catalogue.
- 100 % (default): a cross-hatched portrait; bear.png is recognisable at the default Panel (ears, eyes, nose,
  muzzle), clearer at Panel 0.

Source = Osc substitutes the shell's drifting 640 x 540 triangle lattice x 4 about its middle (nearly two-level):
drifting diamonds of heavy lattice, ringed by blank cells, over the catalogue.

## Engine

- Per field (vblank zone, "no avid for 1024 clocks"): the shared glide (6 knobs + Drift speed), then a sequencer
  of about 70 states, one small operation per clock, on one serial 16-step multiplier and one add / subtract
  loop: pitch (2^(i/256) ROM, interpolated), its 1/sqrt 2, the half-width tables per weight step, cell size, the
  grid origin and its mod-15 starts, the drift, the four seeds (geometric phase mod spacing + drift x spacing +
  half a spacing), the line steps. About 1,100 zone clocks.
- Per line: three line accumulators (H, D1, D2), late-wrap, one add each; the cell grid's vertical state.
- Per pixel: a cell DDA (h units) and an 11-stage cell pipeline (block address, camera word + fabric copy,
  combination ROM, Layers level, camera blend on a 6 x 9 multiplier with its own stage, clamp, mask and half
  widths); three late-wrap family DDAs in pixel widths (Q10, H is constant along a line); the distance from the
  line centre, e = T - |u| on one carry chain, coverage clamp(e + 1/2) over one pixel (two pixels per field line
  for H and the diagonals on interlaced rasters), the union, the luma. No other multiplier, no divider.
- Output: luma [1 2 1]/4, luma 13/16 toward black (white = 777), blanking gate. Generated avid (the shell's
  parity-locked start), one latency (24) in every mode, virtual 1080-line coordinates from the timing ID.
- Chassis: Kinetic riley's [A] timing / zone / width / generated avid, the Hardline / Vellum glide datapath,
  the [D] source field with the dazzle first-line Osc fix, the luma half of [F] (via Straightedge pentagrid).

## Status

- v0.1.0 built 2026-11-22 through the shared queue, **router1**, **every config on seed 1**. Not flashed.
- Post-route Fmax (the builder's last "Max frequency" line): HD Analog 80.91 MHz, HD HDMI 81.95, HD Dual 79.01
  (HD needs 74.25); SD Analog 80.91, SD HDMI 78.55, SD Dual 84.50.
- 6,957 .. 6,978 / 7,680 LC (90.6 .. 90.9 %), 14 / 32 EBR. No emulate_transparency; dupin: 0 self-add carries.
- router2 on the same netlist: HD Analog 74.84, HD HDMI 77.23, HD Dual missed seeds 1-3 and stalled on seed 4
  (stopped); router1 closes every config with margin.
- Rebuild: `NEXTPNR_ROUTER="--router router1" ./build_set.sh straightedge hatch`.
- First RTL 7,428 LC (96 %); cut to 6,976 with the same picture: Q14 -> Q10 DDA widths, a right-shifting
  multiplier, shared adders for the half-width tables and the "step less a period" values, a seed bus into the
  line accumulators, |u| folded into the edge subtract, a shorter cell pipeline.
- Model = RTL (GHDL, the built sources): pixel diff 0 of 918,720 on full 720p60 frames with a black input
  (Drift still, Glide off), with Source = Osc, with bear.png at K6 100 %, and at the full power-up defaults
  (Glide On, Drift 680); 0 of 173,988 on each field of an NTSC frame; 0 on 720p quarter rasters with bear.png,
  with Drift 0 % (fast reverse) and 8 fields after the default frame (exactly 8 more drift steps, same
  direction). The GHDL frame is the model's shifted 4 columns right with 4 blank columns (the generated-avid start).
- Rasters rendered: 720p60 (decimation 1 and 2), NTSC (decimation 1). Not rendered: 1080p / 1080i timings.
- Renders: `.straightedge_work/rb_hatch/rtl_default.png`, `cmp_final.png`, `sheet.png`, `strip.png`, `motion.png`,
  `camera.png` (all from the integer model `imodel.py` except rtl_default and the RTL panels of cmp_final).

## Cut / deviations

- **Diagonal spacing**: the diagonals are spaced P / sqrt 2 (perpendicular), P along the axes, so they pass
  through every V / H crossing and the four families form the exact eight-pointed lattice the spec describes. With
  equal perpendicular spacing (the spec's literal wording) the crossings never coincide and the dense panels are
  an irregular mesh. Bold widths are a quarter of each family's own spacing, so every family has the same tone.
- **Bold is a quarter of the spacing, not a third**: at a third the 3- and 4-family panels went 70-80 % white and
  the default read as white mesh.
- **Camera cell = a quarter panel**, not the panel: at the default panel size a per-panel mosaic is about 9 x 5
  and the bear unreadable. And the camera **blends** each cell's level toward a luma ladder rather than only adding
  families: adding left the catalogue's noise inside the subject and went dead at Layers 100 %. The ladder goes
  past four families into thicker lines so it stays live everywhere.
- **Drift is rigid** (rates 1, -1/2, 1/2, 3/2) instead of the spec's 1, -3/4, 2/3, -1/2: independent rates break
  the eight-pointed lattice into irregular triangles; rigid motion keeps every crossing exact.
- **The 22.5-degree in-between families at the top of P12 were left out**: their slopes cut the exact lattice into
  a quasi-periodic mesh, and eight families at a readable duty would wash the climax white.
- No rotation of the families (the spec allowed leaving them axis-aligned).
- Fine hairlines are 2 PIXELS at every raster (1080p 2, 720p 3, SD 4 virtual px), the thinnest clean line that
  never drops a row; SD and 720p Fine therefore look slightly heavier than 1080p.
- Interlaced rasters (480i, 576i, 1080i) soften H and the diagonals over one field line (the same picture, a
  softer edge). 720p and NTSC were rendered and show the same layout as the model; 1080p is the same by construction
  (virtual coordinates) but was not rendered.

## Not sure of on hardware

- Nothing on this chassis has been flashed. The generated avid, zone and width logic are the heirloom shell's.
- All edges are sharp, full-swing and moving: the [1 2 1] + 13/16 conditioning is built in; if HD HDMI bars
  show, Scale at minimum with Ink at Layers 100 % is the first place to look.
- A cell whose filtered luma sits on a ladder step can flicker between two levels (4-bit grid, no hysteresis).
- The default Drift (680) is gentle (about 3.6 virtual px per second at 1080p); it is clearly moving but slow.
