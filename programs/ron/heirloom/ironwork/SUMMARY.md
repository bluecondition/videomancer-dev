# Ironwork — magenta, green and black contour lace

**v0.1.1 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the still
`photos/vid/WIN_20230717_23_51_09_t7p5.png` (clip WIN_20230717_23_51_09 at 7.5 s; same patch: t0.5, t10.5 and
WIN_20230717_23_51_33 t4.5). Two tiles of mirrored "ironwork" across the frame: bold green hearts, X crossings and
nested diamond eyes on black in two lattice rows, fine bent stripes between them (lime lines on black above, salmon /
red / magenta bands below), pink gothic arches at the bottom, all torn by line-timing slips. Presets are hand-written
placeholders.

v0.1.1 (review fix): the tears are clean shifts of whole bands of the picture (no per-line jitter, no checker cells),
2-4 bands covering ~17 % of the lines, counted in virtual lines so 720p and SD tear like 1080p.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_ironwork/` (`notes.md`, `meas*.py`, `fit*.py`, `proto7.py`, `imodel.py`).
- **Tiles.** Horizontal period 944 px (autocorrelation 0.77–0.93 in every band). Mirror axes every 472 px: the
  lens / heart columns at x = 524 and 1466, the diamond-eye columns at x = 52 and 992.
- **Rows.** Vertical mirror axes at y ≈ 252, 497–516, 776 and 1006: rows of 515 lines, troughs at y = 497 and 1012,
  lattice rows at 240 and 755. They stay put over the whole clip (10 fps track: 252 / 516 / 776 for 6 s).
- **Field.** Every zone edge is a contour of one offset-circle field over the folded triangles,
  r = sqrt((u + 236)² + (1.8 v)²), u = |x − 524| folded at 944, v = |y − 497| folded at 515
  (`fit4.py`: d0 and the 1.8 aspect maximise the green-on-contour KL score). Read in rings of 205 px. Around the
  A axes the circles cross into lenses, on the B axes into X crossings, at the B lattice corners into diamond eyes,
  below the lattice rows into gothic arches.
- **Two colourisers.** The family-by-ring-phase table (`fit6.py`) shows two zone types per ring: X (ring phase
  0.10–0.66) and Y (the rest), each with its own colour per band: top X lime-on-black / Y green; mid X salmon-red /
  Y lime-on-black; low lattice X salmon-magenta-violet / Y green-cyan; bottom X salmon / Y dark. That is a vertical
  colour score (two offsets per line), fitted per 60 lines.
- **Stripes.** Period 37.5 px (FFT 36–39 in every band), bent like ")": relative to y 520 the stripes at y 700 sit
  −24 px at x 300 and −137 px at x 1700, i.e. a frequency modulation of about ±7.5 % (vertical sine, period ~1030
  lines) about an origin left of the frame (x0 ≈ −200 px). The stripes are a KEY (black gaps) whose threshold depends
  on the colour: solid green, salmon 12 % black, lime 30–65 % black (thin lime lines). They also push into the ring
  phase by 0.35 ring × (1 − V), which gives the lace fingers along the zone edges while the lattice rows stay clean.
- **Palette.** k-means 14: black (14, 21, 11) 23 %, salmon (240, 117, 128) 11 %, green (4, 241, 14) 10 %, coral
  (235, 79, 87) 9 %, red (186, 23, 37) 8 %, dark red 6 %, dark green 5 %, magenta (180, 23, 138) 5 %, lime
  (126, 210, 41) 5 %, cyan (11, 181, 211) 4 %, pale green 4 %, violet (185, 52, 227) 3 %, blue (38, 26, 157) 2 %.
- **Tears.** A few clean horizontal shifts of whole bands: rows 0–60 shifted −50 / +45 px; rows 740–830 sheared
  +28 → −30 px; rows 830–1020 shifted +25 px. The lace and rings are intact inside each band.
- **Texture.** Soft horizontal edges (capture blur ~5 px), luma p99 178.

## Motion in the clip
- The geometry is locked to sync; what moves is colour and the stripes.
- **Colour cycle.** The colour families roll through the zones: the top band goes pink 12 % (5.0 s) → 52 % (6.4 s)
  → 13 % (7.5 s) → 70 % (9.0 s) → 8 % (10.5 s), about 2.7 s per cycle (0.37 cycles/s), slightly uneven (Ron's hand
  on the colouriser).
- **Stripe sway.** Frame to frame (34 ms) the stripes shift 1–9 px, 2.3× more on the right half than the left (a
  frequency change, not a slide), sinusoidal in time with a phase that runs down the frame: the bend's vertical sine
  travels, period ~2.8 s.
- **Tears** jump every ~0.5 s. The other stills of the patch show the stripe level and the colouriser offset turned:
  t0.5 (pink arches on top, wide stripes), t10.5 (a green lattice band), 23_51_33 t4.5 (no stripes: clean black
  lenses with green rims and red-magenta-blue crescents).
- **In the program:** K4 Cycle at 700 runs the colour cycle at 0.35 cycles/s and the sway at a 2.9 s period, starting
  from the still's phase at power-up; the tear layout is redrawn every 8–39 fields. The v0.1.0 RTL frame at
  K4 = 1023 two frames later matched the integer model's field 3 (77 % of pixels exact).

## What it does
1. **Beam.** A tile-phase DDA (Q24, K1) folds to u; the line engine gives the row phase v (K2).
2. **Ring field.** X(u) = ((u + d0) / Pr)² sits in a 1024 × 20 block-RAM table refilled every field by a second-order
   DDA (Q12.24). Per line: Y = (γ V)². Per pixel: s = X + Y, normalised by even shifts, square root by a 1024-entry ROM,
   giving the ring phase (Q12).
3. **Lace.** A stripe-phase DDA (frequency × the travelling sway m(y) = 1 − 0.075 cos) makes the triangle S. It adds
   As (1 − V)(S − ½) to the ring phase, nudges the colour by 0.2 (S − ½), and keys: black where S < THR[colour] + lift,
   a half step at the edge.
4. **Colourisers.** Type X or Y by the ring phase; colour index = the line's score (hX or hY, which already holds the
   cycle) ± the slope across the zone; 256-entry palette and threshold ROMs.
5. **Video.** Camera luma (or the Osc) times the Etch gain is added to the ring phase and the stripe phase.
6. **Tear.** Counted in virtual lines: the first band starts 0–255 lines into the field, each next one 160–415 lines
   after the previous ends; a band is 24–87 lines of the same picture shifted 20–146 virtual px (the line's tile and
   stripe start values), straight or sheared smoothly by 1 h per virtual line. The random numbers are drawn only at
   band starts, so the layout in virtual lines is identical at every resolution (2–4 bands, ~17 % of the lines).
7. **Output.** U and V through [1 2 2 2 2 2 2 2 1]/16, Y [1 2 1]/4 on the same centre, luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the lens / eye columns: tile width ×3 (0, ~0.7 tiles across) .. the still's 944 px (512, two lenses) .. ×1/3 (1023, six) |
| K2 Rows | the lattice rows: row height ×3 (0, one row of tall lenses) .. 515 lines (512) .. ×1/3 (1023, stacked bands) |
| K3 Lens | the lens offset d0: concentric circles / rose rings (0) .. the still's lenses, X crossings and diamond eyes (512) .. long flat arches and near-straight bands (1023) |
| K4 Cycle | bipolar (±8 deadband, square law): the colour cycle through the zones and the stripes' travelling sway; 700 = the clip (0.35 cycles/s, sway 2.9 s); 1023 = 2.7 cycles/s; left runs it backwards; centre freezes the colours |
| K5 Rings | ring density: ×1/3 (0, a few broad zones) .. 205 px rings (512) .. ×3 (1023, dense moiré) |
| K6 Etch | how far the camera (or the Osc) bends the rings and the lace: 0 .. 3.4 rings per full luma (square law); 400 = 0.53 ring |
| S7 Tear | On (the still's line-timing slips: 2–4 bands of the picture shifted sideways, ~17 % of the lines) / Off (clean ironwork) |
| S8 Palette | Ironwork (the still: green, salmon, magenta, cyan on black) / Verdigris (aged copper, gold and teal) |
| S9 Weave | Lace (vertical stripes) / Mesh (crossed with horizontal stripes of the same pitch: a woven grid) |
| S10 Key | Lace / Negative (the stripe key inverted: dark ironwork with coloured slots) |
| S11 Source | Video / Osc |
| P12 Lace | the stripe oscillator: 0 % = broad calm bars (150 px) over clean rings; 512 = the still (37.5 px, 0.35-ring push); 100 % = fine filigree (19 px) pushing the rings 0.8 ring and lifting the key's black |

## Video
- **Video (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, times the Etch gain is added to the ring
  phase and the stripe phase: the lenses and lace bend into contour lines around the picture, the way a camera patched
  into the ramp sum would. With no input, Video mode is the still.
- **Osc.** A 640 × 540 virtual-px 2-D triangle lattice, drifting, feeds the same path: at the default Etch the rings
  wobble by up to half a ring and the lace ripples.

## Engine
- **Per-field sequencer (vblank zone).** One serial shift-add multiplier (24 × 48), 14 slots, slot number decoded two
  clocks ahead, operands pre-muxed. The five exponential controls share one barrel on the operand
  ((C << (sh + 4)) × mantissa, every result = acc >> 19). Shared glide for K1, K2, K3, K5, K6 and P12. Per field: tile
  and row rates, the line step and row phase of line 0, α = table step in rings, β = d0 / Pr, γ, α², αβ, β², the stripe
  rate, speed², the etch gain, the lace depth; the colour cycle and sway advance (both power up at the still's
  phase, 0); the tear seed hold (8–39 fields).
- **Ring table fill** (zone clocks 480–1503): X(i + 1) = X(i) + D, D += 2α² (no multiplier), 1024 × 20, `no_rw_check`.
- **Per-line engine** (kicked at each line start, computes the NEXT line in ~185 clocks, fits SD's 858): the row
  fold V, the sway cosine, the tear (Q8 virtual-line counters stepped by the line's virtual step), (γV)², the stripe
  rate × m(y), the stripe and tile start values (with the tear), the colour score (64-entry table, interpolated) plus
  the cycle, the lace depth × (1 − V), the mesh stripe.
- **Shared ROM** (512 × 16): exp2 for the zone, cos and the colour score for the line engine (one port, OR-ed
  addresses; the zone and the line engine never overlap).
- **Pixel path (21 stages + 4 output).** Table address, table (EBR), X + Y, normalise, sqrt address / stripe value,
  sqrt ROM (EBR), fabric copy, phase + lace partials, + offset, + lace − type start, type and slopes, colour index,
  threshold address, threshold ROM, fabric copy, palette address + lift, palette ROM + key distance, fabric copy + key
  level, compose; then the chroma box / luma [1 2 1], luma 13/16, gate. C_LAT 24 in every mode.
- **Block RAM rings** on one write pointer: syncs (22 back + 2 registers), chroma (the box's 8-back samples, 8-bit),
  stripe value (stage 12 → 18).
- **ROMs / RAMs:** sqrt 1024 × 13, palettes 512 × 32 (two palettes), thresholds 512 × 8, shared 512 × 16, ring table
  1024 × 20, line buffer 2048 × 8, three rings: 22 EBR.
- **Contracts:** damier's generated avid; one latency (24) in every mode; the HD HDMI conditioning (chroma
  [1 2 2 2 2 2 2 2 1]/16 with the running-sum resync to the palette black, luma [1 2 1], luma 13/16) before the
  blanking gate; virtual 1080-line coordinates (SD shows the centre 4:3); palettes in the hardware convention
  (U = Cr), luma pre-scaled 16/13 and capped at 940.
- **Verified (GHDL, 720p60, decimation 1).** v0.1.1 default frame vs `imodel.py` at the same tear seed: 73 % of
  pixels exact, 91 % within 8 levels (the rest are 1–2 px zone-edge contours); three tear bands at 720p lines 38–83,
  225–271 and 508–550 (18.5 %), each a clean shift. Model vs still at 1080: colour-family shares dark 31 / 27 %,
  lime 14 / 11, green 13 / 12, cyan 4 / 4, blue 3 / 3, magenta 5 / 5, salmon 20 / 17, red 10 / 16 (model / still);
  4 × 4 region-mean RGB within 10 levels on average.

## Status
- **v0.1.1 built 2026-11-04 with ROUTER1, all six configs on seed 1, NOT flashed.**
  - Post-route Fmax: HD Analog 93.43, HD HDMI 92.45, HD Dual 95.37, SD Analog 91.61, SD HDMI 84.81, SD Dual 88.10 MHz.
  - 6802–6827 / 7680 LC (89 %), 22 / 32 EBR. iceunpack OK × 6; the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/ironwork.vmprog`.
  - Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh ironwork`.
- **v0.1.0** (same day): the first tear (per-line ±2 px interlace jitter against the 37.5 px stripe key, many short
  bands, counted in output lines) rendered as checker cells; replaced in v0.1.1. 6644–6675 LC, HD 86–94 MHz.
- **router2 stalled** on HD Analog (overused = 4 for 3700+ iterations) on a v0.1.0 netlist; stay on router1.
  dupin: 0 `CARRY I0==I1`. Standalone yosys: one transparency emulation (the line buffer, as in lozenge).
- **Fit history.** The first cut (C_LAT 28, two 9-tap boxes on Y/U/V, five exp barrels, FF sync chains) packed to
  8027 LC (104 %). Rings in block RAM, one chroma box, the shared barrel and the merged ROM brought it to 6.7–6.8 k.
- **Watch on hardware:** no pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- The lattice shapes come from one offset-circle field, so the top row's hearts read as rounded X crossings and
  lenses; the still's extra nested cyan / blue rims inside the diamond eyes are single, thinner rims.
- The stripe bend is one travelling vertical sine (±7.5 %) plus the lace push; the still's stripes bulge a little
  more around the lens edges.
- Red is under-represented (10 % vs 16 %) and salmon over (20 % vs 17 %); the bottom band's violet patches are rare.
- Tears are random bands in the still's style and amount, not its exact pattern; the layout is redrawn every
  8–39 fields.
- At the default Cycle the colours roll as in the clip: the still's palette shows at power-up and comes round every
  ~2.9 s (K4 at the centre freezes it).
- The clip's colour cycle was uneven (Ron's hand); the program's is steady.
- Cut for budget: the threshold lift at P12 > 50 % is additive (not proportional); chroma runs at 8 bits inside the
  box (2 LSBs); the second 9-tap box of the older shells is gone (the contract's [1 2 1] luma / 9-tap chroma remain).
- **Resolutions.** 720p shows the same geometry and the same tear layout as 1080p. SD shows the centre 4:3 (about 1.5
  tiles instead of 2). Whites come out at ~777 (the shell's white cap).
