# Cupola — red pillars rolling up behind two breathing cupolas that cross in yellow X's

**v0.1.1 built 2026-11-09, not flashed.** v0.1.1 lowers the camera's etch gain (1/16, was 1/4) and widens its
luma box (8 px, was 4), so the default Etch bends both X's smoothly instead of fraying them line by line. The Osc
keeps its v0.1.0 strength.

Heirloom (video phase), from the clip `WIN_20230907_00_18_59` at 5.5 s (`photos/vid/WIN_20230907_00_18_59_t5p5.png`).
A red and orange field of tall round-bottomed ovals ("pillars"), a dark-green band, a yellow band of circles crossed
by white arcs, and a green floor. Two big cupola cells cross in yellow X's, with magenta domes under a cream top
strip. In the clip the pillar rows roll up and the cupola breathes: the X opens into a V over an inverted V and closes
again. Presets are hand-written placeholders.

## v0.1.1 (2026-11-09, review fix)
- **Camera fraying.**
  - At v0.1.0's default Etch, the camera (selfie.jpg) frayed the right-hand X into ragged per-line jaggies. On
    hardware with a live camera that would read as tearing.
  - The camera's etch term is now 1/16 of the etch product (was 1/4), and its luma box is 8 px (was 4).
  - On the RTL render, the per-line jitter at the X's fell from 2.40 / 1.30 to 0.28 / 0.17 (right / left X). The bend
    is now gentle: a mean change of 3.9 RGB against no camera, down from 14.9.
  - K6 at 1023 still bends the picture strongly.
  - The gain was chosen from a model sweep on selfie.jpg (1/4, 3/16, 1/8, 1/16; 4- and 8-px boxes). Jitter tracked the
    gain, not the box, and 1/8 still frayed the right X.
- **Osc unchanged.** It is now 16x the camera. Its frame is within 0.97 mean RGB of v0.1.0's.
- **Rest unchanged.** The black-input frame is still bit-exact with `imodel.py`.
- **Cost.** +~90 LC (the 8-tap box). HD Fmax is now 79-85 MHz (v0.1.0: 90-97).

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_cupola/` (`notes.md`, `chans.py`, `contours.py`, `vfit.py`, `proto2.py`,
`imodel.py`, `sheet.py`, `thincheck.py`, `cmpfields.py`).
- **It is an RGB patch.** Split into R, G and B, the frame falls apart into three clean grayscale layers
  (`chans.png`):
  - **R: the pillar lattice.** Tall ovals with a period of 325 px (5.9 across, centres at 230 + 325 k), straight
    sides and round bottoms at y ~665. Below them come a dark band (665-775) crossed by thin ">"-shaped lines, a band
    of circles (radius ~88, centred on y ~870), and a dark band with thin arches (950-1080). This is one contour
    family of u1^2 + v(y): tall ovals where the V wave is flat, circles where it dips, zigzag lines where it is high.
  - **G: the cupola lattice.** Big cells with a period of ~940 px (2.04 across). The cusps, where the X's cross,
    are at x 535 and 1475 on row ~515.
    - Above the crossing the X arms are straight (0.53 px per line). Below it they run on until they meet a round
      floor at y 838 at the cell centre.
    - A bright V spike sits in the upper wedge (tip at y ~210), with a dark eye above it.
    - Inside, the cells carry a vertical ramp: G is 244 in the top strip, 95 at y 140, 11 at the crossing row,
      116 at y 760 and 244 below y 840.
    - So the cells are a diamond key (triangle H + triangle V), plus a V ramp, plus a floor bounded by a parabola.
  - **B: the domes.** A band between two parabolas of the distance d from the cell centre: the top edge at
    y = 17 + 5.2e-4 d², and the lower edge (the top of the red dome) at y = 127 + 2.2e-3 d². It fades out beyond
    d ~330 into streaks parallel to the arms. Two thin arcs run along y = 838 - 6.4e-4 d² and 50 lines lower: the
    white stripes. Every measured edge fits its parabola to a few px.
- **Palette** (k = 10, whole still):
  - red (246,19,4) 24.1 %, orange-red (249,61,5) 15.4 %, dark green (33,89,7) 10.6 %;
  - yellow (238,239,10) 10.1 %, green (24,241,20) 8.8 %, brown-orange (161,63,8) 7.8 %;
  - pink-magenta (241,92,212) 6.9 %, cream (224,227,205) 5.5 %, orange (239,138,18) 5.4 %,
    yellow-green (151,224,9) 5.4 %.

  Each colour is a channel sum: red = R, magenta = R + B, yellow = R + G, white = all three, dark green = half G.
- **Edges.** Every channel has the same horizontal step response: a sharp leading edge, an exponential decay to the
  right (tau ~23 px) and a black undershoot after falling edges. That is a horizontal high-boost with a gain of about
  2: the embossed shadows to the right of every line.
- **Texture.** Soft edges (~6-10 px) and faint horizontal line noise. There is no tearing.
- **Fit.**
  - R: a per-row Viterbi fit of v(y) with h = u1² and a binned profile, rms 34.6 on the 8-bit channel. It ships as
    the vR table and the P_R profile.
  - B and G are designed from the measured contours and column profiles. Every separable fit of G stalled at rms
    ~38.
  - Integer model vs still at the defaults: all ten palette colours are present, with shares within 0.4-6 points.
    The mean abs RGB is 31.1 and the 4x4 region means are within 3-23 levels.

## Motion in the clip
- 150 frames from 3.0 to 8.0 s were measured (`fr/`, `kymo.png`, `vshift.py`).
- **The R rows roll up.** The dark band moves from 665-775 (5.5 s) to 600-700 (6.0 s), 560-620 (6.5 s) and 520-560
  (7.0 s): about 120-140 lines/s. Row-mean cross-correlation gives 10-14 lines per 0.1 s from 3.1 to 5.3 s.
- **The cupola breathes.** The X at 5.5 s opens into a V over an inverted V, with a gap of ~80 lines at 6.0 s and
  ~160 at 6.5 s, and closes again by 7.0 s: a cycle of about 3 s. Meanwhile the domes sink (dome top from y 127 to
  ~240-300) and the cream top strip widens to ~110 lines.
- The horizontal structure is static.
- Across the clip Ron moved the vertical levels:
  - 0.5 s: the X pulled apart into a V over an inverted V;
  - 2.0 s: big round cells with blue rings;
  - 10.5 s: the cupola again, with a blue arc band.

  These became P12 (the X level), K5 (dome curvature) and K3 (rows).
- **The program:**
  - At the default Roll (701) the rows table advances 0.95 entries per field: 2.0 lines/field, 120 lines/s at 60 Hz
    (100 at 50 Hz).
  - A triangle LFO (170 fields = 2.8 s at 60 Hz) opens the X by up to ~170 lines, sinks the domes by up to 110 lines
    and widens the cream strip.
  - At power-up the frame is the still.
  - `cmp_motion.png` shows the integer model (bit-exact with the RTL) at +0.5, +1.0 and +1.5 s next to the clip's
    m-frames.

## What it does
1. **Oscillators.** Two horizontal phase accumulators: the pillars (325 px) and the cupolas (940 px), each folded to
   u (0 at a cell centre, maximum at the cusp).
2. **R.** R = P_R(u1² + vR(y)). vR is a 512-entry line table indexed by the rows phase, which is scaled about the
   centre by Rows and rolled by Roll.
3. **G.** G = Gbg(y) + GX(Fd) + GF(t3):
   - Fd = u2 + 0.545 |y - 515| - lev: the diamond, whose 1.0 contour is the X;
   - t3 = y - 838 + 0.137 D2, with D2 = u2² / 256 x Dome: the floor;
   - GX holds the X ridge, the wedge and the V spike, GF the floor's bright band.
4. **B.** B = min(key(y - 17 - 0.109 D2), key(127 - y + 0.469 D2), fade(u2)) + BA(t3) + BX(Fd) gate(y) + top strip:
   the dome band, the white arcs and the cusp streaks.
5. **High-boost.** Per channel, a one-pole low-pass (tau 32 px at 1080, 24 virtual px at 720p, 32 at SD, restarted
   at each line), then out = s + 2 (s - lp), clamped.
6. **Colour.** Colours (RGB rotation) and Polarity, then the BT.601 matrix as three per-channel ROMs. Luma is
   pre-scaled 16/13 and capped at 940.
7. **Output.** Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 and luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Pillars | the oval columns: x0.33 (2 fat pillars) .. 5.9 across (512, the still) .. x3 (~17 thin) |
| K2 Cupolas | the cupola cells: one giant cell (0) .. 2.04 across (512) .. ~6 narrow cells with X's (1023) |
| K3 Rows | the pillar rows' vertical frequency x0.33 .. x3: long pillars (0) .. the still (512) .. stacked rows of circles and dark bands (1023) |
| K4 Roll | bipolar (+-8 deadband, square law): the rows roll up (+) or down (-); 701 = the clip's 120 lines/s; full = ~15 lines/field |
| K5 Dome | the curvature of the domes, the floor and the white arcs: flat bands (0) .. the still (512) .. deep narrow domes and steep arcs (1023) |
| K6 Etch | how far the camera bends every contour (pillars, X, floor and domes): 0 .. strong (square law); 200 = a gentle, smooth bend; the Osc bends 16x as far |
| S7 Flip | Off (the still) / On: upside down, so the domes hang as bowls and the floor becomes a ceiling |
| S8 Colours | Sunset (the still) / Lagoon: RGB rotated, giving blue domes, a magenta lattice, cyan caps and a red floor |
| S9 Lattice | Single (the still) / Double: the X lattice at twice the density (4 X's), the domes unchanged |
| S10 Polarity | Positive (the still) / Negative: every channel inverted, giving a cyan field, blue X's, green domes and a magenta floor |
| S11 Source | Video / Osc |
| P12 Cupola | the X level (glided): 0 = the X's pulled apart into separate arches (a V at the top, an inverted V at the floor); 507 = the still's crossing; 100 % = the cells pinched into ")(" hourglass windows. The breath LFO rides on top |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) - 16 from a one-line buffer, box-filtered over 8 px, is
  multiplied by the Etch gain and scaled 1/16. It is added to three things: the R coordinate, the X coordinate Fd,
  and (/4) the floor and dome coordinates.
  - The pillars, X arms, floor and domes bend gently around the picture, with no per-line fraying at the default
    (`cmp_final.png`, bottom).
  - K6 raises the bend to a strong one.
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice, centred on its mean, feeds the same path at 16x the
  camera's gain. At the default Etch the X arms kink into the lattice and the pillars wave.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier (32-bit) with 14 slots, the slot decoded two clocks ahead and the operands
    pre-muxed;
  - the shared glide on K1, K2, K3, K5, K6 and P12.
  - Per field it computes:
    - the two phase steps (K1 / K2, 1/8-octave exponents) and their start phases (fixed at the screen centre);
    - the rows step and its start, scaled about the centre and rolled;
    - Roll² and the roll; the dome gain; Etch²;
    - the X level (P12) and the breath LFO terms.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ~7 clocks:
  - the virtual line (1079 - y for Flip) and |y - 515|;
  - the Fd, t3, t1 and t2 line terms and the cream strip;
  - two reads of the line table: the rolled vR and the static Gbg, Bbg and gate.
- **Pixel path (20 stages):**
  1. beam;
  2. folds;
  3. squares, the 8-tap luma box (a balanced adder tree);
  4. dome product, the R and X sums, the etch product;
  5. D2g clamp, etch (camera 1/16, Osc x1), fade;
  6. parabola terms, the coordinates with the etch;
  7. t3 / t1 / t2, the R and X ROM addresses;
  8. floor ROM address, the dome keys;
  9-10. ROM data to fabric, the band minimum;
  11-12. channel sums and clamps;
  13. boost low-pass;
  14. edge;
  15. boost;
  16. Colours / Polarity, matrix addresses;
  17-18. matrix ROM data to fabric;
  19. Y / Cr / Cb sums;
  20. luma 16/13 and clamps.

  Then the shell's output conditioning (stages 21-29).
- **Stores** (22 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | line table (vR, Gbg, Bbg, gate) | 512 x 32 | 4 |
  | R profile | 512 x 8 | 1 |
  | X profile (Gx, Bx) | 512 x 16 | 2 |
  | floor profile (Gf, Ba) | 512 x 16 | 2 |
  | matrix shares (R, G, B) | 3 x 256 x 32 | 6 |
  | line buffer | 2048 x 8 | 4 |
  | chroma 8-back ring (`no_rw_check`) | 256 x 20 | 2 |
  | C_LAT sync ring (`no_rw_check`) | 256 x 3 | 1 |
- **Contracts:**
  - the generated avid;
  - one latency, C_LAT 28, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3, the same geometry);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the chroma running sums snap back to neutral after 8 blank samples (resync); the boost low-pass restarts at
    every line.
- **Verified:**
  - GHDL at 720p60, decimation 1, black input:
    - the v0.1.1 default frame (`rtl11_black.png`) is **bit-exact with `imodel.py`** (100 % of pixels at fields = 3);
    - in v0.1.0, a second capture 3 fields later was also bit-exact (fields = 6). 49 % of the pixels changed; the rows
      moved 2 virtual lines per field.
  - v0.1.1 Osc and camera frames: `rtl11_osc.png`, `rtl11_cam.png`.
  - `sheet.png` (K6 row on the camera) and `presets.png` are rendered from the model.
  - `thincheck.py`: 0 single-row runs of 24 px or more at the defaults, at P12 0 / 100 %, Rows 0 / 1023, Dome 0 / 1023,
    Flip, Double and Cupolas 1023, at four phases, at 720p and 1080p.
  - `dupin.py` reported 0 duplicate-input cells on v0.1.0's netlist.
- **Bug found by the v0.1.0 sim:** `unsigned(3 bits) + 8` wrapped to 3 bits, so the dome gain was 0 and the domes were
  flat. It is fixed by resizing before the add.

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1; all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 81.80 |
  | SD Analog | 83.96 |
  | HD HDMI | 84.76 |
  | SD HDMI | 84.31 |
  | HD Dual | 79.35 |
  | SD Dual | 84.31 |

- 6,647-6,667 / 7,680 LC (86.8 %), 22 / 32 EBR. iceunpack OK x6. The packed config carries the labels and presets.
- Output: `out/rev_b/ron/heirloom/cupola.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh cupola`.
- v0.1.0 (same day): router1, all seed 1, 89.98-98.39 MHz, 6,563-6,579 LC.
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **G and B are designed, not fitted.**
  - The X arms are straight diamond contours, slightly wider (~50 px) and brighter yellow than the still's softer
    orange arms.
  - The V spike is a filled V rather than the still's narrow spike with an eye.
  - The lower wedge's dark slits are not drawn.
- **One cell centre (x 1005) for G and B.** The still's B centre is 25 px to the right of G's.
- **R is the fitted table.** The pillars, circles and dark bands are in place, but the thin ">"-shaped zigzag lines in
  the dark bands are faint, and the pillar column at each cusp has no special outline.
- **Bottom arcs.** They are thin lavender / cyan arcs; the still's are broader white bands.
- **Rolling.** The rows table is one frame of the still repeated, so it has a seam. As it rolls, the oval region
  reaches the bottom of the frame and turns the floor yellow for a few seconds per cycle. The clip's floor stays
  green with yellow arches.
- **Breath.** The breath is a fixed triangle LFO, not the clip's irregular hand movement.
- **Osc.** The Osc lattice is velvet's (640 x 540 virtual px), not a feature of the clip.
- **Camera.** The default camera bend is deliberately gentle (v0.1.1), about a quarter of v0.1.0's.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3: the same two X's, round domes and circles. The boost's shadow is 32 virtual px at 1080
    and SD and 24 at 720p.
