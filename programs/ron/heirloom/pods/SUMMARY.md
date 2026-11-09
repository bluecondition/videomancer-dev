# Pods — cyan eyes and maroon cells on corner zone-plate rings, cut into pods by mint arcs, rows rising

**v0.1.1 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20230328_00_11_14` at 0.5 s
(`photos/vid/WIN_20230328_00_11_14_t0p5.png`). A lattice of cyan "eyes" (cyan core, dark-teal ring, yellow-green band)
and maroon cells (dark-brown core, red ring) in olive, mirrored about a seam just right of centre. The cells are small
and dense at the top centre and stretch into long horizontal bars at the edges. A pale mint ground breaks through in
the top corners, along two chains of lens gaps that arc round the corners, and in a big triangle at the bottom centre,
so the lattice forms fanning legs ("pods"). In the clip the rows of cells rise steadily; the rings and the gaps stand
still. Presets are hand-written placeholders.

## v0.1.1 (2026-11-09, review fixes)
- **S7 Quad is now a real four-way flip.**
  - v0.1.0 folded the zone plate at the middle line, which on the sheet changed little more than one row of cells.
  - The zone-plate origin now moves to the middle line: (y - 540)^2 by the same line DDA, started at k 540^2 with
    step k (vs^2 - 1080 vs), so no fold logic.
  - Mint half-discs sit at the middle of the left and right edges, the arcs bow round them, and the picture is
    mirrored top and bottom. The top-corner gaps and the bottom triangle disappear.
- **Osc bends the geometry.**
  - The 4-px-boxed luma x etch gain now also offsets the gap phase (in 1/64 turn), and a quarter of that the ring
    phase: the camera at t >> 10, the Osc (centred on its mean) at (t - 112 eg) >> 5, 32x the camera's bend.
  - The Osc level term is now 2x (was 4x).
  - The arcs, legs and gaps wander and swim as the lattice drifts. Osc vs black input: mean |YUV| 103.8, 79.4 % of
    pixels differ (v0.1.0: 66.9 / 67.5 %).
  - The line buffer is read 9 px ahead so the bend reaches the ROM addresses in time; the level sample sits 6 px ahead
    of the bend's (invisible).
- **K6 Etch** law k6^2 / 8192 (max gain 127, was 255), default 540, which is the same gain as v0.1.0's 380. At max the
  camera bends hard without flooding the picture cyan.
- Everything else is unchanged.
  - The default frame is bit-exact with the model, as in v0.1.0.
  - The new Quad (in the Quad + Chevron + Mosaic + Sweep frame) and the Osc frame are bit-exact too.
  - `thincheck.py` passes, now including Osc, Osc at K6 max and Quad at P12 100 %.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_pods/` (`notes.md`, `rank.py`, `fitE.py`, `fitL2.py`, `fitF.py`, `fitE3.py`,
`fitJ.py`, `tunev2.py`, `proto.py`, `imodel.py`, `sheet.py`, `thincheck.py`, `simcheck2.py`, `osccheck.py`).
- **Palette** (k-means, then the rank classes by nearest centre; median RGB, share):
  - mint (176,224,176) 17.0 %, dark brown (48,30,12) 12.2 %, maroon (89,33,9) 7.4 %, red (153,26,8) 13.0 %;
  - olive (56,95,5) 17.6 %, yellow-green (138,247,0) 14.7 %, dark teal (23,69,41) 9.7 %, cyan (0,255,180) 8.3 %.
  - They are one colouriser, low to high: mint | dark brown | maroon | red | olive | yellow-green | dark teal | cyan.
    Every mint region has a dark-brown rim, and every cyan core a dark-teal one.
- **Mirror seam** at x = 1004.5, not 960: (xL + xR) / 2 over 15 cell pairs on rows 175 / 363 / 551 gives 1001.5..1007.
- **Rows.** Cyan rows at y = 175.5 + 187.5 k and maroon rows half way, exactly horizontal across the whole width: a V
  oscillator, 5.76 rows per frame.
- **Rings.** The cell spacing along a row grows away from the seam (65-90 px at the seam, 120-150 px in the outer
  columns, 300+ px bars at the edges), and the cells line up along arcs round the top corners. That is a zone plate
  cos(k r^2) with r measured from (-50, 0) and X folded at the seam.
  - Regression of a rank proxy on V, the envelope and cos / sin(k r^2) picks the centre (-50, 0) and
    k = 4.139e-5 rad/px^2 (6.587e-6 turns/px^2).
  - Product terms V x cos add nothing (R^2 0.5598 -> 0.5612): the patch is additive.
  - An elliptical zone plate (x-scale 0.6..0.85) fits worse.
- **Gaps.** A 188 x 140 low-pass of the rank proxy is a function of r alone: a circle-centre grid search lands on the
  corner and explains 89 % of its variance. In lattice turns t = k r^2 (0..15 on screen) the gap waveform is:
  - a corner valley centred t = 0.5 (r 275), half-width 0.9 turn, depth 0.92 (the cyan row at y 175 only reaches the
    dark brown there: the dark bar poking into the top-left mint);
  - arc 1 at t = 4.98 (r 870), half-width 0.77, depth 0.67;
  - arc 2 at t = 8.97 (r 1167, the arch gap on the seam at y 427-519), half-width 0.72, depth 0.66;
  - deep from t = 10.4 (r 1256, the bottom triangle, apex (1003, 760)), wall 1.05 turn;
  - plateau +0.19, depth scale 4.5 (rows = +-1).
  The gaps open on the maroon rows, and the lattice bridges every arc on the cyan rows.
- **Field.** F = V + 0.58 cos(2 pi (k r^2 + 0.567)) + 0.19 - 4.5 val(k r^2).
  - Colouriser thresholds (F, rows = 1): -2.48 | -1.165 | -0.82 | -0.336 | 0.254 | 0.743 | 1.175. They are
    quantile-matched, so the class shares come out equal to the still's.
  - Label purity of the float proto: 0.576 (a free 256-bin envelope table reaches 0.60).
- **Texture.** Edges 10-90 % over 10 px horizontally and 7 px vertically; mint edges ragged per line by +-2-4 px; no
  tearing, no noise.
- **Fidelity at the defaults:** integer model vs still mean |RGB| 30.7 (proto 30.9); the 4 x 4 region means agree to a
  median of 4.0 and at most 14.6 per channel.

## Motion in the clip
- The rows scroll **up at 141.6 px/s**, steadily over the whole 7 s clip, one row period every 1.33 s. The phase of the
  bars at the right edge, tracked at 10 fps, moved 495.5 half-res px in 7.0 s.
- The rings and the gap arcs stand still:
  - the cells slide along the rings as the rows pass (m-frame cross-correlation: left column (+17, -35), right column
    (-16, -35) half-res px per 0.5 s, tangent to the corner circles);
  - the gap lenses travel along their arcs with the maroon rows.
- No knob moves in this clip (the 2.5 s candidate is the same patch further on).
- The clip at 00_17_52, six minutes later, shows the same corner geometry, bigger, through a grey / black colouriser
  with rainbow fringes and a ~38 px horizontal sample-and-hold. Palette = Chevron with Mosaic reproduces it.
- **The program:**
  - At the default Speed (700) the row phase advances 421,200 / 2^24 turn per field at 30 fps (half that per field at
    50 / 60): 141.2 px/s.
  - At power-up the rows sit at the still's phase.
  - `cmp_motion.png` shows the integer model at +0.5 / +1.0 / +1.5 s beside the m-frames.
  - A v0.1.0 GHDL frame four fields after the first check was bit-exact with the model at the advanced phase (the
    scroll logic is unchanged in v0.1.1).
  - In Osc mode the drifting lattice also makes the gaps swim.

## What it does
1. **r^2** from just off the top-left corner, folded at the seam (Quad: from the middle of the left edge).
   - Two second-order DDAs per pixel carry the ring phase (kL r^2 + 0.567) and the gap phase (kE r^2) in 2^-28 turns.
   - At the seam pixel they hold for one pixel and reflect the step, so the right half mirrors the left.
   - The y^2 parts come from second-order DDAs in the line engine.
2. **Field.** F = V(y) + Ax cos(ring phase + bend/4) + 0.19 - Dv val(gap phase + bend) + etch * luma.
   - V is the row cosine (quarter-wave ROM, scrolling).
   - cos is a 1024-entry ROM; val is the measured gap waveform, 1024 entries over 16 turns.
   - The bend is the boxed luma x etch gain (Osc 32x), so the camera or the Osc bends the r^2 ramp itself.
3. **Colour.** index = (F + 2.75) x 128, 512 entries per palette.
   - Pods: the still's eight classes with 6-entry blends.
   - Chevron: grey, black and 3-hue rainbow fringes, every hue >= 0.32 F wide.
4. **Mosaic** holds the index for 38 virtual px and the line values for 4 / 3 / 2 lines (1080 / 720 / SD).
5. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes (the capture's horizontal softness and the HD
   HDMI bar fix), then luma 13/16 (the palette is pre-scaled 16/13, capped at 940), then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rows | the rows of eyes and cells: x1/2.83 .. x2.83 rows (exponential; 512 = the still's 187.5-line period) |
| K2 Rings | the zone-plate rings, i.e. the cells' spacing along every row: x1/2.83 (fat eyes, long bars) .. x2.83 (pinstripe cells) |
| K3 Arcs | the gap waveform's rate: x1/4 (one huge corner gap eating the top) .. the still (512) .. x4 (many nested mint arcs) |
| K4 Speed | bipolar (+-8 deadband, square law): the rows scroll up (+) or down (-); 700 = the clip's 141 px/s, full = ~1,100 px/s |
| K5 Cells | the ring level: bars only, rows with a faint ripple (0) .. the still (512, 0.58) .. ring stripes fanning from the corners (1023, 1.98) |
| K6 Etch | how far the camera (or the Osc) lifts the field and bends the r^2 ramp: bright areas close the gaps, swell the eyes and push the arcs (square law, k6^2 / 8192; 540 = the default) |
| S7 Quad | Off (the still) / On: the zone-plate origin on the middle line: mint half-discs at the side middles, arcs bowing round them, mirrored top and bottom (four-way) |
| S8 Palette | Pods (the still) / Chevron (the 00_17_52 clip: grey and black with cyan / magenta / blue and red / yellow / green fringes) |
| S9 Mosaic | Off (the still) / On: 38 virtual px sample-and-hold over 4-line bands (the 00_17_52 clip's blocks) |
| S10 Fold | Mirror (the still) / Sweep: no fold at the seam; the rings and arcs sweep on across the right half (finer cells, a broad mint band, more arcs) |
| S11 Source | Video / Osc |
| P12 Pods | the gaps' depth: 0 % = the bare lattice over the whole frame (the arcs only darken it) .. the still (487) .. 100 % = wide mint arcs pinching the lattice into thin pods |

Glides (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12. Floors keep every control live:
- at P12 0 the gaps still darken the lattice, so K3 moves visible bands;
- at K5 0 the rings still ripple the bars, so K2 stays live.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16 from a one-line buffer, box-filtered over 4 px (the
  camera's bandwidth: it bends rather than shreds), feeds the patch two ways, as an analog artist would:
  - mixed into the colouriser input, so bright areas close the mint gaps and swell the eyes into cyan;
  - summed into the r^2 ramp at a low gain, so the arcs lean round the picture.
  - At the default Etch the rows, legs and gaps still read with `selfie.jpg` (`cmp_final.png`, bottom: mean |YUV|
    51.3 from the black-input frame).
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 640 x 540 virtual-px triangle lattice, centred on its mean, feeds the same path at 2x the
  camera's level and 32x its bend. The arcs, legs and mint gaps wander and swim, and the eyes merge into blobs (mean
  |YUV| 103.8 from the black-input frame; 79.4 % of pixels differ).

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks, zc 18-556):
  - one serial shift-add multiplier (40-bit accumulator, 18-bit operand), 22 slots, operands pre-muxed a clock ahead;
  - a 256-entry exp2 ROM for the Rows / Rings / Arcs exponents;
  - the shared glide.
  - Per field it computes:
    - the row step and the scroll (Speed^2 x 13);
    - kL and kE;
    - per zone plate: the first-pixel step k (2 u0 hp + hp^2), the step of the step 2 k hp^2 (shift-adds),
      k u0^2 (+ the ring offset), k vs^2, and the Quad start values k 540^2 and k (vs^2 - 1080 vs);
    - Ax, Dv, etch^2 and the Osc mean term.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the NEXT line in
  5 clocks:
  - the row cosine;
  - both zone plates' y^2 DDAs (Quad: started on the middle line);
  - the line's start phases, and V + 0.19 + the palette offset.
- **Beam.** Two second-order phase DDAs (32-bit phase mod 16 turns, 28-bit signed step), the seam fold as a one-pixel
  hold with the step reflected, and the mosaic counters.
- **Luma chain.** The line buffer is read 9 px ahead. The chain is a 4-px box, then x etch gain (two partial products),
  then the bend (1/64 turn) added to the gap-waveform and ring ROM addresses.
- **Pixel path (15 stages to the compose):**
  1. beam;
  2. ROM addresses (+ the bend);
  3. ROM data;
  4. fabric copies;
  5. four partial products (cos x Ax, val x Dv);
  6. their sums;
  7. line value + H, the etch level;
  8-9. F and the index clamp;
  10. mosaic hold;
  11. palette address;
  12. palette ROM;
  13. fabric copy;
  14. unpack;
  15. compose.
  Then the two boxes (16-19), luma 13/16 (20) and the gate (21).
- **Memories** (19 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | palette (two palettes) | 1024 x 32 | 8 |
  | ring cosine | 1024 x 10 | 3 |
  | gap waveform | 1024 x 8 | 2 |
  | line buffer | 2048 x 8 | 4 |
  | row cosine | 256 x 12 | 1 |
  | exp2 | 256 x 16 | 1 |
- **Contracts:**
  - the generated avid;
  - one latency (C_LAT 20) in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the running-sum resync on both boxes;
  - luma capped (ROM 940 -> ~764 out).
- **Verified (v0.1.1)** — GHDL at 720p60, decimation 1 (real timing ID), all **bit-exact with `imodel.py`** (100 % of
  pixels):
  - the black-input default frame (field 3);
  - the Quad + Chevron + Mosaic + Sweep frame;
  - the Osc frame, including the line buffer's one-line delay.
  - The selfie frame was rendered (`rtl_cam.png`).
  - `thincheck.py`: 0 single-row dark or bright runs >= 24 px at 1080p and 720p, at two scroll phases, across the
    defaults, P12 0 / 100 %, every switch, the K1 / K2 / K3 / K5 extremes, Osc, Osc at K6 max and Quad at P12 100 %.
  - `dupin.py`: 0 duplicate-input cells.
  - Standalone yosys: 3,358 LUT, 1,165 carry; one transparency emulation (the line buffer, as in every shell).

## Status
- **v0.1.1 built 2026-11-09 with ROUTER1; all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 94.32 |
  | SD Analog | 95.14 |
  | HD HDMI | 98.55 |
  | SD HDMI | 95.71 |
  | HD Dual | 104.28 |
  | SD Dual | 97.62 |

- 6,437-6,455 / 7,680 LC (84 %), 19 / 32 EBR. iceunpack OK x6. The packed config carries the labels and presets.
- Output: `out/rev_b/ron/heirloom/pods.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh pods`.
- v0.1.0 (same day): router1, all six on seed 1, 95.14-106.88 MHz, 6,299-6,312 LC.
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Left-edge bars.** The still's bars at x < 300 run flat to the edge; the circular zone plate curves them slightly
  upward toward the centre. An elliptical zone plate fitted worse overall.
- **Seam cells.** On some rows the still's cells on the seam merge into "M" shapes. The model's fold gives hearts and
  straddling pairs in the same places, with slightly different shapes. The arch gap on the seam is narrower and more
  pointed than the still's.
- **Gap waveform.** Fitted as a corner valley, two raised-cosine arcs and a deep region (label purity 0.576, against
  0.60 for a free table). A few lattice crumbs survive inside the bottom triangle at some scroll phases.
- **Texture.**
  - The per-line ragged jitter on the mint edges is not drawn (per-line jitter aliases into combs on hardware).
  - The still's 7 px vertical softness is approximated by 6-entry blends at every palette boundary.
  - The horizontal softness is the shell's two boxes.
- **Seam position.** The seam is at x 1004.5, as measured, so the right corner gap is narrower than the left, as in the
  still.
- **Cut / not attempted:** a vertical output blur (needs ~10 more EBR); a separate zone-plate centre control (Quad is
  the one origin move).
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3 (virtual x 240..1680): the corner gaps are mostly outside it; the seam, the arcs and the
    bottom triangle are in view. SD is softer (the boxes act per output pixel).
  - At K2 above ~900 the finest rings are ~10 px apart at SD and blur toward flat bands.
