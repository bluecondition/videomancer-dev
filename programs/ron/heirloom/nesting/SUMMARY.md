# Nesting — two eyes of nested blue squares under magenta key rings, breathing into a lattice

**v0.1.0 built 2026-11-10, not flashed.** Heirloom (video phase), from the clip `WIN_20230627_23_37_56` at 7.0 s
(`photos/vid/WIN_20230627_23_37_56_t7p0.png`). The picture is mirrored about the centre: two "eyes" of nested
squares with bevelled corners, in blue, cyan, lavender and black contour rings with red outside. Magenta and purple
key rings sit over them and run on as vertical stripes into a purple vase down the middle. Below is a floor of
red / blue diamonds with grey-teal ghost teeth along the bottom, and every edge is torn into a fine zigzag. In the
clip an LFO breathes the contour gain: the eyes hold for about 2.5 s, then fold into a dense lattice while the
magenta rings thin away, and come back every 5.46 s. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_nesting/` (`notes.md`, `m1..m9.py`, `fit1..fit4.py`, `proto.py`,
`imodel.py`, `msheet.py`, `thincheck.py`, `simcheck.py`, `abl/`).
- **Two layers, two mirrors.** Flip-correlation puts the magenta key layer's mirror at x 1037-1039 and y 516, with
  the eyes at x 568 / 1510. With the key masked, the blue contour layer's eyes sit at x 547 / 1490 (mirror 1018,
  21 px left). Those centres stay put through the whole breath.
- **Key rings (layer B)**, along the axes from (568, 516):
  - horizontal ring centres at 78, 180, 282, 381 and 472 px (about 100 px apart, 30-50 px wide);
  - vertical ones at 77, 194 and 307 px. That is a vertical scale of 0.88, and the duty is larger on the
    top/bottom sides.
  - At the eye row the rings carry on to the mirror as vertical stripes (B runs at y 520: 732-765, 826-869,
    922-975, 1020-1055 ...). Ring 4 is the centre column, widening into the purple vase (solid 700-1233 at y 240).
  - The key exists only for 151 < y < 881. It is shorter near the side edges (ring 3 spans y 300-700 at x 200).
  - Key colour by distance from the centre column (median): (107, 0, 220) purple at 0-80 px, shading to
    (192, 0, 165) magenta at 400-560 px. Ring 2 and beyond are darker (158, 1, 134).
- **Contour layer (A)** along the axis from (547, 516), as label runs: navy 0-14, black 22-38, navy to blue
  46-98, dark navy 110-128, navy / blue / cyan / lavender 134-195, cyan / blue / navy / black 204-258, navy
  264-280, blue 284-296, black 310-328, navy / blue / lavender 340-386, blue / navy / black 392-414, then dark red,
  red and black 424-476, and red beyond. A brightness wave of about 105 px.
- **Palette** (k-means, k = 12, median RGB):
  - black (37, 18, 23), dark navy (22, 29, 91), navy (28, 50, 155), blue (19, 66, 234);
  - cyan (37, 128, 252), lavender (138, 93, 235), dark red (113, 12, 10), red (200, 12, 2), grey (108, 66, 89);
  - key: magenta (198, 0, 184), dark magenta (158, 1, 134), purple (116, 1, 214).
  - Shares: key 30 %, black 12 %, navy 11 %, red 7 %.
- **Field fit.** Two candidates were scored by palette-label purity: a product of oscillators (cos x cos) scored
  0.40 and the octagon max-mixer 0.386, both weak against the torn image. The octagon was kept: it gives the
  even ring spacing, the bevelled corners (diagonal / axis ratio 1.22, so k ≈ 0.15) and the palette shares.
- **Floor.** Below y 846, rows of flat diamonds about 250 x 60 px, in red / blue / cyan.
- **Teeth.** Below y 985, translucent grey-teal slanted teeth at x 560-1350.
- **Tear.** Per-row edge offsets come in runs of 2-4 rows, ±4-7 px, saw-like: the row-shift autocorrelation
  peaks at lag 10 and the FFT at 11 rows. Strong vertical edges also bulge slowly, and horizontal edges are soft
  (6-10 px).

## Motion in the clip
I extracted all 339 frames (30.05 fps, 11.3 s).
- **One LFO.** The row kymograph of layer A (key masked) autocorrelates at **164 frames = 5.46 s** (r 0.77): an
  oscillator, not a hand on a knob. The clip's other candidate still (0.5 s) is the far end of the same cycle.
- **Its shape.** Ring radii around the left eye are symmetric about **7.47 s** (6.66 ↔ 8.12 s, 7.02 ↔ 7.89 s).
  That is the minimum-gain turnaround, and the still is 0.47 s before it.
  - The eye state lasts about 2.5 s of each cycle and the lattice about 3 s, with quick transitions between them:
    a flat-bottomed breath.
  - The lattice cells are about 190 x 90 px, so the gain is about x5 at the peak.
  - The eye centres never move, so the gain scales about the fold points.
- **The key.** The magenta layer is static in x. Its width breathes with the LFO: key fraction 0.18 at 6.0 s,
  0.34 at 8.6 s, nearly gone in the lattice state.
- **What the program does.**
  - The LFO phase steps 200 / 65536 per field at 60 fields/s (400 at 1080p30): 5.46 s.
  - The breath is b = 0.08 c0 + 0.92 max(0, c0 - 0.35) / 0.65, with c0 = (1 - cos) / 2. The gain is
    2^(e/256), e = e0 + D b, with D = 2.32 octaves at the default: x1 → x5.
  - The key duty shrinks as the gain grows, so the rings thin away in the lattice.
  - Power-up starts 0.47 s before the turnaround, which is the still's moment.
  - The tear's wobble drifts (one cycle every ~8 s), so the edges keep crawling while the eyes hold.

## What it does
1. **Layer A (the contours).**
   - u = ||x - 1018| - a|, v = |y - 513|.
   - Triangle folders: Tu = fold(g u / 471), Tv = fold(g v / 471), carried as two Q20 phases. The H phase is a
     beam DDA that reverses at the folds; the V phase is per line.
   - F = max(Tu, Tv) + k min(Tu, Tv), and the colour is PAL[F * 320 / 1024 + etch * luma] (512 entries).
   - Floor (y ≥ 846): H gain 1.9 √g, V gain 8.6 √g, k = 1 (diamonds), PAL[F * 112 / 1024 + 204].
2. **Layer B (the key).**
   - u' = ||x - 1038| - a'| and v' = 0.88 |y - 513|, both in rings of 97 px (Q10).
   - FB = max + k min (+ luma).
   - Magenta where the ring phase is within the duty: 22 / 64 on the u sides and 31 / 64 on the v sides. It is
     thicker toward the window edge (+3 (|y| - 230) / 16) and down the vase (+|y| / 13), inside a window
     |y - 513| + (|dx| - 560) / 4.6 < 365.
   - Colour from a 32-entry table by distance from the centre (5 steps purple → magenta), ring ≥ 2 (darker) and
     ring edge (×0.85).
3. **Ghost teeth.** Below line 985, a slanted phase of the B rings is blended half way to grey-teal.
4. **Tear.**
   - Every 2 lines, an offset from a saw (±6 px, 10 virtual lines, its phase jumped every 32 lines).
   - Plus a wobble: ±5 px, 240 virtual lines, drifting.
   - Both shift both layers.
5. **Output.** The shell's conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line
   resync, then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eyes | the two eyes' spacing from the mirror: 0 = merged into one central nest .. 471 px (512, the still) .. 861 px, eyes out at the frame edges (1023); both layers |
| K2 Corners | the rings' corners, k = -0.45 .. 0.98: 4-point stars with concave sides (0) .. the still's bevelled squares (420, k 0.14) .. diamonds (1023); both layers |
| K3 Pink | the magenta rings' spacing, exponential: 223 px, a few fat rings (0) .. 97 px (512, the still) .. 42 px, fine rings (1023) |
| K4 Breath | the LFO: 0 = no breath (the eyes hold; the wobble still crawls) .. the clip (512: 5.46 s, gain x1 → x5) .. 4x faster (1.4 s) and deeper (up to x12) (1023) |
| K5 Tear | the zigzag and wobble: clean edges (0) .. ±6 px zigzag + 5 px wobble (512, the still) .. ±24 px, 14-line zigzag + 10 px wobble, shredded (1023); square law |
| K6 Etch | how far the camera (or the Osc) bends both ring systems: 5 k² / 16384 (27 at the default 300: white = 23 palette steps and 0.18 key rings; 319 at max: about half the palette and 2 rings) |
| S7 Quad | Off / On: the V ramps folded again at ±270 lines, giving two rows of eyes (four eyes; no floor or teeth) |
| S8 Pink | On (the still) / Off: the magenta key layer removed, leaving the bare blue contour eyes |
| S9 Palette | Photo / Ember: gold, lime and teal contours (the key stays magenta) |
| S10 Colours | Forward (the still) / Reverse: the colouriser runs backwards, giving red-cored eyes in a blue field and a blue floor |
| S11 Source | Video / Osc |
| P12 Nesting | the contour gain, exponential and glided: 0 % = one huge nested square per eye (x0.47) .. 300 = the still .. 100 % = the clip's lattice state, a dense grid of small squares (x6, breathing up to x12) with thin magenta rings |

## Video
- **Video (the default).** Camera luma, (Y >> 2) - 16 from a one-line buffer, times Etch, enters two places:
  - layer A's palette index, so the contours bend and fray around the picture;
  - layer B's ring phase, so the key rings ripple.

  At the default Etch the still's structure stays readable (RTL render with selfie.jpg in `cmp_final.png`). With
  no input it is the still.
- **Osc.** The shell's drifting 640 x 540 virtual-px triangle lattice, centred on its mean, feeds the same path at
  **8x** the camera gain. At the defaults the eyes become octagons, the vase twists and the corners fill with
  diamonds (95 % of pixels differ from the black-input frame). At full Etch it becomes a lattice of diamonds.

## Engine
- **Per-field sequencer (vblank zone).**
  - 20 slots on one serial shift-add multiplier, with the slot number decoded a clock ahead and the operands
    pre-muxed.
  - A 256-entry exp2 ROM for the exponential controls (gain, LFO rate, key spacing).
  - The LFO phase and its cosine, the breath shaping, the clamped gain exponent.
  - Per field: the folder steps SH, SHf and SVf, the key step SB / SVB, the ghost slant, the tear amplitude and
    rate, the wobble amplitude, the etch gain, the key duties, the eye spacing and fold points, the per-pixel steps.
  - The shared glide over K1, K2, K3, K5, K6 and P12.
- **Per-line engine.** Kicked at each line start (and once in the zone for line 0), it computes the NEXT line in
  about 172 clocks, so it fits SD's 858-clock line. It uses the same serial multiplier: the zone has no line starts,
  and the line-0 kick comes after the last slot. Per line:
  - the V folder phase;
  - the tear saw (offset binary, held over 2 lines) and the wobble cosine;
  - the line's first x and |u| at it for both layers;
  - the DDA start phases;
  - the key's v' term, its duty extras and the ghost offset.
- **Beam.** Two folded phase DDAs:
  - A in Q20 (21 bits), B in Q16 rings (22 bits).
  - Each step takes the direction of the destination pixel, from a compare against the three fold points two
    pixels ahead. The first step's directions are set in hblank.
- **Pixel path (15 stages to the compose).**
  - Folder and max / min.
  - k·min partial products and sums.
  - F, FB and the window.
  - Ring phase and duty, palette index.
  - Palette ROM (EBR) + fabric copy, key colour ROM (EBR) + fabric copy.
  - Key mux, ghost blend, compose.
  - Then the output conditioning (two boxes, the 8-sample delays of both boxes in one 256 x 60 block-RAM ring with
    `no_rw_check`), luma 13/16 and the gate.
- **ROMs / RAMs (21 EBR).**
  - Palette 1024 x 32: Photo + Ember (8).
  - Key colours 256 x 32 (2), exp2 (1), cosine x 2 (2).
  - Line buffer 2048 x 8 (4) and the box ring 256 x 60 (4).
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 20, in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
  - The HD HDMI conditioning, with the running-sum resync.
- **Verified.**
  - `check_pkg.sh` is clean.
  - GHDL 720p60 at decimation 1, black input, default knobs: **99.78 % of pixels bit-exact with `imodel.py`**
    (mean |d| 0.06). The remainder is a 1-px difference on one fold line near the top of the right eye and the
    last two rows.
  - `thincheck.py`: 0 single-row hard lines ≥ 24 px at the defaults, P12 0 / 100, Quad, Reverse, Tear max, fine
    Pink and Osc, each at fields 3 / 60 / 110 / 200.
  - Model vs still at 1080p: 4x4 region-mean |RGB| difference 6-31 per cell; frame means (96, 27, 113) vs the
    still's (98, 35, 124).
- **Fit history.**
  - The first cut came to 8438 LC (110 %) and failed placement.
  - Cuts that kept the default frame bit-exact: the box delays went into the block-RAM ring, the key colours into
    EBR, the key datapath to Q10, one shared serial multiplier, no hand-over layer for values the engine holds, one
    |y - 513| for both layers.
  - S10 "Mult" (a cos x cos multiplier field) was replaced by Colours Reverse (-300 LUT).
  - That gave 7282 LC (router2: HD Analog seed 2, HD HDMI missed seed 1), then 7155-7183 on router1, seed 1
    everywhere.

## Status
- **v0.1.0 built 2026-11-10, ROUTER1, all six configs on seed 1:**

  | Config | Fmax (post-route) | LC |
  |---|---|---|
  | HD Analog | 94.27 MHz | 7171 |
  | SD Analog | 83.47 MHz | 7159 |
  | HD HDMI | 88.98 MHz | 7183 |
  | SD HDMI | 90.22 MHz | 7158 |
  | HD Dual | 92.92 MHz | 7155 |
  | SD Dual | 87.35 MHz | 7174 |

  - LC is out of 7680 (93.2-93.5 %, slightly over the ~7,100 target). 21 / 32 EBR.
  - iceunpack OK x6, and the packed config carries the new labels.
- `out/rev_b/ron/heirloom/nesting.vmprog`. **Not flashed.**
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh nesting`.
  `dupin.py`: 0 `CARRY I0==I1`; the one LUT dup input (the Breath rate exponent) was rewritten before this build.
- **Presets** (hand-written): Clip, Lattice, Big Eyes, Four Eyes, Star Ember, Red Eyes, Shredded, Osc Bloom.

## Deviations
- **Frame edges.** The contour field is the octagon max-mixer, so it does not reproduce the still's frame edges
  exactly. The partial eyes at the left and right edges, the red curl at the far left, the diagonal blue rays in
  the top corners and the dark "^" at the top centre are drawn as the field's own continuation: red corners and a
  blue star at the top centre.
- **Key rings** are a hard key with darkened edges. The still's rings are softer and slightly wider (key share
  28 % vs 30 %).
- **Camera artefacts.** The still's capture blur, the green / cyan fringes in the top band and the camera moiré
  are not drawn, so the edges are crisper.
- **Floor and teeth.** The floor is a second set of folder gains: the diamonds are smaller and more regular than
  the still's soft red-dominated ones. The ghost teeth are a translucent slanted band of the key-ring phase, not
  the still's individual tooth shapes.
- **Tear** is deterministic (a jumped saw plus a wobble), not random. Its run length is 2 output lines (2 virtual
  lines at 1080p, 3 at 720p, about 9 at SD), so SD's zigzag is coarser. The saw period is in virtual lines, so the
  shape matches across modes.
- **Vertical centres.** Layer A's and layer B's (510 / 516 measured) were merged at 513 to save logic.
- **The lattice state** is a grid of small squares (the same mechanism at high gain). The clip's lattice reads
  more as hexagons and diamonds.
- **Camera luma** reaches layer A 3 px after layer B (invisible).
- **The Osc** frame is not bit-exact in the integer model (its lattice is approximated there).
- **Resolutions.** 720p matches 1080p. SD shows the centre 4:3: both eyes are inside, but the outer partial
  structures and the screen-edge red corners are cut.
