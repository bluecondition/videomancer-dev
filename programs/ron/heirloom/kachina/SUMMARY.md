# Kachina — a four-way mirrored totem mask, breathing and scrolling on a V oscillator

**v0.1.1 built 2026-11-09, NOT flashed** (v0.1.0 the same day; see [v0.1.1](#v011)). Heirloom from the video clips
(phase 3), made from the still `photos/vid/WIN_20231227_22_52_47_t3p5.png` (clip WIN_20231227_22_52_47, 12.0 s, frame
at 3.5 s; the clip's other looks at 0.5, 5.5 and 10.5 s are the same patch at other settings). A magenta ground with
nested ovals at the top and bottom; blue bands with red rims sweeping from the corners into a big X; yellow spindles and
")" crescents; black / dark-green cavities; and shaded red masks with slit eyes: two big side masks, a top / bottom mask
and small masks riding the X arms. Everything is mirrored left/right and top/bottom. Presets are hand-written
placeholders.

## v0.1.1
Review send-back (two changes; the replica, the table, the engine and every other control are unchanged):
- **K5 Etch default 480 -> 380** (the camera now pushes the bands by up to 66 px per full luma instead of 105). At 480
  a busy camera picture (selfie.jpg) shredded the right side mask; at 380 it reads as a mask (frayed edges on a busy
  picture) while the bands visibly bend. Lower Etch gives a cleaner totem.
- **Osc enters u at 3x the camera's level.** Per field, in Osc mode, the etch gain is tripled (c_egx = 3 eg; the pixel
  path's gain is now 12 bits and the etch sum 20 bits). At the default Etch the drifting lattice displaces u by up to
  ~200 px: the side masks pull in and tilt and the X arms bend and wave, a bold flip from Video (the RTL Osc frame
  differs from the black-input frame by 42 mean |RGB|). K5 stays live in both modes (0 = no bend in either). At Etch
  1023 the lattice folds the totem into big, smooth, mirrored warps (up to ~1330 px of u), structured, not noise
  (GHDL NTSC frame after 30 fields of glide, `rtl_osc1023_sd.png`).
- Preset Ember Melt: K5 900 -> 700 (the same knob setting now bends 3x harder in Osc mode).

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_kachina/` (`notes.md`; `sym*.py` = mirror axes; `pal14_*` / `pal22.txt` = palfit;
`chain.py` / `pal.py` = the colour chain; `vmap*.py` / `clipv.py` / `wavefit.py` / `corr.py` = the motion and the wave;
`build_tab2.py` / `spandp.py` / `span_build.py` / `pack.py` / `gen_py.py` = the table fit; `proto.py` = the float
proto; `imodel.py` / `imrun.py` / `cycles.py` = the integer model; `sheet.py` = the control sheet).
- **Four-way mirror.** Left/right axis x = 990 on every row band (flip-correlation error 7-19 mean |RGB|), top/bottom
  axis y = 587 (14-20). The picture is its top-left quadrant (990 x 587 px); the right half reaches u = 929, the bottom
  half v = 487. Capture borders: the bottom 6 rows, the top 2 rows and the left 4 columns are dark (not used).
- **Palette** (palfit k = 14): magenta (202, 27, 210) 22.1 %, black-green (11, 21, 3) 16.0 %, red (215, 10, 4) 8.1 %,
  blue (98, 4, 220) 8.1 %, yellow (185, 133, 5) 6.6 %, blue (63, 7, 183) 5.8 %, red (175, 17, 4) 5.8 %, navy (48, 9, 131)
  5.5 %, brown (64, 24, 4) 5.0 %, dark navy (36, 12, 72) 4.7 %, dark red (123, 20, 4) 4.7 %, salmon (193, 78, 134) 3.2 %,
  hot pink (199, 9, 103) 2.7 %, violet (123, 17, 108) 1.6 %.
- **One colour chain.** Every band sequence walks one path: yellow - orange - red - dark red - brown - black -
  dark navy - navy - blue - violet - magenta - pink - salmon, plus a short magenta - hot pink - red branch for the thin
  red lines between magenta areas. As a 64-entry chain (52 + 12, spaced by arc length) it reproduces the still at
  11.1 mean |RGB| (5.6 after a 5 x 5 median). The gradients (the blue bands darkening to navy on their inner side, the
  masks' red-to-black shading, the red rims' glow into black) are straight walks along the chain.
- **Elements** (still px): top oval x 340-1640, y 0-150; top mask x 760-1250, y 155-330 (slit eyes at x 880 / 1080);
  side mask x 0-420, y 420-760 (neck at the left edge y 525-640, slit eye x 150-175, pink crescent x 300-380); X-arm
  masks at (330-460, 370-420), (440-640, 320-400), (580-830, 270-330); yellow crescents x 470-680, y 420-750; a yellow
  4-point star x 880-1100, y 380-800 with a red disc at the centre; edge zigzag teeth x 0-130 at y ~165, 290, 370.
- **Texture.** Soft horizontal edges (chroma ~6-8 px), very faint line streaks (row-mean sd 0.07-0.24 luma), slight
  tearing on the mask outlines.
- **Fit.** The quadrant is a run table: 256 v slots map onto 82 spans; each span is a pair of keyrows with the same run
  sequence (17-33 runs, median 23), and a row inside a span has every run boundary and start colour interpolated
  (spans 2-37 lines, median 4; 36 keyrows shared between neighbouring spans). 3038 run records + 82 span records + the
  slot and wave tables = 3878 of 4096 words. Proto vs the still 12.9 mean |RGB| (1080p); integer model (frame 3, with
  the output blur and the 13/16 luma) 15.6; the 4 x 4 region means agree to 1-12 levels; luma mean 64.4 vs 64.3, luma
  sd 35.0 vs 36.8.

## Motion in the clip
- **Only v moves.** Row matching of every clip frame against the still's quadrant (95 frames at 8 fps, plus the
  m-frames): every row of every frame is one row of the still's quadrant, with NO horizontal scale or shift
  (k = 1.000, d = 0 on every tested row), mean row error 2-5 |RGB| on the m-frames. The clip is T(|x - 990|, v(y, t)).
- **The V wave**: v(y) is a soft triangle in y, half period ~550 lines (peak-to-trough 532-564), centred on v ~ 280.
  In the still v = 587 - y above the axis and a measured curve below it (33 at y 600, 144 at 720, 282 at 880, 424 at
  1000, 491 at 1060, 587 extrapolated at 1148): falling half 587 lines, rising half 561, slightly S-shaped.
- **Breath (a bipolar VCA)**: |A| = 16 (t 0.25 s) -> 290 (3.3 s) -> 16 (6.25 s) -> 285 (9.1 s) -> falling at
  11.75 s, and at each trough the wave's peaks and troughs swap places: a triangle LFO of period ~11.8 s, +-290 rows, on
  the V oscillator's amplitude. The still is at the + peak.
- **Scroll**: the trough rises 125-131 lines/s (904 at t 1.0 -> 280 at t 6.0; 616 at 7.0 -> 24 at 11.5).
- **The clip's other stills** are the same table: t 0.5 s = A ~22 (vertical wavy stripes), t 5.5 s = A ~80,
  t 10.5 s = A ~148 at another scroll phase.
- **In the program**: K4 = 700 scrolls the wave up at 128 lines/s (2.13 virtual lines per field at 60 Hz); K3 = 512
  runs the bipolar breath LFO at the clip's 11.8 s period from its + peak at power-up; P12 = 512 is the still's
  amplitude (293.5 rows). The proto at +0.5 s / +1.0 s moves like the m-frames (`cmp_motion.png`: the X centre rises
  64 / 128 lines and the totem compresses as the breath falls). In GHDL, the Scroll 1023 frame is displaced 19 lines
  (720p) from the default frame, exactly the integer model's prediction.

## What it does
1. **Row per line.** A line engine kicked at each line start computes the NEXT line: it advances the V phase (Q24, one
   period = 1148 virtual lines at the default Period), forms W = an asymmetric triangle (trough at 587/1148) plus a
   256-entry correction (the still's measured shape), v = 293.5 + L W with L = A(P12) x breath, folds v into 0..587
   (mirrored at both table ends), and looks up its span: keyrow lists A and B and f = (v - va) / (vb - va).
2. **Expansion.** A 9-step fetch pipeline reads run k of A and B and interpolates the boundary and the start colour at
   f. A writer fills one bank of a 2 x 1024 x 8 line buffer with the chain index (6.2 fixed), one entry per clock,
   adding each run's slope (0, +-1/16, +-1/8, +-1/4, +-1/2 index per px). Worst case 1114 clocks at HD / 720p and 619
   at SD (two table px per entry), so it fits SD's 858-clock line.
3. **Pixel path.** u = |x - 990| x Width (a signed DDA) + camera luma x Etch (or the Osc lattice x 3 Etch), folded at
   the table edge (1024; past it the totem repeats mirrored); Centre = Masks reads |989 - u|. The line buffer gives the
   chain index, the Colour rotation is added, and the palette (Kachina or Frost) gives Y / Cr / Cb.
4. **Tear** (S9): bands of 8-71 virtual lines (gaps of 8-263) shifted 40-198 virtual px, whole lines and the mirror
   axis with them; each pattern holds for 4 fields, then jumps.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Width | the H ramp into the mirror: one wide totem, only the centre X and yellow star (0 = 0.35x) .. the still (512) .. four folded totems side by side (1023 = 4x); exponential, glided |
| K2 Period | the V oscillator's period: x4 (0: one giant stretched half, the side masks become tall drums) .. the still (512, 1148 lines) .. x1/4 (1023: four totems stacked); exponential, glided |
| K3 Breath | the VCA LFO on the V wave: 0 = steady (the still's amplitude, no breathing); 0..512 = depth (512 = the clip: bipolar, the totem melts into stripes and re-forms upside down every 5.9 s); 512..1023 = rate, 11.8 s .. 2.4 s |
| K4 Scroll | bipolar, still in the centre +-8: the V wave scrolls down (left) / up (right), square law; 700 = the clip's 128 lines/s up, 1023 = 1000 lines/s |
| K5 Etch | how far the camera's light pushes the bands, masks and arms outward: 0 .. 239 px per full luma (square law; 380 = 66 px); the Osc lattice enters at 3x that |
| K6 Colour | the colouriser's phase: every element walks the chain together, -28 .. +28 entries (centre +-8 = the still; 0 = a maroon ground, red and pink bands, lilac masks; 1023 = an orange-red ground, violet masks, pink cavities) |
| S7 Wave | Triangle (the clip: mirrored top/bottom) / Saw: the top half of the totem repeated, a hard seam where the X centre meets the next oval |
| S8 Centre | Cross (the still: the X and the yellow star at the centre) / Masks: inside-out, the two side masks meet in the middle and the X splits to the edges |
| S9 Tear | Off (the still) / On: sync-tear bands shifted whole, each pattern held 4 fields |
| S10 Palette | Kachina (the still) / Frost: the 23_03_24 clip's look - cyan ground, deep blue bands, white and lilac masks, pale yellow, red lines |
| S11 Source | Video / Osc (the lattice at 3x the camera's level: the totem ripples and waves) |
| P12 Totem | the VCA gain on the V oscillator: 0 = vertical wavy stripes (every line reads one row: the clip's 0.5 s look), 512 = the still, 1023 = 4x: the table folded four times per half wave, a stacked totem pole of mirrored masks; glided |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16, taken straight from the input (one register, used only as
  an operand, no line buffer), times Etch, is added to u after the mirror: bright parts of the picture push the bands,
  masks and arms outward symmetrically, so the totem bends around the camera's shapes. Black adds nothing, so with no
  input Video mode is the still. At the default Etch (380) the composition reads, the side masks included.
- **Osc mode.** The shell's drifting 640 x 540 virtual-px triangle lattice (~75 px/s) feeds the same path at 3x the
  camera's gain: at the default Etch the side masks pull in and tilt and the X arms wave in a slow drifting lattice
  (up to ~200 px of u); Etch scales it, and at 1023 the totem folds into big smooth mirrored warps.

## Engine
- **Per-field sequencer (vblank zone).** A shared glide for K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field, +-2
  deadband); one time-shared 64-entry 2^x table for the exponential knobs; one serial shift-add multiplier with 12
  slots: (x0 - 990) kx, kp, kp per line, 587 kp, scroll^2, scroll step, breath rate, breath depth, A, A x breath and
  etch^2 (x3 in Osc mode). Then |L| and 3|L| for the line engine, the phase at virtual line 0 (trough phase - 587 kp +
  scroll phase) and the DDA start (+ 8 px for the output lag).
- **Line engine.** A radix-4 serial multiplier (load + 8 iterations) does the triangle, L x W, f and the tear offset
  x kx. The wave, fold, slot (v x 445 >> 14) and span reads share the table ROM's port with the expansion, which
  starts at clock 46 of the line.
- **Pixel path (16 stages, C_LAT 16 in every mode):** 0 DDA; 1 |u|, luma; 2 etch nibble products (12-bit gain);
  3 etch sum; 4 + etch and fold; 5 line-buffer address (bank, Centre, SD half rate); 6 line buffer (EBR); 7 fabric
  copy; 8 palette address (+ Colour); 9 palette (EBR); 10 fabric copy; 11 compose (black off the generated avid);
  12-13 U/V [1 2 2 2 2 2 2 2 1]/16 box (running-sum resync) and Y [1 2 1]; 14 luma 13/16; 15 blanking gate.
- **Block RAM (24 EBR):** table ROM 4096 x 16 (16: slot -> span, wave correction, span records, run lists), line
  buffer 2 x 1024 x 8 (4, `no_rw_check`: the write bank is always the other one), palettes 512 x 32 (4: Kachina and
  Frost; luma pre-scaled 16/13 and capped at 940). 0 transparency emulations.
- **Contracts:** the generated avid (parity-locked); one latency, C_LAT 16, in every mode; virtual 1080-line
  coordinates (SD shows the centre 4:3); the output conditioning before the gate with the 8-sums snapping to neutral
  off the avid; colours in the hardware convention (U = Cr, V = Cb); per-field work in the vblank zone (no avid for
  1024 clocks); the line-0 engine kick in the zone.
- **Verified (GHDL, 720p60 decimation 1, real timing ID):** the default frame with a black input matches the integer
  model to 0.41 mean |RGB| (98.1 % of pixels within 4 levels; the rest is the model's float filter); the Scroll 1023
  frame 3 matches the model's field 3 (0.39); the Osc and Video (selfie.jpg) frames render as designed; Osc at Etch
  1023 checked on an NTSC frame after 30 fields of glide. `check_pkg.sh` clean; standalone yosys 0
  `emulate_transparency`; `dupin.py` 0 `CARRY I0==I1`.

## Status
- **v0.1.1 (2026-11-09), built with ROUTER1, all six configs on seed 1.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh kachina`
    (or `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_heirloom.sh kachina`).
  - Fmax (post-route): HD Analog 88.17, HD HDMI 85.72, HD Dual 80.85, SD Analog 87.44, SD HDMI 82.56,
    SD Dual 87.02 MHz.
  - 6831-6849 / 7680 LC (89 %), 24 / 32 EBR. iceunpack OK x6; the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/kachina.vmprog`. **Not flashed.** `hd_clock_divisor` untouched.
- **v0.1.0 (same day)**, router1, seed 1 everywhere: HD Analog 84.29, HD HDMI 88.15, HD Dual 84.04, SD Analog 71.08,
  SD HDMI 76.74, SD Dual 85.97 MHz; 6780-6803 LC; 24 EBR.
- **History.** The first v0.1.0 attempt (router2; two Y/U/V output boxes; the phase compare -> subtract -> x3
  multiplier load in one clock) came out at 7234 LC and 64 MHz best of six seeds on HD Analog. The output went to the
  contract's single U/V box + Y [1 2 1] (C_LAT 20 -> 16), the multiplier loads were pipelined, and the build moved to
  router1.
- **If a later version gets tight**: stay on router1; LC to reclaim: share one serial multiplier between the field
  sequencer and the line engine (they never run at once), narrow the tear generator.

## Deviations
- The still's fine structure lives in a fitted table, so small details are approximated: the X-arm masks are a little
  softer and their slit eyes less crisp; where the run sequence changes between spans a feature can pop in or out over
  2-4 lines (it reads like the still's own slight tearing). Between keyrows the boundaries are linear, so a few curved
  edges show gentle facets.
- The still's bottom half is not an exact mirror of its top (v jumps ~20 rows at the axis and runs 0.8-1.25 rows per
  line); the program reproduces that through the wave correction, so the bottom half is the top half seen through the
  measured curve, not the still's own bottom pixels.
- The breath LFO starts at its peak; in the clip the still is ~0.2 s past the peak, so the program's +0.5 s frame is a
  little more compressed than m1 (A 243 vs 218 rows).
- Edges: the output box gives the still's soft chroma; luma is sharper ([1 2 1]) than the still's. Whites come out at
  the shell's ~777 cap (luma 13/16). The left 8 px fade in from black (the box warm-up and the generated avid's start).
- The faint line streaks and the capture borders are not drawn.
- The masks are part of the table, not a separately keyed layer, so they have no knob of their own; element control is
  through the patch's modules (V oscillator amplitude / period / scroll / breath / waveform, H ramp, colouriser,
  palette, centre fold). K1 Width is a global H scale (the H ramp's gain).
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3 (virtual x 240-1680), so the side
  masks are cut to their inner halves at the screen edges, and SD expands two table px per line-buffer entry.
- Camera: the camera bends u per pixel, so a busy, high-contrast picture still frays band and mask edges on its bright
  side at the default Etch (the masks keep reading); lower Etch gives a cleaner totem.
