# Owlet — two contour eyes rolling up through a red band of chevrons and tongues

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3b), made from the still
`photos/vid/WIN_20230818_01_17_40_t3p0.png` (clip WIN_20230818_01_17_40, 12.5 s, frame at 3.0 s; the same patch at
0.5 s and 5.5 s, and melted in clip 01_24_01). Two eyes of nested blue / cyan / pink rings with black outlines and violet
triangular pupils sit side by side with a black X between them. Sawtooth tongues ride the bands above the eyes, and a red
band of chevrons crosses the top. In the clip the eye row rolls up the frame, the eyes morph as they pass the centre, and
a red band rolls up about twice as fast, so every other eye passes through it in red. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_owlet/`:
- `notes.md`: the working notes;
- `pal14_*`: palfit at k = 14;
- `r2fast.py` / `dpfit.py`: the rank-2 field fits;
- `palette.py`: the palette design;
- `proto.py`: the float proto;
- `imodel.py`: the integer model of the RTL;
- `fr/`: the clip at 10 fps;
- `slit_*.png`: time slits of single columns.

Measured elements:
- **Eyes.** Axes x = 530 and 1470 (one horizontal period of 940 px; the saddles at x = 1000 and 60 are mirror axes too).
  The black outline is one field level, a notch in the B channel:
  - a flat top on row 345 (a 122 px run);
  - corners on row 520 (x 182 / 878, half-width 348);
  - a pointed bottom apex on row 905, with 45° lower sides and a convex upper arc.
- **Pupil.** A violet (74, 5, 227) / (52, 10, 250) rounded triangle pointing down, flat top near row 480, apex near 730.
- **Rings.** Outward from the pupil: violet → blue → sky → a wide pink (230, 172, 242) → cyan → blue → the black outline
  → blue → cyan → pink → blue → a violet / navy trough → … → a whiter pink → blue → black → red.
  - One colouriser fold is about 195 field units.
  - The troughs alternate violet (the pupil), black, navy, black.
- **Field.** In the eye rows the colours are Pal(V(y) + g(y) S(|x − axis|)), with S = 470 (h / 470)^1.27.
  - The rank-2 fit has an error of 13–20 per channel on rows 450–900.
  - The horizontal gain g runs 0.5 (row 330) → 1.03 (520) → 1.45 (700) → 1.25 (900).
  - V has its minimum (−67) on row 606 and a kink on row 520 (the corners).
- **Saddle X.** Black, at (1000, 520): the level of the second trough.
- **Tongues.** In the bands between the eye tops and the red (rows 100–330, more than 150 px from an axis): sawtooth
  steps with a gradual outward descent and a steep rise, period 129–164 px (horizontal spectra of rows 40–240 and
  950–1040).
- **Red band.** Rows 0–110 at the axes: dark and bright red chevrons (a Λ at each eye axis).
  - The red is a monochrome of the blue palette's brightness.
  - Its lower edge is a field contour, and the cyan tongues poke into it.
- **Left edge.** For x < 70 the field is held, giving horizontal stripes.
- **Palette shares** (palfit k = 14): sky 12.7 %, pink 12.6 %, mid blue 12.1 %, blue 9.2 %, black 9.0 %, navy 6.8 %,
  violet 6.6 %, lavender 6.5 %, red 6.0 %, light sky 6.0 %, cyan 3.9 %, pale cyan 3.4 %. The proto matches every share to
  within 2.4 points.

## Motion in the clip
- **One 5.9 s cycle, repeated exactly.** Frames 5.9 s apart differ by 16 (mean |RGB|), against about 100 at other lags.
  No knob moves in the 12.4 s clip.
- **The eye row rolls up** at about 190 lines/s: the eye centre is on row 720 at −0.5 s, 620 in the still, 525 at +0.5 s,
  435 at +1.0 s and 335 at +1.5 s. The corners stay near rows 490–520 (screen-fixed), so the eye morphs from a heart
  (below the centre) to a lens (at the centre) to an upturned triangle (above).
- **The red band rolls up about twice as fast**, at about 360 lines/s, and is about 950 lines tall.
  - Its lower edge is on row 920 at 0.8 s, 580 at 1.5 s, 380 at 2.0 s and 100 at 3.0 s.
  - Its upper edge enters the bottom at 4.0 s and is on row 880 at 4.5 s.
  - Every other passage of the eye row happens inside it (the clip at 0.5–1.0 s shows red eyes with dark outlines).
- **In the program.**
  - At K4 = 700 the eye oscillator advances one row (1024 virtual lines) per 5.9 s, 2.89 lines per field.
  - The red band runs at exactly twice that rate with a 2048-line period, so the picture repeats every 5.9 s, like the
    clip.
  - Integer-model frames at −0.5 / +0.5 / +1.0 / +1.5 s against the m-frames: `cmp_motion.png`.

## What it does
1. **Line engine** (per line, for the next line):
   - It reads the scroll table at d = (Y − 610) + s: the eye profile Vs, its horizontal gain gs and the fin envelope,
     256 entries over the 1024-line row, interpolated in quarters.
   - It reads the screen table at Y: the still's residuals P and Gf, 256 entries over the frame.
   - It forms V = FP + (Vs + P − FP) kv kr + off kr and g = gs Gf (3/4 + kv/4) kr, then the fin depth a, the red-band
     threshold and the Melt phase.
   - Vs / gs scroll and are symmetric about the eye. P / Gf are screen-fixed, so the default frame is the still and the
     eyes morph as they roll.
2. **Beam.** A triangle oscillator: the phase steps one eye period per 940 virtual px (K1 scales it) and is held left of
   virtual x 70. hn = the distance to the nearest eye axis over half a period.
3. **Pixel.** F = V + g S(hn) + a Z(hn) + Etch × luma.
   - S is the 1.27 power (Lens) or the 0.62 power (Diamond).
   - Z is the fin sawtooth (rise 0.7, fall 0.3, period 134 h-units) times hn.
4. **Colour.**
   - A 1024-entry palette is read at F / 2 + 320 (2 field units per entry, wrapping).
   - Inside the red band (F above the line's threshold), the entry's red level goes through a red ROM instead.
   - The band's edges are a ±100-line ramp of the threshold through the T3 level, so they follow the field's contours.
   - Fire uses the red ROM at 255 − R/2 where F is below the pupil threshold.
5. **Output conditioning.** Chroma goes through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum resync, luma through
   [1 2 1] on the same centre, then luma 13/16 and the blanking gate. Palettes are pre-scaled 16/13 and capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eyes | the eye oscillator's frequency: one wide eye (0, period ×2.9) .. the clip's two eyes (512, period 940 px) .. six narrow eyes (1023) |
| K2 Rings | ring density about the pupil level, ×0.44 .. ×2.3 (512 = the clip): broad glowing eyes .. dense moiré contours |
| K3 Fins | the sawtooth tongues above the eyes: none, smooth arches (0) .. the clip (512) .. ×3.5 with a wider envelope, flames over most of the frame (1023) |
| K4 Roll | bipolar roll speed of the eye row and the red band, ±8 deadband (stops), square law; 700 = the clip (one row per 5.9 s, up); 1023 ≈ one row per 0.75 s; left of centre rolls down |
| K5 Ember | the red band's height: no red (0) .. the clip's 840 lines (512) .. 2000 of 2048 lines, nearly all red (1023) |
| K6 Etch | how far the camera (or the Osc) pushes the field: 0 .. 2.4 (k / 512)² field units per luma step; 140 ≈ 0.18, a bend rather than a shred |
| S7 Twin | one row of eyes / two (the vertical oscillator at twice the rate, screen residuals off) |
| S8 Shape | Lens (the clip) / Diamond: the horizontal shaping turns concave, giving pinched star-diamond eyes |
| S9 Melt | off / the sibling clip's liquid wobble: each line's phase shifted by two rolling triangle waves (±105 virtual px) |
| S10 Fire | off / the pupils burn bright red with a darker rim (the sibling clip's red blobs) |
| S11 Source | Video / Osc |
| P12 GAZE | the eyes' opening: slits with calm horizontal bands (0 %) .. the clip (50 %) .. wide-open eyes with dilated, pink-cored pupils (100 %) |

## Video
- **Video mode (the default).**
  - The camera luma, (Y >> 2) − 16, times the Etch gain is added to the field, so the rings and outlines bend around the
    picture.
  - At the default Etch the eyes still read clearly; at 1023 the picture dominates.
  - With no input, Video mode is the still: the RTL with a black input is bit-exact with the integer model.
- **Osc mode.** A drifting 2-D triangle lattice (240 × 180 virtual px) replaces the camera, fed at 3× the camera gain (a
  per-field gain). The eyes crinkle into a lattice of crosses, and Etch stays live.

## Engine
- **Per-field sequencer** (vblank zone, zc 20–600).
  - One serial shift-add multiplier with 23 slots (18 iterations each); the slot is decoded two clocks ahead and the
    operands are pre-muxed.
  - An exp2 ROM serves the three exponential controls (Eyes, Rings, Gaze).
  - A shared glide covers K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field, ±2 deadband).
  - Per field it computes:
    - the triangle step per h and per pixel, and the line-start phase;
    - kr, kv, KVR, KG and the pupil offset;
    - the fin depth and envelope, and the etch gain (×3 for Osc);
    - the red ramp centre and slope, and the Fire threshold;
    - the roll step, the band height and edge;
    - the Melt steps and the screen-table step.
  - The roll and Melt phases advance at zc 20; the line-0 kick is at zc 600.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in 121 clocks (SD's line is 858):
  - four table reads with quarter-step interpolation;
  - serial products: V, gs·Gf, ×KG, the fin depth, the red ramp, and the Melt phase;
  - the Melt waves at 8-bit precision;
  - the screen-table index as a per-line accumulator.
- **Pixel path (15 stages).**
  1. Beam.
  2. hn → ROM addresses; luma.
  3. ROMs.
  4. Fabric copies; etch partial products.
  5. g×S and a×|Z| partial products; etch sum.
  6. Sums.
  7–8. F.
  9. Palette address; red and Fire tests.
  10. Palette EBR.
  11. Fabric copy.
  12. Red ROM address.
  13. Red ROM.
  14. Fabric copies.
  15. Compose.

  Then the conditioning stages 16–21. One latency (C_LAT 20) in every mode.
- **EBR (19).** Palette 1024 × 32 (8), S 1024 × 10 (3), Z 512 × 8 (1), scroll table 256 × 32 (2), screen table 256 × 32
  (2), red 256 × 32 (2), exp2 256 × 16 (1).
- **Contracts.**
  - The generated avid, one latency in every mode, and the blanking gate.
  - The HD HDMI conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates (h = half a virtual px, steps 2 / 3 / 4; SD shows the centre 1440 virtual px).
  - Colours in the hardware convention (U = Cr), luma pre-scaled 16/13 and capped at 940.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default frame (black input, frame 3) is bit-exact with `imodel.py` on 100 % of pixels from column 4 on; columns
    0–3 are filter warm-up.
  - The Osc and camera renders were checked by eye (`cmp_final.png`).
  - The thin-line check (`thincheck.py`) finds no single-row run of 24 px or more at the defaults, P12 0 / 100 %,
    K5 1023, Melt, Fire or Twin. One or two 38 px runs appear only at K2 = 1023.

## Status
- **v0.1.0 built 2026-11-09, NOT flashed.**
- **Build command:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh owlet`.
- **Results:** ROUTER1, all six configs on seed 1, 19 / 32 EBR, every bitstream passed iceunpack.

  | Config | Post-route Fmax | LC (of 7680) |
  |---|---|---|
  | HD Analog | 95.00 MHz | 7088 |
  | HD HDMI | 91.13 MHz | 7113 |
  | HD Dual | 91.02 MHz | 7097 |
  | SD Analog | 95.43 MHz | 7119 |
  | SD HDMI | 96.01 MHz | 7103 |
  | SD Dual | 92.46 MHz | 7106 |

- **Output:** `out/rev_b/ron/heirloom/owlet.vmprog`.
- **History.**
  - A first router2 build closed on seed 1 but used 7227–7254 LC (94 %), with HD Dual at 74.42 MHz.
  - The trims (a red ROM in place of shift-add colour maths, 8-bit Melt waves, a per-line screen-table accumulator)
    brought it to 92 %.
  - Router2 then stalled on HD Analog seeds 1 and 2, so the netlist is built with router1 (dupin: 0 `CARRY I0==I1`).

## Deviations
- The still's broad, stepped tongues come out as narrower sawtooth spikes (one soft sawtooth harmonic of the eye
  triangle), and only in the bands above the eyes.
- The saddle X is a thin dark seam where the two eyes' troughs meet, not the still's broad black X.
- The palette has no yellow-green, so the pale yellow highlights at the very bottom of each eye axis are not drawn.
- The proto's mean |RGB| error against the still is 42, because band positions are misregistered by a few px over large
  areas. The colour-class shares match to within 2.4 points.
- Motion:
  - The eye rolls up at the clip's rate near the centre but slows above it (the still's screen-fixed residuals hold it).
  - Above the centre it widens into a flat lens, where the clip turns it into an upturned triangle.
  - The red band rolls at exactly 2× the eye rate (the clip is about 2.07×), with straight-ramp edges.
- LC: 92.3–92.7 %, right at the ~7,100 target. Nothing visible was cut.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680): both eyes, but not their outer flanks or the left-edge hold.
