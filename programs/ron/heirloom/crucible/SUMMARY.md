# Crucible — an orange flame cross on magenta, a breathing black figure, a column of blue lenses

**v0.2.0 built 2026-11-21 (Ron's preview notes: glitch fixes + a P12 climax; see the last section). v0.1.x was flashed.**
**v0.1.1 built 2026-11-04.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230518_23_23_28_t0p5.png` (clip WIN_20230518_23_23_28 at 0.5 s; 8.7 s clip, 30 fps).
Violet ground with hot-pink/red flame wedges pointing at the centre column, an orange/yellow X band across
the middle flanked by red, a black head-and-shoulders figure above it and an inverted black diamond below,
pale rims and mauve shading around the black, and a column of stacked blue lenses offset left of the axis
that the cross turns cream. Presets are hand-written placeholders.

v0.1.1 (review fix): S7 Twin On is now two separate lens columns at ±420 virtual px from the axis, one on
each side of the figure; S10 Tear On is now an obvious sync tear. Both Off states are unchanged from v0.1.0.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_crucible/` (`notes.md`, `proto.py` = the float proto, `imodel.py` = the
integer model, `sheet2.py` = the control sheet, `fr/` = every clip frame at 480x270).
- **Mirror axis** x = 992 (flip-correlation error ~20 on every band of rows, all three stills). The vertical
  symmetry is only approximate (best axes 472 / 468 / 536, error ~50): it comes from the oscillator, not a mirror.
- **Palette** (k-means, k = 14): violet (189,0,249) 18 %, black (15,6,6) 16 %, red (251,5,7) 13 %,
  yellow-orange (249,181,1) 8 %, dark purple (122,13,142) 7 %, hot pink (254,1,99) 7 %, grey-mauve
  (111,50,86) 7 %, orange (253,120,0) 6 %, purple (143,0,208) 6 %, blue (10,43,250) 4 %, azure (3,129,244)
  4 %, cream (206,218,149) 2 %. The RGB channels are near-binary keys (R high except in blue/black; B high in
  violet/blue; G only in the X, cream and cyan): an analog RGB keyer patch.
- **Violet shading**: R = 144 + 0.28 (|y - 540| - 270), clipped 110..205 (fits columns x = 300 in two stills);
  B dims toward the middle. **Pink** = the red's B channel growing with |y - 540|: B ≈ 0.7 (|dy| - 280), up to 140.
- **Flame wedges**: top-left red 20–190 at the edge, tip at x = 720 / 1260 (y ~95); bottom 720–890, tips at
  x ~830 / 1150. Fitted as F + 1.27 (1 - |y - 490|/500) > 1.68 with F = W + 1.4 |x - 992| / 960.
- **X band**: 425–500 at the centre column (an orange core 433–456 between cream), 390–594 at the edges,
  flanked by red (315–390, 594–638). Inside it G runs yellow (200) at the edges to orange (112) at the crest.
  Fitted as F > 0.80 inside the figure zone (narrow at the centre, wide outward = the bowtie).
- **Black figure** (per-line half-width from the axis): head 85–90 px from y 120 to 220 with a dome top at
  y 70; shoulders 106 / 138 / 181 / 233 / 297 / 378 / 495 px at y 240 / 260 / ... / 360, flaring into thin
  wings along the X to x 440 / 1560. Lower part a broad diamond: ~400 px at y 660, 340 at 700, 190 at 860,
  0 at ~915.
- **Rims**: pale striated rims (205,180,175) on the figure's outline; grey-mauve fading into violet over
  ~240 px outside it (the big arcs at 1100–1580 x 100–370 and 0–650 / 1330–1920 x 600–760).
- **Lens column**: x 575..1165, centred at x ~870, not on the axis. Rows at y 0–50, 70–118, 140–248 (top),
  900–990, 1020+ (bottom); elliptical (half-width 107 / 202 / 265 / 270 / 175 across a 90-line row). Fitted as
  -cos(2 pi phi(|y - 520|)) + 1.65 ((x - 875)/295)^2 < 0.65, rows at |y - 520| = 0, 326, 426, 512.
- **Fit**: float proto 28.5 mean |RGB| against the still; integer model 28.4; GHDL RTL 28.3 (720p, black input).

## Motion in the clip
- Measured on all 260 frames. Outside the centre column the bands **roll down at 172–176 lines/s** (row
  cross-correlation at x = 8 .. 480, top and bottom alike). Any edge pixel repeats every **2.07 s**
  (autocorrelation 0.74), so the spatial period is **356 lines**.
- The orange X stays in the middle zone (y 375–665 at the edge). The yellow appears each time a crest
  passes through it.
- The lens gap lines at x = 876 **do not travel**: they oscillate with the same 2.07 s period (the top line
  30 → 122 lines in 1.3 s and back in 0.6 s; the bottom line 1046 → 1002 → 1046). The rows breathe.
- The centre cycles head → bell → arcs → diamond band → head every 2.07 s. The still (t 0.5) is the head
  phase; t 0.0 and t 2.0 are the diamond phase. The clip's other stills (t 1.5, t 2.0) are phases of this
  same cycle; no knob moved during the clip. Frame-to-frame mean |diff| is 6.2 (continuous, no cuts).
- **What the program does**: one roll phase drives everything that moves. The flame oscillator rolls at
  172 lines/s (2.87 lines per field at 60 Hz). The figure morphs head ↔ diamond (b = ½ - ½ cos(roll phase),
  the head at the still, the diamond about 1.1 s later). The lens rows breathe ±9 % in the same period. The
  GHDL frames 4 and 11 move as expected: edge bands 10 rows (720p) down, the X down with them, the lens rows
  shifted, the centre column slower.

## What it does
1. **W(y)** = cos(2 pi ((y - 102)/P - roll)), per line from a 256-entry cos ROM. Then F = W + 1.4 |x - 992|/960
   + heat + camera (the H triangle as 3·|x - axis|/16 per pixel).
2. **Zone**: |x - 992| < hw(y). hw comes from two half-width tables (the head phase's upper and lower arms,
   and the diamond phase), lerped by the morph b, minus 80·b h, times the Figure scale. A border band is the
   same tables 50 lines nearer the centre.
3. **Inside the zone**: yellow/orange (index = F - TY) where F > TY(y), cream where that falls on a lens. The
   lens shows where F > TL(y), with a cyan rim at its edge; on trough lines (W < -0.77, wide zone) it shows as
   eyes. Black elsewhere.
4. **Outside**: the lens (blue, deeper toward its centre); red/pink where F > TR(y) or F > TB(y) in the border;
   the pale rim 2–10 vpx outside the zone; else violet by |y - 540| (8 dithered bands) shaded to mauve by the
   distance to the zone (32 levels over 248 vpx).
5. A 1024-entry palette (Y, Cr, Cb), then the shell's output conditioning.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the flame stripes' period: x2.8 fat bands (0) .. 356 lines (512, the photo) .. x0.35 dense stripes (1023). The roll / morph period follows it |
| K2 Roll | the flame oscillator's roll, bipolar (still in the centre ±8, square law): 770 = the clip's 172 lines/s down, 1023 ≈ 700 lines/s, left = rising. The figure morph and the lens breathing are locked to it |
| K3 Figure | the black figure's size (head, shoulders, diamond and the X's zone): x0.44 .. x2.3 |
| K4 Morph | the figure's breathing: 0 = the head holds (the still), 512 = the clip's head ↔ diamond cycle, 1023 = the diamond band holds |
| K5 Lenses | the blue lenses' size: slivers (0) .. the photo (512) .. fat lenses merging into a column (1023) |
| K6 Camera | how much the camera's light (or the Osc lattice) heats the flame field: 0 .. 3.6 F units for white (square law); 400 = 0.47 |
| S7 Twin | Off: one lens column, offset left of the axis at x 875 (the photo). On: two separate lens columns at 992 ± 420 vpx (x 572 and 1412), one each side of the figure, with violet/black between them and the cross cream on both |
| S8 Flip | the figure upright (head on top, the photo) / inverted (head at the bottom, diamond on top) |
| S9 Palette | Flame (the photo) / Ice (Cr ↔ Cb: a cyan cross and blue flames on magenta, orange lenses) |
| S10 Tear | Off: the still's faint line jitter (±1–4.5 vpx on 1 line in 8). On: sync tear; a burst starts on 1 idle line in 8, lasts 8–39 lines and shifts the whole picture ±40–198 vpx (±2 h of jitter inside), so a third or more of every frame is torn |
| S11 Source | Video / Osc |
| P12 Heat | the flame level: 0 = cold (violet ground, the black figure and the blue lenses, no fire), 546 = the photo, 100 % = molten (red everywhere outside, the zone burning yellow, the figure a few dark seams) |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) - 16 from a one-line buffer, times K6, is added to the
  flame field F. Only bright light adds; black adds nothing. Light parts of the picture burn: red outside
  the figure, yellow inside it. Dark parts stay violet or black. The bands fray along the picture's edges.
  With no input, Video mode is the still.
- **Osc mode.** The shell's drifting 640 x 540 virtual-px triangle lattice (~75 px/s) feeds the same path.
  At the default Camera the flames wobble and the lens rows show through the head's top. K6 scales it.

## Engine
- **Vblank-zone sequencer**:
  - A shared 7-channel glide (K1–K6, P12; 1/8 of the gap per field, ±2 deadband).
  - exp2-ROM knob curves: the stripes' 1/P, and Figure's scale and its inverse.
  - One serial multiplier, six slots of 24 clocks: 1/P, speed², the roll step, 102/P, the morph bias, the
    camera gain.
  - The roll phase update, then two cos-ROM lookups for the morph b and the lens breathing. The cos ROM is
    shared with the line engine; the sequencer owns it while zc < 1024.
- **Line engine** (kicked at each line start and at zone clock 1300 for line 0; computes the NEXT line in
  ~192 clocks, so it fits SD's 858-clock line). One serial product at a time:
  - the W phase (y · 1/P) and W from the cos ROM;
  - the scaled distances (|d| / Figure) and the four zone-table reads;
  - two lerps by b and two scales by Figure (hw, hb);
  - the lens row index (|y - 520| · breathing) into the lens ROM, then LT = (th + cos) · 824;
  - the thresholds TY, TB, TL, TR (shift-adds; TY gets ±8 of per-line noise for the yellow index: the
    still's horizontal streaks);
  - the violet band, its dither fraction and the pink index;
  - the tear / jitter offset from a per-line LFSR (Tear On: burst magnitude 80 + 5 r h, length 7 + r lines);
  - the two lens-DDA starts (|dx|² products, D0 = 2 dx xs + xs² from the magnitude).
- **Beam**: x in h, plus two exact second-order DDAs (A += D; D += 2 xs²): A = (x - 875)² and (x - 1109)²
  (Twin Off; only the first is used), or (x - 572)² and (x - 1412)² (Twin On; the minimum is used).
- **Pixel path (15 stages)**:
  - 2: |x - axis|, min A (Twin).
  - 3: the H term 3·ax/16, the zone and border distances, LT - A.
  - 4: flags and indices, the camera luma.
  - 5: the camera product (8 x 5).
  - 6: F.
  - 7: F minus the four thresholds.
  - 8: decisions, the yellow index.
  - 9: the palette address (priority).
  - 10: palette ROMs.
  - 11: fabric copy.
  - 12–14: pads.
  - 15: Ice swap, black outside the generated avid.
  - Then the two [1 2 2 2 2 2 2 2 1]/16 boxes (16–19), luma 13/16 (20) and the gate (21).
- **Block RAM (21 EBR)**:
  - palette 3 x 1024 x 8 (6)
  - zone table 1024 x 12 (3)
  - lens table 512 x 8 (1)
  - cos 256 x 10 (1)
  - exp2 256 x 16 (1)
  - camera line buffer 2048 x 8 (4)
  - output-box 8-back rings 4 x 256 x 16 (4)
  - sync ring 256 x 4 (1)

  All rings carry `no_rw_check`; the only transparency emulation is the line buffer's.
- **Contracts**:
  - the generated avid;
  - one latency, C_LAT 20, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - Y/U/V boxes and luma 13/16 before the gate, with the running sums snapping to neutral off the avid (resync);
  - palette luma pre-scaled 16/13 and capped at 940;
  - colours in the hardware convention (U = Cr).
- **Verified** (GHDL 720p60, decimation 1, v0.1.0 RTL; the v0.1.1 changes touch only the Twin-On and
  Tear-On paths and are checked in the integer-model sheet):
  - Osc, Video (selfie.jpg) and black-input frames. The black-input frame matches `imodel.py` structurally
    at 8.6 mean |RGB|, best-aligned; the residue is only along edges (a few lines of roll phase).
  - Frames 4 and 11 differ as the motion predicts (above).
  - Bug found by the render and fixed before the build: the lens breathing scale overflowed a 9-bit signed
    resize, which made the lens column full-height.

## Status
- **v0.1.1 built 2026-11-04 through the shared queue with ROUTER1, all six configs on seed 1.** NOT flashed.
  - Post-route Fmax: HD Analog 76.80, HD HDMI 82.15, HD Dual 76.03, SD Analog 69.08, SD HDMI 75.63,
    SD Dual 78.45 MHz (SD Fmin 27).
  - 6,994–7,010 / 7,680 LC (91.3 %), 21 / 32 EBR. iceunpack OK x6; the packed config carries the labels and
    presets.
  - dupin.py: 0 duplicated pins (standalone synth).
- Output: `out/rev_b/ron/heirloom/crucible.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh crucible`.
- **History.**
  - v0.1.0 (same day, router1, all seed 1): HD Analog 77.18, HD HDMI 80.48, HD Dual 83.35, SD Analog 83.35,
    SD HDMI 70.58, SD Dual 82.15 MHz; 6,918–6,952 LC.
  - v0.1.1: Twin On moved to two separate columns; Tear On made obvious (+~65 LUTs).
  - router2 stalled with `overused=2` stuck for thousands of iterations: on SD HDMI in the first build, and
    on HD HDMI in the second. That second build was after the three sign-extended self-add shapes that
    `dupin.py` flagged (`(dx<<1)+dx` and the like) were rewritten from magnitudes, and dupin then reported 0.
  - router1 closes all six on seed 1. Stay on router1.
  - HD Analog's margin is thin at 76.80 MHz; if a later edit costs timing, it is the first config to watch.

## Deviations from the photo
- **The figure's outline is a fixed table that morphs between two shapes** (head and diamond).
  - In the clip the outline also bulges and splits in between (the "bell" and the "chess piece with cyan
    arcs" phases). The program's in-between frames are lerps of the two tables.
  - The diamond phase peaks ~1.1 s after the head, where the clip's comes ~1.3 s after; the cycle is a
    cosine, so the timing is symmetric.
- **The lower black part is simpler.**
  - The still's cyan-rimmed blue "shield" (930–1060 x 710–840) is not drawn as such. The program shows lens
    arcs in the lower black instead.
  - The rims are smooth pale lines where the still's are striated: only the faint per-line jitter imitates
    the striation.
- **The X band's red flanks are a 50-line border band**, and the yellow X itself rolls through the fixed
  zone with the stripes. In the clip the crossing point stays nearer y 460 for a while.
- **The lens rows keep one chirp table.** The still's top rows are a little taller (0–50, 70–118) than the
  fit's.
- **Output and edges.**
  - Drawn whites come out at about 777 output luma (the shell's cap).
  - The left edge fades in over 8 px (box warm-up). The capture's dark border is not drawn.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (1440 virtual px), so the outer parts of the flame wedges are cut at the sides.
    The lens column and the figure stay whole.
  - With Twin On, the right column (x 1117–1707) loses its last ~27 vpx at the SD edge.
  - The roll speed is per field, so 50 Hz modes roll 17 % slower.
- **Nothing was cut for budget.**

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. K6 here already was the camera strength, so only its default changed: initial value 400 -> 512 (v0.1.2). The vmprog was repacked from the same bitstreams (verified: only the 150-byte settings block differs), so timing, seeds and LC are as above. Presets keep their own K6 values. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/heirloom/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023), and the camera effect at the top of the knob is 25 % stronger.

Bend / element: K6 is how much the camera's light (or the Osc lattice) heats the flame field.

The per-field gain is now (k6^2 >> 15) + (k6^2 >> 17), 0..38 (was k6^2 >> 15, 0..31): x1.25 at every position (x1.23 at the top). The camera product p5_ct is 14 bits (was 13); F = H + camera stays well inside its 13-bit range. White now heats up to 4.4 F units (was 3.6). Note: with a bright source at K6 = 100 % the flame field floods almost the whole frame, old and new alike.

Old against new at K6 = 100 % (bear, 720p / 4): `.heirloom_work/rb_crucible/k6b/k6b_cmp.png`.

Build: router1, all seed 1, worst HD Fmax 77.18 MHz, 7039 LC (max of 6). Fmax per config: HD Analog 78.88, SD Analog 77.47, HD HDMI 77.18, SD HDMI 81.08, HD Dual 78.24, SD Dual 80.85 MHz.

## v0.2.0: Ron's preview notes (2026-11-21)

Ron, from hardware: "appearance seems ok but glitchy and not fun, try again".

### What was glitchy
- **Not the HD HDMI placement.** The screen of the shipped v0.1.3 netlist found the shipped HD HDMI bitstream (router1 seed 1) at HSYNC +1.18 ns / data <= -0.66 ns, inside the good window. (The NEW netlist's seed 1 is bad, so v0.2.0 ships a re-seeded HD HDMI: see the build line.)
- **The per-line randomness re-randomised every field.** The tear / jitter / yellow-streak source was a 16-bit LFSR stepped once per line and never reset. The Tear Off "striation" (1 line in 8 shifted by 1 to 4.5 vpx) therefore landed on different lines every field, so every vertical and diagonal edge buzzed at 60 Hz: the figure, the lens column, the flame wedges and the X. The yellow-index streaks (+-1 palette step) flickered the same way. Tear On re-tore a third of the frame to new places every field.
- **Camera pixel noise into hard keys.** At the K6 = 100 % default the camera adds 4.75 Q8 F units per 8-bit luma code. The H ramp rises only 0.375 Q8 per pixel, so +-1 code of source noise moved a key edge by about +-12 px, pixel by pixel. Every flame edge sparkled on a live source. Clean-still sims can't show this.
- Minor: the 2 x 2 Bayer phase of the violet bands toggled per line from power-up and was never reset. A source with an odd active-line count per field would flip the dither every field (30 Hz checker); 1080p has an even count, so this is insurance. The tear-burst counter also carried across the field boundary.
- Checked and clean:
  - The line engine finishes in 217 clocks and its results load at the next line start, so it can't race the beam.
  - All per-field constants change in vblank. The switches are field-latched and all seven knobs glide. There are no stepped knobs or zone hysteresis.
  - The box rings carry `no_rw_check` and the running-sum resync.
  - The line buffer never collides. The generated avid's parity vote has no deadband latch.

### What changed
1. **Rolling-frame randomness (glitch fix, S10 both ways).** A 1024 x 16 random ROM (fixed seed, generated in crucible.py, 4 EBR) replaces the LFSR.
   - It is indexed by the line's position in the rolling stripe frame: (virtual line - roll lines) >> 4, 1024-line period.
   - A per-field accumulator `c_rl` adds the roll in Q4 virtual lines, s^2 x 3/4 / 1024 per field, the same law as the stripe phase. It tracks the stripes to within ~0.6 %, about 1 line/s of slip at the default speed.
   - Tear Off: the striation now sits on the flame bands and travels down with them, the same 1 line in 8 at 1 to 4.5 vpx. A still frame looks as before; in motion nothing fizzes.
   - Tear On: the tear bursts (8 to 39 lines, 40 to 198 vpx) also ride the bands. The frame stays torn a third or more, but the tears roll with the fire instead of jumping every field.
   - With K2 at the centre (still), the tears freeze.
   - Burst state and the Bayer phase reset at every field start.
2. **Camera smoothing (glitch fix, K6).** The camera luma passes a 1/8-per-pixel horizontal IIR before the K6 gain. Noise drops by about 3.9x, so edge jitter falls from about +-12 px to +-3 px. The delay is about 7 px, invisible.
   - The camera still heats the flames exactly as much as before: same gain and law, K6 default 100 %.
3. **P12 Heat above the photo is a designed climax (fun).** From 0 to 546 nothing changes: cold, then the photo, bit-identical structure. Above 546 two things grow linearly.
   - **The flame swing.** The V oscillator W is scaled by g = 1 + (P12 - 546) / 256 in all four thresholds:
     - x1.60 at 700, x2.38 at 900, x2.86 at 100 %.
     - The old top was a flat flood: red everywhere outside, yellow everywhere in the zone. The motion vanished there.
     - Now the crests burn red / yellow as hard as before, and the troughs stay cold violet / black. The stripes keep rolling as alternating fire and cold bands right up to 100 %.
     - The heat offset is unchanged: still 0.75 P12 - 410.
   - **The bend.** The V oscillator also offsets the H ramp's line position by W (g - 1) / 4: +-20 vpx at 700, +-44 at 900, +-59 vpx at 100 %.
     - The figure, the lens column and the wedges snake sideways in an S-wave, one swing per stripe period, travelling down with the bands. In the analog patch this would be the V oscillator cross-patched into the H ramp.
     - It adds to the S10 tear offset.
   - 100 % = a roaring, snaking fire.
4. **Timing / fit (no visible change).**
   - The lens-breathing cone (`23 |sn| >> 8` in one sequencer clock) was the HD HDMI critical path at 94.6 % LC: the first v0.2.0 build missed seeds 1 and 2 at about 71.6 MHz.
   - It is now split over three clocks as `3 |sn| >> 5`: lens breathing +-9.4 % (was +-9.0 %).
   - The line buffer carries `no_rw_check`; the read trails the write by >= 3 entries. yosys transparency emulation is now 0 (was 1).
   - The line engine was renumbered so the threshold and lens-DDA states run after the swing product (217 clocks, still fits SD's 858).

### Per control
- K1 to K6, S7 to S9, S11: unchanged in meaning and range.
- K6 is now smoothed (above).
- S10: both states now travel with the bands (above).
- P12: 0 to 546 unchanged; above 546, swing x1 to x2.86 and bend 0 to +-59 vpx (above).
- The HW-captured / TOML defaults are untouched; no control changed meaning.
- **Presets.**
  - "Molten" (P12 980) becomes a roaring, snaking fire (swing x2.70, bend +-54 vpx) rather than the flat flood; the name still fits.
  - "Diamond" (P12 620) gets a slight swing x1.29 and +-9 vpx of bend.
  - The rest sit at the photo's heat and are unchanged.

### Judgement calls
- Tear On no longer re-randomises every field. It is still a big, obvious tear, but now a rolling one; with K2 still, it freezes.
- The striation is now fixed to the bands rather than to the screen. Static screen-fixed jitter rows would read like the known HDMI "hsync lines".
- The P12 climax keeps the old heat offset and adds swing and bend, so the top is "roaring" rather than "flooded".
- Lens breathing went from 23/256 to 3/32 (+-9.0 to +-9.4 %) for timing.

### Resolutions
- The rolling frame, the swing and the bend are in virtual coordinates, so SD, 720p and 1080p show the same picture.
- The camera IIR works in output pixels (8 / 12 / 16 vpx of smoothing at 1080 / 720 / SD), so the camera's heat is a touch softer at SD. That is invisible at the flames' scale.
- As before, SD shows the centre 4:3, and 50 Hz modes roll 17 % slower.

### Build
- **v0.2.0 built 2026-11-21 through the shared queue with ROUTER1, all six configs on seed 1** (log `.heirloom_work/rb_crucible/build_v0.2.0.log`). Ron flashed this build (out/ vmprog dated 02:50).
  - Post-route Fmax: HD Analog 87.51, SD Analog 90.76, HD HDMI 86.63, SD HDMI 81.71, HD Dual 80.80, SD Dual 80.78 MHz (SD Fmin 27).
  - 7,182–7,215 / 7,680 LC (93.5–93.9 %), 25 / 32 EBR (+4 for the random ROM).
  - iceunpack OK x6; dupin.py 0 duplicated pins; yosys transparency emulation 0.
  - Was v0.1.3: worst HD 77.18 MHz, 7,039 LC, 21 EBR.
- The first v0.2.0 build (before the breathing-cone split) reached 7,267 LC. HD Analog passed at 75.03 MHz, but HD HDMI missed seeds 1 and 2 at about 71.6 MHz, so I stopped it. Log: `build_v0.2.0_try1.log`.
- **HD HDMI placement (skew), NOT re-installed.**
  - The shipped v0.1.3 HD HDMI (router1 s1) was fine: +1.18 / <= -0.66 ns.
  - The v0.2.0 builder's HD HDMI (router1 s1, the flashed one) sits at HSYNC **-0.71** / data <= -0.31 ns. That is the known red/blue horizontal-line territory.
  - Screen of the v0.2.0 netlist: cut off by a session end after 89 HD HDMI routes. 50 were timing-clean and 4 fell in the window. HD Analog / HD Dual were not screened.
  - Ron had already flashed the seed-1 build and closed the rework, so out/ was left as flashed. A copy is in `.heirloom_work/rb_crucible/crucible_v020_flashed_hdhdmi_s1.vmprog`.
  - Ready re-seeds, identical except the HD HDMI bitstream:
    - `.heirloom_work/rb_crucible/skew/crucible_v020_spare_hd_hdmi_router1_s2.vmprog`: **router1 s2, +2.02 / <= -0.89 ns, 83.52 MHz**, the pick.
    - `...router1_s38.vmprog`: router1 s38, +1.18 / <= -1.26, 89.68 MHz.
    - Flash s2 if HD HDMI shows thin red/blue lines.
  - To make it the deliverable: `.heirloom_work/skew_tools/install.sh crucible router1 2`.
  - A full builder rebuild re-routes seed 1, so the fix must then be re-installed (or rebuilt with that seed).
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh crucible`.

## v0.2.1: HD HDMI re-seed made the deliverable (2026-11-21)

Ron flashed v0.2.0 and no longer marks crucible as rework. Its HD HDMI bitstream (builder router1 s1) sat at HSYNC
**−0.71 ns** (data up to −0.31), the known-bad side, so the re-seeded placement is now installed in the build and the
deliverable: **HD HDMI = router1 s2 (+2.02 / ≤ −0.89, 83.52 MHz)** (`.heirloom_work/skew_tools/install.sh crucible
router1 2`). Same RTL and TOML (version only). The flashed v0.2.0 vmprog is kept at
`.heirloom_work/rb_crucible/crucible_v020_flashed_hdhdmi_s1.vmprog`; the router1 s38 spare (+1.18 / ≤ −1.26) stays in
`skew/`. HD Analog / HD Dual were not screened (the screen was cut off) and keep the builder's seed 1. A full builder
rebuild re-routes HD HDMI on seed 1: re-run the install afterwards.

## v0.2.2: GHDL integer-minus-signed fix (2026-11-21)

This toolchain's GHDL synthesis builds `<integer> - <signed>` reversed (`N - x` becomes `x - N`); the GHDL simulator
is correct, so every sim showed the intended picture while the hardware did not. Audit list:
`.heirloom_work/ghdl_sub_check/fix/lists/crucible.txt`.

**Sites fixed** (`to_signed(N, w) - x`, every other operand and width unchanged, so the hardware now computes what
the simulator always did):
- **Line 542, K5 Lenses lower half** (lens threshold th, which sets the lens column's size):
  `166 - v_d/2 - v_d/8` had built as `v_d/2 - 166 - v_d/8`.
  - **Was on hardware:** just below 512 the lenses snapped to slivers. Turning K5 further down grew them back, to
    th +26 at 0 (smaller than the photo), so the lower half ran backwards with a jump at the centre. The upper half
    was right.
  - The default 512 sat in the correct branch. But the K5 glide's ±2-count deadband could settle at 510/511 and show
    slivers in place of the photo's lenses.
  - **Now:** the lenses shrink smoothly from the photo's size at 512 to slivers at 0. The default is robust.
- **Line 783, S8 Flip** (the figure / lens row coordinate vf = 1030 − virtual line): had built as line − 1030, which
  clamped to row 0.
  - **Was on hardware:** Flip On wiped the figure. The figure zone, border and lens row stayed at row 0's geometry
    down the frame, so the black figure vanished and the lens column became one full-height bar. Only the flame
    stripes still rolled.
  - **Now:** Flip turns the figure upside down (diamond on top), and the lens chirp flips with it.
  - The "Torn Flip" preset changes accordingly. On hardware it had been the wiped picture.

**Remaps:** none. K5's default is 512 and no preset sets K5 below 512. Flip defaults Off.

**Left as-is on purpose** (part of Ron's approved look): line 639 (K4 Morph) and lines 948 / 952 / 954 (the P12
Heat thresholds). `scan.py` still lists these four.

**Build:** router1, all six on seed 1 (log `.heirloom_work/ghdl_sub_check/fix/logs/crucible.log`).
- Builder Fmax (MHz): HD Analog 84.25, SD Analog 89.23, HD HDMI 85.40, SD HDMI 86.68, HD Dual 84.73, SD Dual 94.73.
- 7,190–7,217 / 7,680 LC (94 %), 25 / 32 EBR. iceunpack OK ×6.
- **Shipped HD placements** (hand-installed, see skew below):
  - HD HDMI router1 s53, 80.35 MHz.
  - HD Analog router1 s2, 85.11 MHz.
  - HD Dual builder s1, 84.73 MHz.
  - Worst shipped HD: 80.35 MHz.

**HD HDMI skew.**
- v0.2.1 shipped router1 s2 (+2.02 / ≤ −0.89) on the old netlist. The v0.2.2 builder's HD HDMI router1 s1 sat at
  **−1.70** / ≤ −1.52, a negative HSYNC, so it could not ship.
- **Screen:** hd_hdmi router1 + router2 seeds 1–60, 113 routed (7 router2 timeouts), 66 timing-clean, 4 in the window.
- **Installed HD HDMI router1 s53: +1.37 / ≤ −1.03, 80.35 MHz.**
- **Spare:** router1 s55 (+1.08 / ≤ −1.08, 87.83 MHz) in
  `.heirloom_work/rb_crucible/skew/crucible_v022_spare_hd_hdmi_router1_s55.vmprog`.
  - Also in the window: router1 s13 (+1.22 / ≤ −0.62) and router2 s15 (+1.05 / ≤ −0.73, 75.68 MHz).
- **HD Analog:** s1 (+2.24 / ≤ +0.03, data too late) → **router1 s2 (−1.29 / ≤ −2.22)**.
- **HD Dual:** s1 (−0.72 / ≤ −1.03) kept.
- **Housekeeping:** the v0.2.0 screen and its spares moved to `.heirloom_work/rb_crucible/skew_v020/`. The v0.2.1
  VHDL, TOML and vmprog are in `.heirloom_work/rb_crucible/crucible_v021.*`.
- **Rebuilding:** a builder rebuild re-routes seed 1. Re-run `install.sh crucible router1 53` and
  `install.sh crucible router1 2 hd_analog` afterwards.
