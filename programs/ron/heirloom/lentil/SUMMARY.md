# Lentil — blue lenses pinched in a band, pink saw stripes flung out of each one

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the clip
`WIN_20230501_01_34_42` at 0.5 s (`photos/vid/WIN_20230501_01_34_42_t0p5.png`). A band of solid blue
lumpy lenses across the middle of the frame, pinched to a point between them; the pink hourglass gaps are
filled with purple → pink → yellow sawtooth stripes that leave each lens parallel to its edge and then fan
out flat; above and below the band the same stripes run free as ")" chevrons. The patch breathes at 1.6 Hz.
Presets are hand-written (two of them are the clip's other looks).

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_lentil/` (`notes.md` has every number).
- **Layout.** Top band y 0..146 and bottom band y 878..1075 hold free stripes; the lens band is y 160..866,
  centred on y = 518. Blue (RGB 3, 0, 177, std 2–5) is 34.9 % of the frame.
- **Lenses.** Period 940 px: centres at x 68, 1008, 1948, pinches at x 540 and 1478 (gap 56–60 px on the
  centre row). The half-height A(d) as a function of the distance d from the pinch is the same on all four
  half-lenses (left/right symmetric), but the top and bottom halves differ: top 0 (d < 36), 24 (60),
  113 (120), 152 (180), 184 (240), 232 (300), 297 (360), 334 (420), 354 (450); bottom 0, 56, 103, 138, 182,
  256, 291, 321, 341. That gives the lumpy S edges, about four ripples per period. The lens fat middles touch
  the band edges.
- **Stripes.** A sawtooth colourised from the still's own colours: the G channel ramps 56 → 233 over 0.1 → 0.89
  of a cycle and falls back over the last 0.11, R clips at 255 above G ≈ 100, B runs 160 → 136. In palette
  order: (88, 7, 166) → (183, 47, 157) → (219, 68, 155) → (241, 85, 152) → (253, 107, 151) → (254, 132, 148) →
  (253, 158, 145) → (252, 183, 143) → (250, 207, 140) → (247, 230, 136).
- **Chevrons.** Top-band period 308.75 px. The per-row phase is a rounded V around y = 58 (0.20 cycle at
  y 2, 0.064 at 58, 0.262 at 142); the bottom band mirrors it around y = 996. The swing grows with x
  (0.09 cycle at x 244, 0.29 at x 1676), so this is frequency modulation, not a phase offset:
  f = f0 (1 + 1.14e-3 · ||y − 527| − 469|), with the tip rounded over 35 lines.
- **Hourglass stripes.** The local frequency is the base f0 everywhere except the first ~100–150 px after a
  lens's right edge, where it is 2.5–3.5× and decays (a lag). The colour just outside every right edge is the
  same dark purple on every row (s ≈ 0.1 at 10 px). Neither a sum of H and V waveforms (residual 0.85) nor a
  per-row linear phase fits. The FM model was chosen by eye on zoomed comparisons, because every variant
  scored the same phase coherence (~0.15–0.18).
- **Band edges.** Above the band there is a 14-line strip of shifted, slanted stripes (y 146..161), and a
  2-line dashed line where the band meets it; below the band the strip is 8 lines.

## Motion in the clip
The clip was re-extracted at 10 fps over all 9 s (`seq/`, `motion2.py`).
- **One breathing, period 0.628 s (1.59 Hz).** It repeats exactly: the frame difference is 1–7 at whole
  periods and 26–30 at half periods. Nothing else drifts, and the lens x positions are fixed.
- **Per cycle:**
  - the pinch gap goes 0 → 64–72 px; it is closed for about 40 % of the cycle, as a blue bar across the
    pinch;
  - the band edges move together: top 144..168, bottom 868..888. That is one vertical threshold offset of
    −8..+16 lines, 0 at the still;
  - the top-band phase at y 58 swings 0.00..0.28 cycle, because the chevron tip follows the same offset.
- **The other looks of this patch** (`01_32_55`, `01_35_56`) show Ron changing:
  - the stripe frequency (a period of 480 px in one, 120 px in the other);
  - the band height (y 340..700 in `01_32_55`);
  - the lens shape (round in `01_32_55`);
  - the chevrons (wavy in `01_32_55`).

  The lens period is the same in all three.
- **What the program does with it.** Breath runs at 1.59 Hz at its default. It drives the band offset with
  12 · sin (a parabola LFO), between −8 and +16 lines. The lens threshold, the band edges and the chevron tip
  all follow it, so the pinches close and open as in the clip. At power-up the program starts at the still's
  phase.

## What it does
1. **Lens.** e is the position within the lens period (a 24-bit DDA). For each row, W is the lens half-width
   from the *inverse* lens profile ROM, looked up at v = |y − 518| / σ − c. A pixel is blue where |e| < W
   inside the band. Rows with v < 0 are blue all the way across (the pinch closed).
2. **Stripe VCO.** f(y) = fs (1 + 1.14e-3 hyp(u)), with u = ||y − 527| − 469| (Bow) or a 64-line zigzag (Wave),
   from a ROM.
   - Outside the band, the phase runs free from a 390 px pre-roll.
   - Inside the band, the phase is re-seeded at every lens exit to 0.05 + Fan · 2W (cycles). A lag tail
     Gt·e^(−d/32 px) is added to f, so the stripes leave each lens parallel to its edge and then fan out flat.
   - The line-start phase is computed from the distance to the last exit, so the leftmost hourglass matches.
3. **Colour.** A 256-entry saw palette in the still's colours. A 26 px dark-purple fringe ramp (7/8 → 0)
   sits outside each lens, and the lens fill is blue. With Ghost, the hidden fast stripes inside the lens show
   through at 3/8.
4. **Band edges.** A 14 / 8-line strip outside the band gets a 0.15 (0.5 + x/W) cycle phase shift and slant.
   The 3 lines just inside the band edge flip phase by half a cycle and drop the lens: the dashed line.
5. **Camera.** Luma frequency-modulates the VCO.
6. **Output.** Two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V (the still's softness and the HD HDMI bar
   fix), then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Stripes | the stripe density, everywhere, x1/4 .. x4 (exponential, 512 = the still's 309 px period) |
| K2 Fan | how hard the stripes are flung out of each lens: the exit-phase gain and lag tail, (k/512)²: 0 = stripes leave each lens as parallel echoes of its edge; 512 = the still; 1023 = 4×, dense sheets fanning out of every lens |
| K3 Lenses | the lens period: 0.6 (one giant lens) .. 6.8 lenses across (2^((512 − k)·7/8 / 256)), 2 at 512 |
| K4 Breath | the breathing rate (pinch open / close, band edges, chevron tip): 0.1 Hz .. 1.59 Hz (512, the clip) .. 6.4 Hz; never stops |
| K5 Pinch | the lens threshold, ±240 profile lines: tiny blue seeds (lentils) at 0 .. the still at 512 .. merged into a blue river with pink slits at 1023 |
| K6 Bend | how far the camera (or the Osc) bends the stripes: luma FM, 3·f0·(k/1023)² at white (384 = +~35 % frequency at white) |
| S7 Shape | Lumpy (the still's measured lens, top and bottom halves different) / Round (a smooth sine lens, the `01_32_55` look) |
| S8 Fill | Solid blue (the still) / Ghost: the fast stripes running hidden inside the lens show through, chevron-textured (the `01_35_56` look) |
| S9 Chevrons | Bow (the still's ")" chevrons) / Wave (a 64-line zigzag frequency wobble: zigzag stripes above and below, wobbling fans inside) |
| S10 Colours | Flamingo (the still: purple → pink → yellow on blue) / Lagoon (teal → aqua → lime on crimson lenses) |
| S11 Source | Video / Osc |
| P12 Band | the lens band's height σ = 2^((p − 650)/220): 0 % = a 90-line seam of tiny flattened lentils between full chevron fields; 650 = the still; 100 % = lenses taller than the frame, fanning stripes everywhere |

**Presets:**
- Still (the defaults);
- Broad Lens: the `01_32_55` look (K1 347, Round, Wave, P12 436);
- Dense Ghost: the `01_35_56` look (K1 861, Ghost);
- Seeds;
- River;
- Lagoon;
- Flutter;
- Tall Lenses.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, is added to the stripe
  VCO's frequency every pixel.
  - Bright areas crowd the stripes, and the phase they gain is carried to the right.
  - In the band, the integral restarts at every lens exit.
  - The picture shows as stripe bending and tearing. The lenses, band and chevrons keep the still's
    composition.
  - With no input, Video mode is the still.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting about 75 px/s, feeds the same path, so the
  stripes ripple gently. Bend stays live in both modes.

## Engine
- **Per-field sequencer** (vblank zone):
  - One serial shift-add multiplier, 15 slots. The slot number is decoded two clocks ahead and the operands are
    pre-muxed.
  - A shared glide (1/8 per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12.
  - A 256 × 16 exp2 ROM.
  - It computes:
    - fs;
    - the lens step, period and start phase, and |e| at the start;
    - the fringe shift;
    - Fan² → the exit gain (Fan · period) and the tail start and area;
    - Bend² → the luma gain;
    - the breathing rate and the parabola-sine;
    - σ and 1/σ (shifts of the exp2 ROM);
    - the band edges (offset · σ);
    - the lens threshold.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 90 clocks (it fits SD's
  858-clock line), using 5 serial products:
  - v = |y|/σ;
  - the profile ROM → W;
  - the chevron ROM → f(y);
  - the exit phase;
  - the distance to the last exit (or the entry);
  - the line-start phase.

  It also sets the band, strip and dashed-line flags.
- **Pixel path (15 stages):**
  - lens DDA;
  - |e|;
  - inside test and fringe distance;
  - exit / entry events, fringe weight, luma nibble products;
  - lag tail (reloaded at each exit; decays 1/32 per pixel, 1/16 at SD);
  - step;
  - phase accumulator (reloaded at exit or line start);
  - palette address, ROM, fabric register;
  - blend operands, difference, × weight;
  - compose.

  Then the output boxes, 13/16 and the gate: C_LAT 20 in every mode.
- **ROMs** (17 EBR):
  - exp2 256 × 16 (1);
  - lens profile 1024 × 16 (4);
  - chevron 1024 × 16 (4);
  - palette 512 × 32 (4);
  - line buffer 2048 × 8 (4).
- **Contracts:**
  - the generated avid;
  - one latency;
  - the blanking gate;
  - output conditioning with the running-sum resync (both boxes snap to black after 8 blank samples);
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr), generated by `lentil.py`;
  - luma pre-scaled 16/13 with a soft knee, capped at 940.
- **Verified** (GHDL 720p60, decimation 1; `cmp_final.png`):
  - Osc frame vs the integer model `imodel.py`: the middle-band rows are bit-exact, and 96.6 % of all pixels
    are within ±2 levels after the tester's 4-px capture offset (the generated avid's start). The rest is the
    Osc lattice's phase in the top and bottom bands.
  - Video (selfie) frame: 86 % within ±2 (the model's line-buffer alignment).
  - Against the still: blue 35.5 % vs 36.4 %; all 16 cells of the 4 × 4 region-mean grid within 2–5 levels;
    mean colour (151, 84, 154) vs (154, 84, 157).
  - A default-speed capture two frames later shows the pinch closed and the band edges moved (the breathing).

## Status
- **v0.1.0, 2026-11-04, router2** (`MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh lentil`). All six
  configs on seed 1.
- Post-route Fmax: HD Analog 84.14, SD Analog 76.93, HD HDMI 79.94, SD HDMI 76.58, HD Dual 81.01,
  SD Dual 79.97 MHz.
- 6910–6948 / 7680 LC (90 %), 17 / 32 EBR. iceunpack OK × 6, and the packed config carries the new labels and
  presets.
- `out/rev_b/ron/heirloom/lentil.vmprog`. **Not flashed.**
- If a later version gets tight, try `NEXTPNR_ROUTER="--router router1"` first.

## Deviations from the photo
- **Stripes.** The hourglass stripes are smoother and more regular than the still's organic S curves: about 5
  per hourglass where the still has about 4. The packing right at the lens edge is lighter. The lag-tail FM
  model was chosen by eye; no FM model fitted the phase numerically better than the others.
- **Lens profile.** It is the median of the four measured half-lenses, so the middle lens's step-shoulder at
  its top right (x ≈ 1100) is not reproduced. Lens tops meet the band edge as about 60 px flats.
- **Brightness.** Drawn yellows are duller than the still's: output peaks are about 775 (the shell's white cap)
  against the still's 836.
- **Camera texture.** The camera's blur and grain are not drawn, nor is the faint stripe texture inside the
  blue (std 2–5). Ghost shows a strong version of it on a switch.
- **Band edges.** The transition strips (a fixed phase shift and slant) and the dashed edge line (a half-cycle
  phase flip) approximate the still's band-edge glitches.
- **Breathing.** The clip's ±0.12-cycle top-band wobble comes only through the chevron tip moving, so it is
  smaller here. The breathing rate scales with the field rate: 1.59 Hz at 60 fields/s, 5/6 of that at 50.
- **Left edge.** It fades in from black over about 8 px (filter warm-up).
- **Resolutions.** 720p shows the same geometry as 1080p. **SD shows the centre 4:3** (virtual x 240..1680),
  so the outer lenses are cut at different points and the leftmost hourglass is off-screen. The dashed edge
  line is 1–2 lines tall at SD.
- Nothing was cut for budget.
