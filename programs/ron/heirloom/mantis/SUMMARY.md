# Mantis — a green / purple / white kaleidoscope insect with colour flowing down through it

**v0.1.1 built 2026-11-10, NOT flashed.** v0.1.1: the default Etch is a quarter of v0.1.0's bend (K6 104, about 8 px), so the
wing edges stay clean curves over a live camera. The Osc lattice is fed x16 instead of x4, so the Osc flip at the default is
unchanged.

Heirloom from the video clips (phase 3b), made from the still `photos/vid/WIN_20230906_22_43_27_t2p5.png` (clip
WIN_20230906_22_43_27, 11.5 s, frame at 2.5 s; the same patch at 0.5 s and 11.0 s).
- A kaleidoscope insect, mirrored top to bottom.
- Two leaves per quadrant make an X of wings.
- Dark "<" chevrons meet at the wing tips: an arm from each side edge plus a shadow from the top edge.
- Pairs of pillars stand behind it.
- Lime / white lobes hang between the pillars under the arms.
- Wood-grain contours fill the bands near the centre row.

In the clip nothing moves but the colour: the bands slide down through the static insect, one cycle per 8.93 s. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_mantis/`:
- `notes.md`;
- `harm.py`, `levels.py` and `psifit.py` (the mechanism);
- `hwm.py`, `hwfit.py` and `calib*.py` (the element fit);
- `imodel.py` (the integer model);
- `fr/` (the clip at 10 fps).

The reading:
- **Mechanism.** Every pixel's colour over the clip is three soft-keyed pulses, one per RGB channel:
  `channel_c = clip(0.5 + g_c (cos(2 pi (psi_c(y) - t / T)) + L_c(x, y)))`, g = 0.7 / 0.7 / 1.0.
  - With the levels measured per pixel from the clip, this model reproduces the still at 15 mean |RGB| (`oracle.png`).
  - **psi_c(y) depends on the row only.** The 1st-harmonic time phase is identical across all columns.
    - It fits `a_c + m_c y + A_c w(frac((y - 150 - off_c) / 371.5))` to 0.011 cycles rms.
    - w is a skewed triangle with an 81 % rise; off_G is half a period, and R, B are 0.
    - A: R 0.290, G 0.317, B -0.201. m: R -0.00046, G -0.00045, B +0.00223 cycles per line.
    - In patch terms: a vertical oscillator at 2.9 cycles per frame, patched into the three colour keyers at different gains.
  - **L_c(x, y) is the static shape signal.** Its top/bottom mirror about row 518 is exact (residual variance 0.02 of
    0.34-0.48). Left/right it is only approximately mirrored, about x 1075.
- **Elements** (1920 x 1080 coordinates, X = |x - 1075|, Y = |y - 518|):
  - **Wings.**
    - The outer leaf runs along X = 260 + 0.615 (Y - 190), a lens of half-width 68 over Y 190-600.
    - The inner leaf runs along X = 88 + 0.779 (Y - 10), half-width 62 over Y 10-240.
    - The shadow runs along X = 375 + 0.806 (Y - 183) and grows from nothing at the tip to 55 at the top edge.
  - **Arms.**
    - The top edge is at y 142 / 214 at x 40 / 460, and the bottom edge at y 214 / 302 at x 40 / 580 (slopes 0.157 and
      0.171).
    - The arm meets the shadow in a sharp tip at about (700, 335) / (1290, 330).
  - **Pillars.**
    - Pairs at x = 120 + 490 k and 280 + 490 k (120, 280, 610, 770, 1100, 1260, 1590, 1750), symmetric about x 935.
    - Wedge-shaped: half-width 16 px, plus 0.10 per line beyond Y 300.
    - Cut by the arms, and shown only where Y > 0.25 X - 50.
  - **Lobes.** Their lower edges are parabolic sags between the pillars, sag = 0.00405 u (gap - u): 110 px in a 330 px gap
    and 26 px in a 160 px gap.
  - **Wood grain.** About 30 px spacing, following the same arcs.
  - **Eyes.** Magenta blobs at both screen edges on the centre row.
- **Palette** (palfit k = 12): pink-white 13.7 %, magenta 11.3 %, violet 10.6 %, green 10.4 %, dark green 10.0 %,
  black 9.6 %, lime 7.7 %, navy 7.4 %, indigo 5.1 %, cream 4.9 %, grey 4.7 %, lilac 4.5 %.
- **Texture.** The whole frame is soft (a camera on a CRT): roughly 9-15 px horizontal blur.

## Motion in the clip
- **Static geometry.** The gradient cross-correlation between the m-frames peaks at (0, 0) with r 0.85-0.88. The alts at
  0.5 s and 11.0 s show the same shapes.
- **The colour repeats every 8.93 s.** The mean frame difference is 14 at a lag of 8.9 s, against 119 at 4.5 s.
  - Within each 371-line segment of the vertical oscillator the bands slide down, at about 32 lines/s on the row profiles.
  - Across segment boundaries the phase steps back. In the time slits this shows as stair-stepped diagonal stripes.
- **In the program.**
  - One flow phase tau advances every field. At K4 = 687 that is one cycle per 8.92 s.
  - The integer model at -0.5 / +0.5 / +1.0 / +1.5 s against the m-frames is in `cmp_motion.png`.
  - Power-up starts at the still's phase (2.5 s into the cycle).

## What it does
1. **Line engine** (per line, for the next line):
   - the three vertical waves V_c = cos(2 pi theta_c), from a 512-entry cosine ROM, with
     theta_c = a_c - tau + m_c y + A_c w(u). The skewed triangle w comes from u (an accumulator) on the serial multiplier;
   - the row table r_c(Y): 70 entries in eighths, interpolated on the multiplier;
   - the unscaled wing geometry: the leaf and shadow edges (axis +- half-width) and the pillar half-width.
2. **Beam.**
   - x and the folds: X = |x - 1075|, or for Twin |x - 480| / |x - 1440|.
   - Etch: X += luma x gain, saturating at 2047. The camera, or the Osc lattice x16.
   - The wings are drawn in X / kw (P12), from one 11 x 10 multiply per pixel.
   - The pillar phase (period 490 px x 2^((k1 - 512)/256)) reads a 512-entry ROM of {distance to the nearest pillar,
     lobe sag}.
3. **Keys.** Each of 10 elements gets a 2-bit soft key, (d >= -8) + (d >= 0) + (d >= 8), computed from the bits of the
   inside-distance d:
   - outer leaf, inner leaf;
   - above-arm, arm edge;
   - cap, eye;
   - lobe, pillars;
   - body.

   The dark chevron (shadow + arm) is structural: it shapes "body" and the grain gate.
4. **Levels.**
   - L_c = col_c(X) + the pair weights + grain.
   - The pair weights come from five 16-entry ROMs: W_a alpha_a / 3 + W_b alpha_b / 3 per channel, Q6.
   - The grain is sin(2 pi field / 32) x gate x depth. The field is S = Y + sag outside the shadow, and X in the body.
   - z_c = (V_c + r_c) +- L_c. Invert subtracts; Key multiplies by 4. z is clipped to +-2.
5. **Tone.**
   - Three 256-entry ROMs give each channel's {Y, Cr, Cb} contribution: a soft knee, R / G / B maxima 250 / 228 / 250, luma
     pre-scaled 16/13 and capped at 940.
   - Hue rotates which z feeds which ROM.
6. **Output.**
   - Y, U and V go through [1 2 2 2 2 2 2 2 1]/16: two 8-sums over a block-RAM delay ring, with the running-sum resync.
     This gives the photo's softness and is also the HD HDMI bar fix.
   - Then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Pillars | the pillar pairs' spacing: x0.25 (about 16 pairs) .. 490 px, the clip (512) .. x4 (one pair); the lobes and grain arcs follow |
| K2 Arms | the arms' height: dropped 200 px onto the centre row (0) .. the clip (512) .. raised 200 px toward the top / bottom edges (1023) |
| K3 Bands | the vertical oscillator's period: 1100 lines, one colour segment (0) .. 371.5 lines, 2.9 segments, the clip (512) .. 126 lines, 8.6 thin bands (1023) |
| K4 Flow | bipolar colour flow, +-8 deadband, square law. 687 = the clip: one cycle per 8.9 s, bands sliding down. Full right is about 1 cycle/s; left of centre the bands rise |
| K5 Grain | wood-grain contour depth: none (0) .. the clip (512) .. x3.9, deep black contours (1023) |
| K6 Etch | how far the camera (or the Osc lattice) pushes the shapes sideways: 0 .. 89 px per full luma, linear. 104 (the default) is about 8 px: the wings bend gently around the picture and keep clean curves. 1023 bends strongly |
| S7 Twin | one mantis / two side by side (folds at 480 and 1440) |
| S8 Hue | RGB / GBR: the keyers rotated (green becomes blue, magenta becomes yellow, navy becomes red) |
| S9 Key | Soft (the clip) / Hard: keyer gain x4, posterised neon edges |
| S10 Invert | the shape signal inverted, so the insect's light and dark parts swap (a dark green night version) |
| S11 Source | Video / Osc |
| P12 WINGS | the wings' span (x0.35 .. x2.8): folded into a thin X along the spine, the arms reaching the centre (0 %) .. the clip (50 %) .. wings spread almost horizontally across the frame (100 %) |

## Video
- **Video mode (the default).** The camera luma, (Y >> 2) - 16, pushes X sideways by luma x Etch. This is how an analog
  artist would patch luma into the horizontal ramp.
  - The wings, arms, lobes and column table bend around the picture. The pillars and the colour bands stay put.
  - At the default Etch (104, about 8 px) the wing edges stay clean curves (selfie render in `cmp_final.png`).
  - With a black input, Video mode is exactly the still.
- **Osc mode.** The drifting 240 x 180 virtual-px triangle lattice replaces the camera at 16x the gain.
  - At the default Etch that is up to about 140 px. The wings crinkle into S-curves and the whole insect wobbles.
  - Etch stays live. The push saturates at the screen edge.

## Engine
- **Per-field sequencer** (vblank zone):
  - one serial shift-add multiplier, 8 slots: the pillar step, per-pixel step and start phase; 2^24 / P, the per-line step
    and the line-0 phase; flow squared; etch;
  - an exp2 ROM for Wings, Pillars and Bands;
  - the shared glide (K1, K2, K3, K5, K6, P12);
  - the flow phase, stepped at zc 20.

  The line-0 kick is at zc 300.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 330 clocks (SD's line is 858).
  - 19 serial products on a 16-iteration multiplier.
  - Every control is decoded a clock ahead into registers: the multiplier load and operand select, the theta operand, and the
    constants of the shared operand units.
  - Shared units: |Y - K|, the w(u) operand, the row difference, and the theta accumulator, with its operand captured in the
    state the product is ready.
- **Pixel path (21 stages).**
  1. Beam.
  2. Folds; pillar ROM address; etch product.
  3. X + etch (saturating); wobble.
  4. X products; the X / kw partials; pillar ROM fabric copy.
  5. X / kw; the Y-side distances.
  6. All keys (leaf and shadow both edges, arm, lobe, cap, eye, pillar, arm edge); grain sine address.
  7. Arm, lobe, above, arm edge.
  8. Dark, pillars, the largest keys; column ROM address.
  9. Body, grain gate, pair ROMs 0-3.
  10. Pair 4; first sums; grain x gate.
  11. Shape sums; grain x depth.
  12. L.
  13. z, Key, clip, Hue: the tone addresses.
  14. Tone ROMs.
  15. Fabric copies.
  16. Y / Cr / Cb sums.
  17. Cap, clamp, compose.
  18. Box sums.
  19. Box output.
  20. Luma 13/16.
  21. Gate.

  One latency, C_LAT 20, in every mode.
- **EBR (17):**
  - tone 3 x 256 x 32 (6);
  - box delay ring 256 x 32 (2);
  - column table 128 x 32 (2);
  - row table 128 x 32 (2);
  - pillar 512 x 16 (2);
  - exp2, cosine and sine (1 each).
- **Contracts.**
  - The generated avid; one latency in every mode; the blanking gate on the latency-aligned generated avid.
  - The output conditioning, with the running-sum resync.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (u = Cr, v = Cb).
- **Verified.**
  - GHDL 720p60 at decimation 1, black input, Flow stopped: **bit-exact with `imodel.py`** on every pixel outside the box
    filter's 24 left / 8 right edge columns.
  - A Flow-at-max capture is bit-exact with the model one field step later, and it moves: 19 mean |YUV| from the still-phase
    frame.
  - The integer model has 0 single-row runs of 24 px or more at the defaults and at P12 0 / 100, at 1080 and 720p
    (`thincheck.py`).
  - `sheet.png` shows every control at min / default / max.

## Status
- v0.1.1 built 2026-11-10 with router2 (the builder default).

  | Config | Seed | Fmax (post-route) | LC |
  |---|---|---|---|
  | HD Analog | **2** (seed 1 missed) | 77.94 MHz | 7310 |
  | SD Analog | **2** (seed 1 stalled the router) | 74.80 MHz | 7302 |
  | HD HDMI | **2** (seed 1 missed) | 76.17 MHz | 7309 |
  | SD HDMI | 1 | 73.33 MHz | 7302 |
  | HD Dual | **2** (seed 1 missed) | 78.06 MHz | 7311 |
  | SD Dual | 1 | 72.39 MHz | 7313 |

  LC is 7302-7313 / 7680 (95 %), with 17 / 32 EBR. iceunpack passes on all six bitstreams.
- Output: `out/rev_b/ron/heirloom/mantis.vmprog`. **Not flashed.**
- v0.1.0 (the same day) had all six configs on seed 1 (HD Analog 75.09, HD HDMI 76.45, HD Dual 82.47 MHz).
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh mantis` (router2).
  - A router1 preview (hd_analog, seed 1) gave 73.8 MHz against router2's 75.1, so stay on router2.
  - `dupin.py` reports 0 `CARRY I0==I1`.
  - HD margins are seed-dependent: let the builder's seed retries run before restructuring.
  - The critical path is the line engine's theta-operand mux: 4.3 ns logic and 9 ns routing, i.e. routing-bound from the
    decoded controls' fanout.
- **Fit history.**
  - The first cut packed to 10,209 LC (132 %): 13 elements with 3 x 13 weight lookups, a body-grain multiplier and
    per-field wing constants.
  - It came down to 7.3 k LC through: bit-logic soft keys; 16-entry pair ROMs; the wings drawn in X / kw instead of 14
    per-field constants; the box delay in block RAM; a shared-datapath line engine; and cutting the spine, stripe and node
    elements.

## Deviations
- **Fidelity.** The replica is a fitted element model, not the camera-feedback image: 32 mean |RGB| against the still. The
  4 x 4 region means are within 2-21.
  - The leaves are straight-axis lenses (the still's are slightly curved).
  - The wood grain is regular arcs, and vertical lines in the body; the still's grain is irregular and fans out.
  - The magenta stripe crossing the outer leaf, the purple spine and the pink pillar "nodes" were cut for LC.
- **The still is softer.** It is blurred vertically too. The program's horizontal edges have 8-px soft-key ramps, and its
  vertical edges get the 9-tap output box.
- **Colours.**
  - White and pink-white are capped by the shell's luma cap (about 776 output).
  - Some yellow band edges near the bottom sit a few rows lower than in the still.
- **Key mode** (S9 Hard, x4 gain) can show short 1-row lines, up to about 200 px at 720p, where a horizontal contour of the
  keyer input touches the threshold. The defaults and P12 0 / 100 are clean.
- **Etch.**
  - At the default (about 8 px) the wings bend cleanly.
  - At K6 max, row-to-row luma changes in a detailed camera image still fray the edges somewhat.
  - No luma box was added: it didn't help at the default and the LC budget was full.
- **Resolutions.**
  - 720p shows the same geometry as 1080.
  - SD shows the centre 4:3, so the outermost pillar pairs and the edge eyes fall outside the frame.
  - The output blur is per output pixel, so SD is relatively softer.
