# Portal — blue rings and magenta domes around a green trapezoid portal, striped eyes, pink bars rising

**v0.1.0 built 2026-11-10, not flashed.** Heirloom (video phase), from the clip `WIN_20231121_21_36_42` at 0.5 s
(`photos/vid/WIN_20231121_21_36_42_t0p5.png`). Two mirrored tiles: magenta / red domes under a thick cyan arc, a
pink dome band with a red V at each cusp, a trapezoid "portal" running yellow → green → cyan → yellow down the
frame, an eye of leaning blue / green stripes on each side of it, pink bars across the middle with a cyan lens
lattice, and a yellow / red / magenta floor. In the clip the bars rise and multiply. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_portal/` (`notes.md`, `geo.py`, `ffit.py`, `rfit2.py`, `profit.py`, `trap.py`,
`stripes.py`, `bars.py`, `proto.py`, `proto2.py`, `imodel.py`, `sheet.py`, `thincheck.py`).
- **Tiles.** Horizontal period 946 px by autocorrelation in all three channels: 2.03 tiles across.
  - Mirror axes: x 542 / 1486 (the rings), 1022 (the cusp), 531 / 1476 (the trapezoid).
  - The trapezoid sits 13 px left of the rings. That offset is the black J-shaped gap on its right side only.
- **It is an RGB patch** (`chans.png`); each channel is a clean grayscale layer set.
  - **B, rings.** A triangle fold of one parabolic field F = |y − 576| + 7.5e-4 u², where u is the distance from
    the tile centre, folded at the cusp.
    - Label-purity fit rms 21/255. An ellipse fits worse (22.7).
    - The dark top ring rises 165 px from centre to cusp with dy = 7.3e-4 dx² all the way, so the field is
      parabolic.
  - **R, domes.** Inside the cyan arc, R = g(|y − 576| − 0.3 u) (rms 9.2).
    - Magenta-red at the top centre, fading to black at y ~210, bright again toward the floor.
    - Bright magenta outside the arc, in the corners.
  - **G, the cyan arc.** A band between y = −17 + 6e-4 u² and y = 3 + 7.5e-4 u².
    - 20 px thick at u 250, 40 px at the cusp, where it forms the V (a Λ at the bottom).
  - **The trapezoid** is a G key (G 186).
    - Half width 32 + y²/5128 about x 531: 30 px at the top, 245 at the bottom.
    - R and B pass through it, so it reads yellow at the top (R bright), green in the middle, cyan lower down
      (B rings), then yellow at the floor.
  - **Dome band.** A constant-thickness band between y = 222 + 6e-4 u² and y = 340 + 6e-4 u².
    - Pink (238, 125, 200) above y 357, ending at u ~395 (x 150 / 940).
    - Below that, a cyan-blue arc runs down into the cusp V (y 400–465).
    - A red V spans y 240–330 at each cusp.
  - **Bars.** R 240 bars at y 533–573, 648–669, 730–749 and 808–825 (x 240).
    - Straight, constant thickness, cut by a dark "pillar" around the trapezoid above y ~720.
    - Spacing (105, 82, 78, 96) fits one vertical half-cosine S(y) = 3.92 (1 − cos(π (y − 330) / 960)). The bar
      levels are 1.03 / 2.05 / 3.03 / 4.02 / 5.23.
  - **Lattice.** G 125 / B 160 lens plateaus around the bars, with blue lens holes at |x − 1022| ~200 and X
    crossings at the cusp.
  - **Eyes.**
    - B stripes: period 50 px, lean 0.2 px per line ("/" left, "\" right, Λ at the cusp), contrast 0..255.
    - G glows next to the trapezoid (~140) and fades over ~220 px.
    - The windows are rounded rectangles: y 410–690, 100–445 px from the centre.
    - Λ-shaped lower windows sit at the cusp, y ~690–940.
- **Texture.** Soft horizontal edges (10–20 px), sharp vertical ones. A few line-start glitch blocks at x < 100,
  y 680–830. No tearing.

## Motion in the clip
- 99 frames at 10 fps over the whole 9.9 s clip (`fr/`, `kymo_c0_x60.png`, `clip_sheet.png`). Strip frames 0–2 are
  the same frame: t − 1.5 s falls before the clip starts.
- The rings, trapezoid, eyes and arcs are static.
- **The bars rise and multiply.**
  - From 0.5 s to 2.5 s they move up ~50 px per 0.5 s (~100 lines/s, spacing ~80).
  - Near y ~300 they slow, fatten and merge into the dome band, and new bars appear below.
  - By 2.0 s about 6 bars fill the middle, where the still has 4.
- The kymograph sways: up ~4 s, back ~1.5 s. The oscillator's level was being swept (a hand or an LFO), not
  scrolled. The lattice moves with the bars, and the dome band thins and climbs as they crowd it.
- **The program.**
  - The bar oscillator's level breathes ±35 % on a 384-field triangle (6.4 s at 60 Hz).
  - Its phase drifts at the default Speed, 0.6 bar periods/s, so the bars stream up out of the floor, multiply and
    ease back.
  - The dome band thins at the breath's peak.
  - At power-up the frame is the still. `cmp_motion_im.png` shows the integer model at +0.5 / +1.0 / +1.5 s beside
    the clip's m-frames.

## What it does
1. **Tile fold.**
   - A Q20 tile phase is folded to u per pixel (0 at a tile centre, 511 at the cusp). A second phase, 13 px left,
     serves the trapezoid.
   - A ROM gives u²/4. The curvature is q = u² (16 + m) >> (13 − s).
2. **Rings.** F = |y − 576| + q + E and F2 = |y − 576| + 0.8 q + E, where E is the camera / Osc bend.
   - B = PB(±F), a 1024-entry table fitted from the still.
   - R = PR(|y − 576| − 0.277 u + E) inside the arc, PO(F2) outside it.
   - The cyan arc (G 150) is where F2 < 593 and F > 573.
3. **Keys.**
   - Trapezoid: G 186 where u_t < hw(y).
   - Eyes: B stripes and G glow, R off, inside the eye-edge ROM thresholds.
   - Lower windows, dome band, red V.
4. **Bars and lattice.**
   - Phase: S(y) + bph + 32 E (+ 29 q with Arches), where S is a (1 − cos) table times the level.
   - Bars: R 240 within a half width that tapers 30 % toward the tile centre.
   - Lattice: G 125 / B 160 within a wider half width, thinnest at u 292 (the lens holes) and capped at 0.40 of a
     period.
   - Whole bars are gated by their centre level (0.6 .. S(880)). The pillar cuts bars centred above y 720.
5. **Colour.** Colours (RGB rotation) and Polarity, then the BT.601 matrix as three per-channel ROMs. Luma is
   pre-scaled 16/13 and capped at 940.
6. **Output.** Chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16, then the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | the mirrored tiles: one wide portal (0, 0.7 across) .. 2.03 (512, the still) .. ~6 narrow portals (1023) |
| K2 Domes | the curvature of the rings, cyan arc, dome band and red V: flat bands (0) .. the still (512) .. deep arches and steep rings (1023), ±2 octaves |
| K3 Trapezoid | the portal's width: a thin needle (0, ×0.4) .. the still (512) .. a wide green funnel (1023, ×2.5) |
| K4 Speed | bipolar (±8 deadband, square law): the bars stream up (+) or sink (−); 700 = the clip's drift (0.6 bar/s); full = ~4.5 bars/s. The level breath runs at any setting |
| K5 Stripes | the eye stripes' density: fat soft bands (0, ×1/2.8) .. the still's 50 px (512) .. fine hatching (1023, ×2.8) |
| K6 Etch | how far the camera (or the Osc, 32× as far) bends the rings, bars and stripes: 0 .. strong (square law); 200 = a gentle bend |
| S7 Flip | Off (the still) / On: upside down, so the domes hang as bowls and the trapezoid narrows downward |
| S8 Arches | Bars (the still) / Arches: the bars and lattice bend along the dome curve into nested arches |
| S9 Colours | Magenta (the still) / Citrus: RGB rotated, giving yellow-green domes, a green V, a violet trapezoid and a red / yellow floor |
| S10 Polarity | Positive (the still) / Negative: every channel inverted |
| S11 Source | Video / Osc |
| P12 Bars | the bar oscillator's level (glided): 0 = no bars and no lattice, a clean portal; 455 = the still's four bars; 100 % = a dense climax of ~12 bar periods and lattice across the middle |

## Video
- **Video mode (the default).**
  - Camera luma, as (Y >> 2) − 16 from a half-rate line buffer, is box-filtered over 4 reads.
  - It is multiplied by the Etch gain and scaled 1/256.
  - It is added to the ring field, ×32 to the bar phase and ×16 to the stripe phase.
  - At the default Etch the rings and bars bend gently around the picture, and the composition stays readable
    (`cmp_final.png`, bottom). K6 at 1023 bends it strongly.
  - With no input, Video mode is the still.
- **Osc mode.**
  - A drifting 640 × 540 virtual-px triangle lattice, centred on its mean, feeds the same path at 32× the camera's
    gain.
  - At the default Etch the rings buckle into chevrons and the bars zigzag: an unmistakable flip.

## Engine
- **Per-field sequencer** (vblank zone).
  - One serial shift-add multiplier with 9 slots: the slot is decoded two clocks ahead and the operands are
    pre-muxed.
  - The shared glide on K1, K2, K3, K5, K6 and P12.
  - Per field it computes:
    - the tile step and start phase (1/16-octave exponent);
    - the bar level with its breath, and S at y 880 and 720;
    - the drift (Speed²), Etch² and the stripe lean;
    - the curvature and trapezoid exponents;
    - the dome breath and the bar width (the P12 fade).
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in ~175 clocks with its own serial
  multiplier (9 products):
  - yv (with Flip) and |yv − 576|;
  - the trapezoid half width, (35 + yv² × 221 / 2^20) × gain;
  - S = GT(yv − 330) × level;
  - the stripe lean;
  - the lower-window, V, dome-end and glow line constants.
- **Pixel path (20 stages):**
  1. beam;
  2. folds;
  3. luma box; trapezoid / window / V tests;
  4. u² and eye-edge ROMs to fabric; etch product;
  5. curvature partials; eye test; glow;
  6. curvature sum; stripe phase; etched distances; bar phase offset;
  7. curvature total; stripe phase; PR address;
  8. curvature shift and clamp; stripe triangle;
  9. F; q split; stripe level; PR data;
  10. q8; PB address; arch term;
  11. F2; bar phase; bar / lattice width products;
  12. arc tests; PO address; bar-centre level; dome test; PB data; widths;
  13. bar distance; band and pillar gates;
  14. bar and lattice keys;
  15. composite;
  16. Colours / Polarity; matrix addresses;
  17. matrix ROM data (EBR);
  18. matrix data to fabric;
  19. Y / Cr / Cb sums;
  20. luma 16/13 and clamps.

  Then the shell's output conditioning (stages 21–25).
- **Stores** (24 EBR):

  | Store | Shape | EBR |
  |---|---|---|
  | B profile | 1024 x 8 | 2 |
  | R profile inside the arc (top / bottom) | 1024 x 8 | 2 |
  | R profile outside the arc | 512 x 8 | 1 |
  | u² / 4 | 512 x 16 | 2 |
  | eye edges {u < 482, top, bottom} | 512 x 16 | 2 |
  | bar oscillator 1 − cos | 1024 x 16 | 4 |
  | matrix shares (R, G, B) | 3 x 256 x 32 | 6 |
  | line buffer (half rate) | 1024 x 8 | 2 |
  | chroma 8-back ring (`no_rw_check`) | 256 x 20 | 2 |
  | C_LAT sync ring (`no_rw_check`) | 256 x 3 | 1 |

- **Contracts:**
  - the generated avid;
  - one latency, C_LAT 24, in every mode;
  - virtual 1080-line coordinates (SD shows the centre 4:3);
  - colours in the hardware convention (U = Cr, V = Cb);
  - the chroma running sums snap back to neutral after 8 blank samples (resync).
- **Verified:**
  - GHDL at 720p60, decimation 1, black input: the default frame is **bit-exact with `imodel.py`** (100 % of
    pixels at fields = 3, at the 4-px output offset).
  - A Speed 1023 capture at field 5 is 99.84 % exact against the model; it predates the stripe fix, and the rest is
    that bug. It confirms the per-field drift and breath; 29 % of pixels moved.
  - The Osc and camera frames (`rtl_osc4.png`, `rtl_cam4.png`) follow the model in structure. The Osc lattice is
    not modelled to the pixel.
  - `sheet.png` (with the K6 row on the camera) is rendered from the model.
  - `thincheck.py` finds 0 single-row runs of 24 px or more at the defaults, at P12 0 and 100 %, and at every knob
    extreme and switch, at four phases, at 720p and 1080p. One exception: Tiles 0 at 720p, one 28 px run at one
    phase.
  - `dupin.py` reports 0 duplicate-input cells.
- **Bugs found by the sims:**
  - A 12-bit intermediate in the bar-level exponent wrapped, halving the level so every other bar was missing.
    Widened.
  - The stripe triangle wrapped to 0 at its crest (2048 − sph = 256 in 8 bits), leaving dark specks. It now
    saturates.

## Status
- **v0.1.0 built 2026-11-10 with ROUTER1; all six configs on seed 1. Not flashed.**
- Post-route Fmax (MHz):

  | Config | Fmax |
  |---|---|
  | HD Analog | 96.50 |
  | SD Analog | 97.68 |
  | HD HDMI | 98.22 |
  | SD HDMI | 93.95 |
  | HD Dual | 99.45 |
  | SD Dual | 95.91 |

- 6,945–6,952 / 7,680 LC (90.5 %), 24 / 32 EBR. iceunpack OK x6. The packed config carries the labels and presets.
- Output: `out/rev_b/ron/heirloom/portal.vmprog`.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh portal`.
- **Fit history.** The first full build overflowed (8138 LC, 105 %). To fit:
  - the eye edges moved into a ROM;
  - the bar and lattice widths became constant shapes computed late in the path, with the P12 fade by subtraction;
  - the phase chains were narrowed to 12 bits;
  - the camera box went from 8 taps to 4;
  - the 4-tap RGB smoothing went, and with it four idle output stages (C_LAT 32 → 24).
- Presets are hand-written. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Lattice: designed, not fitted.** Band fits of the still's lens web stalled at IoU ~0.35. The program's lattice
  is lens bands on the bar phase, thinnest at u 292, with X's at the cusp. It has the still's character (cyan web,
  blue lens holes), but not every lens is in place.
- **Bars.** Straight with a fixed taper. The still's lower bars are lens-shaped in places, and the bottom pink lens
  inside the trapezoid (y 895–930) is not drawn.
- **Dome band.** One flat pink, where the still has a pink-to-lavender gradient and rounder ends.
- **Crisper edges than the still.** The still is smeared over 10–20 px horizontally. Only the shell's chroma 9-tap
  and luma 3-tap remain; the 4-tap RGB smoothing was cut to fit.
- **Eye glow.** A linear ramp from the trapezoid, a little greener than the still's.
- **Not modelled.** The J hook at the right eye's lower-inner corner (a rounded corner instead) and the left-edge
  glitch blocks.
- **Motion.** The breath is a fixed triangle LFO, not the clip's irregular hand sweep. The clip's periodic upper
  stripe windows (~2.6 s) are not modelled.
- **Osc.** The Osc lattice is cupola's (640 × 540 virtual px), not a feature of the clip.
- **Resolutions.**
  - 720p matches 1080p.
  - SD shows the centre 4:3: the cusp in the middle, the two trapezoids at the sides, the outer halves of the eyes
    cut (the family-wide SD convention).
  - The eye edges and stripes are in tile units, so they stretch 3× with Tiles at 0 (the one thincheck exception,
    at 720p).
