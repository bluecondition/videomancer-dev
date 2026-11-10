# Tatter — white tube eyes and crosses over an orange checker, torn along a crack

**v0.1.1 built 2026-11-10, not flashed.** Heirloom (video phase), from the clip `WIN_20230424_23_47_33` at 8.0 s
(`photos/vid/WIN_20230424_23_47_33_t8p0.png`). Presets are hand-written placeholders.

**v0.1.1:** the camera's default bend is about a quarter of v0.1.0's (Etch on a square law, luma box 4 → 8 px), so
the tube edges stay clean curves on a live camera. The Osc and full Etch are as strong as before.

## Reading of the photo
Working files are in `.heirloom_work/rb_tatter/` (`notes.md`, `fitfx.py` / `fit2.py` / `scat.py` / `m*.py`
measurements, `proto.py`, `imodel.py`, `sheet.py`, `thincheck.py`).
- **A sum lattice.** Every grey contour is one field u = Fx(x) + Fy(y) through a periodic tube palette.
  - Fy is a V cosine, period **280 lines**: eye rows at y 232 / 512 / 792, bowties at 92 / 372 / 652.
  - Fx was fitted column by column (a 1-D search per x over all rows, scored on palette purity). Right of the crack:
    the eye column at **x 990** (Fx 0.90), falling to the big checker diamond (min −1.4 at x 1480, diamond x 1000..1830)
    and rising through the wavy band (x 1580..1920) to 1.75 at the edge. Left of the crack (fitted on the cleanest
    frame of the clip, 7.65 s): a different regime, the **cross column at x 510** (min −0.25) on a ~480 px period.
  - Tube palette (median colour per u bin): from the checker edge a rise to white over 0.5 units, a white plateau to
    0.96, a fall to dark green-grey (46, 60, 38) at **1.56** (one cycle), a thin orange ring (190, 108, 12), then the
    next cycle. Eye cores reach u ≈ 1.9: grey ~130 inside the ring, as in the still.
- **The checker** (u < 0).
  - Rows FM'd by the V oscillator: ρ = (y − 5.1)/23.53 + 0.94 sin(2π(y − 1)/280), 12 rows per period, 15..40 px
    tall; residuals < 0.1 row over 41 measured row edges.
  - Even rows carry an H oscillator of **100.5 px**, odd rows one of **176.5 px**; both are clipped sawtooths (slow
    rise, sharp fall) with fixed phases in every row (FFT phase 36 mod 100.5 and 106 mod 176.5 on all 41 rows).
  - Colouriser: G-high yellow (255, 184, 8), G-low red (255, 52, 50); a zone ~105 lines above each eye row turns the
    G-low blocks dark violet-grey (66, 45, 69).
- **The tear.**
  - A crack (a thin light line with dark sides) along **x = 690 + 0.00075 y²**, fitted to 20 row-disagreement points.
  - The sync-lost picture left of it: a strip x < 240 on every row (blue (9, 8, 216) / green (34, 88, 11) in the zone,
    orange (249, 109, 9) / violet (164, 10, 213) elsewhere), a **white bar x 240..740** with 90 px ramps and no V
    modulation from y ~280 down, and the same checker in those colours out to a diagonal x ≈ 340 + 1.1 (y − 280);
    ragged bands, thin white spikes and slipped bands.
- **Palette shares** (k-means, 14): white 18.9 %, orange 16.4 %, red 11.8 %, pink 7.1 %, dark green-grey 6.3 %,
  dark violet-grey 6.2 %, greys 17 %, blue 5.0 %.

## Motion in the clip
- Measured on every frame from 6.0 to 10.3 s (`frames/`, `m10.py`, kymographs `kymo.png`, `kymox.png`,
  `kymo_q.png`).
- **The lattice rises 7.9 px per frame = 238 lines/s** (one eye row every 1.18 s), with surges to 10..11 px per frame
  about every 1.2 s. Nothing moves in x: the eye column, the band and the checker columns are fixed.
- **The checker rows do not scroll.** They wobble ±20 px in place, because their FM rides the scrolling V oscillator,
  while the dark zones scroll up with the lattice.
- **The crack stays put** per row (a few px of wobble). The sync-lost picture is the long-term state, with recovery
  episodes where the lattice reappears from the top down (7.4..8.1 s, 8.7..10.3 s). The slips change every frame.
- The other candidate stills of the clip (0.5 s, 7.5 s) are the same patch at other scroll and tear moments; no knob
  turns are visible.
- **The program:**
  - K4 Scroll at 770 lifts the lattice 3.95 lines per field (237 lines/s at 60 Hz), modulated ×0.75..×1.25 on a
    1.4 s triangle. The integer model measures 48..70 virtual lines per 0.25 s.
  - The checker rows wobble in place (their FM reads the scrolling V oscillator) and the zones scroll with it.
  - The sync-lost region's top edge roves ±160 lines on a 4.3 s triangle. The tear bands are redrawn every 4 fields,
    and the crack wiggles ±5 px with the scrolling Fy.
  - `cmp_motion.png` shows the clip at 8.0 / 8.5 / 9.0 s next to the model at fields 40 / 70 / 100: the eye rows step
    half a period per 0.5 s in both.

## What it does
1. **H phase.** A beam DDA gives D = (x − 990 px)·step. A second accumulator DL carries the left-of-crack regime with
   its band slip, and freezes through a band's hold window. Both index a 2 × 1024-entry Fx table (left / right
   regime). The right regime is mirrored about the eye column for x < 700, so the untorn state is a symmetric
   eye lattice.
2. **Field.** u = Fx + Fy(line) + Eyes offset + camera, in LSB of 1/256 tube cycle. Invert gives 100 − u, and
   Contours gives 3u.
3. **Colour.** For u ≥ 0 a 512-entry tube palette is indexed by {first cycle, u mod 256}. For u < 0 the checker:
   s = the clipped sawtooth of the row's oscillator (A: 2.25 (ψ − 87), B: 3.75 (ψ − 101)), into a 512-entry
   colouriser ROM {set, zone, s}. A light red-orange fringe marks u in [−16, 0).
4. **Tear.** The crack sits at x = X0 + 49 y²/2¹⁶ + Fy/32 + band jitter, with X0 set by P12. Left of it the cross
   regime runs; the scar is drawn ±2.5 px light and 2.5..5 px dark.
   - The sync-lost set covers the strip (x < 240) and, below the roving top edge, the region left of the diagonal
     340 + 1.094 (y − top) + band offset (and left of the crack − 40). In it the white bar (a first-cycle palette
     ramp over x 240..740) shows, and the checker reads the sync-lost colours with its oscillators half a cycle on.
   - Bands of 8..71 virtual lines (holds 4..7) are drawn on even lines: plain 62 %, slip 25 % (±20..144 px × tear),
     hold 12.5 % (active at a tear-dependent rate).
5. **Output.** The box filter, luma 13/16 and the gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | the lattice's horizontal scale about the eye column: ×0.41 (one fat eye column, long diamonds) .. the still (512) .. ×2.46 (four columns of small eyes and crosses); exponential |
| K2 Rows | the lattice's V period: 792 lines (one row of tall eyes) .. the still's 280 (512) .. 99 lines (ten rows of flat eyes); the checker's row FM and zones follow it |
| K3 Eyes | the field offset: the checker floods and the eyes shrink to small cores (0, −200 LSB) .. the still (512) .. the tubes swell and gain a ring, the checker shrinks to diamonds (1023, +144 LSB) |
| K4 Scroll | bipolar (±8 deadband, square law): the lattice rises (+) or sinks (−); 770 = the clip's 238 lines/s; full = 16 lines per field. Centred, the lattice stops but the tear keeps flickering |
| K5 Checker | the checker's block widths and row heights together: ×0.35 (fine weave) .. the still (512) .. ×2.83 (big soft blocks) |
| K6 Etch | how far the camera bends the field: square law, 0 .. 2 LSB per luma step. 300 (default) = a gentle bend with clean tube curves, 600 = a strong etch, 1023 = the camera tears the tubes. The Osc keeps v0.1.0's linear 2·k6 gain (~14× the camera at the default) |
| S7 Crosses | Off (the still) / On: the two H regimes swapped across the crack. The right side becomes columns of white crosses and checker diamonds, and the eye column moves to the left |
| S8 Colours | Sunset (the still's yellow / red / violet) / Lagoon (the sync-lost blue / green / orange / violet everywhere; the torn region turns yellow / red) |
| S9 Rings | Tubes (the still) / Contours (the field ×3: every tube becomes nested ring lines) |
| S10 Invert | Off / On: the field inverted (100 − u). Checker eyes in white rings, white tubes where the checker diamonds were |
| S11 Source | Video / Osc |
| P12 Tear | 0 = in sync (the crack off the left edge, no sync-lost picture: the symmetric eye lattice) .. 25 % (a small torn wedge bottom-left) .. 50 % = the still .. 75 % (the crack mid-frame, the white bar over most of the left) .. 100 % (the crack at the top right, the cross regime and the sync-lost bands over nearly the whole frame, slips ±288 px) |

Glides (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a half-resolution line buffer, is box-filtered over
  8 px (four taps 2 px apart) and ADDED to the field, with a square-law gain (k6² / 2048 in Q8 per luma step).
  - It is unipolar, so a black input is exactly the still.
  - Bright parts of the picture push the tubes outward and shrink the checker around them. At the default Etch the
    tubes bend gently and their edges stay clean curves on `selfie.jpg` (`cmp_final.png`, bottom). From ~600 the
    camera's detail etches the tube edges, and at full Etch it tears them.
- **Osc mode.** A drifting 640 × 540 virtual-px triangle lattice, centred on its mean, feeds the same path with a
  linear gain of 2·k6 (Q8): ~14× the camera's at the default Etch, unchanged from v0.1.0. The eyes pinch, the
  crosses merge and the band wavers as it drifts.

## Engine
- **Per-field sequencer** (vblank zone). One serial shift-add multiplier runs 17 slots, each decoded two clocks
  ahead with pre-muxed operands; a shared glide; a 256-entry exp2 ROM. Per field it computes:
  - the H step (Columns), the V step (Rows), the checker A / B / row steps (Checker), and their first-pixel phases;
  - the scroll step (Scroll², with the wobble) and the V phase at line 0;
  - the etch gain (k6² for the camera, 2·k6 for the Osc; mantissa << shift), the Eyes offset;
  - the tear amount, the strip width, the roving top edge and the crack X0 from P12; a band seed every 4 fields.
- **Line engine.** Kicked at each line start, it computes the NEXT line in ~140 clocks (it fits SD's 858):
  - the V phase, Fy and the zone (V ROM), and the FM read 218 entries back;
  - the row phase and its parity; vy² and the crack;
  - the band draw (even lines) and its Q offset and slip × tear; the Q edge, the hold window (latched per band) and
    the slipped L-regime start phase.
- **Beam.** x, D, DL (with the hold), the checker A and B phases.
- **Pixel path (20 stages).**
  1. Beam.
  2. Regime, phase select, sync-lost flag, bar and crack distances, the row's oscillator phase, luma history (7 deep).
  3. Sawtooth start, bar minimum, the scar, luma pairs (taps 0 + 2, 4 + 6).
  4. Fx address, s, bar index.
  5. Fx ROM.
  6. Fabric copy, etch partial products.
  7. u = Fx + Fy + Eyes, the camera term.
  8. + camera.
  9. Invert.
  10. Contours.
  11. Fringe and the first-cycle flag.
  12. ROM addresses.
  13. ROMs.
  14. Fabric copies.
  15. Select (tube / checker / bar / fringe / scar).
  16. Black off-avid.
  17. The box: running 8-sums, then the 9-tap.
  18. / 16.
  19. Luma 13/16.
  20. The gate.
- **Memory.** ROMs: Fx 2048 × 10, tube palette 512 × 30, colouriser 512 × 30, V 512 × 16, exp2 256 × 16. Line
  buffer 1024 × 8. The box ring is 256 × 30 and the sync ring 256 × 3 (both `no_rw_check`). **21 EBR** in all.
- **Contracts.**
  - Generated avid; one latency (C_LAT 20) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr) straight from the ROMs.
  - Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box whose running sums snap back on blank
    samples; luma 13/16; the blanking gate.
- **Checks.**
  - GHDL analysis / elaboration / synth front end clean (with the package), and the TOML pre-check clean.
  - `imodel.py` vs GHDL, 720p60 dec 1, black input: **100 % of pixels bit-exact** (mean |d| 0.00), tear bands
    included. A second capture 3 fields later is also bit-exact, and the lattice has moved 6 lines up
    (`cmp_rtl_motion.png`). The model's Osc lattice is approximate (mean |d| 2.3 codes against the RTL Osc render).
  - `thincheck.py` (single-row luma runs ≥ 24 px, fields 3 / 41 / 97 / 150): 0 at the defaults and P12 0. At P12 100
    there is one chroma-only 27 px fringe sliver (field 97, its luma between its neighbours'). Crosses, Invert,
    Checker min and Osc give 0. Contours gives 1..2 and Rows max 10..23 (dense contours).
  - dupin on the synth json: 0 duplicate-input LUTs, 0 `CARRY I0==I1`.

## Status
- **v0.1.1 (2026-11-10), router1** (`NEXTPNR_ROUTER="--router router1"`). **All six configs closed on seed 1.**
  - v0.1.1 changes only the camera path: the Etch gain is now k6² / 2048 (a quarter of v0.1.0's bend at the
    default 300, the same at full), and the luma box is 8 px (four taps 2 px apart) instead of 4 px.
  - The Osc gain stays 2·k6 (as v0.1.0).
- Post-route Fmax:
  - HD: Analog 81.96, HDMI 76.64, Dual 80.21 MHz.
  - SD: Analog 83.01, HDMI 77.77, Dual 82.91 MHz (SD Fmin 27).
- 6699-6727 / 7680 LC (87.5 %), 21 / 32 EBR. Every bitstream passed iceunpack.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tatter`.
- Output: `out/rev_b/ron/heirloom/tatter.vmprog`. **Not flashed.**
- v0.1.0 (2026-11-10, same day): router1 6/6 seed 1. Fmax: HD Analog 78.75, HDMI 80.89, Dual 85.65; SD Analog 79.72,
  HDMI 77.01, Dual 80.48 MHz. 6672-6697 LC. Its default camera Etch frizzed the tube edges on a live picture
  (review), hence v0.1.1.
- **Timing history.**
  - A first router2 build of an earlier netlist missed HD Analog on seeds 1 and 2 and was stopped.
  - router1 then froze at 10028 remaining arcs. The cause was four LUTs with a duplicated input: sign-extended
    self-adds in the Eyes offset, the Contours ×3 and the sawtooth shift-adds.
  - Those were rewritten on unsigned / biased values (dupin: 0 duplicate-input LUTs, 0 `CARRY I0==I1`), and four
    single-clock add chains in the line engine and the zone were split. router1 then closed 6/6 on seed 1.
  - If a later version gets tight, stay on router1 and re-run dupin first.

## Deviations
- **Brightness.** Whites come out at ~777 output luma (the shell's cap), so the tubes read light grey next to the
  capture's clipped white, and the yellows read orange (13/16 luma, full chroma).
- **The tear is a designed band engine**, not the clip's exact one. The still's top-left cross column is nearly
  intact with a few long white spikes. The program's slipped bands and held blocks are coarser (≥ 2 rows, holds one
  colour down the band, by the single-row rule), and its sync-lost edge is a cleaner diagonal staircase.
- **Dropped rims.** The thin dark rim where the tube meets the checker, and the dark line inside each orange ring,
  were softened. Where a contour runs horizontally they would be single-row dark lines. The fringe is a light
  red-orange rather than the still's dark red / magenta.
- **No x-gradient in the colouriser.** The still's red blocks turn pinker toward the left; the program's red is
  uniform. The still's ring is magenta in places; the program's is orange.
- **The left regime beyond x 700** (seen only in lower rows left of the crack) is an assumed 480 px continuation of
  the cross column. The clip shows crosses at x 990 there, which this reproduces only approximately.
- **The camera** bends the field only horizontally smoothed (an 8 px box, no vertical filter). At high Etch its fine
  detail still etches the tube edges.
- **Rows max** (K2 1023) and **Contours** leave a few 25..85 px one-row contour slivers. The defaults and P12's ends
  are clean.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (virtual x 240..1680): the
  blue-green strip falls outside it and the white bar starts at the left edge. 1080i sources render both fields alike.
