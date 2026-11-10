# Butterfly — a magenta kaleidoscope of pinched lenses, hearts and nested blue diamonds

**v0.1.1 built 2026-11-10, NOT flashed.** v0.1.1 makes the camera bend the rings smoothly. The default Bend is a
quarter of v0.1.0's, and a vertical + horizontal IIR filter smooths the camera luma. The Osc flip is unchanged.

Heirloom (video clips), from the still `photos/vid/WIN_20231008_20_41_21_t0p5.png` (clip `WIN_20231008_20_41_21`,
8.5 s; triage: "magenta/blue/black kaleido butterfly"). The still is a mirrored lattice:
- big magenta lenses pinched at dark X crossings;
- a blue band bowed over and under each lens;
- a hot-magenta heart above and below every X (tip at the X, notch toward a nested blue diamond);
- blue arcs bracketing the diamonds, and a bow-tie glow between the arcs.

Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_butterfly/`: `notes.md`, `axes.py`, `edges.py`, `palfit1.py`, `tabfit2.py`,
`fproto.py`, `imodel.py`.
- **Lattice.** Mirror axes by flip-correlation:
  - x = 520.5, 990.5 and 1460.5, an H period of 940 px (2.04 across);
  - y = 256.5, 517 and 781, a V period of 524.5 lines (2.06 down).

  The fundamental cell is 470 × 262 px, mirrored four ways. In cell coordinates (u from the X column, v from the
  lens row):
  - the X point is (0, 0);
  - the lens centre is (470, 0);
  - the diamond is (0, 262);
  - the glow is (470, 262).
- **Palette.** One colouriser over a signed level g, symmetric in sign: |g| = 0 is blue (0, 41, 171), 1 is
  black-navy (3, 10, 58), then a red-and-blue ramp up to clipped magenta (255, 0, 255) at |g| ≥ 2.45. It has 26
  entries, measured on the lens columns where the field is known exactly.
- **Lens field F1.** All the lens contours are one pencil of curves through the X point:
  - the lens edge is |v| = 134 sin(π u / 940), to ±2 px;
  - the blue band is the same curve scaled, |v| = 174 sin(π u / 940).

  So F1 = 5.75 ln(v / (174 sin(π u / 940))). Lens edge t = 134, blue band 174, glow edge 224 (symmetric in ln t:
  −0.261 / +0.253). The patch: a rectified H sine and a V triangle into a divider / polar converter.
- **Ring field F2 (hearts, diamonds, arcs).** A ring system around the heart lobes.
  - The best analytic fit, a tilted ellipse about (59 warped px, 119), gets the heart outline to 6.7 % rms and the
    blue ring to 7.8 %. Its far field puts false rings across the lenses.
  - So F2 was measured from the still instead: a 33 × 17-node bilinear table over the cell. It was least-squares
    fitted to the inverted palette where F2 shows, with |F2| inequality constraints where F1 hides it.
- **Composition.** |g| = min(|F1|, |F2|): whichever field is nearer its blue line wins, like a "darker wins" mix of
  two colourised layers. This one rule explains:
  - the chevron where the lens band meets a ")" arc;
  - the diamond, where four heart ring systems meet;
  - the glow: F1's positive side (the bow-tie edge) cut to points by F2's arcs.
- **X arms.** F1's black / blue / black bands squeeze into a wedge that the camera blurred to a 20 px dark-purple
  arm, lighter at the crossing: R 116 at r = 20, 91 at 40, 45 at 80, 5 at 140 px.
- **Detail.**
  - The diamond's blue ring is at 60 px (square, L1).
  - The purple bump on the diamond row (u 130, R 53) is the saddle between a heart and its mirror below.
  - The ")" arcs cross the diamond row at u = 205 and bulge to u = 250.

## Motion in the clip
- **The patch never moved.** The m-frames (−0.5 / +0.5 / +1.0 / +1.5 s) and all eight strip frames differ from the
  still by 0.5–0.9 grey levels (compression noise), with a best shift of 0 px.
  - Frames re-extracted across the whole 8.5 s clip (0.2, 2.5, 4.0, 5.5, 7.0, 8.3 s) differ by 0.55–0.58.
  - No knob was turned, and the clip has no other looks of the patch.
- **What the program does.** At the defaults it holds the still, as the clip did (K4 Flap = 0). Motion comes from:
  - the camera;
  - the Osc in Osc mode: a lattice drifting at 75 px/s;
  - K4, the logical moving part: a wing beat. A triangle LFO (0.53 s per cycle at full, 3.2 s on the Wingbeat
    preset) opens the lenses (t0 × 1.41) while the hearts shrink (+0.8 palette units), then reverses.

## What it does
1. **Coordinates.** Per pixel, a 24-bit H phase; per line, a 24-bit V phase. Both are folded to 15-bit cell
   coordinates u and v, in virtual 1080-line units. The cell scales with K1 / K2.
2. **F1, the lenses.**
   - L = ln v − ln sin(π u / 940) − ln t0, in Q8 natural-log units.
   - ln v comes from a 512-entry ROM, per line. −ln sin comes from a 1024-entry ROM, per pixel.
   - F1 = k1 · L, with k1 = 0.587 G per Q8 at the photo. G units are 26.12 per palette unit.
3. **F2, hearts / diamonds / arcs.**
   - Per line, the line engine lerps the two table rows bracketing v across all 33 columns. It adds the Hearts
     offset, applies the Rings gain, and writes {row, next − row} into a ping-pong block RAM.
   - Per pixel, F2 = row(i) + d(i) · fu.
4. **Camera.** The camera luma is smoothed, then m = luma · gain is added to both fields, so the contours bend
   around the picture. Smoothing:
   - Vertically, a recursive filter on the line-buffer write: f = (previous line's f + new) / 2.
   - Horizontally, a recursive filter on the read: h += (f − h) / 4.

   Osc mode feeds its lattice through the same path at 16× the gain, centred so it bends both ways.
5. **Knot.**
   - r ≈ max + 3/8 min of (u, v) in cell px, and f = 1 − r / 280.
   - The arm level is gl = 1.5 + 0.95 · min(1, r · |F1| / 512); in the RTL, 39 + min(24, r · |F1| >> 9) G.
   - g1 = Ac + f · (gl − Ac), where Ac = min(|F1|, 4.0).
6. **Mix.**
   - Darker: |g| = min(g1, |F2|). Interfere: |g| = |g1 − |F2||.
   - Echo folds |g| with a 128 G (4.9-unit) period, two cycles.
7. **Output.**
   - Palette ROM: two palettes of 128 entries.
   - Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes (the still's softness and the HD HDMI bar fix).
   - Then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Columns | the H oscillator: butterflies across, ×0.5 (one giant lens) .. ×1 (2.04, the still, 512) .. ×3 (six across); the cell keeps its shape |
| K2 Rows | the V oscillator: butterfly rows, ×0.5 (tall diamonds, one lens row) .. the still .. ×3 (flat lenses stacked six deep) |
| K3 Hearts | the hearts' size (F2 offset, ±2.5 palette units, ±8 deadband): small lobes wrapped in extra rings (0) .. the still (512) .. hearts swallowing the diamonds, which shrink to dots (1023) |
| K4 Flap | the wing beat (triangle LFO, square law): 0 = still, as in the clip; full = 1.9 Hz; the lenses open while the hearts shrink |
| K5 Rings | band weight of every ring, lens bands and heart rings together (gain on both fields, ×0.54 .. ×2.46): wide soft bands (0) .. the still (512) .. hairlines between flooded magenta (1023) |
| K6 Bend | how far the smoothed camera (or the Osc) bends the rings, square law: 0 .. 217 G per full luma; the default 224 (gain 6/128) bends the bands as clean curves; 600 a strong bend; 1023 shreds the rings into the picture |
| S7 Echo | Off (the still) / On: every magenta field repeats the ring sequence (lens interiors go dark with magenta outlines) |
| S8 Mix | Darker (the still: the field nearer its blue line wins) / Interfere (the difference of the two fields: blue lenses, star-shaped hearts) |
| S9 Palette | Magenta (the still) / Ember (amber fields, teal bands, black) |
| S10 Twin | Off / On: the heart / diamond field at twice the H rate, so hearts and diamonds also sit on the lens columns |
| S11 Source | Video / Osc |
| P12 Wings | lens width (t0 = 35 .. 855, exponential) with heart size (+1.5 .. −1.5 units): 0 % = closed slivers striped by their bands, small hearts in dark spines; 50 % = the still; 100 % = wings flooding the frame, hearts merged into the glow, the X crossings turned into spiky stars |

## Video
- **Video mode (the default).**
  - Camera luma, taken as (Y >> 2) − 16, is smoothed by the two recursive filters (vertical, then horizontal),
    multiplied by the K6 gain and added to both fields. The lens bands and heart rings bend around the picture.
  - With no input, Video mode is exactly the still.
  - At the default Bend (224, gain 6/128) the selfie bends the bands gently. The lens bands, hearts, diamonds and
    arcs stay clean curves, with no per-line tearing.
  - The bend grows with the square of the knob; full Bend shreds the rings into the picture.
- **Osc mode.**
  - The shell's 2-D triangle lattice (640 × 540 virtual px, drifting 75 px/s) feeds the same path at 16× the camera
    gain (98/128 at the default Bend, as in v0.1.0), with its mid level subtracted, so it pushes the contours both
    ways.
  - At the defaults the lenses ripple and the diamonds and glows warp: an unmistakable flip that keeps the patch.
  - Every video control stays live in Osc mode.

## Engine
- **Per-field sequencer (vblank zone).** One serial shift-add multiplier, 16 slots, with operands pre-muxed. A
  shared glide for K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field). An exp2 ROM. Per field it computes:
  - the H phase step and line-start phase, 8 px ahead to cancel the output boxes' delay;
  - the V step and line-0 phase;
  - k1 and the F2 gain;
  - ln t0, from P12 and the flap LFO;
  - the Hearts offset, from K3, P12 and the LFO;
  - the camera gain, the Osc centring and the LFO step.
- **Per-line engine.** Kicked at each line start, and once in the zone for line 0. It computes the NEXT line in
  about 80 clocks, so it fits SD's 858-clock line:
  - fold v, then read ln v and the knot's v distance;
  - read 33 × 2 table nodes and lerp them;
  - add the offset, apply the gain (split 16 × 5 + 16 × 4), clamp, and write the row.
- **Camera write path.** A 4-stage read-modify-write:
  - read the column's vertical-filter state;
  - an EBR read, then a fabric register;
  - f = (state + new) / 2, written to both the line buffer and the state RAM.
- **Pixel path (25 stages, C_LAT 24).**
  1. Fold.
  2. ROM / row RAM addresses.
  3. EBR read.
  4. Fabric register, and the horizontal camera filter.
  5. L, d · fu, knot max / min.
  6. k1 products (split lo / hi) and F2.
  7. F1 sum, and the camera on F2.
  8. The camera on F1.
  9. One's-complement magnitudes.
  10. r · |F1| (split).
  11. Sum, and f.
  12. gl − Ac.
  13. f · (gl − Ac).
  14. g1.
  15. Mix.
  16. Echo and palette address.
  17. Palette EBR.
  18. Fabric register.
  19. Compose.
  20–23. Two boxes.
  24. Luma 13/16.
  25. Gate.
- **Memories (20 EBR).**
  - −ln sin, 1024 × 12 (3).
  - ln v, 512 × 12 (2).
  - F2 table, 1024 × 8 (2).
  - Palettes, 256 × 32 (2).
  - exp2, 256 × 16 (1).
  - Row ping-pong RAM, 256 × 32 (2, `no_rw_check`).
  - Line buffer, 2048 × 8 (4).
  - Vertical-filter state, 2048 × 8 (4, `no_rw_check`).
- **Contracts.**
  - The generated avid.
  - One latency (24) in every mode.
  - The blanking gate.
  - The output running sums snap back to the palette black after 8 blank samples (resync). The camera filters are
    decaying recursive filters, not running sums, so they need no resync.
  - Virtual 1080-line coordinates; SD shows the centre 4:3.
  - Hardware colour convention (U = Cr); luma pre-scaled 16/13.
- **Verified (v0.1.1).**
  - GHDL 720p60, decimation 1, black input: bit-exact with `imodel.py` (mean |ΔRGB| 0.012 at the tester's 4 px
    capture offset).
  - Selfie: matches the model to 0.044 mean |ΔRGB|.
  - Osc: matches to 0.58 (the remainder is the Osc lattice start).
  - Motion (v0.1.0 sim, same motion path): Osc + Flap 1023, frame 2 vs frame 6, changes 66 % of pixels
    (`cmp_motion.png`).
  - The first v0.1.0 sim found two bugs, both fixed:
    - the flap LFO rested at its minimum (now centred);
    - the output boxes' delay is now cancelled in the beam phase.
  - Integer model vs the still at the defaults:
    - mean |ΔRGB| 6.1;
    - all 16 cells of the 4 × 4 region-mean grid within 3.6 levels;
    - magenta / blue / dark shares 43.5 / 23.1 / 20.8 %, against 41.8 / 23.5 / 21.8 % in the still.
  - Single-row line check (`thincheck.py`), 720p and 1080p, at the defaults, P12 0 / 100, Echo, Interfere, Twin,
    Osc, K5 1023 and K3 0: 0 runs of 24 px or more.

## Status
- **v0.1.1 built 2026-11-10 with ROUTER1. NOT flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh butterfly`
    (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh butterfly`).
  - All six configs on seed 1. Post-route Fmax: HD Analog 92.44, SD Analog 85.52, HD HDMI 87.35, SD HDMI 86.55,
    HD Dual 82.60, SD Dual 88.15 MHz.
  - 7026–7046 / 7680 LC (91.5–91.7 %), 20 / 32 EBR. iceunpack OK on all six.
  - Output: `out/rev_b/ron/heirloom/butterfly.vmprog` (451,586 bytes, unsigned).
  - Router2 was not tried.
- v0.1.0 (2026-11-10, replaced, never flashed): router1, all seed 1. HD Analog 96.63, SD Analog 84.51,
  HD HDMI 87.35, SD HDMI 87.35, HD Dual 87.40, SD Dual 87.03 MHz; 6926–6953 LC; 16 EBR. Its default camera Bend
  (448, unsmoothed) tore the right diamond, glow and arcs into per-line streaks.

## Deviations
- **F2 is measured, not modelled.** The heart / diamond / arc field is a 33 × 17 table fitted to the still. It
  reproduces the still and scales with Columns / Rows. It is not an analytic oscillator patch, so Hearts, Rings,
  Wings and Flap act on its level and gain, not on its shape.
- **The X crossings are a modelled blur.** The camera's blur of the pencil is approximated by the knot: a
  constant-width dark arm within 280 px of each X. The crossings come out slightly crisper than the still's soft
  purple knot.
- **Not drawn:**
  - the faint 6 px seam inside each lens on the lens row;
  - the 3–5 px black capture borders;
  - the camera's blue-leading chroma fringe on some magenta edges (B up to 35 above R).
- **Horizontal edges are slightly crisper than the still's**, because the output boxes blur horizontally only.
- **Motion.** The default is still, matching the clip. The flap is a designed addition: the clip shows no knob
  being turned, so no range was learned from it.
- **Camera lag.** The smoothing makes the camera's bend lag the picture by about one line vertically and a few px
  horizontally.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (1440 virtual px): about 1.5
  butterflies across instead of 2.04, at the same size and still round.
