# Bunsen — an orange bat-crown key over a swaying magenta contour field, its flame fading down the bat

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20231205_23_09_56_t0p5.png` (clip WIN_20231205_23_09_56, frame 15 at 0.5 s of a 6 s clip; the
alts 23_09_56 t4.0, 23_13_01 t0.5 / t7.5 and 23_05_36 t6.5 are the same patch). An orange "bat / crown" key (two
peaks with a notch, two wings, an X crossing, two side triangles, two bottom lobes round an arch) sits over a
magenta contour field with dark contour lines and blue pools in three corners. The key fades out toward the
bottom along a V-shaped ramp, so the lobes read hot pink, and through the clip Ron raises that fade up the bat.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_bunsen/`: `notes.md`; `shapetab.py`, `shapeclean.py`, `rowtab.py` (the key
and pools); `alpha.py` (the fade); `orient.py`, `kmeas.py`, `wsfit.py`, `fieldmodel.py` (the field fit); `motion.py`
and the kymographs (the motion); `proto2.py` (float proto), `imodel.py` (integer model), `sheet.py`.
- **The key** is mirror-symmetric about x 1006.5 (flip agreement of the orange mask 98.1 %). Measured as runs of
  |x − 1006.5| per row, left / right averaged; the lower lobes come from the clip's last frame, where they are drawn hard:
  - peaks: tips (960, 104) and (1053, 104), the round notch between them closing at y 204;
  - wings: tips (112, 100) / (1890, 100), concave top edges down to dips at (≈700, 340) / (≈1310, 340), merging
    with the body at y 346;
  - the X crossing at |X| 424 on y 520; the side triangles' apex at |X| 653, reaching the frame edges, y 240–730;
  - below the crossing the body's edge runs out to |X| 804 by y 1000 and drops straight down (the lobes);
  - the arch between the lobes: apex (1006, 752), |X| 166 at the bottom.
  - The mask rebuilt from the table agrees with the still's on 99.4 % of pixels above y 700 and 98 % in the lobes.
- **Key colour.** The key switches R on and B off; the G channel carries the field straight through (G is one
  continuous field across the key's edge). Inside: (255, G, 0), orange (254, 103, 0) to red (254, 19, 3).
- **The fade** (B inside / outside the shape, per row): alpha 1 down to y 720, 0.83 at 740, 0.49 at 800, 0.31 at
  890, a notch of 0.11 at y ≈ 980, rising again to 0.35 at the bottom. Its lower edge drips along the contours.
- **Edges.** Hard. The trailing edges (where the scan leaves the key) carry a dark red 4–6 px band (R 155–180,
  B 0: the B key turns off late), and the shallow trailing edges are serrated in 4–5-line teeth.
- **The field.** Magenta contours, 10.25 dark lines per row (top rows).
  - Colouriser cycle: magenta (250, 75, 245) → light magenta → violet (193, 41, 243) → purple (143, 43, 203) →
    dark purple (100, 40, 124) → a near-black olive core (49, 41, 29) → blue-violet → magenta.
  - k-means (k 14) shares: orange / red 37.4 %, magenta 18.5 %, violet 10.8 %, hot pink (the lobes) 10.6 %,
    light magenta 5.9 %, purple 5.4 %, dark purple 4.4 %, blue 4.3 %, dark olive 2.7 %.
  - A mirror line at y 520 runs across the whole width (the orientation map flips there): chevrons and eyes are
    mirrored top / bottom in the middle band, drapes of near-vertical contours lie below y 780, big arcs at the top.
  - Fit: F = H(x + D(y)) + V(sgn(Y)(|Y| + E(x))), Y = y − 520. These are two oscillator waveforms, each
    phase-modulated by the other axis, fitted as knot tables to the still's contour orientations (structure tensor
    of G) and its relative contour density: 4 starts × 450 Adam steps, best orientation cost 0.484 (1.0 = random).
    Plain separable and two-sine models scored 0.72–0.86.
- **Blue pools** (10, 23, 254), 4.2 % of the frame, ringed by violet:
  - a top-right bowl (ellipse centre (1545, −30), 128 × 125);
  - bottom-left: x < ~192 below y 790;
  - bottom-right: x > 1812 + 0.22 (y − 720) below y 716.
- Not drawn: the capture border (a green strip x < 22, a dark bottom row).

## Motion in the clip
- **The key shape never moves.**
- **The field breathes on a 3.93 s cycle.**
  - The G channel's temporal autocorrelation peaks at 117.5 frames (29.9 fps, r 0.70).
  - Every cycle shows two contour sweeps (contours swing out and back: lens shapes in the kymographs), the top
    leading the middle by ~0.8 s.
  - Block correlation between 0.5 s frames is only 0.3–0.6: the contours morph rather than translate.
- **Ron's knob is the fade.** Its notch rises through the clip: y ≈ 975 (0.5 s) → ≈ 780 (2.5 s) → ≈ 530 (5.8 s).
  Below the notch the key's red falls ~0.28 per line, so at the end the lobes are olive / dark red (148, 57, 12),
  the middle is clear magenta and the wings stay orange.
- **What the program does with it.**
  - An LFO steps 277 / 65536 per field at K4's default (236.6 fields = 3.95 s at 60 Hz) and sways the field: the H
    oscillator's phase by ±63 virtual px (sin) and the V oscillator's by ±60 lines (cos). The contours swing out and
    back twice a cycle, region by region.
  - P12 is Ron's knob: the notch moves from y −150 (0 %) through 975 (the still, 643) to 1640 (100 %).
  - The float proto at −0.5 … +1.5 s next to the m-frames is `cmp_motion.png`.

## What it does
1. **Field.** Per line the engine forms D(y) × warp, the H start, and |Y| × lattice. Per pixel:
   - an H-coordinate DDA reads the H table (512 × 4 px);
   - E(x) × warp × lattice indexes the V table (512 × 4 lines, signed, wrapping). The E term tapers to 0 within 64
     lines of y 520, so the mirror has no seam;
   - F = H + V + camera.
2. **Colouriser.** u = frac(F × rings) (7 bits) reads the background palette and the key palette: 128 entries per
   family, luma pre-scaled 16/13 and capped ~768 after the scale.
3. **Pools.** Each 4-line row gives the bowl's half-width and the two edges. A 64-px ramp blends the palette toward
   blue in eighths.
4. **Key.**
   - Each 4-line row gives the runs [a0, a1), [b0, b1), [c0, edge), grown by K5 (empty runs stay empty).
   - Per pixel the engine tests X = max(0, |x + jitter − 1006| − camera/4). The jitter is a 0–8 px sawtooth over
     5 lines (the serrated edges).
   - S8 inverts the shape. S9 adds a field-level key (F above its 65th percentile), drawn in the other family's key
     palette (yellow flames on the Ember look).
5. **Fade.** alpha = 0.1 + e/470 + max(0, e − 200) × 3.25/256, with e = |y − notch|.
   - The contour triangle adds ±0.5, so the fade drips along the contours.
   - alpha is clamped and taken in eighths.
   - Below the notch the key's red falls 0.28 per line (a per-line YUV offset).
6. **Output.** out = bg + (key − bg) × alpha.
   - A dark red band follows every key run's end: 5 px at 1080p, 3 at 720p, 2 at SD.
   - Then the shell's two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the running-sum resync, luma 13/16 and the
     blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | The contour bands' density, ×0.4 .. ×1 (512, the photo) .. ×3: a few fat soft bands .. a dense fine etching |
| K2 Lattice | The two oscillators' spatial scale about the bat's axis and the y-520 mirror, ×0.5 .. ×2: big cells .. a fine lattice of eyes and chevrons |
| K3 Warp | The cross-modulation depth (D and E), ×0 .. ×1 (512) .. ×4: a straight lattice of rounded cells .. the photo's drapes .. swirling, torn drapes |
| K4 Speed | The LFO sway, square law: 0 = still, 377 = the clip's 3.9 s cycle, 1023 = 0.5 s |
| K5 Bat | The bat's weight, ±256 virtual px on every run edge: thin slivers (0) .. the photo (512) .. swollen until it fills the frame (1023) |
| K6 Camera | How far the camera (or the Osc) bends the contours (0 .. 3.7 bands per full luma; default 320 = 1.2) and pushes the bat's edges out over bright areas (luma/4 px) |
| S7 Gas | Ember (magenta field, orange bat: the photo) / Blue (a blue field with black-green lines, a yellow-orange bat and magenta pools: the 23_13_01 look) |
| S8 Key | Bat / Cutout: the key inverted, orange everywhere outside the bat and the field inside it |
| S9 Flames | Off / On: flame blobs keyed from the field's level, in the other family's key colours (yellow on Ember, orange on Blue) |
| S10 Flip | The bat (and its pools) upside down: the lobes hang from the top, the peaks point down |
| S11 Source | Video / Osc |
| P12 Fade | The key fade's notch (Ron's knob in the clip): 0 % = embers (the bat clear at the top, dark red and olive / green below); 389 = the clip's end (clear middle, olive lobes); 643 = the photo (notch at y 975, hot-pink lobes); 100 % = the whole bat lit solid orange |

## Video
- **Video mode (the default).**
  - The camera's luma, (Y >> 2) − 16 from a one-line buffer, × K6 is added to F. Divided by 4 (in virtual px) it
    also pushes the key's edges: bright areas bend every contour and swell the bat over them.
  - At the default the bat, its fade and the pools read clearly; a busy picture frays the contours.
  - A black input gives the photo's patch: the 720p60 black-input GHDL frame equals `imodel.py` on 100 % of pixels.
- **Osc mode.** The shell's drifting 640 × 540 virtual-px triangle lattice, made zero-mean (− 120) and doubled,
  feeds the same path. The contours bend into lobes and the bat's edges wobble by up to ~75 px: an unmistakable flip
  that keeps the look with no input.

## Engine
- **Vblank-zone sequencer** (zc 30–105):
  - the shared 7-channel glide;
  - one 24-bit serial multiplier for the rings and lattice curves (two slopes each), warp × lattice and the speed
    square;
  - the LFO phase and its sin / cos (a 64-step table in logic);
  - the grow, the notch (−150 + 1.75 p12) and the camera gain.
- **Line engine.** Kicked at each g_start (and at zc 1300 for line 0), it finishes the next line in 117 clocks, so it
  fits SD's 858-clock line. One serial multiplier computes six products. In order:
  - the row table read: row r for the key and pools, rows r and r + 1 for D, interpolated;
  - the grow, with empty-run flags;
  - D × warp, and the H start ((x0 − 1006 + D + oh) × lattice);
  - |Y| × lattice;
  - the fade alpha and the red fall-off offsets;
  - the per-line warp taper.
- **Pixel path, 19 stages.**
  - 2: the H DDA, the E and H reads.
  - 3: |x − 1006| and the bowl distance.
  - 4: table copies and pool distances.
  - 5–6: E × warp, the camera product, beta.
  - 7: the key x with the camera push.
  - 7–8: the V address and the key compares.
  - 10: F.
  - 11: F × rings in two partial products.
  - 11–12: u and the palette addresses, the flame test.
  - 13–14: palettes, with the fade alpha.
  - 15–16: the pool blend.
  - 17–18: the key mix in eighths.
  - 19: the outline, clamps and the avid gate.
  - Then boxes 20–23, luma 24, gate 25. **C_LAT 24.**
- **Block RAM: 23 EBR.**
  - line buffer 2048 × 8 (4);
  - palettes 2 × 256 × 32 (4);
  - H 512 × 16 (2);
  - V 512 × 16 (2);
  - E 512 × 8 (1);
  - the key / pool / warp row table 256 × 80 (5);
  - output-box rings 4 × 256 × 16 (4, `no_rw_check`);
  - the sync ring (1).
- **Contracts.**
  - Generated avid.
  - One latency (24) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 1440 virtual px).
  - The boxes and 13/16 luma before the gate, with the resync.
  - Colours in the hardware convention (U = Cr).
  - The engines start per line and per field.
- **Verified** (GHDL, 720p60, decimation 1):
  - The black-input frame equals `imodel.py` bit for bit (with the tester's 4-px output shift).
  - A fast-Speed run (K4 1023, 480 × 270) lands exactly on the model's frame two sin steps on, so the per-field LFO
    step is right.
  - Osc and camera renders are in `cmp_final.png`.
  - check (with `bunsen_pkg.vhd`) is clean, the TOML pre-check passes, and dupin finds 0 `CARRY I0==I1`.
  - The sims found three bugs, all fixed: a narrowing `resize(−x, 11)` that drew a phantom pool, a V table read one
    clock late, and a camera / Osc-driven gap down the bat's axis (the key x is now clamped at 0).

## Status
- **v0.1.0 built 2026-11-09 through the shared queue with ROUTER1.** NOT flashed.
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh bunsen`.
  - All six configs on seed 1. Post-route Fmax: HD Analog 82.80, HD HDMI 75.89, HD Dual 77.85, SD Analog 84.71,
    SD HDMI 82.27, SD Dual 77.88 MHz.
  - 6,917–6,945 / 7,680 LC (90 %), 23 / 32 EBR. iceunpack OK ×6; the packed config carries the labels and presets.
  - Output: `out/rev_b/ron/heirloom/bunsen.vmprog`.
- **Build history.**
  - The first build (before the budget cuts below) came to 7,495 LC (97.6 %) and was stopped.
  - Two router1 builds of the final netlist gave identical seeds, Fmax and LC.
  - router2 was not tried; this engine family closes on router1.
  - HD HDMI has the thinnest margin (2 %). If a later version slips, try other router1 seeds before restructuring.

## Deviations
- **The field is a fit, not a copy.** The warped two-oscillator model gets the photo's kind of structure (the
  y-520 mirror with chevrons and eyes, drapes below, arcs above, the band density) but not every contour. The
  rounded-square cells at the left read as elongated eyes, and the still's dark lines are softer and a little
  thicker.
- **Key edges** come from 4-line rows (the row interpolation was cut for budget, see below). With the 0–8 px
  serration they look like the still's toothed edges. The arch and notch are within a few px.
- **The fade** is taken in eighths (cut from sixteenths for budget); the contour term breaks up the steps.
- **Pools** keep the still's shapes; in the clip they drift with the field.
- **The clip's motion** is reproduced as its 3.9 s sway, not the exact morph of each contour.
- **Not drawn:** the capture border. The left edge fades in from black over ~10 px (the output filter's warm-up).
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 1440 virtual px: the side triangles
  are mostly off-screen and the wing tips sit at the edges.
- **Cut for budget** (the first build was 7,495 LC, 97.6 %), bringing it to 6,917–6,945 LC:
  - the key table's per-line interpolation (only the warp D is still interpolated);
  - the 16-level fade alpha (now 8);
  - the serial multipliers, narrowed to 24 bits.
