# Frogspawn — a green/black cellular contour lattice, scrolling down

**v0.1.1 (2026-11-04): built, NOT flashed** (v0.1.1 = the coordinator's review fixes, see below). Heirloom (video phase), rebuilt from the clip
`WIN_20230206_00_35_06` at 0.5 s (`photos/vid/WIN_20230206_00_35_06_t0p5.png`). The still shows one
scalar field through a rainbow-fringed green colouriser: round cells of concentric rings (red core, dark-green
annulus, thin orange/pink/purple/blue fringes, a bright green ring, a black ring) in a row across the middle,
narrow vertical "eyes" between the cells, pairs of nested diamonds ("hearts") on the rows between, and on the
right the field saturates into solid black ovals. Presets are hand-written placeholders.

## v0.1.1 (review fixes)
| Was (v0.1.0) | Now (v0.1.1) |
|---|---|
| The palette's low end (the saddle rows' hearts, ~40 % of the picture) was a frame-wide per-bin average: pale light-green diamonds with one grey ring | Every palette entry is a sharp sample of the still, placed against the model's F levels on the saddle band (`pal5.py`): the left hearts are nested cyan / grey / magenta / purple / yellow-orange rings round a dark-green core, the far-left heart column is red / magenta / purple on dark green, the middle and right hearts stay green with yellow lines as in the still. Field, gain, knee and geometry unchanged (a creased V that pointed the heart tops was tried and rejected: it made every cell 1.2× taller) |
| S9 Diamond: triangle hump in x only (barely distinguishable from Round) | S9 Diamond also makes the rows a triangle wave: straight-sided diamond cells and hearts |
| S10 Wrap: the index wrapped mod 1024 (a few red dots) | S10 **Rings**: the index folds about the top (F = 1.03), so every black oval reads the palette back down and becomes a nested ring cell |
| S7 Twin read as dead on the old sheet (rendered through the selfie etch with the washed palette) | unchanged mirror of phase and gain (black ovals at both edges, a double cell at the centre); the sheet now renders from the still's state |
| Etch centred at luma 128: a black input lowered the whole field by 0.23 | Etch adds the luma, (Y >> 2) − 16, like lozenge: with no camera, Video mode is the still |
| K5 Tilt default 704 (gain slope 0.75) | 768: the fitted slope 1.0 (the RTL law is a = (k5 − 512)/256) |
| power-up row phase put the cell rows at y = 88 + 531 k | the cell row starts on line 551, as in the still (it scrolls from there) |
| cmp_final: RTL Osc at decimation 2 (top-left quadrant) | still \| proto_final \| RTL Video with a black input \| RTL Osc \| RTL Video (selfie), all RTL at 720p60 decimation 1 |

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_frogspawn/` (`notes.md` has the full log).
- **One field, two oscillators.** Every contour is a level of F(x, y) = H(x) + V(y) (the mirror columns 521,
  970, 1428 and the mirror rows 551, 278 are exact; the ring radii of the 970 cell are the same in x and y).
- **H(x)**: period 453.5 px (4.2 cells per line), cell centres at 521, 970, 1428, 1883. Between two cells, a
  narrow eye 0.465 of the period after the cell (732, 1172, 1635), 66 px wide and 190 px tall (the green lens
  of the 735 eye, rows 460–640), with sharp V-shaped dips 62 px either side of its apex (black 4–10 px wide on
  the cell row at 671 and 788). The hump at the cell is round (cosine, half-width ~200 px); the eye is a
  triangle.
- **Levels** (relative to the 970 cell's red core = 1.0, by palette colour): cells 521 −0.55 (green core with
  yellow rings), 970 0, 1428 +0.49 (black oval 238 × 169 px), 1883 +0.76 (326+ × 220); eyes 735 ~−0.85 (green),
  1180 +0.05 (a 25-px black core), 1655 +0.49 (black lens 103 × 168); dips −1.38 (black ring) on the left,
  ~−0.65 (green) in the middle, ~−0.7 on the right. So a gain rising to the right (1 + v + 0.2 v², v = (x −
  960)/960, floored at 0.45) and a soft saturation above the 970 level.
- **V(y)**: period 531 px (2.03 rows per frame), maximum (cell row) at 551, saddle rows at 278 and 817, the next
  cell rows at 20 and 1082; a cosine within the noise (the per-row scan tracks it; the saddle green zones are
  ~10 px wider on their upper sides).
- **Colouriser** (the x = 970 column, sharp): from the top: red core (F > 0.97), fringe, dark green (0.83–0.92),
  fringe with a red line (0.80), dark green, fringe, GREEN 1 (0.06–0.60, yellow lines at 0.41 and 0.24), fringe
  (red 0.11), dark green, fringe (red −0.30), BLACK ring (−0.36..−0.40), fringe (red −0.49), dark green, fringe
  (red −0.72), orange/grey/blue fringe, GREEN 2 (−0.83..−1.0); below (from the eye and dip columns): cyan,
  grey, a red fringe, black/purple, grey, dark green, yellow/cyan, GREEN 3 at the global minimum (the hearts).
  Black above 1.03 (the ovals). The capture is horizontally soft (chroma ~10 px), vertically sharp.
- **Hearts** (row 278, measured along the row from the left heart's centre at x = 632): dark-green core (616–644),
  yellow/orange 644–654, grey, purple 658–672, magenta 672–680, grey, magenta, grey, cyan 702–708, then the green
  crossing under the eye (710–744). The far-left heart column (x ≈ 196 and 335) is red/magenta/purple on dark
  green (row 278: dark green, cyan, dark green, orange, RED 354–382, magenta, purple, cyan, dark green).

## Motion in the clip
- The lattice scrolls **down at ~150 px/s** (2.3–2.7 px per frame at 960 × 540, measured frame to frame over
  0.4–0.9 s; one row period per ~3.6 s). No horizontal motion at all (H is line-locked).
- Ron's knob (0.9–2.3 s, and 4–7 s): the row oscillator grows an octave — each 531-px period gets **two stacked
  cells** (red-core and green-core alternating, vertical autocorrelation peak still at 550 px) — and comes back.
  H never changes (the mirror columns are the same in every frame).
- The program scrolls down at the clip's rate by default (K4 Speed 700) and puts that octave on **P12 Split**.

## What it does
1. **H table.** W(u), 1024 entries over one cell period, refilled every field: W = min(hump, eye) with
   hump = (1 − cos(πu/200 px))/2 (1 beyond ±200 px; Diamond: |u|/200) and eye = E + (1 − E)|u − 0.465 T|/62 px.
2. **Gain and saturation.** r(x) = max(0.45, 1 + a v + b v²) by a second-order DDA per pixel (Tilt sets a, b);
   h = r (2 − W) − 2; above 0 a knee compresses h with slopes 1, 1/2, 1/4, 1/8 per 0.35 units (the ovals).
3. **Rows.** V = cos θ + split · cos 2θ per line (quarter-wave ROM), θ advancing per virtual line and retreating
   per field (the scroll).
4. **Field and colour.** F = h + V + etch · luma; palette index = F/4 + 666 (1/256 of a unit per entry; the offset
   is pre-added to the line's V), clamped (Black: the top 94 entries are black) or folded about entry 929.5
   (Rings). Pink negates both chroma axes. The palette is all sharp samples of the still: the cell column x = 970
   for F ≥ −0.98, its GREEN 2 rows (222–290) for [−1.92, −0.98), the left heart's row profile for [−2.18, −1.92),
   the far-left heart's for [−2.56, −2.18).
5. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the capture's softness and the HD HDMI bar
   fix), luma 13/16 (the palette is pre-scaled 16/13, capped at 940), the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Cells | the cells' period: 1/2.8 .. 2.8 × the photo's 453 px (exponential, 512 = the photo) |
| K2 Rows | the rows' period: 1/2.8 .. 2.8 × the photo's 531 px (512 = the photo) |
| K3 Eyes | the eyes' height: none (0, flat dips) .. the photo (716) .. as tall as the cells (1023) |
| K4 Speed | bipolar (±8 deadband, square law): the lattice scrolls down (+) or up (−); 700 = the clip's 150 px/s, full = 1170 px/s |
| K5 Tilt | the rising gain: black ovals on the left (0) .. flat, every cell like the 970 one (512) .. the photo (768) .. steep, black from the centre (1023) |
| K6 Etch | how far the camera (or the Osc) bends the lattice: 0 .. 3 units per full luma (square law, 400 = 0.46); the luma only adds, so bright areas push the rings outward and the ovals grow |
| S7 Twin | Off / On: the field mirrored about the centre (black ovals at both edges) |
| S8 Colours | Green (the photo) / Pink (chroma negated: a pink lattice with green fringes) |
| S9 Cells | Round (cosine hump and cosine rows, the photo) / Diamond (triangle hump and triangle rows: straight-sided diamond cells and hearts) |
| S10 Top | Black (the saturated field is black, the photo) / Rings (the palette folds about the top: every black oval becomes a nested ring cell) |
| S11 Source | Video / Osc |
| P12 Split | the rows' octave: 0 % = single rows (the photo); 50 % = stacked pairs of cells (the clip at 4 s); 100 % = cos θ + cos 2θ, the stacked lattice at full depth |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is added to the field
  (K6 Etch): the rings bend around the picture like a contour map, bright areas push the field up (rings
  move outward, black ovals spread), the cells and eyes still read at the default. With no input, Video mode
  is exactly the still (the black-input RTL render is bit-exact with the integer model of the still).
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting, feeds the same path: a gentle wobble of the
  rings at the default Etch. It adds like the camera (0 .. 7/8 of full luma), so it also lifts the field a little
  (the 970 core turns black, a few more ovals).

## Engine
- **Per-field sequencer (vblank zone, zc 18–362):** one serial shift-add multiplier, 14 slots (kx from the
  Cells exponent through a 256-entry exp2 ROM, ky from Rows, the line step, speed², the scroll step, the eye
  slope and the triangle's start, |ra| · 0.2, the three gain-DDA constants, etch², the etch gain, the beam's
  start phase); a shared glide (K1, K2, K3, K5, K6, P12). The W table is filled at zc 400–1430 (a 5-stage
  pipeline: two DDAs, the cosine ROM, min, write). The line-0 kick is at zc 1500.
- **Per-line engine:** two quarter-wave ROM reads (θ, 2θ) (Diamond: two triangle waves 2u − 2047 from the
  phase bits instead), one serial product (split · |cos 2θ|), V at lc 22, V + 2664 (the palette offset) at
  lc 23; the beam loads it at the next g_start.
- **Beam:** the H phase (24 bits, 10 of index) ± kx·xs per clock (backward on the left half under Twin, with the
  gain-DDA step corrected at the centre), the gain r and its step d (Q26).
- **Pixel path (15 stages to the compose):** 2 table address / gain clamp, 3 table, 4 fabric copy, 5 2 − W,
  6 the two 6 × 12 partial products, 7 sum, 8 h, 9 knee (and the etch term), 10 F, 11 index (slice, Rings fold,
  clamp), 12 palette ROM, 13 fabric copy, 14 pink, 15 compose; then the two boxes (16–19), luma 13/16 (20) and
  the gate (21).
- **Memories:** palette 1024 × 32 (8 EBR), W table 1024 × 10 (3), line buffer 2048 × 8 (4), exp2 256 × 16 (1),
  cosine 256 × 12 (1).
- **Contracts:** the generated avid; one latency (C_LAT 20) in every mode; virtual 1080-line coordinates (SD
  shows the centre 4:3: cells 521, 970, 1428 and all three eyes are inside it); colours in the hardware
  convention (U = Cr); the running-sum resync on both boxes.
- **Verified (v0.1.1):** GHDL at 720p60, decimation 1 (the full frame, real timing ID): the default frame with a
  black input (Video mode) and a Twin + Diamond + Rings frame are both **bit-exact with the integer model
  `imodel.py`** (100 % of pixels, after the shell's 4-px box-window offset), at the scroll phase of the third
  field. The default frame matches the proto in structure and palette (`cmp_final.png`); Osc and the selfie bend
  it as designed. Standalone yosys (v0.1.0): no transparency emulation on the W table.

## Status
- **v0.1.1 (2026-11-04), built with ROUTER2 through the shared queue, NOT flashed.**
  - Build command: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh frogspawn` (= a scoped clean, then
    `./build_heirloom.sh frogspawn`).
  - **Seeds: HD Analog = seed 2** (seed 1 stalled router2 to the 15-minute timeout), **HD Dual = seed 3** (seeds 1
    and 2 missed); HD HDMI and the three SD configs on seed 1.
  - Fmax (post-route, the builder's last "Max frequency" line): HD Analog 76.21, HD HDMI 74.52, HD Dual 74.73,
    SD Analog 74.05, SD HDMI 77.42, SD Dual 74.73 MHz (SD Fmin 27). **HD HDMI (+0.27) and HD Dual (+0.48) are thin.**
    router2 stalls or misses often on this netlist (v0.1.0 needed seed 2 on all three HD configs). A router1 run of
    the same HD Dual netlist closed on seed 1 at 75.31 MHz (scratch, not installed), and the netlist has no
    SB_CARRY self-adds (dupin.py), so `NEXTPNR_ROUTER="--router router1"` is the first thing to try on the next
    build or if HD HDMI shows a timing glitch on hardware.
  - 6730–6752 / 7680 LC (88 %), 17 / 32 EBR (palette 8, line buffer 4, W table 3, exp2 1, cosine 1). Every
    bitstream passed iceunpack; the packed config is v0.1.1 with the new "Rings" label and the seven presets.
  - Output: `out/rev_b/ron/heirloom/frogspawn.vmprog`. Presets are hand-written; capture Ron's looks with
    `./capture_preset.sh`.
- v0.1.0 (2026-11-04, superseded): router2, HD Analog s2 75.72, HD HDMI s2 80.55, HD Dual s2 80.71, SD s1;
  6676–6694 LC, 17 EBR.

## Deviations from the photo
- The heart tops and bottoms are rounded (the measured cosine rows); the photo's hearts are pointed top and bottom.
  A creased V reproduced the points but made every cell 1.2× taller, so it was not used.
- Each dip carries one heart diamond; the photo has a pair per dip (centres 632 and 690 around the 661 crease),
  so the model's left hearts are wider single diamonds with the photo's ring sequence.
- The far-left red blob (an H maximum cut by the edge at x ≈ 10, as high as the 970 cell) is not reproduced:
  the model's gain is lowest at the left edge, so the left edge shows the far-left heart column instead.
- The saddle under the 1428 oval falls on the black ring (F ≈ −0.38); the photo shows orange/yellow there.
- The right-hand ovals come out ~1.3:1 (the photo's are 1.4:1) and a little smaller; the knee is piecewise
  linear, so their rims are a touch harder than the photo's.
- The 521 cell's yellow rings are faint (its level sits at the bottom of GREEN 1).
- The eyes are symmetric triangles; the photo's eye apex is ~13 px left of the midpoint between its dips.
- The output conditioning (two 9-tap boxes, the HD HDMI bar fix) blurs 17 output pixels, so the cores and fine
  fringes are softer than the photo's (~10 px chroma) at 720p and SD; at 1080p it is close.
- Osc mode lifts the field a little (it adds like the camera), so the 970 core turns black and a few more ovals
  appear; with the camera, bright areas do the same by design.
- Resolutions: 720p shows the same geometry as 1080p; SD shows the centre 4:3 (the 1883 oval is outside it).
  The scroll step halves on 50/60-field modes so the px/s rate is the same.
