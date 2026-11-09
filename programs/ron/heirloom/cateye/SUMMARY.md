# Cateye — a ringed bullet eye in a cream valley between dashed magenta chevron walls

**v0.1.1 built 2026-11-09, NOT flashed.** Heirloom phase 3 (video clips), from the still at 1.0 s of the clip
`WIN_20240102_00_30_49` (`photos/vid/WIN_20240102_00_30_49_t1p0.png`, 10.5 s clip). The still shows a pink and yellow
chevron eye: magenta walls with dashed blue/orange lines bend into a ">" toward the middle of the frame, a pale yellow
cream valley holds a bullet-shaped eye (rounded on the left, pointed on the right) of red and maroon rings with a
cyan-or-orange rim, a pink kite pupil and a pink-red glow, hugged by a yellow chevron ridge. Horizontal stripes every
99 lines turn every line band blue or orange. In the clip Ron turns a colourizer knob: the blue drains out and the eye
grows rings. Presets are hand-written placeholders.

## v0.1.1 (send-back, 2026-11-09)
- **S11 Osc is a bold flip.** In Osc mode the bend gain is 3× the camera's (one per-field change:
  c_b8 = min(255, 3·(K6² >> 13)); the default goes 10 → 30). At the default K6 the walls and the ridge zigzag (~104 px
  peak to peak) and the eye's rings warp; 75 % of the pixels of the RTL Osc frame differ from the black-input frame
  (v0.1.0: 52 %). K6 stays live in both modes; Osc at K6 = 1023 is a structured pattern of diamond eye copies, not
  noise. The camera default is unchanged.
- **K3 Rings top end.** The moat tops out at 2× the rings (was 3×), and above 640 the rings step through three darker
  palettes (red against dark reds of 92, 56 and 24, palette E entries 192–239, chosen by a per-field base index), so
  1023 is a bold red / near-black ringed eye. 0 is still the plain red eye, 512 the still.
- Checks: the RTL K3 sim (the glide reaches 680 three fields after setting 1023) is bit-exact with the integer model at
  K3 680; the default frame is unchanged.
- router1, all six on seed 1: HD Analog 91.54, HD HDMI 93.42, HD Dual 89.12, SD Analog 81.95, SD HDMI 86.95,
  SD Dual 91.58 MHz; 7,147–7,177 LC (+~30), 21 EBR.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_cateye/` (notes.md has the whole chain).
- **One colourizer over a warped horizontal ramp.** Every wall contour obeys `u = x − T(y)·w(x)`: the horizontal ramp
  zoomed by a vertical triangle (a multiplier).
  - Left wall: x(T) = (x0 + 145.8 T)/(1 + 0.306 T), a zoom about x = 476 (concave, rounded apex). Right wall:
    x(T) = (x0 + 49.3 T)/(1 − 0.115 T), about x = −429 (convex, sharp apex). Right-wall residual ~2 px rms.
  - So w = max(0.306 (476 − x), min(0.3135 (x − 476), 49.3 + 0.115 x)) px, and T(y) is a soft-kneed triangle on
    line 540.5, measured per row (`contours.py`, `tshape.py`): the left T is rounder (reaches 0 at |dy| ≈ 460), the
    right sharper (0 beyond |dy| ≈ 340).
  - Contours move by up to +107 px (left magenta edge), +30 px (left cream edge), +195..+229 px (ridge and right
    magenta) at the apex row.
- **Palettes** come straight from the still's top row (median of the 'red' and 'blue' stripe rows): left wall outward
  from the cream edge at 390 px (cream, peach 48 px, pink 78, dark edge 132, line 140–180, red, magenta 378); right
  wall from 1062 px (cream, pink 50, line 68–104, dark maroon, red, magenta 296); the cream valley by position (pale
  (246,238,159) at the edges, yellower (247,243,126) at x ≈ 600, a peach socket dip G 175–200 at x 800–1000).
  k = 14 palette shares: magenta 21 %, red 12 %, maroon 11 %, creams 21 %, peaches 10 %, pinks 7 %, blue 3 %.
- **The eye** is a radial gauge, fitted by label purity (`eyefit.py`): r = max(((dx/416)^p + (dy/228)^p)^(1/p),
  (dx + 0.635 |dy|)/143.4) about (1002, 537), p = 1.92. Along r (red rows): pink pupil (255,49,123) to r 0.1, pupil
  line at r 0.215 (orange (254,167,62) / cyan (47,218,243)), ring zone 0.3–0.78 (red ↔ maroon, darkest at 0.55), rim
  line at 0.825–0.85, dark edge at 0.9, then the glow.
- **Glow**: a pink-red (255,18,84) soft key over the background, ~70 px wide horizontally, ~105 px diagonally, with a
  ~150 px tail upward into the socket.
- **Stripes**: 'blue' rows start at 82, 182, 280 … 1074 (period 99.2 lines, duty 0.5); they recolour only the line
  bands (the dashes, the rim and pupil lines, purple skirts in the eye).
- **Wall shade**: the red/maroon bands darken to ~0.6 at |dy| 230–300 (the chevron knees).
- Texture: ~10 px horizontal camera smear and noise, a dark capture border at x < 6.

## Motion in the clip
- Ron's colourizer knob (`motion.py`, 10 fps frames): the cream's blue goes 204 (0 s) → 143 (1 s) → 34 (2 s) → 6
  (3 s), the right magenta's 235 → 176 → 71 → 8 (magenta → red), and back part-way at ~6 s and ~10 s. With it the eye
  grows (left rim 606 → 568 → 532 → 522 px), gains rings, its pupil shrinks and turns black, the glow turns black, and
  from 2.5 s the dashes spill as blue/orange bars across the socket above and below the eye.
- Rings (`rings.py`, 30 fps): ring contours split from the moat and diverge, ~36 px/s each way (789 → 702 px and
  801 → 865 px over 2.4 s).
- Stripes (`stripe.py`): almost static; the period stretches 98 → 100 lines in 1.8 s (mid-frame edges creep ~6 lines/s).
- The eye and the walls do not move (eye centroid 850–890, 530 throughout).
- The program: P12 Heat is Ron's knob (its 0 %, 34 % and 100 % match the clip at ~0 s, 1 s and ~3 s). The default
  motion is K4 at 683: the rings stream out of the moat at 0.42 px of ring phase per field (one ring every ~1.3 s, the
  clip's diverging rings) and the stripes roll down 0.1 line per field. `cmp_motion.png` shows the proto at
  −0.5 / +0.5 / +1.0 / +1.5 s with the heat following Ron's move, next to the m-frames.

## What it does
1. **Warp.** u = x' − T·w(x'): three linear ramps T·L, T·M, T·R as DDAs (w = max(L, min(M, R))), with T (TL left of
   476 px, TR right of it) from a 512-entry per-row table times K1.
2. **Background.** Left wall d = 390 − u, right wall d = u − 1062 → palettes L / R (2 px per entry); between them the
   cream valley's palette D by u (4 px per entry). The socket dip (0 at u 620 → 43 px at 966 → 0 at 1014) turns into
   the wall palette past a heat threshold: the dashes spill as bars across the socket at high heat.
3. **Eye.** r = max(√E², C): E² = (dx·ka)² by a second-order DDA plus (dy·kb)² per line, a 1024-entry square-root
   table; C = dx·kc + |dy|·kcy (K1 also bends the eye's pointed side). φ(r) table → d_E = G·base(r) + Gm·moat·bump(r)
   → palette E: pink pupil, pupil line, red/maroon rings (16 entries per 32 px of depth, phase-shifted by the flow;
   three darker ring cycles for Rings above 640), rim line, dark edge. Outside, the glow: alpha = 1 − S(dy)·(r − 0.95)
   with S = max(2.875, 5.875 − 4t) per row; the glow (a 33-entry ramp from the pink-red to the cream) wins where
   3.75·alpha is deeper than the background.
4. **Stripes.** A 99.2-line square wave picks the 'red' or 'blue' row of every palette.
5. **Heat.** The palette's blue component Bp = (Y − 16) + 2.13 (V − 128) (8-bit) drains V by 3/128·|κ|·Bp (κ −0.45 →
   0 → 1 with P12), sparing the line bands; G and Gm scale the eye's depth.
6. **Output.** Luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as resyncing running 8-sums, luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Chevron | the V-bend of the walls, the yellow ridge and the eye's pointed side (T ×0 .. ×1 at 512 .. ×2.5): straight vertical bands and a D-shaped eye (0) .. the still .. deep chevrons with the ridge spiking to the right edge (1023) |
| K2 Eye | the eye's size: a bead (0, ×0.42) .. the still (512) .. an eye filling the valley (1023, ×2.4) |
| K3 Rings | the moat's depth: a plain red eye with only its rim and pupil lines (0) .. the still (512) .. 2× the rings in three steps of darker, bolder red / near-black rings (above 640, 768, 896) |
| K4 Speed | bipolar (±8 deadband, square law): rings stream out of the moat toward the rim and the pupil (+) or into it (−), the stripes roll down (+) / up (−); 683 = the clip-like default (one ring every ~1.3 s); full = 4 px of ring phase and ~1 line of stripe roll per field |
| K5 Stripes | the dash/stripe frequency: 4× coarser (397 lines, 0) .. 99 lines (512) .. 4× finer (25 lines, 1023) |
| K6 Bend | how far the camera (or the Osc, at 3× the gain, capped at 2× the camera's maximum) bends the walls and the rings: 0 .. ~435 virtual px and 0.85 gauge units per full camera luma (square law); 300 = ~34 px (camera), ~104 px zigzag (Osc) |
| S7 Facing | Right (the still) / Left: the whole patch reversed, the eye looking the other way |
| S8 Stripes | Dashes (the still: the stripes recolour only the lines) / Bars: every other 99-line stripe's chroma inverted across the picture (green, lavender and cyan bars) |
| S9 Pupil | Pink (the still) / Black: the kite pupil turns black inside its lit line |
| S10 Walls | Magenta (the still) / Black: the red, maroon and magenta wall bands keyed to black (the session's other clips have black grounds); the dashed lines stay |
| S11 Source | Video (default) / Osc: the walls and ridge zigzag and the rings warp under the drifting lattice |
| P12 Heat | Ron's colourizer knob: gain 0.75 → 1 → 1.73 and the blue drain −0.45 → 0 → 1. 0 % = pale dawn (white-cream, magenta, a small eye with a big pink pupil); 34 % = the still; 100 % = blazing yellow and red, the eye full of rings with a dark pupil, the dashes spilling as bars across the socket |

## Video
- **Video (default).** The camera's luma ((Y >> 2) − 16 from the one-line buffer) times K6 is subtracted from u (the
  walls, the ridge and the socket bend around bright areas) and from r (the rings and the rim bend), like a luma CV
  patched into the ramp and into the eye's shape. At the default (b = 0.16) the composition reads clearly; with no
  input (black) Video mode is the still, bit-exact with the integer model.
- **Osc.** The shell's drifting 2-D triangle lattice (640 × 540 virtual px, ~75 px/s) feeds the same path at 3× the
  camera's gain (per field, capped at 255): at the default K6 the walls and the ridge zigzag ~104 px and the rings warp;
  K6 scales it, and at 1023 it folds the picture into a structured pattern of diamond eye copies.

## Engine
- **Vblank sequencer** (zone = no avid for 1024 clocks): the control curves (two slopes each), the shared glide on K1,
  K2, K3, K5, K6 and P12, and 26 products on a slot counter (5-bit slot, phase 0–23; operands muxed straight into the
  multiplier, stores at phase 21, helpers at 22, a duplicated slot index for the mux, strobes registered a clock
  ahead): 1/size, heat gain, drain, bar threshold, ring gain, stripe frequency and its line-0 phase, bend² (×3 for
  Osc), speed², flow and roll steps, the eye gauge (ka, A, Q0, K2, D0, kc, Cx0, kb, kcy), the three ramp start
  constants. It shares the line engine's serial multiplier (36 × 18, 18 iterations), which is idle in the zone.
- **Line engine**, kicked at each line start, computes the NEXT line (~225 clocks, fits SD's 858): TL, TR (ROM by
  |y − 540.5|) × chevron, the three ramp steps and starts, t = |y − 537|·kb, Qy = t², Cy, the glow slope, the stripe
  parity.
- **Beam**: x' (Facing Left runs it down from 3840 − x0), the three ramps, the ellipse's second-order DDA (36 / 28
  bits) and the chevron DDA.
- **Pixel path (24 stages = C_LAT = processing_delay_clks):** E², T·w, chevron r; sqrt address, u0; bend partials;
  bend, sqrt copy; u, r = max(√E², C); wall / socket / cream clamps and the φ address; background family and index;
  φ copy; eye partial products; sums; d_E and alpha; inside / glow decisions and ring phase (+ the ring-palette base);
  the palette address; palette EBRs; fabric copy; compose (black walls / black pupil, Bp); drain product; apply;
  clamps; bars and the off-avid black; then output conditioning (2 stages) and the gate.
- **Memories**: palettes 3 × 2048 × 8 (12 EBR: parity × family L/R/D/E × 256), square root 1024 × 8 (2), φ 256 × 16
  (1), T 512 × 16 (2), line buffer 2048 × 8 (4) = 21 EBR.
- **Contracts**: the generated avid (the beam starts 8 px in); one 24-stage latency in every mode; the blanking gate
  one stage before the output register; output conditioning with the per-blanking resync; virtual 1080-line
  coordinates (SD shows the centre 4:3); palettes in the hardware convention (U = Cr) from `cateye.py`, luma
  pre-scaled 16/13 and capped at 940 (→ ~777 out).
- **Checks**:
  - GHDL analyse / elaborate / synth front end clean with the package (`check_pkg.sh`); `dupin.py`: 0 self-add
    carries; standalone yosys ~3830 LUT, ~2865 FF, 21 EBR, one transparency emulation (the line buffer's).
  - GHDL 720p60 decimation 1: the default Video-black frame (Speed at its default, frame 3) is **bit-exact with
    `imodel.py`** on every pixel but the 12-px filter warm-up at the left edge; a Speed = 1023 frame after 6 warm-up
    frames is bit-exact with the model at 6 flow / roll steps, so the motion engine is verified; the v0.1.1 K3 frame
    (glided to 680) is bit-exact with the model.
  - Fit history: the first RTL (a 3-channel glow blend, the drain on Y/U/V, a Mirror with a turning beam, a slit
    pupil, the wall shade) synthesised to ~9,800 LC; ablations put the blend + drain at 1,205 LUT and Mirror at 351.
    See Deviations for what was cut.

## Status
- **v0.1.1 (2026-11-09), ROUTER1** (`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900
  .heirloom_work/vid/build_queue.sh cateye`, which does a scoped clean and then `./build_heirloom.sh cateye`).
  **All six configs on seed 1.**
- Post-route Fmax: HD Analog 91.54, HD HDMI 93.42, HD Dual 89.12, SD Analog 81.95, SD HDMI 86.95, SD Dual 91.58 MHz
  (HD Fmin 74.25, SD 27). All six bitstreams passed iceunpack.
- 7,147–7,177 / 7,680 LC (93.1–93.4 %), 21 / 32 EBR.
- Output: `out/rev_b/ron/heirloom/cateye.vmprog`. **Not flashed.**
- v0.1.0 (same day, router1, all seed 1): HD Analog 88.59, HD HDMI 85.48, HD Dual 90.19, SD Analog 88.40, SD HDMI
  87.42, SD Dual 85.75 MHz, 7,117–7,132 LC. router2 is marginal on this netlist (HD HDMI 74.35 on seed 2 in one build,
  74.91 on seed 1 in another; HD Dual needed seed 2): rebuild with router1. The LC budget is used up: any addition
  needs a matching cut.

## Deviations
- **Cut for budget**: the wall shade (the red/maroon bands darkening at the chevron knees, |dy| 230–300); the glow as a
  true soft key (it is a fixed pink-to-cream ramp shown where it is deeper than the background, so its tint does not
  follow the background: the still's diagonal glow is bluer); the drain acts on V only; the heat no longer darkens the
  glow to black (the clip's black-rimmed eye at 2 s), the eye's own dark edge and pupil darken instead; the walls are
  not compressed by the heat gain (the clip's doubled lines at 2 s). Two switches were swapped for cheaper ones:
  Mirror (a lemon eye between two ridges) became Facing, and the cat's slit pupil became a black pupil.
- The cream's luma is capped by the shell's white cap (858 → 777 out), so it reads as a deeper yellow than the still's
  near-white cream; most of the remaining error (RTL vs still 16.1 mean RGB; proto 11.0) sits in that band.
- The eye gauge uses p = 2 (an ellipse) instead of the fitted 1.92, so the pupil kite's left vertex and the inner
  rings are rounder than the still's chevron-like inner rings. The rings are regular red/maroon (period 32 px of
  depth); the still's are irregular.
- The eye's top-right corner sits ~13 px lower than the still's; the glow's horizontal extent follows a per-row slope,
  so its left edge is straighter than the still's.
- No camera texture (horizontal smear, noise, the capture border). The left ~6 px come out dark from the output
  filter's warm-up.
- **Resolutions.** Virtual 1080-line coordinates: 720p shows the same picture as 1080p. SD shows the centre 4:3
  (x 240–1680 virtual px), so the outer magenta of both walls falls outside; the eye, both dashed lines and the ridge
  stay in frame. SD 480i fields step 4.45 virtual lines, so the stripes and the T table are coarser there.
