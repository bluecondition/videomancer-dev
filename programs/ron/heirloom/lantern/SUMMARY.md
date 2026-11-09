# Lantern — a hot-pink lantern in crossing petal contours, sinking

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom phase 3 (video clips), from the still at 0.5 s of the clip
`WIN_20231002_01_20_50` (`photos/vid/WIN_20231002_01_20_50_t0p5.png`, 9.1 s clip). The still shows a solid hot-pink
lantern with a pointed apex, wrapped in nested petal contours: thin yellow lines with dark brown cores, purple and
pink between them, crossing each other in X shapes in three fixed rows. The left edge holds the line-start value.
In the clip the lantern sinks out of the bottom and the next one comes in from the top. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_lantern/` (notes.md has the whole chain).
- **Two families of one field.** Every yellow line belongs to one of two families of level sets:
  `f1,2 = A(u) + Vg(y) ± Lh(y)`, lines at integer levels from 2 up (none below: the lantern).
  - The proof: plotted as A(u) mod 1 against y, every line of every clip frame collapses onto two curves
    (`famscatter.py`), and a free spline fit of the two vertical functions gives rms 2.0 px.
  - `A(u) = 2.191e-3 u + 1.115e-6 u²` periods, u = |x − 987| px: an |x| fold plus an x² parabola, 3.1 periods to
    the edge. The left half is 0.7 % wider and sits 0.032 period lower, which makes the photo's step at the apex
    (the left line ends ~7 px higher than the right).
  - `Vg = 0.562 + 2.564 (1 − cos(2π (y − 720.5) / 2500)) / 2`: a vertical sine oscillator, the bowl's valley on
    line 720.5, sub-quadratic toward the top as a sine is.
  - `Lh = 0.962e-3 (y − 517.7)`: a static vertical ramp, exactly linear in the spline fit. The X rows sit where
    Lh = 0, ±½: y ≈ −2, 518, 1038 (the X's cut by the top and bottom edges, and the middle row at x 220, 500,
    1450, 1740).
  - Joint parametric fit on 1700+ line points: rms 4.0 px, median 2.9 px (`fitsine4.py`).
  - In analog terms: a ring oscillator (the |x| + x² + vertical-sine shape through a folder) mixed with a vertical
    bar oscillator and keyed at zero; the X's are its saddles.
- **Colour.** One palette of s = min(e1, e2), e = the distance from f to the nearest drawn level (below the first
  level, the distance to it, so the lantern fills in pink). R is a clean function of s (rms 19 levels; |product|,
  min |sin| or no floor give 55–82).
  - Medians per s bin (RGB): yellow (199, 210, 75) at the line, fading by s 0.04; brown core (44, 20, 2) at
    s 0.06–0.09; blue keys in over s 0.08–0.20 (purple (99, 12, 141) at 0.19); red ramps linearly to pink
    (255, 3, 136) at s 0.48.
  - Shares: pink 19.6 %, brown-black 15.7 %, purples ~40 %, yellows ~10 %.
- **Edges.** Lines ~12 px wide where the field is steep, wider where it is flat. In the left 55 px the field holds
  its line-start value, so contours turn into thick horizontal yellow bars (y ≈ 160, 300–330, 420, 850).

## Motion in the clip
- Whole-frame cross-correlation: the lines move straight down 64–74 px per 0.5 s (~140 px/s), with no horizontal
  motion.
- Per-frame fits (0.5–3.0 s): the bowl's valley moves down at 181–184 lines/s, while A and the tilt Lh stay put.
  The two ring centres move together and their phases drift ±0.18/s in opposite directions, exactly what a moving
  bowl plus a static tilt gives. So the lines move slower than the bowl, and the picture is not a rigid scroll
  (correlation 0.7 after 0.5 s).
- Over the clip the lantern leaves at the bottom (~4 s), the frame becomes an up/down-symmetric X hourglass
  (~5 s, the bowl's ridge mid-screen), and the next lantern enters from the top (7–9 s).
- The program drifts the vertical oscillator's phase: K4 at its default (739) moves the valley 3.02 lines per
  field, 181 lines/s, downward. The proto rendered at −0.5 / +0.5 / +1.0 / +1.5 s moves like the m-frames
  (`cmp_motion.png`); new apex chevrons arrive on the axis at the top as the lantern sinks.
- Alt look of the same patch (`WIN_20231002_01_24_14`): more crossing rows (X rows every ~360 lines, tilt ~1.4×),
  a flatter bowl at times (a tall pink column) and wider black. P12 Lattice and K3 Height reach it.

## What it does
1. **A(u) table.** `A = a u + b u²` in a 1024-entry table by virtual px, refilled each vblank by a second-order
   DDA from the K1 / K2 coefficients. The pixel path reads it at |x − axis| (left half at u (1 + 1/128) with the
   line's Vg − 0.032), held left of 55 virtual px. Twin reads 2u from the nearer of two axes.
2. **Vertical terms, per line.** `Vg = 0.562 + Gv (1 − cos)/2` from an interpolated 1024-entry sine table and
   `Lh = λ (y − 517.7)`; Mirror folds y at the middle first.
3. **Families.** `f1 = A + Vg + bend + Lh`, `f2 = A + Vg + bend − Lh`. Each gives e = floor − f below the floor
   (K5) or its distance to the nearest integer. s = min(e1, e2) indexes the palette (256 steps over half a
   period, four palettes).
4. **Bend.** The camera's luma (or the Osc, ×4), averaged over 4 px and scaled by K6, adds to the field, so the
   contours bend around the picture.
5. **Output.** Luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16 as resyncing running 8-sums, luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Rings | the petal contours' density across: ×0.35 (one giant lantern) .. ×1 (512, the still) .. ×3 (dense petals); the A(u) gain |
| K2 Point | the lantern's shape: round oval rings with no point (0) .. the still (512) .. straight chevrons with sharp points (1023); the |x| / x² blend |
| K3 Height | the vertical bowl (Gv ×0 .. ×1 at 512 .. ×3): a tall pink column over a diamond lattice (0) .. squat lantern with stacked arcs (1023) |
| K4 Drift | bipolar (±8 deadband, square law): the lantern sinks (right) or rises (left); 739 = the clip's 181 lines/s; full = 960 lines/s |
| K5 Lantern | the first drawn level, i.e. the lantern's size: inner rings shown inside it (0) .. the still (2.0, at 512) .. 1.6× the level, swallowing two rings (1023) |
| K6 Bend | how far the camera (or the Osc) bends the contours: 0 .. 1.5 periods per full luma, square law; 300 = 0.13 |
| S7 Twin | Off (the still) / On: two half-width lanterns side by side |
| S8 Ink | Neon (the still: yellow lines, brown cores, pink bands) / Night: the lines glow gold-pink on a black ground |
| S9 Gel | Magenta (the still) / Cyan: cyan lantern, deep blue ramp, orange lines |
| S10 Mirror | Off (the still) / On: the bottom half mirrors the top, a diamond lantern with crossings at the centre line |
| S11 Source | Video (default) / Osc |
| P12 Lattice | the tilt between the two families: 0 = one family, a clean nested lantern with no crossings; 400 = the still (39 %); 1023 = 4×, a dense crossing net of arcs and diamonds |

## Video
- **Video (default).** The camera's luma (Y >> 2 − 16, 4-px average) is added to the field before the families
  are taken, scaled by K6. Contours bend around bright areas the way a luma CV patched into the shape's sum does.
  At the default (0.13 period per full luma) the lantern and its crossings read clearly. With no input (black)
  the picture is the still exactly.
- **Osc.** The shell's drifting 2-D triangle lattice (640 × 540 virtual px) feeds the same path at 4× the camera
  gain, so the contours ripple into a quilted diamond pattern. K6 scales it too.

## Engine
- **Vblank sequencer** (zone = no avid for 1024 clocks), one serial shift-add multiplier, 12 slots loaded at zc
  64 + 24k:
  - the two-slope curves (ring gain, lattice gain, floor);
  - a = KA g1 2p and b = KB g1 2(1 − p) in Q28;
  - Gv, λ, speed², the phase step, and the bend gain.
  - Then the A(u) table fill, zc 400–1423: A += D, D += 2b, saturating past 16 periods, and the line-0 kick at
    zc 1440.
  - Shared glide on K1, K2, K3, K5, K6 and P12.
- **Line engine**, kicked at each line start, computes the NEXT line on its own serial multiplier (~75 clocks,
  fits SD's 858):
  - the mirror, then phase = y′·KP − ph0;
  - two sine-table reads, interpolated with an 8-bit fraction;
  - Lh = λ |y′ − YL| with sign;
  - Vg = G0 + Ci Gv, plus Vg − 0.032 for the left half.
- **Pixel path (20 stages = C_LAT = processing_delay_clks):** hold, axis distance (twin: nearer of 960 / 2880 h),
  |d|, index (left ×(1 + 1/128)), table address, EBR, fabric copy, G, + bend, f1/f2, floor test and fraction, e1/e2,
  min, palette address, palette EBRs, fabric copy, compose (neutral black off the generated avid), then output
  conditioning (3 stages) and the gate.
- **Memories:** A table 1024 × 16 (4 EBR, `no_rw_check`), sine 1024 × 16 (4), palettes 3 × 1024 × 8 (6), line
  buffer 2048 × 8 (4) = 18 EBR.
- **Contracts:**
  - damier's generated avid; the beam starts 8 px in (x0 + 8 xs), the generated avid's lag, as in lozenge.
  - one 20-stage latency in every mode; the blanking gate one stage before the output register.
  - output conditioning with the per-blanking resync.
  - virtual 1080-line coordinates: SD shows the centre 4:3, with the axis on pixel 374.
  - palettes in the hardware convention (U = Cr) by `lantern.py`, luma pre-scaled 16/13, white cap 940 → ~777
    out.
- **Checks:**
  - GHDL analyse / elaborate / synth front end clean (with the package).
  - `dupin.py`: 0 self-add carries. Standalone yosys: 2546 LUT, 18 EBR, one transparency emulation (the line
    buffer's).
  - The RTL Video-black render at 720p60 decimation 1 is **bit-exact with `imodel.py`** (all 720 rows).
  - Two Drift = 1023 captures 8 fields apart (frames 3 and 11) are bit-exact with the model at 2 and 10 phase
    updates, so the motion engine is verified.
  - The sim caught one bug, a narrowing `resize(signed)` that folded |y − YL| ≥ 2^17 and zeroed the tilt on the
    first four lines. It is fixed by slicing.

## Status
- **v0.1.0 (2026-11-09), router2** (`MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh lantern`, which does a
  scoped clean and then `./build_heirloom.sh lantern`). **Build seeds: SD Dual = seed 2** (seed 1 stalled the
  router and timed out); the other five are seed 1.
- Post-route Fmax: HD Analog 92.73, HD HDMI 87.72, HD Dual 97.41, SD Analog 95.68, SD HDMI 94.16, SD Dual 86.68 MHz
  (HD Fmin 74.25, SD 27).
- 5114–5133 / 7680 LC (67 %), 18 / 32 EBR. All six bitstreams passed iceunpack.
- Output: `out/rev_b/ron/heirloom/lantern.vmprog`. **Not flashed.**
- If a rebuild stalls on router2, use `NEXTPNR_ROUTER="--router router1"`. There is plenty of LC headroom: ~2,000
  LC are free for later phase-2 additions.

## Deviations
- The apex hook (the right line's small curl into the step) is not drawn, only the step itself. The dark notch
  above the apex is shorter than in the still.
- The camera capture's black borders (left ~4 px, bottom ~8 px) are not drawn. The left ~5 px come out dark from
  the output filter's warm-up.
- The bottom X row lands ~20 px lower than in the still: the families cross at a shallow angle there, so a small
  tilt error moves the crossing a lot. Overall line positions are within ~3 px (rms 4 px).
- Lines are a touch smoother than the camera's. The X crossings' black wedges are slightly smaller (min of
  distances; the camera adds blur).
- **Period.** The vertical oscillator period is the still's fit (2500 lines). At the clip's speed a new lantern
  arrives every 13.8 s, while the clip's next lantern appears ~2–3 s sooner, implying ~2050 lines. A 2000-line
  period fits the still worse (rms 5.8 vs 4.0 px), so the still won.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 4:3 (1440 virtual px), so the
  outer petal pairs and the left hold strip fall outside the screen. Interlaced modes draw both fields with the
  same lines.
- With non-integer floor values (K5 away from 512), the lantern's edge line is one-sided (half width), a keyed
  look.
