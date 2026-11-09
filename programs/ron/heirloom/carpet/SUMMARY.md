# Carpet — a red / white / orange kaleidoscope carpet of black-cored diamonds and lenses

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the still
`photos/vid/WIN_20230722_01_18_41_t5p5.png` (clip WIN_20230722_01_18_41 at 5.5 s; the same patch is in 01_21_34 and
01_26_41). Tall black-cored diamonds ringed in white and red stand in two columns above and below a mirror row, flat
black lenses sit between them, and where the two ring families meet they cross in X's and close into small bow-tie
stars. V-zigzag bands run across the top and bottom, and H-sync jitter streaks the diamonds. Presets are hand-written
placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_carpet/` (`notes.md`, `fit1.py` / `fit3.py`, `prof1d.py`, `proto.py`,
`ctrl.py`, `imodel.py`, `sheet_model.py`).
- **Symmetry.** Luma flip correlation puts mirror axes at x 538 / 1012 / 1483 (0.84 / 0.95 / 0.83) and y 516 (0.96):
  a horizontal period of 944 px, diamond columns at 539 and 1483, lens columns at 64 and 1012. The diamond rows
  (y 258 / 774) are not mirror axes (0.46): the diamonds are taller above their widest row than below.
- **Not a sum.** A separable fit of the contour slope (log slope = a(x) − b(y)) leaves 0.50 of 0.66 rms: the contours
  are not H + V. They are the **minimum of two diamond fields**, an analog non-additive mix:
  - P1 = |x − 539| + 0.687 (258 − y) above / 0.416 (y − 258) below: tall, asymmetric diamonds.
  - P2 = 0.238 |x − 1012 (mod 944)| + 1.245 |y − 516| below the ridge row 253, and a quadratic cap above it
    (1.245·263 − 0.788 dr − 0.00478 dr²): flat lenses, and the zigzag bands of a lens row above the screen.
  - F = min(P1, P2). Fitted by white-mask IoU over six clip frames (3.5–6.0 s) with a per-frame ring gain (`fit3.py`).
    At the white bands, the inner diamond rings are 100 % P1, the lens rings 100 % P2, and the X centres 62 % P1:
    the X crossings and bow-ties sit on the P1 = P2 ridge.
- **Palette** (k-means, k = 10): black (12, 8, 9) 33.5 %, red (245, 17, 6) 13.7 %, white (240, 227, 227) 10.4 %,
  orange (246, 77, 20) 7.8 %, yellow (243, 171, 17) 4.1 %, plus blur mixes (pink 232/180/185, dark reds).
- **Ring cycle** (down the lens column, `lens_prof.npy`): red, yellow (6 px), dark, WHITE (10 px), dark, red/orange
  (14 px), a thin black (2 px), red (14 px), dark, WHITE (12 px), dark, yellow/orange, red. The lenses repeat it with
  a black gap; the diamonds draw it, a dark line, a second copy, then black.
- **Texture.** Streaks in the diamonds: horizontal autocorrelation falls to 0.5 at 13 px, vertical at 9 lines; yellow
  runs median 11 px, p90 22 px: H jitter of the diamond oscillator. The left ~95 px of every line are flat (line-start
  hold). Soft horizontal edges.

## Motion in the clip
- **The geometry never moves.** The mirror axes stay at x 538 / 1011–1012 / 516 in every frame from 1.0 to 11.0 s.
- **Ron sweeps the ring gain.** Along x 538 above the diamond: one white band at 4.0 s (120–156), two at 5.0 s,
  two (84–108, 168–185) in the still, three at 6.0 s, four at 6.5 s, then the black fills in (0.5–2.0 s and 6.5–11 s
  are high-gain looks). New bands are born on the P1 = P2 ridge and flow into the black cores: the core edge goes
  88 → 74 → 52 → 12 px above the diamond row over 1.5 s (~50 px/s while the knob turns). The m-frames at +0.5 / +1.0 /
  +1.5 s match the model at P12 ≈ 520 / 640 / 760 (`cmp_clipsweep.png`).
- **In the program** that knob is **P12 Folds** (409 = the still). The default motion is K4 Speed at 680: the ring
  phase drifts so the bands flow slowly into the cores, one ring cycle per ~6.6 s (~12 px/s at the still's band
  spacing), the way a slowly turning gain knob reads, with the geometry still. GHDL captures 6 frames apart at full
  Speed show the bands stepping toward the cores (`rtl_motion.png`).

## What it does
1. **H oscillators.** Two sawtooth DDAs (Q8 phase units) at the shared H phase, folded into triangles: P1x = G1·du
   (diamonds) and P2x = G2·(per/2 − du) (lenses). R1's fold decision folds both (same phase). Held left of x = 95 px;
   the jitter adds ±j·J (j redrawn every 16 virtual px from an LFSR) to P1x on the rising side and subtracts on the
   falling side.
2. **V terms**, per line (line engine): V1 = g·kA·(258 − y) above / g·kB·(y − 258) below; V2 = g·k2b·|y − 516| below the
   ridge row, g·(Ft − qa·dr − qb·dr²) above. Rows Double runs y at twice the rate; Stagger moves the lens rows half a
   cell. Both carry − gap/2.
3. **Weave.** F = min(P1, P2) (Max: max); the winner sets the domain. Etch adds camera (or Osc) luma × gain.
4. **Colouriser.** t = F (4096 phase units per ring cycle), q = frac(t + flow) / (1 − gap) as (frac × 6-bit
   mantissa) << shift. Lenses: q < 1 → the coloured part A[q], else black. Diamonds: A[q], a dark line, a second copy,
   then black; black in the core (t < 0) and beyond ND cycles. A is a 256-entry palette ROM (Ember / Ice).
5. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the photo's softness and the HD HDMI bar fix,
   with the running-sum resync), luma 13/16, the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Diamonds | the diamonds' width (the diamond oscillator's H slope, ×2.5 .. ×0.4): 0 = thin spindles, 512 = the photo, 1023 = broad flat diamonds that push the lenses into slivers |
| K2 Zigzag | the lens and zigzag-band slope (the lens oscillator's H slope, ×0.33 .. ×3): 0 = nearly flat bands, 512 = the photo, 1023 = steep zigzags and diamond-shaped lenses |
| K3 Tiles | diamond columns across the screen: 1.0 (0) .. 2.03 (512, the photo) .. 6.1 (1023); the screen centre keeps its phase |
| K4 Speed | ring flow, bipolar (±8 deadband, square law): + = the bands flow into the cores, − = out of them; 680 = the default drift (one ring cycle per ~6.6 s), full = 1.5 cycles/s |
| K5 Jitter | the diamond oscillator's H-sync jitter: 0 = clean diamonds, 300 = the photo's streaks (±3 vpx), 1023 = shredded (±32 vpx) |
| K6 Etch | how far the camera (or the Osc) bends the field, square law: 0 = none, 320 = default (≈ 0.16 ring cycle at full luma), 1023 = ≈ 1.7 cycles, the picture carved into the rings |
| S7 Rows | Mirror (the photo: one row of diamonds above and below the centre line) / Double: the vertical cell at twice the rate, four rows of half-height diamonds |
| S8 Weave | Min (the photo: the nearer pattern wins) / Max: the far pattern wins: bow-tie stars and lens shells floating on black |
| S9 Palette | Ember (the photo's red / white / orange) / Ice (blue, cyan, white) |
| S10 Stagger | Off (the photo) / On: the lens rows slide half a cell onto the diamond rows, bands crossing the diamonds' waists |
| S11 Source | Video / Osc |
| P12 Folds | the ring gain and the black duty (Ron's knob in the clip, glided): 0 % = single thin outlines on black, 409 = the photo, 100 % = up to 8 dense nested ring groups, no black left |

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, times the Etch gain, is added to
  the field before the colouriser, in both the diamonds and the lenses: the rings bend and fray around the picture.
  At the default Etch the carpet's composition still reads clearly. With no input, Video mode is the photo.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting, feeds the same path: the bands tilt and wobble
  gently at the default Etch; Etch scales it.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier (36 × 16, 25 slots of 32 clocks, slot number
  decoded two clocks ahead, operands pre-muxed), a 256-entry exp2 ROM read just before each slot, a shared glide for
  K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field). Per field: the ring gain 2^lg (P12), G1 / G2 (K1, K2 × gain),
  the H phase step and period (K3), the sawtooth wraps W = G·P/16, the start phases at x = 95 px (or SD's 240 px), the
  V-term gains (kA, kB, k2b, qa, qb, Ft × gain), the jitter step (K5²), the etch gain (K6²), the flow step (Speed²),
  the gap and 1/(1 − gap) (a mantissa / shift ROM) and ND.
- **Line engine**, kicked at each line start, computes the NEXT line's V1 and V2 in ~110 clocks on a 32 × 16 serial
  multiplier (fits SD's 858-clock line).
- **Pixel path** (C_LAT 20 in every mode): fold → jitter / P2x → + V → compare & select → + etch → frac + flow, kill
  → ×mantissa → shift → second-copy index → codes and ROM address → palette ROM → fabric copy → compose → two box
  filters → luma 13/16 → gate. The beam runs 8 pixels ahead of the boxes' delay.
- **Block RAM** (14 / 32 EBR): line buffer 2048 × 8 (4), palette 512 × 32 (4), exp2 and 1/(1 − gap) ROMs (2), and the
  output boxes' 8-back taps as two 256 × 32 rings (4, `no_rw_check`, read 6 behind the write plus the EBR and fabric
  registers) instead of 432 flip-flops.
- **Contracts:** damier's generated avid; one latency; the blanking gate; the HD HDMI output conditioning with the
  running-sum resync; virtual 1080-line coordinates (`f_vstep_x/y`); colours in the hardware convention (U = Cr,
  `carpet.py`); drawn whites ≈ 777 after the 13/16 scale.
- **Verified (GHDL, 720p60 decimation 1, real timing ID):** against `imodel.py` with the sim's flow phase and the
  tester's 4-px capture offset, the zigzag-band rows are bit-exact and the lens rows 99.5 %; the diamonds differ only
  by the jitter realisation. RTL mirror axes 363 / 677.5 / 992.5 px and row 344.0 (= the fitted 539 / 1011 / 1483 vpx
  and 516 vpx).

## Status
- **v0.1.0 built 2026-11-04, NOT flashed.** ROUTER1, all six configs on **seed 1**:
  HD Analog 99.00, HD HDMI 104.28, HD Dual 99.21, SD Analog 92.79, SD HDMI 102.04, SD Dual 94.27 MHz.
  6,777–6,794 / 7,680 LC (88 %), 14 / 32 EBR. Every bitstream passed iceunpack. dupin: 0 `CARRY I0==I1`.
  Output: `out/rev_b/ron/heirloom/carpet.vmprog`.
- Rebuilt 2026-11-09 from the same sources through the shared queue (router1): identical seeds, Fmax and LC
  (deterministic), so the shipped sources are exactly the ones in the vmprog.
- Rebuild: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh carpet`.
- Fit history: the first netlist was 7,663 LC (99 %; router2 missed HD Analog on seed 1, cancelled). The output boxes'
  taps moved into block RAM, narrower serial multipliers, a mantissa / shift 1/(1 − gap), compare + select in one stage
  and shared folds brought it to 6,784 LC with the same picture.

## Deviations from the photo
- The outer "spider-leg" ring's kink (where the diamond's outer arm turns into the top zigzag band, ~(360, 95)) is a
  straight diamond side up to where the lens field takes over.
- The bow-ties at (1012, 253) come out as an X crossing of the lens and diamond rings with a small star between, not
  the photo's closed 4-point star.
- The photo's purple chroma fringes at the diamonds' side vertices (capture / encoder chroma lag) are not drawn.
- The yellow speckle inside the diamonds is the coloured part's yellow sub-band broken up by the jitter streaks; the
  photo has more yellow and noise there. The diamonds carry more red relative to white than the photo's.
- Whites are capped at ≈ 777 output luma (the shell's 13/16 scale). The left edge fades in from black over 8 px
  (filter warm-up).
- **Resolutions.** 720p shows the same picture as 1080p. **SD shows the centre 4:3** (240–1680 virtual px): both
  diamond columns and the centre lens are on screen, the left lens column and the left-edge hold fall outside. 1080i
  sources render both fields alike (field_n is tied high on hardware), so 1080i draws the 540-line picture
  line-doubled.
