# Tiger — orange and blue tiger bands fanning across the screen, folding into pink saddles

**v0.1.1 built 2026-11-10, not flashed.** Heirloom (video phase), rebuilt from the clip
`photos/vid/WIN_20231208_23_03_08` at 0.5 s. Presets are hand-written placeholders.
v0.1.1: the camera bend is smoothed (16 px box + vertical IIR on the luma), the default Etch is a quarter of
v0.1.0's bend, and the Osc runs at 24× the camera gain.

## Reading of the photo
Working files are in `.heirloom_work/rb_tiger/` (`notes.md` has the measurements).
- **The picture.** Soft orange / blue / pink bands sweep across the frame. They are near-vertical at the top left
  and twice as dense and leaning at the top right. Near the bottom they bend into a row of pink "X" saddles at
  x ≈ 275, 920, 1560 and pointed "leaf tips" at x ≈ 600, 1240, 1880, all on one fold row (y ≈ 880). Above the
  fold the bands curve like ")"; below it they slant "/". Dark green gaps sit between the bands, with thin red
  glows along the blue edges and dark green lines at the hard edges.
- **Channels.** G is a near-constant floor (75–95, dipping at the hard edges). R and B are two shaded ramp fields
  with hard resets. Over the frame (R, B) fills the square, so there are two fields. Along one row it traces a
  1-D loop, and on the fold row R = B (row correlation 0.89). Along every row max(R, B) is a clean triangle in x;
  which channel carries it is a key. Above the fold, B is high and R low at both the triangle's peaks and its
  troughs (the blue wedges); below the fold it is the reverse (the orange wedges).
- **The patch.**
  - A line-locked H **triangle** oscillator, θ = (x + 166)·f(y) + c(y). Its line-start reset sits 166 px left
    of the picture, so the fan pivots off-screen left.
  - FM from a slow V sine: f = (1 + 1.90·|sin((V − d)/2)|^2.37)/638.8, with V = 2π(880.6 − y)/2754. That gives
    3 periods across on the fold row and 2.2× that at the top, where the period is about 290 px.
  - A **split** w = 0.274·sin V, ADDED to a red colouriser and SUBTRACTED from a blue one. Both colourisers wrap:
    R = frac(T + w), B = frac(T − w + 0.023), where T is the triangle.
  - On the fold row (w = 0) R = B = T: pink peaks and dark troughs. Above it red wraps around the peaks and blue
    around the troughs; below it the reverse. The wedges' width grows with |w|.
  - The fit scores MSE on R and B (`fit5.py`, `fit6.py`). Per-row brute force (`rowgrid.py`) agrees with the
    global model within 0.02–0.06 cycles of phase on almost every row.
- **Camera transfer** (`tune.py`, random search against the still):
  - R = 255·clip((uR − 0.186)/0.739)^1.40 and B = 255·clip((uB − 0.294)/0.585)^1.25;
  - a red glow of 0.58 just after each blue wrap (0.25 cycle wide);
  - G = 57 + 0.117 R + 0.068 B − 32 at the wraps.
- **Palette** (k = 10, share of pixels):
  - orange (231, 70, 8) 14.5 %, brown (98, 62, 12) 14.1 %, mid orange 13.1 %;
  - blue (18, 85, 237) 12.6 %, mid blue 9.7 %, navy 9.0 %;
  - dark green (30, 48, 18) 8.1 %, magenta 7.4 %, violet 5.9 %, pink (202, 80, 182) 5.5 %.
- **Fit at the defaults.** Integer model vs the still at 1920 × 1080:
  - 4 × 4 region means within 10.8 levels on average (worst cell 39);
  - 60 px block luma std 19.9, against the still's 18.8;
  - mean |RGB| error 50.4. This is dominated by the hard wrap edges landing a few pixels off.
  - Palette shares are within 2 points, except brown (9.8 vs 14.1) and pink (9.2 vs 5.5).

## Motion in the clip
`motion.py` refits the still's model to each m-frame, with the fold row, the H phase and the split free:

| clip time | fold row | H phase | split |
|---|---|---|---|
| t − 0.5 s | 764 | 0.680 | 0.05 |
| t | 881 | 0.786 | 0.27 |
| t + 0.5 s | 556 | 0.915 | 0.32 |
| t + 1.0 s | 509 | 0.888 | 0.27 |
| t + 1.5 s | 142 | 0.975 | 0.72 (poor fit) |

- **The fold row rolls up** at about 490 lines/s, 7–9 lines per 60 Hz field. The V oscillator is detuned from the
  field, and the fan rides with it: the cusps climb and the bottom fills with dense "/" bands.
- **The bands creep left.** The H phase advances about 0.13 cycles/s, roughly 80 px/s.
- **Ron opened the split just before the still.** The strip's first three frames (t − 1.5 … t − 0.5 s) are one
  pink/violet fan with thin orange and blue slivers (split ≈ 0.05). That is P12's 0 %.
- **The program's default motion** is a roll of 7.06 virtual lines per field upward plus a drift of 0.0022
  cycles per field (K4 at 690).
  - Model fold row at fields 1 / 31 / 61 / 91: 840 / 660 / 440 / 80. The clip's: 881 / 556 / 509 / 142.
  - GHDL frames 3 and 5 are bit-exact with the model at fields 2 and 4. The fold rises 15 lines between them.
- **The session's other looks.**
  - `WIN_20231208_22_55_57` has two fold rows and a green floor with yellow glows: K3 Folds up and S10 Jungle.
  - `WIN_20231208_22_59_16` has wriggling vertical bands: S8 Wave.

## What it does
1. **Per line** (computed one line ahead), the engine works out:
   - the V phase, sin V and the fan shape (interpolated 256-entry ROMs);
   - the split w = A sin V;
   - F = 1 + aF·fan and the H step f = kb·F;
   - the start phases (x0 + 166)·f + c(y) and (3840 − x0 + 166)·f + c(y);
   - c(y): the lean and the drift, plus the wave if it is on.
2. **Per pixel:**
   - two phase accumulators (A runs up; mirrored, it runs down to the centre and back up; B = A, or reversed
     with Cross);
   - plus the smoothed camera / Osc etch;
   - the triangle T = 2f or 2(1 − f), with no arithmetic;
   - uR = T + w and uB = T − w + 0.023;
   - two 512 × 32 term ROMs. Each holds the {Y, Cr, Cb} contribution of its channel's transfer, glow and edge
     dip, so base + red term + blue term is exactly the BT.601 colour of (R, G, B).
3. **Output.**
   - Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes. They give the camera softness and are the HD HDMI
     bar fix, and they resync to black after 8 blank samples.
   - Luma is scaled 13/16 before the blanking gate. Term luma is pre-scaled 16/13, and the sum is capped at 940.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | the H oscillator's frequency: ×1/4, two broad bands (0) .. the clip's 3 across (512) .. ×3.1, fine stripes (1023) |
| K2 Fan | the FM depth (linear): parallel vertical bands with the X saddles only (0) .. the clip (512) .. twice the clip's fan, dense hooks at the top (1023) |
| K3 Folds | the V oscillator's frequency (×1/4 .. ×4): one long gentle fold, slanted bands (0) .. the clip's single fold row (512) .. about five fold rows of chevrons (1023) |
| K4 Roll | bipolar (±8 deadband, square law): fold rows roll up and the bands creep left (right), or roll down (left); 690 = the clip, full = 62 lines per field |
| K5 Lean | a V ramp into the H phase, pivoting on the fold row: bands lean hard left (0) .. the clip (512) .. upright and leaning right (1023), ±2 cycles over the frame |
| K6 Etch | how far the smoothed camera (or the Osc) bends the bands, square law: 0 .. 0.021 cycles per full luma at 128 (a gentle smooth bend) .. 1.23 at 1023 (floods) |
| S7 Mirror | Off (the clip) / On: the fan mirrored about the centre, giving symmetric chevrons and a central spine |
| S8 Wave | Off (the clip) / On: a 540-line V sine (0.3 cycles) into the H phase, so the bands wriggle like the session's 22_59_16 clip |
| S9 Cross | Off (the clip) / On: the blue colouriser reads a reversed H ramp, so blue bands lean against the orange in a lattice |
| S10 Colours | Tiger (the clip) / Jungle: a green floor instead of the dark gaps, and yellow glows (the session's 22_55_57 clip) |
| S11 Source | Video / Osc |
| P12 Split | the split's depth, square law: 0 % = one pink/violet fan with no orange or blue (the clip a second earlier); 467 = the clip; 100 % = colour bands cycling down the frame into a lattice of diamonds |

**Presets:** Photo, Pink Fan (P12 0), Shattered (P12 100 %, faster roll), Chevrons (Mirror, K2 700), Wriggle
(Wave, K3 700), Lattice (Cross, P12 600), Jungle (Jungle, K3 800), Steep Lean (K5 1000, K2 900).

## Video
- **Video mode (the default).** The camera luma is smoothed, then times Etch it is added to both colourisers' H
  phase, as video patched into the oscillator's phase input. The bands bend around the picture.
  - Smoothing: (Y >> 2) − 16, then a 16 px horizontal running box, then a vertical IIR (1/4 new + 3/4 the
    previous smoothed line).
  - At the default (K6 128: about 13 px at the fold row for full white) the selfie bends the lower bands and the
    X row in smooth curves, with no per-line jaggies.
  - Etch is square law, so 1023 still floods the picture with the camera.
  - With no input, Video mode is the still.
- **Osc mode.** A drifting 640 × 540 virtual-px 2-D triangle lattice replaces the camera, through the same
  smoothing, at 24× the camera gain (1.5× v0.1.0's Osc at the defaults). At the default Etch the bands kink into
  bold zigzags along the lattice ridges.

## Engine
- **Per-field sequencer (vblank zone).** One serial shift-add multiplier, 15 slots. A shared glide covers K1, K2,
  K3, K5, K6 and P12, and a 256-entry exp2 ROM serves the exponential controls. Per field it computes:
  - the H step (exp2 of K1) and the fan depth (K2 × 973 >> 6);
  - the V step per line (exp2 of K3);
  - roll² × the V step, and roll² × the drift constant;
  - the split (P12²), the etch gain (K6², ×24 for Osc);
  - the lean and its per-line step, and the line-0 phase pivoting on the fold row;
  - the wave step.
  - The motion accumulators step once per field.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in about 165 clocks (it fits SD's
  858-clock line) on a second serial multiplier. It does 3 interpolated table reads (sin V, the fan shape,
  the wave sine) and 9 products.
- **Source side (camera smoothing).**
  - (Y >> 2) − 16, or the Osc lattice, then a 16 px running box (resynced to 0 after 16 blank samples).
  - Then a vertical IIR, sm = (box + 3·sm_prev + 2) >> 2, held in the 2048 × 8 line buffer itself. It reads the
    previous smoothed line 2 columns ahead of the write-back, so the buffer carries `no_rw_check` with 0
    transparency emulation.
  - The first line of a field loads the box directly. The previous line's value feeds the pixel path.
- **Pixel path (16 stages).**
  - Phases are carried as bits 23 .. 7. The etch is two nibble partials of luma × (gain >> 7).
  - Then the bent phases, the triangle fold, the colouriser adds and the term-ROM address.
  - Then the EBR read, a fabric copy, base + red + blue, and clamp and compose.
  - The [1 2 2 2 2 2 2 2 1] boxes, 13/16 and the gate follow.
- **ROMs / RAM (15 EBR).**
  - Colouriser terms: 2 × 512 × 32 (8 EBR).
  - exp2, sin and fan: 256 × 16 each (3).
  - Line buffer: 2048 × 8 (4).
- **Contracts.**
  - The generated avid (damier's).
  - One latency, C_LAT 16, in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
  - The running-sum resync on both output boxes and on the camera box.
- **Verified (GHDL, 720p60, decimation 1, defaults).**
  - The black-input frame is bit-exact with `imodel.py` at fields 2 (100 % of pixels, after a constant 15 px
    capture offset). In v0.1.0, frames 3 and 5 were bit-exact at fields 2 and 4.
  - The selfie camera frame is 97 % within 8 levels.
  - Osc is 82 %: the 24× gain amplifies the model's approximate lattice phase; its structure matches.
- **Other checks.**
  - The single-row-line check (`thincheck.py`) finds 0 runs of 24 px or more in every case: default, P12 0 / 100,
    the K1 / K2 / K3 / K5 extremes and every switch.
  - `dupin.py` finds 0 CARRY I0 == I1.

## Status
- **v0.1.1 built 2026-11-10, router2, all six configs on seed 1.**
- Post-route Fmax: HD Analog 76.68, HD HDMI 74.65 (thin), HD Dual 81.18, SD Analog 81.89, SD HDMI 59.91, SD Dual
  84.26 MHz.
- 7061–7074 / 7680 LC (92 %), 15 / 32 EBR. iceunpack OK × 6.
- Output: `out/rev_b/ron/heirloom/tiger.vmprog`. **Not flashed.**
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tiger`. HD HDMI's margin is thin (0.4 MHz), so if a
  rebuild misses, try `NEXTPNR_ROUTER="--router router1"` first.
- **v0.1.0 (2026-11-10).** Etch default 256, unsmoothed camera, Osc 4×. Router2, all seed 1: HD 84.27 / 79.94 /
  84.30, SD 82.51 / 75.69 / 77.32 MHz, 6815–6832 LC.
  - Sent back because the default camera bend gave ragged per-line jaggies on the lower bands.
- **Fit history.** The first cut had a K5 Slope control: the triangle's rise fraction, through a per-pixel
  13 × 8 multiply on both channels. It also had a 24-stage path and came to 7603 LC (98 %, HD Analog missed seed
  1). Dropping the slope multiplier and shortening the path to 16 stages brought it to 6832 LC. K5 became Lean.
  The v0.1.1 camera smoothing added about 240 LC.

## Deviations
- **Band shapes.** The still's "blade" shapes are drawn as regular bands of steadier width. These are the wide
  blue fins with pointed ends, and a few edges crossing diagonally in the upper middle.
- **Colour proportions.** The magenta/violet blobs around the fold are thinner (5.8 / 5.2 % against 7.4 / 5.9 %).
  The X saddles are a little pinker (9.2 % against 5.5 %), and there is less brown.
- **Glows.** The red glows run on one side of each blue wrap; in the still some sit on both sides.
- **Fold row.** It is level; the still's fold row rises about 20 lines across the width.
- **Triangle.** It is symmetric; the fit gives a 0.479 rise fraction. The phase, split and lean were refitted for
  the symmetric triangle with the same score.
- **Motion.** In the clip, Ron also opened the split and moved the fan while the fold rolled. The program rolls
  and drifts at the measured rates with everything else fixed; P12 reproduces the split opening.
- **Camera.** The camera image is blurred (16 px horizontally, about 4 lines vertically) and offset a few px
  before it bends the bands: deliberately, so a live camera bends smoothly instead of tearing.
- **Cut for budget.** The triangle ↔ sawtooth waveform control (K5 Slope) is gone; K5 is Lean.
- **Resolutions.** 720p shows the same geometry as 1080p. SD shows the centre 1440 virtual px (4:3), so the
  leftmost X saddle sits at SD's left edge and the rightmost leaf tip is cut off. SD is also softer, because the
  blur is per output pixel.
