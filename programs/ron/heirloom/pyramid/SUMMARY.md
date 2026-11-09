# Pyramid — a curved pyramid of oscillator stripes drifting over a teal ground of nested arches

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20231124_00_45_44_t5p0.png` (clip `WIN_20231124_00_45_44` at 5.0 s). The still shows a curved-sided
pyramid, like a bullet nose, rising from the bottom edge. It is filled with soft vertical rainbow stripes: green,
yellow-green, crimson, violet, blue and cyan. It stands on a teal ground that is brightest along its edges and falls
off in nested arches to olive-dark sides. Presets are hand-written placeholders.
Working files: `.heirloom_work/rb_pyramid/` (`notes.md`, `proto.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **The key: one ogive.**
  - Apex at (728, 28). The base runs off the bottom of the frame.
  - Half-width at d lines below the apex (left edge, the same in 6 frames to ±10 px): 22 at d = 20, 94 at 140,
    186 at 320, 260 at 500, 345 at 740, 415 at 980.
  - A power law fits: w = 2.29·d^0.755 = 421 px·(d/1000)^0.755. The shape is symmetric about the apex column.
  - Edges: the left edge rises over ~30 px with a faint light rim just inside (±8 levels). The right edge is softer,
    ~60 px, and the stripes fade translucently into the ground. That reads as a slew on the key.
- **Inside: three oscillators patched into R, G and B** (the channel images show three separate structures).
  - **G** is vertical stripes that are fixed on the screen. In all five m-frames the G minima sit at x = 444, 539,
    642, 755, 867 and 953 while the pyramid moves 250 px.
    - The period is 322.75 px (minimum fold residual over all ten frames of the clip).
    - Within a period the shape is a wavefolded wave: a deep trough (~20 / 255), then three peaks (~200), with dips
      (~100) and a shoulder (~140) between them.
  - **B** is a slow horizontal sine: period 660–820 px (700 used), mean ~130, amplitude 66–80.
  - **R** is an egg-crate of wavy bands and ovals, clipped to 0 over large areas.
    - Fit: R = clip(o + am·tri(k·(0.77 cos(2πx/1333) + cos(2πy/896)) + c)), with a dark apex over the first
      ~300 lines.
    - It catches the still's dark apex and its dark C-shaped ring round a bright oval at (850, 580): rms 23
      against an R std of 39.
  - Inside mean RGB is (98, 131, 139), std (40, 56, 53).
- **Ground: keyed by the same shape.**
  - L = 172 − min(0.14·A, 0.219·y − 0.043·|dx| + 84), where A = |dx| − w(d) is the horizontal distance outside the
    edge.
    - Near the key the contours are arches parallel to the edges.
    - A shallow "roof" field takes over in the top corners, so the contours there are nested corners.
    - The darkest spot is on the far side at mid height.
  - Least-squares fits over all ground pixels: rms 4.2 (still), 5.8 (m0) and 2.2 (m3), against an L std of 28–39.
  - The colour is a 1-D ramp of L: olive (59, 60, 41) at L 52, (93, 130, 114) at 112, teal (130, 192, 179) at 172.
  - Left and right agree within 5 L.
- Everything is camera-soft (an out-of-focus capture of a monitor). Small light "^" flecks sit in the core.

## Motion in the clip
- **The pyramid drifts left at a steady ~123 px/s.**
  - The apex x at t −0.5 / 0 / +0.5 / +1.0 / +1.5 s is 792, 728, 665, 604, 528: 58–66 px per 0.5 s.
  - The same rate holds across the clip: 1266 at 0.5 s, 728 at 5.0 s. The second clip, `00_47_03`, goes from 1230 to
    485 over 6 s.
  - The ground's arches travel with it.
- **The G stripes stay put on the screen.** The moving key slides over them.
- **The B wave drifts right**: its peak moves 780 → 1010 px over 2 s (~115 px/s).
- **The R pattern evolves** between bands and ovals at about 0.5–1 cycle/s.
- **Ron's knobs.**
  - The other looks from the same session are the same patch with the pyramid elsewhere: ~10 % wider at 0.5 s.
  - Two minutes later (`00_47_03`) the ground is grey-lavender instead of teal, with the same olive dark end.
    That look became S7 Ground.
- **In the program:**
  - The default Drift (324) moves the pyramid 2.05 px per field left (123 px/s). It wraps with a 2560 px period, so a
    new pyramid enters from the right as the old one leaves.
  - The stripes are screen-fixed.
  - B drifts 2 px per field right; the R phases turn at 2 and −3 rad/s.
  - The integer model at +30 and +60 fields lines up with m1 and m2 (`cmp_motion.png`).

## What it does
1. **Key.** dx = the beam's distance from the apex, in half virtual px. It wraps every period (2560 px, or 960 for
   Twin), so it is a periodic shape oscillator. A = |dx| − w(line).
2. **Half-width per line.** The line engine computes w = W0·(d/1000)^p for the next line: log2 of d by
   normalize + 256-entry mantissa ROM, × p on a serial multiplier, then the 2^x ROM, × W0. Above the apex
   w = −2|d|, so the ground's arches close over the top.
3. **Oscillators.**
   - G = the measured 512-entry waveform at a screen-fixed phase, plus Warp × the source luma.
   - B = 135 + 82 cos (a 700 px phase accumulator drifting right).
   - R = clip(40 + 0.53·tri(gk·(Rx + Ry)·2 + 0.204 + Warp·src) − apex), with Rx = 98 cos(x-phase) per pixel and
     Ry = 127 cos(y-phase) per line. The apex term is 193·(1 − d/300).
4. **Ground.** L as above in Q4 (shift-adds only). Banded: L − 24 on every other 8-level band. A 2 × 256 colour ROM
   (teal / grey).
5. **Mix.** α = clamp((−A)·g + 5)/32, with g = 0.72 per half-px on the left side and 0.27 on the right (the slewed
   edge). out = ground + (osc − ground)·α. A light rim (+6/+8/+10 RGB) in the 10 px inside the left edge.
6. **Colour.** RGB → hardware YUV by three 256-entry contribution ROMs (U = Cr). Luma pre-scaled 16/13 and capped
   at 930 (≤ 768 out).
7. **Output.**
   - U and V go through [1 2 2 2 2 2 2 2 1]/16 with the running-sum resync.
   - Y goes through [1 2 1]/4 on the same centre.
   - Then luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Width | the pyramid's half-width: ×0.35 (a needle spire) .. ×1 (the still, 512) .. ×2.8 (a broad hill that fills the frame); exponential |
| K2 Curve | the edge exponent p: 1.6, a concave spire flaring at the base (0) .. 0.755, the still's ogive (512) .. 0.355, a round dome (1023) |
| K3 Stripes | the G stripe oscillator's period: ×4 broad colour fields (0) .. 322.75 px (the still, 512) .. ×1/4 fine pinstripes (1023) |
| K4 Drift | bipolar pyramid drift, ±8 deadband, square law, up to 16 px per field. 324 = the clip's 2.05 px per field to the left; 512 = the pyramid stands still (the stripes and colours keep moving) |
| K5 Rings | the R shaper's gain: ×1/8, no red, so pure green / blue / violet stripes (0) .. the still (512) .. ×4, dense red rings and ovals (1023) |
| K6 Warp | how far the source bends the G stripes (up to ½ period) and shifts the R rings (up to 1 cycle). 384 is the default |
| S7 Ground | Teal (the still) / Grey: the grey-lavender ground of the clip two minutes later |
| S8 Twin | Off / On: a row of pyramids 960 px apart, each with its own arches |
| S9 Colour | RGB (the still) / GBR: the three oscillators rotated through the guns, giving red / yellow / magenta stripes over a lime field |
| S10 Arches | Smooth (the still) / Banded: the ground's level folded into nested bands that trace both contour families |
| S11 Source | Video / Osc |
| P12 Rise | the apex height: a small striped peak in the lower third (0 %) .. the still (538, apex at line 28) .. a towering trapezoid wall, apex 900 lines above the frame (100 %); glided |

## Video
- **Video mode (the default).**
  - The camera's luma, (Y >> 2) − 16 from a one-line buffer, is patched into the stripe oscillator's phase. Warp
    sets how far: up to half a stripe period for white at full Warp.
  - It also goes into the R shaper's phase, up to one cycle.
  - So inside the pyramid the picture appears as bent and broken stripes and shifted red contours, like a scan
    processor. The ground and the key are untouched, so the composition always reads.
  - With no camera (black input) the stripes are straight: the still.
- **Osc mode.**
  - A drifting 2-D triangle lattice (642 × 541 virtual px, the family's Source oscillator) feeds the same path.
  - At the default Warp it bends the stripes into slow zigzags. The still's stripes have smaller kinks of the same
    kind.
  - Warp stays live in both modes.

## Engine
- **Per-field sequencer (vblank zone).**
  - A shared glide covers K1, K2, K3, K5, K6 and P12.
  - One serial multiplier runs 10 slots: speed², drift step, W0 (exp2 ROM), p, the stripe frequency (exp2 ROM) ×
    step and × the line-start x, the rings gain (exp2 ROM), the apex height (two slopes), the R y-step.
  - The motion: the apex x with the Twin-aware wrap, and the B / R phase offsets.
- **Per-line engine.**
  - Kicked at each line start (and once in the zone for line 0), it computes the next line in ~46 clocks, so it fits
    SD's 858-clock line.
  - Its outputs: w (log2 → × p → exp2 → × W0, one serial multiplier, two-step shift), the roof term 3.5·y + 1344,
    the apex darkening, and the R y-cosine.
- **Beam and pixel path (25 stages).**
  - The beam holds the dx DDA with a precomputed wrap and the R-x phase.
  - The G phase runs late (stage 5) and the B phase later still (stage 8), so neither is carried down the pipe.
  - |dx|, A, ground terms, warp product, G address, R product, ground min, alpha, the R fold, the ground ROM, the
    mix, the matrix ROMs, sums, compose, filters, 13/16, gate.
  - Every multiply has a stage to itself, and every ROM output gets a fabric register.
- **Memories (20 EBR).** Line buffer (4), G / B / R-x / R-y ROMs (4), log2 (1), exp2 × 2 (2), ground colour (3),
  matrix (6).
- **Contracts.** The parity-locked generated avid. One latency of 24 in every mode. Virtual 1080-line coordinates:
  SD shows the centre 4:3. Colours in the hardware convention (U = Cr). The running-sum resync on the chroma box.
- **Verified (GHDL, 720p60, decimation 1).**
  - The default Video frame with a black input is bit-exact with `imodel.py` on every interior pixel (max difference 0).
  - Osc and camera frames match the proto in structure.
  - A K4 = 60 capture shows the pyramid moved at the commanded rate.

## Status
- **v0.1.0 built 2026-11-09 with router2 (the builder's default), all six configs on seed 1. NOT flashed.**
  - Post-route Fmax (builder): HD Analog 88.62, SD Analog 80.28, HD HDMI 84.90, SD HDMI 82.44, HD Dual 82.53,
    SD Dual 84.40 MHz.
  - 6724–6731 / 7680 LC (87.6 %), 20 / 32 EBR. iceunpack OK ×6.
  - Output: `out/rev_b/ron/heirloom/pyramid.vmprog` (477,590 bytes, unsigned).
- **Rebuild:** `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh pyramid` (or `./build_heirloom.sh pyramid`).
  If a later version gets tight, try `NEXTPNR_ROUTER="--router router1"` first. `dupin.py` on the synth json shows
  0 `CARRY I0==I1`, so router1 is safe.
- `hd_clock_divisor` untouched. No pin-skew seed screen was run; if HD HDMI shows lines, re-seed that config.
- **Fit history.** The first netlist went over budget: 7414 LC (96.5 %), with HD Analog 79.0 and HD HDMI 81.2 MHz.
  It had lozenge's two 9-tap boxes on Y, U and V, 24-bit G / B phases carried down the pipe, and C_LAT 28. It was
  stopped after three configs. The cuts:
  - one chroma box, with luma [1 2 1]/4 on the same centre (the REBUILD spec);
  - the G and B phase accumulators run late in the pipe instead of being carried;
  - no spare delay stages, so C_LAT is 24;
  - a narrower sequencer multiplier.
  That gave −606 FF and −156 LUT, and the default frame is still bit-exact with the updated model.

## Deviations
- **R detail.** The R channel is a two-cosine egg-crate fitted to the still (rms 23 / 39). It has the dark apex,
  the crimson / yellow-green stripes at the left, and rings and ovals that evolve. It does not have the still's exact
  C-ring geometry or its thin vertical streaks. Inside mean abs error vs the still is 25 RGB, the ground's 4.6.
- **Not drawn:**
  - the small light "^" flecks and the slight kinks in the core stripes (camera or feedback texture);
  - the faint inner contour lines in B seen in some frames;
  - the dark-then-light edge ringing (only the light rim is drawn).
  The Osc / camera warp stands in for the kinks.
- **Softness.** The still is an out-of-focus camera capture. The program gets its softness from the measured
  (already blurred) G waveform, the sine oscillators and the designed soft key edges, plus the chroma box.
  Luma edges are a little crisper than the photo.
- **Whites.** Drawn luma is capped at 768 out. The still's brightest pixels are ~700, so nothing is clipped at the
  defaults.
- **Edges of the frame.** The left edge fades in from black over ~8 px (filter warm-up). The picture sits 4 px right
  of the input raster (the generated avid).
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680). The pyramid enters and leaves the screen a little earlier, and the
    ground's far sides are cut.
