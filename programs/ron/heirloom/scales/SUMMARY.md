# Scales — crescent fish scales: a bowed saw VCO, cyan lobes swelling on a travelling wave

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the clip
`WIN_20230712_00_46_37` at 2.5 s (`photos/vid/WIN_20230712_00_46_37_t2p5.png`). Ten-plus "(" crescent scales
across the frame, bent harder toward the right; each scale runs pink → salmon → tan into a cyan (lower half) or
dim teal (top left) crescent that ends on a dark maroon line, with red fringes in the olive bands at the top,
the seam and the bottom. In the clip the cyan lobes swell and fade, lower then upper, on a 5.35 s cycle.
Presets are hand-written (Big Crescents is the clip's 0.5 s look).

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_scales/` (`notes.md` has every number).
- **Scale lines.** 3000 detected dark-line points fit one phase field with rms 0.017 cycle (≈ 3 px)
  (`fitgeo.py`): φ = f(y)·(x + 194.8) + 0.455 + bend, f = (1/177.0)·(1 − B·d²), d = (y − 494.8)/540,
  B = 0.272 above the vertex and 0.206 below. In each row the lines are equally spaced: period 216 px at
  y 40, 176 px at y ≈ 495, 227 px at y 1060. The sync point 194.8 px left of the picture is the HD back
  porch: a saw VCO hard-synced at hsync, frequency-modulated by a V parabola.
- **Pairs.** Alternate scales differ: the lines shift ±2.5 px and the key threshold ±0.073–0.093 cycle, so the
  cyan diamonds come in pairs (a divide-by-2 of the VCO into the keyer).
- **Key.** Cyan where the saw passes a threshold k (`keyk.py`, `optK.py`, rms 0.022):
  k = 0.924 − 0.612·tri((y − 766)/635) − 0.457·(y − 540)/540 + 0.363·x/1000 ± 0.073. Vertex 0.155 at y 780
  (the diamonds, rows 600–1000) and 0.718 at y 135 (the teardrops); the fill is cyan with G = B ≈ 95 + 180·(saw
  − k), R ≈ 0 (16, 139, 136 → 0, 218, 216); the upper lobe is only ~50 % opaque (67, 90, 86).
- **Bend.** The lines kink at the lower lobe's vertex: an extra −0.1 cycle phase at y 780 = −0.137·(1 − k_row).
- **Bands.** A B-channel proxy gives P = 82 + 106·|sin(π(y − 533)/510)| − 43·x/1000 ± 8: pink humps at y ≈ 280
  and 790, olive at the top, the seam (y 500–540, a sharp V) and the bottom, widening to the right.
- **Colour.** 6 rows (olive → pink) × 64 saw steps, the medians of the still's pixels re-phased on the locally
  detected lines (`lutsharp.py`): pink (200, 116, 154) → salmon → tan (195, 139, 87) → brown → the line core
  (88, 20, 22) in the pink zones; olive (130, 100, 30) → yellow-olive (140, 118, 10) → orange → red
  (172, 50, 36) in the olive bands. Colour purity per (saw, P) bin: rms 20.7 (the (saw, y, x) floor is 15.4).
- **Line profile.** Luma falls from 100 to 39 over ~20 px to the left of each line and rises to 120 within
  8 px on its right (a saw reset).
- **Texture.** Faint striations parallel to the scales (5–8 per period, 3–5 luma levels), mostly in the right
  two thirds; soft, slightly jagged key edges.

## Motion in the clip
All 180 frames re-extracted at 20 fps (`fr/`, `motion*.py`, kymographs).
- **The scales stand still.** Per-row periods change < 4 % and line positions < 15 px over 2 s.
- **The cyan lobes swell and fade, lower then upper.** The key area in the upper and lower halves has an
  autocorrelation peak at 5.05–5.45 s: lower peaks at t 0.0 and 5.2 s, upper at 1.0 and 6.6 s, quiet between.
  Each lobe drifts up while it lives (upper 349 → 137 lines in 2.4 s, lower 860 → 730).
- **Fit.** A long V sine (λ 2800 lines) travelling up at 523 lines/s (= the 5.35 s cycle) through the key
  threshold reproduces the series (`motion4.py`), but a pure sine can't hold the still's own moment (the clip
  keeps a moderate lower lobe through its quiet phase). The program anchors the still: depth 0.40, the
  wave's phase at power-up puts the lower lobe near its crest. From there the lower lobe holds and shrinks while
  the upper one swells (`cmp_motion.png`), then both repeat every 5.35 s; the clip does the same more slowly
  after the still (its next lower swell comes 2.7 s later).
- **The other looks** of the clip (0.5 s: big cyan in both halves; 5.5 s: a big lower lobe, teardrops above)
  are other phases of the same swell; the Big Crescents preset is the 0.5 s look.

## What it does
1. **Scale VCO.** Per line, f = F0·(1 − B·d²) (Tiled: d folded into three 360-line rows); the phase at the
   first pixel is f·(x′ + 194.8) + 0.455 + bend, and a 25-bit DDA steps it across the line. Its saw is the scale.
2. **Key.** k = Kc − 0.592·tri(lobes) − 0.40·sin(wave) + 0.263·x/1000 ± 0.093 (pairs, from the VCO's
   parity bit); the lobe triangle (period 620 lines) and the wave (2800 lines, travelling up) are per-line phase
   accumulators. Cyan where the saw passes k: the fill is the ramp (saw − k) (the key ROM), the edge a 0.2-cycle
   soft blend, capped by an opacity 3·(E − k(x)) = 2·(1 − k at the lobe tip) − 0.79·x/1000 (dim teardrops,
   dimmer right lobes).
   The same key CV pulls the VCO phase back by 0.137 cycle at full depth (the kinks).
3. **Colour.** P = 82 + 106·|sin| − 43·x/1000 ± 8 picks the band row; a 1024 × 32 ROM holds 2 palettes × 6 rows
   × 64 saw steps (the still's medians). The row is dithered between its two neighbours, and the key against
   the band, on a 16-pixel bit-reversed ordered pattern along the line; the output boxes average it into the
   lerp / blend (a fine grain remains on the key edges, like the still's slightly jagged ones).
4. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the still's softness and the HD HDMI bar
   fix; their 8-back taps in block-RAM rings, with the running-sum resync), luma 13/16, the blanking gate.
5. **Motion.** At the default Drift the wave travels up 523 lines/s: the lobes swell and fade, lower then
   upper, every 5.35 s; the scales stay put, as in the clip.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Scales | the number of scales across: ×1/3 .. ×3 (exponential), 512 = the still's 10.8 |
| K2 Bow | the scales' curvature (the V parabola into the VCO): 0 = reversed ")" arcs, 300 = straight vertical scales, 512 = the still, 1023 = ×3.4, scales wrapping into long horizontal tongues |
| K3 Lobes | the cyan lobes' spacing: 0.43 (one huge crescent band) .. 7 lobes per frame (a lattice of small diamonds); 512 = the still's 1.74 |
| K4 Drift | the travelling wave that swells the lobes: bipolar, ±8 deadband, square law; 512 = still, 768 = the clip (up, 5.35 s), 1023 = ~4×; left runs it down |
| K5 Bands | the pink / olive bands: 0.5 .. 8.5 humps per frame (2.1 at 512); high = horizontal pink / olive striping |
| K6 Warp | how far the camera (or the Osc) bends the scales and opens the cyan: 0 .. 1.2 cycles at white (square law), 400 = 0.18 |
| S7 Wave | Saw (the still) / Tri: the VCO folded, every scale mirrored into a lens with a cyan eye and doubled lines |
| S8 Mirror | Off / On: the H ramp folded at the centre (x′ = 960 + |x − 960|), the right half mirrored onto the left, scales bowing out of a seam (a bow tie) |
| S9 Rows | One (the still) / Tiled: the parabola folded into three 360-line rows, stacked fish-scale armour |
| S10 Colours | Koi (the still: pink, olive, red, cyan) / Reef (the complementary hue: blue-green scales, coral crescents) |
| S11 Source | Video / Osc |
| P12 Crescents | the key level: 0 % = no cyan, plain pink / olive scales with dark lines; 460 = the still; 100 % = cyan floods every scale, both halves, kinked lines |

**Presets:** Still; Big Crescents (the clip's 0.5 s look, P12 720); Armour (Tiled, more scales, deeper bow);
Bow Tie (Mirror); Lens Eyes (Tri); Reef; Reversed (K2 40); Fast Tide (K3 700, K4 1000).

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, is added to the VCO phase
  (luma × gain) and 1.5× the same amount lowers the key threshold. The scale lines fray and tear along the
  picture's edges and bright areas open cyan; at the default Warp the composition still reads. With no input,
  Video mode is the still.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting ~75 px/s, feeds the same path at twice the
  depth: the scales bend in angular waves and the lobes swell into the lattice's bright cells. Warp stays live
  in both modes.

## Engine
- **Per-field sequencer** (vblank zone): one serial shift-add multiplier, 17 slots (the slot number is
  decoded into a registered one-hot and the operands built by a flat AND-OR, no enables); a shared glide
  (1/8 per field) on K1, K2, K3, K5, K6, P12; a 256 × 16 exp2 ROM. It computes F0, the bow coefficients,
  the lobe / band phase steps and line-0 phases, speed² → the wave rate, the wave step, Kc, Warp² → the luma
  gain and the H-ramp starts at x′ (Mirror).
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in ~125 clocks (fits SD's 858) on a
  second serial multiplier: d², q = d²·B, f = F0·(1 − q), f·(x′ + X0), Dk·T, A·sin (256 × 16 sine ROM),
  VA1·|sin|, then Kb, the bend, the line-start phase / key / band values and the opacity reference
  E = Kb + (2/3)(1 − kmin) (five shift-adds). One op per state.
- **Pixel path (15 stages, C_LAT 20).** Stage 1 counts pixels (line-buffer address, the mirror point); the
  phase / key / band DDAs run at stage 5 where the luma × gain arrives (nibble products at 4, sum at 5), so
  nothing rides stages 2–5. 6: phase + warp, E − k. 7: saw (Tri fold), key − 1.5 warp, opacity ×3. 8: pairs,
  opacity Q5. 9: d = saw − k, row clamp. 10: 5d, key level, dithered row → colour ROM / key ROM addresses.
  11: ROM data, blend weight. 12: fabric copies, key decision. 13: band or key colour. 14: carry. 15: compose.
  Then box A 16/17, box B 18/19, luma 13/16 at 20, gate 21.
- **ROMs / RAMs (20 EBR):** colour 1024 × 32 (8), key 128 × 32 (2), exp2 (1), sine (1), line buffer 2048 × 8 (4),
  two box rings 256 × 32 (4, `no_rw_check`).
- **Contracts:** the generated avid (parity-locked); one latency (C_LAT 20) in every mode; virtual 1080-line
  coordinates (SD shows the centre 4:3); colours in the hardware convention (U = Cr) from `scales.py`, luma
  pre-scaled 16/13, capped 940; output conditioning with the running-sum resync before the blanking gate.
- **Verified (GHDL 720p60, decimation 1).** The black-input Video frame is **100 % bit-exact** with the
  integer model `imodel.py` (capture offset 4 px). Osc and camera frames checked by eye (`cmp_final.png`).
  That RTL frame vs the still at 720p: mean colour (146, 103, 63) vs (146, 100, 65); 4 × 4 region means within
  2–18 levels (median 9); cyan in the lower half 16.7 % vs 11.8 %. The control sheet (`sheet.png`) and the
  motion frames (`cmp_motion.png`) are rendered by `imodel.py`.
- **Fit and timing history.**
  - The first full pixel path (six lerp / blend multipliers, FF box delays, DDAs carried through stages 2–5)
    packed at 7388 LC standalone (≈ 8800 in the build). The block-RAM box rings, the late DDAs, the ordered
    dither in place of the multipliers and narrower carries brought it to 5556 standalone (≈ 6960 built).
  - That build missed HD Dual on seed 1 (73.3 MHz): the sequencer's operand pre-mux (z_sel → 20-way case → the
    z_fa / z_fb set / reset pins, 11.9 ns routing). The one-hot AND-OR fetch fixed it (HD Analog 74.8 → 84.6).
  - At 7018 LC HD HDMI then failed placement on seeds 1 and 3 ("unable to find legal placement", all 8 global
    buffers used). The ripple (the striations) was cut and the opacity DDA replaced by 3·(E − key ramp):
    ≈ 6930 LC, and HD HDMI placed on seed 1.
  - router2 then stalled on SD HDMI (overused = 2, seeds 1 and 2); router1 routed it in a minute, so the
    program is built with router1.

## Status
- **v0.1.0, built 2026-11-04 with ROUTER1** (`NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900
  .heirloom_work/vid/build_queue.sh scales`). Not flashed.
- Post-route Fmax: HD Analog s1 89.90, SD Analog **seed 2** 84.28 (seed 1 stalled the router), HD HDMI s1 89.90,
  SD HDMI s1 89.90, HD Dual s1 89.90, SD Dual s1 80.03 MHz. Every HD config passes 74.25 MHz on seed 1.
- 6922–6933 / 7680 LC (90 %), 20 / 32 EBR. iceunpack OK × 6; the packed config carries the labels and the
  8 presets. `out/rev_b/ron/heirloom/scales.vmprog`.
- **Stay on router1:** router2 stalled on SD HDMI (overused = 2 on seeds 1 and 2) where router1 routed it in a
  minute. The synth netlist has 0 `CARRY I0==I1` (dupin), so router1's self-add hang doesn't apply.
- No pin-skew seed screen was run. If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Motion phase.** A travelling sine can't hold the still's own moment, so the default frame is anchored on
  the still and the swell runs from there: the lower lobe holds for ~0.5 s and shrinks while the upper one
  swells by ~+1.5 s, then the 5.35 s cycle repeats. The clip instead stays quiet for ~2.5 s after the still
  (its next lower swell comes 2.7 s later). Period, direction and the lower → upper order are the clip's.
- **Lobes.** The lower diamonds are a little fuller (16.7 % of the lower half cyan vs 11.8 %); the upper dim
  teal teardrops are smaller and pointier than the still's rounded ones.
- **Key edges.** The soft left edge of each crescent comes from an ordered dither averaged by the output
  boxes, so it carries a fine stair-step grain (the still's edges are smooth but slightly jagged).
- **Colour zones.** The colour is a function of (saw, band level) only, so the seam band on the right comes out
  olive / red like the top and bottom bands where the still shows mauve-brown; the colour rows are medians of
  the still (rms 20.7 per bin against a 15.4 floor), a little softer than the still's pinks.
- **Cut for budget:** the faint striations (a ±24-level luma ripple at 8× the scale frequency, ~100 LC) and the
  small horizontal jogs / wiggles of the left-hand lines (y ≈ 220, 380); only the lobe kink (bend) is drawn.
  The still's ±0.2-row vertical tilt of the bands was dropped too.
- **Brightness.** No drawn colour gets near the shell's white cap (the brightest pink is ~650 output luma).
- **Rates** are per field: 5.35 s at 60 fields/s, 6.4 s at 50.
- **Resolutions.** 720p shows the same geometry as 1080p. **SD shows the centre 4:3** (virtual x 240..1680), so
  the leftmost diamond pair is cut and the right edge loses its tightest arcs. The dither grain is 1 SD pixel.
- **Left edge.** It fades in from black over ~8 px (filter warm-up).
