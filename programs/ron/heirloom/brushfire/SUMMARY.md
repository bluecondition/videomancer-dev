# Brushfire — crowns of flame over S-curved bands, breathing down and back

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230820_01_33_09_t0p5.png` (clip WIN_20230820_01_33_09, 9.5 s, frame at 0.5 s; the same patch in
clips 01_32_42 and 01_36_37). Two crowns of flame across the top: red-cored lobes, nested white/yellow arches and
long orange/red streaks, ending in tongues. Red-rimmed chevrons stack down each tile's axis in a pink/purple/brown
zone. Under everything run S-curved bands (white, black hairline, olive, yellow-green, blue) that sweep to the lower
right, crossed by translucent pink diagonals. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_brushfire/` (`notes.md`; `pal16_*` = palfit k = 16; `blk.py` / `fitb.py` = the
band field from 12 black-hairline traces; `fl2.py` / `wz.py` = flame bottom and warm-zone profiles; `fview.py` =
field contours over the still; `proto*.py` = float protos; `imodel.py` = the integer model of the RTL).
- **Tiles.** Horizontal period 945 px (row autocorrelation 0.95 at lag 944-945 for rows 0-250), axes at x = 540 and
  1485 (the two axis columns carry identical colour runs). 2.03 tiles across.
- **Lobes.** Red cores at (261, 318), (807, 278), (1203, 315), (1755, 277): sd 74 × 23 px (≈ 300 × 110 px), tilted
  ±13° (tops lean toward the axis), so ≈ ±273 px either side of each axis. Around each core: orange, yellow,
  pinkish-white rings and thin red lines, bunching outward.
- **Arches.** Nested "^" contours with apexes on the axis (y ≈ 100, 130, 165, 190, 230, 260), arm slope ≈ 2.3 near
  the top, white/yellow alternating with a 40-60 px period, then orange and the red rim (axis y 274-305).
- **Flame bottom** (per |x − axis|): 300 at the axis, 362 at 50 px, 444 at 90, 487 at 130 (the big chevron arm),
  tongues between the arm and the lobe, the lobes to ≈ 470; beyond ≈ 300 px (right) / 390 px (left) the bands
  reach the top, and the contours bunch into thin multi-coloured stripes near the tile boundary.
- **Chevrons.** Red-rimmed "^" stripes down the axis below the rim, orange and cyan/grey inside: apexes ≈ 455, 518,
  609, 722, 774, 798, 854, 898, 918, 945, 974, 990, so the spacing accelerates from ≈ 150 px to ≈ 20 px. Arm slope
  1.8 at the top chevron, 1.0-1.4 below. They are opaque only inside the warm zone.
- **Bands.** One field under everything, F_B = u M(v) + N(v) (u = x/1920, v = y/1080), with M = 1 − 3.056v +
  6.188v² − 3.864v³ and N = 0.523v − 0.977v² + 0.559v³. Fit to 12 black-hairline traces with rms 7 px, one hairline
  per 0.0635 of F_B (the 12 fitted levels cluster at that spacing).
  - Cool cycle, left to right: white (wide) → black hairline (15 px, dark-green fringed) → thin white → olive →
    yellow-green → yellow → blue → purple/pink fringe → white. ≈ 240 px wide at the left.
- **Warm zone.** In each tile's central column below the flames (|x − axis| < ≈ 190 px at y 300, 230 at 600,
  120 at 1000), the same bands are drawn warm with the same phases (label purity by band phase):
  - white → pink (216, 171, 170)
  - blue → purple (157, 77, 180)
  - olive → brown (141, 57, 24)
  - yellow-green → orange-brown (189, 95, 7)
- **Pink diagonals.** Translucent (≈ 45 %) "/" and "\" stripes, the chevron family continued, over the lower half.
- **Palette** (palfit shares): band white (214, 217, 227) 14 %, orange (240, 132, 11) 10 %, pink 9 %, red (246, 45,
  5) 9 %, blue (96, 112, 242) 8 %, purple 6 %, orange-brown 6 %, flame yellow (242, 209, 6) vs band yellow (200, 212,
  0), black hairline (47, 53, 47) 3 %.
- **Texture.** Soft horizontal edges and jagged black hairlines. A capture glitch at the far left (x < 70,
  y 230-300) is not modelled.
- Integer model vs still at defaults: mean |RGB| 62. The Cb/Cr means are within 2-3 levels (Cb −38 vs −40, Cr 38
  vs 40). Luma is 11 brighter (153 vs 142) with less contrast (sd 39 vs 49).

## Motion in the clip
- The x structure never moves: the row space-time images stay mirror-symmetric about fixed axes. The vertical
  structure **breathes** on a **5.8 s** cycle.
  - Bottom-sixth cool-band coverage: 0.54 (t 0) → 0.23 (t 3.0) → 0.59 (t 6.0) → 0.26 (t 9.0).
  - Red-lobe centroid (480-scale y): 116 → 88 (t 1.0) → 153 (t 4.3) → 89 (t 6.8) → 162 (t 9.3).
  - At the far phase (t 3.5-4.5) the lobes are tall lens shapes spanning the frame: y_still ≈ (y − 185)/1.7.
- The **bands stream right**: the hairline near x 100 moves 40 px/s and those near x 1000 move 124-196 px/s;
  new bands enter from the left.
- The fine flame streaks flow (diagonal traces in the axis space-time image).
- **In the program:**
  - The breath LFO moves the flame field's vertical ramp, y' = y·S + O with S = 2^(−0.765 h) and
    O = −0.426 h px, h = (w − w0) + Blaze. It starts 0.03 of a cycle after the flame minimum, as the still does.
  - Model frames at +0.5 / +1.0 / +1.5 s swell down like the m-frames (`cmp_motion.png`).
  - Streak flow: 0.5 fine cycle/s, upward. Band drift: 1/6 cycle/s to the right (≈ 32 px/s at the left edge).
  - Band wobble phase: 0.1 cycle/s. All rates scale with K4.

## What it does
1. **Tiles.** A 24-bit tile-phase DDA folds to a_n (0 on the axis, 1023 at the tile boundary).
2. **Flame field.** F = V(y') − B(y')·H(a_n) − G·by(y')·bx(|a_n − c(y')|) + camera.
   - H is the arch ramp: it rises 2.25 px/px from the axis to the lobe column, then falls toward the tile boundary.
   - Above the rim (y' < 300): V = y', B = 1.
   - Below the rim: V = 300 + 350E and B = 1 + 0.65E, with E = e^((y' − 300)/350) − 1. This gives the
     accelerating chevrons with steady arm slopes.
   - The lobe is a separable gaussian bump (sd 150 lines × 42 px), centred at c = 268 px + 0.22·(y' − 300) (tilt).
3. **Key.** Flame where F < 300, a_n is inside the wall half-width, and the fine phase is above the floor ramp
   (y' 470, tongues over 110 lines).
4. **Colour.**
   - The fine phase is F·kS/128 + flow (period 110 px). Seven heat pages (heat = |x − axis| ramp + lobe bump, 2×2
     dithered) hold 64-step fire cycles: white/yellow at the axis, red/orange rings at the lobes, solid red cores.
     F > 252 uses the red rim ramp.
   - Chevrons (period 220 px) are opaque in the warm zone where F ≥ 300.
   - The bands come from a 24-bit DDA on u·M(y) + N(y), read through the cool or warm palette.
   - Pink diagonals ((y' − 1.6a)/210, y' > 560) blend half pink over the cool bands.
5. **Output.** The shell's two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Tiles | Flame tiles across (the horizontal oscillator), 0.72 .. 5.75; 512 = the photo's 2.03. Lobes, arches and chevrons scale with the tile |
| K2 Lobes | The lobe bump's strength G = 420·(k/512)² px. 0 = no lobes (pure nested arches), 512 = the photo, 1023 = huge red lobes |
| K3 Streaks | Density of the fine flame streaks and of the chevrons, ×0.33 .. ×3 (exponential); 512 = the photo's 110 px period |
| K4 Speed | Bipolar (±8 deadband at 512): breath rate, streak flow and band drift together. 708 = the clip (5.8 s breath); 1023 ≈ 1.7×; left = reverse |
| K5 Bands | Band density, ×0.25 .. ×4 (exponential); 512 = the photo |
| K6 Camera | How far the picture (or the Osc) bends the flames and the bands (square law). 320 = 27 px flame / 0.17 band cycle per full luma |
| S7 Bands | Sweep (the photo's S-curved bands) / Zigzag (horizontal bands with a V per tile, the clip 01_32_42 look) |
| S8 Palette | Photo / Ice: U and V swapped, so blue/cyan flames over orange/teal bands |
| S9 Flip | Hang (flames from the top, the photo) / Rise (the flame field flipped: flames rising from the bottom) |
| S10 Flames | Solid (the photo) / Lines: only the red streak lines and rims are drawn, with the bands showing through |
| S11 Source | Video / Osc |
| P12 Blaze | Flame height, the breath's centre: 0 = a thin crown at the top over bands and chevrons, 360 = the photo, 100 % = the frame on fire (tall lens flames to the bottom) |

## Video
- **Video mode (the default).** Camera luma ((Y >> 2) − 16, one line buffer) × the K6 gain is added to the flame
  field and, ×256, to the band phase.
  - The flames lick around bright objects and the bands bend around the picture.
  - At the default Camera the composition stays intact.
  - With no input, Video mode is the photo.
- **Osc mode.** The shell's drifting 640 × 540 virtual-px triangle lattice, zero-mean and ×4, feeds the same path:
  slow waves through the bands and flame contours. K6 scales it.

## Engine
- **Per-field sequencer (vblank zone, zc 30..236).**
  - One serial right-shift multiplier (17 × 16). Jobs are selected by a code registered a clock ahead.
  - A shared glide for K1..K6 and P12.
  - An exp2/quarter-sine ROM for the knob curves and the breath cosine.
  - It computes: the tile step and line-start phase, G, kS as mantissa + exponent, the band constants, the
    camera gain, the speed rates, h (breath + Blaze), S and O.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ≈ 260 clocks, so it fits SD's
  858-clock line. One serial 22 × 16 multiplier, with job codes registered ahead, computes:
  - the wobble phase and y';
  - the piecewise-linear M and N (5 segments through the fitted cubics) plus the wobble sine;
  - Mk and Nk, the band line start and step (Sweep or Zigzag);
  - the exp position, then V and B;
  - G·by, the lobe centre c, the floor threshold, and the pink diagonal's line term.
- **Pixel path (18 stages to the compose register).**
  1. Fold the tile phase.
  2. H / bx ROM addresses; wall and warm-zone tests; 0.906·a_n.
  3. Camera; pink test.
  4. Camera product; zigzag shift.
  5. B·H in two 5-bit halves; gb·bx; band + zigzag.
  6. Sums; band + camera.
  7. V − B·H.
  8. F.
  9. F × 5-bit kS mantissa; heat.
  10. The streak phase (exponent mux) + flow.
  11. Decisions and the heat page.
  12. Palette addresses.
  13. EBR read, then a fabric register.
  14. Compose; pink mix; Ice; 10-bit; black off the generated avid.
- **Output conditioning** from the shell, with the per-line resync of the running sums.
- **ROMs.**
  - H 512 × 16 (arch ramp + heat_a), BX 512 × 8, FIRE 512 × 24, BAND 256 × 24.
  - SEQ 512 × 16 (exp2 + quarter sine), LN 1024 × 16 (exp2, sine, zone/wall table, lobe gaussian).
  - A 2048 × 8 line buffer, four box rings and a sync ring.
- **Contracts.**
  - Generated avid; one latency (24) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 1440 px).
  - Hardware colour convention (U = Cr; palettes via `brushfire.py`); luma pre-scaled 16/13 and capped 940.
- **Verified.**
  - GHDL 720p60 at decimation 1 (black input, Osc, selfie camera) matches `imodel.py` in structure, palette and
    texture, within a few px of the model.
  - A later RTL frame moves as the model does: chevrons and bands shift in the same directions by about a pixel per
    field at Speed 1023.

## Status
- **v0.1.0 built 2026-11-09 with ROUTER1, NOT flashed.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh brushfire`
    (or `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh brushfire`).
  - Output: `out/rev_b/ron/heirloom/brushfire.vmprog` (492,408 bytes, unsigned). iceunpack OK × 6.
- **Fmax (post-route, the builder's last line), seeds.**

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 86.81 | 7,098 |
  | HD HDMI | 1 | 95.66 | 7,109 |
  | HD Dual | **2** (seed 1 stalled router1) | 87.72 | 7,113 |
  | SD Analog | 1 | 93.59 | 7,099 |
  | SD HDMI | 1 | 94.83 | 7,092 |
  | SD Dual | 1 | 88.98 | 7,087 |

  7,087-7,113 / 7,680 LC (92.3-92.6 %), 23 / 32 EBR.
- **Fit and timing history.**
  - v1 put every constant on shift-adds and had two lobe products, two camera products and an a_n × k zigzag
    multiply: 8,540 LC, so placement failed. `netstat.py` (FF/LUT attribution by net name from the synth json)
    found the hogs.
  - The restructure moved the constants onto two narrow right-shift serial multipliers and cut the pixel path to one
    lobe product, one camera product and a shifted zigzag: 7,070 LC.
  - router2 then missed HD Analog on seeds 1-5. A profile gave 60.8 MHz: the line-counter decode into the
    multiplier's operand mux, 8.2 ns of routing on one net. router1 gave 75.3 MHz, critical on the breath sum.
  - Final: operand loads go through job codes registered a clock ahead (`z_fms` / `z_fld`, `e_ms` / `e_ld`) and the
    breath sum is split over two zone clocks. HD now has 17-29 % margin.
  - `dupin.py`: 0 `CARRY I0==I1`, 2 LUTs with a duplicated sign-extension input (`z_d1`, `z_sh`). These probably
    caused the HD Dual seed-1 router1 stall. If a rebuild stalls, register those operands separately.
- Presets are hand-written placeholders: capture Ron's looks with `./capture_preset.sh`.
- **Watch on hardware:** no pin-skew screen was run. If HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- The flame interior is drawn as many even streaks; the photo has broader glows (Y sd 39 vs 49). Luma is 11 levels
  brighter overall.
- The lobes are symmetric about each axis. In the photo the left lobe sits 40 px lower and 12 px further out, so the
  flame's open edge is at 300 px on one side and 390 px on the other.
- The chevrons are one periodic family: their apexes accelerate smoothly rather than at the photo's irregular
  heights. The bottom bundle is slightly flatter than the photo's.
- The warm zone is a per-line half-width (a column that narrows at the bottom), not a contour. The pink diagonals are
  a separate regular "^ / v" layer (every 210 px), more regular than the photo's.
- The bands use the fitted cubics as 5 linear segments plus one 2.5-cycle wobble. The hairlines' jaggedness (line
  jitter) is not drawn.
- **Motion.** In the clip the swelling flames narrow into lens shapes. Here the width stays at the wall, so at the
  breath's far phase the flames are wider than the clip's. The breath is a sine at a fixed 5.8 s; the clip's knob
  wanders.
- **Resolutions.** 720p matches 1080p. SD shows the centre 4:3, so 1.5 tiles (each flame crown and its chevron
  stack are whole).
