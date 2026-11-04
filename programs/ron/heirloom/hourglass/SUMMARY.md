# Hourglass — a rainbow lens pair pinching through an hourglass of yellow glow

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the video clips (phase 3), made from the still
`photos/vid/WIN_20230618_20_23_10_t0p5.png` (clip WIN_20230618_20_23_10, frame 9 at 0.528 s; the 11 s clip is one
patch scrolling with a 2.37 s period, and clip 20_24_23 repeats it). Two lenses on olive-black: a top lens cut by
the frame top (black core, blue ring, two tilted magenta/yellow stripe pairs, a cyan/green tip) and a bottom lens
(rainbow rings round a yellow-green core), pinched apart at the waist and wrapped in a soft yellow halo. Presets
are hand-written placeholders.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_hourglass/` (`notes.md`; `wprof.py` / `kymo.py` = per-row key edges over all
317 clip frames; `fitW5.py` = the wave fit; `tabbuild.py` = the colour tables; `proto.py` / `proto2.py` = the float
proto; `imodel.py` = the integer model).
- **Background** olive-black (17, 15, 4), 47 % of the frame (hw Y 113, U 519, V 491).
- **The key.** |x − c| < W(y), axis c = 993 in the capture. Top lens W = 600..613 at y 0..30, tip (W = 0) at
  y 327; bottom-lens tip at 527, widest 510 at y 760..810, a neck of 208 at the bottom edge. The gap 327..527 has
  W < 0 (about −70 at y 440, read from the halo). The tips are pointed with concave flanks.
- **W(y, t)** = S(y) + A(y) sin(2π(y/750 − t/2.37) + π/2), in virtual lines and seconds, with t = 0 at the still.
  - S = 106.9 + 366.7|u| + 80.9u² − 144.9|u|³ (the hourglass: a rounded vertical triangle), u = (y − 530)/540.
  - A = 219 + 44(1 − |u|).
  - Fitted on all 317 frames' edge profiles, still-weighted: rms 27.6 px. The rest is a slow 23-px wobble
    (λ 800, T 3.84 s) plus edge-detection noise; the wobble is not modelled.
  - Per-row check: the phase slope is −0.48°/line (λ = 750) over the whole height.
- **Halo** (outside the key, e = |x − c| − W): G = 17 + 185(1 − e/260)^1.6, R = 0.87 G, B 5..13. It fills the gap as
  a soft column.
- **Blue ring.** B is a bump of the depth D9 = W − 0.9|X|: 0 below 0, peak 247 at D9 220..250, 0 above ~400.
  It explains 98 % of B's variance over the clip.
- **R and G.**
  - Thin stripe pairs (magenta + yellow) tilt down to the right at 0.065 line/px, at the lens shoulders.
  - A lens whose peak W passes 490..575 fills with broad rainbow rings (magenta / violet / dark / yellow-green
    core): the bottom lens in the still. Green/cyan fills the tips and necks.
  - The 1-D fields tried fit poorly (≤ 0.62 of the variance). As tables:
    - R = T1[E = W + 0.2X, page], 0.83 of the variance;
    - G = T1[E, page] + T2[D9, page], 0.85;
    - page = |y − 520|/35.
- **Capture artefacts** (not modelled):
  - a horizontal seam at y 775 inside the bottom lens, and short smears at the bulb shoulders, where the camera
    caught two moments of the fast pattern in one frame;
  - 3–6 px softness and a thin dark line at the key edge. The edge line *is* drawn, as a 3-px half-intensity rim.
- Float proto: 15.3 mean |RGB| against the still. RTL default frame: 17.8 (16.0 with the axis offset removed).

## Motion in the clip
- Everything scrolls **down**: the wave moves 750 lines per 2.37 s, which is **317 virtual lines/s**. The hourglass
  profile stays put. So lenses swell at the top, pinch off at the waist (the gap opens and closes), and re-form
  below.
- The cycle runs hourglass-with-blue-bulb → lens pair → rainbow top lens → hourglass.
- The whole picture repeats every 2.36 s, colours included: frame-difference minima at 2.36, 4.72 and 7.07 s.
  - The rainbow appears in a lens while its peak W passes about 500: the bottom lens at 0.4–0.8 s, the top lens at
    1.2–1.6 s.
  - The R/G/B tables reproduce this colour cycling as the lens moves through the pages.
- No knob moves in the 11 s.
- The program advances the wave phase per field by the measured rate. K4 = 700 gives 317 lines/s down at 59.94 Hz.
- The proto renders at t − 0.5 / + 0.5 / + 1.0 / + 1.5 s match the m-frames (`cmp_motion.png`).

## What it does
1. **Per line**, the engine computes the NEXT line's values:
   - |u| from the line;
   - the profile P from a ROM, then S = 107 + kW·P;
   - A = (219 + 44(1 − |u|))·gA;
   - the wave phase θ0 + y·inc into a quarter-sine ROM, then W = S + A sin θ;
   - the colour page |y − 520|/35.
2. **Per pixel**:
   - X = x − 960, with an optional Twin fold. Add the camera term to W.
   - Form E = W + 0.203 X, D9 = W − 0.906 |X| and D = W − |X|.
   - Inside (D > 0): R = T1r, G = T1g + T2g − 80, B = T2b. Both tables are indexed by field bin × page, with a 2×2
     ordered dither between neighbouring bins and pages. Pixels with D ≤ 3 are halved (the edge line).
   - Outside: the halo, GLOW[e × scale] coloured (0.87, 1, 1/32), optionally cut into rings.
   - Ice palette swaps R and B.
3. **Output**: RGB → hardware YUV by chalice's matrix (luma pre-scaled 16/13, capped 940), then the shell's two
   [1 2 2 2 2 2 2 2 1]/16 boxes and 13/16 luma. Drawn whites come out at about 777.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Waist | The hourglass profile's depth (kW ×0 .. ×2, photo 512). 0 = a straight chain of equal lenses down the frame; 1023 = a steep vase with huge top/bottom lenses |
| K2 Bulbs | The wave's wavelength (×2.8 .. ×0.35, photo 512). 0 = one long vase; 1023 = about 5 stacked lenses |
| K3 Glow | The halo's width (×0.25 .. ×4, photo 512). 0 = a thin rim; 1023 = yellow haze over the whole background |
| K4 Speed | Bipolar scroll rate (±8 deadband at 512). 700 = the clip (317 lines/s down); 1023 ≈ 2.8× down; 0 ≈ 2.8× up |
| K5 Colour | Slides both colourisers through the lens (±307 px of field, photo 512). Stripes, rainbow rings and the blue ring move. 0 = green lenses with a cyan core; 1023 = black lenses with green rims |
| K6 Camera | How far the picture (or the Osc) pushes the lens edges and colours. 0 .. 31/16 px per luma step; default 384 = 179 px per full luma |
| S7 Mirror | The wave mirrored about the waist: two equal lenses meet in the middle, scrolling outward |
| S8 Palette | Photo / Ice: red and blue swapped, giving a cyan halo, red ring and cyan stripes |
| S9 Twin | Two half-width hourglasses side by side |
| S10 Halo | Glow / Rings: the halo cut into concentric contour rings, which follow K3 |
| S11 Source | Video / Osc |
| P12 Pinch | The wave's amplitude gA, 0.12 .. 2.0. 0 % = a calm glowing vase (just the hourglass, a gentle ripple); 479 = the photo; 100 % = deep lenses with wide black gaps, a big black-cored lens |

## Video
- **Video mode (the default).** The camera's luma, as (Y >> 2) − 16 from a one-line buffer, times K6, is added to
  W before the key and both colourisers.
  - The lens edges bulge round bright parts of the picture, and the rings and stripes bend through it.
  - A black input gives exactly the still's patch: the RTL black-input render equals the integer model.
- **Osc mode.** The shell's drifting 640 × 540 virtual-px triangle lattice feeds the same path, made zero-mean
  (−120), so the lenses wobble ±90 px at the default K6. At the default the bottom lens grows a dark eye and diamond
  flanks: an unmistakable flip.

## Engine
- **Vblank-zone sequencer.** A shared 7-channel glide (1/8 of the gap per field), then exp2-ROM knob curves for K2
  and K3. Three products on one serial shift-add multiplier: inc = 22370 · 2^(0.75(k2 − 512)/256), the glow scale
  252 · 2^((512 − k3)/256), and the speed (k4 − 512 ∓ 8) · 657. Then the field's wave phase, and the per-dither-phase
  table offsets (zc 30–101).
- **Line engine.** Kicked at each g_start (and at zc 1300 for line 0), it finishes the next line in 79 clocks, so it
  fits SD's 858-clock line. It uses shift-add constants and four serial products (kW·P, Ab·gA, y·inc, A·sin).
- **Pixel path, 19 stages.**
  - 2 X; 3 |X|; 4 Twin fold, camera luma; 5 U, camera × gain.
  - 6–8 the field sums; 9 dithered offsets, the inside / edge flags, the halo distance.
  - 10 clamped indices, glow scale partials; 11 ROM addresses.
  - 12–13 EBR + copy; 14 G sum, halo rings; 15 clamp, halo red.
  - 16 mux, edge, palette; 17–19 matrix.
  - Then boxes 20–23, luma 24, gate 25. **C_LAT 24.**
- **Block RAM: 21 EBR.**
  - line buffer 2048 × 8 (4);
  - T1 and T2, 1024 × 16 (8);
  - quarter-sine + profile 512 × 16 (2);
  - glow 512 × 8 (1);
  - exp2 256 × 16 (1);
  - output-box rings 4 × 256 × 16 (4);
  - sync ring 256 × 4 (1).

  The rings carry `no_rw_check`.
- **Contracts.**
  - generated avid;
  - one latency (24) in every mode;
  - virtual 1080-line coordinates;
  - the Y/U/V boxes and 13/16 luma before the gate;
  - the running sums snap to neutral off the avid (resync);
  - colours in the hardware convention (U = Cr);
  - the engine starts per line and per field (no free-running microcode).
- **Verified** with GHDL at 720p60, decimation 1, default knobs:
  - Black-input Video: the RTL equals `imodel.py` (0.40 mean |RGB| once the 12-px output-filter shift is
    aligned).
  - Osc: matches the model's Osc lattice.
  - Two warm-up frames later the RTL equals the model 2 fields on (0.41 mean |RGB|), and the lenses have moved
    down. The motion is right.
  - Video with selfie.jpg: the camera bends the lenses and halo.
- **Bug found by the GHDL render and fixed before the build:** the field multiplier's 20-bit multiplicand
  overflowed while shifting (garbage wave rate and glow scale).

## Status
- **v0.1.0 built 2026-11-04 through the shared queue, router2 (builder default).** NOT flashed.
- Post-route Fmax (seed):
  - HD Analog 90.60 (1); HD HDMI **83.54 (seed 2: seed 1 stalled the router at the 900 s timeout)**;
    HD Dual 92.36 (1);
  - SD Analog 90.83 (1); SD HDMI 77.23 (1); SD Dual 85.89 (1) MHz.
- 5,648–5,661 / 7,680 LC (74 %), 21 / 32 EBR. iceunpack OK ×6; the packed config carries the new labels and
  presets.
- Output: `out/rev_b/ron/heirloom/hourglass.vmprog`.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh hourglass` (or `./build_heirloom.sh hourglass`).
  HD HDMI needs seed 2. If router2 stalls again, try `NEXTPNR_ROUTER="--router router1"`.

## Deviations from the photo
- **Colour tables.** The interior colours are binned means over the clip, so the still's crisp rings and stripes
  come out softer and slightly pastel.
  - The top-lens stripe pairs read orange/lavender rather than saturated magenta/yellow.
  - The bottom lens's ring order (magenta, violet, dark ring, yellow-green core) is there but blurred.
- **Axis.** It is centred (960); the capture has it at 993.
- **The 23-px slow wobble** (λ 800, T 3.84 s) is not modelled, so the period is a pure 2.37 s.
- **50 Hz modes** scroll 17 % slower: the phase advances per field.
- **Brightness.** The halo peaks at ~650 output luma (the still's brightest glow reads ~810), because of the shell's
  luma cap and 13/16 scale.
- **Capture artefacts.** The still's seam and smears are not drawn. Its whitish haze above the bottom lens tip is
  not drawn either.
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (±720 virtual px). The lenses (±613) fit, but the outer halo is cut at the sides.
  - Dither and edge widths are in output pixels, so SD is slightly coarser.
