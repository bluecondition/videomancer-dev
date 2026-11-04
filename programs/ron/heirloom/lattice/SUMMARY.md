# Lattice — blue diamonds on a lime trellis, crossed by sliding magenta blades

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom phase 3 (video clips), from the still
`photos/vid/WIN_20230401_00_20_03_t0p5.png` (clip WIN_20230401_00_20_03 at 0.5 s; same patch: 00_19_01, 00_25_43).
A lattice of blue diamonds on lime green, big and small ones alternating, each ringed yellow / orange / dark; a faint
relief trellis running diagonally between them; and magenta blades lying on the trellis — X crosses at the saddles
whose arms taper to points, coloured from an orange / dark entry edge through magenta and violet to a white-pink exit.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_lattice/` (`notes.md`, `pfit.py`, `palbin.py`, `blade3.py`, `blade4.py`,
`trellis2.py`, `proto.py` / `proto_c.py`, `imodel.py`).
- **Field.** s = X(x) + Y(y) + Z(y), fitted by IoU of the dark (luma < 60) mask: 0.950.
  - X: a shaped triangle, X = 1 − 2(u − 0.146 u(1 − u)), u = 0 at the big-diamond column (x 1008.1) and 1 at the
    small one, period 944.3 px. It is held at its x = 66 value left of x = 66, so contours run flat into the left edge.
  - Y: the V triangle (peak y 257.5, period 524.8 lines) mixed with 0.4645 of its own ramp (reset at y 410, softened
    over 38 lines), × 1.0095. The ramp is why the big diamonds hang lower (80 lines above / 117 below their widest
    row) and the small ones sit higher (99 above / 57 below). The vertex extents predicted the mix: a slope ratio of
    0.62 gives c = 0.47, and the predicted jump of 0.38 matches the measured 0.374.
  - Z = −7e-7 (y − 600)² below y 600: the bottom row's small diamonds are larger.
- **Diamonds.** Big: luma half-width 194.5 px. Small: 119 px. Blue (24, 16, 247).
  - Big-side palette by s: green to 1.18, yellow 1.24, orange (225, 135, 55) 1.28–1.36, dark (34, 24, 30) 1.40–1.46,
    blue from 1.60.
  - Small side: green to −1.50, orange −1.58 .. −1.68, dark −1.74, blue from −1.86.
- **Trellis.** A relief on the pure-triangle net q = (Tx + Ty)·sign(dX/dx), the diagonals through every saddle. The
  green ramps up to R +8.5 just before the line, drops to −7.5 just after it (a bright edge, a dark crease) and
  recovers to −3.5.
- **Blades.** The band |q + 0.041| < hq: in this coordinate the centre is constant and only the half-width varies.
  - hq = 0.27 (1 − ((φ − 0.265) / 0.29)²) over the phase φ of a second vertical oscillator. That gives 0.076 at the
    window start, 0.26 mid-window and 0.066 at its end, then off for the rest of the cycle.
  - At the still that oscillator runs at the rows' rate, with the window starting 462 lines into each cell.
  - Colour runs across the band from scan entry to exit, the same on both arm directions (normalised to the width):
    orange → dark → magenta (248, 0, 245) → violet (139, 40, 228) → lavender → white-pink (255, 105, 255).
  - Arms are ~126 px wide horizontally and taper to points; the lower arms reach y ≈ 740, the upper ones y ≈ 465.
- **Texture.** Soft horizontal edges (10–90 % ≈ 9 px); no noise or tearing.

## Motion in the clip
- t − 1.5 .. t 0: static (four identical strip frames).
- Then Ron turned the blades' oscillator rate up:
  - the blade windows crowd toward the top of the frame, their heights shrinking 268 → 244 → 216 → 176 lines at
    +0 / +0.5 / +1.0 / +1.5 s;
  - that is ×1.05, 1.15, 1.30 and 1.60 of the rows' rate, with the top window anchored (the oscillator restarts each
    field);
  - the X crosses lift off the saddles, break into spindles on the diagonals and re-form at the other saddles;
  - the top blade slid (+18, −10) then (+32, −18) px per 0.5 s, up and right along the trellis.
- The diamonds stay put. Their widths drift a few percent: big 339 → 351 px, small 236 → 215 px by +1.0 s.
- In the program, K5 Spindles is that rate (512 = locked = the still). The default motion is K4 Drift at 620: the
  oscillator's phase drifts ~41 lines/s, so the blades climb the trellis (X → spindles → X, one cell per ~13 s).
  Proto renders at ×1.30 and ×1.60 match m2 and m3 (`cmp_proto5.png`).

## What it does
1. **Field.**
   - The H phase is a DDA (Columns scales it about the screen centre, held for x < 66); X is read from a 512-entry
     shape ROM.
   - Y, Ty and the sag are per line (line engine).
   - s = X + Y. Each side gets its own size offset (Big / Small), a = |s| + offset.
2. **Fringe and rings.**
   - jj = ((tf − 256 − a)·gw >> 8) + 64: jj < 0 is the fill, 0..127 the photo's fringe.
   - Above 128 come up to four rainbow rings (Rings > 512); otherwise the ground.
   - gw = 64·2^((512 − Rings)/128) squeezes the fringe to a hard edge as Rings goes below 512.
3. **Ground.** Green shaded across the trellis by q (64-entry relief).
4. **Blades.** |q + 42| < hq(line). The colour index is (q + 42 + hq)·32/hq, through a per-line 8-bit reciprocal
   mantissa and shift.
5. **Priority.** fill > fringe > blade > rings > ground, then one palette ROM read
   (pal × 8 regions × 64 entries, hardware U = Cr).
6. **Output.** Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the still's softness and the HD HDMI bar fix,
   with the per-line resync), then luma 13/16. Palette luma is pre-scaled 16/13, so drawn whites come out ≈ 777.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Big | the big diamonds' size: 0 = gone (orange specks), 512 = the photo, 1023 = merging hexagons |
| K2 Small | the small diamonds' size, the same range |
| K3 Columns | the diamond columns across (the H oscillator's rate): ×1/3 (one long diamond) .. 2.03 columns (512) .. ×3 (six narrow columns), about the screen centre |
| K4 Drift | bipolar (±8 deadband, square law): the blades climb (right) or fall (left) along the trellis; full ≈ 2 cells/s; 620 = the default slow climb |
| K5 Spindles | the blades' oscillator rate against the rows, ×1/3 .. ×3. 512 = locked: every cell shows the same X, as in the still. Turning it, as Ron did in the clip, crowds the blade windows toward the top: X's break into spindles that sit at a different phase in each row. 0 = long continuous blades |
| K6 Rings | the diamonds' fringe: 0 = hard blue-on-green edges, 512 = the photo's yellow / orange / dark ring, 1023 = four rainbow contour rings round every diamond (the 00_19_01 look) |
| S7 Shape | Diamond (the photo) / Bulb: both waveforms rounded, so the diamonds become lenses and ellipses |
| S8 Cross | Single (the photo) / Double: a second blade window half a cycle away, giving twice the blades |
| S9 Fill | Flat (the photo's blue, lit by the source) / Gem: diamonds graded from the rim to a pale core |
| S10 Palette | Lime (the photo) / Dusk: amber ground, navy diamonds with magenta fringes, cyan blades |
| S11 Source | Video / Osc |
| P12 Blades | the blades' width and window: 0 % = the bare lattice; 380 (37 %) = the photo; 100 % = continuous wide magenta ribbons weaving the whole trellis (the 00_19_01 wide pink bands). Glided |

Glides (1/8 per field, ±2 deadband) on K1, K2, K3, K5, K6 and P12. Presets: Photo, Spindles, Woven, Rainbow Gems,
Dusk Bulbs.

## Video
- **Video (default).** Camera luma, (Y >> 2) − 16 from a line buffer, lights the diamonds' fill.
  - With no input the fill is the photo's blue.
  - With a camera, each diamond holds the picture as a blue-to-pale monochrome image. This is exactly what the same
    patch shows in the clip's other look (00_19_01: the camera keyed into the diamonds).
  - The lattice, fringes and blades stay as drawn.
  - In Gem mode the camera adds to the gem gradient.
- **Osc.** A drifting 2-D triangle lattice (≈ 25 % of full luma) feeds the same path, so the diamonds shimmer with
  slow lighter-blue ripples.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier, 8 slots: the H step, the start phase, the blade rate and its per-line step, the
    rows' per-line step, drift², the drift step and the blade gain.
  - The exp2 ROM serves Columns, Spindles and the Rings gain. The shared glide is lozenge's.
  - The drift accumulates the blade oscillator's phase per field.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line in ~250 clocks (fits SD's 858):
  - the V triangle, the softened ramp, Y (or the bulb), the sag and Ty;
  - two blade-window parabolas (the second for Cross), each from d², × 1/Wd² and × gain on a serial multiplier;
  - a restoring divider for 2^20 / hq, normalised to an 8-bit mantissa and a shift.
- **Pixel path (15 stages + 6 output).** u9 / X-ROM address / Tx → X ROM → q, s → |s|, e + hq → a, blade test and
  partials → dm, blade product → gw partials, blade shift → jj, gem → zone and palette address → palette ROM → fabric
  copy → compose → boxes → 13/16 → gate.
- **ROMs.**
  - X shapes 1024 × 12 (3 EBR), palette 1024 × 32 (8 EBR), exp2 256 × 16 and a 2048 × 8 line buffer.
  - 15 / 32 EBR in all.
- **Contracts.**
  - damier / lozenge generated avid; one latency (C_LAT 20) in every mode.
  - Virtual 1080-line coordinates; the blanking gate; hardware colour convention.
  - No self-add carries (checked with dupin.py; the one duplicated-input LUT pair was rewritten unsigned).
- **Verified (GHDL, 720p60, decimation 1, real timing ID).**
  - Default frame: bit-exact with `imodel.py` on 100 % of non-fill pixels (after 3 fields of drift).
  - A second frame with K1 300, K2 700, K3 700, K5 760, K6 900, P12 700 and every switch on: bit-exact on 100 % of
    pixels (glides modelled).
  - Model vs still at defaults: median |ΔRGB| 2.7, mean 15.5; 4×4 region means within 3–9 levels; luma-class
    agreement 88 %.

## Status
- **v0.1.0 built 2026-11-04 with router1:** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh lattice`.
- **All six configs on seed 1.** Post-route Fmax:

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 94.11 |
  | HD HDMI | 94.71 |
  | HD Dual | 96.16 |
  | SD Analog | 92.37 |
  | SD HDMI | 94.30 |
  | SD Dual | 95.30 |

- **Fit:** 6,552–6,566 / 7,680 LC (85 %), 15 / 32 EBR.
- **Checks:** iceunpack OK ×6; the packed config carries the labels and presets.
- **Output:** `out/rev_b/ron/heirloom/lattice.vmprog`. **Not flashed.**
- **Watch on hardware:** no pin-skew seed screen was run; if HD HDMI shows lines, re-seed that config.

## Deviations from the photo
- **Fringe.** The dark band is a symmetric ring (thicker only where the field is shallow). In the photo it is heavier
  under the big diamonds' lower-left and over the small diamonds' tops, with a slight pink tint at the small diamonds'
  right vertices. No horizontal chroma lag explained it (all channels are purest at the same offset).
- **Blades.**
  - A dark olive streak inside two blade arms is not drawn.
  - The left-edge hold turns the left blade X into a magenta-pink bar; the photo's bar is whiter.
  - The bottom-row blade is drawn as a small X; the photo shows only its V.
  - Blade arms are straight trellis diagonals; the photo's curve slightly.
- **Left edge.** It starts with 4 px of black and a 12 px fade-in: the shell's generated-avid start and filter
  warm-up, as in lozenge.
- **Whites.** Cap at ≈ 777 output luma (the HD HDMI bar fix).
- **Resolutions.**
  - 720p shows the same geometry as 1080p.
  - SD shows the centre 4:3 (virtual x 240–1680): the outer half-columns and the left-edge hold fall outside, and the
    picture is softer.
