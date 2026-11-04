# Placard: magenta arcs, CMYK diagonals, bars and roads, rolling in a frame

**v0.1.0 built 2026-11-04, NOT flashed.** Heirloom from the clip `WIN_20230221_00_58_22` at 0.5 s
(`photos/vid/WIN_20230221_00_58_22_t0p5.png`; same patch as `WIN_20230221_00_56_54`). A magenta ground with thin black
arcs is cut by hard-edged diagonal zones of white, cyan, flat magenta and yellow. Vertical white / cyan bars and
horizontal stripes are keyed inside the white and cyan zones. Two soft grey "roads" of fine coloured lines and dark
mosaic staircases complete it, the whole vertical structure rolling down the frame inside a black letterbox.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_placard/` (`notes.md`, `proto.py`, `imodel.py`, `diff_rtl.py`,
`cmp_proto.png`, `cmp_final.png`, `sheet.png`, `cmp_motion.png`).
- **Frame.** The picture sits in a black letterbox: x 218..1827, y 87..990 (1610 x 904, 16:9 at 84 %, 63 px right of
  the centre).
- **Ground and arcs.** A magenta ground (251,18,175) carries thin black arcs.
  - Their local tilt dx/dy is proportional to screen x: x/330 at y 600, x/660 at y 640, zero at y ~690 and ~210, and
    reversed over 700-860.
  - So they are contours of a multiplicative field F = (x - 25) u(y) / lambda: an H ramp through a VCA driven by a
    vertical oscillator. ln u = 0.22 cos(2 pi (y - 460) / 470) (ratio 1.55), lambda = 109 px.
  - The pitch is 83 px at y 480, 114 at y 600 and 138 at y 720.
  - Each period carries a thick line (13-20 px, luma ~45) and a thin one (8-12 px, luma ~95) half a period later.
- **Diagonal zones.** The edges are straight at dx/dy = 1.80: the yellow's right edge over y 740-845 fits
  x = 1.7996 y - 480 with rms 1.5 px.
  - Along d = x - 1.8 y the classes fold about d0 = 68: Z = |d - 68|.
  - In order: white < 165 < cyan < 375 < flat magenta (252,22,252, no arcs) < 575 < yellow (252,249,10) < ~1400 < black.
  - Both sides are symmetric within ~10 units.
- **Zone gate.** The zones show for y 150-400 and 640-850 and are absent for 440-640: a vertical square wave with
  60 % duty in each 470-line half-period. The whole structure repeats every 940 lines.
- **Bars.** A global grid: period 100.75 px, width 46, on-phase 25 + 100.75 k. There are three groups:
  - top-left, x < 620: white above / cyan below the zone boundary;
  - centre, x 900-1300: hatched with ~5-row W / C / M / K lines, in the gate-off window;
  - bottom-right, x > 1130.
- **Stripes.** x 560-1250, rows 180-400, pitch 30 (15 white or cyan, 15 ground).
- **Roads.** Their edges are hyperbolae (x + 200)(y - y0) = C:
  - middle road: y0 352, C 49000;
  - bottom road: y0 822, C 40400;
  - top road, a falling ramp: (x + 200)(274 - y) = 77600.

  Inside a road the rows cycle white -> cyan -> magenta -> yellow -> black every ~16 rows at x 1500, 32 at x 700 and
  46 at x 400. They are contours of the product field P = (x + 200)(y - y0) (an H ramp times a V ramp) through the
  zones' cyclic colouriser.
- **Mosaic skirts.** Dark navy (24,27,55): the ground with its red killed, arcs faintly visible.
  - Bottom-left, x 220-900, y 520-740: a staircase of 50-px columns (half the bar period) whose lower edge is the
    hyperbola (x + 200)(822 - y) ~ 88000, sampled and held per column.
  - Top-right, x 1330-1720, y 210-253: blocks of random depth.
- **Palette** (hardware 10-bit, U = Cr):
  - magenta Y426 U876 V650
  - flat magenta Y465 U852 V780
  - yellow Y829 U585 V89
  - white Y878 U503 V518
  - cyan Y692 U131 V649

## Motion in the clip
- **Measured.** The m-frames (0.5 s apart) cross-correlate at dx 0, dy +84..88 px per 0.5 s on every region.
  - The whole vertical structure (roads, zones, gate, bar groups, mosaic) rolls DOWN at ~172 px/s, one 940-line period
    per 5.5 s.
  - The bar grid stays fixed in x.
  - The yellow row segments run 132-272 / 612-794 (t-0.5), 220-356 / 672-882 (t0), 316-438 / 758-966 (t+0.5) and
    428-514 / 842-988 (t+1.0).
- **The other look of the same patch** (clip 00_56_54) has the vertical modulation turned far up: yellow S-tongues
  with wide grey roads and fine contour lines everywhere. That is K2 Squeeze's upper range.
- **In the program.** The default K4 rolls everything down at 3.06 virtual lines per field: 184 px/s at 1080p30,
  720p60, 1080i and SD, and 147-153 px/s at 1080p24/25. The GHDL render shows the structure 20 rows lower 10 frames
  later at 720p60 (180 px/s), with the bars fixed.

## What it does
1. **Vertical phase.** phi = (y - scroll + 118) mod 940 virtual lines; the scroll advances by K4's step per field.
2. **Arcs.** u = 2^(a cos(2 pi (phi - 108) / 470)) from a 1024 x 8 cos ROM and a 256 x 16 exp2 ROM (a from K2).
   - Per line, the arc rate is finc = u / lambda (lambda from K1, exp2), and it runs a Q20 phase DDA per pixel.
   - Thick line where frac(F) < 0.13, thin where |frac - 0.5| < 0.04.
3. **Zones.** Z = tri(x - 1.8 y') (S7 Ramp: sawtooth), with a period scaled by K5.
   - It is kept as an unfolded per-line ramp, stepped -0.9 h per line and wrapped, continued per pixel and folded.
   - Thresholds at 0.094 / 0.219 / 0.344 / 0.820 of the period give W / C / M / Y / K.
4. **Roads.** Pw1 = (x + 200) 1.2 (phi + 4) / 16 is a per-pixel DDA whose start and step are per-line accumulators
   (no pixel multipliers).
   - The narrow road, the wide road's upper hyperbola and the skirt are compares of Pw1 against constant-step ramps,
     with their thresholds folded in.
   - The road colour is a 16-entry cyclic palette on the product's bits (one cycle per 32768); the arcs cross the
     narrow road as dark lines.
5. **Gate.** Zones and bars show above the narrow road where phi >= g1, and below it where phi >= 470 + g2.
   g2 = 470 (1 - Flood) and g1 = clamp(2.5 (g2 - 282)), which gives the clip at Flood 40 %.
6. **Keys.**
   - A bar phase accumulator (K3) makes bars (duty 0.457) in the top-left and bottom-right regions.
   - The stripes (30 lines, from a phase ROM) fill the first region right of x 580 (S8: bars instead).
   - Hatched bars (W / C / M / K, 2-row cycle) fill the gate-off window, x 900-1300.
   - Bars and stripes are white in the W zone and cyan in the C zone.
7. **Mosaic.** The bar phase's half-period toggle clocks a per-column sample-and-hold of the skirt compare: dark
   ground in the yellow below the middle road, x < 1120.
8. **Compose.** The priority runs frame > wide road > narrow road > mosaic > flat zones (M / Y / K) > bars and stripes
   (W / C) > hatch > ground. The result indexes a 64 x 32 palette ROM (Print / Neon).
9. **Output conditioning.** Chroma goes through a [1 2 2 2 2 2 2 2 1]/16 box, luma through [1 2 1]/4, then luma 13/16
   and the blanking gate.

## Controls
| Control | Element of the clip it drives |
|---|---|
| K1 Arcs | the arcs' pitch: x2.8 (0) .. the clip (512) .. x0.35 (1023), exponential, glided |
| K2 Squeeze | the vertical oscillator's depth into the arcs, square law: 0 = straight vertical lines, 512 = the clip (rate ratio 1.55), 1023 = deep S-curves (ratio ~5.6, the 00_56_54 look) |
| K3 Bars | the bar grid's and mosaic columns' pitch x0.5 .. x2 (the clip at 512) |
| K4 Speed | the roll, bipolar (+-8 deadband, square law): 672 = the clip's downward roll (~184 px/s), full = ~930 px/s, left = upward |
| K5 Zones | the diagonal zones' scale x0.40 .. x2.48: narrow (more zone cycles, the black zone appears) .. the clip (512) .. wide |
| K6 Etch | how far the camera (or the Osc) bends the arcs (up to 3 cycles at full luma) and the zone edges (up to ~390 px), square law; 320 = 0.3 cycles / 37 px |
| S7 Fold | Fold (the clip's mirrored W C M Y sequence) / Ramp (one-way W C M Y K, black zone visible) |
| S8 Stripes | Stripes (the clip's horizontal stripes in the top-left region) / Bars (vertical bars there too) |
| S9 Frame | On (the clip's black letterbox) / Off (the patch fills the screen) |
| S10 Colours | Print (CMYK, the clip) / Neon (navy ground; white, green, red, blue zones) |
| S11 Source | Video / Osc |
| P12 Flood | the zone gate's duty, glided: 0 % = arcs and roads only, 40 % = the clip, 100 % = zones and bars everywhere between the roads |

## Video
- **Video mode (the default).**
  - The camera's luma, as (Y >> 2) - 16 from a one-line buffer, is added to the arc phase and (scaled 1/4096) to the
    zone position.
  - The arcs bend around the picture and the diagonal zone edges wobble with it.
  - At the default Etch the composition reads clearly; with no input, Video mode is the clip.
- **Osc mode.** A 640 x 540 virtual-px triangle lattice, drifting about 75 px/s, feeds the same path. K6 stays live
  in both modes.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier: 20 slots, slot number decoded two clocks ahead, operands pre-muxed, one shared
    exponent shifter.
  - A shared glide for K1, K2, K3, K5, K6 and P12.
  - Three exp2 ROM reads. The exponents are formed in unsigned form, so no adder sums a sign bit with itself (that
    was a router1 hang).
  - Per field it computes:
    - 1/lambda, the oscillator depth, the bar step, speed^2, the zone period and thresholds, the etch gain, the flood
      gates;
    - the bar phase at x0;
    - the product field's line-0 value and step, their per-line increments and wrap constants;
    - the four ramp starts and steps, the line-0 phase and the zone ramp.
  - One small operation per state.
- **Per-line engine.** It is kicked at each line start and computes the NEXT line in ~62 clocks (it fits SD's
  858-clock line):
  - the phase and its wrap, prepared in idle clocks;
  - cos ROM -> |cos| k2g -> exp2 ROM -> x 1/lambda, giving the arc rate and the arc phase at x0;
  - the stripe / hatch rows from a 1024 x 4 phase ROM;
  - the product field's row value and step, and the zone ramp, each on one shared adder.
- **Beam.** Seven DDAs (arc phase, Pw1, four offset ramps, bar phase), the wrapping zone ramp and the column tick.
- **Pixel path (15 stages + output).**
  1. Beam.
  2. Compares, keys and x-region flags; fold.
  3. Sample-and-hold; luma.
  4. Etch partials.
  5. Etch; ring A back.
  6. Etched phase and zone position.
  7. Clamp.
  8. Line tests, zone code; ring B back.
  9. Compose.
  10. - 12. Delay.
  13. Palette ROM address.
  14. ROM data.
  15. Copy and blank.
  16. - 17. Chroma box and luma [1 2 1].
  18. Luma 13/16.
  19. Gate.
- **Block RAM.**
  - Ring A (arc phase and zone position) and ring B (flags) carry the pixel-path delays; ring C carries the chroma
    box's 8-back tap. All are 256 x 16 pairs with `no_rw_check`.
  - The line buffer is 2048 x 8.
  - ROMs: cos 1024 x 8, exp2 2 x 256 x 16, phase 1024 x 4, palette 64 x 32.
  - 15 EBR in all.
- **Contracts.**
  - The generated avid.
  - One latency, C_LAT 18, in every mode.
  - The blanking gate.
  - The HD HDMI output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16), with the running-sum
    resync: the box snaps to the palette black after 8 blank samples.
  - Virtual 1080-line coordinates.
  - Colours in the hardware convention (U = Cr).
- **Verified.**
  - GHDL 720p60 Osc / Etch 0 render vs the integer model (`imodel.py` through the output filter, `diff_rtl.py`): the
    structure is identical and 85 % of pixels are within 2 levels. The remainder is the arcs (1-3 px) and the zone
    diagonals (1-2 px), from a fractional field-phase difference; it is not bit-exact.
  - The full-travel sweep of every per-field and per-line value fits its RTL width.
  - Two RTL frames 10 frames apart show the roll.

## Status
- **v0.1.0 built 2026-11-04, NOT flashed.**
- **Router1, all six configs on seed 1.** Post-route Fmax:

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 90.46 |
  | SD Analog | 90.42 |
  | HD HDMI | 93.65 |
  | SD HDMI | 96.39 |
  | HD Dual | 93.95 |
  | SD Dual | 90.00 |

- **Size.** 7150-7158 / 7680 LC (93 %), 15 / 32 EBR. iceunpack OK x6.
- **Output.** `out/rev_b/ron/heirloom/placard.vmprog` (473229 bytes).
- **Rebuild.** `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh placard`.
  Router2 also closes HD Analog on seed 1 (80.8 MHz) but spreads down to 69 MHz on seed 2, so stay on router1.
- **Timing history.**
  - The first router2 build missed HD at 63-70 MHz on all seeds. The critical path was a vblank-sequencer state
    chaining the scroll add, a four-term shift-add and the zone add; every multi-op state was then split.
  - Router1 seed 1 then stalled on SD Analog (two sign-bit self-add LUTs in the exponent adders), fixed by the
    unsigned exponents.
- **No pin-skew seed screen was run.** If HD HDMI shows lines, re-seed that config.
- **Presets** are hand-written placeholders. Capture Ron's looks with `./capture_preset.sh`.

## Deviations
- **Cut for budget.**
  - The top-right hashed mosaic blocks under the top road.
  - K3 does not scale the stripes: they are a fixed 30 lines from a phase ROM.
- **Roads.** The wide roads are the product-field shell with the narrow road's 16-row line cycle. The still's top
  road also has a soft grey core and a 45-degree hatching at its very top, which are not drawn.
- **Bars.** Bar edges are hard. The still's white / cyan boundaries inside the bars are slewed, and its CMY fringes on
  the bar tops appear here only where a road overlaps. The hatched tops of the bottom-right bars are drawn solid.
- **Mosaic.** The staircase is regular 50-px columns sampled from a hyperbola; the still's blocks are irregular.
- **Brightness.** Drawn whites and yellows come out at ~777 output luma, the shell's cap (HD HDMI bar fix).
- **Resolutions.** SD shows the centre 4:3 (virtual x 240..1680), so the letterbox side borders fall outside the
  screen and the right bar group is cut. 720p is the same as 1080p. The roll is per field: 184 px/s at 1080p30,
  720p60, 1080i and SD, slower at 1080p25/24.
- **Model vs RTL.** The integer model and the RTL agree in structure, palette and texture but are not bit-exact
  (arcs within 1-3 px, zone diagonals within 1-2 px).
