# Waterfall — a curtain of chirped lines falling through four bands

**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #45, rebuilt from the photo
`photos/WIN_20230610_00_51_13_Pro.jpg` (same patch session as seismic). A bank of fine vertical
lines (dark navy cores above, red below) fans out going down and sweeps hard left at the foot,
through four horizontal colour bands: rainbow cycles with "(" hue arcs at the top, light cyan,
lavender-to-blue, and orange/yellow/red. A torn chevron seam separates the lavender and orange
bands, and a block area of olive, lavender, pink, cyan, green and a blue blob sits on the left.
Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_waterfall/` (notes.md has the full log).
- **One oscillator, reset at every line start.** In bands B-D the whole left "block area" is the
  oscillator's FIRST, slow cycle (psi -1 -> 0) from the line start to line 0 (x ~550):
  - R is a linear H ramp there and falls just before line 0, the same R shape every cycle has.
  - The lower-left blue blob is the B pulse that follows every odd reset, here the reset at x = 0.
    It narrows toward the bottom as the first cycle shrinks.
- **Chirp.** After line 0 the frequency rises and saturates. On row 450 the line spacing fits
  36 + 71 exp(-(x - 614) / 224) px: 107 px between the first two lines and 36.5 px at the right
  edge. 31 lines are visible. Odd lines are deep and even lines shallow, so a colour cycle is
  two lines.
- **The waterfall.**
  - Upper half: the lines fan out to the right going down (the right lines move ~+110 px
    between y 260 and 780).
  - Below the seam: the right lines keep drifting right (~0.5 px/row) while the left lines sweep
    hard left. Line 0 runs 604 @y850 -> 527 @950 -> 387 @1000 -> 218 @1050 -> 167 @1075.
- **The phase field is measured, not posited.** Four field families were tested and none fits
  everywhere:
  - an affine phase;
  - an affine x-warp (0.02-0.09 line above the tear, fails below);
  - G(x) + h(y) x + c(y);
  - a logistic slewed-FM oscillator.
  psi is therefore a robust 2-D cubic B-spline over the detected lines: 0.030 line rms above the
  tear, 0.014 below. It is stored as 47 row-knots x 31 knots (64 virtual px apart). The first
  cycle is exact as psi = max(-1 + x / x0, psiS).
- **Colour = f(band, row, psi mod 2).** In psi-space the photo becomes period-2 vertical columns.
  - A, y 0..256 (left) / ..313 (right): rainbow cycles (magenta -> pink/tan -> green -> line)
    with "(" arcs (the hue contours move inside each cycle with the row).
  - B, light cyan: G and B clip at ~250, and R pulses early in each cycle. Its top edge is a
    STAIRCASE, one step per colour cycle: y = 256 + 1.85 * 2 floor(psi/2) (plateaus 270, 280,
    289, 297, 307, 313).
  - C, y 546..786, lavender -> blue -> purple at the bottom. The B/C edge is flat at y 545-546.
  - Seam, y 786..836: the stripes are not shifted (row cross-correlation ~0). The first cycle
    becomes a fast chevron stripe (~56 px period, ~5 px/row slope, apex y ~803), and every cycle
    gets "<" smears.
  - D, y 836..1075, orange: G is a clean sawtooth per line, R ~255, and a B pulse follows odd lines.
  - B is keyed off (B ~10) in the olive/green block regions and the band-C bulge.
- **Red VCA.** R min stays 0 while R max fades from x ~1000 to the right edge in bands B/C
  (216 @x940 -> 33 @x1874 on row 450): white-cyan -> cyan, lavender -> blue. The fade is weaker
  in A and absent in D.
- **Line cores are fixed-PIXEL features.** The same pixel width holds at 107 px and at 36 px
  spacing: navy cores in A-C, red G-low lines after the fall in D, the B pulse after odd lines.
- **Beading.** A line-coherent per-row jitter of ~0.5 px rms with a quasi-period of 10-12 rows.

## What it does
1. **psi.** Per line, the engine interpolates two psi row-knots (47 knots, 8-bit increments as
   absolute Q7 values) and blends them toward reference row 448 by Fall (and the Strings gain).
   - The products cb Pk + cc Pk1 + ca Pr per knot run on one serial Booth MAC.
   - The 31 knot values go to a line-RAM pair (even/odd knots); the root x0 comes from one division.
   - Per pixel: psiS = knot + (step x 6-bit fraction) / 64. First cycle: a DDA, psi1 = -1 + x / x0.
2. **Bins.** Each pixel falls into one of three regions:
   - first cycle: 32 bins;
   - first stripe cycle (psi 0..1): 16 bins (the olive bulge, the grey-cyan follower of line 0);
   - the 2-line cycle: 32 bins, plus the colour drift.
   Each pixel takes bin or bin+1 by a 4x4 ordered dither.
3. **Colour.** 32 band-aligned colour row-knots x 80 bins in YUV565 (Y pre-boosted 16/13).
   - Per line, rows k and k+1 of the 112 entries the line uses are copied into a line-LUT pair.
     There are three slots: the stripe band, the first-cycle band, and the staircase / late-seam
     band.
   - Per pixel, the rows are interpolated with a 3-bit weight plus one dither bit, then decoded:
     Y = 64 + 14 q, chroma = 32 + 30 q.
4. **Red fade, in YUV.** R8 = 27 qy / 8 + 6 (qu - 16) from row k. Rm = R8 (256 - g) / 16, where
   g is a per-line DDA, flat to x ~1000 then falling with a per-knot slope x K3. Apply
   Y -= 1.27 Rm, Cr -= 1.75 Rm, Cb += 0.59 Rm.
5. **Line cores.** Pixel distance d = frs x rec[dl >> 2] (in the first cycle d = x - x0).
   The dip table is indexed by band x parity x d (-12..+28 px) and adds YUV offsets.
6. **Seam.** Seam rows remap the colour row into 764..836. The first cycle reads chevron bins from
   a per-row shift table, and colour right of x_t uses band D.
7. Clamp, then Dusk, then output conditioning ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma
   13/16), then the blanking gate on the generated avid.

| Control | Photo element | Function |
|---|---|---|
| P12 Fall | the waterfall bend | psi = ref + f (psi - ref), ref = row 448: 0 = straight hanging strings, 50 % = the photo, 100 % = twice the fan and the sweep (glided) |
| K1 Strings | the line count | psi gain 0.5x .. 2x, square law (1x at 50 %) |
| K2 Bands | the four colour bands | vertical roll of the bands (0 = the photo) |
| K3 Fade | the red fade to the right | R VCA slope 0 .. 2x (1x at 50 %) |
| K4 Speed | motion | bipolar, still in the centre +-8: hue flows through the strings, the fall sways +-12 %, the seam chevrons roll; default slow positive |
| K5 Seam | the torn seam between lavender and orange | seam height 0 (clean edge) .. 2x (1x at 50 %) |
| K6 Melt | the strings | camera luma (or the Osc triangle) bends the strings: +-0.14 line at the default, +-0.5 at max |
| S7 Beads | the beaded line edges | per-line jitter of the line edges On / Off |
| S8 Wind | the bend | Off / On: the bend reversed (the strings blow the other way) |
| S9 Dusk | the palette | Off / On: Cr <-> Cb (cyan -> yellow, orange -> blue, lavender -> pink) |
| S10 Lines | the line cores | Sharp (pixel dips) / Soft |
| S11 Source | | Video / Osc (default Video) |

## Video
The camera enters the patch as an analog artist would: its luma, minus mid-grey, is added to the
oscillator phase, so the strings bend around the picture. Bright areas push the lines one way and
dark areas the other. At the default Melt the composition still reads clearly. Osc mode feeds a
slow 2-D triangle (half amplitude) through the same path, so Melt stays live.

## Engine
- **Per field** (sequencer with one serial multiplier and one serial divider):
  - vstep = 30720 / W, the Strings gain, Fall with the sway;
  - the seam height and its inverse, the band roll;
  - the Fall coefficients, the bead step.
- **Per line** (kicked at the source avid rise, computes the NEXT line, ~770 clocks, which fits
  SD's 858):
  - the row (clamped, then decoded into the psi row-knot, then the band roll and seam offset);
  - the seam remap; the slot bands and knots (two clocks per slot); the staircase threshold;
    the seam shift word;
  - the fade slope lerp and its K3 scaling, cb, cc;
  - the psi loop: 93 products on the serial radix-4 Booth MAC (load + 6 steps, each operand
    fetched while the previous product runs), with the colour copy loop in parallel
    (224 clocks);
  - x0, the first-cycle rate (two divisions), the fade step and start.
- **Pixel path** (24 stages):
  - 1-6: knot read, melt, interpolation (11x6);
  - 7: melt add, reciprocal ROM address, first-cycle / seam beams;
  - 8: region, bins, slot, parity;
  - 9: dithered bin -> LUT entry, row weight;
  - 10-11: LUT read, dip distance;
  - 12: unpack, red from row k, dip address;
  - 13: row lerp, R8, fade weight;
  - 14: decode, Rm;
  - 15: fade terms, dips;
  - 16: sums; 17: clamp + Dusk;
  - 18-23: conditioning; 24: luma 13/16 + gate.
- **Memories** (24 EBR):
  - C_PSI 1536x16 (6);
  - C_COLA 2048x16 (8), C_COLB 512x16 (2);
  - C_DIPRG 512x16 (2), C_DIPB 512x8 (1);
  - C_REC 256x8 (1);
  - the psi line-RAM pair (2) and the colour line-LUT pair (2), both with no_rw_check.
  - Tables are generated by `waterfall.py` into `waterfall_pkg.vhd`, in hardware convention
    (u = Cr, v = Cb).
- **Contracts:** damier's generated avid; one 24-clock latency in every mode
  (processing_delay_clks 24); virtual 1080-line coordinates from the timing ID.
- **Fit history:**
  - v1 was 11,418 LC (148 %) and failed to place.
  - v3 packed to 7,495 LC standalone.
  - v4 (YUV565 tables instead of RGB -> YUV, dithered bins, row lerp moved to the pixel path,
    engine copy loop, reciprocal ROM, 14x11 MAC) packed to 6,505.
  - v5 (serial Booth MAC, red from the YUV codes, 6-bit psi fraction) packed to 6,024.
  - v6 (4 fewer pipeline stages) packs to 5,765 standalone and 7,218-7,229 in the real build.

## Status
- **v0.5.0 built 2026-10-27 with ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh
  waterfall`). All six configs closed on seed 1:
  - HD Analog 75.98, HD HDMI 84.83, HD Dual 76.38 MHz;
  - SD Analog 83.56, SD HDMI 81.38, SD Dual 84.50 MHz.
- 7,218-7,229 LC (94 %), 24/32 EBR, iceunpack OK on all six.
- `out/rev_b/ron/heirloom/waterfall.vmprog` (501,421 B). NOT flashed.
- router2 does not close this netlist reliably: HD Analog gave 74.0 / 76.8 / 67.1 MHz on seeds 1-3
  (router1: 76.0 / 81.0 / 82.1). Rebuild with router1.
- Verified in GHDL (720p60, decimation 1, hardware hues):
  - Osc defaults: 4x4 region-mean error 9.2 against the photo (proto 7.2);
  - Video with selfie.jpg: 9.3, composition holds;
  - motion (K4 max): 27 % of pixels change between fields 3 and 9.

## Deviations from the photo, and why
- Band B's light cyan is darker and greyer: the HDMI-bar luma ceiling (drawn white ~777 after the
  13/16 scale) caps the photo's Y ~848.
- A fine 4x4 ordered-dither texture shows in long gradients (first cycle, band B). The phase-bin
  dither replaced a per-pixel colour lerp to fit. The conditioning softens it.
- Band A's torn top-right cycles are regularised: the "(" arcs are per-row averages over 10
  row-knots, and the small horizontal glitch seams (y 40-60, 180-190) are only partly there.
- The seam chevrons are a hard-edged band. The photo's fade of the chevrons into orange at
  x 350-480 would need a second table read per pixel.
- The lower-left blob and orange shapes are smoother than the photo (colour row-knot resolution).
  Line 0's sweep is within +-5 px.
- Line geometry is smooth: the photo's alternating 37/35 px spacing of deep and shallow lines is
  not reproduced (their alternating depth is).
- The red fade uses a red reconstructed from row k's Y and Cr, so it is approximate.
- Beading is a +-0.8 px sine jitter with a ~10-line period, not the photo's ~2 px sawtooth teeth.
- Resolution: the same picture at SD, 720p and 1080p (virtual coordinates). At SD the right-edge
  lines are ~13.5 real px apart and the dither texture is coarser; the per-line engine has ~10 %
  headroom in SD's 858-clock line.
