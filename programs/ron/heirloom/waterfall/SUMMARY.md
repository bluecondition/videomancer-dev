# Waterfall — a curtain of chirped lines falling through four bands

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/waterfall.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (see [Phase 2](#phase-2-v060);
`Waterfall Diag`, `out/rev_b/ron/_diag/waterfall_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`).
Production v0.5.1 (`programs/ron/heirloom/waterfall/`) is untouched.
**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #45, rebuilt from the photo
`photos/WIN_20230610_00_51_13_Pro.jpg` (same patch session as seismic). A bank of fine vertical
lines (dark navy cores above, red below) fans out going down and sweeps hard left at the foot,
through four horizontal colour bands: rainbow cycles with "(" hue arcs at the top, light cyan,
lavender-to-blue, and orange/yellow/red. A torn chevron seam separates the lavender and orange
bands, and a block area of olive, lavender, pink, cyan, green and a blue blob sits on the left.

## Phase 2 (v0.6.0)
No notes from Ron for waterfall (free hand, 2026-11-09 batch). In v0.5:
- K1 Strings, P12 Fall and K5 Seam only reached 2x;
- K3 Fade was a colour nudge on the right half of bands B / C;
- S7 Beads (+-0.8 px) and S10 Lines Soft barely showed;
- the curtain's edge (line 0, where the long first cycle ends and the strings begin) had no control at all.

The round:
- widens the oscillator and shape knobs;
- gives the oscillator's **phase offset** its own knob (K3 Curtain), so the curtain draws back across the
  screen or sweeps over the block area;
- adds a curtain **swing** to K4 past the photo's speed;
- re-patches the two subtle switches: S7 Beads gets a slow +-32 px zigzag of every row, and S10 turns the
  string geometry upside down.

New motion is zero at the default knob values. With v0.5's sway formula substituted, the default frame is
byte-identical to v0.5.1 (checks below). Phase-2 working files: `.heirloom_work/rb_waterfall/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| P12 Fall: 0 .. 1x (50 %) .. 2x | **P12 Fall** 0 .. 1x (50 %, exact) .. **3x** (p12 / 2 below the middle, p12 - 256 above) | 0 % = straight hanging strings. 100 % = 3x the fan and the foot sweep: the left strings whip out through the left edge at the foot, and the right strings fan wide (2x was v0.5's top) |
| K1 Strings: 0.5x .. 2x, square law | **0.25x .. 1x (50 %, exact) .. 4x**, exponential (an octave per quarter turn) | right-edge line spacing 144 px (about 8 broad strings) .. 36 px (the photo, 31 lines) .. 9 px hairlines (about 120 across; SD aliases above ~3x) |
| K2 Bands: vertical roll | unchanged | the four bands roll through the full height |
| K3 Fade: red-VCA slope 0 .. 2x (a colour nudge on the right half of bands B / C) | **K3 Curtain** (renamed): the oscillator's phase offset, 4 (k3 - 512) Q7 photo lines (-16 .. 0 at 50 % .. +16), applied to every knot before the Strings gain, so it moves the curtain the same screen distance at any K1. The red fade is now fixed at the photo's slope | left: the curtain draws back; line 0 moves from x ~560 to ~1430-1670 (the block area fills 75-87 % of the width). Right: the curtain sweeps over the block area (line 0 reaches the left edge at about 66 %), then the strings keep sliding left, about 11 more lines by 100 % |
| K4 Speed: hue flow, the fall sway (meant as +-12 %), seam chevrons roll | same, **plus a curtain swing** past the photo's speed: in 376..648 it is exactly 0 (the default 640 is inside); beyond, it grows by 2 Q7 per step to +-6 photo lines at the ends, on a triangle a quarter cycle from the sway, at the sway's rate. The sway itself is now a smooth +-3 %, see Engine | at the ends, line 0 swings about +-400 px and every string slides +-6 lines; the period runs from ~2 s just past the photo's speed to ~0.5 s at the ends |
| K5 Seam: height 0 .. 1x .. 2x | **0 .. 1x (50 %, exact) .. 4x** (two-slope) | the torn chevron band from a clean edge (0) through the photo's 72 rows to ~288 rows (y 656..944): chevrons across the block area and "<" smears through most of bands C and D |
| K6 Melt | unchanged | |
| S7 Beads: On / Off (+-0.8 px jitter, invisible) | **S7 Beads: Fine / Zigzag**: Fine = the photo's beading (default); Zigzag = a triangle shift of each row's strings, +-32 px, period ~84 lines, travelling down ~60 lines/s | every string and the curtain edge become a zigzag ("<" chevrons over the whole curtain) |
| S8 Wind | unchanged | |
| S9 Dusk | unchanged | |
| S10 Lines: Sharp / Soft (subtle) | **S10 Sweep: Bottom / Top** (renamed): the string geometry read upside down (the psi row is 1079 - y), the bands staying put. Line cores are now always Sharp | the hard left sweep and the widest fan move to the top (through the rainbow band), and the strings hang straight in the orange at the bottom |
| S11 Source | unchanged | |

**Presets** (preset 0 = the defaults): **Photo** (renamed from "Default"); **Hanging** (P12 0); **Whipped**
(P12 100 %, faster); **Open Curtain** (K3 12 %); **Fountain** (S10 Top, P12 78 %); **Dense Rain** (Strings 88 %,
Seam 88 %); **Zigzag Storm** (S7 Zigzag, K4 near max, Seam 4x, P12 74 %); **Dusk Sweep** (S9 Dusk, few strings,
curtain forward). Dropped: the old "Dusk" (its K3 was the Fade).

**Engine (new).**
- **Strings curve.** g8 = 64 2^(k1 / 256), linear inside each octave, set in the per-field sequencer (the v0.5
  k1^2 multiply is gone).
- **Block float.** The engine MAC's coefficient operand is 12-bit signed (was 11). In K1's top octave g8 stays
  256..511 and `s_ex` makes the knot store shift 7 instead of 8. With Fall up to 3x, gf <= 1708 always fits;
  ca = g (1 -+ f) is clipped at 2047 only with Wind, Fall above ~2.6x and Strings near 2x or 4x. (v0.5
  clipped ca at 1023 over a wider corner.)
- **Curtain.** A per-field offset `s_cof` (K3 + the swing, 13-bit) is added to the MAC's knot operand in the
  psi loop. ca + cb + cc = g at any Fall and Wind, so the stored knot gains g x offset. The root search,
  x0 and the first-cycle DDA all follow it. One 14-bit adder; adding it at the knot store instead had cost
  132 LUT.
- **Swing.** depth = 2 (|v| - 128) when |v| > 128; triangle = the sway phase + a quarter cycle; depth x
  |triangle| on the field multiplier (one more sequencer state).
- **Sway fix.** v0.5's sway, `shift_left(f_acc(17 downto 0), 5)`, kept 18 bits, so the intended +-12 % wrapped
  to a 0..7 sawtooth with small jumps (the CLAUDE.md shift_left trap). The diag uses a smooth +-3 %:
  (fb |tri|) >> 12, a plain shift. That is what v0.5 showed, minus the jumps.
- **Seam.** k5e = k5 + 2 k5(8:0) above the middle; h36 = 9 k5e / 128 (8 bits); inv = 131071 / max(k5e, 64).
- **Zigzag.** n_jit = a triangle from the bead phase (10 bits, +-512 Q4); bead step / 8.
- **Sweep.** The engine decodes the psi row-knot from 1079 - y; the colour row keeps y.
- **Fade fixed.** n_s = clamp(s) straight from the slope lerp (the K3 product state is gone; the same numbers
  at K3 = 512).
- **Fit:**
  - the chroma box filter's 8-back tap moved from two 9 x 10-bit FF chains into a 256 x 20 block-RAM
    ring (`no_rw_check`, read 6 behind the write + a fabric register);
  - the hsync / vsync / field 24-clock delays moved into a 256 x 3 ring (read 22 behind + a fabric register);
  - the line-start-only registers l_g0 / l_cnt0 / l_jit were dropped: the beams load straight from the
    engine's n_* registers, which the engine rewrites 45+ clocks after the avid rise;
  - the unused l_s register was dropped.
  No pipeline stage was added or removed: latency 24 in every mode, U/V parity unchanged.

**Build** (ROUTER1: `NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh waterfall`, 2026-11-09):
HD Analog s1 79.74, SD Analog s1 73.51, HD HDMI s1 80.41, SD HDMI s1 76.71, **HD Dual s2** 78.03 (s1 missed),
SD Dual s1 77.93 MHz. 7,273-7,299 / 7,680 LC (95 %; v0.5.1 7,210-7,228), 27 / 32 EBR (+3: chroma ring 2,
sync ring 1); no RAM transparency emulation; iceunpack OK x6; the packed config carries the new labels and
presets. No pin-skew screen. **Fit history.** The first design was +149 packed LC over v0.5.1 (about 96 %), so:
- the curtain moved from the knot store (132 LUT) onto the MAC operand;
- the 3-level block float became a K1-octave flag with a ca clip;
- the K3 curve became a single slope (an MSB flip) and K5's a single adder;
- the sway lost its adders.
Standalone pack after these: 5,888 vs v0.5.1's 5,835 LC (`p2/abl/`).

**Checks** (working files in `.heirloom_work/rb_waterfall/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every new path at RTL widths over all knob extremes, no overflow:
  - gf max 1708 and ca max 2219 before the clip;
  - the curtain operand stays within the 14-bit m_a (psi ROM -700..3831);
  - the defaults are exact (g8 256, fb 256, k5e 512 -> h36 36, inv 255).
- `spikes.py`: the C_PSI knots along x and along rows and the seam shift words. No single-entry spike. Small
  row-direction kinks (|d2| ~0.2 line) sit at rows 17 and 28, where the row-knot pitch changes (32 -> 16 -> 8
  rows), by construction.
- Default frame (GHDL 720p60, decimation 1, Osc, 2 warm-up + 1 capture, production v0.5.1 simulated the same
  way). With v0.5's sway line substituted (`p2/wfexact_prog`), the raw 10-bit output.txt is **byte-identical**
  (0 of 2,764,800 samples). This proves the rings, the dropped registers, the fade bypass, the new curves and
  the block float at their defaults. The real diag differs only by the sway: a smooth +-3 % instead of the
  wrapped 0..7.
- One extreme frame (`p2/r06_extreme.png`: Strings 88 %, Curtain 20 %, Seam 100 %, Fall 88 %, Zigzag,
  Sweep Top, Osc; rendered while the sway was briefly +-12 %): no wraps. The curtain is drawn back past
  half the width, the strings are dense zigzag hairlines, the curtain edge zigzags, and the seam band is 4x
  tall.
- Not verified:
  - Video mode (unchanged paths);
  - Wind with the new ranges;
  - the swing in motion;
  - Sweep Top on its own (only in the combined extreme frame).

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
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Post-route Fmax: HD Analog s1 75.83, SD Analog s1 81.37, HD HDMI s2 79.47, SD HDMI s1 78.67, HD Dual s1 78.81, SD Dual s1 82.01 MHz; 7210-7228 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions for Ron (2026-11-09 batch)
1. K3 Fade (the red VCA toward the right) became K3 Curtain, and the fade is fixed at the photo's slope. Do you
   want the fade back, e.g. as a Fade / Deep option on a switch, or in place of the curtain swing on K4?
2. The curtain swing on K4 past the photo's speed is quick at the ends (~0.5 s a cycle, +-6 lines). Slower and
   wider instead (e.g. its own slower LFO)?
3. S7 Zigzag is a +-32 px triangle with an ~84-line period. Smoother (a sine) or bigger? And do you miss the
   Beads Off position?
4. S10 Sweep Top flips only the strings, not the bands. Would a whole-picture flip (bands too), or a horizontal
   mirror (the old S8 Mirror idea, block area on the right) be the better look-flip?

## Hardware watch list
- Renamed controls:
  - K3 Fade -> **Curtain** (bipolar, the photo at 50 %);
  - S7 Beads On / Off -> **Fine / Zigzag**;
  - S10 Lines Sharp / Soft -> **Sweep Bottom / Top**;
  - preset "Default" -> "Photo".
- The default's sway is a smooth +-3 % (v0.5's wrapped to a jumpy 0..7 / 256). The default picture breathes
  the same small amount, without the jumps.
- No longer adjustable: the red fade (fixed at the photo's slope), Beads Off and soft line cores.
- Strings above ~75 %: hairline strings (9 px at 4x) alias on SD; at 1080p they shimmer while K4 moves them.
- Wind with Fall above ~2.6x and Strings near 2x or 4x clips the reference coefficient. The strings thin at
  row ~448 in that corner (v0.5 had the same over a wider corner).
- Two new block-RAM rings: the chroma filter tap (256 x 20) and the sync delays (256 x 3). If colours or syncs
  misbehave only on hardware, suspect these first.
- ROUTER1 (as v0.5). HD Dual built on seed 2 (seed 1 missed); the other five on seed 1. HD margins are
  5-8 % (HD Analog 79.74 was v0.5.1's thinnest at 75.83).

## Next step
Wait for Ron's preview notes. Rebuild the diag with
`NEXTPNR_ROUTER="--router router1" .heirloom_work/diag_build.sh waterfall`. On approval: `./heirloom_diag.sh merge
waterfall`, then repack the previewed diag bitstreams under the production TOML instead of re-routing:
- copy `build/programs/ron/waterfall_diag/rev_b/bitstreams/*.bin` into the production build dir;
- regenerate `program_config.bin` from the production TOML;
- run `vmprog_pack.py --no-sign --hardware rev_b`.
Then update the production SUMMARY / README / PHASE2 row and commit only waterfall's files.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023), and the camera effect at the top of the knob is 25 % stronger.

Bend / element: K6 Melt is how far the camera's luma (or the Osc triangle) bends the strings.

Diag copy: the same change (melt gain (k6 >> 4) + (k6 >> 6)), no render. The per-field melt gain is now (k6 >> 4) + (k6 >> 6), 0..78 (was k6 >> 4, 0..63), decoded in the field-start block. Every position is x1.25 (x1.24 at the top). The 9 x 8 product fits its 16-bit register (+-9984). The strings now bend up to +-0.62 line at max (was +-0.5).

Build: router1, seeds other than 1: HD Analog seed 2, HD Dual seed 2, worst HD Fmax 77.64 MHz, 7333 LC (max of 6). Fmax per config: HD Analog 77.64, SD Analog 79.71, HD HDMI 78.12, SD HDMI 79.72, HD Dual 81.93, SD Dual 79.21 MHz.
