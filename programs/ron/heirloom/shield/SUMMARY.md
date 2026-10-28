# Shield — twin striped shields, plaid in nested U bands, blue pillars

**v0.5.0 (2026-10-27): built, NOT flashed.** Heirloom #38, rebuilt from the photo
`photos/WIN_20230529_22_56_41_Pro.jpg`. The photo shows two shields side by side. Each is a nest of
U-shaped bands, with vertical walls at the top and a V tip at the bottom centre. Horizontal stripes cut
the bands into an orange and red plaid. Each shield is ringed by a dark-red ramp, then a navy-to-blue
ramp, with a thin white line on its left arm. Below sit a red M body and bright blue triangles. Blue
pillars stand by the seams, with a column of stripes in each seam. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_shield/` (`notes.md`, `gfit.json`).
- **Tiles.** The shield period is 941.7 px (autocorrelation 943). The axes of the stripe bands are at
  x = 536.5 and 1477.5 (flip-correlation). Inside a tile, u = |x − axis| (an H triangle).
- **Field F2 = u + 5.952e-4·y² + 0.00267·y + 4.19.** This is the H triangle plus a V parabola with its
  vertex at the top of the frame.
  - Fitted as a non-parametric per-row shift, then a quadratic: rms 1.17 px over all rows.
  - The contours are horizontal translates of each other: vertical walls at the top, V tips lower down.
  - The V tips are rounded, u' = sqrt(u² + 6.6²).
- **Field F1 = 1741.4 − 2.892·u − 0.2496·(y − 700) − 1.406e-4·(y − 700)².** This is a steep inverted
  triangle plus a V ramp. Both arms fit the same.
- **F = min(F2, F1)** (a non-additive mix). F1 wins only near the seams in the lower half. That gives:
  - the vertical blue pillars;
  - the M body's vertical sides;
  - the triangles' vertical outer edges (u ≈ 291);
  - the seam column of stripes, with its upward-pointing tips.
- **Stripes.** A vertical square wave: half period 73.545 lines (7.34 cycles per 1080 lines), first
  boundary at line 56.4. The 14 boundaries fit to rms ~1 line. Orange-type rows sit above line 56.4.
- **Colouriser** (F in px):
  - **Bands**, floor(F/93.3), boundaries at 93 / 188 / 283 / 372:
    - orange rows: Ye (255,151,1), Or (255,105,1), Ye, Or (255,103,1), R1 (201,12,7);
    - red rows: OR (254,55,0), Rd (230,55,1), R1 (200,12,7), R2 (160,13,10), OR (253,54,0).
  - **The stripe zone ends at 427.7.** Then:
    - a dark-red ramp, R from 157 at 440 falling 1.5 per unit to ~22 at 530;
    - a navy-to-blue ramp, (17,12,45) to (9,29,229) over 535–630.
  - **The M body** (159,14,12) from 641.7.
  - **Bright blue** (13,9,250) from 830.5.
- **Two frames.** The band edges, the M edge and the triangle edges are symmetric about the 536.5 axes.
  The two ramps are symmetric about 526.2. So the comparator (keyer) path lags the linear ramp path by
  10.5 px. This is why the outermost band is 66 px wide on the left arm and 46 px on the right.
- **White line.** About 4 px of (235,225,230), drawn where F (keyer frame) falls through 530 in scan
  order, after having exceeded 552. On the left arm this runs from (100, 430) to the V tip at (540, 930).
  On the right arm it is a short, fading vertical line at x ≈ 963, y 440–600.
- **Dark line.** A dip of about 8–10 R levels where the scan rises out of the stripe zone (right arms).
- **Left edge.** A line-start hold: the field is held at its x = 66 value.
- **Texture.**
  - Horizontal blur, 10–90 % of 14–15 px on Y, Cr and Cb alike.
  - About 1 px of per-line edge wobble (lag-1 correlation 0.8).
  - No tearing, no noise. Horizontal edges are sharp.

## What it does
1. **Beam.** A Q24 tile-phase DDA, held through the blanking and the first 54 px (HD; 32 at 720p).
   - Triangle: u = tri × 0.9199 in quarter px, plus a 64-entry tip-rounding table.
   - The line's start phase carries a ±2 px LFSR walk (the wobble).
2. **F2 = u + g(line).** **F1 = C1(line) − 2.89·u** (as 3u − u/8 + u/64).
3. **F = min(F2, F1)**, plus luma × Etch. The result wraps every 1024 px.
4. **Keyer.**
   - Bands: (F × Bands gain) + flow, bits 10..8, into an 8-entry table per stripe type.
   - The M and triangle levels, and the white-line levels.
   - All of these are delayed 10 / 7 / 5 px (HD / 720p / SD) behind the ramp path.
5. **One-shots.**
   - White line: armed above 552, fires below 530, 4 px wide. On the F1 side it fades in quarters by the
     line's crease peak.
   - Dark line: 3 px at ×0.69, where the ramp starts.
6. **Palette ROM** (2 tinctures × 256 entries): the ramp in 2 px steps, the 16 band colours, the M body,
   bright blue, the white line.
7. **Output.** Y, U and V go through a 17-tap [1 2 … 2 1]/32 box and a 9-tap box. Together they give
   σ ≈ 5.2 px, which matches the photo's softness and doubles as the HD HDMI bar fix. Luma is then scaled
   13/16, then the blanking gate. Drawn whites come out ≈ 776.
8. **Motion.** At default Speed the bands stream out of the tips at about one band per 4 s, and the stripes
   roll down about 9 lines/s.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Shields | number of shields across, (k + 160)/183: 0.87 .. 6.5; 213 = the photo's 2.04. The band amplitude stays fixed, so the shields narrow. |
| K2 Bands | the nested V bands inside the shields, gain ((k + 64)/576)²: 0 = plain stripes, 512 = the photo, 1023 = ×3.6 fine nested V's |
| K3 Stripes | the horizontal stripes, ((k + 64)/576)² × 7.34 per frame: 0 = one stripe type (nested bands only), 1023 = 26 cycles |
| K4 Speed | bipolar (±8 deadband, square law): the bands stream out of (+) or into (−) the tips (2 bands/s at full) and the stripes roll (0.5 cycles/s at full); 700 = the default drift |
| K5 Pillars | how far the blue pillars, the M body's sides and the seam stripes climb (the F1 level ±820 px): 0 = no pillars (zig-zag bands at the bottom), 1023 = pillars taking over the shields |
| K6 Etch | camera (or Osc) luma added to the field, 0 .. 320 px per full luma; 320 = 100 px |
| S7 Line | On (the photo's white and dark edge lines) / Off |
| S8 Tincture | Gules (the photo) / Azure (chroma inverted: light-blue plaid, teal M, olive triangles) |
| S9 Flip | Off / On: shields upside down (the stripes are not flipped) |
| S10 Stripes | On (the photo) / Off: plain nested orange bands |
| S11 Source | Video / Osc |
| P12 Depth | V-parabola depth, p/365: 0 = flat vertical plaid banners, 365 = the photo, 100 % = ×2.8 deep V's; the palette wraps into a second cycle of stripes, outline and M cascading at the bottom |

## Video
- **Video mode (the default).** Camera luma, as (Y >> 2) − 16 from a one-line buffer, is added to the field.
  - The bands, outlines and pillars bend and fray around the picture.
  - The white line follows the bent contours.
  - Bright pictures shrink the shields.
  - With no input, Video mode is the photo.
- **Osc mode.** A drifting 2-D triangle lattice (640 × 540 virtual px) feeds the same path, so the
  shields wobble gently. Etch scales it.

## Engine
- **Per-field sequencer (vblank zone).**
  - One serial shift-add multiplier, 14 slots. The slot is decoded two clocks ahead and the operands
    are pre-muxed.
  - A shared glide for K1, K2, K3, K5, K6 and P12 (1/8 of the gap per field).
  - It computes: the tile step and the line-start phase; the band gain; the stripe frequency and the
    phase at line 0 (from the timing ID's lines per 540 virtual lines); speed², the flow and roll steps;
    the F1 offset; the etch gain; the depth.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line on its own serial multiplier
  in about 107 clocks, so it fits SD's 858-clock line:
  - G = ((yq² >> 14)·9986 >> 16) + 17, then g = G·depth >> 10;
  - C1 = A1 ∓ |dy|·4089 >> 16 − ((dy² >> 12)·590 >> 16);
  - the crease peak (0.742 g + 0.258 C1), which sets the white-line fade;
  - the stripe bit, the LFSR wobble and the start phase.

  The results are handed to the pixel path in the horizontal blanking, after the previous line has drained.
- **Pixel path.** Latency 32 clocks (C_LAT = processing_delay_clks = 32) in every mode, with the syncs
  delayed to match. The keyer path is a 10-deep shift register with a per-mode tap.
- **Memories (12 EBR).**
  - Palette ROM, 512 × 32 (4 EBR). Its two low lanes carry real flags (dark / white line allowed), so the
    word maps at full width.
  - Luma line buffer, 2048 × 8 (4 EBR).
  - Two 256 × 32 box delay-line rings (2 EBR each), `no_rw_check`. They store {Y, U, V, avid, avid}.
    The running sums re-anchor in every horizontal blanking, so a bad block-RAM read can never leave a
    lasting offset.
- **Contracts.**
  - Generated avid, parity-locked.
  - One latency in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr, V = Cb).
  - Luma pre-scaled 16/13.
- **Verified (GHDL, 720p real timing ID, against the integer model `imodel.py`).**
  - Still frame with black input: bit-exact.
  - Video mode with the selfie: 99.97 % of pixels identical.
  - Osc frames 3 and 9: 99.4 %. They match the model's field 2 and field 8, so the flow runs exactly as
    modelled.
  - Integer model vs the photo at defaults: mean error 4.94 RGB levels; 4×4 region means within 4.0 levels
    (median 1.9); 64-px block luma std 17.1 vs the photo's 17.1.

## Status
- **v0.5.0 (2026-10-27), built with ROUTER1.**
  - Build command: `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_heirloom.sh shield`.
  - All six configs on seed 1. Fmax (post-route): HD Analog 78.36, HD HDMI 87.37, HD Dual 84.63,
    SD Analog 87.37, SD HDMI 83.49, SD Dual 82.91 MHz.
  - 6651–6665 / 7680 LC (87 %), 12 / 32 EBR. Every bitstream passed iceunpack.
  - Output: `out/rev_b/ron/heirloom/shield.vmprog`. **Not flashed.**
- **History.**
  - The first build used FF box delay lines: 7240 LC (94 %). Moving them into EBR rings brought it to 6650.
  - router2 also builds 6/6 on seed 1, but HD HDMI came in at only 74.29 MHz. Stay on router1.

## Deviations from the photo
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3: the seam column in the
  middle, with a shield to either side. The left-edge hold is not visible at SD, and SD is softer, because
  the boxes run per output pixel.
- **Left edge.** The picture fades in from black over about 12–16 px (box warm-up plus the generated
  avid's 4 px offset). The photo's capture border is about 3 px.
- **Right-arm white line.** It is modelled as a fade by the line's crease peak. It sits about 9 px right of
  the photo's line and ends a little earlier.
- **Dark line.** An approximation of a small measured dip.
- **Wobble.** A static ±2 px per-line pattern, the same every field.
- **M body corners.** The photo's corners are slightly rounded; the min() gives sharp ones.
- **Palette wrap.** The palette wraps every 1024 px of field. This shows only at high Depth or Bands, and
  is the designed P12 climax.
- **Osc mode.** At default Etch the lattice's positive mean shrinks the shields by about 45 px.
