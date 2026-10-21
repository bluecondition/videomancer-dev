# Mandala — Heirloom #6

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230128_01_06_08_Pro.jpg` (1920×1080 capture, 2023-01-28). Replaces v0.1.0 (three orbiting
ring centres), which Ron judged too abstract.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_mandala/` (`notes.md` has every number).
- **Composition.** A left/right mirror-symmetric mask, axis x = 996 (flip-correlation).
  - Eye lobes of nested rings at (575, 470) / (1417, 470), with red/yellow cores.
  - A brow cusp on the axis at y ≈ 520, where the ring systems cross.
  - Lens-shaped cheek lobes at y ≈ 640 and red forehead lobes at y ≈ 60.
  - A blue face outline that kinks in at y ≈ 280.
  - A black mouth, apex (996, ~880).
  - Red lobes with dark cores at the corners (300, 900) and edges (40, 400); green/navy bands at the far edges.
- **Mechanism.** One smooth field F drives three channels.
  - **G** is a soft pulse of frac(F): peak 234 at phase 0, trough 6 at 7/16, off below F = −1.8.
    These are the fine contour stripes; periods run from 16 px (chin band) to 100+ px (edges).
  - **B** is a monotone ramp of F: 3 at F ≤ −0.5, then 14, 36, 49, 85, 96, 123, 141, 168, 178, 197, 214
    every half stripe, 244 at F = 5.5, 254 above 6.5.
  - **R** is a soft key on a second, slower field FR. Per-channel mirror axes are R 1012, G 994, B 992.5:
    red is misregistered ~17 px right on the original.
  - Pink, white, yellow, cyan and navy are combinations of these three channels. k-means 16 shares:
    navy 13.7 %, red 10.6 %, blue 8.6 %, green 8.5 %, black 8.4 %, magenta 8.2 %, azure 6.1 %, rose 5.7 %.
- **Texture.**
  - Soft, symmetric edges: the R key's 10–90 % rise is 11–14 px.
  - Horizontally band-limited (≈ a box-8 at 1080).
  - No line tear (row-to-row shift 0 on all 1079 rows); low noise (G σ ≈ 11 in the black mouth).
- **Fit.**
  - Stripe phase from a spiral-phase quadrature transform of G, its sign from the B gradient.
  - Integer stripe order from B, quality-guided unwrap into the B-clipped regions.
  - Robust weighted fit of a quadratic B-spline: 24-virtual-px grid, 43 × 46 control points over the half
    frame. Residual median 0.05 stripe, 90 % 0.21 stripe.
  - The red key: its own spline on the same grid, fitted to the signed distance of the R > 127 boundary.
    99.1 % pixel accuracy.

## What it does
1. **Field.** F (Q9 stripes) and FR (Q7 key units) are the fitted surfaces. Their control points are
   embedded in `mandala.py`, which generates `mandala_pkg.vhd`.
   - Per line, the engine blends three control rows into one row of 43 knots, using the uniform
     quadratic weights w0/w1/w2 of the line's fraction.
   - Per pixel, the cell polynomial F = P0 + s·P1 + s²·P2 is evaluated at the mirrored column
     |x − axis| (FR at |x − axis − 16 virtual px|).
2. **Offsets.** F += Breathe LFO + Melt × source (camera luma or Osc). FR gets 1/8 of that, plus Red.
3. **Colourizer.**
   - G = pulse ROM[width][frac(F·Rings + Flow)] × envelope(F).
   - B = ramp ROM[F + Blue].
   - R = linear key on FR, ×27/8 into 0..255.
   - Optional channel swaps, then RGB → 10-bit YUV in the hardware convention (U = Cr, V = Cb).
4. **Softness.** A box-8 (box-4 at 720p / SD) on Y, U and V reproduces the original's band-limited stripes.
   Then luma [1 2 1], chroma box pair, luma to ~777.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Rings** | The contour stripes' density (how many nested rings): 0 = a few broad soft bands, 50 % = the photo, 100 % = 3.25× dense moiré (gain 0.25 + 3p²). Glided. |
| K1 Flow | Bipolar, still at the centre ±8. Positive streams the rings out of the eyes and mouth, square law; default 640 ≈ 0.8 stripe/s. |
| K2 Width | Stripe line width: fat bands → thin lines (pulse exponent 0.33–3, 8 ROM banks; bank 4 = the photo). |
| K3 Blue | The blue glow level (blue/pink face vs green/navy), ±2 stripes on the ramp. Bipolar, centre = the photo. |
| K4 Red | The red lobes' size (corner, edge, forehead, cheek and eye cores) and the pink face area, ±1.5 key units. Bipolar, centre = the photo. |
| K5 Melt | How far the video (or Osc) bends the contours, up to ±2 stripes. Default 400. |
| K6 Breathe | Depth of a slow LFO (15 s, parabolic sine) on the field: the eyes dilate, the mouth opens and closes. Default 384 (±0.75 stripe). |
| S7 Stripes | Green (the photo) / Blue: the stripe and glow channels swap (a gold mask). |
| S8 Lobes | Red (the photo) / Green: the key and stripe channels swap. |
| S9 Edge | Soft (the photo) / Hard comparator stripes. |
| S10 Faces | One / Two: a second fold at ±480 virtual px, two masks side by side. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in` so the raw input
  stays shallow) × Melt is added into F and, at 1/8, into FR. The contours bend around the picture and
  the red lobes swell on bright areas; at default knobs the mask still reads.
- **Osc:** two slow triangle waves (x and y) drift through the same path, so Melt stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks), on one serial shift-add multiplier, 5 jobs:
  - Rings gain 0.25 + 3p².
  - The column start −(W/2 − c)·kx.
  - Flow speed (fm²).
  - The LFO |sin| (parabola).
  - Breathe depth × LFO.
- **Glides:** a shared bank (±4 deadband, 1/8 ease per field) on K2, K3, K4, K5, K6 and P12.
- **Per line:**
  - Kicked at the source avid rise of line n, it computes line n+1 into a ping-pong cell table
    (bank by line parity); line 0 is computed in the vblank zone.
  - Setup takes 5 clocks: row j, fraction t, weights. Then 43 knots × 3 rows are issued through the
    control-point ROMs (2048 × 12 and 2048 × 8, plus a fabric register), with 9×5 / 9×4 split products,
    accumulated and rounded.
  - The 41 cells' {P0, P1, P2} are written as 39 + 30 bits, in about 140 clocks (fits SD's 858-clock line).
- **Pixel path, 28 stages:**
  - Beam U (Q16 → Q8 cells); Faces fold; |X| and |X − axisR|; cell index / fraction.
  - Table read (EBR + fabric register).
  - Nibble-split s·P1 and s²·P2 products; F, FR; offsets.
  - ×Rings split 5 | 5; envelope / ramp / key clamps.
  - Pulse / ramp ROMs (+ fabric register); G = pulse × envelope; routing; RGB → YUV shift-add terms
    and sums.
  - A 256-entry EBR ring holding {Y, U, V} for the box sums.
  - Output conditioning, blanking gate.
- **Syncs** ride a second 256 × 3 EBR ring (no FF shift registers); `C_LAT` = `processing_delay_clks` = 28
  in every mode.
- **Memories (21 EBR):** CF 6, CR 4, cell tables 3 + 2, pulse ROM 2, ramp ROM 1, box ring 2, sync ring 1.
  The cell tables and rings carry `no_rw_check`.
- **Shell:** heirloom v0.2.0 (escalator).
  - Timing-ID virtual-1080 steps.
  - Generated avid (`p_gavid` / `g_start`).
  - Source-width measurement.
  - Blanking gate on the latency-aligned generated avid.
- **Fit history:** the first RTL (FF delay lines, 25-bit coordinates) was 8115 LC (105 %) and failed
  placement. Moving the box window and sync delay into EBR rings, Q8 coordinates, the unaligned Osc and a
  4-bit FR fraction brought it to 7.03 k LC.
- **Verification** (GHDL 720p60 dec 1 vs the RTL-exact `imodel_rtl.py`):
  - Static frame: 98.7 % of pixels within ±4/1023, after the tester's 4-px capture offset.
  - Motion: the K1-max frame equals the model with the flow advanced 3 fields outward (99.8 % within ±4).

## Status
- **v0.5.0 built 2026-10-21, NOT flashed.** router2, all six configs seed 1. Post-route Fmax:
  HD Analog 95.68, HD HDMI 90.44, HD Dual 94.46, SD Analog 95.58, SD HDMI 90.27, SD Dual 91.11 MHz.
  7027–7085 / 7680 LC (91.5–92.2 %), 21/32 EBR. All bitstreams pass iceunpack.
- An earlier box-16 netlist stalled SD Analog on seed 1 (seed 2 passed). If a later edit stalls again,
  re-seed before restructuring.
- Presets are hand-written placeholders: Mandala, Broad Bands, Moire Climax, Gold Mask, Twin Masks.

## Deviations from the photo, and why
- **Red lobes:** the edge-of-frame lobes have a sharp black core where the photo has a soft dark
  gradient, and thin yellow ripple lines at their rims. The red key is a linear ramp on a 24-px spline,
  so it cannot hold the photo's soft core gradient.
- **Outer band and far edges:** the outer band (x < 200) renders teal/green where the photo is saturated
  navy/blue with weak stripes. The far-edge horizontal green bands become blobby cells. That strip
  carries little phase information, so the spline is extra-smoothed there.
- **Creases:** the sharp brow X and the outline kink at y ≈ 280 are slightly rounded. Explicit |x| kink
  bases gained only ~1 % in the fit.
- **Whites:** drawn whites are ≈ 777 (the HD HDMI bar fix scales luma by 13/16).
- **SD (4:3):** shows the central 1440 virtual px. The corner and edge red lobes are partly cropped;
  720p and 1080 are the same picture.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware, so this is moot).
