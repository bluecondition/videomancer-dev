# Mandala — Heirloom #6

**v0.6.0 FINAL (2026-11-03, phase 2): Ron approved the diag preview as built; it is merged into production.**
`out/rev_b/ron/heirloom/mandala.vmprog` repacks the previewed diag bitstreams (the sources are byte-identical; only the
program_id and name differ). The diag copy stays at `.heirloom_work/rb_mandala/diag/mandala_diag/` for a later round. See [Phase 2](#phase-2-v060).
v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230128_01_06_08_Pro.jpg` (1920×1080 capture, 2023-01-28). Replaces v0.1.0 (three orbiting
ring centres), which Ron judged too abstract.

## Phase 2 (v0.6.0)
No notes from Ron yet (free hand, built while he was away). The v0.5 controls were all channel-wide (one field, one
stripe shaper, one blue ramp, one red key), so the photo's shapes could not move separately. v0.6.0 makes the face's
**elements patch points**: a region ROM tags each of the 43 × 46 control points of the fitted field with an element
and a weight, and the line engine adds that element's per-field offset to the control point before the spline blend.
Each element's contours (and its red key, at 1/8) then move on their own, with soft edges (the weight falls 1, 3/4,
1/2, 1/4 over three control points = 72 px, and the quadratic spline smooths it further).

| Element | Control points | Photo part |
|---|---|---|
| 1 Eyes | ellipse round (i 19.2, j 18.6), core 3.2 × 4.4 | the two nested-ring eye systems and their red / yellow cores |
| 2 Mouth | F < −1.4 stripes below row 35 | the black jaw and the chin bands |
| 3 Cheeks + brow | the two lens cheeks + the brow cusp on the axis | the red / yellow cheek lenses, the X where the ring systems cross |
| 4 Forehead | the top red lobes + the nose-bridge lens | the red forehead lobes and the blue lens on the axis |
| 5 Outer | the edge lobes (x < 140) and the corner lobes | the red lobes with dark cores at the frame edges and corners |

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K3 Blue: blue-ramp offset ±2 stripes (the whole face bluer / greener) | **K3 Eyes**, bipolar, photo at the centre (512) | The eye elements get −4 .. +4 stripes. Right = **wide open**: the pits deepen into black pupils ringed by extra nested rings, the red irises narrow. Left = **shut**: the pits fill, the rings collapse to points and the eyes go pink. **Renamed control** (Blue → Eyes); the blue level is no longer on a knob (see S9 and the questions) |
| K6 Breathe: LFO depth 0 .. ±2 stripes on the whole field, default 384 = ±0.75 | **K6 Breathe**: 0 .. 384 exactly v0.5; above 384 the common LFO keeps deepening to v0.5's ±2 **and each element breathes on its own detuned LFO**, 0 → ±2.5 stripes | Element LFOs (parabolic sine, one phase accumulator): eyes 15.2 s, mouth 20.2 s, cheeks 11.6 s, forehead 9.7 s, outer lobes 27 s. They drift through every combination: eyes dilate while the mouth shuts, the forehead lobes swell, the corners darken. At or below the default the picture is v0.5 bit for bit |
| K2 Width: pulse exponent 0.33 .. 3 in 8 banks, photo bank 4 (default 576) | **0.12 .. 8 in 16 banks**, photo bank 9 (default **608**, the middle of its bank) | Twice the log range each way, finer steps. 0 = the stripes wash out into a pale / white face; 100 % = hair-thin lines on a blue-magenta face. Bank 9 is bit-identical to v0.5's bank 4 |
| P12 Rings: gain 0.25 + 3p², ×0.25 .. ×3.25 | **0.125 + 3.5p², ×0.125 .. ×3.6** | Photo still at 50 % (gain 256, bit-exact). 0 % = the rings dissolve: only the glow, the red lobes and one broad band remain; 100 % = denser moiré |
| K4 Red: ±1.5 key units | **±3 key units** | 0 = no red anywhere (a blue / green mask); 100 % = red floods the face and the outer field |
| K5 Melt: 0 .. ±2 stripes, linear, default 400 | **0 .. ±3.8 stripes**: the v0.5 line up to the default, then 2.5× steeper | Default unchanged (bit-exact); the top is 1.9× v0.5's |
| S9 Edge: Soft / Hard comparator stripes (subtle after the box filter) | **S9 Glow: Ramp (the photo) / Bands** | Bands folds the blue ramp every 2 stripes, so blue bands beat against the green stripes: the face turns pink / orange / blue banded, the outer greens get blue rings, the eye cores go pink |

Unchanged: K1 Flow, S7 Stripes, S8 Lobes, S10 Faces (the elements follow both folds), S11 Source. The defaults draw the
v0.5 picture: the integer model of v0.6.0 equals v0.5's RTL-exact model bit for bit at the defaults (three LFO
phases checked).
Presets: Photo (defaults; was "Mandala"), Broad Bands and Moire Climax re-aimed to the 16-bank Width, Gold Mask, Twin
Masks; new **Wide Eyes** (K3 max), **Banded Glow** (S9 Bands), **Living Mask** (K6 max, eyes half shut, more Melt).

**Implementation.**
- **Engine.** The region ROM (2048 × 5, 3 EBR) is read two issues ahead of C_CF / C_CR. Both addresses now
  self-step by row (+43, +43, −85), replacing v0.5's base counter and its three adders. The region code addresses a
  small offset RAM (one EBR, written in vblank, read by the engine, `no_rw_check`). The element's offset is scaled
  by the weight and added to the control point as it leaves its ROM, so the pipeline gains no FFs. The key gets 1/8
  of the offset, clamped to ±127.
- **Sequencer.** The LFOs run as slot triples on the existing serial multiplier: phase = 20-bit accumulator ×
  rate / 16, then the |sin| parabola, then depth × |sin|. Slots 4–6 are v0.5's common LFO (rate 16 = the
  accumulator itself, so it stays bit-exact) and slots 7–21 the five elements. They share v0.5's fold, subtractor and
  |sin| registers. The line-0 kick moved from zone clock 200 to 704.
- **Pixel path.** The Bands fold is a bit-select on the ramp address. The Blue offset and the Hard-edge logic are
  gone, and Melt is a single 9-bit path. Pixel depth (28) and latency are unchanged.

**Build** (router2, 2026-11-02): HD Analog s2 87.50 (s1 stalled the router), SD Analog s1 71.79, HD HDMI s1 76.92,
SD HDMI s1 83.43, HD Dual s1 77.91, SD Dual s1 72.21 MHz; 7,368–7,401 / 7,680 LC (95.9–96.4 %, +~350 over v0.5.1),
27 / 32 EBR. All six bitstreams pass iceunpack. No pin-skew screen; builder seeds kept. The first build of this
feature set packed at 7,556–7,582 LC (98.5 %). It was slimmed with no feature lost (shared LFO path, self-stepping
addresses, offsets in a block RAM, one-path Melt).

**Checks run.**
- GHDL analyse / elaborate / synth front end.
- An integer model of v0.6.0 (`.heirloom_work/rb_mandala/p2/imodel_p2.py`): at the defaults it equals v0.5's
  RTL-exact model bit for bit at three LFO phases. The table fields stay inside their RTL widths at every extreme
  (P0 / P1 / P2, key P0 ≤ 508 of 511 with the clamp).
- A sweep of every new knob curve at its RTL widths (`p2/sweep.py`): no wraps, monotonic.
- Three GHDL renders at 720p decimation 1 (Osc), against the model: the default (98.7 % of pixels within ±4); K3 max
  on the first netlist (98.7 %; a one-step region misalignment would score ~90 %); and, on the final netlist, K3 max +
  K6 max at LFO field 153, with the glides and the LFO phase preset in a sim-only copy (98.5 %). That last one checks
  the new addressing, every LFO slot and the offset RAM.
- Not checked: hardware; the Melt > 100 segment in RTL (sweep only); SD / 1080 renders.

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
2. **Offsets.** Per control point (v0.6.0): + its element's offset × weight (F; FR gets 1/8, clamped). Per pixel:
   F += Breathe LFO + Melt × source (camera luma or Osc). FR gets 1/8 of that, plus Red.
3. **Colourizer.**
   - G = pulse ROM[width (16 banks)][frac(F·Rings + Flow)] × envelope(F).
   - B = ramp ROM[F] (S9 Bands: the address folded every 2 stripes).
   - R = linear key on FR, ×27/8 into 0..255.
   - Optional channel swaps, then RGB → 10-bit YUV in the hardware convention (U = Cr, V = Cb).
4. **Softness.** A box-8 (box-4 at 720p / SD) on Y, U and V reproduces the original's band-limited stripes.
   Then luma [1 2 1], chroma box pair, luma to ~777.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Rings** | The contour stripes' density (how many nested rings): 0 = ×0.125 (the rings dissolve into the glow and red lobes), 50 % = the photo, 100 % = ×3.6 dense moiré (gain 0.125 + 3.5p²). Glided. |
| K1 Flow | Bipolar, still at the centre ±8. Positive streams the rings out of the eyes and mouth, square law; default 640 ≈ 0.8 stripe/s. |
| K2 Width | Stripe line width: washed-out bands → hair-thin lines (pulse exponent 0.12–8, 16 ROM banks; bank 9 = the photo, default 608). |
| K3 Eyes | The eye ring systems, bipolar ±4 stripes: left = shut (rings collapse, eyes fill pink), centre = the photo, right = wide open (black pupils, extra rings, narrower red irises). |
| K4 Red | The red lobes' size (corner, edge, forehead, cheek and eye cores) and the pink face area, ±3 key units. Bipolar, centre = the photo. |
| K5 Melt | How far the video (or Osc) bends the contours, up to ±3.8 stripes (v0.5's line up to the default 400, 2.5× steeper above). |
| K6 Breathe | 0–384: depth of the v0.5 slow LFO on the whole field (15 s; 384 = ±0.75 stripe, the default). Above 384 it deepens to ±2 and each element breathes on its own detuned LFO (eyes 15 s, mouth 20 s, cheeks 11.6 s, forehead 9.7 s, outer lobes 27 s), up to ±2.5 stripes more. |
| S7 Stripes | Green (the photo) / Blue: the stripe and glow channels swap (a gold mask). |
| S8 Lobes | Red (the photo) / Green: the key and stripe channels swap. |
| S9 Glow | Ramp (the photo) / Bands: the blue glow folds every 2 stripes (pink / orange / blue banded face). |
| S10 Faces | One / Two: a second fold at ±480 virtual px, two masks side by side. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in` so the raw input
  stays shallow) × Melt is added into F and, at 1/8, into FR. The contours bend around the picture and
  the red lobes swell on bright areas; at default knobs the mask still reads.
- **Osc:** two slow triangle waves (x and y) drift through the same path, so Melt stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks), on one serial shift-add multiplier, 21 jobs:
  - Rings gain 0.125 + 3.5p².
  - The column start −(W/2 − c)·kx.
  - Flow speed (fm²).
  - v0.6.0: the LFOs as slot triples (phase = 20-bit accumulator × rate / 16, |sin| parabola, depth × |sin|):
    slots 4–6 the common Breathe LFO (bit-exact with v0.5), slots 7–21 the five elements. The eyes add the K3
    offset. The five offsets are written into a small block RAM read by the line engine.
- **Glides:** a shared bank (±4 deadband, 1/8 ease per field) on K2, K3, K4, K5, K6 and P12.
- **Per line:**
  - Kicked at the source avid rise of line n, it computes line n+1 into a ping-pong cell table
    (bank by line parity); line 0 is computed in the vblank zone (zone clock 704, after the element slots).
  - Setup takes 5 clocks: row j, fraction t, weights. Then 43 knots × 3 rows are issued through the
    control-point ROMs (2048 × 12 and 2048 × 8, plus a fabric register), with 9×5 / 9×4 split products,
    accumulated and rounded.
  - v0.6.0: the CF / CR and region addresses self-step by row (+43, +43, −85); the region ROM runs two issues
    ahead. Its element code reads the per-field offset RAM, ×1/4..1 by its level; the result is added to the
    control point as it leaves C_CF (and /8, clamped ±127, to C_CR).
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
- **Memories (27 EBR, v0.6.0):** CF 6, CR 4, region 3, element offsets 1, cell tables 3 + 2, pulse ROM 4
  (16 banks), ramp ROM 1, box ring 2, sync ring 1.
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
- **v0.6.0 FINAL 2026-11-03: the phase-2 diag merged into production** (see Phase 2). router2: HD Analog seed 2 (87.50; seed 1 stalled the router), HD HDMI s1 76.92, HD Dual s1 77.91, SD Analog s1 71.79, SD HDMI s1 83.43, SD Dual s1 72.21 MHz; 7,368–7,401 / 7,680 LC (96 %), 27 / 32 EBR. The production vmprog repacks the diag bitstreams Ron previewed, so the seeds above are the ones in it. A source rebuild: `./build_heirloom.sh mandala` (seed 1 may stall HD Analog; the builder retries).
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). The once-a-field zone reseed is replaced by the per-line resync; off-avid luma is now 64 (was 0). Post-route Fmax: HD Analog s1 88.44, SD Analog s1 94.60, HD HDMI s1 93.58, SD HDMI s1 88.47, HD Dual s1 92.16, SD Dual s1 92.45 MHz; 7016-7055 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
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

## Open questions (asked 2026-11-02, still unanswered at sign-off 2026-11-03)

1. Bring the Blue level back on a knob?
2. Should per-element breathing also run at K6's default?

**Watch on hardware:**
- Renamed: Blue -> Eyes, Edge -> Glow.
- 96 % LC: little headroom for a second round.
- No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config and repack.

For a further round, start a new diag version from `.heirloom_work/rb_mandala/diag/mandala_diag/` (`.heirloom_work/diag_build.sh mandala` builds it).
