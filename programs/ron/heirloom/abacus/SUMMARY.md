# Abacus — rows of glossy beads on their rods

**v0.6.1 (2026-10-28, phase 2 round 2): built, NOT flashed.** Phase 2 removes the X/Y zooms and resizes every bead
inside its own slot instead (Width, Height). Slide and Roll make each bead and each row swell and shrink on
its own, in X and Y alike; see "Phase 2" below.
Rebuilt in v0.5.0 from the photo WIN_20230605_01_03_50 (Heirloom #42):
rows of red beads between wavy blue/green bands, one bead every 188 px, the rows mirrored about the bead
rows. The beads change across the frame: dark maroon with green lenses and navy "eyes" on the left, bright
red ovals in the middle, pink-cored hourglass beads on the right. The band pinches shut into a red dot and a
yellow "Y" spark in the middle. Every bead carries cream/white glints on its rims. Presets are hand-written
placeholders.

## Reading of the photo
Measured on the frame and on a tile averaged over its 4 half-period copies (they agree to 6.1 RGB levels).
- **Vertical.** Flip-correlation puts a mirror at y = 554 (mean |diff| 7.8) and shift-correlation gives a
  period of 495 lines (8.5). So bead rows sit on lines 59 / 554 / 1049 and band centres on 306.5 / 801.5
  (2.18 periods per frame). d = distance from the bead row, 0 .. 247.5.
- **Horizontal.** There is no global repeat or mirror (residual 31–35).
  - The band-centre green is a clean cosine (rms 4.3; a triangle fits at 5.4). It peaks at x = 161, 348, 533,
    723, 913, 1100, 1289, 1473, 1667, 1853: period 188.0, 10.2 beads across. It rides a slow bowl, with
    G = 38.6 cos + 180 − 481u + 658u² − 163u³ (minimum 82 at u ≈ 0.45).
  - Bead centres sit on the cosine's troughs (254 + 188k), the gaps on its peaks.
  - In the band, blue is a window on green: B = 255 for G in 73..110, falling to 0 by G = 155.
- **Across the frame (trough columns k = 0..8).**
  - The bead-centre red goes 99, 190, 211, 248, 254, 248, …, pink 189 (+G/B 42), 97: past the red peak on
    the right, which makes the hourglass and X beads.
  - Bead depth goes 65 → 150 lines.
  - The red band-edge line sits at d = 170–205 (left), 120–240 (the pinch at x ≈ 800: red dot, Y spark) and
    165–180 (right).
  - Gap colour runs pinkish-brown, maroon, salmon (e27e66 at x ≈ 912–1100), maroon, olive-pink.
- **Left third.** Between bead and band lie an extra green layer (green diamonds at the gaps, d ≈ 100), navy
  eyes at the troughs (d ≈ 165; orange→grey arc, maroon interior) and red/pink bow-ties.
- **Gloss.** Glints are bright warm/neutral pixels: cream arcs on the bead tops and bottoms, cream streaks
  between the middle beads, orange and yellow sparks, grey band-edge glints.
  - They make up 4.9 % of the frame, 1–5 px wide.
  - The outlines change colour along a contour (orange on the left side, grey/violet on the right).
- **No single analog patch fits.** Additive and product separability fails (residual variance 0.42 of
  0.61, where a synthetic additive field gives 0.03). Palette purity of every amplitude-modulated
  vertical × 188-px oscillator family stays poor, per window, jointly, per row, per column and per
  channel. So the field is measured rather than modelled: one scalar field s(x, d) through one palette (the
  mandala/blossom knot-grid mechanism), plus a measured gloss layer.

## What it does
1. **Field.** s(x, d) is a bilinear knot grid of 32 rows (every 8 lines) × 129 columns (every 15 virtual px),
   fitted to a per-pixel solution of the photo.
   - Method: k-means(48) of the tile, spectral seriation of the clusters into a 1-D colour order, a smoothed
     per-pixel solve, then an alternating knot least-squares fit and palette re-learn.
   - The knot field departs from the free field by rms 0.035 (of 1).
   - The horizontal knot column comes from a triangle of the horizontal phase: the photo, then its mirror
     image, repeating.
2. **Controls on the field.** s' = 512 + Rims·(s − 512) + Swell + Etch·(luma − 110). The index is s'/2 into a
   512-entry palette: bright red … maroon … pink/cream … olive … navy … red line … violet … green … blue.
   The band's green diamonds are lifted to the photo's saturation.
3. **Gloss layer** (measured column by column, every 7.5 virtual px, from one tile copy of the photo).
   - Glint and fill pixels are classed into cream, pink, orange, yellow, grey, navy, green and amber where they
     differ from the base colour. Navy, green and amber are allowed only in the left third.
   - Runs are tracked by continuity into 2 slots per column: centre, half-width class (1.5 … 40 lines),
     amplitude (4 levels), colour, and a jump bit.
   - Per pixel, the centre and amplitude are lerped between neighbouring columns (the centre is held at a
     jump). Alpha = amplitude × clamp((w − |d − centre|) / 2 lines). The stronger slot wins, and its colour is
     blended over the base in Y/U/V.
   - Gating: glints only on beads and rims (s' < 768), fills where s' < 960. So at Swell 0 the blue/green
     sea has no glints.
4. **Output.** Palette Swap after the blend, then output conditioning and the blanking gate.

## Controls (v0.6)
| Control | Photo element it drives |
|---|---|
| K1 Width | every bead's width inside its own 188 px slot, spacing kept: −100 = slim slivers (×0.57) in wide gaps, 0 = the photo, +100 = flat-topped blocks filling the slot (×1.46, the flat core over 3/4 of it). The band diamonds at the gaps narrow as the beads widen. |
| K2 Height | every bead row's height inside its 495-line period: −100 = thin bead rows (×0.5) in tall striped bands, 0 = the photo, +100 = tall flat-centred beads (×1.33) with the bands at half height |
| K3 Rims | the dark rims round the beads and the band-edge lines: field gain 0.31–5× about the rim level (v0.5: 0.5–2.5×). 0 = faded salmon beads and a navy band, 100 % = posterized flat red beads and flat blue bands with thin rims |
| K4 Slide | the beads slide along their rods: bipolar pan, still within ±8 of centre; default 640 ≈ 0.8 px/field (≈ 49 px/s) leftward; maximum 14.5 virtual px/field. **v0.6.1:** every bead swells and shrinks on its own, in X and Y alike. Bead k's size offset is depth × triangle(3k/8 − phase), so neighbours sit 3/8 of a cycle apart and differ distinctly. The phase advances with the speed (a cycle in ~2.3 s at the default, ~0.5 s at full). The depth is full (±255 s' levels, pip to swollen) from 64 knob steps off the still zone. |
| K5 Roll | the rows roll up/down: bipolar vertical drift, still at the default centre. **v0.6.1:** every row swells and shrinks on its own the same way (row r: depth × triangle(3r/8 − phase)); the gaps between beads keep their level, so a swollen row stays a row of beads |
| K6 Etch | the picture's luma pushed into the field: bright areas push the contours towards the band colours, dark areas swell the beads (160 = default). **v0.6:** 1.5× stronger on Video (the Osc source unchanged, so the photo still matches) |
| S7 Mirror | left/right mirror about the centre: the left half (lenses, eyes, dark beads into bright beads) on both sides |
| S8 Palette | Photo / Swap (U and V exchanged: blue beads in red/magenta bands) |
| S9 Stack | Mirror (the photo's mirrored rows) / Saw (rows repeat and are cut flat at each band centre) |
| S10 Flip | beads and bands exchange places (s → 1 − s): red bands, blue beads. **v0.6:** it glides in ≈ 0.7 s (43 fields). The flip spreads from the bead cores outward through the contours (cores turn blue, then the rims, then the bands turn red), and switching back retracts it the same way. |
| S11 Source | Video / Osc |
| **P12 Swell** | bead size (palette offset): 0 % = dark pips with green rings in a blue sea (+0.375, deeper than v0.5's +0.25), 50 % = the photo, 100 % = the beads merge into red bars (−0.25) |

Glides (1/8 of the gap per field) act on K1, K2, K3, K6 and P12.

## Phase 2 (v0.6.0)
Ron's notes (2026-10-28): this program is one that prompted "don't prefer X/Y scaling": the individual
beads should be resizable dynamically; widen Rims; Slide should still scroll but also wave the individual
beads' sizes as they move, so they grow and shrink a little individually, and the same for Roll; Etch a
slight boost on the video side; Flip should glide as it flips; Swell deeper towards 0 %.
- **Elements and their drivers.**
  - Bead rows: Width, Height, Swell, Rims, Slide (with the size wave), Roll (with the row wave), Stack,
    Flip.
  - Bands, band diamonds and red band-edge lines: Height (band height), Width (diamond width, inverse),
    Swell, Rims.
  - Glints, lenses and eyes: they ride both warps (the gloss table is read at the warped position and d).
  - Camera: Etch.
- **Removed:** K1 Beads and K2 Rows (horizontal / vertical zoom). Bead count and row count are now fixed at
  the photo's 10.2 across and 2.18 down.
- **Bead warp (horizontal, pixel path).**
  - The bead phase of the folded table position is (position × 4) × 327 / 1024 mod 256: 256 = one bead
    slot, 0 = a bead centre. It is computed from the unfolded position, and the fold bit picks the
    mirrored half of the warp ROM C_AB_WT, so it costs no fold stage.
  - The table position moves by −f·T(phase) (in bead-slot units), f the bead-size factor (−0.75 .. 0.75).
    T rises with slope 1 from the bead centre and falls back to 0 at the gap centre. For growing beads it
    peaks at 3/8 of the slot (slope −3 into the gap); for shrinking ones it peaks at 1/4 (slope −1).
  - T is zero at both the bead and the gap centre, so beads of different sizes still join, and the mapping
    stays monotonic over the whole range.
  - f = 0.75 × (Width + depth × the Slide wave) / 512, saturated. The ROM holds T × 0.5875, so the pixel
    path's one product is a (10 bits) × T (7 bits) / 128.
  - A position pushed past a table end is reflected (the fold's own mirror image), so the warp stays
    continuous across the mirrored continuation.
- **Bead warp (vertical, line engine).**
  - The line's d moves by −b·T(d), in 1/8 lines. T is the slope-1 profile only (peak at 1/4 of the
    half-period), for both signs; the steeper grow profile cost an adder, a compare and a mux input for a
    look Height already reaches.
  - b = Height + depth × the Roll wave, Q10, saturated to −1 .. 1023/1024.
  - Both products (depth × |wave| and |b| × T) share one 11-clock serial shift-add in the line setup.
  - b is Q10 so that the Roll wave changes it smoothly line to line. A Q7 b stepped every ~6 lines, which
    would have jumped the contours by up to 1.4 lines.
- **Fit.** The first v0.6 build packed at 99 % LC. Standalone-yosys ablations priced it at 535 LUT
  (horizontal warp), 375 (vertical) and 343 (the gloss blend's three multipliers). Three cuts brought it
  down 364 LUT:
  - the gloss blend became gated shift-adds over 9 alpha levels (0.14 % of pixels move by more than 4
    levels; glint edges span 2 scanlines);
  - the vertical warp went to 1/8-line units with one profile and the shared serial unit;
  - the horizontal scaling moved into the warp ROM, with overflow saturation.
- **Flip glide.** The flip level τ steps ±24 per field (0 .. 1024); pixels with s' < τ are flipped.
- The photo stays reachable. With Width, Height and Slide still, the integer model's frame is bit-exact
  to v0.5's. At the default Slide 640, the slight size wave moves the frame by a mean 2.7 RGB levels from
  v0.5.

### Round 2 (v0.6.1)
Ron's notes (2026-10-28, after the v0.6.0 control sheet): the Slide and Roll waves should scale the beads in
both X and Y. They should be more prominent, with more scaling even at slow settings. The beads should grow
and shrink separately, with the scale offset along the line so one bead differs distinctly from the next.
- **The size waves are per bead now, not screen-fixed.**
  - Bead k (in the table, so it keeps its identity as it slides) gets S_k = depth × tri(3k/8 − θ_s), and
    row r gets B_r = depth × tri(3r/8 − θ_r).
  - The row index survives rolling: it is vp0's modulo steps plus the roll accumulator's wraps.
  - The phases θ advance 4 × |speed| per field.
  - The depth is 4 × |speed|, capped at 255, so it is full from 64 knob steps off the still zone.
  - Still (±8) means no pulse, which is the photo; the "Photo" preset sets Slide and Roll still. The
    default Slide 640 now pulses at full depth.
- **The mechanism is a per-bead Swell.** The line engine adds δ = (S_k + B_r · E_y(d)) · E_x(column) / 4
  to each knot column's field value R[j], clamped, before the Rims gain.
  - A bead's outline therefore moves outward or inward in X and Y alike, by the same number of pixels at
    any Rims setting.
  - E_y is 1 out to 183 lines from the bead row and falls to 0 at the band centre, where rows meet.
  - E_x is a 5-step fade (1 out to 0.28 of the slot from the bead centre, 0 at the gap centre), where
    beads meet. With it, a swelling row keeps its gaps (without it, rows merged into red bars).
  - The line engine tracks beads along the knot columns with a bead-phase accumulator (15/188 of a bead per
    column) and computes the next bead's S_k on the shared serial unit while it fills the current one.
  - It writes the column's δ / 16 into the row RAM's spare bits. The glint gate adds 2δ for shrinking
    beads, so their glints fade with the outline instead of floating in the gap.
  - A true geometric 2-D zoom per bead (a per-bead vertical warp) was priced at ~+380 LC, about 97 % of the
    chip, and not taken.
- **Removed:** the v0.6.0 screen-fixed Slide width wave (and its per-pixel multiply) and Roll height wave.
  Width and Height stay geometric and static.
- **Fit:** the first version added 207 LUT. Sharing one pulse triangle and one result negator across the
  line engine, and counting row wraps instead of comparing vp0 jumps, brought it back to 4109 LUT
  standalone (v0.6.0: 4101). The gloss soft-edge min(w − e, w + e) also became w − |e| (identical result,
  one adder fewer per slot).

## Video
- **How the camera enters.** Its luma (line buffer, (y >> 2) − 16, clipped) minus 110, scaled by Etch, is
  added to the field before the palette. The contours (bead outlines, band edges, glint gating) bend around
  the picture while the rows, beads and palette stay. At the default Etch the selfie reads in the band and on
  the beads, and the composition holds (cmp_final.png, bottom).
- **Osc** substitutes the shell oscillator, slow diagonal triangles drifting per field, through the same
  path. Every control stays live.

## Engine
- **Per-field sequencer** (vblank zone: no avid for 1024 clocks). It has one serial multiplier with fixed
  slots and one shared glide. It sets the start phases, the pan / roll accumulators, the Width factor, the
  Slide / Roll wave depths, the Height factor, the Rims gain, the Swell offset, the Etch gain and the Flip
  glide level.
- **Per-line engine.** Kicked at each line start, it computes the NEXT line's 129-entry field row: two knot
  rows blended by the line's fraction, written into the other bank of a 2 × 128 row RAM. d is the |triangle|
  of a 495-line oscillator, or a saw for Stack, then warped by Height. Each knot column then gets its bead's
  pulse δ (with the row's term) added before the row is written. One 11-clock serial shift-add does every
  product: the Roll pulse, its band fade, Height × T(d), and each bead's Slide pulse, just in time
  along the line.
- **Pixel path** (latency 28 clocks in every mode).
  - Triangle fold of the horizontal phase. In parallel: the bead phase (two shift-adds and one add), the
    warp ROM (512 × 16), Width × T. Five stages in all.
  - The warped position gives the knot column and fraction (6 bits), plus the gloss column (8 units =
    7.5 virtual px) and its fraction.
  - Row RAM, fabric copy, R1 − R0, lerp product, s − 512, Rims product, + Swell, + Etch, clip, Flip, palette
    ROM (512 × 32), fabric copy.
  - In parallel: gloss ROM (256 × 64: 2 slots × 2 columns), decode, the centre/amplitude fraction products,
    d − centre, the soft edge, alpha, slot select, the s' gate, (gloss − base) × alpha / 16.
  - Blend, Palette Swap, padding, output conditioning.
- **EBR (24).** Knot ROM 4096 × 8 (8), palette 512 × 32 (4), gloss table 256 × 64 (4), line buffer 2048 × 8
  (4), row RAM 256 × 32 (2), bead warp ROM 512 × 16 (2).
- **Contracts.**
  - damier's generated avid (parity-locked).
  - Output conditioning before the blanking gate: chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-sums, luma [1 2 1],
    both centred on the same pixel, then luma 13/16 (drawn whites ≤ ~775).
  - Colours in the hardware convention (U register = Cr, V = Cb).
- **Resolution.** Geometry is in virtual 1080-line units from the timing ID, so 720p shows the same picture.
  **SD (4:3) shows the central 1440 virtual px: 7.7 beads of the photo's 10.2, the same round shapes.**
  Interlaced modes get no field offset (field_n is tied high on hardware).
- **Build hook.** abacus.py holds the measured tables as integers (knots, palette, gloss runs, gloss colours),
  computes the bead warp table, and writes abacus_pkg.vhd.

## Status
- **v0.6.1 (phase 2 round 2) built 2026-10-28, router2, NOT flashed.**
  - **Seeds:** HD HDMI = seed 2, SD Analog = seed 2 (its seed 1 timed out); the rest seed 1.
  - **Fmax (post-route):** HD Analog 79.10, HD HDMI 75.69, HD Dual 77.21, SD Analog 80.87, SD HDMI 77.09,
    SD Dual 75.23 MHz.
  - **Resources:** 7284–7310 / 7680 LC (95 %, FULL band), 24 / 32 EBR. Every bitstream passed iceunpack.
    out/rev_b/ron/heirloom/abacus.vmprog (505 993 B).
  - **Verification** (GHDL 720p, decimation 1, Osc, K4 = K5 = 1023):
    - The first pulse netlist was one knot column off: the column-side registers travel with stage 0,
      not stage 1, so column j got column j+1's pulse (a 15-px lead on every bead's swell). The model
      matched only with the pulse shifted.
    - Fixed with one more delay stage. The built netlist then matches the bit-exact model
      (p2/im12.py) exactly: mean |ΔRGB| 0.000, 100 % within ±2. The fit took 4 pulse-phase fields, the
      pan one field behind, and a row-numbering offset of 2 (rows count from vp0's modulo steps), at
      the usual 3-px beam offset.
  - **Photo:** with Slide and Roll still (preset 0 "Photo") the model is v0.5's frame (0.14 % of pixels
    move by more than 4 levels, from the 9-level glint blend). At the default Slide 640 the beads pulse
    at full depth.
  - **Headroom:** at 95 % LC any further feature needs something removed. Ablate first.
- **v0.6.0 (phase 2) built 2026-10-28, router2 (the builder default), all 6 configs seed 1, NOT flashed.**
  - **Fmax (post-route):** HD Analog 74.57 (thin), HD HDMI 81.00, HD Dual 82.10, SD Analog 76.48, SD HDMI
    78.44, SD Dual 76.09 MHz (SD Fmin 27).
  - **Resources:** 7084–7103 / 7680 LC (92 %, tight), 24 / 32 EBR. Every bitstream passed iceunpack.
    out/rev_b/ron/heirloom/abacus.vmprog (498 953 B).
  - **Latency** 28 clocks (TOML processing_delay_clks = 28).
  - **Verification** (GHDL 720p, decimation 1, Osc, K4 = K5 = 1023 so both size waves run at full depth):
    two netlists on the way to the build each matched the bit-exact model exactly (mean |ΔRGB| 0.000,
    100 % within ±2) at a constant 3-px beam offset, with the pan one field behind the roll (x0 is set
    from the pan before that field's update). The build adds, on top of the second, the rescaled warp
    ROM, the overflow saturations and the single vertical profile, checked line by line against the
    model im9.py but not simulated.
  - **The photo stays reachable:** with Width, Height and Slide still the model is bit-exact to v0.5; at
    the default Slide 640 the slight size wave moves it by a mean 2.7 RGB levels
    (.heirloom_work/rb_abacus/p2/cmp_v060_default.png).
  - **First flash, things to look at:** Width / Height at both extremes, the size waves while sliding and
    rolling (fast K4 / K5), the Flip glide both ways, and whether the HD Analog bitstream's 0.3 MHz margin
    holds (if it glitches, rebuild with `NEXTPNR_ROUTER="--router router1"`).
- **v0.5.0 (gloss) built 2026-10-27, router2, NOT flashed.** Seeds HD Analog 5, HD HDMI 2, HD Dual 6;
  Fmax HD Analog 75.93, HD HDMI 74.54, HD Dual 75.85, SD 72–74 MHz; 6369–6406 LC (83 %), 22 EBR.
- **v0.5 verification** (GHDL 720p, decimation 1):
  - The default render matches the integer model imodel.py at mean |ΔRGB| 0.46, 86 % of pixels within ±1,
    at a constant 2-px beam offset.
  - K4 = 1023, fields 3 → 6: pure translation, 29 px (720p) to the left, residual 1.8; the gloss moves with
    the beads.
- **Metrics vs the photo** (4 × 4 region means; local contrast = rms of luma minus its 15 × 15 mean):

  | | region \|ΔRGB\| | contrast ratio | glint |
  |---|---|---|---|
  | v0.5.0 without gloss | 5.4 | 1.22 | 0 % |
  | with gloss | 4.8 | 1.38 | 2.4 % |
  | photo | — | — | 4.9 % |

## Deviations from the photo, and why
- **Glints are stepped and thinner on the ground.** The gloss layer is measured per 7.5 px column with 2 runs
  per column, so arcs come out in small vertical steps and short bars rather than smooth curves. They cover
  2.4 % of the frame where the photo has 4.9 %. One-pixel cream arcs become soft 2–4-line lines. A third slot
  fitted better but overflowed the chip (8106 LC).
- **Left-third eyes.** They read as dark maroon ovals with red rims, amber flecks on the arc and a navy
  surround, not the photo's clean navy eyes with an orange→grey arc. The green lenses are small green blobs.
  The left column carries 0.2 % glints where the photo has 3.4 %.
- **Y spark.** At the pinch (x ≈ 800) it reads as a double red ring with a small yellow centre.
- **Crisper than the photo.** Local contrast is 1.38× the photo's: the knot field's palette steps and the
  gloss edges are harder than the analog original's soft horizontal bandwidth.
- **One tile.** The top and middle bead rows differ slightly in the photo (mirror residual 7.8); both are
  drawn from the averaged tile.
- **Motion is designed.** The photo is a still and no oscillator patch reproduces it. Slide and Roll move the
  measured picture, and its left→right evolution travels with it, continuing in mirror image past the edge.
- **Capture borders.** The capture's dark top/bottom rows and left edge are not drawn.

## Working files
`.heirloom_work/rb_abacus/` (phase 2 in `p2/`: im12.py = the v0.6.1 integer model (im9.py = v0.6.0),
sheet6.sh, abl/ ablation harness, sheet_v050.png / sheet_v060.png / sheet_v061.png, cmp_v060_default.png;
sheet.png is now the v0.6.1 sheet):
- `notes.md`: measurement log, including every fit that failed.
- `T.npy`: the averaged tile.
- `kalt8.py`, `ka_K_h15.npy`, `ka_P_h15.npy`: knot-grid field and palette.
- `gcol4.py`, `gc4_cfg4.json`, `gc4d.npz`: gloss runs (2 slots, as built).
- `protogloss.sh`: renders `proto_final.png`.
- `imodel.py`: integer model of the RTL.
- `gen_hook.py`, `hook_template.py`: write `abacus.py`.
- `metrics.py`, `metrics_after.txt`: before/after metrics.
- `sheet.png`, `crops_gloss.png`, `cmp_proto.png`, `cmp_final.png`.
- `v1_renders/`: v0.5.0 without gloss.
