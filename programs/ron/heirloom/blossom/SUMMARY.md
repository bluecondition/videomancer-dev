# Blossom — Heirloom #22

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230406_23_49_00_Pro.jpg` (1920×1080 capture, 2023-04-06). Replaces v0.1.0 (V-shaped ring
field with a boundary column), which Ron judged too abstract.

## Reading of the photo (measured)
Working files: `.heirloom_work/rb_blossom/` (`notes.md` has every number).
- **Composition.** Rows mirrored top to bottom.
  - Vertical repeat 524 lines (autocorrelation 0.946), 2.06 per frame.
  - Mirror axes by flip-correlation at y = 260.5, 522.5 and 784.5 (0.979 / 0.986 / 0.967), identical in R, G and
    B. Seams at y = −1.5, 522.5 and 1046.5. No channel misregistration (gradient cross-correlation peaks at 0 px).
  - The four half-period copies agree to 6.5–8 RGB mean abs, so the picture is a function of (x, Y), where
    Y = the distance to the nearest heart row (0..262).
  - **Heart rows** (y ≈ 0, 522, 1046): a magenta heart with a shallow cleft on the left (B > 128 from x = 555 on
    the seam, 544 at Y = 20–28) and a pointed tip at x = 1155. Its flank runs x = 1155 − 8.2 Y.
  - Along the heart row, left to right:
    - dark purple, then a mauve line at x ≈ 133;
    - green, then a grey-green line at x ≈ 181;
    - a red hump band (R 57 → 165 → 39 over x 200–352), then a pink-white line at x ≈ 385;
    - a red ramp (R 114 → 255 over x 400–505), then a blue ramp into magenta (x 526–556);
    - a lavender line at x ≈ 563, then the magenta core.
  - Past the tip: a green ">" chevron with a white line, then red fringe chevrons with dark, green-centred
    "eyes", then a flat magenta flood (the right ~third) with red streaks 11–16 lines apart along its edge.
  - **Lobe rows** (y ≈ 260, 784): magenta lobes at x 4–12, 23–77, 84–145 and 155–320, then a tangle of
    red / green / yellow-green bands with a 62–68 px horizontal ripple out to x ≈ 1100.
- **Palette** (k = 16 shares):
  - magenta (254,2,251) 23.2 %;
  - dark reds (157,7,18) 12.0 % and (105,10,11) 10.8 %;
  - near-black (49,20,17) 10.6 %;
  - dark green (30,65,17) 7.3 %, red (230,4,13) 5.2 %, olive 4.3 %, green (29,118,10) 4.0 %;
  - pink (247,5,168) 3.5 %, grey-green line 3.4 %, rose 3.1 %, yellow-green 3.0 %;
  - mauve line 2.9 %, purple 2.5 %, yellow 2.4 %, lavender line 1.9 %.
  - The label adjacency graph is a chain (magenta – pinks – red – dark reds – black – greens – yellow-green):
    one scalar field through a colourizer. Channel gradients are parallel (median |cos| 0.96–0.98).
- **Mechanism.** One smooth field through a colourizer whose palette changes between the heart rows and the lobe
  rows. The field grows to the right until the colourizer clips to magenta.
  - One palette fits to 13.3 RGB mean abs.
  - Two palettes blended by Y/262 fit to 11.0 (3: 10.9, 8: 10.5).
- **Fit.**
  - Field: quadratic B-spline on (x, Y), fitted jointly with the palettes by robust gradient descent from a
    colour-chain-position initialisation.
  - Knot-grid sweep (mean abs RGB): 16×4 10.15, 20×4 10.49, 24×3 10.82, 16×6 10.96, 24×6 11.84, 20×7 11.86,
    16×8.45 12.08, 12×12 13.18.
  - The 4096-entry ROM (8 EBR at 8 bit) picks 24 virtual px × 262/44 Y: 82 × 46 = 3772 control points, 11.8.
  - 16×4 (8296 points, 16 EBR) looks the same at a glance.
- **Texture.** Soft camera edges, no line tear and no comb.

## What it does
1. **V oscillator.** A per-line phase `ph += rows × 2^24 / 1080 × vstep` (Q24, field start = the roll). It is
   folded into the fold position u (triangle; Saw = one-way), which gives the control row j and fraction t
   (44 rows per half period).
2. **Field.** The line's knots are D[i] = w0·c[j][i] + w1·c[j+1][i] + w2·c[j+2][i] (quadratic B-spline weights of
   t). Per pixel, F = P0 + s·P1 + s²·P2 in the 24-px cell at the pixel's virtual x (Mirror: x > 960 reflects).
3. **Controls on the field.** F' = 3.4 + gain·(F − 3.4) + bloom ± breathe + melt·source.
4. **Colourizer.** Palette entry = F'·32 + 96 (256 entries over −3..5; below the first used entry the palette
   reflects; Hard = 16-entry poster steps). Two palettes: PA (heart rows) and PB (lobe rows), blended by
   w = u + Garden. The palettes are in hardware YUV, Y pre-boosted 16/13.
5. **Output stage.** Indigo exchanges U and V. Then the shell conditioning: chroma box pair [1 2 … 2 1]/16,
   luma [1 2 1], luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Bloom** | The magenta flood and the hearts: field offset ±1.5 palette units. 0 % = closed buds (small red hearts, a green / violet garden, a red flood); 50 % = the photo; 100 % = full bloom, magenta floods everything but the bands at the left edge. Glided. |
| K1 Rows | Heart / lobe row pairs per frame, 1 + 4k² (1–5; 528 = 2.06 = the photo). |
| K2 Bands | Contour density: field gain 0.62–1.62 about the flood edge. More bands squeeze into the garden; the palette reflects below its first band instead of going flat. |
| K3 Garden | The tangle's colour: palette blend offset ±0.75 (violet / teal lines … the photo … yellow-green). Bipolar, centre = the photo. |
| K4 Speed | The rows roll through the frame (the V oscillator unlocked). Bipolar, still at the centre ±8, square law; + = up. Default 684 = one row pair every ~8 s. |
| K5 Melt | How far the video (or Osc) bends the bands: up to ±2 palette units. Default 460. |
| K6 Breathe | Depth of a 12 s LFO (parabolic sine) on the bloom, up to ±1 unit: the hearts swell and shrink. Default 384. |
| S7 Mirror | Off / On: the left half mirrored; twin hearts point at each other. |
| S8 Palette | Photo / Indigo (U and V exchanged: blue hearts on teal). |
| S9 Edge | Soft (the photo) / Hard (16-entry poster bands). |
| S10 Wave | Fold (the photo) / Saw: the V oscillator's waveform; one-way rows with a hard seam. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in`) × Melt is added to the
  field. The bands bend around the picture and bright areas bloom. At default knobs the rows, hearts and flood
  still read, with the face woven into the garden.
- **Osc:** two slow drifting triangle waves (x and y) at half the video's swing go through the same path, so
  Melt stays live and the bands wobble slowly.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks), on one serial shift-add multiplier, 7 jobs:
  - k1²;
  - × 62137 → base (rows × 2^24 / 1080);
  - × vstep → line increment;
  - speed² (+¼);
  - LFO |sin| (parabola);
  - depth × LFO → bloom ± breathe;
  - W/2 × kx → column start.
- **Glides:** a shared bank (±4 deadband, 1/8 ease per field) on K1, K2, K3, K5, K6 and P12.
- **Per line:**
  - Kicked at the source avid rise of line n, it computes line n+1 into a ping-pong cell table; line 0 is
    computed in the vblank zone.
  - Setup takes 8 clocks: fold, 11u/32 rows, weights, j·82.
  - Then 82 knots × 3 rows are issued through the control-point ROM (4096 × 8, plus a fabric register), with
    8×4 split products, accumulated and rounded to Q3.
  - The 80 cells' {P0 11, P1 12, P2 13 bit} are written in about 260 clocks (fits SD's 858-clock line).
  - The line's palette blend is latched at the line switch.
- **Pixel path, 28 stages:**
  - Beam U (Q16 cells), clamp, Mirror, cell address.
  - Table read (EBR + fabric register).
  - Nibble-split s·P1 and s²·P2 products; F; the melt product.
  - × Bands split 4 | 4; F'; palette entry (reflect / clamp / Hard).
  - Palette ROM 256 × 60 (EBR + fabric register).
  - B − A, the blend products split 5 | 4, the sum, clamp; Indigo.
  - Chroma box ring; luma chain aligned to the window centre; [1 2 1]; 13/16; gate.
- **Contracts:**
  - Syncs ride a 256 × 3 EBR ring. `C_LAT` = `processing_delay_clks` = 28 in every mode.
  - Generated avid (`p_gavid` / `g_start`).
  - Virtual-1080 geometry from the timing ID: 4:3 SD shows the central 1440 virtual px.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Memories (18 EBR):** control points 8, palettes 4, cell table 3, chroma ring 2, sync ring 1. The cell table
  and rings carry `no_rw_check`.
- **Verification** (GHDL 720p60, decimation 1):
  - Static frame (Speed still, Melt and Breathe 0): 100 % of pixels identical to the RTL-exact `imodel.py`
    after the tester's 4-px capture offset.
  - Motion: Speed max raises the rows 12 lines (720p) over the captured frames, in the + = up direction.
  - The static sim caught a 1-bit overflow in the palette-blend sum (V saturating in the tangle), now fixed.

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** router2. Post-route Fmax:
  - HD Analog 94.45, HD HDMI 88.94, HD Dual 91.53 MHz (all seed 1).
  - SD Analog 85.48, SD HDMI 92.34 MHz (seed 1).
  - **SD Dual 88.28 MHz on seed 2** (seed 1 stalled the router).
- 6710–6741 / 7680 LC (87.4–87.8 %), 18/32 EBR. All six bitstreams pass iceunpack.
- Presets are hand-written placeholders: Blossom, Closed Buds, Full Bloom, Twin Hearts, Indigo Garden.

## Deviations from the photo, and why
- **Fine texture:** the photo's 12–30-line contour bands are finer than the 24 px × 6-line knot grid (set by the
  4096-entry control-point ROM). These are the tangle's zigzags, the red streaks with cyan specks along the
  flood edge, and the eyes' green centres. Those zones render smoother: local contrast is about 0.6 of the
  photo's.
  - Separable texture terms, band-limited noise and a procedural zigzag ripple were all tried and rejected:
    they either did nothing or looked artificial and smeared the clean bands.
  - A 16 × 4 grid (16 EBR) looks the same at a glance.
- **Contour lines:** the thin white / mauve lines between bands are present but a little softer and thinner in
  places. Each palette entry is a least-squares average where the fitted field is off by a fraction of a band.
- **Edges:** each line starts and ends with a 4-px (2 at 720p / SD) soft dark border from the chroma box
  alignment. The photo also has a dark left border.
- **Whites:** ≈ 777 (the HD HDMI bar fix scales luma by 13/16). The palettes are pre-boosted 16/13, so mid tones
  land on the photo's levels.
- **One averaged pattern:** the field is the mean of the four mirrored half-periods. The photo's slightly
  different bottom heart row and its camera / JPEG noise are not reproduced.
- **SD (4:3):** shows the central 1440 virtual px, which crops the far-left lobes (x < 240) and part of the
  flood. 720p and 1080 are the same picture.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware, so this is moot).
