# Blossom — Heirloom #22

**v0.6.0 (2026-10-30): phase 2 round 1 from Ron's notes on v0.5.0, built, NOT flashed.** See
[Phase 2](#phase-2-v060). Source photo: `photos/WIN_20230406_23_49_00_Pro.jpg` (1920×1080 capture,
2023-04-06). v0.5.0 rebuilt it from the photo; it replaced v0.1.0 (V-shaped ring field with a boundary
column), which Ron judged too abstract.

## Phase 2 (v0.6.0)
Ron's notes on v0.5.0 (2026-10-30), and what v0.6.0 does about each:

| Note | v0.6.0 |
|---|---|
| Rows: something more dynamic than a plain vertical density change; separate the image elements and scale them differently | **K1 Rows is bipolar about the photo (centre).** Right: more rows (×1 → ×3 on average) that **cascade**: the top row is ×5.2 smaller and each row below grows, to ×0.75 (bigger than the photo) at the bottom. Each row's hearts are sized with their row, horizontally as well. Left: fewer rows (×0.44 at 0 %). The **hearts swell tall while the lobe rows squeeze** (a Y warp of the fold: u → u − w (u − 2u²)), and the hearts widen ×1.4 |
| Speed: something other than scrolling | **K4 Sway** replaces the roll. The rows sway sideways in a sine wave that travels through them, one cycle per row pair. Centre ±8 = still (the photo). Away from the centre the amplitude (to 9.8 cells ≈ 235 virtual px) and the rate (to ~1.4 s a cycle) both rise. Right = the wave travels down, left = up |
| Breathe doesn't do much: more dynamic, or modulated by the video | **K6 Breathe = a heartbeat travelling down the rows** (~2.4 s a beat, every other row pair at its crest). As it passes, a row grows taller and wider (to ×1.41 at full depth) and blooms (+2.3 / −1.2 palette units). The rows push each other along like a peristaltic wave. With Source = Video the bloom is scaled per pixel by the camera (dark areas barely bloom, bright areas double); with Osc, by the Osc pattern |
| Mirror and the palettes look good | unchanged (Mirror now reflects on screen, so it stays centred under every new X map) |
| P10 Wave: keep Fold as the default, Saw looks weird; get creative | **S10 Wave = Fold / Twist.** Twist sets each line's horizontal scale to cos(2π × its row phase). The picture turns about the vertical centre line row by row: the heart rows face front, the lobe rows are mirrored, and the rows pinch to a line in between, like a twisted ribbon. Sway, Breathe and the Rows cascade all compose with it |
| Bloom: 100 % more range on both ends | **P12 Bloom ±1.5 → ±3 palette units.** The slope near the centre is unchanged (0.75 v), and it steepens to 2.25 v beyond half travel. 0 % = deep buds (violet / blue garden, dark-red flood); 100 % = magenta floods everything but the reddest bands |

Defaults (Rows, Sway centred, Twist off) still give the photo. The one difference is the default Breathe
(25 %): a gentle heartbeat (rows ±8 %, bloom +0.6 / −0.3 units) keeps the picture alive. Breathe 0 gives
the v0.5 frame exactly.

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
1. **V oscillator.** A per-line phase `ph += step(n) × swell(n)` (Q24, 0 at the top of the field). The step
   starts at the photo's 2.06 rows per frame times the Rows top-row scale and changes by a fixed amount each
   line (the cascade). It is folded into the fold position u (triangle), then Y-warped on the left of Rows,
   which gives the control row j and fraction t (44 rows per half period).
2. **Field.** The line's knots are D[i] = w0·c[j][i] + w1·c[j+1][i] + w2·c[j+2][i] (quadratic B-spline weights of
   t). Per pixel, F = P0 + s·P1 + s²·P2 in the 24-px cell at the pixel's field column U.
3. **X map (per line).** U = 40 cells + sway + (x − W/2) × kx(n), with kx(n) = the X step × the Rows scale
   (cascade) × the heartbeat swell × cos(2π u) under Twist. U < 0 reflects (the field mirrors at its left
   edge); U > 80 cells clamps to the flood. Mirror runs the beam back from the screen centre.
4. **Controls on the field.** F' = 3.4 + gain·(F − 3.4) + bloom + beat bloom(n) + source × (melt + beat(n) / 4).
5. **Colourizer.** Palette entry = F'·32 + 96 (256 entries over −3..5; below the first used entry the palette
   reflects; Hard = 16-entry poster steps). Two palettes: PA (heart rows) and PB (lobe rows), blended in 32nds
   by w = u + Garden. The palettes are in hardware YUV, Y pre-boosted 16/13.
6. **Output stage.** Indigo exchanges U and V. Then the shell conditioning: chroma box pair [1 2 … 2 1]/16,
   luma [1 2 1], luma 13/16, blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| **P12 Bloom** | The magenta flood and the hearts: field offset ±3 palette units (0.75 v near the centre, 2.25 v beyond half travel). 0 % = deep buds (violet / blue garden, dark-red flood); 50 % = the photo; 100 % = magenta floods all but the reddest bands. Glided. |
| K1 Rows | Bipolar, centre = the photo. Right: more rows cascading from ×5.2 small at the top to ×0.75 at the bottom, hearts sized with their row. Left: fewer rows (×0.44), hearts swelling tall (lobe rows squeezed) and ×1.4 wide. Glided. |
| K2 Bands | Contour density: field gain 0.62–1.62 about the flood edge. More bands squeeze into the garden; the palette reflects below its first band instead of going flat. |
| K3 Garden | The tangle's colour: palette blend offset ±0.75 (violet / teal lines … the photo … yellow-green). Bipolar, centre = the photo. |
| K4 Sway | Bipolar, still at the centre ±8. The rows sway sideways in a wave travelling down (right) or up (left), one cycle per row pair. The amplitude (to ~235 virtual px) and the rate (to ~1.4 s a cycle) both rise away from the centre. |
| K5 Melt | How far the video (or Osc) bends the bands: up to ±2 palette units. Default 460. |
| K6 Breathe | Depth of a heartbeat that travels down the rows (~2.4 s a beat). Each row in turn grows (height and width, to ×1.41) and blooms (+2.3 / −1.2 units). The video (or Osc) scales the bloom per pixel. Default 256 (gentle). |
| S7 Mirror | Off / On: the left half mirrored on screen; twin hearts point at each other. |
| S8 Palette | Photo / Indigo (U and V exchanged: blue hearts on teal). |
| S9 Edge | Soft (the photo) / Hard (16-entry poster bands). |
| S10 Wave | Fold (the photo) / Twist: each line's X scale = cos(2π u). The rows turn about the vertical centre line: heart rows face front, lobe rows are mirrored, pinched to a line in between. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video:** the camera luma (2-px average, (Y − 512)/2, tapped straight from `data_in`) × (Melt + the
  line's beat / 4) is added to the field. The bands bend around the picture and bright areas bloom, harder on
  each heartbeat. At default knobs the rows, hearts and flood still read, with the face woven into the garden.
- **Osc:** two slow drifting triangle waves (x and y) at half the video's swing go through the same path, so
  Melt and the beat's modulation stay live and the bands wobble slowly.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks):
  - The Rows ROM `C_BL_ROWS` (256 × 32, by knob / 4) gives the top-row V scale and its fall to the bottom.
    The X scale equals it on the right of centre and is (1 + V) / 2 on the left; the Y-warp weight is 511 − k
    on the left.
  - One serial shift-add multiplier, 4 jobs: the top-row V step (photo step × scale), its change per line,
    the top-row X step, and its change per line. The steps go straight into the line accumulators. The per-timing
    photo step and slopes come from timing-ID tables.
  - Constants that fold the heartbeat's mean (1/3) out of the per-line products, so those stay unsigned:
    swell = (1 + d/3) − d·e², bloom − d/6, melt − d/12. Plus 40 cells − the sway amplitude.
  - Glides: a shared bank (±4 deadband, 1/8 ease per field) on K1, K2, K3, K5, K6 and P12. K4 and the switches are
    read at the zone start.
- **Per line** (kicked at the source avid rise of line n; computes line n+1; line 0 in the vblank zone):
  - The V and X step accumulators step down by their per-line change (never below 2^20 / 2^16).
  - Setup: fold; read u − 2u² from the trig ROM; then the **job machine** (a serial 20 × 16 multiplier, 6 jobs of
    18 clocks, each B operand read from the trig ROM `C_BL_TRIG` three clocks ahead):
    1. w (u − 2u²) → the Y-warped fold;
    2. depth × e²(heartbeat at phase hb − ph/2) → swell, beat;
    3. swell × |cos 2πu| (Twist) → X factor and the mirrored flag;
    4. X step × X factor → the line's X step;
    5. V step × swell → advances the V oscillator;
    6. sway amplitude × (1 + sin 2π(ts + ph)) → the line's centre.
  - Then the v0.5 knot pass: 82 knots × 3 rows through the control-point ROM (4096 × 8, plus a fabric
    register), 8×4 split products, accumulated and rounded to Q3. The 80 cells' {P0 11, P1 12, P2 13 bit} go
    into the ping-pong cell table (~400 clocks in all, which fits SD's 858-clock line).
  - At the line switch: palette blend, beat bloom and melt latched for the screen.
  - **Beam run-back:** after each line's generated avid falls, the beam loads the next line's centre and steps
    back W/2 X steps to its left edge, 16 at a time (≤ 60 clocks, inside HD's 280-clock hblank and the image
    tester's 64). Backward steps use ones' complement (1/65536 cell each, invisible). All beam controls are
    registered a clock ahead, so only one LUT sits before the 27-bit carry chain.
- **Pixel path, 28 stages:**
  - Beam U (Q16 cells), |U| (ones' complement), clamp, cell address.
  - Table read (EBR + fabric register).
  - Nibble-split s·P1 and s²·P2 products; F; the source × (melt + beat/4) product.
  - × Bands split 4 | 4; F'; palette entry (reflect / clamp / Hard).
  - Palette ROM 256 × 60 (EBR + fabric register).
  - B − A, × the blend in 32nds, the sum, clamp; Indigo.
  - Chroma box ring; luma chain aligned to the window centre; [1 2 1]; 13/16; gate.
- **Contracts:**
  - Syncs ride a 256 × 3 EBR ring. `C_LAT` = `processing_delay_clks` = 28 in every mode.
  - Generated avid (`p_gavid` / `g_start`).
  - Virtual-1080 geometry from the timing ID: 4:3 SD shows the central 1440 virtual px.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Memories (24 EBR):** control points 8, palettes 4, trig ROM 4 (cos sign/magnitude, heartbeat e²,
  u − 2u², 1 + sin), Rows ROM 2, cell table 3, chroma ring 2, sync ring 1. The cell table and rings carry
  `no_rw_check`.
- **Fit (phase 2):** the first v0.6 engine cost 8288 LC (107 %). The extras were an 8-job case sequencer
  with signed stores (~830 LC), a centring multiply and per-field adders. Rebuilt lean:
  - offset-binary tables, with per-field constants carrying the offsets;
  - ROM-fed B operands;
  - the beam run-back instead of the centring job;
  - a Rows ROM;
  - the palette blend in 32nds (one product per channel instead of two).
  The v0.6 beam's control cone (centre compare → run-back count → direction → 27-bit adder) first held HD to
  55–70 MHz; registering the controls a clock ahead removed it.
- **Verification** (v0.6, GHDL 720p60): an instrumented run confirmed the per-field products and the per-line
  steps (top-row V step 5.24 × the photo's, falling 299.5 per line at 720p; X step following; swell and twist per
  line). A combined-controls render showed the twist, swell and sway.

## Status
- **v0.6.0 built 2026-10-30, NOT flashed.** router2.
- **Two HD seeds are hand-installed after a pin-skew screen** (vmprog repacked; a plain
  `./build_heirloom.sh blossom` will NOT reproduce them):
  - **HD HDMI = seed 12**: 77.30 MHz; HDMI HSYNC +1.68 ns, data ≤ −0.73. The builder's seed 3 sat at +0.89
    with data straddling the clock (−2.33..+0.04).
  - **HD Analog = seed 5**: 80.11 MHz; HDMI HSYNC −2.09, data ≤ −1.44. The builder's seed 2 was +0.21.
  - Seeds 4–19 (HD HDMI) and 3–14 (HD Analog) were screened with `tools/hdmi_pin_skew.py`; the results are in
    `.heirloom_work/rb_blossom/skew/screen.txt` and `ship.txt`. No HD HDMI seed that met timing reached +2.
    s9 was +2.48 but only 65 MHz.
  - Both re-route byte-exact. To rebuild: regenerate the netlists with make into
    `.heirloom_work/rb_blossom/skew/<config>/`, run `skew/route1.sh hd_hdmi 12` and `route1.sh hd_analog 5`,
    then `vmprog_pack.py --no-sign --hardware rev_b --toml-path blossom.toml pack_v060 <out>`
    (`pack_v060/` is the staging directory). The builder's own vmprog is kept as `skew/blossom_v060_builder.vmprog`.
  - **If HD HDMI shows thin coloured lines, re-pick from `screen.txt`** (s13: −1.10 / ≤ −0.93, 75.87 MHz).
- **Post-route Fmax:**
  - HD Analog 80.11 (s5), HD HDMI 77.30 (s12), HD Dual 75.88 MHz (seed 2; HDMI HSYNC −1.30, data ≤ −0.97).
  - SD Analog 79.37 (seed 1), SD HDMI 70.79 (seed 1), SD Dual 76.71 MHz (seed 2); SD needs 27.
- 7278–7298 / 7680 LC (95.0 %, FULL), 24/32 EBR. iceunpack OK on all six; no `emulate_transparency`.
- `out/rev_b/ron/heirloom/blossom.vmprog` (519,322 bytes, unsigned). `program_config.bin` carries the new labels.
- **Verified:**
  - GHDL analysis, elaboration and synthesis front-end.
  - GHDL 720p60 renders (`.heirloom_work/rb_blossom/cmp_v06.png`): the default frame against v0.5's; Rows max +
    Breathe max + Sway + Twist (the cascade, swell, sway and twist compose); Rows 0 + Mirror (tall twin hearts,
    centred).
  - An instrumented run of the per-field and per-line values.
- Presets are hand-written placeholders: Blossom, Closed Buds, Full Bloom, Twin Hearts (now Rows 260: tall
  hearts), Indigo Garden, Tower (Rows max + Mirror), Ribbon (Twist + Sway).
- History: v0.5.0 (2026-10-27) router2, all seed 1 except SD Dual seed 2; 87.8 % LC, 18 EBR.

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
