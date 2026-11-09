# Marble — cream and warm marble arcs rolling up the frame, cyan rhombi, blue-lined figures

**v0.1.0 (2026-11-09): built, NOT flashed.** Heirloom (video phase), rebuilt from the clip
`WIN_20230914_01_59_34` at 4.0 s (`photos/vid/WIN_20230914_01_59_34_t4p0.png`). The still shows curved bands that
alternate cream (with cyan nested-rhombus chevrons) and yellow / orange / red. Every cream band has a green fringe
on its leading edge and a yellow-to-orange fade on its trailing edge. The warm bands hold dark red figures outlined
in blue / violet, in two rows. Over the clip the arcs roll up the frame: chevrons, then knees, then arcs. Presets are
hand-written placeholders.
Working files: `.heirloom_work/rb_marble/` (`notes.md`, `proto7.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **An RGB patch.** The channel split separates three signals, one per gun:
  - **B is the band key.** It is 0 or about 197 (207 at the top). It rises over ≈ 0.09 of a band period and falls
    over ≈ 0.06. Thin bright contour lines sit inside the warm bands.
  - **G is a wider, slower window of the same bands.** It is 240 in the cream bands. It rises ≈ 0.04 period before
    B (the green fringe, with R low there) and falls ≈ 0.11 period after B (the yellow → orange fade). Inside the
    warm bands G sits on a floor that darkens down the frame and inside the figures.
  - **R is a nested-rhombus lattice over the whole frame,** independent of the bands. It has black cores, dark
    rings and white between.
- **The band oscillator is hard-synced and FM'd.** On every row, the cream rising edges fit (x + 192) = (n + c)·P(y),
  with **c = 0.757 constant on every row of every frame** (0.738–0.757). The 192 px is 44 px of sync plus 148 px of
  back porch, so the horizontal oscillator resets at hsync.
  - P(y) = 1/f(y), with f = f0·(1 + m·cos(2π(y − ypk)/Py)).
  - The still fits f0 = 1/485 cycles/px, **m = 0.282**, **Py = 1900 lines** and ypk = 948 (rms 14·10⁻⁶).
  - Pmin is 376–380 px on every frame and Pmax ≈ 700. **Duty (cream / period) is 0.485.**
- **The lattice.** Flip-correlation finds vertical mirror axes at x = 535, 1005 and 1480 (correlation 0.44–0.52).
  That is a kaleidoscope fold with a 945 px period (zigzag's 940.5 again).
  - The cores lie on chains at (t, y) = (120, 262), (245, 520) and (350, 785), where t is the distance from axis 535.
    That is a triangle pair rotated 24°, with Lv = 286 px along the chain.
  - Along rays from a core, rings come every ≈ 115–125 px. Cores are 30–60 px in radius.
- **The warm floor and the figures.**
  - In the warm-band cores, G fits 185 − 97·y/H − 78·ty (R² 0.34). Here ty is a vertical triangle with a 520-line
    period, zero at y 575, so the figure rows sit at y ≈ 315 and 835.
  - The figures are band-locked: in band coordinates, adjacent warm bands' G correlates 0.46–0.70 in the bottom half.
  - The blue lines cover 10 % of the warm band. Their median run is 10 px and their colour is about (118, 42, 150).
- **Palette** (k-means, 16 clusters, share of the frame):
  - cream (231,240,197) 16.6 %
  - yellow (233,182,1) 11.1 %
  - orange (239,106,1) 10.9 %
  - yellow-green (157,186,5) 7.1 %
  - dark olive (77,74,15) 6.5 %
  - cyan (18..114, 245..249, 197), ≈ 21 % across four clusters
  - red (202,43,17) 5.6 %
  - green (65,196,5) 4.7 %
  - violet line (109,49,153) 3.7 %
- **Texture.** Every edge is soft (30–50 px). There is no tearing.

## Motion in the clip
- **The arcs roll up.** The FM peak row, from Py = 1900 fits of the m-frames, goes 1018 (−0.5 s), 948 (the still),
  871, 793 and 723 (+0.5 … +1.5 s). That is −70..−78 lines per 0.5 s, about **148 lines/s upward**.
  - From t 4 to t 7 it is −152 lines/s. From t 0.5 to t 4 it is −115 lines/s.
  - One cycle takes ≈ 13 s.
  - The clip's chevrons (t 0.5), knee (t 4) and arcs (t 7) are the same sine at successive roll phases.
- **No sideways drift.** c is constant, so the bands are locked to hsync.
- **The lattice barely moves.** Crop shifts are 0..−8 px per 0.5 s, with one region at −20..−28 px upward.
  Correlation between frames is 0.3–0.86: a slow melt. The figures morph as the bands roll through them.
- **The other look of the patch** (clip `WIN_20230914_01_57_28`, 2 minutes earlier) has m = 0, about 22 straight
  stripes and a smaller lattice. It sets the far ends of Bands and Marble.
- **The program.** At the default Speed (698) the FM phase steps 22061/2²⁴ per field. That is 2.5 lines per field,
  150 lines/s at 60 Hz.
  - The model's peak row goes 948 → 873 → 798 at 0 / 0.5 / 1.0 s. The clip's goes 948 → 871 → 793.
  - The lattice drifts along its chains at 1/10 of the roll.
  - Three melt sines drift at 0.07–0.13 cycles/s, so the figures and the lattice wobble keep morphing at Speed 0 too.
  - A GHDL frame one field later is bit-exact with the model at that field.

## What it does
1. **Bands (B, G).**
   - u = ((x + 192)·f(y) − 0.757) mod 1 is a Q24 phase accumulator. The per-line engine computes the step from
     f = f0 + fm·cos(pv) and the start phase from f·4·(192 + x0).
   - Each line pv steps by Py⁻¹. Each field pv0 steps by the roll.
   - A 256 × 16 band ROM over u gives kb, kg and kr: the cream key, the wider G window, and the R dip on the leading
     edge.
2. **Lattice (R).**
   - A fold accumulator folds x into a kaleidoscope (period 945 virtual px).
   - Two Q20 accumulators H and V step ± along it: a triangle pair rotated 24°.
   - dl = 3.27·tri(H) + 1.21·tri(V) is the rhombus distance in rings, times the Rings gain.
   - The melt sum M (three drifting sines), stronger toward the bottom, and the camera / Osc are added. A 1024 × 8
     ring ROM (black core below 0.42 rings, then one dark ring per ring) gives R.
3. **Figures.**
   - Fg = 7·(1.375·tu·ty + 0.2·y/H + 0.14·(M + camera) + (1 − b1)/8 − q0 + zigzag).
     - tu = 1 − tri(u − 0.82) puts the figures at the same phase of every warm band.
     - ty is the vertical triangle, so they sit in two rows.
     - The zigzag is a 56 × 43 px triangle pair that jags the outlines.
   - Outside the figures the warm floor is gout(y) (180 → 95 down the frame). Inside, it darkens to 22.
   - Where |Fg| is small, a blue line is drawn: B up, G cut, R halved. It appears only outside the G window.
4. **Colour.**
   - G = floor + kg·(240 − floor). B = 197·kb + 210·line. R = ring·kr.
   - Guns permute. A fixed shift-add matrix gives Y (16/13 prescale, capped at 876) and clamped Cr / Cb.
   - Then the shell's chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16 and the blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bands | the band oscillator's frequency, ×1/4 .. ×4 (exponential; 512 = the still's ~4 bands across). 0 = one fat band, 1023 = ~16 fine stripes (the alt clip's look with P12 at 0) |
| K2 Arcs | the FM sine's vertical frequency, ×1/4 .. ×4 (512 = the 1900-line period). 0 = long leaning bands, 1023 = ~2.3 tight waves per frame |
| K3 Lattice | the cyan rhombus lattice's zoom, ×0.35 .. ×2.8 (exponential, anchored at the line start). 0 = huge soft rhombi, 1023 = a fine chevron mesh |
| K4 Speed | bipolar roll of the arcs (±8 deadband, square law). 698 = the clip's 150 lines/s upward; full = ~1200 lines/s; left rolls down. The lattice drifts at 1/10 of it |
| K5 Rings | rings per rhombus, ×0.30 .. ×3.3 (exponential). 0 = big cyan / green pools (cores only), 1023 = dense fine rings |
| K6 Figures | the figures' size: 0 = none (clean warm bands), 512 = the still, 1023 = figures flooding the warm bands. Also the camera / Osc gain (k6 / 128) |
| S7 Twin | Off / On: the band oscillator mirrored about the centre, so the arcs meet in pointed arches |
| S8 Inlay | Off / On: the cream and warm keys swapped (orange bands with cyan rhombi where the cream was) |
| S9 Lattice | Positive (the still) / Negative: R inverted, giving cyan bands with cream chevrons and green warm bands |
| S10 Guns | RGB (the still) / GBR: pink-white bands with gold chevrons, violet / blue warm bands, blue figures |
| S11 Source | Video / Osc |
| P12 Marble | FM depth, 0 .. 0.9: 0 % = straight vertical cream / orange stripes; 31 % (321) = the still's 0.28; 100 % = deep bends. Above 59 % the melt also veins the band phase (up to ±0.37 cycle), so the climax is a marbled swirl |

## Video
- **Video mode (the default).** Camera luma is taken unipolar (black = 0, white = 255) and scaled by K6. It enters
  the patch at two points:
  - **The lattice ring index** (white = +0.59 rings at the default K6). The cyan rhombi melt and shift around the
    picture.
  - **The figure field's melt term.** Bright parts of the picture swell into dark, blue-outlined figures inside the
    warm bands.

  The bands themselves are untouched, so the arcs read at any K6. **A black input is exactly the still.**
- **Osc mode.** A drifting triangle lattice (480 × 410 virtual px, about 58 px/s and 37 lines/s, bipolar) feeds the
  same two points. The lattice bends into large pools and the figures grow and travel. K6 scales it.

## Engine
- **Per-field sequencer (vblank zone).** One serial 20 × 16 shift-add multiplier runs 13 slots, with operands
  pre-muxed:
  - f0 (K1) and fm = f0·m (P12);
  - the FM step and its per-line value (K2 × ys);
  - speed² and the roll;
  - the five lattice constants × 2^e (K3);
  - the two lattice per-line steps.

  Exponents use a 256 × 16 exp2 ROM; each is converted once to a Q13 value of 2^e, so every product has a fixed
  slice. A shared glide covers K1, K2, K3, K5, K6 and P12. The texture and figure-row per-line steps are shift-adds
  of ys. The field motion (roll, lattice drift, melt and Osc drifts) runs at zone clock 700.
- **Per-line engine.** It is kicked at each line start (and at zone clock 1300 for line 0).
  - Its accumulators (FM phase, lattice H/V line starts, three melt phases, zigzag, Osc and figure-row phases, y)
    are loaded straight into the pixel path at the line start.
  - It borrows the field multiplier for two products: fm·|cos| (from a 1024 × 16 cosine ROM) and f·4·(192 + x0).
    The results are ready about 45 clocks in, well inside SD's 858-clock line.
- **Pixel path (22 stages to compose).**
  - The accumulators use one-carry-chain add/subtract (XOR form) for the fold direction and Twin.
  - The triangles, three 256 × 8 sine ROMs, the rhombus distance, Rings (9 × 7), the melt and camera terms and the
    vein feed the ring ROM and band ROM addresses at stage 8.
  - Then the figure product (tu 7 b × ty 8 b), Fg, the line and floor, the G blend, B, the line cuts, Guns, two pad
    stages, the matrix (3 stages) and compose.
- **Memories (15 EBR):** band ROM (1), ring ROM (2), three sine ROMs (3), cosine ROM (4) and exp2 ROM (1). Four
  `no_rw_check` rings carry the chroma box tap, the syncs, the output luma delay and the output chroma delay;
  yosys reports 0 transparency emulations.
- **Contracts.**
  - One latency, C_LAT 36, in every mode.
  - The generated avid (parity-locked) and the blanking gate.
  - Luma capped at 940 before the 13/16 scale.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention (U = Cr).
  - Running-sum resync on the chroma box.
  - No `CARRY I0==I1` in the netlist (dupin).
- **Verified (GHDL, 720p60, decimation 1, against `imodel.py`, 16-px borders excluded).**
  - Video with a black input: **100 % bit-exact** on Y, U and V.
  - Osc: 100 % bit-exact.
  - The frame one field later: 100 % bit-exact at its field (the roll and drifts advance correctly).
  - Camera (selfie.jpg): 63 % exact; the model reads the camera without the pipeline's pixel offset.
- **Model against the still.**
  - Mean abs RGB is 42.6 (R 71, where the lattice's exact placement differs; G 33; B 23).
  - The 4×4 region means are within 7–21.
  - The band geometry matches row for row (same f0, m, Py, c and duty).
  - Dark G in the warm bands is 24 % (still 25 %). Blue lines cover 5.4 % of the warm band (still 10 %).

## Status
- **v0.1.0 built 2026-11-09 with router2 (the builder's default), all six configs on seed 1. NOT flashed.**
- `out/rev_b/ron/heirloom/marble.vmprog` (456,142 bytes, unsigned). iceunpack OK on all six bitstreams.
- Post-route Fmax (builder, last line):

  | Config | Fmax (MHz) | LC |
  |---|---|---|
  | HD Analog | 88.87 | 6898 |
  | SD Analog | 82.96 | 6897 |
  | HD HDMI | 84.17 | 6899 |
  | SD HDMI | 89.54 | 6891 |
  | HD Dual | 88.43 | 6902 |
  | SD Dual | 85.48 | 6899 |

  LC is out of 7,680 (≈ 89.8 %); EBR is 15 / 32. `hd_clock_divisor` is untouched.
- **Rebuild:** `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh marble`
- **router1 does not suit this netlist.** A first router1 build passed HD Analog (93.4 MHz) and SD Analog, then
  stalled HD HDMI on seeds 1 and 2 (15-minute timeouts). It was aborted. Router2 routes each config in about
  70 seconds. Stay on router2.
- **Fit history.** The first cut came to 9497 LC (123 %).
  - Shortening the pipeline brought it to 8198. That step took the vein from the melt only (dropping the ring
    ripple), narrowed the pixel multipliers, and moved the per-line products to shift-adds and a DDA for y.
  - Further cuts brought it to 6954:
    - the line engine shares the field multiplier;
    - one-carry-chain add/subtract;
    - fixed-point exp2;
    - the lattice zoom anchored at the line start (no per-field fold or anchor products);
    - 13 slots instead of 23.
  - Widening tu and ty for smooth figure outlines cost 130 LC.
  - Pre-shifting the matrix partials, a 3-bit G blend and moving the output delays into block-RAM rings brought it
    to 6881 pack-only (6891–6902 placed).
- **No pin-skew screen.** If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Cream is dimmer.** It comes out at the shell's luma cap: output Y ≈ 776 against the photo's 861, so it reads
  slightly beige / grey. Cyan, yellow, orange and red are under the cap and exact.
- **The lattice is cleaner.** It is a clean rotated triangle-pair kaleidoscope. The photo's R lattice is melted,
  with organic distortions in the lower half. The internal melt bends it only gently; the camera adds more.
- **Figures.** They are blobs with zigzag-jagged blue outlines, in the photo's two rows at the same band phase. The
  photo's figures are more articulated (star / limb shapes with dark olive and black gradients).
- **Lines are sparser.** Blue lines cover about half the photo's fraction (5.4 % against 10 % of the warm band).
  The photo also has a line hugging each cream leading edge.
- **Lattice zoom.** K3 zooms about the line start (the left edge), not the photo's axis at x = 535. This saves four
  sequencer slots. The default frame is unaffected.
- **Vein.** Above 59 % P12 the vein uses the melt only; the proto's ring-ripple term was cut for budget.
- **Rounded periods.** Texture periods are rounded to shift-friendly multiples of the line step: figure rows
  512 lines (520 measured), melt waves 683 / 512 / 293 lines, zigzag 42.7 lines.
- **Interlace.** There is no field offset under interlace.
- **Resolutions.**
  - 720p is the same picture as 1080p.
  - SD shows the centre 4:3 (1440 virtual px), checked in the model against the 720p centre crop: the same arcs,
    lattice and figures, so about 3 bands across instead of 4.
