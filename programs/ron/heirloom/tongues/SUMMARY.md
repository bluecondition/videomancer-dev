# Tongues — twin flame tongues between green bands, a window key opening across eight columns

**v0.1.0 (2026-11-04): built, NOT flashed.** Heirloom (video phase), rebuilt from the clip
`WIN_20230127_01_48_55` at 25.5 s (`photos/vid/WIN_20230127_01_48_55_t25p5.png`). The still has eight
vertical columns. At the left, thin green lines run on dark red. In the centre, red and yellow flame
tongues converge on a bright yellow apex. At the right, broad green bands run on black. Presets are
hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_tongues/`; `notes.md` has every number.
- **Columns.** Eight columns of ~240 px, with seams at x = 16, 257, 501, 741, 985, 1217 and 1457, and
  weaker sub-seams at 129, 425, 597 and 657. The picture is R and G only (B mean 1.1, max 109).
- **Vertical cosine.** The G bands at the right are a cosine of period 530 lines (2.04 cycles per frame).
  It is a plain cosine, not a fold: the frame mirrors about y = 531.5, and about 266 too. Its centre steps
  per column:
  - cols 0–1: 503;
  - col 2: 541 → 513, tilted inside the column;
  - col 3: 492 → 538, tilted inside the column;
  - cols 4–7: 537, 533, 529, 528.
- **Window key.** G is a window on the cosine, |cos − C| < w.
  - The lower edge is sharp (ΔF ≈ 0.1 in cos units, 25 lines); the upper edge is soft (ΔF ≈ 0.3, 51 lines).
  - At the right the window is [−0.8, +0.55]: 50 % bands, 142–147 lines tall.
  - Its half-width per column is 0.09, 0.10, 0.11, 0.25, 0.51, 0.67, 0.74, 0.79: hairlines at the left,
    opening to the right.
  - Its level rises along each column (col 0: −0.12 → +0.30 over x = 90..250; col 1: +0.07 → +0.41) and
    jumps back at the seam. That is why the lines spread apart inside a column and step at its end.
  - The level starts per column are −0.17, +0.18, −1.0, +1.0, +0.24, +0.04, −0.09, −0.05. Col 2 starts at
    the trough, so the tongue arcs open from its seam; col 3 sits at the peak, which gives the apex glow.
- **Line widths.** In cols 0–4 the inner lines are thinner than the outer ones (col 1: 6–12 vs 27–34
  lines). This is a vertical width term, 0.08·(|y − yc| − 260)/530, that fades to the right.
- **Red.** Each column's red is a + b·cos with a floor (a, b, floor):
  - cols 0–1: (64, 8, 64), (104, 16, 0);
  - cols 2–3: (308, 222, 64), (308, 222, 64);
  - col 4: (112, 24, 0);
  - cols 5–7: (62, 0, 0), (16, 0, 0), (16, 0, 0).

  The red's cosine is centred at 530 in every column. The dark "jaws" at y ≈ 265 and 795 in the centre
  columns are its troughs (R 86); the yellow at the top and bottom of cols 2–3 is its peaks.
- **Palette** (k-means, 10 clusters):
  - dark red (69,9,2) 25 %, red (121,7,0) 15 %, bright red (212,7,0) 10 %;
  - green (12,245,0) 13 %, olive (24,132,0) 7 %, lime (133,224,0) 3.6 %;
  - black (15,16,3) 12 %;
  - orange (244,100,0) 5 %, yellow (250,196,0) 4.5 %, (131,110,0) 4 %.
- **Texture.**
  - Line pairs are identical: the capture is line-doubled.
  - There is no tearing: 972 of 1079 lines have dx = 0, and the rest ±1 on diagonals.
  - The column seams are soft over ~10 px.
  - Blocky stepped patches in the centre columns' red; thin lines kinked at the sub-seams.

## Motion in the clip
Measured on the m-frames (t − 0.5 .. t + 1.5 s) and on the clip's two other stills of this patch:
- **Seams.** The column seams never move: the 8× saw is locked.
- **Tilt.** The vertical centre of the right bands wobbles 513 ↔ 528 and that of the left lines
  541 ↔ 504, in antiphase. This is a tilt of the vertical phase between the left and right halves:
  ±25 lines at ~1 Hz.
- **Breathing.** The band widths breathe: 132 → 145 → 122 → 153 → 115 lines (±0.1 in cos units).
- **Level drift.** The right columns' window level sinks (−0.03 → −0.31 over 2 s) while the left
  columns' rises (0.35 → 1.0); at +1.5 s the left lines merge at the cosine peak.
- **Other stills.**
  - t0p5 has the tilt the other way (−16 lines).
  - t21p0 has one vertical cycle per frame (Ron's knob halved the vertical frequency), with the window
    as a single thin curve.

At the default Speed the program plays all three motions:
- the tilt LFO: ±25 lines at 1 Hz;
- the level drift across the columns: ±0.25 at 0.125 Hz;
- the width breathing: ±0.078 at ~0.7 Hz.

Speed² scales all three rates, its sign reverses them, and the centre (±8) holds still. The integer model
gives right-band widths of 140 → 122 → 158 lines at 0 / +0.5 / +1.0 s; the clip's are 143 → 122 → 150.

## What it does
1. **Columns.** The width is cut into 8 (or 16) columns. Inside a column xl runs 0 → 1, held in a Q20
   accumulator whose carry is the next column.
2. **Cosine.** θ = line phase − column phase − tilt·xl, read from a 1024 × 12 cosine ROM.
   - The line phase steps by Cycles per virtual line.
   - The column phase is yc_k·step, where yc_k comes from the table and is offset by ± the tilt LFO
     according to the column's side.
3. **Window.**
   - The level is C = C0_k + (Stairs + drift ramp) + s_k·Saw·xl + Etch·luma.
   - The width is w = Open·(w_k + wr_k·xl) + the vertical term + breath.
   - G = min(cos − (C − w), 13/32·((C + w) − cos)), clipped to 0..255. Col 3 uses 13/64; Fill keeps only
     the first term. Cols 2–3 are scaled by 13/16 to give the photo's yellow.
4. **Red.** Per line and column, R = clip(Blaze·(a_k + b_k·cos(line phase − 530·step))).
5. **Colour.** The colour is (R, G, 0), converted by a fixed RGB → YUV matrix in the hardware convention
   (luma pre-scaled 16/13, capped at 940). Ice inverts the chroma.
6. **Output.** Two [1 2 2 2 2 2 2 2 1]/16 boxes run on Y, U and V: they give the photo's seam softness and
   are also the HD HDMI bar fix. Then luma is scaled 13/16 and the blanking gate is applied.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Cycles | the vertical cosine's frequency: 0.5 .. 8 cycles per frame (exponential; 2.04 = the still at 512). The t21p0 look's single cycle is at ~250 |
| K2 Stairs | a bipolar ramp of the window level across the columns (the clip's drift, frozen). Left: the right columns' bands sink to the troughs and merge. Right: the left lines climb to the peaks. 512 = the still |
| K3 Saw | how far the window level rises along each column: 0 = flat, stepped lines; 410 = the still; 1023 = 2.5×, where the tongue arcs sweep more than a cycle per column |
| K4 Speed | bipolar (±8 deadband, square law): the tilt wobble, the level drift and the width breathing. 680 = the clip's rates, 1023 ≈ 9×, left of centre = reversed |
| K5 Open | the window's width everywhere: 0 = hairlines (even the right bands become thin line pairs), 512 = the still, 1023 = 3× (solid green at the right, big yellow blobs in the centre) |
| K6 Etch | camera (or Osc) luma added to the window level, 0 .. 1.2 cos units per full luma (square law; 400 = 0.18): the lines and band edges trace the picture's contours |
| S7 Columns | 8 (the still) / 16: the saw and the tables at double rate, twice the tongues |
| S8 Key | Window (the still) / Fill: a one-sided key, so the tongues and bands fill to the cosine peaks |
| S9 Flip | mirror left ↔ right: the green ground at the left, the tongues pointing left |
| S10 Colours | Fire (the still) / Ice (chroma inverted: azure flame, blue apex, magenta bands on black) |
| S11 Source | Video / Osc |
| P12 Blaze | the red flame: 0 = no red (green lines and bands on black), 512 = the still, 1023 = 3×, red and yellow across every column |

## Video
- **Video mode (the default).** Camera luma, taken as (Y >> 2) − 16 from a one-line buffer, is added to
  the window level (Etch).
  - The thin lines at the left trace the picture as contours, the tongues bend around bright shapes, and
    the band edges at the right follow it.
  - Black means no shift, so with no input Video mode shows exactly the still.
  - At the default Etch the camera's influence is moderate: ragged band edges and lifted levels where
    the picture is bright.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting about 75 px/s, feeds the same path: the
  lines and bands undulate with the lattice, and Etch scales the effect.

## Engine
- **Per-field sequencer (vblank zone).**
  - **Multiplier.** One serial 18 × 16 shift-add multiplier runs 58 slots. A slot counter steps them, so
    no zone-counter decode sits on the multiplier's path. The operands are pre-muxed a clock ahead, and
    the result is stored 18 clocks after the load.
  - **Per-column tables.** The per-column slot groups write two 256 × 16 block RAMs:
    - T (the beam's table): phase offset lo/hi, level start, width start, level step;
    - E (the line engine's table): red a and red b, Q4, for the 4 distinct cosine columns.
  - **Flip.** Under Flip the sequencer loads the column END values instead: tables YCF = YC + PSI and
    WE = W + WR, plus the level's rise.
  - **LFOs and glide.** Three LFOs (tilt, drift, breath) are read from the cosine ROM at the zone start,
    with signed per-field steps. A shared glide covers K1, K2, K3, K5, K6 and P12.
- **Per-line engine.** The cosine for line n + 2's red is read in the hblank after line n, when the ROM
  port is free (5 clocks after the generated avid falls).
  - At each line start the engine computes the next line's red for columns 0, 1, 2 (= 3) and 4. Each red
    is a4 + b4·|cos| >> 11 >> 4, clipped to the floor, and is pipelined over 3 clocks after its serial
    product.
  - It also computes the vertical width term: 5 products of 20 clocks, done by clock ~100, which fits
    SD's 858-clock line.
  - The reds of columns 5 and 6 (= 7) are per-field constants. Lines 0 and 1 are prepared in the zone.
- **Beam.** It runs 12 px ahead of the generated avid: the two boxes' group delay (8) plus the shell's
  4-px gate-to-data lag, so the right edge is not cut.
  - The tilt, level and width accumulators step by the column's increments.
  - At every column boundary they reload from the next column's values, prefetched from RAM T at pixels
    24..28 of the current column.
- **Pixel path (15 stages to compose, C_LAT 20).**

  | Stage | Work |
  |---|---|
  | 2 | cosine address; level and width |
  | 3 | cosine ROM, luma |
  | 4 | etch nibble products, the width fade by column pairs (1, ½, ¼, ⅛) |
  | 5 | fabric copy of the cosine, etch sum, width + vertical term + breath |
  | 6 | the level |
  | 7 | the window edges |
  | 8 | the distances into the window |
  | 9 | the two clips |
  | 10 | min (the key) |
  | 11 | 13/16 for cols 2–3, the red mux |
  | 13–14 | the matrix (and Ice) |
  | 15 | clip and compose |

  After stage 15 come the two boxes, whose 8-back taps come from six 256 × 16 block-RAM rings (read 6
  behind the write pointer, then a fabric register), then 13/16 and the gate.
- **ROMs and RAMs (16 EBR).** Cosine 1024 × 12 (3), exp2 256 × 16 (1), line buffer 2048 × 8 (4), tables
  T and E (2), output rings (6). Every collision-free RAM carries `no_rw_check`; the only transparency
  emulation left is the line buffer's, as in lozenge.
- **Contracts.**
  - the generated avid;
  - one latency (20) in every mode;
  - virtual 1080-line coordinates, with the columns spanning the width in every mode;
  - colours composed in the hardware convention (U = Cr);
  - the output running-sum resync;
  - luma capped at 940 before the 13/16 scale.
- **Verified (GHDL, 720p60, decimation 1).** All comparisons are against the integer model `imodel.py`.
  - The default still (Video, no input) is 99.7 % bit-exact; the rest is the left 4 px, where the model
    has no beam lead.
  - Flip + Fill + 16 columns + Ice at maximum Speed is 99.7 % bit-exact at the sim's field. The
    neighbouring fields differ by 21–24 Y on average, so the LFOs move as modelled.
  - Osc and selfie Video renders: `cmp_final.png`.

## Status
- **v0.1.0 built 2026-11-04 with ROUTER1. NOT flashed.**
- `out/rev_b/ron/heirloom/tongues.vmprog`; iceunpack OK on all six bitstreams.
- Seeds and post-route Fmax:

  | Config | Seed | Fmax (MHz) | LC |
  |---|---|---|---|
  | HD Analog | 1 | 80.21 | 6986 |
  | SD Analog | 1 | 79.01 | 6992 |
  | HD HDMI | 1 | 79.23 | 6957 |
  | SD HDMI | **2** (seed 1 stalled router1 up to the 900 s cap) | 78.62 | 6974 |
  | HD Dual | 1 | 82.62 | 6978 |
  | SD Dual | 1 | 82.98 | 6984 |

  LC is out of 7,680 (≈ 91 %); EBR is 16 / 32 in every config.
- **Rebuild.**
  `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh tongues`, or
  `NEXTPNR_ROUTER="--router router1" MAKE_TIMEOUT=900 ./build_heirloom.sh tongues`.
- **router2 does not close this netlist.** On the r3 netlist, HD Analog seed 1 missed and seed 2 stalled
  (overused = 2).
- **Fit history.**
  - The first full build came to 8,526 LC (111 %).
  - Moving the output-filter taps into block-RAM rings, replacing the Flip read-modify-write with END
    tables, deduplicating the red and simplifying the LFOs brought it to 6,930.
  - Pipelining the line engine's red computation took HD Analog from 71.4 to 80.2 MHz.
- **No pin-skew screen.** If HD HDMI shows lines on hardware, re-seed that config.

## Deviations from the photo
- **Textures.** The photo's blocky stepped patches in the centre columns' red and the kinks in the thin
  lines at the sub-seams (x ≈ 129, 425, 597, 657) are not drawn. Gradients are smooth, and each column's
  lines are straight segments.
- **Centre tongues.** The photo's centre holds several nested thin orange outlines; the program draws one
  window there (one thin arc pair per cycle in col 2, a broad glow in col 3).
- **Breathing.** The width breathing is uniform across the columns. In the clip, the left lines thicken
  at +0.5 s while the right bands thin.
- **Brightness.** Brightest greens and yellows come out at the shell's luma cap (13/16, white ~777).
- **R dip.** The R dip inside saturated green, a capture artefact, is not reproduced.
- **Left edge.** The beam lead starts the picture 4 px early at the left edge, and the photo's dark
  capture border is not drawn.
- **Resolutions.**
  - 720p is the same picture as 1080p.
  - SD (4:3) shows all 8 columns across the narrower frame, so each column is 0.75× as wide relative to
    the height and the tongue arcs and line slopes are steeper. The vertical structure is identical.
  - Interlaced SD follows the shell's timing-ID line steps.
