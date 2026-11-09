# Staircase — pixel stair glyphs climbing a cyan and grey brick checker

**v0.1.0 built 2026-11-09, NOT flashed.** Heirloom (video clips), from the clip `WIN_20230318_14_13_38` at
0.5 s (`photos/vid/WIN_20230318_14_13_38_t0p5.png`). The still is a digital pattern on a coarse cell grid. A
cyan / grey brick checker fills the ground. Diagonal chains of identical stair glyphs climb up-right across
it: black rails ending in a black / cyan X, magenta / yellow / white treads on the left edge, yellow / white
rungs on the right. The same session's other clip (`WIN_20230318_13_57_00`) is the same generator in a
pink / teal random block mosaic. Presets are hand-written placeholders.

## Reading of the photo (measured)
Working files are in `.heirloom_work/rb_staircase/` (`notes.md`, `cells.py`, `table.py`, `colours.py`,
`lattice.py`, `proto.py`, `imodel.py`).
- **Cells.** Every element is made of whole cells, 37.55 px × 30.14 lines (51.1 × 35.8 per frame).
  - Column edges sit at x = 25.5 + 37.55 k and row edges at y = 9.0 + 30.14 k (max residual 3 px).
  - The fitted phase is cell (25.81, 18.585) at the screen centre.
- **Eight flat colours** (medians of the cell centres, RGB):
  - cyan (9, 255, 243), 39 % of the frame;
  - grey (127, 121, 114), 38 %;
  - black (34, 29, 23), 5.5 %;
  - yellow (253, 251, 0), cream white (249, 248, 231), magenta (255, 12, 254), silver (191, 188, 176)
    and dark grey (62, 57, 50).
  - Interior spread is ≤ 2 RGB, except dark grey (9).
- **Bricks.** The ground is a checker of bricks 4 cells × 1 row (150.2 px × 30.1 lines). It alternates every
  row: cyan when floor((col − 23) / 4) and the row have equal parity.
- **Chains.**
  - Each glyph sits 8 cells right and 8 rows up from the one below it (300 px, 241 lines). Chains repeat
    every 32 cells (1202 px).
  - The main chain runs from the bottom-left corner (100, 1000) to the top centre (1300, 40), with a second
    chain in the bottom-right.
  - The whole frame is one 32 × 8 table in (col + row mod 32, row mod 8). 1785 cells classified: 0
    mismatches outside the top two rows.
- **The glyph.** 10 × 10 cells with 56 cells drawn, and its left edge is a 45° staircase:
  - two black rails one cell wide, 2 columns apart, 5 and 3 rows long, with dark-grey caps;
  - a 3-row black / cyan X under them, trailed by silver cells;
  - cyan between the rails;
  - magenta tips on the tread steps, with yellow and white tread cells;
  - a yellow + white rung pair on every other row, 2 cells right of the rails.
- **Top edge.** The glyphs whose group starts above the frame lose their X: rows 0–2 show only their treads
  and rung tops (the generator starts at the top of the frame).
- **Texture.**
  - Horizontal edges are soft: 10–90 % takes 8 px for Y and Cr and 9 px for Cb.
  - Vertical edges take 2 px.
  - There are two or three 1-row dark specks (about 30 px long).
- **The alt clip** (same session, `13_57_00` at 0.5 / 9.0 s) is a random block mosaic.
  - Its cells are 77.9 × 35.1 and 112.9 × 56.6 px. Its strip shows Ron sweeping the cell size from tiny to
    large.
  - Palette: pink (255, 7, 150) 28 %, teal (5, 200, 190) 27 %, green 10 %, blue 8 %, orange 7 %, yellow 6 %,
    spring green 5 %, purple 5 %.
  - The two dominant colours stand in for grey and cyan exactly as in the staircase.
- **Fit.**
  - Integer model vs the still at defaults: 8.25 mean |RGB|. Cell interiors come to 2.8, and every colour
    is within 2.6 except yellow (10.5) and white (39): the luma cap, see Deviations.
  - The default GHDL frame (720p) is bit-exact with the model.

## Motion in the clip
- **The geometry is static.**
  - m1–m3 vs the still: cross-correlation shift (0, 0), mean |diff| 3.4 (capture noise; m1 vs m3 is 1.5).
  - Frames at 0.1–0.4, 2.5, 3.5 and 4.9 s have the identical layout.
- **The opening frame is branched.** Strip frames 0–2 are all the clip's first frame (t clamps at 0).
  - At t = 0 (m0), below row 16 (y 480) each chain carries a second copy 8 cells to the right, making a dense
    doubled cluster (mean |diff| 16 vs the still).
  - By 0.1 s the frame is the single chain.
- **Branch flickers.** At 4.2–4.8 s (30 fps), three single frames (11, 14 and 16) show the same branch for one
  frame each: mean |diff| 15, 14 and 9, against 1.0 for normal frames.
- **What the program does with this.**
  - At the defaults the geometry holds still, as in the clip.
  - The branch flickers: a burst starts on about 1 field in 256 (every ~4 s at 60 Hz) and lasts 7 field
    pairs. Each pair (one clip frame) lights with p = 3/8 at a random cell row 13–20.
  - S7 Held keeps the opening frame's branch.
  - K4 Climb (centre = still) moves the glyphs along the chains.

## What it does
1. **Cells.** Two DDAs in virtual 1080-line coordinates give the cell column cx and row cy. Cell size is
   37.55 × 30.14 virtual px × the K1 scale, about the screen centre.
2. **Bricks.** A modulo-L counter gives the brick parity, and cyan or grey is chosen against the row parity.
   Mosaic mode replaces it with random runs: a 16-bit xorshift stepped per cell, seeded per row. A new code
   starts with probability ~2 / (L + 1), weighted like the alt clip.
3. **Glyph lattice.**
   - Group m starts at cell row 17 + Q m − ro, and its glyph's column 0 sits at 11 − G m + co (mod P).
   - Each cell reads the 16 × 16 glyph ROM twice: from its own group (A) and from the group above
     (B, rows Q..15), so glyphs taller than a group stay whole.
   - The top group's X cells are cut.
4. **Family 2.**
   - Cross: the same lattice mirrored about the centre, its column counters running down, drawn under
     family 1.
   - Branch: a copy P/4 cells to the right, drawn over family 1 below the branch row.
5. **Camera / Osc.**
   - A per-cell source value V is sampled at each cell's left edge on its row's first line, and read one
     cell row later from a ping-pong RAM.
   - V shifts the glyph columns by V · E and walks the brick's colour code along C g M Y W S K D by
     V · ELV steps.
6. **Colour.** The 3-bit code goes through a 16-entry palette (2 palettes × 8), in the hardware convention,
   with luma pre-scaled 16/13 and capped at 940.
7. **Output.** Y, U and V go through two [1 2 2 2 2 2 2 2 1]/16 boxes (the capture's softness and the HD
   HDMI bar fix), then luma 13/16 and the blanking gate.

## Controls
| Control | Element it drives |
|---|---|
| K1 Cells | the cell grid (the generator's clock dividers, Ron's knob in the alt clip): ×1/3 (a fine weave of ~150 × 108 cells) .. ×1 at 512 (the still) .. ×3 (a close-up of one glyph), exponential, glided |
| K2 Bricks | brick length 1, 1, 2, 2, 3, 3, 4, 4, 5, 6, 7, 8, 10, 12, 14, 16 cells over the travel (4 = the still at 448): a fine column checker .. long bars. In Mosaic mode it sets the run length |
| K3 Lean | cells each glyph steps right per group, G = 8 + (k − 512)/32: −8 (glyphs stepping down-right) .. 0 (vertical ladders) .. 8 (the still at 512) .. 23 (scattered, flat stairs) |
| K4 Climb | bipolar (±8 deadband, square law): the glyphs climb up-right (+) or down-left (−) along the chains, one row and G/Q cells per tick; full = 14.5 ticks/s, 700 = 1.9 ticks/s; centre = still (the clip) |
| K5 Rise | rows per group Q: 5, 5, 6, 6, 7, 7, 8, 8, 9, 10, 11, 12, 13, 14, 16, 20 (8 = the still at 448): overlapping, braided glyphs .. separate glyphs on a steep chain |
| K6 Etch | how far the camera (or the Osc) moves the glyphs and recolours the bricks: E = 0.81 P (k/1023)² cells, and 8 k/1023 colour steps per full luma. 224 = 1.2 cells and 1.75 steps |
| S7 Branch | Flicker (single-frame branches, as in the clip) / Held (the clip's opening frame: the chains doubled below mid-frame) |
| S8 Bricks | Checker (the still) / Mosaic (random runs of all eight codes, the alt clip) |
| S9 Cross | Off / On: a mirrored second family of chains crossing the first, as a lattice of X's |
| S10 Colours | Cyan Grey (the still) / Pink Teal (the alt clip's palette: teal, pink, purple, yellow, green, spring, blue rails, orange) |
| S11 Source | Video / Osc |
| P12 Stairs | the number of chains (spacing P = 128 · 2^(−3.54 p/1023)), glided: 0 % = one lone staircase crossing the brick wall; 578 = the still (32 cells); 100 % = a packed weave of glyphs 11 cells apart |

## Video
- **Video mode (the default).** Each cell takes the camera's 4-pixel luma average at its left edge, on its
  row's first line, scaled to 0..252.
  - It shifts that cell's glyph columns: up to 1.2 cells at the default Etch, so glyphs jog around bright
    areas but stay readable.
  - It walks the brick colour along C → g → M → Y …. At the default, bright areas turn the checker
    grey / magenta: the camera shows as a mosaic woven into the brick wall.
  - With a black input, Video mode is exactly the still.
  - At full Etch the bricks become an 8-colour posterised mosaic of the camera (the alt clip's look) and the
    glyphs scatter.
- **Osc mode.** A 640 × 540 virtual-px triangle lattice, drifting ~75 px/s right and ~48 lines/s down, feeds
  the same path.
  - It runs at 4× the camera's glyph shift (capped at P − 1) and 2× its colour walk.
  - At the defaults the chains bend in ~18-row waves over magenta / yellow / grey diamonds.
  - K6 scales it.

## Engine
- **Per-field sequencer (vblank zone: no avid for 1024 clocks).**
  - One serial shift-add multiplier in 11 slots, with the slot decoded two clocks ahead and the operands
    pre-muxed.
  - A shared glide for K1, K2, K3, K5, K6 and P12.
  - A 256-entry exp2 ROM.
  - It computes:
    - the cell steps kx and ky = 1/size · 2^(−e);
    - the line step and the DDA starts, with + half a pixel and + 12 px for the output boxes and the
      generated-avid lag;
    - P = 2^(7 − ep) and D = P/4;
    - E and ELV, and the climb increment (|v| − 8)²/16.
  - One divmod loop, a step per clock, runs six jobs: G mod P; co = floor(ro G / Q); the two column bases
    mod P; the brick phase mod 2L; and the top group's row and G·m mod P.
  - The climb phase and the branch-flicker state machine update once per zone.
- **Per-line engine.** It is kicked at each line start (and once in the zone for line 0) and computes the
  NEXT line in 17 clocks:
  - the cell row and new-row flag, the group row (wrapping at Q), G·m mod P and the group count from the top;
  - the four glyph-column starts mod P (A, B, family 2 A / B);
  - the B-enable, top-cut and family-2 gate flags;
  - the mosaic seed: the row + a constant, then 4 xorshift steps. Adjacent rows match 17.5 % against 17.7 %
    at random.
- **Pixel path (21 stages, C_LAT 20).**
  1. The cell DDA runs one pixel ahead, so the counters see only a registered carry: cx, four modulo-P
     glyph-column counters (up, or down for Cross), the brick counter and the mosaic xorshift.
  2. The cell-sample RAM (512 × 8, banked by cell-row parity, `no_rw_check`, collision-free).
  3. A fabric copy.
  4. The V · E and V · ELV nibble products.
  5. The shift S and level LV.
  6. Columns + S.
  7. − P.
  8. Mod P, and the glyph ROM addresses with their validity.
  9. Four glyph ROM reads.
  10. Fabric copies.
  11. Opacity (with the top cut), and the brick code + LV.
  12. Priority.
  13. The palette (LUT logic).
  14. A copy.
  15. Black off the generated avid.
  16. Box A sums.
  17. Box A output.
  18. Box B sums.
  19. Box B output.
  20. Luma 13/16.
  21. The gate.
- **Memory.** 6 EBR: 4 glyph ROMs, the exp2 ROM and the cell-sample RAM. No transparency emulation
  (yosys `emulate_transparency` = 0).
- **Contracts.**
  - damier's generated avid (parity-locked).
  - One latency (C_LAT 20) in every mode.
  - Virtual 1080-line coordinates: SD shows the centre 4:3, 720p the same picture as 1080p.
  - Colours in the hardware convention (U = Cr), from `staircase.py`.
  - The running-sum resync (both boxes snap to black after 8 blank samples).
  - No single-row lines: `thincheck.py` finds 0 one-row runs ≥ 24 px at the defaults, P12 0 / 100, every
    knob's extremes and every switch, in 720p, 1080p and SD.
- **Verified (GHDL vs `imodel.py`).**
  - Bit-exact (0 differing pixels, 720p60 decimation 1):
    - the default frame with a black input;
    - Branch Held + Mosaic + Pink Teal;
    - Cross + P12 / K5 mid-glide (724 / 300).
  - Climb (K4 1023, 13 frames): bit-exact with the model at ro = 2.
  - The Osc and camera frames match the model's structure: 4.0 / 2.5 mean |RGB|, from the Osc phase and the
    RTL's 4-pixel luma average.
  - Osc frames 2 and 10 differ (it drifts).

## Status
- **v0.1.0 (2026-11-09), router2 (the builder's default). All six configs on seed 1.** Fmax (post-route):
  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 86.46 |
  | HD HDMI | 90.64 |
  | HD Dual | 83.13 |
  | SD Analog | 88.43 |
  | SD HDMI | 83.09 |
  | SD Dual | 91.36 |
- 6453–6481 / 7680 LC (84 %), 6 / 32 EBR. Every bitstream passed iceunpack.
- `dupin.py`: 0 CARRY I0 == I1.
- Output: `out/rev_b/ron/heirloom/staircase.vmprog`. **Not flashed.**
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh staircase` (or `./build_heirloom.sh staircase`).

## Deviations from the photo
- **The luma cap.** White and yellow are drawn at the shell's ~776 output luma (the photo's 910 / 830 would
  clip on hardware).
  - The cream white reads light khaki (209, 208, 192 against 249, 248, 231).
  - Yellow is slightly darker (237, 235, 0 against 253, 251, 0).
  - White keeps a little warmth, so it stays apart from silver.
- **Edges.** Vertical edges are hard (the capture's are 2 px soft). The horizontal softness comes from the
  shell's two boxes, about 9 px against the photo's 8.
- **Left edge.** It fades in from black over ~12 px: filter warm-up plus the generated avid's lag. The photo's
  capture border is 3–5 px.
- **Not drawn.** The capture's 1-row dark specks (single-row lines read as sync glitches on hardware).
- **The branch.** It is modelled as one copy P/4 to the right of every chain, across the full width below the
  branch row. The clip's opening frame is a wedge-shaped cluster with a second copy at +16 cells in its lowest
  rows.
- **The mosaic.** Mosaic cells keep the still's 1.25 : 1 aspect (the alt clip's are about 2 : 1). The runs are
  random, with a mean of ~2.5 cells at the default brick length.
- **The camera** enters per cell, one cell row late. That is invisible as a position, but it means the camera
  shows at cell resolution only.
- **Resolutions.**
  - 720p shows the same picture as 1080p.
  - SD shows the centre 4:3 (about cells 6–45 of 51), so the outer chain fragments fall outside.
  - Interlaced modes are line-doubled (no field offset), as in the rest of the family.
