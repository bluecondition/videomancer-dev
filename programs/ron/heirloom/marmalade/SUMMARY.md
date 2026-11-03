# Marmalade — a sample-and-hold mosaic through a nine-zone orange colourizer (Heirloom #14)

**v0.6.0 FINAL (2026-11-03, phase 2): Ron approved the diag preview as built; it is merged into production.**
`out/rev_b/ron/heirloom/marmalade.vmprog` repacks the previewed diag bitstreams (the sources are byte-identical; only the
program_id and name differ). The diag copy stays at `.heirloom_work/rb_marmalade/diag/marmalade_diag/` for a later round. Phase-2 working files: `.heirloom_work/rb_marmalade/p2/` (sweep.py, check.sh, psim.sh, renders).
v0.5.0 (2026-10-21): rebuilt from the photo `photos/WIN_20230307_23_36_05_Pro.jpg`, built, NOT flashed.

## Phase 2 (v0.6.0)
Ron left no notes for marmalade (free hand, 2026-11-02). The map follows the phase-2 brief: per-element parameters
instead of the two global grid scales, about twice the range each side of the photo, bolder switches, more motion,
and one CV driving related modules (P12 count + band width; K3 stripes + cells). The defaults still draw the v0.5
picture.

| Was (v0.5) | Now (v0.6.0) | Notes |
|---|---|---|
| K1 Cells: 12 .. 120 cells across the whole screen, photo 47.7 at 58 % | **Bipolar, per row**: each row's S&H clock detunes by (8 + perm(row)) / 16 of ±1.5 octaves. Right = every row finer by its own factor (×1.7 .. ×4.4); left = coarser (×0.6 .. ×0.22); centre (default 512) = 47.7 cells | Columns break into staggered fragments. The line engine computes each line's clock (exp2 ROM) and centred start phase, so rows stay centred and the 90 px shift stays 90 px |
| K2 Rows: 4 .. 64 rows per master, photo 16 at 45 % | **Bipolar cascade**: the row clock chirps down the frame (top row = the photo's 73.6 lines). Right = rows shrink to ~8 lines at the bottom; left = rows swell into slabs (~900 lines). Centre (default 512) = 16 equal rows | Linear chirp of the row increment per line (a perspective plane); same picture at SD / 720p / 1080p in virtual lines |
| K3 Picture: stripe bend 0 .. ±0.6 cycle | **0 .. ±1.2 cycle** (v0.5's slope up to the 25 % default, steeper above); **above 25 % the source also bends the cell edges**, up to ±4.5 cells | Block edges trace the camera (Video) or wobble with the drifting triangles (Osc); zero cell bend at and below the default |
| K4 Speed | **Same within ±128 (incl. the default +128), top ×3.25** | Also the S7 Drift rate |
| K5 Slew: hard .. photo .. smears (left half near-dead) | **Bipolar about the photo**: 0 .. ~30 % ringing (overshoot through the zones beyond the target and back: rainbow fringes), ~31–37 % hard, 50 % the photo, right half smears (unchanged) | Signed 9-bit kernel ROM; rows 8..15 are v0.5's exactly |
| S7 Shift: Stepped / Straight | **Stepped / Drift** | Drift: every row also slides at its own rate and direction (−3 .. +3 × a phase set by K4, plus a slow free run so it moves at K4 = 0). The 90 px steps stay |
| S9 Spacing: Bunched / Even | **S9 Tilt: Level / Slant** (renamed) | 45° stripe bands (virtual coordinates); bunching always on |
| S10 Clocks: One / Two | **S10 Mirror: Off / On** (renamed) | The cell clock runs backwards past the screen centre: the mosaic mirrors about the centre |
| P12 Stripes: 1 .. 6 .. 21 bands | **1 .. 6 (50 %) .. 36 bands, and the band width**: 25 % of each cycle at 0 % → the photo's 50 % → 87 % at 100 % (the stripes take over the blocks) | Count curve unchanged below 50 %; the stage quantizer compares \|frac − ½\| with four per-field thresholds j · bq, bq 2048 .. 4096 (photo) .. 7162 |

Unchanged: K6 Orange, S8 Keys, S11 Source. K1, K2 and K5 display −100 .. +100 % with the photo at 0.
Fix: v0.5 started each line's cell phase from the previous line's shift state, so the first line of each shifted row
band was unshifted (2 lines per master); v0.6 uses the line's own start phase.

Presets: Photo (defaults), Photo Osc, Candy Stripe, Unripe, Big Blocks (K1 150, K2 300, K5 800, P12 300), and new
Cascade (Osc, K1 800, K2 1000, P12 700), Mirror Drift (S7 Drift, S10 Mirror, K4 700), Ringing (K5 0, S9 Slant, K3 700).

Build (router1, 2026-11-02): HD Analog s1 93.01, SD Analog s1 94.42, HD HDMI s1 92.16, SD HDMI s1 95.17,
HD Dual s1 94.98, SD Dual s1 96.07 MHz; 6,836–6,854 / 7,680 LC (89 %), 10 / 32 EBR. No pin-skew screen.

Checks: GHDL analyse / elaborate / synth OK; p2/sweep.py (all knobs at RTL widths: no wraps, defaults bit-exact);
default-frame GHDL 1080p dec 1 Osc render vs v0.5 rtl_osc.png: 98.1 % identical (rest: 1-px shifts of the Osc-tilted
stripe edges, line-end columns from the v0.5.1 resync, and the shift-toggle rows 145 / 734 fix); one 720p render with
K1 / K2 / K5 / P12 / K3 extreme plus Slant and Mirror. Drift motion and Video-mode cell bend not simulated.

## Engine changes (v0.6.0)
- Line engine (one serial shift-add multiplier, ~46 clocks per line): E1 = detune × (8 + perm(row)) / 16; 2^frac(E1)
  from a 256 × 10 exp ROM; the row's cell clock inc × 2^E1 (14 bits, clamped); the centred start phase
  26 − inc × (W − offsets [+ 2 × shift]) / 2; the S7 drift T × rate(row). The row increment chirps by drp per line (K2).
- Pixel path: the cell DDA loads the line's start phase and clock and runs backwards past the centre under S10; the
  zone ROM address adds the K3 cell bend (split multiply); the stage phase adds the Slant ramp; the quantizer uses
  four per-field thresholds; the slew multiply takes the signed kernel. Latency 28, pixel depth unchanged (the extra
  register is in the parallel stripe path).
- Sequencer: the old K1 / K2 curve and start-phase slots now hold the P12 high-end square, the K2 chirp gain, the
  split K3 curve and the Slant slope (still 17 slots). The second row clock is gone.

## Reading of the photo (measured)
- **One vertical master.** Every vertical structure is locked to a ~1180-line oscillator (about 1.05 frames):
  - **Row clock.** Block rows have a period of 73.61 lines and a phase of −3.15 (boundaries 71 … 1030), so
    there are 16 rows per master cycle.
  - **Stage stripes.** Every stripe band runs k4 → k3 → k2 → k1 → k0 → back, with equal steps (for example
    10, 10, 10, 20, 10, 10, 10 lines), so the source is a linear triangle. Troughs reach L = 0 and plateaus peak
    at L = 8.0–8.2, so k = min(⌊8·tri⌋, 4) and the top half of each cycle is clipped flat. The stage boundaries
    are identical at every x. The 44 measured boundaries fit
    φ(y) = −0.331 + 0.005078·y + 0.436·sin(2πy/1180 + 2.0) to within 0.26 level: a triangle at 6.0× the
    master, FM'd by the master's sine.
  - **The shift.** Rows above y = 148 and from y = 733 down are the middle rows' pattern shifted exactly +90 px.
    The edges at 154 / 233 / 678 / 1161 / 1245 / 1600 reappear at 244 / 323 / 768 / 1252 / 1335 / 1692. The
    shift toggles on row boundaries half a master cycle apart: it is the master's square wave.
  - **Held pattern.** The rows repeat with a 4-row period (294 lines = master/4). Cell agreement is 0.62 at that
    lag, against 0.39–0.47 at the others.
- **Cells.** A horizontal S&H locked to H: family edges cluster at x = 33 + 40.24·n (R = 0.62), 47.7 cells
  across, with a cell boundary at the screen centre. Block widths are multiples of ~40.5 px, most often 81 px.
- **Nine zones × five stages.** With the stage index known, 9 families explain 96% of the pixels:
  - **Magenta stage palette.** M0 runs dark (64,65,47), grey (130,123,117), white (244,224,234),
    cyan (11,233,244), magenta (255,22,253).
  - **Dark stage palette.** D0 runs white, cyan (27,220,234), yellow (255,240,0), olive (34,39,19),
    dark (78,63,47).
  - **Orange tints.** Each stage palette appears at three orange-mix levels: pink M1 and red M2; brown D1 and
    dark orange D2. They fit (1 − m)·base + m·(292,103,8) clipped, with m = 0.33 and 0.73.
  - **Flat warm keys.** Yellow (255,240,0), amber (255,178,0) and orange (255,136,3) ignore the stage.
  - **Zone order.** Exhaustive search over all 9! orders against 260 edge slivers gives magenta, pink, red,
    orange key, dark orange, brown, dark, amber key, yellow key. That order satisfies 223 of the 260. A big step
    between cells slews through the zones between them; for example, magenta → amber leaves a dark D0 seam.
- **Held content.** The columns are fixed across rows (magenta at x 240–325 and 720–840, orange at 325–530,
  amber at 1160–1230), while the other cells vary by row on the 4-row cycle. An additive h(cell) + v(row) model
  explains only 52–56%. The pattern reads as generated and locked to the master, not as a camera scene.
- **Edges.**
  - Horizontal transitions take 12–13 px from 10 to 90% (G, B). G leads B and R by 3–4 px.
  - Half the amber↔magenta edges carry a dark notch, which is the D0 sliver. No other pair has one.
  - Edge positions scatter ±9 px (IQR) about the grid, depending on the family pair.
  - Vertical (stage) edges are hard.
  - There is no line tearing, interlace comb or noise.
- About 10% of cells change family 15–35 lines into a row (y 530, 694, 829, 971). No regular second clock fits
  these splits.

## What it does
1. **Master.** u (Q24) steps per line from the field-start phase. The row phase is R·u (Q28), the second
   clock runs at 1.31×, and the shift is latched at each row start from the master's square
   (frac(u + 0.40625) < 0.5).
2. **Cells.** A Q16 cell DDA per beam pixel, centred, with the 11-pixel filter delay folded into the start
   phase (90 px earlier on shifted rows). The cell index (6 bits) and the row (4 bits; for hashed cells under
   S10, the row is n1 + 5·n2) address the 1024×4 zone ROM holding the photo's pattern.
3. **Slew.** A feed-forward RC on the zone level: after each change, v = t − Δt·(1 − a)^d, with a 1024×8
   decay ROM and the multiply split in two. The rounded value is the slewed zone.
4. **Stage.** φ = the line phase (linear plus the sine-FM product, from the line engine) + Picture × (source −
   128) per pixel. The source is camera luma tapped straight off the input (Video) or two drifting triangles
   (1170 px and 590 lines; Osc). Then k = min(⌊16·|frac(φ) − ½|⌋, 4).
5. **Colour.** The palette RAM {zone, stage} → Y/U/V is rebuilt every field. It interpolates each zone's 5
   colours between the measured anchors by its orange-mix position τ (K6) and the key set (S8).
6. **Output.** Luma 8-box, chroma 3 px behind luma plus an 8-box, then the shell's [1 2 1] luma,
   [1 2 2 2 2 2 2 2 1]/16 chroma, 13/16 luma and the blanking gate.

## Controls
| Control | Photo element | Function |
|---|---|---|
| K1 Cells | the 40.24 px cell grid, row by row | bipolar per-row S&H clock detune: centre = 47.7 cells on every row; right = finer by each row's factor (×1.7 .. ×4.4); left = coarser (×0.6 .. ×0.22) |
| K2 Rows | the 73.6-line row hold | bipolar cascade from the top: centre = 16 equal rows; right = rows shrink to ~8 lines at the bottom; left = rows swell into slabs |
| K3 Picture | the stripe bands; above 25 % the cell edges | camera (Video) / drifting triangles (Osc) bend the stripes 0 .. ±1.2 cycle (25 % default); above 25 % they also bend the cell edges, up to ±4.5 cells |
| K4 Speed | the rolling master | bipolar, still in the centre ±8: the master rolls and the stripes crawl; top ×3.25 v0.5; also the S7 Drift rate |
| K5 Slew | the soft edges and slivers | ringing rainbow fringes (−100 %) .. hard (~−30 %) .. the photo (0) .. long smears (+100 %) |
| K6 Orange | the marmalade orange mix | 0 = unripe, 50 % = the photo, 100 % = burnt |
| S7 Shift | the stepped top/bottom bands | Stepped (the photo's 90 px) / Drift (each row also slides at its own speed, with K4) |
| S8 Keys | the flat yellow / amber / orange blocks | Warm (the photo) / Cool (cyan, blue, violet) |
| S9 Tilt | the stripe bands | Level (the photo) / Slant (45°) |
| S10 Mirror | the mosaic | Off (the photo) / On (mirrored about the centre) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Stripes** | the stripe bands | 1 thin band per master (0 %) .. 6 (50 %, the photo) .. 36 at 100 %, bands widening until the stripes take over the blocks |

## Video
The photo is synthetic, so the camera is woven into the patch the way an analog artist would patch it: camera
luma is summed into the stage triangle before the quantizer. The stripe contours bend around the picture
inside every block, while the blocks keep the photo's layout and colours. The camera is tapped straight off
the input (one register; its few-pixel lead is invisible) rather than carried through a deep pipe. Osc feeds two
slowly drifting triangles into the same input, so Picture stays live; at Picture 0 the Osc frame is the photo's
exact geometry. An earlier prototype that posterized the camera into the zones replaced the composition and
was dropped.

## Engine
- **Vblank sequencer** (the zone starts after 1024 clocks with no avid): switches, speed, roll and crawl
  first; then six glides on one shared datapath; then 17 slots on one serial shift-add multiplier. The slots
  compute the cell increment, the start phases, the per-line master step, rows, the stripe count, the FM depth,
  the Osc row step and the Picture gain. Direct terms (the second clock, the slew index) follow.
- **Palette engine** (vblank): 9 zones × 5 stages, each interpolated between two anchors from a 128×30 ROM on
  three 8-step serial multipliers, written into the palette RAM (`no_rw_check`: written in vblank, read in
  active video).
- **Line engine** (kicked at each line start, computing the next line): sine ROM → |sin|·B (8 steps) → the line
  phase; rows, the second-clock row and the shift latch.
- **Pixel path, 28 stages:** cell DDA → zone ROM → slew tracking → decay ROM → Δ·e (split) → zone; in
  parallel source → ×Picture (split) → phase → stage; then the palette RAM, filters, conditioning and gate.
- **Build hook:** `marmalade.py` generates `marmalade_pkg.vhd`: the zone table, the anchors (luma pre-scaled
  16/13 and capped at 940; capped saturated colours re-solve chroma; U = Cr), the slew ROM and the sine ROM.
- **Contracts:** generated avid; virtual 1080-line coordinates from the timing ID (720p renders the same
  picture as 1080p); one 28-clock latency in every mode; output conditioning before the blanking gate.
- **8 EBR:** zone ROM, 2 slew ROM, sine ROM, 2 anchor ROM, 2 palette RAM.

## Status
- **v0.6.0 FINAL 2026-11-03: the phase-2 diag merged into production** (see Phase 2). ROUTER1, 6/6 seed 1: HD Analog 93.01, HD HDMI 92.16, HD Dual 94.98, SD Analog 94.42, SD HDMI 95.17, SD Dual 96.07 MHz; 6,836–6,854 / 7,680 LC (89 %), 10 / 32 EBR. The production vmprog repacks the diag bitstreams Ron previewed, so the seeds above are the ones in it. A source rebuild: `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh marmalade`.
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Its Y / U / V pre-boxes resync too. Post-route Fmax: HD Analog s1 97.61, SD Analog s1 99.25, HD HDMI s1 97.55, SD HDMI s1 91.81, HD Dual s1 100.52, SD Dual s1 90.13 MHz; 5800-5821 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0 built 2026-10-21 with ROUTER1** (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh marmalade`), not
  flashed.
  - All six configs closed on seed 1.
  - router2 stalled on HD Analog (overused = 2, stuck). The netlist has no self-adds (`dupin.py` is empty), so
    router1 is safe.
  - Post-route Fmax (last line): HD Analog 100.63, HD HDMI 83.84, HD Dual 97.55, SD Analog 90.67, SD HDMI 95.05,
    SD Dual 96.24 MHz.
  - Fit: 5788–5807 / 7680 LC (75%), 8/32 EBR. All six bitstreams pass iceunpack.
  - `out/rev_b/ron/heirloom/marmalade.vmprog` is 376,551 bytes.
- **GHDL renders** (1080p, decimation 1, exact hardware hues):
  - RTL Osc matches the integer model at zero offset (mean difference 2.6 levels).
  - Against the photo, the 4×4 region means are within 1–27 (RGB sum); the worst cell is bottom-left at 27,
    slightly warm. Local contrast is 19.0 against the photo's 20.3.
  - RTL Video (selfie.jpg) matches the proto.
- **Motion** (720p, 6 frames apart, default Speed): the pattern rolls down 13.5 virtual lines, the stripes crawl
  through the blocks, and 37% of pixels change in 0.1 s.
- **SD:** NTSC fields render the central 4:3 crop of the same picture.

## Deviations from the photo
- Whites and yellows are capped by the shell's 13/16 output scale (the HD HDMI bar fix): output Y ≤ 776.
  White reads lilac-grey (225,205,215) instead of (244,224,234). Yellow and amber keep their saturation through
  re-solved chroma.
- The photo's block edges scatter ±9 px about the grid, depending on the family pair. Here every edge sits on
  the grid plus the modelled slew, so the dark magenta/amber seam is about 8–10 px instead of 15–25 px.
- About 10% of the photo's cells change family part-way through a row, irregularly. This is not reproduced;
  S10 Two offers a regular second clock instead.
- The Osc source is the photo's measured pattern in a ROM: 15 measured rows plus a copy of row 11 in the same
  4-row phase, and 14 mirrored padding columns. It is not a computed oscillator.
- At the default Picture (25%), Osc mode gently tilts the stripe bands; Picture 0 gives the photo's straight
  stripes.
- The measured G-channel lead is approximated by delaying chroma 3 px behind luma (YUV path).
- At SD 4:3 only the central 1440 virtual px show, so about 6 cells on each side are cropped. 720p renders the
  same picture as 1080p.
- The capture's black left border is not drawn.

## Open questions (asked 2026-11-02, still unanswered at sign-off 2026-11-03)

1. Bring back a plain global cell/row count?
2. Restore any of the dropped options (S7 Straight, S9 Even, the second row clock)?

**Watch on hardware:**
- Renamed: Spacing -> Tilt, Clocks -> Mirror.
- No pin-skew seed screening was run. If HD HDMI shows lines, re-seed that config and repack.

For a further round, start a new diag version from `.heirloom_work/rb_marmalade/diag/marmalade_diag/` (`.heirloom_work/diag_build.sh marmalade` builds it).
