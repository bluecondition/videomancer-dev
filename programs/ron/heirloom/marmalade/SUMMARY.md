# Marmalade — a sample-and-hold mosaic through a nine-zone orange colourizer (Heirloom #14)

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Source: `photos/WIN_20230307_23_36_05_Pro.jpg`.
v0.1.0 (hashed Mondrian blocks slid sideways) was judged too abstract. This version measures the photo's
mechanism and stores its held pattern. The picture is woven into the stage stripes.
Working files: `.heirloom_work/rb_marmalade/`: `notes.md`, `proto2.py` (float proto), `imodel.py` (integer
model), `sheet_a.png` / `sheet_b.png` (control sheets), `cmp_final.png`.

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
| K1 Cells | the 40.24 px cell grid | cells across, 12 .. 120 (47.7 = the photo), centred |
| K2 Rows | the 73.6-line row hold | rows per master cycle, 4 .. 64 (16 = the photo) |
| K3 Picture | the stripe bands (where the camera enters) | how hard the camera (Video) or the drifting triangles (Osc) bend the stage stripes, 0 .. ±0.6 stripe cycle; 25% default |
| K4 Speed | the rolling master | bipolar, still in the centre ±8: the master rolls (rows, stripes, shift band and FM move down together; +25% ≈ 1 row/s) and the stripes crawl through the blocks |
| K5 Slew | the soft edges and slivers | S&H slew 2^(−0.4 s): hard cells .. the photo (dark seam between magenta and amber) .. long rainbow smears |
| K6 Orange | the marmalade orange mix | 0 = unripe (magenta, cyan, yellow), 50% = the photo, 100% = burnt (zones still distinct) |
| S7 Shift | the stepped top/bottom bands | Stepped (the photo's 90 px) / Straight |
| S8 Keys | the flat yellow / amber / orange blocks | Warm (the photo) / Cool (cyan, blue, violet) |
| S9 Spacing | the stripe bunching | Bunched (FM, the photo) / Even |
| S10 Clocks | the row splits | One (the photo) / Two (a second clock at 1.31× re-picks the hashed half of the cells) |
| S11 Source | the camera | Video (default) / Osc |
| **P12 Stripes** | the stripe bands | 1 band per master at 0 (calm blocks with one band) .. 6 at 50% (the photo) .. 21 at 100% (dense candy stripes through every block) |

K1, K2, K3, K5, K6 and P12 glide (1/8 of the gap per field, ±4 hold). Every control changes the picture at any
P12 position (`sheet_a.png`, `sheet_b.png`).

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
