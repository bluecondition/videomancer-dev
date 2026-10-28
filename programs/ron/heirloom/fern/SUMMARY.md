# Fern — Heirloom #46

**v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed.** Source photo:
`photos/WIN_20230713_00_17_11_Pro.jpg` (1920×1080 capture, 2023-07-13). This replaces v0.1.0 (nested parabola
arcs over a ripple), which Ron judged too abstract. Presets are hand-written placeholders. Working files:
`.heirloom_work/rb_fern/` (`notes.md`, `proto_final.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Reading of the photo (measured)
- **Composition.** Active rows 1–1072, mirrored about row 515.5 (flip test on R).
  - Left: a blue ground of horizontal scanlines, with a small ">" dart at x 0–282, rows 372–661.
  - Right of a big curved ">" boundary (apex (856, 515.5), reaching x ≈ 370 at the top and bottom): nested
    pink/red/yellow frond arms crossed by vertical stems, with flames at the top and bottom edges. The cells
    between them are filled with a green/cyan maze.
  - Shares: ground 22 %, fill 41 %, fronds 31 %. Mean RGB: ground (4, 1, 140), fill (9, 128, 125),
    fronds (241, 80, 116).
  - k-means 12: blue (6,11,188) 16.5 %, navy (7,6,82) 14.6 %, cyan (4,155,180) 14.5 %, teal (6,103,86) 10.3 %,
    green (2,188,76) 9.3 %, red (245,13,92) 7.6 %, pink (246,89,117) 5.9 %, orange-yellow (245,171,102) 5.5 %,
    light pink (243,111,174) 4.4 %, magenta (236,20,173) 4.0 %, orange (248,115,66) 3.9 %, plum (121,28,99) 3.5 %.
- **Per channel (the key to the patch).**
  - R is a near-binary key: 0 or about 245, with a ±3 px soft edge.
  - G is 0 in the ground and a maze in the fill (5–95 % range 44–223, median 142). Inside the fronds it is a ramp,
    0 at the edges (red/magenta) up to about 190 in the cores (yellow). It dips to about 35 on both sides of
    every key edge, which draws dark outlines.
  - B outside the fronds is a scanline saw: period 13.86 lines, slow rise over about 10 lines, fall over 3–4,
    alternate cycles brighter and darker. The phase is the same in the ground and the fill. Inside the fronds
    B is smooth, 60–180.
- **The boundary.** x_b = 856 − G(|dy|). Values of G (|dy| in brackets): slope 2.0 at the apex, 197 (114.5),
  328.5 (214.5), 422 (314.5), 471 (414.5), peak 488.6 (489.5). Fitted as a 6th-order polynomial, rms 0.84 px.
- **Arms.** In (x, G) space every arm is a straight line x = c − s·G, with depth s = 1 + 3.9e-4·(c − 866)
  (77 cell-edge lines, rms 0.046). So c = (x + G(1 − 866k)) / (1 − kG) is linear in x on each line: one line ramp
  whose slope and start change per line.
  - Arm centres: c = 869, 977, 1089, 1205, 1347, 1495, 1641, 1806, 2015, 2210. The spacing grows from 108 to
    200 px (quadratic fit rms 7.5 px).
  - Tips on the centre line: x = 860, 1000, 1235, 1450, 1652, 1833.
- **Stems.** Vertical, period 206 px, centres 90 + 206·j. About 50 px wide in the middle band, 100–140 px flames at
  the top and bottom, 20–35 px at |dy| 300–400.
- **The dart is the same field.** The blue ground is a window on the ramp, 282 < c < 856.
  - x < 82 is a line-start hold: the field is frozen at x = 82, which draws the horizontal bars.
  - Its flat top and bottom (|dy| 144.5) are where c(82, G) reaches 282.
- **Key fit.** No analytic law fits the R key well: a sum of triangles reaches 85 %, a smooth union of arm and stem
  keys 82 %. A signed-distance table over (arm phase, stem phase, 8 height bands) at 16×16×8 matches 86.3 % of
  pixels (64×64×16 nearest-neighbour: 89.5 %). The remaining errors are few-px edge offsets.
- **Maze.** A broad spectrum (18–67 px periods), mostly horizontal stripes tilted along the arms. Mean change
  over 8 px: 32 along x, 47 along y.
- **Fronds' G and B.** A ramp across each stem and arm: on the stems G peaks at 109 left of centre and falls to
  49 at the right edge; across an arm B runs 187 → 77. Thin contour lines run along the arms.
- **Fit scores.** Float proto at Bend 0: 4×4 region means max 13.8 / mean 3.6 of 255; local luma contrast 12.8
  against the photo's 13.0; maze change over 8 px 36 along x / 53 along y. Ten proto rounds are logged in
  `notes.md`.

## What it does
1. **Ramp.** One line ramp per line: c = c0 + dc·x′, with x′ = max(x, 82). c0 and dc come from G(|dy|) × Arrow
   and the fitted depth law, and are scaled by Fronds.
2. **Two phases.**
   - Arm phase θ = T(c): a 512-entry table, knots every 16 px, interpolated in 1/16 steps. It starts at the
     stream phase.
   - Stem phase ψ = x′/206 × Stems.
3. **R (key).** A level L(θ, ψ) is read from the fitted table, blended between the height bands once per line and
   bilinear per pixel. Flesh is added, and the result is maxed with the window rims (the pink outer arm and the
   dart edge). The key edge is drawn in quarter steps.
4. **G.**
   - In the cells: a maze, = profile(vertical oscillator (46-line period) + Maze·cos θ + 2·cos(ψ + q(y)) + a
     source term), times the outline weight (dark from 0 to 10 px off the edge).
   - In the fronds: 2 × the table mean − 112, plus arm contour lines (weighted toward the arms), times the rim
     weight.
5. **B.** The scanline saw outside the fronds (it rolls slowly); 64 + 9 × the table's B level inside.
6. **Ground.** Outside the window (282 < c < 856) the output is R 3, G 0, B the saw.
7. **Output.** RGB → YUV in the hardware convention, then the output conditioning and the blanking gate.

## Controls
Glides: K1, K2, K4, K5, K6 and P12 move 1/8 of the gap per field (deadband 4).

| Control | Photo element it drives |
|---|---|
| **P12 Arrow** | The chevron depth (the G law × 0.25 … 2). 0 % = a shallow bow, with the stems near-vertical flames; 43 % = the photo; 100 % = deep converging arrowheads, and the dart stretches with them. |
| K1 Fronds | How many nested arms: arm density 0.5× … 2× (the ramp is scaled); centre = the photo. |
| K2 Stems | Stem spacing: frequency 0.5× … 2× (period 412 … 103 px); centre = the photo. |
| K3 Speed | Bipolar, still at the centre ±8, square law. The arms stream outward (default 600 ≈ 8–15 px/s) or inward; max ≈ 5–10 px per field. The maze drift and the scanline roll keep running at any setting. |
| K4 Flesh | Frond thickness: key level −8.5 … +11 px, from a thin wireframe to fat flames; centre = the photo. |
| K5 Maze | Maze curl: phase modulation by the arm phase, 0 … 3.4 cycles, from straight scan stripes to a tight curly maze. 27 % (0.92 cycle) = the photo. |
| K6 Bend | How far the source bends the arms, the boundary, the dart and the maze: 0 … ±250 px at full-scale luma. Default 25 % = ±63 px. |
| S7 Mirror | Photo (mirrored ">") / Sweep (the top half bends the other way: a one-way sweep). |
| S8 Palette | Pink (the photo) / Green (Cr negated: green fronds on a magenta maze; the blue ground stays). |
| S9 Dart | The small arrowhead at the left: On / Off. |
| S10 Stems | On / Off. Off holds the stem phase between stems: pure nested chevrons. |
| S11 Source | Video (default) / Osc. |

## Video
- **Video.** The camera luma, (Y − 502)/4 as a signed 8-bit value, is tapped straight from `data_in`. It is
  multiplied by Bend × Fronds and added to the ramp c, so the arms, the boundary and the dart bend around the
  picture. 1/256 of the same product goes into the maze phase. At the default Bend the photo's composition still
  reads (GHDL render fed the selfie collage in `cmp_final.png`). The video never reaches the output directly.
- **Osc.** A diagonal triangle (one cycle per 1280 virtual px across and 864 lines down, drifting about
  0.24 cycle/s) stands in for the camera on the same path. Bend stays live.

## Engine
- **Per field** (vblank zone = no avid for 1024 clocks):
  - K3 (sign and magnitude) and the switches are latched.
  - Field phases: the arm stream (± speed), the maze drift (≈ 11 lines/s), the scanline roll (≈ 3 lines/s) and
    the Osc drift.
  - Glides run in a shared datapath, four clocks per channel.
  - The knob curves, plus products on one serial 18 × 16 multiplier: speed m3²·13, stem step f2·psk, stem start
    f2·12089, Bend k6·f1, and the four window thresholds × f1.
- **Per line** (kicked at the source avid rise of line n; computes line n + 1; line 0 runs in the vblank zone):
  - g0 = G(|dy|) from the table (Q.1 px); the height band j and fraction fg (7 bands over 492 px of G);
    g′ = g0 × Arrow.
  - m = 2^35 / (2^20 ∓ 409·g′/2) on a serial divider; c0 = (164 ± 0.662·g′)·m·f1 and dc = m·sx2·f1 (Q.12 per
    pixel).
  - The scanline saw and maze modulator values.
  - The slice builder: for 256 entries it reads bands j and j + 1 of the level table, blends them by fg/16, and
    writes the next line's half of four parity banks (2 × 64 × 16 each). The pixel path then gets all four
    bilinear corners in one clock.
- **Pixel path (32 stages):**
  - Ramp and source: the virtual x with the line-start hold; the ramp DDA (Q.12); the Osc triangle or video tap
    → the Bend product → the bent ramp; the window and rim flags.
  - Phases: the T table (512 × 24) minus the stream phase. The stem-phase DDA (Q.16) steps 3/8 faster over
    virtual x 200–280, where the dart's stem sits.
  - Lookups: bank, cosine and line-texture addresses; bilinear L / G-mean / B-level (θ lerp, then ψ lerp); the
    maze phase → the maze profile table.
  - Key: L + Flesh (thinned inside the dart) → max with the rims → the key in quarter steps, plus the outline and
    rim weights.
  - Colour: fill green, frond green with the contour lines, frond blue; compose in quarters.
  - RGB → YUV (Q.6, luma pre-boosted 16/13; R from a per-quarter table, G and B in signed-digit form); the
    Palette switch.
  - Output: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums over a block-RAM ring), luma [1 2 1], luma × 13/16
    (white ≈ 777), and the blanking gate on the generated avid.
- **Memories (24 EBR):**

  | Block | EBR |
  |---|---|
  | level table 2048 × 16 | 8 |
  | arm-phase table 512 × 24 | 3 |
  | G-curve / saw / maze-modulator table 1024 × 10 | 3 |
  | two cosine copies, maze profile and line texture, 256 × 8 each | 4 |
  | slice banks 4 × 128 × 16 | 4 |
  | sync ring 256 × 4 and chroma ring 256 × 16 | 2 |

  The banks and both rings carry `no_rw_check`; standalone yosys reports 0 `emulate_transparency`.
- **Contracts:**
  - The heirloom v0.2.0 shell: virtual 1080-line coordinates, vblank zone, measured width, parity-locked
    generated avid.
  - One 32-clock latency in every mode, with the syncs through the block-RAM ring.
  - The HD HDMI bar fix before the gate.
  - Colours in the hardware convention (U = Cr, V = Cb).
- **Fit history.**
  - The first RTL (a direct port of the integer model) came to about 6.05 k LUT standalone, roughly 9 k LC.
  - Standalone yosys ablations, then fixes: Fronds folded into the per-line ramp; the per-field products moved
    onto the serial multiplier; the sync pipe moved into a block-RAM ring (latency 36 → 32); R pre-multiplied per
    key quarter. Result: about 4.35 k LUT.
  - Then: one Osc triangle instead of two, hard rim flags instead of rim distances, a 4-bit line texture, the
    chroma box delay in a block-RAM ring, and a coarser bend-into-maze term. Result: 3998 LUT / about 2.6 k FF
    standalone, 7133–7162 LC placed.
- **Verification** (GHDL 720p60, decimation 1):
  - Against the RTL-faithful `imodel.py` at default knobs: the key agrees on 99.94 % of pixels, and 97.7 % of
    pixels are within ±12/255 (mean difference 1.31).
  - Motion: a capture three frames later matches the model advanced to field 6 (key 99.89 %) and differs from the
    earlier capture.
  - Video mode checked with the selfie collage.
  - The default renders use a sim-only copy whose K5 glide powers up at the default (see Status).

## Status
- **v0.5.0 built 2026-10-27, NOT flashed.** **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh fern`). **Build seeds: HD HDMI = seed 2** (seed 1
  missed); the other five are seed 1.
- Post-route Fmax (last line):

  | Config | Fmax (MHz) |
  |---|---|
  | HD Analog | 79.67 |
  | HD HDMI | 74.38 |
  | HD Dual | 74.84 |
  | SD Analog | 77.94 |
  | SD HDMI | 73.39 |
  | SD Dual | 71.20 |

- 7133–7162 / 7680 LC (92.9–93.3 %), 24/32 EBR. Every bitstream passes iceunpack.
  `out/rev_b/ron/heirloom/fern.vmprog`, 493,031 B.
- **HD margins are thin** (HD HDMI +0.13 MHz, HD Dual +0.59). If a rebuild regresses, try other seeds before
  restructuring.
- The K5 glide powers up at 512 and settles to the TOML default (280) within about a second after load.
- Presets (placeholders): Photo, Wireframe, Deep Arrow, Green Fern, Sweep, Chevrons.

## Deviations from the photo, and why
- **The key** is a fitted 16 × 16 × 8 level table (86 % pixel agreement); no analytic law did better than 85 %.
  Cells and crossings sit where the photo's do on average, but individual edges are a few px off. The arms are
  slightly lumpier and the stems slightly fatter than the photo's glossy continuous arms, and some long arm
  segments in the top right break up.
- **Frond colour** comes from a mean-colour table plus contour lines. The arms read a little redder than the
  photo's pink/magenta highlights.
- **The maze** matches the photo's statistics (level histogram, spectrum, horizontal-vs-vertical balance), not
  its exact pattern. Its dark troughs are thinner navy lines where the photo has broader blue ones.
- **Default Bend (25 %)** in Osc mode wiggles the boundary and the dart by up to ±63 px compared with the static
  photo; Bend 0 reproduces the fit. The RTL Osc default therefore scores 4×4 max 23.0 / mean 5.0, against the
  proto's 13.8 / 3.6 at Bend 0.
- **Brightness:** drawn whites are ≈ 777 because the HD HDMI bar fix scales luma by 13/16, so the picture reads
  darker than the photo.
- **SD (4:3)** shows the left-aligned virtual x 0–1440, which keeps the dart and the line-start hold but cuts the
  right quarter. 720p and 1080 show the same picture.
- **Interlace:** no bottom-field half-line offset (field_n is tied high on hardware).
