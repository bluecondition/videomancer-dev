# Fern — Heirloom #46

**v0.6.2 (2026-11-02): phase 2 FINAL, merged into production; built, not yet flashed.** Ron chose this version, the v0.6 diag ("I like the overall look of the diag version better, the movement, the colors"), and added production v0.5.2's wider ranges: K2 Lean x2, K4 Flesh x2 and P12 Arrow 1/16x .. 3x (see [Phase 2](#phase-2-v060)). Production v0.5.2 (dbf3c18) was the interim build those ranges came from.
v0.5.0 (2026-10-27): rebuilt from the photo, built, NOT flashed. Source photo:
`photos/WIN_20230713_00_17_11_Pro.jpg` (1920×1080 capture, 2023-07-13). This replaces v0.1.0 (nested parabola
arcs over a ripple), which Ron judged too abstract. Presets are hand-written placeholders. Working files:
`.heirloom_work/rb_fern/` (`notes.md`, `proto_final.py`, `imodel.py`, `cmp_final.png`, `sheet.png`).

## Phase 2 (v0.6.0)
Ron gave no fern-specific notes; the map below is Claude's recommendation (Ron, 2026-11-01: "do your recommended
changes"), built as a diag copy (`Fern Diag`) for preview. The aim, from PHASE2.md: per-element parameters instead of
global grid scales, bolder motion, a switch that is a real look-flip, the photo kept at the defaults.

| Was (v0.5) | Now (v0.6.0) | Why |
|---|---|---|
| K1 Fronds: arm density 0.5–2× (a uniform scale of the arm ramp) | **K1 Fan**: the arm depth law k (s = 1 + k (c − 866)), k = 1.5 (K1 − 512) / 2^20 = −7.3e-4 … +7.3e-4; the photo's 3.9e-4 at the default 785, 0 (parallel copies of the boundary) at 512 | Each arm gets its own depth from its place across the screen: the outer arms open into bows and bend back (left), or sharpen into deep nested V's (right). The boundary stays put (c = 866 is the pivot); the dart grows / shrinks with it. |
| K2 Stems: stem frequency 0.5–2× (a uniform scale of the stem lattice) | **K2 Lean**: the stems tilt with \|dy\|, up to ±1.5 px per line, upright at the centre (the photo, deadband ±8) | Left: the stems lean into "<" against the arms (a diamond lattice); right: ">" with the arms (nested chevron bands). Under Sweep the lean follows the signed dy (slanted lattice). |
| K3 Speed: arm stream only | **K3 Speed**: the stream, plus a wave (512 lines, running out from the axis) that ripples the arms (± up to 0.24 cycle) and sways the stems with them (± up to 0.48 stem period) | The depth rises with speed² (≈ 1 px on the arms and 2 px on the stems at the default 600, so the default stays the photo; the fronds writhe like weed in a current at the ends). The wave runs outward with the stream (inward on the left half of the knob). |
| P12 Arrow: chevron depth; flame bands fixed by height | **P12 Arrow**: same depth law; the flame bands now follow the scaled chevron | 0 % = a shallow bow of thin stems, no flames; 43 % = the photo; 100 % = deep arrowheads burning most of the way to the axis. |
| S9 Dart: On / Off (a small element) | **S9 Echo**: Single / Echo | Echo cuts blue scanline-ground bands into the fronds wherever c mod 512 is in [384, 512), from c = 1280 on, with a 16 c pink rim on each side: nested copies of the boundary that follow Fan, Lean and Arrow (two on screen at the defaults). The dart is always on. |

Unchanged: K4 Flesh, K5 Maze, K6 Bend, S7 Mirror, S8 Palette, S10 Stems, S11 Source. The K1 and K5 glides now power
up at their TOML defaults (785, 280).

Fan × Arrow corner: at the right end of both, k g′ reached 0.74 and the top and bottom rows packed into a moiré; the
divider denominator is floored at 2^19 + 4096 (k g′ ≤ 0.496), which also keeps the reciprocal in 16 bits. Rows
below the floor are exact, so K1's whole travel is exact at the photo's Arrow.

Preview sheet (integer model, 720p): `.heirloom_work/rb_fern/diag/sheet_v060.png`. Model: `diag/dmodel.py`.

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
   and the arm depth law k (Fan).
2. **Two phases.**
   - Arm phase θ = T(c): a 512-entry table, knots every 16 px, interpolated in 1/16 steps, minus a per-line
     offset (the stream phase + the Speed ripple).
   - Stem phase ψ = x′/206, started per line at an offset (Lean × |dy| + the Speed sway).
3. **R (key).** A level L(θ, ψ) is read from the fitted table, blended between the height bands once per line and
   bilinear per pixel. Flesh is added, and the result is maxed with the window rims (the pink outer arm and the
   dart edge). The key edge is drawn in quarter steps.
4. **G.**
   - In the cells: a maze, = profile(vertical oscillator (46-line period) + Maze·cos θ + 2·cos(ψ + q(y)) + a
     source term), times the outline weight (dark from 0 to 10 px off the edge).
   - In the fronds: 2 × the table mean − 112, plus arm contour lines (weighted toward the arms), times the rim
     weight.
5. **B.** The scanline saw outside the fronds (it rolls slowly); 64 + 9 × the table's B level inside.
6. **Ground.** Outside the window (282 < c < 856) the output is R 3, G 0, B the saw. Echo adds ground bands where
   c mod 512 is in [384, 512), from c = 1280 on.
7. **Output.** RGB → YUV in the hardware convention, then the output conditioning and the blanking gate.

## Controls
Glides: K1, K2, K4, K5, K6 and P12 move 1/8 of the gap per field (deadband 4).

| Control | Photo element it drives |
|---|---|
| **P12 Arrow** | The chevron depth (the G law × 1/16 … 3; v0.6.2), and the flame bands follow it. Lower half in octaves: 0 % = near-straight vertical arms, 25 % = 1/4 (the old 0 %), 50 % = the photo. Upper half linear: 75 % = 2× (the old 100 %), 100 % = 3×, deep converging arrowheads with flames all the way to the axis, and the dart stretches with them. |
| K1 Fan | Each arm's depth by its place: k = −7.3e-4 (outer arms open into bows and bend back) … 0 (parallel copies of the boundary, 512) … 3.9e-4 (the photo, default 785) … 7.3e-4 (deep nested V's). The boundary stays put. |
| K2 Lean | The stems tilt with \|dy\| (signed dy under Sweep), up to ±3 px per line (v0.6.2, twice v0.6.1's): left "<" (diamond lattice) … upright (the photo, centre) … right ">" (nested chevron bands). |
| K3 Speed | Bipolar, still at the centre ±8, square law. The arms stream outward (default 600 ≈ 8–15 px/s) or inward; max ≈ 5–10 px per field. The depth of a wave running out from the axis (in, left of centre) rises with speed²: it ripples the arms (to ±0.24 cycle) and sways the stems with them (to ±0.48 stem period). The maze drift and the scanline roll keep running at any setting. |
| K4 Flesh | Frond thickness: key level −18.5 … +21 px (v0.6.2, twice v0.6.1's), from a bare wireframe to flames everywhere (the key clamps at ±32 px); centre = the photo. |
| K5 Maze | Maze curl: phase modulation by the arm phase, 0 … 3.4 cycles, from straight scan stripes to a tight curly maze. 27 % (0.92 cycle) = the photo. |
| K6 Bend | How far the source bends the arms, the boundary, the dart and the maze: 0 … ±250 px at full-scale luma. Default 25 % = ±63 px. |
| S7 Mirror | Photo (mirrored ">") / Sweep (the top half bends the other way: a one-way sweep). |
| S8 Palette | Pink (the photo) / Green (Cr negated: green fronds on a magenta maze; the blue ground stays). |
| S9 Echo | Single (the photo) / Echo: blue ground bands with pink rims where c mod 512 is in [384, 512), from c = 1280: nested copies of the boundary across the fronds. |
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
  - The Speed wave phase (±1.5 m3 per field).
  - The knob curves (Fan kq = 1.5 (K1 − 512), Lean |K2 − 512| − 8), plus products on one serial 18 × 16
    multiplier: speed m3²·13 and a1 = 1 − 866 k (Q.16). The ripple depth is a slice of m3².
- **Per line** (kicked at the source avid rise of line n; computes line n + 1; line 0 runs in the vblank zone):
  - g0 = G(|dy|) from the table (Q.1 px); g′ = g0 × Arrow; the height band j and fraction fg from g′ (7 bands
    over 492 px).
  - m = 2^35 / (2^20 ∓ |k|·2^20·g′/2) on a serial divider (denominator floored at 2^19 + 4096); c0 =
    (164 ± a1·g′)·m and dc = m·256·sx2 (Q.12 per pixel).
  - The scanline saw and maze modulator values, and a sine read (C_EG 896..1023) at the wave phase
    f_wph − 128 |dy|.
  - Lean |dy|·lean and ripple depth × sine → the next line's arm phase offset (stream + r) and stem start phase
    (53447 ± Lean − 128 r).
  - The slice builder: for 256 entries it reads bands j and j + 1 of the level table, blends them by fg/16, and
    writes the next line's half of four parity banks (2 × 64 × 16 each). The pixel path then gets all four
    bilinear corners in one clock.
- **Pixel path (32 stages):**
  - Ramp and source: the virtual x with the line-start hold; the ramp DDA (Q.12); the Osc triangle or video tap
    → the Bend product → the bent ramp; the window and rim flags (constant thresholds), and the Echo band
    and rims as bit tests on c (combined one stage later).
  - Phases: the T table (512 × 24) minus the per-line arm offset. The stem-phase DDA (Q.16) steps 3/8 faster over
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
- **Memories (25 EBR):**

  | Block | EBR |
  |---|---|
  | level table 2048 × 16 | 8 |
  | arm-phase table 512 × 24 | 3 |
  | G-curve / saw / maze-modulator / sine table 1024 × 10 | 3 |
  | two cosine copies, maze profile and line texture, 256 × 8 each | 4 |
  | slice banks 4 × 128 × 16 | 4 |
  | sync ring 256 × 4 and chroma ring 256 × 16 | 2 |
  | delay ring 256 × 16 (v0.6.0): six 8-clock pipeline delays on the chroma ring's pointer | 1 |

  The banks and all three rings carry `no_rw_check`; standalone yosys reports 0 `emulate_transparency`.
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
  - v0.6.0: the first phase-2 build came to 4329 LUT / 2691 FF (7550–7570 LC, 98 %; HD Analog closed only at seed 3,
    HD HDMI at seed 4 by 0.2 MHz). Ablations priced Echo at 28 LUT, the Speed waves 69, Lean 71, the Fan knob
    curve 46. Fixes, no feature lost: the sway rides the ripple's wave and product (one sine read, one product),
    the wave table addresses come straight from 9-bit differences, Fan ×1.5 instead of ×25/16, the stem start
    constant folded into the Lean add, the density floor tested on the top bits, Echo aligned to c = 512 n so it
    is bit tests, and six FF delay chains (bend-into-maze, line texture, arm weight, window, dart and rim flags,
    16 bits in all, each exactly 8 clocks) moved into one 256 × 16 block-RAM ring on the chroma ring's pointer.
    Result: 4134 LUT / 2519 FF standalone, 25 EBR.
- **Verification** (GHDL 720p60, decimation 1):
  - Against the RTL-faithful `imodel.py` at default knobs: the key agrees on 99.94 % of pixels, and 97.7 % of
    pixels are within ±12/255 (mean difference 1.31).
  - Motion: a capture three frames later matches the model advanced to field 6 (key 99.89 %) and differs from the
    earlier capture.
  - Video mode checked with the selfie collage.
  - The default renders use a sim-only copy whose K5 glide powers up at the default (see Status).
  - v0.6.0 (first phase-2 netlist, before the fit pass): one GHDL frame with Fan 950, Lean 250, Speed 900, Echo
    on, Osc source, against `diag/dmodel.py`: key 99.94 %, 97.2 % within ±12 (mean 2.10) at the harness's usual
    4 px x offset. It exercises the reordered line engine (g′ before the band, the Fan divider, the Lean and wave
    products) and the Echo window. The fit pass that followed (merged sway, bit-aligned Echo, the delay ring)
    was not re-simulated.

## Status
- **v0.6.2 (2026-11-02): FINAL phase-2 version, merged into production `programs/ron/heirloom/fern/`.** Changes on top of v0.6.1: K2 Lean x2 (to 3 px per line), K4 Flesh offset x2 (+-40 Q.1), and P12 Arrow 1/16x .. 1x in octaves then linear to 3x, with the photo at 50 % (initial 512). The presets were remapped to keep their looks (Lattice K2 282, Echo Bands 710, Sweep 402, Wireframe K4 316, Writhe 576, Deep Arrow P12 757), and the P12 glide powers up at 512.
  - **3x depth:** e_gp 12 bits, a 5-bit flame-band test, e_xa 13 bits, n_c0 from a 17-bit slice. The divider needs no change, because den is floored at 528384 (q <= 65027). Checked numerically over every |dy| at Arrow 1/16x / 1x / 3x x Fan 0 / 256 / 512 / 785 / 1023 x sx2 2-4 x Sweep; b_c >> 8 peaks at 194630 (< 2^18 with bend).
  - **Built 2026-11-02 as production `out/rev_b/ron/heirloom/fern.vmprog`, ROUTER1, NOT flashed.** Post-route Fmax:
    - HD Analog **s7 75.72** (hand re-route; the builder's s5 gave a thin 74.66)
    - HD HDMI s1 75.60
    - HD Dual **s10 75.82** (hand re-route; the builder's six seeds missed, best 72.09)
    - SD Analog s2 76.82, SD HDMI s1 77.07, SD Dual s1 73.51 MHz
    - 7204-7252 / 7680 LC (94 %), iceunpack OK x 6.
  - A plain `NEXTPNR_ROUTER="--router router1" ./build_heirloom.sh fern` lands the builder seeds again. To restore the shipped HD Analog / HD Dual, run `.heirloom_work/resync/reroute.sh fern <cfg> router1 "<seed>"`.
- **v0.6.0 diag built 2026-11-01, awaiting Ron's preview** (`Fern Diag`, program_id `com.czartroniks.fern_diag`,
  loads beside v0.5): `out/rev_b/ron/_diag/fern_diag_v0.6.0.vmprog`, 497,758 B. **ROUTER1**
  (`NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build fern`). 7185–7215 / 7680 LC (93.6–93.9 %), 25/32
  EBR; every bitstream passes iceunpack.

  | Config | Seed | Fmax (MHz) | HDMI HSYNC − clk / closest data (ns) |
  |---|---|---|---|
  | HD Analog | s12, hand-installed | 74.59 | +1.22 / −0.99 (analog encoder −0.87 / −1.18) |
  | HD HDMI | s42, hand-installed | 76.87 | −2.41 / −3.50 |
  | HD Dual | s17, hand-installed | 76.61 | −2.65 / −1.90 |
  | SD Analog | s1 (builder) | 74.93 | |
  | SD HDMI | s1 (builder) | 71.79 | |
  | SD Dual | s1 (builder) | 77.15 | |

  Pin-skew screen (`diag/skew/`, `pick.py`): 110 HD HDMI routes (router1 s1–96, router2 s1–24); none put HSYNC in
  the +1.0 … +2.2 ns window with data ≤ −0.6, and router1 put it negative on all but 6 (the best positive,
  s56 +0.32, is in the danger zone; router2 was 57–68 MHz). s42 has HSYNC far from zero and the widest data
  margin. If HD HDMI shows the thin red / blue line artefact, re-seed from `screen*.txt` before touching the
  RTL. The builder's own picks (HD Analog s2, HD HDMI s3 at −0.64, HD Dual s2 at data −0.34) are packed in
  `diag/skew/fern_diag_v060_builder.vmprog`.
- **v0.6.1 diag built 2026-11-01: colour fix, HW-CONFIRMED (Ron 11-01: colours "look correct"); the v0.6 controls still await his review.** `out/rev_b/ron/_diag/fern_diag_v0.6.1.vmprog`
  (498,654 B), ROUTER1 builder seeds. 7187–7235 LC (93.6–94.2 %), 25 EBR, iceunpack OK × 6.

  | Config | Seed | Fmax (MHz) |
  |---|---|---|
  | HD Analog | s3 | 77.56 |
  | HD HDMI | s2 | 80.33 |
  | HD Dual | s2 | 77.41 |
  | SD Analog | s1 | 71.84 |
  | SD HDMI | s1 | 70.05 |
  | SD Dual | s2 | 74.24 |

  - **The v0.6.0 symptom.** Ron's capture with S8 Pink showed a maroon ground (should be blue), an olive maze
    (green / cyan) and green fronds (pink).
  - **Diagnosis.** The ground's blue scanline saw was intact while R sat flat at ~136. A pure U/V swap doesn't
    match, but one constant chroma offset with mod-1024 wrap applied to the correct render does (Cr +352,
    Cb −240), and the offset differed in his S8 Green capture.
  - **Cause.** The output chroma filter is a running 8-sum with a block-RAM tap and no resync, so one wrong RAM
    sample leaves a permanent wrapped offset.
  - **Fix.** Chroma is neutral off the generated avid (s_gav_sr(22) at p24), and the sums snap to 4096 once
    8 blank samples are in. See memory running_sum_chroma_offset.
  - The earlier pin-skew A/B build `fern_diag_v0.6.0b.vmprog` (same v0.6.0 RTL, HD HDMI s56) is superseded.
- The v0.5 critical path (HD Analog seed 1) is the luma compose: the three-term Y sum and its clamp in one stage
  (stage 24 → s_y25). Moving the clamp to s_y25 would be free (same alignment) if HD margins need it.
- v0.5.0 built 2026-10-27, NOT flashed. **ROUTER1**
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
- (v0.5) The K5 glide powered up at 512 and settled to the TOML default (280) within about a second after load;
  fixed in v0.6.0.
- Presets (placeholders, v0.6.0): Photo, Wireframe, Deep Arrow, Lattice, Echo Bands, Bows, Writhe, Sweep.

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
