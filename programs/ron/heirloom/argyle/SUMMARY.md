# Argyle — Heirloom #8, rebuilt from the photo

**v0.6.0 (2026-10-29, phase 2): K1 Bloom and K2 Scatter size every diamond individually; built, NOT
flashed.** v0.5.0 (2026-10-21) was the rebuild from `photos/WIN_20230207_23_50_54_Pro.jpg`
after Ron judged v0.1.0 too abstract. It now redraws the photo itself: a lattice of nested diamonds
with magenta lens-shaped cores and fine pink/white, lavender, blue and cyan/green rings, green and cyan
X junctions, and big and small cores alternating along each row. Work files are in
`.heirloom_work/rb_argyle/` (`notes.txt`, `proto.py`, `imodel.py`, `cmp_final.png`).

## Reading of the photo (measured)
- **One repeating tile.** The frame (beyond the 60-px sync edge) is one tile repeated in both
  directions: 524.8 px wide (two diamonds) by 261.0 lines, with a 0.00095 line/px tilt. Folding the
  frame onto it leaves 6.2 RGB rms. That makes 7.32 diamonds across and 4.14 rows.
- **Lattice.**
  - A rows sit at y = 147 + 261k. Their cores alternate: big at x = 227 + 525k (64x57 px) and small
    at x = 481 + 525k (40x26).
  - B rows sit at y = 276 + 261k, with eye-shaped cores of 60x28. A B row lies 128.8 lines below its
    A row, so it is not centred (49 %).
- **The field is a sum, s = G(x) + H(y).**
  - The gradient-ratio test dC/dx : dC/dy = G'(x) : H'(y) holds with a log MAD of 0.11.
  - G and H were fitted from 143 luma ridge contours, each ring a level line of s, to 0.75 px rms.
  - **G (horizontal oscillator).** A rounded triangle, which gives the lens-shaped cores.
    - Extremes: A-big +0.91, B −0.91, A-small +0.74, on a scale where H spans ±1.
    - Harmonics: cell fundamental 0.77; half-frequency 0.098, which is the big/small alternation;
      then 0.031, 0.04 and 0.047.
  - **H (vertical oscillator).** A triangle with sharp extremes; its slope is 1.7x steeper toward
    the B row.
- **Rings.**
  - Each half cell has 7 rings plus the junction ridge, at |s| = 0.24, 0.44, 0.64, 0.85, 1.05, 1.26
    and 1.49. The spacing is about 0.2 and is symmetric on the A and B sides.
  - Below |s| = 0.15 is the dark diagonal boundary; above about 1.6 is the flat magenta core.
- **Colour.**
  - Red and blue are smooth functions of the field read 13–18 px to the right: the capture's
    composite chroma offset. With one shared 15 px offset and 32x32 tables over (shifted G, H), red
    fits to 7.5 rms and blue to 6.9.
  - Green carries the rings: green = base + amp × RING(s), 13.4 rms. RING is the measured 512-entry
    ring waveform; a strictly periodic one fits much worse (22.7 rms).
  - Junctions: a dark-centred green X between vertical neighbours, at (G max, H min); a cyan X
    between horizontal ones, at (G min, H max).
- **Mechanism.** A horizontal and a vertical oscillator are summed into a colourizer. Green folds
  the sum into rings. Red and blue are shaped levels of the same oscillators, shifted by the
  capture's chroma delay.

## What it does
- **Per field, G' table.** One table of G' = (gain × GE + gain × pairs × GO) >> 8 over 1024 phases of
  the two-diamond period. Diamond mode uses a triangle for GE.
- **Per line.**
  - H comes from 256 nodes, linearly interpolated; then H' = gain × H.
  - H' selects a row pair of the 32x32 colour table. The line engine interpolates those rows (4-bit
    fraction) into a 64-entry line table: 32 G nodes plus midpoints, holding red, blue, base and amp.
- **Per pixel.**
  - G' is read at the ring phase p and at the colour phase p + Fringe, plus the source bend.
  - s = G' + H' indexes the ring ROM; the shifted G' indexes the line table.
  - green = base + amp × ring / 64, clipped. Warm swaps red and blue.
  - RGB is converted to 10-bit YUV (luma pre-boosted 16/13), then the output conditioning and the
    blanking gate.

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Bloom | **v0.6:** every diamond swells or shrinks in its own cell by its distance from the screen centre; bipolar, the photo at the centre (deadband ±8). + = the centre diamonds swell into flat magenta (k up to 2.5) while the outer ones shrink to ringed pips (k down to 0.38); − = a tunnel, small at the centre and swollen at the edges. It is screen-anchored, so with Drift each diamond swells as it passes the centre |
| K2 Scatter | **v0.6:** every diamond its own random size, 0 % = the photo, 100 % = flat-magenta slabs next to ringed pips. The sizes ride with the lattice and slowly breathe, each diamond on its own phase, at a rate set by Drift's speed (about 4 s a cycle at the default Drift; still when Drift is centred) |
| K3 Pairs | the big/small core alternation (half-frequency part of the horizontal oscillator), 0–400 % (100 % = photo) |
| K4 Drift | the lattice slides along its diagonal; bipolar, still at centre ±8; default 0.5 px/field up-left, ~8 s per diamond |
| K5 Bend | how far camera luma / Osc bends the rings and colours (default 39 %) |
| K6 Fringe | colour offset against the rings, 0–64 px (15 = photo's chroma offset) |
| S7 Saw | vertical oscillator becomes a sawtooth: diamonds turn into stacked chevrons |
| S8 Hard | hard-clipped rings |
| S9 Shape | Lens (photo) / Diamond: straight-sided cores |
| S10 Palette | Neon (photo) / Warm: red and blue swapped, an amber/orange lattice |
| S11 Source | Video (default) / Osc |
| P12 Cores | oscillator gain 0.55 → 1.0 → 2.2: 0 % = no cores, soft cool rings; 50 % = the photo; 100 % = cores and junctions clip into a flat magenta/green/blue argyle |

Glides: K1, K2, K3, K5 and P12 move 1/8 of the gap per field (deadband 4).

## Phase 2 (v0.6.0)
Ron's note (2026-10-29, on v0.5.0): he likes almost everything, but K1 Columns / K2 Rows were only a whole-screen
scale; he wants them to change the individual diamonds in some pattern, or scale them individually at different
sizes.
- **Removed:** K1 Columns and K2 Rows (the X / Y zoom). The lattice is fixed at the photo's 7.32 × 4.14.
- **Elements and their drivers (v0.6).** Each diamond (A and B type): its size (Bloom, Scatter, Cores), core
  shape (Shape, Pairs), ring hardness (Hard). The lattice: Drift, Saw. Colour: Palette, Fringe. Camera: Bend.
- **Mechanism: a per-diamond swell.** Each diamond keeps its cell; its field is scaled about the cell's dark
  boundary. With e = max(|s| − T, 0) and the diamond's k, the pixel path adds Δ = sgn(s) · e · (k − 1) to the
  ring index *and* to the colour-table column. The colour follows because red is almost purely a function of
  |s| (the fitted table is constant along its anti-diagonals), and blue's dependence on G − H matters only at
  the junctions, where e = 0. So a swollen diamond grows a bigger flat magenta core with its rings squeezed
  towards the boundary, and a shrunken one becomes a small ringed pip.
- **Which diamond.** The type is the sign of s (A ≥ 0). The column comes from the ring phase, which now carries
  2 extra bits (8 columns before the pattern repeats, wider than the 7.3 on screen): A column (h+1)/2, B column
  h/2 of the half-column h. The row comes from the line's row phase, which carries 3 extra bits (8 rows).
- **No seams.** Two same-type diamonds touch through a saddle (the cyan and green junction X's), where |s| is
  small but not zero (up to 0.29 beside a small core, more with P12, Pairs or Bend). T = 0.125 plus a window
  that rises to 0.75 within 48/256 of a cell edge, in X (per pixel) and Y (per line), so e = 0 wherever the
  diamond's k could change.
- **The size tables** (2 × 64 × 8 EBR, A and B, B stored negated so the product carries the sign of s) are
  rebuilt every field by a small sequential builder in the vblank zone (128 diamonds × 16 clocks, from zone
  clock 64; it fits the image tester's fake 8-line vblank):
  - Bloom: z = clamp(1 − (|dx| + 2|dy|) / 64, −1, 1), with dx / dy the diamond's offset from the screen centre
    in Q.4 half-cells (ones'-complement magnitude). z = 0 two columns / one row out.
  - Scatter: tri(rnd + τ). rnd comes from a 16-bit LFSR restarted every field, so every diamond keeps its value
    (neighbour correlations ≤ 0.19 over the 8 × 8 tile). τ advances by Drift's speed.
  - e = clamp((|d1| · ±z + d2 · tri) / 256, −1, 1) on one 8-step shift-add MAC. k − 1 = 3/8 e for e > 0,
    5/32 e for e < 0 (Q.6).
- **Pixel path:** three new stages (6a: e to 8 bits and the k select; 6b: e × (k − 1) split 4 / 4; 6c: Δ),
  with the old ring / colour address stage absorbing the final add. Latency 24 → 27 in every mode.
- **Flat swollen cores.** When the swell drives the ring index off the table (Δ ≠ 0), green is forced to 0.
  Otherwise the colour table's edge column, whose fitted base / amp wander row by row, striped the swollen
  cores pink. The flag needs Δ ≠ 0, so v0.5's own P12 = 100 % look is unchanged.
- **The photo stays.** At K1 centre / K2 0 every table entry is 0, so Δ = 0. The integer model's default frame
  is bit-identical to v0.5's (`imodel.py` vs `imodel_v050.py`, max diff 0 in Y, U and V).
- **Fit.**
  - The first cut (a 13-stage pipelined builder with a parallel 10 × 10 multiplier and a 256-entry cross table,
    plus full-width swell stages) overflowed to 8072 LC (105 %).
  - Standalone-yosys ablations priced its builder at 377 LUT and its geometry alone at 166.
  - The rewrite brought it to 7175 LC (93.4 %): a sequential builder with Q.4 geometry, LFSR randoms instead of
    a hash, 8-bit e, 7-bit thresholds, and 8-bit centre adds.
- **Control sheet** (integer model, 720p): `.heirloom_work/rb_argyle/sheet_v060.png`. The prototypes are
  `p2_explore.py` (swell vs lattice warps) and `p2_swell.py`.
  - The lattice-warp alternative (individual column widths / row heights, or a screen lens) was also prototyped.
  - It was not taken: it changes the grid geometry, which reads close to the zoom Ron asked to replace.

## Video
- **How the camera enters.** Camera luma is added to the horizontal oscillator, in both the ring
  phase and the colour phase. The rings and their colours bend around the picture, and the lattice
  still reads at the default Bend.
- **Osc mode.** Two slow drifting triangles stand in for the camera: about 0.9 cycle across, about
  1 cycle down, amplitude 1.25 tx + 0.75 ty. They feed the same path, and Bend stays live in both
  modes.
- **What it never does.** The video tap is luma into geometry only; it never reaches the output.

## Engine
- **Per field (vblank zone).**
  - Glides, then one serial shift-add multiplier over 10 slots: phase increment, phase per virtual
    px, line-start phase, fringe phase, row step, drift squared, drift x/y, gain × pairs.
  - The gain curve is shift-add, one add per clock.
  - The G' builder runs from zone clock 400, two clocks per phase on one split multiplier, writing
    two identical 1024x13 RAMs (no_rw_check).
- **Per line.** Kicked at the source avid rise, it computes the NEXT line in about 390 clocks, which
  fits SD's 858: H interpolation and gain on a serial multiplier, then the 32-node line table into
  the other bank of a ping-pong RAM (no_rw_check). Line 0 is computed in the vblank zone.
- **Pixel path.**
  - The G' RAM pair, bend, s and table column, ring ROM and line RAM (each with a fabric copy),
    amp × ring, green, the 2-term RGB→YUV matrix, clamp.
  - Then the conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to 13/16. Then the gate
    on the generated avid.
  - Latency is 27 clocks in every mode (v0.5: 24; v0.6 adds the three swell stages).
- **Shell.** Heirloom v0.2.0: virtual 1080-line coordinates, generated avid, vblank zone, measured
  width.
- **Memory.** 25 EBR (v0.5: 23):

  | Block | EBR |
  |---|---|
  | GE 512x12 | 2 |
  | GO 256x12 | 1 |
  | H 256x12 | 1 |
  | ring 512x8 | 1 |
  | colour table 1024x32 | 8 |
  | G' RAMs, 2 × 1024x13 | 8 |
  | line RAM 256x32 | 2 |
  | size tables, 2 × 64x8 (v0.6) | 2 |

- **Fit history.** The first cut, with per-pixel gain multipliers and a 128-entry quarter-step line
  table, came to 8126 LC (105 %). Ablations put the per-pixel multipliers at 1595 LUT, the 4-lane
  table builder at 988 and the colour matrix at 471. Moving to per-field G' RAMs, a 64-entry
  midpoint table and 2-term constants brought it to 6337 LC.

## Status
- **v0.6.0** built 2026-10-29 with router2. **HD Analog and HD Dual closed on seed 3** (seeds 1 and 2 missed;
  the builder's own retry found it); the SD configs are seed 1.
  - **HD HDMI is a hand-installed seed 9** (vmprog repacked; a plain rebuild will NOT reproduce it).
    - The builder's seed 1 sat at the edge of the pin-skew window: HSYNC +1.09 ns, closest data bit −0.62.
    - Seeds 2–17 were screened with `tools/hdmi_pin_skew.py`. Seed 9 is centred: HSYNC +1.58, data
      ≤ −0.89, 83.98 MHz.
    - Shipped HD Analog s3: HDMI HSYNC −1.61, data ≤ −0.76. HD Dual s3: −1.60, ≤ −0.85.
    - To re-route: regenerate the json with make into `.heirloom_work/rb_argyle/skew/<config>`, then
      `.heirloom_work/rb_argyle/skew/route1.sh hd_hdmi 9`. All three shipped HD seeds re-route byte-exact.
      Results are in `screen_hdmi.txt` / `ship.txt`.
    - The seed-1 vmprog is kept as `skew/argyle_v060_hdhdmi_s1.vmprog`.
  - Fmax, post-route: HD Analog 77.62, HD HDMI 83.98 (s9), HD Dual 78.38, SD Analog 80.49, SD HDMI 63.02,
    SD Dual 87.15 MHz (SD needs 27).
  - 7162–7186/7680 LC (93.6 %), 25/32 EBR. iceunpack OK on all six.
  - Verified: GHDL analysis / synthesis front-end, and the integer model (default frame bit-identical to
    v0.5's model; control sheet `sheet_v060.png`). The RTL was not simulated. **NOT flashed.**
  - Presets are placeholders: Photo, Flat Argyle, Soft Rings, Chevrons (now with Bloom +60), Amber Quilt,
    Bloom, Tunnel, Scatter.
- v0.5.0 built 2026-10-21 with router2. **HD Dual needed seed 2** (seed 1 missed timing); all other
  configs are seed 1.
- Fmax, post-route: HD Analog 79.69, HD HDMI 80.36, HD Dual 80.91, SD Analog 79.50, SD HDMI 83.94,
  SD Dual 73.90 MHz (SD needs 27).
- 6334–6341/7680 LC (82.5 %), 23/32 EBR. iceunpack OK.
- RTL vs integer model: 7.4 mean absolute difference at 720p (drift phase and edge pixels); motion
  verified. **NOT flashed**; presets are placeholders.

## Deviations from the photo, and why
- **Left edge.** The sync tile (first ~60 px: magenta slivers, teal blobs where the line-start
  oscillator restarts) is not reproduced; it would not fit as a simple restart, so the lattice runs to
  the edge. The 3-px black border and the 0.1 % tilt are dropped.
- **Colour model limit.** The green junction X is a little softer and the outer rings have a little
  less contrast. One smooth colour table plus one ring waveform reaches 13 rms on green.
- **Brightness.** The required HD HDMI conditioning scales luma to 13/16 (whites about 777), so the
  near-white ring peaks are compressed and the picture is a little darker than the capture. The
  2-term colour constants add at most about 2 % chroma error.
- **Osc plateaus.** In Osc mode, where the triangle's slope cancels the oscillator's rounded extreme,
  a junction flattens into a short plateau. Video does this organically.
- **Interlace.** Interlaced modes draw the same picture in both fields, at half vertical resolution.
- **SD.** On 4:3 SD the diamonds keep their shape, so 5.5 fit across instead of 7.3.
  K1 Bloom is measured in diamonds from the screen centre, so SD shows the same bloom with the sides
  cropped, like the rest of the picture.
