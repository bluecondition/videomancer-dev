# Terrace — a colour-bar staircase through two quantizers

**v0.6.2 (2026-11-19): phase 2 FINAL — Ron prefers the diag in every case, so it is merged into production** (`out/rev_b/ron/heirloom/terrace.vmprog`, repacked from the diag's own v0.6.2 bitstreams, not rebuilt; this exact build not flashed). It is the phase-2 round plus the K6 camera pass; status lines below that say "diag" or "production untouched" are history. The replaced v0.5.3 vmprog is in `out/rev_b/ron/_archive/heirloom_2026-11-18/replaced_production/`.

**v0.6.0 (2026-11-09, phase 2): diag preview built, awaiting Ron's preview** (`Terrace Diag`,
`out/rev_b/ron/_diag/terrace_diag_v0.6.0.vmprog`, copy in `out/rev_b/ron/heirloom/_diag/`; see
[Phase 2](#phase-2-v060)). Production v0.5.1 (`programs/ron/heirloom/terrace/`) is untouched.
**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Photo: `photos/WIN_20230305_23_38_39_Pro.jpg`.
It is a strict grid of flat blocks: 2.72 tiles of 6 columns by 25 rows. Down every column the colours run yellow,
magenta, cyan, white, light grey, grey, dark grey, black. Each column is the same sequence dropped one row, so the
colours step down to the right like stairs. At every tile seam a narrow fringe flashes the in-between colours.

## Phase 2 (v0.6.0)
No notes from Ron for terrace (free hand, 2026-11-09 batch). Every v0.5 control was already live and most were bold
(Bend, Seam, Palette, Steps, the four switches). What it lacked was per-element movement: the tiles all moved as one,
the rows were one straight ramp, and Tiles and Rows were linear zooms with the photo near one end. The round:
- gives the **tiles** their own motion: past the photo's speed, K4 slides each tile up or down at its own rate
  (the centre tile still, the outer tiles fastest, in opposite directions), and they glide home when the knob comes
  back;
- gives the **row ramp** a second shape: S9 Fold mirrors the top half onto the bottom, so the stairs become
  chevrons meeting at the centre line and drifting rows flow toward it or away from it;
- makes Tiles and Rows exponential, ×4 each side of the photo, with the photo at 50 %;
- doubles K4's top speed.

Every new motion is exactly zero at the default knob value. Phase-2 working files: `.heirloom_work/rb_terrace/p2/`.

| Was (v0.5) | Now (v0.6.0) | Full sweep, in screen terms (1080p) |
|---|---|---|
| K1 Tiles: 1 .. 8 linear (photo 2.72 at 25 %) | **K1 Tiles**: exponential 0.68 .. 2.72 (50 %, the photo, exact) .. 10.9; readout 0–100 % | tile width 2824 px (one tile wider than the screen) → 706 px → 176 px (a fine comb of staircases) |
| K2 Rows: 4 .. 64 linear (photo 25.1 at 35 %) | **K2 Rows**: exponential 6.3 .. 25.1 (50 %, exact) .. 100; readout 0–100 % | row height 172 lines (big blocks) → 43 → 10.8 lines (fine stripes) |
| K4 Speed: bipolar drift, square law, up to ±1.9 rows/s | **K4 Speed**: same curve up to the photo's speed and on to 256 (default bit-exact), steeper above: top **±3.8 rows/s**; **plus tile slide** past the photo's speed (both directions) | drift up to 164 lines/s (15 % of the height a second). Slide: each tile's row offset = (tile − centre tile) × a phase growing at (|K4| − 190)² / 16; at the ends neighbouring tiles part at 5.6 rows/s (240 lines/s), wrapping every 8 rows (one colour period) so the terraces fall in and out of line. Back in the still/photo zone the slide glides home in ~3 s |
| S9 Lean: the columns shear half a tile over the frame | **S9 Fold** (renamed; Lean removed): Off / On, the row ramp folds at line 540 | the top half mirrors the bottom about the centre line: every staircase becomes a chevron pointing at the centre (with K5 at 50 %, diamonds). With drift the rows stream into the centre line (Speed +) or out of it (Speed −) |

Unchanged: K3 Bend, K5 Seam, K6 Palette, S7 Climb, S8 Ramp, S10 Fill, S11 Source (Video / Osc, default Video),
P12 Steps.

**Presets** (preset 0 = the defaults): **Photo**; **Colour Bars** (P12 0); **Diamonds** (Seam 100 %, Fold);
**Sliding Tiles** (more tiles, K4 past the photo speed); **Big Blocks** (1.6 tiles, 10 rows, Neon, fewer steps);
**Fine Weave** (6.5 tiles, 68 rows, more bend, 15 steps); **Chevron Rush** (Fold, K4 full, Ocean);
**Camera Steps** (Fill, more bend). Dropped: Chevrons (→ Diamonds), Cascade (it needed Lean).

**Engine (changes).**
- **Knob curves.** Tiles and Rows use one piecewise-linear 2^(x/256) ((256 + x mod 256) << (x / 256)) into the
  serial multiplier: tiles = exp(k1) × 695 / 1024 (Q.8; 695 = the v0.5 default exactly), rows = exp(k2) × 6429 /
  1024 (Q.8; 6429 exactly). The multiplier's B operand widened 16 → 18 bits (18 steps a slot), and 21 slots instead
  of 20.
- **Speed.** The speed slot multiplies m by m + 2 (m − 256) when m ≥ 256 (×2 at the top; unchanged below).
- **Tile slide.** A per-field phase f_ts (rows Q.16, mod 8 rows) adds ±(m − 190)² / 16 per field (slot 17 squares
  it); with no slide it eases home by 1/16 per field (the short way round, at least 1). The beam flags the first
  pixel of every tile (the tile-phase carry); at pixel stage 8 a register loads −kc × step at the line start (slot
  21; kc = the centre tile, (tiles + 0.04) / 2) and adds the step at every tile start; stage 9 adds it to the row
  value (11 bits, Q.8). The seam fringe belongs to the tile it ends.
- **Fold.** Slot 16 makes kv × 528.63 (the ramp at virtual line 540); the line engine forms t = ramp − that, |t|,
  and the row value o + |t| (mod 8 rows) instead of the top clamp.
- **Lean removed**: its two sequencer slots, the per-line u0 / rise / fall corrections and the line latches; the
  beam now starts every line from per-field constants.
- No pipeline stages added or removed: latency 28 in every mode, U/V parity unchanged.

**Build** (router2, `.heirloom_work/diag_build.sh terrace`, 2026-11-09): HD Analog **s2** 87.35 (seed 1 stalled the
router at the timeout), SD Analog s1 85.59, HD HDMI s1 77.66, SD HDMI s1 84.13, HD Dual s1 82.08, SD Dual s1
86.39 MHz. 7,014–7,024 / 7,680 LC (91.4 %, v0.5.1 6,850–6,864), 7 / 32 EBR; iceunpack OK ×6; the packed config
carries the new labels and presets. No pin-skew screen.

**Checks** (working files in `.heirloom_work/rb_terrace/p2/`):
- GHDL analyse / elaborate / synth front end: clean (`check.sh`).
- `sweep.py`: every per-field product and register at RTL widths over all K1 / K2 / K4 values and all 16 timing
  IDs (worst-case Steps and Seam): no overflow (tightest: f_inc 252,500 of 262,143 at 10.9 tiles on SD; f_incf
  986k of 1,048k). Defaults exact: tiles 695, rows 6429, speed 4813 (= v0.5). The ease-home converges from every
  start in ≤ 304 fields.
- Default frame (GHDL 720p60 dec 1, Osc, Photo palette, 2 warm-up + 1 capture; production v0.5.1 simulated the
  same way): raw 10-bit output **byte-identical**.
- One extreme frame (`x_fold_slide.png`: Fold on, K4 full, Osc, 4 warm-up fields; 720p60 decimation 2, so the
  frame shows only ~1.4 tiles and the fold line falls at its bottom edge, both decimation artefacts of the timing-ID
  geometry): the frame shows the mirrored (climbing) half, and the second tile's rows sit 11 lines (0.37 rows) off
  the first's, as the model predicts after 4 fields; the step lands inside the seam fringe, with no sliver of the
  old offset either side. (Two 720p dec-1 attempts were SIGTERMed from outside under load.)
- Not verified: Video mode with a camera (unchanged path), the ease-home on hardware, K1 / K2 extremes in GHDL.

## Reading of the photo
Measured in `.heirloom_work/rb_terrace/` (m1–m8.py, fit_fb.py, shape.py; notes.md).
- **Grid.** The grid is strict. Column edges are vertical to ±1 px over the full height, with no shear or curvature
  anywhere. The seams only seem to wobble (±8 px) because their fringe colours change from row to row.
- **Rows.**
  - Row pitch is 42.98 lines; boundaries sit at y = 54.35 + 42.979k (residual ±0.8, from interlace).
  - The vertical ramp starts at y = 11.37 and is clamped, so the top row is stretched to 54 lines.
  - That gives 25.13 rows per 1080 lines and an 8-row colour period of 343.8 lines.
- **Tiles.** A line-locked sawtooth with a period of 705.95 px (2.7197 per 1920 px). Tile k starts at
  x = −13.76 + k·705.95, and every tile starts on colour index = row.
- **Columns.**
  - There are 6 steps per tile, with boundaries at 84.2 / 244.9 / 339.2 / 439.7 / 597.1 px from the tile start.
  - The widths are therefore 84 / 161 / 94 / 100 / 157 / 87 px: narrow, wide, narrow, narrow, wide, narrow.
  - This comes from a bent rise, h(t) = t + D(t). D is a periodic smooth curve through the five boundaries
    (|D| ≤ 0.044, roughly 0.011 sin 2πt + 0.034 sin 4πt).
- **Seam.**
  - The flyback lasts 21.55 px and falls like an RC discharge: col = floor(6 (1 − fr^0.42)).
  - On the way it sweeps back through steps 5..0, which produces the fringe colours. A white→yellow row shows a dark
    notch, light grey→magenta an orange/yellow sliver, and black→light grey a white/blue sliver.
  - The 1-D fit of the 8 row-phase profiles has an RMS error of 4.9.
- **Colour.** The colour index is (row − column) mod 8. Per-cell means (std < 1), with the greys slightly green:
  - yellow (254,255,5), magenta (255,24,255), cyan (7,255,255), white (255,254,252);
  - light grey (210,211,219), grey (133,145,142), dark grey (51,74,58), black (17,43,19).
- **Edges.** Vertical edges are soft: Gaussian σ is R 4.0 / G 4.7 / B 4.9 px, with small channel shifts (a green
  fringe right of yellow). Row edges are sharp, 2–3 lines.
- **Mechanism.** The patch has four stages:
  - a clamped frame ramp into a quantizer gives the rows;
  - a line-locked sawtooth with a bent rise and an RC flyback, into a second quantizer, gives the columns;
  - row − column drives an 8-entry colourizer;
  - the monitor adds a low-pass.
  The natural motion is a DC drift on the row ramp: the rows roll and the staircase climbs.
- **Fit.** The float prototype matches the photo to a mean |diff| of 3.8 (8-bit RGB).

## What it does
1. **Rows.** v = o + max(0, y − 11.37)·rows/1080, in virtual 1080-line units; o is the drift.
2. **Columns.** With u the tile phase:
   - on the rise (u < 1 − f), ch = g·(u/(1 − f) + β·D(u));
   - on the fall, ch = g·(1 − fr^0.42), where fr = (u − r)/f;
   - Video mode adds gv·luma to ch.
3. **Colour.**
   - index = floor(v) − floor(ch) mod 8, into the palette. Climb uses + instead of −; Ramp uses floor(v ∓ ch).
   - Fill blends the grey half of the palette 50 % with the source.
4. **Output.** The soft edges come from the output conditioning: [1 2 … 2 1]/32 on Y/U/V at 1080, /16 at 720p
   and SD. Then luma is scaled 13/16 and the blanking gate is applied.

## Controls
| Control | Photo element and function |
|---|---|
| K1 Tiles | Tile count: sawtooth cycles across, 1–8 linear (2.72 = the photo) |
| K2 Rows | Row height: rows per frame height, 4–64 linear (25.1) |
| K6 Bend | Narrow/wide column alternation: 0–5× the photo's bend, square law (45 % = 1×). Above about 2.5× the columns fold back into zigzags. In Video mode it also scales how far the camera shifts the column steps |
| K4 Speed | Row drift: bipolar, still within ±8 of centre, square law, up to ±2 rows/s. The default (+39 %) is about 0.29 rows/s: rows rise and the stairs climb |
| K5 Seam | Flyback fringe: 0.4–50 % of a tile, cubic (3.05 % = 21.5 px, the photo). At 50 % the sawtooth becomes a triangle, giving chevron terraces |
| K3 Palette | The 8 colours: Photo (fitted) / Primary / Ember / Ocean / Neon / Pastel / Inverse / Mono |
| S7 Climb | Staircase direction: it climbs to the right |
| S8 Ramp | The stair treads become continuous slopes (diagonal bands, waved by the bend) |
| S9 Lean | The columns shear half a tile over the frame height |
| S10 Fill | The grey blocks take the source picture (camera, or the Osc plasma) at 50 % |
| S11 Source | Video / Osc (default Video) |
| P12 Steps | Column steps per tile, 24·p², glided. 0 = flat colour bars (in Video mode, the picture as displaced bars); 50 % = the photo's 6; 100 % = 24 steep fine diagonals curved by the bend |

## Video
- **Video mode.**
  - The camera's luma, through an 8-px box, is added to the column quantizer's input.
  - It is unipolar: black adds nothing, so Video mode with no camera shows the photo.
  - At default Bend, full white shifts the steps by one. The depth scales as (0.25 + 0.75 Bend) and doubles toward
    Steps = 0, so the picture still reads on flat bars as displaced bars.
  - The staircase therefore bends around the picture's shapes, and its luma contours become step boundaries.
  - S10 Fill also puts the camera's colour into the grey blocks.
- **Osc mode.** The bend comes only from the photo's own second-harmonic bend, which acts as the oscillator, so
  Bend stays live. Fill takes the shell's folded-triangle plasma.

## Engine
- **Per-field sequencer.** One serial 24×16 shift-add multiplier, running 20 slots of 32 clocks in the vblank zone.
  - Knob curves: k1·1792, k2·15375, k3², p², and speed².
  - Geometry: the tile increment from 2^24/width, rows per virtual line, and the clamp offset.
  - Products: g·kr and g·β; the rise/fall DDA increments inc·Gr and inc·kf; their line starts U0·Gr and (r − U0)·kf;
    and the Lean increments.
  - Gains: the video gain (0.25 + 0.75β)·max(1, 2 − g/6) and the drift step.
  - The seam constants r, 1/(1 − f) and 1/f come from a 256-entry ROM read on the glided K5.
- **Per-line engine.** Runs at the source avid fall:
  - the row accumulator (Q.20), clamped with max(v, o);
  - the Lean phase, with its c0/f0 corrections.
- **Pixel path.** Three DDAs (accumulators), so there is no wide multiply:
  - tile phase u (24 bit);
  - rise c = g·u/(1 − f) (Q.18);
  - fall (u − r)/f (Q.18, 9 integer bits); its sign is the rise flag.
  The rise and fall DDAs are corrected at every tile wrap, which is known one step ahead from u + inc. After them:
  - the D ROM (512×9) indexed by u, and the F ROM (512×12) indexed by fr, each with a fabric copy;
  - one 15×13 product (g·β·D or g·F), done as three 6×13 partial products summed over two stages;
  - + c, + the video term, the colour index, the palette ROM (64×30, in LUTs), Fill, and the neutral gate;
  - 16-box pairs (1080) or 8-box pairs (720p/SD, taps 4..11) on Y/U/V, normalise, luma 13/16, blanking gate.
  Latency is 28 clocks in every mode.
- **Contracts.**
  - Generated avid (escalator shell), one latency for all modes, output conditioning before the gate (against the HD
    HDMI bars).
  - The drift steps on vblank-zone edges, so vsync serration can't double it.
  - Glides on K1/K2/K3/K5/P12. K6 reads a value < 8 as the firmware's label index.
  - Colours in the hardware convention (U = Cr), generated by `terrace.py`.
- **Palette levels.** Photo luma is unchanged up to 600 and compressed above it as 600 + (y − 600)·175/338, with
  RGB scaled along. It is then pre-boosted 16/13 for the output's 13/16 scale.
  - Output: white 776, yellow 727, light grey 701, cyan 645, grey 547, magenta 474, dark grey 289, black 175.
  - The greys, black and magenta are exactly the photo's. The maximum over all eight palettes is 777, the HD HDMI
    bar-fix level.

## Status
- **v0.5.1 built 2026-11-01: output-filter resync fix, NOT flashed.** The output running-sum filters now take neutral input off the generated avid and snap back to neutral once the box is full of blank samples, so a glitched block-RAM / clock sample can no longer leave a permanent wrapped colour offset (fern_diag hardware capture 2026-11-01). Not touched: the source-side bend-tap running sum (v_box), which steers geometry, not colour. Post-route Fmax: HD Analog s1 83.31, SD Analog s1 83.87, HD HDMI s1 86.01, SD HDMI s1 91.35, HD Dual s1 88.51, SD Dual s1 89.57 MHz; 6850-6864 / 7680 LC. Earlier hand-installed HD seeds are gone (new netlist).
- **v0.5.0, 2026-10-21, router2.** Post-route Fmax:
  - HD Analog 85.19 MHz (seed 1), HD HDMI 84.37 MHz (**seed 2**: seed 1 stalled the router at the 900 s timeout),
    HD Dual 82.68 MHz (seed 1);
  - SD Analog 92.28, SD HDMI 89.13, SD Dual 89.39 MHz (all seed 1).
- **Resources.** 6839–6850 of 7680 LC (89 %), 7 of 32 EBR (bend 2, flyback 2, seam 3).
- **Artefacts.** All six bitstreams pass iceunpack. `out/rev_b/ron/heirloom/terrace.vmprog` is 441,749 bytes.
- **Verified in GHDL, not on hardware.**
  - The 720p60 render matches the integer model `imodel.py` (mean |diff| 0.00 after the 4-px generated-avid offset).
  - The switch paths match the model: Ramp/Climb/Ember to a mean of 0.31, Lean/Fill in structure.
  - Motion: at maximum Speed the rows rise 8 lines in 6 fields (the model predicts 8.3).
  - 720p and NTSC render the same picture.
- **Not flashed.**

## Deviations from the photo
- **Highlights are compressed.** White lands at 776 instead of 938, yellow at 727 instead of 841, and light grey at
  701 instead of 791. This keeps drawn luma at the HW-proven bar-fix level; the picture's moving full-swing block
  edges are the bar trigger. The order and the greys hold, but the white/light-grey contrast is smaller (75 instead
  of 147).
- **Not reproduced:** the per-channel RGB fringe offsets (the green fringe right of yellow) and the 2-px capture
  border.
- **Edge softness differs by resolution.** At 720p and SD the 8-box pair gives about 3.5 px (720p) and 6.3 px (SD)
  in 1920 terms, against the photo's 4.4 px.
- **The bend is tied to the tile phase.** It was fitted at the photo's seam width, so at other K5 settings it is
  slightly off near the seam.
- **The top edge steps while drifting.** Once per row period the clamped top 11 lines switch to the next colour,
  which is what a clamped ramp does. The photo's 54-line top row is the moment just after.

## Working files
`.heirloom_work/rb_terrace/`:
- `ref.jpg` and `notes.md`;
- `m*.py`, `fit_fb.py`, `shape.py`: the measurements and fits;
- `proto.py` → `proto_final.png`, `cmp_proto.png`: the float prototype;
- `ctl.py`, `sheet.py` → `sheet.png`: the control sheet;
- `gen_tables.py`, `mkhook.py`: the tables and the build hook;
- `imodel.py`: the integer model;
- `rdiff.py`: RTL vs model diffs;
- `simx.sh`: the GHDL sim wrapper;
- `cmp_final.png`, `cmp_motion.png`, `cmp_swb.png`: the comparisons.

## Open questions for Ron (2026-11-09 batch)
1. Lean (the columns shearing half a tile over the frame) was replaced by Fold. Bring Lean back, e.g. as Fold's
   partner on another switch (Climb or Fill would have to give way)?
2. The tile slide centres on the middle tile (outer tiles fastest, in opposite directions). Would you rather the left
   tile stayed put and the slide fanned out to the right, or neighbouring tiles simply counter-scroll?
3. When K4 comes back from the slide zone, the tiles glide home in about 3 s (so the photo is always reachable). Or
   should a slid stagger stay where you left it until you slide it again?
4. A "Pitch" control (rows dropped per column step: flat .. the photo's 1 .. 4, negative = climbing) was the strongest
   idea that didn't fit in the 12 controls. Worth trading a knob for it?

## Hardware watch list
- Renamed / moved: S9 Lean → **Fold**; K1 Tiles and K2 Rows now read 0–100 % with the photo at **50 %** (v0.5:
  Tiles 251, Rows 360 = the same picture). Presets re-aimed: Chevrons → Diamonds, Cascade dropped.
- K4 past about 69 % (or below about 31 %) slides the tiles; at the ends it is deliberately frantic (5.6 rows/s
  between neighbours). After a slide the tiles take ~3 s to glide back into line.
- K2 near max (100 rows, 10.8 lines a row): about 2.4 real lines a row on 480i, so the finest rows can shimmer on SD.
- K1 near max (10.9 tiles, 6 columns each): 30 px columns at 1080, 11 px on SD.
- Cut: Lean (replaced, not for fit).
- HD HDMI 77.66 MHz (thinner than v0.5's 86.0). No pin-skew screen; if HD HDMI shows lines, re-seed that config.

## Next step
Wait for Ron's preview notes. Rebuild the diag with `.heirloom_work/diag_build.sh terrace`. On approval:
`./heirloom_diag.sh merge terrace`, then repack the previewed diag bitstreams under the production TOML (copy
`build/programs/ron/terrace_diag/rev_b/bitstreams/*.bin` into the production build dir, regenerate
`program_config.bin` from the production TOML, `vmprog_pack.py --no-sign --hardware rev_b`) instead of re-routing;
update the production SUMMARY / README / PHASE2 row and commit only terrace's files.

## K6 camera pass (2026-11-15)

Ron: K6 is the camera strength in every program, default 50 %. Here the camera strength was on K3 (Bend), so K3 and K6 swapped: **K6 = Bend** (the camera strength, default now 512 = 50 %), **K3 = Palette** (was K6). Nothing else changed: the VHDL swaps the two knob inputs (the processing is identical), the TOML blocks and preset keys moved with them (the Controls rows above are renumbered). v0.6.1, rebuilt with router1: all six seed 1; HD worst 84.38 MHz (HD Analog 92.64, HD HDMI 84.38, HD Dual 89.45), LC LC. Rebuild: `NEXTPNR_ROUTER="--router router1" ./heirloom_diag.sh build terrace`. The pre-pass vmprog is in out/rev_b/ron/_pre_k6/.

## K6 top +25 % and default 100 % (2026-11-15)

Ron: K6 now loads at 100 % (initial_value 1023), and in bend / element programs the camera effect at the top of the knob is 25 % stronger.
- **Classification.** Bend (camera part only): K6 Bend sets both the column bend (not camera-driven) and the depth of the camera's step shift.
- **Change.** The camera gain constant (slot 20) 1197 became 1496, so f_gvv is x 1.25 at every K6 position (top 9548 -> 11933 of 14 bits; the source sum fits its 22 bits). The column bend and everything else K6 drives are unchanged.
- No render for the diag copy (the change is identical to the production program's).
- **Build.** v0.6.2, router1, all six configs on seed 1. HD worst 83.89 MHz (HD Analog 86.58, HD HDMI 83.89, HD Dual 89.45); SD 78.97–97.31 MHz; 7025 / 7680 LC (91 %), 7 EBR. vmprog: out/rev_b/ron/_diag/terrace_diag_v0.6.2.vmprog.
