# Terrace — a colour-bar staircase through two quantizers

**v0.5.0 (2026-10-21): rebuilt from the photo, built, NOT flashed.** Photo: `photos/WIN_20230305_23_38_39_Pro.jpg`.
It is a strict grid of flat blocks: 2.72 tiles of 6 columns by 25 rows. Down every column the colours run yellow,
magenta, cyan, white, light grey, grey, dark grey, black. Each column is the same sequence dropped one row, so the
colours step down to the right like stairs. At every tile seam a narrow fringe flashes the in-between colours.
Presets are hand-written placeholders.

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
| K3 Bend | Narrow/wide column alternation: 0–5× the photo's bend, square law (45 % = 1×). Above about 2.5× the columns fold back into zigzags. In Video mode it also scales how far the camera shifts the column steps |
| K4 Speed | Row drift: bipolar, still within ±8 of centre, square law, up to ±2 rows/s. The default (+39 %) is about 0.29 rows/s: rows rise and the stairs climb |
| K5 Seam | Flyback fringe: 0.4–50 % of a tile, cubic (3.05 % = 21.5 px, the photo). At 50 % the sawtooth becomes a triangle, giving chevron terraces |
| K6 Palette | The 8 colours: Photo (fitted) / Primary / Ember / Ocean / Neon / Pastel / Inverse / Mono |
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
