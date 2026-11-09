# Yolk — red eggs with black yolks rising between white flame and lens-ring bands

**v0.1.0 built 2026-11-09, not flashed.** Heirloom (video phase), from the clip `WIN_20240111_01_01_24` at 0.5 s
(`photos/vid/WIN_20240111_01_01_24_t0p5.png`). Presets are hand-written placeholders.

## Reading of the photo
Working files are in `.heirloom_work/rb_yolk/` (`notes.md`, `m1.py` / `m2.py` / `prof.py` measurements, `fit*.py`,
`proto.py`, `imodel.py`).
- **Egg row.** Four red eggs with black yolks, centres x 320 / 726 / 1240 / 1663. The spacing 406 / 514 / 423 is one
  H period of ~425 px **mirrored about x 993**: the centre pair sits farther apart, with a yellow hourglass between.
  The eggs are rounded diamonds (G-dark region 364 / 379 / 379 / 361 px wide at y 350, y ~160..700). The yolks are
  vertical slots in the R channel, 67 / 93 / 127 / 157 px wide: bigger to the right, where the yellow under the
  eggs also turns green.
- **An RGB colouriser.** chR holds the slot yolks, chG the diamonds and every ring line (the magenta lines are G
  dips), chB a static horizontal band.
- **Static white bands.** B is zero for y 264..770 in every frame of the clip. Inside the window (y < 264,
  y >= 770) a keyer switches to a white layer wherever the field is above a level. So the top band's lower edge
  is scalloped (egg domes up to y ~155, white V-notches down to ~250) and the bottom band starts at a hard edge
  (y 770).
- **Bottom rings.** Magenta lens rings around (530, 950) and (1470, 950), with a ")(" at 993. The inner ring is
  72 x 168 px, the second has half-width 122 and its top at y 808, a third is cut by the band edge. They are
  pointed top and bottom: a triangle in x plus a quadratic vertical maximum.
- **Top flames.** Steep (~70°) U-lines, the lower halves of rings centred above the frame: the vertical wave is
  flat in the top band.
- **Field.** Hand-fitted to measured landmarks: G-diamond widths per row, R-core widths, ring crossings, key edges,
  line slopes. Free separable and single-field fits were tried and dropped (`notes.md`).
  - F_G = T(x) + VG(y), and F_R = (T + VR(y) − 0.4)·(1 + 0.5 (x − 960)/960) + 0.4.
  - VG: a quadratic max at y 950 (deficits 0.17 / 0.625 / 0.95 at 74 / 142 / 180 lines above it), a steep egg
    top, a rounded min at y 345.
  - VR: a shallower bowl with a flat bottom (the slot yolks).
  - Key level 0.55; white-layer line period 0.40 (T units).
- **Palette** (RGB): black yolk (4, 2, 0) → red (250, 14, 0) → orange → yellow (250, 240, 16) → green (70, 215, 18)
  with dark-green fold lines. The white layer is (252, 246, 240) with magenta (255, 20, 232) lines, a red edge and
  a broad yellow side.
- **Textures.** Short horizontal dashes along every key edge (noise on the keyer input); the left 90 px torn into
  horizontal colour bars.
- **Colour shares** (960 x 540), still → integer model at the defaults:
  - top band: white 31 → 25 %, red/orange 25 → 20 %, yellow 14 → 20 %, magenta 10 → 14 %;
  - egg row: yellow 29 → 27 %, red 22 → 31 %, black 14 → 13 %, green 5 → 9 %;
  - bottom band: white 43 → 31 %, yellow 20 → 24 %, magenta 15 → 24 %.
  Mean |dRGB| on a 64 x 36 grid: proto 35.9, model 38.5.

## Motion in the clip
- The egg rows **rise**. Between 0 and 3 s the black row's centre goes 500 → 360 → 300 → 248 → ~220 (pause) → 168
  → 48; the next row enters at the bottom (970 at 3.2 s) and climbs to 220 by 8 s. That is one row every ~6.5 s,
  ~145-160 lines/s on average, uneven (it lingers near y ~200). In the m-frames the egg-2 yolk's top goes
  224 → 94 → 10 → 0 per 0.5 s.
- **The white bands stay put** (the window is static). The eggs rise into the top band, and the next row of bulbs
  comes up through the heat layer (green bulbs with dark-green rings). The bottom rings drift up more slowly than
  the eggs (950 → ~830 in 1.5 s), and the bodies turn green near the top (the R mix lags the G mix).
- The program rolls both vertical waves up at **160 lines/s** by default (K4 Rise = 680), one row per 6.6 s, with
  the white bands static. The dashes and bars are redrawn every field. RTL check: a capture 3 fields later moved
  5 rows up at 720p (5.3 expected).

## What it does
1. **H oscillator.** A beam DDA gives acc = (x − 993.5)·KH; |acc| is the mirrored phase. T is its triangle (0 on
   the egg columns), with a half-depth notch at the centre, held at its x = 90 value at the line start.
2. **V waves.** Per line, psi = y·KV + scroll indexes a 512-entry ROM holding VG and VR (Q10).
3. **Line terms.** P12 Rings gives gm (x0.45 .. x4). The line engine forms (VG − 0.55)·gm + 0.55 − yolk·gm and
   (VR − 0.4)·gm − yolk·gm.
4. **Fields.** F_G = T·gm + line term + camera + noise. F_R = (T·gm + line term + camera + noise/4)·lean + 0.4,
   with the lean in 16 steps (x0.5 .. x1.5 across the frame).
5. **Key.** In the static window (ragged ±12 lines by the noise, its edges on line pairs) and F_G > 0.55: the white
   layer.
6. **Colour.** Indexes (F·64 + 192) fold carry-free into a period-1024 triangle and read the R (2048 x 8) and G/B
   (2048 x 16) palette ROMs. The white layer reads all three from F_G, with vertical yellow strips where T is
   0.66..0.80. Invert is folded into the index adders. A white-balanced RGB → Y / Cr / Cb matrix follows.
7. **Textures.** Noise runs of 8..39 virtual px (an xorshift per pixel, seeded per 4 lines, bursts x4) push F:
   fully inside the window, ~1/3 outside, 1/4 on R. Left of x = 90 the field is the held T plus per-band bar
   offsets (6-37 virtual lines, changing on line pairs).

## Controls
| Control | Photo element it drives |
|---|---|
| K1 Eggs | the eggs across: x0.33 (1.5 eggs, two giant lobes) .. the still's 4.5 (512) .. x3 (13 thin eggs); exponential |
| K2 Rows | the vertical period: x0.31 (tall egg columns, stretched rings) .. the still (512) .. x3.25 (three rows of small eggs between bands of rings) |
| K3 Yolk | the yolks: shrunk to red seeds in a yellow-green field (0) .. the still (512) .. swollen black yolks pushing into the bands (1023) |
| K4 Rise | bipolar (+-8 deadband, square law): the egg rows rise (+) or sink (−); 680 = the clip's 160 lines/s; full ≈ 1600 lines/s |
| K5 Bands | the white bands: none, the heat layer everywhere (0) .. the still (512: y < 264 and y >= 770) .. nearly the whole frame (1023) |
| K6 Etch | how far the camera bends the field: 0 .. 1.4 T-units per full luma (square law); 380 = a gentle bend; the Osc pushes 6x as far |
| S7 Mirror | Off (the still) / On: the frame folded about its centre line, the flames reflected at the bottom, two rows of eggs facing |
| S8 Invert | Off (the still) / On: the field inverted; the ring centres become black yolks, the eggs become ringed yellow bulbs |
| S9 Tear | Dash (the still's dashes) / Slip: bands of 16-79 virtual lines slide sideways up to +-254 px, the dashes doubled |
| S10 Colours | Yolk (the still) / Lagoon (the sibling clip 00_56_40: red ground, green / cyan / blue rings, pale highs) |
| S11 Source | Video / Osc |
| P12 Rings | the colouriser gain x0.45 .. x4 (390 = the still): 0 % = soft amber eggs with few lines; 100 % = every egg a black-and-red target, dense pinstripe rings in the white bands |

Glides (1/8 of the gap per field) on K1, K2, K3, K5, K6 and P12.

## Video
- **Video mode (the default).** Camera luma, (Y >> 2) − 16 from a one-line buffer, is box-filtered over 4 px and
  ADDED to both fields.
  - It is unipolar, so a black input is exactly the still.
  - Bright parts of the picture shrink the yolks and push the ring lines around them.
  - At the default Etch the eggs and bands read clearly on `selfie.jpg` (`cmp_final.png`, bottom).
- **Osc mode.** The drifting 640 x 540 virtual-px triangle lattice, centred on its mean, feeds the same path at 6x
  the camera's gain. The eggs pinch, lean and merge, and the rings warp, as the lattice drifts.

## Engine
- **Per-field sequencer** (vblank zone). One serial shift-add multiplier runs 17 slots, each decoded two clocks
  ahead with pre-muxed operands; a shared glide; a 256-entry exp2 ROM. Per field it computes:
  - KH (eggs), KV (rows) and gm (Rings);
  - the beam start and step, and the line-0 phase;
  - the scroll step (Rise²) and yolk·gm;
  - the Invert index offsets and key constant;
  - the etch gain (7-bit mantissa << shift; Osc x6), the window rows, and the noise and slip seeds.
- **Line engine.** Kicked at each line start, it computes the NEXT line in ~75 clocks (it fits SD's 858):
  - the Mirror fold, psi = vy·KV + ps and the V ROM read;
  - the two gained line terms and the slip shift x KH;
  - the window distances (per line pair) and the noise seed (per 4 lines);
  - the bar and slip bands (counted in virtual lines, changing on even lines).
- **Beam.** The H accumulator, the 16-step lean level, the hold flag and the noise runs.
- **Pixel path (20 stages).**
  1. |acc|.
  2. Phase, notch, window and the luma box.
  3. T and the strip flag.
  4. T·gm partial products.
  5. The product, etch and noise terms.
  6. E and the T term.
  7. Line terms.
  8. The fields.
  9. Key, G index and lean partials.
  10. G fold and F_R.
  11. R index and fold, and the ROM addresses.
  12. The ROMs, then a fabric copy.
  13. Strips and the matrix terms.
  14. Matrix sums, black off-avid.
  15. The box: running 8-sums, then the 9-tap.
  16. / 16.
  17. Luma 13/16.
  18. The gate.
- **Memory.** ROMs: V 512 x 24, R 2048 x 8, G/B 2048 x 16, exp2 256 x 16. Line buffer 2048 x 8. The box ring is
  256 x 30 and the sync ring 256 x 3 (both `no_rw_check`). 23 EBR in all.
- **Contracts.**
  - Generated avid; one latency (C_LAT 20) in every mode.
  - Virtual 1080-line coordinates (SD shows the centre 4:3).
  - Colours in the hardware convention: the matrix writes Cr to U.
  - Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box whose running sums snap back on blank
    samples; luma 13/16; the blanking gate.
- **Checks.**
  - GHDL analysis / elaboration / synth front end clean (with the package), and the TOML pre-check clean.
  - `imodel.py` vs GHDL, 720p60 dec 1, black input: 99.3 % of pixels bit-exact, mean |d| 1.44 codes.
  - `thincheck.py` (single-row runs >= 24 px, fields 1 / 47 / 133): 0 at the defaults, P12 0 and P12 100, Slip,
    Invert, Mirror and Eggs max.
  - dupin: 0 `CARRY I0==I1`.
- **Fit history.** The first RTL packed to 9429 LC. The second (single box with block-RAM rings, low-precision
  matrix, carry-free fold, 16-step lean, Invert in the index adders) reached 7722 in the full build. The shipped
  version (one Rings gain for both layers, C_LAT 20, shared sequencer scratch, mantissa/shift etch gain, simpler
  hold zone) is at 6871-6878.

## Status
- **v0.1.0 (2026-11-09), router2 (the builder's default). Build seeds: HD Analog = seed 2** (seed 1 missed), **HD HDMI
  = seed 2** (seed 1 stalled the router); the other four are seed 1.
- Post-route Fmax:
  - HD: Analog 80.57, HDMI 77.49, Dual 76.52 MHz.
  - SD: Analog 73.24, HDMI 73.75, Dual 77.59 MHz (SD Fmin 27).
- 6871-6878 / 7680 LC (89.5 %), 23 / 32 EBR. Every bitstream passed iceunpack.
- Rebuild: `MAKE_TIMEOUT=900 .heirloom_work/vid/build_queue.sh yolk`. If the HD margins tighten, try
  `NEXTPNR_ROUTER="--router router1"`.
- Output: `out/rev_b/ron/heirloom/yolk.vmprog`. **Not flashed.**

## Deviations
- **Rigid roll.** The clip's sweep is slightly non-linear (the bottom rings rise more slowly than the eggs, and the
  rows linger near y 200), and its R mix lags the G mix, so bodies turn green near the top. The program rolls both
  waves together at one speed; K3 Yolk reaches the green-body look by hand.
- **Hand-fitted shapes.** The still's eggs are a little more diamond-like, its left eggs' cores narrower low down,
  and its top-band flames more chaotic.
- **Brightness.** Whites come out at ~777 output luma (the shell's cap), so the white bands read light grey-white
  next to the capture's clipped white; yellows are 13/16 as bright. The RGB → YUV matrix uses short coefficients
  (white-balanced, chroma within a few %).
- **One gain for both layers** (cut for budget). At P12 100 the white bands become fine magenta pinstripes rather
  than bold lines; the eggs carry the climax.
- **Left-edge bars.** Their colours come from the held T plus random band offsets, not the rows beside them.
- **Simplified detail.** The dark-green arcs on the bottom rings' tops are simplified, and the dashes are 8-39
  virtual px (the still's run to ~60).
- **Osc gain.** The Osc is at 6x the camera gain, above the usual 2-4x, so the flip is unmistakable.
- **K2 Rows near max** (x3) can leave a few 25-35 px one-row contour slivers in the centre notch. The defaults and
  P12's ends are clean.
- **Resolutions.** 720p shows the same picture as 1080p. SD shows the centre 4:3 (x 240..1680 virtual): four eggs
  with the outer two cut, and the left-edge bars fall outside the picture. 1080i sources render both fields alike.
